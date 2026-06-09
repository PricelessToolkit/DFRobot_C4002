#include "DFRobot_C4002.h"

namespace {
const uint8_t FRAME_HEADER[] = {0xFA, 0xF5, 0xAA, 0xA5};

const uint8_t FRAME_TYPE_WRITE_REQUEST = 0x00;
const uint8_t FRAME_TYPE_READ_REQUEST = 0x01;
const uint8_t FRAME_TYPE_WRITE_RESPONSE = 0x02;
const uint8_t FRAME_TYPE_READ_RESPONSE = 0x03;
const uint8_t FRAME_TYPE_NOTIFICATION = 0x04;

const uint8_t CMD_RESTART = 0x00;
const uint8_t CMD_FACTORY_RESET_USER = 0x02;
const uint8_t CMD_SET_BAUD_RATE = 0x21;
const uint8_t CMD_ENVIRONMENT_CALIBRATION = 0x60;
const uint8_t CMD_SET_DISTANCE_GATE = 0x62;
const uint8_t CMD_SET_DISTANCE_GATE_THRESHOLDS = 0x63;
const uint8_t CMD_SET_OUTPUT_MODE = 0xA0;
const uint8_t CMD_SET_LED_MODE = 0xA1;
const uint8_t CMD_SET_RESOLUTION_MODE = 0x66;
const uint8_t CMD_FACTORY_RESET = 0x80;
const uint8_t CMD_GET_VERSION = 0x82;
const uint8_t CMD_SET_REPORT_PERIOD = 0x83;
const uint8_t CMD_TARGET_DISAPPEAR_DELAY = 0x84;
const uint8_t CMD_SET_DETECTION_RANGE = 0x86;
const uint8_t CMD_SET_SENSITIVITY_PRESET = 0x87;
const uint8_t CMD_SET_LIGHT_THRESHOLD = 0x88;

const uint8_t NOTE_PRESENCE_RESULT = 0x60;
const uint8_t NOTE_ENVIRONMENT_CALIBRATION = 0x03;
}

DFRobot_C4002::DFRobot_C4002(Stream &serial) : serial_(&serial) {}

bool DFRobot_C4002::begin(uint8_t outputPin, ResolutionMode mode, uint8_t reportPeriod) {
  outputPin_ = outputPin;
  if (outputPin_ != 255) {
    pinMode(outputPin_, INPUT);
    digitalWrite(outputPin_, HIGH);
  }

  return begin(mode, reportPeriod);
}

bool DFRobot_C4002::begin(ResolutionMode mode, uint8_t reportPeriod) {
  resolutionMode_ = mode;

  if (!setReportPeriod(255)) {
    return false;
  }
  delay(10);

  if (!setResolutionMode(mode)) {
    return false;
  }

  if (!enableAllDistanceGates(true)) {
    return false;
  }

  getOutputMode(outputMode_);
  return setReportPeriod(reportPeriod);
}

DFRobot_C4002::UpdateEvent DFRobot_C4002::update(uint32_t timeoutMs) {
  Packet packet = receivePacket(timeoutMs);
  if (packet.response != SUCCEED || packet.type != FRAME_TYPE_NOTIFICATION) {
    return NO_EVENT;
  }

  if (packet.command == NOTE_PRESENCE_RESULT) {
    parsePresenceData(packet.data, packet.dataLen);
    return PRESENCE_DATA;
  }

  if (packet.command == NOTE_ENVIRONMENT_CALIBRATION && packet.dataLen >= 2) {
    calibrationCountdownSeconds_ = readU16LE(packet.data);
    return CALIBRATION_DATA;
  }

  return NO_EVENT;
}

bool DFRobot_C4002::setReportPeriod(uint8_t period) {
  uint8_t payload[5] = {CMD_SET_REPORT_PERIOD, READ_AND_WRITE_REQ, 5, 0, period};
  Packet reply;
  return command(CMD_SET_REPORT_PERIOD, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::setSerialBaudRate(BaudRate baudRate) {
  uint8_t payload[8] = {CMD_SET_BAUD_RATE, READ_AND_WRITE_REQ, 8, 0};
  uint32_t value = static_cast<uint32_t>(baudRate);
  payload[4] = value & 0xFF;
  payload[5] = (value >> 8) & 0xFF;
  payload[6] = (value >> 16) & 0xFF;
  payload[7] = (value >> 24) & 0xFF;

  Packet reply;
  return command(CMD_SET_BAUD_RATE, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::getVersionInfo(VersionType type, String &version) {
  uint8_t payload[5] = {CMD_GET_VERSION, READ_AND_WRITE_REQ, 5, 0, static_cast<uint8_t>(type)};
  Packet reply;
  if (!command(CMD_GET_VERSION, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply)) {
    return false;
  }

  version = "";
  for (uint16_t i = 0; i < reply.dataLen; i++) {
    version += static_cast<char>(reply.data[i]);
  }
  return true;
}

bool DFRobot_C4002::setResolutionMode(ResolutionMode mode) {
  uint8_t payload[5] = {CMD_SET_RESOLUTION_MODE, READ_AND_WRITE_REQ, 5, 0, static_cast<uint8_t>(mode)};
  Packet reply;
  if (!command(CMD_SET_RESOLUTION_MODE, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply)) {
    return false;
  }
  resolutionMode_ = mode;
  return true;
}

bool DFRobot_C4002::getResolutionMode(ResolutionMode &mode) {
  uint8_t payload[4] = {CMD_SET_RESOLUTION_MODE, READ_AND_WRITE_REQ, 4, 0};
  Packet reply;
  if (!command(CMD_SET_RESOLUTION_MODE, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply) || reply.dataLen < 1) {
    return false;
  }
  mode = static_cast<ResolutionMode>(reply.data[0]);
  resolutionMode_ = mode;
  return true;
}

bool DFRobot_C4002::setDetectionRangeCm(uint16_t closestCm, uint16_t farthestCm) {
  if (farthestCm > 1200) {
    farthestCm = 1200;
  }
  if (closestCm > farthestCm) {
    lastResponse_ = PARAMS_ERR;
    return false;
  }

  uint8_t payload[8] = {CMD_SET_DETECTION_RANGE, READ_AND_WRITE_REQ, 8, 0};
  writeU16LE(&payload[4], closestCm);
  writeU16LE(&payload[6], farthestCm);

  Packet reply;
  return command(CMD_SET_DETECTION_RANGE, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::setDetectionRangeMeters(float closestMeters, float farthestMeters) {
  if (closestMeters < 0.0f || farthestMeters < 0.0f) {
    lastResponse_ = PARAMS_ERR;
    return false;
  }
  return setDetectionRangeCm(static_cast<uint16_t>(closestMeters * 100.0f),
                             static_cast<uint16_t>(farthestMeters * 100.0f));
}

bool DFRobot_C4002::getDetectionRangeCm(uint16_t &closestCm, uint16_t &farthestCm) {
  uint8_t payload[4] = {CMD_SET_DETECTION_RANGE, READ_AND_WRITE_REQ, 4, 0};
  Packet reply;
  if (!command(CMD_SET_DETECTION_RANGE, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply) || reply.dataLen < 4) {
    return false;
  }

  closestCm = readU16LE(&reply.data[0]);
  farthestCm = readU16LE(&reply.data[2]);
  return true;
}

bool DFRobot_C4002::getDetectionRangeMeters(float &closestMeters, float &farthestMeters) {
  uint16_t closestCm = 0;
  uint16_t farthestCm = 0;
  if (!getDetectionRangeCm(closestCm, farthestCm)) {
    return false;
  }

  closestMeters = closestCm * 0.01f;
  farthestMeters = farthestCm * 0.01f;
  return true;
}

bool DFRobot_C4002::setLightThresholdLux(float thresholdLux) {
  if (thresholdLux < 0.0f) {
    lastResponse_ = PARAMS_ERR;
    return false;
  }

  uint8_t payload[6] = {CMD_SET_LIGHT_THRESHOLD, READ_AND_WRITE_REQ, 6, 0};
  writeU16LE(&payload[4], static_cast<uint16_t>(thresholdLux * 10.0f));

  Packet reply;
  return command(CMD_SET_LIGHT_THRESHOLD, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::getLightThresholdLux(float &thresholdLux) {
  uint8_t payload[4] = {CMD_SET_LIGHT_THRESHOLD, READ_AND_WRITE_REQ, 4, 0};
  Packet reply;
  if (!command(CMD_SET_LIGHT_THRESHOLD, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply) || reply.dataLen < 2) {
    return false;
  }

  thresholdLux = readU16LE(reply.data) * 0.1f;
  return true;
}

bool DFRobot_C4002::setTargetDisappearDelay(uint16_t seconds) {
  uint8_t payload[6] = {CMD_TARGET_DISAPPEAR_DELAY, READ_AND_WRITE_REQ, 6, 0};
  writeU16LE(&payload[4], seconds);

  Packet reply;
  return command(CMD_TARGET_DISAPPEAR_DELAY, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::getTargetDisappearDelay(uint16_t &seconds) {
  uint8_t payload[4] = {CMD_TARGET_DISAPPEAR_DELAY, READ_AND_WRITE_REQ, 4, 0};
  Packet reply;
  if (!command(CMD_TARGET_DISAPPEAR_DELAY, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply) || reply.dataLen < 2) {
    return false;
  }

  seconds = readU16LE(reply.data);
  return true;
}

bool DFRobot_C4002::setOutputMode(OutputMode mode) {
  uint8_t payload[5] = {CMD_SET_OUTPUT_MODE, READ_AND_WRITE_REQ, 5, 0, static_cast<uint8_t>(mode)};
  Packet reply;
  if (!command(CMD_SET_OUTPUT_MODE, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply)) {
    return false;
  }
  outputMode_ = mode;
  return true;
}

bool DFRobot_C4002::getOutputMode(OutputMode &mode) {
  uint8_t payload[4] = {CMD_SET_OUTPUT_MODE, READ_AND_WRITE_REQ, 4, 0};
  Packet reply;
  if (!command(CMD_SET_OUTPUT_MODE, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply) || reply.dataLen < 1) {
    return false;
  }
  mode = static_cast<OutputMode>(reply.data[0]);
  outputMode_ = mode;
  return true;
}

bool DFRobot_C4002::setRunLed(bool enabled) {
  uint8_t payload[6] = {CMD_SET_LED_MODE, READ_AND_WRITE_REQ, 6, 0,
                        static_cast<uint8_t>(enabled ? LED_ON : LED_OFF), static_cast<uint8_t>(LED_KEEP)};
  Packet reply;
  return command(CMD_SET_LED_MODE, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::setOutputLed(bool enabled) {
  uint8_t payload[6] = {CMD_SET_LED_MODE, READ_AND_WRITE_REQ, 6, 0,
                        static_cast<uint8_t>(LED_KEEP), static_cast<uint8_t>(enabled ? LED_ON : LED_OFF)};
  Packet reply;
  return command(CMD_SET_LED_MODE, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::getLedStatus(LedStatus &status) {
  uint8_t payload[4] = {CMD_SET_LED_MODE, READ_AND_WRITE_REQ, 4, 0};
  Packet reply;
  if (!command(CMD_SET_LED_MODE, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply) || reply.dataLen < 2) {
    return false;
  }

  status.runLed = static_cast<LedMode>(reply.data[0]);
  status.outputLed = static_cast<LedMode>(reply.data[1]);
  return true;
}

DFRobot_C4002::TargetState DFRobot_C4002::outputPinTargetState() const {
  if (outputPin_ == 255) {
    return TARGET_ERROR;
  }

  bool active = digitalRead(outputPin_) == HIGH;
  if (outputMode_ == OUTPUT_ON_MOTION) {
    return active ? MOVING_TARGET : STATIONARY_TARGET_OR_NO_TARGET;
  }
  if (outputMode_ == OUTPUT_ON_PRESENCE) {
    return active ? STATIONARY_TARGET : MOVING_TARGET_OR_NO_TARGET;
  }
  if (outputMode_ == OUTPUT_ON_MOTION_OR_PRESENCE) {
    return active ? MOVING_OR_STATIONARY_TARGET : NO_TARGET;
  }
  return TARGET_ERROR;
}

bool DFRobot_C4002::enableDistanceGates(DistanceGateType type, const uint8_t *gateMask, uint8_t gates) {
  if (gateMask == nullptr || gates == 0 || gates > MAX_GATE_COUNT) {
    lastResponse_ = PARAMS_ERR;
    return false;
  }

  uint8_t payload[5 + MAX_GATE_COUNT] = {};
  uint16_t payloadLen = 5 + gates;
  payload[0] = CMD_SET_DISTANCE_GATE;
  payload[1] = READ_AND_WRITE_REQ;
  writeU16LE(&payload[2], payloadLen);
  payload[4] = static_cast<uint8_t>(type);
  memcpy(&payload[5], gateMask, gates);

  Packet reply;
  return command(CMD_SET_DISTANCE_GATE, FRAME_TYPE_WRITE_REQUEST, payload, payloadLen, reply);
}

bool DFRobot_C4002::enableAllDistanceGates(bool enabled) {
  uint8_t gates[MAX_GATE_COUNT] = {};
  memset(gates, enabled ? 1 : 0, sizeof(gates));
  uint8_t count = gateCount();

  if (!enableDistanceGates(MOVING_TARGET_GATES, gates, count)) {
    return false;
  }
  return enableDistanceGates(STATIONARY_TARGET_GATES, gates, count);
}

bool DFRobot_C4002::setDistanceGateThresholds(DistanceGateType type, const uint8_t *thresholds,
                                              uint8_t thresholdCount) {
  uint8_t count = gateCount();
  if (thresholds == nullptr || thresholdCount < count || count > MAX_GATE_COUNT) {
    lastResponse_ = PARAMS_ERR;
    return false;
  }

  uint8_t payload[7 + MAX_GATE_COUNT] = {};
  uint16_t payloadLen = 7 + count;
  payload[0] = CMD_SET_DISTANCE_GATE_THRESHOLDS;
  payload[1] = READ_AND_WRITE_REQ;
  writeU16LE(&payload[2], payloadLen);
  payload[4] = static_cast<uint8_t>(type);
  payload[5] = CUSTOM_SENSITIVITY;
  payload[6] = 0x01;
  memcpy(&payload[7], thresholds, count);

  Packet reply;
  return command(CMD_SET_DISTANCE_GATE_THRESHOLDS, FRAME_TYPE_WRITE_REQUEST, payload, payloadLen, reply);
}

bool DFRobot_C4002::getDistanceGateThresholds(DistanceGateType type, uint8_t *thresholds,
                                              uint8_t thresholdCount) {
  uint8_t count = gateCount();
  if (thresholds == nullptr || thresholdCount < count || count > MAX_GATE_COUNT) {
    lastResponse_ = PARAMS_ERR;
    return false;
  }

  uint8_t payload[7] = {CMD_SET_DISTANCE_GATE_THRESHOLDS, READ_AND_WRITE_REQ, 7, 0,
                        static_cast<uint8_t>(type), static_cast<uint8_t>(CURRENT_SENSITIVITY), 0x00};
  Packet reply;
  if (!command(CMD_SET_DISTANCE_GATE_THRESHOLDS, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply) ||
      reply.dataLen < static_cast<uint16_t>(3 + count)) {
    return false;
  }

  memcpy(thresholds, &reply.data[3], count);
  return true;
}

bool DFRobot_C4002::setSensitivityPreset(DistanceGateType type, SensitivityPreset preset) {
  uint8_t payload[6] = {CMD_SET_SENSITIVITY_PRESET, READ_AND_WRITE_REQ, 6, 0,
                        static_cast<uint8_t>(type), static_cast<uint8_t>(preset)};
  Packet reply;
  return command(CMD_SET_SENSITIVITY_PRESET, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::getSensitivityPreset(DistanceGateType type, SensitivityPreset &preset) {
  uint8_t payload[6] = {CMD_SET_SENSITIVITY_PRESET, READ_AND_WRITE_REQ, 6, 0, static_cast<uint8_t>(type), 0};
  Packet reply;
  if (!command(CMD_SET_SENSITIVITY_PRESET, FRAME_TYPE_READ_REQUEST, payload, sizeof(payload), reply) ||
      reply.dataLen < 2) {
    preset = SENSITIVITY_ERROR;
    return false;
  }

  preset = static_cast<SensitivityPreset>(reply.data[1]);
  return true;
}

uint8_t DFRobot_C4002::gateCount() const {
  return resolutionMode_ == RESOLUTION_20CM ? 25 : 15;
}

bool DFRobot_C4002::getAllConfig(Configuration &config) {
  bool ok = true;
  ok = getLedStatus(config.ledStatus) && ok;
  ok = getLightThresholdLux(config.lightThresholdLux) && ok;
  ok = getDetectionRangeCm(config.detectionRange.closestCm, config.detectionRange.farthestCm) && ok;
  ok = getTargetDisappearDelay(config.targetDisappearDelaySeconds) && ok;
  ok = getOutputMode(config.outputMode) && ok;
  ok = getResolutionMode(config.resolutionMode) && ok;
  ok = getSensitivityPreset(MOVING_TARGET_GATES, config.movingSensitivity) && ok;
  ok = getSensitivityPreset(STATIONARY_TARGET_GATES, config.stationarySensitivity) && ok;
  return ok;
}

bool DFRobot_C4002::startEnvironmentCalibration(uint16_t delaySeconds, uint16_t durationSeconds,
                                                bool autoGenerateThresholds) {
  uint8_t payload[9] = {CMD_ENVIRONMENT_CALIBRATION, READ_AND_WRITE_REQ, 9, 0};
  writeU16LE(&payload[4], delaySeconds);
  writeU16LE(&payload[6], durationSeconds);
  payload[8] = autoGenerateThresholds ? 0x01 : 0x00;

  Packet reply;
  return command(CMD_ENVIRONMENT_CALIBRATION, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::restart() {
  uint8_t payload[5] = {CMD_RESTART, READ_AND_WRITE_REQ, 5, 0, 0};
  Packet reply;
  return command(CMD_RESTART, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::factoryReset() {
  uint8_t payload[5] = {CMD_FACTORY_RESET, READ_AND_WRITE_REQ, 5, 0, 0};
  Packet reply;
  if (!command(CMD_FACTORY_RESET, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply)) {
    return false;
  }

  delay(10);
  payload[0] = CMD_FACTORY_RESET_USER;
  return command(CMD_FACTORY_RESET_USER, FRAME_TYPE_WRITE_REQUEST, payload, sizeof(payload), reply);
}

bool DFRobot_C4002::command(uint8_t cmd, uint8_t frameType, const uint8_t *payload, uint16_t payloadLen,
                            Packet &reply) {
  if (!sendPacket(payload, payloadLen, frameType)) {
    lastResponse_ = DATALEN_ERR;
    return false;
  }

  reply = receivePacket(200);
  lastResponse_ = reply.response;
  return reply.response == SUCCEED && reply.command == cmd &&
         (reply.type == FRAME_TYPE_WRITE_RESPONSE || reply.type == FRAME_TYPE_READ_RESPONSE);
}

bool DFRobot_C4002::sendPacket(const uint8_t *payload, uint16_t payloadLen, uint8_t frameType) {
  if (payload == nullptr || payloadLen + 10 > MAX_PACKET_LEN) {
    return false;
  }

  uint8_t packet[MAX_PACKET_LEN] = {};
  uint16_t index = 0;
  packet[index++] = FRAME_HEADER[0];
  packet[index++] = FRAME_HEADER[1];
  packet[index++] = FRAME_HEADER[2];
  packet[index++] = FRAME_HEADER[3];
  writeU16LE(&packet[index], payloadLen + 10);
  index += 2;
  packet[index++] = 0x00;
  packet[index++] = frameType;
  memcpy(&packet[index], payload, payloadLen);
  index += payloadLen;

  uint16_t sum = checksum(packet, index);
  writeU16LE(&packet[index], sum);
  index += 2;

  clearInput();
  return serial_->write(packet, index) == index;
}

DFRobot_C4002::Packet DFRobot_C4002::receivePacket(uint32_t timeoutMs) {
  Packet packet;
  uint8_t headerIndex = 0;
  uint8_t value = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    if (!readByte(value, timeoutMs)) {
      packet.response = AUTHENTICATION_ERR;
      lastResponse_ = packet.response;
      return packet;
    }

    if (value == FRAME_HEADER[headerIndex]) {
      headerIndex++;
      if (headerIndex == sizeof(FRAME_HEADER)) {
        break;
      }
    } else {
      headerIndex = value == FRAME_HEADER[0] ? 1 : 0;
    }
  }

  if (headerIndex != sizeof(FRAME_HEADER)) {
    packet.response = AUTHENTICATION_ERR;
    lastResponse_ = packet.response;
    return packet;
  }

  uint8_t prefix[4] = {};
  if (!readBytes(prefix, sizeof(prefix), timeoutMs)) {
    packet.response = DATALEN_ERR;
    lastResponse_ = packet.response;
    return packet;
  }

  uint16_t packetLen = readU16LE(prefix);
  if (packetLen < 12 || packetLen > MAX_PACKET_LEN) {
    packet.response = DATALEN_ERR;
    lastResponse_ = packet.response;
    return packet;
  }

  uint8_t raw[MAX_PACKET_LEN] = {};
  memcpy(raw, FRAME_HEADER, sizeof(FRAME_HEADER));
  memcpy(&raw[4], prefix, sizeof(prefix));

  uint16_t remaining = packetLen - 8;
  if (!readBytes(&raw[8], remaining, timeoutMs)) {
    packet.response = DATALEN_ERR;
    lastResponse_ = packet.response;
    return packet;
  }

  if (!checksumOk(raw, packetLen)) {
    packet.response = AUTHENTICATION_ERR;
    lastResponse_ = packet.response;
    return packet;
  }

  packet.type = raw[7];
  packet.command = raw[8];
  packet.response = static_cast<ResponseCode>(raw[9]);
  packet.dataLen = readU16LE(&raw[10]);

  uint16_t commandPayloadLen = packet.dataLen >= 4 ? packet.dataLen - 4 : 0;
  if (packet.dataLen < 4 || commandPayloadLen > MAX_DATA_LEN || packet.dataLen + 10 != packetLen) {
    packet.response = DATALEN_ERR;
    lastResponse_ = packet.response;
    return packet;
  }

  memcpy(packet.data, &raw[12], commandPayloadLen);
  packet.dataLen = commandPayloadLen;
  lastResponse_ = packet.response;
  return packet;
}

bool DFRobot_C4002::readBytes(uint8_t *buffer, size_t length, uint32_t timeoutMs) {
  for (size_t i = 0; i < length; i++) {
    if (!readByte(buffer[i], timeoutMs)) {
      return false;
    }
  }
  return true;
}

bool DFRobot_C4002::readByte(uint8_t &value, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    int incoming = serial_->read();
    if (incoming >= 0) {
      value = static_cast<uint8_t>(incoming);
      return true;
    }
    delay(1);
  }
  return false;
}

void DFRobot_C4002::clearInput() {
  while (serial_->available() > 0) {
    serial_->read();
  }
}

uint16_t DFRobot_C4002::checksum(const uint8_t *data, uint16_t length) {
  uint16_t sum = 0;
  for (uint16_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}

bool DFRobot_C4002::checksumOk(const uint8_t *data, uint16_t length) {
  if (length < 2) {
    return false;
  }

  uint16_t expected = readU16LE(&data[length - 2]);
  return checksum(data, length - 2) == expected;
}

uint16_t DFRobot_C4002::readU16LE(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

int16_t DFRobot_C4002::readI16LE(const uint8_t *data) {
  return static_cast<int16_t>(readU16LE(data));
}

uint32_t DFRobot_C4002::readU32LE(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void DFRobot_C4002::writeU16LE(uint8_t *data, uint16_t value) {
  data[0] = value & 0xFF;
  data[1] = value >> 8;
}

void DFRobot_C4002::parsePresenceData(const uint8_t *payload, uint16_t length) {
  if (length < 18) {
    lastResponse_ = DATALEN_ERR;
    return;
  }

  data_.targetState = static_cast<TargetState>(payload[0]);
  data_.lightLux = readU16LE(&payload[1]) * 0.1f;
  data_.stationaryDistanceIndex = readU32LE(&payload[3]);
  data_.stationaryCountdown = readU16LE(&payload[7]);
  data_.stationary.distanceMeters = readU16LE(&payload[9]) * 0.01f;
  data_.stationary.energy = payload[11];
  data_.moving.distanceMeters = readU16LE(&payload[12]) * 0.01f;
  data_.moving.speedMetersPerSecond = readI16LE(&payload[14]) * 0.01f;
  data_.moving.energy = payload[16];
  data_.moving.direction = static_cast<MoveDirection>(payload[17]);
}
