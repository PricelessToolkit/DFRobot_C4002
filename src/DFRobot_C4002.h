#pragma once

#include <Arduino.h>

class DFRobot_C4002 {
 public:
  enum ResolutionMode : uint8_t {
    RESOLUTION_80CM = 0x00,
    RESOLUTION_20CM = 0x01,
  };

  enum DistanceGateType : uint8_t {
    MOVING_TARGET_GATES = 0x00,
    STATIONARY_TARGET_GATES = 0x01,
  };

  enum VersionType : uint8_t {
    HARDWARE_VERSION = 0x00,
    SOFTWARE_VERSION = 0x01,
  };

  enum BaudRate : uint32_t {
    BAUD_57600 = 57600,
    BAUD_115200 = 115200,
    BAUD_230400 = 230400,
    BAUD_460800 = 460800,
    BAUD_500000 = 500000,
    BAUD_921600 = 921600,
    BAUD_1000000 = 1000000,
  };

  enum ResponseCode : uint8_t {
    READ_AND_WRITE_REQ = 0x00,
    SUCCEED = 0x01,
    CMD_ERR = 0x02,
    AUTHENTICATION_ERR = 0x03,
    RESOURCES_BUSY = 0x04,
    PARAMS_ERR = 0x05,
    DATALEN_ERR = 0x06,
    INTERNAL_ERR = 0x07,
  };

  enum MoveDirection : uint8_t {
    AWAY = 0,
    STAY = 1,
    NEAR = 2,
  };

  enum OutputMode : uint8_t {
    OUTPUT_ON_MOTION = 0x01,
    OUTPUT_ON_PRESENCE = 0x02,
    OUTPUT_ON_MOTION_OR_PRESENCE = 0x03,
  };

  enum TargetState : uint8_t {
    NO_TARGET = 0,
    STATIONARY_TARGET = 1,
    MOVING_TARGET = 2,
    MOVING_OR_STATIONARY_TARGET = 3,
    MOVING_TARGET_OR_NO_TARGET = 4,
    STATIONARY_TARGET_OR_NO_TARGET = 5,
    TARGET_ERROR = 255,
  };

  enum LedMode : uint8_t {
    LED_OFF = 0x00,
    LED_ON = 0x01,
    LED_KEEP = 0xFF,
  };

  enum UpdateEvent : uint8_t {
    NO_EVENT = 0,
    PRESENCE_DATA = 1,
    CALIBRATION_DATA = 2,
  };

  enum SensitivityPreset : uint8_t {
    LOW_SENSITIVITY = 0x00,
    MEDIUM_SENSITIVITY = 0x01,
    HIGH_SENSITIVITY = 0x02,
    CUSTOM_SENSITIVITY = 0x03,
    CURRENT_SENSITIVITY = 0xFF,
    SENSITIVITY_ERROR = 0xFE,
  };

  struct MovingTarget {
    float distanceMeters = 0.0f;
    float speedMetersPerSecond = 0.0f;
    uint8_t energy = 0;
    MoveDirection direction = STAY;
  };

  struct StationaryTarget {
    float distanceMeters = 0.0f;
    uint8_t energy = 0;
  };

  struct PresenceData {
    TargetState targetState = NO_TARGET;
    float lightLux = 0.0f;
    uint32_t stationaryDistanceIndex = 0;
    uint16_t stationaryCountdown = 0;
    StationaryTarget stationary;
    MovingTarget moving;
  };

  struct DetectionRange {
    uint16_t closestCm = 0;
    uint16_t farthestCm = 0;
  };

  struct LedStatus {
    LedMode runLed = LED_KEEP;
    LedMode outputLed = LED_KEEP;
  };

  struct Configuration {
    float lightThresholdLux = 0.0f;
    DetectionRange detectionRange;
    OutputMode outputMode = OUTPUT_ON_PRESENCE;
    ResolutionMode resolutionMode = RESOLUTION_80CM;
    SensitivityPreset movingSensitivity = SENSITIVITY_ERROR;
    SensitivityPreset stationarySensitivity = SENSITIVITY_ERROR;
    uint16_t targetDisappearDelaySeconds = 0;
    LedStatus ledStatus;
  };

  explicit DFRobot_C4002(Stream &serial);

  bool begin(ResolutionMode mode = RESOLUTION_80CM, uint8_t reportPeriod = 10);
  bool begin(uint8_t outputPin, ResolutionMode mode = RESOLUTION_80CM, uint8_t reportPeriod = 10);
  UpdateEvent update(uint32_t timeoutMs = 20);

  const PresenceData &data() const { return data_; }
  TargetState targetState() const { return data_.targetState; }
  bool presenceDetected() const { return data_.targetState == STATIONARY_TARGET || data_.targetState == MOVING_TARGET; }
  bool stationaryTargetDetected() const { return data_.targetState == STATIONARY_TARGET; }
  bool movementDetected() const { return data_.targetState == MOVING_TARGET; }
  float lightLux() const { return data_.lightLux; }
  float existingDistanceMeters() const { return data_.stationary.distanceMeters; }
  float movingDistanceMeters() const { return data_.moving.distanceMeters; }
  float movingSpeedMetersPerSecond() const { return data_.moving.speedMetersPerSecond; }
  MoveDirection movingDirection() const { return data_.moving.direction; }
  uint8_t existingEnergy() const { return data_.stationary.energy; }
  uint8_t movingEnergy() const { return data_.moving.energy; }
  uint16_t calibrationCountdownSeconds() const { return calibrationCountdownSeconds_; }
  ResponseCode lastResponse() const { return lastResponse_; }

  bool setReportPeriod(uint8_t period);
  bool setSerialBaudRate(BaudRate baudRate);
  bool getVersionInfo(VersionType type, String &version);
  bool setResolutionMode(ResolutionMode mode);
  bool getResolutionMode(ResolutionMode &mode);
  ResolutionMode resolutionMode() const { return resolutionMode_; }

  bool setDetectionRangeCm(uint16_t closestCm, uint16_t farthestCm);
  bool setDetectionRangeMeters(float closestMeters, float farthestMeters);
  bool getDetectionRangeCm(uint16_t &closestCm, uint16_t &farthestCm);
  bool getDetectionRangeMeters(float &closestMeters, float &farthestMeters);

  bool setLightThresholdLux(float thresholdLux);
  bool getLightThresholdLux(float &thresholdLux);
  bool setTargetDisappearDelay(uint16_t seconds);
  bool getTargetDisappearDelay(uint16_t &seconds);

  bool setOutputMode(OutputMode mode);
  bool getOutputMode(OutputMode &mode);
  TargetState outputPinTargetState() const;
  bool setRunLed(bool enabled);
  bool setOutputLed(bool enabled);
  bool getLedStatus(LedStatus &status);

  bool enableDistanceGates(DistanceGateType type, const uint8_t *gateMask, uint8_t gateCount);
  bool enableAllDistanceGates(bool enabled = true);
  bool setDistanceGateThresholds(DistanceGateType type, const uint8_t *thresholds, uint8_t thresholdCount);
  bool getDistanceGateThresholds(DistanceGateType type, uint8_t *thresholds, uint8_t thresholdCount);
  bool setSensitivityPreset(DistanceGateType type, SensitivityPreset preset);
  bool getSensitivityPreset(DistanceGateType type, SensitivityPreset &preset);
  uint8_t gateCount() const;

  bool getAllConfig(Configuration &config);

  bool startEnvironmentCalibration(uint16_t delaySeconds = 3, uint16_t durationSeconds = 15,
                                   bool autoGenerateThresholds = true);
  bool restart();
  bool factoryReset();

 private:
  static const uint8_t MAX_DATA_LEN = 64;
  static const uint8_t MAX_PACKET_LEN = 96;
  static const uint8_t MAX_GATE_COUNT = 25;

  struct Packet {
    uint8_t command = 0;
    ResponseCode response = AUTHENTICATION_ERR;
    uint16_t dataLen = 0;
    uint8_t data[MAX_DATA_LEN] = {};
    uint8_t type = 0xFF;
  };

  bool command(uint8_t cmd, uint8_t frameType, const uint8_t *payload, uint16_t payloadLen, Packet &reply);
  bool sendPacket(const uint8_t *payload, uint16_t payloadLen, uint8_t frameType);
  Packet receivePacket(uint32_t timeoutMs);
  bool readBytes(uint8_t *buffer, size_t length, uint32_t timeoutMs);
  bool readByte(uint8_t &value, uint32_t timeoutMs);
  void clearInput();

  static uint16_t checksum(const uint8_t *data, uint16_t length);
  static bool checksumOk(const uint8_t *data, uint16_t length);
  static uint16_t readU16LE(const uint8_t *data);
  static int16_t readI16LE(const uint8_t *data);
  static uint32_t readU32LE(const uint8_t *data);
  static void writeU16LE(uint8_t *data, uint16_t value);

  void parsePresenceData(const uint8_t *payload, uint16_t length);

  Stream *serial_;
  PresenceData data_;
  uint16_t calibrationCountdownSeconds_ = 0;
  uint8_t outputPin_ = 255;
  ResolutionMode resolutionMode_ = RESOLUTION_80CM;
  OutputMode outputMode_ = OUTPUT_ON_PRESENCE;
  ResponseCode lastResponse_ = AUTHENTICATION_ERR;
};
