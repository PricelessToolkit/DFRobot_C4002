#include <DFRobot_C4002.h>

#if defined(ESP32)
#define radarSerial Serial2
const int RADAR_RX_PIN = 16;
const int RADAR_TX_PIN = 17;
#elif defined(HAVE_HWSERIAL1)
#define radarSerial Serial1
#else
#include <SoftwareSerial.h>
SoftwareSerial radarSerial(10, 11); // RX, TX
#endif

// Set this to the MCU pin wired to the C4002 OUT pin, or leave it disabled.
const int RADAR_OUT_PIN = -1;

DFRobot_C4002 radar(radarSerial);

const char *targetStateName(DFRobot_C4002::TargetState state) {
  switch (state) {
    case DFRobot_C4002::NO_TARGET:
      return "none";
    case DFRobot_C4002::STATIONARY_TARGET:
      return "stationary";
    case DFRobot_C4002::MOVING_TARGET:
      return "moving";
    case DFRobot_C4002::MOVING_OR_STATIONARY_TARGET:
      return "moving-or-stationary";
    case DFRobot_C4002::MOVING_TARGET_OR_NO_TARGET:
      return "moving-or-none";
    case DFRobot_C4002::STATIONARY_TARGET_OR_NO_TARGET:
      return "stationary-or-none";
    default:
      return "error";
  }
}

const char *sensitivityName(DFRobot_C4002::SensitivityPreset preset) {
  switch (preset) {
    case DFRobot_C4002::LOW_SENSITIVITY:
      return "low";
    case DFRobot_C4002::MEDIUM_SENSITIVITY:
      return "medium";
    case DFRobot_C4002::HIGH_SENSITIVITY:
      return "high";
    case DFRobot_C4002::CUSTOM_SENSITIVITY:
      return "custom";
    case DFRobot_C4002::CURRENT_SENSITIVITY:
      return "current";
    default:
      return "error";
  }
}

bool beginRadar() {
  if (RADAR_OUT_PIN >= 0) {
    return radar.begin((uint8_t)RADAR_OUT_PIN, DFRobot_C4002::RESOLUTION_80CM, 10);
  }
  return radar.begin(DFRobot_C4002::RESOLUTION_80CM, 10);
}

void printVersion(DFRobot_C4002::VersionType type, const char *label) {
  String version;
  if (radar.getVersionInfo(type, version)) {
    Serial.print(label);
    Serial.print(": ");
    Serial.println(version);
  } else {
    Serial.print(label);
    Serial.print(" read failed. Response code: ");
    Serial.println(radar.lastResponse());
  }
}

void printAllConfig() {
  DFRobot_C4002::Configuration config;
  if (!radar.getAllConfig(config)) {
    Serial.print("Config read had one or more failures. Last response code: ");
    Serial.println(radar.lastResponse());
  }

  Serial.print("Range: ");
  Serial.print(config.detectionRange.closestCm);
  Serial.print(" cm to ");
  Serial.print(config.detectionRange.farthestCm);
  Serial.println(" cm");

  Serial.print("Light threshold: ");
  Serial.print(config.lightThresholdLux, 1);
  Serial.println(" lx");

  Serial.print("Disappear delay: ");
  Serial.print(config.targetDisappearDelaySeconds);
  Serial.println(" s");

  Serial.print("Moving sensitivity: ");
  Serial.println(sensitivityName(config.movingSensitivity));

  Serial.print("Stationary sensitivity: ");
  Serial.println(sensitivityName(config.stationarySensitivity));
}

void configureThresholds() {
  uint8_t thresholds[25] = {};
  uint8_t count = radar.gateCount();

  for (uint8_t i = 0; i < count; i++) {
    thresholds[i] = 50;
  }

  if (!radar.setDistanceGateThresholds(DFRobot_C4002::MOVING_TARGET_GATES, thresholds, count)) {
    Serial.print("Moving threshold setup failed. Response code: ");
    Serial.println(radar.lastResponse());
  }

  if (!radar.setDistanceGateThresholds(DFRobot_C4002::STATIONARY_TARGET_GATES, thresholds, count)) {
    Serial.print("Stationary threshold setup failed. Response code: ");
    Serial.println(radar.lastResponse());
  }

  if (radar.getDistanceGateThresholds(DFRobot_C4002::STATIONARY_TARGET_GATES, thresholds, count)) {
    Serial.print("Stationary gate thresholds:");
    for (uint8_t i = 0; i < count; i++) {
      Serial.print(' ');
      Serial.print(thresholds[i]);
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

#if defined(ESP32)
  radarSerial.begin(9600, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#else
  radarSerial.begin(9600);
#endif

  if (!beginRadar()) {
    Serial.print("C4002 begin failed. Response code: ");
    Serial.println(radar.lastResponse());
    return;
  }

  printVersion(DFRobot_C4002::HARDWARE_VERSION, "Hardware version");
  printVersion(DFRobot_C4002::SOFTWARE_VERSION, "Software version");

  radar.setDetectionRangeMeters(0.2f, 6.0f);
  radar.setLightThresholdLux(50.0f);
  radar.setTargetDisappearDelay(5);
  radar.setOutputMode(DFRobot_C4002::OUTPUT_ON_PRESENCE);
  radar.setRunLed(true);
  radar.setOutputLed(true);

  radar.setSensitivityPreset(DFRobot_C4002::MOVING_TARGET_GATES, DFRobot_C4002::HIGH_SENSITIVITY);
  radar.setSensitivityPreset(DFRobot_C4002::STATIONARY_TARGET_GATES, DFRobot_C4002::MEDIUM_SENSITIVITY);
  configureThresholds();

  // Baud rate changes take effect after restart. Uncomment only when you are ready
  // to reopen radarSerial at the same new speed in your sketch.
  // radar.setSerialBaudRate(DFRobot_C4002::BAUD_115200);
  // radar.restart();

  printAllConfig();
}

void loop() {
  if (radar.update() == DFRobot_C4002::PRESENCE_DATA) {
    Serial.print("Presence: ");
    Serial.print(radar.presenceDetected() ? "yes" : "no");

    if (RADAR_OUT_PIN >= 0) {
      Serial.print(" OUT pin: ");
      Serial.print(targetStateName(radar.outputPinTargetState()));
    }

    Serial.println();
  }
}
