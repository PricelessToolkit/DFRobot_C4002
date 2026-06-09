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

const char *directionName(DFRobot_C4002::MoveDirection direction) {
  switch (direction) {
    case DFRobot_C4002::AWAY:
      return "away";
    case DFRobot_C4002::NEAR:
      return "near";
    case DFRobot_C4002::STAY:
    default:
      return "stay";
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

  Serial.println("Starting DFRobot C4002...");
  if (!radar.begin()) {
    Serial.print("C4002 begin failed. Response code: ");
    Serial.println(radar.lastResponse());
    return;
  }

  String version;
  if (radar.getVersionInfo(DFRobot_C4002::SOFTWARE_VERSION, version)) {
    Serial.print("Software version: ");
    Serial.println(version);
  }
}

void loop() {
  if (radar.update() != DFRobot_C4002::PRESENCE_DATA) {
    return;
  }

  Serial.print("target=");
  Serial.print(targetStateName(radar.targetState()));
  Serial.print(" present=");
  Serial.print(radar.presenceDetected() ? "yes" : "no");
  Serial.print(" light=");
  Serial.print(radar.lightLux(), 1);
  Serial.print(" lx stationary=");
  Serial.print(radar.existingDistanceMeters(), 2);
  Serial.print(" m moving=");
  Serial.print(radar.movingDistanceMeters(), 2);
  Serial.print(" m speed=");
  Serial.print(radar.movingSpeedMetersPerSecond(), 2);
  Serial.print(" m/s direction=");
  Serial.println(directionName(radar.movingDirection()));
}
