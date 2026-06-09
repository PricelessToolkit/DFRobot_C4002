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

const uint8_t ZONE_COUNT = 6;
const float ZONE_SIZE_METERS = 1.0f;

int zoneForDistance(float distanceMeters) {
  if (distanceMeters <= 0.0f || distanceMeters > ZONE_COUNT * ZONE_SIZE_METERS) {
    return 0;
  }

  int zone = (int)((distanceMeters - 0.001f) / ZONE_SIZE_METERS) + 1;
  if (zone < 1 || zone > ZONE_COUNT) {
    return 0;
  }
  return zone;
}

float activeTargetDistance() {
  if (radar.movementDetected()) {
    return radar.movingDistanceMeters();
  }

  if (radar.stationaryTargetDetected()) {
    return radar.existingDistanceMeters();
  }

  return 0.0f;
}

void printZones(int activeZone) {
  for (uint8_t zone = 1; zone <= ZONE_COUNT; zone++) {
    Serial.print("Z");
    Serial.print(zone);
    Serial.print("=");
    Serial.print(activeZone == zone ? "ON" : "off");
    if (zone < ZONE_COUNT) {
      Serial.print("  ");
    }
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

#if defined(ESP32)
  radarSerial.begin(9600, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#else
  radarSerial.begin(9600);
#endif

  Serial.println("Starting C4002 six-zone presence example...");

  if (!radar.begin(DFRobot_C4002::RESOLUTION_80CM, 10)) {
    Serial.print("C4002 begin failed. Response code: ");
    Serial.println(radar.lastResponse());
    return;
  }

  // Keep the hardware detection range inside the six software zones.
  if (!radar.setDetectionRangeMeters(0.0f, 6.0f)) {
    Serial.print("Range setup failed. Response code: ");
    Serial.println(radar.lastResponse());
  }

  if (!radar.setSensitivityPreset(DFRobot_C4002::MOVING_TARGET_GATES, DFRobot_C4002::HIGH_SENSITIVITY)) {
    Serial.print("Moving sensitivity setup failed. Response code: ");
    Serial.println(radar.lastResponse());
  }

  if (!radar.setSensitivityPreset(DFRobot_C4002::STATIONARY_TARGET_GATES, DFRobot_C4002::HIGH_SENSITIVITY)) {
    Serial.print("Stationary sensitivity setup failed. Response code: ");
    Serial.println(radar.lastResponse());
  }
}

void loop() {
  if (radar.update() != DFRobot_C4002::PRESENCE_DATA) {
    return;
  }

  float distance = activeTargetDistance();
  int activeZone = zoneForDistance(distance);

  Serial.print("presence=");
  Serial.print(radar.presenceDetected() ? "yes" : "no");
  Serial.print(" distance=");
  Serial.print(distance, 2);
  Serial.print(" m zone=");
  Serial.println(activeZone);

  printZones(activeZone);
}
