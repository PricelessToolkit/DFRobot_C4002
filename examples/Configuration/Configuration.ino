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

void setup() {
  Serial.begin(115200);
  delay(1000);

#if defined(ESP32)
  radarSerial.begin(9600, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#else
  radarSerial.begin(9600);
#endif

  if (!radar.begin(DFRobot_C4002::RESOLUTION_80CM, 10)) {
    Serial.print("C4002 begin failed. Response code: ");
    Serial.println(radar.lastResponse());
    return;
  }

  radar.setDetectionRangeMeters(0.2f, 6.0f);
  radar.setLightThresholdLux(50.0f);
  radar.setTargetDisappearDelay(5);
  radar.setOutputMode(DFRobot_C4002::OUTPUT_ON_PRESENCE);
  radar.setRunLed(true);
  radar.setOutputLed(true);

  float nearest = 0.0f;
  float farthest = 0.0f;
  if (radar.getDetectionRangeMeters(nearest, farthest)) {
    Serial.print("Detection range: ");
    Serial.print(nearest, 2);
    Serial.print(" m to ");
    Serial.print(farthest, 2);
    Serial.println(" m");
  }

  float lightThreshold = 0.0f;
  if (radar.getLightThresholdLux(lightThreshold)) {
    Serial.print("Light threshold: ");
    Serial.print(lightThreshold, 1);
    Serial.println(" lx");
  }
}

void loop() {
  if (radar.update() == DFRobot_C4002::PRESENCE_DATA) {
    Serial.print("Presence: ");
    Serial.println(radar.presenceDetected() ? "yes" : "no");
  }
}
