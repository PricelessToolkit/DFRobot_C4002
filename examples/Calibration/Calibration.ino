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

  Serial.println("Starting environment calibration...");
  if (!radar.startEnvironmentCalibration(3, 15)) {
    Serial.print("Calibration command failed. Response code: ");
    Serial.println(radar.lastResponse());
  }
}

void loop() {
  DFRobot_C4002::UpdateEvent event = radar.update();

  if (event == DFRobot_C4002::CALIBRATION_DATA) {
    Serial.print("Calibration seconds left: ");
    Serial.println(radar.calibrationCountdownSeconds());
  }

  if (event == DFRobot_C4002::PRESENCE_DATA) {
    Serial.print("Presence: ");
    Serial.println(radar.presenceDetected() ? "yes" : "no");
  }
}
