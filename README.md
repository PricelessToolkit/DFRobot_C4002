# DFRobot_C4002

Arduino driver for the DFRobot C4002 mmWave human presence sensor.

This library wraps the C4002 UART protocol in a small Arduino API that works with any `Stream` object.

## Features

- Presence state: no target, stationary target, or moving target
- Stationary target distance and energy
- Moving target distance, speed, direction, and energy
- Light level
- UART configuration commands
- Detection range setup
- Report period setup
- Resolution mode setup
- Run/output LED control
- Output pin mode control
- Environment calibration
- Restart and factory reset

## Wiring

Use a UART port from your board.

| C4002 pin | Arduino / ESP32 pin |
| --- | --- |
| VCC | Sensor-rated supply |
| GND | GND |
| TX | MCU RX |
| RX | MCU TX |

For ESP32, `Serial2` is usually the easiest choice:

```cpp
Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
```

Check your exact C4002 module voltage before wiring. Many radar modules use 5 V power, but the UART logic level depends on the board.

## Install

Copy or symlink the `DFRobot_C4002` folder into your Arduino `libraries` directory, or use it as a local library in PlatformIO.

The folder layout is the normal Arduino library format:

```text
DFRobot_C4002/
  library.properties
  src/
    DFRobot_C4002.h
    DFRobot_C4002.cpp
  examples/
```

## Minimal ESP32 Example

```cpp
#include <DFRobot_C4002.h>

DFRobot_C4002 radar(Serial2);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  if (!radar.begin()) {
    Serial.println("C4002 begin failed");
  }
}

void loop() {
  if (radar.update() == DFRobot_C4002::PRESENCE_DATA) {
    Serial.print("State: ");
    Serial.print(radar.targetState());
    Serial.print(" moving distance: ");
    Serial.print(radar.movingDistanceMeters());
    Serial.print(" m, existing distance: ");
    Serial.print(radar.existingDistanceMeters());
    Serial.println(" m");
  }
}
```

## API

### Construction

```cpp
DFRobot_C4002 radar(Serial2);
```

The constructor accepts any Arduino `Stream`, including `HardwareSerial`, `SoftwareSerial`, or compatible serial classes.

### Startup

```cpp
bool ok = radar.begin();
bool ok = radar.begin(DFRobot_C4002::RESOLUTION_80CM, 10);
```

`reportPeriod` is in 100 ms units. A value of `10` means roughly one report per second.

### Reading Data

Call `update()` often from `loop()`. It returns an event type.

```cpp
DFRobot_C4002::UpdateEvent event = radar.update();

if (event == DFRobot_C4002::PRESENCE_DATA) {
  bool present = radar.presenceDetected();
}

if (event == DFRobot_C4002::CALIBRATION_DATA) {
  uint16_t secondsLeft = radar.calibrationCountdownSeconds();
}
```

Useful getters:

```cpp
radar.presenceDetected();
radar.movementDetected();
radar.stationaryTargetDetected();
radar.targetState();
radar.lightLux();
radar.existingDistanceMeters();
radar.movingDistanceMeters();
radar.movingSpeedMetersPerSecond();
radar.movingDirection();
radar.existingEnergy();
radar.movingEnergy();
```

### Configuration Commands

All configuration commands return `true` when the C4002 accepts the command and `false` when the command fails or times out. Use `radar.lastResponse()` to inspect the last sensor response code.

#### Report Period

```cpp
// Unit is 100 ms. 10 means about one report per second.
radar.setReportPeriod(10);

// Faster reports: about 500 ms.
radar.setReportPeriod(5);
```

#### LEDs

```cpp
// Turn the running/status LED on or off.
radar.setRunLed(true);
radar.setRunLed(false);

// Turn the output LED on or off.
radar.setOutputLed(true);
radar.setOutputLed(false);
```

#### Detection Range

```cpp
// Use meters for easy sketches.
radar.setDetectionRangeMeters(0.2, 6.0);

// Or use centimeters for exact integer values.
radar.setDetectionRangeCm(20, 600);

float nearest = 0.0;
float farthest = 0.0;
if (radar.getDetectionRangeMeters(nearest, farthest)) {
  Serial.print("Range: ");
  Serial.print(nearest);
  Serial.print(" m to ");
  Serial.print(farthest);
  Serial.println(" m");
}
```

The sensor range is clamped to 1200 cm by the library.

#### Resolution Mode

```cpp
radar.setResolutionMode(DFRobot_C4002::RESOLUTION_80CM);
radar.setResolutionMode(DFRobot_C4002::RESOLUTION_20CM);

DFRobot_C4002::ResolutionMode mode;
if (radar.getResolutionMode(mode)) {
  Serial.println(mode == DFRobot_C4002::RESOLUTION_20CM ? "20 cm" : "80 cm");
}
```

`RESOLUTION_80CM` uses 15 distance gates. `RESOLUTION_20CM` uses 25 distance gates.

#### Output Pin Mode

```cpp
// Output is active only when movement is detected.
radar.setOutputMode(DFRobot_C4002::OUTPUT_ON_MOTION);

// Output is active when presence is detected.
radar.setOutputMode(DFRobot_C4002::OUTPUT_ON_PRESENCE);

// Output is active for movement or presence.
radar.setOutputMode(DFRobot_C4002::OUTPUT_ON_MOTION_OR_PRESENCE);

DFRobot_C4002::OutputMode outputMode;
if (radar.getOutputMode(outputMode)) {
  Serial.print("Output mode: ");
  Serial.println(outputMode);
}
```

#### Light Threshold

```cpp
radar.setLightThresholdLux(50.0);

float threshold = 0.0;
if (radar.getLightThresholdLux(threshold)) {
  Serial.print("Light threshold: ");
  Serial.print(threshold);
  Serial.println(" lx");
}
```

#### Target Disappear Delay

```cpp
// Delay in seconds before a disappeared target is cleared.
radar.setTargetDisappearDelay(5);

uint16_t delaySeconds = 0;
if (radar.getTargetDisappearDelay(delaySeconds)) {
  Serial.print("Disappear delay: ");
  Serial.print(delaySeconds);
  Serial.println(" s");
}
```

#### Distance Gates

Most sketches can leave all gates enabled, which is what `begin()` does. Advanced sketches can enable or disable individual gates. Each gate value is `1` for enabled and `0` for disabled.

```cpp
radar.enableAllDistanceGates(true);

uint8_t gates[15] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
gates[0] = 0; // ignore closest gate

radar.enableDistanceGates(DFRobot_C4002::MOVING_TARGET_GATES, gates, 15);
radar.enableDistanceGates(DFRobot_C4002::STATIONARY_TARGET_GATES, gates, 15);
```

Use `15` gates in `RESOLUTION_80CM` mode and `25` gates in `RESOLUTION_20CM` mode. You can also call `radar.gateCount()` after setting the resolution.

### Calibration

```cpp
// Wait 3 seconds, then calibrate for 15 seconds.
radar.startEnvironmentCalibration(3, 15);
```

Keep calling `update()` during calibration. `CALIBRATION_DATA` events report the countdown.

```cpp
if (radar.update() == DFRobot_C4002::CALIBRATION_DATA) {
  Serial.print("Calibration seconds left: ");
  Serial.println(radar.calibrationCountdownSeconds());
}
```

### Restart And Factory Reset

```cpp
// Restart the sensor module.
radar.restart();

// Restore sensor settings to factory defaults.
radar.factoryReset();
```

### Error Handling

Most configuration methods return `true` on success and `false` on failure.

```cpp
if (!radar.setDetectionRangeMeters(0.2, 6.0)) {
  Serial.print("Command failed, response code: ");
  Serial.println(radar.lastResponse());
}
```

## Notes

- The C4002 uses framed binary packets over UART.
- Distances from live readings are exposed in meters.
- Detection range setters accept centimeters or meters.
- `setReportPeriod()` uses the sensor firmware unit: 100 ms per step.
- `RESOLUTION_80CM` uses 15 distance gates. `RESOLUTION_20CM` uses 25 gates.

## Examples

- `BasicPresence`: read and print live presence data.
- `Configuration`: set common configuration values.
- `Calibration`: start environment calibration and print countdown updates.
- `MultiZonePresence`: split presence into six software zones from 1 m to 6 m.
