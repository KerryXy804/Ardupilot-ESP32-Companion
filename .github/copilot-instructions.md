# AI Assistant Instructions for AoA Sensor Project

## Project Overview
This project implements an Angle of Attack (AoA) sensor system using an ESP32 microcontroller and AS5600 magnetic position sensors. The system is designed to measure and display angles from two AoA vanes.

## Architecture
- Built on PlatformIO framework for ESP32
- Uses RobTillaart's AS5600 library for magnetic position sensing
- Core functionality in `src/main.cpp`
- Dual-sensor configuration with calibration capability

## Development Setup

### Dependencies
Required libraries (configured in `platformio.ini`):
- Wire (I2C communication)
- SPI
- SD
- AS5600 (from RobTillaart's GitHub)

### Build and Upload
Use PlatformIO commands:
```
pio run                 # Build project
pio run --target upload # Upload to ESP32
```

## Key Implementation Patterns

### Sensor Initialization
```cpp
AS5600 as5600;
if (!as5600.begin()) {
    Serial.println("AS5600 not detected. Check connections.");
    while (1);
}
```

### Angle Normalization
All angle measurements are normalized to [-180, 180] range:
```cpp
if (angle > 180) angle -= 360;
if (angle <= -180) angle += 360;
```

### Zero-Point Calibration
The system implements a zero-point calibration mechanism:
1. On startup, waits for vane positioning (5 seconds)
2. Captures current angles as zero reference
3. All subsequent measurements are relative to these zero points

## Common Tasks

### Adding New Sensor Features
1. Add configuration in `setup()`
2. Implement reading logic in `loop()`
3. Apply the standard angle normalization pattern

### Serial Output
- Baud rate: 115200
- Format: "AoA 1: X°, AoA 2: Y°"
- Update frequency: 1Hz (configurable via delay)