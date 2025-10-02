# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RB-Avionics is an open-source aviation instruments suite designed for ESP32-S3 based displays. This is the RB-02 variant focusing on "SixPack" aviation instruments display with GPS integration, IMU-based attitude indicator, and Stratux BLE traffic reception.

**Target Hardware:**
- ESP32-S3-Touch-LCD-2.8C or ESP32-S3-Touch-LCD-2.1 (Waveshare)
- BMP280 barometer
- QMI8658 IMU (6-axis gyro/accelerometer)
- GT911 touch controller (2.8") or CST820 (2.1")
- PCF85063 RTC
- SD card for maps, checklists, and data logging
- TTL NMEA GPS (e.g., uBlox NEO M6N)

## Build Commands

This project uses ESP-IDF (version 5.5.1+) build system.

```bash
# Configure project (menuconfig for custom settings)
idf.py menuconfig

# Build the project
idf.py build

# Flash to device (DIO mode, 16MB flash)
idf.py -p <PORT> flash

# Monitor serial output
idf.py -p <PORT> monitor

# Build and flash in one command
idf.py -p <PORT> flash monitor

# Clean build
idf.py fullclean
```

**Flash Configuration:**
- Chip: esp32s3
- Flash mode: DIO
- Flash size: 16MB
- Bootloader: 0x0
- Partition table: 0x8000
- Application: 0x10000

## Architecture

### Build Configuration System

The project uses a templating system for different hardware/feature combinations via `main/RB/BuildMachine.h`. This file defines compile-time flags that control:
- Display size (2.8" or 2.1")
- Touch vs non-touch variants
- Enabled instruments (attitude, altimeter, GPS map, speed, turn coordinator, etc.)
- Frame rate (30Hz typical)
- Feature flags (external maps, checklists, GPS, Bluetooth traffic)

Template files: `main/RB/BuildMachine-Template-RB-02-*.h`

### Component Structure

**main/** - Application code organized by subsystem:
- **RB/** - Core avionics logic (RB02_*.c/h files)
  - `RB02.c/h` - Main UI coordinator and workflow
  - `RB02_AAttitude.c/h` - Advanced attitude indicator (GPS-assisted)
  - `RB02_Altimeter.c/h` - Altimeter with QNH settings
  - `RB02_GPSMap.c/h` - BMP tile-based GPS map
  - `RB02_Gyro.c/h` - Directional gyro (gyroscope + GPS track)
  - `RB02_Workflow.c/h` - State machine and screen navigation
  - `RB02_Config.c/h` - NVS-based persistent configuration
  - `RB02_DataLogger.c/h` - SD card flight logging
  - `RB02_Checklist.c/h` - SD card-based checklists
  - `RB05_Traffic.c/h` - Stratux BLE traffic display
  - `RB05_Radar.c/h` - Traffic radar visualization
  - `RB04_EMS.c/h` - Engine monitoring system
  - `madgwick.c/h` - Madgwick AHRS filter for sensor fusion
  - `Vendor.c/h` - Vendor-specific utilities

- **LCD_Driver/** - ST7701S display driver
- **Touch_Driver/** - GT911/CST820 touch controller support
- **LVGL_Driver/** - LVGL integration layer
- **LVGL_UI/** - LVGL UI components
- **I2C_Driver/** - I2C bus management
- **QMI8658/** - IMU sensor driver
- **PCF85063/** - RTC driver
- **SD_Card/** - SD/MMC interface
- **BAT_Driver/** - Battery voltage monitoring
- **Buzzer/** - Buzzer control
- **Bluetooth/** - RB05 Stratux BLE integration
- **EXIO/** - TCA9554PWR GPIO expander
- **Wireless/** - WiFi/wireless features

**components/** - ESP-IDF managed components:
- `lvgl__lvgl` - LVGL graphics library (~8.2.0)

### Key Architectural Patterns

**FreeRTOS Tasks:**
- `Driver_Loop` - Main sensor polling loop (runs at ~20Hz configurable via `DriverLoopMilliseconds`)
  - Polls QMI8658 IMU every loop
  - Polls RTC, BMP280, and battery at ~1Hz
  - Starts after calibration (`workflow > 100`)

**LVGL Integration:**
- Frame buffer allocated from PSRAM (SPIRAM_MODE_OCT, 80MHz)
- Optional double buffering or bounce buffer modes (see `Kconfig.projbuild`)
- Semaphore-based tear prevention when not using double buffer

**Sensor Fusion:**
- Madgwick filter combines QMI8658 IMU data with GPS track for attitude determination
- GPS-assisted heading and track calculation

**Storage:**
- SD card required for: GPS map tiles (BMP format), checklists (text files), flight data logs
- NVS (non-volatile storage) for configuration persistence
- Map tiles: MapBox-compatible BMP tiles loaded from SD

**Communication:**
- UART-based NMEA GPS parsing (`RB02_NMEA.h`)
- Bluetooth LE for Stratux traffic reception (FLARM protocol)

## Configuration Notes

**menuconfig options:**
- `SPIRAM` configuration is critical for performance (OCT mode, 80MHz)
- Enable `SPIRAM_FETCH_INSTRUCTIONS` and `SPIRAM_RODATA` for higher PCLK
- Frame buffer options: double buffer vs bounce buffer vs semaphore tear prevention
- Bluetooth configuration for Stratux traffic features

**Vibration Correction:**
- The system includes vibration filtering for IMU data - consult with mechanics for proper settings

**Aircraft Profile:**
- Must be configured via setup screens for proper operation
- Stored in NVS

## Important Development Notes

- PSRAM configuration is mandatory for this application
- Display refresh and sensor polling rates can be tuned via `DriverLoopMilliseconds`
- When adding new instruments, follow the pattern in existing `RB02_*.c/h` files
- Build templates in `main/RB/BuildMachine-Template-*.h` show all available feature combinations
- License: GNU AGPLv3 / Dual commercial licensing
