# RB02 v1.5 - Upstream Merge Release

**Release Date:** October 3, 2025
**Version:** v1.5
**Git Commit:** aa95fd6
**Hardware:** ESP32-S3-Touch-LCD-2.1 (Waveshare Round Display)

---

## What's New in v1.5

### 🆕 Panel Alignment System
Merged from upstream [xiaprojects/rb-esp](https://github.com/xiaprojects/rb-esp) - Issue #92

**New Features:**
- **Panel Pitch Adjustment:** Slider for -64° to +64° pitch compensation
- **Panel Roll Adjustment:** Slider for -64° to +64° roll compensation
- **Panel Yaw Adjustment:** Slider for -64° to +64° yaw compensation
- **Real-time Attitude Display:** Shows current Roll/Pitch/Track during adjustment
- **NVS Persistence:** Settings automatically saved to non-volatile storage

**Usage:**
1. Navigate to Settings page (swipe/tap to Settings)
2. Scroll to Panel Alignment section
3. Adjust sliders to compensate for display mounting angle in aircraft
4. Settings persist across reboots

**Why This Matters:**
If your display is not mounted perfectly level in the aircraft panel, you can now compensate for the mounting angle to ensure accurate attitude indication.

### 🔍 I2C Debug Functionality
- Boot-time I2C bus scan (when `RB_DISPLAY_DEBUG` enabled)
- Detects all connected I2C devices (RTC, IMU, Barometer, GPIO Expander)
- Helpful for hardware troubleshooting

---

## Included from v1.4 (Boot Optimization)

All v1.4 stability improvements are preserved:

✅ **100% Boot Success Rate** (was 40-60%)
✅ **~2.0s Boot Time** (was 2.5-3.5s)
✅ **I2C Retry Logic** with 2000ms timeout
✅ **GPS UART Early Init** with NVS baudrate support
✅ **Extended LCD Delays** for ST7701S stability
✅ **PSRAM Memtest Disabled** (saves 500ms boot time)
✅ **Brownout Level 6** (2.43V threshold)

See `BOOT_OPTIMIZATION_v1.4.md` for full v1.4 details.

---

## Bug Fixes in v1.5

**6 Post-Merge Fixes:**
1. ✅ Merge conflict markers resolved
2. ✅ I2C timeout macro redefinition fixed
3. ✅ SettingStatus2 variable declaration fixed
4. ✅ Bluetooth settings ifdef guards added
5. ✅ TCA9554PWR include missing header fixed
6. ✅ **NULL pointer crash fixed** (Settings page stability)

All fixes documented in `UPSTREAM_MERGE_2025-10-03.md`.

---

## Testing Results

**Hardware Tested:** ESP32-S3-Touch-LCD-2.1 Round Display

### Boot Stability
- ✅ 10/10 successful boots (100%)
- ✅ Average boot time: ~2.0 seconds
- ✅ I2C devices: 4/4 detected (TCA9554, PCF85063, QMI8658, BMP280)
- ✅ GPS UART: Receiving NMEA data (810 bytes in 10 attempts)

### Settings Page
- ✅ No crashes when opening Settings
- ✅ Panel Pitch slider functional
- ✅ Panel Roll slider functional
- ✅ Panel Yaw slider functional
- ✅ Real-time display updates correctly
- ✅ Settings persist after reboot

### Display & Touch
- ✅ LCD initialized without black screen
- ✅ CST820 touch controller functional
- ✅ Tab switching stable

---

## Installation

### Prerequisites
- ESP-IDF v5.5.1 or later
- ESP32-S3 with 16MB flash, 8MB PSRAM
- USB-C cable for flashing

### Quick Flash (Recommended)
```bash
# Windows (PowerShell/CMD)
esptool.py --chip esp32s3 --port COM8 --baud 921600 ^
  --before default_reset --after hard_reset write_flash ^
  0x0 bootloader.bin ^
  0x8000 partition-table.bin ^
  0x10000 RB02_Faruk_2.1.bin

# Linux/macOS
esptool.py --chip esp32s3 --port /dev/ttyUSB0 --baud 921600 \
  --before default_reset --after hard_reset write_flash \
  0x0 bootloader.bin \
  0x8000 partition-table.bin \
  0x10000 RB02_Faruk_2.1.bin
```

### Full Build from Source
```bash
# Clone repository
git clone <your-repo-url>
cd ESP32-S3-Touch-LCD-2.8C-Test

# Checkout v1.5
git checkout aa95fd6

# Build
idf.py build

# Flash
idf.py -p COM8 flash monitor
```

---

## Files in This Release

| File | Size | Description |
|------|------|-------------|
| `RB02_Faruk_2.1.bin` | ~1.6 MB | Main application binary |
| `bootloader.bin` | ~27 KB | ESP32-S3 bootloader |
| `partition-table.bin` | ~3 KB | Partition table |
| `RB02_Faruk_2.1.elf` | ~16 MB | ELF file with debug symbols |
| `RB02_Faruk_2.1.map` | ~2 MB | Memory map for debugging |

---

## Configuration

### NVS Settings Preserved
All existing settings are preserved during upgrade:
- GPS baudrate setting
- Bluetooth enable/disable
- QNH calibration values
- Gyro calibration data

### New Settings
After upgrade, new Panel Alignment settings default to 0° (no adjustment).

---

## Hardware Compatibility

**Tested on:**
- ✅ ESP32-S3-Touch-LCD-2.1 (Waveshare Round Display)

**Expected to work:**
- ESP32-S3-Touch-LCD-2.8 (with appropriate BuildMachine.h config)

**Required Peripherals:**
- BMP280 barometer (I2C address 0x76)
- QMI8658 IMU (I2C address 0x6B)
- PCF85063 RTC (I2C address 0x51)
- TCA9554 GPIO expander (I2C address 0x20)
- CST820 touch controller (2.1") or GT911 (2.8")

**Optional:**
- NEO-6M GPS module (UART GPIO43/44, default 9600 baud)
- SD card for maps/checklists

---

## Known Issues

### None
All known issues from v1.4 and upstream merge resolved.

### GPS Fix Acquisition
- NEO-6M requires clear sky view for first fix (may take several minutes)
- Blue LED on module blinks when fix acquired
- External antenna strongly recommended for aircraft use

---

## Upgrade Path

### From v1.4 → v1.5
✅ **Safe upgrade** - No breaking changes
✅ **Settings preserved** - NVS data compatible
✅ **New features added** - Panel Alignment sliders available immediately

### From v1.3 or earlier → v1.5
⚠️ **Full flash recommended** (all three files)
ℹ️ May need to recalibrate gyro after upgrade
ℹ️ QNH and GPS settings preserved

---

## Documentation

- **Upstream Merge Details:** `UPSTREAM_MERGE_2025-10-03.md`
- **v1.4 Boot Optimization:** `BOOT_OPTIMIZATION_v1.4.md`
- **v1.4 Release Notes:** `RELEASE_NOTES_v1.4.md`
- **Project Documentation:** `DOKUMENTATION.md`
- **Code Optimization Analysis:** `OPTIMIERUNG.md`

---

## Version History

**v1.5** (2025-10-03) - Upstream Merge
- Panel Alignment system (Pitch/Roll/Yaw sliders)
- I2C debug scan functionality
- 6 post-merge bug fixes
- Settings page stability improvements

**v1.4** (2025-10-02) - Boot Optimization
- 100% boot success rate
- I2C retry logic with 2000ms timeout
- GPS UART early initialization
- LCD timing improvements
- PSRAM memtest disabled

**v1.3** - Caching Optimization
- Attitude display caching
- Performance improvements

---

## Support & Feedback

**Issues:** Report at project repository
**License:** GNU AGPLv3 / Dual commercial licensing
**Copyright:** 2024 XIAPROJECTS SRL

---

## Build Information

**Compiler:** ESP-IDF v5.5.1
**Toolchain:** xtensa-esp32s3-elf-gcc (GCC) 13.2.0
**Build Date:** October 3, 2025
**Git Hash:** aa95fd6
**ELF SHA256:** c3e7eb16c...

---

**Happy Flying! 🛩️**
