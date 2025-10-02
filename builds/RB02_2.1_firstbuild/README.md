# RB02 2.1" First Build

**Build Date:** 2025-10-02
**Display:** ESP32-S3-Touch-LCD-2.1"
**Git Tag:** RB02_2.1_firstbuild

## Configuration
- Display Size: 2.1" Touch
- All GPS avionics features enabled
- LVGL FS_STDIO disabled (fixed compilation issue)

## Flash Instructions
Flash with ESP-IDF or esptool.py:

```bash
# Using esptool.py
esptool.py -p <PORT> -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 bootloader.bin 0x10000 application.bin 0x8000 partition-table.bin

# Or using idf.py from project root
idf.py -p <PORT> flash
```

## Files
- `application.bin` - Main application (0x10000)
- `bootloader.bin` - Bootloader (0x0)
- `partition-table.bin` - Partition table (0x8000)
- `BuildMachine.h` - Build configuration snapshot

## Features Enabled
- GPS, Map, Speed, Advanced Attitude, Altimeter
- Turn Coordinator, Track, Variometer, G-Meter, Clock
- Checklist, Console, Vibration Test
