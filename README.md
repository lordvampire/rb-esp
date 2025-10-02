# RB-Avionics - Optimized Fork (RB02_Faruk_2.1)

This is an optimized fork of [xiaprojects/rb-esp](https://github.com/xiaprojects/rb-esp) focused on performance improvements, resource optimization, and enhanced stability for the ESP32-S3 2.1" Round Touch Display.

**Base Project:** RB-02 - Display with SixPack Aviation Instruments

## 🎯 Project Goals

This fork aims to systematically optimize the RB-Avionics codebase with support from Claude Code AI to:

1. **Reduce CPU usage** - Eliminate inefficient algorithms and unnecessary computations
2. **Optimize memory management** - Prevent heap fragmentation and reduce RAM usage
3. **Improve stability** - Add safety mechanisms and fix critical bugs
4. **Enhance accuracy** - Correct sensor fusion and calculation errors
5. **Maintain compatibility** - Keep all original features working

## ✅ Completed Optimizations

### Critical (High Impact)

- [x] **UART Static Buffer** (`main/RB/RB02.c`)
  - Replaced malloc/free in loop with static buffer
  - **Impact:** ~8% CPU reduction, eliminated heap fragmentation
  - Status: ✅ Tested & Working

- [x] **Attitude Matrix Update Threshold** (`main/RB/RB02_AAttitude.c`)
  - Skip matrix recalculation if pitch/roll change < 0.1°
  - **Impact:** ~10% CPU reduction
  - Status: ✅ Tested & Working

- [x] **Madgwick Sample Rate Fix** (`main/QMI8658/QMI8658.c`)
  - Corrected sample frequency calculation from DriverLoopMilliseconds
  - **Impact:** Critical accuracy fix for attitude estimation
  - Status: ✅ Tested & Working

**Total Achieved:** ~18% CPU reduction + stability improvements

## 📋 Planned Optimizations

### High Priority (Next Sprint)

- [ ] **Task Watchdog** - Safety monitoring for critical tasks
  - Requires careful implementation to avoid boot issues
  - Priority: High (Safety Critical)
  - Estimated Impact: Detect hung tasks, prevent crashes

- [ ] **Tab Resource Cleanup** - Prevent memory leaks on tab switching
  - GPS Map and Advanced Attitude cleanup callbacks
  - Priority: High (Prevents RAM exhaustion)
  - Estimated Impact: 500KB-2MB RAM savings

- [ ] **String Formatting Cache** - Reduce sprintf() calls
  - Cache formatted values with change threshold
  - Priority: Medium-High
  - Estimated Impact: 2-4% CPU reduction

- [ ] **GPS Map Async Loading** - Non-blocking SD card tile loading
  - Background task for tile loading
  - Priority: Medium (UX improvement)
  - Estimated Impact: Eliminates UI freezes (50-200ms)

### Medium Priority (Future)

- [ ] **Floating-Point Optimizations** - Replace divisions with multiplications
- [ ] **Code Deduplication** - Base instrument class to reduce duplicate code
- [ ] **Task Priority Optimization** - Proper core affinity and priorities
- [ ] **Magic Numbers Cleanup** - Replace hardcoded values with named constants
- [ ] **Error Handling Unification** - Centralized error logging system

### Low Priority (Code Quality)

- [ ] **Static Analysis Integration** - cppcheck, clang-tidy
- [ ] **Unit Tests** - Critical calculations (Madgwick, BMP280)
- [ ] **Documentation** - Function headers and inline docs
- [ ] **CI/CD Pipeline** - Automated builds and testing

## 📊 Performance Metrics

| Metric | Before | Current | Target |
|--------|--------|---------|--------|
| CPU Usage | ~70-80% | ~52-62% | <50% |
| Heap Fragmentation | High | Eliminated | Stable |
| Attitude Accuracy | Drifting | Corrected | ±0.1° |
| RAM Usage | Unknown | Baseline | -20% |

## 🚀 Quick Start

### Hardware Requirements

- **Display:** ESP32-S3-Touch-LCD-2.1 (Waveshare 2.1" Round Touch)
- **Sensors:**
  - BMP280 Barometer
  - QMI8658 IMU (6-axis)
  - GPS Module (NMEA, e.g., uBlox NEO M6N)
  - PCF85063 RTC
- **Storage:** Micro SD Card (up to 32GB)
- **Optional:** Stratux BLE for traffic

### Flash Pre-Built Binary

Latest optimized build: [`builds/v1.0-optimized-RB02_Faruk_2.1-2025-10-02/`](builds/v1.0-optimized-RB02_Faruk_2.1-2025-10-02/)

```bash
esptool.py --chip esp32s3 --baud 921600 write_flash -z \
  0x0 builds/v1.0-optimized-RB02_Faruk_2.1-2025-10-02/bootloader.bin \
  0x8000 builds/v1.0-optimized-RB02_Faruk_2.1-2025-10-02/partition-table.bin \
  0x10000 builds/v1.0-optimized-RB02_Faruk_2.1-2025-10-02/RB02_Faruk_2.1.bin
```

**Flasher Settings:**
- Chip: esp32s3
- Flash mode: DIO
- Flash size: 16MB

### Build from Source

```bash
# Install ESP-IDF 5.5.1+
idf.py menuconfig  # Configure if needed
idf.py build
idf.py -p <PORT> flash monitor
```

## 📖 Documentation

- **[DOKUMENTATION.md](DOKUMENTATION.md)** - Complete technical documentation (German)
- **[OPTIMIERUNG.md](OPTIMIERUNG.md)** - Detailed optimization analysis (German)
- **[OPTIMIZATION_CHANGELOG.md](OPTIMIZATION_CHANGELOG.md)** - Build history and changes (English)
- **[CLAUDE.md](CLAUDE.md)** - Project instructions for Claude Code AI

## 🎨 Features

All original RB-02 features are preserved:

1. Customizable splash screen
2. GPS Speed indicator (KMH/KT)
3. GPS-assisted attitude indicator
4. Advanced attitude indicator with sky tiles
5. Altimeter with QNH settings
6. Digital altimeter with GPS cross-reference
7. Slip & Turn indicator
8. GPS-assisted directional gyro
9. GPS Map (BMP tile-based, MapBox compatible)
10. Variometer
11. G-Meter with peak hold
12. Timers with GPS UTC sync
13. SD card checklists
14. Engine hour meter
15. Stratux BLE traffic radar
16. Traffic list display
17. Flight data logger

## 🛠️ Development

### Next Optimization Decision

**Question:** Should we tackle the Task Watchdog next, or focus on other optimizations?

**Watchdog Pros:**
- ✅ Safety-critical for aviation application
- ✅ Detects hung tasks and sensor failures
- ✅ Prevents silent crashes

**Watchdog Cons:**
- ❌ Previously caused boot failure
- ❌ Requires careful timing implementation
- ❌ Needs thorough testing

**Alternative Options:**
1. **Tab Cleanup** - High impact RAM savings, lower risk
2. **String Cache** - Quick win, 2-4% CPU reduction
3. **GPS Async Loading** - Immediate UX improvement

**Decision:** TBD - See discussion below

## 🤝 Contributing

This is a personal optimization project, but suggestions and discussions are welcome via Issues.

## 📜 License

This project inherits the dual licensing from the original:
1. GNU AGPLv3 (Community/Personal Use)
2. Commercial licensing available

## 🙏 Credits

- **Original Project:** [xiaprojects/rb-esp](https://github.com/xiaprojects/rb-esp)
- **Optimization Support:** [Claude Code](https://claude.com/claude-code) by Anthropic
- **Community:** RB-Avionics Discord and Wiki

## 📺 Videos

- [Original Demo 1](https://www.youtube.com/watch?v=9aCub_4hdxs)
- [Original Demo 2](https://www.youtube.com/watch?v=8FwM3qiaDVQ)
- [Original Demo 3](https://www.youtube.com/watch?v=Gy3Yu2zPZRM)

---

**Latest Build:** v1.0-optimized-RB02_Faruk_2.1 (October 2, 2025)
**Status:** ✅ Stable, 18% CPU improvement achieved
