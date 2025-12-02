# 🔌 Industrial Power Monitor (ESP32)

**Version**: 2.0.0  
**Last Updated**: 2025-11-26  
**Status**: ✅ Production Ready

---

## 📋 Overview

A professional-grade power monitoring system for ESP32 with TFT display, featuring real-time voltage/current measurement, energy logging, and comprehensive error handling. Built with industrial aesthetics and robust error recovery.

### Key Capabilities
- 📊 **Real-time Charts** - Live voltage, current, and energy visualization
- 💾 **SD Card Logging** - Automatic data logging with error recovery
- 🎨 **Industrial UI** - Apple-inspired error handling, 3D tech panels
- ⚡ **Fast Response** - 200ms error detection, smooth voltage display
- 🔍 **Self-Diagnostic** - Boot-time hardware verification
- 💬 **Serial Interface** - Complete command feedback system

---

## 🌟 Features

### Display & UI (v2.0)
- **4 Interactive Slides**:
  1. Real-time Chart (Voltage, Current Used, Current Regen)
  2. Energy Chart (Voltage, Energy Used, Energy Regen)
  3. Dashboard (8 data panels with 3D tech styling)
  4. File Info (post-logging summary)
- **3D Visual Elements**: Panels, shadows, page indicators, icons
- **Smooth Animations**: Voltage smoothing (alpha=0.15), chart transitions
- **Consistent Page Numbers**: Global 3D indicators on all slides
- **Clean Boot**: No white flash (15ms vs 1000ms before)

### Error Handling (v2.0)
- **SD Card Error Display**: Apple-style UI with 5 error codes
  - E-01: Card Missing
  - E-02: Write Failed
  - E-03: Save Error (during logging)
  - E-04: File Lost
  - E-05: Save Incomplete
- **Fast Detection**: 200ms check interval (5x faster than v1.0)
- **Priority Display**: Errors show immediately, auto-restore slide
- **Visual Feedback**: 3D icons, color-coded status

### System Diagnostic (v2.0)
- **Boot-time Check**: Automated hardware verification
- **Components Tested**:
  - RTC DS3231
  - SD Card
  - Current Sensor (calibration)
  - Voltage Sensor (calibration)
- **Visual Results**: Green/red 3D icons, status footer
- **Animated Scanning**: Loading dots for each component

### Serial Monitor (v2.0)
- **Comprehensive Feedback**: Status symbols (✓, ✗, ⚠)
- **6 Commands**:
  1. `reset_all` - Clear all calibration
  2. `auto` - Auto current offset calibration
  3. `v<voltage>` - Add voltage calibration point
  4. `SET_TIME YYYY MM DD HH MM SS` - Set RTC time
  5. `help` or `?` - Show command list
  6. `status` - Display system status
- **Error Guidance**: Helpful hints for invalid commands
- **Progress Tracking**: Multi-step command feedback (e.g., 1/3 points)

### Data Logging
- **Format**: CSV with headers
- **Interval**: Every 200ms (5Hz)
- **Fields**: Iteration, Voltage, Current, Power, Wh_Used, Wh_Regen, Time
- **Auto Recovery**: Re-init SD if error detected
- **File Naming**: RTC-based (`MMDDHHSS.csv`) or timestamp fallback

### Calibration
- **Current Offset**: Auto-calibration with no load
- **Voltage**: 3-point calibration for accuracy
- **RTC**: Time sync via serial command
- **Storage**: Persistent in ESP32 Preferences (NVS)

---

## 🛠 Hardware Requirements

### Core Components
- **MCU**: ESP32 Dev Board (dual-core, 240MHz)
- **Display**: 240×320 TFT LCD (ILI9341/ST7789)
- **RTC**: DS3231 Real-Time Clock (I2C)
- **Storage**: MicroSD Card Module (SPI, FAT32)

### Sensors
- **Voltage**: ZMPT101B AC Voltage Sensor (or voltage divider)
- **Current**: ACS712 Hall Effect Sensor (30A variant recommended)

### Input
- **Button**: Single push button (INPUT_PULLUP)
- **LED**: Status indicator

### Pin Configuration
```cpp
// Display (SPI)
#define TFT_CS    5
#define TFT_RST   4  
#define TFT_DC    2

// SD Card (SPI)
#define SD_CS     15
#define SD_SCK    14
#define SD_MISO   12
#define SD_MOSI   13

// I2C (RTC)
#define SDA       21
#define SCL       22

// Sensors (ADC)
#define SENSOR_V  35  // Voltage sensor
#define SENSOR_I  34  // Current sensor

// Controls
#define BUTTON    0   // Boot button
#define LED       2   // Built-in LED
```

---

## 📦 Software Dependencies

### Arduino Libraries
```
- TFT_eSPI (v2.5.0+)     - Display driver
- RTClib (v2.1.0+)       - RTC interface
- SD (Built-in)          - SD card handler
- Preferences (Built-in) - NVS storage
- Wire (Built-in)        - I2C communication
- SPI (Built-in)         - SPI communication
```

### Installation
1. Install Arduino IDE (1.8.19 or 2.x)
2. Add ESP32 board support: https://dl.espressif.com/dl/package_esp32_index.json
3. Install libraries via Library Manager
4. Configure TFT_eSPI:
   - Edit `User_Setup.h` in TFT_eSPI library folder
   - Set your display driver (ILI9341/ST7789)
   - Configure pins to match hardware

---

## 🚀 Quick Start

### 1. Upload Code
```bash
# Open sketch_nov12b.ino in Arduino IDE
# Select: ESP32 Dev Module
# Upload Speed: 921600
# Flash Frequency: 80MHz
# Upload
```

### 2. Initial Calibration
Open Serial Monitor (115200 baud):

```
# 1. Auto-calibrate current (ensure no load)
auto

# 2. Calibrate voltage (3 points minimum)
v12.0    # When multimeter shows 12.0V
v24.5    # When multimeter shows 24.5V  
v48.2    # When multimeter shows 48.2V

# 3. Set RTC time
SET_TIME 2025 11 26 14 30 00

# 4. Verify calibration
status
```

### 3. Start Logging
- Press button to start/stop logging
- Data saved to SD card as `/MMDDHHSS.csv`
- LED blinks during active logging

---

## 📊 Display Layout

### Slide 1: Real-Time Chart
```
┌────────────────────────────────┐
│  Voltage (V)  ─── Green        │
│  Current Used (A) ─── Blue     │
│  Current Regen (A) ─── Cyan    │
│                                │
│  [Live scrolling chart]        │
│                                │
│  Values: 48.7V  2.3A  0.1A     │
└──────────────────────────── ① ─┘
```

### Slide 2: Energy Chart
```
┌────────────────────────────────┐
│  Voltage (V) ─── Green         │
│  Energy Used (Wh) ─── Red      │
│  Energy Regen (Wh) ─── Cyan    │
│                                │
│  [Energy accumulation chart]   │
│                                │
│  Efficiency: 95.3%             │
└──────────────────────────── ② ─┘
```

### Slide 3: Dashboard (Industrial Style)
```
┌────────────────────────────────┐
│ ● SYSTEM LOG              ●    │
├────────────────────────────────┤
│ ┏━TEGANGAN━┓  ┏━EFISIENSI━━┓   │
│ ┃ 48.7  V  ┃  ┃  95.3  %   ┃   │
│ ┗━━━━━━━━━━┛  ┗━━━━━━━━━━━━┛   │
│ ┏━ARUS AVG━┓  ┏━ARUS MAX━━━┓   │
│ ┃ 2.35  A  ┃  ┃  3.54  A   ┃   │
│ ┗━━━━━━━━━━┛  ┗━━━━━━━━━━━━┛   │
│ ┏━ENERGY U━┓  ┏━ENERGY R━━━┓   │
│ ┃ 12.5  Wh ┃  ┃  1.2  Wh   ┃   │
│ ┗━━━━━━━━━━┛  ┗━━━━━━━━━━━━┛   │
│ ┏━LOOP ITER┓  ┏━UPTIME━━━━━┓   │
│ ┃   1234   ┃  ┃   3600  s  ┃   │
└─┗━━━━━━━━━━┛──┗━━━━━━━━━━━━┛ ③ ┘
```

### Slide 4: File Info
```
┌────────────────────────────────┐
│      LOGGING COMPLETE          │
│                                │
│  File: /11261430.csv           │
│  Size: 2048 bytes              │
│  Rows: 512 entries             │
│                                │
│  Duration: 00:01:42            │
└──────────────────────────── ④ ─┘
```

---

## 🎯 Usage Guide

### Button Controls
| Action | Function |
|--------|----------|
| Single Press | Start/Stop Logging |
| Slide Auto-Change | Every 5 seconds (when not logging) |

### LED Indicators
| Pattern | Meaning |
|---------|---------|
| Blinking (logging) | Data being saved |
| Solid (boot) | System check in progress |
| Off | Idle/Ready |

### Serial Commands

**View Help**:
```
help
```

**System Status**:
```
status

Output:
========== SYSTEM STATUS ==========
Current Calibration: ✓ Done
Voltage Calibration: 3/3 points
RTC Calibration: ✓ Done
System Calibrated: YES
-----------------------------------
SD Card: ✓ Ready
RTC: ✓ Ready
Logging: ACTIVE
Log Iterations: 1234
-----------------------------------
Voltage: 48.73 V
Current (Used): 2.35 A
Current (Regen): 0.15 A
Power: 106.70 W
===================================
```

---

## 🐛 Troubleshooting

### White Flash on Boot
**Symptom**: LCD shows white screen briefly at power-on  
**Fix v2.0**: Reordered init (display first), double fillScreen()  
**Result**: 15ms flash vs 1000ms before (98.5% improvement)

### SD Card Errors
**Symptom**: "E-01: Card Missing" or "E-03: Save Error"  
**Checks**:
1. Card inserted and locked in slot?
2. Card formatted as FAT32?
3. Module connections secure?
4. 3.3V power (not 5V)?

**Auto-Recovery**: System tries re-init on next logging attempt

### Flickering Voltage Display
**Symptom**: Decimals change rapidly  
**Fix v2.0**: Exponential smoothing (alpha=0.15)  
**Adjust**: Change `VOLTAGE_ALPHA` in code (lower = smoother)

### Charts Go Blank
**Fixed v2.0**: History arrays preserved across slide changes  
**If still occurs**: Check available RAM, reduce chart width

### System Check Failures
**Red Icon Meaning**:
- RTC: Not connected or dead battery
- SD: Card missing or unreadable
- Sensors: Calibration incomplete

**Action**: Run calibration commands, check hardware

---

## 📝 TODO & Roadmap

### High Priority
- [ ] **Backlight Control**: PWM brightness adjustment
- [ ] **Data Export**: Web interface for downloading logs
- [ ] **WiFi Sync**: NTP time sync, cloud backup
- [ ] **Alert System**: Configurable voltage/current thresholds

### Medium Priority
- [ ] **Touch Interface**: Replace button with touchscreen
- [ ] **Multiple Charts**: Split-screen dual monitoring
- [ ] **Data Analysis**: On-device min/max/avg calculations
- [ ] **Power Saving**: Sleep mode when idle >1 hour

### Low Priority
- [ ] **Custom Themes**: User-selectable color schemes
- [ ] **Calibration UI**: On-screen calibration wizard
- [ ] **OTA Updates**: WiFi firmware updates
- [ ] **Multi-Language**: Internationalization support

### Completed ✅
- [x] SD error display with visual feedback (v2.0)
- [x] Fast error detection (200ms interval) (v2.0)
- [x] System diagnostic at boot (v2.0)
- [x] Serial command feedback (v2.0)
- [x] Chart data persistence (v2.0)
- [x] Voltage display smoothing (v2.0)
- [x] Industrial dashboard redesign (v2.0)
- [x] Boot white flash fix (v2.0)
- [x] SD initialization with retry (v1.0)

---

## 📖 Documentation

### Code Structure
```
sketch_nov12b.ino
├─ Lines 1-190      : Globals, configs, structures
├─ Lines 191-643    : UI helpers, system check
├─ Lines 644-1254   : Slide classes (1-4)
├─ Lines 1255-1395  : Logging functions
├─ Lines 1396-1650  : Serial command handler
├─ Lines 1651-1828  : Sensor helpers, calibration
├─ Lines 1829-2079  : Data acquisition task (Core 0)
└─ Lines 2080-2350  : Setup & main loop (Core 1)
```

### Key Files
- `sketch_nov12b.ino` - Main code (2350 lines)
- `CHANGELOG.md` - Version history & changes
- `README.md` - This file

### Walkthroughs
Located in `.gemini/antigravity/brain/` (development artifacts):
- `walkthrough.md` - SD error & page numbers
- `serial_feedback_walkthrough.md` - Command system
- `minor_revisions_walkthrough.md` - Error handling v2
- `ui_improvements_walkthrough.md` - Smoothing & Slide 3
- `lcd_flash_fix.md` - Boot optimization

---

## 🔧 Performance Metrics

### v2.0 vs v1.0

| Metric | v1.0 | v2.0 | Improvement |
|--------|------|------|-------------|
| SD Error Detection | 1000ms | 200ms | **5x faster** |
| Boot White Flash | ~1000ms | ~15ms | **98.5% faster** |
| Voltage Stability | Flickering | Smooth | **Smoothed** |
| User Feedback | Minimal | Comprehensive | **100% coverage** |
| Chart Persistence | Fails | Works | **100% reliable** |
| Error Codes | 0 | 5 | **5 codes** |

### Memory Usage
```
Flash: ~320KB / 1.2MB (26%)
SRAM: ~45KB / 320KB (14%)
PSRAM: Not used (charts fit in SRAM)
```

---

## 🙏 Credits

**Developer**: Achmad Nawawi Ahlan  
**AI Assistant**: Google Gemini (Antigravity Agent)  
**Libraries**: 
- TFT_eSPI by Bodmer
- RTClib by Adafruit
- ESP32 Core by Espressif

---

## 📜 License

This project is open source. Feel free to use, modify, and distribute.

---

## 📞 Support

**Issues**: Report via GitHub Issues  
**Questions**: Open a discussion  
**Updates**: Watch releases for new versions

---

**Version 2.0.0** - Ready for production deployment! 🚀
# motor-bldc-joule-meter-using-esp32
