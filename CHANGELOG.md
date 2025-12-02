# CHANGELOG - Industrial Power Monitor

All notable changes to this project will be documented in this file.

## [v3.0.0] - 2025-12-03

### ☁️ Major Update - HTTP API Integration & System Stability

This release transforms the device from a local logger to a cloud-connected IoT device. It also addresses critical stability issues (heap corruption, race conditions) and optimizes performance for dual-core operation.

---

## ✨ New Features

### 1. HTTP API Logging (Replaces SD Card)
- **Async Logging**: Data transmission runs on Core 0 (Network Task) without blocking UI (Core 1).
- **Queue Buffering**: 50-item FreeRTOS queue prevents data loss during network glitches.
- **Configurable Endpoint**: Set API URL via serial command `SET_API <url>`.
- **Fail-Fast**: 4s timeout and smart retry logic to prevent system freeze.

### 2. Dual Core Architecture
- **Core 1 (App Core)**: Dedicated to UI rendering and user interaction (100% responsive).
- **Core 0 (Pro Core)**: Handles WiFi, HTTP requests, and sensor data acquisition.
- **Thread Safety**: Implemented Mutexes for `sysData` and I2C bus protection.

### 3. Network Configuration
- **New Commands**:
  - `SET_WIFI <ssid> <pass>`: Configure WiFi credentials.
  - `SET_API <url>`: Configure backend endpoint.
- **Persistent Storage**: Credentials saved in NVS (Preferences).

### 4. Memory Optimization
- **Safe Strings**: Replaced global `String` objects with `char` arrays to prevent heap fragmentation.
- **Stack Safety**: Increased `loggingTask` stack to 12KB for safe HTTPS SSL handshakes.
- **Efficient Sorting**: Switched to Insertion Sort for faster ADC noise filtering.

---

## 🐛 Critical Bug Fixes

### 1. Heap Corruption (Device ID)
- **Issue**: Passing `String` objects through `xQueueSend` caused double-free errors and reboots.
- **Fix**: Replaced `String` with fixed-size `char[32]` in `LogItem` struct.

### 2. Race Conditions (Data Tearing)
- **Issue**: UI read data while Sensor Task was writing it, causing glitchy values.
- **Fix**: Implemented `sysDataMutex` to ensure atomic access to shared data.

### 3. Retry Storm (Blank Charts)
- **Issue**: Queue full caused infinite retry loop, starving the UI task.
- **Fix**: Updated logic to drop old data when queue is full, preserving system responsiveness.

### 4. I2C Bus Contention
- **Issue**: RTC accessed by multiple tasks simultaneously.
- **Fix**: Implemented `i2cMutex` to serialize access to the DS3231 RTC.

---

## [v2.0.0] - 2025-11-26

### 🎉 Major Update - Complete UI Overhaul & Error Handling

This release brings significant improvements to user experience, error handling, and display aesthetics.

---

## ✨ New Features

### 1. SD Card Error Display System (Apple-Style UI)
**Location**: Lines 325-433

- ✅ Added `drawSDError()` function with Apple-inspired error UI
- 5 error codes implemented:
  - **E-01**: Card Missing (SD not detected at start)
  - **E-02**: Write Failed (file creation failed)
  - **E-03**: Save Error (write failed during logging)
  - **E-04**: File Lost (file handle lost during logging)
  - **E-05**: Save Incomplete (file empty on stop)
- Visual elements:
  - 3D SD card icon with dark gray inactive state
  - Animated red alert badge with X mark
  - Glass-style info card with error details
  - Footer with troubleshooting hint

### 2. Consistent 3D Page Numbers
**Location**: Lines 439-475

- ✅ Added global `drawPageNumber(int page)` function
- 3D button-style indicators with:
  - Light gray face (C_POLE)
  - Dark shadow effect (4px depth)
  - White border highlights
  - Centered black text
- Applied to all 4 slides for consistency

### 3. System Diagnostic at Boot
**Location**: Lines 515-643

- ✅ Added `runSystemCheck()` function
- Runs automatically on every ESP32 power-on
- Features:
  - 3D rounded panel with metallic header
  - Animated scanning dots for each component
  - Checks 4 hardware components:
    1. RTC DS3231
    2. SD Card
    3. Current Sensor (calibration status)
    4. Voltage Sensor (calibration points)
  - 3D success/fail icons (green/red buttons)
  - Final status footer (green "SYSTEM READY" or red "CHECK FAILED")
  - 2-second hold before continuing

### 4. Serial Monitor Feedback System
**Location**: Lines 1397-1499

- ✅ Complete command feedback with status symbols (✓, ✗, ⚠)
- Echo all input with delimiter boxes
- Detailed execution status for every command
- Added 2 new commands:
  - **`help`** or **`?`**: Display all available commands
  - **`status`**: Show comprehensive system status
- Error handling for unknown commands with helpful hints
- Progress tracking for multi-step commands (e.g., voltage calibration 1/3)

### 5. Industrial-Style Dashboard (Slide 3)
**Location**: Lines 1109-1255

- ✅ Complete redesign with tech panel aesthetics
- Visual elements:
  - Metallic header bar with "SYSTEM LOG" title
  - Green LED "Live" indicator
  - 2×4 grid layout with 8 data panels
  - 3D tech panels with:
    - Dark gray face (0x2124)
    - 4px shadows
    - Colored accent bars (matches data type)
    - Screw decorations (industrial theme)
- Helper functions:
  - `drawTechPanel()`: Draws 3D panel with accent
  - `printPanelValue()`: Smart value formatting
- Auto-adjust decimals (remove if value >= 1000)

### 6. Voltage Display Smoothing
**Location**: Lines 182-184, 1915-1924

- ✅ Exponential moving average to reduce decimal flickering
- Smoothing factor: alpha = 0.15 (balance smooth/responsive)
- Formula: `displayVoltage = (0.15 × raw) + (0.85 × previous)`
- Applied to all slides (1, 2, 3)
- Raw data remains accurate for logging

---

## 🐛 Bug Fixes

### 1. Double Display Issue (Success → Error)
**Location**: Lines 1376-1407

**Problem**: When SD card ejected mid-logging, showed success animation then error
**Fix**: 
- Added validation before showing success UI
- Check `fileSize > 0` and `sdReady == true`
- Show E-05 error if validation fails
- Prevents confusing double display

### 2. SD Error Detection Speed
**Location**: Lines 2155-2197

**Problem**: SD errors took up to 1 second to detect
**Fix**:
- Reduced check interval: 1000ms → 200ms (5x faster!)
- Implemented priority flag system:
  - Set flag when error detected (non-blocking)
  - Display at top of next loop iteration (priority #1)
  - Auto-restore slide after error display
- Average detection time: ~200ms vs 1000ms before

### 3. LCD White Flash on Boot
**Location**: Lines 2105-2129

**Problem**: LCD shows white screen for ~1000ms on power-on
**Fix**:
- Reordered initialization: Display init FIRST (before Serial)
- Immediate black fill after tft.init()
- Double fillScreen() for stubborn displays
- Reduced visible flash: 1000ms → ~15ms (98.5% improvement!)

---

## 🔧 Improvements

### 1. Priority Error Display System

Added global flags for immediate error display:
```cpp
bool sdErrorOccurred = false;
String sdErrorCode = "";
String sdErrorMsg = "";
```

Error handling flow:
1. Error detected → Set flag (non-blocking)
2. Next loop iteration → Check flag FIRST (priority)
3. Display error → Restore previous slide
4. User sees error almost immediately

### 2. Chart Data Persistence
**Location**: Lines 623-649, 795-821

**Problem**: Charts went blank after multiple slide transitions
**Fix**:
- Modified `begin()` to check if history arrays exist
- Only initialize arrays if `nullptr`
- Preserve data across slide changes
- Sprite deleted/recreated but data persists

### 3. System Check Card UI Enhancement

User improvements to diagnostic display:
- Rounded corners instead of sharp edges
- Floating pill-style footer
- Tighter spacing for better fit
- Smoother animations (100ms delays)
- Better visual hierarchy

---

## 📊 Statistics

### Code Changes
- **Total Lines Added**: ~600
- **Total Lines Modified**: ~200
- **Total Lines Removed**: ~100
- **Net Change**: +500 lines

### Performance Improvements
| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| SD Error Detection | 1000ms | 200ms | 5x faster |
| LCD Boot Flash | ~1000ms | ~15ms | 98.5% faster |
| Voltage Flicker | High | Low | Smoothed |
| User Feedback | None | Comprehensive | 100% coverage |

### New Files/Functions
- `drawSDError()` - SD card error display
- `drawPageNumber()` - 3D page indicators
- `drawResultIcon()` - System check icons
- `runSystemCheck()` - Boot diagnostic
- `drawTechPanel()` - Industrial panel helper
- `printPanelValue()` - Smart value printer

---

## 🎨 Visual Improvements

### Color Palette Consistency
- Apple-style error red: 0xF965
- Panel dark gray: 0x2124
- Shadow dark: 0x10A2
- Accent colors match data types

### 3D Effects
- All panels now have consistent 3D shadows
- 4px depth for unified appearance
- Rounded corners where appropriate
- Screw/LED decorations for industrial theme

---

## 🧪 Testing Performed

### Error Handling
- ✅ E-01: SD missing at boot
- ✅ E-02: Write-protected SD
- ✅ E-03: SD ejected during logging
- ✅ E-04: File corruption scenario
- ✅ E-05: Stop with empty file

### Serial Commands
- ✅ All existing commands with feedback
- ✅ `help` displays command list
- ✅ `status` shows system info
- ✅ Unknown commands handled gracefully

### Display
- ✅ Boot sequence clean (no white flash)
- ✅ System check displays correctly
- ✅ All 4 slides render properly
- ✅ Page numbers consistent
- ✅ Chart data persists across slides

### Smoothing
- ✅ Voltage decimals smooth at all noise levels
- ✅ Responsiveness maintained
- ✅ Logging data accuracy verified

---

## 📝 Known Issues

### Minor
- System check delay adds ~4 seconds to boot (acceptable for diagnostic value)
- Very rapid SD card removal might still show brief success animation

### Future Improvements
See README.md → TODO section for planned enhancements

---

## 🔄 Migration Guide

### From v1.x to v2.0

No breaking changes! Simply upload the new code.

**What stays the same**:
- All pin configurations
- SD card logging format
- Calibration data (preserved in Preferences)
- Serial command interface (enhanced, not changed)

**What's new**:
- System check on boot (automatic)
- Better error messages
- Smoother voltage display
- New dashboard design

---

## 📖 Documentation

Full walkthroughs available in `.gemini/antigravity/brain/` directory:
- `walkthrough.md` - Initial SD error & page numbers
- `serial_feedback_walkthrough.md` - Command feedback system
- `minor_revisions_walkthrough.md` - SD error handling improvements
- `ui_improvements_walkthrough.md` - Voltage smoothing & Slide 3 redesign
- `lcd_flash_fix.md` - Boot white flash solution

---

## 🙏 Credits

- **Developer**: Achmad Nawawi Ahlan
- **AI Assistant**: Google Gemini (Antigravity Agent)
- **Libraries**: TFT_eSPI, RTClib, SD, Preferences

---

## 📅 Previous Versions

### [v1.0.0] - 2025-11-25
- Initial SD card initialization fix
- Retry mechanism (3 attempts)
- SPI speed optimization (4MHz → 1MHz)
- Power-on delays (500ms + 200ms)
- Verbose diagnostics

---

**Full Changelog**: https://github.com/yourusername/industrial-power-monitor/compare/v1.0.0...v2.0.0