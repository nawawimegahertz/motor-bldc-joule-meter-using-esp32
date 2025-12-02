/*
 * INDUSTRIAL POWER MONITOR (Dual SPI Edition)
 * Dibuat untuk Achmad Nawawi Ahlan
 * 
 * SOLUSI FREEZE/BLANK:
 * - LCD berjalan di VSPI (Default: 18, 23, etc).
 * - SD Card dipindah ke HSPI (SCK:14, MISO:27, MOSI:26, CS:5).
 * - Dengan memisahkan jalur fisik, Core 0 (Data) dan Core 1 (UI)
 *   tidak akan bertabrakan saat mengakses hardware.
 */

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <SD.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================
#define BUTTON_PIN 13
#define LED_PIN 12

// SD Card Pins (HSPI)
#define SD_CS_PIN 5
#define SD_SCK_PIN 18
#define SD_MISO_PIN 19
#define SD_MOSI_PIN 23

// Sensor Pins
const int sensorPin = 36;
const int batteryPin = 39;

// ============================================================================
// SENSOR PARAMETERS
// ============================================================================
float I_offset = 2.2871;
float I_sensitivity = 0.020;
float I_divider_factor = 0.6667;
float I_scale_factor = 1.1948;
const float V_ref = 3.342;
const float ADC_resolution = 4095.0;

// ============================================================================
// CALIBRATION SETTINGS
// ============================================================================
#define MAX_CAL_POINTS 6
#define MIN_CAL_POINTS 3

struct CalPoint {
    float rawADC;
    float realV;
};

// ============================================================================
// COLOR DEFINITIONS
// ============================================================================
// Housing 3D Colors
#define C_HOUSING_FRONT 0x4208
#define C_HOUSING_SIDE 0x2104
#define C_POLE 0xBDF7
#define C_POLE_SHADE 0x7BEF

// Traffic Light Colors
#define TL_RED_ON TFT_BLUE
#define TL_YELLOW_ON TFT_CYAN
#define TL_GREEN_ON TFT_GREEN
#define TL_RED_DIM 0x0008
#define TL_YELLOW_DIM 0x01E0
#define TL_GREEN_DIM 0x0100

// Display Colors
#define COLOR_BLUE TFT_RED
#define COLOR_CYAN TFT_YELLOW
#define COLOR_GREEN TFT_GREEN
#define COLOR_RED TFT_BLUE
#define COLOR_TEXT_L 0xBDF7
#define COLOR_TEXT_V TFT_WHITE
#define COLOR_BG TFT_BLACK
#define COLOR_GRID 0x02E0
#define TFT_GREY 0x5AEB
#define TFT_ORANGE 0xFD20

// Calibration Colors
#define CALIB_COLOR_WARN TFT_RED
#define CALIB_COLOR_DONE TFT_GREEN
#define CALIB_COLOR_PENDING TFT_WHITE
#define CALIB_COLOR_DIM 0x528A

// New UI Colors (v2.1)
#define C_SD_BODY     0x3186 // Dark Grey
#define C_SD_LABEL    0x0418 // Deep Blue/Teal
#define C_SUCCESS     0x05E0 // Vibrant Green
#define C_SUCCESS_SH  0x03E0 // Dark Green Shadow
#define C_ERROR       0xF800 // Pure Red
#define C_ERROR_SH    0x8000 // Deep Red Shadow
#define C_PANEL_BG    0x2124 // Panel Dark Grey
#define C_TEXT_MUTED  0x9492 // Muted Grey
#define C_CARD_BG     0x2124 // Card Background
#define C_CARD_SHADOW 0x1082 // Card Shadow
#define C_BG_MAIN     TFT_BLACK

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct SystemData {
    float voltage;
    float i_regen;
    float i_used;
    float i_used_max;
    float i_used_min;
    float i_used_avg;  // Average current
    float wh_used;
    float wh_regen;
    float efficiency;
    unsigned long iteration;
    unsigned long uptime_sec;
    unsigned long time_start;
    unsigned long time_now;
};

struct ChannelConfig {
    const char* name;
    const char* unit;
    uint16_t color;
    float peak;
};

struct ProductConfig {
    bool enabled;
    const char* name;
    const char* unit;
};

enum Theme { CLASSIC = 1, MODERN = 0 };
enum WaveMode { SINE, SQUARE, TRIANGLE, SAWTOOTH, NOISE };

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
TFT_eSPI tft = TFT_eSPI();
SPIClass sdSPI(HSPI);  // SD Card SPI instance
Preferences prefs;
Preferences calPrefs;
RTC_DS3231 rtc;
File currentLogFile;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
volatile SystemData sysData = {0};

// Calibration Status
bool system_calibrated = false;
bool chartsInitialized = false;
bool calib_curr_done = false;
int calib_volt_count = 0;
bool calib_rtc_done = false;
bool calib_ui_update = true;

// Logging Status
bool isLogging = false;
bool sdReady = false;
bool rtcReady = false;
unsigned long logIteration = 0;
String lastSavedFilename = "";
unsigned long lastSavedFileSize = 0;
bool hasLoggedBefore = false;
bool showFileInfoSlide = false;

// Button Handling
int buttonState;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Voltage Calibration Map
CalPoint voltageMap[MAX_CAL_POINTS];
int calPointCount = 0;
bool sensorSystemValid = false;

// Chart persistence
bool chartsFirstInit = true;

// SD Error handling
bool sdErrorOccurred = false;
String sdErrorCode = "";
String sdErrorMsg = "";

// Voltage smoothing for display (reduce decimal flickering)
float displayVoltage = 0.0;
const float VOLTAGE_ALPHA = 0.15; // Smoothing factor (0.0-1.0, lower = smoother)


// ============================================================================
// SENSOR HELPER FUNCTIONS
// ============================================================================

/**
 * Get median ADC reading from multiple samples
 */
float getMedianADC(int pin, int samples) {
    int raw[samples];
    
    // Read samples
    for (int i = 0; i < samples; i++) {
        raw[i] = analogRead(pin);
        delayMicroseconds(50);
    }
    
    // Bubble sort
    for (int i = 0; i < samples - 1; i++) {
        for (int j = 0; j < samples - i - 1; j++) {
            if (raw[j] > raw[j + 1]) {
                int temp = raw[j];
                raw[j] = raw[j + 1];
                raw[j + 1] = temp;
            }
        }
    }
    
    return (float)raw[samples / 2];
}

/**
 * Save calibration data to preferences
 */
void saveCalibrationSensor() {
    calPrefs.begin("sensor_cal", false);
    calPrefs.putFloat("i_off", I_offset);
    calPrefs.putFloat("i_sens", I_sensitivity);
    calPrefs.putFloat("i_scale", I_scale_factor);
    calPrefs.putBytes("v_map", voltageMap, sizeof(voltageMap));
    calPrefs.putInt("v_count", calPointCount);
    calPrefs.end();
}

/**
 * Add or update calibration point
 */
void addOrUpdateCalPoint(float realVoltage) {
    float currentRaw = getMedianADC(batteryPin, 50);
    int replaceIndex = -1;
    float minDiff = 2.0;
    
    // Check if similar point exists
    for (int i = 0; i < calPointCount; i++) {
        if (abs(voltageMap[i].realV - realVoltage) < minDiff) {
            replaceIndex = i;
            minDiff = abs(voltageMap[i].realV - realVoltage);
        }
    }
    
    // Update or add point
    if (replaceIndex != -1) {
        voltageMap[replaceIndex].rawADC = currentRaw;
        voltageMap[replaceIndex].realV = realVoltage;
    } else if (calPointCount < MAX_CAL_POINTS) {
        voltageMap[calPointCount].rawADC = currentRaw;
        voltageMap[calPointCount].realV = realVoltage;
        calPointCount++;
    } else {
        voltageMap[0].rawADC = currentRaw;
        voltageMap[0].realV = realVoltage;
    }
    
    // Sort calibration points by raw ADC value
    for (int i = 0; i < calPointCount - 1; i++) {
        for (int j = 0; j < calPointCount - i - 1; j++) {
            if (voltageMap[j].rawADC > voltageMap[j + 1].rawADC) {
                CalPoint temp = voltageMap[j];
                voltageMap[j] = voltageMap[j + 1];
                voltageMap[j + 1] = temp;
            }
        }
    }
    
    saveCalibrationSensor();
    
    if (calPointCount >= MIN_CAL_POINTS) {
        sensorSystemValid = true;
    }
}

/**
 * Get interpolated voltage from calibration map
 */
float getInterpolatedVoltage(float rawADC) {
    if (calPointCount == 0) return 0.0;
    
    // Below minimum - return first point
    if (rawADC <= voltageMap[0].rawADC) {
        return voltageMap[0].realV;
    }
    
    // Above maximum - extrapolate
    if (rawADC >= voltageMap[calPointCount - 1].rawADC) {
        if (calPointCount < 2) return voltageMap[0].realV;
        
        float slope = (voltageMap[calPointCount - 1].realV - voltageMap[calPointCount - 2].realV) /
                      (voltageMap[calPointCount - 1].rawADC - voltageMap[calPointCount - 2].rawADC);
        return voltageMap[calPointCount - 1].realV + 
               (slope * (rawADC - voltageMap[calPointCount - 1].rawADC));
    }
    
    // Interpolate between points
    for (int i = 0; i < calPointCount - 1; i++) {
        if (rawADC >= voltageMap[i].rawADC && rawADC <= voltageMap[i + 1].rawADC) {
            float slope = (voltageMap[i + 1].realV - voltageMap[i].realV) /
                          (voltageMap[i + 1].rawADC - voltageMap[i].rawADC);
            return voltageMap[i].realV + (slope * (rawADC - voltageMap[i].rawADC));
        }
    }
    
    return 0.0;
}

/**
 * Calculate automatic current offset (zero calibration)
 */
void calculateAutoOffset() {
    float raw = getMedianADC(sensorPin, 100);
    float V_ADC = (raw / ADC_resolution) * V_ref;
    I_offset = (V_ADC / I_divider_factor) * I_scale_factor;
    saveCalibrationSensor();
}

// ============================================================================
// UI ANIMATION FUNCTIONS
// ============================================================================

/**
 * Fill quadrilateral shape
 */
void fillQuad(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4, uint16_t color) {
    tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
    tft.fillTriangle(x1, y1, x3, y3, x4, y4, color);
}

/**
 * Draw SD Card Error UI (Apple Style)
 */
/**
 * Draw SD Card Success UI (Professional 3D Style)
 */
void drawSDSuccess(float cardSizeGB, String fsType = "FAT32") {
    tft.fillScreen(COLOR_BG); // Black Background

    // --- 2. GAMBAR SD CARD 3D (Posisi & Geometri Sama dengan Error) ---
    int cx = 120; int cy = 70;
    int sdW = 50; int sdH = 70;
    int depth = 6; int tilt = 10; int cutSize = 12;

    int topX = cx + tilt; int topY = cy - (sdH / 2);
    int botX = cx;        int botY = cy + (sdH / 2);
    
    int xTL = topX - (sdW / 2); int xTR = topX + (sdW / 2);
    int xBR = botX + (sdW / 2); int xBL = botX - (sdW / 2);
    int yTL = topY; int yTR = topY; int yBR = botY; int yBL = botY;

    // A. Housing (Sisi 3D / Tebal)
    fillQuad(xTR, yTR + cutSize, xTR + depth, yTR + cutSize, xBR + depth, yBR, xBR, yBR, C_HOUSING_SIDE);
    fillQuad(xBR, yBR, xBR + depth, yBR, xBL + depth, yBL, xBL, yBL, C_HOUSING_SIDE);
    fillQuad(xTR - cutSize, yTR, xTR - cutSize + depth, yTR, xTR + depth, yTR + cutSize, xTR, yTR + cutSize, C_HOUSING_SIDE);

    // B. Main Body
    fillQuad(xTL, yTL, xTR, yTR, xBR, yBR, xBL, yBL, C_SD_BODY);
    tft.fillTriangle(xTR, yTR, xTR, yTR + cutSize, xTR - cutSize, yTR, COLOR_BG); // Potongan sudut

    // C. Sticker Label (Biru Profesional)
    int margin = 5;
    fillQuad(xTL + margin, yTL + 15, xTR - margin - cutSize + 2, yTL + 15, 
             xBR - margin, yBR - 10, xBL + margin, yBR - 10, C_SD_LABEL);
    
    // Hiasan pada label (Garis putih tipis ala SanDisk/Samsung)
    tft.drawLine(xTL + margin, yTL + 40, xBR - margin, yBR - 35, 0x10A2); // Garis samar

    // D. Pin Kuningan (Detail Realistis)
    int pinW = 4; int pinGap = 2; int startPinX = xBL + 10;
    for(int i=0; i<7; i++) {
        int px = startPinX + (i * (pinW + pinGap));
        fillQuad(px, yBR - 8, px + pinW, yBR - 8, px + pinW - 2, yBR - 2, px - 2, yBR - 2, TFT_ORANGE);
    }

    // --- 3. BADGE SUKSES (3D Green Button) ---
    int badgeCX = cx; 
    int badgeCY = cy + 10;
    int r = 22;

    // Shadow Badge (Efek Timbul)
    tft.fillCircle(badgeCX, badgeCY + 3, r, C_SUCCESS_SH);
    // Main Badge
    tft.fillCircle(badgeCX, badgeCY, r, C_SUCCESS);
    // Ring Putih
    tft.drawCircle(badgeCX, badgeCY, r, TFT_WHITE);

    // Gambar Ceklis Putih Tebal
    // Titik sudut ceklis: (Kiri, Bawah, KananAtas)
    int ckX = badgeCX - 2; int ckY = badgeCY + 3;
    tft.drawLine(ckX - 8, ckY - 5, ckX, ckY + 3, TFT_WHITE); // Kiri ke Bawah
    tft.drawLine(ckX - 9, ckY - 5, ckX - 1, ckY + 3, TFT_WHITE); // Tebalkan
    
    tft.drawLine(ckX, ckY + 3, ckX + 10, ckY - 9, TFT_WHITE); // Bawah ke Kanan
    tft.drawLine(ckX, ckY + 4, ckX + 10, ckY - 8, TFT_WHITE); // Tebalkan

    // --- 4. INFO PANEL (Card UI) ---
    // Judul
    tft.setTextColor(C_SUCCESS, COLOR_BG);
    tft.setTextSize(2);
    String title = "SD CARD READY";
    tft.setCursor((240 - tft.textWidth(title)) / 2, 130);
    tft.print(title);

    // Kotak Info (Rounded)
    int cardX = 20; int cardY = 155;
    int cardW = 200; int cardH = 50;
    
    tft.fillRoundRect(cardX, cardY, cardW, cardH, 8, C_PANEL_BG);
    tft.drawRoundRect(cardX, cardY, cardW, cardH, 8, 0x528A); // Border halus

    // Isi Data (Grid 2 Kolom)
    tft.setTextSize(1);
    
    // Label
    tft.setTextColor(0xBDF7, C_PANEL_BG); // Abu Terang
    tft.setCursor(cardX + 15, cardY + 10); tft.print("CAPACITY");
    tft.setCursor(cardX + 110, cardY + 10); tft.print("FORMAT");

    // Divider Vertikal
    tft.drawFastVLine(cardX + 100, cardY + 10, 30, 0x10A2);

    // Value (Putih Besar)
    tft.setTextColor(TFT_WHITE, C_PANEL_BG);
    tft.setTextSize(1); // Atau 2 jika muat
    
    // Print Size
    tft.setCursor(cardX + 15, cardY + 25);
    tft.print(cardSizeGB, 1); tft.print(" GB");

    // Print Format
    tft.setCursor(cardX + 110, cardY + 25);
    tft.print(fsType);

    // --- 5. FOOTER ---
    tft.setTextColor(0x7BEF, COLOR_BG);
    tft.setCursor((240 - tft.textWidth("Sistem siap merekam")) / 2, 225);
    tft.print("Sistem siap merekam");
}

/**
 * Draw SD Card Error UI (Professional 3D Alert Style)
 */
void drawSDError(String errCode, String errMsg) {
    tft.fillScreen(COLOR_BG); // Black Background

    // --- 2. GAMBAR SD CARD 3D (Geometri Identik) ---
    int cx = 120; int cy = 70;
    int sdW = 50; int sdH = 70;
    int depth = 6; int tilt = 10; int cutSize = 12;

    int topX = cx + tilt; int topY = cy - (sdH / 2);
    int botX = cx;        int botY = cy + (sdH / 2);
    
    int xTL = topX - (sdW / 2); int xTR = topX + (sdW / 2);
    int xBR = botX + (sdW / 2); int xBL = botX - (sdW / 2);
    int yTL = topY; int yTR = topY; int yBR = botY; int yBL = botY;

    // A. Housing (Sisi 3D)
    fillQuad(xTR, yTR + cutSize, xTR + depth, yTR + cutSize, xBR + depth, yBR, xBR, yBR, C_HOUSING_SIDE);
    fillQuad(xBR, yBR, xBR + depth, yBR, xBL + depth, yBL, xBL, yBL, C_HOUSING_SIDE);
    fillQuad(xTR - cutSize, yTR, xTR - cutSize + depth, yTR, xTR + depth, yTR + cutSize, xTR, yTR + cutSize, C_HOUSING_SIDE);

    // B. Main Body
    fillQuad(xTL, yTL, xTR, yTR, xBR, yBR, xBL, yBL, C_SD_BODY);
    tft.fillTriangle(xTR, yTR, xTR, yTR + cutSize, xTR - cutSize, yTR, COLOR_BG); // Cut corner

    // C. Sticker Label (Merah Gelap - Menandakan Masalah)
    int margin = 5;
    fillQuad(xTL + margin, yTL + 15, xTR - margin - cutSize + 2, yTL + 15, 
             xBR - margin, yBR - 10, xBL + margin, yBR - 10, C_SD_LABEL);
    
    // Hiasan pola garis bahaya (Hazard Striping) pada stiker
    tft.drawLine(xTL + margin + 5, yBR - 15, xTL + margin + 15, yBR - 30, 0xC618); // Garis samar

    // D. Pin Kuningan (Tetap ada)
    int pinW = 4; int pinGap = 2; int startPinX = xBL + 10;
    for(int i=0; i<7; i++) {
        int px = startPinX + (i * (pinW + pinGap));
        fillQuad(px, yBR - 8, px + pinW, yBR - 8, px + pinW - 2, yBR - 2, px - 2, yBR - 2, TFT_ORANGE);
    }

    // --- 3. BADGE ERROR (3D Red X Button) ---
    int badgeCX = cx; 
    int badgeCY = cy + 10;
    int r = 22;

    // Shadow Badge (Efek Timbul)
    tft.fillCircle(badgeCX, badgeCY + 3, r, C_ERROR_SH);
    // Main Badge
    tft.fillCircle(badgeCX, badgeCY, r, C_ERROR);
    // Ring Putih
    tft.drawCircle(badgeCX, badgeCY, r, TFT_WHITE);

    // Gambar Tanda Silang (X) Putih Tebal
    int cr = 8; // Cross Radius
    // Garis 1 (\)
    tft.drawLine(badgeCX - cr, badgeCY - cr, badgeCX + cr, badgeCY + cr, TFT_WHITE);
    tft.drawLine(badgeCX - cr + 1, badgeCY - cr, badgeCX + cr + 1, badgeCY + cr, TFT_WHITE); // Tebal
    // Garis 2 (/)
    tft.drawLine(badgeCX + 9, badgeCY - 8, badgeCX - 7, badgeCY + 8, TFT_WHITE); // Tebalkan
    tft.drawLine(badgeCX + 7, badgeCY - 8, badgeCX - 9, badgeCY + 8, TFT_WHITE); // Tebalkan


    // --- 4. INFO PANEL (Card UI Error Style) ---
    // Judul
    tft.setTextColor(C_ERROR, COLOR_BG);
    tft.setTextSize(2);
    String title = "SD CARD ERROR!";
    tft.setCursor((240 - tft.textWidth(title)) / 2, 130);
    tft.print(title);

    // Kotak Info (Rounded)
    int cardX = 20; int cardY = 155;
    int cardW = 200; int cardH = 50;
    
    // Background sedikit kemerahan gelap (Very dark red tint)
    tft.fillRoundRect(cardX, cardY, cardW, cardH, 8, C_PANEL_BG);
    // Border Merah Tipis sebagai penegas error
    tft.drawRoundRect(cardX, cardY, cardW, cardH, 8, 0x6000); 

    // Isi Data (Split View: Code | Status)
    tft.setTextSize(1);
    
    // Label
    tft.setTextColor(C_TEXT_MUTED, C_PANEL_BG); 
    tft.setCursor(cardX + 15, cardY + 10); tft.print("ERR CODE");
    tft.setCursor(cardX + 100, cardY + 10); tft.print("DETAIL");

    // Divider Vertikal (Merah Gelap)
    tft.drawFastVLine(cardX + 90, cardY + 10, 30, 0x6000);

    // Value (Putih/Merah Terang)
    tft.setTextSize(1); 
    
    // Print Error Code
    tft.setTextColor(TFT_WHITE, C_PANEL_BG);
    tft.setCursor(cardX + 15, cardY + 25);
    tft.print(errCode);

    // Print Error Message (Jika kepanjangan, potong)
    tft.setTextColor(0xF81F, C_PANEL_BG); // Pinkish Red text
    tft.setCursor(cardX + 100, cardY + 25);
    if(errMsg.length() > 10) errMsg = errMsg.substring(0, 10); // Truncate visual
    tft.print(errMsg);

    // --- 5. FOOTER (Actionable Advice) ---
    tft.setTextColor(C_TEXT_MUTED, COLOR_BG);
    tft.setCursor((240 - tft.textWidth("Cek kartu & restart alat")) / 2, 225);
    tft.print("Cek kartu & restart alat");
}

/**
 * Draw 3D Page Number Indicator
 */
void drawPageNumber(int page) {
    // Koordinat
    int x = 215; 
    int y = 215; 
    int w = 24; 
    int h = 24;
    int depth = 4; // Ketebalan 3D

    // Warna (Mengambil dari palet sebelumnya)
    uint16_t cFront = C_POLE;       // Abu Terang (Muka)
    uint16_t cSide  = C_POLE_SHADE; // Abu Gelap (Sisi 3D)
    
    // 1. Gambar Sisi 3D (Bawah & Kanan)
    // Sisi Kanan
    tft.fillRect(x + w, y + depth, depth, h, cSide);
    // Sisi Bawah
    tft.fillRect(x + depth, y + h, w, depth, cSide);
    // Sudut Bawah-Kanan (penghubung)
    tft.fillRect(x + w, y + h, depth, depth, cSide);

    // 2. Muka Depan (Tombol)
    tft.fillRect(x, y, w, h, cFront);
    
    // 3. Highlight/Border (Opsional: Garis Putih di kiri & atas agar pop)
    tft.drawFastHLine(x, y, w, TFT_WHITE);
    tft.drawFastVLine(x, y, h, TFT_WHITE);

    // 4. Teks Angka (Warna Gelap di atas Terang)
    tft.setTextColor(TFT_BLACK, cFront); // Teks Hitam
    tft.setTextSize(2);
    
    String numStr = String(page);
    int textW = tft.textWidth(numStr);
    
    // Center text
    tft.setCursor(x + (w - textW) / 2, y + 5);
    tft.print(numStr);
}

/**
 * Draw result icon for system check (3D button style)
 */
void drawResultIcon(int x, int y, bool success) {
    int r = 10; // Radius
    
    if (success) {
        // --- SUCCESS ICON (Green Button) ---
        // Shadow effect (bottom)
        tft.fillCircle(x, y + 2, r, 0x01E0); // Dark Green
        // Button face
        tft.fillCircle(x, y, r, CALIB_COLOR_DONE); 
        // Highlight
        tft.fillCircle(x - 3, y - 3, 3, TFT_WHITE);
        
        // White checkmark
        tft.drawLine(x - 4, y, x - 1, y + 4, TFT_WHITE);
        tft.drawLine(x - 1, y + 4, x + 5, y - 3, TFT_WHITE);
        tft.drawLine(x - 4, y + 1, x - 1, y + 5, TFT_WHITE); // Thicken
    } else {
        // --- FAIL ICON (Red Button) ---
        // Shadow effect
        tft.fillCircle(x, y + 2, r, 0x6000); // Dark Red
        // Button face (Apple Red style)
        tft.fillCircle(x, y, r, 0xF965); 
        // Highlight
        tft.fillCircle(x - 3, y - 3, 3, TFT_WHITE);
        
        // White X
        tft.drawLine(x - 4, y - 4, x + 4, y + 4, TFT_WHITE);
        tft.drawLine(x + 4, y - 4, x - 4, y + 4, TFT_WHITE);
    }
}

/**
 * Run comprehensive system hardware check at boot
 */
void runSystemCheck() {
    tft.fillScreen(COLOR_BG);

    // --- 1. KONFIGURASI LAYOUT CARD ---
    int pad = 10;                // Margin kiri/kanan lebih tipis biar luas
    int panelX = pad;
    int panelY = 15;             // Naikkan posisi Y agar muat
    int panelW = 240 - (pad * 2);
    int panelH = 210;            // Tinggi disesuaikan agar sisa margin bawah
    int depth = 5;               // Ketebalan 3D
    int radius = 12;             // Kelengkungan sudut (Rounded)

    // Warna Palette
    uint16_t cFace   = 0x2124;   // Dark Gray Body
    uint16_t cShadow = 0x10A2;   // Deep Shadow
    uint16_t cHeader = C_POLE;   // Light Gray Header

    // --- 2. GAMBAR CARD 3D ROUNDED ---

    // A. Layer Bayangan (Shadow) - Digeser ke kanan bawah
    tft.fillRoundRect(panelX + depth, panelY + depth, panelW, panelH, radius, cShadow);

    // B. Layer Body Utama (Face)
    tft.fillRoundRect(panelX, panelY, panelW, panelH, radius, cFace);

    // C. Header Area (Top Section)
    // Trik: Gambar Rounded Rect penuh untuk header, lalu timpa separuh bawahnya
    // agar sudut atas bulat, tapi bawahnya rata menyatu dengan body.
    int headerH = 38;
    tft.fillRoundRect(panelX, panelY, panelW, headerH, radius, cHeader);
    tft.fillRect(panelX, panelY + (headerH/2), panelW, headerH/2, cHeader); // Ratakan bawah header
    
    // Garis pemisah header & body
    tft.drawFastHLine(panelX, panelY + headerH, panelW, 0x528A); 

    // D. Header Title & Decor
    tft.setTextColor(TFT_BLACK, cHeader);
    tft.setTextSize(2);
    tft.setCursor(panelX + 15, panelY + 12);
    tft.print("SYSTEM DIAG");

    // Lampu Indikator Power (Pojok Kanan Atas)
    tft.fillCircle(panelX + panelW - 20, panelY + 19, 6, TFT_BLACK); // Housing
    tft.fillCircle(panelX + panelW - 20, panelY + 19, 4, TFT_CYAN);  // LED ON

    // --- 3. LOGIKA CEK BERURUTAN ---
    
    // Koordinat isi
    int startY = panelY + headerH + 15; // Jarak dari header
    int gapY = 32; // Jarak antar item dirapatkan sedikit agar muat
    int currentY = startY;

    const char* components[] = { "RTC DS3231", "SD Card", "Sensor Arus", "Sensor Volt" };
    int compCount = 4;
    
    bool overallStatus = true; 

    for (int i = 0; i < compCount; i++) {
        // A. Tulis Nama Komponen
        tft.setTextColor(TFT_WHITE, cFace);
        tft.setTextSize(1);
        tft.setCursor(panelX + 15, currentY + 5);
        tft.print(components[i]);
        
        // Animasi Loading (...)
        int dotX = panelX + 130;
        // Kita simpan posisi kursor dot biar bisa dihapus rapi
        for(int d=0; d<3; d++) {
            tft.setCursor(dotX + (d*6), currentY + 5);
            tft.print(".");
            delay(100); 
        }

        // B. Cek Hardware (Gunakan variabel global status Anda)
        bool result = false;
        switch(i) {
            case 0: result = rtcReady; break;            // Pastikan variabel ini ada
            case 1: result = sdReady; break;             // Pastikan variabel ini ada
            case 2: result = true; break;                // Contoh logic
            case 3: result = (calPointCount >= 1); break; // Contoh logic
        }

        if (!result) overallStatus = false;

        // C. Gambar Hasil
        // Hapus area loading dots dengan kotak warna body
        tft.fillRect(dotX, currentY, 40, 15, cFace); 
        
        // Gambar Ikon (Fungsi drawResultIcon harus sudah ada)
        // Posisi X ikon diratakan kanan dengan margin
        drawResultIcon(panelX + panelW - 25, currentY + 8, result);

        // Garis pemisah tipis (kecuali item terakhir)
        if (i < compCount - 1) {
            // Garis putus-putus atau solid tipis warna gelap
            tft.drawFastHLine(panelX + 15, currentY + gapY - (gapY/2) + 2, panelW - 30, 0x3186);
        }
        
        // Pindah baris
        currentY += gapY;
    }

    // --- 4. FOOTER STATUS (Floating Pill Style) ---
    // Agar tidak merusak rounded corner bawah panel, footer dibuat "mengambang"
    // di dalam panel bagian bawah.
    
    int footerH = 34;
    int footerY = panelY + panelH - footerH - 10; // Margin 10px dari bawah panel
    int footerX = panelX + 15;
    int footerW = panelW - 30;

    uint16_t statusColor = overallStatus ? 0x03E0 : 0xF965; // Hijau vs Merah Apple
    
    // Gambar Background Footer (Rounded)
    tft.fillRoundRect(footerX, footerY, footerW, footerH, 8, statusColor);
    
    // Teks Status
    tft.setTextColor(TFT_WHITE, statusColor);
    tft.setTextSize(2);
    
    String finalMsg = overallStatus ? "SYSTEM READY" : "CHECK FAILED";
    // Center text di dalam tombol footer
    tft.setCursor(footerX + (footerW - tft.textWidth(finalMsg))/2, footerY + 9);
    tft.print(finalMsg);
    
    delay(2000);
}

/**
 * Draw start logging animation (traffic light)
 */
void drawStartLoggingUI() {
    tft.fillScreen(TFT_BLACK);
    
    // Pole parameters
    int baseX = 100, baseY = 230, poleH = 60;
    int boxW = 50, boxH = 110, depth = 8, tiltOffset = 45;
    
    // Draw pole
    int pX_top = baseX + (tiltOffset * poleH / (poleH + boxH));
    int pY_top = baseY - poleH;
    fillQuad(pX_top - 3, pY_top, pX_top + 3, pY_top, 
             baseX + 3, baseY, baseX - 3, baseY, C_POLE);
    fillQuad(pX_top + 3, pY_top, pX_top + 5, pY_top, 
             baseX + 5, baseY, baseX + 3, baseY, C_POLE_SHADE);
    
    // Draw housing
    int bx = pX_top, by = pY_top, topX = bx + 30, topY = by - boxH;
    int xBL = bx - (boxW / 2), xBR = bx + (boxW / 2);
    int xTR = topX + (boxW / 2), xTL = topX - (boxW / 2);
    
    fillQuad(xTL, topY, xTR, topY, xBR, by, xBL, by, C_HOUSING_FRONT);
    fillQuad(xTR, topY, xTR + depth, topY, xBR + depth, by, xBR, by, C_HOUSING_SIDE);
    fillQuad(xBR, by, xBR + depth, by, xBL + depth, by, xBL, by, C_HOUSING_SIDE);
    tft.drawLine(xTL, topY, xTL + depth, topY, C_HOUSING_SIDE);
    
    // Calculate light positions
    int lightRadius = 12;
    int ry = topY + 25, rx = map(ry, topY, by, topX, bx);
    int yy = topY + (boxH / 2), yx = map(yy, topY, by, topX, bx);
    int gy = by - 25, gx = map(gy, topY, by, topX, bx);
    
    // Clear light backgrounds
    tft.fillCircle(rx, ry, lightRadius + 1, TFT_BLACK);
    tft.fillCircle(yx, yy, lightRadius + 1, TFT_BLACK);
    tft.fillCircle(gx, gy, lightRadius + 1, TFT_BLACK);
    
    // Glare effect lambda
    auto glare = [&](int lx, int ly) {
        tft.fillCircle(lx - 4, ly - 4, 3, TFT_WHITE);
    };
    
    tft.setTextSize(2);
    
    // RED light
    tft.fillCircle(rx, ry, lightRadius, TL_RED_ON);
    glare(rx, ry);
    tft.fillCircle(yx, yy, lightRadius, TL_YELLOW_DIM);
    tft.fillCircle(gx, gy, lightRadius, TL_GREEN_DIM);
    tft.setTextColor(TL_RED_ON, TFT_BLACK);
    tft.setCursor(10, 50);
    tft.print("SIAP...");
    delay(600);
    
    // YELLOW light
    tft.fillCircle(rx, ry, lightRadius, TL_RED_DIM);
    tft.fillCircle(yx, yy, lightRadius, TL_YELLOW_ON);
    glare(yx, yy);
    tft.fillRect(0, 40, 100, 40, TFT_BLACK);
    tft.setTextColor(TL_YELLOW_ON, TFT_BLACK);
    tft.setCursor(10, 80);
    tft.print("SET...");
    delay(600);
    
    // GREEN light
    tft.fillCircle(yx, yy, lightRadius, TL_YELLOW_DIM);
    tft.fillCircle(gx, gy, lightRadius, TL_GREEN_ON);
    glare(gx, gy);
    tft.fillRect(0, 70, 100, 40, TFT_BLACK);
    tft.setTextColor(TL_GREEN_ON, TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(10, 110);
    tft.print("GO!!");
    
    tft.setTextSize(1);
    tft.setCursor(10, 140);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("Data Logging...");
    
    delay(1000);
    tft.fillScreen(COLOR_BG);
}

/**
 * Draw stop logging animation (checkered flag)
 */
void drawStopLoggingUI() {
    tft.fillScreen(COLOR_BG);
    
    int poleX = 80, poleY = 60, flagW = 100, flagH = 70, cellSize = 10;
    
    // Draw pole and top
    tft.fillRect(poleX - 4, poleY, 4, 120, TFT_GREY);
    tft.fillCircle(poleX - 2, poleY, 5, TFT_ORANGE);
    
    // Animate waving flag
    for (int frame = 0; frame < 15; frame++) {
        tft.fillRect(poleX, poleY - 10, flagW + 10, flagH + 20, COLOR_BG);
        
        for (int x = 0; x < flagW / cellSize; x++) {
            for (int y = 0; y < flagH / cellSize; y++) {
                int wave = (int)(sin((x + frame) * 0.8) * 5);
                if ((x + y) % 2 == 0) {
                    tft.fillRect(poleX + (x * cellSize), poleY + (y * cellSize) + wave, 
                                cellSize, cellSize, TFT_WHITE);
                }
            }
        }
        delay(30);
    }
    
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.setTextSize(2);
    String msg = "LOG STOPPED";
    tft.setCursor((240 - tft.textWidth(msg)) / 2, 190);
    tft.print(msg);
    
    delay(800);
    tft.fillScreen(COLOR_BG);
}

// ============================================================================
// CHART CLASSES
// ============================================================================

/**
 * Slide 1: Real-time Chart (Voltage, Current Regen, Current Used)
 */
class Slide1_RTChart {
private:
    TFT_eSPI *tft;
    TFT_eSprite *sprite;
    int16_t *hist1, *hist2, *hist3;
    int index, width, height, maxY;
    const int xOffset = 30, yOffset = 10;
    ChannelConfig ch1, ch2, ch3;

public:
    Slide1_RTChart(TFT_eSPI *display) : tft(display) {
        sprite = nullptr;
        hist1 = hist2 = hist3 = nullptr;
    }

    void begin(int w, int h, ChannelConfig c1, ChannelConfig c2, ChannelConfig c3) {
        width = w;
        height = h;
        maxY = h - 1;
        ch1 = c1;
        ch2 = c2;
        ch3 = c3;
        
        // Only initialize history arrays if they don't exist (preserve chart data)
        if (hist1 == nullptr) {
            hist1 = new int16_t[width];
            hist2 = new int16_t[width];
            hist3 = new int16_t[width];
            index = 0;
            
            for (int i = 0; i < width; i++) {
                hist1[i] = maxY;
                hist2[i] = maxY;
                hist3[i] = maxY;
            }
        }
        
        // Always recreate sprite (was deleted in end())
        sprite = new TFT_eSprite(tft);
        sprite->createSprite(width, height);
        refresh();
    }

    void refresh() {
        tft->fillScreen(COLOR_BG);
        drawAxes();
    }

    void end() {
        // Only delete sprite, preserve history data for persistence
        if (sprite) {
            sprite->deleteSprite();
            delete sprite;
            sprite = nullptr;
        }
        // Don't delete history arrays - preserve chart data
        // if (hist1) delete[] hist1;
        // if (hist2) delete[] hist2;
        // if (hist3) delete[] hist3;
    }

    void update(float v1, float v2, float v3) {
        if (!sprite) return;
        
        sprite->fillSprite(COLOR_BG);
        
        // Draw grid
        for (int i = 1; i < 8; i++) {
            sprite->drawFastVLine(i * width / 8, 0, height, COLOR_GRID);
        }
        for (int i = 1; i < 4; i++) {
            sprite->drawFastHLine(0, i * height / 4, width, COLOR_GRID);
        }
        sprite->drawRect(0, 0, width, height, COLOR_GRID);
        
        // Map value to Y coordinate
        auto mapValue = [&](float v, float peak) {
            if (v < 0) v = 0;
            return constrain((int)(maxY - (v / peak) * (height - 2)), 0, height - 1);
        };
        
        // Store current values
        hist1[index] = mapValue(v1, ch1.peak);
        hist2[index] = mapValue(v2, ch2.peak);
        hist3[index] = mapValue(v3, ch3.peak);
        index = (index + 1) % width;
        
        // Draw lines
        for (int x = 0; x < width - 1; x++) {
            int i = (index + x) % width;
            int next = (index + x + 1) % width;
            sprite->drawLine(x, hist1[i], x + 1, hist1[next], ch1.color);
            sprite->drawLine(x, hist2[i], x + 1, hist2[next], ch2.color);
            sprite->drawLine(x, hist3[i], x + 1, hist3[next], ch3.color);
        }
        
        sprite->pushSprite(xOffset, yOffset);
        drawPanel(v1, v2, v3);
        drawPageNumber(1);
    }

private:
    void drawAxes() {
        tft->setTextColor(COLOR_TEXT_L, COLOR_BG);
        tft->setTextSize(1);
        
        // Y-axis labels
        tft->setCursor(12, yOffset);
        tft->print((int)ch1.peak);
        tft->setCursor(12, yOffset + (height / 2) - 4);
        tft->print((int)(ch1.peak / 2));
        tft->setCursor(12, yOffset + height - 8);
        tft->print("0");
        
        // X-axis labels
        int yX = yOffset + height + 4;
        tft->setCursor(xOffset, yX);
        tft->print("0");
        tft->setCursor(xOffset + (width / 2) - 10, yX);
        tft->print("T/2");
        tft->setCursor(xOffset + width - 10, yX);
        tft->print("T");
    }

    void drawPanel(float v1, float v2, float v3) {
        int y = 165;
        char buffer[16];
        tft->setTextPadding(100);
        
        // Print value helper with unit appended
        auto printValue = [&](int x, int y, const char* label, float value, int decimals, const char* unit, uint16_t color) {
            tft->setCursor(x, y);
            tft->setTextColor(color, COLOR_BG);
            tft->setTextSize(1);
            tft->print(label);
            
            tft->setCursor(x, y + 10);
            tft->setTextSize(2);
            dtostrf(value, 4, decimals, buffer);
            tft->print(buffer);
            
            // Append unit
            tft->setTextSize(1);
            tft->setCursor(tft->getCursorX() + 2, y + 17); // Align with bottom of number
            tft->print(unit);
        };
        
        // Shifted X to 10 for better left alignment
        printValue(10, y, ch1.name, v1, 1, "V", ch1.color);
        
        // Uptime
        unsigned long sec = sysData.uptime_sec;
        char timeBuffer[10];
        if (sec < 60) {
            sprintf(timeBuffer, "%ds", (int)sec);
        } else if (sec < 3600) {
            sprintf(timeBuffer, "%.1fm", sec / 60.0);
        } else {
            sprintf(timeBuffer, "%.1fh", sec / 3600.0);
        }
        
        tft->setCursor(130, y);
        tft->setTextColor(COLOR_TEXT_V, COLOR_BG);
        tft->setTextSize(1);
        tft->print("Uptime");
        tft->setCursor(130, y + 10);
        tft->setTextSize(2);
        tft->print(timeBuffer);
        
        printValue(10, y + 35, ch2.name, v2, 2, "A", ch2.color);
        printValue(130, y + 35, ch3.name, v3, 2, "A", ch3.color);
        
        tft->setTextPadding(0);
    }
};

/**
 * Slide 2: Energy Chart (Voltage, Energy Used, Energy Regen)
 */
class Slide2_EnergyChart {
private:
    TFT_eSPI *tft;
    TFT_eSprite *sprite;
    int16_t *hist1, *hist2, *hist3;
    int index, width, height, maxY;
    const int xOffset = 30, yOffset = 10;
    ChannelConfig ch1, ch2, ch3;

public:
    Slide2_EnergyChart(TFT_eSPI *display) : tft(display) {
        sprite = nullptr;
        hist1 = hist2 = hist3 = nullptr;
    }

    void begin(int w, int h, ChannelConfig c1, ChannelConfig c2, ChannelConfig c3) {
        width = w;
        height = h;
        maxY = h - 1;
        ch1 = c1;
        ch2 = c2;
        ch3 = c3;
        
        // Only initialize history arrays if they don't exist (preserve chart data)
        if (hist1 == nullptr) {
            hist1 = new int16_t[width];
            hist2 = new int16_t[width];
            hist3 = new int16_t[width];
            index = 0;
            
            for (int i = 0; i < width; i++) {
                hist1[i] = maxY;
                hist2[i] = maxY;
                hist3[i] = maxY;
            }
        }
        
        // Always recreate sprite (was deleted in end())
        sprite = new TFT_eSprite(tft);
        sprite->createSprite(width, height);
        refresh();
    }

    void refresh() {
        tft->fillScreen(COLOR_BG);
        drawAxes();
    }

    void end() {
        // Only delete sprite, preserve history data for persistence
        if (sprite) {
            sprite->deleteSprite();
            delete sprite;
            sprite = nullptr;
        }
        // Don't delete history arrays - preserve chart data
        // if (hist1) delete[] hist1;
        // if (hist2) delete[] hist2;
        // if (hist3) delete[] hist3;
    }

    void update(float v1, float v2, float v3, float efficiency) {
        if (!sprite) return;
        
        sprite->fillSprite(COLOR_BG);
        
        // Draw grid
        for (int i = 1; i < 8; i++) {
            sprite->drawFastVLine(i * width / 8, 0, height, COLOR_GRID);
        }
        for (int i = 1; i < 4; i++) {
            sprite->drawFastHLine(0, i * height / 4, width, COLOR_GRID);
        }
        sprite->drawRect(0, 0, width, height, COLOR_GRID);
        
        // Map value to Y coordinate
        auto mapValue = [&](float v, float peak) {
            if (v < 0) v = 0;
            return constrain((int)(maxY - (v / peak) * (height - 2)), 0, height - 1);
        };
        
        // Store values
        hist1[index] = mapValue(v1, ch1.peak);
        hist2[index] = mapValue(v2, ch2.peak);
        hist3[index] = mapValue(v3, ch3.peak);
        index = (index + 1) % width;
        
        // Draw lines
        for (int x = 0; x < width - 1; x++) {
            int i = (index + x) % width;
            int next = (index + x + 1) % width;
            sprite->drawLine(x, hist1[i], x + 1, hist1[next], ch1.color);
            sprite->drawLine(x, hist2[i], x + 1, hist2[next], ch2.color);
            sprite->drawLine(x, hist3[i], x + 1, hist3[next], ch3.color);
        }
        
        sprite->pushSprite(xOffset, yOffset);
        drawPanel(v1, v2, v3, efficiency);
        drawPageNumber(2);
    }

private:
    void drawAxes() {
        tft->setTextColor(COLOR_TEXT_L, COLOR_BG);
        tft->setTextSize(1);
        
        // Y-axis labels
        tft->setCursor(12, yOffset);
        tft->print((int)ch1.peak);
        tft->setCursor(12, yOffset + (height / 2) - 4);
        tft->print((int)(ch1.peak / 2));
        tft->setCursor(12, yOffset + height - 8);
        tft->print("0");
        
        // X-axis labels
        int yX = yOffset + height + 4;
        tft->setCursor(xOffset, yX);
        tft->print("0");
        tft->setCursor(xOffset + (width / 2) - 10, yX);
        tft->print("T/2");
        tft->setCursor(xOffset + width - 10, yX);
        tft->print("T");
    }

    void drawPanel(float v1, float v2, float v3, float eff) {
        int y = 165;
        char buffer[16];
        tft->setTextPadding(100);
        
        // Print value helper with unit appended
        auto printValue = [&](int x, int y, const char* label, float value, int decimals, const char* unit, uint16_t color) {
            tft->setCursor(x, y);
            tft->setTextColor(color, COLOR_BG);
            tft->setTextSize(1);
            tft->print(label);
            
            tft->setCursor(x, y + 10);
            tft->setTextSize(2);
            dtostrf(value, 4, decimals, buffer);
            tft->print(buffer);
            
            // Append unit
            tft->setTextSize(1);
            tft->setCursor(tft->getCursorX() + 2, y + 17); // Align with bottom of number
            tft->print(unit);
        };
        
        // Shifted X to 10 for better left alignment
        printValue(10, y, ch1.name, v1, 1, "V", ch1.color);
        
        // Efficiency
        tft->setCursor(130, y);
        tft->setTextColor(COLOR_TEXT_V, COLOR_BG);
        tft->setTextSize(1);
        tft->print("Efisiensi");
        tft->setCursor(130, y + 10);
        tft->setTextSize(2);
        dtostrf(eff, 4, 1, buffer);
        tft->print(buffer);
        tft->setTextSize(1);
        tft->setCursor(tft->getCursorX() + 2, y + 17);
        tft->print("%");
        
        printValue(10, y + 35, ch2.name, v2, 2, "Wh", ch2.color);
        printValue(130, y + 35, ch3.name, v3, 2, "Wh", ch3.color);
        
        tft->setTextPadding(0);
    }
};

/**
 * Slide 3: Dashboard Summary (Clean Card Style)
 */
class Slide3_Dashboard {
private:
    TFT_eSPI *tft;

    // Helper: Menggambar "Clean Card" dengan Aksen Warna
    void drawCleanCard(int x, int y, int w, int h, uint16_t accentColor, const char* label) {
        int r = 8; // Radius sudut

        // 1. Bayangan (Geser sedikit ke kanan bawah)
        tft->fillRoundRect(x + 3, y + 3, w, h, r, C_CARD_SHADOW);

        // 2. Card Body
        tft->fillRoundRect(x, y, w, h, r, C_CARD_BG);

        // 3. Accent Bar (Garis warna di kiri)
        // Kita gambar kotak kecil melengkung di sisi kiri
        tft->fillRoundRect(x, y, 6, h, r, accentColor);
        tft->fillRect(x + 3, y, 3, h, accentColor); // Ratakan sisi kanan aksen agar nyambung ke body

        // 4. Label (Judul Kecil di atas nilai)
        tft->setTextColor(0xBDF7, C_CARD_BG); // Abu terang
        tft->setTextSize(1);
        tft->setCursor(x + 12, y + 8); // Geser agar tidak kena aksen
        tft->print(label);
    }

    // Print Value (Cerdas Menghapus Bekas)
    void printValue(int typeIndex, float value, int decimals, const char* unit, uint16_t color) {
        // typeIndex mempermudah pemanggilan posisi tanpa perlu ingat koordinat X Y
        // 0=Volt, 1=Eff, 2=CurrAvg, 3=CurrMax, 4=EnUsed, 5=EnRegen, 6=Loop, 7=Uptime

        int x, y;
        bool isFooter = false;

        // Mapping Posisi (Hardcoded layout agar rapi)
        switch(typeIndex) {
            // Baris 1
            case 0: x = 10;  y = 60; break;
            case 1: x = 125; y = 60; break;
            // Baris 2
            case 2: x = 10;  y = 115; break;
            case 3: x = 125; y = 115; break;
            // Baris 3
            case 4: x = 10;  y = 170; break;
            case 5: x = 125; y = 170; break;
            // Footer
            case 6: x = 75;  y = 230; isFooter = true; break; // Sebelah label Loop
            case 7: x = 180; y = 230; isFooter = true; break; // Sebelah label Uptime
        }

        if (!isFooter) {
            // --- MODE CARD (Angka Besar) ---
            // Area hapus (di dalam card)
            int valX = x + 12;
            int valY = y + 22; // Di bawah label
            
            // Hapus angka lama dengan warna Card Body
            tft->fillRect(valX, valY, 90, 18, C_CARD_BG);

            // Cetak Angka
            char buffer[16];
            if (value >= 1000) {
               dtostrf(value, 4, 0, buffer);
            } else {
               dtostrf(value, 4, decimals, buffer);
            }
            
            tft->setCursor(valX, valY);
            tft->setTextColor(color, C_CARD_BG);
            tft->setTextSize(2);
            tft->print(buffer);

            // Cetak Satuan (Kecil)
            tft->setTextSize(1);
            tft->setTextColor(0xBDF7, C_CARD_BG);
            tft->setCursor(valX + (strlen(buffer)*12) + 2, valY + 7);
            tft->print(unit);

        } else {
            // --- MODE FOOTER (Angka Kecil) ---
            // Hapus area footer dengan warna Background Utama
            tft->fillRect(x, y, 40, 10, C_BG_MAIN);

            tft->setCursor(x, y);
            tft->setTextColor(TFT_WHITE, C_BG_MAIN);
            tft->setTextSize(1);
            
            if (typeIndex == 7) { // Uptime format (detik -> menit)
                 unsigned long sec = (unsigned long)value;
                 if (sec < 60) {
                     tft->print(sec); tft->print("s");
                 } else if (sec < 3600) {
                     tft->print(sec/60); tft->print("m "); tft->print(sec%60); tft->print("s");
                 } else {
                     tft->print(sec/3600); tft->print("h "); tft->print((sec%3600)/60); tft->print("m");
                 }
            } else {
                 tft->print((int)value);
            }
        }
    }

public:
    Slide3_Dashboard(TFT_eSPI *display) : tft(display) {}

    void begin() {
        refresh();
    }

    void refresh() {
        tft->fillScreen(C_BG_MAIN);
        drawStaticElements();
    }

    void end() {}

    void update(volatile SystemData &data) {
        // Data Utama (Card Besar)
        printValue(0, displayVoltage, 1, "V", TFT_WHITE);      // Volt
        printValue(1, data.efficiency, 1, "%", COLOR_GREEN); // Eff
        printValue(2, data.i_used_avg, 2, "A", COLOR_BLUE);  // Curr Avg
        printValue(3, data.i_used_max, 2, "A", 0x001F);      // Curr Max
        printValue(4, data.wh_used, 2, "Wh", COLOR_RED);  // Energy Used
        printValue(5, data.wh_regen, 2, "Wh", COLOR_CYAN);// Energy Regen
        
        // Data Footer (Teks Kecil)
        if (isLogging) {
            printValue(6, (float)data.iteration, 0, "", TFT_WHITE);     // Loop Iter
            printValue(7, (float)data.uptime_sec, 0, "", TFT_WHITE); // Uptime
        } else {
            // Clear footer values if not logging
             tft->fillRect(75, 230, 40, 10, C_BG_MAIN);
             tft->fillRect(180, 230, 40, 10, C_BG_MAIN);
        }
        
        drawPageNumber(3);
    }

private:
    void drawStaticElements() {
        // --- HEADER (Simpel & Bersih) ---
        tft->setTextColor(TFT_WHITE, C_BG_MAIN);
        tft->setTextSize(2);
        tft->setCursor(10, 15);
        tft->print("SYSTEM LOG");
        
        // Indikator "Live" berkedip (Statik dulu)
        tft->fillCircle(220, 22, 4, CALIB_COLOR_DONE);

        // Garis pemisah tipis
        tft->drawFastHLine(0, 45, 240, 0x3186);

        // --- GRID LAYOUT ---
        int col1 = 10;
        int col2 = 125;
        int w = 105; // Lebar card
        int h = 45;  // Tinggi card

        // Baris 1: TEGANGAN & EFISIENSI (Posisi Y: 60)
        int y1 = 60;
        drawCleanCard(col1, y1, w, h, COLOR_GREEN, "VOLTAGE");
        drawCleanCard(col2, y1, w, h, TFT_WHITE, "EFFICIENCY");

        // Baris 2: ARUS (Posisi Y: 115)
        int y2 = 115;
        drawCleanCard(col1, y2, w, h, COLOR_BLUE, "CURRENT AVG");
        drawCleanCard(col2, y2, w, h, 0x001F, "CURRENT MAX");

        // Baris 3: ENERGI (Posisi Y: 170)
        int y3 = 170;
        drawCleanCard(col1, y3, w, h, COLOR_RED, "ENERGY USED");
        drawCleanCard(col2, y3, w, h, COLOR_CYAN, "ENERGY REGEN");

        // --- FOOTER (SYSTEM STATS) ---
        tft->drawFastHLine(0, 225, 240, 0x3186);
        
        // Label statis untuk footer
        tft->setTextSize(1);
        tft->setTextColor(0x7BEF, C_BG_MAIN);
        
        tft->setCursor(10, 230);
        tft->print("Loop Iter:");
        
        tft->setCursor(130, 230);
        tft->print("Uptime:");
    }
};

/**
 * Slide 4: File Info Display (shown after logging stops)
 */
class Slide4_FileInfo {
private:
    TFT_eSPI *tft;

public:
    Slide4_FileInfo(TFT_eSPI *display) : tft(display) {}

    void begin() {
        refresh();
    }

    void refresh() {
        tft->fillScreen(COLOR_BG);
        drawContent();
    }

    void end() {}

    void update() {
        // No dynamic updates needed for this slide
    }

private:
    void drawContent() {
        // Bersihkan area (opsional, tergantung implementasi parent function)
        // tft->fillScreen(COLOR_BG); 

        // --- 1. KONFIGURASI GEOMETRI SD CARD 3D ---
        int cx = 120;       // Pusat X
        int cy = 70;        // Pusat Y (ikon SD Card)
        
        int sdW = 50;       // Lebar SD Card
        int sdH = 70;       // Tinggi SD Card
        int depth = 6;      // Ketebalan 3D
        int tilt = 10;      // Kemiringan (Skew) ke kanan
        int cutSize = 12;   // Ukuran potongan sudut kanan atas

        // Koordinat Dasar (Front Face)
        // Kita hitung 4 sudut dasar persegi panjang miring
        int topX = cx + tilt; 
        int topY = cy - (sdH / 2);
        int botX = cx;
        int botY = cy + (sdH / 2);
        
        // Sudut-sudut Muka Depan
        int xTL = topX - (sdW / 2);      int yTL = topY;
        int xTR = topX + (sdW / 2);      int yTR = topY; // Ini nanti dipotong
        int xBR = botX + (sdW / 2);      int yBR = botY;
        int xBL = botX - (sdW / 2);      int yBL = botY;

        // --- 2. GAMBAR SD CARD BODY ---
        
        // A. Bayangan/Ketebalan 3D (Sisi Kanan & Bawah)
        // Sisi Kanan
        fillQuad(xTR, yTR + cutSize, xTR + depth, yTR + cutSize, xBR + depth, yBR, xBR, yBR, C_HOUSING_SIDE);
        // Sisi Bawah
        fillQuad(xBR, yBR, xBR + depth, yBR, xBL + depth, yBL, xBL, yBL, C_HOUSING_SIDE);
        // Sudut Potongan (Miring)
        fillQuad(xTR - cutSize, yTR, xTR - cutSize + depth, yTR, xTR + depth, yTR + cutSize, xTR, yTR + cutSize, C_HOUSING_SIDE);

        // B. Muka Depan (Main Body) - Warna Gelap (Housing)
        // Kita gambar Rect penuh dulu, nanti "ditambal" background untuk potongan sudut
        fillQuad(xTL, yTL, xTR, yTR, xBR, yBR, xBL, yBL, C_HOUSING_FRONT);
        
        // C. Buat Efek "Cut Corner" (Potong sudut kanan atas)
        // Caranya: Timpa segitiga pojok kanan atas dengan warna background
        tft->fillTriangle(xTR, yTR, xTR, yTR + cutSize, xTR - cutSize, yTR, COLOR_BG); 

        // D. Stiker Label (Area terang di tengah SD Card)
        int margin = 5;
        fillQuad(xTL + margin, yTL + 15, xTR - margin - cutSize + 2, yTL + 15, 
                xBR - margin, yBR - 10, xBL + margin, yBR - 10, C_POLE); // Warna abu terang

        // E. Pins (Kuningan di bawah)
        int pinW = 4; int pinGap = 2;
        int startPinX = xBL + 10;
        int startPinY = yBR - 8; // Sedikit di atas bawah
        for(int i=0; i<7; i++) { // 7 Pin standar SD Card
            int px = startPinX + (i * (pinW + pinGap));
            // Sesuaikan kemiringan pin
            fillQuad(px, startPinY, px + pinW, startPinY, 
                    px + pinW - 2, yBR - 2, px - 2, yBR - 2, TFT_ORANGE);
        }

        // --- 3. GAMBAR CEKLIS (ANIMASI STROKE) ---
        // Ceklis berada di tengah ikon SD Card
        int checkCX = cx; 
        int checkCY = cy + 5;
        int thick = 4; // Ketebalan garis ceklis

        // Kita gambar lingkaran latar hijau dulu (seperti stiker QC Pass)
        tft->fillCircle(checkCX, checkCY, 22, CALIB_COLOR_DONE);
        tft->drawCircle(checkCX, checkCY, 22, TFT_WHITE); // Ring putih

        // Animasi Menggambar Ceklis (Putih)
        // Titik sudut ceklis: (Kiri, Bawah, KananAtas)
        int p1x = checkCX - 10; int p1y = checkCY;
        int p2x = checkCX - 2;  int p2y = checkCY + 8;
        int p3x = checkCX + 12; int p3y = checkCY - 8;

        // Stroke 1: Turun
        for(int i=0; i<=8; i+=2) {
            tft->fillCircle(p1x + i, p1y + i, 2, TFT_WHITE); // Pakai circle biar ujungnya bulat
            // delay(5); // Opsional: delay mikro untuk efek (hati-hati blocking)
        }
        // Stroke 2: Naik
        for(int i=0; i<=14; i+=2) {
            tft->fillCircle(p2x + i, p2y - i, 2, TFT_WHITE);
        }
        // Rapikan sudut pertemuan
        tft->fillCircle(p2x, p2y, 2, TFT_WHITE);


        // --- 4. TIPOGRAFI & DATA (CARD UI STYLE) ---
        
        // HEADER: TERSIMPAN
        tft->setTextColor(CALIB_COLOR_DONE, COLOR_BG);
        tft->setTextSize(2); // Pastikan font mendukung
        String title = "TERSIMPAN!";
        // Geser Y ke 120 agar tidak nabrak ikon
        tft->setCursor((240 - tft->textWidth(title)) / 2, 120); 
        tft->print(title);

        // KOTAK INFO FILE (Background tipis agar rapi)
        int infoBoxY = 145;
        int infoBoxH = 55;
        // Garis pemisah atas & bawah estetik
        tft->drawFastHLine(20, infoBoxY, 200, TFT_GREY);
        tft->drawFastHLine(20, infoBoxY + infoBoxH, 200, TFT_GREY);

        // File Name Label (Kecil, Abu/Biru Muda)
        tft->setTextSize(1);
        tft->setTextColor(COLOR_TEXT_L, COLOR_BG);
        tft->setCursor(30, infoBoxY + 8);
        tft->print("NAMA FILE:");

        // File Name Value (Besar/Putih)
        tft->setTextSize(1); // Atau 2 jika nama file pendek
        tft->setTextColor(COLOR_TEXT_V, COLOR_BG);
        tft->setCursor(30, infoBoxY + 20);
        if (lastSavedFilename.length() > 0) {
            tft->print(lastSavedFilename);
        } else {
            tft->setTextColor(CALIB_COLOR_WARN, COLOR_BG); // Merah jika error
            tft->print("ERROR / NO FILE");
        }

        // Size Value (Di kanan, sejajar nama file)
        // Trik: Cetak label SIZE di kanan
        tft->setTextColor(COLOR_TEXT_L, COLOR_BG);
        tft->setCursor(150, infoBoxY + 8);
        tft->print("UKURAN:");
        
        tft->setTextColor(COLOR_TEXT_V, COLOR_BG);
        tft->setCursor(150, infoBoxY + 20);
        float fileSizeKB = lastSavedFileSize / 1024.0;
        char sizeBuffer[16];
        dtostrf(fileSizeKB, 1, 1, sizeBuffer); // 1 desimal cukup
        tft->print(sizeBuffer);
        tft->print(" kB");

        // FOOTER (Instruksi)
        // Gunakan warna redup atau C_POLE_SHADE
        tft->setTextSize(1);
        tft->setCursor((240 - tft->textWidth("Tekan tombol untuk lanjut")) / 2, 220);
        tft->setTextColor(TFT_WHITE, COLOR_BG);
        tft->print("Tekan tombol untuk lanjut");
        
        // drawPageNumber(4); // Removed as requested
    }
};

// ============================================================================
// SD CARD & SERIAL FUNCTIONS
// ============================================================================

/**
 * Initialize SD card on HSPI pins with proper reset sequence
 * This function includes retry logic to handle SD card state issues after power cycles
 */
void initSDCard() {
    sdReady = false;
    
    Serial.println(">> Initializing SD Card...");
    
    // Step 1: Set CS pin HIGH to deselect SD card (important for reset)
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    delay(10);
    
    // Step 2: End any previous SPI/SD session to clear state
    sdSPI.end();
    SD.end();
    delay(100);  // Allow SD card internal state to reset
    
    // Step 3: Initialize SPI bus
    sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(50);
    
    // Step 4: Try to initialize SD card with retry mechanism
    const int maxRetries = 3;
    const unsigned long spiSpeed = 1000000;  // 1MHz for better reliability
    
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        Serial.printf(">> SD Card init attempt %d/%d...\n", attempt, maxRetries);
        
        // Toggle CS pin before each attempt
        digitalWrite(SD_CS_PIN, HIGH);
        delay(10);
        digitalWrite(SD_CS_PIN, LOW);
        delay(10);
        digitalWrite(SD_CS_PIN, HIGH);
        delay(50);
        
        // Try to begin SD card
        if (SD.begin(SD_CS_PIN, sdSPI, spiSpeed)) {
            uint8_t cardType = SD.cardType();
            
            if (cardType != CARD_NONE) {
                sdReady = true;
                
                // Print card info
                Serial.println(">> SD Card OK!");
                Serial.print(">> Card Type: ");
                if (cardType == CARD_MMC) {
                    Serial.println("MMC");
                } else if (cardType == CARD_SD) {
                    Serial.println("SDSC");
                } else if (cardType == CARD_SDHC) {
                    Serial.println("SDHC");
                } else {
                    Serial.println("UNKNOWN");
                }
                
                uint64_t cardSize = SD.cardSize() / (1024 * 1024);
                Serial.printf(">> Card Size: %lluMB\n", cardSize);
                
                return;  // Success!
            } else {
                Serial.println(">> No SD card detected");
            }
        }
        
        // If not last attempt, wait before retry
        if (attempt < maxRetries) {
            Serial.println(">> Retrying...");
            SD.end();
            delay(200);
        }
    }
    
    // All attempts failed
    Serial.println(">> SD Card FAIL (Check card/pins/connections)");
    Serial.println(">> Will retry on next logging attempt");
}

/**
 * Start data logging to SD card
 */
void startLogging() {
    // Check if SD card is ready, if not try to re-initialize
    if (!sdReady) {
        Serial.println(">> SD not ready, attempting re-init...");
        initSDCard();
        
        if (!sdReady) {
            Serial.println(">> Cannot start logging - SD Card unavailable");
            drawSDError("E-01", "Card Missing");
            delay(2000);
            return;
        }
    }
    
    String filename;
    
    if (rtcReady) {
        DateTime now = rtc.now();
        char buffer[20];
        sprintf(buffer, "/%02d%02d%02d%02d.csv", now.month(), now.day(), now.hour(), now.minute());
        filename = String(buffer);
    } else {
        filename = "/log_" + String(millis()) + ".csv";
    }
    
    currentLogFile = SD.open(filename, FILE_WRITE);
    
    if (currentLogFile) {
        currentLogFile.println("No,V,I,P,Wh_U,Wh_G,Time");
        logIteration = 0;
        sysData.wh_used = 0;
        sysData.wh_regen = 0;
        isLogging = true;
        
        Serial.println(">> Logging started: " + filename);
        drawStartLoggingUI();
    } else {
        Serial.println(">> Error: Cannot create/write file");
        Serial.println(">> Filename: " + filename);
        // SD might have failed, mark as not ready and show error
        sdReady = false;
        drawSDError("E-02", "Write Failed");
        delay(2000);
    }
}

/**
 * Stop data logging
 */
void stopLogging() {
    isLogging = false;
    
    bool savedSuccessfully = false;
    
    if (currentLogFile) {
        // Get file info before closing
        lastSavedFileSize = currentLogFile.size();
        lastSavedFilename = currentLogFile.name();
        currentLogFile.close();
        
        // Verify file was actually saved (size > 0 means data was written)
        if (lastSavedFileSize > 0 && sdReady) {
            savedSuccessfully = true;
            hasLoggedBefore = true;
            showFileInfoSlide = true;
            
            Serial.println(">> Logging stopped");
            Serial.printf(">> File: %s (%lu bytes)\n", lastSavedFilename.c_str(), lastSavedFileSize);
        } else {
            Serial.println(">> Logging stopped");
            Serial.println(">> WARNING: File may be empty or SD error occurred");
        }
    }
    
    // Only show success animation if actually saved
    if (savedSuccessfully) {
        drawStopLoggingUI();
    } else {
        // Show error if stopped but not saved properly
        drawSDError("E-05", "Save Incomplete");
        delay(2000);
    }
}

// ============================================================================
// SD CARD HELPER FUNCTIONS
// ============================================================================
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println(F("Failed to open directory"));
        return;
    }
    if(!root.isDirectory()){
        Serial.println(F("Not a directory"));
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print(F("  DIR : "));
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print(F("  FILE: "));
            Serial.print(file.name());
            Serial.print(F("  SIZE: "));
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void deleteFile(fs::FS &fs, const char * path) {
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println(F("File deleted"));
    } else {
        Serial.println(F("Delete failed"));
    }
}

void deleteAllFiles(fs::FS &fs) {
    Serial.println(F(">> Deleting ALL files..."));
    File root = fs.open("/");
    if(!root){
        Serial.println(F("Failed to open root"));
        return;
    }
    
    File file = root.openNextFile();
    while(file){
        if(!file.isDirectory()){
             deleteFile(fs, file.path());
        }
        file = root.openNextFile();
    }
    Serial.println(F(">> All files removed"));
    
    // Refresh list
    listDir(fs, "/", 0);
}

/**
 * Process serial commands
 */
void processSerial(String input) {
    input.trim();
    
    if (input.length() == 0) return;
    
    // Echo received command
    Serial.print(F("> INPUT: \""));
    Serial.print(input);
    Serial.println(F("\""));
    
    // --- CALIBRATION COMMANDS (OLD) ---
    
    // Command: reset_all
    if (input.equalsIgnoreCase("reset_all")) {
        Serial.println(F(">> COMMAND: RESET_ALL"));
        Serial.println(F(">> ACTION: Resetting all calibration data..."));
        
        prefs.clear();
        calib_curr_done = false;
        calib_volt_count = 0;
        calib_rtc_done = false;
        system_calibrated = false;
        calib_ui_update = true;
        
        Serial.println(F(">> STATUS: ✓ All calibration cleared"));
        Serial.println(F(">> INFO: System will restart calibration process"));
    }
    // Command: auto
    else if (input.equalsIgnoreCase("auto")) {
        Serial.println(F(">> COMMAND: AUTO"));
        Serial.println(F(">> ACTION: Auto current offset calibration..."));
        
        if (!calib_curr_done) {
            calib_curr_done = true;
            prefs.putBool("c_curr", true);
            calib_ui_update = true;
            calculateAutoOffset();
            
            Serial.println(F(">> STATUS: ✓ Current offset calibrated"));
            Serial.printf(">> RESULT: Offset = %.4f\n", I_offset);
        } else {
            Serial.println(F(">> STATUS: ⚠ Already calibrated"));
            Serial.println(F(">> INFO: Use 'reset_all' to recalibrate"));
        }
    }
    // Command: v<voltage> (e.g. v12.5)
    else if (input.startsWith("v")) {
        Serial.println(F(">> COMMAND: VOLTAGE CALIBRATION"));
        float voltage = input.substring(1).toFloat();
        
        Serial.printf(">> PARAM: Voltage = %.2f V\n", voltage);
        
        if (voltage > 0 && calib_volt_count < 3) {
            Serial.printf(">> ACTION: Adding calibration point #%d...\n", calib_volt_count + 1);
            
            addOrUpdateCalPoint(voltage);
            calib_volt_count++;
            prefs.putInt("c_volt_n", calib_volt_count);
            calib_ui_update = true;
            
            Serial.printf(">> STATUS: ✓ Point added (%d/3)\n", calib_volt_count);
            
            if (calib_volt_count >= 3) {
                Serial.println(F(">> INFO: Voltage calibration complete!"));
            } else {
                Serial.printf(">> INFO: Need %d more points\n", 3 - calib_volt_count);
            }
        } 
        else if (voltage <= 0) {
            Serial.println(F(">> STATUS: ✗ INVALID"));
            Serial.println(F(">> ERROR: Voltage must be > 0"));
            Serial.println(F(">> USAGE: v<voltage> (e.g., v12.5)"));
        }
        else if (calib_volt_count >= 3) {
            Serial.println(F(">> STATUS: ⚠ LIMIT REACHED"));
            Serial.println(F(">> ERROR: Already have 3 calibration points"));
            Serial.println(F(">> INFO: Use 'reset_all' to restart calibration"));
        }
    }
    // Command: SET_TIME YYYY MM DD HH MM SS
    else if (input.startsWith("SET_TIME")) {
        Serial.println(F(">> COMMAND: SET_TIME"));
        
        int year, month, day, hour, minute, second;
        if (sscanf(input.c_str(), "SET_TIME %d %d %d %d %d %d", 
                &year, &month, &day, &hour, &minute, &second) == 6) {
            
            Serial.printf(">> PARAM: %04d-%02d-%02d %02d:%02d:%02d\n", 
                         year, month, day, hour, minute, second);
            Serial.println(F(">> ACTION: Setting RTC time..."));
            
            rtc.adjust(DateTime(year, month, day, hour, minute, second));
            calib_rtc_done = true;
            prefs.putBool("c_rtc", true);
            calib_ui_update = true;
            
            Serial.println(F(">> STATUS: ✓ RTC time set successfully"));
            Serial.println(F(">> INFO: Time calibration complete"));
        } 
        else {
            Serial.println(F(">> STATUS: ✗ INVALID FORMAT"));
            Serial.println(F(">> ERROR: Failed to parse time parameters"));
            Serial.println(F(">> USAGE: SET_TIME YYYY MM DD HH MM SS"));
            Serial.println(F(">> EXAMPLE: SET_TIME 2024 11 26 14 30 00"));
        }
    }
    
    // --- FILE MANAGEMENT COMMANDS (NEW) ---
    
    // Command: ls
    else if (input.equalsIgnoreCase("ls")) {
        if (sdReady) {
            listDir(SD, "/", 0);
        } else {
            Serial.println(F(">> Error: SD Card not ready"));
        }
    }
    // Command: remove_all
    else if (input.equalsIgnoreCase("remove_all")) {
        if (sdReady) {
            deleteAllFiles(SD);
        } else {
            Serial.println(F(">> Error: SD Card not ready"));
        }
    }
    // Command: remove <filename>
    else if (input.startsWith("remove ")) {
        String fname = input.substring(7);
        fname.trim();
        if (sdReady) {
            if (!fname.startsWith("/")) fname = "/" + fname;
            deleteFile(SD, fname.c_str());
            // Show list after delete
            listDir(SD, "/", 0);
        } else {
            Serial.println(F(">> Error: SD Card not ready"));
        }
    }
    // Command: status
    else if (input.equalsIgnoreCase("status")) {
        Serial.println(F(">> SYSTEM STATUS:"));
        Serial.printf("   SD Card: %s\n", sdReady ? "READY" : "ERROR/MISSING");
        Serial.printf("   RTC: %s\n", rtcReady ? "READY" : "ERROR");
        Serial.printf("   Logging: %s\n", isLogging ? "ACTIVE" : "STOPPED");
        Serial.printf("   Calibrated: %s\n", system_calibrated ? "YES" : "NO");
        Serial.printf("   Uptime: %lu s\n", sysData.uptime_sec);
    }
    // Command: help
    else if (input.equalsIgnoreCase("help") || input.equalsIgnoreCase("?")) {
        Serial.println(F(">> AVAILABLE COMMANDS:"));
        Serial.println(F("   ls             : List files on SD card"));
        Serial.println(F("   remove <file>  : Delete specific file"));
        Serial.println(F("   remove_all     : Delete ALL files"));
        Serial.println(F("   status         : Show system status"));
        Serial.println(F("   reset_all      : Reset calibration"));
        Serial.println(F("   auto           : Auto calibrate current"));
        Serial.println(F("   v<voltage>     : Add voltage point (e.g. v12.5)"));
        Serial.println(F("   SET_TIME ...   : Set RTC time"));
        Serial.println(F("   help           : Show this list"));
    }
    else {
        // Unknown command
        Serial.println(">> STATUS: ✗ UNKNOWN COMMAND");
        Serial.print(">> ERROR: Command '");
        Serial.print(input);
        Serial.println("' not recognized");
        Serial.println(">> HINT: Type 'help' or '?' for available commands");
        Serial.println();
    }
}

// ============================================================================
// CALIBRATION UI
// ============================================================================

/**
 * Draw checkmark icon
 */
void drawCheck(int x, int y, uint16_t color) {
    tft.drawLine(x, y + 6, x + 4, y + 10, color);
    tft.drawLine(x + 1, y + 6, x + 5, y + 10, color);
    tft.drawLine(x + 4, y + 10, x + 12, y - 2, color);
    tft.drawLine(x + 5, y + 10, x + 13, y - 2, color);
}

/**
 * Draw calibration item
 */
int drawItem(int y, int idx, String title, String instruction, bool isDone, int progress = -1) {
    if (isDone) {
        tft.setTextColor(CALIB_COLOR_DONE, COLOR_BG);
        tft.setTextSize(2);
        tft.setCursor(5, y);
        tft.print(idx);
        
        drawCheck(25, y, CALIB_COLOR_DONE);
        
        tft.setTextSize(1);
        tft.setCursor(45, y + 4);
        tft.print(title);
        
        tft.setCursor(30, y);
        tft.setTextColor(CALIB_COLOR_DONE, COLOR_BG);
        tft.setTextSize(2);
        tft.print(title);
        
        tft.drawFastHLine(45, y + 18, 150, 0x18E3);
        return 30;
    }
    
    tft.setTextColor(CALIB_COLOR_PENDING, COLOR_BG);
    tft.setTextSize(2);
    tft.setCursor(5, y);
    tft.print(idx);
    
    tft.setCursor(30, y);
    tft.print(title);
    
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT_L, COLOR_BG);
    tft.setCursor(30, y + 20);
    tft.print(instruction);
    
    if (progress != -1) {
        tft.setCursor(30, y + 32);
        for (int i = 0; i < 3; i++) {
            if (i < progress) {
                tft.setTextColor(CALIB_COLOR_DONE);
            } else {
                tft.setTextColor(CALIB_COLOR_DIM);
            }
            tft.print("[*] ");
        }
    }
    
    return (progress != -1) ? 50 : 40;
}

/**
 * Draw calibration screen
 */
void drawCalibrationScreen() {
    tft.fillScreen(COLOR_BG);
    
    int headY = 10;
    
    // Warning header
    tft.fillRect(0, headY, 240, 40, 0x2104);
    tft.drawRect(0, headY, 240, 40, CALIB_COLOR_WARN);
    tft.setTextSize(2);
    tft.setTextColor(CALIB_COLOR_WARN, 0x2104);
    
    String title = "PERINGATAN!";
    tft.setCursor((240 - tft.textWidth(title)) / 2, headY + 8);
    tft.print(title);
    
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, 0x2104);
    tft.setCursor(80, headY + 28);
    tft.print("SYSTEM LOCKED");
    
    int curY = headY + 65;
    
    // Calibration items
    curY += drawItem(curY, 1, "Kalibrasi Arus", "Pastikan 0A, ketik 'auto'", calib_curr_done);
    
    bool voltDone = (calib_volt_count >= 3);
    curY += drawItem(curY, 2, "Kalibrasi Volt", "Input 3x: 'v12.5' dst", voltDone, calib_volt_count);
    
    curY += drawItem(curY, 3, "Setup Waktu", "Ketik 'SET_TIME...'", calib_rtc_done);
    
    // Footer
    tft.drawFastHLine(0, 215, 240, 0x3186);
    tft.setCursor(0, 225);
    tft.setTextSize(1);
    tft.setTextColor(CALIB_COLOR_WARN, COLOR_BG);
    tft.setCursor(65, 225);
    tft.print("MENUNGGU INPUT...");
}

/**
 * Draw success screen
 */
void drawSuccessScreen() {
    tft.fillScreen(CALIB_COLOR_DONE);
    
    int cx = 120;
    int cy = 100;
    
    // Animated circle
    for (int r = 0; r <= 45; r += 5) {
        tft.fillCircle(cx, cy, r, TFT_WHITE);
        delay(10);
    }
    
    tft.fillCircle(cx, cy, 40, TFT_WHITE);
    
    // Animated checkmark
    for (int i = 0; i < 15; i++) {
        tft.fillCircle(cx - 30 + i, cy + i, 4, CALIB_COLOR_DONE);
        delay(10);
    }
    
    for (int i = 0; i < 40; i++) {
        tft.fillCircle(cx - 15 + i, cy + 15 - i, 4, CALIB_COLOR_DONE);
        delay(5);
    }
    
    tft.setTextColor(TFT_BLACK, CALIB_COLOR_DONE);
    tft.setTextSize(2);
    
    String msg = "Selesai Kalibrasi!";
    tft.setCursor((240 - tft.textWidth(msg)) / 2, 165);
    tft.print(msg);
    
    delay(1000);
}

// ============================================================================
// DATA TASK (CORE 0)
// ============================================================================

/**
 * Data acquisition task running on Core 0
 */
void dataTask(void *parameter) {
    unsigned long lastLogTime = 0;
    unsigned long startTime = millis();
    
    sysData.time_start = startTime;
    sysData.i_used_min = 999;
    
    for (;;) {
        // Read voltage
        float rawBattery = getMedianADC(batteryPin, 15);
        
        if (sensorSystemValid) {
            sysData.voltage = getInterpolatedVoltage(rawBattery);
        } else {
            float v_adc = (rawBattery / ADC_resolution) * V_ref;
            sysData.voltage = v_adc * 11.0;
        }
        
        // Apply smoothing for display (reduce decimal flickering from noise)
        if (displayVoltage == 0.0) {
            displayVoltage = sysData.voltage; // Initialize
        } else {
            // Exponential moving average: slower for decimals, faster for whole numbers
            displayVoltage = (VOLTAGE_ALPHA * sysData.voltage) + ((1.0 - VOLTAGE_ALPHA) * displayVoltage);
        }

        
        // Read current
        float rawCurrent = getMedianADC(sensorPin, 20);
        float V_adc_current = (rawCurrent / ADC_resolution) * V_ref;
        float V_sensor = (V_adc_current / I_divider_factor) * I_scale_factor;
        float I_raw = (V_sensor - I_offset) / I_sensitivity;
        
        // Apply dead zone
        if (abs(I_raw) < 0.15) I_raw = 0.0;
        if (sysData.voltage < 3.0) I_raw = 0.0;
        
        // Separate into used and regen current
        if (I_raw >= 0) {
            sysData.i_used = I_raw;
            sysData.i_regen = 0;
        } else {
            sysData.i_used = 0;
            sysData.i_regen = abs(I_raw);
        }
        
        // Track max/min current
        if (sysData.i_used > sysData.i_used_max) {
            sysData.i_used_max = sysData.i_used;
        }
        
        if (sysData.i_used < sysData.i_used_min && sysData.i_used > 0) {
            sysData.i_used_min = sysData.i_used;
        }
        
        // Calculate average current (exponential moving average)
        static float alpha = 0.1;  // Smoothing factor
        static bool avgInitialized = false;
        if (!avgInitialized && sysData.i_used > 0) {
            sysData.i_used_avg = sysData.i_used;
            avgInitialized = true;
        } else if (sysData.i_used > 0) {
            sysData.i_used_avg = (alpha * sysData.i_used) + ((1.0 - alpha) * sysData.i_used_avg);
        }
        
        // Calculate energy (Wh)
        static unsigned long lastWh = 0;
        unsigned long now = millis();
        
        if (now - lastWh > 100) {
            float hours = (now - lastWh) / 3600000.0;
            sysData.wh_used += (sysData.voltage * sysData.i_used * hours);
            sysData.wh_regen += (sysData.voltage * sysData.i_regen * hours);
            lastWh = now;
        }
        
        // Calculate efficiency
        if (sysData.wh_used > 0.001) {
            sysData.efficiency = (sysData.wh_regen / sysData.wh_used) * 100.0;
        } else {
            sysData.efficiency = 0;
        }
        
        if (sysData.efficiency > 100.0) {
            sysData.efficiency = 100.0;
        }
        
        // Update time
        sysData.uptime_sec = (now - startTime) / 1000;
        
        if (rtcReady) {
            sysData.time_now = rtc.now().unixtime() * 1000;
        } else {
            sysData.time_now = now;
        }
        
        sysData.iteration++;
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// SETUP & LOOP (CORE 1)
// ============================================================================

// Chart instances
Slide1_RTChart slide1(&tft);
Slide2_EnergyChart slide2(&tft);
Slide3_Dashboard slide3(&tft);
Slide4_FileInfo slide4(&tft);

// Channel configurations
ChannelConfig s1_voltage = {"Tegangan (V)", "", COLOR_GREEN, 60.0};
ChannelConfig s1_regen = {"Arus Regen (A)", "", COLOR_CYAN, 10.0};
ChannelConfig s1_used = {"Arus Used (A)", "", COLOR_BLUE, 10.0};

ChannelConfig s2_voltage = {"Tegangan (V)", "", COLOR_GREEN, 60.0};
ChannelConfig s2_energyUsed = {"Energy Used (Wh)", "", COLOR_RED, 100.0};
ChannelConfig s2_energyRegen = {"Energy Regen (Wh)", "", COLOR_CYAN, 50.0};

// UI state
int currentSlide = 1;
unsigned long lastSlideChange = 0;
unsigned long lastSDWrite = 0;
unsigned long lastSDCheck = 0; // For hot-swap detection

// Check SD card status (Hot-swap detection)
void checkSDCardStatus() {
    // Check frequently (every 250ms) for responsiveness
    if (millis() - lastSDCheck < 250) return;
    lastSDCheck = millis();

    bool detected = false;
    
    // 1. Cek status fisik/logis
    if (sdReady) {
        // Jika sistem mengira SD ada, verifikasi apakah masih merespon
        if (SD.cardType() == CARD_NONE) {
            detected = false;
        } else {
            detected = true;
        }
    } else {
        // Jika sistem mengira SD tidak ada, coba inisialisasi ulang
        SD.end(); // Bersihkan state sebelumnya (PENTING!)
        if (SD.begin(SD_CS_PIN, sdSPI, 1000000)) {
            // Double check dengan cardType
            if (SD.cardType() != CARD_NONE) {
                detected = true;
            }
        }
    }
    
    // 2. Handle State Transitions
    if (!detected) {
        // Case 1: SD Card Removed / Error
        // Trigger jika sebelumnya ready ATAU jika error belum ditampilkan
        if (sdReady || !sdErrorOccurred) {
            sdReady = false;
            sdErrorOccurred = true;
            sdErrorCode = "E-01";
            sdErrorMsg = "SD Dicabut";
            
            // Show Error UI immediately
            drawSDError(sdErrorCode, sdErrorMsg);
        }
    } 
    else {
        // Case 2: SD Card Present (Recovery)
        if (!sdReady) { // Was missing, now found
            sdReady = true;
            sdErrorOccurred = false;
            
            // Show Success UI
            float cardSize = SD.cardSize() / (1024.0 * 1024.0 * 1024.0);
            drawSDSuccess(cardSize, "FAT32");
            delay(2000); // Show success for 2s
            
            // Restore slide
            if (currentSlide == 1) slide1.refresh();
            else if (currentSlide == 2) slide2.refresh();
            else if (currentSlide == 3) slide3.refresh();
        }
    }
}

void setup() {
    // ========================================================
    // PRIORITY #1: DISPLAY INIT (Minimize white flash time)
    // ========================================================
    // Init display FIRST before anything else to reduce visible flash
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    
    tft.init();
    tft.setRotation(0);
    
    // IMMEDIATE black fill - this is the key to preventing white flash!
    tft.fillScreen(COLOR_BG);
    tft.fillScreen(COLOR_BG); // Double fill for stubborn displays
    
    // ========================================================
    // PRIORITY #2: Serial and other initialization
    // ========================================================
    // Now that screen is black, we can do slower init tasks
    Serial.begin(115200);
    delay(500);  // Wait for serial and power stabilization
    
    Serial.println(F("\n========================================"));
    Serial.println(F("  INDUSTRIAL POWER MONITOR v2.0"));
    Serial.println(F("========================================\n"));
    
    // I2C for RTC
    Wire.begin();
    
    // RTC initialization
    if (!rtc.begin()) {
        Serial.println(F("RTC Fail"));
        rtcReady = false;
    } else {
        rtcReady = true;
    }
    
    // SD card initialization (with power-on delay)
    delay(200);  // Additional delay for SD card power-on
    initSDCard();
    
    // Load calibration preferences
    prefs.begin("calib_ui", false);
    calPrefs.begin("sensor_cal", true);
    
    calib_curr_done = prefs.getBool("c_curr", false);
    calib_volt_count = prefs.getInt("c_volt_n", 0);
    calib_rtc_done = prefs.getBool("c_rtc", false);
    
    I_offset = calPrefs.getFloat("i_off", I_offset);
    
    calPrefs.end();
    
    // Run system hardware check at boot
    runSystemCheck();
    
    // Print startup status
    Serial.println(F(">> SYSTEM READY"));
    Serial.printf(">> SD Status: %s\n", sdReady ? "READY" : "MISSING/ERROR");
    Serial.println(F(">> Type 'help' for available commands"));
    
    // Check if system is calibrated
    if (calib_curr_done && calib_volt_count >= 3 && calib_rtc_done) {
        system_calibrated = true;
        chartsInitialized = true;
        slide1.begin(200, 130, s1_voltage, s1_regen, s1_used);
    } else {
        drawCalibrationScreen();
    }
    
    // Start data acquisition task on Core 0
    xTaskCreatePinnedToCore(dataTask, "DataTask", 8192, NULL, 1, NULL, 0);
}

void loop() {
    // PRIORITY: Check for SD errors and display immediately
    if (sdErrorOccurred) {
        // Only auto-clear transient errors (NOT E-01 "SD Dicabut")
        // E-01 is persistent until checkSDCardStatus detects recovery
        if (sdErrorCode != "E-01") {
            drawSDError(sdErrorCode, sdErrorMsg);
            delay(2000);
            sdErrorOccurred = false; // Clear flag for transient errors
            
            // Restore slide
            if (system_calibrated && chartsInitialized) {
                if (currentSlide == 1) slide1.refresh();
                else if (currentSlide == 2) slide2.refresh();
                else if (currentSlide == 3) slide3.refresh();
            }
        }
    }
    
    // Process serial commands
    if (Serial.available()) {
        processSerial(Serial.readStringUntil('\n'));
    }
    
    // Button debouncing
    int reading = digitalRead(BUTTON_PIN);
    
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != buttonState) {
            buttonState = reading;
            
            if (buttonState == LOW) {
                // Toggle logging
                if (isLogging) {
                    stopLogging();
                    
                    // Show file info slide immediately after logging stops
                    if (system_calibrated && showFileInfoSlide) {
                        if (currentSlide == 1) slide1.end();
                        else if (currentSlide == 2) slide2.end();
                        else if (currentSlide == 3) slide3.end();
                        
                        currentSlide = 4;
                        slide4.begin();
                        lastSlideChange = millis();  // Reset timer
                    }
                } else {
                    startLogging();
                }
                
                // Refresh current slide
                if (system_calibrated && currentSlide != 4) {
                    if (currentSlide == 1) {
                        slide1.refresh();
                    } else if (currentSlide == 2) {
                        slide2.refresh();
                    } else if (currentSlide == 3) {
                        slide3.refresh();
                    }
                }
            }
        }
    }
    
    lastButtonState = reading;
    
    // Calibration mode
    if (!system_calibrated) {
        if (calib_curr_done && calib_volt_count >= 3 && calib_rtc_done) {
            drawSuccessScreen();
            delay(2000);
            tft.fillScreen(COLOR_BG);
            
            slide1.begin(200, 130, s1_voltage, s1_regen, s1_used);
            system_calibrated = true;
            chartsInitialized = true;
        } else if (calib_ui_update) {
            drawCalibrationScreen();
            calib_ui_update = false;
        }
        
        delay(100);
        return;
    }
    
    // SD card data logging (Check every 200ms for faster error detection)
    if (isLogging && sdReady && (millis() - lastSDWrite > 200)) {
        lastSDWrite = millis();
        
        if (currentLogFile) {
            // Try to write data
            size_t written = currentLogFile.printf("%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%lu\n",
                ++logIteration,
                sysData.voltage,
                (sysData.i_used - sysData.i_regen),
                (sysData.voltage * (sysData.i_used - sysData.i_regen)),
                sysData.wh_used,
                sysData.wh_regen,
                sysData.time_now);
            
            // Check if write was successful
            if (written == 0) {
                // Write failed - set error flag for immediate display
                Serial.println(">> Error: File write failed during logging");
                currentLogFile.close();
                isLogging = false;
                sdReady = false;
                
                // Set flag for priority display in next loop iteration
                sdErrorOccurred = true;
                sdErrorCode = "E-03";
                sdErrorMsg = "Save Error";
            } else {
                if (logIteration % 5 == 0) {
                    currentLogFile.flush();
                }
                
                // Blink LED indicator
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            }
        } else {
            // File is not open but logging is active - error state
            Serial.println(">> Error: Log file not available");
            isLogging = false;
            sdReady = false;
            sdErrorCode = "E-04";
            sdErrorMsg = "File Lost";
        }
    } else {
        // Monitor SD card hot-swap when not logging
        checkSDCardStatus();
        
        // Only run slide auto-change if charts are initialized
        if (chartsInitialized) {
            // Auto slide change (every 5 seconds)
            if (millis() - lastSlideChange > 5000) {
                lastSlideChange = millis();
                
                // End current slide
                if (currentSlide == 1) {
                    slide1.end();
                } else if (currentSlide == 2) {
                    slide2.end();
                } else if (currentSlide == 3) {
                    slide3.end();
                } else if (currentSlide == 4) {
                    slide4.end();
                    showFileInfoSlide = false;  // Only show once
                }
                
                // Next slide
                currentSlide++;
                
                // Skip slide 4 unless flag is set
                if (currentSlide == 4 && !showFileInfoSlide) {
                    currentSlide = 1;
                }
                
                if (currentSlide > 4) currentSlide = 1;
                
                // Begin new slide
                if (currentSlide == 1) {
                    slide1.begin(200, 130, s1_voltage, s1_regen, s1_used);
                } else if (currentSlide == 2) {
                    slide2.begin(200, 130, s2_voltage, s2_energyUsed, s2_energyRegen);
                } else if (currentSlide == 3) {
                    slide3.begin();
                } else if (currentSlide == 4) {
                    slide4.begin();
                }
            }
        }
    }
    
    // Update current slide ONLY if no SD error is blocking the view
    if (!sdErrorOccurred) {
        if (currentSlide == 1) {
            slide1.update(displayVoltage, sysData.i_regen, sysData.i_used);
        } else if (currentSlide == 2) {
            slide2.update(displayVoltage, sysData.wh_used, sysData.wh_regen, sysData.efficiency);
        } else if (currentSlide == 3) {
            slide3.update(sysData);
            delay(100);
        } else if (currentSlide == 4) {
            slide4.update();
            delay(100);
        }
    }
}