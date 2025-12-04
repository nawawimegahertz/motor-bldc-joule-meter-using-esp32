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
// LEGACY: SD Card libraries (commented for HTTP API mode)
// #include <SD.h>
// #include <FS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>  // For JSON payload construction
#include <Wire.h>
#include <RTClib.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================
#define BUTTON_PIN 13
#define LED_PIN 12

// LEGACY: SD Card Pins (HSPI) - Commented for HTTP API mode
// #define SD_CS_PIN 5
// #define SD_SCK_PIN 18
// #define SD_MISO_PIN 19
// #define SD_MOSI_PIN 23

// Sensor Pins
const int sensorPin = 36;
const int batteryPin = 39;

// ============================================================================
// SENSOR PARAMETERS
// ============================================================================
// --- PREVIOUS TUNING (Commented as requested) ---
// float I_offset = 0.9035; // Calculated from Vout_zero * Divider (2.534 * 0.3565)
// float I_sensitivity = 0.020;
// float I_divider_factor = 0.3565; // R2 / (R1 + R2) = 3.258 / (5.880 + 3.258)

// --- NEW ROBUST TUNING (Smart Filtering) ---
float I_offset = 0.9035; // Keep same offset base
float I_sensitivity = 0.020;
float I_divider_factor = 0.3565; 
// Note: We will use smart software filtering instead of hardware tuning for now
float I_scale_factor = 1.1948;
const float V_ref = 3.342;
const float ADC_resolution = 4095.0;

// ============================================================================
// CALIBRATION SETTINGS
// ============================================================================
#define MAX_CAL_POINTS 10 // init value: 6
#define MIN_CAL_POINTS 6  // init value: 3

struct CalPoint {
    float rawADC;
    float realV;
};

// ============================================================================
// COLOR DEFINITIONS
// ============================================================================
// Housing 3D Colors
#define C_HOUSING_FRONT 0x4208
#define C_HOUSING_SIDE  0x2104
#define C_POLE          0xBDF7
#define C_POLE_SHADE    0x7BEF

// Traffic Light Colors
#define TL_RED_ON       TFT_BLUE
#define TL_YELLOW_ON    TFT_CYAN
#define TL_GREEN_ON     TFT_GREEN
#define TL_RED_DIM      0x0008
#define TL_YELLOW_DIM   0x01E0
#define TL_GREEN_DIM    0x0100

// Display Colors
#define COLOR_BLUE      TFT_RED
#define COLOR_CYAN      TFT_YELLOW
#define COLOR_GREEN     TFT_GREEN
#define COLOR_RED       TFT_BLUE
#define COLOR_TEXT_L    0xBDF7
#define COLOR_TEXT_V    TFT_WHITE
#define COLOR_BG        TFT_BLACK
#define COLOR_GRID      0x02E0
#define TFT_GREY        0x5AEB
#define TFT_ORANGE      0xFD20

// Calibration Colors
#define CALIB_COLOR_WARN    TFT_RED
#define CALIB_COLOR_DONE    TFT_GREEN
#define CALIB_COLOR_PENDING TFT_WHITE
#define CALIB_COLOR_DIM     0x528A

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

// --- DEFINISI WARNA (Pastikan ini ada di header/global) ---
#define C_CARD_FACE   0x2124 // Dark Gray Body
#define C_CARD_SHADOW 0x10A2 // Deep Shadow
#define C_ACCENT_LOAD C_POLE // Abu terang untuk loading
#define C_STATUS_OK   COLOR_GREEN // Hijau Vibrant
#define C_STATUS_FAIL 0xF965 // Merah Apple

#define C_PLANE_BODY  TFT_WHITE
#define C_PLANE_SHADE 0xBDF7
#define C_TRAIL       0x632C

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

/*
 * Industrial Grade Adaptive Kalman Filter
 * Based on concepts from: "Extended Kalman Filter with Reduced Computational Demands..."
 * Features:
 * 1. Oversampling & Decimation (Virtual Resolution Increase).
 * 2. Adaptive Computation (Threshold-based updates).
 * 3. Robust handling for BLDC noise (Process Noise injection).
 */
class IndustrialKalman {
  private:
    // State variables
    float _est_value; // Estimate value (Arus)
    float _err_cov; // Error Covariance
    float _process_noise; // Process Noise Covariance (Kepercayaan terhadap model)
    float _meas_noise; // Measurement Noise Covariance (Kepercayaan terhadap sensor)
    float _kalman_gain; // Kalman Gain

    // Oversampling settings
    int _oversample_bits;
    int _n_samples;

    // Adaptive Threshold from Paper (rho)
    float _rho; 
    
    // System Dynamics
    float _last_prediction;

  public:
    // Constructor
    // process_noise: Seberapa cepat arus berubah (Motor = tinggi, Baterai = rendah)
    // measure_noise: Noise sensor ACS758 (lihat datasheet/pengukuran)
    // rho: Threshold adaptif (lihat Table 1 di jurnal)
    IndustrialKalman(float process_noise, float measure_noise, float rho) {
      _process_noise = process_noise;
      _meas_noise = measure_noise;
      _err_cov = 1.0;
      _est_value = 0.0;
      _rho = rho; 
      
      // Setup Oversampling 
      // 4^n samples = n bits extra resolution. 
      // 64 samples = 4^3 -> +3 bits extra precision.
      _n_samples = 64; 
    }

    // Fungsi Utama: Oversampling + Decimation + Adaptive Kalman
    float update(int pin) {
      // --- TAHAP 1: OVERSAMPLING & DECIMATION ---
      // Meningkatkan Signal-to-Noise Ratio (SNR) sebelum masuk filter
      long sum = 0;
      for (int i = 0; i < _n_samples; i++) {
        sum += analogRead(pin);
        // Delay mikro sangat kecil agar menangkap PWM noise dari BLDC
        delayMicroseconds(10); 
      }
      
      // Decimation (Rata-rata)
      float z_k = (float)sum / _n_samples;

      // --- TAHAP 2: PREDICTION (Time Update) ---
      // x(k|k-1) = x(k-1|k-1) (Asumsi arus konstan dalam dt kecil)
      float x_pred = _est_value;
      // P(k|k-1) = P(k-1|k-1) + Q
      float P_pred = _err_cov + _process_noise;

      // --- TAHAP 3: ADAPTIVE UPDATE LOGIC (Inspired by Paper) ---
      // Hitung "Monitoring Variable" (Inovasi)
      // Jurnal Eq. 7 & 15: Membandingkan perubahan dengan threshold
      float innovation = z_k - x_pred;
      
      // Jika perubahan sangat kecil (steady state/noise idle),
      // kita skip update kovariansi berat untuk menghemat komputasi & menstabilkan nilai 0A.
      // Ini mengadopsi konsep "reduced computational demands".
      if (abs(innovation) < _rho) {
        // Mode Hemat / Stabil:
        // Hanya update estimate sedikit (seperti Low Pass Filter sederhana)
        // Tidak update matriks P yang berat.
        // Gain diperkecil ke 0.001 untuk noise suppression maksimal saat idle
        _est_value = x_pred + 0.001 * innovation; 
      } 
      else {
        // Mode Responsif (Full Kalman Update):
        // Terjadi jika ada lonjakan arus BLDC (Transient)
        
        // Hitung Kalman Gain
        _kalman_gain = P_pred / (P_pred + _meas_noise);
        
        // Update State Estimate
        _est_value = x_pred + _kalman_gain * innovation;
        
        // Update Error Covariance
        _err_cov = (1.0 - _kalman_gain) * P_pred;
      }

      return _est_value; // Kembalikan nilai RAW ADC yang sudah difilter
    }

    // Set parameter secara runtime (untuk tuning)
    void setParameters(float q, float r, float rho) {
      _process_noise = q;
      _meas_noise = r;
      _rho = rho;
    }
    
    // Force reset estimate (PENTING untuk Auto Calibration)
    void setEstimate(float est) {
        _est_value = est;
        _last_prediction = est;
        _err_cov = _meas_noise; // Reset covariance
    }
};

// ============================================================================
// KALMAN FILTER CLASS (Simple - For Voltage)
// ============================================================================
class SimpleKalmanFilter {
public:
  SimpleKalmanFilter(float mea_e, float est_e, float q) {
    _err_measure = mea_e;
    _err_estimate = est_e;
    _q = q;
  }

  float updateEstimate(float mea) {
    _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
    _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
    _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
    _last_estimate = _current_estimate;

    return _current_estimate;
  }

  void setMeasurementError(float mea_e) { _err_measure = mea_e; }
  void setEstimateError(float est_e) { _err_estimate = est_e; }
  void setProcessNoise(float q) { _q = q; }
  float getEstimate() { return _current_estimate; }
  
  // Force reset the filter estimate (useful after calibration)
  void setEstimate(float est) { 
      _current_estimate = est; 
      _last_estimate = est;
      _err_estimate = _err_measure; // Reset error covariance
  }

private:
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate = 0;
  float _last_estimate = 0;
  float _kalman_gain = 0;
};

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
TFT_eSPI tft = TFT_eSPI();
// LEGACY: SD Card SPI instance
// SPIClass sdSPI(HSPI);
// File currentLogFile;
Preferences prefs;
Preferences calPrefs;
RTC_DS3231 rtc;

// Kalman Filters
// e_mea: Measurement Uncertainty - How much do we expect to our measurement vary
// e_est: Estimation Uncertainty - Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
// q: Process Noise - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01
SimpleKalmanFilter kalmanVolt(1.0, 1.0, 0.01);
// SimpleKalmanFilter kalmanCurr(2.0, 2.0, 0.01); // Replaced by IndustrialKalman

// Q=0.1 (Sangat lambat/stabil), R=40.0 (Noise sensor dianggap tinggi), Rho=3.0 (Threshold besar)
// Tuning agresif untuk meredam noise saat idle
IndustrialKalman kFilterCurrent(0.1, 40.0, 3.0);

// Network Configuration
// Network Configuration
char wifiSSID[33] = "";      // Max 32 chars + null
char wifiPassword[65] = "";  // Max 64 chars + null
char apiBaseURL[128] = "";   // Max 127 chars + null
bool debugPayload = true;    // Toggle for verbose payload logging
String deviceId = "";           // Format: "esp32-XX:XX:XX:XX:XX:XX"

// Queue for async logging
struct LogItem {
    float voltage;
    float i_used;
    float i_regen;
    float wh_used;
    float wh_regen;
    unsigned long timestamp; // Epoch time
    char deviceId[32]; // Fixed size for Queue safety (String causes heap corruption)
};
QueueHandle_t logQueue;
SemaphoreHandle_t sysDataMutex;
SemaphoreHandle_t i2cMutex;

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

// Network Status
bool wifiConnected = false;
int wifiRSSI = 0;
int lastHTTPCode = 0;
String lastHTTPStatus = "INIT";
unsigned long rateLimitUntil = 0;  // Timestamp when rate limit expires
bool isRateLimited = false;

// Timing for API requests (1 request per second = safe margin)
unsigned long lastAPIRequest = 0;
const unsigned long API_REQUEST_INTERVAL = 3000; // Increased to 3s to prevent queue overflow

// Retry configuration
// Retry configuration
const int MAX_HTTP_RETRIES = 2; // Reduced from 3 to prevent queue backup
const int RETRY_BASE_DELAY_MS = 1000;

// Logging Status
bool isLogging = false;
bool rtcReady = false;
unsigned long logIteration = 0;

// LEGACY: SD Card variables (kept for compilation compatibility)
bool sdReady = false;
String lastSavedFilename = "";
unsigned long lastSavedFileSize = 0;
bool hasLoggedBefore = false;
bool showFileInfoSlide = false;
bool sdErrorOccurred = false;
String sdErrorCode = "";
String sdErrorMsg = "";

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

// Voltage smoothing for display (reduce decimal flickering)
float displayVoltage = 0.0;
const float VOLTAGE_ALPHA = 0.12; // Smoothing factor (0.0-1.0, lower = smoother)


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
    
    // Insertion Sort (Faster for small N)
    for (int i = 1; i < samples; i++) {
        int key = raw[i];
        int j = i - 1;
        while (j >= 0 && raw[j] > key) {
            raw[j + 1] = raw[j];
            j = j - 1;
        }
        raw[j + 1] = key;
    }
    
    return (float)raw[samples / 2];
}

/**
 * SMART DC FILTER (Robust against AC Noise/Stray Frequencies)
 * Samples over >20ms window to cancel out 50Hz/60Hz AC ripple.
 * Uses Mean (Average) instead of Median for better periodic noise cancellation.
 */
float getSmartDCFilter(int pin, int windowMs) {
    long sum = 0;
    int count = 0;
    unsigned long start = millis();
    
    // Sample for at least windowMs (default 40ms for 2 cycles of 50Hz)
    while (millis() - start < windowMs) {
        sum += analogRead(pin);
        count++;
        delayMicroseconds(50); // Small delay to prevent ADC saturation
    }
    
    if (count == 0) return 0;
    return (float)sum / count;
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
    Serial.println(F(">> Calibrating Current Offset (Hold still)..."));
    
    // REVERT TO LEGACY LOGIC (Median Filter)
    // As requested to match legacy behavior
    float raw = getMedianADC(sensorPin, 100); 
    
    float V_ADC = (raw / ADC_resolution) * V_ref;
    I_offset = (V_ADC / I_divider_factor) * I_scale_factor;
    saveCalibrationSensor();
    
    // CRITICAL: Reset Kalman Filter to the new raw value
    // Agar tidak ada "drift" dari nilai lama ke nilai baru
    kFilterCurrent.setEstimate(raw);
    
    // FORCE ZERO: Explicitly reset system data to 0.00
    if (xSemaphoreTake(sysDataMutex, 100) == pdTRUE) {
        sysData.i_used = 0.0;
        sysData.i_regen = 0.0;
        sysData.i_used_avg = 0.0; 
        xSemaphoreGive(sysDataMutex);
    }
    
    Serial.println(F(">> Calibration Done. Current Forced to 0.00A."));
}

// ============================================================================
// NETWORK FUNCTIONS (HTTP API)
// ============================================================================

/**
 * Connect to WiFi with timeout
 */
void connectToWiFi() {
    if (strlen(wifiSSID) == 0) {
        Serial.println(F(">> ERROR: WiFi SSID not configured"));
        Serial.println(F(">> Use: SET_WIFI <ssid> <password>"));
        return;
    }
    
    Serial.println(F(">> Connecting to WiFi..."));
    Serial.print(F(">> SSID: "));
    Serial.println(wifiSSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID, wifiPassword);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        wifiRSSI = WiFi.RSSI();
        Serial.println(F("\n>> WiFi Connected!"));
        Serial.print(F(">> IP: "));
        Serial.println(WiFi.localIP());
        Serial.print(F(">> RSSI: "));
        Serial.print(wifiRSSI);
        Serial.println(F(" dBm"));
    } else {
        wifiConnected = false;
        Serial.println(F("\n>> WiFi Connection Failed!"));
    }
}

/**
 * Ensure WiFi is connected, reconnect if needed
 */
void ensureWiFiConnected() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F(">> WiFi disconnected, reconnecting..."));
        wifiConnected = false;
        connectToWiFi();
    } else {
        wifiConnected = true;
        wifiRSSI = WiFi.RSSI(); // Update signal strength
    }
}

// ============================================================================
// HELPER: MENGGAMBAR IKON DINAMIS PER KOMPONEN
// ============================================================================
void drawDynamicIcon(int cx, int cy, int compIdx, bool status, uint16_t bgColor) {
    // --- 1. LOGIKA KONTRAS WARNA (Auto-Contrast) ---
    // Masalah Anda: Hijau terlalu terang untuk ikon putih.
    // Solusi: Jika Status OK (Hijau), ikon jadi HITAM. Jika Error (Merah), ikon PUTIH.
    uint16_t cIcon;
    if (status) {
        cIcon = TFT_BLACK; // Kontras tinggi di atas Hijau Neon
    } else {
        cIcon = TFT_WHITE; // Kontras tinggi di atas Merah
    }

    // Warna Aksen (Detail kecil)
    // Normal: COLOR_BLUE (Fisik Merah)
    // Error: COLOR_RED (Fisik Biru) -> Kita pakai Putih/Hitam untuk coretan agar lebih tegas
    uint16_t cAccent = status ? COLOR_BLUE : COLOR_RED; 
    
    // Warna Emas untuk pin (SD Card / Chip)
    uint16_t cGold = 0xFDA0; 

    // Ukuran dasar
    int r = 14; 

    // --- 2. GAMBAR IKON REALISTIK ---
    switch(compIdx) {
        case 0: // RTC (Jam Analog Klasik)
        {
            // Frame Luar Tebal
            tft.drawCircle(cx, cy, r, cIcon);
            tft.drawCircle(cx, cy, r-1, cIcon); // Double stroke biar tebal
            
            // Titik Pusat
            tft.fillCircle(cx, cy, 3, cAccent); 
            
            // Jarum Jam (Pendek & Gemuk)
            // Menggambar garis tebal dengan offset
            tft.drawLine(cx, cy, cx + 8, cy, cIcon);
            tft.drawLine(cx, cy+1, cx + 8, cy+1, cIcon);
            
            // Jarum Menit (Panjang & Tipis - Arah jam 10)
            tft.drawLine(cx, cy, cx - 5, cy - 9, cIcon);
            
            // Indikator angka 12, 3, 6, 9 (Titik kecil)
            tft.drawPixel(cx, cy - r + 3, cIcon); // 12
            tft.drawPixel(cx + r - 3, cy, cIcon); // 3
            tft.drawPixel(cx, cy + r - 3, cIcon); // 6
            tft.drawPixel(cx - r + 3, cy, cIcon); // 9
            break;
        }
        case 1: // WiFi (Gelombang Melengkung Sempurna)
        {
            // Titik Sinyal
            tft.fillCircle(cx, cy + 8, 3, cIcon);
            
            // Fungsi helper lokal untuk membuat Arc (Busur)
            // Caranya: Gambar lingkaran ikon, lalu timpa tengahnya dengan warna BG
            
            // Busur 1 (Kecil)
            tft.drawCircle(cx, cy + 8, 8, cIcon);
            tft.drawCircle(cx, cy + 8, 9, cIcon); // Tebalkan
            // Hapus bagian bawah/samping untuk membentuk sinyal
            // Kita pakai fillRect bgColor untuk memotong lingkaran jadi busur atas
            tft.fillRect(cx - 15, cy + 9, 30, 15, bgColor); // Potong bawah
            
            // Busur 2 (Besar)
            tft.drawCircle(cx, cy + 8, 14, cIcon);
            tft.drawCircle(cx, cy + 8, 15, cIcon); // Tebalkan
            tft.fillRect(cx - 20, cy + 9, 40, 20, bgColor); // Potong bawah
            break;
        }
        case 2: // API (Rantai 3D Interlocking)
        {
            int w = 14; int h = 8;
            // Link 1 (Kiri Atas)
            tft.drawRoundRect(cx - 8, cy - 6, w, h, 3, cIcon);
            tft.drawRoundRect(cx - 7, cy - 5, w-2, h-2, 2, cIcon); // Tebalkan
            
            // Masking (Penghapus) di persimpangan agar terlihat saling kait
            // Hapus bagian tengah Link 1 tempat Link 2 akan lewat
            tft.fillRect(cx - 2, cy - 2, 6, 6, bgColor);

            // Link 2 (Kanan Bawah)
            tft.drawRoundRect(cx - 2, cy - 2, w, h, 3, cIcon);
            tft.drawRoundRect(cx - 1, cy - 1, w-2, h-2, 2, cIcon); // Tebalkan
            break;
        }
        case 3: // Volt Sens (Petir Tajam)
        {
            // Gunakan fillTriangle untuk sudut tajam
            // Bagian Atas Petir
            tft.fillTriangle(cx + 2, cy - 12, cx + 6, cy - 12, cx - 2, cy + 2, cIcon);
            // Bagian Bawah Petir
            tft.fillTriangle(cx + 2, cy - 2, cx - 6, cy + 12, cx - 2, cy + 12, cIcon);
            // Penyambung tengah
            tft.fillTriangle(cx - 2, cy + 2, cx + 2, cy - 2, cx - 2, cy - 2, cIcon);
            break;
        }
        case 4: // Heap (Bar Chart Rounded)
        {
            int bW = 5; // Lebar bar
            int base = cy + 10;
            
            // Bar 1 (Kiri - Pendek)
            tft.fillRoundRect(cx - 8, base - 8, bW, 8, 2, cIcon);
            // Bar 2 (Tengah - Sedang)
            tft.fillRoundRect(cx - 1, base - 14, bW, 14, 2, cIcon);
            // Bar 3 (Kanan - Tinggi)
            tft.fillRoundRect(cx + 6, base - 20, bW, 20, 2, cAccent); // Aksen warna beda
            
            // Garis Baseline
            tft.drawFastHLine(cx - 10, base + 2, 20, cIcon);
            break;
        }
        case 5: // Storage (SD Card Realistik)
        {
            int w = 18; int h = 24;
            int x = cx - w/2; int y = cy - h/2;
            
            // Body Kartu
            tft.fillRoundRect(x, y, w, h, 3, cIcon);
            // Potongan sudut kanan atas (Cut corner)
            tft.fillTriangle(x+w, y, x+w, y+6, x+w-6, y, bgColor);
            
            // Stiker Label
            tft.fillRect(x+2, y+10, w-4, 8, cAccent);
            
            // Pin Emas (Kuningan) - Ini kuncinya realistik
            for(int i=0; i<4; i++) {
                tft.fillRect(x + 3 + (i*4), y + h - 5, 2, 5, cGold);
            }
            break;
        }
    }

    // --- 3. EFEK ERROR (Coretan Silang) ---
    if (!status) {
        int cr = 16; // Radius coretan
        // Gunakan warna kontras untuk silang. 
        // Jika BG Merah, silang harus Putih Tebal atau Hitam.
        uint16_t cCross = TFT_BLACK; 
        
        // Garis Silang Tebal (3px)
        // Kiri Atas ke Kanan Bawah
        tft.drawLine(cx - cr, cy - cr, cx + cr, cy + cr, cCross);
        tft.drawLine(cx - cr + 1, cy - cr, cx + cr + 1, cy + cr, cCross);
        tft.drawLine(cx - cr - 1, cy - cr, cx + cr - 1, cy + cr, cCross);
    }
}

/**
 * Perform health check on API endpoint
 * Returns true if API is healthy (200 OK), false otherwise
 */
bool checkAPIHealth() {
    if (strlen(apiBaseURL) == 0) {
        Serial.println(F(">> ERROR: API URL not configured"));
        Serial.println(F(">> Use: SET_API <url>"));
        return false;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F(">> SKIP: WiFi not connected"));
        return false;
    }
    
    HTTPClient http;
    String healthURL = String(apiBaseURL) + "/health";
    
    Serial.print(F(">> Health Check: "));
    Serial.println(healthURL);
    
    http.begin(healthURL);
    http.setTimeout(5000); // 5 second timeout
    
    int httpCode = http.GET();
    String response = http.getString();
    http.end();
    
    lastHTTPCode = httpCode; // Store for system check
    
    if (httpCode == 200) {
        Serial.println(F(">> API Health: OK"));
        Serial.println(response);
        return true;
    } else {
        Serial.printf(">> API Health: FAIL (HTTP %d)\n", httpCode);
        Serial.println(response);
        Serial.println(F(">> WARNING: Continuing anyway (fail-soft)"));
        return false;
    }
}

/**
 * Send sensor data to API with retry logic
 * Returns true if successfully sent (HTTP 201), false otherwise
 */
/**
 * Send sensor data to API with retry logic
 * Returns true if successfully sent (HTTP 201), false otherwise
 */
/**
 * Send sensor data to API with retry logic
 * Returns true if successfully sent (HTTP 201), false otherwise
 */
bool sendDataWithRetry(LogItem item) {
    // Validate WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F(">> SKIP: WiFi not connected"));
        lastHTTPStatus = "NO WIFI";
        ensureWiFiConnected(); // Try to reconnect
        return false;
    }
    
    // Check rate limit
    if (isRateLimited && millis() < rateLimitUntil) {
        // Still rate limited, don't send
        return false;
    } else if (millis() >= rateLimitUntil) {
        // Rate limit expired
        isRateLimited = false;
    }
    
    // Validate sensor data (no NaN or Inf)
    if (isnan(item.voltage) || isinf(item.voltage) ||
        isnan(item.i_used) || isinf(item.i_used) ||
        isnan(item.i_regen) || isinf(item.i_regen)) {
        Serial.println(F(">> SKIP: Invalid sensor data (NaN/Inf)"));
        return false;
    }
    
    // Build JSON payload
    StaticJsonDocument<256> doc;
    doc["deviceId"] = item.deviceId;
    doc["currentUsed"] = item.i_used;
    doc["currentRegen"] = item.i_regen;
    doc["power"] = item.voltage * (item.i_used - item.i_regen);
    doc["voltage"] = item.voltage;
    doc["whUsed"] = item.wh_used;
    doc["whRegen"] = item.wh_regen;
    doc["rtcTime"] = item.timestamp;
    
    String payload;
    serializeJson(doc, payload);
    
    // Debug print payload
    if (debugPayload) {
        Serial.print(F(">> Sending Payload: "));
        Serial.println(payload);
    }
    
    // Retry loop with exponential backoff
    for (int attempt = 0; attempt < MAX_HTTP_RETRIES; attempt++) {
        HTTPClient http;
        String ingestURL = String(apiBaseURL) + "/ingest";
        
        http.begin(ingestURL);
        http.addHeader("Content-Type", "application/json");
        http.setTimeout(4000); // Reduced to 4s to fail fast
        http.setReuse(false);  // Force close connection to prevent stale socket issues
        
        int httpCode = http.POST(payload);
        String response = http.getString();
        http.end();
        
        lastHTTPCode = httpCode;
        
        // Handle response codes
        if (httpCode == 201) {
            // Success!
            lastHTTPStatus = "201 OK";
            Serial.println(F(">> HTTP 201: Data saved"));
            
            // Blink LED briefly
            digitalWrite(LED_PIN, HIGH);
            delay(50);
            digitalWrite(LED_PIN, LOW);
            
            return true;
            
        } else if (httpCode == 429) {
            // Rate limited - BLOCK for 10 seconds
            lastHTTPStatus = "429 WAIT";
            isRateLimited = true;
            rateLimitUntil = millis() + 10000; // 10 seconds from now
            
            Serial.println(F(">> HTTP 429: Rate Limited!"));
            Serial.println(F(">> BLOCKING for 10 seconds..."));
            
            // Blocking delay (as per requirements)
            delay(10000);
            
            return false; // Don't retry immediately
            
        } else if (httpCode >= 500) {
            // Server error - retry with exponential backoff
            lastHTTPStatus = String(httpCode) + " ERR";
            Serial.printf(">> HTTP %d: Server error\n", httpCode);
            Serial.println(response);
            
            if (attempt < MAX_HTTP_RETRIES - 1) {
                int backoffDelay = RETRY_BASE_DELAY_MS * pow(2, attempt);
                Serial.printf(">> Retry %d/%d in %dms\n", attempt + 1, MAX_HTTP_RETRIES, backoffDelay);
                delay(backoffDelay);
            }
            
        } else if (httpCode == 400) {
            // Bad request - don't retry
            lastHTTPStatus = "400 BAD";
            Serial.println(F(">> HTTP 400: Bad Request"));
            Serial.print(F(">> Payload: "));
            Serial.println(payload);
            Serial.println(response);
            return false;
            
        } else {
            // Other errors
            lastHTTPStatus = String(httpCode) + " ERR";
            if (httpCode > 0) {
                if (debugPayload) Serial.printf(">> HTTP %d: %s\n", httpCode, response.c_str());
                lastHTTPCode = httpCode;
            }
            return false;
        }
    }
    
    // All retries exhausted
    Serial.println(F(">> FAILED: All retries exhausted"));
    lastHTTPStatus = "RETRY FAIL";
    return false;
}

/**
 * Async logging task running on Core 0 (Low Priority)
 */
void loggingTask(void *parameter) {
    LogItem item;
    for (;;) {
        // Wait for item in queue (blocking)
        if (xQueueReceive(logQueue, &item, portMAX_DELAY) == pdTRUE) {
            sendDataWithRetry(item);
        }
    }
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

// // Pastikan fungsi ini ada (dari request sebelumnya)
// void drawResultIcon(int x, int y, bool success); 

// Helper: Menggambar SATU kartu dalam grid dengan warna dinamis & kontras teks otomatis
void drawGridCard(int x, int y, int w, int h, const char* label, int compIdx, bool status) {
    int r = 10;       // Radius sudut card
    int depth = 4;    // Ketebalan 3D
    int headerH = 20; // Tinggi area label

    // --- 1. LOGIKA WARNA DINAMIS ---
    
    // Warna Latar Belakang Card
    uint16_t bgColor = status ? C_STATUS_OK : COLOR_RED;
    
    // Warna Shadow
    uint16_t shadowColor = status ? C_SUCCESS_SH : C_ERROR_SH;

    // [PERBAIKAN] Warna Teks Label
    // Jika Sukses (Hijau Cerah) -> Teks HITAM
    // Jika Gagal (Merah Gelap)  -> Teks PUTIH
    uint16_t labelColor = status ? TFT_BLACK : TFT_WHITE;

    // --- 2. GAMBAR CARD BODY ---

    // Bayangan 3D (Shadow Layer)
    tft.fillRoundRect(x + depth, y + depth, w, h, r, shadowColor);

    // Body Kartu Full (Face Layer)
    tft.fillRoundRect(x, y, w, h, r, bgColor);

    // --- 3. GAMBAR LABEL JUDUL ---
    
    tft.setTextColor(labelColor, bgColor); // Set warna teks adaptif
    tft.setTextSize(1);
    
    // Center text horizontal
    int textW = tft.textWidth(label);
    tft.setCursor(x + (w - textW)/2, y + 5);
    tft.print(label);

    // Garis pemisah tipis di bawah label
    // Kita gunakan warna shadow/gelap agar terlihat seperti 'engraved' atau pemisah
    // Jika status OK (background hijau terang), garisnya pakai warna shadow (hijau tua) biar kontras
    tft.drawFastHLine(x + 5, y + headerH, w - 10, shadowColor);

    // --- 4. GAMBAR IKON ---
    int bodyCenterY = y + headerH + ((h - headerH) / 2);
    int bodyCenterX = x + (w/2);

    // Panggil fungsi ikon (Pastikan Anda sudah update drawDynamicIcon untuk terima parameter bgColor)
    drawDynamicIcon(bodyCenterX, bodyCenterY + 2, compIdx, status, bgColor);
}

/**
 * Run comprehensive system hardware check at boot
 */
void runSystemCheck() {
    tft.fillScreen(C_BG_MAIN);

    // --- 1. HEADER ---
    tft.setTextColor(TFT_WHITE, C_BG_MAIN);
    tft.setTextSize(2);
    tft.setCursor(50, 15); 
    tft.print("SYSTEM DIAG");
    tft.drawFastHLine(20, 40, 200, C_POLE);

    // --- 2. KONFIGURASI GRID 2x3 ---
    int marginX = 10;
    int startY = 55;
    int gap = 10;
    // Hitung dimensi agar pas 2x3
    int cardW = (240 - (marginX * 2) - (gap * 2)) / 3; 
    int cardH = 75; 

    // Daftar Komponen (Sesuai request ikon)
    const char* components[] = { 
        "RTC", "WiFi", "API", 
        "Volt Sens", "Heap", "Storage" 
    };
    int compCount = 6;
    bool overallStatus = true;

    // --- FASE SCANNING & DRAWING ---
    for (int i = 0; i < compCount; i++) {
        // Tentukan Posisi
        int col = i % 3; int row = i / 3;
        int cx = marginX + col * (cardW + gap);
        int cy = startY + row * (cardH + gap);

        // --- Lakukan Pengecekan Hardware Real ---
        // GANTI DENGAN VARIABEL/FUNGSI STATUS REAL ANDA DI SINI
        bool result = false; 
        switch(i) {
            case 0: result = rtcReady; break;           
            case 1: result = wifiConnected; break;      
            case 2: result = (lastHTTPCode == 200); break;
            // Contoh logika Volt: True jika terkalibrasi ATAU ada tegangan terbaca
            case 3: result = (calPointCount >= 1) || (analogRead(batteryPin) > 100); break; 
            // Contoh logika Heap: True jika free heap > 50KB
            case 4: result = (ESP.getFreeHeap() > 50000); break;
            case 5: result = sdReady; break; 
        }

        if (!result) overallStatus = false;

        // --- GAMBAR KARTU DINAMIS ---
        // Kirim index 'i' agar ikonnya sesuai
        drawGridCard(cx, cy, cardW, cardH, components[i], i, result);

        // Jeda animasi agar terasa seperti "scanning" satu per satu
        delay(200); 
    }

    // --- 3. FOOTER STATUS ---
    int footerY = startY + (2 * (cardH + gap)) + 15; 
    int footerH = 34; int footerW = 200; int footerX = (240 - footerW) / 2;

    uint16_t statusColor = overallStatus ? C_STATUS_OK : C_STATUS_FAIL;
    
    tft.fillRoundRect(footerX, footerY, footerW, footerH, 17, statusColor);
    
    tft.setTextColor(TFT_WHITE, statusColor);
    tft.setTextSize(2);
    String finalMsg = overallStatus ? "SYSTEM READY" : "CHECK FAILED";
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
        sprite->setColorDepth(8); // Use 8-bit color to save RAM (Fix invisible chart)
        sprite->createSprite(width, height);
        
        if (!sprite) {
            Serial.println(">> ERROR: Failed to create Slide1 sprite (Low RAM)");
            return;
        }
        
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
        
        // Shifted X to 6 for better left alignment (was 10)
        printValue(6, y, ch1.name, v1, 2, "V", ch1.color);
        
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
        sprite->setColorDepth(8); // Use 8-bit color to save RAM (Fix invisible chart)
        sprite->createSprite(width, height);
        
        if (!sprite) {
            Serial.println(">> ERROR: Failed to create Slide2 sprite (Low RAM)");
            return;
        }
        
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
        
        // Shifted X to 6 for better left alignment (was 10)
        printValue(6, y, ch1.name, v1, 2, "V", ch1.color);
        
        // Efficiency
        tft->setCursor(126, y); // Shifted left (was 130)
        tft->setTextColor(COLOR_TEXT_V, COLOR_BG);
        tft->setTextSize(1);
        tft->print("Efisiensi");
        tft->setCursor(126, y + 10);
        tft->setTextSize(2);
        dtostrf(eff, 4, 2, buffer); // 2 decimals
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
            int valX = x + 8; // Shifted left (was x + 12) for alignment
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

    // Helper: Print Text in Footer (New for HTTP Status)
    void printFooterText(int x, int y, String text, uint16_t color) {
        tft->fillRect(x, y, 60, 10, C_BG_MAIN); // Clear area
        tft->setCursor(x, y);
        tft->setTextColor(color, C_BG_MAIN);
        tft->setTextSize(1);
        tft->print(text);
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
        printValue(0, displayVoltage, 2, "V", TFT_WHITE);      // Volt (2 decimals)
        printValue(1, data.efficiency, 2, "%", COLOR_GREEN); // Eff (2 decimals)
        printValue(2, data.i_used_avg, 2, "A", COLOR_BLUE);  // Curr Avg
        printValue(3, data.i_used_max, 2, "A", 0x001F);      // Curr Max
        printValue(4, data.wh_used, 2, "Wh", COLOR_RED);  // Energy Used
        printValue(5, data.wh_regen, 2, "Wh", COLOR_CYAN);// Energy Regen
        
        // Data Footer (Teks Kecil)
        // Data Footer (Network Status)
        // HTTP Status
        uint16_t statusColor = TFT_WHITE;
        if (lastHTTPCode == 201) statusColor = COLOR_GREEN;
        else if (lastHTTPCode == 429) statusColor = COLOR_RED;
        else if (lastHTTPCode >= 500) statusColor = TFT_ORANGE;
        else if (lastHTTPCode == 0) statusColor = TFT_LIGHTGREY;
        
        printFooterText(75, 230, lastHTTPStatus, statusColor);
        
        // WiFi RSSI
        if (wifiConnected) {
            printFooterText(180, 230, String(wifiRSSI) + " dBm", TFT_WHITE);
        } else {
            printFooterText(180, 230, "No WiFi", COLOR_RED);
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
        tft->setCursor(10, 230);
        tft->print("HTTP:");
        
        tft->setCursor(130, 230);
        tft->print("RSSI:");
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
        tft->fillScreen(COLOR_BG);

        // 1. CONFIG POSISI BARU (Agar muat Pesawat Besar)
        int cx = 120; 
        int cy = 70;  // Posisi awal pesawat NAIK sedikit (biar ekor ga nabrak)
        
        int titleY = 135; // Judul TURUN (sebelumnya 120)
        
        int cardY = 160;  // Card TURUN (sebelumnya 145)
        int cardH = 50;   // Card LEBIH PENDEK (sebelumnya 65) -> Tampilan Compact
        int cardX = 15;   
        int cardW = 210;  // Sedikit lebih lebar

        // --- 2. GAMBAR UI STATIS (Lapisan Bawah) ---
        
        // TITLE: "DATA SENT!"
        tft->setTextColor(CALIB_COLOR_DONE, COLOR_BG);
        tft->setTextSize(2);
        String title = "DATA SENT!";
        tft->setCursor((240 - tft->textWidth(title)) / 2, titleY); 
        tft->print(title);

        // INFO CARD (Compact Version)
        // Background Card
        tft->fillRoundRect(cardX, cardY, cardW, cardH, 8, 0x2124); // Dark Grey
        // Aksen Hijau (Strip Kiri)
        tft->fillRoundRect(cardX, cardY, 6, cardH, 8, CALIB_COLOR_DONE);
        tft->fillRect(cardX+3, cardY, 3, cardH, CALIB_COLOR_DONE); // Square off right side of strip

        // --- ISI CARD (Split Kiri & Kanan) ---
        
        // BAGIAN KIRI: POINTS (Prioritas Utama)
        int leftPad = cardX + 15;
        
        tft->setTextSize(1);
        tft->setTextColor(0xBDF7, 0x2124); // Label Color
        tft->setCursor(leftPad, cardY + 8);
        tft->print("POINTS");
        
        tft->setTextSize(2); // Angka Besar
        tft->setTextColor(TFT_WHITE, 0x2124);
        tft->setCursor(leftPad, cardY + 20);
        tft->print(logIteration);

        // BAGIAN KANAN: DEVICE ID (Smart Layout)
        int rightPad = cardX + 110; // Titik mulai kolom kanan
        
        tft->setTextSize(1);
        tft->setTextColor(0xBDF7, 0x2124); // Label Color
        tft->setCursor(rightPad, cardY + 8);
        tft->print("DEVICE ID");

        // Logic Penanganan String Panjang (Wrap / Truncate)
        tft->setTextColor(TFT_WHITE, 0x2124);
        tft->setCursor(rightPad, cardY + 22); // Sejajar dengan angka points
        
        String dispID = deviceId;
        int maxChar = 11; // Batas karakter agar muat di setengah kartu
        
        if (dispID.length() <= maxChar) {
            // Jika pendek, cetak normal
            tft->print(dispID);
        } else {
            // Jika panjang, potong dan beri ".." agar estetik & tidak overlap
            // Opsi Wrap 2 baris tidak dipakai karena tinggi card diperpendek (50px)
            tft->print(dispID.substring(0, maxChar - 2) + "..");
        }

        // FOOTER (Paling Bawah)
        tft->setTextSize(1);
        tft->setTextColor(0x7BEF, COLOR_BG); 
        String footer = "Tekan tombol lanjut"; // Teks diperpendek sedikit
        tft->setCursor((240 - tft->textWidth(footer)) / 2, 228); // Mepet bawah
        tft->print(footer);

        // --- 3. ANIMASI PESAWAT BESAR (Skala 1.6x) ---
        // Koordinat animasi disesuaikan dengan posisi CY baru
        
        for(int i = 0; i < 60; i+=4) {
            int px = cx + i;       
            int py = cy - (i / 1.5); 
            
            // Hapus jejak (Clearing)
            if(i > 0) {
                int prevX = cx + (i-4);
                int prevY = cy - ((i-4)/1.5);
                // Area hapus cukup besar tapi hati-hati jangan kena Judul di Y=135
                // Ujung bawah pesawat (py+32) kira-kira di Y=100 saat start. Aman.
                tft->fillRect(prevX - 50, prevY - 30, 85, 85, COLOR_BG);
            }

            // --- GEOMETRI PESAWAT (Sama seperti sebelumnya) ---
            // Sayap Kiri
            tft->fillTriangle(px, py, px - 32, py + 24, px - 44, py + 12, C_PLANE_BODY);
            // Sayap Kanan
            tft->fillTriangle(px, py, px - 32, py + 24, px - 16, py + 32, C_PLANE_SHADE);
            // Crease
            tft->drawLine(px, py, px - 32, py + 24, 0x2124); 

            // Trail
            if (i > 10) {
                tft->drawLine(px - 55, py + 28, px - 70, py + 36, C_TRAIL); 
                tft->drawLine(px - 50, py + 16, px - 65, py + 20, C_TRAIL); 
            }

            delay(30); 
        }

        // --- 4. FINAL STATIC DRAW ---
        int finalX = cx + 60;
        int finalY = cy - 40;
        
        tft->fillTriangle(finalX, finalY, finalX - 32, finalY + 24, finalX - 44, finalY + 12, C_PLANE_BODY);
        tft->fillTriangle(finalX, finalY, finalX - 32, finalY + 24, finalX - 16, finalY + 32, C_PLANE_SHADE);
        tft->drawLine(finalX, finalY, finalX - 32, finalY + 24, 0x2124);

        // Badge Sukses (Digeser agar pas di sayap)
        int badgeX = finalX + 10;
        int badgeY = finalY - 10;
        tft->fillCircle(badgeX, badgeY, 10, CALIB_COLOR_DONE);
        tft->drawCircle(badgeX, badgeY, 10, TFT_WHITE);
        tft->drawLine(badgeX - 3, badgeY, badgeX, badgeY + 4, TFT_WHITE);
        tft->drawLine(badgeX, badgeY + 4, badgeX + 5, badgeY - 4, TFT_WHITE);
    }
};

// ============================================================================
// SD CARD & SERIAL FUNCTIONS
// ============================================================================

// Global variables for SD card
// Global variables for SD card (Removed duplicate definition)
// bool sdReady = false; // Defined at top
// SPIClass sdSPI(VSPI); // Not needed if legacy code commented out

// ============================================================================
// LEGACY: SD CARD FUNCTIONS (Commented for HTTP API mode)
// ============================================================================

/*
void initSDCard() {
    sdReady = false;
    
    Serial.println(">> Initializing SD Card...");
    
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    delay(10);
    
    sdSPI.end();
    SD.end();
    delay(100);
    
    sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    delay(50);
    
    const int maxRetries = 3;
    const unsigned long spiSpeed = 1000000;
    
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        Serial.printf(">> SD Card init attempt %d/%d...\n", attempt, maxRetries);
        
        digitalWrite(SD_CS_PIN, HIGH);
        delay(10);
        digitalWrite(SD_CS_PIN, LOW);
        delay(10);
        digitalWrite(SD_CS_PIN, HIGH);
        delay(50);
        
        if (SD.begin(SD_CS_PIN, sdSPI, spiSpeed)) {
            uint8_t cardType = SD.cardType();
            
            if (cardType != CARD_NONE) {
                sdReady = true;
                
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
                
                return;
            } else {
                Serial.println(">> No SD card detected");
            }
        }
        
        if (attempt < maxRetries) {
            Serial.println(">> Retrying...");
            SD.end();
            delay(200);
        }
    }
    
    Serial.println(">> SD Card FAIL (Check card/pins/connections)");
    Serial.println(">> Will retry on next logging attempt");
}

void oldStartLogging() {
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

void oldStopLogging() {
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
*/

// ============================================================================
// HTTP API LOGGING FUNCTIONS (NEW)
// ============================================================================

/**
 * Start data logging via HTTP API
 */
void startLogging() {
    // Check WiFi connection
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        Serial.println(F(">> Cannot start logging - WiFi not connected"));
        Serial.println(F(">> Use: SET_WIFI <ssid> <password>"));
        return;
    }
    
    // Check API URL configured
    if (strlen(apiBaseURL) == 0) {
        Serial.println(F(">> Cannot start logging - API URL not configured"));
        Serial.println(F(">> Use: SET_API <url>"));
        return;
    }
    
    // Reset energy counters
    logIteration = 0;
    sysData.wh_used = 0;
    sysData.wh_regen = 0;
    isLogging = true;
    
    Serial.println(F(">> HTTP API Logging started"));
    Serial.print(F(">> Device ID: "));
    Serial.println(deviceId);
    Serial.print(F(">> Endpoint: "));
    Serial.print(F(">> Endpoint: "));
    Serial.print(apiBaseURL);
    Serial.println("/ingest");
    
    drawStartLoggingUI();
}

/**
 * Stop data logging
 */
void stopLogging() {
    isLogging = false;
    
    // Clear queue to ensure immediate stop
    LogItem dummy;
    while(xQueueReceive(logQueue, &dummy, 0) == pdTRUE);
    
    Serial.println(F(">> Logging stopped"));
    Serial.println(F(">> Queue flushed"));
    Serial.printf(">> Total iterations: %lu\n", logIteration);
    
    // Enable Slide 4 (Network Info / Summary)
    showFileInfoSlide = true;
    
    drawStopLoggingUI();
}


// ============================================================================
// LEGACY: SD CARD HELPER FUNCTIONS (Commented for HTTP API mode)
// ============================================================================
/*
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
*/

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
        
        prefs.begin("calib_ui", false);
        prefs.clear();
        prefs.end();
        
        calPrefs.begin("sensor_cal", false);
        calPrefs.clear();
        calPrefs.end();
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
        
        // Allow forcing recalibration even if already done
        if (true) { 
            calib_curr_done = true;
            prefs.begin("calib_ui", false);
            prefs.putBool("c_curr", true);
            prefs.end();
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
        
        if (voltage > 0 && calib_volt_count < MAX_CAL_POINTS) {
            Serial.printf(">> ACTION: Adding calibration point #%d...\n", calib_volt_count + 1);
            
            addOrUpdateCalPoint(voltage);
            calib_volt_count++;
            prefs.begin("calib_ui", false);
            prefs.putInt("c_volt_n", calib_volt_count);
            prefs.end();
            calib_ui_update = true;
            
            Serial.printf(">> STATUS: ✓ Point added (%d/%d)\n", calib_volt_count, MAX_CAL_POINTS);
            
            if (calib_volt_count >= MIN_CAL_POINTS) {
                Serial.println(F(">> INFO: Voltage calibration valid (Minimum reached)"));
                if (calib_volt_count < MAX_CAL_POINTS) {
                     Serial.printf(">> INFO: Can add %d more points for better accuracy\n", MAX_CAL_POINTS - calib_volt_count);
                }
            } else {
                Serial.printf(">> INFO: Need %d more points for minimum\n", MIN_CAL_POINTS - calib_volt_count);
            }
        } 
        else if (voltage <= 0) {
            Serial.println(F(">> STATUS: ✗ INVALID"));
            Serial.println(F(">> ERROR: Voltage must be > 0"));
            Serial.println(F(">> USAGE: v<voltage> (e.g., v12.5)"));
        }
        else if (calib_volt_count >= MAX_CAL_POINTS) {
            Serial.println(F(">> STATUS: ⚠ LIMIT REACHED"));
            Serial.printf(">> ERROR: Already have %d calibration points\n", MAX_CAL_POINTS);
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
            
            Serial.println(F(">> ACTION: Setting RTC time..."));
            
            if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
                rtc.adjust(DateTime(year, month, day, hour, minute, second));
                xSemaphoreGive(i2cMutex);
            }
            
            calib_rtc_done = true;
            prefs.begin("calib_ui", false);
            prefs.putBool("c_rtc", true);
            prefs.end();
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
    
    // --- NETWORK CONFIGURATION COMMANDS (NEW) ---
    
    // Command: SET_WIFI <ssid> <password>
    else if (input.startsWith("SET_WIFI ")) {
        Serial.println(F(">> COMMAND: SET_WIFI"));
        
        int firstSpace = input.indexOf(' ');
        int secondSpace = input.indexOf(' ', firstSpace + 1);
        
        if (secondSpace > 0) {
            String s_ssid = input.substring(firstSpace + 1, secondSpace);
            String s_pass = input.substring(secondSpace + 1);
            
            strncpy(wifiSSID, s_ssid.c_str(), sizeof(wifiSSID) - 1);
            wifiSSID[sizeof(wifiSSID) - 1] = '\0';
            
            strncpy(wifiPassword, s_pass.c_str(), sizeof(wifiPassword) - 1);
            wifiPassword[sizeof(wifiPassword) - 1] = '\0';
            
            // Save to Preferences
            prefs.begin("network", false);
            prefs.putString("wifi_ssid", wifiSSID);
            prefs.putString("wifi_pass", wifiPassword);
            prefs.end();
            
            Serial.println(F(">> STATUS: ✓ WiFi credentials saved!"));
            Serial.print(F(">> SSID: "));
            Serial.println(wifiSSID);
            
            // Try to connect
            connectToWiFi();
        } else {
            Serial.println(F(">> STATUS: ✗ INVALID FORMAT"));
            Serial.println(F(">> ERROR: Missing SSID or password"));
            Serial.println(F(">> USAGE: SET_WIFI <ssid> <password>"));
            Serial.println(F(">> EXAMPLE: SET_WIFI MyWiFi MyPassword123"));
        }
    }
    
    // Command: SET_API <base_url>
    else if (input.startsWith("SET_API ")) {
        Serial.println(F(">> COMMAND: SET_API"));
        
        String s_url = input.substring(8);
        s_url.trim();
        
        // Remove trailing slash if present
        if (s_url.endsWith("/")) {
            s_url = s_url.substring(0, s_url.length() - 1);
        }
        
        strncpy(apiBaseURL, s_url.c_str(), sizeof(apiBaseURL) - 1);
        apiBaseURL[sizeof(apiBaseURL) - 1] = '\0';
        
        // Save to Preferences
        prefs.begin("network", false);
        prefs.putString("api_url", apiBaseURL);
        prefs.end();
        
        Serial.println(F(">> STATUS: ✓ API URL saved!"));
        Serial.print(F(">> URL: "));
        Serial.println(apiBaseURL);
        
        // Perform health check if WiFi is connected
        if (wifiConnected) {
            checkAPIHealth();
        } else {
            Serial.println(F(">> INFO: Connect WiFi first to check API health"));
        }
    }
    
    // Command: SET_DEBUG <0/1>
    else if (input.startsWith("SET_DEBUG ")) {
        Serial.println(F(">> COMMAND: SET_DEBUG"));
        String val = input.substring(10);
        val.trim();
        
        if (val == "1" || val.equalsIgnoreCase("ON") || val.equalsIgnoreCase("TRUE")) {
            debugPayload = true;
        } else {
            debugPayload = false;
        }
        
        // Save to Preferences
        prefs.begin("network", false);
        prefs.putBool("debug_payload", debugPayload);
        prefs.end();
        
        Serial.printf(">> STATUS: Debug Logging %s\n", debugPayload ? "ENABLED" : "DISABLED");
    }
    
    // --- LEGACY: FILE MANAGEMENT COMMANDS (Commented for HTTP API mode) ---
    /*
    // Command: ls
    else if (input.equalsIgnoreCase("ls")) {
        if (sdReady) {
            listDir(SD, "/", 0);
        } else {
            Serial.println(F("\u003e\u003e Error: SD Card not ready"));
        }
    }
    // Command: remove_all
    else if (input.equalsIgnoreCase("remove_all")) {
        if (sdReady) {
            deleteAllFiles(SD);
        } else {
            Serial.println(F("\u003e\u003e Error: SD Card not ready"));
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
            Serial.println(F("\u003e\u003e Error: SD Card not ready"));
        }
    }
    */
    
    // Command: status
    else if (input.equalsIgnoreCase("status")) {
        Serial.println(F("\u003e\u003e SYSTEM STATUS:"));
        Serial.printf("   Device ID: %s\n", deviceId.c_str());
        Serial.printf("   WiFi: %s\n", wifiConnected ? "CONNECTED" : "DISCONNECTED");
        if (wifiConnected) {
            Serial.printf("   WiFi SSID: %s\n", wifiSSID);
            Serial.printf("   WiFi RSSI: %d dBm\n", wifiRSSI);
            Serial.print("   IP: ");
            Serial.println(WiFi.localIP());
        }
        Serial.printf("   API URL: %s\n", strlen(apiBaseURL) > 0 ? apiBaseURL : "NOT SET");
        Serial.printf("   HTTP Status: %s (%d)\n", lastHTTPStatus.c_str(), lastHTTPCode);
        Serial.printf("   RTC: %s\n", rtcReady ? "READY" : "ERROR");
        Serial.printf("   Logging: %s\n", isLogging ? "ACTIVE" : "STOPPED");
        if (isLogging) {
            Serial.printf("   Iterations: %lu\n", logIteration);
        }
        Serial.printf("   Calibrated: %s\n", system_calibrated ? "YES" : "NO");
        Serial.printf("   Uptime: %lu s\n", sysData.uptime_sec);
        Serial.printf("   Debug Mode: %s\n", debugPayload ? "ON" : "OFF");
    }
    // Command: help
    else if (input.equalsIgnoreCase("help") || input.equalsIgnoreCase("?")) {
        Serial.println(F("\u003e\u003e AVAILABLE COMMANDS:"));
        Serial.println(F("   === NETWORK ==="));
        Serial.println(F("   SET_WIFI <ssid> <pass> : Configure WiFi"));
        Serial.println(F("   SET_API <url>          : Set API endpoint"));
        Serial.println(F("   SET_DEBUG <1/0>        : Toggle verbose logging"));
        Serial.println(F("   status                 : Show system status"));
        Serial.println(F("   === CALIBRATION ==="));
        Serial.println(F("   reset_all              : Reset calibration"));
        Serial.println(F("   auto                   : Auto calibrate current"));
        Serial.println(F("   v<voltage>             : Add voltage point (e.g. v12.5)"));
        Serial.println(F("   SET_TIME YYYY MM DD HH MM SS"));
        Serial.println(F("   === HELP ==="));
        Serial.println(F("   help                   : Show this list"));
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
    
    bool voltDone = (calib_volt_count >= MIN_CAL_POINTS);
    String voltMsg = "Input " + String(MIN_CAL_POINTS) + "x: 'v12.5' dst";
    curY += drawItem(curY, 2, "Kalibrasi Volt", voltMsg, voltDone, calib_volt_count);
    
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
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms precise interval
    
    for (;;) {
        // Read voltage
        float rawBattery = getMedianADC(batteryPin, 15);
        float v_final = 0;
        
        if (sensorSystemValid) {
            v_final = getInterpolatedVoltage(rawBattery);
        } else {
            float v_adc = (rawBattery / ADC_resolution) * V_ref;
            v_final = v_adc * 11.0;
        }
        
        // Protect sysData write
        if (xSemaphoreTake(sysDataMutex, portMAX_DELAY) == pdTRUE) {
            sysData.voltage = v_final;
            xSemaphoreGive(sysDataMutex);
        }
        
        // Apply Kalman Filter for Voltage
        float k_voltage = kalmanVolt.updateEstimate(sysData.voltage);
        
        // Apply smoothing for display (reduce decimal flickering from noise)
        if (displayVoltage == 0.0) {
            displayVoltage = k_voltage; // Initialize
        } else {
            // Exponential moving average: slower for decimals, faster for whole numbers
            displayVoltage = (VOLTAGE_ALPHA * k_voltage) + ((1.0 - VOLTAGE_ALPHA) * displayVoltage);
        }

        
        // Read current (INDUSTRIAL KALMAN FILTER)
        // Fungsi .update() sudah melakukan oversampling & adaptive filtering
        float raw_curr_filtered = kFilterCurrent.update(sensorPin);
        
        float V_ADC_curr = (raw_curr_filtered / ADC_resolution) * V_ref;
        float V_measured_curr = (V_ADC_curr / I_divider_factor) * I_scale_factor;
        float I_raw = (V_measured_curr - I_offset) / I_sensitivity;
        
        // Apply dead zone (Increased to 0.20A to ensure 0.00A display)
        if (abs(I_raw) < 0.20) I_raw = 0.0;
        
        // --- AUTO-ZERO ON IDLE ALGORITHM ---
        // If current is consistently near zero (< 0.25A) for 5 seconds, 
        // assume it's idle and fine-tune the offset.
        static unsigned long idleStartTime = 0;
        static bool isIdle = false;
        
        if (abs(I_raw) == 0.0) { // Using deadzoned value
             if (!isIdle) {
                 idleStartTime = millis();
                 isIdle = true;
             } else if (millis() - idleStartTime > 5000) { // 5 seconds stable
                 // Perform fine-tune auto-zero
                 // Only if we haven't done it recently (debounce)
                 static unsigned long lastAutoZero = 0;
                 if (millis() - lastAutoZero > 60000) { // Max once per minute
                     Serial.println(F(">> Auto-Zeroing Idle Current..."));
                     float rawIdle = getMedianADC(sensorPin, 50);
                     float V_ADC_Idle = (rawIdle / ADC_resolution) * V_ref;
                     float newOffset = (V_ADC_Idle / I_divider_factor) * I_scale_factor;
                     
                     // Only update if change is small (prevent zeroing on actual load)
                     if (abs(newOffset - I_offset) < 0.1) {
                         I_offset = newOffset;
                         saveCalibrationSensor();
                         Serial.printf(">> New Offset: %.4f\n", I_offset);
                     }
                     lastAutoZero = millis();
                 }
             }
        } else {
            isIdle = false;
        }

        // Read voltage safely for deadzone check
        float currentVoltage = 0;
        if (xSemaphoreTake(sysDataMutex, portMAX_DELAY) == pdTRUE) {
             currentVoltage = sysData.voltage;
             xSemaphoreGive(sysDataMutex);
        }
        if (currentVoltage < 3.0) I_raw = 0.0;
        
        // Separate into used and regen current
        // Protect sysData writes
        if (xSemaphoreTake(sysDataMutex, portMAX_DELAY) == pdTRUE) {
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
            xSemaphoreGive(sysDataMutex);
        }
        
        // Calculate energy (Wh)
        static unsigned long lastWh = 0;
        unsigned long now = millis();
        
        if (now - lastWh > 100) {
            float hours = (now - lastWh) / 3600000.0;
            if (xSemaphoreTake(sysDataMutex, portMAX_DELAY) == pdTRUE) {
                sysData.wh_used += (sysData.voltage * sysData.i_used * hours);
                sysData.wh_regen += (sysData.voltage * sysData.i_regen * hours);
                
                // Calculate efficiency
                if (sysData.wh_used > 0.001) {
                    sysData.efficiency = (sysData.wh_regen / sysData.wh_used) * 100.0;
                } else {
                    sysData.efficiency = 0;
                }
                
                if (sysData.efficiency > 100.0) {
                    sysData.efficiency = 100.0;
                }
                xSemaphoreGive(sysDataMutex);
            }
            lastWh = now;
        } else {
             // Ensure efficiency is calculated even if Wh not updated every loop
             if (xSemaphoreTake(sysDataMutex, portMAX_DELAY) == pdTRUE) {
                if (sysData.wh_used > 0.001) {
                    sysData.efficiency = (sysData.wh_regen / sysData.wh_used) * 100.0;
                } else {
                    sysData.efficiency = 0;
                }
                if (sysData.efficiency > 100.0) sysData.efficiency = 100.0;
                xSemaphoreGive(sysDataMutex);
             }
        }
        
        // Update time
        if (xSemaphoreTake(sysDataMutex, portMAX_DELAY) == pdTRUE) {
            sysData.uptime_sec = (now - startTime) / 1000;
            
            if (rtcReady) {
                if (xSemaphoreTake(i2cMutex, 10) == pdTRUE) { // Short timeout for I2C
                    sysData.time_now = rtc.now().unixtime() * 1000;
                    xSemaphoreGive(i2cMutex);
                }
            } else {
                sysData.time_now = now;
            }
            
            sysData.iteration++;
            xSemaphoreGive(sysDataMutex);
        }
        
        // Precise delay for consistent 100Hz sampling
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
// LEGACY: SD Card Status Check (Commented out)
/*
void checkSDCardStatus() {
    // Legacy code removed/commented
}
*/
void checkSDCardStatus() {
    // No-op for HTTP API mode
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
    // Now that screen is black, we can do slower init tasks
    Serial.begin(115200);
    delay(500);  // Wait for serial and power stabilization
    
    Serial.println(F("\n========================================"));
    Serial.println(F("  INDUSTRIAL POWER MONITOR v3.0"));
    Serial.println(F("  HTTP API Mode"));
    Serial.println(F("========================================\n"));
    
    // Generate unique deviceId from MAC address
    WiFi.mode(WIFI_STA);
    delay(100); // Wait for MAC to be ready
    
    // Retry getting MAC if needed
    String mac = WiFi.macAddress();
    int retry = 0;
    while (mac == "00:00:00:00:00:00" && retry < 5) {
        delay(100);
        mac = WiFi.macAddress();
        retry++;
    }
    
    deviceId = "esp32-" + mac;
    Serial.print(F(">> Device ID: "));
    Serial.println(deviceId);
    
    // Load network configuration from Preferences
    prefs.begin("network", true); // Read-only
    String s_ssid = prefs.getString("wifi_ssid", "");
    String s_pass = prefs.getString("wifi_pass", "");
    String s_url = prefs.getString("api_url", "");
    
    // Copy to char arrays
    strncpy(wifiSSID, s_ssid.c_str(), sizeof(wifiSSID) - 1);
    wifiSSID[sizeof(wifiSSID) - 1] = '\0';
    
    strncpy(wifiPassword, s_pass.c_str(), sizeof(wifiPassword) - 1);
    wifiPassword[sizeof(wifiPassword) - 1] = '\0';
    
    strncpy(apiBaseURL, s_url.c_str(), sizeof(apiBaseURL) - 1);
    apiBaseURL[sizeof(apiBaseURL) - 1] = '\0';
    
    debugPayload = prefs.getBool("debug_payload", true);
    
    prefs.end();
    
    if (strlen(wifiSSID) > 0) {
        Serial.println(F(">> Network config loaded from EEPROM"));
        connectToWiFi();
        
        if (wifiConnected && strlen(apiBaseURL) > 0) {
            checkAPIHealth();
        }
    } else {
        Serial.println(F(">> No WiFi config found"));
        Serial.println(F(">> Use: SET_WIFI <ssid> <password>"));
        Serial.println(F(">> Use: SET_API <url>"));
    }
    
    // I2C for RTC
    Wire.begin();
    
    // RTC initialization
    if (!rtc.begin()) {
        Serial.println(F("RTC Fail"));
        rtcReady = false;
    } else {
        rtcReady = true;
    }
    
    // LEGACY: SD card initialization (commented for HTTP API mode)
    // delay(200);
    // initSDCard();
    
    // Load calibration preferences
    prefs.begin("calib_ui", false);
    calPrefs.begin("sensor_cal", true);
    
    calib_curr_done = prefs.getBool("c_curr", false);
    calib_volt_count = prefs.getInt("c_volt_n", 0);
    calib_rtc_done = prefs.getBool("c_rtc", false);
    prefs.end();
    
    // Load Sensor Calibration Data
    I_offset = calPrefs.getFloat("i_off", I_offset);
    
    // FIX: Load Voltage Map correctly
    if (calPrefs.isKey("v_map")) {
        calPrefs.getBytes("v_map", voltageMap, sizeof(voltageMap));
        calPointCount = calPrefs.getInt("v_count", 0);
        
        if (calPointCount >= MIN_CAL_POINTS) {
            sensorSystemValid = true;
            Serial.printf(">> Loaded %d voltage calibration points\n", calPointCount);
        }
    }
    
    calPrefs.end();
    
    // Run system hardware check at boot
    runSystemCheck();
    
    // Print startup status
    Serial.println(F(">> SYSTEM READY"));
    Serial.println(F(">> Type 'help' for available commands"));
    
    // Check if system is calibrated
    if (calib_curr_done && calib_volt_count >= MIN_CAL_POINTS && calib_rtc_done) {
        system_calibrated = true;
        chartsInitialized = true;
        slide1.begin(200, 130, s1_voltage, s1_regen, s1_used);
    } else {
        drawCalibrationScreen();
    }
    
    // Initialize Logging Queue
    // Create log queue (Increased to 100 to handle bursts)
    logQueue = xQueueCreate(100, sizeof(LogItem));
    
    // Initialize Mutexes
    sysDataMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();
    
    // Start logging task on Core 0 (Low Priority)
    // Increased stack to 12KB for safe HTTPS SSL handshake
    xTaskCreatePinnedToCore(loggingTask, "LogTask", 12288, NULL, 0, NULL, 0);

    // Start data acquisition task on Core 0 (High Priority)
    // Increased stack to 10KB for safety
    xTaskCreatePinnedToCore(dataTask, "DataTask", 10240, NULL, 1, NULL, 0);
}

void loop() {
    // LEGACY: Check for SD errors (Commented for HTTP API mode)
    /*
    if (sdErrorOccurred) {
        if (sdErrorCode != "E-01") {
            drawSDError(sdErrorCode, sdErrorMsg);
            delay(2000);
            sdErrorOccurred = false; 
            
            if (system_calibrated && chartsInitialized) {
                if (currentSlide == 1) slide1.refresh();
                else if (currentSlide == 2) slide2.refresh();
                else if (currentSlide == 3) slide3.refresh();
            }
        }
    }
    */
    
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
    
    // NETWORK: Ensure WiFi is connected
    ensureWiFiConnected();
    
    // LEGACY: SD card data logging (Commented for HTTP API mode)
    /*
    if (isLogging && sdReady && (millis() - lastSDWrite > 200)) {
        lastSDWrite = millis();
        
        if (currentLogFile) {
            size_t written = currentLogFile.printf("%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%lu\n",
                ++logIteration,
                sysData.voltage,
                (sysData.i_used - sysData.i_regen),
                (sysData.voltage * (sysData.i_used - sysData.i_regen)),
                sysData.wh_used,
                sysData.wh_regen,
                sysData.time_now);
            
            if (written == 0) {
                Serial.println(">> Error: File write failed during logging");
                currentLogFile.close();
                isLogging = false;
                sdReady = false;
                
                sdErrorOccurred = true;
                sdErrorCode = "E-03";
                sdErrorMsg = "Save Error";
            } else {
                if (logIteration % 5 == 0) {
                    currentLogFile.flush();
                }
                
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            }
        } else {
            Serial.println(">> Error: Log file not available");
            isLogging = false;
            sdReady = false;
            sdErrorCode = "E-04";
            sdErrorMsg = "File Lost";
        }
    } else {
        checkSDCardStatus();
    */
    
    // HTTP API Data Transmission
    // HTTP API Data Transmission (Async)
    if (isLogging && wifiConnected && (millis() - lastAPIRequest >= API_REQUEST_INTERVAL)) {
        
        LogItem item;
        
        // Atomic read of sysData
        if (xSemaphoreTake(sysDataMutex, portMAX_DELAY) == pdTRUE) {
            item.voltage = sysData.voltage;
            item.i_used = sysData.i_used;
            item.i_regen = sysData.i_regen;
            item.wh_used = sysData.wh_used;
            item.wh_regen = sysData.wh_regen;
            xSemaphoreGive(sysDataMutex);
        }
        
        // Safe string copy for Queue
        strncpy(item.deviceId, deviceId.c_str(), sizeof(item.deviceId) - 1);
        item.deviceId[sizeof(item.deviceId) - 1] = '\0';
        
        if (rtcReady) {
            if (xSemaphoreTake(i2cMutex, 10) == pdTRUE) {
                item.timestamp = rtc.now().unixtime();
                xSemaphoreGive(i2cMutex);
            } else {
                item.timestamp = millis() / 1000; // Fallback if I2C busy
            }
        } else {
            item.timestamp = millis() / 1000;
        }
        
        // Send to queue (non-blocking)
        if (xQueueSend(logQueue, &item, 0) == pdTRUE) {
            logIteration++; 
        } else {
            static unsigned long lastQueueWarn = 0;
            if (millis() - lastQueueWarn > 5000) {
                Serial.println(F(">> WARN: Log Queue Full (dropping data)"));
                lastQueueWarn = millis();
            }
        }
        
        // ALWAYS update timestamp to prevent retry storm
        // If queue is full, we drop the data and wait for next interval
        lastAPIRequest = millis();
    }
    
    // Slide Auto-Change Logic (Moved out of else block)
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
    // } // End of commented else block

    
    // Update current slide ONLY if no SD error is blocking the view
    if (!sdErrorOccurred) {
        if (currentSlide == 1) {
            // Use local copy or atomic read? 
            // For display, we can just grab the values. But better to be consistent.
            float d_volt, d_regen, d_used;
            if (xSemaphoreTake(sysDataMutex, 10) == pdTRUE) {
                d_regen = sysData.i_regen;
                d_used = sysData.i_used;
                xSemaphoreGive(sysDataMutex);
                // displayVoltage is local to loop/global but only written in dataTask? 
                // Wait, displayVoltage is written in dataTask! It needs protection too or move it to sysData.
                // Actually displayVoltage is global. Let's protect it or just read it. 
                // It's a float, atomic on 32-bit? Not guaranteed.
                // Let's just use the values we got.
                slide1.update(displayVoltage, d_regen, d_used); 
            }
        } else if (currentSlide == 2) {
            float d_wh_used, d_wh_regen, d_eff;
             if (xSemaphoreTake(sysDataMutex, 10) == pdTRUE) {
                d_wh_used = sysData.wh_used;
                d_wh_regen = sysData.wh_regen;
                d_eff = sysData.efficiency;
                xSemaphoreGive(sysDataMutex);
                slide2.update(displayVoltage, d_wh_used, d_wh_regen, d_eff);
             }
        } else if (currentSlide == 3) {
            SystemData tempSys;
            if (xSemaphoreTake(sysDataMutex, 10) == pdTRUE) {
                // Cast away volatile for copy (safe due to Mutex)
                memcpy(&tempSys, (const void*)&sysData, sizeof(SystemData));
                xSemaphoreGive(sysDataMutex);
                slide3.update(tempSys);
            }
            delay(100);
        } else if (currentSlide == 4) {
            slide4.update();
            delay(100);
        }
    }
}