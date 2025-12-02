// /*
//  * ESP32 INDUSTRIAL DATA LOGGER (FINAL INTEGRATION)
//  * Gabungan: Real-time Acquisition + SD Logger + RTC + Button Control
//  *
//  * FITUR:
//  * 1. Sensor Acquisition (Core 0): ACS758 & Voltage Divider dengan Median Filter.
//  * 2. Data Logging (Core 1): Simpan ke SD Card & Hitung Energi (Wh).
//  * 3. User Interface: Tombol Fisik (Start/Stop) & Serial Monitor (Kalibrasi & File Man).
//  *
//  * PINOUT:
//  * - SD Card (SPI): CS:D5, SCK:D18, MOSI:D19, MISO:D23
//  * - RTC DS3231 (I2C): SDA:D21, SCL:D22
//  * - Button: D13 (Active LOW / Pull-up) -> Tekan = GND
//  * - LED: D2
//  * - Sensor Arus (ACS758): D36 (VP)
//  * - Sensor Tegangan: D39 (VN)
//  */

// #include <Arduino.h>
// #include <SPI.h>
// #include <SD.h>
// #include <FS.h>
// #include <Wire.h>
// #include <RTClib.h>
// #include <Preferences.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"

// // ==================================================================
// // ===             KONFIGURASI PIN & HARDWARE                     ===
// // ==================================================================
// // Pin Logger
// const int BUTTON_PIN = 13;
// const int LED_PIN    = 2;
// const int CS_PIN     = 5;

// // Pin Sensor
// const int sensorPin  = 36;   // ACS758 (VP)
// const int batteryPin = 39;   // Voltage Divider (VN)

// // Konfigurasi Sistem
// #define DEBOUNCE_DELAY 50       
// #define BUTTON_ACTIVE_STATE LOW 
// #define LOG_INTERVAL_MS 100     // Sampling Rate

// // ==================================================================
// // ===             STRUKTUR DATA & KALIBRASI                      ===
// // ==================================================================
// #define MAX_CAL_POINTS 6
// #define MIN_CAL_POINTS 3

// struct CalPoint {
//   float rawADC;
//   float realV;
// };

// // Queue Data Structure
// struct SensorData {
//   float V_bat;
//   float I_sense;
// };

// // Global Objects
// Preferences preferences;
// const char* PREF_NAME = "ind_grade_v1";
// RTC_DS3231 rtc;
// QueueHandle_t sensorQueue;
// TaskHandle_t TaskSamplingHandle;
// TaskHandle_t TaskLoggingHandle;
// File currentLogFile;

// // Variabel Kalibrasi Default
// CalPoint voltageMap[MAX_CAL_POINTS];
// int calPointCount = 0;
// bool systemValid = false;

// // Parameter Sensor
// float I_offset = 2.2871;        
// float I_sensitivity = 0.020;    
// float I_divider_factor = 0.6667;
// float I_scale_factor = 1.1948;  
// const float V_ref = 3.342;          
// const float ADC_resolution = 4095.0; 

// // Variabel Logger Status
// volatile bool isLogging = false; 
// bool sdReady = false;           
// bool rtcReady = false;          
// unsigned long logIteration = 0; 

// // Integrasi Energi (Session Based)
// float total_Wh_used = 0.0;     
// float total_Wh_regen = 0.0;

// // Variabel Tombol
// int lastSteadyState = !BUTTON_ACTIVE_STATE; 
// int lastFlickerableState = !BUTTON_ACTIVE_STATE;
// unsigned long lastDebounceTime = 0;

// // ==================================================================
// // ===             PROTOTYPE FUNGSI                               ===
// // ==================================================================
// void loadCalibration();
// void saveCalibration();
// void addOrUpdateCalPoint(float realVoltage);
// void sortCalibrationPoints();
// float getInterpolatedVoltage(float rawADC);
// float getMedianADC(int pin, int samples);
// void calculateAutoOffset();
// void initSDCard();
// void startLogging();
// void stopLogging();
// void checkButton();
// void handleSingleClick();
// void processCommand(String input);

// // ==================================================================
// // ===             TASK 1: SENSOR SAMPLING (CORE 0)               ===
// // ==================================================================
// void TaskSampling(void *pvParameters) {
//   (void) pvParameters;
  
//   for (;;) {
//     // 1. Baca Tegangan (Median Filter)
//     float raw_bat = getMedianADC(batteryPin, 15);
//     float V_bat_final = 0.0;

//     if (systemValid) {
//        V_bat_final = getInterpolatedVoltage(raw_bat);
//     } else {
//        // Fallback
//        float v_adc_temp = (raw_bat / ADC_resolution) * V_ref;
//        V_bat_final = v_adc_temp / 0.04367; 
//     }

//     // 2. Baca Arus (Median Filter)
//     float raw_curr = getMedianADC(sensorPin, 20);
//     float V_ADC_curr = (raw_curr / ADC_resolution) * V_ref;
//     float V_measured_curr = (V_ADC_curr / I_divider_factor) * I_scale_factor;
//     float I_final = (V_measured_curr - I_offset) / I_sensitivity;

//     // 3. Kirim ke Queue
//     SensorData data;
//     data.V_bat = V_bat_final;
//     data.I_sense = I_final;
    
//     // Overwrite queue agar Core 1 selalu mendapat data terbaru
//     xQueueOverwrite(sensorQueue, &data);

//     vTaskDelay(pdMS_TO_TICKS(10)); // Jeda 10ms
//   }
// }

// // ==================================================================
// // ===             TASK 2: LOGGING & PROCESSING (CORE 1)          ===
// // ==================================================================
// void TaskLogging(void *pvParameters) {
//   SensorData recvData;
//   unsigned long lastTime = 0;
  
//   // Variabel Display Filter
//   float display_V = 0.0;
//   float display_I = 0.0;
//   float alpha = 0.15;

//   for (;;) {
//     unsigned long now = millis();

//     // Tunggu data dari Queue (Max delay 100ms sesuai rate logging)
//     if (xQueueReceive(sensorQueue, &recvData, portMAX_DELAY) == pdPASS) {
      
//       // 1. Hitung Delta Time (Jam)
//       if (lastTime == 0) lastTime = now;
//       float delta_h = (now - lastTime) / 3600000.0;
//       lastTime = now;

//       // 2. Filter Tampilan (Smoothing)
//       if (display_V == 0) display_V = recvData.V_bat;
//       else display_V = (alpha * recvData.V_bat) + ((1.0 - alpha) * display_V);

//       if (display_I == 0) display_I = recvData.I_sense;
//       else display_I = (alpha * recvData.I_sense) + ((1.0 - alpha) * display_I);

//       // 3. Noise Gate & Cutoff
//       float final_I = display_I;
//       if (abs(final_I) < 0.15) final_I = 0.0; // Zeroing noise arus kecil
//       if (display_V < 3.0) final_I = 0.0;     // Cutoff tegangan rendah

//       // 4. Kalkulasi Power & Energy
//       float power = final_I * display_V;
//       float wh = power * delta_h;
      
//       // Akumulasi Wh (Hanya diupdate jika sedang logging, atau mau global?)
//       // Sesuai request: Kita update terus global variable-nya, 
//       // tapi saat startLogging, variable ini akan di-reset (Lihat fungsi startLogging).
//       if (final_I >= 0) total_Wh_used += wh;
//       else total_Wh_regen += abs(wh);

//       // 5. PROSES LOGGING SD CARD
//       if (isLogging && sdReady) {
//          logIteration++;
         
//          // Ambil Timestamp
//          String timestamp;
//          if (rtcReady) {
//             DateTime dt = rtc.now();
//             char buf[25];
//             sprintf(buf, "%04d/%02d/%02d %02d:%02d:%02d", 
//                     dt.year(), dt.month(), dt.day(), 
//                     dt.hour(), dt.minute(), dt.second());
//             timestamp = String(buf);
//          } else {
//             timestamp = "RTC_ERR:" + String(millis());
//          }

//          // Format CSV: No,V,I,P,Wh_Use,Wh_Regen,Time
//          String dataLine = String(logIteration) + "," + 
//                            String(display_V, 2) + "," + 
//                            String(final_I, 2) + "," + 
//                            String(power, 2) + "," + 
//                            String(total_Wh_used, 4) + "," + 
//                            String(total_Wh_regen, 4) + "," + 
//                            timestamp;

//          if (currentLogFile) {
//             currentLogFile.println(dataLine);
//             if (logIteration % 20 == 0) currentLogFile.flush();
//          } else {
//             // Error handling
//             isLogging = false;
//             digitalWrite(LED_PIN, LOW);
//             Serial.println("\n[ERROR] File lost while logging!");
//          }
//       }

//       // 6. TAMPILKAN KE SERIAL (Setiap 200ms agar tidak spamming)
//       static unsigned long lastPrint = 0;
//       if (now - lastPrint > 200) {
//          lastPrint = now;
//          if (isLogging) Serial.print("[REC] "); 
//          else Serial.print("[MON] ");
         
//          Serial.print("V:"); Serial.print(display_V, 2);
//          Serial.print("V I:"); Serial.print(final_I, 2);
//          Serial.print("A P:"); Serial.print(power, 1);
//          Serial.print("W WhU:"); Serial.print(total_Wh_used, 3);
//          Serial.print(" WhG:"); Serial.println(total_Wh_regen, 3);
//       }
//     }
//   }
// }

// // ==================================================================
// // ===             SETUP & LOOP (CORE 1)                          ===
// // ==================================================================
// void setup() {
//   Serial.begin(115200);
//   delay(1000); // Stabilisasi Power

//   Serial.println("\n--- ESP32 INDUSTRIAL LOGGER BOOT ---");

//   // Config ADC
//   analogReadResolution(12);
//   analogSetAttenuation(ADC_11db);

//   // Load EEPROM
//   loadCalibration();

//   // Setup Hardware
//   pinMode(BUTTON_PIN, INPUT); 
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);

//   Wire.begin(); 
//   SPI.begin(18, 23, 19, 5); // SCK, MISO, MOSI, CS

//   // Init RTC
//   Serial.print("Init RTC... ");
//   if (!rtc.begin()) {
//     Serial.println("GAGAL (Check Wiring)");
//     rtcReady = false;
//   } else {
//     Serial.println("OK");
//     rtcReady = true;
//     if (rtc.lostPower()) {
//       Serial.println("RTC Lost Power, adjusting...");
//       rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//     }
//   }

//   // Init SD
//   initSDCard();

//   // Create Queue & Tasks
//   sensorQueue = xQueueCreate(1, sizeof(SensorData));

//   // Task Sampling (Core 0 - High Priority for ADC)
//   xTaskCreatePinnedToCore(TaskSampling, "Sampling", 4096, NULL, 2, &TaskSamplingHandle, 0);

//   // Task Logging/Processing (Core 1 - Standard Priority)
//   xTaskCreatePinnedToCore(TaskLogging, "Logging", 4096, NULL, 1, &TaskLoggingHandle, 1);
// }

// void loop() {
//   // Loop dijalankan oleh Core 1 (Bersamaan dengan TaskLogging)
//   // Fokus pada responsivitas tombol dan Serial Command

//   // 1. Cek Tombol
//   checkButton();

//   // 2. Cek Serial
//   if (Serial.available()) {
//     processCommand(Serial.readStringUntil('\n'));
//   }

//   delay(10); // Yield untuk Watchdog
// }

// // ==================================================================
// // ===             FUNGSI LOGIKA UTAMA                            ===
// // ==================================================================

// void checkButton() {
//   int currentState = digitalRead(BUTTON_PIN);

//   if (currentState != lastFlickerableState) {
//     lastDebounceTime = millis();
//     lastFlickerableState = currentState;
//   }

//   if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
//     if (currentState != lastSteadyState) {
//       lastSteadyState = currentState;
//       if (lastSteadyState == BUTTON_ACTIVE_STATE) {
//         handleSingleClick();
//       }
//     }
//   }
// }

// void handleSingleClick() {
//   Serial.println("\n>> Tombol Ditekan.");

//   if (!sdReady) {
//     Serial.println(">> SD ERROR. Retry Init...");
//     initSDCard();
//     if (sdReady) Serial.println(">> SD OK. Klik lagi untuk Start.");
//     return;
//   }

//   if (isLogging) stopLogging();
//   else startLogging();
// }

// void startLogging() {
//   String filename;
//   if (rtcReady) {
//     DateTime now = rtc.now();
//     char nameBuf[15];
//     sprintf(nameBuf, "/%02d%02d%02d%02d.csv", now.month(), now.day(), now.hour(), now.minute());
//     filename = String(nameBuf);
//   } else {
//     filename = "/log_" + String(millis()) + ".csv";
//   }

//   Serial.print(">> START LOGGING: "); Serial.println(filename);

//   currentLogFile = SD.open(filename, FILE_WRITE);
//   if (!currentLogFile) {
//     Serial.println("Error: Gagal write file SD.");
//     return;
//   }
  
//   // Header CSV
//   currentLogFile.println("Iteration,V_battery,Current_A,Power_W,Total_Wh_Used,Total_Wh_Regenerated,Timestamp");
  
//   // RESET COUNTER SESI BARU
//   logIteration = 0;
//   total_Wh_used = 0.0;
//   total_Wh_regen = 0.0;
  
//   isLogging = true; 
//   digitalWrite(LED_PIN, HIGH);
// }

// void stopLogging() {
//   isLogging = false;
//   delay(50); // Tunggu task selesai nulis
//   if (currentLogFile) {
//     currentLogFile.close();
//     Serial.println(">> STOP LOGGING: File Saved.");
//   }
//   digitalWrite(LED_PIN, LOW);
// }

// void initSDCard() {
//   sdReady = false;
//   if (isLogging) isLogging = false;

//   for (int i = 1; i <= 3; i++) {
//     if (SD.begin(CS_PIN)) {
//       if (SD.cardType() != CARD_NONE) {
//         Serial.println("SD Card OK.");
//         sdReady = true;
//         return; 
//       }
//     }
//     Serial.print("."); delay(200); 
//   }
//   Serial.println("\nSD Card GAGAL.");
//   // Blink Error
//   for(int k=0; k<3; k++) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }
// }

// // ==================================================================
// // ===             FUNGSI HELPER & KALIBRASI                      ===
// // ==================================================================

// void processCommand(String input) {
//   input.trim();
//   if (input == "") return;

//   // A. Command Logger
//   if (input == "ls") {
//     Serial.println("--- FILE LIST ---");
//     File root = SD.open("/");
//     while (File f = root.openNextFile()) {
//        float sizeKB = f.size() / 1024.0;
//        Serial.print(f.name()); Serial.print("\t"); Serial.print(sizeKB, 2); Serial.println(" kB");
//     }
//   } 
//   else if (input == "remove_all") {
//     File root = SD.open("/");
//     while (File f = root.openNextFile()) {
//         String n = f.name();
//         if (!n.startsWith("/")) n = "/" + n;
//         SD.remove(n);
//     }
//     Serial.println("All Files Removed.");
//   }
//   else if (input == "auto") {
//     calculateAutoOffset();
//   }
//   else if (input == "vreset") {
//     preferences.begin(PREF_NAME, false);
//     preferences.clear();
//     preferences.end();
//     ESP.restart();
//   }
//   // B. Command Kalibrasi (Format: v12.5, o2.5, s0.02)
//   else {
//     char cmd = input.charAt(0);
//     float val = input.substring(1).toFloat();
//     if (val != 0 || cmd == 'o' || cmd == 'v') {
//          if (cmd == 'v') addOrUpdateCalPoint(val);
//          else if (cmd == 'o') { I_offset = val; saveCalibration(); Serial.println("Offset Saved"); }
//          else if (cmd == 's') { I_sensitivity = val; saveCalibration(); Serial.println("Sens Saved"); }
//          else if (cmd == 'f') { I_scale_factor = val; saveCalibration(); Serial.println("Factor Saved"); }
//     }
//   }
// }

// // ... (Fungsi Helper Kalibrasi lainnya sama persis) ...

// void addOrUpdateCalPoint(float realVoltage) {
//   float currentRaw = getMedianADC(batteryPin, 50); 
//   int replaceIndex = -1;
//   float minDiff = 2.0;

//   for (int i=0; i < calPointCount; i++) {
//     if (abs(voltageMap[i].realV - realVoltage) < minDiff) {
//       replaceIndex = i;
//       minDiff = abs(voltageMap[i].realV - realVoltage);
//     }
//   }

//   if (replaceIndex != -1) {
//     voltageMap[replaceIndex].rawADC = currentRaw;
//     voltageMap[replaceIndex].realV = realVoltage;
//     Serial.println("Point Updated.");
//   } else if (calPointCount < MAX_CAL_POINTS) {
//     voltageMap[calPointCount].rawADC = currentRaw;
//     voltageMap[calPointCount].realV = realVoltage;
//     calPointCount++;
//     Serial.println("New Point Added.");
//   } else {
//     // Overwrite logic simple
//     voltageMap[0].rawADC = currentRaw;
//     voltageMap[0].realV = realVoltage;
//     Serial.println("Overwrite Index 0.");
//   }

//   sortCalibrationPoints();
//   saveCalibration();
//   if (calPointCount >= MIN_CAL_POINTS) systemValid = true;
// }

// void sortCalibrationPoints() {
//   for (int i = 0; i < calPointCount - 1; i++) {
//     for (int j = 0; j < calPointCount - i - 1; j++) {
//       if (voltageMap[j].rawADC > voltageMap[j + 1].rawADC) {
//         CalPoint temp = voltageMap[j];
//         voltageMap[j] = voltageMap[j + 1];
//         voltageMap[j + 1] = temp;
//       }
//     }
//   }
// }

// float getInterpolatedVoltage(float rawADC) {
//   if (calPointCount == 0) return 0.0;
//   if (rawADC <= voltageMap[0].rawADC) return voltageMap[0].realV;
//   if (rawADC >= voltageMap[calPointCount-1].rawADC) {
//     if (calPointCount < 2) return voltageMap[0].realV;
//     float m = (voltageMap[calPointCount-1].realV - voltageMap[calPointCount-2].realV) / 
//               (voltageMap[calPointCount-1].rawADC - voltageMap[calPointCount-2].rawADC);
//     return voltageMap[calPointCount-1].realV + (m * (rawADC - voltageMap[calPointCount-1].rawADC));
//   }
//   for (int i = 0; i < calPointCount - 1; i++) {
//     if (rawADC >= voltageMap[i].rawADC && rawADC <= voltageMap[i+1].rawADC) {
//       float m = (voltageMap[i+1].realV - voltageMap[i].realV) / 
//                 (voltageMap[i+1].rawADC - voltageMap[i].rawADC);
//       return voltageMap[i].realV + (m * (rawADC - voltageMap[i].rawADC));
//     }
//   }
//   return 0.0;
// }

// float getMedianADC(int pin, int samples) {
//   int raw[samples];
//   for (int i=0; i<samples; i++) { raw[i] = analogRead(pin); delayMicroseconds(50); }
//   for(int i=0; i<samples-1; i++){
//     for(int j=0; j<samples-i-1; j++){
//       if(raw[j] > raw[j+1]){ int t = raw[j]; raw[j]=raw[j+1]; raw[j+1]=t; }
//     }
//   }
//   return (float)raw[samples/2];
// }

// void saveCalibration() {
//   preferences.begin(PREF_NAME, false);
//   preferences.putFloat("i_off", I_offset);
//   preferences.putFloat("i_sens", I_sensitivity);
//   preferences.putFloat("i_scale", I_scale_factor);
//   preferences.putBytes("v_map", voltageMap, sizeof(voltageMap));
//   preferences.putInt("v_count", calPointCount);
//   preferences.end();
// }

// void loadCalibration() {
//   preferences.begin(PREF_NAME, true);
//   I_offset = preferences.getFloat("i_off", I_offset);
//   I_sensitivity = preferences.getFloat("i_sens", I_sensitivity);
//   I_scale_factor = preferences.getFloat("i_scale", I_scale_factor);
//   if (preferences.isKey("v_map")) {
//     preferences.getBytes("v_map", voltageMap, sizeof(voltageMap));
//     calPointCount = preferences.getInt("v_count", 0);
//   }
//   if (calPointCount >= MIN_CAL_POINTS) systemValid = true;
//   preferences.end();
// }

// void calculateAutoOffset() {
//   Serial.println("Calibrating Offset...");
//   float raw = getMedianADC(sensorPin, 100);
//   float V_ADC = (raw / ADC_resolution) * V_ref;
//   I_offset = (V_ADC / I_divider_factor) * I_scale_factor;
//   saveCalibration();
//   Serial.println("New Offset: " + String(I_offset, 4));
// }