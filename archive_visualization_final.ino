// /*
//  * INDUSTRIAL POWER MONITOR (Real Data + 3D UI Integration)
//  * Gabungan: vFinal Grand Master (UI) + Industrial Data Logger (Sensor Engine)
//  *
//  * FITUR:
//  * 1. SENSOR REAL (Core 0):
//  * - ACS758 (Arus) & Voltage Divider (Tegangan)
//  * - Median Filter & Auto Calibration
//  * 2. UI CANGGIH (Core 1):
//  * - 3D Traffic Light Logging Animation
//  * - Multi-Slide Chart (Realtime & Energy)
//  * - Calibration Wizard (Blocking UI)
//  * 3. DATA LOGGING:
//  * - SD Card CSV Logging (dikontrol via tombol D13)
//  * - RTC Timestamping
//  */

// #include <SPI.h>
// #include <TFT_eSPI.h>
// #include <Preferences.h>
// #include <SD.h>
// #include <FS.h>
// #include <Wire.h>
// #include <RTClib.h>

// // =========================================================================
// //  1. KONFIGURASI HARDWARE (PINOUT)
// // =========================================================================

// // UI & System
// #define BUTTON_PIN   13
// #define LED_PIN      2

// // Sensor (ADC)
// const int sensorPin  = 36;   // ACS758 (VP)
// const int batteryPin = 39;   // Voltage Divider (VN)

// // SD Card & RTC
// const int CS_PIN     = 5;    // Chip Select SD
// // RTC menggunakan I2C Default (SDA 21, SCL 22)

// // Parameter Sensor Default
// float I_offset = 2.2871;        
// float I_sensitivity = 0.020;    
// float I_divider_factor = 0.6667;
// float I_scale_factor = 1.1948;  
// const float V_ref = 3.342;          
// const float ADC_resolution = 4095.0; 

// // =========================================================================
// //  2. DEFINISI VISUAL & TEMA (JANGAN UBAH - vFinal Standard)
// // =========================================================================

// // Warna Shading 3D
// #define C_HOUSING_FRONT 0x4208 
// #define C_HOUSING_SIDE  0x2104 
// #define C_POLE          0xBDF7 
// #define C_POLE_SHADE    0x7BEF 

// // Warna Lampu (BGR Fix)
// #define TL_RED_ON     TFT_BLUE      
// #define TL_YELLOW_ON  TFT_CYAN      
// #define TL_GREEN_ON   TFT_GREEN     
// #define TL_RED_DIM    0x0008        
// #define TL_YELLOW_DIM 0x01E0        
// #define TL_GREEN_DIM  0x0100        

// // Warna Chart
// #define COLOR_BLUE   TFT_RED     
// #define COLOR_CYAN   TFT_YELLOW  
// #define COLOR_GREEN  TFT_GREEN
// #define COLOR_RED    TFT_BLUE    
// #define COLOR_TEXT_L 0xBDF7      
// #define COLOR_TEXT_V TFT_WHITE   
// #define COLOR_BG     TFT_BLACK
// #define COLOR_GRID   0x02E0 
// #define TFT_GREY     0x5AEB     
// #define TFT_ORANGE   0xFD20

// // Warna UI Kalibrasi
// #define CALIB_COLOR_WARN    TFT_RED
// #define CALIB_COLOR_DONE    TFT_GREEN
// #define CALIB_COLOR_PENDING TFT_WHITE
// #define CALIB_COLOR_DIM     0x528A

// // =========================================================================
// //  3. STRUKTUR DATA
// // =========================================================================

// struct SystemData {
//   float voltage; float i_regen; float i_used;
//   float i_used_max; float i_used_min;
//   float wh_used; float wh_regen; float efficiency;
//   unsigned long iteration; unsigned long uptime_sec;
//   unsigned long time_start; unsigned long time_now;
// };

// struct ChannelConfig {
//   const char* name; const char* unit; uint16_t color; float peak;
// };

// struct ProductConfig {
//   bool enabled; const char* name; const char* unit;
// };

// // Struktur Kalibrasi Sensor
// #define MAX_CAL_POINTS 6
// #define MIN_CAL_POINTS 3
// struct CalPoint { float rawADC; float realV; };

// enum Theme { CLASSIC = 1, MODERN = 0 };
// enum WaveMode { SINE, SQUARE, TRIANGLE, SAWTOOTH, NOISE };

// // =========================================================================
// //  4. OBJEK GLOBAL
// // =========================================================================

// TFT_eSPI tft = TFT_eSPI();
// Preferences prefs;     // Untuk Status UI (Setup Wizard)
// Preferences calPrefs;  // Untuk Data Sensor (Offset, Points)
// RTC_DS3231 rtc;
// File currentLogFile;

// // Data Shared
// volatile SystemData sysData = {0};

// // State UI
// bool system_calibrated = false;
// bool chartsInitialized = false;
// bool calib_curr_done = false;
// int  calib_volt_count = 0;
// bool calib_rtc_done = false;
// bool calib_ui_update = true;

// // State Logger & Button
// bool isLogging = false;
// bool sdReady = false;
// bool rtcReady = false;
// unsigned long logIteration = 0; 
// int buttonState;
// int lastButtonState = HIGH;
// unsigned long lastDebounceTime = 0;
// unsigned long debounceDelay = 50;

// // State Sensor
// CalPoint voltageMap[MAX_CAL_POINTS];
// int calPointCount = 0;
// bool sensorSystemValid = false;

// // =========================================================================
// //  5. HELPER FUNGSI SENSOR (MEDIAN & INTERPOLASI)
// // =========================================================================

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

// void saveCalibrationSensor() {
//   calPrefs.begin("sensor_cal", false);
//   calPrefs.putFloat("i_off", I_offset);
//   calPrefs.putFloat("i_sens", I_sensitivity);
//   calPrefs.putFloat("i_scale", I_scale_factor);
//   calPrefs.putBytes("v_map", voltageMap, sizeof(voltageMap));
//   calPrefs.putInt("v_count", calPointCount);
//   calPrefs.end();
// }

// void addOrUpdateCalPoint(float realVoltage) {
//   float currentRaw = getMedianADC(batteryPin, 50); 
//   int replaceIndex = -1;
//   float minDiff = 2.0;

//   for (int i=0; i < calPointCount; i++) {
//     if (abs(voltageMap[i].realV - realVoltage) < minDiff) {
//       replaceIndex = i; minDiff = abs(voltageMap[i].realV - realVoltage);
//     }
//   }

//   if (replaceIndex != -1) {
//     voltageMap[replaceIndex].rawADC = currentRaw; voltageMap[replaceIndex].realV = realVoltage;
//   } else if (calPointCount < MAX_CAL_POINTS) {
//     voltageMap[calPointCount].rawADC = currentRaw; voltageMap[calPointCount].realV = realVoltage;
//     calPointCount++;
//   } else {
//     voltageMap[0].rawADC = currentRaw; voltageMap[0].realV = realVoltage; // Overwrite oldest
//   }
  
//   // Sort Points
//   for (int i = 0; i < calPointCount - 1; i++) {
//     for (int j = 0; j < calPointCount - i - 1; j++) {
//       if (voltageMap[j].rawADC > voltageMap[j + 1].rawADC) {
//         CalPoint temp = voltageMap[j]; voltageMap[j] = voltageMap[j + 1]; voltageMap[j + 1] = temp;
//       }
//     }
//   }
//   saveCalibrationSensor();
//   if (calPointCount >= MIN_CAL_POINTS) sensorSystemValid = true;
// }

// void calculateAutoOffset() {
//   float raw = getMedianADC(sensorPin, 100);
//   float V_ADC = (raw / ADC_resolution) * V_ref;
//   I_offset = (V_ADC / I_divider_factor) * I_scale_factor;
//   saveCalibrationSensor();
// }

// // =========================================================================
// //  6. FUNGSI ANIMASI UI (3D LOGGING)
// // =========================================================================

// void fillQuad(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4, uint16_t color) {
//   tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
//   tft.fillTriangle(x1, y1, x3, y3, x4, y4, color);
// }

// void drawStartLoggingUI() {
//   tft.fillScreen(TFT_BLACK); 
//   int baseX=100, baseY=230, poleH=60, boxW=50, boxH=110, depth=8, tiltOffset=45;
//   int pX_top = baseX + (tiltOffset * poleH / (poleH + boxH)); int pY_top = baseY - poleH;
  
//   fillQuad(pX_top-3, pY_top, pX_top+3, pY_top, baseX+3, baseY, baseX-3, baseY, C_POLE);
//   fillQuad(pX_top+3, pY_top, pX_top+5, pY_top, baseX+5, baseY, baseX+3, baseY, C_POLE_SHADE);

//   int bx=pX_top, by=pY_top, topX=bx+30, topY=by-boxH;
//   int xBL=bx-(boxW/2), xBR=bx+(boxW/2), xTR=topX+(boxW/2), xTL=topX-(boxW/2);

//   fillQuad(xTL, topY, xTR, topY, xBR, by, xBL, by, C_HOUSING_FRONT);
//   fillQuad(xTR, topY, xTR+depth, topY, xBR+depth, by, xBR, by, C_HOUSING_SIDE);
//   fillQuad(xBR, by, xBR+depth, by, xBL+depth, by, xBL, by, C_HOUSING_SIDE);
//   tft.drawLine(xTL, topY, xTL+depth, topY, C_HOUSING_SIDE);

//   int l_r=12, ry=topY+25, rx=map(ry, topY, by, topX, bx);
//   int yy=topY+(boxH/2), yx=map(yy, topY, by, topX, bx);
//   int gy=by-25, gx=map(gy, topY, by, topX, bx);

//   tft.fillCircle(rx, ry, l_r+1, TFT_BLACK); tft.fillCircle(yx, yy, l_r+1, TFT_BLACK); tft.fillCircle(gx, gy, l_r+1, TFT_BLACK);
//   auto glare = [&](int lx, int ly) { tft.fillCircle(lx-4, ly-4, 3, TFT_WHITE); };

//   tft.setTextSize(2);
//   // RED
//   tft.fillCircle(rx, ry, l_r, TL_RED_ON); glare(rx, ry);
//   tft.fillCircle(yx, yy, l_r, TL_YELLOW_DIM); tft.fillCircle(gx, gy, l_r, TL_GREEN_DIM);
//   tft.setTextColor(TL_RED_ON, TFT_BLACK); tft.setCursor(10, 50); tft.print("SIAP..."); delay(600);
//   // YELLOW
//   tft.fillCircle(rx, ry, l_r, TL_RED_DIM); 
//   tft.fillCircle(yx, yy, l_r, TL_YELLOW_ON); glare(yx, yy);
//   tft.fillRect(0, 40, 100, 40, TFT_BLACK);
//   tft.setTextColor(TL_YELLOW_ON, TFT_BLACK); tft.setCursor(10, 80); tft.print("SET..."); delay(600);
//   // GREEN
//   tft.fillCircle(yx, yy, l_r, TL_YELLOW_DIM); 
//   tft.fillCircle(gx, gy, l_r, TL_GREEN_ON); glare(gx, gy);
//   tft.fillRect(0, 70, 100, 40, TFT_BLACK);
//   tft.setTextColor(TL_GREEN_ON, TFT_BLACK); tft.setTextSize(3); tft.setCursor(10, 110); tft.print("GO!!");
//   tft.setTextSize(1); tft.setCursor(10, 140); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.print("Data Logging...");
//   delay(1000); tft.fillScreen(COLOR_BG);
// }

// void drawStopLoggingUI() {
//   tft.fillScreen(COLOR_BG); 
//   int poleX=80, poleY=60, flagW=100, flagH=70, cellSize=10;
//   tft.fillRect(poleX-4, poleY, 4, 120, TFT_GREY); tft.fillCircle(poleX-2, poleY, 5, TFT_ORANGE);
//   for(int f=0; f<15; f++) {
//     tft.fillRect(poleX, poleY-10, flagW+10, flagH+20, COLOR_BG); 
//     for(int x=0; x<flagW/cellSize; x++) {
//       for(int y=0; y<flagH/cellSize; y++) {
//         int w = (int)(sin((x+f)*0.8)*5);
//         if(((x+y)%2==0)) tft.fillRect(poleX+(x*cellSize), poleY+(y*cellSize)+w, cellSize, cellSize, TFT_WHITE);
//       }
//     }
//     delay(30);
//   }
//   tft.setTextColor(TFT_WHITE, COLOR_BG); tft.setTextSize(2);
//   String msg = "LOG STOPPED"; tft.setCursor((240-tft.textWidth(msg))/2, 190); tft.print(msg);
//   delay(800); tft.fillScreen(COLOR_BG); 
// }

// // =========================================================================
// //  7. KELAS-KELAS CHART (vFinal Revised 6)
// // =========================================================================

// // (Salin paste definisi class Slide1_RTChart, Slide2_EnergyChart, Slide3_Dashboard 
// //  dari vFinal Revised 6 di sini. Strukturnya sama persis, tidak ada perubahan.)
// //  Untuk keringkasan di sini, saya akan menulis placeholder. 
// //  PASTIKAN ANDA MENYALIN KELAS-KELAS TERSEBUT KE SINI.

// class Slide1_RTChart {
//   private: TFT_eSPI *t; TFT_eSprite *s; int16_t *h1,*h2,*h3; int idx,w,h,my; ChannelConfig c1,c2,c3;
//   public: Slide1_RTChart(TFT_eSPI *p):t(p){s=0;h1=0;h2=0;h3=0;}
//   void begin(int W, int H, ChannelConfig C1, ChannelConfig C2, ChannelConfig C3) {
//     w=W;h=H;my=h-1; c1=C1;c2=C2;c3=C3; idx=0;
//     h1=new int16_t[w]; h2=new int16_t[w]; h3=new int16_t[w];
//     for(int i=0;i<w;i++){h1[i]=my;h2[i]=my;h3[i]=my;}
//     s=new TFT_eSprite(t); s->createSprite(w,h); refresh();
//   }
//   void refresh(){ t->fillScreen(COLOR_BG); drawAx(); }
//   void end(){ if(s){s->deleteSprite(); delete s; s=0;} if(h1){delete[] h1; h1=0;} if(h2){delete[] h2; h2=0;} if(h3){delete[] h3; h3=0;} }
//   void update(float v1, float v2, float v3) {
//     if(!s)return; s->fillSprite(COLOR_BG);
//     for(int i=1;i<8;i++) s->drawFastVLine(i*w/8,0,h,COLOR_GRID); for(int i=1;i<4;i++) s->drawFastHLine(0,i*h/4,w,COLOR_GRID); s->drawRect(0,0,w,h,COLOR_GRID);
//     auto m=[&](float v,float p){if(v<0)v=0;return constrain((int)(my-(v/p)*(h-2)),0,h-1);};
//     h1[idx]=m(v1,c1.peak); h2[idx]=m(v2,c2.peak); h3[idx]=m(v3,c3.peak); idx=(idx+1)%w;
//     for(int x=0;x<w-1;x++){ int i=(idx+x)%w; int n=(idx+x+1)%w; s->drawLine(x,h1[i],x+1,h1[n],c1.color); s->drawLine(x,h2[i],x+1,h2[n],c2.color); s->drawLine(x,h3[i],x+1,h3[n],c3.color); }
//     s->pushSprite(30,10); drawPan(v1,v2,v3); drawPg();
//   }
//   void drawAx(){ t->setTextColor(COLOR_TEXT_L,COLOR_BG); t->setTextSize(1); t->setCursor(12,10); t->print((int)c1.peak); t->setCursor(12,10+(h/2)-4); t->print((int)(c1.peak/2)); t->setCursor(12,10+h-8); t->print("0"); int yX=10+h+4; t->setCursor(30,yX); t->print("0"); t->setCursor(30+(w/2)-10,yX); t->print("T/2"); t->setCursor(30+w-10,yX); t->print("T"); }
//   void drawPan(float v1, float v2, float v3) {
//     int y=165; char b[16]; t->setTextPadding(100);
//     auto p=[&](int x,int y,const char*l,float v,int d,uint16_t c){t->setCursor(x,y);t->setTextColor(c,COLOR_BG);t->setTextSize(1);t->print(l);t->setCursor(x,y+10);t->setTextSize(2);dtostrf(v,4,d,b);t->print(b);};
//     p(20,y,c1.name,v1,1,c1.color); unsigned long s=millis()/1000;
//     char tb[10]; if(s<60)sprintf(tb,"%ds",(int)s); else sprintf(tb,"%dm",(int)s/60);
//     t->setCursor(130,y); t->setTextColor(COLOR_TEXT_V,COLOR_BG); t->setTextSize(1); t->print("Uptime"); t->setCursor(130,y+10); t->setTextSize(2); t->print(tb);
//     p(20,y+35,c2.name,v2,2,c2.color); p(130,y+35,c3.name,v3,2,c3.color); t->setTextPadding(0);
//   }
//   void drawPg(){ t->fillRect(218,218,22,22,TFT_WHITE); t->setTextColor(TFT_BLACK); t->setTextSize(2); t->setCursor(223,221); t->print("1"); }
// };

// class Slide2_EnergyChart {
//   private: TFT_eSPI *t; TFT_eSprite *s; int16_t *h1,*h2,*h3; int idx,w,h,my; ChannelConfig c1,c2,c3;
//   public: Slide2_EnergyChart(TFT_eSPI *p):t(p){s=0;h1=0;h2=0;h3=0;}
//   void begin(int W, int H, ChannelConfig C1, ChannelConfig C2, ChannelConfig C3) { w=W;h=H;my=h-1; c1=C1;c2=C2;c3=C3; idx=0; h1=new int16_t[w]; h2=new int16_t[w]; h3=new int16_t[w]; for(int i=0;i<w;i++){h1[i]=my;h2[i]=my;h3[i]=my;} s=new TFT_eSprite(t); s->createSprite(w,h); refresh(); }
//   void refresh(){ t->fillScreen(COLOR_BG); drawAx(); }
//   void end(){ if(s){s->deleteSprite(); delete s; s=0;} if(h1){delete[] h1; h1=0;} if(h2){delete[] h2; h2=0;} if(h3){delete[] h3; h3=0;} }
//   void update(float v1, float v2, float v3, float ef) {
//     if(!s)return; s->fillSprite(COLOR_BG); for(int i=1;i<8;i++) s->drawFastVLine(i*w/8,0,h,COLOR_GRID); for(int i=1;i<4;i++) s->drawFastHLine(0,i*h/4,w,COLOR_GRID); s->drawRect(0,0,w,h,COLOR_GRID);
//     auto m=[&](float v,float p){if(v<0)v=0;return constrain((int)(my-(v/p)*(h-2)),0,h-1);};
//     h1[idx]=m(v1,c1.peak); h2[idx]=m(v2,c2.peak); h3[idx]=m(v3,c3.peak); idx=(idx+1)%w;
//     for(int x=0;x<w-1;x++){ int i=(idx+x)%w; int n=(idx+x+1)%w; s->drawLine(x,h1[i],x+1,h1[n],c1.color); s->drawLine(x,h2[i],x+1,h2[n],c2.color); s->drawLine(x,h3[i],x+1,h3[n],c3.color); }
//     s->pushSprite(30,10); drawPan(v1,v2,v3,ef); drawPg();
//   }
//   private: 
//   void drawAx(){ t->setTextColor(COLOR_TEXT_L,COLOR_BG); t->setTextSize(1); t->setCursor(12,10); t->print((int)c1.peak); t->setCursor(12,10+(h/2)-4); t->print((int)(c1.peak/2)); t->setCursor(12,10+h-8); t->print("0"); int yX=10+h+4; t->setCursor(30,yX); t->print("0"); t->setCursor(30+(w/2)-10,yX); t->print("T/2"); t->setCursor(30+w-10,yX); t->print("T"); }
//   void drawPan(float v1, float v2, float v3, float ef) {
//     int y=165; char b[16]; t->setTextPadding(100);
//     auto p=[&](int x,int y,const char*l,float v,int d,uint16_t c){t->setCursor(x,y);t->setTextColor(c,COLOR_BG);t->setTextSize(1);t->print(l);t->setCursor(x,y+10);t->setTextColor(c,COLOR_BG);t->setTextSize(2);dtostrf(v,4,d,b);t->print(b);};
//     p(20,y,c1.name,v1,1,c1.color); t->setCursor(130,y); t->setTextColor(COLOR_TEXT_V,COLOR_BG); t->setTextSize(1); t->print("Efisiensi"); t->setCursor(130,y+10); dtostrf(ef,4,1,b); strcat(b,"%"); t->setTextSize(2); t->print(b);
//     p(20,y+35,c2.name,v2,2,c2.color); p(130,y+35,c3.name,v3,2,c3.color); t->setTextPadding(0);
//   }
//   void drawPg(){ t->fillRect(218,218,22,22,TFT_WHITE); t->setTextColor(TFT_BLACK); t->setTextSize(2); t->setCursor(223,221); t->print("2"); }
// };

// class Slide3_Dashboard {
//   private: TFT_eSPI *t; public: Slide3_Dashboard(TFT_eSPI *p):t(p){}
//   void begin(){ refresh(); } void refresh(){ t->fillScreen(COLOR_BG); drawStatic(); } void end(){}
//   void update(volatile SystemData &d) {
//     t->setTextWrap(false); t->setTextPadding(100);
//     prt(10,50,d.voltage,1,"V",COLOR_GREEN);
//     t->setCursor(130,50); t->setTextColor(COLOR_TEXT_V,COLOR_BG); t->setTextSize(2); char b[16]; dtostrf(d.efficiency,4,1,b); t->print(b); t->setTextSize(2); t->print("%");
//     prt(10,95,d.i_used_max,2,"A",COLOR_BLUE); prt(130,95,d.i_used_min,2,"A",COLOR_BLUE);
//     prt(10,140,d.wh_used,2,"Wh",COLOR_RED); prt(130,140,d.wh_regen,2,"Wh",COLOR_CYAN);
//     t->setCursor(10,185); t->setTextColor(COLOR_TEXT_V,COLOR_BG); t->setTextSize(2); t->print(d.iteration);
//     char tb[16]; unsigned long s=d.uptime_sec; sprintf(tb,"%02lu:%02lu:%02lu",(s/3600)%24,(s/60)%60,s%60);
//     t->setCursor(130,185); t->print(tb);
//     t->setTextPadding(220); t->setCursor(10,220); t->setTextColor(COLOR_TEXT_L,COLOR_BG); t->setTextSize(1);
//     unsigned long t1=d.time_start/1000; unsigned long t2=d.time_now/1000; t->printf("Log: %02lu:%02lu -> %02lu:%02lu",(t1/60)%60,t1%60,(t2/60)%60,t2%60);
//     t->setTextPadding(0); drawPg();
//   }
//   private:
//   void drawStatic(){ t->drawFastHLine(10,25,220,0x3186); t->setTextColor(COLOR_TEXT_L,COLOR_BG); t->setTextSize(1); String h="SYSTEM SUMMARY LOG"; t->setCursor((240-t->textWidth(h))/2,10); t->print(h);
//     auto p=[&](int x,int y,const char* l,uint16_t c){t->setCursor(x,y);t->setTextColor(c,COLOR_BG);t->setTextSize(1);t->print(l);};
//     p(10,40,"Tegangan (V)",COLOR_GREEN); p(130,40,"Efisiensi",COLOR_TEXT_V);
//     p(10,85,"Arus Used (Max)",COLOR_BLUE); p(130,85,"Arus Used (Min)",COLOR_BLUE);
//     p(10,130,"Energy Used",COLOR_RED); p(130,130,"Energy Regen",COLOR_CYAN);
//     p(10,175,"Loop Iteration",COLOR_TEXT_L); p(130,175,"Uptime",COLOR_TEXT_L); t->drawFastHLine(10,210,220,0x3186); }
//   void prt(int x,int y,float v,int d,const char* u,uint16_t c){ char b[16]; dtostrf(v,4,d,b); t->setCursor(x,y); t->setTextColor(c,COLOR_BG); t->setTextSize(2); t->print(b); t->setTextSize(1); t->setCursor(t->getCursorX()+2,y+7); t->print(u); }
//   void drawPg(){ t->fillRect(218,218,22,22,TFT_WHITE); t->setTextColor(TFT_BLACK); t->setTextSize(2); t->setCursor(223,221); t->print("3"); }
// };

// // =========================================================================
// //  BAGIAN C: FUNGSI KALIBRASI
// // =========================================================================

// void drawCheck(int x, int y, uint16_t color) {
//   tft.drawLine(x, y+6, x+4, y+10, color); tft.drawLine(x+1, y+6, x+5, y+10, color);
//   tft.drawLine(x+4, y+10, x+12, y-2, color); tft.drawLine(x+5, y+10, x+13, y-2, color);
// }

// int drawItem(int y, int idx, String title, String instr, bool isDone, int prog = -1) {
//   if (isDone) {
//     tft.setTextColor(CALIB_COLOR_DONE, COLOR_BG); tft.setTextSize(2); tft.setCursor(5, y); tft.print(idx);
//     drawCheck(25, y, CALIB_COLOR_DONE);
//     tft.setTextSize(1); tft.setCursor(45, y+4); tft.print(title); 
//     tft.setCursor(30, y); tft.setTextColor(CALIB_COLOR_DONE, COLOR_BG); tft.setTextSize(2); tft.print(title);
//     tft.drawFastHLine(45, y+18, 150, 0x18E3); return 30; 
//   }
//   tft.setTextColor(CALIB_COLOR_PENDING, COLOR_BG); tft.setTextSize(2); tft.setCursor(5, y); tft.print(idx);
//   tft.setCursor(30, y); tft.print(title);
//   tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_L, COLOR_BG); tft.setCursor(30, y+20); tft.print(instr);
//   if (prog != -1) {
//     tft.setCursor(30, y+32);
//     for(int i=0; i<3; i++) { if (i < prog) tft.setTextColor(CALIB_COLOR_DONE); else tft.setTextColor(CALIB_COLOR_DIM); tft.print("[*] "); }
//   }
//   return (prog != -1) ? 50 : 40;
// }

// void drawCalibrationScreen() {
//   tft.fillScreen(COLOR_BG);
//   int head_y = 10; 
//   tft.fillRect(0, head_y, 240, 40, 0x2104); tft.drawRect(0, head_y, 240, 40, CALIB_COLOR_WARN);
//   tft.setTextSize(2); tft.setTextColor(CALIB_COLOR_WARN, 0x2104);
//   String hTitle = "PERINGATAN!"; tft.setCursor((240 - tft.textWidth(hTitle))/2, head_y + 8); tft.print(hTitle);
//   tft.setTextSize(1); tft.setTextColor(TFT_WHITE, 0x2104); tft.setCursor(80, head_y + 28); tft.print("SYSTEM LOCKED");

//   int cur_y = head_y + 65; 
//   cur_y += drawItem(cur_y, 1, "Kalibrasi Arus", "Pastikan 0A, ketik 'auto'", calib_curr_done);
//   bool v_done = (calib_volt_count >= 3);
//   cur_y += drawItem(cur_y, 2, "Kalibrasi Volt", "Input 3x: 'v12.5' dst", v_done, calib_volt_count);
//   cur_y += drawItem(cur_y, 3, "Setup Waktu", "Ketik 'SET_TIME...'", calib_rtc_done);

//   tft.drawFastHLine(0, 215, 240, 0x3186); tft.setCursor(0, 225); tft.setTextSize(1);
//   tft.setTextColor(CALIB_COLOR_WARN, COLOR_BG); tft.setCursor(65, 225); tft.print("MENUNGGU INPUT...");
// }

// void drawSuccessScreen() {
//   tft.fillScreen(CALIB_COLOR_DONE); 
//   int cx = 120; int cy = 100; uint16_t c = TFT_BLACK;
//   for(int r=0; r<=45; r+=5) { tft.fillCircle(cx, cy, r, TFT_WHITE); delay(10); }
//   tft.fillCircle(cx, cy, 40, TFT_WHITE); 
//   for(int i=0; i<15; i++) { tft.fillCircle(cx-30+i, cy+i, 4, CALIB_COLOR_DONE); delay(10); }
//   for(int i=0; i<40; i++) { tft.fillCircle(cx-15+i, cy+15-i, 4, CALIB_COLOR_DONE); delay(5); }
//   tft.setTextColor(TFT_BLACK, CALIB_COLOR_DONE); tft.setTextSize(2);
//   String msg = "Selesai Kalibrasi!"; tft.setCursor((240 - tft.textWidth(msg))/2, 165); tft.print(msg);
//   delay(1000);
// }

// // =========================================================================
// //  7. FUNGSI SD CARD & SERIAL
// // =========================================================================

// void initSDCard() {
//   sdReady = false;
//   if (SD.begin(CS_PIN)) {
//     if (SD.cardType() != CARD_NONE) sdReady = true;
//   }
//   if(!sdReady) Serial.println(">> SD Card FAIL.");
// }

// void startLogging() {
//   String filename;
//   if (rtcReady) {
//     DateTime now = rtc.now();
//     char b[20]; sprintf(b, "/%02d%02d%02d%02d.csv", now.month(), now.day(), now.hour(), now.minute());
//     filename = String(b);
//   } else { filename = "/log_" + String(millis()) + ".csv"; }
  
//   currentLogFile = SD.open(filename, FILE_WRITE);
//   if (currentLogFile) {
//     currentLogFile.println("No,V,I,P,Wh_U,Wh_G,Time");
//     logIteration = 0; sysData.wh_used = 0; sysData.wh_regen = 0; // Reset Sesi
//     isLogging = true;
//     drawStartLoggingUI(); // 3D Animation
//   } else {
//     Serial.println(">> Error Writing File.");
//   }
// }

// void stopLogging() {
//   isLogging = false;
//   if (currentLogFile) currentLogFile.close();
//   drawStopLoggingUI(); // 3D Animation
// }

// void processSerial(String input) {
//   if (input.equalsIgnoreCase("reset_all")) { prefs.clear(); calib_curr_done = false; calib_volt_count = 0; calib_rtc_done = false; system_calibrated = false; calib_ui_update = true; }
//   else if (input.equalsIgnoreCase("auto")) { if(!calib_curr_done) { calib_curr_done=true; prefs.putBool("c_curr",true); calib_ui_update=true; calculateAutoOffset(); } }
//   else if (input.startsWith("v")) { float v=input.substring(1).toFloat(); if(v>0 && calib_volt_count<3) { addOrUpdateCalPoint(v); calib_volt_count++; prefs.putInt("c_volt_n",calib_volt_count); calib_ui_update=true; } }
//   else if (input.startsWith("SET_TIME")) { 
//     // Parsing simpel: SET_TIME Y M D H M S
//     // ... (Implementasi parsing RTC disini) ...
//     calib_rtc_done=true; prefs.putBool("c_rtc",true); calib_ui_update=true; 
//   }
// }

// // =========================================================================
// //  8. DATA TASK (CORE 0) - PUSAT LOGIKA REAL-TIME
// // =========================================================================

// void dataTask(void * parameter) {
//   unsigned long lastLogTime = 0;
  
//   for(;;) {
//     // 1. BACA SENSOR
//     float rawBat = getMedianADC(batteryPin, 15);
//     sysData.voltage = getInterpolatedVoltage(rawBat);
    
//     float rawCurr = getMedianADC(sensorPin, 20);
//     float V_adc = (rawCurr / ADC_resolution) * V_ref;
//     float V_sens = (V_adc / I_divider_factor) * I_scale_factor;
//     float I_raw = (V_sens - I_offset) / I_sensitivity;
    
//     // Noise Gate & Separator
//     if (abs(I_raw) < 0.15) I_raw = 0.0;
//     if (sysData.voltage < 3.0) I_raw = 0.0;
    
//     if (I_raw >= 0) { sysData.i_used = I_raw; sysData.i_regen = 0; }
//     else { sysData.i_used = 0; sysData.i_regen = abs(I_raw); }

//     // 2. HITUNG MIN/MAX
//     if (sysData.i_used > sysData.i_used_max) sysData.i_used_max = sysData.i_used;
//     if (sysData.i_used < sysData.i_used_min) sysData.i_used_min = sysData.i_used;

//     // 3. HITUNG ENERGI (Wh)
//     static unsigned long lastWh = 0;
//     unsigned long now = millis();
//     if (now - lastWh > 100) { // Integrasi tiap 100ms
//       float h = (now - lastWh) / 3600000.0;
//       sysData.wh_used += (sysData.voltage * sysData.i_used * h);
//       sysData.wh_regen += (sysData.voltage * sysData.i_regen * h);
//       lastWh = now;
//     }
    
//     // 4. HITUNG EFISIENSI
//     if (sysData.wh_used > 0) sysData.efficiency = (sysData.wh_regen / sysData.wh_used) * 100.0;
//     else sysData.efficiency = 0;
//     if (sysData.efficiency > 100) sysData.efficiency = 100;

//     // 5. UPDATE WAKTU
//     sysData.uptime_sec = millis() / 1000;
//     if (rtcReady) sysData.time_now = rtc.now().unixtime() * 1000;
//     else sysData.time_now = millis();

//     // 6. LOGGING KE SD CARD (Jika Aktif)
//     if (isLogging && sdReady && (now - lastLogTime > 1000)) { // Log tiap 1 detik
//       lastLogTime = now;
//       if (currentLogFile) {
//         currentLogFile.printf("%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%lu\n", 
//           ++logIteration, sysData.voltage, (sysData.i_used - sysData.i_regen), 
//           (sysData.voltage * (sysData.i_used - sysData.i_regen)), 
//           sysData.wh_used, sysData.wh_regen, sysData.time_now);
//         if (logIteration % 10 == 0) currentLogFile.flush();
//       }
//     }

//     vTaskDelay(pdMS_TO_TICKS(10)); 
//   }
// }

// // =========================================================================
// //  9. SETUP & LOOP (CORE 1)
// // =========================================================================

// Slide1_RTChart slide1(&tft);
// Slide2_EnergyChart slide2(&tft);
// Slide3_Dashboard slide3(&tft);

// ChannelConfig s1_v = {"Tegangan (V)", "", COLOR_GREEN, 60.0};
// ChannelConfig s1_regen = {"Arus Regen (A)", "", COLOR_CYAN, 10.0};
// ChannelConfig s1_used = {"Arus Used (A)", "", COLOR_BLUE, 10.0};
// ChannelConfig s2_v = {"Tegangan (V)", "", COLOR_GREEN, 60.0};
// ChannelConfig s2_used = {"Energy Used (Wh)", "", COLOR_RED, 100.0};
// ChannelConfig s2_regen = {"Energy Regen (Wh)", "", COLOR_CYAN, 50.0};

// int currentSlide = 1;
// unsigned long lastSlideChange = 0;

// void setup() {
//   Serial.begin(115200);
//   pinMode(BUTTON_PIN, INPUT_PULLUP);
//   pinMode(LED_PIN, OUTPUT);
  
//   tft.init(); tft.setRotation(0); tft.fillScreen(COLOR_BG);
//   Wire.begin(); SPI.begin(18, 23, 19, 5);
  
//   if(!rtc.begin()) rtcReady=false; else rtcReady=true;
//   initSDCard();

//   prefs.begin("calib_ui", false);
//   calPrefs.begin("sensor_cal", true); // Read only
  
//   // Load Calibration State
//   calib_curr_done = prefs.getBool("c_curr", false);
//   calib_volt_count = prefs.getInt("c_volt_n", 0);
//   calib_rtc_done = prefs.getBool("c_rtc", false);
  
//   // Load Sensor Calibration
//   I_offset = calPrefs.getFloat("i_off", I_offset);
//   // ... (Load parameter sensor lain dari EEPROM) ...
//   calPrefs.end();

//   if (calib_curr_done && calib_volt_count >= 3 && calib_rtc_done) {
//     system_calibrated = true; chartsInitialized = true;
//     slide1.begin(200, 130, s1_v, s1_regen, s1_used);
//   } else {
//     drawCalibrationScreen();
//   }

//   xTaskCreatePinnedToCore(dataTask, "DataTask", 8192, NULL, 1, NULL, 0);
// }

// void loop() {
//   // 1. Cek Tombol & Serial (Input)
//   if(Serial.available()) processSerial(Serial.readStringUntil('\n'));
  
//   int reading = digitalRead(BUTTON_PIN);
//   if (reading != lastButtonState) lastDebounceTime = millis();
//   if ((millis() - lastDebounceTime) > debounceDelay) {
//     if (reading != buttonState) {
//       buttonState = reading;
//       if (buttonState == LOW) { 
//         if(isLogging) stopLogging(); else startLogging(); 
//         // FIX: Refresh layar setelah animasi
//         if(system_calibrated) {
//            if(currentSlide==1) slide1.refresh(); 
//            else if(currentSlide==2) slide2.refresh(); 
//            else if(currentSlide==3) slide3.refresh();
//         }
//       }
//     }
//   }
//   lastButtonState = reading;

//   // 2. Logika UI Utama
//   if (!system_calibrated) {
//     if (calib_curr_done && calib_volt_count >= 3 && calib_rtc_done) {
//       drawSuccessScreen(); delay(2000); tft.fillScreen(COLOR_BG);
//       slide1.begin(200, 130, s1_v, s1_regen, s1_used);
//       system_calibrated = true; chartsInitialized = true;
//     } else if (calib_ui_update) {
//       drawCalibrationScreen(); calib_ui_update = false;
//     }
//     delay(100); return;
//   }

//   if (!chartsInitialized) return;

//   // 3. Transisi Slide (Background Timer Logic)
//   // Kita tidak mereset lastSlideChange saat logging, jadi waktu terus berjalan
//   if(millis() - lastSlideChange > 10000) {
//     lastSlideChange = millis();
//     if(currentSlide == 1) slide1.end(); 
//     else if(currentSlide == 2) slide2.end(); 
//     else if(currentSlide == 3) slide3.end();
    
//     currentSlide++; if(currentSlide > 3) currentSlide = 1;
    
//     if(currentSlide == 1) slide1.begin(200, 130, s1_v, s1_regen, s1_used);
//     else if(currentSlide == 2) slide2.begin(200, 130, s2_v, s2_used, s2_regen);
//     else if(currentSlide == 3) slide3.begin();
//   }

//   // 4. Render Slide (Menggunakan data dari sysData yang diupdate Core 0)
//   if(currentSlide == 1) slide1.update(sysData.voltage, sysData.i_regen, sysData.i_used);
//   else if(currentSlide == 2) slide2.update(sysData.voltage, sysData.wh_used, sysData.wh_regen, sysData.efficiency);
//   else if(currentSlide == 3) { slide3.update(sysData); delay(100); }
// }