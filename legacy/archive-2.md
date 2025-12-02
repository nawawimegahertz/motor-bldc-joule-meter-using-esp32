/*
 * POWER MONITOR SYSTEM vFinal Grand Master
 * * Dibuat untuk Achmad Nawawi Ahlan
 *
 * Fitur Final:
 * 1. SMART STARTUP: Jika sudah dikalibrasi, langsung loncat ke Chart (Skip menu).
 * 2. NEW SUCCESS SCREEN: Ikon Ceklis Besar + "Selesai Kalibrasi!" (Centered).
 * 3. MULTI-SLIDE: Realtime Chart -> Energy Chart -> Dashboard Summary.
 * 4. ROBUST: FreeRTOS Core Separation (UI vs Data).
 * 5. LAYOUT PERFECTED: Axis rapi, Warna konsisten (BGR Fix).
 */

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>

// --- 1. DEFINISI & KONFIGURASI GLOBAL ---

// Warna BGR Fix
#define COLOR_BLUE   TFT_RED     
#define COLOR_CYAN   TFT_YELLOW  
#define COLOR_GREEN  TFT_GREEN
#define COLOR_RED    TFT_BLUE    
#define COLOR_TEXT_L 0xBDF7      
#define COLOR_TEXT_V TFT_WHITE   
#define COLOR_BG     TFT_BLACK
#define COLOR_GRID   0x02E0      

// Warna UI Kalibrasi
#define CALIB_COLOR_WARN    TFT_RED
#define CALIB_COLOR_DONE    TFT_GREEN
#define CALIB_COLOR_PENDING TFT_WHITE
#define CALIB_COLOR_DIM     0x528A

struct SystemData {
  float voltage;
  float i_regen;
  float i_used;
  float i_used_max;
  float i_used_min;
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
  uint16_t    color;
  float       peak;
};

enum WaveMode { SINE, SQUARE, TRIANGLE, SAWTOOTH, NOISE };

// --- 2. OBJEK GLOBAL ---

TFT_eSPI tft = TFT_eSPI();
Preferences prefs;

volatile SystemData sysData = {0};
bool system_calibrated = false;
bool chartsInitialized = false;

// Variabel Kalibrasi
bool calib_curr_done = false;
int  calib_volt_count = 0;
bool calib_rtc_done = false;
bool calib_ui_update = true;

// =========================================================================
//  BAGIAN A: KELAS-KELAS CHART
// =========================================================================

// --- CLASS SLIDE 1 ---
class Slide1_RTChart {
  private:
    TFT_eSPI *tft; TFT_eSprite *spr; int16_t *h1, *h2, *h3;
    int idx, w, h, mid_y; const int X_OFFSET = 30; const int Y_OFFSET = 10; 
    ChannelConfig c1, c2, c3;
  public:
    Slide1_RTChart(TFT_eSPI *t) : tft(t) { spr = NULL; h1=NULL; h2=NULL; h3=NULL; }
    void begin(int width, int height, ChannelConfig cf1, ChannelConfig cf2, ChannelConfig cf3) {
      w = width; h = height; mid_y = h-1; c1 = cf1; c2 = cf2; c3 = cf3;
      h1 = new int16_t[w]; h2 = new int16_t[w]; h3 = new int16_t[w];
      for(int i=0; i<w; i++) { h1[i]=mid_y; h2[i]=mid_y; h3[i]=mid_y; }
      idx = 0; spr = new TFT_eSprite(tft); spr->createSprite(w, h);
      tft->fillScreen(COLOR_BG); drawStaticAxis();
    }
    void end() {
      if(spr) { spr->deleteSprite(); delete spr; spr = NULL; }
      if(h1) { delete[] h1; h1=NULL; } if(h2) { delete[] h2; h2=NULL; } if(h3) { delete[] h3; h3=NULL; }
    }
    void update(float v1, float v2, float v3) {
      if(!spr) return;
      auto mapY = [&](float v, float p) { if(v<0) v=0; return constrain((int)(mid_y - (v/p)*(h-2)), 0, h-1); };
      h1[idx] = mapY(v1, c1.peak); h2[idx] = mapY(v2, c2.peak); h3[idx] = mapY(v3, c3.peak); idx = (idx + 1) % w;
      spr->fillSprite(COLOR_BG);
      int numDivX = 8; int numDivY = 4;
      for(int i=1; i<numDivX; i++) spr->drawFastVLine(i*w/numDivX, 0, h, COLOR_GRID);
      for(int i=1; i<numDivY; i++) spr->drawFastHLine(0, i*h/numDivY, w, COLOR_GRID);
      spr->drawRect(0, 0, w, h, COLOR_GRID);
      for(int x=0; x<w-1; x++) {
        int i = (idx + x) % w; int next = (idx + x + 1) % w;
        spr->drawLine(x, h1[i], x+1, h1[next], c1.color);
        spr->drawLine(x, h2[i], x+1, h2[next], c2.color);
        spr->drawLine(x, h3[i], x+1, h3[next], c3.color);
      }
      spr->pushSprite(X_OFFSET, Y_OFFSET); drawPanel(v1, v2, v3); drawPageNum();
    }
  private:
    void drawStaticAxis() {
      tft->setTextColor(COLOR_TEXT_L, COLOR_BG); tft->setTextSize(1);
      tft->setCursor(12, Y_OFFSET); tft->print((int)c1.peak);
      tft->setCursor(12, Y_OFFSET + (h/2) - 4); tft->print((int)(c1.peak/2));
      tft->setCursor(12, Y_OFFSET + h - 8); tft->print("0");
      int yX = Y_OFFSET + h + 4;
      tft->setCursor(X_OFFSET, yX); tft->print("0");
      tft->setCursor(X_OFFSET + (w/2) - 10, yX); tft->print("T/2");
      tft->setCursor(X_OFFSET + w - 10, yX); tft->print("T");
    }
    void drawPanel(float v1, float v2, float v3) {
      int y = 165; char buf[16]; tft->setTextPadding(100); 
      auto prt = [&](int x, int y, const char* l, float v, int d, uint16_t c) {
        tft->setCursor(x, y); tft->setTextColor(c, COLOR_BG); tft->setTextSize(1); tft->print(l);
        tft->setCursor(x, y+10); tft->setTextColor(c, COLOR_BG); tft->setTextSize(2); dtostrf(v, 4, d, buf); tft->print(buf);
      };
      prt(20, y, c1.name, v1, 1, c1.color);
      unsigned long s = millis()/1000; unsigned long d=s/86400; unsigned long h=(s%86400)/3600; unsigned long m=(s%3600)/60;
      if(d>0) sprintf(buf, "%dd %dh %dm", (int)d, (int)h, (int)m); else if(h>0) sprintf(buf, "%dh %dm", (int)h, (int)m); else sprintf(buf, "%dm %ds", (int)m, (int)(s%60));
      tft->setCursor(130, y); tft->setTextColor(COLOR_TEXT_V, COLOR_BG); tft->setTextSize(1); tft->print("Uptime");
      tft->setCursor(130, y+10); tft->setTextSize(2); tft->print(buf);
      prt(20, y+35, c2.name, v2, 2, c2.color);
      prt(130, y+35, c3.name, v3, 2, c3.color);
      tft->setTextPadding(0);
    }
    void drawPageNum() {
      tft->fillRect(218, 218, 22, 22, TFT_WHITE); tft->setTextColor(TFT_BLACK); 
      tft->setTextSize(2); tft->setCursor(223, 221); tft->print("1");
    }
};

// --- CLASS SLIDE 2 ---
class Slide2_EnergyChart {
  private:
    TFT_eSPI *tft; TFT_eSprite *spr; int16_t *h1, *h2, *h3; int idx, w, h, mid_y;
    const int X_OFFSET = 30; const int Y_OFFSET = 10; ChannelConfig c1, c2, c3;
  public:
    Slide2_EnergyChart(TFT_eSPI *t) : tft(t) { spr=NULL; h1=NULL; h2=NULL; h3=NULL; }
    void begin(int width, int height, ChannelConfig cf1, ChannelConfig cf2, ChannelConfig cf3) {
      w = width; h = height; mid_y = h-1; c1 = cf1; c2 = cf2; c3 = cf3;
      h1 = new int16_t[w]; h2 = new int16_t[w]; h3 = new int16_t[w];
      for(int i=0; i<w; i++) { h1[i]=mid_y; h2[i]=mid_y; h3[i]=mid_y; }
      idx = 0; spr = new TFT_eSprite(tft); spr->createSprite(w, h);
      tft->fillScreen(COLOR_BG); drawStaticAxis();
    }
    void end() {
      if(spr) { spr->deleteSprite(); delete spr; spr=NULL; }
      if(h1) { delete[] h1; h1=NULL; } if(h2) { delete[] h2; h2=NULL; } if(h3) { delete[] h3; h3=NULL; }
    }
    void update(float v1, float v2, float v3, float eff) {
      if(!spr) return;
      auto mapY = [&](float v, float p) { if(v<0) v=0; return constrain((int)(mid_y - (v/p)*(h-2)), 0, h-1); };
      h1[idx] = mapY(v1, c1.peak); h2[idx] = mapY(v2, c2.peak); h3[idx] = mapY(v3, c3.peak); idx = (idx + 1) % w;
      spr->fillSprite(COLOR_BG);
      int numDivX = 8; int numDivY = 4;
      for(int i=1; i<numDivX; i++) spr->drawFastVLine(i*w/numDivX, 0, h, COLOR_GRID);
      for(int i=1; i<numDivY; i++) spr->drawFastHLine(0, i*h/numDivY, w, COLOR_GRID);
      spr->drawRect(0, 0, w, h, COLOR_GRID);
      for(int x=0; x<w-1; x++) {
        int i = (idx + x) % w; int next = (idx + x + 1) % w;
        spr->drawLine(x, h1[i], x+1, h1[next], c1.color);
        spr->drawLine(x, h2[i], x+1, h2[next], c2.color);
        spr->drawLine(x, h3[i], x+1, h3[next], c3.color);
      }
      spr->pushSprite(X_OFFSET, Y_OFFSET); drawPanel(v1, v2, v3, eff); drawPageNum();
    }
  private:
    void drawStaticAxis() {
      tft->setTextColor(COLOR_TEXT_L, COLOR_BG); tft->setTextSize(1);
      tft->setCursor(12, Y_OFFSET); tft->print((int)c1.peak);
      tft->setCursor(12, Y_OFFSET + (h/2) - 4); tft->print((int)(c1.peak/2));
      tft->setCursor(12, Y_OFFSET + h - 8); tft->print("0");
      int yX = Y_OFFSET + h + 4;
      tft->setCursor(X_OFFSET, yX); tft->print("0");
      tft->setCursor(X_OFFSET + (w/2) - 10, yX); tft->print("T/2");
      tft->setCursor(X_OFFSET + w - 10, yX); tft->print("T");
    }
    void drawPanel(float v1, float v2, float v3, float eff) {
      int y = 165; char buf[16]; tft->setTextPadding(100);
      auto prt = [&](int x, int y, const char* l, float v, int d, uint16_t c) {
        tft->setCursor(x, y); tft->setTextColor(c, COLOR_BG); tft->setTextSize(1); tft->print(l);
        tft->setCursor(x, y+10); tft->setTextColor(c, COLOR_BG); tft->setTextSize(2); dtostrf(v, 4, d, buf); tft->print(buf);
      };
      prt(20, y, c1.name, v1, 1, c1.color);
      tft->setCursor(130, y); tft->setTextColor(COLOR_TEXT_V, COLOR_BG); tft->setTextSize(1); tft->print("Efisiensi");
      tft->setCursor(130, y+10); tft->setTextColor(COLOR_TEXT_V, COLOR_BG); tft->setTextSize(2);
      dtostrf(eff, 4, 1, buf); strcat(buf, "%"); tft->print(buf);
      prt(20, y+35, c2.name, v2, 2, c2.color);
      prt(130, y+35, c3.name, v3, 2, c3.color);
      tft->setTextPadding(0);
    }
    void drawPageNum() {
      tft->fillRect(218, 218, 22, 22, TFT_WHITE); tft->setTextColor(TFT_BLACK); tft->setTextSize(2); tft->setCursor(223, 221); tft->print("2");
    }
};

// --- CLASS SLIDE 3 ---
class Slide3_Dashboard {
  private: TFT_eSPI *tft; public: Slide3_Dashboard(TFT_eSPI *t) : tft(t) {}
    void begin() { tft->fillScreen(COLOR_BG); drawStatic(); }
    void end() { }
    void update(volatile SystemData &d) {
      tft->setTextWrap(false); tft->setTextPadding(100);
      prtVal(10, 50, d.voltage, 1, "V", COLOR_GREEN);
      tft->setCursor(130, 50); tft->setTextColor(COLOR_TEXT_V, COLOR_BG); tft->setTextSize(2); 
      char b[16]; dtostrf(d.efficiency, 4, 1, b); tft->print(b); tft->setTextSize(2); tft->print("%");
      prtVal(10, 95, d.i_used_max, 2, "A", COLOR_BLUE);
      prtVal(130, 95, d.i_used_min, 2, "A", COLOR_BLUE);
      prtVal(10, 140, d.wh_used, 2, "Wh", COLOR_RED);
      prtVal(130, 140, d.wh_regen, 2, "Wh", COLOR_CYAN);
      tft->setCursor(10, 185); tft->setTextColor(COLOR_TEXT_V, COLOR_BG); tft->setTextSize(2); tft->print(d.iteration);
      char tb[16]; unsigned long sec = d.uptime_sec; unsigned long dd = sec/86400; unsigned long hh = (sec%86400)/3600; unsigned long mm = (sec%3600)/60;
      if(dd>0) sprintf(tb, "%dd%dh%dm", (int)dd, (int)hh, (int)mm); else if(hh>0) sprintf(tb, "%dh%dm", (int)hh, (int)mm); else sprintf(tb, "%dm%ds", (int)mm, (int)(sec%60));
      tft->setCursor(130, 185); tft->setTextColor(COLOR_TEXT_V, COLOR_BG); tft->setTextSize(2); tft->print(tb);
      tft->setTextPadding(220); tft->setCursor(10, 220); tft->setTextColor(COLOR_TEXT_L, COLOR_BG); tft->setTextSize(1);
      unsigned long t1 = d.time_start/1000; unsigned long t2 = d.time_now/1000;
      tft->printf("Log: %02ld:%02ld -> %02ld:%02ld", (t1/60)%60, t1%60, (t2/60)%60, t2%60);
      tft->setTextPadding(0); drawPageNum();
    }
  private:
    void drawStatic() {
      tft->drawFastHLine(10, 25, 220, 0x3186);
      tft->setTextColor(COLOR_TEXT_L, COLOR_BG); tft->setTextSize(1);
      String title = "SYSTEM SUMMARY LOG"; int strW = tft->textWidth(title);
      tft->setCursor((240 - strW)/2, 10); tft->print(title);
      auto prtL = [&](int x, int y, const char* l, uint16_t c) { tft->setCursor(x, y); tft->setTextColor(c, COLOR_BG); tft->setTextSize(1); tft->print(l); };
      prtL(10, 40, "Tegangan (V)", COLOR_GREEN); prtL(130, 40, "Efisiensi", COLOR_TEXT_V);
      prtL(10, 85, "Arus Used (Max)", COLOR_BLUE); prtL(130, 85, "Arus Used (Min)", COLOR_BLUE);
      prtL(10, 130, "Energy Used", COLOR_RED); prtL(130, 130, "Energy Regen", COLOR_CYAN);
      prtL(10, 175, "Loop Iteration", COLOR_TEXT_L); prtL(130, 175, "Uptime", COLOR_TEXT_L);
      tft->drawFastHLine(10, 210, 220, 0x3186);
    }
    void prtVal(int x, int y, float v, int d, const char* u, uint16_t c) {
      char b[16]; dtostrf(v, 4, d, b); tft->setCursor(x, y); tft->setTextColor(c, COLOR_BG); tft->setTextSize(2); tft->print(b);
      tft->setTextSize(1); tft->setCursor(tft->getCursorX()+2, y+7); tft->print(u);
    }
    void drawPageNum() { tft->fillRect(218, 218, 22, 22, TFT_WHITE); tft->setTextColor(TFT_BLACK); tft->setTextSize(2); tft->setCursor(223, 221); tft->print("3"); }
};

// =========================================================================
//  BAGIAN B: FUNGSI KALIBRASI
// =========================================================================

void drawCheck(int x, int y, uint16_t color) {
  tft.drawLine(x, y+6, x+4, y+10, color); tft.drawLine(x+1, y+6, x+5, y+10, color);
  tft.drawLine(x+4, y+10, x+12, y-2, color); tft.drawLine(x+5, y+10, x+13, y-2, color);
}

int drawItem(int y, int idx, String title, String instr, bool isDone, int prog = -1) {
  if (isDone) {
    tft.setTextColor(CALIB_COLOR_DONE, COLOR_BG); tft.setTextSize(2); tft.setCursor(5, y); tft.print(idx);
    drawCheck(25, y, CALIB_COLOR_DONE);
    tft.setTextSize(1); tft.setCursor(45, y+4); tft.print(title); tft.print(" [OK]");
    tft.drawFastHLine(45, y+15, 150, 0x18E3); return 25;
  }
  tft.setTextColor(CALIB_COLOR_PENDING, COLOR_BG); tft.setTextSize(2); tft.setCursor(5, y); tft.print(idx);
  tft.setCursor(30, y); tft.print(title);
  tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_L, COLOR_BG); tft.setCursor(30, y+20); tft.print(instr);
  if (prog != -1) {
    tft.setCursor(30, y+32);
    for(int i=0; i<3; i++) { if (i < prog) tft.setTextColor(CALIB_COLOR_DONE); else tft.setTextColor(CALIB_COLOR_DIM); tft.print("[*] "); }
  }
  return (prog != -1) ? 50 : 40;
}

void drawCalibrationScreen() {
  tft.fillScreen(COLOR_BG);
  int head_y = 10; 
  tft.fillRect(0, head_y, 240, 40, 0x2104); tft.drawRect(0, head_y, 240, 40, CALIB_COLOR_WARN);
  tft.setTextSize(2); tft.setTextColor(CALIB_COLOR_WARN, 0x2104);
  String hTitle = "PERINGATAN!"; tft.setCursor((240 - tft.textWidth(hTitle))/2, head_y + 8); tft.print(hTitle);
  tft.setTextSize(1); tft.setTextColor(TFT_WHITE, 0x2104); tft.setCursor(80, head_y + 28); tft.print("SYSTEM LOCKED");

  int cur_y = head_y + 65; 
  cur_y += drawItem(cur_y, 1, "Kalibrasi Arus", "Pastikan 0A, ketik 'auto'", calib_curr_done);
  bool v_done = (calib_volt_count >= 3);
  cur_y += drawItem(cur_y, 2, "Kalibrasi Volt", "Input 3x: 'v12.5' dst", v_done, calib_volt_count);
  cur_y += drawItem(cur_y, 3, "Setup Waktu", "Ketik 'SET_TIME...'", calib_rtc_done);

  tft.drawFastHLine(0, 215, 240, 0x3186); tft.setCursor(0, 225); tft.setTextSize(1);
  tft.setTextColor(CALIB_COLOR_WARN, COLOR_BG); tft.setCursor(65, 225); tft.print("MENUNGGU INPUT...");
}

// (REVISI 1: Layar Sukses Baru)
void drawSuccessScreen() {
  tft.fillScreen(CALIB_COLOR_DONE); // Background Hijau
  // Ikon Ceklis Besar (Manual Lines)
  int cx = 120; int cy = 100;
  uint16_t c = TFT_BLACK;
  // Garis kiri
  for(int i=0; i<10; i++) tft.drawLine(cx-30+i, cy, cx+i, cy+30, c);
  // Garis kanan
  for(int i=0; i<10; i++) tft.drawLine(cx+i, cy+30, cx+50+i, cy-50, c);
  
  tft.setTextColor(TFT_BLACK, CALIB_COLOR_DONE);
  tft.setTextSize(2);
  String msg = "Selesai Kalibrasi!";
  tft.setCursor((240 - tft.textWidth(msg))/2, 160);
  tft.print(msg);
}

void checkSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); input.trim(); 
    if (input.length() == 0) return;
    if (input.equalsIgnoreCase("auto")) { 
      if(!calib_curr_done) { calib_curr_done = true; prefs.putBool("c_curr", true); calib_ui_update = true; }
    }
    else if (input.startsWith("v") || input.startsWith("V")) {
      float val = input.substring(1).toFloat();
      if (val > 0 && calib_volt_count < 3) { calib_volt_count++; prefs.putInt("c_volt_n", calib_volt_count); calib_ui_update = true; }
    }
    else if (input.startsWith("SET_TIME")) {
      if(!calib_rtc_done) { calib_rtc_done = true; prefs.putBool("c_rtc", true); calib_ui_update = true; }
    }
  }
}

// =========================================================================
//  BAGIAN C: CORE SYSTEM (FreeRTOS & Main Logic)
// =========================================================================

Slide1_RTChart slide1(&tft);
Slide2_EnergyChart slide2(&tft);
Slide3_Dashboard slide3(&tft);

// Config Label
ChannelConfig s1_v = {"Tegangan (V)", "", COLOR_GREEN, 60.0};
ChannelConfig s1_regen = {"Arus Regen (A)", "", COLOR_CYAN, 10.0};
ChannelConfig s1_used = {"Arus Used (A)", "", COLOR_BLUE, 10.0};
ChannelConfig s2_v = {"Tegangan (V)", "", COLOR_GREEN, 60.0};
ChannelConfig s2_used = {"Energy Used (Wh)", "", COLOR_RED, 100.0};
ChannelConfig s2_regen = {"Energy Regen (Wh)", "", COLOR_CYAN, 50.0};

void dataTask(void * parameter) {
  unsigned long start = millis();
  sysData.time_start = start;
  sysData.i_used_min = 999;

  for(;;) {
    unsigned long now = millis();
    float phase = now * 0.002;
    
    sysData.voltage = 48.0 + sin(phase) * 5.0;
    sysData.i_regen = (sin(phase*3) + 1) * 4.0;
    sysData.i_used = (sin(phase*0.5) > 0) ? 8.5 : 1.5;
    
    if(sysData.i_used > sysData.i_used_max) sysData.i_used_max = sysData.i_used;
    if(sysData.i_used < sysData.i_used_min) sysData.i_used_min = sysData.i_used;
    
    static unsigned long lastInteg = 0;
    if(now - lastInteg > 100) {
      float dt = (now - lastInteg) / 3600000.0; 
      sysData.wh_used += (sysData.voltage * sysData.i_used * dt);
      if(sysData.i_regen > 1.0) sysData.wh_regen += (sysData.voltage * sysData.i_regen * dt);
      lastInteg = now;
    }
    
    if(sysData.wh_used > 0) sysData.efficiency = (sysData.wh_regen / sysData.wh_used) * 100.0;
    if(sysData.efficiency > 100) sysData.efficiency = 100;

    sysData.uptime_sec = (now - start) / 1000;
    sysData.time_now = now;
    sysData.iteration++;
    
    vTaskDelay(50 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(COLOR_BG);

  prefs.begin("calib_data", false);
  // prefs.clear(); // Reset Switch
  
  calib_curr_done = prefs.getBool("c_curr", false);
  calib_volt_count = prefs.getInt("c_volt_n", 0);
  calib_rtc_done = prefs.getBool("c_rtc", false);

  // (REVISI 2: Smart-Skip Logic)
  // Jika sudah beres dari awal, langsung set true
  if (calib_curr_done && calib_volt_count >= 3 && calib_rtc_done) {
    system_calibrated = true;
    chartsInitialized = true;
    // Init Chart langsung
    slide1.begin(200, 130, s1_v, s1_regen, s1_used);
  } else {
    // Jika belum, gambar menu kalibrasi
    drawCalibrationScreen();
  }

  xTaskCreatePinnedToCore(dataTask, "DataTask", 10000, NULL, 1, NULL, 0);
}

int currentSlide = 1;
unsigned long lastSlideChange = 0;

void loop() {
  // --- FASE 1: KALIBRASI (BLOCKING) ---
  if (!system_calibrated) {
    checkSerialInput(); // Cek input
    
    // Cek apakah BARU SAJA selesai
    if (calib_curr_done && calib_volt_count >= 3 && calib_rtc_done) {
      // (REVISI 1: Tampilkan Layar Sukses)
      drawSuccessScreen();
      delay(2000); // Tahan sebentar biar terbaca
      
      // Masuk ke mode chart
      tft.fillScreen(COLOR_BG);
      slide1.begin(200, 130, s1_v, s1_regen, s1_used);
      system_calibrated = true;
      chartsInitialized = true;
    } 
    else if (calib_ui_update) {
      drawCalibrationScreen();
      calib_ui_update = false;
    }
    delay(100); 
    return; // Stop di sini sampai selesai
  }

  // --- FASE 2: CHART SLIDESHOW (Core 1) ---
  if (!chartsInitialized) return; 

  if(millis() - lastSlideChange > 10000) {
    lastSlideChange = millis();
    if(currentSlide == 1) slide1.end(); 
    else if(currentSlide == 2) slide2.end(); 
    else if(currentSlide == 3) slide3.end();
    
    currentSlide++; if(currentSlide > 3) currentSlide = 1;
    
    if(currentSlide == 1) slide1.begin(200, 130, s1_v, s1_regen, s1_used);
    else if(currentSlide == 2) slide2.begin(200, 130, s2_v, s2_used, s2_regen);
    else if(currentSlide == 3) slide3.begin();
  }

  if(currentSlide == 1) slide1.update(sysData.voltage, sysData.i_regen, sysData.i_used);
  else if(currentSlide == 2) slide2.update(sysData.voltage, sysData.wh_used, sysData.wh_regen, sysData.efficiency);
  else if(currentSlide == 3) { slide3.update(sysData); delay(100); }
}