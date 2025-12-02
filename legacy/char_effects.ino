// /*
//  * DEMO REEL GRAFIS ANIMASI DINAMIS (v4 - Robust Timer Fix)
//  * * Dibuat untuk Achmad Nawawi Ahlan
//  *
//  * Perbaikan v4:
//  * - Memperbaiki BUG STUCK di Demo 4 (runLinesDemo) dan mencegahnya
//  * terjadi di semua demo lain (runFastLines, runRects, dll.)
//  * - Menerapkan arsitektur timer state-dependent yang benar-benar robust.
//  */

// #include <SPI.h>
// #include <TFT_eSPI.h> // Sertakan library TFT_eSPI

// TFT_eSPI tft = TFT_eSPI();

// // --- Pengaturan State Machine ---
// const int DEMO_COUNT = 11;
// int currentDemo = 0;
// bool demoInitialized = false;

// int16_t w, h; // Variabel global dimensi layar

// // =========================================================================
// //  SETUP
// // =========================================================================
// void setup() {
//   Serial.begin(115200);
//   Serial.println("Memulai Demo Reel Animasi v4 (Robust Timer Fix)...");

//   tft.init();
//   tft.setRotation(0);
//   tft.fillScreen(TFT_BLACK);

//   w = tft.width();
//   h = tft.height();

//   Serial.println("Inisialisasi selesai.");
// }

// // =========================================================================
// //  LOOP UTAMA (State Machine)
// // =========================================================================
// void loop() {
//   bool demoFinished = false;

//   switch (currentDemo) {
//     case 0: demoFinished = runIntroDemo(); break;
//     case 1: demoFinished = runUptimeDemo(); break;
//     case 2: demoFinished = runLoremIpsumDemo(); break;
//     case 3: demoFinished = runPrintTestDemo(); break;
//     case 4: demoFinished = runLinesDemo(); break;
//     case 5: demoFinished = runFastLinesDemo(); break;
//     case 6: demoFinished = runRectsDemo(); break;
//     case 7: demoFinished = runCirclesDemo(); break;
//     case 8: demoFinished = runRoundrectsDemo(); break;
//     case 9: demoFinished = runTrianglesDemo(); break;
//     case 10: demoFinished = runInvertDemo(); break;
//   }

//   if (demoFinished) {
//     Serial.printf("Demo %d selesai.\n", currentDemo);
//     currentDemo = (currentDemo + 1) % DEMO_COUNT;
//     demoInitialized = false;
    
//     tft.setRotation(0);
//     tft.setTextWrap(true);
//     tft.invertDisplay(false);
//     Serial.printf("Memulai Demo %d...\n", currentDemo);
//   }
// }

// // =========================================================================
// //  FUNGSI DEMO (v4 - Timer Diperbaiki)
// // =========================================================================

// // DEMO 0: Intro (Logika v3 sudah benar)
// bool runIntroDemo() {
//   static int stage = 0;
//   static unsigned long lastActionTime = 0;

//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     tft.setCursor(0, 10);
//     stage = 0;
//     lastActionTime = millis();
//     demoInitialized = true;
//   }

//   unsigned long waitTime = (stage < 4) ? 500 : 1000;

//   if (millis() - lastActionTime > waitTime) {
//     lastActionTime = millis(); 
//     switch (stage) {
//       case 0:
//         tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
//         tft.println("Halo!");
//         stage++;
//         break;
//       case 1:
//         tft.println("Ini adalah ESP32");
//         stage++;
//         break;
//       case 2:
//         tft.setTextColor(TFT_GREEN, TFT_BLACK); tft.setTextSize(3);
//         tft.println("ST7789 TFT");
//         stage++;
//         break;
//       case 3:
//         tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.setTextSize(2);
//         tft.println("240x240 Display");
//         stage++;
//         break;
//       case 4:
//         return true; 
//     }
//   }
//   return false;
// }

// // DEMO 1: Uptime (Logika v3 sudah benar)
// bool runUptimeDemo() {
//   static unsigned long startTime, lastUpdateTime;
//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextSize(2);
//     tft.setCursor(0, 80); tft.println("Demo Uptime Dinamis:");
//     startTime = millis(); lastUpdateTime = millis();
//     demoInitialized = true;
//   }

//   if (millis() - startTime > 8000) return true;

//   if (millis() - lastUpdateTime > 100) {
//     tft.setCursor(0, 120);
//     tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextSize(2);
//     tft.print("Uptime: "); tft.print(millis() / 1000); tft.println(" detik "); 
//     lastUpdateTime = millis();
//   }
//   return false;
// }

// // DEMO 2: Lorem Ipsum (Logika v3 sudah benar)
// bool runLoremIpsumDemo() {
//   static const char* lorem = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis.";
//   static const char* charPos;
//   static unsigned long lastCharTime;
//   const int CHAR_DELAY_MS = 20;

//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     tft.setCursor(0, 0);
//     tft.setTextColor(TFT_WHITE); tft.setTextWrap(true); tft.setTextSize(1);
//     charPos = lorem; lastCharTime = millis();
//     demoInitialized = true;
//   }

//   if (millis() - lastCharTime > CHAR_DELAY_MS) {
//     if (*charPos == '\0') {
//       delay(1000); return true;
//     }
//     tft.print(*charPos);
//     charPos++;
//     lastCharTime = millis();
//   }
//   return false;
// }

// // DEMO 3: Print Test (Logika v3 sudah benar)
// bool runPrintTestDemo() {
//   static int stage = 0;
//   static const char* lines[] = { "Hello World!", "Hello World!", "Hello World!", "1234.567" };
//   static const uint16_t colors[] = { TFT_RED, TFT_YELLOW, TFT_GREEN, TFT_BLUE };
//   static const uint8_t sizes[] = { 1, 2, 3, 4 };
//   static int charIndex;
//   static unsigned long lastTime;
//   const int CHAR_DELAY_MS = 50;
//   const int LINE_DELAY_MS = 500;

//   if (!demoInitialized) {
//     stage = 0; charIndex = 0; lastTime = millis();
//     tft.setTextWrap(false); tft.fillScreen(TFT_BLACK);
//     tft.setCursor(0, 30);
//     tft.setTextColor(colors[stage]); tft.setTextSize(sizes[stage]);
//     demoInitialized = true;
//   }

//   if (stage >= 4) {
//     if (millis() - lastTime > 1500) return true;
//     return false;
//   }

//   unsigned long waitTime = (charIndex == 0) ? LINE_DELAY_MS : CHAR_DELAY_MS;

//   if (millis() - lastTime > waitTime) {
//     lastTime = millis();
//     if (lines[stage][charIndex] == '\0') {
//       stage++; charIndex = 0;
//       if (stage < 4) {
//         tft.println();
//         tft.setTextColor(colors[stage]); tft.setTextSize(sizes[stage]);
//       }
//     } else {
//       tft.print(lines[stage][charIndex]);
//       charIndex++;
//     }
//   }
//   return false;
// }

// // DEMO 4: Garis (FIXED)
// bool runLinesDemo() {
//   static int x, y, stage;
//   static unsigned long lastActionTime;

//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     x = 0; y = 0; stage = 0;
//     lastActionTime = millis();
//     demoInitialized = true;
//   }

//   switch (stage) {
//     // Stage 0-7 adalah STAGE AKSI (menggambar)
//     case 0:
//       if (millis() - lastActionTime > 5) {
//         tft.drawLine(0, 0, x, h-1, TFT_YELLOW); x+=6; lastActionTime = millis();
//         if (x >= w) { x=0; stage++; }
//       }
//       break;
//     case 1:
//       if (millis() - lastActionTime > 5) {
//         tft.drawLine(0, 0, w-1, y, TFT_YELLOW); y+=6; lastActionTime = millis();
//         if (y >= h) { y=0; stage++; tft.fillScreen(TFT_BLACK); }
//       }
//       break;
//     case 2:
//       if (millis() - lastActionTime > 5) {
//         tft.drawLine(w-1, 0, x, h-1, TFT_CYAN); x+=6; lastActionTime = millis();
//         if (x >= w) { x=0; stage++; }
//       }
//       break;
//     case 3:
//       if (millis() - lastActionTime > 5) {
//         tft.drawLine(w-1, 0, 0, y, TFT_CYAN); y+=6; lastActionTime = millis();
//         if (y >= h) { y=0; stage++; tft.fillScreen(TFT_BLACK); }
//       }
//       break;
//     case 4:
//       if (millis() - lastActionTime > 5) {
//         tft.drawLine(0, h-1, x, 0, TFT_GREEN); x+=6; lastActionTime = millis();
//         if (x >= w) { x=0; stage++; }
//       }
//       break;
//     case 5:
//       if (millis() - lastActionTime > 5) {
//         tft.drawLine(0, h-1, w-1, y, TFT_GREEN); y+=6; lastActionTime = millis();
//         if (y >= h) { y=0; stage++; tft.fillScreen(TFT_BLACK); }
//       }
//       break;
//     case 6:
//       if (millis() - lastActionTime > 5) {
//         tft.drawLine(w-1, h-1, x, 0, TFT_RED); x+=6; lastActionTime = millis();
//         if (x >= w) { x=0; stage++; }
//       }
//       break;
//     case 7:
//       if (millis() - lastActionTime > 5) {
//         tft.drawLine(w-1, h-1, 0, y, TFT_RED); y+=6; lastActionTime = millis();
//         if (y >= h) { y=0; stage++; lastActionTime = millis(); } // Pindah ke stage 8, reset timer
//       }
//       break;
//     // Stage 8 adalah STAGE TUNGGU
//     case 8:
//       if (millis() - lastActionTime > 1000) { // Cek 1 detik
//         return true; // Selesai!
//       }
//       // Jangan reset timer di sini, biarkan dia menghitung
//       break;
//   }
//   return false;
// }

// // DEMO 5: Garis Cepat (FIXED)
// bool runFastLinesDemo() {
//   static int i, stage;
//   static unsigned long lastActionTime;

//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     i = 0; stage = 0;
//     lastActionTime = millis();
//     demoInitialized = true;
//   }

//   switch (stage) {
//     case 0: // Vertikal (Luar-Dalam)
//       if (millis() - lastActionTime > 5) {
//         tft.drawFastVLine(i, 0, h, TFT_RED);
//         tft.drawFastVLine(w-1-i, 0, h, TFT_RED);
//         i+=3; lastActionTime = millis();
//         if (i >= w/2) { i = w/2; stage++; }
//       }
//       break;
//     case 1: // Vertikal (Dalam-Luar, menghapus)
//       if (millis() - lastActionTime > 5) {
//         tft.drawFastVLine(i, 0, h, TFT_BLACK);
//         tft.drawFastVLine(w-1-i, 0, h, TFT_BLACK);
//         i-=3; lastActionTime = millis();
//         if (i <= 0) { i = 0; stage++; }
//       }
//       break;
//     case 2: // Horizontal (Luar-Dalam)
//       if (millis() - lastActionTime > 5) {
//         tft.drawFastHLine(0, i, w, TFT_BLUE);
//         tft.drawFastHLine(0, h-1-i, w, TFT_BLUE);
//         i+=3; lastActionTime = millis();
//         if (i >= h/2) { i = h/2; stage++; }
//       }
//       break;
//     case 3: // Horizontal (Dalam-Luar, menghapus)
//       if (millis() - lastActionTime > 5) {
//         tft.drawFastHLine(0, i, w, TFT_BLACK);
//         tft.drawFastHLine(0, h-1-i, w, TFT_BLACK);
//         i-=3; lastActionTime = millis();
//         if (i <= 0) { i = 0; stage++; lastActionTime = millis(); }
//       }
//       break;
//     case 4: // Stage tunggu
//       if (millis() - lastActionTime > 500) { // Tahan 0.5 detik
//         return true;
//       }
//       break;
//   }
//   return false;
// }

// // DEMO 6: Kotak (FIXED)
// bool runRectsDemo() {
//   static int x, stage;
//   static unsigned long lastActionTime;
  
//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     x = w - 1; stage = 0;
//     lastActionTime = millis();
//     demoInitialized = true;
//   }

//   switch (stage) {
//     case 0: // testdrawrects (Luar-Dalam)
//       if (millis() - lastActionTime > 20) {
//         tft.drawRect(w/2 -x/2, h/2 -x/2 , x, x, TFT_GREEN);
//         x -= 6; lastActionTime = millis();
//         if (x <= 0) { x = w-1; stage++; tft.fillScreen(TFT_BLACK); }
//       }
//       break;
//     case 1: // testfillrects (Luar-Dalam)
//       if (millis() - lastActionTime > 20) {
//         tft.fillRect(w/2 -x/2, h/2 -x/2 , x, x, TFT_YELLOW);
//         tft.drawRect(w/2 -x/2, h/2 -x/2 , x, x, TFT_MAGENTA);
//         x -= 6; lastActionTime = millis();
//         if (x <= 6) { x=6; stage++; lastActionTime = millis(); } // Pindah ke stage 2, reset timer
//       }
//       break;
//     case 2: // Tahan 1 detik
//       if (millis() - lastActionTime > 1000) {
//         return true;
//       }
//       break;
//   }
//   return false;
// }

// // DEMO 7: Lingkaran (FIXED)
// bool runCirclesDemo() {
//   static int r, stage;
//   static unsigned long lastActionTime;

//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     r = 0; stage = 0;
//     lastActionTime = millis();
//     demoInitialized = true;
//   }
  
//   switch (stage) {
//     case 0: // fillCircle (Dalam-Luar)
//       if (millis() - lastActionTime > 5) {
//         tft.fillCircle(w/2, h/2, r, TFT_BLUE);
//         r += 2; lastActionTime = millis();
//         if (r > h/2) { r=h/2; stage++; }
//       }
//       break;
//     case 1: // fillCircle (Luar-Dalam, menghapus)
//       if (millis() - lastActionTime > 5) {
//         tft.fillCircle(w/2, h/2, r, TFT_BLACK);
//         r -= 2; lastActionTime = millis();
//         if (r <= 0) { r=0; stage++; }
//       }
//       break;
//     case 2: // drawCircle (Dalam-Luar)
//       if (millis() - lastActionTime > 5) {
//         tft.drawCircle(w/2, h/2, r, TFT_WHITE);
//         r += 2; lastActionTime = millis();
//         if (r > h/2) { r=h/2; stage++; }
//       }
//       break;
//     case 3: // drawCircle (Luar-Dalam, menghapus)
//       if (millis() - lastActionTime > 5) {
//         tft.drawCircle(w/2, h/2, r, TFT_BLACK);
//         r -= 2; lastActionTime = millis();
//         if (r <= 0) { r=0; stage++; lastActionTime = millis(); }
//       }
//       break;
//     case 4: // Tahan
//       if (millis() - lastActionTime > 500) {
//         return true;
//       }
//       break;
//   }
//   return false;
// }

// // DEMO 8: Kotak Tumpul (FIXED)
// bool runRoundrectsDemo() {
//   static int i, stage, color;
//   static unsigned long lastActionTime;

//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     i = 0; stage = 0; color = 100;
//     lastActionTime = millis();
//     demoInitialized = true;
//   }
  
//   switch (stage) {
//     case 0: // Dalam-Luar
//       if (millis() - lastActionTime > 25) {
//         int x = 0 + (i*2), y = 0 + (i*3);
//         int rect_w = w - 2 - (i*4), rect_h = h - 2 - (i*6);
//         tft.drawRoundRect(x, y, rect_w, rect_h, 5, color);
//         color += 1100; i++; lastActionTime = millis();
//         if (i > 16) { i = 16; stage++; }
//       }
//       break;
//     case 1: // Luar-Dalam (Menghapus)
//       if (millis() - lastActionTime > 25) {
//         int x = 0 + (i*2), y = 0 + (i*3);
//         int rect_w = w - 2 - (i*4), rect_h = h - 2 - (i*6);
//         tft.drawRoundRect(x, y, rect_w, rect_h, 5, TFT_BLACK);
//         i--; lastActionTime = millis();
//         if (i < 0) { i = 0; stage++; }
//       }
//       break;
//     case 2: // Diam (Gambar ulang & set timer)
//       tft.drawRoundRect(0, 0, w-2, h-2, 5, 100);
//       stage++;
//       lastActionTime = millis();
//       break;
//     case 3: // Tahan 1.5 detik
//       if (millis() - lastActionTime > 1500) {
//         return true;
//       }
//       break;
//   }
//   return false;
// }

// // DEMO 9: Segitiga (FIXED)
// bool runTrianglesDemo() {
//   static int t, color;
//   static int x, y, z, w_mid;
//   static unsigned long lastActionTime;

//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     t = 0; color = TFT_RED; w_mid = w/2;
//     x = h-1; y = 0; z = w;
//     lastActionTime = millis();
//     demoInitialized = true;
//   }

//   switch (t) {
//     case 0 ... 15: // Stage aksi (0 sampai 15)
//       if (millis() - lastActionTime > 40) {
//         tft.drawTriangle(w_mid, y, y, x, z, x, color);
//         x-=4; y+=4; z-=4;
//         color+=100;
//         t++;
//         lastActionTime = millis();
//         // if (t > 15) { stage = 16; } // Pindah ke stage tunggu
//       }
//       break;
//     default: // Stage tunggu (t == 16)
//       if (millis() - lastActionTime > 1000) {
//         return true;
//       }
//       break;
//   }
//   return false;
// }

// // DEMO 10: Invert (Logika v3 sudah benar)
// bool runInvertDemo() {
//   static int invertCount = 0;
//   static bool inverted = false;
//   static unsigned long lastInvertTime = 0;

//   if (!demoInitialized) {
//     tft.fillScreen(TFT_BLACK);
//     tft.setCursor(40, 100);
//     tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
//     tft.println("Demo Invert");
    
//     invertCount = 0; inverted = false;
//     lastInvertTime = millis();
//     demoInitialized = true;
//     delay(1000);
//   }

//   if (millis() - lastInvertTime > 500) {
//     inverted = !inverted;
//     tft.invertDisplay(inverted);
//     lastInvertTime = millis();
//     invertCount++;
//   }

//   if (invertCount >= 10) {
//     tft.invertDisplay(false);
//     return true;
//   }
//   return false;
// }