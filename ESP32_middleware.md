# ESP32/ESP8266 Integration Guide - IoT Backend

Dokumentasi integrasi untuk tim hardware dalam menggunakan IoT Backend API.

## 📋 Table of Contents

1. [System Architecture](#system-architecture)
2. [API Endpoints](#api-endpoints)
3. [ESP32/ESP8266 Code Examples](#code-examples)
4. [Best Practices](#best-practices)
5. [Error Handling](#error-handling)
6. [Do's and Don'ts](#dos-and-donts)
7. [Troubleshooting](#troubleshooting)

---

## 🏗️ System Architecture

### Data Flow

```
ESP32/ESP8266 → WiFi → Internet → Vercel (API) → Rate Limiter → Database
                                      ↓
                                  Response (201/429/500)
```

### Components

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Edge Device** | ESP32/ESP8266 | Sensor reading & HTTP client |
| **API Gateway** | Vercel Serverless | HTTP endpoint & validation |
| **Rate Limiter** | Upstash Redis | Prevent spam (10 req/10s per device) |
| **Database** | Neon PostgreSQL | Data storage |

### Key Characteristics

- **Synchronous**: Data langsung disimpan ke database
- **Response Time**: ~100-300ms
- **Rate Limited**: 10 requests per 10 detik per `deviceId`
- **No Retry Automatic**: Device harus handle retry sendiri

---

## 🔌 API Endpoints

### Base URL

```
Production: {baseUrl}
```

### 1. Health Check

**Endpoint:** `GET /health`

**Purpose:** Verify API availability

**Response (200 OK):**
```json
{
  "status": "ok",
  "timestamp": "2025-12-02T10:00:00.000Z"
}
```

**Usage:** Test sebelum mulai kirim data sensor

### 2. Ingest Sensor Data

**Endpoint:** `POST /ingest`

**Headers:**
```
Content-Type: application/json
```

**Request Body:**
```json
{
  "deviceId": "string (REQUIRED, 1-255 chars)",
  "currentUsed": "number (optional)",
  "currentRegen": "number (optional)",
  "power": "number (optional)",
  "voltage": "number (optional)",
  "whUsed": "number (optional)",
  "whRegen": "number (optional)",
  "rtcTime": "number (optional, Unix timestamp)"
}
```

**Response (201 Created):**
```json
{
  "status": "saved",
  "id": 123,
  "remaining": 9
}
```

**Error Responses:**

| Status | Error | Cause | Action |
|--------|-------|-------|--------|
| `400` | Invalid payload | JSON tidak valid / field kosong | Fix payload |
| `429` | Too Many Requests | Lebih dari 10 req/10s | Tunggu 10 detik |
| `500` | Internal Server Error | Server/database issue | Retry dengan backoff |

---

## 💻 Code Examples

### ESP32 - Arduino IDE

#### Basic Setup

```cpp
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// API Configuration - simpan `baseUrl` di Preference/EEPROM tanpa hardcode
const char* apiUrl = "{baseUrl}/ingest";
const char* deviceId = "esp32-device-001"; // UNIQUE per device!

// Sensor data
float voltage = 0.0;
float current = 0.0;
float power = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
}

void loop() {
  // Read sensors (contoh dummy)
  voltage = readVoltage();
  current = readCurrent();
  power = voltage * current;
  
  // Send to backend
  if (sendData(deviceId, voltage, current, power)) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Failed to send data");
  }
  
  // Wait 1 second (max 10 requests per 10 seconds)
  delay(1000);
}

bool sendData(const char* device, float v, float i, float p) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return false;
  }
  
  HTTPClient http;
  http.begin(apiUrl);
  http.addHeader("Content-Type", "application/json");
  
  // Create JSON payload
  StaticJsonDocument<256> doc;
  doc["deviceId"] = device;
  doc["voltage"] = v;
  doc["currentUsed"] = i;
  doc["power"] = p;
  doc["rtcTime"] = millis() / 1000.0; // Optional: RTC time
  
  String payload;
  serializeJson(doc, payload);
  
  // Send POST request
  int httpCode = http.POST(payload);
  
  // Handle response
  if (httpCode > 0) {
    String response = http.getString();
    Serial.printf("HTTP %d: %s\n", httpCode, response.c_str());
    
    if (httpCode == 201) {
      // Success!
      return true;
    } else if (httpCode == 429) {
      // Rate limited - tunggu
      Serial.println("Rate limited! Waiting 10s...");
      delay(10000);
      return false;
    }
  } else {
    Serial.printf("HTTP Error: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
  return false;
}

// Dummy sensor functions
float readVoltage() { return 220.0 + random(-5, 5); }
float readCurrent() { return 2.5 + random(-10, 10) / 100.0; }
```

### ESP8266 - NodeMCU

```cpp
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASSWORD";
const char* apiUrl = baseUrl+"/ingest";
const char* deviceId = "nodemcu-001";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
    
    http.begin(client, apiUrl);
    http.addHeader("Content-Type", "application/json");
    
    StaticJsonDocument<256> doc;
    doc["deviceId"] = deviceId;
    doc["power"] = readPower();
    doc["voltage"] = readVoltage();
    
    String payload;
    serializeJson(doc, payload);
    
    int httpCode = http.POST(payload);
    Serial.printf("HTTP %d\n", httpCode);
    
    http.end();
  }
  
  delay(1000); // 1 request per second (safe for rate limit)
}

float readPower() { return 100.0; }
float readVoltage() { return 220.0; }
```

### Advanced: Retry with Exponential Backoff

```cpp
bool sendDataWithRetry(const char* device, float v, float i, float p) {
  int maxRetries = 3;
  int retryDelay = 1000; // Start with 1 second
  
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    HTTPClient http;
    http.begin(apiUrl);
    http.addHeader("Content-Type", "application/json");
    
    StaticJsonDocument<256> doc;
    doc["deviceId"] = device;
    doc["voltage"] = v;
    doc["currentUsed"] = i;
    doc["power"] = p;
    
    String payload;
    serializeJson(doc, payload);
    
    int httpCode = http.POST(payload);
    
    if (httpCode == 201) {
      // Success!
      http.end();
      return true;
    } else if (httpCode == 429) {
      // Rate limited - tunggu 10 detik
      Serial.println("Rate limited, waiting 10s");
      http.end();
      delay(10000);
      return false; // Don't retry immediately
    } else if (httpCode >= 500) {
      // Server error - retry with backoff
      Serial.printf("Server error %d, retry %d/%d\n", httpCode, attempt+1, maxRetries);
      http.end();
      delay(retryDelay);
      retryDelay *= 2; // Exponential backoff
    } else {
      // Client error (400) - don't retry
      Serial.printf("Client error %d - check payload\n", httpCode);
      http.end();
      return false;
    }
  }
  
  return false; // Failed after retries
}
```

---

## ✅ Best Practices

### 1. Device ID Management

```cpp
// ❌ JANGAN: Hardcode sama untuk semua device
const char* deviceId = "esp32";

// ✅ DO: Gunakan unique identifier
String deviceId = "esp32-" + WiFi.macAddress();
// Result: "esp32-AA:BB:CC:DD:EE:FF"
```

### 2. WiFi Connection Handling

```cpp
void ensureWiFiConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nReconnected!");
    } else {
      Serial.println("\nReconnection failed!");
    }
  }
}
```

### 3. Rate Limiting Compliance

```cpp
// ✅ Recommended: 1 request per second (safe margin)
const unsigned long SEND_INTERVAL = 1000; // ms
unsigned long lastSendTime = 0;

void loop() {
  unsigned long now = millis();
  
  if (now - lastSendTime >= SEND_INTERVAL) {
    sendData(...);
    lastSendTime = now;
  }
}
```

### 4. Memory Management

```cpp
// ✅ Gunakan StaticJsonDocument untuk predictable memory
StaticJsonDocument<256> doc; // Stack allocation, safe

// ❌ Hindari DynamicJsonDocument jika payload size fixed
// DynamicJsonDocument doc(256); // Heap allocation, bisa fragment
```

### 5. HTTPS Certificate (Optional tapi Recommended)

```cpp
#include <WiFiClientSecure.h>

WiFiClientSecure client;
client.setInsecure(); // Skip verification (development only)

// Production: Load root CA
// client.setCACert(ROOT_CA);
```

---

## 🚨 Error Handling

### HTTP Status Code Handling

```cpp
void handleResponse(int httpCode, String response) {
  switch (httpCode) {
    case 201:
      Serial.println("✓ Data saved");
      // Parse response untuk get ID
      break;
      
    case 400:
      Serial.println("✗ Invalid data:");
      Serial.println(response);
      // Fix payload format
      break;
      
    case 429:
      Serial.println("✗ Rate limited");
      // Tunggu 10 detik sebelum kirim lagi
      delay(10000);
      break;
      
    case 500:
    case 502:
    case 503:
      Serial.println("✗ Server error, will retry");
      // Implement retry with backoff
      break;
      
    default:
      Serial.printf("✗ Unknown status: %d\n", httpCode);
      Serial.println(response);
  }
}
```

### Network Error Handling

```cpp
if (httpCode == -1) {
  // Connection failed
  Serial.println("Connection error");
  ensureWiFiConnected();
} else if (httpCode == -11) {
  // Timeout
  Serial.println("Request timeout");
  // Increase timeout or check network
}
```

---

## 📋 Do's and Don'ts

### ✅ DO's

1. **Gunakan unique `deviceId` untuk setiap device**
   - Contoh: MAC address, serial number, custom ID
   
2. **Respect rate limit: max 10 requests per 10 detik**
   - Recommended: 1 request per detik (safe margin)
   
3. **Implement retry logic untuk error 5xx**
   - Gunakan exponential backoff
   
4. **Validate data sebelum kirim**
   - Check sensor readings valid (tidak NaN/Inf)
   
5. **Handle WiFi disconnection gracefully**
   - Auto-reconnect dengan timeout
   
6. **Log semua HTTP response untuk debugging**
   - Include timestamp dan status code
   
7. **Test dengan health check sebelum production**
   ```cpp
   GET /health → should return 200
   ```
   
8. **Gunakan JSON library yang reliable**
   - ArduinoJson recommended (well-tested)

### ❌ DON'Ts

1. **JANGAN kirim data lebih dari 10x per 10 detik**
   - Akan kena rate limit (429)
   
2. **JANGAN retry immediately setelah 429**
   - Tunggu minimal 10 detik
   
3. **JANGAN hardcode `deviceId` yang sama di semua device**
   - Akan kacau tracking per device
   
4. **JANGAN kirim field `deviceId` kosong**
   - Akan dapat 400 Bad Request
   
5. **JANGAN abaikan HTTP status code**
   - Always check dan handle properly
   
6. **JANGAN kirim data saat WiFi disconnected**
   - Will cause timeout dan waste battery
   
7. **JANGAN simpan credentials di code**
   ```cpp
   // ❌ DON'T
   const char* ssid = "MyWiFi";
   const char* password = "password123";
   
   // ✅ DO: Use separate config file atau EEPROM
   ```
   
8. **JANGAN retry infinitely pada client error (4xx)**
   - 400/401/403/404 = fix code, bukan retry

---

## 🔧 Troubleshooting

### Problem: "WiFi not connected"

**Cause:** WiFi credentials salah atau signal lemah

**Solution:**
```cpp
Serial.println(WiFi.status());
// 0 = WL_IDLE_STATUS
// 3 = WL_CONNECTED
// 4 = WL_CONNECT_FAILED
// 6 = WL_DISCONNECTED
```

### Problem: "HTTP Error: -1"

**Cause:** DNS resolution failed atau server unreachable

**Solution:**
- Test dengan `ping your-app.vercel.app`
- Check internet connection
- Verify API URL correct

### Problem: "Rate limited! (429)"

**Cause:** Lebih dari 10 requests dalam 10 detik

**Solution:**
```cpp
// Increase delay between requests
delay(1500); // 1.5 seconds = max ~6 req/10s (safe)
```

### Problem: "Invalid payload (400)"

**Cause:** JSON format salah atau `deviceId` kosong

**Solution:**
```cpp
// Debug: Print JSON sebelum kirim
String payload;
serializeJson(doc, payload);
Serial.println("Sending: " + payload);
```

### Problem: "Server error (500)"

**Cause:** Backend issue (database down, etc)

**Solution:**
- Check Vercel deployment logs
- Verify environment variables set
- Implement retry with backoff

### Problem: "Timeout setelah beberapa detik"

**Cause:** HTTP timeout default terlalu pendek

**Solution:**
```cpp
http.setTimeout(10000); // 10 seconds timeout
```

---

## 📊 Example: Complete Production-Ready Code

```cpp
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Configuration
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";
const char* API_URL = "https://your-app.vercel.app/ingest";
String DEVICE_ID = ""; // Will be set from MAC

// Timing
const unsigned long SEND_INTERVAL = 1000; // 1 second
unsigned long lastSendTime = 0;

// Retry config
const int MAX_RETRIES = 3;
const int RETRY_BASE_DELAY = 1000;

void setup() {
  Serial.begin(115200);
  
  // Generate unique device ID
  DEVICE_ID = "esp32-" + WiFi.macAddress();
  Serial.println("Device ID: " + DEVICE_ID);
  
  // Connect WiFi
  connectWiFi();
  
  // Test health
  if (checkHealth()) {
    Serial.println("API is healthy!");
  }
}

void loop() {
  ensureWiFiConnected();
  
  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL) {
    // Read sensors
    float voltage = readVoltage();
    float current = readCurrent();
    float power = voltage * current;
    
    // Send data
    if (sendDataWithRetry(DEVICE_ID, voltage, current, power)) {
      Serial.println("✓ Data sent");
    } else {
      Serial.println("✗ Failed to send");
    }
    
    lastSendTime = now;
  }
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.println("IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection failed!");
  }
}

void ensureWiFiConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    connectWiFi();
  }
}

bool checkHealth() {
  HTTPClient http;
  http.begin(String(API_URL).replace("/ingest", "/health"));
  int code = http.GET();
  http.end();
  return (code == 200);
}

bool sendDataWithRetry(String deviceId, float v, float i, float p) {
  for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
    HTTPClient http;
    http.begin(API_URL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(10000);
    
    StaticJsonDocument<256> doc;
    doc["deviceId"] = deviceId;
    doc["voltage"] = v;
    doc["currentUsed"] = i;
    doc["power"] = p;
    doc["rtcTime"] = millis() / 1000.0;
    
    String payload;
    serializeJson(doc, payload);
    
    int code = http.POST(payload);
    String response = http.getString();
    
    http.end();
    
    if (code == 201) {
      return true;
    } else if (code == 429) {
      Serial.println("Rate limited, waiting 10s");
      delay(10000);
      return false;
    } else if (code >= 500) {
      int backoff = RETRY_BASE_DELAY * pow(2, attempt);
      Serial.printf("Server error, retry in %dms\n", backoff);
      delay(backoff);
    } else {
      Serial.printf("Error %d: %s\n", code, response.c_str());
      return false;
    }
  }
  
  return false;
}

float readVoltage() { return 220.0; }
float readCurrent() { return 2.5; }
```

---

## 📞 Support

**Issues or Questions:**
- Backend API: Check Vercel deployment logs
- Hardware: Review this documentation
- Integration: Test with `/health` endpoint first

**Testing Checklist:**
- [ ] WiFi connection stable
- [ ] Health check returns 200
- [ ] First POST returns 201
- [ ] Rate limit triggers after 11 requests
- [ ] Data appears in database

---

**Document Version:** 1.0  
**Last Updated:** 2025-12-02  
**Maintained by:** Backend Team