#include <Wire.h>
#include <Preferences.h>
#include "max32664.h"
#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include <HTTPClient.h>

// -------- PIN (DO NOT MOVE) --------
#define HUB_RESET   3
#define HUB_MFIO    2

#define TFT_SCK     8
#define TFT_MOSI    10
#define TFT_CS      20
#define TFT_DC      21
#define TFT_RST     5
#define TFT_BL      4

// -------- WIFI / DASHBOARD --------
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASS     "YOUR_WIFI_PASSWORD"
#define API_URL       "http://YOUR_DASHBOARD_HOST/api/vitals"  // แก้เป็น endpoint ของ dashboard
#define DEVICE_NAME   "smartwatch-01"

// -------- COLOR --------
#define C_BLACK 0x0000
#define C_RED   0xF800
#define C_CYAN  0x07FF
#define C_WHITE 0xFFFF
#define C_GRAY  0x8410
#define C_GREEN 0x07E0
#define C_WARN  0xFFE0

// -------- SCAN TUNING --------
#define SCAN_TIME_MS      10000UL  // สแกนเต็ม 10 วิ ตามสเปค
#define SCAN_MIN_SAMPLES  20       // ต้องได้ค่า valid อย่างน้อยเท่านี้ถึงยอมรับผล
#define FINGER_LOST_MS    1500UL   // ยกนิ้วต่อเนื่องเกินนี้ = ยกเลิกสแกน กลับหน้าหลัก
#define SHOW_TIMEOUT_MS   8000UL   // โชว์ผลค้างไว้ก่อนกลับหน้านาฬิกา

// -------- OBJECT --------
Max32664 hub(HUB_RESET, HUB_MFIO);
Preferences prefs;

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED
);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST);

// -------- STATE --------
enum State {
  CLOCK_MODE,   // หน้าหลัก รอวางนิ้ว
  DETECTING,    // นิ้ววางอยู่ กำลังสแกน (สูงสุด 10 วิ)
  SHOW_DATA     // แสดงผล + ส่งขึ้น dashboard
};

State state = CLOCK_MODE;
State lastState = SHOW_DATA; // ตั้งให้ต่างกันไว้ เพื่อบังคับเคลียร์จอรอบแรก

// -------- VAR --------
unsigned long lastUpdate = 0;
unsigned long detectStart = 0;
unsigned long fingerLostAt = 0;   // เวลาที่นิ้วหลุดล่าสุด (0 = นิ้วยังอยู่)
unsigned long stateTimer = 0;
int secondsFake = 0;

float finalHr = 0, finalSpo2 = 0, finalSys = 0, finalDia = 0;

// ตัวสะสมค่าเฉลี่ยระหว่างสแกน
float sumHr = 0, sumSpo2 = 0, sumSys = 0, sumDia = 0;
int   nSamples = 0, nBpSamples = 0;
int   lastPctDrawn = -1;

static uint8_t calibVec[824];
static size_t  calibLen = 0;

bool wifiOk = false;
bool dataSent = false;

// -------- UI HELPER --------
void drawCenter(const char* txt, int y, int size, uint16_t color) {
  int len = strlen(txt);
  int w = len * size * 6;
  int x = (240 - w) / 2;

  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(txt);
}

// -------- CALIBRATION STORAGE --------
bool loadCalib() {
  prefs.begin("pulse", true);
  calibLen = prefs.getUInt("len", 0);
  if (calibLen && calibLen <= sizeof(calibVec))
    prefs.getBytes("vec", calibVec, calibLen);
  prefs.end();
  return calibLen > 0 && calibLen <= sizeof(calibVec);
}

void saveCalib() {
  prefs.begin("pulse", false);
  prefs.putUInt("len", calibLen);
  prefs.putBytes("vec", calibVec, calibLen);
  prefs.end();
}

// -------- FORCE SENSOR CALIBRATION --------
void runCalibration() {
  Serial.println("Calibrating Sensor...");
  gfx->fillScreen(C_BLACK);
  drawCenter("CALIBRATE", 70, 3, C_WARN);
  drawCenter("Hold finger 2 min", 130, 2, C_WHITE);

  Max32664LegacyCalibrationRefs refs;
  for (int i = 0; i < 3; i++) {
    refs.systolic[i]  = 120 + i * 2 + (i == 2 ? 1 : 0);
    refs.diastolic[i] = 80  + i;
  }

  if (hub.startCalibration(refs) != Max32664Status::Ok) {
    Serial.println("startCalibration FAIL");
    drawCenter("CALIB FAIL", 120, 2, C_RED);
    delay(3000);
    return;
  }

  Max32664Sample s;
  uint8_t lastPct = 255;
  char buf[24];
  for (unsigned long t0 = millis(); millis() - t0 < 120000UL; delay(40)) {
    if (hub.readSample(s) != Max32664Status::Ok) continue;
    if (s.progress != lastPct) {
      Serial.printf("Calib Progress: %3d%%\n", s.progress);
      gfx->fillScreen(C_BLACK);
      drawCenter("CALIBRATING", 70, 3, C_WARN);
      sprintf(buf, "Progress %d%%", s.progress);
      drawCenter(buf, 130, 2, C_WHITE);
      lastPct = s.progress;
    }
    if (s.bpStatus == Max32664BpStatus::Success && s.progress >= 100) break;
  }

  hub.readCalibrationVector(calibVec, sizeof(calibVec), &calibLen);
  saveCalib();
  hub.stop();
  Serial.printf("Calibration completed and saved (%d bytes)\n", calibLen);
}

void beginEstimation() {
  hub.loadCalibrationVector(calibVec, calibLen);
  hub.startEstimation(Max32664Spo2Coeffs{});
}

// -------- WIFI + JSON UPLOAD --------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  wifiOk = (WiFi.status() == WL_CONNECTED);
  Serial.printf("\nWiFi: %s\n", wifiOk ? WiFi.localIP().toString().c_str() : "FAILED (offline mode)");
}

// ส่งผลวัดขึ้น dashboard: {"device":"...","hr":72,"spo2":98,"bp":"120/80"}
bool sendVitals(float hr, float spo2, float sys, float dia) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Send skipped: no WiFi");
    return false;
  }

  char json[160];
  if (sys > 0 && dia > 0) {
    snprintf(json, sizeof(json),
      "{\"device\":\"%s\",\"hr\":%d,\"spo2\":%d,\"bp\":\"%d/%d\"}",
      DEVICE_NAME, (int)hr, (int)spo2, (int)sys, (int)dia);
  } else {
    // BP เป็น optional — ไม่มีค่าก็ไม่ส่ง field
    snprintf(json, sizeof(json),
      "{\"device\":\"%s\",\"hr\":%d,\"spo2\":%d}",
      DEVICE_NAME, (int)hr, (int)spo2);
  }

  HTTPClient http;
  http.begin(API_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000); // อย่าค้างนานเกิน จอจะ freeze
  int code = http.POST(json);
  http.end();

  Serial.printf("POST %s -> %d : %s\n", API_URL, code, json);
  return code >= 200 && code < 300;
}

// -------- MAIN APP UI --------
void drawClock() {
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    secondsFake++;

    gfx->fillScreen(C_BLACK);

    char buf[10];
    int min = (secondsFake / 60) % 60;
    int sec = secondsFake % 60;

    sprintf(buf, "%02d:%02d", min, sec);
    drawCenter(buf, 80, 4, C_WHITE);
    drawCenter("Smart Watch", 135, 2, C_GRAY);
    drawCenter("Place finger to scan", 170, 1, C_GREEN);
    drawCenter(wifiOk ? "WiFi OK" : "WiFi OFF", 195, 1, wifiOk ? C_CYAN : C_RED);
  }
}

void showDetecting(float hr, float spo2, bool haveSignal) {
  int pct = (int)((millis() - detectStart) * 100UL / SCAN_TIME_MS);
  if (pct > 100) pct = 100;
  if (pct == lastPctDrawn) return; // วาดเฉพาะตอน % เปลี่ยน กันจอกะพริบ
  lastPctDrawn = pct;

  gfx->fillScreen(C_BLACK);
  drawCenter("SCANNING", 55, 3, C_GREEN);

  char buf[24];
  sprintf(buf, "%d%%", pct);
  drawCenter(buf, 100, 4, C_WHITE);

  // progress bar
  gfx->drawRect(45, 145, 150, 12, C_GRAY);
  gfx->fillRect(47, 147, (146 * pct) / 100, 8, C_GREEN);

  // ค่าดิบระหว่างสแกน (live preview)
  if (haveSignal) {
    sprintf(buf, "HR %d  SpO2 %d%%", (int)hr, (int)spo2);
    drawCenter(buf, 172, 1, C_CYAN);
  } else {
    drawCenter("Reading signal...", 172, 1, C_WARN);
  }
  drawCenter("Keep finger still", 195, 1, C_GRAY);
}

void showVitals(float hr, float spo2, float sys, float dia, bool sent) {
  char buf[20];

  drawCenter("RESULT", 25, 2, C_GRAY);

  sprintf(buf, "%d BPM", (int)hr);
  drawCenter(buf, 55, 3, C_RED);

  gfx->drawFastHLine(60, 90, 120, C_GRAY);

  sprintf(buf, "SpO2: %d%%", (int)spo2);
  drawCenter(buf, 105, 3, C_CYAN);

  gfx->drawFastHLine(60, 140, 120, C_GRAY);

  if (sys > 0 && dia > 0) {
    sprintf(buf, "BP %d/%d", (int)sys, (int)dia);
    drawCenter(buf, 155, 2, C_GREEN);
  } else {
    drawCenter("BP: --/--", 155, 2, C_GRAY);
  }

  drawCenter(sent ? "Sent to hospital" : "Send failed / offline",
             190, 1, sent ? C_GREEN : C_WARN);
}

void showScanFail() {
  gfx->fillScreen(C_BLACK);
  drawCenter("SCAN FAILED", 95, 2, C_RED);
  drawCenter("Weak signal, try again", 130, 1, C_WARN);
}

// -------- SCAN HELPERS --------
void resetScan() {
  sumHr = sumSpo2 = sumSys = sumDia = 0;
  nSamples = nBpSamples = 0;
  lastPctDrawn = -1;
  fingerLostAt = 0;
}

// -------- SETUP --------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  gfx->begin();
  gfx->fillScreen(C_BLACK);
  drawCenter("BOOTING...", 120, 2, C_GRAY);

  Wire.begin(6, 7);
  Wire.setClock(400000);

  if (hub.begin() != Max32664Status::Ok) {
    drawCenter("HUB ERROR", 150, 2, C_RED);
    while (1);
  }

  connectWiFi(); // ต่อเน็ตก่อน calibrate จะได้ไม่ไปหน่วงตอนวัด

  if (loadCalib()) {
    Serial.printf("Loaded calib vector (%d bytes)\n", calibLen);
  } else {
    runCalibration();
  }

  beginEstimation();
  gfx->fillScreen(C_BLACK);
}

// -------- LOOP --------
void loop() {
  Max32664Sample sample;
  bool haveSample = false;
  bool fingerOn = false;
  bool valid = false;
  float hr = 0, spo2 = 0, sys = 0, dia = 0;

  if (hub.readSample(sample) == Max32664Status::Ok) {
    haveSample = true;
    hr = sample.heartRate();
    spo2 = sample.spo2();
    sys = sample.systolic;
    dia = sample.diastolic;

    fingerOn = (sample.bpStatus != Max32664BpStatus::NoFinger);
    // ค่าที่เชื่อถือได้จริง: มีนิ้ว + ตัวเลขอยู่ในช่วงมนุษย์
    valid = (fingerOn && hr > 30.0 && hr < 220.0 && spo2 > 50.0 && spo2 <= 100.0);

    // debug ทาง Serial ทุก sample
    Serial.printf("[%s] HR: %.1f | SpO2: %.1f | BP: %.0f/%.0f | Finger: %s | Valid: %s\n",
                  state == CLOCK_MODE ? "HOME" : state == DETECTING ? "SCAN" : "SHOW",
                  hr, spo2, sys, dia, fingerOn ? "ON" : "OFF", valid ? "Y" : "N");
  }

  // เคลียร์จอครั้งเดียวเมื่อเปลี่ยนสเตท
  if (state != lastState) {
    gfx->fillScreen(C_BLACK);
    lastState = state;
    lastUpdate = 0; // บังคับ clock วาดใหม่ทันที
  }

  // -------- STATE MACHINE --------
  switch (state) {

    case CLOCK_MODE:
      drawClock();
      if (haveSample && fingerOn) {
        // เจอนิ้ว -> เริ่มสแกนทันที
        resetScan();
        detectStart = millis();
        state = DETECTING;
      }
      break;

    case DETECTING: {
      // debug: นิ้วหลุดต่อเนื่องเกินกำหนด -> ยกเลิก กลับหน้าหลัก
      if (haveSample) {
        if (fingerOn) {
          fingerLostAt = 0;
        } else if (fingerLostAt == 0) {
          fingerLostAt = millis();
        }
      }
      if (fingerLostAt && millis() - fingerLostAt > FINGER_LOST_MS) {
        Serial.println("Finger removed -> back to home");
        state = CLOCK_MODE;
        break;
      }

      // สะสมค่าเฉลี่ยจากทุก sample ที่ valid
      if (valid) {
        sumHr += hr; sumSpo2 += spo2; nSamples++;
        if (sys > 0 && dia > 0) { sumSys += sys; sumDia += dia; nBpSamples++; }
      }

      showDetecting(hr, spo2, valid);

      // ครบ 10 วิ -> สรุปผล
      if (millis() - detectStart >= SCAN_TIME_MS) {
        if (nSamples >= SCAN_MIN_SAMPLES) {
          finalHr   = sumHr / nSamples;
          finalSpo2 = sumSpo2 / nSamples;
          finalSys  = nBpSamples ? sumSys / nBpSamples : 0;
          finalDia  = nBpSamples ? sumDia / nBpSamples : 0;

          Serial.printf("RESULT: HR %.0f | SpO2 %.0f | BP %.0f/%.0f (%d samples)\n",
                        finalHr, finalSpo2, finalSys, finalDia, nSamples);

          dataSent = sendVitals(finalHr, finalSpo2, finalSys, finalDia);
          gfx->fillScreen(C_BLACK);
          showVitals(finalHr, finalSpo2, finalSys, finalDia, dataSent);
          state = SHOW_DATA;
          stateTimer = millis();
        } else {
          // สัญญาณไม่พอ = วัดไม่สำเร็จ ไม่โชว์ค่ามั่ว
          Serial.printf("Scan failed: only %d valid samples\n", nSamples);
          showScanFail();
          delay(2000); // โชว์ข้อความ fail สั้นๆ พอให้คนอ่านทัน
          state = CLOCK_MODE;
        }
      }
      break;
    }

    case SHOW_DATA:
      // จอวาดไปแล้วตอนเข้าสเตท ไม่ต้องวาดซ้ำ
      if (haveSample && fingerOn) {
        stateTimer = millis(); // นิ้วยังวาง = ค้างหน้าผลไว้
      }
      if (millis() - stateTimer > SHOW_TIMEOUT_MS) {
        state = CLOCK_MODE;
      }
      break;
  }

  delay(20); // กัน FIFO ล้น ห้ามหน่วงมากกว่านี้
}
