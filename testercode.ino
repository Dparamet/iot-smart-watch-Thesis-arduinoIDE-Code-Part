#include <Wire.h>
#include <Preferences.h>
#include "max32664.h"
#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

/* --important infomation --
  max32664D buy from this Provider : https://protocentral.com/product/pulse-express-pulse-ox-heart-rate-sensor-with-max32664/#downloads
  Library from  Provider : https://github.com/Protocentral/protocentral-pulse-express
*/

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

// -------- TIME / BATTERY --------
#define TZ_OFFSET_SEC (7 * 3600)  // ไทย GMT+7
#define NTP_SERVER    "pool.ntp.org"
// XIAO ESP32-C3 ไม่มีวงจรวัดแบตในตัว และ A0-A3 (GPIO2-5) ถูกใช้หมดแล้ว
// ถ้าต่อ voltage divider (แบต -> 220k/220k -> GND) เข้า ADC pin ค่อยแก้เป็นเบอร์ pin
#define BAT_PIN       -1           // -1 = ไม่มีสายวัดแบต จอโชว์ --%

// -------- COLOR --------
#define C_BLACK 0x0000
#define C_RED   0xF800
#define C_CYAN  0x07FF
#define C_WHITE 0xFFFF
#define C_GRAY  0x8410
#define C_GREEN 0x07E0
#define C_WARN  0xFFE0

// -------- SCAN TUNING --------
#define SCAN_TARGET_SAMPLES 100    // valid sample ครบเท่านี้ = 100% (progress ผูกกับ detect จริง)
#define SCAN_MAX_MS       30000UL  // เพดานเวลา ถ้าสัญญาณหลุดบ่อยจนไม่ครบใน 30 วิ = fail
#define FINGER_LOST_MS    1500UL   // ยกนิ้วต่อเนื่องเกินนี้ = ยกเลิกสแกน กลับหน้าหลัก

// -------- OBJECT --------
Max32664 hub(HUB_RESET, HUB_MFIO);
Preferences prefs;

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED
);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST);

// -------- STATE --------
enum State {
  CLOCK_MODE,   // หน้าหลัก (หน้าเดียว: เวลา+แบต+vitals ล่าสุด) รอวางนิ้ว
  DETECTING     // นิ้ววางอยู่ กำลังสแกน
};

State state = CLOCK_MODE;
State lastState = DETECTING; // ตั้งให้ต่างกันไว้ เพื่อบังคับเคลียร์จอรอบแรก
bool needRelease = false;    // วัดเสร็จต้องยกนิ้วก่อน ถึงจะเริ่มสแกนใหม่ได้

// home render state: วาดหน้าทีละช่อง กันจอกระพริบ (ไม่ fillScreen ทั้งจอทุกวิ)
bool homeFullDraw = true;   // true = ต้องวาดหน้าใหม่ทั้งหน้า (ตอนเข้าหน้า/มีค่าใหม่)
int  lastBatShown = -2;
int  lastMinShown = -1;

// -------- VAR --------
unsigned long lastUpdate = 0;
unsigned long detectStart = 0;
unsigned long fingerLostAt = 0;   // เวลาที่นิ้วหลุดล่าสุด (0 = นิ้วยังอยู่)

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

// hub หยุด stream หลังจบรอบวัด ต้อง stop + start ใหม่ทุกครั้งก่อนกลับหน้าหลัก
// ไม่งั้น readSample fail ตลอด = สแกนได้แค่รอบเดียว
void restartEstimation() {
  hub.stop();
  delay(100);
  beginEstimation();
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
  if (wifiOk) configTime(TZ_OFFSET_SEC, 0, NTP_SERVER); // เวลาจริงจาก NTP sync เองเบื้องหลัง
}

// -------- BATTERY --------
int readBatteryPct() {
  if (BAT_PIN < 0) return -1;
  uint32_t mv = analogReadMilliVolts(BAT_PIN) * 2; // divider หาร 2
  int pct = (int)((mv - 3300) * 100 / (4200 - 3300)); // LiPo 3.3V=0% 4.2V=100%
  return constrain(pct, 0, 100);
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
// หน้าเดียวจบ: แบต + เวลาจริง + HR/SpO2/BP (ตาม mind board)
// วาดทีละช่อง ลบเฉพาะกล่องที่เปลี่ยน -> จอนิ่ง ไม่กระพริบ ไม่มีเงาของเก่า
void drawHome() {
  char buf[24];

  // ---- วาดทั้งหน้าครั้งเดียว: กรอบไอคอนแบต + vitals + prompt ----
  if (homeFullDraw) {
    homeFullDraw = false;
    lastBatShown = -2;
    lastMinShown = -1;
    gfx->fillScreen(C_BLACK);

    // กรอบไอคอนแบต (คงที่)
    gfx->drawRect(150, 20, 26, 14, C_WHITE);
    gfx->fillRect(176, 24, 3, 6, C_WHITE);

    // vitals (เปลี่ยนเฉพาะหลังวัดเสร็จ -> วาดตรงนี้พอ)
    if (finalHr > 0) sprintf(buf, "HR   : %d", (int)finalHr);
    else             strcpy(buf, "HR   : --");
    drawCenter(buf, 120, 2, C_RED);

    if (finalSpo2 > 0) sprintf(buf, "SpO2 : %d%%", (int)finalSpo2);
    else               strcpy(buf, "SpO2 : --");
    drawCenter(buf, 150, 2, C_CYAN);

    if (finalSys > 0 && finalDia > 0) sprintf(buf, "BP   : %d/%d", (int)finalSys, (int)finalDia);
    else                              strcpy(buf, "BP   : --/--");
    drawCenter(buf, 180, 2, C_GREEN);

    drawCenter("Place finger to scan", 208, 1, C_GRAY);
  } else if (millis() - lastUpdate < 1000) {
    return; // อัปเดตช่องที่เปลี่ยนวินาทีละครั้งพอ
  }
  lastUpdate = millis();

  // ---- แบต: อัปเดตเฉพาะตอน % เปลี่ยน ----
  int bat = readBatteryPct();
  if (bat != lastBatShown) {
    lastBatShown = bat;
    if (bat >= 0) sprintf(buf, "%d%%", bat);
    else          strcpy(buf, "--%");
    gfx->fillRect(80, 20, 65, 16, C_BLACK);          // ลบเลข % เก่า
    gfx->setTextSize(2);
    gfx->setTextColor(C_WHITE);
    gfx->setCursor(145 - (int)strlen(buf) * 12, 20); // ชิดขวาติดไอคอน
    gfx->print(buf);
    int fw = (bat >= 0 ? bat : 100) * 22 / 100;      // -1 = โชว์เต็มสีเทา
    uint16_t bc = (bat < 0) ? C_GRAY : (bat > 30 ? C_GREEN : C_RED);
    gfx->fillRect(152, 22, 22, 10, C_BLACK);         // ลบไส้เก่า
    gfx->fillRect(152, 22, fw, 10, bc);
  }

  // ---- เวลา: อัปเดตเฉพาะตอนเปลี่ยน ----
  struct tm t;
  int key;
  if (wifiOk && getLocalTime(&t, 0)) {
    sprintf(buf, "%02d:%02d", t.tm_hour, t.tm_min);
    key = t.tm_hour * 60 + t.tm_min;
  } else {
    unsigned long s = millis() / 1000;               // offline: นับจากเปิดเครื่อง
    sprintf(buf, "%02lu:%02lu", (s / 60) % 60, s % 60);
    key = (int)s;
  }
  if (key != lastMinShown) {
    lastMinShown = key;
    gfx->fillRect(0, 60, 240, 40, C_BLACK);          // ลบเวลาเก่า (เฉพาะแถบนี้)
    drawCenter(buf, 60, 5, C_WHITE);
  }
}

bool lastSignalDrawn = true;

void showDetecting(float hr, float spo2, bool haveSignal) {
  // progress = จำนวน valid sample จริง ไม่ใช่เวลา -> สัญญาณหลุด progress จะหยุดรอ ไม่วิ่งมั่ว
  int pct = nSamples * 100 / SCAN_TARGET_SAMPLES;
  if (pct > 100) pct = 100;
  // วาดเฉพาะตอน % หรือสถานะสัญญาณเปลี่ยน กันจอกะพริบ
  if (pct == lastPctDrawn && haveSignal == lastSignalDrawn) return;
  lastPctDrawn = pct;
  lastSignalDrawn = haveSignal;

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
    drawCenter("Weak signal, hold on...", 172, 1, C_WARN);
  }
  drawCenter("Keep finger still", 195, 1, C_GRAY);
}

void showScanFail() {
  gfx->fillScreen(C_BLACK);
  drawCenter("SCAN FAILED", 95, 2, C_RED);
  drawCenter("Weak signal, try again", 130, 1, C_WARN);
}

void showNotFound() {
  gfx->fillScreen(C_BLACK);
  drawCenter("NOT FOUND", 95, 2, C_RED);
  drawCenter("Please try again", 130, 1, C_WARN);
}

// -------- SCAN HELPERS --------
void resetScan() {
  sumHr = sumSpo2 = sumSys = sumDia = 0;
  nSamples = nBpSamples = 0;
  lastPctDrawn = -1;
  lastSignalDrawn = true;
  fingerLostAt = 0;
}

// จบ/ยกเลิกสแกน -> โชว์ข้อความ + restart sensor + กลับหน้าหลัก
void abortScan(void (*screen)()) {
  screen();
  restartEstimation(); // ทำระหว่างโชว์ข้อความ ไม่เสียเวลาเพิ่ม
  delay(2000);
  needRelease = true;  // ต้องยกนิ้วก่อนถึงเริ่มรอบใหม่ กันสแกนวนเองตอนสัญญาณแย่
  lastUpdate = 0;
  state = CLOCK_MODE;
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
                  state == CLOCK_MODE ? "HOME" : "SCAN",
                  hr, spo2, sys, dia, fingerOn ? "ON" : "OFF", valid ? "Y" : "N");
  }

  // เปลี่ยนสเตท -> เข้า home ให้วาดทั้งหน้าใหม่, เข้า scan ให้ล้างจอ
  if (state != lastState) {
    lastState = state;
    if (state == CLOCK_MODE) homeFullDraw = true; // drawHome จะ fillScreen เอง
    else                     gfx->fillScreen(C_BLACK);
    lastUpdate = 0;
  }

  // -------- STATE MACHINE --------
  switch (state) {

    case CLOCK_MODE:
      drawHome();
      if (haveSample && !fingerOn) needRelease = false; // ยกนิ้วแล้ว พร้อมวัดรอบใหม่
      if (haveSample && fingerOn && !needRelease) {
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
        Serial.println("Finger removed -> NOT FOUND -> home");
        abortScan(showNotFound);
        break;
      }

      // สะสมค่าเฉลี่ยจากทุก sample ที่ valid
      // sample หลุดบ้าง (เช่น 7/10) ไม่เป็นไร progress หยุดรอแล้วไปต่อจนครบ 100%
      if (valid) {
        sumHr += hr; sumSpo2 += spo2; nSamples++;
        if (sys > 0 && dia > 0) { sumSys += sys; sumDia += dia; nBpSamples++; }
      }

      showDetecting(hr, spo2, valid);

      // เก็บ valid sample ครบเป้า = 100% -> สรุปผลทันที
      if (nSamples >= SCAN_TARGET_SAMPLES) {
        finalHr   = sumHr / nSamples;
        finalSpo2 = sumSpo2 / nSamples;
        finalSys  = nBpSamples ? sumSys / nBpSamples : 0;
        finalDia  = nBpSamples ? sumDia / nBpSamples : 0;

        Serial.printf("RESULT: HR %.0f | SpO2 %.0f | BP %.0f/%.0f (%d samples, %.1fs)\n",
                      finalHr, finalSpo2, finalSys, finalDia, nSamples,
                      (millis() - detectStart) / 1000.0);

        dataSent = sendVitals(finalHr, finalSpo2, finalSys, finalDia);
        restartEstimation();  // สำคัญ: ไม่ restart แล้ว hub จะไม่ส่ง sample อีก = จอค้างรอบสอง
        needRelease = true;   // กันวัดวนซ้ำทั้งที่นิ้วยังวางอยู่
        lastUpdate = 0;       // บังคับ home วาดค่าใหม่ทันที
        state = CLOCK_MODE;
      } else if (millis() - detectStart >= SCAN_MAX_MS) {
        // สัญญาณแย่จนเก็บไม่ครบในเวลาเพดาน = fail ไม่โชว์ค่ามั่ว
        Serial.printf("Scan failed: only %d/%d valid samples in %lus\n",
                      nSamples, SCAN_TARGET_SAMPLES, SCAN_MAX_MS / 1000);
        abortScan(showScanFail);
      }
      break;
    }
  }

  delay(20); // กัน FIFO ล้น ห้ามหน่วงมากกว่านี้
}
