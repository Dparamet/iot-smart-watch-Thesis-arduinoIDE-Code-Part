#include <Wire.h>
#include <Preferences.h>
#include "max32664.h"
#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include <WiFiManager.h>
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
// ไม่ hardcode SSID/password แล้ว — ใช้ WiFiManager ให้ผู้ใช้ตั้งค่าเองผ่านหน้าเว็บตอนบูตครั้งแรก (บันทึกลง flash เอง)
#define WIFI_MANAGER_AP_NAME         "SmartWatch-Setup"  // ชื่อ AP ตอนเปิดหน้าตั้งค่า WiFi
#define WIFI_MANAGER_AP_PASSWORD     "watch1234"          // รหัสผ่าน AP ตั้งค่า (ต้อง >=8 ตัว) กันคนแปลกหน้าเข้ามาแก้ WiFi/ดักข้อมูล เปลี่ยนได้ตามต้องการ
#define WIFI_CONFIG_PORTAL_TIMEOUT_S 180UL                // ปิด portal เองถ้าไม่มีใครตั้งค่าใน 3 นาที (กันเปิดค้างกินแบต)
#define BOOT_BTN_PIN                 9                    // ปุ่ม BOOT บนบอร์ด XIAO ESP32C3 (active LOW) กดค้างตอนเปิดเครื่อง = ลืม WiFi เดิม
#define API_URL       "http://172.24.155.69:8000/api/iot/vitals/"  // แก้เป็น endpoint ของ dashboard
#define API_KEY       "DufwFwDIRjwFa6LvdwA7PmF3pg4CgA6C"  // ส่งผ่าน header X-API-Key
#define DEVICE_NAME   "WT001"
#define TEMPERATURE_C 36.5f  // backend บังคับส่ง แต่เซนเซอร์นี้วัดอุณหภูมิไม่ได้ — ส่งค่าคงที่ไปก่อน ถ้าต่อ MLX90614 เมื่อไหร่ค่อยเปลี่ยนเป็นค่าวัดจริง

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

// -------- MPU6050 FALL DETECT --------
// ต่อสายเพิ่ม: VCC->3V3, GND->GND, SDA->GPIO6, SCL->GPIO7 (แชร์ I2C bus เดียวกับ MAX32664), AD0->GND
// หลักการ: ล้มจริง = ช่วงตกอิสระ (แรง g รวมต่ำผิดปกติ) ตามด้วยแรงกระแทก (g พุ่งสูง) ภายในเวลาสั้นๆ
// เดินปกติ/แกว่งแขนจะไม่ครบทั้งสองเงื่อนไขติดกัน เลยไม่ค่อย false alarm
uint8_t MPU_ADDR = 0x68;         // AD0 ต่อ GND = 0x68, ต่อ 3V3/ลอย = 0x69 — mpuBegin() ลองทั้งคู่เอง
#define FALL_FREEFALL_G 0.45f    // g รวมต่ำกว่านี้ = กำลังตกอิสระ
#define FALL_IMPACT_G   2.4f     // g รวมเกินนี้หลังตกอิสระ = กระแทกพื้น
#define FALL_WINDOW_MS  600UL    // กระแทกต้องมาภายในเวลานี้หลังเริ่มตก ไม่งั้นถือว่าไม่ใช่การล้ม
#define FALL_ALERT_MS   10000UL  // จอโชว์ FALL DETECTED ค้างนานเท่านี้ก่อนกลับหน้าหลัก

// -------- SCAN TUNING --------
// เซ็นเซอร์ให้ค่า valid ~2 ตัว/วินาที -> 20 ตัว = ~10 วิ (เพดาน 40 วิ กันสัญญาณหลุดบ่อย)
// อย่าตั้ง TARGET สูงกว่าที่เก็บได้ทันในเพดาน ไม่งั้นสแกนจะ FAIL ตลอด จอเลยไม่โชว์ค่า
#define SCAN_TARGET_SAMPLES 20     // valid sample ครบเท่านี้ = 100% (progress ผูกกับ detect จริง)
#define SPO2_MIN_SAMPLES    3      // HR ครบแล้วยังรอ SpO2 ให้ได้อย่างน้อยเท่านี้ก่อนจบรอบ (SpO2 มาช้ากว่า HR มาก)
#define SCAN_MAX_MS       40000UL  // เพดานเวลา ถ้าเก็บไม่ครบใน 40 วิ = fail (รวมเวลารอ SpO2 ด้วย)
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
bool needRelease = false;    // วัดเสร็จต้องยกนิ้วก่อน ถึงจะเริ่มสแกนใหม่ได้

// home render state: วาดหน้าทีละช่อง กันจอกระพริบ (ไม่ fillScreen ทั้งจอทุกวิ)
bool homeFullDraw = true;   // true = ต้องวาดหน้าใหม่ทั้งหน้า (ตอนเข้าหน้า/มีค่าใหม่)
int  lastBatShown = -2;
int  lastMinShown = -1;

// -------- VAR --------
unsigned long lastUpdate = 0;
unsigned long detectStart = 0;
unsigned long fingerLostAt = 0;   // เวลาที่นิ้วหลุดล่าสุด (0 = นิ้วยังอยู่)
unsigned long fingerOffSince = 0; // debounce ตอนรอ "ยกนิ้ว" กันหลุดหลอกหลัง restartEstimation()
#define RELEASE_DEBOUNCE_MS 400UL // ต้องไม่มีนิ้วต่อเนื่องเกินนี้ ถึงถือว่ายกนิ้วจริง
unsigned long resultShownAt = 0;  // เวลาที่เพิ่งวาดผลวัดล่าสุดบนหน้า home
int lastStatusKey = -999;         // สถานะแถบล่างจอที่วาดล่าสุด (กันวาดซ้ำ)
#define MIN_HOME_HOLD_MS 3000UL   // การันตีค่าที่วัดได้ค้างจออย่างน้อยเท่านี้ ไม่ว่าจะมีอะไรมาทำให้สแกนใหม่ก่อนก็ตาม

float finalHr = 0, finalSpo2 = 0, finalSys = 0, finalDia = 0, finalRR = 0;

// ตัวสะสมค่าเฉลี่ยระหว่างสแกน
// HR กับ SpO2 นับแยกกัน: sensor รายงานมาคนละจังหวะ (spo2ReportFlag แยกจาก HR)
// sample ที่ HR valid มักได้ SpO2=0 และกลับกัน ถ้าเช็ครวมจะได้ค่าแค่ฝั่งเดียวเสมอ
float sumHr = 0, sumSpo2 = 0, sumSys = 0, sumDia = 0;
int   nSamples = 0, nSpo2Samples = 0, nBpSamples = 0;

static uint8_t calibVec[824];
static size_t  calibLen = 0;

bool wifiOk = false;
int lastSendStatus = -1; // -1 ยังไม่เคยส่ง, 0 ส่ง fail, 1 ส่งสำเร็จ — โชว์เป็นจุดสีบนจอ

// fall detect state
bool mpuOk = false;               // เจอ MPU6050 ตอนบูตไหม (ไม่เจอ = ข้าม fall detect ระบบวัดทำงานปกติ)
unsigned long freefallAt = 0;     // เวลาที่เริ่มเจอช่วงตกอิสระ (0 = ยังไม่เจอ)
unsigned long fallAlertAt = 0;    // เวลาที่เจอการล้มล่าสุด (0 = ไม่มี alert ค้างจอ)

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
// ใช้ WiFiManager แทน hardcode SSID/password: ถ้าเคยตั้งค่าไว้แล้วจะต่อเองอัตโนมัติ
// ถ้ายังไม่เคยตั้ง (หรือกดปุ่ม BOOT ค้างตอนเปิดเครื่อง) จะเปิด AP ชื่อ WIFI_MANAGER_AP_NAME
// ให้เอามือถือ/คอมไปต่อ แล้วเข้า http://192.168.4.1 เพื่อเลือก WiFi + ใส่รหัสผ่าน (บันทึกลง flash เอง ไม่ต้องแก้โค้ดใหม่)
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // เน็ต IoT บางที่ตัดการเชื่อมต่อถ้าเข้า power-save
  Serial.printf("Device MAC: %s (เอาไปแจ้ง IT ลงทะเบียน MAC ถ้าเน็ตต้อง whitelist)\n", WiFi.macAddress().c_str());

  // สแกนแล้ว print SSID ที่มองเห็นจริงออก Serial — เอาไว้เช็คว่า AP เป้าหมายสัญญาณอ่อน/มองไม่เห็นจริงไหม
  int found = WiFi.scanNetworks();
  Serial.printf("WiFi scan: found %d network(s)\n", found);
  for (int i = 0; i < found; i++) {
    Serial.printf("  %2d) %-32s RSSI=%d dBm  ch=%d  %s\n",
                  i, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OPEN" : "SECURED");
  }

  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
  WiFiManager wm;
  wm.setConfigPortalTimeout(WIFI_CONFIG_PORTAL_TIMEOUT_S);
  wm.setConnectTimeout(20);   // เน็ต IoT บางที่ handshake ช้า ค่า default สั้นไปอาจ timeout ก่อนต่อสำเร็จ
  wm.setConnectRetries(3);    // ลองต่อซ้ำก่อนจะถือว่า fail จริง (กันสัญญาณอ่อน/หลุดชั่วขณะ)

  if (digitalRead(BOOT_BTN_PIN) == LOW) {
    Serial.println("BOOT held at boot -> ลืม WiFi เดิม เปิดหน้าตั้งค่าใหม่");
    wm.resetSettings();
  }

  gfx->fillScreen(C_BLACK);
  drawCenter("WIFI SETUP", 60, 2, C_WARN);
  drawCenter("Connect phone to:", 95, 1, C_WHITE);
  drawCenter(WIFI_MANAGER_AP_NAME, 115, 2, C_CYAN);
  drawCenter("AP password:", 145, 1, C_WHITE);
  drawCenter(WIFI_MANAGER_AP_PASSWORD, 160, 1, C_CYAN);
  drawCenter("then open 192.168.4.1", 180, 1, C_GRAY);
  drawCenter("(skip if already set up)", 195, 1, C_GRAY);

  wifiOk = wm.autoConnect(WIFI_MANAGER_AP_NAME, WIFI_MANAGER_AP_PASSWORD);

  if (wifiOk) {
    WiFi.setAutoReconnect(true); // เน็ตหลุดแล้วต่อเองเบื้องหลัง ไม่งั้นหลุดครั้งเดียว = ส่ง API ไม่ได้อีกเลยจนรีบูต
    WiFi.setSleep(false);        // set ซ้ำหลัง autoConnect เพราะ WiFiManager อาจ reset โหมดระหว่าง portal
    Serial.printf("WiFi: %s\n", WiFi.localIP().toString().c_str());
    configTime(TZ_OFFSET_SEC, 0, NTP_SERVER); // เวลาจริงจาก NTP sync เองเบื้องหลัง
  } else {
    // status code: 1=NO_SSID_AVAIL, 4=CONNECT_FAILED (มักเป็น MAC ยังไม่ลงทะเบียน/รหัสผิด), 6=WRONG_PASSWORD
    Serial.printf("WiFi FAILED (offline mode), status code=%d\n", WiFi.status());
  }
}

// -------- BATTERY --------
int readBatteryPct() {
  if (BAT_PIN < 0) return -1;
  uint32_t mv = analogReadMilliVolts(BAT_PIN) * 2; // divider หาร 2
  int pct = (int)((mv - 3300) * 100 / (4200 - 3300)); // LiPo 3.3V=0% 4.2V=100%
  return constrain(pct, 0, 100);
}

// ประมาณอัตราการหายใจจาก HR — เซนเซอร์นี้ไม่มีทางวัด RR ตรงๆ ได้
// ใช้สัดส่วน HR:RR ~4:1 ที่พบทั่วไปตอนพัก (คร่าวๆเท่านั้น ไม่ใช่ค่าวัดจริง แม่นยำต่ำกว่า HR/SpO2/BP มาก)
float estimateRespRate(float hr) {
  return constrain(hr / 4.0f, 8.0f, 40.0f);
}

// ดึงเวลาปัจจุบันเป็น ISO8601 "YYYY-MM-DDTHH:MM:SS" จาก NTP, false ถ้ายังไม่ sync
bool getIsoTimestamp(char *buf, size_t len) {
  struct tm t;
  if (!getLocalTime(&t, 0)) return false;
  strftime(buf, len, "%Y-%m-%dT%H:%M:%S", &t);
  return true;
}

// ส่งผลวัดขึ้น dashboard: {"device_id":"...","temperature":36.5,"heart_rate":72,"spo2":98,...}
// ไม่ส่ง patient_id — backend ผูก device_id กับผู้ป่วยเอง / temperature เป็น field บังคับ
// sampleCount < SCAN_TARGET_SAMPLES = ส่งมาแม้ scan "fail" (เก็บ sample ไม่ครบ) เพื่อให้เทส API pipeline ได้แม้สแกนไม่สมบูรณ์
// "partial":true บอก backend/dashboard ว่าค่านี้ความเชื่อถือได้ต่ำกว่าปกติ ไม่ใช่ผลวัดที่ครบสมบูรณ์
bool sendVitals(float hr, float spo2, float sys, float dia, int sampleCount) {
  // เน็ตหลุดชั่วขณะเป็นเรื่องปกติ — ลอง reconnect แล้วรอสั้นๆ ก่อนยอมแพ้ (บล็อกจอสูงสุด ~4 วิ เฉพาะตอนหลุดจริง)
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi down -> reconnecting...");
    WiFi.reconnect();
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(200);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Send skipped: no WiFi");
      return false;
    }
  }

  char ts[24];
  bool haveTs = getIsoTimestamp(ts, sizeof(ts));
  bool partial = sampleCount < SCAN_TARGET_SAMPLES;

  // ทุก vital เป็น optional หมด — ส่ง 0 ไป backend จะโดน validation ตีกลับ 400
  // รอบไหนวัดอะไรได้ก็ส่งอันนั้น (เช่น ได้แต่ SpO2 ก็ส่งแต่ SpO2)
  char json[384];
  int n = snprintf(json, sizeof(json),
    "{\"device_id\":\"%s\",\"temperature\":%.1f,\"sample_count\":%d,\"partial\":%s",
    DEVICE_NAME, TEMPERATURE_C, sampleCount, partial ? "true" : "false");
  if (hr > 0) {
    n += snprintf(json + n, sizeof(json) - n,
      ",\"heart_rate\":%d,\"respiratory_rate\":%d", (int)hr, (int)estimateRespRate(hr));
  }
  if (spo2 > 0) {
    n += snprintf(json + n, sizeof(json) - n, ",\"spo2\":%d", (int)spo2);
  }
  if (sys > 0 && dia > 0) {
    // BP เป็น optional — ไม่มีค่าก็ไม่ส่ง field
    n += snprintf(json + n, sizeof(json) - n,
      ",\"blood_pressure_sys\":%d,\"blood_pressure_dia\":%d", (int)sys, (int)dia);
  }
  if (haveTs) {
    n += snprintf(json + n, sizeof(json) - n, ",\"timestamp\":\"%s\"", ts);
  }
  snprintf(json + n, sizeof(json) - n, "}");

  // ลอง 2 รอบ: timeout/หลุดชั่วขณะรอบแรกไม่ควรทำให้ผลวัดหายทั้งรอบ
  int code = -1;
  for (int attempt = 1; attempt <= 2; attempt++) {
    HTTPClient http;
    http.begin(API_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-API-Key", API_KEY);
    http.setConnectTimeout(3000);
    http.setTimeout(5000); // 3000 เดิมสั้นไปสำหรับ backend ที่ตอบช้า ทำให้ fail ทั้งที่ server ได้รับแล้ว
    code = http.POST(json);
    http.end();

    // code < 0 = error ฝั่ง client (ต่อไม่ติด/timeout) — errorToString บอกสาเหตุจริง
    Serial.printf("POST attempt %d -> %d%s : %s\n", attempt, code,
                  code < 0 ? (String(" (") + HTTPClient::errorToString(code) + ")").c_str() : "",
                  json);
    if (code >= 200 && code < 300) return true;
    if (attempt == 1) delay(500);
  }
  return false;
}

// -------- MPU6050 FALL DETECT --------
// คุยกับชิปผ่าน Wire ตรงๆ (ไม่ต้องลง library เพิ่ม): ปลุกชิป + ตั้งช่วงวัด ±8g กันค่ากระแทกโดน clip
bool mpuBegin() {
  Wire.setClock(100000); // ลดเหลือ 100kHz ชั่วคราวตอน scan — สายจัมป์เปอร์ยาว/รางหลวมบางทีวิ่ง 400k ไม่ไหว
  // scan ทั้ง bus ออก Serial ก่อน — เห็นเลยว่าอุปกรณ์ไหนตอบบ้าง (debug สายหลวม/address ผิด)
  Serial.print("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();

  // ลองทั้ง 0x68 และ 0x69 (AD0 ลอย/ต่อ 3V3 = 0x69)
  for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
    Wire.beginTransmission(addr);
    Wire.write(0x6B); Wire.write(0x00);           // PWR_MGMT_1 = 0 ปลุกจาก sleep
    if (Wire.endTransmission() != 0) continue;
    Wire.beginTransmission(addr);
    Wire.write(0x1C); Wire.write(0x10);           // ACCEL_CONFIG = ±8g กันค่ากระแทกโดน clip
    if (Wire.endTransmission() == 0) {
      MPU_ADDR = addr;
      Serial.printf("MPU6050 found at 0x%02X\n", addr);
      Wire.setClock(100000); // เจอที่ 100k ก็อยู่ 100k ต่อ (MAX32664 ใช้ 100k ได้ ช้าลงนิดแต่ชัวร์)
      return true;
    }
  }
  Wire.setClock(400000); // ไม่เจอ MPU -> คืนความเร็วเดิมให้ MAX32664
  return false;
}

// อ่านความเร่งรวม 3 แกนเป็นหน่วย g (นิ่งๆ = ~1.0 จากแรงโน้มถ่วง, ตกอิสระ = ~0, กระแทก = พุ่งสูง)
// คืน -1 ถ้าอ่านไม่สำเร็จ (สายหลุดกลางทาง)
float mpuAccelG() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                              // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MPU_ADDR, 6) != 6) return -1;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  const float s = 4096.0f;                       // ±8g -> 4096 LSB/g
  float x = ax / s, y = ay / s, z = az / s;
  return sqrtf(x * x + y * y + z * z);
}

// แจ้งเหตุล้มขึ้น dashboard ทันที (แยกจากผลวัด vitals): {"device_id":..,"event":"fall",..}
// backend อาจยังไม่มี field นี้ — ถ้าโดน 400 ให้ทีมเว็บเพิ่ม event/fall_detected ฝั่ง Django
void sendFallAlert() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("FALL alert not sent: no WiFi");
    return;
  }
  char ts[24];
  bool haveTs = getIsoTimestamp(ts, sizeof(ts));
  char json[192];
  int n = snprintf(json, sizeof(json),
    "{\"device_id\":\"%s\",\"event\":\"fall\",\"fall_detected\":true", DEVICE_NAME);
  if (haveTs) n += snprintf(json + n, sizeof(json) - n, ",\"timestamp\":\"%s\"", ts);
  snprintf(json + n, sizeof(json) - n, "}");

  HTTPClient http;
  http.begin(API_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);
  http.setConnectTimeout(3000);
  http.setTimeout(5000);
  int code = http.POST(json);
  http.end();
  Serial.printf("FALL POST -> %d : %s\n", code, json);
}

// เรียกทุก loop: จับ pattern ตกอิสระ -> กระแทก ภายใน FALL_WINDOW_MS = ล้ม
void checkFall() {
  if (!mpuOk) return;
  float g = mpuAccelG();
  if (g < 0) return; // อ่านพลาดครั้งนี้ ข้ามไป

  // debug: print เฉพาะตอนค่าหลุดช่วงปกติ (นิ่งๆ ~1.0g) จะได้เห็นว่าใกล้ threshold แค่ไหน
  if (g < 0.6f || g > 1.8f) Serial.printf("[MPU] g=%.2f%s\n", g, freefallAt ? " (freefall!)" : "");

  if (g < FALL_FREEFALL_G) {
    if (freefallAt == 0) freefallAt = millis();       // เริ่มนับหน้าต่างเวลารอกระแทก
  } else if (freefallAt != 0) {
    if (g > FALL_IMPACT_G && millis() - freefallAt <= FALL_WINDOW_MS) {
      Serial.printf("FALL DETECTED! freefall->impact %.2fg in %lums\n", g, millis() - freefallAt);
      freefallAt = 0;
      fallAlertAt = millis();
      gfx->fillScreen(C_RED);
      drawCenter("FALL", 80, 5, C_WHITE);
      drawCenter("DETECTED!", 130, 3, C_WHITE);
      sendFallAlert();
    } else if (millis() - freefallAt > FALL_WINDOW_MS) {
      freefallAt = 0;                                  // หมดหน้าต่างเวลา ไม่มีกระแทกตาม = ไม่ใช่การล้ม
    }
  }
}

// -------- MAIN APP UI --------
// หน้าเดียวจบ ไม่มีหน้า SCANNING แยกแล้ว: แบต + เวลาจริง + HR/SpO2/BP/RR + แถบสถานะล่างสุด
// ที่เปลี่ยนไปตามว่ากำลังวัดอยู่ไหม (progress %) วาดทีละช่อง ลบเฉพาะกล่องที่เปลี่ยน -> จอนิ่ง ไม่กระพริบ
void drawHome() {
  char buf[24];

  // ---- วาดทั้งหน้าครั้งเดียว: กรอบไอคอนแบต + vitals ----
  if (homeFullDraw) {
    homeFullDraw = false;
    lastBatShown = -2;
    lastMinShown = -2; // -2 = ยังไม่วาดเวลาเลย (ต่างจาก key=-1 ตอน NTP ยังไม่ sync)
    lastStatusKey = -999; // บังคับวาดแถบสถานะใหม่ด้วย
    gfx->fillScreen(C_BLACK);

    // กรอบไอคอนแบต (คงที่)
    gfx->drawRect(150, 20, 26, 14, C_WHITE);
    gfx->fillRect(176, 24, 3, 6, C_WHITE);

    // จุดสถานะ API รอบล่าสุด: เขียว=ส่งสำเร็จ แดง=fail เทา=ยังไม่เคยส่ง
    gfx->fillCircle(66, 27, 5, lastSendStatus == 1 ? C_GREEN : lastSendStatus == 0 ? C_RED : C_GRAY);

    // vitals (เปลี่ยนเฉพาะหลังวัดเสร็จ -> วาดตรงนี้พอ)
    if (finalHr > 0) sprintf(buf, "HR   : %d", (int)finalHr);
    else             strcpy(buf, "HR   : --");
    drawCenter(buf, 105, 2, C_RED);

    if (finalSpo2 > 0) sprintf(buf, "SpO2 : %d%%", (int)finalSpo2);
    else               strcpy(buf, "SpO2 : --");
    drawCenter(buf, 130, 2, C_CYAN);

    if (finalSys > 0 && finalDia > 0) sprintf(buf, "BP   : %d/%d", (int)finalSys, (int)finalDia);
    else                              strcpy(buf, "BP   : --/--");
    drawCenter(buf, 155, 2, C_GREEN);

    // RR เป็นค่าประมาณจาก HR (ไม่มี sensor วัดตรง) จึงโชว์คู่กับ "~" กันเข้าใจผิดว่าเป็นค่าวัดจริง
    if (finalRR > 0) sprintf(buf, "RR ~ : %d", (int)finalRR);
    else              strcpy(buf, "RR ~ : --");
    drawCenter(buf, 180, 2, C_WARN);
  }

  // ---- แบต + เวลา: throttle วินาทีละครั้งพอ (ไม่ต้องเร็วกว่านี้) ----
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();

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

    struct tm t;
    int key;
    if (getLocalTime(&t, 0)) {                          // sync แล้ว -> เวลาจริง
      sprintf(buf, "%02d:%02d", t.tm_hour, t.tm_min);
      key = t.tm_hour * 60 + t.tm_min;
    } else {                                            // ยังไม่ sync (รอเน็ต) -> รอ
      strcpy(buf, "--:--");
      key = -1;
    }
    if (key != lastMinShown) {
      lastMinShown = key;
      gfx->fillRect(0, 60, 240, 40, C_BLACK);          // ลบเวลาเก่า (เฉพาะแถบนี้)
      drawCenter(buf, 60, 5, C_WHITE);
    }
  }

  drawStatusLine(); // อัปเดตทุกครั้งที่เรียก (ไม่ throttle) progress % ต้องขยับทันที ไม่รอ 1 วิ
}

// แถบสถานะล่างจอ home: 3 สถานะ — "Scanning... NN%" ตอนวัด, "Lift finger" ตอนวัดเสร็จแต่นิ้วยังวางค้าง
// (needRelease ค้าง = ไม่ยอมเริ่มรอบใหม่จนกว่าจะยกนิ้ว ถ้าไม่บอกผู้ใช้จะค้างแบบงงๆ), "Place finger" ตอนว่างจริง
// แทนที่หน้า SCANNING แยกทั้งหน้าแบบเดิม กันจอวูบวาบ + เห็นค่า HR/SpO2/BP/RR เดิมตลอดเวลาแม้กำลังวัดรอบใหม่
void drawStatusLine() {
  int pct = 0;
  int key;
  if (state == DETECTING) {
    pct = nSamples * 100 / SCAN_TARGET_SAMPLES;
    if (pct > 100) pct = 100;
    key = 1000 + pct;
  } else if (needRelease) {
    key = 2;
  } else {
    key = 0;
  }
  if (key == lastStatusKey) return; // ไม่เปลี่ยน ไม่ต้องวาดซ้ำ กันกระพริบ
  lastStatusKey = key;

  gfx->fillRect(0, 200, 240, 30, C_BLACK); // ลบเฉพาะแถบสถานะ ไม่แตะ vitals ด้านบน

  if (state == DETECTING) {
    char buf[20];
    sprintf(buf, "Scanning... %d%%", pct);
    drawCenter(buf, 203, 1, C_GREEN);
    gfx->drawRect(60, 220, 120, 8, C_GRAY);
    gfx->fillRect(61, 221, (118 * pct) / 100, 6, C_GREEN);
  } else if (needRelease) {
    drawCenter("Done! Lift finger to rescan", 205, 1, C_WARN);
  } else {
    drawCenter("Place finger to scan", 205, 1, C_GRAY);
  }
}

// -------- SCAN HELPERS --------
void resetScan() {
  sumHr = sumSpo2 = sumSys = sumDia = 0;
  nSamples = nSpo2Samples = nBpSamples = 0;
  fingerLostAt = 0;
}

// จบสแกน ไม่ว่าจะเก็บครบเป้าหรือหมดเวลา/นิ้วหลุดก่อน -> สรุปผลเท่าที่เก็บได้แล้วส่ง API
// ส่งแม้ sample ไม่ครบ (nSamples < SCAN_TARGET_SAMPLES) เพื่อให้เทส API pipeline ได้แม้สแกนไม่สมบูรณ์
// (field "partial":true ใน JSON บอก backend ว่าค่านี้ความเชื่อถือได้ต่ำกว่าปกติ)
void finishScan() {
  // แต่ละค่าเฉลี่ยจากตัวนับของตัวเอง — HR/SpO2 มาคนละจังหวะ นับรวมกันไม่ได้
  if (nSamples > 0 || nSpo2Samples > 0) {
    finalHr   = nSamples     ? sumHr / nSamples : 0;
    finalSpo2 = nSpo2Samples ? sumSpo2 / nSpo2Samples : 0;
    finalSys  = nBpSamples   ? sumSys / nBpSamples : 0;
    finalDia  = nBpSamples   ? sumDia / nBpSamples : 0;
    finalRR   = finalHr > 0  ? estimateRespRate(finalHr) : 0;

    Serial.printf("%s: HR %.0f (n=%d) | SpO2 %.0f (n=%d) | BP %.0f/%.0f (%d/%d samples, %.1fs)\n",
                  nSamples >= SCAN_TARGET_SAMPLES ? "RESULT" : "PARTIAL(FAIL)",
                  finalHr, nSamples, finalSpo2, nSpo2Samples, finalSys, finalDia,
                  nSamples, SCAN_TARGET_SAMPLES, (millis() - detectStart) / 1000.0);

    lastSendStatus = sendVitals(finalHr, finalSpo2, finalSys, finalDia, nSamples) ? 1 : 0;
  } else {
    Serial.println("Scan failed: 0 valid samples, nothing to send");
  }

  restartEstimation();  // สำคัญ: ไม่ restart แล้ว hub จะไม่ส่ง sample อีก = จอค้างรอบสอง
  needRelease = false;  // วัดต่อเนื่อง: นิ้ววางค้างไว้ = เริ่มรอบใหม่เองหลังโชว์ผลครบ MIN_HOME_HOLD_MS แล้วส่ง API ทุกรอบ
  resultShownAt = millis(); // เริ่มนับเวลาค้างจอผลลัพธ์
  homeFullDraw = true;  // วาดค่าใหม่ + เคลียร์แถบสถานะกลับเป็น idle ทันที
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

  mpuOk = mpuBegin();
  Serial.printf("MPU6050: %s\n", mpuOk ? "OK" : "NOT FOUND (fall detect disabled)");

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
  checkFall(); // เช็คทุกรอบ (~50Hz) ไม่ว่าจะอยู่ state ไหน — การล้มรอไม่ได้

  // alert ล้มค้างจอ FALL_ALERT_MS แล้วค่อยกลับหน้าหลัก (ระหว่างนี้หยุดวาด/สแกนชั่วคราว)
  if (fallAlertAt) {
    if (millis() - fallAlertAt >= FALL_ALERT_MS) {
      fallAlertAt = 0;
      homeFullDraw = true; // วาดหน้า home ใหม่ทับจอแดง
      resetScan();
      state = CLOCK_MODE;
    } else {
      delay(20);
      return;
    }
  }

  Max32664Sample sample;
  bool haveSample = false;
  bool fingerOn = false;
  bool validHr = false, validSpo2 = false;
  float hr = 0, spo2 = 0, sys = 0, dia = 0;

  if (hub.readSample(sample) == Max32664Status::Ok) {
    haveSample = true;
    hr = sample.heartRate();
    spo2 = sample.spo2();
    sys = sample.systolic;
    dia = sample.diastolic;

    fingerOn = (sample.bpStatus != Max32664BpStatus::NoFinger);
    // เช็คแยกกัน: sensor รายงาน HR กับ SpO2 คนละ sample (เช็ครวมแบบเดิม = ได้ค่าแค่ฝั่งเดียว)
    validHr   = (fingerOn && hr > 30.0 && hr < 220.0);
    validSpo2 = (fingerOn && spo2 > 50.0 && spo2 <= 100.0);

    // debug ทาง Serial ทุก sample
    Serial.printf("[%s] HR: %.1f | SpO2: %.1f | BP: %.0f/%.0f | Finger: %s | Valid: HR=%s SpO2=%s\n",
                  state == CLOCK_MODE ? "HOME" : "SCAN",
                  hr, spo2, sys, dia, fingerOn ? "ON" : "OFF",
                  validHr ? "Y" : "N", validSpo2 ? "Y" : "N");
  }

  // -------- STATE MACHINE --------
  // ไม่มีหน้า SCANNING แยกแล้ว — ทั้งสอง state วาดผ่าน drawHome() เดียวกันเสมอ (สถานะเปลี่ยนแค่แถบล่างจอ)
  switch (state) {

    case CLOCK_MODE:
      drawHome();
      // debounce "ยกนิ้ว": ต้องไม่มีนิ้วต่อเนื่องเกิน RELEASE_DEBOUNCE_MS ถึงเคลียร์ needRelease
      // กัน sample หลอกหลัง restartEstimation() ทำให้จอกลับเข้า DETECTING ทันทีจนเห็นค่าใหม่ไม่ทัน
      if (haveSample) {
        if (!fingerOn) {
          if (fingerOffSince == 0) fingerOffSince = millis();
          if (needRelease && millis() - fingerOffSince > RELEASE_DEBOUNCE_MS) needRelease = false;
        } else {
          fingerOffSince = 0;
        }
      }
      // ต้องค้างหน้าผลลัพธ์ให้ครบ MIN_HOME_HOLD_MS ก่อน ถึงจะยอมเริ่มสแกนรอบใหม่
      if (haveSample && fingerOn && !needRelease && millis() - resultShownAt >= MIN_HOME_HOLD_MS) {
        // เจอนิ้ว -> เริ่มสแกนทันที
        resetScan();
        detectStart = millis();
        state = DETECTING;
      }
      break;

    case DETECTING: {
      // debug: นิ้วหลุดต่อเนื่องเกินกำหนด -> จบสแกน (ส่งผลเท่าที่เก็บได้) กลับหน้าหลัก
      if (haveSample) {
        if (fingerOn) {
          fingerLostAt = 0;
        } else if (fingerLostAt == 0) {
          fingerLostAt = millis();
        }
      }
      if (fingerLostAt && millis() - fingerLostAt > FINGER_LOST_MS) {
        Serial.println("Finger removed early");
        finishScan();
        break;
      }

      // สะสมค่าเฉลี่ยแยกฝั่ง: HR valid เก็บ HR, SpO2 valid เก็บ SpO2 (มาคนละ sample ได้)
      // progress ผูกกับ HR (nSamples) เพราะมาถี่กว่า — SpO2 ได้เท่าไหร่เอาเท่านั้น
      if (validHr)   { sumHr   += hr;   nSamples++; }
      if (validSpo2) { sumSpo2 += spo2; nSpo2Samples++; }
      // BP เก็บแยก: sensor จ่ายมาเมื่อไหร่เก็บเมื่อนั้น (ไม่ผูกกับ valid ของ HR/SpO2)
      // BP ต้อง calibrate สำเร็จก่อน sensor ถึงจะจ่ายค่า ไม่มีก็ปล่อย --/-- (optional)
      if (fingerOn && sys > 0 && dia > 0) {
        sumSys += sys; sumDia += dia; nBpSamples++;
      }

      drawHome(); // อัปเดตแถบสถานะ/progress % บนหน้า home เดิม ไม่สลับหน้า

      // HR ครบเป้าอย่างเดียวยังไม่จบ — รอ SpO2 ให้ได้อย่างน้อย SPO2_MIN_SAMPLES ก่อน
      // (SpO2 มาช้ากว่า HR มาก ถ้าจบทันทีจะได้แต่ HR) ถ้ารอจนชนเพดานเวลาก็จบด้วย timeout ข้างล่าง
      if (nSamples >= SCAN_TARGET_SAMPLES && nSpo2Samples >= SPO2_MIN_SAMPLES) {
        finishScan();
      } else if (millis() - detectStart >= SCAN_MAX_MS) {
        // สัญญาณแย่จนเก็บไม่ครบในเวลาเพดาน = fail แต่ยังส่งผลเท่าที่เก็บได้ขึ้น API เพื่อเทส pipeline
        Serial.printf("Scan timeout: only %d/%d valid samples in %lus\n",
                      nSamples, SCAN_TARGET_SAMPLES, SCAN_MAX_MS / 1000);
        finishScan();
      }
      break;
    }
  }

  delay(20); // กัน FIFO ล้น ห้ามหน่วงมากกว่านี้
}
