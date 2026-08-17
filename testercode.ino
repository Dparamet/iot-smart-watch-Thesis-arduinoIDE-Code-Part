/*
  File: SmartWatch_XIAO_C3_StableVitals_FloorReject_20260802_v10.ino
  Board: Seeed Studio XIAO ESP32-C3

  Revision changes (v10):
  1) Preserve the original clock/dashboard, pin map, fall alert, API and scan state machine
  2) Connect automatically to the previously saved WiFi; open SmartWatch-Setup only when needed
  3) Reconnect WiFi automatically in the background after a temporary disconnect
  4) Restore continuous live HR/SpO2/BP/RR updates on the clock screen while the finger stays on
  5) Keep the stable battery filter, BP firmware/calibration fix and brighter HR text from v04
  6) Fix legacy 824-byte calibration-vector loading by sending one complete I2C frame
  7) Count HR/SpO2 at a controlled 500 ms cadence and cap the result sample count at 20
  8) Keep live values updating separately from the 20-sample result average
  9) Reject impossible battery readings instead of displaying a false 0%
 10) Throttle Serial sensor logs so repeated FIFO values do not flood the monitor
 11) Fix bpStatus=NoSignal (0) being mistaken for Finger ON, which could stall at 19/20
 12) End a partial scan after continuous signal loss instead of printing 0/0 forever
 13) Throttle repeated battery wiring warnings (hardware wiring still must be fixed)
 14) Separate physical contact from usable optical-signal quality
 15) Reject stale SpO2 when HR has disappeared or BP status reports motion/weak signal
 16) End a partial scan after several seconds without a fresh optical pulse instead of waiting 40 seconds
 17) Print an unmistakable V10 build banner at boot so the uploaded sketch can be verified
 18) Reject the repeated 70.0 SpO2 floor/unready value instead of averaging and uploading it
 19) Require SpO2 and BP to be accompanied by a stable HR pulse in the same sample
 20) Reject implausible/resting HR spikes and sudden jumps such as 206/218 bpm
 21) Require several consecutive stable optical samples before counting
 22) Clear scan counters after finishing so HOME logs do not show stale counts
*/

#include <Wire.h>
#include <Preferences.h>
#include "max32664.h"
#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <time.h>

#define BUILD_TAG "SmartWatch V10 StableVitals 2026-08-02"

/* --important infomation --
  max32664D buy from this Provider : https://protocentral.com/product/pulse-express-pulse-ox-heart-rate-sensor-with-max32664/#downloads
  Library from  Provider : https://github.com/Protocentral/protocentral-pulse-express
*/

// -------- PIN MAP: XIAO ESP32-C3 (Battery + Buzzer revision) --------
// MAX32664
#define HUB_RESET   3   // XIAO D1 / GPIO3
#define HUB_MFIO    2   // XIAO D0 / GPIO2

// Shared I2C bus: MAX32664 + MPU6050
#define I2C_SDA     6   // XIAO D4 / GPIO6
#define I2C_SCL     7   // XIAO D5 / GPIO7

// GC9A01 TFT — wiring fix v03
#define TFT_SCK     8   // XIAO D8  / GPIO8  -> TFT SCL/SCK
#define TFT_MOSI    10  // XIAO D10 / GPIO10 -> TFT SDA/MOSI
#define TFT_CS      GFX_NOT_DEFINED  // TFT CS ต่อ GND ค้างไว้
#define TFT_DC      21  // XIAO D6  / GPIO21 -> TFT DC
#define TFT_RST     20  // XIAO D7  / GPIO20 -> TFT RST/RES
// จอรุ่น 7 ขาไม่มี BLK แยก: VCC ต่อ 3V3 โดยตรง

// Added devices
#define BAT_PIN     4   // XIAO D2 / A2 / GPIO4 (ADC1_CH4)
#define BUZZER_PIN  5   // XIAO D3 / A3 / GPIO5 (digital output)
#define BUZZER_ACTIVE_LEVEL   HIGH
#define BUZZER_INACTIVE_LEVEL LOW

// -------- WIFI / DASHBOARD --------
// ไม่ hardcode SSID/password แล้ว — ใช้ WiFiManager ให้ผู้ใช้ตั้งค่าเองผ่านหน้าเว็บตอนบูตครั้งแรก (บันทึกลง flash เอง)
#define WIFI_MANAGER_AP_NAME         "SmartWatch-Setup"  // ชื่อ AP ตอนเปิดหน้าตั้งค่า WiFi
#define WIFI_MANAGER_AP_PASSWORD     "watch1234"          // รหัสผ่าน AP ตั้งค่า (ต้อง >=8 ตัว) กันคนแปลกหน้าเข้ามาแก้ WiFi/ดักข้อมูล เปลี่ยนได้ตามต้องการ
#define WIFI_CONFIG_PORTAL_TIMEOUT_S 180UL                // เปิด portal สูงสุด 3 นาที เฉพาะตอนยังต่อ WiFi เดิมไม่ได้
#define WIFI_SAVED_CONNECT_TIMEOUT_MS 15000UL              // ลอง WiFi ที่บันทึกไว้ก่อน 15 วินาที
#define WIFI_RECONNECT_INTERVAL_MS    10000UL              // เน็ตหลุดให้ลองต่อใหม่ทุก 10 วินาที
#define BOOT_BTN_PIN                 9                    // ปุ่ม BOOT บนบอร์ด XIAO ESP32C3 (active LOW) กดค้างตอนเปิดเครื่อง = ลืม WiFi เดิม
#define API_URL       "http://172.24.155.69:8000/api/iot/vitals/"  // แก้เป็น endpoint ของ dashboard
#define API_KEY       "DufwFwDIRjwFa6LvdwA7PmF3pg4CgA6C"  // ส่งผ่าน header X-API-Key
#define DEVICE_NAME   "WT001"
#define TEMPERATURE_C 36.5f  // backend บังคับส่ง แต่เซนเซอร์นี้วัดอุณหภูมิไม่ได้ — ส่งค่าคงที่ไปก่อน ถ้าต่อ MLX90614 เมื่อไหร่ค่อยเปลี่ยนเป็นค่าวัดจริง

// -------- TIME / BATTERY --------
#define TZ_OFFSET_SEC (7 * 3600)  // ไทย GMT+7
#define NTP_SERVER    "pool.ntp.org"

// วงจรวัดแบต: BAT_SW+ -> R_TOP 200k -> BAT_PIN -> R_BOTTOM 200k -> GND
// แนะนำ C 100nF ต่อจาก BAT_PIN ลง GND เพื่อให้ ADC นิ่ง
#define BAT_R_TOP_OHM          200000.0f
#define BAT_R_BOTTOM_OHM       200000.0f
#define BAT_EMPTY_MV           3300.0f
#define BAT_FULL_MV            4200.0f
#define BAT_SAMPLE_COUNT       31        // จำนวนคี่ เพื่อหา median ได้ตรงกลาง
#define BAT_READ_INTERVAL_MS   5000UL    // แบตจริงไม่เปลี่ยนทุกวินาที ลดการกระโดดจาก WiFi/โหลดจอ
#define BAT_EMA_ALPHA          0.18f     // ยิ่งน้อยยิ่งนิ่ง แต่ตอบสนองช้าลง
#define BAT_DISPLAY_HYST_PCT   2         // ค่าเป้าหมายต้องต่าง >=2% จึงขยับค่าที่แสดง
#define BAT_VALID_MIN_MV        2500.0f   // ต่ำกว่านี้ถือว่าสาย ADC_BAT หลุด/ต่อผิด ไม่ใช่แบตจริง
#define BAT_VALID_MAX_MV        4600.0f   // สูงกว่านี้ถือว่าสเกลตัวต้านทานหรือสายผิด

// -------- COLOR --------
#define C_BLACK     0x0000
#define C_RED       0xF800
#define C_CYAN      0x07FF
#define C_HR_BRIGHT 0xBFFF  // ฟ้าอมขาว สว่างกว่า cyan เดิมบนจอ GC9A01
#define C_WHITE     0xFFFF
#define C_GRAY      0x8410
#define C_GREEN     0x07E0
#define C_WARN      0xFFE0

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
#define SCAN_TARGET_SAMPLES 20       // valid HR sample ครบเท่านี้ = 100%
#define SPO2_MIN_SAMPLES    10       // ต้องมี SpO2 คุณภาพดีอย่างน้อย 10 ค่า จึงถือว่ารอบสมบูรณ์
#define BP_MIN_SAMPLES      1        // ต้องมี BPT report จริงอย่างน้อย 1 ครั้ง
#define BP_WAIT_MAX_MS      30000UL  // HR/SpO2 ครบแล้ว รอ BP ได้สูงสุด 30 วินาที
#define SCAN_MAX_MS         40000UL  // เพดานรวม ถ้าสัญญาณไม่ครบให้จบที่ 40 วินาที
#define FINGER_LOST_MS      1500UL   // ยกนิ้วต่อเนื่องเกินนี้ = ยกเลิกสแกน กลับหน้าหลัก
#define LIVE_VITALS_REFRESH_MS 400UL  // ระหว่างวัด อัปเดต HR/SpO2/BP/RR บนจอทุก 0.4 วินาที
#define SAMPLE_ACCEPT_INTERVAL_MS 500UL // นับค่าใหม่สูงสุด 2 ครั้ง/วินาที ไม่เอาค่า FIFO ซ้ำมานับรัว
#define SENSOR_DEBUG_INTERVAL_MS  500UL // พิมพ์ Serial สูงสุด 2 ครั้ง/วินาที
#define BAT_ERROR_LOG_INTERVAL_MS 15000UL // สายวัดแบตผิดให้เตือนทุก 15 วิ ไม่พ่นข้อความรัว
#define OPTICAL_STARTUP_GRACE_MS    8000UL // ให้ algorithm ตั้งตัวก่อนตัดสินว่าสัญญาณคุณภาพไม่พอ
#define OPTICAL_QUALITY_LOST_MS      3500UL // ไม่มี HR/SpO2 ที่ยืนยันด้วยชีพจรต่อเนื่องเกินนี้ = จบรอบ partial
#define HR_ACCEPT_MIN_BPM              40.0f  // โหมดนี้วัดขณะอยู่นิ่ง ค่าต่ำกว่านี้ให้ถือว่าสัญญาณผิด
#define HR_ACCEPT_MAX_BPM             180.0f  // ตัด spike 206/218 ที่เกิดจาก motion/สัญญาณหลอก
#define HR_MAX_JUMP_BPM                25.0f  // ค่าต่อเนื่องต้องไม่กระโดดเกินนี้
#define STABLE_SAMPLE_STREAK_REQUIRED   3     // ต้องนิ่งติดกันก่อนเริ่มนับ
#define SPO2_ACCEPT_MIN_PCT             70.5f // 70.0 ซ้ำคือค่าขอบล่าง/unready ของชุดนี้ ไม่อัปโหลดเป็นผลจริง
#define SPO2_MAX_JUMP_PCT                6.0f

// ค่าอ้างอิงจากเครื่องวัดความดันแบบผ้าพันแขน ณ ตอน Calibration
// เปลี่ยนสองค่านี้ให้เป็นค่าจริงของผู้ใช้ก่อน Calibration เพื่อให้ BPT มีความหมาย
#define BP_REF_SYSTOLIC     120
#define BP_REF_DIASTOLIC     80
#define BP_CAL_INDEX          0      // firmware 40.5.0+ รองรับ index 0..4
#define CALIB_SCHEMA_VERSION  4      // ใช้ calibration 824-byte เดิมที่วัดสำเร็จแล้วได้
#define LEGACY_I2C_BUFFER_BYTES 1024 // ต้องมากกว่า command 3 bytes + vector 824 bytes
#define LEGACY_I2C_TIMEOUT_MS    300  // ที่ I2C 100kHz เฟรม 827 bytes ใช้เวลามากกว่า timeout 50ms เดิม

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
int  lastHrShown = -999;
int  lastSpo2Shown = -999;
int  lastSysShown = -999;
int  lastDiaShown = -999;
int  lastRRShown = -999;
unsigned long lastLiveVitalsAt = 0;

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
unsigned long lastHrAcceptedAt = 0;
unsigned long lastSpo2AcceptedAt = 0;
unsigned long lastSensorDebugAt = 0;
unsigned long lastGoodOpticalAt = 0; // sample ล่าสุดที่มีสัญญาณ optical ใช้งานได้จริง
unsigned long poorOpticalAt = 0;     // เริ่มนับเมื่อยังสัมผัสอยู่แต่ไม่มี pulse ที่เชื่อถือได้
float lastStableHrCandidate = 0.0f;
float lastStableSpo2Candidate = 0.0f;
int hrStableStreak = 0;
int spo2StableStreak = 0;
bool hrCandidateReady = false;
bool spo2CandidateReady = false;

// ค่าสำหรับแสดงสด แยกจากค่าเฉลี่ย 20 sample ที่ใช้เป็นผลลัพธ์
float liveHr = 0, liveSpo2 = 0, liveSys = 0, liveDia = 0;
bool liveHrReady = false, liveSpo2Ready = false, liveBpReady = false;

static uint8_t calibVec[824];
static size_t  calibLen = 0;

// battery smoothing state
bool     batteryFilterReady = false;
float    batteryFilteredMv = 0.0f;
uint32_t batteryLastRawMv = 0;
unsigned long batteryLastReadAt = 0;
unsigned long batteryLastErrorLogAt = 0;
int      batteryDisplayPct = -1;

bool wifiOk = false;
bool ntpStarted = false;
unsigned long lastWifiReconnectAt = 0;
int lastSendStatus = -1; // -1 ยังไม่เคยส่ง, 0 ส่ง fail, 1 ส่งสำเร็จ — โชว์เป็นจุดสีบนจอ

// fall detect state
bool mpuOk = false;               // เจอ MPU6050 ตอนบูตไหม (ไม่เจอ = ข้าม fall detect ระบบวัดทำงานปกติ)
unsigned long freefallAt = 0;     // เวลาที่เริ่มเจอช่วงตกอิสระ (0 = ยังไม่เจอ)
unsigned long fallAlertAt = 0;    // เวลาที่เจอการล้มล่าสุด (0 = ไม่มี alert ค้างจอ)

// Buzzer alert state (non-blocking: ไม่ใช้ delay ยาว จึงยังเช็คระบบได้)
#define FALL_BUZZER_TOGGLE_MS 250UL
bool buzzerOutputOn = false;
unsigned long buzzerToggleAt = 0;

// -------- BUZZER --------
// โค้ดนี้ออกแบบสำหรับ Active Buzzer 3.3V / Buzzer module ที่ดังเมื่อ SIG เป็น HIGH
// ถ้า Buzzer กินกระแสมาก ห้ามต่อโหลดตรงจาก GPIO ให้ขับผ่านทรานซิสเตอร์
void setBuzzer(bool on) {
  buzzerOutputOn = on;
  digitalWrite(BUZZER_PIN, on ? BUZZER_ACTIVE_LEVEL : BUZZER_INACTIVE_LEVEL);
}

void startFallBuzzer() {
  buzzerToggleAt = millis();
  setBuzzer(true);
}

void updateFallBuzzer() {
  if (!fallAlertAt) {
    setBuzzer(false);
    return;
  }

  if (millis() - buzzerToggleAt >= FALL_BUZZER_TOGGLE_MS) {
    buzzerToggleAt = millis();
    setBuzzer(!buzzerOutputOn);
  }
}

void stopFallBuzzer() {
  setBuzzer(false);
}

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
void clearCalib() {
  prefs.begin("pulse", false);
  prefs.clear();
  prefs.end();
  calibLen = 0;
}

bool loadCalib() {
  prefs.begin("pulse", true);
  uint8_t schema = prefs.getUChar("schema", 0);
  uint8_t savedMulti = prefs.getUChar("multi", 255);
  calibLen = prefs.getUInt("len", 0);

  bool expectedMulti = hub.caps().multiPointCalib;
  bool valid = (schema == CALIB_SCHEMA_VERSION) &&
               (savedMulti == (expectedMulti ? 1 : 0)) &&
               (calibLen == hub.caps().calibVectorBytes) &&
               (calibLen <= sizeof(calibVec));

  if (valid) {
    size_t got = prefs.getBytes("vec", calibVec, calibLen);
    valid = (got == calibLen);
  }
  prefs.end();

  if (!valid) calibLen = 0;
  return valid;
}

void saveCalib() {
  prefs.begin("pulse", false);
  prefs.putUChar("schema", CALIB_SCHEMA_VERSION);
  prefs.putUChar("multi", hub.caps().multiPointCalib ? 1 : 0);
  prefs.putUInt("len", calibLen);
  prefs.putBytes("vec", calibVec, calibLen);
  prefs.end();
}

// -------- FORCE SENSOR CALIBRATION --------
bool runCalibration() {
  Serial.println("Calibrating Sensor...");
  Serial.printf("Reference BP = %d/%d mmHg | mode=%s | vector=%u bytes\n",
                BP_REF_SYSTOLIC, BP_REF_DIASTOLIC,
                hub.caps().multiPointCalib ? "multi-point" : "legacy",
                hub.caps().calibVectorBytes);

  gfx->fillScreen(C_BLACK);
  drawCenter("CALIBRATE", 70, 3, C_WARN);
  drawCenter("Hold finger 2 min", 130, 2, C_WHITE);

  Max32664Status st;
  if (hub.caps().multiPointCalib) {
    Max32664CalibrationRef ref;
    ref.calIndex = BP_CAL_INDEX;
    ref.systolic = BP_REF_SYSTOLIC;
    ref.diastolic = BP_REF_DIASTOLIC;
    st = hub.startCalibration(ref);
  } else {
    Max32664LegacyCalibrationRefs refs;
    refs.systolic[0]  = BP_REF_SYSTOLIC;
    refs.systolic[1]  = BP_REF_SYSTOLIC + 2;
    refs.systolic[2]  = BP_REF_SYSTOLIC + 5;
    refs.diastolic[0] = BP_REF_DIASTOLIC;
    refs.diastolic[1] = BP_REF_DIASTOLIC + 1;
    refs.diastolic[2] = BP_REF_DIASTOLIC + 2;
    st = hub.startCalibration(refs);
  }

  if (st != Max32664Status::Ok) {
    Serial.printf("startCalibration FAIL: 0x%02X\n", (uint8_t)st);
    drawCenter("CALIB START FAIL", 120, 2, C_RED);
    delay(3000);
    return false;
  }

  Max32664Sample s;
  uint8_t lastPct = 255;
  char buf[24];
  bool completed = false;

  for (unsigned long t0 = millis(); millis() - t0 < 120000UL; delay(40)) {
    Max32664Status r = hub.readSample(s);
    if (r != Max32664Status::Ok) continue;

    if (s.progress != lastPct) {
      Serial.printf("Calib Progress: %3d%% | BP status=%u\n",
                    s.progress, (uint8_t)s.bpStatus);
      gfx->fillScreen(C_BLACK);
      drawCenter("CALIBRATING", 70, 3, C_WARN);
      sprintf(buf, "Progress %d%%", s.progress);
      drawCenter(buf, 130, 2, C_WHITE);
      lastPct = s.progress;
    }

    if (s.bpStatus == Max32664BpStatus::Success && s.progress >= 100) {
      completed = true;
      break;
    }

    if (s.bpStatus == Max32664BpStatus::EstimationFailure ||
        s.bpStatus == Max32664BpStatus::SubjectInitFailure ||
        s.bpStatus == Max32664BpStatus::TooManyCalibrations ||
        s.bpStatus == Max32664BpStatus::RefOutOfLimits) {
      Serial.printf("Calibration rejected by hub, BP status=%u\n", (uint8_t)s.bpStatus);
      break;
    }
  }

  if (!completed) {
    Serial.println("Calibration did not reach 100%");
    hub.stop();
    drawCenter("CALIB NOT COMPLETE", 120, 2, C_RED);
    delay(3000);
    return false;
  }

  st = hub.readCalibrationVector(calibVec, sizeof(calibVec), &calibLen);
  if (st != Max32664Status::Ok || calibLen != hub.caps().calibVectorBytes) {
    Serial.printf("readCalibrationVector FAIL: 0x%02X, len=%u expected=%u\n",
                  (uint8_t)st, (unsigned)calibLen,
                  (unsigned)hub.caps().calibVectorBytes);
    hub.stop();
    drawCenter("CALIB READ FAIL", 120, 2, C_RED);
    delay(3000);
    return false;
  }

  saveCalib();
  st = hub.stop();
  if (st != Max32664Status::Ok) {
    Serial.printf("hub.stop after calibration: 0x%02X\n", (uint8_t)st);
  }

  Serial.printf("Calibration completed and saved (%u bytes)\n", (unsigned)calibLen);
  return true;
}

// MAX32664 firmware legacy ใช้ calibration vector 824 bytes ในคำสั่งเดียว
// ไลบรารี 2.0.0 แบ่ง vector เป็นหลาย I2C frame ซึ่ง hub legacy บางรุ่นตอบ 0x01 IllegalIndex
// ESP32-C3 ขยาย Wire buffer ได้ จึงส่ง header 0x50/0x04/0x03 + vector ทั้งหมดในเฟรมเดียว
Max32664Status loadLegacyCalibrationVectorSingleFrame(const uint8_t *vec, size_t len) {
  if (vec == nullptr || len != hub.caps().calibVectorBytes || len != 824) {
    return Max32664Status::InvalidArgument;
  }

#if defined(WIRE_HAS_BUFFER_SIZE)
  const size_t frameBytes = len + 3;
  size_t actualBuffer = Wire.setBufferSize(LEGACY_I2C_BUFFER_BYTES);
  if (actualBuffer < frameBytes) {
    Serial.printf("Legacy vector buffer too small: %u < %u\n",
                  (unsigned)actualBuffer, (unsigned)frameBytes);
    return Max32664Status::BufferTooSmall;
  }

  uint16_t oldTimeout = Wire.getTimeOut();
  Wire.setTimeOut(LEGACY_I2C_TIMEOUT_MS);
  Wire.flush();

  Wire.beginTransmission(MAX32664_I2C_ADDR);
  size_t written = 0;
  written += Wire.write((uint8_t)0x50);
  written += Wire.write((uint8_t)0x04);
  written += Wire.write((uint8_t)0x03);
  written += Wire.write(vec, len);

  uint8_t i2cError = Wire.endTransmission(true);
  if (written != frameBytes || i2cError != 0) {
    Wire.setTimeOut(oldTimeout);
    Serial.printf("Legacy vector I2C write FAIL: wrote=%u/%u, wireError=%u\n",
                  (unsigned)written, (unsigned)frameBytes, i2cError);
    return (written != frameBytes) ? Max32664Status::BufferTooSmall
                                   : Max32664Status::HostCommError;
  }

  delay(100);
  if (Wire.requestFrom((uint8_t)MAX32664_I2C_ADDR, (size_t)1, true) != 1) {
    Wire.setTimeOut(oldTimeout);
    return Max32664Status::HostCommError;
  }

  uint8_t status = (uint8_t)Wire.read();
  Wire.setTimeOut(oldTimeout);
  Serial.printf("Legacy calibration-vector status: 0x%02X\n", status);
  return status == 0x00 ? Max32664Status::Ok : (Max32664Status)status;
#else
  return hub.loadCalibrationVector(vec, len);
#endif
}

bool beginEstimation() {
  Max32664Status st;

  if (hub.caps().multiPointCalib) {
    st = hub.loadCalibrationVector(BP_CAL_INDEX, calibVec, calibLen);
  } else {
    st = loadLegacyCalibrationVectorSingleFrame(calibVec, calibLen);
  }

  if (st != Max32664Status::Ok) {
    Serial.printf("loadCalibrationVector FAIL: 0x%02X | len=%u expected=%u | mode=%s\n",
                  (uint8_t)st, (unsigned)calibLen,
                  (unsigned)hub.caps().calibVectorBytes,
                  hub.caps().multiPointCalib ? "multi-point" : "legacy");
    return false;
  }

  st = hub.startEstimation(Max32664Spo2Coeffs{});
  if (st != Max32664Status::Ok) {
    Serial.printf("startEstimation FAIL: 0x%02X\n", (uint8_t)st);
    return false;
  }

  Serial.printf("MAX32664 estimation started | %s\n", BUILD_TAG);
  return true;
}

// hub หยุด stream หลังจบรอบวัด ต้อง stop + start ใหม่ทุกครั้งก่อนกลับหน้าหลัก
// ไม่งั้น readSample fail ตลอด = สแกนได้แค่รอบเดียว
void restartEstimation() {
  Max32664Status st = hub.stop();
  if (st != Max32664Status::Ok) {
    Serial.printf("hub.stop FAIL during restart: 0x%02X\n", (uint8_t)st);
  }
  delay(100);

  if (!beginEstimation()) {
    Serial.println("ERROR: estimation restart failed");
  }
}

// -------- WIFI + JSON UPLOAD --------
// ใช้ WiFiManager แทน hardcode SSID/password: ถ้าเคยตั้งค่าไว้แล้วจะต่อเองอัตโนมัติ
// ถ้ายังไม่เคยตั้ง (หรือกดปุ่ม BOOT ค้างตอนเปิดเครื่อง) จะเปิด AP ชื่อ WIFI_MANAGER_AP_NAME
// ให้เอามือถือ/คอมไปต่อ แล้วเข้า http://192.168.4.1 เพื่อเลือก WiFi + ใส่รหัสผ่าน (บันทึกลง flash เอง ไม่ต้องแก้โค้ดใหม่)
void startNtpClock() {
  configTime(TZ_OFFSET_SEC, 0, NTP_SERVER);
  ntpStarted = true;
  Serial.println("NTP clock started (GMT+7)");
}

void onWiFiConnected() {
  wifiOk = true;
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  Serial.printf("WiFi connected automatically: %s | IP: %s\n",
                WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  startNtpClock();
}

// ลำดับการต่อ WiFi:
// 1) ถ้ามี WiFi ที่เคยตั้งไว้ ให้ต่อเองทันทีโดยไม่เปิดหน้า Setup
// 2) ถ้ายังไม่เคยตั้ง/ต่อไม่สำเร็จ จึงเปิด AP SmartWatch-Setup
// 3) กด BOOT ค้างตอนเปิดเครื่อง = ล้าง WiFi เดิมและเปิด Setup ใหม่
void connectWiFi() {
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  Serial.printf("Device MAC: %s\n", WiFi.macAddress().c_str());

  WiFiManager wm;
  wm.setConfigPortalTimeout(WIFI_CONFIG_PORTAL_TIMEOUT_S);
  wm.setConnectTimeout(20);
  wm.setConnectRetries(3);

  bool forcePortal = (digitalRead(BOOT_BTN_PIN) == LOW);
  if (forcePortal) {
    Serial.println("BOOT held -> clear saved WiFi and open setup portal");
    wm.resetSettings();
    WiFi.disconnect(true, true);
    delay(300);
  }

  // ลองเชื่อมข้อมูล WiFi ที่ ESP32 บันทึกไว้ก่อน โดยไม่เปิด AP
  if (!forcePortal) {
    gfx->fillScreen(C_BLACK);
    drawCenter("CONNECTING WIFI", 90, 2, C_CYAN);
    drawCenter("Saved network...", 125, 1, C_WHITE);

    WiFi.begin();
    unsigned long startedAt = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startedAt < WIFI_SAVED_CONNECT_TIMEOUT_MS) {
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      onWiFiConnected();
      return;
    }

    Serial.println("Saved WiFi unavailable -> opening SmartWatch-Setup");
  }

  // เข้ามาตรงนี้เฉพาะยังไม่มี WiFi เดิม หรือ WiFi เดิมต่อไม่ได้
  gfx->fillScreen(C_BLACK);
  drawCenter("WIFI SETUP", 60, 2, C_WARN);
  drawCenter("Connect phone to:", 95, 1, C_WHITE);
  drawCenter(WIFI_MANAGER_AP_NAME, 115, 2, C_CYAN);
  drawCenter("AP password:", 145, 1, C_WHITE);
  drawCenter(WIFI_MANAGER_AP_PASSWORD, 160, 1, C_CYAN);
  drawCenter("open 192.168.4.1", 185, 1, C_GRAY);

  wifiOk = wm.autoConnect(WIFI_MANAGER_AP_NAME, WIFI_MANAGER_AP_PASSWORD);
  if (wifiOk && WiFi.status() == WL_CONNECTED) {
    onWiFiConnected();
  } else {
    wifiOk = false;
    Serial.printf("WiFi setup timeout; continue offline, status=%d\n", WiFi.status());
  }
}

// เรียกใน loop ตลอดเวลา: ถ้า WiFi หลุด ระบบวัดและนาฬิกายังทำงานต่อ
// และลองเชื่อม WiFi เดิมใหม่เองทุก WIFI_RECONNECT_INTERVAL_MS
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiOk) {
      onWiFiConnected();
      homeFullDraw = true;
    }
    return;
  }

  if (wifiOk) {
    wifiOk = false;
    Serial.println("WiFi disconnected; watch continues offline");
  }

  unsigned long now = millis();
  if (now - lastWifiReconnectAt < WIFI_RECONNECT_INTERVAL_MS) return;
  lastWifiReconnectAt = now;

  Serial.println("WiFi auto-reconnect attempt...");
  WiFi.reconnect();
}

// -------- BATTERY --------
// 200k/200k ทำให้แหล่งสัญญาณเข้า ADC มี impedance สูง จึงใช้:
// 1) dummy read + เวลาพัก 2) sort แล้วตัดค่าหัว/ท้าย 3) EMA ข้ามเวลา
// 4) อ่านทุก 5 วินาที 5) hysteresis ที่เปอร์เซ็นต์
uint32_t readBatteryMvRawStable() {
  uint16_t samples[BAT_SAMPLE_COUNT];

  // ทิ้งค่าครั้งแรกหลังสลับ ADC mux และให้ C ที่ ADC_BAT มีเวลาชาร์จ
  analogReadMilliVolts(BAT_PIN);
  delay(2);

  for (int i = 0; i < BAT_SAMPLE_COUNT; i++) {
    samples[i] = (uint16_t)analogReadMilliVolts(BAT_PIN);
    delayMicroseconds(500);
  }

  // insertion sort: จำนวนข้อมูลน้อย ใช้ RAM ต่ำ และไม่ต้องเพิ่ม library
  for (int i = 1; i < BAT_SAMPLE_COUNT; i++) {
    uint16_t key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  // ตัด 25% ต่ำสุดและสูงสุดออก แล้วเฉลี่ยส่วนกลาง ลด spike จาก WiFi/ADC
  const int trim = BAT_SAMPLE_COUNT / 4;
  uint32_t sum = 0;
  int used = 0;
  for (int i = trim; i < BAT_SAMPLE_COUNT - trim; i++) {
    sum += samples[i];
    used++;
  }

  float adcMv = sum / (float)used;
  float dividerRatio = (BAT_R_TOP_OHM + BAT_R_BOTTOM_OHM) / BAT_R_BOTTOM_OHM;
  return (uint32_t)lroundf(adcMv * dividerRatio);
}

int batteryMvToPct(float mv) {
  int pct = (int)lroundf(
    (mv - BAT_EMPTY_MV) * 100.0f / (BAT_FULL_MV - BAT_EMPTY_MV)
  );
  return constrain(pct, 0, 100);
}

int readBatteryPct() {
  unsigned long now = millis();

  if (!batteryFilterReady || now - batteryLastReadAt >= BAT_READ_INTERVAL_MS) {
    batteryLastReadAt = now;
    batteryLastRawMv = readBatteryMvRawStable();

    // 8 mV / 0 mV ไม่ใช่แบตหมด แต่หมายถึง ADC_BAT อยู่ใกล้ GND หรือไม่ได้ต่อ BAT_SW+
    if (batteryLastRawMv < BAT_VALID_MIN_MV || batteryLastRawMv > BAT_VALID_MAX_MV) {
      batteryFilterReady = false;
      batteryFilteredMv = 0.0f;
      batteryDisplayPct = -1;
      if (batteryLastErrorLogAt == 0 ||
          now - batteryLastErrorLogAt >= BAT_ERROR_LOG_INTERVAL_MS) {
        batteryLastErrorLogAt = now;
        Serial.printf("[BAT ERROR] raw=%lumV is impossible. Check BAT_SW+ -> 200k -> ADC_BAT(GPIO4) -> 200k -> GND_SYS\n",
                      (unsigned long)batteryLastRawMv);
      }
      return -1;
    }

    if (!batteryFilterReady) {
      batteryFilteredMv = batteryLastRawMv;
      batteryFilterReady = true;
    } else {
      batteryFilteredMv += BAT_EMA_ALPHA * ((float)batteryLastRawMv - batteryFilteredMv);
    }

    int targetPct = batteryMvToPct(batteryFilteredMv);

    if (batteryDisplayPct < 0) {
      batteryDisplayPct = targetPct;
    } else if (abs(targetPct - batteryDisplayPct) >= BAT_DISPLAY_HYST_PCT) {
      // ขยับครั้งละ 1% เพื่อไม่ให้ตัวเลขกระโดดหลายเปอร์เซ็นต์ในครั้งเดียว
      batteryDisplayPct += (targetPct > batteryDisplayPct) ? 1 : -1;
    }

    Serial.printf("[BAT] raw=%lumV | filtered=%.0fmV | target=%d%% | shown=%d%%\n",
                  (unsigned long)batteryLastRawMv, batteryFilteredMv,
                  targetPct, batteryDisplayPct);
  }

  return batteryDisplayPct;
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
      startFallBuzzer();
      gfx->fillScreen(C_RED);
      drawCenter("FALL", 80, 5, C_WHITE);
      drawCenter("DETECTED!", 130, 3, C_WHITE);
      sendFallAlert();
    } else if (millis() - freefallAt > FALL_WINDOW_MS) {
      freefallAt = 0;                                  // หมดหน้าต่างเวลา ไม่มีกระแทกตาม = ไม่ใช่การล้ม
    }
  }
}

// วาดค่า vital แบบแยกแต่ละบรรทัด เพื่อให้ค่าระหว่างสแกนอัปเดตรัวได้
// โดยไม่ต้อง fillScreen ทั้งจอและไม่ทำให้นาฬิกากระพริบ
void drawVitalsPanel(bool force) {
  int hrValue = finalHr > 0 ? (int)lroundf(finalHr) : -1;
  int spo2Value = finalSpo2 > 0 ? (int)lroundf(finalSpo2) : -1;
  int sysValue = (finalSys > 0 && finalDia > 0) ? (int)lroundf(finalSys) : -1;
  int diaValue = (finalSys > 0 && finalDia > 0) ? (int)lroundf(finalDia) : -1;
  int rrValue = finalRR > 0 ? (int)lroundf(finalRR) : -1;

  // ระหว่างวัด แสดงค่าล่าสุดแบบ EMA ต่อเนื่อง แยกจากค่าเฉลี่ยผลลัพธ์ 20 sample
  if (state == DETECTING) {
    if (liveHrReady) {
      hrValue = (int)lroundf(liveHr);
      rrValue = (int)lroundf(estimateRespRate(liveHr));
    }
    if (liveSpo2Ready) {
      spo2Value = (int)lroundf(liveSpo2);
    }
    if (liveBpReady) {
      sysValue = (int)lroundf(liveSys);
      diaValue = (int)lroundf(liveDia);
    }
  }

  char buf[24];

  if (force || hrValue != lastHrShown) {
    lastHrShown = hrValue;
    gfx->fillRect(0, 102, 240, 23, C_BLACK);
    if (hrValue > 0) sprintf(buf, "HR   : %d", hrValue);
    else             strcpy(buf, "HR   : --");
    drawCenter(buf, 105, 2, C_HR_BRIGHT);
  }

  if (force || spo2Value != lastSpo2Shown) {
    lastSpo2Shown = spo2Value;
    gfx->fillRect(0, 127, 240, 23, C_BLACK);
    if (spo2Value > 0) sprintf(buf, "SpO2 : %d%%", spo2Value);
    else               strcpy(buf, "SpO2 : --");
    drawCenter(buf, 130, 2, C_CYAN);
  }

  if (force || sysValue != lastSysShown || diaValue != lastDiaShown) {
    lastSysShown = sysValue;
    lastDiaShown = diaValue;
    gfx->fillRect(0, 152, 240, 23, C_BLACK);
    if (sysValue > 0 && diaValue > 0) sprintf(buf, "BP   : %d/%d", sysValue, diaValue);
    else                              strcpy(buf, "BP   : --/--");
    drawCenter(buf, 155, 2, C_GREEN);
  }

  if (force || rrValue != lastRRShown) {
    lastRRShown = rrValue;
    gfx->fillRect(0, 177, 240, 23, C_BLACK);
    if (rrValue > 0) sprintf(buf, "RR ~ : %d", rrValue);
    else             strcpy(buf, "RR ~ : --");
    drawCenter(buf, 180, 2, C_WARN);
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

    // บังคับวาดค่า HR/SpO2/BP/RR ครั้งแรกของหน้า
    lastHrShown = lastSpo2Shown = lastSysShown = lastDiaShown = lastRRShown = -999;
    drawVitalsPanel(true);
    lastLiveVitalsAt = millis();
  }