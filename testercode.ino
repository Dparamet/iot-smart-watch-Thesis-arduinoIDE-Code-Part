#include <Wire.h>
#include <Preferences.h>
#include "max32664.h"
#include <Arduino_GFX_Library.h>

// -------- PIN --------
#define HUB_RESET   3
#define HUB_MFIO    2

#define TFT_SCK     8
#define TFT_MOSI    10
#define TFT_CS      20
#define TFT_DC      21
#define TFT_RST     5
#define TFT_BL      4

// -------- COLOR --------
#define C_BLACK 0x0000
#define C_RED   0xF800
#define C_CYAN  0x07FF
#define C_WHITE 0xFFFF
#define C_GRAY  0x8410
#define C_GREEN 0x07E0
#define C_WARN  0xFFE0

// -------- OBJECT --------
Max32664 hub(HUB_RESET, HUB_MFIO);
Preferences prefs;

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED
);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST);

// -------- STATE --------
enum State {
  CLOCK_MODE,
  WAIT_FINGER,
  DETECTING,
  SHOW_DATA
};

State state = CLOCK_MODE;
State lastState = CLOCK_MODE;

// -------- VAR --------
unsigned long lastUpdate = 0;
unsigned long detectStart = 0;
unsigned long stateTimer = 0; // ใช้เช็คระยะเวลาหลุดของนิ้วแทน delay()
int secondsFake = 0;

float finalHr = 0, finalSpo2 = 0, finalSys = 0, finalDia = 0;

static uint8_t calibVec[824];
static size_t  calibLen = 0;

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
  drawCenter("วางนิ้วนิ่งๆ 2 นาที", 130, 2, C_WHITE);

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
      sprintf(buf, "คืบหน้า %d%%", s.progress);
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
    drawCenter(buf, 90, 4, C_WHITE);
    drawCenter("Smart Watch", 150, 2, C_GRAY);
  }
}

void showWaiting() {
  // สั่งวาดครั้งเดียวตอนเข้าสเตท ป้องกันหน้าจอกะพริบ
  static State lastDrawState = CLOCK_MODE;
  if (lastDrawState != state) {
    drawCenter("Place Finger", 100, 2, C_WHITE);
    lastDrawState = state;
  }
}

void showDetecting() {
  static State lastDrawState = CLOCK_MODE;
  if (lastDrawState != state) {
    drawCenter("Detecting...", 110, 2, C_GREEN);
    lastDrawState = state;
  }
}

void showVitals(float hr, float spo2, float sys, float dia) {
  char buf[20];

  sprintf(buf, "%d BPM", (int)hr);
  drawCenter(buf, 45, 3, C_RED);

  gfx->drawFastHLine(60, 85, 120, C_GRAY);

  sprintf(buf, "%0.0f/%0.0f", sys, dia);
  drawCenter(buf, 105, 4, C_GREEN);
  drawCenter("mmHg", 145, 2, C_GRAY);

  gfx->drawFastHLine(60, 175, 120, C_GRAY);

  sprintf(buf, "SpO2: %d%%", (int)spo2);
  drawCenter(buf, 190, 3, C_CYAN);
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
  Wire.setClock(400000); // เร่งความเร็ว I2C ให้ดึงข้อมูลได้ไวขึ้น

  if (hub.begin() != Max32664Status::Ok) {
    drawCenter("HUB ERROR", 120, 2, C_RED);
    while (1);
  }

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
  bool valid = false;
  float hr = 0, spo2 = 0, sys = 0, dia = 0;

  // รันด้วยความเร็วสูงสุด ห้ามขวางลูปดึงข้อมูลเด็ดขาด!
  if (hub.readSample(sample) == Max32664Status::Ok) {
    hr = sample.heartRate();
    spo2 = sample.spo2();
    sys = sample.systolic;  
    dia = sample.diastolic; 

    bool fingerOn = (sample.bpStatus != Max32664BpStatus::NoFinger);
    // ค่าสัมบูรณ์ที่ชิปจะเริ่มปล่อยออกมาเมื่อเจอนิ้วจริง
    valid = (fingerOn && hr > 30.0 && spo2 > 50.0);

    Serial.printf("HR: %.1f | SpO2: %.1f | BP: %.0f/%.0f | Finger: %s\n", 
                  hr, spo2, sys, dia, fingerOn ? "ON" : "OFF");
  }

  // รีเฟรชหน้าจอเคลียร์ขยะครั้งเดียวเมื่อเปลี่ยนสเตท
  if (state != lastState) {
    gfx->fillScreen(C_BLACK);
    lastState = state;
  }

  // -------- STATE MACHINE --------
  switch (state) {

    case CLOCK_MODE:
      drawClock();
      if (valid) {
        state = WAIT_FINGER;
      }
      break;

    case WAIT_FINGER:
      showWaiting();
      if (valid) {
        detectStart = millis();
        state = DETECTING;
      } else {
        state = CLOCK_MODE;
      }
      break;

    case DETECTING:
      showDetecting();
      if (valid) {
        // ถือนิ้วค้างไว้ 4 วินาทีเพื่อรอให้อัลกอริทึมชิปนิ่งและปล่อยค่าสมบูรณ์ออกมา
        if (millis() - detectStart > 4000) { 
          finalHr = hr;
          finalSpo2 = spo2;
          finalSys = sys;
          finalDia = dia;
          state = SHOW_DATA;
          stateTimer = millis();
        }
      } else {
        state = CLOCK_MODE;
      }
      break;

    case SHOW_DATA:
      showVitals(finalHr, finalSpo2, finalSys, finalDia);
      
      if (valid) {
        stateTimer = millis(); // รีเซ็ตเวลาตราบใดที่นิ้วยังวางอยู่
      }
      
      // ถ้ายกนิ้วออกต่อเนื่องกันเกิน 3 วินาที (Non-blocking ย้าย State) ถึงจะกลับหน้าปัดนาฬิกา
      if (millis() - stateTimer > 3000) {
        state = CLOCK_MODE;
      }
      break;
  }

  delay(20); // ปรับหน่วงเวลาลงมาเหลือแค่ 20ms เพื่อป้องกัน FIFO Buffer ของเซนเซอร์ล้น!
}