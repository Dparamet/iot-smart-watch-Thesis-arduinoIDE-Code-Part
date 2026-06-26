/*
 * IoT Smart Watch — Protocentral Pulse Express (MAX32664D) + GC9A01 1.28" Round TFT
 *
 * MAX32664D → XIAO ESP32-C3
 *   VCC→3.3V  GND→GND
 *   SDA→D4(GPIO6)   SCL→D5(GPIO7)
 *   MFIO→D0(GPIO2)  RESET→D1(GPIO3)
 *
 * GC9A01 → XIAO ESP32-C3
 *   VCC→3.3V  GND→GND
 *   SCL/SCK→D8(GPIO8)   SDA/MOSI→D10(GPIO10)
 *   CS→D7(GPIO20)       DC→D6(GPIO21)
 *   RST→D3(GPIO5)       BL→D2(GPIO4)
 *
 * Libraries:
 *   - protocentral-pulse-express (github.com/Protocentral/protocentral-pulse-express)
 *   - Arduino_GFX_Library by Moon On Our Nation (Library Manager)
 *
 * ส่ง 'r' ทาง Serial เพื่อ recalibrate
 */

#include <Wire.h>
#include <Preferences.h>
#include "max32664.h"
#include <Arduino_GFX_Library.h>

// ---- MAX32664D pins ----
#define HUB_RESET  3   // D1
#define HUB_MFIO   2   // D0

// ---- GC9A01 pins ----
#define TFT_SCK    8   // D8
#define TFT_MOSI   10  // D10
#define TFT_CS     20  // D7
#define TFT_DC     21  // D6
#define TFT_RST    5   // D3
#define TFT_BL     4   // D2

// ---- display colors ----
#define CLR_BG     0x0000  // black
#define CLR_HR     0xF800  // red
#define CLR_SPO2   0x07FF  // cyan
#define CLR_LABEL  0x8410  // gray
#define CLR_WARN   0xFFE0  // yellow

Max32664    hub(HUB_RESET, HUB_MFIO);
Preferences prefs;

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI);
Arduino_GFX     *gfx = new Arduino_GC9A01(bus, TFT_RST);

static uint8_t calibVec[824];
static size_t  calibLen = 0;

// ---- calibration storage ----

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

// ---- display helpers ----

void drawLabel(const char *top, const char *bot, uint16_t color) {
    gfx->fillScreen(CLR_BG);
    gfx->setTextColor(color);
    gfx->setTextSize(3);
    int16_t tw = strlen(top) * 18;
    gfx->setCursor((240 - tw) / 2, 85);
    gfx->print(top);
    gfx->setTextSize(2);
    int16_t bw = strlen(bot) * 12;
    gfx->setCursor((240 - bw) / 2, 130);
    gfx->print(bot);
}

void drawVitals(float hr, float spo2) {
    gfx->fillScreen(CLR_BG);

    // HR — ตัวใหญ่กลางจอ
    char hrStr[8];
    snprintf(hrStr, sizeof(hrStr), "%d", (int)hr);
    gfx->setTextSize(7);
    gfx->setTextColor(CLR_HR);
    int16_t tw = strlen(hrStr) * 42;
    gfx->setCursor((240 - tw) / 2, 55);
    gfx->print(hrStr);

    // label BPM
    gfx->setTextSize(2);
    gfx->setTextColor(CLR_LABEL);
    gfx->setCursor(96, 120);
    gfx->print("BPM");

    // divider line
    gfx->drawFastHLine(60, 145, 120, CLR_LABEL);

    // SpO2
    char spo2Str[10];
    snprintf(spo2Str, sizeof(spo2Str), "%.1f%%", spo2);
    gfx->setTextSize(4);
    gfx->setTextColor(CLR_SPO2);
    int16_t sw = strlen(spo2Str) * 24;
    gfx->setCursor((240 - sw) / 2, 158);
    gfx->print(spo2Str);

    // label SpO2
    gfx->setTextSize(2);
    gfx->setTextColor(CLR_LABEL);
    gfx->setCursor(92, 200);
    gfx->print("SpO2");
}

// ---- sensor calibration ----

void runCalibration() {
    Serial.println("Calibrating...");
    drawLabel("CALIBRATE", "วางนิ้วนิ่งๆ", CLR_WARN);

    Max32664LegacyCalibrationRefs refs;
    for (int i = 0; i < 3; i++) {
        refs.systolic[i]  = 120 + i * 2 + (i == 2 ? 1 : 0);
        refs.diastolic[i] = 80  + i;
    }

    if (hub.startCalibration(refs) != Max32664Status::Ok) {
        Serial.println("startCalibration FAIL");
        drawLabel("CALIB FAIL", "เช็คสาย", CLR_WARN);
        return;
    }

    Max32664Sample s;
    uint8_t lastPct = 255;
    char buf[24];
    for (unsigned long t0 = millis(); millis() - t0 < 120000UL; delay(40)) {
        if (hub.readSample(s) != Max32664Status::Ok) continue;
        if (s.progress != lastPct) {
            Serial.printf("  %3d%%  HR:%.1f SpO2:%.1f%%\n",
                s.progress, s.heartRate(), s.spo2());
            snprintf(buf, sizeof(buf), "วางนิ้ว %d%%", s.progress);
            drawLabel("CALIBRATE", buf, CLR_WARN);
            lastPct = s.progress;
        }
        if (s.bpStatus == Max32664BpStatus::Success && s.progress >= 100) break;
    }

    hub.readCalibrationVector(calibVec, sizeof(calibVec), &calibLen);
    saveCalib();
    hub.stop();
    Serial.printf("Calibration saved (%d bytes)\n", calibLen);
}

void beginEstimation() {
    hub.loadCalibrationVector(calibVec, calibLen);
    hub.startEstimation(Max32664Spo2Coeffs{});
}

// ---- setup / loop ----

void setup() {
    delay(2000);
    Serial.begin(115200);

    // display init
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    gfx->begin();
    gfx->fillScreen(CLR_BG);
    drawLabel("BOOTING", "please wait...", CLR_LABEL);

    // sensor init
    Wire.begin(6, 7);
    if (hub.begin() != Max32664Status::Ok) {
        Serial.println("hub FAIL");
        drawLabel("HUB FAIL", "เช็คสาย", CLR_WARN);
        while (true) delay(5000);
    }
    Serial.printf("Hub FW: %d.%d.%d\n",
        hub.version().major, hub.version().minor, hub.version().patch);

    if (loadCalib()) {
        Serial.printf("Calibration loaded (%d bytes)\n", calibLen);
    } else {
        runCalibration();
    }

    beginEstimation();
    drawLabel("READY", "วางนิ้ว...", CLR_SPO2);
    Serial.println("พร้อม (ส่ง 'r' เพื่อ recalibrate)");
}

void loop() {
    if (Serial.available() && Serial.read() == 'r') {
        hub.stop();
        prefs.begin("pulse", false); prefs.clear(); prefs.end();
        runCalibration();
        beginEstimation();
        Serial.println("พร้อม");
    }

    static unsigned long lastUpdate = 0;
    static float sumHr = 0, sumSpo2 = 0;
    static int   count = 0;

    Max32664Sample buf[8];
    size_t n = 0;
    if (hub.readSamples(buf, 8, &n) != Max32664Status::Ok) return;

    for (size_t i = 0; i < n; i++) {
        float hr = buf[i].heartRate(), spo2 = buf[i].spo2();
        if (buf[i].bpStatus == Max32664BpStatus::NoFinger) continue;
        if (hr < 40 || hr > 200 || spo2 < 70) continue;
        sumHr += hr; sumSpo2 += spo2; count++;
    }

    if (millis() - lastUpdate >= 2000) {
        lastUpdate = millis();
        if (count > 0) {
            float hr = sumHr / count, spo2 = sumSpo2 / count;
            Serial.printf("HR: %.1f bpm  SpO2: %.1f%%\n", hr, spo2);
            drawVitals(hr, spo2);
            sumHr = sumSpo2 = 0; count = 0;
        } else {
            drawLabel("--", "วางนิ้ว...", CLR_LABEL);
        }
    }
}
