#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <math.h>
#include "protocentral_pulse_express.h"

// ======================================================
// XIAO ESP32-C3 + MAX32664D / Pulse Express
// R5.7: fixes stale timestamp underflow that instantly cleared new HR/SpO2/BP values
// MAX logic/filter/calibration kept from the proven 40.2.2 code.
// Serial Monitor: 115200
//
// VERIFIED ORIGINAL PIN MAP:
// MAX GND  -> GND
// MAX VCC  -> 3V3
// MAX RST  -> D2 / GPIO4
// MAX MFIO -> D1 / GPIO3
// MAX SDA  -> D4 / GPIO6
// MAX SCL  -> D5 / GPIO7
//
// MPU6050:
// SDA -> D4 / GPIO6
// SCL -> D5 / GPIO7
//
// GC9A01:
// SCK  -> D8  / GPIO8
// MOSI -> D10 / GPIO10
// DC   -> D6  / GPIO21
// CS   -> GND
// RST  -> D7  / GPIO20
//
// CURRENT FREE USER PIN:
// D0 / GPIO2 -> FREE (NO BATTERY CODE YET)
//
// First use:
// 1) Measure BP 3 times with a real cuff.
// 2) Serial Monitor -> Newline -> 115200
// 3) Send:
//    CAL SYS1 DIA1 SYS2 DIA2 SYS3 DIA3
// Example FORMAT only:
//    CAL 121 78 119 77 120 79
//
// Display: partial valid values appear immediately with * while filters are confirming.
// Commands:
// HELP / STATUS / SCAN / BEEP / FALLTEST
 // CAL SYS1 DIA1 SYS2 DIA2 SYS3 DIA3
 // ERASE / RESTART
// ======================================================

#define I2C_SDA 6
#define I2C_SCL 7

const int RESET_PIN = 4;  // D2
const int MFIO_PIN  = 3;  // D1

constexpr uint8_t PIN_BUZZER   = 5;   // D3
constexpr uint8_t PIN_TFT_DC   = 21;  // D6
constexpr uint8_t PIN_TFT_RST  = 20;  // D7
constexpr uint8_t PIN_TFT_SCK  = 8;   // D8
constexpr uint8_t PIN_BOOT     = 9;   // D9
constexpr uint8_t PIN_TFT_MOSI = 10;  // D10

PulseExpress hub(RESET_PIN, MFIO_PIN);
Preferences prefs;


// ======================================================
// GC9A01 DISPLAY - ORIGINAL SOFTWARE SPI PIN MAP
// ======================================================

Arduino_DataBus *tftBus = new Arduino_SWSPI(
  PIN_TFT_DC,
  GFX_NOT_DEFINED,      // CS hardwired to GND
  PIN_TFT_SCK,
  PIN_TFT_MOSI,
  GFX_NOT_DEFINED       // no MISO
);

Arduino_GFX *tft = new Arduino_GC9A01(
  tftBus,
  PIN_TFT_RST,
  0,
  true
);

bool displayOK = false;

constexpr uint16_t C_BLACK  = 0x0000;
constexpr uint16_t C_WHITE  = 0xFFFF;
constexpr uint16_t C_RED    = 0xF800;
constexpr uint16_t C_GREEN  = 0x07E0;
constexpr uint16_t C_CYAN   = 0x07FF;
constexpr uint16_t C_YELLOW = 0xFFE0;
constexpr uint16_t C_GRAY   = 0x7BEF;
constexpr uint16_t C_DARK   = 0x2104;

// ======================================================
// MPU6050 - SHARES ORIGINAL I2C BUS
// ======================================================

constexpr uint8_t MPU_ADDR_A = 0x68;
constexpr uint8_t MPU_ADDR_B = 0x69;

bool mpuOK = false;
uint8_t mpuAddr = 0;
float accelG = 1.0f;
float gyroDps = 0.0f;
bool movingTooMuch = false;
uint32_t lastMpuRead = 0;
uint32_t lastUiMs = 0;


// ======================================================
// FALL DETECTION / BUTTON / BUZZER
// Event-based: free-fall -> impact -> post-impact stillness
// ======================================================
enum FallStage : uint8_t { FALL_IDLE, FALL_FREEFALL, FALL_IMPACT_WAIT, FALL_STILL_CHECK };
FallStage fallStage = FALL_IDLE;

uint32_t fallStageMs = 0;
uint32_t fallStillStartMs = 0;
bool fallDetected = false;
bool fallAlarmMuted = false;
uint32_t lastFallBuzzMs = 0;

bool lastBootState = HIGH;
uint32_t bootPressedMs = 0;
uint32_t lastBootDebounceMs = 0;

uint8_t uiPage = 0; // 0=vitals, 1=system/status

bool maxOnline = false;
uint32_t lastMaxRetryMs = 0;

// Firmware 40.2.2 uses legacy calibration vector = 824 bytes.
constexpr size_t CAL_MAX = 824;
uint8_t calVector[CAL_MAX];
size_t calLen = 0;

bool haveCal = false;
bool running = false;
bool fingerPresent = false;

// MAX/ADI example SpO2 polynomial.
// For a final medical product this must be calibrated for the final optics.
constexpr float SPO2_A = 1.5958422f;
constexpr float SPO2_B = -34.659664f;
constexpr float SPO2_C = 112.68987f;

// Broad sanity gates.
// These DO NOT force values into a normal range.
// Bad values are rejected and shown as "--".
constexpr float HR_MIN = 35.0f;
constexpr float HR_MAX = 220.0f;
constexpr float SPO2_MIN = 70.0f;
constexpr float SPO2_MAX = 100.0f;
constexpr int SYS_MIN = 70;
constexpr int SYS_MAX = 250;
constexpr int DIA_MIN = 40;
constexpr int DIA_MAX = 150;

// Filtered outputs.
float hrFiltered = 0;
float spo2Filtered = 0;
float sysFiltered = 0;
float diaFiltered = 0;

bool hrReady = false;
bool spo2Ready = false;
bool bpReady = false;

uint8_t hrCount = 0;
uint8_t spo2Count = 0;
uint8_t bpCount = 0;

uint32_t lastHrMs = 0;
uint32_t lastSpO2Ms = 0;
uint32_t lastBpMs = 0;
uint32_t lastPrintMs = 0;

// Measurement notification / heartbeat.
// Keeps Serial readable: one status line every 10 seconds.
constexpr uint32_t MEASURE_STATUS_MS = 2000UL;
bool hrSpo2ReadyNotified = false;
bool bpReadyNotified = false;

PulseExpressBpStatus lastBpStatus = PulseExpressBpStatus::NoSignal;

// ======================================================
// LIVE DISPLAY CACHE
// ======================================================
// Keep the most recent VALID values separately from the filter-ready flags.
// This lets the watch show "partial" values immediately instead of "--"
// while the algorithm is still collecting enough samples.
float uiHr = 0.0f;
float uiSpO2 = 0.0f;
int uiSys = 0;
int uiDia = 0;

bool uiHrValid = false;
bool uiSpO2Valid = false;
bool uiBpValid = false;

uint32_t uiHrMs = 0;
uint32_t uiSpO2Ms = 0;
uint32_t uiBpMs = 0;

// Keep a recent result visible for a short time even if MAX reports
// a transient contact drop.
constexpr uint32_t UI_HR_HOLD_MS = 20000UL;
constexpr uint32_t UI_SPO2_HOLD_MS = 20000UL;
constexpr uint32_t UI_BP_HOLD_MS = 30000UL;

// Force the next normal UI refresh when a new measurement stage becomes ready.
bool uiForceRefresh = false;

// Contact debounce.
// MAX32664 can briefly report NO_SIGNAL/NO_CONTACT between good samples.
// Do not erase HR/SpO2/BP from the screen because of one transient packet.
constexpr uint8_t CONTACT_LOST_MIN_SAMPLES = 8;
constexpr uint32_t CONTACT_LOST_MIN_MS = 300UL;

uint8_t noContactSamples = 0;
uint32_t noContactSinceMs = 0;
bool resetFiltersOnNextContact = false;

// Calibration screen state.
// Draw the static background only once so software SPI does not waste time
// redrawing the entire 240x240 screen every 500 ms.
bool calibrationUiBaseDrawn = false;


const char *bpStatusName(PulseExpressBpStatus s);

// ======================================================
// DISPLAY HELPERS
// ======================================================

void textAt(
  int16_t x,
  int16_t y,
  const char *txt,
  uint16_t color,
  uint8_t size)
{
  if (!displayOK || txt == nullptr)
    return;

  tft->setTextSize(size);
  tft->setTextColor(color, C_BLACK);
  tft->setCursor(x, y);
  tft->print(txt);
}

void drawHeart(int16_t cx, int16_t cy)
{
  if (!displayOK)
    return;

  tft->fillCircle(cx - 5, cy - 3, 6, C_RED);
  tft->fillCircle(cx + 5, cy - 3, 6, C_RED);

  tft->fillTriangle(
    cx - 11, cy,
    cx + 11, cy,
    cx,      cy + 13,
    C_RED
  );
}

void manualTftReset()
{
  pinMode(PIN_TFT_RST, OUTPUT);

  digitalWrite(PIN_TFT_RST, HIGH);
  delay(150);

  digitalWrite(PIN_TFT_RST, LOW);
  delay(250);

  digitalWrite(PIN_TFT_RST, HIGH);
  delay(350);
}

void drawStaticUI()
{
  if (!displayOK) return;

  tft->fillScreen(C_BLACK);
  tft->drawCircle(120, 120, 117, C_CYAN);
  tft->drawCircle(120, 120, 116, C_DARK);

  if (uiPage == 0)
  {
    textAt(55, 14, "SMART WATCH", C_CYAN, 2);

    drawHeart(38, 60);
    textAt(57, 53, "HEART RATE", C_WHITE, 1);
    textAt(181, 54, "BPM", C_GRAY, 1);

    tft->drawFastHLine(28, 94, 184, C_DARK);

    textAt(34, 108, "SpO2", C_CYAN, 2);
    textAt(183, 110, "%", C_GRAY, 2);

    textAt(34, 149, "BP", C_YELLOW, 2);
    textAt(174, 151, "mmHg", C_GRAY, 1);

    tft->drawFastHLine(28, 188, 184, C_DARK);
    textAt(44, 205, "WAITING SENSOR", C_GRAY, 1);
  }
  else
  {
    textAt(53, 15, "SYSTEM STATUS", C_CYAN, 2);

    textAt(34, 52,  "MAX",  C_WHITE, 1);
    textAt(34, 78,  "MPU",  C_WHITE, 1);
    textAt(34, 104, "ACC",  C_WHITE, 1);
    textAt(34, 130, "GYR",  C_WHITE, 1);
    textAt(34, 156, "FALL", C_WHITE, 1);
    textAt(34, 182, "FREE", C_WHITE, 1);

    textAt(45, 214, "BOOT: change page", C_GRAY, 1);
  }
}

void updateUI()
{
  if (!displayOK) return;

  // Normal redraw is rate-limited, but new valid values can request
  // an immediate refresh so the screen does not sit at 25%.
  if (!uiForceRefresh && millis() - lastUiMs < 300)
    return;

  lastUiMs = millis();
  uiForceRefresh = false;

  char buf[48];

  if (uiPage == 1)
  {
    tft->fillRect(74, 42, 142, 160, C_BLACK);

    textAt(76, 52,
           maxOnline ? "ONLINE" : "OFFLINE",
           maxOnline ? C_GREEN : C_RED,
           1);

    textAt(76, 78,
           mpuOK ? "READY" : "OFFLINE",
           mpuOK ? C_GREEN : C_RED,
           1);

    snprintf(buf, sizeof(buf), "%.2fg", accelG);
    textAt(76, 104, buf, C_WHITE, 1);

    snprintf(buf, sizeof(buf), "%.1fdps", gyroDps);
    textAt(76, 130, buf, C_WHITE, 1);

    if (fallDetected)
      textAt(76, 156,
             fallAlarmMuted ? "ACK" : "ALERT",
             fallAlarmMuted ? C_YELLOW : C_RED,
             1);
    else
      textAt(76, 156, "NORMAL", C_GREEN, 1);

    // D0/GPIO2 is intentionally unused in this clean core build.
    textAt(76, 182, "D0 / GPIO2", C_CYAN, 1);

    return;
  }

  // ----------------------------------------------------
  // LIVE VALUES
  // ----------------------------------------------------
  // READY value = filtered result.
  // Partial value = most recent valid MAX value, shown immediately.

  // Heart rate
  tft->fillRect(75, 65, 104, 28, C_BLACK);

  if (uiHrValid)
  {
    snprintf(buf, sizeof(buf), "%.0f%s", uiHr, hrReady ? "" : "*");
    textAt(hrReady ? 89 : 82, 65, buf, hrReady ? C_RED : C_YELLOW, 3);
  }
  else
  {
    textAt(99, 65, "--", C_GRAY, 3);
  }

  // SpO2
  tft->fillRect(100, 103, 82, 30, C_BLACK);

  if (uiSpO2Valid)
  {
    snprintf(buf, sizeof(buf), "%.1f%s", uiSpO2, spo2Ready ? "" : "*");
    textAt(spo2Ready ? 108 : 101, 104, buf, spo2Ready ? C_CYAN : C_YELLOW, 2);
  }
  else
  {
    textAt(119, 104, "--.-", C_GRAY, 2);
  }

  // Blood pressure
  tft->fillRect(72, 143, 104, 30, C_BLACK);

  if (uiBpValid)
  {
    snprintf(
      buf,
      sizeof(buf),
      bpReady ? "%d/%d" : "%d/%d*",
      uiSys,
      uiDia
    );

    textAt(bpReady ? 78 : 72, 145, buf, C_YELLOW, 2);
  }
  else
  {
    textAt(91, 145, "--/--", C_GRAY, 2);
  }

  // Bottom status + small measurement progress.
  // This percentage is only UI progress (contact -> HR/SpO2 -> BP ready),
  // not a medical accuracy score.
  tft->fillRect(27, 194, 187, 39, C_BLACK);

  const char *msg = "WAITING";
  uint16_t col = C_GRAY;
  uint8_t measurePct = 0;

  if (fallDetected && !fallAlarmMuted)
  {
    msg = "FALL ALERT";
    col = C_RED;
    measurePct = measurementProgressPercent();
  }
  else
  {
    measurePct = measurementProgressPercent();
    msg = measurementProgressText();

    if (!maxOnline || !running)
      col = C_RED;
    else if (measurePct >= 100)
      col = C_GREEN;
    else if (movingTooMuch)
      col = C_YELLOW;
    else if (measurePct >= 50)
      col = C_CYAN;
    else
      col = C_GRAY;
  }

  int16_t x = 46;
  size_t len = strlen(msg);

  if (len <= 10) x = 65;
  else if (len <= 14) x = 49;
  else x = 31;

  textAt(x, 197, msg, col, 1);

  snprintf(buf, sizeof(buf), "MEASURE %u%%", measurePct);
  textAt(80, 211, buf, C_WHITE, 1);

  // Small progress bar.
  const int16_t barX = 55;
  const int16_t barY = 224;
  const int16_t barW = 130;
  const int16_t barH = 7;

  tft->drawRect(barX, barY, barW, barH, C_DARK);

  int16_t fillW =
    (int16_t)(((uint32_t)(barW - 2) * measurePct) / 100UL);

  if (fillW > 0)
    tft->fillRect(barX + 1, barY + 1, fillW, barH - 2, col);
}


// ======================================================
// CALIBRATION DISPLAY
// ======================================================
//
// runCalibration() is a blocking loop, so the normal updateUI()
// function does not run while calibration is active.
// This helper updates the screen directly from the calibration loop.
//
const char *calibrationHint(PulseExpressBpStatus s)
{
  switch (s)
  {
    case PulseExpressBpStatus::NoSignal:
    case PulseExpressBpStatus::NoContact:
    case PulseExpressBpStatus::NoFinger:
      return "PLACE FINGER";

    case PulseExpressBpStatus::Motion:
      return "HOLD STILL";

    case PulseExpressBpStatus::InProgress:
    case PulseExpressBpStatus::EstimationRetry:
      return "MEASURING";

    case PulseExpressBpStatus::Success:
      return "COMPLETE";

    case PulseExpressBpStatus::WeakSignal:
      return "WEAK SIGNAL";

    case PulseExpressBpStatus::EstimationFailure:
      return "CAL FAILED";

    default:
      return bpStatusName(s);
  }
}

void drawCalibrationBase()
{
  if (!displayOK)
    return;

  tft->fillScreen(C_BLACK);
  tft->drawCircle(120, 120, 117, C_CYAN);
  tft->drawCircle(120, 120, 116, C_DARK);

  textAt(50, 28, "BP CALIBRATION", C_YELLOW, 2);

  const int16_t barX = 38;
  const int16_t barY = 112;
  const int16_t barW = 164;
  const int16_t barH = 14;

  tft->drawRect(barX, barY, barW, barH, C_GRAY);

  textAt(58, 174, "KEEP FINGER STILL", C_GRAY, 1);

  calibrationUiBaseDrawn = true;
}

void showCalibrationScreen(
  uint8_t progress,
  PulseExpressBpStatus status)
{
  if (!displayOK)
    return;

  if (progress > 100)
    progress = 100;

  if (!calibrationUiBaseDrawn)
    drawCalibrationBase();

  char pct[12];
  snprintf(pct, sizeof(pct), "%u%%", progress);

  // Update only the changing areas. This is much lighter on XIAO C3
  // than fillScreen() every 500 ms with software SPI.
  tft->fillRect(68, 65, 110, 36, C_BLACK);

  int16_t pctX = 88;
  if (progress < 10) pctX = 103;
  else if (progress < 100) pctX = 94;

  textAt(pctX, 72, pct, C_WHITE, 3);

  const int16_t barX = 38;
  const int16_t barY = 112;
  const int16_t barW = 164;
  const int16_t barH = 14;

  uint16_t barColor = C_CYAN;

  if (status == PulseExpressBpStatus::Motion)
    barColor = C_YELLOW;
  else if (status == PulseExpressBpStatus::Success)
    barColor = C_GREEN;
  else if (status == PulseExpressBpStatus::EstimationFailure)
    barColor = C_RED;

  // Erase only inside the bar, then refill to the new percentage.
  tft->fillRect(barX + 2, barY + 2, barW - 4, barH - 4, C_BLACK);

  int16_t fillW =
    (int16_t)(((uint32_t)(barW - 4) * progress) / 100UL);

  if (fillW > 0)
    tft->fillRect(barX + 2, barY + 2, fillW, barH - 4, barColor);

  const char *hint = calibrationHint(status);

  tft->fillRect(36, 145, 168, 22, C_BLACK);

  size_t len = strlen(hint);
  int16_t hintX = 65;

  if (len <= 8) hintX = 84;
  else if (len <= 12) hintX = 68;
  else hintX = 50;

  textAt(hintX, 151, hint, barColor, 1);

  // Short raw MAX status only.
  tft->fillRect(46, 187, 160, 17, C_BLACK);

  char rawStatus[30];
  snprintf(
    rawStatus,
    sizeof(rawStatus),
    "MAX: %s",
    bpStatusName(status)
  );

  textAt(54, 190, rawStatus, C_GRAY, 1);
}

void showCalibrationError(const char *msg)
{
  if (!displayOK)
    return;

  tft->fillScreen(C_BLACK);
  tft->drawCircle(120, 120, 117, C_RED);

  textAt(61, 62, "CALIBRATION", C_YELLOW, 2);
  textAt(78, 92, "FAILED", C_RED, 2);

  if (msg && *msg)
    textAt(48, 132, msg, C_WHITE, 1);

  textAt(53, 165, "TRY AGAIN / HOLD STILL", C_GRAY, 1);
}



// ======================================================
// BUZZER / FALL / BUTTON
// ======================================================

void buzzerBeep(uint16_t ms)
{
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

void resetFallDetector()
{
  fallStage = FALL_IDLE;
  fallStageMs = 0;
  fallStillStartMs = 0;
}

void triggerFallAlert()
{
  fallDetected = true;
  fallAlarmMuted = false;
  resetFallDetector();

  Serial.println();
  Serial.println("!!! FALL DETECTED !!!");
  Serial.println("Press BOOT to acknowledge/silence alarm.");

  buzzerBeep(180);
  delay(80);
  buzzerBeep(180);
}

void updateFallDetection(uint32_t now)
{
  if (!mpuOK) return;

  // Ignore fall classifier while BP calibration is actively blocking normal use.
  // Sequence reduces false positives compared with a single acceleration threshold.
  switch (fallStage)
  {
    case FALL_IDLE:
      if (accelG < 0.45f)
      {
        fallStage = FALL_FREEFALL;
        fallStageMs = now;
      }
      break;

    case FALL_FREEFALL:
      // Need an impact within 1.2 s after free-fall.
      if (accelG > 2.20f)
      {
        fallStage = FALL_STILL_CHECK;
        fallStageMs = now;
        fallStillStartMs = 0;
      }
      else if (now - fallStageMs > 1200)
      {
        resetFallDetector();
      }
      break;

    case FALL_IMPACT_WAIT:
      // Kept for enum compatibility; current algorithm transitions directly.
      resetFallDetector();
      break;

    case FALL_STILL_CHECK:
    {
      if (now - fallStageMs < 1200)
        break;

      bool postureStable =
        accelG >= 0.70f &&
        accelG <= 1.30f &&
        gyroDps < 35.0f;

      if (postureStable)
      {
        if (fallStillStartMs == 0)
          fallStillStartMs = now;

        if (now - fallStillStartMs >= 1500)
          triggerFallAlert();
      }
      else
      {
        fallStillStartMs = 0;
      }

      if (now - fallStageMs > 5000)
        resetFallDetector();

      break;
    }
  }

  // Alarm pattern: short double chirp every 2 seconds until acknowledged.
  if (fallDetected && !fallAlarmMuted && now - lastFallBuzzMs >= 2000)
  {
    lastFallBuzzMs = now;
    buzzerBeep(80);
    delay(60);
    buzzerBeep(80);
  }
}

void handleBootButton(uint32_t now)
{
  bool state = digitalRead(PIN_BOOT);

  if (state != lastBootState && now - lastBootDebounceMs >= 35)
  {
    lastBootDebounceMs = now;
    lastBootState = state;

    if (state == LOW)
    {
      bootPressedMs = now;
    }
    else
    {
      uint32_t held = now - bootPressedMs;

      if (fallDetected && !fallAlarmMuted)
      {
        fallAlarmMuted = true;
        Serial.println("[FALL] Alarm acknowledged by BOOT button.");
        buzzerBeep(50);
      }
      else if (held < 1500)
      {
        uiPage = (uiPage + 1) % 2;
        drawStaticUI();
      }
      else
      {
        // Long press clears a previous acknowledged fall event.
        if (fallDetected)
        {
          fallDetected = false;
          fallAlarmMuted = false;
          resetFallDetector();
          Serial.println("[FALL] Event cleared.");
          buzzerBeep(60);
        }
      }
    }
  }
}

void scanI2C()
{
  Serial.println();
  Serial.println("========== I2C SCAN ==========");

  uint8_t count = 0;

  for (uint8_t a = 1; a < 127; a++)
  {
    Wire.beginTransmission(a);
    uint8_t e = Wire.endTransmission();

    if (e == 0)
    {
      Serial.printf("FOUND 0x%02X", a);

      if (a == 0x55) Serial.print(" <- MAX32664D");
      if (a == 0x68 || a == 0x69) Serial.print(" <- MPU6050");

      Serial.println();
      count++;
    }
  }

  Serial.printf("TOTAL FOUND = %u\n", count);
  Serial.println("==============================");
  Serial.println("Auto measurement status prints every 10 seconds. Display shows small progress/status.");
}


// ======================================================
// MPU6050 HELPERS
// ======================================================

bool i2cPresent(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool mpuWrite(uint8_t reg, uint8_t value)
{
  if (!mpuAddr) return false;
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadReg(uint8_t reg, uint8_t &value)
{
  if (!mpuAddr) return false;

  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(mpuAddr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool initMpu()
{
  mpuOK = false;
  mpuAddr = 0;

  if (i2cPresent(MPU_ADDR_A)) mpuAddr = MPU_ADDR_A;
  else if (i2cPresent(MPU_ADDR_B)) mpuAddr = MPU_ADDR_B;

  if (!mpuAddr)
  {
    Serial.println("[MPU] NOT FOUND");
    return false;
  }

  uint8_t who = 0;
  if (!mpuReadReg(0x75, who)) return false;

  Serial.printf("[MPU] addr=0x%02X WHO_AM_I=0x%02X\\n", mpuAddr, who);

  if (!mpuWrite(0x6B, 0x00)) return false;
  delay(20);
  if (!mpuWrite(0x1A, 0x03)) return false;
  if (!mpuWrite(0x19, 0x09)) return false;
  if (!mpuWrite(0x1B, 0x08)) return false;
  if (!mpuWrite(0x1C, 0x08)) return false;

  mpuOK = true;
  Serial.println("[MPU] READY");
  return true;
}

bool readMpu()
{
  if (!mpuAddr) return false;

  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) return false;

  int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());

  Wire.read(); Wire.read();

  int16_t gx = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t gy = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read());

  float xg = ax / 8192.0f;
  float yg = ay / 8192.0f;
  float zg = az / 8192.0f;
  accelG = sqrtf(xg*xg + yg*yg + zg*zg);

  float xd = gx / 65.5f;
  float yd = gy / 65.5f;
  float zd = gz / 65.5f;
  gyroDps = sqrtf(xd*xd + yd*yd + zd*zd);

  // MPU is ONLY a warning for UI. It does not change MAX32664 filter logic.
  movingTooMuch = (fabsf(accelG - 1.0f) > 0.20f) || (gyroDps > 45.0f);
  return true;
}

// ======================================================
// TEXT HELPERS
// ======================================================

const char *statusName(PulseExpressStatus s)
{
  switch (s)
  {
    case PulseExpressStatus::Ok:                  return "OK";
    case PulseExpressStatus::IllegalIndex:        return "ILLEGAL_INDEX";
    case PulseExpressStatus::IllegalByteCount:    return "ILLEGAL_BYTE_COUNT";
    case PulseExpressStatus::IllegalConfig:       return "ILLEGAL_CONFIG";
    case PulseExpressStatus::NotInAppMode:        return "NOT_IN_APP_MODE";
    case PulseExpressStatus::DeviceBusy:          return "DEVICE_BUSY";
    case PulseExpressStatus::UnknownHubError:     return "UNKNOWN_HUB_ERROR";
    case PulseExpressStatus::HostCommError:       return "HOST_COMM_ERROR";
    case PulseExpressStatus::UnsupportedFirmware: return "UNSUPPORTED_FW";
    case PulseExpressStatus::Timeout:             return "TIMEOUT";
    case PulseExpressStatus::InvalidArgument:     return "INVALID_ARGUMENT";
    case PulseExpressStatus::BufferTooSmall:      return "BUFFER_TOO_SMALL";
    case PulseExpressStatus::NoDataAvailable:     return "NO_DATA";
    case PulseExpressStatus::NotConfigured:       return "NOT_CONFIGURED";
    default:                                      return "UNKNOWN";
  }
}

const char *bpStatusName(PulseExpressBpStatus s)
{
  switch (s)
  {
    case PulseExpressBpStatus::NoSignal:              return "NO_SIGNAL";
    case PulseExpressBpStatus::InProgress:            return "IN_PROGRESS";
    case PulseExpressBpStatus::Success:               return "SUCCESS";
    case PulseExpressBpStatus::WeakSignal:            return "WEAK_SIGNAL";
    case PulseExpressBpStatus::Motion:                return "MOTION";
    case PulseExpressBpStatus::EstimationFailure:     return "ESTIMATION_FAIL";
    case PulseExpressBpStatus::CalibrationPartial:    return "CAL_PARTIAL";
    case PulseExpressBpStatus::SubjectInitFailure:    return "SUBJECT_INIT_FAIL";
    case PulseExpressBpStatus::InitCompleted:         return "INIT_COMPLETED";
    case PulseExpressBpStatus::RefBpTrendingError:    return "REF_BP_ERROR";
    case PulseExpressBpStatus::RefInconsistency1:     return "REF_INCONSISTENCY_1";
    case PulseExpressBpStatus::RefInconsistency2:     return "REF_INCONSISTENCY_2";
    case PulseExpressBpStatus::RefInconsistency3:     return "REF_INCONSISTENCY_3";
    case PulseExpressBpStatus::RefCountMismatch:      return "REF_COUNT_MISMATCH";
    case PulseExpressBpStatus::RefOutOfLimits:        return "REF_OUT_OF_LIMITS";
    case PulseExpressBpStatus::TooManyCalibrations:   return "TOO_MANY_CAL";
    case PulseExpressBpStatus::PulsePressureOutRange: return "PULSE_PRESSURE_RANGE";
    case PulseExpressBpStatus::HrOutOfRange:          return "HR_OUT_OF_RANGE";
    case PulseExpressBpStatus::HrAboveResting:        return "HR_ABOVE_RESTING";
    case PulseExpressBpStatus::PerfusionOutOfRange:   return "PERFUSION_RANGE";
    case PulseExpressBpStatus::EstimationRetry:       return "RETRY";
    case PulseExpressBpStatus::EstimateOutOfRefRange: return "BP_OUT_OF_REF_RANGE";
    case PulseExpressBpStatus::EstimateOutOfMaxLimit: return "BP_OVER_MAX_LIMIT";
    case PulseExpressBpStatus::NoContact:             return "NO_CONTACT";
    case PulseExpressBpStatus::NoFinger:              return "NO_FINGER";
    default:                                          return "UNKNOWN";
  }
}

void printStatus(const char *label, PulseExpressStatus s)
{
  Serial.print(label);
  Serial.print(" = 0x");
  Serial.print((uint8_t)s, HEX);
  Serial.print(" (");
  Serial.print(statusName(s));
  Serial.println(")");
}

// ======================================================
// VALIDATION
// ======================================================

bool validHR(float v)
{
  return isfinite(v) && v >= HR_MIN && v <= HR_MAX;
}

bool validSpO2(float v)
{
  return isfinite(v) && v >= SPO2_MIN && v <= SPO2_MAX;
}

bool validBP(int sys, int dia)
{
  if (sys < SYS_MIN || sys > SYS_MAX) return false;
  if (dia < DIA_MIN || dia > DIA_MAX) return false;
  if (sys <= dia) return false;

  int pp = sys - dia;
  if (pp < 10 || pp > 120) return false;

  return true;
}

bool noFingerStatus(PulseExpressBpStatus s)
{
  return s == PulseExpressBpStatus::NoSignal ||
         s == PulseExpressBpStatus::NoContact ||
         s == PulseExpressBpStatus::NoFinger;
}

bool badSignalStatus(PulseExpressBpStatus s)
{
  return s == PulseExpressBpStatus::WeakSignal ||
         s == PulseExpressBpStatus::Motion ||
         s == PulseExpressBpStatus::EstimationFailure ||
         s == PulseExpressBpStatus::SubjectInitFailure ||
         s == PulseExpressBpStatus::RefBpTrendingError ||
         s == PulseExpressBpStatus::RefInconsistency1 ||
         s == PulseExpressBpStatus::RefInconsistency2 ||
         s == PulseExpressBpStatus::RefInconsistency3 ||
         s == PulseExpressBpStatus::RefCountMismatch ||
         s == PulseExpressBpStatus::RefOutOfLimits ||
         s == PulseExpressBpStatus::TooManyCalibrations ||
         s == PulseExpressBpStatus::PulsePressureOutRange ||
         s == PulseExpressBpStatus::HrOutOfRange ||
         s == PulseExpressBpStatus::HrAboveResting ||
         s == PulseExpressBpStatus::PerfusionOutOfRange ||
         s == PulseExpressBpStatus::EstimateOutOfRefRange ||
         s == PulseExpressBpStatus::EstimateOutOfMaxLimit;
}

void clearFilters()
{
  hrFiltered = 0;
  spo2Filtered = 0;
  sysFiltered = 0;
  diaFiltered = 0;

  hrReady = false;
  spo2Ready = false;
  bpReady = false;

  hrCount = 0;
  spo2Count = 0;
  bpCount = 0;
}

void clearUiMeasurementCache()
{
  uiHr = 0.0f;
  uiSpO2 = 0.0f;
  uiSys = 0;
  uiDia = 0;

  uiHrValid = false;
  uiSpO2Valid = false;
  uiBpValid = false;

  uiHrMs = 0;
  uiSpO2Ms = 0;
  uiBpMs = 0;
}

void updateUiMeasurementCache(
  float hr,
  float spo2,
  PulseExpressBpStatus bpStatus,
  int sys,
  int dia)
{
  uint32_t now = millis();

  bool changed = false;

  if (validHR(hr))
  {
    if (!uiHrValid || fabsf(uiHr - hr) >= 0.5f)
      changed = true;

    uiHr = hr;
    uiHrValid = true;
    uiHrMs = now;
  }

  if (validSpO2(spo2))
  {
    if (!uiSpO2Valid || fabsf(uiSpO2 - spo2) >= 0.1f)
      changed = true;

    uiSpO2 = spo2;
    uiSpO2Valid = true;
    uiSpO2Ms = now;
  }

  if (bpStatus == PulseExpressBpStatus::Success &&
      validBP(sys, dia))
  {
    if (!uiBpValid || uiSys != sys || uiDia != dia)
      changed = true;

    uiSys = sys;
    uiDia = dia;
    uiBpValid = true;
    uiBpMs = now;
  }

  if (changed)
    uiForceRefresh = true;
}

uint8_t measurementProgressPercent()
{
  // This is a UI progress indicator, not a medical accuracy score.
  if (!maxOnline || !running)
    return 0;

  if (uiBpValid && uiHrValid && uiSpO2Valid)
    return 100;

  if (uiHrValid && uiSpO2Valid)
    return 75;

  if (uiHrValid)
    return 50;

  if (fingerPresent)
    return movingTooMuch ? 20 : 25;

  return 0;
}

const char *measurementProgressText()
{
  if (!maxOnline)
    return "MAX OFFLINE";

  if (!running && !haveCal)
    return "NEED BP CAL";

  if (!running)
    return "ESTIMATION OFF";

  if (uiBpValid && uiHrValid && uiSpO2Valid)
    return "READING READY";

  if (uiHrValid && uiSpO2Valid)
    return "WAITING BP";

  if (uiHrValid)
    return "HR FOUND";

  if (!fingerPresent)
    return "PLACE FINGER";

  if (movingTooMuch)
    return "HOLD STILL";

  return "MEASURING";
}

// ======================================================
// FILTERS
// ======================================================

void updateHR(float v)
{
  if (!validHR(v)) return;

  if (hrCount == 0)
  {
    hrFiltered = v;
    hrCount = 1;
    lastHrMs = millis();
    return;
  }

  // Reject a one-sample jump larger than 30 bpm.
  if (fabsf(v - hrFiltered) > 30.0f) return;

  hrFiltered = 0.22f * v + 0.78f * hrFiltered;

  if (hrCount < 255) hrCount++;
  if (hrCount >= 5) hrReady = true;

  lastHrMs = millis();
}

void updateSpO2(float v)
{
  if (!validSpO2(v)) return;

  if (spo2Count == 0)
  {
    spo2Filtered = v;
    spo2Count = 1;
    lastSpO2Ms = millis();
    return;
  }

  // Reject a sudden 1-sample jump >6%.
  if (fabsf(v - spo2Filtered) > 6.0f) return;

  spo2Filtered = 0.18f * v + 0.82f * spo2Filtered;

  if (spo2Count < 255) spo2Count++;
  if (spo2Count >= 5) spo2Ready = true;

  lastSpO2Ms = millis();
}

void updateBP(int sys, int dia)
{
  if (!validBP(sys, dia)) return;

  if (bpCount == 0)
  {
    sysFiltered = sys;
    diaFiltered = dia;
    bpCount = 1;
    lastBpMs = millis();
    return;
  }

  if (fabsf((float)sys - sysFiltered) > 35.0f) return;
  if (fabsf((float)dia - diaFiltered) > 25.0f) return;

  sysFiltered = 0.25f * sys + 0.75f * sysFiltered;
  diaFiltered = 0.25f * dia + 0.75f * diaFiltered;

  if (bpCount < 255) bpCount++;
  if (bpCount >= 2) bpReady = true;

  lastBpMs = millis();
}

// ======================================================
// NVS CALIBRATION STORAGE
// ======================================================

bool saveCalibration()
{
  PulseExpressVersion v = hub.version();

  prefs.putUInt("magic", 0x504C5345UL);
  prefs.putUChar("fwMaj", v.major);
  prefs.putUChar("fwMin", v.minor);
  prefs.putUChar("fwPat", v.patch);
  prefs.putUInt("len", (uint32_t)calLen);

  size_t n = prefs.putBytes("vector", calVector, calLen);

  return n == calLen;
}

bool loadCalibration()
{
  if (prefs.getUInt("magic", 0) != 0x504C5345UL)
    return false;

  PulseExpressVersion v = hub.version();

  if (prefs.getUChar("fwMaj", 0) != v.major) return false;
  if (prefs.getUChar("fwMin", 0) != v.minor) return false;
  if (prefs.getUChar("fwPat", 0) != v.patch) return false;

  size_t expected = hub.caps().calibVectorBytes;

  if (expected > CAL_MAX) return false;
  if (prefs.getUInt("len", 0) != expected) return false;
  if (prefs.getBytesLength("vector") != expected) return false;

  size_t n = prefs.getBytes("vector", calVector, expected);
  if (n != expected) return false;

  calLen = expected;
  return true;
}


// ======================================================
// LEGACY FW 40.2.2 CALIBRATION VECTOR LOADER
// ======================================================
//
// Legacy MAX32664D firmware 40.2.2 expects the 824-byte
// BPT calibration vector as one I2C command:
//   0x50 0x04 0x03 + 824 data bytes
//
// ESP32 Wire defaults to 128 bytes, so setup() enlarges
// the buffer to 1024 bytes before I2C use.
//
PulseExpressStatus loadLegacyCalibrationVectorSingleFrame(
  const uint8_t *vec,
  size_t len)
{
  if (vec == nullptr || len != 824)
    return PulseExpressStatus::InvalidArgument;

  Wire.beginTransmission(0x55);

  if (Wire.write((uint8_t)0x50) != 1 ||
      Wire.write((uint8_t)0x04) != 1 ||
      Wire.write((uint8_t)0x03) != 1)
  {
    Wire.endTransmission();
    return PulseExpressStatus::BufferTooSmall;
  }

  size_t wrote = Wire.write(vec, len);

  if (wrote != len)
  {
    Wire.endTransmission();
    return PulseExpressStatus::BufferTooSmall;
  }

  uint8_t tx = Wire.endTransmission();

  if (tx != 0)
    return PulseExpressStatus::HostCommError;

  delay(100);

  size_t got = Wire.requestFrom((uint8_t)0x55, (size_t)1);

  if (got != 1 || !Wire.available())
    return PulseExpressStatus::HostCommError;

  uint8_t hubStatus = Wire.read();

  return (PulseExpressStatus)hubStatus;
}

// ======================================================
// START LIVE ESTIMATION
// ======================================================

bool startLive()
{
  PulseExpressStatus s;

  if (calLen != hub.caps().calibVectorBytes)
  {
    Serial.println("[EST] Invalid calibration vector length.");
    return false;
  }

  if (hub.caps().multiPointCalib)
  {
    s = hub.loadCalibrationVector(0, calVector, calLen);
  }
  else
  {
    // Legacy firmware such as 40.2.2:
    // send all 824 calibration bytes in one I2C frame.
    s = loadLegacyCalibrationVectorSingleFrame(
      calVector,
      calLen
    );
  }

  printStatus("[EST] loadCalibrationVector", s);

  if (s != PulseExpressStatus::Ok)
    return false;

  PulseExpressSpo2Coeffs coeffs;
  coeffs.a = SPO2_A;
  coeffs.b = SPO2_B;
  coeffs.c = SPO2_C;

  s = hub.startEstimation(coeffs);

  printStatus("[EST] startEstimation", s);

  if (s != PulseExpressStatus::Ok)
    return false;

  running = true;
  fingerPresent = false;

  noContactSamples = 0;
  noContactSinceMs = 0;
  resetFiltersOnNextContact = false;

  clearFilters();
  clearUiMeasurementCache();
  uiForceRefresh = true;

  Serial.println();
  Serial.println("======================================");
  Serial.println(" LIVE HR / SpO2 / BP STARTED");
  Serial.println(" Put finger fully on sensor and stay still.");
  Serial.println("======================================");

  return true;
}

// ======================================================
// BP CALIBRATION
// ======================================================

void restoreLiveAfterCalibrationFailure(bool hadSavedCalibrationBefore)
{
  if (!hadSavedCalibrationBefore)
    return;

  Serial.println("[CAL] Keeping previous saved calibration.");

  hub.stop();
  delay(250);

  // Reload the previous 824-byte vector already held in calVector/NVS.
  haveCal = true;

  if (startLive())
    Serial.println("[CAL] Previous calibration restored; live measurement resumed.");
  else
    Serial.println("[CAL] WARNING: could not restore previous live measurement.");

  drawStaticUI();
  lastUiMs = 0;
  uiForceRefresh = true;
}

bool runCalibration(
  int s1, int d1,
  int s2, int d2,
  int s3, int d3)
{
  // If the watch already has a saved calibration, preserve it.
  // A failed re-calibration must not leave normal measurement OFF.
  bool hadSavedCalibrationBefore = haveCal;
  if (!validBP(s1, d1) ||
      !validBP(s2, d2) ||
      !validBP(s3, d3))
  {
    Serial.println("[CAL] Invalid cuff values.");
    return false;
  }

  // User has firmware 40.2.2 => legacy 3-reference flow.
  if (hub.caps().multiPointCalib)
  {
    Serial.println("[CAL] This code path is for legacy firmware such as 40.2.2.");
    return false;
  }

  Serial.println();
  Serial.println("======================================");
  Serial.println(" BP CALIBRATION");
  Serial.println("======================================");
  Serial.printf("Cuff #1 = %d/%d\n", s1, d1);
  Serial.printf("Cuff #2 = %d/%d\n", s2, d2);
  Serial.printf("Cuff #3 = %d/%d\n", s3, d3);
  Serial.println("Keep finger still on the optical sensor.");

  running = false;
  clearFilters();

  PulseExpressStatus stopStatus = hub.stop();
  if (stopStatus != PulseExpressStatus::Ok)
    printStatus("[CAL] stop", stopStatus);

  delay(300);

  PulseExpressLegacyCalibrationRefs refs;

  refs.systolic[0] = (uint8_t)s1;
  refs.systolic[1] = (uint8_t)s2;
  refs.systolic[2] = (uint8_t)s3;

  refs.diastolic[0] = (uint8_t)d1;
  refs.diastolic[1] = (uint8_t)d2;
  refs.diastolic[2] = (uint8_t)d3;

  PulseExpressStatus st = hub.startCalibration(refs);
  printStatus("[CAL] startCalibration", st);

  if (st != PulseExpressStatus::Ok)
  {
    showCalibrationError("START ERROR");
    delay(1200);
    drawStaticUI();
    lastUiMs = 0;
    return false;
  }

  // Normal loop() is blocked during calibration, so show progress here.
  uiPage = 0;
  calibrationUiBaseDrawn = false;

  showCalibrationScreen(
    0,
    PulseExpressBpStatus::NoSignal
  );

  uint32_t startMs = millis();
  uint32_t lastMsg = 0;

  PulseExpressSample sample;

  while (true)
  {
    st = hub.readSample(sample);

    if (st == PulseExpressStatus::Ok)
    {
      if (millis() - lastMsg >= 500)
      {
        lastMsg = millis();

        Serial.print("[CAL] progress=");
        Serial.print(sample.progress);
        Serial.print("% status=");
        Serial.print((uint8_t)sample.bpStatus);
        Serial.print(" ");
        Serial.println(bpStatusName(sample.bpStatus));

        showCalibrationScreen(
          sample.progress,
          sample.bpStatus
        );
      }

      if (sample.bpStatus == PulseExpressBpStatus::Success &&
          sample.progress >= 100)
      {
        break;
      }

      if (sample.bpStatus == PulseExpressBpStatus::EstimationFailure ||
          sample.bpStatus == PulseExpressBpStatus::SubjectInitFailure ||
          sample.bpStatus == PulseExpressBpStatus::TooManyCalibrations ||
          sample.bpStatus == PulseExpressBpStatus::RefCountMismatch ||
          sample.bpStatus == PulseExpressBpStatus::RefOutOfLimits)
      {
        Serial.print("[CAL] Calibration rejected: ");
        Serial.println(bpStatusName(sample.bpStatus));

        showCalibrationError(
          bpStatusName(sample.bpStatus)
        );

        delay(1500);
        drawStaticUI();
        return false;
      }
    }
    else if (st != PulseExpressStatus::NoDataAvailable &&
             st != PulseExpressStatus::DeviceBusy)
    {
      printStatus("[CAL] readSample", st);
    }

    if (millis() - startMs > 120000UL)
    {
      Serial.println("[CAL] TIMEOUT 120 sec.");

      showCalibrationError("TIMEOUT");

      delay(1500);
      drawStaticUI();
      return false;
    }

    delay(20);
  }

  Serial.println("[CAL] 100% complete.");

  size_t written = 0;

  st = hub.readCalibrationVector(
    calVector,
    sizeof(calVector),
    &written
  );

  printStatus("[CAL] readCalibrationVector", st);

  if (st != PulseExpressStatus::Ok)
  {
    showCalibrationError("READ VECTOR ERROR");
    delay(1500);
    drawStaticUI();
    lastUiMs = 0;
    return false;
  }

  calLen = written;

  Serial.print("[CAL] Vector bytes = ");
  Serial.println(calLen);

  if (calLen != hub.caps().calibVectorBytes)
  {
    Serial.println("[CAL] Wrong vector size.");
    showCalibrationError("VECTOR SIZE ERROR");
    delay(1500);
    drawStaticUI();
    lastUiMs = 0;
    return false;
  }

  if (!saveCalibration())
  {
    Serial.println("[CAL] NVS save failed.");
    showCalibrationError("NVS SAVE ERROR");
    delay(1500);
    drawStaticUI();
    lastUiMs = 0;
    return false;
  }

  haveCal = true;

  Serial.println("[CAL] Saved to ESP32 NVS.");

  showCalibrationScreen(
    100,
    PulseExpressBpStatus::Success
  );

  delay(900);

  st = hub.stop();
  if (st != PulseExpressStatus::Ok)
    printStatus("[CAL] stop after calibration", st);

  delay(300);

  drawStaticUI();
  lastUiMs = 0;

  return startLive();
}

// ======================================================
// MEASUREMENT NOTIFICATION
// ======================================================

void notifyMeasurementReady()
{
  if (!fingerPresent)
    return;

  // HR + SpO2: one short beep once per contact/measurement cycle.
  if (hrReady && spo2Ready && !hrSpo2ReadyNotified)
  {
    hrSpo2ReadyNotified = true;

    // Mirror the CONFIRMED filtered values to the display cache.
    // This guarantees the screen gets the same values printed to Serial.
    uiHr = hrFiltered;
    uiSpO2 = spo2Filtered;
    uiHrValid = true;
    uiSpO2Valid = true;
    uiHrMs = millis();
    uiSpO2Ms = millis();
    uiForceRefresh = true;

    Serial.println();
    Serial.println("[MEASURE] HR + SpO2 READY");

    Serial.print("[MEASURE] HR=");
    Serial.print(hrFiltered, 1);
    Serial.print(" bpm | SpO2=");
    Serial.print(spo2Filtered, 1);
    Serial.println(" %");

    buzzerBeep(70);
  }

  // BP: two short beeps once when BP becomes valid.
  if (bpReady && !bpReadyNotified)
  {
    bpReadyNotified = true;

    // Mirror confirmed BP to the display cache immediately.
    uiSys = (int)roundf(sysFiltered);
    uiDia = (int)roundf(diaFiltered);
    uiBpValid = true;
    uiBpMs = millis();
    uiForceRefresh = true;

    Serial.print("[MEASURE] BP READY = ");
    Serial.print(uiSys);
    Serial.print("/");
    Serial.print(uiDia);
    Serial.println(" mmHg");

    buzzerBeep(70);
    delay(90);
    buzzerBeep(70);
  }
}

void printMeasurementStatus()
{
  uint32_t now = millis();

  if (now - lastPrintMs < MEASURE_STATUS_MS)
    return;

  lastPrintMs = now;

  uint8_t pct = measurementProgressPercent();

  Serial.print("[MEASURE] ");
  Serial.print(pct);
  Serial.print("% | ");
  Serial.print(measurementProgressText());

  Serial.print(" | MAX=");
  Serial.print(bpStatusName(lastBpStatus));

  Serial.print(" | HR=");
  if (uiHrValid)
  {
    Serial.print(uiHr, 1);
    Serial.print(" bpm");
  }
  else
  {
    Serial.print("--");
  }

  Serial.print(" | SpO2=");
  if (uiSpO2Valid)
  {
    Serial.print(uiSpO2, 1);
    Serial.print("%");
  }
  else
  {
    Serial.print("--");
  }

  Serial.print(" | BP=");
  if (uiBpValid)
  {
    Serial.print(uiSys);
    Serial.print("/");
    Serial.print(uiDia);
  }
  else
  {
    Serial.print("--/--");
  }

  Serial.println();
}

// ======================================================
// PROCESS HUB SAMPLE
// ======================================================

void processSample(const PulseExpressSample &s)
{
  lastBpStatus = s.bpStatus;
  uint32_t now = millis();

  if (noFingerStatus(s.bpStatus))
  {
    if (noContactSinceMs == 0)
      noContactSinceMs = now;

    if (noContactSamples < 255)
      noContactSamples++;

    // Ignore short NO_SIGNAL / NO_CONTACT glitches.
    // This was the reason the R5.3 display often fell back to 35%.
    if (noContactSamples < CONTACT_LOST_MIN_SAMPLES ||
        now - noContactSinceMs < CONTACT_LOST_MIN_MS)
    {
      return;
    }

    if (fingerPresent)
      Serial.println("[FINGER] REMOVED / NO CONTACT");

    fingerPresent = false;
    uiForceRefresh = true;

    // Keep the last valid values visible.
    // The filters will be reset only when a NEW contact begins.
    resetFiltersOnNextContact = true;

    return;
  }

  // Good/contact packet arrived.
  noContactSamples = 0;
  noContactSinceMs = 0;

  // A real removal followed by a new contact starts a fresh measurement.
  if (!fingerPresent && resetFiltersOnNextContact)
  {
    clearFilters();

    hrSpo2ReadyNotified = false;
    bpReadyNotified = false;

    resetFiltersOnNextContact = false;
  }

  fingerPresent = true;

  // Read whatever valid partial values MAX already has.
  // We cache them for the screen even before the filters become READY.
  float hr = s.heartRate();
  float spo2 = s.spo2();

  updateUiMeasurementCache(
    hr,
    spo2,
    s.bpStatus,
    s.systolic,
    s.diastolic
  );

  if (badSignalStatus(s.bpStatus))
    return;

  updateHR(hr);
  updateSpO2(spo2);

  // BP is accepted into the filtered result only on genuine SUCCESS.
  if (s.bpStatus == PulseExpressBpStatus::Success)
  {
    updateBP(
      s.systolic,
      s.diastolic
    );
  }

  notifyMeasurementReady();
}

// ======================================================
// DRAIN FIFO FAST
// ======================================================

void pollLive()
{
  if (!running) return;

  while (true)
  {
    PulseExpressSample samples[8];
    size_t n = 0;

    PulseExpressStatus st =
      hub.readSamples(samples, 8, &n);

    if (st != PulseExpressStatus::Ok)
    {
      if (st != PulseExpressStatus::NoDataAvailable &&
          st != PulseExpressStatus::DeviceBusy)
      {
        printStatus("[LIVE] readSamples", st);
      }
      return;
    }

    for (size_t i = 0; i < n; i++)
      processSample(samples[i]);

    // FIFO drained enough.
    if (n < 8)
      break;
  }
}

// ======================================================
// PRINT LIVE VALUES
// ======================================================

void printLive()
{
  printMeasurementStatus();
}


// ======================================================
// MAX32664 INIT / RETRY
// ======================================================

bool initMax()
{
  Serial.println();
  Serial.println("[MAX] begin()");
  PulseExpressStatus st = hub.begin();
  printStatus("[MAX] begin", st);

  if (st != PulseExpressStatus::Ok)
  {
    maxOnline = false;
    running = false;
    Serial.println("[MAX] INIT FAILED; other watch functions remain active.");
    return false;
  }

  maxOnline = true;

  PulseExpressVersion v = hub.version();
  Serial.printf("[MAX] Firmware: %u.%u.%u\n", v.major, v.minor, v.patch);

  Serial.print("[MAX] Calibration bytes: ");
  Serial.println(hub.caps().calibVectorBytes);

  haveCal = loadCalibration();

  if (haveCal)
  {
    Serial.println("[CAL] Saved calibration found.");

    if (!startLive())
    {
      Serial.println("[EST] Failed to start saved calibration.");
      running = false;
      return false;
    }
  }
  else
  {
    running = false;
    Serial.println();
    Serial.println("NO SAVED BP CALIBRATION.");
    Serial.println("Measure BP THREE times with a real cuff, then send:");
    Serial.println("CAL SYS1 DIA1 SYS2 DIA2 SYS3 DIA3");
  }

  return true;
}


// ======================================================
// COMMANDS
// ======================================================

void printHelp()
{
  Serial.println();
  Serial.println("========== COMMANDS ==========");
  Serial.println("HELP");
  Serial.println("STATUS");
  Serial.println("SCAN");
  Serial.println("BEEP");
  Serial.println("FALLTEST");
  Serial.println("CAL SYS1 DIA1 SYS2 DIA2 SYS3 DIA3");
  Serial.println("ERASE");
  Serial.println("RESTART");
  Serial.println("==============================");
}

void printSystemStatus()
{
  Serial.println();
  Serial.println("========== SYSTEM ==========");

  Serial.print("Display: ");
  Serial.println(displayOK ? "OK" : "OFF");

  Serial.print("MAX32664: ");
  Serial.println(maxOnline ? "ONLINE" : "OFFLINE");

  if (maxOnline)
  {
    PulseExpressVersion v = hub.version();
    Serial.printf("Firmware: %u.%u.%u\n", v.major, v.minor, v.patch);
    Serial.print("Expected calibration bytes: ");
    Serial.println(hub.caps().calibVectorBytes);
  }

  Serial.print("Saved calibration: ");
  Serial.println(haveCal ? "YES" : "NO");

  Serial.print("Live estimation: ");
  Serial.println(running ? "YES" : "NO");

  Serial.print("MPU6050: ");
  Serial.println(mpuOK ? "OK" : "OFF");

  Serial.printf("Motion: acc=%.2fg gyro=%.1fdps moving=%s\n",
                accelG, gyroDps, movingTooMuch ? "YES" : "NO");


  Serial.print("Fall event: ");
  Serial.println(fallDetected ? (fallAlarmMuted ? "ACKNOWLEDGED" : "ALERT") : "NONE");

  Serial.println("============================");
}

void processCommand(String line)
{
  line.trim();

  if (line.length() == 0)
    return;

  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "HELP")
  {
    printHelp();
    return;
  }

  if (cmd == "STATUS")
  {
    printSystemStatus();
    return;
  }

  if (cmd == "SCAN")
  {
    scanI2C();
    return;
  }


  if (cmd == "BEEP")
  {
    buzzerBeep(120);
    return;
  }

  if (cmd == "FALLTEST")
  {
    triggerFallAlert();
    return;
  }

  if (cmd == "ERASE")
  {
    prefs.clear();
    haveCal = false;
    running = false;
    calLen = 0;
    clearFilters();

    Serial.println("[CAL] Calibration erased.");
    Serial.println("Send RESTART before calibrating again.");
    return;
  }

  if (cmd == "RESTART")
  {
    Serial.println("Restarting...");
    delay(300);
    ESP.restart();
    return;
  }

  if (cmd.startsWith("CAL "))
  {
    if (!maxOnline)
    {
      Serial.println("[CAL] MAX32664 OFFLINE. Fix I2C / 0x55 first.");
      return;
    }

    int s1, d1, s2, d2, s3, d3;

    int matched = sscanf(
      cmd.c_str(),
      "CAL %d %d %d %d %d %d",
      &s1, &d1,
      &s2, &d2,
      &s3, &d3
    );

    if (matched != 6)
    {
      Serial.println("Wrong format.");
      Serial.println("Use: CAL SYS1 DIA1 SYS2 DIA2 SYS3 DIA3");
      return;
    }

    runCalibration(
      s1, d1,
      s2, d2,
      s3, d3
    );

    return;
  }

  Serial.println("Unknown command. Type HELP");
}

void pollSerial()
{
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  processCommand(line);
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(200);

  delay(1200);

  Serial.println();
  Serial.println("================================================");
  Serial.println(" XIAO ESP32-C3 SMARTWATCH - CLEAN CORE R5.7");
  Serial.println(" MAX32664 + MPU6050 + GC9A01 + BUZZER + BOOT + FALL + LIVE UI R5.7");
  Serial.println("================================================");

  Serial.println("MAX RST  -> D2 / GPIO4");
  Serial.println("MAX MFIO -> D1 / GPIO3");
  Serial.println("MAX SDA  -> D4 / GPIO6");
  Serial.println("MAX SCL  -> D5 / GPIO7");
  Serial.println("D0 / GPIO2 -> FREE (reserved for future use)");
  Serial.println("[R5.7 FIX] Fresh timestamp after MAX polling; live values will not be cleared immediately.");

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_BOOT, INPUT_PULLUP);


  // Display first so UI still works if a sensor is missing.
  manualTftReset();
  displayOK = tft->begin();
  Serial.printf("[DISPLAY] begin=%s\n", displayOK ? "OK" : "FAIL");

  if (displayOK)
  {
    tft->setRotation(0);
    tft->setTextWrap(false);

    tft->fillScreen(C_RED);
    delay(180);
    tft->fillScreen(C_GREEN);
    delay(180);
    tft->fillScreen(C_BLACK);

    drawStaticUI();
  }

  // Legacy FW 40.2.2 BP vector needs >827 bytes.
  size_t wireBuf = Wire.setBufferSize(1024);

  Serial.print("[I2C] Wire buffer = ");
  Serial.print(wireBuf);
  Serial.println(" bytes");

  if (wireBuf < 827)
  {
    Serial.println("[I2C] WARNING: buffer too small for 824-byte BP vector.");
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(250);

  delay(300);

  Serial.println("[MPU] init on shared I2C bus");
  initMpu();

  prefs.begin("pulse32664", false);

  initMax();

  // Scan after MAX boot sequence so 0x55 has the best chance to appear.
  scanI2C();

  buzzerBeep(70);

  printHelp();
  printSystemStatus();
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  uint32_t now = millis();

  pollSerial();

  // MAX FIFO must be drained continuously.
  // IMPORTANT:
  // pollLive() can create NEW timestamps such as lastHrMs/uiHrMs
  // and may also spend time beeping when a result becomes ready.
  pollLive();

  // R5.7 FIX:
  // Refresh "now" AFTER pollLive().
  // In R5.6, now was captured BEFORE pollLive(), while pollLive() later did:
  //   uiHrMs = millis();
  // Then code calculated:
  //   now - uiHrMs
  // with unsigned uint32_t.
  // Because uiHrMs could be newer than now, subtraction wrapped to a huge
  // number and immediately marked the brand-new value as stale.
  now = millis();

  handleBootButton(now);

  // MPU at 50 Hz for motion/fall event processing.
  if (mpuOK && now - lastMpuRead >= 20)
  {
    lastMpuRead = now;

    if (!readMpu())
    {
      Serial.println("[MPU] read failed -> OFF");
      mpuOK = false;
      mpuAddr = 0;
      movingTooMuch = false;
      resetFallDetector();
    }
    else
    {
      updateFallDetection(now);
    }
  }

  // Retry MAX without freezing the entire watch.
  if (!maxOnline && now - lastMaxRetryMs >= 10000)
  {
    lastMaxRetryMs = now;
    Serial.println("[MAX] retry...");
    initMax();
  }

  // Retry MPU.
  static uint32_t lastMpuRetry = 0;
  if (!mpuOK && now - lastMpuRetry >= 5000)
  {
    lastMpuRetry = now;
    initMpu();
  }

  // Use a FRESH timestamp for stale checks because functions above may have
  // taken time or created newer measurement timestamps.
  uint32_t staleNow = millis();

  // Clear stale FILTER-READY flags only after their real hold time.
  if (hrReady && (uint32_t)(staleNow - lastHrMs) > 5000UL)
    hrReady = false;

  if (spo2Ready && (uint32_t)(staleNow - lastSpO2Ms) > 5000UL)
    spo2Ready = false;

  if (bpReady && (uint32_t)(staleNow - lastBpMs) > 15000UL)
    bpReady = false;

  // Keep partial/latest display values long enough to read.
  if (uiHrValid && (uint32_t)(staleNow - uiHrMs) > UI_HR_HOLD_MS)
    uiHrValid = false;

  if (uiSpO2Valid && (uint32_t)(staleNow - uiSpO2Ms) > UI_SPO2_HOLD_MS)
    uiSpO2Valid = false;

  if (uiBpValid && (uint32_t)(staleNow - uiBpMs) > UI_BP_HOLD_MS)
    uiBpValid = false;

  updateUI();
  printLive();

  delay(2);
}
