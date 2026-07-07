#include <Wire.h>
#include "max32664.h"
#include "Arduino_GFX_Library.h"

// -------- PIN --------
#define HUB_RESET  3
#define HUB_MFIO   2

#define TFT_SCK    8
#define TFT_MOSI   10
#define TFT_CS     20
#define TFT_DC     21
#define TFT_RST    5
#define TFT_BL     4

// -------- COLOR --------
#define C_BLACK 0x0000
#define C_RED   0xF800
#define C_CYAN  0x07FF
#define C_WHITE 0xFFFF
#define C_GRAY  0x8410
#define C_GREEN 0x07E0

// -------- OBJECT --------
Max32664 hub(HUB_RESET, HUB_MFIO);

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

// -------- VAR --------
unsigned long lastUpdate = 0;
unsigned long detectStart = 0;
int secondsFake = 0;

// -------- UI --------
void drawCenter(const char* txt, int y, int size, uint16_t color) {
  int len = strlen(txt);
  int w = len * size * 6;
  int x = (240 - w) / 2;

  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(txt);
}

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
  drawCenter("Place Finger", 100, 2, C_WHITE);
}

void showDetecting() {
  drawCenter("Detecting...", 120, 2, C_GREEN);
}

void showVitals(float hr, float spo2) {
  gfx->fillScreen(C_BLACK);

  char buf[20];

  sprintf(buf, "%d BPM", (int)hr);
  drawCenter(buf, 60, 3, C_RED);

  sprintf(buf, "%.1f %%", spo2);
  drawCenter(buf, 130, 3, C_CYAN);
}

// -------- SETUP --------
void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  gfx->begin();
  gfx->fillScreen(C_BLACK);

  Wire.begin(6, 7);

  if (hub.begin() != Max32664Status::Ok) {
    drawCenter("HUB ERROR", 120, 2, C_RED);
    while (1);
  }

  hub.startEstimation(Max32664Spo2Coeffs{});
}

// -------- LOOP --------
void loop() {

  Max32664Sample sample;
  bool valid = false;
  float hr = 0, spo2 = 0;

  if (hub.readSample(sample) == Max32664Status::Ok) {
    hr = sample.heartRate();
    spo2 = sample.spo2();

    valid = (hr > 40 && hr < 200 && spo2 > 70 && spo2 <= 100);

    Serial.printf("HR: %.1f | SpO2: %.1f\n", hr, spo2);
  }

  // -------- STATE --------
  switch (state) {

    case CLOCK_MODE:
      drawClock();

      if (valid) {
        Serial.println("-> WAIT_FINGER");
        gfx->fillScreen(C_BLACK);
        state = WAIT_FINGER;
      }
      break;

    case WAIT_FINGER:
      showWaiting();

      if (valid) {
        Serial.println("-> DETECTING");
        detectStart = millis();
        state = DETECTING;
      } else {
        state = CLOCK_MODE;
      }
      break;

    case DETECTING:
      showDetecting();

      if (valid) {
        if (millis() - detectStart > 2000) {
          Serial.println("-> SHOW_DATA");
          state = SHOW_DATA;
        }
      } else {
        state = CLOCK_MODE;
      }
      break;

    case SHOW_DATA:
      if (!valid) {
        Serial.println("-> BACK CLOCK");
        gfx->fillScreen(C_BLACK);
        state = CLOCK_MODE;
        break;
      }

      showVitals(hr, spo2);
      break;
  }

  delay(1000);
}