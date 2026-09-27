# IoT Smart Watch — Arduino IDE Firmware

Firmware for the Seeed XIAO ESP32-C3 smartwatch prototype. It reads heart rate, SpO₂, and blood pressure from the Protocentral Pulse Express hub, shows live status on a GC9A01 round display, and uses an MPU6050 to detect a possible fall and sound a buzzer.

The sketch is [`testercode.ino`](testercode.ino). This firmware revision is local-only: it does not configure WiFi or send measurements to a backend.

## Hardware

| Component | Connection / role |
|---|---|
| Seeed XIAO ESP32-C3 | Main controller |
| Protocentral Pulse Express (MAX32664D + MAX30102) | HR, SpO₂, and BP hub |
| GC9A01 240×240 round TFT | Vitals and system status display |
| MPU6050 | Motion and possible-fall detection; shares the I²C bus |
| Buzzer | Audible fall alert |

### Pin map

| Signal | XIAO pin | GPIO |
|---|---:|---:|
| Pulse Express RESET | D2 | 4 |
| Pulse Express MFIO | D1 | 3 |
| I²C SDA (Pulse Express + MPU6050) | D4 | 6 |
| I²C SCL (Pulse Express + MPU6050) | D5 | 7 |
| TFT SCK | D8 | 8 |
| TFT MOSI | D10 | 10 |
| TFT DC | D6 | 21 |
| TFT RST | D7 | 20 |
| TFT CS | GND | — |
| Buzzer | D3 | 5 |
| BOOT button | D9 | 9 |

The sketch probes MPU6050 addresses `0x68` and `0x69`. GPIO2 (D0) is currently unused. There is no battery measurement in this firmware.

## Required libraries

Install these in Arduino IDE:

1. **Protocentral Pulse Express** library from [Protocentral's repository](https://github.com/Protocentral/protocentral-pulse-express).
2. **Arduino_GFX_Library** by Moon On Our Nation.

`Arduino.h`, `Wire`, `Preferences`, and `math.h` are provided by the ESP32 Arduino core/toolchain.

## Build and upload

1. Open `testercode.ino` in Arduino IDE.
2. Select the Seeed XIAO ESP32-C3 board (`XIAO_ESP32C3`).
3. Set **USB CDC On Boot** to **Enabled**.
4. Build and upload the sketch.
5. Open Serial Monitor at **115200 baud** and select **Newline** as the line ending.

## First-time BP calibration

Measure blood pressure three times using a real cuff before calibration. Keep the same person seated and still, place a finger fully on the optical sensor, then enter the three SYS/DIA pairs in Serial Monitor:

```text
CAL SYS1 DIA1 SYS2 DIA2 SYS3 DIA3
```

Example format:

```text
CAL 121 78 119 77 120 79
```

The example numbers are illustrative; enter that person's actual cuff readings. The hub calibration can take up to 120 seconds. A successful vector is saved in ESP32 NVS and loaded on later boots.

## Display and controls

- The vitals page shows valid HR, SpO₂, and BP values as they arrive. A `*` marks a partial value while the filters collect enough samples.
- The system page shows sensor and motion status; use the BOOT button to switch pages or interact with the alert as indicated on screen.
- Fall detection looks for a sequence of free-fall, impact, then post-impact stillness. A detected event sounds the buzzer. Use `FALLTEST` to check the alert output.
- These readings and fall alerts are prototype functions. They are not medical measurements, a diagnosis, or a substitute for emergency response.

## Serial commands

Enter commands in Serial Monitor at 115200 baud with Newline line ending:

| Command | Action |
|---|---|
| `HELP` | List available commands |
| `STATUS` | Print sensor, calibration, and motion status |
| `SCAN` | Scan the I²C bus |
| `BEEP` | Test the buzzer |
| `FALLTEST` | Trigger a fall-alert test |
| `CAL SYS1 DIA1 SYS2 DIA2 SYS3 DIA3` | Run BP calibration using three cuff readings |
| `ERASE` | Erase saved BP calibration from NVS |
| `RESTART` | Restart the ESP32 |

After `ERASE`, send `RESTART` before calibrating again.

## Troubleshooting

| Symptom | Checks |
|---|---|
| MAX32664D offline or no vitals | Check 3.3V/GND, RESET on D2/GPIO4, MFIO on D1/GPIO3, and I²C on D4/D5. Run `SCAN` and `STATUS`. |
| MPU6050 not detected | Check shared SDA/SCL wiring and try address `0x68` or `0x69`; run `SCAN`. |
| TFT blank | Check 3.3V/GND, SCK/MOSI/DC/RST pins, and ensure CS is tied to GND. |
| Calibration rejected or times out | Recheck the three cuff measurements, keep the finger fully placed, and stay still during calibration. |
| No buzzer sound | Check buzzer polarity and the D3/GPIO5 connection; run `BEEP`. |

## Limitations

- BP depends on reference cuff calibration and the Pulse Express algorithm; validate it against a cuff. It is not suitable for clinical use.
- Fall detection uses an accelerometer/gyroscope event sequence and requires real-world validation for the final enclosure and wearing position.
- The firmware currently has no WiFi, backend upload, temperature sensor, or battery gauge.
- SpO₂ accuracy depends on the sensor optics, fit, motion, and calibration.
