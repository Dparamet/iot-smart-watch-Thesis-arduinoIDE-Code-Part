# IoT Smart Watch — Arduino IDE Code

Thesis project: wearable smart watch วัด HR + SpO2 แสดงบนจอ GC9A01 1.28" round TFT

## Hardware

| Component | Part |
|---|---|
| MCU | Seeed XIAO ESP32-C3 |
| Bio Sensor Hub | Protocentral Pulse Express (MAX32664D + MAX30102) |
| Display | GC9A01 1.28" Round TFT 240×240 |

## Wiring

### MAX32664D → XIAO ESP32-C3
| MAX32664D | XIAO ESP32-C3 | GPIO |
|---|---|---|
| VCC | 3.3V | — |
| GND | GND | — |
| SDA | D4 | GPIO6 |
| SCL | D5 | GPIO7 |
| MFIO | D0 | GPIO2 |
| RESET | D1 | GPIO3 |

### GC9A01 → XIAO ESP32-C3
| GC9A01 | XIAO ESP32-C3 | GPIO |
|---|---|---|
| VCC | 3.3V | — |
| GND | GND | — |
| SCL/SCK | D8 | GPIO8 |
| SDA/MOSI | D10 | GPIO10 |
| CS | D7 | GPIO20 |
| DC | D6 | GPIO21 |
| RST | D3 | GPIO5 |
| BL | D2 | GPIO4 |

## Libraries

ติดตั้งใน Arduino IDE:

1. **Protocentral Pulse Express** — `github.com/Protocentral/protocentral-pulse-express`
   - Sketch → Include Library → Add .ZIP Library
2. **Arduino_GFX_Library** by Moon On Our Nation — Library Manager

## Arduino IDE Settings

- Board: `XIAO_ESP32C3`
- Tools → USB CDC On Boot → **Enabled**
- Upload Speed: 921600

## การใช้งาน

1. Upload sketch ครั้งแรก → วางนิ้วที่ sensor นิ่งๆ ~60-90 วิ (calibrate)
2. ค่า calibration บันทึกใน flash — **boot ครั้งถัดไปไม่ต้อง calibrate ใหม่**
3. วางนิ้วที่ sensor → จอแสดง HR (bpm) + SpO2 (%) อัปเดตทุก 2 วิ
4. ส่ง `r` ทาง Serial Monitor เพื่อ recalibrate ใหม่

## Display Layout

```
      ┌──────────┐
      │          │
      │    72    │  ← HR (red, large)
      │   BPM    │
      │ ──────── │
      │  98.0%   │  ← SpO2 (cyan)
      │   SpO2   │
      └──────────┘
```

## Known Limitations

- MAX32664D Hub FW 40.2.2 ไม่รองรับ Maxim Fast algorithm — ใช้ Protocentral library แทน
- ค่า BP (systolic/diastolic) เป็น placeholder — ต้องวัดจาก cuff จริงเพื่อความแม่นยำ
- SpO2 อาจ pin ที่ 100% — ปรับ `Max32664Spo2Coeffs` ตาม Maxim AN6845

## Branch Strategy

| Branch | ใช้ทำ |
|---|---|
| `main` | production-ready เท่านั้น |
| `dev/feature-display-sensor` | โค้ดปัจจุบัน (MAX32664D + GC9A01) |

## What's Next

- [ ] ทดสอบ display บน hardware จริง
- [ ] ปรับ SpO2 calibration coefficients
- [ ] เพิ่ม BLE/WiFi data sync
- [ ] Merge `dev/feature-display-sensor` → `main` หลัง test ผ่าน
