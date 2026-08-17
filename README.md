# IoT Smart Watch — Arduino IDE Code

Thesis project: wearable smart watch วัดสัญญาณชีพ (HR, SpO2, BP, RR) แสดงบนจอ GC9A01 1.28" round TFT แล้วส่งขึ้น dashboard ผ่าน REST API

โค้ดทั้งหมดอยู่ในไฟล์เดียว: [`testercode.ino`](testercode.ino)

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

> ⚠️ pin mapping ห้ามย้าย — A0-A3 ของ XIAO ESP32-C3 ถูกใช้หมดแล้ว (ยังไม่มีขาเหลือสำหรับวงจรวัดแบต จอจึงแสดง `--%`)

## Libraries

ติดตั้งใน Arduino IDE:

1. **Protocentral Pulse Express** — `github.com/Protocentral/protocentral-pulse-express`
   - Sketch → Include Library → Add .ZIP Library
2. **Arduino_GFX_Library** by Moon On Our Nation — Library Manager
3. **WiFiManager** by tzapu — Library Manager
4. ที่มากับ ESP32 core อยู่แล้ว: `WiFi`, `HTTPClient`, `Preferences`, `Wire`, `time`

## Arduino IDE Settings

- Board: `XIAO_ESP32C3`
- Tools → USB CDC On Boot → **Enabled**
- Upload Speed: 921600

## การตั้งค่า WiFi (WiFiManager — ไม่ hardcode SSID/รหัสในโค้ด)

1. เปิดเครื่องครั้งแรก (หรือ **กดปุ่ม BOOT ค้างไว้ตอนเปิดเครื่อง** เพื่อลืม WiFi เดิม)
2. นาฬิกาปล่อย hotspot ชื่อ **`SmartWatch-Setup`** (รหัส `watch1234`)
3. เอามือถือต่อ AP นั้น แล้วเข้า `http://192.168.4.1`
4. เลือก WiFi + ใส่รหัส — บันทึกลง flash เอง บูตครั้งถัดไปต่ออัตโนมัติ
5. Portal ปิดเองใน 3 นาทีถ้าไม่มีใครตั้งค่า (กันเปิดค้างกินแบต)

มี auto-reconnect: เน็ตหลุดระหว่างใช้งานต่อกลับเองเบื้องหลัง และลอง reconnect ซ้ำก่อนส่งข้อมูลทุกครั้ง

## การใช้งาน

1. Upload sketch ครั้งแรก → วางนิ้วที่ sensor นิ่งๆ ~2 นาที (calibrate BP)
2. ค่า calibration บันทึกใน flash — **boot ครั้งถัดไปไม่ต้อง calibrate ใหม่**
   (calibrate ใหม่: Tools → Erase Flash แล้ว upload ใหม่)
3. วางนิ้วที่ sensor → เริ่มสแกนอัตโนมัติ

```
หน้าหลัก (เวลา NTP + แบต + ค่าล่าสุด)
   │ วางนิ้วบนเซนเซอร์
   ▼
สแกน: เก็บ HR 20 sample + SpO2 ≥ 3 sample (เพดาน 40 วิ)
   │ (HR กับ SpO2 นับแยกกัน — เซนเซอร์รายงานมาคนละจังหวะ)
   ▼
เฉลี่ยค่า → POST ขึ้น API → โชว์ผลบนจอ 3 วิ
   │ นิ้วยังวางอยู่ = เริ่มรอบใหม่อัตโนมัติ (ส่งซ้ำทุก ~13-20 วิ)
   ▼
ยกนิ้ว = กลับหน้าหลัก
```

- **จุดสถานะ API บนจอ** (ข้างไอคอนแบต): เขียว = ส่งสำเร็จ / แดง = fail / เทา = ยังไม่เคยส่ง
- **BP trend บนจอ**: `NORMAL` (เขียว) / `CHECK` (เหลือง) / `DANGER` (แดง) ประเมินจาก HR และ SpO2; ไม่ใช่ค่าความดันหรือคำวินิจฉัยทางการแพทย์
- **RR (อัตราหายใจ)** ประมาณจาก HR÷4 — ไม่ใช่ค่าวัดจริง จอแสดงด้วยเครื่องหมาย `~`
- Debug ละเอียดทุก sample ออกทาง Serial Monitor (115200 baud)

## API

```
POST http://<SERVER_IP>:8000/api/iot/vitals/
Content-Type: application/json
X-API-Key: <key>
```

ตัวอย่าง payload — field ไหนวัดไม่ได้จะไม่ส่ง (backend ตีกลับ 400 ถ้าส่งค่า 0):

```json
{
  "device_id": "WT001",
  "temperature": 36.5,
  "sample_count": 20,
  "partial": false,
  "heart_rate": 81,
  "respiratory_rate": 20,
  "spo2": 97,
  "blood_pressure_sys": 120,
  "blood_pressure_dia": 80,
  "timestamp": "2026-07-10T15:00:00"
}
```

- **ไม่ส่ง `patient_id`** — backend ผูก device → patient เองผ่าน device pairing
- **`temperature` เป็น field บังคับของ backend** — เซนเซอร์วัดไม่ได้ ส่งค่าคงที่ 36.5 ไปก่อน (แผน: ต่อ MLX90614 แล้วใช้ค่าวัดจริง)
- `partial: true` = เก็บ sample ไม่ครบเป้า ความน่าเชื่อถือต่ำกว่าปกติ
- ส่งสำเร็จ = HTTP 2xx (`{"success": true, ...}`) มี retry 2 รอบ (connect timeout 3s / read 5s)

ค่าที่ต้องแก้ตามสภาพแวดล้อม (หัวไฟล์ `testercode.ino`):

```cpp
#define API_URL       "http://172.24.155.69:8000/api/iot/vitals/"  // IP เครื่อง backend
#define API_KEY       "..."                                         // X-API-Key
#define DEVICE_NAME   "WT001"
```

> 🔑 API key ในโค้ดเป็น key ทดสอบ — มีแผนย้ายไป `.env`/config แยกในอนาคต

## Troubleshooting

| อาการ | สาเหตุ / วิธีแก้ |
|---|---|
| `POST -> -1 (connection refused)` | backend ไม่ได้รัน หรือ IP เปลี่ยน (DHCP) — เช็ค `ipconfig` เครื่อง server แก้ `API_URL`, รัน server bind `0.0.0.0` และเปิด firewall port 8000 |
| `POST -> 400` | JSON ขาด field บังคับ (`temperature`) หรือส่งค่า 0 ที่ backend ไม่รับ |
| `POST -> 401` | API key ผิด |
| ส่งได้ครั้งเดียวแล้วเงียบ | WiFi หลุดแล้วไม่ reconnect — แก้แล้วด้วย `WiFi.setAutoReconnect(true)` + reconnect ก่อนส่ง |
| HR ขึ้นแต่ SpO2 ไม่ขึ้น (หรือสลับกัน) | MAX32664 รายงาน HR/SpO2 คนละ sample — แก้แล้วโดยแยกตัวนับ + รอ SpO2 ≥ 3 ตัวก่อนจบรอบ |
| สแกนได้รอบเดียว จอค้าง | hub หยุด stream หลังจบรอบ — แก้แล้วด้วย `hub.stop()` + `startEstimation()` ใหม่ทุกรอบ |
| ต่อ WiFi องค์กรไม่ได้ | เน็ตอาจ whitelist MAC — ดู MAC ใน Serial ตอนบูต เอาไปแจ้ง IT |
| BP trend แสดง `--` | ยังไม่มีค่า HR หรือ SpO2 ที่ใช้ประเมินแนวโน้ม |

## Known Limitations

- MAX32664D Hub FW 40.2.2 ไม่รองรับ Maxim Fast algorithm — ใช้ Protocentral library แทน
- BP บนจอเป็นเพียงแนวโน้มจาก HR/SpO2; ค่า SYS/DIA ดิบยังส่ง API เมื่อ sensor มีข้อมูล และต้องเทียบ cuff จริงก่อนใช้อ้างอิง
- SpO2 อาจ pin ที่ 100% — ปรับ `Max32664Spo2Coeffs` ตาม Maxim AN6845
- `temperature` เป็นค่าคงที่ (placeholder) จนกว่าจะต่อเซนเซอร์อุณหภูมิ
- RR เป็นค่าประมาณจาก HR ไม่ใช่การวัดจริง

## Branch Strategy

| Branch | ใช้ทำ |
|---|---|
| `main` | production-ready เท่านั้น |
| `dev/feature-display-sensor` | โค้ดพัฒนา (MAX32664D + GC9A01 + WiFi/API) |

## What's Next

- [ ] ปรับ SpO2 calibration coefficients
- [ ] ต่อเซนเซอร์อุณหภูมิ (MLX90614) แทนค่า placeholder
- [ ] ย้าย API key ออกจากโค้ดไป config แยก
- [ ] วงจรวัดแบต (voltage divider เข้า ADC)
- [ ] ทำ BP ให้เสถียร
