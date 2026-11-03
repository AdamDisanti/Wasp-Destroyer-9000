# Hardware & Factory Protocol

Source: ELEGOO package `ELEGOO Conqueror Robot Tank Kit 2022.03.23` (kept
locally, gitignored). Facts below come from reading the firmware source;
items marked **unverified** still need testing on our robot.

## Board revisions — check ours

| Part | Variants | How to tell | Our board |
|---|---|---|---|
| Motor driver on shield | **TB6612** (shield V1.1) or **DRV8835** (V1.0) | Chip in the middle of the shield; see `01 ReadMeFirst/ReadMeFirst.pdf` | **unverified** |
| UNO USB-serial chip | CH340 or HT42B534 | Chip next to USB port | **unverified** |
| Camera board | ESP32-WROVER-Camera | Label on board | **unverified** |

The UNO firmware differs per motor driver (`02 Main Program/TB6612` vs `DRV8835`).
Only matters if we ever re-flash the UNO.

## Network (ESP32 factory firmware, `ESP32_CameraServer_AP_20220120`)

- Mode: Wi-Fi access point, **open (no password)**, channel 9
- SSID: `ELEGOO-` + chip MAC in hex (e.g. `ELEGOO-08044920F540`)
- ESP32 IP: `192.168.4.1` (ESP32 soft-AP default, confirmed by its boot log in the Camera FAQ)

| Port | Protocol | Purpose |
|---|---|---|
| 80 | HTTP | **`/Test` — MJPEG stream (verified on our robot)**. Also `/capture` single JPEG, `/control?var=…&val=…` camera settings, `/status` |
| 81 | HTTP | `/stream` — MJPEG per source code, but **refuses connections on our robot** (2026-11-03) |
| 100 | raw TCP | JSON robot commands, relayed to the UNO over serial at 9600 baud |

`/Test` uses the same `stream_handler` as `/stream`. The ESP32 HTTP server
handles one request at a time per port, so while `/Test` is streaming, other
port-80 requests (`/control`, `/capture`) may stall. Our robot is named
`ELEGOO-68EE8F55EBF8`.

Camera default: SVGA 800×600 JPEG. Change size with
`http://192.168.4.1/control?var=framesize&val=N`.

## Command channel (TCP port 100)

- Send JSON objects, e.g. `{"N":3,"D1":3,"D2":150}`. Spaces are stripped.
- Optional `"H":"<id>"` field; the UNO replies `{<id>_ok}` for most commands.
- **Heartbeat:** the ESP32 sends `{Heartbeat}` every 1 s. The client must send
  `{Heartbeat}` back. After >3 s without one, the ESP32 drops the connection
  and sends stop (`{"N":100}`) to the UNO. It also sends stop whenever the
  client disconnects. This is a built-in failsafe for us.

### Commands we care about

| Command | Meaning |
|---|---|
| `{"N":100}` | **Stop everything** / clear mode |
| `{"N":3,"D1":dir,"D2":speed}` | Drive until changed. dir 1=left 2=right 3=forward 4=backward, speed 0–255 |
| `{"N":2,"D1":dir,"D2":speed,"T":ms}` | Drive for a limited time |
| `{"N":4,"D1":left,"D2":right}` | Set left/right track speeds directly (0–255) |
| `{"N":5,"D1":servo,"D2":angle}` | Servo to absolute angle 0–180. servo 1 = pan (pin 10), 2 = tilt (pin 11), 3 = both. **Note: the PDF says D1/D2/D3; the firmware actually reads D1=servo, D2=angle.** |
| `{"N":106,"D1":n}` | Nudge camera ±10°: 1=up 2=down 3=left 4=right. Tilt is limited to 30–110° here |
| `{"N":21,"D1":2}` | Read ultrasonic distance |

Full list: `04 Related chip information/Communication protocol for Conqueror.pdf`.

### Limitation for camera alignment

The UNO servo driver calls a **500 ms blocking delay after every servo
move**, so the stock firmware can do at most about 2 servo updates per second. That's
fine for slow alignment. Faster visual servoing would need a small UNO
firmware change later.

## Shield wiring (from `ConquerorCar-Shield-V1.x` schematics)

| Connector | Device | UNO pins |
|---|---|---|
| UART (4-pin) | ESP32 camera board | 0 RX / 1 TX |
| Servo1 / Servo2 | Pan / tilt servos | 10 / 11 |
| Ultrasonic | HC-SR04 | 13 trig, 12 echo |
| LineTrack | IR line sensor | A0–A2 |
| Left / Right ×2 | Track motors | via driver chip |
| POWER IN | Battery | voltage sensed on A3 |

**Upload/Cam slide switch** (S1, next to the UART port): set it to **cam** for normal
operation so the UNO hears the ESP32. Set it to **upload** only when flashing the UNO.

## Camera LED codes (Camera module FAQ)

- Red LED flashing → poor 4-pin cable connection; reseat it.
- Green LED flashing → no Wi-Fi client connected; solid → client connected.
