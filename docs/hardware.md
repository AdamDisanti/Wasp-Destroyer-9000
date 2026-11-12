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
- **Heartbeat (kit source only):** the kit's ESP32 source sends `{Heartbeat}`
  every 1 s, drops clients that stop answering, and sends stop (`{"N":100}`)
  to the UNO when a client leaves.
  **Our robot's ESP32 sends no heartbeats** (observed 2026-11-08: zero in
  10 s, while commands and replies work). Like port 81 being refused, this
  shows it runs different firmware from the kit source. So don't rely on the
  ESP32 failsafe, or on it sending stop when a client disconnects. Safety has
  to come from the UNO side, like the spray firmware's 500 ms lease, which
  releases when resends stop. The client still answers heartbeats if a
  firmware sends them, and watches for them only after it has seen one.

Host-side implementation: `src/wd9000/robot/` (protocol, client, simulator).
Bench verification on the real boards: `scripts/robot_bench.py` (**pending**).

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

The firmware also uses **one** `Servo` object for both ports. It attaches pin
10, then re-attaches to pin 11, and calls `detach()` when done. So the two
servos never move at the same time, and **neither holds position** after the
move. That's acceptable for the camera gimbal, but it can't work for the
two-servo spray actuator. The bench sketch `firmware/servo_spray_test` is the
workaround (see `firmware/README.md`).

## Shield wiring (from `ConquerorCar-Shield-V1.x` schematics)

| Connector | Device | UNO pins |
|---|---|---|
| UART (4-pin) | ESP32 camera board | 0 RX / 1 TX |
| Servo1 / Servo2 | Pan / tilt servos | 10 / 11 |
| Ultrasonic | HC-SR04 | 13 trig, 12 echo |
| LineTrack | IR line sensor | A0–A2 |
| Left / Right ×2 | Track motors | via driver chip |
| POWER IN | Battery | voltage sensed on A3 |

**Power (from the V1.1 schematic, measured 2026-11-08):** the shield has no
5 V regulator. Battery → POWER_IN → SS34 diode → `Vin`, which feeds the TB6612
motor supply directly and the UNO's Vin pin. The shield's `+5V` rail (servo
headers, ESP32 camera board, ultrasonic, LED, IMU) comes from the **UNO's
onboard linear regulator** (~0.8-1 A). Two MG995s moving in free air pulled
that rail from 5.19 V down to 4.41 V and the battery from 7.38 V to 6.28 V,
which reset the ESP32. The spray servos therefore need their own 5-6 V supply
(e.g. a ≥3 A UBEC wired to the battery leads), with grounds joined. The spray
firmware reports supply and battery dips (`vmin`/`bmin` in N:127 status; the
web app's Power chip).

**Upload/Cam slide switch** (S1, next to the UART port): set it to **cam** for normal
operation so the UNO hears the ESP32. Set it to **upload** only when flashing the UNO.

## Camera LED codes (Camera module FAQ)

- Red LED flashing → poor 4-pin cable connection; reseat it.
- Green LED flashing → no Wi-Fi client connected; solid → client connected.

## Spray actuator (planned design, 2026-11-04)

- **Mechanism:** two MG995 servos mounted facing each other above the can, both
  horns pressing the nozzle cap. Top-down, servo 1 is on the left and servo 2
  on the right. They are mirror-mounted (servo 1 turns counterclockwise and
  servo 2 clockwise to press), so the firmware inverts servo 2: both use press
  angles, 90 = rest (horns level) and 0 = pressing (the path that clears the can
  housing). Bench tool: `firmware/servo_spray_test`, driven over Wi-Fi by the
  web app (`python -m wd9000.web`) or `scripts/spray_test.py`. It already implements the
  N:120 lease below, plus N:121-128 for jogging, saving poses and settings.
- **Pins:** spray servos on **A0/A1** (LineTrack header, line sensor removed).
  Camera pan/tilt stays on 10/11.
- **Power:** separate 5–6 V, ≥3 A regulator (UBEC) from the tank battery,
  with grounds tied together. Not the UNO's 5 V.
- **Mount:** can cradle on a pivot at the front of the tank; elevation is set
  **manually** (bolt in an arc slot) before a run, not motorized.
- **Aiming:** camera is **fixed to the can cradle**, so it tilts with the
  elevation setting and a nest at the aim point means the nozzle is on target.
  With elevation fixed during a run, alignment uses the chassis only:
  - **left/right** (`error_x`): turn in place
  - **up/down** (`error_y`): drive forward/back. Moving closer to an overhead
    nest raises it in the frame; moving away lowers it.
  - **Aim point ≠ image center:** the spray arcs downward with range, and the
    camera sits beside the nozzle. Calibrate the aim point (pixel offset) by
    test-spraying a target, and treat it as a function of distance if needed.
- Camera pan/tilt servos are unused if the camera rides on the cradle, which
  leaves pins 10/11 free (e.g. for motorized elevation later).
- **Control path (to build after the bench test):**
  laptop → Wi-Fi → ESP32 (unchanged) → UNO running *modified* factory firmware
  with a new spray command (proposed `{"N":120,"D1":1|0}`):
  - two separate `Servo` objects on A0/A1, always attached, so they hold position
  - short release timer: `D1:1` presses for ~500 ms. The host resends it while
    the operator holds the spray key; the firmware releases when resends stop
  - hard cap on spray duration; `{"N":100}` (stop, also sent by the ESP32 when
    the link drops) releases immediately
  - spray is always operator-triggered. Detections only highlight the target
