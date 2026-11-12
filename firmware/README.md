# Firmware

The robot runs ELEGOO's factory firmware (kept locally in the gitignored
`ELEGOO Conqueror Robot Tank Kit 2022.03.23/` folder). Sketches here are
bench tools that **temporarily replace the UNO firmware**.

## servo_spray_test

Tests whether two servos can press the spray can's nozzle, **controlled over
the robot's Wi-Fi** from the web app's Spray test tab (or `scripts/spray_test.py`).
The ESP32 camera board relays
the commands to the UNO, and the camera stream keeps working.

- **Servo 1 (left)** -> P10 (pin 10), **servo 2 (right)** -> P1 (pin 11).
  The final robot will use A0/A1; change `PIN_SERVO` in the sketch.
- **Angles are press angles: 90 = rest (horns level), 0 = pressing**, the same numbers for
  both servos. The servos are mirror-mounted, so the firmware inverts servo 2
  (raw angle = 180 - angle). If a servo moves the wrong way, flip it with
  `mirror 1 on` / `mirror 2 off` in the console, no re-flash needed.
- Both servos move together and **hold** position. The factory firmware moves
  one at a time and cuts the signal after 500 ms.

### Power, read this first

Two MG995s can draw over 1 A each when stalled against the nozzle. That is far
more than USB or the UNO's 5 V regulator can supply, and the UNO resets
mid-test. During the UART tests the boot counter reached **390**: the board was
browning out over and over. Wi-Fi control doesn't fix that, only the supply does.

- Power the servos' red/black leads from a separate **5-6 V, >=3 A** supply
  (e.g. a 2S LiPo through a 5-6 V UBEC, or a bench supply).
- Connect that supply's **ground to the shield GND**.
- Plugging a servo straight into a P10/P1 header also powers it from the
  shield's 5 V rail. For the press test, connect only the signal wire to the
  header's signal pin (marked 10 / 11) and power the servo from the separate supply.
- `status` shows `boot #N`. If N goes up during a test, the supply is too weak.

### Upload

1. Set the shield's slide switch (next to the UART port) to **upload**.
2. Connect the UNO by USB, then:
   ```bash
   arduino-cli board list    # find the port (it changes with the USB socket)
   arduino-cli compile --upload -p <port> --fqbn arduino:avr:uno firmware/servo_spray_test
   ```
   Or use the Arduino IDE: open the sketch, Board "Arduino Uno", Upload.
3. Set the slide switch to **cam** so the ESP32 can talk to the UNO.
4. Power the robot, join its Wi-Fi (`ELEGOO-...`), close the ELEGOO phone app.

### Test from the web app (easiest)

```bash
python -m wd9000.web      # then open http://localhost:8000 and click "Spray test"
```

The tab has a hold-to-spray button, a timed press, sliders for each servo with
"Save as rest / press", and the max-press-time and speed settings. It works
without Docker running; only the Camera tab's boxes need it. The page is only
reachable from the Mac running it.

### Test from the terminal

```bash
python scripts/spray_test.py
```

```
both 135 | 1 135 | 2 135   move servo(s) to an angle (no press)
both +5  | 1 -5  | 2 +5    nudge
save rest | save press     store where the servos are now
press 300                  press for 300 ms, then release
on                         press and hold until you hit Enter
off  (or just Enter)       release
status                     positions, saved poses, settings, boot #
max 2000                   hard cap on any press, ms
speed 0                    move speed, deg/s (0 = instant, default 120)
mirror 2 on|off            invert a servo
defaults                   rest 90, press 0, servo 2 mirrored, max 2000, speed 120
```

Defaults are rest 90 and press 0 (the direction that clears the can housing on
our mount). Suggested first run, **can removed**:

1. `both 80`. Both horns should move a little toward the cap, in sync. If
   one goes the other way, flip that servo with `mirror`.
2. `both 0`, check the path, then `both 90`.
3. Fine-tune the press pose (`1 5`, `2 +3`, ...), then `save press`. Same
   for rest if 90 isn't clear of the cap: adjust, then `save rest`.
4. With the can in and the separate supply connected: `speed 0`, then
   `press 300`. Covered nozzle first, then live sprays outdoors.

Saved poses and settings are kept in EEPROM.

### Safety

- Boots to rest.
- Every press is capped at `max` ms (default 2000).
- `on` holds only while the Mac keeps resending (500 ms lease). If the console
  crashes or Wi-Fi drops, the press releases on its own.
- On disconnect the ESP32 sends `{"N":100}`, which releases. So do Ctrl+C and `quit`.
- A hold that hits `max` stays released until you send `off`.
- The shield's MODE button toggles press/release (still capped).

If a servo keeps spinning instead of going to an angle, it's a 360°
continuous-rotation MG995 variant, and you need a 180° positional one.

### Command reference

The sketch's header comment lists the JSON commands (N:120-128). Python
builders: `wd9000.robot.protocol.spray*`. Hold helper:
`wd9000.robot.spray.SprayHold`. The simulator (`python -m wd9000.robot.sim`)
emulates them, so the console can be tried without the robot:
`python scripts/spray_test.py --host 127.0.0.1 --port 10100`.

### Put the factory firmware back

The tank won't work with the phone app or Python until you re-flash it.

**Preferred: restore our exact backup.** The firmware that shipped on our UNO
was read off the board on 2026-11-04 before anything was overwritten. It does
**not** match either `.hex` in the kit download. It lives in `firmware/backup/`
(gitignored, ELEGOO copyright; ask Adam for a copy):

```bash
arduino-cli upload -p <port> --fqbn arduino:avr:uno --input-file firmware/backup/uno_factory_app.hex
```

**Fallback: kit firmware.** Check your shield's motor-driver chip
(`01 ReadMeFirst/ReadMeFirst.pdf`), then flash the matching prebuilt `.hex`:

```bash
KIT="ELEGOO Conqueror Robot Tank Kit 2022.03.23/02 Manual & Main Code & APP/02 Main Program"
# TB6612 shield:
arduino-cli upload -p <port> --fqbn arduino:avr:uno \
  --input-file "$KIT/TB6612/ConquerorCar_TB6612_20220323/ConquerorCar_TB6612_20220323.ino.standard.hex"
# DRV8835 shield:
arduino-cli upload -p <port> --fqbn arduino:avr:uno \
  --input-file "$KIT/DRV8835/ConquerorCar_DRV8835_20220322/ConquerorCar_DRV8835_20220322.ino.standard.hex"
```

Set the slide switch back to **cam** afterwards.
