/*
  WD9000 spray servo tester, driven over Wi-Fi.

  The ESP32 camera board relays JSON from TCP port 100 to this UNO at 9600
  baud, so the Mac controls the servos with the web app (python -m wd9000.web)
  or scripts/spray_test.py. The camera stream keeps working.

  Angles are "press angles": 90 = rest (horns level), 0 = pressing, the same
  numbers for both servos. The servos are mirror-mounted (servo 1 left, servo 2 right), so
  servo 2 is inverted by default: raw = 180 - angle. Flip with N:124.

  Commands ({"N":..,"D1":..,"D2":..,"H":"id"}; replies {id_ok}, {id_err}, ...)
    100                       release + stop (the ESP32 sends this when the client drops)
    120  D1=1 | 0             press while resent every < 500 ms (lease) | release
    121  D1=ms                timed press, 50..max_on_ms
    122  D1=servo D2=angle    jog servo 1, 2 or 3 (both) to a press angle
    123  D1=0 | 1             save current positions as rest | press
    124  D1=servo D2=0 | 1    invert servo off | on
    125  D1=ms                max_on_ms (100..30000), hard cap on any press
    126  D1=deg/s             move speed (0..1000, 0 = instant)
    127                       status: {id_st=REST;pos=90,90;...;vcc=..;vmin=..;bat=..;bmin=..}
                              vcc/bat = UNO supply and battery in mV now; vmin/bmin = lowest
                              since the previous status read (catches dips while servos move)
    128                       restore defaults
  Settings live in EEPROM. A lease press that hits max_on_ms latches: further
  presses reply "capped" until a release.

  Wiring
    Servo 1 (left)  -> P10 (pin 10), servo 2 (right) -> P1 (pin 11).
    Final robot: LineTrack header A0/A1, keeping 10/11 for the camera servos.
    Pressing the can needs a separate 5-6 V, >= 3 A servo supply with its
    ground tied to the shield GND.
  Slide switch: "upload" to flash, "cam" to drive it over Wi-Fi.

  Safety: boots to rest; every press is capped at max_on_ms; N:100 (stop or
  client disconnect) releases. The shield's MODE button toggles press/release.
*/

#include <EEPROM.h>
#include <Servo.h>

const uint8_t PIN_SERVO[2] = {10, 11};  // servo headers P10, P1. Final robot: {A0, A1}
const uint8_t PIN_BUTTON = 2;           // shield MODE key, active LOW
const uint8_t MOTOR_PINS[] = {3, 5, 6, 7, 8};  // motor driver inputs, held LOW
const uint8_t PIN_BATTERY = A3;         // battery through the shield's 10k/1.5k divider

const unsigned long LEASE_MS = 500;
const unsigned long MAX_ON_LIMIT_MS = 30000;
const unsigned long DEBOUNCE_MS = 30;
const unsigned long STEP_MS = 10;
const unsigned long POWER_SAMPLE_MS = 3;
const float BATTERY_DIVIDER = (10.0 + 1.5) / 1.5;

// Persistent settings
struct Settings {
  uint16_t magic;
  uint8_t rest[2];    // press angles
  uint8_t press[2];
  uint8_t invert[2];
  uint16_t maxOnMs;
  uint16_t speedDegPerS;  // 0 = instant
};
const uint16_t MAGIC = 0x5D04;
const Settings DEFAULTS = {MAGIC, {90, 90}, {0, 0}, {0, 1}, 2000, 120};
const int EE_SETTINGS = 0;
const int EE_BOOTS = 32;
Settings cfg;

enum State { REST_STATE, PRESS_STATE, MANUAL_STATE };
const char *const STATE_NAMES[] = {"REST", "PRESS", "MANUAL"};

Servo servo[2];
uint8_t target[2];  // press angle each servo is heading to
float raw[2];       // current raw servo angle, ramps toward the target's raw angle
State state = REST_STATE;
unsigned long onSince = 0;
unsigned long leaseUntil = 0;  // >0 while a lease press is running
unsigned long pulseMs = 0;     // >0 while a timed press is running
bool capLatched = false;
uint16_t boots;
uint16_t vccMv, vccMinMv = 0xFFFF, batMv, batMinMv = 0xFFFF;

char token[64];
uint8_t tokenLen = 0;
bool inToken = false;

// UNO supply voltage, measured against the internal 1.1 V bandgap (about +-10% absolute,
// but dips show up reliably).
uint16_t readVccMv() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delayMicroseconds(250);  // let the bandgap settle after the mux change
  for (uint8_t i = 0; i < 2; i++) {  // first conversion after a switch is unreliable
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC)) {}
  }
  return 1125300UL / ADC;  // 1.1 V * 1023 * 1000
}

// Alternate between the supply and the battery; keep the lowest of each.
void samplePower() {
  static unsigned long last = 0;
  static bool battery = false;
  if (millis() - last < POWER_SAMPLE_MS) return;
  last = millis();
  if (battery) {
    analogRead(PIN_BATTERY);  // discard: the mux was on the bandgap
    batMv = analogRead(PIN_BATTERY) * (float)vccMv / 1023.0 * BATTERY_DIVIDER;
    if (batMv < batMinMv) batMinMv = batMv;
  } else {
    vccMv = readVccMv();
    if (vccMv < vccMinMv) vccMinMv = vccMv;
  }
  battery = !battery;
}

uint8_t rawAngle(uint8_t i, uint8_t angle) { return cfg.invert[i] ? 180 - angle : angle; }

void loadSettings() {
  EEPROM.get(EE_SETTINGS, cfg);
  if (cfg.magic != MAGIC) cfg = DEFAULTS;
}

void saveSettings() { EEPROM.put(EE_SETTINGS, cfg); }

void moveTo(const uint8_t pose[2]) {
  target[0] = pose[0];
  target[1] = pose[1];
}

void startPress(unsigned long lease, unsigned long pulse) {
  if (state != PRESS_STATE) onSince = millis();
  state = PRESS_STATE;
  leaseUntil = lease ? millis() + lease : 0;
  pulseMs = pulse;
  moveTo(cfg.press);
}

void release() {
  state = REST_STATE;
  leaseUntil = 0;
  pulseMs = 0;
  moveTo(cfg.rest);
}

// Ramp each servo toward its target at cfg.speedDegPerS.
void updateServos() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < STEP_MS) return;
  float step = cfg.speedDegPerS ? cfg.speedDegPerS * (now - last) / 1000.0 : 180;
  last = now;
  for (uint8_t i = 0; i < 2; i++) {
    float diff = rawAngle(i, target[i]) - raw[i];
    raw[i] += constrain(diff, -step, step);
    servo[i].write((int)(raw[i] + 0.5));
  }
}

void checkPress() {
  if (state != PRESS_STATE) return;
  unsigned long now = millis();
  if (now - onSince >= cfg.maxOnMs) {
    if (leaseUntil) capLatched = true;
    release();
  } else if (leaseUntil && (long)(now - leaseUntil) >= 0) {
    release();
  } else if (pulseMs && now - onSince >= pulseMs) {
    release();
  }
}

// Integer value of "key": in a compact JSON object.
bool jsonInt(const char *json, const char *key, long *out) {
  const char *p = strstr(json, key);
  if (p == NULL) return false;
  p += strlen(key);
  if (!isdigit(*p) && *p != '-') return false;
  *out = atol(p);
  return true;
}

void printPair(const __FlashStringHelper *key, uint8_t a, uint8_t b) {
  Serial.print(key);
  Serial.print(a);
  Serial.print(',');
  Serial.print(b);
}

void printStatus() {
  Serial.print(F("st="));
  Serial.print(STATE_NAMES[state]);
  printPair(F(";pos="), target[0], target[1]);
  printPair(F(";raw="), rawAngle(0, target[0]), rawAngle(1, target[1]));
  printPair(F(";rest="), cfg.rest[0], cfg.rest[1]);
  printPair(F(";press="), cfg.press[0], cfg.press[1]);
  printPair(F(";inv="), cfg.invert[0], cfg.invert[1]);
  Serial.print(F(";max="));
  Serial.print(cfg.maxOnMs);
  Serial.print(F(";spd="));
  Serial.print(cfg.speedDegPerS);
  Serial.print(F(";cap="));
  Serial.print(capLatched);
  Serial.print(F(";boot="));
  Serial.print(boots);
  Serial.print(F(";vcc="));
  Serial.print(vccMv);
  Serial.print(F(";vmin="));
  Serial.print(vccMinMv);
  Serial.print(F(";bat="));
  Serial.print(batMv);
  Serial.print(F(";bmin="));
  Serial.print(batMinMv);
  vccMinMv = vccMv;
  batMinMv = batMv;
}

// Returns the reply value, or NULL for status (printed directly).
const char *runCommand(long n, bool hasD1, long d1, bool hasD2, long d2) {
  switch (n) {
    case 100:
      capLatched = false;
      release();
      return "ok";
    case 120:
      if (!hasD1) return "err";
      if (d1 == 0) {
        capLatched = false;
        release();
        return "ok";
      }
      if (capLatched) return "capped";
      startPress(LEASE_MS, 0);
      return "ok";
    case 121:
      if (!hasD1 || d1 < 50 || d1 > cfg.maxOnMs) return "err";
      startPress(0, d1);
      return "ok";
    case 122:
      if (!hasD1 || !hasD2 || d1 < 1 || d1 > 3 || d2 < 0 || d2 > 180) return "err";
      if (d1 & 1) target[0] = d2;
      if (d1 & 2) target[1] = d2;
      state = MANUAL_STATE;
      leaseUntil = 0;
      pulseMs = 0;
      return "ok";
    case 123:
      if (!hasD1 || d1 < 0 || d1 > 1) return "err";
      memcpy(d1 ? cfg.press : cfg.rest, target, 2);
      saveSettings();
      return "ok";
    case 124:
      if (!hasD1 || !hasD2 || d1 < 1 || d1 > 3 || d2 < 0 || d2 > 1) return "err";
      if (d1 & 1) cfg.invert[0] = d2;
      if (d1 & 2) cfg.invert[1] = d2;
      saveSettings();
      return "ok";
    case 125:
      if (!hasD1 || d1 < 100 || d1 > (long)MAX_ON_LIMIT_MS) return "err";
      cfg.maxOnMs = d1;
      saveSettings();
      return "ok";
    case 126:
      if (!hasD1 || d1 < 0 || d1 > 1000) return "err";
      cfg.speedDegPerS = d1;
      saveSettings();
      return "ok";
    case 127:
      return NULL;
    case 128:
      cfg = DEFAULTS;
      saveSettings();
      capLatched = false;
      release();
      return "ok";
    default:
      return "unknown";
  }
}

void handleToken(char *json) {
  long n, d1 = 0, d2 = 0;
  if (!jsonInt(json, "\"N\":", &n)) return;  // {Factory}, {WA_NO}, ... from the ESP32
  bool hasD1 = jsonInt(json, "\"D1\":", &d1);
  bool hasD2 = jsonInt(json, "\"D2\":", &d2);

  char id[12] = "";
  const char *h = strstr(json, "\"H\":\"");
  if (h) {
    h += 5;
    uint8_t len = 0;
    while (h[len] && h[len] != '"' && len < sizeof(id) - 1) len++;
    memcpy(id, h, len);
    id[len] = '\0';
  }

  const char *value = runCommand(n, hasD1, d1, hasD2, d2);
  if (!id[0] && n != 100) return;  // only tagged commands get replies, like the factory firmware
  Serial.print('{');
  if (id[0]) {
    Serial.print(id);
    Serial.print('_');
  }
  if (value) {
    Serial.print(value);
  } else {
    printStatus();
  }
  Serial.print('}');
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '{') {
      inToken = true;
      tokenLen = 0;
    }
    if (!inToken) continue;
    if (tokenLen < sizeof(token) - 1) {
      token[tokenLen++] = c;
    } else {
      inToken = false;  // too long, drop it
      continue;
    }
    if (c == '}') {
      token[tokenLen] = '\0';
      inToken = false;
      handleToken(token);
    }
  }
}

void readButton() {
  static bool lastStable = HIGH, lastRead = HIGH;
  static unsigned long changedAt = 0;
  bool now = digitalRead(PIN_BUTTON);
  if (now != lastRead) {
    lastRead = now;
    changedAt = millis();
  }
  if (millis() - changedAt > DEBOUNCE_MS && now != lastStable) {
    lastStable = now;
    if (now == LOW) {
      if (state == PRESS_STATE) {
        release();
      } else {
        startPress(0, 0);  // held until pressed again, capped at max_on_ms
      }
    }
  }
}

void setup() {
  for (uint8_t pin : MOTOR_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  loadSettings();

  // Start at rest; write before attach so the first pulse isn't 90.
  moveTo(cfg.rest);
  for (uint8_t i = 0; i < 2; i++) {
    raw[i] = rawAngle(i, target[i]);
    servo[i].write(raw[i]);
    servo[i].attach(PIN_SERVO[i]);
  }

  EEPROM.get(EE_BOOTS, boots);
  EEPROM.put(EE_BOOTS, ++boots);

  vccMv = vccMinMv = readVccMv();
  analogRead(PIN_BATTERY);
  batMv = batMinMv = analogRead(PIN_BATTERY) * (float)vccMv / 1023.0 * BATTERY_DIVIDER;

  Serial.begin(9600);
  Serial.print(F("{spray_boot "));
  Serial.print(boots);
  Serial.print('}');
}

void loop() {
  readSerial();
  readButton();
  checkPress();
  updateServos();
  samplePower();
}
