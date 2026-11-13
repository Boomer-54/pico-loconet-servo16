// SPDX-License-Identifier: MIT
// Projekt: https://github.com/Boomer-54/pico-loconet-servo16
// GCA50/LocoIO-Emulator für den Raspberry Pi Pico (arduino-pico)
// Basis: loconet_servo8-5. Neu: Servos über das Hardware-PWM des RP2040 statt Servo.h,
// dadurch bis zu 16 Servos möglich (Port 1..SERVO_PORTS).
//
// Servo-Befehl = OPC_SW_REQ auf die Adresse eines Servo-Ports (Bit 6 wird nicht ausgewertet):
//   ON  -> Servo fährt zu Pos1 (closed) bzw. Pos2 (thrown), mit Geschwindigkeit V
//   OFF -> (kommt ohne Bit 6, wird über die Adresse zugeordnet) Servo wird nach
//          Erreichen der Endlage abgeschaltet
// Port 1..SERVO_PORTS sind Servo-Ports, sobald für sie Pos1/Pos2/V gesetzt sind, sonst normale
// Ein-/Ausgänge (Spule, Melder).
// SERVO_PORTS auf 0 setzen = Verhalten wie der Sketch ohne Servos.
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// ===================== Pins =====================
#define TXPin 0
#define RXPin 1

const uint8_t NPORTS = 16;
// Port 1..16 -> GPIO 2..17. Frei änderbar.
// Nicht verwenden: GPIO 0/1 (LocoNet), GPIO23-25 und 29 (intern auf dem Pico).
// Für Servos gilt zusätzlich: zwei Servo-Ports dürfen nicht auf demselben PWM-Kanal liegen.
// GPIO n hat Slice (n/2)%8, Kanal n%2 – GPIO 2..17 belegen alle 16 Kanäle genau einmal.
const uint8_t pinMap[NPORTS] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17 };

// Ports 1..SERVO_PORTS können Servo-Ports sein (höchstens 16), je nach Konfiguration.
const uint8_t SERVO_PORTS = 16;

// Eingang aktiv = Pin gegen GND gezogen (INPUT_PULLUP), wie im Referenz-Sketch.
// Zeigt Rocrail "belegt/frei" vertauscht, hier auf false setzen.
#define INPUT_ACTIVE_LOW true

#define DEBOUNCE_MS 10
#define PULSE_MS 150

// ===================== Servo-Einstellungen =====================
// Pos1/Pos2/V von Servo-Port 1 liegen in SV 101..103, die von Port 2 in SV 104..106 usw.
// Port 9..16 liegen damit in SV 125..148 (die bietet der GCA50-Dialog nicht an).
#define SERVO_SV_BASE 101

#define SERVO_MIN_US 600     // Impulsbreite bei Pos1 = 127 (ganz nach minus)
#define SERVO_MAX_US 2400    // Impulsbreite bei Pos2 = 127 (ganz nach plus)
#define SERVO_MID_US 1500    // Mitte (Wert 0)
#define SERVO_PERIOD_US 20000  // 50 Hz
#define SERVO_TICK_MS 10     // Takt der Bewegung
// Geschwindigkeit V (0..5): Dauer der Fahrt über den ganzen Weg in ms, V = 0 = sofort.
// ANNAHME: höheres V = langsamer. Ist es bei dir umgekehrt, SERVO_V_HIGHER_IS_SLOWER auf false setzen.
#define SERVO_V_HIGHER_IS_SLOWER true
const uint16_t SERVO_TRAVEL_MS[6] = { 0, 1000, 2000, 4000, 7000, 10000 };
#define SERVO_SETTLE_MS 300  // Wartezeit nach der Endlage, bevor abgeschaltet wird
#define SERVO_CLOSED_IS_POS1 true  // true: closed -> Pos1, thrown -> Pos2

// ===================== SV-Speicher =====================
// SV0 = Version, SV1 = Adresse, SV2 = Subadresse,
// SV3..SV50 = 16 Ports x (config, value1, value2)
uint8_t sv[256];
const uint8_t FW_VERSION = 200;
const uint8_t MAGIC = 0xA5;
bool svDirty = false;
uint32_t svDirtyMs = 0;

#define PORT_CFG(n) (sv[3 + 3 * (n)])
#define PORT_V1(n) (sv[4 + 3 * (n)])
#define PORT_V2(n) (sv[5 + 3 * (n)])
#define SRV_POS1(n) (sv[SERVO_SV_BASE + 3 * (n)])
#define SRV_POS2(n) (sv[SERVO_SV_BASE + 1 + 3 * (n)])
#define SRV_V(n) (sv[SERVO_SV_BASE + 2 + 3 * (n)])

// ===================== LocoNet-Grundlagen =====================
uint32_t lastRxUs = 0;
uint8_t frame[32];
uint8_t frameLen = 0;

// Zustand des Empfangsparsers (global, damit der Sender ihn zurücksetzen kann)
uint8_t rxBuf[32];
uint8_t rxLen = 0, rxNeed = 0;

// Sendewarteschlange
struct TxMsg {
  uint8_t len;
  uint8_t tries;
  uint8_t b[16];
};
const uint8_t TXQ_SIZE = 24;
TxMsg txq[TXQ_SIZE];
uint8_t txHead = 0, txTail = 0;

// ===================== Port-Zustand =====================
uint16_t portAddr[NPORTS];  // logische Adresse der Ports (1-basiert), 0 = keine
bool lastPin[NPORTS];       // zuletzt gemeldeter Pegel der Eingänge
uint32_t inpTimer[NPORTS];  // Zeitpunkt, zu dem der Pin zuletzt dem gemeldeten Zustand entsprach
uint32_t pulseEnd[NPORTS];  // 0 = kein Puls aktiv

// ===================== Servo-Zustand =====================
struct ServoState {
  float cur;           // aktuelle Impulsbreite in us
  float target;        // Ziel-Impulsbreite in us
  float stepUs;        // us pro Takt, 0 = sofort
  uint32_t settleEnd;  // ab hier darf abgeschaltet werden
  bool active;         // Impuls läuft
  bool offPending;     // nach der Endlage abschalten
  bool pwmPin;         // Pin steht auf PWM-Funktion
};
ServoState servo[16];

bool isOutput(uint8_t n) {
  return (PORT_CFG(n) & 0x80) != 0;
}
bool isUsed(uint8_t n) {
  return PORT_CFG(n) != 0x00;
}  // Annahme: Config 0 = nicht benutzt

uint8_t lnChecksum(const uint8_t *m, uint8_t n) {
  uint8_t c = 0xFF;
  for (uint8_t i = 0; i < n; i++) c ^= m[i];
  return c;
}

// ===================== Flash / EEPROM =====================
void svLoad() {
  EEPROM.begin(264);
  if (EEPROM.read(0) == MAGIC) {
    for (int i = 0; i < 256; i++) sv[i] = EEPROM.read(1 + i);
  } else {
    memset(sv, 0, sizeof(sv));
    sv[1] = 81;  // Werksadresse wie beim LocoIO
    sv[2] = 1;   // Subadresse
    svDirty = true;
    svDirtyMs = millis();
  }
  sv[0] = FW_VERSION;
}

void portsInit();  // weiter unten

void svSaveIfIdle() {
  // erst speichern, wenn 1 s nichts geschrieben wurde und der Bus 5 ms still ist
  if (svDirty && millis() - svDirtyMs > 1000 && micros() - lastRxUs > 5000) {
    EEPROM.write(0, MAGIC);
    for (int i = 0; i < 256; i++) EEPROM.write(1 + i, sv[i]);
    EEPROM.commit();
    svDirty = false;
    portsInit();  // neue Port-Konfiguration übernehmen
    Serial.println("SVs gespeichert, Ports neu initialisiert");
  }
}

// ===================== LocoNet empfangen =====================
// true, wenn ein kompletter Frame mit gültiger Prüfsumme in frame[] steht
bool lnReceive() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    lastRxUs = micros();
    if (b & 0x80) {
      rxLen = 0;
      rxNeed = 0;
    }  // Opcode: neuer Frame
    else if (rxLen == 0)
      continue;  // Datenbyte ohne Opcode
    if (rxLen >= sizeof(rxBuf)) {
      rxLen = 0;
      continue;
    }
    rxBuf[rxLen++] = b;
    if (rxLen == 1) {
      switch ((b >> 5) & 3) {
        case 0: rxNeed = 2; break;
        case 1: rxNeed = 4; break;
        case 2: rxNeed = 6; break;
        default: rxNeed = 0;  // variable Länge, steht in Byte 2
      }
    } else if (rxLen == 2 && rxNeed == 0) {
      rxNeed = b;
      if (rxNeed < 3 || rxNeed > sizeof(rxBuf)) {
        rxLen = 0;
        continue;
      }
    }
    if (rxNeed && rxLen == rxNeed) {
      uint8_t n = rxNeed;
      rxLen = 0;
      if (lnChecksum(rxBuf, n - 1) == rxBuf[n - 1]) {
        memcpy(frame, rxBuf, n);
        frameLen = n;
        return true;
      }
    }
  }
  return false;
}

// ===================== LocoNet senden (Warteschlange) =====================
bool lnQueue(const uint8_t *msg, uint8_t len) {
  uint8_t next = (txHead + 1) % TXQ_SIZE;
  if (next == txTail || len > 16) return false;  // Warteschlange voll
  memcpy(txq[txHead].b, msg, len);
  txq[txHead].len = len;
  txq[txHead].tries = 0;
  txHead = next;
  return true;
}

// Sendet höchstens eine Nachricht pro Aufruf, wenn der Bus lange genug still war
void lnTxService() {
  if (txHead == txTail) return;
  // CD-Backoff 20 Bit + Prioritätsverzögerung 6 Bit (1 Bit = 60 us)
  if (micros() - lastRxUs < 1560) return;

  TxMsg &m = txq[txTail];
  bool ok = true;
  for (uint8_t i = 0; i < m.len && ok; i++) {
    Serial1.write(m.b[i]);
    uint32_t t = micros();
    while (!Serial1.available()) {
      if (micros() - t > 3000) {
        ok = false;
        break;
      }  // kein Echo
    }
    if (ok) {
      if (Serial1.read() != m.b[i]) ok = false;  // Echo falsch: Kollision
      lastRxUs = micros();
    }
  }
  if (ok || ++m.tries >= 4) {
    if (!ok) Serial.println("TX aufgegeben");
    txTail = (txTail + 1) % TXQ_SIZE;
  } else {
    while (Serial1.available()) Serial1.read();  // Reste verwerfen
    rxLen = 0;
    lastRxUs = micros();
  }
}

// ===================== Servos (Hardware-PWM) =====================
// Jeder Servo-Pin bekommt einen eigenen PWM-Kanal: 50 Hz, Zählertakt 1 MHz, also 1 Zählschritt = 1 us.
// Impulsbreite 0 = kein Impuls = Servo abgeschaltet (Pin bleibt LOW).
// Der Pin bleibt dauerhaft auf PWM-Funktion, solange der Port ein Servo-Port ist. So gibt es beim
// Abschalten keinen abgeschnittenen Impuls (der neue Wert gilt erst ab der nächsten Periode).

void servoSetUs(uint8_t n, uint16_t us) {
  pwm_set_gpio_level(pinMap[n], us);
}

// Pin auf PWM schalten (Ausgang LOW, solange Impulsbreite 0)
void servoPinToPwm(uint8_t n) {
  if (servo[n].pwmPin) return;
  servoSetUs(n, 0);
  gpio_set_function(pinMap[n], GPIO_FUNC_PWM);
  servo[n].pwmPin = true;
}

// Pin zurück auf normale GPIO-Funktion (für Ein-/Ausgänge)
void servoPinToGpio(uint8_t n) {
  if (!servo[n].pwmPin) return;
  servoSetUs(n, 0);
  gpio_set_function(pinMap[n], GPIO_FUNC_SIO);
  servo[n].pwmPin = false;
}

void servoPwmInit() {
  // Zählertakt 1 MHz, unabhängig vom eingestellten Systemtakt (125/133/200 MHz)
  float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, div);
  pwm_config_set_wrap(&cfg, SERVO_PERIOD_US - 1);

  // Jede benutzte Slice nur einmal initialisieren
  uint8_t done = 0;
  for (uint8_t n = 0; n < SERVO_PORTS; n++) {
    uint slice = pwm_gpio_to_slice_num(pinMap[n]);
    if (!(done & (1 << slice))) {
      pwm_init(slice, &cfg, false);
      pwm_set_both_levels(slice, 0, 0);
      pwm_set_enabled(slice, true);
      done |= (1 << slice);
    }
  }
  // Prüfen, ob zwei Servo-Ports auf demselben PWM-Kanal liegen
  for (uint8_t a = 0; a < SERVO_PORTS; a++) {
    for (uint8_t b = a + 1; b < SERVO_PORTS; b++) {
      if (pwm_gpio_to_slice_num(pinMap[a]) == pwm_gpio_to_slice_num(pinMap[b]) &&
          pwm_gpio_to_channel(pinMap[a]) == pwm_gpio_to_channel(pinMap[b]))
        Serial.printf("WARNUNG: Port %u und %u (GPIO%u/GPIO%u) teilen sich einen PWM-Kanal\n",
                      a + 1, b + 1, pinMap[a], pinMap[b]);
    }
  }
  Serial.printf("PWM: Systemtakt %lu Hz, Teiler %.2f\n", (unsigned long)clock_get_hz(clk_sys), div);

  for (uint8_t n = 0; n < SERVO_PORTS; n++) {
    servo[n].cur = SERVO_MID_US;  // Annahme für die erste Fahrt: Mitte
    servo[n].target = SERVO_MID_US;
    servo[n].stepUs = 0;
    servo[n].settleEnd = 0;
    servo[n].active = false;
    servo[n].offPending = false;
    servo[n].pwmPin = false;
  }
}

// Ein Servo-Port ist nur aktiv, wenn Pos1, Pos2 oder V gesetzt sind.
// So bleiben unkonfigurierte Ports mit alter Adresse ruhig und stören keinen anderen Servo.
bool servoConfigured(uint8_t n) {
  return ((SRV_POS1(n) | SRV_POS2(n) | SRV_V(n)) & 0x7F) != 0;
}

// Port 1..SERVO_PORTS arbeitet als Servo, wenn für ihn Pos1/Pos2/V gesetzt sind.
// Sonst ist er ein normaler Ein-/Ausgang bzw. Spulenausgang.
bool isServoPort(uint8_t n) {
  return n < SERVO_PORTS && servoConfigured(n);
}

float servoTargetUs(uint8_t n, bool pos1) {
  // Wert 0 = Mitte. Pos1 geht von der Mitte nach "minus" (kürzerer Impuls),
  // Pos2 von der Mitte nach "plus" (längerer Impuls), je 0..127.
  if (pos1) return SERVO_MID_US - (float)(SRV_POS1(n) & 0x7F) * (SERVO_MID_US - SERVO_MIN_US) / 127.0f;
  return SERVO_MID_US + (float)(SRV_POS2(n) & 0x7F) * (SERVO_MAX_US - SERVO_MID_US) / 127.0f;
}

void servoMove(uint8_t n, bool pos1) {
  ServoState &s = servo[n];
  uint8_t v = SRV_V(n) & 0x7F;
  if (v > 5) v = 5;
  uint8_t idx = SERVO_V_HIGHER_IS_SLOWER ? v : (v == 0 ? 0 : 6 - v);
  uint16_t travelMs = SERVO_TRAVEL_MS[idx];
  s.target = servoTargetUs(n, pos1);
  s.stepUs = (travelMs == 0) ? 0.0f
                             : (float)(SERVO_MAX_US - SERVO_MIN_US) * SERVO_TICK_MS / (float)travelMs;
  s.offPending = false;
  s.settleEnd = millis() + SERVO_SETTLE_MS;
  if (!s.active) {
    s.active = true;
    servoPinToPwm(n);
    servoSetUs(n, (uint16_t)(s.cur + 0.5f));
  }
  Serial.printf("Servo %u -> %s (%.0f us, V=%u)\n", n + 1, pos1 ? "Pos1" : "Pos2", s.target, v);
}

void servoService() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < SERVO_TICK_MS) return;
  last = now;

  for (uint8_t n = 0; n < SERVO_PORTS; n++) {
    ServoState &s = servo[n];
    if (!s.active) continue;
    if (s.cur != s.target) {
      float d = s.target - s.cur;
      if (s.stepUs <= 0.0f || fabsf(d) <= s.stepUs) s.cur = s.target;
      else s.cur += (d > 0) ? s.stepUs : -s.stepUs;
      servoSetUs(n, (uint16_t)(s.cur + 0.5f));
      if (s.cur == s.target) s.settleEnd = now + SERVO_SETTLE_MS;
    } else if (s.offPending && (int32_t)(now - s.settleEnd) >= 0) {
      servoSetUs(n, 0);  // keine Impulse mehr = Servo abgeschaltet
      s.active = false;
      s.offPending = false;
      Serial.printf("Servo %u abgeschaltet\n", n + 1);
    }
  }
}

// ===================== Ports =====================
void portsInit() {
  for (uint8_t n = 0; n < NPORTS; n++) {
    pulseEnd[n] = 0;
    inpTimer[n] = millis();
    portAddr[n] = 0;
    if (isServoPort(n)) {  // Servo-Port: nur die Adresse übernehmen, die Impulse kommen vom PWM
      if (isUsed(n)) portAddr[n] = ((((uint16_t)PORT_V2(n) & 0x0F) << 7) | (PORT_V1(n) & 0x7F)) + 1;
      servoPinToPwm(n);  // Pin bleibt LOW, solange der Servo nicht läuft
      continue;
    }
    if (n < SERVO_PORTS) {  // war evtl. Servo, ist jetzt ein normaler Port
      if (servo[n].active) {
        servo[n].active = false;
        servo[n].offPending = false;
      }
      servoPinToGpio(n);
    }
    if (isOutput(n)) {
      portAddr[n] = ((((uint16_t)PORT_V2(n) & 0x0F) << 7) | PORT_V1(n)) + 1;
      pinMode(pinMap[n], OUTPUT);
      // Startzustand HIGH, wenn Config-Bit 0 und Bit 3 gleich 0 sind (wie Referenz-Sketch)
      bool high = !(PORT_CFG(n) & 0x01) && !(PORT_CFG(n) & 0x08);
      digitalWrite(pinMap[n], high ? HIGH : LOW);
    } else {
      pinMode(pinMap[n], INPUT_PULLUP);
      lastPin[n] = digitalRead(pinMap[n]);
    }
  }
}

void sendInput(uint8_t n, bool pinHigh) {
  uint8_t m[4];
  bool level = INPUT_ACTIVE_LOW ? !pinHigh : pinHigh;
  uint8_t v2 = PORT_V2(n) & 0x7F;
  if (level) v2 |= 0x10;
  else v2 &= ~0x10;
  m[0] = 0xB2;  // OPC_INPUT_REP
  m[1] = PORT_V1(n) & 0x7F;
  m[2] = v2;
  m[3] = lnChecksum(m, 3);
  lnQueue(m, 4);
}

void inputsScan() {
  for (uint8_t n = 0; n < NPORTS; n++) {
    if (isServoPort(n) || isOutput(n) || !isUsed(n)) continue;
    bool pin = digitalRead(pinMap[n]);
    if (pin == lastPin[n]) {
      inpTimer[n] = millis();
      continue;
    }
    uint32_t stable = millis() - inpTimer[n];
    if (stable < DEBOUNCE_MS) continue;
    // Block-Melder "verzögert" (Config-Bit 4 = 1, Bit 2 = 0): Freimeldung erst nach 2 s
    bool delayed = (PORT_CFG(n) & 0x10) && !(PORT_CFG(n) & 0x04);
    bool deactivating = INPUT_ACTIVE_LOW ? (pin == HIGH) : (pin == LOW);
    if (delayed && deactivating && stable < 2000) continue;
    lastPin[n] = pin;
    sendInput(n, pin);
  }
}

void reportAllInputs() {
  for (uint8_t n = 0; n < NPORTS; n++) {
    if (isServoPort(n) || isOutput(n) || !isUsed(n)) continue;
    sendInput(n, digitalRead(pinMap[n]));
  }
}

// OPC_SW_REQ (0xB0): B0 sw1 sw2 chk
// sw2: Bit 4 = ON, Bit 5 = Richtung (1 = closed), Bit 6 = Servo-Befehl (nur beim ON-Telegramm)
void handleSwitchReq(const uint8_t *m) {
  uint16_t addr = ((((uint16_t)m[2] & 0x0F) << 7) | (m[1] & 0x7F)) + 1;
  bool on = (m[2] & 0x10) != 0;
  uint8_t dir = (m[2] & 0x20) ? 1 : 0;  // 1 = closed, 0 = thrown
  bool servoCmd = (m[2] & 0x40) != 0;

  // Servo-Ports reagieren auf jeden ON/OFF-Befehl mit ihrer Adresse (Bit 6 wird nicht ausgewertet,
  // so funktioniert auch der Test-Button im Einstell-Dialog, der nur den Spulenbefehl sendet)
  bool servoHit = false;
  for (uint8_t n = 0; n < SERVO_PORTS; n++) {
    if (!isServoPort(n) || portAddr[n] != addr) continue;
    servoHit = true;
    if (on) {
      servoMove(n, SERVO_CLOSED_IS_POS1 ? (dir == 1) : (dir == 0));
    } else if (servo[n].active) {
      servo[n].offPending = true;  // nach der Endlage abschalten
    }
  }
  if (servoHit || servoCmd) return;  // Servo-Befehl ohne passenden Servo wird nicht als Spule behandelt

  for (uint8_t n = 0; n < NPORTS; n++) {
    if (isServoPort(n) || !isOutput(n) || portAddr[n] != addr) continue;
    uint8_t cfg = PORT_CFG(n);
    uint8_t portDir = (PORT_V2(n) >> 5) & 1;
    uint8_t pin = pinMap[n];
    if (cfg & 0x08) {  // Puls
      if (portDir == dir && on) {
        digitalWrite(pin, HIGH);
        pulseEnd[n] = millis() + PULSE_MS;
        if (pulseEnd[n] == 0) pulseEnd[n] = 1;
        break;
      }
    } else if (cfg & 0x04) {  // Dauer, Hardware-Reset (folgt ON/OFF)
      if (portDir == dir) {
        digitalWrite(pin, on ? HIGH : LOW);
        break;
      }
    } else if (on) {  // Dauer, Software-Reset (thrown = an, closed = aus)
      digitalWrite(pin, dir ? LOW : HIGH);
      break;
    }
  }
}

void pulseService() {
  for (uint8_t n = 0; n < NPORTS; n++) {
    if (pulseEnd[n] && (int32_t)(millis() - pulseEnd[n]) >= 0) {
      digitalWrite(pinMap[n], LOW);
      pulseEnd[n] = 0;
    }
  }
}

// ===================== SV-Protokoll (OPC_PEER_XFER) =====================
void handlePeerXfer(const uint8_t *m) {
  // m[2]=SRCL m[3]=DSTL m[4]=DSTH m[5]=PXCT1 m[6..9]=D1..D4 m[10]=PXCT2 m[11..14]=D5..D8
  if (m[1] != 0x10) return;

  uint8_t d[8];
  for (uint8_t i = 0; i < 4; i++) {
    d[i] = m[6 + i] | (((m[5] >> i) & 1) << 7);
    d[4 + i] = m[11 + i] | (((m[10] >> i) & 1) << 7);
  }
  uint8_t cmd = d[0], svn = d[1], val = d[3], sub = d[4];
  uint8_t dst = m[3];

  bool forMe = (dst == 0x00 && sub == 0x00) ||   // Broadcast
               (dst == 0x7F && sub == sv[2]) ||  // alle mit dieser Subadresse
               (dst == sv[1] && sub == sv[2]);   // direkt
  if (!forMe) return;

  uint8_t v6 = 0, v7 = 0, v8 = 0;
  if (cmd == 0x02) {  // lesen
    v6 = sv[svn];
    v7 = sv[(uint8_t)(svn + 1)];
    v8 = sv[(uint8_t)(svn + 2)];
    Serial.printf("LESEN SV%u -> %u %u %u\n", svn, v6, v7, v8);
  } else if (cmd == 0x01) {  // schreiben
    if (svn > 0) {           // SV0 (Version) ist schreibgeschützt
      sv[svn] = val;
      svDirty = true;
      svDirtyMs = millis();
    }
    v8 = val;
    Serial.printf("SCHREIBEN SV%u = %u\n", svn, val);
  } else return;

  uint8_t rd[8] = { cmd, svn, FW_VERSION, 0, sv[2], v6, v7, v8 };
  uint8_t r[16];
  uint8_t p1 = 0, p2 = 0;
  r[0] = 0xE5;
  r[1] = 0x10;
  r[2] = sv[1] & 0x7F;  // Quelle: eigene Adresse
  r[3] = m[2];          // Ziel: Absender der Anfrage
  r[4] = m[4];
  for (uint8_t i = 0; i < 4; i++) {
    r[6 + i] = rd[i] & 0x7F;
    p1 |= (rd[i] >> 7) << i;
    r[11 + i] = rd[4 + i] & 0x7F;
    p2 |= (rd[4 + i] >> 7) << i;
  }
  r[5] = p1;
  r[10] = p2;
  r[15] = lnChecksum(r, 15);
  lnQueue(r, 16);
}

// ===================== Arduino =====================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Start");

  svLoad();
  Serial1.setPinout(TXPin, RXPin);

  Serial1.begin(16666);
  Serial.println("LocoNet-Schnittstelle bereit");

  servoPwmInit();
  Serial.println("Servo-PWM bereit");

  portsInit();
  Serial.printf("GCA50-Emulator bereit, Modul %u/%u, Version %u\n", sv[1], sv[2], sv[0]);
  for (uint8_t n = 0; n < NPORTS; n++) {
    if (isServoPort(n)) {
      Serial.printf("Port %2u  GPIO%-2u  Servo  Adresse %u  Pos1=%u Pos2=%u V=%u (SV%u..%u)\n", n + 1,
                    pinMap[n], portAddr[n], SRV_POS1(n), SRV_POS2(n), SRV_V(n),
                    SERVO_SV_BASE + 3 * n, SERVO_SV_BASE + 3 * n + 2);
    } else {
      Serial.printf("Port %2u  GPIO%-2u  cfg=%02X v1=%02X v2=%02X  %s\n", n + 1, pinMap[n],
                    PORT_CFG(n), PORT_V1(n), PORT_V2(n),
                    !isUsed(n) ? "unbenutzt" : (isOutput(n) ? "Ausgang" : "Eingang"));
    }
  }
  for (uint8_t a = 0; a < SERVO_PORTS; a++) {
    for (uint8_t b = a + 1; b < SERVO_PORTS; b++) {
      if (servoConfigured(a) && servoConfigured(b) && portAddr[a] && portAddr[a] == portAddr[b])
        Serial.printf("WARNUNG: Servo %u und %u haben dieselbe Adresse %u\n", a + 1, b + 1, portAddr[a]);
    }
  }
}
//=========================================================
void loop() {
  if (lnReceive()) {
    switch (frame[0]) {
      case 0xE5:
        Serial.print("E5:");
        for (uint8_t i = 0; i < frameLen; i++) Serial.printf(" %02X", frame[i]);
        Serial.println();
        handlePeerXfer(frame);
        break;
      case 0xB0:
        Serial.printf("B0: %02X %02X\n", frame[1], frame[2]);
        handleSwitchReq(frame);
        break;
      case 0x83:  // Gleisspannung an: Eingangszustände melden
        reportAllInputs();
        break;
    }
  }
  inputsScan();
  pulseService();
  servoService();
  lnTxService();
  svSaveIfIdle();
}
