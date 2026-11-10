# Pico LocoNet Servo16 – LocoNet-Modul mit 16 Servos für den Raspberry Pi Pico

Ein Raspberry Pi Pico verhält sich am LocoNet kompatibel zu einem **GCA50 / LocoIO** und lässt sich deshalb in **Rocrail** über den vorhandenen GCA50-Einstelldialog konfigurieren. Das Modul hat 16 Ports. Jeder Port kann als **Servo**, **Ausgang** (Spule/Dauer) oder **Eingang** (Melder) arbeiten.

Der Pico kostet nur wenige Euro und steuert bis zu **16 Servos** über sein Hardware-PWM. Ein Modul reicht damit für 16 Weichen.

> [!IMPORTANT]
> **Dies ist kein GCA-Produkt.** „GCA“ und „GCA50“ bezeichnen die Module von Peter Giling. Dieses Projekt ist eine **eigene, unabhängige Umsetzung** und steht in keiner Verbindung zu Peter Giling oder den GCA-Modulen. Es wird von dort weder unterstützt noch geprüft. Es enthält keinen Code der Original-Firmware. Die Bezeichnung GCA50 wird nur verwendet, um die Kompatibilität mit dem Rocrail-Dialog zu beschreiben.
>
> Bei Fragen oder Problemen mit diesem Projekt bitte **nicht** an Peter Giling oder den GCA-Support wenden, sondern hier ein [Issue](../../issues) eröffnen.

---

## Funktionen

- LocoNet direkt über die UART des Pico, **ohne zusätzliche LocoNet-Bibliothek**
  (Kollisionserkennung über Echo, Sendewarteschlange, Wiederholungen)
- SV-Programmierung (LocoIO-Protokoll, OPC_PEER_XFER) – lesen und schreiben aus Rocrail
- 16 Ports (GPIO 2–17), je Port:
  - **Servo**: zwei Endlagen (Pos1/Pos2) und Geschwindigkeit (V = 0–5), Abschaltung des Impulses nach Erreichen der Endlage
  - **Ausgang**: Puls (150 ms), Dauer mit Software- oder Hardware-Reset
  - **Eingang**: entprellt, optional verzögerte Freimeldung (Blockmelder)
- Ports wechseln automatisch zwischen Servo und Ein-/Ausgang, je nachdem ob Servo-Werte gesetzt sind
- Einstellungen werden im Flash des Pico gespeichert
- Ausführliche Diagnose über den seriellen Monitor (115200 Baud)

## Hardware

| Was | Hinweis |
|---|---|
| Raspberry Pi Pico (RP2040) | auch kompatible Boards |
| LocoNet-Anschaltung | Pegelwandler zwischen LocoNet (12 V) und 3,3 V, an GPIO 0 (TX) und GPIO 1 (RX) |
| Servos | z. B. SG90 / MG90S |
| 5-V-Netzteil für die Servos | **nicht** aus dem Pico versorgen; Masse von Netzteil, Pico und LocoNet verbinden |

### Schaltplan

[Schaltplan der LocoNet-Anschaltung (PDF)](loconet-pico.pdf)

> [!NOTE]
> Die Servos brauchen eine externe 5-V-Versorgung. USB reicht nur zum Testen mit höchstens 1–2 Servos.
> Die Masse (GND) von Pico, externer Stromquelle und LocoNet muss verbunden sein.
### Pinbelegung

| Port | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| GPIO | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 |

LocoNet: GPIO 0 = TX, GPIO 1 = RX.

GPIO 2–17 belegen alle 16 PWM-Kanäle des RP2040 genau einmal, deshalb können alle 16 Ports gleichzeitig Servos sein. Wer die Pins in `pinMap[]` ändert, muss darauf achten, dass zwei Servo-Ports nicht auf demselben PWM-Kanal liegen. Der Sketch warnt beim Start im seriellen Monitor.

## Software installieren

1. **Arduino IDE** installieren.
2. Den **Arduino-Core für den RP2040 von Earle Philhower** einbinden:
   *Datei → Einstellungen → Zusätzliche Boardverwalter-URLs*:
   `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
   Danach im Boardverwalter **„Raspberry Pi Pico/RP2040“** installieren.
3. Board **„Raspberry Pi Pico“** auswählen.
4. `loconet_servo16/loconet_servo16.ino` öffnen und hochladen.

Weitere Bibliotheken sind nicht nötig.

## Einrichten in Rocrail

1. Der Pico meldet sich ab Werk mit der Adresse **81/1** (wie ein LocoIO).
2. In Rocrail den **GCA50-Dialog** öffnen, Modul abfragen, Adresse nach Wunsch ändern.
3. Ports konfigurieren:
   - **Servo**: Adresse und Pos1/Pos2/V eintragen. Sobald ein Servo-Wert gesetzt ist, arbeitet der Port als Servo.
   - **Ein-/Ausgang**: wie beim GCA50 üblich. Ports ohne Servo-Werte bleiben normale Ein-/Ausgänge.
4. Etwa 1 Sekunde nach dem letzten Schreibzugriff speichert der Pico die Werte und übernimmt die neue Konfiguration.

### Servo-Werte

| Wert | Bereich | Bedeutung |
|---|---|---|
| Pos1 | 0–127 | 0 = Mitte, 127 = ganz nach „minus“ (Stellung *closed*) |
| Pos2 | 0–127 | 0 = Mitte, 127 = ganz nach „plus“ (Stellung *thrown*) |
| V | 0–5 | Fahrzeit über den ganzen Weg: 0 = sofort, 1 = 1 s, 2 = 2 s, 3 = 4 s, 4 = 7 s, 5 = 10 s |

Fährt der Servo verkehrt herum, lässt sich die Richtung im Weichen-Dialog von Rocrail umdrehen.

### SV-Belegung

| SV | Inhalt |
|---|---|
| 0 | Firmware-Version (nur lesbar) |
| 1 / 2 | Moduladresse / Subadresse |
| 3–50 | 16 Ports × (Config, Wert1, Wert2) – wie beim GCA50 |
| 101–124 | Servo 1–8: je Pos1, Pos2, V |
| 125–148 | Servo 9–16: je Pos1, Pos2, V |

## Bekannte Einschränkung: Servo 9–16 im Rocrail-Dialog

Der GCA50-Dialog in Rocrail kennt nur **8 Servos**. Servo 9–16 lassen sich programmieren und fahren korrekt. Der Dialog meldet aber Fehler (z. B. `sv127 is not supported`) und kann die Werte nicht auslesen und anzeigen.

**Idee:** Das Modul meldet eine eigene Versionsnummer (z. B. 200). Der Dialog erkennt daran die 16-Servo-Variante und zeigt die zusätzlichen Servos an. Ein originaler GCA50 meldet weiterhin seine normale Version und bleibt voll kompatibel. Dazu läuft eine Diskussion im Rocrail-Forum. <!-- TODO: Link zum Forumsthema -->

## Einstellungen im Sketch

Oben im Sketch lassen sich anpassen:

| Konstante | Standard | Zweck |
|---|---|---|
| `SERVO_PORTS` | 16 | wie viele Ports Servos sein dürfen (0 = keine Servos) |
| `SERVO_MIN_US` / `SERVO_MAX_US` | 600 / 2400 | Impulsbreite bei den äußersten Endlagen |
| `SERVO_TRAVEL_MS[]` | 0…10000 | Fahrzeiten für V = 0…5 |
| `SERVO_SETTLE_MS` | 300 | Wartezeit vor dem Abschalten |
| `INPUT_ACTIVE_LOW` | true | Melder aktiv bei Masse |
| `PULSE_MS` | 150 | Pulsdauer der Spulenausgänge |

## Hinweis zu LocoNet

LocoNet® ist eine eingetragene Marke und Technologie von Digitrax, Inc. Dieses Projekt nutzt die öffentlich zugängliche „LocoNet Personal Use Edition“. Sie ist **nur für die private, nicht-kommerzielle Nutzung** freigegeben. Der Nachbau für die eigene Anlage ist erwünscht. Für den **Verkauf** von Geräten (auch bestückter Platinen) mit dieser Firmware ist eine Lizenz von Digitrax erforderlich.

## Marken

GCA ist eine Bezeichnung der Module von Peter Giling. LocoNet® ist eine eingetragene Marke von Digitrax, Inc. Rocrail ist ein Produkt des Rocrail-Teams. Alle Namen werden nur zur Beschreibung der Kompatibilität verwendet. Es besteht keine Verbindung zu den Inhabern.

## Lizenz

Der Code steht unter der [MIT-Lizenz](LICENSE). Nutzung auf eigene Gefahr.

## Danke

- Peter Giling für die GCA-Module, die als Vorbild dienten
- dem Rocrail-Team für Rocrail und den GCA50-Dialog
- Earle Philhower für den Arduino-Core für den RP2040
