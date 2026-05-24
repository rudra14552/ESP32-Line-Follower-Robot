# 🤖 Roshogulla — PID Line Follower Robot

> ESP32-based PID line follower with Bluetooth tuning, NVS memory, and auto-stop — built around the DRV8833 motor driver and a Waveshare 5-channel IR sensor array.

---
## Demo 



https://github.com/user-attachments/assets/0787e2dc-1a41-4838-8cc5-ae2d504d8008





## Features

- **PID control** with non-linear Kp scaling — handles both gentle curves and sharp turns
- **Dynamic speed reduction** on tight corners for smoother tracking
- **Live Bluetooth tuning** — adjust Kp, Ki, Kd, and speed from your phone without reflashing
- **NVS flash storage** — calibration, PID gains, and speed survive power-off
- **Auto-stop** if the line is lost for more than 1 second
- **Line recovery** — spins toward last known side during brief loss
- **Two physical buttons** — calibrate and run/stop without a phone
- **LED status indicator** — blinks during calibration, solid while following

---

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32 (any standard 38-pin dev board) |
| Motor driver | DRV8833 dual H-bridge |
| IR sensor | Waveshare 5-channel analog tracker (SmartElex compatible) |
| Battery | 2S LiPo (7.4 V nominal, 8.4 V fully charged) |
| Power regulation | Mini buck converter — 2S LiPo in, 3.3 V out |
| Motors | 2× N20 gear motor, 600 RPM |
| Controls | 2× momentary push buttons |
| Indicator | 1× LED |

---

## Pin Map

### IR Sensor Array

| Sensor | ESP32 pin | Position |
|--------|-----------|----------|
| IR1 | G33 | Leftmost |
| IR2 | G32 | Left-centre |
| IR3 | G35 | Centre |
| IR4 | G34 | Right-centre |
| IR5 | G4 | Rightmost |

> G35 and G34 are input-only pins — no internal pull-up. The sensor drives them actively so this is fine.

### Motor Driver (DRV8833)

| ESP32 pin | DRV8833 pin | Motor |
|-----------|-------------|-------|
| G25 | IN1 | Left A |
| G26 | IN2 | Left B |
| G27 | IN3 | Right A |
| G14 | IN4 | Right B |

**Logic:** Forward = `(PWM, 0)` · Reverse = `(0, PWM)` · Brake = `(0, 0)`  
**PWM:** 5 kHz, 8-bit resolution via `ledcAttach`

> Tie the DRV8833 `nSLEEP` pin to 3.3 V to keep the driver active.

### Controls & Indicator

| Component | ESP32 pin | Mode |
|-----------|-----------|------|
| Calibration button | G18 | INPUT_PULLUP (press = LOW) |
| Run/stop button | G19 | INPUT_PULLUP (press = LOW) |
| LED | G5 | OUTPUT |

### Power

| Connection | Details |
|------------|---------|
| 2S LiPo → Buck Vin | 7.4 V nominal (8.4 V max) |
| Buck Vout → ESP32 VIN | 3.3 V regulated |
| Buck Vout → Sensor VCC | 3.3 V regulated |
| 2S LiPo → DRV8833 VM | Direct — motors run at LiPo voltage |
| GND | Common rail — LiPo, ESP32, DRV8833, sensor, buck |

> **LiPo safety:** Never discharge below 3.0 V per cell (6.0 V total). Use a LiPo alarm or checker. The DRV8833 is rated up to 10.8 V — a fully charged 2S at 8.4 V is well within spec.

---

## Bluetooth Commands

Connect to **"Roshogulla"** with PIN **080326** using any Bluetooth serial app (e.g. Serial Bluetooth Terminal on Android).

| Command | Action |
|---------|--------|
| `c` | Start 10-second calibration |
| `s` | Start line following |
| `x` | Stop |
| `?` | Print status (PID values, speed, calibration state) |
| `P2.5` | Set Kp to 2.5 |
| `I0.01` | Set Ki to 0.01 |
| `D1.8` | Set Kd to 1.8 |
| `UP35` | Set speed to 35% |
| `DOWN20` | Set speed to 20% |
| `reset` | Reset PID and speed to defaults |

All PID and speed changes are saved to NVS flash immediately.

---

## How to Use

### 1. Calibration (required on first run)

1. Power on the robot
2. Press **G18** (or send `c` over Bluetooth)
3. Slowly sweep the sensor array over the **black line and white surface** for 10 seconds
4. The LED blinks once per second during calibration
5. When done, calibration is saved to flash — you won't need to redo this unless the lighting changes

### 2. Start following

- Press **G19** (or send `s` over Bluetooth)
- LED turns solid — robot is following
- Press **G19** again (or send `x`) to stop

### 3. Tune PID over Bluetooth

Start with the defaults, then adjust:

```
P0.08   ← increase if the robot is slow to correct
D0.6    ← increase to reduce oscillation on straights
I0.0    ← keep at 0 unless there's consistent steady-state error
UP25    ← set a comfortable speed to start (25%)
```

---

## PID Details

The error is computed as the deviation of the weighted sensor centroid from the centre position (2000 on a 0–4000 scale).

```
error = 2000 - line_position
PIDvalue = (Kp × errorFactor × P) + (Ki × I) + (Kd × D)
```

- **Non-linear Kp** — scales up proportionally with error magnitude for sharper corner response
- **Dynamic base speed** — reduced automatically when `|error| > 800` to avoid spinning out
- **I-term clamped** to ±500 to prevent integral windup
- **Minimum PWM** of 50/255 to avoid motor stall in slow corrections

### Defaults

| Parameter | Value |
|-----------|-------|
| Kp | 0.08 |
| Ki | 0.0 |
| Kd | 0.6 |
| Base speed | ~20% (51/255) |

---

## LED Behaviour

| State | LED |
|-------|-----|
| Idle / not calibrated | Off |
| Calibrating | Blinks once per second |
| Following line | Solid on |
| Line lost → auto-stopped | Off |

---

## Project Structure

```
├── Line_Follower_V1_3.ino   ← main sketch
└── README.md
```

---

## Dependencies

- **Arduino IDE** with ESP32 board support ([install guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html))
- Built-in libraries only: `BluetoothSerial.h`, `Preferences.h`

No external libraries required.

---

## License

MIT — free to use, modify, and share. Attribution appreciated.

---
