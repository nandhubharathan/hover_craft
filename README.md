# 🚁 Autonomous AI Hovercraft — v5.5

> **Self-correcting autonomous hovercraft** — if someone pushes it, tilts it, or the wind moves it off course, the craft detects the drift and steers itself back to its locked heading automatically. No input from the remote required.

**Platform:** Raspberry Pi Pico (RP2040) · MicroPython  
**Chassis:** 22 cm × 14.5 cm · ~400 g  
**Motors:** 2× Emax 1700KV BLDC · 9.2 cm 2-Blade Props  
**Servo:** MG90S (thrust vectoring / rudder)  
**Battery:** 11.1V 3S 80C LiPo  
**Modes:** Manual RC + Autonomous AI Stabilization

---

## 📁 Project Structure

```
hover_craft/
│
├── firmware/                        # All source code
│   ├── pico/                        # Raspberry Pi Pico (MicroPython)
│   │   ├── main.py                  # Dual-core entry point
│   │   ├── hover_control.py         # PWM, heading memory, safety
│   │   ├── mlp_logic.py             # TinyML inference engine (100 Hz)
│   │   └── weights.py               # MLP weights (int8 quantized)
│   │
│   ├── transmitter_nano/            # Arduino Nano — Hand Controller TX
│   │   └── transmitter_nano.ino     # 433 MHz TX, 6-byte packet
│   │
│   ├── receiver_nano/               # Arduino Nano — RX UART Bridge
│   │   └── receiver_nano.ino        # 433 MHz RX → Pico UART bridge
│   │
│   └── esp32_vision/                # ESP32-CAM — Vision + Ultrasonic
│       └── sketch_apr6a/
│           └── sketch_apr6a.ino     # Geometry + HC-SR04 streamer
│
├── 3d_designs/                      # 3D printable STL files (full chassis)
│   ├── 1-upper-hull-main-body.stl   # Main upper hull body
│   ├── 2-lower-hull.stl             # Lower hull base plate
│   ├── 3-thrust-motor-mount.stl     # Thrust BLDC motor bracket
│   ├── 4-roll-hoop.stl              # Roll protection hoop
│   ├── 5-lower-skirt-mount.stl      # Lower skirt attachment ring
│   ├── 6-upper-skirt-mount-complete.stl  # Upper skirt clamp assembly
│   ├── propeller.stl                # Prop guard / duct
│   ├── camerabox.stl                # ESP32-CAM enclosure
│   ├── joystick.stl                 # Handheld controller shell
│   └── joycover.stl                 # Controller top cover
│
├── simulation/                      # ROS2 / Gazebo SITL
│   ├── hovercraft.urdf              # Robot description
│   ├── hover_sim.launch.py          # ROS2 launch file
│   ├── sitl_bridge.py               # Software-in-the-loop bridge
│   └── world/
│       └── hover_lab.world          # Gazebo world
│
├── circuit_diagrams.md              # Full wiring diagrams (Mermaid)
├── Connection_Map.txt               # Pin-by-pin connection map
├── README.md                        # This file
└── .gitignore
```

---

## 🏗️ System Architecture

```
🎮 Hand Controller          📻 RF Link         🚁 Hovercraft
─────────────────           ──────────         ─────────────

TX Arduino Nano             433 MHz            RX Arduino Nano
  Joystick (A0/A1)   ─────►  ASK  ──────►     (UART Bridge)
  AI Switch  (D2)           2000bps              D12 ← RF DATA
  Lift Button (D3)                               D11 → Pico GP9
  D12 → RF TX DATA                           (1kΩ/2kΩ divider)
                                                      │
                                                      ▼
                                           Raspberry Pi Pico
                                           ┌──────────────────────────┐
                                           │ Core 0 (I/O)             │
                                           │  UART1 GP9 ← Nano Bridge │
                                           │  UART0 GP1 ← ESP32-Cam   │
                                           │  Shared State (mutex)    │
                                           │                          │
                                           │ Core 1 (Control 100Hz)   │
                                           │  MPU-6050 I2C (GP4/GP5)  │
                                           │  MLP TinyML Inference    │
                                           │  Heading Memory + PI     │
                                           │  hover_control PWM       │
                                           │       ↓      ↓      ↓   │
                                           │  GP16    GP17    GP18    │
                                           └───┼───────┼────────┼────┘
                                            Lift ESC Thrust  Rudder
                                            BLDC    ESC BLDC  Servo
```

### ESP32-CAM Sensor Node

```
ESP32-CAM ──── HC-SR04 Ultrasonic (GPIO 13 TRIG / GPIO 15 ECHO)
     │
     └── GPIO1 TX ──► Pico GP1 (UART0, 115200 bps)
         4-byte packet: [0xD4][aspect×10][bb_height][ultra_dist_cm]
```

---

## 🔌 Hardware Requirements

### On-Board (Hovercraft)

| Component | Qty | Notes |
|-----------|-----|-------|
| Raspberry Pi Pico (RP2040) | 1 | Flight controller |
| Arduino Nano (ATmega328P) | 1 | RX UART Bridge |
| 433 MHz RX Module | 1 | ASK/OOK receiver |
| ESP32-CAM (AI-Thinker) | 1 | Vision + ultrasonic |
| HC-SR04 Ultrasonic | 1 | Proximity sensing |
| MPU-6050 IMU | 1 | Gyro + accelerometer |
| Emax 1700KV BLDC | 2 | Lift + Thrust motors |
| 30A ESC | 2 | Lift + Thrust ESCs |
| MG90S Servo | 1 | Rudder / thrust vector |
| 9.2 cm 2-Blade Prop | 2 | CW + CCW pair |
| 11.1V 3S 80C LiPo | 1 | Main power |
| 5V BEC | 1 | Regulated supply |
| 1kΩ / 2kΩ Resistors | 2 | 5V→3.3V level shifter |
| 4.7kΩ Resistors | 2 | I2C SDA/SCL pull-ups |
| ball bearing 6702 | 1 |15x21x4mm|

### Hand Controller (Transmitter)

| Component | Qty | Notes |
|-----------|-----|-------|
| Arduino Nano (ATmega328P) | 1 | TX controller |
| 433 MHz TX Module | 1 | ASK/OOK transmitter |
| Dual-axis Joystick | 1 | Thrust + steer |
| Toggle Switch | 1 | AI mode enable |
| Momentary Push Button | 1 | Lift toggle |
| 9V Battery | 1 | Controller power |

---

## 📡 Wiring Summary

### RF Signal Chain

```
TX Nano D12 ──[433MHz RF]──► RX Nano D12
                                  │
                              RX Nano D11 (5V SoftwareSerial @ 57600bps)
                                  │
                            [1kΩ]─┬─── Pico GP9 (UART1 RX, 3.3V)
                                  │
                                [2kΩ]
                                  │
                                 GND
```

> ⚠️ **Critical:** The Nano outputs 5V on D11. The Pico GPIO is 3.3V max. The 1kΩ/2kΩ resistor divider is **mandatory** to avoid damaging the Pico.

### Pico Pin Map

| GPIO | Function | Protocol |
|------|----------|----------|
| GP0 | ESP32-Cam RX (Pico TX, reserved) | UART0 115200 |
| GP1 | ESP32-Cam TX → Pico RX | UART0 115200 |
| GP4 | MPU-6050 SDA | I2C0 400 kHz |
| GP5 | MPU-6050 SCL | I2C0 400 kHz |
| GP8 | Nano Bridge TX (reserved) | UART1 57600 |
| GP9 | Nano Bridge D11 → Pico RX | UART1 57600 |
| GP16 | Lift BLDC ESC | PWM 50 Hz |
| GP17 | Thrust BLDC ESC | PWM 50 Hz |
| GP18 | MG90S Rudder Servo | PWM 50 Hz |

### RX Nano (Bridge) Pin Map

| Pin | Function |
|-----|----------|
| D12 | 433 MHz RX module DATA (RadioHead rxPin) |
| D11 | SoftwareSerial TX → Pico GP9 (via divider) |
| D10 | RadioHead PTT (unused, held HIGH) |
| 5V | Power from BEC |
| GND | Common ground with Pico |

---

## 📦 UART Packet Format

### RF Packet: TX Nano → RX Nano (6 bytes, RadioHead RH_ASK @ 2000bps)

| Byte | Field | Type | Range |
|------|-------|------|-------|
| 0 | thrust | uint8 | 0–255 |
| 1 | steer | int8 | -100…+100 |
| 2 | liftToggle | uint8 | 0 or 1 |
| 3 | aiMode | uint8 | 0 or 1 |
| 4 | brakeMode | uint8 | 0 or 1 |
| 5 | checksum | uint8 | XOR of bytes 0–4 |

### UART Frame: RX Nano → Pico GP9 (8 bytes, 57600 bps 8N1)

```
[0xAA][0x55][thrust][steer][lift][ai][brake][checksum]
 sync0  sync1   1B     1B    1B   1B    1B      1B
```

Pico unpack: `struct.unpack('<BbBBBB', frame[2:])` 

---

## 🧠 MLP Neural Network

**Architecture:** 7 inputs → 8 hidden (tanh) → 4 outputs (sigmoid)

| Layer | Nodes | Activation |
|-------|-------|------------|
| Input | 7 | — |
| Hidden | 8 | tanh (256-entry LUT) |
| Output | 4 | sigmoid (via tanh identity) |

**Inputs:** `[ax, ay, az, joy_x, joy_y, obstacle_flag, yaw_rate]`  
**Outputs:** `[lift_trim_µs, thrust_trim_µs, servo_trim_µs, alert/confidence]`

**TinyML Optimizations:**
- `int8` quantized weights (range -127..127), dequantized × (1/127)
- 256-entry LUT tanh/sigmoid — ~8× faster than `math.tanh` on M0+
- Pre-allocated buffers — zero GC pressure at 100 Hz
- Confidence gate: if `alert < 0.3` → all trims zeroed

**Target:** < 10 ms per inference on RP2040 @ 125 MHz

---

## ⚙️ PWM Reference (1700KV @ 12.6V)

| PWM | RPM | Purpose |
|-----|-----|---------|
| 1000 µs | 0 | ESC armed / motor off |
| 1100 µs | ~2,142 | Thrust idle / dip manoeuvre |
| 1350 µs | ~7,497 | **Lift hover target** |
| 1500 µs | ~10,710 | Servo neutral |
| 1900 µs | ~19,278 | Thrust maximum |

---

## 🚀 Quick Start

### 1. Flash the Pico

```bash
cd firmware/pico
mpremote cp main.py hover_control.py mlp_logic.py weights.py :
```

### 2. Flash TX Arduino Nano

Open `firmware/transmitter_nano/transmitter_nano.ino` in Arduino IDE → select **Arduino Nano** → Upload.

### 3. Flash RX Arduino Nano (Bridge)

Open `firmware/receiver_nano/receiver_nano.ino` in Arduino IDE → select **Arduino Nano** → Upload.  
Verify Serial Monitor shows: `[OK] RadioHead ready on D12`

### 4. Flash ESP32-CAM

Open `firmware/esp32_vision/sketch_apr6a/sketch_apr6a.ino` in Arduino IDE → select **AI Thinker ESP32-CAM** → Upload.

### 5. Bench Test (NO PROPELLERS)

Pico Serial (Thonny) should show:
```
[CORE 0] I/O thread live — CAM UART0 | Nano UART1 @ 57600 baud
[CORE 1] ESCs armed — 100 Hz
[TX→PICO] THR=0 STR=0 LIFT=0 AI=0 BRK=0
```

### 6. Verify PWM Signals

| Channel | Idle | Active |
|---------|------|--------|
| GP16 (Lift) | 1000 µs | 1350 µs |
| GP17 (Thrust) | 1000 µs | scales with joystick |
| GP18 (Servo) | 1500 µs | ±500 µs with steer |

---

## 🛡️ Safety Checklist

- [ ] Replace placeholder `weights.py` with trained MLP weights before flight
- [ ] Confirm 6-byte TX packet matches `<BbBBBB>` Pico struct format
- [ ] Bench test without propellers — verify all PWM signals
- [ ] Test thrust-dip: observe 100 ms power dip during 180° servo flip
- [ ] Test failsafe: disconnect TX, confirm e-stop within 500 ms
- [ ] Verify voltage divider on Nano D11 → Pico GP9 before powering
- [ ] Add hardware kill switch between LiPo and ESCs
- [ ] Confirm all GNDs are tied: Nano RX ↔ Pico ↔ ESCs ↔ BEC

---

## 🔬 AI Autonomous Mode

When **AI Mode** is toggled ON (D2 switch on transmitter), the craft becomes self-correcting:

### 🔄 Self-Correction — The Key Feature

> **If the craft is pushed, bumped, tilted, or drifts off course by wind or surface irregularity — it corrects itself back to its original heading without any input from the remote.**

This works through a two-layer system running at 100Hz:

- **Heading Memory (Angle Integration):** The gyroscope's yaw rate is continuously integrated into a heading error value (in degrees). If the craft rotates away from its locked heading — whether from a bump, wind, or uneven surface — the error accumulates. Once it crosses 20°, the rudder servo fires a fixed 250 µs correction force to push it back.
- **Zero-Crossing Disengage:** When the craft returns to its locked heading, the correction immediately disengages and a 300 ms cooldown prevents the return momentum from triggering an over-correction in the opposite direction.
- **Tilt Correction (MLP):** The TinyML neural network reads the accelerometer and fires servo trim corrections (up to ±312 µs) to counteract roll tilt in real time.
- **Pilot Overrule:** If you move the joystick, heading memory clears instantly. On release, the craft re-locks to its new position as the fresh zero.

### Full Feature List

1. **Autonomous Heading Hold** — Locks heading at AI-on moment; corrects any external disturbance back to it
2. **Tilt-Rudder Correction** — MLP roll compensation via servo trim at 100 Hz
3. **Human Detection** — ESP32-CAM bounding box geometry → `STATE_BRAKING` (latched, joystick reset to exit)
4. **Ultrasonic Proximity Stop** — Fused with camera: forward thrust blocked within 22 cm, steering preserved for evasion
5. **Torque Offset Compensation** — Proportional rudder bias cancels lift motor rotational torque
6. **Gyro Debounce (Sustained-Spin Gate)** — 5 consecutive ticks above deadband required before integrating — filters vibration spikes
7. **AI Packet Debounce** — 5 consecutive `ai_mode=True` RF packets required before AI activates — prevents corrupt-packet false triggers
8. **Pilot Overrule + Heading Re-lock** — Pilot steering clears heading memory; heading re-locks on stick release

---

---

## 🖨️ 3D Design Files

All structural parts are fully 3D printable. Files are in the `3d_designs/` folder (STL format, ready to slice).

| File | Part | Notes |
|------|------|-------|
| `1-upper-hull-main-body.stl` | Upper hull | Main body — houses Pico, Nano, IMU |
| `2-lower-hull.stl` | Lower hull | Base plate — mounts ESCs and LiPo |
| `3-thrust-motor-mount.stl` | Thrust motor bracket | Rear BLDC mount with servo clearance |
| `4-roll-hoop.stl` | Roll hoop | Protects electronics on tip-over |
| `5-lower-skirt-mount.stl` | Lower skirt ring | Clamps garbage-bag skirt to hull base |
| `6-upper-skirt-mount-complete.stl` | Upper skirt clamp | Seals skirt top to hull |
| `propeller.stl` | Prop guard / duct | Optional duct for efficiency |
| `camerabox.stl` | ESP32-CAM enclosure | Front-facing camera housing |
| `joystick.stl` | Controller shell | Handheld TX enclosure body |
| `joycover.stl` | Controller cover | TX enclosure top cap |

**Recommended print settings:** PLA/PETG · 20% infill · 0.2 mm layer height · 3 perimeters

---

## 🌐 Simulation (ROS2 / Gazebo)

```bash
# Launch Gazebo simulation
ros2 launch simulation/hover_sim.launch.py

# Run SITL bridge (connects Pico serial to ROS2)
python3 simulation/sitl_bridge.py
```

---

## 📋 Libraries Required

### Arduino (Nano TX + RX)
- [RadioHead](http://www.airspayce.com/mikem/arduino/RadioHead/) — `RH_ASK` driver
- `SPI.h` — bundled with Arduino IDE
- `SoftwareSerial.h` — bundled with Arduino IDE (RX Nano only)

### ESP32-CAM
- `esp_camera.h` — bundled with ESP32 Arduino core

### Pico (MicroPython)
- `machine`, `_thread`, `struct`, `time` — all built-in to MicroPython

---

## ⚠️ Known Notes

- **Baud rate:** `receiver_nano.ino` SoftwareSerial runs at **57600 bps**. Ensure `main.py` `UART(1, baudrate=57600)` matches.
- **SD card conflict:** ESP32-CAM GPIO 13/15 are shared with the SD card slot. Do **not** initialize the SD card.
- **Prop direction:** Lift prop pushes air DOWN (into skirt). Thrust prop pushes air AFT through rudder channel.
- **Weights:** `weights.py` ships with Xavier-initialized placeholder values. **Replace with trained weights before any flight.**
- **3.3V rule:** MPU-6050 must be powered from Pico 3V3 pin only.

---

*Karunya Institute of Technology and Sciences — B.Tech Project*
