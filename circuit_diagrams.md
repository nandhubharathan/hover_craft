# Hovercraft v5.5 — Circuit Diagrams

## 1. Hovercraft (On-Board Electronics)

### System Overview

```mermaid
graph TB
    subgraph POWER["🔋 Power"]
        BATT["LiPo Battery<br/>11.1V 3S"]
        BEC["BEC 5V"]
    end

    subgraph BRAIN["🧠 Flight Controller"]
        PICO["Raspberry Pi Pico<br/>(RP2040)"]
    end

    subgraph SENSORS["📡 Sensors"]
        IMU["MPU-6050<br/>Gyro + Accel"]
        ESP["ESP32-CAM<br/>AI-Thinker"]
        ULTRA["HC-SR04<br/>Ultrasonic"]
    end

    subgraph ACTUATORS["⚙️ Actuators"]
        LIFT_ESC["Lift ESC"]
        LIFT_MOTOR["Lift BLDC<br/>1700KV"]
        THRUST_ESC["Thrust ESC"]
        THRUST_MOTOR["Thrust BLDC<br/>1700KV"]
        SERVO["Rudder Servo<br/>MG90S"]
    end

    subgraph RF["📻 RF Receive Chain"]
        RX433["433 MHz RX<br/>Module"]
        NANO_RX["Arduino Nano<br/>(RX UART Bridge)"]
    end

    BATT --> BEC
    BATT --> LIFT_ESC
    BATT --> THRUST_ESC
    BEC --> PICO
    BEC --> ESP
    BEC --> SERVO
    BEC --> NANO_RX

    PICO -->|"GP16 PWM 50Hz"| LIFT_ESC --> LIFT_MOTOR
    PICO -->|"GP17 PWM 50Hz"| THRUST_ESC --> THRUST_MOTOR
    PICO -->|"GP18 PWM 50Hz"| SERVO

    IMU -->|"I2C (GP4/GP5) 400kHz"| PICO
    PICO -->|"3V3"| IMU
    ESP -->|"UART0 GP1 115200bps"| PICO
    ULTRA -->|"TRIG GPIO13"| ESP
    ESP -->|"ECHO GPIO15 (via divider)"| ULTRA

    RX433 -->|"DATA → D12"| NANO_RX
    NANO_RX -->|"D11 → 1kΩ/2kΩ → GP9<br/>SoftSerial 57600bps"| PICO
    BEC -->|"5V"| RX433
```

---

### Complete Signal Chain (RF → Control)

```mermaid
graph LR
    subgraph TX["🎮 Transmitter (Hand Controller)"]
        JOY["Joystick<br/>A0/A1"]
        SW["Switches<br/>D2 D3"]
        TX_NANO["TX Nano<br/>(ATmega328P)"]
        RF_TX["433MHz TX<br/>ASK Module"]
    end

    subgraph AIR["🌊 RF Link"]
        LINK["433 MHz OOK<br/>2000 bps"]
    end

    subgraph CRAFT["🚁 Hovercraft"]
        RF_RX["433MHz RX<br/>Module"]
        RX_NANO["RX Nano<br/>(UART Bridge)"]
        DIV["1kΩ/2kΩ<br/>Divider"]
        PICO2["Pico GP9<br/>UART1 RX"]
    end

    JOY --> TX_NANO
    SW --> TX_NANO
    TX_NANO -->|"D12 RadioHead"| RF_TX
    RF_TX --> LINK --> RF_RX
    RF_RX -->|"DATA → D12"| RX_NANO
    RX_NANO -->|"D11 SoftSerial<br/>57600bps"| DIV
    DIV -->|"3.3V safe"| PICO2
```

---

### Pico (RP2040) Pin Map

```mermaid
graph LR
    subgraph PICO["Raspberry Pi Pico (RP2040)"]
        direction TB
        GP0["GP0 — UART0 TX (reserved)"]
        GP1["GP1 — UART0 RX ← ESP32-Cam"]
        GP4["GP4 — I2C0 SDA"]
        GP5["GP5 — I2C0 SCL"]
        GP8["GP8 — UART1 TX (reserved)"]
        GP9["GP9 — UART1 RX ← Nano RX Bridge"]
        GP16["GP16 — Lift ESC PWM"]
        GP17["GP17 — Thrust ESC PWM"]
        GP18["GP18 — Servo PWM"]
        V3["3V3 OUT → MPU-6050"]
        GND["GND — Common"]
    end

    ESP32["ESP32-CAM GPIO1 TX"] -->|"115200 baud"| GP1
    MPU_SDA["MPU-6050 SDA"] <-->|"I2C 400kHz"| GP4
    MPU_SCL["MPU-6050 SCL"] <-->|"I2C 400kHz"| GP5
    NANO_D11["Nano RX D11<br/>(via 1kΩ/2kΩ divider)"] -->|"57600 baud"| GP9
    GP16 -->|"50 Hz PWM"| LIFT["Lift ESC Signal"]
    GP17 -->|"50 Hz PWM"| THRUST["Thrust ESC Signal"]
    GP18 -->|"50 Hz PWM"| RUDDER["Rudder Servo Signal"]
```

---

### Arduino Nano RX Bridge — Pin Map

```mermaid
graph LR
    subgraph NANO_RX_BOARD["Arduino Nano (RX UART Bridge)"]
        direction TB
        ND12["D12 — RadioHead RX Data"]
        ND11["D11 — SoftwareSerial TX (to Pico)"]
        ND10["D10 — RadioHead PTT (unused, HIGH)"]
        N5V["5V — VCC from BEC"]
        NGND["GND — Common"]
    end

    RF433["433 MHz RX<br/>Module DATA"] -->|"OOK signal"| ND12
    ND11 -->|"5V 57600bps"| R1["1kΩ"]
    R1 --> PICO_GP9["Pico GP9 (UART1 RX)"]
    R1 --> R2["2kΩ"]
    R2 --> GND_NODE["GND"]
    N5V --- BEC_5V["BEC 5V"]
    NGND --- PICO_GND["Pico GND"]
```

---

### Detailed Wiring Table — Hovercraft

| Connection | From | Pin | To | Pin | Notes |
|-----------|------|-----|-----|-----|-------|
| **Lift ESC** | Pico | GP16 | ESC Signal | White | 50 Hz PWM, 1000–1350 µs |
| **Thrust ESC** | Pico | GP17 | ESC Signal | White | 50 Hz PWM, 1000–1900 µs |
| **Rudder Servo** | Pico | GP18 | Servo Signal | Orange | 50 Hz PWM, 500–2500 µs |
| **IMU SDA** | Pico | GP4 | MPU-6050 | SDA | I2C0 @ 400 kHz, 3.3V |
| **IMU SCL** | Pico | GP5 | MPU-6050 | SCL | I2C0 @ 400 kHz, 3.3V |
| **IMU Power** | Pico | 3V3 | MPU-6050 | VCC | 3.3V only |
| **IMU Ground** | Pico | GND | MPU-6050 | GND | Common ground |
| **ESP32 → Pico** | ESP32 | GPIO1 (TX) | Pico | GP1 (RX) | UART0, 115200 baud |
| **Ultrasonic TRIG** | ESP32 | GPIO13 | HC-SR04 | TRIG | 3.3V output |
| **Ultrasonic ECHO** | ESP32 | GPIO15 | HC-SR04 | ECHO | ⚠️ 5V→3.3V divider needed |
| **433MHz RX → Nano** | 433MHz RX | DATA | Nano RX | D12 | RadioHead RH_ASK rxPin |
| **Nano RX → Divider** | Nano RX | D11 | Resistor | 1kΩ | SoftSerial TX 5V |
| **Divider → Pico** | Resistor | 2kΩ | Pico | GP9 (RX) | UART1, 57600 baud, 3.3V |
| **Nano RX Power** | BEC | 5V | Nano RX | VIN/5V | 5V supply |
| **Nano RX Ground** | Nano RX | GND | Pico | GND | MUST share common ground |

### Nano RX → Pico Voltage Divider (5V → 3.3V)

```
Nano RX D11 (5V) ──[1kΩ]──┬── Pico GP9 (UART1 RX, 3.3V)
                            │
                          [2kΩ]
                            │
                           GND
```

> [!IMPORTANT]
> The Arduino Nano outputs **5V** logic on D11. The Pico GPIO is **3.3V only** — connecting them directly **will damage the Pico**. The 1kΩ/2kΩ resistor divider brings 5V down to ~3.33V (safe). The SoftwareSerial runs at **57600 bps**; the Pico UART1 must also be configured at 57600 bps.

---

### ESP32-CAM Wiring

```mermaid
graph LR
    subgraph ESP["ESP32-CAM (AI-Thinker)"]
        GPIO1["GPIO 1 — Serial TX"]
        GPIO3["GPIO 3 — Serial RX (unused)"]
        GPIO13["GPIO 13 — HC-SR04 TRIG"]
        GPIO15["GPIO 15 — HC-SR04 ECHO"]
        ESP_5V["5V IN"]
        ESP_GND["GND"]
    end

    GPIO1 -->|"115200 baud"| PICO_RX["Pico GP1 (UART0 RX)"]
    GPIO13 -->|"10µs pulse"| TRIG["HC-SR04 TRIG"]
    ECHO_PIN["HC-SR04 ECHO"] -->|"via 5V→3.3V divider"| GPIO15
    ESP_5V --- BEC_5V["BEC 5V"]
    ESP_GND --- COMMON_GND["Common GND"]
```

> [!WARNING]
> GPIO 13 and 15 are shared with the SD card slot on the AI-Thinker ESP32-CAM. **Do NOT initialize the SD card** when using the ultrasonic sensor.

---

## 2. Remote Transmitter (Handheld Controller)

### Transmitter Overview

```mermaid
graph TB
    subgraph REMOTE["🎮 Remote Controller"]
        direction TB
        TX_NANO2["Arduino Nano<br/>(ATmega328P)"]
        JOY2["Dual-Axis<br/>Joystick"]
        AI_SW["AI Mode<br/>Toggle Switch"]
        LIFT_SW["Lift<br/>Push Button"]
        TX_MOD["433 MHz TX<br/>ASK Module"]
        BATT_TX["9V Battery"]
    end

    BATT_TX -->|"VIN"| TX_NANO2
    JOY2 -->|"A0 (Thrust)<br/>A1 (Steer)"| TX_NANO2
    AI_SW -->|"D2"| TX_NANO2
    LIFT_SW -->|"D3"| TX_NANO2
    TX_NANO2 -->|"D12 (Data)"| TX_MOD
    TX_NANO2 -->|"D10 (PTT)"| TX_MOD
```

---

### Transmitter Pin Map

```mermaid
graph LR
    subgraph NANO_TX["Arduino Nano (TX Controller)"]
        direction TB
        A0_TX["A0 — Thrust (Joystick X)"]
        A1_TX["A1 — Steer (Joystick Y)"]
        D2_TX["D2 — AI Mode Switch (PULLUP)"]
        D3_TX["D3 — Lift Button (PULLUP)"]
        D10_TX["D10 — PTT (RadioHead)"]
        D11_TX["D11 — RX (unused)"]
        D12_TX["D12 — TX Data (RadioHead)"]
        VIN_TX["VIN — 9V Battery"]
        V5_TX["5V OUT — TX Module VCC"]
        GND_TX["GND"]
    end

    JOY_X["Joystick X<br/>(Thrust/Brake)"] --> A0_TX
    JOY_Y["Joystick Y<br/>(Steer L/R)"] --> A1_TX
    AI_TOG["AI Toggle<br/>Switch"] --> D2_TX
    LIFT_BTN["Lift Button<br/>(Momentary)"] --> D3_TX
    D12_TX --> RF_TX2["433 MHz TX<br/>Data Pin"]
    D10_TX --> RF_PTT["433 MHz TX<br/>PTT Pin"]
    V5_TX --> RF_VCC["433 MHz TX<br/>VCC"]
```

---

### Detailed Wiring Table — Transmitter

| Connection | From | Pin | To | Pin | Notes |
|-----------|------|-----|-----|-----|-------|
| **Joystick X** | Joystick | VRx | TX Nano | A0 | Thrust axis (fwd/back) |
| **Joystick Y** | Joystick | VRy | TX Nano | A1 | Steer axis (left/right) |
| **Joystick VCC** | TX Nano | 5V | Joystick | +5V | 5V reference |
| **Joystick GND** | TX Nano | GND | Joystick | GND | Common ground |
| **AI Toggle** | Switch | COM | TX Nano | D2 | `INPUT_PULLUP` — LOW = AI ON |
| **AI Toggle GND** | Switch | NC | TX Nano | GND | Switch to ground |
| **Lift Button** | Button | COM | TX Nano | D3 | `INPUT_PULLUP` — press to toggle |
| **Lift Button GND** | Button | NC | TX Nano | GND | Switch to ground |
| **433 TX Data** | TX Nano | D12 | TX Module | DATA | RadioHead RH_ASK @ 2000 bps |
| **433 TX PTT** | TX Nano | D10 | TX Module | PTT | Push-to-talk (RadioHead) |
| **433 TX VCC** | TX Nano | 5V | TX Module | VCC | 5V supply |
| **433 TX GND** | TX Nano | GND | TX Module | GND | Common ground |
| **433 TX Antenna** | — | — | TX Module | ANT | 17.3 cm wire (¼λ @ 433 MHz) |

---

### Joystick Axis Mapping

```
                   FORWARD (thrust > 0)
                        ↑
                   ┌────┼────┐
                   │    │    │
         LEFT ─────┼────●────┼───── RIGHT
        (steer-)   │    │    │   (steer+)
                   └────┼────┘
                        ↓
                   BACKWARD (BRK = 1)
```

| Joystick Position | Nano Value | Pico Variable | Effect |
|-------------------|-----------|---------------|--------|
| Forward | `rawX > 552` | `joy_y = 0.0–1.0` | Forward thrust |
| Backward | `rawX < 472` | `brake_mode = True` | Turn-around mode |
| Left | `rawY < 472` | `joy_x < 0` | Steer left |
| Right | `rawY > 552` | `joy_x > 0` | Steer right |
| Center | `472–552` | `joy_x/y ≈ 0` | Neutral (deadzone) |

---

### Serial Packet Format (TX Nano → 433MHz → RX Nano → Pico)

```
RF Packet (6 bytes, RadioHead RH_ASK):
┌────────┬───────┬──────┬───────┬──────────┬──────────┐
│ thrust │ steer │ lift │  AI   │  brake   │ checksum │
│ 0–255  │ ±100  │ 0/1  │  0/1  │   0/1    │   XOR    │
└────────┴───────┴──────┴───────┴──────────┴──────────┘
  1 B      1 B    1 B    1 B      1 B         1 B

UART Frame (8 bytes, RX Nano → Pico at 57600 bps):
┌──────┬──────┬────────┬───────┬──────┬───────┬──────────┬──────────┐
│ 0xAA │ 0x55 │ thrust │ steer │ lift │  AI   │  brake   │ checksum │
│ sync │ sync │ 0–255  │ ±100  │ 0/1  │  0/1  │   0/1    │   XOR    │
└──────┴──────┴────────┴───────┴──────┴───────┴──────────┴──────────┘
  1 B    1 B    1 B      1 B    1 B    1 B      1 B         1 B

Pico UART1: baudrate=57600, GP9 RX, 8N1
Pico struct.unpack: '<BbBBBB' (6 bytes after stripping 2 sync bytes)
```

---

## 3. Power Distribution

```mermaid
graph TD
    LIPO["LiPo 3S 11.1V"] --> LIFT_ESC2["Lift ESC<br/>(direct)"]
    LIPO --> THRUST_ESC2["Thrust ESC<br/>(direct)"]
    LIPO --> BEC2["BEC<br/>11.1V → 5V"]
    BEC2 --> PICO_5V["Pico VSYS (5V)"]
    BEC2 --> SERVO_V["Servo VCC (5V)"]
    BEC2 --> ESP_V["ESP32-CAM VCC (5V)"]
    BEC2 --> NANO_RX_V["Nano RX Bridge VIN (5V)"]
    BEC2 --> ULTRA_V["HC-SR04 VCC (5V)"]
    PICO_5V --> PICO_3V["Pico 3V3 OUT"]
    PICO_3V --> IMU_V["MPU-6050 VCC (3.3V)"]

    TX_BATT["9V Battery (TX)"] --> TX_NANO_V["TX Nano VIN"]
    TX_NANO_V --> TX_5V["TX Nano 5V"]
    TX_5V --> RF_TX_V["433MHz TX Module VCC"]
```

> [!TIP]
> Use a dedicated BEC (not the ESC's built-in BEC) for clean 5V. ESC BECs have switching noise that affects the MPU-6050 gyro readings.

---

## 4. MLP Neural Network Architecture

```
Input Layer (7 nodes)          Hidden Layer (8 nodes)      Output Layer (4 nodes)
─────────────────────          ──────────────────────      ─────────────────────
 ax  (accel X, g)    ──┐
 ay  (accel Y, g)    ──┤       ┌──────────────────┐        lift_trim_us  (±50 µs)
 az  (accel Z, g)    ──┼──────►│  8 × tanh nodes  ├───────►thrust_trim_us(±50 µs)
 joy_x (steer ±1)   ──┤       │  (LUT approx)    │        servo_trim_us (±312 µs)
 joy_y (thrust ±1)  ──┤       └──────────────────┘        alert / conf  (0–1)
 obstacle (0/1)      ──┤
 yaw_rate (°/s /250) ──┘

Weights: int8 quantized (-127..127), dequantized × (1/127)
Activations: 256-entry LUT tanh / sigmoid (no math.exp)
Inference: < 10 ms on RP2040 @ 125 MHz (100 Hz budget)
```

---

## 5. On-Board Dual-Core Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    Raspberry Pi Pico (RP2040)            │
│                                                          │
│  ┌─ Core 0 (I/O) ─────────────────┐                     │
│  │  UART0 (GP1) ← ESP32-Cam       │                     │
│  │   Obstacle veto + Geometry 0xD4│                     │
│  │  UART1 (GP9) ← Nano RX Bridge  │                     │
│  │   57600bps framed packet parser │                     │
│  │            ↓                   │                     │
│  │   Shared State (thread-safe)   │                     │
│  │   joy_x/y, lift, ai, brake,    │                     │
│  │   obstacle_dir, cam_aspect,    │                     │
│  │   cam_height, ultra_dist       │                     │
│  └────────────────────────────────┘                     │
│                     ↓ _lock (mutex)                      │
│  ┌─ Core 1 (Control 100Hz) ───────────────────────┐     │
│  │  MPU-6050 I2C (GP4/GP5)                        │     │
│  │  MLP Inference (mlp_logic.py / weights.py)     │     │
│  │  hover_control.compute_targets()               │     │
│  │  Heading Memory + Pilot Overrule               │     │
│  │  Human Detection + Obstacle Veto               │     │
│  │            ↓                                   │     │
│  │   GP16 Lift ESC    GP17 Thrust ESC   GP18 Servo│     │
│  └────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────┘
```
