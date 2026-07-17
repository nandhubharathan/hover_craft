"""
main.py — Dual-Core Entry Point (1700KV 4-Layer 3cm Skirt v5.4)
================================================================
Raspberry Pi Pico (RP2040) — MicroPython

Core 0 : ESP32-Cam UART0 obstacle veto (ABSOLUTE PRIORITY)
         + 0xD4 Geometry Packet parser (Aspect/Height/UltraDist)
         + Arduino Nano UART1 serial packet parser
Core 1 : MPU-6050 IMU → calibrate → MLP tilt-rudder → PWM @ 100 Hz

Changes in v5.4 vs v5.3
------------------------
1. 0xD4 Geometry Packet expanded to 4 bytes (was 5-byte CRC format):
   [0xD4][aspect_x10][bb_height][ultra_dist_cm]
   Ultrasonic distance from ESP32-CAM HC-SR04 sensor.

2. Dual-Veto Braking:
   STATE_BRAKING triggers on human AI confirmation (latched)
   OR ultrasonic proximity < 25 cm (proximity stop, auto-clears).

3. [V5.4] Telemetry with DIST field for ultrasonic verification.

Previous features (v3.8–v5.3) unchanged:
  Angle Integration (Heading Memory), Pilot Overrule,
  Post-Inflation Lock-In, Max-Input Thrust Mixer,
  Snap burst, auto-idle thermal relief, MLP tilt-rudder PI,
  obstacle veto, failsafe, LPF, confidence gate, post-inflate cal.

Pin Map
-------
  GP0  → UART0 TX  (reserved)
  GP1  ← UART0 RX  (ESP32-Cam TX → obstacle veto + geometry)
  GP4  ↔ MPU-6050 SDA (I2C0)
  GP5  ↔ MPU-6050 SCL (I2C0)
  GP8  → UART1 TX  (reserved)
  GP9  ← UART1 RX  (Nano via 1 kΩ/2 kΩ divider)
  GP16 → Lift ESC PWM
  GP17 → Thrust ESC PWM
  GP18 → Servo PWM

Nano Serial Protocol (9600 8N1)
--------------------------------
  [0xAA][0x55][thrust][steer][lift][ai][brake][crc]
  CRC = thrust ^ (steer & 0xFF) ^ lift ^ ai ^ brake
"""

import _thread
import time
import struct
from machine import Pin, I2C, UART

import hover_control
import mlp_logic
import weights

# ── Configuration ─────────────────────────────────────────────────────
AI_GAIN        = 1.0    # locked for 3 cm heavy-skirt stability testing
CONTROL_HZ     = 100
CONTROL_TICK   = 10     # ms
# GYRO_DEADZONE is now defined in hover_control.py (0.08 °/s) ← v5.5
# Used here via hover_control.GYRO_DEADZONE for consistent jitter filtering.

# =====================================================================
#  SHARED STATE
# =====================================================================

_lock = _thread.allocate_lock()

_shared = {
    "joy_x":        0.0,
    "joy_y":        0.0,
    "lift_on":      False,
    "ai_mode":      False,
    "brake_mode":   False,
    "obstacle_dir": 0,
    "last_rx_ms":   0,
    "do_inflate":   False,
    "cam_aspect":   0.0,      # v4.2: ESP32 0xD4 geometry — aspect ratio
    "cam_height":   0,        # v4.2: ESP32 0xD4 geometry — pixel height
    "ultra_dist":   255,      # v5.4: ultrasonic distance cm (255 = no echo)
}


def _shared_read():
    _lock.acquire()
    snap = dict(_shared)
    _lock.release()
    return snap


def _shared_write(**kwargs):
    _lock.acquire()
    for k, v in kwargs.items():
        _shared[k] = v
    _lock.release()


# =====================================================================
#  ESP32-CAM — UART0 (GP1 RX)
# =====================================================================
#
# Packet protocol (4 bytes, sent by ESP32-CAM):
#
#   [0xAB][DIR][DIST][CRC]
#
#   0xAB  — sync byte
#   DIR   — direction code:  0=clear  1=brake  2=left  3=right
#   DIST  — distance in cm, uint8 (0–254). 255 = no object detected.
#   CRC   — 0xAB ^ DIR ^ DIST  (1-byte XOR checksum)
#
# Obstacle threshold: OBSTACLE_TRIGGER_CM = 20
#   DIST > 20 or DIST == 255  →  clear (code 0).
#   DIST <= 20                →  DIR code forwarded to control loop.
#
# Legacy fallback: old single-byte ASCII '0'–'3' still decoded,
#   forwarded as direction-only with no distance filtering.
#
# ESP32-CAM Arduino sketch send function:
#   void send_obstacle(uint8_t dir, uint8_t dist_cm) {
#       uint8_t crc = 0xAB ^ dir ^ dist_cm;
#       Serial.write(0xAB);
#       Serial.write(dir);
#       Serial.write(dist_cm);
#       Serial.write(crc);
#   }
#   // Send dist_cm=255 when no object is detected.
#
# === 0xD4 Geometry Packet (v5.4 — 4-Byte Compact) ====================
#
#   [0xD4][aspect_x10][bb_height][ultra_dist_cm]
#
#   0xD4           — sync byte for geometry data
#   aspect_x10     — uint8: aspect ratio × 10 (e.g. 6 → 0.6)
#   bb_height      — uint8: bounding box pixel height (0–255)
#   ultra_dist_cm  — uint8: ultrasonic distance in cm (0–255)
#
#   No CRC — compact format.  Validated by sync byte + range.
#   Used by is_human_signature() in hover_control.py.
#   Ultrasonic distance stored in shared["ultra_dist"].
# =====================================================================

OBSTACLE_TRIGGER_CM = 20    # veto activates at this distance or closer

_cam_uart  = None
_cam_buf   = bytearray()
_CAM_SYNC  = 0xAB
_CAM_PKT   = 4              # bytes per new-format packet
_CAM_GEO_SYNC = 0xD4        # v5.4: geometry packet sync byte
_CAM_GEO_PKT  = 4           # v5.4: geometry packet length (4 bytes, no CRC)
_last_cam_dist = 255        # last reported distance (cm); 255 = no object


def _init_cam_uart():
    global _cam_uart
    _cam_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))


def _read_cam():
    """
    Parse ESP32-CAM UART stream and return obstacle direction code.

    New 4-byte packet [0xAB][DIR][DIST][CRC]:
      - CRC validated before accepting.
      - Returns DIR only when DIST <= OBSTACLE_TRIGGER_CM (20 cm).
      - Returns 0 (clear) when object is farther away or absent.

    Legacy single-byte ASCII '0'–'3' still accepted (no distance filter).
    Always uses the most recent valid packet if multiple arrive per tick.
    """
    global _cam_buf, _last_cam_dist

    waiting = _cam_uart.any()
    if waiting:
        chunk = _cam_uart.read(waiting)
        if chunk:
            _cam_buf.extend(chunk)

    result_dir = 0

    while len(_cam_buf) >= _CAM_PKT:

        # ── New 4-byte packet ─────────────────────────────────────────
        if _cam_buf[0] == _CAM_SYNC:
            direction = _cam_buf[1]
            dist_cm   = _cam_buf[2]
            crc       = _cam_buf[3]

            expected  = (_CAM_SYNC ^ direction ^ dist_cm) & 0xFF
            if crc == expected and direction <= 3:
                _last_cam_dist = dist_cm
                if dist_cm <= OBSTACLE_TRIGGER_CM:
                    result_dir = direction
                    print("[CAM] OBSTACLE dir={} dist={}cm ≤ {}cm — VETO ACTIVE".format(
                        direction, dist_cm, OBSTACLE_TRIGGER_CM))
                else:
                    result_dir = 0   # object present but far enough — clear
            _cam_buf = _cam_buf[_CAM_PKT:]   # consume packet (valid or corrupt)

        # ── Legacy single-byte ASCII '0'–'3' ─────────────────────────
        elif 48 <= _cam_buf[0] <= 51:
            result_dir = _cam_buf[0] - 48
            _cam_buf   = _cam_buf[1:]

        # ── 0xD4 Geometry Packet (v5.4 — 4-byte compact: aspect/height/ultra) ──
        elif _cam_buf[0] == _CAM_GEO_SYNC and len(_cam_buf) >= _CAM_GEO_PKT:
            asp_x10   = _cam_buf[1]
            bb_h      = _cam_buf[2]
            dist_cm   = _cam_buf[3]
            _shared_write(
                cam_aspect = asp_x10 / 10.0,
                cam_height = bb_h,
                ultra_dist = dist_cm,
            )
            _cam_buf = _cam_buf[_CAM_GEO_PKT:]

        # ── Unknown byte — discard, re-sync ──────────────────────────
        else:
            _cam_buf = _cam_buf[1:]

    return result_dir


# =====================================================================
#  ARDUINO NANO — UART1 (GP9 RX via 1k/2k divider)
# =====================================================================

_nano_uart = None
_SYNC_A    = 0xAA
_SYNC_B    = 0x55
_FRAME_LEN = 8
_PKT_FMT   = '<BbBBBB'
_rx_buf    = bytearray()


def _init_nano_uart():
    global _nano_uart
    _nano_uart = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9),
                      bits=8, parity=None, stop=1, timeout=0)


def _read_nano_packet():
    global _rx_buf

    waiting = _nano_uart.any()
    if waiting:
        chunk = _nano_uart.read(waiting)
        if chunk:
            _rx_buf.extend(chunk)

    if len(_rx_buf) < _FRAME_LEN:
        return None

    # Hunt for sync header
    idx = -1
    for i in range(len(_rx_buf) - 1):
        if _rx_buf[i] == _SYNC_A and _rx_buf[i + 1] == _SYNC_B:
            idx = i
            break

    if idx < 0:
        _rx_buf = _rx_buf[-1:]
        return None
    if idx > 0:
        _rx_buf = _rx_buf[idx:]
    if len(_rx_buf) < _FRAME_LEN:
        return None

    frame   = _rx_buf[:_FRAME_LEN]
    _rx_buf = _rx_buf[_FRAME_LEN:]

    try:
        thrust, steer, lift_toggle, ai_mode, brake_mode, checksum = \
            struct.unpack(_PKT_FMT, bytes(frame[2:]))
    except Exception:
        return None

    expected = (thrust ^ (steer & 0xFF) ^ lift_toggle ^ ai_mode ^ brake_mode) & 0xFF
    if checksum != expected:
        return None

    print(f"[TX->PICO] THR={thrust} STR={steer} "
          f"LIFT={lift_toggle} AI={ai_mode} BRK={brake_mode}")

    return (thrust, steer, bool(lift_toggle), bool(ai_mode), bool(brake_mode))


# =====================================================================
#  CORE 0 — I/O THREAD
# =====================================================================

_prev_lift_state = False


def _core0_io_loop():
    global _prev_lift_state
    _init_cam_uart()
    _init_nano_uart()
    print("[CORE 0] I/O thread live — CAM UART0 | Nano UART1 @ 9600 baud")

    while True:
        t0 = time.ticks_ms()

        _shared_write(obstacle_dir=_read_cam())

        pkt = _read_nano_packet()
        if pkt is not None:
            thrust, steer, lift_on, ai_mode, brake_mode = pkt
            joy_y = thrust / 255.0
            joy_x = max(-1.0, min(1.0, steer / 100.0))

            if lift_on and not _prev_lift_state:
                _shared_write(do_inflate=True)
            _prev_lift_state = lift_on

            _shared_write(
                joy_x      = joy_x,
                joy_y      = joy_y,
                lift_on    = lift_on,
                ai_mode    = ai_mode,
                brake_mode = brake_mode,
                last_rx_ms = time.ticks_ms(),
            )

        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        if elapsed < 10:
            time.sleep_ms(10 - elapsed)


# =====================================================================
#  MPU-6050
# =====================================================================

_MPU_ADDR     = 0x68
_PWR_MGMT_1   = 0x6B
_ACCEL_XOUT_H = 0x3B
_i2c          = None

# Startup calibration offsets — set by _init_mpu(), used by _read_imu_6dof().
# Stored as (off_x, off_y, off_z) in raw g units.
# After calibration: level craft reads 0.00 g on X/Y, 1.00 g on Z.
# Defaults to (0, 0, 0) if IMU init fails — allows manual-only flight.
_accel_offset = (0.0, 0.0, 0.0)

# Gyro-Z calibration offset — average °/s at rest, subtracted from live reads.
# Set by _init_mpu() during startup cal.  Defaults to 0.0 on failure.
_gyro_z_offset = 0.0

_CAL_SAMPLES    = 50    # ~500 ms @ 10 ms/sample
_CAL_SAMPLE_MS  = 10    # ms between calibration samples


def _init_mpu():
    """
    Wake the MPU-6050, then collect _CAL_SAMPLES readings (~500 ms)
    to compute accel and gyro-Z offsets.

    Offset convention:
      off_x = mean(raw_ax)         → zeroes X at rest
      off_y = mean(raw_ay)         → zeroes Y at rest
      off_z = mean(raw_az) - 1.0   → sets Z baseline to exactly 1.00 g
      gyro_z_off = mean(raw_gz/131) → zeroes gyro drift at rest

    On any OSError during calibration:
      • Prints [FATAL] IMU Calibration Failed
      • Leaves _accel_offset = (0, 0, 0), _gyro_z_offset = 0.0
      • Execution continues — manual flight remains operational.
    """
    global _i2c, _accel_offset, _gyro_z_offset

    _i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)

    try:
        # Skip full reset (0x80) — it puts the chip back to sleep and the
        # subsequent wake write can race with Core 0 UART activity.
        # Simply clear the sleep bit directly: write 0x00 to PWR_MGMT_1.
        _i2c.writeto_mem(_MPU_ADDR, _PWR_MGMT_1, b"\x00")
        time.sleep_ms(500)   # generous settle — sensor ADC needs time to start

        print("[CALIBRATING] Please keep craft level...")

        sum_x  = 0.0
        sum_y  = 0.0
        sum_z  = 0.0
        sum_gz = 0.0

        for _ in range(_CAL_SAMPLES):
            # Read 14 bytes: accel(6) + temp(2) + gyro(6) → regs 0x3B–0x48
            data   = _i2c.readfrom_mem(_MPU_ADDR, _ACCEL_XOUT_H, 14)
            ax_raw = (data[0] << 8) | data[1]
            ay_raw = (data[2] << 8) | data[3]
            az_raw = (data[4] << 8) | data[5]
            # data[6:8] = temperature (ignored)
            gz_raw = (data[12] << 8) | data[13]
            if ax_raw > 32767: ax_raw -= 65536
            if ay_raw > 32767: ay_raw -= 65536
            if az_raw > 32767: az_raw -= 65536
            if gz_raw > 32767: gz_raw -= 65536
            sum_x  += ax_raw / 16384.0
            sum_y  += ay_raw / 16384.0
            sum_z  += az_raw / 16384.0
            sum_gz += gz_raw / 131.0     # LSB/°/s at ±250°/s range
            time.sleep_ms(_CAL_SAMPLE_MS)

        # off_z corrects for gravity: subtract 1.0 g so level Z reads 1.00
        off_x  =  sum_x  / _CAL_SAMPLES
        off_y  =  sum_y  / _CAL_SAMPLES
        off_z  = (sum_z  / _CAL_SAMPLES) - 1.0
        gz_off =  sum_gz / _CAL_SAMPLES

        _accel_offset  = (off_x, off_y, off_z)
        _gyro_z_offset = gz_off
        print(
            "[CALIBRATING] Done — offsets X:{:.4f} Y:{:.4f} Z:{:.4f} GZ:{:.2f}°/s".format(
                off_x, off_y, off_z, gz_off)
        )

    except OSError as exc:
        print(f"[FATAL] IMU Calibration Failed ({exc}) — offsets zeroed, manual flight only")
        _accel_offset  = (0.0, 0.0, 0.0)
        _gyro_z_offset = 0.0


def _read_imu_6dof():
    """
    Read full 6-DOF IMU: accelerometer + gyroscope (14-byte burst).

    Returns (rel_ax, rel_ay, rel_az, yaw_rate_dps) where:
      rel_ax     ≈ 0.00 g   (X tilt from level)
      rel_ay     ≈ 0.00 g   (Y tilt from level)
      rel_az     ≈ 1.00 g   (Z gravity, exactly 1.00 when level)
      yaw_rate   in °/s     (Gyro Z, calibrated — positive = CW spin)

    I2C burst: registers 0x3B–0x48 (14 bytes)
      [0:5]   accel X/Y/Z  (16384 LSB/g at ±2g)
      [6:7]   temperature   (ignored)
      [8:13]  gyro  X/Y/Z  (131 LSB/°/s at ±250°/s)

    Formula: Relative_A = Raw_A - Offset_A
    (For Z the offset already embeds the -1.0 g gravity correction.)
    """
    data   = _i2c.readfrom_mem(_MPU_ADDR, _ACCEL_XOUT_H, 14)
    ax_raw = (data[0] << 8) | data[1]
    ay_raw = (data[2] << 8) | data[3]
    az_raw = (data[4] << 8) | data[5]
    # data[6:8] = temperature (skipped)
    gz_raw = (data[12] << 8) | data[13]
    if ax_raw > 32767: ax_raw -= 65536
    if ay_raw > 32767: ay_raw -= 65536
    if az_raw > 32767: az_raw -= 65536
    if gz_raw > 32767: gz_raw -= 65536

    raw_ax = ax_raw / 16384.0
    raw_ay = ay_raw / 16384.0
    raw_az = az_raw / 16384.0
    yaw_rate = (gz_raw / 131.0) - _gyro_z_offset   # °/s, calibrated

    off_x, off_y, off_z = _accel_offset
    return (raw_ax - off_x), (raw_ay - off_y), (raw_az - off_z), yaw_rate


def _normalize_accel(ax, ay, az):
    """Clamp to ±2.0 g — keeps MLP inputs within trained distribution."""
    c = lambda v: max(-2.0, min(2.0, v))
    return c(ax), c(ay), c(az)


# =====================================================================
#  CORE 1 — CONTROL LOOP @ 100 Hz  (v5.3)
# =====================================================================

_MLP_ZERO = [0.0, 0.0, 0.0, 0.0]

# ── v4.2 Human Detection State Machine ───────────────────────────────
_STATE_NORMAL  = 0
_STATE_BRAKING = 1
_flight_state  = _STATE_NORMAL
_human_confirm = 0              # consecutive ticks with human signature
_brake_thrust_us = 0            # current thrust during braking ramp-down
_prev_dip = False

# MLP lift trim LPF (α=0.2) — prevents jitter around 11 500 RPM target
_LP_ALPHA     = 0.15    # v4.2 tuned: smoother lift-trim at hover RPM
_mlp_lift_lpf = 0.0

# Tilt-rudder PI controller state
# ---------------------------------
# P term: proportional to current tilt — fast reaction to lean
# I term: integrates persistent drift over time — corrects slow yaw
#         even when the craft appears flat but keeps drifting sideways
#
# Tuning guide (printed in banner):
#   YAW_KP  — raise if correction is too weak / slow to react
#              lower if servo oscillates (too aggressive)
#   YAW_KI  — raise if craft corrects lean but still slowly drifts
#              lower if servo slowly walks to one extreme and stays
#   YAW_KI_LIMIT — caps the integrator; prevents windup if craft is
#                   held in place manually or pushed against a wall
_tilt_rudder_lpf  = 0.0   # LPF output fed to servo
_yaw_integral     = 0.0   # PI integrator accumulator

# ── Lift startup state machine (v3.9) ────────────────────────────────
#
# On lift switch press, the sequence is:
#
#  SOFTSTART phase — ramp lift from PWM_MIN → LIFT_HOVER_PWM at
#                    LIFT_SOFTSTART_STEP_US per tick (~0.67 s).
#                    During this ramp, IMU is sampled every tick to
#                    build the HOVER tilt baseline. No burst spike.
#
#  BURST phase     — once at hover RPM, hold LIFT_BURST_PWM (1700 µs)
#                    for LIFT_BURST_TICKS (0.6 s) to pop the skirt bag.
#
#  SETTLE phase    — ramp 1700 → 1536 µs at LIFT_RAMP_STEP_US.
#
#  HOVER phase     — normal compute_targets() with tilt correction
#                    using the hover-RPM baseline.
#
# Two IMU baselines are maintained:
#   _accel_offset       — cold/idle baseline (set at boot by _init_mpu)
#   _hover_accel_offset — hover-RPM baseline (set during SOFTSTART)
#
# The hover baseline accounts for motor vibration shifting the IMU
# average. Tilt correction always uses _hover_accel_offset when
# lift_on, falling back to _accel_offset when lift is off.

_INFLATE_IDLE        = 0
_INFLATE_SOFTSTART   = 1   # ramp to hover + sample hover baseline
_INFLATE_BURST       = 2
_INFLATE_SETTLE      = 3
_INFLATE_CALIBRATING = 4   # v5.0: post-inflation running-zero

_inflate_phase       = _INFLATE_IDLE
_inflate_burst_ticks = 0
_inflate_settle_us   = 0
_softstart_us        = 0   # current PWM during soft ramp
_SOFTSTART_INIT_US   = hover_control.PWM_MIN + 100  # v4.2: start at 1100 µs

# Hover-RPM IMU baseline — set during SOFTSTART, used while lift_on
_hover_accel_offset  = (0.0, 0.0, 0.0)
_hover_cal_sum       = [0.0, 0.0, 0.0]
_hover_cal_count     = 0
_HOVER_CAL_SAMPLES   = 30   # samples taken during last 30 ticks of ramp

# ── Auto-idle thermal relief ──────────────────────────────────────────
_IDLE_TIMEOUT_MS = 5_000
_IDLE_LIFT_SCALE = 0.85
_idle_since_ms   = 0
_prev_lift_on    = False

# Telemetry throttle — one print per 200 ms (non-blocking)
_TELEM_INTERVAL_MS = 200
_last_telem_ms     = 0

# MLP error latch — one message per lift-on session
_mlp_error_reported = False


# Drift correction LPF state — smooths gyro counter-steer
# (v5.3: retained for Pi tilt integration; heading memory is in hover_control)
_drift_trim_lpf = 0.0
_DRIFT_LPF_ALPHA = 0.25  # v4.2 tuned: faster drift-trim for 3.0°/s deadband

# v5.3: Pilot overrule tracking — detect stick release for heading re-lock
_prev_pilot_steering = False

# v5.3: AI-mode debounce — prevents single-tick glitches from corrupted
# Nano packets (1-byte XOR CRC has ~1/256 false positive rate at 100 Hz,
# which means a phantom ai_mode=True can appear roughly every 2.5 s).
# Require 5 consecutive ticks of ai_mode=True before allowing AI.
_AI_DEBOUNCE_TICKS = 5
_ai_confirm_count  = 0
_ai_mode_confirmed = False   # the debounced, safe-to-use flag


def _control_loop():
    """
    Core 1 main loop — v5.3 (Heading Memory + Max-Input Mixer).

    v5.3 additions:
      • Angle Integration (Heading Memory): replaces rate-only correction.
        heading_error += gyro_z * dt.  Rudder holds until craft returns
        to its locked Zero Position.
      • Pilot Overrule: |joy_x| > 0.1 clears heading_error; on release
        the current heading re-locks as the new Zero.
      • Post-Inflation Lock-In: heading_error hard-reset during BURST/SETTLE.
        Integration starts only after calibration completes.
      • Max-Input Thrust Mixer: thrust = max(pilot, STABILIZATION_THRUST_PWM).
      • [V5.3] Telemetry with heading error in degrees.

    All v3.8–v5.1 features preserved: Snap burst, heading hold,
    auto-idle, MLP PI, obstacle veto, failsafe, LPF, human detection,
    post-inflate calibration, active yaw/tilt correction.
    """
    global _prev_dip, _mlp_lift_lpf, _idle_since_ms
    global _inflate_phase, _inflate_burst_ticks, _inflate_settle_us
    global _prev_lift_on, _last_telem_ms, _mlp_error_reported
    global _softstart_us, _hover_accel_offset
    global _hover_cal_sum, _hover_cal_count, _tilt_rudder_lpf
    global _yaw_integral, _drift_trim_lpf
    global _flight_state, _human_confirm, _brake_thrust_us
    global _prev_pilot_steering
    global _ai_confirm_count, _ai_mode_confirmed

    _init_mpu()
    hover_control.init_pwm()

    print("[CORE 1] Arming ESCs (1 s) ...")
    for _ in range(100):
        hover_control.set_lift(hover_control.PWM_MIN)
        hover_control.set_thrust(hover_control.PWM_MIN)
        time.sleep_ms(CONTROL_TICK)
    print("[CORE 1] ESCs armed — 100 Hz | target 11 484 RPM @ 1536 µs")

    while True:
        t0 = time.ticks_ms()

        state      = _shared_read()
        joy_x      = state["joy_x"]
        joy_y      = state["joy_y"]
        lift_on    = state["lift_on"]
        obstacle   = state["obstacle_dir"]
        ai_mode    = state["ai_mode"]
        brake_mode = state["brake_mode"]
        last_rx_ms = state["last_rx_ms"]
        cam_aspect = state["cam_aspect"]    # v4.2
        cam_height = state["cam_height"]    # v4.2
        ultra_dist = state["ultra_dist"]    # v5.4: ultrasonic cm
        do_inflate = state["do_inflate"]

        # ── v5.3: AI-mode debounce (two-way, v5.3.1 hardened) ─────────
        # Nano CRC is single-byte XOR → ~1/256 false positive rate.
        # A single corrupted packet with ai_mode=True would run MLP +
        # PI + heading integration for one tick, producing a visible
        # servo twitch.  Require 5 consecutive ai_mode=True ticks
        # (~50 ms) before activating AI.  Instant drop on ai_mode=False.
        #
        # v5.3.1 FIX: Also unlatch _ai_mode_confirmed when confirm count
        # drops below threshold — prevents stale latch if a burst of
        # corrupt packets (e.g. motor startup noise) falsely triggered
        # the debounce and then no clean ai_mode=False arrives promptly.
        if ai_mode:
            _ai_confirm_count = min(_ai_confirm_count + 1,
                                   _AI_DEBOUNCE_TICKS + 5)
        else:
            _ai_confirm_count  = 0
            _ai_mode_confirmed = False

        if _ai_confirm_count >= _AI_DEBOUNCE_TICKS:
            _ai_mode_confirmed = True
        else:
            # v5.3.1: Two-way unlatch — if count drops below threshold
            # (e.g. after a single valid ai_mode=False resets count to 0),
            # immediately revoke AI.  The old code only cleared on the
            # explicit `else` branch, so a stale True could persist if
            # the very next packet was corrupt again (count goes 0→1
            # without entering the else branch).
            _ai_mode_confirmed = False

        # Use the debounced flag for ALL downstream AI decisions.
        # Raw ai_mode is NEVER used for correction — only for telemetry.
        ai_active = _ai_mode_confirmed

        # Initialise mlp_out and stab_active here so telemetry (which runs
        # before the failsafe continues) can always safely read them.
        mlp_out     = _MLP_ZERO
        _conf       = 0.0
        stab_active = False   # v5.1: thrust boost indicator

        # ── IMU read — always, even during failsafe / motors off ─────
        # Must run before failsafe continues so tilt is always visible.
        # v4.0: 6-DOF read returns accel + gyro-Z yaw rate.
        try:
            ax_raw, ay_raw, az_raw, yaw_rate = _read_imu_6dof()
        except OSError:
            ax_raw, ay_raw, az_raw, yaw_rate = 0.0, 0.0, 1.0, 0.0
        ax, ay, az = _normalize_accel(ax_raw, ay_raw, az_raw)

        # ── v5.5: Gyro deadzone moved to compute_targets() ─────────────
        # The deadzone is now applied to spin_err_dps (gyro_z - op_zero_gz)
        # inside compute_targets(), NOT to raw yaw_rate here.  Filtering
        # the raw reading then subtracting op_zero_gz creates a constant
        # bias of -op_zero_gz which the D-term amplifies into servo jitter.

        # ── v5.3: Pilot Overrule (Heading Reset) ──────────────────────
        # When the pilot is actively steering (|joy_x| > 0.1), the
        # heading memory must NOT fight them.  Clear the accumulator
        # continuously while steering.  On release, the craft's current
        # heading becomes the new Zero Position.
        #
        # v5.3.1 FIX: _prev_pilot_steering is now tracked ONLY when
        # ai_active is True.  Previously it was updated unconditionally,
        # so joystick movements during AI-OFF built up stale state that
        # could trigger heading resets on a glitch ai_active=True tick.
        pilot_steering = abs(joy_x) > 0.1
        if ai_active:
            if pilot_steering:
                # Pilot is steering — clear heading memory continuously
                hover_control.reset_heading_error()
            elif _prev_pilot_steering and not pilot_steering:
                # Pilot just released the stick — re-lock new heading
                hover_control.reset_heading_error()
                print("[V5.3] Pilot released stick — heading re-locked")
            _prev_pilot_steering = pilot_steering
        else:
            # AI OFF — reset tracking so no stale state carries over
            _prev_pilot_steering = False

        # ── v5.5: Drift trim disabled — bang-bang handles all correction ─
        # The bang-bang controller inside compute_targets() handles all
        # heading correction.  drift_trim_lpf is always 0.
        _drift_trim_lpf = 0.0

        # ── v5.5 Obstacle Detection State Machine (AI MODE ONLY) ─────────
        #
        # ONLY active when ai_active is True.  In RC mode, all detection
        # is disabled — human_confirm decays, flight state resets to NORMAL.
        #
        # Two independent veto sources (AI mode):
        #   1. Human AI confirmation (camera geometry) → LATCHED brake
        #      Requires joystick reset to exit.
        #   2. Ultrasonic proximity < 22 cm → FORWARD-BLOCK (v5.5)
        #      Blocks forward thrust but allows left/right steering.
        #      ONLY triggers when camera also sees a bounding box
        #      (cam_height > 0).  Raw ultrasonic alone is ignored.
        #      Auto-clears when obstacle moves away.
        #
        _state_label = "NORMAL"
        _obstacle_fwd_block = False    # v5.5: forward-block flag

        if ai_active:
            _human_detected = hover_control.is_human_signature(cam_aspect, cam_height)
            _cam_sees_obj   = cam_height > 0          # camera has ANY detection
            _ultra_close    = (ultra_dist < hover_control.ULTRA_PROXIMITY_CM
                               and _cam_sees_obj)      # fused: ultrasonic + camera

            if _human_detected:
                _human_confirm = min(_human_confirm + 1,
                                     hover_control.HUMAN_CONFIRM_TICKS + 5)
            else:
                _human_confirm = max(_human_confirm - 1, 0)

            if _flight_state == _STATE_NORMAL:
                # Veto 1: Human AI confirmation (latched)
                if _human_confirm >= hover_control.HUMAN_CONFIRM_TICKS:
                    _flight_state    = _STATE_BRAKING
                    _brake_thrust_us = hover_control._thrust_current_us
                    print("[SAFETY] Human confirmed — entering STATE_BRAKING (LATCHED)")

            # v5.5: Ultrasonic proximity → FORWARD-BLOCK (not STATE_BRAKING)
            # Blocks forward thrust, allows left/right steering for evasion.
            # Auto-clears when obstacle moves beyond ULTRA_PROXIMITY_CM.
            if _ultra_close:
                _obstacle_fwd_block = True
                _state_label = "FWD_BLOCKED"

            if _flight_state == _STATE_BRAKING:
                _state_label = "BRAKING"
        else:
            # RC MODE — all detection disabled, clear any latched state
            _human_confirm = 0
            _flight_state  = _STATE_NORMAL

        # ── v5.3: Determine mode label for telemetry ──────────────────
        if not ai_active:
            _mode_label = "RC_MANUAL"
        elif _inflate_phase == _INFLATE_CALIBRATING:
            _mode_label = "AI_STABILIZED | CALIBRATING"
        elif stab_active:
            _mode_label = "AI_STABILIZED | THRUST_BOOST: ACTIVE"
        elif hover_control._op_zero_done:
            _mode_label = "AI_STABILIZED"
        else:
            _mode_label = "AI_STABILIZED | AWAITING_CAL"

        # ── v5.5: Compute spin/tilt errors for telemetry ──────────────
        _spin_err = yaw_rate - hover_control._op_zero_gz if hover_control._op_zero_done else yaw_rate
        # Force SPIN_ERR to 0 within gyro deadzone to prevent telemetry jitter
        if abs(_spin_err) < hover_control.GYRO_DEADZONE:
            _spin_err = 0.0
        hx, hy, _ = _hover_accel_offset
        _tilt_err = ax - hx
        _heading_err = hover_control.get_heading_error()   # v5.3

        # ── Throttled telemetry v5.3 — always active (200 ms) ─────────
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, _last_telem_ms) >= _TELEM_INTERVAL_MS:
            _last_telem_ms = now_ms
            estimated_rpm  = hover_control.lift_us_to_rpm(
                hover_control._lift_current_us)
            # v5.1: Primary [MODE] status indicator
            print("[MODE] {}".format(_mode_label))
            print(
                "[CONTROL] "
                "SPIN_ERR:{:+.1f} | "
                "TILT_ERR:{:+.2f} | "
                "SERVO_OUT:{}us | "
                "STATE:{}".format(
                    _spin_err,
                    _tilt_err,
                    hover_control._servo_current_us,
                    _mode_label,
                )
            )
            # v5.3: Primary telemetry — heading memory + yaw trim + stab thrust
            print(
                "[V5.4] "
                "AI:{} | "
                "HEADING_ERR:{:+.1f} | "
                "DIST:{}cm | "
                "STATE:{}".format(
                    1 if ai_active else 0,
                    _heading_err,
                    ultra_dist,
                    _state_label,
                )
            )

        # ── Failsafe: no packet yet ───────────────────────────────────
        if last_rx_ms == 0:
            hover_control.set_lift(hover_control.PWM_MIN)
            hover_control.set_thrust(hover_control.PWM_MIN)
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < CONTROL_TICK: time.sleep_ms(CONTROL_TICK - elapsed)
            continue

        # ── Failsafe: Nano link watchdog ──────────────────────────────
        if time.ticks_diff(time.ticks_ms(), last_rx_ms) > hover_control.FAILSAFE_TIMEOUT_MS:
            print("[FAILSAFE] Nano link lost — emergency stop!")
            hover_control.emergency_stop()
            _idle_since_ms      = 0
            _inflate_phase      = _INFLATE_IDLE
            _mlp_lift_lpf       = 0.0
            _mlp_error_reported = False
            _flight_state       = _STATE_NORMAL
            _human_confirm      = 0
            hover_control.reset_operational_zero()   # v5.0
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < CONTROL_TICK: time.sleep_ms(CONTROL_TICK - elapsed)
            continue

        # ── Lift state change ─────────────────────────────────────────
        if _prev_lift_on and not lift_on:
            # Lift switched OFF — reset everything
            _mlp_lift_lpf        = 0.0
            _tilt_rudder_lpf     = 0.0
            _yaw_integral        = 0.0
            _drift_trim_lpf      = 0.0
            _inflate_phase       = _INFLATE_IDLE
            _softstart_us        = 0
            _idle_since_ms       = 0
            _mlp_error_reported  = False
            _hover_accel_offset  = (0.0, 0.0, 0.0)
            _hover_cal_sum       = [0.0, 0.0, 0.0]
            _hover_cal_count     = 0
            _flight_state        = _STATE_NORMAL
            _human_confirm       = 0
            _prev_pilot_steering = False   # v5.3
            hover_control.reset_operational_zero()   # v5.0 + v5.3 heading reset
        _prev_lift_on = lift_on

        # ── 4-Phase Lift Startup State Machine (v3.9) ─────────────────
        #
        # SOFTSTART: smooth ramp PWM_MIN → LIFT_HOVER_PWM.
        #   During the last _HOVER_CAL_SAMPLES ticks of the ramp,
        #   sample the IMU to build the hover-RPM tilt baseline.
        #   This captures any constant motor-vibration offset so the
        #   tilt correction zeros itself at actual hover conditions.
        #
        # BURST: hold at 1700 µs for 0.6 s to pop the skirt bag.
        #
        # SETTLE: ramp 1700 → 1536 µs at 5 µs/tick.
        #
        # HOVER: normal control loop with hover-RPM tilt baseline.

        if do_inflate and _inflate_phase == _INFLATE_IDLE:
            _inflate_phase   = _INFLATE_SOFTSTART
            _softstart_us    = _SOFTSTART_INIT_US    # v4.2: start at 1100 µs
            _hover_cal_sum   = [0.0, 0.0, 0.0]
            _hover_cal_count = 0
            print("[CORE 1] SOFTSTART — {}us → {}us @ {}us/tick (~{}ms)".format(
                _SOFTSTART_INIT_US,
                hover_control.LIFT_HOVER_PWM,
                hover_control.LIFT_SOFTSTART_STEP_US,
                (hover_control.LIFT_HOVER_PWM - _SOFTSTART_INIT_US)
                // hover_control.LIFT_SOFTSTART_STEP_US * CONTROL_TICK))
            _shared_write(do_inflate=False)

        if _inflate_phase == _INFLATE_SOFTSTART:
            _softstart_us += hover_control.LIFT_SOFTSTART_STEP_US
            if _softstart_us >= hover_control.LIFT_HOVER_PWM:
                _softstart_us = hover_control.LIFT_HOVER_PWM

            hover_control.set_lift(_softstart_us)

            # Sample IMU during the last _HOVER_CAL_SAMPLES ticks of
            # the ramp to build hover-RPM tilt baseline.
            ticks_remaining = (hover_control.LIFT_HOVER_PWM - _softstart_us) \
                              // hover_control.LIFT_SOFTSTART_STEP_US
            if ticks_remaining <= _HOVER_CAL_SAMPLES:
                _hover_cal_sum[0] += ax
                _hover_cal_sum[1] += ay
                _hover_cal_sum[2] += az
                _hover_cal_count  += 1

            # Ramp complete → save hover baseline, transition to BURST
            if _softstart_us >= hover_control.LIFT_HOVER_PWM:
                if _hover_cal_count > 0:
                    _hover_accel_offset = (
                        _hover_cal_sum[0] / _hover_cal_count,
                        _hover_cal_sum[1] / _hover_cal_count,
                        _hover_cal_sum[2] / _hover_cal_count,
                    )
                else:
                    _hover_accel_offset = (0.0, 0.0, 1.0)
                print("[CORE 1] Hover baseline — X:{:.4f} Y:{:.4f} Z:{:.4f}".format(
                    *_hover_accel_offset))
                _inflate_phase       = _INFLATE_BURST
                _inflate_burst_ticks = hover_control.LIFT_BURST_TICKS
                print("[CORE 1] Snap BURST — {}us / {}rpm for {}ms".format(
                    hover_control.LIFT_BURST_PWM,
                    hover_control.lift_us_to_rpm(hover_control.LIFT_BURST_PWM),
                    hover_control.LIFT_BURST_TICKS * CONTROL_TICK))

            # Thrust/servo updated every tick during softstart
            _, thrust_us, servo_us, do_dip, _ = hover_control.compute_targets(
                joy_x, joy_y, lift_on, _MLP_ZERO, brake_mode, ai_mode=False)
            if ai_active and obstacle > 0:
                servo_us, thrust_us = hover_control.apply_obstacle_override(
                    servo_us, thrust_us, obstacle)
            if do_dip and not _prev_dip: hover_control.thrust_dip()
            _prev_dip = do_dip
            hover_control.set_thrust(thrust_us)
            hover_control.set_servo(servo_us)
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < CONTROL_TICK: time.sleep_ms(CONTROL_TICK - elapsed)
            continue

        if _inflate_phase == _INFLATE_BURST:
            # v5.5: burst=True allows exceeding PWM_LIFT_CAP for skirt inflation
            hover_control.set_lift(hover_control.LIFT_BURST_PWM, burst=True)
            # v5.3: Hard-reset heading memory during BURST — no integration
            hover_control.reset_heading_error()
            _inflate_burst_ticks -= 1
            if _inflate_burst_ticks <= 0:
                _inflate_phase     = _INFLATE_SETTLE
                _inflate_settle_us = hover_control.LIFT_BURST_PWM
                print("[CORE 1] Snap SETTLE — ramping to {}us / {}rpm".format(
                    hover_control.LIFT_INFLATE_PWM,
                    hover_control.lift_us_to_rpm(hover_control.LIFT_INFLATE_PWM)))
            _, thrust_us, servo_us, do_dip, _ = hover_control.compute_targets(
                joy_x, joy_y, lift_on, _MLP_ZERO, brake_mode, ai_mode=False)
            if ai_active and obstacle > 0:
                servo_us, thrust_us = hover_control.apply_obstacle_override(
                    servo_us, thrust_us, obstacle)
            if do_dip and not _prev_dip: hover_control.thrust_dip()
            _prev_dip = do_dip
            hover_control.set_thrust(thrust_us)
            hover_control.set_servo(servo_us)
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < CONTROL_TICK: time.sleep_ms(CONTROL_TICK - elapsed)
            continue

        if _inflate_phase == _INFLATE_SETTLE:
            _inflate_settle_us -= hover_control.LIFT_RAMP_STEP_US
            # v5.3: Hard-reset heading memory during SETTLE — no integration
            hover_control.reset_heading_error()
            if _inflate_settle_us <= hover_control.LIFT_INFLATE_PWM:
                _inflate_settle_us = hover_control.LIFT_INFLATE_PWM
                # v5.0: transition to CALIBRATING instead of IDLE
                _inflate_phase     = _INFLATE_CALIBRATING
                print("[CORE 1] Snap complete — entering CALIBRATING phase "
                      "({} samples)".format(hover_control.CALIBRATION_SAMPLES))
            hover_control.set_lift(int(_inflate_settle_us))
            _, thrust_us, servo_us, do_dip, _ = hover_control.compute_targets(
                joy_x, joy_y, lift_on, _MLP_ZERO, brake_mode, ai_mode=False)
            if ai_active and obstacle > 0:
                servo_us, thrust_us = hover_control.apply_obstacle_override(
                    servo_us, thrust_us, obstacle)
            if do_dip and not _prev_dip: hover_control.thrust_dip()
            _prev_dip = do_dip
            hover_control.set_thrust(thrust_us)
            hover_control.set_servo(servo_us)
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < CONTROL_TICK: time.sleep_ms(CONTROL_TICK - elapsed)
            continue

        # ── v5.0: Post-Inflation Calibration (Running Zero) ───────────
        # After SETTLE, collect 20 IMU samples to establish the true
        # operational baseline with the skirt pressurised.
        if _inflate_phase == _INFLATE_CALIBRATING:
            hover_control.set_lift(hover_control.LIFT_INFLATE_PWM)
            # v5.3: heading_error stays at 0 during calibration (not yet integrating)
            hover_control.reset_heading_error()
            if hover_control.calibrate_zero_sample(ax, ay, yaw_rate):
                # calibrate_zero_sample stores: ax for tilt-X, ay for tilt-Y,
                # gz (yaw_rate) for Gyro-Z operational zero
                # v5.3: This moment is the "Zero Position" — heading integrator
                # starts from 0.0 at this exact point.
                hover_control.reset_heading_error()
                _inflate_phase = _INFLATE_IDLE
                print("[CORE 1] Calibration complete — HEADING MEMORY LOCKED")
            # During calibration, servo/thrust respond to manual input only
            _, thrust_us, servo_us, do_dip, _ = hover_control.compute_targets(
                joy_x, joy_y, lift_on, _MLP_ZERO, brake_mode, ai_mode=False)
            if ai_active and obstacle > 0:
                servo_us, thrust_us = hover_control.apply_obstacle_override(
                    servo_us, thrust_us, obstacle)
            if do_dip and not _prev_dip: hover_control.thrust_dip()
            _prev_dip = do_dip
            hover_control.set_thrust(thrust_us)
            hover_control.set_servo(servo_us)
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < CONTROL_TICK: time.sleep_ms(CONTROL_TICK - elapsed)
            continue

        # ── v5.1: Strict AI-Mode Hardware Gate ─────────────────────────
        #
        # When ai_mode is OFF (Nano toggle switch):
        #   • MLP infer() is NOT called — saves CPU cycles
        #   • All autonomous trims hard-reset to 0
        #   • Rudder and motors respond ONLY to joystick
        # When ai_mode is ON:
        #   • Operational Zero calibration + AI stabilisation layered on
        #
        # The PI controller keeps the craft pointed straight without
        # joystick input, even when the floor surface or air asymmetry
        # causes a slow unprompted rotation.
        #
        # P term  — proportional to current tilt error.
        #           Reacts immediately to a lean. Fast.
        #
        # I term  — accumulates tilt error over time.
        #           Anti-windup: clamped to ±YAW_KI_LIMIT.
        #
        # Manual joystick (joy_x) always overrides: integrator is frozen
        # (not reset) so it resumes when stick returns to neutral.
        #
        # Deadband applied before both P and I terms.

        if ai_active:
            # ── AI MODE ON: full TinyML + PI stabilisation ────────────
            try:
                _ax_safe = ax if ax == ax else 0.0   # NaN guard

                # Tilt error relative to hover-RPM baseline
                hx, hy, _ = _hover_accel_offset
                _ax_rel = _ax_safe - hx

                # Deadband — ignore noise, protect integrator from dither
                if abs(_ax_rel) < hover_control.TILT_RUDDER_DEADBAND_G:
                    _ax_rel = 0.0

                # Freeze integrator while pilot is actively steering
                # so manual turns don't fight the I term
                if abs(joy_x) < 0.08:
                    _yaw_integral += _ax_rel * hover_control.YAW_KI
                    _yaw_integral  = max(-hover_control.YAW_KI_LIMIT,
                                     min( hover_control.YAW_KI_LIMIT,
                                          _yaw_integral))

                # PI output — invert sign: right tilt → steer left
                _p_term   = -_ax_rel * hover_control.YAW_KP * AI_GAIN
                _pi_raw   = _p_term - _yaw_integral
                _pi_raw   = max(-312.0, min(312.0, _pi_raw))

                # LPF smooths the combined PI signal
                _tilt_rudder_lpf = (
                    hover_control.TILT_RUDDER_LPF_ALPHA * _pi_raw
                    + (1.0 - hover_control.TILT_RUDDER_LPF_ALPHA) * _tilt_rudder_lpf
                )

                mlp_input = mlp_logic.normalize(
                    ax, ay, az, joy_x, joy_y, obstacle > 0,
                    yaw_rate)                               # v4.0
                raw_out = mlp_logic.infer(mlp_input, weights)

                _mlp_lift_lpf = (_LP_ALPHA * raw_out[0]
                                 + (1.0 - _LP_ALPHA) * _mlp_lift_lpf)
                _conf   = raw_out[3]

                # v4.2: combine PI tilt-rudder (drift_trim flows via
                # compute_targets() drift_trim param, not here)
                _combined_servo_trim = _tilt_rudder_lpf
                _combined_servo_trim = max(-312.0, min(312.0, _combined_servo_trim))

                mlp_out = [
                    _mlp_lift_lpf,           # lift trim (µs, LPF'd)
                    raw_out[1],              # thrust trim (µs)
                    _combined_servo_trim,    # PI tilt-rudder correction (µs)
                    _conf,                   # alert / confidence
                ]
                _mlp_error_reported = False

            except Exception as exc:
                if not _mlp_error_reported:
                    print("[ERROR] TinyML infer failed — zero trim. ({})".format(exc))
                    _mlp_error_reported = True
                _tilt_rudder_lpf = 0.0
                _yaw_integral    = 0.0
                _drift_trim_lpf  = 0.0
                mlp_out = _MLP_ZERO
                _conf   = 0.0

        else:
            # ── AI MODE OFF: pure RC — MLP dormant ────────────────────
            # infer() is NOT called — CPU cycles saved.
            # All autonomous trims hard-reset to zero.
            mlp_out          = _MLP_ZERO
            _conf            = 0.0
            _mlp_lift_lpf    = 0.0
            _tilt_rudder_lpf = 0.0
            _yaw_integral    = 0.0
            _drift_trim_lpf  = 0.0
            _prev_pilot_steering = False   # v5.3
            hover_control.reset_heading_error()   # v5.3: clear heading memory
            _mlp_error_reported = False

        # ── v5.3: Hard Safety Gate — force sensor inputs to zero ──────
        # When AI is OFF (debounced), the Pico must see gyro_z=0 and ax=0
        # so that compute_targets() cannot produce ANY autonomous yaw or
        # tilt trims.  Combined with ai_mode=False in compute_targets(),
        # this is the final guarantee of "Cold RC" mode.
        if not ai_active:
            gyro_z_for_ct = 0.0
            ax_for_ct     = 0.0
        else:
            gyro_z_for_ct = yaw_rate
            ax_for_ct     = ax

        # ── v5.5: Obstacle Forward-Block ────────────────────────────────
        # When obstacle is within 22 cm (AI mode + camera confirmed):
        #   - Forward thrust BLOCKED (joy_y clamped to 0)
        #   - Left/right steering ALLOWED (joy_x unchanged)
        #   - Rudder authority maintained via OBSTACLE_STEER_THRUST_PWM
        #   - Auto-clears when obstacle moves away
        joy_y_for_ct = joy_y
        if _obstacle_fwd_block:
            joy_y_for_ct = 0.0   # block forward, craft can only steer

        # ── Compute PWM targets (v5.5 — forward-block aware) ───────────
        # joy_x always flows here — steering is never blocked.
        # joy_y is blocked when obstacle forward-block is active.
        # ai_active is the debounced flag — compute_targets uses it
        # as a HARD GATE to block all corrections when False.
        lift_us, thrust_us, servo_us, do_dip, stab_active = hover_control.compute_targets(
            joy_x, joy_y_for_ct, lift_on, mlp_out, brake_mode,
            tilt_x=ax_for_ct, drift_trim=_drift_trim_lpf,
            gyro_z=gyro_z_for_ct, ai_mode=ai_active)

        # ── v5.5: Steering thrust during forward-block ────────────────
        # When forward is blocked and pilot is steering, provide minimum
        # thrust so the rudder has airflow to physically turn the craft.
        # Without this, joy_y=0 means thrust=idle and rudder has no bite.
        if _obstacle_fwd_block and lift_on and abs(joy_x) > 0.1:
            thrust_us = max(thrust_us, hover_control.OBSTACLE_STEER_THRUST_PWM)

        # ── Auto-idle thermal relief ──────────────────────────────────
        # After 5 s of joystick neutral, scale lift to cooling RPM.
        # SKIP when AI is active — during correction the joystick is
        # neutral but the craft still needs full hover RPM for stability.
        if lift_on and not ai_active and abs(joy_x) < 0.05 and abs(joy_y) < 0.05:
            if _idle_since_ms == 0:
                _idle_since_ms = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), _idle_since_ms) > _IDLE_TIMEOUT_MS:
                lift_us = int(lift_us * _IDLE_LIFT_SCALE)
                lift_us = max(lift_us, hover_control.PWM_MIN)
        else:
            _idle_since_ms = 0

        # ── Obstacle veto (AI MODE ONLY) ──────────────────────────────
        if ai_active and obstacle > 0:
            servo_us, thrust_us = hover_control.apply_obstacle_override(
                servo_us, thrust_us, obstacle)

        # ── v5.5 STATE_BRAKING override (Human Veto Only) ──────────────
        # Ramp thrust down, hold lift steady.
        #   Human veto: LATCHED — requires human gone + joystick reset.
        #   (Ultrasonic proximity now uses forward-block instead, v5.5)
        if _flight_state == _STATE_BRAKING:
            _brake_thrust_us -= hover_control.BRAKE_THRUST_RAMP_STEP
            if _brake_thrust_us < hover_control.PWM_MIN:
                _brake_thrust_us = hover_control.PWM_MIN
            thrust_us = _brake_thrust_us
            # lift_us untouched — skirt stays inflated

            # Human veto exit: human gone AND joystick returned to zero
            human_gone  = _human_confirm == 0
            joy_zero    = abs(joy_y) < 0.05

            if human_gone and joy_zero:
                _flight_state = _STATE_NORMAL
                print("[SAFETY] Human veto clear — resuming NORMAL")

        # ── Thrust-dip ────────────────────────────────────────────────
        if do_dip and not _prev_dip:
            hover_control.thrust_dip()
        _prev_dip = do_dip

        # ── Apply PWM ─────────────────────────────────────────────────
        hover_control.set_lift(lift_us)
        hover_control.set_thrust(thrust_us)
        hover_control.set_servo(servo_us)

        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        if elapsed < CONTROL_TICK:
            time.sleep_ms(CONTROL_TICK - elapsed)


# =====================================================================
#  ENTRY POINT
# =====================================================================

def main():
    print("=" * 60)
    print("  HOVERCRAFT AI CONTROL v5.6  —  Zero-Crossing Heading Hold")
    print("  RP2040 Dual-Core | 100 Hz | AI_GAIN=1.0")
    print("  AI-Mode Gate    : Toggle OFF → pure RC (MLP dormant)")
    print("                    Toggle ON  → AI stabilisation active")
    print("  Heading Correct : Zero-cross | Force={}us | Threshold={}deg | Cooldown={}ticks".format(
        hover_control.HEADING_CORRECTION_FORCE,
        hover_control.HEADING_CORRECTION_THRESHOLD,
        hover_control.CORRECTION_COOLDOWN_TICKS))
    print("  Gyro Filter     : Deadzone={}dps | Deadband={}dps | Sustain={}ticks".format(
        hover_control.GYRO_DEADZONE,
        hover_control.DRIFT_DEADBAND_DPS,
        hover_control.SPIN_SUSTAIN_TICKS))
    print("  Pilot Overrule  : |joy_x| > 0.1 → clear + re-lock")
    print("  Hover target    : {} µs / {} RPM".format(
        hover_control.LIFT_HOVER_PWM,
        hover_control.lift_us_to_rpm(hover_control.LIFT_HOVER_PWM)))
    print("  Hard ceiling    : {} µs / {} RPM".format(
        hover_control.LIFT_MAX_PWM,
        hover_control.lift_us_to_rpm(hover_control.LIFT_MAX_PWM)))
    print("  Stab thrust     : {}us | BRK=turn-around".format(
        hover_control.STABILIZATION_THRUST_PWM))
    print("  Stab thresholds : yaw>{}µs tilt>{}µs".format(
        hover_control.STABILIZATION_YAW_THRESHOLD,
        hover_control.STABILIZATION_TILT_THRESHOLD))
    print("  Fwd mix trim    : +{}µs (dyn min {}µs)".format(
        hover_control.FORWARD_MIX_TRIM,
        hover_control.FWD_MIX_TRIM_DYN_MIN))
    print("  Tilt-rudder     : Kp={} Ki={} deadband={}g LPF={}".format(
        hover_control.YAW_KP,
        hover_control.YAW_KI,
        hover_control.TILT_RUDDER_DEADBAND_G,
        hover_control.TILT_RUDDER_LPF_ALPHA))
    print("  Human detect    : Asp>{} H>{}px confirm={}ticks".format(
        hover_control.HUMAN_ASPECT_MIN,
        hover_control.HUMAN_HEIGHT_MIN,
        hover_control.HUMAN_CONFIRM_TICKS))
    print("  Ultrasonic veto : <{}cm proximity stop (auto-clear)".format(
        hover_control.ULTRA_PROXIMITY_CM))
    print("  Brake ramp      : {}µs/tick thrust ramp-down".format(
        hover_control.BRAKE_THRUST_RAMP_STEP))
    print("  Post-inflate cal: {} samples → Zero Position lock".format(
        hover_control.CALIBRATION_SAMPLES))
    print("  Hover baseline  : sampled at lift-on (last 30 ticks)")
    print("  Nano Link       : UART1 GP9 @ 9600 baud")
    print("  CAM Veto        : UART0 GP1 @ 115200 | threshold 20 cm")
    print("  CAM Geometry    : 0xD4 4-byte (Aspect/Height/UltraDist)")
    print("  Deadband        : 25 µs (ESC jitter guard)")
    print("  Telemetry       : [V5.4] AI | HEADING_ERR | DIST | STATE")
    print("=" * 60)

    _thread.start_new_thread(_core0_io_loop, ())
    _control_loop()


main()
