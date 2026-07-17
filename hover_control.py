"""
hover_control.py — PWM Management & Safety Logic (1700KV 2-Blade v5.4)
=======================================================================
Manages Lift BLDC (GP16), Thrust BLDC (GP17), and Servo (GP18).

All PWM values in microseconds (us).  50 Hz PWM → 20 000 us period.
ESC range: 1000 us (off) → 2000 us (full).
Servo:     500 us (0 deg) → 2500 us (180 deg).

Optimised for: Emax 1700KV | 4-Layer Garbage Bag Skirt (3 cm) | 12.6 V 3S 80C
               Reduced skirt height (3 cm vs previous) lowers the
               required hover RPM to 11 500 vs 12 531 in v3.6.
               Inflation burst shortened to 0.6 s ("Snap" burst)
               at 1700 µs — sufficient to pop the lighter 3 cm bag.

RPM / PWM reference  (1700KV × 12.6 V = 21 420 RPM at 100%)
--------------------------------------------------------------
  Formula:  RPM = ((us - 1000) / 1000) × 21 420
  1000 µs →      0 RPM  (armed, off)
  1300 µs →  6 426 RPM  ← _IDLE_LIFT_SCALE 0.85 × 1536 (ESC cooling)
  1536 µs → 11 484 RPM  ← PRIMARY HOVER TARGET (≈ 11 500 RPM) ← v3.8
  1700 µs → 14 994 RPM  ← SNAP BURST (0.6 s, 3 cm skirt pop)   ← v3.8
  1800 µs → 17 136 RPM  ← EMERGENCY / HARD CEILING (manual peak)← v3.8
  2000 µs → 21 420 RPM  (never reached)

⚠  IDLE SCALE NOTE
   _IDLE_LIFT_SCALE = 0.85 drops lift to 1305 µs / ~6 524 RPM
   (0.85 × 1536 = 1305.6 → rounded 1305 µs).
   At 12.6 V this remains above the sustain threshold for a 3 cm
   4-layer bag.  If skirt deflation is observed during the 5 s
   cooling window, raise to 0.95 (→ 1459 µs / ~9 809 RPM).

Changes in v5.4 vs v5.3
------------------------
1. Dual-Veto Braking:
   STATE_BRAKING triggered by human AI confirmation (latched, requires
   joystick reset) OR ultrasonic proximity < ULTRA_PROXIMITY_CM (25 cm,
   auto-clears when obstacle moves away).
2. ULTRA_PROXIMITY_CM constant added for ultrasonic proximity threshold.
3. All v5.3 features unchanged: Angle Integration, Pilot Overrule,
   Post-Inflation Lock-In, Max-Input Thrust Mixer.
"""

from machine import Pin, PWM
import time

# ── Constants ─────────────────────────────────────────────────────────

PWM_FREQ       = 50         # Hz — strictly 50 Hz (RC ESC standard, switching efficiency)
PWM_PERIOD_US  = 20_000
PWM_MIN        = 1000       # ESC arm / off
PWM_MAX        = 2000       # absolute max (never sent to lift in normal flight)
PWM_LIFT_CAP   = 1350       # RPM throttle cap — max lift output µs   ← v5.6
                             # 1350 µs ≈ 7 500 RPM.  Lift capped at
                             # hover RPM to prevent floating.

# ── Lift BLDC — 7 500 RPM (3 cm 4-layer skirt, sealed hull) ──────────
LIFT_MIN_PWM      = 1350    # 7 500 RPM — PRIMARY hover constant     ← v5.6
LIFT_HOVER_PWM    = 1350    # alias kept for compute_targets()        ← v5.6
LIFT_BURST_PWM    = 1350    # 7 500 RPM — same as hover              ← v5.6
                             # No burst spike — steady 7.5K RPM for
                             # inflation and flight.
LIFT_BURST_TICKS  = 30      # ticks @ 100 Hz = 0.3 s hold            ← v5.6
LIFT_INFLATE_PWM  = 1350    # settle target after burst               ← v5.6
LIFT_MAX_PWM      = 1350    # 7 500 RPM — hard ceiling                ← v5.6

# Soft-start ramp (burst → settle, driven non-blocking in Core 1)
LIFT_RAMP_STEP_US = 10      # µs per tick during settle ramp          ← v5.5
                             # Raised from 5 — settles to 13K RPM faster
                             # (1650→1607 = 4.3 ticks ≈ 43 ms)
LIFT_RAMP_TICK_MS = 10      # ms reference (= CONTROL_TICK)

# Deadband — 25 µs suppresses ESC jitter at 11 500 RPM
# Jitter at this RPM inside a sealed hull is the primary MOSFET heat source.
LIFT_DEADBAND_US  = 25      # µs  (was 30 in v3.6)                   ← v3.8

# Thrust BLDC
THRUST_IDLE_PWM   = 1100
THRUST_MAX_PWM    = 1900

# Thrust-dip manoeuvre
THRUST_DIP_PWM    = 1100
THRUST_DIP_MS     = 100

# Voltage-sag ramp rate
RAMP_STEP         = 50      # max µs change per 10 ms tick

# Servo (MG90S)
SERVO_MIN         = 500
SERVO_NEUTRAL     = 1500
SERVO_MAX         = 2500

# ── v3.9 Steering compensation ────────────────────────────────────────
#
# FORWARD_MIX_TRIM
#   Corrects leftward yaw bias during forward thrust (joy_y > 0).
#   The thrust fan torque reacts against the skirt and pulls the nose
#   left.  This constant adds a fixed rightward servo offset to cancel
#   it.  Increase in steps of 5 µs until the craft tracks straight.
#   Typical range: +15 to +30 µs.  Start at +20.
FORWARD_MIX_TRIM  = 25     # µs rightward bias during forward flight ← v4.2 tuned
#
# TILT_RUDDER_GAIN
#   Scales how aggressively the MLP roll-tilt correction moves the
#   rudder servo.  Units: µs of servo deflection per g of roll.
#   At 1.0 g of roll (craft on its side), full TILT_RUDDER_GAIN is
#   applied.  Typical in-flight tilt ≈ ±0.1–0.3 g → ±20–60 µs trim.
#   Start at 200. Raise for snappier correction, lower if oscillating.
TILT_RUDDER_GAIN  = 280    # µs/g roll correction scale            ← v5.3 raised
#
# TILT_RUDDER_DEADBAND_G
#   Tilt corrections smaller than this (in g) are ignored entirely.
#   Prevents the servo from hunting/buffering around the zero point.
#   0.03 g ≈ 1.7° tilt — below this, treat as level.
TILT_RUDDER_DEADBAND_G = 0.10   # g — ignore tilt smaller than this ← v5.4 raised
                                # 0.03 was too sensitive at 13K RPM — motor vibration
                                # read as tilt, causing false thrust boost.
#
# TILT_RUDDER_LPF_ALPHA
#   Low-pass filter coefficient for the tilt-rudder signal.
#   Lower = smoother but slower response. Higher = faster but noisier.
#   0.15 gives ~65 ms lag at 100 Hz — smooth without being sluggish.
TILT_RUDDER_LPF_ALPHA  = 0.25   # LPF on tilt-rudder trim           ← v5.3 faster
#
# Yaw PI controller gains
# P gain: µs of rudder per g of tilt error. 150 = gentle correction.
# I gain: µs of rudder per g·tick of accumulated error.
#         At 100 Hz, 1 tick = 10 ms. 0.8 µs/(g·tick) accumulates
#         ~8 µs/s per 0.1g of persistent tilt — slow but persistent.
# I limit: caps the integrator at ±80 µs to prevent windup.
YAW_KP       = 200.0    # µs/g  proportional gain               ← v5.3 raised
YAW_KI       = 2.0      # µs/(g·tick) integral gain             ← v5.3 raised
YAW_KI_LIMIT = 100.0    # µs  integrator clamp (anti-windup)    ← v5.3 raised
JOY_DEADZONE       = 0.05      # joystick deadzone for manual inputs  ← v5.5
                                # Inputs within ±0.05 are forced to 0
                                # to prevent jitter in yaw_trim.
GYRO_DEADZONE      = 5.0       # IMU gyro deadzone (°/s)              ← v5.5
                                # Aggressively filters motor vibration
                                # (1–5°/s).  Only real spins (>5°/s)
                                # pass through.  Applied to spin_err_dps.
#
# LIFT_SOFTSTART_STEP_US
#   µs added to lift PWM per tick during soft-start ramp.
#   At 100 Hz: 10 µs/tick × ~54 ticks = ~0.54 s to reach hover.
#   This prevents the voltage sag spike that previously caused
#   IMU vibration at lift-on.
LIFT_SOFTSTART_STEP_US = 10     # µs/tick soft ramp to hover RPM    ← v4.2

# ── v5.6 Heading Correction (Zero-Crossing with Cooldown) ────────────
#
# HEADING_CORRECTION_FORCE
#   Fixed rudder deflection in µs applied during heading correction.
#   NOT proportional to error — avoids jitter from noise amplification.
#   When heading error exceeds threshold, this FIXED amount is applied
#   to the servo in the direction needed to push the craft back.
#   250 µs is a firm correction without being violent.
HEADING_CORRECTION_FORCE = 250.0    # µs fixed rudder deflection     ← v5.6
#
# HEADING_CORRECTION_THRESHOLD
#   Heading error (degrees) to ACTIVATE correction.
#   Below this, the AI does NOTHING — pure manual control.
HEADING_CORRECTION_THRESHOLD = 20.0 # degrees — activate correction  ← v5.6
#
# DRIFT_DEADBAND_DPS
#   Gyro-Z rates below this (°/s) are NOT integrated into heading error.
#   Filters vibration noise so only real spins accumulate.
DRIFT_DEADBAND_DPS = 10.0      # °/s — only integrate above this     ← v5.6
#
# HEADING_ERROR_LIMIT
#   Caps the heading error accumulator to prevent runaway.
HEADING_ERROR_LIMIT = 90.0     # degrees — anti-windup clamp          ← v5.6
#
# HEADING_DECAY_RATE
#   When NOT correcting, heading_error decays each tick to prevent
#   vibration from slowly walking error up to the threshold.
HEADING_DECAY_RATE = 0.998      # per-tick decay when passive          ← v5.6
                                # 0.93 was too aggressive — error could
                                # never reach 20° threshold.  0.998 gives
                                # ~3.5 s half-life: real spins accumulate
                                # while vibration walk-up still decays.
#
# SPIN_SUSTAIN_TICKS
#   Consecutive ticks gyro must read above deadband before integrating.
#   Filters vibration spikes (1-2 ticks) while passing real spins.
SPIN_SUSTAIN_TICKS = 5          # ticks (50ms) before integrating      ← v5.6
#
# CORRECTION_COOLDOWN_TICKS
#   After correction disengages (heading crossed zero), ignore heading
#   error for this many ticks.  Allows momentum to settle before the
#   system starts accumulating error again.  Prevents the return
#   momentum from immediately re-triggering correction in the opposite
#   direction (the "spin-back" problem).
#   30 ticks = 300 ms at 100 Hz.
CORRECTION_COOLDOWN_TICKS = 30  # ticks (300ms) cooldown after correction ← v5.6
#
# FWD_MIX_TRIM_DYN_MIN
#   Minimum FORWARD_MIX_TRIM at low throttle.  Ensures the rudder has
#   enough "bite" to counteract wind even when joy_y is near zero.
#   Dynamic scaling: fwd_mix = max(FWD_MIX_TRIM_DYN_MIN, joy_y * FORWARD_MIX_TRIM)
FWD_MIX_TRIM_DYN_MIN = 12     # µs — minimum fwd mix at low throttle ← v4.2 tuned

# ── v5.5 AI Torque Offset ─────────────────────────────────────────────
#
# TORQUE_OFFSET_GAIN
#   In AI_STABILIZED mode, the lift motor's rotational torque biases
#   the craft's yaw.  As lift RPM increases, apply a proportional
#   counter-bias to the steering servo to cancel the torque reaction.
#
#   torque_offset = ((lift_us - PWM_MIN) / (PWM_LIFT_CAP - PWM_MIN))
#                   × TORQUE_OFFSET_GAIN
#
#   At hover (1397 µs):  (397/397) × 35 ≈ +35 µs rightward bias
#   At idle  (1000 µs):  0 µs (no motor torque)
#
#   The sign convention matches FORWARD_MIX_TRIM: positive = rightward
#   servo offset to counteract the leftward motor torque reaction.
#   Tune in steps of 5 µs.  Start at 35.
TORQUE_OFFSET_GAIN = 35.0     # µs max torque counter-bias           ← v5.5

# ── v4.2 Human Detection (ESP32-CAM 0xD4 Geometry Packet) ────────────
#
# is_human_signature() thresholds — derived from laboratory telemetry.
# Aspect ≈ bounding-box width/height.  Human silhouette > 0.6.
# Height ≈ bounding-box pixel height.  Person within range > 100px.
HUMAN_ASPECT_MIN     = 0.6     # aspect ratio threshold               ← v4.2
HUMAN_HEIGHT_MIN     = 100     # pixel height threshold               ← v4.2
HUMAN_CONFIRM_TICKS  = 8       # consecutive ticks to confirm human   ← v4.2 tuned
#
# BRAKE_THRUST_RAMP_STEP
#   µs/tick ramp-down for thrust during STATE_BRAKING.
#   At 100 Hz: 15 µs/tick → 900 µs drop/s → ESC reaches idle in ~60 ticks.
#   Lift is held steady during braking to prevent skirt collapse.
BRAKE_THRUST_RAMP_STEP = 20    # µs/tick thrust ramp-down in braking ← v4.2 tuned

# ── v5.4 Ultrasonic Proximity Veto ────────────────────────────────────
#
# ULTRA_PROXIMITY_CM
#   Distance threshold for the ultrasonic proximity stop.
#   Craft enters STATE_BRAKING when ultra_dist < this value.
#   Unlike the human veto (latched), the proximity stop auto-clears
#   when the obstacle moves beyond this distance.
ULTRA_PROXIMITY_CM     = 22     # cm — proximity stop threshold       ← v5.4

# ── v5.5 Obstacle Forward-Block ──────────────────────────────────────
#
# OBSTACLE_STEER_THRUST_PWM
#   When an obstacle is within ULTRA_PROXIMITY_CM, forward thrust is
#   blocked but the pilot can still steer left/right to evade.  This
#   constant sets the minimum thrust provided for rudder authority
#   when the pilot is actively steering during a forward-block.
#   Must be high enough for the rudder to bite — 1350 µs gives ~7 500 RPM
#   of prop wash over the rudder, sufficient for a 90° turn.
OBSTACLE_STEER_THRUST_PWM = 1350   # µs — steering thrust during fwd-block ← v5.5

# ── v5.0 Active Closed-Loop Stabilization ─────────────────────────────
#
# Dynamic Thrust Authority ("Bite" Logic)
#   When the AI tries to correct a spin or tilt while the pilot is at
#   low throttle, the rudder has no airflow to physically execute the
#   correction.  stabilization_active = True triggers a thrust override
#   to STABILIZATION_THRUST_PWM, giving the rudder enough "bite" to work.
#   The boost drops back as soon as the craft straightens out.
GYRO_Z_THRUST_BOOST_THRESHOLD = 15.0   # °/s — boost above this spin rate ← v5.0
GYRO_Z_THRUST_BOOST_US        = 80     # µs added to thrust for rudder authority ← v5.0
GYRO_Z_THRUST_BOOST_MAX_JOY_Y = 0.10   # only boost when joy_y below this ← v5.0
#
# Stabilization Thrust Authority ("Bite" override)
#   When stabilization_active is True and pilot thrust is below this value,
#   thrust is overridden to ensure the rudder has airflow for correction.
#   v5.5: lowered from 1900 → 1500.  Full thrust (1900) with high gain
#   caused violent snap-back and overshoot.  1500µs provides enough
#   airflow for the rudder without building excessive angular momentum.
STABILIZATION_THRUST_PWM = 1400  # µs — gentle thrust during correction   ← v5.5 tuned
                                 # Lowered from 1500 to reduce angular momentum
                                 # during heading return.
STABILIZATION_YAW_THRESHOLD  = 8.0   # µs — yaw_trim above this triggers boost   ← v5.1
STABILIZATION_TILT_THRESHOLD = 15.0  # µs — tilt_trim above this triggers boost  ← v5.1
#
# Post-Inflation Calibration
#   After skirt pressurises, the craft sits differently.  A 20-sample
#   "Running Zero" captures the true operational baseline.
CALIBRATION_SAMPLES = 20       # running-zero sample count             ← v5.0

# Failsafe
FAILSAFE_TIMEOUT_MS = 500


# ── Human signature classifier ────────────────────────────────────────

def is_human_signature(aspect, height):
    """
    Return True if ESP32-CAM geometry matches a standing human.

    Based on laboratory telemetry:
      - Aspect ratio > 0.6  (tall, narrow bounding box)
      - Height > 100 px     (person close enough to matter)

    Called once per tick from main.py control loop.
    """
    return aspect > HUMAN_ASPECT_MIN and height > HUMAN_HEIGHT_MIN


# ── RPM estimation (used by main.py telemetry) ────────────────────────

def lift_us_to_rpm(us):
    """
    Estimate lift motor RPM from PWM pulse width.
    Formula: ((us - 1000) / 1000) × 21 420
    Returns 0 for us ≤ 1000 (motor off / armed).
    Calibrated for 1700KV motor on 12.6 V 3S pack.
    """
    return max(0, int(((us - 1000) / 1000) * 21420))


# ── PWM duty helpers ──────────────────────────────────────────────────

def _us_to_duty(us):
    us = max(PWM_MIN, min(PWM_MAX, us))
    return int(us * 65535 // PWM_PERIOD_US)


def _us_to_duty_servo(us):
    us = max(SERVO_MIN, min(SERVO_MAX, us))
    return int(us * 65535 // PWM_PERIOD_US)


# ── Module state ──────────────────────────────────────────────────────

_lift_pwm   = None
_thrust_pwm = None
_servo_pwm  = None

_lift_current_us   = PWM_MIN
_thrust_current_us = PWM_MIN
_servo_current_us  = SERVO_NEUTRAL

# ── Post-Inflation Operational Zero (v5.0) ────────────────────────────
# Set by calibrate_zero_sample() after skirt inflation settles.
# When complete, these replace the boot-time offsets for active correction.
_op_zero_ax   = 0.0
_op_zero_ay   = 0.0
_op_zero_gz   = 0.0     # Gyro-Z zero in °/s
_op_zero_done = False
_op_zero_buf  = []      # accumulator during calibration

# ── v5.6 Heading Memory (Angular Displacement Accumulator) ────────────
# Tracks the craft's angular displacement from its locked "Zero Position".
# heading_error += (gyro_z - op_zero_gz) * dt  every tick (dt = 0.01 s).
# Positive = nose has rotated CW from lock position.
# Reset to 0 on pilot overrule, post-inflate calibration, lift-off/failsafe.
_target_heading_error  = 0.0
_spin_sustain_count    = 0       # consecutive ticks above deadband
_correction_engaged    = False   # correction active latch
_correction_direction  = 0       # +1 or -1: which way we're pushing back
_cooldown_remaining    = 0       # ticks remaining in post-correction cooldown


# ── Initialisation ────────────────────────────────────────────────────

def init_pwm(lift_pin=16, thrust_pin=17, servo_pin=18):
    """
    Configure all three channels at exactly 50 Hz.
    Explicit .freq() call on every channel prevents inheriting a
    higher system default that would increase MOSFET switching losses.
    50 Hz is RC ESC standard — do NOT raise this value.
    """
    global _lift_pwm, _thrust_pwm, _servo_pwm
    global _lift_current_us, _thrust_current_us, _servo_current_us

    _lift_pwm   = PWM(Pin(lift_pin))
    _thrust_pwm = PWM(Pin(thrust_pin))
    _servo_pwm  = PWM(Pin(servo_pin))

    # Lock all channels to 50 Hz — switching efficiency guard
    _lift_pwm.freq(PWM_FREQ)
    _thrust_pwm.freq(PWM_FREQ)
    _servo_pwm.freq(PWM_FREQ)

    _lift_current_us   = PWM_MIN
    _thrust_current_us = PWM_MIN
    _servo_current_us  = SERVO_NEUTRAL

    _lift_pwm.duty_u16(_us_to_duty(PWM_MIN))
    _thrust_pwm.duty_u16(_us_to_duty(PWM_MIN))
    _servo_pwm.duty_u16(_us_to_duty_servo(SERVO_NEUTRAL))


# ── Rate-limited setters ──────────────────────────────────────────────

def set_lift(target_us, burst=False):
    """
    Move lift BLDC toward target_us with thermal guards:
      • Hard cap at LIFT_MAX_PWM (1677 µs / 14 500 RPM)  ← v5.5
        → Joystick full-forward is capped here.
      • burst=True: allows up to LIFT_BURST_PWM (1750 µs) for
        the 0.6 s skirt inflation snap.  Normal flight NEVER
        passes burst=True — only the inflate state machine does.
      • Rate limit: RAMP_STEP (50 µs/tick) — voltage-sag protection.
      • 25 µs deadband: suppresses ESC jitter at hover RPM.
    """
    global _lift_current_us
    # v5.5: burst flag allows inflate phase to exceed normal cap
    if burst:
        cap = LIFT_BURST_PWM
    else:
        cap = min(LIFT_MAX_PWM, PWM_LIFT_CAP)
    target_us = max(PWM_MIN, min(cap, int(target_us)))

    if abs(target_us - _lift_current_us) <= LIFT_DEADBAND_US:
        return _lift_current_us

    delta = target_us - _lift_current_us
    if abs(delta) > RAMP_STEP:
        _lift_current_us += RAMP_STEP if delta > 0 else -RAMP_STEP
    else:
        _lift_current_us = target_us

    _lift_pwm.duty_u16(_us_to_duty(_lift_current_us))
    return _lift_current_us


def set_thrust(target_us):
    global _thrust_current_us
    target_us = max(PWM_MIN, min(THRUST_MAX_PWM, int(target_us)))
    delta = target_us - _thrust_current_us
    if abs(delta) > RAMP_STEP:
        _thrust_current_us += RAMP_STEP if delta > 0 else -RAMP_STEP
    else:
        _thrust_current_us = target_us
    _thrust_pwm.duty_u16(_us_to_duty(_thrust_current_us))
    return _thrust_current_us


def set_servo(target_us):
    global _servo_current_us
    _servo_current_us = max(SERVO_MIN, min(SERVO_MAX, int(target_us)))
    _servo_pwm.duty_u16(_us_to_duty_servo(_servo_current_us))
    return _servo_current_us


# ── Thrust-Dip Manoeuvre ──────────────────────────────────────────────

def thrust_dip():
    """100 ms dip at 10% during 180° servo rotation."""
    global _thrust_current_us
    _thrust_current_us = THRUST_DIP_PWM
    _thrust_pwm.duty_u16(_us_to_duty(THRUST_DIP_PWM))
    time.sleep_ms(THRUST_DIP_MS)


# ── Inflate Skirt stub ────────────────────────────────────────────────

def inflate_skirt():
    """
    STUB — API compatibility only.
    Snap burst + settle sequence runs non-blocking in Core 1 (main.py v3.8).
    Core 0 sets shared['do_inflate']=True on lift rising edge.
    Core 1 state machine:
      BURST phase  : 60 ticks @ 1700 µs (0.6 s "Snap" burst).
      SETTLE phase : ramp from 1700 → 1536 µs at 5 µs/tick (0.33 s).
    """
    pass


# ── Post-Inflation Calibration ────────────────────────────────────────

def calibrate_zero_sample(ax, ay, gz):
    """
    Accumulate one IMU sample for the post-inflation Running Zero (v5.0).

    Call once per control tick after INFLATE_SETTLE completes.
    Returns True when CALIBRATION_SAMPLES have been collected and the
    operational zero is stored.

    The operational zero represents "inflated, level, stationary" —
    the true baseline for all active corrections.
    """
    global _op_zero_ax, _op_zero_ay, _op_zero_gz, _op_zero_done
    _op_zero_buf.append((ax, ay, gz))
    if len(_op_zero_buf) >= CALIBRATION_SAMPLES:
        n = len(_op_zero_buf)
        _op_zero_ax = sum(s[0] for s in _op_zero_buf) / n
        _op_zero_ay = sum(s[1] for s in _op_zero_buf) / n
        _op_zero_gz = sum(s[2] for s in _op_zero_buf) / n
        _op_zero_done = True
        _op_zero_buf.clear()
        print("[CAL] Operational Zero SET — ax:{:.4f} ay:{:.4f} gz:{:.2f}°/s".format(
            _op_zero_ax, _op_zero_ay, _op_zero_gz))
        return True
    return False


def reset_operational_zero():
    """Reset calibration state AND heading memory (called on lift-off / failsafe)."""
    global _op_zero_ax, _op_zero_ay, _op_zero_gz, _op_zero_done
    global _target_heading_error, _spin_sustain_count
    global _correction_engaged, _correction_direction, _cooldown_remaining
    _op_zero_ax   = 0.0
    _op_zero_ay   = 0.0
    _op_zero_gz   = 0.0
    _op_zero_done = False
    _op_zero_buf.clear()
    _target_heading_error  = 0.0
    _spin_sustain_count    = 0
    _correction_engaged    = False
    _correction_direction  = 0
    _cooldown_remaining    = 0


def reset_heading_error():
    """Clear heading memory accumulator (called on pilot overrule)."""
    global _target_heading_error, _spin_sustain_count, _correction_engaged
    global _correction_direction, _cooldown_remaining
    _target_heading_error  = 0.0
    _spin_sustain_count    = 0
    _correction_engaged    = False
    _correction_direction  = 0
    _cooldown_remaining    = 0


def get_heading_error():
    """Return current heading error in degrees (for telemetry)."""
    return _target_heading_error


# ── Target Computation (v5.3 — Angle Integration + Max-Input Mixer) ───

def compute_targets(joy_x, joy_y, lift_on, mlp_out, brake_mode=False,
                    tilt_x=0.0, drift_trim=0.0, gyro_z=0.0, ai_mode=True):
    """
    Map joystick + MLP trims + heading memory + tilt to PWM (v5.5).

    Parameters
    ----------
    joy_x, joy_y   : joystick axes [-1, 1]
    lift_on        : bool — lift motor enabled
    mlp_out        : [lift_trim, thrust_trim, servo_trim, alert]
                     servo_trim is the MLP tilt-rudder correction in µs,
                     already scaled by AI_GAIN in main.py before this call.
    brake_mode     : bool — reverse / brake manoeuvre
    tilt_x         : calibrated roll tilt in g.  Forced 0 when AI OFF.
    drift_trim     : heading-hold correction in µs.  Forced 0 when AI OFF.
    gyro_z         : Gyro-Z in °/s (calibrated).  Forced 0 when AI OFF.
    ai_mode        : bool — explicit AI-mode gate (v5.3).  When False,
                     ALL autonomous corrections are physically blocked
                     inside this function, regardless of internal state.

    v5.5 changes:
      • BRK immediate override: brake_mode kills ALL motor output and
        AI functions instantly — lift to hover hold, thrust to idle,
        stabilization forced off.
      • Torque Offset: In AI_STABILIZED mode, a proportional counter-bias
        is applied to the steering servo as lift RPM increases, cancelling
        motor rotational torque.
      • Deadzone enforcement: joy_x/joy_y within JOY_DEADZONE and
        gyro_z within GYRO_DEADZONE are forced to 0 at entry.
      • Stabilization only runs when ai_mode is True.

    Angle Integration (v5.3 — Heading Memory)
    ──────────────────────────────────────────
    Every tick:  heading_error += (gyro_z - op_zero_gz) * 0.01
    Yaw trim  =  -(heading_error * HEADING_HOLD_GAIN)
    The rudder stays deflected until the craft rotates back to its
    "Zero Position" — not just until it stops spinning.

    Pilot Overrule:  |joy_x| > 0.1  →  heading_error = 0 (caller handles)
    Post-Inflate Lock-In:  heading_error hard-reset during BURST/SETTLE.

    Max-Input Thrust Mixer (v5.3)
    ─────────────────────────────
    thrust_us = max(manual_thrust, STABILIZATION_THRUST_PWM)
    Pilot throttle is NEVER reduced — only boosted when correction
    demands more airflow than the pilot is currently providing.

    Cold RC guarantee (v5.3 hardened)
    ─────────────────────────────────
    When ai_mode=False:
      • heading_error forced to 0 (kills any residual integration)
      • yaw_trim forced to 0
      • servo_trim, drift_trim, thrust_trim forced to 0
      • stabilization_active forced to False
    Triple-redundant: main.py zeroes inputs + ai_mode gate here + debounce.

    Returns (lift_us, thrust_us, servo_us, do_dip, stabilization_active).
    """
    global _target_heading_error, _spin_sustain_count, _correction_engaged
    global _correction_direction, _cooldown_remaining

    lift_trim   = mlp_out[0]
    thrust_trim = mlp_out[1]
    servo_trim  = mlp_out[2]   # MLP tilt-rudder correction, µs
    do_dip = False

    # ── v5.6: Input Deadzone Enforcement ──────────────────────────────
    if abs(joy_x) < JOY_DEADZONE:
        joy_x = 0.0
    if abs(joy_y) < JOY_DEADZONE:
        joy_y = 0.0

    # ── BRK TURN-AROUND MODE ──────────────────────────────────────────
    if brake_mode:
        _target_heading_error  = 0.0
        _spin_sustain_count    = 0
        _correction_engaged    = False
        _correction_direction  = 0
        _cooldown_remaining    = 0
        lift_us  = LIFT_HOVER_PWM if lift_on else PWM_MIN
        servo_us = SERVO_NEUTRAL + joy_x * 500
        servo_us = max(SERVO_MIN, min(SERVO_MAX, int(servo_us)))
        if abs(joy_x) > JOY_DEADZONE and lift_on:
            thrust_us = OBSTACLE_STEER_THRUST_PWM
        else:
            thrust_us = PWM_MIN
        return (lift_us, thrust_us, servo_us, False, False)

    # ── HARD AI-MODE GATE (Cold RC guarantee) ─────────────────────────
    if not ai_mode:
        _target_heading_error  = 0.0
        _correction_engaged    = False
        _correction_direction  = 0
        _cooldown_remaining    = 0
        servo_trim  = 0.0
        drift_trim  = 0.0
        thrust_trim = 0.0
        lift_trim   = 0.0
        gyro_z      = 0.0

    # ── v5.6: Angular Displacement Integration (Heading Memory) ────────
    DT = 0.01  # seconds per tick at 100 Hz

    spin_err_dps = (gyro_z - _op_zero_gz) if ai_mode else 0.0

    # Apply deadzone to spin_err_dps (the ERROR), not raw gyro_z.
    if abs(spin_err_dps) < GYRO_DEADZONE:
        spin_err_dps = 0.0

    # ── Cooldown timer ────────────────────────────────────────────────
    # After correction ends (heading crossed zero), we suppress all
    # error accumulation for CORRECTION_COOLDOWN_TICKS.  This lets
    # the craft's angular momentum dissipate before we start tracking
    # heading drift again.  Without this, the return momentum
    # immediately re-triggers correction in the opposite direction.
    if _cooldown_remaining > 0:
        _cooldown_remaining -= 1
        _target_heading_error = 0.0
        _spin_sustain_count   = 0

    # Two-tier deadband:
    #   NOT correcting: DRIFT_DEADBAND_DPS — only real spins accumulate
    #   IS correcting:  5°/s — return rotation tracked for recovery
    effective_deadband = 5.0 if _correction_engaged else DRIFT_DEADBAND_DPS

    # Sustained-spin gate — require SPIN_SUSTAIN_TICKS consecutive
    # ticks above deadband before integrating.
    if abs(spin_err_dps) > effective_deadband and ai_mode and _cooldown_remaining == 0:
        _spin_sustain_count = min(_spin_sustain_count + 1,
                                  SPIN_SUSTAIN_TICKS + 10)
    else:
        _spin_sustain_count = 0

    if _spin_sustain_count >= SPIN_SUSTAIN_TICKS and ai_mode and _cooldown_remaining == 0:
        _target_heading_error += spin_err_dps * DT
        _target_heading_error = max(-HEADING_ERROR_LIMIT,
                                    min(HEADING_ERROR_LIMIT,
                                        _target_heading_error))

    # Heading error decay — when NOT actively correcting and not in
    # cooldown, decay heading_error toward 0 each tick.
    if not _correction_engaged and _cooldown_remaining == 0:
        _target_heading_error *= HEADING_DECAY_RATE

    # ── v5.6: Heading Correction Gate (Zero-Crossing with Cooldown) ───
    #
    # KEY FIX: The old bang-bang + hysteresis caused perpetual oscillation
    # because the craft's RETURN spin was re-integrated as new heading
    # error in the opposite direction, immediately re-triggering
    # correction (30° right → correct left → 30° overshoot left → loop).
    #
    # New logic:
    #   1. ACTIVATE when |heading_error| crosses THRESHOLD (20°).
    #      Record which direction we're correcting FROM (the sign).
    #   2. Apply fixed rudder deflection to push craft back to zero.
    #   3. DEACTIVATE via ZERO-CROSSING: when heading_error sign flips
    #      (or reaches ±2°), it means the craft has returned past its
    #      locked heading.  Immediately:
    #        a) Disengage correction
    #        b) Zero heading_error (no residual to re-trigger)
    #        c) Start cooldown timer (300ms) to let momentum die
    #   4. During cooldown: heading_error forced to 0, no integration,
    #      no correction.  Pure manual control.
    #   5. After cooldown: fresh start — accumulator at 0, ready to
    #      detect new real drift.
    #
    if ai_mode:
        if not _correction_engaged:
            # NOT correcting — check if error has grown past threshold
            if abs(_target_heading_error) >= HEADING_CORRECTION_THRESHOLD:
                _correction_engaged   = True
                # Record the sign: +1 if drifted right, -1 if drifted left
                _correction_direction = 1 if _target_heading_error > 0 else -1
                print("[V5.6] HEADING CORRECTION ON — err:{:+.1f}° dir:{}".format(
                    _target_heading_error, _correction_direction))
        else:
            # Currently correcting — check for zero-crossing
            # Zero-crossing = heading error has changed sign OR is within ±2°
            # of zero (close enough to call it centered).
            crossed_zero = False
            if abs(_target_heading_error) < 2.0:
                crossed_zero = True
            elif _correction_direction > 0 and _target_heading_error < 0:
                crossed_zero = True   # Was positive (right), now negative — crossed
            elif _correction_direction < 0 and _target_heading_error > 0:
                crossed_zero = True   # Was negative (left), now positive — crossed

            if crossed_zero:
                print("[V5.6] HEADING CORRECTION OFF — zero crossed, cooldown {}ticks".format(
                    CORRECTION_COOLDOWN_TICKS))
                _correction_engaged    = False
                _correction_direction  = 0
                _target_heading_error  = 0.0
                _cooldown_remaining    = CORRECTION_COOLDOWN_TICKS
    else:
        _correction_engaged   = False
        _correction_direction = 0
        _cooldown_remaining   = 0

    correction_active = _correction_engaged and ai_mode

    # ── Yaw trim (v5.6 — fixed deflection, symmetric return) ──────────
    # When correction is active: apply a FIXED rudder force OPPOSITE
    # to the drift direction to push the craft back to center.
    #   _correction_direction = +1 (drifted right) → -250µs (steer left)
    #   _correction_direction = -1 (drifted left)  → +250µs (steer right)
    # Sign is LOCKED at activation — does NOT flip as heading_error
    # passes through zero, because we disengage at zero-crossing.
    if correction_active and abs(joy_x) < JOY_DEADZONE:
        yaw_trim = -_correction_direction * HEADING_CORRECTION_FORCE
    else:
        yaw_trim = 0.0

    # ── Heading memory is SOLE steering authority ─────────────────────
    drift_trim  = 0.0
    servo_trim  = 0.0
    if not correction_active:
        thrust_trim = 0.0
        lift_trim   = 0.0

    # ── Stabilisation flag = correction active (no false triggers) ────
    stabilization_active = correction_active and abs(joy_x) < JOY_DEADZONE

    # ── v5.3: Max-Input Thrust Mixer ───────────────────────────────
    # Compute the pilot's requested thrust as a linear PWM value.
    # joy_y  0.0 → 1000µs (idle/off)
    # joy_y  0.5 → 1500µs (50% throttle)
    # joy_y  1.0 → 2000µs (full throttle)
    base_thrust = int(joy_y * 1000) + PWM_MIN   # maps [0..1] → [1000..2000]
    base_thrust = max(PWM_MIN, min(THRUST_MAX_PWM, base_thrust))

    # ── Lift ──────────────────────────────────────────────────────────
    if lift_on:
        lift_us = LIFT_HOVER_PWM + lift_trim
        lift_us = max(LIFT_MIN_PWM, min(LIFT_MAX_PWM, lift_us))
    else:
        lift_us = PWM_MIN

    # ── Servo mixing (v5.3.2 refined) ────────────────────────────────
    #   servo_us = SERVO_NEUTRAL + (joy_x * 500) + autonomous_trims
    #   autonomous_trims are zero when correction is NOT active.
    ai_trim = servo_trim + drift_trim + yaw_trim

    # ── v5.5: AI Torque Offset ────────────────────────────────────────
    # In AI_STABILIZED mode, apply a proportional counter-bias to the
    # steering servo to counteract the lift motor's rotational torque.
    # The offset scales linearly from 0 (motor off) to TORQUE_OFFSET_GAIN
    # (motor at PWM_LIFT_CAP).  Only active when ai_mode is True.
    torque_offset = 0.0
    if ai_mode and lift_on:
        lift_fraction = max(0.0, (_lift_current_us - PWM_MIN)
                                / max(1, PWM_LIFT_CAP - PWM_MIN))
        torque_offset = lift_fraction * TORQUE_OFFSET_GAIN

    # ── Thrust / Servo per flight mode ───────────────────────────────
    # NOTE: brake_mode case removed — BRK now returns early above.
    if not lift_on:
        thrust_us = PWM_MIN
        servo_us  = SERVO_NEUTRAL

    elif joy_y > 0.05:
        # Forward flight
        fwd_mix   = max(FWD_MIX_TRIM_DYN_MIN, joy_y * FORWARD_MIX_TRIM)
        thrust_us = base_thrust + thrust_trim
        servo_us  = SERVO_NEUTRAL + joy_x * 500 + fwd_mix + ai_trim + torque_offset

    elif joy_y < -0.05:
        do_dip    = True
        thrust_us = THRUST_IDLE_PWM + abs(joy_y) * 200 + thrust_trim
        servo_us  = (SERVO_MIN  if joy_x <= -0.1 else
                     SERVO_MAX  if joy_x >=  0.1 else SERVO_MIN) + ai_trim + torque_offset

    else:
        # Hovering / neutral — pilot at ~0% throttle
        thrust_us = base_thrust
        servo_us  = SERVO_NEUTRAL + joy_x * 500 + ai_trim + torque_offset

    # ── v5.3.2: Full Thrust During Active Correction ──────────────────
    # When the AI detects a real spin (heading_error > 5°), thrust goes
    # to FULL (1900µs) so the rudder has maximum airflow to counter-steer.
    # Pilot's own thrust is NEVER reduced — only overridden upward.
    # When NOT correcting: thrust is 100% pilot-controlled.
    # v5.5: stabilization_active is always False when ai_mode is OFF.
    if stabilization_active and lift_on:
        thrust_us = max(thrust_us, STABILIZATION_THRUST_PWM)

    # Clamp thrust to safe range
    thrust_us = max(PWM_MIN, min(THRUST_MAX_PWM, int(thrust_us)))

    # ── v5.5: Lift RPM cap — enforce PWM_LIFT_CAP (1780 µs) ──────────
    lift_us = min(lift_us, PWM_LIFT_CAP)

    # ── Final servo clamp ─────────────────────────────────────────────
    servo_us = max(SERVO_MIN, min(SERVO_MAX, int(servo_us)))

    return (lift_us, thrust_us, servo_us, do_dip, stabilization_active)


# ── Obstacle Override (v5.0 — safety integration) ────────────────────

def apply_obstacle_override(servo_us, thrust_us, obstacle_code):
    """
    Force servo/thrust based on ESP32-Cam obstacle veto code.

    Obstacle commands override ALL stabilisation trims (safety > autonomy).
    This ensures the human-safety system always has final authority over
    any PID/MLP/yaw correction that might fight the evasive manoeuvre.

    Args:
        servo_us:       int  current servo target (µs) — includes all trims
        thrust_us:      int  current thrust target (µs)
        obstacle_code:  int  0=clear, 1=center-brake, 2=left, 3=right

    Returns:
        (servo_us, thrust_us) — adjusted targets
    """
    if obstacle_code == 1:
        # CENTER obstacle → full brake: servo to mechanical stop, thrust OFF
        print("[SAFETY] CENTER obstacle — BRAKE | servo→{}us thrust→{}us".format(
            SERVO_MIN, PWM_MIN))
        return (SERVO_MIN, PWM_MIN)
    elif obstacle_code == 2:
        # LEFT obstacle → steer hard RIGHT (overrides all yaw/tilt trims)
        sv = max(SERVO_NEUTRAL + 300, min(SERVO_MAX, servo_us + 400))
        print("[SAFETY] LEFT obstacle — steer RIGHT {}us".format(sv))
        return (sv, thrust_us)
    elif obstacle_code == 3:
        # RIGHT obstacle → steer hard LEFT
        sv = min(SERVO_NEUTRAL - 300, max(SERVO_MIN, servo_us - 400))
        print("[SAFETY] RIGHT obstacle — steer LEFT {}us".format(sv))
        return (sv, thrust_us)
    return (servo_us, thrust_us)


# ── Emergency Stop ────────────────────────────────────────────────────

def emergency_stop():
    global _lift_current_us, _thrust_current_us, _servo_current_us
    _lift_current_us   = PWM_MIN
    _thrust_current_us = PWM_MIN
    _servo_current_us  = SERVO_NEUTRAL
    if _lift_pwm:   _lift_pwm.duty_u16(_us_to_duty(PWM_MIN))
    if _thrust_pwm: _thrust_pwm.duty_u16(_us_to_duty(PWM_MIN))
    if _servo_pwm:  _servo_pwm.duty_u16(_us_to_duty_servo(SERVO_NEUTRAL))