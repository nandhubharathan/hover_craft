"""
mlp_logic.py — TinyML Inference Engine (RP2040 / MicroPython)
==============================================================
TinyML design principles applied to the RP2040 Cortex-M0+ @ 125 MHz:

  1. QUANTIZATION-AWARE INFERENCE
     Weights stored as int8 in weights.py (range -127..127).
     Dequantized inline: float_w = int8_w * WEIGHT_SCALE.
     Halves flash footprint vs float32; fits in RP2040's 2 MB.

  2. FIXED ALLOCATION / NO GC PRESSURE
     All buffers allocated once at import time.
     No list/dict creation inside forward().
     Critical for deterministic 100 Hz timing on bare metal.

  3. LOOKUP-TABLE ACTIVATIONS  (TinyML standard)
     tanh and sigmoid replaced with 256-entry LUT (float, [-4,+4]).
     ~8x faster than Pade approximant on M0+ (no float division).
     Linear interpolation between entries: max error < 0.004.
     sigmoid reuses same LUT via identity: sig(x)=0.5+0.5*tanh(x/2).

  4. INPUT NORMALIZATION BUILT-IN
     normalize() maps raw sensor values to [-1, 1]:
       IMU  ±2g  → divide by 2.0
       az         → (az - 1.0) / 2.0  (centres gravity at 0)
       joystick   → already ±1, clamped
       obstacle   → 0.0 / 1.0
       yaw_rate   → divide by 250.0  (±250°/s gyro range)

  5. ZERO-COPY OUTPUT
     _output_buf written in-place and returned by reference.
     No allocation per inference call.

  6. CONFIDENCE GATE
     alert output [3] in [0,1] acts as model confidence.
     If alert < CONFIDENCE_THRESHOLD → trims zeroed.
     Prevents acting on low-confidence MLP outputs mid-flight.

  7. HARD OUTPUT CLAMP
     All outputs clamped to physical limits after dequantization.

Architecture : 7 → 8 (tanh) → 4 (sigmoid)
Inputs       : [ax, ay, az, joy_x, joy_y, obstacle_flag, yaw_rate]  (normalized)
Outputs      : [lift_trim_us, thrust_trim_us, servo_trim_us, alert]
"""

import math

# ── Quantization ──────────────────────────────────────────────────────
WEIGHT_SCALE         = 1.0 / 127.0   # int8 → float  (-127..127 → ±1)
CONFIDENCE_THRESHOLD = 0.3           # alert gate

# ── Input normalization factors ───────────────────────────────────────
_IMU_SCALE  = 0.5           # 1 / 2g  →  ±2g IMU maps to ±1
_GYRO_SCALE = 1.0 / 250.0  # ±250°/s gyro range → ±1

# ── Output physical clamp limits ──────────────────────────────────────
_OUT_LO = [-50.0,  -50.0,  -312.0, 0.0]
_OUT_HI = [ 50.0,   50.0,   312.0, 1.0]

# ── Tanh LUT — built once at import, stored as tuple ─────────────────
_LUT_N    = 256
_LUT_MIN  = -4.0
_LUT_MAX  =  4.0
_LUT_SPAN = _LUT_MAX - _LUT_MIN                    # 8.0
_LUT_STEP = _LUT_SPAN / (_LUT_N - 1)               # ~0.03137
_LUT_STEP_INV = (_LUT_N - 1) / _LUT_SPAN           # ~31.875

_TANH_LUT = tuple(
    math.tanh(_LUT_MIN + i * _LUT_STEP) for i in range(_LUT_N)
)


def _lut_tanh(x):
    """LUT tanh with linear interpolation. No division in hot path."""
    if x <= _LUT_MIN: return -1.0
    if x >= _LUT_MAX:  return  1.0
    fi  = (x - _LUT_MIN) * _LUT_STEP_INV
    idx = int(fi)
    return _TANH_LUT[idx] + (fi - idx) * (_TANH_LUT[idx + 1] - _TANH_LUT[idx])


def _lut_sigmoid(x):
    """Sigmoid via tanh identity — reuses _TANH_LUT, no extra table."""
    return 0.5 + 0.5 * _lut_tanh(x * 0.5)


# ── Pre-allocated inference buffers ──────────────────────────────────
_hidden_buf  = [0.0] * 8
_output_buf  = [0.0] * 4
_norm_buf    = [0.0] * 7


# ── Input normalization ───────────────────────────────────────────────

def normalize(ax, ay, az, joy_x, joy_y, obstacle, yaw_rate=0.0):
    """
    Normalize raw inputs into [-1, 1] and write into _norm_buf.
    az is centred: (az - 1.0) * 0.5  so level craft reads 0.0, not 0.5.
    yaw_rate (°/s) scaled by ±250 range to [-1, 1].
    Returns reference to _norm_buf (no allocation).
    """
    _norm_buf[0] = max(-1.0, min(1.0,  ax          * _IMU_SCALE))
    _norm_buf[1] = max(-1.0, min(1.0,  ay          * _IMU_SCALE))
    _norm_buf[2] = max(-1.0, min(1.0, (az - 1.0)   * _IMU_SCALE))
    _norm_buf[3] = max(-1.0, min(1.0,  joy_x))
    _norm_buf[4] = max(-1.0, min(1.0,  joy_y))
    _norm_buf[5] = 1.0 if obstacle else 0.0
    _norm_buf[6] = max(-1.0, min(1.0,  yaw_rate    * _GYRO_SCALE))
    return _norm_buf


# ── TinyML forward pass ───────────────────────────────────────────────

def forward(inputs, w_hidden, b_hidden, w_output, b_output,
            output_scale, output_offset):
    """
    TinyML forward pass.

    int8 weights dequantized inline:
        accum += (w[i] * WEIGHT_SCALE) * input[i]

    Activations via LUT (no math.exp, no float division).
    Confidence gate zeros trims if alert < CONFIDENCE_THRESHOLD.
    Hard clamp applied to all outputs.
    """
    n_in  = len(inputs)
    n_hid = len(b_hidden)
    n_out = len(b_output)

    # Hidden layer — tanh
    for j in range(n_hid):
        accum = b_hidden[j]
        wj = w_hidden[j]
        for i in range(n_in):
            accum += (wj[i] * WEIGHT_SCALE) * inputs[i]
        _hidden_buf[j] = _lut_tanh(accum)

    # Output layer — sigmoid → scale → clamp
    for k in range(n_out):
        accum = b_output[k]
        wk = w_output[k]
        for j in range(n_hid):
            accum += (wk[j] * WEIGHT_SCALE) * _hidden_buf[j]
        val = _lut_sigmoid(accum) * output_scale[k] + output_offset[k]
        _output_buf[k] = max(_OUT_LO[k], min(_OUT_HI[k], val))

    # Confidence gate — alert is output[3]
    if _output_buf[3] < CONFIDENCE_THRESHOLD:
        _output_buf[0] = 0.0
        _output_buf[1] = 0.0
        _output_buf[2] = 0.0

    return _output_buf


# ── High-level API ────────────────────────────────────────────────────

def infer(inputs, weights_module):
    """
    Run one TinyML inference cycle.

    `inputs` should be pre-normalized via normalize() or already in [-1,1].
    Returns reference to _output_buf — read immediately, do not cache.

    Called from main.py control loop (Core 1) at 100 Hz.
    """
    return forward(
        inputs,
        weights_module.W_HIDDEN,
        weights_module.B_HIDDEN,
        weights_module.W_OUTPUT,
        weights_module.B_OUTPUT,
        weights_module.OUTPUT_SCALE,
        weights_module.OUTPUT_OFFSET,
    )


# ── Benchmark — run from Thonny Shell to verify 100 Hz budget ─────────

def benchmark(weights_module, n=200):
    """
    Measure inference throughput on the RP2040.
    Target: < 10 000 µs per pass (= 100 Hz headroom).

    Usage:
        import mlp_logic, weights
        mlp_logic.benchmark(weights)
    """
    import time
    dummy = normalize(0.05, -0.02, 1.01, 0.1, 0.2, 0, 3.0)
    t0 = time.ticks_us()
    for _ in range(n):
        infer(dummy, weights_module)
    us_total = time.ticks_diff(time.ticks_us(), t0)
    us_each  = us_total // n
    hz       = 1_000_000 // max(1, us_each)
    print("[TINYML BENCH] {} passes | {} µs/pass | {} Hz".format(n, us_each, hz))
    print("[TINYML BENCH] 100 Hz budget (10000 µs) — {}".format(
        "PASS ✓" if us_each < 10_000 else "FAIL ✗ reduce n_hidden or n_inputs"))
