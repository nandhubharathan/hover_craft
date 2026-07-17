#!/usr/bin/env python3
"""
sitl_bridge.py — Software-In-The-Loop Bridge for Hovercraft
=============================================================
ROS 2 node that bridges Gazebo simulation topics to the
hovercraft's MicroPython control logic.

Replaces:
  - MPU-6050 I2C reads     → subscribes /hovercraft/imu
  - ESC PWM (lift/thrust)  → publishes forces/efforts
  - Servo PWM              → publishes joint position commands

Provides keyboard-based joystick emulation:
  W/S = thrust forward/back    A/D = steer left/right
  L   = toggle lift            I   = toggle AI mode
  Q   = quit

Run:  ros2 run <your_pkg> sitl_bridge.py
  or: python3 sitl_bridge.py  (standalone)
"""

import sys
import math
import threading
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64


# ═══════════════════════════════════════════════════════════════════
#  CONSTANTS — mirrored from hover_control.py
# ═══════════════════════════════════════════════════════════════════

PWM_MIN          = 1000
PWM_MAX          = 2000
LIFT_HOVER_PWM   = 1700
LIFT_MAX_PWM     = 1850
THRUST_IDLE_PWM  = 1100
THRUST_MAX_PWM   = 1900
SERVO_MIN        = 500
SERVO_NEUTRAL    = 1500
SERVO_MAX        = 2500
RAMP_STEP        = 50
FAILSAFE_MS      = 500

# Physics mapping: PWM µs → Newtons / rad
# Lift force: LIFT_HOVER_PWM (1700 µs) should produce ~3.92 N (0.4 kg × 9.81)
LIFT_FORCE_SCALE   = 3.92 / (LIFT_HOVER_PWM - PWM_MIN)   # N per µs above idle
# Thrust force: THRUST_MAX_PWM produces roughly 0.8 N
THRUST_FORCE_SCALE = 0.8 / (THRUST_MAX_PWM - PWM_MIN)
# Servo: 500–2500 µs → 0–π rad
SERVO_RAD_SCALE    = math.pi / (SERVO_MAX - SERVO_MIN)


# ═══════════════════════════════════════════════════════════════════
#  KEYBOARD INPUT (non-blocking)
# ═══════════════════════════════════════════════════════════════════

class KeyboardReader:
    """Reads single keystrokes from the terminal without blocking."""

    def __init__(self):
        self._settings = termios.tcgetattr(sys.stdin)

    def read_key(self):
        """Return a single char or '' if nothing pressed."""
        try:
            tty.setraw(sys.stdin.fileno())
            # Non-blocking read
            import select
            if select.select([sys.stdin], [], [], 0.02)[0]:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        return key

    def restore(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)


# ═══════════════════════════════════════════════════════════════════
#  CONTROL STATE — replaces main.py's _shared dict
# ═══════════════════════════════════════════════════════════════════

class ControlState:
    """Thread-safe joystick / mode state."""

    def __init__(self):
        self.lock = threading.Lock()
        self.joy_x       = 0.0
        self.joy_y       = 0.0
        self.lift_on     = False
        self.ai_mode     = False
        self.obstacle    = 0       # 0=clear, 1=center, 2=left, 3=right

        # IMU data from Gazebo
        self.ax = 0.0
        self.ay = 0.0
        self.az = 1.0   # default: gravity pointing down

    def snapshot(self):
        with self.lock:
            return {
                "joy_x":    self.joy_x,
                "joy_y":    self.joy_y,
                "lift_on":  self.lift_on,
                "ai_mode":  self.ai_mode,
                "obstacle": self.obstacle,
                "ax":       self.ax,
                "ay":       self.ay,
                "az":       self.az,
            }

    def set(self, **kwargs):
        with self.lock:
            for k, v in kwargs.items():
                setattr(self, k, v)


# ═══════════════════════════════════════════════════════════════════
#  RAMP HELPER — port of hover_control's rate limiter
# ═══════════════════════════════════════════════════════════════════

def ramp_toward(current, target, step=RAMP_STEP):
    """Move current toward target by at most `step` per call."""
    delta = target - current
    if abs(delta) > step:
        return current + (step if delta > 0 else -step)
    return target


# ═══════════════════════════════════════════════════════════════════
#  COMPUTE TARGETS — port of hover_control.compute_targets()
# ═══════════════════════════════════════════════════════════════════

def compute_targets(joy_x, joy_y, lift_on, mlp_out=None):
    """
    Map joystick + optional MLP trims into PWM targets (µs).
    Returns (lift_us, thrust_us, servo_us).
    """
    if mlp_out is None:
        mlp_out = [0.0, 0.0, 0.0, 0.0]

    lift_trim   = mlp_out[0]
    thrust_trim = mlp_out[1]
    servo_trim  = mlp_out[2]

    # Lift
    lift_us = (LIFT_HOVER_PWM + lift_trim) if lift_on else PWM_MIN

    # Thrust
    if not lift_on:
        thrust_us = PWM_MIN
    elif joy_y > 0.05:
        thrust_range = THRUST_MAX_PWM - THRUST_IDLE_PWM
        thrust_us = THRUST_IDLE_PWM + joy_y * thrust_range + thrust_trim
    elif joy_y < -0.05:
        thrust_us = THRUST_IDLE_PWM + abs(joy_y) * 200 + thrust_trim
    else:
        thrust_us = PWM_MIN

    # Servo
    if joy_y < -0.05:
        if joy_x < -0.1:
            servo_us = SERVO_MIN + servo_trim
        elif joy_x > 0.1:
            servo_us = SERVO_MAX + servo_trim
        else:
            servo_us = SERVO_MIN + servo_trim
    else:
        servo_range_half = 500
        servo_us = SERVO_NEUTRAL + joy_x * servo_range_half + servo_trim

    return (lift_us, thrust_us, servo_us)


def apply_obstacle_override(servo_us, thrust_us, obstacle_code):
    """Force servo/thrust based on obstacle code (0–3)."""
    if obstacle_code == 1:
        return (SERVO_MIN, PWM_MIN)
    elif obstacle_code == 2:
        sv = max(SERVO_NEUTRAL + 300, min(SERVO_MAX, servo_us + 400))
        return (sv, thrust_us)
    elif obstacle_code == 3:
        sv = min(SERVO_NEUTRAL - 300, max(SERVO_MIN, servo_us - 400))
        return (sv, thrust_us)
    return (servo_us, thrust_us)


# ═══════════════════════════════════════════════════════════════════
#  ROS 2 BRIDGE NODE
# ═══════════════════════════════════════════════════════════════════

class SITLBridge(Node):
    """
    ROS 2 node bridging keyboard input + Gazebo sensors → actuator commands.
    """

    def __init__(self, state: ControlState):
        super().__init__('hovercraft_sitl_bridge')
        self.state = state

        # Current PWM values (for ramping)
        self._lift_us   = PWM_MIN
        self._thrust_us = PWM_MIN
        self._servo_us  = SERVO_NEUTRAL

        # QoS for sensor data
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)

        # ── Subscribers ──────────────────────────────────────────
        self.imu_sub = self.create_subscription(
            Imu, '/hovercraft/imu', self._imu_cb, sensor_qos)

        # ── Publishers ───────────────────────────────────────────
        # Lift: vertical force applied to base_link via Wrench
        self.lift_pub = self.create_publisher(
            Wrench, '/hovercraft/lift_force', 10)

        # Thrust: effort on the propeller joint
        self.thrust_pub = self.create_publisher(
            Float64, '/hovercraft/thrust_effort', 10)

        # Servo: position command for servo_joint
        self.servo_pub = self.create_publisher(
            Float64, '/hovercraft/servo_cmd', 10)

        # ── Control loop timer (50 Hz = 20 ms) ──────────────────
        self.timer = self.create_timer(0.02, self._control_tick)

        self.get_logger().info(
            '═' * 50 + '\n'
            '  HOVERCRAFT SITL BRIDGE — ACTIVE\n'
            '  Controls:  W/S=thrust  A/D=steer  L=lift  I=AI  Q=quit\n'
            + '═' * 50)

    # ── IMU Callback ─────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        """Extract linear acceleration from Gazebo IMU → state."""
        acc = msg.linear_acceleration
        # Normalize to ±2 g range (matching MPU-6050 sensitivity)
        self.state.set(
            ax=acc.x / 9.81,
            ay=acc.y / 9.81,
            az=acc.z / 9.81,
        )

    # ── Control Tick (50 Hz) ─────────────────────────────────────

    def _control_tick(self):
        """Run one iteration of the control loop."""
        snap = self.state.snapshot()

        joy_x    = snap["joy_x"]
        joy_y    = snap["joy_y"]
        lift_on  = snap["lift_on"]
        obstacle = snap["obstacle"]

        # ── Compute targets (reuses hover_control logic) ─────────
        lift_tgt, thrust_tgt, servo_tgt = compute_targets(
            joy_x, joy_y, lift_on)

        # Obstacle override
        if obstacle > 0:
            servo_tgt, thrust_tgt = apply_obstacle_override(
                servo_tgt, thrust_tgt, obstacle)

        # ── Rate-limited ramp ────────────────────────────────────
        self._lift_us   = ramp_toward(self._lift_us, lift_tgt)
        self._thrust_us = ramp_toward(self._thrust_us, thrust_tgt)
        self._servo_us  = servo_tgt   # servo: no ramp (low inertia)

        # ── Publish lift force (vertical, on base_link) ──────────
        lift_msg = Wrench()
        if self._lift_us > PWM_MIN:
            lift_msg.force.z = (self._lift_us - PWM_MIN) * LIFT_FORCE_SCALE
        else:
            lift_msg.force.z = 0.0
        self.lift_pub.publish(lift_msg)

        # ── Publish thrust effort ────────────────────────────────
        thrust_msg = Float64()
        if self._thrust_us > PWM_MIN:
            thrust_msg.data = (self._thrust_us - PWM_MIN) * THRUST_FORCE_SCALE
        else:
            thrust_msg.data = 0.0
        self.thrust_pub.publish(thrust_msg)

        # ── Publish servo position (µs → rad) ───────────────────
        servo_msg = Float64()
        servo_msg.data = (self._servo_us - SERVO_MIN) * SERVO_RAD_SCALE
        self.servo_pub.publish(servo_msg)


# ═══════════════════════════════════════════════════════════════════
#  KEYBOARD LOOP (runs in separate thread)
# ═══════════════════════════════════════════════════════════════════

def keyboard_loop(state: ControlState, stop_event: threading.Event):
    """Read keys and update control state."""
    kb = KeyboardReader()
    JOY_STEP = 0.15  # increment per key press
    DECAY    = 0.85   # exponential decay when no key

    print("\n  [KEYBOARD] Ready — W/S/A/D/L/I/Q\n")

    try:
        while not stop_event.is_set():
            key = kb.read_key().lower()
            snap = state.snapshot()
            jx, jy = snap["joy_x"], snap["joy_y"]

            if key == 'q':
                stop_event.set()
                break
            elif key == 'w':
                jy = min(1.0, jy + JOY_STEP)
            elif key == 's':
                jy = max(-1.0, jy - JOY_STEP)
            elif key == 'a':
                jx = max(-1.0, jx - JOY_STEP)
            elif key == 'd':
                jx = min(1.0, jx + JOY_STEP)
            elif key == 'l':
                state.set(lift_on=not snap["lift_on"])
                status = "ON" if not snap["lift_on"] else "OFF"
                print(f"  [LIFT] {status}")
            elif key == 'i':
                state.set(ai_mode=not snap["ai_mode"])
                status = "ON" if not snap["ai_mode"] else "OFF"
                print(f"  [AI MODE] {status}")
            else:
                # Decay toward center when no input
                jx *= DECAY
                jy *= DECAY
                if abs(jx) < 0.02:
                    jx = 0.0
                if abs(jy) < 0.02:
                    jy = 0.0

            state.set(joy_x=jx, joy_y=jy)
    finally:
        kb.restore()


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    state = ControlState()
    node = SITLBridge(state)

    stop_event = threading.Event()

    # Start keyboard input in background thread
    kb_thread = threading.Thread(
        target=keyboard_loop, args=(state, stop_event), daemon=True)
    kb_thread.start()

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("SITL Bridge shutting down …")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
