#!/usr/bin/env python3
"""
odrive_node.py — ROS2 Humble node for ODrive brushless motor control
Target: NVIDIA Jetson (Orin / Xavier / Nano) + ODrive v3.x or Pro via USB

Subscribed topics:
  /odrive/velocity_cmd  [std_msgs/Float64]  — velocity setpoint (turns/s)
  /odrive/position_cmd  [std_msgs/Float64]  — position setpoint (turns)
  /odrive/torque_cmd    [std_msgs/Float64]  — torque setpoint (Nm)
  /odrive/control_mode  [std_msgs/String]   — "velocity" | "position" | "torque"
  /odrive/estop         [std_msgs/Bool]     — True = enter idle / cut power

Published topics:
  /odrive/velocity      [std_msgs/Float64]  — measured velocity (turns/s)
  /odrive/position      [std_msgs/Float64]  — measured position (turns)
  /odrive/current       [std_msgs/Float64]  — measured Iq current (A)
  /odrive/bus_voltage   [std_msgs/Float64]  — DC bus voltage (V)
  /odrive/status        [std_msgs/String]   — human-readable status string
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64, String, Bool

import odrive
from odrive.enums import (
    AxisState,
    ControlMode,
    InputMode,
)
import time


# ──────────────────────────────────────────────────────────────────────────────
# Helper
# ──────────────────────────────────────────────────────────────────────────────

CONTROL_MODE_MAP = {
    "velocity": ControlMode.VELOCITY_CONTROL,
    "position": ControlMode.POSITION_CONTROL,
    "torque":   ControlMode.TORQUE_CONTROL,
}

INPUT_MODE_MAP = {
    "velocity": InputMode.VEL_RAMP,
    "position": InputMode.TRAP_TRAJ,
    "torque":   InputMode.PASSTHROUGH,
}


class ODriveNode(Node):
    """ROS2 node that owns one ODrive axis."""

    def __init__(self):
        super().__init__("odrive_node")

        # ── Parameters (override in config/odrive_params.yaml) ──────────────
        self.declare_parameter("axis_index",           0)
        self.declare_parameter("control_mode",         "velocity")
        self.declare_parameter("vel_limit",            10.0)     # turns/s
        self.declare_parameter("current_limit",        10.0)     # A
        self.declare_parameter("vel_ramp_rate",        2.0)      # turns/s²
        self.declare_parameter("publish_rate_hz",      50.0)
        self.declare_parameter("reconnect_on_error",   True)
        self.declare_parameter("odrive_serial_number", "")       # blank = first found

        # ── Internal state ───────────────────────────────────────────────────
        self.od      = None   # ODrive device handle
        self.axis    = None   # shortcut to the active axis
        self._mode   = self.get_parameter("control_mode").value
        self._estop  = False

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_vel     = self.create_publisher(Float64, "odrive/velocity",    10)
        self.pub_pos     = self.create_publisher(Float64, "odrive/position",    10)
        self.pub_cur     = self.create_publisher(Float64, "odrive/current",     10)
        self.pub_vbus    = self.create_publisher(Float64, "odrive/bus_voltage", 10)
        self.pub_status  = self.create_publisher(String,  "odrive/status",      10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(Float64, "odrive/velocity_cmd",  self._cb_vel,  10)
        self.create_subscription(Float64, "odrive/position_cmd",  self._cb_pos,  10)
        self.create_subscription(Float64, "odrive/torque_cmd",    self._cb_torq, 10)
        self.create_subscription(String,  "odrive/control_mode",  self._cb_mode, 10)
        self.create_subscription(Bool,    "odrive/estop",         self._cb_estop, 10)

        # ── Connect & configure ──────────────────────────────────────────────
        self._connect_and_configure()

        # ── Publish timer ────────────────────────────────────────────────────
        rate = self.get_parameter("publish_rate_hz").value
        self.create_timer(1.0 / rate, self._publish_feedback)

        self.get_logger().info("ODrive node ready.")

    # ──────────────────────────────────────────────────────────────────────────
    # Connection & configuration
    # ──────────────────────────────────────────────────────────────────────────

    def _connect_and_configure(self):
        serial = self.get_parameter("odrive_serial_number").value or None
        self.get_logger().info("Searching for ODrive (serial=%s) …" % (serial or "any"))

        try:
            self.od = odrive.find_any(serial_number=serial, timeout=10)
        except Exception as e:
            self.get_logger().error("Could not find ODrive: %s" % str(e))
            raise RuntimeError("ODrive not found") from e

        axis_idx = self.get_parameter("axis_index").value
        self.axis = getattr(self.od, "axis%d" % axis_idx)

        self.get_logger().info(
            "Connected to ODrive (fw %s, hw %s)" % (
                self.od.fw_version_major,
                self.od.hw_version_major,
            )
        )

        self._apply_limits()
        self._set_control_mode(self._mode)
        self._arm()

    def _apply_limits(self):
        vel_limit  = self.get_parameter("vel_limit").value
        cur_limit  = self.get_parameter("current_limit").value
        ramp_rate  = self.get_parameter("vel_ramp_rate").value

        self.axis.controller.config.vel_limit              = vel_limit
        self.axis.motor.config.current_lim                 = cur_limit
        self.axis.controller.config.vel_ramp_rate          = ramp_rate
        self.axis.controller.config.inertia                = 0.0  # set if known

        self.get_logger().info(
            "Limits applied — vel_limit=%.1f t/s, current_lim=%.1f A, ramp=%.1f t/s²"
            % (vel_limit, cur_limit, ramp_rate)
        )

    def _set_control_mode(self, mode_str: str):
        if mode_str not in CONTROL_MODE_MAP:
            self.get_logger().warn("Unknown mode '%s', defaulting to velocity." % mode_str)
            mode_str = "velocity"

        self.axis.controller.config.control_mode = CONTROL_MODE_MAP[mode_str]
        self.axis.controller.config.input_mode   = INPUT_MODE_MAP[mode_str]
        self._mode = mode_str
        self.get_logger().info("Control mode → %s" % mode_str)

    def _arm(self):
        """Transition axis to CLOSED_LOOP_CONTROL."""
        self.get_logger().info("Requesting CLOSED_LOOP_CONTROL …")
        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL

        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self.axis.current_state == AxisState.CLOSED_LOOP_CONTROL:
                self.get_logger().info("Axis armed — CLOSED_LOOP_CONTROL active.")
                return
            time.sleep(0.05)

        self.get_logger().error(
            "Axis did not enter CLOSED_LOOP_CONTROL (state=%d, error=0x%X)"
            % (self.axis.current_state, self.axis.error)
        )
        raise RuntimeError("ODrive arm failed")

    def _idle(self):
        try:
            self.axis.requested_state = AxisState.IDLE
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    # Subscribers
    # ──────────────────────────────────────────────────────────────────────────

    def _cb_vel(self, msg: Float64):
        if self._estop or self.axis is None:
            return
        if self._mode != "velocity":
            self._set_control_mode("velocity")
        try:
            self.axis.controller.input_vel = msg.data
        except Exception as e:
            self.get_logger().error("vel cmd error: %s" % str(e))

    def _cb_pos(self, msg: Float64):
        if self._estop or self.axis is None:
            return
        if self._mode != "position":
            self._set_control_mode("position")
        try:
            self.axis.controller.input_pos = msg.data
        except Exception as e:
            self.get_logger().error("pos cmd error: %s" % str(e))

    def _cb_torq(self, msg: Float64):
        if self._estop or self.axis is None:
            return
        if self._mode != "torque":
            self._set_control_mode("torque")
        try:
            self.axis.controller.input_torque = msg.data
        except Exception as e:
            self.get_logger().error("torque cmd error: %s" % str(e))

    def _cb_mode(self, msg: String):
        self._set_control_mode(msg.data)

    def _cb_estop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("E-STOP received — going IDLE.")
            self._estop = True
            self._idle()
        else:
            self.get_logger().info("E-STOP cleared — re-arming …")
            self._estop = False
            self._arm()

    # ──────────────────────────────────────────────────────────────────────────
    # Feedback publisher
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_feedback(self):
        if self.axis is None:
            return
        try:
            vel = self.axis.encoder.vel_estimate
            pos = self.axis.encoder.pos_estimate
            cur = self.axis.motor.current_control.Iq_measured
            vbus = self.od.vbus_voltage

            self.pub_vel.publish(Float64(data=float(vel)))
            self.pub_pos.publish(Float64(data=float(pos)))
            self.pub_cur.publish(Float64(data=float(cur)))
            self.pub_vbus.publish(Float64(data=float(vbus)))

            status = (
                "state=%d err=0x%X vel=%.2f pos=%.2f Iq=%.2f Vbus=%.1f"
                % (self.axis.current_state, self.axis.error, vel, pos, cur, vbus)
            )
            self.pub_status.publish(String(data=status))

            # Auto-clear non-fatal errors if reconnect enabled
            if (
                self.axis.error != 0
                and self.get_parameter("reconnect_on_error").value
            ):
                self.get_logger().warn("Axis error 0x%X — attempting clear." % self.axis.error)
                self.od.clear_errors()

        except Exception as e:
            self.get_logger().warn("Feedback read failed: %s" % str(e))

    # ──────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("Shutting down — setting axis IDLE.")
        self._idle()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ODriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
