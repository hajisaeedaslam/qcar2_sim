#!/usr/bin/env python3
"""
Minimal line controller node.

States:
  TRACK  — valid error received recently; P-control on angular velocity.
  SEARCH — no valid error for error_timeout seconds; drive straight slowly.

Subscribes:  error_topic  (std_msgs/Float32)  999.0 = no line detected
Publishes:   cmd_topic    (geometry_msgs/Twist)  at control_rate Hz
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class LineController(Node):

    def __init__(self):
        super().__init__('line_controller')

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter('error_topic',  '/line_error')
        self.declare_parameter('cmd_topic',    '/model/qcar2/cmd_vel')
        self.declare_parameter('kp',           0.8)
        self.declare_parameter('base_speed',   0.35)
        self.declare_parameter('search_speed', 0.10)
        self.declare_parameter('search_angular', 0.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('error_timeout', 1.0)
        self.declare_parameter('log_period',   0.5)

        p = self.get_parameter
        error_topic   = p('error_topic').value
        cmd_topic     = p('cmd_topic').value
        self._kp           = float(p('kp').value)
        self._base_speed   = float(p('base_speed').value)
        self._search_speed = float(p('search_speed').value)
        self._search_ang   = float(p('search_angular').value)
        self._timeout      = float(p('error_timeout').value)
        rate               = float(p('control_rate').value)
        log_period         = float(p('log_period').value)

        self._last_error      = math.nan
        self._last_valid_stamp = None          # None = never received

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Float32, error_topic, self._error_cb, 10)
        self.create_timer(1.0 / rate, self._tick)
        self._log_timer = self.create_timer(log_period, self._log)

        self.get_logger().info(
            f'Controller ready  [{error_topic}] -> [{cmd_topic}]  '
            f'Kp={self._kp}  speed={self._base_speed} m/s')

    # ── subscriber ────────────────────────────────────────────────────
    def _error_cb(self, msg: Float32):
        # perception publishes 999.0 when the line is not detected
        if abs(msg.data) <= 1.0:
            self._last_error = msg.data
            self._last_valid_stamp = self.get_clock().now()

    # ── control loop ──────────────────────────────────────────────────
    def _tick(self):
        cmd = Twist()
        now = self.get_clock().now()

        if self._last_valid_stamp is not None:
            age = (now - self._last_valid_stamp).nanoseconds / 1e9
        else:
            age = float('inf')

        if age <= self._timeout and not math.isnan(self._last_error):
            # TRACK: proportional steering
            err = self._last_error
            cmd.linear.x  = self._base_speed * (1.0 - 0.4 * abs(err))
            cmd.angular.z = -self._kp * err
            self._mode = 'TRACK'
        else:
            # SEARCH: drive straight until the line reappears
            cmd.linear.x  = self._search_speed
            cmd.angular.z = self._search_ang
            self._mode = 'SEARCH'

        self._cmd_pub.publish(cmd)
        self._last_cmd = cmd

    # ── periodic log ──────────────────────────────────────────────────
    def _log(self):
        mode = getattr(self, '_mode', 'INIT')
        cmd  = getattr(self, '_last_cmd', Twist())
        err  = self._last_error if not math.isnan(self._last_error) else float('nan')
        self.get_logger().info(
            f'[{mode}] err={err:+.3f}  '
            f'v={cmd.linear.x:.2f}  w={cmd.angular.z:+.2f}')


def main():
    rclpy.init()
    node = LineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()