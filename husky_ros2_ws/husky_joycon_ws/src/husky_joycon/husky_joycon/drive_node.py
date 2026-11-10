"""Shared ROS-side command refresh and shutdown handling."""

import math
import time

import rclpy
from rclpy.node import Node

from husky import HuskyClient


class DriveNode(Node):
    """Refresh a single input source through the existing Husky client."""

    def __init__(self, name):
        super().__init__(name)
        self.command_timeout = self.positive_parameter('command_timeout', 0.5)
        self.accel = self.positive_parameter('accel', 0.5)
        self.husky = HuskyClient()
        self.left_speed = self.right_speed = 0.0
        self.last_command = None
        self.stop_pending = False
        # The A200 server watchdog expires after 0.1 seconds.
        self.create_timer(0.05, self.send_command)

    def positive_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def set_command(self, left, right):
        self.left_speed, self.right_speed = left, right
        self.last_command = time.monotonic()
        self.stop_pending = True

    def write_command(self, left, right):
        try:
            self.husky.set_differential_speed(
                left_speed=left, right_speed=right,
                left_accel=self.accel, right_accel=self.accel,
            )
            return True
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(
                f'A200 command failed: {error}', throttle_duration_sec=5.0)
            return False

    def send_command(self):
        if self.last_command is None:
            return
        if time.monotonic() - self.last_command <= self.command_timeout:
            self.write_command(self.left_speed, self.right_speed)
        elif self.stop_pending and self.write_command(0.0, 0.0):
            # Stop once, then leave other independently running inputs alone.
            self.stop_pending = False

    def stop(self):
        if self.stop_pending:
            self.write_command(0.0, 0.0)


def run(node_type, args=None):
    """Run either controller and stop before releasing its node."""
    rclpy.init(args=args)
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.stop()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
