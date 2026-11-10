#!/usr/bin/env python3
"""Drive the Husky directly from ROS Joy messages."""

import math

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

from .drive_node import DriveNode, run


class HuskyJoyController(DriveNode):
    """Map the left stick and held shoulder buttons to wheel speeds."""

    def __init__(self):
        super().__init__('husky_joy_controller')
        self.axis_turn = self.index_parameter('axis_turn', 0)
        self.axis_forward = self.index_parameter('axis_forward', 1)
        self.button_slow = self.index_parameter('button_slow', 4)
        self.button_fast = self.index_parameter('button_fast', 5)
        self.max_speed_slow = self.positive_parameter('max_speed_slow', 0.5)
        self.max_speed_fast = self.positive_parameter('max_speed_fast', 1.0)
        self.deadzone = float(self.declare_parameter('deadzone', 0.08).value)
        if not math.isfinite(self.deadzone) or not 0 <= self.deadzone < 1:
            raise ValueError('deadzone must be finite and in [0, 1)')
        self.create_subscription(
            Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Listening on /joy; hold button {self.button_slow} (slow) or '
            f'{self.button_fast} (fast); forward axis {self.axis_forward}, '
            f'turn axis {self.axis_turn}')

    def index_parameter(self, name, default):
        value = self.declare_parameter(name, default).value
        if not isinstance(value, int) or value < 0:
            raise ValueError(f'{name} must be a nonnegative integer')
        return value

    def joy_callback(self, msg):
        if (len(msg.axes) <= max(self.axis_forward, self.axis_turn) or
                len(msg.buttons) <= max(self.button_slow, self.button_fast)):
            self.set_command(0.0, 0.0)
            self.get_logger().warning(
                'Joy message is too short for configured axes/buttons; '
                'check axis_forward, axis_turn, button_slow and button_fast',
                throttle_duration_sec=5.0)
            return
        forward, turn = msg.axes[self.axis_forward], msg.axes[self.axis_turn]
        if not all(math.isfinite(v) for v in (forward, turn)):
            self.set_command(0.0, 0.0)
            return
        forward = self.filter_axis(forward)
        turn = self.filter_axis(turn)
        if msg.buttons[self.button_fast]:
            speed = self.max_speed_fast
        elif msg.buttons[self.button_slow]:
            speed = self.max_speed_slow
        else:
            speed = 0.0
        # ROS Joy positive horizontal axis means left.
        # Cap forward wheel speeds even when speed parameters exceed 1.0.
        self.set_command(
            min(1.0, max(-1.0, min(1.0, forward - turn)) * speed),
            min(1.0, max(-1.0, min(1.0, forward + turn)) * speed),
        )

    def filter_axis(self, value):
        return 0.0 if abs(value) <= self.deadzone else max(-1.0, min(1.0, value))


def main(args=None):
    run(HuskyJoyController, args)


if __name__ == '__main__':
    main()
