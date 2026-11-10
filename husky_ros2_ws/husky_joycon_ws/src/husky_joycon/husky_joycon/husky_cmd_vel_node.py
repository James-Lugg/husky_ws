#!/usr/bin/env python3
"""Drive the Husky directly from ROS Twist messages."""

import math

from geometry_msgs.msg import Twist

from .drive_node import DriveNode, run


class HuskyCmdVelController(DriveNode):
    """Convert linear and angular velocity into differential wheel speeds."""

    def __init__(self):
        super().__init__('husky_cmd_vel_controller')
        self.track_width = self.positive_parameter('track_width', 0.555)
        self.max_cmd_speed = self.positive_parameter('max_cmd_speed', 0.5)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.get_logger().info('Listening on /cmd_vel (geometry_msgs/msg/Twist)')

    def cmd_vel_callback(self, msg):
        linear, angular = msg.linear.x, msg.angular.z
        if not (math.isfinite(linear) and math.isfinite(angular)):
            self.set_command(0.0, 0.0)
            return
        left = linear - angular * self.track_width / 2.0
        right = linear + angular * self.track_width / 2.0
        if not (math.isfinite(left) and math.isfinite(right)):
            self.set_command(0.0, 0.0)
            return
        scale = max(1.0, abs(left) / self.max_cmd_speed,
                    abs(right) / self.max_cmd_speed)
        self.set_command(left / scale, right / scale)


def main(args=None):
    run(HuskyCmdVelController, args)


if __name__ == '__main__':
    main()
