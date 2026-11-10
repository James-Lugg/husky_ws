#!/usr/bin/env python3

import json
import math
from typing import Any

from husky import HuskyClient
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class HuskyTelemetryNode(Node):

    def __init__(self) -> None:
        super().__init__('husky_telemetry_node')

        # Parameters can be changed from the command line or a launch file.
        self.declare_parameter('host', '192.168.131.1')
        self.declare_parameter('port', 9000)
        self.declare_parameter('timeout', 2.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('telemetry_rate', 1.0)
        self.declare_parameter('wheel_separation', 0.555)
        self.declare_parameter('wheel_radius', 0.1651)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        publish_rate = float(
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )

        self.wheel_separation = float(
            self.get_parameter('wheel_separation')
            .get_parameter_value()
            .double_value
        )

        self.odom_frame = (
            self.get_parameter('odom_frame').get_parameter_value().string_value
        )

        self.base_frame = (
            self.get_parameter('base_frame').get_parameter_value().string_value
        )

        telemetry_rate = float(self.get_parameter('telemetry_rate').value)

        if not math.isfinite(telemetry_rate) or telemetry_rate <= 0.0:
            raise ValueError('telemetry_rate must be finite and greater than zero')

        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and greater than zero')

        if (not math.isfinite(self.wheel_separation)
                or self.wheel_separation <= 0.0):
            raise ValueError('wheel_separation must be finite and positive')

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        if not math.isfinite(self.wheel_radius) or self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be finite and positive')

        timeout = float(self.get_parameter('timeout').value)
        port = int(self.get_parameter('port').value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('timeout must be finite and greater than zero')
        if not 1 <= port <= 65535:
            raise ValueError('port must be between 1 and 65535')

        self.publisher = self.create_publisher(
            Odometry,
            '/husky/odom',
            10,
        )

        self.telemetry_publishers = {
            name: self.create_publisher(String, f'/husky/{name}', 10)
            for name in (
                'system_status', 'safety_status',
                'differential_speed',
            )
        }

        self.telemetry_publishers['encoders'] = self.create_publisher(
            JointState, '/husky/encoders', 10,
        )

        self.husky = HuskyClient(
            host=self.get_parameter('host').value,
            port=port,
            timeout=timeout,
        )

        # Estimated robot pose.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous absolute encoder travel values.
        self.previous_left_travel: float | None = None
        self.previous_right_travel: float | None = None

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.update_odometry,
        )

        self.telemetry_timer = self.create_timer(
            1.0 / telemetry_rate, self.update_telemetry,
        )

        self.get_logger().info(
            f'Publishing Husky odometry on /husky/odom at '
            f'{publish_rate:.1f} Hz'
        )

    def publish_telemetry(self, name: str, response: dict[str, Any]) -> None:
        """Publish encoder joint states or the complete JSON status response."""
        if not isinstance(response, dict) or not response.get('ok', False):
            raise ValueError(f'Unsuccessful {name} response: {response}')
        if name == 'encoders':
            encoders = response.get('encoders')
            if not isinstance(encoders, list) or len(encoders) != 2:
                raise ValueError('Expected exactly two encoders: left, right')
            positions = [float(encoder['travel']) for encoder in encoders]
            velocities = [float(encoder['speed']) for encoder in encoders]
            if not all(math.isfinite(value) for value in positions + velocities):
                raise ValueError('Encoder values must be finite')
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = ['left_wheel_joint', 'right_wheel_joint']
            message.position = [value / self.wheel_radius for value in positions]
            message.velocity = [value / self.wheel_radius for value in velocities]
        else:
            message = String()
            message.data = json.dumps(response, allow_nan=False)
        self.telemetry_publishers[name].publish(message)

    def update_telemetry(self) -> None:
        """Read each status independently so a failure cannot skip the rest."""
        for name in ('system_status', 'safety_status', 'differential_speed'):
            try:
                response = getattr(self.husky, f'get_{name}')()
                self.publish_telemetry(name, response)
            except Exception as error:
                self.get_logger().warning(
                    f'Failed to publish Husky {name}: {error}',
                    throttle_duration_sec=5.0,
                )

    def update_odometry(self) -> None:
        try:
            encoder_data: dict[str, Any] = self.husky.get_encoders()
        except Exception as error:
            self.get_logger().error(
                f'Failed to read Husky encoders: {error}'
            )
            return

        if not isinstance(encoder_data, dict) or not encoder_data.get('ok', False):
            self.get_logger().warning(
                f'Husky encoder request was unsuccessful: {encoder_data}'
            )
            return

        try:
            self.publish_telemetry('encoders', encoder_data)
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Failed to publish encoder telemetry: {error}',
                throttle_duration_sec=5.0,
            )

        encoders = encoder_data.get('encoders', [])

        if not isinstance(encoders, list) or len(encoders) < 2:
            self.get_logger().warning(
                f'Expected two encoder values, received: {encoder_data}'
            )
            return

        try:
            left_speed = float(encoders[0]['speed'])
            right_speed = float(encoders[1]['speed'])

            left_travel = float(encoders[0]['travel'])
            right_travel = float(encoders[1]['travel'])
            if not all(math.isfinite(value) for value in (
                left_speed, right_speed, left_travel, right_travel,
            )):
                raise ValueError('Encoder values must be finite')
        except (KeyError, TypeError, ValueError) as error:
            self.get_logger().error(
                f'Invalid encoder response: {error}; data={encoder_data}'
            )
            return

        # The first reading establishes the starting encoder positions.
        if (
            self.previous_left_travel is None
            or self.previous_right_travel is None
        ):
            self.previous_left_travel = left_travel
            self.previous_right_travel = right_travel
            self.publish_odometry(left_speed, right_speed)
            return

        delta_left = left_travel - self.previous_left_travel
        delta_right = right_travel - self.previous_right_travel

        self.previous_left_travel = left_travel
        self.previous_right_travel = right_travel

        # Differential-drive odometry.
        delta_distance = (delta_left + delta_right) / 2.0
        delta_theta = (
            delta_right - delta_left
        ) / self.wheel_separation

        # Midpoint integration gives a better estimate while turning.
        heading_midpoint = self.theta + delta_theta / 2.0

        self.x += delta_distance * math.cos(heading_midpoint)
        self.y += delta_distance * math.sin(heading_midpoint)
        self.theta += delta_theta

        # Keep heading within -pi to pi.
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta),
        )

        self.publish_odometry(left_speed, right_speed)

    def publish_odometry(
        self,
        left_speed: float,
        right_speed: float,
    ) -> None:
        linear_velocity = (left_speed + right_speed) / 2.0

        angular_velocity = (
            right_speed - left_speed
        ) / self.wheel_separation

        message = Odometry()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame

        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = 0.0

        # Quaternion representing rotation around the Z axis.
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        message.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        message.twist.twist.linear.x = linear_velocity
        message.twist.twist.linear.y = 0.0
        message.twist.twist.linear.z = 0.0

        message.twist.twist.angular.x = 0.0
        message.twist.twist.angular.y = 0.0
        message.twist.twist.angular.z = angular_velocity

        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = HuskyTelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
