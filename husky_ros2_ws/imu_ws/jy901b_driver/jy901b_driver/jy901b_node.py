#!/usr/bin/env python3

import math
import struct
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import serial
from serial import SerialException

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature


GRAVITY = 9.80665

# sensor_msgs/Imu convention: if covariance[0] == -1.0, that field of the
# message (orientation / angular_velocity / linear_acceleration) has no
# estimate and downstream consumers should ignore it.
UNKNOWN_COVARIANCE = [
    -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
]

ZERO_COVARIANCE = [
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
]


def euler_to_quaternion(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """
    Convert roll, pitch and yaw in radians to a quaternion.

    The rotations are interpreted as intrinsic XYZ / roll-pitch-yaw.
    Returns x, y, z, w.
    """
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return qx, qy, qz, qw


def decode_frame(frame: bytes) -> Optional[dict]:
    """
    Decode one 11-byte WitMotion/JY901 packet.

    Packet layout:
        byte 0:    0x55
        byte 1:    packet type
        bytes 2-9: four little-endian signed 16-bit values
        byte 10:   checksum
    """
    if len(frame) != 11:
        return None

    if frame[0] != 0x55:
        return None

    expected_checksum = sum(frame[:10]) & 0xFF

    if frame[10] != expected_checksum:
        return None

    packet_type = frame[1]
    value1, value2, value3, value4 = struct.unpack(
        "<hhhh",
        frame[2:10],
    )

    if packet_type == 0x51:
        return {
            "type": "acceleration",
            "x": value1 / 32768.0 * 16.0,
            "y": value2 / 32768.0 * 16.0,
            "z": value3 / 32768.0 * 16.0,
            "temperature": value4 / 100.0,
        }

    if packet_type == 0x52:
        return {
            "type": "gyroscope",
            "x": value1 / 32768.0 * 2000.0,
            "y": value2 / 32768.0 * 2000.0,
            "z": value3 / 32768.0 * 2000.0,
        }

    if packet_type == 0x53:
        return {
            "type": "angle",
            "roll": value1 / 32768.0 * 180.0,
            "pitch": value2 / 32768.0 * 180.0,
            "yaw": value3 / 32768.0 * 180.0,
        }

    return None


class JY901BNode(Node):
    def __init__(self) -> None:
        super().__init__("jy901b_driver")

        self.declare_parameter("port", "/dev/husky-imu")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter(
            "temperature_topic",
            "/imu/temperature",
        )
        self.declare_parameter("publish_temperature", True)

        self.port = str(
            self.get_parameter("port").value
        )
        self.baud_rate = int(
            self.get_parameter("baud_rate").value
        )
        self.frame_id = str(
            self.get_parameter("frame_id").value
        )
        self.publish_temperature = bool(
            self.get_parameter("publish_temperature").value
        )

        imu_topic = str(
            self.get_parameter("imu_topic").value
        )
        temperature_topic = str(
            self.get_parameter("temperature_topic").value
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            imu_topic,
            qos_profile_sensor_data,
        )

        self.temperature_publisher = self.create_publisher(
            Temperature,
            temperature_topic,
            qos_profile_sensor_data,
        )

        self.serial_port: Optional[serial.Serial] = None
        self.buffer = bytearray()

        self.latest_acceleration: Optional[dict] = None
        self.latest_gyroscope: Optional[dict] = None
        self.latest_angle: Optional[dict] = None

        self.invalid_checksum_count = 0
        self.published_message_count = 0

        # Track whether we've warned once about a packet type never
        # having been seen, so logs don't spam every angle packet.
        self._warned_missing_acceleration = False
        self._warned_missing_gyroscope = False

        self.open_serial_port()

        # Poll the serial receive buffer at 500 Hz without blocking ROS.
        self.serial_timer = self.create_timer(
            0.002,
            self.read_serial_data,
        )

        self.status_timer = self.create_timer(
            5.0,
            self.print_status,
        )

    def open_serial_port(self) -> None:
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,
            )

            self.get_logger().info(
                f"Opened JY901B on {self.port} "
                f"at {self.baud_rate} baud"
            )

        except SerialException as error:
            self.get_logger().fatal(
                f"Could not open {self.port}: {error}"
            )
            raise

    def read_serial_data(self) -> None:
        if self.serial_port is None:
            return

        try:
            bytes_waiting = self.serial_port.in_waiting

            if bytes_waiting <= 0:
                return

            received = self.serial_port.read(bytes_waiting)

        except SerialException as error:
            self.get_logger().error(
                f"Serial read failed: {error}"
            )
            raise

        if not received:
            return

        self.buffer.extend(received)
        self.process_buffer()

    def process_buffer(self) -> None:
        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                continue

            frame = bytes(self.buffer[:11])

            if (sum(frame[:10]) & 0xFF) != frame[10]:
                self.invalid_checksum_count += 1
                del self.buffer[0]
                continue

            del self.buffer[:11]

            data = decode_frame(frame)

            if data is None:
                continue

            packet_type = data["type"]

            if packet_type == "acceleration":
                self.latest_acceleration = data

                if self.publish_temperature:
                    self.publish_temperature_message(data)

            elif packet_type == "gyroscope":
                self.latest_gyroscope = data

            elif packet_type == "angle":
                self.latest_angle = data
                # Publish as soon as we have an orientation update.
                # Do NOT gate this on acceleration/gyroscope also being
                # present -- some JY901B units are configured to stream
                # only angle (0x53) packets, and requiring all three
                # packet types meant /imu/data would never publish at
                # all in that case.
                self.publish_imu_message()

    def publish_imu_message(self) -> None:
        # latest_angle is guaranteed non-None here since this is only
        # called right after it's set, but keep the guard for safety.
        if self.latest_angle is None:
            return

        message = Imu()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

        roll = math.radians(self.latest_angle["roll"])
        pitch = math.radians(self.latest_angle["pitch"])
        yaw = math.radians(self.latest_angle["yaw"])

        qx, qy, qz, qw = euler_to_quaternion(
            roll,
            pitch,
            yaw,
        )

        message.orientation.x = qx
        message.orientation.y = qy
        message.orientation.z = qz
        message.orientation.w = qw
        # Orientation is always available whenever this is called,
        # since it's only invoked from a fresh angle packet. Zero
        # covariance means "unknown but present" -- replace with a
        # measured/datasheet variance if you need real fusion accuracy.
        message.orientation_covariance = list(ZERO_COVARIANCE)

        if self.latest_gyroscope is not None:
            # Convert degrees/second to radians/second.
            message.angular_velocity.x = math.radians(
                self.latest_gyroscope["x"]
            )
            message.angular_velocity.y = math.radians(
                self.latest_gyroscope["y"]
            )
            message.angular_velocity.z = math.radians(
                self.latest_gyroscope["z"]
            )
            message.angular_velocity_covariance = list(ZERO_COVARIANCE)
        else:
            message.angular_velocity_covariance = list(
                UNKNOWN_COVARIANCE
            )

            if not self._warned_missing_gyroscope:
                self._warned_missing_gyroscope = True
                self.get_logger().warn(
                    "No gyroscope (0x52) packets received yet -- "
                    "publishing /imu/data with angular_velocity "
                    "marked unknown (covariance[0] = -1). Check that "
                    "the JY901B is configured to output gyro data."
                )

        if self.latest_acceleration is not None:
            # Convert g to metres/second².
            message.linear_acceleration.x = (
                self.latest_acceleration["x"] * GRAVITY
            )
            message.linear_acceleration.y = (
                self.latest_acceleration["y"] * GRAVITY
            )
            message.linear_acceleration.z = (
                self.latest_acceleration["z"] * GRAVITY
            )
            message.linear_acceleration_covariance = list(
                ZERO_COVARIANCE
            )
        else:
            message.linear_acceleration_covariance = list(
                UNKNOWN_COVARIANCE
            )

            if not self._warned_missing_acceleration:
                self._warned_missing_acceleration = True
                self.get_logger().warn(
                    "No acceleration (0x51) packets received yet -- "
                    "publishing /imu/data with linear_acceleration "
                    "marked unknown (covariance[0] = -1). Check that "
                    "the JY901B is configured to output accel data."
                )

        self.imu_publisher.publish(message)
        self.published_message_count += 1

    def publish_temperature_message(
        self,
        acceleration_data: dict,
    ) -> None:
        message = Temperature()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.temperature = float(
            acceleration_data["temperature"]
        )

        # Zero means the temperature variance is unknown.
        message.variance = 0.0

        self.temperature_publisher.publish(message)

    def print_status(self) -> None:
        self.get_logger().info(
            f"Published {self.published_message_count} IMU messages; "
            f"checksum failures: {self.invalid_checksum_count}"
        )

    def destroy_node(self) -> None:
        if self.serial_port is not None:
            if self.serial_port.is_open:
                self.serial_port.close()

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[JY901BNode] = None

    try:
        node = JY901BNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except SerialException as error:
        print(f"Serial error: {error}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
