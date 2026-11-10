"""Exercise telemetry and odometry with ROS messages and a fake client."""

import json
import math

from husky_msgs import husky_telemetry_node
import pytest
import rclpy


class RecordingPublisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def node(monkeypatch):
    class FakeClient:

        def __init__(self, **kwargs):
            self.encoders = {
                'ok': True,
                'encoders': [
                    {'travel': 0.0, 'speed': 1.0},
                    {'travel': 0.0, 'speed': 1.0},
                ],
            }

        def get_encoders(self):
            return self.encoders

        def get_system_status(self):
            raise TimeoutError('System status unavailable')

        def get_safety_status(self):
            return {'ok': True, 'estop': False}

        def get_differential_speed(self):
            return {'ok': True, 'left_speed': 1.0, 'right_speed': 1.0}

    monkeypatch.setattr(husky_telemetry_node, 'HuskyClient', FakeClient)
    rclpy.init()
    instance = husky_telemetry_node.HuskyTelemetryNode()
    instance.publisher = RecordingPublisher()
    instance.telemetry_publishers = {
        name: RecordingPublisher() for name in instance.telemetry_publishers
    }
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_status_failure_does_not_skip_other_topics(node):
    node.update_telemetry()
    assert not node.telemetry_publishers['system_status'].messages
    for name in ('safety_status', 'differential_speed'):
        messages = node.telemetry_publishers[name].messages
        assert len(messages) == 1
        assert json.loads(messages[0].data)['ok']


def test_encoder_telemetry_reuses_odometry_reading(node):
    node.update_odometry()
    for encoder in node.husky.encoders['encoders']:
        encoder['travel'] = 1.0
    node.update_odometry()
    messages = node.telemetry_publishers['encoders'].messages
    assert len(messages) == 2
    message = messages[-1]
    assert message.name == ['left_wheel_joint', 'right_wheel_joint']
    assert list(message.position) == pytest.approx([1.0 / node.wheel_radius] * 2)
    assert list(message.velocity) == pytest.approx([1.0 / node.wheel_radius] * 2)
    assert not message.effort
    assert message.header.stamp.sec > 0
    assert node.publisher.messages[-1].pose.pose.position.x == 1.0
    assert node.publisher.messages[-1].twist.twist.linear.x == 1.0


def test_invalid_encoder_does_not_corrupt_pose(node):
    node.update_odometry()
    node.husky.encoders['encoders'][0]['travel'] = math.nan
    node.update_odometry()
    assert len(node.publisher.messages) == 1
    assert node.previous_left_travel == 0.0
    assert node.x == 0.0


def test_extra_non_json_field_does_not_stop_encoder_publication(node):
    node.husky.encoders['extra'] = math.nan
    node.update_odometry()
    assert len(node.publisher.messages) == 1
    assert len(node.telemetry_publishers['encoders'].messages) == 1


def test_asymmetric_encoders_preserve_order_and_units(node):
    node.husky.encoders['encoders'] = [
        {'travel': 2.0, 'speed': -0.3},
        {'travel': -1.0, 'speed': 0.6},
    ]
    node.update_odometry()
    message = node.telemetry_publishers['encoders'].messages[-1]
    assert list(message.position) == pytest.approx(
        [2.0 / node.wheel_radius, -1.0 / node.wheel_radius])
    assert list(message.velocity) == pytest.approx(
        [-0.3 / node.wheel_radius, 0.6 / node.wheel_radius])


def test_invalid_encoder_is_not_published(node):
    node.husky.encoders['encoders'][0]['travel'] = math.nan
    node.update_odometry()
    assert not node.telemetry_publishers['encoders'].messages
