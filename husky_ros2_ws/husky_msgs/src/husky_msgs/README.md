# Husky odometry and telemetry

The ROS node is named `husky_telemetry_node`.

Run the existing node with the Husky server running and `husky.py` on
`PYTHONPATH`:

```sh
source /opt/ros/lyrical/setup.bash
export PYTHONPATH=/home/robot/husky_ws/husky_essentials/src:$PYTHONPATH
colcon build --packages-select husky_msgs
. install/setup.bash
ros2 run husky_msgs husky_telemetry_node --ros-args -p telemetry_rate:=1.0
```

| Topic | ROS type | Contents | Default rate |
| --- | --- | --- | --- |
| `/husky/odom` | `nav_msgs/msg/Odometry` | Encoder-based pose and velocity | 20 Hz |
| `/husky/encoders` | `sensor_msgs/msg/JointState` | Left/right wheel position (rad) and velocity (rad/s) | 20 Hz |
| `/husky/system_status` | `std_msgs/msg/String` | JSON: ok, uptime_ms, voltages, currents, temperatures | 1 Hz |
| `/husky/safety_status` | `std_msgs/msg/String` | JSON: ok, flags, timeout, lockout, estop, cci, psu, current | 1 Hz |
| `/husky/differential_speed` | `std_msgs/msg/String` | JSON: ok, left_speed, right_speed, left_accel, right_accel | 1 Hz |

Encoder telemetry uses `sensor_msgs/msg/JointState`, with names
`left_wheel_joint` and `right_wheel_joint` in that order. These represent the
two measured drive sides; they are not four separate wheel measurements.
`position` is encoder travel divided by `wheel_radius`; `velocity` is encoder
speed divided by `wheel_radius`. `effort` is empty because it is not measured.
The header records the ROS receipt time. `ok` gates publication and the array
length replaces `count`; extra client fields are not included in this message.

`wheel_radius` defaults to **0.1651 m** (nominal Husky A200 wheel radius).
Set it to the actual effective rolling radius for your robot, for example
`--ros-args -p wheel_radius:=0.1651`. It must be finite and positive. Joint
names describe the two drive sides and may need mapping to your URDF names
before feeding a robot state publisher. Odometry continues to use the original
travel in metres and speed in metres per second and does not use this radius.

The other three telemetry topics retain the complete successful JSON response,
including future fields. Decode `message.data` with `json.loads`. There is no
suitable standard message that preserves all of these fields: `BatteryState`
does not cover system uptime and multiple power/temperature channels; diagnostics
store arbitrary values as strings; `Twist` does not retain separate left/right
speeds and acceleration limits. No custom interfaces are defined. Raw JSON
topics have no ROS header. Failed responses do not publish.

`publish_rate` controls encoder polling and odometry. `telemetry_rate` controls
system status, safety status, and differential speed polling and must be finite
and positive. Encoder telemetry reuses the odometry request. Failed requests
are logged and do not publish stale data; each status request is independent.
Requests use the existing synchronous client, so serial response times and
client timeouts can reduce achieved rates or delay other callbacks. Safety
telemetry is observational and is not an emergency-stop control mechanism.

```sh
ros2 topic echo /husky/system_status
ros2 topic echo /husky/safety_status
ros2 topic echo /husky/differential_speed
ros2 topic echo /husky/encoders
```

Connection parameters are `host` (default `127.0.0.1`), `port` (9000), and
`timeout` (2.0 seconds). For example, append `-p host:=192.168.1.10` after
`--ros-args` to connect to a remote Husky server. Restart an already running
node after rebuilding to load the new publishers.

The encoder topic type has changed. Rebuild and restart the node and update
encoder subscribers from `String`/JSON to `JointState`.
