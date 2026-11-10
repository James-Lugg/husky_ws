import json
import socket


class HuskyClient:
    def __init__(self, host="127.0.0.1", port=9000, timeout=2.0):
        self.host = host
        self.port = port
        self.timeout = timeout

    def _call(self, **payload):
        with socket.create_connection((self.host, self.port), timeout=self.timeout) as s:
            msg = json.dumps(payload).encode("utf-8") + b"\n"
            s.sendall(msg)

            data = b""
            while not data.endswith(b"\n"):
                chunk = s.recv(4096)
                if not chunk:
                    break
                data += chunk

        response = json.loads(data.decode("utf-8"))

        if not response.get("ok", False):
            raise RuntimeError(response.get("error", "Unknown Husky server error"))

        return response

    def ping(self):
        return self._call(cmd="ping")

    def stop(self, accel=1.0):
        return self.set_differential_speed(
                left_speed=0,
                right_speed=0,
                left_accel=0.5,
                right_accel=0.5,
        )

    # Use this for "move forward":
    # forward movement is left_speed == right_speed.
    def set_differential_speed(
        self,
        left_speed,
        right_speed,
        left_accel=1.0,
        right_accel=1.0,
    ):
        return self._call(
            cmd="set_differential_speed",
            left_speed=left_speed,
            right_speed=right_speed,
            left_accel=left_accel,
            right_accel=right_accel,
        )

    def set_velocity(self, translational, rotational, trans_accel=1.0):
        return self._call(
            cmd="set_velocity",
            translational=translational,
            rotational=rotational,
            trans_accel=trans_accel,
        )

    def set_turn(self, translational, turn_radius, trans_accel=1.0):
        return self._call(
            cmd="set_turn",
            translational=translational,
            turn_radius=turn_radius,
            trans_accel=trans_accel,
        )

    def set_differential_output(self, left, right):
        return self._call(
            cmd="set_differential_output",
            left=left,
            right=right,
        )

    def set_ackermann_output(self, steering, throttle, brake):
        return self._call(
            cmd="set_ackermann_output",
            steering=steering,
            throttle=throttle,
            brake=brake,
        )

    def set_max_speed(self, max_fwd, max_rev):
        return self._call(
            cmd="set_max_speed",
            max_fwd=max_fwd,
            max_rev=max_rev,
        )

    def set_max_accel(self, max_fwd, max_rev):
        return self._call(
            cmd="set_max_accel",
            max_fwd=max_fwd,
            max_rev=max_rev,
        )

    def set_gear(self, gear):
        return self._call(cmd="set_gear", gear=gear)

    def set_platform_name(self, name):
        return self._call(cmd="set_platform_name", name=name)

    def set_platform_time(self, time_value):
        return self._call(cmd="set_platform_time", time=time_value)

    def set_safety_system(self, flags):
        return self._call(cmd="set_safety_system", flags=flags)

    def store_settings(self):
        return self._call(cmd="store_settings")

    def restore_user_settings(self):
        return self._call(cmd="restore_user_settings")

    def restore_factory_settings(self):
        return self._call(cmd="restore_factory_settings")

    def processor_reset(self):
        return self._call(cmd="processor_reset")

    def get_system_status(self):
        return self._call(cmd="get_system_status")

    def get_velocity(self):
        return self._call(cmd="get_velocity")

    def get_orientation(self):
        return self._call(cmd="get_orientation")

    def get_rotation(self):
        return self._call(cmd="get_rotation")

    def get_acceleration(self):
        return self._call(cmd="get_acceleration")

    def get_safety_status(self):
        return self._call(cmd="get_safety_status")

    def get_differential_speed(self):
        return self._call(cmd="get_differential_speed")

    def get_encoders(self):
        return self._call(cmd="get_encoders")
