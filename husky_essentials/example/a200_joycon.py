#!/usr/bin/env python3

import time
import select
from evdev import InputDevice, list_devices, ecodes

from husky import HuskyClient


DEADZONE = 0.08
UPDATE_HZ = 20.0

MAX_SPEED_L1 = 0.5
MAX_SPEED_R1 = 1.0
ACCEL = 0.5


def clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadzone(value):
    if abs(value) < DEADZONE:
        return 0.0
    return value


def normalise_axis(value, info, invert=False):
    centre = (info.min + info.max) / 2.0
    half_range = (info.max - info.min) / 2.0

    if half_range == 0:
        return 0.0

    out = (value - centre) / half_range
    out = clamp(out, -1.0, 1.0)

    if invert:
        out = -out

    return apply_deadzone(out)


def find_controller():
    while True:
        devices = [InputDevice(path) for path in list_devices()]

        for dev in devices:
            name = dev.name.lower()
            if "wireless controller" in name or "dualshock" in name or "playstation" in name:
                return dev

        for dev in devices:
            caps = dev.capabilities()
            if ecodes.EV_ABS in caps and ecodes.EV_KEY in caps:
                return dev
        print("No controller found. Waiting...")
        time.sleep(1.0)


def main():
    dev = find_controller()
    dev.grab()

    print(f"Using controller: {dev.name}")
    print(f"Path: {dev.path}")

    abs_info = dev.capabilities().get(ecodes.EV_ABS, [])
    abs_map = {code: info for code, info in abs_info}

    lx_code = ecodes.ABS_X
    ly_code = ecodes.ABS_Y

    if lx_code not in abs_map or ly_code not in abs_map:
        raise RuntimeError("Controller does not expose ABS_X / ABS_Y left stick axes")

    lx = 0.0
    ly = 0.0
    l1 = False
    r1 = False

    husky = HuskyClient()
    dt = 1.0 / UPDATE_HZ
    next_send = time.monotonic()

    try:
        while True:
            timeout = max(0.0, next_send - time.monotonic())
            readable, _, _ = select.select([dev.fd], [], [], timeout)

            if readable:
                for event in dev.read():
                    if event.type == ecodes.EV_ABS:
                        if event.code == ecodes.ABS_X:
                            lx = normalise_axis(event.value, abs_map[ecodes.ABS_X])
                        elif event.code == ecodes.ABS_Y:
                            ly = normalise_axis(event.value, abs_map[ecodes.ABS_Y], invert=True)

                    elif event.type == ecodes.EV_KEY:
                        if event.code == ecodes.BTN_TL:      # L1
                            l1 = event.value == 1
                        elif event.code == ecodes.BTN_TR:    # R1
                            r1 = event.value == 1

            now = time.monotonic()
            if now >= next_send:
                if r1:
                    max_speed = MAX_SPEED_R1
                elif l1:
                    max_speed = MAX_SPEED_L1
                else:
                    max_speed = 0.0

                if max_speed > 0.0:
                    forward = ly
                    turn = lx

                    left_speed = clamp(forward + turn, -1.0, 1.0) * max_speed
                    right_speed = clamp(forward - turn, -1.0, 1.0) * max_speed
                else:
                    left_speed = 0.0
                    right_speed = 0.0

                husky.set_differential_speed(
                    left_speed=left_speed,
                    right_speed=right_speed,
                    left_accel=ACCEL,
                    right_accel=ACCEL,
                )

                next_send = now + dt

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        try:
            husky.set_differential_speed(0.0, 0.0, ACCEL, ACCEL)
        finally:
            dev.ungrab()


if __name__ == "__main__":
    main()
