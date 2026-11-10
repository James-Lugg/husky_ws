from husky import HuskyClient
import time

husky = HuskyClient()
print(husky.get_system_status())
# print(husky.get_velocity())
# print(husky.get_orientation())
# print(husky.get_rotation())
# print(husky.get_acceleration())
print(husky.get_safety_status())
print(husky.get_differential_speed())
print(husky.get_encoders())

speed = -0.2

try:
    print("Moving forward...")
    for i in range(80):
        # print(husky.get_encoders()['encoders'][0]['speed'], husky.get_encoders()['encoders'][1]['speed'])
        husky.set_differential_speed(
            left_speed=speed,
            right_speed=speed,
            left_accel=0.5,
            right_accel=0.5,
        )

        time.sleep(0.05)

finally:
    print("Stopping...")

    # husky.set_differential_speed(
    #         left_speed=0.0,
    #         right_speed=0.0,
    #         left_accel=0.5,
    #         right_accel=0.5,
    # )

    husky.stop()
