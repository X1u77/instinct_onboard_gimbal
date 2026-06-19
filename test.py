import time
from instinct_onboard.servo.controller import GimbalController

controller = GimbalController(
    serial_port="/dev/ttyUSB0",
    baudrate=115200,
    pan_servo_id=0,
    tilt_servo_id=1,
    pan_range=(-90.0, 90.0),
    tilt_range=(-90.0, 90.0),
    use_dryrun=False,
)

if not controller.connect():
    raise RuntimeError("connect failed")

try:
    print("move yaw -> 40")
    controller.set_gimbal_angle(60.0, 30.0)
    time.sleep(2)

    print("move yaw -> -40")
    controller.set_gimbal_angle(-60.0, 30.0)
    time.sleep(2)

    print("move pitch -> 0")
    controller.set_gimbal_angle(0, 0.0)
    time.sleep(2)

    print("move pitch -> 90")
    controller.set_gimbal_angle(0, 90.0)
    time.sleep(2)

finally:
    controller.disconnect()
