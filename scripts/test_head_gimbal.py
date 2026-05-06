#!/usr/bin/env python3

import argparse
import sys
import time

from instinct_onboard.servo.controller import GimbalController


def print_angles(controller: GimbalController, prefix: str = ""):
    pan, tilt = controller.get_gimbal_angles()
    print(f"{prefix}pan={pan}, tilt={tilt}")


def main():
    parser = argparse.ArgumentParser(description="Test the two head gimbal servos.")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--pan-id", type=int, default=0, help="Pan servo id")
    parser.add_argument("--tilt-id", type=int, default=1, help="Tilt servo id")
    parser.add_argument("--read", action="store_true", help="Only read current angles")
    parser.add_argument("--ping", action="store_true", help="Ping the two configured servo ids")
    parser.add_argument("--pan", type=float, default=None, help="Target pan angle in degrees")
    parser.add_argument("--tilt", type=float, default=None, help="Target tilt angle in degrees")
    parser.add_argument("--delay", type=float, default=1.5, help="Delay after each motion in seconds")
    parser.add_argument("--sweep", action="store_true", help="Run a small safe sweep on both axes")
    parser.add_argument("--release", action="store_true", help="Put both servos into damping mode")
    parser.add_argument("--hold", action="store_true", help="Stop both servos in hold mode")
    args = parser.parse_args()

    controller = GimbalController(
        serial_port=args.port,
        baudrate=args.baudrate,
        pan_servo_id=args.pan_id,
        tilt_servo_id=args.tilt_id,
        pan_range=(-90.0, 90.0),
        tilt_range=(-45.0, 45.0),
        use_dryrun=False,
    )

    if not controller.connect():
        print(f"Failed to connect to {args.port}", file=sys.stderr)
        sys.exit(1)

    try:
        if args.ping:
            print(f"ping pan({args.pan_id})={controller.ping(args.pan_id)}")
            print(f"ping tilt({args.tilt_id})={controller.ping(args.tilt_id)}")

        if args.release:
            print(f"damping pan({args.pan_id})={controller.damping(args.pan_id, 500)}")
            print(f"damping tilt({args.tilt_id})={controller.damping(args.tilt_id, 500)}")

        if args.hold:
            print(f"hold pan({args.pan_id})={controller.stop(args.pan_id)}")
            print(f"hold tilt({args.tilt_id})={controller.stop(args.tilt_id)}")

        if args.read:
            print_angles(controller, "current: ")

        if args.pan is not None or args.tilt is not None:
            current_pan, current_tilt = controller.get_gimbal_angles()
            target_pan = current_pan if args.pan is None else args.pan
            target_tilt = current_tilt if args.tilt is None else args.tilt
            print(f"move to pan={target_pan}, tilt={target_tilt}")
            ok = controller.set_gimbal_angle(target_pan, target_tilt)
            print(f"set_gimbal_angle={ok}")
            time.sleep(args.delay)
            print_angles(controller, "after move: ")

        if args.sweep:
            sequence = [
                (0.0, 0.0),
                (20.0, 0.0),
                (-20.0, 0.0),
                (0.0, 10.0),
                (0.0, -10.0),
                (15.0, 10.0),
                (0.0, 0.0),
            ]
            for idx, (pan, tilt) in enumerate(sequence, start=1):
                print(f"[{idx}/{len(sequence)}] move to pan={pan}, tilt={tilt}")
                ok = controller.set_gimbal_angle(pan, tilt)
                print(f"set_gimbal_angle={ok}")
                time.sleep(args.delay)
                print_angles(controller, "after move: ")

        if not any([args.read, args.ping, args.release, args.hold, args.sweep, args.pan is not None, args.tilt is not None]):
            print("No action requested. Try --read, --ping, --pan/--tilt, or --sweep.")

    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
