#!/usr/bin/env python3

import os
import sys
import time
from st3215 import ST3215

SERVO_ID = 1

ACC = 100
SPEED = 2000
ROTATE_SPEED = 500
DELTA = 1000


def require_device():
    dev = os.getenv("ST3215_DEV")
    if not dev:
        print("❌ ST3215_DEV not set (e.g. /dev/ttyUSB0)")
        sys.exit(1)
    return dev


def print_move(label, start, end):
    print(f"{label}: {start} → {end} (Δ {end - start})")


def main():
    print("=== ST3215 MOTION TEST ===")

    servo = ST3215(require_device())

    if not servo.StartServo(SERVO_ID):
        print("❌ Failed to enable servo")
        sys.exit(1)

    servo.SetAcceleration(SERVO_ID, ACC)
    servo.SetSpeed(SERVO_ID, SPEED)

    # ---------------- Rotation mode ----------------
    start = servo.ReadPosition(SERVO_ID)
    print(f"Rotate CW start: {start}")

    servo.Rotate(SERVO_ID, ROTATE_SPEED)
    time.sleep(3)
    servo.Rotate(SERVO_ID, 0)

    end = servo.ReadPosition(SERVO_ID)
    print_move("Rotate CW", start, end)

    start = end
    servo.Rotate(SERVO_ID, -ROTATE_SPEED)
    time.sleep(3)
    servo.Rotate(SERVO_ID, 0)

    end = servo.ReadPosition(SERVO_ID)
    print_move("Rotate CCW", start, end)

    # ---------------- Position mode ----------------
    
    current = servo.ReadPosition(SERVO_ID)
    print(f"\nStart position: {current}")

    target = 3360
    servo.MoveTo(SERVO_ID, target, wait=True)
    time.sleep(1)
    end = servo.ReadPosition(SERVO_ID)
    print_move("Move to 3360", current, end)

    # current = end
    # target = current - DELTA
    # servo.MoveTo(SERVO_ID, target, wait=True)
    # time.sleep(1)
    # end = servo.ReadPosition(SERVO_ID)
    # print_move("Move -DELTA", current, end)

    # servo.StopServo(SERVO_ID)
    # print("✓ Test complete")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted — stopping servo")
        try:
            servo.StopServo(SERVO_ID)
        except:
            pass
