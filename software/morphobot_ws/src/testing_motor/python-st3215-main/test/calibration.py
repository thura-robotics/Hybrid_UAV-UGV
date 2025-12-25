#!/usr/bin/env python3
"""
ST3215 Middle Position Calibration
"""

import os
import sys
from st3215 import ST3215

SERVO_ID = 1

def main():
    print("=== ST3215 SET MIDDLE POSITION ===")

    device = os.getenv("ST3215_DEV")
    if not device:
        print("ST3215_DEV not set")
        sys.exit(1)

    servo = ST3215(device)
    print("✓ Connected")

    # Disable torque
    servo.StopServo(SERVO_ID)
    print("✓ Torque OFF")

    print("\nMove servo to PHYSICAL CENTER position")
    input("Press ENTER to save this as MIDDLE (2047)...")

    servo.DefineMiddle(SERVO_ID)
    print("✓ Middle position saved to EEPROM")

    print("\nPower cycle the servo now!")
    print("Then re-run a motion test.")

if __name__ == "__main__":
    main()
