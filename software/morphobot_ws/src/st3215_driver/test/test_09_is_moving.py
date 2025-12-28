#!/usr/bin/env python3

import os
import sys
import time
from st3215 import ST3215

SERVO_ID = 1
STEP_DELTA = 200
SPEED = 500
ACC = 50
SLEEP = 0.1

def main():
    print("=== ST3215 DIAGNOSTIC TEST ===")

    # -----------------------------
    # Get serial device
    # -----------------------------
    device = os.getenv("ST3215_DEV")
    if not device:
        print("export ST3215_DEV=/dev/ttyUSB0")
        sys.exit(1)

    # -----------------------------
    # Connect
    # -----------------------------
    servo = ST3215(device)
    print("✓ Connected")

    # -----------------------------
    # Enable servo
    # -----------------------------
    if not servo.StartServo(SERVO_ID):
        print("❌ Failed to enable servo")
        sys.exit(1)

    print("✓ Servo enabled")

    # -----------------------------
    # Read servo mode
    # -----------------------------
    mode = servo.ReadMode(SERVO_ID)
    print(f"Servo mode: {mode}")

    # -----------------------------
    # Explicitly set position mode and parameters
    # -----------------------------
    print("Setting position mode...")
    servo.SetMode(SERVO_ID, 0)  # Position mode
    time.sleep(0.1)
    
    print("Setting acceleration...")
    servo.SetAcceleration(SERVO_ID, ACC)
    time.sleep(0.1)
    
    print("Setting speed...")
    servo.SetSpeed(SERVO_ID, SPEED)
    time.sleep(0.1)

    # -----------------------------
    # Read current position
    # -----------------------------
    start = servo.ReadPosition(SERVO_ID)
    if start is None:
        print("❌ Failed to read position")
        servo.StopServo(SERVO_ID)
        sys.exit(1)

    print(f"Start position: {start}")

    # -----------------------------
    # Test rotation mode first (like test_10)
    # -----------------------------
    print("\n=== Testing Rotation Mode ===")
    print("Rotating clockwise for 2 seconds...")
    servo.Rotate(SERVO_ID, 500)
    time.sleep(2)
    
    print("Rotating counter-clockwise for 2 seconds...")
    servo.Rotate(SERVO_ID, -500)
    time.sleep(2)
    
    print("Stopping rotation...")
    servo.SetMode(SERVO_ID, 0)  # Back to position mode
    time.sleep(0.5)
    
    # Read position after rotation
    current = servo.ReadPosition(SERVO_ID)
    print(f"Position after rotation: {current}\n")
    
    # Use current position as new start
    start = current
    print("=== Testing Position Control ===")

    # -----------------------------
    # Move +1000 steps (like test_10)
    # -----------------------------
    target = start + 1000
    print(f"\n→ Move to {target} (+1000)")

    result = servo.MoveTo(SERVO_ID, target, speed=1500, acc=80, wait=True)
    if not result:
        print("  ❌ MoveTo failed")
    
    pos = servo.ReadPosition(SERVO_ID)
    print(f"Reached position: {pos}")
    
    if pos is not None and abs(pos - target) < 50:
        print(f"  ✓ Success! Delta: {pos - start}")
    else:
        print(f"  ✗ Failed! Expected ~{target}, got {pos}")

    time.sleep(0.5)

    # -----------------------------
    # Move -1000 steps from original (like test_10)
    # -----------------------------
    target = start - 1000
    print(f"\n← Move to {target} (-1000)")

    result = servo.MoveTo(SERVO_ID, target, speed=1500, acc=80, wait=True)
    if not result:
        print("  ❌ MoveTo failed")
    
    pos = servo.ReadPosition(SERVO_ID)
    print(f"Reached position: {pos}")
    
    if pos is not None and abs(pos - target) < 50:
        print(f"  ✓ Success! Delta: {pos - start}")
    else:
        print(f"  ✗ Failed! Expected ~{target}, got {pos}")

    time.sleep(0.5)

    # -----------------------------
    # Return to start (like test_10)
    # -----------------------------
    print(f"\n↺ Return to {start}")

    result = servo.MoveTo(SERVO_ID, start, speed=1500, acc=80, wait=True)
    if not result:
        print("  ❌ MoveTo failed")
    
    pos = servo.ReadPosition(SERVO_ID)
    print(f"Reached position: {pos}")
    
    if pos is not None and abs(pos - start) < 50:
        print(f"  ✓ Returned to start!")
    else:
        print(f"  ✗ Failed! Expected ~{start}, got {pos}")

    # -----------------------------
    # Disable servo
    # -----------------------------
    servo.StopServo(SERVO_ID)
    print("\n✓ Servo disabled")
    print("✓ Diagnostic completed")

if __name__ == "__main__":
    main()
