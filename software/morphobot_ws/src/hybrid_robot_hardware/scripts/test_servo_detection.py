#!/usr/bin/env python3
"""
Test script to verify 3 servos are detected and configured.
Run this before launching the ROS 2 system.
"""

import sys

try:
    from st3215 import ST3215
except ImportError:
    print("❌ ST3215 library not found!")
    print("Install with: pip3 install --user -e /home/eisan/Hybrid_UAV-UGV/software/st3215_driver")
    sys.exit(1)


def main():
    print("=" * 70)
    print("ST3215 Servo Detection Test - 3 Servos")
    print("=" * 70)
    
    # Connect to servos
    try:
        servo = ST3215('/dev/ttyUSB0')
        print("✅ Connected to /dev/ttyUSB0")
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        print("\nTroubleshooting:")
        print("  1. Check connection: ls -l /dev/ttyUSB*")
        print("  2. Fix permissions: sudo chmod 666 /dev/ttyUSB0")
        print("  3. Check if servos are powered")
        return 1
    
    # Detect servos
    print("\n" + "-" * 70)
    print("Detecting servos...")
    ids = servo.ListServos()
    
    if not ids:
        print("❌ No servos found!")
        return 1
    
    print(f"✅ Found {len(ids)} servo(s): {ids}")
    
    # Check expected servos
    expected_ids = [1, 3, 4]
    if len(ids) < 3:
        print(f"⚠️  Expected at least 3 servos, found {len(ids)}")
        print("   Using first 3 detected servos for testing")
        expected_ids = ids[:3]
    
    # Configure servos
    print("\n" + "-" * 70)
    print("Configuring servos...")
    print(f"  Servo {expected_ids[0]}: Driving (velocity mode)")
    print(f"  Servo {expected_ids[1]}: Pan (position mode)")
    print(f"  Servo {expected_ids[2]}: Tilt (position mode)")
    
    try:
        # Driving servo (ID 1) - velocity mode
        servo.StartServo(expected_ids[0])
        servo.SetMode(expected_ids[0], 1)  # Velocity mode
        servo.SetSpeed(expected_ids[0], 3500)
        print(f"  ✅ Servo {expected_ids[0]} configured")
        
        # Pan servo (ID 2) - position mode
        servo.StartServo(expected_ids[1])
        servo.SetMode(expected_ids[1], 0)  # Position mode
        servo.SetSpeed(expected_ids[1], 2000)
        servo.SetAcceleration(expected_ids[1], 50)
        print(f"  ✅ Servo {expected_ids[1]} configured")
        
        # Tilt servo (ID 3) - position mode
        servo.StartServo(expected_ids[2])
        servo.SetMode(expected_ids[2], 0)  # Position mode
        servo.SetSpeed(expected_ids[2], 2000)
        servo.SetAcceleration(expected_ids[2], 50)
        print(f"  ✅ Servo {expected_ids[2]} configured")
        
    except Exception as e:
        print(f"❌ Configuration failed: {e}")
        return 1
    
    # Read positions
    print("\n" + "-" * 70)
    print("Reading current positions...")
    
    for servo_id in expected_ids:
        try:
            pos = servo.ReadPosition(servo_id)
            if pos is not None:
                pos_rad = (pos / 4095.0) * 2 * 3.14159
                print(f"  Servo {servo_id}: {pos} ({pos_rad:.3f} rad)")
            else:
                print(f"  Servo {servo_id}: Position read returned None")
        except Exception as e:
            print(f"  Servo {servo_id}: Read failed - {e}")
    
    # Test movements
    print("\n" + "-" * 70)
    print("Testing servo movements...")
    print("  (Servos will move briefly then stop)")
    
    import time
    
    try:
        # Test 1: Spin driving servo for 2 seconds
        print("\n  Test 1: Spinning driving servo (ID 1) for 2 seconds...")
        servo.SetSpeed(expected_ids[0], 1500)  # Moderate speed
        time.sleep(2)
        servo.SetSpeed(expected_ids[0], 0)  # Stop
        print("  ✅ Driving servo stopped")
        
        # Test 2: Move pan servo
        print("\n  Test 2: Moving pan servo (ID 3) to 45 degrees...")
        target_pos = int((0.785 / (2 * 3.14159)) * 4095)  # 45 degrees
        servo.WritePosition(expected_ids[1], target_pos)
        time.sleep(1.5)
        print("  ✅ Pan servo moved")
        
        # Test 3: Move tilt servo
        print("\n  Test 3: Moving tilt servo (ID 4) to -30 degrees...")
        target_pos = int((-0.524 / (2 * 3.14159)) * 4095)  # -30 degrees
        servo.WritePosition(expected_ids[2], target_pos)
        time.sleep(1.5)
        print("  ✅ Tilt servo moved")
        
        # Return pan/tilt to center
        print("\n  Returning pan/tilt to center position...")
        center_pos = int((0.0 / (2 * 3.14159)) * 4095)
        servo.WritePosition(expected_ids[1], center_pos)
        servo.WritePosition(expected_ids[2], center_pos)
        time.sleep(1.5)
        print("  ✅ Servos returned to center")
        
    except Exception as e:
        print(f"  ⚠️  Movement test error: {e}")
    
    # IMPORTANT: Stop all servos before exiting
    print("\n" + "-" * 70)
    print("Stopping all servos...")
    
    try:
        # Stop driving servo (velocity mode - set speed to 0)
        servo.SetSpeed(expected_ids[0], 0)
        print(f"  ✅ Stopped servo {expected_ids[0]} (driving)")
        
        # Disable torque on all servos
        for servo_id in expected_ids:
            servo.StopServo(servo_id)
            print(f"  ✅ Disabled torque on servo {servo_id}")
        
    except Exception as e:
        print(f"  ⚠️  Cleanup warning: {e}")
    
    # Summary
    print("\n" + "=" * 70)
    print("✅ Test Complete - Servos Ready for ROS 2")
    print("=" * 70)
    print("\nJoint Mapping:")
    print(f"  Servo {expected_ids[0]} → wheel_fl_joint (driving)")
    print(f"  Servo {expected_ids[1]} → pan_fl_joint (pan)")
    print(f"  Servo {expected_ids[2]} → tilt_fl_joint (tilt)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
