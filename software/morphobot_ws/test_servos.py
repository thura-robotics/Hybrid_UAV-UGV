#!/usr/bin/env python3
"""
Diagnostic script to test ST3215 servo communication directly.
This bypasses ROS 2 to isolate hardware issues.
"""

import sys
import time

# Add the ST3215 driver to path
sys.path.insert(0, '/home/eisan/Hybrid_UAV-UGV/software/morphobot_ws/src/hybrid_robot_hardware/hybrid_robot_hardware')

from ST3215 import ST3215

def test_servos():
    """Test servo communication and configuration."""
    
    print("=" * 60)
    print("ST3215 Servo Diagnostic Test")
    print("=" * 60)
    
    # Initialize driver
    serial_port = '/dev/ttyUSB0'
    print(f"\n1. Connecting to {serial_port}...")
    
    try:
        servo = ST3215(serial_port)
        print("   ✓ Serial port opened successfully")
    except Exception as e:
        print(f"   ✗ Failed to open serial port: {e}")
        return False
    
    # Detect servos
    print("\n2. Detecting servos...")
    servo_ids = [1, 2, 3]
    detected = []
    
    for sid in servo_ids:
        try:
            pos = servo.ReadPosition(sid)
            if pos is not None:
                detected.append(sid)
                print(f"   ✓ Servo {sid} detected (position: {pos})")
            else:
                print(f"   ✗ Servo {sid} not responding")
        except Exception as e:
            print(f"   ✗ Servo {sid} error: {e}")
    
    if not detected:
        print("\n⚠️  No servos detected! Check:")
        print("   - Power supply to servos")
        print("   - Serial cable connections")
        print("   - Servo IDs are correct")
        return False
    
    print(f"\n   Detected {len(detected)} servo(s): {detected}")
    
    # Test configuration
    print("\n3. Testing servo configuration...")
    
    for sid in detected:
        try:
            # Try to set position mode
            servo.SetMode(sid, 0)  # Position mode
            time.sleep(0.1)
            print(f"   ✓ Servo {sid} set to position mode")
            
            # Try to read position
            pos = servo.ReadPosition(sid)
            if pos is not None:
                print(f"   ✓ Servo {sid} position: {pos} ticks")
            else:
                print(f"   ✗ Servo {sid} position read failed")
                
        except Exception as e:
            print(f"   ✗ Servo {sid} configuration error: {e}")
    
    # Test velocity mode for servo 3
    if 3 in detected:
        print("\n4. Testing velocity mode on servo 3...")
        try:
            servo.SetMode(3, 1)  # Velocity mode
            time.sleep(0.1)
            print("   ✓ Servo 3 set to velocity mode")
            
            # Test velocity command
            servo.WriteSpeed(3, 100)  # Slow speed
            print("   ✓ Velocity command sent (100 ticks/s)")
            time.sleep(0.5)
            
            # Stop
            servo.WriteSpeed(3, 0)
            print("   ✓ Servo 3 stopped")
            
        except Exception as e:
            print(f"   ✗ Velocity mode test failed: {e}")
    
    print("\n" + "=" * 60)
    print("Diagnostic Complete")
    print("=" * 60)
    
    if len(detected) == 3:
        print("\n✓ All servos are working correctly!")
        print("  The issue is likely with ROS 2 service timing.")
        print("  Try increasing the delay in the launch file.")
        return True
    else:
        print(f"\n⚠️  Only {len(detected)}/3 servos detected.")
        print("  Fix hardware issues before running ROS 2 Control.")
        return False

if __name__ == '__main__':
    try:
        success = test_servos()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
