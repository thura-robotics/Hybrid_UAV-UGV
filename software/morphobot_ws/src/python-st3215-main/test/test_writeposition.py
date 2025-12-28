import sys
import os

# Add the st3215 module to Python path
sys.path.insert(0, '/home/eisan/Hybrid_UAV-UGV/software/morphobot_ws/src/python-st3215-main')

from st3215 import ST3215
import time

# Initialize
servo = ST3215('/dev/ttyUSB0')
ids = servo.ListServos()

if not ids:
    print("❌ No servos found!")
    exit()

servo_id = ids[0]  # Use first servo
print(f"Using servo ID: {servo_id}")

# Setup ONCE (torque + position mode + speed + acceleration)
print("Setting up servo...")
servo.StartServo(servo_id)
servo.SetMode(servo_id, 0)  # Position mode
servo.SetSpeed(servo_id, 3000)  # Max speed
servo.SetAcceleration(servo_id, 50)  # High acceleration

print("Moving: 1000 → 500\n")

positions = [1000, 500]

start_time = time.time()

for i, target_position in enumerate(positions, 1):
    print(f"{'='*60}")
    print(f"STEP {i}: Moving to {target_position}")
    
    move_start = time.time()
    success = servo.WritePosition(servo_id, target_position)
    
    if not success:
        print(f"❌ Failed to send command")
        continue
    
    print(f"✅ Command sent")
    
    # Wait for movement to complete using IsMoving()
    timeout = 3  # 5 second timeout
    while time.time() - move_start < timeout:
        is_moving = servo.IsMoving(servo_id)
        if is_moving is False:
            break
        time.sleep(0.01)  # Check every 10ms
    
    move_time = time.time() - move_start
    
    # Verify position
    actual_pos = servo.ReadPosition(servo_id)
    error = abs(actual_pos - target_position) if actual_pos else None
    
    print(f"Target: {target_position}, Actual: {actual_pos}, Error: {error}, Time: {move_time:.3f}s")

end_time = time.time()
total_time = end_time - start_time

print(f"\n{'='*60}")
print(f"✅ Total time: {total_time:.2f} seconds")
print(f"⚡ Average time per move: {total_time/len(positions):.3f} seconds")
