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

# Setup (torque + position mode)
servo.StartServo(servo_id)
servo.SetMode(servo_id, 0)
servo.SetSpeed(servo_id, 800)
servo.SetAcceleration(servo_id, 50)

print("2000 → 1000 → 2000\n")

positions = [2000, 1000, 2000]
for i, target_position in enumerate(positions, 1):
    print(f"{'='*60}")
    print(f"STEP {i}")
    
    
    # Original position
    orig_pos = servo.ReadPosition(servo_id)
    print(f"Original position: {orig_pos}")
    success = servo.MoveTo(servo_id, target_position, speed=3500, acc=50, wait=True)
    # print(f"✅ {'Move OK' if success else '❌ Move failed'}")
    
    
    # first_pos = servo.ReadPosition(servo_id)
    # mode = servo.ReadMode(servo_id)
    second_pos = servo.ReadPosition(servo_id)
    
    # print(f"First position:  {first_pos}")
    # print(f"Mode:           {mode} (should be 0)")
    print(f"Second position:{second_pos}")
    
    
    time.sleep(1)  # Pause between moves

