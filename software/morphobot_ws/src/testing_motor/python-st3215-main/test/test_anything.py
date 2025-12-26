
    
# import time
# # or correct import for your library

# # DEVICENAME = "/dev/ttyUSB0"
# # sts_id = 1

# # sts = ST3215(DEVICENAME)

# # sts.SetMode(sts_id, 1)
# # print("Servo mode :", sts.ReadMode(sts_id))

# from st3215 import ST3215

# servo = ST3215('/dev/ttyUSB0')


# ids = servo.ListServos()

# print("Connected servos:", ids)
 

# servo mode check for 

# for id in ids:
#     while True:
#     # print(f"Servo ID {id} is in { {1: 'velocity', 0: 'position'}.get(servo.ReadMode(id), 'unknown') } mode")
#         servo.MoveTo(id, 2000)
#         print(servo.IsMoving(id))
#         print("Read position:",servo.ReadPosition(id))
#         print("-----------------")


# while True:
#     servo.MoveTo(ids[0], 3000)
#     print(servo.IsMoving(ids[0]))
#     print("Read position:",servo.ReadPosition(ids[0]))
#     print("Read mode :", servo.ReadMode(ids[0]))
#     print("-----------------")
# while True:
#     servo.MoveTo(ids[1], 2000) 
#     print(servo.IsMoving(ids[1])) 
#     print("Read position:",servo.ReadPosition(ids[1]))
#     print("Read mode :", servo.ReadMode(ids[0]))


from st3215 import ST3215
import time

servo = ST3215('/dev/ttyUSB0')
ids = servo.ListServos()

if not ids:
    print("No servos found!")
    exit()

servo_id = ids[0]


# 🔥 CRITICAL SETUP (run ONCE before loop)
servo.StartServo(servo_id)           # Enable torque
servo.SetMode(servo_id, 0)           # Position mode (0)
servo.SetAcceleration(servo_id, 50)  # Smooth accel
servo.SetSpeed(servo_id, 800)        # Moderate speed



# ✅ PROPER POSITION MOVEMENT LOOP
target_position = 1000  # Your desired position (0-4095)

print(f"Original position: {servo.ReadPosition(servo_id)}")
print(f"\n=== Moving to {target_position} ===")
    
    # 1. Send position command
success = servo.MoveTo(servo_id, target_position, speed=800, acc=50, wait=True)
if success:
    print("✅ Move command sent & completed")
else:
    print("❌ Move failed")
    
    # 2. Verify actual position

print(f"First position: {servo.ReadPosition(servo_id)}")
    
    # 3. Status check

mode = servo.ReadMode(servo_id)

print(f"Mode: {mode} (should be 0)")
    

    # Optional: Move back to center
# servo.MoveTo(servo_id, 1024, wait=True)
print(f"Second position: {servo.ReadPosition(servo_id)}")
print("--- -------")
    
