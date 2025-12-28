from st3215 import ST3215
import time

servo = ST3215('/dev/ttyUSB0')
servo_id = 2  # Your servo
servo.Rotate(servo_id, 1500)
print("Current pos :", servo.ReadPosition(servo_id))
time.sleep(5)
print("-----")
servo.Rotate(servo_id, 1500)
print("Current pos :", servo.ReadPosition(servo_id))
time.sleep(1)
print("------------------")