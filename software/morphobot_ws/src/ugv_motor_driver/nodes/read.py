#read servo position 
from st3215 import ST3215
import time

s = ST3215('/dev/ttyUSB0')

# read all servos
# for i in [1,2,3,4,5,6,7,8,9,10,11,12]:
#     print('servo',i,':',s.ReadPosition(i))
#     time.sleep(0.1)


for i in [1, 2, 4, 5, 7, 8, 10, 11]:
    print('servo',i,':',s.ReadPosition(i))
    time.sleep(0.1)
# speed = 2000
# m_speed = -2000
# # Drive servos 3, 6, 9, 12 with velocity mode
# # 3, 6 → forward (300), 9, 12 → opposite direction (-300)
# DRIVE_SERVOS = {3: speed, 6: speed, 9: m_speed, 12: m_speed}
# print("\nStarting velocity mode on servos", list(DRIVE_SERVOS.keys()))
# for sid, speed in DRIVE_SERVOS.items():
#     s.StartServo(sid)
#     s.Rotate(sid, speed)

# time.sleep(10)

# # Stop all
# print("Stopping...")
# for sid in DRIVE_SERVOS:
#     s.Rotate(sid, 0)
#     s.SetMode(sid, 0)  # back to position mode
#     s.StopServo(sid)
# print("Done.")