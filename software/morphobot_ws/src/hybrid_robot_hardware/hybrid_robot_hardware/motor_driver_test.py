# Plug in the servo shield, then run: ls /dev/ttyUSB*
# Typical output: /dev/ttyUSB0
# Run --> python3 st3215_test.py
# Low level test for servo driver

# This script opens a serial connection, sends binary commands to a servo to move it, asks the servo where it is, and prints the answer.

import serial
import time

# ===== CONFIG =====
PORT = "/dev/ttyUSB0"   # CHANGE THIS if needed  /dev/ttyUSB1, /dev/ttyACM0
BAUDRATE = 1000000
SERVO_ID = 1
# ==================

def checksum(data):
    return (~sum(data)) & 0xFF

# Open serial port
ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    timeout=0.1
)

time.sleep(2)
print("Serial port opened")

def set_servo_position(servo_id, position):
    pos_l = position & 0xFF
    pos_h = (position >> 8) & 0xFF

    #Build the command packet
    HEADER = [0x55, 0x55]
    INSTRUCTION = 0x03      # WRITE
    ADDR_POSITION = 0x2A
    LENGTH = 5

    packet = [
        servo_id,
        LENGTH,
        INSTRUCTION,
        ADDR_POSITION,
        pos_l,
        pos_h
    ]

    chk = checksum(packet)
    full_packet = HEADER + packet + [chk]

    ser.write(bytearray(full_packet))

def read_servo_position(servo_id):
    HEADER = [0x55, 0x55]
    INSTRUCTION = 0x02      # READ
    ADDR_POSITION = 0x2A
    SIZE = 2
    LENGTH = 4

    packet = [
        servo_id,
        LENGTH,
        INSTRUCTION,
        ADDR_POSITION,
        SIZE
    ]

    chk = checksum(packet)
    full_packet = HEADER + packet + [chk]

    ser.reset_input_buffer()
    ser.write(bytearray(full_packet))

    response = ser.read(8)

    if len(response) < 8:
        return None

    pos_l = response[5]
    pos_h = response[6]

    return pos_l | (pos_h << 8)

# ===== TEST SEQUENCE =====
try:
    print("Moving to center...")
    set_servo_position(SERVO_ID, 2048)
    time.sleep(1)

    print("Moving to max...")
    set_servo_position(SERVO_ID, 3000)
    time.sleep(1)

    print("Moving to min...")
    set_servo_position(SERVO_ID, 1000)
    time.sleep(1)

    pos = read_servo_position(SERVO_ID)
    print("Current position:", pos)

except KeyboardInterrupt:
    pass

finally:
    ser.close()
    print("Serial port closed")

