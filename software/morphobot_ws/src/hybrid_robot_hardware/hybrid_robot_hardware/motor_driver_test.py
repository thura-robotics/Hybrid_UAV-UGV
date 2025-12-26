#!/usr/bin/env python3
import serial
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 1000000  # ✅ CRITICAL FIX
SERVO_ID = 1

def checksum(packet):  # ✅ Fixed checksum
    return (~sum(packet[2:]) & 0xFF) & 0xFF

def make_ping_packet(servo_id):
    packet = [0xFF, 0xFF, servo_id, 0x03, 0x01, 0x00]
    packet.append(checksum(packet))
    return bytes(packet)

def make_torque_packet(servo_id, enable=1):  # ✅ NEW: Torque enable
    packet = [0xFF, 0xFF, servo_id, 0x04, 0x03, 0x19, enable]
    packet.append(checksum(packet))
    return bytes(packet)

def make_mode_packet(servo_id, mode=0):  # ✅ NEW: Position mode
    packet = [0xFF, 0xFF, servo_id, 0x04, 0x03, 0x11, mode]
    packet.append(checksum(packet))
    return bytes(packet)

def make_move_packet(servo_id, position):  # ✅ Fixed length 0x06
    addr = 0x1E
    low = position & 0xFF
    high = (position >> 8) & 0xFF
    packet = [0xFF, 0xFF, servo_id, 0x06, 0x03, addr, low, high]  # LEN=6
    packet.append(checksum(packet))
    return bytes(packet)

def make_read_position_packet(servo_id):
    packet = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x24, 0x02]
    packet.append(checksum(packet))
    return bytes(packet)

# Open port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
print(f"Connected @ {BAUD_RATE} baud")

# 1️⃣ Essential setup
print("1. Enabling torque...")
ser.write(make_torque_packet(SERVO_ID, 1))
time.sleep(0.2)

print("2. Setting position mode...")
ser.write(make_mode_packet(SERVO_ID, 0))
time.sleep(0.2)

# 2️⃣ Ping test
print("3. Ping test...")
ser.write(make_ping_packet(SERVO_ID))
time.sleep(0.1)
resp = ser.read(10)
print(f"Ping: {resp.hex() if resp else 'No response'}")

# 3️⃣ Test movements
positions = [1800, 2047, 2300, 2047]  # Small movements around center
for i, pos in enumerate(positions):
    print(f"\nMoving to {pos} ({i+1}/4)...")
    ser.write(make_move_packet(SERVO_ID, pos))
    time.sleep(1.5)  # Wait for movement
    
    # Read position
    ser.write(make_read_position_packet(SERVO_ID))
    time.sleep(0.1)
    resp = ser.read(11)
    if resp and len(resp) >= 9:
        current_pos = (resp[6] << 8) | resp[5]
        print(f"✅ Reached: {current_pos}")
    else:
        print("❌ Read failed")

ser.write(make_torque_packet(SERVO_ID, 0))  # Disable torque
ser.close()
print("\n✅ Test complete!")
