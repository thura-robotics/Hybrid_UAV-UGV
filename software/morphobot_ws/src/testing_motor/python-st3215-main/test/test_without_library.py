import serial
import time

# -------- CONFIG --------
PORT = "/dev/ttyUSB0"   # change if needed (ttyTHS1, ttyACM0, etc.)
BAUDRATE = 100000  # Changed to match servo's actual baud rate
SERVO_ID = 3
# ------------------------

def checksum(packet):
    return (~sum(packet) & 0xFF)

def read_position(ser, servo_id):
    """
    Read current position from ST3215 servo
    Returns position (0-4095) or None if failed
    """
    # STS protocol READ packet
    packet = [
        0xFF, 0xFF,           # header
        servo_id,
        4,                    # length
        0x02,                 # instruction: READ
        0x38,                 # address: present position (STS_PRESENT_POSITION_L)
        2                     # read 2 bytes
    ]
    
    packet.append(checksum(packet[2:]))
    
    ser.write(bytearray(packet))
    time.sleep(0.05)
    
    # Read response (should be 8 bytes: FF FF ID LEN ERROR POS_L POS_H CHK)
    response = ser.read(8)
    
    if len(response) >= 7:
        pos_l = response[5]
        pos_h = response[6]
        position = pos_l | (pos_h << 8)
        return position
    else:
        return None

def enable_torque(ser, servo_id):
    """
    Enable torque on the servo (required before movement)
    """
    # STS protocol WRITE packet to enable torque
    packet = [
        0xFF, 0xFF,           # header
        servo_id,
        4,                    # length
        0x03,                 # instruction: WRITE
        0x28,                 # address: STS_TORQUE_ENABLE (register 40)
        1                     # value: 1 (enable)
    ]
    
    packet.append(checksum(packet[2:]))
    
    ser.write(bytearray(packet))
    time.sleep(0.05)

def move_servo(ser, servo_id, position, speed=1000, acc=50):
    """
    Move ST3215 servo
    position: 0–4095
    speed:    0–3000
    acc:      0–255
    """

    pos_l = position & 0xFF
    pos_h = (position >> 8) & 0xFF

    spd_l = speed & 0xFF
    spd_h = (speed >> 8) & 0xFF

    # STS protocol packet
    packet = [
        0xFF, 0xFF,           # header
        servo_id,
        7,                    # length
        0x03,                 # instruction: WRITE
        0x2A,                 # address: goal position
        pos_l, pos_h,
        spd_l, spd_h,
        acc
    ]

    packet.append(checksum(packet[2:]))

    ser.write(bytearray(packet))

# ---------- MAIN ----------
if __name__ == "__main__":
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    time.sleep(1)
    
    # Enable torque (required before movement!)
    print("Enabling torque...")
    enable_torque(ser, SERVO_ID)
    
    # Read current position
    current_pos = read_position(ser, SERVO_ID)
    print(f"Current position: {current_pos}")
    
    print(f"Moving to position {current_pos + 100}")
    move_servo(ser, SERVO_ID, current_pos + 100)
    time.sleep(2)
    
    new_pos = read_position(ser, SERVO_ID)
    print(f"Position after move: {new_pos}")
    
    print(f"\nMoving to position {current_pos + 300}")
    move_servo(ser, SERVO_ID, current_pos + 300)
    time.sleep(2)
    
    final_pos = read_position(ser, SERVO_ID)
    print(f"Final position: {final_pos}")

    ser.close()
    print("\nTest done")
