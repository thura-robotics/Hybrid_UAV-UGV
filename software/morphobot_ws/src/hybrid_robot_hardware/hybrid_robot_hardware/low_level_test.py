import serial
import time
import struct

# Baud rate for this servo (confirmed working)
baud_rate = 100000

# IDs to test
servo_id = 1  # Change to your servo ID

# Position range: 0-4095 (360°), middle is ~2048
# NOTE: Due to corrupted angle limits, only positions 0-26 are allowed
test_positions = [0, 10, 20]

def make_ping_packet(servo_id):
    """Ping packet: FF FF ID 03 00 01 CHK"""
    ID = servo_id
    LEN = 3
    CMD = 0x01  # Ping
    PARAM = 0x00
    chk = (~(ID + LEN + CMD + PARAM)) & 0xFF
    return bytes([0xFF, 0xFF, ID, LEN, CMD, PARAM, chk])

def make_write_register_packet(servo_id, address, value):
    """
    Write to a register: FF FF ID LEN CMD ADDR VALUE CHK
    """
    ID = servo_id
    CMD = 0x03  # Write command
    LEN = 4  # CMD + ADDR + VALUE
    
    # For single byte values
    if isinstance(value, int) and value < 256:
        data = bytes([CMD, address, value])
    else:
        # For multi-byte values (little endian)
        data = bytes([CMD, address, value & 0xFF, (value >> 8) & 0xFF])
        LEN = 5
    
    chk = (~(ID + LEN + sum(data))) & 0xFF
    packet = bytes([0xFF, 0xFF, ID, LEN]) + data + bytes([chk])
    return packet

def make_read_register_packet(servo_id, address, length=2):
    """
    Read from a register: FF FF ID LEN CMD ADDR LENGTH CHK
    """
    ID = servo_id
    CMD = 0x02  # Read command
    LEN = 4  # CMD + ADDR + LENGTH
    
    data = bytes([CMD, address, length])
    chk = (~(ID + LEN + sum(data))) & 0xFF
    packet = bytes([0xFF, 0xFF, ID, LEN]) + data + bytes([chk])
    return packet

def parse_position_response(response):
    """Parse position from read response"""
    if len(response) >= 7:
        # Response format: FF FF ID LEN ERROR POS_L POS_H CHK
        pos_l = response[5]
        pos_h = response[6]
        position = pos_l | (pos_h << 8)
        return position
    return None

def make_write_pos_packet(servo_id, position, speed=1500, acc=50):
    """
    Write Position packet: FF FF ID 0A 03 1F POS_L POS_H SPD_L SPD_H ACC CHK
    Position: 0-4095 (12-bit little endian)
    Speed: 0-4095 
    Acc: 0-255
    """
    ID = servo_id
    LEN = 0x0A  # 10 data bytes
    CMD = 0x1F  # WritePosEx command
    pos = position & 0xFFF  # 12-bit position
    spd = speed & 0xFFF      # 12-bit speed
    acc_val = acc & 0xFF     # 8-bit acceleration
    
    # Pack data: CMD + params (POS_L, POS_H, SPD_L, SPD_H, ACC)
    pos_l = pos & 0xFF
    pos_h = (pos >> 8) & 0xFF
    spd_l = spd & 0xFF
    spd_h = (spd >> 8) & 0xFF
    data = struct.pack('<BBBBBB', CMD, pos_l, pos_h, spd_l, spd_h, acc_val)
    
    chk = (~(ID + LEN + sum(data))) & 0xFF
    packet = bytes([0xFF, 0xFF, ID, LEN]) + data + bytes([chk])
    return packet


# Connect to servo at known baud rate
print(f"Connecting to servo at {baud_rate} baud...")
try:
    ser = serial.Serial('/dev/ttyUSB0', baud_rate, timeout=0.5)
    packet = make_ping_packet(servo_id)
    ser.write(packet)
    time.sleep(0.05)
    resp = ser.read(8)
    if resp:
        print(f"✓ Servo {servo_id} found! Response: {resp.hex()}\n")
    else:
        print("❌ No response from servo!")
        ser.close()
        exit(1)
    ser.close()
except Exception as e:
    print(f"❌ Error: {e}")
    exit(1)

# Now test position control
print(f"Testing position control at {baud_rate} baud...")
try:
    ser = serial.Serial('/dev/ttyUSB0', baud_rate, timeout=1.0)
    
    # IMPORTANT: Enable torque first (write 1 to register 40)
    print("Enabling torque...")
    STS_TORQUE_ENABLE = 40
    torque_packet = make_write_register_packet(servo_id, STS_TORQUE_ENABLE, 1)
    print(f"Torque packet: {torque_packet.hex()}")
    ser.write(torque_packet)
    ser.flush()
    time.sleep(0.1)
    resp = ser.read(20)
    if resp:
        print(f"Torque enable response: {resp.hex()}")
    

    # Register addresses from ST3215 protocol
    STS_MODE = 33
    STS_ACC = 41
    STS_GOAL_POSITION_L = 42
    STS_GOAL_SPEED_L = 46
    
    # Set mode to 0 (position mode)
    print("Setting mode to position control (0)...")
    mode_packet = make_write_register_packet(servo_id, STS_MODE, 0)
    ser.write(mode_packet)
    ser.flush()
    time.sleep(0.05)
    
    # Set acceleration
    print("Setting acceleration to 50...")
    acc_packet = make_write_register_packet(servo_id, STS_ACC, 50)
    ser.write(acc_packet)
    ser.flush()
    time.sleep(0.05)
    
    # Set speed
    print("Setting speed to 1500...")
    speed_packet = make_write_register_packet(servo_id, STS_GOAL_SPEED_L, 1500)
    ser.write(speed_packet)
    time.sleep(0.05)
    
    # Read angle limits to check if they're restricting movement
    print("\nReading servo configuration...")
    STS_MIN_ANGLE_LIMIT_L = 9
    STS_MAX_ANGLE_LIMIT_L = 11
    
    # Read min angle limit
    min_limit_packet = make_read_register_packet(servo_id, STS_MIN_ANGLE_LIMIT_L, 2)
    ser.write(min_limit_packet)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(20)
    if len(resp) >= 7:
        min_limit = resp[5] | (resp[6] << 8)
        print(f"Min angle limit: {min_limit}")
    
    # Read max angle limit
    max_limit_packet = make_read_register_packet(servo_id, STS_MAX_ANGLE_LIMIT_L, 2)
    ser.write(max_limit_packet)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(20)
    if len(resp) >= 7:
        max_limit = resp[5] | (resp[6] << 8)
        print(f"Max angle limit: {max_limit}")
    
    # Register for reading current position
    STS_PRESENT_POSITION_L = 56
    
    for pos in test_positions:
        print(f"\n{'='*50}")
        print(f"Target position: {pos}")
        
        # Read current position before movement
        read_packet = make_read_register_packet(servo_id, STS_PRESENT_POSITION_L, 2)
        ser.write(read_packet)
        ser.flush()
        time.sleep(0.05)
        resp = ser.read(20)
        current_pos = parse_position_response(resp)
        if current_pos is not None:
            print(f"Current position BEFORE: {current_pos}")
        else:
            print(f"Failed to read position. Response: {resp.hex() if resp else 'None'}")
        
        # Write position to registers 42-43 (little endian)
        pos_packet = make_write_register_packet(servo_id, STS_GOAL_POSITION_L, pos)
        print(f"Sending position command: {pos_packet.hex()}")
        
        ser.write(pos_packet)
        ser.flush()
        time.sleep(0.1)
        
        # Read write response
        resp = ser.read(20)
        if resp:
            print(f"Write response: {resp.hex()}")
        
        # Wait for movement
        print("Waiting for movement...")
        time.sleep(2.0)
        
        # Read current position after movement
        read_packet = make_read_register_packet(servo_id, STS_PRESENT_POSITION_L, 2)
        ser.write(read_packet)
        ser.flush()
        time.sleep(0.05)
        resp = ser.read(20)
        new_pos = parse_position_response(resp)
        if new_pos is not None:
            print(f"Current position AFTER: {new_pos}")
            if new_pos == pos:
                print("✓ Position reached!")
            else:
                print(f"⚠ Position mismatch! Expected {pos}, got {new_pos}")
        else:
            print(f"Failed to read position. Response: {resp.hex() if resp else 'None'}")
        
        # Read status register (65) to check for errors
        STS_STATUS = 65
        read_status = make_read_register_packet(servo_id, STS_STATUS, 1)
        ser.write(read_status)
        ser.flush()
        time.sleep(0.05)
        resp = ser.read(20)
        if len(resp) >= 6:
            status_byte = resp[5]
            print(f"Status register: 0x{status_byte:02x} (binary: {bin(status_byte)})")
            errors = []
            if status_byte & 0x01: errors.append("Voltage")
            if status_byte & 0x02: errors.append("Sensor")
            if status_byte & 0x04: errors.append("Temperature")
            if status_byte & 0x08: errors.append("Current")
            if status_byte & 0x10: errors.append("Angle")
            if status_byte & 0x20: errors.append("Overload")
            if errors:
                print(f"⚠ ERRORS: {', '.join(errors)}")
        
        # Read moving register (66)
        STS_MOVING = 66
        read_moving = make_read_register_packet(servo_id, STS_MOVING, 1)
        ser.write(read_moving)
        ser.flush()
        time.sleep(0.05)
        resp = ser.read(20)
        if len(resp) >= 6:
            moving = resp[5]
            print(f"Moving status: {'YES' if moving else 'NO'}")


    
    ser.close()
    print("Position test complete!")
    
except Exception as e:
    print(f"Error during position test: {e}")
