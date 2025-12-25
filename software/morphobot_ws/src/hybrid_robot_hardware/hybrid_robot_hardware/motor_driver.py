#!/usr/bin/env python3
# This is for ros2_control
import serial
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class ST3215Serial:
    """
    Low-level driver for Waveshare ST3215 Serial Bus Servos.
    Uses the correct protocol based on motor_driver_test.py
    """
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, timeout=0.1):
        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.1)  # Allow serial to stabilize

    def _calc_checksum(self, data):
        """Calculate checksum for packet"""
        return (~sum(data)) & 0xFF

    def write_position(self, servo_id, position, speed=0):
        """
        Write target position to servo.
        Position: 0-4095 (maps to 0-360 degrees)
        """
        position = max(0, min(4095, int(position)))
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF

        # Build command packet
        HEADER = [0xFF, 0xFF]  # Correct header for ST3215
        INSTRUCTION = 0x03  # WRITE
        ADDR_POSITION = 0x2A  # Target position address
        LENGTH = 5

        packet = [
            servo_id,
            LENGTH,
            INSTRUCTION,
            ADDR_POSITION,
            pos_l,
            pos_h
        ]

        chk = self._calc_checksum(packet)
        full_packet = HEADER + packet + [chk]
        self.serial.write(bytearray(full_packet))

    def read_position(self, servo_id):
        """
        Read current position from servo.
        Returns position (0-4095) or -1 on failure.
        """
        HEADER = [0xFF, 0xFF]  # Correct header for ST3215
        INSTRUCTION = 0x02  # READ
        ADDR_POSITION = 0x38  # Current position address (not 0x2A!)
        SIZE = 2
        LENGTH = 4

        packet = [
            servo_id,
            LENGTH,
            INSTRUCTION,
            ADDR_POSITION,
            SIZE
        ]

        chk = self._calc_checksum(packet)
        full_packet = HEADER + packet + [chk]

        self.serial.reset_input_buffer()
        self.serial.write(bytearray(full_packet))
        
        # Small delay for servo response
        time.sleep(0.01)

        response = self.serial.read(8)

        if len(response) < 8:
            return -1

        pos_l = response[5]
        pos_h = response[6]

        return pos_l | (pos_h << 8)


class WaveshareST3215HardwareInterface:
    """
    Hardware interface for ros2_control.
    Implements read() and write() methods.
    
    WORKAROUND_MODE: Set to True to bypass half-duplex read issue.
    This tracks commanded positions instead of reading actual positions.
    """
    WORKAROUND_MODE = True  # Set to False when half-duplex hardware is fixed
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, servo_ids=None):
        if servo_ids is None:
            servo_ids = list(range(1, 13))  # Default 1-12
        self.servo_ids = servo_ids
        self.driver = ST3215Serial(port=port, baudrate=baudrate)
        
        # State storage (in radians for ros2_control)
        self.target_positions = {sid: 0.0 for sid in servo_ids}  # radians
        self.current_positions = {sid: 0.0 for sid in servo_ids}  # radians
        self.current_velocities = {sid: 0.0 for sid in servo_ids}  # rad/s
        
        # Previous positions for velocity calculation
        self._prev_positions = {sid: 0.0 for sid in servo_ids}
        self._prev_time = time.time()
        
        # Workaround: simulated positions (gradually move toward target)
        self._simulated_positions = {sid: 0.0 for sid in servo_ids}  # radians

    def _ticks_to_radians(self, ticks):
        """Convert servo ticks (0-4095) to radians (0-2π)"""
        return (ticks / 4095.0) * 2.0 * 3.14159265359

    def _radians_to_ticks(self, radians):
        """Convert radians to servo ticks (0-4095)"""
        # Normalize to 0-2π range
        radians = radians % (2.0 * 3.14159265359)
        return int((radians / (2.0 * 3.14159265359)) * 4095.0)

    def read(self):
        """
        Read actual position from each servo.
        Updates current_positions and current_velocities.
        
        WORKAROUND MODE: Simulates positions moving toward targets.
        """
        current_time = time.time()
        dt = current_time - self._prev_time
        
        if self.WORKAROUND_MODE:
            # Workaround: Simulate gradual movement toward target
            # ST3215 servos move at ~0.17s/60° at max speed
            # That's ~6.1 rad/s max angular velocity
            MAX_VELOCITY = 6.0  # rad/s
            
            for sid in self.servo_ids:
                target = self.target_positions[sid]
                current = self._simulated_positions[sid]
                
                # Calculate error
                error = target - current
                
                # Normalize to [-π, π]
                while error > 3.14159265359:
                    error -= 2 * 3.14159265359
                while error < -3.14159265359:
                    error += 2 * 3.14159265359
                
                # Move toward target with max velocity limit
                if dt > 0:
                    max_step = MAX_VELOCITY * dt
                    step = max(-max_step, min(max_step, error))
                    self._simulated_positions[sid] += step
                    
                    # Normalize position to [0, 2π]
                    self._simulated_positions[sid] = self._simulated_positions[sid] % (2 * 3.14159265359)
                    
                    # Update current position
                    self.current_positions[sid] = self._simulated_positions[sid]
                    
                    # Calculate velocity
                    self.current_velocities[sid] = (self.current_positions[sid] - self._prev_positions[sid]) / dt
                    
                    self._prev_positions[sid] = self.current_positions[sid]
        else:
            # Real hardware read (requires half-duplex fix)
            for sid in self.servo_ids:
                ticks = self.driver.read_position(sid)
                if ticks != -1:
                    pos_rad = self._ticks_to_radians(ticks)
                    self.current_positions[sid] = pos_rad
                    
                    # Calculate velocity
                    if dt > 0:
                        self.current_velocities[sid] = (pos_rad - self._prev_positions[sid]) / dt
                    
                    self._prev_positions[sid] = pos_rad
        
        self._prev_time = current_time
        return self.current_positions

    def write(self):
        """
        Send target position to each servo over serial.
        """
        for sid, target_rad in self.target_positions.items():
            ticks = self._radians_to_ticks(target_rad)
            self.driver.write_position(sid, ticks)

    def set_command(self, servo_id, position_rad):
        """Set command position for a specific servo"""
        if servo_id in self.target_positions:
            self.target_positions[servo_id] = position_rad


class ST3215DriverNode(Node):
    """
    ROS 2 node that publishes joint states and accepts commands.
    This provides a simple interface for testing without full ros2_control.
    """
    def __init__(self):
        super().__init__('st3215_driver_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Initialize hardware interface
        try:
            self.hw_interface = WaveshareST3215HardwareInterface(port=port)
            self.get_logger().info(f'Connected to ST3215 servos on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer for read/write loop
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        
        self.get_logger().info(f'ST3215 driver started at {rate} Hz')

    def control_loop(self):
        """Main control loop: read sensors, publish states, write commands"""
        # Read from hardware
        self.hw_interface.read()
        
        # Publish joint states
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for sid in self.hw_interface.servo_ids:
            msg.name.append(f'servo_joint_{sid}')
            msg.position.append(self.hw_interface.current_positions[sid])
            msg.velocity.append(self.hw_interface.current_velocities[sid])
            msg.effort.append(0.0)  # Not available
        
        self.joint_state_pub.publish(msg)
        
        # Write to hardware
        self.hw_interface.write()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ST3215DriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
