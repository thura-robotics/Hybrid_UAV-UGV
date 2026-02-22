#!/usr/bin/env python3
"""
ROS 2 service node that wraps the ST3215 Python library.
Provides services for the C++ hardware interface to call.
"""

import time
import rclpy
from rclpy.node import Node
from ugv_motor_driver.srv import ReadPositions, WritePositions, WriteVelocities, ReadVelocities

try:
    from st3215 import ST3215
except ImportError:
    print("ERROR: st3215 library not found!")
    print("Make sure PYTHONPATH includes the st3215_driver directory")
    raise


class ST3215ServiceNode(Node):
    """Service node for ST3215 servo communication."""
    
    def __init__(self):
        super().__init__('st3215_service_node')
        
        # Parameters - provide defaults or types to avoid deprecation warnings
        self.declare_parameter('serial_port', rclpy.Parameter.Type.STRING)
        self.declare_parameter('servo_ids', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('position_servo_ids', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('velocity_servo_ids', rclpy.Parameter.Type.INTEGER_ARRAY)
        
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.servo_ids = list(self.get_parameter('servo_ids').get_parameter_value().integer_array_value)
        self.position_servo_ids = list(self.get_parameter('position_servo_ids').get_parameter_value().integer_array_value)
        self.velocity_servo_ids = list(self.get_parameter('velocity_servo_ids').get_parameter_value().integer_array_value)
        
        # Initialize ST3215 driver
        try:
            self.servo = ST3215(port)
            self.get_logger().info(f'Connected to ST3215 servos on {port}')
            
            # Detect servos
            detected = self.servo.ListServos()
            self.get_logger().info(f'Detected servos: {detected}')
            
            # Note: ListServos() doesn't always detect all servos (e.g., higher IDs like 12)
            # So we skip verification and just try to configure all servos
            for sid in self.servo_ids:
                if sid not in detected:
                    self.get_logger().warn(f'Servo {sid} not detected in scan, but will try to use it anyway')
            
            # Configure servos based on position_servo_ids and velocity_servo_ids
            for sid in self.servo_ids:
                if sid in self.velocity_servo_ids:
                    self.servo.StopServo(sid)  # Disable torque before mode switch
                    time.sleep(0.01)
                    self.servo.SetMode(sid, 1)  # Velocity mode
                    self.get_logger().info(f'Servo {sid} configured for velocity mode')
                elif sid in self.position_servo_ids:
                    # Read current position for logging
                    cur_pos = self.servo.ReadPosition(sid)
                    # Set mode/speed/acceleration WITHOUT disabling torque.
                    # Calling StopServo here would allow loaded servos (e.g. servo 10)
                    # to drift under gravity, then stall when the control loop
                    # re-engages torque at the new (wrong) position.
                    self.servo.SetMode(sid, 0)  # Position mode
                    self.servo.SetSpeed(sid, 400)
                    self.servo.SetAcceleration(sid, 50)
                    self.get_logger().info(f'Servo {sid} configured for position mode (speed: 400, pos: {cur_pos})')
                else:
                    self.get_logger().warn(f'Servo {sid} not in position or velocity list, skipping configuration')
                time.sleep(0.01)  # Small delay between servos
            
            self.get_logger().info(f'Configured servos: {self.servo_ids}')
            
            # Flush serial buffers after bulk configuration to prevent stale data
            self.servo.portHandler.ser.reset_input_buffer()
            self.servo.portHandler.ser.reset_output_buffer()
            time.sleep(0.5)  # Let serial bus settle
            self.get_logger().info('Serial buffers flushed, bus settled')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ST3215: {e}')
            raise
        
        # Create services
        self.read_srv = self.create_service(
            ReadPositions,
            'st3215/read_positions',
            self.read_positions_callback
        )
        
        self.write_srv = self.create_service(
            WritePositions,
            'st3215/write_positions',
            self.write_positions_callback
        )
        
        self.write_vel_srv = self.create_service(
            WriteVelocities,
            'st3215/write_velocities',
            self.write_velocities_callback
        )
        
        self.read_vel_srv = self.create_service(
            ReadVelocities,
            'st3215/read_velocities',
            self.read_velocities_callback
        )
        
        self.get_logger().info('ST3215 service node ready!')
        self.get_logger().info('Services:')
        self.get_logger().info('  - /st3215/read_positions')
        self.get_logger().info('  - /st3215/write_positions')
        self.get_logger().info('  - /st3215/write_velocities')
        self.get_logger().info('  - /st3215/read_velocities')
    
    def read_positions_callback(self, request, response):
        """Read positions from specified servos with per-servo retry logic."""
        try:
            positions = []
            has_failures = False
            for servo_id in request.servo_ids:
                # Velocity mode servos don't have meaningful position readings
                if servo_id in self.velocity_servo_ids:
                    positions.append(0)
                    continue
                
                pos = None
                for attempt in range(2):
                    # Flush input buffer before each read to clear stale data
                    self.servo.portHandler.ser.reset_input_buffer()
                    time.sleep(0.005)  # Let bus settle after flush
                    
                    try:
                        pos = self.servo.ReadPosition(servo_id)
                        if pos is not None:
                            break
                    except Exception as read_err:
                        self.get_logger().warn(
                            f'Servo {servo_id} read attempt {attempt+1}/2 exception: {read_err}'
                        )
                    time.sleep(0.05)  # Wait before retry
                
                if pos is not None:
                    positions.append(pos)
                else:
                    # Don't abort - continue reading remaining servos
                    positions.append(0)
                    has_failures = True
                    self.get_logger().warn(f'Servo {servo_id} read failed, using 0')
                
                # Small delay between servo reads to prevent bus congestion
                time.sleep(0.005)
            
            response.positions = positions
            response.success = True
            response.message = 'OK (some servos failed)' if has_failures else 'OK'
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.positions = []
            self.get_logger().error(f'Read error: {e}')
        
        return response
    
    def write_positions_callback(self, request, response):
        """Write positions to specified servos."""
        try:
            if len(request.servo_ids) != len(request.positions):
                response.success = False
                response.message = 'servo_ids and positions length mismatch'
                return response
            
            for servo_id, position in zip(request.servo_ids, request.positions):
                success = self.servo.WritePosition(servo_id, position)
                if not success:
                    response.success = False
                    response.message = f'Failed to write to servo {servo_id}'
                    return response
            
            response.success = True
            response.message = 'OK'
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Write error: {e}')
        
        return response

    def write_velocities_callback(self, request, response):
        """Write velocities to specified servos."""
        try:
            if len(request.servo_ids) != len(request.velocities):
                response.success = False
                response.message = 'servo_ids and velocities length mismatch'
                return response
            
            for servo_id, velocity in zip(request.servo_ids, request.velocities):
                # Clamp velocity to valid range (-3400 to 3400)
                # Negative = CCW, Positive = CW
                clamped_velocity = max(-3400, min(3400, int(velocity)))
                
                if clamped_velocity != velocity:
                    self.get_logger().warn(
                        f'Clamped velocity for servo {servo_id}: {velocity} -> {clamped_velocity}'
                    )
                
                success = self.servo.Rotate(servo_id, clamped_velocity)
                if not success:
                    response.success = False
                    response.message = f'Failed to write velocity to servo {servo_id}'
                    return response
            
            response.success = True
            response.message = 'OK'
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Write velocity error: {e}')
        
        return response

    def read_velocities_callback(self, request, response):
        """Read velocities from specified servos."""
        try:
            velocities = []
            for servo_id in request.servo_ids:
                # Only velocity mode servos have meaningful velocity readings
                if servo_id in self.velocity_servo_ids:
                    speed_result = self.servo.ReadSpeed(servo_id)
                    if speed_result and len(speed_result) >= 1:
                        speed = speed_result[0]  # ReadSpeed returns (speed, comm_result, error)
                        velocities.append(speed if speed is not None else 0)
                    else:
                        velocities.append(0)
                else:
                    # Position mode servos don't have meaningful velocity readings
                    velocities.append(0)
            
            response.velocities = velocities
            response.success = True
            response.message = 'OK'
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Read velocity error: {e}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ST3215ServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
