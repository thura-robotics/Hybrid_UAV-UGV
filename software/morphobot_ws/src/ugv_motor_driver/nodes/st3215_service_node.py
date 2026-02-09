#!/usr/bin/env python3
"""
ROS 2 service node that wraps the ST3215 Python library.
Provides services for the C++ hardware interface to call.
"""

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
        
        # Parameters - no defaults, must be provided via YAML file
        self.declare_parameter('serial_port')
        self.declare_parameter('servo_ids')
        self.declare_parameter('position_servo_ids')
        self.declare_parameter('velocity_servo_ids')
        
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
            
            # Verify configured servos
            for sid in self.servo_ids:
                if sid not in detected:
                    self.get_logger().error(f'Servo {sid} not detected!')
                    raise RuntimeError(f'Servo {sid} not found')
            
            # Configure servos based on position_servo_ids and velocity_servo_ids
            for sid in self.servo_ids:
                if sid in self.velocity_servo_ids:
                    self.servo.SetMode(sid, 1)  # Velocity mode
                    self.get_logger().info(f'Servo {sid} configured for velocity mode')
                elif sid in self.position_servo_ids:
                    self.servo.SetMode(sid, 0)  # Position mode
                    self.servo.SetSpeed(sid, 400)  # Set constant speed to 400
                    self.servo.SetAcceleration(sid, 50)
                    self.get_logger().info(f'Servo {sid} configured for position mode (speed: 400)')
                else:
                    self.get_logger().warn(f'Servo {sid} not in position or velocity list, skipping configuration')
            
            self.get_logger().info(f'Configured servos: {self.servo_ids}')
            
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
        """Read positions from specified servos."""
        try:
            positions = []
            for servo_id in request.servo_ids:
                # Velocity mode servos don't have meaningful position readings
                # Return 0 as a placeholder since position is not meaningful in velocity mode
                if servo_id in self.velocity_servo_ids:
                    positions.append(0)
                else:
                    pos = self.servo.ReadPosition(servo_id)
                    if pos is not None:
                        positions.append(pos)
                    else:
                        response.success = False
                        response.message = f'Failed to read servo {servo_id}'
                        return response
            
            response.positions = positions
            response.success = True
            response.message = 'OK'
            
        except Exception as e:
            response.success = False
            response.message = str(e)
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
