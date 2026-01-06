#!/usr/bin/env python3
"""
ROS 2 service node that wraps the ST3215 Python library.
Provides services for the C++ hardware interface to call.
"""

import rclpy
from rclpy.node import Node
from hybrid_robot_hardware.srv import ReadPositions, WritePositions, WriteVelocities, ReadVelocities

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
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('servo_ids', [1, 3, 4])
        
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.servo_ids = self.get_parameter('servo_ids').get_parameter_value().integer_array_value
        
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
            
            # Configure servos - servo 1 in velocity mode, others in position mode
            for sid in self.servo_ids:
                if sid == 1:
                    self.servo.SetMode(sid, 1)  # Velocity mode for servo 1
                    self.get_logger().info(f'Servo {sid} configured for velocity mode')
                else:
                    self.servo.SetMode(sid, 0)  # Position mode for others
                    self.servo.SetSpeed(sid, 2400)
                    self.servo.SetAcceleration(sid, 50)
                    self.get_logger().info(f'Servo {sid} configured for position mode')
            
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
                # Servo 1 is in velocity mode, position reading may not work reliably
                # Return 0 as a placeholder since position is not meaningful in velocity mode
                if servo_id == 1:
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
                success = self.servo.Rotate(servo_id, velocity)
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
                # Only servo 1 is in velocity mode, read its speed
                if servo_id == 1:
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
