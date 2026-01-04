#!/usr/bin/env python3
"""
ROS 2 service node that wraps the ST3215 Python library.
Provides services for the C++ hardware interface to call.
"""

import rclpy
from rclpy.node import Node
from hybrid_robot_hardware.srv import ReadPositions, WritePositions

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
            
            # Configure servos
            for sid in self.servo_ids:
                self.servo.SetMode(sid, 0)  # Position mode
                self.servo.SetSpeed(sid, 2400)
                self.servo.SetAcceleration(sid, 50)
            
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
        
        self.get_logger().info('ST3215 service node ready!')
        self.get_logger().info('Services:')
        self.get_logger().info('  - /st3215/read_positions')
        self.get_logger().info('  - /st3215/write_positions')
    
    def read_positions_callback(self, request, response):
        """Read positions from specified servos."""
        try:
            positions = []
            for servo_id in request.servo_ids:
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
