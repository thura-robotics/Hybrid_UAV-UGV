#!/usr/bin/env python3
import sys
import rclpy
import math
import threading
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# --- Configurations ---
rad = math.radians(90.0)
STEP_DELAY = 3.0  # Seconds between morphing steps

# Joint order: hip_FL, hip_FR, hip_BL, hip_BR, ankle_FL, ankle_FR, ankle_BL, ankle_BR
MODES = {
    'uav': [
        [rad, -rad, -rad, rad, rad, rad, -rad, rad],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ],
    'ugv': [
        [0.7, -0.7, -0.7, 0.7, rad, rad, -rad, rad],
        [rad, -rad, -rad, rad, rad, rad, -rad, rad],
        [rad, -rad, -rad, rad, 0.0, 0.0, 0.0, 0.0]
    ],
    'zero': [
        [rad, -rad, -rad, rad, 0.0, 0.0, 0.0, 0.0]
    ]
}

# Velocity commands (FL, FR, BL, BR)
# Note: Since the axes for left wheels (FL, BL) were inverted in the URDF earlier,
# we may need to check signs. But the user explicitly inverted them back in step 740.
# So if axes are consistent (0 1 0), then left wheels might need negative velocities to move forward.
# HOWEVER, the user also added wheel_reversed: true for right side in controllers.yaml at step 737.
# Now that we've switched to a raw velocity controller, we control the signs ourselves.

V = 3.0 # Standard speed
DRIVE_MODES = {
    'f':  [V, V, V, V],
    'b':  [-V, -V, -V, -V],
    'l':  [0.8, V, 0.8, V],
    'r':  [V, 0.0, V, 0.0],
    's':  [0.0, 0.0, 0.0, 0.0],
    'c':  [4.0, 0.0, 4.0, 1.0]
}

class SimMorphNode(Node):
    def __init__(self):
        super().__init__('sim_morph_node')
        self.pos_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        
        self.timer = None
        self.current_mode = None
        
        # For morphing
        self.sequence = []
        self.step_index = 0
        
        # For driving
        self.drive_cmd = [0.0, 0.0, 0.0, 0.0]

    def start_command(self, mode):
        if self.timer:
            self.timer.cancel()
            self.timer = None
            
        self.current_mode = mode
        
        if mode in MODES:
            self.sequence = MODES[mode]
            self.step_index = 0
            self.get_logger().info(f'Starting morph sequence: {mode.upper()} ({len(self.sequence)} steps)')
            self.timer = self.create_timer(0.1, self.publish_step) # Start immediately
        elif mode in DRIVE_MODES:
            self.drive_cmd = DRIVE_MODES[mode]
            
            if mode == 's':
                self.get_logger().info('Stopping robot...')
                self.publish_stop()
                return

            self.get_logger().info(f'Starting continuous drive: {mode.upper()} (Type "s" to halt)')
            self.timer = self.create_timer(0.1, self.publish_drive)
        else:
            self.get_logger().error(f"Unknown command '{mode}'")

    def publish_stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.vel_pub.publish(msg)

    def publish_drive(self):
        msg = Float64MultiArray()
        msg.data = self.drive_cmd
        self.vel_pub.publish(msg)

    def publish_step(self):
        if self.step_index == 0:
            self.timer.cancel()
            self.timer = self.create_timer(STEP_DELAY, self.publish_step)

        msg = Float64MultiArray()
        msg.data = self.sequence[self.step_index]
        self.pos_pub.publish(msg)
        self.get_logger().info(f'Morph Step {self.step_index + 1}/{len(self.sequence)} published')
        
        self.step_index += 1
        if self.step_index >= len(self.sequence):
            self.timer.cancel()
            self.timer = None
            self.get_logger().info('Morph sequence complete.')

def input_thread(node):
    print("\n--- Morphobot Sim Control (Direct Velocity) ---")
    print(f"Modes: {list(MODES.keys())}")
    print(f"Drive: {list(DRIVE_MODES.keys())}")
    print("Type 'exit' to quit\n")
    
    while rclpy.ok():
        try:
            cmd = input("Enter command: ").strip().lower()
            if cmd == 'exit':
                rclpy.shutdown()
                break
            if cmd:
                node.start_command(cmd)
        except EOFError:
            break

def main(args=None):
    rclpy.init(args=args)
    node = SimMorphNode()
    
    if len(sys.argv) > 1:
        initial_cmd = sys.argv[1].lower()
        node.start_command(initial_cmd)

    thread = threading.Thread(target=input_thread, args=(node,), daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
