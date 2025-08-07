#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import sys
import os

# Add your SDK to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from STservo_sdk import *

class ServoBridge(Node):
    def __init__(self):
        super().__init__('servo_bridge')
        
        # Initialize your servo SDK
        self.port_handler = PortHandler('/dev/ttyACM0')
        self.packet_handler = sts(self.port_handler)
        
        # Open the port
        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open port /dev/ttyACM0')
            return
            
        if not self.port_handler.setBaudRate(1000000):
            self.get_logger().error('Failed to set baudrate')
            return
        
        self.get_logger().info('Servo port opened successfully')
        
        # Servo IDs
        self.servo_ids = [1, 2, 3, 4, 5]
        
        # Subscribers and Publishers
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, 
            '/servo_commands', 
            self.command_callback, 
            10
        )
        
        self.pos_pub = self.create_publisher(
            Float64MultiArray, 
            '/servo_positions', 
            10
        )
        
        # Timer for reading positions
        self.timer = self.create_timer(0.02, self.read_positions)  # 50Hz
        
        self.get_logger().info('Servo bridge initialized')
    
    def command_callback(self, msg):
        """Send commands to servos"""
        for i, position_rad in enumerate(msg.data):
            if i < len(self.servo_ids):
                # Convert rad to servo ticks (adjust conversion as needed)
                position_ticks = int(position_rad * 512 / 3.14159 + 512)
                position_ticks = max(0, min(1023, position_ticks))  # Clamp
                
                # Send to servo using your SDK
                self.packet_handler.WritePosEx(self.servo_ids[i], position_ticks, 500, 50)
    
    def read_positions(self):
        """Read actual servo positions and publish"""
        positions = []
        for servo_id in self.servo_ids:
            try:
                pos_ticks, result, error = self.packet_handler.ReadPos(servo_id)
                if result == COMM_SUCCESS:
                    # Convert ticks to radians (adjust conversion as needed)
                    pos_rad = (pos_ticks - 512) * 3.14159 / 512
                    positions.append(pos_rad)
                else:
                    positions.append(0.0)  # Default on error
            except Exception as e:
                self.get_logger().warn(f'Failed to read servo {servo_id}: {e}')
                positions.append(0.0)
        
        msg = Float64MultiArray()
        msg.data = positions
        self.pos_pub.publish(msg)

def main():
    rclpy.init()
    bridge = ServoBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
