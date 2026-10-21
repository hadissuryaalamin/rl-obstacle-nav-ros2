#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        print("Lidar Reader Started... Move the robot to the wall!")

    def listener_callback(self, msg):
        # 1. Convert to numpy array
        ranges = np.array(msg.ranges)
        
        # 2. Filter out 'inf' (infinity means far away)
        # We replace inf with 10.0 so it doesn't mess up the min() calculation
        ranges = np.nan_to_num(ranges, posinf=10.0, neginf=10.0)
        
        # 3. Get the Front Cone 
        # 30 degrees Left + 30 degrees Right
        front_cone = np.concatenate((ranges[0:30], ranges[-30:]))
        
        # 4. Find the closest point
        min_dist = np.min(front_cone)
        
        # 5. Print clearly
        # \r makes it overwrite the same line so it doesn't spam your terminal
        print(f"Closest Object: {min_dist:.4f} meters", end='\r')

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()