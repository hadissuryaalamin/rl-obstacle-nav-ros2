#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import Empty as EmptyMsg # Need this for the signal
import math

class RobotEnv(Node):
    def __init__(self):
        super().__init__('robot_env')
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Service to reset Gazebo (Robot Position)
        self.reset_client = self.create_client(EmptySrv, '/reset_simulation')
        
        # NEW: Publisher to tell GoalManager to randomize coordinates
        self.reset_goal_pub = self.create_publisher(EmptyMsg, '/reset_goal', 10)
        
        self.lidar_data = []
        self.start_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Env Node Started. Monitoring Crashes...")

    def scan_callback(self, msg):
        self.lidar_data = msg.ranges

    def reset_simulation(self):
        self.get_logger().warn("CRASH DETECTED! Resetting Robot & Goal...")
        
        # 1. Reset Robot Position
        req = EmptySrv.Request()
        self.reset_client.call_async(req)
        
        # 2. Randomize The Goal
        self.reset_goal_pub.publish(EmptyMsg())
        
        self.start_time = self.get_clock().now()

    def control_loop(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 2.0: return

        if len(self.lidar_data) > 0:
            front_cone = self.lidar_data[0:30] + self.lidar_data[-30:]
            valid_front = [x for x in front_cone if not math.isinf(x) and x > 0.05]
            
            if len(valid_front) > 0:
                if min(valid_front) < 0.35:
                    self.reset_simulation()

def main(args=None):
    rclpy.init(args=args)
    node = RobotEnv()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()