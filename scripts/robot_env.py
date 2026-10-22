#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Empty as EmptyMsg # Need this for the signal
import math

class RobotEnv(Node):
    def __init__(self):
        super().__init__('robot_env')
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Manual reset: ros2 topic pub --once /reset_robot std_msgs/msg/Empty
        self.create_subscription(EmptyMsg, '/reset_robot', lambda _: self.reset_simulation("MANUAL RESET"), 10)
        
        # Service to teleport the robot back to the start (Robot Position)
        self.reset_client = self.create_client(SetEntityPose, '/world/arena/set_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # NEW: Publisher to tell GoalManager to randomize coordinates
        self.reset_goal_pub = self.create_publisher(EmptyMsg, '/reset_goal', 10)
        
        self.lidar_data = []
        self.start_time = self.get_clock().now()
        
        # Stuck detection: driving forward but not moving (e.g. side/rear collision)
        self.position = None
        self.commanded_speed = 0.0
        self.stuck_anchor = None
        self.stuck_since = self.get_clock().now()
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Env Node Started. Monitoring Crashes...")

    def scan_callback(self, msg):
        self.lidar_data = msg.ranges

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def cmd_callback(self, msg):
        self.commanded_speed = abs(msg.linear.x)

    def reset_simulation(self, reason="CRASH DETECTED!"):
        self.get_logger().warn(f"{reason} Resetting Robot & Goal...")
        
        # 1. Reset Robot Position
        self.cmd_pub.publish(Twist())
        req = SetEntityPose.Request()
        req.entity.name = 'box_bot'
        req.pose.position.z = 0.1
        req.pose.orientation.w = 1.0
        self.reset_client.call_async(req).add_done_callback(self.reset_done)
        
        # 2. Randomize The Goal
        self.reset_goal_pub.publish(EmptyMsg())
        
        self.start_time = self.get_clock().now()
        self.stuck_anchor = None

    def reset_done(self, future):
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error("set_pose failed - is /world/arena/set_pose bridged?")

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < 2.0: return

        if len(self.lidar_data) > 0:
            front_cone = self.lidar_data[0:30] + self.lidar_data[-30:]
            # -inf means closer than the lidar's min range, i.e. already touching
            valid_front = [0.0 if x == -math.inf else x for x in front_cone
                           if not math.isnan(x) and x != math.inf]
            
            if len(valid_front) > 0:
                if min(valid_front) < 0.35:
                    self.reset_simulation()
                    return

        if self.position is None: return
        if self.commanded_speed < 0.05 or self.stuck_anchor is None or \
                math.dist(self.position, self.stuck_anchor) > 0.05:
            self.stuck_anchor = self.position
            self.stuck_since = now
        elif (now - self.stuck_since).nanoseconds / 1e9 > 3.0:
            self.reset_simulation("STUCK!")

def main(args=None):
    rclpy.init(args=args)
    node = RobotEnv()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()