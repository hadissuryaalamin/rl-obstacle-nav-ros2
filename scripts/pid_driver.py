#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import math

# Standard Euler conversion function
def euler_from_quaternion(q):
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z # We only need Yaw

class PidDriver(Node):
    def __init__(self):
        super().__init__('pid_driver')
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(Point, '/goal_coordinates', self.goal_callback, 10)
        # Ask GoalManager for a new goal once the current one is reached
        self.reset_goal_pub = self.create_publisher(Empty, '/reset_goal', 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.reached_goal = None  # (x, y) of the goal we already reported as reached
        
        self.get_logger().info("PID Driver Started. Waiting for Goal...")

    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        if self.reached_goal is not None and self.reached_goal != (msg.x, msg.y):
            self.reached_goal = None

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation)
        
        if self.goal_x is not None:
            self.control_loop()

    def control_loop(self):
        # 1. Calculate Error
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.yaw
        
        # Normalize angle error to -pi to +pi
        while angle_error > math.pi: angle_error -= 2 * math.pi
        while angle_error < -math.pi: angle_error += 2 * math.pi

        # 2. PID Control
        cmd = Twist()
        
        if distance > 0.1:
            # Linear speed scales with distance (max 0.5)
            cmd.linear.x = 0.5 * distance
            if cmd.linear.x > 0.4: cmd.linear.x = 0.4
            
            # Angular speed scales with angle error
            cmd.angular.z = 1.5 * angle_error
        else:
            # Stop if close
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.reached_goal is None:
                self.get_logger().info("GOAL REACHED! Requesting new goal...")
                self.reached_goal = (self.goal_x, self.goal_y)
                self.reset_goal_pub.publish(Empty())
            
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PidDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()