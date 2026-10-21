#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z 

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Odometry, '/odom', self.update_pose, qos)
        
        self.goal_x, self.goal_y = 5.0, 3.0
        self.pose = None
        self.get_logger().info("Controller Started. Waiting for Odom...")

    def update_pose(self, msg):
        if self.pose is None:
            self.get_logger().info("Odom Received! Starting control loop.")
            
        self.pose = msg.pose.pose
        self.control_loop()

    def control_loop(self):
        if self.pose is None: return
        
        dx = self.goal_x - self.pose.position.x
        dy = self.goal_y - self.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        q = self.pose.orientation
        (roll, pitch, current_theta) = euler_from_quaternion(q)
        
        goal_theta = math.atan2(dy, dx)
        angle_diff = goal_theta - current_theta

        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        cmd = Twist()
        
        if distance > 0.1:
            cmd.linear.x = 0.5 * distance
            cmd.angular.z = 1.5 * angle_diff
            
            if cmd.linear.x > 0.5: cmd.linear.x = 0.5
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Goal Reached!")
        
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()