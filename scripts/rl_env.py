#!/usr/bin/env python3
import gymnasium as gym
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math
import time

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Empty as EmptyMsg

class TutorBotEnv(gym.Env, Node):
    def __init__(self):
        Node.__init__(self, 'tutor_bot_env')
        
        # --- ACTION SPACE ---
        self.action_space = gym.spaces.Box(
            low=np.array([0.0, -1.0]), high=np.array([0.5, 1.0]), dtype=np.float32)

        # --- OBSERVATION SPACE ---
        self.n_lidar_samples = 10
        self.observation_space = gym.spaces.Box(
            low=0.0, high=3.5, shape=(self.n_lidar_samples + 2,), dtype=np.float32)

        # --- ROS COMM ---
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reset_goal = self.create_publisher(EmptyMsg, '/reset_goal', 10)
        
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.sub_goal = self.create_subscription(Point, '/goal_coordinates', self.goal_callback, 10)
        self.reset_client = self.create_client(SetEntityPose, '/world/arena/set_pose')

        # --- STATE VARIABLES ---
        self.lidar_data = np.full(self.n_lidar_samples, 3.5)
        self.min_front_dist = 3.5
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.yaw = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.current_distance = 0.0
        
        self.data_received = False
        self.goal_received = False  
        
        self.steps_in_episode = 0
        self.max_steps_per_episode = 800
        self.heading_error = 0.0

    def step(self, action):
        self.steps_in_episode += 1
        
        # 1. Action
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        self.pub_cmd_vel.publish(cmd)

        # 2. Wait for Physics
        rclpy.spin_once(self, timeout_sec=0.1)

        # 3. Save Old Distance for Calculation
        prev_distance = self.current_distance 

        # 4. Get New State
        state = self._get_observation() # This updates self.current_distance and self.heading_error
        
        reward = 0.0
        terminated = False
        truncated = False
        info = {}

        if not self.data_received or not self.goal_received:
            return state, 0.0, False, False, info

        # --- REWARD LOGIC ---
        
        # A. CRASH CHECK
        if self.min_front_dist < 0.17:
            print(f"  [X] CRASH! Dist: {self.min_front_dist:.4f}m") 
            reward = -100.0
            terminated = True
            self.pub_cmd_vel.publish(Twist()) 
        
        # B. GOAL CHECK
        elif self.current_distance < 0.20:
            print(f"  [!] GOAL REACHED! Dist: {self.current_distance:.2f}m")
            reward = +100.0
            terminated = True
            self.pub_cmd_vel.publish(Twist()) 
            
        # C. TIMEOUT
        elif self.steps_in_episode >= self.max_steps_per_episode:
            print("  [?] TIMEOUT")
            truncated = True 
            
        # D. SHAPING REWARD 
        else:
            # 1. Distance Improvement (Getting Closer = +Points)
            dist_improvement = prev_distance - self.current_distance
            reward += dist_improvement * 10.0 
            
            # 2. Heading Penalty (Looking Away = -Points)
            heading_penalty = abs(self.heading_error)
            reward -= heading_penalty * 0.5 
            
            # 3. Tiny Existence Penalty
            reward -= 0.01

        return state, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps_in_episode = 0
        
        # 1. Reset Robot (teleport back to the start pose)
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for reset service...')
        self.pub_cmd_vel.publish(Twist())
        req = SetEntityPose.Request()
        req.entity.name = 'box_bot'
        req.pose.position.z = 0.1
        req.pose.orientation.w = 1.0
        self.reset_client.call_async(req)
        
        # 2. Request New Goal
        self.goal_received = False # Reset flag
        self.pub_reset_goal.publish(EmptyMsg())
        
        # 3. Reset Data
        self.data_received = False 
        self.lidar_data = np.full(self.n_lidar_samples, 3.5)
        self.min_front_dist = 3.5

        # 4. Wait for Robot to Settle AND New Goal to Arrive
        wait_cycles = 0
        while (not self.data_received or not self.goal_received) and wait_cycles < 20:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_cycles += 1
            time.sleep(0.05)
            
        # 5. PRINT NEW GOAL (Verification)
        print(f"\n[RESET] New Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) | Robot Start: ({self.robot_x:.2f}, {self.robot_y:.2f})")

        return self._get_observation(), {}

    def _get_observation(self):
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        self.current_distance = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)
        
        # Calculate Error
        heading_err = goal_angle - self.yaw
        
        # Normalize to -pi to +pi
        while heading_err > math.pi: heading_err -= 2 * math.pi
        while heading_err < -math.pi: heading_err += 2 * math.pi
        
        # STORE IT FOR REWARD CALCULATION
        self.heading_error = heading_err 

        obs = np.concatenate((self.lidar_data, [self.current_distance, heading_err]))
        return obs.astype(np.float32)

    def scan_callback(self, msg):
        raw_data = np.array(msg.ranges)
        raw_data = np.nan_to_num(raw_data, posinf=10.0, neginf=10.0)
        
        # Filter Self (Blind Spot < 0.18m)
        raw_data[raw_data < 0.18] = 3.5
        
        # Crash Logic (Front Cone)
        front_cone = np.concatenate((raw_data[0:30], raw_data[-30:]))
        self.min_front_dist = np.min(front_cone)
        
        # AI Vision
        view_data = np.clip(raw_data, 0.0, 3.5)
        step = len(view_data) // self.n_lidar_samples
        self.lidar_data = view_data[::step][:self.n_lidar_samples]
        
        self.data_received = True

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_received = True 