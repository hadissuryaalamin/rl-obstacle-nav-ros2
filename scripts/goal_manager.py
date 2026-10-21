#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from gazebo_msgs.srv import SpawnEntity, DeleteEntity # Need DeleteEntity
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.goal_pub = self.create_publisher(Point, '/goal_coordinates', 10)
        
        # Subscribe to reset signal
        self.create_subscription(Empty, '/reset_goal', self.reset_callback, 10)
        
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.entity_name = "goal_target"
        
        self.timer = self.create_timer(0.5, self.publish_goal)
        
        # Initial Spawn
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        self.spawn_goal()

    def reset_callback(self, msg):
        self.get_logger().info("Reset Signal Received - Moving Goal")
        # 1. Delete old goal
        del_req = DeleteEntity.Request()
        del_req.name = self.entity_name
        self.delete_client.call_async(del_req)
        
        # 2. Spawn new goal (Wait slightly for delete to process implies simple async call sequence)
        # In simple ROS2 scripts, calling spawn immediately after usually works if we randomize logic
        self.spawn_goal()

    def spawn_goal(self):
        self.goal_x = random.uniform(-2.75, 2.75)
        self.goal_y = random.uniform(-2.75, 2.75)
        
        self.get_logger().info(f'New Goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}')

        goal_xml = """
        <?xml version="1.0" ?>
        <sdf version="1.6">
            <model name="goal_target">
                <static>true</static>
                <link name="link">
                    <visual name="visual">
                        <geometry>
                            <cylinder><radius>0.2</radius><length>0.01</length></cylinder>
                        </geometry>
                        <material><script><name>Gazebo/Green</name></script></material>
                    </visual>
                </link>
            </model>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = goal_xml
        req.initial_pose.position.x = self.goal_x
        req.initial_pose.position.y = self.goal_y
        
        self.spawn_client.call_async(req)

    def publish_goal(self):
        msg = Point()
        msg.x = self.goal_x
        msg.y = self.goal_y
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()