import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'tutor' 
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Process the Robot URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # 2. Launch Gazebo (GUI enabled so you can see the wall)
    world_path = os.path.join(pkg_share, 'worlds', 'arena.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        # We load your arena world
        launch_arguments={'gz_args': f'-r {world_path}', 'on_exit_shutdown': 'true'}.items()
    )

    # 3. Robot State Publisher (Required for TF/Lidar to work)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml(),
                     'use_sim_time': True}]
    )

    # 4. Spawn the Robot
    # Note: We spawn at z=0.2 to prevent floor clipping
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', 
                   '-name', 'box_bot', 
                   '-x', '0.0', '-y', '0.0', '-z', '0.2'],
        output='screen'
    )

    # 5. ROS <-> Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen'
    )

    # NOTICE: No 'pid_driver', 'goal_manager', or 'robot_env' here.
    # The robot is purely passive.

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge
    ])
