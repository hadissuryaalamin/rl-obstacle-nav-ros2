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
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        # We load your arena world
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'arena.world')}.items()
    )

    # 3. Robot State Publisher (Required for TF/Lidar to work)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml()}]
    )

    # 4. Spawn the Robot
    # Note: We spawn at z=0.2 to prevent floor clipping
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'box_bot', 
                   '-x', '0.0', '-y', '0.0', '-z', '0.2'],
        output='screen'
    )

    # NOTICE: No 'pid_driver', 'goal_manager', or 'robot_env' here.
    # The robot is purely passive.

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])