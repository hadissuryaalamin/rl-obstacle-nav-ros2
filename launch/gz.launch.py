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

    # 1. Process the URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # 2. Define the World File Path
    world_path = os.path.join(pkg_share, 'worlds', 'arena.world')

    # 3. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml()}]
    )

    # 4. Launch Gazebo with the Arena
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    # 5. Spawn the Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_box_bot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 6. Goal Manager 
    goal_manager = Node(
        package=pkg_name,
        executable='goal_manager.py',
        output='screen'
    )
    # 7. Robot Logic 
    robot_env = Node(
        package=pkg_name,
        executable='robot_env.py',
        output='screen'
    )
    # 8. PID Driver
    pid_driver = Node(
        package=pkg_name,
        executable='pid_driver.py',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        goal_manager,
        robot_env,
        pid_driver
    ])