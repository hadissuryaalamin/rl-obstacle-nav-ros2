import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'tutor' 
    pkg_share = get_package_share_directory(pkg_name)

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Flag to enable/disable Gazebo GUI'
    )
    
    # If gui=false, we run the "server only" (gzserver), otherwise "server + client" (gazebo)
    headless_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')]),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'arena.world')}.items()
    )

    gui_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'arena.world')}.items()
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'arena.world')}.items()
    )

    # 1. Robot Description
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # 2. State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml()}]
    )

    # 3. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'box_bot', '-z', '0.1'],
        output='screen'
    )

    # 4. Goal Manager (We still need this to move the Green Marker!)
    goal_manager = Node(
        package=pkg_name,
        executable='goal_manager.py',
        output='screen'
    )

    return LaunchDescription([
        gui_cmd, #headless_cmd if LaunchConfiguration('gui') == 'false' else gui_cmd,
        node_robot_state_publisher,
        spawn_entity,
        goal_manager
    ])