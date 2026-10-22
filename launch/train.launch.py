import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'tutor' 
    pkg_share = get_package_share_directory(pkg_name)
    world_path = os.path.join(pkg_share, 'worlds', 'arena.world')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Flag to enable/disable Gazebo GUI'
    )
    
    # If gui=false, we run the server only (headless rendering still feeds the gpu_lidar)
    headless_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_launch]),
        launch_arguments={'gz_args': f'-r -s --headless-rendering {world_path}',
                          'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    gui_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_launch]),
        launch_arguments={'gz_args': f'-r {world_path}', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # 1. Robot Description
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # 2. State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config.toxml(),
                     'use_sim_time': True}]
    )

    # 3. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'box_bot', '-z', '0.1'],
        output='screen'
    )

    # 4. ROS <-> Gazebo Bridge (topics + entity services)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                   '/world/arena/create@ros_gz_interfaces/srv/SpawnEntity',
                   '/world/arena/remove@ros_gz_interfaces/srv/DeleteEntity',
                   '/world/arena/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        output='screen'
    )

    # 5. Goal Manager (We still need this to move the Green Marker!)
    goal_manager = Node(
        package=pkg_name,
        executable='goal_manager.py',
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        gui_cmd,
        headless_cmd,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        goal_manager
    ])
