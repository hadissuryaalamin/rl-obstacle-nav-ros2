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
        parameters=[{'robot_description': robot_description_config.toxml(),
                     'use_sim_time': True}]
    )

    # 4. Launch Gazebo with the Arena
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}', 'on_exit_shutdown': 'true'}.items()
    )

    # 5. Spawn the Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'box_bot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 6. ROS <-> Gazebo Bridge (topics + entity services)
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

    # 7. Goal Manager 
    goal_manager = Node(
        package=pkg_name,
        executable='goal_manager.py',
        output='screen'
    )
    # 8. Robot Logic 
    robot_env = Node(
        package=pkg_name,
        executable='robot_env.py',
        output='screen'
    )
    # 9. PID Driver
    pid_driver = Node(
        package=pkg_name,
        executable='pid_driver.py',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        goal_manager,
        robot_env,
        pid_driver
    ])
