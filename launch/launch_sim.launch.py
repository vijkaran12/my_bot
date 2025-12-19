import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_bot'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    
    # Default world
    default_world = os.path.join(pkg_path, 'worlds', 'test_world.sdf')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to SDF world file'
    )

    # Process URDF
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r']
        }.items()
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot'],
        output='screen'
    )

    # ROS-Gazebo Bridge with QoS fixes
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            'cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            'odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            'model/my_bot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        remappings=[
            ('model/my_bot/scan', 'scan'),
        ],
        parameters=[{
            'qos_overrides./scan.history': 'keep_last',
            'qos_overrides./scan.depth': 5,
            'qos_overrides./scan.reliability': 'best_effort',
            'qos_overrides./cmd_vel.history': 'keep_last',
            'qos_overrides./cmd_vel.depth': 5,
            'qos_overrides./cmd_vel.reliability': 'reliable',
            'qos_overrides./odom.history': 'keep_last',
            'qos_overrides./odom.depth': 5,
            'qos_overrides./odom.reliability': 'reliable',
        }]
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])
