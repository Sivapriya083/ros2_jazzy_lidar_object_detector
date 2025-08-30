import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package directories
    pkg_description = get_package_share_directory('tortoisebot_description')
    pkg_gazebo = get_package_share_directory('tortoisebot_gazebo')
    
    # URDF file path
    urdf_file_name = 'tortoisebot.urdf.xacro'
    urdf_path = os.path.join(pkg_description, 'urdf', urdf_file_name)
    
    # World file path
    world_file_name = 'empty_world.world'
    world_path = os.path.join(pkg_gazebo, 'worlds', world_file_name)
    
    # Process the URDF file
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='Full path to world model file to load'),
        
        # Start Gazebo with empty world
        ExecuteProcess(
            cmd=['gz', 'sim', LaunchConfiguration('world'), '-v', '4'],
            output='screen',
            name='gazebo'
        ),

        # Robot State Publisher
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'robot_description': robot_description
                    }],
                ),
            ]
        ),
        
        # Spawn robot in Gazebo
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_tortoisebot',
                    output='screen',
                    arguments=[
                        '-topic', '/robot_description',
                        '-name', 'tortoisebot',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.3'
                    ],
                ),
            ]
        ),

        # ROS-Gazebo Bridges
        TimerAction(
            period=6.0,
            actions=[
                # Command velocity bridge
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='cmd_vel_bridge',
                    arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
                    output='screen'
                ),
                # Odometry bridge
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='odom_bridge',
                    arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
                    output='screen'
                ),
                # Joint states bridge
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='joint_states_bridge',
                    arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
                    output='screen'
                ),
                # Lidar bridge
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='lidar_bridge',
                    arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
                    output='screen'
                ),
            ]
        ),

        # Static transform publisher for base_footprint to odom
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='odom_to_base_footprint',
                    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
                    output='screen'
                ),
            ]
        ),
    ])