import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
  
    try:
        pkg_share = get_package_share_directory('tortoisebot_description')
    except:
      
        pkg_share = os.path.join(os.path.expanduser('~'), 'tortoisebot_ws', 'src', 'tortoisebot_description')
    
  
    urdf_file = os.path.join(pkg_share, 'urdf', 'tortoisebot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_file]),
        value_type=str
    )
    
   
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'tortoisebot.rviz')
    
    return LaunchDescription([
      
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
      
        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='Start joint_state_publisher_gui if true'
        ),
        
      
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
        
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),
        
     
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
        ),
        
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])