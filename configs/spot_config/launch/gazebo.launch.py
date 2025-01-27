from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='/',
            description='Change this for namespacing.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Set to true to run RViz in parallel.'
        ),
        DeclareLaunchArgument(
            'lite',
            default_value='false',
            description='Set to true if you\'re using CHAMP lite version. Only useful for microcontrollers.'
        ),
        DeclareLaunchArgument(
            'ros_control_file',
            default_value=os.path.join(
                get_package_share_directory('spot_config'), 'config/ros_control/ros_control.yaml'
            ),
            description='Path to ROS Control configurations.'
        ),
        DeclareLaunchArgument(
            'gazebo_world',
            default_value=os.path.join(
                get_package_share_directory('spot_config'), 'worlds/outdoor.world'
            ),
            description='Path to Gazebo world you want to load.'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Enable GUI in Gazebo.'
        ),
        DeclareLaunchArgument(
            'world_init_x',
            default_value='0.0',
            description='X Initial position of the robot in Gazebo World.'
        ),
        DeclareLaunchArgument(
            'world_init_y',
            default_value='0.0',
            description='Y Initial position of the robot in Gazebo World.'
        ),
        DeclareLaunchArgument(
            'world_init_heading',
            default_value='0.0',
            description='Initial heading of the robot in Gazebo World.'
        ),

        # Include bringup.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('spot_config'), 'launch/bringup.launch.py'
                )
            ),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'gazebo': 'true',
                'lite': LaunchConfiguration('lite'),
                'rviz': LaunchConfiguration('rviz'),
                'joint_controller_topic': 'joint_group_position_controller/command',
                'hardware_connected': 'false',
                'publish_foot_contacts': 'false',
                'close_loop_odom': 'true'
            }.items()
        ),

        # Include gazebo.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('champ_gazebo'), 'launch/gazebo.launch.py'
                )
            ),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'lite': LaunchConfiguration('lite'),
                'ros_control_file': LaunchConfiguration('ros_control_file'),
                'gazebo_world': LaunchConfiguration('gazebo_world'),
                'world_init_x': LaunchConfiguration('world_init_x'),
                'world_init_y': LaunchConfiguration('world_init_y'),
                'world_init_heading': LaunchConfiguration('world_init_heading'),
                'gui': LaunchConfiguration('gui')
            }.items()
        )
    ])