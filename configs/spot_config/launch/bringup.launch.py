from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define package directories
    spot_config_dir = get_package_share_directory('spot_config')
    champ_bringup_dir = get_package_share_directory('champ_bringup')

    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='/', description='Change this for namespacing.'),
        DeclareLaunchArgument('base_frame', default_value='body', description='Link name of floating base. Do not touch this.'),
        DeclareLaunchArgument('joints_map_file', default_value=os.path.join(spot_config_dir, 'config/joints/joints.yaml'), description='Path to list of joint names. Do not touch this.'),
        DeclareLaunchArgument('links_map_file', default_value=os.path.join(spot_config_dir, 'config/links/links.yaml'), description='Path to list of link names. Do not touch this.'),
        DeclareLaunchArgument('gait_config_file', default_value=os.path.join(spot_config_dir, 'config/gait/gait.yaml'), description='Path to gait parameters. Do not touch this.'),
        DeclareLaunchArgument('description_file', default_value=os.path.join(get_package_share_directory('spot_description'), 'urdf/spot.urdf'), description='Path to URDF file. Do not touch this.'),
        DeclareLaunchArgument('gazebo', default_value='false', description='Set to true during simulation. This is auto-set to true from gazebo.launch.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Set to true to run RViz in parallel.'),
        DeclareLaunchArgument('rviz_ref_frame', default_value='odom', description='Default RVIZ reference frame.'),
        DeclareLaunchArgument('has_imu', default_value='true', description='Set to true if you want to visualize robot but there\'s no IMU. Only useful for microcontrollers.'),
        DeclareLaunchArgument('lite', default_value='false', description='Set to true if you\'re using CHAMP lite version. Only useful for microcontrollers.'),
        DeclareLaunchArgument('close_loop_odom', default_value='false', description='Set to true if you want to calculate odometry using close loop. This is auto-set to true from gazebo.launch.'),
        DeclareLaunchArgument('publish_foot_contacts', default_value='true', description='Set to true if you want the controller to publish the foot contact states. This is auto-set to false from gazebo.launch.'),
        DeclareLaunchArgument('publish_joint_control', default_value='true', description='Set to true if you want the controller to publish the joint_states topic. This is auto-set to false from gazebo.launch.'),
        DeclareLaunchArgument('laser', default_value='sim', description='Set to the 2D LIDAR you\'re using.'),
        DeclareLaunchArgument('joint_controller_topic', default_value='joint_group_position_controller/command', description='Change to remap command topic for actuator controller (ROS control).'),
        DeclareLaunchArgument('hardware_connected', default_value='false', description='Flag useful to launch hardware connected launch files. This auto disables publishing joint_states.'),
        DeclareLaunchArgument('frame_prefix', default_value='', description='Prefix for frames.'),

        # Include bringup.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(champ_bringup_dir, 'launch/bringup.launch.py')),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'base_frame': LaunchConfiguration('base_frame'),
                'joints_map_file': LaunchConfiguration('joints_map_file'),
                'links_map_file': LaunchConfiguration('links_map_file'),
                'gait_config_file': LaunchConfiguration('gait_config_file'),
                'description_file': LaunchConfiguration('description_file'),
                'has_imu': LaunchConfiguration('has_imu'),
                'gazebo': LaunchConfiguration('gazebo'),
                'lite': LaunchConfiguration('lite'),
                'laser': LaunchConfiguration('laser'),
                'rviz': LaunchConfiguration('rviz'),
                'rviz_ref_frame': LaunchConfiguration('rviz_ref_frame'),
                'joint_controller_topic': LaunchConfiguration('joint_controller_topic'),
                'hardware_connected': LaunchConfiguration('hardware_connected'),
                'publish_foot_contacts': LaunchConfiguration('publish_foot_contacts'),
                'publish_joint_control': LaunchConfiguration('publish_joint_control'),
                'close_loop_odom': LaunchConfiguration('close_loop_odom')
            }.items()
        )
    ])
