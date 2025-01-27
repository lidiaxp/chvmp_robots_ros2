import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare('spot_config')

    default_params_file_path = PathJoinSubstitution(
        [this_package, 'config/autonomy', 'slam.yaml']
    )

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_navigation'), 'launch', 'slam.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='slam_params_file',
            default_value=default_params_file_path,
            description='Navigation2 slam params file'
        ),

        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'slam_params_file': LaunchConfiguration("slam_params_file"),
                'sim': LaunchConfiguration("sim"),
                'rviz': LaunchConfiguration("rviz")
            }.items()
        )
    ])
