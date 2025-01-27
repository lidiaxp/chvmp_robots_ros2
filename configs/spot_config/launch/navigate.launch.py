import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare('spot_config')

    default_map_path = PathJoinSubstitution(
        [this_package, 'maps', 'map.yaml']
    )

    default_params_file_path = PathJoinSubstitution(
        [this_package, 'config/autonomy', 'navigation.yaml']
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_navigation'), 'launch', 'navigate.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),
        
        DeclareLaunchArgument(
            name='params_file', 
            default_value=default_params_file_path,
            description='Navigation2 params file'
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
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'params_file': LaunchConfiguration("params_file"),
                'sim': LaunchConfiguration("sim"),
                'rviz': LaunchConfiguration("rviz")
            }.items()
        )
    ])