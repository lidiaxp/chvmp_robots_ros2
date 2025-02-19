import os
import xacro
import launch_ros

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.event_handlers.on_execution_complete import OnExecutionComplete
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


ROBOT_DESCRIPTION = "spot_description"
ROBOT_BASE = "champ_base"
ROBOT_CONFIG = "spot_config"
ROBOT_URDF = "spot.urdf.xacro"
ENABLE_RVIZ = False


def generate_launch_description():
    this_package = FindPackageShare(ROBOT_CONFIG)
    joints_config = PathJoinSubstitution(
        [this_package, 'config', 'joints', 'joints.yaml']
    )
    gait_config = PathJoinSubstitution(
        [this_package, 'config', 'gait', 'gait.yaml']
    )
    links_config = PathJoinSubstitution(
        [this_package, 'config', 'links', 'links.yaml']
    )

    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package=ROBOT_DESCRIPTION
    ).find(ROBOT_DESCRIPTION)

    default_rviz_path = os.path.join(descr_pkg_share, "rviz/urdf_viewer.rviz")
    default_model_path = os.path.join(descr_pkg_share, "urdf/" + ROBOT_URDF)

    ######################## VARIABLES ########################

    declare_sim_time = DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )

    declare_description_path = DeclareLaunchArgument(
            name="description_path",
            default_value=default_model_path,
            description="Absolute path to robot urdf file",
        )

    declare_rviz_path = DeclareLaunchArgument(
            name="rviz_path",
            default_value=default_rviz_path,
            description="Absolute path to rviz file",
        )

    declare_rviz = DeclareLaunchArgument(
            "rviz", default_value="false", description="Launch rviz"
        )

    declare_rviz_ref_frame = DeclareLaunchArgument(
            "rviz_ref_frame", default_value="odom", description="Rviz ref frame"
        )

    declare_joints_map_path = DeclareLaunchArgument(
            name="joints_map_path",
            default_value=joints_config,
            description="Absolute path to joints map file",
        )

    declare_links_map_path = DeclareLaunchArgument(
            name="links_map_path",
            default_value=links_config,
            description="Absolute path to links map file",
        )

    declare_gait_config_path = DeclareLaunchArgument(
            name="gait_config_path",
            default_value=gait_config,
            description="Absolute path to gait config file",
        )

    declare_orientation_from_imu = DeclareLaunchArgument(
            "orientation_from_imu", default_value="false", description="Take orientation from IMU data"
        )

    declare_robot_name = DeclareLaunchArgument(
            "robot_name", default_value="/", description="Robot name"
        )

    declare_base_link_frame = DeclareLaunchArgument(
            "base_link_frame", default_value="base_link", description="Base link frame"
        )

    declare_lite = DeclareLaunchArgument(
            "lite", default_value="false", description="Lite"
        )

    declare_gazebo = DeclareLaunchArgument(
            "gazebo", default_value="false", description="If in gazebo"
        )

    declare_joint_controller_topic = DeclareLaunchArgument(
            "joint_controller_topic",
            default_value="joint_group_effort_controller/joint_trajectory",
            description="Joint controller topic",
        )

    declare_joint_hardware_connected = DeclareLaunchArgument(
            "joint_hardware_connected",
            default_value="false",
            description="Whether hardware is connected",
        )

    declare_publish_joint_control = DeclareLaunchArgument(
            "publish_joint_control",
            default_value="true",
            description="Publish joint control",
        )

    declare_publish_joint_states = DeclareLaunchArgument(
            "publish_joint_states",
            default_value="true",
            description="Publish joint states",
        )

    declare_publish_foot_contacts = DeclareLaunchArgument(
            "publish_foot_contacts",
            default_value="true",
            description="Publish foot contacts",
        )

    declare_publish_odom_tf = DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description="Publish odom tf from cmd_vel estimation",
        )

    declare_close_loop_odom = DeclareLaunchArgument(
            "close_loop_odom", default_value="false", description=""
        )

    ######################## LAUNCHS ########################

    include_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(ROBOT_DESCRIPTION),
                    "launch",
                    "description.launch.py",
                )
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "description_path": LaunchConfiguration("description_path"),
            }.items(),
        )

    ######################## NODES ########################

    node_robot_base_quadruped_controller_node = Node(
            package=ROBOT_BASE,
            executable="quadruped_controller_node",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"gazebo": LaunchConfiguration("gazebo")},
                {"publish_joint_states": LaunchConfiguration("publish_joint_states")},
                {"publish_joint_control": LaunchConfiguration("publish_joint_control")},
                {"publish_foot_contacts": LaunchConfiguration("publish_foot_contacts")},
                {"joint_controller_topic": LaunchConfiguration("joint_controller_topic")},
                {"urdf": Command(['xacro ', LaunchConfiguration('description_path')])},
                LaunchConfiguration('joints_map_path'),
                LaunchConfiguration('links_map_path'),
                LaunchConfiguration('gait_config_path'),
            ],
            remappings=[("/cmd_vel/smooth", "/cmd_vel")],
        )

    node_robot_base_state_estimation_node = Node(
            package=ROBOT_BASE,
            executable="state_estimation_node",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"orientation_from_imu": LaunchConfiguration("orientation_from_imu")},
                {"urdf": Command(['xacro ', LaunchConfiguration('description_path')])},
                LaunchConfiguration('joints_map_path'),
                LaunchConfiguration('links_map_path'),
                LaunchConfiguration('gait_config_path'),
            ],
        )

    node_robot_localization_base_to_footprint_ekf = Node(
            package="robot_localization",
            executable="ekf_node",
            name="base_to_footprint_ekf",
            output="screen",
            parameters=[
                {"base_link_frame": LaunchConfiguration("base_link_frame")},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                os.path.join(
                    get_package_share_directory(ROBOT_BASE),
                    "config",
                    "ekf",
                    "base_to_footprint.yaml",
                ),
            ],
            remappings=[("odometry/filtered", "odom/local")],
        )

    node_robot_localization_footprint_to_odom_ekf = Node(
            package="robot_localization",
            executable="ekf_node",
            name="footprint_to_odom_ekf",
            output="screen",
            parameters=[
                {"base_link_frame": LaunchConfiguration("base_link_frame")},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                os.path.join(
                    get_package_share_directory(ROBOT_BASE),
                    "config",
                    "ekf",
                    "footprint_to_odom.yaml",
                ),
            ],
            remappings=[("odometry/filtered", "odom")],
        )

    node_rviz2 = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration("rviz_path")],
            condition=IfCondition(LaunchConfiguration("rviz"))
        )

    rviz_dependencies = [declare_rviz_path, declare_rviz, declare_rviz_ref_frame, node_rviz2]

    _return = [
        declare_sim_time,
        declare_description_path,
        declare_joints_map_path,
        declare_links_map_path,
        declare_gait_config_path,
        declare_orientation_from_imu,
        declare_orientation_from_imu,    
        declare_base_link_frame,
        declare_lite,
        declare_gazebo,
        declare_joint_controller_topic,
        declare_joint_hardware_connected,
        declare_publish_joint_control,
        declare_publish_joint_states,
        declare_publish_foot_contacts,
        declare_publish_odom_tf,
        declare_close_loop_odom,
        include_description_launch,
        node_robot_base_quadruped_controller_node,
        node_robot_base_state_estimation_node,
        node_robot_localization_base_to_footprint_ekf,
        node_robot_localization_footprint_to_odom_ekf
    ]

    if ENABLE_RVIZ: _return += rviz_dependencies 

    return LaunchDescription(_return)
