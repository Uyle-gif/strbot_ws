#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2_sim.rviz")

    ttbot_config_cmd = os.path.join(config_file_dir, "ttbot_sim.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_sim.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False", 
        description="Whether to launch Rviz2",
    )

    ttbot_config_arg = DeclareLaunchArgument(
        'ttbot_params_file',
        default_value=ttbot_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for camera intrinsics',
    )

    # https://github.com/ros-navigation/navigation2/blob/1c68c212db01f9f75fcb8263a0fbb5dfa711bdea/nav2_bringup/launch/navigation_launch.py#L40
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    ttbot_params_file = LaunchConfiguration('ttbot_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_respawn = LaunchConfiguration('use_respawn')

    return LaunchDescription([
        use_rviz_arg,
        ttbot_config_arg,
        camera_config_arg,
        use_respawn_arg,

        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                ttbot_params_file,
                camera_params_file,
                {'use_sim_time': True} 
            ],
            prefix=[
                # ("gdb -ex run --args"),
                # ("valgrind --log-file=./valgrind_report.log --tool=memcheck --leak-check=full --show-leak-kinds=all -s --track-origins=yes --show-reachable=yes --undef-value-errors=yes --track-fds=yes")
            ],
            output="screen"
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[
                {'use_sim_time': True}
            ],
            output="screen"
        ),
    ])