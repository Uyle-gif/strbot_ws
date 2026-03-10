import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    path_file_arg = DeclareLaunchArgument(
        "path_file", default_value="path_8.csv",
        description="Ten file CSV nam trong thu muc 'path'"
    )

    pkg_dir = get_package_share_directory('ttbot_controller')

    path_node = Node(
        package="ttbot_controller",
        executable="path_publisher", # ĐÃ SỬA THÀNH TÊN BẠN MUỐN
        name="path_publisher",
        output="screen",
        parameters=[{
            "file_path": [pkg_dir, '/path/', LaunchConfiguration("path_file")],
            "frame_id": "map"
        }]
    )

    return LaunchDescription([
        path_file_arg,
        path_node
    ])