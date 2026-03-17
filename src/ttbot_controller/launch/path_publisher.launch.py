import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    path_file_arg = DeclareLaunchArgument(
        "path_file", default_value="circle_r5.csv",
        description="Name of the CSV file inside 'path' folder"
    )

    pkg_dir = get_package_share_directory('ttbot_controller')

    path_node = Node(
        package="ttbot_controller",
        executable="path_publisher",
        name="path_publisher",
        output="screen",
        parameters=[{
            # Nối đường dẫn tuyệt đối một cách an toàn
            "file_path": PathJoinSubstitution([pkg_dir, 'path', LaunchConfiguration("path_file")]),
            "frame_id": "odom"
        }]
    )

    return LaunchDescription([
        path_file_arg,
        path_node
    ])