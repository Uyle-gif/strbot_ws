from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # <--- Thêm cái này
from launch.substitutions import LaunchConfiguration # <--- Thêm cái này
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    common_params = [{"use_sim_time": use_sim_time}]

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "-0.2","--z", "0.13",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_link",
                   "--child-frame-id", "imu_link"],
        parameters=common_params 
    )
    static_tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_lidar",
        arguments=[
            "0.0", "0.0", "0.2",  # x, y, z (Ví dụ: Lidar cao hơn base_link 20cm)
            "0.0", "0.0", "0.0",  # roll, pitch, yaw (Dạng radian hoặc dùng qx qy qz qw)
            "base_link",          # Frame cha
            "lidar_link"          # Frame con (Phải khớp với frame_id trong FAST_LIO config)
        ],
        parameters=common_params
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("ttbot_localization"), "config", "ekf.yaml"),
            common_params[0] 
        ],
    )

    imu_republisher_cpp = Node(
        package="ttbot_localization",
        executable="imu_republisher",
        parameters=common_params
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        static_transform_publisher,
        static_tf_lidar, # Thêm node mới vào đây
        robot_localization,
        imu_republisher_cpp,   
    ])