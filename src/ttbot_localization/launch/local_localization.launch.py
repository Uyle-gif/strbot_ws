from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # <--- Thêm cái này
from launch.substitutions import LaunchConfiguration # <--- Thêm cái này
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    common_params = [{"use_sim_time": use_sim_time}]

    # static_transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["--x", "-0.20", "--y", "-0.03","--z", "1.50",
    #                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
    #                "--frame-id", "base_link",
    #                "--child-frame-id", "imu_link"],
    #     parameters=common_params 
    # )

    # static_tf_lidar = Node(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         name="static_transform_publisher_lidar",
    #         arguments=[
    #             "--x", "-0.30", "--y", "-0.03", "--z", "1.50", 
    #             "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
    #             "--frame-id", "base_link",
    #             "--child-frame-id", "lidar_link"
    #         ],
    #         parameters=common_params
    #     )

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
            
        # static_transform_publisher,
        # static_tf_lidar, 
        robot_localization,
        imu_republisher_cpp,   
    ])