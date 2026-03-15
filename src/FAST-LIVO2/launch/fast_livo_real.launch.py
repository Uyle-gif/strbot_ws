import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node,SetRemap

def generate_launch_description():
    fast_livo_pkg = get_package_share_directory('fast_livo')
    realsense_pkg = get_package_share_directory('realsense2_camera')
    velodyne_pkg = get_package_share_directory('velodyne')
    xsens_pkg = get_package_share_directory('xsens_mti_ros2_driver')

    config_file_dir = os.path.join(fast_livo_pkg, "config")
    ttbot_config_real = os.path.join(config_file_dir, "ttbot_real.yaml") 
    camera_config_real = os.path.join(config_file_dir, "camera_real.yaml") 
    rviz_config_file = os.path.join(fast_livo_pkg, "rviz_cfg", "fast_livo2.rviz")

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(velodyne_pkg, 'launch', 'velodyne-all-nodes-VLP16-launch.py')])
    )

    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(realsense_pkg, 'launch', 'rs_launch.py')]),
    #     launch_arguments={
    #         'pointcloud.enable': 'false', 
    #         'enable_gyro': 'false',
    #         'enable_accel': 'false',
    #     }.items()
    # )
    # =========================================================================
    # CẤU HÌNH REALSENSE IMU VÀ REMAP TOPIC CHUẨN XÁC
    # =========================================================================
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(realsense_pkg, 'launch', 'rs_launch.py')]),
        launch_arguments={
            'pointcloud.enable': 'false', 
            'enable_gyro': 'false',      # Tắt hoàn toàn Gyro của camera
            'enable_accel': 'false',     # Tắt hoàn toàn Accel của camera
        }.items()
    )

    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(xsens_pkg, 'launch', 'xsens_mti_node.launch.py')])
    )

    fastlivo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="laserMapping",
        parameters=[
            ttbot_config_real,
            camera_config_real,
            {'use_sim_time': False} 
        ],
        output="screen"
    )

    # --- RVIZ2 ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output="screen"
    )
    param_blackboard_node = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='parameter_blackboard',
        parameters=[camera_config_real],
        output='screen'
    )

    return LaunchDescription([
        velodyne_launch,
        realsense_launch,
        xsens_launch,
        param_blackboard_node, 
        fastlivo_node,
        #rviz_node
    ])