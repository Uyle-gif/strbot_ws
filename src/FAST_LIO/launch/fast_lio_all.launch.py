import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. Tham số hệ thống ---
    # Luôn mặc định là True khi chạy trong môi trường mô phỏng Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- 2. Định nghĩa đường dẫn linh hoạt (Portable Paths) ---
    # ROS sẽ tự tìm vị trí của package trên mọi máy tính khác nhau
    pkg_fast_lio = FindPackageShare('fast_lio')
    
    # Đường dẫn đến file cấu hình mô phỏng
    fast_lio_config_path = PathJoinSubstitution([
        pkg_fast_lio,
        'config',
        'velodyne_sim.yaml'
    ])

    # Đường dẫn đến file cấu hình RViz (tùy chọn)
    default_rviz_config_path = PathJoinSubstitution([
        pkg_fast_lio,
        'rviz',
        'fastlio_sim.rviz'
    ])

    # --- 3. Khai báo các Node ---

    # Node chính FAST-LIO Mapping
    # Đã sửa code C++ để chạy trực tiếp trên frame 'odom' và 'base_link'
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            fast_lio_config_path,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Node RViz2 để quan sát kết quả
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 4. Tổng hợp Launch Description ---
    ld = LaunchDescription()

    # Khai báo tham số để có thể thay đổi từ dòng lệnh nếu cần
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))

    # Thêm các Node vào hệ thống
    # Lưu ý: Không chạy driver Xsens hay Velodyne thật vì Gazebo đã cung cấp dữ liệu
    ld.add_action(fast_lio_node)

    # ld.add_action(rviz_node)

    return ld