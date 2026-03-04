import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ttbot_mapping = get_package_share_directory('ttbot_mapping')
    
    # Khai báo đường dẫn file map
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(
        pkg_ttbot_mapping, 'maps', 'my_map.yaml'))

    # 1. Node Map Server (Chạy ở trạng thái Unconfigured)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file},
                    {'use_sim_time': True}]
    )

    # 2. Bộ quản lý vòng đời có DELAY (Đợi 2 giây mới kích hoạt)
    # Điều này giúp tránh lỗi 'bad file' do tranh chấp tài nguyên lúc khởi động
    delayed_lifecycle_manager = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server']
                }]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map yaml file to load'),
        map_server_node,
        delayed_lifecycle_manager
    ])