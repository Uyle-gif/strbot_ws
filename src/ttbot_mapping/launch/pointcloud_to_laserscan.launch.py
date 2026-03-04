from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                # QUAN TRỌNG: Lấy dữ liệu đã khử nhiễu từ FAST-LIO
                ('cloud_in', '/cloud_registered'), 
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_link', 
                'transform_tolerance': 0.02,
                'use_sim_time': True, # Bắt buộc vì bạn đang chạy mô phỏng
                
                # Mở rộng dải quét để lấy đủ thân tường
                'min_height': -0.25,
                'max_height': 1.0,  
                
                'angle_min': -3.1415, 
                'angle_max': 3.1415,
                'angle_increment': 0.0043, # Tăng độ phân giải điểm quét
                'scan_time': 0.1,
                'range_min': 0.5, 
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
        )
    ])