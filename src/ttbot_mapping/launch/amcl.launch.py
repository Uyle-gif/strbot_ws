import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_mapping = get_package_share_directory('ttbot_mapping')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(
        pkg_mapping, 'maps', 'my_map.yaml'))

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_fast_lio, 'launch', 'fast_lio_sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )


    included_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_mapping, 'launch', 'pointcloud_to_laserscan.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    delayed_pointcloud = TimerAction(
        period=5.0, 
        actions=[included_pointcloud_launch]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_path}, {'use_sim_time': use_sim_time}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'scan_topic': '/scan',
            'set_initial_pose': True, 
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.yaw': 0.0
        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]
    )

    delayed_localization = TimerAction(
        period=7.0,
        actions=[map_server_node, amcl_node, lifecycle_manager]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path),
        fast_lio_launch,
        delayed_pointcloud, 
        delayed_localization
    ])