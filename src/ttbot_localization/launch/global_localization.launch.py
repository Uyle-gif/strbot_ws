import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ttbot_localization')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    local_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'local_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')

    # Navsat Transform (GPS Lat/Lon -> Odom X/Y)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'navsat_transform_node:=info'],
        remappings=[
            ('imu', '/imu/data_filtered'),
            ('gps/fix', '/gps/fix'),                     # Topic GPS 
            ('gps/filtered', 'gps/filtered'),
            ('odometry/filtered', '/odometry/global'),    # Input feedback: Vị trí global hiện tại
            ('odometry/gps', '/odometry/gps'),              # Output
        ]
    )

    # Node Global EKF (Fuse Odom + IMU + GPS_Odom)
    #  publish TF map -> odom
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/global')    # Output
        ]
    )


    return LaunchDescription([
        use_sim_time_arg,
        local_localization_launch,
       # navsat_transform_node,
       # ekf_global_node
    ])