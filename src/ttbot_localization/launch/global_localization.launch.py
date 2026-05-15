import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ttbot_localization')
    mapping_pkg_share = get_package_share_directory('ttbot_mapping')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(mapping_pkg_share, 'maps', 'bk_map.yaml'),
        description='Full path to map yaml file for ICP localization'
    )
    map_yaml = LaunchConfiguration('map')

    local_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'local_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    initial_icp_localizer_node = Node(
        package='ttbot_localization',
        executable='initial_icp_localizer',
        name='initial_icp_localizer',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'icp_localizer.yaml'),
            {
                'map_yaml': map_yaml,
                'use_sim_time': use_sim_time
            }
        ]
    )

    fastlio_odom_aligner_node = Node(
        package='ttbot_localization',
        executable='fastlio_odom_aligner',
        name='fastlio_odom_aligner',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'fastlio_odom_aligner.yaml'),
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global_livo_lio.yaml')

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/global')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        local_localization_launch,
        # initial_icp_localizer_node,
        # fastlio_odom_aligner_node,
        ekf_global_node
    ])