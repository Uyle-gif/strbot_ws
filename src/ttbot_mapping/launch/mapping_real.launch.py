import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    pkg_ttbot_mapping = get_package_share_directory('ttbot_mapping')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ttbot_mapping, 'launch', 'pointcloud_to_laserscan.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    slam_params_path = PathJoinSubstitution([
        FindPackageShare('ttbot_mapping'),
        'config',
        'mapper_params.yaml' 
    ])

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_path,
            'use_sim_time': use_sim_time
        }.items()
    )

    delayed_slam = TimerAction(
        period=3.0,
        actions=[slam_toolbox_launch]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
            
        pointcloud_to_laserscan_launch,
        delayed_slam
    ])