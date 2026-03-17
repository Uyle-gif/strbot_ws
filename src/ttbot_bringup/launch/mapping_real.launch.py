import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    localization_dir = get_package_share_directory('ttbot_localization')
    fast_livo_dir = get_package_share_directory('fast_livo')
    mapping_dir = get_package_share_directory('ttbot_mapping')
    fast_lio_dir = get_package_share_directory('fast_lio')

    description_dir = get_package_share_directory('ttbot_description')
    urdf_file = os.path.join(description_dir, 'urdf', 'ttbot.urdf.xacro')    
    robot_description = Command(['xacro ', urdf_file])

    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{'use_sim_time': use_sim_time}]
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(localization_dir, 'launch', 'global_localization.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )



    fast_livo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_livo_dir, 'launch', 'fast_livo_real.launch.py'))
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_lio_dir, 'launch', 'fast_lio_real.launch.py'))
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mapping_dir, 'launch', 'mapping_real.launch.py'))
    )

    delay_fast_livo = TimerAction(
        period=3.0,
        actions=[fast_livo_launch]
    )

    delay_fast_lio = TimerAction(
        period=3.0,
        actions=[fast_lio_launch]
    )

    delay_mapping = TimerAction(
        period=6.0,
        actions=[mapping_launch]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,  
        joint_state_publisher_node,
        localization_launch,
        # delay_fast_livo,
        delay_fast_lio,
        # delay_mapping                
    ])