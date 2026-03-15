import os.path
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    fast_lio_pkg = get_package_share_directory('fast_lio')
    xsens_pkg = get_package_share_directory('xsens_mti_ros2_driver')
    velodyne_pkg = get_package_share_directory('velodyne')

    default_config_path = os.path.join(fast_lio_pkg, 'config')
    default_rviz_config_path = os.path.join(fast_lio_pkg, 'rviz', 'fast_lio_real.rviz')
    xsens_params_path = Path(xsens_pkg, 'param', 'xsens_mti_node.yaml')


    realsense_pkg = get_package_share_directory('realsense2_camera')

    # realsense_launch = GroupAction(
    #     actions=[
    #         SetRemap(src='/camera/camera/imu', dst='/imu/camera'), 
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(realsense_pkg, 'launch', 'rs_launch.py')]),
    #             launch_arguments={
    #                 'pointcloud.enable': 'false', 
    #                 'enable_gyro': 'true',       
    #                 'enable_accel': 'true',      
    #                 'unite_imu_method': '2',     
    #             }.items()
    #         )
    #     ]
    # )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(realsense_pkg, 'launch', 'rs_launch.py')]),
        launch_arguments={
            'pointcloud.enable': 'false', 
            'enable_gyro': 'false',      # Tắt hoàn toàn Gyro của camera
            'enable_accel': 'false',     # Tắt hoàn toàn Accel của camera
        }.items()
    )


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    xsens_mti_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[xsens_params_path, {'use_sim_time': use_sim_time}]
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(velodyne_pkg, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
        ])
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            os.path.join(default_config_path, 'velodyne_real.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path],  
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    # ld.add_action(realsense_launch)
    ld.add_action(xsens_mti_node)
    ld.add_action(velodyne_launch)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld