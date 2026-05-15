import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap

def generate_launch_description():

    pkg_description = get_package_share_directory('ttbot_description')
    pkg_localization = get_package_share_directory('ttbot_localization')
    pkg_controller = get_package_share_directory('ttbot_controller')
    pkg_mapping = get_package_share_directory('ttbot_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    run_joy = LaunchConfiguration('run_joy')
    arg_run_joy = DeclareLaunchArgument('run_joy', default_value='true')

    controller_type = LaunchConfiguration('controller_type')
    arg_controller = DeclareLaunchArgument('controller_type', default_value='mpc')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    low_level_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'controller.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    pointcloud_to_laserscan_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_mapping, 'launch', 'pointcloud_to_laserscan.launch.py')
                )
            )
        ]
    )

    joy_launch_group = GroupAction(
        condition=IfCondition(run_joy),
        actions=[
            SetRemap(src='/cmd_vel', dst='/joy_cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'joystick_teleop.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    stamped_mux_node = Node(
        package='ttbot_controller',
        executable='stamped_twist_mux',
        name='stamped_twist_mux',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'joy_timeout': 0.5 
        }],
        remappings=[
            ('joy_cmd_vel', '/joy_cmd_vel'),
            ('mpc_cmd_vel', '/mpc_cmd_vel'),
            ('cmd_cmd_out', '/ackermann_controller/cmd_vel') 
        ]
    )

    localization_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_localization, 'launch', 'global_localization.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    mpc_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'mpc'"])),
        actions=[
            SetRemap(src='/cmd_vel', dst='/mpc_cmd_vel'),
            SetRemap(src='/ackermann_controller/cmd_vel', dst='/mpc_cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'mpc.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    stanley_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'stanley'"])),
        actions=[
            SetRemap(src='/cmd_vel', dst='/mpc_cmd_vel'),
            SetRemap(src='/ackermann_controller/cmd_vel', dst='/mpc_cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'stanley.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    gmpc_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'gmpc'"])),
        actions=[
            SetRemap(src='/cmd_vel', dst='/mpc_cmd_vel'),
            SetRemap(src='/ackermann_controller/cmd_vel', dst='/mpc_cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'gmpc.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    mpc_filter_node = TimerAction(
        period=6.0, 
        actions=[
            Node(
                package='ttbot_controller',
                executable='mpc_state_filter.py',
                name='mpc_state_filter',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_odom_topic': '/odometry/global',
                    'output_odom_topic': '/mpc_state',
                    'cmd_vel_topic': '/ackermann_controller/cmd_vel',
                    
                    'alpha_x': 1.0,    
                    'alpha_y': 1.0,
                    'alpha_yaw': 1.0,
                    'alpha_v': 1.0,    
                    'alpha_wz': 1.0,

                    'max_v_rate': 3.0,
                    'max_wz_rate': 4.0,

                    'v_standstill_threshold': 0.05,
                    'wz_standstill_threshold': 0.05
                }]
            )
        ]
    )

    high_level_control = TimerAction(
        period=8.0,
        actions=[stanley_group, mpc_group, gmpc_group]
    )

    return LaunchDescription([
        arg_sim_time,
        arg_run_joy,
        arg_controller,

        gazebo_launch,
        low_level_control_launch,
        pointcloud_to_laserscan_launch,
        
        joy_launch_group,
        stamped_mux_node, 
        
        localization_launch,
        mpc_filter_node,
        high_level_control
    ])