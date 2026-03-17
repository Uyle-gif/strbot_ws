import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_description = get_package_share_directory('ttbot_description')
    pkg_localization = get_package_share_directory('ttbot_localization')
    pkg_controller = get_package_share_directory('ttbot_controller')
    xsens_pkg = get_package_share_directory('xsens_mti_ros2_driver')

    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false') 

    run_joy = LaunchConfiguration('run_joy')
    arg_run_joy = DeclareLaunchArgument('run_joy', default_value='false')

    controller_type = LaunchConfiguration('controller_type')
    arg_controller = DeclareLaunchArgument('controller_type', default_value='mpc')

    xacro_file = os.path.join(pkg_description, 'urdf', 'ttbot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' is_ignition:=false']), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )



    ackermann_node = Node(
        package='ttbot_controller',
        executable='ackermann_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/ackermann_controller/cmd_vel') 
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

    xsens_launch = GroupAction([
        SetRemap(src='/imu/data', dst='/imu/data'), # Đổi '/imu/data' ở src thành topic thực tế của Xsens nếu nó khác
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(xsens_pkg, 'launch', 'xsens_mti_node.launch.py')])
        )
    ])



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

                    'input_odom_topic': '/odometry/filtered',
                    'output_odom_topic': '/mpc_state',
                    'cmd_vel_topic': '/ackermann_controller/cmd_vel',
                    # --- Bổ sung toàn bộ thông số lọc nhiễu vào đây ---
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

        arg_sim_time, arg_controller, 
        arg_run_joy, 


        robot_state_publisher,        
        ackermann_node,   
        mpc_filter_node,
        joy_launch_group, 
        stamped_mux_node,   
        localization_launch,
        high_level_control, 
        xsens_launch,
    ])