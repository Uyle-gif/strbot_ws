import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap # <--- Nhớ import SetRemap

def generate_launch_description():

    pkg_description = get_package_share_directory('ttbot_description')
    pkg_localization = get_package_share_directory('ttbot_localization')
    pkg_controller = get_package_share_directory('ttbot_controller')
  #  pkg_fast_lio = get_package_share_directory('fast_lio')

    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

  #  run_fastlio = LaunchConfiguration('run_fastlio')
    arg_run_fastlio = DeclareLaunchArgument(
            'run_fastlio',
            default_value='true', 
            description='Set to true to run Fast-LIO mapping in Simulation'
        )
    
    run_qgc = LaunchConfiguration('run_qgc')
    arg_run_qgc = DeclareLaunchArgument('run_qgc', default_value='true', description='Enable QGC Bridge')

    run_joy = LaunchConfiguration('run_joy')
    arg_run_joy = DeclareLaunchArgument('run_joy', default_value='false', description='Run joystick')

    controller_type = LaunchConfiguration('controller_type')
    # arg_controller = DeclareLaunchArgument('controller_type', default_value='mpc')
    arg_controller = DeclareLaunchArgument(
            'controller_type',
            default_value='gmpc',
            description='Controller type: mpc, gmpc, stanley'
    )

    run_path = LaunchConfiguration('run_path')
    arg_run_path = DeclareLaunchArgument('run_path', default_value='false')

    run_rviz = LaunchConfiguration('run_rviz')
    arg_rviz = DeclareLaunchArgument('run_rviz', default_value='false')

    path_file = LaunchConfiguration('path_file')
    arg_path = DeclareLaunchArgument('path_file', default_value='path_l.csv')


    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # fast_lio_config_path = os.path.join(pkg_fast_lio, 'config', 'velodyne_sim.yaml') 

    # fast_lio_node = Node(
    #     condition=IfCondition(run_fastlio),
    #     package='fast_lio',
    #     executable='fastlio_mapping',
    #     name='fastlio_mapping',
    #     output='screen',
    #     parameters=[
    #         fast_lio_config_path,
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )

    low_level_control_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'controller.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
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
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_localization, 'launch', 'global_localization.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )
    mpc_filter_node = TimerAction(
        period=10.0, # Chạy sau localization một chút
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
                    
                    # --- Điều chỉnh độ mượt (Low-pass filter) ---
                    'alpha_x': 0.6,    # Hạ xuống 0.6 để lọc nhiễu vị trí
                    'alpha_y': 0.6,
                    'alpha_yaw': 0.6,
                    'alpha_v': 0.4,    # Hạ v và wz xuống 0.4 để GMPC không bị nhiễu đạo hàm
                    'alpha_wz': 0.4,

                    # --- Giới hạn gia tốc bảo vệ xe ---
                    'max_v_rate': 3.0,
                    'max_wz_rate': 4.0,

                    # --- Deadband chống trôi ---
                    'v_standstill_threshold': 0.05,
                    'wz_standstill_threshold': 0.05
                }]
            )
        ]
    )

    qgc_bridge_node = TimerAction(
        period=15.0, 
        actions=[
            Node(
                package='qgc_bridge_cpp',      
                executable='qgc_bridge_node',
                name='qgc_bridge_node',
                output='screen',
                condition=IfCondition(run_qgc),
                parameters=[{ 
                    'use_sim_time': use_sim_time,
                    'heading_offset_deg': 90.0 
                }]
            )
        ]
    )

    path_pub_launch = GroupAction(
        condition=IfCondition(run_path),
        actions=[
            TimerAction(
                period=16.0, 
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'path_publisher.launch.py')),
                        launch_arguments={'use_sim_time': use_sim_time, 'path_file': path_file}.items()
                    )
                ]
            )
        ]
    )
    
    gmpc_group = GroupAction(
    condition=IfCondition(PythonExpression(["'", controller_type, "' == 'gmpc'"])),
    actions=[
        SetRemap(src='/cmd_vel', dst='/mpc_cmd_vel'),
        SetRemap(src='/ackermann_controller/cmd_vel', dst='/mpc_cmd_vel'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_controller, 'launch', 'gmpc.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'desired_speed': '1.5'
            }.items()
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
                launch_arguments={'use_sim_time': use_sim_time, 'desired_speed': '1.5'}.items()
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
                launch_arguments={'use_sim_time': use_sim_time, 'desired_speed': '1.5'}.items()
            )
        ]
    )

    high_level_control_delayed = TimerAction(
        period=12.0,
        actions=[stanley_group, mpc_group, gmpc_group]
    )

    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_description, 'rviz', 'display.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        arg_sim_time,
        arg_run_fastlio,
        arg_run_qgc,
        arg_controller,
        arg_rviz,
        arg_path,
        arg_run_path,
        arg_run_joy,

        gazebo_launch,
       # fast_lio_node,
        low_level_control_launch,
        mpc_filter_node,
        joy_launch_group,
        stamped_mux_node, 
        # qgc_bridge_node,
        
        localization_launch,
        path_pub_launch,
        high_level_control_delayed,
        
        # rviz_node
    ])