from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------------------------------
    # 1. System params
    # --------------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    # --------------------------------------------------------------------------
    # 2. Vehicle / controller params
    # --------------------------------------------------------------------------
    desired_speed_arg = DeclareLaunchArgument(
        "desired_speed",
        default_value="1.5",
        description="Reference forward speed for GMPC"
    )

    wheel_base_arg = DeclareLaunchArgument(
        "wheel_base",
        default_value="0.65",
        description="Wheel base of the vehicle"
    )

    max_steer_deg_arg = DeclareLaunchArgument(
        "max_steer_deg",
        default_value="30.0",
        description="Maximum steering angle in degrees"
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        "goal_tolerance",
        default_value="0.3",
        description="Distance threshold to consider goal reached"
    )

    # --------------------------------------------------------------------------
    # 3. GMPC horizon / sampling
    # --------------------------------------------------------------------------
    N_p_arg = DeclareLaunchArgument(
        "N_p",
        default_value="20",
        description="Prediction horizon"
    )

    dt_mpc_arg = DeclareLaunchArgument(
        "dt_mpc",
        default_value="0.1",
        description="Sampling time for GMPC"
    )

    # --------------------------------------------------------------------------
    # 4. GMPC weights
    # --------------------------------------------------------------------------
    Q_ex_arg = DeclareLaunchArgument(
        "Q_ex",
        default_value="10.0",
        description="Weight for longitudinal error ex"
    )

    Q_ey_arg = DeclareLaunchArgument(
        "Q_ey",
        default_value="35.0",
        description="Weight for lateral error ey"
    )

    Q_epsi_arg = DeclareLaunchArgument(
        "Q_epsi",
        default_value="21.0",
        description="Weight for heading error epsi"
    )

    R_v_arg = DeclareLaunchArgument(
        "R_v",
        default_value="1.0",
        description="Weight for velocity control effort"
    )

    R_omega_arg = DeclareLaunchArgument(
        "R_omega",
        default_value="0.6",
        description="Weight for angular velocity control effort"
    )

    R_dv_arg = DeclareLaunchArgument(
        "R_dv",
        default_value="1.0",
        description="Weight for delta-v smoothness"
    )

    R_domega_arg = DeclareLaunchArgument(
        "R_domega",
        default_value="6.5",
        description="Weight for delta-omega smoothness"
    )

    # --------------------------------------------------------------------------
    # 5. GMPC node
    # --------------------------------------------------------------------------
    gmpc_node = Node(
        package="ttbot_controller",
        executable="gmpc_controller",
        name="gmpc_controller",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),

            "desired_speed": LaunchConfiguration("desired_speed"),
            "wheel_base": LaunchConfiguration("wheel_base"),
            "max_steer_deg": LaunchConfiguration("max_steer_deg"),
            "goal_tolerance": LaunchConfiguration("goal_tolerance"),

            "N_p": LaunchConfiguration("N_p"),
            "dt_mpc": LaunchConfiguration("dt_mpc"),

            "Q_ex": LaunchConfiguration("Q_ex"),
            "Q_ey": LaunchConfiguration("Q_ey"),
            "Q_epsi": LaunchConfiguration("Q_epsi"),

            "R_v": LaunchConfiguration("R_v"),
            "R_omega": LaunchConfiguration("R_omega"),

            "R_dv": LaunchConfiguration("R_dv"),
            "R_domega": LaunchConfiguration("R_domega"),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,

        desired_speed_arg,
        wheel_base_arg,
        max_steer_deg_arg,
        goal_tolerance_arg,

        N_p_arg,
        dt_mpc_arg,

        Q_ex_arg,
        Q_ey_arg,
        Q_epsi_arg,

        R_v_arg,
        R_omega_arg,
        R_dv_arg,
        R_domega_arg,

        gmpc_node
    ])