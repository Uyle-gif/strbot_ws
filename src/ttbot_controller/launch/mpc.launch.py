import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Khai báo Argument để nhận use_sim_time từ bên ngoài
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation time (true for Gazebo, false for Real Robot)"
    )

    # 2. Lấy đường dẫn thư mục share của package
    pkg_share = get_package_share_directory('ttbot_controller')
    
    # 3. Đường dẫn tới file YAML và file CSV
    config_file = os.path.join(pkg_share, 'config', 'mpc_params.yaml')
    default_csv = os.path.join(pkg_share, 'path', 'path_l.csv')

    # 4. Khởi tạo Node MPC
    mpc_node = Node(
        package="ttbot_controller",
        executable="mpc_controller",
        name="mpc_controller",
        output="screen",
        parameters=[
            config_file,  # Load toàn bộ trọng số (Q, R, Np...) từ file YAML
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"), # Động theo Argument
                "path_file": default_csv  # Đường dẫn file csv
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        mpc_node
    ])