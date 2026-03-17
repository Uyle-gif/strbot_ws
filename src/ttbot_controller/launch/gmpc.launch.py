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
    
    # 3. Đường dẫn tới file config YAML
    config_file = os.path.join(pkg_share, 'config', 'gmpc_params.yaml')

    # 4. Khởi tạo Node GMPC
    gmpc_node = Node(
        package="ttbot_controller",
        executable="gmpc_controller",
        name="gmpc_controller",
        output="screen",
        parameters=[
            config_file,  # Load toàn bộ trọng số (Q, R, N_p...) từ file YAML
            {
                "use_sim_time": LaunchConfiguration("use_sim_time") # Động theo Argument
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gmpc_node
    ])