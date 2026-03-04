import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ttbot_description = get_package_share_directory("ttbot_description")
    world_path = os.path.join(ttbot_description, "worlds", "gps_world.sdf")

    ros_distro = os.environ.get("ROS_DISTRO")
    is_ignition = "True" if ros_distro == "humble" else "False"

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(ttbot_description, "urdf", "ttbot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(ttbot_description).parent.resolve())]
    )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=", is_ignition
        ]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True 
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[
            ("gz_args", ["-r -v 4 ", world_path]) 
        ]
    )

    # gz_spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     arguments=[
    #         "-topic", "robot_description",
    #         "-name", "ttbot",
    #         "-z", "0.1" 
    #     ],
    # )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "ttbot",
            
            "-x", "0.0",  
            "-y", "0.0",  
            "-z", "2"   # Thả từ độ cao 0.5m (để tránh kẹt bánh vào đất)
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'use_sim_time': True}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/gps_world/model/ttbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # --- THÊM 4 DÒNG NÀY CHO REALSENSE D435i ---
            "/camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ('/gps/fix', '/gps/fix'),
            ('/imu', '/imu/out'),
            ('/world/gps_world/model/ttbot/joint_state', '/joint_states'),
            ('/velodyne_points/points', '/velodyne_points/points')
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])