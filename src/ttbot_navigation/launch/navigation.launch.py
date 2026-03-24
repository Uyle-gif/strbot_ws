# The controller_server is running ONLY to maintain & pub local_costmap.
# It not used for control

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ["map_server", "controller_server", "planner_server", "smoother_server", "bt_navigator"]
    
    ttbot_navigation_pkg = get_package_share_directory("ttbot_navigation")
    ttbot_mapping_pkg = get_package_share_directory("ttbot_mapping") 

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    map_yaml_file = LaunchConfiguration('map')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(ttbot_mapping_pkg, 'maps', 'bk_map.yaml'),
        description='Full path to map yaml file to load'
    )

    default_bt_xml_path = os.path.join(
        ttbot_navigation_pkg,
        "behavior_tree",
        "simple_navigation.xml"
    )

    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(
                ttbot_navigation_pkg,
                "config",
                "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(
                ttbot_navigation_pkg,
                "config",
                "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                ttbot_navigation_pkg,
                "config",
                "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(
                ttbot_navigation_pkg,
                "config",
                "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time},
            {"default_nav_to_pose_bt_xml": default_bt_xml_path},
            {"default_nav_through_poses_bt_xml": default_bt_xml_path}
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,                 
        nav2_map_server,         
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
    ])