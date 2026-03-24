cmake_minimum_required(VERSION 3.8)
project(ttbot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Các thư viện toán học
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)

include_directories(include)

add_library(ttbot_motion SHARED src/mpc_controller.cpp)


add_executable(obstacle_avoider src/obstacle_avoider.cpp)
ament_target_dependencies(obstacle_avoider rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)


# Link thư viện ROS 2
ament_target_dependencies(ttbot_motion
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

# Link thư viện toán học (Eigen & OSQP)
target_link_libraries(ttbot_motion
  Eigen3::Eigen
  osqp::osqp
)

pluginlib_export_plugin_description_file(nav2_core motion_planner_plugins.xml)

install(TARGETS 
  ttbot_motion
  obstacle_avoider

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES motion_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp nav2_core nav2_costmap_2d nav2_util pluginlib geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs std_msgs)
ament_export_include_directories(include)

ament_package()