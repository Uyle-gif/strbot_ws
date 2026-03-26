cmake_minimum_required(VERSION 3.8)
project(ttbot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. TÌM PACKAGES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)          # <--- Quan trọng cho TransformListener
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)      # <--- Thêm vào nếu sau này dùng LaserScan/Pointcloud

# Thư viện toán học cho MPC
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)

include_directories(include)

# --- 2. XÂY DỰNG THƯ VIỆN MPC (Controller Plugin) ---
add_library(ttbot_motion SHARED src/mpc_controller.cpp)
ament_target_dependencies(ttbot_motion
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

# Link thư viện toán học (Eigen & OSQP)
target_link_libraries(ttbot_motion
  Eigen3::Eigen
  osqp::osqp
)

# --- 3. XÂY DỰNG EXECUTABLE (Obstacle Avoider Node) ---
add_executable(obstacle_avoider src/obstacle_avoider.cpp)
# Bổ sung tf2_ros vào đây để tránh lỗi link thư viện
ament_target_dependencies(obstacle_avoider 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

# --- 4. EXPORT & INSTALL ---
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

# Export dependencies để các package khác có thể sử dụng
ament_export_dependencies(
  rclcpp 
  nav2_core 
  nav2_costmap_2d 
  nav2_util 
  pluginlib 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  std_msgs
)
ament_export_include_directories(include)
ament_package()