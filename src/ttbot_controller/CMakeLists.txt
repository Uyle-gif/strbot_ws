cmake_minimum_required(VERSION 3.8)
project(ttbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(include)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS} 
  /opt/ros/humble/include
)

# --- Node Ackermann controller ---
add_executable(ackermann_controller
  src/ackermann_controller.cpp
)

ament_target_dependencies(ackermann_controller
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS ackermann_controller
  DESTINATION lib/${PROJECT_NAME}
)

# --- Node Stanley controller ---
add_executable(stanley_controller
  src/stanley_controller.cpp
)
ament_target_dependencies(stanley_controller
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
  tf2_ros
)

install(
  TARGETS stanley_controller
  DESTINATION lib/${PROJECT_NAME}
)


# --- Node mpc_controller ---
add_executable(mpc_controller src/mpc_controller.cpp)


ament_target_dependencies(mpc_controller
    rclcpp
    nav_msgs          
    geometry_msgs
    tf2
    tf2_ros           
    ament_index_cpp
    tf2_geometry_msgs
)

target_link_libraries(mpc_controller
    osqp::osqp 
)

install(
    TARGETS mpc_controller 
    DESTINATION lib/${PROJECT_NAME}
)

# ==========================================================
# --- Node gmpc_controller (Bản mới - Đa tạp Lie SE(2)) ---
# ==========================================================
add_executable(gmpc_controller src/gmpc_controller.cpp)

ament_target_dependencies(gmpc_controller 
    rclcpp 
    nav_msgs 
    geometry_msgs 
    std_msgs
    tf2 
    tf2_ros 
    ament_index_cpp
    tf2_geometry_msgs
)

target_link_libraries(gmpc_controller
    osqp::osqp         # <-- Vẫn link với bộ giải OSQP
    fmt::fmt

)

install(
    TARGETS gmpc_controller 
    DESTINATION lib/${PROJECT_NAME} 
)

# --- Node path_publisher ---
add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  ament_index_cpp
)
install(TARGETS
  path_publisher
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY config launch path
  DESTINATION share/${PROJECT_NAME}/
)

# Cài đặt file Python script
install(PROGRAMS
  scripts/mpc_state_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(stamped_twist_mux src/stamped_twist_mux.cpp)
ament_target_dependencies(stamped_twist_mux 
  rclcpp 
  geometry_msgs 
  nav_msgs
)

install(TARGETS
  stamped_twist_mux
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()