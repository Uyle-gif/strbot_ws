#include "ttbot_planning/a_star_planner_new.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace ttbot_planning
{

void AStarPlannerNew::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Ép kiểu (void) để tắt cảnh báo "unused parameter" cho tf
  (void)tf;

  node_ = parent;
  name_ = name;
  costmap_ = costmap_ros->getCostmap(); 
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in planner configure.");
  }

  // Initialize publisher for MPC path tracking
  mpc_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
  
  RCLCPP_INFO(node->get_logger(), "AStarPlannerNew has been configured.");
}

void AStarPlannerNew::cleanup()
{
  mpc_path_pub_.reset();
  RCLCPP_INFO(node_.lock()->get_logger(), "AStarPlannerNew has been cleaned up.");
}

void AStarPlannerNew::activate()
{
  mpc_path_pub_->on_activate();
  RCLCPP_INFO(node_.lock()->get_logger(), "AStarPlannerNew has been activated.");
}

void AStarPlannerNew::deactivate()
{
  mpc_path_pub_->on_deactivate();
  RCLCPP_INFO(node_.lock()->get_logger(), "AStarPlannerNew has been deactivated.");
}

// =========================================================================
// INTERPOLATION (DENSE PATH FOR MPC TRACKING)
// =========================================================================
nav_msgs::msg::Path AStarPlannerNew::interpolatePath(const nav_msgs::msg::Path &raw_path, double resolution) 
{
  nav_msgs::msg::Path dense_path;
  dense_path.header = raw_path.header;
  
  if (raw_path.poses.size() < 2) return raw_path;

  for (size_t i = 0; i < raw_path.poses.size() - 1; ++i) {
    dense_path.poses.push_back(raw_path.poses[i]);
    
    double x1 = raw_path.poses[i].pose.position.x;
    double y1 = raw_path.poses[i].pose.position.y;
    double x2 = raw_path.poses[i+1].pose.position.x;
    double y2 = raw_path.poses[i+1].pose.position.y;
    
    double dist = std::hypot(x2 - x1, y2 - y1);
    
    // Calculate number of interpolation steps
    int num_steps = std::max(1, static_cast<int>(dist / resolution));
    
    for (int j = 1; j < num_steps; ++j) {
      geometry_msgs::msg::PoseStamped pt = raw_path.poses[i];
      pt.pose.position.x = x1 + (x2 - x1) * ((double)j / num_steps);
      pt.pose.position.y = y1 + (y2 - y1) * ((double)j / num_steps);
      dense_path.poses.push_back(pt);
    }
  }
  dense_path.poses.push_back(raw_path.poses.back());
  return dense_path;
}

// =========================================================================
// MAIN PATH GENERATION (CREATE PLAN)
// =========================================================================
nav_msgs::msg::Path AStarPlannerNew::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  auto node = node_.lock();
  
  nav_msgs::msg::Path final_path;
  final_path.header.frame_id = global_frame_;
  final_path.header.stamp = node->now();

  // -----------------------------------------------------------------
  // 1. OPEN AREA BRANCH (NO OBSTACLES)
  // -----------------------------------------------------------------
  if (isLineOfSightClear(start, goal)) {
    final_path.poses.push_back(start);
    final_path.poses.push_back(goal);
    
    // INTERPOLATE: Add dense waypoints using costmap resolution
    final_path = interpolatePath(final_path, costmap_->getResolution());

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
    RCLCPP_INFO(node->get_logger(), "[A* Improved - OPEN AREA] Time: %.3f ms", elapsed_time.count());
    
    mpc_path_pub_->publish(final_path);
    return final_path;
  }

  // -----------------------------------------------------------------
  // 2. MAZE BRANCH (A* SEARCH)
  // -----------------------------------------------------------------
  
  // TODO: Insert your existing A* search loop and backtracking logic here.
  // final_path = ...
  
  // Check if pathfinding failed
  if (final_path.poses.empty()) {
    RCLCPP_WARN(node->get_logger(), "[A* Improved] Failed to find a valid path.");
    mpc_path_pub_->publish(final_path); // Publish empty path to notify MPC
    return final_path;
  }

  // INTERPOLATE: Densify the maze path before smoothing
  final_path = interpolatePath(final_path, costmap_->getResolution());

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
  RCLCPP_INFO(node->get_logger(), "[A* Improved - MAZE] Time: %.3f ms", elapsed_time.count());

  // Publish the final trajectory for MPC
  mpc_path_pub_->publish(final_path);

  return final_path;
}

// -----------------------------------------------------------------
// TODO: Paste your custom implementations (eliminateInflectionPoints, 
// isLineOfSightClear, gridToWorld, etc.) below.
// -----------------------------------------------------------------
bool AStarPlannerNew::isLineOfSightClear(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    // Ép kiểu (void) để tắt cảnh báo "unused parameter" cho start và goal
    (void)start;
    (void)goal;
    
    // Custom raycasting logic
    return true; 
}

}  // namespace ttbot_planning

// Register the plugin
PLUGINLIB_EXPORT_CLASS(ttbot_planning::AStarPlannerNew, nav2_core::GlobalPlanner)