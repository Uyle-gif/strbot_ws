#ifndef TTBOT_PLANNING__A_STAR_PLANNER_NEW_HPP_
#define TTBOT_PLANNING__A_STAR_PLANNER_NEW_HPP_

#include <string>
#include <memory>
#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace ttbot_planning
{

// Custom Graph Node structure for A* Search
struct GraphNode {
  int x, y;
  double cost, heuristic;
  std::shared_ptr<GraphNode> prev;

  GraphNode() : x(0), y(0), cost(0.0), heuristic(0.0), prev(nullptr) {}
  GraphNode(int x_, int y_) : x(x_), y(y_), cost(0.0), heuristic(0.0), prev(nullptr) {}
  
  GraphNode operator+(const std::pair<int, int> & offset) const {
    return GraphNode(x + offset.first, y + offset.second);
  }

  bool operator==(const GraphNode & other) const {
    return x == other.x && y == other.y;
  }
  
  bool operator>(const GraphNode & other) const {
    return (cost + heuristic) > (other.cost + other.heuristic);
  }
};

class AStarPlannerNew : public nav2_core::GlobalPlanner
{
public:
  AStarPlannerNew() = default;
  ~AStarPlannerNew() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;

  // Nav2 Action Client for Path Smoothing
  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smooth_client_;
  
  // Custom Lifecycle Publisher to feed the downstream MPC Controller
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr mpc_path_pub_;

  double turning_penalty_ = 0.2;
  unsigned char obstacle_threshold_ = 253; 

  // Core Utility Functions
  double euclideanDistance(const GraphNode & node, const GraphNode & goal_node);
  bool isLineOfSightClear(const GraphNode & a, const GraphNode & b);
  nav_msgs::msg::Path eliminateInflectionPoints(const nav_msgs::msg::Path & raw_path);
  
  // Dense Interpolation Function for MPC Tracking
  nav_msgs::msg::Path interpolatePath(const nav_msgs::msg::Path & raw_path, double step_size);
  
  // Map Coordinate Transformations
  bool poseOnMap(const GraphNode & node);
  GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
  geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
  unsigned int poseToCell(const GraphNode & node);
};

}  // namespace ttbot_planning

#endif  // TTBOT_PLANNING__A_STAR_PLANNER_NEW_HPP_