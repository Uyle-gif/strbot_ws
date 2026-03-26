#ifndef TTBOT_PLANNING__A_STAR_PLANNER_HPP_
#define TTBOT_PLANNING__A_STAR_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace ttbot_planning
{

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

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() = default;

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

  // Đã cập nhật thành Euclidean Distance để chống xe chạy sát góc tường
  double euclideanDistance(const GraphNode & node, const GraphNode & goal_node);
  
  bool poseOnMap(const GraphNode & node);
  GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
  geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
  unsigned int poseToCell(const GraphNode & node);
};

}  // namespace ttbot_planning

#endif  // TTBOT_PLANNING__A_STAR_PLANNER_HPP_