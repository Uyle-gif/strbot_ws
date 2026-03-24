#ifndef TTBOT_PLANNING__A_STAR_PLANNER_NEW_HPP_
#define TTBOT_PLANNING__A_STAR_PLANNER_NEW_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace ttbot_planning
{

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
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  
  nav2_costmap_2d::Costmap2D* costmap_; 
  
  std::string global_frame_;
  std::string name_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> mpc_path_pub_;

  
  nav_msgs::msg::Path interpolatePath(const nav_msgs::msg::Path &raw_path, double resolution);
  
  nav_msgs::msg::Path eliminateInflectionPoints(const nav_msgs::msg::Path &path);
  bool isLineOfSightClear(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal);
  

};

}  

#endif  