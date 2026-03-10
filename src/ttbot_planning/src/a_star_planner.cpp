#include <cmath>
#include <algorithm>
#include "ttbot_planning/a_star_planner.hpp"

namespace ttbot_planning
{
void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "smooth_path");
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s of type AStarPlanner", name_.c_str());
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type AStarPlanner", name_.c_str());
  if (!smooth_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server smooth_path not available after waiting");
  }
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type AStarPlanner", name_.c_str());
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<std::pair<int, int>> explore_directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
  };

  std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
  
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  std::vector<bool> closed_set(size_x * size_y, false);
  
  GraphNode start_node = worldToGrid(start.pose);
  GraphNode goal_node = worldToGrid(goal.pose);
  start_node.heuristic = manhattanDistance(start_node, goal_node);
  pending_nodes.push(start_node);

  GraphNode active_node;
  bool found = false;

  while (!pending_nodes.empty() && rclcpp::ok()) {
    active_node = pending_nodes.top();
    pending_nodes.pop();

    if (active_node == goal_node) {
      found = true;
      break;
    }

    unsigned int idx = poseToCell(active_node);
    if(closed_set[idx]) continue;
    closed_set[idx] = true;

    for (const auto & dir : explore_directions) {
        GraphNode new_node = active_node + dir;
        
        if (poseOnMap(new_node)) {
            unsigned int n_idx = poseToCell(new_node);
            unsigned char cost = costmap_->getCost(new_node.x, new_node.y);
            
            if (!closed_set[n_idx] && cost < 99) {
                new_node.cost = active_node.cost + 1 + cost;
                new_node.heuristic = manhattanDistance(new_node, goal_node);
                new_node.prev = std::make_shared<GraphNode>(active_node);
                pending_nodes.push(new_node);
            }
        }
    }
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  if(found) {
    while(active_node.prev && rclcpp::ok()) {
      geometry_msgs::msg::PoseStamped last_pose_stamped;
      last_pose_stamped.header = path.header;
      last_pose_stamped.pose = gridToWorld(active_node);
      path.poses.push_back(last_pose_stamped);
      active_node = *active_node.prev;
    }
    std::reverse(path.poses.begin(), path.poses.end());

    if(smooth_client_->action_server_is_ready()){
      nav2_msgs::action::SmoothPath::Goal path_smooth;
      path_smooth.path = path;
      path_smooth.check_for_collisions = false;
      path_smooth.smoother_id = "simple_smoother"; 
      path_smooth.max_smoothing_duration.sec = 10;
      
      auto future = smooth_client_->async_send_goal(path_smooth);

      if(future.wait_for(std::chrono::seconds(3)) == std::future_status::ready){
        auto goal_handle = future.get();
        if(goal_handle){
          auto result_future = smooth_client_->async_get_result(goal_handle);
          if(result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready){
            auto result_path = result_future.get();
            if(result_path.code == rclcpp_action::ResultCode::SUCCEEDED){
              path = result_path.result->path;
            }
          }
        }
      }
    }
  }
    
  return path;
}

double AStarPlanner::manhattanDistance(const GraphNode &node, const GraphNode &goal_node) {
    return std::abs(node.x - goal_node.x) + std::abs(node.y - goal_node.y);
}

bool AStarPlanner::poseOnMap(const GraphNode & node) {
    return node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) && 
           node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY());
}

GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose) {
    int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
    return GraphNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode & node) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
    pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
    pose.orientation.w = 1.0;
    return pose;
}

unsigned int AStarPlanner::poseToCell(const GraphNode & node) {
    return costmap_->getSizeInCellsX() * node.y + node.x;
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ttbot_planning::AStarPlanner, nav2_core::GlobalPlanner)
