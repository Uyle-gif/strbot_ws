#include <cmath>
#include <algorithm>
#include "ttbot_planning/a_star_planner.hpp"
#include <chrono>

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

  RCLCPP_INFO(node_->get_logger(), "Configured Baseline A* Planner (Strictly Avoid Blue/Cyan Zones)");
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s of type AStarPlanner", name_.c_str());
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type AStarPlanner", name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type AStarPlanner", name_.c_str());
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  std::vector<std::pair<int, int>> explore_directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
  };

  std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
  
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  
  std::vector<uint8_t> closed_set(size_x * size_y, 0); 
  std::vector<double> min_cost(size_x * size_y, 1e9); 
  
  GraphNode start_node = worldToGrid(start.pose);
  GraphNode goal_node = worldToGrid(goal.pose);
  start_node.heuristic = euclideanDistance(start_node, goal_node);
  
  min_cost[poseToCell(start_node)] = 0.0;
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
    closed_set[idx] = 1;

    for (const auto & dir : explore_directions) {
        GraphNode new_node = active_node + dir;
        
        if (poseOnMap(new_node)) {
            unsigned int n_idx = poseToCell(new_node);
            unsigned char cm_cost = costmap_->getCost(new_node.x, new_node.y);
            
            // 🔥 CHẶN VÙNG XANH/TÍM ĐẬM: Hạ ngưỡng xuống 128 (thay vì 253)
            // Nếu cost >= 128, coi như là bức tường tàng hình, không cho phép đi vào.
            if (!closed_set[n_idx] && cm_cost < 128) {
                
                double step_cost = (dir.first == 0 || dir.second == 0) ? 1.0 : 1.41421356;
                
                // 🔥 ÉP RA VÙNG TRẮNG: Phạt bình phương
                // Nếu đi vào vùng lờ mờ tím (cost từ 1 đến 127), điểm phạt sẽ tăng vọt lên rất cao.
                double costmap_penalty = (cm_cost * cm_cost) / 10.0; 
                
                new_node.cost = active_node.cost + step_cost + costmap_penalty;
                
                if (new_node.cost < min_cost[n_idx]) {
                    min_cost[n_idx] = new_node.cost;
                    new_node.heuristic = euclideanDistance(new_node, goal_node);
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    pending_nodes.push(new_node);
                }
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

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
    RCLCPP_INFO(node_->get_logger(), "[A* Baseline] Planning Time: %.3f ms", elapsed_time.count());
  }
    
  return path;
}

double AStarPlanner::euclideanDistance(const GraphNode &node, const GraphNode &goal_node) {
    return std::hypot(node.x - goal_node.x, node.y - goal_node.y);
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