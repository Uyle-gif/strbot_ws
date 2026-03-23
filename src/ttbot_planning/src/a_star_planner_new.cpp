#include <cmath>
#include <algorithm>
#include "ttbot_planning/a_star_planner_new.hpp" 
#include <chrono>

namespace ttbot_planning
{
void AStarPlannerNew::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Initialize the smooth path action client
  smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "smooth_path");
  
  // Initialize the publisher for the MPC path visualization
  mpc_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
  
  RCLCPP_INFO(node_->get_logger(), "Configured Improved A* Planner with Turning Penalty: %.2f", turning_penalty_);
}

void AStarPlannerNew::cleanup() 
{ 
  RCLCPP_INFO(node_->get_logger(), "Cleaning up AStarPlannerNew"); 
}

void AStarPlannerNew::activate() 
{ 
  smooth_client_->wait_for_action_server(std::chrono::seconds(3));
  mpc_path_pub_->on_activate(); 
}

void AStarPlannerNew::deactivate() 
{ 
  RCLCPP_INFO(node_->get_logger(), "Deactivating AStarPlannerNew");
  mpc_path_pub_->on_deactivate();   
}

nav_msgs::msg::Path AStarPlannerNew::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // Start the timer to measure planning performance
  auto start_time = std::chrono::high_resolution_clock::now();

  // 8-connected grid exploration directions
  std::vector<std::pair<int, int>> explore_directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
  };

  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  
  std::vector<uint8_t> closed_set(size_x * size_y, 0); 
  std::vector<double> min_cost(size_x * size_y, 1e9); 
  
  GraphNode start_node = worldToGrid(start.pose);
  GraphNode goal_node = worldToGrid(goal.pose);
  start_node.heuristic = euclideanDistance(start_node, goal_node);
  
  min_cost[poseToCell(start_node)] = 0.0;

  // --- BRANCH 1: OPEN AREA (Direct Line of Sight) ---
  if (isLineOfSightClear(start_node, goal_node)) {
    nav_msgs::msg::Path open_path;
    open_path.header.frame_id = global_frame_;
    open_path.header.stamp = node_->now();
    
    geometry_msgs::msg::PoseStamped p1, p2;
    p1.header = open_path.header; p1.pose = gridToWorld(start_node);
    p2.header = open_path.header; p2.pose = gridToWorld(goal_node);
    
    open_path.poses.push_back(p1);
    open_path.poses.push_back(p2);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
    RCLCPP_INFO(node_->get_logger(), "[A* Improved - OPEN AREA] Time: %.3f ms", elapsed_time.count());
    
    // Publish the direct path to /mpc_path and return
    mpc_path_pub_->publish(open_path);
    return open_path;
  }

  // --- BRANCH 2: MAZE / OBSTACLE AVOIDANCE ---
  std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
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
            
            if (!closed_set[n_idx] && cm_cost < obstacle_threshold_) {
                // Base step cost: 1.0 for straight, 1.414 for diagonal
                double step_cost = (dir.first == 0 || dir.second == 0) ? 1.0 : 1.41421356; 
                double turning_cost = 0.0;
                
                // Penalize turning to create smoother initial paths
                if (active_node.prev != nullptr) {
                    int dx_prev = active_node.x - active_node.prev->x;
                    int dy_prev = active_node.y - active_node.prev->y;       
                    if (dx_prev != dir.first || dy_prev != dir.second) {
                        turning_cost = 0.2; 
                    }
                }

                // TUNE COST: Force the robot to stay in free space, avoid high-cost regions
                // Use a power function to strongly penalize higher costmap values
                double costmap_penalty = pow(cm_cost / 255.0, 2) * 50.0;

                new_node.cost = active_node.cost + step_cost + costmap_penalty + turning_cost;

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

  nav_msgs::msg::Path final_path;
  final_path.header.frame_id = global_frame_;
  final_path.header.stamp = node_->now();

  if(found) {
    // Backtrack to build the path
    GraphNode current = active_node;
    while(rclcpp::ok()) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = final_path.header;
      pose_stamped.pose = gridToWorld(current);
      final_path.poses.push_back(pose_stamped);
      if (current.prev == nullptr) break; 
      current = *current.prev;
    }
    
    std::reverse(final_path.poses.begin(), final_path.poses.end());

    // Prune unnecessary waypoints based on line-of-sight
    final_path = eliminateInflectionPoints(final_path); 

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
    RCLCPP_INFO(node_->get_logger(), "[A* Improved - MAZE] Time: %.3f ms", elapsed_time.count());

    // Call the smoothing action server if the path has enough points
    if(final_path.poses.size() > 2 && smooth_client_->action_server_is_ready()){
      nav2_msgs::action::SmoothPath::Goal path_smooth;
      path_smooth.path = final_path; 
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
              // Overwrite the raw path with the smoothed path
              final_path = result_path.result->path; 
            } else {
              RCLCPP_WARN(node_->get_logger(), "Smoother failed. Using raw path.");
            }
          }
        }
      } else {
        RCLCPP_WARN(node_->get_logger(), "Smoother action server timeout. Using raw path.");
      }
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "[A* Improved] Failed to find a valid path.");
  }
  
  // Publish the resulting path to /mpc_path (either smoothed, raw, or empty if failed)
  mpc_path_pub_->publish(final_path);
  
  return final_path;
}

double AStarPlannerNew::euclideanDistance(const GraphNode &node, const GraphNode &goal_node) {
    return std::hypot(node.x - goal_node.x, node.y - goal_node.y);
}

bool AStarPlannerNew::isLineOfSightClear(const GraphNode &a, const GraphNode &b) {
    int x0 = a.x, y0 = a.y;
    int x1 = b.x, y1 = b.y;
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    // TUNE LOS: Safety threshold for Line-Of-Sight raycasting.
    // If cost >= 30 (entering light purple inflation zone), forbid the ray from passing through
    unsigned char safe_los_threshold = 30; 

    while (true) {
        if (!poseOnMap(GraphNode(x0, y0))) return false;
        
        // Use safe_los_threshold to ensure the shortcut does not clip obstacle corners
        if (costmap_->getCost(x0, y0) >= safe_los_threshold) return false;
        
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    return true;
}

nav_msgs::msg::Path AStarPlannerNew::eliminateInflectionPoints(const nav_msgs::msg::Path &raw_path) {
    nav_msgs::msg::Path pruned_path;
    pruned_path.header = raw_path.header;
    
    if (raw_path.poses.size() <= 2) return raw_path;

    pruned_path.poses.push_back(raw_path.poses.front());
    size_t current_idx = 0;
    
    while (current_idx < raw_path.poses.size() - 1) {
        size_t next_idx = current_idx + 1;
        
        for (size_t j = current_idx + 2; j < raw_path.poses.size(); ++j) {
            GraphNode n1 = worldToGrid(raw_path.poses[current_idx].pose);
            GraphNode n2 = worldToGrid(raw_path.poses[j].pose);
            
            // If the line of sight is clear, we can skip intermediate points
            if (isLineOfSightClear(n1, n2)) {
                next_idx = j; 
            } else {
                break; 
            }
        }
        
        pruned_path.poses.push_back(raw_path.poses[next_idx]);
        current_idx = next_idx;
    }
    
    return pruned_path;
}

bool AStarPlannerNew::poseOnMap(const GraphNode & node) {
    return node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) && 
           node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY());
}

GraphNode AStarPlannerNew::worldToGrid(const geometry_msgs::msg::Pose & pose) {
    int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
    return GraphNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose AStarPlannerNew::gridToWorld(const GraphNode & node) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
    pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
    pose.orientation.w = 1.0;
    return pose;
}

unsigned int AStarPlannerNew::poseToCell(const GraphNode & node) {
    return costmap_->getSizeInCellsX() * node.y + node.x;
}

} // namespace ttbot_planning

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ttbot_planning::AStarPlannerNew, nav2_core::GlobalPlanner)