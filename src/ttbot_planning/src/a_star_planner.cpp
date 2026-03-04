#include "ttbot_planning/a_star_planner.hpp"

namespace ttbot_planning
{
AStarPlanner::AStarPlanner() : Node("ttbot_a_star_planner") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(10);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos, std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));
    
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&AStarPlanner::goalCallback, this, std::placeholders::_1));
    
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
    
    RCLCPP_INFO(get_logger(), "A* Planner node initialized.");
}

void AStarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = msg;
}

void AStarPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if(!map_) { 
        RCLCPP_ERROR(get_logger(), "Map not ready yet!"); 
        return; 
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
        // Tìm tọa độ robot hiện tại trong hệ tọa độ map
        tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(get_logger(), "TF Error: %s", ex.what()); 
        return;
    }

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = tf.transform.translation.x;
    start_pose.position.y = tf.transform.translation.y;
    start_pose.orientation = tf.transform.rotation;

    auto path = plan(start_pose, msg->pose);
    
    if (!path.poses.empty()) {
        RCLCPP_INFO(get_logger(), "Optimal path found successfully.");
        path_pub_->publish(path);
    } else {
        RCLCPP_WARN(get_logger(), "Failed to find a valid path to the goal.");
    }
}

nav_msgs::msg::Path AStarPlanner::plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal) {
    // 8 hướng di chuyển
    std::vector<std::pair<int, int>> dirs = {{-1,0}, {1,0}, {0,-1}, {0,1}, {-1,-1}, {-1,1}, {1,-1}, {1,1}};
    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> open_set;
    std::vector<bool> closed_set(map_->info.width * map_->info.height, false);

    GraphNode start_node = worldToGrid(start);
    GraphNode goal_node = worldToGrid(goal);
    start_node.heuristic = manhattanDistance(start_node, goal_node);
    open_set.push(start_node);

    GraphNode current;
    bool found = false;
    
    while(!open_set.empty() && rclcpp::ok()) {
        current = open_set.top(); open_set.pop();
        
        if(current == goal_node) { found = true; break; }
        
        unsigned int idx = poseToCell(current);
        if(closed_set[idx]) continue;
        closed_set[idx] = true;

        for(auto & d : dirs) {
            GraphNode neighbor = current + d;
            if(poseOnMap(neighbor) && map_->data[poseToCell(neighbor)] == 0 && !closed_set[poseToCell(neighbor)]) {
                neighbor.cost = current.cost + 1;
                neighbor.heuristic = manhattanDistance(neighbor, goal_node);
                neighbor.prev = std::make_shared<GraphNode>(current);
                open_set.push(neighbor);
            }
        }
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = map_->header.frame_id;
    path.header.stamp = get_clock()->now();

    if(found) {
        while(current.prev) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose = gridToWorld(current);
            path.poses.push_back(ps);
            current = *current.prev;
        }
        std::reverse(path.poses.begin(), path.poses.end());
    }
    return path;
}

double AStarPlanner::manhattanDistance(const GraphNode &n, const GraphNode &g) { 
    return std::abs(n.x - g.x) + std::abs(n.y - g.y); 
}

bool AStarPlanner::poseOnMap(const GraphNode & n) { 
    return n.x >= 0 && n.x < (int)map_->info.width && n.y >= 0 && n.y < (int)map_->info.height; 
}

GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & p) { 
    return GraphNode((p.position.x - map_->info.origin.position.x) / map_->info.resolution, 
                     (p.position.y - map_->info.origin.position.y) / map_->info.resolution); 
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode & n) {
    geometry_msgs::msg::Pose p;
    p.position.x = n.x * map_->info.resolution + map_->info.origin.position.x;
    p.position.y = n.y * map_->info.resolution + map_->info.origin.position.y;
    p.orientation.w = 1.0;
    return p;
}

unsigned int AStarPlanner::poseToCell(const GraphNode & n) { 
    return map_->info.width * n.y + n.x; 
}
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ttbot_planning::AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}