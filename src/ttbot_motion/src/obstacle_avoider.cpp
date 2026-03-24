#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

using std::placeholders::_1;

class ObstacleAvoider : public rclcpp::Node
{
public:
    ObstacleAvoider() : Node("obstacle_avoider")
    {
        sub_global_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/mpc_path", 10, std::bind(&ObstacleAvoider::pathCallback, this, _1));
        sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap/costmap", 10, std::bind(&ObstacleAvoider::costmapCallback, this, _1));
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mpc_state", 10, std::bind(&ObstacleAvoider::odomCallback, this, _1));

        pub_local_path_ = this->create_publisher<nav_msgs::msg::Path>("/local_mpc_path", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObstacleAvoider::generateAvoidancePath, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoider Node Initialized!");
    }

private:
    nav_msgs::msg::Path::SharedPtr global_path_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path_;
    rclcpp::TimerBase::SharedPtr timer_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) { global_path_ = msg; }
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { costmap_ = msg; }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; }

    bool checkCollision(double x, double y)
    {
        if (!costmap_) return false;

        double origin_x = costmap_->info.origin.position.x;
        double origin_y = costmap_->info.origin.position.y;
        double res = costmap_->info.resolution;
        int width = costmap_->info.width;
        int height = costmap_->info.height;

        int grid_x = std::round((x - origin_x) / res);
        int grid_y = std::round((y - origin_y) / res);

        if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) return false;

        int index = grid_y * width + grid_x;
        int cost = costmap_->data[index];

        if (cost > 80 || cost == -1) {
            return true; 
        }
        return false;
    }

    void generateAvoidancePath()
    {
        if (!global_path_ || !costmap_ || !odom_) return;
        if (global_path_->poses.empty()) return;

        double robot_x = odom_->pose.pose.position.x;
        double robot_y = odom_->pose.pose.position.y;

        nav_msgs::msg::Path base_local_path;
        base_local_path.header = global_path_->header;
        
        int start_idx = 0;
        double min_dist = 1e9;
        for (size_t i = 0; i < global_path_->poses.size(); ++i) {
            double dx = global_path_->poses[i].pose.position.x - robot_x;
            double dy = global_path_->poses[i].pose.position.y - robot_y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                start_idx = i;
            }
        }

        int lookahead_points = 40;
        for (int i = start_idx; i < std::min((int)global_path_->poses.size(), start_idx + lookahead_points); ++i) {
            base_local_path.poses.push_back(global_path_->poses[i]);
        }

        if (base_local_path.poses.size() < 2) return;

        std::vector<double> lateral_offsets = {0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2};
        
        nav_msgs::msg::Path best_path;
        bool found_safe_path = false;

        for (double offset : lateral_offsets) {
            nav_msgs::msg::Path candidate_path = base_local_path;
            bool is_collision_free = true;

            for (size_t i = 0; i < candidate_path.poses.size() - 1; ++i) {
                double x1 = candidate_path.poses[i].pose.position.x;
                double y1 = candidate_path.poses[i].pose.position.y;
                double x2 = candidate_path.poses[i+1].pose.position.x;
                double y2 = candidate_path.poses[i+1].pose.position.y;

                double theta = std::atan2(y2 - y1, x2 - x1);
                
                candidate_path.poses[i].pose.position.x = x1 - offset * std::sin(theta);
                candidate_path.poses[i].pose.position.y = y1 + offset * std::cos(theta);

                if (checkCollision(candidate_path.poses[i].pose.position.x, candidate_path.poses[i].pose.position.y)) {
                    is_collision_free = false;
                    break; 
                }
            }

            if (is_collision_free) {
                best_path = candidate_path;
                found_safe_path = true;
                
                if (offset != 0.0) {
                    RCLCPP_WARN(this->get_logger(), "Obstacle detected! Shifting path by %.2f meters.", offset);
                }
                break; 
            }
        }

        if (found_safe_path) {
            pub_local_path_->publish(best_path);
        } else {
            RCLCPP_ERROR(this->get_logger(), "ALL PATHS BLOCKED! STOPPING!");
            nav_msgs::msg::Path empty_path;
            empty_path.header = base_local_path.header;
            pub_local_path_->publish(empty_path);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoider>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}