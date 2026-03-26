#include <memory>
#include <vector>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/utils.h>

namespace ttbot_motion
{

class ObstacleAvoider : public rclcpp::Node
{
public:
  ObstacleAvoider()
  : Node("obstacle_avoider"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // --- THÔNG SỐ SIÊU NÉ (CHỈNH TẠI ĐÂY) ---
    avoid_threshold_ = this->declare_parameter<int>("avoid_threshold", 10); // Nhạy với quầng xanh nhạt
    swerve_width_ = this->declare_parameter<double>("swerve_width", 3.0);    // Độ rộng vòng cung (né xa 2.0m)
    lookahead_dist_ = this->declare_parameter<double>("lookahead_dist", 6.0); // Tầm nhìn xa 5m
    
    // Khoảng cách mét để xác định điểm bắt đầu và kết thúc vòng cung
    anchor_dist_before_ = 1.5; 
    anchor_dist_after_ = 2.0;
    
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_plan", 10, std::bind(&ObstacleAvoider::pathCallback, this, std::placeholders::_1));

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", 10, std::bind(&ObstacleAvoider::costmapCallback, this, std::placeholders::_1));

    mpc_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ObstacleAvoider::processPath, this));

    RCLCPP_INFO(this->get_logger(), "BEZIER SUPER AVOIDER READY. Swerving early and wide!");
  }

private:
  struct Point { double x; double y; };

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_global_path_ = msg;
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_costmap_ = msg;
  }

  void processPath() {
    nav_msgs::msg::Path::SharedPtr global_path;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      global_path = latest_global_path_;
      costmap = latest_costmap_;
    }

    if (!global_path || !costmap || global_path->poses.empty()) return;

    // 1. Chuyển path về frame của costmap (odom) để tính toán chính xác tuyệt đối
    nav_msgs::msg::Path working_path;
    if (!transformPath(*global_path, costmap->header.frame_id, working_path)) return;

    // 2. Chặt đuôi (Xóa phần path sau lưng robot)
    prunePath(working_path);

    // 3. LOGIC VÒNG CUNG BEZIER
    if (working_path.poses.size() > 10) {
      int first_hit_idx = -1;
      double dist_acc = 0.0;

      // Tìm điểm đầu tiên đụng lớp inflation trong tầm nhìn lookahead
      for (size_t i = 1; i < working_path.poses.size(); ++i) {
        dist_acc += std::hypot(working_path.poses[i].pose.position.x - working_path.poses[i-1].pose.position.x,
                                working_path.poses[i].pose.position.y - working_path.poses[i-1].pose.position.y);
        
        if (dist_acc > lookahead_dist_) break;

        if (getCostAt(working_path.poses[i].pose.position.x, working_path.poses[i].pose.position.y, *costmap) >= avoid_threshold_) {
          first_hit_idx = i;
          break;
        }
      }

      // Nếu phát hiện vật cản phía trước
      if (first_hit_idx != -1) {
        // Tìm 2 điểm Neo (P0, P3) dựa trên KHOẢNG CÁCH MÉT
        int start_idx = findIndexByDistance(working_path, first_hit_idx, -anchor_dist_before_);
        int end_idx = findIndexByDistance(working_path, first_hit_idx, anchor_dist_after_);

        Point p0 = {working_path.poses[start_idx].pose.position.x, working_path.poses[start_idx].pose.position.y};
        Point p3 = {working_path.poses[end_idx].pose.position.x, working_path.poses[end_idx].pose.position.y};

        // Tính toán vector lách vuông góc
        double dx = p3.x - p0.x;
        double dy = p3.y - p0.y;
        double chord = std::hypot(dx, dy);
        if (chord > 0.1) {
          double nx = -dy / chord; // Pháp tuyến (trái)
          double ny = dx / chord;

          // Quyết định né trái hay phải (bên nào "sạch" costmap hơn)
          double side = (getCostAt(p0.x + nx*0.5, p0.y + ny*0.5, *costmap) > 
                         getCostAt(p0.x - nx*0.5, p0.y - ny*0.5, *costmap)) ? -1.0 : 1.0;

          // Tạo 2 điểm điều khiển (Control Points) để kéo đường thành vòng cung
          Point p1 = {p0.x + dx * 0.3 + nx * side * swerve_width_, p0.y + dy * 0.3 + ny * side * swerve_width_};
          Point p2 = {p0.x + dx * 0.7 + nx * side * swerve_width_, p0.y + dy * 0.7 + ny * side * swerve_width_};

          // Sinh các điểm trên đường cong Bezier bậc 3
          std::vector<geometry_msgs::msg::PoseStamped> bezier_arc;
          for (double t = 0; t <= 1.0; t += 0.05) {
            double b0 = std::pow(1-t, 3);
            double b1 = 3 * std::pow(1-t, 2) * t;
            double b2 = 3 * (1-t) * t * t;
            double b3 = std::pow(t, 3);

            geometry_msgs::msg::PoseStamped ps;
            ps.header = working_path.header;
            ps.pose.position.x = b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x;
            ps.pose.position.y = b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y;
            bezier_arc.push_back(ps);
          }

          // Thay thế đoạn thẳng bằng đoạn vòng cung mượt
          working_path.poses.erase(working_path.poses.begin() + start_idx, working_path.poses.begin() + end_idx);
          working_path.poses.insert(working_path.poses.begin() + start_idx, bezier_arc.begin(), bezier_arc.end());
        }
      }
    }

    // 4. Tính lại hướng Heading mượt mà cho MPC
    for (size_t i = 0; i < working_path.poses.size(); ++i) {
      size_t next = std::min(i + 2, working_path.poses.size() - 1);
      double yaw = std::atan2(working_path.poses[next].pose.position.y - working_path.poses[i].pose.position.y,
                              working_path.poses[next].pose.position.x - working_path.poses[i].pose.position.x);
      tf2::Quaternion q; q.setRPY(0, 0, yaw);
      working_path.poses[i].pose.orientation = tf2::toMsg(q);
    }

    // 5. Publish kết quả (Dùng frame của global plan ban đầu)
    nav_msgs::msg::Path final_path;
    if (transformPath(working_path, global_plan_frame_, final_path)) {
      final_path.header.stamp = this->now();
      mpc_path_pub_->publish(final_path);
    }
  }

  // Hàm bổ trợ: Tìm Index dựa trên khoảng cách mét (tránh phụ thuộc vào mật độ điểm)
  int findIndexByDistance(const nav_msgs::msg::Path & path, int start_idx, double target_dist) {
    double d = 0;
    int i = start_idx;
    int step = (target_dist > 0) ? 1 : -1;
    while (i + step >= 0 && i + step < (int)path.poses.size() && std::abs(d) < std::abs(target_dist)) {
      d += std::hypot(path.poses[i+step].pose.position.x - path.poses[i].pose.position.x, 
                      path.poses[i+step].pose.position.y - path.poses[i].pose.position.y);
      i += step;
    }
    return i;
  }

  int getCostAt(double wx, double wy, const nav_msgs::msg::OccupancyGrid & cmap) {
    double res = cmap.info.resolution;
    int mx = (wx - cmap.info.origin.position.x) / res;
    int my = (wy - cmap.info.origin.position.y) / res;
    if (mx < 0 || my < 0 || mx >= (int)cmap.info.width || my >= (int)cmap.info.height) return -1;
    return cmap.data[my * cmap.info.width + mx];
  }

  void prunePath(nav_msgs::msg::Path & path) {
    geometry_msgs::msg::TransformStamped tf;
    try { tf = tf_buffer_.lookupTransform(path.header.frame_id, "base_link", tf2::TimePointZero); }
    catch (...) { return; }
    auto closest = std::min_element(path.poses.begin(), path.poses.end(), [&](auto a, auto b){
      return std::hypot(a.pose.position.x - tf.transform.translation.x, a.pose.position.y - tf.transform.translation.y) < 
             std::hypot(b.pose.position.x - tf.transform.translation.x, b.pose.position.y - tf.transform.translation.y);
    });
    path.poses.erase(path.poses.begin(), closest);
  }

  bool transformPath(const nav_msgs::msg::Path & in, const std::string & frame, nav_msgs::msg::Path & out) {
    if (in.header.frame_id == frame) { out = in; return true; }
    geometry_msgs::msg::TransformStamped tf;
    try { tf = tf_buffer_.lookupTransform(frame, in.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1)); }
    catch (...) { return false; }
    out.header = in.header; out.header.frame_id = frame;
    out.poses.resize(in.poses.size());
    for (size_t i = 0; i < in.poses.size(); ++i) tf2::doTransform(in.poses[i], out.poses[i], tf);
    global_plan_frame_ = in.header.frame_id;
    return true;
  }

  std::mutex data_mutex_;
  std::string global_plan_frame_;
  nav_msgs::msg::Path::SharedPtr latest_global_path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  int avoid_threshold_;
  double swerve_width_, lookahead_dist_, anchor_dist_before_, anchor_dist_after_;
};
} 

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ttbot_motion::ObstacleAvoider>());
  rclcpp::shutdown();
  return 0;
}