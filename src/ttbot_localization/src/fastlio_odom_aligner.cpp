#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class FastlioOdomAligner : public rclcpp::Node
{
public:
  FastlioOdomAligner()
  : Node("fastlio_odom_aligner")
  {
    this->declare_parameter<std::string>("raw_odom_topic", "/Odometry");
    this->declare_parameter<std::string>("aligned_odom_topic", "/Odometry_aligned");
    this->declare_parameter<std::string>("initial_pose_topic", "/icp_initial_pose");

    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");

    this->declare_parameter<bool>("wait_for_icp", true);
    this->declare_parameter<bool>("use_first_odom_as_zero", true);
    this->declare_parameter<bool>("lock_after_first_icp", true);

    this->declare_parameter<double>("manual_initial_x", 0.0);
    this->declare_parameter<double>("manual_initial_y", 0.0);
    this->declare_parameter<double>("manual_initial_yaw", 0.0);

    this->get_parameter("raw_odom_topic", raw_odom_topic_);
    this->get_parameter("aligned_odom_topic", aligned_odom_topic_);
    this->get_parameter("initial_pose_topic", initial_pose_topic_);

    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("base_frame", base_frame_);

    this->get_parameter("wait_for_icp", wait_for_icp_);
    this->get_parameter("use_first_odom_as_zero", use_first_odom_as_zero_);
    this->get_parameter("lock_after_first_icp", lock_after_first_icp_);

    this->get_parameter("manual_initial_x", initial_x_);
    this->get_parameter("manual_initial_y", initial_y_);
    this->get_parameter("manual_initial_yaw", initial_yaw_);

    if (!wait_for_icp_) {
      has_initial_pose_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "wait_for_icp=false. Using manual initial pose: x=%.3f y=%.3f yaw=%.3f",
        initial_x_, initial_y_, initial_yaw_
      );
    }

    initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        initial_pose_topic_,
        10,
        std::bind(&FastlioOdomAligner::initialPoseCallback, this, std::placeholders::_1)
      );

    raw_odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
        raw_odom_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&FastlioOdomAligner::odomCallback, this, std::placeholders::_1)
      );

    aligned_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(aligned_odom_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "fastlio_odom_aligner started.");
    RCLCPP_INFO(this->get_logger(), "raw odom topic: %s", raw_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "aligned odom topic: %s", aligned_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "initial pose topic: %s", initial_pose_topic_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "lock_after_first_icp: %s",
      lock_after_first_icp_ ? "true" : "false"
    );
  }

private:
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (lock_after_first_icp_ && has_initial_pose_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Initial ICP pose already locked. Ignoring new ICP pose."
      );
      return;
    }

    initial_x_ = msg->pose.pose.position.x;
    initial_y_ = msg->pose.pose.position.y;
    initial_yaw_ = tf2::getYaw(msg->pose.pose.orientation);

    has_initial_pose_ = true;
    has_raw_odom_origin_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "Received and locked ICP initial pose: x=%.3f y=%.3f yaw=%.3f",
      initial_x_, initial_y_, initial_yaw_
    );
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!has_initial_pose_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Waiting for ICP initial pose..."
      );
      return;
    }

    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;
    const double raw_yaw = tf2::getYaw(msg->pose.pose.orientation);

    if (use_first_odom_as_zero_ && !has_raw_odom_origin_) {
      raw_origin_x_ = raw_x;
      raw_origin_y_ = raw_y;
      raw_origin_yaw_ = raw_yaw;
      has_raw_odom_origin_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Captured FAST-LIO raw odom origin: x=%.3f y=%.3f yaw=%.3f",
        raw_origin_x_, raw_origin_y_, raw_origin_yaw_
      );
    }

    double local_x = raw_x;
    double local_y = raw_y;
    double local_yaw = raw_yaw;

    if (use_first_odom_as_zero_) {
      const double dx = raw_x - raw_origin_x_;
      const double dy = raw_y - raw_origin_y_;

      const double c0 = std::cos(-raw_origin_yaw_);
      const double s0 = std::sin(-raw_origin_yaw_);

      local_x = c0 * dx - s0 * dy;
      local_y = s0 * dx + c0 * dy;
      local_yaw = normalizeAngle(raw_yaw - raw_origin_yaw_);
    }

    const double c = std::cos(initial_yaw_);
    const double s = std::sin(initial_yaw_);

    const double aligned_x = initial_x_ + c * local_x - s * local_y;
    const double aligned_y = initial_y_ + s * local_x + c * local_y;
    const double aligned_yaw = normalizeAngle(initial_yaw_ + local_yaw);

    nav_msgs::msg::Odometry out = *msg;

    out.header.stamp = msg->header.stamp;
    out.header.frame_id = map_frame_;
    out.child_frame_id = base_frame_;

    out.pose.pose.position.x = aligned_x;
    out.pose.pose.position.y = aligned_y;
    out.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, aligned_yaw);
    out.pose.pose.orientation = tf2::toMsg(q);

    rotateTwistToMap(msg, out);

    aligned_odom_pub_->publish(out);
  }

  void rotateTwistToMap(
    const nav_msgs::msg::Odometry::SharedPtr & in,
    nav_msgs::msg::Odometry & out)
  {
    const double c = std::cos(initial_yaw_);
    const double s = std::sin(initial_yaw_);

    const double vx = in->twist.twist.linear.x;
    const double vy = in->twist.twist.linear.y;

    out.twist.twist.linear.x = c * vx - s * vy;
    out.twist.twist.linear.y = s * vx + c * vy;
    out.twist.twist.linear.z = in->twist.twist.linear.z;

    out.twist.twist.angular = in->twist.twist.angular;
  }

  double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }

    return angle;
  }

private:
  std::string raw_odom_topic_;
  std::string aligned_odom_topic_;
  std::string initial_pose_topic_;

  std::string map_frame_;
  std::string base_frame_;

  bool wait_for_icp_{true};
  bool use_first_odom_as_zero_{true};
  bool lock_after_first_icp_{true};

  bool has_initial_pose_{false};
  bool has_raw_odom_origin_{false};

  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};

  double raw_origin_x_{0.0};
  double raw_origin_y_{0.0};
  double raw_origin_yaw_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr aligned_odom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastlioOdomAligner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}