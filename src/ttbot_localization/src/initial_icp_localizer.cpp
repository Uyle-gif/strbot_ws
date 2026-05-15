#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class InitialIcpLocalizer : public rclcpp::Node
{
public:
  InitialIcpLocalizer()
  : Node("initial_icp_localizer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    getParameters();

    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map_cloud_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    if (!loadMap()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load map. Node will not run ICP.");
      map_loaded_ = false;
    } else {
      map_loaded_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Map loaded successfully. Boundary points: %zu, downsampled: %zu",
        map_cloud_->size(),
        map_cloud_downsampled_->size()
      );
    }

    map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/icp/map_points", 10);

    scan_initial_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/icp/scan_points_initial", 10);

    scan_aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/icp/scan_points_aligned", 10);

    icp_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      icp_pose_topic_, 10);

    if (use_initialpose_topic_) {
      initialpose_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          initialpose_topic_,
          10,
          std::bind(&InitialIcpLocalizer::initialPoseCallback, this, std::placeholders::_1)
        );

      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for initial guess from topic: %s",
        initialpose_topic_.c_str()
      );
    } else {
      initial_guess_x_ = param_initial_x_;
      initial_guess_y_ = param_initial_y_;
      initial_guess_yaw_ = param_initial_yaw_;
      has_initial_guess_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Using parameter initial guess: x=%.3f, y=%.3f, yaw=%.3f rad",
        initial_guess_x_,
        initial_guess_y_,
        initial_guess_yaw_
      );
    }

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&InitialIcpLocalizer::scanCallback, this, std::placeholders::_1)
    );

    publish_map_timer_ = this->create_wall_timer(
      1000ms,
      std::bind(&InitialIcpLocalizer::publishMapCloud, this)
    );

    RCLCPP_INFO(this->get_logger(), "initial_icp_localizer started.");
    RCLCPP_INFO(this->get_logger(), "scan topic: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "map frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "ICP retry period: %.3f sec", icp_retry_period_sec_);
  }

private:
  void declareParameters()
  {
    this->declare_parameter<std::string>("map_yaml", "");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("laser_frame", "");

    this->declare_parameter<bool>("use_initialpose_topic", true);
    this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
    this->declare_parameter<std::string>("icp_pose_topic", "/icp_initial_pose");

    this->declare_parameter<double>("initial_x", 0.0);
    this->declare_parameter<double>("initial_y", 0.0);
    this->declare_parameter<double>("initial_yaw", 0.0);

    this->declare_parameter<bool>("run_once", true);
    this->declare_parameter<bool>("publish_debug_clouds", true);

    this->declare_parameter<bool>("extract_boundary_only", true);
    this->declare_parameter<double>("map_downsample_resolution", 0.10);
    this->declare_parameter<double>("scan_downsample_resolution", 0.05);

    this->declare_parameter<double>("min_scan_range", 0.5);
    this->declare_parameter<double>("max_scan_range", 25.0);

    this->declare_parameter<double>("max_correspondence_distance", 1.5);
    this->declare_parameter<int>("maximum_iterations", 80);
    this->declare_parameter<double>("transformation_epsilon", 1e-6);
    this->declare_parameter<double>("euclidean_fitness_epsilon", 1e-5);
    this->declare_parameter<double>("fitness_score_threshold", 35.0);

    // Chống spam log: nếu ICP bị reject, chỉ thử lại sau mỗi N giây
    this->declare_parameter<double>("icp_retry_period_sec", 2.0);

    this->declare_parameter<bool>("allow_tf_transform_scan_to_base", true);

    this->declare_parameter<double>("result_covariance_x", 0.05);
    this->declare_parameter<double>("result_covariance_y", 0.05);
    this->declare_parameter<double>("result_covariance_yaw", 0.03);
  }

  void getParameters()
  {
    this->get_parameter("map_yaml", map_yaml_);
    this->get_parameter("scan_topic", scan_topic_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("laser_frame", laser_frame_);

    this->get_parameter("use_initialpose_topic", use_initialpose_topic_);
    this->get_parameter("initialpose_topic", initialpose_topic_);
    this->get_parameter("icp_pose_topic", icp_pose_topic_);

    this->get_parameter("initial_x", param_initial_x_);
    this->get_parameter("initial_y", param_initial_y_);
    this->get_parameter("initial_yaw", param_initial_yaw_);

    this->get_parameter("run_once", run_once_);
    this->get_parameter("publish_debug_clouds", publish_debug_clouds_);

    this->get_parameter("extract_boundary_only", extract_boundary_only_);
    this->get_parameter("map_downsample_resolution", map_downsample_resolution_);
    this->get_parameter("scan_downsample_resolution", scan_downsample_resolution_);

    this->get_parameter("min_scan_range", min_scan_range_);
    this->get_parameter("max_scan_range", max_scan_range_);

    this->get_parameter("max_correspondence_distance", max_correspondence_distance_);
    this->get_parameter("maximum_iterations", maximum_iterations_);
    this->get_parameter("transformation_epsilon", transformation_epsilon_);
    this->get_parameter("euclidean_fitness_epsilon", euclidean_fitness_epsilon_);
    this->get_parameter("fitness_score_threshold", fitness_score_threshold_);

    this->get_parameter("icp_retry_period_sec", icp_retry_period_sec_);

    this->get_parameter("allow_tf_transform_scan_to_base", allow_tf_transform_scan_to_base_);

    this->get_parameter("result_covariance_x", result_covariance_x_);
    this->get_parameter("result_covariance_y", result_covariance_y_);
    this->get_parameter("result_covariance_yaw", result_covariance_yaw_);
  }

  bool loadMap()
  {
    if (map_yaml_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter map_yaml is empty.");
      return false;
    }

    YAML::Node yaml;
    try {
      yaml = YAML::LoadFile(map_yaml_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
      return false;
    }

    if (!yaml["image"] || !yaml["resolution"] || !yaml["origin"]) {
      RCLCPP_ERROR(this->get_logger(), "Invalid map yaml. Need image, resolution, origin.");
      return false;
    }

    std::string image_path = yaml["image"].as<std::string>();
    map_resolution_ = yaml["resolution"].as<double>();

    auto origin = yaml["origin"];
    map_origin_x_ = origin[0].as<double>();
    map_origin_y_ = origin[1].as<double>();
    map_origin_yaw_ = origin[2].as<double>();

    int negate = 0;
    double occupied_thresh = 0.65;

    if (yaml["negate"]) {
      negate = yaml["negate"].as<int>();
    }

    if (yaml["occupied_thresh"]) {
      occupied_thresh = yaml["occupied_thresh"].as<double>();
    }

    std::string map_dir = getDirectory(map_yaml_);
    std::string full_image_path;

    if (!image_path.empty() && image_path[0] == '/') {
      full_image_path = image_path;
    } else {
      full_image_path = map_dir + "/" + image_path;
    }

    cv::Mat image = cv::imread(full_image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to read map image: %s",
        full_image_path.c_str()
      );
      return false;
    }

    map_height_ = image.rows;
    map_width_ = image.cols;

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded map image: %s, width=%d, height=%d, resolution=%.3f, origin=[%.3f, %.3f, %.3f]",
      full_image_path.c_str(),
      map_width_,
      map_height_,
      map_resolution_,
      map_origin_x_,
      map_origin_y_,
      map_origin_yaw_
    );

    cv::Mat occupied = cv::Mat::zeros(image.size(), CV_8UC1);

    for (int v = 0; v < image.rows; ++v) {
      for (int u = 0; u < image.cols; ++u) {
        const unsigned char pixel = image.at<unsigned char>(v, u);

        double occ_prob;
        if (negate == 0) {
          occ_prob = (255.0 - static_cast<double>(pixel)) / 255.0;
        } else {
          occ_prob = static_cast<double>(pixel) / 255.0;
        }

        if (occ_prob >= occupied_thresh) {
          occupied.at<unsigned char>(v, u) = 255;
        }
      }
    }

    cv::Mat points_mask;

    if (extract_boundary_only_) {
      cv::Mat eroded;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
      cv::erode(occupied, eroded, kernel);
      points_mask = occupied - eroded;
    } else {
      points_mask = occupied;
    }

    map_cloud_->clear();

    for (int v = 0; v < points_mask.rows; ++v) {
      for (int u = 0; u < points_mask.cols; ++u) {
        if (points_mask.at<unsigned char>(v, u) > 0) {
          const auto xy = pixelToMap(u, v);
          pcl::PointXYZ p;
          p.x = static_cast<float>(xy.first);
          p.y = static_cast<float>(xy.second);
          p.z = 0.0f;
          map_cloud_->push_back(p);
        }
      }
    }

    map_cloud_->width = static_cast<uint32_t>(map_cloud_->size());
    map_cloud_->height = 1;
    map_cloud_->is_dense = true;

    if (map_cloud_->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Extracted map cloud is empty.");
      return false;
    }

    downsampleCloud(map_cloud_, map_cloud_downsampled_, map_downsample_resolution_);

    return !map_cloud_downsampled_->empty();
  }

  std::string getDirectory(const std::string & path)
  {
    const auto pos = path.find_last_of("/\\");
    if (pos == std::string::npos) {
      return ".";
    }
    return path.substr(0, pos);
  }

  std::pair<double, double> pixelToMap(int u, int v)
  {
    double x = map_origin_x_ + static_cast<double>(u) * map_resolution_;
    double y = map_origin_y_ + static_cast<double>(map_height_ - v) * map_resolution_;

    if (std::abs(map_origin_yaw_) > 1e-9) {
      const double dx = x - map_origin_x_;
      const double dy = y - map_origin_y_;

      const double c = std::cos(map_origin_yaw_);
      const double s = std::sin(map_origin_yaw_);

      x = map_origin_x_ + c * dx - s * dy;
      y = map_origin_y_ + s * dx + c * dy;
    }

    return {x, y};
  }

  void downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output,
    double leaf_size)
  {
    if (leaf_size <= 0.0) {
      *output = *input;
      return;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(
      static_cast<float>(leaf_size),
      static_cast<float>(leaf_size),
      static_cast<float>(leaf_size)
    );
    voxel.filter(*output);
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    initial_guess_x_ = msg->pose.pose.position.x;
    initial_guess_y_ = msg->pose.pose.position.y;
    initial_guess_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    has_initial_guess_ = true;

    // Cho phép chạy lại ICP nếu trước đó chưa accept.
    // Nếu đã accept và run_once=true thì scanCallback vẫn sẽ return.
    if (!icp_completed_) {
      last_icp_attempt_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received initial guess: x=%.3f, y=%.3f, yaw=%.3f rad",
      initial_guess_x_,
      initial_guess_y_,
      initial_guess_yaw_
    );
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!map_loaded_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Map is not loaded. Skipping scan."
      );
      return;
    }

    if (run_once_ && icp_completed_) {
      return;
    }

    if (!has_initial_guess_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "No initial guess yet. Publish /initialpose or disable use_initialpose_topic."
      );
      return;
    }

    // Throttle ICP attempts để tránh spam log khi ICP bị reject.
    const rclcpp::Time now = this->now();

    if (last_icp_attempt_time_.nanoseconds() > 0) {
      const double dt = (now - last_icp_attempt_time_).seconds();

      if (dt < icp_retry_period_sec_) {
        return;
      }
    }

    last_icp_attempt_time_ = now;

    auto scan_cloud_base = scanToCloudBaseFrame(*msg);
    if (!scan_cloud_base || scan_cloud_base->empty()) {
      RCLCPP_WARN(this->get_logger(), "Scan cloud is empty. Skipping ICP.");
      return;
    }

    auto scan_cloud_downsampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>()
    );
    downsampleCloud(scan_cloud_base, scan_cloud_downsampled, scan_downsample_resolution_);

    if (scan_cloud_downsampled->empty()) {
      RCLCPP_WARN(this->get_logger(), "Downsampled scan cloud is empty. Skipping ICP.");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Running ICP. scan points=%zu, map points=%zu, initial=[%.3f, %.3f, %.3f]",
      scan_cloud_downsampled->size(),
      map_cloud_downsampled_->size(),
      initial_guess_x_,
      initial_guess_y_,
      initial_guess_yaw_
    );

    Eigen::Matrix4f initial_guess = makeTransform(
      initial_guess_x_,
      initial_guess_y_,
      initial_guess_yaw_
    );

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_initial_in_map(
      new pcl::PointCloud<pcl::PointXYZ>()
    );
    pcl::transformPointCloud(*scan_cloud_downsampled, *scan_initial_in_map, initial_guess);

    if (publish_debug_clouds_) {
      publishCloud(scan_initial_pub_, scan_initial_in_map, map_frame_, msg->header.stamp);
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_cloud_downsampled);
    icp.setInputTarget(map_cloud_downsampled_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setMaximumIterations(maximum_iterations_);
    icp.setTransformationEpsilon(transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, initial_guess);

    if (!icp.hasConverged()) {
      RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
      return;
    }

    const double fitness = icp.getFitnessScore();

    Eigen::Matrix4f final_tf = icp.getFinalTransformation();

    const double x = final_tf(0, 3);
    const double y = final_tf(1, 3);
    const double yaw = std::atan2(final_tf(1, 0), final_tf(0, 0));

    RCLCPP_INFO(
      this->get_logger(),
      "ICP converged. fitness=%.6f, pose: x=%.3f, y=%.3f, yaw=%.3f rad",
      fitness,
      x,
      y,
      yaw
    );

    if (fitness > fitness_score_threshold_) {
      RCLCPP_WARN(
        this->get_logger(),
        "ICP fitness %.6f is worse than threshold %.6f. Rejecting result.",
        fitness,
        fitness_score_threshold_
      );
      return;
    }

    auto aligned_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>(aligned)
    );

    if (publish_debug_clouds_) {
      publishCloud(scan_aligned_pub_, aligned_cloud, map_frame_, msg->header.stamp);
    }

    publishIcpPose(x, y, yaw, msg->header.stamp);

    icp_completed_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "ICP result accepted. run_once=%s. Further ICP attempts will stop if run_once=true.",
      run_once_ ? "true" : "false"
    );
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scanToCloudBaseFrame(
    const sensor_msgs::msg::LaserScan & scan)
  {
    auto cloud_laser = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>()
    );

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];

      if (!std::isfinite(r)) {
        continue;
      }

      if (r < min_scan_range_ || r > max_scan_range_) {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

      pcl::PointXYZ p;
      p.x = static_cast<float>(static_cast<double>(r) * std::cos(angle));
      p.y = static_cast<float>(static_cast<double>(r) * std::sin(angle));
      p.z = 0.0f;
      cloud_laser->push_back(p);
    }

    cloud_laser->width = static_cast<uint32_t>(cloud_laser->size());
    cloud_laser->height = 1;
    cloud_laser->is_dense = true;

    const std::string scan_frame =
      laser_frame_.empty() ? scan.header.frame_id : laser_frame_;

    if (scan_frame.empty() || scan_frame == base_frame_) {
      return cloud_laser;
    }

    if (!allow_tf_transform_scan_to_base_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Scan frame is '%s' but TF transform is disabled. Using scan points as base frame.",
        scan_frame.c_str()
      );
      return cloud_laser;
    }

    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer_.lookupTransform(
        base_frame_,
        scan_frame,
        scan.header.stamp,
        rclcpp::Duration::from_seconds(0.1)
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform scan from '%s' to '%s': %s. Using raw scan points.",
        scan_frame.c_str(),
        base_frame_.c_str(),
        ex.what()
      );
      return cloud_laser;
    }

    auto cloud_base = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>()
    );

    const double tx = tf.transform.translation.x;
    const double ty = tf.transform.translation.y;
    const double yaw = tf2::getYaw(tf.transform.rotation);

    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    for (const auto & p : cloud_laser->points) {
      pcl::PointXYZ q;
      q.x = static_cast<float>(tx + c * p.x - s * p.y);
      q.y = static_cast<float>(ty + s * p.x + c * p.y);
      q.z = 0.0f;
      cloud_base->push_back(q);
    }

    cloud_base->width = static_cast<uint32_t>(cloud_base->size());
    cloud_base->height = 1;
    cloud_base->is_dense = true;

    return cloud_base;
  }

  Eigen::Matrix4f makeTransform(double x, double y, double yaw)
  {
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();

    const float c = static_cast<float>(std::cos(yaw));
    const float s = static_cast<float>(std::sin(yaw));

    tf(0, 0) = c;
    tf(0, 1) = -s;
    tf(1, 0) = s;
    tf(1, 1) = c;
    tf(0, 3) = static_cast<float>(x);
    tf(1, 3) = static_cast<float>(y);

    return tf;
  }

  void publishIcpPose(double x, double y, double yaw, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;

    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.pose.orientation = tf2::toMsg(q);

    for (double & value : pose.pose.covariance) {
      value = 0.0;
    }

    pose.pose.covariance[0] = result_covariance_x_;
    pose.pose.covariance[7] = result_covariance_y_;
    pose.pose.covariance[35] = result_covariance_yaw_;

    icp_pose_pub_->publish(pose);

    RCLCPP_INFO(
      this->get_logger(),
      "Published ICP initial pose to %s",
      icp_pose_topic_.c_str()
    );
  }

  void publishCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::string & frame_id,
    const rclcpp::Time & stamp)
  {
    if (!pub || !cloud) {
      return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    pub->publish(msg);
  }

  void publishMapCloud()
  {
    if (!publish_debug_clouds_ || !map_loaded_) {
      return;
    }

    publishCloud(
      map_points_pub_,
      map_cloud_downsampled_,
      map_frame_,
      this->now()
    );
  }

private:
  std::string map_yaml_;
  std::string scan_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string laser_frame_;

  std::string initialpose_topic_;
  std::string icp_pose_topic_;

  bool use_initialpose_topic_{true};
  bool run_once_{true};
  bool publish_debug_clouds_{true};
  bool extract_boundary_only_{true};
  bool allow_tf_transform_scan_to_base_{true};

  double param_initial_x_{0.0};
  double param_initial_y_{0.0};
  double param_initial_yaw_{0.0};

  double initial_guess_x_{0.0};
  double initial_guess_y_{0.0};
  double initial_guess_yaw_{0.0};

  bool has_initial_guess_{false};
  bool icp_completed_{false};
  bool map_loaded_{false};

  double map_resolution_{0.05};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double map_origin_yaw_{0.0};
  int map_width_{0};
  int map_height_{0};

  double map_downsample_resolution_{0.10};
  double scan_downsample_resolution_{0.05};

  double min_scan_range_{0.5};
  double max_scan_range_{25.0};

  double max_correspondence_distance_{1.5};
  int maximum_iterations_{80};
  double transformation_epsilon_{1e-6};
  double euclidean_fitness_epsilon_{1e-5};
  double fitness_score_threshold_{35.0};

  double icp_retry_period_sec_{2.0};
  rclcpp::Time last_icp_attempt_time_{0, 0, RCL_ROS_TIME};

  double result_covariance_x_{0.05};
  double result_covariance_y_{0.05};
  double result_covariance_yaw_{0.03};

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_downsampled_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr icp_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_initial_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_aligned_pub_;

  rclcpp::TimerBase::SharedPtr publish_map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitialIcpLocalizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}