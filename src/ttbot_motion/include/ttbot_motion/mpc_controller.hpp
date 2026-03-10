#ifndef TTBOT_MOTION_MPC_CONTROLLER_HPP_
#define TTBOT_MOTION_MPC_CONTROLLER_HPP_

#include <vector>
#include <utility>
#include <algorithm>
#include <string>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2/utils.h"

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <osqp/osqp.h>

namespace ttbot_motion
{

using Control = std::vector<double>; 

class MpcController : public nav2_core::Controller
{
public:
    MpcController() = default;
    ~MpcController() override;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
    void mpcTuningCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);    
    Control solveMPC(double ey0, double epsi0, double v_ref);
    void linearizeErrorModel(double v, double dt);
    size_t findClosestPoint(double x, double y);
    void computeReference(size_t idx, double &rx, double &ry, double &rpsi);
    void computeErrorState(double x, double y, double yaw,
                           double rx, double ry, double rpsi,
                           double &ey, double &epsi);

    void freeOSQPMemory();
    void eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                        OSQPCscMatrix& out_mat,
                        OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p);

    // Nav2 requirements
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_ {rclcpp::get_logger("MpcController")};

    // Parameters
    double desired_speed_;
    double wheel_base_;
    double max_steer_;     
    double max_omega_;     
    int    N_p_;      
    double dt_mpc_;   
    double Q_ey_;
    double Q_epsi_;
    double R_delta_;
    double goal_tolerance_;
    bool reached_goal_;

    std::vector<std::pair<double, double>> path_points_;
    size_t current_index_;
    bool has_path_;

    Eigen::Matrix2d Ad_;
    Eigen::Vector2d Bd_;

    // OSQP
    OSQPSolver* solver_   = nullptr;
    OSQPSettings* settings_ = nullptr;
    OSQPFloat* P_x_ = nullptr; OSQPInt* P_i_ = nullptr; OSQPInt* P_p_ = nullptr;
    OSQPFloat* A_x_ = nullptr; OSQPInt* A_i_ = nullptr; OSQPInt* A_p_ = nullptr;
    OSQPFloat* q_data_ = nullptr;
    OSQPFloat* l_data_ = nullptr;
    OSQPFloat* u_data_ = nullptr;

    // ROS Publishers/Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_tuning_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>> error_cte_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>> error_heading_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>> ackermann_pub_;
};

} // namespace ttbot_motion

#endif // TTBOT_MOTION_MPC_CONTROLLER_HPP_