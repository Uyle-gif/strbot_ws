#ifndef TTBOT_CONTROLLER_GMPC_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_GMPC_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/utils.h"

#include <vector>
#include <utility>
#include <algorithm>
#include <memory>
#include <mutex>
#include <cmath>
#include <chrono>
#include <limits>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <osqp/osqp.h>
#include <sophus/se2.hpp>

using Control = std::vector<double>;
using PathVec = std::vector<std::pair<double, double>>;

class GmpcController : public rclcpp::Node
{
public:
    GmpcController();
    ~GmpcController();

private:
    // =========================
    // ROS callbacks
    // =========================
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void mpcTuningCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // =========================
    // GMPC core
    // =========================
    Control solveMPC(double x0, double y0, double yaw0,
                     const PathVec& path, size_t start_idx);

    void linearizeErrorModel(double v_ref, double omega_ref, double dt);

    size_t findClosestPoint(const PathVec& path,
                            size_t& current_index,
                            double x, double y);

    void computeReference(const PathVec& path,
                          size_t idx,
                          double& rx, double& ry, double& rpsi);

    void computeErrorState(double x, double y, double yaw,
                           double rx, double ry, double rpsi,
                           double& ex, double& ey, double& epsi);

    // =========================
    // OSQP helpers
    // =========================
    void freeOSQPMemory();

    void eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                        OSQPCscMatrix& out_mat,
                        OSQPFloat*& out_x,
                        OSQPInt*& out_i,
                        OSQPInt*& out_p);

    void setupSolver(const Eigen::SparseMatrix<double>& P,
                     const Eigen::SparseMatrix<double>& A,
                     const Eigen::VectorXd& l_bounds,
                     const Eigen::VectorXd& u_bounds);

    bool updateSolverData(const Eigen::SparseMatrix<double>& P,
                          const Eigen::SparseMatrix<double>& A,
                          const Eigen::VectorXd& l_bounds,
                          const Eigen::VectorXd& u_bounds);

    // =========================
    // Parameters
    // =========================
    double desired_speed_;
    double wheel_base_;
    double max_steer_;
    double max_omega_;
    double goal_tolerance_;

    int    N_p_;
    double dt_mpc_;

    double Q_ex_;
    double Q_ey_;
    double Q_epsi_;
    double R_v_;
    double R_omega_;
    double R_dv_;
    double R_domega_;

    // =========================
    // Runtime states
    // =========================
    double prev_v_cmd_ = 0.0;
    double prev_omega_cmd_ = 0.0;

    bool reached_goal_ = false;
    bool has_path_ = false;
    bool has_odom_ = false;

    double current_pose_x_ = 0.0;
    double current_pose_y_ = 0.0;
    double current_pose_yaw_ = 0.0;

    // =========================
    // Path snapshot storage
    // =========================
    std::mutex path_mutex_;
    std::shared_ptr<PathVec> path_points_ptr_;
    size_t current_index_ = 0;

    // =========================
    // Linearized model
    // =========================
    Eigen::Matrix3d Ad_;
    Eigen::Matrix<double, 3, 2> Bd_;

    // =========================
    // OSQP cached workspace
    // =========================
    OSQPSolver* solver_ = nullptr;
    OSQPSettings* settings_ = nullptr;

    OSQPFloat* P_x_ = nullptr;
    OSQPInt*   P_i_ = nullptr;
    OSQPInt*   P_p_ = nullptr;

    OSQPFloat* A_x_ = nullptr;
    OSQPInt*   A_i_ = nullptr;
    OSQPInt*   A_p_ = nullptr;

    OSQPFloat* q_data_ = nullptr;
    OSQPFloat* l_data_ = nullptr;
    OSQPFloat* u_data_ = nullptr;

    bool solver_ready_ = false;
    int solver_n_vars_ = 0;
    int solver_n_cons_ = 0;
    OSQPInt solver_P_nnz_ = 0;
    OSQPInt solver_A_nnz_ = 0;

    // =========================
    // ROS interfaces
    // =========================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_tuning_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_lon_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_cte_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_heading_pub_;
};

#endif  // TTBOT_CONTROLLER_GMPC_CONTROLLER_HPP_