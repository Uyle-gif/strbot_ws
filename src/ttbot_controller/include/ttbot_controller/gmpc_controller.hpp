#ifndef TTBOT_CONTROLLER_GMPC_CONTROLLER_HPP_
#define TTBOT_CONTROLLER_GMPC_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <iostream>
#include <cmath>
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/utils.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <utility>
#include <algorithm>
#include <string>
#include <memory>
#include <cmath>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <osqp/osqp.h>
#include <sophus/se2.hpp> // <-- 1. THÊM THƯ VIỆN ĐẠI SỐ LIE

using Control = std::vector<double>; 

class GmpcController : public rclcpp::Node // <-- 2. ĐỔI TÊN CLASS
{
public:
    GmpcController();
    ~GmpcController();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void mpcTuningCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);    

    // <-- 3. CẬP NHẬT HÀM SOLVE: thêm ex0 và omega_ref
    Control solveMPC(double ex0, double ey0, double epsi0, double v_ref, double omega_ref);
    
    // <-- 3. CẬP NHẬT HÀM LINEARIZE: thêm omega_ref
    void linearizeErrorModel(double v_ref, double omega_ref, double dt);
    
    size_t findClosestPoint(double x, double y);
    void computeReference(size_t idx, double &rx, double &ry, double &rpsi);
    
    // <-- 3. CẬP NHẬT HÀM LỖI: thêm tham chiếu trả về ex
    void computeErrorState(double x, double y, double yaw,
                           double rx, double ry, double rpsi,
                           double &ex, double &ey, double &epsi);

    void freeOSQPMemory();
    void eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                        OSQPCscMatrix& out_mat,
                        OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p);

    double desired_speed_;
    double wheel_base_;
    double max_steer_;     
    double max_omega_;     
    double prev_v_cmd_ = 0.0;
    double prev_omega_cmd_ = 0.0;

    int    N_p_;      
    double dt_mpc_;
    
    // <-- 4. CẬP NHẬT TRỌNG SỐ COST FUNCTION (Đúng chuẩn bài báo)
    double Q_ex_;     // Phạt lỗi dọc
    double Q_ey_;     // Phạt lỗi ngang
    double Q_epsi_;   // Phạt lỗi góc
    double R_v_;      // Phạt lệnh vận tốc dài
    double R_omega_;  // Phạt lệnh vận tốc góc
    double R_dv_;     // Phạt gia tốc dài (độ mượt ga)
    double R_domega_; // Phạt gia tốc góc (độ mượt vô lăng)

    double goal_tolerance_;
    bool reached_goal_;

    std::vector<std::pair<double, double>> path_points_;
    size_t current_index_;
    bool has_path_;

    // <-- 5. CẬP NHẬT KÍCH THƯỚC MA TRẬN TRẠNG THÁI (3x3 và 3x2)
    Eigen::Matrix3d Ad_;
    Eigen::Matrix<double, 3, 2> Bd_;

    OSQPSolver* solver_   = nullptr;
    OSQPSettings* settings_ = nullptr;
    
    OSQPFloat* P_x_ = nullptr; OSQPInt* P_i_ = nullptr; OSQPInt* P_p_ = nullptr;
    OSQPFloat* A_x_ = nullptr; OSQPInt* A_i_ = nullptr; OSQPInt* A_p_ = nullptr;
    OSQPFloat* q_data_ = nullptr;
    OSQPFloat* l_data_ = nullptr;
    OSQPFloat* u_data_ = nullptr;

    // ==== ROS ====
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mpc_tuning_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_cte_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_heading_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_lon_pub_; // Thêm publisher cho lỗi dọc

    double current_pose_x_ = 0.0;
    double current_pose_y_ = 0.0;
    double current_pose_yaw_ = 0.0;
    bool has_odom_ = false;

    double mpc_ref_x_ = 0.0;
    double mpc_ref_y_ = 0.0;
    double mpc_ref_yaw_ = 0.0;
    bool is_ref_set_ = false; 
};

#endif // TTBOT_CONTROLLER_GMPC_CONTROLLER_HPP_