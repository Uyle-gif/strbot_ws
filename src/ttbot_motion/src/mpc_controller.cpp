#include "ttbot_motion/mpc_controller.hpp"
#include "nav2_util/node_utils.hpp"

namespace ttbot_motion
{

// Helper functions
static double deg2rad(double deg) { return deg * M_PI / 180.0; }
static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

MpcController::~MpcController()
{
    freeOSQPMemory();
}

void MpcController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    plugin_name_ = name;
    costmap_ros_ = costmap_ros;
    auto node = node_.lock();
    logger_ = node->get_logger();

    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".desired_speed", rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_base", rclcpp::ParameterValue(0.65));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_steer_deg", rclcpp::ParameterValue(30.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".N_p", rclcpp::ParameterValue(10));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".dt_mpc", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".Q_ey", rclcpp::ParameterValue(10.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".Q_epsi", rclcpp::ParameterValue(5.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".R_delta", rclcpp::ParameterValue(1.0));

    desired_speed_ = node->get_parameter(plugin_name_ + ".desired_speed").as_double();
    wheel_base_    = node->get_parameter(plugin_name_ + ".wheel_base").as_double();
    double max_steer_deg = node->get_parameter(plugin_name_ + ".max_steer_deg").as_double();
    N_p_    = node->get_parameter(plugin_name_ + ".N_p").as_int();
    dt_mpc_ = node->get_parameter(plugin_name_ + ".dt_mpc").as_double();
    Q_ey_   = node->get_parameter(plugin_name_ + ".Q_ey").as_double();
    Q_epsi_ = node->get_parameter(plugin_name_ + ".Q_epsi").as_double();
    R_delta_= node->get_parameter(plugin_name_ + ".R_delta").as_double();
    goal_tolerance_ = node->get_parameter(plugin_name_ + ".goal_tolerance").as_double();

    max_steer_ = deg2rad(max_steer_deg);
    max_omega_ = (std::abs(desired_speed_) / wheel_base_) * std::tan(max_steer_);

    current_index_ = 0;
    has_path_ = false;
    reached_goal_ = false;

    // Publishers & Subscribers
    mpc_tuning_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/mpc_tuning", 10, std::bind(&MpcController::mpcTuningCallback, this, std::placeholders::_1));
        
    error_cte_pub_ = node->create_publisher<std_msgs::msg::Float32>("/mpc/error/cte", 10);
    error_heading_pub_ = node->create_publisher<std_msgs::msg::Float32>("/mpc/error/heading", 10);
    ackermann_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("/ackermann_controller/cmd_vel", 10);
    RCLCPP_INFO(logger_, "MPC Plugin Configured: N=%d, dt=%.2f, Speed=%.2f", N_p_, dt_mpc_, desired_speed_);
}

void MpcController::cleanup()
{
    error_cte_pub_.reset();
    error_heading_pub_.reset();
    mpc_tuning_sub_.reset();
    ackermann_pub_.reset();
    freeOSQPMemory();
}

void MpcController::activate()
{
    error_cte_pub_->on_activate();
    error_heading_pub_->on_activate();
    ackermann_pub_->on_activate();
}

void MpcController::deactivate()
{
    error_cte_pub_->on_deactivate();
    error_heading_pub_->on_deactivate();
    ackermann_pub_->on_deactivate();
}

void MpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    (void)speed_limit; (void)percentage;
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
    if (path.poses.empty()) return;

    path_points_.clear();
    path_points_.reserve(path.poses.size());

    for (const auto &pose : path.poses) {
        path_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    current_index_ = 0;
    has_path_ = true;
    reached_goal_ = false; 
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & /*velocity*/,
    nav2_core::GoalChecker * /*goal_checker*/)
{
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.header.stamp = node_.lock()->now();

    if (!has_path_ || path_points_.empty() || reached_goal_) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = tf2::getYaw(pose.pose.orientation);
    double v_ref = desired_speed_;

    double dx_g = x - path_points_.back().first;
    double dy_g = y - path_points_.back().second;
    double dist_to_goal = std::sqrt(dx_g*dx_g + dy_g*dy_g);

    bool near_end = current_index_ > (path_points_.size() * 0.9);
    if (dist_to_goal < goal_tolerance_ && near_end) {
        reached_goal_ = true;
        RCLCPP_WARN(logger_, "!!! MPC GOAL REACHED (%.2fm) - STOPPING !!!", dist_to_goal);
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
    }

    size_t idx = findClosestPoint(x, y);
    double rx, ry, rpsi;
    computeReference(idx, rx, ry, rpsi);

    double ey, epsi;
    computeErrorState(x, y, yaw, rx, ry, rpsi, ey, epsi);

    std_msgs::msg::Float32 cte_msg, head_msg;
    cte_msg.data = ey; 
    head_msg.data = rad2deg(epsi); 
    error_cte_pub_->publish(cte_msg);
    error_heading_pub_->publish(head_msg);

    Control u = solveMPC(ey, epsi, v_ref);
    double delta = u[0];

    double omega = (v_ref / wheel_base_) * std::tan(delta);
    omega = std::clamp(omega, -max_omega_, max_omega_);

    cmd_vel.twist.linear.x = v_ref;
    cmd_vel.twist.angular.z = omega;

    ackermann_pub_->publish(cmd_vel);
    return cmd_vel;
}

void MpcController::mpcTuningCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 6) return;
    desired_speed_ = (double)msg->data[0];
    int new_Np     = (int)msg->data[1];
    dt_mpc_        = (double)msg->data[2];
    Q_ey_          = (double)msg->data[3];
    Q_epsi_        = (double)msg->data[4];
    R_delta_       = (double)msg->data[5];

    if (new_Np != N_p_) {
        N_p_ = new_Np;
    }
    max_omega_ = (std::abs(desired_speed_) / wheel_base_) * std::tan(max_steer_);
}

size_t MpcController::findClosestPoint(double x, double y) {
    if (path_points_.empty()) return 0;
    size_t best_idx = current_index_;
    double min_dist_sq = std::numeric_limits<double>::infinity();
    size_t search_end = std::min(current_index_ + 100, path_points_.size());
    if (current_index_ == 0) search_end = path_points_.size(); 
    for (size_t i = current_index_; i < search_end; ++i) {
        double dx = x - path_points_[i].first;
        double dy = y - path_points_[i].second;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    current_index_ = best_idx;
    return best_idx;
}

void MpcController::computeReference(size_t idx, double &rx, double &ry, double &rpsi) {
    rx = path_points_[idx].first;
    ry = path_points_[idx].second;
    if (idx + 1 < path_points_.size()) {
        double dx = path_points_[idx+1].first - rx;
        double dy = path_points_[idx+1].second - ry;
        rpsi = std::atan2(dy, dx);
    } else if (idx > 0) {
        double dx = rx - path_points_[idx-1].first;
        double dy = ry - path_points_[idx-1].second;
        rpsi = std::atan2(dy, dx);
    } else {
        rpsi = 0.0;
    }
}

void MpcController::computeErrorState(double x, double y, double yaw,
                                      double rx, double ry, double rpsi,
                                      double &ey, double &epsi) {
    double dx = x - rx;
    double dy = y - ry;
    ey = -std::sin(rpsi) * dx + std::cos(rpsi) * dy;
    epsi = yaw - rpsi;
    while (epsi > M_PI) epsi -= 2.0*M_PI;
    while (epsi < -M_PI) epsi += 2.0*M_PI;
}

void MpcController::linearizeErrorModel(double v, double dt) {
    Ad_ << 1.0, v * dt, 0.0, 1.0;
    Bd_ << 0.0, (v * dt) / wheel_base_;
}

void MpcController::eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat, OSQPCscMatrix& out_mat, OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p) {
    OSQPInt nnz = mat.nonZeros();
    out_x = (OSQPFloat*)malloc(sizeof(OSQPFloat) * nnz);
    out_i = (OSQPInt*)malloc(sizeof(OSQPInt) * nnz);
    out_p = (OSQPInt*)malloc(sizeof(OSQPInt) * (mat.cols() + 1));
    for (int k = 0; k < nnz; k++) {
        out_x[k] = (OSQPFloat)mat.valuePtr()[k];
        out_i[k] = (OSQPInt)mat.innerIndexPtr()[k];
    }
    for (int k = 0; k < mat.cols() + 1; k++) {
        out_p[k] = (OSQPInt)mat.outerIndexPtr()[k];
    }
    out_mat.m = mat.rows(); out_mat.n = mat.cols();
    out_mat.nzmax = nnz; out_mat.nz = -1; out_mat.owned = 0; 
    out_mat.x = out_x; out_mat.i = out_i; out_mat.p = out_p;
}

void MpcController::freeOSQPMemory() {
    if (solver_) { osqp_cleanup(solver_); solver_ = nullptr; }
    if (settings_) { free(settings_); settings_ = nullptr; }
    if (P_x_) { free(P_x_); P_x_ = nullptr; }
    if (P_i_) { free(P_i_); P_i_ = nullptr; }
    if (P_p_) { free(P_p_); P_p_ = nullptr; }
    if (A_x_) { free(A_x_); A_x_ = nullptr; }
    if (A_i_) { free(A_i_); A_i_ = nullptr; }
    if (A_p_) { free(A_p_); A_p_ = nullptr; }
    if (q_data_) { free(q_data_); q_data_ = nullptr; }
    if (l_data_) { free(l_data_); l_data_ = nullptr; }
    if (u_data_) { free(u_data_); u_data_ = nullptr; }
}

Control MpcController::solveMPC(double ey0, double epsi0, double v_ref) {
    freeOSQPMemory();

    int nx = 2; int nu = 1; int N = N_p_;
    int n_vars = (N + 1) * nx + N * nu;
    int n_eq   = (N + 1) * nx; 
    int n_ineq = N * nu;
    int n_cons = n_eq + n_ineq;

    linearizeErrorModel(v_ref, dt_mpc_);

    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;
    for (int k = 0; k <= N; ++k) {
        int offset = k * nx;
        p_triplets.emplace_back(offset, offset, Q_ey_);
        p_triplets.emplace_back(offset + 1, offset + 1, Q_epsi_);
    }
    int u_start_idx = (N + 1) * nx;
    for (int k = 0; k < N; ++k) {
        int offset = u_start_idx + k * nu;
        p_triplets.emplace_back(offset, offset, R_delta_);
    }
    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    Eigen::SparseMatrix<double> A_cons(n_cons, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    Eigen::VectorXd l = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_cons);

    for (int i = 0; i < nx; ++i) {
        a_triplets.emplace_back(i, i, 1.0);
        l(i) = (i == 0) ? ey0 : epsi0;
        u(i) = (i == 0) ? ey0 : epsi0;
    }

    for (int k = 0; k < N; ++k) {
        int row = nx + k * nx;
        int xk  = k * nx;
        int xk1 = (k + 1) * nx;
        int uk  = u_start_idx + k * nu;

        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nx; ++c) {
                if (std::abs(Ad_(r,c)) > 1e-5) a_triplets.emplace_back(row + r, xk + c, -Ad_(r,c));
            }
        }
        for (int r = 0; r < nx; ++r) a_triplets.emplace_back(row + r, xk1 + r, 1.0);
        for (int r = 0; r < nx; ++r) {
            if (std::abs(Bd_(r)) > 1e-5) a_triplets.emplace_back(row + r, uk, -Bd_(r));
        }
        l(row) = 0.0; u(row) = 0.0;
        l(row+1) = 0.0; u(row+1) = 0.0;
    }

    int ineq_start = n_eq;
    for (int k = 0; k < N; ++k) {
        int row = ineq_start + k;
        int col = u_start_idx + k * nu;
        a_triplets.emplace_back(row, col, 1.0);
        l(row) = -max_steer_;
        u(row) =  max_steer_;
    }

    A_cons.setFromTriplets(a_triplets.begin(), a_triplets.end());

    OSQPCscMatrix P_mat, A_mat;
    eigenToOSQPCsc(P, P_mat, P_x_, P_i_, P_p_);
    eigenToOSQPCsc(A_cons, A_mat, A_x_, A_i_, A_p_);

    q_data_ = (OSQPFloat*)calloc(n_vars, sizeof(OSQPFloat));
    l_data_ = (OSQPFloat*)malloc(n_cons * sizeof(OSQPFloat));
    u_data_ = (OSQPFloat*)malloc(n_cons * sizeof(OSQPFloat));
    for(int i=0; i<n_cons; ++i){ l_data_[i] = (OSQPFloat)l(i); u_data_[i] = (OSQPFloat)u(i); }

    settings_ = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;

    osqp_setup(&solver_, &P_mat, q_data_, &A_mat, l_data_, u_data_, n_cons, n_vars, settings_);
    osqp_solve(solver_);

    double delta_opt = 0.0;
    if (solver_->info->status_val == OSQP_SOLVED) {
        delta_opt = solver_->solution->x[u_start_idx];
    } else {
        RCLCPP_WARN(logger_, "MPC Unsolved: %s", solver_->info->status);
    }
    return {delta_opt};
}

} // namespace ttbot_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ttbot_motion::MpcController, nav2_core::Controller)