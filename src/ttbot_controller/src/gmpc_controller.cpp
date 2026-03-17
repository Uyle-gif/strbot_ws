#include "ttbot_controller/gmpc_controller.hpp"

// ==============================================================================
// 1. CONSTRUCTOR / DESTRUCTOR
// ==============================================================================
GmpcController::GmpcController() : Node("gmpc_controller")
{
    this->declare_parameter("desired_speed", 1.5);
    this->declare_parameter("wheel_base", 0.65);
    this->declare_parameter("max_steer_deg", 30.0);
    this->declare_parameter("goal_tolerance", 0.3);

    this->declare_parameter("N_p", 10);
    this->declare_parameter("dt_mpc", 0.1);

    this->declare_parameter("Q_ex", 10.0);
    this->declare_parameter("Q_ey", 10.0);
    this->declare_parameter("Q_epsi", 5.0);

    this->declare_parameter("R_v", 1.0);
    this->declare_parameter("R_omega", 1.0);
    this->declare_parameter("R_dv", 0.1);
    this->declare_parameter("R_domega", 0.1);

    desired_speed_   = this->get_parameter("desired_speed").as_double();
    wheel_base_      = this->get_parameter("wheel_base").as_double();
    goal_tolerance_  = this->get_parameter("goal_tolerance").as_double();

    double max_steer_deg = this->get_parameter("max_steer_deg").as_double();
    max_steer_ = max_steer_deg * M_PI / 180.0;
    max_omega_ = (std::abs(desired_speed_) / wheel_base_) * std::tan(max_steer_);

    N_p_    = this->get_parameter("N_p").as_int();
    dt_mpc_ = this->get_parameter("dt_mpc").as_double();

    Q_ex_     = this->get_parameter("Q_ex").as_double();
    Q_ey_     = this->get_parameter("Q_ey").as_double();
    Q_epsi_   = this->get_parameter("Q_epsi").as_double();
    R_v_      = this->get_parameter("R_v").as_double();
    R_omega_  = this->get_parameter("R_omega").as_double();
    R_dv_     = this->get_parameter("R_dv").as_double();
    R_domega_ = this->get_parameter("R_domega").as_double();

    path_points_ptr_ = std::make_shared<PathVec>();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mpc_state", 10,
        std::bind(&GmpcController::odomCallback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/mpc_path", 1,
        std::bind(&GmpcController::pathCallback, this, std::placeholders::_1));

    mpc_tuning_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/mpc_tuning", 10,
        std::bind(&GmpcController::mpcTuningCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ackermann_controller/cmd_vel", 10);

    error_lon_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gmpc/error/lon", 10);
    error_cte_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gmpc/error/cte", 10);
    error_heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gmpc/error/heading", 10);

    RCLCPP_INFO(this->get_logger(),
                "GMPC initialized | N=%d | dt=%.2f | v=%.2f | max steer=%.1f deg",
                N_p_, dt_mpc_, desired_speed_, max_steer_deg);
}

GmpcController::~GmpcController()
{
    freeOSQPMemory();
}

// ==============================================================================
// 2. ROS CALLBACKS
// ==============================================================================
void GmpcController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_points_ptr_ = std::make_shared<PathVec>();
        current_index_ = 0;
        has_path_ = false;
        reached_goal_ = false;
        prev_v_cmd_ = 0.0;
        prev_omega_cmd_ = 0.0;
        RCLCPP_WARN(this->get_logger(), "Received empty path.");
        return;
    }

    std::vector<std::pair<double, double>> raw_points;
    raw_points.reserve(msg->poses.size());
    for (const auto& pose : msg->poses) {
        raw_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    // Giữ giống baseline để benchmark fair
    int smooth_passes = 1;
    int window_size   = 1;

    std::vector<std::pair<double, double>> smoothed_points = raw_points;

    for (int pass = 0; pass < smooth_passes; ++pass) {
        std::vector<std::pair<double, double>> temp_points = smoothed_points;

        for (size_t i = 1; i < smoothed_points.size() - 1; ++i) {
            double sum_x = 0.0;
            double sum_y = 0.0;
            int count = 0;

            int start_j = std::max(0, static_cast<int>(i) - window_size);
            int end_j   = std::min(static_cast<int>(smoothed_points.size()) - 1,
                                   static_cast<int>(i) + window_size);

            for (int j = start_j; j <= end_j; ++j) {
                sum_x += smoothed_points[j].first;
                sum_y += smoothed_points[j].second;
                count++;
            }

            temp_points[i].first  = sum_x / count;
            temp_points[i].second = sum_y / count;
        }

        smoothed_points = temp_points;
    }

    auto new_path = std::make_shared<PathVec>(std::move(smoothed_points));

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_points_ptr_ = new_path;
        current_index_ = 0;
        has_path_ = true;
        reached_goal_ = false;
        prev_v_cmd_ = 0.0;
        prev_omega_cmd_ = 0.0;
    }

    // path mới -> reset solver để tránh pattern cũ gây lỗi ngầm
    freeOSQPMemory();

    RCLCPP_INFO(this->get_logger(), "--> GMPC received path: %zu points.", new_path->size());
}

void GmpcController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::shared_ptr<PathVec> path_snapshot;
    size_t current_index_snapshot = 0;
    bool has_path_snapshot = false;
    bool reached_goal_snapshot = false;

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_snapshot = path_points_ptr_;
        current_index_snapshot = current_index_;
        has_path_snapshot = has_path_;
        reached_goal_snapshot = reached_goal_;
    }

    if (reached_goal_snapshot || !has_path_snapshot || !path_snapshot || path_snapshot->empty()) {
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = this->now();
        stop_cmd.header.frame_id = "base_link";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(stop_cmd);
        return;
    }

    const auto& path = *path_snapshot;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);

    current_pose_x_ = x;
    current_pose_y_ = y;
    current_pose_yaw_ = yaw;
    has_odom_ = true;

    size_t idx = findClosestPoint(path, current_index_snapshot, x, y);

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_index_ = current_index_snapshot;
    }

    double dx_g = x - path.back().first;
    double dy_g = y - path.back().second;
    double dist_to_goal = std::sqrt(dx_g * dx_g + dy_g * dy_g);
    bool near_end = idx > static_cast<size_t>(path.size() * 0.9);

    if (dist_to_goal < goal_tolerance_ && near_end) {
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            reached_goal_ = true;
        }

        prev_v_cmd_ = 0.0;
        prev_omega_cmd_ = 0.0;

        RCLCPP_WARN(this->get_logger(), "!!! GMPC GOAL REACHED !!!");

        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = this->now();
        stop_cmd.header.frame_id = "base_link";
        stop_cmd.twist.linear.x = 0.0;
        stop_cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(stop_cmd);
        return;
    }

    Control u = solveMPC(x, y, yaw, path, idx);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = u[0];
    cmd.twist.angular.z = u[1];
    cmd_pub_->publish(cmd);
}

void GmpcController::mpcTuningCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 10) return;

    desired_speed_ = static_cast<double>(msg->data[0]);

    int new_Np = static_cast<int>(msg->data[1]);
    bool horizon_changed = (new_Np != N_p_);
    N_p_ = new_Np;

    dt_mpc_    = static_cast<double>(msg->data[2]);
    Q_ex_      = static_cast<double>(msg->data[3]);
    Q_ey_      = static_cast<double>(msg->data[4]);
    Q_epsi_    = static_cast<double>(msg->data[5]);
    R_v_       = static_cast<double>(msg->data[6]);
    R_omega_   = static_cast<double>(msg->data[7]);
    R_dv_      = static_cast<double>(msg->data[8]);
    R_domega_  = static_cast<double>(msg->data[9]);

    max_omega_ = (std::abs(desired_speed_) / wheel_base_) * std::tan(max_steer_);

    if (horizon_changed) {
        freeOSQPMemory();
        RCLCPP_WARN(this->get_logger(), "GMPC horizon changed -> solver reset.");
    }
}

// ==============================================================================
// 3. CORE GMPC
// ==============================================================================
Control GmpcController::solveMPC(double x0, double y0, double yaw0,
                                 const PathVec& path, size_t start_idx)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    const int nx = 3;        // [ex, ey, epsi]
    const int nu = 2;        // [dv, domega]
    const int N  = N_p_;
    const int nx_aug = nx + nu;

    const int n_vars = (N + 1) * nx_aug + N * nu;
    const int n_cons = (N + 1) * nx_aug + N * nu;

    auto wrapAngle = [](double a) {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    struct RefPoint { double x, y, yaw, v, omega; };
    std::vector<RefPoint> ref_traj(N + 1);

    // -------------------------------------------------------------------------
    // 1. Build reference trajectory
    // -------------------------------------------------------------------------
    double accumulated_s = 0.0;
    size_t curr_idx = start_idx;

    for (int k = 0; k <= N; ++k) {
        double target_s = k * desired_speed_ * dt_mpc_;

        while (curr_idx + 1 < path.size()) {
            double dx = path[curr_idx + 1].first - path[curr_idx].first;
            double dy = path[curr_idx + 1].second - path[curr_idx].second;
            double ds = std::hypot(dx, dy);

            if (accumulated_s + ds >= target_s) break;
            accumulated_s += ds;
            curr_idx++;
        }

        double rx, ry, rpsi;
        computeReference(path, curr_idx, rx, ry, rpsi);

        double v_ref = desired_speed_;
        double omega_ref = 0.0;

        if (curr_idx + 1 < path.size()) {
            double rx2, ry2, rpsi2;
            computeReference(path, curr_idx + 1, rx2, ry2, rpsi2);

            double dx = rx2 - rx;
            double dy = ry2 - ry;
            double ds = std::hypot(dx, dy);

            if (ds > 1e-3) {
                double kappa = wrapAngle(rpsi2 - rpsi) / ds;
                omega_ref = v_ref * kappa;
            }
        }

        if (curr_idx >= path.size() - 2) {
            v_ref = 0.0;
            omega_ref = 0.0;
        }

        omega_ref = std::clamp(omega_ref, -max_omega_, max_omega_);
        ref_traj[k] = {rx, ry, rpsi, v_ref, omega_ref};
    }

    // -------------------------------------------------------------------------
    // 2. Current geometric error on SE(2)
    // -------------------------------------------------------------------------
    double ex0, ey0, epsi0;
    computeErrorState(x0, y0, yaw0,
                      ref_traj[0].x, ref_traj[0].y, ref_traj[0].yaw,
                      ex0, ey0, epsi0);

    std_msgs::msg::Float32 lon_msg, cte_msg, head_msg;
    lon_msg.data = static_cast<float>(ex0);
    cte_msg.data = static_cast<float>(ey0);
    head_msg.data = static_cast<float>(epsi0 * 180.0 / M_PI);
    error_lon_pub_->publish(lon_msg);
    error_cte_pub_->publish(cte_msg);
    error_heading_pub_->publish(head_msg);

    // -------------------------------------------------------------------------
    // 3. Build Hessian P
    // -------------------------------------------------------------------------
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;
    p_triplets.reserve((N + 1) * 5 + N * 2);

    for (int k = 0; k <= N; ++k) {
        const int offset = k * nx_aug;

        p_triplets.emplace_back(offset + 0, offset + 0, Q_ex_);
        p_triplets.emplace_back(offset + 1, offset + 1, Q_ey_);
        p_triplets.emplace_back(offset + 2, offset + 2, Q_epsi_);

        if (k > 0) {
            p_triplets.emplace_back(offset + 3, offset + 3, R_v_);
            p_triplets.emplace_back(offset + 4, offset + 4, R_omega_);
        }
    }

    const int du_start = (N + 1) * nx_aug;
    for (int k = 0; k < N; ++k) {
        const int offset = du_start + k * nu;
        p_triplets.emplace_back(offset + 0, offset + 0, R_dv_);
        p_triplets.emplace_back(offset + 1, offset + 1, R_domega_);
    }

    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // -------------------------------------------------------------------------
    // 4. Build constraints A, l, u
    // -------------------------------------------------------------------------
    Eigen::SparseMatrix<double> A_cons(n_cons, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    a_triplets.reserve((N + 1) * nx_aug + N * (nx_aug * nx_aug + nx_aug + nx_aug * nu) + N * nu);

    Eigen::VectorXd l_bounds = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd u_bounds = Eigen::VectorXd::Zero(n_cons);

    const double v_tilde_prev     = prev_v_cmd_     - ref_traj[0].v;
    const double omega_tilde_prev = prev_omega_cmd_ - ref_traj[0].omega;

    // initial augmented state
    for (int i = 0; i < nx_aug; ++i) {
        a_triplets.emplace_back(i, i, 1.0);
    }
    l_bounds.segment(0, nx_aug) << ex0, ey0, epsi0, v_tilde_prev, omega_tilde_prev;
    u_bounds.segment(0, nx_aug) << ex0, ey0, epsi0, v_tilde_prev, omega_tilde_prev;

    // dynamics
    for (int k = 0; k < N; ++k) {
        linearizeErrorModel(ref_traj[k].v, ref_traj[k].omega, dt_mpc_);

        Eigen::MatrixXd A_bar = Eigen::MatrixXd::Zero(nx_aug, nx_aug);
        A_bar.block(0, 0, nx, nx) = Ad_;
        A_bar.block(0, nx, nx, nu) = Bd_;
        A_bar.block(nx, nx, nu, nu) = Eigen::Matrix2d::Identity();

        Eigen::MatrixXd B_bar = Eigen::MatrixXd::Zero(nx_aug, nu);
        B_bar.block(0, 0, nx, nu) = Bd_;
        B_bar.block(nx, 0, nu, nu) = Eigen::Matrix2d::Identity();

        const int row_offset = nx_aug + k * nx_aug;
        const int xk_offset = k * nx_aug;
        const int xk1_offset = (k + 1) * nx_aug;
        const int duk_offset = du_start + k * nu;

        for (int r = 0; r < nx_aug; ++r) {
            for (int c = 0; c < nx_aug; ++c) {
                double val = -A_bar(r, c);
                if (std::abs(val) < 1e-12) val = 1e-12;  // giữ sparsity pattern ổn định hơn
                a_triplets.emplace_back(row_offset + r, xk_offset + c, val);
            }
        }

        for (int i = 0; i < nx_aug; ++i) {
            a_triplets.emplace_back(row_offset + i, xk1_offset + i, 1.0);
        }

        for (int r = 0; r < nx_aug; ++r) {
            for (int c = 0; c < nu; ++c) {
                double val = -B_bar(r, c);
                if (std::abs(val) < 1e-12) val = 1e-12;
                a_triplets.emplace_back(row_offset + r, duk_offset + c, val);
            }
        }
    }

    // bounds on actual command states
    const int input_cons_start = (N + 1) * nx_aug;
    for (int k = 0; k < N; ++k) {
        const int row = input_cons_start + k * nu;
        const int col = (k + 1) * nx_aug + nx;

        a_triplets.emplace_back(row + 0, col + 0, 1.0);
        a_triplets.emplace_back(row + 1, col + 1, 1.0);

        l_bounds(row + 0) = 0.0 - ref_traj[k].v;
        u_bounds(row + 0) = desired_speed_ - ref_traj[k].v;

        l_bounds(row + 1) = -max_omega_ - ref_traj[k].omega;
        u_bounds(row + 1) =  max_omega_ - ref_traj[k].omega;
    }

    A_cons.setFromTriplets(a_triplets.begin(), a_triplets.end());
    P.makeCompressed();
    A_cons.makeCompressed();

    // -------------------------------------------------------------------------
    // 5. Persistent OSQP workspace
    // -------------------------------------------------------------------------
    if (!solver_ready_ ||
        solver_n_vars_ != n_vars ||
        solver_n_cons_ != n_cons ||
        !updateSolverData(P, A_cons, l_bounds, u_bounds))
    {
        setupSolver(P, A_cons, l_bounds, u_bounds);
    }

    if (!solver_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "GMPC solver is null. Fallback to reference input.");
        return {ref_traj[0].v, ref_traj[0].omega};
    }

    osqp_solve(solver_);

    // -------------------------------------------------------------------------
    // 6. Extract control
    // -------------------------------------------------------------------------
    double v_opt = ref_traj[0].v;
    double omega_opt = ref_traj[0].omega;

    if (solver_->info && solver_->info->status_val == OSQP_SOLVED) {
        const int first_utilde_offset = nx_aug + nx;
        const double dv_opt     = solver_->solution->x[first_utilde_offset + 0];
        const double domega_opt = solver_->solution->x[first_utilde_offset + 1];

        v_opt     = std::clamp(ref_traj[0].v + dv_opt, 0.0, desired_speed_);
        omega_opt = std::clamp(ref_traj[0].omega + domega_opt, -max_omega_, max_omega_);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "GMPC OSQP failed, fallback to reference input.");
    }

    prev_v_cmd_ = v_opt;
    prev_omega_cmd_ = omega_opt;

    auto end_time = std::chrono::high_resolution_clock::now();
    double solver_time_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time).count();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[GMPC] solve time: %.3f ms",
        solver_time_ms
    );

    return {v_opt, omega_opt};
}

// ==============================================================================
// 4. GMPC MODEL / GEOMETRIC ERROR
// ==============================================================================
void GmpcController::linearizeErrorModel(double v_ref, double omega_ref, double dt)
{
    Eigen::Matrix3d A_c;
    A_c <<  0.0,        omega_ref, 0.0,
            -omega_ref, 0.0,       v_ref,
             0.0,       0.0,       0.0;

    Eigen::Matrix<double, 3, 2> B_c;
    B_c << 1.0, 0.0,
           0.0, 0.0,
           0.0, 1.0;

    Ad_ = Eigen::Matrix3d::Identity() + A_c * dt;
    Bd_ = B_c * dt;
}

void GmpcController::computeErrorState(double x, double y, double yaw,
                                       double rx, double ry, double rpsi,
                                       double& ex, double& ey, double& epsi)
{
    Sophus::SE2d T_curr(Sophus::SO2d(yaw),  Eigen::Vector2d(x,  y));
    Sophus::SE2d T_ref (Sophus::SO2d(rpsi), Eigen::Vector2d(rx, ry));

    Sophus::SE2d T_err = T_ref.inverse() * T_curr;
    Eigen::Vector3d xi = T_err.log();

    ex   = xi[0];
    ey   = xi[1];
    epsi = xi[2];
}

// ==============================================================================
// 5. PATH HELPERS
// ==============================================================================
size_t GmpcController::findClosestPoint(const PathVec& path,
                                        size_t& current_index,
                                        double x, double y)
{
    if (path.empty()) return 0;

    size_t best_idx = std::min(current_index, path.size() - 1);
    double min_dist_sq = std::numeric_limits<double>::infinity();

    size_t search_end = std::min(current_index + 100, path.size());
    if (current_index == 0) search_end = path.size();

    for (size_t i = current_index; i < search_end; ++i) {
        double dx = x - path[i].first;
        double dy = y - path[i].second;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_idx = i;
        }
    }

    current_index = best_idx;
    return best_idx;
}

void GmpcController::computeReference(const PathVec& path,
                                      size_t idx,
                                      double& rx, double& ry, double& rpsi)
{
    if (path.empty()) {
        rx = 0.0;
        ry = 0.0;
        rpsi = 0.0;
        return;
    }

    idx = std::min(idx, path.size() - 1);
    rx = path[idx].first;
    ry = path[idx].second;

    if (idx + 1 < path.size()) {
        double dx = path[idx + 1].first - rx;
        double dy = path[idx + 1].second - ry;

        if (std::hypot(dx, dy) > 1e-4) {
            rpsi = std::atan2(dy, dx);
        } else if (idx > 0) {
            rpsi = std::atan2(ry - path[idx - 1].second,
                              rx - path[idx - 1].first);
        } else {
            rpsi = 0.0;
        }
    } else if (idx > 0) {
        double dx = rx - path[idx - 1].first;
        double dy = ry - path[idx - 1].second;
        rpsi = std::atan2(dy, dx);
    } else {
        rpsi = 0.0;
    }
}

// ==============================================================================
// 6. OSQP MEMORY / UPDATE
// ==============================================================================
void GmpcController::freeOSQPMemory()
{
    if (solver_)   { osqp_cleanup(solver_); solver_ = nullptr; }
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

    solver_ready_ = false;
    solver_n_vars_ = 0;
    solver_n_cons_ = 0;
    solver_P_nnz_ = 0;
    solver_A_nnz_ = 0;
}

void GmpcController::eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                                    OSQPCscMatrix& out_mat,
                                    OSQPFloat*& out_x,
                                    OSQPInt*& out_i,
                                    OSQPInt*& out_p)
{
    Eigen::SparseMatrix<double> compressed = mat;
    compressed.makeCompressed();

    OSQPInt nnz = static_cast<OSQPInt>(compressed.nonZeros());

    out_x = (OSQPFloat*)malloc(sizeof(OSQPFloat) * nnz);
    out_i = (OSQPInt*)malloc(sizeof(OSQPInt) * nnz);
    out_p = (OSQPInt*)malloc(sizeof(OSQPInt) * (compressed.cols() + 1));

    for (OSQPInt k = 0; k < nnz; ++k) {
        out_x[k] = static_cast<OSQPFloat>(compressed.valuePtr()[k]);
        out_i[k] = static_cast<OSQPInt>(compressed.innerIndexPtr()[k]);
    }

    for (int k = 0; k < compressed.cols() + 1; ++k) {
        out_p[k] = static_cast<OSQPInt>(compressed.outerIndexPtr()[k]);
    }

    out_mat.m = compressed.rows();
    out_mat.n = compressed.cols();
    out_mat.nzmax = nnz;
    out_mat.nz = -1;
    out_mat.owned = 0;
    out_mat.x = out_x;
    out_mat.i = out_i;
    out_mat.p = out_p;
}

void GmpcController::setupSolver(const Eigen::SparseMatrix<double>& P,
                                 const Eigen::SparseMatrix<double>& A,
                                 const Eigen::VectorXd& l_bounds,
                                 const Eigen::VectorXd& u_bounds)
{
    freeOSQPMemory();

    Eigen::SparseMatrix<double> P_comp = P;
    Eigen::SparseMatrix<double> A_comp = A;
    P_comp.makeCompressed();
    A_comp.makeCompressed();

    OSQPCscMatrix P_osqp, A_osqp;
    eigenToOSQPCsc(P_comp, P_osqp, P_x_, P_i_, P_p_);
    eigenToOSQPCsc(A_comp, A_osqp, A_x_, A_i_, A_p_);

    q_data_ = (OSQPFloat*)calloc(P_comp.cols(), sizeof(OSQPFloat));

    l_data_ = (OSQPFloat*)malloc(l_bounds.size() * sizeof(OSQPFloat));
    u_data_ = (OSQPFloat*)malloc(u_bounds.size() * sizeof(OSQPFloat));

    for (int i = 0; i < l_bounds.size(); ++i) {
        l_data_[i] = static_cast<OSQPFloat>(l_bounds(i));
        u_data_[i] = static_cast<OSQPFloat>(u_bounds(i));
    }

    settings_ = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;
    settings_->warm_starting = 1;
    settings_->polishing = 0;

    OSQPInt exitflag = osqp_setup(&solver_, &P_osqp, q_data_, &A_osqp, l_data_, u_data_,
                                  A_comp.rows(), P_comp.cols(), settings_);

    if (exitflag != 0 || !solver_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to setup OSQP solver. exitflag=%d", (int)exitflag);
        return;
    }

    solver_ready_ = true;
    solver_n_vars_ = P_comp.cols();
    solver_n_cons_ = A_comp.rows();
    solver_P_nnz_ = static_cast<OSQPInt>(P_comp.nonZeros());
    solver_A_nnz_ = static_cast<OSQPInt>(A_comp.nonZeros());
}

bool GmpcController::updateSolverData(const Eigen::SparseMatrix<double>& P,
                                      const Eigen::SparseMatrix<double>& A,
                                      const Eigen::VectorXd& l_bounds,
                                      const Eigen::VectorXd& u_bounds)
{
    if (!solver_ready_ || !solver_) return false;

    Eigen::SparseMatrix<double> P_comp = P;
    Eigen::SparseMatrix<double> A_comp = A;
    P_comp.makeCompressed();
    A_comp.makeCompressed();

    if (P_comp.cols() != solver_n_vars_ || A_comp.rows() != solver_n_cons_) {
        return false;
    }

    if (static_cast<OSQPInt>(P_comp.nonZeros()) != solver_P_nnz_ ||
        static_cast<OSQPInt>(A_comp.nonZeros()) != solver_A_nnz_) {
        return false;
    }

    OSQPCscMatrix P_tmp, A_tmp;
    OSQPFloat *Px = nullptr, *Ax = nullptr;
    OSQPInt *Pi = nullptr, *Pp = nullptr;
    OSQPInt *Ai = nullptr, *Ap = nullptr;

    eigenToOSQPCsc(P_comp, P_tmp, Px, Pi, Pp);
    eigenToOSQPCsc(A_comp, A_tmp, Ax, Ai, Ap);

    if (!q_data_ || !l_data_ || !u_data_) {
        free(Px); free(Pi); free(Pp);
        free(Ax); free(Ai); free(Ap);
        return false;
    }

    for (int i = 0; i < solver_n_vars_; ++i) {
        q_data_[i] = 0.0;
    }

    for (int i = 0; i < l_bounds.size(); ++i) {
        l_data_[i] = static_cast<OSQPFloat>(l_bounds(i));
        u_data_[i] = static_cast<OSQPFloat>(u_bounds(i));
    }

    // OSQP 1.x style update
    OSQPInt status = osqp_update_data_mat(
        solver_,
        Px, nullptr, solver_P_nnz_,
        Ax, nullptr, solver_A_nnz_);

    if (status != 0) {
        free(Px); free(Pi); free(Pp);
        free(Ax); free(Ai); free(Ap);
        return false;
    }

    status = osqp_update_data_vec(
        solver_,
        q_data_,
        l_data_,
        u_data_);

    free(Px); free(Pi); free(Pp);
    free(Ax); free(Ai); free(Ap);

    return status == 0;
}

// ==============================================================================
// 7. MAIN
// ==============================================================================
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GmpcController>());
    rclcpp::shutdown();
    return 0;
}