    #include "ttbot_controller/gmpc_controller.hpp"

    // ==============================================================================
    // 1. CONSTRUCTOR & DESTRUCTOR
    // ==============================================================================
    GmpcController::GmpcController() : Node("gmpc_controller")
    {
        // --- Khai báo Parameters (Kế thừa từ MPC cũ và thêm mới cho GMPC) ---
        this->declare_parameter("desired_speed", 1.5);
        this->declare_parameter("wheel_base", 0.65);
        this->declare_parameter("max_steer_deg", 30.0);
        this->declare_parameter("goal_tolerance", 0.3);

        this->declare_parameter("N_p", 10);
        this->declare_parameter("dt_mpc", 0.1);

        // Trọng số trạng thái (State Weights) - Thêm Q_ex cho lỗi dọc
        this->declare_parameter("Q_ex", 10.0);    
        this->declare_parameter("Q_ey", 10.0);
        this->declare_parameter("Q_epsi", 5.0);

        // Trọng số điều khiển (Control Weights) - Tối ưu cả v và omega
        this->declare_parameter("R_v", 1.0);
        this->declare_parameter("R_omega", 1.0);
        
        // Trọng số tốc độ thay đổi (Rate Weights) - Đảm bảo mượt mà (Smoothness)
        this->declare_parameter("R_dv", 0.1);
        this->declare_parameter("R_domega", 0.1);

        // --- Lấy giá trị từ Parameters ---
        desired_speed_ = this->get_parameter("desired_speed").as_double();
        wheel_base_    = this->get_parameter("wheel_base").as_double();
        double max_steer_deg = this->get_parameter("max_steer_deg").as_double();
        
        N_p_    = this->get_parameter("N_p").as_int();
        dt_mpc_ = this->get_parameter("dt_mpc").as_double();
        
        Q_ex_     = this->get_parameter("Q_ex").as_double();
        Q_ey_     = this->get_parameter("Q_ey").as_double();
        Q_epsi_   = this->get_parameter("Q_epsi").as_double();
        R_v_      = this->get_parameter("R_v").as_double();
        R_omega_  = this->get_parameter("R_omega").as_double();
        R_dv_     = this->get_parameter("R_dv").as_double();
        R_domega_ = this->get_parameter("R_domega").as_double();

        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        // Tính toán giới hạn vật lý
        max_steer_ = max_steer_deg * M_PI / 180.0;
        // Max omega dựa trên mô hình hình học xe đạp (Ackermann) [cite: 143-145]
        max_omega_ = (std::abs(desired_speed_) / wheel_base_) * std::tan(max_steer_);

        current_index_ = 0;
        has_path_ = false;
        reached_goal_ = false;
        has_odom_ = false;
        is_ref_set_ = false;

        current_pose_x_ = 0.0;
        current_pose_y_ = 0.0;
        current_pose_yaw_ = 0.0;

        mpc_ref_x_ = 0.0;
        mpc_ref_y_ = 0.0;
        mpc_ref_yaw_ = 0.0;

        // --- Khởi tạo Subscribers (Giữ nguyên topics cũ của hệ thống) ---
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&GmpcController::odomCallback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/mpc_path", 1,
            std::bind(&GmpcController::pathCallback, this, std::placeholders::_1));

        mpc_tuning_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/mpc_tuning", 10,
            std::bind(&GmpcController::mpcTuningCallback, this, std::placeholders::_1));

        // --- Khởi tạo Publishers ---
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/ackermann_controller/cmd_vel", 10);
            
        // Publisher cho các loại lỗi (Phục vụ việc so sánh trong bài báo)
        error_lon_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gmpc/error/lon", 10);
        error_cte_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gmpc/error/cte", 10);
        error_heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gmpc/error/heading", 10);

        RCLCPP_INFO(this->get_logger(), 
            "GMPC Controller Initialized. Mode: SE(2) Manifold. N_p: %d", N_p_);
    }

    GmpcController::~GmpcController()
    {
        freeOSQPMemory();
    }

    // ==============================================================================
    // 2. ROS CALLBACKS
    // ==============================================================================
void GmpcController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (reached_goal_ || !has_path_ || path_points_.empty()) {
        geometry_msgs::msg::TwistStamped stop_cmd;
        stop_cmd.header.stamp = this->now();
        stop_cmd.header.frame_id = "base_link";
        cmd_pub_->publish(stop_cmd);
        return;
    }

    auto wrapAngle = [](double a) {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);

    current_pose_x_ = x;
    current_pose_y_ = y;
    current_pose_yaw_ = yaw;
    has_odom_ = true;

    size_t idx = findClosestPoint(x, y);

    double rx, ry, rpsi;
    computeReference(idx, rx, ry, rpsi);

    mpc_ref_x_ = rx;
    mpc_ref_y_ = ry;
    mpc_ref_yaw_ = rpsi;
    is_ref_set_ = true;

    double dx_g = x - path_points_.back().first;
    double dy_g = y - path_points_.back().second;
    double dist_to_goal = std::sqrt(dx_g * dx_g + dy_g * dy_g);
    bool near_end = idx > static_cast<size_t>(path_points_.size() * 0.9);

    if (dist_to_goal < goal_tolerance_ && near_end) {
        reached_goal_ = true;
        prev_v_cmd_ = 0.0;
        prev_omega_cmd_ = 0.0;
        RCLCPP_WARN(this->get_logger(), "!!! GMPC GOAL REACHED !!!");
        return;
    }

    double ex, ey, epsi;
    computeErrorState(x, y, yaw, rx, ry, rpsi, ex, ey, epsi);

    std_msgs::msg::Float32 lon_msg, cte_msg, head_msg;
    lon_msg.data = ex;
    cte_msg.data = ey;
    head_msg.data = epsi * 180.0 / M_PI;
    error_lon_pub_->publish(lon_msg);
    error_cte_pub_->publish(cte_msg);
    error_heading_pub_->publish(head_msg);

    double v_ref = desired_speed_;
    double omega_ref = 0.0;

    if (idx + 1 < path_points_.size()) {
        double rx2, ry2, rpsi2;
        computeReference(std::min(idx + 1, path_points_.size() - 1), rx2, ry2, rpsi2);
        omega_ref = wrapAngle(rpsi2 - rpsi) / dt_mpc_;
        omega_ref = std::clamp(omega_ref, -max_omega_, max_omega_);
    }

    Control u = solveMPC(ex, ey, epsi, v_ref, omega_ref);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = u[0];
    cmd.twist.angular.z = u[1];
    cmd_pub_->publish(cmd);
}

    void GmpcController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_points_.clear();

        if (msg->poses.empty()) {
            has_path_ = false;
            current_index_ = 0;
            RCLCPP_WARN(this->get_logger(), "Received empty path.");
            return;
        }

        path_points_.reserve(msg->poses.size());
        for (const auto &pose : msg->poses) {
            path_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }

        current_index_ = 0;
        has_path_ = true;
        reached_goal_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "--> GMPC RECEIVED NEW PATH: %zu points.", path_points_.size());
    }

    void GmpcController::mpcTuningCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        (void)msg;
        // TODO: Cập nhật trọng số MPC online
    }

    // ==============================================================================
    // 3. TOÁN HỌC GMPC & LÕI TỐI ƯU
    // ==============================================================================
 Control GmpcController::solveMPC(double ex0, double ey0, double epsi0,
                                 double v_ref, double omega_ref)
{
    freeOSQPMemory();

    const int nx = 3;          // [ex, ey, epsi]
    const int nu = 2;          // [dv, domega] hoặc u_tilde
    const int N  = N_p_;
    const int nx_aug = nx + nu;  // [xi, u_tilde_prev]

    const int n_vars = (N + 1) * nx_aug + N * nu;
    const int n_cons = (N + 1) * nx_aug + N * nu;

    linearizeErrorModel(v_ref, omega_ref, dt_mpc_);

    // x_aug(k+1) = A_bar * x_aug(k) + B_bar * delta_u_tilde(k)
    Eigen::MatrixXd A_bar = Eigen::MatrixXd::Zero(nx_aug, nx_aug);
    A_bar.block(0, 0, nx, nx) = Ad_;
    A_bar.block(0, nx, nx, nu) = Bd_;
    A_bar.block(nx, nx, nu, nu) = Eigen::Matrix2d::Identity();

    Eigen::MatrixXd B_bar = Eigen::MatrixXd::Zero(nx_aug, nu);
    B_bar.block(0, 0, nx, nu) = Bd_;
    B_bar.block(nx, 0, nu, nu) = Eigen::Matrix2d::Identity();

    // -------------------------------------------------------------------------
    // 1) Hessian P
    // -------------------------------------------------------------------------
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;

    // Phạt error state cho mọi k
    for (int k = 0; k <= N; ++k) {
        const int offset = k * nx_aug;
        p_triplets.emplace_back(offset + 0, offset + 0, Q_ex_);
        p_triplets.emplace_back(offset + 1, offset + 1, Q_ey_);
        p_triplets.emplace_back(offset + 2, offset + 2, Q_epsi_);

        // Phạt u_tilde_k nằm trong x_aug(k), nhưng bỏ qua k=0 vì đó là state cố định
        if (k > 0) {
            p_triplets.emplace_back(offset + 3, offset + 3, R_v_);
            p_triplets.emplace_back(offset + 4, offset + 4, R_omega_);
        }
    }

    // Phạt delta_u_tilde
    const int du_start = (N + 1) * nx_aug;
    for (int k = 0; k < N; ++k) {
        const int offset = du_start + k * nu;
        p_triplets.emplace_back(offset + 0, offset + 0, R_dv_);
        p_triplets.emplace_back(offset + 1, offset + 1, R_domega_);
    }

    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // -------------------------------------------------------------------------
    // 2) Constraint matrix A_cons
    // -------------------------------------------------------------------------
    Eigen::SparseMatrix<double> A_cons(n_cons, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    Eigen::VectorXd l_bounds = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd u_bounds = Eigen::VectorXd::Zero(n_cons);

    // x_aug(0) = [xi0, u_tilde_prev]
    const double v_tilde_prev = prev_v_cmd_ - v_ref;
    const double omega_tilde_prev = prev_omega_cmd_ - omega_ref;

    for (int i = 0; i < nx_aug; ++i) {
        a_triplets.emplace_back(i, i, 1.0);
    }

    l_bounds.segment(0, nx_aug) << ex0, ey0, epsi0, v_tilde_prev, omega_tilde_prev;
    u_bounds.segment(0, nx_aug) << ex0, ey0, epsi0, v_tilde_prev, omega_tilde_prev;

    // Dynamics:
    // x_aug(k+1) - A_bar*x_aug(k) - B_bar*delta_u_tilde(k) = 0
    for (int k = 0; k < N; ++k) {
        const int row_offset = nx_aug + k * nx_aug;
        const int xk_offset = k * nx_aug;
        const int xk1_offset = (k + 1) * nx_aug;
        const int duk_offset = du_start + k * nu;

        for (int r = 0; r < nx_aug; ++r) {
            for (int c = 0; c < nx_aug; ++c) {
                if (std::abs(A_bar(r, c)) > 1e-12) {
                    a_triplets.emplace_back(row_offset + r, xk_offset + c, -A_bar(r, c));
                }
            }
        }

        for (int i = 0; i < nx_aug; ++i) {
            a_triplets.emplace_back(row_offset + i, xk1_offset + i, 1.0);
        }

        for (int r = 0; r < nx_aug; ++r) {
            for (int c = 0; c < nu; ++c) {
                if (std::abs(B_bar(r, c)) > 1e-12) {
                    a_triplets.emplace_back(row_offset + r, duk_offset + c, -B_bar(r, c));
                }
            }
        }
    }

    // Input bounds đặt trên u_tilde_k = x_aug(k+1).tail(2)
    const int input_cons_start = (N + 1) * nx_aug;
    for (int k = 0; k < N; ++k) {
        const int row = input_cons_start + k * nu;
        const int col = (k + 1) * nx_aug + nx;

        a_triplets.emplace_back(row + 0, col + 0, 1.0);
        a_triplets.emplace_back(row + 1, col + 1, 1.0);

        // Bound của u_tilde = u - u_ref
        l_bounds(row + 0) = 0.0 - v_ref;
        u_bounds(row + 0) = desired_speed_ - v_ref;

        l_bounds(row + 1) = -max_omega_ - omega_ref;
        u_bounds(row + 1) =  max_omega_ - omega_ref;
    }

    A_cons.setFromTriplets(a_triplets.begin(), a_triplets.end());

    // -------------------------------------------------------------------------
    // 3) Convert to OSQP CSC
    // -------------------------------------------------------------------------
    OSQPCscMatrix P_osqp, A_osqp;
    eigenToOSQPCsc(P, P_osqp, P_x_, P_i_, P_p_);
    eigenToOSQPCsc(A_cons, A_osqp, A_x_, A_i_, A_p_);

    q_data_ = (OSQPFloat*)calloc(n_vars, sizeof(OSQPFloat));
    l_data_ = (OSQPFloat*)malloc(n_cons * sizeof(OSQPFloat));
    u_data_ = (OSQPFloat*)malloc(n_cons * sizeof(OSQPFloat));

    for (int i = 0; i < n_cons; ++i) {
        l_data_[i] = static_cast<OSQPFloat>(l_bounds(i));
        u_data_[i] = static_cast<OSQPFloat>(u_bounds(i));
    }

    // -------------------------------------------------------------------------
    // 4) Solve
    // -------------------------------------------------------------------------
    settings_ = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;
    settings_->warm_starting = 1;

    osqp_setup(&solver_, &P_osqp, q_data_, &A_osqp, l_data_, u_data_,
               n_cons, n_vars, settings_);
    osqp_solve(solver_);

    double v_opt = v_ref;
    double omega_opt = omega_ref;

    if (solver_ && solver_->info && solver_->info->status_val == OSQP_SOLVED) {
        // x_aug(1) chứa u_tilde_0
        const int first_utilde_offset = nx_aug + nx;
        const double dv_opt = solver_->solution->x[first_utilde_offset + 0];
        const double domega_opt = solver_->solution->x[first_utilde_offset + 1];

        v_opt = std::clamp(v_ref + dv_opt, 0.0, desired_speed_);
        omega_opt = std::clamp(omega_ref + domega_opt, -max_omega_, max_omega_);
    } else {
        RCLCPP_WARN(this->get_logger(), "GMPC OSQP failed, fallback to reference input.");
    }

    prev_v_cmd_ = v_opt;
    prev_omega_cmd_ = omega_opt;

    return {v_opt, omega_opt};
}

    void GmpcController::linearizeErrorModel(double v_ref, double omega_ref, double dt)
    {
        // 1. Khởi tạo ma trận A liên tục (Continuous-time) dùng adm(zeta_ref) [cite: 236]
        // Với xe không trượt, v_y = 0 nên ma trận đơn giản hóa như sau:
        Eigen::Matrix3d A_c;
        A_c <<  0.0,         omega_ref,  0.0,
            -omega_ref,   0.0,        v_ref,
                0.0,         0.0,        0.0;

        // 2. Ma trận B liên tục (Mapping từ [v, omega] sang 3 biến trạng thái) [cite: 169, 239]
        Eigen::Matrix<double, 3, 2> B_c;
        B_c << 1.0, 0.0,
            0.0, 0.0,
            0.0, 1.0;

        // 3. Rời rạc hóa bằng phương pháp Euler [cite: 306]
        // Ad = I + Ac * dt
        Ad_ = Eigen::Matrix3d::Identity() + A_c * dt;
        
        // Bd = Bc * dt
        Bd_ = B_c * dt;
    }

    void GmpcController::computeErrorState(double x, double y, double yaw,
                                        double rx, double ry, double rpsi,
                                        double &ex, double &ey, double &epsi)
    {
        // 1. Chuyển đổi Pose hiện tại và Pose tham chiếu sang SE(2) bằng Sophus [cite: 118]
        // Sophus::SE2d nhận vào (góc_heading, vector_tọa_độ)
        Sophus::SE2d T_curr(Sophus::SO2d(yaw), Eigen::Vector2d(x, y));
        Sophus::SE2d T_ref(Sophus::SO2d(rpsi), Eigen::Vector2d(rx, ry));

        // 2. Tính sai số hình học: E = T_ref^-1 * T_curr [cite: 152]
        Sophus::SE2d T_err = T_ref.inverse() * T_curr;

        // 3. Sử dụng ánh xạ Logarithmic để đưa E về vector lỗi 3 chiều [cite: 136]
        // Vector xi có cấu trúc: [e_x, e_y, e_psi]
        Eigen::Vector3d xi = T_err.log();

        ex   = xi[0]; // Lỗi dọc
        ey   = xi[1]; // Lỗi ngang
        epsi = xi[2]; // Lỗi góc (đã được Log map xử lý tự động trong [-pi, pi])
    }

    // ==============================================================================
    // 4. HÀM HỖ TRỢ (HELPERS)
    // ==============================================================================
    size_t GmpcController::findClosestPoint(double x, double y)
    {
        if (path_points_.empty()) return 0;

        size_t best_idx = current_index_;
        double min_dist_sq = std::numeric_limits<double>::infinity();

        // Chỉ tìm kiếm trong cửa sổ 100 điểm phía trước để giảm tải CPU
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

void GmpcController::computeReference(size_t idx, double &rx, double &ry, double &rpsi)
{
    if (path_points_.empty()) {
        rx = 0.0;
        ry = 0.0;
        rpsi = 0.0;
        return;
    }

    idx = std::min(idx, path_points_.size() - 1);

    rx = path_points_[idx].first;
    ry = path_points_[idx].second;

    if (idx + 1 < path_points_.size()) {
        double dx = path_points_[idx + 1].first - rx;
        double dy = path_points_[idx + 1].second - ry;
        rpsi = std::atan2(dy, dx);
    } else if (idx > 0) {
        double dx = rx - path_points_[idx - 1].first;
        double dy = ry - path_points_[idx - 1].second;
        rpsi = std::atan2(dy, dx);
    } else {
        rpsi = 0.0;
    }
}

    // ==============================================================================
    // 5. QUẢN LÝ BỘ NHỚ OSQP
    // ==============================================================================
    void GmpcController::freeOSQPMemory()
    {
        // Giải phóng bộ giải và các cài đặt liên quan [cite: 193]
        if (solver_) { osqp_cleanup(solver_); solver_ = nullptr; }
        if (settings_) { free(settings_); settings_ = nullptr; }

        // Giải phóng các thành phần của ma trận thưa (Sparse Matrices)
        if (P_x_) { free(P_x_); P_x_ = nullptr; }
        if (P_i_) { free(P_i_); P_i_ = nullptr; }
        if (P_p_) { free(P_p_); P_p_ = nullptr; }

        if (A_x_) { free(A_x_); A_x_ = nullptr; }
        if (A_i_) { free(A_i_); A_i_ = nullptr; }
        if (A_p_) { free(A_p_); A_p_ = nullptr; }

        // Giải phóng các vector dữ liệu OSQPFloat
        if (q_data_) { free(q_data_); q_data_ = nullptr; }
        if (l_data_) { free(l_data_); l_data_ = nullptr; }
        if (u_data_) { free(u_data_); u_data_ = nullptr; }
    }

void GmpcController::eigenToOSQPCsc(const Eigen::SparseMatrix<double>& mat,
                                    OSQPCscMatrix& out_mat,
                                    OSQPFloat*& out_x, OSQPInt*& out_i, OSQPInt*& out_p)
{
    Eigen::SparseMatrix<double> compressed = mat;
    compressed.makeCompressed();

    OSQPInt nnz = compressed.nonZeros();
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

    // ==============================================================================
    // 6. HÀM MAIN
    // ==============================================================================
    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<GmpcController>());
        rclcpp::shutdown();
        return 0;
    }