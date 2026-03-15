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
            "/mpc_state", 10,
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

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);

    current_pose_x_ = x;
    current_pose_y_ = y;
    current_pose_yaw_ = yaw;
    has_odom_ = true;

    // 1. Tìm điểm gần nhất trên quỹ đạo
    size_t idx = findClosestPoint(x, y);

    // 2. Kiểm tra xem đã đến đích chưa
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

    // 3. Đưa tọa độ và index thẳng vào bộ LTV-GMPC để giải
    Control u = solveMPC(x, y, yaw, idx);

    // 4. Xuất lệnh điều khiển xuống xe
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = u[0];
    cmd.twist.angular.z = u[1];
    cmd_pub_->publish(cmd);
}
void GmpcController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            has_path_ = false;
            current_index_ = 0;
            RCLCPP_WARN(this->get_logger(), "Received empty path.");
            return;
        }

        // 1. Lấy dữ liệu raw từ topic
        std::vector<std::pair<double, double>> raw_points;
        raw_points.reserve(msg->poses.size());
        for (const auto &pose : msg->poses) {
            raw_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }

        // 2. THUẬT TOÁN VUỐT MƯỢT GÓC (Moving Average Filter)
        // Cấu hình bo góc đủ rộng để GMPC không bị lỗi Singularity
        int smooth_passes = 1 ; // Số lần lặp vuốt (bào mòn góc vuông)
        int window_size = 1;    // Tầm nhìn để bo góc

        std::vector<std::pair<double, double>> smoothed_points = raw_points;

        for (int pass = 0; pass < smooth_passes; ++pass) {
            std::vector<std::pair<double, double>> temp_points = smoothed_points;
            
            // Bỏ qua điểm đầu và cuối để giữ nguyên mốc xuất phát/đích
            for (size_t i = 1; i < smoothed_points.size() - 1; ++i) {
                double sum_x = 0.0;
                double sum_y = 0.0;
                int count = 0;

                int start_j = std::max(0, (int)i - window_size);
                int end_j = std::min((int)smoothed_points.size() - 1, (int)i + window_size);

                for (int j = start_j; j <= end_j; ++j) {
                    sum_x += smoothed_points[j].first;
                    sum_y += smoothed_points[j].second;
                    count++;
                }

                temp_points[i].first = sum_x / count;
                temp_points[i].second = sum_y / count;
            }
            smoothed_points = temp_points;
        }

        // 3. Cập nhật quỹ đạo đã làm mượt vào hệ thống
        path_points_ = smoothed_points;
        current_index_ = 0;
        has_path_ = true;
        reached_goal_ = false;

        RCLCPP_INFO(this->get_logger(),
                    "--> GMPC RECEIVED AND SMOOTHED PATH: %zu points.", path_points_.size());
    }    

    void GmpcController::mpcTuningCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        (void)msg;
        // TODO: Cập nhật trọng số MPC online
    }

    // ==============================================================================
    // 3. TOÁN HỌC GMPC & LÕI TỐI ƯU
    // ==============================================================================
 Control GmpcController::solveMPC(double x0, double y0, double yaw0, size_t start_idx)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    freeOSQPMemory();

    const int nx = 3;            // [ex, ey, epsi]
    const int nu = 2;            // [dv, domega]
    const int N  = N_p_;         // Horizon
    const int nx_aug = nx + nu;  // [xi, u_tilde_prev]
    
    const int n_vars = (N + 1) * nx_aug + N * nu;
    const int n_cons = (N + 1) * nx_aug + N * nu;

    auto wrapAngle = [](double a) {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };

    // -------------------------------------------------------------------------
    // 1. TẠO MẢNG QUỸ ĐẠO THAM CHIẾU TRONG TƯƠNG LAI (REFERENCE TRAJECTORY)
    // -------------------------------------------------------------------------
    struct RefPoint { double x, y, yaw, v, omega; };
    std::vector<RefPoint> ref_traj(N + 1);

    for (int k = 0; k <= N; ++k) {
        size_t traj_idx = std::min(start_idx + k, path_points_.size() - 1);
        double rx, ry, rpsi;
        computeReference(traj_idx, rx, ry, rpsi);
        
        double v_ref = desired_speed_;
        double omega_ref = 0.0;
        
        // Tính omega_ref tại điểm đó dựa trên độ cong của đường
        if (traj_idx + 1 < path_points_.size()) {
            double rx2, ry2, rpsi2;
            computeReference(std::min(traj_idx + 1, path_points_.size() - 1), rx2, ry2, rpsi2);
            omega_ref = wrapAngle(rpsi2 - rpsi) / dt_mpc_;
            omega_ref = std::clamp(omega_ref, -max_omega_, max_omega_);
        }
        ref_traj[k] = {rx, ry, rpsi, v_ref, omega_ref};
    }

    // -------------------------------------------------------------------------
    // 2. TÍNH LỖI HIỆN TẠI (TẠI K = 0) DÙNG LIE ALGEBRA SE(2)
    // -------------------------------------------------------------------------
    double ex0, ey0, epsi0;
    computeErrorState(x0, y0, yaw0, ref_traj[0].x, ref_traj[0].y, ref_traj[0].yaw, ex0, ey0, epsi0);

    // Publish Error cho RQT Plot / Đánh giá
    std_msgs::msg::Float32 lon_msg, cte_msg, head_msg;
    lon_msg.data = ex0;
    cte_msg.data = ey0;
    head_msg.data = epsi0 * 180.0 / M_PI;
    error_lon_pub_->publish(lon_msg);
    error_cte_pub_->publish(cte_msg);
    error_heading_pub_->publish(head_msg);

    // -------------------------------------------------------------------------
    // 3. XÂY DỰNG MA TRẬN HESSIAN P (Chi phí tối ưu)
    // -------------------------------------------------------------------------
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;

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
    // 4. XÂY DỰNG RÀNG BUỘC ĐỘNG HỌC LTV-MPC (THAY ĐỔI THEO THỜI GIAN)
    // -------------------------------------------------------------------------
    Eigen::SparseMatrix<double> A_cons(n_cons, n_vars);
    std::vector<Eigen::Triplet<double>> a_triplets;
    Eigen::VectorXd l_bounds = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd u_bounds = Eigen::VectorXd::Zero(n_cons);

    const double v_tilde_prev = prev_v_cmd_ - ref_traj[0].v;
    const double omega_tilde_prev = prev_omega_cmd_ - ref_traj[0].omega;

    // Khởi tạo điều kiện ban đầu x_aug(0)
    for (int i = 0; i < nx_aug; ++i) {
        a_triplets.emplace_back(i, i, 1.0);
    }
    l_bounds.segment(0, nx_aug) << ex0, ey0, epsi0, v_tilde_prev, omega_tilde_prev;
    u_bounds.segment(0, nx_aug) << ex0, ey0, epsi0, v_tilde_prev, omega_tilde_prev;

    // Ràng buộc động học: TÍNH LẠI A_bar, B_bar CHO MỖI BƯỚC k
    for (int k = 0; k < N; ++k) {
        // [CỐT LÕI]: Cập nhật Jacobian liên tục dựa trên v, omega ở tương lai
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

    // Giới hạn vật lý của Input U (Dựa trên v_ref, omega_ref ĐỘNG)
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

    // -------------------------------------------------------------------------
    // 5. CHUYỂN ĐỔI SANG OSQP CSC VÀ GIẢI
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

    settings_ = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings_);
    settings_->verbose = 0;
    settings_->warm_starting = 1;

    osqp_setup(&solver_, &P_osqp, q_data_, &A_osqp, l_data_, u_data_, n_cons, n_vars, settings_);
    osqp_solve(solver_);

    // -------------------------------------------------------------------------
    // 6. TRÍCH XUẤT NGHIỆM ĐIỀU KHIỂN (CONTROL EFFORT)
    // -------------------------------------------------------------------------
    double v_opt = ref_traj[0].v;
    double omega_opt = ref_traj[0].omega;

    if (solver_ && solver_->info && solver_->info->status_val == OSQP_SOLVED) {
        const int first_utilde_offset = nx_aug + nx;
        const double dv_opt = solver_->solution->x[first_utilde_offset + 0];
        const double domega_opt = solver_->solution->x[first_utilde_offset + 1];

        // Cộng delta vào reference tại bước 0 để ra lệnh thực tế
        v_opt = std::clamp(ref_traj[0].v + dv_opt, 0.0, desired_speed_);
        omega_opt = std::clamp(ref_traj[0].omega + domega_opt, -max_omega_, max_omega_);
    } else {
        RCLCPP_WARN(this->get_logger(), "GMPC OSQP failed, fallback to reference input.");
    }

    prev_v_cmd_ = v_opt;
    prev_omega_cmd_ = omega_opt;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double solver_time_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time - start_time).count();
    
    RCLCPP_INFO(this->get_logger(), "\033[1;33m[LTV-GMPC] Matrix + OSQP Time: %.3f ms\033[0m", solver_time_ms);

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