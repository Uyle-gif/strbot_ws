#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // 1. Khai báo tham số nhận từ file launch
        this->declare_parameter("frame_id", "odom");
        this->declare_parameter("file_path", "");

        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string file_path = this->get_parameter("file_path").as_string();

        // 2. Tạo Publisher
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
        
        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Chua cung cap tham so file_path!");
            return;
        }

        // 3. Đọc dữ liệu từ file CSV 1 lần khi khởi động
        if (load_csv(file_path)) {
            // 4. Chạy Timer publish liên tục mỗi giây (1Hz)
            timer_ = this->create_wall_timer(
                1000ms, std::bind(&PathPublisher::timer_callback, this));
            
            RCLCPP_INFO(this->get_logger(), "Path Publisher Started. Publishing path at 1Hz...");
        }
    }

private:
    bool load_csv(const std::string& file_path)
    {
        path_.header.frame_id = frame_id_;
        path_.poses.clear();

        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Khong the mo file: %s", file_path.c_str());
            return false;
        }

        std::string line;
        int point_count = 0;

        while (std::getline(file, line)) {
            // Bỏ qua dòng trống hoặc dòng tiêu đề (bắt đầu bằng chữ hoặc dấu #)
            if (line.empty() || isalpha(line[0]) || line[0] == '#') continue;

            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;

            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stod(token));
                } catch (...) {
                    continue; 
                }
            }

            // Đảm bảo có ít nhất tọa độ X và Y
            if (values.size() >= 2) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.pose.position.x = values[0];
                pose.pose.position.y = values[1];
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0; 

                path_.poses.push_back(pose);
                point_count++;
            }
        }
        file.close();

        RCLCPP_INFO(this->get_logger(), ">>> Da load thanh cong %d toa do tu file CSV <<<", point_count);
        return true;
    }

    void timer_callback()
    {
        // Cập nhật timestamp hiện tại
        path_.header.stamp = this->now();
        
        // Cập nhật timestamp cho từng điểm bên trong (giúp RViz không báo lỗi TF cũ)
        for (auto& pose : path_.poses) {
            pose.header.stamp = path_.header.stamp;
        }
        
        publisher_->publish(path_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    std::string frame_id_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}