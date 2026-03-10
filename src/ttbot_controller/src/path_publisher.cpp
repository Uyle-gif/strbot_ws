#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathFileReader : public rclcpp::Node
{
public:
    PathFileReader() : Node("path_reader_node")
    {
        // 1. Khai báo tham số
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("file_path", ""); // Đường dẫn tuyệt đối tới file CSV

        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string file_path = this->get_parameter("file_path").as_string();

        // 2. Setup Publisher với QoS Transient Local (Lưu lại cho Node chạy sau)
        rclcpp::QoS qos_profile(10);
        qos_profile.transient_local();
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

        // 3. Kiểm tra đường dẫn và đọc file
        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Chua cung cap duong dan file_path!");
            return;
        }

        load_and_publish_csv(file_path);
    }

private:
    void load_and_publish_csv(const std::string& file_path)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = frame_id_;
        path.header.stamp = this->now();

        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Khong the mo file: %s", file_path.c_str());
            return;
        }

        std::string line;
        int point_count = 0;

        // Đọc từng dòng trong file CSV
        while (std::getline(file, line)) {
            // Bỏ qua dòng trống hoặc dòng comment/tiêu đề (bắt đầu bằng chữ cái)
            if (line.empty() || isalpha(line[0]) || line[0] == '#') continue;

            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;

            // Tách các giá trị bằng dấu phẩy
            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stod(token));
                } catch (...) {
                    continue; // Bỏ qua nếu lỗi ép kiểu
                }
            }

            // Nếu đọc được ít nhất 2 cột (x, y)
            if (values.size() >= 2) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.header.stamp = path.header.stamp;
                
                pose.pose.position.x = values[0]; // Cột 1 là X
                pose.pose.position.y = values[1]; // Cột 2 là Y
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0; 

                path.poses.push_back(pose);
                point_count++;
            }
        }
        file.close();

        // Publish đường đi
        publisher_->publish(path);
        RCLCPP_INFO(this->get_logger(), ">>> Da load va publish thanh cong %d toa do tu file CSV <<<", point_count);
    }

    std::string frame_id_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFileReader>());
    rclcpp::shutdown();
    return 0;
}