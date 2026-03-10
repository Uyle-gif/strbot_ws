import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import numpy as np

class PathTrackingEval(Node):
    def __init__(self):
        super().__init__('path_tracking_eval')
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.sub_path = self.create_subscription(Path, '/mpc_path', self.path_callback, 10)
        self.robot_x, self.robot_y = [], []
        self.path_x, self.path_y = [], []

    def odom_callback(self, msg):
        self.robot_x.append(msg.pose.pose.position.x)
        self.robot_y.append(msg.pose.pose.position.y)

    def path_callback(self, msg):
        self.path_x = [p.pose.position.x for p in msg.poses]
        self.path_y = [p.pose.position.y for p in msg.poses]

def update_plot(frame, node, line_robot, line_path, title_text, ax):
    # 1. Cập nhật dữ liệu
    line_robot.set_data(node.robot_x, node.robot_y)
    line_path.set_data(node.path_x, node.path_y)
    
    # 2. Tự động tính toán khung hình vuông vức (Bounding Box)
    if node.path_x:
        # Lấy biên của cả robot và path
        all_x = node.robot_x + node.path_x
        all_y = node.robot_y + node.path_y
        
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            
            # Tìm tâm của hình
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            
            # Tìm cạnh lớn nhất để tạo hình vuông
            span = max(max_x - min_x, max_y - min_y)
            padding = 1.5 # Thêm lề 1.5m cho thoáng
            half_size = (span / 2) + padding
            
            # Set giới hạn mới
            ax.set_xlim(center_x - half_size, center_x + half_size)
            ax.set_ylim(center_y - half_size, center_y + half_size)
            
            # Tính sai số hiển thị
            if len(node.path_x) > 0 and len(node.robot_x) > 0:
                robot_pos = np.array([node.robot_x[-1], node.robot_y[-1]])
                path_arr = np.column_stack((node.path_x, node.path_y))
                dists = np.linalg.norm(path_arr - robot_pos, axis=1)
                err = np.min(dists)
                title_text.set_text(f"Tracking")

    return line_robot, line_path, title_text

def main():
    rclpy.init()
    node = PathTrackingEval()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, ax = plt.subplots(figsize=(10, 8))
    
    # QUAN TRỌNG: Dùng adjustable='box' để không bị cắt hình
    ax.set_aspect('equal', adjustable='box') 
    
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle=':')

    line_path, = ax.plot([], [], 'g--', linewidth=2, label='Quỹ đạo đặt (Ref)')
    line_robot, = ax.plot([], [], 'r-', linewidth=2, label='Thực tế (Actual)')
    title_text = ax.set_title("Đang chờ dữ liệu...")
    ax.legend(loc='upper right')

    ani = FuncAnimation(fig, update_plot, fargs=(node, line_robot, line_path, title_text, ax), interval=100)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()