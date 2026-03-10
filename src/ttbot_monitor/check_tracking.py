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
        # Đảm bảo topic khớp với Dual EKF và Path Publisher của bạn
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.sub_path = self.create_subscription(Path, '/mpc_path', self.path_callback, 10)
        
        self.robot_x, self.robot_y = [], []
        self.path_x, self.path_y = [], []
        self.lock = threading.Lock() # Thêm lock để bảo vệ dữ liệu khi đa luồng
        self.get_logger().info("Đang chờ dữ liệu từ /mpc_path và /odometry/filtered...")

    def odom_callback(self, msg):
        with self.lock:
            self.robot_x.append(msg.pose.pose.position.x)
            self.robot_y.append(msg.pose.pose.position.y)

    def path_callback(self, msg):
        with self.lock:
            # Lưu lại toàn bộ Path tham chiếu
            self.path_x = [p.pose.position.x for p in msg.poses]
            self.path_y = [p.pose.position.y for p in msg.poses]
            if self.path_x:
                self.get_logger().info(f"Đã nhận Path: {len(self.path_x)} điểm")

def update_plot(frame, node, line_robot, line_path, ax):
    with node.lock:
        if not node.path_x and not node.robot_x:
            return line_robot, line_path

        # Cập nhật dữ liệu lên đồ thị
        line_path.set_data(node.path_x, node.path_y)
        line_robot.set_data(node.robot_x, node.robot_y)
        
        all_x = node.robot_x + node.path_x
        all_y = node.robot_y + node.path_y
        
        if all_x and all_y:
            padding = 2.0
            ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
            ax.set_ylim(min(all_y) - padding, max(all_y) + padding)

    return line_robot, line_path

def main():
    rclpy.init()
    node = PathTrackingEval()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal') 
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Robot Trajectory Tracking Analysis")

    line_path, = ax.plot([], [], 'g--', linewidth=1.5, label='Reference Path')
    line_robot, = ax.plot([], [], 'r-', linewidth=2.0, label='Actual Robot Path')
    ax.legend(loc='upper right')

    ani = FuncAnimation(fig, update_plot, fargs=(node, line_robot, line_path, ax), 
                        interval=200, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()