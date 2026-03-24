#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import csv
import argparse
import os

class PathTrackingEvalAndRecord(Node):
    def __init__(self, prefix: str, odom_topic: str, path_topic: str, save_dir: str):
        super().__init__('path_tracking_eval_and_record')

        self.prefix = prefix
        self.odom_topic = odom_topic
        self.path_topic = path_topic
        self.save_dir = save_dir  

        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.sub_path = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10
        )

        self.robot_data = []
        self.path_xy = []

        self.lock = threading.Lock()
        self.path_received = False
        self.saved = False

        self.get_logger().info(
            f"Waiting for data from {self.path_topic} and {self.odom_topic}..."
        )
        self.get_logger().info(f"Dữ liệu sẽ được lưu tại thư mục: {self.save_dir}")

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        with self.lock:
            self.robot_data.append((
                t, 
                pos.x, pos.y, pos.z, 
                ori.x, ori.y, ori.z, ori.w,
                lin.x, lin.y, lin.z,
                ang.x, ang.y, ang.z
            ))

    def path_callback(self, msg: Path):
        with self.lock:
            self.path_xy = [
                (p.pose.position.x, p.pose.position.y)
                for p in msg.poses
            ]

            if not self.path_received and self.path_xy:
                self.path_received = True
                self.get_logger().info(f"Received path: {len(self.path_xy)} points")

    def save_results(self, fig=None):
        if self.saved:
            return

        with self.lock:
            traj_file = os.path.join(self.save_dir, "traj.csv")
            ref_file = os.path.join(self.save_dir, "ref.csv")
            fig_file = os.path.join(self.save_dir, "overlay.png")

            with open(traj_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "t", "x", "y", "z", 
                    "qx", "qy", "qz", "qw", 
                    "vx", "vy", "vz", 
                    "wx", "wy", "wz"
                ])
                writer.writerows(self.robot_data)

            with open(ref_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y"])
                writer.writerows(self.path_xy)

            if fig is not None:
                fig.savefig(fig_file, dpi=200, bbox_inches="tight")

        self.saved = True
        self.get_logger().info(f"Saved: {traj_file}")
        self.get_logger().info(f"Saved: {ref_file}")
        if fig is not None:
            self.get_logger().info(f"Saved: {fig_file}")


def update_plot(frame, node, line_robot, line_path, marker_start, marker_end, ax):
    with node.lock:
        robot_x = [p[1] for p in node.robot_data]
        robot_y = [p[2] for p in node.robot_data]
        path_x = [p[0] for p in node.path_xy]
        path_y = [p[1] for p in node.path_xy]

    if not robot_x and not path_x:
        return line_robot, line_path, marker_start, marker_end

    line_robot.set_data(robot_y, robot_x)
    line_path.set_data(path_y, path_x)

    if path_x and path_y:
        marker_start.set_data([path_y[0]], [path_x[0]])
        marker_end.set_data([path_y[-1]], [path_x[-1]])

    all_x = robot_x + path_x
    all_y = robot_y + path_y

    if all_x and all_y:
        padding = 2.0
        ax.set_xlim(max(all_y) + padding, min(all_y) - padding)
        ax.set_ylim(min(all_x) - padding, max(all_x) + padding)

    return line_robot, line_path, marker_start, marker_end


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", type=str, default="data") 
    parser.add_argument("--odom_topic", type=str, default="/mpc_state")
    parser.add_argument("--path_topic", type=str, default="/mpc_path")
    # Đặt đường dẫn gốc hướng thẳng vào thư mục paper của package ttbot_viewer
    parser.add_argument("--base_dir", type=str, default=os.path.expanduser("~/ttbot_ws/src/ttbot_viewer/paper"))
    args, unknown = parser.parse_known_args()

    # Tạo thư mục paper nếu nó chưa tồn tại
    os.makedirs(args.base_dir, exist_ok=True)

    # Logic đếm và tạo folder data_1, data_2... bên trong thư mục paper
    counter = 1
    save_dir = os.path.join(args.base_dir, f"{args.prefix}_{counter}")
    while os.path.exists(save_dir):
        counter += 1
        save_dir = os.path.join(args.base_dir, f"{args.prefix}_{counter}")
    
    os.makedirs(save_dir)

    rclpy.init(args=unknown)
    node = PathTrackingEvalAndRecord(
        prefix=args.prefix,
        odom_topic=args.odom_topic,
        path_topic=args.path_topic,
        save_dir=save_dir
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    
    ax.set_xlabel("Y (m)")
    ax.set_ylabel("X (m)")

    line_path, = ax.plot([], [], 'k--', linewidth=1.0, label='Reference Path')
    line_robot, = ax.plot([], [], 'r-', linewidth=1.2, label='Actual Robot Path')
    
    marker_start, = ax.plot([], [], 'ko', markerfacecolor='none', markersize=8, markeredgewidth=1.5, label='Path Start') 
    marker_end, = ax.plot([], [], 'kx', markersize=10, markeredgewidth=2, label='Path End')     

    ax.legend(loc='upper right')

    ani = FuncAnimation(
        fig,
        update_plot,
        fargs=(node, line_robot, line_path, marker_start, marker_end, ax), 
        interval=200,
        blit=False,
        cache_frame_data=False
    )

    def on_close(event):
        node.save_results(fig)

    fig.canvas.mpl_connect("close_event", on_close)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results(fig)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()