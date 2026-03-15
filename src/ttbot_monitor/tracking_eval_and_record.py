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
    def __init__(self, prefix: str, odom_topic: str, path_topic: str):
        super().__init__('path_tracking_eval_and_record')

        self.prefix = prefix
        self.odom_topic = odom_topic
        self.path_topic = path_topic

        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.sub_path = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10
        )

        self.robot_data = []   # [(t, x, y)]
        self.path_xy = []      # [(x, y)]

        self.lock = threading.Lock()
        self.path_received = False
        self.saved = False

        self.get_logger().info(
            f"Đang chờ dữ liệu từ {self.path_topic} và {self.odom_topic}..."
        )

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        with self.lock:
            self.robot_data.append((t, x, y))

    def path_callback(self, msg: Path):
        with self.lock:
            self.path_xy = [
                (p.pose.position.x, p.pose.position.y)
                for p in msg.poses
            ]

            if not self.path_received and self.path_xy:
                self.path_received = True
                self.get_logger().info(f"Đã nhận path: {len(self.path_xy)} điểm")

    def save_results(self, fig=None):
        if self.saved:
            return

        with self.lock:
            traj_file = f"{self.prefix}_traj.csv"
            ref_file = f"{self.prefix}_ref.csv"
            fig_file = f"{self.prefix}_overlay.png"

            with open(traj_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t", "x", "y"])
                writer.writerows(self.robot_data)

            with open(ref_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y"])
                writer.writerows(self.path_xy)

            if fig is not None:
                fig.savefig(fig_file, dpi=200, bbox_inches="tight")

        self.saved = True
        self.get_logger().info(f"Đã lưu: {traj_file}")
        self.get_logger().info(f"Đã lưu: {ref_file}")
        if fig is not None:
            self.get_logger().info(f"Đã lưu: {fig_file}")


def update_plot(frame, node, line_robot, line_path, ax):
    with node.lock:
        robot_x = [p[1] for p in node.robot_data]
        robot_y = [p[2] for p in node.robot_data]
        path_x = [p[0] for p in node.path_xy]
        path_y = [p[1] for p in node.path_xy]

    if not robot_x and not path_x:
        return line_robot, line_path

    line_robot.set_data(robot_x, robot_y)
    line_path.set_data(path_x, path_y)

    all_x = robot_x + path_x
    all_y = robot_y + path_y

    if all_x and all_y:
        padding = 2.0
        ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
        ax.set_ylim(min(all_y) - padding, max(all_y) + padding)

    return line_robot, line_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", type=str, default="mpc")
    parser.add_argument("--odom_topic", type=str, default="/mpc_state")
    parser.add_argument("--path_topic", type=str, default="/mpc_path")
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = PathTrackingEvalAndRecord(
        prefix=args.prefix,
        odom_topic=args.odom_topic,
        path_topic=args.path_topic
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory Tracking Analysis")

    line_path, = ax.plot([], [], 'g--', linewidth=1.8, label='Reference Path')
    line_robot, = ax.plot([], [], 'r-', linewidth=2.0, label='Actual Robot Path')
    ax.legend(loc='upper right')

    ani = FuncAnimation(
        fig,
        update_plot,
        fargs=(node, line_robot, line_path, ax),
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