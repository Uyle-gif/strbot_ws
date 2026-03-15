#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MPCStateFilter(Node):
    def __init__(self) -> None:
        super().__init__("mpc_state_filter")

        # ---------------- Parameters ----------------
        self.declare_parameter("input_odom_topic", "/odometry/filtered")
        self.declare_parameter("output_odom_topic", "/mpc_state")
        self.declare_parameter("cmd_vel_topic","/ackermann_controller/cmd_vel")
        self.declare_parameter("publish_standstill_topic", "/mpc_state/is_standstill")
        self.declare_parameter("use_cmd_vel_for_standstill", True)

        self.declare_parameter("alpha_x", 0.3)
        self.declare_parameter("alpha_y", 0.3)
        self.declare_parameter("alpha_yaw", 0.20)
        self.declare_parameter("alpha_v", 0.25)
        self.declare_parameter("alpha_wz", 0.2)

        self.declare_parameter("v_deadband", 0.02)
        self.declare_parameter("wz_deadband", 0.02)

        self.declare_parameter("v_standstill_threshold", 0.03)
        self.declare_parameter("wz_standstill_threshold", 0.03)
        self.declare_parameter("cmd_v_standstill_threshold", 0.03)
        self.declare_parameter("cmd_wz_standstill_threshold", 0.03)
        self.declare_parameter("standstill_hold_time", 0.30)

        self.declare_parameter("freeze_pose_when_standstill", False)
        self.declare_parameter("freeze_yaw_when_standstill", True)

        self.declare_parameter("max_v_rate", 2.0)    # m/s^2
        self.declare_parameter("max_wz_rate", 4.0)   # rad/s^2

        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("output_child_frame_id", "")

        self.input_odom_topic = self.get_parameter("input_odom_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.publish_standstill_topic = self.get_parameter("publish_standstill_topic").value
        self.use_cmd_vel_for_standstill = bool(
            self.get_parameter("use_cmd_vel_for_standstill").value
        )

        self.alpha_x = float(self.get_parameter("alpha_x").value)
        self.alpha_y = float(self.get_parameter("alpha_y").value)
        self.alpha_yaw = float(self.get_parameter("alpha_yaw").value)
        self.alpha_v = float(self.get_parameter("alpha_v").value)
        self.alpha_wz = float(self.get_parameter("alpha_wz").value)

        self.v_deadband = float(self.get_parameter("v_deadband").value)
        self.wz_deadband = float(self.get_parameter("wz_deadband").value)

        self.v_standstill_threshold = float(
            self.get_parameter("v_standstill_threshold").value
        )
        self.wz_standstill_threshold = float(
            self.get_parameter("wz_standstill_threshold").value
        )
        self.cmd_v_standstill_threshold = float(
            self.get_parameter("cmd_v_standstill_threshold").value
        )
        self.cmd_wz_standstill_threshold = float(
            self.get_parameter("cmd_wz_standstill_threshold").value
        )
        self.standstill_hold_time = float(
            self.get_parameter("standstill_hold_time").value
        )

        self.freeze_pose_when_standstill = bool(
            self.get_parameter("freeze_pose_when_standstill").value
        )
        self.freeze_yaw_when_standstill = bool(
            self.get_parameter("freeze_yaw_when_standstill").value
        )

        self.max_v_rate = float(self.get_parameter("max_v_rate").value)
        self.max_wz_rate = float(self.get_parameter("max_wz_rate").value)

        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.output_child_frame_id = str(self.get_parameter("output_child_frame_id").value)

        # ---------------- State ----------------
        self.initialized = False

        self.last_msg_time_sec: Optional[float] = None

        self.last_raw_yaw: float = 0.0
        self.yaw_continuous: float = 0.0

        self.fx: float = 0.0
        self.fy: float = 0.0
        self.fyaw: float = 0.0
        self.fv: float = 0.0
        self.fwz: float = 0.0

        self.last_cmd_v: float = 0.0
        self.last_cmd_wz: float = 0.0

        self.standstill_candidate_since: Optional[float] = None
        self.is_standstill: bool = False

        self.last_pose_x_for_freeze: float = 0.0
        self.last_pose_y_for_freeze: float = 0.0
        self.last_yaw_for_freeze: float = 0.0

        # ---------------- ROS I/O ----------------
        self.odom_sub = self.create_subscription(
            Odometry, self.input_odom_topic, self.odom_callback, 50
        )

        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_callback, 50
        )

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 50)
        self.standstill_pub = self.create_publisher(
            Bool, self.publish_standstill_topic, 10
        )

        self.get_logger().info("mpc_state_filter started")
        self.get_logger().info(f"input_odom_topic: {self.input_odom_topic}")
        self.get_logger().info(f"output_odom_topic: {self.output_odom_topic}")
        self.get_logger().info(f"cmd_vel_topic: {self.cmd_vel_topic}")
        self.get_logger().info(f"use_cmd_vel_for_standstill: {self.use_cmd_vel_for_standstill}")

    def cmd_callback(self, msg: Twist) -> None:
        self.last_cmd_v = msg.linear.x
        self.last_cmd_wz = msg.angular.z

    def odom_callback(self, msg: Odometry) -> None:
        t_sec = self.stamp_to_sec(msg)

        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        raw_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        raw_v = msg.twist.twist.linear.x
        raw_wz = msg.twist.twist.angular.z

        if not self.initialized:
            self.initialize_filter(raw_x, raw_y, raw_yaw, raw_v, raw_wz, t_sec)
            self.publish_filtered(msg)
            return

        dt = max(1e-3, t_sec - self.last_msg_time_sec)

        # Unwrap yaw before filtering
        dyaw = wrap_to_pi(raw_yaw - self.last_raw_yaw)
        self.yaw_continuous += dyaw
        self.last_raw_yaw = raw_yaw

        # Low-pass filter
        self.fx = self.low_pass(self.fx, raw_x, self.alpha_x)
        self.fy = self.low_pass(self.fy, raw_y, self.alpha_y)
        self.fyaw = self.low_pass(self.fyaw, self.yaw_continuous, self.alpha_yaw)

        v_lp = self.low_pass(self.fv, raw_v, self.alpha_v)
        wz_lp = self.low_pass(self.fwz, raw_wz, self.alpha_wz)

        # Rate limiter
        max_dv = self.max_v_rate * dt
        max_dwz = self.max_wz_rate * dt
        self.fv = clamp(v_lp, self.fv - max_dv, self.fv + max_dv)
        self.fwz = clamp(wz_lp, self.fwz - max_dwz, self.fwz + max_dwz)

        # Deadband around zero
        if abs(self.fv) < self.v_deadband:
            self.fv = 0.0
        if abs(self.fwz) < self.wz_deadband:
            self.fwz = 0.0

        # Standstill detection
        self.update_standstill(t_sec)

        # Freeze selected states when standstill
        if self.is_standstill:
            self.fv = 0.0
            self.fwz = 0.0

            if self.freeze_pose_when_standstill:
                self.fx = self.last_pose_x_for_freeze
                self.fy = self.last_pose_y_for_freeze
            else:
                self.last_pose_x_for_freeze = self.fx
                self.last_pose_y_for_freeze = self.fy

            if self.freeze_yaw_when_standstill:
                self.fyaw = self.last_yaw_for_freeze
            else:
                self.last_yaw_for_freeze = self.fyaw
        else:
            self.last_pose_x_for_freeze = self.fx
            self.last_pose_y_for_freeze = self.fy
            self.last_yaw_for_freeze = self.fyaw

        self.last_msg_time_sec = t_sec
        self.publish_filtered(msg)

    def initialize_filter(
        self,
        raw_x: float,
        raw_y: float,
        raw_yaw: float,
        raw_v: float,
        raw_wz: float,
        t_sec: float,
    ) -> None:
        self.fx = raw_x
        self.fy = raw_y
        self.last_raw_yaw = raw_yaw
        self.yaw_continuous = raw_yaw
        self.fyaw = raw_yaw
        self.fv = raw_v
        self.fwz = raw_wz

        self.last_pose_x_for_freeze = raw_x
        self.last_pose_y_for_freeze = raw_y
        self.last_yaw_for_freeze = raw_yaw

        self.last_msg_time_sec = t_sec
        self.initialized = True

        self.get_logger().info("Filter initialized")

    def update_standstill(self, t_sec: float) -> None:
        small_state = (
            abs(self.fv) < self.v_standstill_threshold
            and abs(self.fwz) < self.wz_standstill_threshold
        )

        if self.use_cmd_vel_for_standstill:
            small_cmd = (
                abs(self.last_cmd_v) < self.cmd_v_standstill_threshold
                and abs(self.last_cmd_wz) < self.cmd_wz_standstill_threshold
            )
        else:
            small_cmd = True

        candidate = small_state and small_cmd

        if candidate:
            if self.standstill_candidate_since is None:
                self.standstill_candidate_since = t_sec

            if (t_sec - self.standstill_candidate_since) >= self.standstill_hold_time:
                self.is_standstill = True
        else:
            self.standstill_candidate_since = None
            self.is_standstill = False

    def publish_filtered(self, input_msg: Odometry) -> None:
        out = Odometry()

        out.header = input_msg.header
        out.child_frame_id = input_msg.child_frame_id

        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id
        if self.output_child_frame_id:
            out.child_frame_id = self.output_child_frame_id

        out.pose.pose.position.x = self.fx
        out.pose.pose.position.y = self.fy
        out.pose.pose.position.z = 0.0

        yaw_wrapped = wrap_to_pi(self.fyaw)
        q = quaternion_from_euler(0.0, 0.0, yaw_wrapped)
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]

        out.twist.twist.linear.x = self.fv
        out.twist.twist.linear.y = 0.0
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = self.fwz

        # Keep original covariance if you want compatibility with downstream nodes.
        out.pose.covariance = input_msg.pose.covariance
        out.twist.covariance = input_msg.twist.covariance

        self.odom_pub.publish(out)

        standstill_msg = Bool()
        standstill_msg.data = self.is_standstill
        self.standstill_pub.publish(standstill_msg)

    @staticmethod
    def low_pass(prev_value: float, new_value: float, alpha: float) -> float:
        return alpha * new_value + (1.0 - alpha) * prev_value

    @staticmethod
    def stamp_to_sec(msg: Odometry) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCStateFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()