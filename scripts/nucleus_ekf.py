#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from interfaces.msg import AHRS, BottomTrack
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_from_matrix(R: np.ndarray) -> Quaternion:
    q = Quaternion()
    tr = R[0, 0] + R[1, 1] + R[2, 2]

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q.w = 0.25 * s
        q.x = (R[2, 1] - R[1, 2]) / s
        q.y = (R[0, 2] - R[2, 0]) / s
        q.z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q.w = (R[2, 1] - R[1, 2]) / s
        q.x = 0.25 * s
        q.y = (R[0, 1] + R[1, 0]) / s
        q.z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q.w = (R[0, 2] - R[2, 0]) / s
        q.x = (R[0, 1] + R[1, 0]) / s
        q.y = 0.25 * s
        q.z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q.w = (R[1, 0] - R[0, 1]) / s
        q.x = (R[0, 2] + R[2, 0]) / s
        q.y = (R[1, 2] + R[2, 1]) / s
        q.z = 0.25 * s

    return q


def body_to_ned_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp,     sr * cp,                cr * cp],
    ])


class NortekDvlEkfOdomEnuBaseNed(Node):
    """
    Internal EKF state is NED:
        x = north
        y = east
        z = down

    Published odom frame is ENU:
        x = east
        y = north
        z = up

    Published base_link frame is NED/FRD:
        x = forward/north at zero yaw
        y = right/east at zero yaw
        z = down

    At startup:
        base_link position == odom origin,
        but base_link axes differ from odom axes.
    """

    def __init__(self):
        super().__init__("nortek_dvl_ekf_odom_enu_base_ned")

        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("ahrs_degrees", True)
        self.declare_parameter("dvl_velocity_scale", 1.0)

        self.declare_parameter("dvl_velocity_std", 0.05)
        self.declare_parameter("attitude_std_deg", 2.0)

        self.declare_parameter("process_noise_position", 0.02)
        self.declare_parameter("process_noise_orientation", 0.01)
        self.declare_parameter("process_noise_linear_velocity", 0.08)
        self.declare_parameter("process_noise_angular_velocity", 0.03)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ahrs_degrees = bool(self.get_parameter("ahrs_degrees").value)
        self.vel_scale = float(self.get_parameter("dvl_velocity_scale").value)

        self.x = np.zeros((12, 1))

        self.P = np.diag([
            0.01, 0.01, 0.01,
            0.01, 0.01, 0.01,
            0.25, 0.25, 0.25,
            0.10, 0.10, 0.10,
        ]) ** 2

        self.last_time = None

        self.R_ned_to_enu = np.array([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            AHRS,
            "/nucleus_node/ahrs_packets",
            self.ahrs_cb,
            qos,
        )

        self.create_subscription(
            BottomTrack,
            "/nucleus_node/bottom_track_packets",
            self.bottom_track_cb,
            qos,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/nortek_dvl_ekf/odom",
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("EKF started: odom=ENU, base_link=NED/FRD")

    def msg_time(self, _msg):
        return self.get_clock().now()

    def predict(self, stamp):
        t = stamp.nanoseconds * 1e-9

        if self.last_time is None:
            self.last_time = t
            return

        dt = max(1e-4, min(t - self.last_time, 1.0))
        self.last_time = t

        self.x[0, 0] += self.x[6, 0] * dt
        self.x[1, 0] += self.x[7, 0] * dt
        self.x[2, 0] += self.x[8, 0] * dt

        self.x[3, 0] = wrap_pi(self.x[3, 0] + self.x[9, 0] * dt)
        self.x[4, 0] = wrap_pi(self.x[4, 0] + self.x[10, 0] * dt)
        self.x[5, 0] = wrap_pi(self.x[5, 0] + self.x[11, 0] * dt)

        F = np.eye(12)
        F[0, 6] = dt
        F[1, 7] = dt
        F[2, 8] = dt
        F[3, 9] = dt
        F[4, 10] = dt
        F[5, 11] = dt

        q_pos = float(self.get_parameter("process_noise_position").value)
        q_ori = float(self.get_parameter("process_noise_orientation").value)
        q_vel = float(self.get_parameter("process_noise_linear_velocity").value)
        q_w = float(self.get_parameter("process_noise_angular_velocity").value)

        Q = np.diag([
            q_pos, q_pos, q_pos,
            q_ori, q_ori, q_ori,
            q_vel, q_vel, q_vel,
            q_w, q_w, q_w,
        ]) ** 2 * dt

        self.P = F @ self.P @ F.T + Q

    def ahrs_cb(self, msg: AHRS):
        stamp = self.msg_time(msg)
        self.predict(stamp)

        roll = float(msg.roll)
        pitch = float(msg.pitch)
        yaw = float(msg.heading)

        if self.ahrs_degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)

        z = np.array([
            [wrap_pi(roll)],
            [wrap_pi(pitch)],
            [wrap_pi(yaw)],
        ])

        H = np.zeros((3, 12))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        H[2, 5] = 1.0

        attitude_std = math.radians(
            float(self.get_parameter("attitude_std_deg").value)
        )

        R = np.diag([
            attitude_std ** 2,
            attitude_std ** 2,
            attitude_std ** 2,
        ])

        y = z - H @ self.x
        y[0, 0] = wrap_pi(y[0, 0])
        y[1, 0] = wrap_pi(y[1, 0])
        y[2, 0] = wrap_pi(y[2, 0])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y

        self.x[3, 0] = wrap_pi(self.x[3, 0])
        self.x[4, 0] = wrap_pi(self.x[4, 0])
        self.x[5, 0] = wrap_pi(self.x[5, 0])

        self.P = (np.eye(12) - K @ H) @ self.P

        self.publish_odom(stamp)

    def bottom_track_cb(self, msg: BottomTrack):
        stamp = self.msg_time(msg)
        self.predict(stamp)

        if not (
            msg.x_velocity_valid
            and msg.y_velocity_valid
            and msg.z_velocity_valid
        ):
            return

        vx_b = float(msg.velocity_x) * self.vel_scale
        vy_b = float(msg.velocity_y) * self.vel_scale
        vz_b = float(msg.velocity_z) * self.vel_scale

        roll = float(self.x[3, 0])
        pitch = float(self.x[4, 0])
        yaw = float(self.x[5, 0])

        R_body_to_ned = body_to_ned_rotation(roll, pitch, yaw)

        v_ned = R_body_to_ned @ np.array([
            [vx_b],
            [vy_b],
            [vz_b],
        ])

        z = v_ned

        H = np.zeros((3, 12))
        H[0, 6] = 1.0
        H[1, 7] = 1.0
        H[2, 8] = 1.0

        vel_std = float(self.get_parameter("dvl_velocity_std").value)

        R = np.diag([
            vel_std ** 2,
            vel_std ** 2,
            vel_std ** 2,
        ])

        y = z - H @ self.x

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(12) - K @ H) @ self.P

        self.publish_odom(stamp)

    def publish_odom(self, stamp):
        odom = Odometry()

        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_frame

        north = float(self.x[0, 0])
        east = float(self.x[1, 0])
        down = float(self.x[2, 0])

        # Convert NED position to ENU position.
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = -down

        roll = float(self.x[3, 0])
        pitch = float(self.x[4, 0])
        yaw = float(self.x[5, 0])

        R_body_to_ned = body_to_ned_rotation(roll, pitch, yaw)

        # Transform rotation:
        # child base_link is NED/FRD,
        # parent odom is ENU.
        R_body_to_enu = self.R_ned_to_enu @ R_body_to_ned

        q_odom_to_base = quat_from_matrix(R_body_to_enu)
        odom.pose.pose.orientation = q_odom_to_base

        # nav_msgs/Odometry twist is expressed in child_frame_id.
        # Therefore publish body-frame FRD velocity.
        v_ned = np.array([
            [float(self.x[6, 0])],
            [float(self.x[7, 0])],
            [float(self.x[8, 0])],
        ])

        v_body = R_body_to_ned.T @ v_ned

        odom.twist.twist.linear.x = float(v_body[0, 0])
        odom.twist.twist.linear.y = float(v_body[1, 0])
        odom.twist.twist.linear.z = float(v_body[2, 0])

        odom.twist.twist.angular.x = float(self.x[9, 0])
        odom.twist.twist.angular.y = float(self.x[10, 0])
        odom.twist.twist.angular.z = float(self.x[11, 0])

        for i in range(6):
            odom.pose.covariance[i * 6 + i] = float(self.P[i, i])
            odom.twist.covariance[i * 6 + i] = float(self.P[i + 6, i + 6])

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = east
        tf.transform.translation.y = north
        tf.transform.translation.z = -down
        tf.transform.rotation = q_odom_to_base

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = NortekDvlEkfOdomEnuBaseNed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
