#!/usr/bin/env python3
"""
SINGLE RUN NODE — untuk perlakuan 2 (obstacle statis) dan 3 (obstacle dinamis)
Robot jalan dari titik START ke titik FINISH sekali jalan lalu berhenti.
Lebih mudah untuk pengukuran jarak deteksi dan waktu reaksi.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus

import math
import time


# ============================================================
# KONFIGURASI — ubah sesuai kebutuhan perlakuan
# ============================================================

# Titik START (posisi awal robot)
START_X   = 0.0
START_Y   = 0.0
START_YAW = 0.0   # radian — 0 = menghadap sumbu X positif

# Titik FINISH (tujuan robot)
FINISH_X   = 5.20    # WP 9 — ujung kanan loop
FINISH_Y   = 0.0
FINISH_YAW = 0.0

# Timeout perjalanan (detik)
TRAVEL_TIMEOUT = 300.0    # 5 menit

# Waktu tunggu AMCL
AMCL_WAIT_TIMEOUT = 5.0
FALLBACK_X = 0.0
FALLBACK_Y = 0.0


class PatrolNode(Node):

    def __init__(self):
        super().__init__('single_run_node')

        self.cb_group = ReentrantCallbackGroup()

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        self.robot_x       = FALLBACK_X
        self.robot_y       = FALLBACK_Y
        self.pose_received = False
        self._wait_elapsed = 0.0
        self._start_time   = None
        self._goal_handle  = None
        self._timeout_timer = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10,
            callback_group=self.cb_group
        )

        # Hitung jarak START → FINISH
        self._jarak = math.sqrt(
            (FINISH_X - START_X)**2 + (FINISH_Y - START_Y)**2
        )

        self.get_logger().info("=" * 55)
        self.get_logger().info("SINGLE RUN NODE — Sekali Jalan")
        self.get_logger().info(f"START  : ({START_X:.3f}, {START_Y:.3f})")
        self.get_logger().info(f"FINISH : ({FINISH_X:.3f}, {FINISH_Y:.3f})")
        self.get_logger().info(f"Jarak lurus : {self._jarak:.3f} m")
        self.get_logger().info(f"Timeout     : {TRAVEL_TIMEOUT}s")
        self.get_logger().info("Menunggu Nav2 action server...")
        self.get_logger().info("=" * 55)

        self._action_client.wait_for_server()
        self.get_logger().info("Nav2 READY! Menunggu AMCL...")

        self._start_timer = self.create_timer(
            1.0, self._wait_for_pose,
            callback_group=self.cb_group
        )

    # ============================================================
    # AMCL
    # ============================================================
    def _amcl_callback(self, msg):
        self.robot_x       = msg.pose.pose.position.x
        self.robot_y       = msg.pose.pose.position.y
        self.pose_received = True

    def _wait_for_pose(self):
        self._wait_elapsed += 1.0
        if self.pose_received:
            self._start_timer.cancel()
            self.get_logger().info(
                f"✅ AMCL diterima ({self._wait_elapsed:.0f}s) "
                f"— x={self.robot_x:.3f}, y={self.robot_y:.3f}"
            )
            self._send_goal()
        elif self._wait_elapsed >= AMCL_WAIT_TIMEOUT:
            self._start_timer.cancel()
            self.get_logger().warn(
                f"⚠️  AMCL timeout — pakai fallback ({FALLBACK_X}, {FALLBACK_Y})"
            )
            self._send_goal()
        else:
            self.get_logger().info(
                f"Menunggu AMCL... ({self._wait_elapsed:.0f}/{AMCL_WAIT_TIMEOUT:.0f}s)"
            )

    # ============================================================
    # KIRIM GOAL KE FINISH
    # ============================================================
    def _send_goal(self):
        self._start_time = time.time()

        q = self._yaw_to_quat(FINISH_YAW)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id    = 'map'
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = FINISH_X
        goal_msg.pose.pose.position.y    = FINISH_Y
        goal_msg.pose.pose.position.z    = 0.0
        goal_msg.pose.pose.orientation.x = q['x']
        goal_msg.pose.pose.orientation.y = q['y']
        goal_msg.pose.pose.orientation.z = q['z']
        goal_msg.pose.pose.orientation.w = q['w']

        self.get_logger().info("=" * 55)
        self.get_logger().info(
            f"🚀 Menuju FINISH ({FINISH_X:.3f}, {FINISH_Y:.3f})..."
        )
        self.get_logger().info("=" * 55)

        # Timeout timer
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        self._timeout_timer = self.create_timer(
            TRAVEL_TIMEOUT,
            self._on_timeout,
            callback_group=self.cb_group
        )

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    # ============================================================
    # CALLBACKS
    # ============================================================
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal DITOLAK Nav2")
            self._finish(success=False, reason="Goal ditolak")
            return
        self._goal_handle = goal_handle
        self.get_logger().info("✅ Goal diterima — robot bergerak...")
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        elapsed = time.time() - self._start_time if self._start_time else 0
        self.get_logger().info(
            f"  Jarak tersisa: {dist:.2f} m | Elapsed: {elapsed:.1f}s",
            throttle_duration_sec=3.0
        )

    def _result_callback(self, future):
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finish(success=True, reason="Sampai di tujuan")
        elif status == GoalStatus.STATUS_ABORTED:
            self._finish(success=False, reason="Goal ABORTED")
        else:
            self._finish(success=False, reason=f"Status={status}")

    def _on_timeout(self):
        self.get_logger().warn(f"⏰ TIMEOUT {TRAVEL_TIMEOUT}s")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._finish(success=False, reason="Timeout")

    # ============================================================
    # FINISH — cetak ringkasan dan keluar
    # ============================================================
    def _finish(self, success, reason):
        durasi  = time.time() - self._start_time if self._start_time else 0
        status  = "✅ BERHASIL" if success else "❌ GAGAL"

        self.get_logger().info("=" * 55)
        self.get_logger().info(f"🏁 SELESAI — {status}")
        self.get_logger().info(f"   Alasan  : {reason}")
        self.get_logger().info(f"   Durasi  : {durasi:.1f}s ({durasi/60:.1f} menit)")
        self.get_logger().info(f"   Jarak lurus START→FINISH: {self._jarak:.3f} m")
        self.get_logger().info(
            f"   Kec. rata-rata: "
            f"{self._jarak/durasi:.3f} m/s" if durasi > 0 else "   Kec. rata-rata: -"
        )
        self.get_logger().info("=" * 55)
        raise SystemExit

    # ============================================================
    # HELPER
    # ============================================================
    def _yaw_to_quat(self, yaw):
        return {
            'x': 0.0, 'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }


def main():
    rclpy.init()
    node = PatrolNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("Node selesai.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
