import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


INITIAL_POSE_X   = 0.0
INITIAL_POSE_Y   = 0.0
INITIAL_POSE_YAW = 0.0
PUBLISH_RATE     = 1.0   # publish setiap 1 detik


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self._amcl_received = False

        # Subscribe ke /amcl_pose — kalau sudah ada data, berhenti publish
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10
        )

        # Publish setiap 1 detik sampai AMCL konfirmasi
        self._timer = self.create_timer(PUBLISH_RATE, self._publish)

        self.get_logger().info(
            f"Initial pose publisher aktif — publish terus sampai AMCL konfirmasi: "
            f"x={INITIAL_POSE_X}, y={INITIAL_POSE_Y}, yaw={math.degrees(INITIAL_POSE_YAW):.1f}°"
        )

    def _amcl_callback(self, msg):
        if not self._amcl_received:
            self._amcl_received = True
            self._timer.cancel()
            self.get_logger().info(
                f"✅ AMCL sudah terima pose — stop publish. "
                f"Posisi robot: x={msg.pose.pose.position.x:.3f}, "
                f"y={msg.pose.pose.position.y:.3f}"
            )
            raise SystemExit

    def _publish(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = INITIAL_POSE_X
        msg.pose.pose.position.y = INITIAL_POSE_Y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(INITIAL_POSE_YAW / 2.0)
        msg.pose.pose.orientation.w = math.cos(INITIAL_POSE_YAW / 2.0)

        cov = [0.0] * 36
        cov[0]  = 0.25
        cov[7]  = 0.25
        cov[35] = 0.07
        msg.pose.covariance = cov

        self._pub.publish(msg)
        self.get_logger().info(
            "📍 Publishing initial pose ke /initialpose... (menunggu AMCL konfirmasi)"
        )


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
