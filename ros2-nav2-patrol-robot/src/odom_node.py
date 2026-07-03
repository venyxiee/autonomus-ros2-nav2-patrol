import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster
import math


class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        # PARAMETER
        self.R = 0.0337   # radius roda
        self.L = 0.24   # tidak dipakai lagi untuk omega

        # STATE
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.omega = 0.0

        self.last_time = self.get_clock().now()

        # TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # SUBSCRIBE
        self.create_subscription(Float32MultiArray, '/wheel_rpm', self.rpm_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # PUBLISH
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TIMER (50 Hz)
        self.timer = self.create_timer(0.02, self.update_odom)

        self.get_logger().info("ODOM READY (ENCODER + IMU)")

    # =====================
    # RPM → LINEAR SPEED
    # =====================
    def rpm_callback(self, msg):
        rpm_l = msg.data[1]
        rpm_r = msg.data[0]

        v_l = (rpm_l * 2 * math.pi * self.R) / 60.0
        v_r = (rpm_r * 2 * math.pi * self.R) / 60.0

        self.v = (v_l + v_r) / 2.0

        # filter noise kecil
        if abs(self.v) < 0.001:
            self.v = 0.0
    # =====================
    # IMU → ANGULAR SPEED
    # =====================
    def imu_callback(self, msg):
        omega = msg.angular_velocity.z

        # filter noise kecil
        if abs(omega) < 0.01:
            omega = 0.0

        self.omega = omega

    # =====================
    # UPDATE LOOP
    # =====================
    def update_odom(self):

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0:
            return

        # update pose
        self.theta += self.omega * dt

        # normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        # quaternion
        theta_vis = self.theta
        q = Quaternion()
        q.w = math.cos(theta_vis / 2)
        q.z = math.sin(theta_vis / 2)
        q.x = 0.0
        q.y = 0.0

        # =====================
        # ODOM MESSAGE
        # =====================
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega

        # covariance (IMU dipercaya untuk rotasi)
        odom.pose.covariance = [
            0.2, 0, 0, 0, 0, 0,
            0, 0.2, 0, 0, 0, 0,
            0, 0, 99999, 0, 0, 0,
            0, 0, 0, 99999, 0, 0,
            0, 0, 0, 0, 99999, 0,
            0, 0, 0, 0, 0, 0.05   # yaw lebih dipercaya (IMU)
        ]

        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

        # =====================
        # TF
        # =====================
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
