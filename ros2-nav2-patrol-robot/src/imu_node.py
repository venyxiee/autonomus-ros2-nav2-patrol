import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
import adafruit_bno055
import math


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        self.get_logger().info("IMU Node FIXED (AXIS + ORIENTATION)")

    def publish_imu(self):

        msg = Imu()
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = "imu_link"

        # =========================
        # GYRO (REMAP KE ROS)
        # =========================
        gyro = self.sensor.gyro

        if gyro is not None:
            gx = gyro[0]
            gy = gyro[1]
            gz = gyro[2]

            # 🔥 REMAP (BNO055 → ROS)
            msg.angular_velocity.x = gy
            msg.angular_velocity.y = -gx
            msg.angular_velocity.z = gz

            if abs(msg.angular_velocity.z) < 0.01:
                msg.angular_velocity.z = 0.0

        # =========================
        # ACCEL (REMAP KE ROS)
        # =========================
        accel = self.sensor.acceleration

        if accel is not None:
            ax = accel[0]
            ay = accel[1]
            az = accel[2]

            msg.linear_acceleration.x = ay
            msg.linear_acceleration.y = -ax
            msg.linear_acceleration.z = az

        # =========================
        # ORIENTATION (WAJIB!)
        # =========================
        quat = self.sensor.quaternion

        if quat is not None:
            # BNO055 format: (w, x, y, z)
            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]
        else:
            msg.orientation.w = 1.0

        # =========================
        # COVARIANCE
        # =========================
        msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.02
        ]

        msg.angular_velocity_covariance = [
            0.02, 0, 0,
            0, 0.02, 0,
            0, 0, 0.02
        ]

        msg.linear_acceleration_covariance = [
            0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.1
        ]

        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
