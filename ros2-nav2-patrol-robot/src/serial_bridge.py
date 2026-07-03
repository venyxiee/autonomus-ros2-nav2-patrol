import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial

# ============================================================
# KONFIGURASI SCALING
#
# MAX_LINEAR_VEL: kecepatan linear max robot (m/s)
#   JGA25-370 R=0.05m → v_max aman ~0.47 m/s
#   Set 0.5 → Nav2 kirim 0.26 m/s = 52% → PWM ~133 ✅
#
# MAX_ANGULAR_VEL: kecepatan angular max (rad/s)
#   Joystick scale_angular = 1.0 rad/s
#   Set 1.0 → joystick full = 100% → PWM 255 ✅
#   Nav2 max_vel_theta = 1.0 rad/s → juga 100% ✅
#
# CATATAN: Arduino sekarang memproses linear dan angular
# secara TERPISAH (tidak digabung base+turn),
# sehingga scaling masing-masing independen.
# ============================================================
MAX_LINEAR_VEL  = 0.5   # m/s
MAX_ANGULAR_VEL = 1.0   # rad/s — samakan dengan max_vel_theta Nav2


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.pub_rpm = self.create_publisher(
            Float32MultiArray,
            '/wheel_rpm',
            10
        )

        self.timer = self.create_timer(0.02, self.read_serial)
        self.get_logger().info(
            f"SerialBridge ready | "
            f"MAX_LINEAR={MAX_LINEAR_VEL} m/s | "
            f"MAX_ANGULAR={MAX_ANGULAR_VEL} rad/s"
        )

    def cmd_callback(self, msg):
        # Normalisasi m/s → -1.0 hingga 1.0
        linear  = msg.linear.x  / MAX_LINEAR_VEL
        angular = msg.angular.z / MAX_ANGULAR_VEL

        # Clamp ke [-1.0, 1.0]
        linear  = max(min(linear,  1.0), -1.0)
        angular = max(min(angular, 1.0), -1.0)

        data = f"{linear:.2f},{angular:.2f}\n"
        self.ser.write(data.encode())
    def cmd_callback(self, msg):
        # Normalisasi m/s → -1.0 hingga 1.0
        linear  = msg.linear.x  / MAX_LINEAR_VEL
        angular = msg.angular.z / MAX_ANGULAR_VEL

        # Clamp ke [-1.0, 1.0]
        linear  = max(min(linear,  1.0), -1.0)
        angular = max(min(angular, 1.0), -1.0)

        data = f"{linear:.2f},{angular:.2f}\n"
        self.ser.write(data.encode())

    def read_serial(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                if ',' in line:
                    rpm_l, rpm_r = line.split(',')
                    msg = Float32MultiArray()
                    msg.data = [float(rpm_l), float(rpm_r)]
                    self.pub_rpm.publish(msg)
            except Exception:
                pass


def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
