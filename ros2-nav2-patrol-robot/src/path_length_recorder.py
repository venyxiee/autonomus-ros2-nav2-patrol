#!/usr/bin/env python3
"""
PATH LENGTH RECORDER
====================
Subscribe ke /plan topic dan hitung panjang path yang di-generate Nav2.
Setiap kali ada path baru, panjangnya dihitung dan disimpan ke file CSV.

Cara pakai:
  ros2 run my_robot path_length_recorder

Atau dengan nama file custom:
  ros2 run my_robot path_length_recorder --ros-args -p output_file:=/home/faldo-ivan/skripsi/data/path_P1_1.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import math
import csv
import os
from datetime import datetime


class PathLengthRecorder(Node):

    def __init__(self):
        super().__init__('path_length_recorder')

        # Parameter output file — bisa di-override via ros args
        self.declare_parameter(
            'output_file',
            os.path.expanduser('~/skripsi/data/path_lengths.csv')
        )
        self.output_file = self.get_parameter('output_file').value

        # Pastikan folder ada
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        # Tulis header CSV kalau file baru
        file_exists = os.path.isfile(self.output_file)
        self._csv_file = open(self.output_file, 'a', newline='')
        self._writer   = csv.writer(self._csv_file)
        if not file_exists:
            self._writer.writerow([
                'timestamp', 'path_index',
                'jumlah_poses', 'panjang_path_m'
            ])
            self._csv_file.flush()

        self._path_count = 0

        # Subscribe ke /plan
        self.create_subscription(
            Path,
            '/plan',
            self._path_callback,
            10
        )

        self.get_logger().info("=" * 55)
        self.get_logger().info("PATH LENGTH RECORDER aktif")
        self.get_logger().info(f"Subscribe ke : /plan")
        self.get_logger().info(f"Output file  : {self.output_file}")
        self.get_logger().info("Menunggu path dari Nav2...")
        self.get_logger().info("=" * 55)

    def _path_callback(self, msg):
        poses = msg.poses

        if len(poses) < 2:
            return

        # Hitung total panjang path
        total = 0.0
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            d  = math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2)
            total += d

        self._path_count += 1
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # Simpan ke CSV
        self._writer.writerow([
            ts,
            self._path_count,
            len(poses),
            round(total, 4)
        ])
        self._csv_file.flush()

        self.get_logger().info(
            f"📏 Path #{self._path_count} | "
            f"Poses: {len(poses)} | "
            f"Panjang: {total:.3f} m"
        )

    def destroy_node(self):
        # Tutup file CSV saat node mati
        if hasattr(self, '_csv_file'):
            self._csv_file.close()
            self.get_logger().info(
                f"✅ Data disimpan ke {self.output_file} "
                f"({self._path_count} path tercatat)"
            )
        super().destroy_node()


def main():
    rclpy.init()
    node = PathLengthRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
