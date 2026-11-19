#!/usr/bin/env python3
"""라이다 노이즈 (실험용): /scan -> /scan_noisy.

svg_mppi 의 scan_topic 을 /scan_noisy 로 바꾸면 컨트롤러는 노이즈 섞인 스캔을 보고,
시뮬레이터의 실제 충돌 판정은 원래대로 계산된다. (시뮬레이터 기본 노이즈 σ=0.01 m 위에 더해짐)

파라미터:
  sigma [m]   거리 가우시안 노이즈 표준편차
  drop  [0-1] 빔이 사라질 확률 (range_max 보다 큰 값 = 측정 없음)
  ghost [0-1] 허상 점 확률 (실제보다 가까운 무작위 거리, 먼지·반사 흉내)
  seed  int
"""
import sys

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNoise(Node):
    def __init__(self):
        super().__init__("scan_noise")
        self.sigma = float(self.declare_parameter("sigma", 0.03, ParameterDescriptor(dynamic_typing=True)).value)
        self.drop = float(self.declare_parameter("drop", 0.02, ParameterDescriptor(dynamic_typing=True)).value)
        self.ghost = float(self.declare_parameter("ghost", 0.005, ParameterDescriptor(dynamic_typing=True)).value)
        self.rng = np.random.default_rng(int(self.declare_parameter("seed", 0, ParameterDescriptor(dynamic_typing=True)).value))
        self.pub = self.create_publisher(LaserScan, "/scan_noisy", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f"scan noise: sigma={self.sigma} drop={self.drop} ghost={self.ghost}")

    def on_scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float32)
        n = r.size
        if self.sigma > 0:
            r = r + self.rng.normal(0.0, self.sigma, n).astype(np.float32)
        if self.ghost > 0:
            g = self.rng.random(n) < self.ghost
            r[g] = (self.rng.uniform(0.2, 1.0, g.sum()) * r[g]).astype(np.float32)
        if self.drop > 0:
            r[self.rng.random(n) < self.drop] = np.float32(m.range_max + 1.0)
        r = np.clip(r, m.range_min, m.range_max + 1.0)
        m.ranges = r.tolist()
        self.pub.publish(m)


def main():
    rclpy.init(args=sys.argv)
    rclpy.spin(ScanNoise())


if __name__ == "__main__":
    main()
