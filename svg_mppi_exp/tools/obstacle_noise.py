#!/usr/bin/env python3
"""장애물 배치 노이즈 (평가 때 교수님이 넣을 "장애물 노이즈" 대비 실험용).

race_manager 의 obstacle_spawner 출력(/sim/obstacles_raw 로 이름을 바꿔 띄움)을 받아,
배치가 바뀔 때마다 장애물마다 위치(가우시안, 2σ 에서 자름)와 회전을 무작위로 흔들어 /sim/obstacles 로 낸다.
시뮬레이터(gym_bridge)는 /sim/obstacles 를 보고 라이다와 충돌을 계산하므로, 흔든 위치가 "실제" 위치가 된다.

파라미터:
  pos_sigma  [m]  위치 노이즈 표준편차 (0 이면 위치 그대로)
  rot        bool 회전을 0~90도 무작위로 (정사각형이라 90도 주기)
  dynamic_sigma [m] 0 보다 크면 배치가 그대로여도 매 발행(1 Hz)마다 이만큼 추가로 흔듦 (움직이는 장애물)
  wall_clear [m]  흔든 장애물과 벽 사이 최소 거리 (넘으면 다시 뽑음)
  seed       int  노이즈 난수 seed (race_manager seed 와 별개)
"""
import math
import os
import random
import sys

import numpy as np
import rclpy
import yaml
from PIL import Image
from geometry_msgs.msg import Pose, PoseArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from scipy.ndimage import distance_transform_edt
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
SIZE = 0.30


class ObstacleNoise(Node):
    def __init__(self):
        super().__init__("obstacle_noise")
        self.pos_sigma = float(self.declare_parameter("pos_sigma", 0.10, ParameterDescriptor(dynamic_typing=True)).value)
        self.rot = bool(self.declare_parameter("rot", True, ParameterDescriptor(dynamic_typing=True)).value)
        self.dyn = float(self.declare_parameter("dynamic_sigma", 0.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.wall_clear = float(self.declare_parameter("wall_clear", 0.05, ParameterDescriptor(dynamic_typing=True)).value)
        seed = int(self.declare_parameter("seed", 0, ParameterDescriptor(dynamic_typing=True)).value)
        map_yaml = self.declare_parameter(
            "map_yaml", os.path.expanduser("~/forza_ws/race_stack/stack_master/maps/berlin/berlin.yaml")).value
        self.rng = random.Random(seed)

        meta = yaml.safe_load(open(map_yaml))
        img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta["image"])).convert("L"), dtype=float) / 255.0
        free = (1.0 - img) < meta["free_thresh"]
        self.dist = distance_transform_edt(free) * meta["resolution"]  # [m] to nearest non-free cell
        self.res = meta["resolution"]
        self.ox, self.oy = meta["origin"][0], meta["origin"][1]
        self.H = img.shape[0]

        self.raw_key = None
        self.base = []      # jittered layout for the current raw layout: (x, y, theta)
        self.pub = self.create_publisher(PoseArray, "/sim/obstacles", LATCHED)
        self.pub_m = self.create_publisher(MarkerArray, "/sim/obstacles/markers", LATCHED)
        self.create_subscription(PoseArray, "/sim/obstacles_raw", self.on_raw, LATCHED)
        self.get_logger().info(f"obstacle noise: pos_sigma={self.pos_sigma} rot={self.rot} dynamic={self.dyn} wall_clear={self.wall_clear}")

    def wall_dist(self, x, y):
        c = int((x - self.ox) / self.res)
        r = self.H - 1 - int((y - self.oy) / self.res)
        if r < 0 or c < 0 or r >= self.dist.shape[0] or c >= self.dist.shape[1]:
            return 0.0
        return self.dist[r, c]

    def fits(self, x, y, th):
        # corners and edge midpoints of the square must keep wall_clear from walls
        h = SIZE / 2
        for u, v in ((h, h), (h, -h), (-h, h), (-h, -h), (h, 0), (-h, 0), (0, h), (0, -h), (0, 0)):
            px = x + u * math.cos(th) - v * math.sin(th)
            py = y + u * math.sin(th) + v * math.cos(th)
            if self.wall_dist(px, py) < self.wall_clear:
                return False
        return True

    def jitter(self, x, y, th, sigma):
        for _ in range(50):
            dx = max(-2 * sigma, min(2 * sigma, self.rng.gauss(0, sigma))) if sigma > 0 else 0.0
            dy = max(-2 * sigma, min(2 * sigma, self.rng.gauss(0, sigma))) if sigma > 0 else 0.0
            nth = self.rng.uniform(0, math.pi / 2) if self.rot else th
            if self.fits(x + dx, y + dy, nth):
                return x + dx, y + dy, nth
        return x, y, th  # could not place: keep the original pose

    def on_raw(self, msg):
        raw = [(p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)) for p in msg.poses]
        key = tuple((round(x, 3), round(y, 3)) for x, y, _ in raw)
        if key != self.raw_key:  # new layout -> new noise
            self.raw_key = key
            self.base = [self.jitter(x, y, th, self.pos_sigma) for x, y, th in raw]
            moved = ", ".join(f"{math.hypot(bx - x, by - y):.2f}" for (x, y, _), (bx, by, _) in zip(raw, self.base))
            self.get_logger().info(f"new layout: {len(raw)} obstacles, moved [m]: {moved}")
        if self.dyn > 0:  # moving obstacles: small extra jitter on every publish, rotation kept
            rot, self.rot = self.rot, False
            out = [self.jitter(x, y, th, self.dyn) for x, y, th in self.base]
            self.rot = rot
        else:
            out = list(self.base)
        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        for i, (x, y, th) in enumerate(out):
            p = Pose()
            p.position.x, p.position.y = x, y
            p.orientation.z, p.orientation.w = math.sin(th / 2), math.cos(th / 2)
            pa.poses.append(p)
            m = Marker()
            m.header = pa.header
            m.ns, m.id, m.type, m.action = "obstacles", i, Marker.CUBE, Marker.ADD
            m.pose = p
            m.scale.x = m.scale.y = SIZE
            m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.35, 0.15, 0.9
            ma.markers.append(m)
        self.pub.publish(pa)
        self.pub_m.publish(ma)


def main():
    rclpy.init(args=sys.argv)
    rclpy.spin(ObstacleNoise())


if __name__ == "__main__":
    main()
