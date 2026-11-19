#!/usr/bin/env python3
"""상황별 파라미터 자동 변경 노드 (학습 B안의 "주행 중" 부분).

/drive 를 내지 않는다. 표(yaml)에 적힌 대로 /svg_mppi 의 파라미터만 바꾼다 -> 컨트롤러는 여전히 SVG-MPPI 하나.
  - 구간(sector): 레이스라인 s 범위별 값
  - 장애물: /mppi/guide_offset 이 0이 아니면(가이드가 장애물을 피하는 중) obstacle 값을 덮어씀
표의 값은 사람이 정해도 되고, tune.py(Optuna)로 학습한 결과(best_schedule.yaml)를 그대로 넣어도 된다.

사용 (컨테이너):
  python3 param_scheduler.py --ros-args -p table:=/home/misys/shared_dir/svg_mppi_exp/configs/schedule_example.yaml

표 형식:
  default:  {reference_sdf.reference_speed_scale: 1.0}
  sectors:
    - {name: hairpin, s: [21.0, 27.0], params: {reference_sdf.reference_speed_scale: 0.85}}
  obstacle: {guide.speed_scale: 0.9}
노드 쪽 파라미터(reference_sdf.*, reference_speed, guide.*)는 즉시 반영된다.
solver 파라미터(svg_mppi.*, common.*)도 되지만 바뀔 때마다 solver 를 새로 만들어 웜스타트가 초기화되므로 구간마다 바꾸는 것은 비추천.
"""
import json
import math
import os
import sys

import rclpy
import yaml
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def to_param(name, value):
    v = ParameterValue()
    if isinstance(value, bool):
        v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
    elif isinstance(value, int):
        v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, value
    elif isinstance(value, float):
        v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, value
    else:
        v.type, v.string_value = ParameterType.PARAMETER_STRING, str(value)
    return Parameter(name=name, value=v)


# node parameters of svg_mppi that are doubles (yaml may say 1 instead of 1.0)
INT_PARAMS = {"reference_sdf.num_waypoints", "svg_mppi.sample_batch_num", "svg_mppi.guide_sample_num",
              "svg_mppi.num_svgd_iteration", "svg_mppi.sample_num_for_grad_estimation", "common.prediction_step_size",
              "common.thread_num", "forward_mppi.sample_batch_num"}
BOOL_PARAMS = {"guide.enable", "reference_sdf.local_search", "constant_speed_mode", "drive_publish_enabled"}


def normalize(params):
    out = {}
    for k, v in (params or {}).items():
        if k in BOOL_PARAMS:
            out[k] = bool(v)
        elif k in INT_PARAMS:
            out[k] = int(v)
        else:
            out[k] = float(v)
    return out


class ParamScheduler(Node):
    def __init__(self):
        super().__init__("param_scheduler")
        table = self.declare_parameter("table", "").value
        wp = self.declare_parameter(
            "waypoints_file", os.path.expanduser("~/forza_ws/race_stack/stack_master/maps/berlin/global_waypoints.json")).value
        self.target = self.declare_parameter("target_node", "/svg_mppi").value
        self.obs_hold = self.declare_parameter("obstacle_hold_s", 0.5).value
        self.load_table(table)
        w = json.load(open(wp))["global_traj_wpnts_iqp"]["wpnts"]
        self.wx = [p["x_m"] for p in w]
        self.wy = [p["y_m"] for p in w]
        self.ws = [p["s_m"] for p in w]
        self.idx = None
        self.obstacle_until = 0.0
        self.applied = {}
        self.cli = self.create_client(SetParameters, self.target + "/set_parameters")
        self.create_subscription(Odometry, "/car_state/odom", self.on_odom, 10)
        self.create_subscription(Float32, "/mppi/guide_offset", self.on_guide, 10)
        self.state = None
        self.create_timer(0.05, self.tick)
        self.pose = None

    def load_table(self, path):
        t = yaml.safe_load(open(path)) if path else {}
        self.default = normalize(t.get("default"))
        self.sectors = [(s["name"], float(s["s"][0]), float(s["s"][1]), normalize(s.get("params"))) for s in t.get("sectors", [])]
        self.obstacle = normalize(t.get("obstacle"))
        self.get_logger().info(f"table: {len(self.sectors)} sectors, default={self.default}, obstacle={self.obstacle}")

    def on_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_guide(self, m):
        if abs(m.data) > 0.05:
            self.obstacle_until = self.get_clock().now().nanoseconds * 1e-9 + self.obs_hold

    def s_of(self, x, y):
        n = len(self.wx)
        rng = range(n) if self.idx is None else [(self.idx + k) % n for k in range(-20, 40)]
        best = min(rng, key=lambda i: (self.wx[i] - x) ** 2 + (self.wy[i] - y) ** 2)
        if self.idx is not None and (self.wx[best] - x) ** 2 + (self.wy[best] - y) ** 2 > 2.0:
            best = min(range(n), key=lambda i: (self.wx[i] - x) ** 2 + (self.wy[i] - y) ** 2)
        self.idx = best
        return self.ws[best]

    def tick(self):
        if self.pose is None or not self.cli.service_is_ready():
            return
        s = self.s_of(*self.pose)
        sector = next((name for name, a, b, _ in self.sectors if a <= s < b), "default")
        obstacle = self.get_clock().now().nanoseconds * 1e-9 < self.obstacle_until
        state = (sector, obstacle)
        if state == self.state:
            return
        want = dict(self.default)
        for name, a, b, p in self.sectors:
            if name == sector:
                want.update(p)
        if obstacle:
            want.update(self.obstacle)
        change = {k: v for k, v in want.items() if self.applied.get(k) != v}
        self.state = state
        if not change:
            return
        req = SetParameters.Request(parameters=[to_param(k, v) for k, v in change.items()])
        self.cli.call_async(req)
        self.applied.update(change)
        self.get_logger().info(f"s={s:.1f} sector={sector} obstacle={obstacle} -> {change}")


def main():
    rclpy.init(args=sys.argv)
    rclpy.spin(ParamScheduler())


if __name__ == "__main__":
    main()
