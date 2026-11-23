#!/usr/bin/env python3
"""학습한 파라미터 표를 주행 중 /svg_mppi 에 넣는 노드.

[11/23] 최종 평가는 run 사이에도 설정을 바꿀 수 없어 train_eval.sh(tune_eval.py)가 설정 파일 하나를 학습한다.
그 방식에서는 이 노드를 쓰지 않는다. (obstacle.guide.speed_scale 은 guide.speed_scale 과 같은 값: 원래 guide 가
기준 경로를 옮긴 곳에서만 쓰이므로 설정 파일에 그냥 적으면 된다.) 이 노드는 이전 방식(train.sh)용으로만 남겨 둔다.

/drive 를 내지 않는다. 표(yaml)의 값으로 /svg_mppi 의 파라미터만 바꾼다 -> 컨트롤러는 여전히 SVG-MPPI 하나.
  - default : 시작할 때 한 번 넣는 값 (전체 속도 배율, guide 여유, λ 등)
  - obstacle: guide 가 장애물을 피하는 중(/mppi/guide_offset != 0)일 때만 덮어쓰는 값
트랙 구간·좌표·웨이포인트 번호에 따라 값을 바꾸는 기능은 넣지 않는다 (최종 평가 규칙: 구간별 하드코딩 감점).
구간 차이가 필요하면 노드 안의 일반 규칙(예: guide.max_curvature, 레이스라인 곡률 기준)으로 처리한다.

사용 (컨테이너):
  python3 param_scheduler.py --ros-args -p table:=$HOME/shared_dir/svg_mppi_exp/tuning/<study>_best_schedule.yaml

표 형식:
  default:  {reference_sdf.reference_speed_scale: 1.1, guide.max_curvature: 0.65}
  obstacle: {guide.speed_scale: 0.9}
노드 쪽 파라미터(reference_sdf.*, reference_speed, guide.*)는 즉시 반영된다.
solver 파라미터(svg_mppi.*, common.*)는 바뀔 때 solver 를 새로 만들므로 default 에만 두는 것을 권장.
"""
import sys

import rclpy
import yaml
from rclpy.node import Node
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


# svg_mppi parameters that are integers / booleans (everything else is sent as double; yaml may say 1 instead of 1.0)
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
        self.target = self.declare_parameter("target_node", "/svg_mppi").value
        self.obs_hold = self.declare_parameter("obstacle_hold_s", 0.5).value
        self.load_table(yaml.safe_load(open(table)) if table else {})
        self.obstacle_until = 0.0
        self.applied = {}
        self.state = None
        self.cli = self.create_client(SetParameters, self.target + "/set_parameters")
        self.create_subscription(Float32, "/mppi/guide_offset", self.on_guide, 10)
        self.create_timer(0.05, self.tick)

    def load_table(self, t):
        if t.get("sectors"):
            raise ValueError("track-section (sector) tables are not supported: per-section hard-coded parameters "
                             "are penalized in the final evaluation. Use default/obstacle only.")
        self.default = normalize(t.get("default"))
        self.obstacle = normalize(t.get("obstacle"))
        self.state = None  # force re-apply
        self.get_logger().info(f"table: default={self.default}, obstacle={self.obstacle}")

    def on_guide(self, m):
        if abs(m.data) > 0.05:
            self.obstacle_until = self.get_clock().now().nanoseconds * 1e-9 + self.obs_hold

    def tick(self):
        if not self.cli.service_is_ready():
            return
        obstacle = self.get_clock().now().nanoseconds * 1e-9 < self.obstacle_until
        if obstacle == self.state:
            return
        want = dict(self.default)
        if obstacle:
            want.update(self.obstacle)
        change = {k: v for k, v in want.items() if self.applied.get(k) != v}
        self.state = obstacle
        if not change:
            return
        self.cli.call_async(SetParameters.Request(parameters=[to_param(k, v) for k, v in change.items()]))
        self.applied.update(change)
        self.get_logger().info(f"obstacle={obstacle} -> {change}")


def main():
    rclpy.init(args=sys.argv)
    rclpy.spin(ParamScheduler())


if __name__ == "__main__":
    main()
