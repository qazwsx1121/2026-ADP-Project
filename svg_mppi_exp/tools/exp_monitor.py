#!/usr/bin/env python3
"""실험 기록 + 자동 복구 노드 (실험 도구일 뿐, 차 제어에는 관여하지 않음).

- 40 Hz로 trace.csv 기록: 위치/속도/명령/MPPI 계산시간/충돌률/레이스라인 기준 s, d
- 충돌(/sim/collision != none)이 hold_s 이상 지속되거나 stuck_s 동안 거의 안 움직이면
  레이스라인 위 back_m 뒤 지점으로 /initialpose 를 보내 복구 (race_manager README의 운영자 복구와 같은 방식).
  복구는 events.csv 에 남고, scorer 는 이것을 teleport 로 기록한다.
"""
import csv
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from visualization_msgs.msg import MarkerArray


class ExpMonitor(Node):
    def __init__(self):
        super().__init__("exp_monitor")
        self.out_dir = self.declare_parameter("out_dir", "/tmp/exp").value
        wpnts_file = self.declare_parameter("waypoints_file", "").value
        self.recover = self.declare_parameter("recover", True).value
        self.hold_s = self.declare_parameter("collision_hold_s", 1.0).value
        self.stuck_s = self.declare_parameter("stuck_s", 4.0).value
        self.back_m = self.declare_parameter("back_m", 1.0).value
        self.ahead_m = self.declare_parameter("ahead_m", 1.5).value
        self.last_rec = None
        self.rec_streak = 0
        self.noprog_s = self.declare_parameter("no_progress_s", 15.0).value
        self.prog = None          # unwrapped forward progress along the raceline [m]
        self.prev_s = None
        self.best_prog = -1e9
        self.best_prog_t = None
        self.best_prog_idx = 0
        os.makedirs(self.out_dir, exist_ok=True)

        w = json.load(open(wpnts_file))["global_traj_wpnts_iqp"]["wpnts"]
        self.wx = [p["x_m"] for p in w]
        self.wy = [p["y_m"] for p in w]
        self.ws = [p["s_m"] for p in w]
        self.n = len(w)

        self.trace = open(os.path.join(self.out_dir, "trace.csv"), "w", newline="")
        self.tw = csv.writer(self.trace)
        self.tw.writerow(["t", "x", "y", "yaw", "v", "steer_cmd", "speed_cmd", "calc_ms", "coll_rate", "cost",
                          "collision", "idx", "s", "d", "best_end_x", "best_end_y", "guide_offset"])
        self.events = open(os.path.join(self.out_dir, "events.csv"), "w", newline="")
        self.ew = csv.writer(self.events)
        self.ew.writerow(["t", "event", "x", "y", "s", "info"])

        self.t0 = time.monotonic()
        self.odom = None
        self.steer = self.speed = self.calc = self.crate = self.cost = float("nan")
        self.coll = "none"
        self.coll_since = None
        self.best_end = (float("nan"), float("nan"))
        self.last_move_pos = None
        self.last_move_t = None
        self.cooldown_until = 0.0
        self.obstacles = []
        self.guide_offset = 0.0
        self.last_gobs_t = -9.0

        self.create_subscription(Odometry, "/car_state/odom_GT", self.on_odom, 10)
        self.create_subscription(AckermannDriveStamped, "/drive", self.on_drive, 10)
        self.create_subscription(Float32, "/mppi/calculation_time", lambda m: setattr(self, "calc", m.data), 10)
        self.create_subscription(Float32, "/mppi/collision_rate", lambda m: setattr(self, "crate", m.data), 10)
        self.create_subscription(Float32, "/mppi/cost", lambda m: setattr(self, "cost", m.data), 10)
        self.create_subscription(String, "/sim/collision", self.on_coll, 10)
        self.create_subscription(Path, "/mppi/best_path", self.on_best, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseArray, "/sim/obstacles", self.on_obs, latched)
        self.create_subscription(Float32, "/mppi/guide_offset", lambda m: setattr(self, "guide_offset", m.data), 10)
        self.create_subscription(MarkerArray, "/mppi/guide_obstacles", self.on_gobs, 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_timer(0.025, self.tick)

    # ---------------- callbacks ----------------
    def now(self):
        return time.monotonic() - self.t0

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw, m.twist.twist.linear.x)

    def on_drive(self, m):
        self.steer = m.drive.steering_angle
        self.speed = m.drive.speed

    def on_coll(self, m):
        if m.data != self.coll:
            if m.data != "none" and self.odom:
                i, s, d = self.frenet(self.odom[0], self.odom[1])
                self.event("collision", self.odom[0], self.odom[1], s, m.data)
            self.coll_since = self.now() if m.data != "none" else None
        self.coll = m.data

    def on_best(self, m):
        if m.poses:
            p = m.poses[-1].pose.position
            self.best_end = (p.x, p.y)

    def on_obs(self, m):
        obs = [(p.position.x, p.position.y) for p in m.poses]
        if obs != self.obstacles:
            self.obstacles = obs
            self.event("obstacles", float("nan"), float("nan"), float("nan"),
                       ";".join("%.2f,%.2f,s=%.1f" % (x, y, self.frenet(x, y)[1]) for x, y in obs))

    def on_gobs(self, m):
        pts = [p for mk in m.markers for p in mk.points]
        if not pts or self.now() - self.last_gobs_t < 0.5:
            return
        self.last_gobs_t = self.now()
        cx = sum(p.x for p in pts) / len(pts)
        cy = sum(p.y for p in pts) / len(pts)
        car = "%.2f,%.2f" % (self.odom[0], self.odom[1]) if self.odom else "?"
        self.event("guide_obstacles", cx, cy, self.frenet(cx, cy)[1], f"n={len(pts)} car={car}")

    # ---------------- helpers ----------------
    def frenet(self, x, y):
        best, bi = 1e9, 0
        for i in range(self.n):
            d2 = (self.wx[i] - x) ** 2 + (self.wy[i] - y) ** 2
            if d2 < best:
                best, bi = d2, i
        j = (bi + 1) % self.n
        tx, ty = self.wx[j] - self.wx[bi], self.wy[j] - self.wy[bi]
        cross = tx * (y - self.wy[bi]) - ty * (x - self.wx[bi])
        d = math.copysign(math.sqrt(best), cross)  # + = left of raceline
        return bi, self.ws[bi], d

    def event(self, kind, x, y, s, info=""):
        self.ew.writerow(["%.3f" % self.now(), kind, "%.3f" % x, "%.3f" % y, "%.2f" % s, info])
        self.events.flush()
        self.get_logger().info(f"{kind} at s={s:.1f} ({x:.2f},{y:.2f}) {info[:80]}")

    def do_recover(self, reason):
        x, y = self.odom[0], self.odom[1]
        i, s, _ = self.frenet(x, y)
        # same place again (within 2 m, 30 s)? -> the car is trapped by this layout: put it past the spot
        same = (self.last_rec is not None and abs(s - self.last_rec[0]) < 2.0 and self.now() - self.last_rec[1] < 30.0)
        self.rec_streak = self.rec_streak + 1 if same else 1
        self.last_rec = (s, self.now())
        if self.rec_streak >= 2:
            k = (i + int(round(self.ahead_m / 0.1))) % self.n
            reason += f" (x{self.rec_streak} here -> skip ahead)"
            for _ in range(20):  # move forward until clear of obstacles
                if all(math.hypot(self.wx[k] - ox, self.wy[k] - oy) > 0.6 for ox, oy in self.obstacles):
                    break
                k = (k + 3) % self.n
            self.place(k, x, y, s, reason)
            return
        k = (i - int(round(self.back_m / 0.1))) % self.n
        # do not put the car on top of an obstacle
        for _ in range(20):
            if all(math.hypot(self.wx[k] - ox, self.wy[k] - oy) > 0.6 for ox, oy in self.obstacles):
                break
            k = (k - 3) % self.n
        self.place(k, x, y, s, reason)

    def place(self, k, x, y, s, reason):
        j = (k + 1) % self.n
        yaw = math.atan2(self.wy[j] - self.wy[k], self.wx[j] - self.wx[k])
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.wx[k]
        msg.pose.pose.position.y = self.wy[k]
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.pub_init.publish(msg)
        self.event("recover", x, y, s, f"{reason} -> s={self.ws[k]:.1f}")
        self.cooldown_until = self.now() + 3.0
        self.coll_since = None
        self.last_move_pos, self.last_move_t = None, None

    def tick(self):
        if self.odom is None:
            return
        x, y, yaw, v = self.odom
        i, s, d = self.frenet(x, y)
        self.tw.writerow(["%.3f" % self.now(), "%.3f" % x, "%.3f" % y, "%.3f" % yaw, "%.3f" % v,
                          "%.4f" % self.steer, "%.3f" % self.speed, "%.2f" % self.calc, "%.3f" % self.crate,
                          "%.1f" % self.cost, self.coll, i, "%.2f" % s, "%.3f" % d,
                          "%.3f" % self.best_end[0], "%.3f" % self.best_end[1], "%.3f" % self.guide_offset])
        if not self.recover or self.now() < self.cooldown_until or self.speed != self.speed:
            return  # no /drive yet (nan) -> controller not started, do nothing
        t = self.now()
        # progress along the track (unwrapped); a car that loops / U-turns in front of a blocked spot makes none
        L = self.ws[-1] + math.hypot(self.wx[0] - self.wx[-1], self.wy[0] - self.wy[-1])
        if self.prev_s is not None:
            ds = s - self.prev_s
            if ds > L / 2:
                ds -= L
            elif ds < -L / 2:
                ds += L
            self.prog += ds
        else:
            self.prog = 0.0
        self.prev_s = s
        if self.best_prog_t is None or self.prog > self.best_prog + 0.5:
            self.best_prog, self.best_prog_t, self.best_prog_idx = self.prog, t, i
        elif t - self.best_prog_t > self.noprog_s:
            k = (self.best_prog_idx + int(round(self.ahead_m / 0.1))) % self.n
            for _ in range(20):
                if all(math.hypot(self.wx[k] - ox, self.wy[k] - oy) > 0.6 for ox, oy in self.obstacles):
                    break
                k = (k + 3) % self.n
            self.place(k, x, y, s, f"no progress for {self.noprog_s:.0f} s (looping?) -> past furthest point")
            self.best_prog_t = None
            self.prev_s = None
            return
        if self.coll != "none" and self.coll_since is not None and t - self.coll_since > self.hold_s:
            self.do_recover("collision:" + self.coll)
            return
        if self.last_move_pos is None or math.hypot(x - self.last_move_pos[0], y - self.last_move_pos[1]) > 0.3:
            self.last_move_pos, self.last_move_t = (x, y), t
        elif t - self.last_move_t > self.stuck_s:
            self.do_recover("stuck")

    def close(self):
        self.trace.close()
        self.events.close()


def main():
    rclpy.init(args=sys.argv)
    node = ExpMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.close()


if __name__ == "__main__":
    main()
