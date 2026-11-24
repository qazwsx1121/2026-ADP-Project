#!/usr/bin/env python3
"""pose_source:=pf 로 실험할 때 시작 위치를 particle filter 에 알려준다.

매뉴얼의 "RViz 2D Pose Estimate 로 시작 위치 지정"을 자동으로 하는 것과 같다.
시뮬레이터의 실제 위치(/car_state/odom_GT)를 한 번 읽어 /initialpose 로 보낸다.
(컨트롤러는 이 값을 쓰지 않는다. 시작할 때 측위를 초기화하는 용도뿐)

[11/24] 보낸 메시지가 사라지는 일이 있었다 (particle filter 가 아직 구독 전이거나, 컴퓨터가 잠깐 멈춤).
그러면 particle filter 는 지도 전체에서 위치를 찾다가 엉뚱한 곳(14 m 밖)에 자리 잡았다.
이제 particle filter 가 구독할 때까지 기다리고, 보낸 뒤 /pf/viz/inferred_pose 가 1 m 안으로 들어왔는지 확인해서
아니면 다시 보낸다 (최대 5번).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def spin_until(node, cond, timeout):
    t0 = time.monotonic()  # the system clock can jump (WSL2)
    while not cond() and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    return cond()


def main():
    rclpy.init()
    node = rclpy.create_node("init_pf_pose")
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    got = {}
    node.create_subscription(Odometry, "/car_state/odom_GT", lambda m: got.setdefault("odom", m), 10)
    node.create_subscription(PoseStamped, "/pf/viz/inferred_pose", lambda m: got.__setitem__("pf", m), 10)
    if not spin_until(node, lambda: "odom" in got, 30.0):
        print("init_pf_pose: /car_state/odom_GT 를 받지 못함", flush=True)
        sys.exit(1)
    if not spin_until(node, lambda: pub.get_subscription_count() > 0, 15.0):
        print("init_pf_pose: /initialpose 구독자가 없음", flush=True)
    m = got["odom"]
    p = m.pose.pose.position
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose = m.pose.pose
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
    msg.pose.covariance[35] = 0.005

    def pf_err():
        q = got.get("pf")
        return math.hypot(q.pose.position.x - p.x, q.pose.position.y - p.y) if q else float("inf")

    for attempt in range(1, 6):
        for _ in range(3):
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.3)
        got.pop("pf", None)
        if spin_until(node, lambda: pf_err() < 1.0, 3.0):
            print(f"init_pf_pose: /initialpose -> ({p.x:.2f}, {p.y:.2f}), particle filter {pf_err():.2f} m "
                  f"(attempt {attempt})", flush=True)
            break
        print(f"init_pf_pose: attempt {attempt}: particle filter still {pf_err():.1f} m away, sending again", flush=True)
    else:
        print(f"init_pf_pose: /initialpose -> ({p.x:.2f}, {p.y:.2f}) NOT confirmed (pf {pf_err():.1f} m away)", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
