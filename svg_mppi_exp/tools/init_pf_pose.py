#!/usr/bin/env python3
"""pose_source:=pf 로 실험할 때 시작 위치를 particle filter 에 알려준다.

매뉴얼의 "RViz 2D Pose Estimate 로 시작 위치 지정"을 자동으로 하는 것과 같다.
시뮬레이터의 실제 위치(/car_state/odom_GT)를 한 번 읽어 /initialpose 로 보낸다.
(컨트롤러는 이 값을 쓰지 않는다. 시작할 때 측위를 초기화하는 용도뿐)
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def main():
    rclpy.init()
    node = rclpy.create_node("init_pf_pose")
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    got = {}
    node.create_subscription(Odometry, "/car_state/odom_GT", lambda m: got.setdefault("odom", m), 10)
    t0 = time.time()
    while "odom" not in got and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "odom" not in got:
        print("init_pf_pose: /car_state/odom_GT 를 받지 못함", flush=True)
        sys.exit(1)
    m = got["odom"]
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose = m.pose.pose
    msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
    msg.pose.covariance[35] = 0.005
    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.3)
    p = m.pose.pose.position
    print(f"init_pf_pose: /initialpose -> ({p.x:.2f}, {p.y:.2f})", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
