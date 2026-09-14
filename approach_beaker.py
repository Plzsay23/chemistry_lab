#!/usr/bin/env python3
"""비커 앞으로 다가가기 (TF 피드백 폐루프).

서버 ROS2 env(isaac_ros)에서 실행. 씬의 FeedbackGraph 가 내보내는
  /tf   world -> base_link, world -> Beaker
  /clock 시뮬레이션 시간
을 읽어서 /isaac_joint_command 로 바퀴 속도를 보낸다.

1) 제자리 회전: 비커가 로봇 정면(base_link +X)에 오도록
2) 전진: 비커 중심이 base_link 앞 --target m 에 올 때까지 (방향 보정하면서)
3) 정지

바퀴에 롤러가 없어 옆으로는 못 간다(스키드 스티어). 좌/우 바퀴 속도 차이로 회전.
양팔 도달 계산상 비커 중심이 base_link 앞 0.30~0.35 m 일 때 양옆을 잡을 수 있다.

사용법:
  python approach_beaker.py                 # 기본 목표 0.32 m
  python approach_beaker.py --target 0.30
"""
import argparse
import functools
import math
import sys
import time

print = functools.partial(print, flush=True)

WHEELS_LEFT = ["left_front_wheel_joint", "left_rear_wheel_joint"]
WHEELS_RIGHT = ["right_front_wheel_joint", "right_rear_wheel_joint"]
WHEEL_RADIUS = 0.05   # m
HALF_TRACK = 0.185    # m, wheel y offset from base_link


def quat_rotate_inv(q, v):
    """Rotate vector v by the inverse of unit quaternion q=(x, y, z, w)."""
    x, y, z, w = q
    x, y, z = -x, -y, -z
    # v' = v + 2w(q x v) + 2 q x (q x v)
    cx = y * v[2] - z * v[1]
    cy = z * v[0] - x * v[2]
    cz = x * v[1] - y * v[0]
    ccx = y * cz - z * cy
    ccy = z * cx - x * cz
    ccz = x * cy - y * cx
    return (v[0] + 2 * (w * cx + ccx), v[1] + 2 * (w * cy + ccy), v[2] + 2 * (w * cz + ccz))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, default=0.32, help="멈출 때 비커 중심의 base_link 앞 거리 m")
    ap.add_argument("--max-speed", type=float, default=0.12, help="최대 전진 속도 m/s")
    ap.add_argument("--max-yaw", type=float, default=0.6, help="최대 회전 속도 rad/s")
    ap.add_argument("--sim-timeout", type=float, default=90.0, help="시뮬레이션 시간 상한 s")
    ap.add_argument("--real-timeout", type=float, default=600.0, help="실제 시간 상한 s")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = Node("approach_beaker")
    state = {"clock": None, "base": None, "beaker": None}

    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id != "world":
                continue
            p = (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
            q = (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            if t.child_frame_id == "base_link":
                state["base"] = (p, q)
            elif t.child_frame_id == "Beaker":
                state["beaker"] = p

    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    node.create_subscription(Clock, "/clock", lambda m: state.update(clock=m.clock.sec + m.clock.nanosec * 1e-9), 10)
    pub = node.create_publisher(JointState, "/isaac_joint_command", 10)

    def send(v, yaw_rate):
        wl = (v - yaw_rate * HALF_TRACK) / WHEEL_RADIUS
        wr = (v + yaw_rate * HALF_TRACK) / WHEEL_RADIUS
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = WHEELS_LEFT + WHEELS_RIGHT
        msg.velocity = [wl, wl, wr, wr]
        pub.publish(msg)

    def stop():
        for _ in range(15):
            send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.03)

    t_real = time.time()
    while None in (state["clock"], state["base"], state["beaker"]) and time.time() - t_real < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if None in (state["clock"], state["base"], state["beaker"]):
        missing = [k for k in ("clock", "base", "beaker") if state[k] is None]
        print(f"[approach] 입력이 없습니다: {missing}. 씬을 열고 Play 했는지, ROS_DOMAIN_ID=42 인지 확인하세요.")
        return 1

    def beaker_in_base():
        (bp, bq), kp = state["base"], state["beaker"]
        d = (kp[0] - bp[0], kp[1] - bp[1], kp[2] - bp[2])
        return quat_rotate_inv(bq, d)

    t0 = state["clock"]
    phase = "turn"
    last_print = -1.0
    result = 1
    while True:
        rclpy.spin_once(node, timeout_sec=0.03)
        sim_t = state["clock"] - t0
        if sim_t > args.sim_timeout or time.time() - t_real > args.real_timeout:
            print(f"[approach] 시간 초과 (시뮬 {sim_t:.1f}s) — 정지")
            break
        x, y, _ = beaker_in_base()
        heading = math.atan2(y, x)
        err = x - args.target

        if phase == "turn":
            if abs(heading) < math.radians(3):
                phase = "drive"
                print(f"[approach] 방향 맞춤 완료 (heading {math.degrees(heading):+.1f}°) → 전진")
                continue
            yaw = max(-args.max_yaw, min(args.max_yaw, 1.5 * heading))
            send(0.0, yaw)
        elif phase == "drive":
            if err <= 0.005:
                print(f"[approach] 도착: 비커 중심 앞 {x:.3f} m, 옆 {y:+.3f} m, heading {math.degrees(heading):+.1f}°")
                result = 0
                break
            v = max(0.02, min(args.max_speed, 0.8 * err))
            yaw = max(-args.max_yaw, min(args.max_yaw, 1.5 * heading))
            send(v, yaw)

        if sim_t - last_print >= 1.0:
            last_print = sim_t
            print(f"[approach] t={sim_t:5.1f}s {phase:5s} 비커 앞 {x:.3f} m 옆 {y:+.3f} m heading {math.degrees(heading):+.1f}°")

    stop()
    node.destroy_node()
    rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
