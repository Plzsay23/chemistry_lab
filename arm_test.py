#!/usr/bin/env python3
"""SO-101 양팔 관절 위치 명령 테스트.

서버 ROS2 env(isaac_ros)에서 실행. /isaac_joint_command 로 팔 관절 위치(rad)를 보내고,
시뮬레이션 시간(/clock) 기준으로 기다린 뒤 /joint_states 의 실제 각도와 비교한다.
(GUI 가 느리면 시뮬 시간이 실제보다 느리게 흐르므로 실제 시계로 기다리지 않는다.)

관절 이름: L_/R_ + Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw

사용법:
  python arm_test.py                              # 양팔 Pitch 를 0.5 rad 로
  python arm_test.py --arm L --joint Elbow --pos -0.8
  python arm_test.py --arm both --joint Jaw --pos 1.0 --dur 3
  python arm_test.py --home                       # 양팔 모든 관절 0 rad
"""
import argparse
import functools
import sys
import time

print = functools.partial(print, flush=True)

ARM_JOINTS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=["L", "R", "both"], default="both")
    ap.add_argument("--joint", choices=ARM_JOINTS, default="Pitch")
    ap.add_argument("--pos", type=float, default=0.5, help="목표 각도 rad")
    ap.add_argument("--home", action="store_true", help="선택한 팔의 모든 관절을 0 rad 로")
    ap.add_argument("--dur", type=float, default=2.0, help="명령 유지 시간 (시뮬레이션 초)")
    ap.add_argument("--timeout", type=float, default=120.0, help="실제 시간 상한 s")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState

    arms = ["L", "R"] if args.arm == "both" else [args.arm]
    if args.home:
        names = [f"{a}_{j}" for a in arms for j in ARM_JOINTS]
        targets = [0.0] * len(names)
    else:
        names = [f"{a}_{args.joint}" for a in arms]
        targets = [args.pos] * len(names)

    rclpy.init()
    node = Node("arm_test")
    state = {"clock": None, "js": None}
    node.create_subscription(Clock, "/clock", lambda m: state.update(clock=m.clock.sec + m.clock.nanosec * 1e-9), 10)
    node.create_subscription(JointState, "/joint_states", lambda m: state.update(js=m), 10)
    pub = node.create_publisher(JointState, "/isaac_joint_command", 10)

    t_real = time.time()
    while (state["clock"] is None or state["js"] is None) and time.time() - t_real < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["clock"] is None or state["js"] is None:
        print("[arm] /clock 또는 /joint_states 가 안 들어옵니다. 씬을 열고 Play 했는지, ROS_DOMAIN_ID=42 인지 확인하세요.")
        return 1
    print(f"[arm] subscribers on /isaac_joint_command = {pub.get_subscription_count()}")

    def measured():
        js = state["js"]
        return [js.position[js.name.index(n)] if n in js.name else float("nan") for n in names]

    print("[arm] 시작 각도 " + ", ".join(f"{n}={v:+.3f}" for n, v in zip(names, measured())))
    print("[arm] 목표 각도 " + ", ".join(f"{n}={v:+.3f}" for n, v in zip(names, targets)))

    t0 = state["clock"]
    while state["clock"] - t0 < args.dur and time.time() - t_real < args.timeout:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = list(targets)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.03)
    sim_elapsed = state["clock"] - t0

    got = measured()
    print(f"[arm] 시뮬 {sim_elapsed:.2f}s 경과 (실제 {time.time() - t_real:.1f}s)")
    for n, tgt, v in zip(names, targets, got):
        print(f"[arm]   {n:14s} 목표 {tgt:+.3f}  실제 {v:+.3f}  오차 {v - tgt:+.3f} rad")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
