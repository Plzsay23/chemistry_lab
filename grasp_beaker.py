#!/usr/bin/env python3
"""양팔로 누운 비커를 양옆에서 감싸 들기 (위치 IK + /tf + /clock + /joint_states).

approach_beaker.py 로 비커 중심이 base_link 앞 약 0.32 m 에 온 뒤 실행한다.
서버 ROS2 env(isaac_ros)에서 실행. 필요한 것: numpy, so101_kinematics.py (같은 폴더).

단계 (각 단계는 시뮬레이션 시간 --seg 초 동안 관절 각도를 부드럽게 보간):
  pregrasp  두 집게를 비커 양옆, 비커보다 --above m 위로
  descend   비커 높이까지 내리기
  squeeze   안쪽으로 조여 비커를 끼우기 (반지름보다 --squeeze m 안쪽을 목표로)
  lift      --lift m 들어올리기

비커는 축이 로봇 전방(+X)으로 누워 있어서, 왼팔(L)은 +Y 쪽 옆면, 오른팔(R)은 -Y 쪽 옆면을 잡는다.

사용법:
  python grasp_beaker.py --dry-run            # IK 해만 계산해서 출력 (팔 안 움직임)
  python grasp_beaker.py --until pregrasp     # 첫 단계만 (팔 방향 확인용)
  python grasp_beaker.py                      # 전체
  python grasp_beaker.py --home               # 양팔 0 rad 로 되돌리기
"""
import argparse
import functools
import sys
import time

import numpy as np

import so101_kinematics as kin

print = functools.partial(print, flush=True)

STAGES = ["pregrasp", "descend", "squeeze", "lift"]
BEAKER_R = 0.034
SIDES = {"L": +1.0, "R": -1.0}


def quat_rotate_inv(q, v):
    x, y, z, w = q
    x, y, z = -x, -y, -z
    cx, cy, cz = y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return np.array([v[0] + 2 * (w * cx + ccx), v[1] + 2 * (w * cy + ccy), v[2] + 2 * (w * cz + ccz)])


def waypoints(beaker, args):
    """Tool targets in base_link for each stage and arm."""
    bx, by, bz = beaker
    z0 = bz + args.z_offset
    wp = {}
    for side, s in SIDES.items():
        open_y = by + s * (BEAKER_R + args.clear)
        grip_y = by + s * (BEAKER_R - args.squeeze)
        wp[side] = {
            "pregrasp": np.array([bx, open_y, z0 + args.above]),
            "descend": np.array([bx, open_y, z0]),
            "squeeze": np.array([bx, grip_y, z0]),
            # lift is split into straight-up sub-steps so the squeeze is held all the way
            "lift": [np.array([bx, grip_y, z0 + args.lift * (k + 1) / args.lift_steps]) for k in range(args.lift_steps)],
        }
    return wp


def solve_plan(wp, start, stages):
    """plan[side][stage] = list of (theta, err), one per sub-step (1 except lift)."""
    plan, ok = {}, True
    for side in SIDES:
        seed = start[side]
        plan[side] = {}
        for i, stage in enumerate(stages):
            targets = wp[side][stage] if isinstance(wp[side][stage], list) else [wp[side][stage]]
            plan[side][stage] = []
            for tgt in targets:
                th, err = kin.ik_best(side, tgt, seeds=[seed] + kin.SEEDS)
                plan[side][stage].append((th, err))
                seed = th
                if err > 0.01:
                    ok = False
    return plan, ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--until", choices=STAGES, default="lift", help="이 단계까지만 실행")
    ap.add_argument("--dry-run", action="store_true", help="IK 결과만 출력")
    ap.add_argument("--home", action="store_true", help="양팔 모든 관절 0 rad 로")
    ap.add_argument("--seg", type=float, default=3.0, help="단계당 시뮬레이션 시간 s")
    ap.add_argument("--clear", type=float, default=0.04, help="준비 자세에서 비커 옆면과의 간격 m")
    ap.add_argument("--above", type=float, default=0.08, help="준비 자세 높이 (비커 중심 위) m")
    ap.add_argument("--squeeze", type=float, default=0.006, help="조일 때 옆면보다 안쪽 목표 m")
    ap.add_argument("--lift", type=float, default=0.10, help="들어올릴 높이 m")
    ap.add_argument("--lift-steps", type=int, default=5, help="들어올리기를 나눌 수직 구간 수")
    ap.add_argument("--z-offset", type=float, default=0.01, help="집게 목표 높이 = 비커 중심 + 이 값 m")
    ap.add_argument("--real-timeout", type=float, default=900.0)
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = Node("grasp_beaker")
    state = {"clock": None, "base": None, "beaker": None, "js": None}

    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id != "world":
                continue
            p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            q = (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            if t.child_frame_id == "base_link":
                state["base"] = (p, q)
            elif t.child_frame_id == "Beaker":
                state["beaker"] = p

    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    node.create_subscription(Clock, "/clock", lambda m: state.update(clock=m.clock.sec + m.clock.nanosec * 1e-9), 10)
    node.create_subscription(JointState, "/joint_states", lambda m: state.update(js=m), 10)
    pub = node.create_publisher(JointState, "/isaac_joint_command", 10)

    t_real = time.time()
    def missing():
        return [k for k, v in state.items() if v is None]  # `None in` would compare numpy arrays

    while missing() and time.time() - t_real < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if missing():
        print(f"[grasp] 입력이 없습니다: {missing()}. 씬 Play·ROS_DOMAIN_ID=42 확인.")
        return 1

    def measured(side):
        js = state["js"]
        return np.array([js.position[js.name.index(n)] for n in kin.joint_names(side)])

    names = kin.joint_names("L") + ["L_Jaw"] + kin.joint_names("R") + ["R_Jaw"]

    def send(th_l, th_r):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(v) for v in th_l] + [0.0] + [float(v) for v in th_r] + [0.0]
        pub.publish(msg)

    def move(target_l, target_r, label):
        start_l, start_r = measured("L"), measured("R")
        t0 = state["clock"]
        while time.time() - t_real < args.real_timeout:
            rclpy.spin_once(node, timeout_sec=0.03)
            a = min(1.0, (state["clock"] - t0) / args.seg)
            a = a * a * (3 - 2 * a)  # smoothstep
            send(start_l + a * (target_l - start_l), start_r + a * (target_r - start_r))
            if state["clock"] - t0 >= args.seg + 0.5:  # hold half a second at the end
                break
        err_l = np.degrees(np.abs(measured("L") - target_l)).max()
        err_r = np.degrees(np.abs(measured("R") - target_r)).max()
        print(f"[grasp] {label:9s} 완료: 관절 최대 오차 L {err_l:.1f}° R {err_r:.1f}°, 비커 z(world) {state['beaker'][2]:+.4f}")

    if args.home:
        move(np.zeros(5), np.zeros(5), "home")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    base_p, base_q = state["base"]
    beaker = quat_rotate_inv(base_q, state["beaker"] - base_p)
    print(f"[grasp] 비커 중심 (base_link): 앞 {beaker[0]:.3f} m, 옆 {beaker[1]:+.3f} m, 높이 {beaker[2]:+.3f} m")
    stages = STAGES[: STAGES.index(args.until) + 1]
    wp = waypoints(beaker, args)
    plan, ok = solve_plan(wp, {s: measured(s) for s in SIDES}, stages)
    for stage in stages:
        for k in range(len(plan["L"][stage])):
            row = []
            for side in SIDES:
                th, err = plan[side][stage][k]
                tgt = wp[side][stage][k] if isinstance(wp[side][stage], list) else wp[side][stage]
                row.append(f"{side} tool {np.round(tgt, 3)} err {err * 100:.2f}cm θ° {np.round(np.degrees(th), 0)}")
            print(f"[grasp] {stage:9s} " + " | ".join(row))
    if not ok:
        print("[grasp] IK 오차가 1 cm 를 넘는 단계가 있어 움직이지 않습니다. 비커 거리(approach --target)를 조정하세요.")
        return 1
    if args.dry_run:
        print("[grasp] dry-run: 움직이지 않고 종료")
        return 0

    z_start = state["beaker"][2]
    for stage in stages:
        n = len(plan["L"][stage])
        for k in range(n):
            label = stage if n == 1 else f"{stage}{k + 1}/{n}"
            move(plan["L"][stage][k][0], plan["R"][stage][k][0], label)
    if "lift" in stages:
        rise = state["beaker"][2] - z_start
        print(f"[grasp] 비커 높이 변화 {rise * 100:+.1f} cm → {'성공' if rise > 0.5 * args.lift else '못 들었음'}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
