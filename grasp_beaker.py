#!/usr/bin/env python3
"""양팔 집게로 누운 비커의 입구(림) 유리벽을 물어서 들기 (IK + /tf + /clock + /joint_states).

씬에서 비커는 로봇 앞에 누워 있고 입구가 로봇을 향한다(build_chem_lab_scene.py).
서버 ROS2 env(isaac_ros)에서 실행. 필요한 것: numpy, so101_kinematics.py (같은 폴더).

집게: 고정 손가락은 입구 안쪽, 움직이는 Jaw 는 유리벽 바깥쪽. Jaw + = 벌림, - = 닫힘.
왼팔(L)은 입구 왼쪽 위, 오른팔(R)은 오른쪽 위를 문다(입구 윗점에서 --phi 도).
집게 끝은 --tilt 도 아래로 기울여 넣는다(수평은 팔이 바닥까지 못 내려감, 오프라인 계산).

단계 (각 단계는 시뮬레이션 시간 --seg 초 동안 부드럽게 보간):
  open      집게 벌리기 (팔은 그대로)
  pregrasp  입구 앞 --approach m 에 집게 대기
  insert    입구 안으로 --depth m 까지 넣기
  close     집게 닫기 (유리벽을 물면 Jaw 가 목표까지 못 가고 멈춤)
  lift      --lift m 을 --lift-steps 구간으로 나눠 수직으로 들기

사용법:
  python grasp_beaker.py --dry-run            # IK 해만 계산해서 출력 (안 움직임)
  python grasp_beaker.py --until pregrasp     # 대기 자세까지만
  python grasp_beaker.py                      # 전체
  python grasp_beaker.py --home               # 양팔·집게 0 rad 로
"""
import argparse
import functools
import math
import sys
import time

import numpy as np

import so101_kinematics as kin

print = functools.partial(print, flush=True)

STAGES = ["open", "pregrasp", "insert", "close", "lift"]
BEAKER_H = 0.0755          # bottom (TF origin of /Beaker) to rim, along the beaker's local +Z
SIDES = {"L": +1.0, "R": -1.0}


def quat_rotate(q, v, inverse=False):
    """Rotate v by unit quaternion q=(x, y, z, w) (or by its inverse)."""
    x, y, z, w = q
    if inverse:
        x, y, z = -x, -y, -z
    cx, cy, cz = y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return np.array([v[0] + 2 * (w * cx + ccx), v[1] + 2 * (w * cy + ccy), v[2] + 2 * (w * cz + ccz)])


def beaker_in_base(base, beaker):
    """(mouth centre, beaker axis from bottom to mouth) in base_link."""
    (bp, bq), (kp, kq) = base, beaker
    bottom = quat_rotate(bq, kp - bp, inverse=True)
    axis = quat_rotate(bq, quat_rotate(kq, np.array([0.0, 0.0, 1.0])), inverse=True)
    return bottom + BEAKER_H * axis, axis


def plan_grasp(mouth, start, args):
    """IK for every arm pose. Returns (plan, ok); plan[side][stage] = list of (theta, pos err m, ang err deg, target)."""
    tilt, phi = math.radians(args.tilt), math.radians(args.phi)
    plan, ok = {}, True
    for side, s in SIDES.items():
        radial = np.array([0.0, s * math.sin(phi), math.cos(phi)])
        rim = mouth + args.rim_r * radial
        targets = {
            "pregrasp": [rim + np.array([-args.approach, 0, 0])],
            "insert": [rim + np.array([args.depth, 0, 0])],
            "lift": [rim + np.array([args.depth, 0, args.lift * (k + 1) / args.lift_steps]) for k in range(args.lift_steps)],
        }
        seed = start[side]
        plan[side] = {}
        for stage in STAGES:
            if stage not in targets:
                continue
            plan[side][stage] = []
            for tgt in targets[stage]:
                th, pe, ae = kin.ik_grip_best(side, tgt, radial, seeds=[seed] + kin.SEEDS, tilt=tilt)
                plan[side][stage].append((th, pe, ae, tgt))
                seed = th
                ok &= pe < 0.003 and ae < 3.0
    return plan, ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--until", choices=STAGES, default="lift", help="이 단계까지만 실행")
    ap.add_argument("--dry-run", action="store_true", help="IK 결과만 출력")
    ap.add_argument("--home", action="store_true", help="양팔·집게 모든 관절 0 rad 로")
    ap.add_argument("--seg", type=float, default=3.0, help="단계(구간)당 시뮬레이션 시간 s")
    ap.add_argument("--tilt", type=float, default=35.0, help="집게 끝을 아래로 기울이는 각도 deg")
    ap.add_argument("--phi", type=float, default=65.0, help="무는 위치: 입구 윗점에서 옆으로 deg (90=옆면)")
    ap.add_argument("--rim-r", type=float, default=0.030, help="무는 유리벽 반지름 m (안 0.0294 / 바깥 0.0307)")
    ap.add_argument("--approach", type=float, default=0.04, help="대기 자세: 입구 앞 거리 m")
    ap.add_argument("--depth", type=float, default=0.006, help="입구 안으로 넣는 깊이 m (림 두께 2 mm)")
    ap.add_argument("--jaw-open", type=float, default=25.0, help="벌린 집게 각도 deg")
    ap.add_argument("--jaw-close", type=float, default=-30.0, help="닫기 목표 deg (벽에 막혀 그 전에 멈춤)")
    ap.add_argument("--lift", type=float, default=0.10, help="들어올릴 높이 m")
    ap.add_argument("--lift-steps", type=int, default=5, help="들어올리기를 나눌 수직 구간 수")
    ap.add_argument("--real-timeout", type=float, default=1800.0)
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
                state["beaker"] = (p, q)

    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    node.create_subscription(Clock, "/clock", lambda m: state.update(clock=m.clock.sec + m.clock.nanosec * 1e-9), 10)
    node.create_subscription(JointState, "/joint_states", lambda m: state.update(js=m), 10)
    pub = node.create_publisher(JointState, "/isaac_joint_command", 10)

    def missing():
        return [k for k, v in state.items() if v is None]  # `None in` would compare numpy arrays

    t_real = time.time()
    while missing() and time.time() - t_real < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if missing():
        print(f"[grasp] 입력이 없습니다: {missing()}. 씬 Play·ROS_DOMAIN_ID=42 확인.")
        return 1

    def measured(side):
        js = state["js"]
        return np.array([js.position[js.name.index(n)] for n in kin.joint_names(side)])

    def measured_jaw(side):
        js = state["js"]
        return js.position[js.name.index(f"{side}_Jaw")]

    names = kin.joint_names("L") + ["L_Jaw"] + kin.joint_names("R") + ["R_Jaw"]

    def send(th_l, jaw_l, th_r, jaw_r):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(v) for v in th_l] + [float(jaw_l)] + [float(v) for v in th_r] + [float(jaw_r)]
        pub.publish(msg)

    def mouth_z():
        return beaker_in_base(state["base"], state["beaker"])[0][2]

    cmd = {"L": None, "R": None, "jaw": None}   # last commanded pose (holds the squeeze instead of re-reading)

    def move(target, jaw_target, label):
        start = {s: (cmd[s] if cmd[s] is not None else measured(s)) for s in SIDES}
        jaw_start = cmd["jaw"] if cmd["jaw"] is not None else {s: measured_jaw(s) for s in SIDES}
        t0 = state["clock"]
        while time.time() - t_real < args.real_timeout:
            rclpy.spin_once(node, timeout_sec=0.03)
            a = min(1.0, (state["clock"] - t0) / args.seg)
            a = a * a * (3 - 2 * a)  # smoothstep
            pose = {s: start[s] + a * (target[s] - start[s]) for s in SIDES}
            jaw = {s: jaw_start[s] + a * (jaw_target[s] - jaw_start[s]) for s in SIDES}
            send(pose["L"], jaw["L"], pose["R"], jaw["R"])
            if state["clock"] - t0 >= args.seg + 0.5:  # hold half a second at the end
                break
        cmd.update(L=target["L"], R=target["R"], jaw=jaw_target)
        err = {s: np.degrees(np.abs(measured(s) - target[s])).max() for s in SIDES}
        jaws = {s: math.degrees(measured_jaw(s)) for s in SIDES}
        print(f"[grasp] {label:10s} 완료: 팔 관절 최대 오차 L {err['L']:.1f}° R {err['R']:.1f}°, "
              f"집게 L {jaws['L']:+.1f}° R {jaws['R']:+.1f}°, 입구 높이 {mouth_z():+.4f} m")
        return jaws

    if args.home:
        move({"L": np.zeros(5), "R": np.zeros(5)}, {"L": 0.0, "R": 0.0}, "home")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    mouth, axis = beaker_in_base(state["base"], state["beaker"])
    print(f"[grasp] 비커 입구 중심 (base_link): 앞 {mouth[0]:.3f} m, 옆 {mouth[1]:+.3f} m, 높이 {mouth[2]:+.3f} m, "
          f"축(바닥→입구) {np.round(axis, 2)}")
    if axis[0] > -0.9 or abs(axis[2]) > 0.3:
        print("[grasp] 입구가 로봇을 향해 누워 있지 않습니다(축이 약 [-1, 0, 0] 이어야 함). 씬을 다시 여세요.")
        return 1

    stages = STAGES[: STAGES.index(args.until) + 1]
    plan, ok = plan_grasp(mouth, {s: measured(s) for s in SIDES}, args)
    for stage in ("pregrasp", "insert", "lift"):
        for k in range(len(plan["L"][stage])):
            row = []
            for side in SIDES:
                th, pe, ae, tgt = plan[side][stage][k]
                row.append(f"{side} 목표 {np.round(tgt, 3)} 오차 {pe * 1000:.1f}mm {ae:.1f}° θ° {np.round(np.degrees(th), 0)}")
            print(f"[grasp] {stage:9s} " + " | ".join(row))
    if not ok:
        print("[grasp] IK 오차가 3 mm / 3° 를 넘는 자세가 있어 움직이지 않습니다. 비커 거리나 --tilt/--phi 를 조정하세요.")
        return 1
    if args.dry_run:
        print("[grasp] dry-run: 움직이지 않고 종료")
        return 0

    jaw_open = {s: math.radians(args.jaw_open) for s in SIDES}
    jaw_close = {s: math.radians(args.jaw_close) for s in SIDES}
    z_start = mouth_z()
    here = {s: measured(s) for s in SIDES}
    for stage in stages:
        if stage == "open":
            move(here, jaw_open, "open")
        elif stage == "close":
            jaws = move(cmd if cmd["L"] is not None else here, jaw_close, "close")
            for s in SIDES:
                if jaws[s] < args.jaw_close + 3:
                    print(f"[grasp] {s} 집게가 끝까지 닫혔습니다 → 유리벽을 못 문 것 같습니다.")
        else:
            jaw = jaw_close if stage == "lift" else jaw_open
            n = len(plan["L"][stage])
            for k in range(n):
                label = stage if n == 1 else f"{stage}{k + 1}/{n}"
                move({s: plan[s][stage][k][0] for s in SIDES}, jaw, label)
    if "lift" in stages:
        rise = mouth_z() - z_start
        print(f"[grasp] 입구 높이 변화 {rise * 100:+.1f} cm → {'성공' if rise > 0.5 * args.lift else '못 들었음'}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
