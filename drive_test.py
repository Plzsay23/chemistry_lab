#!/usr/bin/env python3
"""메카넘 베이스 개루프 주행 테스트.

서버 ROS2 env(isaac_ros)에서 실행. Isaac 쪽 ActionGraph 의
ROS2SubscribeJointState(topic: isaac_joint_command) -> ArticulationController 로
바퀴 4개에 속도(rad/s)를 직접 보낸다.

바퀴 조인트 4개 모두 축이 base +Y, 로컬 회전 동일 ->
  같은 부호  = 전진(+) / 후진(-)
  좌 -, 우 + = 제자리 좌회전(반시계, base +Z)
바퀴 반지름 0.05 m 이므로 지면 속도 v[m/s] = 0.05 * w[rad/s].

사용법:
  python drive_test.py                 # 기본: 3.0 rad/s (0.15 m/s) 로 4 초 전진
  python drive_test.py --vel 3 --dur 4
  python drive_test.py --turn 3 --dur 2   # 제자리 좌회전
  python drive_test.py --vel 0 --turn 0 --dur 1   # 정지 명령만
"""
import argparse
import functools
import sys
import time

print = functools.partial(print, flush=True)

WHEELS = [
    "left_front_wheel_joint",
    "left_rear_wheel_joint",
    "right_front_wheel_joint",
    "right_rear_wheel_joint",
]
SIGN_TURN = [-1.0, -1.0, +1.0, +1.0]  # 좌회전(+turn): 왼쪽 뒤로, 오른쪽 앞으로
WHEEL_RADIUS = 0.05


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vel", type=float, default=3.0, help="전진 바퀴 각속도 rad/s (+전진)")
    ap.add_argument("--turn", type=float, default=0.0, help="회전 성분 rad/s (+좌회전)")
    ap.add_argument("--dur", type=float, default=4.0, help="명령 유지 시간 s")
    ap.add_argument("--rate", type=float, default=30.0, help="퍼블리시 Hz")
    ap.add_argument("--topic", default="/isaac_joint_command")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    print("[drive] rclpy import OK")
    rclpy.init()
    node = Node("drive_test")
    pub = node.create_publisher(JointState, args.topic, 10)
    print("[drive] publisher on %s" % args.topic)

    # Isaac 쪽 구독자가 붙을 때까지 잠깐 기다린다(없어도 계속 진행).
    for _ in range(30):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    print("[drive] subscriber count = %d" % pub.get_subscription_count())
    if pub.get_subscription_count() == 0:
        print("[drive] !! 구독자가 없다. Isaac 에서 씬을 열고 Play 를 눌렀는지,")
        print("[drive]    ROS_DOMAIN_ID 가 양쪽 다 42 인지 확인할 것.")

    def send(vel_cmd, turn_cmd):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(WHEELS)
        msg.velocity = [vel_cmd + turn_cmd * s for s in SIGN_TURN]
        msg.position = []
        msg.effort = []
        pub.publish(msg)
        return msg.velocity

    v = send(args.vel, args.turn)
    print("[drive] 명령 %s" % ["%.2f" % x for x in v])
    print("[drive] 지면 속도 약 %.3f m/s, %.1f 초 -> 약 %.2f m"
          % (args.vel * WHEEL_RADIUS, args.dur, args.vel * WHEEL_RADIUS * args.dur))

    period = 1.0 / args.rate
    t0 = time.time()
    n = 0
    while time.time() - t0 < args.dur:
        send(args.vel, args.turn)
        n += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)
    print("[drive] %d 회 퍼블리시, 정지 명령 보냄" % n)

    # 정지: 0 을 여러 번 보내 확실히 멈춘다.
    for _ in range(15):
        send(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()
    print("[drive] done")


if __name__ == "__main__":
    sys.exit(main())
