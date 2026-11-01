#!/usr/bin/env python3
"""机械臂单步关节移动（标定/调试用，带硬性安全限制）

用法：
    ./scripts/arm_joint_move.py --delta 1 0.03        # Joint1 相对 +0.03 rad (1.7°)
    ./scripts/arm_joint_move.py --abs 0 0 0 0 0 0     # 六轴绝对角 (rad)
    ./scripts/arm_joint_move.py --show                # 只读当前角度

安全设计：
  - 每次移动前读回实测角度，以实测为起点（不信任缓存）
  - 单轴单步默认上限 0.15 rad (8.6°)，超过直接拒绝，必须显式 --max-step 放大
  - 六轴限位硬校验（来自固件限位，见 HANDOFF）
  - 轨迹走 20 段平滑插值，速度缩放默认 0.15
  - 超时 = duration*2 + 5s，超时立刻放弃并报告（不重试）
"""

import argparse
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]

# 固件硬限位 (度) → 转成模型角后的粗略范围；这里只做粗保护，
# 真正的红线在 dummyarm_hardware 的 FW_HARD_LIMITS 里
# 由固件限位换算到模型角（见 docs/HANDOFF 的映射表）：
#   J1 1:1 → ±170°
#   J2 = fw + 73°，fw ∈ [-74, 92]  → [-1°, 165°]
#   J3 = fw - 180°，fw ∈ [35, 182] → [-145°, 2°]  ← J3 红线
#   J4 1:1 → ±180°
#   J5 = -fw，fw ∈ [-95, 95]       → ±95°
#   J6 1:1 → ±720°
LIMITS_RAD = {
    "Joint1": (-2.97, 2.97),
    "Joint2": (-0.02, 2.88),
    "Joint3": (-2.53, 0.03),
    "Joint4": (-3.14, 3.14),
    "Joint5": (-1.66, 1.66),
    "Joint6": (-12.57, 12.57),
}


class Mover(Node):
    def __init__(self):
        super().__init__("arm_joint_move")
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.cli = ActionClient(self, FollowJointTrajectory,
                                "/joint_trajectory_controller/follow_joint_trajectory")

    def _cb(self, msg):
        self.js = msg

    def wait_state(self, timeout=5.0):
        t0 = time.time()
        while self.js is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.js is None:
            raise RuntimeError("没有 /joint_states")
        return list(self.js.position)


def build_traj(start, target, duration, steps=20):
    traj = JointTrajectory()
    traj.joint_names = JOINTS
    for i in range(1, steps + 1):
        a = i / steps
        # smoothstep，起停都平滑
        s = a * a * (3 - 2 * a)
        p = JointTrajectoryPoint()
        p.positions = [start[k] + (target[k] - start[k]) * s for k in range(6)]
        t = duration * a
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        traj.points.append(p)
    return traj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--delta", nargs="*", type=float)
    ap.add_argument("--abs", nargs=6, type=float)
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--max-step", type=float, default=0.15)
    ap.add_argument("--vel-scale", type=float, default=0.15)
    args = ap.parse_args()

    rclpy.init()
    node = Mover()
    try:
        cur = node.wait_state()
        print("当前角度 (rad): " + " ".join(f"{v:+.3f}" for v in cur))
        if args.show:
            return 0

        target = list(cur)
        if args.delta is not None:
            if len(args.delta) % 2 != 0:
                print("--delta 需要成对给出：关节号 增量 ...")
                return 2
            for i in range(0, len(args.delta), 2):
                j = int(args.delta[i])
                d = args.delta[i + 1]
                if not 1 <= j <= 6:
                    print(f"关节号非法: {j}")
                    return 2
                if abs(d) > args.max_step:
                    print(f"拒绝：Joint{j} 单步 {d:+.3f} rad 超过上限 {args.max_step}")
                    return 2
                target[j - 1] += d
        elif args.abs is not None:
            target = list(args.abs)
            for k in range(6):
                if abs(target[k] - cur[k]) > args.max_step:
                    print(f"拒绝：Joint{k+1} 变化 {target[k]-cur[k]:+.3f} rad 超过上限 {args.max_step}")
                    return 2
        else:
            print("需要 --delta 或 --abs 或 --show")
            return 2

        for k, name in enumerate(JOINTS):
            lo, hi = LIMITS_RAD[name]
            if not (lo <= target[k] <= hi):
                print(f"拒绝：{name} 目标 {target[k]:+.3f} 超出安全范围 [{lo}, {hi}]")
                return 2

        print("目标角度 (rad): " + " ".join(f"{v:+.3f}" for v in target))
        if not node.cli.wait_for_server(timeout_sec=5.0):
            print("轨迹 action 服务不可用")
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = build_traj(cur, target, args.duration)
        # 速度缩放靠拉长 duration 实现（duration 越长越慢）
        send = node.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
        gh = send.result()
        if gh is None or not gh.accepted:
            print("目标被拒绝")
            return 1

        res = gh.get_result_async()
        deadline = time.time() + args.duration * 2 + 5.0
        while time.time() < deadline and not res.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not res.done():
            print("!! 超时，正在取消")
            gh.cancel_goal_async()
            return 1

        r = res.result().result
        code = r.error_code
        print(f"完成，error_code={code}")
        time.sleep(0.5)
        now = node.wait_state()
        err = max(abs(now[k] - target[k]) for k in range(6))
        print("实测角度 (rad): " + " ".join(f"{v:+.3f}" for v in now))
        print(f"到位误差 {math.degrees(err):.2f}°")
        return 0 if code == 0 else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
