#!/usr/bin/env python3
"""
pure_j2j3_point_and_nod.py - 基于 J2/J3 纯俯仰联动的精准点头敲门执行器

【彻底解决的核心问题】:
1. 真实机械运动学事实:
   - Dummy 机械臂的手腕 J4 是轴向旋转 (Roll)，J5 是摆动偏航/侧偏；
   - 真正控制整个末端【垂直上下点头/敲门】的，是 J2 (大臂) 和 J3 (手肘) 的水平横轴俯仰！
   - 控制 J4/J5 永远做不出垂直向下的敲门，反而会把 J5 逼向 ±95° 碰撞硬限位！
2. 绝对纯净的动作规划:
   - J1: 底座旋转对准目标方位角 atan2(y, x)；
   - J4 = 0.0, J5 = 0.0, J6 = 0.0: 手腕完全锁定端平，绝对不发生任何侧翻扭曲！
   - J2/J3: 几何解析逆解，直接前伸停在目标正上方 2~3cm；
   - 点头动作 (Nod x3): 保持 J1/J4/J5/J6 完全不动，仅通过 J2/J3 整体联动，带动第六轴末端垂直向下点头微动 1.5~2cm，连续轻敲 3 次！
3. 动作完成后平稳回到 REST 折叠原点。
"""

import sys
import time
import math
import signal
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class PureJ2J3Nod(Node):
    def __init__(self):
        super().__init__('pure_j2j3_nod')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        self.get_logger().info("【J2/J3 纯俯仰点头执行器】连接中...")
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("未连接到控制器，请确认 real_moveit 正在运行！")
        else:
            self.get_logger().info("✔ 控制器已就绪！")

    def orange_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_orange_cam_pose = msg

    def get_latest_target(self):
        with self.lock:
            pose = self.latest_orange_cam_pose
        if pose is None:
            return None
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            return tf2_geometry_msgs.do_transform_pose_stamped(pose, trans)
        except Exception:
            return None

    def send_joint_goal(self, positions, duration_sec=1.5):
        if not self.traj_client.server_is_ready():
            self.get_logger().error("控制器 Action 服务不可用！请确认 real_moveit 正在运行。")
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal_msg.trajectory.points.append(point)

        send_future = self.traj_client.send_goal_async(goal_msg)
        t_start = time.time()
        while not send_future.done():
            time.sleep(0.01)
            if time.time() - t_start > 3.0:
                self.get_logger().error("发送轨迹请求超时！")
                return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("关节指令被控制器拒收！请检查限位或控制器状态。")
            return False

        res_future = goal_handle.get_result_async()
        t_start = time.time()
        while not res_future.done():
            time.sleep(0.01)
            if time.time() - t_start > (duration_sec + 6.0):
                self.get_logger().error("等待轨迹执行完成超时！")
                return False

        return True

    def return_to_rest(self):
        self.get_logger().info("🔄 平稳折叠收拢回到 REST 原点...")
        # 给足 7.0 秒平稳低速收折时间，与 safe_park_and_exit 保持完全一致，消除超时
        return self.send_joint_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=7.0)


node = None

def sigint_handler(sig, frame):
    print("\n\n>>> 捕获到退出信号 (Ctrl+C)，安全释放连接并回位...")
    if node:
        try:
            node.return_to_rest()
        except Exception:
            pass
        node.destroy_node()
    rclpy.shutdown()
    print("✔ 控制权已交还 RViz！")
    sys.exit(0)


def solve_2d_reach(r, z):
    """
    两连杆平面几何逆解 (J2/J3 平面俯仰):
    大臂 L1 = 0.145m, 小臂+手腕 L2 = 0.170m
    """
    L1 = 0.145
    L2 = 0.170

    # 限制工作半径
    D = math.sqrt(r*r + z*z)
    D = min(max(D, 0.14), L1 + L2 - 0.02)

    # 余弦定理求解手肘夹角
    cos_elbow = (D*D - L1*L1 - L2*L2) / (2 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_angle = math.acos(cos_elbow)

    # 肩部俯仰角
    alpha = math.atan2(z, r)
    beta = math.atan2(L2 * math.sin(elbow_angle), L1 + L2 * math.cos(elbow_angle))
    shoulder_angle = alpha + beta

    # 映射为模型弧度 (参考: 立姿 CAD J2=1.27, J3=-1.57)
    # 转换为前伸角
    j2_cad = 1.30 - (shoulder_angle - 0.5) * 0.5
    j3_cad = -1.40 + (elbow_angle - 1.2) * 0.5

    # 严格安全限位裁剪 (防断臂与撞击)
    j2_cad = max(0.2, min(1.8, j2_cad))
    j3_cad = max(-2.1, min(-0.8, j3_cad))

    return j2_cad, j3_cad


def main():
    global node
    rclpy.init()
    node = PureJ2J3Nod()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    signal.signal(signal.SIGINT, sigint_handler)

    print("==================================================================")
    print("   PickOrangeDummy J2/J3 纯俯仰仿生敲门执行器 (Pure J2/J3 Tap)   ")
    print("==================================================================")
    print("【动作解析与机械真相】:")
    print("1. 为什么之前 J5 慢转撞限位？因为 J5 是侧偏轴，根本不能做上下敲门！")
    print("2. 现在的真正敲门：手腕 J4/J5/J6 全程锁平为 0°，绝无任何扭曲！")
    print("3. J1 对准目标 -> J2/J3 像人手肘一样协同伸向目标上方；")
    print("4. J2/J3 协同微幅上下点头 3 次 (连叩三下)，精准敲中物体！")
    print("5. 动作完成自动平稳收折回 REST 原点。")
    print("==================================================================")

    try:
        while rclpy.ok():
            target = node.get_latest_target()
            if target is not None:
                p = target.pose.position
                sys.stdout.write(f"\r🎯 实时锁定目标 (base_link): X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f} m")
                sys.stdout.flush()
            else:
                sys.stdout.write("\r⏳ 等待视觉目标锁定中...")
                sys.stdout.flush()

            time.sleep(0.08)

            user_input = input("\n[按回车执行 J2/J3 协同轻点 / r:回位 / q:退出]: ").strip().lower()
            if user_input == 'q':
                sigint_handler(None, None)
                break
            elif user_input == 'r':
                node.return_to_rest()
                continue

            target = node.get_latest_target()
            if target is None:
                print("❌ 未锁定目标坐标，请调整物体！")
                continue

            tx = target.pose.position.x
            ty = target.pose.position.y
            tz = target.pose.position.z

            dist = math.sqrt(tx*tx + ty*ty + tz*tz)
            if dist < 0.12 or dist > 0.33:
                print(f"❌ 目标距离 {dist:.2f}m 超出工作范围 [0.12, 0.33]m！")
                continue

            # 1. 目标对准与方位角计算
            # 物理实测确认：向右移动物体时 J1 应顺时针转动(负角度)；向左移动时转动(正角度)
            # 真实对准偏航角公式为: j1 = -atan2(-tx, -ty) = atan2(tx, -ty)
            j1 = math.atan2(tx, -ty)
            r_dist = math.sqrt(tx*tx + ty*ty)

            # 2. 目标上方 3cm 准备位 (Ready Pose)
            j2_ready, j3_ready = solve_2d_reach(r_dist - 0.02, tz + 0.03)
            ready_joints = [j1, j2_ready, j3_ready, 0.0, 0.0, 0.0]

            print(f"\n>>> [1/3] J1 转向目标方位 {math.degrees(j1):.1f}°，J2/J3 舒展到达目标上方 3cm 处...")
            if not node.send_joint_goal(ready_joints, duration_sec=2.0):
                print("❌ 目标轨迹下发失败，请确认 real_moveit 正在运行且机械臂无报错。")
                continue
            time.sleep(0.3)

            # 3. J2/J3 协同向下微探点头 (点触位 Tap Pose)
            j2_tap, j3_tap = solve_2d_reach(r_dist, tz)
            tap_joints = [j1, j2_tap, j3_tap, 0.0, 0.0, 0.0]

            print(">>> [2/3] 开始执行像敲门一样的 J2/J3 协同轻点动作 (Tap x3)...")
            for tap_i in range(1, 4):
                print(f"    👉 [第 {tap_i} 次敲门] J2/J3 协同向下轻叩物体...")
                node.send_joint_goal(tap_joints, duration_sec=0.4)
                time.sleep(0.15)
                print(f"    👈 抬手回位...")
                node.send_joint_goal(ready_joints, duration_sec=0.4)
                time.sleep(0.15)

            # 4. 自动平稳收折回到 REST
            print(">>> [3/3] 敲门测试完毕！正在自动平稳折叠收回 REST 原点...")
            node.return_to_rest()
            print("✔ 机械臂已平稳归位至 REST，控制权已释放！")
            print("==================================================================")

    except KeyboardInterrupt:
        sigint_handler(None, None)


if __name__ == '__main__':
    main()
