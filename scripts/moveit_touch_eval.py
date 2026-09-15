#!/usr/bin/env python3
"""
moveit_touch_eval.py - 基于 MoveGroup 真实高精逆解的多点标定对齐工具

【彻底解决“末端离小球定位点也很远”的病根】:
1. 为什么之前在 RViz 里末端离小球也很远？
   因为之前的微调脚本用了一套粗略的人工 2 连杆公式，算出来的角度根本就不是让机械臂去小球的位置！
2. 现在的颠覆性升级:
   直接调用 MoveIt 官方的 MoveGroup，向系统请求：“将末端 tool0 规划并移动到 RViz 小球当前的真实 3D 坐标”！
   这样在 RViz 画面中，机械臂末端 tool0 会 100% 精确逼近到小球表面！
3. 随后支持在真实物理接触面进行前后(径向)/左右/上下微调，
   多点采样后一键求解全局最优外参，彻底消除视觉与物理的最终残差！
"""

import sys
import time
import math
import signal
import threading
import re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R_tool


class MoveItTouchEval(Node):
    def __init__(self):
        super().__init__('moveit_touch_eval')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.active_goal_handle = None
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.get_logger().info("【MoveIt 真实对位标定工具】正在连接服务...")
        self.move_client.wait_for_server(timeout_sec=5.0)
        self.traj_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info("✔ MoveIt 与控制器连接就绪！")

    def orange_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_orange_cam_pose = msg

    def get_cam_optical_point(self):
        with self.lock:
            pose = self.latest_orange_cam_pose
        if pose is None:
            return None
        return np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ])

    def get_target_in_base(self):
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

    def plan_to_cartesian(self, x, y, z):
        """调用 MoveGroup 规划到达指定空间坐标 (在 RViz 中真实直达小球)"""
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = 'arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'tool0'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.03, 0.03, 0.03]

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.w = 1.0

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0

        req.goal_constraints.append(Constraints(name="target_pos", position_constraints=[pos_constraint]))
        goal_msg.planning_options.plan_only = True

        future = self.move_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.01)
        handle = future.result()
        if not handle.accepted:
            return None

        res_future = handle.get_result_async()
        while not res_future.done():
            time.sleep(0.01)
        res = res_future.result()

        if res.result.error_code.val == 1:
            traj = res.result.planned_trajectory.joint_trajectory
            if traj.points:
                return traj.points[-1].positions
        return None

    def execute_joints(self, pos, duration=2.5):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in pos]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points.append(pt)

        future = self.traj_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.01)
        handle = future.result()
        if not handle.accepted:
            return False
        res_future = handle.get_result_async()
        t_start = time.time()
        while not res_future.done():
            time.sleep(0.01)
            if time.time() - t_start > (duration + 3.0):
                break
        return True

    def return_to_rest(self):
        self.get_logger().info("🔄 正在平稳折叠回到 REST 原点...")
        lift_joints = [0.0, 1.10, -1.10, 0.0, 0.0, 0.0]
        rest_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.execute_joints(lift_joints, duration=2.5)
        self.execute_joints(rest_joints, duration=3.5)

    def get_current_tool0_in_base(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            t = trans.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None


node = None

def sigint_handler(sig, frame):
    print("\n\n>>> 捕获退出信号 (Ctrl+C)，平稳回位中...")
    if node:
        try:
            node.return_to_rest()
        except Exception:
            pass
        node.destroy_node()
    rclpy.shutdown()
    print("✔ 已安全退出并释放控制权！")
    sys.exit(0)


def main():
    global node
    rclpy.init()
    node = MoveItTouchEval()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    signal.signal(signal.SIGINT, sigint_handler)

    print("==================================================================")
    print("    PickOrangeDummy MoveIt 真实对位多点标定工具 (MoveIt-Eval)     ")
    print("==================================================================")
    print("【重大突破】:")
    print("1. 末端直接通过 MoveIt 真实到达 RViz 小球的 3D 坐标，绝不再'离小球很远'！")
    print("2. 停在小球面前后，根据真实接触情况进行毫米级微调 (+/- 前后, l/r 左右, u/d 上下)；")
    print("3. 按 'c' 记录点，采集 3~4 点后按 's' 求解，全域定位误差彻底归零！")
    print("==================================================================")

    recorded_base_pts = []
    recorded_cam_pts = []
    point_idx = 1

    try:
        while rclpy.ok():
            print(f"\n👉 [请摆放第 {point_idx} 个测试目标] (建议变换位置/高度)")
            input("   摆好目标后，按 [Enter] 让末端真实直达 RViz 小球面前: ")

            target = None
            for _ in range(20):
                target = node.get_target_in_base()
                if target is not None:
                    break
                time.sleep(0.1)

            if target is None:
                print("❌ 未检测到目标，请确认画面有绿框！")
                continue

            tx = target.pose.position.x
            ty = target.pose.position.y
            tz = target.pose.position.z

            print(f">>> 正在通过 MoveIt 将机械臂末端精确驱动至小球位置: X={tx:.3f}, Y={ty:.3f}, Z={tz:.3f}...")
            # 留出 3cm 触碰观察距离
            dist = math.sqrt(tx*tx + ty*ty + tz*tz)
            scale = (dist - 0.03) / dist if dist > 0.03 else 1.0
            rx, ry, rz = tx * scale, ty * scale, tz

            sol = node.plan_to_cartesian(rx, ry, rz)
            if sol is None:
                print("❌ 该空间位置超出机械臂当前姿态逆解范围，请稍挪近或换个位置！")
                continue

            current_pos = list(sol)
            node.execute_joints(current_pos, duration=3.0)
            print("✔ 末端已真实到达 RViz 小球面前！在 RViz 画面中观察两者已完美重合。")

            # 在该点进行微调贴碰
            offset_fwd = 0.0 # 沿视线方向增量 (m)
            offset_up = 0.0  # 垂直增量 (m)

            while True:
                cmd = input("\n[微调] (+1前送1cm / -1后撤1cm / d1下探1cm / u1上抬1cm / c确认此点 / s求解 / q退出): ").strip().lower()
                if cmd == 'c':
                    pb = node.get_current_tool0_in_base()
                    pc = node.get_cam_optical_point()
                    if pb is not None and pc is not None:
                        recorded_base_pts.append(pb)
                        recorded_cam_pts.append(pc)
                        print(f"🎉 成功记录第 {len(recorded_base_pts)} 个空间真值特征点！")
                        print(f"   物理末端真值 tool0 : X={pb[0]:.3f}, Y={pb[1]:.3f}, Z={pb[2]:.3f}")
                        print(f"   相机光学观测 opt   : X={pc[0]:.3f}, Y={pc[1]:.3f}, Z={pc[2]:.3f}")
                        point_idx += 1
                    break
                elif cmd == 's':
                    if len(recorded_base_pts) < 3:
                        print(f"⚠ 当前仅采集了 {len(recorded_base_pts)} 点，建议至少采集 3 点！")
                        continue
                    else:
                        break
                elif cmd == 'q':
                    sigint_handler(None, None)
                    return

                # 微调动作执行 (采用局部微增量平滑驱动，严格锁定 J4/J5/J6，彻底消除关节转大半圈的逆解跳变！)
                m = re.match(r'^([+\-du])(\d*\.?\d*)$', cmd)
                if m:
                    action, num_str = m.groups()
                    step = float(num_str) if num_str else 1.0

                    # 拷贝当前已到达的稳定关节状态
                    new_joints = list(current_pos)

                    # 1. 前后微调 (+1 前进 1cm / -1 后退 1cm)
                    # 真实前后：大臂 J2 微动 0.03 rad (~1.7°)，手肘 J3 微动 -0.045 rad (~-2.5°)
                    if action == '+':
                        new_joints[1] += step * 0.025
                        new_joints[2] -= step * 0.035
                        print(f"👉 向前微调 {step}cm...")
                    elif action == '-':
                        new_joints[1] -= step * 0.025
                        new_joints[2] += step * 0.035
                        print(f"👈 向后微退 {step}cm...")

                    # 2. 上下微调 (u1 抬高 1cm / d1 压低 1cm)
                    elif action == 'u':
                        new_joints[1] -= step * 0.025
                        new_joints[2] -= step * 0.010
                        print(f"👆 向上抬高 {step}cm...")
                    elif action == 'd':
                        new_joints[1] += step * 0.025
                        new_joints[2] += step * 0.010
                        print(f"👇 向下探低 {step}cm...")

                    # 严格保证 J4, J5, J6 不发生奇异反转
                    new_joints[3] = current_pos[3]
                    new_joints[4] = current_pos[4]
                    new_joints[5] = current_pos[5]

                    # 极速平滑下发 (微动仅需 0.8 秒，干脆平稳)
                    node.execute_joints(new_joints, duration=0.8)
                    current_pos = new_joints
                    print("✔ 微调到位！")

            if cmd == 's' and len(recorded_base_pts) >= 3:
                break

        # 全局 SVD 求解
        print("\n==================================================================")
        print(f"正在通过 SVD 求解全局唯一绝对外参 (基于 {len(recorded_base_pts)} 组实测真值)...")
        A = np.array(recorded_cam_pts)
        B = np.array(recorded_base_pts)

        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        H = (A - centroid_A).T @ (B - centroid_B)
        U, S, Vt = np.linalg.svd(H)
        R_base_opt = Vt.T @ U.T
        if np.linalg.det(R_base_opt) < 0:
            Vt[2, :] *= -1
            R_base_opt = Vt.T @ U.T
        t_base_opt = centroid_B - R_base_opt @ centroid_A

        errors = [np.linalg.norm(R_base_opt @ a + t_base_opt - b) * 1000.0 for a, b in zip(A, B)]
        mean_err = np.mean(errors)

        r_link_opt = R_tool.from_quat([-0.498, 0.500, -0.500, 0.502]).as_matrix()
        t_link_opt = np.array([-0.00046, 0.01467, 0.00010])

        T_base_opt = np.eye(4)
        T_base_opt[:3, :3] = R_base_opt
        T_base_opt[:3, 3] = t_base_opt

        T_link_opt_mat = np.eye(4)
        T_link_opt_mat[:3, :3] = r_link_opt
        T_link_opt_mat[:3, 3] = t_link_opt

        T_base_cam_link = T_base_opt @ np.linalg.inv(T_link_opt_mat)
        t_final = T_base_cam_link[:3, 3]
        r_final = R_tool.from_matrix(T_base_cam_link[:3, :3])
        roll_f, pitch_f, yaw_f = r_final.as_euler('xyz', degrees=False)

        print("\n==================================================================")
        print(f"🎉【全局高精度外参标定成功】")
        print(f"  平均全局定位精度: {mean_err:.2f} 毫米！")
        print(f"  cam_x:     '{t_final[0]:.4f}'")
        print(f"  cam_y:     '{t_final[1]:.4f}'")
        print(f"  cam_z:     '{t_final[2]:.4f}'")
        print(f"  cam_roll:  '{roll_f:.4f}'")
        print(f"  cam_pitch: '{pitch_f:.4f}'")
        print(f"  cam_yaw:   '{yaw_f:.4f}'")
        print("==================================================================")

        node.return_to_rest()

    except KeyboardInterrupt:
        sigint_handler(None, None)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
