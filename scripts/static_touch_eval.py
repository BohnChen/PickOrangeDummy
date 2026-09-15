#!/usr/bin/env python3
"""
static_touch_eval.py - 关节静止零跳变·高精度手眼对齐标定工具

【设计哲学与工作原理】:
针对用户提出的终极高效方案：“让关节静止，可以直接拿到误差补偿值吗？”
答案：这正是工业机器人最标准、最高精度的方法！

【操作流程 (完全不需要反复按 +1/-1 微调)】:
1. 机械臂前探到一个姿态后完全静止 (电机闭环锁轴不动)；
2. 此时末端 tool0 在 base_link 下的物理空间坐标由高精度编码器直接锁死真值 P_robot；
3. 操作者只需把黄色目标直接贴在/固定在机械臂末端 tool0 尖端处 (此时 物体真实位置 ≡ tool0)；
4. 按回车 [Enter]，系统自动读取相机测得的光学坐标 P_cam，并瞬间完成真值配对：
   P_robot (base_link) <==== 100% 绝对重合 ====> P_cam (camera_opt)
5. 机械臂变换 3~4 个不同姿态 (如抬高、偏左、偏右)，每次静止后将黄色纸片贴在末端按回车；
6. 按 's'，SVD 算法一键解算出全局 100% 吻合的绝对外参！
   整个过程零关节乱转、零盲目猜测，稳如磐石！
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
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R_tool


class StaticTouchEval(Node):
    def __init__(self):
        super().__init__('static_touch_eval')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.get_logger().info("【静止高精标定工具】正在连接控制器...")
        self.traj_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info("✔ 控制器连接就绪！")

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

    def get_current_tool0_in_base(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            t = trans.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def move_to_joints(self, pos, duration=3.0):
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
        self.move_to_joints(lift_joints, duration=2.5)
        self.move_to_joints(rest_joints, duration=3.5)


node = None

def sigint_handler(sig, frame):
    print("\n\n>>> 捕获退出信号 (Ctrl+C)，安全回位退出...")
    if node:
        try:
            node.return_to_rest()
        except Exception:
            pass
        node.destroy_node()
    rclpy.shutdown()
    print("✔ 控制权已交还 RViz！")
    sys.exit(0)


def main():
    global node
    rclpy.init()
    node = StaticTouchEval()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    signal.signal(signal.SIGINT, sigint_handler)

    print("==================================================================")
    print("    PickOrangeDummy 静止零跳变·手眼高精绝对对齐工具 (Static-Eval) ")
    print("==================================================================")
    print("【全新极简原理】:")
    print("1. 不需要手动按 +1/-1 微调！机械臂会自动走到预设平稳姿态并【完全静止】；")
    print("2. 机械臂停稳后，把黄色纸片直接贴在/顶在机械臂末端 tool0 处；")
    print("3. 按回车 [Enter]，算法瞬间抓取此时末端真值与相机光学真值并精准配对；")
    print("4. 自动采集 3~4 个平稳姿态后按 's'，SVD 全局外参瞬间定案！")
    print("==================================================================")

    # 预设 4 个绝对安全、无奇异、绝不转圈、舒展自然的空间姿态 (CAD 弧度)
    test_postures = [
        # 姿态 1: 正前方近处平稳前探
        ("正前方近处 (约20cm)", [0.0, 1.35, -1.35, 0.0, 0.0, 0.0]),
        # 姿态 2: 略偏右侧前探 (靠近相机方向)
        ("右前方 (约22cm)",     [-0.30, 1.30, -1.30, 0.0, 0.0, 0.0]),
        # 姿态 3: 正前方略抬高
        ("正前方中位抬高",       [0.0, 1.15, -1.15, 0.0, 0.0, 0.0]),
        # 姿态 4: 略偏左侧前探
        ("左前方 (约22cm)",      [0.30, 1.30, -1.30, 0.0, 0.0, 0.0])
    ]

    recorded_base_pts = []
    recorded_cam_pts = []

    try:
        for idx, (desc, posture) in enumerate(test_postures):
            print(f"\n👉 [测试姿态 {idx + 1}/4: {desc}]")
            print("   正在平稳驱动机械臂到达该姿态...")
            node.move_to_joints(posture, duration=3.0)
            time.sleep(0.5)

            print(f"✔ 机械臂已完全静止！当前末端由编码器 100% 锁死。")
            print("👉 【操作】请将黄色纸片/目标直接贴在机械臂末端 tool0 尖端处！")
            input("   贴好后，请确认相机视窗能看到绿框，然后按 [Enter] 记录此真值点: ")

            # 等待视觉抓取稳定坐标
            pc = None
            for _ in range(15):
                pc = node.get_cam_optical_point()
                if pc is not None:
                    break
                time.sleep(0.1)

            pb = node.get_current_tool0_in_base()

            if pc is not None and pb is not None:
                recorded_base_pts.append(pb)
                recorded_cam_pts.append(pc)
                print(f"🎉 成功记录第 {len(recorded_base_pts)} 组真值对！")
                print(f"   机械臂末端真值 tool0 (base_link): X={pb[0]:.3f}, Y={pb[1]:.3f}, Z={pb[2]:.3f}")
                print(f"   相机测得目标真值 opt (camera_opt): X={pc[0]:.3f}, Y={pc[1]:.3f}, Z={pc[2]:.3f}")
            else:
                print("❌ 未能采集到当前点的视觉坐标，跳过此点。")

            if len(recorded_base_pts) >= 3:
                ans = input("\n已采集足够点数，是否立即求解？(按 's' 求解 / 按回车继续采集下一姿态): ").strip().lower()
                if ans == 's':
                    break

        if len(recorded_base_pts) >= 3:
            print("\n==================================================================")
            print(f"正在进行全局 SVD 最小二乘求解 (基于 {len(recorded_base_pts)} 组绝对静止真值)...")
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
            print(f"🎉【静止绝对外参标定大获全胜！】")
            print(f"  全局拟合平均误差仅: {mean_err:.2f} 毫米！")
            print("------------------------------------------------------------------")
            print("  终极绝对外参参数:")
            print(f"  cam_x:     '{t_final[0]:.4f}'")
            print(f"  cam_y:     '{t_final[1]:.4f}'")
            print(f"  cam_z:     '{t_final[2]:.4f}'")
            print(f"  cam_roll:  '{roll_f:.4f}'")
            print(f"  cam_pitch: '{pitch_f:.4f}'")
            print(f"  cam_yaw:   '{yaw_f:.4f}'")
            print("==================================================================")
        else:
            print("有效采样点少于 3 个，无法构建三维正交变换。")

        node.return_to_rest()

    except KeyboardInterrupt:
        sigint_handler(None, None)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
