#!/usr/bin/env python3
"""
multi_point_touch_eval.py - 多点空间网格交互标定与定位误差校准工具

【核心原则】:
1. 绝对安全红线: 严格限制在物理安全作业空间内，硬件限位与安全姿态零修改零妥协；
2. 修正目标纯粹: 仅优化相机与机械臂基座之间的“空间定位外参 (R, T)”，不掺杂机械臂关节模型篡改；
3. 多点网格支撑: 通过在工作空间不同位置 (如近处、远处、左侧、右侧) 采集 3~4 个真实物理点，
   通过 SVD / 刚体最小二乘法，彻底解耦旋转倾角与平移偏置，实现全空间全局毫米级精准！
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
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R_tool


class MultiPointCalibrator(Node):
    def __init__(self):
        super().__init__('multi_point_calibrator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.traj_client.wait_for_server(timeout_sec=5.0)

    def orange_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_orange_cam_pose = msg

    def get_cam_optical_point(self):
        """获取相机光学坐标系下的原始测得目标点 (Xc, Yc, Zc)"""
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

    def move_to_joints(self, pos, duration=2.5):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in pos]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points.append(pt)

        send_future = self.traj_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.01)
        handle = send_future.result()
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

    def get_current_tool0_in_base(self):
        """读取当前机械臂在经过人工微调贴合后，末端 tool0 在 base_link 下的绝对物理坐标真值"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            t = trans.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None


def solve_arm_kinematics(r, z):
    """
    真机严密 2 连杆闭式几何逆解 (水平 r 与 垂直 z 严格正交解耦):
    大臂有效长 L1 = 0.146m, 小臂有效长 L2 = 0.165m
    """
    L1 = 0.146
    L2 = 0.165

    # 目标距肩部有效距离 D
    h = z - 0.04 # 扣除肩部安装高差
    D = math.sqrt(r * r + h * h)
    D = min(max(D, 0.14), L1 + L2 - 0.015)

    # 余弦定理求肘部夹角
    cos_elbow = (D * D - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    theta_elbow = math.acos(cos_elbow)

    # 肩部俯仰角
    alpha = math.atan2(h, r)
    beta = math.atan2(L2 * math.sin(theta_elbow), L1 + L2 * math.cos(theta_elbow))
    theta_shoulder = alpha + beta

    # 映射回 CAD 模型角:
    # 严格匹配物理方向：前伸时小臂向外展开，上抬时大臂立起
    j2_cad = 1.274 + (1.15 - theta_shoulder) * 0.95
    j3_cad = -1.571 + (theta_elbow - 1.571) * 0.95

    # 严格物理安全限位
    j2_cad = max(0.40, min(1.85, j2_cad))
    j3_cad = max(-2.15, min(-0.65, j3_cad))

    return j2_cad, j3_cad


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
    node = MultiPointCalibrator()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    signal.signal(signal.SIGINT, sigint_handler)

    print("==================================================================")
    print("    PickOrangeDummy 多点空间网格交互标定工具 (Multi-Point Calib) ")
    print("==================================================================")
    print("【测试规则】:")
    print("1. 建议在工作空间采集 3~4 个不同位置 (如近处、远处、左侧、右侧)；")
    print("2. 机械臂前探到位后，根据肉眼观察微调至【末端完美对准并贴碰目标】；")
    print("3. 按 'c' 确认记录该特征点 (Confirm)；")
    print("4. 换下一个位置重复，采集完 3~4 点后按 's' 自动全局求解最优外参；")
    print("5. 随时按 'q' 退出并自动平稳归位至 REST。")
    print("==================================================================")

    recorded_base_pts = []  # 机械臂物理末端真实到达的坐标 (base_link)
    recorded_cam_pts = []   # 对应时刻相机测得的光学坐标 (camera_color_optical_frame)

    point_idx = 1

    try:
        while rclpy.ok():
            print(f"\n👉 [请摆放第 {point_idx} 个测试目标] (建议变换前后距离或左右方位)")
            input("   摆好目标后，按 [Enter] 让机械臂前探到达观察位: ")

            # 等待视觉稳定
            target = None
            for _ in range(20):
                target = node.get_target_in_base()
                if target is not None:
                    break
                time.sleep(0.1)

            if target is None:
                print("❌ 未能稳定检测到目标，请确认目标在相机画面内且有绿框！")
                continue

            tx = target.pose.position.x
            ty = target.pose.position.y
            tz = target.pose.position.z
            r_base = math.sqrt(tx * tx + ty * ty)

            if r_base < 0.12 or r_base > 0.30:
                print(f"❌ 目标水平距离 {r_base:.2f}m 超出安全作业半径 [0.12, 0.30]m，请重新挪位！")
                continue

            # 本点的微调累计量
            offset_r = 0.0   # 前后距离增量 (m)
            offset_lat = 0.0 # 左右横向偏角增量 (rad)
            offset_z = 0.0   # 垂直高度增量 (m)

            while True:
                # 实时计算当前关节位姿
                j1 = math.atan2(tx, -ty) + offset_lat
                r_now = r_base + offset_r
                z_now = tz + offset_z

                j2, j3 = solve_arm_kinematics(r_now, z_now)
                current_joints = [j1, j2, j3, 0.0, 0.0, 0.0]

                print(f"\n正在驱动末端到位 (当前微调累计: 前后 {offset_r*100:+.1f}cm, 左右 {offset_lat*57.3:+.1f}°, 高度 {offset_z*100:+.1f}cm)...")
                node.move_to_joints(current_joints, duration=2.0)
                print("✔ 已到位！请直接肉眼观察机械臂末端与物体的接触情况。")

                cmd = input("[微调指令: +1前 / -1后 / l1左 / r1右 / d1下 / u1上 / c确认并记入此点 / s完成求解]: ").strip().lower()

                if cmd == 'c':
                    # 记录此特征点的物理末端真实坐标 与 对应的相机观测坐标
                    pb = node.get_current_tool0_in_base()
                    pc = node.get_cam_optical_point()
                    if pb is not None and pc is not None:
                        recorded_base_pts.append(pb)
                        recorded_cam_pts.append(pc)
                        print(f"🎉 成功记录第 {len(recorded_base_pts)} 个空间特征点！")
                        print(f"   物理真值 tool0 (base): X={pb[0]:.3f}, Y={pb[1]:.3f}, Z={pb[2]:.3f}")
                        print(f"   相机观测   opt  (cam):  X={pc[0]:.3f}, Y={pc[1]:.3f}, Z={pc[2]:.3f}")
                        point_idx += 1
                    else:
                        print("❌ 记录失败，未同时获取到 TF 与视觉坐标！")
                    break

                elif cmd == 's':
                    if len(recorded_base_pts) < 3:
                        print(f"⚠ 当前仅采集了 {len(recorded_base_pts)} 个点，建议至少采集 3 个点以保证全空间无欠定！")
                        continue
                    else:
                        break

                elif cmd == 'q':
                    sigint_handler(None, None)
                    return

                # 正则鲁棒解析微调指令
                m = re.match(r'^([+\-ldur])(\d*\.?\d*)$', cmd)
                if m:
                    action, num_str = m.groups()
                    step = float(num_str) if num_str else 1.0
                    if action == '+': offset_r += step * 0.01
                    elif action == '-': offset_r -= step * 0.01
                    elif action == 'd': offset_z -= step * 0.01
                    elif action == 'u': offset_z += step * 0.01
                    elif action == 'l': offset_lat += step * (math.pi / 180.0) # 向左偏转指定度数
                    elif action == 'r': offset_lat -= step * (math.pi / 180.0) # 向右偏转指定度数
                else:
                    if 'd' in cmd: offset_z -= 0.01
                    elif 'u' in cmd: offset_z += 0.01
                    elif '+' in cmd: offset_r += 0.01
                    elif '-' in cmd: offset_r -= 0.01
                    elif 'l' in cmd: offset_lat += 0.017
                    elif 'r' in cmd: offset_lat -= 0.017

            if cmd == 's' and len(recorded_base_pts) >= 3:
                break

        # -------------------------------------------------------------
        # 全局 SVD 闭式解析与定位误差严密校准
        # -------------------------------------------------------------
        print("\n==================================================================")
        print(f"正在进行多点空间正交最优拟合解算 (总共 {len(recorded_base_pts)} 组实测真值点)...")
        A = np.array(recorded_cam_pts)  # 相机光学测量点
        B = np.array(recorded_base_pts) # 机械臂真实触碰物理点

        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        H = (A - centroid_A).T @ (B - centroid_B)
        U, S, Vt = np.linalg.svd(H)
        R_base_opt = Vt.T @ U.T
        if np.linalg.det(R_base_opt) < 0:
            Vt[2, :] *= -1
            R_base_opt = Vt.T @ U.T
        t_base_opt = centroid_B - R_base_opt @ centroid_A

        # 计算全局拟合残差
        errors = [np.linalg.norm(R_base_opt @ a + t_base_opt - b) * 1000.0 for a, b in zip(A, B)]
        mean_err = np.mean(errors)

        # 反解得到标准 camera_link 外参
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
        print(f"  各采样点最大残差: {max(errors):.2f} mm | 平均全局定位精度: {mean_err:.2f} 毫米！")
        print("------------------------------------------------------------------")
        print("  建议填入 launch 的最优外参参数:")
        print(f"  cam_x:     '{t_final[0]:.4f}'")
        print(f"  cam_y:     '{t_final[1]:.4f}'")
        print(f"  cam_z:     '{t_final[2]:.4f}'")
        print(f"  cam_roll:  '{roll_f:.4f}'")
        print(f"  cam_pitch: '{pitch_f:.4f}'")
        print(f"  cam_yaw:   '{yaw_f:.4f}'")
        print("==================================================================")

        # 平稳收回
        node.return_to_rest()

    except KeyboardInterrupt:
        sigint_handler(None, None)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
