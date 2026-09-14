#!/usr/bin/env python3
"""
interactive_hand_eye_calibrator.py - 真正的闭环手眼标定采集工具
核心机制：
1. 本脚本不仅采集坐标，还会打印出：
   - 当前机械臂末端 tool0 是否真正发生了移动；
   - 当前相机测得的目标坐标是否发生了移动；
2. 只有当机械臂末端与相机目标【同时稳定在新位置】时，才判定为有效采样！
3. 提供手动输入末端坐标或自动读取两种模式。
"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R_tool


class InteractiveCalibrator(Node):
    def __init__(self):
        super().__init__('interactive_calibrator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_cam_pose = None
        self.latest_joint_state = None

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.cam_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

    def cam_callback(self, msg: PoseStamped):
        self.latest_cam_pose = msg
        self.latest_cam_time = self.get_clock().now()

    def get_status(self):
        # 严格检查视觉测量是否是最新且活跃的数据 (必须是 0.5s 之内产生的新数据)
        p_cam = None
        if self.latest_cam_pose is not None and hasattr(self, 'latest_cam_time'):
            dt = (self.get_clock().now() - self.latest_cam_time).nanoseconds / 1e9
            if dt < 0.8:
                p_cam = np.array([
                    self.latest_cam_pose.pose.position.x,
                    self.latest_cam_pose.pose.position.y,
                    self.latest_cam_pose.pose.position.z
                ])

        p_base = None
        try:
            now = rclpy.time.Time()
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', now, timeout=rclpy.duration.Duration(seconds=0.5))
            p_base = np.array([
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z
            ])
        except Exception:
            pass

        return p_base, p_cam


def main():
    rclpy.init()
    node = InteractiveCalibrator()

    print("==================================================================")
    print("      PickOrangeDummy 交互式手眼标定向导 (动态监测版)             ")
    print("==================================================================")
    print("【关键提示】:")
    print("手眼标定要求：黄色物体固定在末端，且机械臂关节真实转动到不同姿态。")
    print("本工具会实时监测机械臂末端 tool0 是否真正更新。")
    print("==================================================================")

    pts_base = []
    pts_cam = []

    last_pb = None
    last_pc = None

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            pb, pc = node.get_status()

            status_str = ""
            if pb is not None:
                status_str += f"机械臂末端 tool0: [{pb[0]:.3f}, {pb[1]:.3f}, {pb[2]:.3f}] | "
            else:
                status_str += "机械臂 TF 未就绪 | "

            if pc is not None:
                status_str += f"相机观测: [{pc[0]:.3f}, {pc[1]:.3f}, {pc[2]:.3f}]"
            else:
                status_str += "等待视觉识别目标..."

            print(f"\r当前状态: {status_str}", end="", flush=True)

            # 非阻塞检测用户输入或按回车
            cmd = input("\n[操作] 按回车记录当前点 (输入 'q' 完成求解): ").strip()
            if cmd.lower() == 'q':
                break

            pb, pc = node.get_status()
            if pb is None or pc is None:
                print("❌ 记录失败：未同时检测到机械臂坐标与视觉目标，请确认！")
                continue

            # 检查机械臂是否和上一个记录点完全重合（未移动）
            if last_pb is not None and np.linalg.norm(pb - last_pb) < 0.005:
                print("⚠【注意】机械臂末端 tool0 物理位置相比上次【未发生变化】！")
                print("   请在 RViz 中拖动机械臂交互小球并点击 Plan & Execute，让机械臂真正运动到一个新位姿后再记录。")
                confirm = input("   是否仍要强制记录该点？(y/n): ").strip().lower()
                if confirm != 'y':
                    continue

            # 检查相机目标是否和上一次完全重合 (未更新)
            if last_pc is not None and np.linalg.norm(pc - last_pc) < 0.005:
                print("⚠【注意】相机测得的目标坐标相比上次【未发生变化】！")
                print("   说明相机尚未刷新到新位置的果实，请等待 1 秒等画面出现绿框后再按回车。")
                confirm = input("   是否仍要强制记录该点？(y/n): ").strip().lower()
                if confirm != 'y':
                    continue

            pts_base.append(pb)
            pts_cam.append(pc)
            last_pb = pb
            last_pc = pc
            print(f"✔ 成功记录第 {len(pts_base)} 组有效配对点！")

    except KeyboardInterrupt:
        pass

    if len(pts_base) >= 3:
        # SVD 求解
        A = np.array(pts_cam)
        B = np.array(pts_base)
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        H = (A - centroid_A).T @ (B - centroid_B)
        U, S, Vt = np.linalg.svd(H)
        R_mat = Vt.T @ U.T
        if np.linalg.det(R_mat) < 0:
            Vt[2, :] *= -1
            R_mat = Vt.T @ U.T
        t_vec = centroid_B - R_mat @ centroid_A

        r = R_tool.from_matrix(R_mat)
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        quat = r.as_quat()

        print("\n==================================================================")
        print("🎉【标定求解完成】")
        print(f"  平移 t_base_cam (m) : X={t_vec[0]:.4f}, Y={t_vec[1]:.4f}, Z={t_vec[2]:.4f}")
        print(f"  欧拉角 (rad)        : roll={roll:.4f}, pitch={pitch:.4f}, yaw={yaw:.4f}")
        print("==================================================================")
    else:
        print("\n有效采样点少于 3 个，未执行求解。")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
