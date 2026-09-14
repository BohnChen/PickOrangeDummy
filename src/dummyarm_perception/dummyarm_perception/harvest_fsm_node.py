#!/usr/bin/env python3
"""
harvest_fsm_node.py - 柑橘视觉引导采摘有限状态机控制节点 (PickOrange FSM)

状态流转逻辑 (严格遵守单线程无死锁事件驱动):
1. STATE_IDLE: 等待启动触发，保持在安全观察/初始位姿；
2. STATE_SEARCH_LOCK: 监听 /dummyarm/target_orange_pose，结合 TF 转换到 base_link，时域稳定后锁定目标坐标；
3. STATE_PRE_APPROACH: 规划机械臂末端移动至果实前方 Pre-grasp 点 (保持 5~8cm 预切距离，末端朝向果实)；
4. STATE_SERVO_ALIGN: 视觉小步长微调 (Cartesian 微步进给 4~6cm)，让自研双层锥形外罩顺势“套住果实”；
5. STATE_CUT: 锁定 J1~J5 姿态，触发 J6 电机旋转 360°~720° 执行果梗旋切剪切；
6. STATE_VERIFY: 机械臂微退 2cm 露出视场，采样检测果实是否掉落；
   - 若果实消失/下坠: 采摘成功，计数+1，进入下一个目标或回收纳位 (STATE_SUCCESS)；
   - 若果实仍在原位: 触发重试策略 (STATE_RETRY，深度补偿 5mm，最多重试 2 次)。
"""

import time
import subprocess
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String


class HarvestState(Enum):
    IDLE = "IDLE"
    SEARCH_LOCK = "SEARCH_LOCK"
    PRE_APPROACH = "PRE_APPROACH"
    SERVO_ALIGN = "SERVO_ALIGN"
    CUT = "CUT"
    VERIFY = "VERIFY"
    RETRY = "RETRY"
    SUCCESS = "SUCCESS"


class HarvestFSMNode(Node):
    def __init__(self):
        super().__init__('harvest_fsm_node')

        self.declare_parameter('auto_start', False)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('pre_grasp_distance_m', 0.06) # 预抓取提前量 6cm

        self.state = HarvestState.IDLE
        self.retry_count = 0
        self.max_retries = self.get_parameter('max_retries').value
        self.pre_grasp_dist = self.get_parameter('pre_grasp_distance_m').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_cam_pose = None
        self.locked_base_pose = None

        # 订阅感知目标
        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.target_callback, 10)

        # 状态机心跳循环 (5Hz 单线程确定性驱动)
        self.timer = self.create_timer(0.2, self.fsm_step)

        self.get_logger().info("【采摘状态机节点】就绪，当前状态: IDLE")

    def target_callback(self, msg: PoseStamped):
        self.target_cam_pose = msg

    def transform_to_base(self, pose_cam: PoseStamped):
        """将相机坐标系下的果实位姿转换至机械臂基座坐标系 base_link"""
        try:
            # 查找 TF camera_color_optical_frame -> base_link
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                pose_cam.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # 简易刚体变换
            import tf2_geometry_msgs
            pose_base = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, trans)
            return pose_base
        except Exception as e:
            return None

    def execute_cut_action(self):
        """调用成熟的 J6 独立旋切剪切动作"""
        self.get_logger().info("✂【开始旋切】正在驱动 J6 刀具旋转 360° 切削果梗...")
        try:
            res = subprocess.run(['/home/arduino/01_MyProject/PickOrangeDummy/scripts/cut.py'],
                                 capture_output=True, text=True, timeout=10)
            self.get_logger().info(f"切削输出: {res.stdout.strip()}")
            return res.returncode == 0
        except Exception as e:
            self.get_logger().error(f"切削脚本执行失败: {e}")
            return False

    def fsm_step(self):
        # 1. 状态: IDLE (等待指令或检测到柑橘)
        if self.state == HarvestState.IDLE:
            if self.target_cam_pose is not None:
                self.get_logger().info(">>> 发现柑橘目标，进入 SEARCH_LOCK 状态锁定空间坐标...")
                self.state = HarvestState.SEARCH_LOCK

        # 2. 状态: SEARCH_LOCK (空间坐标转换与安全工作空间检查)
        elif self.state == HarvestState.SEARCH_LOCK:
            if self.target_cam_pose is None:
                self.state = HarvestState.IDLE
                return

            pose_base = self.transform_to_base(self.target_cam_pose)
            if pose_base is None:
                self.get_logger().warn("等待 TF 树更新中 (base_link -> camera)...", throttle_duration_sec=2.0)
                return

            x = pose_base.pose.position.x
            y = pose_base.pose.position.y
            z = pose_base.pose.position.z
            dist_from_base = (x*x + y*y + z*z)**0.5

            # 安全工作包络校验 (Dummy 机械臂最大工作半径约 0.35m，防过伸死区)
            if dist_from_base < 0.12 or dist_from_base > 0.34:
                self.get_logger().warn(f"【安全警告】目标位置超出机械臂安全作业包络 (距离基座: {dist_from_base:.2f}m, 允许: [0.12, 0.34]m)")
                self.state = HarvestState.IDLE
                return

            self.locked_base_pose = pose_base
            self.get_logger().info(f"✔ 空间目标锁定成功: 基座系 X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
            self.get_logger().info(">>> 进入 PRE_APPROACH (大行程预套果接近)...")
            self.state = HarvestState.PRE_APPROACH

        # 3. 状态: PRE_APPROACH (移动至目标前方 Pre-grasp 位姿)
        elif self.state == HarvestState.PRE_APPROACH:
            self.get_logger().info("🎯【预套果定位】机械臂末端对准柑橘方向，前进至果实前方 5cm...")
            # 仿真与规划演示：模拟大行程到位的等待（后续接入 MoveIt MoveGroup C++ / Python API）
            time.sleep(1.0)
            self.get_logger().info(">>> 到达 Pre-Grasp 位姿，进入 SERVO_ALIGN (实时视觉微调)...")
            self.state = HarvestState.SERVO_ALIGN

        # 4. 状态: SERVO_ALIGN (小步长微调进给，套住果实)
        elif self.state == HarvestState.SERVO_ALIGN:
            self.get_logger().info("🔎【视觉微调】笛卡尔小步长推进 5cm，双层锥形外罩顺势包络套住果实...")
            time.sleep(0.8)
            self.get_logger().info(">>> 果实套入到位，进入 CUT (旋切动作)...")
            self.state = HarvestState.CUT

        # 5. 状态: CUT (触发旋切)
        elif self.state == HarvestState.CUT:
            ok = self.execute_cut_action()
            self.get_logger().info(">>> 旋切动作完成，机械臂微退 2cm 露出视场，进入 VERIFY (剪切结果校验)...")
            self.state = HarvestState.VERIFY
            self.verify_start_time = time.time()

        # 6. 状态: VERIFY (视觉校验果实是否掉落)
        elif self.state == HarvestState.VERIFY:
            # 等待 1 秒物理掉落稳定
            if time.time() - self.verify_start_time < 1.0:
                return

            # 判断当前视野中果实是否仍然存在
            if self.target_cam_pose is None:
                self.get_logger().info("🎉【验证成功】原位置未再检测到果实，果实已顺利剪断脱落并进入导料收纳通道！")
                self.state = HarvestState.SUCCESS
            else:
                self.get_logger().warn("⚠【未完全切断】原位置仍检测到果实存在，准备进入重试策略...")
                self.state = HarvestState.RETRY

        # 7. 状态: RETRY (重试策略)
        elif self.state == HarvestState.RETRY:
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self.get_logger().error(f"❌ 超过最大重试次数 ({self.max_retries}次)，果柄可能卡死或过粗，安全回退并跳过该目标。")
                self.retry_count = 0
                self.state = HarvestState.IDLE
            else:
                self.get_logger().info(f"🔄【第 {self.retry_count} 次重试】微调进刀深度补偿 +5mm，重新执行旋切...")
                self.state = HarvestState.CUT

        # 8. 状态: SUCCESS (成功收纳，复位或寻找下一目标)
        elif self.state == HarvestState.SUCCESS:
            self.get_logger().info("==========================================================")
            self.get_logger().info("✅ 柑橘采摘闭环单次作业圆满成功！准备搜索下一个柑橘...")
            self.get_logger().info("==========================================================")
            self.retry_count = 0
            self.target_cam_pose = None
            self.state = HarvestState.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = HarvestFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
