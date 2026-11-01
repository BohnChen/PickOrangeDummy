#!/usr/bin/env python3
"""
arm_action_server.py - 机械臂 ROS 2 Action 服务端

托管 `/dev/ttyACM1` (REF 主控)，对外提供:
1. MoveToPose.action (/arm/move_to_pose)
   - 目标点逆解、防碰撞与扫掠检查、平滑运动
   - 内部集成"靠近直接进刀/不可行放弃"策略
2. CutStem.action (/arm/cut_stem)
   - 原子级 J6 剪切：全开 -> 0..62° 合刀 -> 保压 -> 张刀回零 -> 状态校验
"""

import os
import sys
import time
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Vector3
from pickorange_interfaces.action import MoveToPose, CutStem

# 引入底层经过严格安全验证的算子
SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../scripts'))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

try:
    import arm_kin
    import arm_pick
    import arm_safety
    from zero_j6 import find_ref_port
except ImportError:
    # 尝试备选路径
    SCRIPTS_DIR = '/home/arduino/01_MyProject/PickOrangeDummy/scripts'
    if SCRIPTS_DIR not in sys.path:
        sys.path.insert(0, SCRIPTS_DIR)
    import arm_kin
    import arm_pick
    import arm_safety
    from zero_j6 import find_ref_port


class ArmActionServer(Node):
    def __init__(self):
        super().__init__('pickorange_arm')
        self.get_logger().info('正在启动 PickOrange 机械臂 Action 服务节点...')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # 1. 初始化并托管串口
        self.port = find_ref_port()
        self.get_logger().info(f'连接 REF 主控串口: {self.port}')
        
        # 初始化串口连接
        try:
            from calib_eye_to_hand import agx
            self.ser = agx.RefSerial(self.port)
            self.ser.cmd("!START", 0.8)
            joints = self.ser.joints()
            if joints is not None:
                self.get_logger().info(f'机械臂初始姿态: {[round(x, 2) for x in joints]}')
            else:
                self.get_logger().warn('未能成功读取到机械臂关节角度！')
        except Exception as e:
            self.get_logger().error(f'打开 REF 串口失败: {e}')
            self.ser = None

        # 2. 读取 J6 零位配置
        self.blade_cfg_path = os.path.join(
            os.path.dirname(SCRIPTS_DIR), 'config', 'j6_blade.json'
        )
        self.load_j6_config()

        # 3. 注册 Action 服务
        self._move_action_server = ActionServer(
            self,
            MoveToPose,
            '/arm/move_to_pose',
            execute_callback=self.execute_move_callback,
            goal_callback=self.goal_move_callback,
            cancel_callback=self.cancel_move_callback,
            callback_group=self.cb_group
        )

        self._cut_action_server = ActionServer(
            self,
            CutStem,
            '/arm/cut_stem',
            execute_callback=self.execute_cut_callback,
            goal_callback=self.goal_cut_callback,
            cancel_callback=self.cancel_cut_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('PickOrange 机械臂 Action 服务已就绪: /arm/move_to_pose, /arm/cut_stem')

    def load_j6_config(self):
        try:
            with open(self.blade_cfg_path, 'r') as f:
                cfg = json.load(f)
                self.j6_home_deg = float(cfg.get('j6_home_deg', -1.06))
                self.cut_span_deg = float(cfg.get('cut_span_deg', 60.0))
                self.get_logger().info(f'已载入 J6 零点: home={self.j6_home_deg}°, span={self.cut_span_deg}°')
        except Exception as e:
            self.j6_home_deg = -1.06
            self.cut_span_deg = 60.0
            self.get_logger().warn(f'读取 j6_blade.json 失败({e})，使用缺省值: home=-1.06, span=60.0')

    # ==================== MoveToPose Action ====================
    def goal_move_callback(self, goal_request):
        self.get_logger().info('收到 MoveToPose 目标请求')
        if self.ser is None:
            self.get_logger().error('串口未就绪，拒绝目标')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_move_callback(self, goal_handle):
        self.get_logger().info('收到 MoveToPose 取消请求')
        return CancelResponse.ACCEPT

    async def execute_move_callback(self, goal_handle):
        self.get_logger().info('开始执行 MoveToPose 规划与动作...')
        result = MoveToPose.Result()
        
        target_p = goal_handle.request.target.pose.position
        target_xyz = np.array([target_p.x, target_p.y, target_p.z], dtype=float)
        
        cur_js = self.ser.joints() if self.ser else None
        if cur_js is None:
            result.success = False
            result.error_code = "HARDWARE"
            result.message = "无法获取当前关节姿态"
            goal_handle.abort()
            return result

        cur_model = arm_kin.model_from_fw(cur_js)
        
        # 规划采摘路径决策
        plan = arm_pick.plan_pick(target_xyz, current_joints_model=cur_model)
        self.get_logger().info(f'采摘决策状态: {plan["status"]}, 原因: {plan["reason"]}')
        
        if plan["status"] == "ABORT":
            result.success = False
            result.error_code = "UNREACHABLE"
            result.message = plan["reason"]
            goal_handle.abort()
            return result
        
        # 执行动作序列
        if plan["status"] == "STAGE_AND_FEED":
            # 1. 移动到预备位
            stage_fw = plan["stage_pose"][1]
            js = arm_pick.move_to(self.ser, stage_fw, speed=20.0)
            if js is None:
                result.success = False
                result.error_code = "COLLISION"
                result.message = "移动到预备位被扫掠安全拦截或执行失败"
                goal_handle.abort()
                return result

        # 2. 进刀套果
        fruit = {"p_base": target_xyz}
        cur_tcp = arm_kin.fk_full(cur_model)[:3, 3] if plan["status"] == "DIRECT_FEED" else plan["stage_target"]
        fed = arm_pick.do_feed(self.ser, fruit, cur_tcp, dry=False)
        
        if not fed:
            result.success = False
            result.error_code = "UNREACHABLE"
            result.message = "进刀套果未成功到位"
            goal_handle.abort()
            return result

        result.success = True
        result.error_code = "SUCCESS"
        result.message = "套果动作完成并已到位保持"
        goal_handle.succeed()
        return result

    # ==================== CutStem Action ====================
    def goal_cut_callback(self, goal_request):
        self.get_logger().info(f'收到 CutStem 目标请求 (span={goal_request.span_deg}°)')
        if self.ser is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cut_callback(self, goal_handle):
        self.get_logger().warn('CutStem 剪切属于原子操作，不建议中途取消')
        return CancelResponse.ACCEPT

    async def execute_cut_callback(self, goal_handle):
        self.get_logger().info('开始执行 CutStem 剪切流程...')
        result = CutStem.Result()
        
        span = goal_handle.request.span_deg
        if span <= 0 or span > 62.0:
            span = self.cut_span_deg
            
        success = arm_pick.do_cut(self.ser, dry=False)
        
        js = self.ser.joints() if self.ser else None
        final_j6 = js[5] if js else 0.0
        
        if success:
            result.success = True
            result.error_code = "SUCCESS"
            result.final_angle_deg = float(final_j6)
            result.message = f"剪切完成，当前 J6={final_j6:.2f}°"
            goal_handle.succeed()
        else:
            result.success = False
            result.error_code = "HARDWARE"
            result.final_angle_deg = float(final_j6)
            result.message = "剪切操作执行失败"
            goal_handle.abort()
            
        return result


def main(args=None):
    rclpy.init(args=args)
    server = ArmActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if server.ser:
            server.ser.close()
        server.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
