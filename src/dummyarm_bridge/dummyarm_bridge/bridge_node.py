"""dummyarm_bridge: 真机桥接节点(阶段 3a)。

职责:
  1. 周期读取 REF 关节角并发布 /joint_states (rad), 供 robot_state_publisher/RViz 同步
  2. 提供 std_srvs/Trigger 管理服务: start / disable / stop / home / reset
  3. 进程启动绝不自动使能(安全); 串口由本节点独占, 禁止与 dummyarm CLI 同开

安全依赖: dummyarm_link 内置 auk 限位与协议语义; 意外时 !STOP(estop) 立即锁位失能。
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import sys
try:
    from dummyarm_link import DummyArmLink, DummyArmLinkError, JOINT_LIMITS
except ImportError:
    sys.path.insert(0, '/home/arduino/dummyarm_tools')
    from dummyarm_link import DummyArmLink, DummyArmLinkError, JOINT_LIMITS

JOINT_NAMES = [f'Joint{i}' for i in range(1, 7)]
DEG2RAD = 3.141592653589793 / 180.0

# FW→模型 角度映射 (Kabsch 配准, 2026-09-08; 残差 0.04mm):
#   q_model[i] = SIGNS[i] * (q_fw[i] + TH_HOME[i])
# 依据: 固件 DH 表 θ-home [0,-90,90,0,0,0]; 轴归一化符号 +,-,-,+,+,+
# 系统关系(诊断用): p_fw ≈ p_model + (60.85, 0, 171.14)mm
SIGNS = [1, -1, -1, 1, 1, 1]
TH_HOME = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]


def fw_to_model(angles_deg):
    """输入固件关节角(度, GETJPOS), 输出 URDF 模型关节角(度)。"""
    return [SIGNS[i] * (angles_deg[i] + TH_HOME[i]) for i in range(6)]


class DummyArmSystem(Node):
    def __init__(self):
        super().__init__('dummyarm_system')
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('rate', 50.0)

        port = self.get_parameter('port').value
        rate = self.get_parameter('rate').value

        self._link = DummyArmLink(port)
        self._io_lock = threading.Lock()          # 串口独占锁(timer 与 service 共用)
        self._enabled = False

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        period = 1.0 / rate
        self.create_timer(period, self._poll_once)
        self.get_logger().info(f'桥接已启动: {port} @ {rate:.0f}Hz (失能状态, 不自动运动)')

        for name in ('start', 'disable', 'stop', 'home', 'reset'):
            self.create_service(Trigger, f'/dummyarm_system/{name}', self._make_handler(name))

    # ---------- 周期状态 ----------
    def _poll_once(self):
        with self._io_lock:
            try:
                joints = self._link.get_joints()   # 度, [Joint1..6]
            except DummyArmLinkError as e:
                self.get_logger().warn(f'读取失败: {e}')
                return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.name = JOINT_NAMES
        model_deg = fw_to_model(joints)
        msg.position = [j * DEG2RAD for j in model_deg]
        self._pub.publish(msg)

    # ---------- 管理服务 ----------
    def _make_handler(self, name):
        def handler(req, resp):
            with self._io_lock:
                try:
                    if name == 'start':
                        self._link.start(); self._enabled = True
                    elif name == 'disable':
                        self._link.disable(); self._enabled = False
                    elif name == 'stop':
                        self._link.stop(); self._enabled = False
                    elif name == 'home':
                        if not self._enabled:
                            self._link.start(); self._enabled = True
                        self._link.home()
                    elif name == 'reset':
                        if not self._enabled:
                            self._link.start(); self._enabled = True
                        self._link.reset()
                except DummyArmLinkError as e:
                    resp.success = False
                    resp.message = str(e)
                    return resp
            resp.success = True
            resp.message = f'{name} ok'
            return resp
        return handler

    def destroy_node(self):
        try:
            with self._io_lock:
                self._link.disable()
        except Exception:
            pass
        self._link.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DummyArmSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
