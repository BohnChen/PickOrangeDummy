#!/usr/bin/env python3
"""chassis_node — /cmd_vel ⇄ Arduino RouterBridge RPC，带租约心跳与安全默认。

设计要点（见 docs/architecture_v2.md §6 / §10 / §15）：

  1. **单一写者**：底盘串口只有 arduino-router 与它背后的本节点在写。
  2. **心跳独立于 ROS executor**：20 Hz 续租约跑在独立线程里，
     executor 卡住也不会导致车失去租约（反而会主动收油）。
  3. **默认不动车**：`allow_motion` 默认 False。此时节点只做
     status 轮询与状态发布，绝不发 drive/creep——方便在没有安全确认时联调。
  4. **转向取负号**：底盘 steer>0 = 右，ROS angular.z>0 = 左。
  5. **有限步进**：Creep 一定在 duration 后 stop()，且有超时兜底。

参数：
  rpc_target    unix:/var/run/arduino-router.sock | tcp:127.0.0.1:7501
  allow_motion  false（默认）— true 才会真的下发运动指令
  max_speed_pct 50
  k_linear      100.0   %/(m/s)
  k_angular     100.0   %/(rad/s)
  cmd_timeout_s 0.30    /cmd_vel 过期即收油
  v_per_pct     0.008   m/s per 1% 油门（占位，需标定）
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from pickorange_interfaces.action import Creep
from pickorange_interfaces.msg import ChassisState

from .rpc import RpcClient, RpcError

THR_MIN_EFFECTIVE = 30.0   # 实测：15% 车不动，30% 才起步
CREEP_MIN_STEP_M = 0.10    # 斜率限制决定的最小有效步长（见 README）


class ChassisNode(Node):
    def __init__(self):
        super().__init__("chassis_node")

        self.declare_parameter("rpc_target", "unix:/var/run/arduino-router.sock")
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("max_speed_pct", 50)
        self.declare_parameter("k_linear", 100.0)
        self.declare_parameter("k_angular", 100.0)
        self.declare_parameter("cmd_timeout_s", 0.30)
        self.declare_parameter("v_per_pct", 0.008)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("status_period_s", 1.0)

        gp = self.get_parameter
        self.rpc_target = gp("rpc_target").value
        self.allow_motion = bool(gp("allow_motion").value)
        self.max_speed = float(gp("max_speed_pct").value)
        self.k_linear = float(gp("k_linear").value)
        self.k_angular = float(gp("k_angular").value)
        self.cmd_timeout = float(gp("cmd_timeout_s").value)
        self.v_per_pct = float(gp("v_per_pct").value)
        self.rate_hz = float(gp("rate_hz").value)
        self.status_period = float(gp("status_period_s").value)

        self.rpc = RpcClient(self.rpc_target, default_timeout=1.0)

        # 共享状态
        self._lock = threading.Lock()
        self._thr = 0.0
        self._steer = 0.0
        self._last_cmd_t = 0.0
        self._paused = False            # 互锁/急停时为 True
        self._sending = False           # 当前是否正在下非零指令

        # 从固件 status() 解析出来的缓存
        self._fw = {"latched": 0, "leased": 0, "left_ms": 0,
                    "wd_stops": 0, "stops": 0}

        cb = ReentrantCallbackGroup()

        self.create_subscription(TwistStamped, "cmd_vel", self._on_cmd_vel, 10,
                                 callback_group=cb)
        self.state_pub = self.create_publisher(ChassisState, "chassis/state", 10)

        self.create_service(Trigger, "chassis/estop", self._srv_estop,
                            callback_group=cb)
        self.create_service(Trigger, "chassis/arm", self._srv_arm,
                            callback_group=cb)
        self.create_service(Trigger, "chassis/stop", self._srv_stop,
                            callback_group=cb)

        self._creep_srv = ActionServer(
            self, Creep, "chassis/creep",
            execute_callback=self._creep_execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        # 心跳线程：独立于 ROS executor
        self._running = True
        self._hb = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._hb.start()

        self.create_timer(0.1, self._publish_state)

        self.get_logger().info(
            f"chassis_node up | target={self.rpc_target} | "
            f"allow_motion={self.allow_motion}")
        if not self.allow_motion:
            self.get_logger().warn(
                "allow_motion=false：只看状态、不驱动电机。"
                "确认车架空/周围无人后，用 --ros-args -p allow_motion:=true 开启。")

    # ------------------------------------------------------------------ 回调
    def _on_cmd_vel(self, msg: TwistStamped):
        v = msg.twist.linear.x
        w = msg.twist.angular.z
        # 底盘：steer>0 = 右；ROS：angular.z>0 = 左 ⇒ 取负
        thr = max(-self.max_speed, min(self.max_speed, self.k_linear * v))
        steer = max(-100.0, min(100.0, -self.k_angular * w))
        with self._lock:
            self._thr = thr
            self._steer = steer
            self._last_cmd_t = time.monotonic()

    def _srv_estop(self, _req, resp: Trigger.Response):
        try:
            self.rpc.call("estop")
            with self._lock:
                self._paused = True
                self._thr = 0.0
                self._steer = 0.0
            resp.success = True
            resp.message = "latched"
        except RpcError as exc:
            resp.success = False
            resp.message = str(exc)
        return resp

    def _srv_arm(self, _req, resp: Trigger.Response):
        try:
            self.rpc.call("arm")
            with self._lock:
                self._paused = False
                self._thr = 0.0
                self._steer = 0.0
                self._last_cmd_t = 0.0
            resp.success = True
            resp.message = "armed (outputs at neutral)"
        except RpcError as exc:
            resp.success = False
            resp.message = str(exc)
        return resp

    def _srv_stop(self, _req, resp: Trigger.Response):
        try:
            self.rpc.call("stop")
            with self._lock:
                self._thr = 0.0
                self._steer = 0.0
                self._last_cmd_t = 0.0
            resp.success = True
            resp.message = "stopped (neutral, coasting)"
        except RpcError as exc:
            resp.success = False
            resp.message = str(exc)
        return resp

    # ------------------------------------------------------------ 心跳线程
    def _heartbeat_loop(self):
        period = 1.0 / self.rate_hz
        next_status = 0.0
        while self._running:
            now = time.monotonic()
            with self._lock:
                fresh = (now - self._last_cmd_t) <= self.cmd_timeout
                thr, steer = (self._thr, self._steer) if fresh else (0.0, 0.0)
                paused = self._paused

            try:
                if self.allow_motion and not paused and (thr or steer):
                    self.rpc.call("drive", int(round(thr)), int(round(steer)),
                                  timeout=0.3)
                    self._sending = True
                elif self._sending:
                    # 从「有指令」掉到「没指令」时补一帧中位，别等租约过期
                    self.rpc.call("stop", timeout=0.3)
                    self._sending = False

                if now >= next_status:
                    next_status = now + self.status_period
                    self._poll_status()
            except RpcError as exc:
                self.get_logger().warn(f"rpc: {exc}", throttle_duration_sec=2.0)
                self._sending = False

            time.sleep(period)

    def _poll_status(self):
        raw = self.rpc.call("status", timeout=0.5)
        txt = raw.decode() if isinstance(raw, bytes) else str(raw)
        out = {}
        for part in txt.split():
            if "=" in part:
                k, _, v = part.partition("=")
                out[k] = v
        with self._lock:
            self._fw["latched"] = int(out.get("latched", 0))
            self._fw["leased"] = int(out.get("leased", 0))
            left = out.get("left", "0ms").rstrip("ms")
            self._fw["left_ms"] = int(left or 0)
            self._fw["wd_stops"] = int(out.get("wd_stops", 0))
            self._fw["stops"] = int(out.get("stops", 0))

    # --------------------------------------------------------------- 发布
    def _publish_state(self):
        with self._lock:
            thr, steer = self._thr, self._steer
            fw = dict(self._fw)
            paused = self._paused

        msg = ChassisState()
        msg.mode = ChassisState.MODE_FAULT if fw["latched"] else (
            ChassisState.MODE_FWD if thr > 0 else
            ChassisState.MODE_REV if thr < 0 else ChassisState.MODE_STOP)
        msg.throttle_pct = float(thr)
        msg.steer_pct = float(steer)
        msg.leased = bool(fw["leased"])
        msg.lease_remaining_s = fw["left_ms"] / 1000.0
        msg.estop_latched = bool(fw["latched"]) or paused
        msg.wd_stops = fw["wd_stops"]
        msg.stop_calls = fw["stops"]
        msg.battery_v = 0.0
        msg.cmd_seq = 0
        self.state_pub.publish(msg)

    # ------------------------------------------------------------ Creep Action
    def _creep_execute(self, goal_handle):
        goal = goal_handle.request
        dist = float(goal.distance_m)
        spd = float(goal.max_speed) or 30.0
        result = Creep.Result()
        feedback = Creep.Feedback()

        if not self.allow_motion:
            goal_handle.abort()
            result.success = False
            result.error_code = "DRY_RUN"
            result.message = "allow_motion=false"
            return result

        if abs(dist) < CREEP_MIN_STEP_M:
            self.get_logger().warn(
                f"creep {dist:.3f}m 小于最小有效步长 {CREEP_MIN_STEP_M}m，"
                "已被钳制（斜率限制下更短的步进无法起步）")
            dist = math.copysign(CREEP_MIN_STEP_M, dist)

        spd = max(THR_MIN_EFFECTIVE, min(self.max_speed, abs(spd)))
        v = max(0.02, self.v_per_pct * spd)          # m/s 占位估计
        dur = abs(dist) / v
        timeout = max(dur, 1.0) + 1.0

        thr = math.copysign(spd, dist)
        self.get_logger().info(
            f"creep {dist:+.3f}m @ {spd:.0f}% (v≈{v:.3f} m/s, {dur:.2f}s)")

        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < dur:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    break
                if time.monotonic() - t0 > timeout:
                    break
                self.rpc.call("drive", int(round(thr)), 0, timeout=0.3)
                elapsed = time.monotonic() - t0
                feedback.traveled_m_est = float(math.copysign(v * elapsed, dist))
                feedback.remaining_m = float(max(0.0, abs(dist) - v * elapsed))
                goal_handle.publish_feedback(feedback)
                time.sleep(1.0 / self.rate_hz)
        except RpcError as exc:
            result.success = False
            result.error_code = "RPC"
            result.message = str(exc)
            try:
                self.rpc.call("stop", timeout=0.3)
            except RpcError:
                pass
            goal_handle.abort()
            return result
        finally:
            try:
                self.rpc.call("stop", timeout=0.3)
            except RpcError:
                pass

        result.traveled_m_est = float(math.copysign(v * (time.monotonic() - t0), dist))
        result.success = True
        result.error_code = "SUCCESS"
        goal_handle.succeed()
        return result

    # ---------------------------------------------------------------- 收尾
    def destroy_node(self):
        self._running = False
        try:
            self.rpc.call("stop", timeout=0.3)
        except RpcError:
            pass
        self.rpc.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
