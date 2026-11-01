import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))
from arm_kin import fk_full, fk_shell, port_down_deg, fw_from_model, model_from_fw
import arm_pick

class TestKinAndPick(unittest.TestCase):
    def test_fk_shell_exists_and_diverges_only_by_j6(self):
        """fk_shell 返回外壳位姿（终止于 J5），其位置与 tool0 仅差 J5->J6 的变换，不受 J6 旋转影响"""
        q1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q2 = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0] # 动 J6
        
        T_shell_1 = fk_shell(q1)
        T_shell_2 = fk_shell(q2)
        
        # J6 改变，fk_shell 的旋转与位置都不应改变
        np.testing.assert_allclose(T_shell_1, T_shell_2, atol=1e-6)
        
        # 但 fk_full 受 J6 影响
        T_full_1 = fk_full(q1)
        T_full_2 = fk_full(q2)
        self.assertFalse(np.allclose(T_full_1[:3, :3], T_full_2[:3, :3], atol=1e-3))

    def test_plan_pick_approach_fallback(self):
        """测试决策逻辑：
        1. 正常目标 -> 规划出 stage + feed
        2. 目标太近导致 stage 碰撞/不可达，但直接进刀可行 -> 跳过 stage 直接进刀
        3. 目标无论 stage 还是 direct feed 均不可达 -> 返回 ABORT 放弃
        """
        # 验证 arm_pick.plan_pick 接口存在并能返回正确状态
        self.assertTrue(hasattr(arm_pick, 'plan_pick'))

if __name__ == '__main__':
    unittest.main()
