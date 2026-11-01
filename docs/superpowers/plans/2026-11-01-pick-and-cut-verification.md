# 采摘末端闭环与 ROS 2 能力层封装实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成 J6 零点与行程校准，支持“直接进刀/放弃不可行目标”的保护决策，打通“仰角套果 -> 剪切 -> 退刀”的单次采摘闭环，并将底层 Python 算法封装为标准的 ROS 2 Action Server。

**Architecture:** 
1. 运动学与安全策略：扩展 `arm_kin.py`（增加 `fk_shell()`）与 `arm_pick.py`，支持当果实靠近车身使准备位不可达时，评估直接进刀可行性；若直接进刀亦不可达，则主动放弃并安全退出。
2. 标定与硬件防线：基于 J6 0~62° 行程重测零位，更新 `config/j6_blade.json`，确保三道闸锁闭。
3. 节点与能力层：新建 `pickorange_arm` 节点，单进程独占 `/dev/ttyACM1`，提供标准的 `MoveToPose` 与 `CutStem` ROS 2 Action Server。

**Global Constraints:**
- 必须严格遵守 J3 [35°, 180°] 安全红线与车体包围盒保护。
- 运动过程中 J6 轴硬锁，仅 `do_cut()` 允许 J6 在 0~62° 内单程剪切。
- 每一个阶段代码变动均使用 git 跟踪。

---

### Task 1: J6 零位重测与 0~62° 行程标定配置

**Files:**
- Create/Verify: `scripts/zero_j6.py`
- Modify: `config/j6_blade.json`

- [ ] **Step 1: 检查 zero_j6.py 逻辑并运行只读测量**
- [ ] **Step 2: 确认 J6 0~62° 机械行程与当前零位**
- [ ] **Step 3: 更新 config/j6_blade.json**
- [ ] **Step 4: 提交并记录 commit**

---

### Task 2: 修正外壳运动学 `fk_shell()` 与“靠近直接进刀/放弃”策略

**Files:**
- Modify: `scripts/arm_kin.py`
- Modify: `scripts/arm_pick.py`
- Test: `tests/test_kin_and_pick.py`

- [ ] **Step 1: 编写单元测试验证 fk_shell 与放弃策略**
- [ ] **Step 2: 运行测试验证失败**
- [ ] **Step 3: 实现 fk_shell() 及直接进刀/放弃决策逻辑**
- [ ] **Step 4: 运行测试验证通过**
- [ ] **Step 5: 提交并记录 commit**

---

### Task 3: 仰角进刀套果与剪切闭环验证

**Files:**
- Modify/Run: `scripts/arm_pick.py`

- [ ] **Step 1: dry-run 验证不同果实高度下的逆解可行域与出料口偏角**
- [ ] **Step 2: 实机执行仰角套果 -> 保持 -> do_cut() 剪切 -> 退刀归位流程**
- [ ] **Step 3: 记录测试结果并提交**

---

### Task 4: 封装为 ROS 2 Action Server (`pickorange_arm`)

**Files:**
- Create: `src/pickorange_arm/package.xml`
- Create: `src/pickorange_arm/setup.py`
- Create: `src/pickorange_arm/pickorange_arm/arm_action_server.py`
- Test: 单元测试 / 客户端请求测试

- [ ] **Step 1: 创建 ROS 2 Python 包骨架**
- [ ] **Step 2: 实现 MoveToPose 与 CutStem Action Server**
- [ ] **Step 3: colcon build 编译并验证**
- [ ] **Step 4: 提交并推送到远端仓库**
