# Fusion → URDF 导出流程（DummyArm 实测版）

> 2026-09-10 整理。适用：把 Fusion 里的机械臂设计导出为可直接在 RViz 用的 ROS 2 description 包。
> 结论已验证：修正后的导出通过（坐标 ✓、限位 ✓、滑块+RViz ✓）。

## 1. 工具

- 导出器 = **Adriaeik/fusion2URDF v3.1.0**（Fusion 360 脚本：`Shift+S` → 脚本 → My Scripts → Run）
- **补丁版副本**：`~/dummyurdf/fusion2URDF-patched/`（含 double-lift 修复 + 安装说明）
- 补丁文件：`docs/fusion2URDF-root-joint-fix.patch`（对全新 clone 用 `git apply` 重打）
- 注意：上游最新 commit `2044690` 仍含 double-lift bug

## 2. 已知问题与修复（三个坑）

1. **double-lift（根组件拥有的关节）**：关节 origin 出现 0.5~1.1 m 假平移、部件散落。
   - 修复：`core/robot_model.py::_compute_joint_global_origin` 增加 `Root-owned` 分支
     （定义组件=根组件且 geometryOrOrigin one/two 一致时直接用世界坐标，不再 lift）。
2. **生成的 `display.launch.py` 缺 `ParameterValue(value_type=str)`** → `ros2 launch` 直接崩
   （`Unable to parse the value of parameter robot_description as yaml`）。
   - 修复：launch 里 `robot_description = ParameterValue(Command([...]), value_type=str)`。
   - 上游模板未修（`core/package_generator.py`），每次导出后需手工补或后处理。
3. **限位不做 rebase**：导出器把 Fusion 限位**原样**写进 URDF，但 URDF 的 0° = 导出姿态，
   而 Fusion 的 0° = 关节**创建时**的姿态。两者差 `V_d`（该关节当前值）→ 限位整体错位。
   - 规避：设计时让每个关节的零位 = 想要的 0°（见 §3）；或导出前把关节驱动回 0°；
     或限位填 `V_d + 目标值`（超 360° 用等价负数写法）。

## 3. 设计规则（Fusion 侧）

- **关节零位**：让每个关节在 **REST 姿态时为 0°**（把臂摆到 REST 再创建/重建关节，零件不会动）。
- **限位（相对 REST，本机真值）**：
  J1 `±170°` / J2 `[0°,163°]` / J3 `[-145°,0°]` / J4 `±180°` / J5 `±95°` / J6 不设限位（continuous）
- 导出后的 0° = 导出时的设计姿态；导出器 mock 的初始值 0.0 即该姿态。
- 大件导出慢（本例 149 MB 网格）；网格为 DAE（视觉）+ STL（碰撞）。

## 4. 导出后验证清单

1. **坐标**：关节 origin 应为小值（`_1 ≈ (-0.015, 0.035, -0.0375)`、`_3 ≈ (0.141, 0.038, 0.034)`）；
   若出现 0.5~1.1 m = double-lift 补丁没生效（检查安装目录的 `core/robot_model.py` 是否含 `Root-owned`，换文件后需重启 Fusion）。
2. **启动**：`ros2 launch <pkg> display.launch.py` 能起（若崩 = 缺 ParameterValue）。
3. **初始姿态**：`/joint_states` 全 0 时 `base_link→tool0 ≈ (-0.014, -0.048, 0.043)`。
4. **拖动**：滑块版拖动，轴/方向与真机一致、部件始终不散。

## 5. 本机验证记录

- 2026-09-10 20:39 导出（修正版）：坐标 ✓、限位 ✓（J2 `[0,163]`）、滑块+RViz ✓。
- 项目内 `src/dummyarm_cad_description` = **直接采用该导出**（改名 + 补 launch 包装 + 加滑块 launch），
  mock 版与滑块版均验证通过（`tool0 = (-0.014, -0.048, 0.043)` @ 设计姿态）。
- 导出器自带 launch 是 mock 控制版（无滑块）；滑块版 = `display_jsp.launch.py`（本项目新增）。
