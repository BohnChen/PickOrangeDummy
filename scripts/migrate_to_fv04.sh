#!/usr/bin/env bash
# 把旧仓库（VENTUNO Q 版）中仍然成立的资产迁移到新仓库（FV04 版）。
#
# 用法：
#   ./scripts/migrate_to_fv04.sh            # 真正复制
#   ./scripts/migrate_to_fv04.sh --dry-run  # 只打印将复制什么
#
# 前提：新仓库已 git init 且与远端同步；本脚本不碰新仓库的 .git。
# 设计原则：**只搬"零改动或小改动"的资产**，高模/打印文件/训练数据一律不搬。
set -euo pipefail

OLD="/Users/bohn/04_Project/02_My_Dummy/PickOrangeDummy"
NEW="/Users/bohn/04_Project/02_My_Dummy/PickOrangesFV04"

DRY=""
[[ "${1:-}" == "--dry-run" ]] && DRY="-n"

[[ -d "$OLD" ]] || { echo "旧仓库不存在: $OLD" >&2; exit 1; }
[[ -d "$NEW/.git" ]] || { echo "新仓库不是 git 仓库: $NEW" >&2; exit 1; }
[[ "$OLD" != "$NEW" ]] || { echo "源和目标是同一个目录" >&2; exit 1; }

RSYNC=(rsync -a --itemize-changes $DRY)

echo "== 1/7 目录骨架 =="
for d in docs/hardware hardware config scripts src; do
  [[ -n "$DRY" ]] || mkdir -p "$NEW/$d"
  echo "  mkdir -p $NEW/$d"
done

echo "== 2/7 ROS 包（零改动/小改动） =="
for pkg in pickorange_interfaces pickorange_bringup pickorange_chassis pickorange_arm \
           dummyarm_hardware dummyarm_moveit_config; do
  "${RSYNC[@]}" --exclude='__pycache__' --exclude='*.pyc' \
    "$OLD/src/$pkg/" "$NEW/src/$pkg/"
done

echo "== 3/7 机械臂描述（只要 URDF + STL，不要 143M 高模） =="
"${RSYNC[@]}" --exclude='meshes/export/' --exclude='meshes/export' --exclude='urdf_delete/' \
  "$OLD/src/dummyarm_description/" "$NEW/src/dummyarm_description/"

echo "== 4/7 scripts（精选，不搬 STEP/CAD 一次性流水线） =="
KEEP_SCRIPTS=(
  arm_kin.py arm_pick.py arm_safety.py calib_eye_to_hand.py zero_j6.py j6_homing.py
  disable.py estop.py arm_goto_xyz.py safe_park_and_exit.sh cut.py car.py
  test_chassis_control.py web_dual_cam.py sonar.py pick_and_cut_pipeline.py
  generate_true_urdf.py
)
for f in "${KEEP_SCRIPTS[@]}"; do
  [[ -f "$OLD/scripts/$f" ]] && "${RSYNC[@]}" "$OLD/scripts/$f" "$NEW/scripts/$f"
done

echo "== 5/7 docs（关键文档，不含图册里的大 PNG 之外的东西） =="
KEEP_DOCS=(
  safety_and_incidents.md HANDOFF_交接文档.md chassis_control_verified.md
  M0_spec.md M2_spec_j6_blade_home.md POWER_v2_SPEC.md
  ARCHITECTURE_REVIEW_2026-11-10.md HANDOFF_TO_FV04.md
)
for f in "${KEEP_DOCS[@]}"; do
  [[ -f "$OLD/docs/$f" ]] && "${RSYNC[@]}" "$OLD/docs/$f" "$NEW/docs/$f"
done
for d in diagrams calib; do
  [[ -d "$OLD/docs/$d" ]] && "${RSYNC[@]}" "$OLD/docs/$d/" "$NEW/docs/$d/"
done

echo "== 6/7 配置 / DH 参数 / 测试 =="
"${RSYNC[@]}" "$OLD/config/" "$NEW/config/"
"${RSYNC[@]}" "$OLD/DH_Params/" "$NEW/DH_Params/"
"${RSYNC[@]}" --exclude='__pycache__' "$OLD/tests/" "$NEW/tests/"

echo "== 7/7 总结 =="
if [[ -n "$DRY" ]]; then
  echo "（dry-run：没有写入任何文件）"
else
  du -sh "$NEW"/* 2>/dev/null | sort -rh | head -12
  echo
  echo "下一步："
  echo "  1) 改 ROS 2 版本（package.xml / CMakeLists / launch → Humble）"
  echo "  2) 重写 chassis 传输层（arduino-router RPC → USB 串口 或 板载 PWM）"
  echo "  3) 在 FV04 上先跑 TF/RViz（无硬件）"
fi

echo "注意：未复制 models/、carPhotos/、presentation_assets/、dummyarm_AImodel/datasets|runs、"
echo "      dummyarm_cad_description/、dummyarm_bridge/、dummyarm_bringup/、dummyarm_perception/（按需再取）。"
