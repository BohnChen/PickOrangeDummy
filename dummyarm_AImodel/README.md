# dummyarm_AImodel - 柑橘采摘机械臂 AI 视觉模型工程

本项目模块专注于 **PickOrangeDummy** 机器人的端侧实例分割模型训练、微调、验证与导出。

---

## 1. 目录结构

```text
dummyarm_AImodel/
├── README.md                  # 模块操作指南与速查
├── config/                    # 各实验阶段的数据集配置文件 (.yaml)
│   ├── citrus_classes.yaml    # 核心标签与类别字典规范
│   └── exp001_trial.yaml      # EXP-001 测试实验配置
├── scripts/                   # 自动化流水线脚本
│   ├── prepare_yolo_dataset.py# 数据集格式转换、负样本对齐与划分脚本
│   └── train_yolo11_seg.py    # 模块化训练、指标统计与评估脚本
├── datasets/                  # [Git 忽略] 本地生成的 YOLO-seg 格式数据集 (划分版本)
└── runs/                      # [Git 忽略] 训练产物 (权重 best.pt、混淆矩阵、预测对比图)
```

---

## 2. 标签体系规范 (Label Specification)

严格适配自研“套果-旋切”采摘末端执行器：

| 类别 ID | 标签名 | 几何形式 | 作用与物理意义 |
| :---: | :--- | :---: | :--- |
| **0** | `orange` | 多边形 (Polygon) | 可见完整柑橘果体，用于提取果心三维坐标与进刀中心 |
| **1** | `pedicel` | 多边形 (Polygon) | 果实上方 1~3cm 果梗，用于确定末端旋切刀具的进刀法向与切削平面 |
| **2** | `occluded_orange` | 多边形 (Polygon) | 遮挡 >50% 的果实，供采摘状态机实施分层排序与避碰 |

---

## 3. 标准操作工作流

### 步骤 1：准备/更新数据集
从已标注的 JSON 目录抽取指定批次并转换为 YOLO 格式：
```bash
/Users/bohn/miniconda3/envs/yolov8_env/bin/python dummyarm_AImodel/scripts/prepare_yolo_dataset.py \
    --version trial_v1 \
    --cutoff-image d435i_s01_1764981837699.png \
    --val-ratio 0.2
```

### 步骤 2：启动训练 (支持 Apple Silicon MPS 加速)
```bash
/Users/bohn/miniconda3/envs/yolov8_env/bin/python dummyarm_AImodel/scripts/train_yolo11_seg.py \
    --exp-id EXP-001 \
    --config dummyarm_AImodel/config/exp001_trial.yaml \
    --epochs 50 \
    --batch 16 \
    --imgsz 640
```

### 步骤 3：查看评估记录
所有历史实验记录、指标对比与优化建议均持续归档在：
`docs/柑橘实例分割模型训练与验证记录.md`
