# 柑橘采摘数据集标注与 AI 辅助处理全流程指南 (Mac 篇)

> **项目**：PickOrangeDummy 低位柑橘采摘机器人  
> **适用环境**：macOS (Apple Silicon M系列，如 MacBook Pro M4 Max)  
> **核心工具**：AnyLabeling / X-AnyLabeling + Segment Anything (SAM) / YOLO-World  
> **目标模型**：YOLOv8-seg / YOLOv11-seg 实例分割模型（用于 3D 掩码果心与果梗法向提取）

---

## 一、为什么首选 X-AnyLabeling？

对于 14,000+ 张数量级的果园数据集，如果采用传统纯手动打点（一个果实点 20~30 个多边形轮廓点），人工成本极其高昂。
**X-AnyLabeling** 是目前开源界最优秀的自动标注神器：
1. **内置 Segment Anything (SAM / MobileSAM / EfficientViT-SAM)**：在柑橘果实上**只需鼠标点击一下（Point 提示）**，AI 自动毫秒级吸附出完美柑橘果体边缘；
2. **支持 YOLO-World 开放词表自动检测**：输入标签名 `orange`，整张图的所有柑橘自动全部打框/分割，人工仅需检查核对与微调；
3. **完美适配 Apple Silicon**：Mac 上可通过 MPS / CPU 流畅运行，一键直接导出标准 **YOLO 格式**（含 segmentation 掩码坐标），无需繁琐的格式转换。

---

## 二、Mac 上的安装与配置

### 方案 1：直接下载安装包（最推荐，开箱即用）
1. 访问 GitHub Releases 页面：
   - [X-AnyLabeling GitHub Releases](https://github.com/CVHub520/X-AnyLabeling/releases)
2. 找到最新版本的 macOS 安装包：
   - 命名通常为：`X-AnyLabeling-macOS-arm64.dmg`（Apple Silicon 架构）；
3. 下载后双击打开，将 `X-AnyLabeling.app` 拖入 `Applications`（应用程序）文件夹；
4. 若初次打开提示“无法打开，因为来自未识别的开发者”：
   - 进入 Mac **系统设置 -> 隐私与安全性**，往下拉点击“仍要打开”即可。

### 方案 2：基于 Conda / Python 环境安装（方便调用最新模型）
如果你习惯在终端环境使用：
```bash
# 创建并激活专用虚拟环境
conda create -n label python=3.10 -y
conda activate label

# 安装 X-AnyLabeling
pip install x-anylabeling

# 启动
x-anylabeling
```

---

## 三、针对低位柑橘采摘机器人的标签规范定义

我们的机械臂搭载了自研的**“双层锥型侧边开孔旋切”**末端执行器。为了使后续采摘策略（进给方向计算、套果深度、旋切位置）达到最优，推荐建立以下标签体系：

| 标签类别 (Label) | 标注形式 | 作用与物理意义 | 必标/选标 |
| :--- | :--- | :--- | :--- |
| `orange` | **Polygon (多边形) / 掩码** | 完整的可见柑橘果体，用于提取果心质心三维坐标 $(X_c, Y_c, Z_c)$ 以及计算果实包围球径 | **必标（核心）** |
| `pedicel` | **Line / Rectangle / Polygon** | 柑橘上方 1~3cm 的果梗部位。用于辅助确定末端导料槽的进给姿态与切刀切削平面 | 建议重点样本标注 |
| `occluded_orange` | **Polygon (多边形)** | 枝叶遮挡面积超过 50% 的果实。训练模型识别“难采/受阻果”，使路径规划器优先避让或选择次优果实 | 建议样本较多时标注 |

---

## 四、高效标注实战工作流（从 0 到 14000 张）

### 第 1 步：准备与精简数据集（建议先做去冗余）
14,000 张图如果包含大量连续视频抽帧，建议先过滤掉静态重复帧（抽帧步长建议 5~10 帧保留一张），精炼出 **2,000 ~ 3,000 张高质量典型场景图片**（包含：顺光、逆光、树荫阴影、枝叶遮挡、多果重叠簇集）。

### 第 2 步：配置自动标注模型
1. 打开 X-AnyLabeling 软件，左侧工具栏点击 **“打开目录” (Open Dir)**，选择你的柑橘图片文件夹；
2. 左上角菜单栏切换到 **“自动标注” (Auto Labeling)** 模式（图标为一个带闪电的机器人）；
3. 在模型下拉列表中选择：
   - **Segment Anything (SAM) 系列**：推荐选择 `Segment Anything (ViT-B)` 或 `MobileSAM`（速度极快，在 M4 Max 上几毫秒出结果）；
   - 初次选择会自动从官方下载模型权重文件（也可以手动放入 `~/.anylabeling/models/`）。

### 第 3 步：极速打标操作技巧
1. **单点吸附分割（Point 交互）**：
   - 鼠标左键点击一个柑橘中心：绿色正样本点，模型瞬间吸附出柑橘边缘；
   - 若边缘溢出到了背景树叶，鼠标右键点击多余区域：红色负样本点，模型自动精准剔除多余边缘；
   - 按快捷键 `Space`（空格）或 `Enter`：保存当前多边形对象，并选择标签 `orange`。
2. **多目标快速标注**：
   - 一张图内有多个柑橘时，依次左键点击各个柑橘，全部吸附完成后按快捷键保存。
3. **快捷键一览**：
   - `A`：上一张图片；
   - `D`：下一张图片（自动保存上一张）；
   - `Ctrl + Z`：撤销上一步点击；
   - `Del`：删除选中多边形。

---

## 五、导出与数据集划分（YOLO-seg 标准格式）

标注完成后，直接导出为 YOLO 格式：
1. 点击菜单栏 **“导出” (Export) -> “导出 YOLO 格式” (Export YOLO Format)**；
2. 选择 **“YOLOv8/v11 Segment (Polygon)”** 格式导出；
3. 输出目录将生成对应的 `.txt` 标注文件（每行：`class_id x1 y1 x2 y2 ... xn yn` 归一化多边形顶点坐标）。

按照标准结构将数据整理为如下目录：
```text
citrus_dataset/
├── dataset.yaml
├── images/
│   ├── train/    # 约 80% 图片
│   └── val/      # 约 20% 验证图片
└── labels/
    ├── train/    # 对应标签 txt
    └── val/
```

### `dataset.yaml` 配置文件示例：
```yaml
path: /Users/yourname/Documents/citrus_dataset  # 绝对路径
train: images/train
val: images/val

names:
  0: orange
  1: pedicel
  2: occluded_orange
```

---

## 六、在 Mac M4 Max 上进行模型训练与导出 (PyTorch MPS)

MacBook Pro M4 Max 拥有强大的统一内存与 GPU 算力，可以直接使用 Apple Silicon 的 `mps` 加速后端进行训练，无需搭建复杂的 Linux GPU 环境：

```bash
# 1. 安装 ultralytics
pip install ultralytics

# 2. 在 M4 Max 上启动 YOLOv11 实例分割训练 (启用 mps 加速)
yolo segment train \
    data=dataset.yaml \
    model=yolo11n-seg.pt \
    epochs=100 \
    imgsz=640 \
    batch=32 \
    device=mps \
    workers=8

# 3. 训练完成后导出为标准 ONNX 格式 (供 VENTUNO Q 或 ROS2 节点高速推理)
yolo export \
    model=runs/segment/train/weights/best.pt \
    format=onnx \
    opset=12 \
    simplify=True
```

导出得到的 `best.onnx` 模型即可拷贝至机械臂 VENTUNO Q 开发板上，由 ROS 2 视觉感知节点直接加载运行。
