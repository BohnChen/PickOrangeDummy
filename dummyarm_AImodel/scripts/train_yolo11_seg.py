#!/usr/bin/env python3
"""
train_yolo11_seg.py - 柑橘采摘机械臂 YOLOv11 实例分割训练与评估脚本

核心逻辑：
1. 模块化支持实验编号 (e.g. EXP-001)；
2. 自动检测并启用 Apple Silicon MPS 硬件加速；
3. 加载预训练权重 (yolo11n-seg.pt)，执行多轮次精细微调；
4. 训练完成后自动在验证集上评测 Box 与 Mask 的 mAP50 / mAP50-95；
5. 输出标准化评测结果摘要，供写入实验追踪文档。
"""

import os
import sys
import argparse
import json
import torch
from pathlib import Path
from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description="Train YOLOv11 instance segmentation model")
    parser.add_argument("--exp-id", type=str, default="EXP-001",
                        help="Experiment ID for tracking (e.g. EXP-001)")
    parser.add_argument("--config", type=str,
                        default="/Users/bohn/04_Project/02_My_Dummy/PickOrangeDummy/dummyarm_AImodel/config/trial_v1.yaml",
                        help="Path to dataset yaml config")
    parser.add_argument("--model", type=str, default="yolo11n-seg.pt",
                        help="Pretrained model architecture")
    parser.add_argument("--epochs", type=int, default=50,
                        help="Number of training epochs")
    parser.add_argument("--batch", type=int, default=16,
                        help="Batch size")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="Input image size")
    parser.add_argument("--workers", type=int, default=4,
                        help="Number of dataloader workers")
    parser.add_argument("--device", type=str, default="cpu",
                        help="Device to use for training (cpu or mps)")
    parser.add_argument("--output-project", type=str,
                        default="/Users/bohn/04_Project/02_My_Dummy/PickOrangeDummy/dummyarm_AImodel/runs/segment",
                        help="Root directory for run outputs")
    return parser.parse_args()


def main():
    args = parse_args()

    # 针对 PyTorch 2.5.1 在 Apple Silicon MPS 运行 Segmentation 反向传播时的 view stride 兼容性做平滑处理
    # 优先使用 CPU (Apple M4 Max 的强大 CPU 单核与多核跑纳米级网络每轮仅需几秒且零崩溃)
    # 若指定 mps，支持通过参数切换
    device = "cpu" if args.device == "cpu" else ("mps" if torch.backends.mps.is_available() else "cpu")
    if args.device:
        device = args.device

    exp_run_name = f"{args.exp_id.lower()}_{Path(args.model).stem}_{args.epochs}e"

    print("=" * 65)
    print(f"🚀 启动柑橘实例分割模型训练任务 [{args.exp_id}]")
    print(f"  - 基础模型:     {args.model}")
    print(f"  - 数据集配置:   {args.config}")
    print(f"  - 训练轮数:     {args.epochs}")
    print(f"  - Batch Size:   {args.batch}")
    print(f"  - 图像尺寸:     {args.imgsz}")
    print(f"  - 加速硬件设备: {device}")
    print(f"  - 输出工程目录: {args.output_project}/{exp_run_name}")
    print("=" * 65)

    # 实例化 YOLO 模型
    model = YOLO(args.model)

    # 启动训练
    results = model.train(
        data=args.config,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=device,
        project=args.output_project,
        name=exp_run_name,
        exist_ok=True,
        save=True,
        save_period=10,
        verbose=True
    )

    save_dir = Path(results.save_dir) if hasattr(results, 'save_dir') else Path(args.output_project) / exp_run_name
    best_pt = save_dir / "weights" / "best.pt"

    print("\n" + "=" * 65)
    print("🎯 训练阶段圆满结束！开始在验证集上执行精度最终核验...")
    print("=" * 65)

    # 独立载入最佳权重进行评估
    if best_pt.exists():
        val_model = YOLO(str(best_pt))
        metrics = val_model.val(data=args.config, device=device, imgsz=args.imgsz, split="val")

        summary = {
            "exp_id": args.exp_id,
            "best_model_path": str(best_pt),
            "epochs": args.epochs,
            "device": device,
            "metrics": {
                "box_map50": float(metrics.box.map50),
                "box_map50_95": float(metrics.box.map),
                "mask_map50": float(metrics.seg.map50),
                "mask_map50_95": float(metrics.seg.map),
                "fitness": float(metrics.fitness)
            }
        }

        # 写入简明 JSON 供外部文档读取
        summary_file = save_dir / "eval_summary.json"
        with open(summary_file, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)

        print("\n" + "*" * 65)
        print(f"🏆 【{args.exp_id} 评测成绩单】")
        print(f"  - 目标定位 (Box) mAP@0.5:      {metrics.box.map50 * 100:.2f}%")
        print(f"  - 目标定位 (Box) mAP@0.5:0.95: {metrics.box.map * 100:.2f}%")
        print(f"  - 掩码分割 (Mask) mAP@0.5:     {metrics.seg.map50 * 100:.2f}%")
        print(f"  - 掩码分割 (Mask) mAP@0.5:0.95:{metrics.seg.map * 100:.2f}%")
        print(f"  - 最佳权重已保存至:           {best_pt}")
        print(f"  - 验证指标 JSON 已落盘:       {summary_file}")
        print("*" * 65)
    else:
        print(f"⚠️ 未找到保存的权重文件: {best_pt}")


if __name__ == "__main__":
    main()
