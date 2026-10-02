#!/usr/bin/env python3
"""
prepare_yolo_dataset.py - 从 AnyLabeling 标注目录抽取数据并转换为标准 YOLO-seg 格式

特点：
1. 原数据集完全只读保护，支持按图片名截止点抽取当前已标注完的切片；
2. 将多边形绝对坐标无损转换为归一化相对坐标；
3. 对缺失 JSON 或空 JSON 的图片自动生成 0 字节 txt 作为背景负样本；
4. 按照设定比例（默认 8:2）进行确定性随机划分（可复现 seed=42）；
5. 自动输出对应实验版本的 dataset.yaml。
"""

import os
import sys
import json
import random
import shutil
import argparse
from pathlib import Path
from PIL import Image

CLASS_MAP = {
    "orange": 0,
    "pedicel": 1,
    "occluded_orange": 2,
    "tree": 3
}


def parse_args():
    parser = argparse.ArgumentParser(description="Prepare YOLO-seg dataset from AnyLabeling")
    parser.add_argument("--source-dir", type=str,
                        default="/Users/bohn/04_Project/05_Orange/MyOrangeDataset/processed_for_annotation/images",
                        help="Source images directory containing annotations")
    parser.add_argument("--cutoff-image", type=str, default="d435i_s01_1764981837699.png",
                        help="Last annotated image name in alphabetical order")
    parser.add_argument("--version", type=str, default="trial_v1",
                        help="Dataset version name")
    parser.add_argument("--val-ratio", type=float, default=0.2,
                        help="Validation set ratio (default 0.2)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for deterministic split")
    parser.add_argument("--output-base", type=str,
                        default="/Users/bohn/04_Project/02_My_Dummy/PickOrangeDummy/dummyarm_AImodel/datasets",
                        help="Output directory for partitioned dataset")
    parser.add_argument("--config-dir", type=str,
                        default="/Users/bohn/04_Project/02_My_Dummy/PickOrangeDummy/dummyarm_AImodel/config",
                        help="Directory to save dataset yaml")
    return parser.parse_args()


def convert_polygon_to_yolo(points, img_w, img_h):
    """将绝对多边形坐标序列转换为归一化序列，并钳制在 [0, 1]"""
    norm_pts = []
    for x, y in points:
        nx = max(0.0, min(1.0, float(x) / img_w))
        ny = max(0.0, min(1.0, float(y) / img_h))
        norm_pts.extend([nx, ny])
    return norm_pts


def main():
    args = parse_args()
    print("=" * 60)
    print(f"📦 开始抽取并生成 YOLO-seg 数据集: 版本 [{args.version}]")
    print(f"  - 源图片目录: {args.source_dir}")
    print(f"  - 截断图片名: {args.cutoff_image}")
    print(f"  - 划分比例:   Train: {1.0 - args.val_ratio:.1f} / Val: {args.val_ratio:.1f}")
    print("=" * 60)

    src_dir = Path(args.source_dir)
    if not src_dir.exists():
        print(f"❌ 源目录不存在: {src_dir}")
        sys.exit(1)

    all_files = sorted(os.listdir(src_dir))
    image_files = [f for f in all_files if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

    # 确定截断索引
    cutoff_idx = -1
    for idx, f in enumerate(image_files):
        if f == args.cutoff_image:
            cutoff_idx = idx
            break

    if cutoff_idx == -1:
        print(f"⚠️ 未直接匹配到截断文件名 {args.cutoff_image}，尝试使用最后一个包含对应 JSON 的图片...")
        for idx in range(len(image_files) - 1, -1, -1):
            stem = os.path.splitext(image_files[idx])[0]
            if (src_dir / f"{stem}.json").exists():
                cutoff_idx = idx
                break

    target_images = image_files[:cutoff_idx + 1]
    print(f"✔ 成功锁定已标注目标区间图片共: {len(target_images)} 张 (索引 0 ~ {cutoff_idx})")

    # 目标输出路径
    dest_dir = Path(args.output_base) / args.version
    img_train_dir = dest_dir / "images" / "train"
    img_val_dir = dest_dir / "images" / "val"
    lbl_train_dir = dest_dir / "labels" / "train"
    lbl_val_dir = dest_dir / "labels" / "val"

    for d in [img_train_dir, img_val_dir, lbl_train_dir, lbl_val_dir]:
        d.mkdir(parents=True, exist_ok=True)

    # 统计数据
    label_stats = {name: 0 for name in CLASS_MAP}
    neg_count = 0
    pos_count = 0

    # 转换所有样本到临时缓存
    converted_samples = []

    for img_name in target_images:
        src_img_path = src_dir / img_name
        stem = src_img_path.stem
        src_json_path = src_dir / f"{stem}.json"

        # 读取图片尺寸
        try:
            with Image.open(src_img_path) as im:
                img_w, img_h = im.size
        except Exception as e:
            print(f"❌ 读取图片尺寸失败 {src_img_path}: {e}")
            continue

        lines = []
        is_negative = True

        if src_json_path.exists():
            try:
                with open(src_json_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                shapes = data.get("shapes", [])
                for s in shapes:
                    lbl = s.get("label", "").strip()
                    if lbl not in CLASS_MAP:
                        continue
                    class_id = CLASS_MAP[lbl]
                    pts = s.get("points", [])
                    if len(pts) < 3:
                        continue
                    norm_pts = convert_polygon_to_yolo(pts, img_w, img_h)
                    pts_str = " ".join(f"{coord:.6f}" for coord in norm_pts)
                    lines.append(f"{class_id} {pts_str}\n")
                    label_stats[lbl] += 1
                    is_negative = False
            except Exception as e:
                print(f"⚠️ 解析 JSON 出错 {src_json_path}: {e}")

        if is_negative:
            neg_count += 1
        else:
            pos_count += 1

        converted_samples.append({
            "img_name": img_name,
            "src_img_path": src_img_path,
            "label_lines": lines,
            "is_negative": is_negative
        })

    # 随机打乱并划分
    random.seed(args.seed)
    random.shuffle(converted_samples)

    val_size = int(len(converted_samples) * args.val_ratio)
    val_samples = converted_samples[:val_size]
    train_samples = converted_samples[val_size:]

    print(f"📊 划分结果: 训练集 {len(train_samples)} 张，验证集 {len(val_samples)} 张")

    # 写入文件
    def save_split(samples, target_img_dir, target_lbl_dir):
        for s in samples:
            # 拷贝图片
            shutil.copy2(s["src_img_path"], target_img_dir / s["img_name"])
            # 写入标签 txt (即便内容为空也要生成 0 字节文件)
            lbl_file = target_lbl_dir / f"{Path(s['img_name']).stem}.txt"
            with open(lbl_file, "w", encoding="utf-8") as lf:
                lf.writelines(s["label_lines"])

    save_split(train_samples, img_train_dir, lbl_train_dir)
    save_split(val_samples, img_val_dir, lbl_val_dir)

    # 自动生成 dataset yaml
    yaml_dest = Path(args.config_dir) / f"{args.version}.yaml"
    Path(args.config_dir).mkdir(parents=True, exist_ok=True)

    yaml_content = f"""# YOLOv11 Instance Segmentation Dataset Config - [{args.version}]
path: {dest_dir.resolve()}
train: images/train
val: images/val

names:
  0: orange
  1: pedicel
  2: occluded_orange
  3: tree
"""
    with open(yaml_dest, "w", encoding="utf-8") as yf:
        yf.write(yaml_content)

    print("\n" + "=" * 60)
    print(f"🎉 数据集转换与划分成功！")
    print(f"  - 总样本数: {len(converted_samples)} (正样本: {pos_count}, 纯负样本: {neg_count})")
    print(f"  - 类别实例统计: {label_stats}")
    print(f"  - 训练集路径: {img_train_dir} ({len(train_samples)} 张)")
    print(f"  - 验证集路径: {img_val_dir} ({len(val_samples)} 张)")
    print(f"  - YAML 配置文件: {yaml_dest.resolve()}")
    print("=" * 60)


if __name__ == "__main__":
    main()
