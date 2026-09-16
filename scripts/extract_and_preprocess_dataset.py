#!/usr/bin/env python3
"""
extract_and_preprocess_dataset.py - 柑橘数据集多源标准化抽取与索引构建脚本

核心逻辑：
1. 原数据集完全只读保护，所有提取输出至全新的 processed_for_annotation 目录。
2. D435i 数据集处理：
   - 自动跳过 Wrongsession_*；
   - 对 15 个有效 session 的 color 图片按步长（默认 step=5，每 5 帧取 1 帧）等间隔抽样；
   - 保留原图，拷贝重命名为 d435i_s<session_id>_<timestamp>.png；
   - 在 manifest 中记录对应的 depth_raw 路径与内参文件路径；
   - 归档各 session 的 camera_intrinsics_1280x720.json。
3. iPhone 数据集处理：
   - 仅处理上午、中午、下午三个时间段的文件夹；
   - 完全跳过所有 MOV/MP4 视频文件与 AAE 元数据文件；
   - 树枝树叶负样本（作为 YOLO 黄金比例背景负样本 8%~10%）；
   - 果皮果梗连接特写：针对每一组独立画面（stem），优先使用 HEIC 转换为 JPG，缺失 HEIC 时由 DNG 兜底解码转换，保证每个画面只生成单张高保真 JPG，绝无重复；
   - 使用 macOS 原生硬件加速 sips 工具完成高品质转换。
4. 生成样本清单 manifest.json 与 dataset.yaml 模板。
"""

import os
import sys
import shutil
import subprocess
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

D435I_ROOT = "/Users/bohn/04_Project/05_Orange/MyOrangeDataset/citrus_dataset"
IPHONE_ROOT = "/Users/bohn/04_Project/05_Orange/MyOrangeDataset/iphone数据集"
OUTPUT_ROOT = "/Users/bohn/04_Project/05_Orange/MyOrangeDataset/processed_for_annotation"
D435I_SAMPLE_STEP = 5  # 每 5 帧抽取 1 帧


def convert_image_sips(src_path, dst_path):
    """利用 macOS 原生 sips 工具将 HEIC / DNG / PNG 高品质转换为 JPEG"""
    try:
        cmd = ["sips", "-s", "format", "jpeg", "-s", "formatOptions", "90", str(src_path), "--out", str(dst_path)]
        res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        if res.returncode != 0:
            return False, f"sips error: {res.stderr.strip()}"
        return True, None
    except Exception as e:
        return False, str(e)


def process_iphone_sample(task):
    """处理单张 iPhone 样本转换的任务"""
    src_file, dst_file, sample_info = task
    success, err = convert_image_sips(src_file, dst_file)
    if success:
        return True, sample_info, None
    else:
        return False, sample_info, err


def main():
    print("=" * 60)
    print("🚀 启动柑橘采摘数据集标准化提取任务")
    print(f"  - D435i 来源: {D435I_ROOT}")
    print(f"  - iPhone 来源: {IPHONE_ROOT}")
    print(f"  - 输出目录:   {OUTPUT_ROOT}")
    print(f"  - D435i 采样步长: 每 {D435I_SAMPLE_STEP} 帧抽取 1 帧")
    print("=" * 60)

    # 1. 建立目录骨架
    img_dir = Path(OUTPUT_ROOT) / "images"
    meta_dir = Path(OUTPUT_ROOT) / "metadata"
    intrinsics_dir = meta_dir / "d435i_intrinsics"

    img_dir.mkdir(parents=True, exist_ok=True)
    intrinsics_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "dataset_name": "Citrus_Harvest_DummyArm_Dataset",
        "created_at": "2026-09-16",
        "d435i_step": D435I_SAMPLE_STEP,
        "total_images": 0,
        "d435i_count": 0,
        "iphone_detail_count": 0,
        "iphone_negative_count": 0,
        "samples": []
    }

    # ==========================================
    # 2. 处理 RealSense D435i 数据集
    # ==========================================
    print("\n📦 [1/3] 开始扫描并抽取 D435i 深度相机数据...")
    sessions = sorted([d for d in os.listdir(D435I_ROOT) 
                       if os.path.isdir(os.path.join(D435I_ROOT, d)) 
                       and not d.startswith('.') 
                       and not d.startswith('Wrongsession')])
    
    d435i_extracted = 0
    for s_idx, session_name in enumerate(sessions, start=1):
        s_dir = os.path.join(D435I_ROOT, session_name)
        color_dir = os.path.join(s_dir, "color")
        depth_raw_dir = os.path.join(s_dir, "depth_raw")
        intr_file = os.path.join(s_dir, "camera_intrinsics_1280x720.json")

        if not os.path.exists(color_dir):
            continue

        # 备份内参
        intr_backup_name = f"intrinsics_session_{s_idx:02d}_{session_name}.json"
        if os.path.exists(intr_file):
            shutil.copy2(intr_file, intrinsics_dir / intr_backup_name)

        color_files = sorted([f for f in os.listdir(color_dir) if f.endswith('.png')])
        sampled_files = color_files[::D435I_SAMPLE_STEP]

        print(f"  - Session {s_idx:02d} ({session_name}): 总共 {len(color_files)} 帧 -> 抽样 {len(sampled_files)} 帧")

        for f in sampled_files:
            timestamp = os.path.splitext(f)[0]
            new_img_name = f"d435i_s{s_idx:02d}_{timestamp}.png"
            src_color_path = os.path.join(color_dir, f)
            dst_color_path = img_dir / new_img_name

            raw_depth_file = os.path.join(depth_raw_dir, f)
            has_depth = os.path.exists(raw_depth_file)

            # 如果图片已存在，跳过物理拷贝，直接记账
            if not dst_color_path.exists():
                shutil.copy2(src_color_path, dst_color_path)
            d435i_extracted += 1

            manifest["samples"].append({
                "filename": new_img_name,
                "source": "d435i",
                "session": session_name,
                "session_index": s_idx,
                "type": "positive",
                "has_depth": has_depth,
                "raw_color_path": src_color_path,
                "raw_depth_path": raw_depth_file if has_depth else None,
                "intrinsics_file": intr_backup_name
            })

    manifest["d435i_count"] = d435i_extracted
    print(f"✔ D435i 提取完成，共生成 {d435i_extracted} 张图片并记录深度/内参映射！")

    # ==========================================
    # 3. 规划并准备 iPhone 数据集转换任务
    # ==========================================
    print("\n📱 [2/3] 开始规划 iPhone 数据集转换 (上午/中午/下午)...")
    iphone_tasks = []
    converted_detail = 0
    converted_neg = 0
    failed_count = 0

    # 规范定义各目录及其属性
    iphone_sources = [
        # (时间段标识, 类别, 相对子路径)
        ("am", "detail", "上午/果皮果梗连接"),
        ("noon", "detail", "中午/果皮果梗连接处"),
        ("pm", "detail", "下午/果皮果梗连接处"),
        ("am", "negative", "上午/树枝树叶负样本"),
        ("noon", "negative", "中午/树枝树叶负样本"),
        ("pm", "negative", "下午/树叶枝条负样本"),
    ]

    for time_tag, category, rel_sub in iphone_sources:
        sub_path = os.path.join(IPHONE_ROOT, rel_sub)
        if not os.path.exists(sub_path):
            print(f"  ⚠️ 未找到子目录: {rel_sub}")
            continue

        files = [f for f in os.listdir(sub_path) if not f.startswith('.') and not f.lower().endswith(('.mov', '.mp4', '.aae'))]
        # 按 stem 分组
        grouped = {}
        for f in files:
            stem, ext = os.path.splitext(f)
            grouped.setdefault(stem, {})[ext.lower()] = f

        print(f"  - [{time_tag.upper()}] {category} ({rel_sub}): 共 {len(grouped)} 组独立视觉画面")

        for stem, exts in grouped.items():
            # 确定输入源：优先 HEIC，缺失则用 DNG，若都没有但有 JPG/PNG 则直接用
            chosen_file = None
            if '.heic' in exts:
                chosen_file = exts['.heic']
            elif '.dng' in exts:
                chosen_file = exts['.dng']
            elif '.jpg' in exts:
                chosen_file = exts['.jpg']
            elif '.png' in exts:
                chosen_file = exts['.png']

            if not chosen_file:
                continue

            src_file_path = os.path.join(sub_path, chosen_file)
            out_img_name = f"iphone_{time_tag}_{category}_{stem}.jpg"
            dst_file_path = img_dir / out_img_name

            sample_info = {
                "filename": out_img_name,
                "source": "iphone",
                "time_period": time_tag,
                "category": category,
                "type": "negative" if category == "negative" else "positive",
                "has_depth": False,
                "raw_file_path": src_file_path,
                "raw_source_ext": os.path.splitext(chosen_file)[1].lower()
            }

            # 如果目标文件已存在且非空，跳过转换直接入账
            if dst_file_path.exists() and dst_file_path.stat().st_size > 0:
                manifest["samples"].append(sample_info)
                if category == "negative":
                    converted_neg += 1
                else:
                    converted_detail += 1
                continue

            iphone_tasks.append((src_file_path, dst_file_path, sample_info))

    print(f"  准备并行转换 iPhone 剩余图片: 共 {len(iphone_tasks)} 张...")

    if iphone_tasks:
        with ProcessPoolExecutor(max_workers=8) as executor:
            futures = [executor.submit(process_iphone_sample, task) for task in iphone_tasks]
            for idx, fut in enumerate(as_completed(futures), start=1):
                success, s_info, err = fut.result()
                if success:
                    manifest["samples"].append(s_info)
                    if s_info["category"] == "negative":
                        converted_neg += 1
                    else:
                        converted_detail += 1
                else:
                    failed_count += 1
                    print(f"❌ 转码失败 [{s_info['filename']}]: {err}")

                if idx % 200 == 0 or idx == len(iphone_tasks):
                    print(f"    进度: {idx}/{len(iphone_tasks)} 完成...")

    manifest["iphone_detail_count"] = converted_detail
    manifest["iphone_negative_count"] = converted_neg
    manifest["total_images"] = manifest["d435i_count"] + converted_detail + converted_neg

    print(f"✔ iPhone 转换完成: 特写正样本 {converted_detail} 张，树枝树叶负样本 {converted_neg} 张 (失败: {failed_count})")

    # ==========================================
    # 4. 保存样本索引 manifest.json 与 dataset.yaml
    # ==========================================
    print("\n📝 [3/3] 写入元数据清单与 YOLO 配置文件...")
    manifest_path = meta_dir / "sample_manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"  - 样本索引清单已写入: {manifest_path}")

    dataset_yaml_content = f"""# YOLOv8 / YOLOv11 Instance Segmentation Dataset Config
path: {OUTPUT_ROOT}
train: images/train
val: images/val

# 采摘任务三大核心类别定义
names:
  0: orange           # 必标: 完整可见柑橘果体 (提取三维果心及包围球)
  1: pedicel          # 重点: 柑橘上方1~3cm果梗 (用于确定末端旋切刀具进给法向与切削平面)
  2: occluded_orange  # 辅助: 枝叶遮挡>50%果实 (供机械臂规划器避障或排序决策)
"""
    yaml_path = Path(OUTPUT_ROOT) / "dataset.yaml"
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(dataset_yaml_content)
    print(f"  - YOLO 数据集配置模板已写入: {yaml_path}")

    print("\n" + "=" * 60)
    print("🎉 整个抽取与预处理流程全部圆满完成！")
    print("📊 最终数据集资产汇总：")
    print(f"  - 总图片数:           {manifest['total_images']} 张")
    print(f"  - D435i RGB 深度关联:  {manifest['d435i_count']} 张 (步长={D435I_SAMPLE_STEP})")
    print(f"  - iPhone 果梗果皮特写: {manifest['iphone_detail_count']} 张")
    print(f"  - iPhone 纯背景负样本: {manifest['iphone_negative_count']} 张 (占比约 {manifest['iphone_negative_count'] / manifest['total_images'] * 100:.1f}%)")
    print(f"  - 输出图片目录:       {img_dir}")
    print("=" * 60)


if __name__ == "__main__":
    main()
