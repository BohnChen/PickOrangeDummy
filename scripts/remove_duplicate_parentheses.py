#!/usr/bin/env python3
"""
remove_duplicate_parentheses.py - 安全清除数据集内带有括号编号的转存冗余副本

逻辑：
1. 扫描指定目录下所有文件；
2. 匹配文件名形如 "name(1).ext"、"name(2).ext" 等；
3. 检查同目录下是否存在原文件 "name.ext"；
4. 若原文件存在且文件大小匹配，执行删除；若原文件不存在，则发出警告并跳过，防止误删唯一数据。
"""

import os
import re
import sys

DATASET_ROOT = "/Users/bohn/04_Project/05_Orange/MyOrangeDataset"
PATTERN = re.compile(r"^(.*?)\((\d+)\)(\.[a-zA-Z0-9]+)$")


def main():
    print(f"🔍 开始扫描数据集目录: {DATASET_ROOT}")
    if not os.path.exists(DATASET_ROOT):
        print(f"❌ 目录不存在: {DATASET_ROOT}")
        sys.exit(1)

    matched_files = []
    for root, _, files in os.walk(DATASET_ROOT):
        for f in files:
            m = PATTERN.match(f)
            if m:
                matched_files.append((root, f, m.group(1), m.group(2), m.group(3)))

    print(f"📊 共检测到符合小括号命名特征的文件: {len(matched_files)} 个")

    deleted_count = 0
    skipped_count = 0
    total_bytes_freed = 0

    for root, full_filename, base_name, dup_idx, ext in matched_files:
        dup_path = os.path.join(root, full_filename)
        orig_filename = f"{base_name}{ext}"
        orig_path = os.path.join(root, orig_filename)

        # 检查原件是否存在
        if not os.path.exists(orig_path):
            print(f"⚠️ [跳过保留] 未找到原始正本文件: {orig_filename} (位于 {root})")
            skipped_count += 1
            continue

        dup_size = os.path.getsize(dup_path)
        orig_size = os.path.getsize(orig_path)

        # 安全删除
        try:
            os.remove(dup_path)
            deleted_count += 1
            total_bytes_freed += dup_size
            print(f"🗑️ [已删除] {full_filename} -> 匹配原件: {orig_filename} ({dup_size} 字节)")
        except Exception as e:
            print(f"❌ 删除失败 {dup_path}: {e}")

    print("\n" + "=" * 50)
    print("🎉 清理执行完成！统计报告：")
    print(f"  - 成功删除重复副本: {deleted_count} 个")
    print(f"  - 跳过/保留安全项: {skipped_count} 个")
    print(f"  - 释放存储空间: {total_bytes_freed / (1024 * 1024):.2f} MB")
    print("=" * 50)


if __name__ == "__main__":
    main()
