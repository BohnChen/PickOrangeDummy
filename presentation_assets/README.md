# presentation_assets · 答辩材料

2026 高通智能边缘大赛答辩 PPT 的**生成器与素材**。最终产物在仓库外：
`/Users/bohn/04_Project/PickOrangeDummy_QualcommEdge_答辩汇报.pptx`。

## 目录

| 路径 | 内容 |
|---|---|
| `build_ppt_v3.py` | **当前版本**（专业工程版，16:9，含演讲备注） |
| `build_ppt.py` | v1 旧版，保留对照 |
| `check_layout.py` / `check_text.py` | 版式与文字溢出检查 |
| `photo_jpg/` | 实车照片（IMG_5588/5589/5590，压缩版） |
| `video_frames/` | 视频抽帧（双相机预览、MoveIt 演示等） |

## 重新生成

```bash
python3 -m pip install python-pptx Pillow   # 依赖
python3 presentation_assets/build_ppt_v3.py
```

脚本里的 `ROOT` 与 `OUT` 是**本机绝对路径**，换机器要改这两行。
配图直接引用 `docs/diagrams/04-tf.png`、`05-mission-fsm.png` 等现有图册产物，
以及本目录下的照片与抽帧——**图片换了要重新跑一次脚本**。

> 注意：PPT 引用的架构图与实拍都来自 2026-11-01 的状态；
> 计算板损坏（2026-11-10）之后，答辩口径以
> [`../docs/ARCHITECTURE_REVIEW_2026-11-10.md`](../docs/ARCHITECTURE_REVIEW_2026-11-10.md) 为准。
