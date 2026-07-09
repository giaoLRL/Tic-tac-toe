# Tic-Tac-Toe Robot

三子棋（井字棋）机器人项目，包含视觉识别与机械臂控制两部分。

## 项目结构

```
├── mechanical_arm/       # 机械臂控制部分
│   └── grbl/             # GRBL 固件（CNC/机械臂运动控制）
├── vision/               # 视觉识别部分
│   ├── k230_canmv/       # K230 CanMV 版本（当前主力）
│   │   ├── main_blob.py         # 主程序（背景差分 + 自适应直方图均衡）
│   │   └── vision_core_rgb.py   # 视觉核心库
│   └── openmv/           # OpenMV 版本（原始）
│       ├── main_blob.py         # 主程序（LAB 颜色阈值识别）
│       ├── vision_core_rgb.py   # 视觉核心库
│       └── README_BLOB_OPENMV.md
└── README.md
```

## 视觉识别

### K230 CanMV 版本（推荐）

基于 K230 CanMV 开发板（k230_canmv_yahboom，ST7701 显示屏，GC2093 摄像头）。

**算法特点：**
- **背景差分**：锁定棋盘后自动拍摄空棋盘作为参考底片，逐格 RGB 比较
- **自适应直方图均衡**：灰度图 + `histeq(adaptive=True)` + 二值化，解决局部高光/反光问题
- **时域中值滤波**：5 帧窗口 RGB 中值滤波，抑制传感器噪声
- **棋盘去抖动**：连续 5 帧稳定才输出结果
- **参考自动更新**：连续 30 帧全空时 EMA 缓慢更新参考底片

**硬件要求：**
- K230 CanMV Yahboom 开发板
- ST7701 显示屏
- GC2093 摄像头
- 橙色棋盘背景

### OpenMV 版本（原始版本）

基于 OpenMV 摄像头，使用 LAB 颜色空间阈值识别棋盘和棋子。

详见 `vision/openmv/README_BLOB_OPENMV.md`

## 机械臂控制

使用 GRBL 固件进行 CNC/机械臂运动控制，实现自动落子。
