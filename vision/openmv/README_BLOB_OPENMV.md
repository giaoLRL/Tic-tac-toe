# OpenMV最大黑线连通区域识别版

使用文件：

- `main_blob.py`
- `vision_core_rgb.py`

此版本不再使用 `find_rects()`，也不要求同时检测大框和多个小框。它仿照原始 `sanqi.py`：

```text
黑色网格阈值
→ find_blobs()寻找连通区域
→ 选择外接面积最大的网格色块
→ min_corners()取得旋转外框
→ 数学划分3×3
→ 读取九个内接圆RGB平均值
```

## 实时跟踪与取色优化

- 锁定后每一帧都重新检测网格，不再每隔10帧更新；
- 小范围角点抖动采用平滑，大范围移动立即跟随；
- 当前帧找不到可靠网格时显示 `FRAME MISS`，暂停RGB采样；
- 锁定后先显示 `CAMERA SETTLE` 约20帧，让曝光、增益和白平衡适应白色棋盘；
- 出现 `CAMERA_FROZEN` 后才正式计算九个内接圆RGB；
- 采样半径为格子短边的28%，减少网格线进入采样区；
- 每格最近5帧RGB分别取中位数，画面显示滤波值；
- 终端同时输出 `RAW_RGB` 和 `FILTERED_RGB`，分类使用后者。

## OpenMV IDE测试

1. 把最新版 `vision_core_rgb.py` 保存到OpenMV设备根目录。
2. 在电脑中打开 `main_blob.py`并点击运行。
3. 云台保持不动，将打印棋盘完整放入画面。
4. 搜索画面显示：
   - 青色框：检测到但未通过棋盘筛选的黑色连通区域；
   - 紫色框：面积、长宽比和密度通过筛选的棋盘候选；
   - 黄色粗框：当前最大的棋盘候选；
   - `BLOBS n CAND n`：黑色连通区域数和棋盘候选数；
   - `Axx Dxx`：该区域占画面面积百分比及黑色像素密度百分比。
5. 候选连续稳定6帧后进入 `GRID LOCKED`。

测试通过后，可将 `main_blob.py` 保存到设备根目录并改名为 `main.py`，实现上电运行。

## 黑线阈值

默认值：

```python
BLACK_GRID_THRESHOLD = (0, 50, -25, 25, -25, 25)
```

如果显示 `BLOBS 0 CAND 0`，使用OpenMV IDE阈值编辑器选择打印的黑线区域，并替换该LAB阈值。也可先逐步提高亮度上限：

```python
(0, 60, -30, 30, -30, 30)
(0, 70, -35, 35, -35, 35)
```

上限过高会把阴影和灰色背景一起识别。

## 有BLOBS但没有CAND

查看画面中的 `Axx Dxx`：

- `A`小于8：棋盘太远，或修改 `min_area_ratio`；
- `A`大于80：棋盘太近，或检测到了整个背景；
- `D`大于55：区域过于实心，通常不是网格；
- 外框太扁：放宽 `min_side_ratio/max_side_ratio`。

可调参数：

```python
BOARD_LIMITS = {
    "min_area_ratio": 0.08,
    "max_area_ratio": 0.80,
    "min_side_ratio": 0.50,
    "max_side_ratio": 1.80,
    "min_density": 0.01,
    "max_density": 0.55,
}
```

## 网格线断开

如果一张棋盘被识别成多个BLOB：

- 调焦并避免过曝；
- 确认打印线完整；
- 将 `BLOB_MERGE_MARGIN` 从4提高到6或8；
- 放宽黑线阈值。

## 棋子RGB阈值

锁定后终端会输出 `RGB_STATS`。黑棋使用G/B范围，白棋使用R/G范围；根据现场数据修改 `RGB_THRESHOLDS`。默认值不能代替真实相机标定。
