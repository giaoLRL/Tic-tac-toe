#ifndef __TEST_GRAB_H
#define __TEST_GRAB_H

#include "common.h"

/* ================================================================
 *  三子棋机械臂抓取动作测试模块
 *
 *  功能: 在 3×3 棋盘上模拟连续抓取动作
 *        悬停 → 下降 → 抓取 → 提升 → 下一个坐标
 *
 *  触发方式: 上位机发送 #TEST\r\n 启动测试
 *            上位机发送 #TSEQ,0,4,8,2,6\r\n 自定义抓取序列
 *
 *  棋盘坐标系:
 *         y ↑
 *     ┌────┬────┬────┐
 *     │ 6  │ 7  │ 8  │  ← y = +35
 *     ├────┼────┼────┤
 *     │ 3  │ 4  │ 5  │  ← y = 0
 *     ├────┼────┼────┤
 *     │ 0  │ 1  │ 2  │  ← y = -35
 *     └────┴────┴────┘
 *      x=70  x=100 x=130  → x 轴
 * ================================================================ */

/* 外部依赖 (main.c 中定义) */
extern volatile uint8 planner_busy;

/* ---- 抓取阶段 ---- */
typedef enum {
    GRAB_IDLE = 0,        /* 空闲 */
    GRAB_MOVE_HOVER,      /* 移动到目标上方 */
    GRAB_WAIT_HOVER,      /* 等待到达悬停点 */
    GRAB_MOVE_GRAB,       /* 下降到抓取高度 */
    GRAB_WAIT_GRAB,       /* 等待到达抓取点 (模拟抓取) */
    GRAB_MOVE_LIFT,       /* 提升 */
    GRAB_WAIT_LIFT,       /* 等待提升完成 */
    GRAB_DONE             /* 序列完成 */
} GrabState;

/* 坐标点 */
typedef struct {
    float x, y;
    float z_hover;
    float z_grab;
} GrabPoint;

/* ---- 外部接口 ---- */
void TestGrab_Init(void);
void TestGrab_Start(uint8 *seq, uint8 len);
void TestGrab_Tick(void);
void TestGrab_HandleCmd(char *str);

#endif
