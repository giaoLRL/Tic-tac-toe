#ifndef __TEST_GRAB_H
#define __TEST_GRAB_H

#include "common.h"

/* ================================================================
 *  多点巡回测试模块
 *
 *  功能: 按序列依次移动到 9 个坐标点, 每点停留 2 秒
 *
 *  触发方式: 上位机发送 #TEST\r\n 启动测试
 *            上位机发送 #TSEQ,0,4,8,2,6\r\n 自定义序列
 * ================================================================ */

/* 外部依赖 (main.c 中定义) */
extern volatile uint8 planner_busy;

/* ---- 测试状态 ---- */
typedef enum {
    TEST_IDLE = 0,        /* 空闲 */
    TEST_MOVE,            /* 移动中 */
    TEST_WAIT,            /* 到达后停留 */
    TEST_DONE             /* 序列完成 */
} TestState;

/* 坐标点 */
typedef struct {
    float x, y, z;
} TestPoint;

/* ---- 外部接口 ---- */
void TestGrab_Init(void);
void TestGrab_Start(uint8 *seq, uint8 len);
void TestGrab_Tick(void);
void TestGrab_HandleCmd(char *str);

#endif
