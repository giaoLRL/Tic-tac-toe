/**
 *  三子棋机械臂抓取动作测试
 *
 *  抓取流程:
 *    1. 悬停到目标上方 (z_hover)
 *    2. 下降到抓取高度 (z_grab)
 *    3. 等待 500ms 模拟抓取
 *    4. 提升回悬停高度
 *    5. 移动到下一个目标点
 *
 *  通信协议:
 *    通过设置 cmd_type/target_x/y/z 与主循环 Exec_Cmd() 交互
 */

#include "test_grab.h"
#include "usart.h"
#include "delay.h"
#include <string.h>
#include <stdlib.h>

/* 由 main.c 维护: planner_busy = 1 表示正在插补运动 */
volatile uint8 planner_busy = 0;

/* ================================================================
 *  棋盘坐标预设 (9 个格子)
 *  索引对应棋盘位置:
 *    0  1  2     (y = -35)
 *    3  4  5     (y = 0)
 *    6  7  8     (y = +35)
 *  x: 70 / 100 / 130
 * ================================================================ */
static const GrabPoint board[9] = {
    /* idx 0 */ { 70.0f, -35.0f, 40.0f, 3.0f },
    /* idx 1 */ {100.0f, -35.0f, 40.0f, 3.0f },
    /* idx 2 */ {130.0f, -35.0f, 40.0f, 3.0f },
    /* idx 3 */ { 70.0f,   0.0f, 40.0f, 3.0f },
    /* idx 4 */ {100.0f,   0.0f, 40.0f, 3.0f },   /* 中心 */
    /* idx 5 */ {130.0f,   0.0f, 40.0f, 3.0f },
    /* idx 6 */ { 70.0f,  35.0f, 40.0f, 3.0f },
    /* idx 7 */ {100.0f,  35.0f, 40.0f, 3.0f },
    /* idx 8 */ {130.0f,  35.0f, 40.0f, 3.0f },
};

/* ================================================================
 *  测试序列配置
 *  默认抓取序列: 中心 → 四角 → 四边中点 (共 9 步)
 *  可通过 #TSEQ,idx1,idx2,... 自定义
 * ================================================================ */
static uint8 grab_seq[9] = {4, 0, 2, 6, 8, 1, 3, 5, 7};
static uint8 seq_len = 9;
static uint8 seq_idx = 0;

/* 状态机变量 */
static GrabState g_state = GRAB_IDLE;
static uint8      g_active = 0;       /* 测试是否激活 */
static uint32     g_pause_ms = 0;     /* 抓取停留计时 */
static GrabPoint  g_current;          /* 当前目标点 */

/* ---- 延时 (主循环节拍计数, 每 20ms 一次 Tick) ---- */
#define TICK_MS  20

/* ================================================================
 *  内部函数: 发送世界坐标移动指令
 *  通过原子设置共享变量, 由主循环 Exec_Cmd() 执行 IK+插补
 * ================================================================ */
static void SendPosCmd(float x, float y, float z)
{
    /* 原子写入: 关中断防止 ISR 与主循环的竞态 */
    __disable_irq();
    target_x = x;
    target_y = y;
    target_z = z;
    cmd_type = 2;   /* #POS 指令类型 */
    __enable_irq();
}

/* ================================================================
 *  TestGrab_Init: 初始化测试模块
 * ================================================================ */
void TestGrab_Init(void)
{
    g_state   = GRAB_IDLE;
    g_active  = 0;
    seq_idx   = 0;
    g_pause_ms = 0;
}

/* ================================================================
 *  TestGrab_Start: 启动抓取测试
 *  seq: 棋盘索引数组 (NULL 则使用默认全序列)
 *  len: 数组长度
 * ================================================================ */
void TestGrab_Start(uint8 *seq, uint8 len)
{
    if (seq != 0 && len > 0 && len <= 9) {
        uint8 i;
        for (i = 0; i < len; i++) {
            grab_seq[i] = seq[i];
        }
        seq_len = len;
    }
    /* else: 保持默认序列 */

    seq_idx   = 0;
    g_active  = 1;
    g_state   = GRAB_MOVE_HOVER;

    /* 加载第一个目标 */
    g_current = board[grab_seq[0]];

    /* 发送悬停指令 */
    SendPosCmd(g_current.x, g_current.y, g_current.z_hover);

    UART_PutStr(USART1, "TEST START\r\n");
}

/* ================================================================
 *  TestGrab_Tick: 主循环节拍 (每 20ms 调用一次)
 *  与 planner_busy 协同, 管理抓取状态机
 * ================================================================ */
void TestGrab_Tick(void)
{
    if (!g_active) return;

    switch (g_state) {

    /* ---- 阶段 1: 移动到悬停点, 等待 planner 完成 ---- */
    case GRAB_MOVE_HOVER:
        if (!planner_busy && cmd_type == 0) {
            g_state = GRAB_WAIT_HOVER;
        }
        break;

    case GRAB_WAIT_HOVER:
        /* 稳定片刻后下降 */
        g_state = GRAB_MOVE_GRAB;
        SendPosCmd(g_current.x, g_current.y, g_current.z_grab);
        break;

    /* ---- 阶段 2: 下降到抓取高度 ---- */
    case GRAB_MOVE_GRAB:
        if (!planner_busy && cmd_type == 0) {
            g_state = GRAB_WAIT_GRAB;
            g_pause_ms = 0;  /* 开始计时 */
        }
        break;

    case GRAB_WAIT_GRAB:
        /* 停留 500ms 模拟抓取动作 */
        g_pause_ms += TICK_MS;
        if (g_pause_ms >= 500) {
            g_state = GRAB_MOVE_LIFT;
            SendPosCmd(g_current.x, g_current.y, g_current.z_hover);
        }
        break;

    /* ---- 阶段 3: 提升回悬停高度 ---- */
    case GRAB_MOVE_LIFT:
        if (!planner_busy && cmd_type == 0) {
            g_state = GRAB_WAIT_LIFT;
            /* 打印当前完成坐标 */
            {
                char buf[32];
                sprintf(buf, "OK GRAB %d,%d,%d\r\n",
                        (int)g_current.x,
                        (int)g_current.y,
                        (int)g_current.z_grab);
                UART_PutStr(USART1, buf);
            }
        }
        break;

    case GRAB_WAIT_LIFT:
        /* 移动到下一个目标, 或结束 */
        seq_idx++;
        if (seq_idx >= seq_len) {
            g_state  = GRAB_DONE;
            g_active = 0;
            UART_PutStr(USART1, "TEST DONE\r\n");
        } else {
            g_current = board[grab_seq[seq_idx]];
            g_state = GRAB_MOVE_HOVER;
            SendPosCmd(g_current.x, g_current.y, g_current.z_hover);
        }
        break;

    case GRAB_DONE:
    case GRAB_IDLE:
    default:
        break;
    }
}

/* ================================================================
 *  TestGrab_HandleCmd: 解析测试相关指令 (由 usart.c ISR 调用)
 *
 *  支持的测试指令:
 *    #TEST            → 运行默认 9 点全抓取序列
 *    #TSEQ,0,4,8      → 自定义抓取序列 (棋盘索引, 逗号分隔)
 *    #TGRAB,x,y       → 单次抓取指定坐标
 *    #TSTOP           → 停止测试
 * ================================================================ */
void TestGrab_HandleCmd(char *str)
{
    int idx_list[9];
    int n, i;

    /* 去除 \r\n */
    char *p = strchr(str, '\r'); if (p) *p = 0;
    p = strchr(str, '\n');       if (p) *p = 0;

    /* ---- #TEST: 默认全序列 ---- */
    if (strncmp(str, "#TEST", 5) == 0 && (str[5] == '\0' || str[5] == '\r' || str[5] == '\n')) {
        TestGrab_Start(0, 0);
        return;
    }

    /* ---- #TSEQ,idx1,idx2,... ---- */
    if (strncmp(str, "#TSEQ,", 6) == 0) {
        /* 解析逗号分隔的索引 */
        n = 0;
        char *tok = strtok(str + 6, ",");
        while (tok && n < 9) {
            idx_list[n] = atoi(tok);
            if (idx_list[n] < 0) idx_list[n] = 0;
            if (idx_list[n] > 8) idx_list[n] = 8;
            n++;
            tok = strtok(0, ",");
        }
        if (n > 0) {
            TestGrab_Start((uint8 *)idx_list, (uint8)n);
        }
        return;
    }

    /* ---- #TGRAB,x,y (单次抓取指定棋盘坐标) ---- */
    if (strncmp(str, "#TGRAB,", 7) == 0) {
        int x_mm = 100, y_mm = 0;
        sscanf(str, "#TGRAB,%d,%d", &x_mm, &y_mm);

        /* 找最近的棋盘格子 */
        int best = 0;
        float best_dist = 99999.0f;
        for (i = 0; i < 9; i++) {
            float dx = board[i].x - (float)x_mm;
            float dy = board[i].y - (float)y_mm;
            float dist = dx * dx + dy * dy;
            if (dist < best_dist) {
                best_dist = dist;
                best = i;
            }
        }
        /* 单点抓取 */
        uint8 single = (uint8)best;
        TestGrab_Start(&single, 1);
        return;
    }

    /* ---- #TSTOP: 停止测试 ---- */
    if (strncmp(str, "#TSTOP", 6) == 0) {
        g_active = 0;
        g_state  = GRAB_IDLE;
        UART_PutStr(USART1, "TEST STOP\r\n");
        return;
    }
}
