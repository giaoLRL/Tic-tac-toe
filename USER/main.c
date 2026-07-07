/**
 *  四自由度机械臂控制器（含腕部俯仰补偿）
 *
 *  功能:  接收串口坐标指令 → 逆运动学解算 → 插补移动到目标
 *         腕部舵机由 IK 自动求解, 保持末端吸盘始终垂直向下
 *  协议:
 *    #PWM,1500,1600,1200,1500\r\n  → 直接设置 4 路 PWM (底座/大臂/小臂/腕部)
 *    #POS,120,80,5\r\n             → 世界坐标 (mm), 走 IK (含腕部补偿)
 *    #HOME\r\n                     → 归位
 *    #CAL,1500,1500,1500,1500\r\n  → 标定偏移 (base/shoulder/elbow/wrist)
 *
 *  架构:  插补在主循环中按 20ms 节拍执行, 串口指令在中断中解析
 *         ISR 只设标志不阻塞, 所有 UART 发送在主循环完成
 */

#include <stdlib.h>
#include "stm32f10x.h"
#include "servor.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "led.h"
#include "ik.h"
#include "common.h"
#include "test_grab.h"

/* ========== 外部 ISR 标志 ========== */
volatile uint8 flag_planner = 0;    /* TIM4 ISR 置位, 20ms 周期 */

/* ========== 插补器 ========== */
typedef struct {
    uint16 target[SERVO_NUM + 1];
    float  accum[SERVO_NUM + 1];
    float  inc[SERVO_NUM + 1];
    uint16 total_steps;
    uint16 current_step;
    uint8  running;
} Planner;

static Planner planner;

#define DEFAULT_DURATION_MS  500

/* ========== 前向声明 ========== */
static void Planner_Start(uint16 s1, uint16 s2, uint16 s3, uint16 s4, uint16 duration_ms);
static uint8 Planner_Tick(void);
static void Exec_Cmd(void);

/* ========== 初始化 ========== */
int main(void)
{
    SysTick_Init();
    Servor_GPIO_Config();
    Uart_Init(2);
    Timer_Init();
    Timer_ON();
    LED_Init();
    USART_Config(USART2, 115200);

    Led_Test();

    UART_PutStr(USART2, "Arm Ready (4DOF)\r\n");
    UART_PutStr(USART2, "Commands: #PWM,s1,s2,s3,s4 | #POS,x,y,z | #HOME | #CAL,b,s,e,w\r\n");
    UART_PutStr(USART2, "Test: #TEST | #TSEQ,0,4,8 | #TGRAB,x,y | #TSTOP\r\n");

    /*
     * CPWM 初始值已为 {2150,1500,500,1500} (servor.c 中定义),
     * 上电时舵机保持各零位, 无需额外归位动作。
     * 腕部(CPWM[4]=1500)上电为机械中位, 第一条 #POS 指令后由 IK 自动解算到位。
     */

   

    while (1) {

        /* ---- 响应发送: ISR 只设 resp_ready, 这里真正发 ---- */
        if (resp_ready) {
            resp_ready = 0;
            UART_PutStr(USART2, resp_msg);
        }

        /* ---- 插补节拍: 每 20ms 走一步 ---- */
        if (flag_planner) {
            flag_planner = 0;
            if (planner.running) {
                Planner_Tick();
            }
        }

        /* ---- 抓取测试状态机 ---- */
        TestGrab_Tick();

        /* ---- 处理上位机指令 ---- */
        if (!planner.running && cmd_type != 0) {
            Exec_Cmd();
        }
    }
}

/* ================================================================
 *  插补器
 * ================================================================ */

static void Planner_Start(uint16 s1, uint16 s2, uint16 s3, uint16 s4, uint16 duration_ms)
{
    int i;
    uint16 diff[5], max_diff = 0;
    uint16 target[5];

    target[1] = s1; target[2] = s2; target[3] = s3; target[4] = s4;

    /* 限幅 */
    for (i = 1; i <= SERVO_NUM; i++) {
        if (target[i] < SERVO_MIN) target[i] = SERVO_MIN;
        if (target[i] > SERVO_MAX) target[i] = SERVO_MAX;
    }

    /* 计算总步数 */
    for (i = 1; i <= SERVO_NUM; i++) {
        diff[i] = abs((int)target[i] - (int)CPWM[i]);
        if (diff[i] > max_diff) max_diff = diff[i];
    }

    if (max_diff == 0) {
        planner.running = 1;
        planner_busy = 1;
        __disable_irq();
        for (i = 1; i <= SERVO_NUM; i++) {
            CPWM[i] = target[i];
        }
        __enable_irq();
        planner.running = 0;
        planner_busy = 0;
        UART_PutStr(USART2, "DONE\r\n");
        return;
    }

    if (duration_ms == 0) duration_ms = DEFAULT_DURATION_MS;
    planner.total_steps = duration_ms / INTERP_MS;
    if (planner.total_steps < 1) planner.total_steps = 1;

    for (i = 1; i <= SERVO_NUM; i++) {
        planner.target[i] = target[i];
        planner.inc[i]    = (float)((int)target[i] - (int)CPWM[i])
                          / (float)planner.total_steps;
        planner.accum[i]  = (float)CPWM[i];
    }

    planner.current_step = 0;
    planner.running = 1;
    planner_busy = 1;
}

/*
 * 走一步插补 (主循环每 20ms 调用)
 * CPWM[] 被 TIM2 ISR 读取, 写入时必须关中断保证原子性
 */
static uint8 Planner_Tick(void)
{
    int i;

    planner.current_step++;

    if (planner.current_step >= planner.total_steps) {
        /* 最后一步: 精确到位 (临界区保护) */
        __disable_irq();
        for (i = 1; i <= SERVO_NUM; i++) {
            CPWM[i] = planner.target[i];
        }
        __enable_irq();
        planner.running = 0;
        planner_busy = 0;
        /* DONE 响应: 等当前 resp_ready 被消费后再发, 简单起见直接覆盖 */
        /* 注意: 此处用 UART_PutStr 是安全的, 因为不在 ISR 中 */
        UART_PutStr(USART2, "DONE\r\n");
        return 1;
    }

    /* 累加一步 (临界区保护: 防止 ISR 读到半更新值) */
    __disable_irq();
    for (i = 1; i <= SERVO_NUM; i++) {
        planner.accum[i] += planner.inc[i];
        CPWM[i] = (uint16)planner.accum[i];
    }
    __enable_irq();

    return 0;
}

/* ================================================================
 *  指令执行 (读取 ISR 共享变量时关中断, 防止跨帧混合)
 * ================================================================ */
static void Exec_Cmd(void)
{
    uint8  type;
    float  x, y, z;
    uint16 s1, s2, s3, s4;

    /* 原子快照: 关中断一次性拷贝所有共享变量 */
    __disable_irq();
    type = cmd_type;
    x = target_x; y = target_y; z = target_z;
    s1 = target_s1; s2 = target_s2; s3 = target_s3; s4 = target_s4;
    cmd_type = 0;
    __enable_irq();

    switch (type) {

    case 1: /* #PWM,s1,s2,s3,s4 */
        Planner_Start(s1, s2, s3, s4, DEFAULT_DURATION_MS);
        break;

    case 2: /* #POS,x,y,z */
        {
            uint16 ik_s1, ik_s2, ik_s3, ik_s4;
            int ret = IK_Solve(x, y, z, &ik_s1, &ik_s2, &ik_s3, &ik_s4);
            if (ret == 0) {
                Planner_Start(ik_s1, ik_s2, ik_s3, ik_s4, DEFAULT_DURATION_MS);
            } else {
                UART_PutStr(USART2, "ERR IK FAIL\r\n");
            }
        }
        break;

    case 3: /* #CAL,b,s,e,w */
        IK_SetCalib(s1, s2, s3, s4);
        break;

    case 4: /* #HOME */
        Planner_Start(SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID, 1000);
        break;

    default:
        break;
    }
}
