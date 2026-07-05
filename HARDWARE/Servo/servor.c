#include "servor.h"
#include "common.h"

/*
 *  3 路舵机 PWM 生成（软件 GPIO 时分复用）
 *
 *  时序设计:
 *    TIM2 时钟 = 72MHz / 72 = 1MHz (1us精度)
 *    单路舵机: 高电平 CPWM[n] + 低电平 (MAXPWM - CPWM[n]) = MAXPWM = 2500us
 *    3 路舵机: 3 × MAXPWM = 7500us
 *    目标周期: 20000us (50Hz 标准舵机频率)
 *    padding:  20000 - 7500 = 12500us  ← 补足到 20ms
 *    总周期 = 7500 + 12500 = 20000us = 20ms ✓
 *
 *  GPIO 分配:
 *    Servo 1 (底座旋转): PB15, CPWM[1]
 *    Servo 2 (大臂):     PA8,  CPWM[2]
 *    Servo 3 (小臂):     PB5,  CPWM[3]
 *
 *  ⚠ volatile 必须: ISR 读取 CPWM[], 主循环写入, 编译器不得优化
 */

volatile uint8  count1;
volatile uint16 CPWM[SERVO_NUM + 1] = {0, 2150, 1500, 500};

/* GPIO 初始化: PB15(舵机1), PA8(舵机2), PB5(舵机3) */
void Servor_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PA8 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PB5, PB15 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
 * GPIO 电平翻转状态机（TIM2 中断中调用）
 *
 * count1 在 Servo1() 中递增后进入本函数
 * 每调用一次走一个 case，控制一个 GPIO 的翻转
 *
 * Case 1-6: 3路舵机 × (高+低) = 6次翻转
 * Case 7:   padding，补足到 ~20ms 总周期
 */
void Flip_GPIO_One(void)
{
    switch (count1) {

    /* --- Servo 1: PB15 --- */
    case 1:
        TIM2->ARR = CPWM[1];
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        break;
    case 2:
        TIM2->ARR = MAXPWM - CPWM[1];
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        break;

    /* --- Servo 2: PA8 --- */
    case 3:
        TIM2->ARR = CPWM[2];
        GPIO_SetBits(GPIOA, GPIO_Pin_8);
        break;
    case 4:
        TIM2->ARR = MAXPWM - CPWM[2];
        GPIO_ResetBits(GPIOA, GPIO_Pin_8);
        break;

    /* --- Servo 3: PB5 --- */
    case 5:
        TIM2->ARR = CPWM[3];
        GPIO_SetBits(GPIOB, GPIO_Pin_5);
        break;
    case 6:
        TIM2->ARR = MAXPWM - CPWM[3];
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);
        break;

    /* --- Padding: 补足到 20ms 周期 --- */
    /* 3 × MAXPWM = 7500us, 需要再加 12500us 凑满 20000us = 20ms */
    case 7:
        TIM2->ARR = 12500;
        count1 = 0;         /* 周期结束，重置计数器 */
        break;

    /* 异常恢复: 若 count1 意外越界，写入安全值并立即恢复 */
    default:
        TIM2->ARR = MAXPWM;
        count1 = 0;
        break;
    }
}

/* TIM2 中断服务函数中调用 */
void Servo1(void)
{
    count1++;
    Flip_GPIO_One();
}
