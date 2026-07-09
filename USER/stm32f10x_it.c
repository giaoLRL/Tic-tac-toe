/**
  * @file    stm32f10x_it.c
  * @brief   中断服务函数
  */

#include "stm32f10x.h"
#include "delay.h"
#include "timer.h"
#include "servor.h"
#include "led.h"

/* 与 main.c 共享, 声明必须匹配 volatile */
extern volatile uint8 flag_planner;

/* TIM4 内部计数器: 10 × 2ms = 20ms 触发一次插补 */
static uint8 tick_count = 0;

/* ========== Cortex-M3 异常处理 ========== */
void NMI_Handler(void) {}
void HardFault_Handler(void)  { while (1); }
void MemManage_Handler(void)  { while (1); }
void BusFault_Handler(void)   { while (1); }
void UsageFault_Handler(void) { while (1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

/* ========== SysTick — 延时递减 ========== */
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
}

/* ========== TIM2 — 舵机 PWM 生成 ========== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
        Servo1();   /* → Flip_GPIO_One() */
        /* 注意: 不再冗余调用 ClearFlag, ClearITPendingBit 已清除 UIF */
    }
}

/* ========== TIM4 — 系统节拍 2ms ========== */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);

        tick_count++;
        if (tick_count >= 10) {       /* 10 × 2ms = 20ms */
            tick_count = 0;
            flag_planner = 1;         /* 触发插补更新 */

            /* LED 心跳: 每 200ms 翻转 (10 × 20ms) */
            static uint8 led_tick = 0;
            led_tick++;
            if (led_tick >= 10) {
                led_tick = 0;
                /* 使用 BSRR 原子操作翻转, 避免与主循环 LED1() 竞态 */
                if (GPIOB->ODR & GPIO_Pin_14)
                    GPIOB->BRR = GPIO_Pin_14;   /* 清 */
                else
                    GPIOB->BSRR = GPIO_Pin_14;  /* 置 */
            }
        }
    }
}
