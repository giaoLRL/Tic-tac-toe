#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"
#include "common.h"

void Uart_Init(uint16 uart_num);
void USART_Config(USART_TypeDef *TUSARTx, uint32 bound);
void UART_PutChar(USART_TypeDef *USARTx, uint8_t Data);
void UART_PutStr (USART_TypeDef *USARTx, char *str);

/* ---- ISR 与主循环共享变量 (全部 volatile, 读时关中断) ---- */
extern volatile uint8  cmd_type;
extern volatile uint16 target_s1, target_s2, target_s3, target_s4;
extern volatile float  target_x, target_y, target_z;

/* 响应机制: ISR 只设标志, 主循环负责发送 (避免 ISR 中阻塞轮询) */
extern volatile uint8  resp_ready;
extern          char   resp_msg[32];

/* 兼容旧代码 */
extern volatile uint8 flag_RecFul;

#endif
