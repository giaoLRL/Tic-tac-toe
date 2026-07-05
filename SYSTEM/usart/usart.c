#include "sys.h"
#include "usart.h"
#include "led.h"
#include <string.h>
#include <stdlib.h>

/* ========== 全局变量 (ISR 共享, 全部 volatile) ========== */
volatile uint8  flag_RecFul = 0;
volatile uint8  cmd_type    = 0;
volatile uint16 target_s1, target_s2, target_s3;
volatile float  target_x, target_y, target_z;

/* 响应机制: ISR 只设标志和消息, 主循环负责 UART_PutStr */
volatile uint8  resp_ready = 0;
char            resp_msg[32] = {0};

/* 接收缓冲区 */
static volatile uint8 recv_idx = 0;
static          char  recv_buf[64];

/* ========== printf 重定向 ========== */
#if 1
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;

void _sys_exit(int x) { (void)x; }

int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0x40) == 0);
    USART1->DR = (uint8)ch;
    return ch;
}
#endif

/* ========== NVIC ========== */
static void Uart1_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ========== GPIO ========== */
static void Uart1_Gpio_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* ========== 初始化入口 ========== */
void Uart_Init(uint16 uart_num)
{
    if (uart_num == 1) {
        Uart1_NVIC_Init();
        Uart1_Gpio_Config();
    }
}

void USART_Config(USART_TypeDef *TUSARTx, uint32 bound)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(TUSARTx, &USART_InitStructure);
    USART_ITConfig(TUSARTx, USART_IT_RXNE, ENABLE);
    USART_Cmd(TUSARTx, ENABLE);
}

/* ========== 串口发送 ========== */
void UART_PutChar(USART_TypeDef *USARTx, uint8_t Data)
{
    USART_SendData(USARTx, Data);
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

void UART_PutStr(USART_TypeDef *USARTx, char *str)
{
    while (*str != '\0') {
        UART_PutChar(USARTx, *str++);
    }
}

/* ========== 辅助: 安全拷贝响应消息 ========== */
static void SetResp(const char *msg)
{
    uint8 i = 0;
    while (msg[i] && i < sizeof(resp_msg) - 1) {
        resp_msg[i] = msg[i];
        i++;
    }
    resp_msg[i] = '\0';
    resp_ready = 1;
}

/* ========== 协议解析 (中断上下文中调用, 不阻塞) ========== */
static void ParseCmd(char *str)
{
    int a, b, c;
    int n;

    /* 去除末尾 \r\n */
    char *p = strchr(str, '\r');
    if (p) *p = 0;
    p = strchr(str, '\n');
    if (p) *p = 0;

    /* --- #PWM,s1,s2,s3 --- */
    if (strncmp(str, "#PWM,", 5) == 0) {
        n = sscanf(str, "#PWM,%d,%d,%d", &a, &b, &c);
        if (n != 3) {
            SetResp("ERR FORMAT\r\n");
            return;
        }
        /* 范围检查: 防止负数或溢出值损坏舵机 */
        if (a < 500 || a > 2500 || b < 500 || b > 2500 ||
            c < 500 || c > 2500) {
            SetResp("ERR RANGE\r\n");
            return;
        }
        target_s1 = (uint16)a;
        target_s2 = (uint16)b;
        target_s3 = (uint16)c;
        cmd_type = 1;
        SetResp("OK PWM\r\n");
        return;
    }

    /* --- #POS,x,y,z --- */
    if (strncmp(str, "#POS,", 5) == 0) {
        n = sscanf(str, "#POS,%f,%f,%f", &target_x, &target_y, &target_z);
        if (n == 3) {
            cmd_type = 2;
            SetResp("OK POS\r\n");
        } else {
            SetResp("ERR FORMAT\r\n");
        }
        return;
    }

    /* --- #CAL,off1,off2,off3 --- */
    if (strncmp(str, "#CAL,", 5) == 0) {
        n = sscanf(str, "#CAL,%d,%d,%d", &a, &b, &c);
        if (n != 3) {
            SetResp("ERR FORMAT\r\n");
            return;
        }
        /* 范围检查 */
        if (a < 500 || a > 2500 || b < 500 || b > 2500 ||
            c < 500 || c > 2500) {
            SetResp("ERR RANGE\r\n");
            return;
        }
        target_s1 = (uint16)a;
        target_s2 = (uint16)b;
        target_s3 = (uint16)c;
        cmd_type = 3;
        SetResp("OK CAL\r\n");
        return;
    }

    /* --- #HOME --- */
    if (strncmp(str, "#HOME", 5) == 0) {
        cmd_type = 4;
        SetResp("OK HOME\r\n");
        return;
    }

    /* 未知指令 */
    SetResp("ERR UNKNOWN\r\n");
}

/* ========== USART1 中断接收 ========== */
void USART1_IRQHandler(void)
{
    uint8 ch;

    /* 先处理正常接收, 再处理溢出 (避免误读有效数据) */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        ch = USART_ReceiveData(USART1);

        if (ch == '\n' || recv_idx >= 63) {
            recv_buf[recv_idx] = '\0';
            recv_idx = 0;
            if (recv_buf[0] == '#') {
                ParseCmd(recv_buf);
            }
        } else {
            recv_buf[recv_idx++] = ch;
        }
    }

    /* ORE: 必须先读 SR 再读 DR 才能清除 (参考手册 RM0008 §25.3.5) */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) {
        (void)USART_ReceiveData(USART1);  /* 读 DR 完成 ORE 清除序列 */
    }
}

/* ========== 外部接口 (保留兼容) ========== */
void DealRec(void)
{
    /* 指令在中断中解析，此处占位 */
}
