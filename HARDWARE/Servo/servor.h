#ifndef __SERVOR_H
#define __SERVOR_H

#include "stm32f10x.h"
#include "common.h"

/* ISR 与主循环共享, 必须 volatile */
extern volatile uint16 CPWM[SERVO_NUM + 1];
extern volatile uint8  count1;

void Servo1(void);
void Servor_GPIO_Config(void);

#endif
