#ifndef _COMMON_H_
#define _COMMON_H_

/* ========== 基础类型 ========== */
#define int8    signed char
#define int16   signed short int
#define int32   signed int
#define uint8   unsigned char
#define uint16  unsigned short int
#define uint32  unsigned int

/* ========== 舵机数量 ========== */
#define SERVO_NUM   4

/* ========== 舵机 PWM 范围 ========== */
/* 180° 舵机: 500~2500us 对应 0°~180°
   270° 舵机: 500~2500us 对应 0°~270° */
#define SERVO_MIN   500
#define SERVO_MAX   2490
#define SERVO_MID   1500

/* ========== PWM 低电平时间最小值 ========== */
/* 软件 PWM 生成时，低电平时间不能为 0，否则 TIM2 ARR=0 会立即触发中断
   导致时序混乱，舵机无法识别信号 */
#define MIN_LOW_TIME   10

/* ========== PWM 定时参数 ========== */
/* 单路舵机 PWM 周期内的最大计数值 (2.5ms @ 1MHz = 2500 计数) */
#define MAXPWM      2500

/* ========== 插补运动参数 ========== */
#define INTERP_MS   20        /* 插补更新间隔 (ms) */

/* ========== 机械臂连杆长度 (mm, 需实测) ========== */
#define L1          105.0f     /* 大臂长度 10.5cm */
#define L2          95.0f      /* 小臂长度 9.5cm */

/* ========== 弧度/PWM 转换 ========== */
/* 仅保留 IK_RadToPWM() 函数实现, 不提供宏以免不一致
   映射: 0 rad → 1500 (中位), ±π rad → 700~2300
   公式: pwm = 1500 + rad × 254.65 */

/* ========== 通讯协议指令前缀 ========== */
/* 直接 PWM 模式: #PWM,1500,1600,1200\r\n */
/* 世界坐标模式: #POS,120,80,5\r\n     (x,y,z 单位mm) */

#endif
