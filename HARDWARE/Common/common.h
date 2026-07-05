#ifndef _COMMON_H_
#define _COMMON_H_

/* ========== 基础类型 ========== */
#define uint8   unsigned char
#define uint32  unsigned int
#define uint16  unsigned short int

/* ========== 舵机数量 ========== */
#define SERVO_NUM   3

/* ========== 舵机 PWM 范围 ========== */
/* 标准舵机: 500~2500us 对应 0°~180°, 1500us 为中位 90° */
#define SERVO_MIN   700
#define SERVO_MAX   2300
#define SERVO_MID   1500

/* ========== PWM 定时参数 ========== */
/* 单路舵机 PWM 周期内的最大计数值 (2.5ms @ 1MHz = 2500 计数) */
#define MAXPWM      2500

/* ========== 插补运动参数 ========== */
#define INTERP_MS   20        /* 插补更新间隔 (ms) */

/* ========== 机械臂连杆长度 (mm, 需实测) ========== */
#define L1          80.0f     /* 大臂长度 */
#define L2          70.0f     /* 小臂长度 */

/* ========== 弧度/PWM 转换 ========== */
/* 仅保留 IK_RadToPWM() 函数实现, 不提供宏以免不一致
   映射: 0 rad → 1500 (中位), ±π rad → 700~2300
   公式: pwm = 1500 + rad × 254.65 */

/* ========== 通讯协议指令前缀 ========== */
/* 直接 PWM 模式: #PWM,1500,1600,1200\r\n */
/* 世界坐标模式: #POS,120,80,5\r\n     (x,y,z 单位mm) */

#endif
