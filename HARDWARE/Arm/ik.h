#ifndef _IK_H_
#define _IK_H_

#include "common.h"

/*
 * 三自由度机械臂逆运动学
 *
 * 几何模型:
 *         z ↑
 *           │
 *    shoulder●──── L1(大臂) ────● elbow
 *           │                    \
 *           │                     L2(小臂)
 *           │                       \
 *    ───────┼────────────────────────● end-effector (x,y,z)
 *           │ base
 *   ←────────────────── r = sqrt(x²+y²) ──────→
 *
 *  俯视图:
 *         y ↑
 *           │
 *           │    target(x,y)
 *           │    /
 *           │   /
 *           │  / θ1 = atan2(y,x)
 *           │ /
 *     base  ●──────────→ x
 */

/* 舵机角度零位偏移 (需根据实际装配标定)
   单位: PWM值, 1500 = 90°中位 */
typedef struct {
    uint16 base_offset;     /* 底座舵机零位 */
    uint16 shoulder_offset; /* 大臂舵机零位 */
    uint16 elbow_offset;    /* 小臂舵机零位 */
} IK_Calib;

/* 默认标定值 (实物标定)
   底座 270°舵机, 零位 → PWM=2150
   大臂 270°舵机, 零位 → PWM=1500
   小臂 270°舵机, 零位 → PWM=500  */
#define DEFAULT_BASE_OFFSET      2150
#define DEFAULT_SHOULDER_OFFSET  1500
#define DEFAULT_ELBOW_OFFSET     500

/* 肩部距桌面高度 (mm) */
#define SHOULDER_HEIGHT          65.0f

/*
 * 世界坐标 → 3个舵机 PWM 值
 * 输入:  x, y, z  末端执行器的世界坐标 (mm)
 * 输出:  s1, s2, s3  三个舵机的 PWM 值
 * 返回:  0=成功, -1=坐标不可达
 */
int IK_Solve(float x, float y, float z,
             uint16 *s1, uint16 *s2, uint16 *s3);

/*
 * 设置标定偏移
 */
void IK_SetCalib(uint16 base, uint16 shoulder, uint16 elbow);

/*
 * 弧度 → PWM 转换 (0=700, π=2300, 线性)
 */
uint16 IK_RadToPWM(float rad);

#endif
