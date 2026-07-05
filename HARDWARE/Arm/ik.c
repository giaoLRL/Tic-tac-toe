#include "ik.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* 标定偏移 (运行时可通过 IK_SetCalib 修改) */
static IK_Calib calib = {
    DEFAULT_BASE_OFFSET,
    DEFAULT_SHOULDER_OFFSET,
    DEFAULT_ELBOW_OFFSET
};

void IK_SetCalib(uint16 base, uint16 shoulder, uint16 elbow)
{
    calib.base_offset     = base;
    calib.shoulder_offset = shoulder;
    calib.elbow_offset    = elbow;
}

uint16 IK_RadToPWM(float rad)
{
    int32_t pwm;

    /*
     * 线性映射: 0 rad → 1500 (中位), +π rad → 2300, -π rad → 700
     * pwm = 1500 + rad × (800/π) = 1500 + rad × 254.65
     */
    pwm = (int32_t)(1500.0f + rad * 254.65f);

    /* 限幅 */
    if (pwm < SERVO_MIN) pwm = SERVO_MIN;
    if (pwm > SERVO_MAX) pwm = SERVO_MAX;

    return (uint16)pwm;
}

/*
 * 三自由度机械臂逆运动学求解
 *
 * 坐标系（右手系）:
 *   原点 = 底座旋转轴在桌面上的投影
 *   x 轴 = 底座前方
 *   y 轴 = 底座左侧
 *   z 轴 = 垂直桌面向上
 *
 * 关节定义:
 *   θ1 = 底座旋转角 (绕 z 轴), 0 = 正前方
 *   θ2 = 大臂俯仰角 (相对水平面), 0 = 水平, 正 = 向上
 *   θ3 = 小臂俯仰角 (相对大臂延长线), 0 = 伸直, 正 = 向上弯
 */
int IK_Solve(float x, float y, float z,
             uint16 *s1, uint16 *s2, uint16 *s3)
{
    float r, d, z_rel;
    float theta1, theta2, theta3;
    float cos_theta3, sin_theta3;
    float alpha, beta;

    /* ---- 1. 底座旋转角 ---- */
    r = sqrtf(x * x + y * y);
    theta1 = atan2f(y, x);   /* 弧度, 范围 [-π, π] */

    /* ---- 2. 相对肩部坐标 ---- */
    z_rel = z - SHOULDER_HEIGHT;   /* 末端相对肩部的高度 */

    /* 距离校验 */
    d = sqrtf(r * r + z_rel * z_rel);
    if (d > (L1 + L2) * 0.95f) {
        /* 坐标太远, 缩放到可达范围 */
        float scale = (L1 + L2) * 0.95f / d;
        r     *= scale;
        z_rel *= scale;
        d      = (L1 + L2) * 0.95f;
    }

    /* ---- 3. 小臂角 (余弦定理) ---- */
    cos_theta3 = (d * d - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

    /* 数值保护 */
    if (cos_theta3 > 1.0f)  cos_theta3 = 1.0f;
    if (cos_theta3 < -1.0f) cos_theta3 = -1.0f;

    theta3 = acosf(cos_theta3);     /* 肘关节夹角 */
    sin_theta3 = sinf(theta3);

    /* ---- 4. 大臂角 ---- */
    alpha = atan2f(z_rel, r);       /* 目标方向角 */
    beta  = atan2f(L2 * sin_theta3, L1 + L2 * cos_theta3);
    theta2 = alpha - beta;          /* elbow-down 配置 */

	/* ---- 5. 角度 → PWM ---- */
	/*
	 * 底座 (270°舵机, 零位 PWM=2150): PWM = 2150 - theta1 × 424.41
	 * 大臂 (270°舵机, 零位 PWM=1500):   PWM = 1500 - theta2 × 424.41
	 * 小臂 (270°舵机, 垂直 PWM=1167):  PWM = 1167 + (θ3 - π/2) × 424.41
	 *   θ3=90°(垂直) → 1167, θ3=0°(伸直) → 500, θ3=180°(折叠) → 1833
	 */
	#define BASE_SCALE      424.41f    /* 2000 / (270°→rad) */
	#define SHOULDER_SCALE  424.41f
	#define ELBOW_SCALE     424.41f    /* 2000 / (270°→rad) */
	
	*s1 = (uint16)((float)calib.base_offset     - theta1 * BASE_SCALE);
	*s2 = (uint16)((float)calib.shoulder_offset - theta2 * SHOULDER_SCALE);
	*s3 = (uint16)((float)calib.elbow_offset    + (theta3 - 1.57079633f) * ELBOW_SCALE);  /* 以垂直为基准 */

    /* 限幅 */
    if (*s1 < SERVO_MIN) *s1 = SERVO_MIN;
    if (*s1 > SERVO_MAX) *s1 = SERVO_MAX;
    if (*s2 < SERVO_MIN) *s2 = SERVO_MIN;
    if (*s2 > SERVO_MAX) *s2 = SERVO_MAX;
    if (*s3 < SERVO_MIN) *s3 = SERVO_MIN;
    if (*s3 > SERVO_MAX) *s3 = SERVO_MAX;

    return 0;
}
