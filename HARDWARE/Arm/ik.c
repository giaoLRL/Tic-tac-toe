#include "ik.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* 标定偏移 (运行时可通过 IK_SetCalib 修改) */
static IK_Calib calib = {
    DEFAULT_BASE_OFFSET,
    DEFAULT_SHOULDER_OFFSET,
    DEFAULT_ELBOW_OFFSET,
    DEFAULT_WRIST_OFFSET
};

void IK_SetCalib(uint16 base, uint16 shoulder, uint16 elbow, uint16 wrist)
{
    calib.base_offset     = base;
    calib.shoulder_offset = shoulder;
    calib.elbow_offset    = elbow;
    calib.wrist_offset    = wrist;
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
 * 四自由度机械臂逆运动学求解（含腕部俯仰补偿）
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
 *   θ4 = 腕部俯仰角 (相对小臂), 0 = 与小臂共线
 *        由 IK 自动求解: θ4 = -π/2 - (θ2+θ3), 使末端吸盘垂直向下
 */
int IK_Solve(float x, float y, float z,
             uint16 *s1, uint16 *s2, uint16 *s3, uint16 *s4)
{
    float r, d, z_rel;
    float theta1, theta2, theta3, theta4;
    float cos_theta3, sin_theta3;
    float alpha, beta;

    /* ---- 1. 底座旋转角 ---- */
    r = sqrtf(x * x + y * y);
    theta1 = atan2f(y, x);   /* 弧度, 范围 [-π, π] */

    /* ---- 2. 相对肩部坐标 ---- */
    z_rel = z - SHOULDER_HEIGHT;   /* 末端相对肩部的高度 */

    /* 距离校验 */
    d = sqrtf(r * r + z_rel * z_rel);
    if (d > (L1 + L2) * 1) {
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

	/* ---- 5. 腕部俯仰角 (保持末端吸盘垂直向下) ---- */
	/* θ4=0(PWM=零位)时吸盘与小臂共线朝上, θ4=-π/2时⊥小臂朝下 */
	/* TODO: θ4公式产生的角度远超180°舵机范围, 腕部公式待修正 */
	theta4 = -(theta2 + theta3) - 1.57079633f;

	/* ---- 6. 角度 → PWM ---- */
	/*
	 *  底座 (270°舵机, 零位 θ1=0→PWM=1500):
	 *    PWM = base_offset + θ1 × 424.41
	 *
	 *  大臂 (270°舵机, 零位 θ2=90°竖直→PWM=500):
	 *    PWM = shoulder_offset + (θ2 - π/2) × 254.65
	 *    注: offset改为500, 公式和比例需重新标定
	 *
	 *  小臂 (180°舵机, 零位 θ3=0伸直→PWM=1900):
	 *    PWM = elbow_offset - θ3 × 636.62
	 *    θ3=0(伸直)→1900, θ3增大→PWM减小
	 *
	 *  腕部 (180°舵机, 零位 θ4=0共线→PWM=1600):
	 *    PWM = wrist_offset + θ4 × 636.62
	 *    θ4=0(与小臂共线)→1600
	 */
	#define BASE_SCALE      424.41f    /* 2000 / (270°→rad) = 4000/(3π) */
	#define SHOULDER_SCALE  254.65f    /* 800/π, IK_RadToPWM比例 */
	#define ELBOW_SCALE     636.62f    /* 2000 / (180°→rad) = 2000/π */
	#define WRIST_SCALE     636.62f    /* 2000 / (180°→rad) = 2000/π */

	*s1 = (uint16)((float)calib.base_offset     + theta1 * BASE_SCALE);
	*s2 = (uint16)((float)calib.shoulder_offset + theta2 * SHOULDER_SCALE);
	*s3 = (uint16)((float)calib.elbow_offset    - theta3 * ELBOW_SCALE);
	*s4 = (uint16)((float)calib.wrist_offset    + theta4 * WRIST_SCALE);

    /* 限幅 */
    if (*s1 < SERVO_MIN) *s1 = SERVO_MIN;
    if (*s1 > SERVO_MAX) *s1 = SERVO_MAX;
    if (*s2 < SERVO_MIN) *s2 = SERVO_MIN;
    if (*s2 > SERVO_MAX) *s2 = SERVO_MAX;
    if (*s3 < SERVO_MIN) *s3 = SERVO_MIN;
    if (*s3 > SERVO_MAX) *s3 = SERVO_MAX;
    if (*s4 < SERVO_MIN) *s4 = SERVO_MIN;
    if (*s4 > SERVO_MAX) *s4 = SERVO_MAX;

    return 0;
}
