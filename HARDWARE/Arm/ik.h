#ifndef _IK_H_
#define _IK_H_

#include "common.h"

/*
 * 四自由度机械臂逆运动学（含腕部俯仰补偿）
 *
 * 几何模型:
 *         z ↑
 *           │
 *    shoulder●──── L1(大臂) ────● elbow
 *           │                    \
 *           │                     L2(小臂)
 *           │                       \
 *           │                        ● wrist ── 末端吸盘(垂直向下)
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
 *
 *  腕关节 θ4 由 IK 自动求解, 补偿 θ2+θ3 使末端吸盘始终垂直向下:
 *     末端(小臂)方向角 = θ2 + θ3
 *     要求末端方向 = -π/2 (垂直向下)
 *     θ4 = -π/2 - (θ2 + θ3)
 */

/* 舵机角度零位偏移 (需根据实际装配标定)
   单位: PWM值, 1500 = 90°中位 */
typedef struct {
    uint16 base_offset;     /* 底座舵机零位 */
    uint16 shoulder_offset; /* 大臂舵机零位 */
    uint16 elbow_offset;    /* 小臂舵机零位 */
    uint16 wrist_offset;    /* 腕部舵机零位 (θ4=0 即腕与小臂共线时 PWM) */
} IK_Calib;

/* 默认标定值 (实物标定, 2024实测)
 *
 *  舵机类型:
 *    底座 270°, 大臂 270°, 小臂 180°, 腕部 180°
 *
 *  零位定义 (各关节几何角度为0时的PWM值):
 *    底座: θ1=0(正前方) → PWM=1500
 *    大臂: θ2=90°(竖直向上) → PWM=1400
 *    小臂: θ3=0(完全伸直) → PWM=1900
 *    腕部: θ4=0(与小臂共线, 吸盘朝上) → PWM=1600
 *
 *  已知问题: 大臂PWM变化率对应scale≈254.65(800/π),
 *           而非标准270°舵机scale=424.41, 原因待查 */
#define DEFAULT_BASE_OFFSET      1500
#define DEFAULT_SHOULDER_OFFSET  1400
#define DEFAULT_ELBOW_OFFSET     1900
#define DEFAULT_WRIST_OFFSET     1600

/* 肩部距桌面高度 (mm) */
#define SHOULDER_HEIGHT          65.0f

/*
 * 世界坐标 → 4个舵机 PWM 值
 * 输入:  x, y, z  末端执行器的世界坐标 (mm)
 * 输出:  s1, s2, s3, s4  四个舵机的 PWM 值 (底座/大臂/小臂/腕部)
 *        s4 由 θ4 = -π/2 - (θ2+θ3) 自动求解, 使末端吸盘垂直向下
 * 返回:  0=成功, -1=坐标不可达
 */
int IK_Solve(float x, float y, float z,
             uint16 *s1, uint16 *s2, uint16 *s3, uint16 *s4);

/*
 * 设置标定偏移 (base/shoulder/elbow/wrist 四路)
 */
void IK_SetCalib(uint16 base, uint16 shoulder, uint16 elbow, uint16 wrist);

/*
 * 弧度 → PWM 转换 (0=700, π=2300, 线性)
 * 注意: 此函数为旧版3DOF遗留, 4DOF路径不使用, 保留仅供兼容
 */
uint16 IK_RadToPWM(float rad);

#endif
