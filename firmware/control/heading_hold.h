/***************************************************************************//**
 * @file    heading_hold.h
 * @brief   基于双轮速度 PI 的航向保持与相对转角控制
 *
 * 控制链：yaw 误差 -> 目标 yaw 角速度 -> 左右轮 RPM 差 -> SpeedControl。
 * Algorithm 层只复用姿态与速度 PI 接口，不直接写 PWM 或 BSP。
 ******************************************************************************/

#ifndef HEADING_HOLD_H
#define HEADING_HOLD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEADING_UPDATE_PERIOD_MS       (20U)
#define HEADING_BASE_MIN_RPM           (40)
#define HEADING_BASE_MAX_RPM           (200)
/* 运行中允许被减速到的下限（里程闭环末段减速用）。
 * 取 30 与 HEADING_WHEEL_MIN_RPM 对齐，有两个硬理由：
 *  ① 轮速差限幅算式 (base − 30) 在 base<30 时变负，会让 Clamp(min>max) 逻辑错乱；
 *  ② 速度环堵转检测的下界也是 30rpm，再低就落进「转不动也不报错」的盲区。
 * 末段不是靠继续降速停下，而是到位即停（见 distance_run 的到位判据）。 */
#define HEADING_DRIVE_MIN_RPM          (30)
#define HEADING_TURN_OUTER_MIN_RPM     (30)
#define HEADING_TURN_OUTER_MAX_RPM     (80)
#define HEADING_DEFAULT_TURN_RPM       (40)
#define HEADING_TURN_MAX_ANGLE_DEG     (90.0f)

typedef enum
{
    HEADING_HOLD_MODE_OFF = 0,
    HEADING_HOLD_MODE_HOLD,
    HEADING_HOLD_MODE_TURN
} HeadingHoldMode_t;

typedef enum
{
    HEADING_HOLD_RESULT_IDLE = 0,
    HEADING_HOLD_RESULT_ACTIVE,
    HEADING_HOLD_RESULT_SETTLED,
    HEADING_HOLD_RESULT_TIMEOUT,
    HEADING_HOLD_RESULT_SPEED_FAULT,
    HEADING_HOLD_RESULT_OVERSPEED,
    HEADING_HOLD_RESULT_IMU_INVALID,
    HEADING_HOLD_RESULT_STOPPED
} HeadingHoldResult_t;

void HeadingHold_Init(void);

/** 锁定当前航向并以前进基础速度持续保持，成功返回 0。 */
uint8_t HeadingHold_Start(int32_t base_rpm);

/**
 * 锁定当前航向并按指定方向直行（里程闭环 distance_run 用），成功返回 0。
 *
 * dir=+1 前进 / -1 后退。倒车时左右轮速差对 yaw 的作用**反号**——
 * 车体角速度正比于 (v_右 − v_左)，后退时两轮线速度都取负，故
 * m1 = base + diff·dir、m2 = base − diff·dir，否则航向环变正反馈。
 */
uint8_t HeadingHold_StartDrive(int32_t base_rpm, int8_t dir);

/** 运行中调整基础速度（末段减速用），HEADING_DRIVE_MIN_RPM..HEADING_BASE_MAX_RPM，成功返回 0。 */
uint8_t HeadingHold_SetBaseRpm(int32_t base_rpm);

/** 当前行进方向：+1 前进 / -1 后退。 */
int8_t HeadingHold_GetDirection(void);

/** 单外轮执行短距相对转角；outer_max_rpm 为减速曲线的外轮上限，成功返回 0。 */
uint8_t HeadingHold_StartTurn(float delta_yaw_deg, int32_t outer_max_rpm);

/** 关闭航向控制；若本模块正在运行，同时停止其下级速度 PI。 */
void HeadingHold_Stop(void);

void HeadingHold_Update(void);

uint8_t HeadingHold_IsEnabled(void);
HeadingHoldMode_t HeadingHold_GetMode(void);
HeadingHoldResult_t HeadingHold_GetResult(void);
int32_t HeadingHold_GetTargetYaw10(void);
int32_t HeadingHold_GetLastError10(void);
int16_t HeadingHold_GetLastTargetRate10(void);
int16_t HeadingHold_GetLastDiffRpm10(void);

#ifdef __cplusplus
}
#endif

#endif /* HEADING_HOLD_H */
