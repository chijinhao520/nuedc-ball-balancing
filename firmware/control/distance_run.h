/***************************************************************************//**
 * @file    distance_run.h
 * @brief   编码器里程闭环直线行驶（支持定距往返）
 *
 * 控制链：编码器里程 -> 剩余距离 -> 梯形减速基础速度 -> HeadingHold -> SpeedControl。
 *
 * 职责切分（与 heading_hold 不重叠）：
 *   - 本模块只管**纵向**：走多远、当前该多快、何时停；
 *   - 横向（左右轮速差 / 航向修正）仍由 heading_hold 负责。
 *   两者不会同时写 SpeedControl —— 本模块只通过 HeadingHold_SetBaseRpm 调基础速度。
 *
 * 往返回原点的判据是**净位移**而非「再走一遍标称距离」：回程盯着相对起点的
 * 净位移收敛到 0，因此去程多走/少走的误差会在回程被自动补偿掉。
 ******************************************************************************/

#ifndef ALGORITHM_DISTANCE_RUN_H
#define ALGORITHM_DISTANCE_RUN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DISTANCE_RUN_PERIOD_MS        (20U)
#define DISTANCE_RUN_MIN_CM           (10)      /* 短于此放不下减速段，无意义 */
#define DISTANCE_RUN_MAX_CM           (500)     /* 单程上限 5m，防打错一位数冲出去 */
#define DISTANCE_RUN_DEFAULT_RPM      (60)      /* ≈0.20 m/s，与 H 题带球一圈速度同量级 */
/* 启动下限必须对齐 HEADING_BASE_MIN_RPM(40)——HeadingHold_StartDrive 走 StartCommon，
 * 用的是 BASE 而非 DRIVE 门限（heading_hold.c:95 vs :192）。原值 30 抄成了
 * HEADING_DRIVE_MIN_RPM，导致 'm <cm> 30..39' 被 StartDrive 拒绝、却报成
 * 误导性的 FAULT_HEADING（2026-07-29 实测复现）。末段减速走 SetBaseRpm，
 * 仍可降到 HEADING_DRIVE_MIN_RPM(30)，不受本上调影响。 */
#define DISTANCE_RUN_MIN_RPM          (40)      /* = HEADING_BASE_MIN_RPM（启动门限） */
#define DISTANCE_RUN_MAX_RPM          (150)
#define DISTANCE_RUN_TOLERANCE_MM     (5)       /* 到位容差；题面②停车偏差要求 ≤2cm */
#define DISTANCE_RUN_DECEL_MM         (150)     /* 末段 15cm 线性减速到 MIN_RPM */
#define DISTANCE_RUN_PAUSE_MS         (800U)    /* 段间静止窗：等车停稳再读净位移 */

typedef enum
{
    DISTANCE_RUN_STATE_IDLE = 0,
    DISTANCE_RUN_STATE_OUTBOUND,      /* 去程 */
    DISTANCE_RUN_STATE_PAUSE,         /* 段间静止等待 */
    DISTANCE_RUN_STATE_RETURN,        /* 回程（净位移收敛到 0） */
    DISTANCE_RUN_STATE_DONE,
    DISTANCE_RUN_STATE_FAULT
} DistanceRunState_t;

typedef enum
{
    DISTANCE_RUN_FAULT_NONE = 0,
    DISTANCE_RUN_FAULT_SPEED,         /* 速度环故障（堵转/编码器） */
    DISTANCE_RUN_FAULT_HEADING,       /* 航向环掉线（IMU 非有限值 / yaw 超速） */
    DISTANCE_RUN_FAULT_TIMEOUT        /* 本段超时 */
} DistanceRunFault_t;

void DistanceRun_Init(void);

/**
 * 启动定距直线行驶。
 *
 * @param distance_cm 去程距离，带符号：正=前进，负=倒车起步；|值| 须在 10..500
 * @param cruise_rpm  巡航速度 30..150 rpm
 * @param round_trip  1=到位后倒车回原点（净位移归零），0=单程
 * @return 0=已启动，1=参数非法或下级拒绝
 */
uint8_t DistanceRun_Start(int32_t distance_cm, int32_t cruise_rpm, uint8_t round_trip);

/** 立即中止并停车（刹车）。 */
void DistanceRun_Stop(void);

void DistanceRun_Update(void);

uint8_t DistanceRun_IsActive(void);
DistanceRunState_t DistanceRun_GetState(void);
DistanceRunFault_t DistanceRun_GetFault(void);

/** 相对起点的净位移（mm，带符号，前进为正）。往返结束后它就是回原点误差。 */
int32_t DistanceRun_GetNetMm(void);
/** 当前段到目标还差多少（mm，非负）。 */
int32_t DistanceRun_GetRemainingMm(void);
/** 当前下发的基础速度（rpm）。 */
int32_t DistanceRun_GetBaseRpm(void);

/**
 * 里程标定系数（counts/米），默认取 motor_params.h 的 ENC_COUNTS_PER_METER。
 * 实测反推流程见 motor_params.h 标定说明。范围 1000..30000。
 */
uint8_t DistanceRun_SetCountsPerMeter(int32_t counts_per_meter);
int32_t DistanceRun_GetCountsPerMeter(void);

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_DISTANCE_RUN_H */
