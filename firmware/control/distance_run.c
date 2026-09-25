/***************************************************************************//**
 * @file    distance_run.c
 * @brief   编码器里程闭环直线行驶（支持定距往返）
 ******************************************************************************/

#include "distance_run.h"

#include "heading_hold.h"
#include "speed_control.h"
#include "motor.h"
#include "motor_params.h"

#define DISTANCE_PAUSE_TICKS      (DISTANCE_RUN_PAUSE_MS / DISTANCE_RUN_PERIOD_MS)
#define DISTANCE_CPM_MIN          (1000)
#define DISTANCE_CPM_MAX          (30000)

/* 超时余量：按巡航速度算出的理论 tick 数 ×3 再加 2s 固定余量。
 * 68 = rpm→mm/tick 的定标（rpm/60 × 204.2mm × 0.02s × 1000）。 */
#define DISTANCE_MM_PER_TICK_X1000(rpm)   ((rpm) * 68)
#define DISTANCE_TIMEOUT_MARGIN_TICKS     (2000U / DISTANCE_RUN_PERIOD_MS)

static DistanceRunState_t state;
static DistanceRunFault_t fault;
static uint8_t  round_trip_enabled;
static int8_t   segment_dir;              /* 当前段行进方向 */
static int32_t  cruise_rpm;
static int32_t  base_rpm;                 /* 当前下发的基础速度 */
static int32_t  counts_per_meter;
static int32_t  start_counts;             /* 起点编码器均值（净位移基准） */
static int32_t  goal_mm;                  /* 当前段的目标净位移 */
static int32_t  net_mm;
static int32_t  remaining_mm;
static uint32_t elapsed_ticks;
static uint32_t timeout_ticks;
static uint32_t pause_ticks;

static int32_t DistanceRun_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

/** 双轮编码器均值。驱动层已保证「前进=正计数」，故均值即车体纵向位移。 */
static int32_t DistanceRun_ReadCounts(void)
{
    motor1_encoder_poll();                /* 幂等：与 SpeedControl 的调用不冲突 */
    return (motor1_encoder_accum() + motor2_encoder_accum()) / 2;
}

static int32_t DistanceRun_CountsToMm(int32_t counts)
{
    return (int32_t)(((int64_t)counts * 1000LL) / (int64_t)counts_per_meter);
}

static void DistanceRun_HardStop(void)
{
    HeadingHold_Stop();                   /* 内部会 SpeedControl_Stop（方向复位为前进） */
    motor_brake(MOTOR_1);                 /* 主动刹车而非滑行：到位精度靠它 */
    motor_brake(MOTOR_2);
    base_rpm = 0;
}

static void DistanceRun_Fail(DistanceRunFault_t reason)
{
    DistanceRun_HardStop();
    state = DISTANCE_RUN_STATE_FAULT;
    fault = reason;
}

/** 启动一个行进段：锁当前航向，按 goal 与当前净位移的差值决定方向。 */
static uint8_t DistanceRun_BeginSegment(void)
{
    const int32_t delta_mm = goal_mm - net_mm;

    segment_dir = (delta_mm >= 0) ? 1 : -1;
    remaining_mm = DistanceRun_Abs(delta_mm);
    base_rpm = cruise_rpm;
    elapsed_ticks = 0U;

    timeout_ticks = (uint32_t)(((int64_t)remaining_mm * 1000LL) /
                               (int64_t)DISTANCE_MM_PER_TICK_X1000(cruise_rpm)) * 3U +
                    DISTANCE_TIMEOUT_MARGIN_TICKS;

    if (0U != HeadingHold_StartDrive(base_rpm, segment_dir))
    {
        DistanceRun_Fail(DISTANCE_RUN_FAULT_HEADING);
        return 1U;
    }
    return 0U;
}

void DistanceRun_Init(void)
{
    state = DISTANCE_RUN_STATE_IDLE;
    fault = DISTANCE_RUN_FAULT_NONE;
    round_trip_enabled = 0U;
    segment_dir = 1;
    cruise_rpm = DISTANCE_RUN_DEFAULT_RPM;
    base_rpm = 0;
    counts_per_meter = ENC_COUNTS_PER_METER;
    start_counts = 0;
    goal_mm = 0;
    net_mm = 0;
    remaining_mm = 0;
    elapsed_ticks = 0U;
    timeout_ticks = 0U;
    pause_ticks = 0U;
}

uint8_t DistanceRun_Start(int32_t distance_cm, int32_t new_cruise_rpm, uint8_t round_trip)
{
    const int32_t magnitude_cm = DistanceRun_Abs(distance_cm);

    if ((magnitude_cm < DISTANCE_RUN_MIN_CM) || (magnitude_cm > DISTANCE_RUN_MAX_CM) ||
        (new_cruise_rpm < DISTANCE_RUN_MIN_RPM) || (new_cruise_rpm > DISTANCE_RUN_MAX_RPM))
    {
        return 1U;
    }

    cruise_rpm = new_cruise_rpm;
    round_trip_enabled = (0U != round_trip) ? 1U : 0U;
    fault = DISTANCE_RUN_FAULT_NONE;
    pause_ticks = 0U;

    start_counts = DistanceRun_ReadCounts();
    net_mm = 0;
    goal_mm = distance_cm * 10;

    state = DISTANCE_RUN_STATE_OUTBOUND;
    if (0U != DistanceRun_BeginSegment())
    {
        return 1U;
    }
    return 0U;
}

void DistanceRun_Stop(void)
{
    if ((DISTANCE_RUN_STATE_IDLE != state) && (DISTANCE_RUN_STATE_DONE != state) &&
        (DISTANCE_RUN_STATE_FAULT != state))
    {
        DistanceRun_HardStop();
        state = DISTANCE_RUN_STATE_IDLE;
    }
}

uint8_t DistanceRun_IsActive(void)
{
    return ((DISTANCE_RUN_STATE_OUTBOUND == state) ||
            (DISTANCE_RUN_STATE_PAUSE == state) ||
            (DISTANCE_RUN_STATE_RETURN == state)) ? 1U : 0U;
}

DistanceRunState_t DistanceRun_GetState(void)
{
    return state;
}

DistanceRunFault_t DistanceRun_GetFault(void)
{
    return fault;
}

int32_t DistanceRun_GetNetMm(void)
{
    return net_mm;
}

int32_t DistanceRun_GetRemainingMm(void)
{
    return remaining_mm;
}

int32_t DistanceRun_GetBaseRpm(void)
{
    return base_rpm;
}

uint8_t DistanceRun_SetCountsPerMeter(int32_t new_counts_per_meter)
{
    if ((new_counts_per_meter < DISTANCE_CPM_MIN) ||
        (new_counts_per_meter > DISTANCE_CPM_MAX) ||
        (0U != DistanceRun_IsActive()))
    {
        return 1U;
    }
    counts_per_meter = new_counts_per_meter;
    return 0U;
}

int32_t DistanceRun_GetCountsPerMeter(void)
{
    return counts_per_meter;
}

void DistanceRun_Update(void)
{
    int32_t delta_mm;
    int32_t target_rpm;

    /* 净位移在所有状态下都刷新，PAUSE 段也要看车是否真的停稳。 */
    net_mm = DistanceRun_CountsToMm(DistanceRun_ReadCounts() - start_counts);

    if (DISTANCE_RUN_STATE_PAUSE == state)
    {
        pause_ticks++;
        if (pause_ticks >= DISTANCE_PAUSE_TICKS)
        {
            pause_ticks = 0U;
            goal_mm = 0;              /* 回程目标 = 净位移归零，自动补偿去程误差 */
            state = DISTANCE_RUN_STATE_RETURN;
            (void)DistanceRun_BeginSegment();
        }
        return;
    }

    if (0U == DistanceRun_IsActive())
    {
        return;
    }

    /* 下级故障优先于到位判断：速度环锁存故障后再谈里程没有意义。 */
    if (0U != SpeedControl_GetFaultMask())
    {
        DistanceRun_Fail(DISTANCE_RUN_FAULT_SPEED);
        return;
    }
    if (0U == HeadingHold_IsEnabled())
    {
        DistanceRun_Fail(DISTANCE_RUN_FAULT_HEADING);
        return;
    }

    elapsed_ticks++;
    if (elapsed_ticks >= timeout_ticks)
    {
        DistanceRun_Fail(DISTANCE_RUN_FAULT_TIMEOUT);
        return;
    }

    delta_mm = goal_mm - net_mm;
    remaining_mm = DistanceRun_Abs(delta_mm);

    /* 到位：含冲过头（delta 变号）也判到位，避免来回追打。 */
    if ((remaining_mm <= DISTANCE_RUN_TOLERANCE_MM) ||
        ((delta_mm > 0) != (segment_dir > 0)))
    {
        DistanceRun_HardStop();
        if ((DISTANCE_RUN_STATE_OUTBOUND == state) && (0U != round_trip_enabled))
        {
            pause_ticks = 0U;
            state = DISTANCE_RUN_STATE_PAUSE;
        }
        else
        {
            state = DISTANCE_RUN_STATE_DONE;
        }
        return;
    }

    /* 梯形减速：末段 DECEL_MM 内从巡航线性降到 MIN_RPM。 */
    if (remaining_mm >= DISTANCE_RUN_DECEL_MM)
    {
        target_rpm = cruise_rpm;
    }
    else
    {
        target_rpm = DISTANCE_RUN_MIN_RPM +
                     (((cruise_rpm - DISTANCE_RUN_MIN_RPM) * remaining_mm) /
                      DISTANCE_RUN_DECEL_MM);
    }
    if (target_rpm < DISTANCE_RUN_MIN_RPM)
    {
        target_rpm = DISTANCE_RUN_MIN_RPM;
    }

    if (target_rpm != base_rpm)
    {
        if (0U == HeadingHold_SetBaseRpm(target_rpm))
        {
            base_rpm = target_rpm;
        }
    }
}
