/***************************************************************************//**
 * @file    heading_hold.c
 * @brief   基于双轮速度 PI 的航向保持与相对转角控制
 ******************************************************************************/

#include "heading_hold.h"

#include "attitude.h"
#include "speed_control.h"
#include "motor_params.h"       /* YAW_DIFF_SIGN：M1 在车头的左侧还是右侧 */

/* 角度外环：航向误差 -> 目标 yaw 角速度。 */
#define HEADING_ANGLE_KP_DPS_PER_DEG       (1.0f)
#define HEADING_RATE_LIMIT_DPS             (40.0f)

/* yaw 速率环：前馈负责给出基本轮速差，反馈修正实测角速度误差。 */
#define HEADING_YAW_FF_RPM_PER_DPS         (0.45f)
#define HEADING_YAW_FB_RPM_PER_DPS         (0.15f)
#define HEADING_DIFF_LIMIT_RPM             (40.0f)
#define HEADING_WHEEL_MIN_RPM              (30.0f)

#define HEADING_TURN_SETTLE_ERROR_DEG      (2.0f)
#define HEADING_TURN_SETTLE_RATE_DPS       (5.0f)
#define HEADING_OVERSPEED_LIMIT_DPS         (60.0f)
#define HEADING_TURN_SETTLE_TICKS          (200U / HEADING_UPDATE_PERIOD_MS)
#define HEADING_TURN_TIMEOUT_TICKS         (3000U / HEADING_UPDATE_PERIOD_MS)
#define HEADING_TELEMETRY_LIMIT_DEG        (1000000.0f)

static uint8_t enabled;
static HeadingHoldMode_t mode;
static HeadingHoldResult_t result;
static int8_t drive_dir;                  /* +1 前进 / -1 后退；仅 HOLD 模式有意义 */
static int32_t base_rpm10;
static float target_yaw;
static uint16_t elapsed_ticks;
static uint8_t settle_ticks;
static int32_t last_error10;
static int16_t last_target_rate10;
static int16_t last_diff_rpm10;

static float HeadingHold_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float HeadingHold_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t HeadingHold_IsFinite(float value)
{
    return ((value == value) &&
            (value <= HEADING_TELEMETRY_LIMIT_DEG) &&
            (value >= -HEADING_TELEMETRY_LIMIT_DEG)) ? 1U : 0U;
}

static int32_t HeadingHold_DegTo10(float value)
{
    value = HeadingHold_Clamp(value,
                              -HEADING_TELEMETRY_LIMIT_DEG,
                              HEADING_TELEMETRY_LIMIT_DEG);
    return (int32_t)(value * 10.0f);
}

static void HeadingHold_StopWithResult(HeadingHoldResult_t stop_result)
{
    if (0U != enabled)
    {
        SpeedControl_Stop();
    }
    enabled = 0U;
    mode = HEADING_HOLD_MODE_OFF;
    drive_dir = 1;                        /* 与 SpeedControl_Stop 的方向复位保持一致 */
    result = stop_result;
    elapsed_ticks = 0U;
    settle_ticks = 0U;
}

static uint8_t HeadingHold_StartCommon(HeadingHoldMode_t new_mode,
                                       float new_target_yaw,
                                       int32_t new_command_rpm,
                                       int8_t new_dir)
{
    if ((0U == HeadingHold_IsFinite(new_target_yaw)) ||
        ((HEADING_HOLD_MODE_HOLD == new_mode) &&
         ((new_command_rpm < HEADING_BASE_MIN_RPM) ||
          (new_command_rpm > HEADING_BASE_MAX_RPM))) ||
        ((HEADING_HOLD_MODE_TURN == new_mode) &&
         ((new_command_rpm < HEADING_TURN_OUTER_MIN_RPM) ||
          (new_command_rpm > HEADING_TURN_OUTER_MAX_RPM))))
    {
        return 1U;
    }

    if (0U != enabled)
    {
        HeadingHold_StopWithResult(HEADING_HOLD_RESULT_STOPPED);
    }
    else
    {
        SpeedControl_Stop();
    }

    base_rpm10 = new_command_rpm * 10;
    target_yaw = new_target_yaw;
    elapsed_ticks = 0U;
    settle_ticks = 0U;
    last_error10 = 0;
    last_target_rate10 = 0;
    last_diff_rpm10 = 0;
    mode = new_mode;
    drive_dir = (HEADING_HOLD_MODE_TURN == new_mode) ? 1 : new_dir;
    result = HEADING_HOLD_RESULT_ACTIVE;

    /* 必须在上面的 SpeedControl_Stop() 之后设置——Stop 会把方向复位为 +1。 */
    if (0U != SpeedControl_SetDirection(drive_dir))
    {
        mode = HEADING_HOLD_MODE_OFF;
        drive_dir = 1;
        result = HEADING_HOLD_RESULT_SPEED_FAULT;
        return 1U;
    }

    if (HEADING_HOLD_MODE_TURN == new_mode)
    {
        const uint8_t positive_turn = (new_target_yaw > attitude_yaw_deg) ? 1U : 0U;
        SpeedControl_Start((0U != positive_turn) ? base_rpm10 : 0,
                           (0U != positive_turn) ? 0 : base_rpm10);
    }
    else
    {
        SpeedControl_Start(base_rpm10, base_rpm10);
    }
    if (0U == SpeedControl_IsEnabled())
    {
        mode = HEADING_HOLD_MODE_OFF;
        result = HEADING_HOLD_RESULT_SPEED_FAULT;
        return 1U;
    }

    enabled = 1U;
    return 0U;
}

void HeadingHold_Init(void)
{
    enabled = 0U;
    mode = HEADING_HOLD_MODE_OFF;
    drive_dir = 1;
    result = HEADING_HOLD_RESULT_IDLE;
    base_rpm10 = 0;
    target_yaw = 0.0f;
    elapsed_ticks = 0U;
    settle_ticks = 0U;
    last_error10 = 0;
    last_target_rate10 = 0;
    last_diff_rpm10 = 0;
}

uint8_t HeadingHold_Start(int32_t new_base_rpm)
{
    return HeadingHold_StartCommon(HEADING_HOLD_MODE_HOLD,
                                   attitude_yaw_deg,
                                   new_base_rpm,
                                   1);
}

uint8_t HeadingHold_StartDrive(int32_t new_base_rpm, int8_t new_dir)
{
    if ((new_dir != 1) && (new_dir != -1))
    {
        return 1U;
    }
    return HeadingHold_StartCommon(HEADING_HOLD_MODE_HOLD,
                                   attitude_yaw_deg,
                                   new_base_rpm,
                                   new_dir);
}

uint8_t HeadingHold_SetBaseRpm(int32_t new_base_rpm)
{
    if ((0U == enabled) || (HEADING_HOLD_MODE_HOLD != mode) ||
        (new_base_rpm < HEADING_DRIVE_MIN_RPM) ||
        (new_base_rpm > HEADING_BASE_MAX_RPM))
    {
        return 1U;
    }
    base_rpm10 = new_base_rpm * 10;
    return 0U;
}

int8_t HeadingHold_GetDirection(void)
{
    return drive_dir;
}

uint8_t HeadingHold_StartTurn(float delta_yaw_deg, int32_t outer_max_rpm)
{
    if ((delta_yaw_deg < -HEADING_TURN_MAX_ANGLE_DEG) ||
        (delta_yaw_deg > HEADING_TURN_MAX_ANGLE_DEG) ||
        (HeadingHold_Abs(delta_yaw_deg) < 0.5f))
    {
        return 1U;
    }

    return HeadingHold_StartCommon(HEADING_HOLD_MODE_TURN,
                                   attitude_yaw_deg + delta_yaw_deg,
                                   outer_max_rpm,
                                   1);
}

void HeadingHold_Stop(void)
{
    HeadingHold_StopWithResult(HEADING_HOLD_RESULT_STOPPED);
    base_rpm10 = 0;
    target_yaw = 0.0f;
    last_error10 = 0;
    last_target_rate10 = 0;
    last_diff_rpm10 = 0;
}

uint8_t HeadingHold_IsEnabled(void)
{
    return enabled;
}

HeadingHoldMode_t HeadingHold_GetMode(void)
{
    return mode;
}

HeadingHoldResult_t HeadingHold_GetResult(void)
{
    return result;
}

int32_t HeadingHold_GetTargetYaw10(void)
{
    return HeadingHold_DegTo10(target_yaw);
}

int32_t HeadingHold_GetLastError10(void)
{
    return last_error10;
}

int16_t HeadingHold_GetLastTargetRate10(void)
{
    return last_target_rate10;
}

int16_t HeadingHold_GetLastDiffRpm10(void)
{
    return last_diff_rpm10;
}

void HeadingHold_Update(void)
{
    float error_deg;
    float target_rate_dps;
    float rate_error_dps;
    float diff_rpm;
    float dynamic_diff_limit_rpm;
    int32_t motor1_target_rpm10;
    int32_t motor2_target_rpm10;

    if (0U == enabled)
    {
        return;
    }

    if ((0U != SpeedControl_GetFaultMask()) ||
        ((HEADING_HOLD_MODE_HOLD == mode) && (0U == SpeedControl_IsEnabled())))
    {
        HeadingHold_StopWithResult(HEADING_HOLD_RESULT_SPEED_FAULT);
        return;
    }

    if ((0U == HeadingHold_IsFinite(attitude_yaw_deg)) ||
        (0U == HeadingHold_IsFinite(attitude_gyro_z_dps)) ||
        (0U == HeadingHold_IsFinite(target_yaw)))
    {
        HeadingHold_StopWithResult(HEADING_HOLD_RESULT_IMU_INVALID);
        return;
    }

    /* attitude_yaw_deg 是多圈展开角，目标与当前可直接相减。 */
    error_deg = target_yaw - attitude_yaw_deg;
    if (HeadingHold_Abs(attitude_gyro_z_dps) > HEADING_OVERSPEED_LIMIT_DPS)
    {
        HeadingHold_StopWithResult(HEADING_HOLD_RESULT_OVERSPEED);
        return;
    }
    if ((HEADING_HOLD_MODE_TURN == mode) &&
        (HeadingHold_Abs(error_deg) <= HEADING_TURN_SETTLE_ERROR_DEG))
    {
        target_rate_dps = 0.0f;
    }
    else
    {
        target_rate_dps = HeadingHold_Clamp(HEADING_ANGLE_KP_DPS_PER_DEG * error_deg,
                                            -HEADING_RATE_LIMIT_DPS,
                                            HEADING_RATE_LIMIT_DPS);
    }
    rate_error_dps = target_rate_dps - attitude_gyro_z_dps;

    /* M1=右轮、M2=左轮：正轮速差(M1快)产生正 yaw，07-22 实板已核定。 */
    diff_rpm = HEADING_YAW_FF_RPM_PER_DPS * target_rate_dps +
               HEADING_YAW_FB_RPM_PER_DPS * rate_error_dps;
    if (HEADING_HOLD_MODE_TURN == mode)
    {
        float outer_rpm = 0.0f;

        dynamic_diff_limit_rpm = (float)base_rpm10 * 0.05f;
        diff_rpm = HeadingHold_Clamp(diff_rpm,
                                     -dynamic_diff_limit_rpm,
                                     dynamic_diff_limit_rpm);

        /* 短距测试：只驱动转弯外侧轮，车体绕静止内轮转，避免向前画长弧撞墙。 */
        if (((target_rate_dps > 0.0f) && (diff_rpm > 0.0f)) ||
            ((target_rate_dps < 0.0f) && (diff_rpm < 0.0f)))
        {
            outer_rpm = HeadingHold_Abs(diff_rpm) * 2.0f;
            outer_rpm = HeadingHold_Clamp(outer_rpm,
                                          (float)HEADING_TURN_OUTER_MIN_RPM,
                                          (float)base_rpm10 * 0.1f);
        }

        /* 正 yaw = 左转，须驱动**右侧**轮绕左轮转；右侧轮是 M1 还是 M2 由 YAW_DIFF_SIGN 决定。 */
        if (target_rate_dps > 0.0f)
        {
            motor1_target_rpm10 = (YAW_DIFF_SIGN > 0) ? (int32_t)(outer_rpm * 10.0f) : 0;
            motor2_target_rpm10 = (YAW_DIFF_SIGN > 0) ? 0 : (int32_t)(outer_rpm * 10.0f);
        }
        else if (target_rate_dps < 0.0f)
        {
            motor1_target_rpm10 = (YAW_DIFF_SIGN > 0) ? 0 : (int32_t)(outer_rpm * 10.0f);
            motor2_target_rpm10 = (YAW_DIFF_SIGN > 0) ? (int32_t)(outer_rpm * 10.0f) : 0;
        }
        else
        {
            motor1_target_rpm10 = 0;
            motor2_target_rpm10 = 0;
        }
    }
    else
    {
        dynamic_diff_limit_rpm = ((float)base_rpm10 * 0.1f) - HEADING_WHEEL_MIN_RPM;
        if (dynamic_diff_limit_rpm > HEADING_DIFF_LIMIT_RPM)
        {
            dynamic_diff_limit_rpm = HEADING_DIFF_LIMIT_RPM;
        }
        /* base 减到 30rpm 时限幅恰为 0；再低会变负而使 Clamp(min>max) 错乱，兜底钳零。 */
        if (dynamic_diff_limit_rpm < 0.0f)
        {
            dynamic_diff_limit_rpm = 0.0f;
        }
        diff_rpm = HeadingHold_Clamp(diff_rpm,
                                     -dynamic_diff_limit_rpm,
                                     dynamic_diff_limit_rpm);
        /* 两个符号缺一不可，否则航向环变正反馈：
         *  drive_dir     —— 倒车时两轮线速度都取负，(v_右−v_左) 整体反号；
         *  YAW_DIFF_SIGN —— M1 究竟在右侧还是左侧（07-29 车头翻转后 M1 已变左轮）。 */
        motor1_target_rpm10 = base_rpm10 +
                              (int32_t)(diff_rpm * 10.0f) * (int32_t)drive_dir * YAW_DIFF_SIGN;
        motor2_target_rpm10 = base_rpm10 -
                              (int32_t)(diff_rpm * 10.0f) * (int32_t)drive_dir * YAW_DIFF_SIGN;
    }

    if ((0 == motor1_target_rpm10) && (0 == motor2_target_rpm10))
    {
        SpeedControl_Stop();
    }
    else
    {
        SpeedControl_Start(motor1_target_rpm10, motor2_target_rpm10);
        if (0U == SpeedControl_IsEnabled())
        {
            HeadingHold_StopWithResult(HEADING_HOLD_RESULT_SPEED_FAULT);
            return;
        }
    }

    last_error10 = HeadingHold_DegTo10(error_deg);
    last_target_rate10 = (int16_t)(target_rate_dps * 10.0f);
    /* 乘回 drive_dir 与 YAW_DIFF_SIGN 还原成物理轮速差，
     * 使遥测语义恒为「正 = 右轮快 = 左转需求」，与前进/倒车、M1 在左/在右都无关。 */
    last_diff_rpm10 = (int16_t)(((motor1_target_rpm10 - motor2_target_rpm10) / 2) *
                                (int32_t)drive_dir * YAW_DIFF_SIGN);

    if (HEADING_HOLD_MODE_TURN == mode)
    {
        elapsed_ticks++;
        if ((HeadingHold_Abs(error_deg) <= HEADING_TURN_SETTLE_ERROR_DEG) &&
            (HeadingHold_Abs(attitude_gyro_z_dps) <= HEADING_TURN_SETTLE_RATE_DPS))
        {
            if (settle_ticks < HEADING_TURN_SETTLE_TICKS)
            {
                settle_ticks++;
            }
            if (settle_ticks >= HEADING_TURN_SETTLE_TICKS)
            {
                HeadingHold_StopWithResult(HEADING_HOLD_RESULT_SETTLED);
                return;
            }
        }
        else
        {
            settle_ticks = 0U;
        }

        if (elapsed_ticks >= HEADING_TURN_TIMEOUT_TICKS)
        {
            HeadingHold_StopWithResult(HEADING_HOLD_RESULT_TIMEOUT);
        }
    }
}
