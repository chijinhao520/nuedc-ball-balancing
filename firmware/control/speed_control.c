#include "speed_control.h"

#include "motor.h"
#include "motor_params.h"

#define SPEED_FILTER_LENGTH       (3U)
#define SPEED_KP_X100_DEFAULT     (200)
#define SPEED_KI_X100_DEFAULT     (1600)
#define SPEED_KP_X100_MAX         (2000)
#define SPEED_KI_X100_MAX         (5000)
#define SPEED_INTEGRAL_MAX        ((float)SPEED_CONTROL_OUTPUT_MAX)
#define SPEED_STALL_TARGET_MIN_RPM10  (300)
#define SPEED_STALL_MEASURED_MAX_RPM10 (50)
#define SPEED_STALL_OUTPUT_MIN        (300)
#define SPEED_STALL_TIMEOUT_TICKS     (500U / SPEED_CONTROL_PERIOD_MS)

static uint8_t enabled;
static int8_t  direction;                 /* +1=前进 / -1=后退；只在测速入口与 PWM 出口翻符号 */
static uint8_t fault_mask;
static uint8_t hard_fault_mask;
static int32_t kp_x100;
static int32_t ki_x100;
static int32_t target_rpm10[2];
static int32_t measured_rpm10[2];
static int32_t output_duty[2];
static float integral_duty[2];
static int32_t encoder_previous[2];
static int32_t delta_buffer[2][SPEED_FILTER_LENGTH];
static int32_t delta_sum[2];
static uint8_t filter_index;
static uint8_t filter_count;
static uint8_t stall_ticks[2];

static float SpeedControl_Clamp(float value, float minimum, float maximum)
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

static int32_t SpeedControl_ClampTarget(int32_t target_rpm10)
{
    const int32_t maximum = SPEED_CONTROL_TARGET_MAX_RPM * 10;

    if (target_rpm10 < 0)
    {
        return 0;
    }
    return (target_rpm10 > maximum) ? maximum : target_rpm10;
}

static void SpeedControl_ResetState(void)
{
    uint8_t motor;
    uint8_t sample;

    motor1_encoder_poll();
    encoder_previous[0] = motor1_encoder_accum();
    encoder_previous[1] = motor2_encoder_accum();
    filter_index = 0U;
    filter_count = 0U;

    for (motor = 0U; motor < 2U; motor++)
    {
        measured_rpm10[motor] = 0;
        output_duty[motor] = 0;
        integral_duty[motor] = 0.0f;
        delta_sum[motor] = 0;
        stall_ticks[motor] = 0U;
        for (sample = 0U; sample < SPEED_FILTER_LENGTH; sample++)
        {
            delta_buffer[motor][sample] = 0;
        }
    }
}

void SpeedControl_Init(void)
{
    enabled = 0U;
    direction = 1;
    fault_mask = 0U;
    hard_fault_mask = 0U;
    kp_x100 = SPEED_KP_X100_DEFAULT;
    ki_x100 = SPEED_KI_X100_DEFAULT;
    target_rpm10[0] = 0;
    target_rpm10[1] = 0;
    SpeedControl_ResetState();
}

void SpeedControl_Start(int32_t target1_rpm10, int32_t target2_rpm10)
{
    target1_rpm10 = SpeedControl_ClampTarget(target1_rpm10);
    target2_rpm10 = SpeedControl_ClampTarget(target2_rpm10);
    if ((0 == target1_rpm10) && (0 == target2_rpm10))
    {
        SpeedControl_Stop();
        return;
    }
    if (0U != hard_fault_mask)
    {
        fault_mask = hard_fault_mask;
        SpeedControl_Stop();
        return;
    }
    if (0U == enabled)
    {
        SpeedControl_ResetState();
    }
    fault_mask = 0U;
    target_rpm10[0] = target1_rpm10;
    target_rpm10[1] = target2_rpm10;
    enabled = 1U;
}

void SpeedControl_Stop(void)
{
    enabled = 0U;
    direction = 1;                        /* fail-safe：停车后方向复位为前进 */
    target_rpm10[0] = 0;
    target_rpm10[1] = 0;
    integral_duty[0] = 0.0f;
    integral_duty[1] = 0.0f;
    output_duty[0] = 0;
    output_duty[1] = 0;
    motor_set(MOTOR_1, 0);
    motor_set(MOTOR_2, 0);
}

uint8_t SpeedControl_SetDirection(int8_t dir)
{
    if (((dir != 1) && (dir != -1)) || (0U != enabled))
    {
        return 1U;                        /* 运行中切方向 = 瞬间反转，拒绝 */
    }
    direction = dir;
    return 0U;
}

int8_t SpeedControl_GetDirection(void)
{
    return direction;
}

uint8_t SpeedControl_IsEnabled(void)
{
    return enabled;
}

uint8_t SpeedControl_GetFaultMask(void)
{
    return fault_mask;
}

void SpeedControl_ReportEncoderFault(uint8_t motor_mask)
{
    motor_mask &= (SPEED_CONTROL_FAULT_MOTOR1 | SPEED_CONTROL_FAULT_MOTOR2);
    if (0U != motor_mask)
    {
        hard_fault_mask |= motor_mask;
        fault_mask |= motor_mask;
        SpeedControl_Stop();
    }
}

uint8_t SpeedControl_SetGainsX100(int32_t new_kp_x100, int32_t new_ki_x100)
{
    if ((new_kp_x100 < 0) || (new_kp_x100 > SPEED_KP_X100_MAX) ||
        (new_ki_x100 < 0) || (new_ki_x100 > SPEED_KI_X100_MAX))
    {
        return 1U;
    }
    kp_x100 = new_kp_x100;
    ki_x100 = new_ki_x100;
    return 0U;
}

int32_t SpeedControl_GetKpX100(void)
{
    return kp_x100;
}

int32_t SpeedControl_GetKiX100(void)
{
    return ki_x100;
}

int32_t SpeedControl_GetTargetRpm10(uint8_t motor_index)
{
    return (motor_index < 2U) ? target_rpm10[motor_index] : 0;
}

int32_t SpeedControl_GetMeasuredRpm10(uint8_t motor_index)
{
    return (motor_index < 2U) ? measured_rpm10[motor_index] : 0;
}

int32_t SpeedControl_GetOutput(uint8_t motor_index)
{
    return (motor_index < 2U) ? output_duty[motor_index] : 0;
}

int32_t SpeedControl_GetIntegralX10(uint8_t motor_index)
{
    return (motor_index < 2U) ? (int32_t)(integral_duty[motor_index] * 10.0f) : 0;
}

void SpeedControl_Update(void)
{
    int32_t encoder_now[2];
    int32_t delta[2];
    uint8_t motor;
    uint8_t new_fault_mask = 0U;

    motor1_encoder_poll();
    encoder_now[0] = motor1_encoder_accum();
    encoder_now[1] = motor2_encoder_accum();

    for (motor = 0U; motor < 2U; motor++)
    {
        /* 测速入口翻符号：后退时编码器负增长 → 正 rpm，PI 始终看到正误差 */
        delta[motor] = (encoder_now[motor] - encoder_previous[motor]) * (int32_t)direction;
        encoder_previous[motor] = encoder_now[motor];
        delta_sum[motor] -= delta_buffer[motor][filter_index];
        delta_buffer[motor][filter_index] = delta[motor];
        delta_sum[motor] += delta[motor];
    }

    if (filter_count < SPEED_FILTER_LENGTH)
    {
        filter_count++;
    }
    filter_index++;
    if (filter_index >= SPEED_FILTER_LENGTH)
    {
        filter_index = 0U;
    }

    for (motor = 0U; motor < 2U; motor++)
    {
        const int32_t denominator = ENC_COUNTS_PER_REV * (int32_t)filter_count;
        measured_rpm10[motor] = (int32_t)(((int64_t)delta_sum[motor] * 30000LL) / denominator);
    }

    if (0U == enabled)
    {
        return;
    }

    for (motor = 0U; motor < 2U; motor++)
    {
        float error_rpm;
        float proportional;
        float integral_candidate;
        float raw_output;

        if (0 == target_rpm10[motor])
        {
            integral_duty[motor] = 0.0f;
            output_duty[motor] = 0;
            motor_set((motor_id_t)motor, 0);
            continue;
        }

        error_rpm = (float)(target_rpm10[motor] - measured_rpm10[motor]) * 0.1f;
        proportional = ((float)kp_x100 * 0.01f) * error_rpm;
        integral_candidate = integral_duty[motor] +
                             ((float)ki_x100 * 0.01f) * error_rpm *
                             ((float)SPEED_CONTROL_PERIOD_MS * 0.001f);
        integral_candidate = SpeedControl_Clamp(integral_candidate, 0.0f, SPEED_INTEGRAL_MAX);
        raw_output = proportional + integral_candidate;

        /* 条件积分 anti-windup：饱和且误差继续把输出推向饱和时冻结积分。 */
        if (((raw_output >= 0.0f) && (raw_output <= (float)SPEED_CONTROL_OUTPUT_MAX)) ||
            ((raw_output > (float)SPEED_CONTROL_OUTPUT_MAX) && (error_rpm < 0.0f)) ||
            ((raw_output < 0.0f) && (error_rpm > 0.0f)))
        {
            integral_duty[motor] = integral_candidate;
        }

        raw_output = proportional + integral_duty[motor];
        raw_output = SpeedControl_Clamp(raw_output, 0.0f, (float)SPEED_CONTROL_OUTPUT_MAX);
        output_duty[motor] = (int32_t)(raw_output + 0.5f);
        /* PWM 出口翻符号：output_duty 遥测保持「出力大小」语义，实际 PWM 带方向 */
        motor_set((motor_id_t)motor, output_duty[motor] * (int32_t)direction);

        if ((target_rpm10[motor] >= SPEED_STALL_TARGET_MIN_RPM10) &&
            (output_duty[motor] >= SPEED_STALL_OUTPUT_MIN) &&
            (measured_rpm10[motor] <= SPEED_STALL_MEASURED_MAX_RPM10))
        {
            if (stall_ticks[motor] < SPEED_STALL_TIMEOUT_TICKS)
            {
                stall_ticks[motor]++;
            }
            if (stall_ticks[motor] >= SPEED_STALL_TIMEOUT_TICKS)
            {
                new_fault_mask |= (uint8_t)(1U << motor);
            }
        }
        else
        {
            stall_ticks[motor] = 0U;
        }
    }

    if (0U != new_fault_mask)
    {
        fault_mask |= new_fault_mask;
        SpeedControl_Stop();
    }
}
