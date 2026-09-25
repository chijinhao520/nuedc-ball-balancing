#ifndef ALGORITHM_SPEED_CONTROL_H
#define ALGORITHM_SPEED_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPEED_CONTROL_PERIOD_MS       (20U)
#define SPEED_CONTROL_TARGET_MAX_RPM  (250)
#define SPEED_CONTROL_OUTPUT_MAX      (900)

#define SPEED_CONTROL_FAULT_MOTOR1    (0x01U)
#define SPEED_CONTROL_FAULT_MOTOR2    (0x02U)

void SpeedControl_Init(void);
/** 目标单位为 rpm x10；每路在算法入口钳制到 0..2500。
 *  目标永远是「速度大小」（非负），前进/后退由 SpeedControl_SetDirection 决定。 */
void SpeedControl_Start(int32_t target1_rpm10, int32_t target2_rpm10);
void SpeedControl_Stop(void);

/**
 * 设置行进方向：+1=前进（默认），-1=后退。
 *
 * PI 本体始终工作在正域——方向只在两端翻符号：测速入口把编码器增量乘以
 * dir（后退时负增长翻成正 rpm），PWM 出口把输出乘以 dir。因此 07-28 架空
 * 整定的 Kp=2.00/Ki=16.00、条件积分 anti-windup 与堵转检测全部原样复用，
 * 倒车不需要另一套参数。
 *
 * 仅允许在停车态（IsEnabled()==0）切换，运行中切换会造成瞬间反转，直接拒绝。
 * SpeedControl_Stop() 会把方向复位为 +1（fail-safe：任何停车后默认前进）。
 *
 * @return 0=已设置，1=拒绝（方向非法或速度环正在运行）
 */
uint8_t SpeedControl_SetDirection(int8_t dir);
int8_t  SpeedControl_GetDirection(void);
void SpeedControl_Update(void);
void SpeedControl_ReportEncoderFault(uint8_t motor_mask);

uint8_t SpeedControl_IsEnabled(void);
uint8_t SpeedControl_GetFaultMask(void);
uint8_t SpeedControl_SetGainsX100(int32_t kp_x100, int32_t ki_x100);
int32_t SpeedControl_GetKpX100(void);
int32_t SpeedControl_GetKiX100(void);
int32_t SpeedControl_GetTargetRpm10(uint8_t motor_index);
int32_t SpeedControl_GetMeasuredRpm10(uint8_t motor_index);
int32_t SpeedControl_GetOutput(uint8_t motor_index);
int32_t SpeedControl_GetIntegralX10(uint8_t motor_index);

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_SPEED_CONTROL_H */
