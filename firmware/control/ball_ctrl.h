/***************************************************************************//**
 * @file    ball_ctrl.h
 * @brief   H 题槽内钢球一维定位控制（题③：0 停留 → 经 +5 折返 → 停 -5）
 *
 * PC 原型 vision/tools/ball_task3_pc.py + ball_brake_pc.py 的 1:1 移植（08-01
 * 定稿版）。控制律与全部参数在 PC 上经 8 轮实跑标定，移植时**逻辑一字不改**，
 * 只换执行环境：HTTP 取数 → TYPE_BALL 帧（CarTasks_GetBallState，已带帧龄
 * 外推），无线串口 'a' 命令 → CarTasks_GimbalSetTarget 直调。
 *
 * 单位约定：PC 侧 "units"（0.1° 电机角，>0 = 往 +x 推）沿用 ——
 *   摆杆目标 deg = (neutral10 - units) / 10
 * neutral10 即 PC 的 neutral（默认 700 = 70.0°，x=0 处平衡角）。
 *
 * CLI：'q 3'=跑题③  'q 0'=中止（回平衡角）  'q'=看状态
 ******************************************************************************/

#ifndef BALL_CTRL_H
#define BALL_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 状态机状态（遥测/CLI 显示用） */
typedef enum
{
    BALL_ST_IDLE = 0,
    BALL_ST_HOLD0,      /* 起点保持展示（计时未开始） */
    BALL_ST_LEG1,       /* 0 → +5：ACC/DEC/峰值跟踪 */
    BALL_ST_LEG2,       /* +5 → -3 粗跳：ACC/DEC/保持 0.4s */
    BALL_ST_SETTLE,     /* 闭环蹭到 -5（平衡角前馈 + PD + 死区保底） */
    BALL_ST_HOLD_END,   /* 终点保持展示（计时已停） */
    BALL_ST_DONE,
    BALL_ST_FAULT,      /* 视觉失效超时等，摆杆回平衡角 */
    /* ---- 题④：车沿 AB 段行驶 1.5m ≤8s，球保持管中心 ±1cm ---- */
    BALL_ST_R_HOLD0,    /* 球稳 0 确认（车未动） */
    BALL_ST_RIDE,       /* 行驶中稳球：平衡角 + PD 锚 0 + 车加速度前馈 */
    BALL_ST_R_END,      /* 停车后保持展示（计时已停） */
    BALL_ST_R_COAST,    /* 循迹 DONE→车真停（编码器归零）：滑停滚阻前馈 */
    BALL_ST_LAP2        /* 题②：空载整圈 ≤20s，出线即停（无球，不走视觉门） */
} BallCtrlState_t;

void    BallCtrl_Init(void);
void    BallCtrl_Update(void);                  /* 20ms 调度任务 */
/** @brief 启动题③流程。前提：摆杆已 'e 1' 标定使能、球已人工放在 0 点附近。
 *  @return 0=已启动 1=拒绝（摆杆未标定 / 视觉不可用 / 正在跑） */
uint8_t BallCtrl_Start(void);
/** @brief 启动题④流程（循迹 1.5m + 行驶稳球）。前提同上 + 车对准 A 点线上。
 *  @return 0=已启动 1=拒绝 */
uint8_t BallCtrl_Start4(void);
/** @brief 题⑤：整圈循迹稳球（锚 0，放球需对中 ±0.6）。 */
uint8_t BallCtrl_Start5(void);
/** @brief 题⑥：整圈循迹稳球，**锚点=启动时球的位置**（±8cm 内任意）——
 *  裁判指定哪就把球放哪，摆放即设定，无需输入界面。 */
uint8_t BallCtrl_Start6(void);
/** @brief 题②：空载整圈 ≤20s，检出停车线即停（车头 ±2cm，不延走）。
 *  无球任务：不走视觉门控，摆杆置平衡角。 */
uint8_t BallCtrl_Start2(void);
void    BallCtrl_Stop(void);                    /* 中止：回平衡角、状态清 IDLE */
BallCtrlState_t BallCtrl_GetState(void);
void    BallCtrl_SetNeutral10(int16_t deg10);   /* 默认 700；标定变了在线改 */
int16_t BallCtrl_GetNeutral10(void);
void    BallCtrl_PrintStatus(void);             /* CLI 'q' 无参：一行状态 */
/** @brief 遥测快照（X5 显示上行用）：状态/位置 x100cm/最远点 x100cm/计时 ms。
 *  计时：运行中=实时值，结束=定格总时长，HOLD0 前=0 */
void    BallCtrl_GetTelem(uint8_t *state, int16_t *x100, int16_t *peak100,
                          uint32_t *t_ms);

#ifdef __cplusplus
}
#endif

#endif /* BALL_CTRL_H */
