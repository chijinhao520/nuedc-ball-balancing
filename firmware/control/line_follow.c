/***************************************************************************//**
 * @file    line_follow.c
 * @brief   灰度阵列循迹 + 停车线检测跑圈 实现
 *
 * 设计依据、赛道账与未知量清单见 line_follow.h 文件头。
 ******************************************************************************/

#include "line_follow.h"
#include "motor.h"
#include "motor_params.h"
#include "speed_control.h"

/*==============================================================================
 * 未知量宏（首测对不上再翻，同 gray.h「翻宏矩阵」约定）
 *============================================================================*/

/* （旧 8 路的极性开关已删：6 路模块的余量判据 analog>threshold 无极性歧义，
 *   07-30 实测 6/6 通道钉死。历史标定履历见 line_follow.c.gray8-20260730.bak。） */

/**
 * @brief 转向方向符号默认值：外轮/内轮的分配方向（见 Update 末尾的不对称分配）。
 *
 * @details 叠加了两个独立未知量，**必须能独立翻**，故不从 YAW_DIFF_SIGN 派生：
 *   ① 灰度 bit 权重的左右朝向（bit7 在物理左还是右）
 *   ② M1/M2 相对车头的左右（motor_params.h 的 YAW_DIFF_SIGN 为 −1 = M1 在左）
 * 上跑道若「越修越偏」，**先用 CLI 'l t -1' 在线翻这个符号，不要先改 PID**。
 *
 * 履历：
 *   2026-07-30  初值 +1（猜测占位）
 *   2026-07-30  旧 8 路实测反转为 −1：+1 时 e10 单调发散（正反馈）起步 16.8cm 丢线；
 *               −1 后整圈 7.69m 零丢线。连带反推 bit7 在物理左侧。
 *   ⚠️ 教训：符号错误一度被「转向权限饱和」掩盖——对称差速分配下 base=40 时
 *   diff_max 只有 10rpm，err 变化被初始姿态主导，看起来像震荡而非发散。
 *   必须先修好权限（不对称分配）才能暴露符号问题。
 *   2026-07-30  **换 6 路模块 → 符号重新变成未知量**：
 *               新模块的通道 1 在物理哪一侧取决于装车朝向，旧值不能继承。
 *   2026-07-30  赛道低速实测钉死 **+1**：沿用 −1 起跑复现教科书发散
 *               （e10 −10→−30→−80→满偏，14cm 丢线 FAULT），'l t 1' 翻转后
 *               60rpm 全程 e10 ±10 内收敛。6 路装车朝向与旧 8 路相反。
 */
#define LINE_TURN_SIGN_DEFAULT  (+1)

/*==============================================================================
 * 内部常量
 *============================================================================*/

/* 边缘外插：线段含最外通道时把误差往外推，越接近丢线纠偏越猛（非线性增益）。
 * 权重表 ch1=−5 / ch6=+5，故含 ch6 → +EXTRA，含 ch1 → −EXTRA。 */
#define LINE_ERR_EDGE_EXTRA     (3.0f)
/* 全丢线时的饱和误差：×Kp(6.0)=60rpm 顶满 diff_max 满舵找线（满偏 5+3=8，取 10） */
#define LINE_ERR_LOST_MAG       (10.0f)

/* 误差慢低通 α = dt/τ = 0.01/0.20 → 时间常数 200ms，作曲率估计（半圆持续 ~10s 足够收敛） */
#define LINE_ERR_LP_ALPHA       (0.05f)
/* 微分低通 α = 0.01/0.033 → 33ms。数字量 err 步长为 2，裸差分 Δ2/10ms=200单位/s，
 * 不滤波则 Kd 稍大即被量化噪声顶得抖振（限幅只有 45rpm）。 */
#define LINE_D_LP_ALPHA         (0.30f)

/* 起步必然压在起停线上（车就摆在那儿），走出这段距离前不判 BLIND、不检停车线。
 * 100mm @0.15m/s ≈ 0.67s，足够走出 18mm 线宽。 */
#define LINE_START_CLEAR_MM     (100)

/*==============================================================================
 * 状态
 *============================================================================*/

/*
 * 6 通道对称权重，下标 = Line6 通道号−1（state_mask 的 bit 序号）。
 * 单位 = 半个探头间距 ≈ 7.4mm（模块 88mm/6 探头，间距 ≈14.7mm）。
 * 数组本身左右对称，物理装反时只需翻 turn_sign，不用改这里。
 */
static const int8_t line_weight[LINE6_CHANNELS] = { -5, -3, -1, 1, 3, 5 };

/* —— 可在线调的增益/速度 —— */
static float   kp, kd, kff, kslow;
static int32_t cruise_rpm;
static int32_t park_rpm;
static uint8_t cross_min_active;
static int32_t cross_margin_min;
static int8_t  turn_sign;
static int32_t stop_advance_mm;
static uint8_t strong_count;          /* 余量≥门槛的通道数（停车/宽黑区判据用） */

/* —— 运行时 —— */
static LineLapState_t    lap_state;
static LineFollowState_t sense_state;
static LineFollowFault_t fault;
static LineStopReason_t  stop_reason;
static int32_t           stop_mm;

static int32_t lap_mm;                /* 0 = 纯循迹模式（不做里程门/不停车） */
static int32_t ramp_rpm_per_s = LINE_RAMP_RPM_PER_S;  /* SetRamp 可改（稳球剖面） */
static uint8_t ramp_symmetric;        /* 1=降速也走斜坡（稳球）；0=瞬跳（默认） */
static uint8_t gentle_stopping;       /* 稳球缓停段：1=降速中 2=爬行等待实际转速收敛 */
static uint32_t gentle_wait_ms;       /* 爬行等待累计 */
static int8_t  plan_accel;            /* 规划加速度标志 +1爬升/-1缓停降速/0平 ——
                                       * 球前馈唯一信号源。曾从 base 差分猜，
                                       * 巡航段 kslow 波动 ±1rpm 被当成满斜坡，
                                       * 前馈 ±34 units 打摆反而晃球 */
static int32_t start_counts;
static int32_t travel_mm;
static int32_t base_rpm;              /* 当前下发的基础速度 */
static float   base_rpm_f;            /* 斜坡用的连续量，避免整数斜坡卡死 */

static float   err_now;
static float   err_lp;
static float   err_prev;
static float   d_lp;
static float   last_valid_error;
static uint8_t has_last_valid;
static float   diff_rpm;

static uint8_t  active_count;
static uint8_t  segment_count;
static uint8_t  wide_frames;
static uint32_t wide_ms;
static uint32_t lost_ms;
static uint32_t lap_ms;
static uint32_t cross_count;
static int32_t  cross_tr_mm;          /* 检出启停线时的里程，外推段的起算点 */

/*==============================================================================
 * 小工具
 *============================================================================*/

static float LineFollow_Clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static int32_t LineFollow_Clampi(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/** @brief 不依赖 libm 的浮点绝对值 */
static float LineFollow_Absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/** @brief 双轮编码器均值。驱动层保证「前进=正计数」，故均值即车体纵向位移。 */
static int32_t LineFollow_ReadCounts(void)
{
    motor1_encoder_poll();            /* 幂等：与 SpeedControl 的调用不冲突 */
    return (motor1_encoder_accum() + motor2_encoder_accum()) / 2;
}

static int32_t LineFollow_CountsToMm(int32_t counts)
{
    return (int32_t)(((int64_t)counts * 1000LL) / (int64_t)ENC_COUNTS_PER_METER);
}

/**
 * @brief 停环（可选刹车）。顺序同 distance_run 的 HardStop：先停环，否则下一 tick 又被覆盖。
 * @param brake 1=主动刹车（到位/急停，②项停车精度靠它）；0=只停环不动电机
 * @details brake=0 是给**互斥让路**场景用的：'1 300' / 's' / 'm 100' 等命令会先调
 *          LineFollow_SetEnable(0) 让路，紧接着自己设定电机状态。若这里无条件刹车，
 *          就会出现「先急刹再滑行」这种与命令语义相反的动作。
 */
static void LineFollow_HardStop(uint8_t brake)
{
    SpeedControl_Stop();
    if (0U != brake)
    {
        motor_brake(MOTOR_1);
        motor_brake(MOTOR_2);
    }
    base_rpm   = 0;
    base_rpm_f = 0.0f;
    diff_rpm   = 0.0f;
}

/** @brief 复位控制器内部状态（不含增益与速度设定），避免下次启用时残留突跳。 */
static void LineFollow_ResetControl(void)
{
    err_now          = 0.0f;
    err_lp            = 0.0f;
    err_prev          = 0.0f;
    d_lp              = 0.0f;
    last_valid_error  = 0.0f;
    has_last_valid    = 0U;
    diff_rpm          = 0.0f;
    active_count      = 0U;
    strong_count      = 0U;
    segment_count     = 0U;
    wide_frames       = 0U;
    wide_ms           = 0U;
    lost_ms           = 0U;
    lap_ms            = 0U;
    cross_tr_mm       = 0;
    travel_mm         = 0;
    base_rpm          = 0;
    base_rpm_f        = 0.0f;
    gentle_stopping   = 0U;
    sense_state       = LINE_FOLLOW_STATE_DISABLED;
}

static void LineFollow_Fail(LineFollowFault_t reason)
{
    stop_mm     = travel_mm;          /* 故障点里程留着诊断（跑到哪儿丢线的） */
    stop_reason = LINE_STOP_FAULT;
    LineFollow_HardStop(1U);          /* 故障必须刹住，不能带着未知状态滑行 */
    lap_state   = LINE_LAP_FAULT;
    fault       = reason;
}

static void LineFollow_FinishEx(LineStopReason_t reason, uint8_t brake_flag)
{
    LineFollow_HardStop(brake_flag);
    lap_state   = LINE_LAP_DONE;
    fault       = LINE_FAULT_NONE;
    stop_reason = reason;
    stop_mm     = travel_mm;          /* 供回填「实测一圈里程」 */
}

static void LineFollow_Finish(LineStopReason_t reason)
{
    LineFollow_FinishEx(reason, 1U);
}

/*==============================================================================
 * 生命周期
 *============================================================================*/

void LineFollow_Init(void)
{
    kp    = (float)LINE_KP_X100_DEFAULT    / 100.0f;
    kd    = (float)LINE_KD_X100_DEFAULT    / 100.0f;
    kff   = (float)LINE_KFF_X100_DEFAULT   / 100.0f;
    kslow = (float)LINE_KSLOW_X100_DEFAULT / 100.0f;

    cruise_rpm       = LINE_CRUISE_DEFAULT_RPM;
    park_rpm         = LINE_PARK_DEFAULT_RPM;
    cross_min_active = LINE_CROSS_MIN_ACTIVE;
    cross_margin_min = LINE_CROSS_MARGIN_MIN;
    turn_sign        = LINE_TURN_SIGN_DEFAULT;
    stop_advance_mm  = LINE_STOP_ADVANCE_MM;

    lap_state   = LINE_LAP_IDLE;
    fault       = LINE_FAULT_NONE;
    stop_reason = LINE_STOP_NONE;
    stop_mm     = 0;
    lap_mm      = 0;
    cross_count = 0U;
    start_counts = 0;
    LineFollow_ResetControl();
}

uint8_t LineFollow_StartLap(int32_t new_lap_mm, int32_t new_cruise_rpm)
{
    if (0 != new_lap_mm)
    {
        if ((new_lap_mm < LINE_LAP_MIN_MM) || (new_lap_mm > LINE_LAP_MAX_MM))
        {
            return 1U;
        }
    }
    if ((new_cruise_rpm < LINE_CRUISE_MIN_RPM) || (new_cruise_rpm > LINE_CRUISE_MAX_RPM))
    {
        return 1U;
    }
    /* 三电机分时启动闸门未解锁时起步会「命令下去了车不动」，直接拒绝而不是空转 */
    if (0U == motor_is_brushed_armed())
    {
        return 1U;
    }

    LineFollow_ResetControl();
    lap_mm       = new_lap_mm;
    cruise_rpm   = new_cruise_rpm;
    fault        = LINE_FAULT_NONE;
    stop_reason  = LINE_STOP_NONE;
    stop_mm      = 0;
    start_counts = LineFollow_ReadCounts();

    /* 从 MIN 起步，由 RAMP 态斜坡爬到 cruise。
     * 稳球模式起步 6（近乎零）：任何非零初值都是速度环的目标**阶跃**，
     * 瞬态加速度 0.3~0.5m/s² 超球权限(0.26) 且逐跑随机（PWM 死区破开时刻/
     * 静摩擦/电压）—— 22 起步实测 worst 散布 0.5~2.6，固定前馈补不掉随机
     * 超限。6 起步 = PI 积分缓爬 duty、车贴着静摩擦阈值起动，阶跃变连续
     * 过程，散布源头消失。起动推迟 ~0.3s，计时线裕量足够。 */
    base_rpm_f = (0U != ramp_symmetric) ? 6.0f : (float)LINE_CRUISE_MIN_RPM;
    base_rpm   = (int32_t)base_rpm_f;
    gentle_stopping = 0U;

    /* 先 Stop 再 Start：Stop 会清 PI 状态并把方向复位为前进（循迹只前进），
     * 也保证从 heading_hold/distance_run 切过来时不残留上一环的积分与方向位。 */
    SpeedControl_Stop();
    SpeedControl_Start(base_rpm * 10, base_rpm * 10);
    if (0U == SpeedControl_IsEnabled())
    {
        fault = LINE_FAULT_SPEED;
        return 1U;
    }

    lap_state = LINE_LAP_RAMP;
    return 0U;
}

/** @brief 内部统一收尾：brake=1 急停刹车 / brake=0 互斥让路（见 HardStop 注释） */
static void LineFollow_StopCommon(uint8_t brake)
{
    if ((LINE_LAP_RAMP == lap_state) || (LINE_LAP_CRUISE == lap_state) ||
        (LINE_LAP_APPROACH == lap_state) || (LINE_LAP_ADVANCE == lap_state))
    {
        /* 人工中止也必须先留下里程，否则紧接着的 ResetControl 会清掉 travel_mm——
         * 而「'l 1' 跑一圈 + 到停车线处敲 'b' 读里程」正是标定圈长的主要手段。 */
        stop_mm     = travel_mm;
        stop_reason = LINE_STOP_MANUAL;
        LineFollow_HardStop(brake);
    }
    lap_state = LINE_LAP_IDLE;
    LineFollow_ResetControl();        /* 内含 sense_state = DISABLED */
}

void LineFollow_SetEnable(uint8_t on)
{
    if (0U != on)
    {
        (void)LineFollow_StartLap(0, cruise_rpm);   /* 纯循迹：lap_mm=0 → 不做里程门 */
    }
    else
    {
        LineFollow_StopCommon(0U);    /* 让路语义：不刹车，由调用方决定电机去向 */
    }
}

void LineFollow_Abort(void)
{
    LineFollow_StopCommon(1U);
}

/*==============================================================================
 * 主循环
 *============================================================================*/

/**
 * @brief 余量加权质心误差（含边缘外插）—— 本次传感器切换的核心收益所在
 * @param margin 各通道正余量（analog−threshold，≤0 已清零），调用前保证非全零
 * @param active_mask margin>0 的位图，用于拆段
 * @return 质心误差；同时更新 segment_count
 *
 * @details 与旧 8 路数字质心的两点区别：
 *   ① 段内用 **margin 加权**而非等权 —— 亚探头连续分辨率，且阈值毛刺被天然
 *      压制（实测在线通道余量 +943 vs 毛刺 +99，权重比 10:1；数字量下两者等权，
 *      毛刺会把质心拽偏 1~2 个权重单位）；
 *   ② 拆段仍按位图（margin>0），多线段时沿用「最接近上一有效位置」的连续性选择。
 */
static float LineFollow_Centroid(const int32_t *margin, uint8_t active_mask)
{
    uint8_t i = 0U;
    int32_t best_wsum  = 0;
    int32_t best_msum  = 0;
    uint8_t best_lo    = 0U;
    uint8_t best_hi    = 0U;
    uint8_t best_n     = 0U;
    float   best_dist  = 0.0f;
    float   err;

    segment_count = 0U;

    /* 拆成连续线段；多线段时选**最接近上一有效位置**的一段（岔路/干扰的连续性选择） */
    while (i < LINE6_CHANNELS)
    {
        uint8_t seg_lo;
        uint8_t seg_n    = 0U;
        int32_t seg_wsum = 0;         /* Σ(weight·margin) */
        int32_t seg_msum = 0;         /* Σ margin */
        float   seg_err;
        float   dist;

        while ((i < LINE6_CHANNELS) && (0U == ((active_mask >> i) & 0x01U))) { i++; }
        if (i >= LINE6_CHANNELS) { break; }

        seg_lo = i;
        while ((i < LINE6_CHANNELS) && (0U != ((active_mask >> i) & 0x01U)))
        {
            seg_wsum += (int32_t)line_weight[i] * margin[i];
            seg_msum += margin[i];
            seg_n++;
            i++;
        }

        segment_count++;
        seg_err = (float)seg_wsum / (float)seg_msum;
        dist    = (0U != has_last_valid) ? LineFollow_Absf(seg_err - last_valid_error)
                                        : LineFollow_Absf(seg_err);

        if ((0U == best_n) || (dist < best_dist) ||
            ((dist == best_dist) && (seg_n > best_n)))
        {
            best_wsum = seg_wsum;
            best_msum = seg_msum;
            best_lo   = seg_lo;
            best_hi   = (uint8_t)(i - 1U);
            best_n    = seg_n;
            best_dist = dist;
        }
    }

    err = (float)best_wsum / (float)best_msum;

    /* 边缘外插：贴到阵列最外侧说明快出线了，误差非线性放大，纠偏更猛 */
    if ((LINE6_CHANNELS - 1U) == best_hi) { err += LINE_ERR_EDGE_EXTRA; }
    if (0U == best_lo)                    { err -= LINE_ERR_EDGE_EXTRA; }

    return err;
}

/**
 * @brief 按状态机算本拍基础速度：斜坡启动 → 巡航(弯道降速) → 末段减速
 * @return 0=正常，1=已在本函数内终止（DONE/FAULT），调用方应立即 return
 */
static uint8_t LineFollow_UpdateBase(void)
{
    const float ramp_step = (float)ramp_rpm_per_s * ((float)LINE_UPDATE_PERIOD_MS / 1000.0f);
    int32_t target;

    /* 里程判据（仅跑圈模式）——**停车主判据**，不是辅助门。
     * 依据：循迹是闭环跟线，同参数同赛道的路径重复性远好于开环行驶，实测一圈
     * 里程的重复性通常 ±1~2cm，够 2cm 要求；而启停线的 active_count 因虚线相位
     * 随机而不稳定（见 line_follow.h 判据注释），不能独自承担停车。
     * 注意 lap_mm 必须用**实测**值回填（车弯道内切/外切，实走 ≠ 中心线 6.14m）。 */
    if (0 != lap_mm)
    {
        /* 缓停段最优先处理：一旦触发即接管速度直到刹停。⚠ 它必须能被
         * **两条**停车路径共同触达 —— 08-01 实测教训：缓停最初只挂在 ODO
         * 兜底分支上，而 B 点有停车线、车实际走的是 ADVANCE(检出线) 路径，
         * 三跑"缓停版"固件其实一直在硬刹（冲击谷 -3.4/-3.9/-4.3 纹丝不降）。 */
        if (0U != gentle_stopping)
        {
            /* 降速斜坡减半（08-01 收尾）：缓停全程在计时线 B 之后，时间
             * 免费 —— 斜坡越缓，球侧需要的前馈越小，前馈失配的绝对误差
             * 同步减半（0.16 版实测减速段仍随机爬 +0.5~+1.7）。 */
            base_rpm_f -= (ramp_step * 0.5f);
            plan_accel = (base_rpm_f > 6.0f) ? -1 : 0;
            if (base_rpm_f <= 6.0f)
            {
                /* 到 12 后恒速爬行 300ms 再 **coast 滑停**（brake=0）：
                 * · 等待段让实际轮速真正收敛到 12（base 到 12 时实测还有
                 *   20+，速度环滞后 —— coast 版实测球仍被甩 5cm/s）；
                 * · coast 不打 motor_brake：brake 冲击量取决于刹瞬实际轮速，
                 *   低速编码器稀疏赌不准（同一逻辑 16 跑无冲击/19 跑甩 9），
                 *   0.04m/s 滚阻半秒自停，没有 brake 就没有冲击。
                 * 停车点精度无所谓 —— 已在计时线 B 之后。 */
                /* 爬行速度 12→6（08-01）：coast 甩球量 ∝ 爬行 Δv。12 时球
                 * 稳吃 -1.5~-2.0cm 正落 -1.1/-1.9 物理死区（71 units/7.1°
                 * 都破不开的管面缺陷点），settle 蹭回来这条路不可靠 ——
                 * 唯一稳的办法是把停车甩幅砍到球压根不进死区。B 后时间
                 * 不计分，爬得慢没有代价。 */
                base_rpm_f = 6.0f;
                if (1U == gentle_stopping)
                {
                    gentle_stopping = 2U;
                    gentle_wait_ms = 0U;
                }
                else
                {
                    gentle_wait_ms += LINE_UPDATE_PERIOD_MS;
                    /* 爬行最后 150ms 预置 plan=-1（停车预置角）：coast 从
                     * 12rpm 滑停只要 ~0.1s，比摆杆执行链(~100ms)还快 ——
                     * 停车时刻才打前馈永远迟到，球每次吃满整个滑停
                     * Δv≈4cm/s（08-01 三连实测，coast 态 ff 加到 0.25 也
                     * 拦不住，相位问题加幅度无效）。停车时刻完全可预测
                     * → 提前一拍让角度先到位等冲击，与发车预置角同理。 */
                    if (gentle_wait_ms >= 150U)
                    {
                        plan_accel = -1;
                    }
                    if (gentle_wait_ms >= 300U)
                    {
                        gentle_stopping = 0U;
                        LineFollow_FinishEx(LINE_STOP_ODO, 0U);
                        return 1U;
                    }
                }
            }
            base_rpm = (int32_t)base_rpm_f;
            return 0U;                 /* 缓停段接管速度，跳过巡航逻辑 */
        }
        /* 外推段优先判：已拿到绝对基准，只差走完固定几何偏移 */
        if (LINE_LAP_ADVANCE == lap_state)
        {
            if (travel_mm >= (cross_tr_mm + stop_advance_mm))
            {
                if (0U != ramp_symmetric)
                {
                    gentle_stopping = 1U;  /* 稳球：过停车点转缓停（车多滑 ~9cm） */
                }
                else
                {
                    LineFollow_Finish(LINE_STOP_CROSS);
                    return 1U;
                }
            }
        }
        else
        {
            if ((LINE_LAP_CRUISE == lap_state) && (travel_mm >= (lap_mm - LINE_APPROACH_MM)))
            {
                lap_state = LINE_LAP_APPROACH;
            }
            /* 稳球模式（ramp_symmetric）：lap_mm 是**计时线不是停车点**
             * （题④判 AB 段用时，过 B 停表、车允许继续走）。全速跑过 B，
             * 之后才开始缓停 —— 时间与停车冲击彻底解耦，两边都不妥协：
             * 72→12rpm 沿斜坡降 0.8s / 约 9cm（在 B 后，不占计时不占里程
             * 精度），12rpm 刹停冲量仅 0.03cm/s 量级，球无感。
             * 过线时刻由调用方盯 GetTravelMm() 自行停表。 */
            if ((LINE_LAP_APPROACH == lap_state) && (travel_mm >= lap_mm))
            {
                if (0U != ramp_symmetric)
                {
                    gentle_stopping = 1U;  /* 过 B：开始缓停（车继续走） */
                }
                else
                {
                    LineFollow_Finish(LINE_STOP_ODO);  /* 常规模式：里程停 */
                    return 1U;
                }
            }
        }
        /* 兜底对 ADVANCE 段同样有效：外推走飞了也要停 */
        if (travel_mm > (lap_mm + LINE_OVERSHOOT_MM))
        {
            LineFollow_Fail(LINE_FAULT_OVERSHOOT); /* 连里程链都失效了才算故障 */
            return 1U;
        }
    }

    if (LINE_LAP_RAMP == lap_state)
    {
        /* 稳球模式两段起步（用户定版）：先 15rpm 低速台阶 400ms —— 让车
         * 平稳破静摩擦并锁线（起步阶跃的瞬态加速度逐跑随机、球权限吃不住，
         * 22 起步实测 worst 散 0.5~2.6），台阶期间球环只需扛 15rpm 一次
         * 小起动；之后再按斜坡提速，加速段前馈按已知斜坡打，全程可预测。 */
        /* 台阶 1.0s（0.4s 实测不够）：起动冲击把球带到 5.4cm/s、偏 0.7cm，
         * 还没被球环收回斜坡就开始，叠加成 1.41 —— 破出冲击消不掉（10~50ms
         * 尺度前馈是聋的），唯一解是**串行化**：台阶期让球环把冲击完整
         * 消化归零，球稳了才提速。 */
        /* 1.4s = 破出(~0.5s) + 冲击窗(0.3s) + 球收稳(0.5s)：斜坡开始时球
         * 必须已静止 —— 1.0s 时球还带回摆动量，撞上斜坡前馈叠成 -1.47。 */
        if ((0U != ramp_symmetric) && (lap_ms < 1700U))
        {
            /* 0→15 缓上（6 起步 300ms 爬满）**已实证压不住破出甩幅**
             * （满电 +2.51 vs 瞬时台阶 +2.3）：冲击来自静→动摩擦切换的
             * 固有力矩落差，输出爬多慢都躲不开。留着只为限制 PI 过冲，
             * 别再指望它治破出 —— 破出治理在 ball_ctrl 侧（接球阻尼）。 */
            /* slew 300→800ms（用户观察"缓启动不连续有加速度突变"）：
             * 粘滑释放的跳变幅度 ≈ 崩开瞬间的指令速度 —— 300ms 太快，
             * 轮子憋到指令 12~15 才崩、一崩就是全速跳变；拉长到 800ms
             * 让崩开发生在指令 7~9 的低位，跳变幅度约减半。
             * 时间账：台阶期均速略降，全程 +0.1s 左右，仍在 8s 内。 */
            base_rpm_f = 15.0f;
            if (lap_ms < 800U)
            {
                base_rpm_f = 6.0f + ((9.0f * (float)lap_ms) / 800.0f);
            }
            base_rpm   = (int32_t)base_rpm_f;
            /* 台阶 1.4→1.7s：满电破出把球甩 ~2.5cm，1.4s 时球还带回摆速度
             * （实测 x=-0.5 仍在往负走）就撞上斜坡前馈，叠成 -2.7。
             * 时间账：+0.3s 后全程 ~7.6s，仍在 8s 内。
             * 最后 100ms 提前置 plan_accel=+1（用户点破"管子要比车快"）：
             * 摆杆执行链滞后 ~100ms，前馈与提速同拍发=车先动管后倾，
             * 球白吃 100ms 的 0.2m/s²。提前一拍打，管到位时车刚好开始加速。 */
            plan_accel = (lap_ms >= 1600U) ? 1 : 0;
            return 0U;
        }
        base_rpm_f += ramp_step;
        plan_accel = (base_rpm_f < (float)cruise_rpm) ? 1 : 0;
        if ((base_rpm_f >= (float)cruise_rpm) && (travel_mm >= LINE_START_CLEAR_MM))
        {
            lap_state = LINE_LAP_CRUISE;      /* 走出起停线才算起步完成，见 START_CLEAR 注释 */
        }
        base_rpm_f = LineFollow_Clampf(base_rpm_f,
                        (0U != ramp_symmetric) ? 15.0f : (float)LINE_CRUISE_MIN_RPM,
                        (float)cruise_rpm);
        base_rpm   = (int32_t)base_rpm_f;
        return 0U;
    }
    plan_accel = 0;                           /* 巡航段：规划加速度恒 0 */

    /* 弯道自动降速：err_lp 是曲率指标，弯道慢、直道快，才是把一圈压进 20s 的正解。
     * 外推段沿用 park 速度——它决定 brake 制动距离，换速度会破坏已标定的停车精度。 */
    target = ((LINE_LAP_APPROACH == lap_state) || (LINE_LAP_ADVANCE == lap_state))
             ? park_rpm : cruise_rpm;
    target -= (int32_t)(kslow * LineFollow_Absf(err_lp));
    target  = LineFollow_Clampi(target, LINE_CRUISE_MIN_RPM, cruise_rpm);

    /* 降速默认立刻生效（安全方向），升速走斜坡（避免地面打滑）。
     * 稳球模式(ramp_symmetric)降速也限斜坡：瞬跳的减速度会把球甩出中心区
     * —— 车载钢球时"温和"才是安全方向，跟线安全性由 ④⑤ 的低巡航速度兜底。 */
    if ((float)target < base_rpm_f)
    {
        if (0U != ramp_symmetric)
        {
            base_rpm_f -= ramp_step;
            if (base_rpm_f < (float)target) { base_rpm_f = (float)target; }
        }
        else
        {
            base_rpm_f = (float)target;
        }
    }
    else
    {
        base_rpm_f += ramp_step;
        if (base_rpm_f > (float)target) { base_rpm_f = (float)target; }
    }
    base_rpm = (int32_t)base_rpm_f;
    return 0U;
}

void LineFollow_Update(const Line6_Data_t *frame, uint8_t fresh)
{
    int32_t margin[LINE6_CHANNELS];
    uint8_t active_mask;
    uint8_t i;
    uint8_t wide;
    float   d_raw;
    int32_t diff_max;
    int32_t m1_rpm, m2_rpm;

    if ((LINE_LAP_IDLE == lap_state) || (LINE_LAP_DONE == lap_state) ||
        (LINE_LAP_FAULT == lap_state))
    {
        return;                       /* 未接管：CLI 手动 duty 保持有效 */
    }

    lap_ms += LINE_UPDATE_PERIOD_MS;
    if (lap_ms > LINE_LAP_TIMEOUT_MS)
    {
        LineFollow_Fail(LINE_FAULT_TIMEOUT);
        return;
    }

    /* 速度环故障（堵转/编码器）直接上抛，不要带着坏内环继续循迹 */
    if (0U != SpeedControl_GetFaultMask())
    {
        LineFollow_Fail(LINE_FAULT_SPEED);
        return;
    }

    travel_mm = LineFollow_CountsToMm(LineFollow_ReadCounts() - start_counts);

    /*——— 输入解释：余量 + 位图 ———
     * margin>0 = 压黑线（07-30 实测钉死：state_bit ≡ analog>threshold，值大=黑）。
     * 数据过期（I2C 失败/熔断）按**全丢线**处理：margin 全清零 → 走外插 + LOST
     * 超时刹车 —— 传感器猝死与物理丢线共用同一条保护路径，不拿旧数据裸奔。 */
    active_mask  = 0U;
    active_count = 0U;
    strong_count = 0U;
    for (i = 0U; i < LINE6_CHANNELS; i++)
    {
        int32_t m = 0;
        if ((0U != fresh) && (0 != frame))
        {
            m = (int32_t)frame->analog[i] - (int32_t)frame->threshold[i];
            if (m < 0) { m = 0; }
        }
        margin[i] = m;
        if (m > 0)
        {
            active_mask |= (uint8_t)(1U << i);
            active_count++;               /* 循迹质心用：不设门槛，加权自然稀释毛刺 */
        }
        if (m >= cross_margin_min)
        {
            strong_count++;               /* 停车/宽黑区计数用：滤掉 +40~+170 级毛刺 */
        }
    }

    /*——— 停车线 / 宽黑区判定 ———
     * 三件事必须区分开（最初版本把它们混成一个 AMBIGUOUS 停车）：
     *   停车线   = active ≥ cross_min_active(3) 的**短脉冲**，只在 APPROACH 段认
     *   模块故障 = active ≥ LINE_BLIND_MIN_ACTIVE(6=全亮) **持续** > BLIND_TIMEOUT
     *   丢线     = active == 0（见下方）
     * 两个阈值必须分开：停车线只盖 3~4 个探头（题面 5cm 垂直段），而 3 拿来做异常
     * 保护会满圈误报——正常循迹就有 a=2、偶发 a=3。详见 line_follow.h 阈值注释。 */
    wide = (strong_count >= cross_min_active) ? 1U : 0U;
    if (0U != wide)
    {
        if (wide_frames < 255U) { wide_frames++; }
        /* 用 == 只在确认那一帧计一次。计的是**整圈所有**疑似启停线（不限 APPROACH 段）——
         * 跑完看 x=：=1 说明只有真启停线触发；偏大说明弯道斜压在误触发（里程门挡住了
         * 没造成误停，但阈值该提高）。这是阈值标定的直接依据。 */
        if (LINE_CROSS_CONFIRM_FRAMES == wide_frames)
        {
            cross_count++;
        }
    }
    else
    {
        wide_frames = 0U;
    }

    /* 宽黑区异常独立计时，用**更高**的阈值，不受停车线阈值影响 */
    if (strong_count >= LINE_BLIND_MIN_ACTIVE)
    {
        wide_ms += LINE_UPDATE_PERIOD_MS;
    }
    else
    {
        wide_ms = 0U;
    }

    if ((LINE_LAP_RAMP != lap_state) && (wide_ms > LINE_BLIND_TIMEOUT_MS))
    {
        LineFollow_Fail(LINE_FAULT_BLIND);
        return;
    }
    if ((LINE_LAP_APPROACH == lap_state) && (wide_frames >= LINE_CROSS_CONFIRM_FRAMES))
    {
        /* 拿到绝对基准 → 转入外推段。**不 return** —— 后面还要继续循迹走完 Δ，
         * 那段路仍需正常的误差计算与轮速下发（判分点通常不在阵列位置上）。 */
        cross_tr_mm = travel_mm;
        lap_state   = LINE_LAP_ADVANCE;
        if (0 == stop_advance_mm)
        {
            LineFollow_Finish(LINE_STOP_CROSS);   /* Δ=0：阵列位置即判分点 */
            return;
        }
    }

    /*——— 误差 ———*/
    if (0U == active_count)
    {
        lost_ms += LINE_UPDATE_PERIOD_MS;
        if (0U == has_last_valid)
        {
            LineFollow_Fail(LINE_FAULT_NO_LINE_AT_START);   /* 起步就没线，别瞎跑 */
            return;
        }
        if ((lost_ms > LINE_LOST_TIMEOUT_MS) && (0U == gentle_stopping))
        {
            /* 缓停段（gentle_stopping）豁免丢线故障（08-01）：B 后缓停
             * 不循迹（diff=0 盲走直线），降速斜坡放缓后车会滑出引导线
             * 尽头 —— 这不是故障是设计内路径；若走 Fail 就是 HardStop
             * 硬刹，把一上午消掉的停车冲击又请回来（实测 f=1 硬刹）。
             * 豁免后缓停按原剖面走完（降速→爬行→coast），真正的兜底
             * 仍有 OVERSHOOT(+800mm) 里程保护。 */
            LineFollow_Fail(LINE_FAULT_LOST);
            return;
        }
        /* 容忍窗内：保持上次符号的饱和误差满舵找线，**不停车**。
         * 旧版丢线即停，半圆弧上瞬时丢线就崩，这是最关键的一处改动。 */
        err_now     = (last_valid_error < 0.0f) ? -LINE_ERR_LOST_MAG : LINE_ERR_LOST_MAG;
        sense_state = LINE_FOLLOW_STATE_EXTRAP;
    }
    else if (active_count >= LINE_BLIND_MIN_ACTIVE)
    {
        /* 真正的宽黑区：多通道质心≈0 会骗人（看着"居中"其实无横向信息），
         * 沿用上次有效误差直线穿过，并且**不更新** last_valid_error。
         * ⚠️ 这里刻意**不用** cross_min_active(3) —— 那是停车线触发阈值。正常循迹
         * 偶发 a=3（车斜压线）时误差仍然可信，用 3 会白丢一帧真实误差、劣化循迹。 */
        lost_ms     = 0U;
        err_now     = last_valid_error;
        sense_state = LINE_FOLLOW_STATE_WIDE;
    }
    else
    {
        lost_ms          = 0U;
        err_now          = LineFollow_Centroid(margin, active_mask);
        last_valid_error = err_now;
        has_last_valid   = 1U;
        sense_state      = (segment_count > 1U) ? LINE_FOLLOW_STATE_MULTI
                                                : LINE_FOLLOW_STATE_NORMAL;
    }

    /*——— 滤波：慢低通作曲率估计，微分低通压量化噪声 ———*/
    err_lp += (err_now - err_lp) * LINE_ERR_LP_ALPHA;
    d_raw   = (err_now - err_prev) / ((float)LINE_UPDATE_PERIOD_MS / 1000.0f);
    d_lp   += (d_raw - d_lp) * LINE_D_LP_ALPHA;
    err_prev = err_now;

    /*——— 基础速度 ———*/
    if (0U != LineFollow_UpdateBase())
    {
        return;
    }

    /*——— 轮速差：PD + 曲率前馈 ———
     * 前馈项 kff·err_lp 与 P 同向，在恒定曲率弯道上等价于把 Kp 抬高，
     * 消除纯 P 必然存在的稳态偏差，又不像 I 项那样会 windup。 */
    /* 稳球缓停段（B 后）不打舵：低速打舵晃车即晃球，计时线已过、循迹
     * 意义消失，直行滑出即可（用户定版）。 */
    if (0U != gentle_stopping)
    {
        diff_rpm = 0.0f;
    }
    else
    {
        diff_rpm = (kp * err_now) + (kd * d_lp) + (kff * err_lp);
    }

    diff_max = LINE_DIFF_ABS_MAX_RPM;
    diff_rpm = LineFollow_Clampf(diff_rpm, (float)(-diff_max), (float)diff_max);

    /*——— 差速**不对称**分配 ———
     * 原实现是对称的（内轮 base−diff / 外轮 base+diff），并把 diff_max 钳到
     * (base − LINE_INNER_MIN_RPM)。**2026-07-30 实测证明这是错的**：base=40 时
     * diff_max 只剩 10rpm，而 err=3 按 Kp=6 需要 18rpm —— 车全程满舵仍转不过来，
     * 起步 13cm 即丢线（df10 从头到尾顶在 ±100）。
     * 结论反直觉但明确：**低速不等于安全，低速把转向权限压死了反而更容易冲出线。**
     *
     * 改为：内轮减到 LINE_INNER_MIN_RPM 触底后不再往下减（那以下是速度环堵转检测
     * 盲区），把它"欠"的那部分差速补到外轮上。这样有效差速不再被 base 绑死，
     * 40rpm 也能拿到 45rpm 的转向权限，代价只是转弯时平均车速略升（而转弯本身
     * 弧长更长，升速是可接受的）。 */
    {
        const int32_t diff_abs = (int32_t)((diff_rpm < 0.0f) ? -diff_rpm : diff_rpm);
        int32_t inner = base_rpm - diff_abs;
        int32_t outer = base_rpm + diff_abs;
        const int32_t sgn = (int32_t)turn_sign * ((diff_rpm < 0.0f) ? -1 : 1);

        if (inner < LINE_INNER_MIN_RPM)
        {
            outer += (LINE_INNER_MIN_RPM - inner);   /* 内轮欠的差速转嫁给外轮 */
            inner  = LINE_INNER_MIN_RPM;
        }
        outer = LineFollow_Clampi(outer, LINE_INNER_MIN_RPM,
                                  LINE_CRUISE_MAX_RPM + LINE_DIFF_ABS_MAX_RPM);

        /* sgn>0 表示需要 M1 更快（turn_sign 与 diff 符号共同决定哪边是外轮） */
        m1_rpm = (sgn > 0) ? outer : inner;
        m2_rpm = (sgn > 0) ? inner : outer;
    }

    SpeedControl_Start(m1_rpm * 10, m2_rpm * 10);
    if (0U == SpeedControl_IsEnabled())
    {
        LineFollow_Fail(LINE_FAULT_SPEED);
    }
}

/*==============================================================================
 * 在线调参
 *============================================================================*/

uint8_t LineFollow_SetGains(int32_t kp_x100, int32_t kd_x100, int32_t kff_x100)
{
    if ((kp_x100 < 0) || (kp_x100 > 5000) ||
        (kd_x100 < 0) || (kd_x100 > 2000) ||
        (kff_x100 < 0) || (kff_x100 > 5000))
    {
        return 1U;
    }
    kp  = (float)kp_x100  / 100.0f;
    kd  = (float)kd_x100  / 100.0f;
    kff = (float)kff_x100 / 100.0f;
    return 0U;
}

uint8_t LineFollow_SetSpeeds(int32_t new_cruise, int32_t new_park, int32_t kslow_x100)
{
    if ((new_cruise < LINE_CRUISE_MIN_RPM) || (new_cruise > LINE_CRUISE_MAX_RPM) ||
        (new_park   < LINE_CRUISE_MIN_RPM) || (new_park   > new_cruise) ||
        (kslow_x100 < 0) || (kslow_x100 > 5000))
    {
        return 1U;
    }
    cruise_rpm = new_cruise;
    park_rpm   = new_park;
    kslow      = (float)kslow_x100 / 100.0f;
    return 0U;
}

uint8_t LineFollow_SetCrossMinActive(int32_t min_active)
{
    /* 下限 2：题面启停线只盖 3~4 个探头，阈值必须能设到 3（默认值）。
     * 误触发由里程门承担（识别仅在 APPROACH 段开启），不再靠高阈值防御。
     * 上限 = 通道数 6（07-30 换 6 路模块同步收紧，设 7 以上永远触发不了）。 */
    if ((min_active < 2) || (min_active > (int32_t)LINE6_CHANNELS))
    {
        return 1U;
    }
    cross_min_active = (uint8_t)min_active;
    return 0U;
}

uint8_t LineFollow_SetTurnSign(int32_t sign)
{
    if ((1 != sign) && (-1 != sign))
    {
        return 1U;
    }
    turn_sign = (int8_t)sign;
    return 0U;
}

uint8_t LineFollow_SetCrossMarginMin(int32_t new_margin)
{
    if ((new_margin < 0) || (new_margin > 2000))
    {
        return 1U;
    }
    cross_margin_min = new_margin;
    return 0U;
}

int32_t LineFollow_GetCrossMarginMin(void)
{
    return cross_margin_min;
}

uint8_t LineFollow_GetStrongCount(void)
{
    return strong_count;
}

uint8_t LineFollow_SetStopAdvance(int32_t mm)
{
    if ((mm < 0) || (mm > LINE_STOP_ADVANCE_MAX_MM))
    {
        return 1U;
    }
    stop_advance_mm = mm;
    return 0U;
}

int32_t LineFollow_GetStopAdvance(void)
{
    return stop_advance_mm;
}

void LineFollow_GetGainsX100(int32_t *out_kp, int32_t *out_kd, int32_t *out_kff, int32_t *out_kslow)
{
    if (0 != out_kp)    { *out_kp    = (int32_t)(kp    * 100.0f); }
    if (0 != out_kd)    { *out_kd    = (int32_t)(kd    * 100.0f); }
    if (0 != out_kff)   { *out_kff   = (int32_t)(kff   * 100.0f); }
    if (0 != out_kslow) { *out_kslow = (int32_t)(kslow * 100.0f); }
}

/*==============================================================================
 * 遥测
 *============================================================================*/

uint8_t LineFollow_IsEnabled(void)
{
    return ((LINE_LAP_RAMP == lap_state) || (LINE_LAP_CRUISE == lap_state) ||
            (LINE_LAP_APPROACH == lap_state) || (LINE_LAP_ADVANCE == lap_state)) ? 1U : 0U;
}

LineFollowState_t LineFollow_GetState(void)        { return sense_state; }
LineLapState_t    LineFollow_GetLapState(void)     { return lap_state; }
LineFollowFault_t LineFollow_GetFault(void)        { return fault; }
LineStopReason_t  LineFollow_GetStopReason(void)   { return stop_reason; }
int32_t           LineFollow_GetStopMm(void)       { return stop_mm; }
uint8_t           LineFollow_GetActiveCount(void)  { return active_count; }
uint8_t           LineFollow_GetSegmentCount(void) { return segment_count; }
int16_t           LineFollow_GetLastError10(void)  { return (int16_t)(err_now  * 10.0f); }
int16_t           LineFollow_GetErrLp10(void)      { return (int16_t)(err_lp   * 10.0f); }
int16_t           LineFollow_GetDiffRpm10(void)    { return (int16_t)(diff_rpm * 10.0f); }
int32_t           LineFollow_GetBaseRpm(void)      { return base_rpm; }
int32_t           LineFollow_GetTravelMm(void)     { return travel_mm; }
uint32_t          LineFollow_GetCrossCount(void)   { return cross_count; }
int32_t           LineFollow_GetCruiseRpm(void)    { return cruise_rpm; }
int32_t           LineFollow_GetParkRpm(void)      { return park_rpm; }

void LineFollow_SetRamp(int32_t rpm_per_s, uint8_t symmetric)
{
    ramp_rpm_per_s = (rpm_per_s > 0) ? rpm_per_s : LINE_RAMP_RPM_PER_S;
    ramp_symmetric = (0U != symmetric) ? 1U : 0U;
}

int8_t LineFollow_GetPlanAccel(void)
{
    return plan_accel;
}
uint8_t           LineFollow_GetCrossMinActive(void) { return cross_min_active; }
int8_t            LineFollow_GetTurnSign(void)     { return turn_sign; }
