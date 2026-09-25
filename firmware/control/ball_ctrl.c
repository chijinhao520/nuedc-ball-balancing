/***************************************************************************//**
 * @file    ball_ctrl.c
 * @brief   H 题题③钢球定位状态机 —— PC 定稿控制律的 1:1 移植
 *
 * 参数与判据全部来自 PC 侧 8 轮实跑标定（vision/tools/ball-control.md §2'），
 * 每个数字后面都有一跑实测撑着，**移植时不做任何"顺手优化"**：
 *   · a_eff 方向不对称：段1(刹-45)=4.98、段2(刹+45)=12 —— 同方向散 6~15，
 *     两段各用各的，统一模型已两次被数据推翻（+10.68 / +6.54 两次冲管）。
 *   · ACC→DEC 必须要求 v·sign>0：d_brake 只看 v² 不分方向，球带反向速度
 *     起步会被误判"需要刹车"，而 DEC 角按 -sign 打，对反向球是顺风加油
 *     （实测被推到 +11.28 撞端点）。
 *   · DEC 反向松手（dec_fwd 门）：|v|≤1.5 的窗口只在过峰几拍存在，检出
 *     闪动错过后刹车角挂死（实测 -45 挂 2.4s 把球倒推 16cm 撞左端）。
 *   · 保持/收尾环预置角 = 5·x 平衡角（梯度 5 units/cm 静态标定），不按
 *     运动方向赌 ±18（赌错等于坡度之上再加 44 units 净推力）。
 *   · settle 死区保底 floor=22：PD 输出落进死区(20)球会僵在超差位 ——
 *     这正是当年 cascade 极限环的成因。
 ******************************************************************************/

#include "ball_ctrl.h"

#include <stdio.h>

#include "attitude.h"
#include "beeper.h"
#include "car_tasks.h"
#include "console.h"
#include "line_follow.h"
#include "scheduler.h"
#include "speed_control.h"

/*========================== 参数（PC 定稿，units = 0.1° 电机角） ============*/

#define BC_P1_CM            (  5.0f )   /* 折返点 */
#define BC_P2_CM            ( -5.0f )   /* 终点 */
#define BC_LEG2_BIAS_CM     (  2.0f )   /* 粗跳目标 = P2 + 此偏置 */
#define BC_SHOW_MS          ( 2000U )   /* 起/终点保持展示时长 */

#define BC_U_PUSH           ( 45.0f )
#define BC_U_BRAKE          ( 45.0f )
#define BC_U_MAX            ( 45.0f )
#define BC_SLOPE_PER_CM     (  5.0f )   /* 平衡角梯度 units/cm（静态逐点标定） */

/* ⚠ a_eff/lag 是**执行环境相关**的，不能照搬 PC 标定值：PC 的 4.98/0.33 与
 * 12/0.144 把无线串口(吞包重发)+HTTP 取数的链路延迟全都摊进了参数里；M0
 * 直发 CAN(5ms) + 帧龄外推后这些延迟不存在，照搬会把刹车距离高估 3.7 倍
 * —— 01:35 实跑 dec 预测 4.46cm 实滑 1.20cm，peak 只到 +1.81。
 * 下面是用该跑数据反算的 M0 环境值（lag=0.05 ≈ CAN 周期+内环滞后）。 */
#define BC_LEG1_AEFF        ( 15.0f )   /* 刹 -45：d=1.20 @ v=5.3 反算 */
#define BC_LEG1_LAG_S       ( 0.05f )
#define BC_LEG1_VEND        (  1.0f )
#define BC_LEG2_AEFF        (  8.0f )   /* 刹 +45：d=3.47 @ v=7.1 反算 */
#define BC_LEG2_LAG_S       ( 0.05f )
#define BC_LEG2_VEND        (  0.0f )

#define BC_TOL_CM           (  0.8f )   /* 起步已在容差内则不动（推了必冲出去） */
#define BC_VTOL_CMS         (  1.5f )   /* DEC 判停 |v| 门 */
#define BC_LEG_TIMEOUT_MS   ( 3500U )
#define BC_PEAK_TRACK_MS    (  120U )   /* 段1 刹停后继续跟踪最远点 */
#define BC_LEG2_HOLD_MS     (  200U )   /* 粗跳后保持环顶住时长（250 时 5.04s 超 0.04，
                                         * 压 50ms 到 ~4.99s；再压伤末位稳定性不划算） */

#define BC_SETTLE_KP        ( 12.0f )
#define BC_SETTLE_KV        ( 10.0f )   /* 6 压不住破开后的加速，01:35 过冲 1.66 */
/* 破静摩擦保底改**递增试探**：固定 22 + 平衡角合计 -42 单位的破开冲量太大，
 * 粘滞区一破球就滑 1cm 级（01:39 从 -4.02 被推过头到 -6.25），20ms 检测粒度
 * 来不及撤力。从低于死区的 16 起步，每 0.24s +4，球一动立即回落 —— 冲量
 * 逼近"刚好破开"的最小值，破开后的滑距同步最小化。 */
#define BC_SETTLE_FLOOR0    ( 32.0f )   /* 起始保底：权限标定后已知静摩擦
                                         * 死区 ~35u，从 16 爬纯浪费 0.5s，
                                         * 直接站到死区边缘起步 */
#define BC_SETTLE_FSTEP     (  6.0f )   /* 每级增量（4 时 32→100 要 2s） */
#define BC_SETTLE_FSTEP_MS  (  120U )   /* 静止多久升一级（240 时收敛 0.75cm/s
                                         * 太慢：2.5cm 拉回 3.5s 走不完） */
/* FMAX 60 > 权限 45：floor 作用在 u_pd 上，而净输出 = 平衡角(5x) + u_pd ——
 * 球在 -3 时 5x=-15 把 u_pd 抵掉一截，FMAX=45 时净输出最多 30、破不开粘滞
 * 区边缘（实测卡死 -2.98）。放到 60 让 floor 能把总输出顶到 ±45 饱和；
 * 总输出仍由 BC_U_MAX 收口，不会超权限。 */
/* 60→75→140（08-01 权限标定定版）：所谓"深粘滞点破不开"其实是死区
 * ~35u + bc_set_units 内部 ±80 硬钳联手 —— u=70 净力才 0.27m/s²。
 * 140u 净力 ~0.8，配合 FLOOR0 从死区边缘起爬，破开是秒级的事。 */
#define BC_SETTLE_FMAX      ( 140.0f )
#define BC_SETTLE_VFLOOR    (  1.5f )   /* |v| 超此值撤 floor —— 球已破开静摩擦 */
#define BC_SETTLE_OK_CM     (  0.7f )   /* 判分 ±1，窗开到 0.7 让球早点被锁住 */
#define BC_SETTLE_OK_CMS    (  0.4f )
#define BC_SETTLE_OK_MS     (  250U )
#define BC_SETTLE_MAX_MS    ( 2200U )

#define BC_VISION_FAULT_MS  (  300U )   /* 球数据连续不可用超此时长 → FAULT */
/* 行驶模式单独放宽（08-01 题⑥实证）：球在 |x|>4 区域 YOLO found 0/1 闪烁
 * （+4.2~5.8 段丢帧断续），300ms 神经质到把好好的圈中途拉停两次。闪烁
 * 期间帧仍断续到达、帧龄外推能撑，700ms 只在真断流（相机/链路死）时
 * 触发；64rpm 下盲驶 0.7s ≈ 15cm，兜底代价可接受。静止题③保持 300。 */
#define BC_VISION_FAULT_RIDE_MS ( 700U )

/*---- 题④（行驶稳球）参数 ----
 * 时间账：1.5m/8s → 巡航 66rpm(≈0.22m/s)，斜坡 75rpm/s(≈0.25m/s²)：
 * 加减速各 ~0.9s + 匀速 ~5.9s ≈ 7.7s，卡在 8s 内。
 * 物理账：摆杆权限 ±45 units → 最大可补偿 (45·0.42)·(7/5)/100 ≈ 0.26 m/s²
 * 车加速度，75rpm/s 恰好贴着此上限 —— 巡航/斜坡再快球就必然出窗。 */
/* lap_mm = 计时线（过 B 停表，用户定版）：车全速跑过 B 后才缓停，缓停的
 * ~9cm 在 B 之外、不占计时。时间账（全程无降速段）：RAMP 0.56s + 匀速
 * (1.5-0.1)/0.233 ≈ 6.0s → 约 6.6s，8s 限有 1.4s 裕量。 */
#define BC_RIDE_LAP_MM      ( 1500 )    /* AB 段 = 计时线 */
#define BC_RIDE_RPM         (   64 )   /* 60 的基础上拿回起步台阶加长(0.4→1.0s)
                                         * 的时间，cross B ≈7.3~7.4s */
#define BC_RIDE_RAMP        (   60 )    /* rpm/s，SetRamp 对称模式。75 时前馈
                                         * 满额贴权限、PD 无余量压起步瞬态
                                         * (worst 1.33 全在起步)；60 → 前馈
                                         * 34 units，留 11 给 PD。 */
#define BC_RIDE_KP          ( 16.0f )   /* 锚 0 点 PD；12/10 时未建模扰动（速度
                                         * 环瞬时调节等）压不进 1cm，温和上调 */
#define BC_RIDE_KV          ( 12.0f )
/* 车加速度前馈：u_ff = KFF · a_car。a_car 用 base_rpm 规划值 20ms 差分
 * （规划无噪声，比 IMU 干净）。|KFF| = (5/7)·100/0.42 ≈ 170 units/(m/s²)。
 * 符号 08-01 实测定版为负：+170 满值前馈那跑 worst=7.57cm，比首跑弱前馈
 * (~1cm) 恶化 7 倍 —— 越补越歪即方向反，管 +x 朝车尾。 */
/* -170 是 07-31 深夜在**亏电电池**上整定的：亏电时速度环追不上 60rpm/s
 * 规划斜坡（实际 ~0.13m/s²），-34 units 恰好。08-01 晨满电四跑实证欠补：
 * 球从静止(v=0)进斜坡仍稳定被推 -1.5cm（-1.05→-2.53 等），刹车段同样
 * 欠补 +1.08。-230 按 Δ 比例回推。**KFF 与电池状态强相关 —— 比赛/整定
 * 一律满电起跑**（满电=速度环真跟上规划值，是可复现的那个工况）。 */
#define BC_RIDE_KFF         ( -230.0f )
#define BC_RIDE_RPM2MS      ( 0.00333f )/* rpm → m/s（60rpm≈0.20m/s 标定） */
/* 前馈加速度改**编码器实测**，加/减速分开（08-01 晨定论）：
 * 规划斜坡 60rpm/s(=0.20m/s²) 是 base 的斜率，但台阶期轮速实测已被抬到
 * ~37rpm（[spd] 371/371 @ base15，机理待查），主斜坡真实 Δv 只有 37→61、
 * 历时 1s ≈ **0.08m/s²** —— 按 0.20 打前馈 = 过补 2.5 倍，就是斜坡段
 * -2.5 dive 的元凶（KFF -170→-230 后 dive -1.5→-2.45，过补梯度实证）。
 * 减速段相反：摩擦助减速、天然比加速猛，+34 欠补(+1.08) / +46 微过(-0.4)
 * → 取 0.18(×230≈41 units) 居中。 */
/* A_ACC 改**负值**（反直觉，但三点外推实证）：ff -46/-34/-18 units 对应
 * dive -2.45/-1.9/-1.2，线性外推 dive=0 要 ff≈+9 —— 加速段球根本不往车尾
 * 跑，反而往车头滚。机理：加速→底盘尾蹲（IMU pitch 0.3~0.5°）→管子随
 * 车抬头→球沿管下滑(-x)，重力分量 g·sinθ≈0.05~0.09 盖过惯性 (5/7)·0.08。
 * 前馈的对象是"惯性-俯仰重力"的**净值**，符号为负。 */
/* -0.04 的"俯仰重力主导"外推（-46/-34/-18→dive 线性）是在**带 -8u 恒偏**
 * 的车上做的 —— 等效 +9.2u 正推恰好 ≈ 恒偏本身！OFF10 185 修偏后它成了
 * 纯多余正推（实测斜坡段球骑 +0.4、交接峰 +1.0~+1.2 恒正）→ 归零。
 * 修偏后的车斜坡段基本自平衡（0.08m/s² 实际加速度 × 死区≈互抵）。 */
#define BC_RIDE_A_ACC       ( 0.0f )    /* 主斜坡：修偏后无需前馈 */
#define BC_RIDE_A_DEC       ( 0.05f )   /* 缓停段。斜坡减半后 0.08 仍余
                                         * +1.1/+1.2 恒正尾巴（3 跑 2 超）→
                                         * 夹逼下探；若翻负取中点 */
#define BC_RIDE_A_COAST     ( 0.06f )   /* 尾段滑停滚阻减速。6rpm 爬行的
                                         * Δv 本就微小，与 A_DEC 同步夹逼
                                         * 下探（尾巴恒正 +1.1 的另一半） */
#define BC_RIDE_HOLD0_MS    (  800U )   /* 起步前球稳确认 */
#define BC_RIDE_PRESET_MS   (  150U )   /* 起步预置角提前量（执行链滞后 ~100ms） */
#define BC_RIDE_TIMEOUT_MS  ( 15000U )  /* 循迹超时兜底 */
/*---- 题②（空载整圈 ≤20s，出线即停）----*/
#define BC_Q2_LAP_MM        ( 6300 )    /* 圈长实测 6328（07 月 'l r' 定数），
                                         * odo 兜底 6300+800 */
#define BC_Q2_RPM           (  120 )    /* 20s 账：6.33m/20s=0.32m/s 均速；直道
                                         * 0.38、弯道 kslow 降后 ~0.27。首跑标定 */
#define BC_Q2_PARK          (   75 )    /* 进近段降速。40 时最后 90cm 爬行感
                                         * 明显（用户：像缓停）；75≈0.24m/s
                                         * 过线，硬刹滑距 ~1cm 仍守住 ±2cm
                                         * 判据。完全不减速则滑 3~4cm 超判 */
#define BC_Q2_ADV_MM        (    0 )    /* 出线即停（判据车头 ±2cm，不延走） */
#define BC_Q2_TIMEOUT_MS    ( 30000U )
#define BC_LAP_ADV_MM       (  300 )    /* ④⑤⑥缓停触发点：过线后 300mm（原全局
                                         * 默认，题②会改它 → 每次 start 显式置回） */

/*---- 题⑤⑥（整圈循迹稳球）：复用题④ RIDE 全链，仅换里程/速度/锚点 ----*/
#define BC_LAP_FULL_MM      ( 6200 )    /* 里程兜底（停车线 ~5956 会先触发
                                         * ADVANCE→缓停，此值只是 odo 保险） */
#define BC_LAP_TIME_MM      ( 5956 )    /* 计时线：07 月循迹实测启停线里程 */
#define BC_LAP_RPM          (   64 )    /* 首跑 72 时 24.5s（余 5.5s），但弯道
                                         * kslow 回加速无前馈把球推 +1.17 ——
                                         * 用时间余量买稳：64 是整个上午验证过
                                         * 的速度，弯道速度摆幅等比缩，预计
                                         * ~27.3s 仍在 30s 内 */
/* 3.5→9.0（用户定版）：题面要求任意位置都能发车，机器无权拒绝裁判 ——
 * 门槛只拦物理不可行（球出视野/贴死端墙没有摆动空间）。精度分区如实：
 *   |锚|≤3.5  可靠区（行驶 ±1.5、回锚 ±0.8）
 *   3.5~5    发车允许，worst 2~5（发车下挖随深度放大 + 深区行驶检测闪烁）
 *   >5       尽力而为，有撞端风险（-7.81 实测回摆怼端）
 * 历史限幅证据留档：8→5 锚-7.81 怼管端；5→3.5 锚-4.36 worst4.63。 */
#define BC_ANCHOR_MAX_CM    (  9.0f )   /* 题⑥锚点物理限幅（视野+摆动空间） */

#define BC_RIDE_PARK_RPM    (   50 )    /* 题④停车段速度：硬刹的 Δv 全灌给球，
                                         * 66rpm 刹死=球得 15cm/s 冲量必出窗。
                                         * 40 时冲量 9cm/s 但全程 8.52s 超时，
                                         * 50 折中：冲量 12cm/s、时间约 7.3s */

/*========================== 运行态 =========================================*/

typedef enum { LEG_ACC = 0, LEG_DEC, LEG_HOLD } LegPhase_t;

typedef struct
{
    float    target;
    float    a_eff;
    float    lag_s;
    float    v_end;
    uint32_t hold_ms;       /* 刹停后保持环时长（段1=0，粗跳=400） */
} LegParam_t;

static BallCtrlState_t st = BALL_ST_IDLE;
static int16_t  neutral10 = 700;        /* x=0 平衡角 tgt10（PC neutral） */

static LegParam_t   leg;                /* 当前段参数 */
static LegPhase_t   leg_phase;
static float        leg_sign;
static uint8_t      dec_fwd;
static float        peak_cm;            /* 段1 最远点 */
static float        hold_anchor;
static float        hold_u;
static uint8_t      hold_anchored;

static uint32_t t_state;                /* 当前状态进入时刻 */
static uint32_t t_run0;                 /* 计时起点（离开 HOLD0） */
static uint32_t t_leg1_ms;
static uint32_t t_total_ms;
static uint32_t vis_lost_ms;            /* 球数据连续不可用累计 */
static uint32_t ok_since;               /* settle 到位起始时刻（0=未到位） */
static float    settle_floor;           /* 递增破摩擦保底当前值 */
static uint32_t settle_still;           /* 球静止起始时刻（floor 升级计时） */
static int32_t  ride_prev_rpm;          /* 题④前馈：上拍 base_rpm 规划值 */
static float    ride_worst;             /* 题④行驶中球最大偏离（评估用） */
static uint8_t  ride_timed;             /* 题④已过 B 线停表 */
static float    ride_integ;             /* 题④慢积分：吃掉行驶中恒定静差 */
static int32_t  ride_prev_meas;         /* 题④上拍实测轮速（0.1rpm，双轮均） */
static float    ride_a_lp;              /* 实测加速度低通 */
static uint32_t ride_breakout;          /* 破出时刻（0=车还没动） */
static float    ride_anchor;            /* 稳球锚点：q4/q5=0，q6=放球位（题⑥
                                         * 摆放即设定——判据/PD/积分/settle
                                         * 全部相对它） */
static int32_t  ride_lap_mm;            /* 本次循迹里程（q4=1500 / 圈=6200） */
static int32_t  ride_time_mm;           /* 计时线里程（q4=1500 / 圈=5956） */
static int16_t  ride_rpm;               /* 本次巡航速度 */
static uint32_t ride_timeout;           /* 循迹超时（1.5m=15s / 整圈=40s） */
static float    settle_target;          /* settle 目标：q3=-5，q4 停车后=锚点 */
static uint8_t  q4_mode;                /* 1=当前流程是题④（settle 后收尾不同） */
static float    last_x;                 /* 最近一次有效位置（FAULT/盲飞用） */
static uint8_t  bc_blind;               /* 1=本拍无球数据、行驶态冻结保持中 */

/*========================== 执行器 =========================================*/

/** @brief 管身坡度前馈（平衡角项）：±4.5cm 内线性 5x（08-01 标定核实
 *  -4.35 处真平衡 -20.7 vs 模型 -21.8），出界**钳位不再增长** —— 线性
 *  外推在管端发散：+9 处要 +45u，而管端实际下垂（权限标定时球在 +11.4
 *  松手自己回滚），多出的角度全是往端头的净推力，settle 曾因此把球从
 *  +7.5 推到 +10.6（深锚"力矩不一样"的主凶，用户观察触发本修）。 */
static float bc_cmd_prev;               /* 上拍下发角（深负区转角限速用） */

static float bc_slope_u(float x)
{
    float xa = x;

    /* 钳位仅行驶族（④⑤⑥，q4_mode=1）——题⑥深锚才需要；题③（q4_mode=0）
     * 维持原线性 5x：已全链验证（4.96s 达标），赛前一行不动（用户定版）。
     * 两侧不对称（摆杆行程 +389/-167u，机械即不对称）：
     *   +侧 4.5 —— 管端下垂实证（+11.4 松手回滚），线性外推发散
     *   -侧 6.0 —— 无下垂证据；4.5 时 -7.5 处欠角 ~15u，恒差喂出 ±1.7 振荡 */
    if (0U != q4_mode)
    {
        if (xa > 4.5f)  { xa = 4.5f;  }
        if (xa < -6.0f) { xa = -6.0f; }
    }
    return BC_SLOPE_PER_CM * xa;
}

/** @brief 下发摆杆角。units>0 = 往 +x 推（减小 tgt10），与 PC Arm.set 一致 */
static void bc_set_units(float u)
{
    float deg;

    /* ±80→±145（08-01 权限标定）：'a' 阶跃实测 units→球加速度曲线 ——
     * ±50u 仅 0.15m/s²（V 槽滚动摩擦死区 ~35u 吃掉大半），+100u 0.50、
     * +150u ≥0.45（受限于脉冲长度，实际更高）。电机限位 310..866、
     * neutral 699：+145→554、-145→844 双向都在限位内（电流 0.3A/额定
     * 10A，力矩余量巨大）。行驶小信号仍由各环节自身钳位收口。 */
    if (u > 145.0f)  { u = 145.0f;  }
    if (u < -145.0f) { u = -145.0f; }
    /* 深负区防跳球（用户实测：x<-5 时急打角会把球抽离槽面弹跳）：
     * 负端连杆几何的角速度增益大，位置限幅 ±60 + 转角限速 8u/拍
     * （≈400u/s，防管身鞭甩）。仅行驶族（q4_mode），题③不受影响。 */
    if ((0U != q4_mode) && (last_x < -5.0f))
    {
        float du;

        if (u > 60.0f)  { u = 60.0f;  }
        if (u < -60.0f) { u = -60.0f; }
        du = u - bc_cmd_prev;
        if (du > 8.0f)  { u = bc_cmd_prev + 8.0f; }
        if (du < -8.0f) { u = bc_cmd_prev - 8.0f; }
    }
    bc_cmd_prev = u;
    deg = ((float)neutral10 - u) * 0.1f;
    (void)CarTasks_GimbalSetTarget(deg);    /* 超软限幅被拒即维持原目标，安全 */
}

/** @brief 保持环一拍：预置平衡角 + PD 拉回。PC hold_here/STOP 保持环的合并 */
static void bc_hold_tick(float x, float v)
{
    float drift;

    if (0U == hold_anchored)
    {
        hold_anchor = x;
        hold_anchored = 1U;
        hold_u = bc_slope_u(x);          /* 平衡角预置，不赌方向 */
        if (hold_u > 30.0f)  { hold_u = 30.0f;  }
        if (hold_u < -30.0f) { hold_u = -30.0f; }
    }
    drift = x - hold_anchor;
    if ((v > 0.25f) || (v < -0.25f) || (drift > 0.15f) || (drift < -0.15f))
    {
        hold_u = -((v * 6.0f) + (drift * 12.0f));
        if (hold_u > BC_U_MAX)  { hold_u = BC_U_MAX;  }
        if (hold_u < -BC_U_MAX) { hold_u = -BC_U_MAX; }
    }
    else
    {
        hold_u *= 0.9f;                        /* 已定住，缓慢回收 */
    }
    bc_set_units(hold_u);
}

/** @brief 一段推-刹的一拍。返回 1=本段完成（含保持环走完） */
static uint8_t bc_leg_tick(float x, float v, uint32_t now)
{
    float err = leg.target - x;
    float d_brake;
    float av = (v < 0.0f) ? -v : v;

    if (LEG_ACC == leg_phase)
    {
        float vv = (v * v) - (leg.v_end * leg.v_end);

        if (vv < 0.0f) { vv = 0.0f; }
        d_brake = (vv / (2.0f * leg.a_eff)) + (av * leg.lag_s);

        /* v·sign>0 门 + 越过目标兜底（越过时必是自己冲过去的，v·sign>0） */
        if ((((err < 0.0f ? -err : err) <= d_brake) && ((v * leg_sign) > 0.0f)) ||
            ((err * leg_sign) <= 0.0f))
        {
            char line[64];

            leg_phase = LEG_DEC;
            bc_set_units(-BC_U_BRAKE * leg_sign);
            snprintf(line, sizeof(line), "[q3] dec x=%d v=%d need=%d\r\n",
                     (int)(x * 100.0f), (int)(v * 10.0f),
                     (int)(d_brake * 100.0f));
            Console_Write(line);
        }
        else
        {
            bc_set_units(BC_U_PUSH * leg_sign);     /* 每拍重发 */
        }
    }
    else if (LEG_DEC == leg_phase)
    {
        if ((v * leg_sign) > 1.0f)
        {
            dec_fwd = 1U;
        }
        if ((av <= BC_VTOL_CMS) ||
            ((0U != dec_fwd) && ((v * leg_sign) < -1.0f)))
        {
            char line[48];

            leg_phase = LEG_HOLD;
            hold_anchored = 0U;
            t_state = now;                          /* HOLD 段起点 */
            bc_set_units(0.0f);
            snprintf(line, sizeof(line), "[q3] stop x=%d v=%d\r\n",
                     (int)(x * 100.0f), (int)(v * 10.0f));
            Console_Write(line);
        }
        else
        {
            bc_set_units(-BC_U_BRAKE * leg_sign);
        }
    }
    else                                            /* LEG_HOLD */
    {
        if ((uint32_t)(now - t_state) >= leg.hold_ms)
        {
            return 1U;
        }
        bc_hold_tick(x, v);
    }
    return 0U;
}

/** @brief 开始一段：定方向、清运行态。起步已在容差内直接报完成 */
static uint8_t bc_leg_begin(const LegParam_t *p, float x, uint32_t now)
{
    leg = *p;
    leg_phase = LEG_ACC;
    dec_fwd = 0U;
    hold_anchored = 0U;
    t_state = now;
    if (((leg.target - x) < BC_TOL_CM) && ((leg.target - x) > -BC_TOL_CM))
    {
        bc_set_units(0.0f);
        return 1U;                                  /* 不动它：推了必冲出去 */
    }
    leg_sign = (leg.target > x) ? 1.0f : -1.0f;
    bc_set_units(BC_U_PUSH * leg_sign);
    return 0U;
}

/*========================== 状态机 =========================================*/

void BallCtrl_Init(void)
{
    st = BALL_ST_IDLE;
}

uint8_t BallCtrl_Start(void)
{
    int32_t lmin = 0, lmax = 0;
    float x, v;

    CarTasks_GimbalGetLimits10(&lmin, &lmax);
    if ((0 == lmin) && (0 == lmax))
    {
        Console_Write("[q3] reject: gimbal uncalibrated (run 'e 1')\r\n");
        return 1U;
    }
    if (0U == CarTasks_GetBallState(&x, &v))
    {
        Console_Write("[q3] reject: no ball data (X5 --serial? mode ball_yolo1d?)\r\n");
        return 1U;
    }
    /* 起点防呆：球必须人工放在 0 点附近。现场没人看串口，放错必须当场拒 ——
     * 实测球留在上一跑终点(-12)按了启动，控制器从端点推 17cm 长行程，全盘
     * 皆输。4 声蜂鸣区别于"链路不通"的 3 声。 */
    if ((x > 1.5f) || (x < -1.5f))
    {
        char line[64];

        snprintf(line, sizeof(line), "[q3] reject: ball at %+dx100cm, place it at 0\r\n",
                 (int)(x * 100.0f));
        Console_Write(line);
        Beeper_Start(4U);
        return 1U;
    }
    /* HOLD_END/R_END 是"量分保持"不是"忙"：允许直接开下一把（无限保持
     * 后 DONE 永远不会自己到来） */
    if ((BALL_ST_IDLE != st) && (BALL_ST_DONE != st) && (BALL_ST_FAULT != st) &&
        (BALL_ST_HOLD_END != st) && (BALL_ST_R_END != st))
    {
        Console_Write("[q3] reject: busy ('q 0' to abort)\r\n");
        return 1U;
    }
    {
        char line[64];
        snprintf(line, sizeof(line), "[q3] start x=%+dx100cm\r\n", (int)(x * 100.0f));
        Console_Write(line);
    }
    peak_cm = x;
    last_x = x;
    vis_lost_ms = 0U;
    hold_anchored = 0U;
    q4_mode = 0U;
    st = BALL_ST_HOLD0;
    t_state = (uint32_t)SCHEDULER_GET_MS();
    return 0U;
}

/** @brief 题④⑤⑥公共启动：整套 RIDE 链只差里程/计时线/速度/锚点。
 *  anchor_any=0：锚 0，放球门槛 ±0.6（起点误差是纯坏账）；
 *  anchor_any=1：锚点=当前球位（题⑥摆放即设定），限 ±8 避开管端。 */
static uint8_t bc_start_ride(int32_t lap_mm, int32_t time_mm, int16_t rpm,
                             uint8_t anchor_any)
{
    int32_t lmin = 0, lmax = 0;
    float x, v;

    CarTasks_GimbalGetLimits10(&lmin, &lmax);
    if ((0 == lmin) && (0 == lmax))
    {
        Console_Write("[q4] reject: gimbal uncalibrated\r\n");
        return 1U;
    }
    if (0U == CarTasks_GetBallState(&x, &v))
    {
        Console_Write("[q4] reject: no ball data\r\n");
        return 1U;
    }
    /* 1.5→0.6（08-01 统计场教训）：放球偏 -0.62 那把台阶摆动直接叠到
     * -1.31 超标 —— 判据是 ±1，起点误差就是纯坏账，0.6 以上不许发车。 */
    {
        const float lim = (0U != anchor_any) ? BC_ANCHOR_MAX_CM : 0.6f;

        if ((x > lim) || (x < -lim))
        {
            char line[56];

            snprintf(line, sizeof(line),
                     "[q4] reject: ball at %+dx100cm, center it\r\n",
                     (int)(x * 100.0f));
            Console_Write(line);
            Beeper_Start(4U);
            return 1U;
        }
    }
    if ((BALL_ST_IDLE != st) && (BALL_ST_DONE != st) && (BALL_ST_FAULT != st) &&
        (BALL_ST_HOLD_END != st) && (BALL_ST_R_END != st))
    {
        Console_Write("[q4] reject: busy\r\n");
        return 1U;
    }
    ride_anchor = (0U != anchor_any) ? x : 0.0f;
    ride_lap_mm = lap_mm;
    ride_time_mm = time_mm;
    ride_rpm = rpm;
    ride_timeout = (lap_mm > 3000) ? 40000U : BC_RIDE_TIMEOUT_MS;
    (void)LineFollow_SetStopAdvance(BC_LAP_ADV_MM);     /* 题②会改它，置回 */
    if ((ride_anchor > 3.5f) || (ride_anchor < -3.5f))
    {
        Console_Write("[q4] deep anchor: best-effort zone (see envelope note)\r\n");
    }
    {
        char line[72];

        snprintf(line, sizeof(line),
                 "[q4] start: lap=%dmm rpm=%d anchor=%+dx100cm\r\n",
                 (int)lap_mm, (int)rpm, (int)(ride_anchor * 100.0f));
        Console_Write(line);
    }
    last_x = x;
    peak_cm = 0.0f;
    ride_worst = 0.0f;
    vis_lost_ms = 0U;
    hold_anchored = 0U;
    q4_mode = 1U;
    ok_since = 0U;                      /* R_HOLD0 对中环运行态 */
    settle_floor = BC_SETTLE_FLOOR0;
    settle_still = (uint32_t)SCHEDULER_GET_MS();
    st = BALL_ST_R_HOLD0;
    t_state = (uint32_t)SCHEDULER_GET_MS();
    return 0U;
}

uint8_t BallCtrl_Start4(void)
{
    return bc_start_ride(BC_RIDE_LAP_MM, BC_RIDE_LAP_MM, BC_RIDE_RPM, 0U);
}

uint8_t BallCtrl_Start5(void)
{
    return bc_start_ride(BC_LAP_FULL_MM, BC_LAP_TIME_MM, BC_LAP_RPM, 0U);
}

uint8_t BallCtrl_Start6(void)
{
    return bc_start_ride(BC_LAP_FULL_MM, BC_LAP_TIME_MM, BC_LAP_RPM, 1U);
}

uint8_t BallCtrl_Start2(void)
{
    int32_t ks = 0;

    if ((BALL_ST_IDLE != st) && (BALL_ST_DONE != st) && (BALL_ST_FAULT != st) &&
        (BALL_ST_HOLD_END != st) && (BALL_ST_R_END != st))
    {
        Console_Write("[q2] reject: busy\r\n");
        return 1U;
    }
    /* 无球任务：默认剖面（非稳球斜坡）+ 出线即停 + 摆杆置平 */
    LineFollow_SetRamp(0, 0U);
    (void)LineFollow_SetStopAdvance(BC_Q2_ADV_MM);
    LineFollow_GetGainsX100(NULL, NULL, NULL, &ks);
    (void)LineFollow_SetSpeeds(BC_Q2_RPM, BC_Q2_PARK, ks);
    if (0U != LineFollow_StartLap(BC_Q2_LAP_MM, BC_Q2_RPM))
    {
        (void)LineFollow_SetStopAdvance(BC_LAP_ADV_MM);
        Console_Write("[q2] FAULT: line-follow start rejected\r\n");
        return 1U;
    }
    bc_set_units(0.0f);                 /* 管子放平，纯观感 */
    ride_timed = 0U;
    t_total_ms = 0U;
    q4_mode = 0U;
    t_run0 = (uint32_t)SCHEDULER_GET_MS();
    st = BALL_ST_LAP2;
    Console_Write("[q2] start: bare lap, stop at line\r\n");
    return 0U;
}

void BallCtrl_Stop(void)
{
    if ((BALL_ST_R_HOLD0 == st) || (BALL_ST_RIDE == st) ||
        (BALL_ST_LAP2 == st))
    {
        LineFollow_Abort();                 /* 行驶类中止连车一起停 */
    }
    bc_set_units(0.0f);
    st = BALL_ST_IDLE;
}

BallCtrlState_t BallCtrl_GetState(void)
{
    return st;
}

void BallCtrl_SetNeutral10(int16_t deg10)
{
    neutral10 = deg10;
}

int16_t BallCtrl_GetNeutral10(void)
{
    return neutral10;
}

void BallCtrl_GetTelem(uint8_t *state, int16_t *x100, int16_t *peak100,
                       uint32_t *t_ms)
{
    if (NULL != state)
    {
        *state = (uint8_t)st;
    }
    if (NULL != x100)
    {
        *x100 = (int16_t)(last_x * 100.0f);
    }
    if (NULL != peak100)
    {
        *peak100 = (int16_t)(peak_cm * 100.0f);
    }
    if (NULL != t_ms)
    {
        if ((BALL_ST_LEG1 == st) || (BALL_ST_LEG2 == st) ||
            (BALL_ST_SETTLE == st) || (BALL_ST_RIDE == st))
        {
            *t_ms = (uint32_t)((uint32_t)SCHEDULER_GET_MS() - t_run0);
        }
        else if ((BALL_ST_HOLD_END == st) || (BALL_ST_DONE == st) ||
                 (BALL_ST_R_END == st))
        {
            *t_ms = t_total_ms;
        }
        else
        {
            *t_ms = 0U;
        }
    }
}

void BallCtrl_PrintStatus(void)
{
    char line[96];
    float x = 0.0f, v = 0.0f;
    const uint8_t ok = CarTasks_GetBallState(&x, &v);

    snprintf(line, sizeof(line),
             "[q3] st=%u ball=%u x=%+d v=%+d (x100cm/x10cms) neutral10=%d "
             "peak=%+d worst=%d t=%ums\r\n",
             (unsigned int)st, (unsigned int)ok,
             (int)(x * 100.0f), (int)(v * 10.0f), (int)neutral10,
             (int)(peak_cm * 100.0f), (int)(ride_worst * 100.0f),
             (unsigned int)t_total_ms);
    Console_Write(line);
}

void BallCtrl_Update(void)
{
    const uint32_t now = (uint32_t)SCHEDULER_GET_MS();
    float x, v;
    uint8_t ok;

    if ((BALL_ST_IDLE == st) || (BALL_ST_DONE == st) || (BALL_ST_FAULT == st))
    {
        return;
    }

    /* 题②空载圈：无球任务，绕过视觉门控（没球时视觉必然报无数据） */
    if (BALL_ST_LAP2 == st)
    {
        const LineLapState_t ls = LineFollow_GetLapState();
        char line[72];

        if ((0U == ride_timed) && (LINE_LAP_ADVANCE == ls))
        {
            ride_timed = 1U;                        /* 检出停车线 = 车头到线 */
            t_total_ms = (uint32_t)(now - t_run0);
            snprintf(line, sizeof(line), "[q2] head at line t=%ums\r\n",
                     (unsigned int)t_total_ms);
            Console_Write(line);
        }
        if ((LINE_LAP_DONE == ls) || (LINE_LAP_FAULT == ls) ||
            (LINE_LAP_IDLE == ls))
        {
            if (0U == ride_timed)
            {
                t_total_ms = (uint32_t)(now - t_run0);
            }
            snprintf(line, sizeof(line), "[q2] %s t=%ums stop=%dmm\r\n",
                     (LINE_LAP_DONE == ls) ? "done" : "line-fault",
                     (unsigned int)t_total_ms,
                     (int)LineFollow_GetStopMm());
            Console_Write(line);
            (void)LineFollow_SetStopAdvance(BC_LAP_ADV_MM);  /* 恢复④⑤⑥ */
            st = BALL_ST_DONE;
            Beeper_Start((LINE_LAP_DONE == ls) ? 2U : 3U);
        }
        else if ((uint32_t)(now - t_run0) > BC_Q2_TIMEOUT_MS)
        {
            LineFollow_Abort();
            (void)LineFollow_SetStopAdvance(BC_LAP_ADV_MM);
            st = BALL_ST_FAULT;
            Console_Write("[q2] FAULT: lap timeout\r\n");
            Beeper_Start(3U);
        }
        return;
    }

    ok = CarTasks_GetBallState(&x, &v);
    /* 合理性过滤：超龄帧外推会产出爆值（x=-234/v=-327 级实录），一帧
     * 混进来就污染 worst 与控制（worst 曾被打到 7.83 而真球没超 1.6）。
     * 真球物理范围 |x|<12、|v|<60 —— 出界按无效帧走盲驶/失锁路径。 */
    if ((0U != ok) &&
        ((x > 12.5f) || (x < -12.5f) || (v > 60.0f) || (v < -60.0f)))
    {
        ok = 0U;
    }
    bc_blind = 0U;
    if (0U == ok)
    {
        /* 行驶态盲驶保持（用户定版，替代断连急停）：车行驶中本就稳定，
         * 丢图不停车 —— 反馈冻结在最后已知位置（x=last_x, v=0，PD 输出
         * ≈该点平衡角+锚点回拉的常量角），规划前馈与循迹照常跑，图像
         * 回锁即无缝恢复闭环。深锚区检测闪烁从此只是几拍盲飞。
         * 积分/worst 在盲飞拍冻结（假数据不入账）。 */
        if ((0U != q4_mode) &&
            ((BALL_ST_RIDE == st) || (BALL_ST_R_COAST == st)))
        {
            bc_blind = 1U;
            x = last_x;
            v = 0.0f;
        }
        else
        {
        /* 静止场景短暂失锁：维持上拍指令（PC 版 x is None 即 continue）。
         * 连续超时判 FAULT：回平衡角（按最近位置），停状态机 —— 静止时
         * 丢球=球被取走/相机死，才是真故障。 */
        vis_lost_ms += 20U;
        if (vis_lost_ms >= ((0U != q4_mode) ? BC_VISION_FAULT_RIDE_MS
                                            : BC_VISION_FAULT_MS))
        {
            float u = bc_slope_u(last_x);

            if (u > 30.0f)  { u = 30.0f;  }
            if (u < -30.0f) { u = -30.0f; }
            if ((BALL_ST_RIDE == st) || (BALL_ST_R_HOLD0 == st))
            {
                LineFollow_Abort();     /* 盲开车比盲摆杆更危险，连车一起停 */
                LineFollow_SetRamp(0, 0U);
            }
            bc_set_units(u);
            st = BALL_ST_FAULT;
            Console_Write("[q3] FAULT: ball data lost >300ms\r\n");
        }
        return;
        }
    }
    else
    {
        vis_lost_ms = 0U;
        last_x = x;
    }

    switch (st)
    {
    case BALL_ST_HOLD0:
        bc_hold_tick(x, v);
        if ((uint32_t)(now - t_state) >= BC_SHOW_MS)
        {
            static const LegParam_t LEG1 =
            { BC_P1_CM, BC_LEG1_AEFF, BC_LEG1_LAG_S, BC_LEG1_VEND, 0U };

            t_run0 = now;                           /* ---- 计时开始 ---- */
            peak_cm = x;
            st = BALL_ST_LEG1;
            if (0U != bc_leg_begin(&LEG1, x, now))
            {
                st = BALL_ST_LEG2;                  /* 理论不可达（起点在 0） */
            }
        }
        break;

    case BALL_ST_LEG1:
        if (x > peak_cm)
        {
            peak_cm = x;                            /* 判分量：最远点全程跟踪 */
        }
        if (0U != bc_leg_tick(x, v, now))
        {
            /* 段1 完成（含 0.12s 峰值跟踪 —— 用 hold_ms 复用实现） */
        }
        /* 刹停后不保持：hold_ms=0 时 bc_leg_tick 在进 HOLD 的下一拍即返 1。
         * 峰值继续跟踪 BC_PEAK_TRACK_MS 由 LEG_HOLD 改造成本太高，改为：
         * 完成后停 120ms 再起段2，期间峰值跟踪照常（上面的 if 每拍都在跑） */
        if ((LEG_HOLD == leg_phase) &&
            ((uint32_t)(now - t_state) >= BC_PEAK_TRACK_MS))
        {
            static const LegParam_t LEG2 =
            { BC_P2_CM + BC_LEG2_BIAS_CM, BC_LEG2_AEFF, BC_LEG2_LAG_S,
              BC_LEG2_VEND, BC_LEG2_HOLD_MS };
            char line[80];

            t_leg1_ms = (uint32_t)(now - t_run0);
            snprintf(line, sizeof(line), "[q3] leg1 peak=%+d t=%ums\r\n",
                     (int)(peak_cm * 100.0f), (unsigned int)t_leg1_ms);
            Console_Write(line);
            st = BALL_ST_LEG2;
            (void)bc_leg_begin(&LEG2, x, now);
        }
        else if ((uint32_t)(now - t_state) > BC_LEG_TIMEOUT_MS)
        {
            bc_set_units(0.0f);
            st = BALL_ST_FAULT;
            Console_Write("[q3] FAULT: leg1 timeout\r\n");
        }
        break;

    case BALL_ST_LEG2:
        if ((0U != bc_leg_tick(x, v, now)) ||
            ((uint32_t)(now - t_state) > (BC_LEG_TIMEOUT_MS + BC_LEG2_HOLD_MS)))
        {
            st = BALL_ST_SETTLE;                    /* 超时也进收尾闭环兜底 */
            t_state = now;
            ok_since = 0U;
            settle_target = BC_P2_CM;
            settle_floor = BC_SETTLE_FLOOR0;
            settle_still = now;
        }
        break;

    case BALL_ST_SETTLE:
    {
        const float de = settle_target - x;
        const float ade = (de < 0.0f) ? -de : de;
        const float av = (v < 0.0f) ? -v : v;
        /* q3 的 settle 在 5s 计时内，上限只能给 2.2s；q4 停车即停表，
         * settle 拉回不占时间账，放到 3.5s 把 2.5cm 量级的甩出也接得住。
         * 5s 时深粘滞点（-1.2 附近）两跑超时收不完，floor 递增还没爬到
         * 破开值就被掐 → 放 8s，反正不占计时账 */
        const uint32_t settle_max = (0U != q4_mode) ? 8000U : BC_SETTLE_MAX_MS;

        if ((ade <= BC_SETTLE_OK_CM) && (av <= BC_SETTLE_OK_CMS))
        {
            if (0U == ok_since)
            {
                ok_since = now;
            }
            bc_set_units(bc_slope_u(x));      /* 平衡角锁住 */
            if ((uint32_t)(now - ok_since) < BC_SETTLE_OK_MS &&
                (uint32_t)(now - t_state) < settle_max)
            {
                break;
            }
        }
        else
        {
            float u_pd = (BC_SETTLE_KP * de) - (BC_SETTLE_KV * v);

            /* 泵振防护（08-01：settle 跑飞 -6.87 事故）：球速大时 kv 项
             * 打出 ±100+，执行链滞后 100ms 使满舵恒慢半拍 —— 每个来回
             * 给球注一次能量，球被泵着荡出管（-1.7 死区磨 4.5s 后一路
             * 冲到 -9）。动态 PD 钳回 ±60 稳定包络；±140 只留给 floor
             * （准静态破摩擦，无动力学不会泵）。 */
            if (u_pd > 60.0f)  { u_pd = 60.0f;  }
            if (u_pd < -60.0f) { u_pd = -60.0f; }

            ok_since = 0U;
            /* floor 只在球**静止**时给且逐级递增；球一动立即回落交还 PD。
             * 08-01 修：floor 起点必须踩在**当前 PD 输出**之上 —— 原来只在
             * |u_pd|<floor 时替换，可球在 u_pd=29 时照样卡死（蠕动一下就被
             * kv 项咬回死区的极限环），29>16 使 floor 永远不接管，实测 2.2s
             * 只挪 0.08cm。现在卡多久涨多久，直到破开为止。 */
            /* ade 门（08-01 FLOOR0 抬到 32 后必须加）：球已在容差内时
             * floor 还在踹（32u=死区边缘，踹一下就动），±0.3 极限环 3s
             * 凑不满静止判据。floor 只负责"把球弄进容差"，进了就闭嘴、
             * 交给小信号 PD 让静止计时走完。 */
            if ((av < BC_SETTLE_VFLOOR) && (ade > BC_SETTLE_OK_CM))
            {
                const float apd = (u_pd < 0.0f) ? -u_pd : u_pd;

                if (settle_floor < apd)
                {
                    settle_floor = apd;             /* 从 PD 之上起爬 */
                }
                if ((uint32_t)(now - settle_still) >= BC_SETTLE_FSTEP_MS)
                {
                    settle_still = now;
                    if (settle_floor < BC_SETTLE_FMAX)
                    {
                        settle_floor += BC_SETTLE_FSTEP;
                    }
                }
                u_pd = (de >= 0.0f) ? settle_floor : -settle_floor;
            }
            else
            {
                /* 破开值记忆：回落到上次破开值附近而非清零 —— 静摩擦
                 * 门槛短时间内不会大变，从头爬是纯浪费（实测每周期 1s，
                 * 收敛只有 0.75cm/s）。0.7 时深粘滞点（-1.2 附近）限入
                 * 极限环：破开→挪 1mm→回落太深→重爬，8s 都收不完（08-01
                 * 晨三跑两卡）→ 0.85 让再破只差一两级。 */
                settle_floor *= 0.85f;
                if (settle_floor < BC_SETTLE_FLOOR0)
                {
                    settle_floor = BC_SETTLE_FLOOR0;
                }
                settle_still = now;
            }
            {
                /* settle 权限放到 60（0.1°单位，摆杆软限幅内安全）：45 时
                 * 球卡 -1.9 深粘滞点、floor 顶 63 输出仍被钳死，十几拍
                 * 纹丝不动 —— 深锁定的破开力矩就是要比行车权限大。 */
                float u = (bc_slope_u(x)) + u_pd;
                static uint32_t dbg_last;

                if (u > BC_SETTLE_FMAX)  { u = BC_SETTLE_FMAX;  }
                if (u < -BC_SETTLE_FMAX) { u = -BC_SETTLE_FMAX; }
                bc_set_units(u);
                if ((uint32_t)(now - dbg_last) >= 400U)
                {
                    char line[64];

                    dbg_last = now;
                    snprintf(line, sizeof(line),
                             "[q3] stl x=%d v=%d fl=%d u=%d\r\n",
                             (int)(x * 100.0f), (int)(v * 10.0f),
                             (int)settle_floor, (int)u);
                    Console_Write(line);
                }
            }
            if ((uint32_t)(now - t_state) < settle_max)
            {
                break;
            }
        }
        /* 到位保持满 0.25s，或 2.2s 上限 → 收尾展示。
         * 题③此刻停表；题④的表在停车瞬间已停（settle 拉回不计时）。 */
        if (0U == q4_mode)
        {
            t_total_ms = (uint32_t)(now - t_run0);
            {
                char line[96];

                snprintf(line, sizeof(line),
                         "[q3] done x=%+d peak=%+d t1=%u t2=%u total=%ums\r\n",
                         (int)(x * 100.0f), (int)(peak_cm * 100.0f),
                         (unsigned int)t_leg1_ms,
                         (unsigned int)(t_total_ms - t_leg1_ms),
                         (unsigned int)t_total_ms);
                Console_Write(line);
            }
            Beeper_Start(2U);                       /* 2 声=完成（q4 停车时已鸣） */
        }
        else
        {
            char line[56];

            snprintf(line, sizeof(line), "[q4] settled x=%+d (x100cm)\r\n",
                     (int)(x * 100.0f));
            Console_Write(line);
        }
        hold_anchored = 0U;
        st = (0U != q4_mode) ? BALL_ST_R_END : BALL_ST_HOLD_END;
        t_state = now;
        break;
    }

    case BALL_ST_HOLD_END:
        /* 无限保持（08-01 用户定版）：原来 SHOW_MS 后撒手交静摩擦，但裁判
         * 什么时候量完不归我们管 —— 保持环一直锚住 -5，直到 'q 0'/重新
         * 启动才释放。功耗/发热无虞（保持角电流 ~0.3A）。 */
        bc_hold_tick(x, v);
        break;

    /*==================== 题④：行驶稳球 ====================*/
    case BALL_ST_R_HOLD0:
    {
        /* 发车前对中环做过一版又撤了（用户定）：手动摆放误差不大，
         * 对中反而常 5s 超时拖流程。保持环锚放球位即可。 */
        if ((uint32_t)(now - t_state) < (BC_RIDE_HOLD0_MS - BC_RIDE_PRESET_MS))
        {
            bc_hold_tick(x, v);
            break;
        }
        if ((uint32_t)(now - t_state) < BC_RIDE_HOLD0_MS)
        {
            /* 发车前预置角 = **起动冲击补偿值**（与 RIDE 头 350ms 窗口同值
             * 0.13，跨发车瞬间无跳变，用户定版时序）：提前 150ms 在位，
             * 破出冲击来时角度已经等着。原来挂斜坡满值(-34)发车瞬间跳变
             * 到窗口值，衔接毛刺。斜坡前馈到提速段才上。 */
            /* 0.13→0.10：KFF -170→-230 后预置 units 被动涨到 -30、把球
             * 预推过头（08-01 台阶期 -1.5 过推），换算回验证过的 -23 */
            bc_set_units((bc_slope_u(x))
                         + (BC_RIDE_KFF * 0.10f));
            break;
        }
        {
            int32_t ks = 0;

            /* 稳球速度剖面（斜坡 75rpm/s、降速也走斜坡）+ 低速停车段 +
             * 循迹定距启动。失败（未 armed / 传感异常）FAULT 蜂 3 声。 */
            LineFollow_SetRamp(BC_RIDE_RAMP, 1U);
            LineFollow_GetGainsX100(NULL, NULL, NULL, &ks);
            /* park = cruise：B 前不降速（B 是计时线，冲击由 B 后缓停解决） */
            (void)LineFollow_SetSpeeds(ride_rpm, ride_rpm, ks);
            if (0U != LineFollow_StartLap(ride_lap_mm, ride_rpm))
            {
                LineFollow_SetRamp(0, 0U);
                bc_set_units(bc_slope_u(x));
                st = BALL_ST_FAULT;
                Beeper_Start(3U);
                Console_Write("[q4] FAULT: line-follow start rejected\r\n");
                break;
            }
            ride_prev_rpm = LineFollow_GetBaseRpm();
            ride_prev_meas = (SpeedControl_GetMeasuredRpm10(0U) +
                              SpeedControl_GetMeasuredRpm10(1U)) / 2;
            ride_a_lp = 0.0f;
            /* 预载撤销（08-01 尾修）：恒偏已在源头修（GIMBAL_NEUTRAL_OFF10
             * 177→185，等效全场景 +8u），再预载就是双重补偿往 +x 推。 */
            ride_integ = 0.0f;
            ride_worst = 0.0f;
            ride_timed = 0U;
            ride_breakout = 0U;
            t_run0 = now;                           /* ---- 计时开始 ---- */
            st = BALL_ST_RIDE;
        }
        break;
    }

    case BALL_ST_RIDE:
    {
        const LineLapState_t ls = LineFollow_GetLapState();
        const int32_t rpm_now = LineFollow_GetBaseRpm();
        /* 车加速度前馈按**规划状态查表**，不等差分（差分要等变化发生，
         * 而执行链滞后 ~100ms，等测到球已被甩 —— 首跑实证"来不及反应"）：
         *   RAMP     斜坡加速度已知恒定 → 满前馈 +0.25
         *   降速段   对称斜坡在降(base 差分为负) → 满前馈 -0.25
         *   ADVANCE  已见停车点、马上硬刹 → 抢先打反向角迎接冲击
         *   CRUISE   匀速 → 差分兜底（跟弯道降速那类小变化） */
        float a_car;
        float u;
        float kv_eff;

        /* 前馈信号源 = line_follow 的规划加速度标志（+1/-1/0）——它自己
         * 知道正处在爬升/缓停/平速，不需要猜。此前从 base 差分推断：巡航
         * 段 kslow 弯道降速让 base 波动 ±1rpm，一拍负差分就被当满斜坡，
         * 前馈 ±34 units 打摆反把球晃出 ±0.7（遥测 a100 全程乱跳实证）。
         * 实测轮速差分也试过（worst 3.37）：低通滞后 60ms 追不上瞬态。 */
        {
            const int8_t pa = LineFollow_GetPlanAccel();

            a_car = (pa > 0) ? BC_RIDE_A_ACC
                             : ((pa < 0) ? -BC_RIDE_A_DEC : 0.0f);
        }
        /* 起动破出补偿改**事件触发**（用户点破"把握启动关闭时间"）：
         * 破出实际发生在发车后 350~750ms（15rpm 台阶下速度环积分要 0.5s
         * 才破静摩擦），固定 0~350ms 窗恰好在冲击**前**关闭 —— 角度提前
         * 拉了球（现场看着像打反了）、冲击来时反而没角度接，峰 1.58 照旧。
         * 时刻随电压/摩擦漂，赌不准 → 改盯实测轮速：首次 ≥3rpm = 破出，
         * 该时刻起再保 300ms；破出前一直挂着等它。 */
        /* 破出窗补偿量改 **IMU 实测**（用户提议，补上"固定补偿不适配随机
         * 幅度"的洞）：冲击幅度逐跑随机 0.5~2.2cm（同一事件窗 +0.54 与
         * +2.19 两重天），固定 0.13 只移均值不压散布。IMU 加速度计直接测、
         * 5ms 采样 25Hz 低通仅 ~15ms 延迟，冲多大补多大。
         * 钳位 [0.10, 0.45]：下限保证角度始终在位等冲击（IMU 静时读数≈0，
         * 不能让角度掉光），上限防俯仰重力泄漏放大；窗外不用 IMU（振动噪
         * 声不值得全程吃）。 */
        if ((0U == ride_breakout) ||
            ((uint32_t)(now - ride_breakout) < 200U))
        {
            /* 取负：遥测逐把核对（08-01 晨 4 跑 + 昨夜 2 跑）加速段 ia
             * 恒负、刹车段恒正 —— 底盘俯仰重力泄漏（加速→尾蹲→g·sinθ
             * 投影为负）幅度盖过真实加速度、符号占主导。原符号直用被
             * 下限钳位吃掉，"IMU 实测补偿"实际退化成固定 0.10。泄漏幅度
             * 与下蹲力矩（=冲击强度）正相关，取负后反而是可用的冲击
             * 强度信号；钳位窗 [0.10,0.45] 继续兜住噪声。 */
            float a_imu = -attitude_fwd_acc_mps2;

            /* "补偿给久了"（用户点破，08-01 终版）：冲击本体 ~50ms，但
             * 固定下限让反向角挂满整个窗 —— 冲击一过它就是纯回推力，球
             * 被送到 -1.4（连锁：低速段停 0.7s → 积分膨胀 +13 → 斜坡
             * 带着膨胀积分反冲 +1.83）。破出**前**保留下限（预倾角在位
             * 等冲击）；破出**后**下限放 0，补偿跟 IMU 实测自然衰减
             * （LP 尾巴 ~40ms）；窗口 300→200ms 只作兜底。 */
            const float a_lo = (0U == ride_breakout) ? 0.07f : 0.0f;

            if (a_imu < a_lo)   { a_imu = a_lo;  }
            if (a_imu > 0.45f)  { a_imu = 0.45f; }
            a_car += a_imu;
            if (0U == ride_breakout)
            {
                const int32_t meas = (SpeedControl_GetMeasuredRpm10(0U) +
                                      SpeedControl_GetMeasuredRpm10(1U)) / 2;

                if (meas >= 30)
                {
                    ride_breakout = now;            /* 车动了 */
                }
            }
        }
        ride_prev_rpm = rpm_now;
        /* 前馈**非对称**平滑（08-01 深夜定版，两跑教训拼接）：
         *   幅度增大（进入加/减速段）→ 直通。相位就是一切 —— 对称低通那
         *   版把发车前馈拖慢 200ms，预置角刚撤前馈还没到位，球被甩 +2.0；
         *   幅度减小（退出）→ 低通 0.12。RAMP→CRUISE 交接查表值 0.20→0
         *   阶跃，而车真实加速度渐消（速度环追尾），直通撤等于净反向推力
         *   （实测甩 -1.62）。 */
        {
            const float aa = (a_car < 0.0f) ? -a_car : a_car;
            const float al = (ride_a_lp < 0.0f) ? -ride_a_lp : ride_a_lp;

            if (aa >= al)
            {
                /* 进入：直通。软化（0.5 混合）试过一版即撤：它把破出窗的
                 * 角度加载也拖慢两拍，10~50ms 尺度的冲击等不起（球冲
                 * +1.96、连锁到 -2.09）。斜坡与回摆的叠加问题由台阶加长
                 * （1.4s，球收稳后才提速）解决，不牺牲前馈相位。 */
                ride_a_lp = a_car;
            }
            else if ((0.0f == a_car) && (rpm_now <= 15))
            {
                ride_a_lp = 0.0f;                   /* 车近停：干脆收，低通
                                                     * 尾巴会拖 200ms 把球往
                                                     * +x 推 0.5（用户实观） */
            }
            else
            {
                ride_a_lp += 0.12f * (a_car - ride_a_lp);  /* 退出：平滑 */
            }
        }
        a_car = ride_a_lp;
        {
            const float ex = x - ride_anchor;       /* 判据相对锚点（题⑥） */
            const float ax = (ex < 0.0f) ? -ex : ex;

            if ((0U == bc_blind) && (ax > ride_worst))
            {
                ride_worst = ax;        /* 盲飞拍冻结的 x 不入账 */
            }
        }
        /* 慢积分吃恒定静差：遥测实证匀速段球稳稳钉在 -0.9±0.15 —— 车上
         * 的平衡角与静态标定有恒偏（底盘姿态/装配），PD 无积分吃不掉，
         * 这 0.9 就是 worst 的地板。ki=3 units/(cm·s)：0.9cm 偏差 ~2s 收敛；
         * 限幅 ±16（一个死区）防 windup。 */
        /* 低速台阶段（base≤20）冻结积分（08-01 尾修）：弱振动区摩擦
         * 满额、积分推不动球只会憋劲 —— 球趴 -1.1 它照涨到 +30，提速
         * 振动一回归全数倒出，球被弹 +1.94（worst 就是它）。窗口期只
         * 靠预载 +8 顶恒偏，真正的学习从车提速后开始。 */
        if ((rpm_now > 20) && (0U == bc_blind))
        {
            ride_integ += (ride_anchor - x) * 14.0f * 0.02f;    /* ki=3 追不上
                                                     * 沿途坡度缓变；8 仍留
                                                     * -0.45 恒偏贯穿 → 14；
                                                     * 盲飞拍冻结防假误差积累 */
        }
        if (ride_integ > 35.0f)  { ride_integ = 35.0f;  }   /* ±16 实测顶满
                                                     * 仍剩 -1.1 静差，放开 */
        if (ride_integ < -35.0f) { ride_integ = -35.0f; }
        /* 接球阻尼：破出冲击在源头消不掉（缓上台阶实证无效，静→动摩擦
         * 力矩落差固有），只能接 —— 台阶期（前 1.7s）阻尼加倍，甩出去的
         * 动能在回摆里吃掉，球进主斜坡时接近静止。窗外恢复 12：巡航段
         * v 噪声 ±1.5cm/s × 22 = ±33 units 抖动吃不消。 */
        kv_eff = ((uint32_t)(now - t_run0) < 1700U) ? 26.0f : BC_RIDE_KV;
        /* 接球阻尼 22→26：破出已被按到 +0.3，但回摆仍带球到 -0.9~-1.3
         * （台阶洼地的主体），再加深一点阻尼把回摆能量吃干净。 */
        {
            /* 弹弓教训（08-01 终版结构）：大权限窗若罩住整个 u，冲击瞬间
             * kv×v(=22×3.3=73u) 全额拍出去、球反飞时窗口已关只剩 45 收 ——
             * 打人满权限救人半权限，+0.28 的小破出被弹到 -2.76。
             * 定版：**反馈永远 ±45**（稳定包络，±60 时代它就是饱和稳定器），
             * 破出窗 ±130 只给前馈 —— IMU 补偿与冲击同相，物理上不激振；
             * 130u 净力 ~0.73m/s²（死区 35 后）盖得住 0.3~0.5 的冲击。 */
            float u_fb = (bc_slope_u(x))      /* 管身坡度前馈（绝对位置）*/
                         - (BC_RIDE_KP * (x - ride_anchor)) /* 锚点 PD */
                         - (kv_eff * v)
                         + ride_integ;              /* 慢积分（恒偏校正） */

            /* 台阶窗小正偏置：破出回摆低点恒在负侧（-1.1 对 +0.9）——
             * +6u 拉回摆动中心。窗口 1700→1500：只罩踢球+回摆段，斜坡
             * 开始前撤掉，别跟斜坡段叠加把交接峰顶出去（+1.18 实测）。 */
            if ((uint32_t)(now - t_run0) < 1500U)
            {
                u_fb += 6.0f;
            }
            /* 深锚起步偏置（题⑥）：起步扰动 ∝ 锚点、方向离心 —— 锚 +6.8
             * 被推到 +8.7 / 锚 -3.3 挖到 -5.3 镜像对称（管被球压沉 ∝ 偏
             * 距）。窗口罩台阶+主斜坡（2600ms），锚 0 时严格为零：③④⑤
             * 与浅锚行为一字不变。系数 1.2 由 +6.8 过冲 1.9 反推首标。 */
            {
                /* 两侧系数分立：+侧 2.2（1.2→1.9过冲/1.8→1.52/2.2 封版）；
                 * -侧 1.1 —— 镜像套 2.2 时起步被反推 +1.3 再回摆 -10.8
                 * 险怼端（-7.57 实测），负侧有效力/管型皆不同，减半首标。
                 * 撤销改 600ms 线性淡出（原 2600ms 一刀切）：交接峰 +1.4
                 * 两侧同号同幅、时刻正卡撤销点 —— 阶跃撤销本身就是 15u
                 * 级扰动，叠上斜坡收尾的速度环补冲尾巴。 */
                const uint32_t tr = (uint32_t)(now - t_run0);

                if (tr < 3200U)
                {
                    float w = 1.0f;

                    if (tr > 2600U)
                    {
                        w = 1.0f - ((float)(tr - 2600U) / 600.0f);
                    }
                    u_fb -= ((ride_anchor > 0.0f) ? 2.2f : 1.1f)
                            * ride_anchor * w;
                }
            }
            const float u_ff = BC_RIDE_KFF * a_car; /* 车加速度前馈 */
            const float um = ((0U == ride_breakout) ||
                              ((uint32_t)(now - ride_breakout) < 300U))
                             ? 130.0f : BC_U_MAX;

            if (u_fb > BC_U_MAX)  { u_fb = BC_U_MAX;  }
            if (u_fb < -BC_U_MAX) { u_fb = -BC_U_MAX; }
            u = u_fb + u_ff;
            if (u > um)  { u = um;  }
            if (u < -um) { u = -um; }
        }
        bc_set_units(u);
        /* 行驶过程遥测：给 KFF 精修用 —— 看球偏差与哪个速度段相关。
         * a100>0 = 加速段、<0 = 减速段、=0 = 匀速；x 与 a 同号漂 = 前馈
         * 不足（|KFF| 加大），反号漂 = 过补。 */
        {
            static uint32_t r_dbg;

            if ((uint32_t)(now - r_dbg) >= 250U)
            {
                char line[64];

                r_dbg = now;
                snprintf(line, sizeof(line),
                         "[q4r] x=%d v=%d a100=%d ia=%d ls=%u\r\n",
                         (int)(x * 100.0f), (int)(v * 10.0f),
                         (int)(a_car * 100.0f),
                         (int)(attitude_fwd_acc_mps2 * 100.0f),
                         (unsigned int)ls);
                Console_Write(line);
            }
        }

        /* ---- 过 B 线停表（车继续走，缓停在计时之外）---- */
        if ((0U == ride_timed) &&
            (LineFollow_GetTravelMm() >= ride_time_mm))
        {
            char line[48];

            ride_timed = 1U;
            t_total_ms = (uint32_t)(now - t_run0);
            snprintf(line, sizeof(line), "[q4] cross B t=%ums\r\n",
                     (unsigned int)t_total_ms);
            Console_Write(line);
        }
        if ((LINE_LAP_DONE == ls) || (LINE_LAP_FAULT == ls) ||
            (LINE_LAP_IDLE == ls))
        {
            char line[80];

            if (0U == ride_timed)
            {
                t_total_ms = (uint32_t)(now - t_run0);  /* 异常兜底停表 */
            }
            snprintf(line, sizeof(line),
                     "[q4] %s t=%ums worst=%d x=%d (x100cm)\r\n",
                     (LINE_LAP_DONE == ls) ? "done" : "line-fault",
                     (unsigned int)t_total_ms,
                     (int)(ride_worst * 100.0f), (int)(x * 100.0f));
            Console_Write(line);
            LineFollow_SetRamp(0, 0U);              /* 恢复默认剖面 */
            /* DONE ≠ 车停：只是指令归零，车还带 ~12rpm 在滚阻里滑 0.4s。
             * 原来这里直转 settle，滑停减速没人补偿（settle 不打前馈角），
             * 球被最后一脚从 +0.63 甩到 -2.58 正落深粘滞区、8s 抠不回
             * （08-01 晨实测）→ 先进 R_COAST 给滚阻前馈，车真停再 settle。 */
            st = BALL_ST_R_COAST;
            t_state = now;
            ok_since = 0U;                          /* R_COAST 静止判据计时 */
            Beeper_Start((LINE_LAP_DONE == ls) ? 2U : 3U);
        }
        else if ((uint32_t)(now - t_run0) > ride_timeout)
        {
            LineFollow_Abort();
            LineFollow_SetRamp(0, 0U);
            bc_set_units(bc_slope_u(x));
            st = BALL_ST_FAULT;
            Console_Write("[q4] FAULT: ride timeout\r\n");
        }
        break;
    }

    case BALL_ST_R_COAST:
    {
        /* 滑停滚阻前馈：coast 段减速度由滚阻决定（12rpm→0 约 0.4s），
         * 与刹车段同向（车减速 → 球往 -x 甩 → 需要 +units 顶住）。
         * 阻尼用接球值 22：这段球速可观、噪声占比小。 */
        const int32_t meas = (SpeedControl_GetMeasuredRpm10(0U) +
                              SpeedControl_GetMeasuredRpm10(1U)) / 2;
        float u = (bc_slope_u(x))
                  - (BC_RIDE_KP * (x - ride_anchor)) - (22.0f * v);

        /* 所有"额外的力"都只在轮子还转时有资格存在；车一停只留小信号
         * PD 扶着（08-01 尾修）：
         * · coast 前馈——轮子还转 = 还在减速，编码器读零立即掐；
         * · ride_integ——那是行驶振动工况学的恒偏，静止时它+死区补偿
         *   合成 +45u 真实推力，把停在 -0.01 的球硬推到 +0.40，settle
         *   拉回又过冲 -1.17（用户看到的"回来的有些多"全链条）；
         * · 死区补偿——同理，静止时它就是把小误差放大成 35u 的锤子。 */
        if (meas > 5)
        {
            u += ride_integ;
            u += (BC_RIDE_KFF * -BC_RIDE_A_COAST);
            /* 死区补偿已撤（08-01 尾修 2）：它是 12rpm 爬行时代"顶不住
             * 0.3 滑停"的产物；6rpm 爬行 + 预置角 + 修偏后，coast 的活
             * 只剩轻扶，+35 反成尾巴 +1.24 过推的主力。 */
        }

        if (u > 140.0f)  { u = 140.0f;  }
        if (u < -140.0f) { u = -140.0f; }
        bc_set_units(u);
        /* 真停判据 = 双轮均 <0.5rpm **持续 250ms**：低速编码器脉冲稀疏，
         * 两个脉冲之间瞬时读数就是 0 —— 首版单次判过，coast 一拍就交班，
         * 球照样被甩 -2.98（08-01 实测）。ok_since 复用作静止计时。
         * 1.5s 兜底防编码器脏数据卡死 */
        if (meas <= 5)
        {
            if (0U == ok_since)
            {
                ok_since = now;
            }
        }
        else
        {
            ok_since = 0U;
        }
        if (((0U != ok_since) && ((uint32_t)(now - ok_since) >= 250U)) ||
            ((uint32_t)(now - t_state) > 1500U))
        {
            st = BALL_ST_SETTLE;
            t_state = now;
            ok_since = 0U;
            settle_target = ride_anchor;    /* q6 停车后回锚点而非 0 */
            settle_floor = BC_SETTLE_FLOOR0;
            settle_still = now;
        }
        break;
    }

    case BALL_ST_R_END:
        bc_hold_tick(x, v);                 /* 同 HOLD_END：无限保持到复位 */
        break;

    default:
        break;
    }
}
