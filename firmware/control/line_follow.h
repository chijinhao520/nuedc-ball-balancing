/***************************************************************************//**
 * @file    line_follow.h
 * @brief   6 路巡线传感器循迹 + 停车线检测跑圈（Algorithm 层，不碰寄存器）
 *
 * 输入：App 层统一采集的 Line6 快照（state + analog + threshold，10ms 一帧）。
 * 输出：**双轮目标转速**（rpm）→ SpeedControl，不再直写 PWM。
 *
 * ===== 2026-07-30 传感器切换：8 路数字灰度 → Hiwonder 6 路 I2C（带模拟量）=====
 * 切换动机是打掉数字量的量化天花板：旧 8 路只有 0/1，err 步长 2，单帧量化跳变
 * 经微分低通后 ≈60 单位/s，Kd 超过 0.5 就顶满差速限幅 —— 蛇形振幅压不下去，
 * 而蛇形正是整圈路径长度浮动 ±26cm 的来源（07-30 停车标定实测）。
 * 新误差 = **(analog − threshold) 正余量加权质心**，连续量、亚探头分辨率：
 *   - 量化天花板消失，Kd 的可用范围打开；
 *   - 阈值毛刺天然被压制（实测在线通道余量 +943 vs 毛刺通道 +99，权重比 10:1，
 *     数字量下两者等权）。
 * 旧 8 路版本备份于 line_follow.c.gray8-20260730.bak（工程 .git 为空，防丢）。
 *
 * 控制链：
 *   Line6 快照 → 余量加权质心 err → PD+曲率前馈 → 轮速差 diff(rpm)
 *                                ↓
 *            里程(编码器) → base_rpm（斜坡启动 / 弯道降速 / 末段减速）
 *                                ↓
 *            m1 = base + sign·diff, m2 = base − sign·diff → SpeedControl
 *
 * ===== 为什么输出 rpm 而不是 duty（2026-07-30 架构改动） =====
 * 旧版直写 motor_set(duty) 有三个致命缺陷，都卡在题面第②项上：
 *   ① duty→速度非线性且随电池电压/地面阻力漂移 → 一圈耗时不可预测，
 *      而②项要求「一圈 ≤20s」必须能按周长/速度算时间账；
 *   ② 无法与里程闭环共用一套减速逻辑 → 「停车偏差 ≤2cm」做不到；
 *   ③ 与 SpeedControl / HeadingHold 互斥，已整定好的 Kp=2.00/Ki=16.00 白放着。
 * 改为输出 rpm 后，循迹变成速度环的**上层**，与 heading_hold 平级同构。
 *
 * ===== 赛道账（参数取值的依据，改赛道必须重算） =====
 *   周长 6.14 m = 2×1.5m 直线 + 2×半圆(r=0.5m)；黑线宽 1.8cm
 *   轮周长 0.2227 m（ENC_COUNTS_PER_METER=7005 实测反推）
 *   ②项 ≤20s → 需平均 0.31 m/s = 83 rpm
 *   半圆过弯差速需求仅 ±11 rpm（ω=v/r=0.52rad/s，轮距≈0.16m）
 *     → r=0.5m 相对轮距是**大半径弯**，弯道本身不难，难的是十字不误判 + 停车精度
 *   停车线通过时长 = 1.8cm / 0.31m/s = 58ms ≈ 6 帧（10ms 采集），够 2 帧确认
 *
 * ===== 未知量（实测对不上就翻，不要先改 PID） =====
 * - 极性**无歧义**：余量判据 (analog − threshold > 0 = 压黑线) 07-30 已实测钉死
 *   6/6 通道，旧 8 路的 LINE_ACTIVE_HIGH 开关随之删除；
 * - LINE_TURN_SIGN：换 6 路模块后「通道 1 在物理左还是右」重新变成未知量
 *   （旧 −1 是对旧 8 路 bit 序实测的，**不能继承**）。仍用 CLI 'l t <±1>'
 *   在线翻转实测钉死；装车后第一趟低速点动必测；
 * - 新模块装车位置/高度改变后必须**重新按键学习校准**（阈值随高度变）。
 ******************************************************************************/

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>
#include "line6.h"      /* 输入快照类型 Line6_Data_t（Algorithm 依赖 Driver 头，同 motor.h 先例） */

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_UPDATE_PERIOD_MS      (10U)   /* 与 scheduler.c 的 Task_LineFollow 周期一致 */

/* —— 赛道常数 —— */
/* `l r <cm>` 的 lap 参数现在是**兜底上限**而不是停车位置（停车由灰度触发决定），
 * 所以取值要**大于**真实圈长：几何中心线是 6140mm，但实测灰度触发点在
 * 5689~5956mm 浮动（蛇形路径长度不可重复），配 APPROACH_MM=900 时
 * 6200 让窗口覆盖 5300~6200mm，两端各留余量。 */
#define LINE_LAP_DEFAULT_MM        (6200)
#define LINE_LAP_MIN_MM            (500)
#define LINE_LAP_MAX_MM            (20000)
/* 距一圈还剩这么多时降速并开启启停线识别。
 * 07-30 演进：600 →（漏检启停线）→ 1200 →（减速段太长吃时间）→ 900。
 * 里程只承担**粗定位**（开门），门太窄会漏掉启停线退化成里程兜底停车；门太宽
 * 则低速段行程长、直接吃掉一圈 20s 的余量。900mm 是这两者的折中：实测启停线
 * 在 5956mm，配 lap≈620cm 时窗口 5300~6200mm，两端各留 250mm 余量。 */
#define LINE_APPROACH_MM           (900)
#define LINE_OVERSHOOT_MM          (800)   /* 超出一圈这么多仍未见停车线 → 兜底刹车 */

/**
 * @brief 检出启停线后**继续前进**的距离（mm），走完才刹车。
 *
 * @details 这是把「绝对基准」和「几何偏移」解耦的关键设计：
 *   - 灰度触发消除**累积误差**——触发时刻阵列物理压在启停线上，与整圈实走路径
 *     长度无关（蛇形振荡让路径浮动 ±2cm 级，见 CROSS 阈值注释）；
 *   - 但触发时车身相对启停线的位置是固定几何量（阵列装在车头前/后 X cm），
 *     判分点若不是阵列位置，就差一个**常量** —— 用这段外推补掉。
 * 外推段只有十几~几十厘米、在直线段上、无弯道无蛇形累积，里程误差仅 1~2mm，
 * 所以补偿本身不会把灰度挣来的精度还回去。
 * 精度合计 ≈ 触发延迟(30ms×车速) + 外推里程误差(~2mm) + brake 距离(~5mm)。
 * 运行时可调（CLI 'l d <mm>'）——判分点理解一变，只改这个数。
 */
#define LINE_STOP_ADVANCE_MM       (300)
#define LINE_STOP_ADVANCE_MAX_MM   (1000)

/* —— 速度（rpm）——
 * 2026-07-30 整套速度参数由实测标定，达标记录：**一圈 16.1s（要求 ≤20s）+ 停在终点线上**。
 * 时间账演进（一圈 6.14m，轮周长 0.2227m）：
 *   cruise=40  → 41.2s ❌   cruise=85 → 22.9s ❌   cruise=110 → 17.9s ✅
 *   再把 park 45→80、APPROACH_MM 1200→900 → **16.1s**（余量 3.9s）
 * ⚠️ 提速的前置条件是先放开 LINE_DIFF_ABS_MAX_RPM（见该宏注释）——85rpm 时
 * df 已饱和，不修限幅直接提速只会让纠偏退化成开环。 */
#define LINE_CRUISE_DEFAULT_RPM    (110)
/* APPROACH/ADVANCE 段速度 = 0.296m/s。停车精度 ≈ 触发延迟 30ms×0.296 = 8.9mm
 * + brake 距离约 5mm ≈ 1.5cm，在 2cm 要求内。再提速会让 brake 距离吃掉余量。 */
#define LINE_PARK_DEFAULT_RPM      (80)
#define LINE_CRUISE_MIN_RPM        (40)    /* = HEADING_BASE_MIN_RPM，速度环可控下限对齐 */
#define LINE_CRUISE_MAX_RPM        (180)
#define LINE_INNER_MIN_RPM         (30)    /* 内轮不得低于此：速度环堵转检测下界，再低进盲区 */
/* 轮速差绝对上限。**2026-07-30 由 45 上调到 60**：85rpm 整圈实测 df10 负侧顶到
 * −450（=45rpm 限幅），表现为纠偏进入开环 —— 振幅由 ±3 涨到 −4、穿零波长由
 * 190mm 拉长到 372mm。核算：err=−4 时 Kp 项即 24rpm，加 Kff(6.6) 与 Kd 后约
 * 36rpm，瞬态很容易顶满；而弯道的**几何**差速需求只有 13rpm —— 饱和是动态纠偏
 * 吃掉的，不是弯道稳态需求。提速到 110rpm 时纠偏+弯道稳态合计约 51rpm，故取 60。 */
#define LINE_DIFF_ABS_MAX_RPM      (60)
#define LINE_RAMP_RPM_PER_S        (150)   /* 起步斜坡：约 0.4s 从 MIN 爬到 60rpm */

/* —— 停车线判据 ——
 * ⚠️ 2026-07-30 实地看过赛道后修正。原设计假设启停线是一条均匀横实线（正过时盖
 * 7~8 通道，阈值 6 余量极大）。实际形态是**非均匀**的：
 *   纵向循迹线正常（实线 1.8cm）；横向停车线 = 中间一小段粗实线（较短，跨在
 *   循迹线上） + 向两侧延伸的**细虚线**。
 * 两侧虚线仍落在阵列覆盖范围内（阵列横向约 8~10cm），但压到实线段还是落进空隙
 * 取决于**虚线相位**，是随机量。故过线瞬间 active_count 的构成是：
 *   中间粗段（确定 2~3 通道） + 两侧虚线（随机 0~3 通道） = 4~6，不稳定
 * 而正常循迹只有 1~2 通道。
 *
 * ===== 阈值的两次修订（实测驱动，勿凭直觉改回去）=====
 * 题面几何（_h_plan.txt:545）：A 点启停线仅 **5cm** 垂直段 + 30cm×**0.1cm** 虚线。
 * 按实测反推的探头间距约 1.6cm（5cm 盖 3 个探头、1.8cm 环线盖 1~2 个，两条吻合），
 * 启停线最多盖 3 个探头；那条 1mm 虚线远小于探头光斑，根本不可能触发 ——
 * 它是**给裁判量偏差用的基准线**，不是给车检测的。
 *
 * 07-30 演进：4 →（弯道斜压误触发）→ 7 →（漏检启停线）→ **拆成两个参数**，
 * 因为两个角色的阈值需求相反：
 *   - CROSS（停车线触发）需要 **3** —— 旧 8 路整圈实测 a=3 稳定出现且只在启停线处
 *     （a 分布 1:65 / 2:111 / 3:2，三趟一致）。
 *   - BLIND（宽黑区异常保护）需要**全通道**级别，只在模块故障/整车压进黑区触发。
 * CROSS=3 的误触发风险由**里程门**消掉：识别只在最后 LINE_APPROACH_MM 内开启。
 *
 * ⚠️ 换 6 路模块（07-30）的连带修订：
 *   - BLIND 7→**6**：只有 6 个通道，7 永远达不到 = 保护形同虚设；
 *   - 探头间距变为约 14.7mm（88mm/6），5cm 启停线粗段盖 3~4 个探头、1.8cm 循迹线
 *     盖 1~2 个 → CROSS=3 的区分度依旧成立，但**须装车后实测复核**（'l x' 可在线调）。
 *
 * ===== 为什么停车必须回到灰度（不能纯里程）=====
 * 里程是累积量，而实走路径长度本身不可重复：实测蛇形振幅 A≈3cm、波长 λ=37.2cm，
 * 正弦弧长增长 = (2πA/λ)²/4 = 6.4% → 整圈多走 **39cm**，且振幅/初相位每趟都不同。
 * 实证：指令圈长 614→556（减 58cm），车头结果 +58→−30（变 88cm），增益 1.52≠1,
 * 说明随机误差在 30cm 量级 —— 比 2cm 要求差一个数量级，标定无法消除。
 * 启停线是**绝对位置**，检到即无累积误差，精度只由检测延迟决定（约 1cm）。 */
#define LINE_CROSS_MIN_ACTIVE      (3U)   /* 停车线触发：绝对位置基准 */
#define LINE_BLIND_MIN_ACTIVE      (6U)   /* 宽黑区异常保护 = 6 通道全亮 */
/* 停车/宽黑区计数的**余量门槛**（analog−threshold ≥ 此值才计入）——模拟量对停车
 * 检测的核心贡献，2026-07-30 静置实测直接给出取值区间：
 *   真停车线（十字粗段+虚线）：余量 403/1120/422/80/41/7 → ≥200 的有 **3** 个
 *   直线+阈值毛刺：           余量 168/44/1008/81       → ≥200 的只有 **1** 个
 * 二值计数（margin>0）在后者数出 a=4，进 APPROACH 段就误停；200 取毛刺上限(+168)
 * 与虚线下限(+403)的几何中间。仅用于停车/宽黑区计数——**循迹质心不设门槛**，
 * 小余量通道的信息由加权自然稀释，硬切掉反而丢分辨率。CLI 'l x <n> <margin>' 可调。 */
#define LINE_CROSS_MARGIN_MIN      (200)
#define LINE_CROSS_CONFIRM_FRAMES  (2U)    /* 连续 N 帧确认，滤单帧毛刺（20ms → 6mm@0.31m/s） */
#define LINE_BLIND_TIMEOUT_MS      (400U)  /* active≥阈值持续超此 = 模块故障/宽黑区，非停车线 */
#define LINE_LOST_TIMEOUT_MS       (300U)  /* 全丢线容忍窗；窗内靠边缘外插继续走，超则报错 */
#define LINE_LAP_TIMEOUT_MS        (60000U)

/* —— 控制增益默认值（×100 定点由 CLI 传入，内部 float）—— */
/* Kp=6.0：err=1（偏半通道≈0.6cm）→ diff 6rpm → 转向半径 2.16m@0.3m/s，
 *          纠正 0.6cm 偏差需走 0.16m ≈ 0.53s，不激进也不迟钝 */
#define LINE_KP_X100_DEFAULT       (600)
/* Kd=0.50 —— 旧 8 路上按 1.5Hz 极限环标定（Kd≈Kp/ω=0.64，因量化噪声退档 0.50，
 * 实测波长 100→190mm、整圈零丢线）。
 * 换 6 路模拟量后**量化天花板已拆**（err 连续，不再有步长 2 的单帧跳变），
 * Kd 的可用上限打开 —— 若装车实测仍有残余蛇形，优先加 Kd（0.6~0.8）而不是动 Kp。
 * 注：D 项对空间波长的阻尼作用 ∝ 车速（derr/dt = derr/ds · v），提速时自动增强。 */
#define LINE_KD_X100_DEFAULT       (50)
/* Kff=2.0：弯道稳态时 err_lp≈err，等价把弯道 Kp 从 6 抬到 8，消除恒定曲率稳态误差。
 * 实测佐证：整圈 lp10 稳定偏负（−21~+8）——跑道形赛道两个半圆**同向**，
 * 51% 行程都在往同一边转，误差慢低通持续偏一侧是几何必然，不是 bug。 */
#define LINE_KFF_X100_DEFAULT      (200)
/* Kslow=3.0 —— 07-30 定案（4.0 → 6.0 → 3.0）。
 * 6.0 时实测把弯道从 85 压到 65rpm，过度保守、白吃掉 2.5s；3.0 时弯道约
 * 110−3.0×3.3 ≈ 100rpm，配 cruise=110 得到 16.1s。
 * ⚠️ 仅当 cruise > LINE_CRUISE_MIN_RPM 时有效——cruise=40 时降速目标会被
 * clamp 回 40（实测 base 全程恒 40），低速调试时看不到它起作用。 */
#define LINE_KSLOW_X100_DEFAULT    (300)

typedef enum
{
    LINE_FOLLOW_STATE_DISABLED = 0,
    LINE_FOLLOW_STATE_NORMAL,          /* 单线段，正常循迹 */
    LINE_FOLLOW_STATE_MULTI,           /* 多不连续线段，选最接近上次的一段 */
    LINE_FOLLOW_STATE_EXTRAP,          /* 全丢线，靠上次符号外插继续走 */
    LINE_FOLLOW_STATE_WIDE             /* 多通道压线（停车线候选 / 宽黑区） */
} LineFollowState_t;

typedef enum
{
    LINE_LAP_IDLE = 0,
    LINE_LAP_RAMP,                     /* base 斜坡爬升到 cruise */
    LINE_LAP_CRUISE,                   /* 巡航；十字识别**关闭**（免疫沿途标记误触发） */
    LINE_LAP_APPROACH,                 /* 里程进门：降速到 park_rpm，启停线识别开启 */
    LINE_LAP_ADVANCE,                  /* 已检出启停线，正走完 LINE_STOP_ADVANCE_MM */
    LINE_LAP_DONE,                     /* 已刹车停在目标位置 */
    LINE_LAP_FAULT
} LineLapState_t;

typedef enum
{
    LINE_FAULT_NONE = 0,
    LINE_FAULT_LOST,                   /* 丢线超 LINE_LOST_TIMEOUT_MS */
    LINE_FAULT_BLIND,                  /* 多通道压线超 LINE_BLIND_TIMEOUT_MS（模块故障？） */
    LINE_FAULT_SPEED,                  /* 速度环故障（堵转/编码器） */
    LINE_FAULT_OVERSHOOT,              /* 超一圈 +OVERSHOOT 还没停下 = 里程链也失效了 */
    LINE_FAULT_TIMEOUT,
    LINE_FAULT_NO_LINE_AT_START        /* 使能瞬间就没线，拒绝起步 */
} LineFollowFault_t;

/**
 * @brief 停车触发来源。**这是判断灰度停车线到底能不能用的唯一实证**——
 *        连跑几圈看 sr=，全是 ODO 就说明那段短粗线测不到，别再在它上面花时间。
 */
typedef enum
{
    LINE_STOP_NONE = 0,
    LINE_STOP_CROSS,                   /* 灰度检出停车线（更准，优先） */
    LINE_STOP_ODO,                     /* 里程到位（主判据） */
    LINE_STOP_MANUAL,                  /* 人工中止（'l 0' / 's' / 'b' / 切别的闭环） */
    LINE_STOP_FAULT
} LineStopReason_t;

/** @brief 初始化：复位 PID/状态机，增益取本文件默认值。上电调一次。 */
void LineFollow_Init(void);

/**
 * @brief 纯循迹模式（调参用）：只循迹，不做里程门、不检停车线、不自动停。
 * @param on 1=接管电机 0=停车并交还 CLI 手动控制
 * @details 等价于 LineFollow_StartLap(0, 当前 cruise_rpm)。签名保持 void 以兼容
 *          car_main.c 里 5 处 `LineFollow_SetEnable(0)` 互斥调用点；需要知道启动
 *          是否成功时用 LineFollow_StartLap 的返回值。
 */
void LineFollow_SetEnable(uint8_t on);

/**
 * @brief 跑圈模式（题面第②项）：循迹走 lap_mm 后进入停车线搜索并精确停车。
 * @param lap_mm     一圈周长 mm；传 0 = 纯循迹不停车（同 SetEnable(1)）
 * @param cruise_rpm 巡航速度 rpm，范围 LINE_CRUISE_MIN_RPM..LINE_CRUISE_MAX_RPM
 * @return 0=已启动，1=参数非法 / 起步无线 / 速度环拒绝
 */
uint8_t LineFollow_StartLap(int32_t lap_mm, int32_t cruise_rpm);

/** @brief 立即中止：刹车 + 停速度环 + 复位状态机。 */
void LineFollow_Abort(void);

/**
 * @brief 调度任务周期调用（10ms）：余量加权质心 → PD → 下发轮速。
 * @param frame App 层最近一次成功采集的 Line6 快照（threshold 由采集层低频刷新）
 * @param fresh 1=本快照在新鲜窗内；0=数据过期（I2C 失败/熔断中）。
 *              过期时**按丢线路径处理**：外插继续走 + LOST 超时刹车 ——
 *              传感器猝死与物理丢线共用同一条保护，不给"拿着旧数据裸奔"留口子。
 */
void LineFollow_Update(const Line6_Data_t *frame, uint8_t fresh);

/* —— 在线调参（避免每调一次参数都要重编译烧录）—— */
uint8_t LineFollow_SetGains(int32_t kp_x100, int32_t kd_x100, int32_t kff_x100);
uint8_t LineFollow_SetSpeeds(int32_t cruise_rpm, int32_t park_rpm, int32_t kslow_x100);
uint8_t LineFollow_SetCrossMinActive(int32_t min_active);
/** @brief 停车/宽黑区计数的余量门槛，范围 0..2000（见 LINE_CROSS_MARGIN_MIN） */
uint8_t LineFollow_SetCrossMarginMin(int32_t margin);
int32_t LineFollow_GetCrossMarginMin(void);
/** @brief 余量 ≥ 门槛的通道数（停车判据实际看的数；[lf] 遥测的 a= 即此值） */
uint8_t LineFollow_GetStrongCount(void);
/** @brief 设检出启停线后的外推距离（mm），0..LINE_STOP_ADVANCE_MAX_MM。 */
uint8_t LineFollow_SetStopAdvance(int32_t mm);
int32_t LineFollow_GetStopAdvance(void);
uint8_t LineFollow_SetTurnSign(int32_t sign);
/* （SetActiveHigh 已删：余量判据 analog>threshold 无极性歧义，07-30 实测钉死） */
void    LineFollow_GetGainsX100(int32_t *kp, int32_t *kd, int32_t *kff, int32_t *kslow);

/* —— 遥测 —— */
uint8_t           LineFollow_IsEnabled(void);
LineFollowState_t LineFollow_GetState(void);
LineLapState_t    LineFollow_GetLapState(void);
LineFollowFault_t LineFollow_GetFault(void);
LineStopReason_t  LineFollow_GetStopReason(void);
/**
 * @brief 停车瞬间的里程（mm）。**这就是「这台车这条赛道的一圈实测里程」**，
 *        回填给下一次 'l r <cm>' 用，比几何算的 6.14m 准（车弯道会内切/外切）。
 * @details 人工中止（'l 0' / 'b'）也会记录，这是**标定圈长的主要手段**：
 *          'l 1' 纯循迹跑一圈，人在停车线处敲 'b'，读这个值即得实测圈长。
 *          第一圈灰度未必能触发，所以不能只依赖 CROSS 来拿这个数。
 */
int32_t           LineFollow_GetStopMm(void);
uint8_t           LineFollow_GetActiveCount(void);
uint8_t           LineFollow_GetSegmentCount(void);
int16_t           LineFollow_GetLastError10(void);   /* 质心误差 ×10 */
int16_t           LineFollow_GetErrLp10(void);       /* 误差慢低通 ×10（曲率指标） */
int16_t           LineFollow_GetDiffRpm10(void);     /* 轮速差 ×10 */
int32_t           LineFollow_GetBaseRpm(void);
int32_t           LineFollow_GetTravelMm(void);      /* 本圈已走里程 */
uint32_t          LineFollow_GetCrossCount(void);    /* 累计确认的停车线次数 */
int32_t           LineFollow_GetCruiseRpm(void);
int32_t           LineFollow_GetParkRpm(void);

/**
 * @brief 稳球速度剖面（H 题④⑤：车载钢球行驶）。
 * @param ramp_rpm_per_s 升速斜坡率；0 = 恢复默认 LINE_RAMP_RPM_PER_S(150)
 * @param symmetric      1 = 降速也按同斜坡走（默认降速瞬跳，只为循迹安全；
 *                       但瞬跳的减速度会把球甩出中心区，稳球时必须对称限斜坡）
 * @note 摆杆权限 ±45 units 只能补偿约 0.26 m/s² 车加速度，换算斜坡率
 *       ≈78 rpm/s —— ④⑤ 用 75 以内。设置持续有效，跑③或普通循迹前记得清。
 */
void              LineFollow_SetRamp(int32_t ramp_rpm_per_s, uint8_t symmetric);
/** @brief 规划加速度标志：+1 起步斜坡爬升 / -1 缓停降速 / 0 平（含低速台阶）。
 *  球前馈的唯一信号源 —— 从 base 差分猜会把巡航段 kslow 噪声当满斜坡。 */
int8_t            LineFollow_GetPlanAccel(void);
uint8_t           LineFollow_GetCrossMinActive(void);
int8_t            LineFollow_GetTurnSign(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOW_H */
