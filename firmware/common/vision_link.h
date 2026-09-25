/***************************************************************************//**
 * @file    vision_link.h
 * @brief   RDK X5 视觉链路生产帧解析（car_main / gimbal_main 共享模块）
 *
 * 帧格式与 vision/vision_base/serial_out.py、BSP/x5_parse_bringup 对齐：
 *   [0]=0xAA [1]=0x55 [2]=type [3..4]=steer_i16 little-endian
 *   [5]=flags [6..7]=aux_i16 little-endian [8]=XOR([2..7])
 *
 * 解析器不碰 UART 寄存器。工程侧 RX ISR 逐字节调用 VisionLink_Feed()，
 * 应用任务用 VisionLink_GetLatest() 读取一致快照并独立执行新鲜度门控。
 ******************************************************************************/

#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_LINK_FRAME_LEN       9U
#define VISION_LINK_HDR0            0xAAU
#define VISION_LINK_HDR1            0x55U

#define VISION_TYPE_LINE            1U
#define VISION_TYPE_COLOR           2U
#define VISION_TYPE_DETECT          3U
/* 槽内钢球一维位置（H 题）：steer 字段改载 x_est_cm*100（±1250），
 * aux 载 v*10（0.1cm/s）。发的是 X5 侧 alpha-beta 观测器的平滑输出，
 * 不是原始测量 —— X5 有全精度浮点与真实时间戳，比 MCU 再滤一遍更准。 */
#define VISION_TYPE_BALL            4U
/* type=4 的 steer 量程：管半长 12.5cm → x*100 达 ±1250，留 50 余量 */
#define VISION_BALL_STEER_LIMIT     1300

/** @brief 一帧视觉结果；steer_x1000 范围由发送端限制为 -1000..1000。 */
typedef struct
{
    uint8_t  type;
    int16_t  steer_x1000;
    uint8_t  found;
    int16_t  aux;
    uint32_t update_cnt;
} VisionResult_t;

/** @brief 链路统计（观测/告警用）。 */
typedef struct
{
    uint32_t frames_ok;
    uint32_t bad_checksum;
    uint32_t bad_payload;
    uint32_t resync_bytes;
} VisionLinkStats_t;

/**
 * @brief 喂入一个 UART 字节。
 * @return 1=刚提交一帧有效结果；0=仍在收集或本帧无效。
 */
uint8_t VisionLink_Feed(uint8_t byte);

/**
 * @brief 复制最新一致快照。
 * @return 1=已有有效帧且快照一致；0=尚无有效帧或读取期间连续更新。
 */
uint8_t VisionLink_GetLatest(VisionResult_t *out);

/** @brief 复制当前统计快照。 */
void VisionLink_GetStats(VisionLinkStats_t *out);

/** @brief 复位解析状态、结果与统计。 */
void VisionLink_Reset(void);

/**
 * @brief 喂入乱码、坏校验帧和正确帧，验证重同步、丢弃与小端解析。
 * @return 0=PASS；非 0=失败步骤号。
 */
uint8_t VisionLink_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* VISION_LINK_H */
