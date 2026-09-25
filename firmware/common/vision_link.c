/***************************************************************************//**
 * @file    vision_link.c
 * @brief   RDK X5 视觉链路生产帧解析（AA55 + little-endian + XOR）
 ******************************************************************************/

#include "vision_link.h"
#include <stddef.h>

static volatile VisionResult_t    g_result;
static volatile VisionLinkStats_t g_stats;
static volatile uint32_t g_result_seq;

static uint8_t g_frame[VISION_LINK_FRAME_LEN];
static uint8_t g_idx;

static uint8_t vision_checksum(const uint8_t *frame)
{
    uint8_t checksum = 0U;
    uint8_t index;

    for (index = 2U; index < (VISION_LINK_FRAME_LEN - 1U); index++)
    {
        checksum ^= frame[index];
    }
    return checksum;
}

uint8_t VisionLink_Feed(uint8_t byte)
{
    if (0U == g_idx)
    {
        if (VISION_LINK_HDR0 == byte)
        {
            g_frame[g_idx++] = byte;
        }
        else
        {
            g_stats.resync_bytes++;
        }
        return 0U;
    }

    if (1U == g_idx)
    {
        if (VISION_LINK_HDR1 == byte)
        {
            g_frame[g_idx++] = byte;
        }
        else
        {
            g_stats.resync_bytes++;
            g_idx = 0U;
            if (VISION_LINK_HDR0 == byte)
            {
                g_frame[g_idx++] = byte;
            }
        }
        return 0U;
    }

    g_frame[g_idx++] = byte;
    if (g_idx < VISION_LINK_FRAME_LEN)
    {
        return 0U;
    }
    g_idx = 0U;

    if (vision_checksum(g_frame) != g_frame[VISION_LINK_FRAME_LEN - 1U])
    {
        g_stats.bad_checksum++;
        return 0U;
    }

    {
        const int16_t steer = (int16_t)((uint16_t)g_frame[3] |
                                        ((uint16_t)g_frame[4] << 8));
        /* type=4(球位置) 的 steer 载的是 x_cm*100，量程比转向量大，
         * 故按类型分别设限；其余类型仍是 ±1000 的转向量。 */
        const int16_t lim = (VISION_TYPE_BALL == g_frame[2])
                                ? VISION_BALL_STEER_LIMIT : 1000;
        if ((g_frame[2] < VISION_TYPE_LINE) ||
            (g_frame[2] > VISION_TYPE_BALL) ||
            (steer < -lim) || (steer > lim))
        {
            g_stats.bad_payload++;
            return 0U;
        }

        /* 奇偶序列锁：奇数表示 ISR 正在提交，偶数表示快照稳定。 */
        g_result_seq++;
        g_result.type = g_frame[2];
        g_result.steer_x1000 = steer;
    }
    g_result.found = (uint8_t)(g_frame[5] & 0x01U);
    g_result.aux = (int16_t)((uint16_t)g_frame[6] |
                             ((uint16_t)g_frame[7] << 8));
    g_result.update_cnt++;
    g_result_seq++;
    g_stats.frames_ok++;
    return 1U;
}

uint8_t VisionLink_GetLatest(VisionResult_t *out)
{
    uint8_t attempt;

    if (NULL == out)
    {
        return 0U;
    }

    for (attempt = 0U; attempt < 2U; attempt++)
    {
        const uint32_t before = g_result_seq;

        if ((0U != (before & 1U)) || (0U == g_result.update_cnt))
        {
            return 0U;
        }
        out->type = g_result.type;
        out->steer_x1000 = g_result.steer_x1000;
        out->found = g_result.found;
        out->aux = g_result.aux;
        out->update_cnt = g_result.update_cnt;
        if (before == g_result_seq)
        {
            return 1U;
        }
    }
    return 0U;
}

void VisionLink_GetStats(VisionLinkStats_t *out)
{
    if (NULL != out)
    {
        out->frames_ok = g_stats.frames_ok;
        out->bad_checksum = g_stats.bad_checksum;
        out->bad_payload = g_stats.bad_payload;
        out->resync_bytes = g_stats.resync_bytes;
    }
}

void VisionLink_Reset(void)
{
    g_idx = 0U;
    g_result_seq = 0U;
    g_result.type = 0U;
    g_result.steer_x1000 = 0;
    g_result.found = 0U;
    g_result.aux = 0;
    g_result.update_cnt = 0U;
    g_stats.frames_ok = 0U;
    g_stats.bad_checksum = 0U;
    g_stats.bad_payload = 0U;
    g_stats.resync_bytes = 0U;
}

uint8_t VisionLink_SelfTest(void)
{
    uint8_t frame[VISION_LINK_FRAME_LEN] =
    {
        VISION_LINK_HDR0, VISION_LINK_HDR1, VISION_TYPE_DETECT,
        0xBFU, 0xFEU, 0x01U, 0x19U, 0x00U, 0U
    };
    const uint8_t noise[4] = {0x00U, VISION_LINK_HDR0, 0x42U, 0xFFU};
    VisionResult_t result;
    VisionLinkStats_t stats;
    uint8_t index;

    frame[VISION_LINK_FRAME_LEN - 1U] = vision_checksum(frame);
    VisionLink_Reset();

    for (index = 0U; index < (uint8_t)sizeof(noise); index++)
    {
        (void)VisionLink_Feed(noise[index]);
    }

    frame[VISION_LINK_FRAME_LEN - 1U] ^= 0x01U;
    for (index = 0U; index < VISION_LINK_FRAME_LEN; index++)
    {
        (void)VisionLink_Feed(frame[index]);
    }
    frame[VISION_LINK_FRAME_LEN - 1U] ^= 0x01U;

    frame[2] = 0x7FU;
    frame[VISION_LINK_FRAME_LEN - 1U] = vision_checksum(frame);
    for (index = 0U; index < VISION_LINK_FRAME_LEN; index++)
    {
        (void)VisionLink_Feed(frame[index]);
    }
    frame[2] = VISION_TYPE_DETECT;
    frame[VISION_LINK_FRAME_LEN - 1U] = vision_checksum(frame);
    for (index = 0U; index < VISION_LINK_FRAME_LEN; index++)
    {
        (void)VisionLink_Feed(frame[index]);
    }

    VisionLink_GetStats(&stats);
    if ((1U != stats.frames_ok) || (1U != stats.bad_checksum) ||
        (1U != stats.bad_payload) ||
        (0U == stats.resync_bytes))
    {
        return 1U;
    }
    if ((0U == VisionLink_GetLatest(&result)) ||
        (VISION_TYPE_DETECT != result.type) ||
        (-321 != result.steer_x1000) || (1U != result.found) ||
        (25 != result.aux))
    {
        return 2U;
    }

    VisionLink_Reset();
    return 0U;
}
