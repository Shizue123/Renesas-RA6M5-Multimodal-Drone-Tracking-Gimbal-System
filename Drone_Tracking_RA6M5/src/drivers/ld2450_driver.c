/**
 * @file ld2450_driver.c
 * @brief HLK-LD2450 毫米波雷达非阻塞式解析驱动实现
 *
 * 设计原则：
 * - 逐字节状态机解析，ISR 安全 (由 uart2_callback 逐字节喂入)
 * - 不依赖任何 FSP API，通过回调解耦
 * - 与 openmv_drv.c 采用相同的 GetData()/is_new 模式
 *
 * 设计完全基于 LD2450 数据手册帧格式自主实现，无外部参考依赖
 */

#include "ld2450_driver.h"
#include "bsp_api.h"
#include <string.h>

/*===========================================================================*/
/*                              状态机枚举                                     */
/*===========================================================================*/

typedef enum {
    LD2450_STATE_IDLE = 0,       /**< 等待帧头 0xAA */
    LD2450_STATE_WAIT_HEADER2,   /**< 等待帧头 0xFF */
    LD2450_STATE_RECV_DATA,      /**< 接收 26 字节有效数据 */
    LD2450_STATE_WAIT_FOOTER1,   /**< 等待帧尾 0x55 */
    LD2450_STATE_WAIT_FOOTER2    /**< 等待帧尾 0xCC */
} LD2450_ParseState_e;

/*===========================================================================*/
/*                              静态变量                                      */
/*===========================================================================*/

/* 使用双缓冲 A/B Buffer 防止中断撕裂，摒弃 __disable_irq() */
static LD2450_Data_t       g_radar_data[2];
static volatile uint8_t    g_radar_write_idx = 0;
static volatile uint8_t    g_radar_read_idx  = 0;
static volatile bool       g_radar_new_data  = false;
static LD2450_Stats_t      g_radar_stats;
static LD2450_ParseState_e g_parse_state;
static uint8_t             g_rx_buffer[LD2450_DATA_LEN];
static uint8_t             g_rx_index;

/** @brief IIR 一阶低通滤波状态 (per-target X / Y) */
static float g_iir_x[LD2450_MAX_TARGETS];
static float g_iir_y[LD2450_MAX_TARGETS];
static bool  g_iir_primed;  /**< 首帧直接用原始值初始化滤波器 */

/*===========================================================================*/
/*                              内部函数                                      */
/*===========================================================================*/

static void reset_parser(void)
{
    g_parse_state = LD2450_STATE_IDLE;
    g_rx_index = 0;
}

/**
 * @brief 解析缓冲区中的 3 个目标数据并存储
 *
 * g_rx_buffer 布局 (26 字节):
 *   [0-1]   : 类型字段 (帧内 byte[2-3])
 *   [2-9]   : 目标 1 (帧内 byte[4-11])
 *   [10-17] : 目标 2 (帧内 byte[12-19])
 *   [18-25] : 目标 3 (帧内 byte[20-27])
 *
 * 每目标 8 字节: [XL XH YL YH SpeedL SpeedH Res1 Res2]
 */
static void parse_and_store(void)
{
    uint8_t valid_count = 0;
    /* 获取当前的写入 Buffer */
    LD2450_Data_t *p_target_buf = (LD2450_Data_t *)&g_radar_data[g_radar_write_idx];

    for (uint8_t i = 0; i < LD2450_MAX_TARGETS; i++)
    {
        const uint8_t *tp = &g_rx_buffer[2 + i * 8]; /* 跳过 2 字节类型字段 */

        uint16_t raw_x     = (uint16_t)tp[0] | ((uint16_t)tp[1] << 8);
        uint16_t raw_y     = (uint16_t)tp[2] | ((uint16_t)tp[3] << 8);
        uint16_t raw_speed = (uint16_t)tp[4] | ((uint16_t)tp[5] << 8);

        /* X: 有符号直接转换 (int16 two's complement, 正=右, 负=左) */
        p_target_buf->targets[i].x = (int16_t)raw_x;

        /* Y: 无符号偏移量编码, 减去 32768 得到实际深度 */
        p_target_buf->targets[i].y = (int16_t)((int32_t)raw_y - 32768);

        /* Speed: 有符号直接转换 (正=远离, 负=靠近) */
        p_target_buf->targets[i].speed = (int16_t)raw_speed;

        /* 有效性: 深度在 0~6000mm 范围内 */
        if (p_target_buf->targets[i].y >= 0 && p_target_buf->targets[i].y <= 6000)
        {
            p_target_buf->targets[i].valid = true;
            valid_count++;

            /* IIR 滤波: 拑平帧间抖动 (仅对有效目标) */
            float raw_xf = (float)p_target_buf->targets[i].x;
            float raw_yf = (float)p_target_buf->targets[i].y;
            if (!g_iir_primed) {
                g_iir_x[i] = raw_xf;
                g_iir_y[i] = raw_yf;
            } else {
                g_iir_x[i] = LD2450_IIR_ALPHA * raw_xf + (1.0f - LD2450_IIR_ALPHA) * g_iir_x[i];
                g_iir_y[i] = LD2450_IIR_ALPHA * raw_yf + (1.0f - LD2450_IIR_ALPHA) * g_iir_y[i];
            }
            p_target_buf->targets[i].x = (int16_t)g_iir_x[i];
            p_target_buf->targets[i].y = (int16_t)g_iir_y[i];
        }
        else
        {
            p_target_buf->targets[i].valid = false;
        }
    }

    p_target_buf->valid_count = valid_count;
    p_target_buf->timestamp   = LD2450_GetTickMs();
    p_target_buf->is_new      = true; // 仅为旧接口兼容
    g_iir_primed              = true;

    g_radar_stats.valid_frames++;
    g_radar_stats.last_receive_time = p_target_buf->timestamp;

    /* Lock-Free 原子翻转 */
    g_radar_read_idx = g_radar_write_idx;       /* 更新可读索引 */
    g_radar_write_idx ^= 1;                     /* 交替写入索引 */
    g_radar_new_data = true;                    /* 标记新数据已产生 */
}

/*===========================================================================*/
/*                              公开 API                                      */
/*===========================================================================*/

void LD2450_Init(void)
{
    memset((void*)g_radar_data, 0, sizeof(g_radar_data));
    g_radar_write_idx = 0;
    g_radar_read_idx = 0;
    g_radar_new_data = false;
    memset(&g_radar_stats, 0, sizeof(g_radar_stats));
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_iir_x, 0, sizeof(g_iir_x));
    memset(g_iir_y, 0, sizeof(g_iir_y));
    g_iir_primed = false;
    reset_parser();
}

/**
 * @brief 逐字节状态机解析 — 由 ISR (uart2_callback) 调用
 *
 * 状态流转:
 * IDLE ─(0xAA)─→ WAIT_HEADER2 ─(0xFF)─→ RECV_DATA ─(×26)─→ WAIT_FOOTER1 ─(0x55)─→ WAIT_FOOTER2 ─(0xCC)─→ parse → IDLE
 *                     │(0xAA→保持)                                  │(失败→IDLE)              │(失败→IDLE)
 */
void LD2450_ParseByte(uint8_t byte)
{
    static uint32_t last_rx_time = 0;
    uint32_t current_time = LD2450_GetTickMs();

    /* 字节级 50ms 超时强制断帧复位保护，防止雷达因为单字丢信落入死锁泥潭 */
    if (g_parse_state != LD2450_STATE_IDLE && (current_time - last_rx_time > 50))
    {
        reset_parser();
    }
    last_rx_time = current_time;

    g_radar_stats.total_bytes++;

    switch (g_parse_state)
    {
        case LD2450_STATE_IDLE:
            if (byte == LD2450_FRAME_HEADER1)
            {
                g_parse_state = LD2450_STATE_WAIT_HEADER2;
            }
            break;

        case LD2450_STATE_WAIT_HEADER2:
            if (byte == LD2450_FRAME_HEADER2)
            {
                g_rx_index = 0;
                g_parse_state = LD2450_STATE_RECV_DATA;
            }
            else if (byte == LD2450_FRAME_HEADER1)
            {
                /* 连续 0xAA，保持等待 0xFF */
            }
            else
            {
                g_radar_stats.error_frames++;
                reset_parser();
            }
            break;

        case LD2450_STATE_RECV_DATA:
            /* 严格的索边界防越界保护 */
            if (g_rx_index < sizeof(g_rx_buffer))
            {
                g_rx_buffer[g_rx_index++] = byte;
            }
            else
            {
                reset_parser();
                break;
            }
            
            if (g_rx_index >= LD2450_DATA_LEN)
            {
                g_parse_state = LD2450_STATE_WAIT_FOOTER1;
            }
            break;

        case LD2450_STATE_WAIT_FOOTER1:
            if (byte == LD2450_FRAME_FOOTER1)
            {
                g_parse_state = LD2450_STATE_WAIT_FOOTER2;
            }
            else
            {
                g_radar_stats.error_frames++;
                reset_parser();
            }
            break;

        case LD2450_STATE_WAIT_FOOTER2:
            if (byte == LD2450_FRAME_FOOTER2)
            {
                /* 完整一帧，解析存储 */
                parse_and_store();
            }
            else
            {
                g_radar_stats.error_frames++;
            }
            reset_parser();
            break;

        default:
            reset_parser();
            break;
    }
}

bool LD2450_GetData(LD2450_Data_t *data)
{
    bool has_new = false;

    if (data == NULL)
    {
        return false;
    }

    /* 无锁设计 (Lock-Free) 替换 __disable_irq() 避免阻塞其他中断 */
    if (g_radar_new_data)
    {
        uint8_t safe_read_idx = g_radar_read_idx; 
        memcpy(data, (void*)&g_radar_data[safe_read_idx], sizeof(LD2450_Data_t));
        g_radar_new_data = false;
        has_new = true;
    }

    return has_new;
}

bool LD2450_HasNewData(void)
{
    return g_radar_new_data;
}

bool LD2450_IsValid(uint32_t current_time_ms)
{
    return ((current_time_ms - g_radar_stats.last_receive_time) < LD2450_TIMEOUT_MS);
}

void LD2450_GetStats(LD2450_Stats_t *stats)
{
    if (stats != NULL)
    {
        /* 统计数据非严格时间线依赖，允许被中断稍微夹断，直接取走以追求极致去阻塞 */
        memcpy(stats, &g_radar_stats, sizeof(LD2450_Stats_t));
    }
}

void LD2450_Reset(void)
{
    LD2450_Init();
}
