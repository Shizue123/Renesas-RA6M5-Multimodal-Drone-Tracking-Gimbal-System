/**
 * @file openmv_drv.c
 * @brief OpenMV 视觉数据接收驱动实现（8字节高精度协议）
 */

#include "openmv_drv.h"
#include "bsp_api.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef enum {
    PARSE_STATE_IDLE = 0,
    PARSE_STATE_HEADER_1,
    PARSE_STATE_STATUS,
    PARSE_STATE_X_H,
    PARSE_STATE_X_L,
    PARSE_STATE_Y_H,
    PARSE_STATE_Y_L,
    PARSE_STATE_CHECKSUM
} ParseState_e;

static VisionData_t  g_vision_data[2];
static volatile uint8_t g_vision_write_idx = 0;
static volatile uint8_t g_vision_read_idx  = 0;
static volatile bool    g_vision_new_data  = false;
static VisionStats_t g_vision_stats;
static ParseState_e  g_parse_state;

static uint8_t g_rx_status;
static uint8_t g_rx_x_h;
static uint8_t g_rx_x_l;
static uint8_t g_rx_y_h;
static uint8_t g_rx_y_l;

static const char * g_status_names[] = {
    "LOST",
    "LOCKED",
    "PREDICTED",
    "HEARTBEAT"
};

static void reset_parser(void)
{
    g_parse_state = PARSE_STATE_IDLE;
    g_rx_status = 0;
    g_rx_x_h = 0;
    g_rx_x_l = 0;
    g_rx_y_h = 0;
    g_rx_y_l = 0;
}

static bool verify_and_store(uint8_t rx_checksum)
{
    uint8_t calculated = (uint8_t) ((OPENMV_FRAME_HEADER_1 + OPENMV_FRAME_HEADER_2 +
                                     g_rx_status + g_rx_x_h + g_rx_x_l + g_rx_y_h + g_rx_y_l) & 0xFF);

    if (calculated != rx_checksum)
    {
        g_vision_stats.checksum_errors++;
        return false;
    }

    if (g_rx_status > (uint8_t) VISION_STATUS_HEARTBEAT)
    {
        g_vision_stats.frame_errors++;
        return false;
    }

    int16_t err_x = (int16_t) (((uint16_t) g_rx_x_h << 8) | (uint16_t) g_rx_x_l);
    int16_t err_y = (int16_t) (((uint16_t) g_rx_y_h << 8) | (uint16_t) g_rx_y_l);

    VisionData_t *p_buf = (VisionData_t *)&g_vision_data[g_vision_write_idx];

    p_buf->status = (VisionStatus_e) g_rx_status;
    p_buf->error_x = err_x;
    p_buf->error_y = err_y;
    p_buf->x = (int16_t) (OPENMV_FRAME_CENTER_X + err_x);
    p_buf->y = (int16_t) (OPENMV_FRAME_CENTER_Y + err_y);
    p_buf->timestamp = OpenMV_GetTickMs();
    
    /* 继承上一帧累加 */
    VisionData_t *p_old = (VisionData_t *)&g_vision_data[g_vision_write_idx ^ 1];
    p_buf->frame_count = p_old->frame_count + 1;
    
    p_buf->is_new = true; // 保持接口兼容

    g_vision_stats.valid_frames++;
    g_vision_stats.last_receive_time = p_buf->timestamp;
    g_vision_stats.comm_status = COMM_STATUS_OK;

    /* Lock-Free 原子翻转 */
    g_vision_read_idx = g_vision_write_idx;
    g_vision_write_idx ^= 1;
    g_vision_new_data = true;

    return true;
}

void OpenMV_Init(void)
{
    memset((void*)g_vision_data, 0, sizeof(g_vision_data));
    g_vision_write_idx = 0;
    g_vision_read_idx  = 0;
    g_vision_new_data  = false;
    memset(&g_vision_stats, 0, sizeof(g_vision_stats));

    g_vision_data[0].status = VISION_STATUS_LOST;
    g_vision_data[0].x = OPENMV_FRAME_CENTER_X;
    g_vision_data[0].y = OPENMV_FRAME_CENTER_Y;
    g_vision_data[1].status = VISION_STATUS_LOST;
    g_vision_data[1].x = OPENMV_FRAME_CENTER_X;
    g_vision_data[1].y = OPENMV_FRAME_CENTER_Y;

    g_vision_stats.comm_status = COMM_STATUS_OK;
    g_vision_stats.last_receive_time = OpenMV_GetTickMs();

    reset_parser();
}

void OpenMV_ParseByte(uint8_t byte)
{
    static uint32_t last_rx_time = 0;
    uint32_t current_time = OpenMV_GetTickMs();

    /* 50ms 超时强制断帧复位保护 */
    if (g_parse_state != PARSE_STATE_IDLE && (current_time - last_rx_time > 50))
    {
        reset_parser();
    }
    last_rx_time = current_time;

    g_vision_stats.total_bytes++;

    switch (g_parse_state)
    {
        case PARSE_STATE_IDLE:
            if (byte == OPENMV_FRAME_HEADER_1)
            {
                g_parse_state = PARSE_STATE_HEADER_1;
            }
            break;

        case PARSE_STATE_HEADER_1:
            if (byte == OPENMV_FRAME_HEADER_2)
            {
                g_parse_state = PARSE_STATE_STATUS;
            }
            else if (byte == OPENMV_FRAME_HEADER_1)
            {
            }
            else
            {
                g_vision_stats.frame_errors++;
                reset_parser();
            }
            break;

        case PARSE_STATE_STATUS:
            g_rx_status = byte;
            g_parse_state = PARSE_STATE_X_H;
            break;

        case PARSE_STATE_X_H:
            g_rx_x_h = byte;
            g_parse_state = PARSE_STATE_X_L;
            break;

        case PARSE_STATE_X_L:
            g_rx_x_l = byte;
            g_parse_state = PARSE_STATE_Y_H;
            break;

        case PARSE_STATE_Y_H:
            g_rx_y_h = byte;
            g_parse_state = PARSE_STATE_Y_L;
            break;

        case PARSE_STATE_Y_L:
            g_rx_y_l = byte;
            g_parse_state = PARSE_STATE_CHECKSUM;
            break;

        case PARSE_STATE_CHECKSUM:
            (void) verify_and_store(byte);
            reset_parser();
            break;

        default:
            g_vision_stats.frame_errors++;
            reset_parser();
            break;
    }
}

bool OpenMV_GetData(VisionData_t *data)
{
    bool has_new = false;

    if (data == NULL)
    {
        return false;
    }

    /* 无锁设计 (Lock-Free)，抛弃 __disable_irq() 避免阻塞外设 */
    if (g_vision_new_data)
    {
        uint8_t safe_read_idx = g_vision_read_idx;
        memcpy(data, (void*)&g_vision_data[safe_read_idx], sizeof(VisionData_t));
        g_vision_new_data = false;
        has_new = true;
    }

    return has_new;
}

bool OpenMV_HasNewData(void)
{
    return g_vision_new_data;
}

VisionStatus_e OpenMV_GetStatus(void)
{
    return g_vision_data[g_vision_read_idx].status;
}

void OpenMV_GetStats(VisionStats_t *stats)
{
    if (stats != NULL)
    {
        /* 去除锁阻塞 */
        memcpy(stats, &g_vision_stats, sizeof(VisionStats_t));
    }
}

CommStatus_e OpenMV_UpdateCommStatus(uint32_t current_time_ms)
{
    if ((current_time_ms - g_vision_stats.last_receive_time) > OPENMV_TIMEOUT_MS)
    {
        g_vision_stats.comm_status = COMM_STATUS_TIMEOUT;
    }
    else
    {
        g_vision_stats.comm_status = COMM_STATUS_OK;
    }

    return g_vision_stats.comm_status;
}

void OpenMV_Reset(void)
{
    OpenMV_Init();
}

const char * OpenMV_GetStatusName(VisionStatus_e status)
{
    if (status <= VISION_STATUS_HEARTBEAT)
    {
        return g_status_names[status];
    }

    return "UNKNOWN";
}

bool OpenMV_IsCommOK(void)
{
    return (g_vision_stats.comm_status == COMM_STATUS_OK);
}
