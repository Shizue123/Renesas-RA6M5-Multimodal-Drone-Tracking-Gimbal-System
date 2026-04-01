/**
 * @file st3215_driver.c
 * @brief 飞特 ST-3215 串行总线舵机驱动实现
 */

#include "st3215_driver.h"
#include <string.h>

/* 回调函数指针 */
static ST3215_TxCallback_t s_tx_callback = NULL;
static ST3215_DirCallback_t s_dir_callback = NULL;
static ST3215_DelayCallback_t s_delay_callback = NULL;

/* 接收缓冲区与状态机 */
#define RX_BUF_SIZE 32
static uint8_t s_rx_buffer[RX_BUF_SIZE];
static uint8_t s_rx_index = 0;

/* 响应解析状态机 */
typedef enum {
    RX_WAIT_HEADER1 = 0,
    RX_WAIT_HEADER2,
    RX_WAIT_ID,
    RX_WAIT_LEN,
    RX_RECV_BODY,     /* 接收 Error + Data + Checksum */
} ST3215_RxState_e;

static volatile ST3215_RxState_e s_rx_state = RX_WAIT_HEADER1;
static volatile uint8_t s_rx_pkt_id = 0;
static volatile uint8_t s_rx_pkt_len = 0;    /* Length 字段值 (包含 Error + Params + Checksum) */
static volatile uint8_t s_rx_body_idx = 0;

/* 最近一次有效响应缓存 (双缓冲: ISR 写入 pending, 主循环读取 latest) */
static volatile uint8_t  s_resp_id = 0;
static volatile uint8_t  s_resp_error = 0xFF;
static volatile uint8_t  s_resp_data[8];
static volatile uint8_t  s_resp_data_len = 0;
static volatile bool     s_resp_ready = false;

static void clear_response_state(void)
{
    s_resp_id = 0;
    s_resp_error = 0xFF;
    s_resp_data_len = 0;
    s_resp_ready = false;
}

/**
 * @brief 计算校验和
 */
static uint8_t calculate_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 2; i < len - 1; i++) {
        sum += data[i];
    }
    return ~sum;
}

static uint16_t clamp_u16(uint16_t value, uint16_t min, uint16_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static uint16_t apply_servo_safety_limit(uint8_t id, uint16_t target)
{
    /* 强制安全基线: 舵机1(Tilt/上方) 仅允许 [0,2457] */
    if (id == GIMBAL_TILT_SERVO_ID)
    {
        return clamp_u16(target, TILT_SERVO_SAFE_MIN, TILT_SERVO_SAFE_MAX);
    }
    return target;
}

/**
 * @brief 发送数据包
 */
static void send_packet(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_len)
{
    if (s_tx_callback == NULL) return;
    
    uint8_t packet[64];
    uint8_t total_len = 6 + param_len;
    
    /* 构造数据包 */
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = param_len + 2;
    packet[4] = cmd;
    
    if (params != NULL && param_len > 0) {
        memcpy(&packet[5], params, param_len);
    }
    
    packet[total_len - 1] = calculate_checksum(packet, total_len);
    
    /* 切换到发送模式 */
    if (s_dir_callback != NULL) {
        s_dir_callback(true);
    }
    
    /* 发送数据 */
    s_tx_callback(packet, total_len);
    
    /* 延时后切换到接收模式 */
    if (s_delay_callback != NULL) {
        s_delay_callback(500);
    }
    
    if (s_dir_callback != NULL) {
        s_dir_callback(false);
    }
}

void ST3215_Init(ST3215_TxCallback_t tx_cb, 
                 ST3215_DirCallback_t dir_cb,
                 ST3215_DelayCallback_t delay_cb)
{
    s_tx_callback = tx_cb;
    s_dir_callback = dir_cb;
    s_delay_callback = delay_cb;
    s_rx_index = 0;
    s_rx_state = RX_WAIT_HEADER1;
    s_rx_pkt_id = 0;
    s_rx_pkt_len = 0;
    s_rx_body_idx = 0;
    clear_response_state();
}

void ST3215_SetPosition(uint8_t id, uint16_t position, uint16_t time)
{
    uint8_t params[5];
    uint16_t limited_pos = apply_servo_safety_limit(id, position);
    
    params[0] = ST3215_ADDR_TARGET_POSITION;
    params[1] = (uint8_t)(limited_pos & 0xFF);
    params[2] = (uint8_t)((limited_pos >> 8) & 0xFF);
    params[3] = (uint8_t)(time & 0xFF);
    params[4] = (uint8_t)((time >> 8) & 0xFF);
    
    send_packet(id, ST3215_CMD_WRITE, params, 5);
}

void ST3215_SyncSetPosition(const uint8_t *ids, const uint16_t *positions, 
                            uint8_t count, uint16_t time)
{
    if (ids == NULL || positions == NULL || count == 0) return;
    
    uint8_t params[64];
    uint8_t idx = 0;
    
    /* Sync Write 参数: 起始地址, 数据长度 */
    params[idx++] = ST3215_ADDR_TARGET_POSITION;
    params[idx++] = 4; /* 每个舵机4字节: 位置(2) + 时间(2) */
    
    /* 组装每个舵机的数据 */
    for (uint8_t i = 0; i < count; i++) {
        uint16_t limited_pos = apply_servo_safety_limit(ids[i], positions[i]);
        params[idx++] = ids[i];
        params[idx++] = (uint8_t)(limited_pos & 0xFF);
        params[idx++] = (uint8_t)((limited_pos >> 8) & 0xFF);
        params[idx++] = (uint8_t)(time & 0xFF);
        params[idx++] = (uint8_t)((time >> 8) & 0xFF);
    }
    
    send_packet(0xFE, ST3215_CMD_SYNC_WRITE, params, idx);
}

void ST3215_SetTorque(uint8_t id, bool enable)
{
    uint8_t params[2];
    
    params[0] = ST3215_ADDR_TORQUE_ENABLE;
    params[1] = enable ? 0x01 : 0x00;
    
    send_packet(id, ST3215_CMD_WRITE, params, 2);
}

uint16_t ST3215_ReadPosition(uint8_t id)
{
    uint8_t params[2];
    
    params[0] = ST3215_ADDR_PRESENT_POSITION;
    params[1] = 2; /* 读取2字节 */
    
    /* 清除上次响应, 准备接收新响应 */
    clear_response_state();
    
    send_packet(id, ST3215_CMD_READ, params, 2);
    
    /* 等待响应 (最多 5ms, 1Mbps 下足够几十字节往返) */
    for (uint16_t i = 0; i < 5000; i++)
    {
        if (s_resp_ready && s_resp_id == id && s_resp_error == 0x00 && s_resp_data_len >= 2)
        {
            return (uint16_t)s_resp_data[0] | ((uint16_t)s_resp_data[1] << 8);
        }
        if (s_delay_callback != NULL)
        {
            s_delay_callback(1);
        }
    }
    
    return 0xFFFF;
}

bool ST3215_Ping(uint8_t id)
{
    clear_response_state();

    send_packet(id, ST3215_CMD_PING, NULL, 0);

    for (uint16_t i = 0; i < 4000U; i++)
    {
        if (s_resp_ready && (s_resp_id == id) && (s_resp_error == 0x00))
        {
            return true;
        }

        if (s_delay_callback != NULL)
        {
            s_delay_callback(1U);
        }
    }

    return false;
}

bool ST3215_GetLastPosition(uint8_t id, uint16_t *out_position)
{
    if (!s_resp_ready || s_resp_id != id || s_resp_error != 0x00 || s_resp_data_len < 2)
    {
        return false;
    }
    if (out_position != NULL)
    {
        *out_position = (uint16_t)s_resp_data[0] | ((uint16_t)s_resp_data[1] << 8);
    }
    return true;
}

void ST3215_ParseByte(uint8_t byte)
{
    switch (s_rx_state)
    {
        case RX_WAIT_HEADER1:
            if (byte == 0xFF)
            {
                s_rx_state = RX_WAIT_HEADER2;
            }
            break;

        case RX_WAIT_HEADER2:
            if (byte == 0xFF)
            {
                s_rx_state = RX_WAIT_ID;
            }
            else
            {
                s_rx_state = RX_WAIT_HEADER1;
            }
            break;

        case RX_WAIT_ID:
            s_rx_pkt_id = byte;
            s_rx_state = RX_WAIT_LEN;
            break;

        case RX_WAIT_LEN:
            s_rx_pkt_len = byte;
            s_rx_body_idx = 0;
            s_rx_index = 0;
            if (s_rx_pkt_len < 2 || s_rx_pkt_len > RX_BUF_SIZE)
            {
                /* 非法长度 */
                s_rx_state = RX_WAIT_HEADER1;
            }
            else
            {
                s_rx_state = RX_RECV_BODY;
            }
            break;

        case RX_RECV_BODY:
            if (s_rx_body_idx < RX_BUF_SIZE)
            {
                s_rx_buffer[s_rx_body_idx] = byte;
            }
            s_rx_body_idx++;

            if (s_rx_body_idx >= s_rx_pkt_len)
            {
                /* 帧接收完毕, 校验 checksum */
                uint8_t sum = s_rx_pkt_id + s_rx_pkt_len;
                for (uint8_t i = 0; i < (uint8_t)(s_rx_pkt_len - 1); i++)
                {
                    sum += s_rx_buffer[i];
                }
                uint8_t expected_chk = ~sum;
                uint8_t actual_chk = s_rx_buffer[s_rx_pkt_len - 1];

                if (expected_chk == actual_chk)
                {
                    /* 校验通过: buffer[0] = Error, buffer[1..N-2] = Data */
                    s_resp_id = s_rx_pkt_id;
                    s_resp_error = s_rx_buffer[0];
                    s_resp_data_len = (uint8_t)(s_rx_pkt_len - 2); /* 排除 Error 和 Checksum */
                    for (uint8_t i = 0; i < s_resp_data_len && i < sizeof(s_resp_data); i++)
                    {
                        s_resp_data[i] = s_rx_buffer[1 + i];
                    }
                    s_resp_ready = true;
                }
                s_rx_state = RX_WAIT_HEADER1;
            }
            break;

        default:
            s_rx_state = RX_WAIT_HEADER1;
            break;
    }
}
