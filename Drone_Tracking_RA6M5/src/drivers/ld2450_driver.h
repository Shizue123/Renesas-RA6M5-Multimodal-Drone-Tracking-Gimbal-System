/**
 * @file ld2450_driver.h
 * @brief HLK-LD2450 毫米波雷达非阻塞式解析驱动
 * @version 2.0
 *
 * 硬件说明：
 * - 通信方式：UART (全双工 TTL)
 * - 波特率：256000 bps, 8N1
 * - FSP 实例：g_uart2 (SCI2, P301=RXD2, P302=TXD2)
 *
 * 帧格式（30 字节, Little-Endian）：
 * [0xAA][0xFF][TypeH][TypeL]  [T1×8] [T2×8] [T3×8]  [0x55][0xCC]
 *  ╰── 帧头 ──╯╰─ 类型字 ─╯  ╰────── 3 个目标 ──────╯  ╰── 帧尾 ──╯
 *
 * 每个目标 8 字节（LE）:
 *   [XL][XH][YL][YH][SpeedL][SpeedH][Res1][Res2]
 *   X: int16 (mm), 正=右, 负=左
 *   Y: uint16 - 32768 (mm), 深度距离 (0 ~ 6000)
 *   Speed: int16 (cm/s), 正=远离, 负=靠近
 */

#ifndef LD2450_DRIVER_H
#define LD2450_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/*                              协议常量                                      */
/*===========================================================================*/

#define LD2450_FRAME_SIZE        30U
#define LD2450_FRAME_HEADER1     0xAA
#define LD2450_FRAME_HEADER2     0xFF
#define LD2450_FRAME_FOOTER1     0x55
#define LD2450_FRAME_FOOTER2     0xCC

#define LD2450_MAX_TARGETS       3U
#define LD2450_TIMEOUT_MS        500U

/** @brief IIR 滤波系数: 新值权重, 越小越平滑 (0.0~1.0) */
#define LD2450_IIR_ALPHA         0.3f

/** @brief 帧头帧尾之间的数据长度 (byte[2] ~ byte[27]) */
#define LD2450_DATA_LEN          26U

/*===========================================================================*/
/*                              数据结构                                      */
/*===========================================================================*/

/** @brief 单个雷达目标 */
typedef struct {
    int16_t  x;          /**< X 横向坐标 (mm), 正=右, 负=左 */
    int16_t  y;          /**< Y 深度距离 (mm), 有效范围 0~6000 */
    int16_t  speed;      /**< 速度 (cm/s), 正=远离, 负=靠近 */
    bool     valid;      /**< 目标是否有效 */
} LD2450_Target_t;

/** @brief 雷达全量数据 */
typedef struct {
    LD2450_Target_t targets[LD2450_MAX_TARGETS];
    uint8_t         valid_count;   /**< 有效目标数量 */
    uint32_t        timestamp;     /**< 数据时间戳 (ms) */
    volatile bool   is_new;        /**< 是否有未读取的新数据 */
} LD2450_Data_t;

/** @brief 雷达通信统计 */
typedef struct {
    uint32_t total_bytes;
    uint32_t valid_frames;
    uint32_t error_frames;
    uint32_t last_receive_time;
} LD2450_Stats_t;

/*===========================================================================*/
/*                              API                                          */
/*===========================================================================*/

void  LD2450_Init(void);
void  LD2450_ParseByte(uint8_t byte);
bool  LD2450_GetData(LD2450_Data_t *data);
bool  LD2450_HasNewData(void);
bool  LD2450_IsValid(uint32_t current_time_ms);
void  LD2450_GetStats(LD2450_Stats_t *stats);
void  LD2450_Reset(void);

/** @brief 外部实现: 获取系统毫秒时间戳 (定义在 hal_entry.c) */
extern uint32_t LD2450_GetTickMs(void);

#ifdef __cplusplus
}
#endif

#endif /* LD2450_DRIVER_H */
