/**
 * @file openmv_drv.h
 * @brief OpenMV 视觉数据接收驱动（8字节高精度协议）
 * @version 4.0
 *
 * 协议格式（固定8字节，Big-Endian）：
 * [0x55][0xAA][Status][X_H][X_L][Y_H][Y_L][Checksum]
 *
 * 校验和算法：
 * checksum = (0x55 + 0xAA + Status + X_H + X_L + Y_H + Y_L) & 0xFF
 */

#ifndef OPENMV_DRV_H
#define OPENMV_DRV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OPENMV_FRAME_HEADER_1      0x55
#define OPENMV_FRAME_HEADER_2      0xAA
#define OPENMV_FRAME_LENGTH        8U

#define OPENMV_FRAME_CENTER_X      120
#define OPENMV_FRAME_CENTER_Y      120
#define OPENMV_TIMEOUT_MS          500U

typedef enum {
    VISION_STATUS_LOST       = 0,
    VISION_STATUS_LOCKED     = 1,
    VISION_STATUS_PREDICTED  = 2,
    VISION_STATUS_HEARTBEAT  = 3
} VisionStatus_e;

typedef enum {
    COMM_STATUS_OK           = 0,
    COMM_STATUS_TIMEOUT      = 1,
    COMM_STATUS_ERROR        = 2
} CommStatus_e;

typedef struct {
    VisionStatus_e status;
    int16_t x;
    int16_t y;
    int16_t error_x;
    int16_t error_y;
    uint32_t timestamp;
    uint32_t frame_count;
    volatile bool is_new;
} VisionData_t;

typedef struct {
    uint32_t total_bytes;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint32_t last_receive_time;
    CommStatus_e comm_status;
} VisionStats_t;

uint32_t OpenMV_GetTickMs(void);

void OpenMV_Init(void);
void OpenMV_ParseByte(uint8_t byte);
bool OpenMV_GetData(VisionData_t *data);
bool OpenMV_HasNewData(void);
VisionStatus_e OpenMV_GetStatus(void);
void OpenMV_GetStats(VisionStats_t *stats);
CommStatus_e OpenMV_UpdateCommStatus(uint32_t current_time_ms);
void OpenMV_Reset(void);
const char * OpenMV_GetStatusName(VisionStatus_e status);
bool OpenMV_IsCommOK(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENMV_DRV_H */
