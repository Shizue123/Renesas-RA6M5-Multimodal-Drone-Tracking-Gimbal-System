/**
 * @file st3215_driver.h
 * @brief 飞特 ST-3215 串行总线舵机驱动
 * @version 2.0 - 增加用户可配置宏
 *
 * 硬件说明：
 * - 通信方式：单线半双工 TTL (使用 URT-1 调试板桥接)
 * - 波特率：1Mbps
 * - 协议：飞特标准串行总线协议
 */

#ifndef ST3215_DRIVER_H
#define ST3215_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "hal_data.h"  /* 引入 FSP 生成的句柄定义 */

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/*                         用户可配置宏 (根据硬件调整)                          */
/*===========================================================================*/

/**
 * @brief 舵机 UART 句柄 (FSP 实例名)
 * @note  必须与 FSP configuration.xml 中配置的实例名一致
 */
#define ST3215_UART_HANDLE      g_uart5_ctrl

/**
 * @brief 半双工方向控制引脚 (P402)
 * @note  若使用 URT-1 调试板，硬件自动切换方向，此引脚作为安全冗余
 */
#define ST3215_DIR_PIN          BSP_IO_PORT_04_PIN_02

/**
 * @brief DIR 引脚电平定义
 */
#define ST3215_DIR_TX_LEVEL     BSP_IO_LEVEL_HIGH   /* 发送模式电平 */
#define ST3215_DIR_RX_LEVEL     BSP_IO_LEVEL_LOW    /* 接收模式电平 */

/*===========================================================================*/
/*                              云台舵机配置                                   */
/*===========================================================================*/

/** @brief 水平轴舵机 ID (Yaw/Pan) — 物理底部舵机 */
#define GIMBAL_PAN_SERVO_ID     1

/** @brief 俯仰轴舵机 ID (Pitch/Tilt) — 物理顶部舵机 */
#define GIMBAL_TILT_SERVO_ID    2

/** @brief 舵机位置中心值 (12位分辨率: 0-4095) */
#define SERVO_POSITION_CENTER   2047

/** @brief 物理安全软限位: 舵机1(Tilt) 允许区间, 下限高于天顶(1023)防极性翻转 */
#define TILT_SERVO_SAFE_MIN     1080
#define TILT_SERVO_SAFE_MAX     2457

/** @brief 舵机位置最小值 */
#define SERVO_POSITION_MIN      0

/** @brief 舵机位置最大值 */
#define SERVO_POSITION_MAX      4095

/*===========================================================================*/
/*                              飞特指令定义                                   */
/*===========================================================================*/

#define ST3215_CMD_PING         0x01    /* Ping 指令 */
#define ST3215_CMD_READ         0x02    /* 读取指令 */
#define ST3215_CMD_WRITE        0x03    /* 写入指令 */
#define ST3215_CMD_REG_WRITE    0x04    /* 异步写入 */
#define ST3215_CMD_ACTION       0x05    /* 触发动作 */
#define ST3215_CMD_SYNC_WRITE   0x83    /* 同步写入 */

/*===========================================================================*/
/*                              内存表地址                                     */
/*===========================================================================*/

#define ST3215_ADDR_TORQUE_ENABLE       0x28    /* 扭矩使能 (1字节) */
#define ST3215_ADDR_TARGET_POSITION     0x2A    /* 目标位置 (2字节, Little-Endian) */
#define ST3215_ADDR_RUNNING_TIME        0x2C    /* 运行时间 (2字节) */
#define ST3215_ADDR_PRESENT_POSITION    0x38    /* 当前位置 (2字节) */
#define ST3215_ADDR_PRESENT_SPEED       0x3A    /* 当前速度 (2字节) */
#define ST3215_ADDR_PRESENT_LOAD        0x3C    /* 当前负载 (2字节) */
#define ST3215_ADDR_PRESENT_VOLTAGE     0x3E    /* 当前电压 (1字节) */
#define ST3215_ADDR_PRESENT_TEMPERATURE 0x3F    /* 当前温度 (1字节) */

/*===========================================================================*/
/*                              回调函数类型                                   */
/*===========================================================================*/

/**
 * @brief 串口发送回调函数类型
 * @param data 数据指针
 * @param len 数据长度
 */
typedef void (*ST3215_TxCallback_t)(const uint8_t *data, uint16_t len);

/**
 * @brief 方向控制回调函数类型
 * @param is_tx true=发送模式, false=接收模式
 */
typedef void (*ST3215_DirCallback_t)(bool is_tx);

/**
 * @brief 延时回调函数类型
 * @param us 微秒数
 */
typedef void (*ST3215_DelayCallback_t)(uint32_t us);

/*===========================================================================*/
/*                              API 函数声明                                   */
/*===========================================================================*/

/**
 * @brief 初始化舵机驱动
 * @param tx_cb 发送回调函数
 * @param dir_cb 方向控制回调函数 (可为 NULL 如使用 URT-1)
 * @param delay_cb 延时回调函数
 */
void ST3215_Init(ST3215_TxCallback_t tx_cb,
                 ST3215_DirCallback_t dir_cb,
                 ST3215_DelayCallback_t delay_cb);

/**
 * @brief 设置单个舵机目标位置
 * @param id 舵机 ID (1-253)
 * @param position 目标位置 (0-4095)
 * @param time 运行时间 (ms), 0=最快速度
 */
void ST3215_SetPosition(uint8_t id, uint16_t position, uint16_t time);

/**
 * @brief 同步设置多个舵机位置 (广播指令)
 * @param ids 舵机 ID 数组
 * @param positions 位置数组
 * @param count 舵机数量
 * @param time 运行时间 (ms)
 */
void ST3215_SyncSetPosition(const uint8_t *ids, const uint16_t *positions,
                            uint8_t count, uint16_t time);

/**
 * @brief 设置舵机扭矩使能
 * @param id 舵机 ID
 * @param enable true=使能, false=关闭
 */
void ST3215_SetTorque(uint8_t id, bool enable);

/**
 * @brief 读取舵机当前位置
 * @param id 舵机 ID
 * @return 当前位置值 (0-4095), 失败返回 0xFFFF
 */
uint16_t ST3215_ReadPosition(uint8_t id);

/**
 * @brief 获取最近一次成功读取的位置
 * @param id 舵机 ID
 * @param out_position 输出位置值
 * @return true=数据有效, false=无响应或校验失败
 */
bool ST3215_GetLastPosition(uint8_t id, uint16_t *out_position);

/**
 * @brief 解析舵机返回的字节数据 (中断上下文调用)
 * @param byte 接收到的字节
 */
void ST3215_ParseByte(uint8_t byte);

/**
 * @brief Ping 舵机检测在线状态
 * @param id 舵机 ID
 * @return true=在线, false=无响应
 */
bool ST3215_Ping(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* ST3215_DRIVER_H */
