/**
 * @file hal_entry.c
 * @brief RA6M5 事件驱动主循环 + 追踪控制适配层
 * @version 4.0 - 集成 LD2450 毫米波雷达 + 多模态融合追踪
 *
 * 硬件配置说明：
 * - 视觉链路 (OpenMV): g_uart9 (P601=RXD9, P602=TXD9) @ 115200
 * - 控制链路 (ST-3215 舵机 via URT-1): g_uart5 (P501=TXD5, P502=RXD5) @ 1Mbps
 * - 雷达链路 (LD2450): g_uart2 (P301=RXD2, P302=TXD2) @ 256000
 * - DIR 方向控制: P402 (BSP_IO_PORT_04_PIN_02)
 *   注: URT-1 具备硬件自动流向控制，P402 翻转逻辑作为安全冗余保留
 */

#include "hal_data.h"
#include "openmv_drv.h"
#include "drivers/st3215_driver.h"
#include "drivers/ld2450_driver.h"
#include "control/tracking_controller.h"
#include <string.h>
#include <stdio.h>

/*===========================================================================*/
/*                              宏定义                                        */
/*===========================================================================*/

/** @brief 舵机半双工方向控制引脚 (P402) */
#define SERVO_DIR_PIN           BSP_IO_PORT_04_PIN_02

/** @brief DIR 引脚电平定义 (根据 URT-1/MAX485 逻辑调整) */
#define DIR_LEVEL_TX            BSP_IO_LEVEL_HIGH   /* 发送模式 */
#define DIR_LEVEL_RX            BSP_IO_LEVEL_LOW    /* 接收模式 */

/* ================= LED 指示灯引脚定义 ================= */
/* 适配 DshanMCU-RA6M5 开发板 (百问网):
   仅有一个用户指示灯 LED (D12)，连接到 P400，共阳极连接 (Active Low) */
#define LED_PIN_USER            BSP_IO_PORT_04_PIN_00

/** @brief 指示灯点亮电平 (共阳极连接，低电平点亮) */
#define LED_ON                  BSP_IO_LEVEL_LOW
#define LED_OFF                 BSP_IO_LEVEL_HIGH

/*===========================================================================*/
/*                              全局变量                                       */
/*===========================================================================*/

volatile uint32_t g_sys_run_time_ms = 0;
static TrackingController_t g_tracker;
static volatile bool g_servo_initialized = false;
static volatile bool g_servo_tx_complete = true;
static volatile bool g_servo_uart_ok = false;
static volatile bool g_servo_link_ok = false;
static volatile bool g_vision_uart_ok = false;
static volatile bool g_radar_uart_ok  = false;
static volatile bool g_dbg_uart_ok    = false;
static uart_cfg_t g_uart3_runtime_cfg;

static volatile char g_dbg_cmd_buf[96];
static volatile uint8_t g_dbg_cmd_idx = 0U;
static volatile bool g_dbg_cmd_ready = false;
static char g_dbg_cmd_line[96];
static unsigned long g_last_telemetry_ms = 0UL;
static unsigned long g_last_uart5_retry_ms = 0UL;

static void Process_Debug_Command(const char *line);
static void Send_Telemetry_To_ESP32(const VisionData_t *vision, const LD2450_Data_t *radar);
static void Try_Recover_ServoUart(void);
static bool Servo_ProbeBus(void);
static bool Servo_StartupSequence(void);
static void Update_StatusLed(uint32_t current_ms, bool vision_active, bool radar_active);

/*===========================================================================*/
/*                              系统时间接口                                   */
/*===========================================================================*/

/**
 * @brief 获取系统运行时间 (毫秒) - 供 openmv_drv.c 调用
 */
unsigned long OpenMV_GetTickMs(void)
{
    return g_sys_run_time_ms;
}

/**
 * @brief 获取系统运行时间 (毫秒) - 供 ld2450_driver.c 调用
 */
uint32_t LD2450_GetTickMs(void)
{
    return g_sys_run_time_ms;
}

/**
 * @brief 真 SysTick 硬件中断，每 1ms 触发一次
 */
void SysTick_Handler(void)
{
    g_sys_run_time_ms++;
}

/*===========================================================================*/
/*                              追踪控制适配层                                  */
/*===========================================================================*/

/**
 * @brief 追踪控制适配层：将 OpenMV 视觉数据转换为舵机控制指令
 * @param vision_status 视觉状态 (0=丢失, 1=锁定, 2=预测)
 * @param target_x 目标 X 坐标 (像素)
 * @param target_y 目标 Y 坐标 (像素)
 * @param current_time 当前时间 (ms)
 */
void Tracking_Update(unsigned char vision_status,
                     short target_x,
                     short target_y,
                     unsigned long current_time)
{
    if (!g_servo_initialized)
    {
        return;
    }

    Tracking_UpdateController(&g_tracker, vision_status, target_x, target_y, current_time);
}

/*===========================================================================*/
/*                              UART 回调函数                                  */
/*===========================================================================*/

/**
 * @brief UART9 中断回调 - OpenMV 视觉数据接收
 * @note  回调函数名必须与 FSP 配置中填写的名称一致
 */
void uart9_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            /* 逐字节解析 OpenMV 8 字节协议 */
            OpenMV_ParseByte((uint8_t)p_args->data);
            break;

        default:
            /* 忽略其他事件 (TX_COMPLETE 等) */
            break;
    }
}

/**
 * @brief UART5 中断回调 - 舵机通信 (TX 完成 + RX 数据)
 * @note  回调函数名必须与 FSP 配置中填写的名称一致
 */
void uart5_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            /* 发送完成，切换 DIR 至接收模式 (安全冗余，URT-1 硬件也会自动切换) */
            R_IOPORT_PinWrite(&g_ioport_ctrl, SERVO_DIR_PIN, DIR_LEVEL_RX);
            g_servo_tx_complete = true;
            break;

        case UART_EVENT_RX_CHAR:
            /* 解析舵机返回数据 (如读取位置反馈) */
            ST3215_ParseByte((uint8_t)p_args->data);
            break;

        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_BREAK_DETECT:
            /* 错误中断风暴阻断：清除状态兵强制切回 RX，防止 TX 常亮死锁 */
            R_IOPORT_PinWrite(&g_ioport_ctrl, SERVO_DIR_PIN, DIR_LEVEL_RX);
            g_servo_tx_complete = true;  /* 解除 while 阻塞 */
            g_servo_link_ok = false;
            g_servo_initialized = false;
            break;

        default:
            break;
    }
}

/**
 * @brief UART2 中断回调 - LD2450 毫米波雷达数据接收
 * @note  回调函数名必须与 FSP 配置中填写的名称一致
 */
void uart2_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
            /* 逜字节解析 LD2450 30 字节帧 */
            LD2450_ParseByte((uint8_t)p_args->data);
            break;

        default:
            break;
    }
}

/**
 * @brief UART3 中断回调 - 调参命令接收 + ESP32链路
 */
void uart3_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_RX_CHAR)
    {
        char ch = (char)p_args->data;
        if (ch == '\r')
        {
            return;
        }

        if (ch == '\n')
        {
            if (g_dbg_cmd_idx > 0U)
            {
                g_dbg_cmd_buf[g_dbg_cmd_idx] = '\0';
                g_dbg_cmd_ready = true;
                g_dbg_cmd_idx = 0U;
            }
            return;
        }

        if ((g_dbg_cmd_idx + 1U) < (uint8_t)sizeof(g_dbg_cmd_buf))
        {
            g_dbg_cmd_buf[g_dbg_cmd_idx++] = ch;
        }
        else
        {
            g_dbg_cmd_idx = 0U;
        }
    }
}

/*===========================================================================*/
/*                              舵机驱动回调接口                                */
/*===========================================================================*/

/**
 * @brief 舵机 UART 发送回调 - 供 st3215_driver.c 调用
 * @param data 待发送数据指针
 * @param len  数据长度
 */
static void Servo_TxCallback(const uint8_t *data, uint16_t len)
{
    fsp_err_t err;

    if (!g_servo_uart_ok)
    {
        return;
    }

    /* 1. 切换 DIR 至发送模式 (安全冗余) */
    R_IOPORT_PinWrite(&g_ioport_ctrl, SERVO_DIR_PIN, DIR_LEVEL_TX);

    /* 2. 标记发送未完成 */
    g_servo_tx_complete = false;

    /* 3. 启动非阻塞发送 */
    err = R_SCI_UART_Write(&g_uart5_ctrl, data, len);
    if (FSP_SUCCESS != err)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, SERVO_DIR_PIN, DIR_LEVEL_RX);
        g_servo_tx_complete = true;
        g_servo_link_ok = false;
        g_servo_initialized = false;
        return;
    }

    /* 4. 等待发送完成 (由 uart5_callback 中 TX_COMPLETE 事件触发) */
    uint32_t timeout = 2500U; /* 降级到 2.5ms 极短超时硬截断 (1Mbps波特率发十几个字节也就几百us) */
    while (!g_servo_tx_complete && (timeout > 0U))
    {
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MICROSECONDS);
        timeout--;
    }

    /* 5. 超时保护：强制切回接收模式 */
    if (!g_servo_tx_complete)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, SERVO_DIR_PIN, DIR_LEVEL_RX);
        g_servo_tx_complete = true;
        g_servo_link_ok = false;
        g_servo_initialized = false;
    }
}

/**
 * @brief 舵机方向控制回调 - 供 st3215_driver.c 调用
 * @param is_tx true=发送模式, false=接收模式
 * @note  此回调作为安全冗余保留；若仅依赖 URT-1 硬件，可置为空操作
 */
static void Servo_DirCallback(bool is_tx)
{
    if (is_tx)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, SERVO_DIR_PIN, DIR_LEVEL_TX);
    }
    else
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, SERVO_DIR_PIN, DIR_LEVEL_RX);
    }
}

/**
 * @brief 延时回调 - 供 st3215_driver.c 调用
 * @param us 延时微秒数
 */
static void Servo_DelayCallback(uint32_t us)
{
    R_BSP_SoftwareDelay(us, BSP_DELAY_UNITS_MICROSECONDS);
}

static bool Servo_ProbeBus(void)
{
    for (uint8_t attempt = 0; attempt < 3U; attempt++)
    {
        if (ST3215_Ping(GIMBAL_PAN_SERVO_ID) && ST3215_Ping(GIMBAL_TILT_SERVO_ID))
        {
            return true;
        }

        R_BSP_SoftwareDelay(20U, BSP_DELAY_UNITS_MILLISECONDS);
        R_IWDT_Refresh(&g_iwdt0_ctrl);
    }

    return false;
}

static bool Servo_StartupSequence(void)
{
    if (!Servo_ProbeBus())
    {
        g_servo_link_ok = false;
        g_servo_initialized = false;
        return false;
    }

    g_servo_link_ok = true;

    ST3215_SetTorque(GIMBAL_PAN_SERVO_ID, false);
    R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MILLISECONDS);
    ST3215_SetTorque(GIMBAL_TILT_SERVO_ID, false);
    R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS);
    R_IWDT_Refresh(&g_iwdt0_ctrl);

    ST3215_SetPosition(GIMBAL_PAN_SERVO_ID, SERVO_POSITION_CENTER, 800);
    R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MILLISECONDS);
    ST3215_SetPosition(GIMBAL_TILT_SERVO_ID, SERVO_POSITION_CENTER, 800);
    R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MILLISECONDS);

    ST3215_SetTorque(GIMBAL_PAN_SERVO_ID, true);
    R_BSP_SoftwareDelay(20U, BSP_DELAY_UNITS_MILLISECONDS);
    ST3215_SetTorque(GIMBAL_TILT_SERVO_ID, true);
    R_BSP_SoftwareDelay(20U, BSP_DELAY_UNITS_MILLISECONDS);
    R_IWDT_Refresh(&g_iwdt0_ctrl);

    g_servo_initialized = true;
    return true;
}

static void Update_StatusLed(uint32_t current_ms, bool vision_active, bool radar_active)
{
    if (!g_servo_uart_ok || !g_servo_link_ok || !g_servo_initialized)
    {
        if ((current_ms % 200U) < 100U)
        {
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_ON);
        }
        else
        {
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_OFF);
        }
        return;
    }

    if (vision_active && radar_active)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_ON);
    }
    else if (vision_active)
    {
        if ((current_ms % 200U) < 100U)
        {
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_ON);
        }
        else
        {
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_OFF);
        }
    }
    else if (radar_active)
    {
        if ((current_ms % 1000U) < 500U)
        {
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_ON);
        }
        else
        {
            R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_OFF);
        }
    }
    else if ((current_ms % 1000U) < 60U)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_ON);
    }
    else
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_OFF);
    }
}

static void Process_Debug_Command(const char *line)
{
    TrackerConfig_t cfg;
    const TrackerConfig_t *cur = Tracking_GetRuntimeConfig(&g_tracker);
    char ack[96];

    if ((line == NULL) || (cur == NULL))
    {
        return;
    }

    cfg = *cur;

    if (sscanf(line, "SET THRES %f %hu", &cfg.max_allowed_deviation_deg, &cfg.vision_confirm_frames) == 2)
    {
        Tracking_SetRuntimeConfig(&g_tracker, &cfg);
        (void)snprintf(ack, sizeof(ack), "OK THRES %.1f %u\n", (double)cfg.max_allowed_deviation_deg, cfg.vision_confirm_frames);
        if (g_dbg_uart_ok) { (void)R_SCI_UART_Write(&g_uart3_ctrl, (uint8_t *)ack, (uint32_t)strlen(ack)); }
        return;
    }

    if (sscanf(line, "SET TIMEOUT %lu %lu", &cfg.coasting_timeout_ms, &cfg.ghost_timeout_ms) == 2)
    {
        Tracking_SetRuntimeConfig(&g_tracker, &cfg);
        (void)snprintf(ack, sizeof(ack), "OK TIMEOUT %lu %lu\n", cfg.coasting_timeout_ms, cfg.ghost_timeout_ms);
        if (g_dbg_uart_ok) { (void)R_SCI_UART_Write(&g_uart3_ctrl, (uint8_t *)ack, (uint32_t)strlen(ack)); }
        return;
    }

    if (sscanf(line, "SET PID %f %f %f", &cfg.vision_pid_kp, &cfg.vision_pid_ki, &cfg.vision_pid_kd) == 3)
    {
        Tracking_SetRuntimeConfig(&g_tracker, &cfg);
        (void)snprintf(ack, sizeof(ack), "OK PID %.3f %.3f %.3f\n", (double)cfg.vision_pid_kp, (double)cfg.vision_pid_ki, (double)cfg.vision_pid_kd);
        if (g_dbg_uart_ok) { (void)R_SCI_UART_Write(&g_uart3_ctrl, (uint8_t *)ack, (uint32_t)strlen(ack)); }
        return;
    }

    if (sscanf(line, "SET KF %f %f", &cfg.kf_q, &cfg.kf_r) == 2)
    {
        Tracking_SetRuntimeConfig(&g_tracker, &cfg);
        (void)snprintf(ack, sizeof(ack), "OK KF %.3f %.3f\n", (double)cfg.kf_q, (double)cfg.kf_r);
        if (g_dbg_uart_ok) { (void)R_SCI_UART_Write(&g_uart3_ctrl, (uint8_t *)ack, (uint32_t)strlen(ack)); }
        return;
    }

    (void)snprintf(ack, sizeof(ack), "ERR CMD\n");
    if (g_dbg_uart_ok) { (void)R_SCI_UART_Write(&g_uart3_ctrl, (uint8_t *)ack, (uint32_t)strlen(ack)); }
}

static void Send_Telemetry_To_ESP32(const VisionData_t *vision, const LD2450_Data_t *radar)
{
    char packet[256];
    const char *state = "SEARCH";
    uint16_t pan_pos = 0U;
    uint16_t tilt_pos = 0U;
    uint8_t vision_active = 0U;

    if (!g_dbg_uart_ok)
    {
        return;
    }

    if ((g_sys_run_time_ms - g_last_telemetry_ms) < 100UL)
    {
        return;
    }
    g_last_telemetry_ms = g_sys_run_time_ms;

    Tracking_GetServoPositions(&g_tracker, &pan_pos, &tilt_pos);

    switch (g_tracker.status.state)
    {
        case STATE_SEARCHING:        state = "SEARCH"; break;
        case STATE_RADAR_OUTER_LOOP: state = "RADAR_SLEW"; break;
        case STATE_VISION_INNER_LOOP:state = "VISION_LOCK"; vision_active = 1U; break;
        case STATE_VISION_COASTING:  state = "COASTING"; break;
        default:                     state = "SEARCH"; break;
    }

    uint8_t hover_active = 0U;
    if (g_tracker.status.state == STATE_VISION_INNER_LOOP) {
        float px = g_tracker.status.error_x;
        float py = g_tracker.status.error_y;
        if ((px * px < 225.0f) && (py * py < 225.0f)) { /* 15.0f * 15.0f = 225.0f */
            hover_active = 1U;
        }
    }

    /* 雷达深度: 取首个有效目标的 Y 距离 (mm), 无目标时为 0 */
    int16_t radar_dist = 0;
    if ((radar != NULL) && (radar->valid_count > 0U) && radar->targets[0].valid)
    {
        radar_dist = radar->targets[0].y;
    }
    /* 钳位到 WebCloud 可接受范围 [0, 6000], 防止负值或超限导致上传被拒 */
    if (radar_dist < 0) radar_dist = 0;
    if (radar_dist > 6000) radar_dist = 6000;

    /* 视觉目标像素坐标 (钳位到 [0, 239] 保证上传不被拒) */
    int16_t target_x = 120;
    int16_t target_y = 120;
    if (vision != NULL)
    {
        target_x = vision->x;
        target_y = vision->y;
        if (target_x < 0) target_x = 0;
        if (target_x > 239) target_x = 239;
        if (target_y < 0) target_y = 0;
        if (target_y > 239) target_y = 239;
    }

    (void)snprintf(packet,
                   sizeof(packet),
                   "{\"state\":\"%s\",\"target_angle\":%u,\"vision_active\":%u,"
                   "\"tracking_mode\":\"%s\",\"hover_active\":%u,"
                   "\"radar_dist\":%d,\"target_x\":%d,\"target_y\":%d,\"gimbal_tilt\":%u}\n",
                   state,
                   pan_pos,
                   vision_active,
                   Tracking_GetStateName(g_tracker.status.state),
                   hover_active,
                   (int)radar_dist,
                   (int)target_x,
                   (int)target_y,
                   tilt_pos);
    (void)R_SCI_UART_Write(&g_uart3_ctrl, (uint8_t *)packet, (uint32_t)strlen(packet));
}

static void Try_Recover_ServoUart(void)
{
    fsp_err_t err;

    if (g_servo_uart_ok && g_servo_link_ok && g_servo_initialized)
    {
        return;
    }

    if ((g_sys_run_time_ms - g_last_uart5_retry_ms) < 500UL)
    {
        return;
    }

    g_last_uart5_retry_ms = g_sys_run_time_ms;
    g_servo_uart_ok = false;
    g_servo_link_ok = false;
    g_servo_initialized = false;
    R_SCI_UART_Close(&g_uart5_ctrl);
    err = R_SCI_UART_Open(&g_uart5_ctrl, &g_uart5_cfg);
    if (FSP_SUCCESS != err)
    {
        return;
    }

    g_servo_uart_ok = true;
    ST3215_Init(Servo_TxCallback, Servo_DirCallback, Servo_DelayCallback);
    R_BSP_SoftwareDelay(50U, BSP_DELAY_UNITS_MILLISECONDS);

    if (Servo_StartupSequence())
    {
        Tracking_Reset(&g_tracker);
    }
}

/*===========================================================================*/
/*                              主函数入口                                     */
/*===========================================================================*/

void hal_entry(void)
{
    fsp_err_t err;

    /* 保证系统时基优先级高于 UART 中断，避免串口洪峰挤占时间基准。 */
    NVIC_SetPriority(SysTick_IRQn, 7U);
    SysTick_Config(SystemCoreClock / 1000); /* 激活硬件真 1ms 滴答时基 */

    /* 初始化并开启独立看门狗 (由 FSP 配合生成 g_iwdt0_ctrl) */
    R_IWDT_Open(&g_iwdt0_ctrl, &g_iwdt0_cfg);
    R_IWDT_Refresh(&g_iwdt0_ctrl); /* 上电立即喂狗强心 */

    /*=======================================================================*/
    /* 1. 配置 DIR 引脚 (P402) 为输出，初始状态：接收模式                       */
    /*=======================================================================*/
    R_IOPORT_PinCfg(&g_ioport_ctrl, SERVO_DIR_PIN,
                    (uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW));

    /* 初始化用户 LED 引脚为输出模式，并默认关闭 */
    R_IOPORT_PinCfg(&g_ioport_ctrl, LED_PIN_USER,
                    (uint32_t)(IOPORT_CFG_PORT_DIRECTION_OUTPUT | (LED_OFF == BSP_IO_LEVEL_HIGH ? IOPORT_CFG_PORT_OUTPUT_HIGH : IOPORT_CFG_PORT_OUTPUT_LOW)));
    R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_USER, LED_ON);

    err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    if (FSP_SUCCESS != err)
    {
        g_vision_uart_ok = false;
        /* 视觉 UART 初始化失败, 允许降级为纯雷达模式 */
    }
    else
    {
        g_vision_uart_ok = true;
    }

    /*=======================================================================*/
    /* 3. 初始化舵机通信 (UART5/SCI5: P501/P502, 1Mbps)                        */
    /*=======================================================================*/
    err = R_SCI_UART_Open(&g_uart5_ctrl, &g_uart5_cfg);
    if (FSP_SUCCESS != err)
    {
        g_servo_uart_ok = false;
        g_servo_link_ok = false;
        g_servo_initialized = false;
    }
    else
    {
        g_servo_uart_ok = true;
        g_servo_link_ok = false;
    }

    /*=======================================================================*/
    /* 3b. 初始化 LD2450 雷达通信 (UART2/SCI2: P301/P302, 256000bps)             */
    /*=======================================================================*/
    LD2450_Init();

    err = R_SCI_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    if (FSP_SUCCESS != err)
    {
        g_radar_uart_ok = false;
        /* 雷达 UART 初始化失败, 允许降级为纯视觉模式 */
    }
    else
    {
        g_radar_uart_ok = true;
    }

    /* 唤醒 ESP32 网关链路 (SCI3: P706=RX, P707=TX)
     * 使用运行时 cfg 覆盖回调，实现动态调参与遥测共用链路。
     */
    memcpy(&g_uart3_runtime_cfg, &g_uart3_cfg, sizeof(uart_cfg_t));
    g_uart3_runtime_cfg.p_callback = uart3_callback;
    err = R_SCI_UART_Open(&g_uart3_ctrl, &g_uart3_runtime_cfg);
    g_dbg_uart_ok = (FSP_SUCCESS == err);

    /* 等待 UART 稳定 */
    R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS);
    R_IWDT_Refresh(&g_iwdt0_ctrl); /* 沿途喂狗防线 */

    /*=======================================================================*/
    /* 4. 初始化舵机驱动层                                                     */
    /*=======================================================================*/
    if (g_servo_uart_ok)
    {
        ST3215_Init(Servo_TxCallback, Servo_DirCallback, Servo_DelayCallback);
        R_BSP_SoftwareDelay(50U, BSP_DELAY_UNITS_MILLISECONDS);
        (void)Servo_StartupSequence();
    }

    /*=======================================================================*/
    /* 5. 初始化追踪控制器                                                     */
    /*=======================================================================*/
    TrackingConfig_t track_config;
    Tracking_GetDefaultConfig(&track_config);

    /* PID 参数调整 (根据实际云台性能微调) */
    track_config.pan_pid_params.kp = 5.0f;
    track_config.pan_pid_params.ki = 0.05f;
    track_config.pan_pid_params.kd = 1.0f;
    track_config.tilt_pid_params.kp = 4.0f;
    track_config.tilt_pid_params.ki = 0.04f;
    track_config.tilt_pid_params.kd = 0.8f;

    /* 方向反转 (根据实际舵机安装方向调整) */
    track_config.invert_pan = true;
    track_config.invert_tilt = true;

    Tracking_Init(&g_tracker, &track_config);

    /* 舵机归中测试 */
    if (g_servo_initialized)
    {
        Tracking_Reset(&g_tracker);
    }
    /* 拆分长达 1000ms 的硬延时，插入喂狗，规避 1000ms 的 IWDT 复位限制 */
    for (int delay_slice = 0; delay_slice < 10; delay_slice++) {
        R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS);
        R_IWDT_Refresh(&g_iwdt0_ctrl); /* 沿途喂狗防线 */
    }

    /*=======================================================================*/
    /* 6. 事件驱动主循环 (多模态融合: 视觉 + 雷达)                          */
    /*=======================================================================*/
    static VisionData_t vision_data = {0};
    static LD2450_Data_t radar_data = {0};

    while (1)
    {
        /* 主循环内持续喂狗 */
        R_IWDT_Refresh(&g_iwdt0_ctrl);
        
        uint32_t current_ms = g_sys_run_time_ms;

        /* 避免 UART5 启动瞬态失败导致系统永久卡死，后台重试恢复舵机链路 */
        Try_Recover_ServoUart();

        /* 轮询两个传感器的最新数据 (非阻塞, UART 失败时跳过) */
        (void)(g_vision_uart_ok ? OpenMV_GetData(&vision_data) : false);
        (void)(g_radar_uart_ok  ? LD2450_GetData(&radar_data)  : false);

        /* 视觉 UART 失败或通信超时 → 强制 LOST */
        if (!g_vision_uart_ok || !OpenMV_IsCommOK())
        {
            vision_data.status = VISION_STATUS_LOST;
        }

        /* 仅在舵机链路恢复后进入融合控制，避免故障期间持续刷 UART5 干扰恢复。 */
        if (g_servo_initialized)
        {
            Tracking_UpdateMultiModal(&g_tracker,
                                      &vision_data,
                                      &radar_data,
                                      current_ms);
        }
        else
        {
            g_tracker.status.state = STATE_SEARCHING;
        }

        if (g_dbg_cmd_ready)
        {
            __disable_irq();
            strcpy(g_dbg_cmd_line, (const char *)g_dbg_cmd_buf);
            g_dbg_cmd_ready = false;
            __enable_irq();
            Process_Debug_Command(g_dbg_cmd_line);
        }

        Send_Telemetry_To_ESP32(&vision_data, &radar_data);

        /* 检测通信超时 */
        if (g_vision_uart_ok)
        {
            (void)OpenMV_UpdateCommStatus(current_ms);
        }

        /* 状态指示灯逻辑 (主循环周期执行) */
        /* 判断当前 OpenMV 是否有效识别到目标 */
        bool vision_active = (vision_data.status == VISION_STATUS_LOCKED) || (vision_data.status == VISION_STATUS_PREDICTED);
        
        /* 判断当前雷达是否有有效目标 (数据不仅获取成功，且距离此刻不超过 500ms) */
        bool radar_active = ((radar_data.valid_count > 0U) && ((current_ms - radar_data.timestamp) < 500U));
        
        /* LED 状态指示 (DshanMCU RA6M5 只有单色灯 P400)
         * - 舵机链路故障     ➜ 5Hz 快闪
         * - 正常但无目标     ➜ 1Hz 心跳
         * - 仅雷达检测到     ➜ 慢闪（500ms）
         * - 仅视觉检测到     ➜ 快闪（100ms）
         * - 视觉+雷达融合稳定 ➜ 常亮
         */
        Update_StatusLed(current_ms, vision_active, radar_active);

        /* 1ms 时基 */
        R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
    }
}
