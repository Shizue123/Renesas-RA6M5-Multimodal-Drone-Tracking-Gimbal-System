/**
 * @file   ra6m5_tracking_core_update.c
 * @brief  RA6M5 追踪核心更新片段 — 边界钳位 + 平滑归中 + SCI3 JSON 网关上传
 *
 * 本文件为 Competition_Deliverables 参考代码片段, 与
 * Drone_Tracking_RA6M5/src/control/tracking_controller.c 及 hal_entry.c 保持同步。
 *
 * 硬件连接:
 *   SCI3 (UART3): P706 (TXD3) → ESP32 RXD, P707 (RXD3) ← ESP32 TXD
 *   波特率: 115200 bps
 *
 * 上行帧格式 (JSON):
 *   {"state":"<state>","target_angle":<pan>,"vision_active":<0|1>,
 *    "tracking_mode":"<mode>","hover_active":<0|1>,
 *    "radar_dist":<dist>,"target_x":<x>,"target_y":<y>,"gimbal_tilt":<tilt>}\n
 *
 * 追踪状态机 (与 tracking_controller.h 同步):
 *   STATE_SEARCHING → STATE_RADAR_OUTER_LOOP → STATE_VISION_INNER_LOOP → STATE_VISION_COASTING
 */

#include "hal_data.h"
#include "control/tracking_controller.h"
#include "openmv_drv.h"
#include "drivers/ld2450_driver.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ══════════════════════════════════════════════════════════════════════════
 *  1. 严苛边界钳位 — Pan 0~4095 (全域), Tilt 1080~2457 (天顶安全)
 * ══════════════════════════════════════════════════════════════════════════ */

/** @brief 通用 uint16 钳位, 防止 PID 输出超越物理极限 */
static inline uint16_t gimbal_clamp_u16(long value, uint16_t lo, uint16_t hi)
{
    if (value < (long)lo) return lo;
    if (value > (long)hi) return hi;
    return (uint16_t)value;
}

/* 舵机物理边界常量 (与 tracking_controller.h 保持一致) */
#define PAN_POS_MIN     PAN_PHYS_MIN_POS    /* 0U */
#define PAN_POS_MAX     PAN_PHYS_MAX_POS    /* 4095U */
#define PAN_POS_CENTER  2047U

#define TILT_POS_MIN    TILT_PHYS_MIN_POS   /* 1080U */
#define TILT_POS_MAX    TILT_PHYS_MAX_POS   /* 2457U */
#define TILT_POS_CENTER 2047U

/**
 * @brief 应用边界钳位到追踪控制器输出
 *
 * 在 PID 增量计算后, 舵机下发前调用。确保任何异常的误差累积
 * 都不会将舵机驱动至物理死角。
 */
static void apply_boundary_clamp(TrackingController_t *ctrl)
{
    ctrl->status.pan_position =
        gimbal_clamp_u16((long)ctrl->status.pan_position, PAN_POS_MIN, PAN_POS_MAX);
    ctrl->status.tilt_position =
        gimbal_clamp_u16((long)ctrl->status.tilt_position, TILT_POS_MIN, TILT_POS_MAX);
}


/* ══════════════════════════════════════════════════════════════════════════
 *  2. 平滑步进归中策略 — 丢失目标后分阶段恢复
 * ══════════════════════════════════════════════════════════════════════════ */

/** @brief 归中时的舵机运动时间 (ms), 控制梯形速度规划, 防止机械冲击 */
#define RETURN_CENTER_MOVE_TIME  500U

/** @brief 丢失超时窗口 (ms), 在此期间保持当前位置等待目标恢复 */
#define LOST_TIMEOUT_MS          500U

/** @brief 平滑步进归中步距: 每次控制周期最多移动的舵机步数 */
#define SMOOTH_RETURN_STEP       8U

/**
 * @brief 平滑步进归中 — 逐步逼近中心位置, 防止瞬间跳跃
 *
 * 当双重丢失超时后调用此函数, 每个主循环周期 (1ms) 将 pan/tilt
 * 各移动最多 ±SMOOTH_RETURN_STEP 步数, 逐帧趋近中心。
 *
 * @param[in,out] ctrl  追踪控制器实例
 * @return true 如果已到达中心 (归中完毕)
 */
static bool smooth_step_to_center(TrackingController_t *ctrl)
{
    bool pan_done  = false;
    bool tilt_done = false;

    /* --- Pan 轴归中 --- */
    int32_t pan_diff = (int32_t)PAN_POS_CENTER - (int32_t)ctrl->status.pan_position;
    if (pan_diff > (int32_t)SMOOTH_RETURN_STEP) {
        ctrl->status.pan_position += SMOOTH_RETURN_STEP;
    } else if (pan_diff < -(int32_t)SMOOTH_RETURN_STEP) {
        ctrl->status.pan_position -= SMOOTH_RETURN_STEP;
    } else {
        ctrl->status.pan_position = PAN_POS_CENTER;
        pan_done = true;
    }

    /* --- Tilt 轴归中 --- */
    int32_t tilt_diff = (int32_t)TILT_POS_CENTER - (int32_t)ctrl->status.tilt_position;
    if (tilt_diff > (int32_t)SMOOTH_RETURN_STEP) {
        ctrl->status.tilt_position += SMOOTH_RETURN_STEP;
    } else if (tilt_diff < -(int32_t)SMOOTH_RETURN_STEP) {
        ctrl->status.tilt_position -= SMOOTH_RETURN_STEP;
    } else {
        ctrl->status.tilt_position = TILT_POS_CENTER;
        tilt_done = true;
    }

    /* 钳位保护 */
    apply_boundary_clamp(ctrl);

    return (pan_done && tilt_done);
}


/* ══════════════════════════════════════════════════════════════════════════
 *  3. SCI3 网关 JSON 帧发送 — 与 hal_entry.c Send_Telemetry_To_ESP32 一致
 * ══════════════════════════════════════════════════════════════════════════ */

/** @brief SCI3 发送缓冲区 (UART3 → ESP32 网关) */
static char g_sci3_tx_buf[256];

/**
 * @brief 通过 SCI3 发送追踪状态 JSON 帧至 ESP32 网关
 *
 * JSON 字段:
 *   state         - 简短状态: "SEARCH"/"RADAR_SLEW"/"VISION_LOCK"/"COASTING"
 *   target_angle  - Pan 舵机原始位置 (0-4095)
 *   vision_active - 视觉是否激活 (0/1)
 *   tracking_mode - 完整状态机名称 (STATE_SEARCHING 等)
 *   hover_active  - 悬停锁定指示 (误差<15px 时为 1)
 *   radar_dist    - 雷达深度距离 (mm, 0-6000)
 *   target_x      - 视觉目标 X 像素 (0-239, 以 120 为中心)
 *   target_y      - 视觉目标 Y 像素 (0-239, 以 120 为中心)
 *   gimbal_tilt   - Tilt 舵机原始位置 (1080-2457)
 *
 * @param ctrl        追踪控制器 (获取状态, pan/tilt 位置)
 * @param vision      视觉数据 (获取 x, y)
 * @param radar       雷达数据 (获取距离)
 */
void Gateway_SendTrackingJSON(const TrackingController_t *ctrl,
                              const VisionData_t         *vision,
                              const LD2450_Data_t        *radar)
{
    if (ctrl == NULL) return;

    uint16_t pan_pos = 0U, tilt_pos = 0U;
    Tracking_GetServoPositions(ctrl, &pan_pos, &tilt_pos);

    const char *state = "SEARCH";
    uint8_t vision_active = 0U;
    switch (ctrl->status.state) {
        case STATE_SEARCHING:         state = "SEARCH";      break;
        case STATE_RADAR_OUTER_LOOP:  state = "RADAR_SLEW";  break;
        case STATE_VISION_INNER_LOOP: state = "VISION_LOCK"; vision_active = 1U; break;
        case STATE_VISION_COASTING:   state = "COASTING";    break;
        default:                      state = "SEARCH";      break;
    }

    /* 悬停判定: 视觉内环锁定且误差 < 15px */
    uint8_t hover_active = 0U;
    if (ctrl->status.state == STATE_VISION_INNER_LOOP) {
        float px = ctrl->status.error_x;
        float py = ctrl->status.error_y;
        if ((px * px < 225.0f) && (py * py < 225.0f)) {
            hover_active = 1U;
        }
    }

    /* 雷达深度: 首个有效目标 Y 距离 (mm) */
    int16_t radar_dist = 0;
    if ((radar != NULL) && (radar->valid_count > 0U) && radar->targets[0].valid) {
        radar_dist = radar->targets[0].y;
    }
    if (radar_dist < 0) radar_dist = 0;
    if (radar_dist > 6000) radar_dist = 6000;

    /* 视觉目标像素 (0-239, 以 120 为中心) */
    int16_t target_x = 120, target_y = 120;
    if (vision != NULL) {
        target_x = vision->x;
        target_y = vision->y;
        if (target_x < 0) target_x = 0;
        if (target_x > 239) target_x = 239;
        if (target_y < 0) target_y = 0;
        if (target_y > 239) target_y = 239;
    }

    int len = snprintf(g_sci3_tx_buf, sizeof(g_sci3_tx_buf),
                       "{\"state\":\"%s\",\"target_angle\":%u,\"vision_active\":%u,"
                       "\"tracking_mode\":\"%s\",\"hover_active\":%u,"
                       "\"radar_dist\":%d,\"target_x\":%d,\"target_y\":%d,\"gimbal_tilt\":%u}\n",
                       state,
                       pan_pos,
                       vision_active,
                       Tracking_GetStateName(ctrl->status.state),
                       hover_active,
                       (int)radar_dist,
                       (int)target_x,
                       (int)target_y,
                       tilt_pos);

    if (len <= 0 || len >= (int)sizeof(g_sci3_tx_buf)) return;

    R_SCI_UART_Write(&g_uart3_ctrl, (const uint8_t *)g_sci3_tx_buf, (uint32_t)len);
}


/* ══════════════════════════════════════════════════════════════════════════
 *  4. 集成示例 — 在 hal_entry 主循环中调用
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * 下方代码展示如何在 hal_entry.c 的主循环中集成以上模块。
 * 请根据实际工程将相关片段合并到现有 hal_entry() 函数中。
 *
 * 注意: 实际 hal_entry.c 已包含完整实现 (Send_Telemetry_To_ESP32 等),
 *       此处仅作为独立参考/验证用途。
 */

#if 0  /* 集成参考, 编译时外层 #if 改为 1 */

/* 网关上传节流: 每 100ms 发送一次 (~10Hz) */
#define GW_UPLOAD_INTERVAL_MS  100U
static uint32_t gw_last_upload_ms = 0;

void hal_entry(void)
{
    /* ... 现有初始化代码 (UART9/SCI9 视觉, UART2/SCI2 雷达, UART5/SCI5 舵机) ... */

    /* 新增: 打开 SCI3 (UART3) 用于 ESP32 网关通信 */
    R_SCI_UART_Open(&g_uart3_ctrl, &g_uart3_cfg);

    TrackingController_t tracker;
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

    Tracking_Init(&tracker, &track_config);
    Tracking_Reset(&tracker);

    VisionData_t  vision_data = {0};
    LD2450_Data_t radar_data  = {0};

    while (1)
    {
        R_IWDT_Refresh(&g_iwdt0_ctrl);
        uint32_t now = g_sys_run_time_ms;

        /* === 读取传感器数据 (由 ISR 回调更新) === */
        OpenMV_GetData(&vision_data);
        LD2450_GetData(&radar_data);

        /* 视觉 UART 失败或通信超时 → 强制 LOST */
        if (!OpenMV_IsCommOK()) {
            vision_data.status = VISION_STATUS_LOST;
        }

        /* === 多模态融合 + PID + 舵机控制 === */
        Tracking_UpdateMultiModal(&tracker, &vision_data, &radar_data, now);

        /* === 边界钳位 (双重保险) === */
        apply_boundary_clamp(&tracker);

        /* === SCI3 网关上传 (节流 10Hz) === */
        if ((now - gw_last_upload_ms) >= GW_UPLOAD_INTERVAL_MS) {
            gw_last_upload_ms = now;
            Gateway_SendTrackingJSON(&tracker, &vision_data, &radar_data);
        }

        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);  /* 1ms 主循环 */
    }
}

#endif  /* 集成参考 */
