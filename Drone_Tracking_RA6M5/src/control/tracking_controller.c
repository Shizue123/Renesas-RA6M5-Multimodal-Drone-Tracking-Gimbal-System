/**
 * @file tracking_controller.c
 * @brief 云台追踪控制器实现
 */

#include "tracking_controller.h"
#include "../drivers/st3215_driver.h"
#include "../openmv_drv.h"
#include <string.h>
#include <math.h>

extern volatile uint32_t g_sys_run_time_ms;

/** 舵机下发速率限制周期 (ms)，对应 50 Hz */
#define SERVO_TX_PERIOD_MS  20U

static const char* g_state_names[] = {
    "STATE_SEARCHING", "STATE_RADAR_OUTER_LOOP", "STATE_VISION_INNER_LOOP", "STATE_VISION_COASTING"
};

static uint16_t clamp_u16(long value, uint16_t min, uint16_t max)
{
    if (value < (long)min) return min;
    if (value > (long)max) return max;
    return (uint16_t)value;
}

static uint16_t approach_u16(uint16_t current, uint16_t target, uint16_t step)
{
    if (current < target)
    {
        uint16_t delta = (uint16_t)(target - current);
        return (delta > step) ? (uint16_t)(current + step) : target;
    }
    if (current > target)
    {
        uint16_t delta = (uint16_t)(current - target);
        return (delta > step) ? (uint16_t)(current - step) : target;
    }
    return current;
}

void Tracking_GetDefaultConfig(TrackingConfig_t *config)
{
    if (config == NULL) return;
    
    /* 水平轴PID参数 */
    config->pan_pid_params.kp = 2.4f;   // scaled by 0.4 due to FOV fix (96->240)
    config->pan_pid_params.ki = 0.08f;
    config->pan_pid_params.kd = 0.6f;   // scaled by 0.4 due to FOV fix (96->240)
    config->pan_pid_params.deadzone = 8.0f;
    config->pan_pid_params.output_max = 200.0f;
    config->pan_pid_params.output_min = -200.0f;
    config->pan_pid_params.integral_max = 100.0f;
    config->pan_pid_params.integral_separation = 60.0f;
    config->pan_pid_params.error_lpf_alpha = 0.4f; /* 引入 IIR 低通滤波 (0.4) 平滑误差 */
    config->pan_pid_params.d_lpf_alpha = 0.2f;     /* 极强地平滑微分项，防止相机掉帧引发突跳 */
    
    /* 俯仰轴PID参数 */
    config->tilt_pid_params.kp = 2.0f;   // scaled by 0.4 due to FOV fix (96->240)
    config->tilt_pid_params.ki = 0.06f;
    config->tilt_pid_params.kd = 0.48f;  // scaled by 0.4 due to FOV fix (96->240)
    config->tilt_pid_params.deadzone = 8.0f;
    config->tilt_pid_params.output_max = 150.0f;
    config->tilt_pid_params.output_min = -150.0f;
    config->tilt_pid_params.integral_max = 80.0f;
    config->tilt_pid_params.integral_separation = 50.0f;
    config->tilt_pid_params.error_lpf_alpha = 0.4f;
    config->tilt_pid_params.d_lpf_alpha = 0.2f;
    
    /* 舵机配置 */
    config->pan_servo_id = GIMBAL_PAN_SERVO_ID;
    config->tilt_servo_id = GIMBAL_TILT_SERVO_ID;
    config->pan_center = 2047;
    config->tilt_center = 2047;
    config->pan_min = PAN_PHYS_MIN_POS;
    config->pan_max = PAN_PHYS_MAX_POS;
    config->tilt_min = TILT_PHYS_MIN_POS;
    config->tilt_max = TILT_PHYS_MAX_POS;
    
    /* 追踪参数 */
    config->error_deadzone = 10.0f;
    config->lost_timeout_ms = 500;
    config->servo_move_time = 0;
    config->invert_pan = false;
    config->invert_tilt = false;

    /* 多模态融合默认参数 */
    config->fusion.vision_weight_pan      = 0.9f;
    config->fusion.radar_weight_pan       = 0.1f;
    config->fusion.near_distance_mm       = 1000.0f;
    config->fusion.far_distance_mm        = 3000.0f;
    config->fusion.near_gain_scale        = 0.4f;
    config->fusion.far_gain_scale         = 1.3f;
    config->fusion.conflict_slow_factor   = 0.3f;
    config->fusion.zero_speed_attenuation = 0.15f;
    config->fusion.radar_dir_threshold    = 50;
    config->fusion.zero_speed_thresh      = 5;
    config->fusion.radar_stale_ms         = 300U;
}

static void tracker_set_runtime_defaults(TrackerConfig_t *cfg)
{
    if (cfg == NULL) return;

    cfg->max_allowed_deviation_deg = TRACKER_DEF_ANGLE_GATE_DEG;
    cfg->vision_confirm_frames = TRACKER_DEF_VISION_CONFIRM_FRAMES;
    cfg->coasting_timeout_ms = TRACKER_DEF_COASTING_TIMEOUT_MS;
    cfg->ghost_timeout_ms = TRACKER_DEF_GHOST_TIMEOUT_MS;
    cfg->kf_q = TRACKER_DEF_KF_Q;
    cfg->kf_r = TRACKER_DEF_KF_R;
    cfg->vision_pid_kp = TRACKER_DEF_VISION_PID_KP;
    cfg->vision_pid_ki = TRACKER_DEF_VISION_PID_KI;
    cfg->vision_pid_kd = TRACKER_DEF_VISION_PID_KD;
    cfg->pan_max_vel = TRACKER_DEF_PAN_MAX_VEL;
    cfg->pan_max_acc = TRACKER_DEF_PAN_MAX_ACC;
    cfg->tilt_max_vel = TRACKER_DEF_TILT_MAX_VEL;
    cfg->tilt_max_acc = TRACKER_DEF_TILT_MAX_ACC;
}

void Tracking_Init(TrackingController_t *ctrl, const TrackingConfig_t *config)
{
    if (ctrl == NULL) return;
    
    if (config != NULL) {
        memcpy(&ctrl->config, config, sizeof(TrackingConfig_t));
    } else {
        Tracking_GetDefaultConfig(&ctrl->config);
    }
    
    PID_Init(&ctrl->pan_pid, &ctrl->config.pan_pid_params);
    PID_Init(&ctrl->tilt_pid, &ctrl->config.tilt_pid_params);

    /* 保存 PID 基准参数 (深度缩放的基线) */
    memcpy(&ctrl->pan_pid_base_params,  &ctrl->config.pan_pid_params,  sizeof(PID_Params_t));
    memcpy(&ctrl->tilt_pid_base_params, &ctrl->config.tilt_pid_params, sizeof(PID_Params_t));

    memset(&ctrl->status, 0, sizeof(TrackingStatus_t));
    ctrl->status.state = STATE_SEARCHING;
    ctrl->status.pan_position = ctrl->config.pan_center;
    ctrl->status.tilt_position = ctrl->config.tilt_center;
    ctrl->last_radar_timestamp = 0U;
    ctrl->vision_agree_count = 0U;
    ctrl->coasting_start_time = 0U;
    ctrl->ghost_start_time = 0U;
    ctrl->search_step_time = 0U;
    ctrl->search_pan_dir = 1;
    ctrl->search_pitch_index = 0;
    ctrl->last_valid_velocity_pan = 0.0f;
    ctrl->last_valid_velocity_tilt = 0.0f;
    ctrl->coasting_accum_pan = 0.0f;
    ctrl->coasting_accum_tilt = 0.0f;
    ctrl->last_vision_timestamp = 0U;
    ctrl->step_cache_pan = 0.0f;
    ctrl->step_cache_tilt = 0.0f;
    tracker_set_runtime_defaults(&ctrl->runtime_cfg);
    ctrl->shaping_last_pan = ctrl->status.pan_position;
    ctrl->shaping_last_tilt = ctrl->status.tilt_position;
    ctrl->shaping_last_pan_vel = 0.0f;
    ctrl->shaping_last_tilt_vel = 0.0f;
    ctrl->shaping_last_time = 0UL;
    ctrl->shaping_initialized = false;

    /* 视觉内环 PID 可运行时调参，初始化时先应用默认值 */
    ctrl->pan_pid.params.kp = ctrl->runtime_cfg.vision_pid_kp;
    ctrl->pan_pid.params.ki = ctrl->runtime_cfg.vision_pid_ki;
    ctrl->pan_pid.params.kd = ctrl->runtime_cfg.vision_pid_kd;
    ctrl->tilt_pid.params.kp = ctrl->runtime_cfg.vision_pid_kp;
    ctrl->tilt_pid.params.ki = ctrl->runtime_cfg.vision_pid_ki;
    ctrl->tilt_pid.params.kd = ctrl->runtime_cfg.vision_pid_kd;
    
    ctrl->initialized = true;
}

void Tracking_Reset(TrackingController_t *ctrl)
{
    if (ctrl == NULL || !ctrl->initialized) return;
    
    PID_Reset(&ctrl->pan_pid);
    PID_Reset(&ctrl->tilt_pid);
    
    ctrl->status.pan_position = ctrl->config.pan_center;
    ctrl->status.tilt_position = ctrl->config.tilt_center;
    ctrl->status.state = STATE_SEARCHING;
    ctrl->status.error_x = 0.0f;
    ctrl->status.error_y = 0.0f;
    ctrl->vision_agree_count = 0U;
    ctrl->ghost_start_time = 0U;
    ctrl->search_step_time = 0U;
    ctrl->search_pan_dir = 1;
    ctrl->search_pitch_index = 0;
    ctrl->last_valid_velocity_pan = 0.0f;
    ctrl->last_valid_velocity_tilt = 0.0f;
    ctrl->coasting_accum_pan = 0.0f;
    ctrl->coasting_accum_tilt = 0.0f;
    ctrl->last_vision_timestamp = 0U;
    ctrl->step_cache_pan = 0.0f;
    ctrl->step_cache_tilt = 0.0f;
    ctrl->shaping_last_pan = ctrl->status.pan_position;
    ctrl->shaping_last_tilt = ctrl->status.tilt_position;
    ctrl->shaping_last_pan_vel = 0.0f;
    ctrl->shaping_last_tilt_vel = 0.0f;
    ctrl->shaping_last_time = 0UL;
    ctrl->shaping_initialized = false;
    
    radar_kf_reset();

    uint8_t ids[2] = { ctrl->config.pan_servo_id, ctrl->config.tilt_servo_id };
    uint16_t positions[2] = { ctrl->status.pan_position, ctrl->status.tilt_position };
    ST3215_SyncSetPosition(ids, positions, 2, 500);
}

void Tracking_UpdateController(TrackingController_t *ctrl, unsigned char vision_status,
                               short target_x, short target_y, unsigned long current_time)
{
    if (ctrl == NULL || !ctrl->initialized) return;

    ctrl->status.last_update_time = current_time;
    ctrl->status.error_x = (float)(target_x - OPENMV_FRAME_CENTER_X);
    ctrl->status.error_y = (float)(target_y - OPENMV_FRAME_CENTER_Y);

    if (ctrl->config.invert_pan)  ctrl->status.error_x = -ctrl->status.error_x;
    if (ctrl->config.invert_tilt) ctrl->status.error_y = -ctrl->status.error_y;

    if (vision_status == VISION_STATUS_LOCKED)
    {
        float delta_pan = PID_ComputeIncremental(&ctrl->pan_pid, 0.0f, ctrl->status.error_x);
        float delta_tilt = PID_ComputeIncremental(&ctrl->tilt_pid, 0.0f, ctrl->status.error_y);

        ctrl->status.pan_position = clamp_u16((long)ctrl->status.pan_position + (long)delta_pan,
                                              ctrl->config.pan_min, ctrl->config.pan_max);
        ctrl->status.tilt_position = clamp_u16((long)ctrl->status.tilt_position + (long)delta_tilt,
                                               ctrl->config.tilt_min, ctrl->config.tilt_max);
        ctrl->status.state = STATE_VISION_INNER_LOOP;
    }
    else
    {
        ctrl->status.state = STATE_SEARCHING;
    }

    /* ---- 50 Hz 速率限制: 仅当距上次下发 >= 20 ms 才真正发送 ---- */
    {
        static uint32_t last_simple_tx_time = 0;
        uint32_t now = g_sys_run_time_ms;
        if ((now - last_simple_tx_time) >= SERVO_TX_PERIOD_MS)
        {
            last_simple_tx_time = now;
            uint8_t ids[2] = { ctrl->config.pan_servo_id, ctrl->config.tilt_servo_id };
            uint16_t positions[2] = { ctrl->status.pan_position, ctrl->status.tilt_position };
            ST3215_SyncSetPosition(ids, positions, 2, ctrl->config.servo_move_time);
        }
    }
}

void Tracking_GetServoPositions(const TrackingController_t *ctrl,
                                uint16_t *pan_position, uint16_t *tilt_position)
{
    if (ctrl == NULL) return;
    if (pan_position != NULL) *pan_position = ctrl->status.pan_position;
    if (tilt_position != NULL) *tilt_position = ctrl->status.tilt_position;
}

const char* Tracking_GetStateName(TrackingState_e state)
{
    if (state <= STATE_VISION_COASTING) return g_state_names[state];
    return "UNKNOWN";
}

/*===========================================================================*/
/*                  双闭环 FSM: 雷达外环 + 视觉内环 + 盲滑                       */
/*===========================================================================*/

typedef struct
{
    float x;
    float y;
    float vx;
    float vy;
    float pxx;
    float pvx;
    float pyy;
    float pvy;
    uint32_t last_ms;
    bool initialized;
} RadarCVKalman_t;

static RadarCVKalman_t g_radar_kf;
static void radar_kf_reset(void);  /* forward declaration */

static void radar_kf_reset(void)
{
    memset(&g_radar_kf, 0, sizeof(g_radar_kf));
}

static void radar_kf_predict(float dt_s, float q_proc)
{
    if (!g_radar_kf.initialized) return;

    if (q_proc < 0.01f) q_proc = 0.01f;

    g_radar_kf.x += g_radar_kf.vx * dt_s;
    g_radar_kf.y += g_radar_kf.vy * dt_s;

    /* 简化协方差推进 (CV 模型, x/vx 和 y/vy 各自独立) */
    g_radar_kf.pxx += (q_proc + fabsf(g_radar_kf.vx) * 0.02f) * dt_s;
    g_radar_kf.pvx += (q_proc * 0.75f + fabsf(g_radar_kf.vx) * 0.01f) * dt_s;
    g_radar_kf.pyy += (q_proc + fabsf(g_radar_kf.vy) * 0.02f) * dt_s;
    g_radar_kf.pvy += (q_proc * 0.75f + fabsf(g_radar_kf.vy) * 0.01f) * dt_s;
}

static void radar_kf_update(float mx, float my, float dt_s, float q_proc, float r_meas)
{
    if (!g_radar_kf.initialized)
    {
        g_radar_kf.x = mx;
        g_radar_kf.y = my;
        g_radar_kf.vx = 0.0f;
        g_radar_kf.vy = 0.0f;
        g_radar_kf.pxx = 20.0f;
        g_radar_kf.pvx = 20.0f;
        g_radar_kf.pyy = 20.0f;
        g_radar_kf.pvy = 20.0f;
        g_radar_kf.initialized = true;
        return;
    }

    /* 先预测，再用测量校正 */
    radar_kf_predict(dt_s, q_proc);

    {
        if (r_meas < 1.0f) r_meas = 1.0f;
        float kx = g_radar_kf.pxx / (g_radar_kf.pxx + r_meas);
        float ky = g_radar_kf.pyy / (g_radar_kf.pyy + r_meas);

        float ex = mx - g_radar_kf.x;
        float ey = my - g_radar_kf.y;

        g_radar_kf.x += kx * ex;
        g_radar_kf.y += ky * ey;
        g_radar_kf.vx += 0.35f * ex;
        g_radar_kf.vy += 0.35f * ey;

        g_radar_kf.pxx *= (1.0f - kx);
        g_radar_kf.pyy *= (1.0f - ky);
    }
}

/**
 * @brief 动态 mm → px 映射: k(d) = FOV_px / (2 · d · tan(FOV_rad/2))
 */
static float compute_dynamic_mm_to_px(float depth_mm)
{
    if (depth_mm < FUSION_DEPTH_MIN_MM) {
        depth_mm = FUSION_DEPTH_MIN_MM;
    }
    return (float)OPENMV_FOV_PX / (2.0f * depth_mm * OPENMV_TAN_HALF_FOV);
}

static float radar_angle_deg_from_xy(float x_mm, float y_mm)
{
    if (y_mm < FUSION_DEPTH_MIN_MM) {
        y_mm = FUSION_DEPTH_MIN_MM;
    }
    return atan2f(x_mm, y_mm) * 57.29578f;
}

static float vision_angle_deg_from_error(float error_x)
{
    return error_x * (OPENMV_FOV_RAD / (float)OPENMV_FOV_PX) * 57.29578f;
}

/**
 * @brief 根据雷达深度 Y 动态缩放 PID 的 Kp / Kd
 */
static void apply_depth_scaling(TrackingController_t *ctrl, int16_t depth_mm)
{
    const FusionConfig_t *fc = &ctrl->config.fusion;
    float scale = 1.0f;

    if (depth_mm <= 0) {
        scale = fc->near_gain_scale;
    } else {
        float d = (float)depth_mm;
        if (d < fc->near_distance_mm) {
            float ratio = d / fc->near_distance_mm;
            scale = fc->near_gain_scale + (1.0f - fc->near_gain_scale) * ratio;
        } else if (d > fc->far_distance_mm) {
            float t = (d - fc->far_distance_mm) / (6000.0f - fc->far_distance_mm);
            if (t > 1.0f) t = 1.0f;
            scale = 1.0f + t * (fc->far_gain_scale - 1.0f);
        }
    }

    ctrl->pan_pid.params.kp  = ctrl->pan_pid_base_params.kp  * scale;
    ctrl->pan_pid.params.kd  = ctrl->pan_pid_base_params.kd  * scale;
    ctrl->tilt_pid.params.kp = ctrl->tilt_pid_base_params.kp * scale;
    ctrl->tilt_pid.params.kd = ctrl->tilt_pid_base_params.kd * scale;
}

static void restore_base_pid(TrackingController_t *ctrl)
{
    ctrl->pan_pid.params.kp  = ctrl->pan_pid_base_params.kp;
    ctrl->pan_pid.params.kd  = ctrl->pan_pid_base_params.kd;
    ctrl->tilt_pid.params.kp = ctrl->tilt_pid_base_params.kp;
    ctrl->tilt_pid.params.kd = ctrl->tilt_pid_base_params.kd;
}

static void send_servo_positions(TrackingController_t *ctrl)
{
    float dt_s;
    float max_dv_pan;
    float max_dv_tilt;
    float pan_des_vel;
    float tilt_des_vel;
    float pan_vel_limited;
    float tilt_vel_limited;
    float pan_delta;
    float tilt_delta;
    uint16_t safe_pan;
    uint16_t safe_tilt;

    if (!ctrl->shaping_initialized)
    {
        ctrl->shaping_last_pan = ctrl->status.pan_position;
        ctrl->shaping_last_tilt = ctrl->status.tilt_position;
        ctrl->shaping_last_pan_vel = 0.0f;
        ctrl->shaping_last_tilt_vel = 0.0f;
        ctrl->shaping_last_time = ctrl->status.last_update_time;
        ctrl->shaping_initialized = true;
    }

    dt_s = 0.01f;
    if (ctrl->shaping_last_time != 0UL)
    {
        uint32_t dt_ms = (uint32_t)(ctrl->status.last_update_time - ctrl->shaping_last_time);
        if (dt_ms > 0U)
        {
            dt_s = (float)dt_ms * 0.001f;
            if (dt_s > 0.05f) dt_s = 0.05f;
        }
    }

    if (dt_s < 0.001f) dt_s = 0.001f;

    pan_des_vel = ((float)ctrl->status.pan_position - (float)ctrl->shaping_last_pan) / dt_s;
    tilt_des_vel = ((float)ctrl->status.tilt_position - (float)ctrl->shaping_last_tilt) / dt_s;

    max_dv_pan = ctrl->runtime_cfg.pan_max_acc * dt_s;
    max_dv_tilt = ctrl->runtime_cfg.tilt_max_acc * dt_s;

    pan_vel_limited = pan_des_vel;
    if (pan_vel_limited > (ctrl->shaping_last_pan_vel + max_dv_pan)) {
        pan_vel_limited = ctrl->shaping_last_pan_vel + max_dv_pan;
    }
    if (pan_vel_limited < (ctrl->shaping_last_pan_vel - max_dv_pan)) {
        pan_vel_limited = ctrl->shaping_last_pan_vel - max_dv_pan;
    }
    if (pan_vel_limited > ctrl->runtime_cfg.pan_max_vel) {
        pan_vel_limited = ctrl->runtime_cfg.pan_max_vel;
    }
    if (pan_vel_limited < -ctrl->runtime_cfg.pan_max_vel) {
        pan_vel_limited = -ctrl->runtime_cfg.pan_max_vel;
    }

    tilt_vel_limited = tilt_des_vel;
    if (tilt_vel_limited > (ctrl->shaping_last_tilt_vel + max_dv_tilt)) {
        tilt_vel_limited = ctrl->shaping_last_tilt_vel + max_dv_tilt;
    }
    if (tilt_vel_limited < (ctrl->shaping_last_tilt_vel - max_dv_tilt)) {
        tilt_vel_limited = ctrl->shaping_last_tilt_vel - max_dv_tilt;
    }
    if (tilt_vel_limited > ctrl->runtime_cfg.tilt_max_vel) {
        tilt_vel_limited = ctrl->runtime_cfg.tilt_max_vel;
    }
    if (tilt_vel_limited < -ctrl->runtime_cfg.tilt_max_vel) {
        tilt_vel_limited = -ctrl->runtime_cfg.tilt_max_vel;
    }

    pan_delta = pan_vel_limited * dt_s;
    tilt_delta = tilt_vel_limited * dt_s;

    safe_pan = clamp_u16((long)((float)ctrl->shaping_last_pan + pan_delta),
                         ctrl->config.pan_min,
                         ctrl->config.pan_max);
    safe_tilt = clamp_u16((long)((float)ctrl->shaping_last_tilt + tilt_delta),
                          ctrl->config.tilt_min,
                          ctrl->config.tilt_max);

    ctrl->status.pan_position = safe_pan;
    ctrl->status.tilt_position = safe_tilt;
    ctrl->shaping_last_pan = safe_pan;
    ctrl->shaping_last_tilt = safe_tilt;
    ctrl->shaping_last_pan_vel = pan_vel_limited;
    ctrl->shaping_last_tilt_vel = tilt_vel_limited;
    ctrl->shaping_last_time = ctrl->status.last_update_time;

    /* ---- 50 Hz 速率限制: 仅当距上次下发 >= 20 ms 才真正发送 ---- */
    {
        static uint32_t last_servo_tx_time = 0;
        uint32_t now = g_sys_run_time_ms;
        if ((now - last_servo_tx_time) >= SERVO_TX_PERIOD_MS)
        {
            last_servo_tx_time = now;
            uint8_t ids[2] = { ctrl->config.pan_servo_id, ctrl->config.tilt_servo_id };
            uint16_t positions[2] = { safe_pan, safe_tilt };
            ST3215_SyncSetPosition(ids, positions, 2, ctrl->config.servo_move_time);
        }
    }
}

static bool try_update_pan_with_boundary_guard(TrackingController_t *ctrl, float delta_pan)
{
    long proposed = (long)ctrl->status.pan_position + (long)delta_pan;

    if (proposed < (long)PAN_PHYS_MIN_POS)
    {
        ctrl->status.pan_position = PAN_PHYS_MIN_POS;
        ctrl->status.state = STATE_SEARCHING;
        ctrl->search_pan_dir = 1; /* 原路退绕: 往右回扫 */
        ctrl->search_step_time = ctrl->status.last_update_time;
        ctrl->vision_agree_count = 0U;
        ctrl->ghost_start_time = 0U;
        send_servo_positions(ctrl);
        return false;
    }

    if (proposed > (long)PAN_PHYS_MAX_POS)
    {
        ctrl->status.pan_position = PAN_PHYS_MAX_POS;
        ctrl->status.state = STATE_SEARCHING;
        ctrl->search_pan_dir = -1; /* 原路退绕: 往左回扫 */
        ctrl->search_step_time = ctrl->status.last_update_time;
        ctrl->vision_agree_count = 0U;
        ctrl->ghost_start_time = 0U;
        send_servo_positions(ctrl);
        return false;
    }

    ctrl->status.pan_position = (uint16_t)proposed;
    return true;
}

void Tracking_UpdateMultiModal(TrackingController_t *ctrl,
                               VisionData_t *vision,
                               LD2450_Data_t *radar,
                               unsigned long current_time)
{
    int8_t best_idx = -1;
    bool radar_valid = false;
    bool vision_valid = false;
    float radar_x_mm = 0.0f;
    float radar_y_mm = (float)ctrl->config.fusion.near_distance_mm;
    float radar_angle_deg = 0.0f;
    float vision_angle_deg = 0.0f;
    float angle_diff_deg = 180.0f;
    float pan_error = 0.0f;
    float tilt_error = 0.0f;
    float dt_s;
    const FusionConfig_t *fc;
    const TrackerConfig_t *tc;

    if (ctrl == NULL || !ctrl->initialized) return;
    fc = &ctrl->config.fusion;
    tc = &ctrl->runtime_cfg;

    dt_s = 0.01f;
    if (ctrl->status.last_update_time != 0UL)
    {
        uint32_t dt_ms = (uint32_t)(current_time - ctrl->status.last_update_time);
        if (dt_ms > 0U) {
            dt_s = (float)dt_ms * 0.001f;
            if (dt_s > 0.05f) dt_s = 0.05f;
        }
    }
    ctrl->status.last_update_time = current_time;

    float vis_err_x = 0.0f;
    float vis_err_y = 0.0f;

    if (vision != NULL)
    {
        uint32_t age_v = (uint32_t)(current_time - vision->timestamp);
        vision_valid = (age_v < OPENMV_TIMEOUT_MS) &&
                       (vision->status == VISION_STATUS_LOCKED);

        if (vision_valid) {
            vis_err_x = (float)vision->error_x;
            vis_err_y = (float)vision->error_y;

            /* 天顶极性补偿：Tilt 低于天顶(1023)时摄像头物理倒置，
             * 视觉画面 X 轴极性翻转，需对 Pan 误差取反以防正反馈暴走 */
            if (ctrl->status.tilt_position < TILT_ZENITH_POS) {
                vis_err_x = -vis_err_x;
            }
        }
    }

    if ((radar != NULL) && (radar->valid_count > 0U))
    {
        uint32_t age_r = (uint32_t)(current_time - radar->timestamp);
        if (age_r < fc->radar_stale_ms)
        {
            int16_t min_y = 32767;
            for (uint8_t i = 0; i < LD2450_MAX_TARGETS; i++)
            {
                if (radar->targets[i].valid && radar->targets[i].y < min_y)
                {
                    min_y = radar->targets[i].y;
                    best_idx = (int8_t)i;
                }
            }
            radar_valid = (best_idx >= 0);
        }
    }

    if (radar_valid)
    {
        if ((radar != NULL) && (radar->timestamp != ctrl->last_radar_timestamp))
        {
            ctrl->last_radar_timestamp = radar->timestamp;
            radar_kf_update((float)radar->targets[best_idx].x,
                            (float)radar->targets[best_idx].y,
                            dt_s,
                            tc->kf_q,
                            tc->kf_r);
        }
        else
        {
            radar_kf_predict(dt_s, tc->kf_q);
        }

        if (g_radar_kf.initialized)
        {
            radar_x_mm = g_radar_kf.x;
            radar_y_mm = g_radar_kf.y;
        }
        else
        {
            radar_x_mm = (float)radar->targets[best_idx].x;
            radar_y_mm = (float)radar->targets[best_idx].y;
        }

        if (radar_y_mm < FUSION_DEPTH_MIN_MM) radar_y_mm = FUSION_DEPTH_MIN_MM;
        radar_angle_deg = radar_angle_deg_from_xy(radar_x_mm, radar_y_mm);
    }
    else
    {
        radar_kf_predict(dt_s, tc->kf_q);
    }

    if (vision_valid)
    {
        float vx = vis_err_x;
        if (ctrl->config.invert_pan) vx = -vx;
        vision_angle_deg = vision_angle_deg_from_error(vx);
    }

    ctrl->status.radar_angle_deg = radar_angle_deg;
    ctrl->status.vision_angle_deg = vision_angle_deg;
    if (vision_valid && radar_valid)
    {
        angle_diff_deg = fabsf(vision_angle_deg - radar_angle_deg);
    }

    switch (ctrl->status.state)
    {
        case STATE_SEARCHING:
            restore_base_pid(ctrl);
            /* 凝视打断: 搜索态高频轮询，发现目标立刻跃迁 */
            if (radar_valid)
            {
                ctrl->status.state = STATE_RADAR_OUTER_LOOP;
                ctrl->vision_agree_count = 0U;
                break;
            }
            if (vision_valid)
            {
                ctrl->status.state = STATE_VISION_INNER_LOOP;
                break;
            }

            /* 非阻塞 Step-and-Stare: 每 SEARCH_STARE_MS 执行一次步进 */
            if (ctrl->search_step_time == 0UL)
            {
                ctrl->search_step_time = current_time;
                send_servo_positions(ctrl);
                break;
            }

            if ((uint32_t)(current_time - ctrl->search_step_time) >= SEARCH_STARE_MS)
            {
                long next_pan = (long)ctrl->status.pan_position +
                                (long)(ctrl->search_pan_dir * (int16_t)SEARCH_SCAN_STEP_PAN);

                if (next_pan >= (long)PAN_PHYS_MAX_POS)
                {
                    next_pan = PAN_PHYS_MAX_POS;
                    ctrl->search_pan_dir = -1;
                }
                else if (next_pan <= (long)PAN_PHYS_MIN_POS)
                {
                    next_pan = PAN_PHYS_MIN_POS;
                    ctrl->search_pan_dir = 1;
                }

                ctrl->status.pan_position = (uint16_t)next_pan;

                /* 每次 Pan 反向时切换到下一个 Tilt 层级，循环遍历所有层 */
                if (next_pan == (long)PAN_PHYS_MAX_POS || next_pan == (long)PAN_PHYS_MIN_POS)
                {
                    ctrl->search_pitch_index = (uint8_t)((ctrl->search_pitch_index + 1U) % SEARCH_PITCH_LEVEL_COUNT);
                }
                ctrl->status.tilt_position = SEARCH_PITCH_LEVELS[ctrl->search_pitch_index];

                ctrl->status.tilt_position = clamp_u16((long)ctrl->status.tilt_position,
                                                       TILT_PHYS_MIN_POS,
                                                       TILT_PHYS_MAX_POS);

                send_servo_positions(ctrl);
                ctrl->search_step_time = current_time;
            }
            break;

        case STATE_RADAR_OUTER_LOOP:
            if (!radar_valid)
            {
                ctrl->status.state = STATE_SEARCHING;
                break;
            }

            apply_depth_scaling(ctrl, (int16_t)radar_y_mm);
            pan_error = radar_x_mm * compute_dynamic_mm_to_px(radar_y_mm);

            /* 双轴解耦: Pan 吃 X 误差，Tilt 吃 Y 误差 */
            tilt_error = vision_valid ? vis_err_y : 0.0f;
            if (ctrl->config.invert_tilt) tilt_error = -tilt_error;

            {
                float feedforward_pan = 0.0f;
                /* ω = Vx / Y (rad/s) 前馈补偿 */
                if (radar_valid && g_radar_kf.initialized && radar_y_mm > 100.0f) {
                    float omega_rad_s = g_radar_kf.vx / radar_y_mm;
                    float K_ff = 0.8f; /* 雷达环前馈增益 */
                    feedforward_pan = omega_rad_s * dt_s * (4096.0f / (2.0f * 3.14159265f)) * K_ff;

                }

                float delta_pan = PID_ComputeIncremental(&ctrl->pan_pid, 0.0f, pan_error) + feedforward_pan;
                float delta_tilt = PID_ComputeIncremental(&ctrl->tilt_pid, 0.0f, tilt_error);

                if (!try_update_pan_with_boundary_guard(ctrl, delta_pan)) {
                    break;
                }

                ctrl->status.tilt_position = clamp_u16((long)ctrl->status.tilt_position + (long)delta_tilt,
                                                       ctrl->config.tilt_min,
                                                       ctrl->config.tilt_max);
                send_servo_positions(ctrl);
            }

            if (vision_valid)
            {
                if (angle_diff_deg < tc->max_allowed_deviation_deg)
                {
                    if (ctrl->vision_agree_count < 255U) ctrl->vision_agree_count++;
                    if (ctrl->vision_agree_count >= tc->vision_confirm_frames)
                    {
                        ctrl->status.state = STATE_VISION_INNER_LOOP;
                        ctrl->vision_agree_count = 0U;
                        ctrl->ghost_start_time = 0U;
                    }
                }
                else
                {
                    ctrl->vision_agree_count = 0U;
                }
            }
            break;

        case STATE_VISION_INNER_LOOP:
            if (!vision_valid)
            {
                if (ctrl->status.lost_start_time == 0UL) {
                    ctrl->status.lost_start_time = current_time;
                }

                if ((current_time - ctrl->status.lost_start_time) < 1500UL) {
                    // 短暂丢失不到 1500ms，维持上一帧位置不动等待视觉恢复
                    // (遮挡、运动模糊、逆光闪烁等常见场景通常在此窗口内恢复)
                    send_servo_positions(ctrl);
                    break;
                }

                // 彻底物理脱锁，转移至盲滑去靠残余速度寻找
                ctrl->status.state = STATE_VISION_COASTING;
                ctrl->coasting_start_time = current_time;
                // 进入盲滑时清空累加器，防止把上一段积累的残差带入带来额外跳变
                ctrl->coasting_accum_pan = 0.0f;
                ctrl->coasting_accum_tilt = 0.0f;
                ctrl->status.lost_start_time = 0UL; // 重置
                break;
            }
            
            // 正常捕获，重置丢失计时器
            ctrl->status.lost_start_time = 0UL;

            /* ==== 视觉捕获目标后，彻底切断/忽略雷达任何数据 ==== */
            ctrl->ghost_start_time = 0U;
            restore_base_pid(ctrl);

            if (vision->timestamp != ctrl->last_vision_timestamp) {
                // ============== 采样阻断：仅在新帧时计算一次完整 PID ==============
                // 动态计算帧间隔，用于后续微插值除数
                float frame_interval = (ctrl->last_vision_timestamp == 0U)
                    ? 33.0f
                    : (float)(vision->timestamp - ctrl->last_vision_timestamp);
                // 安全限幅：对应 50fps(20ms) ~ 10fps(100ms)
                if (frame_interval < 20.0f) frame_interval = 20.0f;
                if (frame_interval > 100.0f) frame_interval = 100.0f;

                ctrl->last_vision_timestamp = vision->timestamp;

                pan_error = vis_err_x;
                tilt_error = vis_err_y;
                if (ctrl->config.invert_pan) pan_error = -pan_error;
                if (ctrl->config.invert_tilt) tilt_error = -tilt_error;

                /* ==== 引入悬停缓追特判 ==== */
                if (fabsf(pan_error) < 15.0f && fabsf(tilt_error) < 15.0f) {
                    /* 误差极小，进入悬停模式：极小 Kp，无 Kd (去除倒置抽搐和差分毛刺) */
                    ctrl->pan_pid.params.kp = tc->vision_pid_kp * 0.25f;
                    ctrl->pan_pid.params.ki = tc->vision_pid_ki * 0.5f;
                    ctrl->pan_pid.params.kd = 0.0f;
                    ctrl->tilt_pid.params.kp = tc->vision_pid_kp * 0.25f;
                    ctrl->tilt_pid.params.ki = tc->vision_pid_ki * 0.5f;
                    ctrl->tilt_pid.params.kd = 0.0f;
                } else {
                    /* 雷达断开，100% 交给视觉误差闭环 */
                    ctrl->pan_pid.params.kp = tc->vision_pid_kp;
                    ctrl->pan_pid.params.ki = tc->vision_pid_ki;
                    ctrl->pan_pid.params.kd = tc->vision_pid_kd;
                    ctrl->tilt_pid.params.kp = tc->vision_pid_kp;
                    ctrl->tilt_pid.params.ki = tc->vision_pid_ki;
                    ctrl->tilt_pid.params.kd = tc->vision_pid_kd;
                }

                // 完全不使用雷达前馈，视觉自己做主
                float delta_pan = PID_ComputeIncremental(&ctrl->pan_pid, 0.0f, pan_error);
                float delta_tilt = PID_ComputeIncremental(&ctrl->tilt_pid, 0.0f, tilt_error);

                /* ==== 加入切角绝对限幅，强行切断暴走甩头 ==== */
                /* 最大允许单帧剧烈转动约2度 (22.7步) */
                float slew_limit = 23.0f;
                if (delta_pan > slew_limit) delta_pan = slew_limit;
                if (delta_pan < -slew_limit) delta_pan = -slew_limit;
                if (delta_tilt > slew_limit) delta_tilt = slew_limit;
                if (delta_tilt < -slew_limit) delta_tilt = -slew_limit;

                // 供盲滑使用的单帧满额推力记忆
                ctrl->last_valid_velocity_pan = delta_pan;
                ctrl->last_valid_velocity_tilt = delta_tilt;

                // ============== 浮点微插值：打散为 1000Hz 小步进缓存 ==============
                // 根据实际帧间隔动态计算每毫秒发放量
                ctrl->step_cache_pan = delta_pan / frame_interval;
                ctrl->step_cache_tilt = delta_tilt / frame_interval;
            }

            // ============== 空闲周期与插值释放：不调 PID，依赖累加器释放 ==============
            if (dt_s > 0.0f) {
                /* 帧间插值超时保护: 超过 100ms 未收到新视觉帧则清零 step_cache,
                 * 防止相机丢帧/断流时残余 step_cache 持续积累导致舵机暴走卡死 */
                {
                    uint32_t ms_since_frame = (uint32_t)(current_time - ctrl->last_vision_timestamp);
                    if (ms_since_frame > 100U) {
                        ctrl->step_cache_pan = 0.0f;
                        ctrl->step_cache_tilt = 0.0f;
                    }
                }

                float slices = dt_s / 0.001f;
                ctrl->coasting_accum_pan += ctrl->step_cache_pan * slices;
                ctrl->coasting_accum_tilt += ctrl->step_cache_tilt * slices;

                int step_pan = 0;
                int step_tilt = 0;

                if (fabsf(ctrl->coasting_accum_pan) >= 1.0f) {
                    step_pan = (int)ctrl->coasting_accum_pan;
                    ctrl->coasting_accum_pan -= (float)step_pan;
                }

                if (fabsf(ctrl->coasting_accum_tilt) >= 1.0f) {
                    step_tilt = (int)ctrl->coasting_accum_tilt;
                    ctrl->coasting_accum_tilt -= (float)step_tilt;
                }

                if (step_pan != 0 || step_tilt != 0) {
                    if (!try_update_pan_with_boundary_guard(ctrl, (float)step_pan)) {
                        break;
                    }

                    ctrl->status.tilt_position = clamp_u16((long)ctrl->status.tilt_position + (long)step_tilt,
                                                           ctrl->config.tilt_min, ctrl->config.tilt_max);
                    send_servo_positions(ctrl);
                }
            }
            break;

        case STATE_VISION_COASTING:
            restore_base_pid(ctrl);

            // 空间浮点累加方案 (Route B)：
            // 保持 1000Hz 全速运行，但利用累加器收集浮点精度的小数位移
            // 当累加值绝对值大于等于 1.0 时才提取为整数步进
            if (dt_s > 0.0f) {
                float scale = dt_s / 0.033f; // 缩放因子：基于 30Hz 参考帧率
                ctrl->coasting_accum_pan += ctrl->last_valid_velocity_pan * scale;
                ctrl->coasting_accum_tilt += ctrl->last_valid_velocity_tilt * scale;

                int step_pan = 0;
                int step_tilt = 0;

                if (fabsf(ctrl->coasting_accum_pan) >= 1.0f) {
                    step_pan = (int)ctrl->coasting_accum_pan;
                    ctrl->coasting_accum_pan -= (float)step_pan;
                }

                if (fabsf(ctrl->coasting_accum_tilt) >= 1.0f) {
                    step_tilt = (int)ctrl->coasting_accum_tilt;
                    ctrl->coasting_accum_tilt -= (float)step_tilt;
                }

                if (step_pan != 0 || step_tilt != 0) {
                    if (!try_update_pan_with_boundary_guard(ctrl, (float)step_pan)) {
                        break;
                    }

                    ctrl->status.tilt_position = clamp_u16((long)ctrl->status.tilt_position + (long)step_tilt,
                                                           ctrl->config.tilt_min, ctrl->config.tilt_max);
                    send_servo_positions(ctrl);
                }
            }

            if (vision_valid && radar_valid && (angle_diff_deg < tc->max_allowed_deviation_deg))
            {
                ctrl->status.state = STATE_VISION_INNER_LOOP;
                break;
            }

            if (((uint32_t)(current_time - ctrl->coasting_start_time)) >= tc->coasting_timeout_ms)
            {
                if (radar_valid)
                {
                    ctrl->status.state = STATE_RADAR_OUTER_LOOP;
                }
                else
                {
                    ctrl->status.state = STATE_SEARCHING;
                }
            }
            break;

        default:
            ctrl->status.state = STATE_SEARCHING;
            break;
    }

    ctrl->status.error_x = pan_error;
    ctrl->status.error_y = tilt_error;

}

const TrackerConfig_t * Tracking_GetRuntimeConfig(const TrackingController_t *ctrl)
{
    if (ctrl == NULL) return NULL;
    return &ctrl->runtime_cfg;
}

void Tracking_SetRuntimeConfig(TrackingController_t *ctrl, const TrackerConfig_t *cfg)
{
    if ((ctrl == NULL) || (cfg == NULL)) return;

    ctrl->runtime_cfg = *cfg;

    if (ctrl->runtime_cfg.max_allowed_deviation_deg < 1.0f) {
        ctrl->runtime_cfg.max_allowed_deviation_deg = 1.0f;
    }
    if (ctrl->runtime_cfg.vision_confirm_frames < 1U) {
        ctrl->runtime_cfg.vision_confirm_frames = 1U;
    }
    if (ctrl->runtime_cfg.coasting_timeout_ms < 100U) {
        ctrl->runtime_cfg.coasting_timeout_ms = 100U;
    }
    if (ctrl->runtime_cfg.ghost_timeout_ms < 100U) {
        ctrl->runtime_cfg.ghost_timeout_ms = 100U;
    }
    if (ctrl->runtime_cfg.kf_q < 0.01f) {
        ctrl->runtime_cfg.kf_q = 0.01f;
    }
    if (ctrl->runtime_cfg.kf_r < 1.0f) {
        ctrl->runtime_cfg.kf_r = 1.0f;
    }
    if (ctrl->runtime_cfg.vision_pid_kp < 0.0f) {
        ctrl->runtime_cfg.vision_pid_kp = 0.0f;
    }
    if (ctrl->runtime_cfg.vision_pid_ki < 0.0f) {
        ctrl->runtime_cfg.vision_pid_ki = 0.0f;
    }
    if (ctrl->runtime_cfg.vision_pid_kd < 0.0f) {
        ctrl->runtime_cfg.vision_pid_kd = 0.0f;
    }
    if (ctrl->runtime_cfg.pan_max_vel < 10.0f) {
        ctrl->runtime_cfg.pan_max_vel = 10.0f;
    }
    if (ctrl->runtime_cfg.pan_max_acc < 10.0f) {
        ctrl->runtime_cfg.pan_max_acc = 10.0f;
    }
    if (ctrl->runtime_cfg.tilt_max_vel < 10.0f) {
        ctrl->runtime_cfg.tilt_max_vel = 10.0f;
    }
    if (ctrl->runtime_cfg.tilt_max_acc < 10.0f) {
        ctrl->runtime_cfg.tilt_max_acc = 10.0f;
    }

    ctrl->pan_pid.params.kp = ctrl->runtime_cfg.vision_pid_kp;
    ctrl->pan_pid.params.ki = ctrl->runtime_cfg.vision_pid_ki;
    ctrl->pan_pid.params.kd = ctrl->runtime_cfg.vision_pid_kd;
    ctrl->tilt_pid.params.kp = ctrl->runtime_cfg.vision_pid_kp;
    ctrl->tilt_pid.params.ki = ctrl->runtime_cfg.vision_pid_ki;
    ctrl->tilt_pid.params.kd = ctrl->runtime_cfg.vision_pid_kd;
}
