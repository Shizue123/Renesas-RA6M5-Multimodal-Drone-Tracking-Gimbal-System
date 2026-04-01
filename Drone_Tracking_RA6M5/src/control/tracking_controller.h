/**
 * @file tracking_controller.h
 * @brief 云台追踪控制器
 */

#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid_controller.h"
#include "../openmv_drv.h"
#include "../drivers/ld2450_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/*                      融合算法宏定义 (FOV-Based Dynamic Mapping)              */
/*===========================================================================*/

/** @brief OpenMV 实际输出帧水平分辨率 (pixels, 240×240) */
#define OPENMV_FOV_PX           240

/** @brief OpenMV 镜头水平视场角 (radians, ≈65°) */
#define OPENMV_FOV_RAD          1.15f

/** @brief tan(FOV_RAD / 2) 预计算常量, 避免运行时 tanf 调用 */
#define OPENMV_TAN_HALF_FOV    0.6421f

/** @brief 动态映射最小深度钳位 (mm), 防除零 */
#define FUSION_DEPTH_MIN_MM     100.0f

/** @brief 默认: 视觉接管雷达时的角度一致性阈值 (deg) */
#define TRACKER_DEF_ANGLE_GATE_DEG       15.0f

/** @brief 默认: 雷达->视觉接管连续确认帧数 */
#define TRACKER_DEF_VISION_CONFIRM_FRAMES 3U

/** @brief 默认: 视觉遮挡盲滑时长 (ms) */
#define TRACKER_DEF_COASTING_TIMEOUT_MS  800U

/** @brief 默认: 虚警持续超时 (ms) */
#define TRACKER_DEF_GHOST_TIMEOUT_MS     800U

/** @brief 默认: 卡尔曼过程噪声 Q */
#define TRACKER_DEF_KF_Q                 0.8f

/** @brief 默认: 卡尔曼测量噪声 R */
#define TRACKER_DEF_KF_R                60.0f

/** @brief 默认: 视觉内环 PID 参数 */
#define TRACKER_DEF_VISION_PID_KP        5.0f
#define TRACKER_DEF_VISION_PID_KI        0.05f
#define TRACKER_DEF_VISION_PID_KD        1.0f

/** @brief 默认轨迹整形参数 (单位: 编码计数/秒, 编码计数/秒²) */
#define TRACKER_DEF_PAN_MAX_VEL         1200.0f
#define TRACKER_DEF_PAN_MAX_ACC         4000.0f
#define TRACKER_DEF_TILT_MAX_VEL         900.0f
#define TRACKER_DEF_TILT_MAX_ACC        3000.0f

/** @brief 物理限位: Pan(下舵机) 全域 */
#define PAN_PHYS_MIN_POS                  0U
#define PAN_PHYS_MAX_POS                  4095U

/** @brief 物理限位: Tilt(上舵机) 安全区间
 *  下限设为天顶位置(1023)上方 57 步，禁止越过天顶导致极性翻转振荡 */
#define TILT_PHYS_MIN_POS                 1080U
#define TILT_PHYS_MAX_POS                 2457U

/** @brief Tilt 轴天顶位置 (编码器计数); 低于此值时摄像头物理倒置, Pan 误差极性反转 */
#define TILT_ZENITH_POS                   1023U

/** @brief Searching 扫描参数 */
#define SEARCH_SCAN_STEP_PAN              200U
#define SEARCH_STARE_MS                   250U
#define SEARCH_PITCH_LEVEL_COUNT          4U
static const uint16_t SEARCH_PITCH_LEVELS[SEARCH_PITCH_LEVEL_COUNT] = {
    2300U,   /* 接近水平 (低仰角) */
    2047U,   /* 正前方中心 */
    1700U,   /* 中高仰角 */
    1400U,   /* 高仰角 (接近天顶安全上限) */
};

/* 追踪状态枚举 */
typedef enum {
    STATE_SEARCHING = 0,          /* 全局搜索/复位 */
    STATE_RADAR_OUTER_LOOP,       /* 雷达外环粗瞄 */
    STATE_VISION_INNER_LOOP,      /* 视觉内环精瞄 */
    STATE_VISION_COASTING,        /* 视觉遮挡盲滑 */
} TrackingState_e;

/* 统一运行时配置: 所有可动态调参项 */
typedef struct {
    float max_allowed_deviation_deg; /**< 雷达视觉防欺骗角度阈值, 默认15 */
    uint16_t vision_confirm_frames;  /**< 雷达->视觉接管连续确认帧数, 默认3 */
    uint32_t coasting_timeout_ms;    /**< 盲滑时长, 默认500 */
    uint32_t ghost_timeout_ms;       /**< 虚警超时, 默认800 */
    float kf_q;                      /**< 卡尔曼过程噪声 Q */
    float kf_r;                      /**< 卡尔曼测量噪声 R */
    float vision_pid_kp;             /**< 视觉内环 PID Kp */
    float vision_pid_ki;             /**< 视觉内环 PID Ki */
    float vision_pid_kd;             /**< 视觉内环 PID Kd */
    float pan_max_vel;               /**< Pan 最大速度限制 */
    float pan_max_acc;               /**< Pan 最大加速度限制 */
    float tilt_max_vel;              /**< Tilt 最大速度限制 */
    float tilt_max_acc;              /**< Tilt 最大加速度限制 */
} TrackerConfig_t;

/* 多模态融合配置 */
typedef struct {
    float vision_weight_pan;       /**< 场景A: 视觉 Pan 权重 (默认 0.9) */
    float radar_weight_pan;        /**< 场景A: 雷达 Pan 权重 (默认 0.1) */
    float near_distance_mm;        /**< 近场阈值 (默认 1000mm) */
    float far_distance_mm;         /**< 远场阈值 (默认 3000mm) */
    float near_gain_scale;         /**< 近场 PID 增益缩放 (默认 0.4) */
    float far_gain_scale;          /**< 远场 PID 增益缩放 (默认 1.3) */
    float conflict_slow_factor;    /**< 方向冲突减速因子 (默认 0.3) */
    float zero_speed_attenuation;  /**< 场景C: 零速衰减系数 (默认 0.15, 衰减85%) */
    int16_t radar_dir_threshold;   /**< 雷达方向判定最小 X(mm) (默认 50) */
    int16_t zero_speed_thresh;     /**< 速度判零阈值 (cm/s, 默认 5) */
    uint32_t radar_stale_ms;       /**< 雷达数据过期阈值 (ms, 默认 300) */
} FusionConfig_t;

/* 追踪配置结构体 */
typedef struct {
    PID_Params_t pan_pid_params;    /* 水平轴PID参数 */
    PID_Params_t tilt_pid_params;   /* 俯仰轴PID参数 */
    FusionConfig_t fusion;          /* 多模态融合配置 */
    uint8_t  pan_servo_id;          /* 水平舵机ID */
    uint8_t  tilt_servo_id;         /* 俯仰舵机ID */
    uint16_t pan_center;            /* 水平中心位置 */
    uint16_t tilt_center;           /* 俯仰中心位置 */
    uint16_t pan_min;               /* 水平最小值 */
    uint16_t pan_max;               /* 水平最大值 */
    uint16_t tilt_min;              /* 俯仰最小值 */
    uint16_t tilt_max;              /* 俯仰最大值 */
    float    error_deadzone;        /* 误差死区(像素) */
    uint16_t lost_timeout_ms;       /* 丢失超时(ms) */
    uint16_t servo_move_time;       /* 舵机运动时间(ms) */
    bool     invert_pan;            /* 反转水平方向 */
    bool     invert_tilt;           /* 反转俯仰方向 */
} TrackingConfig_t;

/* 追踪状态结构体 */
typedef struct {
    TrackingState_e state;          /* 当前状态 */
    uint16_t pan_position;          /* 水平位置 */
    uint16_t tilt_position;         /* 俯仰位置 */
    float    error_x;               /* X轴误差 */
    float    error_y;               /* Y轴误差 */
    unsigned long last_update_time; /* 上次更新时间 */
    unsigned long lost_start_time;  /* 丢失开始时间 */
    unsigned long locked_frames;    /* 锁定帧数 */
    float radar_angle_deg;          /* 雷达角度估计 (deg) */
    float vision_angle_deg;         /* 视觉角度估计 (deg) */
} TrackingStatus_t;

/* 追踪控制器结构体 */
typedef struct {
    TrackingConfig_t config;
    TrackingStatus_t status;
    PID_Controller_t pan_pid;
    PID_Controller_t tilt_pid;
    PID_Params_t pan_pid_base_params;   /**< Pan PID 基准参数 (深度缩放基线) */
    PID_Params_t tilt_pid_base_params;  /**< Tilt PID 基准参数 (深度缩放基线) */
    uint32_t last_radar_timestamp;      /**< 最近消费的雷达时间戳 */
    uint16_t vision_agree_count;        /**< 视觉接管连续一致帧计数 */
    unsigned long coasting_start_time;  /**< 盲滑起始时间 */
    unsigned long ghost_start_time;     /**< 视觉疑似虚警起始时间 */
    unsigned long search_step_time;     /**< 搜索步进时间戳 */
    int8_t search_pan_dir;              /**< 搜索 Pan 方向 (+1/-1) */
    uint8_t search_pitch_index;         /**< 搜索 Pitch 层索引 (0..SEARCH_PITCH_LEVEL_COUNT-1) */
    float last_valid_velocity_pan;      /**< 视觉有效期末速度记忆: Pan */
    float last_valid_velocity_tilt;     /**< 视觉有效期末速度记忆: Tilt */
    float coasting_accum_pan;           /**< 盲滑期间 Pan 位置微小累加器 */
    float coasting_accum_tilt;          /**< 盲滑期间 Tilt 位置微小累加器 */
    uint32_t last_vision_timestamp;     /**< 上次处理的视觉帧时间戳 */
    float step_cache_pan;               /**< 视觉帧期间的微插值缓存: Pan */
    float step_cache_tilt;              /**< 视觉帧期间的微插值缓存: Tilt */
    TrackerConfig_t runtime_cfg;        /**< 运行时统一可调配置 */
    uint16_t shaping_last_pan;          /**< 轨迹整形上一帧 Pan 位置 */
    uint16_t shaping_last_tilt;         /**< 轨迹整形上一帧 Tilt 位置 */
    float shaping_last_pan_vel;         /**< 轨迹整形上一帧 Pan 速度 */
    float shaping_last_tilt_vel;        /**< 轨迹整形上一帧 Tilt 速度 */
    unsigned long shaping_last_time;    /**< 轨迹整形上一帧时间戳 */
    bool shaping_initialized;           /**< 轨迹整形初始化标志 */
    bool initialized;
} TrackingController_t;

/**
 * @brief 获取默认配置
 */
void Tracking_GetDefaultConfig(TrackingConfig_t *config);

/**
 * @brief 初始化追踪控制器
 */
void Tracking_Init(TrackingController_t *ctrl, const TrackingConfig_t *config);

/**
 * @brief 重置追踪控制器
 */
void Tracking_Reset(TrackingController_t *ctrl);

/**
 * @brief 更新追踪控制
 * @param ctrl 控制器指针
 * @param vision_status 视觉状态 (0=丢失, 1=锁定, 2=预测)
 * @param target_x 目标X坐标 (像素)
 * @param target_y 目标Y坐标 (像素)
 * @param current_time 当前时间(ms)
 */
void Tracking_UpdateController(TrackingController_t *ctrl, unsigned char vision_status,
                               short target_x, short target_y, unsigned long current_time);

/**
 * @brief 获取舵机位置
 */
void Tracking_GetServoPositions(const TrackingController_t *ctrl,
                                uint16_t *pan_position, uint16_t *tilt_position);

/**
 * @brief 获取状态名称
 */
const char* Tracking_GetStateName(TrackingState_e state);

/**
 * @brief 多模态融合追踪更新 (视觉 + 雷达)
 *
 * 融合策略 (五场景):
 *   Tilt 轴: 100% 视觉 (LD2450 无 Z 轴)
 *   Pan  轴: 场景A → 90%视觉+10%雷达 (双强锁定, 含方向冲突检测)
 *            场景B → 100%雷达盲跟 (视觉致盲)
 *            场景C → 雷达滤噪 (速度≈0, 视觉可能误检, 进入OBSERVING)
 *            纯视觉 → 雷达离线/过期时的回退模式
 *            场景D → 双重丢失, 超时归中
 *   动态映射: k(d) = FOV_px / (2·d·tan(θ/2)) 代替静态系数
 *   深度自适应: 雷达 Y → PID Kp/Kd Gain Scheduling
 *
 * @param ctrl         追踪控制器
 * @param vision       视觉数据 (可为最近一次的缓存)
 * @param radar        雷达数据 (可为最近一次的缓存)
 * @param current_time 当前时间 (ms)
 */
void Tracking_UpdateMultiModal(TrackingController_t *ctrl,
                               VisionData_t *vision,
                               LD2450_Data_t *radar,
                               unsigned long current_time);

/**
 * @brief 获取运行时统一配置
 */
const TrackerConfig_t * Tracking_GetRuntimeConfig(const TrackingController_t *ctrl);

/**
 * @brief 设置运行时统一配置
 */
void Tracking_SetRuntimeConfig(TrackingController_t *ctrl, const TrackerConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* TRACKING_CONTROLLER_H */
