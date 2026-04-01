/**
 * @file pid_controller.h
 * @brief 增量式PID控制器 (带死区、积分限幅、积分分离)
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID参数结构体
 */
typedef struct {
    float kp;                      /* 比例系数 */
    float ki;                      /* 积分系数 */
    float kd;                      /* 微分系数 */
    float deadzone;                /* 死区范围 */
    float output_max;              /* 输出上限 */
    float output_min;              /* 输出下限 */
    float integral_max;            /* 积分限幅 */
    float integral_separation;     /* 积分分离阈值 */
    float error_lpf_alpha;         /* 误差低通滤波系数 (0~1, 1为不滤波) */
    float d_lpf_alpha;             /* 微分项低通滤波系数 (0~1, 削弱D项突变) */
} PID_Params_t;

/**
 * @brief PID控制器状态结构体
 */
typedef struct {
    float error_last;              /* 上次误差 */
    float error_last_last;         /* 上上次误差 */
    float error_filtered_last;     /* 上次低通滤波后的误差 */
    float derivative_last;         /* 上次微分项 (用于D项滤波) */
    float integral;                /* 积分累积值 */
    float output_last;             /* 上次输出 */
} PID_State_t;

/**
 * @brief PID控制器结构体
 */
typedef struct {
    PID_Params_t params;
    PID_State_t state;
    bool initialized;
} PID_Controller_t;

/**
 * @brief 初始化PID控制器
 */
void PID_Init(PID_Controller_t *pid, const PID_Params_t *params);

/**
 * @brief 重置PID控制器
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief 增量式PID计算
 * @param pid PID控制器指针
 * @param setpoint 目标值
 * @param feedback 反馈值
 * @return 增量输出值(相对上次的变化量)
 */
float PID_ComputeIncremental(PID_Controller_t *pid, float setpoint, float feedback);

/**
 * @brief 判断误差是否在死区内
 */
bool PID_IsInDeadzone(const PID_Controller_t *pid, float error);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
