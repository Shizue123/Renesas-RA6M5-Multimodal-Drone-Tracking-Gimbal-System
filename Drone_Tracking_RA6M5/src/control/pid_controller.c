/**
 * @file pid_controller.c
 * @brief 增量式PID控制器实现
 */

#include "pid_controller.h"
#include <string.h>
#include <math.h>

/**
 * @brief 限幅函数
 */
static float clamp_float(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void PID_Init(PID_Controller_t *pid, const PID_Params_t *params)
{
    if (pid == NULL) return;
    
    if (params != NULL) {
        memcpy(&pid->params, params, sizeof(PID_Params_t));
    } else {
        memset(&pid->params, 0, sizeof(PID_Params_t));
    }
    
    memset(&pid->state, 0, sizeof(PID_State_t));
    pid->initialized = true;
}

void PID_Reset(PID_Controller_t *pid)
{
    if (pid == NULL || !pid->initialized) return;
    memset(&pid->state, 0, sizeof(PID_State_t));
}

bool PID_IsInDeadzone(const PID_Controller_t *pid, float error)
{
    if (pid == NULL) return false;
    return fabsf(error) < pid->params.deadzone;
}

float PID_ComputeIncremental(PID_Controller_t *pid, float setpoint, float feedback)
{
    if (pid == NULL || !pid->initialized) return 0.0f;
    
    /* 1. 获取原始误差 */
    float error_raw = setpoint - feedback;
    
    /* 2. 误差低通滤波 (IIR) */
    float alpha_e = (pid->params.error_lpf_alpha > 0.0f && pid->params.error_lpf_alpha <= 1.0f) ? pid->params.error_lpf_alpha : 1.0f;
    float error = alpha_e * error_raw + (1.0f - alpha_e) * pid->state.error_filtered_last;
    pid->state.error_filtered_last = error;
    
    /* 3. 死区处理: 处于死区内直接清零增量，并将历史误差对齐 */
    if (PID_IsInDeadzone(pid, error)) {
        pid->state.error_last_last = pid->state.error_last;
        pid->state.error_last = error;
        pid->state.derivative_last = 0.0f;
        return 0.0f;
    }
    
    /* 4. 积分分离: 误差较大时不进行积分累积 */
    if (fabsf(error) < pid->params.integral_separation) {
        pid->state.integral += error;
        pid->state.integral = clamp_float(pid->state.integral,
                                         -pid->params.integral_max,
                                         pid->params.integral_max);
    }
    
    /* 5. 计算微分项，并应用低通滤波削弱D项突变(Derivative Kick) */
    float derivative_raw = error - pid->state.error_last;
    float alpha_d = (pid->params.d_lpf_alpha > 0.0f && pid->params.d_lpf_alpha <= 1.0f) ? pid->params.d_lpf_alpha : 1.0f;
    float derivative = alpha_d * derivative_raw + (1.0f - alpha_d) * pid->state.derivative_last;
    
    /* 6. 增量式PID公式计算 
     * P项: Kp * (e(k) - e(k-1))
     * I项: Ki * e(k)
     * D项: Kd * (d(k) - d(k-1)) 差分后平滑
     */
    float p_term = pid->params.kp * (error - pid->state.error_last);
    float i_term = pid->params.ki * error;
    float d_term = pid->params.kd * (derivative - pid->state.derivative_last);
    
    float delta_output = p_term + i_term + d_term;
    
    /* 限幅 */
    delta_output = clamp_float(delta_output, pid->params.output_min, pid->params.output_max);
    
    /* 更新历史状态 */
    pid->state.error_last_last = pid->state.error_last;
    pid->state.error_last = error;
    pid->state.derivative_last = derivative;
    pid->state.output_last = delta_output;
    
    return delta_output;
}
