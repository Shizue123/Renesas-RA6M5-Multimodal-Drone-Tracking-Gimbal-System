# 核心代码精炼摘录

本文档仅保留各模块最核心的算法与控制逻辑，适合论文正文或答辩 PPT 直接引用。

## 1. 视觉侧

源码位置：`Drone_Tracking_RA6M5/openmv_scripts/main_tracking.py`

### 1.1 FOMO 后处理核心

```python
def make_fomo_post_process(threshold_list):
    def fomo_post_process(model, inputs, outputs):
        out_b, out_h, out_w, out_c = model.output_shape[0]
        results = [[] for _ in range(out_c)]
        for cls_idx in range(out_c):
            heatmap = image.Image(outputs[0][0, :, :, cls_idx] * 255)
            blobs = heatmap.find_blobs(threshold_list, x_stride=1, y_stride=1,
                                       area_threshold=1, pixels_threshold=1)
            for b in blobs:
                rx, ry, rw, rh = b.rect()
                score = heatmap.get_statistics(thresholds=threshold_list, roi=b.rect()).l_mean() / 255.0
                results[cls_idx].append((rx, ry, rw, rh, score))
        return results
    return fomo_post_process
```

### 1.2 移动平均与 Kalman 预测核心

```python
class MovingAverageFilter:
    def update(self, x, y):
        self.x_buf.append(x)
        self.y_buf.append(y)
        if len(self.x_buf) > self.window_size:
            self.x_buf.pop(0)
            self.y_buf.pop(0)
        return int(sum(self.x_buf) / len(self.x_buf)), int(sum(self.y_buf) / len(self.y_buf))


class SimpleKalmanFilter:
    def update(self, measured_x, measured_y):
        if not self.initialized:
            self.x, self.y = float(measured_x), float(measured_y)
            self.initialized = True
            return int(self.x), int(self.y)

        pred_x, pred_y = self.x + self.vx, self.y + self.vy
        pred_px, pred_py = self.px + self.q, self.py + self.q
        kx, ky = pred_px / (pred_px + self.r), pred_py / (pred_py + self.r)

        self.x = pred_x + kx * (measured_x - pred_x)
        self.y = pred_y + ky * (measured_y - pred_y)
        self.px = (1.0 - kx) * pred_px
        self.py = (1.0 - ky) * pred_py
        self.vx = self.x - pred_x
        self.vy = self.y - pred_y
        return int(self.x), int(self.y)

    def predict(self):
        if not self.initialized:
            return CENTER_X, CENTER_Y
        self.x, self.y = self.x + self.vx, self.y + self.vy
        return int(self.x), int(self.y)
```

### 1.3 检测、滤波、预测与显示核心

```python
detections = model.predict([img], callback=callback_fn)

if best is not None:
    raw_x, raw_y, w, h, score = best
    ma_x, ma_y = ma_filter.update(raw_x, raw_y)
    smooth_x, smooth_y = kf_filter.update(ma_x, ma_y)
    miss_count = 0
    lock_count += 1
    status = STATUS_LOCKED if lock_count >= LOCK_THRESHOLD_FRAMES else STATUS_LOST
else:
    miss_count += 1
    lock_count = 0
    if miss_count <= MAX_PREDICT_FRAMES and kf_filter.initialized:
        smooth_x, smooth_y = kf_filter.predict()
        status = STATUS_PREDICTED
    else:
        smooth_x, smooth_y = CENTER_X, CENTER_Y
        status = STATUS_LOST
        ma_filter.reset()
        kf_filter.reset()

if status == STATUS_LOCKED:
    img.draw_cross(smooth_x, smooth_y, color=255, size=8)
elif status == STATUS_PREDICTED:
    img.draw_circle(smooth_x, smooth_y, 8, color=200)
else:
    img.draw_cross(CENTER_X, CENTER_Y, color=128, size=8)
```

## 2. 雷达侧

源码位置：`Drone_Tracking_RA6M5/src/drivers/ld2450_driver.c`

说明：`ld2450_driver.c` 中仅包含状态机解析与 IIR 平滑；雷达 Kalman 位于控制器文件中。

### 2.1 LD2450 状态机解析核心

```c
void LD2450_ParseByte(uint8_t byte)
{
    switch (g_parse_state)
    {
        case LD2450_STATE_IDLE:
            if (byte == LD2450_FRAME_HEADER1) g_parse_state = LD2450_STATE_WAIT_HEADER2;
            break;

        case LD2450_STATE_WAIT_HEADER2:
            if (byte == LD2450_FRAME_HEADER2) {
                g_rx_index = 0;
                g_parse_state = LD2450_STATE_RECV_DATA;
            } else {
                reset_parser();
            }
            break;

        case LD2450_STATE_RECV_DATA:
            g_rx_buffer[g_rx_index++] = byte;
            if (g_rx_index >= LD2450_DATA_LEN) g_parse_state = LD2450_STATE_WAIT_FOOTER1;
            break;

        case LD2450_STATE_WAIT_FOOTER1:
            g_parse_state = (byte == LD2450_FRAME_FOOTER1) ? LD2450_STATE_WAIT_FOOTER2 : LD2450_STATE_IDLE;
            break;

        case LD2450_STATE_WAIT_FOOTER2:
            if (byte == LD2450_FRAME_FOOTER2) parse_and_store();
            reset_parser();
            break;
    }
}
```

### 2.2 IIR 平滑核心

```c
for (uint8_t i = 0; i < LD2450_MAX_TARGETS; i++)
{
    p_target_buf->targets[i].x = (int16_t)raw_x;
    p_target_buf->targets[i].y = (int16_t)((int32_t)raw_y - 32768);

    if (p_target_buf->targets[i].y >= 0 && p_target_buf->targets[i].y <= 6000)
    {
        float raw_xf = (float)p_target_buf->targets[i].x;
        float raw_yf = (float)p_target_buf->targets[i].y;
        if (!g_iir_primed) {
            g_iir_x[i] = raw_xf;
            g_iir_y[i] = raw_yf;
        } else {
            g_iir_x[i] = LD2450_IIR_ALPHA * raw_xf + (1.0f - LD2450_IIR_ALPHA) * g_iir_x[i];
            g_iir_y[i] = LD2450_IIR_ALPHA * raw_yf + (1.0f - LD2450_IIR_ALPHA) * g_iir_y[i];
        }
        p_target_buf->targets[i].x = (int16_t)g_iir_x[i];
        p_target_buf->targets[i].y = (int16_t)g_iir_y[i];
    }
}
```

### 2.3 雷达 Kalman 核心

源码位置：`Drone_Tracking_RA6M5/src/control/tracking_controller.c`

```c
static void radar_kf_predict(float dt_s, float q_proc)
{
    if (!g_radar_kf.initialized) return;
    g_radar_kf.x += g_radar_kf.vx * dt_s;
    g_radar_kf.y += g_radar_kf.vy * dt_s;
    g_radar_kf.pxx += (q_proc + fabsf(g_radar_kf.vx) * 0.02f) * dt_s;
    g_radar_kf.pyy += (q_proc + fabsf(g_radar_kf.vy) * 0.02f) * dt_s;
}

static void radar_kf_update(float mx, float my, float dt_s, float q_proc, float r_meas)
{
    if (!g_radar_kf.initialized)
    {
        g_radar_kf.x = mx;
        g_radar_kf.y = my;
        g_radar_kf.vx = 0.0f;
        g_radar_kf.vy = 0.0f;
        g_radar_kf.initialized = true;
        return;
    }

    radar_kf_predict(dt_s, q_proc);
    float kx = g_radar_kf.pxx / (g_radar_kf.pxx + r_meas);
    float ky = g_radar_kf.pyy / (g_radar_kf.pyy + r_meas);
    float ex = mx - g_radar_kf.x;
    float ey = my - g_radar_kf.y;
    g_radar_kf.x += kx * ex;
    g_radar_kf.y += ky * ey;
    g_radar_kf.vx += 0.35f * ex;
    g_radar_kf.vy += 0.35f * ey;
}
```

## 3. 控制侧

源码位置：`Drone_Tracking_RA6M5/src/control/tracking_controller.c`

状态映射：

- `STATE_SEARCHING`：搜索
- `STATE_RADAR_OUTER_LOOP`：粗瞄
- `STATE_VISION_INNER_LOOP`：精瞄
- `STATE_VISION_COASTING`：盲滑

### 3.1 PID 核心参数

```c
config->pan_pid_params.kp = 2.4f;
config->pan_pid_params.ki = 0.08f;
config->pan_pid_params.kd = 0.6f;
config->pan_pid_params.error_lpf_alpha = 0.4f;
config->pan_pid_params.d_lpf_alpha = 0.2f;

config->tilt_pid_params.kp = 2.0f;
config->tilt_pid_params.ki = 0.06f;
config->tilt_pid_params.kd = 0.48f;
config->tilt_pid_params.error_lpf_alpha = 0.4f;
config->tilt_pid_params.d_lpf_alpha = 0.2f;
```

### 3.2 状态机核心

```c
switch (ctrl->status.state)
{
    case STATE_SEARCHING:
        if (radar_valid) {
            ctrl->status.state = STATE_RADAR_OUTER_LOOP;
            break;
        }
        if (vision_valid) {
            ctrl->status.state = STATE_VISION_INNER_LOOP;
            break;
        }
        break;

    case STATE_RADAR_OUTER_LOOP:
        if (!radar_valid) {
            ctrl->status.state = STATE_SEARCHING;
            break;
        }
        pan_error = radar_x_mm * compute_dynamic_mm_to_px(radar_y_mm);
        delta_pan = PID_ComputeIncremental(&ctrl->pan_pid, 0.0f, pan_error) + feedforward_pan;
        delta_tilt = PID_ComputeIncremental(&ctrl->tilt_pid, 0.0f, tilt_error);
        break;

    case STATE_VISION_INNER_LOOP:
        if (!vision_valid) {
            ctrl->status.state = STATE_VISION_COASTING;
            ctrl->coasting_start_time = current_time;
            break;
        }
        delta_pan = PID_ComputeIncremental(&ctrl->pan_pid, 0.0f, pan_error);
        delta_tilt = PID_ComputeIncremental(&ctrl->tilt_pid, 0.0f, tilt_error);
        ctrl->step_cache_pan = delta_pan / frame_interval;
        ctrl->step_cache_tilt = delta_tilt / frame_interval;
        break;

    case STATE_VISION_COASTING:
        ctrl->coasting_accum_pan += ctrl->last_valid_velocity_pan * (dt_s / 0.033f);
        ctrl->coasting_accum_tilt += ctrl->last_valid_velocity_tilt * (dt_s / 0.033f);
        if (((uint32_t)(current_time - ctrl->coasting_start_time)) >= tc->coasting_timeout_ms)
            ctrl->status.state = radar_valid ? STATE_RADAR_OUTER_LOOP : STATE_SEARCHING;
        break;
}
```

### 3.3 轨迹整形核心

```c
pan_des_vel = ((float)ctrl->status.pan_position - (float)ctrl->shaping_last_pan) / dt_s;
tilt_des_vel = ((float)ctrl->status.tilt_position - (float)ctrl->shaping_last_tilt) / dt_s;

max_dv_pan = ctrl->runtime_cfg.pan_max_acc * dt_s;
max_dv_tilt = ctrl->runtime_cfg.tilt_max_acc * dt_s;

if (pan_des_vel > (ctrl->shaping_last_pan_vel + max_dv_pan))
    pan_vel_limited = ctrl->shaping_last_pan_vel + max_dv_pan;
if (pan_des_vel < (ctrl->shaping_last_pan_vel - max_dv_pan))
    pan_vel_limited = ctrl->shaping_last_pan_vel - max_dv_pan;

if (tilt_des_vel > (ctrl->shaping_last_tilt_vel + max_dv_tilt))
    tilt_vel_limited = ctrl->shaping_last_tilt_vel + max_dv_tilt;
if (tilt_des_vel < (ctrl->shaping_last_tilt_vel - max_dv_tilt))
    tilt_vel_limited = ctrl->shaping_last_tilt_vel - max_dv_tilt;
```

## 4. 一句话概括

1. 视觉侧：FOMO 检测后，通过移动平均和 Kalman 预测稳定目标位置，并在 OpenMV IDE 中显示锁定点或预测点。
2. 雷达侧：LD2450 通过逐字节状态机完成帧解析，再用 IIR 低通抑制抖动。
3. 控制侧：通过搜索、粗瞄、精瞄、盲滑四态切换，结合 PID 与轨迹整形实现稳定云台跟踪。