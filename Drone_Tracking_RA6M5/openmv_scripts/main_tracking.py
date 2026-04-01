# OpenMV H7 Plus 视觉追踪脚本（96x96灰度FOMO + MA + Kalman + 8字节高精度协议）
# 协议格式: [0x55][0xAA][Status][X_H][X_L][Y_H][Y_L][Checksum]

import sensor, image, time, ml, math, gc
import uos
import ustruct as struct
from pyb import UART, LED

# =========================== 通信与状态定义 ===========================
UART_BAUDRATE = 115200
FRAME_HEADER_1 = 0x55
FRAME_HEADER_2 = 0xAA

STATUS_LOST = 0
STATUS_LOCKED = 1
STATUS_PREDICTED = 2

# =========================== 图像与模型配置 ===========================
MODEL_INPUT_W = 96
MODEL_INPUT_H = 96
CENTER_X = MODEL_INPUT_W // 2
CENTER_Y = MODEL_INPUT_H // 2

TARGET_LABEL = "drone"
LOCK_THRESHOLD_FRAMES = 3
MAX_PREDICT_FRAMES = 8
TRACKING_RADIUS = 24
MIN_CONFIDENCE_HIGH = 0.80
MIN_CONFIDENCE_LOW = 0.60

# =========================== 平滑滤波参数 ===========================
MA_WINDOW_SIZE = 4
KALMAN_Q = 0.12
KALMAN_R = 0.45

# =========================== 硬件初始化 ===========================
sensor.reset()
# 必须为96x96黑白模型使用灰度图，降低内存占用并适配模型输入
sensor.set_pixformat(sensor.GRAYSCALE)
# 使用更接近96x96的基础分辨率，再窗口裁切
sensor.set_framesize(sensor.QQVGA)  # 160x120
sensor.set_windowing((MODEL_INPUT_W, MODEL_INPUT_H))
sensor.skip_frames(time=2000)

uart = UART(3, UART_BAUDRATE, timeout_char=10)
red_led = LED(1)
green_led = LED(2)

# =========================== 滤波器定义 ===========================
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.x_buf = []
        self.y_buf = []

    def reset(self):
        self.x_buf = []
        self.y_buf = []

    def update(self, x, y):
        self.x_buf.append(x)
        self.y_buf.append(y)
        if len(self.x_buf) > self.window_size:
            self.x_buf.pop(0)
            self.y_buf.pop(0)
        avg_x = int(sum(self.x_buf) / len(self.x_buf))
        avg_y = int(sum(self.y_buf) / len(self.y_buf))
        return avg_x, avg_y


class SimpleKalmanFilter:
    def __init__(self, q, r):
        self.q = q
        self.r = r
        self.reset()

    def reset(self):
        self.initialized = False
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.px = 1.0
        self.py = 1.0

    def update(self, measured_x, measured_y):
        if not self.initialized:
            self.x = float(measured_x)
            self.y = float(measured_y)
            self.initialized = True
            return int(self.x), int(self.y)

        pred_x = self.x + self.vx
        pred_y = self.y + self.vy
        pred_px = self.px + self.q
        pred_py = self.py + self.q

        kx = pred_px / (pred_px + self.r)
        ky = pred_py / (pred_py + self.r)

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

        pred_x = int(self.x + self.vx)
        pred_y = int(self.y + self.vy)

        if pred_x < 0:
            pred_x = 0
        elif pred_x >= MODEL_INPUT_W:
            pred_x = MODEL_INPUT_W - 1

        if pred_y < 0:
            pred_y = 0
        elif pred_y >= MODEL_INPUT_H:
            pred_y = MODEL_INPUT_H - 1

        self.x = float(pred_x)
        self.y = float(pred_y)

        return pred_x, pred_y


# =========================== 协议发送 ===========================
def send_offset_to_ra6m5(err_x, err_y, status):
    # 限幅到int16
    if err_x > 32767:
        err_x = 32767
    elif err_x < -32768:
        err_x = -32768

    if err_y > 32767:
        err_y = 32767
    elif err_y < -32768:
        err_y = -32768

    x_u16 = err_x & 0xFFFF
    y_u16 = err_y & 0xFFFF

    x_h = (x_u16 >> 8) & 0xFF
    x_l = x_u16 & 0xFF
    y_h = (y_u16 >> 8) & 0xFF
    y_l = y_u16 & 0xFF

    checksum = (FRAME_HEADER_1 + FRAME_HEADER_2 + status + x_h + x_l + y_h + y_l) & 0xFF

    # > 表示Big-Endian，h为16位有符号整数
    frame = struct.pack(">BBBhhB", FRAME_HEADER_1, FRAME_HEADER_2, status, int(err_x), int(err_y), checksum)
    uart.write(frame)


# =========================== FOMO后处理 ===========================
def make_fomo_post_process(threshold_list):
    def fomo_post_process(model, inputs, outputs):
        out_b, out_h, out_w, out_c = model.output_shape[0]
        x_scale = inputs[0].roi[2] / out_w
        y_scale = inputs[0].roi[3] / out_h
        scale = min(x_scale, y_scale)
        x_offset = ((inputs[0].roi[2] - (out_w * scale)) / 2) + inputs[0].roi[0]
        y_offset = ((inputs[0].roi[3] - (out_h * scale)) / 2) + inputs[0].roi[1]

        results = [[] for _ in range(out_c)]
        for cls_idx in range(out_c):
            heatmap = image.Image(outputs[0][0, :, :, cls_idx] * 255)
            blobs = heatmap.find_blobs(threshold_list, x_stride=1, y_stride=1,
                                       area_threshold=1, pixels_threshold=1)
            for b in blobs:
                rect = b.rect()
                rx, ry, rw, rh = rect
                score = heatmap.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0
                x = int((rx * scale) + x_offset)
                y = int((ry * scale) + y_offset)
                w = int(rw * scale)
                h = int(rh * scale)
                results[cls_idx].append((x, y, w, h, score))

        return results

    return fomo_post_process


# =========================== 模型加载 ===========================
model = None
labels = []
try:
    model = ml.Model("trained.tflite",
                     load_to_fb=uos.stat("trained.tflite")[6] > (gc.mem_free() - (64 * 1024)))
    labels = [line.rstrip("\n") for line in open("labels.txt")]
    print("Model loaded")
except Exception as e:
    print("Model load failed:", e)


target_class_index = -1
for idx, label in enumerate(labels):
    if label.lower() == TARGET_LABEL.lower():
        target_class_index = idx
        break

if (model is not None) and (target_class_index < 0) and (len(labels) > 0):
    # 未找到目标标签时，默认用第一个非背景类
    target_class_index = 1 if len(labels) > 1 else 0

# =========================== 主循环变量 ===========================
ma_filter = MovingAverageFilter(MA_WINDOW_SIZE)
kf_filter = SimpleKalmanFilter(KALMAN_Q, KALMAN_R)
clock = time.clock()

lock_count = 0
miss_count = 0
last_x = CENTER_X
last_y = CENTER_Y

print("OpenMV tracking start: GRAYSCALE 96x96 + MA + Kalman + 8-byte frame")

while True:
    clock.tick()
    img = sensor.snapshot()

    if model is None:
        send_offset_to_ra6m5(0, 0, STATUS_LOST)
        red_led.on()
        green_led.off()
        continue

    current_conf = MIN_CONFIDENCE_LOW if lock_count >= LOCK_THRESHOLD_FRAMES else MIN_CONFIDENCE_HIGH
    threshold_list = [(math.ceil(current_conf * 255), 255)]
    callback_fn = make_fomo_post_process(threshold_list)

    detections = model.predict([img], callback=callback_fn)

    candidates = []
    if 0 <= target_class_index < len(detections):
        for x, y, w, h, score in detections[target_class_index]:
            if score >= current_conf:
                cx = int(x + w / 2)
                cy = int(y + h / 2)
                candidates.append((cx, cy, w, h, score))

    best = None
    if len(candidates) > 0:
        # 已锁定时优先连续性目标
        min_dist = 999999
        for cx, cy, w, h, score in candidates:
            dist = math.sqrt((cx - last_x) * (cx - last_x) + (cy - last_y) * (cy - last_y))
            if dist < min_dist and dist <= TRACKING_RADIUS * 2:
                min_dist = dist
                best = (cx, cy, w, h, score)

        # 若连续性筛选失败，回退最高置信度
        if best is None:
            best = max(candidates, key=lambda item: item[4])

    if best is not None:
        raw_x, raw_y, w, h, score = best
        ma_x, ma_y = ma_filter.update(raw_x, raw_y)
        smooth_x, smooth_y = kf_filter.update(ma_x, ma_y)

        miss_count = 0
        lock_count += 1

        if lock_count >= LOCK_THRESHOLD_FRAMES:
            status = STATUS_LOCKED
        else:
            status = STATUS_LOST
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

    last_x, last_y = smooth_x, smooth_y

    # 转为“中心偏差”后发送（16位有符号）
    err_x = int(smooth_x - CENTER_X)
    err_y = int(smooth_y - CENTER_Y)
    send_offset_to_ra6m5(err_x, err_y, status)

    if status == STATUS_LOCKED:
        green_led.on()
        red_led.off()
        img.draw_cross(smooth_x, smooth_y, color=255, size=8)
    elif status == STATUS_PREDICTED:
        green_led.toggle()
        red_led.off()
        img.draw_circle(smooth_x, smooth_y, 8, color=200)
    else:
        green_led.off()
        red_led.on()
        img.draw_cross(CENTER_X, CENTER_Y, color=128, size=8)

    img.draw_string(2, 2, "FPS:%.1f" % clock.fps(), color=255)
    img.draw_string(2, 12, "E:(%d,%d)" % (err_x, err_y), color=255)
