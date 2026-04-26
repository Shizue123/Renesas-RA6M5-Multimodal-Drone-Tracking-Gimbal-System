# 无人机多模态追踪云台系统

**[English](./README.md) | 简体中文**

[![许可证: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![语言: C | Python | MicroPython | Arduino](https://img.shields.io/badge/Languages-C%20%7C%20Python%20%7C%20MicroPython%20%7C%20Arduino-blue)](https://github.com/Shizue123/Renesas-RA6M5-Multimodal-Drone-Tracking-Gimbal-System)
[![技术栈: Renesas RA6M5 | FastAPI | TensorFlow Lite](https://img.shields.io/badge/Built%20with-Renesas%20RA6M5%20%7C%20FastAPI%20%7C%20TFLite-brightgreen)](https://github.com/Shizue123/Renesas-RA6M5-Multimodal-Drone-Tracking-Gimbal-System)

---

## 项目概览

基于 **双传感器融合**（毫米波雷达 + 机器视觉）的 **实时自主无人机追踪云台系统**，从嵌入式控制到云端看板的完整链路集成。通过 Kalman 滤波、增量 PID 控制和四态有限状态机实现超低延迟目标追踪，全程四路独立 UART 通道硬件可追踪。

---

## 核心特性

| 特性 | 技术细节 | 状态 |
|------|--------|------|
| **双传感器融合** | LD2450 毫米波雷达（6m 量程）+ OpenMV H7+ FOMO（96×96 目标检测）| ✅ 生产就绪 |
| **实时处理** | FOMO 后处理 + IIR/Kalman 滤波 + 增量 PID 控制 | ✅ 现场验证 |
| **硬件控制** | Renesas RA6M5（4 路独立 UART）+ ST-3215 舵机云台（半双工串行）| ✅ 完整集成 |
| **无线网关** | ESP32 WiFi 桥接（FreeRTOS，HTTP POST 到 FastAPI 后端）| ✅ 已部署 |
| **云端看板** | FastAPI 实时 WebSocket + SQLite 遥测日志 | ✅ 实时监控 |
| **系统诊断** | 4 态 LED 指示 + 每模块健康遥测 | ✅ 内置 |

---

## 技术栈

### 嵌入式固件
- **主控制器**: Renesas RA6M5 (ARM Cortex-M4 @ 120MHz)
- **RTOS**: FreeRTOS（抢占式多任务）
- **开发框架**: Renesas FSP
- **IDE**: e² studio
- **编程语言**: C (ISO C99)

### 机器视觉
- **平台**: OpenMV H7+ (ARM Cortex-M7 @ 216MHz)
- **模型**: TensorFlow Lite FOMO
- **编程语言**: MicroPython
- **训练**: 自定义数据集（无人机检测）

### 无线网关
- **MCU**: ESP32 (双核 Xtensa @ 240MHz)
- **固件**: Arduino Framework + FreeRTOS
- **协议**: WiFi + HTTP POST

### 云端后端
- **框架**: FastAPI (异步 Python)
- **数据库**: SQLite (遥测)
- **实时推送**: WebSocket
- **前端**: HTML5 + JavaScript + Chart.js

---

## 目录结构

| 路径 | 用途 | 技术栈 |
|------|------|--------|
| `Drone_Tracking_RA6M5/` | RA6M5 主固件工程 | C / Renesas FSP / e² studio |
| `Drone_Tracking_RA6M5/openmv_scripts/` | 机载视觉处理管线 | MicroPython / TFLite FOMO |
| `Drone_Tracking_RA6M5/src/control/` | PID 控制器 + 追踪状态机 | C 头文件 + 实现 |
| `Drone_Tracking_RA6M5/src/drivers/` | LD2450 雷达 + ST-3215 舵机驱动 | C UART 抽象层 |
| `ESP32-Competition_Deliverables/` | WiFi 网关固件 | Arduino / FreeRTOS |
| `Drone_Tracking_WebCloud/` | 云端看板后端 | Python / FastAPI / SQLite |
| `Key-document/` | 设计规格、协议文档、调参指南 | Markdown |
| `openmv-code/` | SD 卡部署文件（模型 + 标签） | MicroPython / TFLite |

---

## 快速启动

### 系统需求
- RA6M5 开发板（120MHz Cortex-M4）
- OpenMV H7+（含 OV7725 摄像头）
- LD2450 毫米波雷达（UART 接口）
- ST-3215 舵机（半双工串行）
- ESP32 开发板（标准 ESP32-WROOM-32）
- 工具: e² studio、OpenMV IDE、Arduino IDE、Python 3.9+

### 1. RA6M5 主固件
- 打开 e² studio → 导入 `Drone_Tracking_RA6M5/`
- 验证 FSP 配置（`configuration.xml`）
- 编译 → 通过 J-Link 调试器烧录
- P400 LED 应在上电后闪烁

### 2. OpenMV 视觉管线
- 准备 SD 卡：`trained.tflite`、`labels.txt`、`main.py`
- 通过 USB 连接 OpenMV H7+
- 通过 OpenMV IDE 文件管理器上传文件
- 按下 OpenMV IDE 中的播放键 → 通过 UART 输出检测坐标

### 3. ESP32 WiFi 网关
- 编辑 `esp32_async_gateway.ino` 中的 WiFi 和 CLOUD_URL
- 通过 Arduino IDE 编译和上传
- 通过串口监视器验证（115200 波特率）

### 4. 云端看板后端
```bash
cd Drone_Tracking_WebCloud
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000
```
- 管理后台: http://localhost:8000/admin.html (admin / admin123)
- 实时看板: http://localhost:8000/dashboard.html

---

## 硬件接线参考

| RA6M5 引脚 | 方向 | 外设 | 用途 | UART | 波特率 |
|-----------|------|------|------|------|--------|
| P601 (RXD9) | ← | OpenMV H7+ P4 | 视觉 RX | SCI9 | 115200 |
| P301 (RXD2) | ← | LD2450 TX | 雷达 RX | SCI2 | 256000 |
| P501 (TXD5) | → | URT-1 TX | 舵机命令 TX | SCI5 | 1000000 |
| P502 (RXD5) | ← | URT-1 RX | 舵机反馈 RX | SCI5 | 1000000 |
| P707 (TXD3) | → | ESP32 GPIO16 | 遥测 TX | SCI3 | 115200 |
| P400 | — | LED_D12 | 状态指示（低电平点亮）| GPIO | N/A |

舵机供电：URT-1 蓝色端子（V1+G）→ 12V 稳压模块 → ST-3215 三色总线。

---

## 核心算法文档

| 文档 | 覆盖内容 | 状态 |
|------|--------|------|
| [core-code.md](Key-document/core-code.md) | FOMO 后处理、Kalman 滤波、PID、FSM | ✅ 代码同步 |
| [PID调制.md](Key-document/PID调制.md) | PID 调参理论 + 本项目实战参数 | ✅ 最新 |
| [接口协议文档.md](Key-document/接口协议文档.md) | UART 帧格式、状态机转移 | ✅ 最新 |
| [硬件接线连接.md](Key-document/硬件接线连接.md) | 四路 UART 逐脚接线 | ✅ 代码同步 |

---

## LED 状态码

| 闪烁模式 | 含义 | 处理建议 |
|---------|------|--------|
| 常亮 | 视觉 + 舵机正常 | ✅ 正常运行 |
| 200ms 脉冲 | 舵机离线或仅视觉在线（搜索）| 检查舵机供电/UART 电缆 |
| 1s 周期（500ms 亮）| 仅雷达在线，无视觉 | 检查 OpenMV UART 连接 |
| 60ms 短脉冲 | 双传感器丢失（自动恢复中）| 等待 5 秒；检查所有 UART 电缆 |

---

## 性能基准

- 视觉: 30 FPS @ 96×96 FOMO (OpenMV H7+)
- 雷达: ~10 Hz 更新率 (LD2450)
- 控制循环: <50ms 延迟 (RA6M5)
- 舵机响应: ~20ms (1Mbps 半双工)
- 云端延迟: <200ms (ESP32 WiFi + FastAPI)
- 追踪精度: ±3° (理想光线条件)

---

## 贡献指南

欢迎提交 Bug 报告、功能请求和 Pull Request。请遵循 [Conventional Commits](https://www.conventionalcommits.org/) 规范。

---

## 许可证

MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 联系与支持

- **GitHub Issues**: [项目议题](https://github.com/Shizue123/Renesas-RA6M5-Multimodal-Drone-Tracking-Gimbal-System/issues)
- **作者**: Shizue123
