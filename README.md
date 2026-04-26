# 无人机多模态追踪云台系统

[**English**](./README_en.md) | [**简体中文**](./README.md)

[![License](https://img.shields.io/badge/License-MIT-green.svg)](./LICENSE)
[![MCU](https://img.shields.io/badge/MCU-Renesas%20RA6M5-blue.svg)](./Drone_Tracking_RA6M5)
[![Vision](https://img.shields.io/badge/Vision-OpenMV%20%2B%20TFLite-orange.svg)](./openmv-code)
[![Cloud](https://img.shields.io/badge/Cloud-FastAPI%20%2B%20SQLite-purple.svg)](./Drone_Tracking_WebCloud)

## 项目概述
本项目是一个面向无人机目标追踪的多模态云台系统，采用毫米波雷达与边缘视觉协同感知，构建从传感输入、控制决策到云端观测的完整闭环。系统主控基于 Renesas RA6M5，通过 Kalman 滤波、增量式 PID 与状态机实现低延迟、高鲁棒的目标跟踪。

## 核心特性
- 双传感器融合：LD2450 雷达 + OpenMV FOMO 视觉检测，兼顾速度与稳定性。
- 实时控制链路：RA6M5 多路 UART 并行通信，舵机半双工闭环控制。
- 云边协同：ESP32 将遥测数据异步上报 FastAPI，WebSocket 驱动看板实时刷新。
- 工程化分层：驱动层、控制层、网关层、云端层职责清晰，便于调试与扩展。

## 核心架构
```text
OpenMV H7+ (FOMO)
      │ UART9
      ▼
┌───────────────────┐      LD2450 mmWave
│   Renesas RA6M5   │◄──────── UART2
│  追踪控制核心(FSP) │
└──────┬───────┬────┘
       │       │
  UART5│       │UART3
       ▼       ▼
 ST-3215舵机   ESP32网关
   (云台)        │ WiFi/HTTP
                ▼
           FastAPI + SQLite
           Web Dashboard
```

## 技术栈
- 嵌入式：C、Renesas FSP、FreeRTOS、e² studio
- 视觉侧：OpenMV、MicroPython、TensorFlow Lite FOMO
- 网关侧：ESP32、Arduino、HTTP
- 云端侧：Python、FastAPI、SQLite、WebSocket

## 快速启动
1. RA6M5 固件
   - 使用 e² studio 导入 `Drone_Tracking_RA6M5/`。
   - 校验 `configuration.xml` 并生成 FSP 代码。
   - 编译并通过 J-Link 烧录。
2. OpenMV 视觉
   - 将 `trained.tflite`、`labels.txt`、`main.py` 拷入 OpenMV 存储。
   - 在 OpenMV IDE 运行脚本，确认 UART 持续输出目标数据。
3. ESP32 网关
   - 在 `esp32_async_gateway.ino` 配置 WiFi 与云端地址。
   - 编译上传并在串口监视器确认联网与上报状态。
4. 云端服务
   - 进入 `Drone_Tracking_WebCloud/` 安装依赖。
   - 启动 FastAPI 服务并访问管理后台与实时看板。

## 相关目录
- `Drone_Tracking_RA6M5/`：主控固件
- `ESP32-Competition_Deliverables/`：网关程序
- `Drone_Tracking_WebCloud/`：云端后端与页面
- `Key-document/`：协议与工程文档
- `openmv-code/`：视觉模型与脚本
