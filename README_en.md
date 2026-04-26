# Multimodal Drone Tracking Gimbal System

[**English**](./README_en.md) | [**简体中文**](./README.md)

[![License](https://img.shields.io/badge/License-MIT-green.svg)](./LICENSE)
[![MCU](https://img.shields.io/badge/MCU-Renesas%20RA6M5-blue.svg)](./Drone_Tracking_RA6M5)
[![Vision](https://img.shields.io/badge/Vision-OpenMV%20%2B%20TFLite-orange.svg)](./openmv-code)
[![Cloud](https://img.shields.io/badge/Cloud-FastAPI%20%2B%20SQLite-purple.svg)](./Drone_Tracking_WebCloud)

## Overview
This project is a multimodal gimbal system for drone tracking. It fuses mmWave radar and edge vision to build a full closed loop from sensing and control to cloud observability. The control core runs on Renesas RA6M5, combining Kalman filtering, incremental PID, and a state machine for low-latency and robust target tracking.

## Core Features
- Dual-sensor fusion: LD2450 radar + OpenMV FOMO vision detection for stable tracking.
- Real-time control pipeline: multi-UART concurrency on RA6M5 with half-duplex servo feedback loop.
- Cloud-edge collaboration: ESP32 sends telemetry to FastAPI, WebSocket powers live dashboard updates.
- Layered engineering design: clear boundaries across drivers, control, gateway, and cloud services.

## Core Architecture
```text
OpenMV H7+ (FOMO)
      │ UART9
      ▼
┌───────────────────┐      LD2450 mmWave
│   Renesas RA6M5   │◄──────── UART2
│ Tracking Core(FSP)│
└──────┬───────┬────┘
       │       │
  UART5│       │UART3
       ▼       ▼
 ST-3215 Servo  ESP32 Gateway
    Gimbal         │ WiFi/HTTP
                   ▼
             FastAPI + SQLite
             Web Dashboard
```

## Tech Stack
- Embedded: C, Renesas FSP, FreeRTOS, e² studio
- Vision: OpenMV, MicroPython, TensorFlow Lite FOMO
- Gateway: ESP32, Arduino, HTTP
- Cloud: Python, FastAPI, SQLite, WebSocket

## Getting Started
1. RA6M5 firmware
   - Import `Drone_Tracking_RA6M5/` in e² studio.
   - Verify `configuration.xml` and generate FSP code.
   - Build and flash via J-Link.
2. OpenMV vision
   - Deploy `trained.tflite`, `labels.txt`, and `main.py` to OpenMV storage.
   - Run in OpenMV IDE and verify continuous UART target output.
3. ESP32 gateway
   - Configure WiFi and cloud URL in `esp32_async_gateway.ino`.
   - Build/upload and verify network upload in serial monitor.
4. Cloud service
   - Install dependencies in `Drone_Tracking_WebCloud/`.
   - Start FastAPI and open admin/live dashboard pages.

## Repository Map
- `Drone_Tracking_RA6M5/`: tracking firmware core
- `ESP32-Competition_Deliverables/`: telemetry gateway
- `Drone_Tracking_WebCloud/`: cloud backend and web pages
- `Key-document/`: protocol/design documents
- `openmv-code/`: model and vision scripts
