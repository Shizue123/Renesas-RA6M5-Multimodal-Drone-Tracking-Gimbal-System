# Multimodal Drone Tracking Gimbal System

**[简体中文](./README_zh-CN.md) | English**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Languages: C | Python | MicroPython | Arduino](https://img.shields.io/badge/Languages-C%20%7C%20Python%20%7C%20MicroPython%20%7C%20Arduino-blue)](https://github.com/Shizue123/Renesas-RA6M5-Multimodal-Drone-Tracking-Gimbal-System)
[![Built with: Renesas RA6M5 | FastAPI | TensorFlow Lite](https://img.shields.io/badge/Built%20with-Renesas%20RA6M5%20%7C%20FastAPI%20%7C%20TFLite-brightgreen)](https://github.com/Shizue123/Renesas-RA6M5-Multimodal-Drone-Tracking-Gimbal-System)

---

## Overview

A **real-time autonomous drone tracking gimbal system** built on **dual-sensor fusion** (mmWave radar + machine vision), delivering end-to-end embedded control and cloud dashboard integration. The system achieves ultra-low-latency target tracking through Kalman filtering, incremental PID control, and a four-state finite machine, with full hardware traceability across four independent UART channels.

---

## Core Features

| Feature | Technical Details | Status |
|---------|------------------|--------|
| **Dual-Sensor Fusion** | LD2450 mmWave radar (6m range) + OpenMV H7+ FOMO (96×96 object detection) | ✅ Production Ready |
| **Real-Time Processing** | FOMO post-processing + IIR/Kalman filtering + Incremental PID control | ✅ Field Tested |
| **Hardware Control** | Renesas RA6M5 (4 independent UART) + ST-3215 servo gimbal (half-duplex serial) | ✅ Full Integration |
| **Wireless Gateway** | ESP32 WiFi bridge (FreeRTOS, HTTP POST to FastAPI backend) | ✅ Deployed |
| **Cloud Dashboard** | FastAPI real-time WebSocket + SQLite telemetry logging | ✅ Live Monitoring |
| **System Diagnostics** | 4-state LED indicator + per-module health telemetry | ✅ Built-in |

---

## Technology Stack

### Embedded Firmware
- **Main Controller**: Renesas RA6M5 (ARM Cortex-M4 @ 120MHz)
- **RTOS**: FreeRTOS (preemptive multi-tasking)
- **Dev Framework**: Renesas FSP
- **IDE**: e² studio
- **Language**: C (ISO C99)

### Machine Vision
- **Platform**: OpenMV H7+ (ARM Cortex-M7 @ 216MHz)
- **Model**: TensorFlow Lite FOMO
- **Language**: MicroPython
- **Training**: Custom dataset (drone detection)

### Wireless Gateway
- **MCU**: ESP32 (Dual-core Xtensa @ 240MHz)
- **Firmware**: Arduino Framework + FreeRTOS
- **Protocol**: WiFi + HTTP POST

### Cloud Backend
- **Framework**: FastAPI (async Python)
- **Database**: SQLite (telemetry)
- **Real-Time**: WebSocket push
- **Frontend**: HTML5 + JavaScript + Chart.js

---

## Directory Structure

| Path | Purpose | Tech Stack |
|------|---------|-----------|
| `Drone_Tracking_RA6M5/` | RA6M5 main firmware project | C / Renesas FSP / e² studio |
| `Drone_Tracking_RA6M5/openmv_scripts/` | On-board vision pipeline | MicroPython / TFLite FOMO |
| `Drone_Tracking_RA6M5/src/control/` | PID controller + tracking FSM | C headers + implementations |
| `Drone_Tracking_RA6M5/src/drivers/` | LD2450 radar + ST-3215 servo drivers | C UART abstraction layer |
| `ESP32-Competition_Deliverables/` | WiFi gateway firmware | Arduino / FreeRTOS |
| `Drone_Tracking_WebCloud/` | Cloud dashboard backend | Python / FastAPI / SQLite |
| `Key-document/` | Design specs, protocol docs, tuning guides | Markdown |
| `openmv-code/` | SD card deployment files (model + labels) | MicroPython / TFLite |

---

## Getting Started

### Prerequisites
- RA6M5 Dev Kit (120MHz Cortex-M4)
- OpenMV H7+ with OV7725 camera
- LD2450 mmWave Radar (UART interface)
- ST-3215 Servo (half-duplex serial)
- ESP32 Board (standard ESP32-WROOM-32)
- Tools: e² studio, OpenMV IDE, Arduino IDE, Python 3.9+

### 1. RA6M5 Main Firmware
- Open e² studio → Import `Drone_Tracking_RA6M5/`
- Verify FSP configuration (`configuration.xml`)
- Build → Flash via J-Link debugger
- P400 LED should blink on power-up

### 2. OpenMV Vision Pipeline
- Prepare SD card: `trained.tflite`, `labels.txt`, `main.py`
- Connect OpenMV H7+ via USB
- Upload files via OpenMV IDE File Manager
- Press Play in OpenMV IDE → outputs detection coordinates via UART

### 3. ESP32 WiFi Gateway
- Edit WiFi & CLOUD_URL in `esp32_async_gateway.ino`
- Build & upload via Arduino IDE
- Verify via serial monitor (115200 baud)

### 4. Cloud Dashboard Backend
```bash
cd Drone_Tracking_WebCloud
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000
```
- Admin: http://localhost:8000/admin.html (admin / admin123)
- Dashboard: http://localhost:8000/dashboard.html

---

## Hardware Wiring Reference

| RA6M5 Pin | Dir | Peripheral | Purpose | UART | Baudrate |
|-----------|-----|-----------|---------|------|----------|
| P601 (RXD9) | ← | OpenMV H7+ P4 | Vision RX | SCI9 | 115200 |
| P301 (RXD2) | ← | LD2450 TX | Radar RX | SCI2 | 256000 |
| P501 (TXD5) | → | URT-1 TX | Servo CMD TX | SCI5 | 1000000 |
| P502 (RXD5) | ← | URT-1 RX | Servo FB RX | SCI5 | 1000000 |
| P707 (TXD3) | → | ESP32 GPIO16 | Telemetry TX | SCI3 | 115200 |
| P400 | — | LED_D12 | Status (active-low) | GPIO | N/A |

Servo power: URT-1 blue terminal (V1+G) → 12V stabilized regulator → ST-3215 tri-color bus.

---

## Core Algorithm Documentation

| Document | Coverage | Status |
|----------|----------|--------|
| [core-code.md](Key-document/core-code.md) | FOMO post-processing, Kalman filter, PID, FSM | ✅ Code-synced |
| [PID调制.md](Key-document/PID调制.md) | PID tuning theory + real-world parameters | ✅ Latest |
| [接口协议文档.md](Key-document/接口协议文档.md) | UART frame format, state machine | ✅ Latest |
| [硬件接线连接.md](Key-document/硬件接线连接.md) | Pin-by-pin wiring for all UART channels | ✅ Code-synced |

---

## LED Status Codes

| Pattern | Meaning | Action |
|---------|---------|--------|
| Solid On | Vision + servos normal | ✅ Operating |
| 200ms pulse | Servo offline or vision searching | Check power/UART cable |
| 1s period (500ms on) | Radar online, no vision | Check OpenMV UART |
| 60ms burst | Both sensors lost (auto-recovery) | Wait 5s; check all UART cables |

---

## Performance Benchmarks

- Vision: 30 FPS @ 96×96 FOMO (OpenMV H7+)
- Radar: ~10 Hz update rate (LD2450)
- Control Loop: <50ms latency (RA6M5)
- Servo Response: ~20ms (at 1Mbps half-duplex)
- Cloud Latency: <200ms (ESP32 WiFi + FastAPI)
- Tracking Accuracy: ±3° (under optimal lighting)

---

## Contributing

We welcome bug reports, feature requests, and pull requests. Please follow [Conventional Commits](https://www.conventionalcommits.org/).

---

## License

MIT License. See [LICENSE](LICENSE) for details.

---

## Contact & Support

- **GitHub Issues**: [Project Issues](https://github.com/Shizue123/Renesas-RA6M5-Multimodal-Drone-Tracking-Gimbal-System/issues)
- **Author**: Shizue123
