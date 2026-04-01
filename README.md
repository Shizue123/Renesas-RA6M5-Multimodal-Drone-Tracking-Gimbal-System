# 无人机多模态追踪云台系统

基于 **雷达 + 视觉** 双传感器融合的无人机实时追踪云台，覆盖从嵌入式控制到云端看板的完整链路。

---

## 系统架构

```
  OpenMV H7+ (96×96 FOMO 目标检测)
         │ UART9, 115200, 8N1
         ▼
  ┌─────────────────────────┐          LD2450 毫米波雷达 (6m 量程)
  │    Renesas RA6M5        │◄─────────── UART2, 256000, 8N1
  │    (追踪控制核心)        │
  └────┬───────────┬────────┘
       │           │
  UART5, 1Mbps     │ UART3, 115200
  半双工(URT-1)     ▼
       │       ESP32 网关
       ▼       (WiFi → HTTP POST)
  ST-3215 云台          │
  Pan(ID=1)             ▼
  Tilt(ID=2)       FastAPI 云端看板
                   (WebSocket 实时推送)
```

---

## 目录结构

| 目录 | 内容 | 技术栈 |
|------|------|--------|
| `Drone_Tracking_RA6M5/` | RA6M5 主控固件 | C / Renesas FSP / e² studio |
| `Drone_Tracking_RA6M5/openmv_scripts/` | OpenMV 机载视觉脚本 | MicroPython / TFLite FOMO |
| `ESP32-Competition_Deliverables/` | ESP32 WiFi 遥测网关 | Arduino / FreeRTOS |
| `Drone_Tracking_WebCloud/` | 云端监控看板 | Python / FastAPI / SQLite |
| `Key-document/` | 核心设计文档（已验证与代码同步） | Markdown |
| `References/` | 硬件参考资料、数据集 | — |

---

## 快速启动

### 1. RA6M5 主控固件

1. 用 **e² studio** 导入 `Drone_Tracking_RA6M5/` 工程
2. 确认 FSP 配置无误（`configuration.xml`，4 路 UART 已配好回调）
3. 编译 → **J-Link** 烧录到 RA6M5 开发板
4. 上电后舵机自动归中（position=2047），P400 LED 闪烁指示状态

### 2. OpenMV 视觉脚本

1. 用 **OpenMV IDE** 打开 `Drone_Tracking_RA6M5/openmv_scripts/main_tracking.py`
2. 确保 OpenMV 内置存储根目录包含：
   - `trained.tflite` — FOMO 模型文件
   - `labels.txt` — 标签文件（含 "drone"）
3. 连接 OpenMV H7+ → 点击运行

### 3. ESP32 遥测网关

1. **Arduino IDE** 打开 `ESP32-Competition_Deliverables/esp32_async_gateway/esp32_async_gateway.ino`
2. 修改以下常量：
   ```cpp
   #define WIFI_SSID     "你的WiFi名称"
   #define WIFI_PASSWORD "你的WiFi密码"
   #define CLOUD_URL     "http://你的服务器IP:8000/api/device/upload"
   ```
3. 选择 ESP32 开发板 → 编译上传

### 4. 云端看板

```bash
cd Drone_Tracking_WebCloud
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000
```

- 管理后台：`http://localhost:8000/admin.html`
- 实时看板：`http://localhost:8000/dashboard.html`
- 默认管理员：`admin` / `admin123`
- 建议通过环境变量覆盖密钥，见 `.env.example`

---

## 硬件接线

| RA6M5 引脚 | 方向 | 外设引脚 | 用途 | UART | 波特率 |
|-----------|------|---------|------|------|--------|
| P601 (RXD9) | ← | OpenMV P4 (TX) | 视觉数据接收 | g_uart9 / SCI9 | 115200 |
| P602 (TXD9) | → | OpenMV P5 (RX) | （预留） | g_uart9 / SCI9 | 115200 |
| P301 (RXD2) | ← | LD2450 TX | 雷达数据接收 | g_uart2 / SCI2 | 256000 |
| P302 (TXD2) | → | LD2450 RX | （预留） | g_uart2 / SCI2 | 256000 |
| P501 (TXD5) | → | URT-1 TX | 舵机指令发送 | g_uart5 / SCI5 | 1000000 |
| P502 (RXD5) | ← | URT-1 RX | 舵机反馈接收 | g_uart5 / SCI5 | 1000000 |
| P707 (TXD3) | → | ESP32 GPIO16 (RXD2) | 遥测数据发送 | g_uart3 / SCI3 | 115200 |
| P706 (RXD3) | ← | ESP32 GPIO17 (TXD2) | （预留） | g_uart3 / SCI3 | 115200 |
| P402 | — | URT-1 DIR | 半双工方向控制（硬件冗余） | — | — |
| P400 | — | LED_D12 | 系统状态指示（低电平点亮） | — | — |

**舵机供电**：URT-1 蓝色端子 V1+G 接 12V 稳压模块；三色总线接舵机。

详见 → [Key-document/硬件接线连接.md](Key-document/硬件接线连接.md)

---

## 核心设计文档索引

| 文档 | 内容 | 状态 |
|------|------|------|
| [Key-document/core-code.md](Key-document/core-code.md) | 算法核心：FOMO后处理、IIR/Kalman滤波、增量PID、四态状态机 | ✅ 与代码同步 |
| [Key-document/PID调制.md](Key-document/PID调制.md) | PID 理论与本项目调参实战指南 | ✅ 2026/3/30 |
| [Key-document/系统理性运行期望状态.md](Key-document/系统理性运行期望状态.md) | 系统五层运行目标定义 | ✅ 与代码同步 |
| [Key-document/硬件接线连接.md](Key-document/硬件接线连接.md) | 四路 UART 物理接线 | ✅ 与代码同步 |
| [Key-document/接口协议文档.md](Key-document/接口协议文档.md) | 四路 UART 帧格式逐字节定义 + 状态机转移表 | ✅ 新增 |
| [Key-document/参数速查表.md](Key-document/参数速查表.md) | 所有关键常量集中索引（含源码行号） | ✅ 新增 |
| [Key-document/已知问题与陷阱.md](Key-document/已知问题与陷阱.md) | 经审计确认的设计缺陷和调试经验 | ✅ 新增 |
| [Key-document/串口联调测试清单.md](Key-document/串口联调测试清单.md) | Hex 测试帧、联调步骤、故障排查 | ✅ v3.0 |
| [Key-document/目录结构说明.md](Key-document/目录结构说明.md) | 每个文件夹存放了什么、开发者去哪找代码 | ✅ 新增 |

---

## LED 状态指示

| LED 表现 | 含义 |
|---------|------|
| 常亮 | 视觉+舵机均正常 |
| 200ms 闪烁 | 舵机离线 或 仅视觉在线（搜索中） |
| 1s 周期（500ms 亮） | 仅雷达在线，无视觉 |
| 60ms 短脉冲 | 双传感器均丢失，自动搜索中 |

---

