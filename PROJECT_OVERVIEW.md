# 多传感器数据采集与呼气检测系统

> 基于 **ESP32-S3**（Arduino 框架 / PlatformIO）的多传感器固件工程，集成 **气体传感器**、**流量传感器**、**隔膜泵** 与 **电磁阀**，支持呼吸气体采样、流量闭环控制与上位机通讯。

---

## 1. 工程概述

本工程是一个面向呼气检测场景的多传感器测量与控制平台，主要完成以下任务：

- 采集 **Alphasense B43F** 电化学气体传感器的工作电极（WE）与辅助电极（AE）信号，计算补偿信号并滤波；
- 采集 **FS4001-500-CV-A** 流量传感器数据（sccm）；
- 通过 **PID 闭环** 调节隔膜泵 PWM 占空比，维持目标流量；
- 控制电磁阀通断，切换气路；
- 通过串口提供**调试模式**（CSV 数据流 + 交互命令）与**主机协议模式**（ASCII 报文）两种操作方式。

---

## 2. 硬件连接

| 子系统 | 器件 | 接口 | ESP32-S3 引脚 | 备注 |
|---|---|---|---|---|
| 气体传感 | Alphasense B43F | I2C（经 ADS1115 16-bit ADC） | SDA=41, SCL=42 | ADC 地址 0x48 |
| 流量传感 | FS4001-500-CV-A | RS232 / UART | RX=17, TX=18 | 需 RS232-TTL 电平转换，38400-8-N-1 |
| 隔膜泵 | 隔膜泵 | PWM（LEDC） | GPIO16 | 20kHz，10-bit 分辨率 |
| 电磁阀 | 电磁阀 | GPIO | GPIO7 | 高电平开启 |
| 调试串口 | USB-UART（CH340C） | UART0 / CDC | — | 115200-8-N-1 |

> 引脚与参数均可在 `platformio.ini`（编译期 `-D` 覆盖）与 `include/config.h` 中调整。

---

## 3. 目录结构

```
test_platformio_alphasense-main/
├── platformio.ini              # PlatformIO 构建配置
├── src/
│   └── main.cpp                # 主程序（setup / loop / 命令处理）
├── include/
│   ├── config.h                # 全局配置（引脚、采样、校准、PID 等）
│   ├── alphasense_b43f.h       # B43F 气体传感器驱动（经 ADS1115）
│   ├── flow_sensor_fs4001.h    # FS4001 流量传感器驱动
│   ├── pump_controller.h       # 隔膜泵 PWM 控制
│   ├── valve_controller.h      # 电磁阀控制
│   ├── flow_controller.h       # 流量 PID 闭环控制
│   ├── pid_controller.h        # PID 算法
│   └── protocol_handler.h      # 主机串口通讯协议
├── *.md                        # 调试/排障笔记与各类测试工具源码
└── *.cpp                       # 调试期独立测试工具（UART/流量等）
```

---

## 4. 功能模块

### 4.1 气体传感器（`alphasense_b43f.h`）

- 通过 ADS1115（`GAIN_TWO` ±2.048V，128SPS）分别读取 WE（AIN0）与 AE（AIN1）。
- 单端模式：`signal_v = we_v - ae_v`（软件差分）；支持硬件差分模式切换（`ADS1115_DIFFERENTIAL_MODE`）。
- 每通道支持 **offset/gain 校准**：`voltage_cal = (raw - OFFSET) * GAIN`。
- **EMA 指数滤波**（`EMA_ALPHA`）平滑电压与信号。
- I2C 通信失败自动重试（`I2C_MAX_RETRIES`），非阻塞。
- 浓度换算 `convertToConcentration()` 为 TODO 预留接口。

### 4.2 流量传感器（`flow_sensor_fs4001.h`）

- RS232 通讯（38400-8-N-1），支持操作模式（自动连续）与用户查询模式。
- 查询模式解析响应中的十进制流量值，换算为 sccm（÷1000）。
- 独立的 EMA 滤波（`FLOW_EMA_ALPHA`）与 UART 重试恢复。

### 4.3 隔膜泵控制（`pump_controller.h`）

- ESP32-S3 LEDC 输出 20kHz PWM，10-bit 分辨率，占空比 0–100%。
- 支持占空比设置、查询、启停与状态查看。

### 4.4 电磁阀控制（`valve_controller.h`）

- GPIO 高低电平控制阀门开闭（默认关闭），支持开关/切换/状态查询。

### 4.5 流量闭环控制（`flow_controller.h` + `pid_controller.h`）

- 通过 PID 调节泵占空比，将实际流量收敛至目标流量（0–450 sccm）。
- PID 输出为占空比偏移量，叠加在基准占空比上；含报警阈值（500 sccm）。

### 4.6 主机通讯协议（`protocol_handler.h`）

- ASCII 文本报文：`[A|Q|R] <CmdId>, <key=value>`，CRLF 结尾。
- `A`=Action 命令、`Q`=Query 查询、`R`=Response/主动上报。
- 支持气体类型（`NO` / `CO`）、目标流量、测试启停、运行时长查询等。
- 主机模式下按固定间隔（500ms）自动上报数据，1 秒 TICK 上报。

---

## 5. 数据输出格式

调试模式的采样周期（`SAMPLE_INTERVAL_MS`，默认 1000ms）输出一行 CSV：

```
timestamp_ms, we_raw, ae_raw, we_v, ae_v, signal_v, we_ema, ae_ema, signal_ema, flow_rate, voltage, mode, pump_duty, valve_state
```

| 字段 | 说明 |
|---|---|
| `timestamp_ms` | 系统运行毫秒时间戳 |
| `we_raw` / `ae_raw` | WE / AE 原始 ADC 计数 |
| `we_v` / `ae_v` | WE / AE 校准后电压（V） |
| `signal_v` | 补偿信号 = WE − AE（V） |
| `we_ema` / `ae_ema` / `signal_ema` | 对应 EMA 滤波值 |
| `flow_rate` | 流量（sccm） |
| `voltage` | 流量传感器电压（V） |
| `mode` | FS4001 工作模式（`00`=操作 / `11`=用户） |
| `pump_duty` | 泵当前占空比（%） |
| `valve_state` | 阀门状态（1=开 / 0=关） |

---

## 6. 两种工作模式

**调试模式（DEBUG）**
- 定时打印 CSV 数据流，便于 SerialPlot 等工具可视化。
- 支持交互命令：`help`、`mode host/debug`、`flow set/stop/status/alarm`、`pump set/get/status/on/off`、`valve on/off/toggle/status`。

**主机协议模式（HOST）**
- 默认模式，接收上位机 ASCII 指令并自动上报。
- 示例：`A START, t=0, gas=NO, flow=200` 启动测试；`Q ELAPSED` 查询时长；`A STOP` 停止。

> 两种模式通过 `mode debug` / `mode host` 命令互切。

---

## 7. 构建与烧录

```bash
# 方式一：PlatformIO CLI
pio run -e esp32s3
pio run -e esp32s3 -t upload

# 方式二：VSCode PlatformIO 插件
# 打开工程 → Build / Upload
```

**环境要求**
- PlatformIO（`espressif32` 平台，`arduino` 框架）
- 板型：`esp32-s3-devkitc-1`，8MB Flash + OPI PSRAM（`qio_opi`）
- 依赖库：`adafruit/Adafruit ADS1X15 @ ^2.5.0`

> 注意：编译时 `-DARDUINO_USB_MODE=1` 与 `-DARDUINO_USB_CDC_ON_BOOT=1` 默认被注释，`Serial` 走 CH340C 硬件串口（115200）。如需 USB CDC，请取消注释。

---

## 8. 关键配置项（`config.h`）

| 配置 | 默认值 | 说明 |
|---|---|---|
| `SDA_PIN` / `SCL_PIN` | 41 / 42 | ADS1115 I2C 引脚 |
| `ADS1115_I2C_ADDRESS` | 0x48 | ADC 地址 |
| `ADS1115_GAIN` | `GAIN_TWO` | ±2.048V |
| `ADS1115_SPS` | 128 SPS | 采样率 |
| `SAMPLE_INTERVAL_MS` | 1000 | 采样周期 |
| `EMA_ALPHA` | 0.1 | 气体信号滤波系数 |
| `FLOW_RX_PIN` / `FLOW_TX_PIN` | 17 / 18 | 流量传感器 UART |
| `FS4001_BAUD_RATE` | 38400 | 流量传感器波特率 |
| `PUMP_PWM_PIN` | 16 | 泵 PWM 引脚 |
| `PUMP_PWM_FREQ_HZ` | 20000 | PWM 频率 |
| `VALVE_PIN` | 7 | 电磁阀引脚 |
| `FLOW_CONTROL_*` | — | 流量 PID 闭环与报警参数 |

---

## 9. 待完成事项（TODO）

- [ ] `convertToConcentration()` 浓度换算（需 TIA 增益、传感器灵敏度 nA/ppb、零偏标定数据）。
- [ ] 气体传感器实际输出范围标定，确认是否可切换更高增益档位。
- [ ] 流量传感器校准参数（`FLOW_OFFSET_SCCM` / `FLOW_GAIN_CAL`）实际标定。

---

## 10. 调试备注

根目录下的 `*.cpp` 与 `*.md` 文件为调试期产物（UART 环回测试、流量传感器原始数据监控、编译排障笔记等），**不参与正式固件编译**（PlatformIO 仅编译 `src/main.cpp`）。需要时可用于排查硬件或通信问题。
