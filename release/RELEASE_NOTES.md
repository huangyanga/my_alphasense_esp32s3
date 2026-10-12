# 固件发布说明 - v1.0

**日期**: 2026-09-18
**固件文件**: `firmware_v1.0.bin`（ESP32-S3, 8MB Flash, QIO + OPI PSRAM）

## 功能清单

- **双模式串口协议**（HOST / DEBUG）：`A START` / `A STOP` / `Q LAST` / `Q ELAPSED` / `Q PRESSURE` / `Q GAS_TYPE` / `Q BASELINE`
- **测试时长自动停止**：`A START, t=<秒>` 到时自动执行 `A STOP` 并回复 `R STOP_OK`
- **浓度计算**：`conc(ppb) = 2 × ΔV(mV)`；`STOP_OK` 返回平均浓度与最后一次 wa
- **零点校准（A ZERO）**：零气下 20Hz 采样 WE/AE 原始电压各 n 点（10~300），平均值存为 `WE_OFFSET_V` / `AE_OFFSET_V`（NVS 持久化，断电不丢）；`A ZERO, CLR` 清除
- **流量开环控制**：下行 0~500 sccm 线性映射泵占空比 50~100%（`duty = 50 + flow/10`）；FS4001 仅作监控显示
- **2.4" ST7789 LCD 曲线界面**：
  - 按单次测试会话显示曲线（横轴按实际数据点数缩放，缓存 600 点）
  - 纵轴固定 -5~+5 ppm，刻度标注；标题栏显示最新值，底部信息区显示 wa/ppb/flow/pump
  - 测试结束底部大字号显示 `avg=` / `last=`（ppb）
  - 标题栏右侧显示版本号 `v1.0`
  - **双方案自动识别**：IO10 低电平（方案B，无 XL9555）→ 位带逐字节 CS 选通驱动；IO10 悬空（方案A）→ Adafruit 硬件 SPI 驱动

## 硬件

| 外设 | 引脚 |
|---|---|
| ADS1115 (I2C 0x48) | SDA=41, SCL=42 |
| XL9555 (I2C 0x20，仅方案A) | 同上总线 |
| FS4001 流量 (UART) | RX=17, TX=18, 38400 |
| 泵 PWM | GPIO16, 20kHz, 10bit |
| 电磁阀 | GPIO7 |
| LCD SPI (方案A/B 共用) | SCLK=12, MOSI=11, MISO=13, CS=21, DC=40 |
| LCD RST/BL（方案B 直连） | RST=GPIO9, BL=GPIO8 |

## 已知说明

- 方案B 位带驱动刷新较慢（整屏清 ~1.4s），每秒 1 次采样可跟上
- 通道增益 `WE_GAIN_CAL` / `AE_GAIN_CAL` 仍为 1.0（直通），如需灵敏度标定需另行配置
