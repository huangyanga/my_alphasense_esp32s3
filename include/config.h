#pragma once
// ============================================================
// config.h  –  Alphasense B43F via ADS1115 (I2C) configuration
// ============================================================
// Edit this file to match your actual hardware wiring.
// All values marked "!! SOSTITUIRE !!" MUST be verified
// against your custom board schematic before flashing.
//
// I2C pin defaults match the FirmwareSensy sensy_2024 V3/V4
// ESP32-S3 build_flags (-DSDA_PIN=8 -DSCL_PIN=9).
// Override via build_flags in platformio.ini if using a
// different board variant (e.g. V1: SDA=1, SCL=2).
// ============================================================

// ------------------------------------------------------------
// I2C bus pins
// From FirmwareSensy platformio.ini:
//   sensy_2024_V1_green : SDA=1,  SCL=2
//   sensy_2024_V2_ENEA  : SDA=8,  SCL=9
//   sensy_2024_V3_red   : SDA=8,  SCL=9  ← default
//   sensy_2024_V4_green : SDA=8,  SCL=9
// !! SOSTITUIRE se la tua board usa pin diversi !!
// ------------------------------------------------------------
#ifndef SDA_PIN
#define SDA_PIN  41
#endif
#ifndef SCL_PIN
#define SCL_PIN  42
#endif

// ------------------------------------------------------------
// ADS1115 I2C address
// ADDR pin → GND  : 0x48  (default)
// ADDR pin → VDD  : 0x49
// ADDR pin → SDA  : 0x4A
// ADDR pin → SCL  : 0x4B
// ------------------------------------------------------------
#define ADS1115_I2C_ADDRESS   0x48

// ------------------------------------------------------------
// ADS1115 channel mapping (validate against custom board PCB)
// WE (Working Electrode output)    → AIN0
// AE (Auxiliary Electrode output)  → AIN1
// !! SOSTITUIRE se la board mappa diversamente !!
// ------------------------------------------------------------
#define ADS_CH_WE   0   // AIN0 = WE
#define ADS_CH_AE   1   // AIN1 = AE

// ------------------------------------------------------------
// ADS1115 PGA gain
// GAIN_TWOTHIRDS → ±6.144 V  (0.1875 mV/bit)
// GAIN_ONE       → ±4.096 V  (0.125  mV/bit)
// GAIN_TWO       → ±2.048 V  (0.0625 mV/bit)  ← recommended
// GAIN_FOUR      → ±1.024 V  (0.03125 mV/bit)
// GAIN_EIGHT     → ±0.512 V
// GAIN_SIXTEEN   → ±0.256 V
//
// Alphasense ISB Rev5 outputs are nominally 0–~2 V @ 3.3 V.
// TODO: verify actual output range with oscilloscope before
//       changing to a tighter gain setting.
// ------------------------------------------------------------
#define ADS1115_GAIN   GAIN_TWO   // ±2.048 V

// ------------------------------------------------------------
// ADS1115 data rate (samples per second)
// Options: RATE_ADS1115_8SPS  16  32  64  128  250  475  860
// ------------------------------------------------------------
#define ADS1115_SPS    RATE_ADS1115_128SPS

// ------------------------------------------------------------
// Acquisition mode
// 0 = single-ended A0 and A1, then compute WE-AE in software
// 1 = hardware differential AIN0-AIN1 (single read)
// Start with 0 to diagnose each channel individually.
// ------------------------------------------------------------
#define ADS1115_DIFFERENTIAL_MODE   0

// ------------------------------------------------------------
// Calibration offset and gain (per channel, in Volts)
// voltage_cal = (voltage_raw_V - OFFSET_V) * GAIN_CAL
// Measure with a precision reference and update these values.
// !! SOSTITUIRE CON VALORI CALIBRATI !!
// ------------------------------------------------------------
#define WE_OFFSET_V    0.0f
#define WE_GAIN_CAL    1.0f
#define AE_OFFSET_V    0.0f
#define AE_GAIN_CAL    1.0f
// #define WE_OFFSET_V    2.00774f
// #define AE_OFFSET_V    2.00192f
// ------------------------------------------------------------
// EMA (Exponential Moving Average) filter
// alpha = 1.0 → no filtering (raw values)
// alpha = 0.1 → strong smoothing (10-sample effective window)
// Can be overridden at compile time via build_flags.
// ------------------------------------------------------------
#ifndef EMA_ALPHA
#define EMA_ALPHA      0.1f
#endif

// ------------------------------------------------------------
// Sampling interval (ms between acquisitions)
// 降低到50ms以实现快速响应
// Can be overridden at compile time via build_flags.
// ------------------------------------------------------------
#ifndef SAMPLE_INTERVAL_MS
#define SAMPLE_INTERVAL_MS   1000U
#endif

// ------------------------------------------------------------
// I2C error recovery
// ------------------------------------------------------------
#define I2C_RETRY_DELAY_MS   500U
#define I2C_MAX_RETRIES      5U

// ------------------------------------------------------------
// Serial baud rate
// ------------------------------------------------------------
#define SERIAL_BAUD   115200

// ============================================================
// FS4001-500-CV-A 流量传感器配置
// ============================================================
// 使用 UART 通信 (RS232 接口，需要电平转换)
// ------------------------------------------------------------

// ------------------------------------------------------------
// 流量传感器 UART 引脚
// ESP32-S3 GPIO17 → FS4001/FS4008 TX (通过 RS232-TTL 转换器)
// ESP32-S3 GPIO18 → FS4001/FS4008 RX (通过 RS232-TTL 转换器)
// ------------------------------------------------------------
#define FLOW_RX_PIN  17  // ESP32 接收流量数据
#define FLOW_TX_PIN  18  // ESP32 发送命令

// ------------------------------------------------------------
// 流量传感器类型选择脚（IO39）
// 低电平 → FS4008（9位串口 + FS4000协议，38400/SERIAL_8E1）
// 高电平/悬空 → FS4001（原方式，38400/SERIAL_8N1）
// ------------------------------------------------------------
#define FLOW_SELECT_PIN  39

// ------------------------------------------------------------
// FS4001 通信参数
// 波特率: 38400, 数据位: 8, 停止位: 1, 校验位: 无
// ------------------------------------------------------------
#define FS4001_BAUD_RATE          38400

// ------------------------------------------------------------
// 流量传感器量程和校准参数
// FS4001-500-CV-A: 500 sccm 量程
// ------------------------------------------------------------
#define FLOW_SENSOR_RANGE_SCCM   500.0f  // 最大流量 500 sccm

// 流量校准参数 (sccm)
// flow_cal = (flow_raw - offset) * gain
// 需要用标准流量计校准后更新这些值
#define FLOW_OFFSET_SCCM         0.0f
#define FLOW_GAIN_CAL            1.0f

// ------------------------------------------------------------
// 流量传感器 UART 错误恢复
// ------------------------------------------------------------
#define FLOW_UART_MAX_RETRIES      5U
#define FLOW_UART_RETRY_DELAY_MS   500U

// ------------------------------------------------------------
// 流量传感器 EMA 滤波器
// 独立于气体传感器的滤波器参数
// ------------------------------------------------------------
#ifndef FLOW_EMA_ALPHA
#define FLOW_EMA_ALPHA      0.1f  // 默认 0.1，可根据需要调整
#endif

// ------------------------------------------------------------
// 流量传感器调试选项
// ------------------------------------------------------------
#define FS4001_DEBUG_OUTPUT     0  // 1=启用调试输出, 0=禁用

// ============================================================
// 隔膜泵 PWM 控制配置
// ============================================================
// 使用 ESP32-S3 LEDC 输出 PWM 信号
// ------------------------------------------------------------

// ------------------------------------------------------------
// PWM 引脚配置
// ESP32-S3 GPIO16 → 隔膜泵控制端
// ------------------------------------------------------------
#define PUMP_PWM_PIN  16  // PWM 输出引脚

// ------------------------------------------------------------
// PWM 参数
// 频率: 20kHz，适用于大多数隔膜泵
// 占空比: 0-100% (默认 50%)
// ------------------------------------------------------------
#define PUMP_PWM_FREQ_HZ      20000   // PWM 频率 20kHz
#define PUMP_PWM_RESOLUTION   10      // PWM 分辨率 10-bit (0-1023)
#define PUMP_DEFAULT_DUTY     50.0f   // 默认占空比 50%
#define PUMP_MAX_DUTY         100.0f  // 最大占空比 100%
#define PUMP_MIN_DUTY         0.0f    // 最小占空比 0%

// ------------------------------------------------------------
// PWM 调试选项
// ------------------------------------------------------------
#define PUMP_DEBUG_OUTPUT     1  // 1=启用调试输出, 0=禁用

// ============================================================
// 电磁阀控制配置
// ============================================================
// 使用 GPIO 输出高低电平控制电磁阀开关
// ------------------------------------------------------------

// ------------------------------------------------------------
// 电磁阀引脚配置
// ESP32-S3 GPIO7 → 电磁阀控制端
// ------------------------------------------------------------
#define VALVE_PIN  7  // 电磁阀控制引脚

// ------------------------------------------------------------
// 电磁阀控制逻辑
// ------------------------------------------------------------
#define VALVE_ACTIVE_LEVEL    HIGH    // 开启电平（高电平）
#define VALVE_INACTIVE_LEVEL  LOW     // 关闭电平（低电平）

// ------------------------------------------------------------
// 电磁阀初始状态
// ------------------------------------------------------------
#define VALVE_DEFAULT_STATE   VALVE_INACTIVE_LEVEL  // 默认关闭

// ------------------------------------------------------------
// 电磁阀调试选项
// ------------------------------------------------------------
#define VALVE_DEBUG_OUTPUT    1  // 1=启用调试输出, 0=禁用

// ============================================================
// 流量控制系统配置
// ============================================================
// 通过PID控制泵占空比来调节流量
// ------------------------------------------------------------

// ------------------------------------------------------------
// 流量控制范围
// ------------------------------------------------------------
#define FLOW_CONTROL_MIN_FLOW      0.0f    // 最小流量 0 sccm
#define FLOW_CONTROL_MAX_FLOW      500.0f  // 最大流量 500 sccm（传感器量程上限）
#define FLOW_CONTROL_ALARM_FLOW    500.0f  // 报警流量 500 sccm（开环监控模式下不自动停机）

// ------------------------------------------------------------
// 泵占空比限制
// PID输出是偏移量（-50% ~ +50%），实际占空比 = 最小占空比 + PID输出
// 例如：PID=0 → 实际=50%，PID=+20 → 实际=70%，PID=-10 → 实际=40%（限制到50%）
// ------------------------------------------------------------
#define FLOW_CONTROL_PUMP_MIN_DUTY  50.0f   // 最小工作占空比 50%（0 sccm 对应值）
#define FLOW_CONTROL_PUMP_MAX_DUTY  100.0f  // 最大占空比 100%

// ------------------------------------------------------------
// PID参数（根据你的系统调整）
// PID输出是偏移量：实际占空比 = 50% + PID输出
// 系统增益：50% PWM → 80 sccm，增益 ≈ 1.6 sccm/%
// PID范围：-50%（降低到50%）到 +50%（增加到100%）
// 控制周期：3秒
// ------------------------------------------------------------
#define FLOW_CONTROL_PID_KP        0.01f     // 比例系数：误差10sccm → 调节1.0%（更快响应）
#define FLOW_CONTROL_PID_KI        0.1f     // 积分系数：消除稳态误差
#define FLOW_CONTROL_PID_KD        0.02f    // 微分系数：抑制震荡

// ------------------------------------------------------------
// 控制周期
// ------------------------------------------------------------
#define FLOW_CONTROL_INTERVAL_MS   1000    // 控制周期3000ms（3秒调节一次）

// ------------------------------------------------------------
// 流量控制调试选项
// ------------------------------------------------------------
#define FLOW_CONTROL_DEBUG_OUTPUT  1       // 1=启用调试输出以观察PID计算, 0=禁用

