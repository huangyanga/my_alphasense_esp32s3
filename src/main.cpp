// ============================================================
// main.cpp  –  Multi-Sensor Data Acquisition System
// ESP32-S3 · Arduino framework · PlatformIO
// ============================================================
// What this firmware does
// -----------------------
//   • Initialises Wire (I2C) and the ADS1115 16-bit ADC for gas sensor
//   • Initialises UART for FS4001 flow sensor (RS232 interface)
//   • Every SAMPLE_INTERVAL_MS ms it:
//       – reads WE (AIN0) and AE (AIN1) from Alphasense B43F gas sensor
//       – reads flow rate and voltage from FS4001 flow sensor via RS232
//       – applies per-channel calibration offset/gain
//       – computes signal = WE - AE for gas sensor
//       – updates EMA (Exponential Moving Average) filter
//       – prints a CSV row over Serial-CDC with all sensor data
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "alphasense_b43f.h"
#include "flow_sensor_fs4001.h"
#include "pump_controller.h"
#include "flow_controller.h"
#include "valve_controller.h"
#include "protocol_handler.h"
#include "lcd_st7789.h"
#include "scd30.h"

// ------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------
void printHeader();
void printReading(const B43FReading& gasReading, const FS4001Reading& flowReading);
void printFlowSensorInfo();
void processSerialCommand();

// ------------------------------------------------------------
// setup()
// ------------------------------------------------------------
void setup() {
    // 配置串口
    Serial.begin(SERIAL_BAUD);
    Serial.setTimeout(100);
    delay(100);
    Serial.println();

    Serial.println(F("========================================"));
    Serial.println(F("  Multi-Sensor Data Acquisition System"));
    Serial.println(F("  ESP32-S3 + Alphasense B43F + FS4001"));
    Serial.println(F("========================================"));

    Serial.println(F("\n  Hardware Configuration:"));
    Serial.printf("    Gas Sensor (ADS1115): I2C SDA=%d, SCL=%d, Addr=0x%02X\n",
                  SDA_PIN, SCL_PIN, ADS1115_I2C_ADDRESS);
    Serial.printf("    Flow Sensor (FS4001):  UART RX=%d, TX=%d, Baud=%d\n",
                  FLOW_RX_PIN, FLOW_TX_PIN, FS4001_BAUD_RATE);
    Serial.printf("    Pump PWM:             GPIO%d, Freq=%dHz, Res=%d-bit\n",
                  PUMP_PWM_PIN, PUMP_PWM_FREQ_HZ, PUMP_PWM_RESOLUTION);
    Serial.printf("    Valve Control:        GPIO%d\n", VALVE_PIN);

    Serial.printf("\n  Sample interval  : %u ms\n", SAMPLE_INTERVAL_MS);
    Serial.println(F("========================================"));
    Serial.println(F("NOTE: voltages and flow rates are estimates."));
    Serial.println(F("      Calibrate sensors with reference equipment."));
    Serial.println(F("========================================\n"));

    // 初始化气体传感器
    if (!adsInit()) {
        Serial.println(F("[WARN] ADS1115 init failed. Will retry each sample."));
    }

    // 初始化 SCD30 CO2 传感器（与 ADS1115 共用 I2C 总线 SDA=41/SCL=42）
    if (!scd30Init()) {
        Serial.println(F("[WARN] SCD30 init failed."));
    } else {
        Serial.println(F("[SCD30] CO2 sensor ready"));
    }

    // 初始化流量传感器
    if (!flowSensorInit()) {
        Serial.println(F("[WARN] FS4001 flow sensor init failed. Will retry each sample."));
    }

    // 初始化隔膜泵控制
    if (!pumpInit()) {
        Serial.println(F("[WARN] Pump PWM init failed."));
    }

    // 初始化电磁阀控制
    if (!valveInit()) {
        Serial.println(F("[WARN] Valve controller init failed."));
    }

    // 初始化流量控制系统
    if (!flowControlInit()) {
        Serial.println(F("[WARN] Flow control system init failed."));
    }

    // 初始化协议处理器
    if (!protocolInit()) {
        Serial.println(F("[WARN] Protocol handler init failed."));
    }

    // 初始化 LCD 并显示曲线界面
    if (!lcdInit()) {
        Serial.println(F("[WARN] LCD init failed."));
    } else {
        lcdCurveInit("CO/NO TEST");
        Serial.println(F("[LCD] Curve display ready"));
    }

    printHeader();

    // 显示使用提示
    Serial.println(F("\n========== Quick Start =========="));
    Serial.printf("当前模式: %s\n", protocolGetMode() == PROTOCOL_MODE_HOST ? "HOST" : "DEBUG");
    Serial.println(F("\n使用方法:"));
    Serial.println(F("1. 确保串口监视器设置为 '无行尾' 或 'LF (\\n)'"));
    Serial.println(F("2. 输入命令后按回车键"));
    Serial.println(F("3. 所有命令必须小写，如 'help'、'mode debug'"));
    Serial.println(F("\n可用命令:"));
    Serial.println(F("  help           - 显示帮助"));
    Serial.println(F("  mode debug     - 切换到调试模式"));
    Serial.println(F("  mode host      - 切换到主机协议模式"));
    Serial.println(F("\n主机协议示例:"));
    Serial.println(F("  A START, t=0, gas=NO, flow=200"));
    Serial.println(F("  Q ELAPSED"));
    Serial.println(F("  A STOP"));
    Serial.println(F("====================================\n"));
}

// ------------------------------------------------------------
// loop()
// ------------------------------------------------------------
void loop() {
    static unsigned long lastSampleMs = 0;
    static unsigned long lastControlMs = 0;

    // 处理串口命令
    processSerialCommand();

    // 零点校准采样（内部20Hz节流，非阻塞）
    protocolZeroCalUpdate();

    unsigned long now = millis();

    // SCD30 CO2 周期读取（测量间隔2s）并显示到LCD标题栏
    static unsigned long lastCo2Ms = 0;
    if (now - lastCo2Ms >= 2000) {
        lastCo2Ms = now;
        float co2 = 0.0f, t = 0.0f, rh = 0.0f;
        if (scd30Read(&co2, &t, &rh)) {
            lcdShowCo2(co2);
            Serial.printf("[SCD30] CO2=%.1fppm T=%.1fC RH=%.1f%%\n", co2, t, rh);
        }
    }

    // 采样周期
    if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
        lastSampleMs = now;

        // 读取气体传感器数据
        B43FReading gasReading = adsRead();

        // 读取流量传感器数据
        FS4001Reading flowReading = flowSensorRead();

        // 主机模式：处理协议自动上报 + LCD曲线显示
        if (protocolGetMode() == PROTOCOL_MODE_HOST) {
            static bool _lcdPrevRunning = false;  // LCD侧测试状态跟踪
            bool runningNow = protocolTestRunning();

            // 新测试开始：清空曲线缓存
            if (runningNow && !_lcdPrevRunning) {
                lcdCurveReset();
            }

            protocolUpdate(gasReading, flowReading);

            // 测试结束（手动或自动超时）：绘制最终曲线并标注平均值/最后值
            if (_lcdPrevRunning && !protocolTestRunning()) {
                lcdCurveEnd(protocolTestAvgPpb(), protocolTestLastPpb());
            }
            _lcdPrevRunning = protocolTestRunning();

            // 测试运行中：将上报数据绘制为LCD曲线
            if (protocolTestRunning() && gasReading.valid) {
                float wa_mv    = gasReading.signal_ema * 1000.0f;
                float conc_ppb = 2.0f * wa_mv;            // 浓度(ppb) = 2 × ΔV(mV)（已含偏移校正）
                float conc_ppm = conc_ppb / 1000.0f;      // 曲线纵轴单位: ppm
                lcdCurveAdd(conc_ppm);
                lcdShowInfo(wa_mv, conc_ppb,
                            flowReading.valid ? flowReading.flow_rate : 0.0f,
                            pumpGetDuty());
            }
        } else {
            // 调试模式：打印CSV数据
            printReading(gasReading, flowReading);
        }

        // 更新流量控制系统
        if (flowReading.valid && (now - lastControlMs >= FLOW_CONTROL_INTERVAL_MS)) {
            lastControlMs = now;
            flowControlUpdate(flowReading);
        }
    }
}

// ------------------------------------------------------------
// printHeader()
// CSV column header printed once at startup.
// ------------------------------------------------------------
void printHeader() {
    Serial.println(F("timestamp_ms, "
                     // 气体传感器数据
                     "we_raw, ae_raw, we_v, ae_v, signal_v, we_ema, ae_ema, signal_ema, "
                     // 流量传感器数据
                     "flow_rate, voltage, mode, "
                     // 泵控制数据
                     "pump_duty, "
                     // 阀门控制数据（1=OPEN, 0=CLOSED）
                     "valve_state"));
}

// ------------------------------------------------------------
// printReading()
// One CSV row per sample burst with all sensor data.
// ------------------------------------------------------------
void printReading(const B43FReading& gasReading, const FS4001Reading& flowReading) {
    // 时间戳
    Serial.printf("%lu, ", millis());

    // 气体传感器数据
    if (!gasReading.valid) {
        Serial.print("ERROR: ADS1115 not responding, ");
    } else {
        Serial.printf("%6d, %6d, %8.5f, %8.5f, %9.5f, %8.5f, %8.5f, %9.5f, ",
                      (int)gasReading.we_raw, (int)gasReading.ae_raw,
                      gasReading.we_v,    gasReading.ae_v,    gasReading.signal_v,
                      gasReading.we_ema,  gasReading.ae_ema,  gasReading.signal_ema);
    }

    // 流量传感器数据
    if (!flowReading.valid) {
        Serial.print("ERROR: FS4001 not responding, ");
    } else {
        // 格式化工作模式：AUTO=00, QUERY=11
        const char* modeStr = "UNKNOWN";
        if (flowReading.mode == FS4001_MODE_OPERATION) modeStr = "00";
        else if (flowReading.mode == FS4001_MODE_USER) modeStr = "11";

        Serial.printf("%8.3f, %6.2f, %s, ",
                      flowReading.flow_rate,
                      flowReading.voltage,
                      modeStr);
    }

    // 泵控制数据
    Serial.printf("%5.1f, ", pumpGetDuty());

    // 阀门控制数据：OPEN=1, CLOSED=0
    Serial.printf("%d", valveIsOpen() ? 1 : 0);

    Serial.println(); // 换行
}

// ------------------------------------------------------------
// printFlowSensorInfo()
// 打印流量传感器详细信息和状态
// ------------------------------------------------------------
void printFlowSensorInfo() {
    FS4001Reading reading = flowSensorRead();

    Serial.println(F("\n=== Flow Sensor Info ==="));
    Serial.printf("Valid: %s\n", reading.valid ? "YES" : "NO");
    Serial.printf("Flow Rate: %.3f sccm\n", reading.flow_rate);
    Serial.printf("Voltage: %.2f V\n", reading.voltage);
    Serial.printf("Voltage Code: %u\n", reading.voltage_code);
    Serial.printf("Mode: %s\n", reading.mode == FS4001_MODE_OPERATION ? "AUTO (OPERATION)" : "QUERY (USER)");
    Serial.printf("Data Ready: %s\n", reading.data_ready ? "YES" : "NO");
    Serial.printf("Error: %s\n", reading.error ? "YES" : "NO");
    Serial.printf("Timestamp: %lu ms\n", reading.timestamp_ms);
    Serial.println(F("========================\n"));
}

// ------------------------------------------------------------
// processSerialCommand()
// 处理串口命令
// ------------------------------------------------------------
void processSerialCommand() {
    // 检查是否有数据可读
    if (Serial.available() == 0) return;

    // 主机模式：直接处理协议
    if (protocolGetMode() == PROTOCOL_MODE_HOST) {
#if 1  // 临时调试
        static bool firstTime = true;
        if (firstTime) {
            Serial.println("[DEBUG] Processing in HOST mode");
            firstTime = false;
        }
#endif
        protocolProcess();
        return;
    }

    // 调试模式：处理原有命令
    // 读取一行命令
    static char cmdBuffer[128] = {0};
    static size_t cmdPos = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();

        // 处理回车或换行
        if (c == '\r' || c == '\n') {
            if (cmdPos > 0) {
                cmdBuffer[cmdPos] = '\0';

                // 跳过空行
                if (strlen(cmdBuffer) > 0) {
                    Serial.printf("\n[CMD] Executing: %s\n", cmdBuffer);

                    // 模式切换命令
                    if (strcmp(cmdBuffer, "mode host") == 0) {
                        protocolSetMode(PROTOCOL_MODE_HOST);
                        Serial.println(F("\n[INFO] 已切换到主机协议模式"));
                        Serial.println(F("[INFO] 命令格式: [A|Q|R] <CmdId>, <key=value>"));
                        Serial.println(F("[INFO] 示例: A START, t=0, gas=NO, flow=200"));
                    }
                    else if (strcmp(cmdBuffer, "mode debug") == 0) {
                        protocolSetMode(PROTOCOL_MODE_DEBUG);
                        Serial.println(F("\n[INFO] 已切换到调试模式"));
                    }
                    // 尝试泵命令
                    else if (pumpProcessCommand(cmdBuffer)) {
                        // 泵命令已处理
                    }
                    // 尝试流量控制命令
                    else if (flowControlProcessCommand(cmdBuffer)) {
                        // 流量控制命令已处理
                    }
                    // 尝试阀门命令
                    else if (valveProcessCommand(cmdBuffer)) {
                        // 阀门命令已处理
                    }
                    else if (strcmp(cmdBuffer, "help") == 0 || strcmp(cmdBuffer, "?") == 0) {
                        Serial.println(F("\n========== Available Commands =========="));
                        Serial.println(F("Mode Control:"));
                        Serial.println(F("  mode debug        - Switch to debug mode"));
                        Serial.println(F("  mode host         - Switch to host protocol mode"));
                        Serial.println(F("\nFlow Control:"));
                        Serial.println(F("  flow set <sccm>   - Set target flow (0-500 sccm)"));
                        Serial.println(F("  flow stop         - Stop flow control"));
                        Serial.println(F("  flow status       - Show flow control status"));
                        Serial.println(F("  flow alarm        - Show alarm count"));
                        Serial.println(F("\nPump Control:"));
                        Serial.println(F("  pump set <duty>   - Set duty cycle (50-100%)"));
                        Serial.println(F("  pump get          - Get current duty"));
                        Serial.println(F("  pump status       - Get pump status"));
                        Serial.println(F("  pump on/off       - Enable/disable pump"));
                        Serial.println(F("\nValve Control:"));
                        Serial.println(F("  valve on          - Open valve"));
                        Serial.println(F("  valve off         - Close valve"));
                        Serial.println(F("  valve toggle      - Toggle valve state"));
                        Serial.println(F("  valve status      - Get valve status"));
                        Serial.println(F("\nSystem Commands:"));
                        Serial.println(F("  help / ?          - Show this help"));
                        Serial.println(F("======================================"));
                    }
                    else {
                        // 未知命令
                        Serial.println(F("[CMD] Unknown command"));
                        Serial.println(F("[CMD] Type 'help' for available commands"));
                    }

                    Serial.println(); // 空行分隔
                }

                // 清空缓冲区
                cmdPos = 0;
                cmdBuffer[0] = '\0';
            }
            break;
        }
        else if (c == '\b') {
            // 处理退格键
            if (cmdPos > 0) {
                cmdPos--;
                Serial.print("\b \b");
            }
        }
        else if (cmdPos < sizeof(cmdBuffer) - 1) {
            // 添加字符到缓冲区
            cmdBuffer[cmdPos++] = c;
            Serial.write(c); // 回显
        }
    }
}