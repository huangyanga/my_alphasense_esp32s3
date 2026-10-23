#pragma once
// ============================================================
// flow_sensor_fs4001.h  –  流量传感器驱动 (RS232)
//   支持两种传感器，开机由 IO39 自动选择：
//     IO39 低电平 → FS4008（9位串口 SERIAL_8E1 + FS4000协议）
//     IO39 高电平 → FS4001（8N1 + 原协议）
// ============================================================
// 提供功能：
//   FS4001Reading struct: 流量数据结构
//   flowSensorInit(): 初始化流量传感器 UART 通信
//   flowSensorSetMode(): 设置传感器工作模式
//   flowSensorRead(): 读取流量数据
//
// 硬件连接：
//   ESP32-S3 GPIO17 → 传感器 TX (通过 RS232-TTL 转换器)
//   ESP32-S3 GPIO18 → 传感器 RX (通过 RS232-TTL 转换器)
//   电源：8-24V DC，需要 RS232 电平转换器
//
// FS4001 通信协议：38400/8N1，查询模式
// FS4008 通信协议：38400/8E1（9位），读瞬时流量 9D F0 01 08 F9 0D，
//   应答 9D F0 03 FRH FRM FRL CRC 0D，流量=3字节大端÷1000(SLPM)，
//   本系统按 sccm 存储（SLPM×1000 = 原始3字节值）
// ============================================================

#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include "config.h"

// ------------------------------------------------------------
// FS4001 工作模式定义
// ------------------------------------------------------------
#define FS4001_MODE_OPERATION     0x54  // 操作模式：自动连续输出
#define FS4001_MODE_USER          0x00  // 用户模式：按需查询
#define FS4001_MODE_QUERY         0x55  // 查询瞬时流量
#define FS4001_SYNC_CMD           0x9d  // 同步命令

// ------------------------------------------------------------
// FS4001 UART 通信参数
// ------------------------------------------------------------
// 注意：FS4001_BAUD_RATE 在 config.h 中定义
#define FS4001_CMD_DELAY_MS       10    // 命令间隔 > 5ms
#define FS4001_RESPONSE_TIMEOUT_MS  2000 // 响应超时时间 (2000ms)
#define FS4001_LINE_TIMEOUT_MS     1000  // 单行读取超时 (1000ms)

// ------------------------------------------------------------
// 数据解析参数
// ------------------------------------------------------------
#define FS4001_FLOW_BUFFER_SIZE   256   // 接收缓冲区大小
#define FS4001_MAX_FLOW_VALUE     65535.0f // 最大流量值 (16位无符号，不限制)

// ------------------------------------------------------------
// 流量传感器测量结果
// ------------------------------------------------------------
struct FS4001Reading {
    float flow_rate;        // 流量值 (sccm)
    float voltage;          // 电压值 (V)
    uint16_t voltage_code;  // 电压代码
    uint8_t mode;           // 当前工作模式
    bool data_ready;        // 数据就绪标志
    bool error;             // 错误标志
    bool valid;             // 数据有效性标志
    unsigned long timestamp_ms; // 时间戳
};

// ------------------------------------------------------------
// FS4008（FS4000协议）查询命令
// 读瞬时流量: 9D F0 01 08 F9 0D（F9 = F0^01^08 全异或）
// ------------------------------------------------------------
#define FS4008_CMD_READ_FLOW_LEN   6
#define FS4008_FRAME_TIMEOUT_MS    500   // 应答超时

// ------------------------------------------------------------
// 模块私有状态 (使用前缀避免与其他模块冲突)
// ------------------------------------------------------------
static HardwareSerial* _flowSerial = nullptr;
static bool _flowSensorOk = false;
static uint8_t _flowErrCount = 0;
static uint8_t _currentMode = 0xFF;
static float _flowEma = 0.0f;
static bool _flowEmaInit = false;
static bool _flowIsFS4008 = false;   // true=FS4008（IO39低），false=FS4001

// ------------------------------------------------------------
// 辅助函数声明
static bool sendFS4001Command(uint8_t cmd);
static bool syncFS4001Sensor();
static bool parseOperationModeData(const char* buffer, FS4001Reading* reading);
static bool parseQueryResponse(const char* buffer, FS4001Reading* reading);
static float parseFloatValue(const char* buffer, const char* prefix);
static void flushRXBuffer();
inline bool flowSensorSetMode(uint8_t mode);
static bool fs4008ReadFlow(FS4001Reading* reading);
// ------------------------------------------------------------
// 公共函数实现
// ------------------------------------------------------------

// flowSensorInit()
// 初始化流量传感器 UART 通信（开机由 IO39 选择 FS4008/FS4001）
inline bool flowSensorInit() {
    Serial.println(F("[FS4001] Initializing UART..."));

    // 开机检测 IO39 选择流量传感器类型
    pinMode(FLOW_SELECT_PIN, INPUT_PULLUP);
    delay(1);
    _flowIsFS4008 = (digitalRead(FLOW_SELECT_PIN) == LOW);
    Serial.printf("[FS4001] IO39=%s -> %s\n",
                  _flowIsFS4008 ? "LOW" : "HIGH",
                  _flowIsFS4008 ? "FS4008 (9bit 8E1 + FS4000)" : "FS4001 (8N1)");

    // 初始化 UART1 用于流量传感器
    _flowSerial = &Serial1;

    Serial.printf("[FS4001] Configuring Serial1: RX=GPIO%d, TX=GPIO%d, Baud=%d\n",
                  FLOW_RX_PIN, FLOW_TX_PIN, FS4001_BAUD_RATE);

    // FS4008 每字节11位（起始+8数据+第9位D8+停止），用 8E1 校验位接收第9位；
    // FS4001 用 8N1
    if (_flowIsFS4008) {
        _flowSerial->begin(FS4001_BAUD_RATE, SERIAL_8E1, FLOW_RX_PIN, FLOW_TX_PIN);
    } else {
        _flowSerial->begin(FS4001_BAUD_RATE, SERIAL_8N1, FLOW_RX_PIN, FLOW_TX_PIN);
    }
    _flowSerial->setTimeout(FS4001_RESPONSE_TIMEOUT_MS);

    Serial.println(F("[FS4001] Serial1 configured"));

    // 清空接收缓冲区
    flushRXBuffer();
    Serial.println(F("[FS4001] RX buffer flushed"));

    delay(100); // 等待传感器启动

    // FS4008：无需同步/模式设置，直接可用
    if (_flowIsFS4008) {
        Serial.println(F("[FS4008] Mode: query-based, no sync needed"));
        _flowSensorOk = true;
        return true;
    }

    // 尝试与传感器同步
    Serial.println(F("[FS4001] Attempting sensor sync..."));
    if (!syncFS4001Sensor()) {
        Serial.printf("[FS4001] ERROR: Cannot sync with sensor (RX=GPIO%d, TX=GPIO%d)\n",
                      FLOW_RX_PIN, FLOW_TX_PIN);
        _flowSensorOk = false;
        return false;
    }

    Serial.println(F("[FS4001] Sensor sync successful"));

    // 【查询模式】直接进入用户模式（按需查询）
    if (!flowSensorSetMode(FS4001_MODE_USER)) {
        Serial.println("[FS4001] ERROR: Cannot set user query mode");
        _flowSensorOk = false;
        return false;
    }

    Serial.printf("[FS4001] Initialized successfully (UART: %d baud, Mode: QUERY)\n",
                  FS4001_BAUD_RATE);
    _flowSensorOk = true;
    return true;
}

// flowSensorSetMode()
// 设置传感器工作模式
inline bool flowSensorSetMode(uint8_t mode) {
    // if (!syncFS4001Sensor()) {
    //     return false;
    // }

    delay(FS4001_CMD_DELAY_MS);

    if (!sendFS4001Command(mode)) {
        Serial.printf("[FS4001] ERROR: Cannot send mode command 0x%02X\n", mode);
        return false;
    }

    // 等待响应
    delay(50);

    if (mode == FS4001_MODE_OPERATION) {
        _currentMode = FS4001_MODE_OPERATION;
        Serial.println("[FS4001] Set to OPERATION mode (auto output)");
    } else if (mode == FS4001_MODE_USER) {
        _currentMode = FS4001_MODE_USER;
        Serial.println("[FS4001] Set to USER mode (query based)");
    }

    return true;
}

// flowSensorRead()
// 读取流量传感器数据（查询模式）
inline FS4001Reading flowSensorRead() {
    FS4001Reading reading = {};
    reading.valid = false;
    reading.timestamp_ms = millis();

    // 检查传感器是否就绪
    if (!_flowSensorOk) {
        _flowErrCount++;
        if (_flowErrCount >= FLOW_UART_MAX_RETRIES) {
            _flowErrCount = 0;
            Serial.println("[FS4001] Attempting re-initialization...");
            _flowSensorOk = flowSensorInit();
        }
        return reading;
    }

    // FS4008：FS4000协议查询（9D F0 01 08 F9 0D）
    if (_flowIsFS4008) {
        reading.valid = fs4008ReadFlow(&reading);
        if (reading.valid) {
            reading.data_ready = true;
            reading.error = false;
            _flowErrCount = 0;
        }
        return reading;
    }

    // 【查询模式】先同步，再查询
    if (!syncFS4001Sensor()) {
        _flowSensorOk = false;
        _flowErrCount++;
        return reading;
    }

    delay(FS4001_CMD_DELAY_MS);

    if (!sendFS4001Command(FS4001_MODE_QUERY)) {
        Serial.println("[FS4001] ERROR: Cannot send query command");
        _flowSensorOk = false;
        _flowErrCount++;
        return reading;
    }

    // 读取响应 - 格式: "55 [空格] [16进制流量值] IN USER MODE......\n\r"
    char buffer[FS4001_FLOW_BUFFER_SIZE];
    unsigned long startTime = millis();

    // 等待响应（最多500ms）
    while (_flowSerial->available() == 0 && (millis() - startTime) < 50) {
        delay(1);
    }

    int bytesRead = _flowSerial->readBytesUntil('\n', buffer, sizeof(buffer) - 1);

    if (bytesRead > 0) {
        buffer[bytesRead] = '\0';

#if FS4001_DEBUG_OUTPUT
        // 调试：显示接收到的原始数据
        Serial.printf("[FS4001] RX (%d bytes): \"", bytesRead);
        for (int i = 0; i < bytesRead; i++) {
            if (buffer[i] == '\n') Serial.print("\\n");
            else if (buffer[i] == '\r') Serial.print("\\r");
            else if (buffer[i] == '\t') Serial.print("\\t");
            else if (buffer[i] >= 32 && buffer[i] <= 126) Serial.print(buffer[i]);
            else Serial.printf("\\x%02X", (uint8_t)buffer[i]);
        }
        Serial.println("\"");
#endif

        reading.valid = parseQueryResponse(buffer, &reading);
    } else {
        Serial.println("[FS4001] WARNING: No response to query");
        _flowErrCount++;
    }

    if (reading.valid) {
        reading.mode = _currentMode;
        reading.data_ready = true;
        reading.error = false;
        _flowErrCount = 0;
    }

    return reading;
}

// ------------------------------------------------------------
// 私有辅助函数实现
// ------------------------------------------------------------

// 发送命令到传感器
static bool sendFS4001Command(uint8_t cmd) {
    if (!_flowSerial) return false;

    _flowSerial->write(cmd);
    _flowSerial->flush();
    return true;
}

// 与传感器同步（发送0x9D，不等待响应）
static bool syncFS4001Sensor() {
    // 发送同步命令
    if (!sendFS4001Command(FS4001_SYNC_CMD)) {
        return false;
    }

    // 【注意】传感器不会响应同步命令，只响应查询命令
    // 所以这里不需要等待和读取响应，直接返回成功
    return true;
}

// ------------------------------------------------------------
// FS4008 读瞬时流量（FS4000协议）
//   发送: 9D F0 01 08 F9 0D
//   应答: 9D F0 03 FRH FRM FRL CRC 0D
//   流量: (FRH<<16 | FRM<<8 | FRL) ÷ 1000 = SLPM
//         本系统按 sccm 存储：SLPM×1000 = 3字节大端原始值
//   校验: CRC = F0 ^ 03 ^ FRH ^ FRM ^ FRL
// ------------------------------------------------------------
static bool fs4008ReadFlow(FS4001Reading* reading) {
    if (!_flowSerial) return false;

    flushRXBuffer();

    // 发送查询命令（HEX）
    static const uint8_t query[FS4008_CMD_READ_FLOW_LEN] = {0x9D, 0xF0, 0x01, 0x08, 0xF9, 0x0D};
    _flowSerial->write(query, FS4008_CMD_READ_FLOW_LEN);
    _flowSerial->flush();

    // 找帧头 0x9D（最多等500ms）
    uint8_t b = 0;
    unsigned long start = millis();
    while (millis() - start < FS4008_FRAME_TIMEOUT_MS) {
        if (_flowSerial->available()) {
            b = _flowSerial->read();
            if (b == 0x9D) break;
        } else {
            delay(1);
        }
    }
    if (b != 0x9D) {
        Serial.println(F("[FS4008] No frame header (0x9D)"));
        return false;
    }

    // 读取后续 7 字节: F0 03 FRH FRM FRL CRC 0D
    uint8_t rest[7];
    int m = _flowSerial->readBytes(rest, 7);
    if (m < 7) {
        Serial.printf("[FS4008] Short frame (%d bytes)\n", m);
        return false;
    }
    if (rest[0] != 0xF0 || rest[1] != 0x03) {
        Serial.printf("[FS4008] Bad frame: 0x%02X 0x%02X\n", rest[0], rest[1]);
        return false;
    }

    uint8_t d0 = rest[2], d1 = rest[3], d2 = rest[4], crc = rest[5];
    uint8_t calc = 0xF0 ^ 0x03 ^ d0 ^ d1 ^ d2;
    if (calc != crc) {
        Serial.printf("[FS4008] CRC fail: calc=0x%02X rx=0x%02X\n", calc, crc);
        return false;
    }

    uint32_t raw = ((uint32_t)d0 << 16) | ((uint32_t)d1 << 8) | d2;
    reading->flow_rate = (float)raw;   // sccm（= SLPM × 1000）
    reading->voltage = 0.0f;
    reading->voltage_code = 0;
    reading->mode = 0;

#if FS4001_DEBUG_OUTPUT
    Serial.printf("[FS4008] Flow: raw=%lu -> %.3f sccm (%.3f SLPM)\n",
                  raw, reading->flow_rate, raw / 1000.0f);
#endif
    return true;
}

// 解析操作模式数据
// 格式: 多行数据
//   V=      0
//   F=      393
//   A= 0
//   ;
static bool parseOperationModeData(const char* buffer, FS4001Reading* reading) {
    // 查找流量值
    const char* flowPrefix = "F=";
    float flowValue = parseFloatValue(buffer, flowPrefix);

    // 查找电压值（某些模式可能不发送电压数据）
    const char* voltagePrefix = "V=";
    float voltageValue = parseFloatValue(buffer, voltagePrefix);
    bool hasVoltage = (voltageValue >= 0);  // -1 表示未找到电压值

    // 检查是否找到了有效的流量值（>= 0 表示找到了，包括0值）
    if (flowValue >= 0 && flowValue <= FS4001_MAX_FLOW_VALUE) {
        reading->flow_rate = flowValue;

        // 如果有电压数据则使用，否则设为0
        if (hasVoltage) {
            reading->voltage = voltageValue;
            reading->voltage_code = (uint16_t)(voltageValue * 1000);
        } else {
            reading->voltage = 0.0f;
            reading->voltage_code = 0;
        }

        // 调试输出
        if (hasVoltage) {
            Serial.printf("[FS4001] Parsed: F=%.3f, V=%.2f\n", flowValue, voltageValue);
        } else {
            Serial.printf("[FS4001] Parsed: F=%.3f (no voltage data)\n", flowValue);
        }

        return true;
    }

    // 调试输出
    Serial.printf("[FS4001] Parse failed - FlowValue=%.2f, Buffer: \"%s\"\n", flowValue, buffer);

    return false;
}

// 解析查询响应
// 格式: "U [空格] [十进制流量值] IN USER MODE......\n\r"
// 例如: "U     3189IN USER MODE......\n\r"
// 16进制: 55 20 20 20 20 33 31 38 39 49 4E...
// ASCII:  U        3  1  8  9  I  N ...
// 流量值 = 3189 / 1000 = 3.189 sccm
static bool parseQueryResponse(const char* buffer, FS4001Reading* reading) {
    // 查找 "IN " 标记（固定分隔符）
    const char* inMarker = strstr(buffer, "IN ");
    if (!inMarker) {
#if FS4001_DEBUG_OUTPUT
        Serial.printf("[FS4001] Parse failed - Cannot find \"IN \" marker\n");
#endif
        return false;
    }

    // 从 "IN " 前面向后查找十进制数字
    const char* endPtr = inMarker - 1;

    // 跳过 "IN " 之前的所有空格
    while (endPtr > buffer && *endPtr == ' ') {
        endPtr--;
    }

    // 从后往前提取十进制数字
    const char* startPtr = endPtr;
    while (startPtr >= buffer && isdigit(*startPtr)) {
        startPtr--;
    }
    startPtr++;  // 回到最后一个有效的十进制字符

    // 检查是否找到了有效的十进制字符串
    if (startPtr > endPtr) {
#if FS4001_DEBUG_OUTPUT
        Serial.printf("[FS4001] Parse failed - No decimal digits found before \"IN \"\n");
#endif
        return false;
    }

    // 提取十进制字符串
    size_t numLen = endPtr - startPtr + 1;
    char numStr[32];
    if (numLen >= sizeof(numStr)) numLen = sizeof(numStr) - 1;
    strncpy(numStr, startPtr, numLen);
    numStr[numLen] = '\0';

    // 解析十进制流量值
    long rawValue = strtol(numStr, nullptr, 10);

    // 转换为sccm：除以1000（假设返回值是整数表示小数点后3位）
    float flowValue = rawValue / 1000.0f;

    if (flowValue >= 0) {
        reading->flow_rate = flowValue;
        reading->voltage = 0.0f; // 查询模式不提供电压数据
        reading->voltage_code = 0;

#if FS4001_DEBUG_OUTPUT
        Serial.printf("[FS4001] Parsed: decimal=\"%s\" (raw=%ld) -> F=%.3f sccm\n",
                      numStr, rawValue, flowValue);
#endif

        return true;
    }

#if FS4001_DEBUG_OUTPUT
    Serial.printf("[FS4001] Parse failed - Invalid flow value: %.3f (from decimal: %s)\n",
                  flowValue, numStr);
#endif

    return false;
}

// 从缓冲区解析浮点值
// 支持多行数据和前面的空格
static float parseFloatValue(const char* buffer, const char* prefix) {
    const char* ptr = strstr(buffer, prefix);
    if (!ptr) return -1.0f;

    ptr += strlen(prefix); // 跳过前缀

    // 跳过前导空格
    while (*ptr == ' ') {
        ptr++;
    }

    // 查找数值结束位置（遇到非数字字符停止）
    const char* end = ptr;
    while (*end && (*end == '.' || (*end >= '0' && *end <= '9') || (*end == '-'))) {
        end++;
    }

    // 提取数值字符串
    char numStr[32];
    size_t len = end - ptr;
    if (len >= sizeof(numStr)) len = sizeof(numStr) - 1;
    strncpy(numStr, ptr, len);
    numStr[len] = '\0';

    float result = strtof(numStr, nullptr);
    return result;
}

// 清空接收缓冲区
static void flushRXBuffer() {
    if (_flowSerial) {
        while (_flowSerial->available() > 0) {
            _flowSerial->read();
        }
    }
}