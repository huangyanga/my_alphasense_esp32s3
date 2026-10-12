#pragma once
// ============================================================
// pump_controller.h  –  隔膜泵 PWM 控制驱动
// ============================================================
// 提供功能：
//   pumpInit(): 初始化 PWM 输出
//   pumpSetDuty(): 设置占空比 (0-100%)
//   pumpGetDuty(): 获取当前占空比
//   pumpProcessCommand(): 处理串口命令
//
// 硬件连接：
//   ESP32-S3 GPIO16 → 隔膜泵控制端
//
// PWM 参数：
//   频率: 20kHz
//   分辨率: 10-bit (0-1023)
//   默认占空比: 50%
// ============================================================

#include <Arduino.h>
#include "config.h"

// ------------------------------------------------------------
// LEDC 通道配置 (ESP32-S3 有多个 LEDC 通道)
// 使用通道0，避免与其他功能冲突
// ------------------------------------------------------------
#define PUMP_LEDC_CHANNEL      0  // LEDC 通道 0

// ------------------------------------------------------------
// 泵控制状态
// ------------------------------------------------------------
struct PumpStatus {
    float duty_percent;       // 当前占空比百分比
    uint16_t duty_raw;        // 占空比原始值 (0-1023)
    bool initialized;         // 初始化状态
    bool enabled;             // 使能状态
    unsigned long last_update_ms; // 最后更新时间戳
};

// ------------------------------------------------------------
// 模块私有状态
// ------------------------------------------------------------
static PumpStatus _pumpStatus = {};

// ------------------------------------------------------------
// 公共函数实现
// ------------------------------------------------------------
inline bool pumpSetDuty(float duty_percent);

// pumpInit()
// 初始化 PWM 输出
inline bool pumpInit() {
    Serial.println(F("[PUMP] Initializing PWM controller..."));

    // 配置 LEDC PWM 参数
    ledcSetup(PUMP_LEDC_CHANNEL, PUMP_PWM_FREQ_HZ, PUMP_PWM_RESOLUTION);

    // 绑定 PWM 引脚
    ledcAttachPin(PUMP_PWM_PIN, PUMP_LEDC_CHANNEL);

    // 设置默认占空比
    pumpSetDuty(PUMP_DEFAULT_DUTY);

    _pumpStatus.initialized = true;
    _pumpStatus.enabled = true;

    Serial.println(F("[PUMP] PWM controller initialized"));
    Serial.printf("[PUMP] Pin: GPIO%d, Freq: %d Hz, Duty: %.1f%%\n",
                  PUMP_PWM_PIN, PUMP_PWM_FREQ_HZ, _pumpStatus.duty_percent);

    return true;
}

// pumpSetDuty()
// 设置占空比 (0-100%)
inline bool pumpSetDuty(float duty_percent) {
    // 限制范围
    if (duty_percent < PUMP_MIN_DUTY) duty_percent = PUMP_MIN_DUTY;
    if (duty_percent > PUMP_MAX_DUTY) duty_percent = PUMP_MAX_DUTY;

    // 计算原始占空比值 (10-bit: 0-1023)
    uint16_t duty_raw = (uint16_t)((duty_percent / 100.0f) * ((1 << PUMP_PWM_RESOLUTION) - 1));

    // 设置 PWM 占空比
    ledcWrite(PUMP_LEDC_CHANNEL, duty_raw);

    // 更新状态
    _pumpStatus.duty_percent = duty_percent;
    _pumpStatus.duty_raw = duty_raw;
    _pumpStatus.last_update_ms = millis();

#if PUMP_DEBUG_OUTPUT
    Serial.printf("[PUMP] Duty set to %.1f%% (raw: %u)\n", duty_percent, duty_raw);
#endif

    return true;
}

// pumpGetDuty()
// 获取当前占空比百分比
inline float pumpGetDuty() {
    return _pumpStatus.duty_percent;
}

// pumpGetStatus()
// 获取泵状态
inline PumpStatus pumpGetStatus() {
    return _pumpStatus;
}

// pumpEnable()
// 启用泵
inline void pumpEnable() {
    _pumpStatus.enabled = true;
    ledcWrite(PUMP_LEDC_CHANNEL, _pumpStatus.duty_raw);
    Serial.println(F("[PUMP] Enabled"));
}

// pumpDisable()
// 禁用泵
inline void pumpDisable() {
    _pumpStatus.enabled = false;
    ledcWrite(PUMP_LEDC_CHANNEL, 0);
    Serial.println(F("[PUMP] Disabled"));
}

// pumpProcessCommand()
// 处理串口命令
// 支持的命令：
//   pump set <duty>   - 设置占空比 (0-100)
//   pump get          - 获取当前占空比
//   pump status       - 获取泵状态
//   pump on           - 启用泵
//   pump off          - 禁用泵
//   pump help         - 显示帮助信息
inline bool pumpProcessCommand(const char* command) {
    // 解析命令
    char cmd[32] = {0};
    char arg1[32] = {0};
    char arg2[32] = {0};

    int parsed = sscanf(command, "%31s %31s %31s", cmd, arg1, arg2);
    if (parsed < 1) return false;

    // 检查是否是泵命令
    if (strcmp(cmd, "pump") != 0) return false;

    // 处理泵命令
    if (parsed == 1) {
        // 只有 "pump" 命令，显示状态
        pumpProcessCommand("pump status");
        return true;
    }

    // 处理子命令
    if (strcmp(arg1, "set") == 0 && parsed >= 3) {
        // pump set <duty>
        float duty = atof(arg2);
        if (duty >= PUMP_MIN_DUTY && duty <= PUMP_MAX_DUTY) {
            pumpSetDuty(duty);
            Serial.printf("[PUMP] Duty set to %.1f%%\n", duty);
        } else {
            Serial.printf("[PUMP] ERROR: Invalid duty %.1f%% (range: %.1f-%.1f%%)\n",
                          duty, PUMP_MIN_DUTY, PUMP_MAX_DUTY);
        }
        return true;
    }
    else if (strcmp(arg1, "get") == 0) {
        // pump get
        Serial.printf("[PUMP] Current duty: %.1f%% (raw: %u)\n",
                      _pumpStatus.duty_percent, _pumpStatus.duty_raw);
        return true;
    }
    else if (strcmp(arg1, "status") == 0) {
        // pump status
        Serial.println(F("[PUMP] Status:"));
        Serial.printf("  Initialized: %s\n", _pumpStatus.initialized ? "YES" : "NO");
        Serial.printf("  Enabled: %s\n", _pumpStatus.enabled ? "YES" : "NO");
        Serial.printf("  Duty: %.1f%% (raw: %u)\n", _pumpStatus.duty_percent, _pumpStatus.duty_raw);
        Serial.printf("  Freq: %d Hz\n", PUMP_PWM_FREQ_HZ);
        Serial.printf("  Pin: GPIO%d\n", PUMP_PWM_PIN);
        Serial.printf("  Last update: %lu ms ago\n", millis() - _pumpStatus.last_update_ms);
        return true;
    }
    else if (strcmp(arg1, "on") == 0 || strcmp(arg1, "enable") == 0) {
        // pump on / pump enable
        pumpEnable();
        return true;
    }
    else if (strcmp(arg1, "off") == 0 || strcmp(arg1, "disable") == 0) {
        // pump off / pump disable
        pumpDisable();
        return true;
    }
    else if (strcmp(arg1, "help") == 0 || strcmp(arg1, "?") == 0) {
        // pump help
        Serial.println(F("[PUMP] Available commands:"));
        Serial.println(F("  pump set <duty>   - Set duty cycle (0-100%)"));
        Serial.println(F("  pump get          - Get current duty cycle"));
        Serial.println(F("  pump status       - Get pump status"));
        Serial.println(F("  pump on           - Enable pump"));
        Serial.println(F("  pump off          - Disable pump"));
        Serial.println(F("  pump help         - Show this help"));
        return true;
    }
    else {
        Serial.printf("[PUMP] ERROR: Unknown command '%s'\n", arg1);
        Serial.println(F("[PUMP] Use 'pump help' for available commands"));
        return true;
    }

    return false;
}
