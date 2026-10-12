#pragma once
// ============================================================
// valve_controller.h  –  电磁阀控制驱动
// ============================================================
// 提供功能：
//   valveInit(): 初始化电磁阀控制
//   valveOpen(): 打开电磁阀（高电平）
//   valveClose(): 关闭电磁阀（低电平）
//   valveToggle(): 切换电磁阀状态
//   valveIsOpen(): 获取当前状态
//   valveProcessCommand(): 处理串口命令
//
// 硬件连接：
//   ESP32-S3 GPIO7 → 电磁阀控制端
//   高电平开，低电平关
// ============================================================

#include <Arduino.h>
#include "config.h"

// ------------------------------------------------------------
// 阀门状态
// ------------------------------------------------------------
struct ValveStatus {
    bool initialized;         // 初始化状态
    bool is_open;             // 当前状态（true=打开, false=关闭）
    unsigned long last_update_ms; // 最后更新时间戳
};

// ------------------------------------------------------------
// 模块私有状态
// ------------------------------------------------------------
static ValveStatus _valveStatus = {};

// ------------------------------------------------------------
// 公共函数实现
// ------------------------------------------------------------
inline bool valveInit();
inline bool valveOpen();
inline bool valveClose();
inline bool valveToggle();
inline bool valveIsOpen();
inline ValveStatus valveGetStatus();
inline bool valveProcessCommand(const char* command);

// valveInit()
// 初始化电磁阀控制
inline bool valveInit() {
    Serial.println(F("[VALVE] Initializing valve controller..."));

    // 配置阀门引脚为输出
    pinMode(VALVE_PIN, OUTPUT);

    // 设置默认状态
    digitalWrite(VALVE_PIN, VALVE_DEFAULT_STATE);
    _valveStatus.is_open = (VALVE_DEFAULT_STATE == VALVE_ACTIVE_LEVEL);
    _valveStatus.initialized = true;
    _valveStatus.last_update_ms = millis();

    Serial.println(F("[VALVE] Valve controller initialized"));
    Serial.printf("[VALVE] Pin: GPIO%d, Active Level: %s\n",
                  VALVE_PIN,
                  VALVE_ACTIVE_LEVEL == HIGH ? "HIGH" : "LOW");
    Serial.printf("[VALVE] Initial State: %s\n",
                  _valveStatus.is_open ? "OPEN" : "CLOSED");

    return true;
}

// valveOpen()
// 打开电磁阀
inline bool valveOpen() {
    if (!_valveStatus.initialized) {
        Serial.println(F("[VALVE] ERROR: Valve not initialized"));
        return false;
    }

    digitalWrite(VALVE_PIN, VALVE_ACTIVE_LEVEL);
    _valveStatus.is_open = true;
    _valveStatus.last_update_ms = millis();

#if VALVE_DEBUG_OUTPUT
    Serial.println(F("[VALVE] Valve opened"));
#endif

    return true;
}

// valveClose()
// 关闭电磁阀
inline bool valveClose() {
    if (!_valveStatus.initialized) {
        Serial.println(F("[VALVE] ERROR: Valve not initialized"));
        return false;
    }

    digitalWrite(VALVE_PIN, VALVE_INACTIVE_LEVEL);
    _valveStatus.is_open = false;
    _valveStatus.last_update_ms = millis();

#if VALVE_DEBUG_OUTPUT
    Serial.println(F("[VALVE] Valve closed"));
#endif

    return true;
}

// valveToggle()
// 切换电磁阀状态
inline bool valveToggle() {
    if (!_valveStatus.initialized) {
        Serial.println(F("[VALVE] ERROR: Valve not initialized"));
        return false;
    }

    if (_valveStatus.is_open) {
        valveClose();
    } else {
        valveOpen();
    }

    return true;
}

// valveIsOpen()
// 获取当前阀门状态
inline bool valveIsOpen() {
    return _valveStatus.is_open;
}

// valveGetStatus()
// 获取阀门状态
inline ValveStatus valveGetStatus() {
    return _valveStatus;
}

// valveProcessCommand()
// 处理串口命令
// 支持的命令：
//   valve on          - 打开电磁阀
//   valve off         - 关闭电磁阀
//   valve toggle      - 切换电磁阀状态
//   valve status      - 获取阀门状态
//   valve help        - 显示帮助信息
inline bool valveProcessCommand(const char* command) {
    // 解析命令
    char cmd[32] = {0};
    char arg1[32] = {0};
    char arg2[32] = {0};

    int parsed = sscanf(command, "%31s %31s %31s", cmd, arg1, arg2);
    if (parsed < 1) return false;

    // 检查是否是阀门命令
    if (strcmp(cmd, "valve") != 0) return false;

    // 处理阀门命令
    if (parsed == 1) {
        // 只有 "valve" 命令，显示状态
        valveProcessCommand("valve status");
        return true;
    }

    // 处理子命令
    if (strcmp(arg1, "on") == 0 || strcmp(arg1, "open") == 0) {
        // valve on / valve open
        valveOpen();
        return true;
    }
    else if (strcmp(arg1, "off") == 0 || strcmp(arg1, "close") == 0) {
        // valve off / valve close
        valveClose();
        return true;
    }
    else if (strcmp(arg1, "toggle") == 0) {
        // valve toggle
        valveToggle();
        return true;
    }
    else if (strcmp(arg1, "status") == 0) {
        // valve status
        Serial.println(F("[VALVE] Status:"));
        Serial.printf("  Initialized: %s\n", _valveStatus.initialized ? "YES" : "NO");
        Serial.printf("  State: %s\n", _valveStatus.is_open ? "OPEN" : "CLOSED");
        Serial.printf("  Pin: GPIO%d\n", VALVE_PIN);
        Serial.printf("  Active Level: %s\n",
                      VALVE_ACTIVE_LEVEL == HIGH ? "HIGH" : "LOW");
        Serial.printf("  Last update: %lu ms ago\n",
                      millis() - _valveStatus.last_update_ms);
        return true;
    }
    else if (strcmp(arg1, "help") == 0 || strcmp(arg1, "?") == 0) {
        // valve help
        Serial.println(F("[VALVE] Available commands:"));
        Serial.println(F("  valve on           - Open valve"));
        Serial.println(F("  valve off          - Close valve"));
        Serial.println(F("  valve toggle       - Toggle valve state"));
        Serial.println(F("  valve status       - Get valve status"));
        Serial.println(F("  valve help         - Show this help"));
        return true;
    }
    else {
        Serial.printf("[VALVE] ERROR: Unknown command '%s'\n", arg1);
        Serial.println(F("[VALVE] Use 'valve help' for available commands"));
        return true;
    }

    return false;
}