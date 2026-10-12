#pragma once
// ============================================================
// flow_controller.h  –  流量控制系统
// ============================================================
// 提供两种控制模式：
//   1. 开环控制：0~500 sccm 线性映射到泵占空比 50~100%（当前使用）
//   2. PID控制：自动调节（保留，未使用）
//
// 功能：
//   flowControlInit(): 初始化流量控制系统
//   flowControlSetTarget(): 设置目标流量（开环下直接设置泵占空比）
//   flowControlSetMode(): 切换控制模式（开环/PID）
//   flowControlUpdate(): 更新控制（每个周期调用）
//   flowControlProcessCommand(): 处理串口命令
//
// 监控说明：
//   - 流量传感器仅作监控，实时显示在LCD/串口，不参与闭环调节
//   - 占空比限制在50%-100%
//   - 流量设定值限制在0-500 sccm
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "pid_controller.h"
#include "flow_sensor_fs4001.h"
#include "pump_controller.h"

// ------------------------------------------------------------
// 控制模式
// ------------------------------------------------------------
#define FLOW_CONTROL_MODE_OPENLOOP  0  // 开环控制（直接映射）
#define FLOW_CONTROL_MODE_PID       1  // PID控制

// ------------------------------------------------------------
// 流量控制状态
// ------------------------------------------------------------
struct FlowControlStatus {
    bool enabled;                // 控制使能
    bool alarm;                  // 报警状态
    uint8_t mode;                // 控制模式：0=开环, 1=PID
    float target_flow;           // 目标流量 (sccm)
    float current_flow;          // 当前流量 (sccm)
    float pump_duty;             // 泵占空比 (%)
    float pid_output;            // PID输出
    float pid_error;             // PID误差
    unsigned long last_update_ms;// 最后更新时间
    uint32_t alarm_count;        // 报警计数
};

// ------------------------------------------------------------
// 模块私有状态
// ------------------------------------------------------------
static FlowControlStatus _flowControlStatus = {};
static PIDController* _pidController = nullptr;
static bool _flowControlInited = false;

// ------------------------------------------------------------
// 开环控制：流量→占空比映射
// 下行 0~500 sccm 线性映射到泵 PWM 占空比 50~100%
//   duty(%) = 50 + flow(sccm) / 10
//   例：0→50%, 250→75%, 500→100%
// 流量传感器仅作监控显示，不参与闭环调节
// ------------------------------------------------------------
inline float flowToDutyOpenloop(float flow_sccm) {
    float duty = 50.0f + flow_sccm * 0.1f;

    // 限制范围
    if (duty < FLOW_CONTROL_PUMP_MIN_DUTY) duty = FLOW_CONTROL_PUMP_MIN_DUTY;
    if (duty > FLOW_CONTROL_PUMP_MAX_DUTY) duty = FLOW_CONTROL_PUMP_MAX_DUTY;

    return duty;
}

// ------------------------------------------------------------
// 公共函数实现
// ------------------------------------------------------------

// flowControlInit()
// 初始化流量控制系统
inline bool flowControlInit() {
    Serial.println(F("[FLOW_CTRL] Initializing flow control system..."));

    // PID输出范围：-50% ~ +50%（相对于最小占空比的偏移量）
    // 实际占空比 = 最小占空比 + PID输出
    float pid_output_min = -50.0f;
    float pid_output_max = FLOW_CONTROL_PUMP_MAX_DUTY - FLOW_CONTROL_PUMP_MIN_DUTY;

    // 创建PID控制器
    _pidController = new PIDController(
        FLOW_CONTROL_PID_KP,
        FLOW_CONTROL_PID_KI,
        FLOW_CONTROL_PID_KD,
        pid_output_min,
        pid_output_max
    );

    if (!_pidController) {
        Serial.println(F("[FLOW_CTRL] ERROR: Failed to create PID controller"));
        return false;
    }

    // 初始化状态
    _flowControlStatus.enabled = false;
    _flowControlStatus.alarm = false;
    _flowControlStatus.mode = FLOW_CONTROL_MODE_OPENLOOP;  // 默认开环模式
    _flowControlStatus.target_flow = 0.0f;
    _flowControlStatus.current_flow = 0.0f;
    _flowControlStatus.pump_duty = PUMP_DEFAULT_DUTY;
    _flowControlStatus.pid_output = 0.0f;
    _flowControlStatus.pid_error = 0.0f;
    _flowControlStatus.last_update_ms = millis();
    _flowControlStatus.alarm_count = 0;

    _flowControlInited = true;

    Serial.println(F("[FLOW_CTRL] Flow control system initialized"));
    Serial.printf("[FLOW_CTRL] Default mode: %s\n",
                  _flowControlStatus.mode == FLOW_CONTROL_MODE_OPENLOOP ? "OPENLOOP" : "PID");
    Serial.printf("[FLOW_CTRL] Flow range: %.1f - %.1f sccm\n",
                  FLOW_CONTROL_MIN_FLOW, FLOW_CONTROL_MAX_FLOW);
    Serial.printf("[FLOW_CTRL] Pump duty range: %.1f - %.1f%%\n",
                  FLOW_CONTROL_PUMP_MIN_DUTY, FLOW_CONTROL_PUMP_MAX_DUTY);
    Serial.printf("[FLOW_CTRL] Openloop mapping: duty = 50 + flow / 10 (0-500 sccm → 50-100%%)\n");
    Serial.printf("[FLOW_CTRL] PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                  FLOW_CONTROL_PID_KP, FLOW_CONTROL_PID_KI, FLOW_CONTROL_PID_KD);
    Serial.printf("[FLOW_CTRL] Alarm threshold: %.1f sccm\n", FLOW_CONTROL_ALARM_FLOW);

    return true;
}

// flowControlSetMode()
// 设置控制模式
inline bool flowControlSetMode(uint8_t mode) {
    if (mode != FLOW_CONTROL_MODE_OPENLOOP && mode != FLOW_CONTROL_MODE_PID) {
        Serial.println(F("[FLOW_CTRL] ERROR: Invalid mode"));
        return false;
    }

    _flowControlStatus.mode = mode;

    // 重置PID控制器
    if (_pidController) {
        _pidController->disable();
        _pidController->reset();
    }

#if FLOW_CONTROL_DEBUG_OUTPUT
    Serial.printf("[FLOW_CTRL] Mode set to: %s\n",
                  mode == FLOW_CONTROL_MODE_OPENLOOP ? "OPENLOOP" : "PID");
#endif

    return true;
}

// flowControlSetTarget()
// 设置目标流量
inline bool flowControlSetTarget(float target_flow) {
    if (!_flowControlInited) {
        Serial.println(F("[FLOW_CTRL] ERROR: Flow control not initialized"));
        return false;
    }

    // 限制目标流量范围
    if (target_flow < FLOW_CONTROL_MIN_FLOW) target_flow = FLOW_CONTROL_MIN_FLOW;
    if (target_flow > FLOW_CONTROL_MAX_FLOW) {
        Serial.printf("[FLOW_CTRL] ERROR: Target flow %.1f sccm exceeds max %.1f sccm\n",
                      target_flow, FLOW_CONTROL_MAX_FLOW);
        return false;
    }

    _flowControlStatus.target_flow = target_flow;
    _flowControlStatus.enabled = true;
    _flowControlStatus.alarm = false;

    // 根据模式设置控制
    if (_flowControlStatus.mode == FLOW_CONTROL_MODE_OPENLOOP) {
        // 开环控制：直接计算占空比
        _flowControlStatus.pump_duty = flowToDutyOpenloop(target_flow);
        pumpSetDuty(_flowControlStatus.pump_duty);
#if FLOW_CONTROL_DEBUG_OUTPUT
        Serial.printf("[FLOW_CTRL] Openloop: Target %.1f sccm → Duty %.1f%%\n",
                      target_flow, _flowControlStatus.pump_duty);
#endif
    } else {
        // PID控制
        if (_pidController) {
            _pidController->reset();
            _pidController->setSetpoint(target_flow);
            _pidController->enable();
        }
#if FLOW_CONTROL_DEBUG_OUTPUT
        Serial.printf("[FLOW_CTRL] PID: Target %.1f sccm\n", target_flow);
#endif
    }

    return true;
}

// flowControlStop()
// 停止流量控制
inline void flowControlStop() {
    _flowControlStatus.enabled = false;
    _flowControlStatus.target_flow = 0.0f;

    if (_pidController) {
        _pidController->disable();
        _pidController->reset();
    }

    // 关闭泵
    pumpDisable();

#if FLOW_CONTROL_DEBUG_OUTPUT
    Serial.println(F("[FLOW_CTRL] Flow control stopped"));
#endif
}

// flowControlUpdate()
// 更新控制（每个周期调用）
inline void flowControlUpdate(const FS4001Reading& flowReading) {
    if (!_flowControlInited || !_flowControlStatus.enabled) {
        return;
    }

    unsigned long now = millis();

    // 检查数据是否有效
    if (!flowReading.valid) {
#if FLOW_CONTROL_DEBUG_OUTPUT
        Serial.println(F("[FLOW_CTRL] WARNING: Invalid flow reading, skipping update"));
#endif
        return;
    }

    // 更新当前流量（仅监控，不参与控制/不停泵）
    _flowControlStatus.current_flow = flowReading.flow_rate;

    // 根据模式更新控制
    if (_flowControlStatus.mode == FLOW_CONTROL_MODE_PID) {
        // PID控制
        if (_pidController) {
            float pidOutput = _pidController->compute(_flowControlStatus.current_flow, now);

            // PID输出是偏移量，需要加上最小占空比
            // 实际占空比 = 最小占空比 + PID输出
            float actual_duty = FLOW_CONTROL_PUMP_MIN_DUTY + pidOutput;

            // 限制范围
            if (actual_duty < FLOW_CONTROL_PUMP_MIN_DUTY) actual_duty = FLOW_CONTROL_PUMP_MIN_DUTY;
            if (actual_duty > FLOW_CONTROL_PUMP_MAX_DUTY) actual_duty = FLOW_CONTROL_PUMP_MAX_DUTY;

            _flowControlStatus.pump_duty = actual_duty;
            _flowControlStatus.pid_output = pidOutput;
            _flowControlStatus.pid_error = _pidController->getError();

            // 应用PID输出到泵占空比
            pumpSetDuty(_flowControlStatus.pump_duty);

#if FLOW_CONTROL_DEBUG_OUTPUT
            Serial.printf("[FLOW_CTRL] PID: Flow %.1f, Target %.1f, Error %.1f, PID %.1f, Duty %.1f%%\n",
                          _flowControlStatus.current_flow, _flowControlStatus.target_flow,
                          _flowControlStatus.pid_error, pidOutput, _flowControlStatus.pump_duty);
#endif
        }
    }
    // 开环模式不需要周期更新，只在设置时计算

    _flowControlStatus.last_update_ms = now;
}

// flowControlGetStatus()
// 获取流量控制状态
inline FlowControlStatus flowControlGetStatus() {
    return _flowControlStatus;
}

// flowControlProcessCommand()
// 处理串口命令
inline bool flowControlProcessCommand(const char* command) {
    // 解析命令
    char cmd[32] = {0};
    char arg1[32] = {0};
    char arg2[32] = {0};

    int parsed = sscanf(command, "%31s %31s %31s", cmd, arg1, arg2);
    if (parsed < 1) return false;

    // 检查是否是流量控制命令
    if (strcmp(cmd, "flow") != 0) return false;

    // 处理流量控制命令
    if (parsed == 1) {
        // 只有 "flow" 命令，显示状态
        flowControlProcessCommand("flow status");
        return true;
    }

    // 处理子命令
    if (strcmp(arg1, "set") == 0 && parsed >= 3) {
        // flow set <sccm>
        float target = atof(arg2);
        if (flowControlSetTarget(target)) {
            Serial.printf("[FLOW_CTRL] Target flow set to %.1f sccm (%s mode)\n",
                          target, _flowControlStatus.mode == FLOW_CONTROL_MODE_OPENLOOP ? "openloop" : "PID");
        }
        return true;
    }
    else if (strcmp(arg1, "mode") == 0 && parsed >= 3) {
        // flow mode <openloop|pid>
        uint8_t newMode;
        if (strcmp(arg2, "openloop") == 0 || strcmp(arg2, "0") == 0) {
            newMode = FLOW_CONTROL_MODE_OPENLOOP;
        } else if (strcmp(arg2, "pid") == 0 || strcmp(arg2, "1") == 0) {
            newMode = FLOW_CONTROL_MODE_PID;
        } else {
            Serial.println(F("[FLOW_CTRL] ERROR: Invalid mode. Use 'openloop' or 'pid'"));
            return true;
        }

        if (flowControlSetMode(newMode)) {
            Serial.printf("[FLOW_CTRL] Control mode changed to %s\n",
                          newMode == FLOW_CONTROL_MODE_OPENLOOP ? "OPENLOOP" : "PID");
        }
        return true;
    }
    else if (strcmp(arg1, "stop") == 0) {
        // flow stop
        flowControlStop();
        Serial.println(F("[FLOW_CTRL] Flow control stopped"));
        return true;
    }
    else if (strcmp(arg1, "status") == 0) {
        // flow status
        Serial.println(F("[FLOW_CTRL] Status:"));
        Serial.printf("  Enabled: %s\n", _flowControlStatus.enabled ? "YES" : "NO");
        Serial.printf("  Mode: %s\n",
                      _flowControlStatus.mode == FLOW_CONTROL_MODE_OPENLOOP ? "OPENLOOP" : "PID");
        Serial.printf("  Alarm: %s\n", _flowControlStatus.alarm ? "YES" : "NO");
        Serial.printf("  Target flow: %.1f sccm\n", _flowControlStatus.target_flow);
        Serial.printf("  Current flow: %.1f sccm\n", _flowControlStatus.current_flow);
        Serial.printf("  Pump duty: %.1f%%\n", _flowControlStatus.pump_duty);
        if (_flowControlStatus.mode == FLOW_CONTROL_MODE_PID) {
            Serial.printf("  PID error: %.1f sccm\n", _flowControlStatus.pid_error);
        }
        Serial.printf("  Last update: %lu ms ago\n", millis() - _flowControlStatus.last_update_ms);
        Serial.printf("  Alarm count: %u\n", _flowControlStatus.alarm_count);
        return true;
    }
    else if (strcmp(arg1, "alarm") == 0) {
        // flow alarm
        Serial.printf("[FLOW_CTRL] Alarm count: %u\n", _flowControlStatus.alarm_count);
        Serial.printf("[FLOW_CTRL] Alarm status: %s\n", _flowControlStatus.alarm ? "ACTIVE" : "CLEARED");
        return true;
    }
    else if (strcmp(arg1, "reset") == 0) {
        // flow reset
        _flowControlStatus.alarm = false;
        _flowControlStatus.alarm_count = 0;
        Serial.println(F("[FLOW_CTRL] Alarm state reset"));
        return true;
    }
    else if (strcmp(arg1, "help") == 0 || strcmp(arg1, "?") == 0) {
        // flow help
        Serial.println(F("[FLOW_CTRL] Available commands:"));
        Serial.println(F("  flow set <sccm>        - Set target flow (0-500 sccm)"));
        Serial.println(F("  flow mode <openloop|pid> - Set control mode"));
        Serial.println(F("  flow stop              - Stop flow control"));
        Serial.println(F("  flow status            - Show flow control status"));
        Serial.println(F("  flow alarm             - Show alarm count"));
        Serial.println(F("  flow reset             - Reset alarm state"));
        Serial.println(F("  flow help              - Show this help"));
        Serial.println(F("\n  Openloop mode (recommended):"));
        Serial.println(F("    Direct flow→duty mapping, stable and fast"));
        Serial.println(F("    Mapping: duty = 50 + flow / 10 (0-500 sccm → 50-100%)"));
        Serial.println(F("\n  PID mode:"));
        Serial.println(F("    Automatic adjustment, needs tuning"));
        return true;
    }
    else {
        Serial.printf("[FLOW_CTRL] ERROR: Unknown command '%s'\n", arg1);
        Serial.println(F("[FLOW_CTRL] Use 'flow help' for available commands"));
        return true;
    }

    return false;
}