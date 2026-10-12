#pragma once
// ============================================================
// pid_controller.h  –  简单PID控制器
// ============================================================
// 用于流量控制：通过调节泵占空比来控制流量
// ============================================================

#include <Arduino.h>

// ------------------------------------------------------------
// PID参数结构
// ------------------------------------------------------------
struct PIDParams {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float output_min;   // 输出最小值
    float output_max;   // 输出最大值
    float integral_min; // 积分项最小值（抗积分饱和）
    float integral_max; // 积分项最大值
};

// ------------------------------------------------------------
// PID控制器类
// ------------------------------------------------------------
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float out_min, float out_max);

    // 设置目标值
    void setSetpoint(float setpoint);

    // 设置PID参数
    void setPID(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);

    // 计算输出
    float compute(float input, unsigned long now);

    // 重置PID状态
    void reset();

    // 获取当前状态
    float getSetpoint() const { return _setpoint; }
    float getOutput() const { return _lastOutput; }
    float getError() const { return _lastError; }
    bool isEnabled() const { return _enabled; }

    // 使能/禁用
    void enable() { _enabled = true; }
    void disable() { _enabled = false; reset(); }

private:
    PIDParams _params;
    float _setpoint;
    float _lastInput;
    float _lastOutput;
    float _integral;
    float _lastError;
    unsigned long _lastTime;
    bool _enabled;
};

// ------------------------------------------------------------
// 实现
// ------------------------------------------------------------

PIDController::PIDController(float kp, float ki, float kd, float out_min, float out_max) {
    _params.kp = kp;
    _params.ki = ki;
    _params.kd = kd;
    _params.output_min = out_min;
    _params.output_max = out_max;
    _params.integral_min = -out_max * 2.0f;  // 积分项限制为输出范围的2倍
    _params.integral_max = out_max * 2.0f;

    _setpoint = 0.0f;
    _lastInput = 0.0f;
    _lastOutput = 0.0f;
    _integral = 0.0f;
    _lastError = 0.0f;
    _lastTime = 0;
    _enabled = false;
}

void PIDController::setSetpoint(float setpoint) {
    _setpoint = setpoint;
}

void PIDController::setPID(float kp, float ki, float kd) {
    _params.kp = kp;
    _params.ki = ki;
    _params.kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    _params.output_min = min;
    _params.output_max = max;
    _params.integral_min = -max * 2.0f;
    _params.integral_max = max * 2.0f;
}

float PIDController::compute(float input, unsigned long now) {
    if (!_enabled) return 0.0f;

    // 首次计算
    if (_lastTime == 0) {
        _lastTime = now;
        _lastInput = input;
        _lastOutput = 0.0f;
        return 0.0f;
    }

    // 计算时间差（秒）
    float dt = (now - _lastTime) / 1000.0f;
    if (dt <= 0.0f) return _lastOutput;

    // 计算误差
    float error = _setpoint - input;
    _lastError = error;

    // 比例项
    float pTerm = _params.kp * error;

    // 积分项（带抗饱和）
    _integral += error * dt;
    if (_integral > _params.integral_max) _integral = _params.integral_max;
    if (_integral < _params.integral_min) _integral = _params.integral_min;
    float iTerm = _params.ki * _integral;

    // 微分项（基于输入变化，避免设定值突变导致的微分冲击）
    float dInput = input - _lastInput;
    float dTerm = -_params.kd * dInput / dt;

    // 计算输出
    float output = pTerm + iTerm + dTerm;

    // 限制输出范围
    if (output > _params.output_max) output = _params.output_max;
    if (output < _params.output_min) output = _params.output_min;

    // 更新状态
    _lastTime = now;
    _lastInput = input;
    _lastOutput = output;

    return output;
}

void PIDController::reset() {
    _integral = 0.0f;
    _lastError = 0.0f;
    _lastOutput = 0.0f;
    _lastTime = 0;
}
