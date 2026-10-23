#pragma once
// ============================================================
// protocol_handler.h  –  呼气检测系统串口通讯协议
// ============================================================
// 协议版本：v1.0
// 报文格式：[A|Q|R] <CmdId>, <key=value,key2=value> CRLF
// 编码：ASCII 文本，无二进制数据
// ============================================================

#include <Arduino.h>
#include <stdarg.h>
#include <Preferences.h>
#include "config.h"
#include "alphasense_b43f.h"
#include "flow_sensor_fs4001.h"
#include "pump_controller.h"
#include "flow_controller.h"
#include "valve_controller.h"

// ------------------------------------------------------------
// 协议模式
// ------------------------------------------------------------
#define PROTOCOL_MODE_DEBUG   0  // 调试模式（原有命令）
#define PROTOCOL_MODE_HOST    1  // 主机通讯模式（新协议）

// ------------------------------------------------------------
// 报文类型
// ------------------------------------------------------------
#define MSG_TYPE_ACTION       'A'  // Action 命令（上位机→下位机）
#define MSG_TYPE_QUERY        'Q'  // Query 查询（上位机→下位机）
#define MSG_TYPE_RESPONSE     'R'  // Response/主动上报（下位机→上位机）

// ------------------------------------------------------------
// 气体类型
// ------------------------------------------------------------
#define GAS_TYPE_NO           "NO"
#define GAS_TYPE_CO           "CO"

// ------------------------------------------------------------
// 错误码
// ------------------------------------------------------------
#define ERR_INVALID_CMD       "01"  // 非法命令
#define ERR_INVALID_PARAM     "02"  // 非法参数
#define ERR_NOT_STARTED       "03"  // 未开始测试
#define ERR_NOT_READY         "04"  // 采样器未就绪

// ------------------------------------------------------------
// 测试状态
// ------------------------------------------------------------
struct TestState {
    bool is_running;              // 测试运行状态
    bool has_started;             // 是否已启动
    unsigned long start_time_ms;   // 开始时间戳
    unsigned long elapsed_s;      // 已运行秒数
    char gas_type[8];             // 气体类型 "NO" 或 "CO"
    float target_flow;            // 目标流速 sccm
    float total_wa;               // WE-AE 累积值 (V, 按采样次数累加)
    uint32_t sample_count;        // 采样次数
    float last_wa_v;              // 最近一次 wa 采样值 (V)
    unsigned long test_duration_s; // 测试时长(秒)，0=不限时自动停止
    float last_test_avg_ppb;      // 最近一次测试的平均浓度 (ppb)，供LCD等显示
    float last_test_last_ppb;     // 最近一次测试的最后浓度 (ppb)
};

// ------------------------------------------------------------
// 协议配置
// ------------------------------------------------------------
#define PROTOCOL_AUTO_UPLOAD_INTERVAL_MS  500   // 自动上传间隔 500ms
#define PROTOCOL_TICK_INTERVAL_MS         1000  // TICK 上报间隔 1秒
#define PROTOCOL_MAX_LINE_LENGTH          128   // 最大报文长度

// ------------------------------------------------------------
// 模块私有状态
// ------------------------------------------------------------
static uint8_t _protocolMode = PROTOCOL_MODE_HOST;  // 默认主机协议模式
static TestState _testState = {
    false, false, 0, 0, GAS_TYPE_NO, 0.0f, 0.0f, 0, 0.0f, 0, 0.0f, 0.0f
};
static unsigned long _lastAutoUploadMs = 0;
static unsigned long _lastTickMs = 0;

// ------------------------------------------------------------
// 零点校准状态 (ZERO)
// ------------------------------------------------------------
#define ZERO_CAL_SAMPLE_MS      50      // 采样间隔 20Hz
#define ZERO_CAL_DEFAULT_N      60      // 默认采样点数
#define ZERO_CAL_MIN_N          10      // 最少采样点数
#define ZERO_CAL_MAX_N          300     // 最多采样点数

struct ZeroCalState {
    bool   active;              // 校准进行中
    uint16_t target_n;          // 目标采样点数
    uint16_t count;             // 已采样点数
    float  sum_we;              // WE 原始电压累加 (V)
    float  sum_ae;              // AE 原始电压累加 (V)
    float  min_wa;              // 差分 wa=WE-AE 最小值 (V)，用于计算spread上报
    float  max_wa;              // 差分 wa=WE-AE 最大值 (V)
    unsigned long last_sample_ms;  // 上次采样时刻
};

static ZeroCalState _zeroCal = {};
static Preferences  _prefs;
static bool  _zeroOffsetsValid = false;   // 双通道偏移是否已校准（NVS zero_valid）

// ------------------------------------------------------------
// 公共函数声明
// ------------------------------------------------------------
inline bool protocolInit();
inline void protocolSetMode(uint8_t mode);
inline uint8_t protocolGetMode();
inline void protocolProcess();
inline bool protocolHandleCommand(const char* line);
inline void protocolSendResponse(const char* msgType, const char* format, ...);
inline void protocolSendError(const char* errorCode);
inline void protocolUpdate(const B43FReading& gasReading, const FS4001Reading& flowReading);
inline bool protocolTestRunning();      // 测试是否运行中
inline float protocolTestAvgPpb();      // 最近一次测试平均浓度 (ppb)
inline float protocolTestLastPpb();     // 最近一次测试最后浓度 (ppb)
inline unsigned long protocolTestDurationS();  // 本次测试下行时长(秒)，0=不限时
inline bool handleCmd_ZERO(const char* line);       // A ZERO 零点校准/清除
inline bool handleQuery_BASELINE(const char* line); // Q BASELINE 查询零点
inline void protocolZeroCalUpdate();    // 零点校准采样（主循环调用，内部20Hz节流）

// ============================================================
// 内部辅助函数
// ============================================================

// 解析键值对
inline bool parseKeyValue(const char* line, const char* key, char* value, size_t maxLen) {
    char searchKey[32];
    snprintf(searchKey, sizeof(searchKey), "%s=", key);

    const char* p = strstr(line, searchKey);
    if (!p) {
#if 0  // 调试：显示未找到的键（默认关闭）
        Serial.printf("[HOST] Key '%s' not found in line\n", key);
#endif
        return false;
    }

    p += strlen(searchKey);

    // 查找分隔符或结束
    size_t len = 0;
    while (p[len] != '\0' && p[len] != ',' && p[len] != ' ' && p[len] != '\r' && p[len] != '\n') {
        if (len < maxLen - 1) {
            value[len] = p[len];
        }
        len++;
    }
    value[len] = '\0';

#if 0  // 调试：显示解析的值（默认关闭）
    Serial.printf("[HOST] Parsed %s=%s\n", key, value);
#endif

    return true;
}

// 获取整数值
inline bool getIntParam(const char* line, const char* key, int* value) {
    char valStr[32];
    if (!parseKeyValue(line, key, valStr, sizeof(valStr))) return false;
    *value = atoi(valStr);
    return true;
}

// 获取浮点值
inline bool getFloatParam(const char* line, const char* key, float* value) {
    char valStr[32];
    if (!parseKeyValue(line, key, valStr, sizeof(valStr))) return false;
    *value = atof(valStr);
    return true;
}

// 获取字符串值
inline bool getStringParam(const char* line, const char* key, char* value, size_t maxLen) {
    return parseKeyValue(line, key, value, maxLen);
}

// ============================================================
// 协议初始化
// ============================================================
inline bool protocolInit() {
    // 加载持久化双通道偏移（NVS，断电不丢）
    _prefs.begin("b43f", false);
    _zeroOffsetsValid = _prefs.getBool("zero_valid", false);
    if (_zeroOffsetsValid) {
        float we_off = _prefs.getFloat("we_off", WE_OFFSET_V);
        float ae_off = _prefs.getFloat("ae_off", AE_OFFSET_V);
        adsSetOffsets(we_off, ae_off);
    }

    Serial.println(F("[PROTOCOL] Protocol handler initialized"));
    Serial.printf("[PROTOCOL] Mode: %s\n",
                  _protocolMode == PROTOCOL_MODE_DEBUG ? "DEBUG" : "HOST");
    Serial.printf("[PROTOCOL] Calibration offsets: %s (WE=%.4f V, AE=%.4f V)\n",
                  _zeroOffsetsValid ? "VALID" : "NONE",
                  adsGetWeOffsetV(), adsGetAeOffsetV());
    return true;
}

// ============================================================
// 模式切换
// ============================================================
inline void protocolSetMode(uint8_t mode) {
    if (mode != PROTOCOL_MODE_DEBUG && mode != PROTOCOL_MODE_HOST) {
        Serial.println(F("[PROTOCOL] ERROR: Invalid mode"));
        return;
    }

    _protocolMode = mode;
    Serial.printf("[PROTOCOL] Mode changed to: %s\n",
                  _protocolMode == PROTOCOL_MODE_DEBUG ? "DEBUG" : "HOST");
}

inline uint8_t protocolGetMode() {
    return _protocolMode;
}

// 测试状态查询（供 LCD 等外部模块读取）
inline bool protocolTestRunning() {
    return _testState.is_running;
}
inline float protocolTestAvgPpb() {
    return _testState.last_test_avg_ppb;
}
inline float protocolTestLastPpb() {
    return _testState.last_test_last_ppb;
}
inline unsigned long protocolTestDurationS() {
    return _testState.test_duration_s;
}

// ============================================================
// 发送响应
// ============================================================
inline void protocolSendResponse(const char* msgType, const char* format, ...) {
    char buffer[PROTOCOL_MAX_LINE_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.printf("%c %s\r\n", msgType[0], buffer);
}

// ============================================================
// 发送错误响应
// ============================================================
inline void protocolSendError(const char* errorCode) {
    protocolSendResponse("R", "ERR,%s", errorCode);
}

// ============================================================
// 命令处理：START
// ============================================================
inline bool handleCmd_START(const char* line) {
    int t;
    char gas[8];
    float flow;

#if 0  // 调试：显示原始命令（默认关闭）
    Serial.printf("[HOST] Handling START: %s\n", line);
#endif

    if (!getIntParam(line, "t", &t)) {
#if 0  // 调试：显示详细错误信息
        Serial.println("[HOST] Missing or invalid 't' parameter");
#endif
        protocolSendError(ERR_INVALID_PARAM);
        return false;
    }
    if (!getStringParam(line, "gas", gas, sizeof(gas))) {
#if 0  // 调试：显示详细错误信息
        Serial.println("[HOST] Missing or invalid 'gas' parameter");
#endif
        protocolSendError(ERR_INVALID_PARAM);
        return false;
    }
    if (!getFloatParam(line, "flow", &flow)) {
#if 0  // 调试：显示详细错误信息
        Serial.println("[HOST] Missing or invalid 'flow' parameter");
#endif
        protocolSendError(ERR_INVALID_PARAM);
        return false;
    }

    // 验证气体类型
    if (strcmp(gas, GAS_TYPE_NO) != 0 && strcmp(gas, GAS_TYPE_CO) != 0) {
#if 0  // 调试：显示详细错误信息
        Serial.printf("[HOST] Invalid gas type: %s\n", gas);
#endif
        protocolSendError(ERR_INVALID_PARAM);
        return false;
    }

    // 验证流速范围
    if (flow < 0 || flow > FLOW_CONTROL_MAX_FLOW) {
#if 0  // 调试：显示详细错误信息
        Serial.printf("[HOST] Invalid flow rate: %.1f\n", flow);
#endif
        protocolSendError(ERR_INVALID_PARAM);
        return false;
    }

#if 0  // 调试：显示解析的参数（默认关闭）
    Serial.printf("[HOST] Parsed: t=%d, gas=%s, flow=%.1f\n", t, gas, flow);
#endif

    // 初始化测试状态
    _testState.has_started = true;
    _testState.is_running = true;
    _testState.start_time_ms = millis();
    _testState.elapsed_s = 0;
    _testState.test_duration_s = (t > 0) ? (unsigned long)t : 0;  // 下行测试时长，0=不限时
    strncpy(_testState.gas_type, gas, sizeof(_testState.gas_type));
    _testState.target_flow = flow;
    flowControlSetTarget(flow);   // 开环：0-500 sccm → 泵占空比 50-100%
    _testState.total_wa = 0.0f;
    _testState.sample_count = 0;
    _testState.last_wa_v = 0.0f;

    // 打开阀门和泵
    valveOpen();
    pumpEnable();

    // 读取当前传感器数据用于响应
    B43FReading currentReading = adsRead();
    float we_mv = currentReading.valid ? currentReading.we_v * 1000.0f : 0.0f;
    float ae_mv = currentReading.valid ? currentReading.ae_v * 1000.0f : 0.0f;
    float wa_mv = currentReading.valid ? currentReading.signal_ema * 1000.0f : 0.0f;  // 使用EMA滤波值

    // 响应
    protocolSendResponse("R", "START_OK, t=%d, gas=%s, flow=%.1f, we=%.1f, ae=%.1f, wa=%.1f",
                        t, gas, flow, we_mv, ae_mv, wa_mv);

    return true;
}

// ============================================================
// 命令处理：STOP
// ============================================================
inline bool handleCmd_STOP(const char* line) {
    if (!_testState.has_started) {
        protocolSendError(ERR_NOT_STARTED);
        return false;
    }

    _testState.is_running = false;

    // 计算平均浓度：conc(ppb) = 2 × ΔV(mV)，ΔV 取全测试周期 EMA 平均值（已含偏移校正）
    float avg_wa_mv = _testState.sample_count > 0 ?
                      (_testState.total_wa / _testState.sample_count) * 1000.0f : 0.0f;
    float conc_ppb = 2.0f * avg_wa_mv;
    float last_wa_mv = _testState.last_wa_v * 1000.0f;  // 最后一次 wa (mV)

    // 关闭阀门和泵
    valveClose();
    pumpDisable();

    // 响应
    protocolSendResponse("R", "STOP_OK, dur=%lu, gas=%s, last_wa=%.2f, conc=%.2f, unit=ppb",
                        _testState.elapsed_s,
                        _testState.gas_type,
                        last_wa_mv,
                        conc_ppb);

    // 保存本次测试统计（供LCD等显示，直到下一次测试结束才更新）
    _testState.last_test_avg_ppb  = conc_ppb;
    _testState.last_test_last_ppb = 2.0f * last_wa_mv;

    // 重置状态
    _testState.has_started = false;
    _testState.total_wa = 0.0f;
    _testState.sample_count = 0;
    _testState.last_wa_v = 0.0f;

    return true;
}

// ============================================================
// 命令处理：ZERO（零点校准）
//   A ZERO, n=<点数>    开始校准（n=10~300，默认60）
//   A ZERO, CLR         清除已保存偏移（WE_OFFSET_V / AE_OFFSET_V 恢复为0）
// 零气下以约20Hz分别采样 WE、AE 原始电压共 n 点，各自取平均
// 作为 WE_OFFSET_V / AE_OFFSET_V 并持久化到NVS（断电不丢）。
// 校准后每次上传 WE/AE 都使用这两个偏移：we_v=(we_raw_v-偏移)×增益
// ============================================================
inline bool handleCmd_ZERO(const char* line) {
    // 清除偏移（WE/AE 偏移恢复为 0）
    if (strstr(line, "CLR") || strstr(line, "clr")) {
        _zeroCal.active = false;
        _zeroOffsetsValid = false;
        adsResetOffsets();
        _prefs.putFloat("we_off", 0.0f);
        _prefs.putFloat("ae_off", 0.0f);
        _prefs.putBool("zero_valid", false);
        protocolSendResponse("R", "ZERO_CLR_OK");
        return true;
    }

    // 测试已开始（A START 后、A STOP 前）不允许校准 → ERR,03
    if (_testState.is_running) {
        protocolSendError(ERR_NOT_STARTED);
        return false;
    }

#if ADS1115_DIFFERENTIAL_MODE
    // 硬件差分模式无法分别采样 WE/AE，不支持双通道偏移校准
    protocolSendError(ERR_INVALID_CMD);
    return false;
#endif

    // 解析采样点数（默认60，范围10~300）
    int n = ZERO_CAL_DEFAULT_N;
    getIntParam(line, "n", &n);
    if (n < ZERO_CAL_MIN_N) n = ZERO_CAL_MIN_N;
    if (n > ZERO_CAL_MAX_N) n = ZERO_CAL_MAX_N;

    // 启动校准状态机（非阻塞，主循环 protocolZeroCalUpdate 采样）
    _zeroCal.active = true;
    _zeroCal.target_n = (uint16_t)n;
    _zeroCal.count = 0;
    _zeroCal.sum_we = 0.0f;
    _zeroCal.sum_ae = 0.0f;
    _zeroCal.min_wa = 1e9f;
    _zeroCal.max_wa = -1e9f;
    _zeroCal.last_sample_ms = millis();

    protocolSendResponse("R", "ZERO_BEGIN, n=%d", n);
    return true;
}

// 查询零点（差分零点 = WE偏移 - AE偏移，mV）
inline bool handleQuery_BASELINE(const char* line) {
    float zero_wa_mv = (adsGetWeOffsetV() - adsGetAeOffsetV()) * 1000.0f;
    protocolSendResponse("R", "BASELINE, zero_wa=%.2f, valid=%d",
                        zero_wa_mv, _zeroOffsetsValid ? 1 : 0);
    return true;
}

// 零点校准采样（主循环每轮调用，内部按20Hz节流，非阻塞）
// 分别累加 WE/AE 原始电压，共 n 点后各自取平均 → WE_OFFSET_V / AE_OFFSET_V
inline void protocolZeroCalUpdate() {
    if (!_zeroCal.active) return;

    unsigned long now = millis();
    if (now - _zeroCal.last_sample_ms < ZERO_CAL_SAMPLE_MS) return;
    _zeroCal.last_sample_ms = now;

    B43FReading r = adsRead();
    if (!r.valid) return;  // 读取失败则跳过（不计入点数）

    float we = r.we_raw_v;  // 原始电压（未校正）
    float ae = r.ae_raw_v;
    float wa = we - ae;     // 差分，仅用于稳定性（极差）判定
    _zeroCal.sum_we += we;
    _zeroCal.sum_ae += ae;
    if (wa < _zeroCal.min_wa) _zeroCal.min_wa = wa;
    if (wa > _zeroCal.max_wa) _zeroCal.max_wa = wa;
    _zeroCal.count++;

    if (_zeroCal.count >= _zeroCal.target_n) {
        _zeroCal.active = false;
        float avg_we    = _zeroCal.sum_we / _zeroCal.count;
        float avg_ae    = _zeroCal.sum_ae / _zeroCal.count;
        float spread_mv = (_zeroCal.max_wa - _zeroCal.min_wa) * 1000.0f;

        // 无论稳定性如何都保存偏移（spread 仅上报供参考，不再拒存）
        _zeroOffsetsValid = true;
        adsSetOffsets(avg_we, avg_ae);
        _prefs.putFloat("we_off", avg_we);
        _prefs.putFloat("ae_off", avg_ae);
        _prefs.putBool("zero_valid", true);
        float zero_wa_mv = (avg_we - avg_ae) * 1000.0f;  // 差分零点（上报用）
        protocolSendResponse("R", "ZERO_OK, zero_wa=%.2f, n=%u, spread=%.2f",
                            zero_wa_mv, _zeroCal.target_n, spread_mv);
    }
}

// 浓度(ppb) = 2 × ΔV(mV)。ΔV 已在 adsRead 中用校准偏移（WE_OFFSET_V/AE_OFFSET_V）
// 做过零点校正，故零气下 wa≈0 → conc≈0；未校准时偏移为0，即原始公式。

// ============================================================
// 命令处理：ELAPSED（查询）
// ============================================================
inline bool handleQuery_ELAPSED(const char* line) {
    if (!_testState.has_started) {
        protocolSendError(ERR_NOT_STARTED);
        return false;
    }

    protocolSendResponse("R", "ELAPSED, s=%lu", _testState.elapsed_s);
    return true;
}

// ============================================================
// 命令处理：PRESSURE（查询）
// ============================================================
inline bool handleQuery_PRESSURE(const char* line) {
    // 模拟气压读取（实际应用中需要连接气压传感器）
    float pressure = 101.3f;  // 标准大气压 kPa
    protocolSendResponse("R", "PRESSURE, kPa=%.1f", pressure);
    return true;
}

// ============================================================
// 命令处理：GAS_TYPE（查询/设置）
// ============================================================
inline bool handleQuery_GAS_TYPE(const char* line) {
    protocolSendResponse("R", "GAS_TYPE, type=%s", _testState.gas_type);
    return true;
}

// ============================================================
// 命令处理：LAST（查询最新数据）
// ============================================================
inline bool handleQuery_LAST(const char* line) {
    // 获取最新传感器数据
    B43FReading gasReading = adsRead();
    FS4001Reading flowReading = flowSensorRead();

    if (!gasReading.valid) {
        protocolSendError(ERR_NOT_READY);
        return false;
    }

    // 使用EMA滤波后的signal作为wa值
    float we_mv = gasReading.we_v * 1000.0f;
    float ae_mv = gasReading.ae_v * 1000.0f;
    float wa_mv = gasReading.signal_ema * 1000.0f;  // 使用EMA滤波值（已含偏移校正）
    float conc = 2.0f * wa_mv;  // 浓度(ppb) = 2 × ΔV(mV)

    protocolSendResponse("R", "LAST, we=%.1f, ae=%.1f, wa=%.1f, conc=%.2f, unit=ppb",
                        we_mv, ae_mv, wa_mv, conc);
    return true;
}

// ============================================================
// 主命令处理器
// ============================================================
inline bool protocolHandleCommand(const char* line) {
    // 跳过空行
    if (strlen(line) == 0) return false;

    char msgType = line[0];
    const char* cmdId = line + 2;  // 跳过空格

#if 0  // 调试：显示解析信息（默认关闭）
    Serial.printf("[HOST] Parsed: msgType='%c', cmdId='%s'\n", msgType, cmdId);
#endif

    // 处理命令（A=Action, Q=Query）
    if (msgType != MSG_TYPE_ACTION && msgType != MSG_TYPE_QUERY) {
#if 0  // 调试：显示无效消息类型
        Serial.printf("[HOST] Invalid message type: %c\n", msgType);
#endif
        return false;  // 不是协议命令，可能是调试命令
    }

    // START 命令
    if (strncmp(cmdId, "START", 5) == 0) {
        return handleCmd_START(line);
    }
    // STOP 命令
    else if (strncmp(cmdId, "STOP", 4) == 0) {
        return handleCmd_STOP(line);
    }
    // ELAPSED 查询
    else if (strncmp(cmdId, "ELAPSED", 7) == 0) {
        return handleQuery_ELAPSED(line);
    }
    // PRESSURE 查询
    else if (strncmp(cmdId, "PRESSURE", 8) == 0) {
        return handleQuery_PRESSURE(line);
    }
    // GAS_TYPE 查询
    else if (strncmp(cmdId, "GAS_TYPE", 8) == 0) {
        return handleQuery_GAS_TYPE(line);
    }
    // LAST 查询
    else if (strncmp(cmdId, "LAST", 4) == 0) {
        return handleQuery_LAST(line);
    }
    // ZERO 零点校准（开始/清除）
    else if (strncmp(cmdId, "ZERO", 4) == 0) {
        return handleCmd_ZERO(line);
    }
    // BASELINE 查询零点
    else if (strncmp(cmdId, "BASELINE", 8) == 0) {
        return handleQuery_BASELINE(line);
    }
    // 未知命令
    else {
#if 0  // 调试：显示未知命令
        Serial.printf("[HOST] Unknown command: %s\n", cmdId);
#endif
        protocolSendError(ERR_INVALID_CMD);
        return false;
    }
}

// ============================================================
// 协议更新（周期调用）
// ============================================================
inline void protocolUpdate(const B43FReading& gasReading, const FS4001Reading& flowReading) {
    unsigned long now = millis();

    // 更新测试时间
    if (_testState.is_running) {
        _testState.elapsed_s = (now - _testState.start_time_ms) / 1000;

        // 累积 WE-AE EMA滤波值
        if (gasReading.valid) {
            float wa = gasReading.signal_ema;  // 使用EMA滤波值
            _testState.total_wa += wa;
            _testState.sample_count++;
            _testState.last_wa_v = wa;  // 记录最近一次采样值
        }

        // 自动超时停止：到达下行的测试时长 t 后自动结束测试并回复 STOP_OK
        if (_testState.test_duration_s > 0 &&
            _testState.elapsed_s >= _testState.test_duration_s) {
            handleCmd_STOP("A STOP");
        }
    }

    // TICK 上报（每秒）
    if (_testState.is_running && (now - _lastTickMs >= PROTOCOL_TICK_INTERVAL_MS)) {
        _lastTickMs = now;
        protocolSendResponse("R", "TICK, s=%lu, gas=%s",
                            _testState.elapsed_s, _testState.gas_type);
    }

    // 实时数据自动上传（每0.5~1秒）
    if (_testState.is_running && (now - _lastAutoUploadMs >= PROTOCOL_AUTO_UPLOAD_INTERVAL_MS)) {
        _lastAutoUploadMs = now;

        if (gasReading.valid) {
            float we_mv = gasReading.we_v * 1000.0f;
            float ae_mv = gasReading.ae_v * 1000.0f;
            // 使用EMA滤波后的signal作为wa值（调试模式的signal_ema）
            float wa_mv = gasReading.signal_ema * 1000.0f;
            float conc = 2.0f * wa_mv;  // 浓度(ppb) = 2 × ΔV(mV)（已含偏移校正）

            protocolSendResponse("R", "WE, we=%.1f, ae=%.1f, wa=%.1f, conc=%.2f, unit=ppb",
                                we_mv, ae_mv, wa_mv, conc);
        }
    }
}

// ============================================================
// 协议处理（在loop中调用）
// ============================================================
inline void protocolProcess() {
    static char lineBuffer[PROTOCOL_MAX_LINE_LENGTH];
    static size_t linePos = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();

#if 1  // 调试：显示接收到的字符
        Serial.printf("[HOST] Char: 0x%02X '%c'\n", (unsigned char)c, isprint(c) ? c : '.');
#endif

        // 处理回车或换行
        if (c == '\r' || c == '\n') {
            if (linePos > 0) {
                lineBuffer[linePos] = '\0';

#if 1  // 调试：显示接收到的命令（临时启用诊断问题）
                Serial.printf("[HOST] Received: '%s' (len=%d)\n", lineBuffer, strlen(lineBuffer));
#endif

                // 处理系统命令（所有模式都有效）
                if (strcmp(lineBuffer, "test") == 0) {
                    Serial.println("R OK, message_received");
                    Serial.printf("[HOST] Test successful! Received: '%s'\n", lineBuffer);
                }
                else if (strcmp(lineBuffer, "help") == 0 || strcmp(lineBuffer, "?") == 0) {
                    Serial.println(F("\n========== Available Commands =========="));
                    Serial.println(F("Mode Control:"));
                    Serial.println(F("  mode debug        - Switch to debug mode"));
                    Serial.println(F("  mode host         - Switch to host protocol mode"));
                    Serial.println(F("\nHost Protocol Mode Commands:"));
                    Serial.println(F("  A START, t=0, gas=NO, flow=200  - Start test"));
                    Serial.println(F("  A STOP                      - Stop test"));
                    Serial.println(F("  Q ELAPSED                   - Query elapsed time"));
                    Serial.println(F("  Q PRESSURE                  - Query pressure"));
                    Serial.println(F("  Q LAST                      - Query latest data"));
                    Serial.println(F("\nDebug Mode Commands:"));
                    Serial.println(F("  pump set <duty>             - Set pump duty"));
                    Serial.println(F("  valve on/off                - Control valve"));
                    Serial.println(F("  flow set <sccm>             - Set target flow"));
                    Serial.println(F("======================================"));
                }
                // 模式切换命令
                else if (strcmp(lineBuffer, "mode debug") == 0) {
                    protocolSetMode(PROTOCOL_MODE_DEBUG);
                    Serial.println(F("\n[INFO] 已切换到调试模式"));
                }
                else if (strcmp(lineBuffer, "mode host") == 0) {
                    protocolSetMode(PROTOCOL_MODE_HOST);
                    Serial.println(F("\n[INFO] 已切换到主机协议模式"));
                }
                // 处理协议命令
                else if (!protocolHandleCommand(lineBuffer)) {
                    // 不是协议命令，发送错误响应
#if 0  // 调试：显示未知命令
                    Serial.printf("[HOST] Unknown command: %s\n", lineBuffer);
#endif
                    protocolSendError(ERR_INVALID_CMD);
                }

                linePos = 0;
                lineBuffer[0] = '\0';
            }
            break;
        }
        else if (linePos < PROTOCOL_MAX_LINE_LENGTH - 1) {
            lineBuffer[linePos++] = c;
        }
    }
}