#pragma once
// ============================================================
// alphasense_b43f.h  –  Alphasense B43F via ADS1115 (I2C)
// ============================================================
// Provides:
//   B43FReading struct   : WE/AE raw counts, calibrated
//                          voltages, signal=WE-AE, EMA values
//   adsInit()            : initialise Wire + ADS1115
//   adsRead()            : acquire one burst, return reading
//   convertToConcentration() : stub (TODO – needs cal data)
//
// Configuration is pulled from config.h.
// Hardware: ADS1115 at I2C address ADS1115_I2C_ADDRESS,
//           WE → AIN0 (ADS_CH_WE), AE → AIN1 (ADS_CH_AE).
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "config.h"

// ------------------------------------------------------------
// Measurement result
// ------------------------------------------------------------
struct B43FReading {
    int16_t we_raw;       // WE raw ADS1115 count
    int16_t ae_raw;       // AE raw ADS1115 count
    float   we_raw_v;     // WE raw voltage (V, 未校正，供零点校准)
    float   ae_raw_v;     // AE raw voltage (V, 未校正，供零点校准)
    float   we_v;         // WE calibrated voltage (V)
    float   ae_v;         // AE calibrated voltage (V)
    float   signal_v;     // Compensated signal = WE - AE (V)
    float   we_ema;       // WE EMA-filtered voltage (V)
    float   ae_ema;       // AE EMA-filtered voltage (V)
    float   signal_ema;   // EMA-filtered signal (V)
    bool    valid;        // false when I2C communication failed
};

// ------------------------------------------------------------
// Module-private state
// ------------------------------------------------------------
static Adafruit_ADS1115 _ads;
static bool    _adsOk    = false;
static float   _weEma    = 0.0f;
static float   _aeEma    = 0.0f;
static bool    _emaInit  = false;
static uint8_t _errCount = 0;

// 运行时通道偏移（默认取 config.h 宏，零点校准 A ZERO 后覆盖并持久化到 NVS）
static float _weOffsetV = WE_OFFSET_V;
static float _aeOffsetV = AE_OFFSET_V;

// 设置双通道偏移（零点校准调用）
inline void adsSetOffsets(float weOff, float aeOff) {
    _weOffsetV = weOff;
    _aeOffsetV = aeOff;
    Serial.printf("[ADS1115] Offsets set: WE=%.4f V, AE=%.4f V\n", weOff, aeOff);
}

// 清除偏移（恢复 config.h 默认值）
inline void adsResetOffsets() {
    _weOffsetV = WE_OFFSET_V;
    _aeOffsetV = AE_OFFSET_V;
    Serial.println(F("[ADS1115] Offsets reset to defaults"));
}

inline float adsGetWeOffsetV() { return _weOffsetV; }
inline float adsGetAeOffsetV() { return _aeOffsetV; }

// ------------------------------------------------------------
// adsInit()
// Call once from setup().  Starts Wire on SDA_PIN/SCL_PIN,
// sets PGA gain and data rate, and checks for the ADS1115.
// Returns true if the device was found on the I2C bus.
// ------------------------------------------------------------
inline bool adsInit() {
    Wire.begin(SDA_PIN, SCL_PIN);
    _ads.setGain(ADS1115_GAIN);
    _ads.setDataRate(ADS1115_SPS);
    _adsOk = _ads.begin(ADS1115_I2C_ADDRESS, &Wire);
    if (!_adsOk) {
        Serial.printf("[ADS1115] ERROR: not found at 0x%02X "
                      "(SDA=GPIO%d, SCL=GPIO%d)\n",
                      ADS1115_I2C_ADDRESS, SDA_PIN, SCL_PIN);
    }
    return _adsOk;
}

// ------------------------------------------------------------
// adsRead()
// Reads WE and AE channels, applies calibration and EMA.
// If the ADS1115 is not responding the reading is marked
// invalid; a re-init attempt is made automatically after
// I2C_MAX_RETRIES consecutive failures.
// ------------------------------------------------------------
inline B43FReading adsRead() {
    B43FReading r = {};
    r.valid = false;

    // Re-init after repeated failures (non-blocking)
    if (!_adsOk) {
        _errCount++;
        if (_errCount >= I2C_MAX_RETRIES) {
            _errCount = 0;
            Serial.println(F("[ADS1115] Retrying init..."));
            _adsOk = _ads.begin(ADS1115_I2C_ADDRESS, &Wire);
        }
        if (!_adsOk) {
            Serial.printf("[ADS1115] Not ready (attempt %u/%u)\n",
                          _errCount, I2C_MAX_RETRIES);
            return r;
        }
    }

#if ADS1115_DIFFERENTIAL_MODE
    // Hardware differential AIN0-AIN1 (single conversion)
    int16_t diff   = _ads.readADC_Differential_0_1();
    float   diff_v = _ads.computeVolts(diff);
    r.we_raw   = diff;
    r.ae_raw   = 0;
    r.we_raw_v = diff_v;
    r.ae_raw_v = 0.0f;
    r.we_v     = diff_v;
    r.ae_v     = 0.0f;
    r.signal_v = diff_v;
#else
    // Single-ended: read A0 (WE) and A1 (AE) separately
    r.we_raw       = _ads.readADC_SingleEnded(ADS_CH_WE);
    r.ae_raw       = _ads.readADC_SingleEnded(ADS_CH_AE);
    float we_raw_v = _ads.computeVolts(r.we_raw);
    float ae_raw_v = _ads.computeVolts(r.ae_raw);
    // 保存原始电压（未校正，供零点校准采样）
    r.we_raw_v = we_raw_v;
    r.ae_raw_v = ae_raw_v;
    // Apply per-channel calibration (offset + gain)，偏移为运行时值（可被 A ZERO 校准）
    r.we_v     = (we_raw_v - _weOffsetV) * WE_GAIN_CAL;
    r.ae_v     = (ae_raw_v - _aeOffsetV) * AE_GAIN_CAL;
    r.signal_v = r.we_v - r.ae_v;
#endif

    // EMA filter
    if (!_emaInit) {
        _weEma  = r.we_v;
        _aeEma  = r.ae_v;
        _emaInit = true;
    } else {
        _weEma = EMA_ALPHA * r.we_v + (1.0f - EMA_ALPHA) * _weEma;
        _aeEma = EMA_ALPHA * r.ae_v + (1.0f - EMA_ALPHA) * _aeEma;
    }
    r.we_ema     = _weEma;
    r.ae_ema     = _aeEma;
    r.signal_ema = _weEma - _aeEma;

    r.valid   = true;
    _errCount = 0;
    return r;
}

// ------------------------------------------------------------
// convertToConcentration()
// TODO: implement once the full analogue signal chain
//       (TIA gain, Alphasense sensitivity nA/ppb, zero offset)
//       has been characterised for your specific B43F unit.
//
// References:
//   Alphasense AAN 803 – B4 series circuit description
//   Alphasense AAN 110 – Individual sensor board (ISB) setup
//
// Steps (to be filled in):
//   1. Remove baseline voltage (zero-air / clean-air offset)
//   2. current_A = signal_v / TIA_GAIN_OHM
//   3. ppb = current_A / SENSITIVITY_A_PER_PPB
//   4. Apply temperature compensation if required
// ------------------------------------------------------------
inline float convertToConcentration(float signal_v, uint8_t channel) {
    (void)signal_v;
    (void)channel;
    return 0.0f;
}
