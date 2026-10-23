#pragma once
// ============================================================
// scd30.h  –  Sensirion SCD30 CO2 传感器驱动 (I2C 0x61)
// 与 ADS1115(0x48)/XL9555(0x20) 共用 I2C 总线（SDA=41, SCL=42）
//
// 要点（SCD30_Interface_Description.pdf）：
//   - I2C 地址 0x61；上电启动时间 < 2s
//   - 最大 100kHz；支持时钟拉伸（ESP32 硬件 I2C 支持）
//   - 命令为16位，数据带 CRC-8（poly 0x31, init 0xFF）
//   - 读测量值：CO2/温度/湿度 各4字节 float（大端 IEEE754），
//     每4字节 float 拆成两段 2字节，各带1字节 CRC
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#define SCD30_I2C_ADDR       0x61
#define SCD30_CMD_START      0x0010   // 触发连续测量（参数=压力补偿mBar，0=禁用）
#define SCD30_CMD_READY      0x0202   // 数据就绪查询（1=可读）
#define SCD30_CMD_READ_MEAS  0x0300   // 读测量值（CO2/T/RH）

static bool  _scd30Ok   = false;
static float _scd30Co2  = 0.0f;
static float _scd30Temp = 0.0f;
static float _scd30Rh   = 0.0f;

// CRC-8: poly 0x31, init 0xFF, no reflect, xorout 0x00
static uint8_t scd30Crc(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// 写16位命令（无参数）
static bool scd30WriteCmd(uint16_t cmd) {
    Wire.beginTransmission(SCD30_I2C_ADDR);
    Wire.write(cmd >> 8);
    Wire.write(cmd & 0xFF);
    return (Wire.endTransmission() == 0);
}

// 写16位命令 + 2字节参数 + CRC
static bool scd30WriteCmdArg(uint16_t cmd, uint16_t arg) {
    uint8_t d[5];
    d[0] = cmd >> 8; d[1] = cmd & 0xFF;
    d[2] = arg >> 8; d[3] = arg & 0xFF;
    d[4] = scd30Crc(&d[2], 2);
    Wire.beginTransmission(SCD30_I2C_ADDR);
    Wire.write(d, 5);
    return (Wire.endTransmission() == 0);
}

// 初始化：触发连续测量（无压力补偿），带重试（上电需 <2s）
inline bool scd30Init() {
    for (int i = 0; i < 6; i++) {
        if (scd30WriteCmdArg(SCD30_CMD_START, 0x0000)) {
            _scd30Ok = true;
            Serial.println(F("[SCD30] Continuous measurement started (0x0010)"));
            return true;
        }
        delay(500);
    }
    Serial.println(F("[SCD30] ERROR: no response at 0x61"));
    return false;
}

// 读测量值：CO2(ppm)/温度(°C)/湿度(%RH)，成功返回 true
inline bool scd30Read(float* co2, float* temp, float* rh) {
    if (!_scd30Ok) return false;

    // 1. 数据就绪查询（写 0x0202，读 2字节状态 + CRC）
    if (!scd30WriteCmd(SCD30_CMD_READY)) return false;
    Wire.requestFrom((uint8_t)SCD30_I2C_ADDR, (uint8_t)3);
    uint8_t r[3];
    if (Wire.available() < 3) return false;
    for (int i = 0; i < 3; i++) r[i] = Wire.read();
    if (scd30Crc(r, 2) != r[2]) return false;
    if (!(r[0] == 0x00 && r[1] == 0x01)) return false;   // 数据未就绪

    // 2. 读测量值（18字节 = 6组：每组 2数据字节 + 1 CRC）
    if (!scd30WriteCmd(SCD30_CMD_READ_MEAS)) return false;
    Wire.requestFrom((uint8_t)SCD30_I2C_ADDR, (uint8_t)18);
    uint8_t b[18];
    if (Wire.available() < 18) return false;
    for (int i = 0; i < 18; i++) b[i] = Wire.read();

    // 校验每组 CRC
    for (int g = 0; g < 6; g++) {
        if (scd30Crc(&b[g * 3], 2) != b[g * 3 + 2]) {
            Serial.printf("[SCD30] CRC fail group %d\n", g);
            return false;
        }
    }

    // 组合 4 字节大端 float（每组2字节，float 拆两段）
    auto combine = [&](int i0, int i1, int i2, int i3) {
        uint32_t u = ((uint32_t)b[i0] << 24) | ((uint32_t)b[i1] << 16)
                   | ((uint32_t)b[i2] << 8)  | ((uint32_t)b[i3]);
        float f;
        memcpy(&f, &u, 4);
        return f;
    };

    _scd30Co2  = combine(0, 1, 3, 4);     // CO2 ppm
    _scd30Temp = combine(6, 7, 9, 10);    // 温度 °C
    _scd30Rh   = combine(12, 13, 15, 16); // 湿度 %RH

    if (co2)  *co2  = _scd30Co2;
    if (temp) *temp = _scd30Temp;
    if (rh)   *rh   = _scd30Rh;
    return true;
}

inline bool  scd30IsReady() { return _scd30Ok; }
inline float scd30GetCo2()  { return _scd30Co2; }
inline float scd30GetTemp() { return _scd30Temp; }
inline float scd30GetRh()   { return _scd30Rh; }
