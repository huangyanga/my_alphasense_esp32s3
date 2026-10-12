#pragma once
// ============================================================
// xl9555.h  –  XL9555 IO扩展器驱动
// ============================================================
// 正点原子 DNESP32S3 开发板上的 IO 扩展器 (I2C 地址 0x20)，
// 用于控制 LCD 电源/复位/背光、蜂鸣器、喇叭等。
//
// XL9555 挂在 I2C 总线 (SDA=41, SCL=42) 上，
// 与 ADS1115 (0x48) 共用总线，地址不冲突。
// 当前仅实现 LCD 所需功能（电源、复位、背光）。
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ------------------------------------------------------------
// XL9555 I2C 地址
// ------------------------------------------------------------
#define XL9555_ADDR                 0x20

// ------------------------------------------------------------
// XL9555 寄存器
// ------------------------------------------------------------
#define XL9555_INPUT_PORT0_REG      0   // 输入寄存器0
#define XL9555_INPUT_PORT1_REG      1   // 输入寄存器1
#define XL9555_OUTPUT_PORT0_REG     2   // 输出寄存器0
#define XL9555_OUTPUT_PORT1_REG     3   // 输出寄存器1
#define XL9555_INVERSION_PORT0_REG  4   // 极性反转寄存器0
#define XL9555_INVERSION_PORT1_REG  5   // 极性反转寄存器1
#define XL9555_CONFIG_PORT0_REG     6   // 方向配置寄存器0 (0=输出, 1=输入)
#define XL9555_CONFIG_PORT1_REG     7   // 方向配置寄存器1

// ------------------------------------------------------------
// XL9555 引脚定义（16位：bit0-7 = P0，bit8-15 = P1）
// ------------------------------------------------------------
#define SPK_EN_IO       0x0004   // P0.2 喇叭使能
#define BEEP_IO         0x0008   // P0.3 蜂鸣器
#define LCD_BL_IO       0x0100   // P1.0 LCD 背光
#define CT_RST_IO       0x0200   // P1.1 触摸屏复位
#define SLCD_RST_IO     0x0400   // P1.2 LCD 复位
#define SLCD_PWR_IO     0x0800   // P1.3 LCD 电源

// ------------------------------------------------------------
// 底层 I2C 读写
// ------------------------------------------------------------
inline bool xl9555_write_byte(uint8_t reg, const uint8_t* data, uint8_t len) {
    Wire.beginTransmission(XL9555_ADDR);
    Wire.write(reg);
    for (uint8_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return (Wire.endTransmission() == 0);
}

inline bool xl9555_read_byte(uint8_t* data, uint8_t len) {
    Wire.beginTransmission(XL9555_ADDR);
    Wire.write(XL9555_INPUT_PORT0_REG);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)XL9555_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        if (Wire.available()) data[i] = Wire.read();
        else return false;
    }
    return true;
}

// ------------------------------------------------------------
// 控制某个 IO 的电平（读-改-写）
// ------------------------------------------------------------
inline void xl9555_pin_write(uint16_t pin, int val) {
    uint8_t w_data[2];
    if (!xl9555_read_byte(w_data, 2)) return;

    if (pin <= 0x0080) {
        if (val) w_data[0] |= (uint8_t)(0xFF & pin);
        else     w_data[0] &= ~(uint8_t)(0xFF & pin);
    } else {
        if (val) w_data[1] |= (uint8_t)(0xFF & (pin >> 8));
        else     w_data[1] &= ~(uint8_t)(0xFF & (pin >> 8));
    }
    xl9555_write_byte(XL9555_OUTPUT_PORT0_REG, w_data, 2);
}

// ------------------------------------------------------------
// IO 方向配置（0=输出, 1=输入）
// ------------------------------------------------------------
inline void xl9555_ioconfig(uint16_t config_value) {
    uint8_t data[2];
    data[0] = (uint8_t)(0xFF & config_value);
    data[1] = (uint8_t)(0xFF & (config_value >> 8));
    xl9555_write_byte(XL9555_CONFIG_PORT0_REG, data, 2);
}

// ------------------------------------------------------------
// 初始化 XL9555
// 与正点原子例程一致：
//   - 方向配置 0xF003（P0.0/P0.1 输入=中断，P1.4-P1.7 输入=按键，其余输出）
//   - 关闭蜂鸣器和喇叭
// ------------------------------------------------------------
inline bool xl9555_init() {
    // 确保 I2C 已初始化（幂等）
    Wire.begin(SDA_PIN, SCL_PIN);

    // 上电先读取一次清除中断标志
    uint8_t r_data[2];
    xl9555_read_byte(r_data, 2);

    // 配置 IO 方向
    xl9555_ioconfig(0xF003);

    // 关闭蜂鸣器、喇叭
    xl9555_pin_write(BEEP_IO, 1);
    xl9555_pin_write(SPK_EN_IO, 1);

    return true;
}
