#pragma once
// ============================================================
// lcd_st7789.h  –  2.4寸 ST7789 SPI LCD 驱动封装
// ============================================================
// 基于 Adafruit_ST7789 + Adafruit_GFX 库。
//
// 引脚（与正点原子 DNESP32S3 例程 10_spilcd 一致）：
//   SCLK = GPIO12, MOSI = GPIO11, MISO = GPIO13
//   CS   = GPIO21, DC   = GPIO40
//   RST / PWR / 背光 通过 XL9555 (I2C 0x20) 控制
// 开机检测 IO10 选择引脚方案：
//   IO10 低电平（非悬空）→ RST=GPIO9, BL=GPIO8（直连）；其余不变
//   IO10 悬空           → RST/BL 保持 XL9555 控制（现有方案）
//
// 屏幕：2.4寸 ST7789，320x240（横屏），RGB565，反显。
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "xl9555.h"
#include "bb_font.h"

// ------------------------------------------------------------
// LCD SPI 引脚
// ------------------------------------------------------------
#define LCD_SCLK_PIN   12
#define LCD_MOSI_PIN   11
#define LCD_MISO_PIN   13
#define LCD_CS_PIN     21
#define LCD_DC_PIN     40

// ------------------------------------------------------------
// 全局对象
// ------------------------------------------------------------
static SPIClass*        _lcdSpi = nullptr;
static Adafruit_ST7789* _tft    = nullptr;

// ------------------------------------------------------------
// LCD 引脚方案开机检测
// 上电检测 IO10（内部上拉）：
//   - 读到低电平（板上存在下拉，非悬空）→ 方案B：RST=GPIO9, BL=GPIO8 直连
//   - 读到高电平（悬空，内部上拉生效）   → 保持 XL9555 控制 RST/BL（现有方案）
// 其余引脚（SCLK/MOSI/MISO/CS/DC/PWR）两种方案完全相同
// ------------------------------------------------------------
#define LCD_PIN_DETECT_IO   10  // 开机检测脚
#define LCD_RST_ALT_PIN     9   // 方案B：LCD 复位
#define LCD_BL_ALT_PIN      8   // 方案B：LCD 背光
#define LCD_PWR_ALT_PIN     (-1) // 方案B：LCD 电源使能脚；-1 = 电源硬连/不控制
                                 // （方案B板无XL9555，若实测发现电源需GPIO使能，改成对应脚号）

// 固件版本号（显示在LCD标题栏右侧）
#define LCD_FW_VERSION      "1.0.00"

static bool _lcdVariantB = false;  // true = IO10 低电平，RST/BL 用直连 GPIO

// 位带渲染前向声明（定义见下方位带渲染区）
inline void bbPinSetup();
inline void bbInitPanel();

// 开机检测 IO10 电平，选择 LCD 引脚方案（在 lcdInit 最前调用）
inline void lcdDetectPinout() {
    pinMode(LCD_PIN_DETECT_IO, INPUT_PULLUP);
    delay(1);
    _lcdVariantB = (digitalRead(LCD_PIN_DETECT_IO) == LOW);
    Serial.printf("[LCD] IO10=%s -> %s\n",
                  _lcdVariantB ? "LOW" : "FLOAT/HIGH",
                  _lcdVariantB ? "RST=GPIO9, BL=GPIO8" : "RST/BL via XL9555");
}

// ------------------------------------------------------------
// 初始化 LCD
// 调用前无需初始化 XL9555（本函数内部完成）
// ------------------------------------------------------------
inline bool lcdInit() {
    // 0. 开机检测 IO10，选择 RST/BL 引脚方案
    lcdDetectPinout();

    // 0b. 方案B：先把 RST(GPIO9) 拉低钳位。
    //     与方案A（XL9555 输出寄存器默认0 → RST 低）保持一致，
    //     屏在上电期间处于复位态，避免上电瞬间 RST 悬空导致异常
    if (_lcdVariantB) {
        pinMode(LCD_RST_ALT_PIN, OUTPUT);
        digitalWrite(LCD_RST_ALT_PIN, LOW);
    }

    // 1. 方案B 无 XL9555（IO10低电平板），跳过 XL9555 路径；
    //    方案A 初始化 XL9555 并探测其在总线上
    if (_lcdVariantB) {
        Serial.println(F("[LCD] VariantB: XL9555 not present, skip"));
    } else {
        xl9555_init();
        uint8_t probe[2];
        bool xlOk = xl9555_read_byte(probe, 2);
        Serial.printf("[LCD] XL9555 on bus: %s\n", xlOk ? "YES" : "NO");
    }

    // 2. LCD 上电：方案A 由 XL9555 P1.3 控制；
    //    方案B 若 LCD_PWR_ALT_PIN>=0 则用该 GPIO 使能，否则视为硬连
    if (_lcdVariantB) {
        if (LCD_PWR_ALT_PIN >= 0) {
            pinMode(LCD_PWR_ALT_PIN, OUTPUT);
            digitalWrite(LCD_PWR_ALT_PIN, HIGH);
        }
    } else {
        xl9555_pin_write(SLCD_PWR_IO, 1);
    }
    delay(50);

    // 3. 打开背光（方案B用GPIO8直连，否则走XL9555）
    if (_lcdVariantB) {
        pinMode(LCD_BL_ALT_PIN, OUTPUT);
        digitalWrite(LCD_BL_ALT_PIN, HIGH);
    } else {
        xl9555_pin_write(LCD_BL_IO, 1);
    }

    // 4. 初始化 SPI 总线：方案A 用硬件 SPI；
    //    方案B 不用（位带直接驱动GPIO，避免SPI外设抢占CS/SCLK/MOSI引脚）
    _lcdSpi = new SPIClass(FSPI);
    if (!_lcdVariantB) {
        _lcdSpi->begin(LCD_SCLK_PIN, LCD_MISO_PIN, LCD_MOSI_PIN, LCD_CS_PIN);
    }

    // 5. 创建 ST7789 对象（RST 传 -1；方案B 仅作非空标志，绘图全部走位带）
    _tft = new Adafruit_ST7789(_lcdSpi, LCD_CS_PIN, LCD_DC_PIN, -1);

    // 6. 释放复位（方案B：GPIO9 已在步骤0b拉低，现在释放；方案A：走XL9555）
    if (_lcdVariantB) {
        digitalWrite(LCD_RST_ALT_PIN, HIGH);
        delay(120);
    } else {
        xl9555_pin_write(SLCD_RST_IO, 0);
        delay(50);
        xl9555_pin_write(SLCD_RST_IO, 1);
        delay(120);
    }

    // 7. 初始化面板：
    //    方案B 用位带（新屏仅响应逐字节CS选通，Adafruit 整事务序列被忽略）；
    //    方案A 用 Adafruit 标准驱动 + 反显
    if (_lcdVariantB) {
        bbPinSetup();   // SPI 引脚转 GPIO（位带模式）
        bbInitPanel();  // ST7789 完整初始化序列（含横屏 MADCTL=0x60）
    } else {
        _tft->init(240, 320);
        _tft->invertDisplay(true);
        _tft->setRotation(3);
    }

    // 9. 清屏（方案A 用 Adafruit；方案B 位带，由 lcdCurveInit 清屏，
    //    此处不可调用 _tft（方案B 未初始化 SPI 总线，会空指针崩溃））
    if (!_lcdVariantB) {
        _tft->fillScreen(ST77XX_BLACK);
    }

    // 10. 诊断：打印方案B引脚最终状态
    if (_lcdVariantB) {
        Serial.printf("[LCD] VariantB final: RST(GPIO9)=%d, BL(GPIO8)=%d\n",
                      digitalRead(LCD_RST_ALT_PIN), digitalRead(LCD_BL_ALT_PIN));
    }

    return true;
}

// ------------------------------------------------------------
// 在屏幕中央显示一行文本（大字）
// ------------------------------------------------------------
inline void lcdShowText(const char* text, uint16_t color = ST77XX_WHITE) {
    if (!_tft) return;
    _tft->fillScreen(ST77XX_BLACK);
    _tft->setTextColor(color);
    _tft->setTextSize(3);
    _tft->setCursor(85, 105);
    _tft->print(text);
}

// ------------------------------------------------------------
// 清屏
// ------------------------------------------------------------
inline void lcdClear(uint16_t color = ST77XX_BLACK) {
    if (!_tft) return;
    _tft->fillScreen(color);
}

// ------------------------------------------------------------
// 获取 LCD 对象指针（供外部直接使用 GFX API）
// ------------------------------------------------------------
inline Adafruit_ST7789* lcdGetTft() {
    return _tft;
}

// ============================================================
// 单次测试曲线显示（浓度 ppm，纵轴固定 -5~+5 ppm）
// 布局（320x240 横屏）：
//   标题区      : y  0 ~ 24
//   曲线区域    : y 25 ~ 190（本次测试全部数据点，按点数均匀分布）
//   实时信息区  : y 195 ~ 235
// 行为：
//   - 测试开始(lcdCurveReset)清空缓存，测试期间实时添加数据点
//   - 横轴按"本次测试实际数据点数"均匀缩放，不固定窗口
//   - 最新值实时显示在标题栏右侧（曲线区域之外，不遮挡曲线）
//   - 测试结束(lcdCurveEnd)在底部信息区大字号显示平均值(avg)与最后值(last)
// ============================================================
#define LCD_CURVE_POINTS   600          // 单次测试最大缓存点数（1点/秒≈10分钟，超出丢最旧）
#define LCD_CURVE_X0       30           // 曲线区域左边界（左侧留Y轴刻度）
#define LCD_CURVE_X1       314          // 曲线区域右边界
#define LCD_CURVE_Y0       25           // 曲线区域上边界
#define LCD_CURVE_Y1       190          // 曲线区域下边界
#define LCD_CURVE_PPM_MIN  (-5.0f)      // 纵轴最小值 (ppm)
#define LCD_CURVE_PPM_MAX  ( 5.0f)      // 纵轴最大值 (ppm)

static float    _curveBuf[LCD_CURVE_POINTS];
static uint16_t _curveCount = 0;        // 本次测试已有点数

// ------------------------------------------------------------
// 渲染分派层前向声明（定义见下方位带渲染区）
// 方案B（IO10低电平）→ 位带渲染；方案A → Adafruit
// ------------------------------------------------------------
inline void lcdFillScreen(uint16_t c);
inline void lcdFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
inline void lcdDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
inline void lcdDrawHLine(int16_t x, int16_t y, int16_t w, uint16_t c);
inline void lcdDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t c);
inline void lcdFillCircle(int16_t x, int16_t y, int16_t r, uint16_t c);
inline void lcdPrintText(int16_t x, int16_t y, const char* s, uint16_t c, uint8_t size);

// 值 -> Y 坐标（PPM_MIN~PPM_MAX 映射到 Y1~Y0，支持负值）
inline int lcdCurveY(float v) {
    float ratio = (v - LCD_CURVE_PPM_MIN) / (LCD_CURVE_PPM_MAX - LCD_CURVE_PPM_MIN);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return LCD_CURVE_Y1 - (int)(ratio * (LCD_CURVE_Y1 - LCD_CURVE_Y0));
}

// 绘制 Y 轴刻度与网格线（-5 / -2.5 / 0 / 2.5 / 5 ppm）
inline void lcdCurveDrawScale() {
    if (!_tft) return;
    const char* labels[5] = {"-5", "-2.5", "0", "2.5", "5"};
    for (int g = 0; g < 5; g++) {
        int y = LCD_CURVE_Y1 - g * (LCD_CURVE_Y1 - LCD_CURVE_Y0) / 4;
        // 网格线（0 线用白色醒目，其余用蓝色）
        uint16_t gridColor = (g == 2) ? ST77XX_WHITE : ST77XX_BLUE;
        if (g > 0 && g < 4) {
            lcdDrawHLine(LCD_CURVE_X0 + 1, y, LCD_CURVE_X1 - LCD_CURVE_X0 - 1, gridColor);
        }
        // 刻度标签（右对齐到曲线区左缘）
        int w = strlen(labels[g]) * 6;
        lcdPrintText(LCD_CURVE_X0 - 3 - w, y - 4, labels[g], ST77XX_WHITE, 1);
    }
}

// 在标题栏右侧显示最新值（ppb，大字号，位于曲线区域之外不遮挡曲线）
inline void lcdCurveDrawLatest(float lastPpb) {
    if (!_tft || _curveCount == 0) return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%dppb", (int)lastPpb);
    int w = strlen(buf) * 12 + 4;   // 字号2: 12px/字符
    int h = 16;
    int tx = 283 - w;               // 放在 "ppm" 标签(285,5)左侧
    int ty = 3;

    lcdFillRect(tx, ty, w, h, ST77XX_BLACK);
    lcdDrawRect(tx, ty, w, h, ST77XX_YELLOW);
    lcdPrintText(tx + 2, ty + 1, buf, ST77XX_YELLOW, 2);
}

// 重绘曲线区域（清屏 + 刻度网格 + 本次测试折线）
// 横轴：本次测试的数据点均匀分布在 X0~X1（按实际点数缩放）
inline void lcdCurveRedraw() {
    if (!_tft) return;

    // 清空曲线区域
    lcdFillRect(LCD_CURVE_X0, LCD_CURVE_Y0,
                LCD_CURVE_X1 - LCD_CURVE_X0 + 1,
                LCD_CURVE_Y1 - LCD_CURVE_Y0 + 1, ST77XX_BLACK);
    // 边框
    lcdDrawRect(LCD_CURVE_X0, LCD_CURVE_Y0,
                LCD_CURVE_X1 - LCD_CURVE_X0 + 1,
                LCD_CURVE_Y1 - LCD_CURVE_Y0 + 1, ST77XX_WHITE);
    // Y轴刻度与网格
    lcdCurveDrawScale();

    if (_curveCount == 0) return;
    if (_curveCount == 1) {  // 单点画圆点
        lcdFillCircle(LCD_CURVE_X0, lcdCurveY(_curveBuf[0]), 2, ST77XX_GREEN);
        return;
    }

    int span = LCD_CURVE_X1 - LCD_CURVE_X0;
    for (uint16_t i = 1; i < _curveCount; i++) {
        int x1 = LCD_CURVE_X0 + (int)((long)(i - 1) * span / (_curveCount - 1));
        int x2 = LCD_CURVE_X0 + (int)((long)i * span / (_curveCount - 1));
        int y1 = lcdCurveY(_curveBuf[i - 1]);
        int y2 = lcdCurveY(_curveBuf[i]);
        lcdDrawLine(x1, y1, x2, y2, ST77XX_GREEN);
    }
}

// 添加一个数据点（值单位: ppm）并重绘（测试运行中实时调用）
inline void lcdShowInfo(float wa_mv, float ppb, float flow, float duty);  // 前向声明（定义见下方）

inline void lcdCurveAdd(float valuePpm) {
    if (_curveCount >= LCD_CURVE_POINTS) {
        // 超出缓存：丢弃最旧点，保留最近 LCD_CURVE_POINTS 个
        memmove(_curveBuf, _curveBuf + 1, (LCD_CURVE_POINTS - 1) * sizeof(float));
        _curveCount = LCD_CURVE_POINTS - 1;
    }
    _curveBuf[_curveCount++] = valuePpm;
    lcdCurveRedraw();
    lcdCurveDrawLatest(valuePpm * 1000.0f);  // 实时标注最后值(ppb)
}

// 测试结束：重绘最终曲线（不加标注），底部信息区原有两行保留，
// 在其下方追加一行大字号最终结果 avg/last（不刷掉 wa/ppb/flow/pump 文字）
inline void lcdCurveEnd(float avgPpb, float lastPpb) {
    if (!_tft) return;
    lcdCurveRedraw();
    lcdCurveDrawLatest(lastPpb);   // 标题栏右侧显示最后值，不遮挡曲线

    // 第三行（前两行下方）：最终结果，大字号黄色
    int y3 = LCD_CURVE_Y1 + 8 + 22;
    char buf[32];
    snprintf(buf, sizeof(buf), "avg=%5.0fppb last=%5.0fppb", avgPpb, lastPpb);
    lcdFillRect(0, y3 - 2, 320, 240 - y3 + 2, ST77XX_BLACK);
    lcdPrintText(5, y3, buf, ST77XX_YELLOW, 2);
}

// 测试开始：清空曲线缓存并重绘框架
inline void lcdCurveReset() {
    _curveCount = 0;
    lcdCurveRedraw();
    lcdShowInfo(0.0f, 0.0f, 0.0f, 0.0f);
}

// 底部实时信息区（wa=信号mV, ppb=浓度, flow=流量sccm, duty=泵占空比%）
inline void lcdShowInfo(float wa_mv, float ppb, float flow, float duty) {
    if (!_tft) return;
    int infoY = LCD_CURVE_Y1 + 8;
    lcdFillRect(0, infoY - 2, 320, 240 - infoY + 2, ST77XX_BLACK);
    char b1[32], b2[32];
    snprintf(b1, sizeof(b1), "wa=%7.1fmV  ppb=%7.1f", wa_mv, ppb);
    snprintf(b2, sizeof(b2), "flow=%6.1fsccm pump=%5.1f%%", flow, duty);
    lcdPrintText(5, infoY, b1, ST77XX_WHITE, 1);
    lcdPrintText(5, infoY + 12, b2, ST77XX_WHITE, 1);
}

// 初始化曲线界面（清屏 + 标题 + 版本号 + 空曲线框架）
inline void lcdCurveInit(const char* title) {
    if (!_tft) return;
    lcdFillScreen(ST77XX_BLACK);
    lcdPrintText(5, 5, title, ST77XX_WHITE, 1);
    lcdPrintText(70, 5, LCD_FW_VERSION, ST77XX_GREEN, 1);  // 版本号（标题右侧）
    lcdPrintText(285, 5, "ppm", ST77XX_WHITE, 1);
    lcdCurveReset();
}

// 标题栏显示 CO2 浓度（ppm，位于版本号右侧，不遮挡右上最新值标签）
inline void lcdShowCo2(float ppm) {
    if (!_tft) return;
    char buf[20];
    snprintf(buf, sizeof(buf), "CO2 %dppm", (int)ppm);
    int w = strlen(buf) * 6 + 2;
    lcdFillRect(115, 3, w, 10, ST77XX_BLACK);   // 清背景重绘
    lcdPrintText(115, 5, buf, ST77XX_WHITE, 1);
}

// ------------------------------------------------------------
// 方案B 软件SPI（位带）—— 复刻 STM32 参考代码协议
// 关键差异：每字节 CS/WR 选通（字节前CS低、字节后CS高），
// 而 Adafruit 是整事务保持CS低。该模块只有逐字节选通才响应。
// 模式0（SCK空闲低、上升沿锁存）、MSB先行，与参考代码一致。
// ------------------------------------------------------------
inline void bbPinSetup();   // 前向声明（定义见下方）
inline void bbInitPanel();  // 前向声明（定义见下方）

inline void bbPinSetup() {
    pinMode(LCD_SCLK_PIN, OUTPUT);
    pinMode(LCD_MOSI_PIN, OUTPUT);
    pinMode(LCD_CS_PIN, OUTPUT);
    pinMode(LCD_DC_PIN, OUTPUT);
    digitalWrite(LCD_SCLK_PIN, 0);
    digitalWrite(LCD_MOSI_PIN, 0);
    digitalWrite(LCD_CS_PIN, 1);
    digitalWrite(LCD_DC_PIN, 1);
}

inline void bbByte(uint8_t byte) {
    digitalWrite(LCD_CS_PIN, 0);            // WR/CS 低（字节选通开始）
    for (int i = 0; i < 8; i++) {
        digitalWrite(LCD_SCLK_PIN, 0);
        digitalWrite(LCD_MOSI_PIN, (byte & 0x80) ? 1 : 0);
        byte <<= 1;
        digitalWrite(LCD_SCLK_PIN, 1);      // 上升沿锁存
    }
    digitalWrite(LCD_CS_PIN, 1);            // WR/CS 高（字节选通结束）
    digitalWrite(LCD_SCLK_PIN, 0);
}

inline void bbCmd(uint8_t c) { digitalWrite(LCD_DC_PIN, 0); bbByte(c); }
inline void bbData(uint8_t d) { digitalWrite(LCD_DC_PIN, 1); bbByte(d); }

// 位带完整初始化（ST7789，2.4TFTST7789参考代码序列，已验证可用）
inline void bbInitPanel() {
    bbCmd(0x01);  // Software Reset
    delay(120);
    bbCmd(0x11);  // Sleep Out
    delay(120);

    // ---- ST7789 寄存器 ----
    bbCmd(0x3A); bbData(0x05);       // COLMOD 65k
    bbCmd(0xC5); bbData(0x1A);       // VCOM
    bbCmd(0x36); bbData(0x60);       // MADCTL 横屏(MX|MV)
    bbCmd(0xB2); bbData(0x05); bbData(0x05); bbData(0x00); bbData(0x33); bbData(0x33); // Porch
    bbCmd(0xB7); bbData(0x05);       // Gate control
    bbCmd(0xBB); bbData(0x3F);       // VCOM
    bbCmd(0xC0); bbData(0x2C);       // Power control
    bbCmd(0xC2); bbData(0x01);       // VDV/VRH enable
    bbCmd(0xC3); bbData(0x0F);       // VRH
    bbCmd(0xC4); bbData(0x20);       // VDV
    bbCmd(0xC6); bbData(0x01);       // Frame rate
    bbCmd(0xD0); bbData(0xA4); bbData(0xA1);  // Power control 1
    bbCmd(0xE8); bbData(0x03);       // Power control 1
    bbCmd(0xE9); bbData(0x09); bbData(0x09); bbData(0x08);  // Equalize
    bbCmd(0xE0);                     // Gamma +
    { uint8_t g[14] = {0xD0,0x05,0x09,0x09,0x08,0x14,0x28,0x33,0x3F,0x07,0x13,0x14,0x28,0x30};
      for (int i = 0; i < 14; i++) bbData(g[i]); }
    bbCmd(0xE1);                     // Gamma -
    { uint8_t g[14] = {0xD0,0x05,0x09,0x09,0x08,0x03,0x24,0x32,0x32,0x3B,0x14,0x13,0x28,0x2F};
      for (int i = 0; i < 14; i++) bbData(g[i]); }

    bbCmd(0x20);  // INV OFF（若颜色反了改 0x21）
    delay(120);
    bbCmd(0x29);  // Display ON
    Serial.println(F("[LCD] BB panel init done (ST7789)"));
}

// 设置地址窗口（逻辑坐标，横屏 320x240；MADCTL 已设横屏，面板自行重映射）
inline void bbSetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    bbCmd(0x2A);
    bbData(x0 >> 8); bbData(x0 & 0xFF);
    bbData(x1 >> 8); bbData(x1 & 0xFF);
    bbCmd(0x2B);
    bbData(y0 >> 8); bbData(y0 & 0xFF);
    bbData(y1 >> 8); bbData(y1 & 0xFF);
    bbCmd(0x2C);
}

// 填充矩形（逻辑坐标）
inline void bbFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (w <= 0 || h <= 0) return;
    // 裁剪到 320x240
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > 320) w = 320 - x;
    if (y + h > 240) h = 240 - y;
    if (w <= 0 || h <= 0) return;

    bbSetWindow(x, y, x + w - 1, y + h - 1);
    uint32_t n = (uint32_t)w * (uint32_t)h;
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (uint32_t i = 0; i < n; i++) { bbData(hi); bbData(lo); }
}

// 整屏填充（320x240）
inline void bbFillScreen(uint16_t color) {
    bbFillRect(0, 0, 320, 240, color);
}

// 画点
inline void bbDrawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || y < 0 || x >= 320 || y >= 240) return;
    bbSetWindow(x, y, x, y);
    bbData(color >> 8); bbData(color & 0xFF);
}

// 水平/垂直线
inline void bbDrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    bbFillRect(x, y, w, 1, color);
}
inline void bbDrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    bbFillRect(x, y, 1, h, color);
}

// 线段（Bresenham）
inline void bbDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy, e2;
    for (;;) {
        bbDrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// 矩形边框
inline void bbDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    bbDrawHLine(x, y, w, color);
    bbDrawHLine(x, y + h - 1, w, color);
    bbDrawVLine(x, y, h, color);
    bbDrawVLine(x + w - 1, y, h, color);
}

// 小圆点（半径2 → 3x3）
inline void bbFillCircle(int16_t x, int16_t y, int16_t r, uint16_t color) {
    (void)r;
    bbFillRect(x - 1, y - 1, 3, 3, color);
}

// 单字符（5x7，size=1/2）
inline void bbDrawChar(int16_t x, int16_t y, char c, uint16_t color, uint8_t size) {
    if (c < 0x20 || c > 0x7E) c = '?';
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = pgm_read_byte(&bbFont5x7[c * 5 + i]);
        for (uint8_t j = 0; j < 8; j++, line >>= 1) {
            if (line & 1) {
                if (size == 1) bbDrawPixel(x + i, y + j, color);
                else           bbFillRect(x + i * size, y + j * size, size, size, color);
            }
        }
    }
}

// 字符串
inline void bbDrawText(int16_t x, int16_t y, const char* s, uint16_t color, uint8_t size) {
    while (*s) {
        bbDrawChar(x, y, *s++, color, size);
        x += 6 * size;
    }
}

// ------------------------------------------------------------
// 渲染分派层：UI 函数统一经这里绘图
// 方案B（IO10低电平）→ 位带渲染；方案A → Adafruit
// ------------------------------------------------------------
inline void lcdFillScreen(uint16_t c) {
    if (_lcdVariantB) bbFillScreen(c);
    else if (_tft)    _tft->fillScreen(c);
}
inline void lcdFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) {
    if (_lcdVariantB) bbFillRect(x, y, w, h, c);
    else if (_tft)    _tft->fillRect(x, y, w, h, c);
}
inline void lcdDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) {
    if (_lcdVariantB) bbDrawRect(x, y, w, h, c);
    else if (_tft)    _tft->drawRect(x, y, w, h, c);
}
inline void lcdDrawHLine(int16_t x, int16_t y, int16_t w, uint16_t c) {
    if (_lcdVariantB) bbDrawHLine(x, y, w, c);
    else if (_tft)    _tft->drawFastHLine(x, y, w, c);
}
inline void lcdDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t c) {
    if (_lcdVariantB) bbDrawLine(x0, y0, x1, y1, c);
    else if (_tft)    _tft->drawLine(x0, y0, x1, y1, c);
}
inline void lcdFillCircle(int16_t x, int16_t y, int16_t r, uint16_t c) {
    if (_lcdVariantB) bbFillCircle(x, y, r, c);
    else if (_tft)    _tft->fillCircle(x, y, r, c);
}
// 文本（x,y 为左上角）
inline void lcdPrintText(int16_t x, int16_t y, const char* s, uint16_t c, uint8_t size) {
    if (_lcdVariantB) bbDrawText(x, y, s, c, size);
    else if (_tft) {
        _tft->setTextColor(c);
        _tft->setTextSize(size);
        _tft->setCursor(x, y);
        _tft->print(s);
    }
}
