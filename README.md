# Alphasense B43F – ESP32-S3 PlatformIO Reader (ADS1115 I2C)

Firmware for reading analogue outputs of the **Alphasense B4 Series (B43F)**
electrochemical gas sensor via a **custom board** based on the
**Alphasense ISB Rev5 4-Elec**, using an **ADS1115 16-bit I2C ADC** connected
to an **ESP32-S3** running the **Arduino framework** inside **PlatformIO**.

The ADS1115 replaces the internal ESP32-S3 ADC for higher resolution (16-bit
vs 12-bit), a stable internal reference, and better noise performance — all
important for low-current electrochemical sensor outputs.

---

## Quick Start

### Prerequisites

| Tool | Version |
|------|---------|
| [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/) | ≥ 6.x |
| Python | ≥ 3.8 |
| USB driver for your ESP32-S3 board | – |

### 1. Clone / open the project

```bash
git clone https://github.com/carminelau/test_platformio_alphasense.git
cd test_platformio_alphasense
```

### 2. Verify I2C pins and channel mapping (⚠ required before first flash)

Edit **`include/config.h`** and confirm:
- `SDA_PIN` / `SCL_PIN` match your ESP32-S3 board variant (see table below)
- `ADS_CH_WE` / `ADS_CH_AE` match the ADS1115 input connected to WE and AE

### 3. Build

```bash
pio run -e esp32s3
```

### 4. Upload

```bash
pio run -e esp32s3 --target upload
```

### 5. Monitor Serial output

```bash
pio device monitor
```

Expected output (CSV, one row per sample):

```
timestamp_ms, we_raw, ae_raw, we_v, ae_v, signal_v, we_ema, ae_ema, signal_ema
1001,  12345,  11900,  0.77152,  0.74375,  0.02777,  0.07715,  0.07438,  0.00278
2002,  12350,  11905,  0.77184,  0.74406,  0.02778,  0.14374,  0.13832,  0.00542
```

`signal_v` = `we_v − ae_v` (compensated, removes common-mode drift).
`*_ema` columns are the EMA-filtered versions for smooth trending.

---

## Configurazione

All tunables live in **`include/config.h`**.
Build-flag overrides (e.g. `-DSAMPLE_INTERVAL_MS=500`) can be set in
`platformio.ini` without editing source.

### I2C Pins

| Board variant (FirmwareSensy) | SDA_PIN | SCL_PIN |
|-------------------------------|---------|---------|
| sensy_2024_V1_green           | 1       | 2       |
| sensy_2024_V2_ENEA            | 8       | 9       |
| sensy_2024_V3_red             | 8       | 9       |
| sensy_2024_V4_green / _black  | 8       | 9 ← **default** |

Change via `build_flags` in `platformio.ini`: `-DSDA_PIN=1 -DSCL_PIN=2`

### ADS1115 Configuration

| Constant | Default | Description |
|----------|---------|-------------|
| `ADS1115_I2C_ADDRESS` | `0x48` | I2C address (ADDR→GND=0x48, →VDD=0x49) |
| `ADS_CH_WE` | `0` | ADS1115 AIN channel for WE output ⚠ |
| `ADS_CH_AE` | `1` | ADS1115 AIN channel for AE output ⚠ |
| `ADS1115_GAIN` | `GAIN_TWO` | PGA ±2.048 V (0.0625 mV/bit) |
| `ADS1115_SPS` | `RATE_ADS1115_128SPS` | 128 samples/s conversion rate |
| `ADS1115_DIFFERENTIAL_MODE` | `0` | 0=single-ended, 1=HW differential |

### Calibration Constants

| Constant | Default | Description |
|----------|---------|-------------|
| `WE_OFFSET_V` | `0.0` | WE zero offset (V) ⚠ calibrate |
| `WE_GAIN_CAL` | `1.0` | WE slope gain ⚠ calibrate |
| `AE_OFFSET_V` | `0.0` | AE zero offset (V) ⚠ calibrate |
| `AE_GAIN_CAL` | `1.0` | AE slope gain ⚠ calibrate |

### Sampling / Filtering

| Constant | Default | Description |
|----------|---------|-------------|
| `SAMPLE_INTERVAL_MS` | `1000` | Acquisition period (ms) |
| `EMA_ALPHA` | `0.1` | EMA smoothing factor (0=max, 1=off) |

---

## Calibrazione e limiti

### Procedura di Calibrazione Base

1. Power up with a **clean-air environment** (no target gas).
2. Read `we_v` and `ae_v` for ~5 minutes to get a stable baseline.
3. Set `WE_OFFSET_V = mean(we_v)` and `AE_OFFSET_V = mean(ae_v)` in `config.h`.
   After this, `signal_v` should hover near **0 V** in clean air.
4. For gain calibration, apply a known voltage to the ISB output connector
   (with sensor disconnected), measure the reported `we_v`, and compute:
   `WE_GAIN_CAL = reference_V / reported_we_v`.
5. Re-flash and verify `signal_v ≈ 0` in clean air.

> **Note**: ADS1115 GAIN_TWO (±2.048 V) has a guaranteed accuracy of
> ±0.05% (typ). Offset and gain error are each ≤ 1 LSB typ. No Vref
> calibration is needed — unlike the ESP32 internal ADC.

### ADS1115 vs ESP32 Internal ADC

| Property | ESP32-S3 internal ADC | ADS1115 |
|----------|-----------------------|---------|
| Resolution | 12-bit | 16-bit |
| Vref accuracy | ±5% (chip-to-chip) | ±0.05% internal |
| INL | ≈1–2% | < 0.01% |
| Noise | ~300 µV rms | ~30 µV rms (at 128 SPS) |
| Range (this config) | 0–2.6 V | ±2.048 V |

### Rumore e Drift Termico

- ADS1115 datasheet specifies 0.1 µV/°C offset drift (typ) — excellent for
  electrochemical sensors, which also drift.
- The EMA filter (`EMA_ALPHA = 0.1`, ~10-sample effective window at 1 Hz)
  attenuates high-frequency noise. Decrease alpha for more smoothing,
  increase for faster tracking.
- An optional **RC low-pass filter** (10 kΩ + 100 nF, fc ≈ 160 Hz) on each
  ADS1115 AINx line reduces RF pick-up before the ADC input.
- **Single-point GND**: connect ESP32-S3 GND, ADS1115 GND, and ISB GND at
  one point to avoid ground loops.

### Conversione in Concentrazione

`convertToConcentration()` in `include/alphasense_b43f.h` is intentionally
a stub (returns 0.0). To implement it you need (from Alphasense docs):

1. TIA (transimpedance amplifier) gain on the ISB board (Ω)
2. Sensor sensitivity from the Alphasense calibration certificate (nA/ppb)
3. Zero-current baseline voltage in clean air
4. Optional: temperature-compensation coefficients

See Alphasense application notes **AAN 803** (B4 series circuit) and
**AAN 110** (ISB setup) for the full algorithm.

---

## Schema di Collegamento

### Block Diagram

```
  3.3 V supply (ESP32-S3 3V3 pin)
       │
       ├─────────────────────────────────────────────────┐
       │                                                 │
  ┌────┴──────────────────────────────┐     ┌───────────┴──────────────┐
  │   Alphasense ISB Rev5 4-Elec      │     │   ADS1115 (I2C, 16-bit)  │
  │   (custom board)                  │     │                          │
  │                                   │     │  AIN0 ◄── ISB WE out ⚠  │
  │  [B43F Sensor]─WE out ───────────►│─────► AIN0                    │
  │              ─AE out ───────────►│─────► AIN1 ◄── ISB AE out ⚠  │
  │                                   │     │                          │
  │  GND ─────────────────────────────│─────► GND                     │
  └───────────────────────────────────┘     │  VDD ◄── 3.3 V          │
                                            │  SCL ◄── ESP32 GPIO9 ⚠  │
                                            │  SDA ◄── ESP32 GPIO8 ⚠  │
                                            │  ADDR → GND (addr 0x48) │
                                            └──────────┬───────────────┘
                                                       │ I2C
                                          ┌────────────┴─────────┐
                                          │  ESP32-S3 DevKit      │
                                          │  GPIO8  = SDA ⚠       │
                                          │  GPIO9  = SCL ⚠       │
                                          │  3V3    → ADS1115 VDD │
                                          │  GND    → common GND  │
                                          └───────────────────────┘
```

> ⚠ All GPIO numbers and ISB connector labels are **placeholders** based on
> the FirmwareSensy sensy_2024 V3/V4 default (`SDA=8`, `SCL=9`).
> **Verify against your custom board schematic and PCB silkscreen**,
> then update `SDA_PIN`, `SCL_PIN`, `ADS_CH_WE`, `ADS_CH_AE` in `config.h`.

### Wiring Table

| Segnale | Da | A | Note |
|---------|-----|---|------|
| SDA | ESP32-S3 `GPIO8` ⚠ | ADS1115 `SDA` | I2C data – verify pin for your board variant |
| SCL | ESP32-S3 `GPIO9` ⚠ | ADS1115 `SCL` | I2C clock – verify pin for your board variant |
| ADS1115 VDD | ESP32-S3 `3V3` | ADS1115 `VDD` | 3.3 V power |
| ADS1115 ADDR | ADS1115 `GND` | ADS1115 `ADDR` | Sets I2C address to 0x48 |
| WE output | ISB `WE_OUT` ⚠ | ADS1115 `AIN0` | Working electrode analogue output |
| AE output | ISB `AE_OUT` ⚠ | ADS1115 `AIN1` | Auxiliary electrode analogue output |
| GND | ESP32-S3 `GND` | ADS1115 `GND` + ISB `GND` | Single-point common ground |
| ISB VCC | ESP32-S3 `3V3` or `5V` | ISB `VCC` ⚠ | Check ISB supply voltage requirement |

> ⚠ Connector/pin names on the ISB board (`WE_OUT`, `AE_OUT`, `VCC`) are
> based on typical ISB Rev5 labelling.
> **SOSTITUIRE CON NOMI REALI** from your custom board schematic.

---

## Verifica (checklist)

- [ ] **Compila** – `pio run -e esp32s3` → exit 0, no errors
- [ ] **Upload** – `pio run -e esp32s3 --target upload` → "Leaving... Hard resetting…"
- [ ] **Monitor** – `pio device monitor` → CSV rows appear every ~1 s
- [ ] **ADS1115 found** – no `[ADS1115] ERROR: not found` message at startup
- [ ] **Valori plausibili** – `we_v` and `ae_v` in 0–2 V range when ISB powered;
      `signal_v` near 0 V in clean air after offset calibration
- [ ] **Calibrazione** – apply known voltage to ISB output, verify `we_v` matches,
      set `WE_OFFSET_V` / `WE_GAIN_CAL` if needed