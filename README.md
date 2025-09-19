Shield: [![CC BY-NC 4.0][cc-by-nc-shield]][cc-by-nc]

This work is licensed under a
[Creative Commons Attribution-NonCommercial 4.0 International License][cc-by-nc].

[![CC BY-NC 4.0][cc-by-nc-image]][cc-by-nc]

[cc-by-nc]: https://creativecommons.org/licenses/by-nc/4.0/
[cc-by-nc-image]: https://licensebuttons.net/l/by-nc/4.0/88x31.png
[cc-by-nc-shield]: https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg


# PPO2 Display

A high-precision Partial Pressure of Oxygen (PPO2) monitoring system for technical diving, built on ESP32-C3 with dual O2 sensor support and advanced signal processing algorithms.

## Safety Notes

⚠️ **WARNING**:

OXYGEN PPO MONITORING SOFTWARE

This software is not a dive computer, life-support system, medical device, or safety-critical instrument. It is provided for informational and educational purposes only. Readings may be inaccurate, delayed, or fail due to sensor drift/aging, moisture, calibration error, power or hardware faults, electromagnetic interference, or software bugs.

Do not rely on this software to determine breathing-gas safety. Always follow certified dive training and procedures, use approved equipment, carry independent/redundant instruments, and maintain conservative dive plans. Diving—especially with rebreathers—carries inherent risks of serious injury or death.

The software is provided “AS IS,” without warranties of any kind. To the maximum extent permitted by law, the developer disclaims all liability for any loss, damage, injury, or death arising from use or misuse of this software. By downloading, installing, or using the software, you acknowledge and accept all risks and agree to release, indemnify, and hold the developer harmless from any claims.

If you do not agree, do not use this software.

⚠️ **IMPORTANT**: This sostware developed  using Anthropic Claude heavly with lack of human verification and testing.  

⚠️ **IMPORTANT**: This system is designed for TESTING PURPUSES ONLY. Users must:

1. **Validate Calibration**: Always verify calibration with known gas mixtures
2. **Dual Sensor Monitoring**: Never dive with single sensor operation
3. **Regular Maintenance**: Check sensor health and calibration frequently
4. **Backup Systems**: Always carry independent PPO2 monitoring
5. **Training Required**: Proper technical diving certification essential

This system is a diving aid and should not be the sole method of gas monitoring during technical dives.



## Required Tools & Environment

  - ESP-IDF Framework - Espressif's official development framework
  - CMake (3.16+) - Build system
  - Python - For ESP-IDF tools and flashing
  - Git - For version control and ESP-IDF installation

## Hardware Platform

  - Target MCU: ESP32-C3 Development Board
  - Display: 128x128 SH1107 OLED (I2C at 0x3D, 400kHz)
  - Sensors: Internal ADC for O2 sensor readings
  - Buttons: 2x GPIO buttons (MODE/SELECT)
  - LED: Built-in RGB LED for warnings

## Key Libraries & Components

  - FreeRTOS - Real-time operating system (1kHz tick)
  - ESP-IDF I2C Master Driver - Hardware communication
  - NVS Flash - Non-volatile storage for configuration
  - ESP Task Watchdog - System safety monitoring
  - Custom Components:
    - Display Manager (SH1107 driver)
    - Sensor Manager (O2 sensor interface)
    - Button Manager (GPIO with debouncing)
    - Warning Manager (LED control)
    - PPO2 Logger (data logging)

### Pin Configuration
```
GPIO 0  -> ADC1_CH0 (O2 Sensor 1)
GPIO 1  -> ADC1_CH1 (O2 Sensor 2) 
GPIO 3  -> ADC1_CH3 (Battery voltage divider 1/3)
GPIO 6  -> I2C SDA (Display)
GPIO 7  -> I2C SCL (Display)
GPIO 5  -> MODE Button
GPIO 10  -> SELECT Button
GPIO 8  -> RGB LED (Warning system)
```

![scheme](https://github.com/iho77/ppo_v2/blob/main/circuit.png?raw=true)

## Key Features



### Sensor Management
- **Dual O2 Sensor Support**: Independent sensor processing with disagreement detection
- **Single sensor auto detection**: Short  Sensor + in cable connector to ground - device starts to work in 1 sensor modeI
- **Multipoint Calibration System**: Air (21%), O2 (100%) or custom mix calibration points
- **Advanced Signal Processing**: Median-of-5 → EMA → Slew-rate limiting pipeline
- **Sensor Health Monitoring**: Automatic failure detection and recovery
- **Battery Monitoring**: Real-time voltage and percentage display


### Display System
- **Main Display**: Large PPO2 values with calibration status indicators
- **Warning System**: Scrolling text warnings with LED color coding
- **Battery Display**: Dynamic battery icon with voltage readout
- **Menu System**: Clean navigation with visual selection indicators

### Warning System
- **PPO2 Thresholds**: Configurable low/high warning and alarm levels
- **Sensor Disagreement**: Automatic detection of sensor variance
- **Visual Alerts**: Color-coded LED (Green/Yellow/Red) with blinking patterns
- **Display Integration**: Warning messages shown on main screen

## Core Algorithms

### 1. Robust Signal Processing Pipeline

The system implements a sophisticated 3-stage filtering pipeline for each O2 sensor:

Use 3 stages: spike removal → low-pass → rate limit + debounced alarms.

Spike/outlier killer (median-of-5)
At each 20 Hz step, keep the last 5 raw samples and take the median.

Purpose: remove single-sample glitches without adding much lag (~0.1–0.15 s).

Exponential moving average (EMA) with ~5 s time constant
Let y[n] = y[n−1] + α (x_med[n] − y[n−1]), where x_med is from step 1.

Sampling period dt = 1/20 = 0.05 s.

Choose time constant τ ≈ 5 s → α = dt/τ = 0.05/5 = 0.01.

Why: 5 s LPF kills high-freq jitter while keeping latency modest (~τ behind fast ramps).

Slew-rate limiter (enforce physics)
If the environment cannot change by more than Δ_meaningful per 10 s, set

S_max = Δ_meaningful / 10 (units per second).

Per-sample limit: Δ_per_sample = S_max * dt.
Then clamp the EMA output change:
y_lim[n] = clamp(y[n−1] − Δ_per_sample, y[n], y[n−1] + Δ_per_sample).

Purpose: guarantees your displayed value can’t “move” faster than reality.

Debounced, hysteretic warnings (no flicker)

Trigger threshold: T_hi. Clear threshold: T_lo (with T_lo closer to normal than T_hi → hysteresis gap).

Require persistence: only raise a warning if y_lim[n] > T_hi continuously for T_confirm (e.g., 2–3 s).

Only clear if y_lim[n] < T_lo continuously for T_clear (e.g., 2–3 s).

Implement with two counters that increment while condition holds and reset otherwise.

That stack is O(1) memory and CPU, super stable, and easy to tune.

### 2. Multipoint Calibration System

Replaces traditional linear calibration with a robust 1 or 2-point system:

#### Calibration Points
1. **Air Calibration** (21% O2 at 1 bar): `PPO2 = 0.21 bar`
2. **Oxygen Calibration** (100% O2 at 1 bar): `PPO2 = 1.00 bar`
2. **Custom mix Calibration** (Custom % O2 at 1 bar)

System can works with 1 point calibration or 2 points calibration. In case of 1 point calibration second point is 0 mv\ 0% oxygen.


### 3. Sensor Health Monitoring


#### Health Checks
1. **Voltage Range Validation**: 0-1000mV acceptable range
2. **Calibration Validity**: Both sensors must have valid calibration
3. **Sensor Disagreement**: PPO2 difference > 0.05 bar triggers warning
4. **Communication Errors**: ADC read failures tracked and recovered


# Calibration-Based Drift & Linearity Tracking (Strategy 1)

A clear, hands-on guide you can mirror in logs, a spreadsheet, or firmware notes.

## What you track (per sensor)

* **Gain / sensitivity `k`** (mV/ata): millivolts per 1.0 ata ppO₂.
* **Offset `b`** (mV): reading at ppO₂ = 0 (should be near 0).
* **Normalized sensitivity `S = k / k₀`** vs **new-sensor baseline** `k₀`.
* **Linearity** across multiple known ppO₂ points: **`R²`** and **max residual** (mV).
* **Drift rate** (e.g., %/month) and **predicted EOL** date (when `S` hits your fail threshold, e.g., `0.70`).

---

## Two-Point Calibration (minimum viable)

At each calibration, expose the sensor to two **known** ppO₂ points and record:

* `(p₁, m₁)`: e.g., **air at surface** → `p₁ ≈ 0.209` ata, measured `m₁` (mV)
* `(p₂, m₂)`: e.g., **100% O₂ at surface** → `p₂ = 1.00` ata, measured `m₂` (mV)

Fit a line `m = k·p + b`:

```
k = (m₂ − m₁) / (p₂ − p₁)
b = m₁ − k · p₁
```

Then compute **normalized sensitivity**:

```
S = k / k₀
```

Where `k₀` is your **new-sensor baseline** (mV/ata).

---

## Worked Example A — two-point checks over time (one sensor)

Assume `k₀ = 52.0` mV/ata (≈ 11 mV in air, ≈ 52 mV at 1.0 ata O₂).

| Date       | Air mV `m₁` | O₂ mV `m₂` | Estimated `k` (mV/ata) | `b` (mV) | `S = k/k₀` |
| ---------- | ----------: | ---------: | ---------------------: | -------: | ---------: |
| 2025-01-01 |       10.99 |      52.25 |                  52.16 |     0.09 |      1.003 |
| 2025-05-01 |       10.45 |      49.51 |                  49.37 |     0.14 |      0.950 |
| 2025-10-01 |        9.48 |      45.86 |                  46.00 |    −0.14 |      0.885 |
| 2026-03-01 |        8.18 |      39.82 |                  39.99 |    −0.18 |      0.769 |

### Interpretation

* Sensitivity `S` drifts from \~1.00 → 0.77 over \~14 months (≈ 1.6 %/month).
* **Caution** when `S < 0.80` (here: 2026-03-01).
* **Predict EOL** when `S` hits **0.70**: extrapolation from the last trend ≈ **Aug 2026** (\~19 months from first calibration).

**Log each time:** the two mV readings and the computed `k`, `b`, `S`, plus a trendline of `S` vs time (and ETA to `S = 0.70`).

---

## Practical guardrails

* If `|b| > 2–3` mV or it drifts steadily, investigate (zero-current leakage, moisture, wiring).
* If degradation accelerates (> \~3–5 %/month), plan an early swap.



#### Failure Recovery
- **Graceful Degradation**: System continues with single sensor
- **Fail-Safe PPO2**: Conservative 0.21 bar default during total failure
- **Automatic Recovery**: Sensors re-validated on successful reads

### 4. Warning State Machine

#### Warning States
```c
typedef enum {
    WARNING_STATE_NORMAL = 0,    // Green LED - PPO2 in range
    WARNING_STATE_WARNING,       // Yellow blinking - PPO2 at limits  
    WARNING_STATE_ALARM         // Red blinking - PPO2 dangerous
} warning_state_t;
```

#### PPO2 Thresholds (Configurable)
- **Low Warning**: 0.19 bar (hypoxia risk)
- **Low Alarm**: 0.18 bar (severe hypoxia)
- **High Warning**: 1.4 bar (oxygen toxicity risk)
- **High Alarm**: 1.6 bar (severe toxicity risk)

#### LED Patterns
- **Normal**: Solid green
- **Warning**: Yellow blink at 2Hz (250ms on/off)
- **Alarm**: Red blink at 4Hz (125ms on/off)


## System Architecture

### Component Organization
```
├── main/                   # Main application and mode management
│   ├── main.c             # System initialization and 20Hz main loop
│   └── modes/             # Mode-specific implementations
├── components/
│   ├── app_common/        # Shared types and configuration
│   ├── sensor_manager/    # O2 sensor processing and calibration
│   ├── display_manager/   # LVGL display driver and UI
│   ├── warning_manager/   # LED warning system
│   ├── button_manager/    # GPIO button handling
│   └── sh1107_driver/     # Low-level display driver
```

### Data Flow
```
Sensors (20Hz) → Filtering → Calibration → Warning Analysis → Display Update
                     ↓
              Health Monitoring → Recovery Actions
```

## Menu System

### Main Menu Items
1. **Sensor Calibration** - Dual sensor calibration with gas selection
2. **Reset Calibration** - Individual or bulk calibration reset
3. **Print Logs** - Debug output to serial console
4. **Sensor Health** - Real-time sensor diagnostics
5. **System Setup** - Configuration and thresholds
6. **Power Off** - System shutdown

### Calibration Workflow
1. **Gas Selection**: Air, O2, or Custom (configurable %)
2. **Dual Sensor Display**: Live mV readings for both sensors
3. **Calibration Action**: Simultaneous calibration of both sensors
4. **Validation**: Automatic sanity checks and confirmation

### Setup Options
- **PPO2 Thresholds**: Warning and alarm levels for low/high PPO2
- **Display Settings**: Brightness and contrast
- **Sensor Configuration**: Individual sensor enable/disable
- **System Parameters**: Sample rates and filter constants

## Technical Specifications

### Performance
- **Update Rate**: 20 Hz sensor sampling and display refresh
- **Response Time**: ~5 seconds (90% settling due to EMA filtering)
- **Accuracy**: ±0.01 bar PPO2 (post-calibration)
- **Resolution**: 12-bit ADC (0.25mV resolution)

## Contributing

Contributions welcome! Please read CONTRIBUTING.md for guidelines on:
- Code style and standards
- Testing requirements
- Safety considerations for diving equipment
