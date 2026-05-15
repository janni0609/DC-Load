# DC Electronic Load

A programmable DC electronic load built on the Raspberry Pi Pico 2 (RP2350). It sinks a user-defined current from a DC source under test and measures voltage, current, power, capacity, and energy in real time on a colour TFT display.

## Hardware

| Component | Part | Interface |
|---|---|---|
| MCU | Raspberry Pi Pico 2 (RP2350) | — |
| Display | ILI9488 TFT 240×320 | SPI (GPIO 16/18/19/17/20) |
| DAC (current setpoint) | DAC80501 16-bit | I2C1 0x48 |
| ADC (V + I measurement) | ADS1115 16-bit | I2C1 0x49 |
| I/O expander (buttons/encoder) | TCAL9539 16-bit | I2C1 0x74 |
| Temperature sensors | 3× MCP9700 | ADC (GPIO 26/27/28) |
| Fans | 2× PWM | GPIO 2, 3 (25 kHz) |
| Beeper | — | GPIO 22 |

I2C1 runs on GPIO 14 (SDA) / GPIO 15 (SCL) at 1 MHz.

## Specifications

| Parameter | Range |
|---|---|
| Input voltage | 0 – 100 V |
| Set current | 0 – 100 A |
| Under-voltage / CV setpoint (UVP) | 0 – 99.99 V |
| Max power (beeper threshold) | 1000 W |

The DAC output (0–2.5 V) drives the current-control circuit: 2.5 V = 100 A full scale. The ADS1115 uses auto-ranging PGA for both channels, scaling 2.5 V → 100 V (voltage) and 2.5 V → 100 A (current).

## Display Layout

Portrait 240×320, divided into sections:

```
┌──────────────────────────┐
│    DC ELECTRONIC LOAD    │  ← green = load ON, yellow = load OFF
├──────────────────────────┤
│ VOLTAGE          xx.xx V │
├──────────────────────────┤
│ CURRENT         x.xxx A  │
├──────────────────────────┤
│ POWER           xxx.xx W │
├──────────────┬───────────┤
│ CAPACITY x.xxx Ah │ x.xx Wh ENERGY │
├──────────────┬───────────┤
│ SET CURR x.x A │ SET UVP xx.xx V │  ← UVP turns red in CV mode
├──────────────┬───────────┤
│ TEMP  xx.x°C │ HH:MM:SS  │
└──────────────┴───────────┘
```

## Controls

All buttons and the rotary encoder are read via the TCAL9539 I/O expander. Button behaviour is context-sensitive: the main screen, debug screen, and calibration screens each respond differently.

### Main screen

| Input | Function |
|---|---|
| Rotary encoder | Increment/decrement the selected digit of the active setpoint |
| P0.0 (encoder push) | Toggle between editing **SET CURR** and **SET UVP** |
| P0.3 | Move cursor left |
| P0.4 | Move cursor right |
| P0.5 | Switch to debug screen |
| P0.6 | Reset elapsed timer and clear Ah/Wh accumulators |
| P0.7 | Toggle internal / external voltage sense |
| P1.0 | Load ON / OFF (blocked while OTP is active) |

### Debug screen

The debug screen is a navigable menu. P0.3/P0.4 move the selection, the encoder adjusts the highlighted item, and P0.0 (encoder push) activates it.

| Menu item | Function |
|---|---|
| I SENSE | Toggle internal / external current shunt (persisted to EEPROM) |
| VOLT CAL | Enter voltage calibration wizard |
| CURR CAL | Enter current calibration wizard |
| VIEW CAL | Display stored calibration tables |
| OTP TRIP | Set over-temperature protection threshold with encoder (persisted) |
| UVP Kp | Adjust UVP PI proportional gain with encoder (persisted) |
| UVP Ki | Adjust UVP PI integral gain with encoder (persisted) |

### Calibration screen

| Input | Function |
|---|---|
| Rotary encoder | Adjust the entered reference value |
| P0.3 / P0.4 | Move digit cursor left / right |
| P0.0 (encoder push) | Confirm step and advance |
| P0.5 | Cancel and return to debug screen |

## UVP / Constant-Voltage Mode

When **SET UVP** is non-zero and the load is on, a software PI controller monitors the measured voltage. If it drops below the UVP setpoint, the controller reduces the sink current to regulate voltage at that level (constant-voltage / CV mode). The SET UVP field turns red while CV mode is active.

CV mode exits with 0.5 V hysteresis to avoid chattering. The PI gains (Kp, Ki) are tunable from the debug screen and stored in EEPROM.

## Over-Temperature Protection (OTP)

The OTP threshold (default 70 °C, adjustable 30–120 °C) is compared against the highest reading across all three MCP9700 sensors. When the threshold is exceeded:

- The load is turned off immediately.
- Five 500 ms beeps are sounded.
- The load ON button is blocked until the temperature drops 5 °C below the threshold.

## Fan Control

Fan speed is temperature-controlled using the highest reading across all three MCP9700 sensors:

- Off below 30 °C
- Linear ramp from 0–100 % between 30 °C and 50 °C
- Full speed above 50 °C

## Calibration

Both voltage and current channels support piecewise-linear calibration (4 points for voltage, 5 for current). The calibration wizard drives the load to each reference point, reads the raw ADC value, and prompts for the reference meter reading. Calibration data is stored in EEPROM and applied on every measurement via `piecewiseLerp`.

### EEPROM layout

| Address | Content |
|---|---|
| 0 | Current shunt select (0x01 = external) |
| 1 | Current cal valid marker (0xCA) |
| 2 | Voltage cal valid marker (0xCA) |
| 3–42 | Current cal: 5× raw + 5× user (10 floats) |
| 43–74 | Voltage cal: 4× raw + 4× user (8 floats) |
| 75 | OTP valid marker (0xCA) |
| 76–79 | OTP threshold (float) |
| 80 | UVP PI valid marker (0xCA) |
| 81–84 | UVP Kp (float) |
| 85–88 | UVP Ki (float) |

## Building

The project uses [PlatformIO](https://platformio.org/) with the Arduino framework.

```bash
# Build
pio run

# Upload
pio run --target upload

# Serial monitor
pio device monitor --baud 115200
```

Target board: `rpipico2`. Dependencies are fetched automatically (`Bodmer/TFT_eSPI`).
