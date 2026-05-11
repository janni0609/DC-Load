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
| Under-voltage protection (UVP) | 0 – 99.99 V |
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
│ SET CURR x.x A │ SET UVP xx.xx V │
├──────────────┬───────────┤
│ TEMP  xx.x°C │ HH:MM:SS  │
└──────────────┴───────────┘
```

A debug screen (toggled by button) shows raw I2C device status for DAC80501, ADS1115, and TCAL9539, plus all three temperature sensor readings.

## Controls

All buttons and the rotary encoder are read via the TCAL9539 I/O expander.

| Input | Function |
|---|---|
| Rotary encoder | Increment/decrement the selected digit of the active setpoint |
| P0.0 | Toggle between editing **SET CURR** and **SET UVP** |
| P0.3 | Move cursor left |
| P0.4 | Move cursor right |
| P0.5 | Toggle debug screen |
| P0.6 | Reset elapsed timer and clear Ah/Wh accumulators |
| P0.7 | Toggle internal / external voltage sense |
| P1.0 | Load ON / OFF |

## Fan Control

Fan speed is temperature-controlled using the highest reading across all three MCP9700 sensors:

- Off below 35 °C
- Linear ramp from 0–100 % between 35 °C and 55 °C
- Full speed above 55 °C

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
