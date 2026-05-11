#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include "TCAL9539.h"

// ── DAC80501 ─────────────────────────────────────────────────────────────────
// Datasheet: SBAS794E (DAC80501/DAC70501/DAC60501)
// I2C address: SYNC/A0=GND → 1001 000 = 0x48
// Internal reference: 2.5 V
// GAIN reg resets to 0x0001 (BUFF-GAIN=1 → output gain=2 → full-scale = 5 V)
// REF-ALARM in STATUS reg forces output to 0 V when VREFIO headroom is too low
#define DAC80501_ADDR        0x48
#define DAC80501_REG_DEVID   0x01  // [14:12]=RESOLUTION, [7]=RSTSEL; 0x0115 = DAC80501Z
#define DAC80501_REG_CONFIG  0x03  // [8]=REF_PWDWN, [0]=DAC_PWDWN; reset=0x0000
#define DAC80501_REG_GAIN    0x04  // [8]=REF-DIV, [0]=BUFF-GAIN;   reset=0x0001
#define DAC80501_REG_STATUS  0x07  // [0]=REF-ALARM (1 → output forced to 0 V)
#define DAC80501_REG_DAC     0x08  // DAC-DATA[15:0], MSB-aligned;  reset=0x0000

static void dac80501Write(uint8_t reg, uint16_t val) {
    Wire1.beginTransmission(DAC80501_ADDR);
    Wire1.write(reg);
    Wire1.write(val >> 8);
    Wire1.write(val & 0xFF);
    Wire1.endTransmission();
}

static bool dac80501Read(uint8_t reg, uint16_t &out) {
    Wire1.beginTransmission(DAC80501_ADDR);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((uint8_t)DAC80501_ADDR, (uint8_t)2) != 2) return false;
    out = ((uint16_t)Wire1.read() << 8) | Wire1.read();
    return true;
}

// Confirms I2C ACK and RESOLUTION bits [14:12] == 000b (16-bit DAC80501)
static bool dac80501Verify() {
    uint16_t devid;
    if (!dac80501Read(DAC80501_REG_DEVID, devid)) return false;
    return ((devid >> 12) & 0x7) == 0x0;
}

// ── ADS1115 ──────────────────────────────────────────────────────────────────
// I2C address: ADDR=VDD → 1001 001 = 0x49
// Config register (P[1:0]=01b) resets to 0x8583:
//   OS=1, MUX=000, PGA=010 (±2.048 V), MODE=1 (single-shot),
//   DR=100 (128 SPS), COMP_QUE=11 (disabled)
#define ADS1115_ADDR        0x49
#define ADS1115_REG_CONV    0x00  // conversion result (read after OS=1)
#define ADS1115_REG_CONFIG  0x01

// Base config words with PGA field zeroed; OR with (pga_idx << 9) at runtime.
// Single-shot, 64 SPS (DR=011b), comparator disabled (COMP_QUE=11b).
// Voltage: AIN0–AIN1 (MUX=000) → high=0x81, low=0x63
#define ADS1115_BASE_VOLT   0x8163
// Current: AIN2–AIN3 (MUX=011) → high=0xB1, low=0x63
#define ADS1115_BASE_CURR   0xB163

// 2.5 V on AIN0-AIN1 = 100 V on load input
#define ADS1115_V_SCALE     40.0f
// 2.5 V on AIN2-AIN3 = 100 A
#define ADS1115_I_SCALE     40.0f

// FSR for each PGA[2:0] index (0–5)
static const float kPgaFsr[] = {6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f};
static uint8_t g_pgaV = 1;  // start at ±4.096 V (covers 2.5 V max on voltage channel)
static uint8_t g_pgaA = 2;  // start at ±2.048 V for current channel

static bool ads1115Write(uint8_t reg, uint16_t val) {
    Wire1.beginTransmission(ADS1115_ADDR);
    Wire1.write(reg);
    Wire1.write(val >> 8);
    Wire1.write(val & 0xFF);
    return Wire1.endTransmission() == 0;
}

static bool ads1115Read(uint8_t reg, uint16_t &out) {
    Wire1.beginTransmission(ADS1115_ADDR);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2) != 2) return false;
    out = ((uint16_t)Wire1.read() << 8) | Wire1.read();
    return true;
}

static bool ads1115Ping() {
    Wire1.beginTransmission(ADS1115_ADDR);
    return Wire1.endTransmission() == 0;
}

static bool ads1115Verify() {
    uint16_t cfg;
    if (!ads1115Read(ADS1115_REG_CONFIG, cfg)) return false;
    return cfg == 0x8583;
}

// Triggers a single-shot conversion with auto-ranging PGA.
// Returns NAN on I2C error. Blocks ~16 ms (one 64-SPS cycle).
// Thresholds: >87.5 % FS → lower gain; <43.75 % FS → higher gain (hysteresis gap prevents oscillation).
static float ads1115ReadChannel(uint16_t base_cfg, uint8_t &pga, float scale) {
    uint16_t cfg_word = base_cfg | ((uint16_t)pga << 9);
    if (!ads1115Write(ADS1115_REG_CONFIG, cfg_word)) return NAN;
    uint16_t cfg;
    uint32_t t = millis();
    do {
        if (!ads1115Read(ADS1115_REG_CONFIG, cfg)) return NAN;
    } while (!(cfg & 0x8000) && (millis() - t < 25));
    uint16_t raw;
    if (!ads1115Read(ADS1115_REG_CONV, raw)) return NAN;
    int16_t s = (int16_t)raw;
    float result = s * kPgaFsr[pga] / 32768.0f * scale;
    int16_t mag = s < 0 ? -s : s;
    if      (mag > 28671 && pga > 0) pga--;  // >87.5 % FS → lower gain next time
    else if (mag < 14335 && pga < 5) pga++;  // <43.75 % FS → higher gain next time
    return result;
}

static float ads1115ReadVoltage() {
    return ads1115ReadChannel(ADS1115_BASE_VOLT, g_pgaV, ADS1115_V_SCALE);
}

static float ads1115ReadCurrent() {
    return ads1115ReadChannel(ADS1115_BASE_CURR, g_pgaA, ADS1115_I_SCALE);
}

// Full-scale = 2 × (VREF/2) = 2.5 V  (REF-DIV=1 → VREFIO=1.25 V, BUFF-GAIN=1 → ×2)
// REF-DIV=1 is required on 3.3 V supply: without it VREFIO=2.5 V exceeds 0.5×VDD limit
// and REF-ALARM fires, forcing the output to 0 V.
static void dac80501SetVoltage(float v) {
    uint16_t code = (uint16_t)constrain(v / 2.5f * 65536.0f, 0, 65535);
    dac80501Write(DAC80501_REG_DAC, code);
}

#define PIN_SDA  14
#define PIN_SCL  15
#define PIN_INT   6

// Digital inputs
#define PIN_DIN_11  11

// Digital outputs
#define PIN_DOUT_2   2
#define PIN_DOUT_3   3
#define PIN_DOUT_8   8
#define PIN_DOUT_9   9
#define PIN_DOUT_10 10
#define PIN_DOUT_13 13
#define PIN_BEEPER  22

// Analog inputs (ADC0–ADC2 on RP2040)
#define PIN_ADC0    26
#define PIN_ADC1    27
#define PIN_ADC2    28

// MCP9700: V_out = 500 mV + 10 mV/°C; 2.5 V ADC reference, 10-bit counts
#define MCP9700_V0C    0.500f   // output at 0°C [V]
#define MCP9700_TC     0.010f   // temperature coefficient [V/°C]
#define MCP9700_VREF   2.5f     // ADC reference [V]
#define MCP9700_ADC_FS 1023.0f  // 10-bit full-scale count
#define MCP9700_T_MIN -10.0f    // below this → sensor absent/invalid [°C]
#define MCP9700_T_MAX  125.0f   // above this → sensor absent/invalid [°C]

TCAL9539 ioExp(Wire1);

volatile bool g_intPending = false;
void onTCALInt() { g_intPending = true; }

uint16_t      g_prevState    = 0xFFFF;
int8_t        g_cursorPos    = 3;      // 0=hundreds 1=tens 2=ones 3=tenths
bool          g_loadOn       = false;
bool          g_extVSense    = false;  // false = internal V-sense, true = external (FET on P11)
bool          g_editUVP      = false;  // false = editing set current, true = editing UVP
int8_t        g_uvpCursorPos = 1;      // 0=tens 1=ones 2=tenths 3=hundredths
unsigned long g_timerStart   = 0;      // millis() snapshot for elapsed timer; reset via P0.6

TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

// Color palette — values are pre-inverted (~original) to compensate for
// TFT_INVERSION_ON on this ILI9488 panel. The display hardware inverts them
// back, so they render as the intended colours.
#define COL_BG       0xFFFF    // ~black   → appears black
#define COL_HDR_BG   0xFFF0    // ~0x000F  → appears dark navy
#define COL_DIV      0xD6BA    // ~0x2945  → appears dark grey
#define COL_LABEL    0x8410    // ~0x7BEF  → appears mid grey
#define COL_VOLT     0xF800    // ~cyan    → appears cyan
#define COL_CURR     0xF81F    // ~green   → appears green
#define COL_PWR      0x001F    // ~yellow  → appears yellow
#define COL_CAP      0x0841    // ~0xF7BE  → appears warm white
#define COL_SET      0x02DF    // ~0xFD20  → appears orange
#define COL_TEMP_OK  0x02DF    // ~0xFD20  → appears orange
#define COL_TEMP_HOT 0x07FF    // ~red     → appears red
#define COL_DBG_HDR_BG  0x057F  // ~0xFA80  → appears orange

// Portrait 240x320 — section Y positions and heights
//  Y    H   Section
//  0   24   header
//  24  62   voltage
//  86  62   current
// 148  62   power
// 210  40   capacity (left) | energy (right)
// 250  40   set curr (left) | set UVP (right)
// 290  30   heatsink temp
#define Y_HDR   0
#define Y_VOLT  24
#define Y_CURR  86
#define Y_PWR   148
#define Y_CAP   210
#define Y_SET   250
#define Y_TEMP  290

// Debug screen section Y positions (header 24 + 10×28 = 304 px; screen need not be full)
#define Y_DBG_T0    24
#define Y_DBG_T1    52
#define Y_DBG_T2    80
#define Y_DBG_GPIO 108
#define Y_DBG_DAC  136
#define Y_DBG_ADS  164
#define Y_DBG_TCAL 192
#define Y_DBG_FAN  220
#define Y_DBG_VRAW 248
#define Y_DBG_IRAW 276

// Dummy measurement values
float measV   = 12.34f;
float measA   = 5.678f;
float measW   = 0.0f;
float measAh  = 0.0f;
float measWh  = 0.0f;
float setA    = 6.000f;
float setUVP  = 10.00f;
float tempC   = 42.5f;
float g_tempAll[3]   = {NAN, NAN, NAN};
bool    g_debugScreen  = false;
bool    g_dacOk        = false;
bool    g_adsOk        = false;
bool    g_tcalOk       = false;
uint8_t g_fanPwm       = 0;

// 100 A → 2.5 V; DAC always tracks setpoint, load on/off is handled by a separate enable signal
static void applySetCurrent() {
    dac80501SetVoltage(setA / 100.0f * 2.5f);
}

static float readMCP9700(uint8_t pin) {
    float v = analogRead(pin) * (MCP9700_VREF / MCP9700_ADC_FS);
    float t = (v - MCP9700_V0C) / MCP9700_TC;
    return (t >= MCP9700_T_MIN && t <= MCP9700_T_MAX) ? t : NAN;
}

// ── Primitives ───────────────────────────────────────────────────────────────

void drawVSenseIndicator() {
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(1);
    tft.setTextColor(g_extVSense ? COL_VOLT : COL_LABEL, COL_BG);
    tft.drawString(g_extVSense ? "EXT" : "INT", 92, Y_VOLT + 6);
}

void hline(int y) { tft.drawFastHLine(0, y, 240, COL_DIV); }
void vline(int x, int y, int h) { tft.drawFastVLine(x, y, h, COL_DIV); }

void drawLabel(const char *s, int x, int y) {
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(COL_LABEL, COL_BG);
    tft.setTextSize(2);
    tft.drawString(s, x, y);
}

// Right-align value at (rx, y); rendered off-screen then pushed in one transfer to avoid flicker
void drawValue(const char *s, int rx, int y, int cw, int ch, uint16_t col, uint8_t sz) {
    spr.createSprite(cw, ch);
    spr.fillSprite(COL_BG);
    spr.setTextDatum(TR_DATUM);
    spr.setTextColor(col, COL_BG);
    spr.setTextSize(sz);
    spr.drawString(s, cw, 0);
    spr.pushSprite(rx - cw, y);
    spr.deleteSprite();
}

// ── Static frame (once) ──────────────────────────────────────────────────────

void drawHeader() {
    uint16_t bg = g_loadOn ? 0xF81F : 0x001F;  // ~green / ~yellow → appears green / yellow
    tft.fillRect(0, Y_HDR, 240, 24, bg);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(0xFFFF, bg);  // ~black → appears black
    tft.setTextSize(2);
    tft.drawString("DC ELECTRONIC LOAD", 120, 12);
}

void drawFrame() {
    tft.fillScreen(COL_BG);
    drawHeader();

    // Horizontal dividers
    hline(Y_CURR - 1);
    hline(Y_PWR  - 1);
    hline(Y_CAP  - 1);
    hline(Y_SET  - 1);
    hline(Y_TEMP - 1);

    // Vertical splits for the two half-width rows
    vline(119, Y_CAP,  40);
    vline(119, Y_SET,  40);
    vline(119, Y_TEMP, 30);

    // Section labels — textSize 2 (12×16 px per char)
    drawLabel("VOLTAGE",   4,   Y_VOLT + 4);
    drawLabel("CURRENT",   4,   Y_CURR + 4);
    drawLabel("POWER",     4,   Y_PWR  + 4);
    drawLabel("CAPACITY",  4,   Y_CAP  + 4);
    drawLabel("ENERGY",  124,   Y_CAP  + 4);
    drawLabel("SET CURR",  4,   Y_SET  + 4);
    drawLabel("SET UVP", 124,   Y_SET  + 4);
    drawLabel("TEMP",      4,   Y_TEMP + 7);  // same line as value
    drawVSenseIndicator();
}

// ── Debug screen ─────────────────────────────────────────────────────────────

void drawDebugFrame() {
    tft.fillScreen(COL_BG);
    tft.fillRect(0, 0, 240, 24, COL_DBG_HDR_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(0xFFFF, COL_DBG_HDR_BG);
    tft.setTextSize(2);
    tft.drawString("DEBUG INFO", 120, 12);

    hline(Y_DBG_T1   - 1);
    hline(Y_DBG_T2   - 1);
    hline(Y_DBG_GPIO - 1);
    hline(Y_DBG_DAC  - 1);
    hline(Y_DBG_ADS  - 1);
    hline(Y_DBG_TCAL - 1);
    hline(Y_DBG_FAN  - 1);
    hline(Y_DBG_VRAW - 1);
    hline(Y_DBG_IRAW - 1);

    drawLabel("TEMP 0",    4, Y_DBG_T0   + 6);
    drawLabel("TEMP 1",    4, Y_DBG_T1   + 6);
    drawLabel("TEMP 2",    4, Y_DBG_T2   + 6);
    drawLabel("UNREG_MON", 4, Y_DBG_GPIO + 6);
    drawLabel("DAC80501",  4, Y_DBG_DAC  + 6);
    drawLabel("ADS1115",   4, Y_DBG_ADS  + 6);
    drawLabel("TCAL9539",  4, Y_DBG_TCAL + 6);
    drawLabel("FAN PWM",   4, Y_DBG_FAN  + 6);
    drawLabel("V RAW",     4, Y_DBG_VRAW + 6);
    drawLabel("I RAW",     4, Y_DBG_IRAW + 6);
}

void updateDebugValues() {
    char buf[16];
    static const int yBase[] = { Y_DBG_T0, Y_DBG_T1, Y_DBG_T2 };
    for (uint8_t i = 0; i < 3; i++) {
        if (isnan(g_tempAll[i])) {
            drawValue("N/A", 236, yBase[i] + 6, 72, 16, COL_LABEL, 2);
        } else {
            snprintf(buf, sizeof(buf), "%.1fC", g_tempAll[i]);
            uint16_t col = (g_tempAll[i] >= 60.0f) ? COL_TEMP_HOT : COL_TEMP_OK;
            drawValue(buf, 236, yBase[i] + 6, 72, 16, col, 2);
        }
    }
    bool gpio13 = digitalRead(PIN_DOUT_13);
    drawValue(gpio13 ? "HIGH" : "LOW", 236, Y_DBG_GPIO + 6, 70, 16,
              gpio13 ? COL_LABEL : COL_CURR, 2);
    drawValue(g_dacOk  ? "OK" : "ERR", 236, Y_DBG_DAC  + 6, 50, 16,
              g_dacOk  ? COL_CURR : COL_TEMP_HOT, 2);
    drawValue(g_adsOk  ? "OK" : "ERR", 236, Y_DBG_ADS  + 6, 50, 16,
              g_adsOk  ? COL_CURR : COL_TEMP_HOT, 2);
    drawValue(g_tcalOk ? "OK" : "ERR", 236, Y_DBG_TCAL + 6, 50, 16,
              g_tcalOk ? COL_CURR : COL_TEMP_HOT, 2);
    snprintf(buf, sizeof(buf), "%3u%%", (uint16_t)g_fanPwm * 100 / 255);
    drawValue(buf, 236, Y_DBG_FAN + 6, 50, 16, COL_LABEL, 2);
    snprintf(buf, sizeof(buf), "%.3fV P%u", measV / ADS1115_V_SCALE, g_pgaV);
    drawValue(buf, 236, Y_DBG_VRAW + 6, 120, 16, COL_VOLT, 2);
    snprintf(buf, sizeof(buf), "%.3fV P%u", measA / ADS1115_I_SCALE, g_pgaA);
    drawValue(buf, 236, Y_DBG_IRAW + 6, 120, 16, COL_CURR, 2);
}

// ── Set-current editor ───────────────────────────────────────────────────────
// "X.XXX A" = 7 chars, textSize 2 (12×16 px), right-aligned at x=115, y=Y_SET+22
// "%5.1f A" = 7 chars: [hundreds][tens][ones][.][tenths][ ][A]
// char indices of selectable digits and their step values:
static const uint8_t kDigitChar[] = {0, 1, 2, 4};
static const float   kDigitStep[] = {100.0f, 10.0f, 1.0f, 0.1f};

// "%5.2f V" = 7 chars: [tens][ones][.][tenths][hundredths][ ][V]
static const uint8_t kUVPChar[] = {0, 1, 3, 4};
static const float   kUVPStep[] = {10.0f, 1.0f, 0.1f, 0.01f};

void drawSetCurrent() {
    char buf[8];
    snprintf(buf, sizeof(buf), "%5.1f A", setA);

    const int x0    = 115 - 7 * 12;  // left edge of 7-char string
    const int y     = Y_SET + 22;

    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    for (uint8_t i = 0; i < 7; i++) {
        bool hl = !g_editUVP && (i == kDigitChar[g_cursorPos]);
        int  cx = x0 + i * 12;
        tft.fillRect(cx, y, 12, 16, hl ? COL_SET : COL_BG);
        tft.setTextColor(hl ? COL_BG : COL_SET, hl ? COL_SET : COL_BG);
        char cs[2] = {buf[i], '\0'};
        tft.drawString(cs, cx, y);
    }
}

// "XX.XX V" = 7 chars, textSize 2, right-aligned at x=235, y=Y_SET+22
void drawSetUVP() {
    char buf[8];
    snprintf(buf, sizeof(buf), "%5.2f V", setUVP);

    const int x0 = 235 - 7 * 12;  // left edge of 7-char string
    const int y  = Y_SET + 22;

    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    for (uint8_t i = 0; i < 7; i++) {
        bool hl = g_editUVP && (i == kUVPChar[g_uvpCursorPos]);
        int  cx = x0 + i * 12;
        tft.fillRect(cx, y, 12, 16, hl ? COL_SET : COL_BG);
        tft.setTextColor(hl ? COL_BG : COL_SET, hl ? COL_SET : COL_BG);
        char cs[2] = {buf[i], '\0'};
        tft.drawString(cs, cx, y);
    }
}

// ── Dynamic values ────────────────────────────────────────────────────────────

void updateValues() {
    char buf[24];

    // Big three — textSize 3 (18×24 px/char), right-align at x=236
    snprintf(buf, sizeof(buf), "%.2f V", measV);
    drawValue(buf, 236, Y_VOLT + 34, 230, 26, COL_VOLT, 3);

    snprintf(buf, sizeof(buf), "%.3f A", measA);
    drawValue(buf, 236, Y_CURR + 34, 230, 26, COL_CURR, 3);

    snprintf(buf, sizeof(buf), "%.2f W", measW);
    drawValue(buf, 236, Y_PWR  + 34, 230, 26, COL_PWR,  3);

    // Split rows — textSize 2 (12×16 px/char)
    // Left half: right-align at x=115  |  Right half: right-align at x=235
    snprintf(buf, sizeof(buf), "%.3f Ah", measAh);
    drawValue(buf, 115, Y_CAP + 22, 110, 16, COL_CAP, 2);

    snprintf(buf, sizeof(buf), "%.2f Wh", measWh);
    drawValue(buf, 235, Y_CAP + 22, 110, 16, COL_CAP, 2);

    // Temp — left half (after "TEMP" label at x=4, 4×12=48 px)
    snprintf(buf, sizeof(buf), "%.1fC", tempC);
    uint16_t tc = (tempC >= 60.0f) ? COL_TEMP_HOT : COL_TEMP_OK;
    drawValue(buf, 115, Y_TEMP + 7, 62, 18, tc, 2);
    // Elapsed timer — right half; reset via P0.6 button
    unsigned long elapsed = (millis() - g_timerStart) / 1000UL;
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", elapsed / 3600, (elapsed % 3600) / 60, elapsed % 60);
    drawValue(buf, 235, Y_TEMP + 7, 114, 18, COL_CAP, 2);
}

// ── Input handling ────────────────────────────────────────────────────────────

static void handleInputs() {
    if (!g_intPending) return;
    g_intPending = false;
    Serial.println("interrupt!");
    uint16_t state   = ioExp.readPorts();
    uint16_t changed = state ^ g_prevState;

    // Rotary encoder on P0.1 (A) and P0.2 (B) — bits 1 and 2
    if (changed & 0x06) {
        static const int8_t enc[] = {0,+1,-1,0,-1,0,0,+1,+1,0,0,-1,0,-1,+1,0};
        static int8_t acc = 0;
        uint8_t prevAB = (g_prevState >> 1) & 0x03;
        uint8_t newAB  = (state >> 1) & 0x03;
        acc += enc[(prevAB << 2) | newAB];
        if (acc <= -4) {
            acc = 0;
            if (g_editUVP) { setUVP = constrain(setUVP + kUVPStep[g_uvpCursorPos], 0.0f, 99.99f); drawSetUVP(); }
            else           { setA   = constrain(setA   + kDigitStep[g_cursorPos],   0.0f, 100.0f); drawSetCurrent(); applySetCurrent(); }
        } else if (acc >= 4) {
            acc = 0;
            if (g_editUVP) { setUVP = constrain(setUVP - kUVPStep[g_uvpCursorPos], 0.0f, 99.99f); drawSetUVP(); }
            else           { setA   = constrain(setA   - kDigitStep[g_cursorPos],   0.0f, 100.0f); drawSetCurrent(); applySetCurrent(); }
        }
    }

    g_prevState = state;

    // Buttons — all pins except the encoder (P0.1 and P0.2)
    uint16_t btnChanged = changed & ~0x06u;
    for (uint8_t i = 0; i < 11; i++) {
        if (!(btnChanged & (1u << i))) continue;
        bool low = !(state & (1u << i));
        if (!low) continue;  // act on press only
        if (i == 0)  { g_editUVP = !g_editUVP; drawSetCurrent(); drawSetUVP(); }     // P0.0 = toggle UVP/current edit
        if (i == 3)  {                                                                // P0.3 = left
            if (g_editUVP) { if (g_uvpCursorPos > 0) { g_uvpCursorPos--; drawSetUVP(); } }
            else           { if (g_cursorPos    > 0) { g_cursorPos--;    drawSetCurrent(); } }
        }
        if (i == 4)  {                                                                // P0.4 = right
            if (g_editUVP) { if (g_uvpCursorPos < 3) { g_uvpCursorPos++; drawSetUVP(); } }
            else           { if (g_cursorPos    < 3) { g_cursorPos++;    drawSetCurrent(); } }
        }
        if (i == 5)  {                                                                // P0.5 = debug screen toggle
            g_debugScreen = !g_debugScreen;
            if (g_debugScreen) {
                g_dacOk  = dac80501Verify();
                g_adsOk  = ads1115Ping();
                g_tcalOk = ioExp.ping();
                drawDebugFrame();
                updateDebugValues();
            } else {
                drawFrame();
                updateValues();
                drawSetCurrent();
                drawSetUVP();
            }
        }
        if (i == 6)  { g_timerStart = millis(); measAh = 0.0f; measWh = 0.0f; }                                 // P0.6 = timer + capacity reset
        if (i == 7)  { g_extVSense = !g_extVSense; ioExp.writePin(9, g_extVSense); drawVSenseIndicator(); }   // P7 = V-sense: P11(bit9) LOW=INT, HIGH=EXT
        if (i == 8)  { g_loadOn = !g_loadOn; digitalWrite(PIN_DOUT_13, g_loadOn ? LOW : HIGH); if (g_loadOn) applySetCurrent(); else dac80501SetVoltage(0); drawHeader(); }  // P10 = on/off; GPIO13 active-low
    }
}

// ── Arduino entry points ─────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    // Wait for Serial before printing result (up to 2 s)
    for (uint32_t t = millis(); !Serial && (millis() - t < 2000); ) {}



    pinMode(PIN_DIN_11,  INPUT);
    pinMode(PIN_DOUT_2,  OUTPUT);
    pinMode(PIN_DOUT_3,  OUTPUT);
    analogWriteFreq(25000);        // 25 kHz — standard PC fan PWM
    analogWrite(PIN_DOUT_2, 0);   // fan 1 — off until temp control kicks in
    analogWrite(PIN_DOUT_3, 0);   // fan 2 — off until temp control kicks in
    pinMode(PIN_DOUT_8,  OUTPUT);
    pinMode(PIN_DOUT_9,  OUTPUT);
    pinMode(PIN_DOUT_10, OUTPUT);
    digitalWrite(PIN_DOUT_10, LOW);   // current-sense select: LOW = internal
    pinMode(PIN_DOUT_13, OUTPUT);
    digitalWrite(PIN_DOUT_13, HIGH);  // active-low: HIGH = load off at startup
    pinMode(PIN_BEEPER,  OUTPUT);
    digitalWrite(PIN_BEEPER, LOW);
    pinMode(PIN_ADC0,    INPUT);
    pinMode(PIN_ADC1,    INPUT);
    pinMode(PIN_ADC2,    INPUT);

    // I2C1 on GPIO14/15 for TCAL9539
    Wire1.setSDA(PIN_SDA);
    Wire1.setSCL(PIN_SCL);
    Wire1.begin();
    Wire1.setClock(1000000);

    delay(500);  // wait for devices to power up



    if (dac80501Verify()) {
        // REF-DIV=1, BUFF-GAIN=1: VREFIO=1.25 V (safe on 3.3 V supply), full-scale=2.5 V
        dac80501Write(DAC80501_REG_GAIN, 0x0101);
        uint16_t status = 0;
        dac80501Read(DAC80501_REG_STATUS, status);
        if (status & 0x0001) {
            Serial.println("DAC80501 REF-ALARM active");
        } else {
            Serial.println("DAC80501 OK");
            dac80501SetVoltage(0);  // load is off at startup
        }
    } else {
        Serial.println("DAC80501 NOT found");
    }

    if (ioExp.verify()) {
        Serial.println("TCAL9539 OK");
    } else if (ioExp.ping()) {
        Serial.println("TCAL9539 ACKs but REG_CONFIG_P0 != 0xFF — wrong device or reset state");
    } else {
        Serial.println("TCAL9539 NOT found (no I2C ACK at 0x74)");
    }

    if (ads1115Verify()) {
        Serial.println("ADS1115 OK");
    } else if (ads1115Ping()) {
        uint16_t cfg = 0;
        ads1115Read(ADS1115_REG_CONFIG, cfg);
        Serial.print("ADS1115 ACKs but config=0x");
        Serial.println(cfg, HEX);
    } else {
        Serial.println("ADS1115 NOT found (no I2C ACK at 0x49)");
    }

    // P0: pull-ups on all except P0.1/P0.2 (0xF9, hw pull-ups on encoder); all unmasked (0x00)
    // P1: pull-ups on P1.0 and P1.2 (0x05, skip P1.1 = output); P1.1 masked, P1.3+ masked (0xFA)
    ioExp.begin(0xF9, 0x05, 0x00, 0xFA);
    ioExp.setPinOutput(9, false);   // P11 = P1.1 = bit9, LOW = internal V-sense
    g_prevState = ioExp.readPorts();

    // INT is active-low open-drain; RP2040 internal pull-up keeps it high at rest
    pinMode(PIN_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_INT), onTCALInt, FALLING);

    g_dacOk  = dac80501Verify();
    g_adsOk  = ads1115Ping();
    g_tcalOk = ioExp.ping();

    tft.init();
    tft.setRotation(2);
    drawFrame();
    updateValues();
    drawSetCurrent();
    drawSetUVP();
}

void loop() {
    handleInputs();

    static unsigned long lastUpdate = 0;
    static unsigned long lastTempUpdate = 0;
    unsigned long now = millis();

    if (now - lastTempUpdate >= 1000) {
        lastTempUpdate = now;
        g_tempAll[0] = readMCP9700(PIN_ADC0);
        g_tempAll[1] = readMCP9700(PIN_ADC1);
        g_tempAll[2] = readMCP9700(PIN_ADC2);
        float tMax = NAN;
        for (float ti : g_tempAll) if (!isnan(ti) && (isnan(tMax) || ti > tMax)) tMax = ti;
        if (!isnan(tMax)) tempC = tMax;
        // off below 35°C, linear ramp to 100% at 55°C
        g_fanPwm = (uint8_t)constrain((tempC - 35.0f) / 20.0f * 255.0f, 0.0f, 255.0f);
        analogWrite(PIN_DOUT_2, g_fanPwm);
        analogWrite(PIN_DOUT_3, g_fanPwm);
        if (g_debugScreen) {
            g_dacOk  = dac80501Verify();
            g_adsOk  = ads1115Ping();
            g_tcalOk = ioExp.ping();
            updateDebugValues();
        }
    }

    if (now - lastUpdate >= 100) {
        float dtH = (now - lastUpdate) / 3600000.0f;
        lastUpdate = now;

        float v = ads1115ReadVoltage();   // ~16 ms blocking
        if (!isnan(v)) measV = v;
        handleInputs();                   // service any pending input before second conversion

        float a = ads1115ReadCurrent();   // ~16 ms blocking
        if (!isnan(a)) measA = a;
        handleInputs();                   // service any pending input before display update

        measW = measV * measA;
        digitalWrite(PIN_BEEPER, measW > 1000.0f ? HIGH : LOW);
        if (g_loadOn) {
            measAh += measA * dtH;
            measWh += measW * dtH;
        }

        if (!g_debugScreen) updateValues();
    }
}
