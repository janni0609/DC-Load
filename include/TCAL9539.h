#pragma once
#include <Wire.h>

// TCAL9539 16-bit I2C I/O expander (I2C addr 0x74..0x77 via A0/A1)
class TCAL9539 {
public:
    explicit TCAL9539(TwoWire &wire, uint8_t addr = 0x74)
        : _wire(wire), _addr(addr) {}

    // Returns true if the device ACKs on the I2C bus.
    bool ping() {
        _wire.beginTransmission(_addr);
        return _wire.endTransmission() == 0;
    }

    // Verify ping AND that REG_CONFIG_P0 reads its power-on default (0xFF).
    // Distinguishes a real TCAL9539 from address collisions with other devices.
    bool verify() {
        return ping() && (readReg(REG_CONFIG_P0) == 0xFF);
    }

    // port0/1Pullups : bitmask of pins that get the internal pull-up enabled.
    // intMask0/1     : bits set to 1 suppress INT for that pin (0 = fires INT).
    // Call after Wire has been started.
    void begin(uint8_t port0Pullups = 0x00, uint8_t port1Pullups = 0x00,
               uint8_t intMask0 = 0xFF,    uint8_t intMask1 = 0xFF) {
        if (port0Pullups) {
            writeReg(REG_PUPD_EN_P0,  readReg(REG_PUPD_EN_P0)  | port0Pullups);
            writeReg(REG_PUPD_SEL_P0, readReg(REG_PUPD_SEL_P0) | port0Pullups);
        }
        if (port1Pullups) {
            writeReg(REG_PUPD_EN_P1,  readReg(REG_PUPD_EN_P1)  | port1Pullups);
            writeReg(REG_PUPD_SEL_P1, readReg(REG_PUPD_SEL_P1) | port1Pullups);
        }
        writeReg(REG_INTMASK_P0, intMask0);
        writeReg(REG_INTMASK_P1, intMask1);
        // REG_CONFIG_P0/P1 default to 0xFF (all inputs) — no change needed
    }

    // Read both ports in one I2C transaction (auto-increments from 0x00 to 0x01).
    // Returns P1 in high byte, P0 in low byte. Also clears the INT line.
    uint16_t readPorts() {
        _wire.beginTransmission(_addr);
        _wire.write(REG_INPUT_P0);
        _wire.endTransmission(false);
        _wire.requestFrom(_addr, (uint8_t)2);
        uint8_t p0 = _wire.available() ? _wire.read() : 0xFF;
        uint8_t p1 = _wire.available() ? _wire.read() : 0xFF;
        return ((uint16_t)p1 << 8) | p0;
    }

    uint8_t readPort0() { return readReg(REG_INPUT_P0); }
    uint8_t readPort1() { return readReg(REG_INPUT_P1); }

    // Configure pin 0-15 as output with initial level (0-7 = P0.x, 8-15 = P1.x).
    void setPinOutput(uint8_t pin, bool level) {
        uint8_t regOut = (pin < 8) ? REG_OUTPUT_P0 : REG_OUTPUT_P1;
        uint8_t regCfg = (pin < 8) ? REG_CONFIG_P0 : REG_CONFIG_P1;
        uint8_t bit    = 1u << ((pin < 8) ? pin : (pin - 8));
        uint8_t out    = readReg(regOut);
        writeReg(regOut, level ? (out | bit) : (out & ~bit));
        writeReg(regCfg, readReg(regCfg) & ~bit);
    }

    // Drive an already-configured output pin high or low.
    void writePin(uint8_t pin, bool level) {
        uint8_t regOut = (pin < 8) ? REG_OUTPUT_P0 : REG_OUTPUT_P1;
        uint8_t bit    = 1u << ((pin < 8) ? pin : (pin - 8));
        uint8_t out    = readReg(regOut);
        writeReg(regOut, level ? (out | bit) : (out & ~bit));
    }

private:
    static constexpr uint8_t REG_INPUT_P0    = 0x00;
    static constexpr uint8_t REG_INPUT_P1    = 0x01;
    static constexpr uint8_t REG_OUTPUT_P0   = 0x02;
    static constexpr uint8_t REG_OUTPUT_P1   = 0x03;
    static constexpr uint8_t REG_CONFIG_P0   = 0x06;
    static constexpr uint8_t REG_CONFIG_P1   = 0x07;
    static constexpr uint8_t REG_PUPD_EN_P0  = 0x46;  // pull enable (default 0x00)
    static constexpr uint8_t REG_PUPD_EN_P1  = 0x47;
    static constexpr uint8_t REG_PUPD_SEL_P0 = 0x48;  // 1=pull-up, 0=pull-down (default 0xFF)
    static constexpr uint8_t REG_PUPD_SEL_P1 = 0x49;
    static constexpr uint8_t REG_INTMASK_P0  = 0x4A;  // 0=unmasked (fires INT), 1=masked (default 0x00)
    static constexpr uint8_t REG_INTMASK_P1  = 0x4B;

    TwoWire &_wire;
    uint8_t  _addr;

    uint8_t readReg(uint8_t reg) {
        _wire.beginTransmission(_addr);
        _wire.write(reg);
        _wire.endTransmission(false);
        _wire.requestFrom(_addr, (uint8_t)1);
        return _wire.available() ? _wire.read() : 0xFF;
    }

    void writeReg(uint8_t reg, uint8_t val) {
        _wire.beginTransmission(_addr);
        _wire.write(reg);
        _wire.write(val);
        _wire.endTransmission();
    }
};
