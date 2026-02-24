#pragma once

#include <cstdint>

namespace ads1115 {

// ── I2C Addresses ─────────────────────────────────────────────────────────────
enum class Address : uint8_t {
    GND = 0x48,
    VDD = 0x49,
    SDA = 0x4A,
    SCL = 0x4B,
};

// ── Input Multiplexer (bits [14:12], pre-shifted) ────────────────────────────
enum class Mux : uint16_t {
    DIFF_0_1 = 0x0000,  // AIN0 − AIN1 (default)
    DIFF_0_3 = 0x1000,  // AIN0 − AIN3
    DIFF_1_3 = 0x2000,  // AIN1 − AIN3
    DIFF_2_3 = 0x3000,  // AIN2 − AIN3
    SINGLE_0 = 0x4000,  // AIN0 vs GND
    SINGLE_1 = 0x5000,  // AIN1 vs GND
    SINGLE_2 = 0x6000,  // AIN2 vs GND
    SINGLE_3 = 0x7000,  // AIN3 vs GND
};

// ── Programmable Gain Amplifier (bits [11:9], pre-shifted) ───────────────────
enum class PGA : uint16_t {
    FSR_6144 = 0x0000,  // ±6.144V  187.5 µV/LSB  ⚠ do not exceed VDD+0.3V
    FSR_4096 = 0x0200,  // ±4.096V  125.0 µV/LSB  ⚠ same warning
    FSR_2048 = 0x0400,  // ±2.048V   62.5 µV/LSB  (default)
    FSR_1024 = 0x0600,  // ±1.024V   31.25 µV/LSB
    FSR_0512 = 0x0800,  // ±0.512V   15.625 µV/LSB
    FSR_0256 = 0x0A00,  // ±0.256V    7.8125 µV/LSB
};

// ── Data Rate (bits [7:5], pre-shifted) ──────────────────────────────────────
enum class DataRate : uint16_t {
    SPS_8   = 0x0000,  // 125 ms / sample
    SPS_16  = 0x0020,
    SPS_32  = 0x0040,
    SPS_64  = 0x0060,
    SPS_128 = 0x0080,  // 7.8 ms / sample (default)
    SPS_250 = 0x00A0,
    SPS_475 = 0x00C0,
    SPS_860 = 0x00E0,  // 1.16 ms / sample
};

// ── Conversion Mode (bit 8) ───────────────────────────────────────────────────
enum class Mode : uint16_t {
    CONTINUOUS  = 0x0000,
    SINGLE_SHOT = 0x0100,  // (default)
};

// ── Comparator ───────────────────────────────────────────────────────────────
enum class CompMode : uint16_t {
    TRADITIONAL = 0x0000,  // hysteresis (default)
    WINDOW      = 0x0010,
};

enum class CompPolarity : uint16_t {
    ACTIVE_LOW  = 0x0000,  // (default)
    ACTIVE_HIGH = 0x0008,
};

enum class CompLatch : uint16_t {
    NON_LATCHING = 0x0000,  // (default)
    LATCHING     = 0x0004,
};

enum class CompQueue : uint16_t {
    AFTER_1  = 0x0000,
    AFTER_2  = 0x0001,
    AFTER_4  = 0x0002,
    DISABLED = 0x0003,  // ALERT/RDY = high-Z (default)
};

// ── Error codes ──────────────────────────────────────────────────────────────
enum class Error {
    None = 0,
    I2COpenFailed,
    I2CSetAddrFailed,
    I2CWriteFailed,
    I2CReadFailed,
    ConversionTimeout,
    InvalidArgument,
};

// ── Register addresses ────────────────────────────────────────────────────────
namespace reg {
    constexpr uint8_t CONVERSION = 0x00;
    constexpr uint8_t CONFIG     = 0x01;
    constexpr uint8_t LO_THRESH  = 0x02;
    constexpr uint8_t HI_THRESH  = 0x03;
}  // namespace reg

// ── Config register bit masks ─────────────────────────────────────────────────
namespace config {
    constexpr uint16_t OS_START      = 0x8000;  // write: start single-shot
    constexpr uint16_t OS_IDLE       = 0x8000;  // read: device not converting
    constexpr uint16_t COMP_DISABLED = 0x0003;  // COMP_QUE = 11 (high-Z)
    constexpr uint16_t DEFAULT       = 0x8583;
}  // namespace config

// ── LSB sizes (µV) indexed by PGA[2:0] = bits[11:9] >> 9 ─────────────────────
// V = (int16_t)raw * LSB_UV[pga_idx] * 1e-6f
constexpr float LSB_UV[] = {
    187.5f,   // 0 → ±6.144V
    125.0f,   // 1 → ±4.096V
    62.5f,    // 2 → ±2.048V (default)
    31.25f,   // 3 → ±1.024V
    15.625f,  // 4 → ±0.512V
    7.8125f,  // 5 → ±0.256V
    7.8125f,  // 6 → ±0.256V
    7.8125f,  // 7 → ±0.256V
};

}  // namespace ads1115
