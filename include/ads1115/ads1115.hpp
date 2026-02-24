#pragma once

#include <ads1115/types.hpp>
#include <ads1115/i2c_interface.hpp>

#include <chrono>
#include <cstdint>
#include <optional>

namespace ads1115 {

class ADS1115 {
public:
    explicit ADS1115(II2CDevice& i2c);

    // ── Setup ────────────────────────────────────────────────────────────────

    /// Open the device at the given I2C address.
    /// Waits 50 µs for power-up, then verifies comms by reading the config reg.
    [[nodiscard]] Error begin(Address addr = Address::GND);

    // ── Configuration setters (update local state; no I2C traffic) ───────────

    void setMux(Mux mux);
    void setPGA(PGA pga);
    void setDataRate(DataRate dr);
    void setMode(Mode mode);

    // ── Single-shot operation ────────────────────────────────────────────────

    /// Write config register with OS=1 to trigger one conversion.
    /// Device must be in SINGLE_SHOT mode.
    [[nodiscard]] bool startConversion();

    /// Busy-wait (with sleep) until OS bit = 1 (idle) or timeout elapses.
    [[nodiscard]] bool waitForConversion(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    /// Read the 16-bit signed raw conversion result.
    [[nodiscard]] std::optional<int16_t> readRaw();

    /// Read voltage in volts using the current PGA setting.
    [[nodiscard]] std::optional<float> readVoltage();

    /// Convenience: set mux channel, trigger, wait, read. Blocking.
    [[nodiscard]] std::optional<float> readChannel(Mux channel);

    // ── Continuous mode ──────────────────────────────────────────────────────

    /// Read the most recently completed conversion (continuous mode).
    /// Does not trigger a new conversion.
    [[nodiscard]] std::optional<int16_t> readRawContinuous();

    // ── Comparator ───────────────────────────────────────────────────────────

    /// Configure and enable the threshold comparator.
    /// lo_thresh / hi_thresh are in raw ADC codes (same scale as conversion reg).
    /// hi_thresh must be > lo_thresh.
    [[nodiscard]] bool setComparator(
        CompMode     mode,
        CompPolarity polarity,
        CompLatch    latch,
        CompQueue    queue,
        int16_t      lo_thresh,
        int16_t      hi_thresh);

    /// Disable comparator (COMP_QUE = 0b11, ALERT/RDY = high-Z).
    [[nodiscard]] bool disableComparator();

    /// Configure ALERT/RDY pin as conversion-ready output.
    /// Writes special threshold values (Hi=0x8000, Lo=0x0000) per datasheet §6.
    [[nodiscard]] bool enableConversionReady(CompQueue queue = CompQueue::AFTER_1);

    /// Write Lo_thresh and Hi_thresh registers directly (raw ADC codes).
    [[nodiscard]] bool setThresholds(int16_t lo, int16_t hi);

    // ── Utility ──────────────────────────────────────────────────────────────

    /// Issue I2C general call reset (0x00, 0x06). Resets all devices on bus.
    [[nodiscard]] bool reset();

    [[nodiscard]] Error lastError() const { return last_error_; }

    /// Pure conversion: raw ADC code → voltage (V) for the given PGA setting.
    [[nodiscard]] static float toVoltage(int16_t raw, PGA pga);

private:
    [[nodiscard]] bool                    writeRegister(uint8_t reg, uint16_t value);
    [[nodiscard]] std::optional<uint16_t> readRegister(uint8_t reg);

    /// Assemble the 16-bit config register from current state (without OS bit).
    [[nodiscard]] uint16_t buildConfig() const;

    II2CDevice& i2c_;

    Address  address_{Address::GND};
    Mux      mux_{Mux::SINGLE_0};
    PGA      pga_{PGA::FSR_2048};
    DataRate dr_{DataRate::SPS_128};
    Mode     mode_{Mode::SINGLE_SHOT};
    Error    last_error_{Error::None};
};

}  // namespace ads1115
