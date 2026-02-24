#include <ads1115/ads1115.hpp>
#include <ads1115/types.hpp>

#include <thread>
#include <chrono>

namespace ads1115 {

// Conversion time for a given data rate, with ~10% margin for oscillator tolerance.
static std::chrono::milliseconds conversionDelay(DataRate dr) {
    switch (dr) {
        case DataRate::SPS_8:   return std::chrono::milliseconds{138};
        case DataRate::SPS_16:  return std::chrono::milliseconds{69};
        case DataRate::SPS_32:  return std::chrono::milliseconds{35};
        case DataRate::SPS_64:  return std::chrono::milliseconds{18};
        case DataRate::SPS_128: return std::chrono::milliseconds{9};
        case DataRate::SPS_250: return std::chrono::milliseconds{5};
        case DataRate::SPS_475: return std::chrono::milliseconds{3};
        case DataRate::SPS_860: return std::chrono::milliseconds{2};
    }
    return std::chrono::milliseconds{10};
}

ADS1115::ADS1115(II2CDevice& i2c) : i2c_(i2c) {}

// ── Setup ─────────────────────────────────────────────────────────────────────

Error ADS1115::begin(Address addr) {
    address_ = addr;

    std::this_thread::sleep_for(std::chrono::microseconds{50});

    auto val = readRegister(reg::CONFIG);
    if (!val) return last_error_;  // I2CWriteFailed or I2CReadFailed already set

    if (*val != config::DEFAULT) return Error::UnexpectedDevice;

    return Error::None;
}

// ── Configuration ─────────────────────────────────────────────────────────────

void ADS1115::setMux(Mux mux)         { mux_  = mux;  }
void ADS1115::setPGA(PGA pga)         { pga_  = pga;  }
void ADS1115::setDataRate(DataRate dr) { dr_   = dr;   }
void ADS1115::setMode(Mode mode)       { mode_ = mode; }

// ── Single-shot ───────────────────────────────────────────────────────────────

bool ADS1115::startConversion() {
    return writeRegister(reg::CONFIG, buildConfig() | config::OS_START);
}

bool ADS1115::waitForConversion(std::chrono::milliseconds timeout) {
    const auto deadline      = std::chrono::steady_clock::now() + timeout;
    const auto poll_interval = conversionDelay(dr_);

    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(poll_interval);

        auto val = readRegister(reg::CONFIG);
        if (!val) return false;  // last_error_ already set by readRegister

        if (*val & config::OS_IDLE) return true;
    }

    last_error_ = Error::ConversionTimeout;
    return false;
}

std::optional<int16_t> ADS1115::readRaw() {
    auto val = readRegister(reg::CONVERSION);
    if (!val) return std::nullopt;
    return static_cast<int16_t>(*val);
}

std::optional<float> ADS1115::readVoltage() {
    auto raw = readRaw();
    if (!raw) return std::nullopt;
    return toVoltage(*raw, pga_);
}

std::optional<float> ADS1115::readChannel(Mux channel) {
    setMux(channel);
    if (!startConversion())     return std::nullopt;
    if (!waitForConversion())   return std::nullopt;
    return readVoltage();
}

// ── Continuous mode ───────────────────────────────────────────────────────────

std::optional<int16_t> ADS1115::readRawContinuous() {
    return readRaw();  // conversion reg always holds latest result in continuous mode
}

// ── Comparator ───────────────────────────────────────────────────────────────

bool ADS1115::setComparator(CompMode mode, CompPolarity polarity, CompLatch latch,
                             CompQueue queue, int16_t lo_thresh, int16_t hi_thresh) {
    if (hi_thresh <= lo_thresh) {
        last_error_ = Error::InvalidArgument;
        return false;
    }

    if (!writeRegister(reg::LO_THRESH, static_cast<uint16_t>(lo_thresh))) return false;
    if (!writeRegister(reg::HI_THRESH, static_cast<uint16_t>(hi_thresh))) return false;

    comp_mode_     = mode;
    comp_polarity_ = polarity;
    comp_latch_    = latch;
    comp_queue_    = queue;

    return writeRegister(reg::CONFIG, buildConfig());
}

bool ADS1115::disableComparator() {
    comp_mode_     = CompMode::TRADITIONAL;
    comp_polarity_ = CompPolarity::ACTIVE_LOW;
    comp_latch_    = CompLatch::NON_LATCHING;
    comp_queue_    = CompQueue::DISABLED;

    return writeRegister(reg::CONFIG, buildConfig());
}

bool ADS1115::enableConversionReady(CompQueue queue) {
    if (queue == CompQueue::DISABLED) {
        last_error_ = Error::InvalidArgument;
        return false;
    }

    // Datasheet §6: Hi_thresh MSB=1 (0x8000), Lo_thresh MSB=0 (0x0000).
    // ALERT/RDY then pulses ~8µs after each conversion (continuous) or
    // asserts low after conversion completes (single-shot).
    if (!writeRegister(reg::HI_THRESH, 0x8000)) return false;
    if (!writeRegister(reg::LO_THRESH, 0x0000)) return false;

    comp_queue_ = queue;
    return writeRegister(reg::CONFIG, buildConfig());
}

bool ADS1115::setThresholds(int16_t lo, int16_t hi) {
    if (hi <= lo) {
        last_error_ = Error::InvalidArgument;
        return false;
    }
    if (!writeRegister(reg::LO_THRESH, static_cast<uint16_t>(lo))) return false;
    if (!writeRegister(reg::HI_THRESH, static_cast<uint16_t>(hi))) return false;
    return true;
}

// ── Utility ───────────────────────────────────────────────────────────────────

bool ADS1115::reset() {
    return i2c_.generalCallReset();
}

float ADS1115::toVoltage(int16_t raw, PGA pga) {
    const uint8_t idx = static_cast<uint8_t>(static_cast<uint16_t>(pga) >> 9);
    return raw * LSB_UV[idx] * 1e-6f;
}

// ── Private helpers ───────────────────────────────────────────────────────────

bool ADS1115::writeRegister(uint8_t reg_addr, uint16_t value) {
    const uint8_t buf[3] = {
        reg_addr,
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF),
    };
    if (!i2c_.write(buf, sizeof(buf))) {
        last_error_ = Error::I2CWriteFailed;
        return false;
    }
    return true;
}

std::optional<uint16_t> ADS1115::readRegister(uint8_t reg_addr) {
    // Set address pointer
    if (!i2c_.write(&reg_addr, 1)) {
        last_error_ = Error::I2CWriteFailed;
        return std::nullopt;
    }
    // Read two bytes MSB-first
    uint8_t buf[2];
    if (!i2c_.read(buf, sizeof(buf))) {
        last_error_ = Error::I2CReadFailed;
        return std::nullopt;
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
}

uint16_t ADS1115::buildConfig() const {
    return static_cast<uint16_t>(mux_)
         | static_cast<uint16_t>(pga_)
         | static_cast<uint16_t>(mode_)
         | static_cast<uint16_t>(dr_)
         | static_cast<uint16_t>(comp_mode_)
         | static_cast<uint16_t>(comp_polarity_)
         | static_cast<uint16_t>(comp_latch_)
         | static_cast<uint16_t>(comp_queue_);
}

}  // namespace ads1115
