#pragma once

#include <ads1115/i2c_interface.hpp>

#include <cstdint>
#include <initializer_list>
#include <queue>
#include <vector>

namespace ads1115::test {

/// Simple record-and-replay mock for unit tests.
///
/// - All bytes passed to write() are recorded in writes().
/// - Bytes to be returned by read() are enqueued with enqueueRead().
/// - write_ok / read_ok can be set to false to simulate I2C errors.
class MockI2CDevice : public II2CDevice {
public:
    // ── Replay ───────────────────────────────────────────────────────────────

    /// Queue bytes that the next read() call will return.
    void enqueueRead(std::initializer_list<uint8_t> bytes) {
        read_queue_.push(std::vector<uint8_t>(bytes));
    }

    // ── Inspection ───────────────────────────────────────────────────────────

    /// All write() call payloads in call order.
    const std::vector<std::vector<uint8_t>>& writes() const { return writes_; }

    void clearWrites() { writes_.clear(); }

    // ── Fault injection ───────────────────────────────────────────────────────

    bool write_ok{true};
    bool read_ok{true};
    bool reset_ok{true};

    // ── II2CDevice ────────────────────────────────────────────────────────────

    bool write(const uint8_t* data, std::size_t len) override {
        if (write_ok) writes_.emplace_back(data, data + len);
        return write_ok;
    }

    bool generalCallReset() override {
        if (reset_ok) reset_call_count_++;
        return reset_ok;
    }

    int resetCallCount() const { return reset_call_count_; }

    bool read(uint8_t* data, std::size_t len) override {
        if (!read_ok || read_queue_.empty()) return false;
        auto& front = read_queue_.front();
        if (front.size() < len) return false;
        std::copy(front.begin(), front.begin() + static_cast<std::ptrdiff_t>(len), data);
        read_queue_.pop();
        return true;
    }

private:
    std::vector<std::vector<uint8_t>> writes_;
    std::queue<std::vector<uint8_t>>  read_queue_;
    int reset_call_count_{0};
};

}  // namespace ads1115::test
