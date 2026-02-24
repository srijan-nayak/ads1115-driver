#pragma once

#include <cstddef>
#include <cstdint>

namespace ads1115 {

/// Minimal I2C transport abstraction.
///
/// The slave address is set at construction time (e.g. via ioctl I2C_SLAVE)
/// and is not part of the payload — write()/read() deal only with data bytes.
///
/// Implementations:
///   - LinuxI2CDevice  (include/ads1115/linux_i2c.hpp) — production
///   - MockI2CDevice   (tests/mock_i2c.hpp)            — unit tests
class II2CDevice {
public:
    virtual ~II2CDevice() = default;

    /// Write `len` bytes from `data` to the device.
    /// Returns false on error.
    virtual bool write(const uint8_t* data, std::size_t len) = 0;

    /// Read `len` bytes from the device into `data`.
    /// Returns false on error.
    virtual bool read(uint8_t* data, std::size_t len) = 0;
};

}  // namespace ads1115
