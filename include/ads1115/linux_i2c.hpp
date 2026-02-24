#pragma once

#include <ads1115/i2c_interface.hpp>
#include <cstdint>

namespace ads1115 {

/// Linux i2c-dev backend.
///
/// Opens /dev/i2c-{bus_num} and sets the slave address via ioctl I2C_SLAVE.
/// Requires i2c-dev kernel module and read/write access to /dev/i2c-N
/// (add user to the 'i2c' group, or run as root).
class LinuxI2CDevice : public II2CDevice {
public:
    /// Opens /dev/i2c-{bus_num} and sets slave address.
    /// Check isOpen() after construction.
    LinuxI2CDevice(int bus_num, uint8_t address);
    ~LinuxI2CDevice() override;

    LinuxI2CDevice(const LinuxI2CDevice&)            = delete;
    LinuxI2CDevice& operator=(const LinuxI2CDevice&) = delete;

    [[nodiscard]] bool isOpen() const { return fd_ >= 0; }

    bool write(const uint8_t* data, std::size_t len) override;
    bool read(uint8_t* data, std::size_t len) override;

private:
    int fd_{-1};
};

}  // namespace ads1115
