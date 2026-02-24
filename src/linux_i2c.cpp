#include <ads1115/linux_i2c.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <string>

namespace ads1115 {

LinuxI2CDevice::LinuxI2CDevice(int bus_num, uint8_t address) : bus_num_(bus_num) {
    const std::string path = "/dev/i2c-" + std::to_string(bus_num);

    fd_ = ::open(path.c_str(), O_RDWR);
    if (fd_ < 0) return;

    if (::ioctl(fd_, I2C_SLAVE, static_cast<int>(address)) < 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

LinuxI2CDevice::~LinuxI2CDevice() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool LinuxI2CDevice::write(const uint8_t* data, std::size_t len) {
    return ::write(fd_, data, len) == static_cast<ssize_t>(len);
}

bool LinuxI2CDevice::read(uint8_t* data, std::size_t len) {
    return ::read(fd_, data, len) == static_cast<ssize_t>(len);
}

bool LinuxI2CDevice::generalCallReset() {
    // Open a separate fd so the slave address on fd_ is not disturbed.
    const std::string path = "/dev/i2c-" + std::to_string(bus_num_);
    const int gc_fd = ::open(path.c_str(), O_RDWR);
    if (gc_fd < 0) return false;

    bool ok = false;
    if (::ioctl(gc_fd, I2C_SLAVE, 0x00) >= 0) {
        const uint8_t cmd = 0x06;
        ok = (::write(gc_fd, &cmd, 1) == 1);
    }
    ::close(gc_fd);
    return ok;
}

}  // namespace ads1115
