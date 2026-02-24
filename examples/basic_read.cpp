#include <ads1115/ads1115.hpp>
#include <ads1115/linux_i2c.hpp>

#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // Open I2C bus 1 (/dev/i2c-1), device address = ADDR pin → GND (0x48)
    ads1115::LinuxI2CDevice i2c{1, static_cast<uint8_t>(ads1115::Address::GND)};
    if (!i2c.isOpen()) {
        std::cerr << "Failed to open /dev/i2c-1\n";
        return 1;
    }

    ads1115::ADS1115 adc{i2c};

    if (adc.begin() != ads1115::Error::None) {
        std::cerr << "ADS1115 not responding\n";
        return 1;
    }

    adc.setPGA(ads1115::PGA::FSR_2048);
    adc.setDataRate(ads1115::DataRate::SPS_128);

    for (int i = 0; i < 10; ++i) {
        auto v = adc.readChannel(ads1115::Mux::SINGLE_0);
        if (v) {
            std::cout << "AIN0: " << *v << " V\n";
        } else {
            std::cerr << "Read failed\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    return 0;
}
