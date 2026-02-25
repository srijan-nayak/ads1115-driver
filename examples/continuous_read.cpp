#include <ads1115/ads1115.hpp>
#include <ads1115/linux_i2c.hpp>

#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

static volatile bool running = true;
static void onSignal(int) { running = false; }

int main() {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

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

    // Configure for continuous conversion on AIN0 at 128 SPS, ±2.048 V FSR.
    adc.setPGA(ads1115::PGA::FSR_2048);
    adc.setDataRate(ads1115::DataRate::SPS_128);
    adc.setMux(ads1115::Mux::SINGLE_0);
    adc.setMode(ads1115::Mode::CONTINUOUS);

    // Writing config with MODE=0 starts continuous conversions immediately.
    if (!adc.startConversion()) {
        std::cerr << "Failed to start continuous conversion\n";
        return 1;
    }

    // At 128 SPS a new result is ready every ~7.8 ms.
    // Sleep slightly longer to ensure each read returns a fresh sample.
    constexpr auto sample_interval = std::chrono::milliseconds{10};

    std::cout << "Sampling AIN0 continuously at 128 SPS. Press Ctrl-C to stop.\n";

    while (running) {
        std::this_thread::sleep_for(sample_interval);

        const auto raw = adc.readRawContinuous();
        if (!raw) {
            std::cerr << "Read failed\n";
            break;
        }

        const float voltage = ads1115::ADS1115::toVoltage(*raw, ads1115::PGA::FSR_2048);
        std::cout << "AIN0: " << voltage << " V  (raw=" << *raw << ")\n";
    }

    // Return device to low-power idle state before exiting.
    (void)adc.powerDown();

    return 0;
}
