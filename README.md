# ADS1115 C++17 Driver

A production-quality C++17 driver for the Texas Instruments [ADS1115](https://www.ti.com/product/ADS1115) 16-bit I²C ADC, targeting Linux platforms (Raspberry Pi and similar).

## Features

- Single-shot and continuous conversion modes
- All four input channels — single-ended and differential
- Full PGA control (±6.144V to ±0.256V FSR)
- Data rates from 8 to 860 SPS
- Threshold comparator with traditional and window modes
- Conversion-ready output via ALERT/RDY pin
- Non-blocking `isConversionReady()` for interrupt/event-loop patterns
- No exceptions, no dynamic allocation
- Testable via injected I²C interface — unit tests run on the host without hardware
- CMake install target with `find_package` support

## Requirements

- Linux with the `i2c-dev` kernel module loaded (`modprobe i2c-dev`)
- Read/write access to `/dev/i2c-N` (add user to `i2c` group, or run as root)
- CMake ≥ 3.16
- C++17 compiler (GCC or Clang)

## Building

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build          # run unit tests (no hardware required)
```

To build the examples (requires hardware):

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release -DADS1115_BUILD_EXAMPLES=ON
cmake --build build
```

## Installation

```sh
cmake --install build --prefix /usr/local
```

This installs:
- `lib/libads1115.a`
- `include/ads1115/*.hpp`
- `lib/cmake/ads1115/` — CMake package files for `find_package`

### Installing on Raspberry Pi

```sh
# 1. Enable I2C (if not already done)
sudo raspi-config   # Interface Options → I2C → Enable
sudo reboot

# 2. Install build tools
sudo apt update
sudo apt install cmake g++ git

# 3. Add your user to the i2c group (avoids needing sudo for /dev/i2c-*)
sudo usermod -aG i2c $USER
newgrp i2c

# 4. Clone, build, and install
git clone <repo-url>
cd ads1115-driver
cmake -B build -DCMAKE_BUILD_TYPE=Release -DADS1115_BUILD_TESTS=OFF
cmake --build build
sudo cmake --install build
```

> Disabling tests (`-DADS1115_BUILD_TESTS=OFF`) skips the Catch2 network fetch
> and significantly speeds up the configure step on the Pi.

To consume from another CMake project:

```cmake
find_package(ads1115 REQUIRED)
target_link_libraries(my_app PRIVATE ads1115::ads1115)
```

## Hardware Setup

### I²C Address

Connect the ADDR pin to select the device address:

| ADDR pin | Address |
|----------|---------|
| GND      | 0x48    |
| VDD      | 0x49    |
| SDA      | 0x4A    |
| SCL      | 0x4B    |

### Wiring Notes

- 1 kΩ–10 kΩ pullup resistors on SDA and SCL
- 100 nF decoupling capacitor close to VDD
- Do not apply more than VDD + 0.3 V to any analog input
- ALERT/RDY is open-drain — add a pullup to VDD when used

## Usage

### Single-shot read (blocking)

```cpp
#include <ads1115/ads1115.hpp>
#include <ads1115/linux_i2c.hpp>
#include <iostream>

int main() {
    ads1115::LinuxI2CDevice i2c{1, static_cast<uint8_t>(ads1115::Address::GND)};
    if (!i2c.isOpen()) { /* handle error */ }

    ads1115::ADS1115 adc{i2c};
    if (adc.begin() != ads1115::Error::None) { /* handle error */ }

    adc.setPGA(ads1115::PGA::FSR_2048);       // ±2.048 V
    adc.setDataRate(ads1115::DataRate::SPS_128);

    auto voltage = adc.readChannel(ads1115::Mux::SINGLE_0);
    if (voltage) {
        std::cout << "AIN0: " << *voltage << " V\n";
    }
}
```

### Non-blocking read (interrupt / event-loop pattern)

```cpp
adc.setPGA(ads1115::PGA::FSR_2048);
adc.setDataRate(ads1115::DataRate::SPS_128);

adc.startConversion();

// ... do other work ...

auto ready = adc.isConversionReady();
if (ready && *ready) {
    auto raw = adc.readRaw();
    float v  = ads1115::ADS1115::toVoltage(*raw, ads1115::PGA::FSR_2048);
}
```

### Continuous mode

```cpp
adc.setMux(ads1115::Mux::SINGLE_0);
adc.setPGA(ads1115::PGA::FSR_2048);
adc.setDataRate(ads1115::DataRate::SPS_128);
adc.setMode(ads1115::Mode::CONTINUOUS);
adc.startConversion();  // starts continuous sampling

// Read latest result at any time
auto raw = adc.readRawContinuous();
float v  = ads1115::ADS1115::toVoltage(*raw, ads1115::PGA::FSR_2048);

// Stop when done
adc.powerDown();
```

### Threshold comparator

```cpp
// Assert ALERT/RDY (active-low) when AIN0 exceeds 1.5 V
// Thresholds are in raw ADC codes for the current PGA setting
// ±2.048 V → 62.5 µV/LSB → 1.5 V ≈ 24000 counts
adc.setComparator(
    ads1115::CompMode::TRADITIONAL,
    ads1115::CompPolarity::ACTIVE_LOW,
    ads1115::CompLatch::NON_LATCHING,
    ads1115::CompQueue::AFTER_1,
    /*lo=*/0, /*hi=*/24000);
```

### Conversion-ready via ALERT/RDY

```cpp
// ALERT/RDY pulses ~8 µs after each conversion completes.
// Wire it to a GPIO and trigger an interrupt; read in the ISR/callback.
adc.enableConversionReady(ads1115::CompQueue::AFTER_1);
```

## API Reference

### Setup

| Method | Description |
|--------|-------------|
| `begin()` | Initialise driver; verify device presence. Must be called first. |

### Configuration (no I²C traffic)

| Method | Description |
|--------|-------------|
| `setMux(Mux)` | Select input channel / differential pair |
| `setPGA(PGA)` | Set full-scale range |
| `setDataRate(DataRate)` | Set samples per second |
| `setMode(Mode)` | `SINGLE_SHOT` or `CONTINUOUS` |

### Reading

| Method | Returns | Description |
|--------|---------|-------------|
| `readChannel(Mux)` | `optional<float>` | Blocking: trigger, wait, return voltage |
| `startConversion()` | `bool` | Trigger one conversion (single-shot) |
| `waitForConversion()` | `bool` | Block until OS bit set or timeout |
| `isConversionReady()` | `optional<bool>` | Non-blocking OS bit poll |
| `readRaw()` | `optional<int16_t>` | Read conversion register |
| `readVoltage()` | `optional<float>` | `readRaw()` converted to volts |
| `readRawContinuous()` | `optional<int16_t>` | Latest result in continuous mode |
| `toVoltage(raw, PGA)` | `float` | Static: raw code → volts |

### Comparator

| Method | Description |
|--------|-------------|
| `setComparator(...)` | Configure and enable threshold comparator |
| `disableComparator()` | Disable ALERT/RDY (high-Z) |
| `enableConversionReady(CompQueue)` | Use ALERT/RDY as conversion-ready signal |
| `setThresholds(lo, hi)` | Write threshold registers directly |

### Utility

| Method | Description |
|--------|-------------|
| `powerDown()` | Enter low-power idle (stops continuous mode) |
| `reset()` | I²C general call reset (all devices on bus) |
| `lastError()` | Return last `Error` code |

### PGA / FSR Reference

| `PGA` | FSR | LSB |
|-------|-----|-----|
| `FSR_6144` | ±6.144 V | 187.5 µV |
| `FSR_4096` | ±4.096 V | 125.0 µV |
| `FSR_2048` | ±2.048 V | 62.5 µV (default) |
| `FSR_1024` | ±1.024 V | 31.25 µV |
| `FSR_0512` | ±0.512 V | 15.625 µV |
| `FSR_0256` | ±0.256 V | 7.8125 µV |

> ⚠️ FSR_6144 and FSR_4096 can represent voltages above VDD. Never apply more than VDD + 0.3 V to any analog input pin.

## Testing

Unit tests run entirely on the host using a mock I²C backend — no hardware required:

```sh
ctest --test-dir build
```

To write tests against the driver inject a `MockI2CDevice` from `tests/mock_i2c.hpp` in place of `LinuxI2CDevice`.

## License

[MIT](LICENSE)
