# ADS1115 C++17 Driver — Project Guide

## Project Overview

A production-quality C++17 driver for the Texas Instruments ADS1115 16-bit I2C ADC, targeting Linux platforms (Raspberry Pi and similar). Full datasheet reference is in `ADS1115_driver_reference.md`.

**Device summary:** 16-bit ADC, 4-ch single-ended / 2-ch differential, I2C, programmable gain amplifier (PGA), built-in comparator, 8–860 SPS.

## Build System

CMake (minimum 3.16). Standard layout:
```
ads1115-driver/
  CMakeLists.txt
  cmake/                # package config template (ads1115Config.cmake.in)
  include/ads1115/      # public headers (all installed)
    ads1115.hpp         # main driver class
    types.hpp           # enums, constants, Error, LSB table
    i2c_interface.hpp   # II2CDevice abstract interface
    linux_i2c.hpp       # LinuxI2CDevice — Linux i2c-dev backend (public, users instantiate it)
  src/                  # implementation
    ads1115.cpp
    linux_i2c.cpp
  tests/                # unit tests (host, no hardware required)
  examples/             # usage examples for RPi
```

Build commands:
```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
cmake --install build --prefix /usr/local
```

## Language & Style

- **C++17** — use `std::optional`, `std::byte`, `[[nodiscard]]`, structured bindings, `if constexpr` where appropriate
- No exceptions in the core driver — use `std::optional` or a simple `Result<T, Error>` type for error returns
- No dynamic allocation in the core driver (no `new`/`malloc`)
- RAII for the I2C file descriptor
- Strong types via `enum class` (all register enums are already defined in the reference doc)
- `constexpr` for all register constants and lookup tables

## Linux I2C Interface

Use the Linux kernel `i2c-dev` interface via `ioctl`:
- Include `<linux/i2c-dev.h>` and `<sys/ioctl.h>`
- Open `/dev/i2c-N`, set slave address with `ioctl(fd, I2C_SLAVE, addr)`
- `write()` / `read()` for raw byte transfers
- Do **not** depend on `libi2c` (wiringPi, pigpio, etc.) — keep it portable to any Linux i2c-dev device

### Register Access Pattern
- Write register: 3 bytes — pointer byte, MSB, LSB (big-endian)
- Read register: write pointer byte, then read 2 bytes MSB-first; combine as `(msb << 8) | lsb`

## Key Design Decisions

- **No exceptions** — all I2C errors return via `std::optional<T>` or an error enum
- **Blocking and non-blocking reads** — `readChannel()` blocks internally; `startConversion()` + `isConversionReady()` + `readRaw()` for non-blocking use
- **Conversion-ready via ALERT/RDY** — supported by writing special threshold values (`Hi=0x8000`, `Lo=0x0000`) and wiring GPIO interrupt; driver exposes the threshold config via `enableConversionReady()`, GPIO handling is left to the caller
- **No global state** — driver is a class; multiple instances allowed for multi-device buses

## Register/Enum Conventions

All enums and constants are defined in the reference doc (Section 11). Key rules:
- MUX enum values encode directly into the config register bits [14:12] (already shifted)
- PGA enum values encode into bits [11:9] (already shifted)
- DR enum values encode into bits [7:5] (already shifted)
- Config register default: `0x8583`

## Voltage Conversion

```
V = (int16_t)raw_code * lsb_uV * 1e-6f
```
LSB sizes per PGA setting are in `ADS1115_driver_reference.md` Section 11.

## Testing

- Unit tests run on the host (no hardware) using a mock/stub I2C backend injected via a thin interface
- Test: register encoding, voltage conversion math, single-shot sequence, comparator config
- Hardware-in-loop tests go in `examples/` (not in `tests/`)
- Preferred test framework: **Catch2 v3** (fetched via CMake `FetchContent`)

## Important Hardware Notes

- ALERT/RDY is open-drain — requires external pullup to VDD
- Do not exceed VDD+0.3V on analog inputs (no software mitigation possible)
- Update threshold registers whenever PGA changes if comparator is in use
- Power-up settling: 50µs before first communication
- I2C addresses: GND→0x48, VDD→0x49, SDA→0x4A, SCL→0x4B

## Reference

- `ADS1115_driver_reference.md` — compressed datasheet (register map, timing, enums, method checklist). Consult this before touching any register field.
