#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <ads1115/ads1115.hpp>
#include "mock_i2c.hpp"

using namespace ads1115;
using namespace ads1115::test;
using Catch::Approx;

TEST_CASE("toVoltage: zero code produces 0 V", "[voltage]") {
    CHECK(ADS1115::toVoltage(0, PGA::FSR_2048) == Approx(0.0f).margin(1e-9));
}

TEST_CASE("toVoltage: positive full-scale (±2.048V PGA)", "[voltage]") {
    // 0x7FFF = 32767 → 32767 × 62.5e-6 ≈ 2.04794 V
    const float v = ADS1115::toVoltage(32767, PGA::FSR_2048);
    CHECK(v == Approx(32767 * 62.5e-6f).epsilon(1e-5));
}

TEST_CASE("toVoltage: negative full-scale (±2.048V PGA)", "[voltage]") {
    // 0x8000 = -32768 → -32768 × 62.5e-6 ≈ -2.048 V
    const float v = ADS1115::toVoltage(-32768, PGA::FSR_2048);
    CHECK(v == Approx(-32768 * 62.5e-6f).epsilon(1e-5));
}

TEST_CASE("toVoltage: LSB size per PGA setting", "[voltage]") {
    // Each PGA setting defines an LSB; one raw count should equal exactly that LSB.
    CHECK(ADS1115::toVoltage(1, PGA::FSR_6144) == Approx(187.5e-6f).epsilon(1e-5));
    CHECK(ADS1115::toVoltage(1, PGA::FSR_4096) == Approx(125.0e-6f).epsilon(1e-5));
    CHECK(ADS1115::toVoltage(1, PGA::FSR_2048) == Approx(62.5e-6f).epsilon(1e-5));
    CHECK(ADS1115::toVoltage(1, PGA::FSR_1024) == Approx(31.25e-6f).epsilon(1e-5));
    CHECK(ADS1115::toVoltage(1, PGA::FSR_0512) == Approx(15.625e-6f).epsilon(1e-5));
    CHECK(ADS1115::toVoltage(1, PGA::FSR_0256) == Approx(7.8125e-6f).epsilon(1e-5));
}

TEST_CASE("toVoltage: known value 1.0 V with ±2.048V PGA", "[voltage]") {
    // 1.0 V / 62.5e-6 = 16000 counts
    CHECK(ADS1115::toVoltage(16000, PGA::FSR_2048) == Approx(1.0f).epsilon(1e-4));
}

TEST_CASE("readVoltage uses current PGA setting", "[voltage][integration]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    mock.enqueueRead({0x85, 0x83});  // for begin() config verify
    REQUIRE(adc.begin() == Error::None);
    adc.setPGA(PGA::FSR_1024);

    // Queue a raw value of 8000 for the CONVERSION register read
    // readRaw() does: write(pointer=0x00), read(2 bytes)
    // 8000 in big-endian = 0x1F40
    mock.enqueueRead({0x1F, 0x40});

    const auto v = adc.readVoltage();
    REQUIRE(v.has_value());
    // 8000 × 31.25e-6 = 0.25 V
    CHECK(*v == Approx(0.25f).epsilon(1e-4));
}
