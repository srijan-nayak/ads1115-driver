#include <catch2/catch_test_macros.hpp>

#include <ads1115/ads1115.hpp>
#include "mock_i2c.hpp"

using namespace ads1115;
using namespace ads1115::test;

static void beginOK(MockI2CDevice& mock, ADS1115& adc) {
    mock.enqueueRead({0x85, 0x83});
    REQUIRE(adc.begin() == Error::None);
    mock.clearWrites();
}

TEST_CASE("isConversionReady returns true when OS bit is set", "[is_ready]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.enqueueRead({0x85, 0x83});  // bit 15 set → idle
    const auto result = adc.isConversionReady();
    REQUIRE(result.has_value());
    CHECK(*result == true);
}

TEST_CASE("isConversionReady returns false when OS bit is clear", "[is_ready]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.enqueueRead({0x05, 0x83});  // bit 15 clear → converting
    const auto result = adc.isConversionReady();
    REQUIRE(result.has_value());
    CHECK(*result == false);
}

TEST_CASE("isConversionReady returns nullopt on I2C read error", "[is_ready]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.read_ok = false;
    CHECK_FALSE(adc.isConversionReady().has_value());
    CHECK(adc.lastError() == Error::I2CReadFailed);
}

TEST_CASE("isConversionReady returns nullopt on I2C write error", "[is_ready]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.write_ok = false;
    CHECK_FALSE(adc.isConversionReady().has_value());
    CHECK(adc.lastError() == Error::I2CWriteFailed);
}
