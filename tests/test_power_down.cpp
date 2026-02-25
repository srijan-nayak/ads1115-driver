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

static uint16_t regValue(const std::vector<uint8_t>& w) {
    REQUIRE(w.size() == 3);
    return static_cast<uint16_t>((static_cast<uint16_t>(w[1]) << 8) | w[2]);
}

TEST_CASE("powerDown writes config with MODE=SINGLE_SHOT and OS bit clear", "[powerdown]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    // Start in continuous mode
    adc.setMode(Mode::CONTINUOUS);
    mock.clearWrites();

    REQUIRE(adc.powerDown());
    REQUIRE(mock.writes().size() == 1);

    const uint16_t cfg = regValue(mock.writes()[0]);
    CHECK((cfg & 0x0100) == static_cast<uint16_t>(Mode::SINGLE_SHOT));  // MODE bit set
    CHECK((cfg & 0x8000) == 0);                                          // OS bit not set
}

TEST_CASE("powerDown updates mode state so subsequent startConversion uses single-shot", "[powerdown]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setMode(Mode::CONTINUOUS);
    REQUIRE(adc.powerDown());
    mock.clearWrites();

    (void)adc.startConversion();
    REQUIRE(mock.writes().size() == 1);
    const uint16_t cfg = regValue(mock.writes()[0]);
    CHECK((cfg & 0x0100) == static_cast<uint16_t>(Mode::SINGLE_SHOT));
}

TEST_CASE("powerDown returns false on I2C write failure", "[powerdown]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.write_ok = false;
    CHECK_FALSE(adc.powerDown());
    CHECK(adc.lastError() == Error::I2CWriteFailed);
}
