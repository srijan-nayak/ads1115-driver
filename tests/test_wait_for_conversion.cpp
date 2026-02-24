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

// Config register value with OS_IDLE bit set (device not converting).
constexpr uint8_t OS_SET_HI = 0x85;  // 0x8583 MSB — bit 15 set
constexpr uint8_t OS_SET_LO = 0x83;

// Config register value with OS_IDLE bit clear (conversion in progress).
constexpr uint8_t OS_CLR_HI = 0x05;  // bit 15 clear
constexpr uint8_t OS_CLR_LO = 0x83;

TEST_CASE("waitForConversion returns true when OS bit is set on first poll", "[wait]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setDataRate(DataRate::SPS_860);  // 2ms sleep — keep tests fast
    mock.enqueueRead({OS_SET_HI, OS_SET_LO});

    CHECK(adc.waitForConversion(std::chrono::milliseconds{500}));
}

TEST_CASE("waitForConversion returns true after initially busy then ready", "[wait]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setDataRate(DataRate::SPS_860);
    mock.enqueueRead({OS_CLR_HI, OS_CLR_LO});  // first poll: busy
    mock.enqueueRead({OS_SET_HI, OS_SET_LO});  // second poll: ready

    CHECK(adc.waitForConversion(std::chrono::milliseconds{500}));
}

TEST_CASE("waitForConversion returns false and sets ConversionTimeout when deadline passes", "[wait]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setDataRate(DataRate::SPS_860);
    // Never report ready
    for (int i = 0; i < 10; ++i)
        mock.enqueueRead({OS_CLR_HI, OS_CLR_LO});

    const bool result = adc.waitForConversion(std::chrono::milliseconds{5});
    CHECK_FALSE(result);
    CHECK(adc.lastError() == Error::ConversionTimeout);
}

TEST_CASE("waitForConversion returns false on I2C read error", "[wait]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setDataRate(DataRate::SPS_860);
    mock.read_ok = false;

    CHECK_FALSE(adc.waitForConversion(std::chrono::milliseconds{500}));
    CHECK(adc.lastError() == Error::I2CReadFailed);
}
