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

TEST_CASE("reset() delegates to generalCallReset()", "[reset]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    CHECK(adc.reset());
    CHECK(mock.resetCallCount() == 1);
}

TEST_CASE("reset() returns false when generalCallReset() fails", "[reset]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.reset_ok = false;
    CHECK_FALSE(adc.reset());
}
