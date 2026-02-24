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

static uint8_t regAddr(const std::vector<uint8_t>& w) {
    REQUIRE(!w.empty());
    return w[0];
}

// ── setThresholds ─────────────────────────────────────────────────────────────

TEST_CASE("setThresholds writes LO then HI register", "[thresholds]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    REQUIRE(adc.setThresholds(-500, 500));
    REQUIRE(mock.writes().size() == 2);

    CHECK(regAddr(mock.writes()[0])  == reg::LO_THRESH);
    CHECK(regValue(mock.writes()[0]) == static_cast<uint16_t>(int16_t{-500}));

    CHECK(regAddr(mock.writes()[1])  == reg::HI_THRESH);
    CHECK(regValue(mock.writes()[1]) == 500);
}

TEST_CASE("setThresholds rejects hi <= lo", "[thresholds]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    SECTION("equal") {
        CHECK_FALSE(adc.setThresholds(100, 100));
        CHECK(adc.lastError() == Error::InvalidArgument);
        CHECK(mock.writes().empty());
    }
    SECTION("hi < lo") {
        CHECK_FALSE(adc.setThresholds(200, 100));
        CHECK(adc.lastError() == Error::InvalidArgument);
        CHECK(mock.writes().empty());
    }
}

TEST_CASE("setThresholds returns false on I2C failure", "[thresholds]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.write_ok = false;
    CHECK_FALSE(adc.setThresholds(-100, 100));
    CHECK(adc.lastError() == Error::I2CWriteFailed);
}

// ── enableConversionReady ─────────────────────────────────────────────────────

TEST_CASE("enableConversionReady writes HI=0x8000 then LO=0x0000 then config", "[conv_ready]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    REQUIRE(adc.enableConversionReady(CompQueue::AFTER_1));
    REQUIRE(mock.writes().size() == 3);

    // Datasheet §6 special threshold values
    CHECK(regAddr(mock.writes()[0])  == reg::HI_THRESH);
    CHECK(regValue(mock.writes()[0]) == 0x8000);

    CHECK(regAddr(mock.writes()[1])  == reg::LO_THRESH);
    CHECK(regValue(mock.writes()[1]) == 0x0000);

    // Config write with COMP_QUE != DISABLED
    CHECK(regAddr(mock.writes()[2]) == reg::CONFIG);
    const uint16_t cfg = regValue(mock.writes()[2]);
    CHECK((cfg & 0x0003) == static_cast<uint16_t>(CompQueue::AFTER_1));
}

TEST_CASE("enableConversionReady rejects CompQueue::DISABLED", "[conv_ready]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    CHECK_FALSE(adc.enableConversionReady(CompQueue::DISABLED));
    CHECK(adc.lastError() == Error::InvalidArgument);
    CHECK(mock.writes().empty());
}

TEST_CASE("enableConversionReady queue setting persists into startConversion", "[conv_ready]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    REQUIRE(adc.enableConversionReady(CompQueue::AFTER_2));
    mock.clearWrites();

    (void)adc.startConversion();
    REQUIRE(mock.writes().size() == 1);
    const uint16_t cfg = regValue(mock.writes()[0]);
    CHECK((cfg & 0x0003) == static_cast<uint16_t>(CompQueue::AFTER_2));
}

TEST_CASE("enableConversionReady returns false on I2C failure", "[conv_ready]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.write_ok = false;
    CHECK_FALSE(adc.enableConversionReady(CompQueue::AFTER_1));
    CHECK(adc.lastError() == Error::I2CWriteFailed);
}
