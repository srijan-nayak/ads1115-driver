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

TEST_CASE("setComparator writes threshold registers then config", "[comparator]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    const bool ok = adc.setComparator(
        CompMode::TRADITIONAL, CompPolarity::ACTIVE_LOW,
        CompLatch::NON_LATCHING, CompQueue::AFTER_1,
        1000, 2000);

    REQUIRE(ok);
    REQUIRE(mock.writes().size() == 3);

    // Write 0: LO_THRESH = 1000 (0x03E8)
    CHECK(regAddr(mock.writes()[0])  == reg::LO_THRESH);
    CHECK(regValue(mock.writes()[0]) == 1000);

    // Write 1: HI_THRESH = 2000 (0x07D0)
    CHECK(regAddr(mock.writes()[1])  == reg::HI_THRESH);
    CHECK(regValue(mock.writes()[1]) == 2000);

    // Write 2: CONFIG with comparator bits set
    CHECK(regAddr(mock.writes()[2]) == reg::CONFIG);
    const uint16_t cfg = regValue(mock.writes()[2]);
    CHECK((cfg & 0x0003) == static_cast<uint16_t>(CompQueue::AFTER_1));
    CHECK((cfg & 0x0010) == static_cast<uint16_t>(CompMode::TRADITIONAL));
    CHECK((cfg & 0x0008) == static_cast<uint16_t>(CompPolarity::ACTIVE_LOW));
    CHECK((cfg & 0x0004) == static_cast<uint16_t>(CompLatch::NON_LATCHING));
}

TEST_CASE("setComparator: window mode and active-high polarity encode correctly", "[comparator]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    REQUIRE(adc.setComparator(
        CompMode::WINDOW, CompPolarity::ACTIVE_HIGH,
        CompLatch::LATCHING, CompQueue::AFTER_4,
        -100, 100));

    const uint16_t cfg = regValue(mock.writes()[2]);
    CHECK((cfg & 0x0010) == static_cast<uint16_t>(CompMode::WINDOW));
    CHECK((cfg & 0x0008) == static_cast<uint16_t>(CompPolarity::ACTIVE_HIGH));
    CHECK((cfg & 0x0004) == static_cast<uint16_t>(CompLatch::LATCHING));
    CHECK((cfg & 0x0003) == static_cast<uint16_t>(CompQueue::AFTER_4));
}

TEST_CASE("setComparator rejects hi_thresh <= lo_thresh", "[comparator]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    SECTION("equal") {
        CHECK_FALSE(adc.setComparator(
            CompMode::TRADITIONAL, CompPolarity::ACTIVE_LOW,
            CompLatch::NON_LATCHING, CompQueue::AFTER_1,
            500, 500));
        CHECK(adc.lastError() == Error::InvalidArgument);
        CHECK(mock.writes().empty());
    }
    SECTION("hi < lo") {
        CHECK_FALSE(adc.setComparator(
            CompMode::TRADITIONAL, CompPolarity::ACTIVE_LOW,
            CompLatch::NON_LATCHING, CompQueue::AFTER_1,
            1000, 500));
        CHECK(adc.lastError() == Error::InvalidArgument);
        CHECK(mock.writes().empty());
    }
}

TEST_CASE("setComparator comparator state persists into subsequent startConversion", "[comparator]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    REQUIRE(adc.setComparator(
        CompMode::TRADITIONAL, CompPolarity::ACTIVE_HIGH,
        CompLatch::NON_LATCHING, CompQueue::AFTER_2,
        0, 1000));
    mock.clearWrites();

    (void)adc.startConversion();
    REQUIRE(mock.writes().size() == 1);
    const uint16_t cfg = regValue(mock.writes()[0]);
    CHECK((cfg & 0x0008) == static_cast<uint16_t>(CompPolarity::ACTIVE_HIGH));
    CHECK((cfg & 0x0003) == static_cast<uint16_t>(CompQueue::AFTER_2));
}

TEST_CASE("disableComparator resets comparator bits and writes config", "[comparator]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    // First enable, then disable
    REQUIRE(adc.setComparator(
        CompMode::WINDOW, CompPolarity::ACTIVE_HIGH,
        CompLatch::LATCHING, CompQueue::AFTER_1,
        0, 1000));
    mock.clearWrites();

    REQUIRE(adc.disableComparator());
    REQUIRE(mock.writes().size() == 1);
    const uint16_t cfg = regValue(mock.writes()[0]);
    CHECK((cfg & 0x0003) == static_cast<uint16_t>(CompQueue::DISABLED));
    CHECK((cfg & 0x0010) == static_cast<uint16_t>(CompMode::TRADITIONAL));
    CHECK((cfg & 0x0008) == static_cast<uint16_t>(CompPolarity::ACTIVE_LOW));
    CHECK((cfg & 0x0004) == static_cast<uint16_t>(CompLatch::NON_LATCHING));
}

TEST_CASE("setComparator returns false on I2C write failure", "[comparator]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.write_ok = false;
    CHECK_FALSE(adc.setComparator(
        CompMode::TRADITIONAL, CompPolarity::ACTIVE_LOW,
        CompLatch::NON_LATCHING, CompQueue::AFTER_1,
        0, 1000));
    CHECK(adc.lastError() == Error::I2CWriteFailed);
}
