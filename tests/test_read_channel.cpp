#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <ads1115/ads1115.hpp>
#include "mock_i2c.hpp"

using namespace ads1115;
using namespace ads1115::test;
using Catch::Approx;

static void beginOK(MockI2CDevice& mock, ADS1115& adc) {
    mock.enqueueRead({0x85, 0x83});
    REQUIRE(adc.begin() == Error::None);
    mock.clearWrites();
}

// Queue everything readChannel() needs after begin():
//   1. One CONFIG read for waitForConversion() poll (OS_IDLE set)
//   2. One CONVERSION read for readRaw()
static void enqueueReadChannel(MockI2CDevice& mock, uint16_t raw_code) {
    mock.enqueueRead({0x85, 0x83});  // waitForConversion poll: OS_IDLE set
    mock.enqueueRead({
        static_cast<uint8_t>(raw_code >> 8),
        static_cast<uint8_t>(raw_code & 0xFF),
    });
}

TEST_CASE("readChannel returns correct voltage for a known raw code", "[readchannel]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setPGA(PGA::FSR_2048);
    adc.setDataRate(DataRate::SPS_860);

    // 16000 counts × 62.5 µV = 1.0 V
    enqueueReadChannel(mock, 16000);

    const auto v = adc.readChannel(Mux::SINGLE_0);
    REQUIRE(v.has_value());
    CHECK(*v == Approx(1.0f).epsilon(1e-4));
}

TEST_CASE("readChannel sets MUX before triggering conversion", "[readchannel]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setDataRate(DataRate::SPS_860);
    enqueueReadChannel(mock, 0);

    (void)adc.readChannel(Mux::SINGLE_2);

    // First write is the CONFIG register write from startConversion()
    REQUIRE(!mock.writes().empty());
    const auto& cfg_write = mock.writes()[0];
    REQUIRE(cfg_write.size() == 3);
    const uint16_t cfg = (static_cast<uint16_t>(cfg_write[1]) << 8) | cfg_write[2];
    CHECK((cfg & 0x7000) == static_cast<uint16_t>(Mux::SINGLE_2));
}

TEST_CASE("readChannel returns nullopt when startConversion fails", "[readchannel]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    mock.write_ok = false;
    CHECK_FALSE(adc.readChannel(Mux::SINGLE_0).has_value());
}

TEST_CASE("readChannel returns nullopt when waitForConversion fails", "[readchannel]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setDataRate(DataRate::SPS_860);
    // No reads queued — readRegister inside waitForConversion will fail immediately
    mock.read_ok = false;

    CHECK_FALSE(adc.readChannel(Mux::SINGLE_0).has_value());
    CHECK(adc.lastError() != Error::None);
}

TEST_CASE("readChannel returns nullopt when conversion read fails", "[readchannel]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setDataRate(DataRate::SPS_860);
    mock.enqueueRead({0x85, 0x83});  // waitForConversion: ready
    mock.read_ok = false;            // readRaw() read will fail

    CHECK_FALSE(adc.readChannel(Mux::SINGLE_0).has_value());
}
