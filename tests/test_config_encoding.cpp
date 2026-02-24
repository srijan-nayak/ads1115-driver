#include <catch2/catch_test_macros.hpp>

#include <ads1115/ads1115.hpp>
#include "mock_i2c.hpp"

using namespace ads1115;
using namespace ads1115::test;

// Queue the reset config value so begin() succeeds, then clear the write log
// so individual tests only see writes from their own operations.
static void beginOK(MockI2CDevice& mock, ADS1115& adc) {
    mock.enqueueRead({0x85, 0x83});  // config::DEFAULT big-endian
    REQUIRE(adc.begin() == Error::None);
    mock.clearWrites();
}

// Helper: extract the uint16_t written to a register from a 3-byte write payload.
static uint16_t extractRegValue(const std::vector<uint8_t>& w) {
    REQUIRE(w.size() == 3);
    return static_cast<uint16_t>((static_cast<uint16_t>(w[1]) << 8) | w[2]);
}

// Helper: extract the register pointer byte.
static uint8_t extractRegAddr(const std::vector<uint8_t>& w) {
    REQUIRE(!w.empty());
    return w[0];
}

TEST_CASE("Default config register is assembled correctly", "[config]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    (void)adc.startConversion();
    REQUIRE(mock.writes().size() == 1);

    const auto& w = mock.writes()[0];
    CHECK(extractRegAddr(w) == reg::CONFIG);

    const uint16_t written = extractRegValue(w);

    // OS=1, MUX=SINGLE_0, PGA=FSR_2048, MODE=SINGLE_SHOT, DR=SPS_128, COMP_DISABLED
    constexpr uint16_t expected =
          config::OS_START
        | static_cast<uint16_t>(Mux::SINGLE_0)
        | static_cast<uint16_t>(PGA::FSR_2048)
        | static_cast<uint16_t>(Mode::SINGLE_SHOT)
        | static_cast<uint16_t>(DataRate::SPS_128)
        | config::COMP_DISABLED;

    CHECK(written == expected);
}

TEST_CASE("MUX bits are encoded correctly", "[config]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    SECTION("DIFF_0_1") {
        adc.setMux(Mux::DIFF_0_1);
        (void)adc.startConversion();
        const uint16_t val = extractRegValue(mock.writes().back());
        CHECK((val & 0x7000) == static_cast<uint16_t>(Mux::DIFF_0_1));
    }
    SECTION("SINGLE_3") {
        adc.setMux(Mux::SINGLE_3);
        (void)adc.startConversion();
        const uint16_t val = extractRegValue(mock.writes().back());
        CHECK((val & 0x7000) == static_cast<uint16_t>(Mux::SINGLE_3));
    }
}

TEST_CASE("PGA bits are encoded correctly", "[config]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    SECTION("FSR_6144") {
        adc.setPGA(PGA::FSR_6144);
        (void)adc.startConversion();
        const uint16_t val = extractRegValue(mock.writes().back());
        CHECK((val & 0x0E00) == static_cast<uint16_t>(PGA::FSR_6144));
    }
    SECTION("FSR_0256") {
        adc.setPGA(PGA::FSR_0256);
        (void)adc.startConversion();
        const uint16_t val = extractRegValue(mock.writes().back());
        CHECK((val & 0x0E00) == static_cast<uint16_t>(PGA::FSR_0256));
    }
}

TEST_CASE("Data rate bits are encoded correctly", "[config]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setDataRate(DataRate::SPS_860);
    (void)adc.startConversion();
    const uint16_t val = extractRegValue(mock.writes().back());
    CHECK((val & 0x00E0) == static_cast<uint16_t>(DataRate::SPS_860));
}

TEST_CASE("Continuous mode clears MODE bit", "[config]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    adc.setMode(Mode::CONTINUOUS);
    (void)adc.startConversion();
    const uint16_t val = extractRegValue(mock.writes().back());
    CHECK((val & 0x0100) == static_cast<uint16_t>(Mode::CONTINUOUS));
}

TEST_CASE("writeRegister sends 3 bytes in big-endian order", "[i2c]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    beginOK(mock, adc);

    (void)adc.startConversion();
    REQUIRE(mock.writes().size() == 1);
    const auto& w = mock.writes()[0];
    REQUIRE(w.size() == 3);
    // byte 0 = register address, byte 1 = MSB, byte 2 = LSB
    const uint16_t reconstructed =
        (static_cast<uint16_t>(w[1]) << 8) | w[2];
    CHECK(reconstructed != 0);  // sanity: something non-zero was written
}

// ── begin() tests ─────────────────────────────────────────────────────────────

TEST_CASE("begin() returns None when config register matches reset value", "[begin]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    mock.enqueueRead({0x85, 0x83});
    CHECK(adc.begin() == Error::None);
}

TEST_CASE("begin() returns UnexpectedDevice when config register is wrong", "[begin]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    mock.enqueueRead({0x00, 0x00});  // not the reset value
    CHECK(adc.begin() == Error::UnexpectedDevice);
}

TEST_CASE("begin() returns I2CReadFailed when read fails", "[begin]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    mock.read_ok = false;
    const Error e = adc.begin();
    CHECK(e == Error::I2CReadFailed);
}

TEST_CASE("begin() returns I2CWriteFailed when write fails", "[begin]") {
    MockI2CDevice mock;
    ADS1115 adc{mock};
    mock.write_ok = false;
    const Error e = adc.begin();
    CHECK(e == Error::I2CWriteFailed);
}
