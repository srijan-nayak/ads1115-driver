include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
)
FetchContent_MakeAvailable(Catch2)

add_executable(ads1115_tests
    test_config_encoding.cpp
    test_voltage_conversion.cpp
    test_wait_for_conversion.cpp
    test_comparator.cpp
    test_thresholds.cpp
    test_reset.cpp
    test_read_channel.cpp
    test_is_conversion_ready.cpp
    test_power_down.cpp
)

target_link_libraries(ads1115_tests
    PRIVATE ads1115 Catch2::Catch2WithMain
)

# Expose src/ so tests can include mock_i2c.hpp alongside driver internals
target_include_directories(ads1115_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

include(Catch)
catch_discover_tests(ads1115_tests)
