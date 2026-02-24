cmake_minimum_required(VERSION 3.16)
project(ads1115-driver VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Library ──────────────────────────────────────────────────────────────────
add_library(ads1115
    src/ads1115.cpp
    src/linux_i2c.cpp
)

target_include_directories(ads1115
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    PRIVATE src
)

target_compile_options(ads1115 PRIVATE -Wall -Wextra -Wpedantic)

# ── Options ───────────────────────────────────────────────────────────────────
option(ADS1115_BUILD_TESTS    "Build unit tests" ON)
option(ADS1115_BUILD_EXAMPLES "Build examples"   OFF)

# ── Subdirectories ────────────────────────────────────────────────────────────
if(ADS1115_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(ADS1115_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
