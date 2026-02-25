cmake_minimum_required(VERSION 3.16)
project(ads1115-driver VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# ── Install ───────────────────────────────────────────────────────────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ads1115
    EXPORT  ads1115Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/ads1115
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ads1115Targets
    FILE      ads1115Targets.cmake
    NAMESPACE ads1115::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ads1115
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ads1115ConfigVersion.cmake"
    VERSION       ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ads1115Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ads1115Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ads1115
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ads1115Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ads1115ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ads1115
)

# ── Subdirectories ────────────────────────────────────────────────────────────
if(ADS1115_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(ADS1115_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
