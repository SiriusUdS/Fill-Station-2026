cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# Host (native) build for the HAL-free logic unit tests — consolidated.
#
# One harness for the whole monorepo. The test translation units live at the
# per-site dirs src/app/logic/<site>/tests (one site per board + 'common' for the
# shared logic); this file owns the shared infra (GoogleTest, the FakeBus + other
# doubles under support/, and the portable logic include dirs) and pulls each site
# in via add_subdirectory.
#
#   cmake -P src/app/tests/run.cmake            # configure + build + test
#   cmake -P src/app/tests/run.cmake clean      # wipe the build dir first
# ---------------------------------------------------------------------------

project(fillstation_host_tests CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# On Windows/MinGW, make the test binaries self-contained so they don't depend on
# the MinGW runtime DLLs being on PATH (gtest_discover_tests runs the exe at build
# time).
if(MINGW)
    add_link_options(-static -static-libgcc -static-libstdc++)
endif()

# --- GoogleTest -----------------------------------------------------------
# Fetched on demand into third_party/googletest (git-ignored) by
# fetch-googletest.cmake, which run.cmake invokes automatically.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/googletest/CMakeLists.txt")
    message(FATAL_ERROR
        "GoogleTest is missing. Run the tests via 'cmake -P src/app/tests/run.cmake', or "
        "fetch it once with 'cmake -P src/app/tests/fetch-googletest.cmake'.")
endif()
add_subdirectory(third_party/googletest)

enable_testing()
include(GoogleTest)

# --- Shared test infra + portable logic include dirs ----------------------
# Mirrors how the firmware build resolves these headers, so tests compile the
# real logic. Visible to each add_subdirectory()'d site below.
set(APP_DIR      ${CMAKE_CURRENT_SOURCE_DIR}/..)        # src/app
set(TEST_SUPPORT ${CMAKE_CURRENT_SOURCE_DIR})          # so #include "support/..." resolves
set(FAKES_SRC    ${CMAKE_CURRENT_SOURCE_DIR}/support/fakes.cpp)   # the FakeBus impl
set(CRC_HOST_SRC ${CMAKE_CURRENT_SOURCE_DIR}/support/crc32_host.cpp)  # software def of the data-integrity CRC seam
set(WD_HOST_SRC  ${CMAKE_CURRENT_SOURCE_DIR}/support/watchdog_host.cpp)  # no-op def of the watchdog feed seam (handlePing kick)
set(LOGIC_COMMON ${APP_DIR}/logic/common)
set(LOGIC_FCU    ${APP_DIR}/logic/fcu)
set(LOGIC_ECU    ${APP_DIR}/logic/ecu)
set(LOGIC_COMMON_INCLUDE_DIRS
    ${LOGIC_COMMON}                        # interfaces + control/ + communication/
    ${LOGIC_COMMON}/communication/protocol # wire formats: peripherals/, devices/, framing/, command/, system/, telemetry/
)

# --- Test sites (one per board + common) ----------------------------------
add_subdirectory(${APP_DIR}/logic/common/tests ${CMAKE_BINARY_DIR}/common-tests)
add_subdirectory(${APP_DIR}/logic/fcu/tests    ${CMAKE_BINARY_DIR}/fcu-tests)
add_subdirectory(${APP_DIR}/logic/ecu/tests    ${CMAKE_BINARY_DIR}/ecu-tests)
