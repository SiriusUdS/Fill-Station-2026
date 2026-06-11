cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# Host (native) build for stm-2026-common's HAL-free logic unit tests.
#
# Standalone-friendly: this builds on its own (CI clones just this submodule and
# runs `cmake -P tests/run.cmake`), and is ALSO add_subdirectory()'d by the
# parent repo's test build so the parent runs these tests too. It compiles only
# code that lives in this submodule; the parent owns tests for parent-only code.
#
# When included by the parent it exports STMCOMMON_LOGIC_INCLUDE_DIRS (cache) so
# the parent can build its own targets against the same headers, and it brings
# in the shared GoogleTest, so neither is duplicated.
# ---------------------------------------------------------------------------

project(stmcommon_logic_tests CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# On Windows/MinGW, make test binaries self-contained (gtest_discover_tests runs
# them at build time). Harmless if already set by the parent.
if(MINGW)
    add_link_options(-static -static-libgcc -static-libstdc++)
endif()

# --- GoogleTest -----------------------------------------------------------
# Downloaded on demand into third_party/googletest (git-ignored) by
# fetch-googletest.cmake, which run.cmake (and the parent) invoke automatically.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/googletest/CMakeLists.txt")
    message(FATAL_ERROR
        "GoogleTest is missing. Run the tests via 'cmake -P tests/run.cmake', or "
        "fetch it once with 'cmake -P tests/fetch-googletest.cmake'.")
endif()
add_subdirectory(third_party/googletest)

enable_testing()
include(GoogleTest)

# Logic include dirs, mirroring how the firmware build resolves these headers.
# Exported to the cache so the parent's test build can reuse them verbatim.
set(COMMON_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(STMCOMMON_LOGIC_INCLUDE_DIRS
    ${COMMON_ROOT}/logic                       # logic interfaces + control/ + communication/
    ${COMMON_ROOT}/logic/Inc                   # CAN protocol defs (dil/can_types.h)
    ${COMMON_ROOT}/logic/communication/protocol # command/, can/, ethernet/, sirius-headers-common/
    CACHE INTERNAL "stm-2026-common logic include dirs for host tests")

# Host tests are split into unit/ (one unit in isolation) and integration/
# (several units assembled). This submodule currently has only unit tests; see
# tests/integration/README.md. Tests are labelled so `ctest -L unit` works.

# --- Persistent state test (unit) -----------------------------------------
add_executable(persistent_state_test
    unit/logic/control/persistent_state_test.cpp
    ${COMMON_ROOT}/logic/control/persistent_state.cpp
)
target_include_directories(persistent_state_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${STMCOMMON_LOGIC_INCLUDE_DIRS}
)
target_link_libraries(persistent_state_test PRIVATE GTest::gtest_main)
gtest_discover_tests(persistent_state_test PROPERTIES LABELS "unit")

# --- Valve seam test (unit) -----------------------------------------------
# Header-only on the logic side (the Valve concept + FakeValve double), so the
# test compiles just its own TU; the platform BallValve is firmware-only (HAL).
add_executable(valve_test
    unit/logic/actuation/valve_test.cpp
)
target_include_directories(valve_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${STMCOMMON_LOGIC_INCLUDE_DIRS}
)
target_link_libraries(valve_test PRIVATE GTest::gtest_main)
gtest_discover_tests(valve_test PROPERTIES LABELS "unit")

# --- Command parser test (unit) -------------------------------------------
set(COMMAND_DIR ${COMMON_ROOT}/logic/communication/protocol/command)
add_executable(command_test
    unit/logic/communication/protocol/command/command_test.cpp
    ${COMMAND_DIR}/parser/command_can_parser.cpp
    ${COMMAND_DIR}/parser/command_ethernet_parser.cpp
)
target_include_directories(command_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${STMCOMMON_LOGIC_INCLUDE_DIRS}
)
target_link_libraries(command_test PRIVATE GTest::gtest_main)
gtest_discover_tests(command_test PROPERTIES LABELS "unit")
