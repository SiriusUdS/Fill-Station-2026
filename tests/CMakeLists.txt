cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# Host (native) build for the HAL-free logic unit tests.
#
# Separate project from the firmware cross-compile build. The test FRAMEWORK
# (GoogleTest fetch, toolchain selection, the FakeBus, and the tests for code
# that lives in stm-2026-common) is NOT duplicated here — it lives in the
# submodule and we reuse it via add_subdirectory below. This file adds only the
# parent-only targets on top.
#
# Configure, build & run on any platform with a single command:
#   cmake -P tests/run.cmake
# ---------------------------------------------------------------------------

project(fillstation_logic_tests CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# On Windows/MinGW, make the test binaries self-contained so they don't depend on
# the MinGW runtime DLLs being on PATH (gtest_discover_tests runs the exe at build
# time). On Linux a full -static link pulls in static glibc, which is often absent
# and warns about getaddrinfo &c., so we only do this for MinGW.
if(MINGW)
    add_link_options(-static -static-libgcc -static-libstdc++)
endif()

enable_testing()

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(STMCOMMON ${REPO_ROOT}/stm-2026-common)

# Reuse the submodule's test build: this defines GoogleTest, the common-owned
# tests (persistent_state, command), and exports STMCOMMON_LOGIC_INCLUDE_DIRS.
# So running the parent suite covers the common code too, with zero duplication.
add_subdirectory(${STMCOMMON}/tests ${CMAKE_BINARY_DIR}/stmcommon-tests)

include(GoogleTest)

# Host tests are split into unit/ (one unit in isolation) and integration/
# (several real units assembled — no fakes). Tests are labelled accordingly, so
# `ctest -L unit` / `ctest -L integration` select a subset; `run.cmake` runs all.

# The command-handler translation units, reused by the unit and integration
# targets that link the handlers (single list, no duplication).
set(COMMAND_HANDLERS_DIR ${REPO_ROOT}/CM7/app/logic/control/command_handlers)
set(COMMAND_HANDLER_SOURCES
    ${COMMAND_HANDLERS_DIR}/main_handler.cpp
    ${COMMAND_HANDLERS_DIR}/ping.cpp
    ${COMMAND_HANDLERS_DIR}/execute_ping.cpp
    ${COMMAND_HANDLERS_DIR}/set_state.cpp
    ${COMMAND_HANDLERS_DIR}/set_state_transitions/activate_igniter.cpp
    ${COMMAND_HANDLERS_DIR}/set_valve_position.cpp
    ${COMMAND_HANDLERS_DIR}/execute_set_valve_position.cpp
    ${COMMAND_HANDLERS_DIR}/synchronise.cpp
    ${COMMAND_HANDLERS_DIR}/execute_synchronise.cpp
)

# --- FCU controller logic test (unit) -------------------------------------
# Tests CM7/app/logic, but depends on the submodule's FakeBus + persistent_state.
add_executable(fcu_controller_test
    unit/logic/fcu_controller_test.cpp
    ${STMCOMMON}/tests/support/fakes.cpp
    ${REPO_ROOT}/CM7/app/logic/fcu_controller.cpp
    ${STMCOMMON}/logic/control/persistent_state.cpp
)
target_include_directories(fcu_controller_test PRIVATE
    ${STMCOMMON}/tests                  # so #include "support/fakes.hpp" resolves
    ${REPO_ROOT}/CM7/app/logic          # app logic + its interface headers (ethernet.hpp)
    ${STMCOMMON_LOGIC_INCLUDE_DIRS}     # exported by the submodule's test build
)
target_link_libraries(fcu_controller_test PRIVATE GTest::gtest_main)
gtest_discover_tests(fcu_controller_test PROPERTIES LABELS "unit")

# --- Command handlers test (unit) -----------------------------------------
# Covers CM7/app/logic/control/command_handlers. SetState commits through the
# submodule's persistent_state, so that source is compiled in too.
add_executable(command_handlers_test
    unit/logic/control/command_handlers/command_handlers_test.cpp
    ${COMMAND_HANDLER_SOURCES}
    ${STMCOMMON}/logic/control/persistent_state.cpp
)
target_include_directories(command_handlers_test PRIVATE
    ${REPO_ROOT}/CM7/app/logic          # the handlers + their "control/..." includes
    ${STMCOMMON_LOGIC_INCLUDE_DIRS}     # command/, control/ headers from the submodule
)
target_link_libraries(command_handlers_test PRIVATE GTest::gtest_main)
gtest_discover_tests(command_handlers_test PROPERTIES LABELS "unit")

# --- Ethernet -> state change (integration) -------------------------------
# Full command pipeline: the submodule's Ethernet parser feeds the main command
# handler, which commits to persistent state. Real units, no fakes.
add_executable(ethernet_state_change_test
    integration/ethernet_state_change_test.cpp
    ${COMMAND_HANDLER_SOURCES}
    ${STMCOMMON}/logic/communication/protocol/command/parser/command_ethernet_parser.cpp
    ${STMCOMMON}/logic/control/persistent_state.cpp
)
target_include_directories(ethernet_state_change_test PRIVATE
    ${REPO_ROOT}/CM7/app/logic          # command handlers
    ${STMCOMMON_LOGIC_INCLUDE_DIRS}     # parser, command/, control/, ethernet/ headers
)
target_link_libraries(ethernet_state_change_test PRIVATE GTest::gtest_main)
gtest_discover_tests(ethernet_state_change_test PROPERTIES LABELS "integration")
