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

# --- FCU controller logic test (parent-only) ------------------------------
# Tests CM7/app/logic, but depends on the submodule's FakeBus + persistent_state.
add_executable(fcu_controller_test
    logic/fcu_controller_test.cpp
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
gtest_discover_tests(fcu_controller_test)
