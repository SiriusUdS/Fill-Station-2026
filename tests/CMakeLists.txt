cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# Host (native) build for the HAL-free logic unit tests.
#
# This is a SEPARATE project from the firmware cross-compile build: it compiles
# the logic modules together with GoogleTest and host test doubles (the FakeBus
# in support/) and produces a native executable you run on your machine.
#
# Configure, build & run on any platform with a single command:
#   cmake -P tests/run.cmake
# (run.cmake selects the host toolchain: bundled MinGW on Windows, the system
# compiler on Linux/macOS. See tests/README.md for the manual steps.)
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

# --- GoogleTest -----------------------------------------------------------
# Downloaded on demand into third_party/googletest (git-ignored, not committed)
# by tests/fetch-googletest.cmake, which run.cmake invokes automatically. Update
# the version by editing GTEST_VERSION/GTEST_SHA256 in that script.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/googletest/CMakeLists.txt")
    message(FATAL_ERROR
        "GoogleTest is missing. Run it via 'cmake -P tests/run.cmake', or fetch "
        "it once with 'cmake -P tests/fetch-googletest.cmake'.")
endif()
add_subdirectory(third_party/googletest)

enable_testing()
include(GoogleTest)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Include paths mirror CM7/CMakeLists.txt so the logic resolves its headers the
# same way it does in the firmware build.
set(LOGIC_INCLUDE_DIRS
    ${REPO_ROOT}/CM7/app/logic          # app logic + its interface headers (ethernet.hpp)
    ${REPO_ROOT}/CM7/Core/Inc           # sirius-headers-common submodule lives here
    ${REPO_ROOT}/stm-2026-common/logic        # board-common logic interfaces (can.hpp)
    ${REPO_ROOT}/stm-2026-common/logic/Inc    # board-common CAN protocol defs (dil/can_types.h)
)

# --- FCU controller logic test -------------------------------------------
add_executable(fcu_controller_test
    fcu_controller_test.cpp
    support/fakes.cpp
    ${REPO_ROOT}/CM7/app/logic/fcu_controller.cpp
)
target_include_directories(fcu_controller_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}        # so #include "support/fakes.hpp" resolves
    ${LOGIC_INCLUDE_DIRS}
)
target_link_libraries(fcu_controller_test PRIVATE GTest::gtest_main)

gtest_discover_tests(fcu_controller_test)
