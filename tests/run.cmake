# ---------------------------------------------------------------------------
# Configure, build and run the host logic unit tests — cross-platform.
#
# Driven by CMake itself (script mode), so the same command works on Windows,
# Linux and macOS with no extra dependency beyond the cmake you already need:
#
#     cmake -P tests/run.cmake            # configure + build + test
#     cmake -P tests/run.cmake clean      # wipe the build dir first
#
# On Windows it uses the portable MinGW-w64 toolchain in tools/ (see README);
# on Linux/macOS it uses the system C++ compiler (GCC >= 13 / Clang >= 16).
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

# This script lives in tests/, so its directory is the test source dir.
get_filename_component(TESTS_DIR "${CMAKE_SCRIPT_MODE_FILE}" DIRECTORY)
set(BUILD_DIR "${TESTS_DIR}/build")

# Locate ctest next to the running cmake (CMAKE_CTEST_COMMAND isn't set in -P mode).
get_filename_component(CMAKE_BIN_DIR "${CMAKE_COMMAND}" DIRECTORY)
find_program(CTEST_EXE NAMES ctest HINTS "${CMAKE_BIN_DIR}" REQUIRED)

# Optional trailing "clean" argument wipes the build dir first.
set(DO_CLEAN FALSE)
foreach(i RANGE ${CMAKE_ARGC})
    if(DEFINED CMAKE_ARGV${i} AND "${CMAKE_ARGV${i}}" STREQUAL "clean")
        set(DO_CLEAN TRUE)
    endif()
endforeach()
if(DO_CLEAN AND EXISTS "${BUILD_DIR}")
    message(STATUS "Cleaning ${BUILD_DIR}")
    file(REMOVE_RECURSE "${BUILD_DIR}")
endif()

# Per-platform configure arguments.
set(CONFIGURE_ARGS -S "${TESTS_DIR}" -B "${BUILD_DIR}")
if(CMAKE_HOST_WIN32)
    # The firmware toolchain is arm-none-eabi (can't run on the host), so Windows
    # uses a portable MinGW-w64 GCC kept in tools/ (git-ignored, not committed).
    set(MINGW_BIN "${TESTS_DIR}/../tools/mingw64/bin")
    if(NOT EXISTS "${MINGW_BIN}/g++.exe")
        message(FATAL_ERROR
            "Host toolchain not found at ${MINGW_BIN}/g++.exe.\n"
            "See tests/README.md for how to install the portable MinGW-w64 into tools/.")
    endif()
    list(APPEND CONFIGURE_ARGS
        -G "MinGW Makefiles"
        "-DCMAKE_CXX_COMPILER=${MINGW_BIN}/g++.exe"
        "-DCMAKE_C_COMPILER=${MINGW_BIN}/gcc.exe"
        "-DCMAKE_MAKE_PROGRAM=${MINGW_BIN}/mingw32-make.exe")
endif()

# Run a step and abort with a clear message on failure.
function(run_step description)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "${description} failed (exit ${result})")
    endif()
endfunction()

# GoogleTest is git-ignored (not committed); fetch it on first run. The fetch
# script is a no-op when it's already present, so this is cheap on later runs.
run_step("GoogleTest fetch" ${CMAKE_COMMAND} -P "${TESTS_DIR}/fetch-googletest.cmake")
run_step("CMake configure" ${CMAKE_COMMAND} ${CONFIGURE_ARGS})
run_step("Build"           ${CMAKE_COMMAND} --build "${BUILD_DIR}")
run_step("Tests"           ${CTEST_EXE} --test-dir "${BUILD_DIR}" --output-on-failure)
