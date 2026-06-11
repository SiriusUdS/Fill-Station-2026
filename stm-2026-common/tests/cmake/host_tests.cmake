# ---------------------------------------------------------------------------
# Shared host-test runner — the single home of the test-harness logic.
#
# This module lives in stm-2026-common (the submodule), so it works when the
# submodule is tested standalone AND when the parent repo reuses it: the parent
# always has the submodule checked out, but never the other way round. Each repo
# keeps only a ~4-line run.cmake that include()s this and calls run_host_tests().
#
#     get_filename_component(TESTS_DIR "${CMAKE_SCRIPT_MODE_FILE}" DIRECTORY)
#     include("<.../stm-2026-common/tests/cmake>/host_tests.cmake")
#     run_host_tests("${TESTS_DIR}")
#
# Pass a trailing "clean" to the run.cmake to wipe the build dir first.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

# Captured at include() time: this file's dir (stm-2026-common/tests/cmake), used
# to locate the canonical fetch-googletest.cmake regardless of who include()s us.
set(_HOST_TESTS_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(_run_step description)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "${description} failed (exit ${result})")
    endif()
endfunction()

# Configure + build + ctest the host-test project rooted at TESTS_DIR.
function(run_host_tests TESTS_DIR)
    set(BUILD_DIR "${TESTS_DIR}/build")

    # Locate ctest next to the running cmake (CMAKE_CTEST_COMMAND isn't set in -P).
    get_filename_component(_cmake_bin_dir "${CMAKE_COMMAND}" DIRECTORY)
    find_program(CTEST_EXE NAMES ctest HINTS "${_cmake_bin_dir}" REQUIRED)

    # Optional trailing "clean" argument wipes the build dir first.
    set(_do_clean FALSE)
    foreach(i RANGE ${CMAKE_ARGC})
        if(DEFINED CMAKE_ARGV${i} AND "${CMAKE_ARGV${i}}" STREQUAL "clean")
            set(_do_clean TRUE)
        endif()
    endforeach()
    if(_do_clean AND EXISTS "${BUILD_DIR}")
        message(STATUS "Cleaning ${BUILD_DIR}")
        file(REMOVE_RECURSE "${BUILD_DIR}")
    endif()

    # Per-platform configure arguments.
    set(_configure_args -S "${TESTS_DIR}" -B "${BUILD_DIR}")
    if(CMAKE_HOST_WIN32)
        # The firmware toolchain is arm-none-eabi (can't run on the host), so
        # Windows uses a portable MinGW-w64 GCC kept in the parent repo's tools/
        # (git-ignored). Search the likely locations; override with -DMINGW_DIR=.
        set(_mingw_candidates
            "$ENV{MINGW_DIR}/bin"
            "${MINGW_DIR}/bin"
            "${TESTS_DIR}/../tools/mingw64/bin"        # parent layout: <repo>/tests/../tools
            "${TESTS_DIR}/../../tools/mingw64/bin")     # nested submodule: <repo>/stm-2026-common/tests/../../tools
        set(_mingw_bin "")
        foreach(c ${_mingw_candidates})
            if(c AND EXISTS "${c}/g++.exe")
                set(_mingw_bin "${c}")
                break()
            endif()
        endforeach()
        if(NOT _mingw_bin)
            string(REPLACE ";" "\n  " _looked "${_mingw_candidates}")
            message(FATAL_ERROR
                "Host toolchain (MinGW-w64 g++) not found. Looked in:\n  ${_looked}\n"
                "Set -DMINGW_DIR=<path-to-mingw64>, or run the tests via the parent repo. "
                "See tests/README.md.")
        endif()
        list(APPEND _configure_args
            -G "MinGW Makefiles"
            "-DCMAKE_CXX_COMPILER=${_mingw_bin}/g++.exe"
            "-DCMAKE_C_COMPILER=${_mingw_bin}/gcc.exe"
            "-DCMAKE_MAKE_PROGRAM=${_mingw_bin}/mingw32-make.exe")
    endif()

    # GoogleTest is git-ignored (not committed); fetch it on first run via the
    # canonical script next to this module. It downloads into its own dir's
    # third_party/, so the submodule and parent share one copy and one version.
    _run_step("GoogleTest fetch" ${CMAKE_COMMAND} -P "${_HOST_TESTS_CMAKE_DIR}/../fetch-googletest.cmake")
    _run_step("CMake configure"  ${CMAKE_COMMAND} ${_configure_args})
    _run_step("Build"            ${CMAKE_COMMAND} --build "${BUILD_DIR}")
    _run_step("Tests"            ${CTEST_EXE} --test-dir "${BUILD_DIR}" --output-on-failure)
endfunction()
