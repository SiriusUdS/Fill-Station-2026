# ---------------------------------------------------------------------------
# Configure, build and run the stm-2026-common host unit tests — standalone.
#
#     cmake -P tests/run.cmake            # configure + build + test
#     cmake -P tests/run.cmake clean      # wipe the build dir first
#
# All the harness logic lives in cmake/host_tests.cmake (shared with the parent
# repo). This entry point only names which test project to run.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

get_filename_component(TESTS_DIR "${CMAKE_SCRIPT_MODE_FILE}" DIRECTORY)
include("${TESTS_DIR}/cmake/host_tests.cmake")
run_host_tests("${TESTS_DIR}")
