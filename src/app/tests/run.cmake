# ---------------------------------------------------------------------------
# Configure, build and run the full host test suite (all sites: common + boards).
#
#     cmake -P src/app/tests/run.cmake            # configure + build + test
#     cmake -P src/app/tests/run.cmake clean      # wipe the build dir first
#
# The harness logic lives once in cmake/host_tests.cmake; this entry point only
# names which test project to run.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

get_filename_component(TESTS_DIR "${CMAKE_SCRIPT_MODE_FILE}" DIRECTORY)
include("${TESTS_DIR}/cmake/host_tests.cmake")
run_host_tests("${TESTS_DIR}")
