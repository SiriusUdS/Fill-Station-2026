# ---------------------------------------------------------------------------
# Configure, build and run the full host test suite (parent + stm-2026-common).
#
#     cmake -P tests/run.cmake            # configure + build + test
#     cmake -P tests/run.cmake clean      # wipe the build dir first
#
# The harness logic is NOT kept here — it lives once in the submodule
# (stm-2026-common/tests/cmake/host_tests.cmake) and we reuse it. This entry
# point only names which test project to run.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

get_filename_component(TESTS_DIR "${CMAKE_SCRIPT_MODE_FILE}" DIRECTORY)
include("${TESTS_DIR}/../stm-2026-common/tests/cmake/host_tests.cmake")
run_host_tests("${TESTS_DIR}")
