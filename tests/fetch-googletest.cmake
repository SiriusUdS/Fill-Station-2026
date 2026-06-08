# ---------------------------------------------------------------------------
# Download a pinned GoogleTest release into tests/third_party/googletest.
#
# GoogleTest is NOT committed (it's git-ignored) to keep the repo light; this
# script fetches it on demand. Driven by CMake so it works on every platform
# with no curl/unzip/tar dependency:
#
#     cmake -P tests/fetch-googletest.cmake          # download if missing
#     cmake -P tests/fetch-googletest.cmake force     # re-download even if present
#
# run.cmake invokes this automatically, so you normally never call it directly.
# To bump the version: change GTEST_VERSION + GTEST_SHA256 (the release asset's
# SHA-256, printed by the GitHub release page or `sha256sum`).
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

set(GTEST_VERSION 1.15.2)
set(GTEST_SHA256  7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926)

get_filename_component(TESTS_DIR "${CMAKE_SCRIPT_MODE_FILE}" DIRECTORY)
set(THIRD_PARTY "${TESTS_DIR}/third_party")
set(DEST        "${THIRD_PARTY}/googletest")

# `cmake -P fetch-googletest.cmake force` re-downloads even if already present.
set(FORCE FALSE)
foreach(i RANGE ${CMAKE_ARGC})
    if(DEFINED CMAKE_ARGV${i} AND "${CMAKE_ARGV${i}}" STREQUAL "force")
        set(FORCE TRUE)
    endif()
endforeach()

if(EXISTS "${DEST}/CMakeLists.txt" AND NOT FORCE)
    return()  # already downloaded; nothing to do
endif()
if(EXISTS "${DEST}")
    file(REMOVE_RECURSE "${DEST}")
endif()

set(URL "https://github.com/google/googletest/releases/download/v${GTEST_VERSION}/googletest-${GTEST_VERSION}.tar.gz")
set(ARCHIVE "${THIRD_PARTY}/googletest-${GTEST_VERSION}.tar.gz")

message(STATUS "Downloading GoogleTest ${GTEST_VERSION} ...")
file(DOWNLOAD "${URL}" "${ARCHIVE}"
    EXPECTED_HASH SHA256=${GTEST_SHA256}
    STATUS download_status)
list(GET download_status 0 download_code)
if(NOT download_code EQUAL 0)
    list(GET download_status 1 download_msg)
    file(REMOVE "${ARCHIVE}")
    message(FATAL_ERROR "GoogleTest download failed: ${download_msg}")
endif()

message(STATUS "Extracting GoogleTest ...")
# Yields third_party/googletest-<version>/; rename to the stable googletest/ path.
file(ARCHIVE_EXTRACT INPUT "${ARCHIVE}" DESTINATION "${THIRD_PARTY}")
file(RENAME "${THIRD_PARTY}/googletest-${GTEST_VERSION}" "${DEST}")
file(REMOVE "${ARCHIVE}")

message(STATUS "GoogleTest ${GTEST_VERSION} ready at ${DEST}")
