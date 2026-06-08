cmake_minimum_required(VERSION 3.22)
include(ExternalProject)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Set the project name
set(CMAKE_PROJECT_NAME FillStation)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Single-core mode: only build the CM7 application and skip CM4.
# mx-generated.cmake builds each core whose name matches BUILD_CONTEXT (or all
# of them when BUILD_CONTEXT is undefined). Override with -DBUILD_CONTEXT=CM4CM7
# to restore the dual-core build.
if(NOT DEFINED BUILD_CONTEXT)
    set(BUILD_CONTEXT CM7)
endif()

# Include external project file
include("mx-generated.cmake")

