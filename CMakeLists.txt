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

# CM7-only build — neither board uses the CM4 core. This is the DURABLE control
# point: CubeMX does NOT regenerate this file, whereas it DOES regenerate
# mx-generated.cmake (whose CM4 ExternalProject is gated on BUILD_CONTEXT). Forcing
# CM7 here keeps that CM4 block inert even after a CubeMX regen, so we never edit the
# generated file. The CM4/ project dir is untracked/.gitignore'd; if a regen recreates
# it, it is ignored and never built.
if(NOT DEFINED BUILD_CONTEXT)
    set(BUILD_CONTEXT CM7)
endif()

# Include external project file
include("mx-generated.cmake")

