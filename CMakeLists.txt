# Copyright (c) CATIE 2023
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(MCP3425_driver)

target_sources(app PRIVATE src/main.c)
#FILE(GLOB app_sources src/*.c)

# add custom subdirectory for WIP 6tron driver
add_subdirectory(drivers)

#set(CMAKE_COLOR_DIAGNOSTICS ON)
#set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
#set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
