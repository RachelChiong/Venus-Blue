# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(prac2)

target_sources(app PRIVATE src/main.c)

zephyr_include_directories(../../include)

file(GLOB MYLIB_SOURCES ../../mylib/*.c)

foreach(mylib_src ${MYLIB_SOURCES})
    zephyr_library_sources(${mylib_src})
endforeach()