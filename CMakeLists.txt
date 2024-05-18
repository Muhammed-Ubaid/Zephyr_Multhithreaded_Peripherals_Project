# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky)

add_subdirectory("GPIOs")
add_subdirectory("Embedded_C_Design_Patterns")
add_subdirectory("i2c")
add_subdirectory("Generic_Declarations")

target_sources(app PRIVATE src/main.c)
