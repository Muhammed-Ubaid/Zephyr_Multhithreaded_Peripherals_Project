# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(MPP)

add_subdirectory("GPIOs")
add_subdirectory("Embedded_C_Design_Patterns")
add_subdirectory("i2c")
add_subdirectory("Generic_Declarations")
add_subdirectory("Bit_Manipulation")
add_subdirectory("Embedded_C_DSA")
add_subdirectory("SPI")


target_sources(app PRIVATE src/main.c)
