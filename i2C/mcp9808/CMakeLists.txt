# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

target_sources(app PRIVATE mcp9808.c)

target_include_directories(app PUBLIC "./")
