# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

add_subdirectory_ifdef(CONFIG_MCP9808_MPP mcp9808)
