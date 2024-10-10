# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

add_subdirectory_ifdef(CONFIG_INTG_LED2COLOR LED_2_Color)
add_subdirectory_ifdef(CONFIG_KY_038 KY_038_Clap_Sensor)