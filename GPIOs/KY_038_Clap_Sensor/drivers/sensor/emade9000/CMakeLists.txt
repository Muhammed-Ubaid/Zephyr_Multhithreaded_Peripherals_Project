# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_EMADE9000 emade9000.c)
zephyr_library_sources_ifdef(CONFIG_EMADE9000 emade9000_reg_map.c)