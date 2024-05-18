/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <generic_declarations.h>

/**
 * @brief : delay if needed
 */
#define MCP9808_SLEEP_MS 1000

/**
 * @brief : MCP9808 register definition
 */
#define MCP9808_I2C_ADDRESS 0x18
#define MCP9808_TEMPERATURE_REGISTER 0x05

/**
 * @brief : MCP9808 i2C handle definition
 */
#define I2C_NODE DT_NODELABEL(i2c0)
static const struct device *i2c0_mcp9808 = DEVICE_DT_GET(I2C_NODE);

/**
 * @brief : MCP9808 init & device handle get API
 */
bool mcp9808_Init();

/**
 * @brief : Read & return temperature from MCP9808 over i2C
 */
float mcp9808_Read_Temperature();
