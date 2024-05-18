/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mcp9808.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MCP9808_MPP, LOG_LEVEL_COMPLETE);

/**
 * @brief : Private instance of i2c0 data for mcp9808
 */
static uint8_t i2c_buffer[2];
static int err = 0;

/**
 * @brief : MCP9808 init & device handle get API
 */
bool mcp9808_Init()
{
    if (i2c0_mcp9808 == NULL)
    {
        LOG_ERR("\n mcp9808_Init Failed, device binding is NULL \n");
    }
    else
    {
        LOG_INF("\n mcp9808_Init device binding successful \n");
    }
}

float mcp9808_CalculateTemp(uint8_t upperByte, uint8_t lowerByte)
{
    float temperature = (upperByte & 0x0F) * 16 + (float)(lowerByte) / 16;
    // if temperature less than 0deg C
    if ((upperByte & 0x10) == 0x10)
    {
        temperature = 256 - temperature; // 2's compliment, subtract with 2^8
    }

    return temperature;
}

/**
 * @brief : Read & return temperature from MCP9808 over i2C
 */
float mcp9808_Read_Temperature()
{
    float l_temperature = 0.0;
    err = i2c_write(i2c0_mcp9808, i2c_buffer, 1, MCP9808_I2C_ADDRESS);
    if (err != SUCCESS)
    {
        LOG_ERR("i2c_write failed with %d", err);
        return err;
    }

    err = i2c_read(i2c0_mcp9808, i2c_buffer, 2, MCP9808_I2C_ADDRESS);
    if (err != SUCCESS)
    {
        LOG_ERR("i2c_read failed with %d", err);
        return err;
    }

    l_temperature = mcp9808_CalculateTemp(i2c_buffer[0], i2c_buffer[1]);
    LOG_INF("\n Calculated temperature is %.2f Cel \n", l_temperature);

    return l_temperature;
}