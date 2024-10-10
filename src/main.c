/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
// #include <mcp9808.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

int main(void)
{
	// bool is_init = mcp9808_Init();
	// float calculated_Temp = mcp9808_Read_Temperature();
    const struct device *const ky038 = DEVICE_DT_GET_ONE(zephyr_ky038);

	if (!device_is_ready(ky038)) {
		printf("Device %s is not ready\n", ky038->name);
		return 0;
	}

    	while (true) {
		int rc = sensor_sample_fetch(ky038);

		if (rc != 0) {
			printf("Sensor fetch failed: %d\n", rc);
			break;
		}
        }
	return 0;
}
