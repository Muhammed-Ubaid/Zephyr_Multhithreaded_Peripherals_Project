/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <mcp9808.h>

int main(void)
{
	bool is_init = mcp9808_Init();
	float calculated_Temp = mcp9808_Read_Temperature();
	return 0;
}
