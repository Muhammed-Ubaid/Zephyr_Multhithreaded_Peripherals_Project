/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "bme280_mpp.h"
#include <zephyr/drivers/spi.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static struct bme280_Reg_Data bme280_data = {0};

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/**
 * @brief : Get SPI-1 binding
 */
static const struct device *SPI1 = DEVICE_DT_GET(DT_NODELABEL(spi1));

/**
 * @brief : Configure and get CS for BME280 over SPI1
 */
#define BME280_CS 7
static const struct device *BME_CS = DEVICE_DT_GET(DT_NODELABEL(gpio1));

/**
 * @brief : Configure SPI communication
 */
static struct spi_config BME280_SPI =
	{
		.frequency = 125000U,
		.operation = SPI_WORD_SET(8),
		.slave = 0,
		// .cs = &BME_CS
};

static void readBME280Register(uint8_t reg, uint8_t values[], uint8_t size)
{
	int err;

	uint8_t tx_buffer[1];
	tx_buffer[0] = reg;

	struct spi_buf tx_spi_bufs[] =
		{
			{.buf = tx_buffer,
			 .len = sizeof(tx_buffer)},
		};

	struct spi_buf_set spi_tx_buffer_set =
		{
			.buffers = tx_spi_bufs,
			.count = 1};

	struct spi_buf rx_spi_bufs[] =
		{
			{.buf = values,
			 .len = size},
		};

	struct spi_buf_set spi_rx_buffer_set =
		{
			.buffers = rx_spi_bufs,
			.count = 1};

	/*Set CS to low and enable*/
	gpio_pin_set(BME_CS, BME280_CS, 0);
	do
	{
		err = spi_write(SPI1, &BME280_SPI, &spi_tx_buffer_set);
		if (err < 0)
		{
			printk("\n SPI write error \n");
			break;
		}

		err = spi_read(SPI1, &BME280_SPI, &spi_rx_buffer_set);
	} while (false);

	gpio_pin_set(BME_CS, BME280_CS, 1);

	if (err < 0)
	{
		printk("\n SPI read reg error \n");
	}
}

static void readChipID(void)
{
	uint8_t rx_buf_chipid[1];
	readBME280Register(BME280_DEVID_REG, rx_buf_chipid, 1);
	bme280_data.chip_id = rx_buf_chipid[0];
	printk("\n received device ID : 0x%02X \n", bme280_data.chip_id);
}

static void readCaliRegs(void)
{
	uint8_t values[2];

	readBME280Register(BME280_REGISTER_DIG_T1, values, 2);
	bme280_data.dig_t1 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_T2, values, 2);
	bme280_data.dig_t2 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_T3, values, 2);
	bme280_data.dig_t3 = (((uint16_t)values[1]) << 8) | values[0];

	readBME280Register(BME280_REGISTER_DIG_P1, values, 2);
	bme280_data.dig_p1 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_P2, values, 2);
	bme280_data.dig_p2 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_P3, values, 2);
	bme280_data.dig_p3 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_P4, values, 2);
	bme280_data.dig_p4 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_P5, values, 2);
	bme280_data.dig_p5 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_P6, values, 2);
	bme280_data.dig_p6 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_P7, values, 2);
	bme280_data.dig_p7 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_P8, values, 2);
	bme280_data.dig_p8 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_P9, values, 2);
	bme280_data.dig_p9 = (((uint16_t)values[1]) << 8) | values[0];

	readBME280Register(BME280_REGISTER_DIG_H1, values, 1);
	bme280_data.dig_h1 = values[0];
	readBME280Register(BME280_REGISTER_DIG_H2, values, 2);
	bme280_data.dig_h2 = (((uint16_t)values[1]) << 8) | values[0];
	readBME280Register(BME280_REGISTER_DIG_H3, values, 2);
	bme280_data.dig_h3 = values[0];
	readBME280Register(BME280_REGISTER_DIG_H4, values, 2);
	bme280_data.dig_h4 = (((uint16_t)values[0]) << 4) | (values[1] & 0x0F);
	readBME280Register(BME280_REGISTER_DIG_H5, values, 2);
	bme280_data.dig_h5 = (((uint16_t)values[1]) << 4) | ((values[0] >> 4) & 0x0F);
	readBME280Register(BME280_REGISTER_DIG_H6, values, 2);
	bme280_data.dig_h6 = values[0];

	printk("\n T1: %d T2: %d T3:%d \n", bme280_data.dig_t1, bme280_data.dig_t2, bme280_data.dig_t3);
	printk("\n P1: %d P2: %d P3:%d \n", bme280_data.dig_p1, bme280_data.dig_p2, bme280_data.dig_p3);
	printk("\n P4: %d P5: %d P6:%d \n", bme280_data.dig_p4, bme280_data.dig_p5, bme280_data.dig_p6);
	printk("\n P7: %d P8: %d P9:%d \n", bme280_data.dig_p7, bme280_data.dig_p8, bme280_data.dig_p9);
	printk("\n H1: %d H2: %d H3:%d \n", bme280_data.dig_h1, bme280_data.dig_h2, bme280_data.dig_h3);
	printk("\n H4: %d H5: %d H6:%d \n", bme280_data.dig_h4, bme280_data.dig_h5, bme280_data.dig_h6);
}

static void setBME280Register(uint8_t reg, uint8_t value)
{
	int err;
	uint8_t tx_values[] = {(reg & 0x7F), value};

	struct spi_buf tx_spi_bufs[] = {
		{.buf = tx_values, .len = sizeof(tx_values)}};

	struct spi_buf_set spi_tx_buffer_set = {
		.buffers = tx_spi_bufs,
		.count = 1};

	gpio_pin_set(BME_CS, BME280_CS, 0);
	err = spi_write(SPI1, &BME280_SPI, &spi_tx_buffer_set);
	gpio_pin_set(BME_CS, BME280_CS, 1);

	if (err < 0)
	{
		printk("\n Set Reg failed \n");
	}
}

static void readSensorValues(void)
{
	int err;

	uint8_t tx_buf_reg[] = {0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF};
	uint8_t rx_buf_values[sizeof(tx_buf_reg)];

	struct spi_buf tx_spi_bufs[] =
		{
			{.buf = tx_buf_reg,
			 .len = sizeof(tx_buf_reg)},
		};

	struct spi_buf_set spi_tx_buffer_set =
		{
			.buffers = tx_spi_bufs,
			.count = 1};

	struct spi_buf rx_spi_bufs[] =
		{
			{.buf = rx_buf_values,
			 .len = sizeof(rx_buf_values)},
		};

	struct spi_buf_set spi_rx_buffer_set =
		{
			.buffers = rx_spi_bufs,
			.count = 1};

	gpio_pin_set(BME_CS, BME280_CS, 0);
	err = spi_transceive(SPI1, &BME280_SPI, &spi_tx_buffer_set, &spi_rx_buffer_set);
	gpio_pin_set(BME_CS, BME280_CS, 1);

	if(err < 0)
	{
		printk("Read sensor val error");
	}
	else
	{
		bme280_data.ucomp_press = ((rx_buf_values[1] << 12) | (rx_buf_values[2] << 4) | (rx_buf_values[3] >> 4));
		bme280_data.ucomp_temp = ((rx_buf_values[4] << 12) | (rx_buf_values[5] << 4) | (rx_buf_values[6] >> 4));
		bme280_data.ucomp_hum = (rx_buf_values[7] << 8) | rx_buf_values[8];
		printk("\n ucomp temp : %d ,ucomp press : %d , ucomp hum : %d \n",bme280_data.ucomp_temp,bme280_data.ucomp_press,bme280_data.ucomp_hum);
	}
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature( )
{
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = (((double)bme280_data.ucomp_temp) / 16384.0 - ((double)bme280_data.dig_t1) / 1024.0);
    var1 = var1 * ((double)bme280_data.dig_t2);
    var2 = (((double)bme280_data.ucomp_temp) / 131072.0 - ((double)bme280_data.dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)bme280_data.dig_t3);
    bme280_data.t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
static double compensate_pressure( )
{
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)bme280_data.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)bme280_data.dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)bme280_data.dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)bme280_data.dig_p4) * 65536.0);
    var3 = ((double)bme280_data.dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)bme280_data.dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)bme280_data.dig_p1);

    /* Avoid exception caused by division by zero */
    if (var1 > (0.0))
    {
        pressure = 1048576.0 - (double) bme280_data.ucomp_press;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)bme280_data.dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)bme280_data.dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)bme280_data.dig_p7)) / 16.0;

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }

    return (pressure/100);
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
static double compensate_humidity( )
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)bme280_data.t_fine) - 76800.0;
    var2 = (((double)bme280_data.dig_h4) * 64.0 + (((double)bme280_data.dig_h5) / 16384.0) * var1);
    var3 = bme280_data.ucomp_hum - var2;
    var4 = ((double)bme280_data.dig_h2) / 65536.0;
    var5 = (1.0 + (((double)bme280_data.dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)bme280_data.dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)bme280_data.dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
    else if (humidity < humidity_min)
    {
        humidity = humidity_min;
    }

    return humidity;
}

int main(void)
{
	int ret;
	double tempra, prs, humid;
	gpio_pin_configure(BME_CS, BME280_CS, GPIO_OUTPUT);
	gpio_pin_set(BME_CS, BME280_CS, 1);

	if (!device_is_ready(SPI1))
	{
		printk("\n SPI not ready \n");
	}

	readChipID();
	k_msleep(SLEEP_TIME_MS);
	readCaliRegs();
	k_msleep(2 * SLEEP_TIME_MS);
	readSensorValues();
	k_usleep(10);

	setBME280Register(BME280_CTRL_HUM_REG, 0x04); //Set humidity 8x oversampling rate to start ADC sampling
	setBME280Register(BME280_CTRL_MEAS_REG, 0x93); // Set temp & pressure 8x oversampling rate to start ADC sampling

	while (1)
	{
		readCaliRegs();
		k_msleep(SLEEP_TIME_MS);
		tempra = compensate_temperature( );
		prs = compensate_pressure( );
		humid = compensate_humidity( );
		printk("\n temperature : %f \n",tempra );
		printk("\n Pressure : %f \n",prs );
		printk("\n Humidity : %f \n",humid );
	}

	return 0;
}
