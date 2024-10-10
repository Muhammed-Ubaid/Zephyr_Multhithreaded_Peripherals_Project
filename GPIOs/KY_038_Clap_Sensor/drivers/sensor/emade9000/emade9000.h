#ifndef ZEPHYR_DRIVERS_SENSOR_EMADE9000_EMADE9000_H_
#define ZEPHYR_DRIVERS_SENSOR_EMADE9000_EMADE9000_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define ADE9000_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

#define ADE9000_READ_MASK_VALUE 8

enum ADE9000_INT_FLAGS
{
    ADE9000_INT_FLAG_NONE=0,
	ADE9000_INT_FLAG_CF4=1,
    ADE9000_INT_FLAG_IRQ1=2,
};

struct ade9000_data
{
    const struct device *dev;
    const struct device *gpio_ss;
    const struct device *gpio_cf4;
    const struct device *gpio_irq1;

    struct gpio_callback cf4_cb;
	struct gpio_callback irq1_cb;
    uint8_t test1;
    uint8_t interrupt_flag;
};

struct ade9000_config
{
    struct spi_dt_spec spi;
    struct gpio_dt_spec gpio_cf4;
    struct gpio_dt_spec gpio_irq1;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_EMADE9000_EMADE9000_H_ */
