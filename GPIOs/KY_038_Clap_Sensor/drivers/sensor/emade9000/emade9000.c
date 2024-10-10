/* emade9000.c - Driver for energy meter */

/*
 * Copyright (c) 2024, ubaid
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include "emade9000.h"

#define DT_DRV_COMPAT zephyr_emade9000

LOG_MODULE_REGISTER(EMADE9000, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "EMADE9000 driver enabled without any devices"
#endif

static void emade9000_raise_int_flag(const struct device *dev, int bit)
{
	struct ade9000_data *data = dev->data;

	atomic_set_bit(&data->interrupt_flag, bit);
}

static void emade9000_cf4_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct ade9000_data *data = CONTAINER_OF(cb, struct ade9000_data, cf4_cb);
	emade9000_raise_int_flag(data->dev, ADE9000_INT_FLAG_CF4);
}

static void emade9000_irq1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct ade9000_data *data = CONTAINER_OF(cb, struct ade9000_data, irq1_cb);
	emade9000_raise_int_flag(data->dev, ADE9000_INT_FLAG_IRQ1);
}

static int emade9000_init_int_pin(const struct gpio_dt_spec *pin, struct gpio_callback *pin_cb, gpio_callback_handler_t handler)
{
	int ret;

	if (!pin->port) {
		return 0;
	}

	if (!device_is_ready(pin->port)) {
		LOG_DBG("%s not ready", pin->port->name);
		return -ENODEV;
	}

	gpio_init_callback(pin_cb, handler, BIT(pin->pin));

	ret = gpio_pin_configure_dt(pin, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(pin, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		return ret;
	}

	ret = gpio_add_callback(pin->port, pin_cb);
	if (ret) {
		return ret;
	}

	return 0;
}

static int emade9000_init_interrupts(const struct device *dev)
{
	const struct ade9000_config *cfg = dev->config;
	struct ade9000_data *data = dev->data;
	int ret;

    //configure if seperate thread required
    
    //configure if seperate work queue
	

	ret = emade9000_init_int_pin(&cfg->gpio_cf4, &data->cf4_cb, emade9000_cf4_callback);
	if (ret)
    {
		LOG_ERR("Failed to initialize cf4");
		return -EINVAL;
	}

	ret = emade9000_init_int_pin(&cfg->gpio_irq1, &data->irq1_cb, emade9000_irq1_callback);
	if (ret) {
		LOG_ERR("Failed to initialize irq1");
		return -EINVAL;
	}

    /* write control register data if you have any... */
	return 0;
}

static inline int emade9000_bus_check(const struct device *dev)
{
    struct ade9000_config *cfg = dev->config;

	return spi_is_ready_dt(&(cfg->spi)) ? 0 : -ENODEV;
}

static int emade9000_init(const struct device *dev)
{
	int ret;
	struct emade9000_data *data = dev->data;
	uint8_t chip_id;
	uint8_t soft_reset_cmd;
	uint8_t init_ctrl;
	uint8_t msg;
	uint8_t tries;
	uint8_t adv_pwr_save;

	ret = emade9000_bus_check(dev);
	if (ret < 0) {
		LOG_ERR("Could not initialize bus");
		return ret;
	}

    /* add all init configuration here
        like parameter set
     */

    /*  add interrupts here */
	ret = emade9000_init_interrupts(dev);
	if (ret)
    {
		LOG_ERR("emade9000_init_interrupts returned %d", ret);
		return ret;
	}

	return ret;
}

#ifdef CONFIG_PM_DEVICE
/**
 * @brief Set the Device Power Management State.
 *
 * @param dev - The device structure.
 * @param action - power management state
 * @retval 0 on success
 * @retval -ENOTSUP if an unsupported action is given
 *
 */
static int emade9000_pm_ctrl(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	
    default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static int emade9000_read_spi(const struct device *dev, uint16_t addr, uint8_t values[], uint8_t rx_size)
{
	int ret = 0;

	uint16_t act_addr = (((addr << 4) & 0xFFF0)+ ADE9000_READ_MASK_VALUE);
	const struct ade9000_config *cfg = dev->config;

	uint8_t tx_buffer[2];
	tx_buffer[0] = addr >> 8;
	tx_buffer[1] = addr;

	struct spi_buf tx_spi_bufs[] = {
		{
			.buf = tx_buffer, 
			.len = sizeof(tx_buffer)
		},
	};

	struct spi_buf_set spi_tx_buffer_set = {
		.buffers = tx_spi_bufs,
		.count = 1
	};

	struct spi_buf rx_spi_bufs[] = {
		{
			.buf = values,
			.len = rx_size
		},
	};

	struct spi_buf_set spi_rx_buffer_set = {
			.buffers = rx_spi_bufs,
			.count = 1
	};    

	//gpio_pin_set(gpio0_dev, GPIO_0_CS,0);

	do
	{
		ret = spi_transceive_dt(&cfg->spi, &spi_tx_buffer_set, &spi_rx_buffer_set);
		//ret = spi_write(cfg->spi, &spi_cfg, &spi_tx_buffer_set);
		//if(ret<0){break;}
		//ret = spi_read(cfg->spi, &spi_cfg, &spi_rx_buffer_set);
	} while (false);

	//gpio_pin_set(gpio0_dev, GPIO_0_CS,1);

	if(ret != 0)
	{
		printk("spi read failed\n");
	}

	return ret;        
}

static int emade9000_readVal16(const struct device *dev, uint16_t addr, uint16_t *values)
{
	return emade9000_read_spi(dev, addr, (uint8_t)*values, 2);
}

static int emade9000_readVal32(const struct device *dev, uint16_t addr, uint32_t *values)
{
	return emade9000_read_spi(dev, addr, (uint8_t)*values, 4);
}

static int emade9000_write_spi(const struct device *dev, uint16_t addr, uint8_t values[], uint8_t tx_size)
{
	int ret;
	const struct ade9000_config *cfg = dev->config;
	const struct spi_buf tx_buf[2] = {
		{.buf = (uint8_t *)addr, .len = sizeof(addr)},
		{.buf = (uint8_t *)values, .len = tx_size}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf)
	};

	ret = spi_write_dt(&cfg->spi, &tx);
	if (ret < 0) {
		LOG_ERR("spi_write_dt failed %i", ret);
		return ret;
	}

	return ret;
}

static int emade9000_writeVal16(const struct device *dev, uint16_t addr, uint16_t *values)
{
	return emade9000_write_spi(dev, addr, (uint8_t)*values, 2);
}

static int emade9000_writeVal32(const struct device *dev, uint16_t addr, uint32_t *values)
{
	return emade9000_write_spi(dev, addr, (uint8_t)*values, 4);
}

static int emade9000_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    int ret;

    return ret;
}

static int emade9000_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct ade9000_data *data = dev->data;

    /*
	if (chan != SENSOR_CHAN_DISTANCE) {
		return -ENOTSUP;
	}
	val->val1 = (uint32_t) (data->data / (uint16_t) 1000);
	val->val2 = (uint32_t) ((data->data % 1000) * 1000);
	*/
    return 0;
}

static int emade9000_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
    int ret;

    return ret;
}

static const struct sensor_driver_api emade9000_api_funs = {
	.sample_fetch = emade9000_sample_fetch,
	.channel_get = emade9000_channel_get,
    .attr_set = emade9000_attr_set,
	//.attr_get = ;
	//.trigger_set = ;
	//.get_decoder = ;
	//.submit = ;
};

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define EMADE9000_DEFINE(inst)						\
	static struct ade9000_data ade9000_data_##inst;			\
	static struct ade9000_config ade9000_config_##inst ={			\
		.spi = SPI_DT_SPEC_INST_GET(inst, ADE9000_SPI_OPERATION, 0),	\
		.gpio_cf4 = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, cf4_gpios, 0, {}),\
		.gpio_irq1 = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, irq1_gpios, 0, {}),\
	}; \
    PM_DEVICE_DT_DEFINE(DT_DRV_INST(0), ade9000_pm_ctrl);\
    SENSOR_DEVICE_DT_INST_DEFINE(inst, emade9000_init, NULL, \
                    &ade9000_data_##inst, &ade9000_config_##inst, POST_KERNEL, \
		      CONFIG_SENSOR_INIT_PRIORITY, &emade9000_api_funs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(EMADE9000_DEFINE)

//GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, cf4_gpios, 0, {}),
//.gpio_cf4 = GPIO_DT_SPEC_INST_GET(inst, cf4_gpios),