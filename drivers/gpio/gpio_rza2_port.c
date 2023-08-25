
/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT renesas_r7s9210_gpio_port

#include <errno.h>
#include <soc.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>

#include "gpio_rza2.h"

struct gpio_rza2_port_config {
	uint8_t port;
};

static void rza2_pin_to_gpio(uint8_t port, uint8_t pin, uint8_t dir)
{
	pinctrl_soc_pin_t p;
	p.pin = pin;
	p.port = port;
	p.func = dir;

	pinctrl_configure_pins(&p, 1, PINCTRL_REG_NONE);
}

static int rza2_chip_direction_input(uint8_t port, uint8_t pin)
{
	rza2_pin_to_gpio(port, pin, GPIO_FUNC_INPUT);

	return 0;
}

static int rza2_chip_direction_output(uint8_t port, uint8_t pin)
{
	rza2_chip_set(port, pin);
	rza2_pin_to_gpio(port, pin, GPIO_FUNC_OUTPUT);

	return 0;
}

static int rza2_gpio_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	int ret = 0;
	const struct gpio_rza2_port_config *config = port->config;

	if ((flags & GPIO_OUTPUT) && (flags & GPIO_INPUT)) {
		/* Pin cannot be configured as input and output */
		return -ENOTSUP;
	} else if (!(flags & (GPIO_INPUT | GPIO_OUTPUT))) {
		/* Pin has to be configured as input or output */
		return -ENOTSUP;
	}

	if (flags & GPIO_OUTPUT) {
		ret = rza2_chip_direction_input(config->port, pin);
	} else {
		ret = rza2_chip_direction_output(config->port, pin);
	}

	return ret;
}

static int rza2_gpio_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	const struct gpio_rza2_port_config *config = port->config;

	*value = rza2_chip_get(config->port);
	return 0;
}

static int rza2_gpio_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t base_value = rza2_chip_get(config->port);

	rza2_chip_set(config->port, (base_value & ~mask) | (value & mask));
	return 0;
}

static int rza2_gpio_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t base_value = rza2_chip_get(config->port);

	rza2_chip_set(config->port, (base_value | pins));
	return 0;
}

static int rza2_gpio_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t base_value = rza2_chip_get(config->port);

	rza2_chip_set(config->port, (base_value & ~pins));
	return 0;
}

static int rza2_gpio_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_rza2_port_config *config = port->config;
	uint8_t base_value = rza2_chip_get(config->port);

	rza2_chip_set(config->port, (base_value ^ pins));
	return 0;
}

static const struct gpio_driver_api rza2_gpio_driver_api = {
	.pin_configure = rza2_gpio_configure,
	.port_get_raw = rza2_gpio_port_get_raw,
	.port_set_masked_raw = rza2_gpio_port_set_masked_raw,
	.port_set_bits_raw = rza2_gpio_port_set_bits_raw,
	.port_clear_bits_raw = rza2_gpio_port_clear_bits_raw,
	.port_toggle_bits = rza2_gpio_port_toggle_bits,
};

static int gpio_rza2_port_init(const struct device *dev)
{
	return 0;
}

/* RZA2 GPIO port driver must be initialized after RZA2 PINCTRL driver */
BUILD_ASSERT(CONFIG_GPIO_RZA2_PORT_INIT_PRIORITY > CONFIG_PINCTRL_RZA2_INIT_PRIORITY);

#define GPIO_RZA2_PORT_INIT(inst)					\
	static const struct gpio_rza2_port_config gpio_rza2_cfg_##inst = { \
		.port = DT_INST_REG_ADDR(inst)				\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, gpio_rza2_port_init, NULL, NULL,	\
			      &gpio_rza2_cfg_##inst, POST_KERNEL,	\
			      CONFIG_GPIO_RZA2_PORT_INIT_PRIORITY, &rza2_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_RZA2_PORT_INIT)
