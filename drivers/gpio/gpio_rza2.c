/*
 * Copyright 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r7s9210_gpio

#include <soc.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>

#include "gpio_rza2.h"

#define RZA2_PODR(port) (0x0000 + (port)) /* Output Data 8-bit */
#define RZA2_PIDR(port) (0x0020 + (port)) /* Input Data 8-bit */

DEVICE_MMIO_TOPLEVEL_STATIC(reg_base, DT_DRV_INST(0));
#define RZA2_REG_BASE DEVICE_MMIO_TOPLEVEL_GET(reg_base)

int rza2_chip_get(uint8_t port)
{
	return sys_read8(RZA2_REG_BASE + RZA2_PIDR(port));
}

void rza2_chip_set(uint8_t port, uint8_t value)
{
	sys_write8(value, RZA2_REG_BASE + RZA2_PODR(port));
}

__boot_func static int gpio_rza2_driver_init(void)
{
	DEVICE_MMIO_TOPLEVEL_MAP(reg_base, K_MEM_CACHE_NONE);
	return 0;
}

SYS_INIT(gpio_rza2_driver_init, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY);
