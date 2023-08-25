/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r7s9210_pinctrl

#include <errno.h>
#include <soc.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>

#define RZA2_PDR(port)      (0x0000 + (port)*2)           /* Direction 16-bit */
#define RZA2_PMR(port)      (0x0000 + (port))             /* Mode 8-bit */
#define RZA2_PFS(port, pin) (0x0180 + ((port)*8) + (pin)) /* Fnct 8-bit */

#define RZA2_PWPR   0x02ff /* Write Protect 8-bit */
#define RZA2_PFENET 0x0820 /* Ethernet Pins 8-bit */
#define RZA2_PPOC   0x0900 /* Dedicated Pins 32-bit */
#define RZA2_PHMOMO 0x0980 /* Peripheral Pins 32-bit */
#define RZA2_PCKIO  0x09d0 /* CKIO Drive 8-bit */

#define RZA2_PDR_INPUT  0x02
#define RZA2_PDR_OUTPUT 0x03
#define RZA2_PDR_MASK   0x03

#define PWPR_PFSWE BIT(6) /* PFS Register Write Enable */

#define RZA2_PDR_REG DT_INST_REG_ADDR_BY_NAME(0, pdr)
#define RZA2_PDR_SIZE DT_INST_REG_SIZE_BY_NAME(0, pdr)
#define RZA2_PINCTRL_REG DT_INST_REG_ADDR_BY_NAME(0, pinctrl)
#define RZA2_PINCTRL_SIZE DT_INST_REG_SIZE_BY_NAME(0, pinctrl)

static struct rza2_pinctrl_data {
	mm_reg_t pdr_addr;
	mm_reg_t pinctrl_addr;
	struct k_mutex lock;
} rza2_pinctrl_data;

static void rza2_set_pin_function(uint8_t port, uint8_t pin, uint8_t func)
{
	uint16_t mask16;
	uint16_t reg16;
	uint8_t reg8;

	k_mutex_lock(&rza2_pinctrl_data.lock, K_FOREVER);

	/* Set pin to 'Non-use (Hi-z input protection)'  */
	reg16 = sys_read8(rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));
	mask16 = RZA2_PDR_MASK << (pin * 2);
	reg16 &= ~mask16;
	sys_write16(reg16, rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));

	/* Temporarily switch to GPIO */
	reg8 = sys_read8(rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));
	reg8 &= ~BIT(pin);
	sys_write8(reg8, rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));

	/* PFS Register Write Protect : OFF */
	sys_write8(0x00, rza2_pinctrl_data.pinctrl_addr + RZA2_PWPR);	      /* B0WI=0, PFSWE=0 */
	sys_write8(PWPR_PFSWE, rza2_pinctrl_data.pinctrl_addr + RZA2_PWPR);   /* B0WI=0, PFSWE=1 */

	/* Set Pin function (interrupt disabled, ISEL=0) */
	sys_write8(func, rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));

	/* PFS Register Write Protect : ON */
	sys_write8(0x00, rza2_pinctrl_data.pinctrl_addr + RZA2_PWPR);	/* B0WI=0, PFSWE=0 */
	sys_write8(0x80, rza2_pinctrl_data.pinctrl_addr + RZA2_PWPR);	/* B0WI=1, PFSWE=0 */

	/* Port Mode  : Peripheral module pin functions */
	reg8 = sys_read8(rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));
	reg8 |= BIT(pin);
	sys_write8(reg8, rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));

	k_mutex_unlock(&rza2_pinctrl_data.lock);
}

static void rza2_pin_to_gpio(uint8_t port, uint8_t pin, uint8_t dir)
{
	uint16_t mask16;
	uint16_t reg16;

	k_mutex_lock(&rza2_pinctrl_data.lock, K_FOREVER);

	reg16 = sys_read16(rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));
	mask16 = RZA2_PDR_MASK << (pin * 2);
	reg16 &= ~mask16;
	reg16 |= dir << (pin * 2);

	sys_write16(reg16, rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));

	k_mutex_unlock(&rza2_pinctrl_data.lock);
}

static int pinctrl_configure_pin(const pinctrl_soc_pin_t pin)
{
	if (pin.func == GPIO_FUNC_INPUT) {
		rza2_pin_to_gpio(pin.port, pin.pin, RZA2_PDR_INPUT);
	} else if (pin.func == GPIO_FUNC_OUTPUT) {
		rza2_pin_to_gpio(pin.port, pin.pin, RZA2_PDR_OUTPUT);
	} else {
		rza2_set_pin_function(pin.port, pin.pin, pin.func);
	}
	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	int ret = 0;

	ARG_UNUSED(reg);
	while (pin_cnt-- > 0U) {
		ret = pinctrl_configure_pin(*pins++);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

__boot_func static int pinctrl_rza2_driver_init(void)
{
	device_map(&rza2_pinctrl_data.pdr_addr, RZA2_PDR_REG, RZA2_PDR_SIZE, K_MEM_CACHE_NONE);
	device_map(&rza2_pinctrl_data.pinctrl_addr, RZA2_PINCTRL_REG, RZA2_PINCTRL_SIZE,
		   K_MEM_CACHE_NONE);

	k_mutex_init(&rza2_pinctrl_data.lock);
	return 0;
}

SYS_INIT(pinctrl_rza2_driver_init, PRE_KERNEL_1, CONFIG_PINCTRL_RZA2_INIT_PRIORITY);
