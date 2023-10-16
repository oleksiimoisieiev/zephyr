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
#include <zephyr/dt-bindings/pinctrl/renesas/pinctrl-r7s9210.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>

#define RZA2_PDR(port)      (0x0000 + (port) * 2)           /* Direction 16-bit */
#define RZA2_PMR(port)      (0x0000 + (port))               /* Mode 8-bit */
#define RZA2_PFS(port, pin) (0x0180 + ((port) * 8) + (pin)) /* Fnct 8-bit */
#define RZA2_PCKIO          (0x0950)
#define RZA2_PPOC           (0x880)
#define RZA2_PSPIBSC        (0x8e0)

#define RZA2_PWPR 0x027f /* Write Protect 8-bit */

#define RZA2_PDR_INPUT  0x02
#define RZA2_PDR_OUTPUT 0x03
#define RZA2_PDR_MASK   0x03

#define PWPR_PFSWE BIT(6) /* PFSWE Bit Enable */
#define PWPR_B0WI  BIT(7) /* PFSWE Bit Disable */

#define RZA2_PDR_REG      DT_INST_REG_ADDR_BY_NAME(0, pdr)
#define RZA2_PDR_SIZE     DT_INST_REG_SIZE_BY_NAME(0, pdr)
#define RZA2_PINCTRL_REG  DT_INST_REG_ADDR_BY_NAME(0, pinctrl)
#define RZA2_PINCTRL_SIZE DT_INST_REG_SIZE_BY_NAME(0, pinctrl)

#define PPOC_POC0          (0x00000001u)
#define PPOC_POC0_SHIFT    (0u)
#define PPOC_POC2          (0x00000004u)
#define PPOC_POC2_SHIFT    (2u)
#define PPOC_POC3          (0x00000008u)
#define PPOC_POC3_SHIFT    (3u)
#define PPOC_POCSEL0       (0x00000100u)
#define PPOC_POCSEL0_SHIFT (8u)

#define PSPIBSC_QSPI0_SPCLK_DRV       (0x00000003u)
#define PSPIBSC_QSPI0_SPCLK_DRV_SHIFT (0u)
#define PSPIBSC_QSPI0_IO0_DRV         (0x0000000Cu)
#define PSPIBSC_QSPI0_IO0_DRV_SHIFT   (2u)
#define PSPIBSC_QSPI0_IO1_DRV         (0x00000030u)
#define PSPIBSC_QSPI0_IO1_DRV_SHIFT   (4u)
#define PSPIBSC_QSPI0_IO2_DRV         (0x000000C0u)
#define PSPIBSC_QSPI0_IO2_DRV_SHIFT   (6u)
#define PSPIBSC_QSPI0_IO3_DRV         (0x00000300u)
#define PSPIBSC_QSPI0_IO3_DRV_SHIFT   (8u)
#define PSPIBSC_QSPI0_SSL_DRV         (0x00000C00u)
#define PSPIBSC_QSPI0_SSL_DRV_SHIFT   (10u)
#define PSPIBSC_RPC_RESETN_DRV        (0x00003000u)
#define PSPIBSC_RPC_RESETN_DRV_SHIFT  (12u)
#define PSPIBSC_RPC_WPN_DRV           (0x0000C000u)
#define PSPIBSC_RPC_WPN_DRV_SHIFT     (14u)
#define PSPIBSC_QSPI1_SPCLK_DRV       (0x00030000u)
#define PSPIBSC_QSPI1_SPCLK_DRV_SHIFT (16u)
#define PSPIBSC_QSPI1_IO0_DRV         (0x000C0000u)
#define PSPIBSC_QSPI1_IO0_DRV_SHIFT   (18u)
#define PSPIBSC_QSPI1_IO1_DRV         (0x00300000u)
#define PSPIBSC_QSPI1_IO1_DRV_SHIFT   (20u)
#define PSPIBSC_QSPI1_IO2_DRV         (0x00C00000u)
#define PSPIBSC_QSPI1_IO2_DRV_SHIFT   (22u)
#define PSPIBSC_QSPI1_IO3_DRV         (0x03000000u)
#define PSPIBSC_QSPI1_IO3_DRV_SHIFT   (24u)
#define PSPIBSC_QSPI1_SSL_DRV         (0x0C000000u)
#define PSPIBSC_QSPI1_SSL_DRV_SHIFT   (26u)

static struct rza2_pinctrl_data {
	mm_reg_t pdr_addr;
	mm_reg_t pinctrl_addr;
	struct k_mutex lock;
} rza2_pinctrl_data;

static void rza2_pin_write_prot(uint8_t mask)
{
	/* PFS Register Write Protect set*/
	sys_write8(0x00, rza2_pinctrl_data.pinctrl_addr + RZA2_PWPR);
	sys_write8(mask, rza2_pinctrl_data.pinctrl_addr + RZA2_PWPR);
}

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
	rza2_pin_write_prot(PWPR_PFSWE); /* B0WI=0, PFSWE=1 */

	/* Set Pin function (interrupt disabled, ISEL=0) */
	sys_write8(func, rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));

	/* PFS Register Write Protect : ON */
	rza2_pin_write_prot(PWPR_B0WI); /* B0WI=1, PFSWE=0 */

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
	uint8_t reg8;

	k_mutex_lock(&rza2_pinctrl_data.lock, K_FOREVER);

	reg16 = sys_read16(rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));
	mask16 = RZA2_PDR_MASK << (pin * 2);
	reg16 &= ~mask16;
	reg16 |= dir << (pin * 2);

	sys_write16(reg16, rza2_pinctrl_data.pdr_addr + RZA2_PDR(port));

	reg8 = sys_read8(rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));
	reg8 &= ~BIT(pin);
	sys_write8(reg8, rza2_pinctrl_data.pinctrl_addr + RZA2_PMR(port));

	k_mutex_unlock(&rza2_pinctrl_data.lock);
}

static void rza2_set_gpio_int(uint8_t port, uint8_t pin, bool int_en)
{
	uint8_t value;
	/* PFS Register Write Protect : OFF */
	rza2_pin_write_prot(PWPR_PFSWE); /* B0WI=0, PFSWE=1 */

	if (int_en) {
		/* Enable interrupt, ISEL=1 */
		sys_write8(BIT(6), rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));
	} else {
		value = sys_read8(rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));
		value &= ~BIT(6);
		sys_write8(value, rza2_pinctrl_data.pinctrl_addr + RZA2_PFS(port, pin));
	}
	/* PFS Register Write Protect : ON */
	rza2_pin_write_prot(PWPR_B0WI); /* B0WI=1, PFSWE=0 */
}

static int rza2_set_ckio_drive(uint8_t drive_strength)
{
	if (drive_strength != 0b01 && drive_strength != 0b10) {
		return 0;
	}

	sys_write8(drive_strength, rza2_pinctrl_data.pinctrl_addr + RZA2_PCKIO);
	return 0;
}

static void rza2_set_reg(uint32_t reg_off, uint32_t value, uint32_t shift, uint32_t mask)
{
	uint32_t reg_value = sys_read32(rza2_pinctrl_data.pinctrl_addr + reg_off);

	reg_value = ((reg_value & (~mask)) | (value << shift));

	sys_write32(reg_value, rza2_pinctrl_data.pinctrl_addr + reg_off);
}

/*
 * Update PSPIBSC register according to the POCSEL0 and POC0 bits in PPOC register.
 * For Boot Mode 3 boot program will reconfigure the initial settings if needed.
 */
static void rza2_update_pspibsc(void)
{
	uint32_t ppoc;
	uint32_t reg_value;

	ppoc = sys_read32(rza2_pinctrl_data.pinctrl_addr + RZA2_PPOC);

	ppoc &= (PPOC_POCSEL0 | PPOC_POC0);

	/*
	 * According to the 51.3.34 section, all fields of PSCIBSC should be
	 * set according to the POC0 and POCSEL0 fields. So we can set all values
	 * at once.
	 */
	if (ppoc == (PPOC_POCSEL0 | PPOC_POC0)) {
		/* 3.3v */
		reg_value = 0x5555555;
	} else {
		/* 1.8v */
		reg_value = 0xFFFFFFF;
	}

	sys_write32(reg_value, rza2_pinctrl_data.pinctrl_addr + RZA2_PSPIBSC);
}

static int rza2_set_ppoc(const pinctrl_soc_pin_t pin)
{
	int ret = 0;

	k_mutex_lock(&rza2_pinctrl_data.lock, K_FOREVER);

	switch (pin.pin) {
	case PIN_POSEL:
		uint32_t value = ((pin.func & 0x1) << PPOC_POC0_SHIFT) |
				 (((pin.func & 0x2) >> 1) << PPOC_POCSEL0_SHIFT);

		rza2_set_reg(RZA2_PPOC, value, 0, PPOC_POC0 | PPOC_POCSEL0);
		rza2_update_pspibsc();
		break;
	default:
		ret = -EINVAL;
	}

	k_mutex_unlock(&rza2_pinctrl_data.lock);
	return ret;
}

static int pinctrl_configure_pin(const pinctrl_soc_pin_t pin)
{
	int ret = 0;

	if (pin.func & FUNC_GPIO_INPUT) {
		rza2_pin_to_gpio(pin.port, pin.pin, RZA2_PDR_INPUT);
	} else if (pin.func & FUNC_GPIO_OUTPUT) {
		rza2_pin_to_gpio(pin.port, pin.pin, RZA2_PDR_OUTPUT);
	} else if (pin.func & FUNC_GPIO_INT_EN) {
		rza2_set_gpio_int(pin.port, pin.pin, true);
	} else if (pin.func & FUNC_GPIO_INT_DIS) {
		rza2_set_gpio_int(pin.port, pin.pin, false);
	} else if (pin.port == PORTCKIO) {
		ret = rza2_set_ckio_drive(pin.drive_strength);
	} else if (pin.port == PORTPPOC) {
		ret = rza2_set_ppoc(pin);
	} else {
		rza2_set_pin_function(pin.port, pin.pin, pin.func);
	}
	return ret;
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
