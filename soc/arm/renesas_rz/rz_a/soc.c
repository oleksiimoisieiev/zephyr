/*
 * Copyright (c) 2023 EPAM Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm/aarch32/cortex_a_r/cmsis.h>
#include <zephyr/arch/arm/aarch32/cortex_a_r/lib_helpers.h>
#include "soc.h"

static int soc_rz_init(void)
{
	return 0;
}
SYS_INIT(soc_rz_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

extern char _vector_start[];

void relocate_vector_table(void)
{
#if defined(CONFIG_XIP) && (CONFIG_FLASH_BASE_ADDRESS != 0) || \
	!defined(CONFIG_XIP) && (CONFIG_SRAM_BASE_ADDRESS != 0)
	write_sctlr(read_sctlr() & ~HIVECS);
#elif defined(CONFIG_SW_VECTOR_RELAY) || defined(CONFIG_SW_VECTOR_RELAY_CLIENT)
	_vector_table_pointer = _vector_start;
#endif
	__set_VBAR(POINTER_TO_UINT(_vector_start));
}

/* Platform-specific early initialization */
void z_arm_platform_init(void) {}
