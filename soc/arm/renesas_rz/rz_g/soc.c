/*
 * Copyright (c) 2024 EPAM Systems
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <cmsis_core.h>
#include <zephyr/arch/arm/cortex_m/nvic.h>
#include <zephyr/irq.h>
#include <zephyr/irq_nextlevel.h>
#include "soc.h"

static int soc_rz_init(void)
{
	return 0;
}

SYS_INIT(soc_rz_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

extern char _vector_start[];

void relocate_vector_table(void)
{
	__set_VBAR(POINTER_TO_UINT(_vector_start));
}

/* Platform-specific early initialization */
void z_arm_platform_init(void) {}
