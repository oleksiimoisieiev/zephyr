/*
 * Copyright (c) 2022 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_RENESAS_RENESAS_CPG_MSSR_H_
#define ZEPHYR_DRIVERS_RENESAS_RENESAS_CPG_MSSR_H_

#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/device_mmio.h>

#define CPG_NUM_DOMAINS 2

struct cpg_clk_info_table {
	uint32_t domain;
	uint32_t module;
	mem_addr_t offset;
	uint32_t parent_id;

	int64_t in_freq;
	int64_t out_freq;

	/* TODO: add setting of this field and add function for getting status */
	enum clock_control_status status;

	struct cpg_clk_info_table *parent;
	struct cpg_clk_info_table *children_list;
	struct cpg_clk_info_table *next_sibling;
};

struct renesas_cpg_mssr_data {
	DEVICE_MMIO_RAM; /* Must be first */

	struct cpg_clk_info_table *clk_info_table[CPG_NUM_DOMAINS];
	const uint32_t clk_info_table_size[CPG_NUM_DOMAINS];

	struct k_spinlock lock;

	uint32_t (*get_div_helper)(mem_addr_t reg_addr, uint32_t module);
	uint32_t (*get_mul_helper)(mem_addr_t reg_addr, uint32_t module);
	int (*set_rate_helper)(const struct device *dev, struct cpg_clk_info_table *clk_info,
			       uint32_t div);
};

#define RENESAS_CPG_NONE -1
#define RENESAS_CPG_KHZ(khz) ((khz) * 1000U)
#define RENESAS_CPG_MHZ(mhz) (RENESAS_CPG_KHZ(mhz) * 1000U)

#define RENESAS_CORE_CLK_INFO_ITEM(id, off, par_id, in_frq)	\
	{							\
		.domain		= CPG_CORE,			\
		.module		= id,				\
		.offset		= off,				\
		.parent_id	= par_id,			\
		.in_freq	= in_frq,			\
		.out_freq	= RENESAS_CPG_NONE,		\
		.status		= CLOCK_CONTROL_STATUS_UNKNOWN,	\
		.parent		= NULL,				\
		.children_list	= NULL,				\
		.next_sibling	= NULL,				\
	}

#define RENESAS_MOD_CLK_INFO_ITEM(id, par_id)			\
	{							\
		.domain		= CPG_MOD,			\
		.module		= id,				\
		.offset		= RENESAS_CPG_NONE,		\
		.parent_id	= par_id,			\
		.in_freq	= RENESAS_CPG_NONE,		\
		.out_freq	= RENESAS_CPG_NONE,		\
		.status		= CLOCK_CONTROL_STATUS_UNKNOWN,	\
		.parent		= NULL,				\
		.children_list	= NULL,				\
		.next_sibling	= NULL,				\
	}

struct cpg_clk_info_table *renesas_cpg_find_clk_info_by_module_id(const struct device *dev,
								  uint32_t domain,
								  uint32_t id);

void renesas_cpg_build_clock_relationship(const struct device *dev);

void renesas_cpg_update_all_in_out_freq(const struct device *dev);

int renesas_cpg_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate);

int renesas_cpg_set_rate(const struct device *dev, clock_control_subsys_t sys,
			 clock_control_subsys_rate_t rate);

#endif /* ZEPHYR_DRIVERS_RENESAS_RENESAS_CPG_MSSR_H_ */
