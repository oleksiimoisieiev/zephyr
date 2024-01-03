/*
 * Copyright 2024 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/sys/util.h>
#include <zephyr/arch/arm/mmu/arm_mmu.h>
#include <zephyr/devicetree.h>

extern char _vector_start[];

static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("vector_tables",
			      POINTER_TO_UINT(_vector_start),
			      0x1000,
			      MT_STRONGLY_ORDERED | MPERM_R | MPERM_X),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
