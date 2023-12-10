/*
 * Copyright (c) 2023 EPAM Systemss
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r7s9210_intc

/**
 * @file
 * @brief r7s9210 interrupt controller driver
 *
 * This driver provides support for level 2 interrupts on the r7s9210
 * SoC. The r7s9210 interrupt controller is a front-end for the GIC found
 * on Renesas RZ/A2 SoCs: IRQ sense select for 8 external interrupts,
 * 1:1-mapped to 8 GIC SPI interrupts and NMI edge select.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq_nextlevel.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/dt-bindings/interrupt-controller/arm-gic.h>
#include <zephyr/dt-bindings/interrupt-controller/r7s9210-intc.h>

/**
 * CR0 is a 16-bit register that sets the input signal detection mode for the
 * external interrupt input pin NMI, and indicates the input level at the NMI pin.
 */
#define R7S9210_INTC_CR0 0x0
#define R7S9210_INTC_CR0_NMIF        BIT(1)   /* NMI Input Level */
#define R7S9210_INTC_CR0_NMIE        BIT(8)   /* NMI Edge Select */
#define R7S9210_INTC_CR0_NMIE_OFFSET 8
#define R7S9210_INTC_CR0_NMIL        BIT(15)  /* NMI Interrupt Request */

/**
 * CR1 is a 16-bit register that specifies the detection mode for external interrupt
 * input pins IRQ7 to IRQ0 individually: low level, falling edge, rising edge, or
 * both edges
 */
#define R7S9210_INTC_CR1 0x2

/**
 * Possible values for type should be taken from r7s9210-intc.h
 */
#define R7S9210_SET_DETECTION_MODE(reg, irq, type) \
	reg = (reg & ~(0x3U << (irq * 2U))) | (type << (irq * 2U))

/**
 * RR is a 16-bit register that indicates interrupt requests from external input
 * pins IRQ7 to IRQ0
 */
#define R7S9210_INTC_RR 0x4

/**
 * MSK is a bit to set release, when the IRQ signal is used as a software standby
 * cancel source signal. 0 - masked, 1 - unmasked. When MSK is 1 (unmasked), the
 * IRQ signal can be used as a software standby cancel source.
 */
#define R7S9210_INTC_RR_MSK BIT(15)

struct r7s9210_intc_line_cfg {
	unsigned int line;
	unsigned int parent_line;
};

struct r7s9210_intc_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
	struct r7s9210_intc_line_cfg *line_map;
	unsigned int num_lines;
};

struct r7s9210_intc_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct k_spinlock lock;
};

static void r7s9210_isr(const void *arg)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	struct r7s9210_intc_data *data = (struct r7s9210_intc_data *)dev->data;
	k_spinlock_key_t key;
	const uint32_t line = POINTER_TO_UINT(arg);
	const struct _isr_table_entry *entry = &_sw_isr_table[CONFIG_2ND_LVL_ISR_TBL_OFFSET + line];
	uint16_t rr;

	entry->isr(entry->arg);

	key = k_spin_lock(&data->lock);
	/*
	 * For non-level detection, we should execute a clear sequence for the IRQ line
	 * but let's save time in the ISR and perform the clear sequence for low-level mode too.
	 */
	rr = sys_read16(DEVICE_MMIO_GET(dev) + R7S9210_INTC_RR);
	rr &= ~BIT(line);
	sys_write16(rr, DEVICE_MMIO_GET(dev) + R7S9210_INTC_RR);
	k_spin_unlock(&data->lock, key);
}

/* is this irq belongs to this intc if yes - return parent irq */
static unsigned int r7s9210_intr_get_parent_irq(const struct device *dev, unsigned int irq)
{
	const struct r7s9210_intc_cfg *cfg = (const struct r7s9210_intc_cfg *)dev->config;
	unsigned int parent_line = irq_parent_level_2(irq) + 32;

	irq = irq_from_level_2(irq);
	parent_line += irq;

	for (unsigned int line = 0; line < cfg->num_lines; line++) {
		if (cfg->line_map[line].line != irq) {
			continue;
		}

		if (cfg->line_map[line].parent_line == parent_line) {
			return parent_line;
		}
	}

	return 0;
}

/* The driver can't enable IRQ by this driver, so lets request it from the parent driver. */
static void r7s9210_intr_enable(const struct device *dev, unsigned int irq)
{
	unsigned int parent_irq = r7s9210_intr_get_parent_irq(dev, irq);

	if (!parent_irq) {
		return;
	}

	irq_enable(parent_irq);
}

/* The driver can't disable IRQ by this driver, so lets request it from the parent driver. */
static void r7s9210_intr_disable(const struct device *dev, unsigned int irq)
{
	unsigned int parent_irq = r7s9210_intr_get_parent_irq(dev, irq);

	if (!parent_irq) {
		return;
	}

	irq_disable(parent_irq);
}

/**
 * The driver can't determine the status of whether the IRQs on lines are enabled or not.
 * So, let's request information from the parent driver.
 */
static unsigned int r7s9210_intr_get_state(const struct device *dev)
{
	const struct r7s9210_intc_cfg *cfg = (const struct r7s9210_intc_cfg *)dev->config;

	for (unsigned int line = 0; line < cfg->num_lines; line++) {
		if (irq_is_enabled(cfg->line_map[line].parent_line)) {
			return 1;
		}
	}

	return 0;
}

static void r7s9210_intr_set_priority(const struct device *dev, unsigned int irq,
				      unsigned int prio, uint32_t flags)
{
	uint16_t cr1;
	struct r7s9210_intc_data *data = (struct r7s9210_intc_data *)dev->data;
	k_spinlock_key_t key;
	unsigned int parent_irq = r7s9210_intr_get_parent_irq(dev, irq);

	if (!parent_irq) {
		return;
	}

	irq = irq_from_level_2(irq);

	switch (flags) {
	case IRQ_TYPE_LOW_LEVEL:
		flags = 0;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		flags = 1;
		break;
	case IRQ_TYPE_EDGE_RISING:
		flags = 2;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		flags = 3;
		break;
	default:
		break;
	}

	key = k_spin_lock(&data->lock);
	cr1 = sys_read16(DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR1);
	R7S9210_SET_DETECTION_MODE(cr1, irq, flags);
	sys_write16(cr1, DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR1);
	k_spin_unlock(&data->lock, key);
}

/**
 * The driver can't determine the status of whether the IRQ on this line is enabled or not.
 * Therefore, let's request this information from the parent driver.
 */
static int r7s9210_intr_get_line_state(const struct device *dev, unsigned int irq)
{
	unsigned int parent_irq = r7s9210_intr_get_parent_irq(dev, irq);

	if (!parent_irq) {
		return 0;
	}

	return irq_is_enabled(parent_irq);
}

static const struct irq_next_level_api r7s9210_intc_next_lvl = {
	.intr_enable = r7s9210_intr_enable,
	.intr_disable = r7s9210_intr_disable,
	.intr_get_state = r7s9210_intr_get_state,
	.intr_set_priority = r7s9210_intr_set_priority,
	.intr_get_line_state = r7s9210_intr_get_line_state,
};

BUILD_ASSERT(DT_NUM_IRQS(DT_DRV_INST(0)) == DT_INST_PROP_LEN(0, map),
	     "Number of items in interrupts and map should be the same");

#define IRQ_CONFIGURE(n, inst)                                             \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq),                      \
		    DT_INST_IRQ_BY_IDX(inst, n, priority), r7s9210_isr,    \
		    DT_INST_PROP_BY_IDX(inst, map, n),                     \
		    DT_INST_IRQ_BY_IDX(inst, n, flags));                   \
	BUILD_ASSERT(DT_INST_IRQ_BY_IDX(inst, n, type) == GIC_SPI,         \
				 "Driver can work only with SPI parent interrupts");

#define CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, IRQ_CONFIGURE, (), inst)

#define FILL_ONE_LINE_MAP(n, inst)                        \
	{                                                 \
		DT_INST_PROP_BY_IDX(inst, map, n),        \
		DT_INST_IRQ_BY_IDX(inst, n, irq),         \
	}
#define FILL_LINE_MAP_CONFIG(inst, n) \
	(struct r7s9210_intc_line_cfg []){LISTIFY(n, FILL_ONE_LINE_MAP, (,), inst)}

static inline void r7s9210_intc_setup_nmi_edge(const struct device *dev)
{
	k_spinlock_key_t key;
	uint16_t cr0;
	struct r7s9210_intc_data *data = (struct r7s9210_intc_data *)dev->data;
	uint16_t nmi_edge = DT_INST_PROP_OR(0, nmi_edge, IRQ_TYPE_EDGE_FALLING) >> 2;

	nmi_edge <<= R7S9210_INTC_CR0_NMIE_OFFSET;

	key = k_spin_lock(&data->lock);
	cr0 = sys_read16(DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR0);
	cr0 &= R7S9210_INTC_CR0_NMIE;
	cr0 |= nmi_edge;
	sys_write16(cr0, DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR0);
	k_spin_unlock(&data->lock, key);
}

/*
 * note: This function only makes sense when 'z_arm_nmi_set_handler' is called since the default
 *       NMI handler never exits. Moreover, NMI handled incorrectly for Cortex A/R platforms:
 *         * LR isn't saved before entering NMI handler (look at 'z_arm_nmi'), so dabort happened;
 *         * interrupt nesting counter is never incremented for NMI, but decremented on exit.
 */
void z_arm_nmi_eoi(void)
{
	const struct device *const dev = DEVICE_DT_INST_GET(0);
	uint16_t cr0;

	/* cancel current NMI, this function is called after nmi handler */
	cr0 = sys_read16(DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR0);
	cr0 &= R7S9210_INTC_CR0_NMIF;
	sys_write16(cr0, DEVICE_MMIO_GET(dev) + R7S9210_INTC_CR0);
}

static int r7s9210_intc_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	CONFIGURE_ALL_IRQS(0, DT_NUM_IRQS(DT_DRV_INST(0)));

	r7s9210_intc_setup_nmi_edge(dev);
	return 0;
}

static const struct r7s9210_intc_cfg drv_cfg = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),

	.num_lines = DT_NUM_IRQS(DT_DRV_INST(0)),
	.line_map = FILL_LINE_MAP_CONFIG(0, DT_NUM_IRQS(DT_DRV_INST(0))),
};

static struct r7s9210_intc_cfg drv_data;

DEVICE_DT_INST_DEFINE(0, &r7s9210_intc_init, NULL,
		      &drv_data, &drv_cfg, PRE_KERNEL_1,
		      CONFIG_INTC_R7S9210_INIT_PRIORITY, &r7s9210_intc_next_lvl);