/*
 * Copyright (c) 2021 IoT.bzh
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <string.h>
#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#endif

LOG_MODULE_REGISTER(uart_scif, CONFIG_UART_LOG_LEVEL);
/* TODO: Add error handling */
/* TODO: Add multithread locking */

struct scif_reg {
	uint8_t offset, size;
};

/*
 * SCI register subset common for all port types.
 * Not all registers will exist on all parts.
 */
enum {
	SCSMR,				/* Serial Mode Register */
	SCBRR,				/* Bit Rate Register */
	SCSCR,				/* Serial Control Register */
	SCFSR,				/* Serial Status Register */
	SCFCR,				/* FIFO Control Register */
	SCFDR,				/* FIFO Data Count Register */
	SCFTDR,				/* Transmit (FIFO) Data Register */
	SCFRDR,				/* Receive (FIFO) Data Register */
	SCLSR,				/* Line Status Register */
	SCTFDR,				/* Transmit FIFO Data Count Register */
	SCRFDR,				/* Receive FIFO Data Count Register */
	SCSPTR,				/* Serial Port Register */
	SCDL,				/* BRG Frequency Division Register */
	SEMR,				/* Serial extended mode register */
	FTCR,				/* FIFO Trigger Control Register */

	SCIF_NR_REGS,
};

enum {
	SCIF_SCIFA_TYPE,
	SCIF_RZ_SCIFA_TYPE,

	NR_REGTYPES,
};

struct scif_params {
	const struct scif_reg regs[SCIF_NR_REGS];
	uint16_t init_lsr_mask;
	uint16_t init_interrupt_mask;
};

struct uart_scif_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct device *clock_dev;
	struct renesas_cpg_clk mod_clk;
	struct renesas_cpg_clk bus_clk;
	const struct pinctrl_dev_config *pcfg;
	const struct scif_params *params;
	uint32_t type;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
};

#ifdef CONFIG_UART_ASYNC_API
struct dma_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_conf;
	uint8_t dma_slot;
	struct dma_config dma_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	int fifo_threshold;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	size_t offset;
	volatile int32_t counter;
	int32_t timeout;
	struct k_work_delayable timeout_work;
	bool enabled;
	const char *name;
};

#define RX_SRC_INCREMENT	0
#define RX_DST_INCREMENT	1
#define TX_SRC_INCREMENT	1
#define TX_DST_INCREMENT	0

static int uart_scif_async_rx_disable(const struct device *dev);
static int uart_scif_async_tx_abort(const struct device *dev);
static void uart_scif_dma_rx_flush(const struct device *dev);
static inline void scif_async_timer_start(struct k_work_delayable *work, int32_t timeout);
#endif

struct uart_scif_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct uart_config current_config;
	uint32_t clk_rate;
	struct k_spinlock lock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t irq_cb;
	void *irq_cb_data;
#endif
#ifdef CONFIG_UART_ASYNC_API
	const struct device *uart_dev;
	uart_callback_t async_cb;
	void *async_cb_data;
	struct dma_stream dma_tx;
	struct dma_stream dma_rx;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
	bool async_inited;
#endif
};

/* SCSMR (Serial Mode Register) */
#define SCSMR_C_A       BIT(7)  /* Communication Mode */
#define SCSMR_CHR       BIT(6)  /* 7-bit Character Length */
#define SCSMR_PE        BIT(5)  /* Parity Enable */
#define SCSMR_O_E       BIT(4)  /* Odd Parity */
#define SCSMR_STOP      BIT(3)  /* Stop Bit Length */
#define SCSMR_CKS1      BIT(1)  /* Clock Select 1 */
#define SCSMR_CKS0      BIT(0)  /* Clock Select 0 */

/* SCSCR (Serial Control Register) */
#define SCSCR_TEIE_SCIFA  BIT(11) /* Transmit End Interrupt Enable on SCIFA*/
#define SCSCR_TEIE_RZ   BIT(2) /* Transmit End Interrupt Enable RZ */
#define SCSCR_TIE       BIT(7)  /* Transmit Interrupt Enable */
#define SCSCR_RIE       BIT(6)  /* Receive Interrupt Enable */
#define SCSCR_TE        BIT(5)  /* Transmit Enable */
#define SCSCR_RE        BIT(4)  /* Receive Enable */
#define SCSCR_REIE      BIT(3)  /* Receive Error Interrupt Enable */
#define SCSCR_TOIE_SCIFA  BIT(2)  /* Timeout Interrupt Enable on SCIFA */
#define SCSCR_CKE1      BIT(1)  /* Clock Enable 1 */
#define SCSCR_CKE0      BIT(0)  /* Clock Enable 0 */

/* SCFCR (FIFO Control Register) */
#define SCFCR_RTRG1     BIT(7)  /* Receive FIFO Data Count Trigger 1 */
#define SCFCR_RTRG0     BIT(6)  /* Receive FIFO Data Count Trigger 0 */
#define SCFCR_TTRG1     BIT(5)  /* Transmit FIFO Data Count Trigger 1 */
#define SCFCR_TTRG0     BIT(4)  /* Transmit FIFO Data Count Trigger 0 */
#define SCFCR_MCE       BIT(3)  /* Modem Control Enable */
#define SCFCR_TFRST     BIT(2)  /* Transmit FIFO Data Register Reset */
#define SCFCR_RFRST     BIT(1)  /* Receive FIFO Data Register Reset */
#define SCFCR_LOOP      BIT(0)  /* Loopback Test */

/* SCFSR (Serial Status Register) */
#define SCFSR_PER3      BIT(15) /* Parity Error Count 3 */
#define SCFSR_PER2      BIT(14) /* Parity Error Count 2 */
#define SCFSR_PER1      BIT(13) /* Parity Error Count 1 */
#define SCFSR_PER0      BIT(12) /* Parity Error Count 0 */
#define SCFSR_FER3      BIT(11) /* Framing Error Count 3 */
#define SCFSR_FER2      BIT(10) /* Framing Error Count 2 */
#define SCFSR_FER1      BIT(9)  /* Framing Error Count 1 */
#define SCFSR_FER0      BIT(8)  /* Framing Error Count 0 */
#define SCFSR_ER        BIT(7)  /* Receive Error */
#define SCFSR_TEND      BIT(6)  /* Transmission ended */
#define SCFSR_TDFE      BIT(5)  /* Transmit FIFO Data Empty */
#define SCFSR_BRK       BIT(4)  /* Break Detect */
#define SCFSR_FER       BIT(3)  /* Framing Error */
#define SCFSR_PER       BIT(2)  /* Parity Error */
#define SCFSR_RDF       BIT(1)  /* Receive FIFO Data Full */
#define SCFSR_DR        BIT(0)  /* Receive Data Ready */

/* SCLSR (Line Status Register) on (H)SCIF */
#define SCLSR_TO_SCIFA  BIT(2)  /* Timeout on SCIFA */
#define SCLSR_ORER      BIT(0)  /* Overrun Error */

/* Serial Extended Mode Register */
#define SEMR_ABCS0      BIT(0)  /* Asynchronous Base Clock Select */
#define SEMR_NFEN       BIT(2)  /* Noise Cancellation Enable */
#define SEMR_DIR        BIT(3)  /* Data Transfer Direction Select */
#define SEMR_MDDRS      BIT(4)  /* Modulation Duty Register Selec */
#define SEMR_BRME       BIT(5)  /* Bit Rate Modulation Enable */
/* Baud Rate Generator Double-Speed Mode Select */
#define SEMR_BGDM       BIT(7)

/* Registers */
static const struct scif_params port_params[NR_REGTYPES] = {
	[SCIF_SCIFA_TYPE] = {
		.regs = {
			[SCSMR]		= { 0x00, 16 },
			[SCBRR]		= { 0x04,  8 },
			[SCSCR]		= { 0x08, 16 },
			[SCFTDR]	= { 0x0c,  8 },
			[SCFSR]		= { 0x10, 16 },
			[SCFRDR]	= { 0x14,  8 },
			[SCFCR]		= { 0x18, 16 },
			[SCFDR]		= { 0x1c, 16 },
			[SCSPTR]	= { 0x20, 16 },
			[SCLSR]		= { 0x24, 16 },
		},
		.init_lsr_mask = SCLSR_ORER | SCLSR_TO_SCIFA,
		.init_interrupt_mask = SCSCR_TIE | SCSCR_RIE | SCSCR_REIE |
				       SCSCR_TEIE_SCIFA | SCSCR_TOIE_SCIFA,
	},
	[SCIF_RZ_SCIFA_TYPE] = {
		.regs = {
			[SCSMR]		= { 0x00, 16 },
			[SCBRR]		= { 0x02,  8 },
			[SCSCR]		= { 0x04, 16 },
			[SCFTDR]	= { 0x06,  8 },
			[SCFSR]		= { 0x08, 16 },
			[SCFRDR]	= { 0x0A,  8 },
			[SCFCR]		= { 0x0C, 16 },
			[SCFDR]		= { 0x0E, 16 },
			[SCSPTR]	= { 0x10, 16 },
			[SCLSR]		= { 0x12, 16 },
			[SEMR]		= { 0x14, 8 },
			[FTCR]		= { 0x16, 16 },
		},
		.init_lsr_mask = SCLSR_ORER,
		.init_interrupt_mask = SCSCR_TIE | SCSCR_RIE | SCSCR_REIE | SCSCR_TEIE_RZ,
	},
};

#define scif_getreg(dev, offset)	\
	&(((const struct uart_scif_cfg *)((dev)->config))->params->regs[offset])

/* TODO: Make unify uart_scif_read/uart_scif_write functions */

static uint8_t uart_scif_read_8(const struct device *dev,
				uint32_t offs)
{
	const struct uart_scif_cfg *config = dev->config;
	uint32_t offset = config->params->regs[offs].offset;

	return sys_read8(DEVICE_MMIO_GET(dev) + offset);
}

static void uart_scif_write_8(const struct device *dev,
			      uint32_t offs, uint8_t value)
{
	const struct uart_scif_cfg *config = dev->config;
	uint32_t offset = config->params->regs[offs].offset;

	sys_write8(value, DEVICE_MMIO_GET(dev) + offset);
}

static uint16_t uart_scif_read_16(const struct device *dev,
				  uint32_t offs)
{
	const struct uart_scif_cfg *config = dev->config;
	uint32_t offset = config->params->regs[offs].offset;

	return sys_read16(DEVICE_MMIO_GET(dev) + offset);
}

static void uart_scif_write_16(const struct device *dev,
			       uint32_t offs, uint16_t value)
{
	const struct uart_scif_cfg *config = dev->config;
	uint32_t offset = config->params->regs[offs].offset;

	sys_write16(value, DEVICE_MMIO_GET(dev) + offset);
}

static void uart_scif_set_baudrate(const struct device *dev,
				   uint32_t baud_rate)
{
	const struct uart_scif_cfg *config = dev->config;
	struct uart_scif_data *data = dev->data;
	uint8_t reg_val;

	/* Those formulas for hardcoded parameters */
	if (config->type == SCIF_RZ_SCIFA_TYPE) {
		reg_val = ((data->clk_rate + 4 * baud_rate) / (8 * baud_rate) - 1);
	} else {
		reg_val = ((data->clk_rate + 16 * baud_rate) / (32 * baud_rate) - 1);
	}
	uart_scif_write_8(dev, SCBRR, reg_val);
}

static int uart_scif_poll_in(const struct device *dev, unsigned char *p_char)
{
	struct uart_scif_data *data = dev->data;
	uint16_t reg_val;
	int ret = 0;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Receive FIFO empty */
	if (!((uart_scif_read_16(dev, SCFSR)) & SCFSR_RDF)) {
		ret = -1;
		goto unlock;
	}

	*p_char = uart_scif_read_8(dev, SCFRDR);

	reg_val = uart_scif_read_16(dev, SCFSR);
	reg_val &= ~SCFSR_RDF;
	uart_scif_write_16(dev, SCFSR, reg_val);

unlock:
	k_spin_unlock(&data->lock, key);

	return ret;
}

static void uart_scif_poll_out(const struct device *dev, unsigned char out_char)
{
	struct uart_scif_data *data = dev->data;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Wait for empty space in transmit FIFO */
	while (!(uart_scif_read_16(dev, SCFSR) & SCFSR_TDFE)) {
	}

	uart_scif_write_8(dev, SCFTDR, out_char);

	reg_val = uart_scif_read_16(dev, SCFSR);
	reg_val &= ~(SCFSR_TDFE | SCFSR_TEND);
	uart_scif_write_16(dev, SCFSR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_scif_configure(const struct device *dev,
			       const struct uart_config *cfg)
{
	const struct uart_scif_cfg *config = dev->config;
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	uint8_t reg_val8;
	k_spinlock_key_t key;

	if (cfg->parity != UART_CFG_PARITY_NONE ||
	    cfg->stop_bits != UART_CFG_STOP_BITS_1 ||
	    cfg->data_bits != UART_CFG_DATA_BITS_8 ||
	    cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

#ifdef CONFIG_UART_ASYNC_API
	uart_scif_async_rx_disable(dev);
	uart_scif_async_tx_abort(dev);
#endif

	/* Disable Transmit and Receive */
	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_TE | SCSCR_RE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	/* Emptying Transmit and Receive FIFO */
	reg_val = uart_scif_read_16(dev, SCFCR);
	reg_val |= (SCFCR_TFRST | SCFCR_RFRST);
	uart_scif_write_16(dev, SCFCR, reg_val);

	/* Resetting Errors Registers */
	reg_val = uart_scif_read_16(dev, SCFSR);
	reg_val &= ~(SCFSR_ER | SCFSR_DR | SCFSR_BRK | SCFSR_RDF);
	uart_scif_write_16(dev, SCFSR, reg_val);

	reg_val = uart_scif_read_16(dev, SCLSR);
	reg_val &= ~(config->params->init_lsr_mask);
	uart_scif_write_16(dev, SCLSR, reg_val);

	/* Select internal clock */
	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_CKE1 | SCSCR_CKE0);
	uart_scif_write_16(dev, SCSCR, reg_val);

	/* Serial Configuration (8N1) & Clock divider selection */
	reg_val = uart_scif_read_16(dev, SCSMR);
	reg_val &= ~(SCSMR_C_A | SCSMR_CHR | SCSMR_PE | SCSMR_O_E | SCSMR_STOP |
		     SCSMR_CKS1 | SCSMR_CKS0);
	uart_scif_write_16(dev, SCSMR, reg_val);

	/* Set baudrate */
	uart_scif_set_baudrate(dev, cfg->baudrate);

	/* FIFOs data count trigger configuration */
	reg_val = uart_scif_read_16(dev, SCFCR);
	reg_val &= ~(SCFCR_RTRG1 | SCFCR_RTRG0 | SCFCR_TTRG1 | SCFCR_TTRG0 |
		     SCFCR_MCE | SCFCR_TFRST | SCFCR_RFRST);
	uart_scif_write_16(dev, SCFCR, reg_val);

	/* Enable Transmit & Receive + disable Interrupts */
	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val |= (SCSCR_TE | SCSCR_RE);
	reg_val &= ~(config->params->init_interrupt_mask);
	uart_scif_write_16(dev, SCSCR, reg_val);

	if (config->type == SCIF_RZ_SCIFA_TYPE) {
		reg_val8 = uart_scif_read_8(dev, SEMR);
		reg_val8 |= (SEMR_ABCS0 | SEMR_BGDM);
		uart_scif_write_8(dev, SEMR, reg_val8);
	}

	data->current_config = *cfg;

	k_spin_unlock(&data->lock, key);

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_scif_config_get(const struct device *dev,
				struct uart_config *cfg)
{
	struct uart_scif_data *data = dev->data;

	*cfg = data->current_config;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_ASYNC_API
static void uart_scif_async_rx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct dma_stream *rx_stream = CONTAINER_OF(dwork, struct dma_stream, timeout_work);
	struct uart_scif_data *data = CONTAINER_OF(rx_stream, struct uart_scif_data, dma_rx);
	const struct device *dev = data->uart_dev;
	struct dma_status stat;

	if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
		size_t rx_rcv_len = data->dma_rx.buffer_length - stat.pending_length;

		if (rx_rcv_len == 0) {
			k_work_reschedule(dwork, K_USEC(rx_stream->timeout));
			return;
		}
	}

	if (data->dma_rx.counter == data->dma_rx.buffer_length) {
		uart_scif_async_rx_disable(dev);
	} else {
		uart_scif_dma_rx_flush(dev);
	}
}

static void uart_scif_async_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct dma_stream *tx_stream = CONTAINER_OF(dwork, struct dma_stream, timeout_work);
	struct uart_scif_data *data = CONTAINER_OF(tx_stream, struct uart_scif_data, dma_tx);
	const struct device *dev = data->uart_dev;

	uart_scif_async_tx_abort(dev);
}

static void uart_scif_async_init(const struct device *dev)
{
	const struct uart_scif_cfg *config = dev->config;
	struct uart_scif_data *data = dev->data;
	uint32_t offset;

	if (!device_is_ready(data->dma_rx.dma_dev)) {
		return;
	}

	if (!device_is_ready(data->dma_tx.dma_dev)) {
		return;
	}

	data->uart_dev = dev;

	/* Disable both TX and RX DMA requests */
	data->dma_rx.enabled = false;

	k_work_init_delayable(&data->dma_rx.timeout_work, uart_scif_async_rx_timeout);
	k_work_init_delayable(&data->dma_tx.timeout_work, uart_scif_async_tx_timeout);

	/* Configure dma rx config */
	offset = config->params->regs[SCFRDR].offset;
	memset(&data->dma_rx.blk_cfg, 0, sizeof(data->dma_rx.blk_cfg));
	data->dma_rx.blk_cfg.source_address =
				(uint32_t)DEVICE_MMIO_ROM_PTR(dev)->phys_addr + offset;
	data->dma_rx.blk_cfg.dest_address = 0; /* dest not ready */

	if (data->dma_rx.src_addr_increment) {
		data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	if (data->dma_rx.dst_addr_increment) {
		data->dma_rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	/* RX disable circular buffer */
	data->dma_rx.blk_cfg.source_reload_en = 0;
	data->dma_rx.blk_cfg.dest_reload_en = 0;
	data->dma_rx.blk_cfg.fifo_mode_control = data->dma_rx.fifo_threshold;

	data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
	data->dma_rx.dma_cfg.user_data = (void *)dev;
	data->dma_rx.dma_cfg.dma_slot = data->dma_rx.dma_slot;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	/* Configure dma tx config */
	offset = config->params->regs[SCFTDR].offset;
	memset(&data->dma_tx.blk_cfg, 0, sizeof(data->dma_tx.blk_cfg));

	data->dma_tx.blk_cfg.dest_address = (uint32_t)DEVICE_MMIO_ROM_PTR(dev)->phys_addr + offset;

	data->dma_tx.blk_cfg.source_address = 0; /* not ready */

	if (data->dma_tx.src_addr_increment) {
		data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	if (data->dma_tx.dst_addr_increment) {
		data->dma_tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	data->dma_tx.blk_cfg.fifo_mode_control = data->dma_tx.fifo_threshold;

	data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
	data->dma_tx.dma_cfg.user_data = (void *)dev;
	data->dma_tx.dma_cfg.dma_slot = data->dma_tx.dma_slot;
	data->async_inited = true;
}
#endif

static int uart_scif_init(const struct device *dev)
{
	const struct uart_scif_cfg *config = dev->config;
	struct uart_scif_data *data = dev->data;
	int ret;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev,
			       (clock_control_subsys_t)&config->mod_clk);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev,
				     (clock_control_subsys_t)&config->bus_clk,
				     &data->clk_rate);
	if (ret < 0) {
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	ret = uart_scif_configure(dev, &data->current_config);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
	uart_scif_async_init(dev);
#endif

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static bool uart_scif_irq_is_enabled(const struct device *dev,
				     uint32_t irq)
{
	return !!(uart_scif_read_16(dev, SCSCR) & irq);
}

static int uart_scif_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data,
			       int len)
{
	struct uart_scif_data *data = dev->data;
	int num_tx = 0;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while (((len - num_tx) > 0) &&
	       (uart_scif_read_16(dev, SCFSR) & SCFSR_TDFE)) {
		/* Send current byte */
		uart_scif_write_8(dev, SCFTDR, tx_data[num_tx]);

		reg_val = uart_scif_read_16(dev, SCFSR);
		reg_val &= ~(SCFSR_TDFE | SCFSR_TEND);
		uart_scif_write_16(dev, SCFSR, reg_val);

		num_tx++;
	}

	k_spin_unlock(&data->lock, key);

	return num_tx;
}

static int uart_scif_fifo_read(const struct device *dev, uint8_t *rx_data,
			       const int size)
{
	struct uart_scif_data *data = dev->data;
	int num_rx = 0;
	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	while (((size - num_rx) > 0) &&
	       (uart_scif_read_16(dev, SCFSR) & SCFSR_RDF)) {
		/* Receive current byte */
		rx_data[num_rx++] = uart_scif_read_8(dev, SCFRDR);

		reg_val = uart_scif_read_16(dev, SCFSR);
		reg_val &= ~(SCFSR_RDF);
		uart_scif_write_16(dev, SCFSR, reg_val);

	}

	k_spin_unlock(&data->lock, key);

	return num_rx;
}

static void uart_scif_irq_tx_enable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val |= (SCSCR_TIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_scif_irq_tx_disable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_TIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_scif_irq_tx_ready(const struct device *dev)
{
	return !!(uart_scif_read_16(dev, SCFSR) & SCFSR_TDFE);
}

static void uart_scif_irq_rx_enable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val |= (SCSCR_RIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_scif_irq_rx_disable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_RIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_scif_irq_rx_ready(const struct device *dev)
{
	return !!(uart_scif_read_16(dev, SCFSR) & SCFSR_RDF);
}

static void uart_scif_irq_err_enable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val |= (SCSCR_REIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static void uart_scif_irq_err_disable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	uint16_t reg_val;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	reg_val = uart_scif_read_16(dev, SCSCR);
	reg_val &= ~(SCSCR_REIE);
	uart_scif_write_16(dev, SCSCR, reg_val);

	k_spin_unlock(&data->lock, key);
}

static int uart_scif_irq_is_pending(const struct device *dev)
{
	return (uart_scif_irq_rx_ready(dev) && uart_scif_irq_is_enabled(dev, SCSCR_RIE)) ||
	       (uart_scif_irq_tx_ready(dev) && uart_scif_irq_is_enabled(dev, SCSCR_TIE));
}

static int uart_scif_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_scif_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_scif_data *data = dev->data;

	data->irq_cb = cb;
	data->irq_cb_data = cb_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	data->async_cb = NULL;
	data->async_cb_data = NULL;
#endif
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the irq callback function, if one exists.
 *
 */
void uart_scif_isr(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	if (data->irq_cb) {
		data->irq_cb(dev, data->irq_cb_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_ASYNC_API
static inline void scif_async_timer_start(struct k_work_delayable *work, int32_t timeout)
{
	struct dma_stream *stream = CONTAINER_OF(work, struct dma_stream, timeout_work);

	if (timeout != SYS_FOREVER_US && timeout != 0) {
		/* start timer */
		LOG_DBG("async timer started for %d usi on %s", timeout, stream->name);
		k_work_reschedule(work, K_USEC(timeout));
	}
}

static inline void scif_async_user_callback(struct uart_scif_data *data, struct uart_event *event)
{
	if (data->async_cb) {
		data->async_cb(data->uart_dev, event, data->async_cb_data);
	}
}

static int uart_scif_async_callback_set(const struct device *dev,
					uart_callback_t callback,
					void *cb_data)
{
	struct uart_scif_data *data = dev->data;

	data->async_cb = callback;
	data->async_cb_data = cb_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	data->irq_cb = NULL;
	data->irq_cb_data = NULL;
#endif

	return 0;
}

static inline void scif_async_evt_rx_rdy(struct uart_scif_data *data)
{
	LOG_DBG("rx_rdy: (%d %d)", data->dma_rx.offset, data->dma_rx.counter);

	struct uart_event event = {
		.type = UART_RX_RDY,
		.data.rx.buf = data->dma_rx.buffer,
		.data.rx.len = data->dma_rx.counter - data->dma_rx.offset,
		.data.rx.offset = data->dma_rx.offset
	};

	/* update the current pos for new data */
	data->dma_rx.offset = data->dma_rx.counter;

	/* send event only for new data */
	if (event.data.rx.len > 0) {
		scif_async_user_callback(data, &event);
	}
}

static inline void scif_async_evt_rx_err(struct uart_scif_data *data, int err_code)
{
	LOG_DBG("rx error: %d", err_code);

	struct uart_event event = {
		.type = UART_RX_STOPPED,
		.data.rx_stop.reason = err_code,
		.data.rx_stop.data.len = data->dma_rx.counter,
		.data.rx_stop.data.offset = 0,
		.data.rx_stop.data.buf = data->dma_rx.buffer
	};

	scif_async_user_callback(data, &event);
}

static inline void scif_async_evt_tx_done(struct uart_scif_data *data)
{
	LOG_DBG("tx done: %d", data->dma_tx.counter);

	struct uart_event event = {
		.type = UART_TX_DONE,
		.data.tx.buf = data->dma_tx.buffer,
		.data.tx.len = data->dma_tx.counter
	};

	/* Reset tx buffer */
	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

	scif_async_user_callback(data, &event);
}

static inline void scif_async_evt_tx_abort(struct uart_scif_data *data)
{
	LOG_DBG("tx abort: %d", data->dma_tx.counter);

	struct uart_event event = {
		.type = UART_TX_ABORTED,
		.data.tx.buf = data->dma_tx.buffer,
		.data.tx.len = data->dma_tx.counter
	};

	/* Reset tx buffer */
	data->dma_tx.buffer_length = 0;
	data->dma_tx.counter = 0;

	scif_async_user_callback(data, &event);
}

static inline void scif_async_evt_rx_buf_request(struct uart_scif_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	scif_async_user_callback(data, &evt);
}

static inline void scif_async_evt_rx_buf_release(struct uart_scif_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = data->dma_rx.buffer,
	};

	scif_async_user_callback(data, &evt);
}

static void uart_scif_dma_replace_buffer(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;
	int ret;

	if (!data->async_inited) {
		return;
	}
	/* Replace the buffer and reload the DMA */
	LOG_DBG("Replacing RX buffer: %d", data->rx_next_buffer_len);

	/* reload DMA */
	data->dma_rx.offset = 0;
	data->dma_rx.counter = 0;
	data->dma_rx.buffer = data->rx_next_buffer;
	data->dma_rx.buffer_length = data->rx_next_buffer_len;
	data->dma_rx.blk_cfg.block_size = data->dma_rx.buffer_length;
	data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
	ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &data->dma_rx.dma_cfg);
	if (ret < 0) {
		return;
	}
	ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
	if (ret < 0) {
		return;
	}

	/* Request next buffer */
	scif_async_evt_rx_buf_request(data);
}

static void uart_scif_dma_rx_flush(const struct device *dev)
{
	struct dma_status stat;
	struct uart_scif_data *data = dev->data;

	if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
		int32_t rx_rcv_len = data->dma_rx.buffer_length - stat.pending_length;

		if (rx_rcv_len > data->dma_rx.offset) {
			data->dma_rx.counter = rx_rcv_len;
			scif_async_evt_rx_rdy(data);
		}
	}
}

static int uart_scif_async_rx_disable(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;
	struct uart_event disabled_event = {
		.type = UART_RX_DISABLED
	};

	if (!data->dma_rx.enabled) {
		scif_async_user_callback(data, &disabled_event);
		return -EFAULT;
	}

	uart_scif_dma_rx_flush(dev);

	scif_async_evt_rx_buf_release(data);

	data->dma_rx.enabled = false;

	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);

	if (data->rx_next_buffer) {
		struct uart_event rx_next_buf_release_evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf.buf = data->rx_next_buffer,
		};

		scif_async_user_callback(data, &rx_next_buf_release_evt);
	}
	uart_scif_write_16(dev, SCSCR, uart_scif_read_16(dev, SCSCR) & ~(SCSCR_RE | SCSCR_RIE));

	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0;

	LOG_DBG("rx: disabled");

	scif_async_user_callback(data, &disabled_event);

	return 0;
}

static int uart_scif_async_tx(const struct device *dev, const uint8_t *buf, size_t buf_size,
			      int32_t timeout)
{
	struct uart_scif_data *data = dev->data;
	int ret;

	if (!data->dma_tx.dma_dev || !data->async_inited) {
		return -ENODEV;
	}

	if (data->dma_tx.buffer_length != 0) {
		return -EBUSY;
	}

	data->dma_tx.buffer = (uint8_t *)buf;
	data->dma_tx.buffer_length = buf_size;
	data->dma_tx.timeout = timeout;
	data->dma_tx.blk_cfg.block_size = buf_size;
	data->dma_tx.blk_cfg.source_address = (uint32_t)buf;
	data->dma_tx.blk_cfg.fifo_mode_control = 0;

	data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
	ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &data->dma_tx.dma_cfg);
	if (ret < 0) {
		return ret;
	}

	uart_scif_write_16(dev, SCSCR, uart_scif_read_16(dev, SCSCR) | SCSCR_TE | SCSCR_TIE);
	sys_cache_data_flush_range((void *)buf, buf_size);
	ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel);

	/* Start TX timer */
	scif_async_timer_start(&data->dma_tx.timeout_work, data->dma_tx.timeout);

	return ret;
}

static int uart_scif_async_tx_abort(const struct device *dev)
{
	struct uart_scif_data *data = dev->data;

	(void)k_work_cancel_delayable(&data->dma_tx.timeout_work);
	return 0;
}

static int uart_scif_async_rx_enable(const struct device *dev, uint8_t *buf, size_t buf_size,
				     int32_t timeout)
{
	struct uart_scif_data *data = dev->data;
	int ret;

	if (!data->dma_rx.dma_dev || !data->async_inited) {
		return -ENODEV;
	}

	if (data->dma_rx.enabled) {
		LOG_WRN("RX was already enabled");
		return -EBUSY;
	}

	data->dma_rx.offset = 0;
	data->dma_rx.buffer = buf;
	data->dma_rx.buffer_length = buf_size;
	data->dma_rx.counter = 0;
	data->dma_rx.timeout = timeout;

	data->dma_rx.blk_cfg.block_size = buf_size;
	data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;

	data->dma_rx.blk_cfg.fifo_mode_control = 0;

	data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
	ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &data->dma_rx.dma_cfg);

	if (ret != 0) {
		LOG_ERR("UART ERR: RX DMA config failed!");
		return -EINVAL;
	}
	sys_cache_data_invd_range(data->dma_rx.buffer, buf_size);
	if (dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel)) {
		LOG_ERR("UART ERR: RX DMA start failed!");
		return -EFAULT;
	}

	uart_scif_write_16(dev, SCSCR, uart_scif_read_16(dev, SCSCR) | SCSCR_RE | SCSCR_RIE);
	/* Enable RX DMA requests */
	data->dma_rx.enabled = true;

	/* Request next buffer */
	scif_async_evt_rx_buf_request(data);

	scif_async_timer_start(&data->dma_rx.timeout_work, data->dma_rx.timeout);
	/* Start the RX timer not null */
	LOG_DBG("async rx enabled");

	return ret;
}

static int uart_scif_async_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t buf_size)
{
	struct uart_scif_data *data = dev->data;

	if (!data->dma_rx.enabled) {
		return -EACCES;
	}
	if (data->rx_next_buffer) {
		return -EBUSY;
	}
	data->rx_next_buffer = buf;
	data->rx_next_buffer_len = buf_size;

	return 0;
}

static void dma_tx_callback(const struct device *dev, void *user_data,
			    uint32_t channel, int status)
{
	const struct device *uart_dev = user_data;
	struct uart_scif_data *data = uart_dev->data;
	struct dma_status stat;
	unsigned int key = irq_lock();

	(void)k_work_cancel_delayable(&data->dma_tx.timeout_work);
	if (!dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat)) {
		data->dma_tx.counter = data->dma_tx.buffer_length - stat.pending_length;
	}
	scif_async_evt_tx_done(data);
	data->dma_tx.buffer_length = 0;

	irq_unlock(key);
}

static void dma_rx_callback(const struct device *dev, void *user_data,
			    uint32_t channel, int status)
{
	const struct device *uart_dev = user_data;
	struct uart_scif_data *data = uart_dev->data;

	(void)k_work_cancel_delayable(&data->dma_rx.timeout_work);
	if (status < 0) {
		scif_async_evt_rx_err(data, status);
		return;
	}
	/* true since this functions occurs when buffer if full */
	data->dma_rx.counter = data->dma_rx.buffer_length;

	scif_async_evt_rx_rdy(data);

	if (data->rx_next_buffer != NULL) {
		scif_async_evt_rx_buf_release(data);

		/* replace the buffer when the current
		 * is full and not the same as the next
		 * one.
		 */
		uart_scif_dma_replace_buffer(uart_dev);
	} else {
		k_work_reschedule(&data->dma_rx.timeout_work, K_TICKS(1));
	}
}
#endif

static const struct uart_driver_api uart_scif_driver_api = {
	.poll_in = uart_scif_poll_in,
	.poll_out = uart_scif_poll_out,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_scif_configure,
	.config_get = uart_scif_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_scif_fifo_fill,
	.fifo_read = uart_scif_fifo_read,
	.irq_tx_enable = uart_scif_irq_tx_enable,
	.irq_tx_disable = uart_scif_irq_tx_disable,
	.irq_tx_ready = uart_scif_irq_tx_ready,
	.irq_rx_enable = uart_scif_irq_rx_enable,
	.irq_rx_disable = uart_scif_irq_rx_disable,
	.irq_rx_ready = uart_scif_irq_rx_ready,
	.irq_err_enable = uart_scif_irq_err_enable,
	.irq_err_disable = uart_scif_irq_err_disable,
	.irq_is_pending = uart_scif_irq_is_pending,
	.irq_update = uart_scif_irq_update,
	.irq_callback_set = uart_scif_irq_callback_set,
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_scif_async_callback_set,
	.tx = uart_scif_async_tx,
	.tx_abort = uart_scif_async_tx_abort,
	.rx_enable = uart_scif_async_rx_enable,
	.rx_disable = uart_scif_async_rx_disable,
	.rx_buf_rsp = uart_scif_async_rx_buf_rsp,
#endif  /* CONFIG_UART_ASYNC_API */
};

/* Device Instantiation */
#define UART_SCIF_DECLARE_CFG(n, soc_type, IRQ_FUNC_INIT)				\
	PINCTRL_DT_INST_DEFINE(n);							\
	static const struct uart_scif_cfg uart_scif_cfg_##n = {				\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),					\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),			\
		.mod_clk.module =							\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),			\
		.mod_clk.domain =							\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),			\
		.bus_clk.module =							\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),			\
		.bus_clk.domain =							\
			DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		.params = &port_params[soc_type],					\
		.type = soc_type,							\
		IRQ_FUNC_INIT								\
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SCIF_IRQ_CONFIG_FUNC_SCIFA(n)						\
	static void irq_config_func_##n(const struct device *dev)			\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(n),						\
			    DT_INST_IRQ(n, priority),					\
			    uart_scif_isr,						\
			    DEVICE_DT_INST_GET(n), 0);					\
											\
		irq_enable(DT_INST_IRQN(n));						\
	}

#define UART_SET_IRQ(n, name)								\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, name, irq),				\
			    DT_INST_IRQ_BY_NAME(n, name, priority),			\
			    uart_scif_isr,						\
			    DEVICE_DT_INST_GET(n), 0);					\
											\
		irq_enable(DT_INST_IRQ_BY_NAME(n, name, irq));

#define UART_SCIF_IRQ_CONFIG_FUNC_RZ(n)							\
	static void irq_config_func_##n(const struct device *dev)			\
	{										\
		UART_SET_IRQ(n, eri);							\
		UART_SET_IRQ(n, rxi);							\
		UART_SET_IRQ(n, txi);							\
		UART_SET_IRQ(n, tei);							\
	}
#define UART_SCIF_IRQ_CFG_FUNC_INIT(n)							\
	.irq_config_func = irq_config_func_##n
#define UART_SCIF_INIT_CFG(n, soc_type)						\
	UART_SCIF_DECLARE_CFG(n, soc_type, UART_SCIF_IRQ_CFG_FUNC_INIT(n))
#else
#define UART_SCIF_IRQ_CONFIG_FUNC(n)
#define UART_SCIF_IRQ_CFG_FUNC_INIT
#define UART_SCIF_INIT_CFG(n, soc_type)						\
	UART_SCIF_DECLARE_CFG(n, soc_type, UART_SCIF_IRQ_CFG_FUNC_INIT)
#endif

#ifdef CONFIG_UART_ASYNC_API
#define UART_DMA_CHANNEL(inst, dir, dir_cap, src_dev, dest_dev)				\
	.dma_##dir = {									\
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(inst, dir),				\
		(									\
			.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, dir)),	\
			.dma_channel = DT_INST_DMAS_CELL_BY_NAME(inst, dir, channel),	\
			.dma_conf = DT_INST_DMAS_CELL_BY_NAME(inst, dir, config),	\
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(inst, dir, slot),		\
		),									\
		(.dma_dev = NULL,))							\
		.dma_cfg = {								\
			.block_count = 1,						\
			.channel_direction = src_dev##_TO_##dest_dev,			\
			.source_data_size = 1,  /* 8bit default */			\
			.dest_data_size = 1,    /* 8bit default */			\
			.source_burst_length = 1, /* SINGLE transfer */			\
			.dest_burst_length = 1,						\
			.channel_priority = 0,						\
			.dma_callback = dma_##dir##_callback,				\
		},									\
		.src_addr_increment = dir_cap##_SRC_INCREMENT,				\
		.dst_addr_increment = dir_cap##_DST_INCREMENT,				\
		.name = #dir_cap							\
	},
#else
#define UART_DMA_CHANNEL(index, dir, DIR, src, dest)
#endif

#define UART_SCIF_XXX_INIT(n, soc_type)							\
	static struct uart_scif_data uart_scif_data_##n = {				\
		.current_config = {							\
			.baudrate = DT_INST_PROP(n, current_speed),			\
			.parity = UART_CFG_PARITY_NONE,					\
			.stop_bits = UART_CFG_STOP_BITS_1,				\
			.data_bits = UART_CFG_DATA_BITS_8,				\
			.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,				\
		},									\
		UART_DMA_CHANNEL(n, rx, RX, PERIPHERAL, MEMORY)				\
		UART_DMA_CHANNEL(n, tx, TX, MEMORY, PERIPHERAL)				\
	};										\
	static const struct uart_scif_cfg uart_scif_cfg_##n;				\
	DEVICE_DT_INST_DEFINE(n,							\
			      uart_scif_init,						\
			      NULL,							\
			      &uart_scif_data_##n,					\
			      &uart_scif_cfg_##n,					\
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,		\
			      &uart_scif_driver_api);					\
											\
	UART_SCIF_IRQ_CONFIG_FUNC(n)							\
	UART_SCIF_INIT_CFG(n, soc_type);

#define DT_DRV_COMPAT renesas_rcar_scif
#define UART_SCIF_SCIFA_INIT(n) UART_SCIF_XXX_INIT(n, SCIF_SCIFA_TYPE)
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_SCIF_IRQ_CONFIG_FUNC UART_SCIF_IRQ_CONFIG_FUNC_SCIFA
#endif
DT_INST_FOREACH_STATUS_OKAY(UART_SCIF_SCIFA_INIT)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT renesas_rza2m_scif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#undef UART_SCIF_IRQ_CONFIG_FUNC
#define UART_SCIF_IRQ_CONFIG_FUNC UART_SCIF_IRQ_CONFIG_FUNC_RZ
#endif
#define UART_SCIF_RZ_INIT(n) UART_SCIF_XXX_INIT(n, SCIF_RZ_SCIFA_TYPE)
DT_INST_FOREACH_STATUS_OKAY(UART_SCIF_RZ_INIT)
