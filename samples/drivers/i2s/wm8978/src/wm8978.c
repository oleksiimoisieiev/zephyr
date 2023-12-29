/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wolfson_wm8978

#define WM8978_CODEC_INIT_PRIORITY 90

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(codec_wm8978);

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/audio/codec.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>

/* ==== Register Address ==== */
#define WM8978_REG_SOFT_RESET		0x00
#define WM8978_REG_POW_MANAGE1		0x01
#define WM8978_REG_POW_MANAGE2		0x02
#define WM8978_REG_POW_MANAGE3		0x03
#define WM8978_REG_AUDIO_IF_CTL		0x04
#define WM8978_REG_COMPADING_CTL	0x05
#define WM8978_REG_CLK_GEN_CTL		0x06
#define WM8978_REG_DAC_CTL		0x0A
#define WM8978_REG_ADC_CTL		0x0E
#define WM8978_REG_INPUT_CTL		0x2C
#define WM8978_REG_LINPPGAGAIN		0x2D
#define WM8978_REG_RINPPGAGAIN		0x2E
#define WM8978_REG_LMIXER_CTL		0x32
#define WM8978_REG_RMIXER_CTL		0x33
#define WM8978_REG_LOUT1_VOL_CTL	0x34
#define WM8978_REG_ROUT1_VOL_CTL	0x35

/* ==== Register Setting Value ==== */
/* RESET R0 */
#define WM8978_RESET_INI_VALUE		0x000
/* WM8978_REG_POW_MANAGE1 R1 */
#define WM8978_MANAGE1_INI_VALUE	0x000
#define WM8978_MANAGE1_VMIDSEL_75K	BIT(0)
#define WM8978_MANAGE1_BUFIOEN_ON	BIT(2)
#define WM8978_MANAGE1_BIASEN_ON	BIT(3)
#define WM8978_MANAGE1_MICBEN_ON	BIT(4)
#define WM8978_MANAGE1_PLLEN_ON		BIT(5)
/* WM8978_REG_POW_MANAGE2 R2 */
#define WM8978_MANAGE2_INI_VALUE	0x000
#define WM8978_MANAGE2_ADCENL_ON	BIT(0)
#define WM8978_MANAGE2_ADCENR_ON	BIT(1)
#define WM8978_MANAGE2_INPPGAENL_ON	BIT(2)
#define WM8978_MANAGE2_INPPGAENR_ON	BIT(3)
#define WM8978_MANAGE2_BOOSTENL_ON	BIT(4)
#define WM8978_MANAGE2_BOOSTENR_ON	BIT(5)
#define WM8978_MANAGE2_LOUT1EN_ON	BIT(7)
#define WM8978_MANAGE2_ROUT1EN_ON	BIT(8)
/* WM8978_REG_POW_MANAGE3 R3 */
#define WM8978_MANAGE3_INI_VALUE	0x000
#define WM8978_MANAGE3_DACENL_ON	BIT(0)
#define WM8978_MANAGE3_DACENR_ON	BIT(1)
#define WM8978_MANAGE3_LMIXEN_ON	BIT(2)
#define WM8978_MANAGE3_RMIXEN_ON	BIT(3)
/* WM8978_REG_AUDIO_IF_CTL R4 */
#define WM8978_AUDIO_IF_INI_VALUE	0x050
#define WM8978_AUDIO_IF_WL_BIT		0x060
#define WM8978_AUDIO_IF_WL_16BIT	0x000
/* WM8978_REG_COMPADING_CTL R5 */
#define WM8978_COMPADING_LOOPBACK	BIT(0)
/* WM8978_REG_CLK_GEN_CTL  R6 */
#define WM8978_CLK_GEN_CTL_INI_VALUE	0x040
#define WM8978_CLK_GEN_CTL_MCLKDIV_BIT	0x0e0
#define WM8978_CLK_GEN_CTL_MCLKDIV_DIV1	0x000
/* WM8978_REG_ADC_CTL R14 */
#define WM8978_ADC_CTL_INI_VALUE	0x100
#define WM8978_ADC_CTL_HPFEN_BIT	0x100
#define WM8978_ADC_CTL_ADCOSR128_ON	0x008
/* WM8978_REG_DAC_CTL R10 */
#define WM8978_DAC_CTL_INI_VALUE	0x000
#define WM8978_DAC_CTL_DACOSR128_ON	0x008
/* WM8978_REG_INPUT_CTL R44 */
#define WM8978_INPUTCTL_INI_VALUE	0x033
#define WM8978_INPUTCTL_L2_2INPPGA_ON	0x004
#define WM8978_INPUTCTL_R2_2INPPGA_ON	0x040
/* WM8978_REG_LINPPGAGAIN R45 */
#define WM8978_LINPPGAGAIN_INI_VOLL	0x018
#define WM8978_LINPPGAGAIN_MUTEL_ON	0x040
/* WM8978_REG_RINPPGAGAIN R46 */
#define WM8978_RINPPGAGAIN_INI_VOLL	0x018
#define WM8978_RINPPGAGAIN_MUTER_BIT	0x040
/* WM8978_REG_LMIX_CTL R50 */
#define WM8978_LMIX_CTL_INI_VALUE	0x001
#define WM8978_LMIX_CTL_DACL2LMIX_BIT	0x001
#define WM8978_LMIX_CTL_BYPL2LMIX_BIT	0x002
/* WM8978_REG_RMIX_CTL R51 */
#define WM8978_RMIX_CTL_INI_VALUE	0x001
#define WM8978_RMIX_CTL_DACR2RMIX_BIT	0x001
#define WM8978_RMIX_CTL_BYPR2RMIX_BIT	0x002
/* WM8978_REG_LOUT1_VOL_CTL R52 */
#define WM8978_LOUT1_HPVU_BIT		0x100
/* WM8978_REG_ROUT1_VOL_CTL R53 */
#define WM8978_ROUT1_HPVU_BIT		0x100

#define WM8978_CMD(reg, val)	((((reg) << 9) & 0xFE00) | ((val) & 0x1FF))

struct wm8978_cfg {
	struct spi_dt_spec bus;
};

struct wm8978_data {
	uint8_t	prop1;
};

static int wm8978_write_bus_spi(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct wm8978_cfg *config = dev->config;

	struct spi_buf tx_buf = {
		.buf = buf,
		.len = len
	};

	struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1
	};

	return spi_write_dt(&config->bus, &tx_bufs);
}

static inline int wm8978_write(const struct device *dev, uint16_t value)
{
	return wm8978_write_bus_spi(dev, (uint8_t *)&value, sizeof(uint16_t));
}

static int wm8978_init(const struct device *dev)
{
	const struct wm8978_cfg *cfg = dev->config;

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("Bus device %s not ready!", cfg->bus.bus->name);
		return -EINVAL;
	}
#ifdef CONFIG_WM8978_ANALOG_LOOPBACK
	LOG_WRN("Analog loopback");
#endif
#ifdef CONFIG_WM8978_LOOPBACK
	LOG_WRN("Loopback");
#endif
#ifdef CONFIG_WM8978_MIXER
	LOG_WRN("Analog mixer");
#endif
	return 0;
}

static int codec_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	uint16_t reg;
	uint16_t temp;

	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	wm8978_write(dev, WM8978_CMD(WM8978_REG_SOFT_RESET, WM8978_RESET_INI_VALUE));

	/* ==== Set L/RMIXEN = 1 and DACENL/R = 1 in register R3.  ==== */
	reg = WM8978_MANAGE3_INI_VALUE | WM8978_MANAGE3_RMIXEN_ON | WM8978_MANAGE3_LMIXEN_ON |
	      WM8978_MANAGE3_DACENL_ON | WM8978_MANAGE3_DACENR_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_POW_MANAGE3, reg));

	/* ==== Set BUFIOEN = 1 and VMIDSEL[1:0] to required value in register R1. ==== */
	reg = WM8978_MANAGE1_INI_VALUE | WM8978_MANAGE1_BUFIOEN_ON | WM8978_MANAGE1_VMIDSEL_75K |
	      WM8978_MANAGE1_MICBEN_ON | WM8978_MANAGE1_PLLEN_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_POW_MANAGE1, reg));
	/* ==== Set BIASEN = 1 in register R1.  ==== */
	reg |= WM8978_MANAGE1_BIASEN_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_POW_MANAGE1, reg));

	/* ==== Set L/ROUT1EN = 1 in register R2.  ==== */
	reg = WM8978_MANAGE2_INI_VALUE | WM8978_MANAGE2_LOUT1EN_ON | WM8978_MANAGE2_ROUT1EN_ON |
	      WM8978_MANAGE2_BOOSTENL_ON | WM8978_MANAGE2_BOOSTENR_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_POW_MANAGE2, reg));
	/* ==== Set INPPGAENL = 1 in register R2.  ==== */
	reg |= WM8978_MANAGE2_INPPGAENL_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_POW_MANAGE2, reg));
	/* ==== Set INPPGAENR = 1 in register R2.  ==== */
	reg |= WM8978_MANAGE2_INPPGAENR_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_POW_MANAGE2, reg));
	temp = reg;
#ifdef CONFIG_WM8978_LOOPBACK
	LOG_WRN("Set loopback");
	/* ==== Set LOOPBACK = 1 in register R5.  ==== */
	reg = WM8978_COMPADING_LOOPBACK;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_COMPADING_CTL, reg));
#endif
	/* ==== Set L2_2INPPGA = 1 and R2_2INPPGA = 1 in register R44.  ==== */
	reg = WM8978_INPUTCTL_INI_VALUE | WM8978_INPUTCTL_L2_2INPPGA_ON |
	      WM8978_INPUTCTL_R2_2INPPGA_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_INPUT_CTL, reg));

	/* ==== Set LINPPGAGAIN = 0x0018 and MUTEL = 0 in register R45.  ==== */
	reg = WM8978_LINPPGAGAIN_INI_VOLL & ~(WM8978_LINPPGAGAIN_MUTEL_ON);
	wm8978_write(dev, WM8978_CMD(WM8978_REG_LINPPGAGAIN, reg));

	/* ==== Set RINPPGAGAIN = 0x0018 and MUTEL = 0 in register R46.  ==== */
	reg = WM8978_RINPPGAGAIN_INI_VOLL & ~WM8978_RINPPGAGAIN_MUTER_BIT;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_RINPPGAGAIN, reg));

	/* ==== Set ADCENL/ADCENR = 1 in register R2.  ==== */
	reg = temp | WM8978_MANAGE2_INPPGAENR_ON | WM8978_MANAGE2_ADCENL_ON |
	      WM8978_MANAGE2_ADCENR_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_POW_MANAGE2, reg));

	/* ==== Set ADCOSR128 = 1 in register R14.  ==== */
	reg = WM8978_ADC_CTL_INI_VALUE | WM8978_ADC_CTL_ADCOSR128_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_ADC_CTL, reg));

	/* ==== Set HPFEN = 0 in register R14.  ==== */
	reg &= ~WM8978_ADC_CTL_HPFEN_BIT;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_ADC_CTL, reg));

	/* ==== Set MCLKDIV = 0 in register R6.  ==== */
	reg = (WM8978_CLK_GEN_CTL_INI_VALUE & ~WM8978_CLK_GEN_CTL_MCLKDIV_BIT) |
	      WM8978_CLK_GEN_CTL_MCLKDIV_DIV1;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_CLK_GEN_CTL, reg));

	/* ==== Set WL = 0 in register R4.  ==== */
	reg = (WM8978_AUDIO_IF_INI_VALUE & ~WM8978_AUDIO_IF_WL_BIT) | WM8978_AUDIO_IF_WL_16BIT;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_AUDIO_IF_CTL, reg));

	/* ==== Set DACOSR128 = 1 in register R10.  ==== */
	reg = WM8978_DAC_CTL_INI_VALUE | WM8978_DAC_CTL_DACOSR128_ON;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_DAC_CTL, reg));

#ifdef CONFIG_WM8978_ANALOG_LOOPBACK
	LOG_WRN("Set analog loopback");
	/* Set BYPL2LMIX = 0 in register R50. */
	reg = WM8978_LMIX_CTL_INI_VALUE;
#ifndef CONFIG_WM8978_MIXER
	LOG_WRN("Clear mixer");
	reg &= ~WM8978_LMIX_CTL_DACL2LMIX_BIT;
#endif
	reg |= WM8978_LMIX_CTL_BYPL2LMIX_BIT;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_LMIXER_CTL, reg));

	/* Set BYPR2RMIX = 1 in register R51. */
	reg = WM8978_RMIX_CTL_INI_VALUE;
#ifndef CONFIG_WM8978_MIXER
	LOG_WRN("Clear mixer");
	reg &= ~WM8978_RMIX_CTL_DACR2RMIX_BIT;
#endif
	reg |= WM8978_RMIX_CTL_BYPR2RMIX_BIT;
	wm8978_write(dev, WM8978_CMD(WM8978_REG_RMIXER_CTL, reg));
#endif
	return 0;
}

static void codec_start_output(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void codec_stop_output(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static int codec_set_property(const struct device *dev, audio_property_t property,
			      audio_channel_t channel, audio_property_value_t val)
{
	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
	case AUDIO_PROPERTY_OUTPUT_MUTE:
	default:
		break;
	}

	return -EINVAL;
}

static int codec_apply_properties(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* nothing to do because there is nothing cached */
	return 0;
}

static const struct audio_codec_api wm8978_api = {
	.configure		= codec_configure,
	.start_output		= codec_start_output,
	.stop_output		= codec_stop_output,
	.set_property		= codec_set_property,
	.apply_properties	= codec_apply_properties,
};

#define WM8978_CONFIG_SPI(n)								\
	.bus = SPI_DT_SPEC_INST_GET(							\
			n, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(16), 0),

#define WM8978_DEFINE(n)								\
	static struct wm8978_data wm8978_data_##n;					\
	static const struct wm8978_cfg wm8978_cfg_##n = {				\
		WM8978_CONFIG_SPI(n)							\
	};										\
	DEVICE_DT_INST_DEFINE(n, wm8978_init, NULL, &wm8978_data_##n, &wm8978_cfg_##n,	\
			 POST_KERNEL, WM8978_CODEC_INIT_PRIORITY, &wm8978_api);

DT_INST_FOREACH_STATUS_OKAY(WM8978_DEFINE)
