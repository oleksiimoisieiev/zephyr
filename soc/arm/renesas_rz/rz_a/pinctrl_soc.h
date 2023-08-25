/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_SOC_ARM_RENESAS_RZA2_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_RENESAS_RZA2_PINCTRL_SOC_H_

#include <zephyr/types.h>

typedef struct {
	uint8_t port;
	uint8_t pin;
	uint8_t func;
} pinctrl_soc_pin_t;

#define RZA2_PINS_PER_PORT 8

/*
 * According to the HW Manual - pinctrl mux function for each port is represented by 3bit field
 * so the maximum value is 7. Define GPIO_FUNC_INPUT to 8 and GPIO_FUNC_OUTPUT to 9 to explicitly
 * set pin to GPIO input or output mode.
 */
#define GPIO_FUNC_INPUT 8
#define GPIO_FUNC_OUTPUT 9

/*
 * Use 16 lower bits [15:0] for pin identifier
 * Use 16 higher bits [31:16] for pin mux function
 */
#define MUX_PIN_ID_MASK   GENMASK(15, 0)
#define MUX_FUNC_MASK     GENMASK(31, 16)
#define MUX_FUNC_OFFS     16
#define RZA_FUNC(prop) ((prop & MUX_FUNC_MASK) >> MUX_FUNC_OFFS)
#define RZA_PINCODE(prop) (prop & MUX_PIN_ID_MASK)
#define RZA_PORT(prop) ((RZA_PINCODE(prop)) / RZA2_PINS_PER_PORT)
#define RZA_PIN(prop)  ((RZA_PINCODE(prop)) % RZA2_PINS_PER_PORT)

/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param state_prop State property name.
 * @param idx State property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, state_prop, idx)		\
	{								\
		.port = RZA_PORT(DT_PROP_BY_IDX(node_id, state_prop, idx)), \
		.pin = RZA_PIN(DT_PROP_BY_IDX(node_id, state_prop, idx)), \
		.func = RZA_FUNC(DT_PROP_BY_IDX(node_id, state_prop, idx)), \
	},

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			\
	{ DT_FOREACH_PROP_ELEM(DT_PHANDLE(node_id, prop), pinmux, Z_PINCTRL_STATE_PIN_INIT) };

#endif /* ZEPHYR_SOC_ARM_RENESAS_RZA2_PINCTRL_SOC_H_ */
