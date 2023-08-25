/*
 * Copyright (c) 2023 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_RZA2_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_RZA2_H_

/**
 * @brief Read GPIO port value
 *
 * @param port number
 *
 * @return port register value
 */
int rza2_chip_get(uint8_t port);

/**
 * @brief Write GPIO port value
 *
 * @param port number
 * @param value to set to the port
 */
void rza2_chip_set(uint8_t port, uint8_t value);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_RZA2_H_ */
