/*
 * Copyright (c) 2023 Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ATLAS_SCIENTIFIC_OEM_ATLAS_SCIENTIFIC_OEM_H_
#define ZEPHYR_DRIVERS_SENSOR_ATLAS_SCIENTIFIC_OEM_ATLAS_SCIENTIFIC_OEM_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/*
 * ATLAS_SCIENTIFIC_OEM registers definition
 */

#define ATLAS_SCIENTIFIC_OEM_READ                    0x01u
#define ATLAS_SCIENTIFIC_OEM_REG_READ(x)             (((x & 0xFF) << 1) | ATLAS_SCIENTIFIC_OEM_READ)
#define ATLAS_SCIENTIFIC_OEM_REG_WRITE(x)            ((x & 0xFF) << 1)
#define ATLAS_SCIENTIFIC_OEM_TO_I2C_REG(x)           ((x) >> 1)

enum atlas_scientific_oem_op_mode {
	ATLAS_SCIENTIFIC_OEM_ACTIVE_MODE,
	ATLAS_SCIENTIFIC_OEM_SLEEP_MODE
};

struct atlas_scientific_oem_data {

#ifdef CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER
	struct gpio_callback gpio_cb;
	uint16_t int_config;

	struct k_mutex trigger_mutex;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;
	const struct device *dev;

#ifdef CONFIGATLAS_SCIENTIFIC_OEM_TRIGGER_OWN_THREAD
	K_THREAD_STACK_MEMBER(thread_stack, CONFIGATLAS_SCIENTIFIC_OEM_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif CONFIGATLAS_SCIENTIFIC_OEM_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
#endif /* CONFIGATLAS_SCIENTIFIC_OEM_TRIGGER */
};

struct atlas_scientific_oem_config {
	struct i2c_dt_spec i2c;

#ifdef CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif
};

int atlas_scientific_oem_reg_write_mask(const struct device *dev, uint8_t reg_addr, uint16_t mask,
					uint16_t data);

#ifdef CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER

int atlas_scientific_oem_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
				     sensor_trigger_handler_t handler);

int atlas_scientific_oem_init_interrupt(const struct device *dev);
#endif /* CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_ATLAS_SCIENTIFIC_OEM_ATLAS_SCIENTIFIC_OEM_H_ */
