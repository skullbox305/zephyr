/*
 * Copyright (c) 2023 Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atlas_oem

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "atlas_scientific_oem.h"

#include <stdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(ATLAS_SCIENTIFIC_OEM, CONFIG_SENSOR_LOG_LEVEL);

static void atlas_scientific_oem_thread_cb(const struct device *dev)
{
	struct atlas_scientific_oem_data *drv_data = dev->data;
	uint16_t status;

	/* Clear the status */

	k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
	if ((drv_data->drdy_handler != NULL) && ATLAS_SCIENTIFIC_OEM_STATUS_DRDY(status)) {
		drv_data->drdy_handler(dev, drv_data->drdy_trigger);
	}
	k_mutex_unlock(&drv_data->trigger_mutex);
}

static void atlas_scientific_oem_gpio_callback(const struct device *dev, struct gpio_callback *cb,
					       uint32_t pins)
{
	struct atlas_scientific_oem_data *drv_data =
		CONTAINER_OF(cb, struct atlas_scientific_oem_data, gpio_cb);

#ifdef CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER_OWN_THREAD
	k_sem_give(&drv_data->gpio_sem);
#elif CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER_GLOBAL_THREAD
	k_work_submit(&drv_data->work);
#endif
}

#ifdef CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER_OWN_THREAD
static void atlas_scientific_oem_thread(struct atlas_scientific_oem_data *drv_data)
{
	while (true) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		atlas_scientific_oem_thread_cb(drv_data->dev);
	}
}

#elif CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER_GLOBAL_THREAD
static void atlas_scientific_oem_work_cb(struct k_work *work)
{
	struct atlas_scientific_oem_data *drv_data =
		CONTAINER_OF(work, struct atlas_scientific_oem_data, work);

	atlas_scientific_oem_thread_cb(drv_data->dev);
}
#endif

int atlas_scientific_oem_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
				     sensor_trigger_handler_t handler)
{
	struct atlas_scientific_oem_data *drv_data = dev->data;
	const struct atlas_scientific_oem_config *cfg = dev->config;
	uint16_t status, int_mask, int_en;
	int ret;

	gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		k_mutex_lock(&drv_data->trigger_mutex, K_FOREVER);
		drv_data->drdy_handler = handler;
		drv_data->drdy_trigger = trig;
		k_mutex_unlock(&drv_data->trigger_mutex);

		// int_mask = ;
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		ret = -ENOTSUP;
		goto out;
	}

	if (handler) {
		int_en = int_mask;
		drv_data->int_config |= int_mask;
	} else {
		int_en = 0U;
	}

	ret = atlas_scientific_oem_reg_write_mask(dev, ATLAS_SCIENTIFIC_OEM_ERROR_CONFIG, int_mask,
						  int_en);

	/* Clear INTB pin by reading STATUS register */
	atlas_scientific_oem_get_status(dev, &status);
out:
	gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);

	return ret;
}

int atlas_scientific_oem_init_interrupt(const struct device *dev)
{
	struct atlas_scientific_oem_data *drv_data = dev->data;
	const struct atlas_scientific_oem_config *cfg = dev->config;
	int ret;

	k_mutex_init(&drv_data->trigger_mutex);

	if (!device_is_ready(cfg->int_gpio.port)) {
		LOG_ERR("%s: int_gpio device not ready", cfg->int_gpio.port->name);
		return -ENODEV;
	}

	ret = atlas_scientific_oem_set_interrupt_pin(dev, true);
	if (ret) {
		return ret;
	}

	gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);

	gpio_init_callback(&drv_data->gpio_cb, atlas_scientific_oem_gpio_callback,
			   BIT(cfg->int_gpio.pin));

	if (gpio_add_callback(cfg->int_gpio.port, &drv_data->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback!");
		return -EIO;
	}

	drv_data->dev = dev;

#ifdef CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER_OWN_THREAD
	k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_ATLAS_SCIENTIFIC_OEM_THREAD_STACK_SIZE,
			(k_thread_entry_t)atlas_scientific_oem_thread, drv_data, 0, NULL,
			K_PRIO_COOP(CONFIG_ATLAS_SCIENTIFIC_OEM_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER_GLOBAL_THREAD
	drv_data->work.handler = atlas_scientific_oem_work_cb;
#endif

	return 0;
}
