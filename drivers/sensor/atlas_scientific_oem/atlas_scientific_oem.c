/* atlas_scientific_oem.c - Driver for the Atlas Scientific OEM sensors (pH, EC, ORP, D.O. and RTD)
 */

/*
 * Copyright (c) 2023 Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atlas_oem

#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "atlas_scientific_oem.h"
#include <zephyr/drivers/sensor/atlas_scientific_oem.h>

LOG_MODULE_REGISTER(ATLAS_SCIENTIFIC_OEM, CONFIG_SENSOR_LOG_LEVEL);

static int atlas_scientific_oem_init_config(const struct device *dev);

/**
 * Read/Write from device.
 * @param dev - The device structure.
 * @param reg - The register address. Use ATLAS_SCIENTIFIC_OEM_REG_READ(x) or
 *				ATLAS_SCIENTIFIC_OEM_REG_WRITE(x).
 * @param data - The register data.
 * @param length - Number of bytes being read
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_bus_access(const struct device *dev, uint8_t reg, uint8_t *data,
					   size_t length)
{
	const struct atlas_scientific_oem_config *cfg = dev->config;

	if (reg & ATLAS_SCIENTIFIC_OEM_READ) {
		return i2c_burst_read_dt(&cfg->i2c, ATLAS_SCIENTIFIC_OEM_TO_I2C_REG(reg), data,
					 length);
	} else {
		if (length != 2) {
			return -EINVAL;
		}

		uint8_t buf[3];

		buf[0] = ATLAS_SCIENTIFIC_OEM_TO_I2C_REG(reg);
		memcpy(buf + 1, data, sizeof(uint16_t));

		return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
	}
}

/**
 * Read (16 Bit) from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_reg_read(const struct device *dev, uint8_t reg_addr,
					 uint16_t *reg_data)
{
	uint8_t buf[2];
	int ret;

	ret = atlas_scientific_oem_bus_access(dev, ATLAS_SCIENTIFIC_OEM_REG_READ(reg_addr), buf, 2);
	*reg_data = ((uint16_t)buf[0] << 8) | buf[1];

	return ret;
}

/**
 * Write (16 Bit) to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_reg_write(const struct device *dev, uint8_t reg_addr,
					  uint16_t reg_data)
{
	LOG_DBG("[0x%x] = 0x%x", reg_addr, reg_data);

	uint8_t buf[2];

	buf[0] = (uint8_t)(reg_data >> 8);
	buf[1] = (uint8_t)reg_data;

	return atlas_scientific_oem_bus_access(dev, ATLAS_SCIENTIFIC_OEM_REG_WRITE(reg_addr), buf,
					       2);
}

/**
 * I2C write (16 Bit) to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int atlas_scientific_oem_reg_write_mask(const struct device *dev, uint8_t reg_addr, uint16_t mask,
					uint16_t data)
{
	int ret;
	uint16_t tmp;

	ret = atlas_scientific_oem_reg_read(dev, reg_addr, &tmp);
	if (ret) {
		return ret;
	}
	LOG_DBG("read [0x%x] = 0x%x", reg_addr, tmp);
	LOG_DBG("mask: 0x%x", mask);

	tmp &= ~mask;
	tmp |= data;

	return atlas_scientific_oem_reg_write(dev, reg_addr, tmp);
}

#ifdef CONFIG_PM_DEVICE

/**
 * Set the Device Power Management State.
 * @param dev - The device structure.
 * @param pm_state - power management state
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_device_pm_action(const struct device *dev,
						 enum pm_device_action action)
{
	int ret;
	const struct atlas_scientific_oem_config *cfg = dev->config;
	enum pm_device_state curr_state;

	(void)pm_device_state_get(dev, &curr_state);

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:

		break;
	case PM_DEVICE_ACTION_SUSPEND:

		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif

/**
 * Set attributes for the device.
 * @param dev - The device structure.
 * @param chan - The sensor channel type.
 * @param attr - The sensor attribute.
 * @param value - The sensor attribute value.
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_attr_set(const struct device *dev, enum sensor_channel chan,
					 enum sensor_attribute attr, const struct sensor_value *val)
{
	return -ENOTSUP;
}

/**
 * Fetch sensor data from the device.
 * @param dev - The device structure.
 * @param chan - The sensor channel type.
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;

	(void)pm_device_state_get(dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		LOG_ERR("Sample fetch failed, device is not in active mode");
		return -ENXIO;
	}
#endif

	return 0;
}

/**
 * Get sensor channel value from the device.
 * @param dev - The device structure.
 * @param chan - The sensor channel type.
 * @param val - The sensor channel value.
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_channel_get(const struct device *dev, enum sensor_channel chan,
					    struct sensor_value *val)
{
	//const struct atlas_scientific_oem_config *cfg = dev->config;
	//double ch_data;

	switch ((int16_t)chan) {
	case 0:

		break;
	default:
		LOG_ERR("Channel type not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api atlas_scientific_oem_api_funcs = {
	.attr_set = atlas_scientific_oem_attr_set,
	.sample_fetch = atlas_scientific_oem_sample_fetch,
	.channel_get = atlas_scientific_oem_channel_get,
#ifdef CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER
	.trigger_set = atlas_scientific_oem_trigger_set,
#endif
};

static int atlas_scientific_oem_init_config(const struct device *dev)
{

	return 0;
}

/**
 * Probe device (Check if it is the correct device).
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_probe(const struct device *dev)
{

	return 0;
}

/**
 * Initialization of the device.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int atlas_scientific_oem_init(const struct device *dev)
{
	//const struct atlas_scientific_oem_config *cfg = dev->config;

	return 0;
}

#define ATLAS_SCIENTIFIC_OEM_INT_PROPS(n) .int_gpio = GPIO_DT_SPEC_INST_GET(n, int_gpios),

#define ATLAS_SCIENTIFIC_OEM_INT(n)                                                                \
	IF_ENABLED(CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER, (ATLAS_SCIENTIFIC_OEM_INT_PROPS(n)))

#define ATLAS_SCIENTIFIC_OEM_INIT(n)                                                               \
	static struct atlas_scientific_oem_data atlas_scientific_oem_data_##n;                      \
                                                                                                   \
	static const struct atlas_scientific_oem_config atlas_scientific_oem_config_##n = {        \
		.i2c = I2C_DT_SPEC_INST_GET(n), ATLAS_SCIENTIFIC_OEM_INT(n)};                      \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(n, atlas_scientific_oem_device_pm_action);                        \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(                                                              \
		n, atlas_scientific_oem_init, PM_DEVICE_DT_INST_GET(n),                            \
		&atlas_scientific_oem_data_##n, &atlas_scientific_oem_config_##n, POST_KERNEL,     \
		CONFIG_SENSOR_INIT_PRIORITY, &atlas_scientific_oem_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(ATLAS_SCIENTIFIC_OEM_INIT)
