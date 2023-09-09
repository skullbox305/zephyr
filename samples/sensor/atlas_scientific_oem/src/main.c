/*
 * Copyright (c) 2023 Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/atlas_scientific_oem.h>
#include <stdio.h>

#define CH_BUF_INIT(m) {},

K_SEM_DEFINE(sem, 0, 1);

#ifdef CONFIG_ATLAS_SCIENTIFIC_OEM_TRIGGER
static void trigger_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
	switch (trigger->type) {
	case SENSOR_TRIG_DATA_READY:
		if (sensor_sample_fetch(dev)) {
			printk("Sample fetch error\n");
			return;
		}
		k_sem_give(&sem);
		break;
	default:
		printk("Unknown trigger\n");
	}
}
#endif

#ifdef CONFIG_PM_DEVICE
static void pm_info(enum pm_device_action action, int status)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		printk("Enter ACTIVE_STATE ");
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		printk("Enter SUSPEND_STATE ");
		break;
	default:
		printk("Unknown power state");
	}

	if (status) {
		printk("Fail\n");
	} else {
		printk("Success\n");
	}
}
#endif

#define DEVICE_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(atlas_oem)

int main(void)
{

	return 0;
}
