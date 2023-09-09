/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

int main(void)
{
	uint8_t test = 0;
 	printk("Hello World! %s\n", CONFIG_BOARD);

	for(uint8_t i = 0; i < 255; i++) {
		test++;
		printk("%i\n", test);
	}
	return 0;
}
