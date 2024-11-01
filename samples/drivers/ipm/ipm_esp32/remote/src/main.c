/*
 * Copyright (c) 2022 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/gpio.h>

static const struct device *ipm_dev;
static char resp[64];
static uint32_t msg_cnt;
struct k_sem sync;

static void ipm_receive_callback(const struct device *ipmdev, void *user_data,
			       uint32_t id, volatile void *data)
{
	k_sem_give(&sync);
}

int main(void)
{
	int ret;

	k_sem_init(&sync, 0, 1);

	ipm_dev = DEVICE_DT_GET(DT_NODELABEL(ipm0));
	if (!ipm_dev) {
		printk("Failed to get IPM device.\n\r");
		return 0;
	}

	ipm_register_callback(ipm_dev, ipm_receive_callback, NULL);

	while (1) {
		ret = snprintf(resp, sizeof(resp), "Response id %i\n", ++msg_cnt);
		if (ret < 0) {
			continue;
		} else {
			k_sem_take(&sync, K_FOREVER);
			ipm_send(ipm_dev, -1, sizeof(resp), &resp, sizeof(resp));
		}
	}

	return 0;
}
