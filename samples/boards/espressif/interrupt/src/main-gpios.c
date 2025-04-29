/*
 * Copyright (c) 2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/interrupt_util.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
//#include <zephyr/devicetree.h>
//#include <zephyr/sys/printk.h>

#define PINA 8
#define PINB 9
static const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static struct gpio_callback gpio_cb_data;

static const struct device *timer0 = DEVICE_DT_GET(DT_NODELABEL(timer0));
static const struct device *timer1 = DEVICE_DT_GET(DT_NODELABEL(timer1));
static const struct device *timer2 = DEVICE_DT_GET(DT_NODELABEL(timer2));

void pin_triggered(const struct device *dev, struct gpio_callback *cb,
            uint32_t pins)
{
    printk("PIN pressed at %" PRIu32 "\n", k_cycle_get_32());
    gpio_pin_toggle(gpio, PINB);
}

#define ALARM_A_US 1000
#define ALARM_B_US 2000
#define ALARM_C_US 3000
#define DELAY_US 3000
#define CYCLE_MS 10

static struct counter_alarm_cfg alarm_a;
static struct counter_alarm_cfg alarm_b;
static struct counter_alarm_cfg alarm_c;

static int token = 0;
static int errors = 0;

static void alarm_handler_c(const struct device *dev, uint8_t chan_id,
			   uint32_t counter,
			   void *user_data)
{
	if (dev == timer2) {
		token++;
	}
	counter_stop(dev);
}

static void alarm_handler_b(const struct device *dev, uint8_t chan_id,
			   uint32_t counter,
			   void *user_data)
{
	if (dev == timer1) {
		token++;
	}
	counter_stop(dev);
	k_busy_wait(DELAY_US);
}

static void alarm_handler_a(const struct device *dev, uint8_t chan_id,
			   uint32_t counter,
			   void *user_data)
{
	if (dev == timer0) {
		token++;
	}
	counter_stop(dev);
	k_busy_wait(DELAY_US);
}


int main(void)
{
	int ret;
#if 0
	if (!device_is_ready(gpio)) {
		printk("Error: gpio device %s is not ready\n",
		       gpio->name);
		return 0;
	}
	printk("Done: gpio device %s is ready\n", gpio->name);

	ret = gpio_pin_configure(gpio, PINB, GPIO_OUTPUT | GPIO_PULL_UP);

	ret = gpio_pin_configure(gpio, PINA, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		printk("Error %d: failed to configure %s pin %d\n", ret, gpio->name, PINA);
		return 0;
	}
	printk("Done: configured %s pin %d\n", gpio->name, PINA);

	ret = gpio_pin_interrupt_configure(gpio, PINA, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, gpio->name, PINA);
		return 0;
	}

	gpio_init_callback(&gpio_cb_data, pin_triggered, BIT(PINA));
	gpio_add_callback(gpio, &gpio_cb_data);
	printk("Set up gpio at %s pin %d\n", gpio->name, PINA);
#endif
	printk("counter 0 channels %d\n", counter_get_num_of_channels(timer0));
	printk("counter 0 frequency %d\n", counter_get_frequency(timer0));
	printk("counter 0 top value %u\n", counter_get_top_value(timer0));

	printk("counter 1 channels %d\n", counter_get_num_of_channels(timer1));
	printk("counter 1 frequency %d\n", counter_get_frequency(timer1));
	printk("counter 1 top value %u\n", counter_get_top_value(timer1));

	printk("counter 2 channels %d\n", counter_get_num_of_channels(timer2));
	printk("counter 2 frequency %d\n", counter_get_frequency(timer2));
	printk("counter 2 top value %u\n", counter_get_top_value(timer2));

	// timer isr

	if (!device_is_ready(timer0)) {
		printk("Error: device %s is not ready\n",
		       timer0->name);
		return 0;
	}
	printk("Done: timer0 device %s is ready\n", timer0->name);

	if (!device_is_ready(timer1)) {
		printk("Error: device %s is not ready\n",
		       timer1->name);
		return 0;
	}
	printk("Done: timer1 device %s is ready\n", timer1->name);

	if (!device_is_ready(timer2)) {
		printk("Error: device %s is not ready\n",
		       timer2->name);
		return 0;
	}
	printk("Done: timer1 device %s is ready\n", timer2->name);

	while (1) {
		ret = counter_start(timer0);
		if (ret != 0) {
		    printk("Failed to start counter 0: %d\n", ret);
		    return 1;
		}
		ret = counter_start(timer1);
		if (ret != 0) {
		    printk("Failed to start counter 1: %d\n", ret);
		    return 1;
		}
		ret = counter_start(timer2);
		if (ret != 0) {
		    printk("Failed to start counter 1: %d\n", ret);
		    return 1;
		}

		/* Configure alarms */
		alarm_a.ticks = counter_us_to_ticks(timer0, ALARM_A);
		alarm_a.callback = alarm_handler_a;

		alarm_b.ticks = counter_us_to_ticks(timer1, ALARM_B);
		alarm_b.callback = alarm_handler_b;

		alarm_c.ticks = counter_us_to_ticks(timer2, ALARM_C);
		alarm_c.callback = alarm_handler_c;

		counter_reset(timer0);
		counter_reset(timer1);
		counter_reset(timer2);

		ret = counter_set_channel_alarm(timer0, 0, &alarm_a);
		if (ret != 0) {
		    printk("Failed to set alarm A: %d\n", ret);
		    return 1;
		}
		ret = counter_set_channel_alarm(timer1, 0, &alarm_b);
		if (ret != 0) {
		    printk("Failed to set alarm B: %d\n", ret);
		    return 1;
		}

		ret = counter_set_channel_alarm(timer2, 0, &alarm_c);
		if (ret != 0) {
		    printk("Failed to set alarm C: %d\n", ret);
		    return 1;
		}

		k_msleep(CYCLE_MS);

		if (token == 3) {
			printk(".");
		} else {
			printk("E");
			errors ++;
		}

		if (errors) {
			printk("\nerr_cnt=%d\n", errors);
		}

		token = 0;
	}

	return 0;
}
