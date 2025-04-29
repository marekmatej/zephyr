/*
 * Copyright (c) 2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

//#include <zephyr/interrupt_util.h>   // ESP is missing this
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>


/* Test sets the three timer unuits, each with different interrupt priority level.
 * Each timer/counter is set to respond to three separate alarms.
 * The alarm times are set so each would trigger one after another separated by
 * the `ALARM_DIFF_US` time.
 * Each ISR sets its corresponding token into its token slots and wait for the
 * subsequent alarm ISR to be called.
 * After the last alarm ISR finishes the main loop checks if the ISR were called.
 */

#define TOKEN_A 0xBEEFBABE
#define TOKEN_B 0xC0FEB00B
#define TOKEN_C 0xDEEE5AA5

#define ALARM_DIFF_US 1000
#define ALARM_A_US (1000 + ALARM_DIFF_US)
#define ALARM_B_US (ALARM_A_US + ALARM_DIFF_US)
#define ALARM_C_US (ALARM_B_US + ALARM_DIFF_US)
#define ISR_DELAY_US (ALARM_DIFF_US * 3)
#define CYCLE_MS 10
#define TEST_CYCLES 100000

static const struct device *timer0 = DEVICE_DT_GET(DT_NODELABEL(timer0));
static const struct device *timer1 = DEVICE_DT_GET(DT_NODELABEL(timer1));
static const struct device *timer2 = DEVICE_DT_GET(DT_NODELABEL(timer2));

static struct counter_alarm_cfg alarm_a;
static struct counter_alarm_cfg alarm_b;
static struct counter_alarm_cfg alarm_c;

static int token[3];
static int success_cnt;
static int error_cnt;

static void alarm_handler_c(const struct device *dev, uint8_t chan_id,
			   uint32_t counter,
			   void *user_data)
{
	if (dev == timer2 &&
	    token[0] == TOKEN_A &&
	    token[1] == TOKEN_B &&
	    token[2] == 0) {
		token[2] = TOKEN_C;
	}

	k_busy_wait(ISR_DELAY_US);
}

static void alarm_handler_b(const struct device *dev, uint8_t chan_id,
			   uint32_t counter,
			   void *user_data)
{
	if (dev == timer1 &&
	    token[0] == TOKEN_A &&
	    token[1] == 0 &&
	    token[2] == 0) {
		token[1] = TOKEN_B;
	}

	k_busy_wait(ISR_DELAY_US);
}

static void alarm_handler_a(const struct device *dev, uint8_t chan_id,
			   uint32_t counter,
			   void *user_data)
{
	if (dev == timer0 &&
	    token[0] == 0 &&
	    token[1] == 0 &&
	    token[2] == 0) {
		token[0] = TOKEN_A;
	}

	k_busy_wait(ISR_DELAY_US);
}

static int test_setup(void)
{
	int err;

	printk("counter 0 channels %d\n", counter_get_num_of_channels(timer0));
	printk("counter 0 frequency %d\n", counter_get_frequency(timer0));
	printk("counter 0 top value %u\n", counter_get_top_value(timer0));

	printk("counter 1 channels %d\n", counter_get_num_of_channels(timer1));
	printk("counter 1 frequency %d\n", counter_get_frequency(timer1));
	printk("counter 1 top value %u\n", counter_get_top_value(timer1));

	printk("counter 2 channels %d\n", counter_get_num_of_channels(timer2));
	printk("counter 2 frequency %d\n", counter_get_frequency(timer2));
	printk("counter 2 top value %u\n", counter_get_top_value(timer2));

	if (!device_is_ready(timer0)) {
		printk("Error: device %s is not ready\n", timer0->name);
		return -1;
	}

	if (!device_is_ready(timer1)) {
		printk("Error: device %s is not ready\n", timer1->name);
		return -1;
	}

	if (!device_is_ready(timer2)) {
		printk("Error: device %s is not ready\n", timer2->name);
		return -1;
	}

	err = counter_start(timer0);
	if (err) {
	    printk("Failed to start counter 0: %d\n", err);
	    return -1;
	}

	err = counter_start(timer1);
	if (err) {
	    printk("Failed to start counter 1: %d\n", err);
	    return -1;
	}

	err = counter_start(timer2);
	if (err) {
	    printk("Failed to start counter 2: %d\n", err);
	    return -1;
	}

	return 0;
}

int main(void)
{
	int err;
	uint32_t cnt;

	if (test_setup()) {
		printk("Failed to setup the test!\n");
		return 1;
	}

	cnt = TEST_CYCLES;
	/* while (cnt--) { */
	while (1) {

		alarm_a.ticks = counter_us_to_ticks(timer0, ALARM_A_US);
		alarm_a.callback = alarm_handler_a;

		alarm_b.ticks = counter_us_to_ticks(timer1, ALARM_B_US);
		alarm_b.callback = alarm_handler_b;

		alarm_c.ticks = counter_us_to_ticks(timer2, ALARM_C_US);
		alarm_c.callback = alarm_handler_c;

		counter_reset(timer0);
		counter_reset(timer1);
		counter_reset(timer2);

		err = counter_set_channel_alarm(timer0, 0, &alarm_a);
		if (err) {
		    printk("Failed to set alarm A: %d\n", err);
		    return 1;
		}
		err = counter_set_channel_alarm(timer1, 0, &alarm_b);
		if (err) {
		    printk("Failed to set alarm B: %d\n", err);
		    return 1;
		}

		err = counter_set_channel_alarm(timer2, 0, &alarm_c);
		if (err) {
		    printk("Failed to set alarm C: %d\n", err);
		    return 1;
		}

		k_msleep(CYCLE_MS);

		if (token[0] == TOKEN_A &&
		    token[1] == TOKEN_B &&
		    token[2] == TOKEN_C) {
			printk(".");
			success_cnt ++;
		} else {
			printk("results A(%c) ", token[0] == TOKEN_A ? 'V' : 'x');
			printk("B(%c) ", token[1] == TOKEN_B ? 'V' : 'x');
			printk("C(%c)\n", token[2] == TOKEN_C ? 'V' : 'x');
			error_cnt ++;
		}

		if (error_cnt) {
			printk("\nerrors=%d / success=%d\n", error_cnt, success_cnt);
		}

		token[0] = 0;
		token[1] = 0;
		token[2] = 0;
	}

	return 0;
}
