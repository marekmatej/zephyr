/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>

#include <soc/rtc.h>
#include <soc.h>
#include <esp_cpu.h>
#include <rom/rtc.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#define NVS_PARTITION			storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define TEST_STATE_ID	1
#define TEST_RESULT_ID  2

enum test_state {
	RST_INIT,
	RST_SYSTEM,
	RST_CPU,
	RST_WDT,
	RST_BROWNOUT,
	RST_EXTERNAL,

	RST_NUM		/* count label only */
};

enum test_result {
	TST_FAIL,
	TST_PASS,
	TST_SKIP
};

static const char *const resultsStr[] = {"FAIL", "PASS", "SKIP"};

static struct nvs_fs fs;
static uint8_t testResult[RST_NUM];

#ifndef WDT_MAX_WINDOW
#define WDT_MAX_WINDOW  1000U
#endif

#ifndef WDT_MIN_WINDOW
#define WDT_MIN_WINDOW  0U
#endif

#ifndef WDG_FEED_INTERVAL
#define WDG_FEED_INTERVAL 50U
#endif

#define WDT_FEED_TRIES 5

#ifndef WDT_OPT
#define WDT_OPT WDT_OPT_PAUSE_HALTED_BY_DBG
#endif

void flash_init(void);
int wdt_install(void);

/**
 * @brief Test delay during boot
 * @defgroup kernel_init_tests Init
 * @ingroup all_tests
 * @{
 */

/**
 * @brief This module verifies the delay specified during boot.
 */
ZTEST(zz_reset, test_reset)
{
	int rc = 0;
	int count = 0;
	int testState;
	int rstReason = rtc_get_reset_reason(0);

	flash_init();

	rc = nvs_read(&fs, TEST_STATE_ID, &testState, sizeof(testState));

	if (rc > 0) { /* item was found, show it */
		printk("Id: %d, Test state: %d\n",
			TEST_STATE_ID, testState);
	} else   {/* item was not found, add it */
		testState = RST_INIT;
		printk("No Reboot counter found, adding it at id %d\n",
		       TEST_STATE_ID);
		(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  sizeof(testState));
	}

	rc = nvs_read(&fs, TEST_RESULT_ID, &testResult, sizeof(testResult));

	if (rc == 0) {
		for (int i=0; i<RST_NUM; i++)
		{
			testResult[i] = TST_FAIL;	/* init */
		}
		/* item was not found, add it */
		(void)nvs_write(&fs, TEST_RESULT_ID, &testResult,
			  sizeof(testResult));
	}

	printk("\nWe are coming from a reset type %d\n", rstReason);

	switch (testState)
	{
		case RST_INIT:

			/* save next state on NVS */
			testState = RST_SYSTEM;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  sizeof(testState));

			printk("\nIssuing a CPU reset...\n");
			printk("===================================================================\n");
			k_sleep(K_MSEC(100));
			software_reset_cpu(0);

		break;

		case RST_CPU:

			/* save next state on NVS */
			testState = RST_SYSTEM;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  sizeof(testState));

			/* store test result */
			if (rstReason == SW_CPU_RESET)
			{
				testResult[RST_CPU] = TST_PASS;
				(void)nvs_write(&fs, TEST_RESULT_ID, &testResult,
			  		sizeof(testResult));
			}

			printk("\nIssuing a system reset...\n");
			printk("===================================================================\n");
			k_sleep(K_MSEC(100));
			software_reset();

		break;

		case RST_SYSTEM:

			/* save next state on NVS */
			testState = RST_WDT;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  sizeof(testState));

			/* store test result */
			if (rstReason == SW_RESET)
			{
				testResult[RST_SYSTEM] = TST_PASS;
				(void)nvs_write(&fs, TEST_RESULT_ID, &testResult,
			  		sizeof(testResult));
			}

			wdt_install();
			printk("\nWaiting for WDT reset...\n");
			printk("===================================================================\n");

			/* Provoke WDT reset */
			while(1) {
				k_yield();
			}

		break;

		case RST_WDT:
			
			/* save next state on NVS */
			testState = RST_BROWNOUT;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  sizeof(testState));

			/* store test result */
			if (rstReason == TG0WDT_SYS_RESET)
			{
				testResult[RST_WDT] = TST_PASS;
				(void)nvs_write(&fs, TEST_RESULT_ID, &testResult,
			  		sizeof(testResult));
			}

			printk("\nPlease decrease VCC voltage below the appropriate threshold for a brown out reset...");

			count = 30;

			while(count > 0)
			{
				printk(".");
				k_sleep(K_SECONDS(1));
				count--;
			}

			printk(" skipped\n");
			
			testResult[RST_BROWNOUT] = TST_SKIP;
			(void)nvs_write(&fs, TEST_RESULT_ID, &testResult,
			  		sizeof(testResult));

			/* save next state on NVS */
			testState = RST_EXTERNAL;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  sizeof(testState));

			printk("\nPlease press button EN on board to simulate external reset (Power on reset)...");

			count = 30;

			while(count > 0)
			{
				printk(".");
				k_sleep(K_SECONDS(1));
				count--;
			}

			printk(" skipped\n");
			
			testResult[RST_EXTERNAL] = TST_SKIP;
			(void)nvs_write(&fs, TEST_RESULT_ID, &testResult,
			  		sizeof(testResult));

			/* save next state on NVS */
			testState = RST_NUM;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  sizeof(testState));

			printk("\nIssuing a system reset...\n");
			printk("===================================================================\n");
			k_sleep(K_MSEC(100));
			software_reset();

			printk("===================================================================\n");

			while(1) {
				k_yield();
			}

		break;

		case RST_BROWNOUT:
			
			/* save next state on NVS */
			testState = RST_EXTERNAL;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  sizeof(testState));

			/* store test result */
			if (rstReason == RTCWDT_BROWN_OUT_RESET)
			{
				testResult[RST_BROWNOUT] = TST_PASS;
				(void)nvs_write(&fs, TEST_RESULT_ID, &testResult,
			  		sizeof(testResult));
			}

			printk("\nPlease press button EN on board to simulate external reset (hard reset)...\n");

			count = 30;

			while(count > 0)
			{
				printk(".");
				k_sleep(K_SECONDS(1));
				count--;
			}

			printk(" skipped\n");

			printk("\nIssuing a system reset...\n");
			printk("===================================================================\n");
			k_sleep(K_MSEC(100));
			software_reset();

			printk("===================================================================\n");

			while(1) {
				k_yield();
			}

		break;

		case RST_EXTERNAL:
			
			/* store test result */
			if (rstReason == POWERON_RESET)
			{
				testResult[RST_EXTERNAL] = TST_PASS;
				(void)nvs_write(&fs, TEST_RESULT_ID, &testResult,
			  		sizeof(testResult));
			}

		__fallthrough;		/* last test, show results */

		case RST_NUM:
			
			printk("\n");
			printk("\nTest results summary:\n");
			printk("\n");
			printk("\nCPU reset: %s", resultsStr[testResult[RST_CPU]]);
			printk("\nSystem reset: %s", resultsStr[testResult[RST_SYSTEM]]);
			printk("\nWatchdog reset: %s", resultsStr[testResult[RST_WDT]]);
			printk("\nBrownout reset: %s", resultsStr[testResult[RST_BROWNOUT]]);
			printk("\nExternal (hard) reset: %s\n\n", resultsStr[testResult[RST_EXTERNAL]]);

			int result = (testResult[RST_CPU] && testResult[RST_SYSTEM] &&
						  testResult[RST_WDT] && testResult[RST_BROWNOUT] &&
						  testResult[RST_EXTERNAL]);

			testState = RST_INIT;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  	sizeof(testState));

			zassert_true(result,
				"At least one reset test failed (see summary above)!");

		break;

		default:

			/* should not happen */
			testState = RST_INIT;
			(void)nvs_write(&fs, TEST_STATE_ID, &testState,
			  	sizeof(testState));

		break;
	}
}

void flash_init(void)
{
	struct flash_pages_info info;
	int rc = 0;

	/* define the nvs file system by settings with:
	 *	sector_size equal to the pagesize,
	 *	3 sectors
	 *	starting at NVS_PARTITION_OFFSET
	 */
	fs.flash_device = NVS_PARTITION_DEVICE;
	
	if (!device_is_ready(fs.flash_device)) {
		printk("Flash device %s is not ready\n", fs.flash_device->name);
		return;
	}
	
	fs.offset = NVS_PARTITION_OFFSET;
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	
	if (rc) {
		printk("Unable to get page info\n");
		return;
	}
	
	fs.sector_size = info.size;
	fs.sector_count = 3U;

	rc = nvs_mount(&fs);
	
	if (rc) {
		printk("Flash Init failed\n");
		return;
	}
}

int wdt_install(void)
{
	int err;
	int wdt_channel_id;
	const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));

	if (!device_is_ready(wdt)) {
		printk("%s: device not ready.\n", wdt->name);
		return 0;
	}

	struct wdt_timeout_cfg wdt_config = {
		/* Reset SoC when watchdog timer expires. */
		.flags = WDT_FLAG_RESET_SOC,

		/* Expire watchdog after max window */
		.window.min = WDT_MIN_WINDOW,
		.window.max = WDT_MAX_WINDOW,
	};

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);

	if (wdt_channel_id == -ENOTSUP) {
		/* IWDG driver for STM32 doesn't support callback */
		printk("Callback support rejected, continuing anyway\n");
		wdt_config.callback = NULL;
		wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	}
	if (wdt_channel_id < 0) {
		printk("Watchdog install error\n");
		return 0;
	}

	err = wdt_setup(wdt, WDT_OPT);
	if (err < 0) {
		printk("Watchdog setup error\n");
		return 0;
	}

#if WDT_MIN_WINDOW != 0
	/* Wait opening window. */
	k_msleep(WDT_MIN_WINDOW);
#endif
	/* Feeding watchdog. */
	printk("Feeding watchdog %d times\n", WDT_FEED_TRIES);
	for (int i = 0; i < WDT_FEED_TRIES; ++i) {
		printk("Feeding watchdog...\n");
		wdt_feed(wdt, wdt_channel_id);
		k_sleep(K_MSEC(WDG_FEED_INTERVAL));
	}

	return 0;
}

extern void *common_setup(void);
ZTEST_SUITE(zz_reset, NULL, common_setup, NULL, NULL, NULL);
