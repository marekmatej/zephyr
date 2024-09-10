/*
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>

#include <soc.h>
#include <esp_cpu.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

//#include "esp32s3/rom/uart.h"
//#include "esp_mcuboot_image.h"
//#include "esp_loader.h"
//#include "esp_memory_utils.h"
//#include "bootloader_flash_priv.h"

void smp_log(const char *msg)
{
	while (*msg) {
		esp_rom_uart_tx_one_char(*msg++);
	}
	esp_rom_uart_tx_one_char('\r');
	esp_rom_uart_tx_one_char('\n');
}

#if 0 //def CONFIG_SOC_ENABLE_APPCPU
void esp_appcpu_start(void *entry_point)
{
	esp_cpu_unstall(1);

	if (!REG_GET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN)) {
		REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN);
		REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RUNSTALL);
		REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETTING);
		REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETTING);
	}

	esp_rom_ets_set_appcpu_boot_addr((void *)entry_point);

	ets_delay_us(50000);

	smp_log("ESP32S3: CPU1 start sequence complete");
}


static int load_segment(const struct flash_area *fap, uint32_t data_addr,
                        uint32_t data_len, uint32_t load_addr)
{
	const uint32_t *data = (const uint32_t *)bootloader_mmap((fap->fa_off + data_addr), data_len);
	if (!data) {
		ets_printf("%s: Bootloader mmap failed", __func__);
		return -1;
	}
	memcpy((void *)load_addr, data, data_len);
	bootloader_munmap(data);
	return 0;
}

#include "hexdump.h"

void esp_appcpu_image_load(unsigned int hdr_offset, unsigned int *entry_addr)
{
	const struct flash_area *fap;
	uint8_t fa_id = FIXED_PARTITION_ID(slot0_appcpu_partition);
	int esp_image_format;
	int rc;

	if (entry_addr == NULL) {
		ets_printf("cant return the entry address. Aborting!\n");
		abort();
		return;
	}

	rc = flash_area_open(fa_id, &fap);
	if (rc) {
	    ets_printf("%s: flash_area_open failed with %d\n", __func__, rc);
	    abort();
	}

//	ets_printf("Loading image %d - slot %d from flash, area id: %d\n",
//	img_index, slot, fa_id);

	const uint32_t *data = (const uint32_t *)bootloader_mmap((fap->fa_off + hdr_offset),
				sizeof(esp_image_load_header_t));
	esp_image_load_header_t load_header = {0};
	memcpy((void *)&load_header, data, sizeof(esp_image_load_header_t));
	bootloader_munmap(data);

	hexdump("hdr", &load_header, 0x20 /*sizeof(esp_image_load_header_t)*/);

	if (load_header.header_magic == ESP_LOAD_HEADER_MAGIC) {
		esp_image_format = 1;
	} else if (load_header.header_magic & 0xff == 0xE9) {
		esp_image_format = 0;
	}

	if (esp_image_format) {
		ets_printf("ESP image format - header magic found\n");
	} else {
		ets_printf("MCUboot image format - header magic found\n");
#if 0
		if (!esp_ptr_in_iram((void *)load_header.iram_dest_addr) ||
		    !esp_ptr_in_iram((void *)(load_header.iram_dest_addr + load_header.iram_size))) {
		    ets_printf("IRAM region in load header is not valid. Aborting");
		    abort();
		}

		if (!esp_ptr_in_dram((void *)load_header.dram_dest_addr) ||
		    !esp_ptr_in_dram((void *)(load_header.dram_dest_addr + load_header.dram_size))) {
		    ets_printf("DRAM region in load header is not valid. Aborting");
		    abort();
		}

		if (!esp_ptr_in_iram((void *)load_header.entry_addr)) {
		    ets_printf("Application entry point (%xh) is not in IRAM. Aborting",
		    load_header.entry_addr);
		    abort();
		}
#endif
	}

#if 0
	ets_printf("Application start=%xh\n", load_header.entry_addr);
	ets_printf("DRAM segment: paddr=%08xh, vaddr=%08xh, size=%05xh (%6d) load\n",
	(fap->fa_off + load_header.dram_flash_offset), load_header.dram_dest_addr,
	load_header.dram_size, load_header.dram_size);
	load_segment(fap, load_header.dram_flash_offset,
	load_header.dram_size, load_header.dram_dest_addr);

	ets_printf("IRAM segment: paddr=%08xh, vaddr=%08xh, size=%05xh (%6d) load\n",
	(fap->fa_off + load_header.iram_flash_offset), load_header.iram_dest_addr,
	load_header.iram_size, load_header.iram_size);
	load_segment(fap, load_header.iram_flash_offset,
	load_header.iram_size, load_header.iram_dest_addr);

	uart_tx_wait_idle(0);

	assert(entry_addr != NULL);
	*entry_addr = load_header.entry_addr;
#endif
}

void esp_appcpu_image_start(unsigned int hdr_offset)
{
	static int started = 0;
	unsigned int entry_addr;

	if (started) {
		printk("APPCPU allready started.\n");
		return;
	}

	esp_appcpu_image_load(hdr_offset, &entry_addr);

//	esp_appcpu_start((void *)entry_addr);
}

static int start_appcpu(void)
{
	esp_appcpu_image_start(0);
	return 0;
}

//SYS_INIT(start_appcpu, APPLICATION, 99);
#endif
