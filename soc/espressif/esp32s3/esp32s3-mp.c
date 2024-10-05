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

#include "esp_mcuboot_image.h"
//#include "esp_image_loader.h"
#include "esp_memory_utils.h"


void smp_log(const char *msg)
{
	while (*msg) {
		esp_rom_uart_tx_one_char(*msg++);
	}
	esp_rom_uart_tx_one_char('\r');
	esp_rom_uart_tx_one_char('\n');
}

#ifdef CONFIG_SOC_ENABLE_APPCPU

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
	esp_cpu_reset(1);

//	ets_delay_us(20000);

//	esp_cpu_unstall(0);
	smp_log("CPU0: CPU1 start sequence complete");
}

static int load_segment(uint32_t src_addr, uint32_t src_len, uint32_t dst_addr)
{
	const uint32_t *data = (const uint32_t *)sys_mmap(src_addr, src_len);

	if (!data) {
		ets_printf("%s: mmap failed", __func__);
		return -1;
	}

	volatile uint32_t *dst = (volatile uint32_t *)dst_addr;

	for (int i = 0; i < src_len / 4; i++) {
		dst[i] = data[i];
	}

	sys_munmap(data);

	smp_log("ESP32S3: segment loaded"); //CPU1 start sequence complete");
	return 0;
}

#include "hexdump.h"

int esp_appcpu_image_load(unsigned int hdr_offset, unsigned int *entry_addr)
{
	const uint32_t img_off = FIXED_PARTITION_OFFSET(slot0_appcpu_partition);
	const uint32_t fa_size = FIXED_PARTITION_SIZE(slot0_appcpu_partition);
	const uint8_t fa_id = FIXED_PARTITION_ID(slot0_appcpu_partition);
	int rc = 0;

	if (entry_addr == NULL) {
		ets_printf("cant return the entry address. Aborting!\n");
		abort();
		return -1;
	}

	ets_printf("Loading appcpu image from flash, area id: %d, offset: 0x%x, hdr.offset: 0x%x, size: %d kB\n",
	fa_id, img_off, hdr_offset, fa_size/1024);

	uint32_t mcuboot_header[8] = {0};
	esp_image_load_header_t img_header = {0};

	const uint32_t *data = (const uint32_t *)sys_mmap(img_off, 0x40);  // sizeof(esp_image_load_header_t));

	memcpy((void *)&mcuboot_header, data, sizeof(mcuboot_header));
	memcpy((void *)&img_header, data + (hdr_offset/sizeof(uint32_t)), sizeof(esp_image_load_header_t));

	sys_munmap(data);

	hexdump("APPCPU MCUboot hdr:", &mcuboot_header, sizeof(mcuboot_header));
	hexdump("APPCPU ESPmeta hdr:", &img_header, sizeof(esp_image_load_header_t));

	if (img_header.header_magic == ESP_LOAD_HEADER_MAGIC) {
		ets_printf("MCUboot image format - header magic found\n");
	} else if ((img_header.header_magic & 0xff) == 0xE9) {
		ets_printf("ESP image format - header magic found\n");
		ets_printf("but it is not supported yet\n");
		abort();
	} else {
		ets_printf("Unknown or empty image detected. Aborting!\n");
		abort();
	}

	if (!esp_ptr_in_iram((void *)img_header.iram_dest_addr) ||
	    !esp_ptr_in_iram((void *)(img_header.iram_dest_addr + img_header.iram_size))) {
	    ets_printf("IRAM region in load header is not valid. Aborting");
	    abort();
	}

	if (!esp_ptr_in_dram((void *)img_header.dram_dest_addr) ||
	    !esp_ptr_in_dram((void *)(img_header.dram_dest_addr + img_header.dram_size))) {
	    ets_printf("DRAM region in load header is not valid. Aborting");
	    abort();
	}

	if (!esp_ptr_in_iram((void *)img_header.entry_addr)) {
	    ets_printf("Application entry point (%xh) is not in IRAM. Aborting",
	    img_header.entry_addr);
	    abort();
	}

	ets_printf("Application start=%xh\n", img_header.entry_addr);
	ets_printf("iram segment: paddr=%08xh, vaddr=%08xh, size=%05xh (%6d) load\n",
	(img_off + img_header.iram_flash_offset), img_header.iram_dest_addr,
	img_header.iram_size, img_header.iram_size);

	load_segment(img_off + img_header.iram_flash_offset, img_header.iram_size, img_header.iram_dest_addr);

	ets_printf("dram segment: paddr=%08xh, vaddr=%08xh, size=%05xh (%6d) load\n",
	(img_off + img_header.dram_flash_offset), img_header.dram_dest_addr,
	img_header.dram_size, img_header.dram_size);

	load_segment(img_off + img_header.dram_flash_offset, img_header.dram_size, img_header.dram_dest_addr);

	uart_tx_wait_idle(0);

	assert(entry_addr != NULL);
	*entry_addr = img_header.entry_addr;

	return rc;
}

void esp_appcpu_image_start(unsigned int hdr_offset)
{
	static int started = 0;
	unsigned int entry_addr = 0;

	if (started) {
		printk("APPCPU allready started.\n");
		return;
	}

	/* Input image meta header, output appcpu entry point */
	esp_appcpu_image_load(hdr_offset, &entry_addr);

	ets_printf("Starting APPCPU with entry address 0x%x\n", entry_addr);
	esp_appcpu_start((void *)entry_addr);
	esp_cpu_unstall(0);
}

int esp_start_appcpu(void)
{

	esp_appcpu_image_start(0x20);
	return 0;
}
#ifndef CONFIG_MCUBOOT
SYS_INIT(esp_start_appcpu, APPLICATION, 99);
#endif
#endif /* CONFIG_SOC_ENABLE_APPCPU */
