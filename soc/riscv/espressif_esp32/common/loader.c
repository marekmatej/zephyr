/*
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <hal/mmu_hal.h>
#include <hal/mmu_types.h>
#include <hal/cache_types.h>
#include <hal/cache_ll.h>
#include <hal/cache_hal.h>
#include <rom/cache.h>
#include <esp_rom_sys.h>

#define MMU_FLASH_MASK    (~(CONFIG_MMU_PAGE_SIZE - 1))

#include <esp_err.h>
#include <esp_app_format.h>
#include <zephyr/storage/flash_map.h>
#include "esp_rom_uart.h"
#include "esp_flash.h"
#include "bootloader_init.h"

#define CHECKSUM_ALIGN 16
#define IS_PADD(addr) (addr == 0)
#define IS_DRAM(addr) (addr >= SOC_DRAM_LOW && addr < SOC_DRAM_HIGH)
#define IS_IRAM(addr) (addr >= SOC_IRAM_LOW && addr < SOC_IRAM_HIGH)
#define IS_IROM(addr) (addr >= SOC_IROM_LOW && addr < SOC_IROM_HIGH)
#define IS_DROM(addr) (addr >= SOC_DROM_LOW && addr < SOC_DROM_HIGH)
#define IS_SRAM(addr) (IS_IRAM(addr) || IS_DRAM(addr))
#define IS_MMAP(addr) (IS_IROM(addr) || IS_DROM(addr))
#define IS_NONE(addr) (!IS_IROM(addr) && !IS_DROM(addr) \
			&& !IS_IRAM(addr) && !IS_DRAM(addr) && !IS_PADD(addr))

#define BOOT_LOG_INF(_fmt, ...) \
	ets_printf("[" CONFIG_SOC_SERIES "] [INF] " _fmt "\n\r", ##__VA_ARGS__)
#define BOOT_LOG_ERR(_fmt, ...) \
	ets_printf("[" CONFIG_SOC_SERIES "] [ERR] " _fmt "\n\r", ##__VA_ARGS__)

#define HDR_ATTR __attribute__((section(".entry_addr"))) __attribute__((used))

void __start(void);
static HDR_ATTR void (*_entry_point)(void) = &__start;

extern esp_image_header_t bootloader_image_hdr;
extern uint32_t _image_irom_start, _image_irom_size, _image_irom_vaddr;
extern uint32_t _image_drom_start, _image_drom_size, _image_drom_vaddr;

static uint32_t _app_irom_start = (FIXED_PARTITION_OFFSET(slot0_partition) +
				(uint32_t)&_image_irom_start);
static uint32_t _app_irom_size = (uint32_t)&_image_irom_size;
static uint32_t _app_irom_vaddr = ((uint32_t)&_image_irom_vaddr);

static uint32_t _app_drom_start = (FIXED_PARTITION_OFFSET(slot0_partition) +
				(uint32_t)&_image_drom_start);
static uint32_t _app_drom_size = (uint32_t)&_image_drom_size;
static uint32_t _app_drom_vaddr = ((uint32_t)&_image_drom_vaddr);

#ifdef CONFIG_ESP_SIMPLE_BOOT
static esp_err_t spi_flash_read(uint32_t address, void *buffer, size_t length)
{
	return esp_flash_read(NULL, buffer, address, length);
}
#endif /* CONFIG_ESP_SIMPLE_BOOT */

void map_rom_segments(uint32_t app_drom_start, uint32_t app_drom_vaddr,
			uint32_t app_drom_size, uint32_t app_irom_start,
			uint32_t app_irom_vaddr, uint32_t app_irom_size)
{
	uint32_t app_irom_start_aligned = app_irom_start & MMU_FLASH_MASK;
	uint32_t app_irom_vaddr_aligned = app_irom_vaddr & MMU_FLASH_MASK;

	uint32_t app_drom_start_aligned = app_drom_start & MMU_FLASH_MASK;
	uint32_t app_drom_vaddr_aligned = app_drom_vaddr & MMU_FLASH_MASK;
	uint32_t actual_mapped_len = 0;

#ifdef CONFIG_ESP_SIMPLE_BOOT
	esp_image_segment_header_t WORD_ALIGNED_ATTR segment_hdr;
	size_t offset = FIXED_PARTITION_OFFSET(boot_partition);
	bool padding_checksum = false;
	unsigned int segments = 0;
	unsigned int ram_segments = 0;

	/* Using already fetched bootloader image header from bootloader_init */
	offset += sizeof(esp_image_header_t);

	while (segments++ < 16) {

		if (spi_flash_read(offset, &segment_hdr,
				sizeof(esp_image_segment_header_t)) != ESP_OK) {
			BOOT_LOG_ERR("Failed to read segment header at %x", offset);
			abort();
		}

		/* TODO: Find better end-of-segment detection */
		if (IS_NONE(segment_hdr.load_addr)) {
			/* Total segment count = (segments - 1) */
			break;
		}

		BOOT_LOG_INF("%s: lma 0x%08x vma 0x%08x len 0x%-6x (%u)",
			IS_NONE(segment_hdr.load_addr) ? "???" :
			 IS_MMAP(segment_hdr.load_addr) ?
			  IS_IROM(segment_hdr.load_addr) ? "IMAP" : "DMAP" :
			   IS_PADD(segment_hdr.load_addr) ? "padd" :
			    IS_DRAM(segment_hdr.load_addr) ? "DRAM" : "IRAM",
			offset + sizeof(esp_image_segment_header_t),
			segment_hdr.load_addr, segment_hdr.data_len, segment_hdr.data_len);

		/* Fix drom and irom produced be the linker, as this
		 * is later invalidated by the elf2image command
		 */
		if (IS_DROM(segment_hdr.load_addr)) {
			app_drom_start = offset + sizeof(esp_image_segment_header_t);
			app_drom_start_aligned = app_drom_start & MMU_FLASH_MASK;
		}
		if (IS_IROM(segment_hdr.load_addr)) {
			app_irom_start = offset + sizeof(esp_image_segment_header_t);
			app_irom_start_aligned = app_irom_start & MMU_FLASH_MASK;
		}
		if (IS_SRAM(segment_hdr.load_addr)) {
			ram_segments++;
		}
		offset += sizeof(esp_image_segment_header_t) + segment_hdr.data_len;

		if (ram_segments == bootloader_image_hdr.segment_count && !padding_checksum) {
			offset += (CHECKSUM_ALIGN - 1) - (offset % CHECKSUM_ALIGN) + 1;
			padding_checksum = true;
		}
	}
	if (segments == 0 || segments == 16) {
		BOOT_LOG_ERR("Error parsing segments");
		abort();
	}

	BOOT_LOG_INF("Image with %d segments", segments - 1);
#endif /* CONFIG_ESP_SIMPLE_BOOT */

	cache_hal_disable(CACHE_TYPE_ALL);

	/* Clear the MMU entries that are already set up,
	 * so the new app only has the mappings it creates.
	 */
	mmu_hal_unmap_all();

	mmu_hal_map_region(0, MMU_TARGET_FLASH0,
			app_drom_vaddr & MMU_FLASH_MASK,
			app_drom_start & MMU_FLASH_MASK,
			app_drom_size, &actual_mapped_len);

	mmu_hal_map_region(0, MMU_TARGET_FLASH0,
			app_irom_vaddr & MMU_FLASH_MASK,
			app_irom_start & MMU_FLASH_MASK,
			app_irom_size, &actual_mapped_len);

	/* ----------------------Enable corresponding buses---------------- */
	cache_bus_mask_t bus_mask = cache_ll_l1_get_bus(0,
					app_drom_vaddr & MMU_FLASH_MASK, app_drom_size);

	cache_ll_l1_enable_bus(0, bus_mask);
	bus_mask = cache_ll_l1_get_bus(0, app_irom_vaddr & MMU_FLASH_MASK, app_irom_size);
	cache_ll_l1_enable_bus(0, bus_mask);
#if CONFIG_MP_MAX_NUM_CPUS > 1
	bus_mask = cache_ll_l1_get_bus(1, app_drom_vaddr & MMU_FLASH_MASK, app_drom_size);
	cache_ll_l1_enable_bus(1, bus_mask);
	bus_mask = cache_ll_l1_get_bus(1, app_irom_vaddr & MMU_FLASH_MASK, app_irom_size);
	cache_ll_l1_enable_bus(1, bus_mask);
#endif

	/* ----------------------Enable Cache---------------- */
	cache_hal_enable(CACHE_TYPE_ALL);

	/* Show map segments continue using same log format as during MCUboot phase */
	BOOT_LOG_INF("DROM segment: paddr=%08Xh, vaddr=%08Xh, size=%05Xh (%6d) map",
		app_drom_start & MMU_FLASH_MASK, app_drom_vaddr & MMU_FLASH_MASK,
		app_drom_size, app_drom_size);
	BOOT_LOG_INF("IROM segment: paddr=%08Xh, vaddr=%08Xh, size=%05Xh (%6d) map\r\n",
		app_irom_start & MMU_FLASH_MASK, app_irom_vaddr & MMU_FLASH_MASK,
		app_irom_size, app_irom_size);
	esp_rom_uart_tx_wait_idle(0);
}

void __start(void)
{
#ifdef CONFIG_RISCV_GP
	/* Configure the global pointer register
	 * (This should be the first thing startup does, as any other piece of code could be
	 * relaxed by the linker to access something relative to __global_pointer$)
	 */
	__asm__ __volatile__(".option push\n"
				".option norelax\n"
				"la gp, __global_pointer$\n"
				".option pop");
#endif /* CONFIG_RISCV_GP */

#ifdef CONFIG_ESP_SIMPLE_BOOT
	/* Init fundamental components */
	if (bootloader_init()) {
		BOOT_LOG_INF("HW init failed, aborting\n");
		abort();
	}
#endif /* CONFIG_ESP_SIMPLE_BOOT */

#ifndef CONFIG_MCUBOOT
	map_rom_segments(_app_drom_start, _app_drom_vaddr, _app_drom_size,
			_app_irom_start, _app_irom_vaddr, _app_irom_size);
#endif /* !CONFIG_MCUBOOT */

	__esp_platform_start();
}
