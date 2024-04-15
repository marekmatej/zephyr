/*
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

/* LP-SRAM (16kB) memory */
#define SRAM0_IRAM_START   0x50000000
#define SRAM0_SIZE         0x4000
/* HP-SRAM (512kB) memory */
#define SRAM1_DRAM_START   0x40800000
#define SRAM1_IRAM_START   0x40800000
/* ICache size is fixed to 32KB on ESP32-C6 */
#define ICACHE_SIZE        0x8000

/** Simplified memory map for the bootloader.
 *  Make sure the bootloader can load into main memory without overwriting itself.
 *
 *  ESP32-C6 ROM static data usage is as follows:
 *  - 0x4086ad08 - 0x4087c610: Shared buffers, used in UART/USB/SPI download mode only
 *  - 0x4087c610 - 0x4087e610: PRO CPU stack, can be reclaimed as heap after RTOS startup
 *  - 0x4087e610 - 0x40880000: ROM .bss and .data (not easily reclaimable)
 *
 *  The 2nd stage bootloader can take space up to the end of ROM shared
 *  buffers area (0x4087c610).
 */

#define DRAM_BUFFERS_START       0x4086ad08
#define DRAM_STACK_START         0x4087c610
#define DRAM_ROM_BSS_DATA_START  0x4087e610

/* Base address used for calculating memory layout
 * counted from Dbus backwards and back to the Ibus
 */
#define BOOTLOADER_USABLE_DRAM_END DRAM_BUFFERS_START

/* For safety margin between bootloader data section and startup stacks */
#define BOOTLOADER_STACK_OVERHEAD      0x0
/* These lengths can be adjusted, if necessary: */
#define BOOTLOADER_DRAM_SEG_LEN        0x9000
#define BOOTLOADER_IRAM_LOADER_SEG_LEN 0x3000
#define BOOTLOADER_IRAM_SEG_LEN        0x8000

/* Start of the lower region is determined by region size and the end of the higher region */
#define BOOTLOADER_DRAM_SEG_END   (BOOTLOADER_USABLE_DRAM_END + BOOTLOADER_STACK_OVERHEAD)
#define BOOTLOADER_DRAM_SEG_START (BOOTLOADER_DRAM_SEG_END - BOOTLOADER_DRAM_SEG_LEN)
#define BOOTLOADER_IRAM_LOADER_SEG_START (BOOTLOADER_DRAM_SEG_START - \
					BOOTLOADER_IRAM_LOADER_SEG_LEN)
#define BOOTLOADER_IRAM_SEG_START (BOOTLOADER_IRAM_LOADER_SEG_START - BOOTLOADER_IRAM_SEG_LEN)

/* Flash */
#ifdef CONFIG_FLASH_SIZE
#define FLASH_SIZE         CONFIG_FLASH_SIZE
#else
#define FLASH_SIZE         0x400000
#endif

/* Cached memory */
#define CACHE_ALIGN        CONFIG_MMU_PAGE_SIZE
#define IROM_SEG_ORG       0x42000000
#define IROM_SEG_LEN       FLASH_SIZE
#define DROM_SEG_ORG       0x42000000
#define DROM_SEG_LEN       FLASH_SIZE
