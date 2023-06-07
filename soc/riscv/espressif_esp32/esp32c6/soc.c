/*
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include esp-idf headers first to avoid redefining BIT() macro */
#include <soc/timer_group_reg.h>
#include <soc/ext_mem_defs.h>
#include <soc/gpio_reg.h>
#include <soc/system_reg.h>
#include "hal/wdt_hal.h"
#include "esp_cpu.h"
#include "hal/soc_hal.h"
#include "hal/cpu_hal.h"
#include "esp_timer.h"
#include "esp_private/system_internal.h"
#include "esp_clk_internal.h"
#include <soc/interrupt_reg.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32c3.h>

#include <zephyr/kernel_structs.h>
#include <kernel_internal.h>
#include <string.h>
#include <zephyr/toolchain/gcc.h>
#include <soc.h>

#ifdef CONFIG_MCUBOOT
#include "bootloader_init.h"
#endif /* CONFIG_MCUBOOT */

/*
 * This is written in C rather than assembly since, during the port bring up,
 * Zephyr is being booted by the Espressif bootloader.  With it, the C stack
 * is already set up.
 */
void IRAM_ATTR __esp_platform_start(void)
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

	__asm__ __volatile__("la t0, _esp32c6_vector_table\n"
						"csrw mtvec, t0\n");

	z_bss_zero();

	/* Disable normal interrupts. */
	csr_read_clear(mstatus, MSTATUS_MIE);

#ifdef CONFIG_MCUBOOT
	/* MCUboot early initialisation.
	 */
	bootloader_init();

#else
	/* ESP-IDF 2nd stage bootloader enables RTC WDT to check on startup sequence
	 * related issues in application. Hence disable that as we are about to start
	 * Zephyr environment.
	 */
	wdt_hal_context_t rtc_wdt_ctx = {.inst = WDT_RWDT, .rwdt_dev = &LP_WDT};

	wdt_hal_write_protect_disable(&rtc_wdt_ctx);
	wdt_hal_disable(&rtc_wdt_ctx);
	wdt_hal_write_protect_enable(&rtc_wdt_ctx);

	/* Configure the Cache MMU size for instruction and rodata in flash. */
	extern uint32_t Cache_Set_IDROM_MMU_Size(uint32_t irom_size, uint32_t drom_size);

	extern int _rodata_reserved_start;
	uint32_t rodata_reserved_start_align =
		(uint32_t)&_rodata_reserved_start & ~(CONFIG_MMU_PAGE_SIZE - 1);
	uint32_t cache_mmu_irom_size =
		((rodata_reserved_start_align - SOC_DROM_LOW) / CONFIG_MMU_PAGE_SIZE) *
		sizeof(uint32_t);

	Cache_Set_IDROM_MMU_Size(cache_mmu_irom_size,
		CACHE_DROM_MMU_MAX_END - cache_mmu_irom_size);

	/* Configures the CPU clock, RTC slow and fast clocks, and performs
	 * RTC slow clock calibration.
	 */
	esp_clk_init();

	esp_timer_early_init();

#if CONFIG_SOC_FLASH_ESP32
	spi_flash_guard_set(&g_flash_guard_default_ops);
#endif

#endif /* CONFIG_MCUBOOT */

	/*Initialize the esp32c6 interrupt controller */
	esp_intr_initialize();

	/* Start Zephyr */
	z_cstart();

	CODE_UNREACHABLE;
}

/* Boot-time static default printk handler, possibly to be overridden later. */
int IRAM_ATTR arch_printk_char_out(int c)
{
	if (c == '\n') {
		esp_rom_uart_tx_one_char('\r');
	}
	esp_rom_uart_tx_one_char(c);
	return 0;
}

void sys_arch_reboot(int type)
{
	esp_restart_noos();
}
