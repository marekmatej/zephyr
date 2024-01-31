/*
 * Copyright (c) 2016 Cadence Design Systems, Inc.
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/linker/sections.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>

#define IRAM_START     0x40020000
#define DRAM_START     0x3ffb0000
/* DRAM_END is where data for ROM code SRAM usage
 * starts. It should not be overlapped by our code */
#define DRAM_END       0x3ffeab00

#define I_D_RAM_OFFSET (IRAM_START - DRAM_START)

#define SRAM_CACHE_SIZE (CONFIG_ESP32S2_INSTRUCTION_CACHE_SIZE \
                       + CONFIG_ESP32S2_DATA_CACHE_SIZE)
#define IRAM_SEG_ORG   (IRAM_START + SRAM_CACHE_SIZE)
#define DRAM_SEG_ORG   (DRAM_START + SRAM_CACHE_SIZE)

/* Total available space for IRAM ^ DRAM */
#define I_D_RAM_SIZE   (DRAM_END - DRAM_SEG_ORG)

#ifdef CONFIG_FLASH_SIZE
#define FLASH_SIZE     CONFIG_FLASH_SIZE
#else
#define FLASH_SIZE     0x400000
#endif /* CONFIG_FLASH_SIZE */

#define DROM_SEG_ORG   0x3f000000
#define DROM_SEG_LEN   FLASH_SIZE
#define IROM_SEG_ORG   0x40080000
#define IROM_SEG_LEN   0x780000
#define ROM_SEG_ALIGN  0x10000

/* Aliases for Zephyr linker scripts */
#define RAMABLE_REGION dram0_0_seg
#define ROMABLE_REGION FLASH

#define IRAM_REGION    iram0_0_seg
#define DRAM_REGION    dram0_0_seg
#define RODATA_REGION  drom0_0_seg
#define ROTEXT_REGION  irom0_0_seg

/* Flash segments (rodata and text) should be mapped in virtual address space by providing VMA.
 * Executing directly from LMA is not possible. */
#undef GROUP_ROM_LINK_IN
#define GROUP_ROM_LINK_IN(vregion, lregion) > RODATA_REGION AT > lregion

#define RESERVE_RTC_MEM 0

MEMORY
{
#ifdef CONFIG_BOOTLOADER_MCUBOOT
  mcuboot_hdr (R): org = 0x0, len = 0x20
  metadata (R): org = 0x20, len = 0x20
  FLASH (R): org = 0x40, len = FLASH_SIZE
#else
  /* Make safety margin in the FLASH memory size so the
   * (esp_img_header + (n*esp_seg_headers)) would fit */
  FLASH (R): org = 0x0, len = FLASH_SIZE - 0x100
#endif /* CONFIG_BOOTLOADER_MCUBOOT */

  iram0_0_seg(RX): org = IRAM_SEG_ORG, len = I_D_RAM_SIZE
  dram0_0_seg(RW): org = DRAM_SEG_ORG, len = I_D_RAM_SIZE
  irom0_0_seg(RX): org = IROM_SEG_ORG, len = IROM_SEG_LEN
  drom0_0_seg(R):  org = DROM_SEG_ORG, len = DROM_SEG_LEN

  rtc_iram_seg(RWX): org = 0x40070000, len = 0x2000 - RESERVE_RTC_MEM
  rtc_slow_seg(RW):  org = 0x50000000, len = 0x2000
  /* RTC fast memory (same block as above, rtc_iram_seg), viewed from data bus */
  rtc_data_seg(RW) : org = 0x3ff9e000, len = 0x2000 - RESERVE_RTC_MEM
  /* We reduced the size of rtc_data_seg and rtc_iram_seg by RESERVE_RTC_MEM value.
   * It reserves the amount of RTC fast memory that we use for this memory segment.
   * This segment is intended for keeping:
   *  - (lower addr) rtc timer data (s_rtc_timer_retain_mem, see esp_clk.c files).
   *  - (higher addr) bootloader rtc data (s_bootloader_retain_mem, when a Kconfig option is on).
   * The aim of this is to keep data that will not be moved around and have a fixed address.
   */
  rtc_reserved_seg(RW) : org = 0x3ff9e000 + 0x2000 - RESERVE_RTC_MEM, len = RESERVE_RTC_MEM
#ifdef CONFIG_ESP_SPIRAM
  ext_ram_seg(RW): org = 0x3f800000, len = CONFIG_ESP_SPIRAM_SIZE /* OR 0x780000 */
#endif /* CONFIG_ESP_SPIRAM */

#ifdef CONFIG_GEN_ISR_TABLES
  IDT_LIST(RW): org = 0x3ebfe010, len = 0x2000
#endif /* CONFIG_GEN_ISR_TABLES */
}

/* Default entry point:  */
ENTRY(CONFIG_KERNEL_ENTRY)

_rom_store_table = 0;

_heap_sentry = 0x3fffe710;

SECTIONS
{
  _i_d_ram_offset = I_D_RAM_OFFSET;

#ifdef CONFIG_BOOTLOADER_MCUBOOT
  /* Reserve space for MCUboot header in the binary */
  .mcuboot_header :
  {
    QUAD(0x0)
    QUAD(0x0)
    QUAD(0x0)
    QUAD(0x0)
  } > mcuboot_hdr
  .metadata :
  {
    /* 0. Magic byte for load header */
    LONG(0xace637d3)

    /* 1. Application entry point address */
    KEEP(*(.entry_addr))

    /* IRAM load:
     * 2. Destination address (VMA) for IRAM region
     * 3. Flash offset (LMA) for start of IRAM region
     * 4. Size of IRAM region
     */
    LONG(ADDR(.iram0.vectors))
    LONG(LOADADDR(.iram0.vectors))
    LONG(LOADADDR(.iram0.text) + SIZEOF(.iram0.text) - LOADADDR(.iram0.vectors))

    /* DRAM load:
     * 5. Destination address (VMA) for DRAM region
     * 6. Flash offset (LMA) for start of DRAM region
     * 7. Size of DRAM region
     */
    LONG(ADDR(.dram0.data))
    LONG(LOADADDR(.dram0.data))
    LONG(LOADADDR(.dram0.data_end) - LOADADDR(.dram0.data))
  } > metadata
#endif /* CONFIG_BOOTLOADER_MCUBOOT */

/* Virtual un-loadable sections */
#include <zephyr/linker/rel-sections.ld>

  /* --- START OF RTC --- */

  /* RTC fast memory holds RTC wake stub code,
   * including from any source file named rtc_wake_stub*.c
   */
  .rtc.text :
  {
    _rtc_text_start = ABSOLUTE(.);
    . = ALIGN(4);

    _rtc_code_start = .;

    /* mapping[rtc_text] */

    *rtc_wake_stub*.*(.literal .text .literal.* .text.*)
    _rtc_code_end = .;

    /* possibly align + add 16B for CPU dummy speculative instr. fetch */
    . = ((_rtc_code_end - _rtc_code_start) == 0) ? ALIGN(0) : ALIGN(4) + 16;

    _rtc_text_end = ABSOLUTE(.);
  } GROUP_DATA_LINK_IN(rtc_iram_seg, ROMABLE_REGION)

  /*
   * This section is required to skip rtc.text area because rtc_iram_seg and
   * rtc_data_seg are reflect the same address space on different buses.
   */
  .rtc.dummy :
  {
    _rtc_dummy_start = ABSOLUTE(.);
    _rtc_fast_start = ABSOLUTE(.);
    . = SIZEOF(.rtc.text);
    _rtc_dummy_end = ABSOLUTE(.);
  } GROUP_DATA_LINK_IN(rtc_iram_seg, ROMABLE_REGION)

  /* This section located in RTC FAST Memory area.
   *  It holds data marked with RTC_FAST_ATTR attribute.
   *  See the file "esp_attr.h" for more information.
   */
  .rtc.force_fast :
  {
    . = ALIGN(4);
    _rtc_force_fast_start = ABSOLUTE(.);

    /* mapping[rtc_force_fast] */

    *(.rtc.force_fast .rtc.force_fast.*)
    . = ALIGN(4) ;
    _rtc_force_fast_end = ABSOLUTE(.);
  } > rtc_data_seg

  /* RTC data section holds RTC wake stub
   *  data/rodata, including from any source file
   *  named rtc_wake_stub*.c and the data marked with
   *  RTC_DATA_ATTR, RTC_RODATA_ATTR attributes.
   *  The memory location of the data is dependent on
   *  CONFIG_ESP32S2_RTCDATA_IN_FAST_MEM option.
   */
  .rtc.data :
  {
    _rtc_data_start = ABSOLUTE(.);

    /* mapping[rtc_data] */

    *rtc_wake_stub*.*(.data .rodata .data.* .rodata.*)
    _rtc_data_end = ABSOLUTE(.);
  } > rtc_data_seg

  /* RTC bss, from any source file named rtc_wake_stub*.c */
  .rtc.bss (NOLOAD) :
  {
    _rtc_bss_start = ABSOLUTE(.);
    *rtc_wake_stub*.*(.bss .bss.*)
    *rtc_wake_stub*.*(COMMON)

    //mapping[rtc_bss]

    _rtc_bss_end = ABSOLUTE(.);
  } > rtc_data_seg

  /* This section holds data that should not be initialized at power up
   *  and will be retained during deep sleep.
   *  User data marked with RTC_NOINIT_ATTR will be placed
   *  into this section. See the file "esp_attr.h" for more information.
   *  The memory location of the data is dependent on
   *  CONFIG_ESP32S2_RTCDATA_IN_FAST_MEM option.
   */
  .rtc_noinit (NOLOAD):
  {
    . = ALIGN(4);
    _rtc_noinit_start = ABSOLUTE(.);
    *(.rtc_noinit .rtc_noinit.*)
    . = ALIGN(4) ;
    _rtc_noinit_end = ABSOLUTE(.);
  } > rtc_data_location

  /* This section located in RTC SLOW Memory area.
   *  It holds data marked with RTC_SLOW_ATTR attribute.
   *  See the file "esp_attr.h" for more information.
   */
  .rtc.force_slow :
  {
    . = ALIGN(4);
    _rtc_force_slow_start = ABSOLUTE(.);
    *(.rtc.force_slow .rtc.force_slow.*)
    . = ALIGN(4) ;
    _rtc_force_slow_end = ABSOLUTE(.);
  } > rtc_slow_seg

  /**
   * This section holds RTC data that should have fixed addresses.
   * The data are not initialized at power-up and are retained during deep sleep.
   */
  .rtc_reserved (NOLOAD):
  {
    . = ALIGN(4);
    _rtc_reserved_start = ABSOLUTE(.);
    /* New data can only be added here to ensure existing data are not moved.
       Because data have adhered to the end of the segment and code is relied on it.
       >> put new data here << */

    *(.rtc_timer_data_in_rtc_mem .rtc_timer_data_in_rtc_mem.*)
    KEEP(*(.bootloader_data_rtc_mem .bootloader_data_rtc_mem.*))
    _rtc_reserved_end = ABSOLUTE(.);
  } > rtc_reserved_seg

  /* TODO: add rtc segments sanity check */

  /* --- END OF RTC --- */

  /* --- START OF IRAM --- */

  /* Send .iram0 code to iram */
  .iram0.vectors : ALIGN(4)
  {
    _iram_start = ABSOLUTE(.);
    /* Vectors go to IRAM */
    _init_start = ABSOLUTE(.);
    /* Vectors according to builds/RF-2015.2-win32/esp108_v1_2_s5_512int_2/config.html */
    . = 0x0;
    KEEP(*(.WindowVectors.text));
    . = 0x180;
    KEEP(*(.Level2InterruptVector.text));
    . = 0x1c0;
    KEEP(*(.Level3InterruptVector.text));
    . = 0x200;
    KEEP(*(.Level4InterruptVector.text));
    . = 0x240;
    KEEP(*(.Level5InterruptVector.text));
    . = 0x280;
    KEEP(*(.DebugExceptionVector.text));
    . = 0x2c0;
    KEEP(*(.NMIExceptionVector.text));
    . = 0x300;
    KEEP(*(.KernelExceptionVector.text));
    . = 0x340;
    KEEP(*(.UserExceptionVector.text));
    . = 0x3C0;
    KEEP(*(.DoubleExceptionVector.text));
    . = 0x400;
    *(.*Vector.literal)

    *(.UserEnter.literal);
    *(.UserEnter.text);
    . = ALIGN (16);
    *(.entry.text)
    *(.init.literal)
    *(.init)
    _init_end = ABSOLUTE(.);

    /* This goes here, not at top of linker script, so addr2line finds it last,
       and uses it in preference to the first symbol in IRAM */
  } GROUP_DATA_LINK_IN(IRAM_REGION, ROMABLE_REGION)

  .iram0.text : ALIGN(4)
  {
    _iram_text_start = ABSOLUTE(.);
    . = ALIGN(8);
    /* Code marked as running out of IRAM */
    *(.iram1 .iram1.*)
    *(.iram0.literal .iram.literal .iram.text.literal .iram0.text .iram.text)
    *libesp32.a:panic.*(.literal .text .literal.* .text.*)
    *librtc.a:(.literal .text .literal.* .text.*)

    *libsubsys__net__l2__ethernet.a:(.literal .text .literal.* .text.*)
    *libsubsys__net__lib__config.a:(.literal .text .literal.* .text.*)
    *libsubsys__net__ip.a:(.literal .text .literal.* .text.*)
    *libsubsys__net.a:(.literal .text .literal.* .text.*)

    *libarch__xtensa__core.a:(.literal .text .literal.* .text.*)
    *libkernel.a:(.literal .text .literal.* .text.*)
    *libzephyr.a:rtc_*.*(.literal .text .literal.* .text.*)
    *libzephyr.a:cpu_util.*(.literal .text .literal.* .text.*)
    *libgcc.a:lib2funcs.*(.literal .text .literal.* .text.*)
    *libzephyr.a:windowspill_asm.*(.literal .text .literal.* .text.*)
    *libzephyr.a:log_noos.*(.literal .text .literal.* .text.*)
    *libdrivers__timer.a:xtensa_sys_timer.*(.literal .text .literal.* .text.*)
    *libzephyr.a:log_core.*(.literal .text .literal.* .text.*)
    *libzephyr.a:cbprintf_complete.*(.literal .text .literal.* .text.*)
    *libzephyr.a:printk.*(.literal.printk .literal.vprintk .literal.char_out .text.printk .text.vprintk .text.char_out)
    *libzephyr.a:log_msg.*(.literal .text .literal.* .text.*)
    *libzephyr.a:log_list.*(.literal .text .literal.* .text.*)
    *libdrivers__console.a:uart_console.*(.literal.console_out .text.console_out)
    *libzephyr.a:log_output.*(.literal .text .literal.* .text.*)
    *libzephyr.a:log_backend_uart.*(.literal .text .literal.* .text.*)
    *libzephyr.a:cache_hal.*(.literal .text .literal.* .text.*)
    *libzephyr.a:mmu_hal.*(.literal .text .literal.* .text.*)
    *libzephyr.a:loader.*(.literal .text .literal.* .text.*)
    *liblib__libc__minimal.a:string.*(.literal .text .literal.* .text.*)
    *liblib__libc__newlib.a:string.*(.literal .text .literal.* .text.*)
    *libc.a:*(.literal .text .literal.* .text.*)
    *libphy.a:( .phyiram .phyiram.*)
    *libgcov.a:(.literal .text .literal.* .text.*)
    *libzephyr.a:cpu.*(.literal.esp_cpu_compare_and_set .text.esp_cpu_compare_and_set)
    *libzephyr.a:cpu.*(.literal.esp_cpu_reset .text.esp_cpu_reset)
    *libzephyr.a:cpu.*(.literal.esp_cpu_stall .text.esp_cpu_stall)
    *libzephyr.a:cpu.*(.literal.esp_cpu_unstall .text.esp_cpu_unstall)
    *libzephyr.a:cpu.*(.literal.esp_cpu_wait_for_intr .text.esp_cpu_wait_for_intr)
    *libzephyr.a:esp_gpio_reserve.*(.literal.esp_gpio_is_pin_reserved .text.esp_gpio_is_pin_reserved)
    *libzephyr.a:esp_gpio_reserve.*(.literal.esp_gpio_reserve_pins .text.esp_gpio_reserve_pins)
    *libzephyr.a:esp_memory_utils.*(.literal .literal.* .text .text.*)
    *libzephyr.a:periph_ctrl.*(.literal .text .literal.* .text.*)
    *libzephyr.a:rtc_clk.*(.literal .literal.* .text .text.*)
    *libzephyr.a:rtc_sleep.*(.literal .literal.* .text .text.*)
    *libzephyr.a:rtc_time.*(.literal .literal.* .text .text.*)
    *libzephyr.a:rtc_wdt.*(.literal .literal.* .text .text.*)
    *libzephyr.a:sar_periph_ctrl.*(.literal.sar_periph_ctrl_power_enable .text.sar_periph_ctrl_power_enable)
    *libzephyr.a:systimer.*(.literal .literal.* .text .text.*)
    *libzephyr.a:esp_cache.*(.literal .literal.* .text .text.*)
    *libzephyr.a:esp_rom_cache_esp32s2_esp32s3.*(.literal .literal.* .text .text.*)
    *libzephyr.a:esp_rom_spiflash.*(.literal .literal.* .text .text.*)
    *libzephyr.a:esp_rom_systimer.*(.literal .literal.* .text .text.*)
    *libzephyr.a:esp_err.*(.literal .literal.* .text .text.*)
    *libzephyr.a:esp_system_chip.*(.literal.esp_system_abort .text.esp_system_abort)
    *libzephyr.a:i2c_hal_iram.*(.literal .literal.* .text .text.*)
    *libzephyr.a:ledc_hal_iram.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_encrypt_hal_iram.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_hal_gpspi.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_hal_iram.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_hal.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_init.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_hal_iram.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_slave_hal_iram.*(.literal .literal.* .text .text.*)
    *libzephyr.a:systimer_hal.*(.literal .literal.* .text .text.*)
    *libzephyr.a:wdt_hal_iram.*(.literal .literal.* .text .text.*)
    *libzephyr.a:flash_brownout_hook.*(.literal .literal.* .text .text.*)
    *libzephyr.a:memspi_host_driver.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_chip_boya.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_chip_gd.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_chip_generic.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_chip_issi.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_chip_mxic.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_chip_th.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_chip_winbond.*(.literal .literal.* .text .text.*)
    *libzephyr.a:spi_flash_wrap.*(.literal .literal.* .text .text.*)

    *libzephyr.a:esp_mmu_map.*(.literal .literal.* .text .text.*)
    *libzephyr.a:ext_mem_layout.*(.literal .literal.* .text .text.*)
    *libzephyr.a:reset_reason.*(.literal .literal.* .text .text.*)

#ifdef CONFIG_ESP32_WIFI_IRAM_OPT
    *libnet80211.a:( .wifi0iram  .wifi0iram.* .wifislpiram .wifislpiram.*)
    *libpp.a:( .wifi0iram  .wifi0iram.* .wifislpiram .wifislpiram.* .wifiorslpiram .wifiorslpiram.*)
#endif /* CONFIG_ESP32_WIFI_IRAM_OPT */

#ifdef CONFIG_ESP32_WIFI_RX_IRAM_OPT
    *libnet80211.a:( .wifirxiram  .wifirxiram.* .wifislprxiram .wifislprxiram.*)
    *libpp.a:( .wifirxiram  .wifirxiram.* .wifislprxiram .wifislprxiram.*)
#endif /* CONFIG_ESP32_WIFI_RX_IRAM_OPT */

    /* align + add 16B for CPU dummy speculative instr. fetch */
    . = ALIGN(4) + 16;
    _iram_text = ABSOLUTE(.);
    _iram_text_end = ABSOLUTE(.);
  } GROUP_DATA_LINK_IN(IRAM_REGION, ROMABLE_REGION)

#ifdef CONFIG_ESP_SIMPLE_BOOT
  .loader.text :
  {
    . =  ALIGN(4);
    *libzephyr.a:bootloader_soc.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_flash.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_flash_config_esp32s2.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_clock_loader.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_common.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_common_loader.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_mem.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_init.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_esp32s2.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_random.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_random*.*(.literal.bootloader_random_disable .text.bootloader_random_disable)
    *libzephyr.a:bootloader_random*.*(.literal.bootloader_random_enable .text.bootloader_random_enable)
    *libzephyr.a:bootloader_efuse_esp32s2.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_utility.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_sha.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_console.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_console_loader.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_panic.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_clock_init.*(.literal .text .literal.* .text.*)
    *libzephyr.a:bootloader_wdt.*(.literal .text .literal.* .text.*)

    *libzephyr.a:esp_flash_api.*(.literal .text .literal.* .text.*)
    *libzephyr.a:esp_flash_spi_init.*(.literal .text .literal.* .text.*)
    *libzephyr.a:esp_image_format.*(.literal .text .literal.* .text.*)
    *libzephyr.a:flash_encrypt.*(.literal .text .literal.* .text.*)

    *libzephyr.a:flash_encryption_secure_features.*(.literal .text .literal.* .text.*)
    *libzephyr.a:flash_partitions.*(.literal .text .literal.* .text.*)
    *libzephyr.a:secure_boot.*(.literal .text .literal.* .text.*)
    *libzephyr.a:secure_boot_secure_features.*(.literal .text .literal.* .text.*)
    *libzephyr.a:secure_boot_signatures_bootloader.*(.literal .text .literal.* .text.*)

    *libzephyr.a:wdt_hal_iram.*(.literal .text .literal.* .text.*)
    *libzephyr.a:efuse_hal.*(.literal .text .literal.* .text.*)

    *libzephyr.a:esp_efuse_table.*(.literal .text .literal.* .text.*)
    *libzephyr.a:esp_efuse_fields.*(.literal .text .literal.* .text.*)
    *libzephyr.a:esp_efuse_api.*(.literal .text .literal.* .text.*)
    *libzephyr.a:esp_efuse_utility.*(.literal .text .literal.* .text.*)
    *libzephyr.a:esp_efuse_api_key_esp32.*(.literal .text .literal.* .text.*)

    *libzephyr.a:app_cpu_start.*(.literal .text .literal.* .text.*)
    *libzephyr.a:mpu_hal.*(.literal .text .literal.* .text.*)
    *libzephyr.a:rtc_init.*(.literal .text .literal.* .text.*)
    *libzephyr.a:rtc_time.*(.literal .text .literal.* .text.*)
    *libzephyr.a:rtc_clk.*(.literal .text .literal.* .text.*)
    *libzephyr.a:rtc_clk_init.*(.literal .text .literal.* .text.*)

    *(.fini.literal)
    *(.fini)

    . =  ALIGN(4);
    _loader_text_end = ABSOLUTE(.);
  } GROUP_DATA_LINK_IN(IRAM_REGION, ROMABLE_REGION)
#endif /* CONFIG_ESP_SIMPLE_BOOT */

  .iram0.text_end (NOLOAD) :
  {
    . = ALIGN(16);
    _iram_text_end_end = ABSOLUTE(.);
  } GROUP_LINK_IN(IRAM_REGION)

  .iram0.data :
  {
    . = ALIGN(16);
    *(.iram.data)
    *(.iram.data*)
  } GROUP_DATA_LINK_IN(IRAM_REGION, ROMABLE_REGION)

  .iram0.bss (NOLOAD) :
  {
    . = ALIGN(16);
    _iram_bss_start = ABSOLUTE(.);
    *(.iram.bss)
    *(.iram.bss.*)
    _iram_bss_end = ABSOLUTE(.);

    . = ALIGN(4);
    _iram_end = ABSOLUTE(.);
  } GROUP_LINK_IN(IRAM_REGION)

  /* --- END OF IRAM --- */

  /* --- START OF DRAM --- */
  .dram0.reserved (NOLOAD):
  {
    /* This section is required to skip .iram0.text area because iram0_0_seg and
     * dram0_0_seg reflect the same address space on different buses.
     */
    . = ORIGIN(dram0_0_seg) + MAX(_iram_end, IRAM_SEG_ORG) - IRAM_SEG_ORG;
    . += 0x10;
    . = ALIGN (16);
  } GROUP_LINK_IN(DRAM_REGION)

  .dram0.data :
  {
    . = ALIGN (8);
    _dram_data_start = ABSOLUTE(.);
    _data_start = ABSOLUTE(.);
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    *(.data1)
    *(.sdata)
    *(.sdata.*)
    *(.gnu.linkonce.s.*)
    *(.sdata2)
    *(.sdata2.*)
    *(.gnu.linkonce.s2.*)
    /* rodata for panic handler(libarch__xtensa__core.a) and all
     * dependent functions should be placed in DRAM to avoid issue
     * when flash cache is disabled */
    *libarch__xtensa__core.a:(.rodata .rodata.*)
    *libkernel.a:fatal.*(.rodata .rodata.*)
    *libkernel.a:init.*(.rodata .rodata.*)
    *libzephyr.a:cbprintf_complete*(.rodata .rodata.*)
    *libzephyr.a:log_core.*(.rodata .rodata.*)
    *libzephyr.a:log_backend_uart.*(.rodata .rodata.*)
    *libzephyr.a:log_output.*(.rodata .rodata.*)
    *libzephyr.a:loader.*(.rodata .rodata.*)
    *libdrivers__flash.a:flash_esp32.*(.rodata  .rodata.*)
    *libzephyr.a:spi_flash_rom_patch.*(.rodata  .rodata.*)
    *libdrivers__serial.a:uart_esp32.*(.rodata .rodata.*)

    *libzephyr.a:esp_mmu_map.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:ext_mem_layout.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:reset_reason.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:periph_ctrl.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)

    *libzephyr.a:esp_memory_utils.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:rtc_clk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:systimer.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:esp_cache.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:esp_rom_cache_esp32s2_esp32s3.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:esp_rom_spiflash.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:esp_rom_systimer.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:esp_err.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:cache_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:i2c_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:ledc_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:mmu_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_encrypt_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_hal_gpspi.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_init.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_slave_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:systimer_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:wdt_hal_iram.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:flash_brownout_hook.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:memspi_host_driver.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_chip_boya.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_chip_gd.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_chip_generic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_chip_issi.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_chip_mxic.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_chip_th.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_chip_winbond.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:spi_flash_wrap.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)

    KEEP(*(.jcr))
    *(.dram1 .dram1.*)

    . = ALIGN(4);
    #include <snippets-rwdata.ld>
    . = ALIGN(4);
  } GROUP_DATA_LINK_IN(DRAM_REGION, ROMABLE_REGION)

#ifndef CONFIG_BOOTLOADER_MCUBOOT
  .loader.data :
  {
    . = ALIGN(4);
    _loader_data_start = ABSOLUTE(.);

    *libzephyr.a:cpu_util.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:clk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:esp_clk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:rtc_clk.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:rtc_clk_init.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:rtc_time.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:efuse_hal.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)


    *libzephyr.a:esp_flash_api.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:esp_flash_spi_init.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)

    *libzephyr.a:bootloader_efuse.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)

    *libzephyr.a:bootloader_clock_init.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:bootloader_common.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:bootloader_common_loader.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:bootloader_init.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:bootloader_esp32s2.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:bootloader_wdt.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:bootloader_flash.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:bootloader_flash_config_esp32s2.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    *libzephyr.a:bootloader_random_esp32s2.*(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.*)
    . = ALIGN(4);
    _loader_data_end = ABSOLUTE(.);
  } GROUP_DATA_LINK_IN(DRAM_REGION, ROMABLE_REGION)
#endif /* !CONFIG_BOOTLOADER_MCUBOOT */

  #include <zephyr/linker/cplusplus-rom.ld>
  #include <snippets-data-sections.ld>
  #include <zephyr/linker/common-ram.ld>
  #include <snippets-ram-sections.ld>
  #include <zephyr/linker/cplusplus-ram.ld>

  /* logging sections should be placed in RAM area to avoid flash cache disabled issues */
  #pragma push_macro("GROUP_ROM_LINK_IN")
  #undef GROUP_ROM_LINK_IN
  #define GROUP_ROM_LINK_IN GROUP_DATA_LINK_IN
  #include <zephyr/linker/common-rom/common-rom-logging.ld>
  #pragma pop_macro("GROUP_ROM_LINK_IN")

  .dram0.data_end :
  {
    . = ALIGN(4);
    _data_end = ABSOLUTE(.);
    _dram_data_end = ABSOLUTE(.);

  } GROUP_DATA_LINK_IN(DRAM_REGION, ROMABLE_REGION)

  /* .bss */
  SECTION_DATA_PROLOGUE(_BSS_SECTION_NAME,(NOLOAD),)
  {
    . = ALIGN (8);
    __bss_start = ABSOLUTE(.);
    _bss_start = ABSOLUTE(.);

    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.bss)
    *(.bss.*)
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN (8);
    __bss_end = ABSOLUTE(.);
    _bss_end = ABSOLUTE(.);
  } GROUP_LINK_IN(DRAM_REGION)

#ifdef CONFIG_ESP_SPIRAM
  /* This section holds .ext_ram.bss data, and will be put in PSRAM */
  .ext_ram.bss (NOLOAD):
  {
    _ext_ram_data_start = ABSOLUTE(.);
    _ext_ram_bss_start = ABSOLUTE(.);
    *(.ext_ram.bss*)
    . = ALIGN(4);
    _ext_ram_bss_end = ABSOLUTE(.);
  } > ext_ram_seg

  .ext_ram_noinit (NOLOAD) :
  {
#ifdef CONFIG_ESP32_WIFI_NET_ALLOC_SPIRAM
    *libdrivers__wifi.a:(.noinit .noinit.*)
    *libsubsys__net__l2__ethernet.a:(.noinit .noinit.*)
    *libsubsys__net__lib__config.a:(.noinit .noinit.*)
    *libsubsys__net__ip.a:(.noinit .noinit.*)
    *libsubsys__net.a:(.noinit .noinit.*)
#endif /* CONFIG_ESP32_WIFI_NET_ALLOC_SPIRAM */

    _spiram_heap_start = ABSOLUTE(.);
    . = . + CONFIG_ESP_SPIRAM_HEAP_SIZE;

    _ext_ram_data_end = ABSOLUTE(.);
  } > ext_data_ram_seg
#endif /* CONFIG_ESP_SPIRAM */

  /* .noinit */
  SECTION_DATA_PROLOGUE(_NOINIT_SECTION_NAME, (NOLOAD),)
  {
    . = ALIGN(8);
    *(.noinit)
    *(.noinit.*)
    . = ALIGN(8);
  } GROUP_LINK_IN(DRAM_REGION)

  .dram0.end (NOLOAD) :
  {
    . = ALIGN(4);
    _end = ABSOLUTE(.);
  } GROUP_DATA_LINK_IN(DRAM_REGION, ROMABLE_REGION)

  _image_drom_start = LOADADDR(_RODATA_SECTION_NAME);
  _image_drom_size = LOADADDR(_RODATA_SECTION_END) + SIZEOF(_RODATA_SECTION_END)  - _image_drom_start;
  _image_drom_vaddr = ADDR(_RODATA_SECTION_NAME);

  /* --- END OF DRAM --- */

  /* --- START OF .rodata --- */

  /* Align next section to 64k to allow mapping */
  .flash.rodata_dummy (NOLOAD) :
  {
    . = ALIGN(ROM_SEG_ALIGN);
  } GROUP_LINK_IN(ROMABLE_REGION)

  SECTION_PROLOGUE(_RODATA_SECTION_NAME,,ALIGN(4))
  {
    _rodata_reserved_start = ABSOLUTE(.);
    __rodata_region_start = ABSOLUTE(.);

    . = ALIGN(4);
    #include <snippets-rodata.ld>
    . = ALIGN(4);

    *(.rodata .rodata.* .sdata2 .sdata2.* .srodata .srodata.* )

    *(.rodata_wlog_debug.* .rodata_wlog_info.* .rodata_wlog_warning.* .rodata_wlog_error.* .rodata_wlog_verbose.* )

    . = ALIGN(4);
    *(.irom1.text) /* catch stray ICACHE_RODATA_ATTR */
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE_ = ABSOLUTE(.);
    *(.xt_except_table)
    *(.gcc_except_table .gcc_except_table.*)
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    . = (. + 3) & ~ 3;
    __eh_frame = ABSOLUTE(.);
    KEEP(*(.eh_frame))
    . = (. + 7) & ~ 3;
    /*  C++ constructor and destructor tables are
     *  provided by the Zephyr linker includes */

    /*  C++ exception handlers table:  */
    __XT_EXCEPTION_DESCS_ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    *(.dynamic)
    *(.gnu.version_d)
    . = ALIGN(4);
    __rodata_region_end = ABSOLUTE(.);
    /* Literals are also RO data. */
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
    . = ALIGN(4);
  } GROUP_DATA_LINK_IN(RODATA_REGION, ROMABLE_REGION)

  #include <zephyr/linker/common-rom/common-rom-cpp.ld>
  #include <zephyr/linker/common-rom/common-rom-kernel-devices.ld>
  #include <zephyr/linker/common-rom/common-rom-ztest.ld>
  #include <zephyr/linker/common-rom/common-rom-net.ld>
  #include <zephyr/linker/common-rom/common-rom-debug.ld>
  #include <zephyr/linker/common-rom/common-rom-misc.ld>
  #include <zephyr/linker/thread-local-storage.ld>
  #include <snippets-sections.ld>

  /* Create an explicit section at the end of all the data that shall be mapped into drom.
   * This is used to calculate the size of the _image_drom_size variable */
  SECTION_PROLOGUE(_RODATA_SECTION_END,,)
  {
    . = ALIGN(ROM_SEG_ALIGN);
    _image_rodata_end = ABSOLUTE(.);
    _rodata_region_end = ABSOLUTE(.);
    _rodata_reserved_end = ABSOLUTE(.);

  } GROUP_DATA_LINK_IN(RODATA_REGION, ROMABLE_REGION)

  /* --- END OF .rodata --- */


  /* --- START OF .flash.text --- */

  _image_irom_start = LOADADDR(.flash.text);
  _image_irom_size = LOADADDR(.flash.text) + SIZEOF(.flash.text) - _image_irom_start;
  _image_irom_vaddr = ADDR(.flash.text);

  .flash.text_dummy (NOLOAD):
  {
    . = ALIGN(ROM_SEG_ALIGN);
  } GROUP_LINK_IN(ROMABLE_REGION)

  .flash.text : ALIGN(4)
  {
    . = ALIGN(4);
    _stext = .;
    _instruction_reserved_start = ABSOLUTE(.);  /* This is a symbol marking the flash.text start, this can be used for mmu driver to maintain virtual address */
    _text_start = ABSOLUTE(.);

#ifndef CONFIG_ESP32_WIFI_IRAM_OPT
    *libnet80211.a:( .wifi0iram  .wifi0iram.* .wifislpiram .wifislpiram.*)
    *libpp.a:( .wifi0iram  .wifi0iram.* .wifislpiram .wifislpiram.* .wifiorslpiram .wifiorslpiram.*)
#endif /* CONFIG_ESP32_WIFI_IRAM_OPT */

    . = ALIGN(8);
#ifndef CONFIG_ESP32_WIFI_RX_IRAM_OPT
    *libnet80211.a:( .wifirxiram  .wifirxiram.* .wifislprxiram .wifislprxiram.*)
    *libpp.a:( .wifirxiram  .wifirxiram.* .wifislprxiram .wifislprxiram.*)
#endif /* CONFIG_ESP32_WIFI_RX_IRAM_OPT */

    *(.stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.irom0.text) /* catch stray ICACHE_RODATA_ATTR */

#ifdef CONFIG_BOOTLOADER_MCUBOOT
    *(.fini.literal)
    *(.fini)
#endif /* CONFIG_BOOTLOADER_MCUBOOT */

    *(.gnu.version)
    *(.literal .text .literal.* .text.*)
    . += 16;

    _text_end = ABSOLUTE(.);
    _instruction_reserved_end = ABSOLUTE(.);
    _etext = .;

    /* Similar to _iram_start, this symbol goes here so it is
     * resolved by addr2line in preference to the first symbol in
     * the flash.text segment.
     */
    _flash_cache_start = ABSOLUTE(0);
  } GROUP_DATA_LINK_IN(ROTEXT_REGION, ROMABLE_REGION)

  /* --- END OF .flash.text --- */

#ifdef CONFIG_GEN_ISR_TABLES
#include <zephyr/linker/intlist.ld>
#endif /* CONFIG_GEN_ISR_TABLES */

#include <zephyr/linker/debug-sections.ld>

  .xtensa.info  0 :  { *(.xtensa.info) }
  .xt.insn 0 :
  {
    KEEP (*(.xt.insn))
    KEEP (*(.gnu.linkonce.x.*))
  }
  .xt.prop 0 :
  {
    KEEP (*(.xt.prop))
    KEEP (*(.xt.prop.*))
    KEEP (*(.gnu.linkonce.prop.*))
  }
  .xt.lit 0 :
  {
    KEEP (*(.xt.lit))
    KEEP (*(.xt.lit.*))
    KEEP (*(.gnu.linkonce.p.*))
  }
  .xt.profile_range 0 :
  {
    KEEP (*(.xt.profile_range))
    KEEP (*(.gnu.linkonce.profile_range.*))
  }
  .xt.profile_ranges 0 :
  {
    KEEP (*(.xt.profile_ranges))
    KEEP (*(.gnu.linkonce.xt.profile_ranges.*))
  }
  .xt.profile_files 0 :
  {
    KEEP (*(.xt.profile_files))
    KEEP (*(.gnu.linkonce.xt.profile_files.*))
  }
}

ASSERT(((__bss_end - ORIGIN(dram0_0_seg)) <= LENGTH(dram0_0_seg)),
          "DRAM0 segment data does not fit.")

ASSERT(((_iram_end - ORIGIN(iram0_0_seg)) <= LENGTH(iram0_0_seg)),
          "IRAM0 segment data does not fit.")

#ifdef CONFIG_ESP_SPIRAM
ASSERT(((_ext_ram_data_end - _ext_ram_data_start) <= CONFIG_ESP_SPIRAM_SIZE),
          "External SPIRAM overflowed.")
#endif /* CONFIG_ESP_SPIRAM */
