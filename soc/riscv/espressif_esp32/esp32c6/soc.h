/*
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_H__
#define __SOC_H__

#ifndef _ASMLANGUAGE
#include <soc/soc.h>
#include <rom/ets_sys.h>
#include <rom/spi_flash.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include "esp_private/esp_clk.h"
#endif

/* ECALL Exception numbers */
#define SOC_MCAUSE_ECALL_EXP 11 /* Machine ECALL instruction */
#define SOC_MCAUSE_USER_ECALL_EXP 8 /* User ECALL instruction */

/* Interrupt Mask */
#define SOC_MCAUSE_IRQ_MASK (1 << 31)
/* Exception code Mask */
#define SOC_MCAUSE_EXP_MASK 0x7FFFFFFF

#ifndef _ASMLANGUAGE

void __esp_platform_start(void);

extern void esp_rom_route_intr_matrix(int cpu_no, uint32_t model_num, uint32_t intr_num);
extern void esp_rom_intr_matrix_set(int cpu_no, uint32_t model_num, uint32_t intr_num);
extern int esp_rom_uart_tx_one_char(uint8_t chr);
extern int esp_rom_gpio_matrix_in(uint32_t gpio, uint32_t signal_index,
				    bool inverted);
extern int esp_rom_gpio_matrix_out(uint32_t gpio, uint32_t signal_index,
				     bool out_inverted,
				     bool out_enabled_inverted);

#endif /* _ASMLANGUAGE */

#endif /* __SOC_H__ */
