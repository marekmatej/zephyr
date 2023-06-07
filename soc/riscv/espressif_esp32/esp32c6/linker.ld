/*
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linker command/script file
 *
 */

#if defined(CONFIG_MCUBOOT)
  /* Using mcuboot as ESP32C6 2nd stage bootloader */
  #error "MCUBOOT is not yet supported for ESP32C6"

#else
  /* Application default linker script */
  #include "default.ld"

#endif /* CONFIG_MCUBOOT */
