/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "soc/interrupts.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: [ESP32C5-PERIPH] (inherit from C6)

typedef enum {
    /* HP peripherals */
    PERIPH_LEDC_MODULE = 0,
    PERIPH_UART0_MODULE,
    PERIPH_UART1_MODULE,
    PERIPH_USB_DEVICE_MODULE,  // USB Serial Jtag
    PERIPH_I2C_MODULE,
    PERIPH_I2S_MODULE,
    PERIPH_TIMG0_MODULE,
    PERIPH_TIMG1_MODULE,
    PERIPH_UHCI0_MODULE,
    PERIPH_RMT_MODULE,
    PERIPH_PCNT_MODULE,
    PERIPH_MSPI0_MODULE,  //SPI0
    PERIPH_MSPI1_MODULE,  //SPI1
    PERIPH_GPSPI2_MODULE, //SPI2
    PERIPH_TWAI0_MODULE,
    PERIPH_TWAI1_MODULE,
    PERIPH_RNG_MODULE,
    PERIPH_RSA_MODULE,
    PERIPH_AES_MODULE,
    PERIPH_SHA_MODULE,
    PERIPH_ECC_MODULE,
    PERIPH_HMAC_MODULE,
    PERIPH_DS_MODULE,
    PERIPH_GDMA_MODULE,
    PERIPH_MCPWM0_MODULE,
    PERIPH_ETM_MODULE,
    PERIPH_PARLIO_MODULE,
    PERIPH_SYSTIMER_MODULE,
    PERIPH_SARADC_MODULE,
    PERIPH_TEMPSENSOR_MODULE,
    PERIPH_ASSIST_DEBUG_MODULE,
    PERIPH_INT_MATRIX_MODULE,
    PERIPH_PVT_MONITOR_MODULE,
    PERIPH_BITSCRAMBLER_MODULE,
    PERIPH_KEY_MANAGE_MODULE,
    PERIPH_ECDSA_MODULE,
    PERIPH_MEM_MONITOR_MODULE,
    PERIPH_TEE_MODULE,
    PERIPH_HP_APM_MODULE,
    /* LP peripherals */
    PERIPH_LP_I2C0_MODULE,
    PERIPH_LP_UART0_MODULE,
    PERIPH_LP_TEE_MODULE,
    PERIPH_LP_APM_MODULE,
    PERIPH_LP_ANA_PERI_MODULE,
    PERIPH_LP_PERI_MODULE,
    PERIPH_HUK_MODULE,
    PERIPH_OTP_DEBUG_MODULE,
    /* Peripherals clock managed by the modem_clock driver must be listed last in the enumeration */
    PERIPH_WIFI_MODULE,
    PERIPH_BT_MODULE,
    PERIPH_COEX_MODULE,
    PERIPH_PHY_MODULE,
    PERIPH_ANA_I2C_MASTER_MODULE,
    PERIPH_MODEM_ETM_MODULE,
    PERIPH_MODEM_ADC_COMMON_FE_MODULE,
    PERIPH_MODULE_MAX
    /*  !!! Don't append soc modules here !!! */
} periph_module_t;

#define PERIPH_MODEM_MODULE_MIN PERIPH_WIFI_MODULE
#define PERIPH_MODEM_MODULE_MAX PERIPH_MODEM_ADC_COMMON_FE_MODULE
#define PERIPH_MODEM_MODULE_NUM (PERIPH_MODEM_MODULE_MAX - PERIPH_MODEM_MODULE_MIN + 1)
#define IS_MODEM_MODULE(periph)  ((periph>=PERIPH_MODEM_MODULE_MIN) && (periph<=PERIPH_MODEM_MODULE_MAX))

#ifdef __cplusplus
}
#endif
