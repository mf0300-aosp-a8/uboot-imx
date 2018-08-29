/*
 * Copyright (C) 2012-2014 FIC Computer, Inc.
 *
 * Configuration settings for the Fic i.MX6Q MF0300 board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QMF0300_CONFIG_H
#define __MX6QMF0300_CONFIG_H

#define CONFIG_MACH_TYPE               3774                       /* set the machine type number  */
#define CONFIG_MXC_UART_BASE           UART4_BASE                 /* default console device */
#define CONFIG_CONSOLE_DEV             "ttymxc3"                  /* console device */
#define CONFIG_MMCROOT                 "/dev/mmcblk3p2"           /* kernel partition */
#define PHYS_SDRAM_SIZE                SZ_2G                      /* 2GB RAM */

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC                  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS                   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT        2        /* enabled USB controller number */

/* MAX7310 configs*/
#define CONFIG_MAX7310_IOEXP
#define CONFIG_IOEXP_DEVICES_NUM               3
#define CONFIG_IOEXP_DEV_PINS_NUM              8

#define CONFIG_SYS_FSL_USDHC_NUM               2
#define CONFIG_SYS_MMC_ENV_DEV                 1        /* environment is stored in mmc1 device */
#define CONFIG_SYS_MMC_ENV_PART                0        /* environment is stored in 0 partition */

#define CONFIG_SYS_MMC_SDCARD_DEV              0        /* mmc0 is sdcard device for sdcard flasher */
#define CONFIG_SYS_MMC_SDCARD_DEV_PART         1        /* sdcard partition to load */
#define CONFIG_SYS_MMC_INTERNAL_DEV            1        /* mmc1 is main internal boot device */

#define CONFIG_MENUKEY                        'm'
#include "mx6qmf0300_common.h"
#include <asm/imx-common/gpio.h>

#endif                         /* __MX6QMF0300_CONFIG_H */
