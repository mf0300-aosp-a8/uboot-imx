/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#ifndef __MX6QMF0300_COMMON_CONFIG_H
#define __MX6QMF0300_COMMON_CONFIG_H

#define CONFIG_MX6

/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE                    0x4000
#endif
#endif

#include "mx6_common.h"
#include <linux/sizes.h>

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN            (16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR        USDHC4_BASE_ADDR
#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE                       ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE                RGMII
#define CONFIG_ETHPRIME                    "FEC"
#define CONFIG_FEC_MXC_PHYADDR             1

#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX                  1
#define CONFIG_BAUDRATE                    115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY                   3

#define CONFIG_LOADADDR                    0x10800000
#define CONFIG_SYS_TEXT_BASE               0x17800000
#define CONFIG_SYS_MMC_IMG_LOAD_PART       1

#define CONFIG_BASE_ENV_SETTINGS \
    "stdin=serial\0" \
    "stdout=serial\0" \
    "stderr=serial" \
    "\0" \
    "splashpos=m,m" \
    "\0" \
    "panel=" __stringify(CONFIG_DEFAULT_PANEL) "" \
    "\0" \
    "baudrate=" __stringify(CONFIG_BAUDRATE) "" \
    "\0" \
    "loadaddr=" __stringify(CONFIG_LOADADDR) "" \
    "\0" \
    "initrd_addr=0x12800000\0" \
    "initrd_high=0xffffffff" \
    "\0" \
    "fdt_addr=0x18000000\0" \
    "fdt_high=0xffffffff" \
    "\0" \
    "internal_mmc_dev=" __stringify(CONFIG_SYS_MMC_INTERNAL_DEV) "\0" \
    "sdcard_mmc_dev=" __stringify(CONFIG_SYS_MMC_SDCARD_DEV) "\0" \
    "sdcard_mmc_dev_part=" __stringify(CONFIG_SYS_MMC_SDCARD_DEV_PART) "\0" \
    "fastboot_dev=mmc" __stringify(CONFIG_SYS_MMC_INTERNAL_DEV) "\0" \
    "androidboot.hardware=freescale\0"

#define CONFIG_MFG_ENV_SETTINGS \
    "mfgtool_args=setenv bootargs console=" CONFIG_CONSOLE_DEV "," __stringify(CONFIG_BAUDRATE) " " \
        "rdinit=/linuxrc " \
        "g_mass_storage.stall=0 g_mass_storage.removable=1 " \
        "g_mass_storage.file=/fat g_mass_storage.ro=1 " \
        "g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
        "g_mass_storage.iSerialNumber=\"\" "\
        "enable_wait_mode=off "\
        "\0" \
    "bootcmd_mfg=run mfgtool_args;" \
        "bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0"

#define CONFIG_SDCARD_FLASHER_ENV_SETTINGS \
    "sdcard_args=setenv bootargs console=" CONFIG_CONSOLE_DEV "," __stringify(CONFIG_BAUDRATE) " " \
        "rdinit=/sdcard.flasher.rc " \
        "enable_wait_mode=off "\
        "video=mxcfb0:dev=ldb,fbpix=RGB32,bpp=32 " \
        "video=mxcfb1:dev=hdmi,if=RGB24,bpp=32 " \
        "video=mxcfb2:off " \
        "video=mxcfb3:off " \
        "consoleblank=0 " \
        "androidboot.hardware=freescale " \
        "androidboot.soc_type=imx6q " \
        "cma=384M " \
        "\0" \
    "script=firmware/boot.scr\0" \
    "sdcard_loadbootscript=" \
        "fatload mmc ${sdcard_mmc_dev}:${sdcard_mmc_dev_part} ${loadaddr} ${script};\0" \
    "sdcard_bootscript=echo Running boot.scr from mmc" __stringify(CONFIG_SYS_MMC_SDCARD_DEV) " ...; " \
        "source\0" \
    "bootcmd_sdcard=run sdcard_args; mmc dev ${sdcard_mmc_dev};" \
        "mmc rescan; run sdcard_loadbootscript; run sdcard_bootscript;\0"

#define CONFIG_ANDROID_ENV_SETTINGS \
    "android_args=setenv bootargs console=" CONFIG_CONSOLE_DEV "," __stringify(CONFIG_BAUDRATE) " " \
        "init=/init " \
        "rootfstype=ext4 " \
        "enable_wait_mode=off "\
        "video=mxcfb0:dev=ldb,fbpix=RGB32,bpp=32 " \
        "video=mxcfb1:dev=hdmi,if=RGB24,bpp=32 " \
        "video=mxcfb2:off " \
        "video=mxcfb3:off " \
        "vmalloc=128M " \
        "androidboot.console=" CONFIG_CONSOLE_DEV " " \
        "consoleblank=0 " \
        "androidboot.hardware=freescale " \
        "androidboot.soc_type=imx6q " \
        "cma=448M " \
        "galcore.contiguousSize=33554432 " \
        "\0" \
    "bootcmd_android=run android_args;" \
        "boota mmc${internal_mmc_dev};\0" \
    "bootcmd_android_recovery=run android_args;" \
        "boota mmc${internal_mmc_dev} recovery;\0"

#define CONFIG_ANDROID_PERMISSIVE_ENV_SETTINGS \
    "android_permissive_args=setenv bootargs console=" CONFIG_CONSOLE_DEV "," __stringify(CONFIG_BAUDRATE) " " \
        "init=/init " \
        "rootfstype=ext4 " \
        "enable_wait_mode=off "\
        "video=mxcfb0:dev=ldb,fbpix=RGB32,bpp=32 " \
        "video=mxcfb1:dev=hdmi,if=RGB24,bpp=32 " \
        "video=mxcfb2:off " \
        "video=mxcfb3:off " \
        "vmalloc=128M " \
        "androidboot.console=" CONFIG_CONSOLE_DEV " " \
        "consoleblank=0 " \
        "androidboot.hardware=freescale " \
        "androidboot.soc_type=imx6q " \
        "androidboot.selinux=permissive " \
        "cma=448M " \
        "galcore.contiguousSize=33554432 " \
        "\0" \
    "bootcmd_android_permissive=run android_permissive_args;" \
        "boota mmc${internal_mmc_dev};\0"

#define CONFIG_EXTRA_ENV_SETTINGS \
    CONFIG_BASE_ENV_SETTINGS \
    CONFIG_MFG_ENV_SETTINGS \
    CONFIG_SDCARD_FLASHER_ENV_SETTINGS \
    CONFIG_ANDROID_ENV_SETTINGS \
    CONFIG_ANDROID_PERMISSIVE_ENV_SETTINGS

#define CONFIG_ARP_TIMEOUT                200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2        "> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE                 1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS                256
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR            CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE                SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS            1
#define PHYS_SDRAM                      MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE           PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR        IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE        IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
    (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
    (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE                 SZ_8K

#ifndef CONFIG_SYS_NOSMP
#define CONFIG_SYS_NOSMP
#endif

#if defined CONFIG_SYS_BOOT_MMC
#define CONFIG_ENV_IS_IN_MMC            /* read U-Boot command from MMC */
#define CONFIG_BOOTCOMMAND              "run bootcmd_android;"
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#elif defined CONFIG_SYS_BOOT_MFG
#define CONFIG_ENV_IS_NOWHERE           /* don't read U-Boot command from storage */
#define CONFIG_BOOTCOMMAND              "run bootcmd_mfg;"
#elif defined CONFIG_SYS_BOOT_SDCARD
#define CONFIG_ENV_IS_NOWHERE           /* don't read U-Boot command from storage */
#define CONFIG_BOOTCOMMAND              "run bootcmd_sdcard;"
#else
#define CONFIG_ENV_IS_IN_MMC            /* read U-Boot command from MMC by default */
#define CONFIG_BOOTCOMMAND              "run bootcmd_android;"
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#endif

#if defined CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET               (8 * SZ_64K)
#endif

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK                264000000
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

#if defined(CONFIG_ANDROID_SUPPORT)
#include "mx6qmf0300android_common.h"
#endif
#endif                         /* __MX6QMF0300_COMMON_CONFIG_H */
