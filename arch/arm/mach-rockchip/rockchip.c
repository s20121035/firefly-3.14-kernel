/*
 * Device Tree support for Rockchip SoCs
 *
 * Copyright (c) 2013 MundoReader S.L.
 * Author: Heiko Stuebner <heiko@sntech.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irqchip.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>
#include "core.h"
#include "pm.h"
#include "loader.h"
#include "cpu_axi.h"
#include <linux/rockchip/common.h>
#include <linux/rockchip/iomap.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/cru.h>
#include <linux/rockchip/dvfs.h>
#include <linux/rockchip/grf.h>
#include <linux/rockchip/pmu.h>
#define RK3288_DEVICE(name) \
        { \
                .virtual        = (unsigned long) RK_##name##_VIRT, \
                .pfn            = __phys_to_pfn(RK3288_##name##_PHYS), \
                .length         = RK3288_##name##_SIZE, \
                .type           = MT_DEVICE, \
        }

#define RK3288_SERVICE_DEVICE(name) \
        RK_DEVICE(RK3288_SERVICE_##name##_VIRT, RK3288_SERVICE_##name##_PHYS, RK3288_SERVICE_##name##_SIZE)

#define RK3288_IMEM_VIRT (RK_BOOTRAM_VIRT + SZ_32K)
#define RK3288_TIMER7_VIRT (RK_TIMER_VIRT + 0x20)


static struct map_desc rk3288_io_desc[] __initdata = {
        RK3288_DEVICE(CRU),
        RK3288_DEVICE(GRF),
        RK3288_DEVICE(SGRF),
        RK3288_DEVICE(PMU),
        RK3288_DEVICE(ROM),
        RK3288_DEVICE(EFUSE),
        RK3288_SERVICE_DEVICE(CORE),
        RK3288_SERVICE_DEVICE(DMAC),
        RK3288_SERVICE_DEVICE(GPU),
        RK3288_SERVICE_DEVICE(PERI),
        RK3288_SERVICE_DEVICE(VIO),
        RK3288_SERVICE_DEVICE(VIDEO),
        RK3288_SERVICE_DEVICE(HEVC),
        RK3288_SERVICE_DEVICE(BUS),
        RK_DEVICE(RK_DDR_VIRT, RK3288_DDR_PCTL0_PHYS, RK3288_DDR_PCTL_SIZE),
        RK_DEVICE(RK_DDR_VIRT + RK3288_DDR_PCTL_SIZE, RK3288_DDR_PUBL0_PHYS, RK3288_DDR_PUBL_SIZE),
        RK_DEVICE(RK_DDR_VIRT + RK3288_DDR_PCTL_SIZE + RK3288_DDR_PUBL_SIZE, RK3288_DDR_PCTL1_PHYS, RK3288_DDR_PCTL_SIZE),
        RK_DEVICE(RK_DDR_VIRT + 2 * RK3288_DDR_PCTL_SIZE + RK3288_DDR_PUBL_SIZE, RK3288_DDR_PUBL1_PHYS, RK3288_DDR_PUBL_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(0), RK3288_GPIO0_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(1), RK3288_GPIO1_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(2), RK3288_GPIO2_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(3), RK3288_GPIO3_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(4), RK3288_GPIO4_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(5), RK3288_GPIO5_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(6), RK3288_GPIO6_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(7), RK3288_GPIO7_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_GPIO_VIRT(8), RK3288_GPIO8_PHYS, RK3288_GPIO_SIZE),
        RK_DEVICE(RK_DEBUG_UART_VIRT, RK3288_UART_DBG_PHYS, RK3288_UART_SIZE),
        RK_DEVICE(RK_GIC_VIRT, RK3288_GIC_DIST_PHYS, RK3288_GIC_DIST_SIZE),
        RK_DEVICE(RK_GIC_VIRT + RK3288_GIC_DIST_SIZE, RK3288_GIC_CPU_PHYS, RK3288_GIC_CPU_SIZE),
        RK_DEVICE(RK_BOOTRAM_VIRT, RK3288_BOOTRAM_PHYS, RK3288_BOOTRAM_SIZE),
        RK_DEVICE(RK3288_IMEM_VIRT, RK3288_IMEM_PHYS, SZ_4K),
        RK_DEVICE(RK_TIMER_VIRT, RK3288_TIMER6_PHYS, RK3288_TIMER_SIZE),
};


static int boot_mode;

static inline const char *boot_mode_name(u32 mode)
{
        switch (mode) {
        case BOOT_MODE_NORMAL: return "NORMAL";
        case BOOT_MODE_FACTORY2: return "FACTORY2";
        case BOOT_MODE_RECOVERY: return "RECOVERY";
        case BOOT_MODE_CHARGE: return "CHARGE";
        case BOOT_MODE_POWER_TEST: return "POWER_TEST";
        case BOOT_MODE_OFFMODE_CHARGING: return "OFFMODE_CHARGING";
        case BOOT_MODE_REBOOT: return "REBOOT";
        case BOOT_MODE_PANIC: return "PANIC";
        case BOOT_MODE_WATCHDOG: return "WATCHDOG";
        case BOOT_MODE_TSADC: return "TSADC";
	case BOOT_MODE_RAMFS: return "RAMFS";
        default: return "";
        }
}

static inline const char *boot_flag_name(u32 flag)
{
        flag -= SYS_KERNRL_REBOOT_FLAG;
        switch (flag) {
        case BOOT_NORMAL: return "NORMAL";
        case BOOT_LOADER: return "LOADER";
        case BOOT_MASKROM: return "MASKROM";
        case BOOT_RECOVER: return "RECOVER";
        case BOOT_NORECOVER: return "NORECOVER";
        case BOOT_SECONDOS: return "SECONDOS";
        case BOOT_WIPEDATA: return "WIPEDATA";
        case BOOT_WIPEALL: return "WIPEALL";
        case BOOT_CHECKIMG: return "CHECKIMG";
        case BOOT_FASTBOOT: return "FASTBOOT";
        case BOOT_CHARGING: return "CHARGING";
	case BOOT_RAMFS: return "RAMFS";
        default: return "";
        }
}

static void usb_uart_init(void)
{
        u32 soc_status2;

        writel_relaxed(0x00c00000, RK_GRF_VIRT + RK3288_GRF_UOC0_CON3);
        soc_status2 = (readl_relaxed(RK_GRF_VIRT + RK3288_GRF_SOC_STATUS2));

#ifdef CONFIG_RK_USB_UART
        if (!(soc_status2 & (1<<14)) && (soc_status2 & (1<<17))) {
                /* software control usb phy enable */
                writel_relaxed(0x00040004, RK_GRF_VIRT + RK3288_GRF_UOC0_CON2);
                /* usb phy enter suspend */
                writel_relaxed(0x003f002a, RK_GRF_VIRT + RK3288_GRF_UOC0_CON3);
                writel_relaxed(0x00c000c0, RK_GRF_VIRT + RK3288_GRF_UOC0_CON3);
        }
#endif
}


void __init rockchip_boot_mode_init(u32 flag, u32 mode)
{
        boot_mode = mode;
        if (mode || ((flag & 0xff) && ((flag & 0xffffff00) == SYS_KERNRL_REBOOT_FLAG)))
                printk("Boot mode: %s (%d) flag: %s (0x%08x)\n", boot_mode_name(mode), mode, boot_flag_name(flag), flag);
        //atomic_notifier_chain_register(&panic_notifier_list, &panic_block);
}


void rockchip_restart_get_boot_mode(const char *cmd, u32 *flag, u32 *mode)
{
        *flag = SYS_LOADER_REBOOT_FLAG + BOOT_NORMAL;
        *mode = BOOT_MODE_REBOOT;

        if (cmd) {
                if (!strcmp(cmd, "loader") || !strcmp(cmd, "bootloader"))
                        *flag = SYS_LOADER_REBOOT_FLAG + BOOT_LOADER;
                else if(!strcmp(cmd, "recovery"))
                        *flag = SYS_LOADER_REBOOT_FLAG + BOOT_RECOVER;
                else if (!strcmp(cmd, "fastboot"))
                        *flag = SYS_LOADER_REBOOT_FLAG + BOOT_FASTBOOT;
                else if (!strcmp(cmd, "charge")) {
                        *flag = SYS_LOADER_REBOOT_FLAG + BOOT_CHARGING;
                        *mode = BOOT_MODE_CHARGE;
		}else if(!strcmp(cmd, "ramfs")){
                        *flag = SYS_LOADER_REBOOT_FLAG + BOOT_RAMFS;
                        *mode = BOOT_MODE_RAMFS;
                }
        }
}
static void __init rk3288_boot_mode_init(void)
{
        u32 flag = readl_relaxed(RK_PMU_VIRT + RK3288_PMU_SYS_REG0);
        u32 mode = readl_relaxed(RK_PMU_VIRT + RK3288_PMU_SYS_REG1);
        u32 rst_st = readl_relaxed(RK_CRU_VIRT + RK3288_CRU_GLB_RST_ST);
	 flag = SYS_LOADER_REBOOT_FLAG + BOOT_RAMFS;
         mode = BOOT_MODE_RAMFS;

        if (flag == (SYS_KERNRL_REBOOT_FLAG | BOOT_RECOVER))
                mode = BOOT_MODE_RECOVERY;
        if (rst_st & ((1 << 4) | (1 << 5)))
                mode = BOOT_MODE_WATCHDOG;
        else if (rst_st & ((1 << 2) | (1 << 3)))
                mode = BOOT_MODE_TSADC;
        rockchip_boot_mode_init(flag, mode);
}

static void __init rk3288_dt_map_io(void)
{
        u32 v;

        rockchip_soc_id = ROCKCHIP_SOC_RK3288;

        iotable_init(rk3288_io_desc, ARRAY_SIZE(rk3288_io_desc));
        debug_ll_io_init();
        usb_uart_init();

        /* pmu reset by second global soft reset */
        v = readl_relaxed(RK_CRU_VIRT + RK3288_CRU_GLB_RST_CON);
        v &= ~(3 << 2);
        v |= 1 << 2;
        writel_relaxed(v, RK_CRU_VIRT + RK3288_CRU_GLB_RST_CON);

        /* rkpwm is used instead of old pwm */
        writel_relaxed(0x00010001, RK_GRF_VIRT + RK3288_GRF_SOC_CON2);

        /* disable address remap */
        writel_relaxed(0x08000000, RK_SGRF_VIRT + RK3288_SGRF_SOC_CON0);

        /* enable timer7 for core */
        writel_relaxed(0, RK3288_TIMER7_VIRT + 0x10);
        dsb();
        writel_relaxed(0xFFFFFFFF, RK3288_TIMER7_VIRT + 0x00);
        writel_relaxed(0xFFFFFFFF, RK3288_TIMER7_VIRT + 0x04);
        dsb();
        writel_relaxed(1, RK3288_TIMER7_VIRT + 0x10);
        dsb();

        /* power up/down GPU domain wait 1us */
        writel_relaxed(24, RK_PMU_VIRT + RK3288_PMU_GPU_PWRDWN_CNT);
        writel_relaxed(24, RK_PMU_VIRT + RK3288_PMU_GPU_PWRUP_CNT);

        rk3288_boot_mode_init();
}


static void __init rockchip_dt_init(void)
{
	l2x0_of_init(0, ~0UL);
	rockchip_suspend_init();
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	platform_device_register_simple("cpufreq-cpu0", 0, NULL, 0);
}

extern struct ion_platform_data ion_pdata;
extern void __init ion_reserve(struct ion_platform_data *data);
extern int __init rockchip_ion_find_heap(unsigned long node,
				const char *uname, int depth, void *data);
void __init rockchip_ion_reserve(void)
{
#ifdef CONFIG_ION_ROCKCHIP
	printk("%s\n", __func__);
	of_scan_flat_dt(rockchip_ion_find_heap, (void*)&ion_pdata);
	ion_reserve(&ion_pdata);
#endif
}
static void __init rockchip_memory_init(void)
{
	memblock_reserve(0xfe000000, 0x1000000);
	/* reserve memory for ION */
	rockchip_ion_reserve();
}

static void rk3288_restart(char mode, const char *cmd)
{
        u32 boot_flag, boot_mode;
	
        rockchip_restart_get_boot_mode(cmd, &boot_flag, &boot_mode);
        writel_relaxed(boot_flag, RK_PMU_VIRT + RK3288_PMU_SYS_REG0);   // for loader
        writel_relaxed(boot_mode, RK_PMU_VIRT + RK3288_PMU_SYS_REG1);   // for linux
        dsb();
	        /* pll enter slow mode */
        writel_relaxed(0xf3030000, RK_CRU_VIRT + RK3288_CRU_MODE_CON);
        dsb();
        writel_relaxed(0xeca8, RK_CRU_VIRT + RK3288_CRU_GLB_SRST_SND_VALUE);
        dsb();


}

static const char * const rockchip_board_dt_compat[] = {
	"rockchip,rk2928",
	"rockchip,rk3066a",
	"rockchip,rk3066b",
	"rockchip,rk3188",
	"rockchip,rk3288",
	NULL,
};

DT_MACHINE_START(ROCKCHIP_DT, "Rockchip (Device Tree)")
	.map_io		= rk3288_dt_map_io,
	.init_machine	= rockchip_dt_init,
	.dt_compat	= rockchip_board_dt_compat,
	.reserve        = rockchip_memory_init,
	.restart 	= rk3288_restart,
MACHINE_END
