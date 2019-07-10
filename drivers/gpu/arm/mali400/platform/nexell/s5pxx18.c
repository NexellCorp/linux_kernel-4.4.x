/*
 * Copyright (C) 2010, 2012-2016 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file s5pxx18.c
 * Platform specific Mali driver functions for:
 * - Nexell s5p6818 platforms with ARM CortexA53 8 cores.
 * - Nexell s5p4418 platforms with ARM CortexA9 4 cores.
 */
 
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/pm.h>
#include "mali_kernel_linux.h"
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif
#include <asm/io.h>
#include <linux/mali/mali_utgard.h>
#include "mali_kernel_common.h"
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#ifdef CONFIG_MALI_DT /* org */
#include <linux/reset.h>
#else 
#endif
#include <linux/clk.h>

#ifdef CONFIG_MALI_DT /* org */
#include <linux/soc/nexell/cpufreq.h>
#else
#endif
#include <linux/pm_qos.h>
#include "s5pxx18_core_scaling.h"
#include "mali_executor.h"

#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
#include <linux/devfreq_cooling.h>
#include <linux/thermal.h>
#endif

#ifdef CONFIG_MALI_DT
#ifdef CONFIG_MALI_PLATFORM_S5P6818
#include <dt-bindings/tieoff/s5p6818-tieoff.h>
#include <soc/nexell/tieoff.h>
#endif
#endif

#ifndef CONFIG_MALI_DT
static void mali_platform_device_release(struct device *device);

#if defined(CONFIG_ARCH_VEXPRESS)

#if defined(CONFIG_ARM64)
/* Juno + Mali-450 MP6 in V7 FPGA */
static struct resource mali_gpu_resources_m450_mp6[] = {
	MALI_GPU_RESOURCES_MALI450_MP6_PMU(0x6F040000, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200)
};

static struct resource mali_gpu_resources_m470_mp4[] = {
	MALI_GPU_RESOURCES_MALI470_MP4_PMU(0x6F040000, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200)
};

static struct resource mali_gpu_resources_m470_mp3[] = {
	MALI_GPU_RESOURCES_MALI470_MP3_PMU(0x6F040000, 200, 200, 200, 200, 200, 200, 200, 200, 200)
};

static struct resource mali_gpu_resources_m470_mp2[] = {
	MALI_GPU_RESOURCES_MALI470_MP2_PMU(0x6F040000, 200, 200, 200, 200, 200, 200, 200)
};

static struct resource mali_gpu_resources_m470_mp1[] = {
	MALI_GPU_RESOURCES_MALI470_MP1_PMU(0x6F040000, 200, 200, 200, 200, 200)
};

#else
static struct resource mali_gpu_resources_m450_mp8[] = {
	MALI_GPU_RESOURCES_MALI450_MP8_PMU(0xFC040000, -1, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 68)
};

static struct resource mali_gpu_resources_m450_mp6[] = {
	MALI_GPU_RESOURCES_MALI450_MP6_PMU(0xFC040000, -1, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 68)
};

static struct resource mali_gpu_resources_m450_mp4[] = {
	MALI_GPU_RESOURCES_MALI450_MP4_PMU(0xFC040000, -1, 70, 70, 70, 70, 70, 70, 70, 70, 70, 68)
};

static struct resource mali_gpu_resources_m470_mp4[] = {
	MALI_GPU_RESOURCES_MALI470_MP4_PMU(0xFC040000, -1, 70, 70, 70, 70, 70, 70, 70, 70, 70, 68)
};
#endif /* CONFIG_ARM64 */

#elif defined(CONFIG_ARCH_REALVIEW)

static struct resource mali_gpu_resources_m300[] = {
	MALI_GPU_RESOURCES_MALI300_PMU(0xC0000000, -1, -1, -1, -1)
};

static struct resource mali_gpu_resources_m400_mp1[] = {
	MALI_GPU_RESOURCES_MALI400_MP1_PMU(0xC0000000, -1, -1, -1, -1)
};

static struct resource mali_gpu_resources_m400_mp2[] = {
	MALI_GPU_RESOURCES_MALI400_MP2_PMU(0xC0000000, -1, -1, -1, -1, -1, -1)
};

#endif
#endif

static int mali_core_scaling_enable = 0;
static struct clk *clk_mali;
static struct reset_control *rst_mali;
#ifdef CONFIG_ARM_S5Pxx18_DEVFREQ
static bool nexell_qos_added;
static struct pm_qos_request nexell_gpu_qos;
static int bus_clk_step;
static struct delayed_work qos_work;
#endif

void mali_gpu_utilization_callback(struct mali_gpu_utilization_data *data);

#ifdef CONFIG_MALI_DT
#ifdef CONFIG_MALI_PLATFORM_S5P6818
static void s5p6818_mali_axibus_lpi_exit(void)
{
	/* Set PBUS CSYSREQ to High */
	nx_tieoff_set(NX_TIEOFF_Inst_VR_PBUS_AXILPI_S0_CSYSREQ, 1);

	/* Set MBUS CSYSREQ to High */
	nx_tieoff_set(NX_TIEOFF_Inst_VR_MBUS_AXILPI_S0_CSYSREQ, 1);
}

static void s5p6818_mali_axibus_lpi_enter(void)
{
	/* Set PBUS LPI CSYSREQ to Low */
	nx_tieoff_set(NX_TIEOFF_Inst_VR_PBUS_AXILPI_S0_CSYSREQ, 0);

	/* Set MBUS LPI CSYSREQ to Low */
	nx_tieoff_set(NX_TIEOFF_Inst_VR_MBUS_AXILPI_S0_CSYSREQ, 0);
}
#endif
#endif

#if !defined(CONFIG_MALI_DT)
#ifdef CONFIG_MALI_PLATFORM_S5P4418
/* NEXELL_FEATURE_PORTING */
//added by nexell
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/delay.h>	/* mdelay */

#define PHY_BASEADDR_VR_PMU					(0xC0070000 + 0x2000) //~ + 0x10
#define PHY_BASEADDR_VR_PMU_REG_SIZE		0x10
#define PHY_BASEADDR_PMU_ISOLATE			(0xC0010D00) //~ + 0x10
#define PHY_BASEADDR_PMU_ISOLATE_REG_SIZE	0x10
#define PHY_BASEADDR_POWER_GATE				(0xC0010800) //~ + 0x4
#define PHY_BASEADDR_POWER_GATE_REG_SIZE	0x4
#define PHY_BASEADDR_CLOCK_GATE				(0xC00C3000) //~ + 0x4
#define PHY_BASEADDR_CLOCK_GATE_REG_SIZE	0x4
#define PHY_BASEADDR_RESET  				(0xC0012000) //~ + 0xC
#define PHY_BASEADDR_RESET_REG_SIZE  		0xC
#ifdef CONFIG_MALI_PLATFORM_S5P6818
#define PHY_BASEADDR_LPI_ACTIVE				0xC001120C
#define PHY_BASEADDR_LPI_ACTIVE_REG_SIZE	0x4
#define PHY_BASEADDR_LPI_REQ				0xC0011114
#define PHY_BASEADDR_LPI_REQ_REG_SIZE		0x4
#endif

enum
{
	PHY_BASEADDR_VR_PMU_IDX,
	PHY_BASEADDR_PMU_ISOLATE_IDX,
	PHY_BASEADDR_POWER_GATE_IDX,
	PHY_BASEADDR_CLOCK_GATE_IDX,
	PHY_BASEADDR_RESET_IDX,
#ifdef CONFIG_MALI_PLATFORM_S5P6818
	PHY_BASEADDR_LPI_ACTIVE_IDX,
	PHY_BASEADDR_LPI_REQ_IDX,
#endif	
	PHY_BASEADDR_IDX_MAX
};
typedef struct
{
	unsigned int reg_addr;
	unsigned int reg_size;
}VR_REG_MAPS;

static VR_REG_MAPS __g_VRRegPhysMaps[PHY_BASEADDR_IDX_MAX] = {
	{PHY_BASEADDR_VR_PMU, 		PHY_BASEADDR_VR_PMU_REG_SIZE},
	{PHY_BASEADDR_PMU_ISOLATE, 	PHY_BASEADDR_PMU_ISOLATE_REG_SIZE},
	{PHY_BASEADDR_POWER_GATE, 	PHY_BASEADDR_POWER_GATE_REG_SIZE},
	{PHY_BASEADDR_CLOCK_GATE, 	PHY_BASEADDR_CLOCK_GATE_REG_SIZE},
	{PHY_BASEADDR_RESET, 		PHY_BASEADDR_RESET_REG_SIZE},
#ifdef CONFIG_MALI_PLATFORM_S5P6818
	{PHY_BASEADDR_LPI_ACTIVE,	PHY_BASEADDR_LPI_ACTIVE_REG_SIZE},
	{PHY_BASEADDR_LPI_REQ,		PHY_BASEADDR_LPI_REQ_REG_SIZE},
#endif	
};
static void* __gp_VRRegVirtMaps[PHY_BASEADDR_IDX_MAX];

#define POWER_DELAY_MS	100
#if 0
#define VR_DBG printk
#define VR_PM_DBG printk
#define VR_IOCTL_DBG printk
#else
#define VR_DBG
#define VR_PM_DBG //PM_DBGOUT
#define VR_IOCTL_DBG
#endif

void nx_vr_make_reg_virt_maps(void)
{
	int i;
	if(!__gp_VRRegVirtMaps[i])
	{
		for(i = 0 ; i < PHY_BASEADDR_IDX_MAX ; i++)
		{
			__gp_VRRegVirtMaps[i] = ioremap_nocache(__g_VRRegPhysMaps[i].reg_addr, __g_VRRegPhysMaps[i].reg_size);
			if(!__gp_VRRegVirtMaps[i])
			{
				MALI_PRINT(("ERROR! can't run 'ioremap_nocache()'\n"));
				break;
			}
		}
	}
}

void nx_vr_release_reg_virt_maps(void)
{
	int i;
	for(i = 0 ; i < PHY_BASEADDR_IDX_MAX ; i++)
	{
		iounmap(__gp_VRRegVirtMaps[i]);
	}
}

#if defined( CONFIG_MALI_PLATFORM_S5P4418 )
void nx_vr_power_down_all_s5p4418(void)
{
	void* virt_addr_page;
	u32 offset;

	//reset
	offset = 8;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_RESET_IDX];
	{
		unsigned int temp32 = ioread32(((u8*)virt_addr_page + offset));
		unsigned int bit_mask = 1<<1; //65th
		VR_DBG("setting Reset VR addr(0x%x)\n", (u8*)virt_addr_page + offset);

		temp32 &= ~bit_mask;
		iowrite32(temp32, ((u8*)virt_addr_page + offset));
	}

	//clk disalbe
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_CLOCK_GATE_IDX];
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting ClockGen, set 0\n");
		iowrite32(read_val & ~0x3, ((u8*)virt_addr_page + offset));
	}

	//ready
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_POWER_GATE_IDX];
	{
		VR_DBG("setting PHY_BASEADDR_POWER_GATE, set 1\n");
		iowrite32(0x1, ((u8*)virt_addr_page + offset));
	}

	//enable ISolate
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE, set 0\n");
		iowrite32(read_val & ~1, ((u8*)virt_addr_page + offset));
	}

	//pre charge down
	offset = 4;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE+4, set 1\n");
		iowrite32(read_val | 1, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//powerdown
	offset = 8;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE+8, set 1\n");
		iowrite32(read_val | 1, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//wait ack
	VR_DBG("read PHY_BASEADDR_PMU_ISOLATE + 0xC\n");
	offset = 0xC;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	do{
		unsigned int powerUpAck = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("Wait Power Down Ack(powerUpAck=0x%08x)\n", powerUpAck);
		if( (powerUpAck & 0x1) )
			break;
		VR_DBG("Wait Power Down Ack(powerUpAck=0x%08x)\n", powerUpAck);
	}while(1);
}

void nx_vr_power_up_all_s5p4418(void)
{
	void* virt_addr_page;
	u32 offset;

	//ready
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_POWER_GATE_IDX];
	{
		VR_DBG("setting PHY_BASEADDR_POWER_GATE, set 1\n");
		iowrite32(0x1, ((u8*)virt_addr_page + offset));
	}

	//pre charge up
	offset = 4;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE+4, set 0\n");
		iowrite32(read_val & ~0x1, ((u8*)virt_addr_page + offset));
	}

	//power up
	offset = 8;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE+8, set 0\n");
		iowrite32(read_val & ~0x1, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//disable ISolate
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE, set 1\n");
		iowrite32(read_val | 1, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//wait ack
	VR_DBG("read PHY_BASEADDR_PMU_ISOLATE + 0xC\n");
	offset = 0xC;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	do{
		unsigned int powerUpAck = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("Wait Power UP Ack(powerUpAck=0x%08x)\n", powerUpAck);
		if( !(powerUpAck & 0x1) )
			break;
		VR_DBG("Wait Power UP Ack(powerUpAck=0x%08x)\n", powerUpAck);
	}while(1);

	//clk enable
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_CLOCK_GATE_IDX];
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting ClockGen, set 1\n");
		iowrite32(0x3 | read_val, ((u8*)virt_addr_page + offset));
	}

	//reset release
	offset = 8;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_RESET_IDX];
	{
		unsigned int temp32 = ioread32(((u8*)virt_addr_page + offset));
		unsigned int bit_mask = 1<<1; //65th
		VR_DBG("setting Reset VR addr(0x%x)\n", (u8*)virt_addr_page + offset);
		temp32 |= bit_mask;
		iowrite32(temp32, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//mask vr400 PMU interrupt
	offset = 0xC;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_VR_PMU_IDX];
	{
		VR_PM_DBG("mask PMU INT, addr(0x%x)\n", (u8*)virt_addr_page + offset);
		iowrite32(0x0, ((u8*)virt_addr_page + offset));
	}

	//power up vr400
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_VR_PMU_IDX];
	{
		VR_DBG("setting PHY_BASEADDR_VR_PMU addr(0x%x)\n", (u8*)virt_addr_page + offset);
		iowrite32(0xF/*GP, L2C, PP0, PP1*/, ((u8*)virt_addr_page + offset));
	}
}
#endif
#if defined(CONFIG_MALI_PLATFORM_S5P6818)
static void nx_vr_power_down_enter_reset_s5p6818(void)
{
	void* virt_addr_page;
	u32 read32;
	u32 offset;

	//=========================
	// [ mali PBUS ]
	//=========================
	// wait until LPI ACTIVE HIGH
	//=========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_ACTIVE_IDX];
	//printk("LPI ACTIVE HIGH waitting start...\n");
	do{
		read32 = ioread32((u8*)virt_addr_page + offset);
		if( (read32>>12) & 0x01 )
			break;
	}while(1);
	//printk("LPI ACTIVE HIGH waitting done\n");

	//==========================
	// LPI REQ, Set LOW
	//==========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_REQ_IDX];
	read32 = ioread32((u8*)virt_addr_page + offset);
	read32 = read32 & (~(1<<2)); // CSYSREQ LOW
	iowrite32(read32, ((u8*)virt_addr_page + offset));

	//==========================
	// wait until LPI ACK LOW
	//==========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_ACTIVE_IDX];
	//printk("LPI ACK LOW waitting start...\n");
	do{
		read32 = ioread32((u8*)virt_addr_page + offset);
		if( !((read32>>13) & 0x01) )
			break;
	}while(1);
	//printk("LPI ACK LOW waitting done\n");

	//=========================
	// [ mali MBUS ]
	//=========================
	// wait until LPI ACTIVE HIGH
	//=========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_ACTIVE_IDX];
	//printk("LPI ACTIVE HIGH waitting start...\n");
	do{
		read32 = ioread32((u8*)virt_addr_page + offset);
		if( (read32>>20) & 0x01 )
			break;
	}while(1);
	//printk("LPI ACTIVE HIGH waitting done\n");

	//==========================
	// LPI REQ, Set LOW
	//==========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_REQ_IDX];
	read32 = ioread32((u8*)virt_addr_page + offset);
	read32 = read32 & (~(1<<1)); // CSYSREQ LOW
	iowrite32(read32, ((u8*)virt_addr_page + offset));

	//==========================
	// wait until LPI ACK LOW
	//==========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_ACTIVE_IDX];
	//printk("LPI ACTIVE LOW waitting start...\n");
	do{
		read32 = ioread32((u8*)virt_addr_page + offset);
		if( !((read32>>21) & 0x01) )
			break;
	}while(1);
	//printk("LPI ACTIVE LOW waitting done\n");
}

static void nx_vr_power_up_leave_reset_s5p6818(void)
{
	void* virt_addr_page;
	u32 read32;
	u32 offset;

	//==========================
	// [ mali PBUS ]
	//==========================
	// LPI REQ, Set HIGH
	//==========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_REQ_IDX];
	read32 = ioread32((u8*)virt_addr_page + offset);
	read32 = read32 | (1<<2); // CSYSREQ HIGH
	iowrite32(read32, ((u8*)virt_addr_page + offset));

	//==========================
	// wait until LPI ACK HIGH
	//==========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_ACTIVE_IDX];
	//printk("LPI ACK HIGH waitting start...\n");
	do{
		read32 = ioread32((u8*)virt_addr_page + offset);
		if( (read32>>13) & 0x01 )
			break;
	}while(1);
	//printk("LPI ACK HIGH waitting done\n");

	//==========================
	// [ mali MBUS ]
	//==========================
	// LPI REQ, Set HIGH
	//==========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_REQ_IDX];
	read32 = ioread32((u8*)virt_addr_page + offset);
	read32 = read32 | (1<<1); // CSYSREQ HIGH
	iowrite32(read32, ((u8*)virt_addr_page + offset));

	//==========================
	// wait until LPI ACK HIGH
	//==========================
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_LPI_ACTIVE_IDX];
	//printk("LPI ACK ACK waitting start...\n");
	do{
		read32 = ioread32((u8*)virt_addr_page + offset);
		if( (read32>>21) & 0x01 )
			break;
	}while(1);
	//printk("LPI ACK ACK waitting done\n");

}

void nx_vr_power_down_all_s5p6818(void)
{
	void* virt_addr_page;
	u32 map_size;
	u32 offset;

	//reset
	nx_vr_power_down_enter_reset_s5p6818();
	offset = 8;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_RESET_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int temp32 = ioread32(((u8*)virt_addr_page + offset));
		unsigned int bit_mask = 1<<1; //65th
		VR_DBG("setting Reset VR addr(0x%x)\n", (u8*)virt_addr_page + offset);
		temp32 &= ~bit_mask;
		iowrite32(temp32, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//clk disalbe
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_CLOCK_GATE_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting ClockGen, set 0\n");
		iowrite32(read_val & ~0x3, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//ready
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_POWER_GATE_IDX];
	if (NULL != virt_addr_page)
	{
		VR_DBG("setting PHY_BASEADDR_POWER_GATE, set 1\n");
		iowrite32(0x1, ((u8*)virt_addr_page + offset));
	}

	//enable ISolate
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE, set 0\n");
		iowrite32(read_val & ~1, ((u8*)virt_addr_page + offset));
	}

	//pre charge down
	offset = 4;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE+4, set 1\n");
		iowrite32(read_val | 1, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//powerdown
	offset = 8;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE+8, set 1\n");
		iowrite32(read_val | 1, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//wait ack
	VR_DBG("read PHY_BASEADDR_PMU_ISOLATE + 0xC\n");
	offset = 0xC;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	do{
		unsigned int powerUpAck = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("Wait Power Down Ack(powerUpAck=0x%08x)\n", powerUpAck);
		if( (powerUpAck & 0x1) )
			break;
		VR_DBG("Wait Power Down Ack(powerUpAck=0x%08x)\n", powerUpAck);
	}while(1);
}

void nx_vr_power_up_all_s5p6818(void)
{
	void* virt_addr_page;
	u32 offset;

	//ready
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_POWER_GATE_IDX];
	if (NULL != virt_addr_page)
	{
		VR_DBG("setting PHY_BASEADDR_POWER_GATE, set 1\n");
		iowrite32(0x1, ((u8*)virt_addr_page + offset));
	}

	//pre charge up
	offset = 4;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE+4, set 0\n");
		iowrite32(read_val & ~0x1, ((u8*)virt_addr_page + offset));
	}

	//power up
	offset = 8;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE+8, set 0\n");
		iowrite32(read_val & ~0x1, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//disable ISolate
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting PHY_BASEADDR_PMU_ISOLATE, set 1\n");
		iowrite32(read_val | 1, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//wait ack
	VR_DBG("read PHY_BASEADDR_PMU_ISOLATE + 0xC\n");
	offset = 0xC;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_PMU_ISOLATE_IDX];
	do{
		unsigned int powerUpAck = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("Wait Power UP Ack(powerUpAck=0x%08x)\n", powerUpAck);
		if( !(powerUpAck & 0x1) )
			break;
		VR_DBG("Wait Power UP Ack(powerUpAck=0x%08x)\n", powerUpAck);
	}while(1);

	//clk enable
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_CLOCK_GATE_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int read_val = ioread32((u8*)virt_addr_page + offset);
		VR_DBG("setting ClockGen, set 1\n");
		iowrite32(0x3 | read_val, ((u8*)virt_addr_page + offset));
	}

	//reset
	offset = 8;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_RESET_IDX];
	if (NULL != virt_addr_page)
	{
		unsigned int temp32 = ioread32(((u8*)virt_addr_page + offset));
		unsigned int bit_mask = 1<<1; //65th
		VR_DBG("setting Reset VR addr(0x%x)\n", (u8*)virt_addr_page + offset);

		//reset leave
		nx_vr_power_up_leave_reset_s5p6818();
		temp32 |= bit_mask;
		iowrite32(temp32, ((u8*)virt_addr_page + offset));
	}
	mdelay(1);

	//mask vr400 PMU interrupt
	offset = 0xC;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_VR_PMU_IDX];
	if (NULL != virt_addr_page)
	{
		VR_PM_DBG("mask PMU INT, addr(0x%x)\n", (u8*)virt_addr_page + offset);
		iowrite32(0x0, ((u8*)virt_addr_page + offset));
	}

	//power up vr400
	offset = 0;
	virt_addr_page = __gp_VRRegVirtMaps[PHY_BASEADDR_VR_PMU_IDX];
	if (NULL != virt_addr_page)
	{
		VR_DBG("setting PHY_BASEADDR_VR_PMU addr(0x%x)\n", (u8*)virt_addr_page + offset);
		iowrite32(0x3F/*GP, L2C, PP0, PP1, PP2, PP3*/, ((u8*)virt_addr_page + offset));
	}
}
#endif


void nexell_platform_resume(struct device *dev)
{
	nx_vr_make_reg_virt_maps();
	#if defined(CONFIG_MALI_PLATFORM_S5P4418) 
	nx_vr_power_up_all_s5p4418();
	#elif defined(CONFIG_MALI_PLATFORM_S5P6818) 
	nx_vr_power_up_all_s5p6818();
	#endif
}

void nexell_platform_suspend(struct device *dev)
{
	#if defined(CONFIG_MALI_PLATFORM_S5P4418) 
	nx_vr_power_down_all_s5p4418();
	#elif defined(CONFIG_MALI_PLATFORM_S5P6818) 
	nx_vr_power_down_all_s5p6818();
	#endif
	nx_vr_release_reg_virt_maps();
}
#endif

#else /* org */ /* !CONFIG_MALI_DT */

void nexell_platform_resume(struct device *dev)
{
	clk_prepare_enable(clk_mali);
	reset_control_reset(rst_mali);
#ifdef CONFIG_MALI_PLATFORM_S5P6818
	s5p6818_mali_axibus_lpi_exit();
#endif
}

void nexell_platform_suspend(struct device *dev)
{
	if (rst_mali) {
#ifdef CONFIG_MALI_PLATFORM_S5P6818
		s5p6818_mali_axibus_lpi_enter();
#endif
		reset_control_assert(rst_mali);
	}

	if (clk_mali)
		clk_disable_unprepare(clk_mali);
}
#endif /* !CONFIG_MALI_DT */

#ifdef CONFIG_ARM_S5Pxx18_DEVFREQ
static struct mali_gpu_clk_item gpu_clocks[] = {
	{
		.clock = 100,	/* NX_BUS_CLK_IDLE_KHZ */
	}, {
		.clock = 200,	/* Fake clock */
	}, {
		.clock = 300,	/* Fake clock */
	}, {
		.clock = 400,	/* NX_BUS_CLK_GPU_KHZ */
	}
};

struct mali_gpu_clock gpu_clock = {
	.item = gpu_clocks,
	.num_of_steps = ARRAY_SIZE(gpu_clocks),
};

static void
nexell_gpu_qos_work_handler(struct work_struct *work)
{
	u32 clk_khz = gpu_clocks[bus_clk_step].clock * 1000;

	if (clk_khz != NX_BUS_CLK_IDLE_KHZ)
		clk_khz = NX_BUS_CLK_GPU_KHZ;

	if (!nexell_qos_added) {
		pm_qos_add_request(&nexell_gpu_qos, PM_QOS_BUS_THROUGHPUT,
				clk_khz);
		nexell_qos_added = true;
	} else {
		pm_qos_update_request(&nexell_gpu_qos, clk_khz);
	}
}

static void nexell_get_clock_info(struct mali_gpu_clock **data)
{
	*data = &gpu_clock;
}

static int nexell_get_freq(void)
{
	return bus_clk_step;
}

static int nexell_set_freq(int setting_clock_step)
{
	if (bus_clk_step != setting_clock_step) {
		bus_clk_step = setting_clock_step;

		INIT_DELAYED_WORK(&qos_work, nexell_gpu_qos_work_handler);
		queue_delayed_work(system_power_efficient_wq, &qos_work, 0);
	}

	return 0;
}
#endif

static struct mali_gpu_device_data mali_gpu_data = {
	.max_job_runtime = 60000, /* 60 seconds */

	/* Mali OS memory limit */
	#if 0 /* NEXELL_FEATURE_PORTING */
	.shared_mem_size = 2 * 1024 * 1024 * 1024, /* 2GB */
	#endif

	/* Some framebuffer drivers get the framebuffer dynamically, such as through GEM,
	* in which the memory resource can't be predicted in advance.
	*/
	.fb_start = 0x0,
	.fb_size = 0xFFFFF000,
	.control_interval = 1000, /* 1000ms */
	.utilization_callback = mali_gpu_utilization_callback,
#ifdef CONFIG_ARM_S5Pxx18_DEVFREQ
	.get_clock_info = nexell_get_clock_info,
	.get_freq = nexell_get_freq,
	.set_freq = nexell_set_freq,
#endif
	.secure_mode_init = NULL,
	.secure_mode_deinit = NULL,
	.gpu_reset_and_secure_mode_enable = NULL,
	.gpu_reset_and_secure_mode_disable = NULL,
	.platform_suspend = nexell_platform_suspend,
	.platform_resume = nexell_platform_resume,
};

#ifndef CONFIG_MALI_DT
static struct platform_device mali_gpu_device = {
	.name = MALI_GPU_NAME_UTGARD,
	.id = 0,
	.dev.release = mali_platform_device_release,
	.dev.dma_mask = &mali_gpu_device.dev.coherent_dma_mask,
	.dev.coherent_dma_mask = DMA_BIT_MASK(32),

	.dev.platform_data = &mali_gpu_data,
};

int mali_platform_device_register(void)
{
	int err = -1;
	int num_pp_cores = 0;

	MALI_DEBUG_PRINT(4, ("mali_platform_device_register() called\n"));

	/* Detect present Mali GPU and connect the correct resources to the device */

	/* Register the platform device */
	err = platform_device_register(&mali_gpu_device);
	if (0 == err) {
#ifdef CONFIG_PM_RUNTIME
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
		pm_runtime_set_autosuspend_delay(&(mali_gpu_device.dev), 1000);
		pm_runtime_use_autosuspend(&(mali_gpu_device.dev));
#endif
		pm_runtime_enable(&(mali_gpu_device.dev));
#endif
		MALI_DEBUG_ASSERT(0 < num_pp_cores);
		mali_core_scaling_init(num_pp_cores);

		return 0;
	}

	return err;
}

void mali_platform_device_unregister(void)
{
	MALI_DEBUG_PRINT(4, ("mali_platform_device_unregister() called\n"));

	mali_core_scaling_term();
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&(mali_gpu_device.dev));
#endif
	platform_device_unregister(&mali_gpu_device);

	platform_device_put(&mali_gpu_device);

#if defined(CONFIG_ARCH_REALVIEW)
	mali_write_phys(0xC0010020, 0x9); /* Restore default (legacy) memory mapping */
#endif
}

static void mali_platform_device_release(struct device *device)
{
	MALI_DEBUG_PRINT(4, ("mali_platform_device_release() called\n"));
}

#else /* CONFIG_MALI_DT */
int mali_platform_device_init(struct platform_device *device)
{
	int num_pp_cores = 2;
	int err = -1;
	struct device *dev = &device->dev;

	clk_mali = devm_clk_get(dev, "clk_mali");
	if (IS_ERR_OR_NULL(clk_mali)) {
		dev_err(dev, "failed to get mali clock\n");
		return -ENODEV;
	}

	clk_prepare_enable(clk_mali);

	rst_mali = devm_reset_control_get(dev, "vr-reset");

	if (IS_ERR(rst_mali)) {
		dev_err(dev, "failed to get reset_control\n");
		return -EINVAL;
	}

	reset_control_reset(rst_mali);
#ifdef CONFIG_MALI_PLATFORM_S5P6818
	s5p6818_mali_axibus_lpi_exit();
#endif

#ifdef CONFIG_MALI_PLATFORM_S5P6818
	num_pp_cores = 4;
#endif
	/* After kernel 3.15 device tree will default set dev
	 * related parameters in of_platform_device_create_pdata.
	 * But kernel changes from version to version,
	 * For example 3.10 didn't include device->dev.dma_mask parameter setting,
	 * if we didn't include here will cause dma_mapping error,
	 * but in kernel 3.15 it include  device->dev.dma_mask parameter setting,
	 * so it's better to set must need paramter by DDK itself.
	 */
	if (!device->dev.dma_mask)
		device->dev.dma_mask = &device->dev.coherent_dma_mask;
#ifndef CONFIG_ARM64
	device->dev.archdata.dma_ops = &arm_dma_ops;
#endif

	err = platform_device_add_data(device, &mali_gpu_data, sizeof(mali_gpu_data));

	if (0 == err) {
#ifdef CONFIG_PM_RUNTIME
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
		pm_runtime_set_autosuspend_delay(&(device->dev), 1000);
		pm_runtime_use_autosuspend(&(device->dev));
#endif
		pm_runtime_enable(&(device->dev));
#endif
		MALI_DEBUG_ASSERT(0 < num_pp_cores);
		mali_core_scaling_init(num_pp_cores);
	}

#if defined(CONFIG_MALI_DEVFREQ) && defined(CONFIG_DEVFREQ_THERMAL)
	/* Get thermal zone */
	gpu_tz = thermal_zone_get_zone_by_name("soc_thermal");
	if (IS_ERR(gpu_tz)) {
		MALI_DEBUG_PRINT(2, ("Error getting gpu thermal zone (%ld), not yet ready?\n",
				     PTR_ERR(gpu_tz)));
		gpu_tz = NULL;

		err =  -EPROBE_DEFER;
	}
#endif

	return err;
}

int mali_platform_device_deinit(struct platform_device *device)
{
	MALI_IGNORE(device);

	MALI_DEBUG_PRINT(4, ("mali_platform_device_deinit() called\n"));

	mali_core_scaling_term();

#ifdef CONFIG_ARM_S5Pxx18_DEVFREQ
	if (nexell_qos_added) {
		pm_qos_remove_request(&nexell_gpu_qos);
		nexell_qos_added = false;
	}
#endif

	if (rst_mali) {
#ifdef CONFIG_MALI_PLATFORM_S5P6818
		s5p6818_mali_axibus_lpi_enter();
#endif
		reset_control_assert(rst_mali);
	}

	if (clk_mali)
		clk_disable_unprepare(clk_mali);

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&(device->dev));
#endif

	return 0;
}
#endif /* CONFIG_MALI_DT */

static int param_set_core_scaling(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if (1 == mali_core_scaling_enable) {
		mali_core_scaling_sync(mali_executor_get_num_cores_enabled());
	}
	return ret;
}

static struct kernel_param_ops param_ops_core_scaling = {
	.set = param_set_core_scaling,
	.get = param_get_int,
};

module_param_cb(mali_core_scaling_enable, &param_ops_core_scaling, &mali_core_scaling_enable, 0644);
MODULE_PARM_DESC(mali_core_scaling_enable, "1 means to enable core scaling policy, 0 means to disable core scaling policy");

void mali_gpu_utilization_callback(struct mali_gpu_utilization_data *data)
{
	if (1 == mali_core_scaling_enable) {
		mali_core_scaling_update(data);
	}
}
