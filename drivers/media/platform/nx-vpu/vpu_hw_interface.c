// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell VPU driver
 * Copyright (c) 2019 Sungwon Jo <doriya@nexell.co.kr>
 */

#include <asm/io.h>
#include "vpu_hw_interface.h"

#define DBG_VBS 0

static void *gstBaseAddr;

/*----------------------------------------------------------------------------
 *	Register Interface
 */
void VpuWriteReg(uint32_t offset, uint32_t value)
{
	uint32_t *addr = (uint32_t*)((void*)(gstBaseAddr+offset));
	writel(value, addr);
#if 1
	NX_DbgMsg(NX_REG_EN_MSG, "wr(0x%p, 0x%08x, 0x%08x)\n",
		addr, offset, value);
#else
	NX_DbgMsg(NX_REG_EN_MSG, "wr(0x%p, 0x%08x, 0x%08x) verify(0x%08x)%s\n",
		addr, offset, value, readl(addr),
		(value != readl(addr)) ? " Mismatch" : "");
#endif
}

uint32_t VpuReadReg(uint32_t offset)
{
	uint32_t *addr = (uint32_t*)((void*)(gstBaseAddr+offset));
	NX_DbgMsg(NX_REG_EN_MSG, "rd(0x%p, 0x%08x, 0x%08x)\n",
		addr, offset, readl(addr));
	return readl(addr);
}

void VpuWriteRegNoMsg(uint32_t offset, uint32_t value)
{
	uint32_t *addr = (uint32_t*)((void*)(gstBaseAddr+offset));
	writel(value, addr);
}

uint32_t VpuReadRegNoMsg(uint32_t offset)
{
	uint32_t *addr = (uint32_t*)((void*)(gstBaseAddr+offset));
	return readl(addr);
}

void WriteReg32(uint32_t *address, uint32_t value)
{
	writel(value, address);
}

uint32_t ReadReg32(uint32_t *address)
{
	return readl(address);
}

void InitVpuRegister(void *virAddr)
{
	gstBaseAddr = virAddr;
}

uint32_t *GetVpuRegBase(void)
{
	return gstBaseAddr;
}

/*----------------------------------------------------------------------------
 *		Host Command
 */
void VpuBitIssueCommand(struct nx_vpu_codec_inst *inst, enum nx_vpu_cmd cmd)
{
	NX_DbgMsg(DBG_VBS, "VpuBitIssueCommand : cmd = %d, address=0x%llx, ",
		cmd, inst->instBufPhyAddr);
	NX_DbgMsg(DBG_VBS, "instIndex=%d, codecMode=%d, auxMode=%d\n",
		inst->instIndex, inst->codecMode, inst->auxMode);

	VpuWriteReg(BIT_WORK_BUF_ADDR, (uint32_t)inst->instBufPhyAddr);
	VpuWriteReg(BIT_BUSY_FLAG, 1);
	VpuWriteReg(BIT_RUN_INDEX, inst->instIndex);
	VpuWriteReg(BIT_RUN_COD_STD, inst->codecMode);
	VpuWriteReg(BIT_RUN_AUX_STD, inst->auxMode);
	VpuWriteReg(BIT_RUN_COMMAND, cmd);
}
