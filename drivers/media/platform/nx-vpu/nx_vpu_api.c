// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell VPU driver
 * Copyright (c) 2019 Sungwon Jo <doriya@nexell.co.kr>
 */

#ifndef UNUSED
#define UNUSED(p) ((void)(p))
#endif

#include <linux/delay.h>
#include <linux/io.h>

#ifdef CONFIG_ARCH_S5P6818
#include <dt-bindings/tieoff/s5p6818-tieoff.h>
#endif

#ifdef CONFIG_ARCH_NXP3220_COMMON
#include "blackbird_v3.6.25.h"
#else
#include "blackbird_v2.3.10.h"
#endif
#include "vpu_hw_interface.h"		/* Register Access */
#include "nx_vpu_api.h"

#define	DBG_POWER			0
#define	DBG_CLOCK			0
#define	INFO_MSG			0

/*--------------------------------------------------------------------------- */
/* Static Global Variables */
static int gstIsInitialized;
static int gstIsVPUOn;
static unsigned int gstVpuRegStore[64];

#ifndef CONFIG_ARCH_NXP3220_COMMON
static uint32_t *gstCodaClockEnRegVir;
static uint32_t *gstIsolateBase;
static uint32_t *gstAliveBase;
#endif

static struct nx_vpu_codec_inst gstVpuInstance[NX_MAX_VPU_INSTANCE];

/*--------------------------------------------------------------------------- */
/*  Define Static Functions */
static unsigned int VPU_IsBusy(void);

/*----------------------------------------------------------------------------
 *		Nexell Specific VPU Hardware On/Off Logic
 *--------------------------------------------------------------------------- */
#define VPU_ALIVEGATE_REG		0xC0010800
#define VPU_NISOLATE_REG		0xC0010D00
#define CODA960CLKENB_REG		0xC00C7000

#define	POWER_PMU_VPU_MASK		0x00000002

#ifdef CONFIG_ARCH_S5P6818
/*	Async XUI Power Down
 *
 *	Step 1. Waiting until CACTIVE to High
 *	Step 2. Set CSYSREQ to Low
 *	Step 3. Waiting until CSYSACK to Low
 */
static void NX_ASYNCXUI_PowerDown(void)
{
	int32_t tmpVal;

	FUNC_IN();

	/* Apply To Async XUI 0 */

	/* Step 1. Waiting until CACTIVE to High */
	do {
		tmpVal = nx_tieoff_get(NX_TIEOFF_Inst_CODA960_ASYNCXIU0_CACTIVE_S);
	} while (!tmpVal);

	/* Step 2. Set CSYSREQ to Low */
	nx_tieoff_set(NX_TIEOFF_Inst_ASYNCXUI0_CSYSREQ, 0);

	/*Step 3. Waiting until CSYSACK to Low */
	do {
		tmpVal = nx_tieoff_get(NX_TIEOFF_Inst_CODA960_ASYNCXIU0_CSYSACK_S);
	} while (tmpVal);

	/* Apply To Async XUI 1 */

	/* Step 1. Waiting until CACTIVE to High */
	do {
		tmpVal = nx_tieoff_get(NX_TIEOFF_Inst_CODA960_ASYNCXIU1_CACTIVE_S);
	} while (!tmpVal);

	/* Step 2. Set CSYSREQ to Low */
	nx_tieoff_set(NX_TIEOFF_Inst_ASYNCXUI1_CSYSREQ, 0);

	/* Step 3. Waiting until CSYSACK to Low */
	do {
		tmpVal = nx_tieoff_get(NX_TIEOFF_Inst_CODA960_ASYNCXIU1_CSYSACK_S);
	} while (tmpVal);


	FUNC_OUT();
}

/*	Async XUI Power Up
 *
 *	Step 1. Set CSYSREQ to High
 *	Step 2. Waiting until CSYSACK to High
 */
static void NX_ASYNCXUI_PowerUp(void)
{
	int32_t tmpVal;

	FUNC_IN();

	/* Apply To Async XUI 0 */

	/* Step 1. Set CSYSREQ to High */
	nx_tieoff_set(NX_TIEOFF_Inst_ASYNCXUI0_CSYSREQ, 1);

	/* Step 2. Waiting until CSYSACK to High */
	do {
		tmpVal = nx_tieoff_get(NX_TIEOFF_Inst_CODA960_ASYNCXIU0_CSYSACK_S);
	} while (!tmpVal);

	/* Apply To Async XUI 1 */

	/* Step 1. Set CSYSREQ to High */
	nx_tieoff_set(NX_TIEOFF_Inst_ASYNCXUI1_CSYSREQ, 1);

	/* Step 2. Waiting until CSYSACK to High */
	do {
		tmpVal = nx_tieoff_get(NX_TIEOFF_Inst_CODA960_ASYNCXIU1_CSYSACK_S);
	} while (!tmpVal);

	FUNC_OUT();
}
#endif

void NX_VPU_HwOn(void *dev, void *pVpuBaseAddr)
{
#ifndef CONFIG_ARCH_NXP3220_COMMON
	uint32_t tmpVal;
	uint32_t *pNPreCharge, *pNPowerUp, *pNPowerAck;
#endif

	NX_DbgMsg(DBG_POWER, "NX_VPU_HwOn() ++\n");

	/* Already Power On */
	if (gstIsVPUOn)
		return;

	vpu_soc_peri_reset_enter(dev);

	InitVpuRegister(pVpuBaseAddr);

#ifndef CONFIG_ARCH_NXP3220_COMMON
	/* Initialize ISolate Register's */
	pNPreCharge  = gstIsolateBase + 1;
	pNPowerUp    = gstIsolateBase + 2;
	pNPowerAck   = gstIsolateBase + 3;

	NX_DbgMsg(INFO_MSG, "====================================\n");
	NX_DbgMsg(INFO_MSG, "pVpuBaseAddr = %p\n", pVpuBaseAddr);
	NX_DbgMsg(INFO_MSG, "pIsolateBase = %p\n", gstIsolateBase);
	NX_DbgMsg(INFO_MSG, "pNPreCharge  = %p\n", pNPreCharge);
	NX_DbgMsg(INFO_MSG, "pNPowerUp    = %p\n", pNPowerUp);
	NX_DbgMsg(INFO_MSG, "pNPowerAck   = %p\n", pNPowerAck);
	NX_DbgMsg(INFO_MSG, "====================================\n");

	WriteReg32(gstAliveBase,  0x3);

	/* Enable PreCharge */
	tmpVal = ReadReg32(pNPreCharge);
	tmpVal &= (~POWER_PMU_VPU_MASK);
	WriteReg32(pNPreCharge, tmpVal);

	/* Enable Power On */
	tmpVal = ReadReg32(pNPowerUp);
	tmpVal &= (~POWER_PMU_VPU_MASK);
	WriteReg32(pNPowerUp, tmpVal);

	/* Disable ISolate */
	tmpVal = ReadReg32(gstIsolateBase);
	tmpVal |= (POWER_PMU_VPU_MASK);
	WriteReg32(gstIsolateBase, tmpVal);

	mdelay(1);
#endif

	NX_VPU_ClockOn(dev);

#ifdef CONFIG_ARCH_S5P6818
	NX_ASYNCXUI_PowerUp();
#endif

	vpu_soc_peri_hw_on(dev);
	vpu_soc_peri_reset_exit(dev);

	gstIsVPUOn = 1;


	NX_DbgMsg(DBG_POWER, "NX_VPU_HwOn() --\n");
}

void NX_VPU_HWOff(void *dev)
{
	FUNC_IN();

	if (gstIsVPUOn) {
#ifndef CONFIG_ARCH_NXP3220_COMMON
		unsigned int tmpVal;
		uint32_t *pNPreCharge, *pNPowerUp, *pNPowerAck;
#endif

#ifdef CONFIG_ARCH_S5P6818
		NX_ASYNCXUI_PowerDown();
#endif
		/* H/W Reset */
		vpu_soc_peri_reset_enter(dev);

		NX_DbgMsg(DBG_POWER, "NX_VPU_HWOff() ++\n");

#ifndef CONFIG_ARCH_NXP3220_COMMON
		/* Initialize ISolate Register's */
		pNPreCharge = gstIsolateBase + 4;
		pNPowerUp = gstIsolateBase + 8;
		pNPowerAck = gstIsolateBase + 12;

		/* Enter Coda Reset State */
		WriteReg32(gstAliveBase,  0x3);

		/* Isolate VPU H/W */
		tmpVal = ReadReg32(gstIsolateBase);
		tmpVal &= (~POWER_PMU_VPU_MASK);
		WriteReg32(gstIsolateBase, tmpVal);

		/* Pre Charget Off */
		tmpVal = ReadReg32(pNPreCharge);
		tmpVal |= POWER_PMU_VPU_MASK;
		WriteReg32(pNPreCharge, tmpVal);

		/* Power Down */
		tmpVal = ReadReg32(pNPowerUp);
		tmpVal |= POWER_PMU_VPU_MASK;
		WriteReg32(pNPowerUp, tmpVal);

		/* Isolate VPU H/W */
		tmpVal = ReadReg32(gstIsolateBase);
		tmpVal &= (~POWER_PMU_VPU_MASK);
		WriteReg32(gstIsolateBase, tmpVal);
#endif

		vpu_soc_peri_hw_off(dev);
		gstIsVPUOn = 0;

		NX_DbgMsg(DBG_POWER, "NX_VPU_HWOff() --\n");
	}

	FUNC_OUT();
}

int NX_VPU_GetCurPowerState(void)
{
	return gstIsVPUOn;
}

void NX_VPU_ResetEnter(void *dev)
{
	FUNC_IN();
	vpu_soc_peri_reset_enter(dev);
	FUNC_OUT();
}

void NX_VPU_ResetRelease(void *dev)
{
	FUNC_IN();
	vpu_soc_peri_reset_exit(dev);
	FUNC_OUT();
}

void NX_VPU_ClockOn(void *dev)
{
	FUNC_IN();

#ifndef CONFIG_ARCH_NXP3220_COMMON
	WriteReg32(gstCodaClockEnRegVir, 0x0000000F);
#endif
	vpu_soc_peri_clock_on(dev);
	FUNC_OUT();
}

void NX_VPU_ClockOff(void *dev)
{
	FUNC_IN();
#ifndef CONFIG_ARCH_NXP3220_COMMON
	WriteReg32(gstCodaClockEnRegVir, 0x00000000);
#endif
	vpu_soc_peri_clock_off(dev);
	FUNC_OUT();
}

static unsigned int VPU_IsBusy(void)
{
	unsigned int ret = VpuReadRegNoMsg(BIT_BUSY_FLAG);

	return ret != 0;
}

static int VPU_WaitBusBusy(int mSeconds, unsigned int busyFlagReg)
{
	while (mSeconds > 0) {
		if (0x77 == VpuReadReg(busyFlagReg))
			return VPU_RET_OK;
		DrvMSleep(1);
		mSeconds--;
	}
	return VPU_RET_ERR_TIMEOUT;
}

int VPU_WaitVpuBusy(int mSeconds, unsigned int busyFlagReg)
{
	while (mSeconds > 0) {
		if (VpuReadRegNoMsg(busyFlagReg) == 0)
			return VPU_RET_OK;

		DrvMSleep(1);
		mSeconds--;
	}
	return VPU_RET_ERR_TIMEOUT;
}

void VPU_GetVersionInfo(struct nx_vpu_version *version)
{
	if (version) {
		VpuWriteReg(RET_FW_VER_NUM, 0);
		VpuWriteReg(BIT_WORK_BUF_ADDR, 0);
		VpuWriteReg(BIT_BUSY_FLAG, 1);
		VpuWriteReg(BIT_RUN_INDEX, 0);
		VpuWriteReg(BIT_RUN_COD_STD, 0);
		VpuWriteReg(BIT_RUN_AUX_STD, 0);
		VpuWriteReg(BIT_RUN_COMMAND, FIRMWARE_GET);
		if (VPU_RET_OK != VPU_WaitVpuBusy(VPU_BUSY_CHECK_TIMEOUT,
			BIT_BUSY_FLAG)) {
			NX_ErrMsg("Version Read Failed!!!\n");
			return;
		}

		version->firm_version = VpuReadReg(RET_FW_VER_NUM);
		version->firm_revision = VpuReadReg(RET_FW_CODE_REV);

		version->product_name = VpuReadReg(DBG_CONFIG_REPORT_0);
		version->product_version = VpuReadReg(DBG_CONFIG_REPORT_1);
		/*
		version-> = VpuRegread(DBG_CONFIG_REPORT_2);
		version-> = VpuRegread(DBG_CONFIG_REPORT_3);
		*/
		version->release_version = VpuReadReg(DBG_CONFIG_REPORT_4);
		version->config_date = VpuReadReg(DBG_CONFIG_REPORT_5);
		version->config_revision = VpuReadReg(DBG_CONFIG_REPORT_6);
		version->config_type = VpuReadReg(DBG_CONFIG_REPORT_7);
	}
}

void CheckVersion(void)
{
	struct  nx_vpu_version version;
	NX_DrvMemset(&version, 0x00, sizeof(version));

	VPU_GetVersionInfo(&version);

	NX_DbgMsg(INFO_MSG,
		"Firmware Version => "
		"project number: %x | "
		"version: %04d.%04d.%08d | "
		"revision: r%d\n",
		(version.firm_version>>16),
		(version.firm_version>>12)&0x0F,
		(version.firm_version>>8)&0x0F,
		(version.firm_version>>0)&0xFF,
		version.firm_revision);

	NX_DbgMsg(INFO_MSG,
		"Product Version => "
		"%c%c%c%c%04X\n",
		((version.product_name>>24)&0xFF),
		((version.product_name>>16)&0xFF),
		((version.product_name>> 8)&0xFF),
		((version.product_name>> 0)&0xFF),
		(version.product_version & 0xFFFF));

	NX_DbgMsg(INFO_MSG,
		"Release Version => %08X\n",
		version.release_version);

	NX_DbgMsg(INFO_MSG,
		"Config Date => %d\n",
		version.config_date);

	NX_DbgMsg(INFO_MSG,
		"Config Revision => %d\n",
		version.config_revision);

	NX_DbgMsg(INFO_MSG,
		"Config Type => %d\n",
		version.config_type);
}


/*--------------------------------------------------------------------------- */

/* VPU_SWReset
 * IN
 *    forcedReset : 1 if there is no need to waiting for BUS transaction,
 *                     0 for otherwise
 * OUT
 *    RetCode : RETCODE_FAILURE if failed to reset,
 *                RETCODE_SUCCESS for otherwise
 */

/* SW Reset command */
#define VPU_SW_RESET_BPU_CORE	0x008
#define VPU_SW_RESET_BPU_BUS	0x010
#define VPU_SW_RESET_VCE_CORE	0x020
#define VPU_SW_RESET_VCE_BUS	0x040
#define VPU_SW_RESET_GDI_CORE	0x080
#define VPU_SW_RESET_GDI_BUS	0x100

int VPU_SWReset(int resetMode)
{
	unsigned int cmd;

	if (resetMode == SW_RESET_SAFETY || resetMode == SW_RESET_ON_BOOT) {
		/* Waiting for completion of bus transaction */
		/* Step1 : No more request */
		/* no more request {3'b0,no_more_req_sec,3'b0,no_more_req} */
		VpuWriteReg(GDI_BUS_CTRL, 0x11);

		/* Step2 : Waiting for completion of bus transaction */
		/* while (VpuReadReg(coreIdx, GDI_BUS_STATUS) != 0x77) */
		if (VPU_WaitBusBusy(VPU_BUSY_CHECK_TIMEOUT, GDI_BUS_STATUS) ==
			VPU_RET_ERR_TIMEOUT) {
			VpuWriteReg(GDI_BUS_CTRL, 0x00);
			return VPU_RET_ERR_TIMEOUT;
		}

		/* Step3 : clear GDI_BUS_CTRL */
		VpuWriteReg(GDI_BUS_CTRL, 0x00);
	}

	cmd = 0;
	/* Software Reset Trigger */
	if (resetMode != SW_RESET_ON_BOOT)
		cmd =  VPU_SW_RESET_BPU_CORE | VPU_SW_RESET_BPU_BUS;
	cmd |= VPU_SW_RESET_VCE_CORE | VPU_SW_RESET_VCE_BUS;
	if (resetMode == SW_RESET_ON_BOOT)
		/* If you reset GDI, tiled map should be reconfigured */
		cmd |= VPU_SW_RESET_GDI_CORE | VPU_SW_RESET_GDI_BUS;
	VpuWriteReg(BIT_SW_RESET, cmd);

	/* wait until reset is done */
	if (VPU_WaitVpuBusy(VPU_BUSY_CHECK_TIMEOUT, BIT_SW_RESET_STATUS) ==
		VPU_RET_ERR_TIMEOUT) {
		VpuWriteReg(BIT_SW_RESET, 0x00);
		return VPU_RET_ERR_TIMEOUT;
	}

	VpuWriteReg(BIT_SW_RESET, 0);

	return VPU_RET_OK;
}

/*----------------------------------------------------------------------------
 *	VPU Initialization.
 *--------------------------------------------------------------------------- */
int NX_VpuInit(void *dev, void *baseAddr, void *firmVirAddr,
	uint32_t firmPhyAddr)
{
	enum nx_vpu_ret ret = VPU_RET_OK;
	int32_t i;
	uint32_t tmpData;
	uint32_t codeBufAddr, tmpBufAddr, paramBufAddr;

	if (gstIsInitialized)
		return VPU_RET_OK;

	codeBufAddr = firmPhyAddr;
	tmpBufAddr  = codeBufAddr + CODE_BUF_SIZE;
	paramBufAddr = tmpBufAddr + TEMP_BUF_SIZE;

	NX_VPU_HwOn(dev, baseAddr);

	/* if BIT processor is not running. */
	if (VpuReadReg(BIT_CUR_PC) == 0) {
		for (i = 0; i < 64; i++)
			VpuWriteReg(BIT_BASE + 0x100 + (i*4), 0x0);
	}

	VPU_SWReset(SW_RESET_ON_BOOT);

	{
		unsigned char *dst = (unsigned char *)firmVirAddr;

		for (i = 0; i < ARRAY_SIZE(bit_code) ; i += 4) {
			*dst++ = (unsigned char)(bit_code[i+3] >> 0);
			*dst++ = (unsigned char)(bit_code[i+3] >> 8);

			*dst++ = (unsigned char)(bit_code[i+2] >> 0);
			*dst++ = (unsigned char)(bit_code[i+2] >> 8);

			*dst++ = (unsigned char)(bit_code[i+1] >> 0);
			*dst++ = (unsigned char)(bit_code[i+1] >> 8);

			*dst++ = (unsigned char)(bit_code[i+0] >> 0);
			*dst++ = (unsigned char)(bit_code[i+0] >> 8);
		}
	}

	VpuWriteReg(BIT_INT_ENABLE, 0);
	VpuWriteReg(BIT_CODE_RUN, 0);

	for (i = 0 ; i < 2048 ; i++) {
		tmpData = bit_code[i];
		VpuWriteRegNoMsg(BIT_CODE_DOWN, (i<<16)|tmpData);
	}

	VpuWriteReg(BIT_PARA_BUF_ADDR, paramBufAddr);
	VpuWriteReg(BIT_CODE_BUF_ADDR, codeBufAddr);
	VpuWriteReg(BIT_TEMP_BUF_ADDR, tmpBufAddr);

	VpuWriteReg(BIT_BIT_STREAM_CTRL, VPU_STREAM_ENDIAN);

	/* Interleave bit position is modified */
	VpuWriteReg(BIT_FRAME_MEM_CTRL, CBCR_INTERLEAVE<<2|VPU_FRAME_ENDIAN);

	VpuWriteReg(BIT_BIT_STREAM_PARAM, 0);

	VpuWriteReg(BIT_AXI_SRAM_USE, 0);
	VpuWriteReg(BIT_INT_ENABLE, 0);
	VpuWriteReg(BIT_ROLLBACK_STATUS, 0);

	tmpData  = (1<<VPU_INT_BIT_BIT_BUF_FULL);
	tmpData |= (1<<VPU_INT_BIT_BIT_BUF_EMPTY);
	tmpData |= (1<<VPU_INT_BIT_DEC_MB_ROWS);
	tmpData |= (1<<VPU_INT_BIT_SEQ_INIT);
	tmpData |= (1<<VPU_INT_BIT_DEC_FIELD);
	tmpData |= (1<<VPU_INT_BIT_PIC_RUN);
	VpuWriteReg(BIT_INT_ENABLE, tmpData);
	VpuWriteReg(BIT_INT_CLEAR, 0x1);

#ifndef CONFIG_ARCH_NXP3220_COMMON
	VpuWriteReg(BIT_USE_NX_EXPND, USE_NX_EXPND);
#endif
	VpuWriteReg(BIT_BUSY_FLAG, 0x1);
	VpuWriteReg(BIT_CODE_RESET, 1);
	VpuWriteReg(BIT_CODE_RUN, 1);

	if (VPU_RET_OK != VPU_WaitVpuBusy(VPU_BUSY_CHECK_TIMEOUT,
		BIT_BUSY_FLAG)) {
		NX_ErrMsg("NX_VpuInit() Failed. Timeout(%d)\n",
			VPU_BUSY_CHECK_TIMEOUT);
		return ret;
	}

	CheckVersion();

	for (i = 0 ; i < NX_MAX_VPU_INSTANCE ; i++) {
		gstVpuInstance[i].inUse = 0;
		gstVpuInstance[i].paramPhyAddr = paramBufAddr;
		gstVpuInstance[i].paramVirAddr = (unsigned long)(firmVirAddr +
			CODE_BUF_SIZE + TEMP_BUF_SIZE);
		gstVpuInstance[i].paramBufSize = PARA_BUF_SIZE;
	}
	gstIsInitialized = 1;

	return ret;
}

int NX_VpuDeInit(void *dev)
{
	if (!gstIsInitialized) {
		NX_ErrMsg("VPU Already Denitialized!!!\n");
		return VPU_RET_ERR_INIT;
	}

	if (VPU_IsBusy()) {
		NX_ErrMsg("NX_VpuDeInit() failed. VPU_IsBusy!!!\n");
		return VPU_RET_BUSY;
	}

	NX_VPU_HWOff(dev);

	gstIsInitialized = 0;
	return VPU_RET_OK;
}

int NX_VpuSuspend(void *dev)
{
	int i;

	if (!gstIsInitialized)
		return VPU_RET_ERR_INIT;

	if (VPU_IsBusy())
		return VPU_RET_BUSY;
	for (i = 0 ; i < 64 ; i++)
		gstVpuRegStore[i] = VpuReadReg(BIT_BASE+0x100 + (i * 4));

	NX_VPU_HWOff(dev);

	return VPU_RET_OK;
}

int NX_VpuResume(void *dev, void *pVpuBaseAddr)
{
	int i;
	unsigned int value;

	if (!gstIsInitialized)
		return VPU_RET_ERR_INIT;

	NX_VPU_HwOn(dev, pVpuBaseAddr);

	VPU_SWReset(SW_RESET_ON_BOOT);

	for (i = 0 ; i < 64 ; i++)
		VpuWriteReg(BIT_BASE+0x100+(i * 4), gstVpuRegStore[i]);

	VpuWriteReg(BIT_CODE_RUN, 0);
	/* Bit Code */
	for (i = 0; i < 2048; i++) {
		value = bit_code[i];
		VpuWriteReg(BIT_CODE_DOWN, ((i << 16) | value));
	}

	VpuWriteReg(BIT_BUSY_FLAG, 1);
	VpuWriteReg(BIT_CODE_RESET, 1);
	VpuWriteReg(BIT_CODE_RUN, 1);

#ifndef CONFIG_ARCH_NXP3220_COMMON
	VpuWriteReg(BIT_USE_NX_EXPND, USE_NX_EXPND);
#endif

	if (VPU_RET_OK != VPU_WaitVpuBusy(VPU_BUSY_CHECK_TIMEOUT,
		BIT_BUSY_FLAG)) {
		NX_ErrMsg("NX_VpuResume() Failed. Timeout(%d)\n",
			VPU_BUSY_CHECK_TIMEOUT);
		return VPU_RET_ERR_TIMEOUT;
	}

	return VPU_RET_OK;
}

struct nx_vpu_codec_inst *NX_VpuGetInstance(int index)
{
	return &gstVpuInstance[index];
}

int NX_VpuIsInitialized(void)
{
	return gstIsInitialized;
}

int swap_endian(unsigned char *data, int len)
{
	unsigned int *p;
	unsigned int v1, v2, v3;
	int i;
	int swap = 0;

	p = (unsigned int *)data;

	for (i = 0; i < len/4; i += 2) {
		v1 = p[i];
		v2  = (v1 >> 24) & 0xFF;
		v2 |= ((v1 >> 16) & 0xFF) <<  8;
		v2 |= ((v1 >>  8) & 0xFF) << 16;
		v2 |= ((v1 >>  0) & 0xFF) << 24;
		v3 =  v2;
		v1  = p[i+1];
		v2  = (v1 >> 24) & 0xFF;
		v2 |= ((v1 >> 16) & 0xFF) <<  8;
		v2 |= ((v1 >>  8) & 0xFF) << 16;
		v2 |= ((v1 >>  0) & 0xFF) << 24;
		p[i]   =  v2;
		p[i+1] = v3;
	}

	return swap;
}

int NX_VpuParaInitialized(void *dev)
{
	gstIsInitialized = 0;
	gstIsVPUOn = 0;

#ifndef CONFIG_ARCH_NXP3220_COMMON
	gstCodaClockEnRegVir = (uint32_t *)devm_ioremap_nocache(dev,
		CODA960CLKENB_REG, 4);
	if (!gstCodaClockEnRegVir)
		return -1;

	gstIsolateBase = (uint32_t *)devm_ioremap_nocache(dev,
		VPU_NISOLATE_REG, 128);
	gstAliveBase = (uint32_t *)devm_ioremap_nocache(dev,
		VPU_ALIVEGATE_REG, 128);
	if (!gstIsolateBase || !gstAliveBase)
		return -1;
#endif

	return 0;
}
