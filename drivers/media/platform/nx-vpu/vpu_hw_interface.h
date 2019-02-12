// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell VPU driver
 * Copyright (c) 2019 Sungwon Jo <doriya@nexell.co.kr>
 */

#ifndef __VPU_HW_INTERFACE_H__
#define	__VPU_HW_INTERFACE_H__
asdf
#include "nx_vpu_config.h"
#include "regdefine.h"
#include "nx_vpu_api.h"

/*----------------------------------------------------------------------------
 *		Register Interface
 */

/* Offset Based Register Access for VPU */
void VpuWriteReg(uint32_t offset, uint32_t value);
uint32_t VpuReadReg(uint32_t offset);
void VpuWriteRegNoMsg(uint32_t offset, uint32_t value);
uint32_t VpuReadRegNoMsg(uint32_t offset);

/* Direct Register Access API */
void WriteReg32(uint32_t *address, uint32_t value);
uint32_t ReadReg32(uint32_t *address);

void InitVpuRegister(void *virAddr);
uint32_t *GetVpuRegBase(void);

/*----------------------------------------------------------------------------
 *		Host Command Interface
 */
void VpuBitIssueCommand(struct nx_vpu_codec_inst *inst, enum nx_vpu_cmd cmd);

#endif		/* __VPU_HW_INTERFACE_H__ */
