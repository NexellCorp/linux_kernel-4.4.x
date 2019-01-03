// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell VPU driver
 * Copyright (c) 2019 Sungwon Jo <doriya@nexell.co.kr>
 */

#ifndef __NX_VPU_GDI_H__
#define	__NX_VPU_GDI_H__

int SetTiledMapType(int mapType, int stride, int interleave);
int ConfigEncSecAXI(int codStd, struct sec_axi_info *sa, int width,
	int height, uint32_t sramAddr, uint32_t sramSize);
int ConfigDecSecAXI(int codStd, struct sec_axi_info *sa, int width,
	int height, uint32_t sramAddr, uint32_t sramSize);
unsigned int MaverickCache2Config(int decoder, int interleave, int bypass,
	int burst, int merge, int mapType, int wayshape);

#endif		/*__NX_VPU_GDI_H__ */
