// SPDX-License-Identifier: GPL-2.0+
/*
 * Nexell VPU driver
 * Copyright (c) 2019 Sungwon Jo <doriya@nexell.co.kr>
 */

#ifndef __NX_ALLOC_MEM_H__
#define __NX_ALLOC_MEM_H__

#include <linux/types.h>
#include <linux/kernel.h>

#ifndef CONFIG_ARCH_NXP3220_COMMON
#define USE_JPEG
#define USE_ENCODER
#define USE_DEPRECATED_APIS
#define USE_DEPRECATED_SYSCALL
#define USE_DEPRECATED_STRUCTURE
#endif

/* #define USE_ION_MEMORY */

#define FUNC_MSG		0

#ifndef NX_DTAG
#define NX_DTAG			"[DRV|VPU]"
#endif

#define NX_Trace(...)		do {					\
					printk(NX_DTAG			\
						" "__VA_ARGS__);	\
				} while (0)

#define NX_DbgMsg(COND, ...)	do {					\
					if (COND) {			\
						printk(NX_DTAG		\
							" "__VA_ARGS__);\
					}				\
				} while (0)

#define NX_ErrMsg(...)  	do {					\
					printk(NX_DTAG			\
						" ""file: %s, line: %d",\
						__FILE__, __LINE__ );	\
					printk(NX_DTAG			\
						" "__VA_ARGS__);	\
				} while (0)

#if FUNC_MSG
#define FUNC_IN()		do {					\
					printk("%s(), %d IN.\n", 	\
						__FUNCTION__, __LINE__);\
				} while (0)

#define FUNC_OUT()		do {					\
					printk("%s(), %d OUT.\n", 	\
						__FUNCTION__, __LINE__);\
				} while (0)
#else
#define FUNC_IN()		do {} while (0)
#define FUNC_OUT()		do {} while (0)
#endif

#ifndef ALIGN
#define ALIGN(X, N)		((X+N-1) & (~(N-1)))
#endif

#define NX_MAX_PLANES		4

/*
 * struct nx_memory_info - nexell private memory type
 */
struct nx_memory_info {
	void *fd;
	int32_t size;
	int32_t align;
	void *virAddr;
	dma_addr_t phyAddr;
};

/*
 * struct nx_vid_memory_info - nexell private video memory type
 */
struct nx_vid_memory_info {
	int32_t width;
	int32_t height;
	int32_t align;
	int32_t planes;
	uint32_t format;			/* Pixel Format(N/A) */

	void *fd[NX_MAX_PLANES];
	int32_t size[NX_MAX_PLANES];		/* Each plane's size. */
	int32_t stride[NX_MAX_PLANES];		/* Each plane's stride */
	void *virAddr[NX_MAX_PLANES];
	uint32_t phyAddr[NX_MAX_PLANES];
	struct nx_memory_info *mem[NX_MAX_PLANES];
};

/*
 * nexell private memory allocator
 */
struct nx_memory_info *nx_alloc_memory(void *drv, int32_t size, int32_t align);
void nx_free_memory(struct nx_memory_info *mem);

/*
 * video specific allocator wrapper
 */
struct nx_vid_memory_info *nx_alloc_frame_memory(void *drv, int32_t width,
	int32_t height, int32_t planes, uint32_t format, int32_t align);
void nx_free_frame_memory(struct nx_vid_memory_info *vid);

void NX_DrvMemset(void *ptr, int value, int size);
void NX_DrvMemcpy(void *dst, void *src, int size);

void DrvMSleep(unsigned int mSeconds);
void DrvUSleep(unsigned int uSeconds);

#endif		/* __NX_ALLOC_MEM_H__ */
