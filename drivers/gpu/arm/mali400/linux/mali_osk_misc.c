/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2014, 2016-2018 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file mali_osk_misc.c
 * Implementation of the OS abstraction layer for the kernel device driver
 */
#include <linux/kernel.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,12,0)
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif
#include <asm/cacheflush.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include "mali_osk.h"

#if !defined(CONFIG_MALI_QUIET)
void _mali_osk_dbgmsg(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vprintk(fmt, args);
	va_end(args);
}
#endif /* !defined(CONFIG_MALI_QUIET) */

u32 _mali_osk_snprintf(char *buf, u32 size, const char *fmt, ...)
{
	int res;
	va_list args;
	va_start(args, fmt);

	res = vscnprintf(buf, (size_t)size, fmt, args);

	va_end(args);
	return res;
}

#include <linux/delay.h>	/* mdelay */

void _mali_osk_abort(void)
{
	/* make a simple fault by dereferencing a NULL pointer */
	#if 1
	dump_stack();
	*(int *)0 = 0;
	#else /* temp test */
	#if 0
	dump_stack();
	*(int *)0xF = 0;
	#else
	/*while(1)*/
	{
		mdelay(1000);
	}
	#endif
	#endif
}

void _mali_osk_break(void)
{
	/*printk("_mali_osk_break from %s\n", pfunc);*/ //temp test
	_mali_osk_abort();
}

u32 _mali_osk_get_pid(void)
{
	/* Thread group ID is the process ID on Linux */
	return (u32)current->tgid;
}

char *_mali_osk_get_comm(void)
{
	return (char *)current->comm;
}


u32 _mali_osk_get_tid(void)
{
	/* pid is actually identifying the thread on Linux */
	u32 tid = current->pid;

	/* If the pid is 0 the core was idle.  Instead of returning 0 we return a special number
	 * identifying which core we are on. */
	if (0 == tid) {
		tid = -(1 + raw_smp_processor_id());
	}

	return tid;
}
