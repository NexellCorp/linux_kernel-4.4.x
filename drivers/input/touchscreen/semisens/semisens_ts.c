/****************************************************************
 *
 * semisens_ts.c : I2C Touchscreen device driver
 *
 * Copyright (c) 2012 SEMISENS Co.,Ltd
 *      http://www.semisens.com
 *
 ****************************************************************/
/*
 * Version: 1.0.8
 *
 * History
 * 1.0.0, 20140821, first release
 * 1.0.1, 20140922, sample platform removed
 * 1.0.2, 20141014, tegra(LG optimus 2x) sample code added, 
 *                  pointer or touchscreen can be set in probe,
 *                  some print log changed.
 * 1.0.3, 20141022, charger info, x/y rotation, probe wakeup added 
 * 1.0.4, 20141119, TSC debug info thread added 
 * 1.0.5, 20141125, Power Mgmt functions revised 
 * 1.0.6, 20141210, BTN_TOUCH event added when touch on/off for android 4.4.4 
 * 1.0.7, 20150122, pinctrl is added 
 * 1.0.8, 20150126, the name of member "disabled" in struct touch is changed to "enabled",
 * 					some log are changed, the timing of enabling irq is changed.
 * 1.0.9, 20150204, power sample func. modified.
 * 1.0.10, 20150520, proximity added, h/w reset scenario modified. 
 */
#include <linux/version.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <asm/unaligned.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

//----------------------------------------------
#if defined(CONFIG_HAS_EARLYSUSPEND)
	#include <linux/wakelock.h>
	#include <linux/earlysuspend.h>
	#include <linux/suspend.h>
#endif
#if defined(CONFIG_PM_RUNTIME)
	#include <linux/pm_runtime.h>
#endif

//----------------------------------------------
#include <linux/input/mt.h>
#include <linux/input/semisens_ts.h>

//----------------------------------------------
//#include <mach/gpio.h>
//#include <mach/irqs.h>

//----------------------------------------------
#define KNOCK_ON_MODE		0	/* enable/disable double tap or gesture wake-up feature, before enabling it, you MUST contact your FAE. */
#define PROXIMITY_MODE		0	/* enable/disable proximity feature, before enabling it, you MUST contact your FAE. */
#define FW_UPGRADE_ENABLE	1	/* enable/disable TSC f/w upgrade feature, before enabling it, you MUST contact your FAE. */
#define CHARGER_INFO_ENABLE	0	/* get/set charger attach/detach info */
#define I2C_RETRY_COUNT		3	/* I2C Retry Count */
#define XY_CHANGE_ENABLE	0	/* reports exchanged x/y coordinate when the device is rotated clockwise 270 degrees */ 
#define PROBE_WAKEUP_ENABLE	1	/* wake up before TSC probe */
#define PROBE_FAIL_CHECK	0	/* if TSC probe failed, unload device driver */

/* pinctrl funcs are used or not */
#define PINCTRL_ENABLE	0

/* if enabled, touchscreen will operate as pointer device on Android */
#define TOUCH_DEVICE_TYPE_POINTER	0 

/* for some samsung models, input open/close is called when system is resumed/suspended */
#define INPUT_PM_ENABLE	0

/* To make sure of reset, after H/W reset, do s/w reset */ 
#define TOUCH_SW_RESET_ENABLE	1	

#define SEMISENS_TS_NATIVE_INTERFACE /* interface to debug semisens TSC */ 
//----------------------------------------------

//----------------------------------------------
/* Debugging Macro */
#define DEBUG_TOUCH			0	/* TSC full log can be displayed */
#define DEBUG_TOUCH_KEY		0	/* TSC key log can be displayed */
#define DEBUG_TSC_INFO		0	/* TSC Info can be displayed to console */
#define DEBUG_PRINT			0	/* TSC debug can be displayed to console */


#define DBG_TOUCH(fmt, ...) \
	do { if(DEBUG_TOUCH) { \
			printk(KERN_INFO "%s:%d:%s():" fmt, "semisens", __LINE__, __func__, ##__VA_ARGS__); \
	} } while (0)

#define DBG_TOUCH_KEY(fmt, ...) \
	do { if(DEBUG_TOUCH_KEY) { \
			printk(KERN_INFO "%s:%d:%s():" fmt, "semisens", __LINE__, __func__, ##__VA_ARGS__); \
	} } while (0)

#define DBG_PRINT(fmt, ...) \
	do { if(DEBUG_PRINT) { \
			printk(KERN_NOTICE "%s:%d:%s():" fmt, "semisens", __LINE__, __func__, ##__VA_ARGS__); \
	} } while (0)
//----------------------------------------------

//----------------------------------------------
#if FW_UPGRADE_ENABLE
#define TSC_EEPROM_PAGE_SIZE	64 
#define FW_UPGRADE_RETRY_COUNT	2
#define BUSY_CHECK_RETRY_COUNT	20
#define TSC_FLASH_FW_VER_POS	0xCA3F	
#define TSC_FILE_FW_VER_POS		SWAP_16BITS(TSC_FLASH_FW_VER_POS)

#define TSC_FW_ARRAY_NAME		example_R00_fw
#include "semisens_ts_fw.h"
#endif /* End of #if FW_UPGRADE_ENABLE */
//----------------------------------------------

//----------------------------------------------
#if defined(SEMISENS_TS_NATIVE_INTERFACE)
#include <linux/miscdevice.h>
#include <linux/syscalls.h>

typedef struct {
	u32	addr;
	s16	*buf;
	u32	size;
} packet_t;

struct touch *g_ts;
s32 g_miscInitialize = 0;
static s32 semisens_ts_misc_probe(struct touch *ts);
static s32 semisens_ts_misc_open(struct inode *inode, struct file *file);
static long semisens_ts_misc_ioctl(struct file *file, u32 cmd, unsigned long arg);
static void semisens_ts_misc_remove(void);
#endif /* End of #if defined(SEMISENS_TS_NATIVE_INTERFACE) */
//----------------------------------------------

#if CHARGER_INFO_ENABLE
enum {
	E_TS_CHARGER_DETACHED = 0,
	E_TS_CHARGER_ATTACHED,
	E_TS_CHARGER_MAX
};

s16 g_chargerInfo = E_TS_CHARGER_DETACHED; 
#endif

enum {
	E_BOOTUP_DELAY_NORMAL,
	E_BOOTUP_DELAY_FAST,
};

s32 ts_keycode[] = {KEY_MENU, KEY_BACK, KEY_POWER};

#if KNOCK_ON_MODE
unsigned int g_knockOnFlag = 0;
#endif

#if defined(CONFIG_MACH_FORTUNA_CMCC)
	extern struct class *sec_class;						/* debugging filesystem for fortuna model */
#endif

//----------------------------------------------
// function prototype define
//----------------------------------------------
#if defined(CONFIG_HAS_EARLYSUSPEND)
	static void	touch_suspend(struct early_suspend *h);
	static void	touch_resume(struct early_suspend *h);
#endif

#if PINCTRL_ENABLE 
	static int pinctrl_configure(struct touch *ts, bool active);
#endif

#if defined(CONFIG_PM)
	static int touch_suspend_pm(struct device *dev);
	static int touch_resume_pm(struct device *dev);
#endif

irqreturn_t touch_irq(s32 irq, void *handle);
static void touch_work_q(struct work_struct *work);
static void touch_key_report(struct touch *ts, u8 button_data);
static void touch_report_protocol_a(struct touch *ts);
static void touch_report_protocol_b(struct touch *ts);
static void touch_event_clear(struct touch *ts);

static void touch_input_close(struct input_dev *input);
static s32 touch_input_open(struct input_dev *input);
static s32 touch_check_functionality(struct touch_pdata *pdata);
void touch_hw_reset(struct touch *ts, u8 delayMode);
s32 touch_info_display(struct touch *ts);

u8 semisens_ts_id_tracking(struct	touch *ts, u8 find_id);
s32 semisens_ts_i2c_read(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len);
s32 semisens_ts_i2c_write(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len);
void semisens_ts_enable(struct touch *ts);
void semisens_ts_disable(struct touch *ts);
s32 semisens_ts_early_probe(struct touch *ts);
s32 semisens_ts_probe(struct touch *ts);
void semisens_ts_work(struct touch *ts);

static s32 semisens_init_gpio(struct touch_pdata *pdata);
static s32 semisens_init_power(struct touch *ts);
#ifdef CONFIG_OF
	static s32 semisens_parse_dt(struct device *dev, struct touch_pdata *pdata);
#endif

static s32 ts_power_ctrl(struct touch *ts, s32 onOff, bool log_en);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0) 
	s32 touch_probe(struct i2c_client *client, const struct i2c_device_id *client_id);
	s32 touch_remove(struct i2c_client *client);
#else
	s32 touch_probe(struct i2c_client *client);
	s32 touch_remove(struct device *dev);
#endif

#if FW_UPGRADE_ENABLE
	static bool touch_upgrade_preparation(struct touch *ts);
	static bool touch_upgrade_start_sequence(struct touch *ts, u8 flashType, u16 opMode);
	static bool touch_upgrade_end_sequence(struct touch *ts, u8 flashType);
	static bool touch_upgrade_erase(struct touch *ts, u8 flashType);
	static bool touch_upgrade_program(struct touch *ts, const u8 *fwData, u32 fwSize, u8 flashType);
	static bool touch_upgrade_check(struct touch *ts, const u8 *fwData, u32 fwSize, u8 *flashType /* OUT */, bool forcedFlag);
	static bool touch_upgrade_fw(struct touch *ts, const u8 *fwData, u32 fwSize, u8 flashType); 
#endif

#if CHARGER_INFO_ENABLE
	extern struct tsp_callbacks *charger_callbacks;		/* for charger event */
	void semisens_register_callback(void *callback);	/* for charger event */
	void touch_charger_info_cb(struct tsp_callbacks *cb, bool status);
	void touch_charger_info_notify(struct touch *ts);
#endif
		
#if FW_UPGRADE_ENABLE
//----------------------------------------------
static bool touch_upgrade_preparation(struct touch *ts)
{
	u16 cmd = 0;
	u16 wdata = 0;

	cmd = REG_ISP_MODE; 
	wdata = 0x0001;
	if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
		DBG_PRINT("I2C fail with cmd(0x%x):data(0x%x)\n", cmd, wdata);
		return false;
	}

	cmd = REG_ISP_MODE; 
	wdata = 0x0002;
	if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
		DBG_PRINT("I2C fail with cmd(0x%x):data(0x%x)\n", cmd, wdata);
		return false;
	}

	cmd = REG_ISP_MODE_BUS; 
	wdata = 0x0000;
	if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
		DBG_PRINT("I2C fail with cmd(0x%x):data(0x%x)\n", cmd, wdata);
		return false;
	}

	cmd = REG_ISP_MODE_ENABLE; 
	wdata = 0xFFFF;
	if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
		DBG_PRINT("I2C fail with cmd(0x%x):data(0x%x)\n", cmd, wdata);
		return false;
	}

	return true;
}

//----------------------------------------------
static bool touch_upgrade_check(struct touch *ts, const u8 *fwData, u32 fwSize, u8 *flashType /* OUT */, bool forcedFlag)
{
	u16 rdata = 0;
	u16 flashAddr = 0;
	u16 fileVer = 0;
	u8 flashMemType = E_MEM_TYPE_EFLASH;
	bool ret = false; /* don't upgrade */

	mutex_lock(&ts->mutex);

	/* Prepare to upgrade F/W */
	if(touch_upgrade_preparation(ts) == false)
		goto FW_VER_FAIL; 

	/* Check Flash Type */ 
	flashAddr = REG_ISP_MEM_TYPE;
	if(ts->pdata->i2c_read(ts->client, (u8 *)&flashAddr, sizeof(flashAddr), (u8 *)&rdata, sizeof(rdata)) < 0) {
		DBG_PRINT("Memory Type I2C read fail!\n");
		goto FW_VER_FAIL; 
	} 

	if(rdata == REG_ISP_VAL_ERROR) { 
		DBG_PRINT("Memory Type Error!\n");
		goto FW_VER_FAIL;
	}
	else if(rdata == REG_ISP_VAL_EEPROM) {
		flashMemType = E_MEM_TYPE_EEPROM;
	}
	else {
		flashMemType = E_MEM_TYPE_EFLASH;
	}

	flashMemType = (rdata == REG_ISP_VAL_EEPROM) ? E_MEM_TYPE_EEPROM : E_MEM_TYPE_EFLASH;
	DBG_PRINT("Flash Memory Type: %s\n",
			(flashMemType == E_MEM_TYPE_EFLASH) ? TO_STR(E_MEM_TYPE_EFLASH) : TO_STR(E_MEM_TYPE_EEPROM)); 
	*flashType = flashMemType;

	/* Read Operation */
	if(touch_upgrade_start_sequence(ts, flashMemType, E_FLASH_OPMODE_READ) == false) {
		DBG_PRINT("touch_upgrade_start_sequence fail.\n");
		goto FW_VER_FAIL;
	}

	/* if forced flag is ture, then do f/w upgrade without regard to f/w version */
	if(forcedFlag == false)
	{
		/* Read F/W Versions from both file and TSC IC */
		if(TSC_FILE_FW_VER_POS >= fwSize) {
			DBG_PRINT("F/W size(0x%x) is invalid.\n", fwSize);
			goto FW_VER_FAIL;
		}

		fileVer = (fwData[TSC_FILE_FW_VER_POS]) | (fwData[TSC_FILE_FW_VER_POS + 1] << 8);
		flashAddr = TSC_FLASH_FW_VER_POS;
		if(ts->pdata->i2c_read(ts->client, (u8 *)&flashAddr, sizeof(flashAddr), (u8 *)&rdata, sizeof(rdata)) < 0) {
			DBG_PRINT("FW Ver Check I2C Fail at addr(0x%x)\n", flashAddr);
			goto FW_VER_FAIL; 
		} 
		DBG_PRINT("=== TSC FW Upgrade Check: Current Version[0x%04x] VS. File Version[0x%04x] ===\n\n", rdata, fileVer);

		/* file version is higher or there is no f/w on TSC flash */
		if((fileVer > rdata) || (rdata == 0xFFFF) || (rdata == 0x0000)) {
			ret = true;
		}
		else {
			goto FW_VER_FAIL; 
		}
	}
	else {
		ret = true;
	}

	mutex_unlock(&ts->mutex);

	return ret;

FW_VER_FAIL:
	mutex_unlock(&ts->mutex);

	touch_hw_reset(ts, E_BOOTUP_DELAY_FAST);

	return false;
}

//----------------------------------------------
static bool touch_upgrade_start_sequence(struct touch *ts, u8 flashType, u16 opMode)
{
	u16 cmd = 0;
	u16 wdata = 0;

	if(flashType == E_MEM_TYPE_EEPROM) {
		cmd = REG_CMD_EER_PDOWN;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_RESET;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_CSCON;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}
	}
	else {
		cmd = REG_CMD_FLASH_CON_EN;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}
	}

	if(flashType == E_MEM_TYPE_EEPROM) {
		if(opMode == E_FLASH_OPMODE_READ) {
			cmd = REG_CMD_EER_MODE;
			wdata = 0x0000;
			if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
				return false;
			}
		}
	}
	else {
		if(opMode != E_FLASH_OPMODE_READ) {
			cmd = REG_CMD_FLASH_AUTH;
			wdata = 0x0000;
			if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
				return false;
			}
		}
	}

	return true;
}

static bool touch_upgrade_end_sequence(struct touch *ts, u8 flashType)
{
	u16 cmd = 0;
	u16 wdata = 0;

	if(flashType == E_MEM_TYPE_EEPROM) {
		cmd = REG_CMD_EER_CSCON;
		wdata = 0x0000;
	}
	else {
		cmd = REG_CMD_FLASH_AUTH;
		wdata = 0x0001;
	}

	if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
		return false;
	}

	return true;
}

//----------------------------------------------
static bool touch_upgrade_erase(struct touch *ts, u8 flashType)
{
	u16 cmd = 0;
	u16 rdata = 0;
	u16 wdata = 0;
	u16 retry = 0;

	/* Start of Erase Operation */
	DBG_PRINT(">>>> Start\n");
	if(touch_upgrade_start_sequence(ts, flashType, E_FLASH_OPMODE_ERASE) == false) {
		DBG_PRINT("touch_upgrade_start_sequence fail.\n");
		return false;
	}

	if(flashType == E_MEM_TYPE_EFLASH) {
		cmd = REG_CMD_FLASH_COMMAND;
		wdata = 0x0002;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = 0x0000;
		wdata = 0xFFFF;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_FLASH_BUSY;
		rdata = 0x8000;
		retry = 0;
		do {
			mdelay(10);
			if(ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&rdata, sizeof(rdata)) < 0) {
				DBG_PRINT("Busy check I2C read fail (%d)\n", retry);
			}

			if(retry) {
				DBG_TOUCH("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, cmd);
			}

			retry++;
		} while ((rdata & 0x8000) && (retry < BUSY_CHECK_RETRY_COUNT)); /* check busy flag & retry count */
	}
	else {
		cmd = REG_CMD_EER_XPROT;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_MODE;
		wdata = 0x0007;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = 0x0000;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_XEN;
		wdata = 0x0001;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = 0x0000;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_STATE;
		rdata = 0x0000;
		retry = 0;
		do {
			mdelay(10);
			if(ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&rdata, sizeof(rdata)) < 0) {
				DBG_PRINT("Busy check I2C read fail (%d)\n", retry);
			}

			if(retry) {
				DBG_TOUCH("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, cmd);
			}

			retry++;
		} while(((rdata & 0x0004) == 0) && (retry < BUSY_CHECK_RETRY_COUNT)); /* check busy flag & retry count */

		cmd = REG_CMD_EER_XEN;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_XPROT;
		wdata = 0x0001;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_MODE;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}
	}

	if(retry >= BUSY_CHECK_RETRY_COUNT) {
		DBG_PRINT("FW Upgrade Erase Busy Check Fail\n");
		return false;
	}

	if(touch_upgrade_end_sequence(ts, flashType) == false) {
		DBG_PRINT("touch_upgrade_end_sequence fail\n");
		return false;
	}

	DBG_PRINT("<<<< End\n");
	/* End of Erase Operation */

	return true;
}

//----------------------------------------------
static bool touch_upgrade_program(struct touch *ts, const u8 *fwData, u32 fwSize, u8 flashType)
{
	u16 flashAddr = 0;
	u16 cmd = 0;
	u16 rdata = 0;
	u16 wdata = 0;
	u16 retry = 0;

	/* Start of Program Operation */
	DBG_PRINT(">>>> Start\n");

	if(touch_upgrade_start_sequence(ts, flashType, E_FLASH_OPMODE_WRITE) == false) {
		DBG_PRINT("touch_upgrade_start_sequence fail.\n");
		return false;
	}

	if(flashType == E_MEM_TYPE_EFLASH) {
		cmd = REG_CMD_FLASH_COMMAND;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		for(flashAddr = 0; flashAddr < fwSize; flashAddr += 2)
		{
			u16 wAddr = SWAP_16BITS(flashAddr);
			if(ts->pdata->i2c_write(ts->client, (u8 *)&wAddr, sizeof(wAddr), (u8 *)&fwData[flashAddr], 2) < 0) {
				return false;
			}

			cmd = REG_CMD_FLASH_BUSY;
			rdata = 0x8000;
			retry = 0;
			do {
				/* if it failed once then, waiting time will be increased */ 
				if(retry) udelay(50);

				if(ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&rdata, sizeof(rdata)) < 0) {
					DBG_PRINT("Busy check I2C read fail (%d)\n", retry);
				}

				if(retry) {
					DBG_TOUCH("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, cmd);
				}

				retry++;
			} while ((rdata & 0x8000) && (retry < BUSY_CHECK_RETRY_COUNT)); /* check busy flag & retry count */

			if(retry >= BUSY_CHECK_RETRY_COUNT) {
				DBG_PRINT("FW Upgrade Program Busy Check Fail\n");
				return false;
			}
		}

	} else {
		u16 pageIndex = 0, pageOffset = 0;
		u16 numOfPage = (fwSize + TSC_EEPROM_PAGE_SIZE - 1) / TSC_EEPROM_PAGE_SIZE;
		u16 targetAddr = 0;
		u16 wAddr = 0;

		cmd = REG_CMD_EER_XPROT;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_XEN;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_MODE;
		wdata = 0x0008;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_EXTEND;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		for(pageIndex = 0; pageIndex < numOfPage; pageIndex++)
		{
			u16 wLen = (pageIndex == numOfPage - 1) ? (fwSize - (numOfPage - 1) * TSC_EEPROM_PAGE_SIZE) : TSC_EEPROM_PAGE_SIZE;
			for(pageOffset = 0; pageOffset < wLen; pageOffset += 2)
			{
				targetAddr = pageIndex * TSC_EEPROM_PAGE_SIZE + pageOffset;
				wAddr = SWAP_16BITS(targetAddr);
				if(ts->pdata->i2c_write(ts->client, (u8 *)&wAddr, sizeof(wAddr), 
							(u8 *)&fwData[targetAddr], 2) < 0) {
					return false;
				}
			}

			cmd = REG_CMD_EER_XEN;
			wdata = 0x0001;
			if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
				return false;
			}

			cmd = SWAP_16BITS((pageIndex * TSC_EEPROM_PAGE_SIZE));
			wdata = 0x0000; 
			if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
				return false;
			}

			cmd = REG_CMD_EER_STATE;
			rdata = 0x0000;
			retry = 0;
			do {
				/* if it failed once then, waiting time will be increased */ 
				if(retry) udelay(100);

				if(ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&rdata, sizeof(rdata)) < 0) {
					DBG_PRINT("Busy check I2C read fail (%d)\n", retry);
				}

				if(retry) {
					DBG_TOUCH("[%d] Busy flag (0x%x) at (0x%x)\n", retry, rdata, cmd);
				}

				retry++;
			} while(((rdata & 0x0004) == 0) && (retry < BUSY_CHECK_RETRY_COUNT)); /* check busy flag & retry count */

			if(retry >= BUSY_CHECK_RETRY_COUNT) {
				DBG_PRINT("FW Upgrade Program Busy Check Fail\n");
				return false;
			}

			cmd = REG_CMD_EER_XEN;
			wdata = 0x0000;
			if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
				return false;
			}

			DBG_TOUCH("FW Upgrade Write Page(%d) of Total(%d) OK!\n", pageIndex, numOfPage);
		}

		cmd = REG_CMD_EER_XPROT;
		wdata = 0x0001;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}

		cmd = REG_CMD_EER_MODE;
		wdata = 0x0000;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			return false;
		}
	}

	if(touch_upgrade_end_sequence(ts, flashType) == false) {
		DBG_PRINT("touch_upgrade_end_sequence fail\n");
		return false;
	}

	DBG_PRINT("<<<< End\n");
	/* End of Program Operation */

	return true;
}

//----------------------------------------------
static bool touch_upgrade_fw(struct touch *ts, const u8 *fwData, u32 fwSize, u8 flashType)
{
	u16 flashAddr = 0;
	u16 cmd = 0;
	u16 rdata = 0;
	u16 wdata = 0;

	mutex_lock(&ts->mutex);

	/* Prepare to upgrade F/W */
	if(touch_upgrade_preparation(ts) == false)
		goto FW_UPGRADE_FAIL;

	if(touch_upgrade_erase(ts, flashType) == false) {
		DBG_PRINT("FW Upgrade Erase Fail\n");
		goto FW_UPGRADE_FAIL;
	}

	if(touch_upgrade_program(ts, fwData, fwSize, flashType) == false) {
		DBG_PRINT("FW Upgrade Program Fail\n");
		goto FW_UPGRADE_FAIL;
	}

	/* Start of Verify Operation */
	DBG_PRINT(">>>> Start of Verify\n");
	if(touch_upgrade_start_sequence(ts, flashType, E_FLASH_OPMODE_READ) == false) {
		DBG_PRINT("touch_upgrade_start_sequence fail.\n");
		return false;
	}

	for(flashAddr = 0; flashAddr < fwSize; flashAddr += 2)
	{
		u16 wAddr = SWAP_16BITS(flashAddr);
		u16 data = (fwData[flashAddr]) | (fwData[flashAddr + 1] << 8);

		if(ts->pdata->i2c_read(ts->client, (u8 *)&wAddr, sizeof(wAddr), (u8 *)&rdata, sizeof(rdata)) < 0) {
			goto FW_UPGRADE_FAIL;
		}

		if(data != rdata) {
			DBG_PRINT(" *** FW Upgrade Verify Fail!!! data[0x%0x] != rdata[0x%0x] at Addr[%d] \n", data, rdata, flashAddr);
			goto FW_UPGRADE_FAIL;
		}
	}

	if(touch_upgrade_end_sequence(ts, flashType) == false) {
		DBG_PRINT("touch_upgrade_end_sequence fail\n");
		return false;
	}

	cmd = REG_ISP_MODE;
	wdata = 0x0001;
	if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
		DBG_PRINT("FW Upgrade Final Step I2C Fail\n");
		goto FW_UPGRADE_FAIL;
	}
	DBG_PRINT("<<<< End of Verify\n");
	/* End of Verify Operation */

	mutex_unlock(&ts->mutex);
	return true;

FW_UPGRADE_FAIL:
	mutex_unlock(&ts->mutex);

	touch_hw_reset(ts, E_BOOTUP_DELAY_FAST);

	return false;
}
#endif /* End of #if FW_UPGRADE_ENABLE */

#if CHARGER_INFO_ENABLE
void semisens_register_callback(void *callback)
{
	charger_callbacks = (struct tsp_callbacks *)callback;
}

void touch_charger_info_cb(struct tsp_callbacks *cb, bool status)
{
	struct touch *ts = container_of(cb, struct touch, callbacks);

	DBG_PRINT("charger_informed (%d)\n", status);
	g_chargerInfo = status ? E_TS_CHARGER_ATTACHED : E_TS_CHARGER_DETACHED;

	if(ts) 
		touch_charger_info_notify(ts);
}

void touch_charger_info_notify(struct touch *ts)
{
	u16 addr = 0x00F2;
	u16 data = 0x0303;

	DBG_PRINT("charger_info (%d)\n\n", g_chargerInfo);

	if(g_chargerInfo == E_TS_CHARGER_DETACHED) {
		data = 0x0303;
	}
	else {
		data = 0x0404;
	}

	ts->pdata->i2c_write(ts->client, (u8 *)&addr, sizeof(addr), (u8 *)&data, sizeof(data));
}
#endif /* End of #if CHARGER_INFO_ENABLE */

//----------------------------------------------
// Touch i2c control function
//----------------------------------------------
s32 semisens_ts_i2c_read(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len)
{
	struct i2c_msg msg[2];
	s32 ret = 0;
	u8 i = 0;
	u8 cmd_tmp[10] = {0, };
	u8 retryCount = 0;

	if((len == 0) || (data == NULL)) {
		dev_err(&client->dev, "I2C read error: Null pointer or length == 0\n");	
		return 	-1;
	}
	
	memset(cmd_tmp, 0x00, sizeof(cmd_tmp));

	if(cmd_len)	{
		for(i = 0; i < cmd_len; i++) {
			cmd_tmp[i] = cmd[cmd_len -1 -i];
		}
	}

	memset(msg, 0x00, sizeof(msg));
	msg[0].addr		= client->addr;
	msg[0].flags	= client->flags & I2C_M_TEN;
	msg[0].len		= cmd_len;
	msg[0].buf		= cmd_tmp;

	msg[1].addr		= client->addr;
	msg[1].flags	= client->flags & I2C_M_TEN;
	msg[1].flags	|= I2C_M_RD;
	msg[1].len		= len;
	msg[1].buf		= data;

#if I2C_RETRY_COUNT 
	do {
		if((ret = i2c_transfer(client->adapter, msg, 2)) != 2) {
			dev_err(&client->dev, "Retry(%d) I2C read error: (%d) reg: 0x%X len: %d\n", 
					retryCount, ret, cmd_tmp[0], len);
		}
		else {
			break;
		}
	} while(++retryCount >= I2C_RETRY_COUNT);
#else
	if((ret = i2c_transfer(client->adapter, msg, 2)) != 2) {
		dev_err(&client->dev, "Retry(%d) I2C read error: (%d) reg: 0x%X len: %d\n", 
				retryCount, ret, cmd_tmp[0], len);
		return -EIO;
	}
#endif

	return 	len;
}

s32 semisens_ts_i2c_write(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len)
{
	s32 ret = 0;
	u8 block_data[10] = {0, };
	u8 i = 0;
	u8 cmd_tmp[10] = {0, };
	u8 retryCount = 0;

	if((cmd_len + len) >= sizeof(block_data)) {
		dev_err(&client->dev, "I2C write error: wdata overflow reg: 0x%X len: %d\n", 
				cmd[0], cmd_len + len);
		return	-1;
	}

	memset(block_data, 0x00, sizeof(block_data));
	memset(cmd_tmp, 0x00, sizeof(cmd_tmp));

	if(cmd_len) {
		for(i = 0; i < cmd_len; i++) {
			cmd_tmp[i] = cmd[cmd_len -1 -i];
		}
	}

	if(cmd_len)
		memcpy(&block_data[0], &cmd_tmp[0], cmd_len);

	if(len)
		memcpy(&block_data[cmd_len], &data[0], len);

#if I2C_RETRY_COUNT 
	do {
		if((ret = i2c_master_send(client, block_data, (cmd_len + len))) < 0) {
			dev_err(&client->dev, "Retry(%d) I2C write error: (%d) reg: 0x%X len: %d\n", 
					retryCount, ret, cmd[0], len);
		}
		else { 
			break;
		}
	} while(++retryCount >= I2C_RETRY_COUNT);
#else
	if((ret = i2c_master_send(client, block_data, (cmd_len + len))) < 0) {
		dev_err(&client->dev, "Retry(%d) I2C write error: (%d) reg: 0x%X len: %d\n", 
				retryCount, ret, cmd[0], len);
		return ret;
	}
#endif
	
	return len;
}

//----------------------------------------------
// Touch initialize & finalize function
//----------------------------------------------
void semisens_ts_enable(struct touch *ts)
{
	if(ts->enabled == false) {
		enable_irq(ts->irq);
		DBG_PRINT("touch irq(%d) is enabled!\n", ts->irq);

		ts->enabled = true;
	}
}

void semisens_ts_disable(struct touch *ts)
{
	if(ts->enabled == true) {
		disable_irq(ts->irq);
		DBG_PRINT("touch irq(%d) is disabled!\n", ts->irq);

		ts->enabled = false;

		if(ts->pdata->event_clear)
			ts->pdata->event_clear(ts);
	}
}

s32 semisens_ts_early_probe(struct touch *ts)
{
	// nothing to do...
	DBG_PRINT("called\n");
	return	0;
}

s32 semisens_ts_probe(struct touch *ts)
{
	u16 cmd = REG_FIRMWARE_VERSION;
	u16 rdata = 0;

	/* 
	 * for some low-power models such as RCUs, smart watches, 
	 * due to TSC's sleep mode, in order to read data via I2C bus, gpio control should be needed 
	 */ 
#if PROBE_WAKEUP_ENABLE
	/* TSC wake-up to read data */
	gpio_direction_output(ts->pdata->irq_gpio, 0);
	gpio_set_value(ts->pdata->irq_gpio, 0);
	mdelay(1);
#endif

	// Touch screen information display
	if(ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&rdata, sizeof(rdata)) < 0) {
		DBG_PRINT("fail to get touch ic firmware version.\n");
		return	-1;
	}
	
#if PROBE_WAKEUP_ENABLE
	/* restore gpio for irq */
	gpio_set_value(ts->pdata->irq_gpio, 1);
	gpio_direction_input(ts->pdata->irq_gpio);
#endif

	ts->fw_version = rdata;

	DBG_PRINT("touch ic firmware version : %d \n", rdata);
	
	return 	0;
}

//----------------------------------------------
// Touch data processing function
//----------------------------------------------
void semisens_ts_work(struct touch *ts)
{
	u8 find_slot = 0;
	u16 cmd = 0;
	status_reg_u status;
	data_reg_t data;
	button_u button;
	u32 ids = 0;
	s32 i = 0;

/* NOTE. if compile error happens here, then block below, but you MUST change the permission in other ways */
#if defined(SEMISENS_TS_NATIVE_INTERFACE) && !defined(MODULE)
	if(g_miscInitialize == 0)	{
		if(sys_chmod((const char __user *)"/dev/sn310m_dist", 0x0666) < 0) {
			DBG_PRINT("failed to change the \"/dev/sn310m_dist\" permission.\n");
		}
		else {
			DBG_PRINT("succeeded to change the \"/dev/sn310m_dist\" permission.\n");
			g_miscInitialize = 1;
		}
	}
#endif

#if KNOCK_ON_MODE
	if(g_knockOnFlag)
	{
		const char *resultString[3] = { "GESTURE", "MULTI-GESTURE", "INVALID" };
		u16 readValue;
		u8 gestureType = 0;
		s8 gestureValue = 0;

		DBG_PRINT("=== interrupt arised while knock on sleep ===\n\n");

		/* Check if the result bit of knock-on/code is set */ 
		cmd = REG_TS_STATUS;
		if(ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), 
					(u8 *)&status.uint, sizeof(status_reg_u)) < 0) { 
			DBG_PRINT("Reading knock-on/code at 0x%x results in failure!\n", cmd);
		} 
		else
		{
			DBG_PRINT("====================================================\n");
			DBG_PRINT(" raw data[15:0] is [0x%02x].\n", status.uint);
			DBG_PRINT(" wakeup source is [%s].\n", resultString[status.bits.gesture]);


			cmd = 0x40E0; /* Addr For Gesture Value */
			if(ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), 
						(u8 *)&readValue, sizeof(readValue)) < 0) { 
				DBG_PRINT("Reading knock-on/code at 0x%x results in failure!\n", cmd);
			} 
			else
			{
				/* check if gesture is enabled */
				gestureType = (readValue >> 8);
				gestureValue = (readValue & 0xFF);
				DBG_PRINT("Gesture type [0x%x], Gesture Value [%c].\n", 
						gestureType, ((gestureType != 0x04) ? '?' : gestureValue));
			}
			DBG_PRINT("====================================================\n\n");
		}

		/* check if multi gesture, gesture detected, gesture value are appropriate */
#if 0	
		if((status.bits.gesture) && (gestureType == 0x04) && (gestureValue == '7')) {
			//TODO: check if ts->pdata->event_clear(ts); or ts->pdata->report(ts); 		
			DBG_PRINT("=== wakeup from knock on sleep ===\n\n\n");
			g_knockOnFlag = 0;

			/* Do Reset before wakeup */
			touch_hw_reset(ts, E_BOOTUP_DELAY_FAST);

			input_report_key(ts->input, KEY_POWER, true);
			input_sync(ts->input);
			mdelay(10);
			input_report_key(ts->input, KEY_POWER, false);
			input_sync(ts->input);
		}
#else
		if(status.bits.gesture) {
			DBG_PRINT("=== wakeup from knock on sleep ===\n\n\n");
			g_knockOnFlag = 0;

			/* Do Reset before wakeup */
			touch_hw_reset(ts, E_BOOTUP_DELAY_FAST);

			input_report_key(ts->input, KEY_POWER, true);
			input_sync(ts->input);
			mdelay(10);
			input_report_key(ts->input, KEY_POWER, false);
			input_sync(ts->input);
		}
#endif

		return;
	}
#endif /* End of #if KNOCK_ON_MODE */

	mutex_lock(&ts->mutex);

	cmd = REG_TS_STATUS;
	ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&status.uint, sizeof(status_reg_u));

#if PROXIMITY_MODE
	/* if proximity is enabled, check proximity result here */
	if(status.bits.proximity == 0x1) // Near
	{
		/* Do something */
	}
	else if(status.bits.proximity == 0x2) // Far
	{
		/* Do something */
	}	
#endif

	if(status.bits.ts_cnt <= ts->pdata->max_fingers) {
		u8 cnt = 0;

		if(ts->pdata->keycode && (status.bits.ts_cnt == 0)) {
			button.bits.bt0_press = (status.bits.button & 0x01) ? 1 : 0;
			button.bits.bt1_press = (status.bits.button & 0x02) ? 1 : 0;
			button.bits.bt2_press = (status.bits.button & 0x04) ? 1 : 0;
			button.bits.bt3_press = (status.bits.button & 0x08) ? 1 : 0;
			
			ts->pdata->key_report(ts, button.ubyte);
		}

		for(cnt = 0; cnt < status.bits.ts_cnt; cnt++) {
			u32 id;
			u32 x;
			u32 y;
			u32 area;
			u32 pressure;

			cmd = REG_TS_DATA(cnt);
			ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&data.packet0, sizeof(data_reg_t));

			id = data.packet0 >> 12;
#if XY_CHANGE_ENABLE
			x = data.packet1 & 0xFFF;
			y = data.packet0 & 0xFFF;
			y = ts->abs_max_y - y;
#else
			x = data.packet0 & 0xfff;
			y = data.packet1 & 0xfff;
#endif
			area = data.packet2 & 0xfff;
			pressure = ((data.packet1 >> 8) & 0x00f0) + (data.packet2 >> 12);	

			DBG_TOUCH("cmd=%d, id=%d, x=%d, y=%d, area=%d, pressure=%d \n", cmd, id, x, y, area, pressure);
			DBG_TOUCH("pkt0=%x pkt1=%x pkt2=%x \n", data.packet0, data.packet1, data.packet2);

			if((x > ts->pdata->abs_max_x) || (y > ts->pdata->abs_max_y)) {
				if(ts->pdata->event_clear)		
					ts->pdata->event_clear(ts);

				DBG_TOUCH("ERROR -> x(%d) or y(%d) value overflow!\n", x, y);
				continue;
			}

			if(ts->pdata->id_max) {
				if((id > ts->pdata->id_max) || (id < ts->pdata->id_min)) {
					if(ts->pdata->event_clear)
						ts->pdata->event_clear(ts);

					DBG_TOUCH("ERROR -> id(%d) value overflow!\n", id);
					continue;
				}

				if((find_slot = semisens_ts_id_tracking(ts, id)) == 0xFF) {
					DBG_TOUCH("ERROR -> empty slot not found!\n");
					continue;
				}
			}
			else {
				if(id == 0)	
					continue;

				find_slot = cnt;
			}
			
			if(ts->finger[find_slot].event == TS_EVENT_UNKNOWN)
				ts->finger[find_slot].event	= TS_EVENT_PRESS;
			else if((ts->finger[find_slot].event == TS_EVENT_PRESS) || (ts->finger[find_slot].event == TS_EVENT_MOVE))
				ts->finger[find_slot].event = TS_EVENT_MOVE;

			ts->finger[find_slot].status	= true;
			ts->finger[find_slot].id		= id;
			ts->finger[find_slot].x			= x;
			ts->finger[find_slot].y			= y;
			ts->finger[find_slot].area		= (ts->pdata->area_max < area) ? ts->pdata->area_max : area;
			ts->finger[find_slot].pressure	= (ts->pdata->press_max < pressure) ? ts->pdata->press_max : pressure;
			ids |= 1 << find_slot;
		}
	}

	for(i = 0; i < ts->pdata->max_fingers; i++) {
		if(!(ids & (1 << i))) {
			if(ts->finger[i].event != TS_EVENT_UNKNOWN) {
				ts->finger[i].status	= true;
				ts->finger[i].event 	= TS_EVENT_RELEASE;
			}
		}
	}

	ts->pdata->report(ts);	

	mutex_unlock(&ts->mutex);
}

u8 semisens_ts_id_tracking(struct touch *ts, u8 find_id)
{
	u8 find_slot = 0xFF;
	s32 i = 0;

	for(i = 0; i < ts->pdata->max_fingers; i++) {
		if(ts->finger[i].id == find_id)
			find_slot = i;
			
		if((ts->finger[i].event == TS_EVENT_UNKNOWN) && (find_slot == 0xFF))	
			find_slot = i;
	}
	return	find_slot;
}

//----------------------------------------------
irqreturn_t touch_irq(s32 irq, void *handle)
{
	struct touch *ts = handle;

	DBG_TOUCH("called IRQ(%d)\n", irq);

	if(ts->pdata->irq_mode == IRQ_MODE_NORMAL)		
	{
		queue_work(ts->work_queue, &ts->work);	
	}
	else if(ts->pdata->irq_mode == IRQ_MODE_THREAD)						
	{
		disable_irq_nosync(ts->irq);
		queue_work(ts->work_queue, &ts->work);	
		enable_irq(ts->irq);
	}

	return IRQ_HANDLED;
}

//----------------------------------------------
static void touch_work_q(struct work_struct *work)
{
	struct touch *ts = container_of(work, struct touch, work);
	
	ts->pdata->touch_work(ts);
}

//----------------------------------------------
static void touch_key_report(struct touch *ts, u8 button_data)
{
	static button_u	button_old; 
	button_u button_new;

	button_new.ubyte = button_data;
	if(button_old.ubyte != button_new.ubyte) {
		if((button_old.bits.bt0_press != button_new.bits.bt0_press) && (ts->pdata->keycnt > 0)) {
			if(button_new.bits.bt0_press)	input_report_key(ts->input, ts->pdata->keycode[0], true);
			else							input_report_key(ts->input, ts->pdata->keycode[0], false);

			DBG_TOUCH_KEY("keycode[0](0x%04X) %s\n", 
					ts->pdata->keycode[0], button_new.bits.bt0_press ? "press":"release");
		}

		if((button_old.bits.bt1_press != button_new.bits.bt1_press) && (ts->pdata->keycnt > 1)) {
			if(button_new.bits.bt1_press)	input_report_key(ts->input, ts->pdata->keycode[1], true);
			else							input_report_key(ts->input, ts->pdata->keycode[1], false);

			DBG_TOUCH_KEY("keycode[1](0x%04X) %s\n", 
					ts->pdata->keycode[1], button_new.bits.bt1_press ? "press":"release");
		}

		if((button_old.bits.bt2_press != button_new.bits.bt2_press) && (ts->pdata->keycnt > 2)) {
			if(button_new.bits.bt2_press)	input_report_key(ts->input, ts->pdata->keycode[2], true);
			else							input_report_key(ts->input, ts->pdata->keycode[2], false);

			DBG_TOUCH_KEY("keycode[2](0x%04X) %s\n", 
					ts->pdata->keycode[2], button_new.bits.bt2_press ? "press":"release");
		}

		if((button_old.bits.bt3_press != button_new.bits.bt3_press) && (ts->pdata->keycnt > 3)) {
			if(button_new.bits.bt3_press)	input_report_key(ts->input, ts->pdata->keycode[3], true);
			else							input_report_key(ts->input, ts->pdata->keycode[3], false);

			DBG_TOUCH_KEY("keycode[3](0x%04X) %s\n", 
					ts->pdata->keycode[3], button_new.bits.bt3_press ? "press":"release");
		}

		if((button_old.bits.bt4_press != button_new.bits.bt4_press) && (ts->pdata->keycnt > 4)) {
			if(button_new.bits.bt4_press)	input_report_key(ts->input, ts->pdata->keycode[4], true);
			else							input_report_key(ts->input, ts->pdata->keycode[4], false);

			DBG_TOUCH_KEY("keycode[4](0x%04X) %s\n", 
					ts->pdata->keycode[4], button_new.bits.bt4_press ? "press":"release");
		}

		if((button_old.bits.bt5_press != button_new.bits.bt5_press) && (ts->pdata->keycnt > 5)) {
			if(button_new.bits.bt5_press)	input_report_key(ts->input, ts->pdata->keycode[5], true);
			else							input_report_key(ts->input, ts->pdata->keycode[5], false);

			DBG_TOUCH_KEY("keycode[5](0x%04X) %s\n", 
					ts->pdata->keycode[5], button_new.bits.bt5_press ? "press":"release");
		}

		if((button_old.bits.bt6_press != button_new.bits.bt6_press) && (ts->pdata->keycnt > 6)) {
			if(button_new.bits.bt6_press)	input_report_key(ts->input, ts->pdata->keycode[6], true);
			else							input_report_key(ts->input, ts->pdata->keycode[6], false);

			DBG_TOUCH_KEY("keycode[6](0x%04X) %s\n", 
					ts->pdata->keycode[6], button_new.bits.bt6_press ? "press":"release");
		}

		if((button_old.bits.bt7_press != button_new.bits.bt7_press) && (ts->pdata->keycnt > 7)) {
			if(button_new.bits.bt7_press)	input_report_key(ts->input, ts->pdata->keycode[7], true);
			else							input_report_key(ts->input, ts->pdata->keycode[7], false);

			DBG_TOUCH_KEY("keycode[7](0x%04X) %s\n", 
					ts->pdata->keycode[7], button_new.bits.bt7_press ? "press":"release");
		}

		button_old.ubyte = button_new.ubyte;
	}
}

//----------------------------------------------
static void touch_report_protocol_a(struct touch *ts)
{
	s32 id;
	
	for(id = 0; id < ts->pdata->max_fingers; id++) {

		if(ts->finger[id].event == TS_EVENT_UNKNOWN)		continue;
		
		if(ts->finger[id].event != TS_EVENT_RELEASE) {
			if(ts->pdata->id_max)		input_report_abs(ts->input, ABS_MT_TRACKING_ID, ts->finger[id].id);
			if(ts->pdata->area_max)		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, ts->finger[id].area ? ts->finger[id].area : 10);
			if(ts->pdata->press_max)	input_report_abs(ts->input, ABS_MT_PRESSURE, ts->finger[id].pressure);
	
			input_report_abs(ts->input, ABS_MT_POSITION_X, 	ts->finger[id].x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,	ts->finger[id].y);

			DBG_TOUCH("id = %d, x = %d, y = %d\n", ts->finger[id].id, ts->finger[id].x, ts->finger[id].y);
		}
		else {
			ts->finger[id].event = TS_EVENT_UNKNOWN;
			DBG_TOUCH("release id = %d\n", ts->finger[id].id);
		}

		input_mt_sync(ts->input);
	}

	input_sync(ts->input);
}

//----------------------------------------------
static void touch_report_protocol_b(struct touch *ts)
{
	s32	id;
	char *event_str[] = {"unknown", "press", "move", "release"};

	for(id = 0; id < ts->pdata->max_fingers; id++) {

		if((ts->finger[id].event == TS_EVENT_UNKNOWN) || (ts->finger[id].status == false))	continue;

		input_mt_slot(ts->input, id);	ts->finger[id].status = false;	

		if(ts->finger[id].event != TS_EVENT_RELEASE) {
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, ts->finger[id].id);
			input_report_abs(ts->input, ABS_MT_POSITION_X, 	ts->finger[id].x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,	ts->finger[id].y);
			input_report_key(ts->input, BTN_TOUCH, 1);

			if(ts->pdata->area_max)		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, ts->finger[id].area ? ts->finger[id].area : 10);
			if(ts->pdata->press_max)	input_report_abs(ts->input, ABS_MT_PRESSURE, ts->finger[id].pressure);
	
			DBG_TOUCH("event = %s, slot = %d, id = %d, x = %d, y = %d\n", 
					event_str[ts->finger[id].event],  id, ts->finger[id].id, ts->finger[id].x, ts->finger[id].y);
		}
		else {
			DBG_TOUCH("event = %s, slot = %d, id = %d\n", 
					event_str[ts->finger[id].event], id, ts->finger[id].id);
			ts->finger[id].event = TS_EVENT_UNKNOWN;
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, -1);
			input_report_key(ts->input, BTN_TOUCH, 0);
		}
	}
	input_sync(ts->input);
}

//----------------------------------------------
static void touch_event_clear(struct touch *ts)
{
	u8 id;
	
	for(id = 0; id < ts->pdata->max_fingers; id++) {
		if(ts->finger[id].event == TS_EVENT_MOVE) {
			ts->finger[id].status	= true;
			ts->finger[id].event 	= TS_EVENT_RELEASE;
		}
	}
	ts->pdata->report(ts);
	if(ts->pdata->keycode)
		ts->pdata->key_report(ts, 0x00);
}

//----------------------------------------------
static s32 touch_input_open(struct input_dev *input)
{
	struct touch *ts;

	DBG_PRINT("called");
	ts = input_get_drvdata(input);

	if(ts->enabled == true) {
		DBG_PRINT("already enabled\n");
		return 0;
	}

#if INPUT_PM_ENABLE
	touch_resume_pm(&ts->client->dev);
#else
	ts->pdata->enable(ts);
#endif /* End of #if INPUT_PM_ENABLE */

	return 0;
}

//----------------------------------------------
static void touch_input_close(struct input_dev *input)
{
	struct touch *ts;

	DBG_PRINT("called");
	ts = input_get_drvdata(input);

	if(ts->enabled == false) {
		DBG_PRINT("already disbled\n");
		return;
	}

#if INPUT_PM_ENABLE
	touch_suspend_pm(&ts->client->dev);
#else
	#if !KNOCK_ON_MODE 
	ts->pdata->disable(ts);
	#endif
#endif /* End of #if INPUT_PM_ENABLE */
}

//----------------------------------------------
static s32 touch_check_functionality(struct touch_pdata *pdata)
{
	if(!pdata) {
		DBG_PRINT("Error : Platform data is NULL pointer!\n");	
		return	-1;
	}

	pdata->name = TS_INPUT_NAME;
	pdata->irq_mode = IRQ_MODE_THREAD;	// IRQ_MODE_THREAD, IRQ_MODE_NORMAL

	if(pdata->irq_mode == IRQ_MODE_THREAD)
		pdata->irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	else
		pdata->irq_flags = IRQF_TRIGGER_FALLING /*| IRQF_DISABLED*/;

	pdata->max_fingers	= SEMISENS_TS_FINGER_MAX;

	pdata->area_max		= 10; 
	pdata->press_max	= 255;
	pdata->id_max		= SEMISENS_TS_FINGER_MAX - 1;
	pdata->id_min		= 0;
	pdata->keycnt		= SEMISENS_TS_KEY_MAX;

#if XY_CHANGE_ENABLE
	if(1) {
		/* SWAP x/y */
		u32 temp = pdata->abs_max_x;
		pdata->abs_max_x = pdata->abs_max_y;
		pdata->abs_max_y = temp;
	}
#endif

#if KNOCK_ON_MODE
	pdata->wakeup		= true;
#else
	pdata->wakeup		= false;
#endif

	if(!pdata->power)		pdata->power = ts_power_ctrl;

	if(!pdata->i2c_read)	pdata->i2c_read	= semisens_ts_i2c_read;
	if(!pdata->i2c_write)	pdata->i2c_write = semisens_ts_i2c_write;
		
	if(!pdata->enable)		pdata->enable = semisens_ts_enable;
	if(!pdata->disable)		pdata->disable = semisens_ts_disable;

	if(!pdata->probe)		pdata->probe = semisens_ts_probe;

	if(!pdata->report) {
		if(pdata->id_max)	pdata->report = touch_report_protocol_b;
		else				pdata->report = touch_report_protocol_a;
	}

	if(!pdata->keycode)		pdata->keycode = ts_keycode;

	if(!pdata->key_report)	pdata->key_report = touch_key_report;

	if(!pdata->touch_work)	pdata->touch_work = semisens_ts_work;
	
	if(!pdata->irq_func)	pdata->irq_func = touch_irq;

	if(!pdata->event_clear)	pdata->event_clear = touch_event_clear;
		
#if defined(CONFIG_HAS_EARLYSUSPEND)
	if(!pdata->resume)	pdata->resume	= touch_resume;
	if(!pdata->suspend)	pdata->suspend	= touch_suspend;
#endif

#if CHARGER_INFO_ENABLE
	if(!pdata->register_cb) pdata->register_cb = semisens_register_callback;
#endif
	return	0;
}

//----------------------------------------------
void touch_hw_reset(struct touch *ts, u8 delayMode)
{
	if(gpio_is_valid(ts->pdata->reset_gpio)) {
		if(ts->pdata->power) {
			/* power sequence with resetb & power(PVDD): reset low -> power on -> reset high -> 2ms active -> 1ms reset -> active */
			DBG_PRINT("power sequence with resetb & power(PVDD): reset low -> power on -> reset high\n");
			gpio_direction_output(ts->pdata->reset_gpio, 0);
			gpio_set_value(ts->pdata->reset_gpio, 0); 
			mdelay(1);
			
			ts->pdata->power(ts, 1, true);
			mdelay(5);
			
			gpio_set_value(ts->pdata->reset_gpio, 1); 
			mdelay(2);
			
			gpio_set_value(ts->pdata->reset_gpio, 0); 
			mdelay(1);
			
			gpio_set_value(ts->pdata->reset_gpio, 1); 
		}
		else {
			/* if there is no power control for touch, then just do reset (low -> high (2ms active) -> low (1ms reset) -> high) */
			DBG_PRINT("no power control for touch, then just do reset (high -> low -> high)\n");
			gpio_direction_output(ts->pdata->reset_gpio, 0);
			gpio_set_value(ts->pdata->reset_gpio, 0); 
			mdelay(1);
			
			gpio_set_value(ts->pdata->reset_gpio, 1); 
			mdelay(2);
			
			gpio_set_value(ts->pdata->reset_gpio, 0); 
			mdelay(1);
			
			gpio_set_value(ts->pdata->reset_gpio, 1); 
		}
	}	
	else {
		if(ts->pdata->power) {
			/* power sequence without resetb: power on -> power off -> power on */
			DBG_PRINT("power sequence without resetb: power on -> power off -> power on\n");
			ts->pdata->power(ts, 1, true);
			mdelay(1);
			
			ts->pdata->power(ts, 0, true);
			mdelay(5);
			
			ts->pdata->power(ts, 1, true);
		}
		else {
			/* there is no way to control touch h/w power */
			DBG_PRINT("touch h/w reset error!\n");
		}
	}

#if TOUCH_SW_RESET_ENABLE
	if(1) {
		u16 cmd = 0;
		u16 wdata = 0;
		
		udelay(500);
		
		cmd = REG_ISP_MODE; 
		wdata = 0x0001;
		if(ts->pdata->i2c_write(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&wdata, sizeof(wdata)) < 0) {
			DBG_PRINT("SW RESET I2C Fail\n");
		}
	}
#endif

	/* TSC boot-up delay */ 
	if(delayMode == E_BOOTUP_DELAY_NORMAL) 
		mdelay(5); 
	else
		udelay(500);
}

//----------------------------------------------
s32 touch_info_display(struct touch *ts)
{
	printk("--------------------------------------------------------\n");
	printk("           TOUCH SCREEN INFORMATION\n");
	printk("--------------------------------------------------------\n");
	if(gpio_is_valid(ts->pdata->irq_gpio))	{
		printk("TOUCH INPUT Name = %s\n", ts->pdata->name);
		
		switch(ts->pdata->irq_mode)	{
			default	:
			case	IRQ_MODE_THREAD:	printk("TOUCH IRQ Mode   = %s\n", "IRQ_MODE_THREAD");	break;
			case	IRQ_MODE_NORMAL:	printk("TOUCH IRQ Mode   = %s\n", "IRQ_MODE_NORMAL");	break;
			case	IRQ_MODE_POLLING:	printk("TOUCH IRQ Mode   = %s\n", "IRQ_MODE_POLLING");	break;
		}
		printk("TOUCH F/W Version = %d.%02d\n", ts->fw_version / 100, ts->fw_version % 100);
	
		printk("TOUCH FINGRES MAX = %d\n", ts->pdata->max_fingers);
		printk("TOUCH ABS X MAX = %d, TOUCH ABS X MIN = %d\n", ts->pdata->abs_max_x, ts->pdata->abs_min_x);
		printk("TOUCH ABS Y MAX = %d, TOUCH ABS Y MIN = %d\n", ts->pdata->abs_max_y, ts->pdata->abs_min_y);
	
		if(ts->pdata->area_max)
			printk("TOUCH MAJOR MAX = %d, TOUCH MAJOR MIN = %d\n", ts->pdata->area_max, ts->pdata->area_min);
		
		if(ts->pdata->press_max)
			printk("TOUCH PRESS MAX = %d, TOUCH PRESS MIN = %d\n", ts->pdata->press_max, ts->pdata->press_min);
	
		if(ts->pdata->id_max) {
			printk("TOUCH ID MAX = %d, TOUCH ID MIN = %d\n", ts->pdata->id_max, ts->pdata->id_min);
			printk("Mulit-Touch Protocol-B Used.\n");
		}
		else
			printk("Mulit-Touch Protocol-A Used.\n");
	
		if(gpio_is_valid(ts->pdata->reset_gpio))	
			printk("H/W Reset function implemented\n");
	}
	else {
		printk("TOUCH INPUT Name = %s\n", ts->pdata->name);
		printk("Dummy Touchscreen driver!\n");
	}
	printk("--------------------------------------------------------\n");
	
	return	0;
}

//----------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0) 
s32 touch_probe(struct i2c_client *client, const struct i2c_device_id *client_id)
#else
s32 touch_probe(struct i2c_client *client)
#endif
{
	s32 rc = -1;
	struct device *dev = &client->dev;
	struct touch *ts;
	struct touch_pdata *pdata;
#if FW_UPGRADE_ENABLE 
	u8 flashType;
#endif

	DBG_PRINT(">>>> Start");

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		return -EIO;
	}

#ifdef CONFIG_OF 
	if(client->dev.of_node) {
		DBG_PRINT("Device Tree Based Touch Device Driver.\n");
		pdata = devm_kzalloc(&client->dev, sizeof(struct touch_pdata), GFP_KERNEL);
		if(!pdata) {
			dev_err(&client->dev, "Failed to allocate memory!\n");
			return -ENOMEM;
		}

		rc = semisens_parse_dt(&client->dev, pdata);
		if(rc) {
			dev_err(&client->dev, "Failed to parse DT!\n");
			return rc;
		}

	}
	else 
#endif /* End of #ifdef CONFIG_OF */
	{
		DBG_PRINT("Platform Data Based Touch Device Driver.\n");
		pdata = client->dev.platform_data;
	}

	if(touch_check_functionality(pdata) < 0) {
		dev_err(&client->dev, "Platform data is not available!\n");
		return -EINVAL;
	}

	if(!(ts = kzalloc(sizeof(struct touch), GFP_KERNEL))) {
		dev_err(&client->dev, "touch struct malloc error!\n");			
		return	-ENOMEM;
	}
	ts->client	= client;
	ts->pdata 	= pdata;
	
#if CHARGER_INFO_ENABLE 
	ts->register_cb = pdata->register_cb;
	ts->callbacks.inform_charger = touch_charger_info_cb;
	if(ts->register_cb)
		ts->register_cb(&ts->callbacks);
#endif

	i2c_set_clientdata(client, ts);

	semisens_init_gpio(ts->pdata);

#if PINCTRL_ENABLE
	/* Get pinctrl if target uses pinctrl */
	ts->pinctrl = devm_pinctrl_get(&client->dev);
	if(IS_ERR(ts->pinctrl)) {
		if (PTR_ERR(ts->pinctrl) == -EPROBE_DEFER)
			return	-EPROBE_DEFER;

		DBG_PRINT("Target does not use pinctrl ERR=%ld\n", PTR_ERR(ts->pinctrl));
		ts->pinctrl = NULL;
	}

	if(ts->pinctrl) {
		rc = pinctrl_configure(ts, true);
		if(rc) {
			DBG_PRINT("cannot set pinctrl state\n");
		}
	}
#endif /* End of #if PINCTRL_ENABLE */
	
	/* if you haven't known IRQ number of GPIO for TS interrupt pin, then find here and set it */
	if(client->irq)
		ts->irq = client->irq;
	else
		ts->irq = gpio_to_irq(ts->pdata->irq_gpio);

	DBG_PRINT("ts->irq = %d\n", ts->irq);

	if(ts->pdata->max_fingers) {
		if(!(ts->finger = kzalloc(sizeof(finger_t) * ts->pdata->max_fingers, GFP_KERNEL))) {
			kfree(ts);
			DBG_PRINT("touch data struct malloc error!\n");	
			return	-ENOMEM;
		}
	}
	
	if(ts->pdata->early_probe) {
		if((rc = ts->pdata->early_probe(ts)) < 0)	
			goto err_free_mem;
	}

	dev_set_drvdata(dev, ts);

	if(semisens_init_power(ts)) {
		DBG_PRINT("semisens touch power init failed!\n");
		goto err_free_mem;
	}

	if(!(ts->input = input_allocate_device())) 
		goto err_free_mem;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));

	ts->input->open			= touch_input_open;
	ts->input->close		= touch_input_close;

	ts->input->name			= ts->pdata->name;
	ts->input->phys			= ts->phys;
	ts->input->dev.parent	= dev;
	ts->input->id.bustype	= BUS_I2C;

	set_bit(EV_SYN, ts->input->evbit);		
	set_bit(EV_ABS, ts->input->evbit);
	set_bit(BTN_TOUCH, ts->input->keybit);		/* for multi-touch, indicates whether the tool is touching the device */

#if defined(CONFIG_MACH_FORTUNA_CMCC)
	set_bit(MT_TOOL_FINGER, ts->input->keybit);
	set_bit(EV_LED, ts->input->evbit);
	set_bit(LED_MISC, ts->input->ledbit);
#endif

#if TOUCH_DEVICE_TYPE_POINTER
	/* device type will be set to pointer */
	set_bit(INPUT_PROP_POINTER, ts->input->propbit);
	set_bit(INPUT_PROP_SEMI_MT, ts->input->propbit);
#else
	/* device type will be set to touch screen */
	set_bit(INPUT_PROP_DIRECT, ts->input->propbit);
#endif

	/* Register Touch Key Event */
	if(ts->pdata->keycode) {
		s32	key;

		set_bit(EV_KEY,	ts->input->evbit);

		for(key = 0; key < ts->pdata->keycnt; key++) {
			if(ts->pdata->keycode[key] <= 0) continue;
			set_bit(ts->pdata->keycode[key] & KEY_MAX, ts->input->keybit);
		}
	}

#if KNOCK_ON_MODE
	input_set_capability(ts->input, EV_KEY, KEY_POWER);
#endif

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, ts->pdata->abs_min_x, ts->pdata->abs_max_x, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, ts->pdata->abs_min_y, ts->pdata->abs_max_y, 0, 0);
		
	if(ts->pdata->area_max)
		input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, ts->pdata->area_min, ts->pdata->area_max, 0, 0);
		
	if(ts->pdata->press_max)
		input_set_abs_params(ts->input, ABS_MT_PRESSURE, ts->pdata->press_min, ts->pdata->press_max, 0, 0);

	if(ts->pdata->id_max) {
		input_set_abs_params(ts->input, ABS_MT_TRACKING_ID, ts->pdata->id_min, ts->pdata->id_max, 0, 0);

	/* added flags to func input_mt_init_slots() from kernel 3.7 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0) 
	#if TOUCH_DEVICE_TYPE_POINTER
		input_mt_init_slots(ts->input, ts->pdata->max_fingers, INPUT_MT_POINTER);
	#else
		input_mt_init_slots(ts->input, ts->pdata->max_fingers, INPUT_MT_DIRECT);
	#endif
#else
		input_mt_init_slots(ts->input, ts->pdata->max_fingers);
#endif /* End of #if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0) */
	}

	/* it will do nothing even though input_open called while registering input dev. */
	ts->enabled = true; 

	DBG_PRINT("Registering input device (%s)...\n", ts->input->name);

	input_set_drvdata(ts->input, ts);
	if((rc = input_register_device(ts->input))) {
		dev_err(dev, "input(%s) register fail!\n", ts->input->name);
		goto err_free_input_mem;
	}

	mutex_init(&ts->mutex);

	if(ts->irq)	{
		/* don't enable irq(ts->irq) automatically when requesting irq */
		irq_set_status_flags(ts->irq, IRQ_NOAUTOEN);

		switch(ts->pdata->irq_mode)	{
			case IRQ_MODE_THREAD:
				INIT_WORK(&ts->work, touch_work_q);
				if((ts->work_queue = create_singlethread_workqueue("ts_work_queue")) == NULL)	
					goto err_free_input_mem;

				if((rc = request_threaded_irq(ts->irq, NULL, ts->pdata->irq_func, 
								ts->pdata->irq_flags, ts->pdata->name, ts))) {
					dev_err(dev, "irq %d request fail!\n", ts->irq);
					goto err_free_input_mem;
				}
				DBG_PRINT("IRQ_MODE_THREAD: irq %d request ok!\n", ts->irq);
				break;

			case IRQ_MODE_NORMAL:
				INIT_WORK(&ts->work, touch_work_q);
				if((ts->work_queue = create_singlethread_workqueue("ts_work_queue")) == NULL)	
					goto err_free_input_mem;

				if((rc = request_irq(ts->irq, ts->pdata->irq_func, ts->pdata->irq_flags, 
								ts->pdata->name, ts))) {
					dev_err(dev, "irq %d request fail!\n", ts->irq);
					goto err_free_input_mem;
				}
				DBG_PRINT("IRQ_MODE_NORMAL: irq %d request ok!\n", ts->irq);
				break;

			case IRQ_MODE_POLLING:
				/* fall-through */
			default	:
				DBG_PRINT("IRQ_MODE(%d) is not supported\n", ts->pdata->irq_mode);
				break;
		} /* End of switch(ts->pdata->irq_mode)	*/
	}

	/* turn on the power and reset of Touch IC */ 
	touch_hw_reset(ts, E_BOOTUP_DELAY_FAST);

#if FW_UPGRADE_ENABLE 
#if PROBE_WAKEUP_ENABLE
	/* TSC wake-up to read data */
	gpio_direction_output(ts->pdata->irq_gpio, 0);
	gpio_set_value(ts->pdata->irq_gpio, 0);
	mdelay(1);
#endif

	if(touch_upgrade_check(ts, TSC_FW_ARRAY_NAME, sizeof(TSC_FW_ARRAY_NAME), &flashType, false) == true)
	{
		u8 upgradeRetryCnt = FW_UPGRADE_RETRY_COUNT;
		DBG_PRINT("==== Start upgrading TSC F/W ... ====\n"); 
		for(upgradeRetryCnt = 0; upgradeRetryCnt < FW_UPGRADE_RETRY_COUNT; upgradeRetryCnt++)
		{
			if(touch_upgrade_fw(ts, TSC_FW_ARRAY_NAME, sizeof(TSC_FW_ARRAY_NAME), flashType) == true) {
				DBG_PRINT("==== TSC F/W upgrade done... ====\n\n"); 
				break;
			}
			DBG_PRINT("Retry(%d) TSC F/W upgrade...\n", upgradeRetryCnt);
		}
	}
	
#if PROBE_WAKEUP_ENABLE
	/* restore gpio for irq */
	gpio_set_value(ts->pdata->irq_gpio, 1);
	gpio_direction_input(ts->pdata->irq_gpio);
#endif

	touch_hw_reset(ts, E_BOOTUP_DELAY_FAST);
#endif

#if KNOCK_ON_MODE
	/* 
	 * device is configured to stay partially awake in suspended state in order to trigger the wake up event 
	 * The device's .suspend function will contain the call to enable_irq_wake for the particular irq line 
	 * and the .resume function will contain a call to disable_irq_line. 
	 * So device can be partially functional in suspended state
	 */
	if(device_init_wakeup(&client->dev, ts->pdata->wakeup))
		dev_err(&client->dev, "\n\n\t device_init_wakeup error! \n\n\n");	
#endif 

#if defined(CONFIG_HAS_EARLYSUSPEND)
	if(ts->pdata->suspend)	ts->power.suspend	= ts->pdata->suspend;
	if(ts->pdata->resume)	ts->power.resume	= ts->pdata->resume;

	ts->power.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	
	register_early_suspend(&ts->power);
#endif

	if(ts->pdata->probe) {
#if PROBE_FAIL_CHECK
		if((rc = ts->pdata->probe(ts)) < 0) {
			dev_err(dev, "(%s) TSC probe failed! Unload driver!\n", ts->input->name);
			goto err_free_irq;
		}
#else
		ts->pdata->probe(ts);
#endif /* End of #if PROBE_FAIL_CHECK */
	}

	touch_info_display(ts);

#if defined(SEMISENS_TS_NATIVE_INTERFACE)
	if(semisens_ts_misc_probe(ts) < 0) {
		DBG_PRINT("semisens_ts_misc_probe(), fail\n");
	}
#endif
	
#if defined(CONFIG_MACH_FORTUNA_CMCC)
	if(1) {
		struct device *factory_ts_dev;
		
		factory_ts_dev = device_create(sec_class, NULL, 0, ts, "tsp");
		if(unlikely(!factory_ts_dev)) {
			dev_err(&ts->client->dev, "Failed to create factory dev\n");
			rc = -ENODEV;
			goto err_create_sysfs;
		}

		rc = sysfs_create_link(&factory_ts_dev->kobj, 
				&ts->input->dev.kobj, 
				"input");
		if(rc < 0) {
			dev_err(&ts->client->dev, "%s: Failed to create input symbolic link %d\n", __func__, rc);
		}
	}
#endif

	/* I2C Touch Probing is done, enable irq here */
	enable_irq(ts->irq);
	DBG_PRINT("touch irq(%d) is enabled!\n", ts->irq);

	DBG_PRINT("<<<< End");
	return 0;

err_create_sysfs:

#if PROBE_FAIL_CHECK
err_free_irq:
#endif
	free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
err_free_input_mem:
	input_free_device(ts->input);
	ts->input = NULL;
err_free_mem:
	kfree(ts->finger); ts->finger = NULL;
	kfree(ts); ts = NULL;
	return rc;
}

//----------------------------------------------
//
// Power Management function
//
//----------------------------------------------
#if defined(CONFIG_PM)
static int touch_suspend_pm(struct device *dev)
{
	struct touch *ts = dev_get_drvdata(dev);

#if KNOCK_ON_MODE
	unsigned short addr = 0x00F2;
	unsigned short data = 0xF0F0;

	/* set TSC to knock-on mode, and don't turn off both irq and power of TSC */
	if(device_may_wakeup(&ts->client->dev)) {
		DBG_PRINT("enable_irq_wake (%d)\n", ts->irq);
		enable_irq_wake(ts->irq);
	}

	DBG_PRINT("=== knock on sleep ===\n\n");

	g_knockOnFlag = 1;
	ts->pdata->i2c_write(ts->client, (u8 *)&addr, sizeof(addr), (u8 *)&data, sizeof(data));
#else
	/* TSC enters deep sleep mode */
	DBG_PRINT("touch resetb & power go down!\n\n");

	ts->pdata->disable(ts);

#if PINCTRL_ENABLE 
	if(ts->pinctrl) 
		pinctrl_configure(ts, false);
#endif

	/* power off */
	if(gpio_is_valid(ts->pdata->reset_gpio)) {
		gpio_direction_output(ts->pdata->reset_gpio, 0);
		gpio_set_value(ts->pdata->reset_gpio, 0); 
	}

	ts->pdata->power(ts, 0, true);
	mdelay(5);

#endif /* End of #if KNOCK_ON_MODE */

	return 0;
}

//----------------------------------------------
static int touch_resume_pm(struct device *dev)
{
	struct touch *ts = dev_get_drvdata(dev);

#if KNOCK_ON_MODE
	if(device_may_wakeup(&ts->client->dev)) {
		DBG_PRINT("disable_irq_wake (%d)\n", ts->irq);
		disable_irq_wake(ts->irq);
	}

	if(g_knockOnFlag)
	{
		/* set TSC to active mode by force */
		DBG_PRINT("=== forced wakeup from knock on sleep ===\n\n");

		/* power reset */
		touch_hw_reset(ts, E_BOOTUP_DELAY_NORMAL);

		g_knockOnFlag = 0;
	}
#else
	/* TSC enters active mode */
	DBG_PRINT("touch resetb & power go up!\n\n");

	/* power on */
	#if 0
	if(gpio_is_valid(ts->pdata->reset_gpio)) {
		gpio_direction_output(ts->pdata->reset_gpio, 0);
		gpio_set_value(ts->pdata->reset_gpio, 0); 
	}
	mdelay(1);

	ts->pdata->power(ts, 1, true);
	mdelay(5);

	if(gpio_is_valid(ts->pdata->reset_gpio)) {
		gpio_set_value(ts->pdata->reset_gpio, 1); 
	}
	mdelay(15);
	#else
	touch_hw_reset(ts, E_BOOTUP_DELAY_NORMAL);
	#endif

#if PINCTRL_ENABLE 
	if(ts->pinctrl) 
		pinctrl_configure(ts, true);
#endif

	ts->pdata->enable(ts);

#if CHARGER_INFO_ENABLE
	touch_charger_info_notify(ts);
#endif

#endif /* End of #if KNOCK_ON_MODE */

	return 0;
}
#endif /* End of #if defined(CONFIG_PM) */

#if defined(CONFIG_HAS_EARLYSUSPEND) 
static void touch_suspend(struct early_suspend *h)
{
	struct touch *ts = container_of(h, struct touch, power);

	touch_suspend_pm(&ts->client->dev);
}

//----------------------------------------------
static void touch_resume(struct early_suspend *h)
{
	struct touch *ts = container_of(h, struct touch, power);

	touch_resume_pm(&ts->client->dev);
}
#endif /* End of #if defined(CONFIG_HAS_EARLYSUSPEND) */

//----------------------------------------------
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0) 
s32 touch_remove(struct i2c_client *client)
#else
s32 touch_remove(struct device *dev)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0) 
	struct device *dev = &client->dev;
#endif

	struct touch *ts = dev_get_drvdata(dev);

#if KNOCK_ON_MODE
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)) 
	device_init_wakeup(&client->dev, 0);
	#else
	device_init_wakeup(dev, 0);
#endif
#endif

	if(ts->irq)	free_irq(ts->irq, ts);

	if(gpio_is_valid(ts->pdata->reset_gpio)) gpio_free(ts->pdata->reset_gpio);

	if(gpio_is_valid(ts->pdata->irq_gpio)) gpio_free(ts->pdata->irq_gpio);

	if(gpio_is_valid(ts->pdata->ldo_en_gpio)) gpio_free(ts->pdata->ldo_en_gpio);

	input_unregister_device(ts->input);
	
	dev_set_drvdata(dev, NULL);

#if defined(SEMISENS_TS_NATIVE_INTERFACE)
	semisens_ts_misc_remove();
#endif

	kfree(ts->finger); ts->finger = NULL;
	kfree(ts); ts = NULL;

	return 0;
}

#if defined(SEMISENS_TS_NATIVE_INTERFACE)
static const struct file_operations semisens_ts_misc_fops = 
{
	.owner = THIS_MODULE,
	.open = semisens_ts_misc_open,
	.unlocked_ioctl = semisens_ts_misc_ioctl,
};

static struct miscdevice semisens_ts_miscdev = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sn310m_dist",
	.fops = &semisens_ts_misc_fops,
};

static long semisens_ts_misc_ioctl(struct file *file, u32 cmd, unsigned long arg)
{
	packet_t* packet = (packet_t*)arg;
	s32 i;

	mutex_lock(&g_ts->mutex);
	switch(cmd)	{
		case 0: // write data
			if(packet->size) {
				u16 addr = (packet->addr >> 8) | (packet->addr & 0x00ff) << 8;
				g_ts->pdata->i2c_write(g_ts->client, (u8 *)&addr, sizeof(addr), (u8 *)packet->buf, packet->size*2);
			}
			break;

		case 1: // read data
			if(packet->size) {			
				u16 addr = (packet->addr >> 8) | (packet->addr & 0x00ff) << 8;
				s16 buffer[500] = {0, };	

				g_ts->pdata->i2c_read(g_ts->client, (u8 *)&addr, sizeof(addr), (u8 *)buffer, packet->size*2);
				for(i = 0; (i < packet->size) && (i < 500); i++) {
					packet->buf[i] = buffer[i];
				}
			}
			break;

		default:
			mutex_unlock(&g_ts->mutex);
			return -ENOIOCTLCMD;
	}

	mutex_unlock(&g_ts->mutex);
	return 0;
}

static s32 semisens_ts_misc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static s32 semisens_ts_misc_probe(struct touch *ts)
{
	s32 result = 0;

	g_ts = ts;
	result = misc_register(&semisens_ts_miscdev);
	if(result == 0) {
		DBG_PRINT("succeeded to register semisens_ts_miscdev!\n");
	}
	else {
		DBG_PRINT("failed to register semisens_ts_miscdev!\n");
	}

	return result;
}

static void semisens_ts_misc_remove(void)
{
	misc_deregister(&semisens_ts_miscdev);
	g_ts = NULL;
}
#endif

/* 20140723, by limst, you MUST implement power functions for touch contoller */
void ts_power_onoff(struct touch *ts, s32 onOff)
{
	s32 rc = 0;	

	/* regulator_enable() will return > zero when the regulator is enabled. */
	/* Power supplied by PMIC(power management integrated circuits) */ 
	if(ts->pdata->vdd) {
		if(onOff) {
			/* Power On */
			if(regulator_is_enabled(ts->pdata->vdd) == 0) {
				rc = regulator_enable(ts->pdata->vdd);
				if(rc) {	
					DBG_PRINT("VDD enabled failed!\n");
				}
				else { 									
					DBG_PRINT("VDD enabled!\n");
				}
			}
			else {
				DBG_PRINT("VDD already enabled!\n");
			}
		}
		else {
			/* Power Off */
			if(regulator_is_enabled(ts->pdata->vdd) > 0) {
				rc = regulator_disable(ts->pdata->vdd);
				if(rc) {
					DBG_PRINT("VDD disabled failed!\n");
				}
				else {
					DBG_PRINT("VDD disabled!\n");
				}
			}
		}
	} /* End of if(ts->pdata->vdd) */

	/* Power supplied by GPIO controlled LDO(low-dropout regulator) */
	if(gpio_is_valid(ts->pdata->ldo_en_gpio)) {
		gpio_direction_output(ts->pdata->ldo_en_gpio, onOff);
		gpio_set_value(ts->pdata->ldo_en_gpio, onOff); 
	}

	mdelay(5);
	return;
}

#if PINCTRL_ENABLE
static int pinctrl_configure(struct touch *ts, bool active)
{
	struct i2c_client *client = ts->client;
	struct pinctrl_state *set_state_tsp;

	int retval;

	dev_info(&client->dev,"%s: %s\n", __func__, active ? "ACTIVE" : "SUSPEND");

	if(active) {
		set_state_tsp =
			pinctrl_lookup_state(ts->pinctrl,
						"tsp_en_gpio_active");
		if(IS_ERR(set_state_tsp)) {
			dev_info(&client->dev,"%s: cannot get pinctrl(tsp_en_gpio) active state\n", __func__);
			return PTR_ERR(set_state_tsp);
		}

	} else {
		set_state_tsp =
			pinctrl_lookup_state(ts->pinctrl,
						"tsp_en_gpio_suspend");
		if(IS_ERR(set_state_tsp)) {
			dev_info(&client->dev,"%s: cannot get pinctrl(tsp_en_gpio) sleep state\n", __func__);
			return PTR_ERR(set_state_tsp);
		}
	}

	retval = pinctrl_select_state(ts->pinctrl, set_state_tsp);
	if(retval) {
		dev_info(&client->dev,"%s: cannot set pinctrl %s state\n",
				__func__, active ? "active" : "suspend");
		return retval;
	}

	return 0;
}
#endif /* End of #if PINCTRL_ENABLE */

static s32 ts_power_ctrl(struct touch *ts, s32 onOff, bool log_en)
{
	s32 rc = -EINVAL;

	if(log_en) {
		DBG_PRINT("[Touch Power]:[%s]\n", onOff ? "On" : "Off");
	}

	/* TODO: you must implement and add the power function for you system */
	ts_power_onoff(ts, onOff);

	mdelay(2); /* you should adjust delay for your system */
	return rc;
}

static s32 semisens_init_power(struct touch *ts)
{
	s32 rc = 0;

	DBG_PRINT("[TOUCH POWER] getting regulator information\n");

#if defined(CONFIG_MACH_STAR_SU660)
	struct regulator *iovdd = NULL;

	ts->pdata->vdd = regulator_get(NULL, "vcc_touch_3v1");		/* touch pvdd */
	if(IS_ERR(ts->pdata->vdd)) {
		ts->pdata->vdd = NULL;
		rc = PTR_ERR(ts->pdata->vdd);
		DBG_PRINT("Regulator for pvdd get failed rc=%d\n", rc);
		return rc;
	}

	ts->pdata->vcc_i2c = regulator_get(NULL, "vdd_onetouch");	/* touch sda/scl */
	if(IS_ERR(ts->pdata->vcc_i2c)) {
		ts->pdata->vcc_i2c = NULL;
		rc = PTR_ERR(ts->pdata->vcc_i2c);
		DBG_PRINT("Regulator for i2c get failed rc=%d\n", rc);
		return rc;
	}

	iovdd = regulator_get(NULL, "vcc_touch_1v8");				/* touch iovdd */
	if(IS_ERR(iovdd)) {
		iovdd = NULL;
		rc = PTR_ERR(iovdd);
		DBG_PRINT("Regulator for iovdd get failed rc=%d\n", rc);
		return rc;
	}
	
	if(regulator_enable(iovdd))
		DBG_PRINT("Touch IOVDD failed to turn on!\n");
	else
		DBG_PRINT("Touch IOVDD turned on!\n");

	regulator_set_voltage(ts->pdata->vcc_i2c, 1800000, 1800000);

	if(regulator_enable(ts->pdata->vcc_i2c))
		DBG_PRINT("Touch I2C Power failed to turn on!\n");
	else
		DBG_PRINT("Touch I2C Power turned on!\n");

#elif defined(CONFIG_MACH_FORTUNA_CMCC)
	#ifdef CONFIG_OF
	ts->pdata->vdd = regulator_get(&ts->client->dev, "vddo");
	#else
	ts->pdata->vdd = regulator_get(NULL, "pm8916_l6");
	#endif
	if(IS_ERR(ts->pdata->vdd)) {
		ts->pdata->vdd = NULL;
		rc = PTR_ERR(ts->pdata->vdd);
		DBG_PRINT("Regulator get failed vddo rc=%d\n", rc);
		return rc;
	}
	DBG_PRINT("Regulator get for VDDO success\n");

#endif 

	return rc;
}

#ifdef CONFIG_OF
static s32 semisens_parse_dt(struct device *dev, struct touch_pdata *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp;

	pdata->reset_gpio = of_get_named_gpio(np, "semisens,reset_gpio", 0); 
	if(pdata->reset_gpio <= 0) { /* assume that gpio for touch resetb is not assigned to 0 */
		pdata->reset_gpio = GPIO_FOR_TS_RESETB; 
	}

	pdata->irq_gpio = of_get_named_gpio(np, "semisens,irq_gpio", 0);
	pdata->ldo_en_gpio = of_get_named_gpio(np, "semisens,ldo_en_gpio", 0);

	if(pdata->irq_gpio < 0 || pdata->ldo_en_gpio < 0)
		return -EINVAL;

	of_property_read_u32(np, "semisens,x_resolution", &temp);
	pdata->abs_max_x = (u16)temp - 1;

	of_property_read_u32(np, "semisens,y_resolution", &temp);
	pdata->abs_max_y = (u16)temp - 1;

	return 0;
}
#endif /* End of #ifdef CONFIG_OF */

static s32 semisens_init_gpio(struct touch_pdata *pdata)
{
	s32 rc = -EINVAL;

	/* 1. Configuration for Reset GPIO */
	if(gpio_is_valid(pdata->reset_gpio)) {
		rc = gpio_request(pdata->reset_gpio, "touch_reset");
		if(rc < 0) {
			pr_err("%s: touch_reset(%d) gpio request fail\n", __func__, pdata->reset_gpio);
			return rc;
		}
		gpio_direction_output(pdata->reset_gpio, 1);
	}

	/* 2. Configuration for Interrupt GPIO */
	if(gpio_is_valid(pdata->irq_gpio)) {
		rc = gpio_request(pdata->irq_gpio, "touch_irq");
		if(rc < 0) {
			pr_err("%s: touch_irq(%d) gpio request fail\n", __func__, pdata->irq_gpio);
			return rc;
		}
		gpio_direction_input(pdata->irq_gpio);
	}

	/* 3. Configuration for LDO GPIO */
	if(gpio_is_valid(pdata->ldo_en_gpio)) {
		rc = gpio_request(pdata->ldo_en_gpio, "touch_ldo");
		if(rc < 0) {
			pr_err("%s: touch_ldo(%d) gpio request fail\n", __func__, pdata->ldo_en_gpio);
			return rc;
		}
		gpio_direction_output(pdata->ldo_en_gpio, 1);
	}
#if defined(CONFIG_ARCH_MSM) && defined(CONFIG_MACH_Q7LTE_KOR)
	gpio_tlmm_config(GPIO_CFG(pdata->ldo_en_gpio, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif

	/* 4. Configuration for Other GPIOs */

	return rc;
}

static const struct i2c_device_id semisens_ts_id[] = {
	{ SEMISENS_TS_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id semisens_match_table[] = {
	{ .compatible = "semisens,semisens_ts",},
	{ },
};
#else
#define semisens_match_table NULL
#endif

#if (INPUT_PM_ENABLE == 0) && (defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND))
static const struct dev_pm_ops semisens_ts_pm_ops = {
	#if defined(CONFIG_PM_RUNTIME)
	SET_RUNTIME_PM_OPS(touch_suspend_pm, touch_resume_pm, NULL)
	#else
	SET_SYSTEM_SLEEP_PM_OPS(touch_suspend_pm, touch_resume_pm)
	#endif
};
#endif

static struct i2c_driver semisens_touch_driver = {
	.driver	 = {
		.owner	= THIS_MODULE,
		.name   = SEMISENS_TS_NAME,
		.of_match_table = semisens_match_table,
#if (INPUT_PM_ENABLE == 0) && (defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND))
		.pm = &semisens_ts_pm_ops,
#endif
	},
	.probe   = touch_probe,
	.remove	 = touch_remove,
	.id_table = semisens_ts_id,
};


#if DEBUG_TSC_INFO && defined(SEMISENS_TS_NATIVE_INTERFACE)
#define DEBUG_INFO_LEN	(10)
struct task_struct *g_pKDbgThread = NULL;
static u16 gReadBuf[DEBUG_INFO_LEN + 1] = {0, };

void read_debug_info(void)
{
	u16 readAddr = 0x007F; /* Addr depends on TSC f/w */
	u32 i;

	mutex_lock(&g_ts->mutex);

	memset(gReadBuf, DEBUG_INFO_LEN * sizeof(short), 0x0);
	g_ts->pdata->i2c_read(g_ts->client, (u8 *)&readAddr, sizeof(readAddr), 
				(u8 *)gReadBuf, DEBUG_INFO_LEN * sizeof(u16));

	printk("\t======= Semisens TSC Info ======\n");
	for(i = 0; i < DEBUG_INFO_LEN; i++) {
		DBG_PRINT("dbg[%d]=[%d]\n", i, gReadBuf[i]);
	}
	printk("\t=======================\n\n");

	mutex_unlock(&g_ts->mutex);
}

static int KthreadToDbg(void * arg)
{
	msleep(500);
	DBG_PRINT(">>>> Start\n");
	while(!kthread_should_stop()) {
		/* Polling-Read Touch */
		if(g_ts != NULL && gpio_is_valid(g_ts->pdata->irq_gpio)) {
			DBG_PRINT(" INT pin %d(%d)\n", g_ts->pdata->irq_gpio, gpio_get_value(g_ts->pdata->irq_gpio));
			if(gpio_get_value(g_ts->pdata->irq_gpio) == 0) {
				semisens_ts_work(g_ts);
			}
		}

		if(g_ts->disabled == false)
		{
			read_debug_info();
			ssleep(1);
		}
		ssleep(1); 
	}

	DBG_PRINT("<<<< End\n");
	return 0;
}
#endif /* End of #if DEBUG_TSC_INFO && defined(SEMISENS_TS_NATIVE_INTERFACE) */

static s32 __init semisens_ts_init(void)
{
	DBG_PRINT("called");

#if DEBUG_TSC_INFO && defined(SEMISENS_TS_NATIVE_INTERFACE)
	if(g_pKDbgThread == NULL) { 
		g_pKDbgThread = (struct task_struct *)kthread_run(KthreadToDbg, NULL, "KThreadForTouch");
	}
#endif
	return i2c_add_driver(&semisens_touch_driver);
}

static void __exit semisens_ts_exit(void)
{
	DBG_PRINT("called");

#if DEBUG_TSC_INFO && defined(SEMISENS_TS_NATIVE_INTERFACE)
	if(g_pKDbgThread) {
		kthread_stop(g_pKDbgThread);
		g_pKDbgThread = NULL;
	}
#endif
	i2c_del_driver(&semisens_touch_driver);
}

module_init(semisens_ts_init);
module_exit(semisens_ts_exit);

MODULE_AUTHOR("sangtaeg.lim@semisens.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Semisens Touchscreen Controller Driver");
