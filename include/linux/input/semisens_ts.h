/****************************************************************
 *
 * semisens_ts.h : I2C Touchscreen device driver
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
#ifndef _SEMISENS_TS_H_
#define _SEMISENS_TS_H_

#ifdef CONFIG_HAS_EARLYSUSPEND
	#include <linux/earlysuspend.h>
#endif

#include <linux/interrupt.h>

#define TO_STR(X)		#X
#define SWAP_16BITS(p) \
	((((p) & 0xFF00) >> 8) | (((p) & 0x00FF) << 8))

typedef unsigned char           u8;
typedef unsigned short          u16;
typedef unsigned int            u32;
typedef unsigned long long      u64;
typedef signed char             s8;
typedef short                   s16;
typedef int                     s32;
typedef long long               s64;

//----------------------------------------------
// board-dependent define 
//----------------------------------------------
#define SEMISENS_TS_NAME			"semisens_ts"
#if defined(CONFIG_MACH_FORTUNA_CMCC)
	#define TS_INPUT_NAME			"sec_touchscreen"
#else
	#define TS_INPUT_NAME			SEMISENS_TS_NAME	
#endif

#define SEMISENS_TS_I2C_SLAVE_ADDR	(0x3c)
#define SEMISENS_TS_FINGER_MAX		5
#define SEMISENS_TS_KEY_MAX			3

#if !defined(CONFIG_OF)
#define SEMISENS_TS_RESOLUTION_X	1280
#define SEMISENS_TS_RESOLUTION_Y	800
#define I2C_BUS_ID_FOR_TS			(0) /* I2C channel(bus) connected to TSC */
#endif

/* NOTE: Unused or Unassigned GPIO pin should be set to -1 */
#define UNUSED_GPIO				(-1)
#define GPIO_FOR_TS_IRQ			UNUSED_GPIO /* the number of GPIO for TSC interrupt */
#define GPIO_FOR_TS_RESETB		UNUSED_GPIO
#define GPIO_FOR_TOUCH_LDO		UNUSED_GPIO /* vdd_en-gpio */

#if defined(CONFIG_ARCH_MSM) && defined(MSM_GPIO_TO_INT)
	#define	IRQ_NUM_FOR_TS_INTERRUPT	MSM_GPIO_TO_INT(GPIO_FOR_TS_IRQ)
#elif defined(CONFIG_ARCH_TEGRA)
	#define IRQ_NUM_FOR_TS_INTERRUPT	TEGRA_GPIO_TO_IRQ(GPIO_FOR_TS_IRQ)
#else
	#define IRQ_NUM_FOR_TS_INTERRUPT	(0)
#endif

//----------------------------------------------
// Common register address for firmware update
//----------------------------------------------
#define REG_ISP_MODE			0x02F1	// ISP mode control
#define REG_ISP_MODE_BUS		0x04F1	// ISP mode bus functions 
#define REG_ISP_MODE_ENABLE		0x08F1	// ISP mode enable 
#define REG_ISP_MEM_TYPE		0x00F1	// MEM TYPE: EEPROM/EFLASH

//----------------------------------------------
// EFLASH register address for firmware update
//----------------------------------------------
#define REG_CMD_FLASH_AUTH		0x00F4	// 0x0100 : get eFlash approach authority
#define REG_CMD_FLASH_CON_EN	0x02F4	// 0x0000 : enable eFlash controller
#define REG_CMD_FLASH_COMMAND	0x04F4	// 0x0200 : erase eFlash, 0x0000 : write eFlash
#define REG_CMD_FLASH_BUSY		0x08F4	// [15] bit is busy flag for eflash operation.

//----------------------------------------------
// EEPROM register address for firmware update
//----------------------------------------------
#define REG_CMD_EER_XPROT		0x00F4	// 0x0000 : get EEPROM approach authority
#define REG_CMD_EER_PDOWN		0x02F4	// 0x0000 : enable EEPROM controller
#define REG_CMD_EER_RESET		0x04F4	// 0x0000 : set EEPROM state to ACTIVE 
#define REG_CMD_EER_MODE		0x06F4	// 0x0007 : Enable EEPROM erase function 
										// 0x0008 : Enable EEPROM write function
#define REG_CMD_EER_XEN			0x08F4	// 0x0001 : EEPROM Excution Enable 
#define REG_CMD_EER_EXTEND		0x0EF4	// 0x0001 : EEPROM chip select control 
#define REG_CMD_EER_CSCON		0x10F4	// 0x0001 : EEPROM chip select control 
#define REG_CMD_EER_STATE		0x12F4	// [2] bit is busy flag for EEPROM operation. 
										// the value 0 of [2] bit means EEPROM busy 

//----------------------------------------------
// Touch status & data register address
//----------------------------------------------
#define	REG_TS_STATUS			0x00E0
#define REG_TS_GEST_STATUS		0x40E0
#define	REG_TS_DATA_BASE		0x02E0
#define	REG_TS_DATA(x)			(((x * 6) << 8) + REG_TS_DATA_BASE)
#define REG_FIRMWARE_VERSION	(0x3EE0)

//--------------------------------------------
// Touch Event type define
//--------------------------------------------
#define	TS_EVENT_UNKNOWN		0x00
#define	TS_EVENT_PRESS			0x01
#define	TS_EVENT_MOVE			0x02
#define	TS_EVENT_RELEASE		0x03

//--------------------------------------------
// Touch Interrupt mode define
//--------------------------------------------
#define	IRQ_MODE_THREAD			0
#define	IRQ_MODE_NORMAL			1
#define	IRQ_MODE_POLLING		2

//--------------------------------------------
// Touch Memory type define
//--------------------------------------------
#define REG_ISP_VAL_EEPROM		0xBABA	
#define REG_ISP_VAL_ERROR		0xDEAD	
enum {
	E_MEM_TYPE_EFLASH = 0,
	E_MEM_TYPE_EEPROM = 1,
	E_MEM_TYPE_MAX
};

//--------------------------------------------
// operation mode define for firmware update 
//--------------------------------------------
enum {
	E_FLASH_OPMODE_READ = 0,
	E_FLASH_OPMODE_WRITE = 1,
	E_FLASH_OPMODE_ERASE = 2,
	E_FLASH_OPMODE_MAX
};

//--------------------------------------------
// Button struct (1 = press, 0 = release)
//--------------------------------------------
typedef struct button__t {
	u8 bt0_press	:1; // lsb
	u8 bt1_press	:1;		
	u8 bt2_press	:1;
	u8 bt3_press	:1;
	u8 bt4_press	:1;
	u8 bt5_press	:1;
	u8 bt6_press	:1;
	u8 bt7_press	:1; // msb
} __attribute__ ((packed)) button_t;

typedef union button__u {
	u8			ubyte;
	button_t	bits;
} __attribute__ ((packed)) button_u;

typedef struct finger__t {
	u32 status;		// true : ts data updated, false : no update data
	u32 event;		// ts event type
	u32 id;			// ts received id
	u32 x;			// ts data x
	u32 y;			// ts data y
	u32 area;		// ts finger area
	u32 pressure;	// ts finger pressure
} __attribute__ ((packed)) finger_t;

typedef struct status_reg__t {
	u32 ts_cnt		:4;	// lsb
	u32 proximity	:4;
	u32 button		:5;
	u32 reserved2	:2;	
	u32 gesture		:1;	// msb
} __attribute__ ((packed)) status_reg_t;

typedef union status_reg__u {
	u16				uint;
	status_reg_t	bits;
}	__attribute__ ((packed)) status_reg_u;

typedef struct data_reg__t {
	u16 packet0;
	u16 packet1;
	u16 packet2;
} __attribute__ ((packed)) data_reg_t;

typedef union data_reg__u {
	u32			uint;
	data_reg_t	bits;
} __attribute__ ((packed)) data_reg_u;

struct tsp_callbacks {
	void (*inform_charger) (struct tsp_callbacks *cb, bool status);
};

struct touch {
	s32						irq;
	struct i2c_client		*client;
	struct touch_pdata		*pdata;
	struct input_dev		*input;
	char					phys[32];

	finger_t				*finger; // finger data
	struct mutex			mutex;

	u8						enabled; // interrupt status
	u8						fw_version;
	
	struct workqueue_struct	*work_queue;
	struct work_struct		work;
	
#if	defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	power;
#endif	

	void (*register_cb)		(void *callback); //charger info
	struct tsp_callbacks	callbacks; //charger info

	struct pinctrl			*pinctrl; //Pin mux 
};

struct i2c_client;
struct input_dev;
struct device;

//--------------------------------------------
// IRQ type & trigger action
//--------------------------------------------
//
// IRQF_TRIGGER_RISING, IRQF_TRIGGER_FALLING, IRQF_TRIGGER_HIGH, IRQF_TRIGGER_LOW
// IRQF_DISABLED, IRQF_SHARED, IRQF_IRQPOLL, IRQF_ONESHOT, IRQF_NO_THREAD
//
//--------------------------------------------
struct touch_pdata {
	char	*name;

	s32 	irq_gpio;
	s32 	reset_gpio;	
	s32		ldo_en_gpio;

	s32		irq_mode;
	s32 	irq_flags;			

	s32		abs_max_x, abs_max_y;
	s32 	abs_min_x, abs_min_y;
	
	s32 	area_max, area_min;
	s32		press_max, press_min;
	s32		id_max, id_min;
	
	s32		vendor, product, version;

	s32		max_fingers;
	
	s32		*keycode, keycnt;

	//--------------------------------------------
	// Control function 
	//--------------------------------------------
	irqreturn_t	(*irq_func)		(s32 irq, void *handle);
	void		(*touch_work)	(struct touch *ts);
	
	void 		(*report)		(struct touch *ts);
	void		(*key_report)	(struct touch *ts, u8 button_data);

	s32			(*early_probe)	(struct touch *ts);
	s32			(*probe)		(struct touch *ts);
	void		(*enable)		(struct touch *ts);
	void		(*disable)		(struct touch *ts);

	void		(*event_clear)	(struct touch *ts);

#ifdef	CONFIG_HAS_EARLYSUSPEND
	void		(*resume)		(struct early_suspend *h);
	void		(*suspend)		(struct early_suspend *h);
#endif

	void		(*register_cb)	(void *callback); 

	s32			(*power)		(struct touch *ts, s32 on, bool log_en);
	bool		wakeup;

	//--------------------------------------------
	// I2C control function
	//--------------------------------------------
	s32			(*i2c_write)	(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len);
	s32			(*i2c_read)		(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len);

	struct regulator 			*vdd;
	struct regulator			*vcc_i2c;
};

#endif /* end of #ifndef _SEMISENS_TS_H_ */
