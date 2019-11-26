/*
 * mp8845c-regulator.h  --  Regulator driver for the mp8845c
 *
 * Copyright (C) 2009. Nexell Co., <pjsin865@nexell.co.kr>
 *
 * See file CREDITS for list of people who contributed to this project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __LINUX_MP8845C_H
#define __LINUX_MP8845C_H

#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

/*
 * register map
 */
#define	MP8845C_REG_VSEL				0x00
#define	MP8845C_REG_SYSCNTL1			0x01
#define	MP8845C_REG_SYSCNTL2			0x02
#define	MP8845C_REG_ID1					0x03
#define	MP8845C_REG_ID2					0x04
#define	MP8845C_REG_STATUS				0x05


/*
 * register bit position
 */
/* VSEL */
#define	MP8845C_POS_EN					(7)
#define	MP8845C_POS_OUT_VOL				(0)
#define	MP8845C_POS_OUT_VOL_MASK		(0x7F)

/* SYSCNTLREG1 */
#define	MP8845C_POS_SWITCHING_FREQ		(5)
#define	MP8845C_POS_PG_LOHI				(2)
#define	MP8845C_POS_VIN_OVP				(1)
#define	MP8845C_POS_MODE				(0)

/* SYSCNTLREG2 */
#define	MP8845C_POS_GO					(5)
#define	MP8845C_POS_OUTPUT_DISCHARGE	(4)
#define	MP8845C_POS_SLEW_RATE			(0)

/* ID1 */
#define	MP8845C_POS_VENDOR_ID			(4)
#define	MP8845C_POS_DIE_ID				(0)

/* ID2 */
#define	MP8845C_POS_DIE_REV				(0)

/* STATUS */
#define	MP8845C_POS_ILIM				(7)
#define	MP8845C_POS_UVLO				(6)
#define	MP8845C_POS_OVP					(5)
#define	MP8845C_POS_VOOV				(4)
#define	MP8845C_POS_VOUV				(3)
#define	MP8845C_POS_PGOOD				(2)
#define	MP8845C_POS_OTW					(1)
#define	MP8845C_POS_EN_STAT				(0)

enum mp8845c_id {
	MP8845C_0_VOUT,
	MP8845C_1_VOUT,
};

struct mp8845c_platform_data {
	int			id;
	int 		num_regulators;
	void		*platform_data;
	const char	*name;
	struct mp8845c_regulator_platform_data *regulators;
};

struct mp8845c {
	struct device		*dev;
	struct mp8845c_platform_data *pdata;
	struct i2c_client	*client;
	struct mutex		io_lock;
};


extern int mp8845c_read(struct i2c_client *client, u8 reg, uint8_t *val);
extern int mp8845c_write(struct i2c_client *client, u8 reg, uint8_t val);
extern int mp8845c_set_bits(struct i2c_client *client, u8 reg, uint8_t bit_mask);
extern int mp8845c_clr_bits(struct i2c_client *client, u8 reg, uint8_t bit_mask);
extern int mp8845c_update(struct i2c_client *client, u8 reg, uint8_t val, uint8_t mask);

#endif
