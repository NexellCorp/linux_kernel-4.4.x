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

#ifndef __LINUX_REGULATOR_MP8845C_H
#define __LINUX_REGULATOR_MP8845C_H

#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

struct mp8845c_regulator {
	const char *name;
	int id;
	u8 reg_en_reg;
	u8 en_bit;
	u8 vout_en;
	u8 vout_reg;
	u8 vout_mask;
	u8 vout_reg_cache;
	int min_uV;
	int max_uV;
	int step_uV;
	int nsteps;
	u16 delay;
	u16 en_delay;
	u16 cmd_delay;
	struct regulator_desc desc;
	int const *voltages;
	int voltages_len;

	struct device *dev;
	struct i2c_client *client;
	struct regulator_dev *rdev;
	struct regmap *regmap;
};

struct mp8845c_regulator_platform_data {
		struct regulator_init_data 	*initdata;
		struct device_node 			*reg_node;
		int				id;
		int 			init_uV;
		unsigned 		init_enable:1;
		unsigned	 	init_apply:1;
		int 			sleep_uV;
		int 			sleep_slots;
		unsigned long 	ext_pwr_req;
		unsigned long 	flags;
};

#define mp8845c_rails(_name) "MP8845C_"#_name"_VOUT"

#endif
