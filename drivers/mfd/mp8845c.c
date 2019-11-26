/*
 * mp8845c-regulator.c  --  Regulator driver for the mp8845c
 *
 * Copyright(C) 2009. Nexell Co., <pjsin865@nexell.co.kr>
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <linux/mfd/core.h>
#include <linux/mfd/mp8845c.h>

int mp8845c_read(struct i2c_client *client, u8 reg, uint8_t *val)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "\e[31mError\e[0m: reading at 0x%02x\n",
		reg);
		return ret;
	}

	*val = (uint8_t)ret;

	dev_dbg(&client->dev, "read reg=%x, val=%x\n", reg, *val);

	return 0;
}
EXPORT_SYMBOL_GPL(mp8845c_read);

int mp8845c_write(struct i2c_client *client, u8 reg, uint8_t val)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(client, reg, val);

	if (ret < 0) {
		dev_err(&client->dev, "\e[31mError\e[0m: writing 0x%02x to 0x%02x\n",
		val, reg);
		return ret;
	}

	dev_dbg(&client->dev, "write reg=%x, val=%x\n", reg, val);

	return 0;
}
EXPORT_SYMBOL_GPL(mp8845c_write);

int mp8845c_set_bits(struct i2c_client *client, u8 reg, uint8_t bit_mask)
{
	uint8_t reg_val;
	int ret = 0;

	ret = mp8845c_read(client, reg, &reg_val);

	if (ret < 0)
		return ret;

	if ((reg_val & bit_mask) != bit_mask) {
		reg_val |= bit_mask;
		ret = mp8845c_write(client, reg, reg_val);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(mp8845c_set_bits);

int mp8845c_clr_bits(struct i2c_client *client, u8 reg, uint8_t bit_mask)
{
	uint8_t reg_val;
	int ret = 0;

	ret = mp8845c_read(client, reg, &reg_val);

	if (ret < 0)
		return ret;

	if (reg_val & bit_mask) {
		reg_val &= ~bit_mask;
		ret = mp8845c_write(client, reg, reg_val);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(mp8845c_clr_bits);

int mp8845c_update(struct i2c_client *client, u8 reg, uint8_t val, uint8_t mask)
{
	uint8_t reg_val;
	int ret = 0;

	ret = mp8845c_read(client, reg, &reg_val);

	if (ret < 0)
		return ret;

	if ((reg_val & mask) != val) {
		reg_val = (reg_val & ~mask) | (val & mask);
		ret = mp8845c_write(client, reg, reg_val);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(mp8845c_update);


static struct mfd_cell mp8845c_devs[] = {
	{
		.name = "mp8845c-regulator",
		.id = 0,
	},
	{
		.name = "mp8845c-regulator",
		.id = 1,
	},
};

#ifdef CONFIG_OF
static const struct of_device_id mp8845c_pmic_dt_match[] = {
	{.compatible = "nx,mp8845c", .data = NULL},
	{},
};

static struct mp8845c_platform_data *mp8845c_i2c_parse_dt_pdata
	(struct device *dev)
{
	struct mp8845c_platform_data *pd;
	struct device_node *np;
	u32 val;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return ERR_PTR(-ENOMEM);

	np = of_node_get(dev->of_node);

	if (!of_property_read_u32(np, "nx,id", &val))
		pd->id = val;
	else
		dev_err(dev, "%s() \e[31mError\e[0m : id\n", __func__);

	/*
	 * ToDo: the 'wakeup' member in the platform data is more of a linux
	 * specfic information. Hence, there is no binding for that yet and
	 * not parsed here.
	 */

	return pd;
}
#else
static struct mp8845c_platform_data *mp8845c_i2c_parse_dt_pdata
	(struct device *dev)
{
	return 0;
}
#endif


static int mp8845c_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct mp8845c *mp8845c;
	struct mp8845c_platform_data *pdata;
	int ret = -EIO;

	mp8845c = kzalloc(sizeof(struct mp8845c), GFP_KERNEL);
	if (mp8845c == NULL)
		return -ENOMEM;

	mp8845c->dev = &client->dev;
	mp8845c->client = client;

	i2c_set_clientdata(client, mp8845c);

	if (client->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(mp8845c_pmic_dt_match),
			&client->dev);
		if (!match) {
			dev_err(&client->dev, "%s() \e[31mError\e[0m: No device match found\n",
				__func__);
			goto error;
		}
	} else
		dev_err(&client->dev, "%s() \e[31mError\e[0m: No device match found\n",
			__func__);

	if (mp8845c->dev->of_node) {
		pdata = mp8845c_i2c_parse_dt_pdata(mp8845c->dev);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			dev_err(&client->dev, "%s() \e[31mError\e[0m: could not allocate memory for pdata\n",
				__func__);
			goto error;
		}
	}

	if (!pdata) {
		dev_err(&client->dev, "%s() \e[31mError\e[0m: could not allocate memory for pdata\n",
			__func__);
		goto error;
	}

	mp8845c->pdata = pdata;

	mutex_init(&mp8845c->io_lock);

	ret = mfd_add_devices(mp8845c->dev, -1, &mp8845c_devs[pdata->id],
		1, NULL, 0, NULL);

	if (ret) {
		dev_err(&client->dev, "%s() \e[31mError\e[0m: add devices failed: %d\n",
			__func__, ret);
		goto error;
	}

	return 0;

error:
	kfree(mp8845c);
	return ret;
}

static int mp8845c_i2c_remove(struct i2c_client *client)
{
	struct mp8845c *mp8845c = i2c_get_clientdata(client);

	kfree(mp8845c);

	return 0;
}

#ifdef CONFIG_PM
static int mp8845c_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int mp8845c_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static const struct i2c_device_id mp8845c_i2c_id[] = {
	{"mp8845c", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, mp8845c_i2c_id);

static struct i2c_driver mp8845c_i2c_driver = {
	.driver = {
		   .name = "mp8845c",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(mp8845c_pmic_dt_match),
		   },
	.probe = mp8845c_i2c_probe,
	.remove = mp8845c_i2c_remove,
	.id_table = mp8845c_i2c_id,
};

static int __init mp8845c_i2c_init(void)
{
	int ret = -ENODEV;

	ret = i2c_add_driver(&mp8845c_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(mp8845c_i2c_init);

static void __exit mp8845c_i2c_exit(void)
{
	i2c_del_driver(&mp8845c_i2c_driver);
}
module_exit(mp8845c_i2c_exit);

MODULE_DESCRIPTION("MPS MP8845C multi-function core driver");
MODULE_LICENSE("GPL");
