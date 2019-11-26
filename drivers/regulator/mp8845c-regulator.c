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

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/mp8845c-regulator.h>
#include <linux/mfd/mp8845c.h>

#ifdef FEATURE_ASV_CORE_TABLE
#include <linux/clk.h>

struct asv_core_tb_info {
	int ids;
	int ro;
	int uV;
};

static struct asv_core_tb_info asv_core_tables[] = {
#if 1 // REV0.3
	[0] = {	.ids = 6,	.ro = 90,	.uV = 1200000, },
	[1] = {	.ids = 15,	.ro = 130,	.uV = 1175000, },
	[2] = {	.ids = 38,	.ro = 170,	.uV = 1150000, },
	[3] = {	.ids = 78,	.ro = 200,	.uV = 1100000, },
	[4] = {	.ids = 78,	.ro = 200,	.uV = 1050000, },
#else // REV0.2
	[0] = {	.ids = 6,	.ro = 80,	.uV = 1200000, },
	[1] = {	.ids = 15,	.ro = 120,	.uV = 1175000, },
	[2] = {	.ids = 38,	.ro = 160,	.uV = 1150000, },
	[3] = {	.ids = 78,	.ro = 190,	.uV = 1100000, },
	[4] = {	.ids = 78,	.ro = 190,	.uV = 1050000, },
#endif
};

#define	ASV_CORE_ARRAY_SIZE	ARRAY_SIZE(asv_core_tables)


extern void nxp_cpu_id_string(u32 string[12]);
extern void nxp_cpu_id_ecid(u32 ecid[4]);

static inline unsigned int MtoL(unsigned int data, int bits)
{
	unsigned int result = 0;
	unsigned int mask = 1;
	int i = 0;
	for (i = 0; i<bits ; i++) {
		if (data&(1<<i))
			result |= mask<<(bits-i-1);
	}
	return result;
}

static void asv_core_setup(struct mp8845c_regulator *ri, struct mp8845c_regulator_platform_data *mp8845c_pdata)
{
	struct clk *clk = clk_get(NULL, "mpegbclk");
	unsigned long clk_rate;
	unsigned int ecid[4] = { 0, };
	//unsigned int string[12] = { 0, };
	int i, ids = 0, ro = 0;
	int idslv, rolv, asvlv;

	if(clk != NULL)
	{
		clk_rate = clk_get_rate(clk);
		// if(clk_rate > 400000000)
		{
			//nxp_cpu_id_string(string);
			nxp_cpu_id_ecid(ecid);

			/* Use IDS/Ro */
			ids = MtoL((ecid[1]>>16) & 0xFF, 8);
			ro  = MtoL((ecid[1]>>24) & 0xFF, 8);

			/* find IDS Level */
			for (i=0; i<(ASV_CORE_ARRAY_SIZE-1); i++)
			{
				if (ids <= asv_core_tables[i].ids)
					break;
			}
			idslv = ASV_CORE_ARRAY_SIZE != i ? i: (ASV_CORE_ARRAY_SIZE-1);

			/* find RO Level */
			for (i=0; i<(ASV_CORE_ARRAY_SIZE-1); i++)
			{
				if (ro <= asv_core_tables[i].ro)
					break;
			}
			rolv = ASV_CORE_ARRAY_SIZE != i ? i: (ASV_CORE_ARRAY_SIZE-1);

			/* find Lowest ASV Level */
			asvlv = idslv > rolv ? rolv: idslv;

			if(asvlv <= (ASV_CORE_ARRAY_SIZE-1))
				mp8845c_pdata->init_uV = asv_core_tables[asvlv].uV;

			dev_info(ri->dev, "IDS(%dmA) RO(%d) ASV(%d) Vol(%duV) CLK(%luHz) \n", ids, ro, asvlv, mp8845c_pdata->init_uV, clk_rate);
		}
	}

	return;
}
#endif

static const int vout_uV_list[] = {
	6000,    // 0
	6067,
	6134,
	6201,
	6268,
	6335,
	6401,
	6468,
	6535,
	6602,
	6669,    // 10
	6736,
	6803,
	6870,
	6937,
	7004,
	7070,
	7137,
	7204,
	7271,
	7338,    // 20
	7405,
	7472,
	7539,
	7606,
	7673,
	7739,
	7806,
	7873,
	7940,
	8007,    // 30
	8074,
	8141,
	8208,
	8275,
	8342,
	8408,
	8475,
	8542,
	8609,
	8676,   // 40
	8743,
	8810,
	8877,
	8944,
	9011,
	9077,
	9144,
	9211,
	9278,
	9345,   // 50
	9412,
	9479,
	9546,
	9613,
	9680,
	9746,
	9813,
	9880,
	9947,
	10014,   // 60
	10081,
	10148,
	10215,
	10282,
	10349,
	10415,
	10482,
	10549,
	10616,
	10683,   // 70
	10750,
	10817,
	10884,
	10951,
	11018,
	11084,
	11151,
	11218,
	11285,
	11352,   // 80
	11419,
	11486,
	11553,
	11620,
	11687,
	11753,
	11820,
	11887,
	11954,
	12021,   // 90
	12088,
	12155,
	12222,
	12289,
	12356,
	12422,
	12489,
	12556,
	12623,
	12690,   // 100
	12757,
	12824,
	12891,
	12958,
	13025,
	13091,
	13158,
	13225,
	13292,
	13359,   // 110
	13426,
	13493,
	13560,
	13627,
	13694,
	13760,
	13827,
	13894,
	13961,
	14028,   // 120
	14095,
	14162,
	14229,
	14296,
	14363,
	14429,
	14496,
};

#define to_i2c_client(d) container_of(d, struct i2c_client, dev)

static int mp8845c_regulator_enable_time(struct regulator_dev *rdev)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);

	return ri->en_delay;
}

static int mp8845c_reg_is_enabled(struct regulator_dev *rdev)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);
	uint8_t control = 0;
	int ret;

	ret = mp8845c_read(ri->client, ri->reg_en_reg, &control);
	if (ret < 0) {
		dev_err(&rdev->dev, "\e[31mError\e[0m: in %s()!\n", __func__);
		return ret;
	}
	return (((control >> ri->en_bit) & 1) == 1);
}

static int mp8845c_reg_enable(struct regulator_dev *rdev)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);
	int ret;

	ret = mp8845c_set_bits(ri->client, ri->reg_en_reg, (1 << ri->en_bit));
	if (ret < 0)
		dev_err(&rdev->dev, "\e[31mError\e[0m: in %s()!\n", __func__);
	else
		ri->vout_en = 1;

	return ret;
}

static int mp8845c_reg_disable(struct regulator_dev *rdev)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);
	int ret;

	ret = mp8845c_clr_bits(ri->client, ri->reg_en_reg, (1 << ri->en_bit));
	if (ret < 0)
		dev_err(&rdev->dev, "\e[31mError\e[0m: in %s()!\n", __func__);
	else
		ri->vout_en = 0;

	return ret;
}

static int mp8845c_list_voltage(struct regulator_dev *rdev, unsigned index)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);
	int ret = 0;

	//ret = ri->min_uV + (ri->step_uV * index);

	ret = ri->voltages[index]*100;

	return ret;
}

static int __mp8845c_set_voltage(struct mp8845c_regulator *ri, int min_uV, int max_uV, unsigned *selector)
{
	int i,j;
	int vsel;
	int ret;
	uint8_t vout_val;

	if ((min_uV < ri->min_uV) || (max_uV > ri->max_uV))
		return -EDOM;

	j = ((min_uV - ri->min_uV + ri->step_uV)/ri->step_uV)-1;
	for(i=j; i<ri->voltages_len; i++)
	{
		if(min_uV <= ri->voltages[i]*100)
			break;
	}
	vsel = i;

	if (vsel > ri->nsteps)
		return -EDOM;

	if (selector)
		*selector = vsel;

	vout_val = (ri->vout_en << ri->en_bit)|(ri->vout_reg_cache & ~ri->vout_mask)|(vsel & ri->vout_mask);

	mp8845c_set_bits(ri->client, MP8845C_REG_SYSCNTL2, (1 << MP8845C_POS_GO));
	ret = mp8845c_write(ri->client, ri->vout_reg, vout_val);

	if (ret < 0)
		dev_err(ri->dev, "\e[31mError\e[0m: in writing the Voltage register\n");
	else
		ri->vout_reg_cache = vout_val;

	return ret;
}

static int mp8845c_set_voltage_time_sel(struct regulator_dev *rdev, unsigned int old_sel, unsigned int new_sel)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);

	if (old_sel < new_sel)
		return ((new_sel - old_sel) * ri->delay) + ri->cmd_delay;

	return 0;
}

#if 1
static int mp8845c_set_voltage_sel(struct regulator_dev *rdev, unsigned selector)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);
	int uV;

	//uV = ri->min_uV + (ri->step_uV * selector);
	uV = ri->voltages[selector]*100;

	return __mp8845c_set_voltage(ri, uV, uV, NULL);
}

static int mp8845c_get_voltage_sel(struct regulator_dev *rdev)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);
	uint8_t vsel = 0;
	uint8_t vout_val = 0;
	int ret = 0;

	ret = mp8845c_read(ri->client, ri->vout_reg, &vout_val);
	if(ret < 0)
		vsel = ri->vout_reg_cache & ri->vout_mask;
	else
		vsel = vout_val & ri->vout_mask;

	return vsel;
}
#else
static int mp8845c_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV, unsigned *selector)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);

	return __mp8845c_set_voltage(ri, min_uV, max_uV, selector);
}

static int mp8845c_get_voltage(struct regulator_dev *rdev)
{
	struct mp8845c_regulator *ri = rdev_get_drvdata(rdev);
	uint8_t vsel;

	vsel = ri->vout_reg_cache & ri->vout_mask;
	return ri->min_uV + vsel * ri->step_uV;
}
#endif

static struct regulator_ops mp8845c_vout_ops = {
	.list_voltage		= mp8845c_list_voltage,
#if 1
	.set_voltage_sel	= mp8845c_set_voltage_sel,
	.set_voltage_time_sel = mp8845c_set_voltage_time_sel,
	.get_voltage_sel	= mp8845c_get_voltage_sel,
#else
	.set_voltage		= mp8845c_set_voltage,
	.get_voltage		= mp8845c_get_voltage,
#endif
	.enable				= mp8845c_reg_enable,
	.disable			= mp8845c_reg_disable,
	.is_enabled			= mp8845c_reg_is_enabled,
	.enable_time		= mp8845c_regulator_enable_time,
};

#define MP8845C_REG(_name, _id, _name_id, _en_reg, _en_bit, _vout_reg, _vout_mask,	\
				_min_mv, _max_mv, _step_uV, _nsteps, _delay, _cmd_delay, _ops)	\
{											\
	.name			= #_name,				\
	.id				= MP8845C_##_id##_VOUT,	\
	.reg_en_reg		= _en_reg,				\
	.en_bit			= _en_bit,				\
	.vout_reg		= _vout_reg,			\
	.vout_mask		= _vout_mask,			\
	.min_uV			= _min_mv * 1000,		\
	.max_uV			= _max_mv * 1000,		\
	.step_uV		= _step_uV,				\
	.nsteps			= _nsteps, 				\
	.delay			= _delay,				\
	.cmd_delay		= _cmd_delay,			\
	.desc = {								\
		.name		= mp8845c_rails(_id),	\
		.id			= MP8845C_##_id##_VOUT,	\
		.n_voltages = _nsteps,				\
		.ops		= _ops,					\
		.type		= REGULATOR_VOLTAGE,	\
		.owner		= THIS_MODULE,			\
	},										\
	.voltages		= vout_uV_list,			\
	.voltages_len		= ARRAY_SIZE(vout_uV_list),	\
}

static struct mp8845c_regulator mp8845c_regulators[] =
{
	MP8845C_REG(vout1, 0, 1,
				MP8845C_REG_VSEL,
				MP8845C_POS_EN,
				MP8845C_REG_VSEL,
				MP8845C_POS_OUT_VOL_MASK,
				600, 1450, 6700, 0x80, 1, 0,
				&mp8845c_vout_ops),

	MP8845C_REG(vout2, 1, 2,
				MP8845C_REG_VSEL,
				MP8845C_POS_EN,
				MP8845C_REG_VSEL,
				MP8845C_POS_OUT_VOL_MASK,
				600, 1450, 6700, 0x80, 1, 0,
				&mp8845c_vout_ops),
};

static inline struct mp8845c_regulator *find_regulator_info(int id)
{
	struct mp8845c_regulator *ri;
	int i;

	for (i = 0; i < ARRAY_SIZE(mp8845c_regulators); i++) {
		ri = &mp8845c_regulators[i];
		if (ri->desc.id == id)
			return ri;
	}
	return NULL;
}

static int mp8845c_regulator_preinit(struct mp8845c_regulator *ri, struct mp8845c_regulator_platform_data *mp8845c_pdata)
{
	int ret = 0;

#ifdef FEATURE_ASV_CORE_TABLE
	if(!strcmp(ri->name, "vout2"))
		return ret;

//	asv_core_setup(ri, mp8845c_pdata);
#endif

	mp8845c_set_bits(ri->client, MP8845C_REG_SYSCNTL1, (1 << MP8845C_POS_MODE));

	if (mp8845c_pdata->init_enable)
	{
		ret = mp8845c_set_bits(ri->client, ri->reg_en_reg, (1 << ri->en_bit));

		if (ret < 0)
			dev_err(ri->dev, "\e[31mError\e[0m: Not able to enable rail %d err %d\n", ri->desc.id, ret);
		else
			ri->vout_en = 1;
	}
	else
	{
		ret = mp8845c_clr_bits(ri->client, ri->reg_en_reg, (1 << ri->en_bit));
		if (ret < 0)
			dev_err(ri->dev, "\e[31mError\e[0m: Not able to disable rail %d err %d\n", ri->desc.id, ret);
		else
			ri->vout_en = 0;
	}

	if (mp8845c_pdata->init_uV > -1) {
		ret = __mp8845c_set_voltage(ri, mp8845c_pdata->init_uV, mp8845c_pdata->init_uV, NULL);
		if (ret < 0) {
			dev_err(ri->dev, "\e[31mError\e[0m: Not able to initialize voltage %d for rail %d err %d\n",
								mp8845c_pdata->init_uV, ri->desc.id, ret);
			return ret;
		}
	}

	return ret;
}


static inline int mp8845c_cache_regulator_register(struct mp8845c_regulator *ri)
{
	ri->vout_reg_cache = 0;
	return mp8845c_read(ri->client, ri->vout_reg, &ri->vout_reg_cache);
}


#ifdef CONFIG_OF
static int mp8845c_regulator_dt_parse_pdata(struct platform_device *pdev, struct mp8845c_platform_data *pdata)
{
	struct mp8845c *iodev = dev_get_drvdata(pdev->dev.parent);
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct mp8845c_regulator_platform_data *rdata;

	unsigned int i;
	u32 val;

	const char *devname = NULL;
	devname = dev_name(iodev->dev);


	pmic_np = of_node_get(iodev->dev->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(&pdev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	/* count the number of regulators to be supported in pmic */
	pdata->num_regulators = of_get_child_count(regulators_np);

	rdata = devm_kzalloc(&pdev->dev, sizeof(*rdata)*pdata->num_regulators, GFP_KERNEL);
	if (!rdata) {
		of_node_put(regulators_np);
		dev_err(&pdev->dev, "could not allocate memory for regulator data\n");
		return -ENOMEM;
	}

	pdata->regulators = rdata;

	for_each_child_of_node(regulators_np, reg_np)
	{
		for (i = 0; i < ARRAY_SIZE(mp8845c_regulators); i++)
			if (!of_node_cmp(reg_np->name, mp8845c_regulators[i].name))
				break;
		if (i == ARRAY_SIZE(mp8845c_regulators)) {
			dev_warn(&pdev->dev, "\e[31mError\e[0m : don't know how to configure regulator %s\n",
				 reg_np->name);
			continue;
		}
		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(&pdev->dev, reg_np, &mp8845c_regulators[i].desc);
		rdata->reg_node = reg_np;

		if(!of_property_read_u32(reg_np, "nx,id", &val))
			rdata->id 	= val;
		else
			dev_err(&pdev->dev, "%s() \e[31mError\e[0m : id \n", __func__);

		//rdata->consumer.supply = of_get_property(reg_np, "nx,supply_name", NULL);
		//rdata->initdata->num_consumer_supplies = 1;
		//rdata->initdata->consumer_supplies = &rdata->consumer;//of_get_property(reg_np, "nx,consurem_supply", NULL);
		//rdata->initdata->supply_regulator = of_get_property(reg_np, "nx,supply_regulator", NULL);
		//rdata->supply_name = of_get_property(reg_np, "nx,supply_regulator", NULL);


//		if(!of_property_read_u32(reg_np, "nx,always_on", &val))
//			rdata->initdata->constraints.always_on = val;
//		else
//			dev_err(&pdev->dev, "%s() \e[31mError\e[0m : always_on \n", __func__);

//		if(!of_property_read_u32(reg_np, "nx,boot_on", &val))
//			rdata->initdata->constraints.boot_on = val;
//		else
//			dev_err(&pdev->dev, "%s() \e[31mError\e[0m : always_on \n", __func__);

		if(!of_property_read_u32(reg_np, "nx,init_enable", &val))
			rdata->init_enable 	= val;
		else
			dev_err(&pdev->dev, "%s() \e[31mError\e[0m : init_enable \n", __func__);

		if(!of_property_read_u32(reg_np, "nx,init_uV", &val))
			rdata->init_uV 	= (int)val;
		else
			dev_err(&pdev->dev, "%s() \e[31mError\e[0m : init_uV \n", __func__);

		rdata++;
	}
	of_node_put(regulators_np);

	return 0;
}
#else
static int mp8845c_regulator_dt_parse_pdata(struct platform_device *pdev,
					struct mp8845c_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */


static int mp8845c_regulator_probe(struct platform_device *pdev)
{
	struct mp8845c *iodev = dev_get_drvdata(pdev->dev.parent);
	struct mp8845c_platform_data *pdata = iodev->pdata;

	struct mp8845c_regulator *mp8845c = NULL;
	struct mp8845c_regulator_platform_data *tps_pdata;

	struct regulator_config config = { };
	int i, ret=-EIO, err;

	if (iodev->dev->of_node) {
		err = mp8845c_regulator_dt_parse_pdata(pdev, pdata);
		if (err)
		{
			dev_err(&pdev->dev, "%s() \e[31mError\e[0m: No parse pdata\n", __func__);
			return err;
		}
	}
	else
	{
		dev_err(&pdev->dev, "%s() \e[31mError\e[0m: Not find of_node\n", __func__);
		goto err_out;
	}

	tps_pdata = pdata->regulators;

	for (i = 0; i < pdata->num_regulators; i++) {
		mp8845c = find_regulator_info(tps_pdata->id);
		if (mp8845c == NULL) {
			dev_err(&pdev->dev, "%s() \e[31mError\e[0m: invalid regulator ID specified\n", __func__);
			return -EINVAL;
		}

		mp8845c->client = iodev->client;
		mp8845c->dev = &pdev->dev;

		err = mp8845c_cache_regulator_register(mp8845c);
		if (err) {
			dev_err(&pdev->dev, "%s() \e[31mError\e[0m: Fail in caching register\n", __func__);
			return err;
		}

		err = mp8845c_regulator_preinit(mp8845c, tps_pdata);
		if (err) {
			dev_err(&pdev->dev, "%s() \e[31mError\e[0m: Fail in pre-initialisation\n", __func__);
			return err;
		}

		/* Register the regulators */
		config.dev = &pdev->dev;
		config.init_data = tps_pdata->initdata;;
		config.driver_data = mp8845c;
		config.of_node = tps_pdata->reg_node;

		mp8845c_regulators[i].rdev = regulator_register(&mp8845c->desc, &config);
		if (IS_ERR(mp8845c_regulators[i].rdev)) {
			dev_err(&pdev->dev, "%s() \e[31mError\e[0m: regulator register failed\n", __func__);
			ret = PTR_ERR(mp8845c_regulators[i].rdev);
			goto err;
		}
		tps_pdata++;
	}

	return 0;
err:
	while (--i >= 0)
		regulator_unregister(mp8845c_regulators[i].rdev);
err_out:
	dev_err(&pdev->dev, "%s() \e[31mError\e[0m \n", __func__);
	return ret;
}

static int mp8845c_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);

	return 0;
}

#ifdef CONFIG_PM
static int mp8845c_regulator_suspend(struct device *dev)
{
	return 0;
}

static int mp8845c_regulator_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops mp8845c_regulator_pm_ops = {
	.suspend	= mp8845c_regulator_suspend,
	.resume		= mp8845c_regulator_resume,
};
#endif

static struct platform_driver mp8845c_regulator_driver = {
	.driver	= {
		.name	= "mp8845c-regulator",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &mp8845c_regulator_pm_ops,
#endif
	},
	.probe		= mp8845c_regulator_probe,
	.remove		= mp8845c_regulator_remove,
};

static int __init mp8845c_regulator_init(void)
{
	return platform_driver_register(&mp8845c_regulator_driver);
}
subsys_initcall(mp8845c_regulator_init);

static void __exit mp8845c_regulator_exit(void)
{
	platform_driver_unregister(&mp8845c_regulator_driver);
}
module_exit(mp8845c_regulator_exit);

MODULE_DESCRIPTION("MP8845C current regulator driver");
MODULE_AUTHOR("pjsin865@nexell.co.kr");
MODULE_LICENSE("GPL");

