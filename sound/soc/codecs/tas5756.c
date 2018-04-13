/*
 * TAS5086 ASoC codec driver
 *
 * Copyright (c) 2013 Daniel Mack <zonque@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * TODO:
 *  - implement DAPM and input muxing
 *  - implement modulation limit
 *  - implement non-default PWM start
 *
 * Note that this chip has a very unusual register layout, specifically
 * because the registers are of unequal size, and multi-byte registers
 * require bulk writes to take effect. Regmap does not support that kind
 * of devices.
 *
 * Currently, the driver does not touch any of the registers >= 0x20, so
 * it doesn't matter because the entire map can be accessed as 8-bit
 * array. In case more features will be added in the future
 * that require access to higher registers, the entire regmap H/W I/O
 * routines have to be open-coded.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "tas5756.h"

struct tas5756_private {
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	void *control_data;
	int mute_gpio;
	int spk_en_gpio;
};


#define RATES	SNDRV_PCM_RATE_8000_192000
#define FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)


#define TAS5756_3d_VOL_SPK			0x3d
#define TAS5756_3e_VOL_SPK			0x3e

static DECLARE_TLV_DB_SCALE(tas5756_tlv, -10350, 50, 0);
static const u8 tas5756_reg[1] = {
	0x3d,
};



static const struct snd_kcontrol_new tas5756_snd_controls[] = {
	SOC_SINGLE_TLV("Speaker Volume",TAS5756_3d_VOL_SPK ,
			0,255,0,tas5756_tlv),
	/*SOC_SINGLE_TLV("SpeakerB Volume", TAS5756_3e_VOL_SPK,
			0,255,0,tas5756_tlv),*/

};
static inline unsigned int tas5756_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	unsigned char data;
	unsigned char reg_char;
	reg_char=(unsigned char)reg;
	i2c_master_send(codec->control_data, &reg_char, 1);
	i2c_master_recv(codec->control_data, &data, 1);
	return data;
}
static inline void tas5756_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	cache[reg] = value;
}

static int tas5756_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data_a[2],data_b[2];
	data_a[0] = reg;
	data_a[1] = value;
	data_b[0] = reg+1;
	data_b[1] = value;
	tas5756_write_reg_cache(codec, reg, value);
	tas5756_write_reg_cache(codec, reg+1, value);
/*	if (!snd_soc_codec_is_active(codec))
		return 0;*/
	if (codec->hw_write(codec->control_data, data_a, 2) == 2 &&
		codec->hw_write(codec->control_data, data_b, 2) == 2) {
		unsigned int val_a,val_b;
		i2c_master_send(codec->control_data, &data_a[0], 1);
		i2c_master_recv(codec->control_data, &data_a[1], 1);
		val_a = data_a[1];
		i2c_master_send(codec->control_data, &data_b[0], 1);
		i2c_master_recv(codec->control_data, &data_b[1], 1);
		val_b = data_b[1];
		if (val_a!= value ||val_b!= value) {
			printk("tas5756: READ BACK VAL A:%x and B:%x\n",
					data_a[1],data_b[1]);
			return -EIO;
		}
		return 0;
	} else
		return -EIO;
}
static int tas5756_probe(struct snd_soc_codec *codec)
{
	struct tas5756_private *tas5756 = snd_soc_codec_get_drvdata(codec);
	tas5756->codec= codec;
	codec->hw_write = (hw_write_t)i2c_master_send;
	codec->control_data = tas5756->control_data;
	return 0;
}

static int tas5756_remove(struct snd_soc_codec *codec)
{
	return 0;
}


static struct snd_soc_codec_driver soc_codec_tas5756 = {
	.probe =	tas5756_probe,
	.remove =	tas5756_remove,
	.read =		tas5756_read_reg_cache,
	.write =	tas5756_write,
	.reg_cache_size = ARRAY_SIZE(tas5756_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = tas5756_reg,
	.reg_cache_step = 1,
	.controls = tas5756_snd_controls,
	.num_controls = ARRAY_SIZE(tas5756_snd_controls),
};

static struct snd_soc_dai_driver dummy_codec_dai[] = {
	{
		.name		= "dummy-aif1",
		.playback 	= {
			.stream_name	= "AIF1 Playback",
			.channels_min	= 1,
			.channels_max	= 384,
			.rates		= RATES,
			.formats	= FORMATS,
		},
		.capture 	= {
			.stream_name	= "AIF1 Capture",
			.channels_min	= 1,
			.channels_max	= 384,
			.rates		= RATES,
			.formats	= FORMATS,
		},
	}, {
		.name		= "dummy-aif2",
		.playback 	= {
			.stream_name	= "AIF2 Playback",
			.channels_min	= 1,
			.channels_max	= 384,
			.rates		= RATES,
			.formats	= FORMATS,
		},
		.capture 	= {
			.stream_name	= "AIF2 Capture",
			.channels_min	= 1,
			.channels_max	= 384,
			.rates		= RATES,
			.formats	= FORMATS,
		},
	}, {
		.name		= "dummy-aif3",
		.playback 	= {
			.stream_name	= "AIF3 Playback",
			.channels_min	= 1,
			.channels_max	= 384,
			.rates		= RATES,
			.formats	= FORMATS,
		},
		.capture 	= {
			.stream_name	= "AIF3 Capture",
			.channels_min	= 1,
			.channels_max	= 384,
			.rates		= RATES,
			.formats	= FORMATS,
		},
	}, {
		.name		= "dummy-aif4",
		.playback 	= {
			.stream_name	= "AIF4 Playback",
			.channels_min	= 1,
			.channels_max	= 384,
			.rates		= RATES,
			.formats	= FORMATS,
		},
		.capture 	= {
			.stream_name	= "AIF4 Capture",
			.channels_min	= 1,
			.channels_max	= 384,
			.rates		= RATES,
			.formats	= FORMATS,
		},
	}
};


static int tas5756_init(struct tas5756_private *priv)
{
	int ret, j;
	int num=sizeof(registers)/sizeof(registers[0]);

	dev_dbg(&priv->i2c->dev, "%s: i2c addr: 0x%x;\n",
			__func__, priv->i2c->addr);

	for (j = 0; j < num; j++)
	{
		ret = i2c_smbus_write_byte_data(priv->i2c,
					registers[j].command, registers[j].param);
		if (ret < 0) {
			dev_err(&priv->i2c->dev, "%s: i2c communication failed: "
					"addr(0x%x), register(0x%02x), data(0x%02x)\n", __func__,
					priv->i2c->addr, registers[j].command, registers[j].param);
			return ret;
		}
	}
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tas5756_dt_ids[] = {
	{ .compatible = "ti,tas5756m", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5756_dt_ids);
#endif

static const struct i2c_device_id tas5756_i2c_id[] = {
	{ "tas5756", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas5756_i2c_id);

static int tas5756_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct tas5756_private *priv;
	struct device *dev = &i2c->dev;
	struct device_node *np = i2c->dev.of_node;
	int ret = 0;

	priv = devm_kzalloc(dev, sizeof(struct tas5756_private), GFP_KERNEL);
	if (!priv) {
		dev_err(&i2c->dev, "%s: no enough memory\n", __func__);
		return -ENOMEM;
	}
	i2c_set_clientdata(i2c, priv);
	priv->i2c = i2c;
	priv->control_data=i2c;

    priv->spk_en_gpio = of_get_named_gpio(np, "tas5756m,spk_en_gpio", 0);
    if(priv->spk_en_gpio <= 0) { /* assume that gpio for touch resetb is not assigned to 0 */
        dev_warn(&i2c->dev, "%s: spk_en_gpio not find.\n", __func__);
        goto tas_init;
    }

    if(gpio_is_valid(priv->spk_en_gpio)) {
        ret = gpio_request(priv->spk_en_gpio, "tas5756m_spk_en");
        if(ret < 0) {
            pr_err("%s: tas5756m_en(%d) gpio request fail\n", __func__, priv->spk_en_gpio);
            return ret;
        }
        gpio_direction_output(priv->spk_en_gpio, 1);
    }

	priv->mute_gpio = of_get_named_gpio(np, "tas5756m,mute_gpio", 0);
	if(priv->mute_gpio <= 0) { /* assume that gpio for touch resetb is not assigned to 0 */
		dev_warn(&i2c->dev, "%s: mute_gpio not find.\n", __func__);
		goto tas_init;
	}

	if(gpio_is_valid(priv->mute_gpio)) {
		ret = gpio_request(priv->mute_gpio, "tas5756m_mute");
		if(ret < 0) {
			pr_err("%s: tas5756m_mute(%d) gpio request fail\n", __func__, priv->mute_gpio);
			return ret;
		}
		gpio_direction_output(priv->mute_gpio, 1);
	}

tas_init:
	dev_dbg(&i2c->dev, "%s: \n", __func__);
	tas5756_init(priv);
	ret=snd_soc_register_codec(&i2c->dev, &soc_codec_tas5756,
			dummy_codec_dai, ARRAY_SIZE(dummy_codec_dai));

	return ret;
}

static int tas5756_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static struct i2c_driver tas5756_i2c_driver = {
	.driver = {
		.name	= "tas5756",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tas5756_dt_ids),
	},
	.id_table	= tas5756_i2c_id,
	.probe		= tas5756_i2c_probe,
	.remove		= tas5756_i2c_remove,
};

module_i2c_driver(tas5756_i2c_driver);

MODULE_AUTHOR("Daniel Mack <zonque@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments TAS5756 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
