/*
 * (C) Copyright 2009
 * Hyunseok Jung, Nexell Co, <hsjung@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/jack.h>

#include "nexell-spdif.h"
#include "nexell-pcm.h"

/*
 * SPDIF CARD DAI
 */
static struct snd_soc_dai_link spdifrx_dai_link = {
	.name		= "SPDIF Receiver",
	.stream_name	= "SPDIF PCM Capture",
	.cpu_dai_name	= "c005a000.spdifrx",	/* spdif_driver name */
	.platform_name  = "nxp-pcm",			/* nxp_snd_pcm_driver name */
	.codec_dai_name = "dir-hifi",
	.codec_name	= "0.spdif-dir",
	.symmetric_rates = 1,
};

static struct snd_soc_card spdifrx_card = {
	.name		= "SPDIF-Receiver",		/* proc/asound/cards */
	.dai_link	= &spdifrx_dai_link,
	.num_links	= 1,
};

static int spdifrx_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &spdifrx_card;
	struct snd_soc_dai *codec_dai = NULL;
	struct snd_soc_dai_driver *driver = NULL;
	unsigned int sample_rate = 0, format = 0;
	int ret;
	const char *format_name;

	/* register card */
	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);
		return ret;
	}

    of_property_read_u32(pdev->dev.of_node, "sample_rate", &sample_rate);
    format_name = of_get_property(pdev->dev.of_node, "format", NULL);
    if (format_name != NULL) {
        if (strcmp(format_name,"S16") == 0)
            format = SNDRV_PCM_FMTBIT_S16_LE;
        else if(strcmp(format_name,"S24") == 0)
            format = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE;
    }

	if (card->rtd) {
		codec_dai = card->rtd->codec_dai;
		if (codec_dai)
			driver = codec_dai->driver;
	}

	/* Reset spdif sample rates and format */
	if (sample_rate) {
		sample_rate = snd_pcm_rate_to_rate_bit(sample_rate);
		if (SNDRV_PCM_RATE_KNOT != sample_rate)
			driver->capture.rates = sample_rate;
		else
			printk("%s, invalid sample rates=%d\n", __func__, sample_rate);
	}

	if(format)
		driver->capture.formats = format;

	pr_debug("spdif-rx-dai: register card %s -> %s\n",
		card->dai_link->codec_dai_name, card->dai_link->cpu_dai_name);
	return ret;
}

static int spdifrx_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nxp_spdifrx_match[] = {
	{ .compatible = "nexell,spdif-receiver" },
	{},
};
MODULE_DEVICE_TABLE(of, nxp_spdifrx_match);
#else
#define nxp_spdifrx_match NULL
#endif

static struct platform_driver spdifrx_driver = {
	.driver		= {
		.name	= "spdif-receiver",
		.owner	= THIS_MODULE,
		.pm 	= &snd_soc_pm_ops,	/* for suspend */
        .of_match_table = of_match_ptr(nxp_spdifrx_match),
	},
	.probe		= spdifrx_probe,
	.remove		= spdifrx_remove,
};
module_platform_driver(spdifrx_driver);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Sound SPDIF receiver driver for the SLSI");
MODULE_LICENSE("GPL");

