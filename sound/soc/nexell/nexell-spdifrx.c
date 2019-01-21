/*
 * Copyright (C) 2019  Nexell Co., Ltd.
 * Author: Hyunseok, Jung<hsjung@nexell.co.kr>
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#ifdef CONFIG_RESET_CONTROLLER
#include <linux/reset.h>
#endif

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "nexell-spdif.h"

#define	DEF_SAMPLE_RATE			48000
#define	DEF_SAMPLE_BIT			16	// 8, 16, 24 (PCM)

#define	SPDIF_BASEADDR			0xC005A000
#define	SPDIF_BUS_WIDTH			4	// Byte
#define	SPDIF_MAX_BURST			4	// Byte

#define	SPDIFRX_PRESETn			(42)

/*
 * SPDIF register
 */
struct spdif_register {
	unsigned int ctrl;
	unsigned int INTC;
};

#define	SPDIF_CTRL_OFFSET		(0x00)
#define	SPDIF_INTC_OFFSET		(0x04)
#define	SPDIF_DAT_OFFSET		(0x68)	// DMA: 0x68

#define	CTRL_CPUHEADER_POS		16	//	[16]
#define	CTRL_DECRATE_POS		12	//	[12:15]
#define	CTRL_DATA_ORDER_POS		11	//	[11]
#define	CTRL_LOCK_POS			10	//	[10]
#define	CTRL_CLRFIFO_POS		9	//	[9]
#define	CTRL_PHASE_DET_POS		8	//	[8]
#define	CTRL_VALID_SAMPLE_POS		4	//	[4:7]
#define	CTRL_CAP_USER_DAT_POS		3	//	[3]
#define	CTRL_DMA_DATA_POS		2	//	[2]
#define	CTRL_DMA_SWAP_POS		1	//	[1]
#define	CTRL_DECODE_ENB_POS		0	//	[0]

#define	CTRL_ORDER_LSB_MSB		0
#define	CTRL_ORDER_MSB_LSB		1

#define	CTRL_VALID_SAMPLE_8		0
#define	CTRL_VALID_SAMPLE_7		1
#define	CTRL_VALID_SAMPLE_6		2
#define	CTRL_VALID_SAMPLE_5		3
#define	CTRL_VALID_SAMPLE_4		4
#define	CTRL_VALID_SAMPLE_3		5
#define	CTRL_VALID_SAMPLE_2		6
#define	CTRL_VALID_SAMPLE_1		7
#define	CTRL_VALID_SAMPLE_0		8

#define	CTRL_DMA_SWAP_CHA_CHB		0
#define	CTRL_DMA_SWAP_CHB_CHA		1

/*
 * parameters
 */
struct nx_spdif_snd_param {
	int sample_rate;
	int status;
	spinlock_t lock;
	/* DMA channel */
	struct nx_pcm_dma_param dma;
	/* Register */
	void __iomem *base_addr;
	struct spdif_register spdif;
};

static void spdif_reset(struct device *dev,
			struct nx_spdif_snd_param *par)
{
	void __iomem *base = par->base_addr;

#ifdef CONFIG_RESET_CONTROLLER
	struct reset_control *rst;

	rst = reset_control_get(dev, "spdifrx-reset");

	if (!IS_ERR(rst)) {
		if (reset_control_status(rst))
			reset_control_reset(rst);
		reset_control_put(rst);
	}
#endif
}

static int  spdif_start(struct nx_spdif_snd_param *par, int stream)
{
	struct spdif_register *spdif = &par->spdif;
	void __iomem *ctrl = par->base_addr + SPDIF_CTRL_OFFSET;
	void __iomem *pend = par->base_addr + SPDIF_INTC_OFFSET;

	/* clear fifo, pend */
	writel(readl(ctrl) | (1<<CTRL_CLRFIFO_POS), ctrl);
	writel(spdif->ctrl, ctrl);
	writel(readl(pend) | 0xF0, pend);

	/* run */
	spdif->ctrl |= (1<<CTRL_DECODE_ENB_POS);
	spdif->ctrl |= (1<<CTRL_PHASE_DET_POS);
	writel(spdif->ctrl, ctrl);

	par->status |= SNDDEV_STATUS_CAPT;
	return 0;
}

static void spdif_stop(struct nx_spdif_snd_param *par, int stream)
{
	struct spdif_register *spdif = &par->spdif;
	void __iomem *ctrl = par->base_addr + SPDIF_CTRL_OFFSET;

	spdif->ctrl &= ~((1<<CTRL_DECODE_ENB_POS) | (1<<CTRL_PHASE_DET_POS));
	writel(spdif->ctrl, ctrl);

	par->status &= ~SNDDEV_STATUS_CAPT;
}

static int nx_spdif_check_param(struct nx_spdif_snd_param *par)
{
	struct spdif_register *spdif = &par->spdif;
	struct nx_pcm_dma_param *dmap = &par->dma;
	int ORDER = CTRL_ORDER_LSB_MSB;
	int VALID_SAMPLE = CTRL_VALID_SAMPLE_8;
	int DATA_ONLY = 1;
	int DAM_SWAP = 1;	// CHA_CHB = 0, CHB_CHA = 1

	dmap->real_clock = par->sample_rate;

	spdif->ctrl =   (ORDER << CTRL_DATA_ORDER_POS) |	// [11]
		(0 << CTRL_LOCK_POS) |				// [10]
		(0 << CTRL_CLRFIFO_POS) |			// [9]
		(0 << CTRL_PHASE_DET_POS) |			// [8]	-> Enable
		(VALID_SAMPLE << CTRL_VALID_SAMPLE_POS) |	// [4:7]
		(1 << CTRL_CAP_USER_DAT_POS) |			// 3
		(DATA_ONLY << CTRL_DMA_DATA_POS) |		// 2
		(DAM_SWAP << CTRL_DMA_SWAP_POS) |		// 1
		(0	<< CTRL_DECODE_ENB_POS);		// 0

	return 0;
}

static int nx_spdif_set_plat_param(struct nx_spdif_snd_param *par, void *data)
{
	struct platform_device *pdev = data;
	struct nx_pcm_dma_param *dma = &par->dma;
	unsigned int phy_base = SPDIF_BASEADDR;

	of_property_read_u32(pdev->dev.of_node, "sample_rate", &par->sample_rate);
	if (!par->sample_rate)
		par->sample_rate = DEF_SAMPLE_RATE;

	par->base_addr = of_iomap(pdev->dev.of_node, 0);
	spin_lock_init(&par->lock);

	dma->active = true;
	dma->dev = &pdev->dev;

	dma->peri_addr = phy_base + SPDIF_DAT_OFFSET;	/* SPDIF DAT */
	dma->bus_width_byte = SPDIF_BUS_WIDTH;
	dma->max_burst_byte = SPDIF_MAX_BURST;
	pr_debug("spdif-rx: %s, %s dma, addr 0x%p, bus %dbyte, burst %dbyte\n",
		STREAM_STR(1), dma->dma_ch_name, (void *)dma->peri_addr,
		dma->bus_width_byte, dma->max_burst_byte);

	return nx_spdif_check_param(par);
}

static int nx_spdif_setup(struct snd_soc_dai *dai)
{
	struct nx_spdif_snd_param *par = snd_soc_dai_get_drvdata(dai);
	struct spdif_register *spdif = &par->spdif;
	void __iomem *ctrl = par->base_addr + SPDIF_CTRL_OFFSET;
	struct clk *clk = NULL;
	unsigned long pclk_hz = 0;

	if (SNDDEV_STATUS_SETUP & par->status)
		return 0;

	clk = clk_get(NULL, "pclk");
	pclk_hz = clk_get_rate(clk);
	clk_put(clk);

	spdif_reset(dai->dev, par);

	printk(KERN_INFO "spdif-rx: PCLK=%ldhz \n", pclk_hz);
	spdif->ctrl &= ~(0xf << CTRL_DECRATE_POS);
	if(pclk_hz > 180000000)
		spdif->ctrl |= 8 << CTRL_DECRATE_POS;
	else if(pclk_hz > 100000000)
		spdif->ctrl |= 6 << CTRL_DECRATE_POS;
	else
		spdif->ctrl |= 3 << CTRL_DECRATE_POS;

	writel(spdif->ctrl, ctrl);
	par->status |= SNDDEV_STATUS_SETUP;

	return 0;
}

static void nx_spdif_release(struct snd_soc_dai *dai)
{
	struct nx_spdif_snd_param *par = snd_soc_dai_get_drvdata(dai);
	struct spdif_register *spdif = &par->spdif;
	void __iomem *ctrl = par->base_addr + SPDIF_CTRL_OFFSET;

	spdif->ctrl &= ~(1 << CTRL_DECODE_ENB_POS);
	writel(spdif->ctrl, ctrl);

	par->status = SNDDEV_STATUS_CLEAR;
}

/*
 * snd_soc_dai_ops
 */
static int  nx_spdif_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct nx_spdif_snd_param *par = snd_soc_dai_get_drvdata(dai);
	struct nx_pcm_dma_param *dmap = &par->dma;

	snd_soc_dai_set_dma_data(dai, substream, dmap);
	return 0;
}

static void nx_spdif_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct nx_spdif_snd_param *par = snd_soc_dai_get_drvdata(dai);
	spdif_stop(par, substream->stream);
}

static int nx_spdif_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct nx_spdif_snd_param *par = snd_soc_dai_get_drvdata(dai);
	int stream = substream->stream;
	pr_debug("%s: %s cmd=%d\n", __func__, STREAM_STR(stream), cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
		spdif_start(par, stream);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		spdif_stop(par, stream);
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int nx_spdif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct nx_spdif_snd_param *par = snd_soc_dai_get_drvdata(dai);
	struct spdif_register *spdif = &par->spdif;
	unsigned int format = params_format(params);

	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		pr_debug("spdiftx: change sample bits S16\n");
		spdif->ctrl &= ~((0xF<<CTRL_VALID_SAMPLE_POS) | (1<<CTRL_DMA_DATA_POS));
		spdif->ctrl |= (CTRL_VALID_SAMPLE_8<<CTRL_VALID_SAMPLE_POS) | (1<<CTRL_DMA_DATA_POS);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		pr_debug("spdiftx: change sample bits S24\n");
		spdif->ctrl &= ~((0xF<<CTRL_VALID_SAMPLE_POS) | (1<<CTRL_DMA_DATA_POS));
		spdif->ctrl |= (CTRL_VALID_SAMPLE_0<<CTRL_VALID_SAMPLE_POS) | (0<<CTRL_DMA_DATA_POS);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct snd_soc_dai_ops nx_spdif_ops = {
	.startup	= nx_spdif_startup,
	.shutdown	= nx_spdif_shutdown,
	.trigger	= nx_spdif_trigger,
	.hw_params	= nx_spdif_hw_params,
};

/*
 * snd_soc_dai_driver
 */
static int nx_spdif_dai_suspend(struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s\n", __func__);

	return 0;
}

static int nx_spdif_dai_resume(struct snd_soc_dai *dai)
{
	struct nx_spdif_snd_param *par = snd_soc_dai_get_drvdata(dai);
	struct spdif_register *spdif = &par->spdif;
	void __iomem *ctrl = par->base_addr + SPDIF_CTRL_OFFSET;

	dev_dbg(dai->dev, "%s\n", __func__);

	spdif_reset(dai->dev, par);
	writel(spdif->ctrl, ctrl);
	return 0;
}

static int nx_spdif_dai_probe(struct snd_soc_dai *dai)
{
	return nx_spdif_setup(dai);
}

static int nx_spdif_dai_remove(struct snd_soc_dai *dai)
{
	nx_spdif_release(dai);
	return 0;
}

static struct snd_soc_dai_driver spdif_dai_driver = {
	.capture	= {
		.channels_min	= 2,
		.channels_max	= 2,
		.formats	= SND_SOC_SPDIF_RX_FORMATS,
		.rates		= SND_SOC_SPDIF_RATES,
		.rate_min	= 0,
		.rate_max	= 1562500,
		},
	.probe		= nx_spdif_dai_probe,
	.remove		= nx_spdif_dai_remove,
	.suspend	= nx_spdif_dai_suspend,
	.resume		= nx_spdif_dai_resume,
	.ops		= &nx_spdif_ops,
};

static const struct snd_soc_component_driver nx_spdif_component = {
	.name       = "nx-spdif-rxc",
};

static int nx_spdif_probe(struct platform_device *pdev)
{
	struct nx_spdif_snd_param *par;
	int ret = 0;

	/*  allocate i2c_port data */
	par = kzalloc(sizeof(struct nx_spdif_snd_param), GFP_KERNEL);
	if (! par) {
		return -ENOMEM;
	}

	ret = nx_spdif_set_plat_param(par, pdev);
	if (ret)
		goto err_out;

	ret = snd_soc_register_component(&pdev->dev, &nx_spdif_component,
				 &spdif_dai_driver, 1);
	if (ret) {
		dev_err(&pdev->dev, "fail, %s snd_soc_register_component ...\n",
			pdev->name);
		goto err_out;
	}

	ret = devm_snd_soc_register_platform(&pdev->dev, &nx_pcm_platform);
	if (ret) {
		dev_err(&pdev->dev, "fail, snd_soc_register_platform...\n");
		goto err_out;
	}

	dev_set_drvdata(&pdev->dev, par);
	return ret;

err_out:
	kfree(NULL);
	return ret;
}

static int nx_spdif_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	kfree(NULL);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nx_spdif_match[] = {
	{ .compatible = "nexell,nexell-spdif-rx" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_spdif_match);
#else
#define nx_spdif_match NULL
#endif

static struct platform_driver spdif_driver = {
	.probe  = nx_spdif_probe,
	.remove = nx_spdif_remove,
	.driver = {
		.name	= "nexell-spdif-rx",
		.owner	= THIS_MODULE,
        .of_match_table = of_match_ptr(nx_spdif_match),
	},
};

static int __init nx_spdif_init(void)
{
	return platform_driver_register(&spdif_driver);
}

static void __exit nx_spdif_exit(void)
{
	platform_driver_unregister(&spdif_driver);
}

module_init(nx_spdif_init);
module_exit(nx_spdif_exit);

MODULE_AUTHOR("Hyunseok Jung <hsjung@nexell.co.kr>");
MODULE_DESCRIPTION("Sound S/PDIF rx driver for Nexell sound");
MODULE_LICENSE("GPL");
