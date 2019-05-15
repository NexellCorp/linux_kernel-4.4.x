/*
 * rt5633.c  --  RT5633 ALSA SoC audio codec driver
 *
 * Copyright 2011 Realtek Semiconductor Corp.
 * Author: Johnny Hsu <johnnyhsu@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "rt5633.h"

struct rt5633_priv {
	struct snd_soc_codec *codec;
	struct regmap *regmap;
	int master;
	int sysclk;
};

static unsigned int BLCK_FREQ=32;	/* 32fs,bitclk is 32 fs */
module_param(BLCK_FREQ, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(BLCK_FREQ, "relationship between bclk and fs");

static const struct reg_default rt5633_reg[] = {
	{ RT5633_RESET,			0x0001 },
	{ RT5633_SPK_OUT_VOL,		0xe000 },
	{ RT5633_HP_OUT_VOL,		0x8080 },
	{ RT5633_AUXOUT_VOL,		0xc080 },
	{ RT5633_LINE_IN_1_VOL,		0x0808 },
	{ RT5633_LINE_IN_2_VOL,		0x0808 },
	{ RT5633_MIC_CTRL_1,		0x0808 },
	{ RT5633_REC_MIXER_CTRL,	0x7f7f },
	{ RT5633_HPMIXER_CTRL,		0x3f3f },
	{ RT5633_AUXMIXER_CTRL,		0x3f3f },
	{ RT5633_SPKMIXER_CTRL,		0x00ff },
	{ RT5633_SPK_AMP_CTRL,		0x8000 },
	{ RT5633_SDP_CTRL,		0x8000 },
	{ RT5633_STEREO_AD_DA_CLK_CTRL,	0x2000 },
	{ RT5633_GEN_PUR_CTRL_1,	0x0c00 },
	{ RT5633_DIG_BEEP_IRQ_CTRL,	0x0280 },
	{ RT5633_GEN_PUR_CTRL_2,	0x40c0 },
	{ RT5633_DEPOP_CTRL_2,		0x3000 },
	{ RT5633_ZC_SM_CTRL_1,		0x0489 },
	{ RT5633_ZC_SM_CTRL_2,		0x5ffe },
	{ RT5633_ALC_CTRL_1,		0x0206 },
	{ RT5633_ALC_CTRL_3,		0x2000 },
	{ RT5633_PSEUDO_SPATL_CTRL,	0x0553 },
	{ RT5633_EQ_CTRL_1,		0x1000 },
	{ RT5633_VERSION,		0x0002 },
	{ RT5633_VENDOR_ID1,		0x10ec },
	{ RT5633_VENDOR_ID2,		0x6179 },
};

static const struct reg_sequence rt5633_init_list[] = {
	{RT5633_SPK_OUT_VOL		, 0x8000}, //speaker output volume is 0db by default
	{RT5633_SPK_HP_MIXER_CTRL	, 0x0020}, //HP from HP_VOL
	{RT5633_HP_OUT_VOL 		, 0xc8c8}, //HP output volume is -12 db by default
	{RT5633_AUXOUT_VOL		, 0x0010}, //Auxout volume is 0db by default
	{RT5633_REC_MIXER_CTRL		, 0x7d7d}, //ADC Record Mixer Control
	{RT5633_MIC_CTRL_2		, 0x5500}, //boost 40db
	{RT5633_HPMIXER_CTRL		, 0x3e3e}, //"HP Mixer Control"
	//{RT5633_AUXMIXER_CTRL		, 0x3e3e}, //"AUX Mixer Control"
	{RT5633_SPKMIXER_CTRL		, 0x08fc}, //"SPK Mixer Control"
	{RT5633_ZC_SM_CTRL_1		, 0x0001}, //Disable Zero Cross
	{RT5633_ZC_SM_CTRL_2		, 0x3000}, //Disable Zero cross
	{RT5633_MIC_CTRL_1       	, 0x8808}, //set mic1 to differnetial mode
	{RT5633_DEPOP_CTRL_2		, 0xb000},
	{RT5633_PRI_REG_ADD		, 0x0056},
	{RT5633_PRI_REG_DATA		, 0x303f},
	//JD setting
	//{RT5633_ZC_SM_CTRL_1		, 0x04b0},
	//{RT5633_ZC_SM_CTRL_2		, 0x3000},
	//{RT5633_JACK_DET_CTRL           , 0x6e00},
};

static int rt5633_index_sync(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rt5633_init_list); i++)
		if (RT5633_PRI_REG_ADD == rt5633_init_list[i].reg ||
			RT5633_PRI_REG_DATA == rt5633_init_list[i].reg)
			snd_soc_write(codec, rt5633_init_list[i].reg,
					rt5633_init_list[i].def);

	return 0;
}

static int rt5633_reset(struct snd_soc_codec *codec)
{
	return snd_soc_write(codec, RT5633_RESET, 0);
}

static bool rt5633_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RT5633_RESET:
	case RT5633_INT_ST_STICKY_CTRL:
	case RT5633_ALC_CTRL_3:
	case RT5633_PRI_REG_DATA:
	case RT5633_EQ_CTRL_1:
	case RT5633_EQ_CTRL_2:
	case RT5633_VERSION:
	case RT5633_VENDOR_ID1:
	case RT5633_VENDOR_ID2:
		return true;

	default:
		return false;
	}
}

static bool rt5633_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RT5633_RESET:
	case RT5633_SPK_OUT_VOL:
	case RT5633_SPK_HP_MIXER_CTRL:
	case RT5633_HP_OUT_VOL:
	case RT5633_AUXOUT_VOL:
	case RT5633_LINE_IN_1_VOL:
	case RT5633_LINE_IN_2_VOL:
	case RT5633_DAC_CTRL:
	case RT5633_DAC_DIG_VOL:
	case RT5633_MIC_CTRL_1:
	case RT5633_ADC_CTRL:
	case RT5633_REC_MIXER_CTRL:
	case RT5633_ADC_DIG_VOL:
	case RT5633_HPMIXER_CTRL:
	case RT5633_AUXMIXER_CTRL:
	case RT5633_SPKMIXER_CTRL:
	case RT5633_SPK_AMP_CTRL:
	case RT5633_MIC_CTRL_2:
	case RT5633_SDP_CTRL:
	case RT5633_STEREO_AD_DA_CLK_CTRL:
	case RT5633_PWR_MANAG_ADD1:
	case RT5633_PWR_MANAG_ADD2:
	case RT5633_PWR_MANAG_ADD3:
	case RT5633_PWR_MANAG_ADD4:
	case RT5633_GEN_PUR_CTRL_1:
	case RT5633_GBL_CLK_CTRL:
	case RT5633_PLL_CTRL:
	case RT5633_DIG_BEEP_IRQ_CTRL:
	case RT5633_INT_ST_STICKY_CTRL:
	case RT5633_GPIO_CTRL_1:
	case RT5633_GPIO_CTRL_2:
	case RT5633_GEN_PUR_CTRL_2:
	case RT5633_DEPOP_CTRL_1:
	case RT5633_DEPOP_CTRL_2:
	case RT5633_JACK_DET_CTRL:
	case RT5633_ZC_SM_CTRL_1:
	case RT5633_ZC_SM_CTRL_2:
	case RT5633_ALC_CTRL_1:
	case RT5633_ALC_CTRL_2:
	case RT5633_ALC_CTRL_3:
	case RT5633_PSEUDO_SPATL_CTRL:
	case RT5633_PRI_REG_ADD:
	case RT5633_PRI_REG_DATA:
	case RT5633_EQ_CTRL_1:
	case RT5633_EQ_CTRL_2:
	case RT5633_VERSION:
	case RT5633_VENDOR_ID1:
	case RT5633_VENDOR_ID2:
		return 1;

	default:
		return 0;
	}
}

static const DECLARE_TLV_DB_SCALE(out_vol_tlv, -4650, 150, 0);
static const DECLARE_TLV_DB_SCALE(in_vol_tlv, -3450, 150, 0);
/* {0, +20, +24, +30, +35, +40, +44, +50, +52} dB */
static unsigned int mic_bst_tlv[] = {
	TLV_DB_RANGE_HEAD(7),
	0, 0, TLV_DB_SCALE_ITEM(0, 0, 0),
	1, 1, TLV_DB_SCALE_ITEM(2000, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(2400, 0, 0),
	3, 5, TLV_DB_SCALE_ITEM(3000, 500, 0),
	6, 6, TLV_DB_SCALE_ITEM(4400, 0, 0),
	7, 7, TLV_DB_SCALE_ITEM(5000, 0, 0),
	8, 8, TLV_DB_SCALE_ITEM(5200, 0, 0),
};

static const char *rt5633_auxout_mode[] = {"Differential", "Single ended"};

static const SOC_ENUM_SINGLE_DECL(rt5633_auxout_mode_enum,
	RT5633_AUXOUT_VOL, RT5633_AUXOUT_MODE_SEL_SFT, rt5633_auxout_mode);

static const char *rt5633_input_mode[] = {"Single ended", "Differential"};

static const SOC_ENUM_SINGLE_DECL(rt5633_mic1_mode_enum,
	RT5633_MIC_CTRL_1, RT5633_MIC_1_MODE_SEL_SFT, rt5633_input_mode);

static const SOC_ENUM_SINGLE_DECL(rt5633_mic2_mode_enum,
	RT5633_MIC_CTRL_1, RT5633_MIC_2_MODE_SEL_SFT, rt5633_input_mode);

static const SOC_ENUM_SINGLE_DECL(rt5633_spkout_mode_enum,
	RT5633_SPK_OUT_VOL, RT5633_SPK_CLSAB_M_SFT, rt5633_input_mode);

static const char *rt5633_spon_clsd_ctl_sel[] = {"RN", "RP", "LN", "VMID"};

static const SOC_ENUM_SINGLE_DECL(rt5633_spon_clsd_ctl_enum,
	RT5633_SPK_OUT_VOL, RT5633_SPON_CTL_SFT, rt5633_spon_clsd_ctl_sel);

static const char *rt5633_spon_clsab_ctl_sel[] = {"VMID", "RP", "LN", "RN"};

static const SOC_ENUM_SINGLE_DECL(rt5633_spon_clsab_ctl_enum,
	RT5633_SPK_OUT_VOL, RT5633_SPON_CTL_SFT, rt5633_spon_clsab_ctl_sel);

static const struct snd_kcontrol_new rt5633_snd_controls[] = {
	SOC_ENUM("MIC1 Mode Control", rt5633_mic1_mode_enum),
	SOC_SINGLE_TLV("MIC1 Boost", RT5633_MIC_CTRL_2,
		RT5633_MIC1_BOOST_CTRL_SFT, 8, 0, mic_bst_tlv),
	SOC_ENUM("MIC2 Mode Control", rt5633_mic2_mode_enum),
	SOC_SINGLE_TLV("MIC2 Boost", RT5633_MIC_CTRL_2,
		RT5633_MIC2_BOOST_CTRL_SFT, 8, 0, mic_bst_tlv),
	SOC_ENUM("Classab Mode Control", rt5633_spkout_mode_enum),
	SOC_ENUM("SPO_N Class D Output Control", rt5633_spon_clsd_ctl_enum),
	SOC_ENUM("SPO_N Class AB Output Control", rt5633_spon_clsab_ctl_enum),
	SOC_ENUM("AUXOUT Control", rt5633_auxout_mode_enum),
	SOC_DOUBLE_TLV("Line1 Capture Volume", RT5633_LINE_IN_1_VOL,
		RT5633_L_VOL_SFT, RT5633_R_VOL_SFT, 31, 1, in_vol_tlv),
	SOC_DOUBLE_TLV("Line2 Capture Volume", RT5633_LINE_IN_2_VOL,
		RT5633_L_VOL_SFT, RT5633_R_VOL_SFT, 31, 1, in_vol_tlv),
	SOC_SINGLE_TLV("MIC1 Playback Volume", RT5633_MIC_CTRL_1,
		RT5633_L_VOL_SFT, 31, 1, in_vol_tlv),
	SOC_SINGLE_TLV("MIC2 Playback Volume", RT5633_MIC_CTRL_1,
		RT5633_R_VOL_SFT, 31, 1, in_vol_tlv),
	SOC_SINGLE("AXOL Playback Switch", RT5633_AUXOUT_VOL,
				RT5633_L_MUTE_SFT, 1, 1),
	SOC_SINGLE("AXOR Playback Switch", RT5633_AUXOUT_VOL,
				RT5633_R_MUTE_SFT, 1, 1),
	SOC_DOUBLE("AUX Playback Volume", RT5633_AUXOUT_VOL,
		RT5633_L_VOL_SFT, RT5633_R_VOL_SFT, 31, 1),
	SOC_SINGLE("SPK Playback Switch", RT5633_SPK_OUT_VOL,
				RT5633_L_MUTE_SFT, 1, 1),
	SOC_DOUBLE_TLV("SPK Playback Volume", RT5633_SPK_OUT_VOL,
		RT5633_SPKL_VOL_SFT, RT5633_SPKR_VOL_SFT, 31, 1, out_vol_tlv),
	SOC_SINGLE("HPL Playback Switch", RT5633_HP_OUT_VOL,
				RT5633_L_MUTE_SFT, 1, 1),
	SOC_SINGLE("HPR Playback Switch", RT5633_HP_OUT_VOL,
				RT5633_R_MUTE_SFT, 1, 1),
	SOC_DOUBLE_TLV("HP Playback Volume", RT5633_HP_OUT_VOL,
		RT5633_L_VOL_SFT, RT5633_R_VOL_SFT, 31, 1, out_vol_tlv),
};

static const struct snd_kcontrol_new rt5633_recmixl_mixer_controls[] = {
	SOC_DAPM_SINGLE("HPMIXL Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_L_HPM_SFT, 1, 1),
	SOC_DAPM_SINGLE("AUXMIXL Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_L_AUXM_SFT, 1, 1),
	SOC_DAPM_SINGLE("SPKMIX Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_L_SPKM_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE1L Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_L_LINE1_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE2L Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_L_LINE2_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC1 Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_L_MIC1_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC2 Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_L_MIC2_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5633_recmixr_mixer_controls[] = {
	SOC_DAPM_SINGLE("HPMIXR Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_R_HPM_SFT, 1, 1),
	SOC_DAPM_SINGLE("AUXMIXR Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_R_AUXM_SFT, 1, 1),
	SOC_DAPM_SINGLE("SPKMIX Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_R_SPKM_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE1R Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_R_LINE1_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE2R Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_R_LINE2_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC1 Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_R_MIC1_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC2 Capture Switch", RT5633_REC_MIXER_CTRL,
				RT5633_M_RM_R_MIC2_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5633_hp_mixl_mixer_controls[] = {
	SOC_DAPM_SINGLE("RECMIXL Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_L_RM_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC1 Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_L_MIC1_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC2 Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_L_MIC2_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE1 Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_L_LINE1_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE2 Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_L_LINE2_SFT, 1, 1),
	SOC_DAPM_SINGLE("DAC Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_L_DAC_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5633_hp_mixr_mixer_controls[] = {
	SOC_DAPM_SINGLE("RECMIXR Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_R_RM_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC1 Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_R_MIC1_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC2 Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_R_MIC2_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE1 Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_R_LINE1_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE2 Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_R_LINE2_SFT, 1, 1),
	SOC_DAPM_SINGLE("DAC Playback Switch", RT5633_HPMIXER_CTRL,
				RT5633_M_HPM_R_DAC_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5633_auxmixl_mixer_controls[] = {
	SOC_DAPM_SINGLE("RECMIXL Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_L_RM_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC1 Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_L_MIC1_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC2 Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_L_MIC2_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE1 Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_L_LINE1_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE2 Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_L_LINE2_SFT, 1, 1),
	SOC_DAPM_SINGLE("DAC Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_L_DAC_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5633_auxmixr_mixer_controls[] = {
	SOC_DAPM_SINGLE("RECMIXR Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_R_RM_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC1 Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_R_MIC1_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC2 Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_R_MIC2_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE1 Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_R_LINE1_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE2 Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_R_LINE2_SFT, 1, 1),
	SOC_DAPM_SINGLE("DAC Playback Switch", RT5633_AUXMIXER_CTRL,
				RT5633_M_AM_R_DAC_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5633_spkmixr_mixer_controls[]  = {
	SOC_DAPM_SINGLE("MIC1 Playback Switch", RT5633_SPKMIXER_CTRL,
				RT5633_M_SM_R_MIC1_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIC2 Playback Switch", RT5633_SPKMIXER_CTRL,
				RT5633_M_SM_R_MIC2_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE1L Playback Switch", RT5633_SPKMIXER_CTRL,
				RT5633_M_SM_R_L1_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE1R Playback Switch", RT5633_SPKMIXER_CTRL,
				RT5633_M_SM_R_L1_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE2L Playback Switch", RT5633_SPKMIXER_CTRL,
				RT5633_M_SM_R_L2_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("LINE2R Playback Switch", RT5633_SPKMIXER_CTRL,
				RT5633_M_SM_R_L2_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACL Playback Switch", RT5633_SPKMIXER_CTRL,
				RT5633_M_SM_R_DACL_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACR Playback Switch", RT5633_SPKMIXER_CTRL,
				RT5633_M_SM_R_DACR_SFT, 1, 1),
};

/* DAC function select MUX */
static const char *rt5633_spo_src_sel[] = {"VMID", "HPMIX", "SPKMIX", "AUXMIX"};

static const struct soc_enum rt5633_spo_src_enum =
	SOC_ENUM_SINGLE(RT5633_SPKMIXER_CTRL, RT5633_SPK_VOL_S_SFT,
		ARRAY_SIZE(rt5633_spo_src_sel), rt5633_spo_src_sel);

static const struct snd_kcontrol_new rt5633_spo_src_mux =
	SOC_DAPM_ENUM("SPO SRC Mux", rt5633_spo_src_enum);

static int rt5633_check_sclk(struct snd_soc_dapm_widget *source,
			 struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	unsigned int val;

	val = snd_soc_read(codec, RT5633_GBL_CLK_CTRL);
	val &= RT5633_SCLK_SRC_MASK;

	return (val == RT5633_SCLK_SRC_PLL);
}

static int rt5633_check_classd(struct snd_soc_dapm_widget *source,
			 struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	unsigned int val;

	val = snd_soc_read(codec, RT5633_SPK_AMP_CTRL);
	val &= RT5633_SPK_AMP_MODE_MASK;

	return (val == RT5633_SPK_AMP_MODE_D);
}

static int rt5633_check_classab(struct snd_soc_dapm_widget *source,
			 struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(source->dapm);
	unsigned int val;

	val = snd_soc_read(codec, RT5633_SPK_AMP_CTRL);
	val &= RT5633_SPK_AMP_MODE_MASK;

	return (val == RT5633_SPK_AMP_MODE_AB);
}

/* HP power on depop */
static void hp_depop_mode2(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD3,
		RT5633_P_MAIN_BIAS | RT5633_P_VREF,
		RT5633_P_MAIN_BIAS | RT5633_P_VREF);
	snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD4,
		RT5633_P_HP_L_VOL | RT5633_P_HP_R_VOL,
		RT5633_P_HP_L_VOL | RT5633_P_HP_R_VOL);
	snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD3,
		RT5633_P_HP_AMP, RT5633_P_HP_AMP);
	snd_soc_update_bits(codec, RT5633_DEPOP_CTRL_1,
		RT5633_PW_SOFT_GEN | RT5633_EN_DEPOP_2,
		RT5633_PW_SOFT_GEN | RT5633_EN_DEPOP_2);
	msleep(300);
	snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD3,
		RT5633_P_HP_DIS_DEPOP, RT5633_P_HP_DIS_DEPOP);
	snd_soc_update_bits(codec, RT5633_DEPOP_CTRL_1, RT5633_EN_DEPOP_2, 0);
}

/* HP mute/unmute depop */
static void hp_mute_unmute_depop(struct snd_soc_codec *codec, int Mute)
{
	if (Mute) {
		snd_soc_update_bits(codec, RT5633_DEPOP_CTRL_1,
			RT5633_PW_SOFT_GEN | RT5633_EN_SOFT_FOR_S_M_DEPOP |
			RT5633_EN_HP_R_M_UM_DEPOP | RT5633_EN_HP_L_M_UM_DEPOP,
			RT5633_PW_SOFT_GEN | RT5633_EN_SOFT_FOR_S_M_DEPOP |
			RT5633_EN_HP_R_M_UM_DEPOP | RT5633_EN_HP_L_M_UM_DEPOP);
		snd_soc_update_bits(codec, RT5633_HP_OUT_VOL,
			RT5633_L_MUTE | RT5633_R_MUTE,
			RT5633_L_MUTE | RT5633_R_MUTE);
		mdelay(80);
		snd_soc_update_bits(codec, RT5633_DEPOP_CTRL_1,
			RT5633_PW_SOFT_GEN | RT5633_EN_SOFT_FOR_S_M_DEPOP |
			RT5633_EN_HP_R_M_UM_DEPOP | RT5633_EN_HP_L_M_UM_DEPOP,
			0);
	} else {
		snd_soc_update_bits(codec, RT5633_DEPOP_CTRL_1,
			RT5633_PW_SOFT_GEN | RT5633_EN_SOFT_FOR_S_M_DEPOP |
			RT5633_EN_HP_R_M_UM_DEPOP | RT5633_EN_HP_L_M_UM_DEPOP,
			RT5633_PW_SOFT_GEN | RT5633_EN_SOFT_FOR_S_M_DEPOP |
			RT5633_EN_HP_R_M_UM_DEPOP | RT5633_EN_HP_L_M_UM_DEPOP);
		snd_soc_update_bits(codec, RT5633_HP_OUT_VOL,
			RT5633_L_MUTE | RT5633_R_MUTE, 0);
		mdelay(80);
		snd_soc_update_bits(codec, RT5633_DEPOP_CTRL_1,
			RT5633_PW_SOFT_GEN | RT5633_EN_SOFT_FOR_S_M_DEPOP |
			RT5633_EN_HP_R_M_UM_DEPOP | RT5633_EN_HP_L_M_UM_DEPOP,
			0);
	}
}

static int open_hp_end_widgets(struct snd_soc_codec *codec)
{
	hp_mute_unmute_depop(codec, 0);

	return 0;
}

static int close_hp_end_widgets(struct snd_soc_codec *codec)
{
	hp_mute_unmute_depop(codec, 1);
	snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD3,
		RT5633_P_HP_AMP | RT5633_P_HP_BUF |
		RT5633_P_HP_DIS_DEPOP | RT5633_P_HP_AMP_DRI, 0);
	return 0;
}

static int rt5633_hp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		close_hp_end_widgets(codec);
		break;

	case SND_SOC_DAPM_POST_PMU:
		hp_depop_mode2(codec);
		open_hp_end_widgets(codec);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dapm_widget rt5633_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("PLL", RT5633_PWR_MANAG_ADD2,
			RT5633_P_PLL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("I2S", RT5633_PWR_MANAG_ADD1,
			RT5633_P_MAIN_I2S_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC Ref", RT5633_PWR_MANAG_ADD1,
			RT5633_P_DAC_REF_BIT, 0, NULL, 0),

	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("LINE1L"),
	SND_SOC_DAPM_INPUT("LINE2L"),
	SND_SOC_DAPM_INPUT("LINE1R"),
	SND_SOC_DAPM_INPUT("LINE2R"),

	SND_SOC_DAPM_MICBIAS("Mic Bias1", RT5633_PWR_MANAG_ADD2,
				RT5633_P_MICBIAS1_BIT, 0),
	SND_SOC_DAPM_MICBIAS("Mic Bias2", RT5633_PWR_MANAG_ADD2,
				RT5633_P_MICBIAS2_BIT, 0),

	SND_SOC_DAPM_PGA("Mic1 Boost", RT5633_PWR_MANAG_ADD2,
			RT5633_P_MIC1_BST_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Mic2 Boost", RT5633_PWR_MANAG_ADD2,
			RT5633_P_MIC2_BST_BIT, 0, NULL, 0),

	SND_SOC_DAPM_PGA("LINE1L Inp Vol", RT5633_PWR_MANAG_ADD2,
				RT5633_P_L1_L_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LINE1R Inp Vol", RT5633_PWR_MANAG_ADD2,
				RT5633_P_L1_R_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LINE2L Inp Vol", RT5633_PWR_MANAG_ADD2,
				RT5633_P_L2_L_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LINE2R Inp Vol", RT5633_PWR_MANAG_ADD2,
				RT5633_P_L2_R_BIT, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("RECMIXL Mixer", RT5633_PWR_MANAG_ADD2,
		RT5633_P_RM_L_BIT, 0, rt5633_recmixl_mixer_controls,
		ARRAY_SIZE(rt5633_recmixl_mixer_controls)),
	SND_SOC_DAPM_MIXER("RECMIXR Mixer", RT5633_PWR_MANAG_ADD2,
		RT5633_P_RM_R_BIT, 0, rt5633_recmixr_mixer_controls,
		ARRAY_SIZE(rt5633_recmixr_mixer_controls)),

	SND_SOC_DAPM_ADC("Left ADC", "HIFI Capture", RT5633_PWR_MANAG_ADD1,
					RT5633_P_ADC_L_CLK_BIT, 0),
	SND_SOC_DAPM_ADC("Right ADC", "HIFI Capture", RT5633_PWR_MANAG_ADD1,
					RT5633_P_ADC_R_CLK_BIT, 0),
	SND_SOC_DAPM_DAC("Left DAC", "HIFI Playback", RT5633_PWR_MANAG_ADD1,
					RT5633_P_DAC_L_CLK_BIT, 0),
	SND_SOC_DAPM_DAC("Right DAC", "HIFI Playback", RT5633_PWR_MANAG_ADD1,
					RT5633_P_DAC_R_CLK_BIT, 0),

	SND_SOC_DAPM_SUPPLY("Left DAC DF2SE", RT5633_PWR_MANAG_ADD1,
			RT5633_P_DAC_L_TO_MIX_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Right DAC DF2SE", RT5633_PWR_MANAG_ADD1,
			RT5633_P_DAC_R_TO_MIX_BIT, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("HPMIXL Mixer", RT5633_PWR_MANAG_ADD2,
		RT5633_P_HM_L_BIT, 0, rt5633_hp_mixl_mixer_controls,
		ARRAY_SIZE(rt5633_hp_mixl_mixer_controls)),
	SND_SOC_DAPM_MIXER("HPMIXR Mixer", RT5633_PWR_MANAG_ADD2,
		RT5633_P_HM_R_BIT, 0, rt5633_hp_mixr_mixer_controls,
		ARRAY_SIZE(rt5633_hp_mixr_mixer_controls)),
	SND_SOC_DAPM_MIXER("AUXMIXL Mixer", RT5633_PWR_MANAG_ADD2,
		RT5633_P_AM_L_BIT, 0, rt5633_auxmixl_mixer_controls,
		ARRAY_SIZE(rt5633_auxmixl_mixer_controls)),
	SND_SOC_DAPM_MIXER("AUXMIXR Mixer", RT5633_PWR_MANAG_ADD2,
		RT5633_P_AM_R_BIT, 0, rt5633_auxmixr_mixer_controls,
		ARRAY_SIZE(rt5633_auxmixr_mixer_controls)),
	SND_SOC_DAPM_MIXER("SPXMIX Mixer", RT5633_PWR_MANAG_ADD2,
		RT5633_P_SM_BIT, 0, rt5633_spkmixr_mixer_controls,
		ARRAY_SIZE(rt5633_spkmixr_mixer_controls)),

	SND_SOC_DAPM_PGA("Left SPK Vol", RT5633_PWR_MANAG_ADD4,
			RT5633_P_SPK_L_VOL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right SPK Vol", RT5633_PWR_MANAG_ADD4,
			RT5633_P_SPK_R_VOL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA_E("Left HP Vol", RT5633_PWR_MANAG_ADD4,
			RT5633_P_HP_L_VOL_BIT, 0, NULL, 0, rt5633_hp_event,
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_E("Right HP Vol", RT5633_PWR_MANAG_ADD4,
			RT5633_P_HP_R_VOL_BIT, 0, NULL, 0, rt5633_hp_event,
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA("Left AUX Out Vol", RT5633_PWR_MANAG_ADD4,
			RT5633_P_AUXOUT_L_VOL_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right AUX Out Vol", RT5633_PWR_MANAG_ADD4,
			RT5633_P_AUXOUT_R_VOL_BIT, 0, NULL, 0),

	SND_SOC_DAPM_MUX("SPKO Mux", SND_SOC_NOPM, 0, 0, &rt5633_spo_src_mux),

	SND_SOC_DAPM_SUPPLY("Class D Amp", RT5633_PWR_MANAG_ADD1,
				RT5633_P_CLS_D_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Class AB Amp", RT5633_PWR_MANAG_ADD1,
				RT5633_P_CLS_AB_BIT, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("AUXOUTL"),
	SND_SOC_DAPM_OUTPUT("AUXOUTR"),
	SND_SOC_DAPM_OUTPUT("SPOL"),
	SND_SOC_DAPM_OUTPUT("SPOR"),
	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),
};

static const struct snd_soc_dapm_route rt5633_dapm_routes[] = {
	{"I2S", NULL, "DAC Ref"},
	{"I2S", NULL, "PLL", rt5633_check_sclk},

	{"Mic1 Boost", NULL, "MIC1"},
	{"Mic2 Boost", NULL, "MIC2"},

	{"LINE1L Inp Vol", NULL, "LINE1L"},
	{"LINE1R Inp Vol", NULL, "LINE1R"},

	{"LINE2L Inp Vol", NULL, "LINE2L"},
	{"LINE2R Inp Vol", NULL, "LINE2R"},

	{"RECMIXL Mixer", "HPMIXL Capture Switch", "HPMIXL Mixer"},
	{"RECMIXL Mixer", "AUXMIXL Capture Switch", "AUXMIXL Mixer"},
	{"RECMIXL Mixer", "SPKMIX Capture Switch", "SPXMIX Mixer"},
	{"RECMIXL Mixer", "LINE1L Capture Switch", "LINE1L Inp Vol"},
	{"RECMIXL Mixer", "LINE2L Capture Switch", "LINE2L Inp Vol"},
	{"RECMIXL Mixer", "MIC1 Capture Switch", "Mic1 Boost"},
	{"RECMIXL Mixer", "MIC2 Capture Switch", "Mic2 Boost"},

	{"RECMIXR Mixer", "HPMIXR Capture Switch", "HPMIXR Mixer"},
	{"RECMIXR Mixer", "AUXMIXR Capture Switch", "AUXMIXR Mixer"},
	{"RECMIXR Mixer", "SPKMIX Capture Switch", "SPXMIX Mixer"},
	{"RECMIXR Mixer", "LINE1R Capture Switch", "LINE1R Inp Vol"},
	{"RECMIXR Mixer", "LINE2R Capture Switch", "LINE2R Inp Vol"},
	{"RECMIXR Mixer", "MIC1 Capture Switch", "Mic1 Boost"},
	{"RECMIXR Mixer", "MIC2 Capture Switch", "Mic2 Boost"},

	{"Left ADC", NULL, "I2S"},
	{"Left ADC", NULL, "RECMIXL Mixer"},
	{"Right ADC", NULL, "I2S"},
	{"Right ADC", NULL, "RECMIXR Mixer"},

	{"Left DAC", NULL, "Left DAC DF2SE"},
	{"Left DAC", NULL, "I2S"},
	{"Left DAC", NULL, "Right DAC DF2SE"},
	{"Right DAC", NULL, "I2S"},

	{"HPMIXL Mixer", "RECMIXL Playback Switch", "RECMIXL Mixer"},
	{"HPMIXL Mixer", "MIC1 Playback Switch", "Mic1 Boost"},
	{"HPMIXL Mixer", "MIC2 Playback Switch", "Mic2 Boost"},
	{"HPMIXL Mixer", "LINE1 Playback Switch", "LINE1L Inp Vol"},
	{"HPMIXL Mixer", "LINE2 Playback Switch", "LINE2L Inp Vol"},
	{"HPMIXL Mixer", "DAC Playback Switch", "Left DAC"},

	{"HPMIXR Mixer", "RECMIXR Playback Switch", "RECMIXR Mixer"},
	{"HPMIXR Mixer", "MIC1 Playback Switch", "Mic1 Boost"},
	{"HPMIXR Mixer", "MIC2 Playback Switch", "Mic2 Boost"},
	{"HPMIXR Mixer", "LINE1 Playback Switch", "LINE1R Inp Vol"},
	{"HPMIXR Mixer", "LINE2 Playback Switch", "LINE2R Inp Vol"},
	{"HPMIXR Mixer", "DAC Playback Switch", "Right DAC"},

	{"AUXMIXL Mixer", "RECMIXL Playback Switch", "RECMIXL Mixer"},
	{"AUXMIXL Mixer", "MIC1 Playback Switch", "Mic1 Boost"},
	{"AUXMIXL Mixer", "MIC2 Playback Switch", "Mic2 Boost"},
	{"AUXMIXL Mixer", "LINE1 Playback Switch", "LINE1L Inp Vol"},
	{"AUXMIXL Mixer", "LINE2 Playback Switch", "LINE2L Inp Vol"},
	{"AUXMIXL Mixer", "DAC Playback Switch", "Left DAC"},

	{"AUXMIXR Mixer", "RECMIXR Playback Switch", "RECMIXR Mixer"},
	{"AUXMIXR Mixer", "MIC1 Playback Switch", "Mic1 Boost"},
	{"AUXMIXR Mixer", "MIC2 Playback Switch", "Mic2 Boost"},
	{"AUXMIXR Mixer", "LINE1 Playback Switch", "LINE1R Inp Vol"},
	{"AUXMIXR Mixer", "LINE2 Playback Switch", "LINE2R Inp Vol"},
	{"AUXMIXR Mixer", "DAC Playback Switch", "Right DAC"},

	{"SPXMIX Mixer", "MIC1 Playback Switch", "Mic1 Boost"},
	{"SPXMIX Mixer", "MIC2 Playback Switch", "Mic2 Boost"},
	{"SPXMIX Mixer", "DACL Playback Switch", "Left DAC"},
	{"SPXMIX Mixer", "DACR Playback Switch", "Right DAC"},
	{"SPXMIX Mixer", "LINE1L Playback Switch", "LINE1L Inp Vol"},
	{"SPXMIX Mixer", "LINE1R Playback Switch", "LINE1R Inp Vol"},
	{"SPXMIX Mixer", "LINE2L Playback Switch", "LINE2L Inp Vol"},
	{"SPXMIX Mixer", "LINE2R Playback Switch", "LINE2R Inp Vol"},

	{"SPKO Mux", "HPMIX", "HPMIXL Mixer"},
	{"SPKO Mux", "SPKMIX", "SPXMIX Mixer"},
	{"SPKO Mux", "AUXMIX", "AUXMIXL Mixer"},

	{"Left SPK Vol",  NULL, "SPKO Mux"},
	{"Right SPK Vol",  NULL, "SPKO Mux"},

	{"Right HP Vol",  NULL, "HPMIXR Mixer"},
	{"Left HP Vol",  NULL, "HPMIXL Mixer"},

	{"Left AUX Out Vol",  NULL, "AUXMIXL Mixer"},
	{"Right AUX Out Vol",  NULL, "AUXMIXR Mixer"},

	{"AUXOUTL", NULL, "Left AUX Out Vol"},
	{"AUXOUTR", NULL, "Right AUX Out Vol"},
	{"SPOL", NULL, "Class D Amp", rt5633_check_classd},
	{"SPOL", NULL, "Class AB Amp", rt5633_check_classab},
	{"SPOL", NULL, "Left SPK Vol"},
	{"SPOR", NULL, "Class D Amp", rt5633_check_classd},
	{"SPOR", NULL, "Class AB Amp", rt5633_check_classab},
	{"SPOR", NULL, "Right SPK Vol"},
	{"HPOL", NULL, "Left HP Vol"},
	{"HPOR", NULL, "Right HP Vol"},
};

struct _coeff_div{
	unsigned int mclk;
	unsigned int bclk;
	unsigned int rate;
	unsigned int reg_val;
};

/*PLL divisors*/
struct _pll_div {
	u32 pll_in;
	u32 pll_out;
	u16 regvalue;
};

static struct _pll_div codec_master_pll_div[] = {
	{  2048000,   8192000,  0x0ea0},
	{  3686400,   8192000,  0x4e27},
	{ 12000000,   8192000,  0x456b},
	{ 13000000,   8192000,  0x495f},
	{ 13100000,   8192000,  0x0320},
	{  2048000,  11289600,  0xf637},
	{  3686400,  11289600,  0x2f22},
	{ 12000000,  11289600,  0x3e2f},
	{ 13000000,  11289600,  0x4d5b},
	{ 13100000,  11289600,  0x363b},
	{  2048000,  16384000,  0x1ea0},
	{  3686400,  16384000,  0x9e27},
	{ 12000000,  16384000,  0x452b},
	{ 13000000,  16384000,  0x542f},
	{ 13100000,  16384000,  0x03a0},
	{  2048000,  16934400,  0xe625},
	{  3686400,  16934400,  0x9126},
	{ 12000000,  16934400,  0x4d2c},
	{ 13000000,  16934400,  0x742f},
	{ 13100000,  16934400,  0x3c27},
	{  2048000,  22579200,  0x2aa0},
	{  3686400,  22579200,  0x2f20},
	{ 12000000,  22579200,  0x7e2f},
	{ 13000000,  22579200,  0x742f},
	{ 13100000,  22579200,  0x3c27},
	{  2048000,  24576000,  0x2ea0},
	{  3686400,  24576000,  0xee27},
	{ 12000000,  24576000,  0x2915},
	{ 13000000,  24576000,  0x772e},
	{ 13100000,  24576000,  0x0d20},
	{ 26000000,  24576000,  0x2027},
	{ 26000000,  22579200,  0x392f},
	{ 24576000,  22579200,  0x0921},
	{ 24576000,  24576000,  0x02a0},
};

static struct _pll_div codec_slave_pll_div[] = {
	{   256000,   2048000,  0x46f0},
	{   256000,   4096000,  0x3ea0},
	{   352800,   5644800,  0x3ea0},
	{   512000,   8192000,  0x3ea0},
	{  1024000,   8192000,  0x46f0},
	{   705600,  11289600,  0x3ea0},
	{  1024000,  16384000,  0x3ea0},
	{  1411200,  22579200,  0x3ea0},
	{  1536000,  24576000,  0x3ea0},
	{  2048000,  16384000,  0x1ea0},
	{  2822400,  22579200,  0x1ea0},
	{  2822400,  45158400,  0x5ec0},
	{  5644800,  45158400,  0x46f0},
	{  3072000,  24576000,  0x1ea0},
	{  3072000,  49152000,  0x5ec0},
	{  6144000,  49152000,  0x46f0},
	{   705600,  11289600,  0x3ea0},
	{   705600,   8467200,  0x3ab0},
	{ 24576000,  24576000,  0x02a0},
	{  1411200,  11289600,  0x1690},
	{  2822400,  11289600,  0x0a90},
	{  1536000,  12288000,  0x1690},
	{  3072000,  12288000,  0x0a90},
};

static struct _coeff_div coeff_div[] = {
	//sysclk is 256fs
	{ 2048000,  8000 * 32,  8000, 0x1000},
	{ 2048000,  8000 * 64,  8000, 0x0000},
	{ 2822400, 11025 * 32, 11025, 0x1000},
	{ 2822400, 11025 * 64, 11025, 0x0000},
	{ 4096000, 16000 * 32, 16000, 0x1000},
	{ 4096000, 16000 * 64, 16000, 0x0000},
	{ 5644800, 22050 * 32, 22050, 0x1000},
	{ 5644800, 22050 * 64, 22050, 0x0000},
	{ 8192000, 32000 * 32, 32000, 0x1000},
	{ 8192000, 32000 * 64, 32000, 0x0000},
	{11289600, 44100 * 32, 44100, 0x1000},
	{11289600, 44100 * 64, 44100, 0x0000},
	{12288000, 48000 * 32, 48000, 0x1000},
	{12288000, 48000 * 64, 48000, 0x0000},
	//sysclk is 512fs
	{ 4096000,  8000 * 32,  8000, 0x3000},
	{ 4096000,  8000 * 64,  8000, 0x2000},
	{ 5644800, 11025 * 32, 11025, 0x3000},
	{ 5644800, 11025 * 64, 11025, 0x2000},
	{ 8192000, 16000 * 32, 16000, 0x3000},
	{ 8192000, 16000 * 64, 16000, 0x2000},
	{11289600, 22050 * 32, 22050, 0x3000},
	{11289600, 22050 * 64, 22050, 0x2000},
	{16384000, 32000 * 32, 32000, 0x3000},
	{16384000, 32000 * 64, 32000, 0x2000},
	{22579200, 44100 * 32, 44100, 0x3000},
	{22579200, 44100 * 64, 44100, 0x2000},
	{24576000, 48000 * 32, 48000, 0x3000},
	{24576000, 48000 * 64, 48000, 0x2000},
	//SYSCLK is 22.5792Mhz or 24.576Mhz(8k to 48k)
	{24576000, 48000 * 32, 48000, 0x3000},
	{24576000, 48000 * 64, 48000, 0x2000},
	{22579200, 44100 * 32, 44100, 0x3000},
	{22579200, 44100 * 64, 44100, 0x2000},
	{24576000, 32000 * 32, 32000, 0x1080},
	{24576000, 32000 * 64, 32000, 0x0080},
	{22579200, 22050 * 32, 22050, 0x5000},
	{22579200, 22050 * 64, 22050, 0x4000},
	{24576000, 16000 * 32, 16000, 0x3080},
	{24576000, 16000 * 64, 16000, 0x2080},
	{22579200, 11025 * 32, 11025, 0x7000},
	{22579200, 11025 * 64, 11025, 0x6000},
	{24576000,  8000 * 32,  8000, 0x7080},
	{24576000,  8000 * 64,  8000, 0x6080},
};

static int get_coeff(int mclk, int rate, int timesofbclk)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++)
		if ((coeff_div[i].mclk == mclk)
			&& (coeff_div[i].rate == rate)
			&& ((coeff_div[i].bclk / coeff_div[i].rate) == timesofbclk))
			return i;
	return -1;
}

static int get_coeff_in_slave_mode(int mclk, int rate)
{
	return get_coeff(mclk, rate, BLCK_FREQ);
}

static int get_coeff_in_master_mode(int mclk, int rate)
{
	return get_coeff(mclk, rate ,BLCK_FREQ);
}

static int rt5633_hifi_pcm_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct rt5633_priv *rt5633 = snd_soc_codec_get_drvdata(codec);
	int rate = params_rate(params), coeff = 0;
	unsigned int iface = 0;

	dev_dbg(codec->dev, "enter %s\n", __func__);
	if (rt5633->master)
		coeff = get_coeff_in_master_mode(rt5633->sysclk, rate);
	else
		coeff = get_coeff_in_slave_mode(rt5633->sysclk, rate);
	if (coeff < 0) {
		dev_err(codec->dev, "get_coeff err!\n");
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= RT5633_SDP_I2S_DL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= RT5633_SDP_I2S_DL_24;
		break;
	case SNDRV_PCM_FORMAT_S8:
		iface |= RT5633_SDP_I2S_DL_8;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, RT5633_SDP_CTRL,
		RT5633_SDP_I2S_DL_MASK, iface);
	snd_soc_write(codec, RT5633_STEREO_AD_DA_CLK_CTRL,
			coeff_div[coeff].reg_val);

	return 0;
}

static int rt5633_hifi_codec_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5633_priv *rt5633 = snd_soc_codec_get_drvdata(codec);
	unsigned int iface = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		rt5633->master = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface |= RT5633_SDP_MODE_SEL_SLAVE;
		rt5633->master = 0;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= RT5633_SDP_I2S_DF_RIGHT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= RT5633_SDP_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface  |= RT5633_SDP_I2S_DF_PCM;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= RT5633_SDP_I2S_BCLK_POL_INV;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, RT5633_SDP_CTRL,
		RT5633_SDP_MODE_SEL_MASK | RT5633_SDP_I2S_DF_MASK |
		RT5633_SDP_I2S_BCLK_POL_INV, iface);

	return 0;
}
static int rt5633_hifi_codec_set_dai_sysclk(
	struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5633_priv *rt5633 = snd_soc_codec_get_drvdata(codec);

	if (freq == rt5633->sysclk)
		return 0;

	rt5633->sysclk = freq;
	dev_dbg(dai->dev, "Sysclk is %dHz\n", freq);

	return 0;
}

static int rt5633_codec_set_dai_pll(struct snd_soc_dai *dai, int pll_id,
		int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5633_priv *rt5633 = snd_soc_codec_get_drvdata(codec);
	struct _pll_div *pll_tab;
	int i, ret = -EINVAL, tab_num;
	unsigned int val, mask;

	dev_dbg(codec->dev, "enter %s\n", __func__);
	if (!freq_in || !freq_out) {
		dev_dbg(codec->dev, "PLL disabled\n");
		snd_soc_update_bits(codec, RT5633_GBL_CLK_CTRL,
			RT5633_SCLK_SRC_MASK, RT5633_SCLK_SRC_MCLK);
		return 0;
	}

	if (rt5633->master) {
		tab_num = ARRAY_SIZE(codec_master_pll_div);
		pll_tab = codec_master_pll_div;
		val = RT5633_SCLK_SRC_PLL | RT5633_PLL_SRC_MCLK;
		mask = RT5633_SCLK_SRC_MASK | RT5633_PLL_SRC_MASK;
	} else {
		tab_num = ARRAY_SIZE(codec_slave_pll_div);
		pll_tab = codec_slave_pll_div;
		val = RT5633_SCLK_SRC_PLL | RT5633_PLL_SRC_BCLK;
		mask = RT5633_SCLK_SRC_MASK | RT5633_PLL_SRC_MASK;
	}

	for (i = 0; i < tab_num; i ++)
		if (freq_in == pll_tab[i].pll_in &&
			freq_out == pll_tab[i].pll_out) {
			snd_soc_write(codec, RT5633_PLL_CTRL,
					pll_tab[i].regvalue);
			msleep(20);
			snd_soc_update_bits(codec,
				RT5633_GBL_CLK_CTRL, mask, val);
			ret = 0;
			break;
		}

	return ret;
}

static int rt5633_set_bias_level(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	struct rt5633_priv *rt5633 = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		snd_soc_update_bits(codec, RT5633_SPK_OUT_VOL,
			RT5633_L_MUTE, 0);
		snd_soc_update_bits(codec, RT5633_HP_OUT_VOL,
			RT5633_L_MUTE | RT5633_R_MUTE, 0);
		break;

	case SND_SOC_BIAS_PREPARE:
		snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD2,
			RT5633_P_MICBIAS1, RT5633_P_MICBIAS1);
		break;

	case SND_SOC_BIAS_STANDBY:
		snd_soc_update_bits(codec, RT5633_SPK_OUT_VOL,
			RT5633_L_MUTE, RT5633_L_MUTE);
		snd_soc_update_bits(codec, RT5633_HP_OUT_VOL,
			RT5633_L_MUTE | RT5633_R_MUTE,
			RT5633_L_MUTE | RT5633_R_MUTE);
		snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD2,
					RT5633_P_MICBIAS1, 0);
		if (snd_soc_codec_get_bias_level(codec) == SND_SOC_BIAS_OFF) {
			snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD3,
				RT5633_P_VREF | RT5633_P_MAIN_BIAS,
				RT5633_P_VREF | RT5633_P_MAIN_BIAS);
			msleep(110);
			snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD3,
					RT5633_P_DIS_FAST_VREF,
					RT5633_P_DIS_FAST_VREF);
			regcache_mark_dirty(rt5633->regmap);
			regcache_sync(rt5633->regmap);
			rt5633_index_sync(codec);
		}
		break;

	case SND_SOC_BIAS_OFF:
		snd_soc_write(codec, RT5633_PWR_MANAG_ADD1, 0x0000);
		snd_soc_write(codec, RT5633_PWR_MANAG_ADD2, 0x0000);
		snd_soc_write(codec, RT5633_PWR_MANAG_ADD3, 0x0000);
		snd_soc_write(codec, RT5633_PWR_MANAG_ADD4, 0x0000);
		break;

	default:
		break;
	}

	return 0;
}

static int rt5633_probe(struct snd_soc_codec *codec)
{
	struct rt5633_priv *rt5633 = snd_soc_codec_get_drvdata(codec);

	rt5633_reset(codec);
	snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD3,
		RT5633_P_VREF | RT5633_P_MAIN_BIAS,
		RT5633_P_VREF | RT5633_P_MAIN_BIAS);
	msleep(110);
	snd_soc_update_bits(codec, RT5633_PWR_MANAG_ADD3,
			RT5633_P_DIS_FAST_VREF,
			RT5633_P_DIS_FAST_VREF);
	regmap_multi_reg_write(rt5633->regmap,
		rt5633_init_list, ARRAY_SIZE(rt5633_init_list));
	snd_soc_codec_force_bias_level(codec, SND_SOC_BIAS_OFF);
	rt5633->codec = codec;

	return 0;
}

static int rt5633_remove(struct snd_soc_codec *codec)
{
	rt5633_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

#ifdef CONFIG_PM
static int rt5633_suspend(struct snd_soc_codec *codec)
{
	rt5633_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int rt5633_resume(struct snd_soc_codec *codec)
{
	rt5633_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}
#else
#define rt5633_suspend NULL
#define rt5633_resume NULL
#endif

#define RT5633_STEREO_RATES (SNDRV_PCM_RATE_8000_48000)
#define RT5633_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S8)

static const struct snd_soc_dai_ops rt5633_ops = {
	.hw_params = rt5633_hifi_pcm_params,
	.set_fmt = rt5633_hifi_codec_set_dai_fmt,
	.set_sysclk = rt5633_hifi_codec_set_dai_sysclk,
	.set_pll = rt5633_codec_set_dai_pll,
};

static struct snd_soc_dai_driver rt5633_dai[] = {
	{
		.name = "rt5633-hifi",
		.id = RT5633_AIF1,
		.playback = {
			.stream_name = "HIFI Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5633_STEREO_RATES,
			.formats = RT5633_FORMATS,
		}	,
		.capture = {
			.stream_name = "HIFI Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5633_STEREO_RATES,
			.formats = RT5633_FORMATS,
		},
		.ops =&rt5633_ops,
	},
	{
		.name = "rt5633-voice",
		.id = RT5633_AIF2,
	}
};

static struct snd_soc_codec_driver soc_codec_dev_rt5633 = {
	.probe = rt5633_probe,
	.remove = rt5633_remove,
	.suspend = rt5633_suspend,
	.resume = rt5633_resume,
	.set_bias_level = rt5633_set_bias_level,
	.component_driver = {
		.controls		= rt5633_snd_controls,
		.num_controls		= ARRAY_SIZE(rt5633_snd_controls),
		.dapm_widgets		= rt5633_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(rt5633_dapm_widgets),
		.dapm_routes		= rt5633_dapm_routes,
		.num_dapm_routes	= ARRAY_SIZE(rt5633_dapm_routes),
	},
};

static const struct of_device_id rt5633_of_match[] = {
	{ .compatible = "realtek,rt5633", },
	{},
};
MODULE_DEVICE_TABLE(of, rt5633_of_match);

static const struct i2c_device_id rt5633_i2c_id[] = {
	{ "rt5633", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5633_i2c_id);

static const struct regmap_config rt5633_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.use_single_rw = true,
	.max_register = RT5633_VENDOR_ID2,
	.volatile_reg = rt5633_volatile_register,
	.readable_reg = rt5633_readable_register,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = rt5633_reg,
	.num_reg_defaults = ARRAY_SIZE(rt5633_reg),
};

static int rt5633_i2c_probe(struct i2c_client *i2c,
		    const struct i2c_device_id *id)
{
	struct rt5633_priv *rt5633;
	int ret;

	rt5633 = devm_kzalloc(&i2c->dev, sizeof(struct rt5633_priv),
		GFP_KERNEL);

	if (rt5633 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, rt5633);

	rt5633->regmap = devm_regmap_init_i2c(i2c, &rt5633_regmap);
	if (IS_ERR(rt5633->regmap)) {
		ret = PTR_ERR(rt5633->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_rt5633,
			rt5633_dai, ARRAY_SIZE(rt5633_dai));

        if (ret != 0)
                dev_err(&i2c->dev, "Failed to register codec: %d\n", ret);

	return ret;
}

static int rt5633_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);

	return 0;
}

static void rt5633_i2c_shutdown(struct i2c_client *client)
{
	struct rt5633_priv *rt5633 = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = rt5633->codec;

	if (codec != NULL)
		rt5633_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static struct i2c_driver rt5633_i2c_driver = {
	.driver = {
		.name = "rt5633",
		.of_match_table = of_match_ptr(rt5633_of_match),
	},
	.probe = rt5633_i2c_probe,
	.remove   = rt5633_i2c_remove,
	.shutdown = rt5633_i2c_shutdown,
	.id_table = rt5633_i2c_id,
};

module_i2c_driver(rt5633_i2c_driver);

MODULE_DESCRIPTION("ASoC RT5633 driver");
MODULE_AUTHOR("Johnny Hsu <johnnyhsu@realtek.com>");
MODULE_LICENSE("GPL");
