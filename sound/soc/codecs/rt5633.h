/*
 * rt5633.h  --  RT5633 ALSA SoC audio driver
 *
 * Copyright 2011 Realtek Microelectronics
 * Author: Johnny Hsu <johnnyhsu@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RT5633_H__
#define __RT5633_H__

#define RT5633_RESET				0x00
#define RT5633_SPK_OUT_VOL			0x02
#define RT5633_SPK_HP_MIXER_CTRL		0x03
#define RT5633_HP_OUT_VOL			0x04
#define RT5633_AUXOUT_VOL			0x06
#define RT5633_LINE_IN_1_VOL			0x08
#define RT5633_LINE_IN_2_VOL			0x0a
#define RT5633_DAC_CTRL				0x0c
#define RT5633_DAC_DIG_VOL			0x0e
#define RT5633_MIC_CTRL_1			0x10
#define RT5633_ADC_CTRL				0x12
#define RT5633_REC_MIXER_CTRL			0x14
#define RT5633_ADC_DIG_VOL			0x16
#define RT5633_HPMIXER_CTRL			0x18
#define RT5633_AUXMIXER_CTRL			0x1a
#define RT5633_SPKMIXER_CTRL			0x1c
#define RT5633_SPK_AMP_CTRL			0x1e
#define RT5633_MIC_CTRL_2			0x22
#define RT5633_SDP_CTRL				0x34
#define RT5633_STEREO_AD_DA_CLK_CTRL		0x38
#define RT5633_PWR_MANAG_ADD1			0x3a
#define RT5633_PWR_MANAG_ADD2			0x3b
#define RT5633_PWR_MANAG_ADD3			0x3c
#define RT5633_PWR_MANAG_ADD4			0x3e
#define RT5633_GEN_PUR_CTRL_1			0x40
#define RT5633_GBL_CLK_CTRL			0x42
#define RT5633_PLL_CTRL				0x44
#define RT5633_DIG_BEEP_IRQ_CTRL		0x48
#define RT5633_INT_ST_STICKY_CTRL		0x4a
#define RT5633_GPIO_CTRL_1			0x4c
#define RT5633_GPIO_CTRL_2			0x4d
#define RT5633_GEN_PUR_CTRL_2			0x52
#define RT5633_DEPOP_CTRL_1			0x54
#define RT5633_DEPOP_CTRL_2			0x56
#define RT5633_JACK_DET_CTRL			0x5a
#define RT5633_ZC_SM_CTRL_1			0x5c
#define RT5633_ZC_SM_CTRL_2			0x5d
#define RT5633_ALC_CTRL_1			0x64
#define RT5633_ALC_CTRL_2			0x65
#define RT5633_ALC_CTRL_3			0x66
#define RT5633_PSEUDO_SPATL_CTRL		0x68
#define RT5633_PRI_REG_ADD			0x6a
#define RT5633_PRI_REG_DATA			0x6c
#define RT5633_EQ_CTRL_1			0x6e
#define RT5633_EQ_CTRL_2			0x70
#define RT5633_VERSION				0x7a
#define RT5633_VENDOR_ID1			0x7c
#define RT5633_VENDOR_ID2			0x7e


/* global definition */
#define RT5633_L_MUTE				(0x1 << 15)
#define RT5633_L_MUTE_SFT			15
#define RT5633_L_VOL_SFT			8
#define RT5633_R_MUTE				(0x1 << 7)
#define RT5633_R_MUTE_SFT			7
#define RT5633_R_VOL_SFT			0

/* Speaker Output Control (0x02) */
#define RT5633_SPON_CTL_MASK			(0x3 << 13)
#define RT5633_SPON_CTL_SFT			13
#define RT5633_SPK_CLSAB_M_SFT			12
#define RT5633_SPKL_VOL_MASK			(0x1f << 5)
#define RT5633_SPKL_VOL_SFT			5
#define RT5633_SPKR_VOL_MASK			(0x1f)
#define RT5633_SPKR_VOL_SFT			0

/* Speaker/HP Output Mixer Control (0x03) */
#define RT5633_UM_DACL_TO_SPOL_MIXER		(0x1 << 8)
#define RT5633_UM_DACR_TO_SPOR_MIXER		(0x1 << 6)
#define RT5633_UM_DAC_TO_HPO_MIXER		(0x1 << 5)
#define RT5633_UM_HPVOL_TO_HPO_MIXER		(0x1 << 4)

/* Headphone Output Control (0x04) */
#define RT5633_HP_L_VOL_SEL_MASK		(0x1 << 14)
#define RT5633_HP_L_VOL_SEL_VMID		(0x0 << 14)
#define RT5633_HP_L_VOL_SEL_HPMIX_L		(0x1 << 14)
#define RT5633_HP_R_VOL_SEL_MASK		(0x1 << 6)
#define RT5633_HP_R_VOL_SEL_VMID		(0x0 << 6)
#define RT5633_HP_R_VOL_SEL_HPMIX_R		(0x1 << 6)

/* Output Control for AUXOUT(0x06) */
#define RT5633_AUXOUT_MODE_SEL_MASK		(0x1 << 14)
#define RT5633_AUXOUT_MODE_SEL_SFT		14
#define RT5633_AUXOUT_MODE_SEL_MONO		(0x0 << 14)
#define RT5633_AUXOUT_MODE_SEL_STEREO		(0x1 << 14)

/* Microphone Input Control(0x10) */
#define RT5633_MIC_1_MODE_SEL_MASK		(0x1 << 15)
#define RT5633_MIC_1_MODE_SEL_SFT		15
#define RT5633_MIC_1_MODE_SEL_SE		(0x0 << 15)
#define RT5633_MIC_1_MODE_SEL_DIFF		(0x1 << 15)
#define RT5633_MIC_2_MODE_SEL_MASK		(0x1 << 7)
#define RT5633_MIC_2_MODE_SEL_SFT		7
#define RT5633_MIC_2_MODE_SEL_SE		(0x0 << 7)
#define RT5633_MIC_2_MODE_SEL_DIFF		(0x1 << 7)

/* ADC Recording Mixer Control (0x14) */
#define RT5633_M_RM_L_HPM			(0x1 << 14)
#define RT5633_M_RM_L_HPM_SFT			14
#define RT5633_M_RM_L_AUXM			(0x1 << 13)
#define RT5633_M_RM_L_AUXM_SFT			13
#define RT5633_M_RM_L_SPKM			(0x1 << 12)
#define RT5633_M_RM_L_SPKM_SFT			12
#define RT5633_M_RM_L_LINE1			(0x1 << 11)
#define RT5633_M_RM_L_LINE1_SFT			11
#define RT5633_M_RM_L_LINE2			(0x1 << 10)
#define RT5633_M_RM_L_LINE2_SFT			10
#define RT5633_M_RM_L_MIC1			(0x1 << 9)
#define RT5633_M_RM_L_MIC1_SFT			9
#define RT5633_M_RM_L_MIC2			(0x1 << 8)
#define RT5633_M_RM_L_MIC2_SFT			8
#define RT5633_M_RM_R_HPM			(0x1 << 6)
#define RT5633_M_RM_R_HPM_SFT			6
#define RT5633_M_RM_R_AUXM			(0x1 << 5)
#define RT5633_M_RM_R_AUXM_SFT			5
#define RT5633_M_RM_R_SPKM			(0x1 << 4)
#define RT5633_M_RM_R_SPKM_SFT			4
#define RT5633_M_RM_R_LINE1			(0x1 << 3)
#define RT5633_M_RM_R_LINE1_SFT			3
#define RT5633_M_RM_R_LINE2			(0x1 << 2)
#define RT5633_M_RM_R_LINE2_SFT			2
#define RT5633_M_RM_R_MIC1			(0x1 << 1)
#define RT5633_M_RM_R_MIC1_SFT			1
#define RT5633_M_RM_R_MIC2			(0x1)
#define RT5633_M_RM_R_MIC2_SFT			0

/* Headphone Mixer Control (0x18) */
#define RT5633_M_HPM_L_RM			(0x1 << 13)
#define RT5633_M_HPM_L_RM_SFT			13
#define RT5633_M_HPM_L_MIC1			(0x1 << 12)
#define RT5633_M_HPM_L_MIC1_SFT			12
#define RT5633_M_HPM_L_MIC2			(0x1 << 11)
#define RT5633_M_HPM_L_MIC2_SFT			11
#define RT5633_M_HPM_L_LINE1			(0x1 << 10)
#define RT5633_M_HPM_L_LINE1_SFT		10
#define RT5633_M_HPM_L_LINE2			(0x1 << 9)
#define RT5633_M_HPM_L_LINE2_SFT		9
#define RT5633_M_HPM_L_DAC			(0x1 << 8)
#define RT5633_M_HPM_L_DAC_SFT			8
#define RT5633_M_HPM_R_RM			(0x1 << 5)
#define RT5633_M_HPM_R_RM_SFT			5
#define RT5633_M_HPM_R_MIC1			(0x1 << 4)
#define RT5633_M_HPM_R_MIC1_SFT			4
#define RT5633_M_HPM_R_MIC2			(0x1 << 3)
#define RT5633_M_HPM_R_MIC2_SFT			3
#define RT5633_M_HPM_R_LINE1			(0x1 << 2)
#define RT5633_M_HPM_R_LINE1_SFT		2
#define RT5633_M_HPM_R_LINE2			(0x1 << 1)
#define RT5633_M_HPM_R_LINE2_SFT		1
#define RT5633_M_HPM_R_DAC			(0x1)
#define RT5633_M_HPM_R_DAC_SFT			0

/* Aux Mixer Control (0x1a) */
#define RT5633_M_AM_L_RM			(0x1 << 13)
#define RT5633_M_AM_L_RM_SFT			13
#define RT5633_M_AM_L_MIC1			(0x1 << 12)
#define RT5633_M_AM_L_MIC1_SFT			12
#define RT5633_M_AM_L_MIC2			(0x1 << 11)
#define RT5633_M_AM_L_MIC2_SFT			11
#define RT5633_M_AM_L_LINE1			(0x1 << 10)
#define RT5633_M_AM_L_LINE1_SFT			10
#define RT5633_M_AM_L_LINE2			(0x1 << 9)
#define RT5633_M_AM_L_LINE2_SFT			9
#define RT5633_M_AM_L_DAC			(0x1 << 8)
#define RT5633_M_AM_L_DAC_SFT			8
#define RT5633_M_AM_R_RM			(0x1 << 5)
#define RT5633_M_AM_R_RM_SFT			5
#define RT5633_M_AM_R_MIC1			(0x1 << 4)
#define RT5633_M_AM_R_MIC1_SFT			4
#define RT5633_M_AM_R_MIC2			(0x1 << 3)
#define RT5633_M_AM_R_MIC2_SFT			3
#define RT5633_M_AM_R_LINE1			(0x1 << 2)
#define RT5633_M_AM_R_LINE1_SFT			2
#define RT5633_M_AM_R_LINE2			(0x1 << 1)
#define RT5633_M_AM_R_LINE2_SFT			1
#define RT5633_M_AM_R_DAC			(0x1)
#define RT5633_M_AM_R_DAC_SFT			0

/* Speaker Mixer Control (0x1c) */
#define RT5633_SPK_VOL_S_MASK			(0x3 << 10)
#define RT5633_SPK_VOL_S_SFT			10
#define RT5633_SPK_VOL_S_VMID			(0x0 << 10)
#define RT5633_SPK_VOL_S_SM			(0x2 << 10)
#define RT5633_SPK_VOL_S_AM			(0x3 << 10)
#define RT5633_M_SM_R_MIC1			(0x1 << 7)
#define RT5633_M_SM_R_MIC1_SFT			7
#define RT5633_M_SM_R_MIC2			(0x1 << 6)
#define RT5633_M_SM_R_MIC2_SFT			6
#define RT5633_M_SM_R_L1_L			(0x1 << 5)
#define RT5633_M_SM_R_L1_L_SFT			5
#define RT5633_M_SM_R_L1_R			(0x1 << 4)
#define RT5633_M_SM_R_L1_R_SFT			4
#define RT5633_M_SM_R_L2_L			(0x1 << 3)
#define RT5633_M_SM_R_L2_L_SFT			3
#define RT5633_M_SM_R_L2_R			(0x1 << 2)
#define RT5633_M_SM_R_L2_R_SFT			2
#define RT5633_M_SM_R_DACL			(0x1 << 1)
#define RT5633_M_SM_R_DACL_SFT			1
#define RT5633_M_SM_R_DACR			(0x1)
#define RT5633_M_SM_R_DACR_SFT			0

/* Speaker Amplifier Control (0x1e) */
#define RT5633_SPK_AMP_MODE_MASK		(0x1 << 15)
#define RT5633_SPK_AMP_MODE_AB			(0x0 << 15)
#define RT5633_SPK_AMP_MODE_D			(0x1 << 15)
#define RT5633_SPK_CLASS_AB_SEL_MASK		(0x1 << 14)
#define RT5633_SPK_CLASS_AB_SEL_STRONG		(0x0 << 14)
#define RT5633_SPK_CLASS_AB_SEL_WEAK		(0x1 << 14)

/* Micphone Input Control 2 (0x22) */
#define RT5633_MIC_BIAS_90_PRECNET_AVDD		1
#define RT5633_MIC_BIAS_75_PRECNET_AVDD		2
#define RT5633_MIC1_BOOST_CTRL_MASK		(0xf << 12)
#define RT5633_MIC1_BOOST_CTRL_SFT		12
#define RT5633_MIC1_BOOST_CTRL_BYPASS		(0x0 << 12)
#define RT5633_MIC1_BOOST_CTRL_20DB		(0x1 << 12)
#define RT5633_MIC1_BOOST_CTRL_24DB		(0x2 << 12)
#define RT5633_MIC1_BOOST_CTRL_30DB		(0x3 << 12)
#define RT5633_MIC1_BOOST_CTRL_35DB		(0x4 << 12)
#define RT5633_MIC1_BOOST_CTRL_40DB		(0x5 << 12)
#define RT5633_MIC1_BOOST_CTRL_44DB		(0x6 << 12)
#define RT5633_MIC1_BOOST_CTRL_50DB		(0x7 << 12)
#define RT5633_MIC1_BOOST_CTRL_52DB		(0x8 << 12)
#define RT5633_MIC2_BOOST_CTRL_MASK		(0xf << 8)
#define RT5633_MIC2_BOOST_CTRL_SFT		8
#define RT5633_MIC2_BOOST_CTRL_BYPASS		(0x0 << 8)
#define RT5633_MIC2_BOOST_CTRL_20DB		(0x1 << 8)
#define RT5633_MIC2_BOOST_CTRL_24DB		(0x2 << 8)
#define RT5633_MIC2_BOOST_CTRL_30DB		(0x3 << 8)
#define RT5633_MIC2_BOOST_CTRL_35DB		(0x4 << 8)
#define RT5633_MIC2_BOOST_CTRL_40DB		(0x5 << 8)
#define RT5633_MIC2_BOOST_CTRL_44DB		(0x6 << 8)
#define RT5633_MIC2_BOOST_CTRL_50DB		(0x7 << 8)
#define RT5633_MIC2_BOOST_CTRL_52DB		(0x8 << 8)
#define RT5633_MICBIAS1_VOLT_CTRL_MASK		(0x1 << 7)
#define RT5633_MICBIAS1_VOLT_CTRL_90P		(0x0 << 7)
#define RT5633_MICBIAS1_VOLT_CTRL_75P		(0x1 << 7)
#define RT5633_MICBIAS1_S_C_DET_MASK		(0x1 << 6)
#define RT5633_MICBIAS1_S_C_DET_DIS		(0x0 << 6)
#define RT5633_MICBIAS1_S_C_DET_ENA		(0x1 << 6)
#define RT5633_MICBIAS1_SHORT_CURR_MASK		(0x3 << 4)
#define RT5633_MICBIAS1_SHORT_CURR_600UA	(0x0 << 4)
#define RT5633_MICBIAS1_SHORT_CURR_1500UA	(0x1 << 4)
#define RT5633_MICBIAS1_SHORT_CURR_2000UA	(0x2 << 4)
#define RT5633_MICBIAS2_VOLT_CTRL_MASK		(0x1 << 3)
#define RT5633_MICBIAS2_VOLT_CTRL_90P		(0x0 << 3)
#define RT5633_MICBIAS2_VOLT_CTRL_75P		(0x1 << 3)
#define RT5633_MICBIAS2_S_C_DET_MASK		(0x1 << 2)
#define RT5633_MICBIAS2_S_C_DET_DIS		(0x0 << 2)
#define RT5633_MICBIAS2_S_C_DET_ENA		(0x1 << 2)
#define RT5633_MICBIAS2_SHORT_CURR_MASK		(0x3)
#define RT5633_MICBIAS2_SHORT_CURR_600UA	(0x0)
#define RT5633_MICBIAS2_SHORT_CURR_1500UA	(0x1)
#define RT5633_MICBIAS2_SHORT_CURR_2000UA	(0x2)

/* Digital Microphone Control (0x24) */
#define RT5633_DMIC_ENA_MASK			(0x1 << 15)
#define RT5633_DMIC_ENA				(0x1 << 15)
#define RT5633_DMIC_DIS				(0x0 << 15)
#define RT5633_M_ADC_TO_DIGITAL_MIXER      	(0x1 << 14)
#define RT5633_DMIC_L_CH_MUTE_MASK		(0x1 << 13)
#define RT5633_DMIC_L_CH_UNMUTE			(0x0 << 13)
#define RT5633_DMIC_L_CH_MUTE	 		(0x1 << 13)
#define RT5633_DMIC_R_CH_MUTE_MASK		(0x1 << 12)
#define RT5633_DMIC_R_CH_UNMUTE			(0x0 << 12)
#define RT5633_DMIC_R_CH_MUTE			(0x1 << 12)
#define RT5633_DMIC_L_CH_LATCH_MASK		(0x1 << 9)
#define RT5633_DMIC_L_CH_LATCH_RISING		(0x1 << 9)
#define RT5633_DMIC_L_CH_LATCH_FALLING		(0x0 << 9)
#define RT5633_DMIC_R_CH_LATCH_MASK		(0x1 << 8)
#define RT5633_DMIC_R_CH_LATCH_RISING		(0x1 << 8)
#define RT5633_DMIC_R_CH_LATCH_FALLING		(0x0 << 8)
#define RT5633_DMIC_CLK_SET_MASK		(0x7 << 3)
#define RT5633_DMIC_CLK_SET_256FS_DIV2		(0x0 << 3)
#define RT5633_DMIC_CLK_SET_256FS_DIV4		(0x1 << 3)
#define RT5633_DMIC_CLK_SET_256FS_DIV6		(0x2 << 3)
#define RT5633_DMIC_CLK_SET_256FS_DIV12		(0x3 << 3)
#define RT5633_DMIC_CLK_SET_256FS_DIV24		(0x4 << 3)

/* Stereo I2S Serial Data Port Control (0x34) */
#define RT5633_SDP_MODE_SEL_MASK		(0x1 << 15)
#define RT5633_SDP_MODE_SEL_MASTER		(0x0 << 15)
#define RT5633_SDP_MODE_SEL_SLAVE		(0x1 << 15)
#define RT5633_SDP_CP_ADC_L_TO_ADC_R		(0x1 << 14)
#define RT5633_SDP_CP_ADC_R_TO_ADC_L		(0x1 << 13)
#define RT5633_SDP_ADC_CPS_SEL_MASK		(0x3 << 10)
#define RT5633_SDP_ADC_CPS_SEL_OFF		(0x0 << 10)
#define RT5633_SDP_ADC_CPS_SEL_U_LAW		(0x1 << 10)
#define RT5633_SDP_ADC_CPS_SEL_A_LAW		(0x2 << 10)
#define RT5633_SDP_DAC_CPS_SEL_MASK		(0x3 << 8)
#define RT5633_SDP_DAC_CPS_SEL_OFF		(0x0 << 8)
#define RT5633_SDP_DAC_CPS_SEL_U_LAW		(0x1 << 8)
#define RT5633_SDP_DAC_CPS_SEL_A_LAW		(0x2 << 8)
#define RT5633_SDP_I2S_BCLK_POL_INV		(0x1 << 7)
#define RT5633_SDP_DAC_R_INV			(0x1 << 6)
#define RT5633_SDP_ADC_DATA_L_R_SWAP		(0x1 << 5)
#define RT5633_SDP_DAC_DATA_L_R_SWAP		(0x1 << 4)
#define RT5633_SDP_I2S_DL_MASK			(0x3 << 2)
#define RT5633_SDP_I2S_DL_16			(0x0 << 2)
#define RT5633_SDP_I2S_DL_20			(0x1 << 2)
#define RT5633_SDP_I2S_DL_24			(0x2 << 2)
#define RT5633_SDP_I2S_DL_8			(0x3 << 2)
#define RT5633_SDP_I2S_DF_MASK			(0x3)
#define RT5633_SDP_I2S_DF_I2S			(0x0)
#define RT5633_SDP_I2S_DF_RIGHT			(0x1)
#define RT5633_SDP_I2S_DF_LEFT			(0x2)
#define RT5633_SDP_I2S_DF_PCM			(0x3)

/* Stereo AD/DA Clock Control (0x38h) */
#define RT5633_I2S_PRE_DIV_MASK			(0x7 << 13)
#define RT5633_I2S_PRE_DIV_1			(0x0 << 13)
#define RT5633_I2S_PRE_DIV_2			(0x1 << 13)
#define RT5633_I2S_PRE_DIV_4			(0x2 << 13)
#define RT5633_I2S_PRE_DIV_8			(0x3 << 13)
#define RT5633_I2S_PRE_DIV_16			(0x4 << 13)
#define RT5633_I2S_PRE_DIV_32			(0x5 << 13)
#define RT5633_I2S_LRCK_SEL_N_BCLK_MASK		(0x1 << 12)
#define RT5633_I2S_LRCK_SEL_64_BCLK		(0x0 << 12)
#define RT5633_I2S_LRCK_SEL_32_BCLK		(0x1 << 12)
#define RT5633_DAC_OSR_SEL_MASK			(0x3 << 10)
#define RT5633_DAC_OSR_SEL_128FS		(0x3 << 10)
#define RT5633_DAC_OSR_SEL_64FS			(0x3 << 10)
#define RT5633_DAC_OSR_SEL_32FS			(0x3 << 10)
#define RT5633_DAC_OSR_SEL_16FS			(0x3 << 10)
#define RT5633_ADC_OSR_SEL_MASK			(0x3 << 8)
#define RT5633_ADC_OSR_SEL_128FS		(0x3 << 8)
#define RT5633_ADC_OSR_SEL_64FS			(0x3 << 8)
#define RT5633_ADC_OSR_SEL_32FS			(0x3 << 8)
#define RT5633_ADC_OSR_SEL_16FS			(0x3 << 8)
#define RT5633_ADDA_FILTER_CLK_SEL_MASK		(1 << 7)
#define RT5633_ADDA_FILTER_CLK_SEL_256FS	(0 << 7)
#define RT5633_ADDA_FILTER_CLK_SEL_384FS	(1 << 7)

/* Power managment addition 1 (0x3a) */
#define RT5633_P_MAIN_I2S			(0x1 << 15)
#define RT5633_P_MAIN_I2S_BIT			15
#define RT5633_P_CLS_D				(0x1 << 13)
#define RT5633_P_CLS_D_BIT			13
#define RT5633_P_ADC_L_CLK			(0x1 << 12)
#define RT5633_P_ADC_L_CLK_BIT			12
#define RT5633_P_ADC_R_CLK			(0x1 << 11)
#define RT5633_P_ADC_R_CLK_BIT			11
#define RT5633_P_DAC_L_CLK			(0x1 << 10)
#define RT5633_P_DAC_L_CLK_BIT			10
#define RT5633_P_DAC_R_CLK			(0x1 << 9)
#define RT5633_P_DAC_R_CLK_BIT			9
#define RT5633_P_DAC_REF			(0x1 << 8)
#define RT5633_P_DAC_REF_BIT			8
#define RT5633_P_DAC_L_TO_MIX			(0x1 << 7)
#define RT5633_P_DAC_L_TO_MIX_BIT		7
#define RT5633_P_DAC_R_TO_MIX			(0x1 << 6)
#define RT5633_P_DAC_R_TO_MIX_BIT		6
#define RT5633_P_CLS_AB				(0x1 << 5)
#define RT5633_P_CLS_AB_BIT			5
#define RT5633_P_PS_CUR_EN			(0x1 << 4)
#define RT5633_P_PS_CUR_EN_BIT			4

/* Power managment addition 2 (0x3b) */
#define RT5633_P_HM_L				(0x1 << 15)
#define RT5633_P_HM_L_BIT			15
#define RT5633_P_HM_R				(0x1 << 14)
#define RT5633_P_HM_R_BIT			14
#define RT5633_P_AM_L				(0x1 << 13)
#define RT5633_P_AM_L_BIT			13
#define RT5633_P_AM_R				(0x1 << 12)
#define RT5633_P_AM_R_BIT			12
#define RT5633_P_RM_L				(0x1 << 11)
#define RT5633_P_RM_L_BIT			11
#define RT5633_P_RM_R				(0x1 << 10)
#define RT5633_P_RM_R_BIT			10
#define RT5633_P_L1_L				(0x1 << 9)
#define RT5633_P_L1_L_BIT			9
#define RT5633_P_L1_R				(0x1 << 8)
#define RT5633_P_L1_R_BIT			8
#define RT5633_P_L2_L				(0x1 << 7)
#define RT5633_P_L2_L_BIT			7
#define RT5633_P_L2_R				(0x1 << 6)
#define RT5633_P_L2_R_BIT			6
#define RT5633_P_MIC1_BST			(0x1 << 5)
#define RT5633_P_MIC1_BST_BIT			5
#define RT5633_P_MIC2_BST			(0x1 << 4)
#define RT5633_P_MIC2_BST_BIT			4
#define RT5633_P_MICBIAS1			(0x1 << 3)
#define RT5633_P_MICBIAS1_BIT			3
#define RT5633_P_MICBIAS2			(0x1 << 2)
#define RT5633_P_MICBIAS2_BIT			2
#define RT5633_P_PLL				(0x1 << 1)
#define RT5633_P_PLL_BIT			1
#define RT5633_P_SM				(0x1)
#define RT5633_P_SM_BIT				0

/* Power managment addition 3 (0x3c) */
#define RT5633_P_VREF				(0x1 << 15)
#define RT5633_P_VREF_BIT			15
#define RT5633_P_DIS_FAST_VREF			(0x1 << 14)
#define RT5633_P_DIS_FAST_VREF_BIT		14
#define RT5633_P_MAIN_BIAS			(0x1 << 13)
#define RT5633_P_MAIN_BIAS_BIT			13
#define RT5633_P_TP_ADC				(0x1 << 12)
#define RT5633_P_TP_ADC_BIT			12
#define RT5633_P_HP_AMP				(0x1 << 3)
#define RT5633_P_HP_AMP_BIT			3
#define RT5633_P_HP_BUF				(0x1 << 2)
#define RT5633_P_HP_BUF_BIT			2
#define RT5633_P_HP_DIS_DEPOP			(0x1 << 1)
#define RT5633_P_HP_DIS_DEPOP_BIT		1
#define RT5633_P_HP_AMP_DRI			(0x1)
#define RT5633_P_HP_AMP_DRI_BIT			0

/* Power managment addition 4 (0x3e) */
#define RT5633_P_SPK_L_VOL			(0x1 << 15)
#define RT5633_P_SPK_L_VOL_BIT			15
#define RT5633_P_SPK_R_VOL			(0x1 << 14)
#define RT5633_P_SPK_R_VOL_BIT			14
#define RT5633_P_HP_L_VOL			(0x1 << 11)
#define RT5633_P_HP_L_VOL_BIT			11
#define RT5633_P_HP_R_VOL			(0x1 << 10)
#define RT5633_P_HP_R_VOL_BIT			10
#define RT5633_P_AUXOUT_L_VOL			(0x1 << 9)
#define RT5633_P_AUXOUT_L_VOL_BIT		9
#define RT5633_P_AUXOUT_R_VOL			(0x1 << 8)
#define RT5633_P_AUXOUT_R_VOL_BIT		8
#define RT5633_P_LDO				(0x1 << 7)
#define RT5633_P_LDO_BIT			7

/* General Purpose Control Register (0x40) */
#define RT5633_SPK_AMP_AUTO_RATIO_EN		(0x1 << 15)
#define RT5633_SPK_AMP_RATIO_CTRL_MASK		(0x7 << 12)
#define RT5633_SPK_AMP_RATIO_CTRL_2_34		(0x7 << 12)
#define RT5633_SPK_AMP_RATIO_CTRL_2_00		(0x6 << 12)
#define RT5633_SPK_AMP_RATIO_CTRL_1_68		(0x5 << 12)
#define RT5633_SPK_AMP_RATIO_CTRL_1_56		(0x4 << 12)
#define RT5633_SPK_AMP_RATIO_CTRL_1_44		(0x3 << 12)
#define RT5633_SPK_AMP_RATIO_CTRL_1_27		(0x2 << 12)
#define RT5633_SPK_AMP_RATIO_CTRL_1_10		(0x1 << 12)
#define RT5633_SPK_AMP_RATIO_CTRL_1_00		(0x0 << 12)
#define RT5633_STEREO_DAC_HI_PASS_FILT_EN	(0x1 << 11)
#define RT5633_STEREO_ADC_HI_PASS_FILT_EN	(0x1 << 10)
#define RT5633_ADC_WIND_FILT_COFF_MASK		(0x3f << 2)
#define RT5633_ADC_WIND_FILT_CTRL_MASK		(0x3)
#define RT5633_ADC_WIND_FILT_CTRL_DISABLE	(0x0)
#define RT5633_ADC_WIND_FILT_CTRL_1S_HPF	(0x1)
#define RT5633_ADC_WIND_FILT_CTRL_2S_HPF	(0x2)

/* Global Clock Control Register (0x42) */
#define RT5633_SCLK_SRC_MASK			(0x1 << 14)
#define RT5633_SCLK_SRC_MCLK			(0x0 << 14)
#define RT5633_SCLK_SRC_PLL			(0x1 << 14)
#define RT5633_SCLK_SRC_PLL_TCK			(0x2 << 14)
#define RT5633_PLL_SRC_MASK			(0x1 << 13)
#define RT5633_PLL_SRC_MCLK			(0x0 << 13)
#define RT5633_PLL_SRC_BCLK			(0x1 << 13)
#define RT5633_PLL_PRE_DIV_MASK			(0x1 << 11)
#define RT5633_PLL_PRE_DIV1			(0x0 << 11)
#define RT5633_PLL_PRE_DIV2			(0x1 << 11)

/* Video Buffer Control (0x46) */
#define RT5633_VIDEO_BUF_ENA			(0x1 << 15)
#define RT5633_VIDEO_BUF_CLAMP_CTRL		(0x1 << 14)
#define RT5633_VIDEO_BUF_FAST_CLAMP_CTRL	(0x1 << 13)
#define RT5633_VIDEO_CLAMP_REF_SEL_MASK		(0x7 << 10)
#define RT5633_VIDEO_CLAMP_REF_SEL_0V		(0x0 << 10)
#define RT5633_VIDEO_CLAMP_REF_SEL_20MV		(0x1 << 10)
#define RT5633_VIDEO_CLAMP_REF_SEL_40MV		(0x2 << 10)
#define RT5633_VIDEO_CLAMP_REF_SEL_60MV		(0x3 << 10)
#define RT5633_VIDEO_CLAMP_REF_SEL_80MV		(0x4 << 10)
#define RT5633_VIDEO_CLAMP_REF_SEL_100MV	(0x5 << 10)
#define RT5633_VIDEO_CLAMP_REF_SEL_120MV	(0x6 << 10)
#define RT5633_VIDEO_CLAMP_REF_SEL_140MV	(0x7 << 10)
#define RT5633_VIDEO_PULL_DOWN_RES_MASK		(0x7)
#define RT5633_VIDEO_PULL_DOWN_RES_1_6K		(0x0)
#define RT5633_VIDEO_PULL_DOWN_RES_9K		(0x1)
#define RT5633_VIDEO_PULL_DOWN_RES_150K		(0x2)
#define RT5633_VIDEO_PULL_DOWN_RES_440K		(0x3)
#define RT5633_VIDEO_PULL_DOWN_RES_730K		(0x4)
#define RT5633_VIDEO_PULL_DOWN_RES_1_46M	(0x5)
#define RT5633_VIDEO_PULL_DOWN_RES_3M		(0x6)
#define RT5633_VIDEO_PULL_DOWN_RES_4_3M		(0x7)

/* GPIO Control 1 (0x4c) */
#define RT5633_GPIO1_PIN_FUN_SEL_MASK		(0x3 << 1)
#define RT5633_GPIO1_PIN_FUN_SEL_GPIO1		(0x0 << 1)
#define RT5633_GPIO1_PIN_FUN_SEL_DIMC		(0x1 << 1)
#define RT5633_GPIO1_PIN_FUN_SEL_IRQ		(0x2 << 1)

/* GPIO Control 2 (0x4d) */
#define RT5633_GPIO_PIN_CONF_MASK		(0x1 << 11)
#define RT5633_GPIO_PIN_CONF_INPUT		(0x0 << 11)
#define RT5633_GPIO_PIN_CONF_OUTPUT		(0x1 << 11)
#define RT5633_GPIO_OUTPUT_DRI_MASK		(0x1 << 10)
#define RT5633_GPIO_OUTPUT_DRI_LOW		(0x0 << 10)
#define RT5633_GPIO_OUTPUT_DRI_HIGH		(0x1 << 10)

/* DEPOP MODE CONTROL 1 (0x54) */
#define RT5633_PW_SOFT_GEN			(0x1 << 15)
#define RT5633_EN_SOFT_FOR_S_M_DEPOP		(0x1 << 14)
#define RT5633_EN_DEPOP_2			(0x1 << 7)
#define RT5633_EN_DEPOP_1			(0x1 << 6)
#define RT5633_EN_ONE_BIT_HP_DEPOP		(0x1 << 3)
#define RT5633_EN_HP_L_M_UM_DEPOP		(0x1 << 1)
#define RT5633_EN_HP_R_M_UM_DEPOP 		(0x1)

/* Jack Detect Control Register (0x5a) */
#define RT5633_JD_USE_MASK			(0x7 << 13)
#define RT5633_JD_USE_JD2			(0x5 << 13)
#define RT5633_JD_USE_JD1			(0x4 << 13)
#define RT5633_JD_OFF				(0x0 << 13)
#define RT5633_JD_HP_EN				(0x1 << 11)
#define RT5633_JD_HP_TRI_MASK			(0x1 << 10)
#define RT5633_JD_HP_TRI_HI			(0x1 << 10)
#define RT5633_JD_HP_TRI_LO			(0x0 << 10)
#define RT5633_JD_SPK_ENA_MASK			(0x3 << 8)
#define RT5633_JD_SPK_DIS			(0x0 << 8)
#define RT5633_JD_SPK_ENA_SPK_LP_LN		(0x1 << 8)
#define RT5633_JD_SPK_ENA_SPK_LP_RP		(0x2 << 8)
#define RT5633_JD_SPK_TRI_MASK			(0x1 << 7)
#define RT5633_JD_SPK_TRI_HI			(0x1 << 7)
#define RT5633_JD_SPK_TRI_LO			(0x0 << 7)
#define RT5633_JD_LINE1_L_SHARE_JD1		(0x1 << 3)
#define RT5633_JD_LINE1_R_SHARE_JD2		(0x1 << 2)
#define RT5633_JD_AUXOUT_EN			(0x1 << 1)
#define RT5633_JD_AUXOUT_MASK			(0x1)
#define RT5633_JD_AUXOUT_TRI_HI			(0x1)
#define RT5633_JD_AUXOUT_TRI_LO			(0x0)

/* ALC CONTROL 1 (0x64) */
#define RT5633_ALC_ATTACK_RATE_MASK		(0x1f << 8)
#define RT5633_ALC_RECOVERY_RATE_MASK		(0x1f)

/* ALC CONTROL 2 (0x65) */
#define RT5633_ALC_COM_NOISE_GATE_MASK		(0xf)

/* ALC CONTROL 3 (0x66) */
#define RT5633_ALC_FUN_MASK			(0x3 << 14)
#define RT5633_ALC_FUN_DIS			(0x0 << 14)
#define RT5633_ALC_ENA_DAC_PATH			(0x1 << 14)
#define RT5633_ALC_ENA_ADC_PATH			(0x3 << 14)
#define RT5633_ALC_PARA_UPDATE			(0x1 << 13)
#define RT5633_ALC_LIMIT_LEVEL_MASK		(0x1f << 8)
#define RT5633_ALC_NOISE_GATE_FUN_MASK		(0x1 << 7)
#define RT5633_ALC_NOISE_GATE_FUN_DIS		(0x0 << 7)
#define RT5633_ALC_NOISE_GATE_FUN_ENA		(0x1 << 7)
#define RT5633_ALC_NOISE_GATE_H_D_MASK		(0x1 << 6)
#define RT5633_ALC_NOISE_GATE_H_D_DIS		(0x0 << 6)
#define RT5633_ALC_NOISE_GATE_H_D_ENA		(0x1 << 6)

/* Psedueo Stereo & Spatial Effect Block Control (0x68) */
#define RT5633_SPATIAL_CTRL_EN			(0x1 << 15)
#define RT5633_ALL_PASS_FILTER_EN		(0x1 << 14)
#define RT5633_PSEUDO_STEREO_EN			(0x1 << 13)
#define RT5633_STEREO_EXPENSION_EN		(0x1 << 12)
#define RT5633_GAIN1_3D_PARA_MASK		(0x3 << 10)
#define RT5633_GAIN1_3D_PARA_1_00		(0x0 << 10)
#define RT5633_GAIN1_3D_PARA_1_50		(0x1 << 10)
#define RT5633_GAIN1_3D_PARA_2_00		(0x2 << 10)
#define RT5633_RATIO1_3D_MASK			(0x3 << 8)
#define RT5633_RATIO1_3D_0_0			(0x0 << 8)
#define RT5633_RATIO1_3D_0_66			(0x1 << 8)
#define RT5633_RATIO1_3D_1_0			(0x2 << 8)
#define RT5633_GAIN2_3D_PARA_MASK		(0x3 << 6)
#define RT5633_GAIN2_3D_PARA_1_00		(0x0 << 6)
#define RT5633_GAIN2_3D_PARA_1_50		(0x1 << 6)
#define RT5633_GAIN2_3D_PARA_2_00		(0x2 << 6)
#define RT5633_RATIO2_3D_MASK			(0x3 << 4)
#define RT5633_RATIO2_3D_0_0			(0x0 << 4)
#define RT5633_RATIO2_3D_0_66			(0x1 << 4)
#define RT5633_RATIO2_3D_1_0			(0x2 << 4)
#define RT5633_APF_FUN_SLE_MASK			(0x3)
#define RT5633_APF_FUN_SEL_48K			(0x3)
#define RT5633_APF_FUN_SEL_44_1K		(0x2)
#define RT5633_APF_FUN_SEL_32K			(0x1)
#define RT5633_APF_FUN_DIS			(0x0)

/* EQ CONTROL 1 (0x6e) */
#define RT5633_HW_EQ_PATH_SEL_MASK		(0x1 << 15)
#define RT5633_HW_EQ_PATH_SEL_DAC		(0x0 << 15)
#define RT5633_HW_EQ_PATH_SEL_ADC		(0x1 << 15)
#define RT5633_HW_EQ_UPDATE_CTRL		(0x1 << 13)
#define RT5633_HW_EQ_ZC_UPDATE_PARA		(0x1 << 12)

/* EQ CONTROL 2 (0x70) */
#define RT5633_EN_HW_EQ_HPF2			(0x1 << 7)
#define RT5633_EN_HW_EQ_HPF1			(0x1 << 6)
#define RT5633_EN_HW_EQ_BP4			(0x1 << 4)
#define RT5633_EN_HW_EQ_BP3			(0x1 << 3)
#define RT5633_EN_HW_EQ_BP2			(0x1 << 2)
#define RT5633_EN_HW_EQ_BP1			(0x1 << 1)
#define RT5633_EN_HW_EQ_LPF			(0x1)


enum {
	RT5633_AIF1,
	RT5633_AIF2,
	RT5633_AIFS,
};

#endif /* __RT5633_H__ */
