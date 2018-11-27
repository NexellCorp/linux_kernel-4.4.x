#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/atomic.h>
#include <linux/semaphore.h>

#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <dt-bindings/interrupt-controller/s5p4418-irq.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/kthread.h>

#include "../nx-v4l2.h"

#if defined(CONFIG_ARCH_S5P4418) && defined(CONFIG_SECURE_REG_ACCESS)
#include <linux/soc/nexell/sec_reg.h>

#define USE_PSCI_REG_READ_WRITE
#endif

#define NX_CSI_DEV_NAME		"nx-csi"

static u32 enable_ints = 0;
MODULE_PARM_DESC(enable_ints, "csi interrupts enable");
module_param(enable_ints, uint, 0644);

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
struct task_struct *g_CSIThread;
#endif

/*#define _ENABLE_IRQ_ALL_*/

/**
 * register set
 */

/* CSIS_CTRL */
#define CSIS_CTRL_S_DPDN_SWAP_CLK	31
#define CSIS_CTRL_S_DPDN_SWAP_DAT	30
#define CSIS_CTRL_DECOMP_FORM		26
#define CSIS_CTRL_DECOMP_PREDICT	25
#define CSIS_CTRL_DECOMP_EN		24
#define CSIS_CTRL_INTERLEAVE_MODE	22
#define CSIS_CTRL_DOUBLE_CMPNT		21
#define CSIS_CTRL_PARALLEL		20
#define CSIS_CTRL_UPDATE_SHADOW		16
#define CSIS_CTRL_RGB_SWAP		12
#define CSIS_CTRL_RGB_SWAP_CH3		15
#define CSIS_CTRL_RGB_SWAP_CH2		14
#define CSIS_CTRL_RGB_SWAP_CH1		13
#define CSIS_CTRL_RGB_SWAP_CH0		12
#define CSIS_CTRL_WCLK_SRC		8
#define CSIS_CTRL_WCLK_SRC_CH3		11
#define CSIS_CTRL_WCLK_SRC_CH2		10
#define CSIS_CTRL_WCLK_SRC_CH1		9
#define CSIS_CTRL_WCLK_SRC_CH0		8
#define CSIS_CTRL_SW_RST		4
#define CSIS_CTRL_NUMOFDATALANE		2
#define CSIS_CTRL_CSI_EN		0

/* CSIS_DPHYCTRL */
#define CSIS_DPHYCTRL_HSSETTLE		24
#define CSIS_DPHYCTRL_S_CLKSETTLECT	22
#define CSIS_DPHYCTRL_DPHY_ON_DATALANE3	4
#define CSIS_DPHYCTRL_DPHY_ON_DATALANE2	3
#define CSIS_DPHYCTRL_DPHY_ON_DATALANE1	2
#define CSIS_DPHYCTRL_DPHY_ON_DATALANE0	1
#define CSIS_DPHYCTRL_DPHY_ON_CLOCKLANE	0
#define CSIS_DPHYCTRL_DPHY_ON		0

/* CSIS_CONFIG_CH0 */
#define CSIS_CONFIG_CH0_HSYNC_LINTV_CH0	26
#define CSIS_CONFIG_CH0_VSYNC_SINTV_CH0	20
#define CSIS_CONFIG_CH0_VSYNC_EINTV_CH0	8
#define CSIS_CONFIG_CH0_DATAFORMAT_CH0	2
#define CSIS_CONFIG_CH0_VIRTUAL_CHANNEL_CH0	0

/* CSIS_DPHYSTS */
#define CSIS_DPHYSTS_ULPSDAT		8
#define CSIS_DPHYSTS_STOPSTATEDAT	4
#define CSIS_DPHYSTS_ULPSCLK		1
#define CSIS_DPHYSTS_STOPSTATECLK	0

/* CSIS_CTRL2 */
#define CSIS_CTRL2_DCOMP_PREDICT_CH3	31
#define CSIS_CTRL2_DCOMP_PREDICT_CH2	30
#define CSIS_CTRL2_DCOMP_PREDICT_CH1	29
#define CSIS_CTRL2_DOUBLE_CMPNT_CH3	27
#define CSIS_CTRL2_DOUBLE_CMPNT_CH2	26
#define CSIS_CTRL2_DOUBLE_CMPNT_CH1	25
#define CSIS_CTRL2_DOUBLE_CMPNT_CH0	24
#define CSIS_CTRL2_PARALLEL_CH3		23
#define CSIS_CTRL2_PARALLEL_CH2		22
#define CSIS_CTRL2_PARALLEL_CH1		21

/* CSIS_RESOL_CH0 */
#define CSIS_RESOL_CH0_HRESOL_CH0	16
#define CSIS_RESOL_CH0_VRESOL_CH0	0

/* CSIS_INT */
#define CSIS_INTMSK_EVENBEFORE		31
#define CSIS_INTMSK_EVENAFTER		30
#define CSIS_INTMSK_ODDBEFORE		29
#define CSIS_INTMSK_ODDAFTER		28
#define CSIS_INTMSK_FRAMESTART_CH3	27
#define CSIS_INTMSK_FRAMESTART_CH2	26
#define CSIS_INTMSK_FRAMESTART_CH1	25
#define CSIS_INTMSK_FRAMESTART_CH0	24
#define CSIS_INTMSK_FRAMEEND_CH3	23
#define CSIS_INTMSK_FRAMEEND_CH2	22
#define CSIS_INTMSK_FRAMEEND_CH1	21
#define CSIS_INTMSK_FRAMEEND_CH0	20
#define CSIS_INTMSK_ERR_SOT_HS		16
#define CSIS_INTMSK_ERR_LOST_FS_CH3	15
#define CSIS_INTMSK_ERR_LOST_FS_CH2	14
#define CSIS_INTMSK_ERR_LOST_FS_CH1	13
#define CSIS_INTMSK_ERR_LOST_FS_CH0	12
#define CSIS_INTMSK_ERR_LOST_FE_CH3	11
#define CSIS_INTMSK_ERR_LOST_FE_CH2	10
#define CSIS_INTMSK_ERR_LOST_FE_CH1	9
#define CSIS_INTMSK_ERR_LOST_FE_CH0	8
#define CSIS_INTMSK_ERR_OVER_CH3	7
#define CSIS_INTMSK_ERR_OVER_CH2	6
#define CSIS_INTMSK_ERR_OVER_CH1	5
#define CSIS_INTMSK_ERR_OVER_CH0	4
#define CSIS_INTMSK_RESERVED		3
#define CSIS_INTMSK_ERR_ECC		2
#define CSIS_INTMSK_ERR_CRC		1
#define CSIS_INTMSK_ERR_ID		0
#define CSIS_INTMSK_EN_ALL		(~(1 << CSIS_INTMSK_RESERVED))
#define CSIS_INTMSK_EN_ERR_CH1		((1 << CSIS_INTMSK_ERR_SOT_HS) | \
					(1 << CSIS_INTMSK_ERR_LOST_FS_CH1) | \
					(1 << CSIS_INTMSK_ERR_LOST_FE_CH1) | \
					(1 << CSIS_INTMSK_ERR_OVER_CH1) | \
					(1 << CSIS_INTMSK_ERR_ECC) | \
					(1 << CSIS_INTMSK_ERR_CRC) | \
					(1 << CSIS_INTMSK_ERR_ID))
/* SDW_CONFIG_CH0 */
#define SDW_CONFIG_CH0_SDW_HSYNC_LINTV_CH0	26
#define SDW_CONFIG_CH0_SDW_VSYNC_SINTV_CH0	20
#define SDW_CONFIG_CH0_SDW_VSYNC_EINTV_CH0	8
#define SDW_CONFIG_CH0_SDW_DATAFORMAT_CH0	2
#define SDW_CONFIG_CH0_SDW_VIRTUAL_CHANNEL_CH0	0

/* SDW_RESOL_CH0 */
#define SDW_RESOL_CH0_SDW_HRESOL_CH0	16
#define SDW_RESOL_CH0_SDW_VRESOL_CH0	0

/* DSIM_PLLCTRL */
#define DSIM_PLLCTRL_BANDCTRL	24
#define DSIM_PLLCTRL_ENABLE	23
#define DSIM_PLLCTRL_PLLPMS	1

/**
 * pre defined values
 */
#define PLL_STABLE_TIME_VAL	0xffffffff
#define PMS_FOR_750MHz		0x43e8
#define PMS_FOR_1000MHz		0x33e8
#define BANDCTL_FOR_750MHz	0xc
#define BANDCTL_FOR_1000MHz	0xf

enum {
	NX_CSI_PAD_SINK = 0,
	NX_CSI_PAD_SOURCE,
	NX_CSI_PAD_MAX
};

enum {
	STATE_IDLE = 0,
	STATE_RUNNING = (1 << 0),
};

enum {
	HSSETTLE_VGA = 0,
	HSSETTLE_HD,
	HSSETTLE_FHD,
	HSSETTLE_5M,
	HSSETTLE_MAX,
};

struct nx_csi {
	u32 module;
	void *base;
	atomic_t state;
	struct semaphore s_stream_sem;

	u32 width;
	u32 height;

	u32 data_lane;
	u32 swap_clocklane;
	u32 swap_datalane;
	u32 pllval;
	u32 hssettle[HSSETTLE_MAX];

	int irq;
	struct reset_control *rst_mipi;
	struct reset_control *rst_csi;
	struct reset_control *rst_phy_s;
	struct clk *clk;

	struct media_pad pads[NX_CSI_PAD_MAX];
	struct v4l2_subdev subdev;
	struct device *dev;
};


/**
 * primitives
 */
struct nx_mipi_register_set {
	u32 csis_control;
	u32 csis_dphyctrl;
	u32 csis_config_ch0;
	u32 csis_dphysts;
	u32 csis_intmsk;
	u32 csis_intsrc;
	u32 csis_ctrl2;
	u32 csis_version;
	u32 csis_dphyctrl_0;
	u32 csis_dphyctrl_1;
	u32 __reserved0;
	u32 csis_resol_ch0;
	u32 __reserved1;
	u32 __reserved2;
	u32 sdw_config_ch0;
	u32 sdw_resol_ch0;
	u32 csis_config_ch1;
	u32 csis_resol_ch1;
	u32 sdw_config_ch1;
	u32 sdw_resol_ch1;
	u32 csis_config_ch2;
	u32 csis_resol_ch2;
	u32 sdw_config_ch2;
	u32 sdw_resol_ch2;
	u32 csis_config_ch3;
	u32 csis_resol_ch3;
	u32 sdw_config_ch3;
	u32 sdw_resol_3;
	u32 __reserved3[(16 + 128) / 4];

	u32 dsim_status;
	u32 dsim_swrst;
	u32 dsim_clkctrl;
	u32 dsim_timeout;
	u32 dsim_config;
	u32 dsim_escmode;
	u32 dsim_mdresol;
	u32 dsim_mvporch;
	u32 dsim_mhporch;
	u32 dsim_msync;
	u32 dsim_sdresol;
	u32 dsim_intsrc;
	u32 dsim_intmsk;
	u32 dsim_pkthdr;
	u32 dsim_payload;
	u32 dsim_rxfifo;
	u32 dsim_fifothld;
	u32 dsim_fifoctrl;
	u32 dsim_memacchr;
	u32 dsim_pllctrl;
	u32 dsim_plltmr;
	u32 dsim_phyacchr;
	u32 dsim_phyacchr1;

	u32 __reserved4[(0x2000 - 0x015C) / 4];

	u32 mipi_csis_pktdata[0x2000 / 4];
};

enum {
	nx_mipi_csi_format_yuv420_8 = 0x18,
	nx_mipi_csi_format_yuv420_10 = 0x19,
	nx_mipi_csi_format_yuv420_8l = 0x1A,
	nx_mipi_csi_format_yuv420_8c = 0x1C,
	nx_mipi_csi_format_yuv420_10c = 0x1D,
	nx_mipi_csi_format_yuv422_8 = 0x1E,
	nx_mipi_csi_format_yuv422_10 = 0x1F,
	nx_mipi_csi_format_rgb565 = 0x22,
	nx_mipi_csi_format_rgb666 = 0x23,
	nx_mipi_csi_format_rgb888 = 0x24,
	nx_mipi_csi_format_raw6 = 0x28,
	nx_mipi_csi_format_raw7 = 0x29,
	nx_mipi_csi_format_raw8 = 0x2A,
	nx_mipi_csi_format_raw10 = 0x2B,
	nx_mipi_csi_format_raw12 = 0x2C,
	nx_mipi_csi_format_raw14 = 0x2D,
	nx_mipi_csi_format_user0 = 0x30,
	nx_mipi_csi_format_user1 = 0x31,
	nx_mipi_csi_format_user2 = 0x32,
	nx_mipi_csi_format_user3 = 0x33,
};

enum {
	nx_mipi_csi_interleave_ch0 = 0,
	nx_mipi_csi_interleave_dt = 1,
	nx_mipi_csi_interleave_vc = 2,
	nx_mipi_csi_interleave_vcdt = 3
};

enum {
	nx_mipi_csi_prediction_simple = 0,
	nx_mipi_csi_prediction_normal = 1
};

enum {
	nx_mipi_csi_yuv422layout_half = 0,
	nx_mipi_csi_yuv422layout_full = 1
};

enum {
	nx_mipi_csi_rgblayout_rgb = 0,
	nx_mipi_csi_rgblayout_bgr = 1
};

enum {
	nx_mipi_csi_vclksrc_pclk = 0,
	nx_mipi_csi_vclksrc_extclk = 1
};

static struct nx_mipi_register_set *__g_pregister[1];

#ifdef USE_PSCI_REG_READ_WRITE
#define MIPI_CSI_PHYSICAL_BASE	0xC00D0000

#define write_reg_wrapper(val, reg)	do { \
	void *phys = (void *)(MIPI_CSI_PHYSICAL_BASE + ((char *)reg - (char *)__g_pregister[0]));	\
	write_sec_reg_by_id(phys, val, NEXELL_MIPI_SEC_ID); \
	} while (0)

#define read_reg_wrapper(pval, reg)	do { \
	void *phys = (void *)(MIPI_CSI_PHYSICAL_BASE + ((char *)reg - (char *)__g_pregister[0]));	\
	*pval = read_sec_reg_by_id(phys, NEXELL_MIPI_SEC_ID); \
	} while (0)

#define writereg(regname, mask, value)	\
	do {	\
		u32 regvalue; \
		read_reg_wrapper(&regvalue, &pregister->regname); \
		regvalue = (regvalue & (~(mask))) | (value);	\
		write_reg_wrapper(regvalue, &pregister->regname);	\
	} while (0)
#else
#define write_reg_wrapper(val, reg)	writel(val, reg)

#define read_reg_wrapper(pval, reg)	do { \
	*pval = readl(reg); \
	} while (0)

#define writereg(regname, mask, value)	\
	do {	\
		register u32 regvalue = pregister->regname;	\
		regvalue = (regvalue & (~(mask))) | (value);	\
		writel(regvalue, &pregister->regname);	\
	} while (0)
#endif

static int nx_csi_clear_pending_interrupts_all(struct nx_csi *me);
static int nx_csi_enable_interrupts_all(struct nx_csi *me, int enable);
static int nx_csi_enable_interrupts(struct nx_csi *me, u32 val);

static void nx_mipi_set_base_address(u32 module_index, void *base_address)
{
	__g_pregister[module_index] =
		(struct nx_mipi_register_set *)base_address;
}
static int nx_mipi_find_hssettle(struct nx_csi *me)
{
	int ret;

	if (me->width <= 640 && me->height <= 480)
		ret = me->hssettle[HSSETTLE_VGA];
	else if (me->width <= 1280 && me->height <= 720)
		ret = me->hssettle[HSSETTLE_HD];
	else if (me->width <= 1920 && me->height <= 1080)
		ret = me->hssettle[HSSETTLE_FHD];
	else
		ret = me->hssettle[HSSETTLE_5M];

	dev_info(me->dev, "hssettle found(%dx%d) = %d\n",
			me->width, me->height, ret);

	return ret;
}

static int nx_mipi_open_module(struct nx_csi *me)
{
	register struct nx_mipi_register_set *pregister;
	u32 module_index;

	module_index = me->module;

	pregister = __g_pregister[module_index];

	write_reg_wrapper(0, &pregister->csis_dphyctrl_1);
	write_reg_wrapper(nx_mipi_find_hssettle(me) << CSIS_DPHYCTRL_HSSETTLE,
			  &pregister->csis_dphyctrl);

	return true;
}


static void nx_mipi_csi_set_size(u32 module_index, int channel, u32 width,
				 u32 height)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];

	switch (channel) {
	case 0:
		write_reg_wrapper((width << CSIS_RESOL_CH0_HRESOL_CH0) |
				  (height << CSIS_RESOL_CH0_VRESOL_CH0),
				  &pregister->csis_resol_ch0);
		break;
	case 1:
		write_reg_wrapper((width << CSIS_RESOL_CH0_HRESOL_CH0) |
				  (height << CSIS_RESOL_CH0_VRESOL_CH0),
				  &pregister->csis_resol_ch1);
		break;
	case 2:
		write_reg_wrapper((width << CSIS_RESOL_CH0_HRESOL_CH0) |
				  (height << CSIS_RESOL_CH0_VRESOL_CH0),
				  &pregister->csis_resol_ch2);
		break;
	case 3:
		write_reg_wrapper((width << CSIS_RESOL_CH0_HRESOL_CH0) |
				  (height << CSIS_RESOL_CH0_VRESOL_CH0),
				  &pregister->csis_resol_ch3);
		break;
	default:
		break;
	}
}

static void nx_mipi_csi_set_format(u32 module_index, int channel,
				   u32 format)
{
	register struct nx_mipi_register_set *pregister;
	const u32 regmask = 0xFC;
	u32 newvalue = format << CSIS_CONFIG_CH0_DATAFORMAT_CH0;

	pregister = __g_pregister[module_index];

	switch (channel) {
	case 0:
		writereg(csis_config_ch0, regmask, newvalue);
		break;
	case 1:
		writereg(csis_config_ch1, regmask, newvalue);
		break;
	case 2:
		writereg(csis_config_ch2, regmask, newvalue);
		break;
	case 3:
		writereg(csis_config_ch3, regmask, newvalue);
		break;
	default:
		break;
	}
}

static void nx_mipi_csi_set_interleave_mode(u32 module_index,
					    u32 mode)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];
	writereg(csis_control, (3 << CSIS_CTRL_INTERLEAVE_MODE),
		 (mode << CSIS_CTRL_INTERLEAVE_MODE));
}

static void nx_mipi_csi_set_timing_control(u32 module_index, int channel,
					   int t1, int t2, int t5)
{
	register struct nx_mipi_register_set *pregister;
	const u32 regmask = 0xFFFFFF00;
	u32 newvalue = ((t2 - 2) << CSIS_CONFIG_CH0_HSYNC_LINTV_CH0) |
		((t1 - 1) << CSIS_CONFIG_CH0_VSYNC_SINTV_CH0) |
		((t5) << CSIS_CONFIG_CH0_VSYNC_EINTV_CH0);

	pregister = __g_pregister[module_index];

	switch (channel) {
	case 0:
		writereg(csis_config_ch0, regmask, newvalue);
		break;
	case 1:
		writereg(csis_config_ch1, regmask, newvalue);
		break;
	case 2:
		writereg(csis_config_ch2, regmask, newvalue);
		break;
	case 3:
		writereg(csis_config_ch3, regmask, newvalue);
		break;
	default:
		break;
	}
}

static void nx_mipi_csi_set_interleave_channel(u32 module_index, int channel,
					       int interleave_channel)
{
	register struct nx_mipi_register_set *pregister;
	const u32 regmask = 0x3;
	u32 newvalue = interleave_channel;

	pregister = __g_pregister[module_index];

	switch (channel) {
	case 0:
		writereg(csis_config_ch0, regmask, newvalue);
		break;
	case 1:
		writereg(csis_config_ch1, regmask, newvalue);
		break;
	case 2:
		writereg(csis_config_ch2, regmask, newvalue);
		break;
	case 3:
		writereg(csis_config_ch3, regmask, newvalue);
		break;
	default:
		break;
	}
}

static void nx_mipi_csi_enable_decompress(u32 module_index, int enable)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];
	writereg(csis_control, 1 << CSIS_CTRL_DECOMP_EN,
		 enable << CSIS_CTRL_DECOMP_EN);
}

__attribute__((unused))
static void nx_mipi_csi_set_prediction(u32 module_index, int channel,
				       u32 prediction)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];

	switch (channel) {
	case 0:
		writereg(csis_control, 1 << CSIS_CTRL_DECOMP_PREDICT,
			 prediction << CSIS_CTRL_DECOMP_PREDICT);
		break;
	case 1:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_DCOMP_PREDICT_CH1,
			 prediction << CSIS_CTRL2_DCOMP_PREDICT_CH1);
		break;
	case 2:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_DCOMP_PREDICT_CH2,
			 prediction << CSIS_CTRL2_DCOMP_PREDICT_CH2);
		break;
	case 3:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_DCOMP_PREDICT_CH3,
			 prediction << CSIS_CTRL2_DCOMP_PREDICT_CH3);
		break;
	default:
		break;
	}
}

static void nx_mipi_csi_set_yuv422_layout(u32 module_index, int channel,
					  u32 layout)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];
	switch (channel) {
	case 0:
		writereg(csis_control, 1 << CSIS_CTRL_DOUBLE_CMPNT,
			 layout << CSIS_CTRL_DOUBLE_CMPNT);
		break;
	case 1:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_DOUBLE_CMPNT_CH1,
			 layout << CSIS_CTRL2_DOUBLE_CMPNT_CH1);
		break;
	case 2:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_DOUBLE_CMPNT_CH2,
			 layout << CSIS_CTRL2_DOUBLE_CMPNT_CH2);
		break;
	case 3:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_DOUBLE_CMPNT_CH3,
			 layout << CSIS_CTRL2_DOUBLE_CMPNT_CH3);
		break;
	default:
		break;
	}
}

static void nx_mipi_csi_set_parallel_data_alignment32(u32 module_index,
						      int channel,
						      int enable_align32)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];

	switch (channel) {
	case 0:
		writereg(csis_control, 1 << CSIS_CTRL_PARALLEL,
			 enable_align32 << CSIS_CTRL_PARALLEL);
		break;
	case 1:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_PARALLEL_CH1,
			 enable_align32 << CSIS_CTRL2_PARALLEL_CH1);
		break;
	case 2:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_PARALLEL_CH2,
			 enable_align32 << CSIS_CTRL2_PARALLEL_CH2);
		break;
	case 3:
		writereg(csis_ctrl2, 1 << CSIS_CTRL2_PARALLEL_CH3,
			 enable_align32 << CSIS_CTRL2_PARALLEL_CH3);
		break;
	default:
		break;
	}
}

__attribute__((unused))
static void nx_mipi_csi_set_rgblayout(u32 module_index, int channel,
				      u32 rgb_layout)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];

	switch (channel) {
	case 0:
		writereg(csis_control, 1 << CSIS_CTRL_RGB_SWAP_CH0,
			 rgb_layout << CSIS_CTRL_RGB_SWAP_CH0);
		break;
	case 1:
		writereg(csis_control, 1 << CSIS_CTRL_RGB_SWAP_CH1,
			 rgb_layout << CSIS_CTRL_RGB_SWAP_CH1);
		break;
	case 2:
		writereg(csis_control, 1 << CSIS_CTRL_RGB_SWAP_CH2,
			 rgb_layout << CSIS_CTRL_RGB_SWAP_CH2);
		break;
	case 3:
		writereg(csis_control, 1 << CSIS_CTRL_RGB_SWAP_CH3,
			 rgb_layout << CSIS_CTRL_RGB_SWAP_CH3);
		break;
	default:
		break;
	}
}

static void nx_mipi_csi_set_vclk(u32 module_index, int channel,
				 u32 clock_source)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];

	switch (channel) {
	case 0:
		writereg(csis_control, 1 << CSIS_CTRL_WCLK_SRC_CH0,
			 clock_source << CSIS_CTRL_WCLK_SRC_CH0);
		break;
	case 1:
		writereg(csis_control, 1 << CSIS_CTRL_WCLK_SRC_CH1,
			 clock_source << CSIS_CTRL_WCLK_SRC_CH1);
		break;
	case 2:
		writereg(csis_control, 1 << CSIS_CTRL_WCLK_SRC_CH2,
			 clock_source << CSIS_CTRL_WCLK_SRC_CH2);
		break;
	case 3:
		writereg(csis_control, 1 << CSIS_CTRL_WCLK_SRC_CH3,
			 clock_source << CSIS_CTRL_WCLK_SRC_CH3);
		break;
	default:
		break;
	}
}

__attribute__((unused))
static void nx_mipi_csi_software_reset(u32 module_index)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];
	writereg(csis_control, (1 << CSIS_CTRL_SW_RST),
		 (1 << CSIS_CTRL_SW_RST));
}

bool nx_mipi_csi_get_enable(u32 module_index)
{
	register struct nx_mipi_register_set *pregister;
	u32 data;

	pregister = __g_pregister[module_index];
	read_reg_wrapper(&data, &pregister->csis_control);
	return (data & (1 << CSIS_CTRL_CSI_EN));
}

void nx_mipi_csi_set_enable(u32 module_index, int enable)
{
	register struct nx_mipi_register_set *pregister;

	pregister = __g_pregister[module_index];

	writereg(csis_control,
		 (1 << CSIS_CTRL_CSI_EN) |
		 (15 << CSIS_CTRL_UPDATE_SHADOW) |
		 (15 << CSIS_CTRL_RGB_SWAP),
		 (enable << CSIS_CTRL_CSI_EN) |
		 (15 << CSIS_CTRL_UPDATE_SHADOW) |
		 (0 << CSIS_CTRL_RGB_SWAP));
}

static void nx_mipi_csi_set_phy(u32 module_index, u32 number_of_data_lanes,
				int enable_clock_lane, int enable_data_lane0,
				int enable_data_lane1, int enable_data_lane2,
				int enable_data_lane3, int swap_clocklane,
				int swap_datalane)
{
	register struct nx_mipi_register_set *pregister;
	u32 newvalue;

	pregister = __g_pregister[module_index];
	newvalue = (enable_data_lane3 << CSIS_DPHYCTRL_DPHY_ON_DATALANE3) |
		(enable_data_lane2 << CSIS_DPHYCTRL_DPHY_ON_DATALANE2) |
		(enable_data_lane1 << CSIS_DPHYCTRL_DPHY_ON_DATALANE1) |
		(enable_data_lane0 << CSIS_DPHYCTRL_DPHY_ON_DATALANE0) |
		(enable_clock_lane << CSIS_DPHYCTRL_DPHY_ON_CLOCKLANE);
	writereg(csis_dphyctrl, (0x1F << CSIS_DPHYCTRL_DPHY_ON), newvalue);
	newvalue = (swap_clocklane << CSIS_CTRL_S_DPDN_SWAP_CLK) |
		(swap_datalane << CSIS_CTRL_S_DPDN_SWAP_DAT) |
		(number_of_data_lanes << CSIS_CTRL_NUMOFDATALANE);
	writereg(csis_control, (0x3 << CSIS_CTRL_S_DPDN_SWAP_DAT) |
		 (3 << CSIS_CTRL_NUMOFDATALANE), newvalue);
}

__attribute__((unused))
static u32 nx_mipi_csi_get_version(u32 module_index)
{
	register struct nx_mipi_register_set *pregister;
	u32 version;

	pregister = __g_pregister[module_index];
	read_reg_wrapper(&version, &pregister->csis_version);
	return version;
}

__attribute__((unused))
static u32 nx_mipi_csi_get_non_image_data(u32 module_index, u32 address32)
{
	register struct nx_mipi_register_set *pregister;
	u32 data;

	pregister = __g_pregister[module_index];
	read_reg_wrapper(&data, &pregister->mipi_csis_pktdata[address32]);
	return data;
}

static void nx_mipi_setpll(u32 module_index, int enable, u32 pllstabletimer,
			   u32 m_pllpms, u32 m_bandctl, u32 m_dphyctl,
			   u32 b_dphyctl)
{
	register struct nx_mipi_register_set *pregister;
	u32 newvalue;

	pregister = __g_pregister[module_index];
	if (!enable) {
		newvalue = (enable << DSIM_PLLCTRL_ENABLE);
		newvalue |= (m_pllpms << DSIM_PLLCTRL_PLLPMS);
		newvalue |= (m_bandctl << DSIM_PLLCTRL_BANDCTRL);
		writereg(dsim_pllctrl, 0x0fffffff, newvalue);
	}
	write_reg_wrapper(m_dphyctl, &pregister->dsim_phyacchr);
	write_reg_wrapper(pllstabletimer, &pregister->dsim_plltmr);
	write_reg_wrapper(b_dphyctl << 9, &pregister->dsim_phyacchr1);
	if (enable) {
		newvalue = (enable << DSIM_PLLCTRL_ENABLE);
		newvalue |= (m_pllpms << DSIM_PLLCTRL_PLLPMS);
		newvalue |= (m_bandctl << DSIM_PLLCTRL_BANDCTRL);
		writereg(dsim_pllctrl, 0x0fffffff, newvalue);
	}
}

static bool nx_mipi_is_pll_enable(u32 module_index)
{
	register struct nx_mipi_register_set *pregister;
	u32 regvalue;

	pregister = __g_pregister[module_index];
	read_reg_wrapper(&regvalue, &pregister->dsim_pllctrl);

	return regvalue & (1 << DSIM_PLLCTRL_ENABLE) ? true : false;
}

static void nx_csi_run(struct nx_csi *me)
{
	u32 module = me->module;
	u32 pms;
	u32 bandctl;

	clk_prepare_enable(me->clk);
	reset_control_assert(me->rst_mipi);
	reset_control_assert(me->rst_csi);
	reset_control_assert(me->rst_phy_s);
	reset_control_deassert(me->rst_mipi);
	reset_control_deassert(me->rst_csi);

	nx_mipi_open_module(me);

	/* disable interrupts */
	nx_csi_enable_interrupts_all(me, false);
	/* clear pending interrupts */
	nx_csi_clear_pending_interrupts_all(me);

	clk_disable_unprepare(me->clk);
	clk_set_rate(me->clk, 300000000);
	clk_prepare_enable(me->clk);

	nx_mipi_csi_set_parallel_data_alignment32(module, 1, 0);
	nx_mipi_csi_set_yuv422_layout(module, 1, nx_mipi_csi_yuv422layout_full);
	nx_mipi_csi_set_format(module, 1, nx_mipi_csi_format_yuv422_8);
	nx_mipi_csi_enable_decompress(module, 0);
	nx_mipi_csi_set_interleave_mode(module, nx_mipi_csi_interleave_vc);
	nx_mipi_csi_set_timing_control(module, 1, 32, 16, 368);
	nx_mipi_csi_set_interleave_channel(module, 0, 1);
	nx_mipi_csi_set_interleave_channel(module, 1, 0);
	nx_mipi_csi_set_size(module, 1, me->width, me->height);
	nx_mipi_csi_set_vclk(module, 1, nx_mipi_csi_vclksrc_extclk);


	switch (me->data_lane) {
	case 1:
		nx_mipi_csi_set_phy(module,
				    me->data_lane - 1,
				    1, /* enable clocklane */
				    1, /* enable datalane0 */
				    0, /* enable datalane1 */
				    0, /* enable datalane2 */
				    0, /* enable datalane3 */
				    me->swap_clocklane,
				    me->swap_datalane);
		break;
	case 2:
		nx_mipi_csi_set_phy(module,
				    me->data_lane - 1,
				    1, /* enable clocklane */
				    1, /* enable datalane0 */
				    1, /* enable datalane1 */
				    0, /* enable datalane2 */
				    0, /* enable datalane3 */
				    me->swap_clocklane,
				    me->swap_datalane);
		break;
	case 3:
		nx_mipi_csi_set_phy(module,
				    me->data_lane - 1,
				    1, /* enable clocklane */
				    1, /* enable datalane0 */
				    1, /* enable datalane1 */
				    1, /* enable datalane2 */
				    0, /* enable datalane3 */
				    me->swap_clocklane,
				    me->swap_datalane);
		break;
	case 4:
		nx_mipi_csi_set_phy(module,
				    me->data_lane - 1,
				    1, /* enable clocklane */
				    1, /* enable datalane0 */
				    1, /* enable datalane1 */
				    1, /* enable datalane2 */
				    1, /* enable datalane3 */
				    me->swap_clocklane,
				    me->swap_datalane);
		break;
	default:
		BUG();
	}
#ifdef _ENABLE_IRQ_ALL_
	nx_csi_enable_interrupts_all(me, true);
#else
	if (enable_ints)
		nx_csi_enable_interrupts(me, enable_ints);
	else
		nx_csi_enable_interrupts(me, CSIS_INTMSK_EN_ERR_CH1);
#endif
	nx_mipi_csi_set_enable(module, 1);

	reset_control_deassert(me->rst_phy_s);

	if (!nx_mipi_is_pll_enable(module)) {
		if (me->pllval == 750) {
			pms = PMS_FOR_750MHz;
			bandctl = BANDCTL_FOR_750MHz;
		} else if (me->pllval == 1000) {
			pms = PMS_FOR_1000MHz;
			bandctl = BANDCTL_FOR_1000MHz;
		} else {
			dev_err(me->dev, "unknown pllval %d --> set default\n",
				me->pllval);
			pms = PMS_FOR_750MHz;
			bandctl = BANDCTL_FOR_750MHz;
		}
		nx_mipi_setpll(module, 1, PLL_STABLE_TIME_VAL, pms, bandctl, 0, 0);
	}
}

static void nx_csi_stop(struct nx_csi *me)
{
	nx_csi_enable_interrupts_all(me, false);
	nx_mipi_csi_set_enable(me->module, 0);
	clk_disable_unprepare(me->clk);
}

/**
 * v4l2 subdev ops
 */
static struct v4l2_subdev *get_remote_subdev(struct nx_csi *me, int pad_index)
{
	struct media_pad *pad;

	pad = media_entity_remote_pad(&me->pads[pad_index]);
	if (pad)
		return media_entity_to_v4l2_subdev(pad->entity);
	return NULL;
}

static int nx_csi_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct nx_csi *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_subdev(me, NX_CSI_PAD_SINK);

	me->width = format->format.width;
	me->height = format->format.height;

	return v4l2_subdev_call(remote, pad, set_fmt, cfg, format);
}

static int nx_csi_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum
				  *frame)
{
	struct nx_csi *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_subdev(me, NX_CSI_PAD_SINK);

	return v4l2_subdev_call(remote, pad, enum_frame_size, 0, frame);
}

static int nx_csi_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum
				      *frame)
{
	struct nx_csi *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_subdev(me, NX_CSI_PAD_SINK);

	return v4l2_subdev_call(remote, pad, enum_frame_interval, 0, frame);
}

static irqreturn_t nx_csi_irq_handler(int irq, void *dev_id)
{
	struct nx_csi *me = dev_id;
	register struct nx_mipi_register_set *pregister;
	u32 int_status;

	pregister = __g_pregister[me->module];
	read_reg_wrapper(&int_status, &pregister->csis_intsrc);
	dev_dbg(me->dev, "[%s] %x\n", __func__, int_status);

	write_reg_wrapper(int_status, &pregister->csis_intsrc);

	return IRQ_HANDLED;
}

static int register_irq_handler(struct nx_csi *me)
{
	int ret;
	struct device *dev = me->dev;

	ret = devm_request_irq(dev, me->irq, nx_csi_irq_handler,
			IRQF_SHARED, dev_name(dev), me);
	if (ret) {
		pr_err("[%s] failed to request irq\n", __func__);
		return ret;
	}
	return 0;
}

static int nx_csi_clear_pending_interrupts_all(struct nx_csi *me)
{
	register struct nx_mipi_register_set *pregister;
	u32 val = 0;

	pregister = __g_pregister[me->module];
	val = CSIS_INTMSK_EN_ALL;
	write_reg_wrapper(val, &pregister->csis_intmsk);
	return 0;
}

static int nx_csi_enable_interrupts_all(struct nx_csi *me, int enable)
{
	register struct nx_mipi_register_set *pregister;
	u32 val = 0;

	pregister = __g_pregister[me->module];
	if (enable) {
		val = CSIS_INTMSK_EN_ALL;
	}
	write_reg_wrapper(val, &pregister->csis_intmsk);
	return 0;
}

static int nx_csi_enable_interrupts(struct nx_csi *me, u32 val)
{
	register struct nx_mipi_register_set *pregister;
	u32 status;

	pregister = __g_pregister[me->module];
	write_reg_wrapper(0, &pregister->csis_intmsk);
	read_reg_wrapper(&status, &pregister->csis_intmsk);
	write_reg_wrapper(val, &pregister->csis_intmsk);
	read_reg_wrapper(&status, &pregister->csis_intmsk);
	return 0;
}

static int nx_csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct nx_csi *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote_source =
		get_remote_subdev(me, NX_CSI_PAD_SINK);

	if (!remote_source) {
		dev_err(me->dev, "can't s_stream. you must connect source to this device\n");
		return -ENODEV;
	}

	ret = down_interruptible(&me->s_stream_sem);

	if (enable) {
		if (!(NX_ATOMIC_READ(&me->state) & STATE_RUNNING)) {
#ifdef CONFIG_VIDEO_MAX9286
			nx_csi_run(me); 	
			ret = v4l2_subdev_call(remote_source, video, s_stream, 1);
			if (ret) {
				dev_err(me->dev, "failed to s_stream %d\n", enable);
				goto UP_AND_OUT;
			}
#else
			ret = v4l2_subdev_call(remote_source, video, s_stream, 1);
			if (ret) {
				dev_err(me->dev, "failed to s_stream %d\n", enable);
				goto UP_AND_OUT;
			}
			nx_csi_run(me);
#endif
			NX_ATOMIC_SET(&me->state, STATE_RUNNING);
		} else if (!nx_mipi_csi_get_enable(me->module))
			nx_mipi_csi_set_enable(me->module, 1);
	} else {
		if (NX_ATOMIC_READ(&me->state) & STATE_RUNNING) {
			nx_csi_stop(me);
			v4l2_subdev_call(remote_source, video, s_stream, 0);
			NX_ATOMIC_CLEAR_MASK(STATE_RUNNING, &me->state);
		} else if (nx_mipi_csi_get_enable(me->module))
			nx_mipi_csi_set_enable(me->module, 0);
	}
UP_AND_OUT:
	up(&me->s_stream_sem);

	return ret;
}

static int nx_csi_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct nx_csi *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_subdev(me, NX_CSI_PAD_SINK);

	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	return v4l2_subdev_call(remote, video, g_parm, param);
}

static int nx_csi_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct nx_csi *me = v4l2_get_subdevdata(sd);
	struct v4l2_subdev *remote = get_remote_subdev(me, NX_CSI_PAD_SINK);

	if (!remote) {
		WARN_ON(1);
		return -ENODEV;
	}

	return v4l2_subdev_call(remote, video, s_parm, param);
}

static struct v4l2_subdev_video_ops nx_csi_video_ops = {
	.s_stream = nx_csi_s_stream,
	.g_parm = nx_csi_g_parm,
	.s_parm = nx_csi_s_parm,
};

static struct v4l2_subdev_pad_ops nx_csi_pad_ops = {
	.set_fmt = nx_csi_set_fmt,
	.enum_frame_size = nx_csi_enum_frame_size,
	.enum_frame_interval = nx_csi_enum_frame_interval,
};

static struct v4l2_subdev_ops nx_csi_subdev_ops = {
	.video = &nx_csi_video_ops,
	.pad = &nx_csi_pad_ops,
};

/**
 * initialize
 */
static void init_me(struct nx_csi *me)
{
	NX_ATOMIC_SET(&me->state, STATE_IDLE);
	sema_init(&me->s_stream_sem, 1);
}

static int init_v4l2_subdev(struct nx_csi *me)
{
	int ret;
	struct v4l2_subdev *sd = &me->subdev;
	struct media_pad *pads = me->pads;
	struct media_entity *entity = &sd->entity;

	v4l2_subdev_init(sd, &nx_csi_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "%s%d", NX_CSI_DEV_NAME,
		 me->module);
	v4l2_set_subdevdata(sd, me);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	pads[NX_CSI_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[NX_CSI_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(entity, NX_CSI_PAD_MAX, pads, 0);
	if (ret < 0) {
		dev_err(me->dev, "failed to media_entity_init\n");
		return ret;
	}

	return 0;
}

static int nx_csi_parse_dt(struct platform_device *pdev, struct nx_csi *me)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource res;

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get base address\n");
		return -ENXIO;
	}

	me->base = devm_ioremap_nocache(dev, res.start, resource_size(&res));
	if (!me->base) {
		dev_err(dev, "failed to ioremap\n");
		return -EBUSY;
	}

	me->clk = devm_clk_get(dev, "mipi");
	if (IS_ERR(me->clk)) {
		dev_err(dev, "failed to devm_clk_get for mipi\n");
		return -ENODEV;
	}

	me->rst_mipi = devm_reset_control_get(dev, "mipi-reset");
	if (IS_ERR(me->rst_mipi)) {
		dev_err(dev, "failed to get reset control of mipi\n");
		return -ENODEV;
	}

	me->rst_csi = devm_reset_control_get(dev, "mipi_csi-reset");
	if (IS_ERR(me->rst_csi)) {
		dev_err(dev, "failed to get reset control of mipi_csi\n");
		return -ENODEV;
	}

	me->rst_phy_s = devm_reset_control_get(dev, "mipi_phy_s-reset");
	if (IS_ERR(me->rst_phy_s)) {
		dev_err(dev, "failed to get reset control of mipi_phy_s\n");
		return -ENODEV;
	}

	if (of_property_read_u32(np, "data_lane", &me->data_lane)) {
		dev_err(dev, "failed to get dt data_lane\n");
		return -EINVAL;
	}
	if (me->data_lane < 1 || me->data_lane > 4) {
		dev_err(dev, "invalid data_lane %d --> force set 2\n",
			me->data_lane);
		me->data_lane = 2;
	}

	if (of_property_read_u32(np, "swap_clocklane", &me->swap_clocklane)) {
		dev_err(dev, "failed to get dt swap_clocklane\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "swap_datalane", &me->swap_datalane)) {
		dev_err(dev, "failed to get dt swap_datalane\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "pllval", &me->pllval)) {
		dev_err(dev, "failed to get dt pllval\n");
		return -EINVAL;
	}

	if (of_property_read_u32_array(np, "hssettle", me->hssettle,
				HSSETTLE_MAX)) {
		if (of_property_read_u32(np, "hssettle", &me->hssettle[0])) {
			dev_err(dev, "failed to get dt hssettle\n");
			return -EINVAL;
		} else
			me->hssettle[1] = me->hssettle[2] = me->hssettle[3]
				= me->hssettle[0];
	}
	dev_dbg(dev, "hssettle:%d:%d:%d:%d\n", me->hssettle[0], me->hssettle[1],
			me->hssettle[2], me->hssettle[3]);

	return 0;
}

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
static int init_csi_th(void *args)
{
	int ret;
	struct nx_csi *me = args;

	init_me(me);

	ret = init_v4l2_subdev(me);
	if (ret)
		return ret;

	ret = nx_v4l2_register_subdev(&me->subdev);
	if (ret)
		BUG();

	nx_mipi_set_base_address(me->module, me->base);

	return ret;
}
#endif

/**
 * platform driver specific
 */
static int nx_csi_probe(struct platform_device *pdev)
{
	int ret;
	struct nx_csi *me;

	me = devm_kzalloc(&pdev->dev, sizeof(*me), GFP_KERNEL);
	if (!me) {
		WARN_ON(1);
		return -ENOMEM;
	}
	me->module = 0;

#ifndef CONFIG_V4L2_INIT_LEVEL_UP
	init_me(me);

	ret = nx_csi_parse_dt(pdev, me);
	if (ret)
		return ret;

	ret = init_v4l2_subdev(me);
	if (ret)
		return ret;

	ret = nx_v4l2_register_subdev(&me->subdev);
	if (ret)
		BUG();

	nx_mipi_set_base_address(me->module, me->base);
#else
	ret = nx_csi_parse_dt(pdev, me);
	if (ret)
		return ret;

	if (g_CSIThread == NULL)
		g_CSIThread = kthread_run(init_csi_th, me, "KthreadForNxCSI");
#endif

	me->dev = &pdev->dev;
	platform_set_drvdata(pdev, me);

	dev_dbg(me->dev, "[%s] enable_interrupts:%x\n", __func__,
			enable_ints);
	ret = platform_get_irq(pdev, 0);
	if (ret >= 0) {
		me->irq = ret;
		register_irq_handler(me);
	}
	return 0;
}

static int nx_csi_remove(struct platform_device *pdev)
{
	struct nx_csi *me = platform_get_drvdata(pdev);

	devm_free_irq(me->dev, me->irq, me);
	if (me)
		v4l2_device_unregister_subdev(&me->subdev);

	return 0;
}

static struct platform_device_id nx_csi_id_table[] = {
	{ NX_CSI_DEV_NAME, 0},
	{},
};

static const struct of_device_id nx_csi_dt_match[] = {
	{ .compatible = "nexell,mipi_csi" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_csi_dt_match);

static struct platform_driver nx_csi_driver = {
	.probe		= nx_csi_probe,
	.remove		= nx_csi_remove,
	.id_table	= nx_csi_id_table,
	.driver = {
		.name	= NX_CSI_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nx_csi_dt_match),
	},
};

#ifdef CONFIG_V4L2_INIT_LEVEL_UP
static int __init nx_csi_init(void)
{
	return platform_driver_register(&nx_csi_driver);
}

static void __exit nx_csi_exit(void)
{
	platform_driver_unregister(&nx_csi_driver);
}
subsys_initcall(nx_csi_init);
module_exit(nx_csi_exit);
#else
module_platform_driver(nx_csi_driver);
#endif
MODULE_AUTHOR("swpark <swpark@nexell.co.kr>");
MODULE_DESCRIPTION("Nexell S5Pxx18 series SoC mipi-csi device driver");
MODULE_LICENSE("GPL");
