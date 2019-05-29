/*
 * platform.c - DesignWare HS OTG Controller platform driver
 *
 * Copyright (C) Matthijs Kooijman <matthijs@stdin.nl>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_data/s3c-hsotg.h>
#include <linux/reset.h>

#include <linux/usb/of.h>

#include "core.h"
#include "hcd.h"
#include "debug.h"

static const char dwc2_driver_name[] = "dwc2";


static struct kobject *otg_kobj;
struct dwc2_hsotg *g_hsotg;

static int dwc2_usbotg_device_set_test_mode(struct dwc2_hsotg *hsotg, int testmode)
{
	int val = 0;

	switch (testmode) {
	case 0:
	case TEST_J:
	case TEST_K:
	case TEST_SE0_NAK:
	case TEST_PACKET:
	case TEST_FORCE_EN:
		val = testmode << DCTL_TSTCTL_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	dwc2_writel(val, hsotg->regs + DCTL);

	return 0;
}

static ssize_t show_device_test_mode(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	dev_info(dev, "ex) # echo test_packet > /sys/usbotg_test_mode/device \n");
	dev_info(dev, "           test_j \n");
	dev_info(dev, "           test_k \n");
	dev_info(dev, "           test_se0_nak \n");
	dev_info(dev, "           test_packet \n");
	dev_info(dev, "           test_force_enable \n");
	dev_info(dev, "           test_mode_disable \n");

	return 0;
}

static ssize_t store_device_test_mode(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long flags;
	ssize_t ret;
	u32 testmode = 0;

	if (!strncmp(buf, "test_j", 6)) {
		testmode = TEST_J;
	}
	else if (!strncmp(buf, "test_k", 6)) {
		testmode = TEST_K;
	}
	else if (!strncmp(buf, "test_se0_nak", 12)) {
		testmode = TEST_SE0_NAK;
	}
	else if (!strncmp(buf, "test_packet", 11)) {
		testmode = TEST_PACKET;
	}
	else if (!strncmp(buf, "test_force_enable", 17)) {
		testmode = TEST_FORCE_EN;
	}
	else if (!strncmp(buf, "test_mode_disable", 17)) {
		testmode = 0;
	} else {
		ret = count;
		return ret;
	}

	dev_info(dev, " [\e[31m%s\e[0m():%d] otg device testmode:%d, buf:%s\n",
		__func__, __LINE__, testmode, buf);

	spin_lock_irqsave(&g_hsotg->lock, flags);

	dwc2_usbotg_device_set_test_mode(g_hsotg, testmode);

	spin_unlock_irqrestore(&g_hsotg->lock, flags);

	ret = count;

	return ret;
}
static DEVICE_ATTR(device, 0644, show_device_test_mode, store_device_test_mode);

static int dwc2_usbotg_host_set_test_mode(struct dwc2_hsotg *hsotg, int testmode)
{
	int val = 0;

	switch (testmode) {
	case 0:
	case TEST_J:
	case TEST_K:
	case TEST_SE0_NAK:
	case TEST_PACKET:
	case TEST_FORCE_EN:
		val = testmode << HPRT0_TSTCTL_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	dwc2_writel(val, hsotg->regs + HPRT0);

	return 0;
}

static ssize_t show_host_test_mode(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	dev_info(dev, "ex) # echo test_packet > /sys/usbotg_test_mode/host \n");
	dev_info(dev, "           test_j \n");
	dev_info(dev, "           test_k \n");
	dev_info(dev, "           test_se0_nak \n");
	dev_info(dev, "           test_packet \n");
	dev_info(dev, "           test_force_enable \n");
	dev_info(dev, "           test_mode_disable \n");

	return 0;
}

static ssize_t store_host_test_mode(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long flags;
	ssize_t ret;
	u32 testmode = 0;

	if (!strncmp(buf, "test_j", 6)) {
		testmode = TEST_J;
	}
	else if (!strncmp(buf, "test_k", 6)) {
		testmode = TEST_K;
	}
	else if (!strncmp(buf, "test_se0_nak", 12)) {
		testmode = TEST_SE0_NAK;
	}
	else if (!strncmp(buf, "test_packet", 11)) {
		testmode = TEST_PACKET;
	}
	else if (!strncmp(buf, "test_force_enable", 17)) {
		testmode = TEST_FORCE_EN;
	}
	else if (!strncmp(buf, "test_mode_disable", 17)) {
		testmode = 0;
	} else {
		ret = count;
		return ret;
	}

	dev_info(dev, " [\e[31m%s\e[0m():%d] otg host testmode:%d, buf:%s\n",
		__func__, __LINE__, testmode, buf);

	spin_lock_irqsave(&g_hsotg->lock, flags);

	dwc2_usbotg_host_set_test_mode(g_hsotg, testmode);

	spin_unlock_irqrestore(&g_hsotg->lock, flags);

	ret = count;

	return ret;
}
static DEVICE_ATTR(host, 0644, show_host_test_mode, store_host_test_mode);

static int create_test_mode_sysfs_files(struct dwc2_hsotg *hsotg)
{
	int	ret = 0;

	g_hsotg = hsotg;

	otg_kobj = kobject_create_and_add("usbotg_test_mode", NULL);
	if (otg_kobj == NULL) {
		dev_err(hsotg->dev, "otg_kobj:" "kobject_create_and_add failed\n");
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(otg_kobj, &dev_attr_host.attr);
	if (ret) {
		dev_err(hsotg->dev, "otg host:" "sysfs_create_group failed\n");
		kobject_del(otg_kobj);
		return ret;
	}

	ret = sysfs_create_file(otg_kobj, &dev_attr_device.attr);
	if (ret) {
		dev_err(hsotg->dev, "otg device:" "sysfs_create_group failed\n");
		sysfs_remove_file(otg_kobj, &dev_attr_host.attr);
		kobject_del(otg_kobj);
		return ret;
	}
	return ret;
}

static inline void remove_test_mode_sysfs_files(struct dwc2_hsotg *hsotg)
{
	sysfs_remove_file(otg_kobj, &dev_attr_host.attr);
	sysfs_remove_file(otg_kobj, &dev_attr_device.attr);
	kobject_del(otg_kobj);
}

/*
 * Check the dr_mode against the module configuration and hardware
 * capabilities.
 *
 * The hardware, module, and dr_mode, can each be set to host, device,
 * or otg. Check that all these values are compatible and adjust the
 * value of dr_mode if possible.
 *
 *                      actual
 *    HW  MOD dr_mode   dr_mode
 *  ------------------------------
 *   HST  HST  any    :  HST
 *   HST  DEV  any    :  ---
 *   HST  OTG  any    :  HST
 *
 *   DEV  HST  any    :  ---
 *   DEV  DEV  any    :  DEV
 *   DEV  OTG  any    :  DEV
 *
 *   OTG  HST  any    :  HST
 *   OTG  DEV  any    :  DEV
 *   OTG  OTG  any    :  dr_mode
 */
static int dwc2_get_dr_mode(struct dwc2_hsotg *hsotg)
{
	enum usb_dr_mode mode;

	hsotg->dr_mode = usb_get_dr_mode(hsotg->dev);
	if (hsotg->dr_mode == USB_DR_MODE_UNKNOWN)
		hsotg->dr_mode = USB_DR_MODE_OTG;

	mode = hsotg->dr_mode;

	if (dwc2_hw_is_device(hsotg)) {
		if (IS_ENABLED(CONFIG_USB_DWC2_HOST)) {
			dev_err(hsotg->dev,
				"Controller does not support host mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_PERIPHERAL;
	} else if (dwc2_hw_is_host(hsotg)) {
		if (IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL)) {
			dev_err(hsotg->dev,
				"Controller does not support device mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_HOST;
	} else {
		if (IS_ENABLED(CONFIG_USB_DWC2_HOST))
			mode = USB_DR_MODE_HOST;
		else if (IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL))
			mode = USB_DR_MODE_PERIPHERAL;
	}

	if (mode != hsotg->dr_mode) {
		dev_warn(hsotg->dev,
			 "Configuration mismatch. dr_mode forced to %s\n",
			mode == USB_DR_MODE_HOST ? "host" : "device");

		hsotg->dr_mode = mode;
	}

	return 0;
}

static int __dwc2_lowlevel_hw_enable(struct dwc2_hsotg *hsotg)
{
	struct platform_device *pdev = to_platform_device(hsotg->dev);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(hsotg->supplies),
				    hsotg->supplies);
	if (ret)
		return ret;

	if (hsotg->clk) {
		ret = clk_prepare_enable(hsotg->clk);
		if (ret)
			return ret;
	}

	if (of_device_is_compatible(hsotg->dev->of_node,
					    "nexell,nexell-dwc2otg")) {
#ifdef CONFIG_RESET_CONTROLLER
			struct reset_control *rst;

			rst = reset_control_get(hsotg->dev,
						     "usbotg-reset");
			if (!IS_ERR(rst)) {
				if (reset_control_status(rst))
					reset_control_reset(rst);
				reset_control_put(rst);
			}
#endif
	}

	if (hsotg->uphy) {
		ret = usb_phy_init(hsotg->uphy);
	} else if (hsotg->plat && hsotg->plat->phy_init) {
		ret = hsotg->plat->phy_init(pdev, hsotg->plat->phy_type);
	} else {
		ret = phy_power_on(hsotg->phy);
		if (ret == 0)
			ret = phy_init(hsotg->phy);
	}

	return ret;
}

/**
 * dwc2_lowlevel_hw_enable - enable platform lowlevel hw resources
 * @hsotg: The driver state
 *
 * A wrapper for platform code responsible for controlling
 * low-level USB platform resources (phy, clock, regulators)
 */
int dwc2_lowlevel_hw_enable(struct dwc2_hsotg *hsotg)
{
	int ret = __dwc2_lowlevel_hw_enable(hsotg);

	if (ret == 0)
		hsotg->ll_hw_enabled = true;
	return ret;
}

static int __dwc2_lowlevel_hw_disable(struct dwc2_hsotg *hsotg)
{
	struct platform_device *pdev = to_platform_device(hsotg->dev);
	int ret = 0;

	if (hsotg->uphy) {
		usb_phy_shutdown(hsotg->uphy);
	} else if (hsotg->plat && hsotg->plat->phy_exit) {
		ret = hsotg->plat->phy_exit(pdev, hsotg->plat->phy_type);
	} else {
		ret = phy_exit(hsotg->phy);
		if (ret == 0)
			ret = phy_power_off(hsotg->phy);
	}
	if (ret)
		return ret;

	if (hsotg->clk)
		clk_disable_unprepare(hsotg->clk);

	ret = regulator_bulk_disable(ARRAY_SIZE(hsotg->supplies),
				     hsotg->supplies);

	return ret;
}

/**
 * dwc2_lowlevel_hw_disable - disable platform lowlevel hw resources
 * @hsotg: The driver state
 *
 * A wrapper for platform code responsible for controlling
 * low-level USB platform resources (phy, clock, regulators)
 */
int dwc2_lowlevel_hw_disable(struct dwc2_hsotg *hsotg)
{
	int ret = __dwc2_lowlevel_hw_disable(hsotg);

	if (ret == 0)
		hsotg->ll_hw_enabled = false;
	return ret;
}

static int dwc2_lowlevel_hw_init(struct dwc2_hsotg *hsotg)
{
	int i, ret;

	hsotg->reset = devm_reset_control_get_optional(hsotg->dev, "dwc2");
	if (IS_ERR(hsotg->reset)) {
		ret = PTR_ERR(hsotg->reset);
		dev_err(hsotg->dev, "error getting reset control %d\n", ret);
		return ret;
	}

	reset_control_deassert(hsotg->reset);

	/* Set default UTMI width */
	hsotg->phyif = GUSBCFG_PHYIF16;

	/*
	 * Attempt to find a generic PHY, then look for an old style
	 * USB PHY and then fall back to pdata
	 */
	hsotg->phy = devm_phy_get(hsotg->dev, "usb2-phy");
	if (IS_ERR(hsotg->phy)) {
		ret = PTR_ERR(hsotg->phy);
		switch (ret) {
		case -ENODEV:
		case -ENOSYS:
			hsotg->phy = NULL;
			break;
		case -EPROBE_DEFER:
			return ret;
		default:
			dev_err(hsotg->dev, "error getting phy %d\n", ret);
			return ret;
		}
	}

	if (!hsotg->phy) {
		hsotg->uphy = devm_usb_get_phy(hsotg->dev, USB_PHY_TYPE_USB2);
		if (IS_ERR(hsotg->uphy)) {
			ret = PTR_ERR(hsotg->uphy);
			switch (ret) {
			case -ENODEV:
			case -ENXIO:
				hsotg->uphy = NULL;
				break;
			case -EPROBE_DEFER:
				return ret;
			default:
				dev_err(hsotg->dev, "error getting usb phy %d\n",
					ret);
				return ret;
			}
		}
	}

	hsotg->plat = dev_get_platdata(hsotg->dev);

	if (hsotg->phy) {
		/*
		 * If using the generic PHY framework, check if the PHY bus
		 * width is 8-bit and set the phyif appropriately.
		 */
		if (phy_get_bus_width(hsotg->phy) == 8)
			hsotg->phyif = GUSBCFG_PHYIF8;
	}

	/* Clock */
	hsotg->clk = devm_clk_get(hsotg->dev, "otg");
	if (IS_ERR(hsotg->clk)) {
		hsotg->clk = NULL;
		dev_dbg(hsotg->dev, "cannot get otg clock\n");
	}

	/* Regulators */
	for (i = 0; i < ARRAY_SIZE(hsotg->supplies); i++)
		hsotg->supplies[i].supply = dwc2_hsotg_supply_names[i];

	ret = devm_regulator_bulk_get(hsotg->dev, ARRAY_SIZE(hsotg->supplies),
				      hsotg->supplies);
	if (ret) {
		dev_err(hsotg->dev, "failed to request supplies: %d\n", ret);
		return ret;
	}
	return 0;
}

/**
 * dwc2_driver_remove() - Called when the DWC_otg core is unregistered with the
 * DWC_otg driver
 *
 * @dev: Platform device
 *
 * This routine is called, for example, when the rmmod command is executed. The
 * device may or may not be electrically present. If it is present, the driver
 * stops device processing. Any resources used on behalf of this device are
 * freed.
 */
static int dwc2_driver_remove(struct platform_device *dev)
{
	struct dwc2_hsotg *hsotg = platform_get_drvdata(dev);

	dwc2_debugfs_exit(hsotg);
	if (hsotg->hcd_enabled)
		dwc2_hcd_remove(hsotg);
	if (hsotg->gadget_enabled)
		dwc2_hsotg_remove(hsotg);

	if (hsotg->ll_hw_enabled)
		dwc2_lowlevel_hw_disable(hsotg);

	reset_control_assert(hsotg->reset);

	remove_test_mode_sysfs_files(hsotg);

	return 0;
}

/**
 * dwc2_driver_shutdown() - Called on device shutdown
 *
 * @dev: Platform device
 *
 * In specific conditions (involving usb hubs) dwc2 devices can create a
 * lot of interrupts, even to the point of overwhelming devices running
 * at low frequencies. Some devices need to do special clock handling
 * at shutdown-time which may bring the system clock below the threshold
 * of being able to handle the dwc2 interrupts. Disabling dwc2-irqs
 * prevents reboots/poweroffs from getting stuck in such cases.
 */
static void dwc2_driver_shutdown(struct platform_device *dev)
{
	struct dwc2_hsotg *hsotg = platform_get_drvdata(dev);

	disable_irq(hsotg->irq);
}

/**
 * dwc2_driver_probe() - Called when the DWC_otg core is bound to the DWC_otg
 * driver
 *
 * @dev: Platform device
 *
 * This routine creates the driver components required to control the device
 * (core, HCD, and PCD) and initializes the device. The driver components are
 * stored in a dwc2_hsotg structure. A reference to the dwc2_hsotg is saved
 * in the device private data. This allows the driver to access the dwc2_hsotg
 * structure on subsequent calls to driver methods for this device.
 */
static int dwc2_driver_probe(struct platform_device *dev)
{
	struct dwc2_hsotg *hsotg;
	struct resource *res;
	int retval;

	hsotg = devm_kzalloc(&dev->dev, sizeof(*hsotg), GFP_KERNEL);
	if (!hsotg)
		return -ENOMEM;

	hsotg->dev = &dev->dev;

	/*
	 * Use reasonable defaults so platforms don't have to provide these.
	 */
	if (!dev->dev.dma_mask)
		dev->dev.dma_mask = &dev->dev.coherent_dma_mask;
	retval = dma_set_coherent_mask(&dev->dev, DMA_BIT_MASK(32));
	if (retval)
		return retval;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	hsotg->regs = devm_ioremap_resource(&dev->dev, res);
	if (IS_ERR(hsotg->regs))
		return PTR_ERR(hsotg->regs);

	dev_dbg(&dev->dev, "mapped PA %08lx to VA %p\n",
		(unsigned long)res->start, hsotg->regs);

	retval = dwc2_lowlevel_hw_init(hsotg);
	if (retval)
		return retval;

	spin_lock_init(&hsotg->lock);
	spin_lock_init(&hsotg->channel_lock);

	hsotg->irq = platform_get_irq(dev, 0);
	if (hsotg->irq < 0) {
		dev_err(&dev->dev, "missing IRQ resource\n");
		return hsotg->irq;
	}

	dev_dbg(hsotg->dev, "registering common handler for irq%d\n",
		hsotg->irq);
	retval = devm_request_irq(hsotg->dev, hsotg->irq,
				  dwc2_handle_common_intr, IRQF_SHARED,
				  dev_name(hsotg->dev), hsotg);
	if (retval)
		return retval;

	if (of_device_is_compatible(hsotg->dev->of_node,
				    "nexell,nexell-dwc2otg")) {
#ifdef CONFIG_GPIOLIB
		hsotg->ext_vbus_io = of_get_named_gpio(dev->dev.of_node,
						       "gpios", 0);
		if (gpio_is_valid(hsotg->ext_vbus_io)) {
			retval = devm_gpio_request_one(&dev->dev,
						   hsotg->ext_vbus_io,
						   GPIOF_OUT_INIT_LOW,
						   "otg_vbus");

			if (retval < 0) {
				dev_err(hsotg->dev,
					"can't request otg_vbus gpio %d\n",
					hsotg->ext_vbus_io);
				return 0;
			}
		}
#endif
	}

	retval = dwc2_lowlevel_hw_enable(hsotg);
	if (retval)
		return retval;

	retval = dwc2_get_dr_mode(hsotg);
	if (retval)
		goto error;

	/*
	 * Reset before dwc2_get_hwparams() then it could get power-on real
	 * reset value form registers.
	 */
	if (of_device_is_compatible(hsotg->dev->of_node,
				    "nexell,nexell-dwc2otg"))
		dev_dbg(hsotg->dev, "Don't reset\n");
	else
		dwc2_core_reset_and_force_dr_mode(hsotg);

	/* Detect config values from hardware */
	retval = dwc2_get_hwparams(hsotg);
	if (retval)
		goto error;

	dwc2_force_dr_mode(hsotg);

	retval = dwc2_init_params(hsotg);
	if (retval)
		goto error;

	if (hsotg->dr_mode != USB_DR_MODE_HOST) {
		retval = dwc2_gadget_init(hsotg, hsotg->irq);
		if (retval)
			goto error;
		hsotg->gadget_enabled = 1;
	}

	if (hsotg->dr_mode != USB_DR_MODE_PERIPHERAL) {
		retval = dwc2_hcd_init(hsotg);
		if (retval) {
			if (hsotg->gadget_enabled)
				dwc2_hsotg_remove(hsotg);
			goto error;
		}
		hsotg->hcd_enabled = 1;
	}

	platform_set_drvdata(dev, hsotg);

	dwc2_debugfs_init(hsotg);

	create_test_mode_sysfs_files(hsotg);

	/* Gadget code manages lowlevel hw on its own */
	if (hsotg->dr_mode == USB_DR_MODE_PERIPHERAL)
		dwc2_lowlevel_hw_disable(hsotg);

	return 0;

error:
	dwc2_lowlevel_hw_disable(hsotg);
	return retval;
}

struct dwc2_core_global_regs {
	uint32_t gotgctl;
	uint32_t gotgint;
	uint32_t gahbcfg;
	uint32_t gusbcfg;
	uint32_t grstctl;
	uint32_t gintmsk;
	uint32_t grxfsiz;
	uint32_t gnptxfsiz;
	uint32_t gi2cctl;
	uint32_t gpvndctl;
	uint32_t ggpio;
	uint32_t ghwcfg1;
	uint32_t ghwcfg2;
	uint32_t ghwcfg3;
	uint32_t ghwcfg4;
	uint32_t glpmcfg;
	uint32_t gpwrdn;
	uint32_t gdfifocfg;
	uint32_t adpctl;
	uint32_t hptxfsiz;
	uint32_t dtxfsiz[15];
};

static struct dwc2_core_global_regs save_global_regs;

static void dwc2_driver_suspend_regs(struct dwc2_hsotg *hsotg, int suspend)
{
	struct dwc2_core_global_regs *regs = &save_global_regs;
	int idx;

	dev_dbg(hsotg->dev, "%s %d suspend %d\n", __func__, __LINE__, suspend);

	if (suspend) {
		regs->gotgctl = readl(hsotg->regs + GOTGCTL);
		regs->gotgint = readl(hsotg->regs + GOTGINT);
		regs->gahbcfg = readl(hsotg->regs + GAHBCFG);
		regs->gusbcfg = readl(hsotg->regs + GUSBCFG);
		regs->grstctl = readl(hsotg->regs + GRSTCTL);
		regs->gintmsk = readl(hsotg->regs + GINTMSK);
		regs->grxfsiz = readl(hsotg->regs + GRXFSIZ);
		regs->gnptxfsiz = readl(hsotg->regs + GNPTXFSIZ);
		regs->gi2cctl = readl(hsotg->regs + GI2CCTL);
		regs->gpvndctl = readl(hsotg->regs + GPVNDCTL);
		regs->ggpio = readl(hsotg->regs + GGPIO);
		regs->ghwcfg1 = readl(hsotg->regs + GHWCFG1);
		regs->ghwcfg2 = readl(hsotg->regs + GHWCFG2);
		regs->ghwcfg3 = readl(hsotg->regs + GHWCFG3);
		regs->ghwcfg4 = readl(hsotg->regs + GHWCFG4);
		regs->glpmcfg = readl(hsotg->regs + GLPMCFG);
		regs->gpwrdn = readl(hsotg->regs + GPWRDN);
		regs->gdfifocfg = readl(hsotg->regs + GDFIFOCFG);
		regs->adpctl = readl(hsotg->regs + ADPCTL);
		regs->hptxfsiz = readl(hsotg->regs + HPTXFSIZ);
		for (idx = 1; idx < hsotg->num_of_eps; idx++)
			regs->dtxfsiz[idx] = readl(hsotg->regs +
						   DPTXFSIZN(idx));
	} else {
		writel(regs->gotgctl, hsotg->regs + GOTGCTL);
		writel(regs->gotgint, hsotg->regs + GOTGINT);
		writel(regs->gahbcfg, hsotg->regs + GAHBCFG);
		writel(regs->gusbcfg, hsotg->regs + GUSBCFG);
		writel(regs->grstctl, hsotg->regs + GRSTCTL);
		writel(regs->gintmsk, hsotg->regs + GINTMSK);
		writel(regs->grxfsiz, hsotg->regs + GRXFSIZ);
		writel(regs->gnptxfsiz, hsotg->regs + GNPTXFSIZ);
		writel(regs->gi2cctl, hsotg->regs + GI2CCTL);
		writel(regs->gpvndctl, hsotg->regs + GPVNDCTL);
		writel(regs->ggpio, hsotg->regs + GGPIO);
		writel(regs->ghwcfg1, hsotg->regs + GHWCFG1);
		writel(regs->ghwcfg2, hsotg->regs + GHWCFG2);
		writel(regs->ghwcfg3, hsotg->regs + GHWCFG3);
		writel(regs->ghwcfg4, hsotg->regs + GHWCFG4);
		writel(regs->glpmcfg, hsotg->regs + GLPMCFG);
		writel(regs->gpwrdn, hsotg->regs + GPWRDN);
		writel(regs->gdfifocfg, hsotg->regs + GDFIFOCFG);
		writel(regs->adpctl, hsotg->regs + ADPCTL);
		writel(regs->hptxfsiz, hsotg->regs + HPTXFSIZ);
		for (idx = 1; idx < hsotg->num_of_eps; idx++)
			writel(regs->dtxfsiz[idx], hsotg->regs +
			       DPTXFSIZN(idx));
	}
}

static int __maybe_unused dwc2_suspend(struct device *dev)
{
	struct dwc2_hsotg *dwc2 = dev_get_drvdata(dev);
	int ret = 0;

	if (dwc2_is_device_mode(dwc2)) {
		ret = dwc2_hsotg_suspend(dwc2);
		if (ret)
			return ret;
	}

	if (of_device_is_compatible(dwc2->dev->of_node,
				"nexell,nexell-dwc2otg"))
		dwc2_driver_suspend_regs(dwc2, 1);

	if (dwc2->ll_hw_enabled)
		ret = __dwc2_lowlevel_hw_disable(dwc2);

	return ret;
}

static int __maybe_unused dwc2_resume(struct device *dev)
{
	struct dwc2_hsotg *dwc2 = dev_get_drvdata(dev);
	int ret = 0;

	if (dwc2->ll_hw_enabled) {
		ret = __dwc2_lowlevel_hw_enable(dwc2);
		if (ret)
			return ret;
	}

	if (of_device_is_compatible(dwc2->dev->of_node,
				"nexell,nexell-dwc2otg")) {
		dwc2_driver_suspend_regs(dwc2, 0);
		if (dwc2->op_state == OTG_STATE_B_PERIPHERAL)
			ret = dwc2_hsotg_resume(dwc2);
	} else {
		if (dwc2_is_device_mode(dwc2))
			ret = dwc2_hsotg_resume(dwc2);
	}

	return ret;
}

static const struct dev_pm_ops dwc2_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc2_suspend, dwc2_resume)
};

static struct platform_driver dwc2_platform_driver = {
	.driver = {
		.name = dwc2_driver_name,
		.of_match_table = dwc2_of_match_table,
		.pm = &dwc2_dev_pm_ops,
	},
	.probe = dwc2_driver_probe,
	.remove = dwc2_driver_remove,
	.shutdown = dwc2_driver_shutdown,
};
#if CONFIG_DWC_INIT_LEVEL_UP
static int __init dwc2_pltfm_init(void)
{
    return platform_driver_register(&dwc2_platform_driver);
}
fs_initcall(dwc2_pltfm_init)
#else
module_platform_driver(dwc2_platform_driver);
#endif

MODULE_DESCRIPTION("DESIGNWARE HS OTG Platform Glue");
MODULE_AUTHOR("Matthijs Kooijman <matthijs@stdin.nl>");
MODULE_LICENSE("Dual BSD/GPL");
