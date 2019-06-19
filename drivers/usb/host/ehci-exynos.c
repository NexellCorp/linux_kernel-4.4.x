/*
 * SAMSUNG EXYNOS USB HOST EHCI Controller
 *
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Author: Jingoo Han <jg1.han@samsung.com>
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#ifdef CONFIG_RESET_CONTROLLER
#include <linux/reset.h>
#endif

#include "ehci.h"

#if defined(CONFIG_ARCH_S5P4418)
#include <dt-bindings/tieoff/s5p4418-tieoff.h>
#elif defined(CONFIG_ARCH_S5P6818)
#include <dt-bindings/tieoff/s5p6818-tieoff.h>
#endif
#include <soc/nexell/tieoff.h>

#define DRIVER_DESC "EHCI EXYNOS driver"

#define EHCI_INSNREG00(base)			(base + 0x90)
#define EHCI_INSNREG00_ENA_INCR16		(0x1 << 25)
#define EHCI_INSNREG00_ENA_INCR8		(0x1 << 24)
#define EHCI_INSNREG00_ENA_INCR4		(0x1 << 23)
#define EHCI_INSNREG00_ENA_INCRX_ALIGN		(0x1 << 22)
#define EHCI_INSNREG00_ENABLE_DMA_BURST	\
	(EHCI_INSNREG00_ENA_INCR16 | EHCI_INSNREG00_ENA_INCR8 |	\
	 EHCI_INSNREG00_ENA_INCR4 | EHCI_INSNREG00_ENA_INCRX_ALIGN)
#if defined(CONFIG_ARCH_S5P4418) || defined(CONFIG_ARCH_S5P6818)
#define PORTSC_1(base)					(base + 0x54)
#define EHCI_INSNREG08(base)			(base + 0xb0)
#endif

static const char hcd_name[] = "ehci-exynos";
static struct hc_driver __read_mostly exynos_ehci_hc_driver;

#ifdef CONFIG_USB_EHCI_EXYNOS_TEST_MODE
static struct kobject *host_kobj;
struct usb_hcd *host_hcd;
#endif

#define PHY_NUMBER 3

struct exynos_ehci_hcd {
	struct clk *clk;
	struct phy *phy[PHY_NUMBER];
};

#define to_exynos_ehci(hcd) (struct exynos_ehci_hcd *)(hcd_to_ehci(hcd)->priv)

static int exynos_ehci_get_phy(struct device *dev,
				struct exynos_ehci_hcd *exynos_ehci)
{
	struct device_node *child;
	struct phy *phy;
	int phy_number;
	int ret;

	/* Get PHYs for the controller */
	for_each_available_child_of_node(dev->of_node, child) {
		ret = of_property_read_u32(child, "reg", &phy_number);
		if (ret) {
			dev_err(dev, "Failed to parse device tree\n");
			of_node_put(child);
			return ret;
		}

		if (phy_number >= PHY_NUMBER) {
			dev_err(dev, "Invalid number of PHYs\n");
			of_node_put(child);
			return -EINVAL;
		}

		phy = devm_of_phy_get(dev, child, NULL);
		exynos_ehci->phy[phy_number] = phy;
		if (IS_ERR(phy)) {
			ret = PTR_ERR(phy);
			if (ret == -EPROBE_DEFER) {
				of_node_put(child);
				return ret;
			} else if (ret != -ENOSYS && ret != -ENODEV) {
				dev_err(dev,
					"Error retrieving usb2 phy: %d\n", ret);
				of_node_put(child);
				return ret;
			}
		}
	}

	return 0;
}

static int exynos_ehci_phy_enable(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct exynos_ehci_hcd *exynos_ehci = to_exynos_ehci(hcd);
	int i;
	int ret = 0;

	if (of_device_is_compatible(dev->of_node,
					"nexell,nexell-ehci")) {
#ifdef CONFIG_RESET_CONTROLLER
		struct reset_control *rst;

		rst = reset_control_get(dev, "usbhost-reset");
		if (!IS_ERR(rst)) {
			if (reset_control_status(rst))
				reset_control_reset(rst);
			reset_control_put(rst);
		}
#endif
	}

	for (i = 0; ret == 0 && i < PHY_NUMBER; i++)
		if (!IS_ERR(exynos_ehci->phy[i]))
			ret = phy_power_on(exynos_ehci->phy[i]);
	if (ret)
		for (i--; i >= 0; i--)
			if (!IS_ERR(exynos_ehci->phy[i]))
				phy_power_off(exynos_ehci->phy[i]);

	return ret;
}

static void exynos_ehci_phy_disable(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct exynos_ehci_hcd *exynos_ehci = to_exynos_ehci(hcd);
	int i;

	for (i = 0; i < PHY_NUMBER; i++)
		if (!IS_ERR(exynos_ehci->phy[i]))
			phy_power_off(exynos_ehci->phy[i]);
}

static void exynos_setup_vbus_gpio(struct device *dev)
{
	int err;
	int gpio;

	if (!dev->of_node)
		return;

	gpio = of_get_named_gpio(dev->of_node, "samsung,vbus-gpio", 0);
	if (!gpio_is_valid(gpio))
		return;

	err = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_HIGH,
				    "ehci_vbus_gpio");
	if (err)
		dev_err(dev, "can't request ehci vbus gpio %d", gpio);
}

#ifdef CONFIG_USB_EHCI_EXYNOS_TEST_MODE
static ssize_t show_host_test_mode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int32_t readVal;

	readVal = readl(PORTSC_1(host_hcd->regs));

	pr_info("%s():%d host testmode HOST_PORTSC_1:0x%x\n", __func__,
		__LINE__, readVal);

	return sprintf(buf, "HOST_PORTSC_1 Port Test Control [0x%x]\n",
		       (readVal >> 16) & 0xf);
}

static ssize_t store_host_test_mode(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 5))
		return -EINVAL;

	pr_info("HOST_PORTSC_1 value = 0x%x\n", value);

	writel((readl(PORTSC_1(host_hcd->regs)) & 0xfff0ffff) | (value << 16),
	       PORTSC_1(host_hcd->regs));

	return len;
}

static DEVICE_ATTR(host_test_mode, 0644, show_host_test_mode,
		   store_host_test_mode);

/* NX_TIEOFF_USB20HOST0_i_COMPDISTUNE */
static ssize_t show_host_compdistune(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_COMPDISTUNE);

	return sprintf(buf, "HOST_COMPDISTUNE [0x%x]\n", readVal);
}

static ssize_t store_host_compdistune(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 7))
		return -EINVAL;

	pr_info("HOST_COMPDISTUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_COMPDISTUNE, value);

	return len;
}

static DEVICE_ATTR(host_compdistune, 0644, show_host_compdistune,
		   store_host_compdistune);

/* NX_TIEOFF_USB20HOST0_i_SQRXTUNE */
static ssize_t show_host_sqrxtune(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_SQRXTUNE);

	return sprintf(buf, "HOST_SQRXTUNE [0x%x]\n", readVal);
}

static ssize_t store_host_sqrxtune(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 7))
		return -EINVAL;

	pr_info("HOST_SQRXTUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_SQRXTUNE, value);

	return len;
}

static DEVICE_ATTR(host_sqrxtune, 0644, show_host_sqrxtune,
		   store_host_sqrxtune);

/* NX_TIEOFF_USB20HOST0_i_OTGTUNE */
static ssize_t show_host_otgtune(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_OTGTUNE);

	return sprintf(buf, "HOST_OTGTUNE [0x%x]\n", readVal);
}

static ssize_t store_host_otgtune(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 7))
		return -EINVAL;

	pr_info("HOST_OTGTUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_OTGTUNE, value);

	return len;
}

static DEVICE_ATTR(host_otgtune, 0644, show_host_otgtune, store_host_otgtune);

/* NX_TIEOFF_USB20HOST0_i_TXHSXVTUNE */
static ssize_t show_host_txhsxvtune(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_TXHSXVTUNE);

	return sprintf(buf, "HOST_TXHSXVTUNE [0x%x]\n", readVal);
}

static ssize_t store_host_txhsxvtune(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 3))
		return -EINVAL;

	pr_info("HOST_TXHSXVTUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_TXHSXVTUNE, value);

	return len;
}

static DEVICE_ATTR(host_txhsxvtune, 0644, show_host_txhsxvtune,
		   store_host_txhsxvtune);

/* NX_TIEOFF_USB20HOST0_i_TXFSLSTUNE */
static ssize_t show_host_txfslstune(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_TXFSLSTUNE);

	return sprintf(buf, "HOST_TXFSLSTUNE [0x%x]\n", readVal);
}

static ssize_t store_host_txfslstune(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value == 0) || (value == 1) || (value == 3) || (value == 7) ||
	    (value == 15))
		pr_info("%s() valid value = 0x%x\n", __func__, value);
	else
		return -EINVAL;

	pr_info("HOST_TXFSLSTUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_TXFSLSTUNE, value);

	return len;
}

static DEVICE_ATTR(host_txfslstune, 0644, show_host_txfslstune,
		   store_host_txfslstune);

/* NX_TIEOFF_USB20HOST0_i_TXVREFTUNE */
static ssize_t show_host_txvreftune(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_TXVREFTUNE);

	return sprintf(buf, "HOST_TXVREFTUNE [0x%x]\n", readVal);
}

static ssize_t store_host_txvreftune(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 15))
		return -EINVAL;

	pr_info("HOST_TXVREFTUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_TXVREFTUNE, value);

	return len;
}

static DEVICE_ATTR(host_txvreftune, 0644, show_host_txvreftune,
		   store_host_txvreftune);

/* NX_TIEOFF_USB20HOST0_i_TXRISETUNE */
static ssize_t show_host_txrisetune(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_TXRISETUNE);

	return sprintf(buf, "HOST_TXRISETUNE [0x%x]\n", readVal);
}

static ssize_t store_host_txrisetune(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 3))
		return -EINVAL;

	pr_info("HOST_TXRISETUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_TXRISETUNE, value);

	return len;
}

static DEVICE_ATTR(host_txrisetune, 0644, show_host_txrisetune,
		   store_host_txrisetune);

/* NX_TIEOFF_USB20HOST0_i_TXRESTUNE */
static ssize_t show_host_txrestune(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_TXRESTUNE);

	return sprintf(buf, "HOST_TXRESTUNE [0x%x]\n", readVal);
}

static ssize_t store_host_txrestune(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 3))
		return -EINVAL;

	pr_info("HOST_TXRESTUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_TXRESTUNE, value);

	return len;
}

static DEVICE_ATTR(host_txrestune, 0644, show_host_txrestune,
		   store_host_txrestune);

/* NX_TIEOFF_USB20HOST0_i_TXPREEMPAMPTUNE */
static ssize_t show_host_txpreempamptune(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_TXPREEMPAMPTUNE);

	return sprintf(buf, "HOST_TXPREEMPAMPTUNE [0x%x]\n", readVal);
}

static ssize_t store_host_txpreempamptune(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 3))
		return -EINVAL;

	pr_info("HOST_TXPREEMPAMPTUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_TXPREEMPAMPTUNE, value);

	return len;
}

static DEVICE_ATTR(host_txpreempamptune, 0644, show_host_txpreempamptune,
		   store_host_txpreempamptune);

/* NX_TIEOFF_USB20HOST0_i_TXPREEMPPULSETUNE */
static ssize_t show_host_txpreemppulsetune(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	int32_t readVal;

	readVal = nx_tieoff_get(NX_TIEOFF_USB20HOST0_i_TXPREEMPPULSETUNE);

	return sprintf(buf, "HOST_TXPREEMPPULSETUNE [0x%x]\n", readVal);
}

static ssize_t store_host_txpreemppulsetune(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t len)
{
	int value;

	if (kstrtoint(buf, 0, &value))
		return -EINVAL;

	if ((value < 0) || (value > 1))
		return -EINVAL;

	pr_info("HOST_TXPREEMPPULSETUNE value = 0x%x\n", value);

	nx_tieoff_set(NX_TIEOFF_USB20HOST0_i_TXPREEMPPULSETUNE, value);

	return len;
}

static DEVICE_ATTR(host_txpreemppulsetune, 0644, show_host_txpreemppulsetune,
		   store_host_txpreemppulsetune);

static struct attribute *host_attrs[] = {
	&dev_attr_host_test_mode.attr,
	&dev_attr_host_compdistune.attr,
	&dev_attr_host_sqrxtune.attr,
	&dev_attr_host_otgtune.attr,
	&dev_attr_host_txhsxvtune.attr,
	&dev_attr_host_txfslstune.attr,
	&dev_attr_host_txvreftune.attr,
	&dev_attr_host_txrisetune.attr,
	&dev_attr_host_txrestune.attr,
	&dev_attr_host_txpreempamptune.attr,
	&dev_attr_host_txpreemppulsetune.attr,
	NULL,
};

static struct attribute_group usb_host_attr_group = {
	.attrs = (struct attribute **)host_attrs,
};

static int create_host_test_mode_sysfs(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	int	ret = 0;

	host_hcd = hcd;

	host_kobj = kobject_create_and_add("usb_host_test", NULL);
	if (host_kobj == NULL) {
		dev_err(dev, "host_kobj: kobject_create_and_add failed\n");
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_group(host_kobj, &usb_host_attr_group);
	if (ret) {
		dev_err(dev, "%s: Failed, sysfs group for usb_host\n",
				__func__);
		kobject_del(host_kobj);
	}

	return ret;
}

static inline void remove_host_test_mode_sysfs(struct device *dev)
{
	if (host_kobj) {
		sysfs_remove_group(host_kobj, &usb_host_attr_group);
		kobject_del(host_kobj);
	}
}
#endif

static int exynos_ehci_probe(struct platform_device *pdev)
{
	struct exynos_ehci_hcd *exynos_ehci;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	struct resource *res;
	int irq;
	int err;

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we move to full device tree support this will vanish off.
	 */
	err = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (err)
		return err;

	exynos_setup_vbus_gpio(&pdev->dev);

	hcd = usb_create_hcd(&exynos_ehci_hc_driver,
			     &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		return -ENOMEM;
	}
	exynos_ehci = to_exynos_ehci(hcd);

	if (of_device_is_compatible(pdev->dev.of_node,
					"samsung,exynos5440-ehci"))
		goto skip_phy;

	err = exynos_ehci_get_phy(&pdev->dev, exynos_ehci);
	if (err)
		goto fail_clk;

skip_phy:
	exynos_ehci->clk = devm_clk_get(&pdev->dev, "usbhost");

	if (IS_ERR(exynos_ehci->clk)) {
		dev_err(&pdev->dev, "Failed to get usbhost clock\n");
		err = PTR_ERR(exynos_ehci->clk);
		goto fail_clk;
	}

	err = clk_prepare_enable(exynos_ehci->clk);
	if (err)
		goto fail_clk;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		err = PTR_ERR(hcd->regs);
		goto fail_io;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENODEV;
		goto fail_io;
	}

	err = exynos_ehci_phy_enable(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable USB phy\n");
		goto fail_io;
	}

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;

	/* DMA burst Enable */
	writel(EHCI_INSNREG00_ENABLE_DMA_BURST, EHCI_INSNREG00(hcd->regs));

	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err) {
		dev_err(&pdev->dev, "Failed to add USB HCD\n");
		goto fail_add_hcd;
	}
	device_wakeup_enable(hcd->self.controller);

	if (of_device_is_compatible(pdev->dev.of_node, "nexell,nexell-ehci"))
		pm_runtime_forbid(&hcd->self.root_hub->dev);

#ifdef CONFIG_USB_EHCI_EXYNOS_TEST_MODE
	create_host_test_mode_sysfs(&pdev->dev);
#endif

	platform_set_drvdata(pdev, hcd);

	return 0;

fail_add_hcd:
	exynos_ehci_phy_disable(&pdev->dev);
fail_io:
	clk_disable_unprepare(exynos_ehci->clk);
fail_clk:
	usb_put_hcd(hcd);
	return err;
}

static int exynos_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct exynos_ehci_hcd *exynos_ehci = to_exynos_ehci(hcd);

	usb_remove_hcd(hcd);

	exynos_ehci_phy_disable(&pdev->dev);

	clk_disable_unprepare(exynos_ehci->clk);

	usb_put_hcd(hcd);
#ifdef CONFIG_USB_EHCI_EXYNOS_TEST_MODE
	remove_host_test_mode_sysfs(&pdev->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int exynos_ehci_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct exynos_ehci_hcd *exynos_ehci = to_exynos_ehci(hcd);

	bool do_wakeup = device_may_wakeup(dev);
	int rc;

	rc = ehci_suspend(hcd, do_wakeup);
	if (rc)
		return rc;

	exynos_ehci_phy_disable(dev);

	clk_disable_unprepare(exynos_ehci->clk);

	return rc;
}

static int exynos_ehci_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct exynos_ehci_hcd *exynos_ehci = to_exynos_ehci(hcd);
	int ret;

	clk_prepare_enable(exynos_ehci->clk);

	ret = exynos_ehci_phy_enable(dev);
	if (ret) {
		dev_err(dev, "Failed to enable USB phy\n");
		clk_disable_unprepare(exynos_ehci->clk);
		return ret;
	}

	/* DMA burst Enable */
	writel(EHCI_INSNREG00_ENABLE_DMA_BURST, EHCI_INSNREG00(hcd->regs));

	ehci_resume(hcd, false);
	return 0;
}
#else
#define exynos_ehci_suspend	NULL
#define exynos_ehci_resume	NULL
#endif

static const struct dev_pm_ops exynos_ehci_pm_ops = {
	.suspend	= exynos_ehci_suspend,
	.resume		= exynos_ehci_resume,
};

#if defined(CONFIG_ARCH_S5P4418) || defined(CONFIG_ARCH_S5P6818)
int hsic_port_power(struct usb_hcd *hcd, int portnum, bool enable)
{
	if (portnum == 1) {
		if (enable)
			writel(readl(EHCI_INSNREG08(hcd->regs)) |
			       1 << portnum, EHCI_INSNREG08(hcd->regs));
		else
			writel(readl(EHCI_INSNREG08(hcd->regs)) &
			       ~(1 << portnum), EHCI_INSNREG08(hcd->regs));
	}
	return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id exynos_ehci_match[] = {
	{ .compatible = "samsung,exynos4210-ehci" },
	{ .compatible = "samsung,exynos5440-ehci" },
	{ .compatible = "nexell,nexell-ehci" },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_ehci_match);
#endif

static struct platform_driver exynos_ehci_driver = {
	.probe		= exynos_ehci_probe,
	.remove		= exynos_ehci_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver = {
		.name	= "exynos-ehci",
		.pm	= &exynos_ehci_pm_ops,
		.of_match_table = of_match_ptr(exynos_ehci_match),
	}
};
static const struct ehci_driver_overrides exynos_overrides __initdata = {
	.extra_priv_size = sizeof(struct exynos_ehci_hcd),
#if defined(CONFIG_ARCH_S5P4418) || defined(CONFIG_ARCH_S5P6818)
	.port_power = hsic_port_power,
#endif
};

static int __init ehci_exynos_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);
	ehci_init_driver(&exynos_ehci_hc_driver, &exynos_overrides);
	return platform_driver_register(&exynos_ehci_driver);
}
module_init(ehci_exynos_init);

static void __exit ehci_exynos_cleanup(void)
{
	platform_driver_unregister(&exynos_ehci_driver);
}
module_exit(ehci_exynos_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:exynos-ehci");
MODULE_AUTHOR("Jingoo Han");
MODULE_AUTHOR("Joonyoung Shim");
MODULE_LICENSE("GPL v2");
