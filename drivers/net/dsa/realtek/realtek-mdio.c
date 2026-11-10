// SPDX-License-Identifier: GPL-2.0+
/* Realtek MDIO interface driver
 *
 * ASICs we intend to support with this driver:
 *
 * RTL8366   - The original version, apparently
 * RTL8369   - Similar enough to have the same datsheet as RTL8366
 * RTL8366RB - Probably reads out "RTL8366 revision B", has a quite
 *             different register layout from the other two
 * RTL8366S  - Is this "RTL8366 super"?
 * RTL8367   - Has an OpenWRT driver as well
 * RTL8368S  - Seems to be an alternative name for RTL8366RB
 * RTL8370   - Also uses SMI
 *
 * Copyright (C) 2017 Linus Walleij <linus.walleij@linaro.org>
 * Copyright (C) 2010 Antti Seppälä <a.seppala@gmail.com>
 * Copyright (C) 2010 Roman Yeryomin <roman@advem.lv>
 * Copyright (C) 2011 Colin Leitner <colin.leitner@googlemail.com>
 * Copyright (C) 2009-2010 Gabor Juhos <juhosg@openwrt.org>
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/regmap.h>
#include <linux/wordpart.h>

#include "realtek.h"
#include "realtek-mdio.h"
#include "rtl83xx.h"

/* Read/write via mdiobus */
#define REALTEK_MDIO_CTRL0_REG		31
#define REALTEK_MDIO_START_REG		29
#define REALTEK_MDIO_CTRL1_REG		21
#define REALTEK_MDIO_ADDRESS_REG	23
#define REALTEK_MDIO_DATA_WRITE_REG	24
#define REALTEK_MDIO_DATA_READ_REG	25

#define REALTEK_MDIO_START_OP		0xFFFF
#define REALTEK_MDIO_ADDR_OP		0x000E
#define REALTEK_MDIO_READ_OP		0x0001
#define REALTEK_MDIO_WRITE_OP		0x0003

static int realtek_mdio_write(void *ctx, u32 reg, u32 val)
{
	struct realtek_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	int ret;

	mutex_lock(&bus->mdio_lock);

	ret = bus->write(bus, priv->mdio_addr, REALTEK_MDIO_CTRL0_REG, REALTEK_MDIO_ADDR_OP);
	if (ret)
		goto out_unlock;

	ret = bus->write(bus, priv->mdio_addr, REALTEK_MDIO_ADDRESS_REG, reg);
	if (ret)
		goto out_unlock;

	ret = bus->write(bus, priv->mdio_addr, REALTEK_MDIO_DATA_WRITE_REG, val);
	if (ret)
		goto out_unlock;

	ret = bus->write(bus, priv->mdio_addr, REALTEK_MDIO_CTRL1_REG, REALTEK_MDIO_WRITE_OP);

out_unlock:
	mutex_unlock(&bus->mdio_lock);

	return ret;
}

static int realtek_mdio_read(void *ctx, u32 reg, u32 *val)
{
	struct realtek_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	int ret;

	mutex_lock(&bus->mdio_lock);

	ret = bus->write(bus, priv->mdio_addr, REALTEK_MDIO_CTRL0_REG, REALTEK_MDIO_ADDR_OP);
	if (ret)
		goto out_unlock;

	ret = bus->write(bus, priv->mdio_addr, REALTEK_MDIO_ADDRESS_REG, reg);
	if (ret)
		goto out_unlock;

	ret = bus->write(bus, priv->mdio_addr, REALTEK_MDIO_CTRL1_REG, REALTEK_MDIO_READ_OP);
	if (ret)
		goto out_unlock;

	ret = bus->read(bus, priv->mdio_addr, REALTEK_MDIO_DATA_READ_REG);
	if (ret >= 0) {
		*val = ret;
		ret = 0;
	}

out_unlock:
	mutex_unlock(&bus->mdio_lock);

	return ret;
}

static const struct realtek_interface_info realtek_mdio_info = {
	.reg_read = realtek_mdio_read,
	.reg_write = realtek_mdio_write,
};

/* Read/write via mdiobus, RTL837x */
#define RTL837X_MDIO_CTRL_REG		21
#define RTL837X_MDIO_ADDR_REG		22
#define RTL837X_MDIO_DATA_LO_REG	23
#define RTL837X_MDIO_DATA_HI_REG	24

#define RTL837X_MDIO_READ_OP		0x001b
#define RTL837X_MDIO_WRITE_OP		0x0019

static int rtl837x_mdio_read(void *ctx, u32 reg, u32 *val)
{
	struct realtek_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	int addr = priv->mdio_addr;
	int lo, hi, ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	ret = __mdiobus_write(bus, addr, RTL837X_MDIO_ADDR_REG, reg);
	if (ret)
		goto out_unlock;

	ret = __mdiobus_write(bus, addr, RTL837X_MDIO_CTRL_REG,
			      RTL837X_MDIO_READ_OP);
	if (ret)
		goto out_unlock;

	lo = __mdiobus_read(bus, addr, RTL837X_MDIO_DATA_LO_REG);
	if (lo < 0) {
		ret = lo;
		goto out_unlock;
	}

	hi = __mdiobus_read(bus, addr, RTL837X_MDIO_DATA_HI_REG);
	if (hi < 0) {
		ret = hi;
		goto out_unlock;
	}

	*val = (u32)hi << 16 | lo;

out_unlock:
	mutex_unlock(&bus->mdio_lock);

	return ret;
}

static int rtl837x_mdio_write(void *ctx, u32 reg, u32 val)
{
	struct realtek_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	int addr = priv->mdio_addr;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	ret = __mdiobus_write(bus, addr, RTL837X_MDIO_ADDR_REG, reg);
	if (ret)
		goto out_unlock;

	ret = __mdiobus_write(bus, addr, RTL837X_MDIO_DATA_LO_REG,
			      lower_16_bits(val));
	if (ret)
		goto out_unlock;

	ret = __mdiobus_write(bus, addr, RTL837X_MDIO_DATA_HI_REG,
			      upper_16_bits(val));
	if (ret)
		goto out_unlock;

	ret = __mdiobus_write(bus, addr, RTL837X_MDIO_CTRL_REG,
			      RTL837X_MDIO_WRITE_OP);

out_unlock:
	mutex_unlock(&bus->mdio_lock);

	return ret;
}

static const struct realtek_interface_info rtl837x_mdio_info = {
	.reg_read = rtl837x_mdio_read,
	.reg_write = rtl837x_mdio_write,
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
};

static int __realtek_mdio_probe(struct mdio_device *mdiodev,
				const struct realtek_interface_info *info)
{
	struct device *dev = &mdiodev->dev;
	struct realtek_priv *priv;
	int ret;

	priv = rtl83xx_probe(dev, info);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	priv->bus = mdiodev->bus;
	priv->mdio_addr = mdiodev->addr;
	priv->write_reg_noack = info->reg_write;

	ret = rtl83xx_register_switch(priv);
	if (ret) {
		rtl83xx_remove(priv);
		return ret;
	}

	return 0;
}

/**
 * realtek_mdio_probe() - Probe a platform device for an MDIO-connected switch
 * @mdiodev: mdio_device to probe on.
 *
 * This function should be used as the .probe in an mdio_driver. After
 * calling the common probe function for both interfaces, it initializes the
 * values specific for MDIO-connected devices. Finally, it calls a common
 * function to register the DSA switch.
 *
 * Context: Can sleep. Takes and releases priv->map_lock.
 * Return: Returns 0 on success, a negative error on failure.
 */
int realtek_mdio_probe(struct mdio_device *mdiodev)
{
	return __realtek_mdio_probe(mdiodev, &realtek_mdio_info);
}
EXPORT_SYMBOL_NS_GPL(realtek_mdio_probe, "REALTEK_DSA");

/**
 * rtl837x_mdio_probe() - Probe an MDIO-connected RTL837x switch
 * @mdiodev: mdio_device to probe on.
 *
 * Like realtek_mdio_probe(), for the RTL837x family: 16-bit register
 * addresses, 32-bit values and its own indirect access sequence. Removal and
 * shutdown use realtek_mdio_remove() and realtek_mdio_shutdown().
 *
 * Context: Can sleep. Takes and releases priv->map_lock.
 * Return: Returns 0 on success, a negative error on failure.
 */
int rtl837x_mdio_probe(struct mdio_device *mdiodev)
{
	return __realtek_mdio_probe(mdiodev, &rtl837x_mdio_info);
}
EXPORT_SYMBOL_NS_GPL(rtl837x_mdio_probe, "REALTEK_DSA");

/**
 * realtek_mdio_remove() - Remove the driver of an MDIO-connected switch
 * @mdiodev: mdio_device to be removed.
 *
 * This function should be used as the .remove in an mdio_driver. First
 * it unregisters the DSA switch and then it calls the common remove function.
 *
 * Context: Can sleep.
 * Return: Nothing.
 */
void realtek_mdio_remove(struct mdio_device *mdiodev)
{
	struct realtek_priv *priv = dev_get_drvdata(&mdiodev->dev);

	if (!priv)
		return;

	rtl83xx_unregister_switch(priv);

	rtl83xx_remove(priv);
}
EXPORT_SYMBOL_NS_GPL(realtek_mdio_remove, "REALTEK_DSA");

/**
 * realtek_mdio_shutdown() - Shutdown the driver of a MDIO-connected switch
 * @mdiodev: mdio_device shutting down.
 *
 * This function should be used as the .shutdown in a platform_driver. It calls
 * the common shutdown function.
 *
 * Context: Can sleep.
 * Return: Nothing.
 */
void realtek_mdio_shutdown(struct mdio_device *mdiodev)
{
	struct realtek_priv *priv = dev_get_drvdata(&mdiodev->dev);

	if (!priv)
		return;

	rtl83xx_shutdown(priv);
}
EXPORT_SYMBOL_NS_GPL(realtek_mdio_shutdown, "REALTEK_DSA");
