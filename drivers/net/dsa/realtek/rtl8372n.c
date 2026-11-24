// SPDX-License-Identifier: GPL-2.0
/*
 * Realtek RTL8372N DSA switch driver
 *
 * The register addresses, values and setup sequences are based on the
 * rtl837x DSA driver by StarField Xu.
 */
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/if_bridge.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/mdio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/wordpart.h>
#include <net/dsa.h>

#include "realtek.h"
#include "realtek-mdio.h"
#include "rtl83xx.h"

#define RTL8372N_CHIP_MODEL_REG		0x0004
#define RTL8372N_CHIP_MODEL_RTL8372N	0x83727000

#define RTL8372N_RESET_REG		0x0024
#define RTL8372N_RESET_SOFT		BIT(0)

#define RTL8372N_PORT_ISOLATION_REG(port)	(0x50c0 + (port) * 4)

/* Spanning tree state of each port, two bits per port. There is one register
 * per filtering database; the driver keeps them all the same.
 */
#define RTL8372N_STP_STATE_REG(fid)	(0x5310 + (fid) * 4)
#define RTL8372N_STP_STATE_MASK(port)	(GENMASK(1, 0) << ((port) * 2))
#define RTL8372N_STP_STATE_DISABLED	0
#define RTL8372N_STP_STATE_BLOCKING	1
#define RTL8372N_STP_STATE_LEARNING	2
#define RTL8372N_STP_STATE_FORWARDING	3
#define RTL8372N_NUM_FIDS		16

/* Maximum frame length per port: one field for 10/100 Mbit/s links,
 * one for 1 Gbit/s and faster
 */
#define RTL8372N_PORT_MAX_LEN_REG(port)	(0x1250 + (port) * 0x100)
#define RTL8372N_PORT_MAX_LEN_FE	GENMASK(13, 0)
#define RTL8372N_PORT_MAX_LEN_GE	GENMASK(27, 14)
#define RTL8372N_PORT_MAX_LEN_MAX	0x3fff

/* MIB counters: a command selects port and counter, then the value is read */
#define RTL8372N_MIB_CMD_REG		0x0f60
#define RTL8372N_MIB_CMD_BUSY		BIT(0)
#define RTL8372N_MIB_CMD_PORT		GENMASK(4, 1)
#define RTL8372N_MIB_CMD_COUNTER	GENMASK(10, 5)
#define RTL8372N_MIB_DATA_HI_REG	0x0f64
#define RTL8372N_MIB_DATA_LO_REG	0x0f68

#define RTL8372N_EXT_CPU_PORT_REG	0x6724
#define RTL8372N_EXT_CPU_PORT		GENMASK(3, 0)

#define RTL8372N_CPU_CTRL_REG		0x6720
#define RTL8372N_CPU_CTRL_TAG_EN	BIT(1)
#define RTL8372N_CPU_CTRL_INSERT_MODE	GENMASK(11, 10)
#define RTL8372N_CPU_CTRL_INSERT_ALL	0

#define RTL8372N_CPU_TAG_AWARE_REG	0x603c

#define RTL8372N_L2_LIMIT_ACTION_REG		0x4f80
#define RTL8372N_L2_LIMIT_ACTION		GENMASK(1, 0)
#define RTL8372N_L2_LIMIT_ACTION_FORWARD	0

#define RTL8372N_L2_LEARN_LIMIT_REG(port)	(0x5384 + (port) * 4)
#define RTL8372N_L2_LEARN_LIMIT			GENMASK(12, 0)
/* Size of the address table: a port with this limit learns without limit */
#define RTL8372N_L2_TABLE_SIZE			4160

/* Remove the learned addresses of the ports in the command */
#define RTL8372N_L2_FLUSH_CMD_REG		0x53d4
#define RTL8372N_L2_FLUSH_CMD_PORTS		GENMASK(9, 0)
#define RTL8372N_L2_FLUSH_CMD_START		BIT(16)
#define RTL8372N_L2_FLUSH_CMD_BUSY		BIT(17)

#define RTL8372N_L2_FLUSH_MODE_REG		0x53dc
#define RTL8372N_L2_FLUSH_MODE			GENMASK(1, 0)
#define RTL8372N_L2_FLUSH_MODE_PORT		0

/* Indirect access to the address table: a command register, and three data
 * words for an entry
 */
#define RTL8372N_TBL_CMD_REG			0x5cac
#define RTL8372N_TBL_CMD_INDEX			GENMASK(28, 16)
#define RTL8372N_TBL_CMD_TYPE			GENMASK(10, 8)
#define RTL8372N_TBL_CMD_TYPE_L2		4
/* Start bit, reads as 1 while the command runs */
#define RTL8372N_TBL_CMD_EXEC			BIT(0)

#define RTL8372N_TBL_RDATA_REG(i)		(0x5ccc + (i) * 4)
#define RTL8372N_L2_ENTRY_WORDS			3

/* How a read command searches the address table, and its result */
#define RTL8372N_L2_LOOKUP_REG			0x5cb0
#define RTL8372N_L2_LOOKUP_PORT			GENMASK(22, 19)
#define RTL8372N_L2_LOOKUP_METHOD		GENMASK(17, 14)
/* Next unicast entry of the port, starting at the index of the command */
#define RTL8372N_L2_LOOKUP_METHOD_NEXT_UC_PORT	7
#define RTL8372N_L2_LOOKUP_HIT			BIT(12)
#define RTL8372N_L2_LOOKUP_INDEX		GENMASK(11, 0)

/* Unicast entry: word 0 holds the last four bytes of the address */
#define RTL8372N_L2_UC_D1_MAC_HI		GENMASK(15, 0)
#define RTL8372N_L2_UC_D1_VID			GENMASK(27, 16)
/* The entry is keyed by VLAN ID; without it the field is a filtering ID */
#define RTL8372N_L2_UC_D1_IVL			BIT(29)
#define RTL8372N_L2_UC_D2_STATIC		BIT(16)

#define RTL8372N_SDS_CMD_REG		0x03f8
#define RTL8372N_SDS_CMD_INDEX		BIT(0)
#define RTL8372N_SDS_CMD_PAGE		GENMASK(6, 1)
#define RTL8372N_SDS_CMD_REGNUM		GENMASK(11, 7)
#define RTL8372N_SDS_CMD_RW		BIT(14)
#define RTL8372N_SDS_CMD_RW_READ	0
#define RTL8372N_SDS_CMD_RW_WRITE	1
#define RTL8372N_SDS_CMD_EXEC		BIT(15)
#define RTL8372N_SDS_RDATA_REG		0x03fc
#define RTL8372N_SDS_WDATA_REG		0x0400

#define RTL8372N_SDS_FRC_PAGE		0x20
#define RTL8372N_SDS_FRC_REG		0x00
#define RTL8372N_SDS_FRC_RX		GENMASK(5, 4)
#define RTL8372N_SDS_FRC_PDOWN		GENMASK(7, 6)
#define RTL8372N_SDS_FRC_CMU		GENMASK(11, 10)
#define RTL8372N_SDS_FRC_AUTO		0
#define RTL8372N_SDS_FRC_0		1
#define RTL8372N_SDS_FRC_1		3

/* Code the SerDes sends in its USXGMII negotiation word */
#define RTL8372N_SDS_USX_AN_PAGE	0x07
#define RTL8372N_SDS_USX_AN_REG		0x10
#define RTL8372N_SDS_USX_AN_OPCODE	GENMASK(7, 0)
#define RTL8372N_SDS_USX_AN_OPCODE_STD	0x03
/* USXGMII negotiation on, one bit per channel of the SerDes */
#define RTL8372N_SDS_USX_AN_EN_REG	0x11
#define RTL8372N_SDS_USX_AN_EN		GENMASK(3, 0)

#define RTL8372N_SDS_MODE_REG		0x7b20
#define RTL8372N_SDS_MODE_SDS0		GENMASK(4, 0)
#define RTL8372N_SDS_MODE_SDS1		GENMASK(9, 5)
#define RTL8372N_SDS_MODE_SDS0_USX_SUB	GENMASK(14, 10)
#define RTL8372N_SDS_MODE_SDS1_USX_SUB	GENMASK(20, 16)
#define RTL8372N_SDS_MODE_MAC3_8221B	BIT(21)
#define RTL8372N_SDS_MODE_MAC8_8221B	BIT(22)
#define RTL8372N_SDS_MODE_USXGMII	0x0d
#define RTL8372N_SDS_MODE_10GBASER	0x1a

#define RTL8372N_CHIP_INFO_REG		0x000c
#define RTL8372N_CHIP_INFO_EN		GENMASK(19, 16)
#define RTL8372N_CHIP_INFO_EN_KEY	0xa
#define RTL8372N_CHIP_INFO_VER		GENMASK(31, 28)

#define RTL8372N_PHY_MASK_REG		0x6438
#define RTL8372N_PHY_MASK		GENMASK(8, 0)
#define RTL8372N_PHY_CMD_REG		0x643c
#define RTL8372N_PHY_CMD_EXEC		BIT(0)
#define RTL8372N_PHY_CMD_C45		BIT(1)
#define RTL8372N_PHY_CMD_RW		BIT(2)
#define RTL8372N_PHY_CMD_RW_READ	0
#define RTL8372N_PHY_CMD_RW_WRITE	1
#define RTL8372N_PHY_CMD_REGNUM		GENMASK(18, 3)
#define RTL8372N_PHY_CMD_DEVAD		GENMASK(23, 19)
#define RTL8372N_PHY_CMD_FAIL		GENMASK(26, 24)
#define RTL8372N_PHY_ADDR_MAX		8
#define RTL8372N_PHY_RDATA_REG		0x6440
#define RTL8372N_PHY_WDATA_REG		0x6444

#define RTL8372N_SMI_POLL_CTRL_REG	0x632c
#define RTL8372N_SMI_POLL_MASK		GENMASK(20, 12)
#define RTL8372N_SMI_MAC_TYPE_REG	0x6330
#define RTL8372N_SMI_MAC_TYPE_P3	GENMASK(7, 6)
#define RTL8372N_SMI_MAC_TYPE_P8	GENMASK(17, 16)
#define RTL8372N_SMI_POLL_SEL_REG	0x6334
#define RTL8372N_SMI_CTRL_REG		0x6454
#define RTL8372N_SMI_CTRL_MDC_EN	GENMASK(14, 12)	/* MDC clock of SMI 0-2 */

#define RTL8372N_SERDES_PORTS		(BIT(3) | BIT(8))
#define RTL8372N_LINK_STATUS_REG	0x63e8	/* link up, one bit per port */
/* Speed of a port, four bits per port */
#define RTL8372N_SPEED_STATUS_REG(port)	(0x63f0 + ((port) / 8) * 4)
#define RTL8372N_SPEED_STATUS(port)	(GENMASK(3, 0) << (((port) % 8) * 4))
#define RTL8372N_SPEED_STATUS_10	0
#define RTL8372N_SPEED_STATUS_100	1
#define RTL8372N_SPEED_STATUS_1000	2
#define RTL8372N_SPEED_STATUS_10000	4
#define RTL8372N_SPEED_STATUS_2500	5
#define RTL8372N_SPEED_STATUS_5000	6
#define RTL8372N_DUPLEX_STATUS_REG	0x63f8	/* full duplex, one bit per port */
#define RTL8372N_INT_PHY_PORTS		GENMASK(7, 4)

#define RTL8372N_NUM_PORTS		9

struct rtl8372n_sds_step {
	u16 field;
	u16 val;
	unsigned int delay_ms;
};

struct rtl8372n_sds_reg {
	u8 page;
	u8 reg;
	u16 val;
};

struct rtl8372n_pcs {
	struct phylink_pcs pcs;
	struct realtek_priv *priv;
	int sds;
	int port;
};

/* Chip data, in realtek_priv::chip_data */
struct rtl8372n {
	struct mutex sds_lock;		/* SerDes access sequences (0x03f8-0x0400) */
	struct mutex mib_lock;		/* MIB access sequences (0x0f60-0x0f68) */
	struct mutex tbl_lock;		/* table access sequences (0x5cac-0x5cd4) */
	u32 chip_ver;
	struct rtl8372n_pcs pcs[2];	/* one per SerDes */
};

/* Wait until the SerDes access unit has finished its last command */
static int rtl8372n_sds_wait(struct realtek_priv *priv)
{
	u32 val;

	return regmap_read_poll_timeout(priv->map, RTL8372N_SDS_CMD_REG, val,
					!(val & RTL8372N_SDS_CMD_EXEC), 10,
					10 * USEC_PER_MSEC);
}

static u32 rtl8372n_sds_prep_cmd(int sds, int page, int reg, int rw)
{
	return FIELD_PREP(RTL8372N_SDS_CMD_INDEX, sds) |
	       FIELD_PREP(RTL8372N_SDS_CMD_PAGE, page) |
	       FIELD_PREP(RTL8372N_SDS_CMD_REGNUM, reg) |
	       FIELD_PREP(RTL8372N_SDS_CMD_RW, rw) |
	       RTL8372N_SDS_CMD_EXEC;
}

static int __rtl8372n_sds_read(struct realtek_priv *priv, int sds, int page,
			       int reg, u16 *val)
{
	struct rtl8372n *chip = priv->chip_data;
	u32 data;
	int ret;

	lockdep_assert_held(&chip->sds_lock);

	ret = rtl8372n_sds_wait(priv);
	if (ret)
		return ret;
	ret = regmap_write(priv->map, RTL8372N_SDS_CMD_REG,
			   rtl8372n_sds_prep_cmd(sds, page, reg,
						 RTL8372N_SDS_CMD_RW_READ));
	if (ret)
		return ret;
	ret = rtl8372n_sds_wait(priv);
	if (ret)
		return ret;
	ret = regmap_read(priv->map, RTL8372N_SDS_RDATA_REG, &data);
	if (!ret)
		*val = lower_16_bits(data);
	return ret;
}

static int __rtl8372n_sds_write(struct realtek_priv *priv, int sds, int page,
				int reg, u16 val)
{
	struct rtl8372n *chip = priv->chip_data;
	int ret;

	lockdep_assert_held(&chip->sds_lock);

	ret = rtl8372n_sds_wait(priv);
	if (ret)
		return ret;
	ret = regmap_write(priv->map, RTL8372N_SDS_WDATA_REG, val);
	if (ret)
		return ret;
	ret = regmap_write(priv->map, RTL8372N_SDS_CMD_REG,
			   rtl8372n_sds_prep_cmd(sds, page, reg,
						 RTL8372N_SDS_CMD_RW_WRITE));
	if (ret)
		return ret;
	return rtl8372n_sds_wait(priv);
}

static int rtl8372n_sds_write(struct realtek_priv *priv, int sds, int page,
			      int reg, u16 val)
{
	struct rtl8372n *chip = priv->chip_data;
	int ret;

	mutex_lock(&chip->sds_lock);
	ret = __rtl8372n_sds_write(priv, sds, page, reg, val);
	mutex_unlock(&chip->sds_lock);
	return ret;
}

static int rtl8372n_sds_update_bits(struct realtek_priv *priv, int sds,
				    int page, int reg, u16 mask, u16 val)
{
	struct rtl8372n *chip = priv->chip_data;
	u16 old;
	int ret;

	mutex_lock(&chip->sds_lock);
	ret = __rtl8372n_sds_read(priv, sds, page, reg, &old);
	if (!ret)
		ret = __rtl8372n_sds_write(priv, sds, page, reg,
					   (old & ~mask) | (val & mask));
	mutex_unlock(&chip->sds_lock);
	return ret;
}

static int rtl8372n_sds_run_steps(struct realtek_priv *priv, int sds,
				  const struct rtl8372n_sds_step *steps,
				  int num_steps)
{
	const struct rtl8372n_sds_step *step;
	int i, ret;

	for (i = 0; i < num_steps; i++) {
		step = &steps[i];
		ret = rtl8372n_sds_update_bits(priv, sds, RTL8372N_SDS_FRC_PAGE,
					       RTL8372N_SDS_FRC_REG, step->field,
					       field_prep(step->field,
							  step->val));
		if (ret)
			return ret;
		msleep(step->delay_ms);
	}
	return 0;
}

static const struct rtl8372n_sds_step rtl8372n_sds_off_steps[] = {
	{ RTL8372N_SDS_FRC_RX, RTL8372N_SDS_FRC_1, 20 },
	{ RTL8372N_SDS_FRC_RX, RTL8372N_SDS_FRC_0, 50 },
	{ RTL8372N_SDS_FRC_PDOWN, RTL8372N_SDS_FRC_0, 20 },
	{ RTL8372N_SDS_FRC_PDOWN, RTL8372N_SDS_FRC_1, 50 },
	{ RTL8372N_SDS_FRC_CMU, RTL8372N_SDS_FRC_1, 20 },
	{ RTL8372N_SDS_FRC_CMU, RTL8372N_SDS_FRC_0, 50 },
};

static const struct rtl8372n_sds_step rtl8372n_sds_on_steps[] = {
	{ RTL8372N_SDS_FRC_CMU, RTL8372N_SDS_FRC_0, 20 },
	{ RTL8372N_SDS_FRC_CMU, RTL8372N_SDS_FRC_1, 50 },
	{ RTL8372N_SDS_FRC_CMU, RTL8372N_SDS_FRC_AUTO, 20 },
	{ RTL8372N_SDS_FRC_PDOWN, RTL8372N_SDS_FRC_1, 20 },
	{ RTL8372N_SDS_FRC_PDOWN, RTL8372N_SDS_FRC_0, 50 },
	{ RTL8372N_SDS_FRC_PDOWN, RTL8372N_SDS_FRC_AUTO, 20 },
	{ RTL8372N_SDS_FRC_RX, RTL8372N_SDS_FRC_0, 20 },
	{ RTL8372N_SDS_FRC_RX, RTL8372N_SDS_FRC_1, 50 },
	{ RTL8372N_SDS_FRC_RX, RTL8372N_SDS_FRC_AUTO, 50 },
};

static int rtl8372n_sds_off(struct realtek_priv *priv, int sds)
{
	return rtl8372n_sds_run_steps(priv, sds, rtl8372n_sds_off_steps,
				      ARRAY_SIZE(rtl8372n_sds_off_steps));
}

static int rtl8372n_sds_on(struct realtek_priv *priv, int sds)
{
	return rtl8372n_sds_run_steps(priv, sds, rtl8372n_sds_on_steps,
				      ARRAY_SIZE(rtl8372n_sds_on_steps));
}

static int rtl8372n_sds_set_mode(struct realtek_priv *priv, int sds, u32 mode)
{
	u32 mode_mask = sds ? RTL8372N_SDS_MODE_SDS1 : RTL8372N_SDS_MODE_SDS0;
	u32 sub_mask = sds ? RTL8372N_SDS_MODE_SDS1_USX_SUB :
			     RTL8372N_SDS_MODE_SDS0_USX_SUB;
	u32 mac_8221b = sds ? RTL8372N_SDS_MODE_MAC8_8221B :
			      RTL8372N_SDS_MODE_MAC3_8221B;

	return regmap_update_bits(priv->map, RTL8372N_SDS_MODE_REG,
				  mode_mask | sub_mask | mac_8221b,
				  field_prep(mode_mask, mode));
}

/* Vendor values for 10.3125 Gbaud (10GBASE-R, USXGMII); meaning undocumented */
static const struct rtl8372n_sds_reg rtl8372n_sds_10g_a[] = {
	{ 0x21, 0x10, 0x4480 }, { 0x21, 0x13, 0x0400 }, { 0x21, 0x18, 0x6d02 },
	{ 0x21, 0x1b, 0x424e }, { 0x21, 0x1d, 0x0002 }, { 0x36, 0x1c, 0x1390 },
	{ 0x2e, 0x04, 0x0080 }, { 0x2e, 0x06, 0x0408 }, { 0x2e, 0x07, 0x020d },
	{ 0x2e, 0x09, 0x0601 }, { 0x2e, 0x0b, 0x222c }, { 0x2e, 0x0c, 0x9217 },
	{ 0x2e, 0x16, 0x0743 }, { 0x2e, 0x1d, 0xabb0 },
};

static const struct rtl8372n_sds_reg rtl8372n_sds_10g_b[] = {
	{ 0x21, 0x10, 0x4480 }, { 0x21, 0x13, 0x0400 }, { 0x21, 0x18, 0x6d02 },
	{ 0x21, 0x1b, 0x424e }, { 0x21, 0x1d, 0x0002 }, { 0x36, 0x1c, 0x1390 },
	{ 0x36, 0x14, 0x003f }, { 0x36, 0x10, 0x0200 }, { 0x2e, 0x04, 0x0080 },
	{ 0x2e, 0x06, 0x0408 }, { 0x2e, 0x07, 0x020d }, { 0x2e, 0x09, 0x0601 },
	{ 0x2e, 0x0b, 0x222c }, { 0x2e, 0x0c, 0xa217 }, { 0x2e, 0x0d, 0xfe40 },
	{ 0x2e, 0x15, 0xf5c1 }, { 0x2e, 0x16, 0x0443 }, { 0x2e, 0x1d, 0xabb0 },
};

/* Vendor values for the switch side of the link; meaning undocumented */
static const struct rtl8372n_sds_reg rtl8372n_sds_mac[] = {
	{ 0x06, 0x12, 0x5078 }, { 0x07, 0x06, 0x9401 }, { 0x07, 0x08, 0x9401 },
	{ 0x07, 0x0a, 0x9401 }, { 0x07, 0x0c, 0x9401 }, { 0x1f, 0x0b, 0x0003 },
	{ 0x06, 0x03, 0xc45c }, { 0x06, 0x1f, 0x2100 },
};

static int rtl8372n_sds_write_regs(struct realtek_priv *priv, int sds,
				   const struct rtl8372n_sds_reg *regs,
				   int num_regs)
{
	int i, ret;

	for (i = 0; i < num_regs; i++) {
		ret = rtl8372n_sds_write(priv, sds, regs[i].page, regs[i].reg,
					 regs[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

static int rtl8372n_sds_tune_10g(struct realtek_priv *priv, int sds)
{
	const struct rtl8372n_sds_reg *regs = rtl8372n_sds_10g_b;
	int num_regs = ARRAY_SIZE(rtl8372n_sds_10g_b);
	struct rtl8372n *chip = priv->chip_data;
	int ret;

	if (chip->chip_ver == 0) {
		regs = rtl8372n_sds_10g_a;
		num_regs = ARRAY_SIZE(rtl8372n_sds_10g_a);
	}
	ret = rtl8372n_sds_write_regs(priv, sds, regs, num_regs);
	if (ret)
		return ret;

	return rtl8372n_sds_write_regs(priv, sds, rtl8372n_sds_mac,
				       ARRAY_SIZE(rtl8372n_sds_mac));
}

/* Vendor values that change a SerDes in USXGMII mode from Realtek's variant of
 * USXGMII, the reset default, to the standard one; meaning undocumented
 */
static const struct rtl8372n_sds_reg rtl8372n_sds_usxgmii_std[] = {
	{ 0x06, 0x0d, 0x0f00 }, { 0x06, 0x1d, 0x0600 }, { 0x07, 0x06, 0x1401 },
	{ 0x07, 0x08, 0x1401 }, { 0x07, 0x0a, 0x1401 }, { 0x07, 0x0c, 0x1401 },
	{ 0x06, 0x12, 0x00a4 }, { 0x06, 0x13, 0x0000 }, { 0x06, 0x14, 0x0000 },
	{ 0x06, 0x15, 0x0000 }, { 0x06, 0x16, 0x0000 }, { 0x06, 0x17, 0x0000 },
	{ 0x06, 0x18, 0x0000 },
};

static int rtl8372n_sds_usxgmii_standard(struct realtek_priv *priv, int sds)
{
	int ret;

	ret = rtl8372n_sds_write_regs(priv, sds, rtl8372n_sds_usxgmii_std,
				      ARRAY_SIZE(rtl8372n_sds_usxgmii_std));
	if (ret)
		return ret;

	return rtl8372n_sds_update_bits(priv, sds, RTL8372N_SDS_USX_AN_PAGE,
					RTL8372N_SDS_USX_AN_REG,
					RTL8372N_SDS_USX_AN_OPCODE,
					RTL8372N_SDS_USX_AN_OPCODE_STD);
}

static struct rtl8372n_pcs *to_rtl8372n_pcs(struct phylink_pcs *pcs)
{
	return container_of(pcs, struct rtl8372n_pcs, pcs);
}

static int rtl8372n_pcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			       phy_interface_t interface,
			       const unsigned long *advertising,
			       bool permit_pause_to_mac)
{
	struct rtl8372n_pcs *rpcs = to_rtl8372n_pcs(pcs);
	struct realtek_priv *priv = rpcs->priv;
	u32 mode;
	int ret;

	switch (interface) {
	case PHY_INTERFACE_MODE_10GBASER:
		mode = RTL8372N_SDS_MODE_10GBASER;
		break;
	case PHY_INTERFACE_MODE_USXGMII:
		mode = RTL8372N_SDS_MODE_USXGMII;
		break;
	default:
		return -EOPNOTSUPP;
	}

	ret = rtl8372n_sds_off(priv, rpcs->sds);
	if (ret)
		return ret;
	ret = rtl8372n_sds_set_mode(priv, rpcs->sds, mode);
	if (ret)
		return ret;
	ret = rtl8372n_sds_tune_10g(priv, rpcs->sds);
	if (ret)
		return ret;
	if (interface == PHY_INTERFACE_MODE_USXGMII) {
		u16 an = 0;

		if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED)
			an = RTL8372N_SDS_USX_AN_EN;

		ret = rtl8372n_sds_update_bits(priv, rpcs->sds,
					       RTL8372N_SDS_USX_AN_PAGE,
					       RTL8372N_SDS_USX_AN_EN_REG,
					       RTL8372N_SDS_USX_AN_EN, an);
		if (ret)
			return ret;
		ret = rtl8372n_sds_usxgmii_standard(priv, rpcs->sds);
		if (ret)
			return ret;
	}
	msleep(250);

	return rtl8372n_sds_on(priv, rpcs->sds);
}

static unsigned int rtl8372n_pcs_inband_caps(struct phylink_pcs *pcs,
					     phy_interface_t interface)
{
	/* 10GBASE-R has no in-band negotiation */
	if (interface == PHY_INTERFACE_MODE_10GBASER)
		return LINK_INBAND_DISABLE;
	/* The speed of a USXGMII link is only known from the negotiation */
	if (interface == PHY_INTERFACE_MODE_USXGMII)
		return LINK_INBAND_ENABLE;

	return 0;
}

static void rtl8372n_pcs_get_state(struct phylink_pcs *pcs,
				   unsigned int neg_mode,
				   struct phylink_link_state *state)
{
	struct rtl8372n_pcs *rpcs = to_rtl8372n_pcs(pcs);
	u32 val;

	state->link = false;
	if (regmap_read(rpcs->priv->map, RTL8372N_LINK_STATUS_REG, &val))
		return;

	state->link = !!(val & BIT(rpcs->port));
	state->speed = SPEED_10000;
	state->duplex = DUPLEX_FULL;

	/* A 10GBASE-R link has one speed; USXGMII negotiates it */
	if (state->interface != PHY_INTERFACE_MODE_USXGMII || !state->link)
		return;

	state->an_complete = true;
	state->speed = SPEED_UNKNOWN;
	state->duplex = DUPLEX_UNKNOWN;

	if (regmap_read(rpcs->priv->map, RTL8372N_DUPLEX_STATUS_REG, &val))
		return;
	state->duplex = val & BIT(rpcs->port) ? DUPLEX_FULL : DUPLEX_HALF;

	if (regmap_read(rpcs->priv->map,
			RTL8372N_SPEED_STATUS_REG(rpcs->port), &val))
		return;

	switch (field_get(RTL8372N_SPEED_STATUS(rpcs->port), val)) {
	case RTL8372N_SPEED_STATUS_10:
		state->speed = SPEED_10;
		break;
	case RTL8372N_SPEED_STATUS_100:
		state->speed = SPEED_100;
		break;
	case RTL8372N_SPEED_STATUS_1000:
		state->speed = SPEED_1000;
		break;
	case RTL8372N_SPEED_STATUS_2500:
		state->speed = SPEED_2500;
		break;
	case RTL8372N_SPEED_STATUS_5000:
		state->speed = SPEED_5000;
		break;
	case RTL8372N_SPEED_STATUS_10000:
		state->speed = SPEED_10000;
		break;
	}
}

static const struct phylink_pcs_ops rtl8372n_pcs_ops = {
	.pcs_inband_caps = rtl8372n_pcs_inband_caps,
	.pcs_get_state = rtl8372n_pcs_get_state,
	.pcs_config = rtl8372n_pcs_config,
};

static int rtl8372n_phy_wait(struct realtek_priv *priv)
{
	u32 cmd;
	int ret;

	ret = regmap_read_poll_timeout(priv->map, RTL8372N_PHY_CMD_REG, cmd,
				       !(cmd & RTL8372N_PHY_CMD_EXEC), 10,
				       10 * USEC_PER_MSEC);
	if (ret)
		return ret;

	return cmd & RTL8372N_PHY_CMD_FAIL ? -EIO : 0;
}

static u32 rtl8372n_phy_prep_cmd(int devad, int reg, int rw)
{
	return FIELD_PREP(RTL8372N_PHY_CMD_DEVAD, devad) |
	       FIELD_PREP(RTL8372N_PHY_CMD_REGNUM, reg) |
	       FIELD_PREP(RTL8372N_PHY_CMD_RW, rw) |
	       RTL8372N_PHY_CMD_C45 | RTL8372N_PHY_CMD_EXEC;
}

/* Called with the user MDIO bus lock held, which serializes the sequence */
static int rtl8372n_phy_read_c45(struct realtek_priv *priv, int addr,
				 int devad, int reg)
{
	u32 val;
	int ret;

	if (addr > RTL8372N_PHY_ADDR_MAX)
		return -EINVAL;

	ret = regmap_write(priv->map, RTL8372N_PHY_WDATA_REG, addr);
	if (ret)
		return ret;
	ret = regmap_write(priv->map, RTL8372N_PHY_CMD_REG,
			   rtl8372n_phy_prep_cmd(devad, reg,
						 RTL8372N_PHY_CMD_RW_READ));
	if (ret)
		return ret;
	ret = rtl8372n_phy_wait(priv);
	if (ret)
		return ret;
	ret = regmap_read(priv->map, RTL8372N_PHY_RDATA_REG, &val);
	return ret ?: lower_16_bits(val);
}

static int rtl8372n_phy_write_c45(struct realtek_priv *priv, int addr,
				  int devad, int reg, u16 val)
{
	int ret;

	if (addr > RTL8372N_PHY_ADDR_MAX)
		return -EINVAL;

	ret = regmap_write(priv->map, RTL8372N_PHY_MASK_REG,
			   FIELD_PREP(RTL8372N_PHY_MASK, BIT(addr)));
	if (ret)
		return ret;
	ret = regmap_write(priv->map, RTL8372N_PHY_WDATA_REG, val);
	if (ret)
		return ret;
	ret = regmap_write(priv->map, RTL8372N_PHY_CMD_REG,
			   rtl8372n_phy_prep_cmd(devad, reg,
						 RTL8372N_PHY_CMD_RW_WRITE));
	if (ret)
		return ret;

	return rtl8372n_phy_wait(priv);
}

/* Ports 3 and 8 connect through SerDes 0 and 1 */
static bool rtl8372n_is_serdes_port(int port)
{
	return port == 3 || port == 8;
}

static int rtl8372n_port_to_sds(int port)
{
	return port == 8 ? 1 : 0;
}

/* Ports 4-7 have the chip's internal PHYs */
static bool rtl8372n_has_internal_phy(int port)
{
	return port >= 4 && port <= 7;
}

static enum dsa_tag_protocol
rtl8372n_get_tag_protocol(struct dsa_switch *ds, int port,
			  enum dsa_tag_protocol mprot)
{
	return DSA_TAG_PROTO_RTL8_4;
}

static int rtl8372n_soft_reset(struct realtek_priv *priv)
{
	u32 val;
	int ret;

	ret = regmap_write(priv->map, RTL8372N_RESET_REG, RTL8372N_RESET_SOFT);
	if (ret)
		return ret;

	msleep(250);

	return regmap_read_poll_timeout(priv->map, RTL8372N_RESET_REG, val,
					!(val & RTL8372N_RESET_SOFT),
					20 * USEC_PER_MSEC, USEC_PER_SEC);
}

static int rtl8372n_setup_isolation(struct realtek_priv *priv)
{
	struct dsa_port *dp;
	int ret;

	/* port_enable opens the path between a user port and the CPU port */
	dsa_switch_for_each_port(dp, &priv->ds) {
		ret = regmap_write(priv->map,
				   RTL8372N_PORT_ISOLATION_REG(dp->index), 0);
		if (ret)
			return ret;
	}

	return 0;
}

static int rtl8372n_setup_cpu_port(struct realtek_priv *priv)
{
	u32 cpu_ports = dsa_cpu_ports(&priv->ds);
	int cpu, ret;

	if (hweight32(cpu_ports) != 1) {
		dev_err(priv->dev, "exactly one CPU port is supported\n");
		return -EINVAL;
	}
	cpu = __ffs(cpu_ports);

	ret = regmap_update_bits(priv->map, RTL8372N_EXT_CPU_PORT_REG,
				 RTL8372N_EXT_CPU_PORT,
				 FIELD_PREP(RTL8372N_EXT_CPU_PORT, cpu));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->map, RTL8372N_CPU_CTRL_REG,
				 RTL8372N_CPU_CTRL_INSERT_MODE |
				 RTL8372N_CPU_CTRL_TAG_EN,
				 FIELD_PREP(RTL8372N_CPU_CTRL_INSERT_MODE,
					    RTL8372N_CPU_CTRL_INSERT_ALL) |
				 RTL8372N_CPU_CTRL_TAG_EN);
	if (ret)
		return ret;

	return regmap_update_bits(priv->map, RTL8372N_CPU_TAG_AWARE_REG,
				  BIT(cpu), BIT(cpu));
}

static int rtl8372n_setup_learning(struct realtek_priv *priv)
{
	struct dsa_port *dp;
	int ret;

	ret = regmap_update_bits(priv->map, RTL8372N_L2_LIMIT_ACTION_REG,
				 RTL8372N_L2_LIMIT_ACTION,
				 FIELD_PREP(RTL8372N_L2_LIMIT_ACTION,
					    RTL8372N_L2_LIMIT_ACTION_FORWARD));
	if (ret)
		return ret;

	dsa_switch_for_each_port(dp, &priv->ds) {
		ret = regmap_update_bits(priv->map,
					 RTL8372N_L2_LEARN_LIMIT_REG(dp->index),
					 RTL8372N_L2_LEARN_LIMIT, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static int rtl8372n_setup_polling(struct realtek_priv *priv)
{
	int ret;

	ret = regmap_update_bits(priv->map, RTL8372N_SMI_MAC_TYPE_REG,
				 RTL8372N_SMI_MAC_TYPE_P3 |
				 RTL8372N_SMI_MAC_TYPE_P8, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->map, RTL8372N_SMI_POLL_SEL_REG,
				 RTL8372N_INT_PHY_PORTS,
				 RTL8372N_INT_PHY_PORTS);
	if (ret)
		return ret;

	return regmap_update_bits(priv->map, RTL8372N_SMI_POLL_CTRL_REG,
				  RTL8372N_SMI_POLL_MASK,
				  FIELD_PREP(RTL8372N_SMI_POLL_MASK,
					     RTL8372N_SERDES_PORTS |
					     RTL8372N_INT_PHY_PORTS));
}

/* The PHY access unit needs the MDC clock of the MDIO masters */
static int rtl8372n_setup_smi(struct realtek_priv *priv)
{
	return regmap_update_bits(priv->map, RTL8372N_SMI_CTRL_REG,
				  RTL8372N_SMI_CTRL_MDC_EN,
				  RTL8372N_SMI_CTRL_MDC_EN);
}

static int rtl8372n_setup(struct dsa_switch *ds)
{
	struct realtek_priv *priv = ds->priv;
	int ret;

	ret = rtl8372n_soft_reset(priv);
	if (ret) {
		dev_err(ds->dev, "soft reset failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	ret = rtl8372n_setup_isolation(priv);
	if (ret) {
		dev_err(ds->dev, "port isolation failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	ret = rtl8372n_setup_cpu_port(priv);
	if (ret) {
		dev_err(ds->dev, "CPU port setup failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	ret = rtl8372n_setup_learning(priv);
	if (ret) {
		dev_err(ds->dev, "learning setup failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	ret = rtl8372n_setup_polling(priv);
	if (ret) {
		dev_err(ds->dev, "polling setup failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	ret = rtl8372n_setup_smi(priv);
	if (ret) {
		dev_err(ds->dev, "MDIO master setup failed: %pe\n", ERR_PTR(ret));
		return ret;
	}

	ret = rtl83xx_setup_user_mdio(ds);
	if (ret)
		dev_err(ds->dev, "MDIO bus setup failed: %pe\n", ERR_PTR(ret));

	return ret;
}

static void rtl8372n_phylink_get_caps(struct dsa_switch *ds, int port,
				      struct phylink_config *config)
{
	if (rtl8372n_has_internal_phy(port)) {
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		config->mac_capabilities = MAC_10 | MAC_100 | MAC_1000FD |
					   MAC_2500FD | MAC_SYM_PAUSE |
					   MAC_ASYM_PAUSE;
	} else if (rtl8372n_is_serdes_port(port)) {
		__set_bit(PHY_INTERFACE_MODE_10GBASER,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_USXGMII,
			  config->supported_interfaces);
		config->mac_capabilities = MAC_10FD | MAC_100FD | MAC_1000FD |
					   MAC_2500FD | MAC_5000FD |
					   MAC_10000FD | MAC_SYM_PAUSE |
					   MAC_ASYM_PAUSE;
	}
}

static struct phylink_pcs *
rtl8372n_mac_select_pcs(struct phylink_config *config,
			phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct realtek_priv *priv = dp->ds->priv;
	struct rtl8372n *chip = priv->chip_data;

	if (!rtl8372n_is_serdes_port(dp->index))
		return NULL;

	return &chip->pcs[rtl8372n_port_to_sds(dp->index)].pcs;
}

static void rtl8372n_mac_config(struct phylink_config *config,
				unsigned int mode,
				const struct phylink_link_state *state)
{
}

static void rtl8372n_mac_link_down(struct phylink_config *config,
				   unsigned int mode,
				   phy_interface_t interface)
{
}

static void rtl8372n_mac_link_up(struct phylink_config *config,
				 struct phy_device *phy, unsigned int mode,
				 phy_interface_t interface, int speed,
				 int duplex, bool tx_pause, bool rx_pause)
{
}

static const struct phylink_mac_ops rtl8372n_phylink_mac_ops = {
	.mac_select_pcs = rtl8372n_mac_select_pcs,
	.mac_config = rtl8372n_mac_config,
	.mac_link_down = rtl8372n_mac_link_down,
	.mac_link_up = rtl8372n_mac_link_up,
};

static int rtl8372n_port_enable(struct dsa_switch *ds, int port,
				struct phy_device *phy)
{
	int cpu = dsa_upstream_port(ds, port);
	struct realtek_priv *priv = ds->priv;
	int ret;

	if (!dsa_is_user_port(ds, port))
		return 0;

	/* Open the path between the port and the CPU port. The other bits of
	 * the port's mask belong to the bridge the port may be in.
	 */
	ret = regmap_update_bits(priv->map, RTL8372N_PORT_ISOLATION_REG(port),
				 BIT(cpu), BIT(cpu));
	if (ret)
		return ret;

	return regmap_update_bits(priv->map, RTL8372N_PORT_ISOLATION_REG(cpu),
				  BIT(port), BIT(port));
}

static void rtl8372n_port_disable(struct dsa_switch *ds, int port)
{
	int cpu = dsa_upstream_port(ds, port);
	struct realtek_priv *priv = ds->priv;
	int ret;

	if (!dsa_is_user_port(ds, port))
		return;

	ret = regmap_update_bits(priv->map, RTL8372N_PORT_ISOLATION_REG(port),
				 BIT(cpu), 0);
	if (!ret)
		ret = regmap_update_bits(priv->map,
					 RTL8372N_PORT_ISOLATION_REG(cpu),
					 BIT(port), 0);
	if (ret)
		dev_err(priv->dev, "cannot isolate port %d: %pe\n", port,
			ERR_PTR(ret));
}

static int rtl8372n_port_add_isolation(struct realtek_priv *priv, int port,
				       u32 mask)
{
	return regmap_update_bits(priv->map, RTL8372N_PORT_ISOLATION_REG(port),
				  mask, mask);
}

static int rtl8372n_port_remove_isolation(struct realtek_priv *priv, int port,
					  u32 mask)
{
	return regmap_update_bits(priv->map, RTL8372N_PORT_ISOLATION_REG(port),
				  mask, 0);
}

/* Ports learn addresses only while they are in a bridge */
static int rtl8372n_port_set_learning(struct realtek_priv *priv, int port,
				      bool enable)
{
	u32 limit = enable ? RTL8372N_L2_TABLE_SIZE : 0;

	return regmap_update_bits(priv->map, RTL8372N_L2_LEARN_LIMIT_REG(port),
				  RTL8372N_L2_LEARN_LIMIT,
				  FIELD_PREP(RTL8372N_L2_LEARN_LIMIT, limit));
}

static int rtl8372n_l2_flush_wait(struct realtek_priv *priv)
{
	u32 val;

	return regmap_read_poll_timeout(priv->map, RTL8372N_L2_FLUSH_CMD_REG,
					val,
					!(val & RTL8372N_L2_FLUSH_CMD_BUSY),
					100, 100 * USEC_PER_MSEC);
}

static void rtl8372n_port_fast_age(struct dsa_switch *ds, int port)
{
	struct realtek_priv *priv = ds->priv;
	int ret;

	ret = rtl8372n_l2_flush_wait(priv);
	if (ret)
		goto err;

	ret = regmap_update_bits(priv->map, RTL8372N_L2_FLUSH_MODE_REG,
				 RTL8372N_L2_FLUSH_MODE,
				 FIELD_PREP(RTL8372N_L2_FLUSH_MODE,
					    RTL8372N_L2_FLUSH_MODE_PORT));
	if (ret)
		goto err;

	ret = regmap_update_bits(priv->map, RTL8372N_L2_FLUSH_CMD_REG,
				 RTL8372N_L2_FLUSH_CMD_PORTS, BIT(port));
	if (ret)
		goto err;

	ret = regmap_update_bits(priv->map, RTL8372N_L2_FLUSH_CMD_REG,
				 RTL8372N_L2_FLUSH_CMD_START,
				 RTL8372N_L2_FLUSH_CMD_START);
	if (ret)
		goto err;

	ret = rtl8372n_l2_flush_wait(priv);
	if (ret)
		goto err;

	return;

err:
	dev_err(priv->dev, "cannot flush the addresses of port %d: %pe\n", port,
		ERR_PTR(ret));
}

static int rtl8372n_tbl_wait(struct realtek_priv *priv)
{
	u32 val;

	return regmap_read_poll_timeout(priv->map, RTL8372N_TBL_CMD_REG, val,
					!(val & RTL8372N_TBL_CMD_EXEC),
					100, 100 * USEC_PER_MSEC);
}

/* Read the first unicast entry of a port at or after *index. Returns -ENOENT
 * if there is none.
 */
static int rtl8372n_l2_read_next(struct realtek_priv *priv, int port,
				 u32 *index, u32 *data)
{
	struct rtl8372n *chip = priv->chip_data;
	int ret, i;
	u32 val;

	mutex_lock(&chip->tbl_lock);

	ret = rtl8372n_tbl_wait(priv);
	if (ret)
		goto out;

	ret = regmap_update_bits(priv->map, RTL8372N_L2_LOOKUP_REG,
				 RTL8372N_L2_LOOKUP_METHOD |
				 RTL8372N_L2_LOOKUP_PORT,
				 FIELD_PREP(RTL8372N_L2_LOOKUP_METHOD,
					    RTL8372N_L2_LOOKUP_METHOD_NEXT_UC_PORT) |
				 FIELD_PREP(RTL8372N_L2_LOOKUP_PORT, port));
	if (ret)
		goto out;

	ret = regmap_write(priv->map, RTL8372N_TBL_CMD_REG,
			   FIELD_PREP(RTL8372N_TBL_CMD_INDEX, *index) |
			   FIELD_PREP(RTL8372N_TBL_CMD_TYPE,
				      RTL8372N_TBL_CMD_TYPE_L2) |
			   RTL8372N_TBL_CMD_EXEC);
	if (ret)
		goto out;

	ret = rtl8372n_tbl_wait(priv);
	if (ret)
		goto out;

	ret = regmap_read(priv->map, RTL8372N_L2_LOOKUP_REG, &val);
	if (ret)
		goto out;

	if (!(val & RTL8372N_L2_LOOKUP_HIT)) {
		ret = -ENOENT;
		goto out;
	}
	*index = FIELD_GET(RTL8372N_L2_LOOKUP_INDEX, val);

	for (i = 0; i < RTL8372N_L2_ENTRY_WORDS; i++) {
		ret = regmap_read(priv->map, RTL8372N_TBL_RDATA_REG(i),
				  &data[i]);
		if (ret)
			goto out;
	}

out:
	mutex_unlock(&chip->tbl_lock);

	return ret;
}

static int rtl8372n_port_fdb_dump(struct dsa_switch *ds, int port,
				  dsa_fdb_dump_cb_t *cb, void *data)
{
	struct realtek_priv *priv = ds->priv;
	u32 entry[RTL8372N_L2_ENTRY_WORDS];
	u8 addr[ETH_ALEN];
	u32 index = 0;
	u32 found;
	u16 vid;
	int ret;

	while (index < RTL8372N_L2_TABLE_SIZE) {
		found = index;
		ret = rtl8372n_l2_read_next(priv, port, &found, entry);
		if (ret == -ENOENT)
			break;
		if (ret)
			return ret;

		/* After the last entry the search starts again at the first */
		if (found < index)
			break;

		put_unaligned_be16(FIELD_GET(RTL8372N_L2_UC_D1_MAC_HI,
					     entry[1]), &addr[0]);
		put_unaligned_be32(entry[0], &addr[2]);

		vid = 0;
		if (entry[1] & RTL8372N_L2_UC_D1_IVL)
			vid = FIELD_GET(RTL8372N_L2_UC_D1_VID, entry[1]);

		ret = cb(addr, vid, entry[2] & RTL8372N_L2_UC_D2_STATIC, data);
		if (ret)
			return ret;

		index = found + 1;
	}

	return 0;
}

static void rtl8372n_port_stp_state_set(struct dsa_switch *ds, int port,
					u8 state)
{
	struct realtek_priv *priv = ds->priv;
	int fid, ret;
	u32 val;

	switch (state) {
	case BR_STATE_DISABLED:
		val = RTL8372N_STP_STATE_DISABLED;
		break;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		val = RTL8372N_STP_STATE_BLOCKING;
		break;
	case BR_STATE_LEARNING:
		val = RTL8372N_STP_STATE_LEARNING;
		break;
	case BR_STATE_FORWARDING:
		val = RTL8372N_STP_STATE_FORWARDING;
		break;
	default:
		dev_err(priv->dev, "invalid STP state %u for port %d\n", state,
			port);
		return;
	}

	for (fid = 0; fid < RTL8372N_NUM_FIDS; fid++) {
		ret = regmap_update_bits(priv->map,
					 RTL8372N_STP_STATE_REG(fid),
					 RTL8372N_STP_STATE_MASK(port),
					 val << (port * 2));
		if (ret) {
			dev_err(priv->dev, "cannot set STP state of port %d: %pe\n",
				port, ERR_PTR(ret));
			return;
		}
	}
}

static int rtl8372n_port_change_mtu(struct dsa_switch *ds, int port,
				    int new_mtu)
{
	u32 len = new_mtu + VLAN_ETH_HLEN + ETH_FCS_LEN;
	struct realtek_priv *priv = ds->priv;

	if (len > RTL8372N_PORT_MAX_LEN_MAX)
		return -EINVAL;

	/* Both fields get the same length: the limit is the same at any speed */
	return regmap_update_bits(priv->map, RTL8372N_PORT_MAX_LEN_REG(port),
				  RTL8372N_PORT_MAX_LEN_FE |
				  RTL8372N_PORT_MAX_LEN_GE,
				  FIELD_PREP(RTL8372N_PORT_MAX_LEN_FE, len) |
				  FIELD_PREP(RTL8372N_PORT_MAX_LEN_GE, len));
}

static int rtl8372n_port_max_mtu(struct dsa_switch *ds, int port)
{
	return RTL8372N_PORT_MAX_LEN_MAX - VLAN_ETH_HLEN - ETH_FCS_LEN;
}

/*
 * A counter number selects a 64-bit slot that holds one 64-bit counter or
 * two 32-bit counters (even word = high half, odd word = low half).
 */
struct rtl8372n_mib {
	u8 word;	/* position in 32-bit words */
	u8 words;	/* 1 or 2 */
};

enum {
	RTL8372N_MIB_IN_OCTETS,
	RTL8372N_MIB_OUT_OCTETS,
	RTL8372N_MIB_IN_MCAST,
	RTL8372N_MIB_IN_BCAST,
	RTL8372N_MIB_OUT_MCAST,
	RTL8372N_MIB_OUT_BCAST,
	RTL8372N_MIB_SINGLE_COLL,
	RTL8372N_MIB_MULTI_COLL,
	RTL8372N_MIB_DEFERRED,
	RTL8372N_MIB_LATE_COLL,
	RTL8372N_MIB_EXCESS_COLL,
	RTL8372N_MIB_RX_CRC_ERR,
	RTL8372N_MIB_TX_GOOD,
	RTL8372N_MIB_RX_GOOD,
	RTL8372N_MIB_NUM
};

static const struct rtl8372n_mib rtl8372n_mibs[RTL8372N_MIB_NUM] = {
	[RTL8372N_MIB_IN_OCTETS]	= {  0, 2 },
	[RTL8372N_MIB_OUT_OCTETS]	= {  2, 2 },
	[RTL8372N_MIB_IN_MCAST]		= {  6, 2 },
	[RTL8372N_MIB_IN_BCAST]		= {  8, 2 },
	[RTL8372N_MIB_OUT_MCAST]	= { 12, 2 },
	[RTL8372N_MIB_OUT_BCAST]	= { 14, 2 },
	[RTL8372N_MIB_SINGLE_COLL]	= { 18, 1 },
	[RTL8372N_MIB_MULTI_COLL]	= { 19, 1 },
	[RTL8372N_MIB_DEFERRED]		= { 20, 1 },
	[RTL8372N_MIB_LATE_COLL]	= { 21, 1 },
	[RTL8372N_MIB_EXCESS_COLL]	= { 22, 1 },
	[RTL8372N_MIB_RX_CRC_ERR]	= { 31, 1 },
	[RTL8372N_MIB_TX_GOOD]		= { 92, 2 },
	[RTL8372N_MIB_RX_GOOD]		= { 94, 2 },
};

static int rtl8372n_mib_read(struct realtek_priv *priv, int port,
			     const struct rtl8372n_mib *mib, u64 *val)
{
	u32 cmd, hi, lo;
	int ret;

	ret = regmap_write(priv->map, RTL8372N_MIB_CMD_REG,
			   FIELD_PREP(RTL8372N_MIB_CMD_COUNTER, mib->word / 2) |
			   FIELD_PREP(RTL8372N_MIB_CMD_PORT, port) |
			   RTL8372N_MIB_CMD_BUSY);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(priv->map, RTL8372N_MIB_CMD_REG, cmd,
				       !(cmd & RTL8372N_MIB_CMD_BUSY), 10,
				       10 * USEC_PER_MSEC);
	if (ret)
		return ret;

	ret = regmap_read(priv->map, RTL8372N_MIB_DATA_HI_REG, &hi);
	if (ret)
		return ret;
	ret = regmap_read(priv->map, RTL8372N_MIB_DATA_LO_REG, &lo);
	if (ret)
		return ret;

	if (mib->words == 2)
		*val = (u64)hi << 32 | lo;
	else
		*val = mib->word % 2 ? lo : hi;

	return 0;
}

static void rtl8372n_get_eth_mac_stats(struct dsa_switch *ds, int port,
				       struct ethtool_eth_mac_stats *mac_stats)
{
	struct realtek_priv *priv = ds->priv;
	struct rtl8372n *chip = priv->chip_data;
	u64 cnt[RTL8372N_MIB_NUM];
	int i, ret = 0;

	mutex_lock(&chip->mib_lock);
	for (i = 0; i < RTL8372N_MIB_NUM && !ret; i++)
		ret = rtl8372n_mib_read(priv, port, &rtl8372n_mibs[i], &cnt[i]);
	mutex_unlock(&chip->mib_lock);
	if (ret)
		return;

	mac_stats->FramesTransmittedOK = cnt[RTL8372N_MIB_TX_GOOD];
	mac_stats->FramesReceivedOK = cnt[RTL8372N_MIB_RX_GOOD];
	/* The octet counters include header and FCS, the IEEE ones do not */
	mac_stats->OctetsTransmittedOK = cnt[RTL8372N_MIB_OUT_OCTETS] -
		(ETH_HLEN + ETH_FCS_LEN) * cnt[RTL8372N_MIB_TX_GOOD];
	mac_stats->OctetsReceivedOK = cnt[RTL8372N_MIB_IN_OCTETS] -
		(ETH_HLEN + ETH_FCS_LEN) * cnt[RTL8372N_MIB_RX_GOOD];
	mac_stats->MulticastFramesXmittedOK = cnt[RTL8372N_MIB_OUT_MCAST];
	mac_stats->BroadcastFramesXmittedOK = cnt[RTL8372N_MIB_OUT_BCAST];
	mac_stats->MulticastFramesReceivedOK = cnt[RTL8372N_MIB_IN_MCAST];
	mac_stats->BroadcastFramesReceivedOK = cnt[RTL8372N_MIB_IN_BCAST];
	mac_stats->FrameCheckSequenceErrors = cnt[RTL8372N_MIB_RX_CRC_ERR];
	mac_stats->SingleCollisionFrames = cnt[RTL8372N_MIB_SINGLE_COLL];
	mac_stats->MultipleCollisionFrames = cnt[RTL8372N_MIB_MULTI_COLL];
	mac_stats->FramesWithDeferredXmissions = cnt[RTL8372N_MIB_DEFERRED];
	mac_stats->LateCollisions = cnt[RTL8372N_MIB_LATE_COLL];
	mac_stats->FramesAbortedDueToXSColls = cnt[RTL8372N_MIB_EXCESS_COLL];
}

static const struct dsa_switch_ops rtl8372n_switch_ops = {
	.get_tag_protocol = rtl8372n_get_tag_protocol,
	.setup = rtl8372n_setup,
	.phylink_get_caps = rtl8372n_phylink_get_caps,
	.port_enable = rtl8372n_port_enable,
	.port_disable = rtl8372n_port_disable,
	.port_stp_state_set = rtl8372n_port_stp_state_set,
	.port_fast_age = rtl8372n_port_fast_age,
	.port_fdb_dump = rtl8372n_port_fdb_dump,
	.port_bridge_join = rtl83xx_port_bridge_join,
	.port_bridge_leave = rtl83xx_port_bridge_leave,
	.port_change_mtu = rtl8372n_port_change_mtu,
	.port_max_mtu = rtl8372n_port_max_mtu,
	.get_eth_mac_stats = rtl8372n_get_eth_mac_stats,
};

/*
 * The version field is only valid while the enable field holds the key,
 * so set the key, read the register and clear the key again.
 */
static int rtl8372n_read_chip_ver(struct realtek_priv *priv)
{
	struct rtl8372n *chip = priv->chip_data;
	u32 val;
	int ret;

	ret = regmap_update_bits(priv->map, RTL8372N_CHIP_INFO_REG,
				 RTL8372N_CHIP_INFO_EN,
				 FIELD_PREP(RTL8372N_CHIP_INFO_EN,
					    RTL8372N_CHIP_INFO_EN_KEY));
	if (ret)
		return ret;

	ret = regmap_read(priv->map, RTL8372N_CHIP_INFO_REG, &val);
	if (!ret)
		ret = regmap_update_bits(priv->map, RTL8372N_CHIP_INFO_REG,
					 RTL8372N_CHIP_INFO_EN, 0);
	if (ret)
		return ret;

	chip->chip_ver = FIELD_GET(RTL8372N_CHIP_INFO_VER, val);
	return 0;
}

static int rtl8372n_detect(struct realtek_priv *priv)
{
	struct rtl8372n *chip = priv->chip_data;
	int i, ret;
	u32 id;

	ret = regmap_read(priv->map, RTL8372N_CHIP_MODEL_REG, &id);
	if (ret)
		return dev_err_probe(priv->dev, ret, "cannot read chip ID\n");

	if (id != RTL8372N_CHIP_MODEL_RTL8372N) {
		dev_err(priv->dev, "unrecognized switch, chip ID 0x%08x\n", id);
		return -ENODEV;
	}

	ret = devm_mutex_init(priv->dev, &chip->sds_lock);
	if (ret)
		return ret;

	ret = devm_mutex_init(priv->dev, &chip->mib_lock);
	if (ret)
		return ret;

	ret = devm_mutex_init(priv->dev, &chip->tbl_lock);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(chip->pcs); i++) {
		chip->pcs[i].pcs.ops = &rtl8372n_pcs_ops;
		chip->pcs[i].priv = priv;
		chip->pcs[i].sds = i;
		chip->pcs[i].port = i ? 8 : 3;
		__set_bit(PHY_INTERFACE_MODE_10GBASER,
			  chip->pcs[i].pcs.supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_USXGMII,
			  chip->pcs[i].pcs.supported_interfaces);
	}

	ret = rtl8372n_read_chip_ver(priv);
	if (ret)
		return dev_err_probe(priv->dev, ret,
				     "cannot read chip version\n");

	dev_info(priv->dev, "found an RTL8372N switch, version %u\n",
		 chip->chip_ver);

	priv->num_ports = RTL8372N_NUM_PORTS;
	return 0;
}

static const struct realtek_ops rtl8372n_ops = {
	.detect = rtl8372n_detect,
	.phy_read_c45 = rtl8372n_phy_read_c45,
	.phy_write_c45 = rtl8372n_phy_write_c45,
	.port_add_isolation = rtl8372n_port_add_isolation,
	.port_remove_isolation = rtl8372n_port_remove_isolation,
	.port_set_learning = rtl8372n_port_set_learning,
};

static const struct realtek_variant rtl8372n_variant = {
	.ds_ops = &rtl8372n_switch_ops,
	.ops = &rtl8372n_ops,
	.phylink_mac_ops = &rtl8372n_phylink_mac_ops,
	.chip_data_sz = sizeof(struct rtl8372n),
};

static const struct of_device_id rtl8372n_of_match[] = {
	{ .compatible = "realtek,rtl8372n", .data = &rtl8372n_variant },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, rtl8372n_of_match);

static struct mdio_driver rtl8372n_driver = {
	.mdiodrv.driver = {
		.name = "rtl8372n",
		.of_match_table = rtl8372n_of_match,
	},
	.probe = rtl837x_mdio_probe,
	.remove = realtek_mdio_remove,
	.shutdown = realtek_mdio_shutdown,
};

mdio_module_driver(rtl8372n_driver);

MODULE_DESCRIPTION("RTL8372N DSA driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("REALTEK_DSA");
