// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 */

#include <linux/iopoll.h>
#include <dm/device_compat.h>

#include "phy-samsung-ufs.h"

#define EXYNOS9820_PHY_CTRL		0x724
#define EXYNOS9820_PHY_CTRL_MASK	0x1
#define EXYNOS9820_PHY_CTRL_EN		BIT(0)
#define PHY_EXYNOS9820_LANE_OFFSET	0x400

#define PHY_TRSV_REG_CFG_EXYNOS9820(o, v, d) \
	PHY_TRSV_REG_CFG_OFFSET(o, v, d, PHY_EXYNOS9820_LANE_OFFSET)

#define PHY_PMA_TRSV_ADDR(reg, lane)	(PHY_APB_ADDR((reg) + \
					((lane) * PHY_EXYNOS9820_LANE_OFFSET)))

/* Calibration for phy initialization */
static const struct samsung_ufs_phy_cfg exynos9820_pre_init_cfg[] = {
	PHY_TRSV_REG_CFG_EXYNOS9820(0x6B0, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x584, 0x44, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x6E4, 0x10, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x644, 0x05, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x5A4, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x708, 0x01, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0xFC, 0x10, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x6D8, 0x40, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x708, 0x25, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x430, 0x04,  PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x628, 0x13,  PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x67C, 0x19,  PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x688, 0x30,  PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x5A0, 0x40,  PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x5A4, 0x38,  PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x6E4, 0x18,  PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0xFC, 0x18, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0xFC, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg exynos9820_post_init_cfg[] = {
	PHY_COMN_REG_CFG(0xFC, 0x10, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x600, 0x04, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x6D8, 0xC0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x400, 0x00, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x404, 0x06, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x408, 0x06, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x40C, 0x0A, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x410, 0x01, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x414, 0x11, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x414, 0x11, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x0F4, 0x01, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x48C, 0x1C, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x4C8, 0xF0, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x5B0, 0x01, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x7CC, 0x0A, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x7DC, 0x06, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x4E0, 0x56, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x54C, 0xFD, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x64C, 0x0A, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x4DC, 0x7F, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_EXYNOS9820(0x4DC, 0x7F, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0xFC, 0x18, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0xFC, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *exynos9820_ufs_phy_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT]		= exynos9820_pre_init_cfg,
	[CFG_POST_INIT]		= exynos9820_post_init_cfg,
};

static const char * const exynos9820_ufs_phy_clks[] = {
};

static int exynos9820_phy_wait_for_calibration(struct phy *phy, u8 lane)
{
	log_debug("%s: waitomg on lane %hhu ...\n", __func__, lane);
	udelay(0xC8);
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	const unsigned int timeout_us = 40000;
	u32 val;
	u32 off;
	int err;

	off = PHY_PMA_TRSV_ADDR(0x7B4, lane);

	err = readl_poll_timeout(ufs_phy->reg_pma + off,
				 val, (val & 0x10),
				 timeout_us);

	if (err) {
		dev_err(ufs_phy->dev,
			"failed to get phy cal done %d\n", err);
	}
	return err;
}
	

const struct samsung_ufs_phy_drvdata exynos9820_ufs_phy = {
	.cfgs = exynos9820_ufs_phy_cfgs,
	.isol = {
		.offset = EXYNOS9820_PHY_CTRL,
		.mask = EXYNOS9820_PHY_CTRL_MASK,
		.en = EXYNOS9820_PHY_CTRL_EN,
	},
	.clk_list = exynos9820_ufs_phy_clks,
	.num_clks = ARRAY_SIZE(exynos9820_ufs_phy_clks),
	.wait_for_cal = exynos9820_phy_wait_for_calibration,
};
