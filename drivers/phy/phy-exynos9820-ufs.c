// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 */

#include "phy-samsung-ufs.h"

#define EXYNOS9820_PHY_CTRL		0x724
#define EXYNOS9820_PHY_CTRL_MASK	0x1
#define EXYNOS9820_PHY_CTRL_EN		BIT(0)

static const struct samsung_ufs_phy_cfg *exynos9820_ufs_phy_cfgs[CFG_TAG_MAX] = {
};

static const char * const exynos9820_ufs_phy_clks[] = {
};

const struct samsung_ufs_phy_drvdata exynos9820_ufs_phy = {
	.cfgs = exynos9820_ufs_phy_cfgs,
	.isol = {
		.offset = EXYNOS9820_PHY_CTRL,
		.mask = EXYNOS9820_PHY_CTRL_MASK,
		.en = EXYNOS9820_PHY_CTRL_EN,
	},
	.clk_list = exynos9820_ufs_phy_clks,
	.num_clks = ARRAY_SIZE(exynos9820_ufs_phy_clks),
};
