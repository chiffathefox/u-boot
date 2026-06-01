// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS PHY driver for Samsung SoC
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 * Author: Seungwon Jeon <essuuj@gmail.com>
 * Author: Alim Akhtar <alim.akhtar@samsung.com>
 * Author: chiffathefox <chiffathefoxx@gmail.com>
 *
 */

#include <asm/io.h>
#include <clk.h>
#include <dm/devres.h>
#include <dm/device_compat.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <syscon.h>

#include "phy-samsung-ufs.h"

#define for_each_phy_lane(phy, i) \
	for (i = 0; i < (phy)->lane_cnt; i++)
#define for_each_phy_cfg(cfg) \
	for (; (cfg)->id; (cfg)++)

#define PHY_DEF_LANE_CNT	2
 void phy_pma_writel(u32 val, u32 reg);
void samsung_ufs_phy_config(struct samsung_ufs_phy *phy,
			    const struct samsung_ufs_phy_cfg *cfg,
			    u8 lane)
{
	enum {LANE_0, LANE_1}; /* lane index */

	switch (lane) {
	case LANE_0:
		writel(cfg->val, (phy)->reg_pma + cfg->off_0);
		break;
	case LANE_1:
		if (cfg->id == PHY_TRSV_BLK)
			writel(cfg->val, (phy)->reg_pma + cfg->off_1);
		break;
	}
}

int samsung_ufs_phy_wait_for_lock_acq(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	const unsigned int timeout_us = 100000;
	u32 val;
	int err;

	err = readl_poll_timeout(
			ufs_phy->reg_pma + PHY_APB_ADDR(PHY_PLL_LOCK_STATUS),
			val, (val & PHY_PLL_LOCK_BIT), timeout_us);
	if (err) {
		dev_err(ufs_phy->dev,
			"failed to get phy pll lock acquisition %d\n", err);
		goto out;
	}

	err = readl_poll_timeout(
			ufs_phy->reg_pma +
			PHY_APB_ADDR(ufs_phy->drvdata->cdr_lock_status_offset),
			val, (val & PHY_CDR_LOCK_BIT), timeout_us);
	if (err)
		dev_err(ufs_phy->dev,
			"failed to get phy cdr lock acquisition %d\n", err);
out:
	return err;
}

static int samsung_ufs_phy_calibrate(struct phy *phy)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	const struct samsung_ufs_phy_cfg * const *cfgs = ufs_phy->cfgs;
	const struct samsung_ufs_phy_cfg *cfg;
	int err = 0;
	int i;

	if (unlikely(ufs_phy->ufs_phy_state < CFG_PRE_INIT ||
		     ufs_phy->ufs_phy_state >= CFG_TAG_MAX)) {
		dev_err(ufs_phy->dev, "invalid phy config index %d\n", ufs_phy->ufs_phy_state);
		return -EINVAL;
	}

	cfg = cfgs[ufs_phy->ufs_phy_state];
	if (!cfg)
		goto out;

	for_each_phy_cfg(cfg) {
		for_each_phy_lane(ufs_phy, i) {
			samsung_ufs_phy_config(ufs_phy, cfg, i);
		}
	}

	for_each_phy_lane(ufs_phy, i) {
		if (ufs_phy->ufs_phy_state == CFG_PRE_INIT &&
		    ufs_phy->drvdata->wait_for_cal) {
			err = ufs_phy->drvdata->wait_for_cal(phy, i);
			if (err)
				goto out;
		}

		if (ufs_phy->ufs_phy_state == CFG_POST_PWR_HS &&
		    ufs_phy->drvdata->wait_for_cdr) {
			err = ufs_phy->drvdata->wait_for_cdr(phy, i);
			if (err)
				goto out;
		}
	}

	/**
	 * In Samsung ufshci, PHY need to be calibrated at different
	 * stages / state mainly before Linkstartup, after Linkstartup,
	 * before power mode change and after power mode change.
	 * Below state machine to make sure to calibrate PHY in each
	 * state. Here after configuring PHY in a given state, will
	 * change the state to next state so that next state phy
	 * calibration value can be programed
	 */
out:
	switch (ufs_phy->ufs_phy_state) {
	case CFG_PRE_INIT:
		ufs_phy->ufs_phy_state = CFG_POST_INIT;
		break;
	case CFG_POST_INIT:
		ufs_phy->ufs_phy_state = CFG_PRE_PWR_HS;
		break;
	case CFG_PRE_PWR_HS:
		ufs_phy->ufs_phy_state = CFG_POST_PWR_HS;
		break;
	case CFG_POST_PWR_HS:
		/* Change back to INIT state */
		ufs_phy->ufs_phy_state = CFG_PRE_INIT;
		break;
	default:
		dev_err(ufs_phy->dev, "wrong state for phy calibration\n");
	}

	return err;
}

static int samsung_ufs_phy_configure(struct phy *phy, void *param)
{
	(void)param;
	return samsung_ufs_phy_calibrate(phy);
}

static int samsung_ufs_phy_clks_init(struct samsung_ufs_phy *phy)
{
	const struct samsung_ufs_phy_drvdata *drvdata = phy->drvdata;
	int num_clks = drvdata->num_clks;
	int i;
	int ret;
	struct udevice *dev = phy->dev;

	phy->clks = devm_kcalloc(phy->dev, num_clks, sizeof(*phy->clks),
				 GFP_KERNEL);
	if (!phy->clks)
		return -ENOMEM;

	/**
	 * U-Boot's clk_bulk API only works on clocks and clock-names properties in DTB, not on custom lists.
	 * Hence the boilerplate code.
	 */
	for (i = 0; i < num_clks; i++) {
		ret = clk_get_by_name(dev, drvdata->clk_list[i], &phy->clks[i]);
		if (ret) {
			dev_err(dev, "failed to find clock %s\n",
				drvdata->clk_list[i]);
			return ret;
		}
	}

	return 0;
}

static int samsung_ufs_phy_init(struct phy *phy)
{
	struct samsung_ufs_phy *ss_phy = get_samsung_ufs_phy(phy);

	// ss_phy->lane_cnt = phy->attrs.bus_width;
	ss_phy->ufs_phy_state = CFG_PRE_INIT;

	return 0;
}

static int samsung_ufs_phy_power_on(struct phy *phy)
{
	struct samsung_ufs_phy *ss_phy = get_samsung_ufs_phy(phy);
	int ret;
	int i;
	struct clk *clk;

	samsung_ufs_phy_ctrl_isol(ss_phy, false);

	for (i = 0; i < ss_phy->drvdata->num_clks; i++) {
		clk = &ss_phy->clks[i];
		ret = clk_enable(clk);
		if (ret) {
			dev_err(ss_phy->dev,
				"failed to enable phy clock %s (%lu)\n",
				clk->dev->name, clk->id);
			return ret;
		}
	}
// TODO: ufs: cleanup
	// if (ss_phy->ufs_phy_state == CFG_PRE_INIT) {
	// 	ret = samsung_ufs_phy_calibrate(phy);
	// 	if (ret)
	// 		dev_err(ss_phy->dev, "ufs phy calibration failed\n");
	// }

	return ret;
}

static int samsung_ufs_phy_power_off(struct phy *phy)
{
	struct samsung_ufs_phy *ss_phy = get_samsung_ufs_phy(phy);
	int err;

	err = clk_release_all(ss_phy->clks,
			      (unsigned int)ss_phy->drvdata->num_clks);
	if (err) {
		dev_err(ss_phy->dev, "failed to release clocks, err = %d\n",
			err);
	}

	samsung_ufs_phy_ctrl_isol(ss_phy, true);

	return 0;
}

static int samsung_ufs_phy_set_mode(struct phy *generic_phy,
				    enum phy_mode mode, int submode)
{
	struct samsung_ufs_phy *ss_phy = get_samsung_ufs_phy(generic_phy);

	ss_phy->mode = PHY_MODE_INVALID;

	if (mode > 0)
		ss_phy->mode = mode;

	return 0;
}

static int samsung_ufs_phy_exit(struct phy *phy)
{
	struct samsung_ufs_phy *ss_phy = get_samsung_ufs_phy(phy);

	ss_phy->ufs_phy_state = CFG_TAG_MAX;

	return 0;
}

static const struct phy_ops samsung_ufs_phy_ops = {
	.init		= samsung_ufs_phy_init,
	.exit		= samsung_ufs_phy_exit,
	.power_on	= samsung_ufs_phy_power_on,
	.power_off	= samsung_ufs_phy_power_off,
	.configure	= samsung_ufs_phy_configure,
	.set_mode	= samsung_ufs_phy_set_mode,
};

static int samsung_ufs_phy_probe(struct udevice *dev)
{
	struct samsung_ufs_phy *phy = dev_get_priv(dev);
	struct resource res;
	const struct samsung_ufs_phy_drvdata *drvdata;
	int ret = 0;
	u32 isol_offset;

	ret = dev_read_resource_byname(dev, "phy-pma", &res);
	if (ret) {
		dev_err(dev, "failed get phy-pma register address, ret = %d\n", ret);
		goto out;
	}
	phy->reg_pma = (void *)res.start;

	dev_info(dev, "reg_pma=%p\n", phy->reg_pma);

	phy->reg_pmu = syscon_regmap_lookup_by_phandle(dev,
						       "samsung,pmu-syscon");
	if (IS_ERR(phy->reg_pmu)) {
		ret = PTR_ERR(phy->reg_pmu);
		dev_err(dev, "failed syscon remap for pmu\n");
		goto out;
	}

	drvdata = (const struct samsung_ufs_phy_drvdata *)dev_get_driver_data(dev);
	phy->dev = dev;
	phy->drvdata = drvdata;
	phy->cfgs = drvdata->cfgs;
	phy->cfgs_hibern8 = drvdata->cfgs_hibern8;
	memcpy(&phy->isol, &drvdata->isol, sizeof(phy->isol));
	if (!dev_read_u32_index(dev, "samsung,pmu-syscon", 1, &isol_offset))
		phy->isol.offset = isol_offset;
	phy->lane_cnt = PHY_DEF_LANE_CNT;

	ret = samsung_ufs_phy_clks_init(phy);
	if (ret) {
		dev_err(dev, "failed to get phy clocks\n");
		goto out;
	}

out:
	return ret;
}

static const struct udevice_id samsung_ufs_phy_match[] = {
	{
		.compatible = "samsung,exynos9820-ufs-phy",
		.data = (ulong)&exynos9820_ufs_phy,
	},
	{},
};

U_BOOT_DRIVER(exynos_ufs_phy) = {
	.name		= "samsung-ufs-phy",
	.id		= UCLASS_PHY,
	.of_match	= samsung_ufs_phy_match,
	.ops		= &samsung_ufs_phy_ops,
	.probe		= samsung_ufs_phy_probe,
	.priv_auto	= sizeof(struct samsung_ufs_phy),
};
