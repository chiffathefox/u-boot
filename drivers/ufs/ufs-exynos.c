// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS Host Controller driver for Exynos specific extensions
 *
 * Copyright (C) 2014-2015 Samsung Electronics Co., Ltd.
 * Author: Seungwon Jeon  <essuuj@gmail.com>
 * Author: Alim Akhtar <alim.akhtar@samsung.com>
 *
 */

// #include <linux/unaligned.h>
// #include <crypto/aes.h>
#include <clk-uclass.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <generic-phy.h>
#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/bug.h>
// #include <linux/clk.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/log2.h>
// #include <linux/module.h>
// #include <linux/of.h>
// #include <linux/of_address.h>
// #include <linux/mfd/syscon.h>
// #include <linux/phy/phy.h>
// #include <linux/platform_device.h>
// #include <linux/regmap.h>
#include <linux/time.h>
#include <regmap.h>
#include <syscon.h>
#include <time.h>

// #include <ufs/ufshcd.h>
// #include "ufshcd-pltfrm.h"
// #include <ufs/ufshci.h>
// #include <ufs/unipro.h>

#include "ufs.h"
#include "ufs-exynos.h"

#define DATA_UNIT_SIZE		4096

/*
 * Exynos's Vendor specific registers for UFSHCI
 */
#define HCI_TXPRDT_ENTRY_SIZE	0x00
#define PRDT_PREFETCH_EN	BIT(31)
#define HCI_RXPRDT_ENTRY_SIZE	0x04
#define HCI_1US_TO_CNT_VAL	0x0C
#define CNT_VAL_1US_MASK	0x3FF
#define HCI_UTRL_NEXUS_TYPE	0x40
#define HCI_UTMRL_NEXUS_TYPE	0x44
#define HCI_SW_RST		0x50
#define UFS_LINK_SW_RST		BIT(0)
#define UFS_UNIPRO_SW_RST	BIT(1)
#define UFS_SW_RST_MASK		(UFS_UNIPRO_SW_RST | UFS_LINK_SW_RST)
#define HCI_DATA_REORDER	0x60
#define HCI_UNIPRO_APB_CLK_CTRL	0x68
#define UNIPRO_APB_CLK(v, x)	(((v) & ~0xF) | ((x) & 0xF))
#define HCI_AXIDMA_RWDATA_BURST_LEN	0x6C
#define WLU_EN			BIT(31)
#define WLU_BURST_LEN(x)	((x) << 27 | ((x) & 0xF))
#define HCI_GPIO_OUT		0x70
#define HCI_ERR_EN_PA_LAYER	0x78
#define HCI_ERR_EN_DL_LAYER	0x7C
#define HCI_ERR_EN_N_LAYER	0x80
#define HCI_ERR_EN_T_LAYER	0x84
#define HCI_ERR_EN_DME_LAYER	0x88
#define HCI_V2P1_CTRL		0x8C
#define IA_TICK_SEL		BIT(16)
#define HCI_CLKSTOP_CTRL	0xB0
#define REFCLKOUT_STOP		BIT(4)
#define MPHY_APBCLK_STOP	BIT(3)
#define REFCLK_STOP		BIT(2)
#define UNIPRO_MCLK_STOP	BIT(1)
#define UNIPRO_PCLK_STOP	BIT(0)
#define CLK_STOP_MASK		(REFCLKOUT_STOP | REFCLK_STOP |\
				 UNIPRO_MCLK_STOP | \
				 UNIPRO_PCLK_STOP)
/* HCI_MISC is also known as HCI_FORCE_HCS */
#define HCI_MISC		0xB4
#define REFCLKOUT_STOP_EN	BIT(11)
#define MPHY_APBCLK_STOP_EN	BIT(10)
#define UFSP_DRCG_EN		BIT(8)   //FMP
#define REFCLK_CTRL_EN		BIT(7)
#define UNIPRO_PCLK_CTRL_EN	BIT(6)
#define UNIPRO_MCLK_CTRL_EN	BIT(5)
#define HCI_CORECLK_CTRL_EN	BIT(4)
#define CLK_CTRL_EN_MASK	(REFCLK_CTRL_EN |\
				 UNIPRO_PCLK_CTRL_EN |\
				 UNIPRO_MCLK_CTRL_EN |\
				/*ugly*/ REFCLKOUT_STOP_EN | MPHY_APBCLK_STOP_EN | UFSP_DRCG_EN)

#define HCI_IOP_ACG_DISABLE	0x100
#define HCI_IOP_ACG_DISABLE_EN	BIT(0)

/* Device fatal error */
#define DFES_ERR_EN		BIT(31)
#define DFES_DEF_L2_ERRS	(UIC_DATA_LINK_LAYER_ERROR_RX_BUF_OF |\
				 UIC_DATA_LINK_LAYER_ERROR_PA_INIT)
#define DFES_DEF_L3_ERRS	(UIC_NETWORK_UNSUPPORTED_HEADER_TYPE |\
				 UIC_NETWORK_BAD_DEVICEID_ENC |\
				 UIC_NETWORK_LHDR_TRAP_PACKET_DROPPING)
#define DFES_DEF_L4_ERRS	(UIC_TRANSPORT_UNSUPPORTED_HEADER_TYPE |\
				 UIC_TRANSPORT_UNKNOWN_CPORTID |\
				 UIC_TRANSPORT_NO_CONNECTION_RX |\
				 UIC_TRANSPORT_BAD_TC)

/* UFS Shareability */
#define UFS_EXYNOSAUTO_WR_SHARABLE	BIT(2)
#define UFS_EXYNOSAUTO_RD_SHARABLE	BIT(1)
#define UFS_EXYNOSAUTO_SHARABLE		(UFS_EXYNOSAUTO_WR_SHARABLE | \
					 UFS_EXYNOSAUTO_RD_SHARABLE)
#define UFS_GS101_WR_SHARABLE		BIT(1)
#define UFS_GS101_RD_SHARABLE		BIT(0)
#define UFS_GS101_SHARABLE		(UFS_GS101_WR_SHARABLE | \
					 UFS_GS101_RD_SHARABLE)
#define EXYNOS9820_WR_SHARABLE		BIT(23)
#define EXYNOS9820_RD_SHARABLE		BIT(22)
#define EXYNOS9820_SHARABLE		(EXYNOS9820_WR_SHARABLE | \
					 EXYNOS9820_RD_SHARABLE)
#define UFS_SHAREABILITY_OFFSET		0x710

/* Multi-host registers */
#define MHCTRL			0xC4
#define MHCTRL_EN_VH_MASK	(0xE)
#define MHCTRL_EN_VH(vh)	(vh << 1)
#define PH2VH_MBOX		0xD8

#define MH_MSG_MASK		(0xFF)

#define MH_MSG(id, msg)		((id << 8) | (msg & 0xFF))
#define MH_MSG_PH_READY		0x1
#define MH_MSG_VH_READY		0x2

#define ALLOW_INQUIRY		BIT(25)
#define ALLOW_MODE_SELECT	BIT(24)
#define ALLOW_MODE_SENSE	BIT(23)
#define ALLOW_PRE_FETCH		GENMASK(22, 21)
#define ALLOW_READ_CMD_ALL	GENMASK(20, 18)	/* read_6/10/16 */
#define ALLOW_READ_BUFFER	BIT(17)
#define ALLOW_READ_CAPACITY	GENMASK(16, 15)
#define ALLOW_REPORT_LUNS	BIT(14)
#define ALLOW_REQUEST_SENSE	BIT(13)
#define ALLOW_SYNCHRONIZE_CACHE	GENMASK(8, 7)
#define ALLOW_TEST_UNIT_READY	BIT(6)
#define ALLOW_UNMAP		BIT(5)
#define ALLOW_VERIFY		BIT(4)
#define ALLOW_WRITE_CMD_ALL	GENMASK(3, 1)	/* write_6/10/16 */

#define ALLOW_TRANS_VH_DEFAULT	(ALLOW_INQUIRY | ALLOW_MODE_SELECT | \
				 ALLOW_MODE_SENSE | ALLOW_PRE_FETCH | \
				 ALLOW_READ_CMD_ALL | ALLOW_READ_BUFFER | \
				 ALLOW_READ_CAPACITY | ALLOW_REPORT_LUNS | \
				 ALLOW_REQUEST_SENSE | ALLOW_SYNCHRONIZE_CACHE | \
				 ALLOW_TEST_UNIT_READY | ALLOW_UNMAP | \
				 ALLOW_VERIFY | ALLOW_WRITE_CMD_ALL)

#define HCI_MH_ALLOWABLE_TRAN_OF_VH		0x30C
#define HCI_MH_IID_IN_TASK_TAG			0X308

#define PH_READY_TIMEOUT_MS			(5 * MSEC_PER_SEC)

enum {
	UNIPRO_L1_5 = 0,/* PHY Adapter */
	UNIPRO_L2,	/* Data Link */
	UNIPRO_L3,	/* Network */
	UNIPRO_L4,	/* Transport */
	UNIPRO_DME,	/* DME */
};

/*
 * UNIPRO registers
 */
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER0	0x7888
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER1	0x788c
#define UNIPRO_DME_POWERMODE_REQ_LOCALL2TIMER2	0x7890
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER0	0x78B8
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER1	0x78BC
#define UNIPRO_DME_POWERMODE_REQ_REMOTEL2TIMER2	0x78C0

/*
 * UFS Protector registers
 */
#define UFSPRSECURITY	0x010
#define NSSMU		BIT(14)
#define UFSPSBEGIN0	0x200
#define UFSPSEND0	0x204
#define UFSPSLUN0	0x208
#define UFSPSCTRL0	0x20C

#define CNTR_DIV_VAL 40

static void exynos_ufs_auto_ctrl_hcc(struct exynos_ufs *ufs, bool en);
static void exynos_ufs_ctrl_clkstop(struct exynos_ufs *ufs, bool en);

static inline void exynos_ufs_enable_auto_ctrl_hcc(struct exynos_ufs *ufs)
{
	exynos_ufs_auto_ctrl_hcc(ufs, true);
}

static inline void exynos_ufs_disable_auto_ctrl_hcc(struct exynos_ufs *ufs)
{
	exynos_ufs_auto_ctrl_hcc(ufs, false);
}

static inline void exynos_ufs_disable_auto_ctrl_hcc_save(
					struct exynos_ufs *ufs, u32 *val)
{
	*val = hci_readl(ufs, HCI_MISC);
	exynos_ufs_auto_ctrl_hcc(ufs, false);
}

static inline void exynos_ufs_auto_ctrl_hcc_restore(
					struct exynos_ufs *ufs, u32 *val)
{
	hci_writel(ufs, *val, HCI_MISC);
}

static inline void exynos_ufs_gate_clks(struct exynos_ufs *ufs)
{
	exynos_ufs_ctrl_clkstop(ufs, true);
}

static inline void exynos_ufs_ungate_clks(struct exynos_ufs *ufs)
{
	exynos_ufs_ctrl_clkstop(ufs, false);
}

static int exynos_ufs_shareability(struct exynos_ufs *ufs)
{
	/* IO Coherency setting */
	if (ufs->sysreg) {
		return regmap_update_bits(ufs->sysreg,
					  ufs->iocc_offset,
					  ufs->iocc_mask, ufs->iocc_val);
	}

	return 0;
}

static int exynos9820_ufs_drv_init(struct exynos_ufs *ufs)
{
	u32 reg;

	/* set ACG to be controlled by UFS_ACG_DISABLE */
	reg = hci_readl(ufs, HCI_IOP_ACG_DISABLE);
	hci_writel(ufs, reg & (~HCI_IOP_ACG_DISABLE_EN), HCI_IOP_ACG_DISABLE);

	return exynos_ufs_shareability(ufs);
}

static int gs101_ufs_drv_init(struct exynos_ufs *ufs)
{
	u32 reg;

	/* set ACG to be controlled by UFS_ACG_DISABLE */
	reg = hci_readl(ufs, HCI_IOP_ACG_DISABLE);
	hci_writel(ufs, reg & (~HCI_IOP_ACG_DISABLE_EN), HCI_IOP_ACG_DISABLE);

	return exynos_ufs_shareability(ufs);
}

static int exynosauto_ufs_drv_init(struct exynos_ufs *ufs)
{
	return exynos_ufs_shareability(ufs);
}

static int exynosauto_ufs_post_hce_enable(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	/* Enable Virtual Host #1 */
	ufshcd_rmwl(hba, MHCTRL_EN_VH_MASK, MHCTRL_EN_VH(1), MHCTRL);
	/* Default VH Transfer permissions */
	hci_writel(ufs, ALLOW_TRANS_VH_DEFAULT, HCI_MH_ALLOWABLE_TRAN_OF_VH);
	/* IID information is replaced in TASKTAG[7:5] instead of IID in UCD */
	hci_writel(ufs, 0x1, HCI_MH_IID_IN_TASK_TAG);

	return 0;
}

static int exynosauto_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate) / NSEC_PER_MSEC;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate) / NSEC_PER_MSEC;

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);
	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x79);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		/* Not to affect VND_TX_LINERESET_PVALUE to VND_TX_CLK_PRD */
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);

		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       (tx_line_reset_period) & 0xFF);

		/* TX PWM Gear Capability / PWM_G1_ONLY */
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 0x1);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0xa011), 0x8000);

	return 0;
}

static int exynos7_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	u32 val = attr->pa_dbg_opt_suite1_val;
	struct ufs_hba *hba = ufs->hba;
	int i;

	exynos_ufs_enable_ov_tm(hba);
	for_each_ufs_tx_lane(ufs, i)
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x297, i), 0x17);
	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x362, i), 0xff);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x363, i), 0x00);
	}
	exynos_ufs_disable_ov_tm(hba);

	for_each_ufs_tx_lane(ufs, i)
		ufshcd_dme_set(hba,
			UIC_ARG_MIB_SEL(TX_HIBERN8_CONTROL, i), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_TXPHY_CFGUPDT), 0x1);
	udelay(1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
					val | (1 << 12));
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_SKIP_RESET_PHY), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_SKIP_LINE_RESET), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_LINE_RESET_REQ), 0x1);
	udelay(1600);
	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off), val);

	return 0;
}

static int exynos7_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;

	exynos_ufs_enable_ov_tm(hba);
	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x28b, i), 0x83);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x29a, i), 0x07);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x277, i),
			TX_LINERESET_N(exynos_ufs_calc_time_cntr(ufs, 200000)));
	}
	exynos_ufs_disable_ov_tm(hba);

	exynos_ufs_enable_dbg_mode(hba);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0xbb8);
	exynos_ufs_disable_dbg_mode(hba);

	return 0;
}

/*
 * exynos_ufs_auto_ctrl_hcc - HCI core clock control by h/w
 * Control should be disabled in the below cases
 * - Before host controller S/W reset
 * - Access to UFS protector's register
 */
static void exynos_ufs_auto_ctrl_hcc(struct exynos_ufs *ufs, bool en)
{
	u32 misc = hci_readl(ufs, HCI_MISC);

	if (en)
		hci_writel(ufs, misc | HCI_CORECLK_CTRL_EN, HCI_MISC);
	else
		hci_writel(ufs, misc & ~HCI_CORECLK_CTRL_EN, HCI_MISC);
}

static void exynos_ufs_ctrl_clkstop(struct exynos_ufs *ufs, bool en)
{
	u32 ctrl = hci_readl(ufs, HCI_CLKSTOP_CTRL);
	u32 misc = hci_readl(ufs, HCI_MISC);

	if (en) {
		hci_writel(ufs, misc | CLK_CTRL_EN_MASK, HCI_MISC);
		hci_writel(ufs, ctrl | CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
	} else {
		hci_writel(ufs, ctrl & ~CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
		hci_writel(ufs, misc & ~CLK_CTRL_EN_MASK, HCI_MISC);
	}
}


#define HCI_UFS_ACG_DISABLE		0xFC
#define HCI_UFS_ACG_DISABLE_EN		BIT(0)
inline void exynos_ufs_set_hwacg_control(struct exynos_ufs *ufs, bool en)
{
	u32 reg;

	/*
	 * default value 1->0 at KC. so,
	 * need to set "1(disable HWACG)" during UFS init
	 */
	reg = hci_readl(ufs, HCI_UFS_ACG_DISABLE);
	if (en)
		hci_writel(ufs, reg & (~HCI_UFS_ACG_DISABLE_EN), HCI_UFS_ACG_DISABLE);
	else
		hci_writel(ufs, reg | HCI_UFS_ACG_DISABLE_EN, HCI_UFS_ACG_DISABLE);

}

static int exynos_ufs_get_clk_info(struct exynos_ufs *ufs)
{
	struct udevice *dev = ufs->hba->dev;
	struct clk clk;
	unsigned long pclk_rate;
	u32 f_min, f_max;
	u8 div = 0;
	int ret;

	ret = clk_get_by_name(dev, "sclk_unipro_main", &clk);
	if (ret) {
		dev_err(dev, "failed to get sclk_unipro_main\n");
		ret = -EINVAL;
		goto out;
	}
	ufs->mclk_rate = clk_get_rate(&clk);

	ret = clk_get_by_name(dev, "core_clk", &clk);
	if (ret) {
		dev_err(dev, "failed to get core_clk\n");
		ret = -EINVAL;
		goto out;
	}
	pclk_rate = clk_get_rate(&clk);
	f_min = ufs->pclk_avail_min;
	f_max = ufs->pclk_avail_max;

	if (ufs->opts & EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL) {
		do {
			pclk_rate /= (div + 1);

			if (pclk_rate <= f_max)
				break;
			div++;
		} while (pclk_rate >= f_min);
	}

	if (unlikely(pclk_rate < f_min || pclk_rate > f_max)) {
		dev_err(dev, "not available pclk range %lu\n", pclk_rate);
		ret = -EINVAL;
		goto out;
	}

	ufs->pclk_rate = pclk_rate;
	ufs->pclk_div = div;

out:
	return ret;
}

static void exynos_ufs_set_unipro_pclk_div(struct exynos_ufs *ufs)
{
	if (ufs->opts & EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL) {
		u32 val;

		val = hci_readl(ufs, HCI_UNIPRO_APB_CLK_CTRL);
		hci_writel(ufs, UNIPRO_APB_CLK(val, ufs->pclk_div),
			   HCI_UNIPRO_APB_CLK_CTRL);
	}
}

static void exynos_ufs_set_pwm_clk_div(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;

	ufshcd_dme_set(hba,
		UIC_ARG_MIB(CMN_PWM_CLK_CTRL), attr->cmn_pwm_clk_ctrl);
}

static void exynos_ufs_calc_pwm_clk_div(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	const unsigned int div = 30, mult = 20;
	const unsigned long pwm_min = 3 * 1000 * 1000;
	const unsigned long pwm_max = 9 * 1000 * 1000;
	const int divs[] = {32, 16, 8, 4};
	unsigned long clk = 0, _clk, clk_period;
	int i = 0, clk_idx = -1;

	clk_period = UNIPRO_PCLK_PERIOD(ufs);
	for (i = 0; i < ARRAY_SIZE(divs); i++) {
		_clk = NSEC_PER_SEC * mult / (clk_period * divs[i] * div);
		if (_clk >= pwm_min && _clk <= pwm_max) {
			if (_clk > clk) {
				clk_idx = i;
				clk = _clk;
			}
		}
	}

	if (clk_idx == -1) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(CMN_PWM_CLK_CTRL), &clk_idx);
		dev_err(hba->dev,
			"failed to decide pwm clock divider, will not change\n");
	}

	attr->cmn_pwm_clk_ctrl = clk_idx & PWM_CLK_CTRL_MASK;
}

long exynos_ufs_calc_time_cntr(struct exynos_ufs *ufs, long period)
{
	const int precise = 10;
	long pclk_rate = ufs->pclk_rate;
	long clk_period, fraction;

	clk_period = UNIPRO_PCLK_PERIOD(ufs);
	fraction = ((NSEC_PER_SEC % pclk_rate) * precise) / pclk_rate;

	return (period * precise) / ((clk_period * precise) + fraction);
}

static void exynos_ufs_specify_phy_time_attr(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_phy_time_cfg *t_cfg = &ufs->t_cfg;

	if (ufs->opts & EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR)
		return;

	t_cfg->tx_linereset_p =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_dif_p_nsec);
	t_cfg->tx_linereset_n =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_dif_n_nsec);
	t_cfg->tx_high_z_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_high_z_cnt_nsec);
	t_cfg->tx_base_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_base_unit_nsec);
	t_cfg->tx_gran_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_gran_unit_nsec);
	t_cfg->tx_sleep_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->tx_sleep_cnt);

	t_cfg->rx_linereset =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_dif_p_nsec);
	t_cfg->rx_hibern8_wait =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_hibern8_wait_nsec);
	t_cfg->rx_base_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_base_unit_nsec);
	t_cfg->rx_gran_n_val =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_gran_unit_nsec);
	t_cfg->rx_sleep_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_sleep_cnt);
	t_cfg->rx_stall_cnt =
		exynos_ufs_calc_time_cntr(ufs, attr->rx_stall_cnt);
}

static void exynos_ufs_config_phy_time_attr(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct ufs_phy_time_cfg *t_cfg = &ufs->t_cfg;
	int i;

	exynos_ufs_set_pwm_clk_div(ufs);

	exynos_ufs_enable_ov_tm(hba);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_FILLER_ENABLE, i),
				ufs->drv_data->uic_attr->rx_filler_enable);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_LINERESET_VAL, i),
				RX_LINERESET(t_cfg->rx_linereset));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_BASE_NVAL_07_00, i),
				RX_BASE_NVAL_L(t_cfg->rx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_BASE_NVAL_15_08, i),
				RX_BASE_NVAL_H(t_cfg->rx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_GRAN_NVAL_07_00, i),
				RX_GRAN_NVAL_L(t_cfg->rx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_GRAN_NVAL_10_08, i),
				RX_GRAN_NVAL_H(t_cfg->rx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_OV_SLEEP_CNT_TIMER, i),
				RX_OV_SLEEP_CNT(t_cfg->rx_sleep_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(RX_OV_STALL_CNT_TIMER, i),
				RX_OV_STALL_CNT(t_cfg->rx_stall_cnt));
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_LINERESET_P_VAL, i),
				TX_LINERESET_P(t_cfg->tx_linereset_p));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_HIGH_Z_CNT_07_00, i),
				TX_HIGH_Z_CNT_L(t_cfg->tx_high_z_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_HIGH_Z_CNT_11_08, i),
				TX_HIGH_Z_CNT_H(t_cfg->tx_high_z_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_BASE_NVAL_07_00, i),
				TX_BASE_NVAL_L(t_cfg->tx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_BASE_NVAL_15_08, i),
				TX_BASE_NVAL_H(t_cfg->tx_base_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_GRAN_NVAL_07_00, i),
				TX_GRAN_NVAL_L(t_cfg->tx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_GRAN_NVAL_10_08, i),
				TX_GRAN_NVAL_H(t_cfg->tx_gran_n_val));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_OV_SLEEP_CNT_TIMER, i),
				TX_OV_H8_ENTER_EN |
				TX_OV_SLEEP_CNT(t_cfg->tx_sleep_cnt));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(TX_MIN_ACTIVATETIME, i),
				ufs->drv_data->uic_attr->tx_min_activatetime);
	}

	exynos_ufs_disable_ov_tm(hba);
}

static void exynos_ufs_config_phy_cap_attr(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	int i;

	exynos_ufs_enable_ov_tm(hba);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G1_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g1_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G2_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g2_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G3_SYNC_LENGTH_CAP, i),
				attr->rx_hs_g3_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G1_PREP_LENGTH_CAP, i),
				attr->rx_hs_g1_prep_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G2_PREP_LENGTH_CAP, i),
				attr->rx_hs_g2_prep_sync_len_cap);
		ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_HS_G3_PREP_LENGTH_CAP, i),
				attr->rx_hs_g3_prep_sync_len_cap);
	}

	if (attr->rx_adv_fine_gran_sup_en == 0) {
		for_each_ufs_rx_lane(ufs, i) {
			ufshcd_dme_set(hba,
				UIC_ARG_MIB_SEL(RX_ADV_GRANULARITY_CAP, i), 0);

			if (attr->rx_min_actv_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(
					RX_MIN_ACTIVATETIME_CAPABILITY, i),
					attr->rx_min_actv_time_cap);

			if (attr->rx_hibern8_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_HIBERN8TIME_CAP, i),
						attr->rx_hibern8_time_cap);
		}
	} else if (attr->rx_adv_fine_gran_sup_en == 1) {
		for_each_ufs_rx_lane(ufs, i) {
			if (attr->rx_adv_fine_gran_step)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_ADV_GRANULARITY_CAP,
						i), RX_ADV_FINE_GRAN_STEP(
						attr->rx_adv_fine_gran_step));

			if (attr->rx_adv_min_actv_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(
						RX_ADV_MIN_ACTIVATETIME_CAP, i),
						attr->rx_adv_min_actv_time_cap);

			if (attr->rx_adv_hibern8_time_cap)
				ufshcd_dme_set(hba,
					UIC_ARG_MIB_SEL(RX_ADV_HIBERN8TIME_CAP,
						i),
						attr->rx_adv_hibern8_time_cap);
		}
	}

	exynos_ufs_disable_ov_tm(hba);
}

static void exynos_ufs_establish_connt(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	enum {
		DEV_ID		= 0x00,
		PEER_DEV_ID	= 0x01,
		PEER_CPORT_ID	= 0x00,
		TRAFFIC_CLASS	= 0x00,
	};

	/* allow cport attributes to be set */
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_IDLE);

	/* local unipro attributes */
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), DEV_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), true);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), PEER_DEV_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERCPORTID), PEER_CPORT_ID);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CPORTFLAGS), CPORT_DEF_FLAGS);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_TRAFFICCLASS), TRAFFIC_CLASS);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
}

static void exynos_ufs_config_smu(struct exynos_ufs *ufs)
{
	u32 reg, val;

	if (ufs->opts & EXYNOS_UFS_OPT_UFSPR_SECURE)
		return;

	exynos_ufs_disable_auto_ctrl_hcc_save(ufs, &val);

	/* make encryption disabled by default */
	reg = ufsp_readl(ufs, UFSPRSECURITY);
	ufsp_writel(ufs, reg | NSSMU, UFSPRSECURITY);
	ufsp_writel(ufs, 0x0, UFSPSBEGIN0);
	ufsp_writel(ufs, 0xffffffff, UFSPSEND0);
	ufsp_writel(ufs, 0xff, UFSPSLUN0);
	ufsp_writel(ufs, 0xf1, UFSPSCTRL0);

	exynos_ufs_auto_ctrl_hcc_restore(ufs, &val);
}

static int exynos_ufs_phy_init(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	struct udevice *dev = hba->dev;
	int ret = 0;

	if (ufs->avail_ln_rx == 0 || ufs->avail_ln_tx == 0) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILRXDATALANES),
			&ufs->avail_ln_rx);
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILTXDATALANES),
			&ufs->avail_ln_tx);
		WARN(ufs->avail_ln_rx != ufs->avail_ln_tx,
			"available data lane is not equal(rx:%d, tx:%d)\n",
			ufs->avail_ln_rx, ufs->avail_ln_tx);
	}

	ret = generic_phy_get_by_name(dev, "ufs-phy", &ufs->phy);
	if (ret) {
		dev_err(dev, "failed to get ufs-phy, ret = %d\n", ret);
		return ret;
	}

	ret = generic_phy_init(&ufs->phy);
	if (ret) {
		dev_err(dev, "%s: phy init failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	ret = generic_phy_power_on(&ufs->phy);
	if (ret)
		goto out_exit_phy;

	return 0;

out_exit_phy:
	generic_phy_exit(&ufs->phy);

	return ret;
}

static void exynos_ufs_config_unipro(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_hba *hba = ufs->hba;

	if (attr->pa_dbg_clk_period_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_clk_period_off),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXTRAILINGCLOCKS),
			ufs->drv_data->uic_attr->tx_trailingclks);

	if (attr->pa_dbg_opt_suite1_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
			       attr->pa_dbg_opt_suite1_val);

	if (attr->pa_dbg_opt_suite2_off)
		ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite2_off),
			       attr->pa_dbg_opt_suite2_val);
}

static void exynos_ufs_config_intr(struct exynos_ufs *ufs, u32 errs, u8 index)
{
	switch (index) {
	case UNIPRO_L1_5:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_PA_LAYER);
		break;
	case UNIPRO_L2:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_DL_LAYER);
		break;
	case UNIPRO_L3:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_N_LAYER);
		break;
	case UNIPRO_L4:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_T_LAYER);
		break;
	case UNIPRO_DME:
		hci_writel(ufs, DFES_ERR_EN | errs, HCI_ERR_EN_DME_LAYER);
		break;
	}
}

static int exynos_ufs_setup_clocks(struct ufs_hba *hba, bool on,
				   enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);

	if (!ufs)
		return 0;

	if (on && status == PRE_CHANGE) {
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_disable_auto_ctrl_hcc(ufs);
		exynos_ufs_ungate_clks(ufs);
		exynos_ufs_set_hwacg_control(ufs, false);
	} else if (!on && status == POST_CHANGE) {
		exynos_ufs_gate_clks(ufs);
		if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL)
			exynos_ufs_enable_auto_ctrl_hcc(ufs);
	}

	return 0;
}

static int exynos_ufs_pre_link(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);

	/* hci */
	exynos_ufs_config_intr(ufs, DFES_DEF_L2_ERRS, UNIPRO_L2);
	exynos_ufs_config_intr(ufs, DFES_DEF_L3_ERRS, UNIPRO_L3);
	exynos_ufs_config_intr(ufs, DFES_DEF_L4_ERRS, UNIPRO_L4);
	exynos_ufs_set_unipro_pclk_div(ufs);

	exynos_ufs_setup_clocks(hba, true, PRE_CHANGE);

	/* unipro */
	exynos_ufs_config_unipro(ufs);

	if (ufs->drv_data->pre_link)
		ufs->drv_data->pre_link(ufs);

	/* m-phy */
	exynos_ufs_phy_init(ufs);
	if (!(ufs->opts & EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR)) {
		exynos_ufs_config_phy_time_attr(ufs);
		exynos_ufs_config_phy_cap_attr(ufs);
	}

	return 0;
}

static void exynos_ufs_fit_aggr_timeout(struct exynos_ufs *ufs)
{
	u32 val;

	/* Select function clock (mclk) for timer tick */
	if (ufs->opts & EXYNOS_UFS_OPT_TIMER_TICK_SELECT) {
		val = hci_readl(ufs, HCI_V2P1_CTRL);
		val |= IA_TICK_SEL;
		hci_writel(ufs, val, HCI_V2P1_CTRL);
	}

	val = exynos_ufs_calc_time_cntr(ufs, IATOVAL_NSEC / CNTR_DIV_VAL);
	hci_writel(ufs, val & CNT_VAL_1US_MASK, HCI_1US_TO_CNT_VAL);
}

static int exynos_ufs_post_link(struct ufs_hba *hba)
{
	struct udevice *dev = hba->dev;
	struct exynos_ufs *ufs = dev_get_priv(dev);
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	u32 val = ilog2(DATA_UNIT_SIZE);

	exynos_ufs_establish_connt(ufs);
	exynos_ufs_fit_aggr_timeout(ufs);

	hci_writel(ufs, 0xa, HCI_DATA_REORDER);

	hci_writel(ufs, val, HCI_TXPRDT_ENTRY_SIZE);

	hci_writel(ufs, ilog2(DATA_UNIT_SIZE), HCI_RXPRDT_ENTRY_SIZE);
	// TODO:
	// hci_writel(ufs, BIT(hba->nutrs) - 1, HCI_UTRL_NEXUS_TYPE);
	// hci_writel(ufs, BIT(hba->nutmrs) - 1, HCI_UTMRL_NEXUS_TYPE);
	hci_writel(ufs, 0xf, HCI_AXIDMA_RWDATA_BURST_LEN);

	if (ufs->opts & EXYNOS_UFS_OPT_SKIP_CONNECTION_ESTAB)
		ufshcd_dme_set(hba,
			UIC_ARG_MIB(T_DBG_SKIP_INIT_HIBERN8_EXIT), true);

	if (attr->pa_granularity) {
		exynos_ufs_enable_dbg_mode(hba);
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_GRANULARITY),
				attr->pa_granularity);
		exynos_ufs_disable_dbg_mode(hba);

		if (attr->pa_tactivate)
			ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TACTIVATE),
					attr->pa_tactivate);
		if (attr->pa_hibern8time &&
		    !(ufs->opts & EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER))
			ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
					attr->pa_hibern8time);
	}

	if (ufs->opts & EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER) {
		if (!attr->pa_granularity)
			ufshcd_dme_get(hba, UIC_ARG_MIB(PA_GRANULARITY),
					&attr->pa_granularity);
		if (!attr->pa_hibern8time)
			ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
					&attr->pa_hibern8time);
		/*
		 * not wait for HIBERN8 time to exit hibernation
		 */
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), 0);

		if (attr->pa_granularity < 1 || attr->pa_granularity > 6) {
			/* Valid range for granularity: 1 ~ 6 */
			dev_warn(dev,
				"%s: pa_granularity %d is invalid, assuming backwards compatibility\n",
				__func__,
				attr->pa_granularity);
			attr->pa_granularity = 6;
		}
	}

	generic_phy_configure(&ufs->phy, NULL); // should be calibrate

	if (ufs->drv_data->post_link)
		ufs->drv_data->post_link(ufs);

	return 0;
}

static int exynos_ufs_parse_dt(struct udevice *dev, struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr;
	int ret = 0;

	ufs->drv_data =
		(const struct exynos_ufs_drv_data *)dev_get_driver_data(dev);

	if (ufs->drv_data && ufs->drv_data->uic_attr) {
		attr = ufs->drv_data->uic_attr;
	} else {
		dev_err(dev, "failed to get uic attributes\n");
		ret = -EINVAL;
		goto out;
	}

	ufs->sysreg = syscon_regmap_lookup_by_phandle(dev, "samsung,sysreg");
	if (IS_ERR(ufs->sysreg))
		ufs->sysreg = NULL;
	else
		ufs->iocc_offset = dev_read_u32_index_default(
			dev, "samsung,sysreg", 1, UFS_SHAREABILITY_OFFSET);

	ufs->iocc_mask = ufs->drv_data->iocc_mask;
	/*
	 * no 'dma-coherent' property means the descriptors are
	 * non-cacheable so iocc shareability should be disabled.
	 */
	if (dev_read_bool(dev, "dma-coherent"))
		ufs->iocc_val = ufs->iocc_mask;
	else
		ufs->iocc_val = 0;

	log_debug("%s: iocc_mask = %X, iocc_val = %X\n", __func__, ufs->iocc_mask, ufs->iocc_val);

	ufs->pclk_avail_min = PCLK_AVAIL_MIN;
	ufs->pclk_avail_max = PCLK_AVAIL_MAX;

	attr->rx_adv_fine_gran_sup_en = RX_ADV_FINE_GRAN_SUP_EN;
	attr->rx_adv_fine_gran_step = RX_ADV_FINE_GRAN_STEP_VAL;
	attr->rx_adv_min_actv_time_cap = RX_ADV_MIN_ACTV_TIME_CAP;
	attr->pa_granularity = PA_GRANULARITY_VAL;
	attr->pa_tactivate = PA_TACTIVATE_VAL;
	attr->pa_hibern8time = PA_HIBERN8TIME_VAL;

out:
	return ret;
}

static inline void exynos_ufs_priv_init(struct ufs_hba *hba,
					struct exynos_ufs *ufs)
{
	ufs->hba = hba;
	ufs->opts = ufs->drv_data->opts;
	ufs->rx_sel_idx = PA_MAXDATALANES;
	if (ufs->opts & EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX)
		ufs->rx_sel_idx = 0;
	hba->quirks = ufs->drv_data->quirks;
}
#ifdef CONFIG_SCSI_UFS_CRYPTO

/*
 * Support for Flash Memory Protector (FMP), which is the inline encryption
 * hardware on Exynos and Exynos-based SoCs.  The interface to this hardware is
 * not compatible with the standard UFS crypto.  It requires that encryption be
 * configured in the PRDT using a nonstandard extension.
 */

enum fmp_crypto_algo_mode {
	FMP_BYPASS_MODE = 0,
	FMP_ALGO_MODE_AES_CBC = 1,
	FMP_ALGO_MODE_AES_XTS = 2,
};
enum fmp_crypto_key_length {
	FMP_KEYLEN_256BIT = 1,
};

/**
 * struct fmp_sg_entry - nonstandard format of PRDT entries when FMP is enabled
 *
 * @base: The standard PRDT entry, but with nonstandard bitfields in the high
 *	bits of the 'size' field, i.e. the last 32-bit word.  When these
 *	nonstandard bitfields are zero, the data segment won't be encrypted or
 *	decrypted.  Otherwise they specify the algorithm and key length with
 *	which the data segment will be encrypted or decrypted.
 * @file_iv: The initialization vector (IV) with all bytes reversed
 * @file_enckey: The first half of the AES-XTS key with all bytes reserved
 * @file_twkey: The second half of the AES-XTS key with all bytes reserved
 * @disk_iv: Unused
 * @reserved: Unused
 */
struct fmp_sg_entry {
	struct ufshcd_sg_entry base;
	__be64 file_iv[2];
	__be64 file_enckey[4];
	__be64 file_twkey[4];
	__be64 disk_iv[2];
	__be64 reserved[2];
};

#define SMC_CMD_FMP_SECURITY	\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1810)
#define SMC_CMD_SMU		\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1850)
#define SMC_CMD_FMP_SMU_RESUME	\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1860)
#define SMU_EMBEDDED			0
#define SMU_INIT			0
#define CFG_DESCTYPE_3			3

static void exynos_ufs_fmp_init(struct ufs_hba *hba, struct exynos_ufs *ufs)
{
	struct blk_crypto_profile *profile = &hba->crypto_profile;
	struct arm_smccc_res res;
	int err;

	/*
	 * Check for the standard crypto support bit, since it's available even
	 * though the rest of the interface to FMP is nonstandard.
	 *
	 * This check should have the effect of preventing the driver from
	 * trying to use FMP on old Exynos SoCs that don't have FMP.
	 */
	if (!(ufshcd_readl(hba, REG_CONTROLLER_CAPABILITIES) &
	      MASK_CRYPTO_SUPPORT))
		return;

	/*
	 * The below sequence of SMC calls to enable FMP can be found in the
	 * downstream driver source for gs101 and other Exynos-based SoCs.  It
	 * is the only way to enable FMP that works on SoCs such as gs101 that
	 * don't make the FMP registers accessible to Linux.  It probably works
	 * on other Exynos-based SoCs too, and might even still be the only way
	 * that works.  But this hasn't been properly tested, and this code is
	 * mutually exclusive with exynos_ufs_config_smu().  So for now only
	 * enable FMP support on SoCs with EXYNOS_UFS_OPT_UFSPR_SECURE.
	 */
	if (!(ufs->opts & EXYNOS_UFS_OPT_UFSPR_SECURE))
		return;

	/*
	 * This call (which sets DESCTYPE to 0x3 in the FMPSECURITY0 register)
	 * is needed to make the hardware use the larger PRDT entry size.
	 */
	BUILD_BUG_ON(sizeof(struct fmp_sg_entry) != 128);
	arm_smccc_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3,
		      0, 0, 0, 0, &res);
	if (res.a0) {
		dev_warn(hba->dev,
			 "SMC_CMD_FMP_SECURITY failed on init: %ld.  Disabling FMP support.\n",
			 res.a0);
		return;
	}
	ufshcd_set_sg_entry_size(hba, sizeof(struct fmp_sg_entry));

	/*
	 * This is needed to initialize FMP.  Without it, errors occur when
	 * inline encryption is used.
	 */
	arm_smccc_smc(SMC_CMD_SMU, SMU_INIT, SMU_EMBEDDED, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(hba->dev,
			"SMC_CMD_SMU(SMU_INIT) failed: %ld.  Disabling FMP support.\n",
			res.a0);
		return;
	}

	/* Advertise crypto capabilities to the block layer. */
	err = devm_blk_crypto_profile_init(hba->dev, profile, 0);
	if (err) {
		/* Only ENOMEM should be possible here. */
		dev_err(hba->dev, "Failed to initialize crypto profile: %d\n",
			err);
		return;
	}
	profile->max_dun_bytes_supported = AES_BLOCK_SIZE;
	profile->key_types_supported = BLK_CRYPTO_KEY_TYPE_RAW;
	profile->dev = hba->dev;
	profile->modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		DATA_UNIT_SIZE;

	/* Advertise crypto support to ufshcd-core. */
	hba->caps |= UFSHCD_CAP_CRYPTO;

	/* Advertise crypto quirks to ufshcd-core. */
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_CRYPTO_PROFILE |
		       UFSHCD_QUIRK_BROKEN_CRYPTO_ENABLE |
		       UFSHCD_QUIRK_KEYS_IN_PRDT;

}

static void exynos_ufs_fmp_resume(struct ufs_hba *hba)
{
	struct arm_smccc_res res;

	if (!(hba->caps & UFSHCD_CAP_CRYPTO))
		return;

	arm_smccc_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3,
		      0, 0, 0, 0, &res);
	if (res.a0)
		dev_err(hba->dev,
			"SMC_CMD_FMP_SECURITY failed on resume: %ld\n", res.a0);

	arm_smccc_smc(SMC_CMD_FMP_SMU_RESUME, 0, SMU_EMBEDDED, 0, 0, 0, 0, 0,
		      &res);
	if (res.a0)
		dev_err(hba->dev,
			"SMC_CMD_FMP_SMU_RESUME failed: %ld\n", res.a0);
}

static inline __be64 fmp_key_word(const u8 *key, int j)
{
	return cpu_to_be64(get_unaligned_le64(
			key + AES_KEYSIZE_256 - (j + 1) * sizeof(u64)));
}

/* Fill the PRDT for a request according to the given encryption context. */
static int exynos_ufs_fmp_fill_prdt(struct ufs_hba *hba,
				    const struct bio_crypt_ctx *crypt_ctx,
				    void *prdt, unsigned int num_segments)
{
	struct fmp_sg_entry *fmp_prdt = prdt;
	const u8 *enckey = crypt_ctx->bc_key->bytes;
	const u8 *twkey = enckey + AES_KEYSIZE_256;
	u64 dun_lo = crypt_ctx->bc_dun[0];
	u64 dun_hi = crypt_ctx->bc_dun[1];
	unsigned int i;

	/* If FMP wasn't enabled, we shouldn't get any encrypted requests. */
	if (WARN_ON_ONCE(!(hba->caps & UFSHCD_CAP_CRYPTO)))
		return -EIO;

	/* Configure FMP on each segment of the request. */
	for (i = 0; i < num_segments; i++) {
		struct fmp_sg_entry *prd = &fmp_prdt[i];
		int j;

		/* Each segment must be exactly one data unit. */
		if (prd->base.size != cpu_to_le32(DATA_UNIT_SIZE - 1)) {
			dev_err(hba->dev,
				"data segment is misaligned for FMP\n");
			return -EIO;
		}

		/* Set the algorithm and key length. */
		prd->base.size |= cpu_to_le32((FMP_ALGO_MODE_AES_XTS << 28) |
					      (FMP_KEYLEN_256BIT << 26));

		/* Set the IV. */
		prd->file_iv[0] = cpu_to_be64(dun_hi);
		prd->file_iv[1] = cpu_to_be64(dun_lo);

		/* Set the key. */
		for (j = 0; j < AES_KEYSIZE_256 / sizeof(u64); j++) {
			prd->file_enckey[j] = fmp_key_word(enckey, j);
			prd->file_twkey[j] = fmp_key_word(twkey, j);
		}

		/* Increment the data unit number. */
		dun_lo++;
		if (dun_lo == 0)
			dun_hi++;
	}
	return 0;
}

#else /* CONFIG_SCSI_UFS_CRYPTO */

static void exynos_ufs_fmp_init(struct ufs_hba *hba, struct exynos_ufs *ufs)
{
}

#define exynos_ufs_fmp_fill_prdt NULL

#endif /* !CONFIG_SCSI_UFS_CRYPTO */


static int exynos_ufs_init(struct ufs_hba *hba)
{
	struct udevice *dev = hba->dev;
	struct exynos_ufs *ufs = dev_get_priv(dev);
	struct resource res;
	int ret;

	/* exynos-specific hci */
	ret = dev_read_resource_byname(dev, "vs_hci", &res);
	if (ret) {
		dev_err(dev, "cannot ioremap for hci vendor register\n");
		return ret;
	}
	ufs->reg_hci = (void *)res.start;

	/* unipro */
	ret = dev_read_resource_byname(dev, "unipro", &res);
	if (ret) {
		dev_err(dev, "cannot ioremap for unipro register\n");
		return ret;
	}
	ufs->reg_unipro = (void *)res.start;

	/* ufs protector */
	ret = dev_read_resource_byname(dev, "ufsp", &res);
	if (ret) {
		dev_err(dev, "cannot ioremap for ufs protector register\n");
		return ret;
	}
	ufs->reg_ufsp = (void *)res.start;

	ret = exynos_ufs_parse_dt(dev, ufs);
	if (ret) {
		dev_err(dev, "failed to get dt info.\n");
		return ret;
	}

	exynos_ufs_priv_init(hba, ufs);

	exynos_ufs_fmp_init(hba, ufs);

	if (ufs->drv_data->drv_init) {
		ret = ufs->drv_data->drv_init(ufs);
		if (ret) {
			dev_err(dev, "failed to init drv-data\n");
			return ret;
		}
	}

	ret = exynos_ufs_get_clk_info(ufs);
	if (ret)
		return ret;
	exynos_ufs_specify_phy_time_attr(ufs);

	exynos_ufs_config_smu(ufs);
	exynos_ufs_debug_init(&ufs->debug, hba);

	return 0;
}

static int exynos_ufs_host_reset(struct ufs_hba *hba)
{
	struct udevice *dev = hba->dev;
	struct exynos_ufs *ufs = dev_get_priv(dev);
	ulong start = get_timer(0);
	u32 val;
	int ret = 0;

	exynos_ufs_disable_auto_ctrl_hcc_save(ufs, &val);

	hci_writel(ufs, UFS_SW_RST_MASK, HCI_SW_RST);

	do {
		if (!(hci_readl(ufs, HCI_SW_RST) & UFS_SW_RST_MASK))
			goto out;
	} while (get_timer(start) < 1);

	dev_err(dev, "timeout host sw-reset\n");
	ret = -ETIMEDOUT;

out:
	exynos_ufs_auto_ctrl_hcc_restore(ufs, &val);
	return ret;
}

static void exynos_ufs_dev_hw_reset(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);

	hci_writel(ufs, 0 << 0, HCI_GPIO_OUT);
	udelay(5);
	hci_writel(ufs, 1 << 0, HCI_GPIO_OUT);
}

static int exynos_ufs_hce_enable_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);
	int ret = 0;

	switch (status) {
	case PRE_CHANGE:
		if (ufs->drv_data->pre_hce_enable) {
			ret = ufs->drv_data->pre_hce_enable(ufs);
			if (ret)
				return ret;
		}

		ret = exynos_ufs_host_reset(hba);
		if (ret)
			return ret;
		exynos_ufs_dev_hw_reset(hba);
		break;
	case POST_CHANGE:
		exynos_ufs_calc_pwm_clk_div(ufs);
		if (!(ufs->opts & EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL))
			exynos_ufs_enable_auto_ctrl_hcc(ufs);

		if (ufs->drv_data->post_hce_enable)
			ret = ufs->drv_data->post_hce_enable(ufs);

		break;
	}

	return ret;
}

static int exynos_ufs_link_startup_notify(struct ufs_hba *hba,
					  enum ufs_notify_change_status status)
{
	int ret = 0;

	switch (status) {
	case PRE_CHANGE:
		ret = exynos_ufs_pre_link(hba);
		break;
	case POST_CHANGE:
		ret = exynos_ufs_post_link(hba);
		break;
	}

	return ret;
}
#ifdef EXYNOSAUTO_UFS_VH
static int exynosauto_ufs_vh_link_startup_notify(struct ufs_hba *hba,
						 enum ufs_notify_change_status status)
{
	if (status == POST_CHANGE) {
		ufshcd_set_link_active(hba);
		ufshcd_set_ufs_dev_active(hba);
	}

	return 0;
}

static int exynosauto_ufs_vh_wait_ph_ready(struct ufs_hba *hba)
{
	u32 mbox;
	ktime_t start, stop;

	start = ktime_get();
	stop = ktime_add(start, ms_to_ktime(PH_READY_TIMEOUT_MS));

	do {
		mbox = ufshcd_readl(hba, PH2VH_MBOX);
		/* TODO: Mailbox message protocols between the PH and VHs are
		 * not implemented yet. This will be supported later
		 */
		if ((mbox & MH_MSG_MASK) == MH_MSG_PH_READY)
			return 0;

		usleep_range(40, 50);
	} while (ktime_before(ktime_get(), stop));

	return -ETIME;
}

static int exynosauto_ufs_vh_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_ufs *ufs;
	int ret;

	ufs = devm_kzalloc(dev, sizeof(*ufs), GFP_KERNEL);
	if (!ufs)
		return -ENOMEM;

	/* exynos-specific hci */
	ufs->reg_hci = devm_platform_ioremap_resource_byname(pdev, "vs_hci");
	if (IS_ERR(ufs->reg_hci)) {
		dev_err(dev, "cannot ioremap for hci vendor register\n");
		return PTR_ERR(ufs->reg_hci);
	}

	ret = exynosauto_ufs_vh_wait_ph_ready(hba);
	if (ret)
		return ret;

	ufs->drv_data = device_get_match_data(dev);
	if (!ufs->drv_data)
		return -ENODEV;

	exynos_ufs_priv_init(hba, ufs);

	return 0;
}
#endif

static int fsd_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	struct ufs_hba *hba = ufs->hba;
	int i;

	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_clk_period_off),
		       DIV_ROUND_UP(NSEC_PER_SEC,  ufs->mclk_rate));
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x201), 0x12);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0xAA, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x8F, i), 0x3F);
	}

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x12, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x5C, i), 0x38);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x0F, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x65, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x69, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x21, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x22, i), 0x0);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_AUTOMODE_THLD), 0x4E20);

	ufshcd_dme_set(hba, UIC_ARG_MIB(attr->pa_dbg_opt_suite1_off),
		       0x2e820183);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);

	exynos_ufs_establish_connt(ufs);

	return 0;
}

static int fsd_ufs_post_link(struct exynos_ufs *ufs)
{
	int i;
	struct ufs_hba *hba = ufs->hba;
	u32 hw_cap_min_tactivate;
	u32 peer_rx_min_actv_time_cap;
	u32 max_rx_hibern8_time_cap;

	ufshcd_dme_get(hba, UIC_ARG_MIB_SEL(0x8F, 4),
			&hw_cap_min_tactivate); /* HW Capability of MIN_TACTIVATE */
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_TACTIVATE),
			&peer_rx_min_actv_time_cap);    /* PA_TActivate */
	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
			&max_rx_hibern8_time_cap);      /* PA_Hibern8Time */

	if (peer_rx_min_actv_time_cap >= hw_cap_min_tactivate)
		ufshcd_dme_peer_set(hba, UIC_ARG_MIB(PA_TACTIVATE),
					peer_rx_min_actv_time_cap + 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME), max_rx_hibern8_time_cap + 1);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0x01);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0xFA);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0x00);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x35, i), 0x05);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x73, i), 0x01);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x41, i), 0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x42, i), 0xAC);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);

	return 0;
}

static inline u32 get_mclk_period_unipro_18(struct exynos_ufs *ufs)
{
	return (16 * 1000 * 1000000UL / ufs->mclk_rate);
}

static int gs101_ufs_pre_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;

	unipro_writel(ufs, get_mclk_period_unipro_18(ufs), COMP_CLK_PERIOD);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x69);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       (tx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x7F, i), 0);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xA006), 0x8000);

	return 0;
}

static int gs101_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	/*
	 * Enable Write Line Unique. This field has to be 0x3
	 * to support Write Line Unique transaction on gs101.
	 */
	hci_writel(ufs, WLU_EN | WLU_BURST_LEN(3), HCI_AXIDMA_RWDATA_BURST_LEN);

	exynos_ufs_enable_dbg_mode(hba);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0x3e8);
	exynos_ufs_disable_dbg_mode(hba);

	return 0;
}

static int exynos9820_ufs_pre_link(struct exynos_ufs *ufs)
{
	log_debug("%s: ...\n", __func__);
	struct ufs_hba *hba = ufs->hba;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	rx_line_reset_period = (RX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;
	tx_line_reset_period = (TX_LINE_RESET_TIME * ufs->mclk_rate)
				/ NSEC_PER_MSEC;

	unipro_writel(ufs, get_mclk_period_unipro_18(ufs), COMP_CLK_PERIOD);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_rx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2f, i), 0x79);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x76, i), 0x3);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i) {
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
			       (tx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x04, i), 1);
		// ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x7F, i), 0);
	}

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x0);
	
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_LOCAL_TX_LCC_ENABLE), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID), 0x0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(N_DEVICEID_VALID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_PEERDEVICEID), 0x1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), CPORT_CONNECTED);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xA006), 0x8000);

	return 0;
}

static void exynos9820_unipro_adapt_length(struct ufs_hba *hba, u32 addr)
{
	u32 value;

	ufshcd_dme_get(hba, UIC_ARG_MIB(addr), &value);
	if (value & 0x80) {
		if ((value & 0x7F) < 2)
			ufshcd_dme_set(hba, UIC_ARG_MIB(addr), 0x82);
	} else {
		if (((value + 1) % 4) != 0) {
			do {
				value++;
			} while (((value + 1) % 4) != 0);
			ufshcd_dme_set(hba, UIC_ARG_MIB(addr), value);
		}
	}
}

static int exynos9820_ufs_post_link(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;

	exynos9820_unipro_adapt_length(hba, 0x15D2);
	exynos9820_unipro_adapt_length(hba, 0x15D3);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0x01);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0x3E8);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_DBG_MODE), 0x00);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0xA006), 0x80000000);
	udelay(0x7d0);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xA006), 0x0);

	hci_writel(ufs, 0xFFFFFFFF, HCI_UTRL_NEXUS_TYPE);
	hci_writel(ufs, 0xFFFFFFFF, HCI_UTMRL_NEXUS_TYPE);

	/*
	 * Enable Write Line Unique. This field has to be 0x3
	 * to support Write Line Unique transaction on gs101.
	 */
	hci_writel(ufs, WLU_EN | WLU_BURST_LEN(3), HCI_AXIDMA_RWDATA_BURST_LEN);

	exynos_ufs_enable_dbg_mode(hba);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SAVECONFIGTIME), 0x3e8);
	exynos_ufs_disable_dbg_mode(hba);

	struct ufs_pa_layer_attr *p = &hba->pwr_info;
	log_debug("%s: gear_rx = %u, gear_tx = %u, lane_rx = %u, lane_tx = %u, "
		  "pwr_rx = %u, pwr_tx = %u, hs_rate = %u, avail_ln_rx = %d, "
		  "avail_ln_tx = %d\n",
		  __func__, p->gear_rx, p->gear_tx, p->lane_rx, p->lane_tx, p->pwr_rx,
		  p->pwr_tx, p->hs_rate, ufs->avail_ln_rx, ufs->avail_ln_rx);

		exynos_ufs_show_uic_info(hba);

	return 0;
}

static struct ufs_hba_ops ufs_hba_exynos_ops = {
	.init				= exynos_ufs_init,
	.hce_enable_notify		= exynos_ufs_hce_enable_notify,
	.link_startup_notify		= exynos_ufs_link_startup_notify,
};
#ifdef EXYNOSAUTO_UFS_VH
static struct ufs_hba_ops ufs_hba_exynosauto_vh_ops = {
	.init				= exynosauto_ufs_vh_init,
	.link_startup_notify		= exynosauto_ufs_vh_link_startup_notify,
};
#endif
static int exynos_ufs_probe(struct udevice *dev)
{
	int ret;
	struct ufs_hba_ops *ops = &ufs_hba_exynos_ops;
	const struct exynos_ufs_drv_data *drv_data =
		(const struct exynos_ufs_drv_data *)dev_get_driver_data(dev);

	if (drv_data && drv_data->ops)
		ops = drv_data->ops;

	ret = ufshcd_probe(dev, ops);
	if (ret)
		dev_err(dev, "ufshcd_probe() failed %d\n", ret);

	return ret;
}

static struct exynos_ufs_uic_attr exynos7_uic_attr = {
	.tx_trailingclks		= 0x10,
	.tx_dif_p_nsec			= 3000000,	/* unit: ns */
	.tx_dif_n_nsec			= 1000000,	/* unit: ns */
	.tx_high_z_cnt_nsec		= 20000,	/* unit: ns */
	.tx_base_unit_nsec		= 100000,	/* unit: ns */
	.tx_gran_unit_nsec		= 4000,		/* unit: ns */
	.tx_sleep_cnt			= 1000,		/* unit: ns */
	.tx_min_activatetime		= 0xa,
	.rx_filler_enable		= 0x2,
	.rx_dif_p_nsec			= 1000000,	/* unit: ns */
	.rx_hibern8_wait_nsec		= 4000000,	/* unit: ns */
	.rx_base_unit_nsec		= 100000,	/* unit: ns */
	.rx_gran_unit_nsec		= 4000,		/* unit: ns */
	.rx_sleep_cnt			= 1280,		/* unit: ns */
	.rx_stall_cnt			= 320,		/* unit: ns */
	.rx_hs_g1_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g2_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g3_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g1_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g2_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g3_prep_sync_len_cap	= PREP_LEN(0xf),
	.pa_dbg_clk_period_off		= PA_DBG_CLK_PERIOD,
	.pa_dbg_opt_suite1_val		= 0x30103,
	.pa_dbg_opt_suite1_off		= PA_DBG_OPTION_SUITE,
};

static const struct exynos_ufs_drv_data exynosauto_ufs_drvs = {
	.uic_attr		= &exynos7_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
	.iocc_mask		= UFS_EXYNOSAUTO_SHARABLE,
	.drv_init		= exynosauto_ufs_drv_init,
	.post_hce_enable	= exynosauto_ufs_post_hce_enable,
	.pre_link		= exynosauto_ufs_pre_link,
};
#ifdef EXYNOSAUTO_UFS_VH
static const struct exynos_ufs_drv_data exynosauto_ufs_vh_drvs = {
	.ops			= &ufs_hba_exynosauto_vh_ops,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_BROKEN_HCE |
				  UFSHCD_QUIRK_BROKEN_UIC_CMD |
				  UFSHCD_QUIRK_SKIP_PH_CONFIGURATION |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
};
#endif

static const struct exynos_ufs_drv_data exynos_ufs_drvs = {
	.uic_attr		= &exynos7_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCI_QUIRK_BROKEN_HCE |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX |
				  EXYNOS_UFS_OPT_SKIP_CONNECTION_ESTAB |
				  EXYNOS_UFS_OPT_USE_SW_HIBERN8_TIMER,
	.pre_link		= exynos7_ufs_pre_link,
	.post_link		= exynos7_ufs_post_link,
};

static struct exynos_ufs_uic_attr gs101_uic_attr = {
	.tx_trailingclks		= 0xff,
	.pa_dbg_opt_suite1_val		= 0x90913C1C,
	.pa_dbg_opt_suite1_off		= PA_GS101_DBG_OPTION_SUITE1,
	.pa_dbg_opt_suite2_val		= 0xE01C115F,
	.pa_dbg_opt_suite2_off		= PA_GS101_DBG_OPTION_SUITE2,
};

static struct exynos_ufs_uic_attr fsd_uic_attr = {
	.tx_trailingclks		= 0x10,
	.tx_dif_p_nsec			= 3000000,	/* unit: ns */
	.tx_dif_n_nsec			= 1000000,	/* unit: ns */
	.tx_high_z_cnt_nsec		= 20000,	/* unit: ns */
	.tx_base_unit_nsec		= 100000,	/* unit: ns */
	.tx_gran_unit_nsec		= 4000,		/* unit: ns */
	.tx_sleep_cnt			= 1000,		/* unit: ns */
	.tx_min_activatetime		= 0xa,
	.rx_filler_enable		= 0x2,
	.rx_dif_p_nsec			= 1000000,	/* unit: ns */
	.rx_hibern8_wait_nsec		= 4000000,	/* unit: ns */
	.rx_base_unit_nsec		= 100000,	/* unit: ns */
	.rx_gran_unit_nsec		= 4000,		/* unit: ns */
	.rx_sleep_cnt			= 1280,		/* unit: ns */
	.rx_stall_cnt			= 320,		/* unit: ns */
	.rx_hs_g1_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g2_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g3_sync_len_cap		= SYNC_LEN_COARSE(0xf),
	.rx_hs_g1_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g2_prep_sync_len_cap	= PREP_LEN(0xf),
	.rx_hs_g3_prep_sync_len_cap	= PREP_LEN(0xf),
	.pa_dbg_clk_period_off		= PA_DBG_CLK_PERIOD,
	.pa_dbg_opt_suite1_val		= 0x2E820183,
	.pa_dbg_opt_suite1_off		= PA_DBG_OPTION_SUITE,
};

static const struct exynos_ufs_drv_data fsd_ufs_drvs = {
	.uic_attr               = &fsd_uic_attr,
	.quirks                 = UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR,
	.opts                   = EXYNOS_UFS_OPT_HAS_APB_CLK_CTRL |
				  EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_BROKEN_RX_SEL_IDX,
	.pre_link               = fsd_ufs_pre_link,
	.post_link              = fsd_ufs_post_link,
};

static const struct exynos_ufs_drv_data gs101_ufs_drvs = {
	.uic_attr		= &gs101_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING,
	.opts			= EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_UFSPR_SECURE |
				  EXYNOS_UFS_OPT_TIMER_TICK_SELECT,
	.iocc_mask		= UFS_GS101_SHARABLE,
	.drv_init		= gs101_ufs_drv_init,
	.pre_link		= gs101_ufs_pre_link,
	.post_link		= gs101_ufs_post_link,
};

static const struct exynos_ufs_drv_data exynos9820_ufs_drvs = {
	.uic_attr		= &gs101_uic_attr,
	.quirks			= UFSHCD_QUIRK_PRDT_BYTE_GRAN |
				  UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR |
				  UFSHCI_QUIRK_BROKEN_REQ_LIST_CLR |
				  UFSHCD_QUIRK_BROKEN_OCS_FATAL_ERROR |
				  UFSHCI_QUIRK_SKIP_MANUAL_WB_FLUSH_CTRL |
				  UFSHCD_QUIRK_SKIP_DEF_UNIPRO_TIMEOUT_SETTING |
				  UFSHCD_QUIRK_BROKEN_CRYPTO_ENABLE |
				  UFSHCD_QUIRK_BROKEN_INTR_AGGR,
	.opts			= EXYNOS_UFS_OPT_SKIP_CONFIG_PHY_ATTR |
				  EXYNOS_UFS_OPT_BROKEN_AUTO_CLK_CTRL |
				  EXYNOS_UFS_OPT_UFSPR_SECURE |
				  EXYNOS_UFS_OPT_TIMER_TICK_SELECT,
	.iocc_mask		= EXYNOS9820_SHARABLE,
	.drv_init		= exynos9820_ufs_drv_init,
	.pre_link		= exynos9820_ufs_pre_link,
	.post_link		= exynos9820_ufs_post_link,
};

static const struct udevice_id exynos_ufs_of_match[] = {
	{ .compatible = "samsung,exynos9820-ufs",
	  .data	      = (ulong)&exynos9820_ufs_drvs },
	{},
};

U_BOOT_DRIVER(exynos_ufs) = {
	.name		= "exynos-ufshc",
	.id		= UCLASS_UFS,
	.of_match	= exynos_ufs_of_match,
	.probe		= exynos_ufs_probe,
	.priv_auto	= sizeof(struct exynos_ufs),
};
