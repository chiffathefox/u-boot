// SPDX-License-Identifier: GPL-2.0-only
/*
 * UFS Host Controller driver for Exynos specific extensions
 *
 * Copyright (C) 2014-2015 Samsung Electronics Co., Ltd.
 * Author: Seungwon Jeon  <essuuj@gmail.com>
 * Author: Alim Akhtar <alim.akhtar@samsung.com>
 *
 */

#include <clk-uclass.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <generic-phy.h>
#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/log2.h>
#include <linux/time.h>
#include <regmap.h>
#include <syscon.h>
#include <time.h>

#include "ufs.h"
#include "ufs-exynos.h"

#define DATA_UNIT_SIZE		4096

/*
 * Exynos's Vendor specific registers for UFSHCI
 */
#define HCI_TXPRDT_ENTRY_SIZE	0x00
#define PRDT_PREFETCH_EN	BIT(31)
#define PRDT_SET_SIZE(x)	((x) & 0x1F)
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

struct exynos_ufs *g_ufs;

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

static void exynos_ufs_ctrl_clk(struct exynos_ufs *ufs, bool en)
{
	u32 misc = hci_readl(ufs, HCI_MISC);
	if (en) {
		hci_writel(ufs, misc | CLK_CTRL_EN_MASK, HCI_MISC);
	} else {
		hci_writel(ufs, misc & ~CLK_CTRL_EN_MASK, HCI_MISC);
	}
}

static void exynos_ufs_ctrl_clkstop(struct exynos_ufs *ufs, bool en)
{
	u32 ctrl = hci_readl(ufs, HCI_CLKSTOP_CTRL);
	// u32 misc = hci_readl(ufs, HCI_MISC);

#if 1
	if (en) {
		// hci_writel(ufs, misc | CLK_CTRL_EN_MASK, HCI_MISC);
		hci_writel(ufs, ctrl | CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
	} else {
		hci_writel(ufs, ctrl & ~CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
		// hci_writel(ufs, misc & ~CLK_CTRL_EN_MASK, HCI_MISC);
	}
#else

	if (en) {
		hci_writel(ufs, ctrl | CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
		hci_writel(ufs, misc & ~CLK_CTRL_EN_MASK, HCI_MISC);
	} else {
		hci_writel(ufs, misc | CLK_CTRL_EN_MASK, HCI_MISC);
		hci_writel(ufs, ctrl & ~CLK_STOP_MASK, HCI_CLKSTOP_CTRL);
	}
#endif
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

	// TODO:
	// phy_set_bus_width(generic_phy, ufs->avail_ln_rx);

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

	if (ufs->avail_ln_rx == 0 || ufs->avail_ln_tx == 0) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILRXDATALANES),
			&ufs->avail_ln_rx);
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILTXDATALANES),
			&ufs->avail_ln_tx);
		WARN(ufs->avail_ln_rx != ufs->avail_ln_tx,
			"available data lane is not equal(rx:%d, tx:%d)\n",
			ufs->avail_ln_rx, ufs->avail_ln_tx);
	}
	
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

static int exynos_ufs_post_link(struct ufs_hba *hba)
{
	struct udevice *dev = hba->dev;
	struct exynos_ufs *ufs = dev_get_priv(dev);
	struct exynos_ufs_uic_attr *attr = ufs->drv_data->uic_attr;
	u32 val = ilog2(DATA_UNIT_SIZE);
	u32 nutrs = (hba->capabilities & MASK_TRANSFER_REQUESTS_SLOTS_SDB) + 1;
	u32 nutmrs = 
	((hba->capabilities & MASK_TASK_MANAGEMENT_REQUEST_SLOTS) >> 16) + 1;

	exynos_ufs_establish_connt(ufs);
	// exynos_ufs_fit_aggr_timeout(ufs);

	hci_writel(ufs, 0xa, HCI_DATA_REORDER);

	if (hba->caps & UFSHCD_CAP_CRYPTO)
		val |= PRDT_PREFETCH_EN;
	hci_writel(ufs, val, HCI_TXPRDT_ENTRY_SIZE);

	hci_writel(ufs, ilog2(DATA_UNIT_SIZE), HCI_RXPRDT_ENTRY_SIZE);
	hci_writel(ufs, BIT(nutrs) - 1, HCI_UTRL_NEXUS_TYPE);
	hci_writel(ufs, BIT(nutmrs) - 1, HCI_UTMRL_NEXUS_TYPE);
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


	// TODO: questionably? wrong order?
	if (ufs->drv_data->post_link)
		ufs->drv_data->post_link(ufs);
	generic_phy_configure(&ufs->phy, NULL); // TODO: should be calibrate
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xA006), 0x0);

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

#define SMC_CMD_FMP_SECURITY	\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1810)
#define SMC_CMD_FMP_DISK_KEY_SET	\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1830)
#define SMC_CMD_SMU		\
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x1850)
#define SMU_EMBEDDED			0
#define SMU_INIT			0
#define CFG_DESCTYPE_3			3

static void exynos_ufs_fmp_init(struct ufs_hba *hba, struct exynos_ufs *ufs)
{
	// struct blk_crypto_profile *profile = &hba->crypto_profile;
	struct arm_smccc_res res;
	// int err;

	// TODO: if (IS_ENABLED(UFS_CRYPTO))

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
	// BUILD_BUG_ON(sizeof(struct fmp_sg_entry) != 128);
	arm_smccc_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3,
		      0, 0, 0, 0, &res);
	if (res.a0) {
		dev_warn(hba->dev,
			 "SMC_CMD_FMP_SECURITY failed on init: %ld.  Disabling FMP support.\n",
			 res.a0);
		return;
	}
	// ufshcd_set_sg_entry_size(hba, sizeof(struct fmp_sg_entry));

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
#if 0
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
#endif
	/* Advertise crypto support to ufshcd-core. */
	hba->caps |= UFSHCD_CAP_CRYPTO;

	/* Advertise crypto quirks to ufshcd-core. */
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_CRYPTO_PROFILE |
		       UFSHCD_QUIRK_BROKEN_CRYPTO_ENABLE |
		       UFSHCD_QUIRK_KEYS_IN_PRDT;

	// log_debug("%s: done\n", __func__);
}

static void exynos_ufs_debug(struct exynos_ufs *ufs)
{
	struct udevice *dev = ufs->hba->dev;

	dev_dbg(dev, "ufs=%p\n", (void *)ufs);
	dev_dbg(dev, "ufs.reg_hci=%p\n", (void *)ufs->reg_hci);
	dev_dbg(dev, "ufs.reg_unipro=%p\n", (void *)ufs->reg_unipro);
	dev_dbg(dev, "ufs.ref_ufsp=%p\n", (void *)ufs->reg_ufsp);
	dev_dbg(dev, "ufs.pclk_rate=%u\n", ufs->pclk_rate);
	dev_dbg(dev, "ufs.pclk_div=%u\n", ufs->pclk_div);
	dev_dbg(dev, "ufs.pclk_avail_min=%u\n", ufs->pclk_avail_min);
	dev_dbg(dev, "ufs.pclk_avail_max=%u\n", ufs->pclk_avail_max);
	dev_dbg(dev, "ufs.mclk_rate=%lu\n", ufs->mclk_rate);
	dev_dbg(dev, "ufs.avail_ln_rx=%d\n", ufs->avail_ln_rx);
	dev_dbg(dev, "ufs.avail_ln_tx=%d\n", ufs->avail_ln_tx);
	dev_dbg(dev, "ufs.rx_sel_idx=%d\n", ufs->rx_sel_idx);
	dev_dbg(dev, "ufs.systeg.range_count=%d\n", ufs->sysreg->range_count);
	if (ufs->sysreg->range_count > 0) {
		dev_dbg(dev, "ufs.systeg.rages[0].start=0x%lx\n",
			ufs->sysreg->ranges[0].start);
		dev_dbg(dev, "ufs.systeg.rages[0].size=0x%lx\n",
			ufs->sysreg->ranges[0].size);
	}
	dev_dbg(dev, "ufs.iocc_offset=0x%x\n", ufs->iocc_offset);
	dev_dbg(dev, "ufs.iocc_mask=0x%x\n", ufs->iocc_mask);
	dev_dbg(dev, "ufs.iocc_val=0x%x\n", ufs->iocc_val);
	dev_dbg(dev, "ufs.opts=0x%x\n", ufs->opts);
}

static int exynos_ufs_init(struct ufs_hba *hba)
{
	struct udevice *dev = hba->dev;
	struct exynos_ufs *ufs = dev_get_priv(dev);
	struct resource res;
	int ret;
	g_ufs = ufs; // TODO: cleanup

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


	// dev_err(hba->dev, "ufs state from s-boot?\n"); // TODO:  cleanup
	// exynos_ufs_show_uic_info(hba);

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

	// TODO: remove
	exynos_ufs_debug_init(&ufs->debug, hba);

	// dev_err(hba->dev, "post ufs init\n");
	// exynos_ufs_show_uic_info(hba);

	// TODO: remove
	// hba->host->dma_alignment = DATA_UNIT_SIZE - 1;
	exynos_ufs_debug(ufs);

	return 0;
}

static int exynos_ufs_host_reset(struct ufs_hba *hba)
{
	struct udevice *dev = hba->dev;
	struct exynos_ufs *ufs = dev_get_priv(dev);
	ulong start = get_timer(0);
	u32 val;
	int ret = 0;

	dev_dbg(dev, "%s\n", __func__); //TODO: remove

	exynos_ufs_disable_auto_ctrl_hcc_save(ufs, &val);

	dev_dbg(dev, "%s: %u\n", __func__, val); //TODO: remove

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
		/*
		 * TODO: review?
		 * The maximum segment size must be set after scsi_host_alloc()
		 * has been called and before LUN scanning starts
		 * (ufshcd_async_scan()). Note: this callback may also be called
		 * from other functions than ufshcd_init().
		 */
		// hba->host->max_segment_size = DATA_UNIT_SIZE;

		if (ufs->drv_data->pre_hce_enable) {
			ret = ufs->drv_data->pre_hce_enable(ufs);
			if (ret)
				return ret;
		}

		ret = exynos_ufs_host_reset(hba);
		if (ret)
			return ret;
		exynos_ufs_dev_hw_reset(hba);

		// dev_err(hba->dev, "post exynos_ufs_dev_hw_reset\n");// TODO: remove
		// exynos_ufs_show_uic_info(hba);
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
	// const char *name = status == PRE_CHANGE ? "pre_link" : "post_link";

	// dev_err(hba->dev, "pre %s\n", name); // TODO: remove
	// exynos_ufs_show_uic_info(hba);
	
	switch (status) {
	case PRE_CHANGE:
		ret = exynos_ufs_pre_link(hba);
		break;
	case POST_CHANGE:
		ret = exynos_ufs_post_link(hba);
		break;
	}

	// dev_err(hba->dev, "post %s\n", name);
	// exynos_ufs_show_uic_info(hba);

	return ret;
}

static inline u32 get_mclk_period_unipro_18(struct exynos_ufs *ufs)
{
	return (16 * 1000 * 1000000UL / ufs->mclk_rate);
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

/*
 * This is a recommendation from Samsung UFS device vendor.
 *
 * Activate time: host < device
 * Hibern time: host > device
 */
static void exynos9820_ufs_calib_hibern8_values(void *hba)
{
	u32 hw_cap_min_tactivate;
	u32 peer_rx_min_actv_time_cap;
	u32 max_rx_hibern8_time_cap;
// TODO: cleanup
#define RX_LANE_0 4
	ufshcd_dme_get(
		hba, UIC_ARG_MIB_SEL(0x8F, RX_LANE_0),
		&hw_cap_min_tactivate); /* HW Capability of MIN_TACTIVATE */

	ufshcd_dme_get(hba, UIC_ARG_MIB(0x15A8),
		       &peer_rx_min_actv_time_cap); /* PA_TActivate */
	ufshcd_dme_get(hba, UIC_ARG_MIB(0x15A7),
		       &max_rx_hibern8_time_cap); /* PA_Hibern8Time */

	if (peer_rx_min_actv_time_cap >= hw_cap_min_tactivate)
		ufshcd_dme_peer_set(hba, UIC_ARG_MIB(0x15A8),
				    peer_rx_min_actv_time_cap + 1);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x15A7), max_rx_hibern8_time_cap + 1);
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
	generic_phy_configure(&ufs->phy, NULL);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0xA006), 0x0);

	exynos9820_ufs_calib_hibern8_values(hba);

	// TODO: remove
	struct ufs_pa_layer_attr *p = &hba->pwr_info;
	log_debug("%s: gear_rx = %u, gear_tx = %u, lane_rx = %u, lane_tx = %u, "
		  "pwr_rx = %u, pwr_tx = %u, hs_rate = %u, avail_ln_rx = %d, "
		  "avail_ln_tx = %d\n",
		  __func__, p->gear_rx, p->gear_tx, p->lane_rx, p->lane_tx,
		  p->pwr_rx, p->pwr_tx, p->hs_rate, ufs->avail_ln_rx,
		  ufs->avail_ln_rx);
	// exynos_ufs_show_uic_info(hba);

	return 0;
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

static int exynos9820_ufs_drv_init(struct exynos_ufs *ufs)
{
	exynos_ufs_disable_auto_ctrl_hcc(ufs);
	exynos_ufs_ungate_clks(ufs);
	exynos_ufs_set_hwacg_control(ufs, false);
	exynos_ufs_phy_init(ufs);

	return exynos_ufs_shareability(ufs);
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
	// XXX: TODO: wtf
	hci_writel(ufs, 0x000000a6, HCI_1US_TO_CNT_VAL);
}

static void exynos9820_ufs_ctrl_cport_log(struct exynos_ufs *ufs)
{
	hci_writel(ufs, 0, 0x114);
	hci_writel(ufs, 1, 0x110);
}

static int exynos9820_ufs_pre_hce_enable(struct exynos_ufs *ufs)
{
	struct ufs_hba *hba = ufs->hba;
	u32 reg;

	exynos_ufs_disable_auto_ctrl_hcc(ufs);
	exynos_ufs_fit_aggr_timeout(ufs);

	/* misc HCI configurations */
	hci_writel(ufs, 0xA, HCI_DATA_REORDER);
	hci_writel(ufs, PRDT_PREFETCH_EN | PRDT_SET_SIZE(12),
		   HCI_TXPRDT_ENTRY_SIZE);
	hci_writel(ufs, PRDT_SET_SIZE(12), HCI_RXPRDT_ENTRY_SIZE);
	hci_writel(ufs, 0xFFFFFFFF, HCI_UTRL_NEXUS_TYPE);
	hci_writel(ufs, 0xFFFFFFFF, HCI_UTMRL_NEXUS_TYPE);

	reg = hci_readl(ufs, HCI_AXIDMA_RWDATA_BURST_LEN) & ~WLU_BURST_LEN(0);
	hci_writel(ufs, WLU_EN | WLU_BURST_LEN(3), HCI_AXIDMA_RWDATA_BURST_LEN);

	/*
	 * Enable HWAGC control by IOP
	 *
	 * default value 1->0 at KC.
	 * always "0"(controlled by UFS_ACG_DISABLE)
	 */
	reg = hci_readl(ufs, HCI_IOP_ACG_DISABLE);
	hci_writel(ufs, reg & (~HCI_IOP_ACG_DISABLE_EN), HCI_IOP_ACG_DISABLE);

	/* device reset */
	exynos_ufs_dev_hw_reset(hba);

	exynos9820_ufs_ctrl_cport_log(ufs);

	return 0;
}

static int
exynos9820_ufs_hce_enable_notify(struct ufs_hba *hba,
				 enum ufs_notify_change_status status)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);
	int ret = 0;

	dev_dbg(hba->dev, "%s: status=%d\n", __func__, status);

	switch (status) {
	case PRE_CHANGE:
		ret = exynos9820_ufs_pre_hce_enable(ufs);
		break;
	case POST_CHANGE:
		break;
	}

	return ret;
}

static int exynos9820_ufs_pre_link(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);
	int ret = 0;
	int i;
	u32 tx_line_reset_period, rx_line_reset_period;

	/* hci */
	exynos_ufs_config_intr(ufs, DFES_DEF_L2_ERRS, UNIPRO_L2);
	exynos_ufs_config_intr(ufs, DFES_DEF_L3_ERRS, UNIPRO_L3);
	exynos_ufs_config_intr(ufs, DFES_DEF_L4_ERRS, UNIPRO_L4);

	exynos_ufs_ctrl_clk(ufs, true);
	exynos_ufs_ungate_clks(ufs);
	exynos_ufs_set_hwacg_control(ufs, false);

	if (ufs->avail_ln_rx == 0 || ufs->avail_ln_tx == 0) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILRXDATALANES),
			       &ufs->avail_ln_rx);
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_AVAILTXDATALANES),
			       &ufs->avail_ln_tx);
		WARN(ufs->avail_ln_rx != ufs->avail_ln_tx,
		     "available data lane is not equal(rx:%d, tx:%d)\n",
		     ufs->avail_ln_rx, ufs->avail_ln_tx);
	}

	rx_line_reset_period =
		(RX_LINE_RESET_TIME * ufs->mclk_rate) / NSEC_PER_MSEC;
	tx_line_reset_period =
		(TX_LINE_RESET_TIME * ufs->mclk_rate) / NSEC_PER_MSEC;

	unipro_writel(ufs, get_mclk_period_unipro_18(ufs), COMP_CLK_PERIOD);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x200), 0x40);

	for_each_ufs_rx_lane(ufs, i)
	{
		dev_info(hba->dev, "%s: rx lane %i\n", __func__,
			 i); // TODO: remove
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_CLK_PRD_EN, i), 0x0);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE2, i),
			       (rx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE1, i),
			       (rx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_RX_LINERESET_VALUE0, i),
			       (rx_line_reset_period) & 0xFF);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x2F, i), 0x79);
		// ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x76, i), 0x3);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x84, i), 0x1);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(0x25, i), 0xf6);
	}

	for_each_ufs_tx_lane(ufs, i)
	{
		dev_info(hba->dev, "%s: tx lane %i\n", __func__, i);
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD, i),
			       DIV_ROUND_UP(NSEC_PER_SEC, ufs->mclk_rate));
		ufshcd_dme_set(hba, UIC_ARG_MIB_SEL(VND_TX_CLK_PRD_EN, i),
			       0x02);
		ufshcd_dme_set(hba,
			       UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE2, i),
			       (tx_line_reset_period >> 16) & 0xFF);
		ufshcd_dme_set(hba,
			       UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE1, i),
			       (tx_line_reset_period >> 8) & 0xFF);
		ufshcd_dme_set(hba,
			       UIC_ARG_MIB_SEL(VND_TX_LINERESET_PVALUE0, i),
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

	ret = generic_phy_configure(&ufs->phy, NULL);

	return ret;
}

static int
exynos9820_ufs_link_startup_notify(struct ufs_hba *hba,
				   enum ufs_notify_change_status status)
{
	int ret = 0;
	// const char *name = status == PRE_CHANGE ? "pre_link" : "post_link";
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);

	// dev_err(hba->dev, "pre %s\n", name); // TODO: remove
	// exynos_ufs_show_uic_info(hba);

	switch (status) {
	case PRE_CHANGE:
		ret = exynos9820_ufs_pre_link(hba);
		break;
	case POST_CHANGE:
	// TODO: cleaqnup arguments
		ret = exynos9820_ufs_post_link(ufs);
		break;
	}

	// dev_err(hba->dev, "post %s\n", name);
	// exynos_ufs_show_uic_info(hba);

	return ret;
}

static int exynos9820_ufs_pre_pwr_change(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);

	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TXHSADAPTTYPE), 0x1);

	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_FC0PROTTIMEOUTVAL), 8064);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_TC0REPLAYTIMEOUTVAL), 28224);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DL_AFC0REQTIMEOUTVAL), 20160);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 16000);

	ufshcd_dme_set(hba, UIC_ARG_MIB(0x7888), 8064);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x788C), 28224);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x7890), 20160);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x78B8), 12000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x78BC), 32000);
	ufshcd_dme_set(hba, UIC_ARG_MIB(0x78C0), 16000);

	return generic_phy_configure(&ufs->phy, NULL);
}

int exynos9820_ufs_post_pwr_change(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);

	return generic_phy_configure(&ufs->phy, NULL);
}

static int
exynos9820_ufs_get_max_pwr_mode(struct ufs_hba *hba,
				struct ufs_pwr_mode_info *max_pwr_info)
{
	dev_dbg(hba->dev,
		"%s: max_pwr_info: valid=%d gear_rx=%u gear_tx=%u lane_rx=%u lane_tx=%u pwr_rx=%u pwr_tx=%u hs_rate=%u\n",
		__func__, max_pwr_info->is_valid, max_pwr_info->info.gear_rx,
		max_pwr_info->info.gear_tx, max_pwr_info->info.lane_rx,
		max_pwr_info->info.lane_tx, max_pwr_info->info.pwr_rx,
		max_pwr_info->info.pwr_tx, max_pwr_info->info.hs_rate);

	return exynos9820_ufs_pre_pwr_change(hba);
}

static struct ufs_hba_ops ufs_hba_exynos_ops = {
	.init				= exynos_ufs_init,
	.hce_enable_notify		= exynos_ufs_hce_enable_notify,
	.link_startup_notify		= exynos_ufs_link_startup_notify,
};

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

static struct ufs_hba_ops ufs_hba_exynos9820_ops = {
	.init				= exynos_ufs_init,
	.device_reset			= exynos_ufs_host_reset,
	.hce_enable_notify		= exynos9820_ufs_hce_enable_notify,
	.link_startup_notify		= exynos9820_ufs_link_startup_notify,
	.get_max_pwr_mode		= exynos9820_ufs_get_max_pwr_mode,
};

static struct exynos_ufs_uic_attr exynos9820_uic_attr = {
	.tx_trailingclks		= 0xff,
};

static const struct exynos_ufs_drv_data exynos9820_ufs_drvs = {
	.ops			= &ufs_hba_exynos9820_ops,
	.uic_attr		= &exynos9820_uic_attr,
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
// TODO: ufs: cleanup
 void phy_pma_writel(u32 val, u32 reg)
{
	struct exynos_ufs *ufs = g_ufs;
	// u32 reg1 = hci_readl(ufs, HCI_CLKSTOP_CTRL);

	// hci_writel(ufs, reg1 & ~MPHY_APBCLK_STOP, HCI_CLKSTOP_CTRL);
	dev_info(ufs->hba->dev, "%s: 0x%x -> 0x%x\n", __func__, val, reg);
	writel(val, (void*)0x13d64000 + reg);
	// hci_writel(ufs, reg1 | MPHY_APBCLK_STOP, HCI_CLKSTOP_CTRL);
}


void exynos_ufs_set_nexus_t_xfer_req(struct ufs_hba *hba,
				int tag, void *cmd)
{
	struct exynos_ufs *ufs = dev_get_priv(hba->dev);
	u32 type;

	type =  hci_readl(ufs, HCI_UTRL_NEXUS_TYPE);

	if (cmd)
		type |= (1 << tag);
	else
		type &= ~(1 << tag);

	hci_writel(ufs, type, HCI_UTRL_NEXUS_TYPE);
}

struct fmp_table_setting {
	__le32 des0;		/* des0 */
#define GET_CMDQ_LENGTH(d) \
	(((d)->des0 & 0xffff0000) >> 16)
	__le32 des1;		/* des1 */
	__le32 des2;		/* des2 */
	__le32 des3;		/* des3 */
/* Legacy Operation */
#define FKL BIT(26)
#define DKL BIT(27)
#define SET_KEYLEN(d, v) ((d)->des3 |= (uint32_t)v)
#define SET_FAS(d, v) \
	((d)->des3 = ((d)->des3 & 0xcfffffff) | v << 28)
#define SET_DAS(d, v) \
	((d)->des3 = ((d)->des3 & 0x3fffffff) | v << 30)
#define GET_FAS(d)	((d)->des3 & 0x30000000)
#define GET_DAS(d)	((d)->des3 & 0xc0000000)
#define GET_LENGTH(d) \
	((d)->des3 & 0x3ffffff)
/* CMDQ Operation */
#define FKL_CMDQ BIT(0)
#define DKL_CMDQ BIT(1)
#define SET_CMDQ_KEYLEN(d, v) ((d)->des3 |= (uint32_t)v)
#define SET_CMDQ_FAS(d, v) \
	((d)->des3 = ((d)->des3 & 0xfffffff3) | v << 2)
#define SET_CMDQ_DAS(d, v) \
	((d)->des3 = ((d)->des3 & 0xffffffcf) | v << 4)
#define GET_CMDQ_FAS(d)	((d)->des3 & 0x0000000c)
#define GET_CMDQ_DAS(d)	((d)->des3 & 0x00000030)
	__le32 file_iv0;	/* des4 */
	__le32 file_iv1;	/* des5 */
	__le32 file_iv2;	/* des6 */
	__le32 file_iv3;	/* des7 */
	__le32 file_enckey0;	/* des8 */
	__le32 file_enckey1;	/* des9 */
	__le32 file_enckey2;	/* des10 */
	__le32 file_enckey3;	/* des11 */
	__le32 file_enckey4;	/* des12 */
	__le32 file_enckey5;	/* des13 */
	__le32 file_enckey6;	/* des14 */
	__le32 file_enckey7;	/* des15 */
	__le32 file_twkey0;	/* des16 */
	__le32 file_twkey1;	/* des17 */
	__le32 file_twkey2;	/* des18 */
	__le32 file_twkey3;	/* des19 */
	__le32 file_twkey4;	/* des20 */
	__le32 file_twkey5;	/* des21 */
	__le32 file_twkey6;	/* des22 */
	__le32 file_twkey7;	/* des23 */
	__le32 disk_iv0;	/* des24 */
	__le32 disk_iv1;	/* des25 */
	__le32 disk_iv2;	/* des26 */
	__le32 disk_iv3;	/* des27 */
	__le32 reserved0;	/* des28 */
	__le32 reserved1;	/* des29 */
	__le32 reserved2;	/* des30 */
	__le32 reserved3;	/* des31 */
};

enum fmp_crypto_algo_mode {
	EXYNOS_FMP_BYPASS_MODE = 0,
	EXYNOS_FMP_ALGO_MODE_AES_CBC = 1,
	EXYNOS_FMP_ALGO_MODE_AES_XTS = 2,
};

#define FMP_IV_SIZE_16		16
#define WORD_SIZE 4
#define FMP_IV_MAX_IDX (FMP_IV_SIZE_16 / WORD_SIZE)

#define byte2word(b0, b1, b2, b3)       \
			(((unsigned int)(b0) << 24) | \
			((unsigned int)(b1) << 16) | \
			((unsigned int)(b2) << 8) | (b3))
#define get_word(x, c)  byte2word(((unsigned char *)(x) + 4 * (c))[0], \
				((unsigned char *)(x) + 4 * (c))[1], \
				((unsigned char *)(x) + 4 * (c))[2], \
				((unsigned char *)(x) + 4 * (c))[3])

#include <scsi.h>
void exynos9820_ufs_fmp_fill_prdt(struct ufshcd_sg_entry *entry,
				  struct scsi_cmd *pccb)
{
	struct fmp_table_setting *table = (struct fmp_table_setting *)entry;


	u64 sector = (((u64)pccb->cmd[2]) << 24) | (((u64)pccb->cmd[3]) << 16) |
		     (((u64)pccb->cmd[4]) << 8) | ((u64)pccb->cmd[5]);
	debug("sector=%llu\n", sector);

	SET_FAS(table, 0);
	SET_DAS(table, 0);
return;
	SET_DAS(table, EXYNOS_FMP_ALGO_MODE_AES_XTS);

	struct arm_smccc_res res;
	arm_smccc_smc(SMC_CMD_FMP_DISK_KEY_SET, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		pr_err("SMC_CMD_FMP_DISK_KEY_SET() failed: %ld.\n", res.a0);
	}

	SET_KEYLEN(table, DKL);
	u8 iv[FMP_IV_SIZE_16];
	memset(iv, 0, FMP_IV_SIZE_16);
	memcpy(iv, &sector, sizeof(sector));

	int idx;
	for (idx = 0; idx < FMP_IV_MAX_IDX; idx++)
		*(&table->disk_iv0 + idx) =
			get_word(iv, FMP_IV_MAX_IDX - (idx + 1));
}
