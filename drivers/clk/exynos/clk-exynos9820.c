// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung exynos9820 clock driver.
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 * Author: chiffathefox <chiffathefoxx@gmail.com>
 */

#include "linux/clk-provider.h"
#include <dm.h>
#include <asm/io.h>
#include <dt-bindings/clock/exynos9820.h>
#include "clk.h"

enum exynos9820_cmu_id {
	CMU_PERIC0,
	CMU_FSYS1,
	CMU_TOP,
};

/*
 * Register offsets for CMU_PERIC0 (0x10400000)
 */
#define PLL_CON0_MUX_CLKCMU_PERIC0_BUS_USER				0x0100
#define PLL_CON0_MUX_CLKCMU_PERIC0_UART_DBG				0x0120
#define CLK_CON_DIV_DIV_CLK_PERIC0_UART_DBG				0x1800
#define CLK_CON_GAT_PERIC0_CMU_PERIC0_PCLK				0x2004
#define CLK_CON_GAT_PERIC0_UART_DBG_CLK					0x2028
#define CLK_CON_GAT_PERIC0_UART_DBG_IPCLK				0x205c
#define CLK_CON_GAT_PERIC0_UART_DBG_PCLK				0x2060

/* List of parent clocks for muxes in CMU_PERIC0 */
PNAME(mout_peric0_bus_user_p) = { "oscclk", "bus" };
PNAME(mout_peric0_uart_dbg_p) = { "oscclk", "ip" };

static const struct samsung_mux_clock peric0_mux_clks[] = {
	MUX(CLK_MOUT_PERIC0_BUS_USER, "mout_peric0_bus_user",
	    mout_peric0_bus_user_p, PLL_CON0_MUX_CLKCMU_PERIC0_BUS_USER, 4, 1),
	MUX(CLK_MOUT_PERIC0_UART_DBG, "mout_peric0_uart_dbg",
	    mout_peric0_uart_dbg_p, PLL_CON0_MUX_CLKCMU_PERIC0_UART_DBG, 4, 1),
};

static const struct samsung_div_clock peric0_div_clks[] = {
	DIV(CLK_DOUT_PERIC0_UART_DBG, "dout_peric0_uart_dbg",
	    "mout_peric0_uart_dbg", CLK_CON_DIV_DIV_CLK_PERIC0_UART_DBG, 0, 4),
};

static const struct samsung_gate_clock peric0_gate_clks[] = {
	GATE(CLK_GOUT_PERIC0_CMU_PERIC0_PCLK, "gout_peric0_cmu_peric0_pclk",
	     "mout_peric0_bus_user", CLK_CON_GAT_PERIC0_CMU_PERIC0_PCLK, 21,
	     CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_PERIC0_UART_DBG_CLK, "gout_peric0_uart_dbg_clk",
	     "dout_peric0_uart_dbg", CLK_CON_GAT_PERIC0_UART_DBG_CLK, 21,
	     CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERIC0_UART_DBG_IPCLK, "gout_peric0_uart_dbg_ipclk",
	     "dout_peric0_uart_dbg", CLK_CON_GAT_PERIC0_UART_DBG_IPCLK, 21,
	     CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_PERIC0_UART_DBG_PCLK, "gout_peric0_uart_dbg_pclk",
	     "mout_peric0_bus_user", CLK_CON_GAT_PERIC0_UART_DBG_PCLK, 21,
	     CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_clk_group peric0_cmu_clks[] = {
	{ S_CLK_MUX, peric0_mux_clks, ARRAY_SIZE(peric0_mux_clks) },
	{ S_CLK_DIV, peric0_div_clks, ARRAY_SIZE(peric0_div_clks) },
	{ S_CLK_GATE, peric0_gate_clks, ARRAY_SIZE(peric0_gate_clks) },
};

static int exynos9820_cmu_peric0_probe(struct udevice *dev)
{
	return samsung_register_cmu(dev, CMU_PERIC0, peric0_cmu_clks,
				    exynos9820_cmu_peric0);
}

static const struct udevice_id exynos9820_cmu_peric0_ids[] = {
	{ .compatible = "samsung,exynos9820-cmu-peric0" },
	{ }
};

SAMSUNG_CLK_OPS(exynos9820_cmu_peric0, CMU_PERIC0);

U_BOOT_DRIVER(exynos9820_cmu_peric0) = {
	.name		= "exynos9820-cmu-peric0",
	.id		= UCLASS_CLK,
	.of_match	= exynos9820_cmu_peric0_ids,
	.ops		= &exynos9820_cmu_peric0_clk_ops,
	.probe		= exynos9820_cmu_peric0_probe,
	.flags		= DM_FLAG_PRE_RELOC,
};

/*
 * Register offsets for CMU_FSYS1 (0x13c00000)
 */
#define PLL_CON0_MUX_CLKCMU_FSYS1_BUS_USER				0x0100
#define PLL_CON0_MUX_CLKCMU_FSYS1_UFS_EMBD_USER				0x0180
#define PLL_CON0_PLL_MMC						0x01a0
#define CLK_CON_MUX_MUX_CLK_FSYS1_BUS					0x1000
#define CLK_CON_DIV_DIV_CLK_FSYS1_BUS					0x1800
#define CLK_CON_GAT_FSYS1_CMU_FSYS1_PCLK				0x2018
#define CLK_CON_GAT_FSYS1_UFS_EMBD_I_ACLK				0x2094
#define CLK_CON_GAT_FSYS1_UFS_EMBD_I_CLK_UNIPRO				0x2098
#define CLK_CON_GAT_FSYS1_UFS_EMBD_I_FMP_CLK				0x209c

static const struct samsung_pll_clock fsys1_pll_clks[] = {
	PLL(pll_0831x, CLK_FOUT_PLL_MMC, "fout_pll_mmc", "oscclk",
	    PLL_CON0_PLL_MMC),
};

/* List of parent clocks for muxes in CMU_FSYS1 */
PNAME(mout_fsys1_bus_user_p) = { "oscclk", "bus" };
PNAME(mout_fsys1_ufs_embd_user_p) = { "oscclk", "ufs_embd" };
PNAME(mout_fsys1_bus_p) = { "mout_fsys1_bus_user", "fout_pll_mmc" };

static const struct samsung_mux_clock fsys1_mux_clks[] = {
	MUX(CLK_MOUT_FSYS1_BUS_USER, "mout_fsys1_bus_user",
	    mout_fsys1_bus_user_p, PLL_CON0_MUX_CLKCMU_FSYS1_BUS_USER, 4, 1),
	MUX(CLK_MOUT_FSYS1_UFS_EMBD_USER, "mout_fsys1_ufs_embd_user",
	    mout_fsys1_ufs_embd_user_p, PLL_CON0_MUX_CLKCMU_FSYS1_UFS_EMBD_USER,
	    4, 1),
	MUX(CLK_MOUT_FSYS1_BUS, "mout_fsys1_bus", mout_fsys1_bus_p,
	    CLK_CON_MUX_MUX_CLK_FSYS1_BUS, 0, 1),
};

static const struct samsung_div_clock fsys1_div_clks[] = {
	DIV(CLK_DOUT_FSYS1_BUS, "dout_fsys1_bus", "mout_fsys1_bus",
	    CLK_CON_DIV_DIV_CLK_FSYS1_BUS, 0, 4),
};

static const struct samsung_gate_clock fsys1_gate_clks[] = {
	GATE(CLK_GOUT_FSYS1_CMU_FSYS1_PCLK, "gout_fsys1_cmu_fsys1_pclk",
	     "dout_fsys1_bus", CLK_CON_GAT_FSYS1_CMU_FSYS1_PCLK, 21,
	     CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_FSYS1_UFS_EMBD_I_ACLK, "gout_fsys1_ufs_embd_i_aclk",
	     "dout_fsys1_bus", CLK_CON_GAT_FSYS1_UFS_EMBD_I_ACLK, 21,
	     CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_FSYS1_UFS_EMBD_I_CLK_UNIPRO,
	     "gout_fsys1_ufs_embd_i_clk_unipro", "mout_fsys1_ufs_embd_user",
	     CLK_CON_GAT_FSYS1_UFS_EMBD_I_CLK_UNIPRO, 21, CLK_SET_RATE_PARENT,
	     0),
	GATE(CLK_GOUT_FSYS1_UFS_EMBD_I_FMP_CLK, "gout_fsys1_ufs_embd_i_fmp_clk",
	     "dout_fsys1_bus", CLK_CON_GAT_FSYS1_UFS_EMBD_I_FMP_CLK, 21,
	     CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_clk_group fsys1_cmu_clks[] = {
	{ S_CLK_PLL, fsys1_pll_clks, ARRAY_SIZE(fsys1_pll_clks) },
	{ S_CLK_MUX, fsys1_mux_clks, ARRAY_SIZE(fsys1_mux_clks) },
	{ S_CLK_DIV, fsys1_div_clks, ARRAY_SIZE(fsys1_div_clks) },
	{ S_CLK_GATE, fsys1_gate_clks, ARRAY_SIZE(fsys1_gate_clks) },
};

static int exynos9820_cmu_fsys1_probe(struct udevice *dev)
{
	return samsung_register_cmu(dev, CMU_FSYS1, fsys1_cmu_clks,
				    exynos9820_cmu_fsys1);
}

static const struct udevice_id exynos9820_cmu_fsys1_ids[] = {
	{ .compatible = "samsung,exynos9820-cmu-fsys1" },
	{ }
};

SAMSUNG_CLK_OPS(exynos9820_cmu_fsys1, CMU_FSYS1);

U_BOOT_DRIVER(exynos9820_cmu_fsys1) = {
	.name		= "exynos9820-cmu-fsys1",
	.id		= UCLASS_CLK,
	.of_match	= exynos9820_cmu_fsys1_ids,
	.ops		= &exynos9820_cmu_fsys1_clk_ops,
	.probe		= exynos9820_cmu_fsys1_probe,
	.flags		= DM_FLAG_PRE_RELOC,
};

/*
 * Register offsets for CMU_TOP (0x1a240000)
 */
#define PLL_CON0_APLL_SHARED0						0x0100
#define PLL_CON0_PLL_SHARED0						0x0160
#define PLL_CON0_PLL_SHARED2						0x01a0
#define PLL_CON0_PLL_SHARED3						0x01c0
#define PLL_CON0_PLL_SHARED4						0x01e0
#define CLK_CON_MUX_MUX_CLKCMU_FSYS0A_USB31DRD				0x1054
#define CLK_CON_MUX_MUX_CLKCMU_FSYS1_BUS				0x1068
#define CLK_CON_MUX_MUX_CLKCMU_FSYS1_UFS_EMBD				0x1078
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_BUS				0x10b8
#define CLK_CON_MUX_MUX_CLKCMU_PERIC0_IP				0x10bc
#define CLK_CON_MUX_MUX_PLL_SHARED0					0x10dc
#define CLK_CON_DIV_CLKCMU_FSYS0A_USB31DRD				0x1848
#define CLK_CON_DIV_CLKCMU_FSYS1_UFS_EMBD				0x1864
#define CLK_CON_DIV_CLKCMU_PERIC0_BUS					0x18a4
#define CLK_CON_DIV_CLKCMU_PERIC0_IP					0x18a8
#define CLK_CON_DIV_PLL_SHARED0_DIV2					0x18cc
#define CLK_CON_DIV_PLL_SHARED0_DIV4					0x18d4
#define CLK_CON_DIV_PLL_SHARED2_DIV2					0x18e4
#define CLK_CON_DIV_PLL_SHARED3_DIV2					0x18e8
#define CLK_CON_DIV_PLL_SHARED4_DIV2					0x18ec
#define CLK_CON_GAT_CLKCMU_FSYS1_BUS					0x2000
#define CLK_CON_GAT_GATE_CLKCMU_FSYS0A_USB31DRD				0x2064
#define CLK_CON_GAT_GATE_CLKCMU_FSYS1_UFS_EMBD				0x2080
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_BUS				0x20bc
#define CLK_CON_GAT_GATE_CLKCMU_PERIC0_IP				0x20c0

static const struct samsung_pll_clock top_pll_clks[] = {
	PLL(pll_0822x, CLK_FOUT_APLL_SHARED0, "fout_apll_shared0", "oscclk",
	    PLL_CON0_APLL_SHARED0),
	PLL(pll_0817x, CLK_FOUT_PLL_SHARED0, "fout_pll_shared0", "oscclk",
	    PLL_CON0_PLL_SHARED0),
	PLL(pll_0818x, CLK_FOUT_PLL_SHARED2, "fout_pll_shared2", "oscclk",
	    PLL_CON0_PLL_SHARED2),
	PLL(pll_0818x, CLK_FOUT_PLL_SHARED3, "fout_pll_shared3", "oscclk",
	    PLL_CON0_PLL_SHARED3),
	PLL(pll_1018x, CLK_FOUT_PLL_SHARED4, "fout_pll_shared4", "oscclk",
	    PLL_CON0_PLL_SHARED4),
};

/* List of parent clocks for muxes in CMU_TOP */
PNAME(mout_pll_shared0_p) = { "fout_pll_shared0", "fout_apll_shared0" };
PNAME(mout_cmu_peric0_bus_p) = { "dout_pll_shared0_div4",
				 "dout_pll_shared2_div2" };
PNAME(mout_cmu_peric0_ip_p) = { "dout_pll_shared0_div4",
				"dout_pll_shared2_div2" };
PNAME(mout_cmu_fsys1_bus_p) = { "dout_pll_shared0_div4",
				"dout_pll_shared2_div2",
				"dout_pll_shared4_div2",
				"dout_pll_shared3_div2" };
PNAME(mout_cmu_fsys1_ufs_embd_p) = { "oscclk", "dout_pll_shared0_div4",
				     "dout_pll_shared2_div2",
				     "dout_pll_shared4_div2" };
PNAME(mout_cmu_fsys0a_usb31drd_p) = { "oscclk", "dout_pll_shared0_div4",
				      "dout_pll_shared2_div2",
				      "dout_pll_shared4_div2" };

static const struct samsung_mux_clock top_pll_mux_clks[] = {
	MUX(CLK_MOUT_PLL_SHARED0, "mout_pll_shared0", mout_pll_shared0_p,
	    CLK_CON_MUX_MUX_PLL_SHARED0, 0, 1),
};

static const struct samsung_div_clock top_pll_div_clks[] = {
	DIV(CLK_DOUT_PLL_SHARED0_DIV2, "dout_pll_shared0_div2",
	    "mout_pll_shared0", CLK_CON_DIV_PLL_SHARED0_DIV2, 0, 1),
	DIV(CLK_DOUT_PLL_SHARED0_DIV4, "dout_pll_shared0_div4",
	    "dout_pll_shared0_div2", CLK_CON_DIV_PLL_SHARED0_DIV4, 0, 1),
	DIV(CLK_DOUT_PLL_SHARED2_DIV2, "dout_pll_shared2_div2",
	    "fout_pll_shared2", CLK_CON_DIV_PLL_SHARED2_DIV2, 0, 1),
	DIV(CLK_DOUT_PLL_SHARED3_DIV2, "dout_pll_shared3_div2",
	    "fout_pll_shared3", CLK_CON_DIV_PLL_SHARED3_DIV2, 0, 1),
	DIV(CLK_DOUT_PLL_SHARED4_DIV2, "dout_pll_shared4_div2",
	    "fout_pll_shared4", CLK_CON_DIV_PLL_SHARED4_DIV2, 0, 1),
};

static const struct samsung_mux_clock top_mux_clks[] = {
	MUX(CLK_MOUT_CMU_PERIC0_BUS, "mout_cmu_peric0_bus",
	    mout_cmu_peric0_bus_p, CLK_CON_MUX_MUX_CLKCMU_PERIC0_BUS, 0, 1),
	MUX(CLK_MOUT_CMU_PERIC0_IP, "mout_cmu_peric0_ip", mout_cmu_peric0_ip_p,
	    CLK_CON_MUX_MUX_CLKCMU_PERIC0_IP, 0, 1),
	MUX(CLK_MOUT_CMU_FSYS1_BUS, "mout_cmu_fsys1_bus", mout_cmu_fsys1_bus_p,
	    CLK_CON_MUX_MUX_CLKCMU_FSYS1_BUS, 0, 2),
	MUX(CLK_MOUT_CMU_FSYS1_UFS_EMBD, "mout_cmu_fsys1_ufs_embd",
	    mout_cmu_fsys1_ufs_embd_p, CLK_CON_MUX_MUX_CLKCMU_FSYS1_UFS_EMBD, 0,
	    2),
	MUX(CLK_MOUT_CMU_FSYS0A_USB31DRD, "mout_cmu_fsys0a_usb31drd",
	    mout_cmu_fsys0a_usb31drd_p, CLK_CON_MUX_MUX_CLKCMU_FSYS0A_USB31DRD,
	    0, 2),
};

static const struct samsung_gate_clock top_gate_clks[] = {
	GATE(CLK_GOUT_CMU_PERIC0_BUS, "gout_cmu_peric0_bus",
	     "mout_cmu_peric0_bus", CLK_CON_GAT_GATE_CLKCMU_PERIC0_BUS, 21,
	     CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_PERIC0_IP, "gout_cmu_peric0_ip", "mout_cmu_peric0_ip",
	     CLK_CON_GAT_GATE_CLKCMU_PERIC0_IP, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_FSYS1_BUS, "gout_cmu_fsys1_bus", "mout_cmu_fsys1_bus",
	     CLK_CON_GAT_CLKCMU_FSYS1_BUS, 21, CLK_IS_CRITICAL, 0),
	GATE(CLK_GOUT_CMU_FSYS1_UFS_EMBD, "gout_cmu_fsys1_ufs_embd",
	     "mout_cmu_fsys1_ufs_embd", CLK_CON_GAT_GATE_CLKCMU_FSYS1_UFS_EMBD,
	     21, CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_CMU_FSYS0A_USB31DRD, "gout_cmu_fsys0a_usb31drd",
	     "mout_cmu_fsys0a_usb31drd",
	     CLK_CON_GAT_GATE_CLKCMU_FSYS0A_USB31DRD, 21, CLK_SET_RATE_PARENT,
	     0),
};

static const struct samsung_div_clock top_div_clks[] = {
	DIV(CLK_DOUT_CMU_PERIC0_BUS, "dout_cmu_peric0_bus",
	    "gout_cmu_peric0_bus", CLK_CON_DIV_CLKCMU_PERIC0_BUS, 0, 4),
	DIV(CLK_DOUT_CMU_PERIC0_IP, "dout_cmu_peric0_ip", "gout_cmu_peric0_ip",
	    CLK_CON_DIV_CLKCMU_PERIC0_IP, 0, 4),
	DIV(CLK_DOUT_CMU_FSYS1_UFS_EMBD, "dout_cmu_fsys1_ufs_embd",
	    "gout_cmu_fsys1_ufs_embd", CLK_CON_DIV_CLKCMU_FSYS1_UFS_EMBD, 0, 3),
	DIV(CLK_DOUT_CMU_FSYS0A_USB31DRD, "dout_cmu_fsys0a_usb31drd",
	    "gout_cmu_fsys0a_usb31drd", CLK_CON_DIV_CLKCMU_FSYS0A_USB31DRD, 0,
	    4),
};

static const struct samsung_clk_group top_cmu_clks[] = {
	{ S_CLK_PLL, top_pll_clks, ARRAY_SIZE(top_pll_clks) },
	{ S_CLK_MUX, top_pll_mux_clks, ARRAY_SIZE(top_pll_mux_clks) },
	{ S_CLK_DIV, top_pll_div_clks, ARRAY_SIZE(top_pll_div_clks) },
	{ S_CLK_MUX, top_mux_clks, ARRAY_SIZE(top_mux_clks) },
	{ S_CLK_GATE, top_gate_clks, ARRAY_SIZE(top_gate_clks) },
	{ S_CLK_DIV, top_div_clks, ARRAY_SIZE(top_div_clks) },
};

static int exynos9820_cmu_top_probe(struct udevice *dev)
{
	return samsung_register_cmu(dev, CMU_TOP, top_cmu_clks,
				    exynos9820_cmu_top);
}

static const struct udevice_id exynos9820_cmu_top_ids[] = {
	{ .compatible = "samsung,exynos9820-cmu-top" },
	{ }
};

SAMSUNG_CLK_OPS(exynos9820_cmu_top, CMU_TOP);

U_BOOT_DRIVER(exynos9820_cmu_top) = {
	.name		= "exynos9820-cmu-top",
	.id		= UCLASS_CLK,
	.of_match	= exynos9820_cmu_top_ids,
	.ops		= &exynos9820_cmu_top_clk_ops,
	.probe		= exynos9820_cmu_top_probe,
	.flags		= DM_FLAG_PRE_RELOC,
};
