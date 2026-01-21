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
	CMU_CMU,
	CMU_FSYS1,
	CMU_PERIC0,
};

/*
 * Register offsets for CMU_CMU (0x1a240000)
 */
#define CLK_CON_DIV_CMU_SHARED0_PLL_DIV4				0x18d4
#define CLK_CON_DIV_CMU_SHARED0_PLL_DIV2				0x18cc
#define CLK_CON_MUX_CMU_SHARED0_PLL					0x10dc
#define CLK_CON_DIV_CMU_SHARED2_PLL_DIV2				0x18e4
#define PLL_CON_CMU_SHARED0_PLL						0x160
#define PLL_CON_CMU_SHARED0_APLL					0x100
#define PLL_CON_CMU_SHARED2_PLL						0x1a0
#define CLK_CON_MUX_CMU_PERIC0_PERI					0x10bc
#define CLK_CON_GAT_CMU_PERIC0_PERI					0x20c0
#define CLK_CON_DIV_CMU_PERIC0_PERI					0x18a8
#define CLK_CON_MUX_CMU_PERIC0_UART0					0x120
#define CLK_CON_DIV_CMU_SHARED4_PLL_DIV2				0x18ec
#define CLK_CON_DIV_CMU_FSYS1_UFS					0x1864
#define CLK_CON_GAT_CMU_FSYS1_UFS					0x2080
#define CLK_CON_MUX_CMU_FSYS1_UFS					0x1078
#define PLL_CON_CMU_SHARED4_PLL						0x1e0

static const struct samsung_pll_clock cmu_pll_clks[] = {
	PLL(pll_0817x, CLK_FOUT_CMU_SHARED0_PLL,
	    "fout_cmu_shared0_pll", "oscclk",
	    PLL_CON_CMU_SHARED0_PLL),
	PLL(pll_0822x, CLK_FOUT_CMU_SHARED0_APLL,
	    "fout_cmu_shared0_apll", "oscclk",
	    PLL_CON_CMU_SHARED0_APLL),
	PLL(pll_0818x, CLK_FOUT_CMU_SHARED2_PLL,
	    "fout_cmu_shared2_pll", "oscclk",
	    PLL_CON_CMU_SHARED2_PLL),
	PLL(pll_1018x, CLK_FOUT_CMU_SHARED4_PLL,
	    "fout_cmu_shared4_pll", "oscclk",
	    PLL_CON_CMU_SHARED4_PLL),
};

/* List of parent clocks for muxes in CMU_CMU */
PNAME(mout_cmu_shared0_pll_p)			= { "fout_cmu_shared0_pll",
						    "fout_cmu_shared0_apll" };
PNAME(mout_cmu_peric0_peri_p)			= { "dout_cmu_shared0_pll_div4",
						    "dout_cmu_shared2_pll_div2" };
PNAME(mout_cmu_peric0_uart0_p)			= { "oscclk",
						    "dout_cmu_peric0_peri" };
PNAME(mout_cmu_fsys1_ufs_p)			= { "oscclk",
						    "dout_cmu_shared0_pll_div4",
						    "dout_cmu_shared2_pll_div2",
						    "dout_cmu_shared4_pll_div2" };

static const struct samsung_mux_clock cmu_pll_mux_clks[] = {
	MUX(CLK_MOUT_CMU_SHARED0_PLL,
	    "mout_cmu_shared0_pll", mout_cmu_shared0_pll_p,
	    CLK_CON_MUX_CMU_SHARED0_PLL, 0, 1),
};

static const struct samsung_div_clock cmu_pll_div_clks[] = {
	DIV(CLK_DOUT_CMU_SHARED0_PLL_DIV2,
	    "dout_cmu_shared0_pll_div2", "mout_cmu_shared0_pll",
	    CLK_CON_DIV_CMU_SHARED0_PLL_DIV2, 0, 1),
	DIV(CLK_DOUT_CMU_SHARED0_PLL_DIV4,
	    "dout_cmu_shared0_pll_div4", "dout_cmu_shared0_pll_div2",
	    CLK_CON_DIV_CMU_SHARED0_PLL_DIV4, 0, 1),
	DIV(CLK_DOUT_CMU_SHARED2_PLL_DIV2,
	    "dout_cmu_shared2_pll_div2", "fout_cmu_shared2_pll",
	    CLK_CON_DIV_CMU_SHARED2_PLL_DIV2, 0, 1),
	DIV(CLK_DOUT_CMU_SHARED4_PLL_DIV2,
	    "dout_cmu_shared4_pll_div2", "fout_cmu_shared4_pll",
	    CLK_CON_DIV_CMU_SHARED4_PLL_DIV2, 0, 1),
};

static const struct samsung_mux_clock cmu_mux_clks[] = {
	MUX(CLK_MOUT_CMU_PERIC0_PERI,
	    "mout_cmu_peric0_peri", mout_cmu_peric0_peri_p,
	    CLK_CON_MUX_CMU_PERIC0_PERI, 0, 1),
	MUX(CLK_MOUT_CMU_FSYS1_UFS,
	    "mout_cmu_fsys1_ufs", mout_cmu_fsys1_ufs_p,
	    CLK_CON_MUX_CMU_FSYS1_UFS, 0, 2),
};

static const struct samsung_gate_clock cmu_gate_clks[] = {
	GATE(CLK_GOUT_CMU_PERIC0_PERI,
	     "gout_cmu_peric0_peri", "mout_cmu_peric0_peri",
	     CLK_CON_GAT_CMU_PERIC0_PERI, 20,
	     CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_CMU_FSYS1_UFS,
	     "gout_cmu_fsys1_ufs", "mout_cmu_fsys1_ufs",
	     CLK_CON_GAT_CMU_FSYS1_UFS, 21,
	     CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_div_clock cmu_peric0_peri_div_clks[] = {
	DIV(CLK_DOUT_CMU_PERIC0_PERI,
	    "dout_cmu_peric0_peri", "gout_cmu_peric0_peri",
	    CLK_CON_DIV_CMU_PERIC0_PERI, 0, 4),
	DIV(CLK_DOUT_CMU_FSYS1_UFS,
	    "dout_cmu_fsys1_ufs", "gout_cmu_fsys1_ufs",
	    CLK_CON_DIV_CMU_FSYS1_UFS, 0, 3),
};

static const struct samsung_mux_clock cmu_peric0_mux_clks[] = {
	MUX(CLK_MOUT_CMU_PERIC0_UART0,
	    "mout_cmu_peric0_uart0", mout_cmu_peric0_uart0_p,
	    CLK_CON_MUX_CMU_PERIC0_UART0, 4, 1),
};

static const struct samsung_clk_group cmu_cmu_clks[] = {
	{ S_CLK_PLL, cmu_pll_clks, ARRAY_SIZE(cmu_pll_clks) },
	{ S_CLK_MUX, cmu_pll_mux_clks, ARRAY_SIZE(cmu_pll_mux_clks) },
	{ S_CLK_DIV, cmu_pll_div_clks, ARRAY_SIZE(cmu_pll_div_clks) },
	{ S_CLK_MUX, cmu_mux_clks, ARRAY_SIZE(cmu_mux_clks) },
	{ S_CLK_GATE, cmu_gate_clks, ARRAY_SIZE(cmu_gate_clks) },
	{ S_CLK_DIV, cmu_peric0_peri_div_clks, ARRAY_SIZE(cmu_peric0_peri_div_clks) },
	{ S_CLK_MUX, cmu_peric0_mux_clks, ARRAY_SIZE(cmu_peric0_mux_clks) },
};

static int exynos9820_cmu_cmu_probe(struct udevice *dev)
{
	return samsung_register_cmu(dev, CMU_CMU, cmu_cmu_clks,
				    exynos9820_cmu_cmu);
}

static const struct udevice_id exynos9820_cmu_cmu_ids[] = {
	{ .compatible = "samsung,exynos9820-cmu-cmu" },
	{ }
};

SAMSUNG_CLK_OPS(exynos9820_cmu_cmu, CMU_CMU);

U_BOOT_DRIVER(exynos9820_cmu_cmu) = {
	.name		= "exynos9820-cmu-cmu",
	.id		= UCLASS_CLK,
	.of_match	= exynos9820_cmu_cmu_ids,
	.ops		= &exynos9820_cmu_cmu_clk_ops,
	.probe		= exynos9820_cmu_cmu_probe,
	.flags		= DM_FLAG_PRE_RELOC,
};

/*
 * Register offsets for CMU_FSYS1 (0x13c00000)
 */
#define CLK_CON_GAT_FSYS1_UFS_ACLK					0x3070
#define CLK_CON_MUX_FSYS1_UFS_ACLK					0x180

/* List of parent clocks for muxes in CMU_FSYS1 */
PNAME(mout_fsys1_ufs_aclk_p)			= { "oscclk",
						    "dout_cmu_fsys1_ufs" };

static const struct samsung_mux_clock fsys1_mux_clks[] = {
	MUX(CLK_MOUT_FSYS1_UFS_ACLK,
	    "mout_fsys1_ufs_aclk", mout_fsys1_ufs_aclk_p,
	    CLK_CON_MUX_FSYS1_UFS_ACLK, 4, 1),
};

static const struct samsung_gate_clock fsys1_gate_clks[] = {
	GATE(CLK_GOUT_FSYS1_UFS_ACLK,
	     "gout_fsys1_ufs_aclk", "mout_fsys1_ufs_aclk",
	     CLK_CON_GAT_FSYS1_UFS_ACLK, 0,
	     CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_clk_group fsys1_cmu_clks[] = {
	{ S_CLK_MUX, fsys1_mux_clks, ARRAY_SIZE(fsys1_mux_clks) },
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
 * Register offsets for CMU_PERIC0 (0x10400000)
 */
#define CLK_CON_GAT_PERIC0_UART0_PCLK					0x301c
#define CLK_CON_GAT_PERIC0_BUS						0x301c
#define CLK_CON_MUX_PERIC0_BUS_USER					0x100
#define CLK_CON_MUX_PERIC0_BUS						0x10b8
#define CLK_CON_DIV_PERIC0_BUS						0x18a4
#define CLK_CON_DIV_PERIC0_UART0_SCLK					0x1800

static const struct samsung_div_clock peric0_div_clks[] = {
	DIV(CLK_DOUT_PERIC0_UART0_SCLK,
	    "dout_peric0_uart0_sclk", "cmu_peric0_uart0",
	    CLK_CON_DIV_PERIC0_UART0_SCLK, 0, 4),
};

static const struct samsung_gate_clock peric0_gate_clks[] = {
	GATE(CLK_GOUT_PERIC0_UART0_PCLK,
	     "gout_peric0_uart0_pclk", "oscclk",
	     CLK_CON_GAT_PERIC0_UART0_PCLK, 1,
	     CLK_SET_RATE_PARENT, 0),
};

static const struct samsung_clk_group peric0_cmu_clks[] = {
	{ S_CLK_DIV, peric0_div_clks, ARRAY_SIZE(peric0_div_clks) },
	{ S_CLK_GATE, peric0_gate_clks, ARRAY_SIZE(peric0_gate_clks) },
};

static int exynos9820_cmu_peric0_probe(struct udevice *dev)
{
	log_debug("%s(%p)\n", __func__, dev);
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
