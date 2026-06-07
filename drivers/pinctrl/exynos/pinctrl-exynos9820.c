// SPDX-License-Identifier: GPL-2.0+
/*
 * EXYNOS9820 pinctrl driver.
 *
 * based on drivers/pinctrl/exynos/pinctrl-exynos78x0.c :
 * Copyright (C) 2016 Samsung Electronics
 * Thomas Abraham <thomas.ab@samsung.com>
 * chiffathefox <chiffathefoxx@gmail.com>
 */

#include <dm.h>
#include <dm/pinctrl.h>
#include <asm/arch/pinmux.h>
#include "pinctrl-exynos.h"

static const struct pinctrl_ops exynos9820_pinctrl_ops = {
	.set_state	= exynos_pinctrl_set_state
};

/* pin banks of exynos9820 pin-controller 0 (ALIVE) */
static const struct samsung_pin_bank_data exynos9820_pin_banks0[] = {
	EXYNOS_PIN_BANK(8, 0x000, "gpa0"),
	EXYNOS_PIN_BANK(8, 0x020, "gpa1"),
	EXYNOS_PIN_BANK(8, 0x040, "gpa2"),
	EXYNOS_PIN_BANK(8, 0x060, "gpa3"),
	EXYNOS_PIN_BANK(2, 0x080, "gpa4"),
	EXYNOS_PIN_BANK(7, 0x0A0, "gpq0"),
	EXYNOS_PIN_BANK(6, 0x0C0, "etc0"),
};

/* pin banks of exynos9820 pin-controller 1 (AUD) */
static const struct samsung_pin_bank_data exynos9820_pin_banks1[] = {
	EXYNOS_PIN_BANK(5, 0x000, "gpb0"),
	EXYNOS_PIN_BANK(8, 0x020, "gpb1"),
	EXYNOS_PIN_BANK(4, 0x040, "gpb2"),
};

/* pin banks of exynos9820 pin-controller 2 (CMGP) */
static const struct samsung_pin_bank_data exynos9820_pin_banks2[] = {
	EXYNOS_PIN_BANK(1, 0x000, "gpm0"),
	EXYNOS_PIN_BANK(1, 0x020, "gpm1"),
	EXYNOS_PIN_BANK(1, 0x040, "gpm2"),
	EXYNOS_PIN_BANK(1, 0x060, "gpm3"),
	EXYNOS_PIN_BANK(1, 0x080, "gpm4"),
	EXYNOS_PIN_BANK(1, 0x0A0, "gpm5"),
	EXYNOS_PIN_BANK(1, 0x0C0, "gpm6"),
	EXYNOS_PIN_BANK(1, 0x0E0, "gpm7"),
	EXYNOS_PIN_BANK(1, 0x100, "gpm8"),
	EXYNOS_PIN_BANK(1, 0x120, "gpm9"),
	EXYNOS_PIN_BANK(1, 0x140, "gpm10"),
	EXYNOS_PIN_BANK(1, 0x160, "gpm11"),
	EXYNOS_PIN_BANK(1, 0x180, "gpm12"),
	EXYNOS_PIN_BANK(1, 0x1A0, "gpm13"),
	EXYNOS_PIN_BANK(1, 0x1C0, "gpm14"),
	EXYNOS_PIN_BANK(1, 0x1E0, "gpm15"),
	EXYNOS_PIN_BANK(1, 0x200, "gpm16"),
	EXYNOS_PIN_BANK(1, 0x220, "gpm17"),
	EXYNOS_PIN_BANK(1, 0x240, "gpm18"),
	EXYNOS_PIN_BANK(1, 0x260, "gpm19"),
	EXYNOS_PIN_BANK(1, 0x280, "gpm20"),
	EXYNOS_PIN_BANK(1, 0x2A0, "gpm21"),
	EXYNOS_PIN_BANK(1, 0x2C0, "gpm22"),
	EXYNOS_PIN_BANK(1, 0x2E0, "gpm23"),
	EXYNOS_PIN_BANK(1, 0x300, "gpm24"),
	EXYNOS_PIN_BANK(1, 0x320, "gpm25"),
	EXYNOS_PIN_BANK(1, 0x340, "gpm26"),
	EXYNOS_PIN_BANK(1, 0x360, "gpm27"),
	EXYNOS_PIN_BANK(1, 0x380, "gpm28"),
	EXYNOS_PIN_BANK(1, 0x3A0, "gpm29"),
	EXYNOS_PIN_BANK(1, 0x3C0, "gpm30"),
	EXYNOS_PIN_BANK(1, 0x3E0, "gpm31"),
};

/* pin banks of exynos9820 pin-controller 3 (FSYS0) */
static const struct samsung_pin_bank_data exynos9820_pin_banks3[] = {
	EXYNOS_PIN_BANK(6, 0x000, "gpf0"),
};

/* pin banks of exynos9820 pin-controller 4 (FSYS1) */
static const struct samsung_pin_bank_data exynos9820_pin_banks4[] = {
	EXYNOS_PIN_BANK(6, 0x000, "gpf1"),
	EXYNOS_PIN_BANK(3, 0x020, "gpf2"),
	EXYNOS_PIN_BANK(3, 0x040, "gpf3"),
};

/* pin banks of exynos9820 pin-controller 5 (PERIC0) */
static const struct samsung_pin_bank_data exynos9820_pin_banks5[] = {
	EXYNOS_PIN_BANK(8, 0x000, "gpp0"),
	EXYNOS_PIN_BANK(8, 0x020, "gpp1"),
	EXYNOS_PIN_BANK(8, 0x040, "gpp2"),
	EXYNOS_PIN_BANK(5, 0x060, "gpp3"),
	EXYNOS_PIN_BANK(8, 0x080, "gpg0"),
	EXYNOS_PIN_BANK(8, 0x0A0, "gpg1"),
	EXYNOS_PIN_BANK(7, 0x0C0, "gpg2"),
	EXYNOS_PIN_BANK(4, 0x0E0, "gpg4"),
};

/* pin banks of exynos9820 pin-controller 6 (PERIC1) */
static const struct samsung_pin_bank_data exynos9820_pin_banks6[] = {
	EXYNOS_PIN_BANK(8, 0x000, "gpp4"),
	EXYNOS_PIN_BANK(8, 0x020, "gpp5"),
	EXYNOS_PIN_BANK(4, 0x040, "gpp6"),
	EXYNOS_PIN_BANK(8, 0x060, "gpc0"),
	EXYNOS_PIN_BANK(8, 0x080, "gpc1"),
	EXYNOS_PIN_BANK(4, 0x0A0, "gpd0"),
	EXYNOS_PIN_BANK(7, 0x0C0, "gpg3"),
	EXYNOS_PIN_BANK(8, 0x0E0, "gph0"),
	EXYNOS_PIN_BANK(5, 0x100, "gph1"),
};

/* pin banks of exynos9820 pin-controller 7 (VTS) */
static const struct samsung_pin_bank_data exynos9820_pin_banks7[] = {
	EXYNOS_PIN_BANK(4, 0x000, "gpv0"),
};

const struct samsung_pin_ctrl exynos9820_pin_ctrl[] = {
	{
		/* pin-controller instance 0 Alive data */
		.pin_banks	= exynos9820_pin_banks0,
		.nr_banks	= ARRAY_SIZE(exynos9820_pin_banks0),
	}, {
		/* pin-controller instance 1 AUD data */
		.pin_banks	= exynos9820_pin_banks1,
		.nr_banks	= ARRAY_SIZE(exynos9820_pin_banks1),
	}, {
		/* pin-controller instance 2 CMGP data */
		.pin_banks	= exynos9820_pin_banks2,
		.nr_banks	= ARRAY_SIZE(exynos9820_pin_banks2),
	}, {
		/* pin-controller instance 3 FSYS0 data */
		.pin_banks	= exynos9820_pin_banks3,
		.nr_banks	= ARRAY_SIZE(exynos9820_pin_banks3),
	}, {
		/* pin-controller instance 4 FSYS1 data */
		.pin_banks	= exynos9820_pin_banks4,
		.nr_banks	= ARRAY_SIZE(exynos9820_pin_banks4),
	}, {
		/* pin-controller instance 5 PERIC0 data */
		.pin_banks	= exynos9820_pin_banks5,
		.nr_banks	= ARRAY_SIZE(exynos9820_pin_banks5),
	}, {
		/* pin-controller instance 6 PERIC1 data */
		.pin_banks	= exynos9820_pin_banks6,
		.nr_banks	= ARRAY_SIZE(exynos9820_pin_banks6),
	}, {
		/* pin-controller instance 7 VTS data */
		.pin_banks	= exynos9820_pin_banks7,
		.nr_banks	= ARRAY_SIZE(exynos9820_pin_banks7),
	},
	{/* list terminator */}
};

static const struct udevice_id exynos9820_pinctrl_ids[] = {
	{ .compatible = "samsung,exynos9820-pinctrl",
		.data = (ulong)exynos9820_pin_ctrl },
	{ }
};

U_BOOT_DRIVER(pinctrl_exynos9820) = {
	.name		= "pinctrl_exynos9820",
	.id		= UCLASS_PINCTRL,
	.of_match	= exynos9820_pinctrl_ids,
	.priv_auto 	= sizeof(struct exynos_pinctrl_priv),
	.ops		= &exynos9820_pinctrl_ops,
	.probe		= exynos_pinctrl_probe,
	.bind		= exynos_pinctrl_bind,
};
