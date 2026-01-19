#pragma once

#include <generic-phy.h>
#include "ufs.h"

struct exynos_ufs_sfr_log {
	const char* name;
	const u32 offset;
#define LOG_STD_HCI_SFR		0xFFFFFFF0
#define LOG_VS_HCI_SFR		0xFFFFFFF1
#define LOG_FMP_SFR		0xFFFFFFF2
#define LOG_UNIPRO_SFR		0xFFFFFFF3
#define LOG_PMA_SFR		0xFFFFFFF4
	u32 val;
};

struct exynos_ufs_debug {
	struct exynos_ufs_sfr_log* std_sfr;
        struct ufs_hba *hba;
};


void exynos_ufs_debug_init(struct exynos_ufs_debug *debug, struct ufs_hba *hba);
void exynos_ufs_show_uic_info(struct ufs_hba *hba);