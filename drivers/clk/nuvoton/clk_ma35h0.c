// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Nuvoton technology corporation
 * All rights reserved.
 */

#include <common.h>
#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <log.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <dt-bindings/clock/ma35h0-clk.h>
#include <misc.h>

#include "regs-ma35h0-clock.h"
#include "clk-ma35h0.h"

DECLARE_GLOBAL_DATA_PTR;

struct __ma35h0_clk_priv {
	void *base;
	const struct __ma35h0_clk_ops *ops;
	struct __ma35h0_clk_gate_data *gate_data;
	struct __ma35h0_pll_data *pll;
	struct __ma35h0_clk_data *data;
};

/* struct __ma35h0_clk_ops - clock operations */
struct __ma35h0_clk_ops {
	int (*set_rate)(struct __ma35h0_clk_priv *priv,unsigned long rate,
	                unsigned long parent_rate);
	unsigned long (*round_rate)(struct __ma35h0_clk_priv *priv,unsigned long rate,
	                            unsigned long *parent_rate);
	unsigned long (*recalc_rate)(struct __ma35h0_clk_priv *priv,
	                             unsigned long parent_rate);
	int (*enable_clk)(struct __ma35h0_clk_priv *priv, bool enable);
};

static const char * const clk_names[clk_max] = {
/*0 */ "capll", "ddrpll", "apll", "epll", "vpll",
/*5 */ "hxt_gate", "lxt_gate", "hirc_gate", "lirc_gate", "ddr0_gate", "ddr6_gate", "sdh0_gate", "sdh1_gate",
/*13 */ "nand_gate", "usbh_gate", "husbh0_gate", "husbh1_gate", "dcu_gate", "emac0_gate", "emac1_gate", "rtc_gate",
/*21 */ "ddr_gate", "i2c0_gate", "i2c1_gate", "i2c2_gate", "i2c3_gate", "i2c4_gate", "i2c5_gate", "qspi0_gate",
/*29 */ "qspi1_gate", "spi0_gate", "spi1_gate", "spi2_gate", "spi3_gate", "gpa_gate", "gpb_gate", "gpc_gate",
/*37 */ "gpd_gate", "gpe_gate", "gpf_gate", "gpg_gate", "gph_gate", "gpi_gate", "gpj_gate", "gpk_gate",
/*45 */ "gpl_gate", "gpm_gate", "gpn_gate", "ca35clk_mux", "sysclk0_mux", "sysclk1_mux", "sdh0_mux", "sdh1_mux",
/*53 */ "dcu_mux", "spi0_mux", "spi1_mux", "spi2_mux", "spi3_mux", "qspi0_mux", "qspi1_mux",
/*60 */ "dcup_div", "emac0_div", "emac1_div", "aclk0_div", "wdt1_gate"
};

static const struct vsipll_freq_conf_reg_tbl ma35h0pll_freq[] = {
	{ 1000000000, VSIPLL_INTEGER_MODE, 0x307d, 0x10, 0 },
	{ 884736000, VSIPLL_FRACTIONAL_MODE, 0x41024, 0xdd2f1b11, 0 },
	{ 533000000, VSIPLL_SS_MODE, 0x12b8102c, 0x6aaaab20, 0x12317 },
	{ }
};

ulong hxt_hz = 0;
ulong lxt_hz = 0;
ulong hirc_hz = 0;
ulong lirc_hz = 0;
ulong usbphy0_hz = 0, usbphy1_hz = 0;
ulong pllfreq[5] = {0};
u32 pllmode[5] = {0};
ulong caclk = 0;
ulong sysclk[2] = {0};


/* SMIC PLL for CAPLL */
unsigned long CLK_GetPLLFreq_SMICPLL(struct __ma35h0_clk_priv *priv,
                             unsigned long PllSrcClk)
{
	struct __ma35h0_clk_data *data = priv->data;
	struct __ma35h0_pll_data *pll = priv->pll;
	u32 u32M, u32N, u32P, u32OutDiv;
	u32 val;
	unsigned long u64PllClk;
	u32 clk_div_table[] = { 1, 2, 4, 8};

	val = readl(data->base + pll->ctl0);

	u32N = (val & SMICPLLCTL0_FBDIV_MSK);
	u32M = (val & SMICPLLCTL0_INDIV_MSK) >> SMICPLLCTL0_INDIV_POS;
	u32P = (val & SMICPLLCTL0_OUTDIV_MSK) >> SMICPLLCTL0_OUTDIV_POS;

	u32OutDiv = clk_div_table[u32P];

	if (val & SMICPLLCTL0_BP_MSK)
		u64PllClk = PllSrcClk;
	else
		u64PllClk = (PllSrcClk * u32N) / (u32OutDiv * (u32M));

	pr_debug("CAPLL: [ %ld ] Hz,    M, N, P: %d, %d, %d(%d)\n", u64PllClk,
	       u32M, u32N, u32P, u32OutDiv);

	return u64PllClk;
}

/* VSI-PLL: INTEGER_MODE */
unsigned long CLK_CalPLLFreq_Mode0(unsigned long PllSrcClk,
                                   unsigned long u64PllFreq, u32 *u32Reg)
{
	u32 u32TmpM, u32TmpN, u32TmpP;
	u32 u32RngMinN, u32RngMinM, u32RngMinP;
	u32 u32RngMaxN, u32RngMaxM, u32RngMaxP;
	u32 u32Tmp, u32Min, u32MinN, u32MinM, u32MinP;
	unsigned long u64PllClk;
	unsigned long u64Con1, u64Con2, u64Con3;

	u64PllClk = 0;
	u32Min = (u32) -1;
	u32MinM = 0UL;
	u32MinN = 0UL;
	u32MinP = 0UL;

	if (!((u64PllFreq >= VSIPLL_FCLKO_MIN_FREQ)
	      && (u64PllFreq <= VSIPLL_FCLKO_MAX_FREQ))) {
		u32Reg[0] = ma35h0pll_freq[0].ctl0_reg;
		u32Reg[1] = ma35h0pll_freq[0].ctl1_reg;
		u64PllClk = ma35h0pll_freq[0].freq;

		return u64PllClk;
	}

	/* Find best solution */
	u32RngMinM = 1UL;
	u32RngMaxM = 63UL;

	u32RngMinM = ((PllSrcClk / VSIPLL_FREFDIVM_MAX_FREQ) > 1) ?
	             (PllSrcClk / VSIPLL_FREFDIVM_MAX_FREQ) : 1;
	u32RngMaxM = ((PllSrcClk / VSIPLL_FREFDIVM_MIN_FREQ0) < u32RngMaxM) ?
	             (PllSrcClk / VSIPLL_FREFDIVM_MIN_FREQ0) : u32RngMaxM;

	for (u32TmpM = u32RngMinM; u32TmpM < (u32RngMaxM + 1); u32TmpM++) {
		u64Con1 = PllSrcClk / u32TmpM;

		u32RngMinN = 16UL;
		u32RngMaxN = 2047UL;

		u32RngMinN = ((VSIPLL_FCLK_MIN_FREQ / u64Con1) > u32RngMinN) ?
		             (VSIPLL_FCLK_MIN_FREQ / u64Con1) : u32RngMinN;
		u32RngMaxN = ((VSIPLL_FCLK_MAX_FREQ / u64Con1) < u32RngMaxN) ?
		             (VSIPLL_FCLK_MAX_FREQ / u64Con1) : u32RngMaxN;

		for (u32TmpN = u32RngMinN; u32TmpN < (u32RngMaxN + 1); u32TmpN++) {
			u64Con2 = u64Con1 * u32TmpN;

			u32RngMinP = 1UL;
			u32RngMaxP = 7UL;

			u32RngMinP = ((u64Con2 / VSIPLL_FCLKO_MAX_FREQ) > 1) ? (u64Con2 /
			             VSIPLL_FCLKO_MAX_FREQ) : 1;
			u32RngMaxP = ((u64Con2 / VSIPLL_FCLKO_MIN_FREQ) < u32RngMaxP) ?
			             (u64Con2 / VSIPLL_FCLKO_MIN_FREQ) : u32RngMaxP;

			for (u32TmpP = u32RngMinP; u32TmpP < (u32RngMaxP + 1); u32TmpP++) {
				u64Con3 = u64Con2 / u32TmpP;
				if (u64Con3 > u64PllFreq)
					u32Tmp = u64Con3 - u64PllFreq;
				else
					u32Tmp = u64PllFreq - u64Con3;

				if (u32Tmp < u32Min) {
					u32Min = u32Tmp;
					u32MinM = u32TmpM;
					u32MinN = u32TmpN;
					u32MinP = u32TmpP;

					/* Break when get good results */
					if (u32Min == 0UL) {
						u32Reg[0] = (u32MinM << 12) | (u32MinN);
						u32Reg[1] = (u32MinP << 4);

						return ((PllSrcClk * u32MinN) / (u32MinP * u32MinM));
					}
				}
			}
		}
	}

	/* Enable and apply new PLL setting. */
	u32Reg[0] = (u32MinM << 12) | (u32MinN);
	u32Reg[1] = (u32MinP << 4);

	/* Actual PLL output clock frequency */
	u64PllClk = (PllSrcClk * u32MinN) / (u32MinP * u32MinM);

	return u64PllClk;
}

/* VSI-PLL: FRACTIONAL_MODE */
unsigned long CLK_CalPLLFreq_Mode1(unsigned long PllSrcClk,
                                   unsigned long u64PllFreq, u32 *u32Reg)
{
	unsigned long u64X, u64N, u64M, u64P, u64tmp, u64tmpP, u64tmpM;
	unsigned long u64PllClk, u64FCLKO;
	u32 u32FRAC, i;

	// check condition 1
	if ((PllSrcClk > 200000000) || (PllSrcClk < 1000000)) {
		// Fref is incorrect, return fail
		return 0;
	}

	// check condition 4
	if(u64PllFreq < 85700000) {
		// Adjust u64FCLKO
		u64FCLKO = 0;

		for(i = 2; i < 100; i++) {
			u64tmp = (i * u64PllFreq);
			if (u64tmp > 85700000) {
				u64FCLKO = u64tmp;
				break;
			}
		}

		if (u64FCLKO == 0) return 0;
	} else if(u64PllFreq >= 2400000000) {
		u32Reg[0] = 0x30FA;
		u32Reg[1] = (0x2 << 4);
		u64PllClk = 1000000000;
		return u64PllClk;
	} else
		u64FCLKO = u64PllFreq;

	// Find P
	u64P = 0;
	for(i = 1; i < 8; i++) {
		u64tmpP =  i * u64FCLKO;
		// it should be condition 3
		if((u64tmpP <= 2400000000) && (u64tmpP >= 600000000)) {
			u64P = i;
			break;
		}
	}

	// No reasonable P is found, return fail.
	if (u64P == 0)	return 0;

	// Find M
	u64M = 0; // Initialize it, and use it to judge reasonable M is found or not
	for(i = 1; i < 64; i++) {
		u64tmpM = PllSrcClk / i;
		if((u64tmpM <= 40000000) && (u64tmpM >= 10000000)) { // condition 2
			u64M = i;
			break;
		}
	}

	if (u64M == 0) { // No reasonable M is found
		return 0;
	}

	u64tmp = (u64FCLKO * u64P * u64M * 1000)/ PllSrcClk;
	u64N = u64tmp / 1000;
	u64X = u64tmp % 1000;
	u32FRAC = ((u64X << 24) + 500) / 1000;

	u32Reg[0] = (u64M << 12) | (u64N);
	u32Reg[1] = (u64P << 4) | (u32FRAC << 8);

	/* Actual PLL output clock frequency */
	u64PllClk = (PllSrcClk * u64tmp) / u64P / u64M / 1000;

	return u64PllClk;
}

/* VSI-PLL: SS_MODE */
unsigned long CLK_CalPLLFreq_Mode2(unsigned long PllSrcClk,
                                   unsigned long u64PllFreq, u32 u32SR, u32 u32Fmod, u32 *u32Reg)
{

	unsigned long u64PllClk;
	unsigned long u64X, u64N, u64M, u64P, u64tmp, u64tmpP, u64tmpM;
	unsigned long u64FCLKO, u64SSRATE, u64SLOPE;
	u32 u32FRAC, i;

	// check condition 1
	if ((PllSrcClk > 200000000) || (PllSrcClk < 1000000)) {
		// Fref is incorrect, return fail case
		return 0;
	}

	// check condition 4
	if(u64PllFreq < 85700000) {
		u64FCLKO = 0;
		for(i = 2; i < 8; i++) {
			u64tmp = (i * u64PllFreq);
			if (u64tmp > 85700000) {
				u64FCLKO = u64tmp;
			}
		}

		if (u64FCLKO == 0) return 0;
	} else if(u64PllFreq >= 2400000000) {
		u32Reg[0] = 0x30FA;
		u32Reg[1] = (0x2 << 4);
		u64PllClk = 1000000000;
		return u64PllClk;
	} else
		u64FCLKO = u64PllFreq;

	// Find P
	u64P = 0;
	for(i = 1; i < 8; i++) {
		u64tmpP =  i * u64FCLKO;
		if((u64tmpP <= 2400000000) && (u64tmpP >= 600000000)) {
			u64P = i;
			break;
		}
	}

	// No reasonable P is found, return fail.
	if (u64P == 0)	return 0;

	// Find M
	u64M = 0; // Initialize it, and use it to judge reasonable M is found or not
	for(i = 1; i < 64; i++) {
		u64tmpM = PllSrcClk / i;
		if((u64tmpM <= 40000000) && (u64tmpM >= 10000000)) { // condition 2
			u64M = i;
			break;
		}
	}

	if (u64M == 0) { // No reasonable M is found
		return 0;
	}

	u64tmp = (u64FCLKO * u64P * u64M * 1000)/ PllSrcClk;
	u64N = u64tmp / 1000;
	u64X = u64tmp % 1000;
	u32FRAC = ((u64X << 24) + 500) / 1000;

	u64SSRATE = ((PllSrcClk >> 1) / (u32Fmod*2)) -1;
	u64SLOPE = ((u64tmp * u32SR / u64SSRATE) << 24) / 100 / 1000;

	u32Reg[0] = (u64SSRATE << 20) | (u64M << 12) | (u64N);
	u32Reg[1] = (u64P << 4) | (u32FRAC << 8);
	u32Reg[2] = u64SLOPE;

	/* Actual PLL output clock frequency */
	u64PllClk = (PllSrcClk * u64tmp) / u64P / u64M / 1000;

	return u64PllClk;
}

unsigned long CLK_SetPLLFreq(struct __ma35h0_clk_priv *priv,
                             unsigned long PllSrcClk, unsigned long u64PllFreq)
{
	struct __ma35h0_clk_data *data = priv->data;
	struct __ma35h0_pll_data *pll = priv->pll;
	u32 u32Reg[3] = {0}, val_ctl0, val_ctl1, val_ctl2;
	unsigned long u64PllClk;

	pr_debug("%s PLL%d Mode %d\n", __func__, pll->type, pll->mode);
	val_ctl0 = readl(data->base + pll->ctl0);
	val_ctl1 = readl(data->base + pll->ctl1);
	val_ctl2 = readl(data->base + pll->ctl2);
	//printf("      CTL0~2: 0x%08x, 0x%08x, 0x%08x\n", val_ctl0, val_ctl1, val_ctl2);

	// PLL Operation mode setting
	val_ctl0 = (val_ctl0 & ~(0xc0000)) | (pll->mode << 18);

	if(pll->mode == 0) {
		u64PllClk = CLK_CalPLLFreq_Mode0(PllSrcClk, u64PllFreq, u32Reg);
		val_ctl0 = (val_ctl0 & ~(0x3f000) & ~(0x7ff)) | u32Reg[0];
		val_ctl1 = (val_ctl1 & ~(0x7 << 4)) | u32Reg[1];
	} else if(pll->mode == 1) {
		u64PllClk = CLK_CalPLLFreq_Mode1(PllSrcClk, u64PllFreq, u32Reg);
		val_ctl0 = (val_ctl0 & ~(0x3f<< 12) & ~(0x7ff)) | u32Reg[0];
		val_ctl1 = (val_ctl1 & ~(0x7 << 4) & ~(0xffffff << 8)) | u32Reg[1];
	} else { //pll->mode == 2
		u64PllClk = CLK_CalPLLFreq_Mode2(PllSrcClk, u64PllFreq, 50000, 194,
		                                 u32Reg); //50khz, 1.94%
		val_ctl0 = (val_ctl0 & ~(0x7ff << 20) & ~(0x3f << 12) & ~(0x7ff)) | u32Reg[0];
		val_ctl1 = (val_ctl1 & ~(0x7 << 4) & ~(0xffffff << 8)) | u32Reg[1];
		val_ctl2 = u32Reg[2];
	}

	__raw_writel(val_ctl0, data->base + pll->ctl0);
	__raw_writel(val_ctl1, data->base + pll->ctl1);
	__raw_writel(val_ctl2, data->base + pll->ctl2);

	//printf("    CTL0~2: 0x%08x, 0x%08x, 0x%08x\n", val_ctl0, val_ctl1, val_ctl2);
	return u64PllClk;
}

unsigned long CLK_GetPLLFreq(struct __ma35h0_clk_priv *priv,
                             unsigned long PllSrcClk)
{
	struct __ma35h0_clk_data *data = priv->data;
	struct __ma35h0_pll_data *pll = priv->pll;
	u32 u32M, u32N, u32P, u32X, u32SR, u32FMOD;
	u32 val_ctl0, val_ctl1, val_ctl2, val_mode;
	unsigned long u64PllClk, u64X;

	pr_debug("%s PLL%d\n", __func__, pll->type);
	val_ctl0 = readl(data->base + pll->ctl0);
	val_ctl1 = readl(data->base + pll->ctl1);
	val_ctl2 = readl(data->base + pll->ctl2);

	// PLL Operation mode setting
	val_mode = (val_ctl0 & (0xc0000)) >> 18;
	pll->mode = val_mode;

	if(val_ctl0 == 0)
		u64PllClk = 0;
	else {

		if(pll->mode == 0) {
			u32N = (val_ctl0 & (0x7ff));
			u32M = (val_ctl0 & (0x3f000)) >> 12;

			u32P = ((val_ctl1 & (0x70)) >> 4);

			/* Actual PLL output clock frequency */
			u64PllClk = (PllSrcClk * u32N) / (u32P * (u32M));

		} else if(pll->mode == 1) {
			u32N = (val_ctl0 & (0x7ff));
			u32M = (val_ctl0 & (0x3f000)) >> 12;

			u32P = ((val_ctl1 & (0x70)) >> 4);
			u32X = (val_ctl1 & 0xffffff00) >> 8;

			/* Actual PLL output clock frequency */
			u64X = (u64)u32X;
			u64X = (((u64X * 1000)+500) >> 24);
			u64PllClk = (PllSrcClk * ((u32N*1000) + u64X)) / 1000 / u32P / u32M;

		} else { //pll->mode == 2
			u32N = (val_ctl0 & (0x7ff));
			u32M = (val_ctl0 & (0x3f000)) >> 12;
			u32SR = (val_ctl0 & (0x7ff00000)) >> 20;

			u32P = ((val_ctl1 & (0x70)) >> 4);
			u32X =	(val_ctl1 & 0xffffff00) >> 8;

			u32FMOD = (val_ctl2 & (0x00ffffff));

			/* Actual PLL output clock frequency */
			u64X = (u64)u32X;
			u64X = ((u64X * 1000) >> 24);

			u64PllClk = (PllSrcClk * ((u32N*1000) + u64X)) / 1000 / u32P / u32M;
			//u64PllClk = pll->rate;
		}
	}
	//printf("PLL[%d]: %ld Hz\n\n", pll->type, u64PllClk);
	return u64PllClk;
}

static unsigned long ma35h0_pll_recalc_rate(struct __ma35h0_clk_priv *priv,
        unsigned long parent_rate)
{
	struct __ma35h0_pll_data *pll = priv->pll;
	unsigned long freq;

	pr_debug("%s PLL%d\n", __func__, pll->type);

	if(pll->type == MA35H0_CAPLL)
		freq = CLK_GetPLLFreq_SMICPLL(priv, parent_rate);
	else
		freq = CLK_GetPLLFreq(priv, parent_rate);

	return freq;
}

static int ma35h0_pll_set_rate(struct __ma35h0_clk_priv *priv,
                                unsigned long rate,
                                unsigned long parent_rate)
{
	struct __ma35h0_pll_data *pll = priv->pll;

	if((pll->type != MA35H0_CAPLL) || (pll->type != MA35H0_DDRPLL))
		pll->rate = CLK_SetPLLFreq(priv, parent_rate, rate);

	return pll->rate;
}


static int ma35h0_pll_clock_enable(struct __ma35h0_clk_priv *priv, bool enable)
{
	struct __ma35h0_pll_data *pll = priv->pll;
	struct __ma35h0_clk_data *data = priv->data;
	u32 val;

	pr_debug("%s PLL%d\n", __func__, pll->type);

	if((pll->type != MA35H0_CAPLL) || (pll->type != MA35H0_DDRPLL))
	{
		if (enable) {
			val = readl(data->base+pll->ctl1);
			val &= ~0x1;/* PD = 0, power down mode disable */
			writel(val, data->base+pll->ctl1);
			pr_debug("%s Enable:[0x%x]\n", __func__, readl(data->base+pll->ctl1));
		} else {
			val = readl(data->base+pll->ctl1);
			val |= 0x1;/* PD = 1, power down mode enable */
			writel(val, data->base+pll->ctl1);
			pr_debug("%s Disnable:[0x%x]\n", __func__, readl(data->base+pll->ctl1));
		}
	}
	return 0;
}


static const struct __ma35h0_clk_ops ma35h0_pll_clk_ops = {
	.set_rate = ma35h0_pll_set_rate,
	.recalc_rate = ma35h0_pll_recalc_rate,
	.enable_clk = ma35h0_pll_clock_enable,
};

static struct __ma35h0_clk_priv __ma35h0_init_plls[] = {
	[capll] = {
		.ops = &ma35h0_pll_clk_ops,
		.gate_data = &__clk_id_gate_sets[capll],
		.pll = &__ma35h0_capll_data,
	},
	[ddrpll] = {
		.ops = &ma35h0_pll_clk_ops,
		.gate_data = &__clk_id_gate_sets[ddrpll],
		.pll = &__ma35h0_ddrpll_data,
	},
	[apll] = {
		.ops = &ma35h0_pll_clk_ops,
		.gate_data = &__clk_id_gate_sets[apll],
		.pll = &__ma35h0_apll_data,
	},
	[epll] = {
		.ops = &ma35h0_pll_clk_ops,
		.gate_data = &__clk_id_gate_sets[epll],
		.pll = &__ma35h0_epll_data,
	},
	[vpll] = {
		.ops = &ma35h0_pll_clk_ops,
		.gate_data = &__clk_id_gate_sets[vpll],
		.pll = &__ma35h0_vpll_data,
	},
};

static ulong ma35h0_pll_parent_rate(struct __ma35h0_clk_priv *priv)
{
	ulong parent_rate;

	pr_debug("%s\n", __func__);

	parent_rate = clk_get_rate(&priv->data->parent_hxt);

	return parent_rate;
}


ulong Assign_root_clk(u32 clk_sel_type)
{
	ulong freq = 0;
	switch(clk_sel_type) {
	case TYPE_HXT_HZ:
		freq = hxt_hz;
		break;
	case TYPE_LXT_HZ:
		freq = lxt_hz;
		break;
	case TYPE_HIRC_HZ:
		freq = hirc_hz;
		break;
	case TYPE_LIRC_HZ:
		freq = lirc_hz;
		break;
	case TYPE_CAPLL_HZ:
		freq = pllfreq[0];
		break;
	case TYPE_SYSPLL_HZ:
		freq = 180000000;
		break;
	case TYPE_DDRPLL_HZ:
		freq = pllfreq[1];
		break;
	case TYPE_APLL_HZ:
		freq = pllfreq[2];
		break;
	case TYPE_EPLL_HZ:
		freq = pllfreq[3];
		break;
	case TYPE_VPLL_HZ:
		freq = pllfreq[4];
		break;
	case TYPE_CAPLL_DIV2_HZ:
		freq = pllfreq[0]/2;
		break;
	case TYPE_CAPLL_DIV4_HZ:
		freq = pllfreq[0]/4;
		break;
	case TYPE_EPLL_DIV2_HZ:
		freq = pllfreq[3]/2;
		break;
	case TYPE_EPLL_DIV4_HZ:
		freq = pllfreq[3]/4;
		break;
	case TYPE_VPLL_DIV2_HZ:
		freq =  pllfreq[4]/2;
		break;
	case TYPE_CA35_HZ:
		freq = caclk;
		break;
	case TYPE_AXI0_ACLK_HZ:
		freq = pllfreq[0]/2;
		break;
	case TYPE_SYSCLK0_HZ:
		freq = sysclk[0];
		break;
	case TYPE_SYSCLK1_HZ:
		freq = sysclk[1];
		break;
	case TYPE_SYSCLK1_DIV2_HZ:
		freq = sysclk[1]/2;
		break;
	case TYPE_HCLK0_HZ:
	case TYPE_HCLK1_HZ:
	case TYPE_HCLK2_HZ:
		freq = sysclk[1];
		break;
	case TYPE_HCLK3_HZ:
		freq = sysclk[1]/2;
		break;
	case TYPE_PCLK0_HZ:
	case TYPE_PCLK1_HZ:
	case TYPE_PCLK2_HZ:
		freq = sysclk[1];
		break;
	case TYPE_PCLK3_HZ:
	case TYPE_PCLK4_HZ:
		freq = sysclk[1]/2;
		break;
	case TYPE_PCLK3_DIV_4096_HZ:
	case TYPE_PCLK4_DIV_4096_HZ:
		freq = sysclk[1]/2/4096;
		break;
	case TYPE_USBPHY0_HZ:
		freq = usbphy0_hz;
		break;
	case TYPE_USBPHY1_HZ:
		freq = usbphy1_hz;
		break;
	default:
		freq = hxt_hz;
		break;

	}
	return freq;
}

ulong get_rate_reg(struct clk *clk, void *base)
{
	u32 maskMap[5] = {BITMASK_1, BITMASK_1, BITMASK_2, BITMASK_3, BITMASK_4};
	u32 regVal, gateId, muxId, divId, div_val;
	struct ___ma35h0_clk_reg_info reg_info;
	ulong freq = 0;

	pr_debug("%s\n", __func__);

	gateId = clk->id;
	if ((clk->id >= ca35clk_mux) && (clk->id < dcup_div))
		muxId  = clk->id - ca35clk_mux;
	else
		muxId = UNKNOWN_MUX_ID;

	if (clk->id == dcup_div)
		muxId = UNKNOWN_MUX_ID;

	if (clk->id >= dcup_div)
		divId  = clk->id - dcup_div;
	else
		divId = UNKNOWN_DIV_ID;

	reg_info.clkGate = __clk_id_gate_sets[gateId];
	reg_info.clkMux  = __clk_id_mux_sets[muxId];
	reg_info.clkDiv  = __clk_id_div_sets[divId];

	//Get root clock source
	reg_info.clkGate.rate = Assign_root_clk(__clk_id_gate_sets[gateId].type);

	//Assign_root_clk(u32 clk_sel_type)
	reg_info.clkMux.muxMap[0] = Assign_root_clk(
	                                __clk_sel_type_tbl[muxId].muxMap[0]);
	reg_info.clkMux.muxMap[1] = Assign_root_clk(
	                                __clk_sel_type_tbl[muxId].muxMap[1]);
	reg_info.clkMux.muxMap[2] = Assign_root_clk(
	                                __clk_sel_type_tbl[muxId].muxMap[2]);
	reg_info.clkMux.muxMap[3] = Assign_root_clk(
	                                __clk_sel_type_tbl[muxId].muxMap[3]);

	if(muxId == UNKNOWN_MUX_ID) {
		if(divId == UNKNOWN_DIV_ID) {
			return reg_info.clkGate.rate;
		} else {
			regVal = readl(base + reg_info.clkDiv.offst);
			div_val = (regVal & (maskMap[reg_info.clkDiv.bitWidth] <<
			                     reg_info.clkDiv.bitIdx)) >> reg_info.clkDiv.bitIdx;
			freq = reg_info.clkGate.rate/reg_info.clkDiv.divMap[div_val];
		}
	} else {
		regVal = readl(base + reg_info.clkMux.offst);
		regVal = regVal >> reg_info.clkMux.bitIdx;
		regVal = regVal & maskMap[reg_info.clkMux.bitWidth];
		if(divId == UNKNOWN_DIV_ID) {
			freq = reg_info.clkMux.muxMap[regVal];
			pr_debug("Get rate: %s, [sel_val = %d] @[%ld] Hz\n", clk_names[clk->id], regVal,
			         freq);
		} else {
			regVal = readl(base + reg_info.clkDiv.offst);
			div_val = (regVal & (maskMap[reg_info.clkDiv.bitWidth] <<
			                     reg_info.clkDiv.bitIdx)) >> reg_info.clkDiv.bitIdx;
			freq = reg_info.clkGate.rate/reg_info.clkDiv.divMap[div_val];
			pr_debug("Get rate: %s, regVal=0x%x, [div_val: %d](/%d) @[%ld]\n",
			         clk_names[clk->id], regVal, div_val, reg_info.clkDiv.divMap[div_val], freq);
		}
	}

	return freq;
}

static ulong ma35h0_clk_get_rate(struct clk *clk)
{
	struct __ma35h0_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate = 0;

	pr_debug("%s\n", __func__);

	if(clk->id <= vpll) {
		if (ARRAY_SIZE(__ma35h0_init_plls) <= clk->id)
			return -ENXIO;
		priv = &__ma35h0_init_plls[clk->id];
		if (!priv->data || !priv->ops->recalc_rate)
			return -ENXIO;
		rate = priv->ops->recalc_rate(priv, ma35h0_pll_parent_rate(priv));
	} else {
		rate = get_rate_reg(clk, priv->base);
	}

	return rate;
}

ulong set_rate_reg(struct clk *clk, ulong rate, void *base)
{
	u32 i, selectIdx = 0xFF, sel_val, div_val;
	u32 maskMap[5] = {BITMASK_1, BITMASK_1, BITMASK_2, BITMASK_3, BITMASK_4};
	u32 regVal, gateId, muxId, divId;
	struct ___ma35h0_clk_reg_info reg_info;
	ulong freq = 0;

	pr_debug("%s\n", __func__);

	gateId = clk->id;
	if ((clk->id >= ca35clk_mux) && (clk->id < dcup_div))
		muxId  = clk->id - ca35clk_mux;
	else
		muxId = UNKNOWN_MUX_ID;

	if (clk->id == dcup_div)
		muxId = UNKNOWN_MUX_ID;

	if (clk->id >= dcup_div)
		divId  = clk->id - dcup_div;
	else
		divId = UNKNOWN_DIV_ID;

	reg_info.clkGate = __clk_id_gate_sets[gateId];
	reg_info.clkMux  = __clk_id_mux_sets[muxId];
	reg_info.clkDiv  = __clk_id_div_sets[divId];

	//Get root clock source
	reg_info.clkGate.rate = Assign_root_clk(__clk_id_gate_sets[gateId].type);

	//Assign_root_clk(u32 clk_sel_type)
	reg_info.clkMux.muxMap[0] = Assign_root_clk(
	                                __clk_sel_type_tbl[muxId].muxMap[0]);
	reg_info.clkMux.muxMap[1] = Assign_root_clk(
	                                __clk_sel_type_tbl[muxId].muxMap[1]);
	reg_info.clkMux.muxMap[2] = Assign_root_clk(
	                                __clk_sel_type_tbl[muxId].muxMap[2]);
	reg_info.clkMux.muxMap[3] = Assign_root_clk(
	                                __clk_sel_type_tbl[muxId].muxMap[3]);

	if(muxId == UNKNOWN_MUX_ID) {
		if(divId == UNKNOWN_DIV_ID) {
			return reg_info.clkGate.rate;
		} else {
			regVal = readl(base + reg_info.clkDiv.offst);
			div_val = (regVal & (maskMap[reg_info.clkDiv.bitWidth]
			                     <<reg_info.clkDiv.bitIdx)) >> reg_info.clkDiv.bitIdx;
			freq = reg_info.clkGate.rate/reg_info.clkDiv.divMap[div_val];
		}
	} else {
		regVal = readl(base + reg_info.clkMux.offst);

		for (i=0; i < 16; i++) {
			if (reg_info.clkMux.muxMap[i] == rate) {
				selectIdx = i;
				break;
			}
		}
		if (selectIdx == 0xFF) {
			// not found, need to error handle, SET default value
			sel_val = selectIdx << reg_info.clkMux.bitIdx;
			sel_val = (regVal & ~
			           (maskMap[reg_info.clkMux.bitWidth]<<reg_info.clkMux.bitIdx)) | sel_val;
			writel(sel_val, base + reg_info.clkMux.offst);
			freq = reg_info.clkGate.rate;
			return freq;
		} else {
			sel_val = selectIdx << reg_info.clkMux.bitIdx;
			sel_val = (regVal & ~
			           (maskMap[reg_info.clkMux.bitWidth]<<reg_info.clkMux.bitIdx)) | sel_val;
			writel(sel_val, base + reg_info.clkMux.offst);
			freq = reg_info.clkMux.muxMap[selectIdx];

			// DCUP & EMAC DIV Read only
			if(divId == UNKNOWN_DIV_ID) {
				freq = reg_info.clkMux.muxMap[selectIdx];
			} else {
				regVal = readl(base + reg_info.clkDiv.offst);
				div_val = (regVal & (maskMap[reg_info.clkDiv.bitWidth]
				                     <<reg_info.clkDiv.bitIdx)) >> reg_info.clkDiv.bitIdx;
				freq = reg_info.clkGate.rate/reg_info.clkDiv.divMap[div_val];
			}
		}
	}

	return freq;
}

static ulong ma35h0_clk_set_rate(struct clk *clk, ulong rate)
{
	ulong freq = 0;
	struct __ma35h0_clk_priv *priv = dev_get_priv(clk->dev);

	pr_debug("%s\n", __func__);

	if(clk->id <= vpll) {
		if (ARRAY_SIZE(__ma35h0_init_plls) <= clk->id)
			return -ENXIO;
		priv = &__ma35h0_init_plls[clk->id];
		if (!priv->data || !priv->ops->set_rate)
			return -ENXIO;
		freq = priv->ops->set_rate(priv, rate, ma35h0_pll_parent_rate(priv));
		if (freq)
			return freq;
	} else {
		freq = set_rate_reg(clk, rate, priv->base);
	}

	return freq;
}

static int ma35h0_clk_enable(struct clk *clk)
{
	struct __ma35h0_clk_priv *priv = dev_get_priv(clk->dev);
	int ret = 0;
	u32 regVal, sel_val;

	pr_debug("%s\n", __func__);

	if(clk->id <= vpll) {
		if (ARRAY_SIZE(__ma35h0_init_plls) <= clk->id)
			return -ENXIO;
		priv = &__ma35h0_init_plls[clk->id];
		if (!priv->data)
			return -ENXIO;

		if (priv->ops->enable_clk)
			ret = priv->ops->enable_clk(priv, 1);
	} else {
		regVal = readl(priv->base+__clk_id_gate_sets[clk->id].offst);
		sel_val = (regVal & ~(BITMASK_1 << __clk_id_gate_sets[clk->id].bitIdx)) |
		          (BITMASK_1 << __clk_id_gate_sets[clk->id].bitIdx);
		writel(sel_val, priv->base+__clk_id_gate_sets[clk->id].offst);

		if(clk->id == sdh0_gate){ /* Set sdh0 mux to syspll */
			regVal=readl(priv->base+0x18);
			regVal&=~(0x3<<16);
			writel(regVal,priv->base+0x18);
		}else if(clk->id == sdh1_gate){ /* Set sdh1 mux to syspll */
			regVal=readl(priv->base+0x18);
			regVal&=~(0x3<<18);
			writel(regVal,priv->base+0x18);
		}
	}

	return ret;
}

static int ma35h0_clk_disable(struct clk *clk)
{
	struct __ma35h0_clk_priv *priv = dev_get_priv(clk->dev);
	int ret = 0;
	u32 regVal, sel_val;

	pr_debug("%s\n", __func__);
	if(clk->id <= vpll) {
		if (ARRAY_SIZE(__ma35h0_init_plls) <= clk->id)
			return -ENXIO;

		priv = &__ma35h0_init_plls[clk->id];
		if (!priv->data)
			return -ENXIO;

		if (priv->ops->enable_clk)
			ret = priv->ops->enable_clk(priv, 0);
	} else {
		regVal = readl(priv->base+__clk_id_gate_sets[clk->id].offst);
		sel_val = (regVal & ~(BITMASK_1 << __clk_id_gate_sets[clk->id].bitIdx)) & 
		          ~(BITMASK_1 << __clk_id_gate_sets[clk->id].bitIdx);
		writel(sel_val, priv->base+__clk_id_gate_sets[clk->id].offst);
	}

	return ret;
}

ulong find_caclk_rate(void *base)
{
	u32 regVal, idx;
	u32 maskMap[5] = {BITMASK_1, BITMASK_1, BITMASK_2, BITMASK_3, BITMASK_4};
	ulong freq = 0;

	pr_debug("%s\n", __func__);

	idx = ca35clk_mux-ca35clk_mux;
	regVal = readl(base + __clk_id_mux_sets[idx].offst);
	regVal = regVal >> __clk_id_mux_sets[idx].bitIdx;
	regVal = regVal & maskMap[__clk_id_mux_sets[idx].bitWidth];

	switch(regVal) {
	case 0:
		freq = hxt_hz;
		break;
	case 1:
		freq = pllfreq[capll];
		break;
	case 2:
		freq = pllfreq[ddrpll];
		break;
	}

	return freq;
}

ulong find_sysclk_rate(void *base, u8 num)
{
	u32 regVal, idx;
	u32 maskMap[5] = {BITMASK_1, BITMASK_1, BITMASK_2, BITMASK_3, BITMASK_4};
	ulong freq = 0;

	pr_debug("%s\n", __func__);

	idx = (sysclk0_mux - ca35clk_mux) + num;
	regVal = readl(base + __clk_id_mux_sets[idx].offst);
	regVal = (regVal >> __clk_id_mux_sets[idx].bitIdx);
	regVal = regVal & maskMap[__clk_id_mux_sets[idx].bitWidth];

	if(num == 0) {
		switch(regVal) {
		case 0:
			freq = pllfreq[epll]/2;
			break;
		default:
			freq = 180000000;//syspll
			break;
		}
	} else {
		switch(regVal) {
		case 0:
			freq = hxt_hz;
			break;
		case 1:
		default:
			freq = 180000000;//syspll
			break;
		}
	}
	return freq;
}

static int ma35h0_clk_probe(struct udevice *dev)
{
	int i, err;
	struct __ma35h0_clk_priv *priv;
	struct __ma35h0_clk_data *data = dev_get_priv(dev);

	pr_debug("%s\n", __func__);

	data->base = (void *)dev_read_addr(dev);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	pr_debug("\n%s(dev=%p) 0x%p\n", __func__, dev, data->base);

	err = clk_get_by_index(dev, 0, &data->parent_hxt);
	if (err)
		return err;
	err = clk_get_by_index(dev, 1, &data->parent_lxt);
	if (err)
		return err;
	err = clk_get_by_index(dev, 2, &data->parent_hirc);
	if (err)
		return err;
	err = clk_get_by_index(dev, 3, &data->parent_lirc);
	if (err)
		return err;
	err = clk_get_by_index(dev, 4, &data->parent_usbphy0);
	if (err)
		return err;
	err = clk_get_by_index(dev, 5, &data->parent_usbphy1);
	if (err)
		return err;

	data->parent_hxt.rate = clk_get_rate(&data->parent_hxt);
	hxt_hz = clk_get_rate(&data->parent_hxt);

	data->parent_lxt.rate = clk_get_rate(&data->parent_lxt);
	lxt_hz = clk_get_rate(&data->parent_lxt);

	data->parent_hirc.rate = clk_get_rate(&data->parent_hirc);
	hirc_hz = clk_get_rate(&data->parent_hirc);

	data->parent_lirc.rate = clk_get_rate(&data->parent_lirc);
	lirc_hz = clk_get_rate(&data->parent_lirc);

	data->parent_usbphy0.rate = clk_get_rate(&data->parent_usbphy0);
	usbphy0_hz = clk_get_rate(&data->parent_usbphy0);

	data->parent_usbphy1.rate = clk_get_rate(&data->parent_usbphy1);
	usbphy1_hz = clk_get_rate(&data->parent_usbphy1);

	for (i = 0; i < ARRAY_SIZE(__ma35h0_init_plls); ++i) {
		priv = &__ma35h0_init_plls[i];
		priv->data = data;
		if(priv->pll->type == MA35H0_CAPLL)
			priv->pll->rate = pllfreq[priv->pll->type]= CLK_GetPLLFreq_SMICPLL(priv, data->parent_hxt.rate);
		else
			priv->pll->rate = pllfreq[priv->pll->type]= CLK_GetPLLFreq(priv, data->parent_hxt.rate);
	}

	caclk = find_caclk_rate(data->base);
	sysclk[0] = find_sysclk_rate(data->base, 0);
	sysclk[1] = find_sysclk_rate(data->base, 1);

// debug
#if 0
	printf("clk\t\tfrequency\n");
	for (i = 0; i < clk_max; i++) {
		const char *name = clk_names[i];
		if (name) {
			struct clk clk;
			unsigned long rate;

			clk.id = i;
			err = clk_request(dev, &clk);
			if (err < 0)
				return err;

			rate = clk_get_rate(&clk);

			clk_free(&clk);

			if ((rate == (unsigned long)-ENOSYS) ||
			    (rate == (unsigned long)-ENXIO) ||
			    (rate == (unsigned long)-EIO))
				printf("%10s%20s\n", name, "unknown");
			else
				printf("%10s%20lu\n", name, rate);
		}
	}
#endif
	return 0;
}

static struct clk_ops ma35h0_clk_ops = {
	.set_rate = ma35h0_clk_set_rate,
	.get_rate = ma35h0_clk_get_rate,
	.enable = ma35h0_clk_enable,
	.disable = ma35h0_clk_disable,
};

static const struct udevice_id ma35h0_clk_ids[] = {
	{ .compatible = "nuvoton,ma35h0-clk" },
	{ },
};

U_BOOT_DRIVER(ma35h0_clk) = {
	.name = "ma35h0_clk",
	.id = UCLASS_CLK,
	.of_match = ma35h0_clk_ids,
	.ops = &ma35h0_clk_ops,
	.priv_auto_alloc_size = sizeof(struct __ma35h0_clk_data),
	.probe = ma35h0_clk_probe,
	.flags = DM_FLAG_PRE_RELOC,
};
