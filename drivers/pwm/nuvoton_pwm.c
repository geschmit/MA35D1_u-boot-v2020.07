// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2024
 * Nuvoton Technology Corp. <www.nuvoton.com>
 *
 * PWM driver for MA35D1
 */

#include <common.h>
#include <clk.h>
#include <div64.h>
#include <dm.h>
#include <log.h>
#include <pwm.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <power/regulator.h>

DECLARE_GLOBAL_DATA_PTR;

/* PWM Registers */

#define REG_CLK_APBCLK1         (0x40460210)

#define REG_PWM_CTL0            (0x00)
#define REG_PWM_CTL1            (0x04)
#define REG_PWM_CLKPSC01        (0x14)
#define REG_PWM_CLKPSC23        (0x18)
#define REG_PWM_CLKPSC45        (0x1C)
#define REG_PWM_CNTEN           (0x20)
#define REG_PWM_PERIOD0         (0x30)
#define REG_PWM_PERIOD1         (0x34)
#define REG_PWM_PERIOD2         (0x38)
#define REG_PWM_PERIOD3         (0x3C)
#define REG_PWM_PERIOD4         (0x40)
#define REG_PWM_PERIOD5         (0x44)
#define REG_PWM_CMPDAT0         (0x50)
#define REG_PWM_CMPDAT1         (0x54)
#define REG_PWM_CMPDAT2         (0x58)
#define REG_PWM_CMPDAT3         (0x5C)
#define REG_PWM_CMPDAT4         (0x60)
#define REG_PWM_CMPDAT5         (0x64)
#define REG_PWM_CNT0            (0x90)
#define REG_PWM_CNT1            (0x94)
#define REG_PWM_CNT2            (0x98)
#define REG_PWM_CNT3            (0x9C)
#define REG_PWM_WGCTL0          (0xB0)
#define REG_PWM_WGCTL1          (0xB4)
#define REG_PWM_POLCTL          (0xD4)
#define REG_PWM_POEN            (0xD8)
#define REG_PWM_CAPINEN         (0x200)
#define REG_PWM_CAPCTL          (0x204)
#define REG_PWM_CAPSTS          (0x208)
#define REG_PWM_RCAPDAT0        (0x20C)
#define REG_PWM_RCAPDAT1        (0x214)
#define REG_PWM_RCAPDAT2        (0x21C)
#define REG_PWM_RCAPDAT3        (0x224)
#define REG_PWM_FCAPDAT0        (0x210)
#define REG_PWM_FCAPDAT1        (0x218)
#define REG_PWM_FCAPDAT2        (0x220)
#define REG_PWM_FCAPDAT3        (0x228)
#define REG_PWM_CAPIEN          (0x250)
#define REG_PWM_CAPIF           (0x254)

#define WGCTL_MASK              0x3
#define WGCTL_HIGH              0x2
#define WGCTL_LOW               0x1


static unsigned int pwm_id_to_regbase(int pwm_id)
{
	if ((pwm_id >= 0) && (pwm_id <= 5))
		return (unsigned int) 0x40580000;
	else if ((pwm_id >= 6) && (pwm_id <= 11))
		return (unsigned int) 0x40590000;
	else if ((pwm_id >= 12) && (pwm_id <= 17))
		return (unsigned int) 0x405A0000;

	return 0;
}

int pwm_init(int pwm_id, int div, int invert)
{
	unsigned int regbase = pwm_id_to_regbase(pwm_id);
	int channel = pwm_id % 6;

	writel(readl(REG_CLK_APBCLK1) | 0x7000000, REG_CLK_APBCLK1);

	if (invert)
		writel(readl(regbase + REG_PWM_POLCTL)
		       & ~(1 << channel), regbase + REG_PWM_POLCTL);
	else
		writel(readl(regbase + REG_PWM_POLCTL)
		       | (1 << channel), regbase + REG_PWM_POLCTL);


	return 0;
}

int pwm_config(int pwm_id, int duty_ns, int period_ns)
{
	unsigned int regbase = pwm_id_to_regbase(pwm_id);
	unsigned long period, duty, prescale;
	int channel = pwm_id % 6;

	debug("%s: period_ns=%u, duty_ns=%u\n", __func__, period_ns, duty_ns);

	/* Get PCLK, calculate valid parameter range. */
	prescale = 180000000 / 50000 - 1;

	/* now pwm time unit is 20000ns. */
	period = (period_ns + 10000) / 20000;
	duty = (duty_ns + 10000) / 20000;

	/* don't want the minus 1 below change the value to -1 (0xFFFF) */
	if (period == 0)
		period = 1;
	if (duty == 0)
		duty = 1;

	/* Set prescale */
	if (channel < 2)
		writel(prescale, regbase + REG_PWM_CLKPSC01);
	else if (channel < 4)
		writel(prescale, regbase + REG_PWM_CLKPSC23);
	else
		writel(prescale, regbase + REG_PWM_CLKPSC45);

	if (channel == 0) {
		writel(period - 1, regbase + REG_PWM_PERIOD0);
		writel(duty, regbase + REG_PWM_CMPDAT0);
	} else if (channel == 1) {
		writel(period - 1, regbase + REG_PWM_PERIOD1);
		writel(duty, regbase + REG_PWM_CMPDAT1);
	} else if (channel == 2) {
		writel(period - 1, regbase + REG_PWM_PERIOD2);
		writel(duty, regbase + REG_PWM_CMPDAT2);
	} else if (channel == 3) {
		writel(period - 1, regbase + REG_PWM_PERIOD3);
		writel(duty, regbase + REG_PWM_CMPDAT3);
	} else if (channel == 4) {
		writel(period - 1, regbase + REG_PWM_PERIOD4);
		writel(duty, regbase + REG_PWM_CMPDAT4);
	} else {/* channel 5 */
		writel(period - 1, regbase + REG_PWM_PERIOD5);
		writel(duty, regbase + REG_PWM_CMPDAT5);
	}

	debug("%s: period=%lu, duty=%lu\n", __func__, period, duty);

	return 0;
}

int pwm_enable(int pwm_id)
{
	unsigned int regbase = pwm_id_to_regbase(pwm_id);
	int channel = pwm_id % 6;

	writel((readl(regbase + REG_PWM_WGCTL0) & ~(WGCTL_MASK << (channel*2)))
	       | (WGCTL_HIGH << (channel*2)), regbase + REG_PWM_WGCTL0);
	writel((readl(regbase + REG_PWM_WGCTL1) & ~(WGCTL_MASK << (channel*2)))
	       | (WGCTL_LOW << (channel*2)), regbase + REG_PWM_WGCTL1);
	writel(readl(regbase + REG_PWM_POEN)
	       | (1 << channel), regbase + REG_PWM_POEN);
	writel(readl(regbase + REG_PWM_CNTEN)
	       | (1 << channel), regbase + REG_PWM_CNTEN);

	return 0;
}

void pwm_disable(int pwm_id)
{
	unsigned int regbase = pwm_id_to_regbase(pwm_id);
	int channel = pwm_id % 6;

	writel(readl(regbase + REG_PWM_POEN)
	       & ~(1 << channel), regbase + REG_PWM_POEN);
	writel(readl(regbase + REG_PWM_CNTEN)
	       & ~(1 << channel), regbase + REG_PWM_CNTEN);

	return 0;
}

