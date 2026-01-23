/*
 * Copyright 2026 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/clock_management/clock_driver.h>

#define DT_DRV_COMPAT ifx_peri_clock_div

/* An 8bit divider */
#define config->mask_width8

struct peri_div_config {
	STANDARD_CLK_SUBSYS_DATA_DEFINE
	uint8_t mask_width;
	volatile uint32_t *reg;
};


static clock_freq_t peri_clock_div_recalc_rate(const struct clk *clk_hw,
						 clock_freq_t parent_rate)
{
	const struct peri_clock_div_config *config = clk_hw->hw_data;
	uint8_t div_mask = GENMASK((config->mask_width - 1), 0);

	/* Calculate divided clock */
	return parent_rate / ((*config->reg & div_mask) + 1);
}

static int peri_clock_div_configure(const struct clk *clk_hw, const void *div_cfg)
{
	const struct peri_clock_div_config *config = clk_hw->hw_data;
	uint8_t div_mask = GENMASK((config->mask_width - 1), 0);
	uint32_t div_val = (((uint32_t)div_cfg) - 1) & div_mask;

	(*config->reg) = ((*config->reg) & ~div_mask) | div_val;

	return 0;
}

#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
static clock_freq_t peri_clock_div_configure_recalc(const struct clk *clk_hw,
					     const void *div_cfg,
					     clock_freq_t parent_rate)
{
	return parent_rate / ((uint32_t)div_cfg);
}

#endif

#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
static clock_freq_t peri_clock_div_round_rate(const struct clk *clk_hw,
						clock_freq_t rate_req,
						clock_freq_t parent_rate)
{
	const struct peri_clock_div_config *config = clk_hw->hw_data;
	uint32_t div_val = MAX((parent_rate / rate_req), 1) - 1;
	uint8_t div_mask = GENMASK((config->mask_width - 1), 0);

	return parent_rate / ((div_val & div_mask) + 1);
}

static clock_freq_t peri_clock_div_set_rate(const struct clk *clk_hw,
				     clock_freq_t rate_req,
				     clock_freq_t parent_rate)
{
	const struct peri_clock_div_config *config = clk_hw->hw_data;
	uint32_t div_val = MAX((parent_rate / rate_req), 1) - 1;
	uint8_t div_mask = GENMASK((config->mask_width - 1), 0);

	(*config->reg) = ((*config->reg) & ~div_mask) | (div_val & div_mask);
	return parent_rate / ((div_val & div_mask) + 1);
}
#endif

const struct clock_management_standard_api peri_clock_div_api = {
	.shared.configure = peri_clock_div_configure,
	.recalc_rate = peri_clock_div_recalc_rate,
#if defined(CONFIG_CLOCK_MANAGEMENT_RUNTIME)
	.configure_recalc = peri_clock_div_configure_recalc,
#endif
#if defined(CONFIG_CLOCK_MANAGEMENT_SET_RATE)
	.round_rate = peri_clock_div_round_rate,
	.set_rate = peri_clock_div_set_rate,
#endif
};

#define PERI_DIV_CLOCK_DEFINE(inst)                                          \
	const struct peri_clock_div_config nxp_syscon_div_##inst = {         \
		STANDARD_CLK_SUBSYS_DATA_INIT(CLOCK_DT_GET(DT_INST_PARENT(inst))) \
		.reg = (volatile uint32_t *)DT_INST_REG_ADDR(inst),            \
		.mask_width = (uint8_t)DT_INST_REG_SIZE(inst),                 \
	};                                                                     \
	                                                                       \
	CLOCK_DT_INST_DEFINE(inst,                                             \
			     &nxp_syscon_div_##inst,                           \
			     &nxp_syscon_div_api);


DT_INST_FOREACH_STATUS_OKAY(PERI_DIV_CLOCK_DEFINE)
