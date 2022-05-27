
#include "wm8960.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>

static const uint8_t wm8960_init_seq[][2] = {
	WM8960_PAIR(WM8960_RESET, 0),     /* Reset to defaults */
	WM8960_PAIR(WM8960_PWR1, 0x01FE), /* Everything on (vmid, vref, ain, adc, micb, digenb) */
	WM8960_PAIR(WM8960_PWR2, 0x01FF), /* Everything on (dac, spk, out3, pll) */
	WM8960_PAIR(WM8960_PWR3, 0x001F), /* Everything on (mixers/mics) */
	WM8960_PAIR(WM8960_IFACE1,0x0002), /* Slave mode, Word Length 16 bits, Format I2S */
	WM8960_PAIR(WM8960_LDAC, 0x01FF), /* 0dB */
	WM8960_PAIR(WM8960_RDAC, 0x01FF), /* 0dB */
	WM8960_PAIR(WM8960_LOUT1, 0x016F), /* 0dB */
	WM8960_PAIR(WM8960_ROUT1, 0x016F), /* 0dB */
	WM8960_PAIR(WM8960_LMIX, 0x0100), /* DAC routing */
	WM8960_PAIR(WM8960_RMIX, 0x0100), /* DAC routing */
	WM8960_PAIR(WM8960_CTL2, 0x0004), /* Unmute ramp volume */
	WM8960_PAIR(WM8960_CTL1, 0x0000), /* Unmute */
	WM8960_PAIR(WM8960_CLASSD1, 0x00F7) /* enable class d */
};

K_MEM_SLAB_DEFINE(tx_mem_slab, 128, 4, 32);

/**
 * Sets up the WM8960 as a RTIO IODEV
 *
 * With a DAC input and Headphone Output
 */
void wm8960_configure(struct rtio_wm8960 *wm8960,
		      const struct device *i2c,
		      const struct device *i2s)
{
	int ret;

	wm8960->i2c = i2c;
	wm8960->i2s = i2s;

	for(int i = 0; i < ARRAY_SIZE(wm8960_init_seq); i++) {
		ret = i2c_write(wm8960->i2c, wm8960_init_seq[i], 2, WM8960_ADDR);
		__ASSERT(ret == 0, "Failed to write out wm8960 init sequence");
	}

	struct i2s_config i2s_cfg;

	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 44100;
	i2s_cfg.block_size = 128;
	i2s_cfg.timeout = 1000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_mem_slab;

	ret = i2s_configure(wm8960->i2s, I2S_DIR_TX, &i2s_cfg);
}
