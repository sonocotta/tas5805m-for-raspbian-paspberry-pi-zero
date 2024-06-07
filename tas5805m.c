// SPDX-License-Identifier: GPL-2.0
//
// Driver for the TAS5805M Audio Amplifier
//
// Author: Andy Liu <andy-liu@ti.com>
// Author: Daniel Beer <daniel.beer@igorinstitute.com>
// Author: J.P. van Coolwijk <jpvc36@gmail.com>
//
// This is based on a driver originally written by Andy Liu at TI and
// posted here:
//
//    https://e2e.ti.com/support/audio-group/audio/f/audio-forum/722027/linux-tas5825m-linux-drivers
//
// It has been simplified a little and reworked for the 5.x ALSA SoC API.
// Silicon available to me has a functional REG_VOL_CTL (0x4c), so the original dsp workaround is not necessary.
// This driver works with a binary firmware file. When missing or invalid a minimal config for PVDD=24V is used.

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/tlv.h>									// New

/* Datasheet-defined registers on page 0, book 0 */
#define REG_PAGE		0x00
#define REG_DEVICE_CTRL_1	0x02
#define REG_DEVICE_CTRL_2	0x03
#define REG_SIG_CH_CTRL		0x28
#define REG_SAP_CTRL_1		0x33
#define REG_FS_MON		0x37
#define REG_BCK_MON		0x38
#define REG_CLKDET_STATUS	0x39
#define REG_VOL_CTL		0x4c
#define REG_AGAIN		0x54
#define REG_ADR_PIN_CTRL	0x60
#define REG_ADR_PIN_CONFIG	0x61
#define REG_CHAN_FAULT		0x70
#define REG_GLOBAL_FAULT1	0x71
#define REG_GLOBAL_FAULT2	0x72
#define REG_FAULT		0x78
#define REG_BOOK		0x7f

#define TAS5805M_RATES		(SNDRV_PCM_RATE_8000_96000)
#define TAS5805M_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
				SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

/* DEVICE_CTRL_2 register values */
#define DCTRL2_MODE_DEEP_SLEEP	0x00
#define DCTRL2_MODE_SLEEP	0x01
#define DCTRL2_MODE_HIZ		0x02
#define DCTRL2_MODE_PLAY	0x03

#define DCTRL2_MUTE		0x08
#define DCTRL2_DIS_DSP		0x10

/* REG_FAULT register values */
#define ANALOG_FAULT_CLEAR	0x80

/* This sequence of register writes must always be sent, prior to the
 * 5ms delay while we wait for the DSP to boot. 
 * Sending 0x11 to DCTL2 resets the volume setting, so we send 0x01.
 */
static const uint8_t dsp_cfg_preboot[] = {
	0x00, 0x00, 0x7f, 0x00, 0x03, 0x02, 0x01, 0x10,			// reset dsp & write DCTRL2_MODE_HIZ to DCTL2, keeps TLDV setting
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x7f, 0x00, 0x03, 0x02,
};

static const uint8_t dsp_cfg_firmware_missing[] = {
	0x00, 0x00, 0x7f, 0x00, 0x54, 0x03, 0x78, 0x80,			// minimal config for PVDD = 24V when firmware is missing
};

struct tas5805m_priv {
	struct regulator		*pvdd;
	struct gpio_desc		*gpio_pdn_n;

	uint8_t				*dsp_cfg_data;
	int				dsp_cfg_len;

	struct regmap			*regmap;

	bool				is_powered;
	bool				is_muted;
};

static void tas5805m_refresh(struct snd_soc_component *component)
{
	struct tas5805m_priv *tas5805m =
		snd_soc_component_get_drvdata(component);
	struct regmap *rm = tas5805m->regmap;

	regmap_write(rm, REG_PAGE, 0x00);
	regmap_write(rm, REG_BOOK, 0x00);
	regmap_write(rm, REG_PAGE, 0x00);

	/* Set/clear digital soft-mute */
	regmap_write(rm, REG_DEVICE_CTRL_2,
		(tas5805m->is_muted ? DCTRL2_MUTE : 0) |
		DCTRL2_MODE_PLAY);
	regmap_write(rm, REG_FAULT, ANALOG_FAULT_CLEAR);					// Is necessary for compatibility with TAS5828m
}

static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(tas5805m_vol_tlv, -10350, 50, 1);			// New (name, min, step, mute)

static const struct snd_kcontrol_new tas5805m_snd_controls[] = {				// New
	SOC_SINGLE_TLV	("Master Playback Volume", REG_VOL_CTL, 0, 255, 1, tas5805m_vol_tlv),	// (xname, reg, shift, max, invert, tlv_array)
};

static void send_cfg(struct regmap *rm,
		     const uint8_t *s, unsigned int len)
{
	unsigned int i;

	for (i = 0; i + 1 < len; i += 2)
		regmap_write(rm, s[i], s[i + 1]);
}

/* The TAS5805M DSP can't be configured until the I2S clock has been
 * present and stable for 5ms, or else it won't boot and we get no
 * sound.
 */
static int tas5805m_trigger(struct snd_pcm_substream *substream, int cmd,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tas5805m_priv *tas5805m =
		snd_soc_component_get_drvdata(component);
	struct regmap *rm = tas5805m->regmap;
	unsigned int chan, global1, global2;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:					// 1
	case SNDRV_PCM_TRIGGER_RESUME:					// 6
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:				// 4
		dev_dbg(component->dev, "DSP startup\n");

		/* We mustn't issue any I2C transactions until the I2S
		 * clock is stable. Furthermore, we must allow a 5ms
		 * delay after the first set of register writes to
		 * allow the DSP to boot before configuring it.
		 */
		usleep_range(5000, 10000);
		send_cfg(rm, dsp_cfg_preboot,				// reset all / dsp regs & write DCTRL2_MODE_HIZ to DCTL2
			ARRAY_SIZE(dsp_cfg_preboot));			// 0x01 0x11 kills TLVD volume setting
		usleep_range(5000, 15000);
		if(tas5805m->dsp_cfg_data){
			send_cfg(rm, tas5805m->dsp_cfg_data,		// write firmware
				tas5805m->dsp_cfg_len);
			}
		else	{
			send_cfg(rm, dsp_cfg_firmware_missing,		// write minimal config
				ARRAY_SIZE(dsp_cfg_firmware_missing));
			}
		tas5805m->is_powered = true;
		tas5805m_refresh(component);
		break;

	case SNDRV_PCM_TRIGGER_STOP:					// 0
	case SNDRV_PCM_TRIGGER_SUSPEND:					// 5
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:				// 3
		dev_dbg(component->dev, "DSP shutdown\n");

		tas5805m->is_powered = false;

		regmap_write(rm, REG_PAGE, 0x00);			// rm, 0x00, 0x00
		regmap_write(rm, REG_BOOK, 0x00);			// rm, 0x7f, 0x00

		regmap_read(rm, REG_CHAN_FAULT, &chan);			// rm, 0x70, 0x00
		regmap_read(rm, REG_GLOBAL_FAULT1, &global1);		// rm, 0x71, 0x00
		regmap_read(rm, REG_GLOBAL_FAULT2, &global2);		// rm, 0x72, 0x00

		dev_dbg(component->dev,
			"fault regs: CHAN=%02x, GLOBAL1=%02x, GLOBAL2=%02x\n",
			chan, global1, global2);

		regmap_write(rm, REG_DEVICE_CTRL_2,			// ramp volume up / down
				DCTRL2_MODE_PLAY | DCTRL2_MUTE);	// rm, 0x03, 0x0b
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dapm_route tas5805m_audio_map[] = {
	{ "DAC", NULL, "DAC IN" },
	{ "OUT", NULL, "DAC" },
};

static const struct snd_soc_dapm_widget tas5805m_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("DAC IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT")
};

static const struct snd_soc_component_driver soc_codec_dev_tas5805m = {
	.controls		= tas5805m_snd_controls,
	.num_controls		= ARRAY_SIZE(tas5805m_snd_controls),
	.dapm_widgets		= tas5805m_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tas5805m_dapm_widgets),
	.dapm_routes		= tas5805m_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(tas5805m_audio_map),
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int tas5805m_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct tas5805m_priv *tas5805m =
		snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "set mute=%d (is_powered=%d)\n",
		mute, tas5805m->is_powered);
	tas5805m->is_muted = mute;
	if (tas5805m->is_powered)
		tas5805m_refresh(component);

	return 0;
}

static const struct snd_soc_dai_ops tas5805m_dai_ops = {
	.trigger		= tas5805m_trigger,
	.mute_stream		= tas5805m_mute,
	.no_capture_mute	= 1,
};

static struct snd_soc_dai_driver tas5805m_dai = {
	.name		= "tas5805m-amplifier",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= TAS5805M_RATES,
		.formats	= TAS5805M_FORMATS,
	},
	.ops		= &tas5805m_dai_ops,
};

static const struct regmap_config tas5805m_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,

	/* We have quite a lot of multi-level bank switching and a
	 * relatively small number of register writes between bank
	 * switches.
	 */
	.cache_type	= REGCACHE_NONE,
};

static int tas5805m_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regmap *regmap;
	struct tas5805m_priv *tas5805m;
	char filename[128];
	const char *config_name;
	const struct firmware *fw;
	int ret;

	regmap = devm_regmap_init_i2c(i2c, &tas5805m_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "unable to allocate register map: %d\n", ret);
		return ret;
	}

	tas5805m = devm_kzalloc(dev, sizeof(struct tas5805m_priv), GFP_KERNEL);
	if (!tas5805m)
		return -ENOMEM;

	tas5805m->pvdd = devm_regulator_get(dev, "pvdd");
	if (IS_ERR(tas5805m->pvdd)) {
		dev_err(dev, "failed to get pvdd supply: %ld\n",
			PTR_ERR(tas5805m->pvdd));
		return PTR_ERR(tas5805m->pvdd);
	}

	dev_set_drvdata(dev, tas5805m);
	tas5805m->regmap = regmap;
	tas5805m->gpio_pdn_n = devm_gpiod_get(dev, "pdn", GPIOD_OUT_LOW);
	if (IS_ERR(tas5805m->gpio_pdn_n)) {
		dev_err(dev, "error requesting PDN gpio: %ld\n",
			PTR_ERR(tas5805m->gpio_pdn_n));
		return PTR_ERR(tas5805m->gpio_pdn_n);
	}

	/* This configuration must be generated by PPC3. The file loaded
	 * consists of a sequence of register writes, where bytes at
	 * even indices are register addresses and those at odd indices
	 * are register values.
	 *
	 * The fixed portion of PPC3's output prior to the 5ms delay
	 * should be omitted.
	 */
	if (device_property_read_string(dev, "ti,dsp-config-name",
					&config_name))
		config_name = "default";

	snprintf(filename, sizeof(filename), "tas5805m_dsp_%s.bin",
		 config_name);
	ret = request_firmware(&fw, filename, dev);
	if (ret) {		
		dev_err(dev, "firmware not found, using minimal config\n");
		goto err;
	}
	if ((fw->size < 2) || (fw->size & 1)) {
		dev_err(dev, "firmware is invalid, using minimal config\n");
		release_firmware(fw);
		goto err;
	}

	tas5805m->dsp_cfg_len = fw->size;
	tas5805m->dsp_cfg_data = devm_kmalloc(dev, fw->size, GFP_KERNEL);
	if (!tas5805m->dsp_cfg_data) {
		dev_err(dev, "firmware is not loaded, using minimal config\n");
		release_firmware(fw);
		goto err;
	}
	memcpy(tas5805m->dsp_cfg_data, fw->data, fw->size);
	release_firmware(fw);

	err:

	/* Do the first part of the power-on here, while we can expect
	 * the I2S interface to be quiet. We must raise PDN# and then
	 * wait 5ms before any I2S clock is sent, or else the internal
	 * regulator apparently won't come on.
	 *
	 * Also, we must keep the device in power down for 100ms or so
	 * after PVDD is applied, or else the ADR pin is sampled
	 * incorrectly and the device comes up with an unpredictable I2C
	 * address.
	 */

	ret = regulator_enable(tas5805m->pvdd);
	if (ret < 0) {
		dev_err(dev, "failed to enable pvdd: %d\n", ret);
		return ret;
	}

	usleep_range(100000, 150000);
	gpiod_set_value(tas5805m->gpio_pdn_n, 1);
	usleep_range(10000, 15000);

	/* Don't register through devm. We need to be able to unregister
	 * the component prior to deasserting PDN#
	 */
	ret = snd_soc_register_component(dev, &soc_codec_dev_tas5805m,
					 &tas5805m_dai, 1);
	if (ret < 0) {
		dev_err(dev, "unable to register codec: %d\n", ret);
		gpiod_set_value(tas5805m->gpio_pdn_n, 0);
		regulator_disable(tas5805m->pvdd);
		return ret;
	}

	return 0;
}

static void tas5805m_i2c_remove(struct i2c_client *i2c)	// for linux-6.xx
//static int tas5805m_i2c_remove(struct i2c_client *i2c)		// for linux-5.xx
{
	struct device *dev = &i2c->dev;
	struct tas5805m_priv *tas5805m = dev_get_drvdata(dev);

	snd_soc_unregister_component(dev);
	gpiod_set_value(tas5805m->gpio_pdn_n, 0);
	usleep_range(10000, 15000);
	regulator_disable(tas5805m->pvdd);
//								   for linux-6.xx
//	return 0;						// for linux-5.xx
}

static const struct i2c_device_id tas5805m_i2c_id[] = {
	{ "tas5805m", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas5805m_i2c_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id tas5805m_of_match[] = {
	{ .compatible = "ti,tas5805m", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5805m_of_match);
#endif

static struct i2c_driver tas5805m_i2c_driver = {
	.probe		= tas5805m_i2c_probe,
	.remove		= tas5805m_i2c_remove,
	.id_table	= tas5805m_i2c_id,
	.driver		= {
		.name		= "tas5805m",
		.of_match_table = of_match_ptr(tas5805m_of_match),
	},
};

module_i2c_driver(tas5805m_i2c_driver);

MODULE_AUTHOR("Andy Liu <andy-liu@ti.com>");
MODULE_AUTHOR("Daniel Beer <daniel.beer@igorinstitute.com>");
MODULE_AUTHOR("J.P. van Coolwijk <jpvc36@gmail.com>");
MODULE_DESCRIPTION("TAS5805M Audio Amplifier Driver");
MODULE_LICENSE("GPL v2");
