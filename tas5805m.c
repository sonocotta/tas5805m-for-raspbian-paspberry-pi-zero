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
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/version.h>
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
#include <sound/tlv.h>

#define IS_KERNEL_MAJOR_BELOW_5 (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))

/* Datasheet-defined registers on page 0, book 0 */
#define REG_PAGE        0x00
#define REG_DEVICE_CTRL_1   0x02
#define REG_DEVICE_CTRL_2   0x03
#define REG_SIG_CH_CTRL     0x28
#define REG_SAP_CTRL_1      0x33
#define REG_FS_MON      0x37
#define REG_BCK_MON     0x38
#define REG_CLKDET_STATUS   0x39
#define REG_VOL_CTL     0x4c
#define REG_AGAIN       0x54
#define REG_ADR_PIN_CTRL    0x60
#define REG_ADR_PIN_CONFIG  0x61
#define REG_CHAN_FAULT      0x70
#define REG_GLOBAL_FAULT1   0x71
#define REG_GLOBAL_FAULT2   0x72
#define REG_FAULT       0x78
#define REG_BOOK        0x7f

#define TAS5805M_RATES      (SNDRV_PCM_RATE_8000_96000)
#define TAS5805M_FORMATS    (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
                SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

/* DEVICE_CTRL_2 register values */
#define DCTRL2_MODE_DEEP_SLEEP 0x00
#define DCTRL2_MODE_SLEEP  0x01
#define DCTRL2_MODE_HIZ    0x02
#define DCTRL2_MODE_PLAY   0x03

#define DCTRL2_MUTE        0x08
#define DCTRL2_DIS_DSP     0x10

/* REG_FAULT register values */
#define ANALOG_FAULT_CLEAR 0x80

/* This sequence of register writes must always be sent, prior to the
 * 5ms delay while we wait for the DSP to boot. 
 * Sending 0x11 to DCTL2 resets the volume setting, so we send 0x01.
 */
static const uint8_t dsp_cfg_preboot[] = {
    0x00, 0x00, 0x7f, 0x00, 0x03, 0x02, 0x01, 0x10,   // reset dsp & write DCTRL2_MODE_HIZ to DCTL2, keeps TLDV setting
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0x00, 0x03, 0x02,
};

static const uint8_t dsp_cfg_firmware_missing[] = {
    0x00, 0x00, 0x7f, 0x00, 0x54, 0x03, 0x78, 0x80,   // minimal config for PVDD = 24V when firmware is missing
};

struct tas5805m_priv {
    struct regulator        *pvdd;
    struct gpio_desc        *gpio_pdn_n;

    uint8_t             *dsp_cfg_data;
    int             dsp_cfg_len;

    struct regmap          *regmap;

    bool                is_powered;
    bool                is_muted;

	struct workqueue_struct *my_wq;
	struct work_struct work;
	struct snd_soc_component *component;
    int trigger_cmd;
};

static void tas5805m_refresh(struct snd_soc_component *component)
{
    struct tas5805m_priv *tas5805m =
        snd_soc_component_get_drvdata(component);
    struct regmap *rm = tas5805m->regmap;
    int ret;

    printk(KERN_DEBUG "tas5805m_refresh: Refreshing the component\n");

    printk(KERN_DEBUG "\tregmap_write: %#02x %#02x", REG_PAGE, 0x00);
    ret = regmap_write(rm, REG_PAGE, 0x00);
    if (ret)
        printk(KERN_ERR "tas5805m_refresh: regmap_write failed for REG_PAGE: %d\n", ret);

    printk(KERN_DEBUG "\tregmap_write: %#02x %#02x", REG_BOOK, 0x00);
    ret = regmap_write(rm, REG_BOOK, 0x00);
    if (ret)
        printk(KERN_ERR "tas5805m_refresh: regmap_write failed for REG_BOOK: %d\n", ret);

    printk(KERN_DEBUG "\tregmap_write: %#02x %#02x", REG_PAGE, 0x00);
    ret = regmap_write(rm, REG_PAGE, 0x00);
    if (ret)
        printk(KERN_ERR "tas5805m_refresh: regmap_write failed for REG_PAGE: %d\n", ret);

    /* Set/clear digital soft-mute */
    printk(KERN_DEBUG "\tregmap_write: %#02x %#02x", REG_DEVICE_CTRL_2, (tas5805m->is_muted ? DCTRL2_MUTE : 0) |
        DCTRL2_MODE_PLAY);
    ret = regmap_write(rm, REG_DEVICE_CTRL_2,
        (tas5805m->is_muted ? DCTRL2_MUTE : 0) |
        DCTRL2_MODE_PLAY);
    if (ret)
        printk(KERN_ERR "tas5805m_refresh: regmap_write failed for REG_DEVICE_CTRL_2: %d\n", ret);

    printk(KERN_DEBUG "\tregmap_write: %#02x %#02x", REG_FAULT, ANALOG_FAULT_CLEAR);
    ret = regmap_write(rm, REG_FAULT, ANALOG_FAULT_CLEAR);    // Is necessary for compatibility with TAS5828m
    if (ret)
        printk(KERN_ERR "tas5805m_refresh: regmap_write failed for REG_FAULT: %d\n", ret);
}

static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(tas5805m_vol_tlv, -10350, 50, 1);    // New (name, min, step, mute)

static const struct snd_kcontrol_new tas5805m_snd_controls[] = {        // New
    SOC_SINGLE_TLV ("Master Playback Volume", REG_VOL_CTL, 0, 255, 1, tas5805m_vol_tlv), // (xname, reg, shift, max, invert, tlv_array)
};

static void send_cfg(struct regmap *rm,
             const uint8_t *s, unsigned int len)
{
    unsigned int i;
    int ret;

    printk(KERN_DEBUG "send_cfg: Sending configuration to the device\n");

    for (i = 0; i + 1 < len; i += 2) {
        printk(KERN_DEBUG "\tregmap_write: %#02x %#02x", s[i], s[i + 1]);
        ret = regmap_write(rm, s[i], s[i + 1]);
        if (ret)
            printk(KERN_ERR "send_cfg: regmap_write failed for %#02x: %d\n", s[i], ret);
    }
}

static void tas5805m_work_handler(struct work_struct *work) {
    struct tas5805m_priv *tas5805m = container_of(work, struct tas5805m_priv, work);
    struct regmap *rm = tas5805m->regmap;
    int ret;
    unsigned int chan, global1, global2;

    switch (tas5805m->trigger_cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        printk(KERN_DEBUG "tas5805m_work_handler: DSP startup\n");

        usleep_range(5000, 10000);
        send_cfg(rm, dsp_cfg_preboot, ARRAY_SIZE(dsp_cfg_preboot));
        usleep_range(5000, 15000);

        if (tas5805m->dsp_cfg_data) {
            send_cfg(rm, tas5805m->dsp_cfg_data, tas5805m->dsp_cfg_len);
        } else {
            send_cfg(rm, dsp_cfg_firmware_missing, ARRAY_SIZE(dsp_cfg_firmware_missing));
        }

        tas5805m->is_powered = true;
        tas5805m_refresh(tas5805m->component);
        break;

    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        printk(KERN_DEBUG "tas5805m_work_handler: DSP shutdown\n");

        tas5805m->is_powered = false;

        ret = regmap_write(rm, REG_PAGE, 0x00);
        if (ret)
            printk(KERN_ERR "tas5805m_work_handler: regmap_write failed for REG_PAGE: %d\n", ret);

        ret = regmap_write(rm, REG_BOOK, 0x00);
        if (ret)
            printk(KERN_ERR "tas5805m_work_handler: regmap_write failed for REG_BOOK: %d\n", ret);

        ret = regmap_read(rm, REG_CHAN_FAULT, &chan);
        if (ret)
            printk(KERN_ERR "tas5805m_work_handler: regmap_read failed for REG_CHAN_FAULT: %d\n", ret);

        ret = regmap_read(rm, REG_GLOBAL_FAULT1, &global1);
        if (ret)
            printk(KERN_ERR "tas5805m_work_handler: regmap_read failed for REG_GLOBAL_FAULT1: %d\n", ret);

        ret = regmap_read(rm, REG_GLOBAL_FAULT2, &global2);
        if (ret)
            printk(KERN_ERR "tas5805m_work_handler: regmap_read failed for REG_GLOBAL_FAULT2: %d\n", ret);

        printk(KERN_DEBUG "tas5805m_work_handler: fault regs: CHAN=%02x, GLOBAL1=%02x, GLOBAL2=%02x\n",
               chan, global1, global2);

        ret = regmap_write(rm, REG_DEVICE_CTRL_2, DCTRL2_MODE_PLAY | DCTRL2_MUTE);
        if (ret)
            printk(KERN_ERR "tas5805m_work_handler: regmap_write failed for REG_DEVICE_CTRL_2: %d\n", ret);
        break;

    default:
        printk(KERN_ERR "tas5805m_work_handler: Invalid command\n");
        break;
    }
}

/* The TAS5805M DSP can't be configured until the I2S clock has been
 * present and stable for 5ms, or else it won't boot and we get no
 * sound.
 */
static int tas5805m_trigger(struct snd_pcm_substream *substream, int cmd,
                            struct snd_soc_dai *dai) {
    struct snd_soc_component *component = dai->component;
    struct tas5805m_priv *tas5805m = snd_soc_component_get_drvdata(component);

    printk(KERN_DEBUG "tas5805m_trigger: cmd=%d\n", cmd);

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        tas5805m->trigger_cmd = cmd;
        tas5805m->component = component;
        schedule_work(&tas5805m->work);
        break;

    default:
        printk(KERN_ERR "tas5805m_trigger: Invalid command\n");
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
    .controls       = tas5805m_snd_controls,
    .num_controls       = ARRAY_SIZE(tas5805m_snd_controls),
    .dapm_widgets       = tas5805m_dapm_widgets,
    .num_dapm_widgets   = ARRAY_SIZE(tas5805m_dapm_widgets),
    .dapm_routes        = tas5805m_audio_map,
    .num_dapm_routes    = ARRAY_SIZE(tas5805m_audio_map),
    .use_pmdown_time    = 1,
    .endianness     = 1,
};

static int tas5805m_mute(struct snd_soc_dai *dai, int mute, int direction)
{
    struct snd_soc_component *component = dai->component;
    struct tas5805m_priv *tas5805m =
        snd_soc_component_get_drvdata(component);

    printk(KERN_DEBUG "tas5805m_mute: set mute=%d (is_powered=%d)\n",
        mute, tas5805m->is_powered);
    tas5805m->is_muted = mute;
    if (tas5805m->is_powered)
        tas5805m_refresh(component);

    return 0;
}

static const struct snd_soc_dai_ops tas5805m_dai_ops = {
    .trigger        = tas5805m_trigger,
    .mute_stream        = tas5805m_mute,
    .no_capture_mute    = 1,
};

static struct snd_soc_dai_driver tas5805m_dai = {
    .name       = "tas5805m-amplifier",
    .playback   = {
        .stream_name    = "Playback",
        .channels_min   = 2,
        .channels_max   = 2,
        .rates      = TAS5805M_RATES,
        .formats    = TAS5805M_FORMATS,
    },
    .ops        = &tas5805m_dai_ops,
};

static const struct regmap_config tas5805m_regmap = {
    .reg_bits   = 8,
    .val_bits   = 8,

    /* We have quite a lot of multi-level bank switching and a
     * relatively small number of register writes between bank
     * switches.
     */
    .cache_type = REGCACHE_NONE,
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

    printk(KERN_DEBUG "tas5805m_i2c_probe: Probing the I2C device\n");

    regmap = devm_regmap_init_i2c(i2c, &tas5805m_regmap);
    if (IS_ERR(regmap)) {
        ret = PTR_ERR(regmap);
        printk(KERN_ERR "unable to allocate register map: %d\n", ret);
        return ret;
    }

    tas5805m = devm_kzalloc(dev, sizeof(struct tas5805m_priv), GFP_KERNEL);
    if (!tas5805m)
        return -ENOMEM;

    tas5805m->pvdd = devm_regulator_get(dev, "pvdd");
    if (IS_ERR(tas5805m->pvdd)) {
        printk(KERN_ERR "failed to get pvdd supply: %ld\n",
            PTR_ERR(tas5805m->pvdd));
        return PTR_ERR(tas5805m->pvdd);
    }

    dev_set_drvdata(dev, tas5805m);
    tas5805m->regmap = regmap;
    tas5805m->gpio_pdn_n = devm_gpiod_get(dev, "pdn", GPIOD_OUT_LOW);
    if (IS_ERR(tas5805m->gpio_pdn_n)) {
        printk(KERN_ERR "error requesting PDN gpio: %ld\n",
            PTR_ERR(tas5805m->gpio_pdn_n));
        return PTR_ERR(tas5805m->gpio_pdn_n);
    }

    printk(KERN_DEBUG "tas5805m_i2c_probe: Requesting firmware\n");

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
        printk(KERN_ERR "firmware not found, using minimal config\n");
        goto err;
    }
    if ((fw->size < 2) || (fw->size & 1)) {
        printk(KERN_ERR "firmware is invalid, using minimal config\n");
        release_firmware(fw);
        goto err;
    }

    tas5805m->dsp_cfg_len = fw->size;
    tas5805m->dsp_cfg_data = devm_kmalloc(dev, fw->size, GFP_KERNEL);
    if (!tas5805m->dsp_cfg_data) {
        printk(KERN_ERR "firmware is not loaded, using minimal config\n");
        release_firmware(fw);
        goto err;
    }
    memcpy(tas5805m->dsp_cfg_data, fw->data, fw->size);
    release_firmware(fw);

err:
    printk(KERN_DEBUG "tas5805m_i2c_probe: Powering on the device\n");

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
        printk(KERN_ERR "failed to enable pvdd: %d\n", ret);
        return ret;
    }

    usleep_range(100000, 150000);
    gpiod_set_value(tas5805m->gpio_pdn_n, 1);
    usleep_range(10000, 15000);

    /* Don't register through devm. We need to be able to unregister
     * the component prior to deasserting PDN#
     */
    ret = snd_soc_register_component(dev, &soc_codec_dev_tas5805m, &tas5805m_dai, 1);
    if (ret < 0) {
        printk(KERN_ERR "unable to register codec: %d\n", ret);
        gpiod_set_value(tas5805m->gpio_pdn_n, 0);
        regulator_disable(tas5805m->pvdd);
        return ret;
    }

	printk(KERN_DEBUG "tas5805m_i2c_probe: Allocating work queue\n");
    tas5805m->my_wq = create_singlethread_workqueue("_wq");
    INIT_WORK(&tas5805m->work, tas5805m_work_handler);

    return 0;
}

#if IS_KERNEL_MAJOR_BELOW_5
static int tas5805m_i2c_remove(struct i2c_client *i2c)   // for linux-5.xx
#else
static void tas5805m_i2c_remove(struct i2c_client *i2c)    // for linux-6.xx
#endif
{
    struct device *dev = &i2c->dev;
    struct tas5805m_priv *tas5805m = dev_get_drvdata(dev);

    printk(KERN_DEBUG "tas5805m_i2c_remove: Removing the I2C device\n");

    snd_soc_unregister_component(dev);
    gpiod_set_value(tas5805m->gpio_pdn_n, 0);
    usleep_range(10000, 15000);
    regulator_disable(tas5805m->pvdd);

	cancel_work_sync(&tas5805m->work);
    flush_workqueue(tas5805m->my_wq);
    destroy_workqueue(tas5805m->my_wq);
#if IS_KERNEL_MAJOR_BELOW_5
	return 0;
#endif
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
#if IS_KERNEL_MAJOR_BELOW_5
    .probe_new  = tas5805m_i2c_probe,
#else
    .probe      = tas5805m_i2c_probe,
#endif
    .remove     = tas5805m_i2c_remove,
    .id_table   = tas5805m_i2c_id,
    .driver     = {
        .name       = "tas5805m",
        .of_match_table = of_match_ptr(tas5805m_of_match),
    },
};

module_i2c_driver(tas5805m_i2c_driver);

MODULE_AUTHOR("Andy Liu <andy-liu@ti.com>");
MODULE_AUTHOR("Daniel Beer <daniel.beer@igorinstitute.com>");
MODULE_AUTHOR("J.P. van Coolwijk <jpvc36@gmail.com>");
MODULE_DESCRIPTION("TAS5805M Audio Amplifier Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andriy Malyshenko <andriy@sonocotta.com>");