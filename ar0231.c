/*
 * ar0231.c - ar0231 sensor driver
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>

#include <media/tegracam_core.h>
#include <media/ar0231.h>

#include "ar0231_mode_tbls.h"

#define AR0231_MAX_COARSE_DIFF			(10)

#define AR0231_GAIN_FACTOR			(1000000)
#define AR0231_MIN_GAIN				(1 * AR0231_GAIN_FACTOR)
#define AR0231_MAX_GAIN				(48 * AR0231_GAIN_FACTOR)
#define AR0231_MIN_FRAME_LENGTH			(1522)
#define AR0231_MAX_FRAME_LENGTH			(15520)
#define AR0231_MIN_COARSE_TIME			(5)
#define AR0231_MIN_EXPOSURE_COARSE		(0x0005)
#define AR0231_MAX_EXPOSURE_COARSE	\
	(AR0231_MAX_FRAME_LENGTH-AR0231_MAX_COARSE_DIFF)

#define AR0231_LINE_LENGTH              (1928)
#define AR0231_PIX_CLK			(88000000)


#define AR0231_EEPROM_ADDRESS		0x57
#define AR0231_EEPROM_SIZE		256
#define AR0231_EEPROM_STR_SIZE		(AR0231_EEPROM_SIZE * 2)
#define AR0231_EEPROM_BLOCK_SIZE	(1 << 8)
#define AR0231_EEPROM_NUM_BLOCKS \
	(AR0231_EEPROM_SIZE / AR0231_EEPROM_BLOCK_SIZE)

#define AR0231_FUSE_ID_START_ADDR	91
#define AR0231_FUSE_ID_SIZE		8
#define AR0231_FUSE_ID_STR_SIZE		(AR0231_FUSE_ID_SIZE * 2)

extern int fpdlink_init(struct i2c_client *client);

static const struct of_device_id ar0231_of_match[] = {
	{ .compatible = "nvidia,ar0231", },
	{ },
};
MODULE_DEVICE_TABLE(of, ar0231_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_EXPOSURE_SHORT,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_GROUP_HOLD,
	TEGRA_CAMERA_CID_HDR_EN,
	TEGRA_CAMERA_CID_FUSE_ID,
};

struct ar0231 {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;

	const char			*devname;
	struct dentry			*debugfs_dir;
	struct mutex			streaming_lock;
	bool				streaming;

	struct camera_common_eeprom_data eeprom[AR0231_EEPROM_NUM_BLOCKS];
	u8				eeprom_buf[AR0231_EEPROM_STR_SIZE];
	u8				fuse_id[AR0231_FUSE_ID_SIZE];
	u32				frame_length;
	u32				vmax;
	s64				last_exposure_long;
	s64				last_exposure_short;
	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
};

static inline void ar0231_get_frame_length_regs(ar0231_reg *regs,
				u16 frame_length)
{
	regs->addr = AR0231_FRAME_LENGTH_ADDR;
	regs->val = frame_length & 0xffff;
}

static inline void ar0231_get_coarse_time_regs(ar0231_reg *regs,
				u16 coarse_time)
{
	regs->addr = AR0231_COARSE_TIME_ADDR;
	regs->val = coarse_time & 0xffff;
}

static inline void ar0231_get_gain_regs(ar0231_reg *regs,
				u16 gain)
{
	regs->addr = AR0231_GAIN_ADDR;
	regs->val = (gain) & 0xffff;
}


//static int test_mode;
//module_param(test_mode, int, 0644);

static inline int ar0231_read_reg8(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	return 0;
}

static int ar0231_write_reg8(struct camera_common_data *s_data, u16 addr, u8 val)
{
	return 0;
}

static inline int ar0231_read_reg(struct camera_common_data *s_data,
				u16 addr, u16 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int ar0231_write_reg(struct camera_common_data *s_data, u16 addr, u16 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int ar0231_write_table(struct ar0231 *priv,
				const ar0231_reg table[])
{
#if 0
	return regmap_util_write_table_8(priv->s_data->regmap,
					 table,
					 NULL, 0,
					 AR0231_TABLE_WAIT_MS,
					 AR0231_TABLE_END);
#endif
	int i = 0;
	while (table[i].addr != AR0231_TABLE_END) {
		ar0231_write_reg(priv->s_data, table[i].addr, table[i].val);
		i++;
	}

	return 0;
}

static int ar0231_set_gain(struct tegracam_device *tc_dev, s64 val);
static int ar0231_set_frame_rate(struct tegracam_device *tc_dev, s64 val);
static int ar0231_set_exposure(struct tegracam_device *tc_dev, s64 val);

static int ar0231_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ar0231 *priv = (struct ar0231 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int err;

	priv->group_hold_prev = val;
	err = ar0231_write_reg(s_data,
				AR0231_GROUP_HOLD_ADDR, val);
	if (err) {
		dev_err(dev,
			"%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

static int ar0231_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct ar0231 *priv = (struct ar0231 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	ar0231_reg reg_list[1];
	int err;
	s32 gain64;
	u16 gain;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);	

	if (val < AR0231_MIN_GAIN)
		val = AR0231_MIN_GAIN;
	else if (val > AR0231_MAX_GAIN)
		val = AR0231_MAX_GAIN;

	/* translate value */
	gain64 = (s32)val;

	gain = (u16)(gain64 * 160 / 48 / AR0231_GAIN_FACTOR);

	ar0231_get_gain_regs(reg_list, gain);

	err = ar0231_write_reg(priv->s_data, reg_list[0].addr,
		 reg_list[0].val);
	if (err)
		goto fail;

	return 0;

fail:
	dev_err(dev, "%s: GAIN control error\n", __func__);
	return err;
}

int g_frame_length = AR0231_MIN_FRAME_LENGTH;
static int ar0231_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct ar0231 *priv = (struct ar0231 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	ar0231_reg reg_list[1];
	int err;
	int i = 0;
	s64 frame_length;

	frame_length = AR0231_PIX_CLK /  val / AR0231_LINE_LENGTH;

	g_frame_length = frame_length;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	ar0231_get_frame_length_regs(reg_list, (u16)frame_length);

	for (i = 0; i < 1; i++) {
		err = ar0231_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_err(dev, "%s: FRAME_RATE control error\n", __func__);
	return err;
}

static int ar0231_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct ar0231 *priv = (struct ar0231 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	ar0231_reg reg_list[1];
	int err;
	int i = 0;
	s64 coarse_time;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);

	coarse_time = val * AR0231_PIX_CLK / AR0231_LINE_LENGTH
			  / mode->control_properties.exposure_factor;

	if (coarse_time > (s64)AR0231_MAX_EXPOSURE_COARSE)
		coarse_time = AR0231_MAX_EXPOSURE_COARSE;
	if (coarse_time < (s64)AR0231_MIN_EXPOSURE_COARSE)
		coarse_time = AR0231_MIN_EXPOSURE_COARSE;

	if(g_frame_length < coarse_time)
	{
		ar0231_get_frame_length_regs(reg_list, (u16)coarse_time);
		for (i = 0; i < 1; i++) {
			err = ar0231_write_reg(priv->s_data, reg_list[i].addr,
				 reg_list[i].val);
			if (err)
				goto fail;
		}
	}

	ar0231_get_coarse_time_regs(reg_list, (u16)coarse_time);

	for (i = 0; i < 1; i++) {
		err = ar0231_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_err(dev, "%s: Exposure control error\n", __func__);
	return err;
}

static int ar0231_set_exposure_shr_dol_short(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}


static int ar0231_fill_string_ctrl(struct tegracam_device *tc_dev,
				struct v4l2_ctrl *ctrl)
{
	struct ar0231 *priv = (struct ar0231 *)tc_dev->priv;
	int i;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_FUSE_ID:
		for (i = 0; i < AR0231_FUSE_ID_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x",
				priv->fuse_id[i]);
		break;
	default:
		return -EINVAL;
	}

	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

static struct tegracam_ctrl_ops ar0231_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = {0, AR0231_FUSE_ID_STR_SIZE},
	.set_gain = ar0231_set_gain,
	.set_exposure = ar0231_set_exposure,
	.set_exposure_short = ar0231_set_exposure_shr_dol_short,
	.set_frame_rate = ar0231_set_frame_rate,
	.set_group_hold = ar0231_set_group_hold,
	.fill_string_ctrl = ar0231_fill_string_ctrl,
};

static int ar0231_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);

	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 1);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 0);
	usleep_range(10, 20);

	if (pw->dvdd)
		err = regulator_enable(pw->dvdd);
	if (err)
		goto ar0231_dvdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto ar0231_iovdd_fail;

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto ar0231_avdd_fail;

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 1);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 1);

	/* 1.2v input is generated on module board, adds more latency */
	usleep_range(10000, 10010);

	pw->state = SWITCH_ON;
	return 0;

ar0231_dvdd_fail:
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 0);

ar0231_iovdd_fail:
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

ar0231_avdd_fail:
	if (pw->iovdd)
		regulator_disable(pw->iovdd);

	dev_err(dev, "%s failed.\n", __func__);
	return -ENODEV;
}

static int ar0231_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
		goto power_off_done;
	}
return 0;
	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	if (pw->af_gpio)
		gpio_set_value(pw->af_gpio, 0);
	if (pw->pwdn_gpio)
		gpio_set_value(pw->pwdn_gpio, 0);
	usleep_range(1, 2);

	if (pw->avdd)
		regulator_disable(pw->avdd);
	if (pw->iovdd)
		regulator_disable(pw->iovdd);
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

power_off_done:
	pw->state = SWITCH_OFF;
	return 0;
}

static int ar0231_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);
	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);
	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->avdd = NULL;
	pw->dvdd = NULL;
	pw->iovdd = NULL;

	gpio_free(pw->pwdn_gpio);
	gpio_free(pw->reset_gpio);
	gpio_free(pw->af_gpio);

	return 0;
}

static int ar0231_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	/* ananlog 2.7v */
	err |= camera_common_regulator_get(dev,
			&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	err |= camera_common_regulator_get(dev,
			&pw->iovdd, pdata->regulators.iovdd);
	/* digital 1.2v, not all ar0231 modules draw this from CVB */
	if (pdata->regulators.dvdd != NULL)
		err |= camera_common_regulator_get(dev,
			&pw->dvdd, pdata->regulators.dvdd);

	if (!err) {
		pw->reset_gpio = pdata->reset_gpio;
		pw->af_gpio = pdata->af_gpio;
		pw->pwdn_gpio = pdata->pwdn_gpio;
	}

	pw->state = SWITCH_OFF;
	return err;
}

static
struct camera_common_pdata *ar0231_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(ar0231_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	board_priv_pdata->reset_gpio = of_get_named_gpio(node,
			"reset-gpios", 0);

	of_property_read_string(node, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	of_property_read_string(node, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);
	of_property_read_string(node, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);

	board_priv_pdata->has_eeprom =
		of_property_read_bool(node, "has-eeprom");

	of_property_read_u32(node, "fuse_id_start_addr",
		&board_priv_pdata->fuse_id_addr);

	return board_priv_pdata;
}


static int ar0231_set_mode(struct tegracam_device *tc_dev)
{
	struct ar0231 *priv = (struct ar0231 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err;

	/* usleep_range(200000, 300000); */
	fpdlink_init(priv->i2c_client);
	usleep_range(100000, 110000);

	err = ar0231_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

	return 0;
}

static int ar0231_start_streaming(struct tegracam_device *tc_dev)
{
	struct ar0231 *priv = (struct ar0231 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	int err;

	mutex_lock(&priv->streaming_lock);

	err = ar0231_write_table(priv, mode_table[AR0231_MODE_START_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else {
		priv->streaming = true;
		mutex_unlock(&priv->streaming_lock);
	}

	return 0;

exit:
	dev_err(dev, "%s: error starting stream\n", __func__);
	return err;
}

static int ar0231_stop_streaming(struct tegracam_device *tc_dev)
{
	struct ar0231 *priv = (struct ar0231 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	int err;

	mutex_lock(&priv->streaming_lock);
	err = ar0231_write_table(priv, mode_table[AR0231_MODE_STOP_STREAM]);
	if (err) {
		mutex_unlock(&priv->streaming_lock);
		goto exit;
	} else  {
		priv->streaming = false;
		mutex_unlock(&priv->streaming_lock);
	}

	return 0;

exit:
	dev_err(dev, "%s: error stopping stream\n", __func__);
	return err;
}

static struct camera_common_sensor_ops ar0231_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ar0231_frmfmt),
	.frmfmt_table = ar0231_frmfmt,
	.power_on = ar0231_power_on,
	.power_off = ar0231_power_off,
	.write_reg = ar0231_write_reg8,
	.read_reg = ar0231_read_reg8,
	.parse_dt = ar0231_parse_dt,
	.power_get = ar0231_power_get,
	.power_put = ar0231_power_put,
	.set_mode = ar0231_set_mode,
	.start_streaming = ar0231_start_streaming,
	.stop_streaming = ar0231_stop_streaming,
};

static int ar0231_debugfs_streaming_show(void *data, u64 *val)
{
	struct ar0231 *priv = data;

	mutex_lock(&priv->streaming_lock);
	*val = priv->streaming;
	mutex_unlock(&priv->streaming_lock);

	return 0;
}

static int ar0231_debugfs_streaming_write(void *data, u64 val)
{
	int err = 0;
	struct ar0231 *priv = data;
	struct i2c_client *client = priv->i2c_client;
	bool enable = (val != 0);
	int mode_index = enable ?
		(AR0231_MODE_START_STREAM) : (AR0231_MODE_STOP_STREAM);

	dev_info(&client->dev, "%s: %s sensor\n",
			__func__, (enable ? "enabling" : "disabling"));

	mutex_lock(&priv->streaming_lock);

	err = ar0231_write_table(priv, mode_table[mode_index]);
	if (err) {
		dev_err(&client->dev, "%s: error setting sensor streaming\n",
			__func__);
		goto done;
	}

	priv->streaming = enable;

done:
	mutex_unlock(&priv->streaming_lock);

	return err;
}

DEFINE_SIMPLE_ATTRIBUTE(ar0231_debugfs_streaming_fops,
	ar0231_debugfs_streaming_show,
	ar0231_debugfs_streaming_write,
	"%lld\n");

static void ar0231_debugfs_remove(struct ar0231 *priv);

static int ar0231_debugfs_create(struct ar0231 *priv)
{
	int err = 0;
	struct i2c_client *client = priv->i2c_client;
	const char *devnode;
	char debugfs_dir[16];

	err = of_property_read_string(client->dev.of_node, "devnode", &devnode);
	if (err) {
		dev_err(&client->dev, "devnode not in DT\n");
		return err;
	}
	snprintf(debugfs_dir, sizeof(debugfs_dir), "camera-%s", devnode);

	priv->debugfs_dir = debugfs_create_dir(debugfs_dir, NULL);
	if (priv->debugfs_dir == NULL)
		return -ENOMEM;

	if (!debugfs_create_file("streaming", 0644, priv->debugfs_dir, priv,
			&ar0231_debugfs_streaming_fops))
		goto error;

	return 0;

error:
	ar0231_debugfs_remove(priv);

	return -ENOMEM;
}

static void ar0231_debugfs_remove(struct ar0231 *priv)
{
	debugfs_remove_recursive(priv->debugfs_dir);
	priv->debugfs_dir = NULL;
}

static int ar0231_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ar0231_subdev_internal_ops = {
	.open = ar0231_open,
};

static int ar0231_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct tegracam_device *tc_dev;
	struct ar0231 *priv;
	int err;

	dev_info(dev, "probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct ar0231), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ar0231", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &ar0231_common_ops;
	tc_dev->v4l2sd_internal_ops = &ar0231_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ar0231_ctrl_ops;

	mutex_init(&priv->streaming_lock);

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

#if 0
	err = ar0231_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}
#endif

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	err = ar0231_debugfs_create(priv);
	if (err) {
		dev_err(&client->dev, "error creating debugfs interface");
		ar0231_debugfs_remove(priv);
		return err;
	}

	dev_info(dev, "Detected AR0231 sensor\n");

	return 0;
}

static int ar0231_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0231 *priv = (struct ar0231 *)s_data->priv;

	ar0231_debugfs_remove(priv);

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	mutex_destroy(&priv->streaming_lock);

	return 0;
}

static const struct i2c_device_id ar0231_id[] = {
	{ "ar0231", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ar0231_id);

static struct i2c_driver ar0231_i2c_driver = {
	.driver = {
		.name = "ar0231",
		.owner = THIS_MODULE,
	},
	.probe = ar0231_probe,
	.remove = ar0231_remove,
	.id_table = ar0231_id,
};

module_i2c_driver(ar0231_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony AR0231");
MODULE_AUTHOR("Josh Kuo <joshk@nvidia.com>");
MODULE_LICENSE("GPL v2");
