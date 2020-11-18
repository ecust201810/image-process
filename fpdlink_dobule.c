/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>

/* TI FPD Link III 954 deser I2C address */
#define TI954_ADDR  (0x30)
/* TI FPD Link III 953 ser I2C address */
#define TI953_ADDR  (0x18)

#define SENSOR_ADDR (0x10)

/* TI 953 alias address */
#define TI953_CAM1_ADDR (0x19)
#define TI953_CAM2_ADDR (0X1A)
/* CAM alias address */
#define CAM1_SENSOR_ADDR (0x11)
#define CAN2_SENSOR_ADDR (0x12)

struct fpdlink_reg_struct {
	u8 addr;
	u8 val;
};

/*
 *struct for ti 953 register list.
 */
struct fpdlink_953_reg_struct {
	u8 cam_addr;
	u8 addr;
	u8 val;
};

struct fpdlink_reg_struct TI954_reg_list[] = {
	{0x01, 0x02},
	{0xB3, 0x00},	/* BIST */
	{0x1F, 0x00}, 	/* 800Mbps csi clock per lane//1.6Gbps csi clock per lane */
	{0x32, 0x01},
	{0x33, 0x01},	/* 4line */
	/* {0x21, 0x3c}, */
	{0x20, 0x00},

	{0x4C, 0x01},
	{0x58, 0x5E},

	{0x5B, 0x18 << 1},
	{0x5C, (TI953_CAM1_ADDR << 1)},	/* TI 953 alias address */
	{0x5D, (SENSOR_ADDR << 1)},	/* CAM salve address */
	{0x65, (CAM1_SENSOR_ADDR << 1)},	/* CAM alias address */
	{0x72, 0xE8},
	{0x0F, 0x7F},	/* GPIOs as input */
	{0x6E, 0x10},	/* GPIO0->GPIO0, GPIO1->GPIO1 */
	{0x6F, 0x32},	/* GPIO2->GPIO2, GPIO3->GPIO3 */

	{0x4C, 0x12},
	{0x58, 0x5E},
	{0x5B, 0x18 << 1},
	{0x5C, (TI953_CAM2_ADDR << 1)},
	{0x5D, (SENSOR_ADDR << 1)},
	{0x65, (CAN2_SENSOR_ADDR << 1)},
	{0x72, 0xE9},
	{0x0F, 0x7F},	/* GPIOs as input */
	{0x6E, 0x10},	/* GPIO0->GPIO0, GPIO1->GPIO1 */
	{0x6F, 0x32},	/* GPIO2->GPIO2, GPIO3->GPIO3 */
};

struct fpdlink_953_reg_struct TI953_reg_list[] = {
	{TI953_CAM1_ADDR, 0x0E, 0xF0},	/* Enable GPIOs as output */
	{TI953_CAM1_ADDR, 0x0D, 0x00},	/* enable remote data */
	{TI953_CAM1_ADDR, 0x0D, 0x3C},	/* enable remote data */

	{TI953_CAM2_ADDR, 0x0E, 0xF0},	/* Enable GPIOs as output */
	{TI953_CAM2_ADDR, 0x0D, 0x00},	/* enable remote data */
	{TI953_CAM2_ADDR, 0x0D, 0x3C},	/* enable remote data */
};

static int i2c_wr8(struct i2c_client *client, u8 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	u8 data[2];

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = addr;
	data[1] = val;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		printk("%s: wr8 register failed\n", __func__);
		return 0;
	}

	return 0;
}

static int TI954_reg_write(struct i2c_client *client, u8 addr, u8 val)
{
	int addr_bak = client->addr;

	client->addr = TI954_ADDR;
	i2c_wr8(client, addr, val);
	client->addr = addr_bak;

	return 0;
}

static int TI953_reg_write(struct i2c_client *client,  u8 slave, u8 addr, u8 val)
{
	int addr_bak = client->addr;

	client->addr = slave;
	i2c_wr8(client, addr, val);
	client->addr = addr_bak;
	mdelay(50);
	return 0;
}
static int Ti954_flag[4] = {0};
int fpdlink_Unreset(int loc)
{
	Ti954_flag[loc] -= 1;
	return 0;
}

EXPORT_SYMBOL(fpdlink_Unreset);
int fpdlink_reset(struct i2c_client *client,int loc,int addr)
{
	int size, i;

	if(Ti954_flag[loc] == 0)
	{
		
		size = sizeof(TI954_reg_list) / sizeof(struct fpdlink_reg_struct);
		for (i = 0; i < size; i++) {
			TI954_reg_write(client, TI954_reg_list[i].addr, TI954_reg_list[i].val);
			if (TI954_reg_list[i].addr == 0x01)
				mdelay(240);
		}
		mdelay(200);
	}
	Ti954_flag[loc] += 1;

	size = sizeof(TI953_reg_list) / sizeof(struct fpdlink_953_reg_struct);
	for (i = 0; i < size; i++) {
		if(TI953_reg_list[i].cam_addr == addr)
		{
			TI953_reg_write(client, TI953_reg_list[i].cam_addr, TI953_reg_list[i].addr, TI953_reg_list[i].val);
			mdelay(100);
		}
	}
	return 0;
}

EXPORT_SYMBOL(fpdlink_reset);
