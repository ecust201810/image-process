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

struct fpdlink_reg_struct {
	u8 addr;
	u8 val;
};

struct fpdlink_reg_struct TI954_reg_list[] = {

	{0xB3, 0x00},	/* BIST */

	{0x4C, 0x01},
	{0x58, 0x5E},
	{0x5B, (TI953_ADDR << 1)},	/* TI 953 slave address, it is able to automatically get from TI 953 */
	{0x5C, (TI953_ADDR << 1)},	/* TI 953 alias address */
	{0x5D, (SENSOR_ADDR << 1)},	/* CAM salve address */
	{0x65, (SENSOR_ADDR << 1)},	/* CAM alias address */
        {0x1F, 0x03}, 	/* 800Mbps csi clock per lane//1.6Gbps csi clock per lane */
	{0x32, 0x01},
	{0x33, 0x01},	/* 4line /2lane */
	{0x20, 0x20},

	{0x0F, 0x7F},	/* GPIOs as input */
	{0x6E, 0x10},	/* GPIO0->GPIO0, GPIO1->GPIO1 */
	{0x6F, 0x32},	/* GPIO2->GPIO2, GPIO3->GPIO3 */

};

struct fpdlink_reg_struct TI953_reg_list[] = {
	{0x0E, 0xF0},	/* Enable GPIOs as output */
	{0x0D, 0x00},	/* enable remote data */
	{0x0D, 0x0F},	/* enable remote data */
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

static u8 i2c_rd8(struct i2c_client *client, u8 addr, u8 *val)
{
	int err;
	u8 buf[2] = { addr, 0 };

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		printk("%s: reading register 0x%x from 0x%x failed\n",
			__func__, addr, client->addr);
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

static int TI954_reg_read(struct i2c_client *client, u8 addr, u8 *val)
{
	int addr_bak = client->addr;

	client->addr = TI954_ADDR;
	i2c_rd8(client, addr, val);
	client->addr = addr_bak;

	return 0;
}

static int TI953_reg_write(struct i2c_client *client, u8 addr, u8 val)
{
	int addr_bak = client->addr;

	client->addr = TI953_ADDR;
	i2c_wr8(client, addr, val);
	client->addr = addr_bak;

	return 0;
}

static int TI953_reg_read(struct i2c_client *client, u8 addr, u8 *val)
{
	int addr_bak = client->addr;

	client->addr = TI953_ADDR;
	i2c_rd8(client, addr, val);
	client->addr = addr_bak;

	return 0;
}

int fpdlink_init(struct i2c_client *client)
{
	int size, i;
	u8 val;

	size = sizeof(TI954_reg_list) / sizeof(struct fpdlink_reg_struct);
	for (i = 0; i < size; i++) {
		if (TI954_reg_list[i].addr == 0x5D || TI954_reg_list[i].addr == 0x65) {
			TI954_reg_list[i].val = client->addr << 1;
		}
		TI954_reg_write(client, TI954_reg_list[i].addr, TI954_reg_list[i].val);
		TI954_reg_read(client, TI954_reg_list[i].addr, &val);
		//printk("TI954 0x%x = 0x%x\n", TI954_reg_list[i].addr, val);
	}

	mdelay(100);

	size = sizeof(TI953_reg_list) / sizeof(struct fpdlink_reg_struct);
	for (i = 0; i < size; i++) {
		TI953_reg_write(client, TI953_reg_list[i].addr, TI953_reg_list[i].val);
		TI953_reg_read(client, TI953_reg_list[i].addr, &val);
		mdelay(200);
		//printk("TI953 0x%x = 0x%x\n", TI953_reg_list[i].addr, val);
	}

	return 0;
}

EXPORT_SYMBOL(fpdlink_init);


int fpdlink_debug(struct i2c_client *client)
{
	u8 val;



	mdelay(3000);
	TI954_reg_read(client, 0x73, &val);
	printk("TI954 0x%x = 0x%x\n", 0x73, val);
	TI954_reg_read(client, 0x74, &val);
	printk("TI954 0x%x = 0x%x\n", 0x74, val);
	TI954_reg_read(client, 0x75, &val);
	printk("TI954 0x%x = 0x%x\n", 0x75, val);
	TI954_reg_read(client, 0x76, &val);
	printk("TI954 0x%x = 0x%x\n", 0x76, val);


	return 0;
}

EXPORT_SYMBOL(fpdlink_debug);
