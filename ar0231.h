/**
 * Copyright (c) 2016-2017, NVIDIA Corporation.  All rights reserved.
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

#ifndef __AR0231_H__
#define __AR0231_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <media/nvc.h>
#include <media/nvc_image.h>

#define AR0231_IOCTL_SET_MODE			_IOW('o', 1, struct ar0231_mode)
#define AR0231_IOCTL_GET_STATUS			_IOR('o', 2, __u8)
#define AR0231_IOCTL_SET_FRAME_LENGTH		_IOW('o', 3, __u32)
#define AR0231_IOCTL_SET_COARSE_TIME		_IOW('o', 4, __u32)
#define AR0231_IOCTL_SET_GAIN			_IOW('o', 5, __u16)
#define AR0231_IOCTL_GET_SENSORDATA		_IOR('o', 6, \
	 struct ar0231_sensordata)
#define AR0231_IOCTL_SET_GROUP_HOLD		_IOW('o', 7, struct ar0231_ae)
#define AR0231_IOCTL_SET_HDR_COARSE_TIME	_IOW('o', 8, struct ar0231_hdr)
#define AR0231_IOCTL_SET_POWER			_IOW('o', 20, __u32)


#define AR0231_FRAME_LENGTH_ADDR		0x300A
#define AR0231_COARSE_TIME_ADDR			0x3012
#define AR0231_GAIN_ADDR			0x3014 /* GAIN ADDR */
#define AR0231_GROUP_HOLD_ADDR			0x3022 /* REG HOLD */
#define AR0231_SW_RESET_ADDR			0x3003 /* SW RESET */


struct ar0231_mode {
	__u32 xres;
	__u32 yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct ar0231_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct ar0231_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

#ifdef __KERNEL__
struct ar0231_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *ext_reg1;
	struct regulator *ext_reg2;
	struct clk *mclk;
	unsigned int pwdn_gpio;
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
};

struct ar0231_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int cam1_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	bool ext_reg;
	int (*power_on)(struct ar0231_power_rail *pw);
	int (*power_off)(struct ar0231_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __AR0231_H__ */
