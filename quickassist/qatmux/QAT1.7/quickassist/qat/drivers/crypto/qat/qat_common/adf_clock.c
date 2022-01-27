/*
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 * Copyright(c) 2018 Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 * qat-linux@intel.com
 *
 * BSD LICENSE
 * Copyright(c) 2018 Intel Corporation.
 * Redistribution and use in source and binary forms, with or without
 * odification, are permitted provided that the following conditions
 * re met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/delay.h>
#include <linux/debugfs.h>

#include "adf_accel_devices.h"
#include "adf_common_drv.h"
#include "adf_dev_err.h"

#define MEASURE_CLOCK_RETRIES 10
#define MEASURE_CLOCK_DELTA_THRESHOLD 100
#define MEASURE_CLOCK_DELAY 10000
#define ME_CLK_DIVIDER 16

/* Tolerance value in percent applied to both MIN and MAX AE frequencies */
#define ADF_AE_FREQ_TOLERANCE (1)

#ifdef CONFIG_DEBUG_FS
#define CLK_DBGFS_FILE "frequency"

static int clock_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t clock_debugfs_read(struct file *file, char __user *user_buf,
				  size_t count, loff_t *ppos)
{
	struct adf_accel_dev *accel_dev = file->private_data;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	u32 speed;

	char buf[16] = {0};
	int len = 0;

	speed = hw_data->clock_frequency;

	len = scnprintf(buf, sizeof(buf), "%u\n", speed);
	if (len < 0)
		return -EFAULT;
	return simple_read_from_buffer(user_buf, count, ppos, buf, len + 1);
}

static const struct file_operations clock_fops = {
	.open = clock_debugfs_open,
	.read = clock_debugfs_read,
};

int adf_clock_debugfs_add(struct adf_accel_dev *accel_dev)
{
	accel_dev->clock_dbgfile = debugfs_create_file(CLK_DBGFS_FILE, 0400,
						       accel_dev->debugfs_dir,
						       accel_dev,
						       &clock_fops);
	if (!accel_dev->clock_dbgfile) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to create frequency debugfs entry\n");
		return -EFAULT;
	}

	return 0;
}
#else
int adf_clock_debugfs_add(struct adf_accel_dev *accel_dev)
{
	return 0;
}
#endif
EXPORT_SYMBOL_GPL(adf_clock_debugfs_add);

static inline s64 timespec_to_us(const struct timespec *ts)
{
	return ((s64)ts->tv_sec * USEC_PER_SEC +
		 (ts->tv_nsec + NSEC_PER_USEC / 2) / NSEC_PER_USEC);
}

/**
 * measure_clock() -- Measure the CPM clock frequency
 * @accel_dev: Pointer to acceleration device.
 * @frequency: Pointer to returned frequency in Hz.
 *
 * Return: 0 on success, error code otherwise.
 */
static int measure_clock(struct adf_accel_dev *accel_dev,
			 u32 *frequency)
{
	struct timespec ts1;
	struct timespec ts2;
	struct timespec ts3;
	struct timespec ts4;
	u64 delta_us = 0;
	u64 timestamp1 = 0;
	u64 timestamp2 = 0;
	u64 temp = 0;
	int tries = 0;

	if (!accel_dev || !frequency)
		return -EIO;

	do {
		getnstimeofday(&ts1);
		if (adf_get_fw_timestamp(accel_dev, &timestamp1)) {
			dev_err(&GET_DEV(accel_dev),
				"Failed to get fw timestamp\n");
			return -EIO;
		}
		getnstimeofday(&ts2);
		delta_us = timespec_to_us(&ts2) - timespec_to_us(&ts1);
	} while (delta_us > MEASURE_CLOCK_DELTA_THRESHOLD &&
		 ++tries < MEASURE_CLOCK_RETRIES);
	if (tries >= MEASURE_CLOCK_RETRIES) {
		dev_err(&GET_DEV(accel_dev), "Excessive clock measure delay\n");
		return -EIO;
	}

	usleep_range(MEASURE_CLOCK_DELAY, MEASURE_CLOCK_DELAY * 2);

	tries = 0;
	do {
		getnstimeofday(&ts3);
		if (adf_get_fw_timestamp(accel_dev, &timestamp2)) {
			dev_err(&GET_DEV(accel_dev),
				"Failed to get fw timestamp\n");
			return -EIO;
		}
		getnstimeofday(&ts4);
		delta_us = timespec_to_us(&ts4) - timespec_to_us(&ts3);
	} while (delta_us > MEASURE_CLOCK_DELTA_THRESHOLD &&
		 ++tries < MEASURE_CLOCK_RETRIES);
	if (tries >= MEASURE_CLOCK_RETRIES) {
		dev_err(&GET_DEV(accel_dev), "Excessive clock measure delay\n");
		return -EIO;
	}

	delta_us = timespec_to_us(&ts3) - timespec_to_us(&ts1);

	/* Don't pretend that this gives better than 100KHz resolution */
	temp = (timestamp2 - timestamp1) * ME_CLK_DIVIDER * 10 + (delta_us / 2);
	do_div(temp, delta_us);
	*frequency = temp * 100000;

	return 0;
}

/**
 * adf_dev_measure_clock() -- Measure the CPM clock frequency
 * @accel_dev: Pointer to acceleration device.
 * @frequency: Pointer to returned frequency in Hz.
 * @min: Minimum expected frequency
 * @max: Maximum expected frequency
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_dev_measure_clock(struct adf_accel_dev *accel_dev,
			  u32 *frequency, u32 min, u32 max)
{
	int ret = 0;
	u32 freq = 0;
	u32 min_tolerance = ADF_APPLY_PERCENTAGE(min, ADF_AE_FREQ_TOLERANCE);
	u32 max_tolerance = ADF_APPLY_PERCENTAGE(max, ADF_AE_FREQ_TOLERANCE);

	ret = measure_clock(accel_dev, &freq);
	if (ret)
		return ret;

	if (freq < (min - min_tolerance)) {
		dev_warn(&GET_DEV(accel_dev),
			 "Slow clock %d MHz measured, assuming %d\n",
			 freq, min);
		freq = min;
	} else if (freq > (max + max_tolerance)) {
		dev_warn(&GET_DEV(accel_dev),
			 "Fast clock %d MHz measured, assuming %d\n",
			 freq, max);
		freq = max;
	}
	*frequency = freq;
	return 0;
}
EXPORT_SYMBOL_GPL(adf_dev_measure_clock);
