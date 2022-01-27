/*
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 *  redistributing this file, you may do so under either license.
 *
 *  GPL LICENSE SUMMARY
 *  Copyright(c) 2017 Intel Corporation.
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  Contact Information:
 *
 *  qat-linux@intel.com
 *
 *  BSD LICENSE
 *  Copyright(c) 2017 Intel Corporation.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/seq_file.h>
#include "adf_cfg.h"
#include "adf_common_drv.h"

static DEFINE_MUTEX(qat_cfg_read_lock);

static void *qat_dev_cfg_start(struct seq_file *sfile, loff_t *pos)
{
	struct adf_cfg_device_data *dev_cfg = sfile->private;

	mutex_lock(&qat_cfg_read_lock);
	return seq_list_start(&dev_cfg->sec_list, *pos);
}

static int qat_dev_cfg_show(struct seq_file *sfile, void *v)
{
	struct list_head *list;
	struct adf_cfg_section *sec =
				list_entry(v, struct adf_cfg_section, list);

	seq_printf(sfile, "[%s]\n", sec->name);
	list_for_each(list, &sec->param_head) {
		struct adf_cfg_key_val *ptr =
			list_entry(list, struct adf_cfg_key_val, list);
		seq_printf(sfile, "%s = %s\n", ptr->key, ptr->val);
	}
	return 0;
}

static void *qat_dev_cfg_next(struct seq_file *sfile, void *v, loff_t *pos)
{
	struct adf_cfg_device_data *dev_cfg = sfile->private;

	return seq_list_next(v, &dev_cfg->sec_list, pos);
}

static void qat_dev_cfg_stop(struct seq_file *sfile, void *v)
{
	mutex_unlock(&qat_cfg_read_lock);
}

static const struct seq_operations qat_dev_cfg_sops = {
	.start = qat_dev_cfg_start,
	.next = qat_dev_cfg_next,
	.stop = qat_dev_cfg_stop,
	.show = qat_dev_cfg_show
};

static int qat_dev_cfg_open(struct inode *inode, struct file *file)
{
	int ret = seq_open(file, &qat_dev_cfg_sops);

	if (!ret) {
		struct seq_file *seq_f = file->private_data;

		seq_f->private = inode->i_private;
	}
	return ret;
}

static const struct file_operations qat_dev_cfg_fops = {
	.open = qat_dev_cfg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

/**
 * adf_cfg_dev_dbg_add() - Create debugfs entry for device configuration
 * @accel_dev:  Pointer to acceleration device.
 * *
 * Return: 0 on success, error code otherwise.
 */
int adf_cfg_dev_dbg_add(struct adf_accel_dev *accel_dev)
{
	struct adf_cfg_device_data *dev_cfg_data = accel_dev->cfg;

	/* accel_dev->debugfs_dir should always be non-NULL here */
	dev_cfg_data->debug = debugfs_create_file("dev_cfg", 0400,
						  accel_dev->debugfs_dir,
						  dev_cfg_data,
						  &qat_dev_cfg_fops);
	if (!dev_cfg_data->debug) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to create qat cfg debugfs entry.\n");
		return -EFAULT;
	}

	return 0;
}

/**
 * adf_cfg_dev_dbg_remove() - Remove debugfs entry for device configuration
 * @accel_dev:  Pointer to acceleration device.
 * *
 * Return: void
 */
void adf_cfg_dev_dbg_remove(struct adf_accel_dev *accel_dev)
{
	struct adf_cfg_device_data *dev_cfg_data = accel_dev->cfg;

	debugfs_remove(dev_cfg_data->debug);
}
