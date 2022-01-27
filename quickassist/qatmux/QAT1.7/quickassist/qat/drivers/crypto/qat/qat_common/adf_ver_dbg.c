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

#include "adf_ver_dbg.h"
#include "adf_common_drv.h"
#include "adf_cfg.h"

static int qat_ver_common_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t qat_hw_ver_read(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct adf_accel_dev *accel_dev = file->private_data;

	char buf[16];
	int len = 0;

	len = scnprintf(buf, sizeof(buf), "%u\n",
			accel_dev->accel_pci_dev.revid);
	if (len < 0)
		return -EFAULT;
	return simple_read_from_buffer(user_buf, count, ppos, buf, len + 1);
}

static const struct file_operations qat_hw_ver_fops = {
	.open = qat_ver_common_open,
	.read = qat_hw_ver_read,
};

static ssize_t qat_fw_ver_read(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct adf_accel_dev *accel_dev = file->private_data;

	char buf[16];
	int len = 0;

	len = scnprintf(buf, sizeof(buf), "%u.%u.%u\n",
			accel_dev->fw_versions.fw_version_major,
			accel_dev->fw_versions.fw_version_minor,
			accel_dev->fw_versions.fw_version_patch);
	if (len < 0)
		return -EFAULT;

	return simple_read_from_buffer(user_buf, count, ppos, buf, len + 1);
}

static const struct file_operations qat_fw_ver_fops = {
	.open = qat_ver_common_open,
	.read = qat_fw_ver_read,
};

static ssize_t qat_mmp_ver_read(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct adf_accel_dev *accel_dev = file->private_data;

	char buf[16];
	int len = 0;

	len = scnprintf(buf, sizeof(buf), "%u.%u.%u\n",
			accel_dev->fw_versions.mmp_version_major,
			accel_dev->fw_versions.mmp_version_minor,
			accel_dev->fw_versions.mmp_version_patch);
	if (len < 0)
		return -EFAULT;

	return simple_read_from_buffer(user_buf, count, ppos, buf, len + 1);
}

static const struct file_operations qat_mmp_ver_fops = {
	.open = qat_ver_common_open,
	.read = qat_mmp_ver_read,
};

int adf_ver_dbg_add(struct adf_accel_dev *accel_dev)
{
	struct adf_ver *p_ver = NULL;

	kfree(accel_dev->pver);
	accel_dev->pver = NULL;

	accel_dev->pver = kzalloc(sizeof(*accel_dev->pver),
						GFP_KERNEL);
	if (!accel_dev->pver)
		return -ENOMEM;

	p_ver = accel_dev->pver;

	/*Create directory for version files*/

	p_ver->ver_dir = debugfs_create_dir("version",
					accel_dev->debugfs_dir);
	if (!p_ver->ver_dir) {
		dev_err(&GET_DEV(accel_dev),
			"Unable to create version debugfs entry\n");
		goto err;
	}

	/*Create files to expose fm, hw, mmp version*/

	p_ver->hw_version = debugfs_create_file("hw", 0400,
						accel_dev->pver->ver_dir,
						accel_dev,
						&qat_hw_ver_fops);
	if (!p_ver->hw_version) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to create qat hardware version debugfs entry.\n");
		goto err;
	}

	p_ver->fw_version = debugfs_create_file("fw", 0400,
						accel_dev->pver->ver_dir,
						accel_dev,
						&qat_fw_ver_fops);
	if (!p_ver->fw_version) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to create qat firmware version debugfs entry.\n");
		goto err;
	}

	p_ver->mmp_version = debugfs_create_file("mmp", 0400,
						accel_dev->pver->ver_dir,
						accel_dev,
						&qat_mmp_ver_fops);
	if (!p_ver->mmp_version) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to create qat mmp firmware version debugfs entry.\n");
		goto err;
	}

	return 0;

err:
	adf_ver_dbg_del(accel_dev);

	return -EFAULT;
}

void adf_ver_dbg_del(struct adf_accel_dev *accel_dev)
{
	struct adf_ver *p_ver = accel_dev->pver;

	if (p_ver) {
		debugfs_remove(p_ver->mmp_version);
		p_ver->mmp_version = NULL;

		debugfs_remove(p_ver->fw_version);
		p_ver->fw_version = NULL;

		debugfs_remove(p_ver->hw_version);
		p_ver->hw_version = NULL;

		debugfs_remove(p_ver->ver_dir);
		p_ver->ver_dir = NULL;

		kfree(accel_dev->pver);
		accel_dev->pver = NULL;
	}
}
