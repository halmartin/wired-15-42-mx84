/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.

 * GPL LICENSE SUMMARY
 * Copyright(c) 2018 Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.

 * Contact Information:
 * qat-linux@intel.com

 * BSD LICENSE
 * Copyright(c) 2018 Intel Corporation.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:

 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.

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

#include "adf_fw_counters.h"
#include "adf_common_drv.h"
#include "icp_qat_fw_init_admin.h"
#include <linux/seq_file.h>

static int qat_fw_counters_show(struct seq_file *sfile, void *v)
{
	struct adf_accel_dev *accel_dev;
	struct adf_hw_device_data *hw_device;
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp resp;
	u8 num_aes = 0;
	u8 i = 0;
	char line[] = "+------------------------------------------------+\n";
	char banner[] = "| FW Statistics for Qat Device                   |\n";

	accel_dev = sfile->private;
	hw_device = accel_dev->hw_device;
	if (!hw_device) {
		dev_dbg(&GET_DEV(accel_dev),
			"Failed to get hw_device.\n");
		return -EFAULT;
	}
	num_aes = hw_device->get_num_aes(hw_device);

	seq_printf(sfile, line);
	seq_printf(sfile, banner);
	seq_printf(sfile, line);
	memset(&req, 0, sizeof(struct icp_qat_fw_init_admin_req));
	req.cmd_id = ICP_QAT_FW_COUNTERS_GET;
	for (i = 0; i < num_aes; i++) {
		memset(&resp, 0, sizeof(struct icp_qat_fw_init_admin_resp));
		if (adf_put_admin_msg_sync(accel_dev, i, &req, &resp) ||
		    resp.status) {
			return -EFAULT;
		}
		seq_printf(sfile,
			   "| %s[AE %2d]:%20llu |\n",
			   "Firmware Requests ", i,
			   resp.req_rec_count);
		seq_printf(sfile,
			   "| %s[AE %2d]:%20llu |\n",
			   "Firmware Responses", i,
			   resp.resp_sent_count);
		seq_printf(sfile, line);
	}
	return 0;
}

static int qat_fw_counters_open(struct inode *inode, struct file *file)
{
	struct adf_accel_dev *accel_dev;

	accel_dev = inode->i_private;
	if (!accel_dev)
		return -EFAULT;

	if (!adf_dev_started(accel_dev))
		return -EFAULT;

	return single_open(file, qat_fw_counters_show, accel_dev);
}

static const struct file_operations qat_fw_counters_fops = {
	.owner = THIS_MODULE,
	.open = qat_fw_counters_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * adf_fw_counters_add() - Create debugfs entry for
 * acceleration device FW counters.
 * @accel_dev:  Pointer to acceleration device.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_fw_counters_add(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_device;

	if (!accel_dev)
		return -EFAULT;

	hw_device = accel_dev->hw_device;
	if (!hw_device) {
		dev_dbg(&GET_DEV(accel_dev),
			"Failed to get hw_device.\n");
		return -EFAULT;
	}

	/* accel_dev->debugfs_dir should always be non-NULL here */
	accel_dev->fw_cntr_dbgfile = debugfs_create_file("fw_counters", 0400,
							 accel_dev->debugfs_dir,
							 accel_dev,
							 &qat_fw_counters_fops);
	if (!accel_dev->fw_cntr_dbgfile) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to create qat fw counters debugfs entry.\n");
		return -EFAULT;
	}
	return 0;
}

/**
 * adf_fw_counters_remove() - Remove debugfs entry for
 * acceleration device FW counters.
 * @accel_dev:  Pointer to acceleration device.
 *
 * Return: void
 */
void adf_fw_counters_remove(struct adf_accel_dev *accel_dev)
{
	debugfs_remove(accel_dev->fw_cntr_dbgfile);
}
