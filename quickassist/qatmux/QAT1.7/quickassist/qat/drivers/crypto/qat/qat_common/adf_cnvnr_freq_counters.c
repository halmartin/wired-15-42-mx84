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

#include "adf_cnvnr_freq_counters.h"
#include "adf_common_drv.h"
#include "icp_qat_fw_init_admin.h"
#include <linux/seq_file.h>

#define CNVNR_DBG_FILE "cnv_errors"
#define ADF_CNVNR_ERR_MASK 0xFFF
#define LINE	\
	"+-----------------------------------------------------------------+\n"
#define BANNER	\
	"|             CNV Error Freq Statistics for Qat Device            |\n"
static char *cnvnr_err_str[] = {"No Error      ",
				"Checksum Error",
				"Length Error-P",
				"Decomp Error  ",
				"Xlat Error    ",
				"Length Error-C",
				"Unknown Error "};

static int qat_cnvnr_freq_counters_show(struct seq_file *sfile, void *v)
{
	struct adf_accel_dev *accel_dev;
	struct adf_hw_device_data *hw_device;
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp resp;
	u8 num_aes = 0;
	u8 i = 0;
	u8 error_type = 0;
	s16 latest_error = 0;

	accel_dev = sfile->private;
	hw_device = accel_dev->hw_device;
	if (!hw_device) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to get hw_device.\n");
		return -EFAULT;
	}
	num_aes = hw_device->get_num_aes(hw_device);

	seq_printf(sfile, LINE);
	seq_printf(sfile, BANNER);
	seq_printf(sfile, LINE);
	memset(&req, 0, sizeof(struct icp_qat_fw_init_admin_req));
	req.cmd_id = ICP_QAT_FW_CNV_STATS_GET;
	for (i = 0; i < num_aes; i++) {
		memset(&resp, 0, sizeof(struct icp_qat_fw_init_admin_resp));
		if (adf_put_admin_msg_sync(accel_dev, i, &req, &resp) ||
		    resp.status) {
			return -EFAULT;
		}
		error_type = CNV_ERROR_TYPE_GET(resp.latest_error);
		if (error_type == CNV_ERR_TYPE_DECOMP_PRODUCED_LENGTH_ERROR ||
		    error_type == CNV_ERR_TYPE_DECOMP_CONSUMED_LENGTH_ERROR) {
			latest_error = CNV_ERROR_LENGTH_DELTA_GET(
							resp.latest_error);
		} else if (error_type == CNV_ERR_TYPE_DECOMPRESSION_ERROR ||
			  error_type == CNV_ERR_TYPE_TRANSLATION_ERROR) {
			latest_error = CNV_ERROR_DECOMP_STATUS_GET(
							resp.latest_error);
		} else {
			latest_error = resp.latest_error & ADF_CNVNR_ERR_MASK;
		}
		seq_printf(sfile,
			   "|[AE %2d]: TotalErrors: %5d : LastError: %s [%5d]  |\n",
			   i, resp.error_count,
			   cnvnr_err_str[error_type],
			   latest_error);
		seq_printf(sfile, LINE);
	}
	return 0;
}

static int qat_cnvnr_freq_counters_open(struct inode *inode, struct file *file)
{
	struct adf_accel_dev *accel_dev;

	accel_dev = inode->i_private;
	if (!accel_dev)
		return -EFAULT;

	if (!adf_dev_started(accel_dev)) {
		dev_err(&GET_DEV(accel_dev),
			"Qat Device not started\n");
		return -EFAULT;
	}
	return single_open(file, qat_cnvnr_freq_counters_show, accel_dev);
}

static const struct file_operations qat_cnvnr_ctr_fops = {
	.owner = THIS_MODULE,
	.open = qat_cnvnr_freq_counters_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * adf_cnvnr_freq_counters_add() - Create debugfs entry for
 * acceleration device Freq counters.
 * @accel_dev:  Pointer to acceleration device.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_cnvnr_freq_counters_add(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_device;

	if (!accel_dev)
		return -EFAULT;

	hw_device = accel_dev->hw_device;
	if (!hw_device) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to get hw_device.\n");
		return -EFAULT;
	}

	/* accel_dev->debugfs_dir should always be non-NULL here */
	accel_dev->cnvnr_dbgfile = debugfs_create_file(CNVNR_DBG_FILE, 0400,
						       accel_dev->debugfs_dir,
						       accel_dev,
						       &qat_cnvnr_ctr_fops);
	if (!accel_dev->cnvnr_dbgfile) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to create qat cnvnr freq counters debugfs entry.\n");
		return -EFAULT;
	}
	return 0;
}

/**
 * adf_cnvnr_freq_counters_remove() - Remove debugfs entry for
 * acceleration device Freq counters.
 * @accel_dev:  Pointer to acceleration device.
 *
 * Return: void
 */
void adf_cnvnr_freq_counters_remove(struct adf_accel_dev *accel_dev)
{
	debugfs_remove(accel_dev->cnvnr_dbgfile);
}
