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
 * modification, are permitted provided that the following conditions
 * are met:
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
#include <linux/seq_file.h>

#include "adf_accel_devices.h"
#include "adf_common_drv.h"
#include "adf_dev_err.h"

#define ADF_PFVF_DEBUG_NAME_LEN 16

static void *adf_pfvf_start(struct seq_file *sfile, loff_t *pos)
{
	if (*pos >= NUM_PFVF_COUNTERS)
		return NULL;

	return pos;
}

static void *adf_pfvf_next(struct seq_file *sfile, void *v, loff_t *pos)
{
	(*pos)++;

	if (*pos >= NUM_PFVF_COUNTERS)
		return NULL;

	return pos;
}

static int adf_pfvf_show(struct seq_file *sfile, void *v)
{
	struct pfvf_stats *pfvf_counters = sfile->private;
	unsigned int value = 0;
	char *string = "unknown";
	loff_t field = *(loff_t *)(v);

	switch (field) {
	case 0:
		string = "Messages written to CSR";
		value = pfvf_counters->tx;
		break;
	case 1:
		string = "Messages read from CSR";
		value = pfvf_counters->rx;
		break;
	case 2:
		string = "Spurious Interrupt";
		value = pfvf_counters->spurious;
		break;
	case 3:
		string = "Block messages sent";
		value = pfvf_counters->blk_tx;
		break;
	case 4:
		string = "Block messages received";
		value = pfvf_counters->blk_rx;
		break;
	case 5:
		string = "Blocks received with CRC errors";
		value = pfvf_counters->crc_err;
		break;
	case 6:
		string = "CSR in use";
		value = pfvf_counters->busy;
		break;
	case 7:
		string = "No acknowledgment";
		value = pfvf_counters->no_ack;
		break;
	case 8:
		string = "Collisions";
		value = pfvf_counters->collision;
		break;
	case 9:
		string = "Put msg timeout";
		value = pfvf_counters->tx_timeout;
		break;
	case 10:
		string = "No response received";
		value = pfvf_counters->rx_timeout;
		break;
	case 11:
		string = "Responses received";
		value = pfvf_counters->rx_rsp;
		break;
	case 12:
		string = "Messages re-transmitted";
		value = pfvf_counters->retry;
		break;
	case 13:
		string = "Put event timeout";
		value = pfvf_counters->event_timeout;
		break;
	}
	if (value)
		seq_printf(sfile, "%s %u\n", string, value);

	return 0;
}

static void adf_pfvf_stop(struct seq_file *sfile, void *v)
{
}

static const struct seq_operations adf_pfvf_sops = {
	.start = adf_pfvf_start,
	.next = adf_pfvf_next,
	.stop = adf_pfvf_stop,
	.show = adf_pfvf_show
};

static int pfvf_debugfs_open(struct inode *inode, struct file *file)
{
	int ret = seq_open(file, &adf_pfvf_sops);

	if (!ret) {
		struct seq_file *seq_f = file->private_data;

		seq_f->private = inode->i_private;
	}
	return ret;
}

static const struct file_operations pfvf_fops = {
	.open = pfvf_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

int adf_pfvf_debugfs_add(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_to_pci_dev(accel_dev);
	int totalvfs;
	char filename[ADF_PFVF_DEBUG_NAME_LEN];
	int vf;
	struct adf_accel_vf_info *vf_info;

	accel_dev->pfvf_dbgdir = debugfs_create_dir("pfvf",
						    accel_dev->debugfs_dir);
	if (!accel_dev->pfvf_dbgdir) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to create pf/vf debugfs directory\n");
		return -EFAULT;
	}

	if (accel_dev->is_vf) {
		accel_dev->vf.pfvf_counters.stats_file =
			debugfs_create_file("pf", 0400,
					    accel_dev->pfvf_dbgdir,
					    &accel_dev->vf.pfvf_counters,
					    &pfvf_fops);
	} else {
		totalvfs = pci_sriov_get_totalvfs(pdev);
		for (vf = 0, vf_info = accel_dev->pf.vf_info;
				vf < totalvfs; vf++, vf_info++) {
			snprintf(filename, sizeof(filename), "vf%d", vf);
			vf_info->pfvf_counters.stats_file =
				debugfs_create_file(filename, 0400,
						    accel_dev->pfvf_dbgdir,
						    &vf_info->pfvf_counters,
						    &pfvf_fops);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(adf_pfvf_debugfs_add);
