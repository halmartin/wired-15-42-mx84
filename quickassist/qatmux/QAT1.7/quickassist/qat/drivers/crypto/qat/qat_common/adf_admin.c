/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2014 Intel Corporation.
  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  qat-linux@intel.com

  BSD LICENSE
  Copyright(c) 2014 Intel Corporation.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include "adf_accel_devices.h"
#include "adf_common_drv.h"
#include "icp_qat_fw_init_admin.h"
#include "adf_heartbeat.h"
#include "adf_cfg.h"

static const u8 const_tab[1024] __aligned(1024) = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x02, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x13, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13,
0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0xfe, 0xdc, 0xba, 0x98, 0x76,
0x54, 0x32, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd, 0xab,
0x89, 0x98, 0xba, 0xdc, 0xfe, 0x10, 0x32, 0x54, 0x76, 0xc3, 0xd2, 0xe1, 0xf0,
0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc1, 0x05, 0x9e,
0xd8, 0x36, 0x7c, 0xd5, 0x07, 0x30, 0x70, 0xdd, 0x17, 0xf7, 0x0e, 0x59, 0x39,
0xff, 0xc0, 0x0b, 0x31, 0x68, 0x58, 0x15, 0x11, 0x64, 0xf9, 0x8f, 0xa7, 0xbe,
0xfa, 0x4f, 0xa4, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6a, 0x09, 0xe6, 0x67, 0xbb, 0x67, 0xae,
0x85, 0x3c, 0x6e, 0xf3, 0x72, 0xa5, 0x4f, 0xf5, 0x3a, 0x51, 0x0e, 0x52, 0x7f,
0x9b, 0x05, 0x68, 0x8c, 0x1f, 0x83, 0xd9, 0xab, 0x5b, 0xe0, 0xcd, 0x19, 0x05,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0xcb, 0xbb, 0x9d, 0x5d, 0xc1, 0x05, 0x9e, 0xd8, 0x62, 0x9a, 0x29,
0x2a, 0x36, 0x7c, 0xd5, 0x07, 0x91, 0x59, 0x01, 0x5a, 0x30, 0x70, 0xdd, 0x17,
0x15, 0x2f, 0xec, 0xd8, 0xf7, 0x0e, 0x59, 0x39, 0x67, 0x33, 0x26, 0x67, 0xff,
0xc0, 0x0b, 0x31, 0x8e, 0xb4, 0x4a, 0x87, 0x68, 0x58, 0x15, 0x11, 0xdb, 0x0c,
0x2e, 0x0d, 0x64, 0xf9, 0x8f, 0xa7, 0x47, 0xb5, 0x48, 0x1d, 0xbe, 0xfa, 0x4f,
0xa4, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x6a, 0x09, 0xe6, 0x67, 0xf3, 0xbc, 0xc9, 0x08, 0xbb,
0x67, 0xae, 0x85, 0x84, 0xca, 0xa7, 0x3b, 0x3c, 0x6e, 0xf3, 0x72, 0xfe, 0x94,
0xf8, 0x2b, 0xa5, 0x4f, 0xf5, 0x3a, 0x5f, 0x1d, 0x36, 0xf1, 0x51, 0x0e, 0x52,
0x7f, 0xad, 0xe6, 0x82, 0xd1, 0x9b, 0x05, 0x68, 0x8c, 0x2b, 0x3e, 0x6c, 0x1f,
0x1f, 0x83, 0xd9, 0xab, 0xfb, 0x41, 0xbd, 0x6b, 0x5b, 0xe0, 0xcd, 0x19, 0x13,
0x7e, 0x21, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define ADF_ADMIN_POLL_INTERVAL_US 20
#define ADF_ADMIN_POLL_RETRIES 5000

int adf_put_admin_msg_sync(struct adf_accel_dev *accel_dev, u32 ae,
			   void *in, void *out)
{
	struct adf_admin_comms *admin = accel_dev->admin;
	int offset = ae * ADF_ADMINMSG_LEN * 2;
	void __iomem *mailbox = admin->mailbox_addr;
	int mb_offset = ae * ADF_MAILBOX_STRIDE;
	int times, received;
	struct icp_qat_fw_init_admin_req *request = in;

	mutex_lock(&admin->lock);

	if (ADF_CSR_RD(mailbox, mb_offset) == 1) {
		mutex_unlock(&admin->lock);
		return -EAGAIN;
	}

	memcpy(admin->virt_addr + offset, in, ADF_ADMINMSG_LEN);
	ADF_CSR_WR(mailbox, mb_offset, 1);
	received = 0;
	for (times = 0; times < ADF_ADMIN_POLL_RETRIES; times++) {
		usleep_range(ADF_ADMIN_POLL_INTERVAL_US,
			     ADF_ADMIN_POLL_INTERVAL_US * 2);
		if (ADF_CSR_RD(mailbox, mb_offset) == 0) {
			received = 1;
			break;
		}
	}
	if (received)
		memcpy(out, admin->virt_addr + offset +
		       ADF_ADMINMSG_LEN, ADF_ADMINMSG_LEN);
	else
		dev_err(&GET_DEV(accel_dev),
			"Failed to send admin msg %d to accelerator %d\n",
			request->cmd_id, ae);

	mutex_unlock(&admin->lock);
	return received ? 0 : -EFAULT;
}

static inline int adf_set_dc_ibuf(struct adf_accel_dev *accel_dev,
				  struct icp_qat_fw_init_admin_req *req)
{
	char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
	unsigned long ibuf_size = 0;

	if (!adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				     ADF_INTER_BUF_SIZE, val)) {
		if (kstrtoul(val, 0, &ibuf_size))
			return -EFAULT;
	}

	if ((ibuf_size != 32) && (ibuf_size != 64))
		ibuf_size = 64;

	req->ibuf_size_in_kb = ibuf_size;

	return 0;
}

int adf_send_admin(struct adf_accel_dev *accel_dev,
		   struct icp_qat_fw_init_admin_req *req,
		   struct icp_qat_fw_init_admin_resp *resp,
		   u32 ae_mask)
{
	int i;
	u32 mask;

	for (i = 0, mask = ae_mask; mask; i++, mask >>= 1) {
		if (!(mask & 1))
			continue;
		if (adf_put_admin_msg_sync(accel_dev, i, req, resp) ||
		    resp->status)
			return -EFAULT;
	}

	return 0;
}

static int adf_init_me(struct adf_accel_dev *accel_dev)
{
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp resp;
	struct adf_hw_device_data *hw_device = accel_dev->hw_device;
	u32 ae_mask = hw_device->ae_mask;

	memset(&req, 0, sizeof(req));
	req.cmd_id = ICP_QAT_FW_INIT_ME;

	if (adf_set_dc_ibuf(accel_dev, &req))
		return -EFAULT;
	if (adf_send_admin(accel_dev, &req, &resp, ae_mask))
		return -EFAULT;

	return 0;
}

static int adf_get_dc_capabilities(struct adf_accel_dev *accel_dev,
				   u32 *capabilities)
{
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp resp;
	u32 ae_mask = 1;

	memset(&req, 0, sizeof(req));
	req.cmd_id = ICP_QAT_FW_COMP_CAPABILITY_GET;

	if (adf_send_admin(accel_dev, &req, &resp, ae_mask))
		return -EFAULT;

	*capabilities = resp.extended_features;

	return 0;
}

static int adf_set_heartbeat_timer(struct adf_accel_dev *accel_dev)
{
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp resp;
	struct adf_hw_device_data *hw_device = accel_dev->hw_device;
	u32 ae_mask = hw_device->ae_mask;

	memset(&req, 0, sizeof(req));
	req.cmd_id = ICP_QAT_FW_HEARTBEAT_TIMER_SET;
	req.init_cfg_ptr = accel_dev->admin->phy_hb_addr;
	if (adf_get_hb_timer(accel_dev, &req.heartbeat_ticks))
		return -EINVAL;

	if (adf_send_admin(accel_dev, &req, &resp, ae_mask))
		return -EFAULT;

	return 0;
}

static int adf_set_fw_constants(struct adf_accel_dev *accel_dev)
{
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp resp;
	struct adf_hw_device_data *hw_device = accel_dev->hw_device;
	u32 ae_mask = hw_device->ae_mask;

	memset(&req, 0, sizeof(req));
	req.cmd_id = ICP_QAT_FW_CONSTANTS_CFG;

	req.init_cfg_sz = sizeof(const_tab);
	req.init_cfg_ptr = accel_dev->admin->const_tbl_addr;

	if (adf_send_admin(accel_dev, &req, &resp, ae_mask))
		return -EFAULT;

	return 0;
}

static int adf_get_fw_status(struct adf_accel_dev *accel_dev,
			     u8 *major, u8 *minor, u8 *patch)
{
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp resp;
	u32 ae_mask = 1;

	memset(&req, 0, sizeof(req));
	req.cmd_id = ICP_QAT_FW_STATUS_GET;

	if (adf_send_admin(accel_dev, &req, &resp, ae_mask))
		return -EFAULT;

	*major = resp.version_major_num;
	*minor = resp.version_minor_num;
	*patch = resp.version_patch_num;

	return 0;
}

int adf_get_fw_timestamp(struct adf_accel_dev *accel_dev, u64 *timestamp)
{
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp rsp;
	unsigned int ae_mask = 1;

	if (!accel_dev || !timestamp)
		return -EFAULT;

	memset(&req, 0, sizeof(req));
	req.cmd_id = ICP_QAT_FW_TIMER_GET;

	if (adf_send_admin(accel_dev, &req, &rsp, ae_mask))
		return -EFAULT;

	*timestamp = rsp.timestamp;
	return 0;
}

/**
 * adf_send_admin_init() - Function sends init message to FW
 * @accel_dev: Pointer to acceleration device.
 *
 * Function sends admin init message to the FW
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_send_admin_init(struct adf_accel_dev *accel_dev)
{
	u32 dc_capabilities = 0;
	int ret;

	ret = adf_init_me(accel_dev);
	if (ret)
		return ret;
	ret = adf_get_dc_capabilities(accel_dev, &dc_capabilities);
	if (ret)
		return ret;
	accel_dev->hw_device->extended_dc_capabilities = dc_capabilities;

	ret = adf_set_heartbeat_timer(accel_dev);
	if (ret == -EINVAL)
		return ret;
	else if (ret)
		dev_err(&GET_DEV(accel_dev), "Heartbeat is not supported\n");

	adf_get_fw_status(accel_dev,
			  &accel_dev->fw_versions.fw_version_major,
			  &accel_dev->fw_versions.fw_version_minor,
			  &accel_dev->fw_versions.fw_version_patch);
	dev_dbg(&GET_DEV(accel_dev), "FW version: %d.%d.%d\n",
		accel_dev->fw_versions.fw_version_major,
		accel_dev->fw_versions.fw_version_minor,
		accel_dev->fw_versions.fw_version_patch);

	return adf_set_fw_constants(accel_dev);
}
EXPORT_SYMBOL_GPL(adf_send_admin_init);

int adf_init_admin_comms(struct adf_accel_dev *accel_dev)
{
	struct adf_admin_comms *admin;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct adf_bar *pmisc =
		&GET_BARS(accel_dev)[hw_data->get_misc_bar_id(hw_data)];
	void __iomem *csr = pmisc->virt_addr;
	void __iomem *mailbox = (void __iomem *)((uintptr_t)csr +
				 ADF_MAILBOX_BASE_OFFSET);
	u64 reg_val;

	admin = kzalloc_node(sizeof(*accel_dev->admin), GFP_KERNEL,
			     dev_to_node(&GET_DEV(accel_dev)));
	if (!admin)
		return -ENOMEM;
	admin->virt_addr = dma_alloc_coherent(&GET_DEV(accel_dev), PAGE_SIZE,
					      &admin->phy_addr, GFP_KERNEL);
	if (!admin->virt_addr) {
		dev_err(&GET_DEV(accel_dev), "Failed to allocate dma buff\n");
		kfree(admin);
		return -ENOMEM;
	}

	admin->virt_tbl_addr = dma_alloc_coherent(&GET_DEV(accel_dev),
						  PAGE_SIZE,
						  &admin->const_tbl_addr,
						  GFP_KERNEL);
	if (!admin->virt_tbl_addr) {
		dev_err(&GET_DEV(accel_dev), "Failed to allocate const_tbl\n");
		dma_free_coherent(&GET_DEV(accel_dev), PAGE_SIZE,
				  admin->virt_addr, admin->phy_addr);
		kfree(admin);
		return -ENOMEM;
	}

	memcpy(admin->virt_tbl_addr, const_tab, sizeof(const_tab));

	admin->virt_hb_addr = dma_alloc_coherent(&GET_DEV(accel_dev),
						 PAGE_SIZE,
						 &admin->phy_hb_addr,
						 GFP_KERNEL);
	if (!admin->virt_hb_addr) {
		dev_err(&GET_DEV(accel_dev), "Failed to allocate hb stats\n");
		dma_free_coherent(&GET_DEV(accel_dev), PAGE_SIZE,
				  admin->virt_addr, admin->phy_addr);
		dma_free_coherent(&GET_DEV(accel_dev), PAGE_SIZE,
				  admin->virt_tbl_addr, admin->const_tbl_addr);
		kfree(admin);
		return -ENOMEM;
	}

	reg_val = (u64)admin->phy_addr;
	ADF_CSR_WR(csr, ADF_ADMINMSGUR_OFFSET, reg_val >> 32);
	ADF_CSR_WR(csr, ADF_ADMINMSGLR_OFFSET, reg_val);
	mutex_init(&admin->lock);
	admin->mailbox_addr = mailbox;
	accel_dev->admin = admin;
	return 0;
}
EXPORT_SYMBOL_GPL(adf_init_admin_comms);

void adf_exit_admin_comms(struct adf_accel_dev *accel_dev)
{
	struct adf_admin_comms *admin = accel_dev->admin;

	if (!admin)
		return;

	if (admin->virt_addr)
		dma_free_coherent(&GET_DEV(accel_dev), PAGE_SIZE,
				  admin->virt_addr, admin->phy_addr);
	if (admin->virt_tbl_addr)
		dma_free_coherent(&GET_DEV(accel_dev), PAGE_SIZE,
				  admin->virt_tbl_addr, admin->const_tbl_addr);

	if (admin->virt_hb_addr)
		dma_free_coherent(&GET_DEV(accel_dev), PAGE_SIZE,
				  admin->virt_hb_addr, admin->phy_hb_addr);


	mutex_destroy(&admin->lock);
	kfree(admin);
	accel_dev->admin = NULL;
}
EXPORT_SYMBOL_GPL(adf_exit_admin_comms);
