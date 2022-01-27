/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2017 Intel Corporation.
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
  Copyright(c) 2017 Intel Corporation.
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

#include <linux/random.h>
#include <linux/module.h>
#include "adf_heartbeat.h"
#include "adf_common_drv.h"
#include "adf_cfg.h"
#include "adf_cfg_strings.h"
#include "icp_qat_fw_init_admin.h"
#include "adf_transport_internal.h"

#define MAX_HB_TICKS 0xFFFFFFFF

int adf_heartbeat_init(struct adf_accel_dev *accel_dev)
{
	if (accel_dev->heartbeat)
		adf_heartbeat_clean(accel_dev);

	accel_dev->heartbeat = kzalloc(sizeof(*accel_dev->heartbeat),
					 GFP_KERNEL);
	if (!accel_dev->heartbeat)
		return -ENOMEM;

	return 0;
}

void adf_heartbeat_clean(struct adf_accel_dev *accel_dev)
{
	kfree(accel_dev->heartbeat);
	accel_dev->heartbeat = NULL;
}

int adf_get_hb_timer(struct adf_accel_dev *accel_dev, unsigned int *value)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	char timer_str[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
	unsigned int timer_val = ADF_CFG_HB_DEFAULT_VALUE;
	u32 clk_per_sec = 0;

	if (!hw_data->get_ae_clock)
		return -EINVAL;

	clk_per_sec = (u32)hw_data->get_ae_clock(hw_data);

	/* Get Heartbeat Timer value from the configuration */
	if (!adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				     ADF_HEARTBEAT_TIMER, (char *)timer_str)) {
		if (kstrtouint((char *)timer_str, ADF_CFG_BASE_DEC,
			       &timer_val))
			timer_val = ADF_CFG_HB_DEFAULT_VALUE;
	}

	if (timer_val < ADF_MIN_HB_TIMER_MS) {
		dev_err(&GET_DEV(accel_dev),
			"%s value cannot be lesser than %u\n",
			ADF_HEARTBEAT_TIMER, ADF_MIN_HB_TIMER_MS);
		return -EINVAL;
	}

	/* Convert msec to clocks */
	clk_per_sec = clk_per_sec / 1000;
	*value = timer_val * clk_per_sec;

	return 0;
}

struct adf_hb_count {
	u16	ae_thread[ADF_NUM_HB_CNT_PER_AE];
};

int adf_get_heartbeat_status(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_device = accel_dev->hw_device;
	struct icp_qat_fw_init_admin_hb_stats *live_s =
			(struct icp_qat_fw_init_admin_hb_stats *)
					accel_dev->admin->virt_hb_addr;
	const size_t max_aes = hw_device->get_num_aes(hw_device);
	const size_t stats_size = max_aes *
			sizeof(struct icp_qat_fw_init_admin_hb_stats);
	int ret = 0;
	size_t ae, thr;

	/*
	 * Memory layout of Heartbeat
	 *
	 * +----------------+----------------+---------+
	 * |   Live value   |   Last value   |  Count  |
	 * +----------------+----------------+---------+
	 * \_______________/\_______________/\________/
	 *         ^                ^            ^
	 *         |                |            |
	 *         |                |            max_aes * sizeof(adf_hb_count)
	 *         |            max_aes * sizeof(icp_qat_fw_init_admin_hb_stats)
	 *         max_aes * sizeof(icp_qat_fw_init_admin_hb_stats)
	 */
	struct icp_qat_fw_init_admin_hb_stats *curr_s;
	struct icp_qat_fw_init_admin_hb_stats *last_s = live_s + max_aes;
	struct adf_hb_count *count = (struct adf_hb_count *)(last_s + max_aes);

	curr_s = kmalloc(stats_size, GFP_KERNEL);
	if (!curr_s)
		return -ENOMEM;

	memcpy(curr_s, live_s, stats_size);

	for (ae = 0; ae < max_aes; ++ae) {
		for (thr = 0; thr < ADF_NUM_HB_CNT_PER_AE; ++thr) {
			struct icp_qat_fw_init_admin_hb_cnt *curr =
					&curr_s[ae].stats[thr];
			struct icp_qat_fw_init_admin_hb_cnt *prev =
					&last_s[ae].stats[thr];
			u16 req = curr->req_heartbeat_cnt;
			u16 resp = curr->resp_heartbeat_cnt;
			u16 last = prev->resp_heartbeat_cnt;

			if ((thr == ADF_AE_ADMIN_THREAD || req != resp) &&
			    resp == last) {
				u16 retry = ++count[ae].ae_thread[thr];

				if (retry >= ADF_CFG_HB_COUNT_THRESHOLD)
					ret = -EIO;
			} else {
				count[ae].ae_thread[thr] = 0;
			}
		}
	}

	/* Copy current stats for the next iteration */
	memcpy(last_s, curr_s, stats_size);
	kfree(curr_s);

	return ret;
}
EXPORT_SYMBOL_GPL(adf_get_heartbeat_status);

int adf_heartbeat_status(struct adf_accel_dev *accel_dev,
			 enum adf_device_heartbeat_status *hb_status)
{
	/* Heartbeat is not implemented in VFs at the moment so they do not
	 * set get_heartbeat_status. Also, in case the device is not up,
	 * unsupported should be returned */
	if (!accel_dev || !accel_dev->hw_device ||
	    !accel_dev->hw_device->get_heartbeat_status) {
		*hb_status = DEV_HB_UNSUPPORTED;
		return 0;
	}

	if (!adf_dev_started(accel_dev) ||
	    test_bit(ADF_STATUS_RESTARTING, &accel_dev->status)) {
		*hb_status = DEV_HB_UNRESPONSIVE;
		return 0;
	}

	accel_dev->heartbeat->hb_sent_counter++;
	if (unlikely(accel_dev->hw_device->get_heartbeat_status(accel_dev))) {
		dev_err(&GET_DEV(accel_dev),
			"ERROR: QAT is not responding.");
		*hb_status = DEV_HB_UNRESPONSIVE;
		accel_dev->heartbeat->hb_failed_counter++;
		return adf_notify_fatal_error(accel_dev);
	}

	*hb_status = DEV_HB_ALIVE;

	return 0;
}

#ifdef QAT_HB_FAIL_SIM
static int adf_set_max_hb_timer(struct adf_accel_dev *accel_dev)
{
	struct icp_qat_fw_init_admin_req req;
	struct icp_qat_fw_init_admin_resp resp;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	u32 ae_mask = hw_data->ae_mask;

	memset(&req, 0, sizeof(req));
	req.cmd_id = ICP_QAT_FW_HEARTBEAT_TIMER_SET;

	if (!accel_dev->admin) {
		dev_err(&GET_DEV(accel_dev), "adf_admin not available\n");
		return -EFAULT;
	}

	req.init_cfg_ptr = accel_dev->admin->phy_hb_addr;
	req.heartbeat_ticks = MAX_HB_TICKS;

	if (adf_send_admin(accel_dev, &req, &resp, ae_mask)) {
		dev_err(&GET_DEV(accel_dev),
			"Error changing Heartbeat timer\n");
		return -EFAULT;
	}

	return 0;
}

static int adf_disable_arbiter(struct adf_accel_dev *accel_dev,
			       u32 ae,
			       u32 thr)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	void __iomem *csr = accel_dev->transport->banks[0].csr_addr;
	const u32 *thd_2_arb_cfg;
	u32 ae_thr_map;

	if (ADF_AE_STRAND0_THREAD == thr || ADF_AE_STRAND1_THREAD == thr)
		thr = ADF_AE_ADMIN_THREAD;

	hw_data->get_arb_mapping(accel_dev, &thd_2_arb_cfg);
	if (!thd_2_arb_cfg)
		return -EFAULT;

	/* Disable scheduling for this particular AE and thread */
	ae_thr_map = *(thd_2_arb_cfg + ae);
	ae_thr_map &= ~(0x0F << (thr * 4));

	adf_write_csr_arb_wrk_2_ser_map(csr, ae, ae_thr_map);

	return 0;
}

static int adf_set_hb_counters_fail(struct adf_accel_dev *accel_dev,
				    u32 ae,
				    u32 thr)
{
	struct icp_qat_fw_init_admin_hb_stats *stats =
			(struct icp_qat_fw_init_admin_hb_stats *)
					accel_dev->admin->virt_hb_addr;
	struct adf_hw_device_data *hw_device = accel_dev->hw_device;
	const size_t max_aes = hw_device->get_num_aes(hw_device);
	u16 num_rsp = stats[ae].stats[thr].resp_heartbeat_cnt;

	/* Inject live.req != live.rsp and live.rsp == last.rsp
	 * to trigger the heartbeat error detection
	 */
	stats[ae].stats[thr].req_heartbeat_cnt++;
	stats += max_aes;
	stats[ae].stats[thr].resp_heartbeat_cnt = num_rsp;

	return 0;
}

int adf_heartbeat_simulate_failure(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_device = accel_dev->hw_device;
	const size_t max_aes = hw_device->get_num_aes(hw_device);
	u32 rand, rand_ae, rand_thr;

	get_random_bytes(&rand, sizeof(rand));
	rand_ae = rand % max_aes;
	get_random_bytes(&rand, sizeof(rand));
	rand_thr = rand % ADF_NUM_HB_CNT_PER_AE;

	dev_info(&GET_DEV(accel_dev),
		 "adf_heartbeat_simulate_failure for random AE %u, thr %u\n",
		 rand_ae, rand_thr);

	/* Increase the heartbeat timer to prevent FW updating HB counters */
	if (adf_set_max_hb_timer(accel_dev))
		return -EFAULT;

	/* Change arbiter to stop processing any packet */
	if (adf_disable_arbiter(accel_dev, rand_ae, rand_thr))
		return -EFAULT;

	/* Change HB counters memory to simulate a hang */
	if (adf_set_hb_counters_fail(accel_dev, rand_ae, rand_thr))
		return -EFAULT;

	return 0;
}

#endif
