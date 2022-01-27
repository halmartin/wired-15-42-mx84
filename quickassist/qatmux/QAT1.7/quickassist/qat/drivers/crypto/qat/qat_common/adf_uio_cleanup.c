/*
  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY
  Copyright(c) 2015 Intel Corporation.
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
  Copyright(c) 2015 Intel Corporation.
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

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/uio_driver.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/sched.h>
#if (KERNEL_VERSION(4, 16, 0) <= LINUX_VERSION_CODE)
#include <linux/dma-direct.h>
#endif
#include "adf_uio.h"
#include "adf_uio_control.h"
#include "adf_accel_devices.h"
#include "adf_transport_access_macros.h"
#include "adf_common_drv.h"
#include "adf_transport_internal.h"
#include "icp_qat_fw.h"
#include "icp_qat_fw_pke.h"
#include "adf_uio_cleanup.h"

#define     MSG_NULL_FLAGS   0xdeadbeefdeadbeefULL

#define     CLEANUP_PH1_LIMIT    500   /* ms */
#define     CLEANUP_PH1_INTERVAL 5     /* ms */
#define     CLEANUP_PH2_LIMIT    5000  /* ms */
#define     CLEANUP_PH2_INTERVAL 250   /* ms */

#define     ADF_RING_CONFIG_RING_SIZE 0x05

struct ring_base {
	void *base_virt;
	dma_addr_t  addr;
	uint32_t size;
};

struct bundle_orphan_ring {
	unsigned long  tx_mask;
	unsigned long  rx_mask;
	unsigned long  asym_mask;
	unsigned long  rcv_null;
	void __iomem   *csr_base;
	phys_addr_t    addr;
	struct ring_base *base;
	char bank_number;
};

struct timer_state_s {
	int phase;
	int interval;
	int limit;
	int elapsed;
};

static inline uint32_t get_ring_size(void __iomem *csr_base_addr,
				     int ring)
{
	uint32_t config, ring_size;

	config = ADF_CSR_RD(csr_base_addr,
			    ADF_RING_CSR_RING_CONFIG + (ring << 2));
	ring_size = config & ((1 << ADF_RING_CONFIG_RING_SIZE) - 1);

	return ADF_SIZE_TO_RING_SIZE_IN_BYTES(ring_size);
}

static void build_null_msg(struct icp_qat_fw_comn_req *req)
{
	memset(req, 0, sizeof(*req));
	ICP_QAT_FW_COMN_OV_SRV_TYPE_SET(req->comn_hdr,
					ICP_QAT_FW_COMN_REQ_NULL);
	ICP_QAT_FW_COMN_VALID_FLAG_SET(req->comn_hdr,
				       ICP_QAT_FW_COMN_REQ_FLAG_SET);
	req->comn_mid.opaque_data = MSG_NULL_FLAGS;
}

static int is_null_rsp(struct icp_qat_fw_comn_resp  *rsp)
{
	return ((ICP_QAT_FW_COMN_HDR_VALID_FLAG_GET(rsp->comn_hdr)) &&
		(ICP_QAT_FW_COMN_RESP_SERV_NULL ==
			rsp->comn_hdr.response_type) &&
		(MSG_NULL_FLAGS == rsp->opaque_data));
}

/*  make sure that  head tail are already init to zero  */
/*  just set the msg size to 128, because our empty msg is the last msg  */
static void send_null_msg(struct bundle_orphan_ring *orphan_bundle,
			  const u8 num_rings_per_bank)
{
	int i;
	u8 req_size = 0;
	struct ring_base *r_base;
	struct icp_qat_fw_comn_req null_msg;

	build_null_msg(&null_msg);
	for (i = 0; i < num_rings_per_bank; i++) {
		if (!test_bit(i, &orphan_bundle->tx_mask))
			continue;

		if (test_bit(i, &orphan_bundle->asym_mask))
			req_size = sizeof(struct icp_qat_fw_pke_request);
		else
			req_size = sizeof(null_msg);

		r_base = orphan_bundle->base + i;
		memcpy(r_base->base_virt, &null_msg, sizeof(null_msg));
		WRITE_CSR_RING_TAIL(orphan_bundle->csr_base, 0, i,
				    req_size);
	}
}

static int find_null_response(void *start, void *end)
{
	struct icp_qat_fw_comn_resp *pos = start;

	while ((void *)(pos) < end) {
		if ((void *)(pos + 1) > end)
			break;

		if (is_null_rsp(pos))
			return 1;
		pos++;
	}

	return 0;
}

static int rcv_null_response(struct ring_base *r_base,
			     uint32_t head,
			     uint32_t tail)
{
	void *start = r_base->base_virt + head;
	void *end = r_base->base_virt + tail;
	int ret;

	if (head < tail)
		return find_null_response(start, end);
	ret = find_null_response(start, r_base->base_virt + r_base->size);

	if (ret)
		return ret;
	return find_null_response(r_base->base_virt, end);
}

static int timer_sm(struct timer_state_s *timer_state)
{
	/* state machine for timer's phases
	 * initial phase on first loop iteration
	 */
	if (timer_state->phase <= 0) {
		timer_state->phase = 1;
		timer_state->interval = CLEANUP_PH1_INTERVAL;
		timer_state->limit = CLEANUP_PH1_LIMIT;
	}

	if (timer_state->elapsed >= timer_state->limit) {
		if (timer_state->phase == 1) {
			timer_state->phase = 2;
			timer_state->interval = CLEANUP_PH2_INTERVAL;
			timer_state->limit = CLEANUP_PH2_LIMIT;
		} else {
			/* time elapsed */
			return 1;
		}
	}

	return 0;
}

/* before this, we must empty response ring,
 * set head == tail,  before send null msg  */
static void wait_for_null_msg(struct bundle_orphan_ring *orphan_bundle,
			      const u8 num_rings_per_bank)
{
	int i;

	struct timer_state_s timer_state = {
		.phase = 0,
		.interval = 0,
		.limit = 0,
		.elapsed = 0,
	};

	struct ring_base *r_base;
	uint32_t head;
	uint32_t tail;

	while (1) {
		for (i = 0; i < num_rings_per_bank; i++) {
			if (!test_bit(i, &orphan_bundle->rx_mask) ||
			    test_bit(i, &orphan_bundle->rcv_null))
				continue;

			r_base = orphan_bundle->base + i;
			head = READ_CSR_RING_HEAD(orphan_bundle->csr_base,
						  0, i);
			tail = READ_CSR_RING_TAIL(orphan_bundle->csr_base,
						  0, i);

			if (rcv_null_response(r_base, head, tail))
				__set_bit(i, &orphan_bundle->rcv_null);

			WRITE_CSR_RING_HEAD(orphan_bundle->csr_base,
					    0, i, tail);
		}

		/* check received all null message */
		if (orphan_bundle->rcv_null == orphan_bundle->rx_mask) {
			pr_debug("QAT: orphan ring handler - null message received correctly\n");
			break;
		}

		/* check timer */
		if (timer_sm(&timer_state) != 0) {
			pr_err("QAT: failed to receive response message in %d",
			       timer_state.elapsed);
			break;
		}

		/* sleep before next check */
		msleep(timer_state.interval);
		timer_state.elapsed += timer_state.interval;
	}

	/*  when run here,  we received all empty response,
	 *  so all inflight is finished */
}

/*  get virt address from dma address */
static void *ring_dma_to_virt(struct device *dev, dma_addr_t daddr)
{
	phys_addr_t paddr = dma_to_phys(dev, daddr << 6);

	return phys_to_virt(paddr);
}

/*
*    if orphan->tx_mask does not match with orphan->rx_mask
*/
static void check_orphan_ring(struct bundle_orphan_ring *orphan,
			      struct adf_hw_device_data *hw_data)
{
	int i;
	int tx_rx_gap = hw_data->tx_rx_gap;
	u8 num_rings_per_bank = hw_data->num_rings_per_bank;
	void __iomem *csr_base = orphan->csr_base;

	for (i = 0; i < num_rings_per_bank; i++) {
		if (test_bit(i, &orphan->tx_mask)) {
			int rx_ring = i + tx_rx_gap;

			if (!test_bit(rx_ring, &orphan->rx_mask)) {
				__clear_bit(i, &orphan->tx_mask);

				/* clean up this tx ring  */
				WRITE_CSR_RING_CONFIG(csr_base, 0, i, 0);
				WRITE_CSR_RING_BASE(csr_base, 0, i, 0);
			}

		} else if (test_bit(i, &orphan->rx_mask)) {
			int tx_ring = i - tx_rx_gap;

			if (!test_bit(tx_ring, &orphan->tx_mask)) {
				__clear_bit(i, &orphan->rx_mask);

				/* clean up this rx ring */
				WRITE_CSR_RING_CONFIG(csr_base, 0, i, 0);
				WRITE_CSR_RING_BASE(csr_base, 0, i, 0);
			}
		}
	}
}

static int get_orphan_bundle(struct uio_info *info,
			     struct adf_uio_control_accel *accel,
			     struct bundle_orphan_ring **orphan)
{
	int i;
	int ret = 0;
	void __iomem *csr_base;
	unsigned long tx_mask;
	unsigned long asym_mask;
	struct adf_accel_dev *accel_dev = accel->accel_dev;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	u8 num_rings_per_bank = hw_data->num_rings_per_bank;
	struct bundle_orphan_ring *orphan_bundle;
	struct ring_base *r_base;
	uint64_t base;
	struct list_head *entry;
	struct qat_uio_bundle_dev *priv = info->priv;
	struct adf_uio_control_bundle *bundle = priv->bundle;
	struct adf_uio_instance_rings *instance_rings;
	uint16_t ring_mask = 0;

	orphan_bundle = kzalloc(sizeof(*orphan_bundle), GFP_KERNEL);
	if (!orphan_bundle)
		return -ENOMEM;

	orphan_bundle->base = kzalloc(sizeof(*orphan_bundle->base) *
				      num_rings_per_bank,
				      GFP_KERNEL);
	if (!orphan_bundle->base) {
		ret = -ENOMEM;
		goto base_failed;
	}

	csr_base = info->mem[0].internal_addr;
	orphan_bundle->csr_base = csr_base;
	orphan_bundle->addr = info->mem[0].addr;

	orphan_bundle->tx_mask = 0;
	orphan_bundle->rx_mask = 0;
	orphan_bundle->bank_number = bundle->hardware_bundle_number;
	tx_mask = accel_dev->hw_device->tx_rings_mask;
	asym_mask = accel_dev->hw_device->asym_rings_mask;

	/* Get ring mask for this process. */
	mutex_lock(&bundle->list_lock);
	list_for_each(entry, &bundle->list) {
		instance_rings = list_entry(entry,
					    struct adf_uio_instance_rings,
					    list);
		if (instance_rings->user_pid == current->tgid) {
			ring_mask = instance_rings->ring_mask;
			break;
		}
	}
	mutex_unlock(&bundle->list_lock);

	for (i = 0; i < num_rings_per_bank; i++) {
		base = READ_CSR_RING_BASE(csr_base, 0, i);

		if (!base)
			continue;
		if (!(ring_mask & 1 << i))
			continue; /* Not reserved for this process. */

		r_base = orphan_bundle->base + i;
		r_base->addr = base;
		r_base->size = get_ring_size(csr_base, i);
		r_base->base_virt = ring_dma_to_virt(&GET_DEV(accel_dev),
						     r_base->addr);
		if (!r_base->base_virt) {
			dev_err(&GET_DEV(accel_dev),
				"Failed to get ring %d base virtual address\n",
				i);
			ret = -ENOMEM;
			goto map_failed;
		}

		if (test_bit(i, &tx_mask))
			__set_bit(i, &orphan_bundle->tx_mask);
		else
			__set_bit(i, &orphan_bundle->rx_mask);

		if (test_bit(i, &asym_mask))
			__set_bit(i, &orphan_bundle->asym_mask);
	}

	if (orphan_bundle->tx_mask || orphan_bundle->rx_mask)
		check_orphan_ring(orphan_bundle, hw_data);

	*orphan = orphan_bundle;
	return ret;

map_failed:
	kfree(orphan_bundle->base);

base_failed:
	kfree(orphan_bundle);
	return ret;
}

static void put_orphan_bundle(struct bundle_orphan_ring *bundle)
{
	if (!bundle)
		return;

	kfree(bundle->base);
	bundle->base = NULL;
	kfree(bundle);
}

/* cleanup all ring  */
static void cleanup_all_ring(struct adf_uio_control_accel *accel,
			     struct bundle_orphan_ring *orphan)
{
	int i;
	void __iomem *csr_base = orphan->csr_base;
	unsigned long  mask = orphan->rx_mask | orphan->tx_mask;
	struct adf_accel_dev *accel_dev = accel->accel_dev;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	u8 num_rings_per_bank = hw_data->num_rings_per_bank;

	for (i = 0; i < num_rings_per_bank; i++) {
		if (!test_bit(i, &mask))
			continue;

		WRITE_CSR_RING_CONFIG(csr_base, 0, i, 0);
		WRITE_CSR_RING_BASE(csr_base, 0, i, 0);
	}

	adf_disable_ring_arb(orphan->csr_base, orphan->tx_mask);
}

/* must disable tx ring first  */
/* rewrite the base */
static void reset_tx_rings(struct bundle_orphan_ring *orphan,
			   const u8 num_rings_per_bank)
{
	int i;
	void __iomem *csr_base = orphan->csr_base;
	struct ring_base *base = NULL;

	for (i = 0; i < num_rings_per_bank; i++) {
		if (!test_bit(i, &orphan->tx_mask))
			continue;

		base = orphan->base + i;
		WRITE_CSR_RING_BASE(csr_base, 0, i, base->addr);
	}
}

/*  empty the rx rings, set the tail value into head
 *  consumer can set head value. */
static void empty_rx_rings(struct bundle_orphan_ring *bundle,
			   const u8 num_rings_per_bank)
{
	int i;
	unsigned int tail;

	for (i = 0; i < num_rings_per_bank; i++) {
		if (!test_bit(i, &bundle->rx_mask))
			continue;

		tail = READ_CSR_RING_TAIL(bundle->csr_base, 0, i);
		WRITE_CSR_RING_HEAD(bundle->csr_base, 0, i, tail);
	}
}

static int bundle_need_cleanup(struct uio_info *info,
			       const u8 num_rings_per_bank)
{
	int i;
	void __iomem *csr_base = info->mem[0].internal_addr;

	if (!csr_base)
		return 0;

	for (i = 0; i < num_rings_per_bank; i++) {
		if (READ_CSR_RING_BASE(csr_base, 0, i))
			return 1;
	}

	return 0;
}

static void cleanup_orphan_ring(struct bundle_orphan_ring *orphan,
				struct adf_uio_control_accel *accel)
{
	struct adf_accel_dev *accel_dev = accel->accel_dev;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	u8 number_rings_per_bank = hw_data->num_rings_per_bank;

	/* disable the interrupt */
	WRITE_CSR_INT_COL_EN(orphan->csr_base, 0, 0);

	/*
	 * wait firmware finish the in-process ring
	 * 1. first disable all tx rings
	 * 2. reset tx rings: rewrite the base address
	 * 3. empty rx rings: set head == tail
	 * 4. enable all tx rings
	 * 5. send empty request
	 * 6. wait for the empty response
	 * send an empty request first and wait for the empty response
	 *  or firmware may hang
	 */
	adf_disable_ring_arb(orphan->csr_base, orphan->tx_mask);
	reset_tx_rings(orphan, number_rings_per_bank);
	adf_enable_ring_arb(orphan->csr_base, orphan->tx_mask);
	empty_rx_rings(orphan, number_rings_per_bank);
	send_null_msg(orphan, number_rings_per_bank);
	wait_for_null_msg(orphan, number_rings_per_bank);

	/*  when run into here, we think that
	 *  there is no inflight ring  and no in-process ring
	 *  */
	cleanup_all_ring(accel, orphan);
	pr_debug("QAT: orphan rings cleaned\n");
}

void adf_uio_do_cleanup_orphan(struct uio_info *info,
			       struct adf_uio_control_accel *accel)
{
	int ret;
	struct bundle_orphan_ring *orphan = NULL;
	struct adf_accel_dev *accel_dev = accel->accel_dev;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	u8 number_rings_per_bank = hw_data->num_rings_per_bank;
	struct qat_uio_bundle_dev *priv = info->priv;
	struct adf_uio_control_bundle *bundle = priv->bundle;
	struct adf_uio_instance_rings *instance_rings, *tmp;
	int pid_found = 0;

	if (!bundle_need_cleanup(info, number_rings_per_bank))
		goto release;

	ret = get_orphan_bundle(info, accel, &orphan);
	if (ret < 0) {
		dev_err(&GET_DEV(accel_dev),
			"get orphan ring failed to cleanup bundle\n");
		return;
	}

	if (!orphan->tx_mask && !orphan->rx_mask)
		goto out;

	dev_warn(&GET_DEV(accel_dev), "Process %d %s exit with orphan rings\n",
		 current->tgid, current->comm);
	/*
	 * If the device is in reset phase, we do not need to clean the ring
	 * here since we have disabled BME and will clean the ring in
	 * stop/shutdown stage.
	 */
	if (!test_bit(ADF_STATUS_RESTARTING, &accel_dev->status))
		cleanup_orphan_ring(orphan, accel);
out:
	put_orphan_bundle(orphan);
release:
	/*
	 * If the user process died without releasing the rings
	 * then force a release here.
	*/
	mutex_lock(&bundle->list_lock);
	list_for_each_entry_safe(instance_rings, tmp, &bundle->list, list) {
		if (instance_rings->user_pid == current->tgid) {
			pid_found = 1;
			break;
		}
	}
	mutex_unlock(&bundle->list_lock);

	if (pid_found) {
		mutex_lock(&bundle->lock);
		bundle->rings_used &= ~instance_rings->ring_mask;
		mutex_unlock(&bundle->lock);
	}
}
