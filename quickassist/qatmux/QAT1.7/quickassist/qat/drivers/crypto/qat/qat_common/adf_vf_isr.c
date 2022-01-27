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
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include "adf_accel_devices.h"
#include "adf_common_drv.h"
#include "adf_cfg.h"
#include "adf_cfg_strings.h"
#include "adf_cfg_common.h"
#include "adf_transport_access_macros.h"
#include "adf_transport_internal.h"
#include "adf_pf2vf_msg.h"

#define ADF_VINTSOU_OFFSET	0x204
#define ADF_VINTMSK_OFFSET	0x208
#define ADF_VINTSOU_BUN		BIT(0)
#define ADF_VINTSOU_PF2VF	BIT(1)

static struct workqueue_struct *adf_vf_stop_wq;
static DEFINE_MUTEX(vf_stop_wq_lock);

struct adf_vf_stop_data {
	struct adf_accel_dev *accel_dev;
	struct work_struct vf_stop_work;
};

static int adf_enable_msi(struct adf_accel_dev *accel_dev)
{
	struct adf_accel_pci *pci_dev_info = &accel_dev->accel_pci_dev;
	int stat = pci_enable_msi(pci_dev_info->pci_dev);

	if (stat) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to enable MSI interrupts\n");
		return stat;
	}

	accel_dev->vf.irq_name = kzalloc(ADF_MAX_MSIX_VECTOR_NAME, GFP_KERNEL);
	if (!accel_dev->vf.irq_name)
		return -ENOMEM;

	return stat;
}

static void adf_disable_msi(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_to_pci_dev(accel_dev);

	kfree(accel_dev->vf.irq_name);
	pci_disable_msi(pdev);
}

static void adf_dev_stop_async(struct work_struct *work)
{
	struct adf_vf_stop_data *stop_data =
		container_of(work, struct adf_vf_stop_data, vf_stop_work);
	struct adf_accel_dev *accel_dev = stop_data->accel_dev;

	adf_dev_restarting_notify(accel_dev);
	adf_dev_stop(accel_dev);
	adf_dev_shutdown(accel_dev);

	/* Re-enable PF2VF interrupts */
	adf_enable_pf2vf_interrupts(accel_dev);
	kfree(stop_data);
}

static void adf_pf2vf_bh_handler(void *data)
{
	struct adf_accel_dev *accel_dev = data;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct adf_bar *pmisc =
			&GET_BARS(accel_dev)[hw_data->get_misc_bar_id(hw_data)];
	void __iomem *pmisc_bar_addr = pmisc->virt_addr;
	u32 msg;
	bool is_notification = false;

	/* Read the message from PF */
	msg = ADF_CSR_RD(pmisc_bar_addr, hw_data->get_pf2vf_offset(0));
	if (!(msg & ADF_PF2VF_INT)) {
		dev_err(&GET_DEV(accel_dev),
			"Spurious PF2VF interrupt. msg %X. Ignored\n", msg);
		accel_dev->vf.pfvf_counters.spurious++;
		goto out;
	}
	accel_dev->vf.pfvf_counters.rx++;

	if (!(msg & ADF_PF2VF_MSGORIGIN_SYSTEM)) {
		dev_err(&GET_DEV(accel_dev),
			"Ignore non-system PF2VF message(0x%x)\n", msg);
		/*
		 * To ack, clear the VF2PFINT bit.
		 * Because this must be a legacy message, the far side
		 * must clear the in-use pattern.
		 */
		msg &= ~ADF_PF2VF_INT;
		ADF_CSR_WR(pmisc_bar_addr, hw_data->get_pf2vf_offset(0), msg);
		goto out;
	}

	switch ((msg & ADF_PF2VF_MSGTYPE_MASK) >> ADF_PF2VF_MSGTYPE_SHIFT) {
	case ADF_PF2VF_MSGTYPE_RESTARTING: {
		struct adf_vf_stop_data *stop_data;
		is_notification = true;

		dev_dbg(&GET_DEV(accel_dev),
			"Restarting msg received from PF 0x%x\n", msg);

		clear_bit(ADF_STATUS_PF_RUNNING, &accel_dev->status);

		stop_data = kzalloc(sizeof(*stop_data), GFP_ATOMIC);
		if (!stop_data) {
			dev_err(&GET_DEV(accel_dev),
				"Couldn't schedule stop for vf_%d\n",
				accel_dev->accel_id);
			goto out;
		}
		stop_data->accel_dev = accel_dev;
		INIT_WORK(&stop_data->vf_stop_work, adf_dev_stop_async);
		queue_work(adf_vf_stop_wq, &stop_data->vf_stop_work);
		break;
	}
	case ADF_PF2VF_MSGTYPE_VERSION_RESP:
		dev_dbg(&GET_DEV(accel_dev),
			"Version resp received from PF 0x%x\n", msg);
		is_notification = false;
		accel_dev->vf.pf_version =
			(msg & ADF_PF2VF_VERSION_RESP_VERS_MASK) >>
			ADF_PF2VF_VERSION_RESP_VERS_SHIFT;
		accel_dev->vf.compatible =
			(msg & ADF_PF2VF_VERSION_RESP_RESULT_MASK) >>
			ADF_PF2VF_VERSION_RESP_RESULT_SHIFT;
		complete(&accel_dev->vf.iov_msg_completion);
		break;
	case ADF_PF2VF_MSGTYPE_BLOCK_RESP:
		is_notification = false;
		accel_dev->vf.pf2vf_block_byte =
			(msg & ADF_PF2VF_BLOCK_RESP_DATA_MASK) >>
			ADF_PF2VF_BLOCK_RESP_DATA_SHIFT;
		accel_dev->vf.pf2vf_block_resp_type =
			(msg & ADF_PF2VF_BLOCK_RESP_TYPE_MASK) >>
			ADF_PF2VF_BLOCK_RESP_TYPE_SHIFT;
		complete(&accel_dev->vf.iov_msg_completion);
		break;
	case ADF_PF2VF_MSGTYPE_FATAL_ERROR:
		dev_err(&GET_DEV(accel_dev),
			"Fatal error received from PF 0x%x\n", msg);
		is_notification = true;

		if (adf_notify_fatal_error(accel_dev))
			dev_err(&GET_DEV(accel_dev),
				"Couldn't notify fatal error\n");
		break;
	default:
		dev_err(&GET_DEV(accel_dev),
			"Unknown PF2VF message(0x%x)\n", msg);
	}

	/* To ack, clear the PF2VFINT bit */
	msg &= ~ADF_PF2VF_INT;
	/*
	 * Clear the in-use pattern if the sender won't do it.
	 * Because the compatibility version must be the first message
	 * exchanged between the VF and PF, the pf.version must be
	 * set at this time.
	 * The in-use pattern is not cleared for notifications so that
	 * it can be used for collision detection.
	 */
	if (accel_dev->vf.pf_version >= ADF_PFVF_COMPATIBILITY_FAST_ACK &&
	    !is_notification)
		msg &= ~ADF_PF2VF_IN_USE_BY_PF_MASK;
	ADF_CSR_WR(pmisc_bar_addr, hw_data->get_pf2vf_offset(0), msg);

out:
	/* Re-enable PF2VF interrupts */
	adf_enable_pf2vf_interrupts(accel_dev);
	return;
}

static int adf_setup_pf2vf_bh(struct adf_accel_dev *accel_dev)
{
	tasklet_init(&accel_dev->vf.pf2vf_bh_tasklet,
		     (void *)adf_pf2vf_bh_handler, (unsigned long)accel_dev);

	mutex_init(&accel_dev->vf.vf2pf_lock);
	return 0;
}

static void adf_cleanup_pf2vf_bh(struct adf_accel_dev *accel_dev)
{
	tasklet_disable(&accel_dev->vf.pf2vf_bh_tasklet);
	tasklet_kill(&accel_dev->vf.pf2vf_bh_tasklet);
	mutex_destroy(&accel_dev->vf.vf2pf_lock);
}

static irqreturn_t adf_isr(int irq, void *privdata)
{
	struct adf_accel_dev *accel_dev = privdata;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct adf_bar *pmisc =
			&GET_BARS(accel_dev)[hw_data->get_misc_bar_id(hw_data)];
	void __iomem *pmisc_bar_addr = pmisc->virt_addr;
	u32 v_int, v_mask;
	int handled = 0;

	/* Read VF INT source CSR to determine the source of VF interrupt */
	v_int = ADF_CSR_RD(pmisc_bar_addr, ADF_VINTSOU_OFFSET);
	v_mask = ADF_CSR_RD(pmisc_bar_addr, ADF_VINTMSK_OFFSET);

	/* Check for PF2VF interrupt */
	if ((v_int & ~v_mask) & ADF_VINTSOU_PF2VF) {
		/* Disable PF to VF interrupt */
		adf_disable_pf2vf_interrupts(accel_dev);

		/* Schedule tasklet to handle interrupt BH */
		tasklet_hi_schedule(&accel_dev->vf.pf2vf_bh_tasklet);
		handled = 1;
	}

	/* We only need to handle the interrupt in case this is a kernel bundle.
	 * If it is a user bundle, the UIO resp handler will handle the IRQ
	 */
	if (accel_dev->num_ker_bundles > 0 &&
	    (v_int & ~v_mask) & ADF_VINTSOU_BUN) {
		struct adf_etr_data *etr_data = accel_dev->transport;
		struct adf_etr_bank_data *bank = &etr_data->banks[0];

		/* Disable Flag and Coalesce Ring Interrupts */
		WRITE_CSR_INT_FLAG_AND_COL(bank->csr_addr, bank->bank_number,
					   0);
		tasklet_hi_schedule(&bank->resp_handler);
		handled = 1;
	}

	if (handled)
		return IRQ_HANDLED;
	return IRQ_NONE;
}

static int adf_request_msi_irq(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_to_pci_dev(accel_dev);
	unsigned int cpu;
	int ret;
	unsigned int irq_flags = 0;
	char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
	unsigned long num_ker_bundles = accel_dev->hw_device->num_banks;

	if (adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				    ADF_FIRST_USER_BUNDLE, val) == 0) {
		if (kstrtoul(val, 10, &num_ker_bundles))
			return -1;

	}
	accel_dev->num_ker_bundles = num_ker_bundles;

	snprintf(accel_dev->vf.irq_name, ADF_MAX_MSIX_VECTOR_NAME,
		 "qat_%02x:%02d.%02d", pdev->bus->number, PCI_SLOT(pdev->devfn),
		 PCI_FUNC(pdev->devfn));
	/* We need to share the interrupt with the UIO device in case this is
	 * a user bundle
	 */
	if (!num_ker_bundles)
		irq_flags = IRQF_SHARED | IRQF_ONESHOT;
	ret = request_irq(pdev->irq, adf_isr, irq_flags, accel_dev->vf.irq_name,
			  (void *)accel_dev);
	if (ret) {
		dev_err(&GET_DEV(accel_dev), "failed to enable irq for %s\n",
			accel_dev->vf.irq_name);
		return ret;
	}
	cpu = accel_dev->accel_id % num_online_cpus();
	irq_set_affinity_hint(pdev->irq, get_cpu_mask(cpu));
	accel_dev->vf.irq_enabled = true;

	return ret;
}

static int adf_setup_bh(struct adf_accel_dev *accel_dev)
{
	struct adf_etr_data *priv_data = accel_dev->transport;

	tasklet_init(&priv_data->banks[0].resp_handler, adf_response_handler,
		     (unsigned long)priv_data->banks);
	return 0;
}

static void adf_cleanup_bh(struct adf_accel_dev *accel_dev)
{
	struct adf_etr_data *priv_data = accel_dev->transport;

	tasklet_disable(&priv_data->banks[0].resp_handler);
	tasklet_kill(&priv_data->banks[0].resp_handler);
}

/**
 * adf_vf_isr_resource_free() - Free IRQ for acceleration device
 * @accel_dev:  Pointer to acceleration device.
 *
 * Function frees interrupts for acceleration device virtual function.
 */
void adf_vf_isr_resource_free(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_to_pci_dev(accel_dev);

	if (accel_dev->vf.irq_enabled) {
		irq_set_affinity_hint(pdev->irq, NULL);
		free_irq(pdev->irq, (void *)accel_dev);
	}

	adf_cleanup_bh(accel_dev);
	adf_cleanup_pf2vf_bh(accel_dev);
	adf_disable_msi(accel_dev);
}
EXPORT_SYMBOL_GPL(adf_vf_isr_resource_free);

/**
 * adf_vf_isr_resource_alloc() - Allocate IRQ for acceleration device
 * @accel_dev:  Pointer to acceleration device.
 *
 * Function allocates interrupts for acceleration device virtual function.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_vf_isr_resource_alloc(struct adf_accel_dev *accel_dev)
{
	if (adf_enable_msi(accel_dev))
		goto err_out;

	if (adf_setup_pf2vf_bh(accel_dev))
		goto err_out;

	if (adf_setup_bh(accel_dev))
		goto err_out;

	if (adf_request_msi_irq(accel_dev))
		goto err_out;

	return 0;
err_out:
	adf_vf_isr_resource_free(accel_dev);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(adf_vf_isr_resource_alloc);

/**
 * adf_flush_vf_wq() - Flush workqueue for VF
 *
 * Function flushes workqueue 'adf_vf_stop_wq' for VF.
 *
 * Return: void.
 */
void adf_flush_vf_wq(void)
{
	if (adf_vf_stop_wq)
		flush_workqueue(adf_vf_stop_wq);
}
EXPORT_SYMBOL_GPL(adf_flush_vf_wq);

/**
 * adf_init_vf_wq() - Init workqueue for VF
 *
 * Function init workqueue 'adf_vf_stop_wq' for VF.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_init_vf_wq(void)
{
	int ret = 0;

	mutex_lock(&vf_stop_wq_lock);
	if (!adf_vf_stop_wq)
		adf_vf_stop_wq = alloc_workqueue("adf_vf_stop_wq",
						 WQ_MEM_RECLAIM, 0);

	if (!adf_vf_stop_wq)
		ret = -ENOMEM;

	mutex_unlock(&vf_stop_wq_lock);
	return ret;
}

/**
 * adf_exit_vf_wq() - Destroy workqueue for VF
 *
 * Function destroy workqueue 'adf_vf_stop_wq' for VF.
 *
 * Return: void.
 */
void adf_exit_vf_wq(void)
{
	if (adf_vf_stop_wq) {
		destroy_workqueue(adf_vf_stop_wq);
		adf_vf_stop_wq = NULL;
	}
}
