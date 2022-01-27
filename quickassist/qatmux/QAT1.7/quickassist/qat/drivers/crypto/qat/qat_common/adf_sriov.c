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
#include <linux/workqueue.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/iommu.h>
#include "adf_common_drv.h"
#include "adf_cfg.h"
#include "adf_pf2vf_msg.h"

#define AE2FUNCTION_MAP_A_OFFSET	(0x3A400 + 0x190)
#define AE2FUNCTION_MAP_B_OFFSET	(0x3A400 + 0x310)
#define AE2FUNCTION_MAP_REG_SIZE	4
#define AE2FUNCTION_MAP_VALID		BIT(7)

#define READ_CSR_AE2FUNCTION_MAP_A(pmisc_bar_addr, index)		\
	ADF_CSR_RD(pmisc_bar_addr, AE2FUNCTION_MAP_A_OFFSET +		\
		   AE2FUNCTION_MAP_REG_SIZE * (index))

#define WRITE_CSR_AE2FUNCTION_MAP_A(pmisc_bar_addr, index, value)	\
	ADF_CSR_WR(pmisc_bar_addr, AE2FUNCTION_MAP_A_OFFSET +		\
		   AE2FUNCTION_MAP_REG_SIZE * (index), value)

#define READ_CSR_AE2FUNCTION_MAP_B(pmisc_bar_addr, index)		\
	ADF_CSR_RD(pmisc_bar_addr, AE2FUNCTION_MAP_B_OFFSET +		\
		   AE2FUNCTION_MAP_REG_SIZE * (index))

#define WRITE_CSR_AE2FUNCTION_MAP_B(pmisc_bar_addr, index, value)	\
	ADF_CSR_WR(pmisc_bar_addr, AE2FUNCTION_MAP_B_OFFSET +		\
		   AE2FUNCTION_MAP_REG_SIZE * (index), value)

static struct workqueue_struct *pf2vf_resp_wq;
static int pf2vf_resp_wq_reference;
static DEFINE_MUTEX(pf2vf_resp_wq_ref_lock);

struct adf_pf2vf_resp {
	struct work_struct pf2vf_resp_work;
	struct adf_accel_vf_info *vf_info;
};

void adf_configure_iov_threads(struct adf_accel_dev *accel_dev, bool enable)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct adf_bar *pmisc =
			&GET_BARS(accel_dev)[hw_data->get_misc_bar_id(hw_data)];
	void __iomem *pmisc_addr = pmisc->virt_addr;
	u32 i, reg;

	/* Set/Unset Valid bits in AE Thread to PCIe Function Mapping */
	for (i = 0; i < ADF_NUM_THREADS_PER_AE * hw_data->num_engines; i++) {
		reg = READ_CSR_AE2FUNCTION_MAP_A(pmisc_addr, i);
		if (enable)
			reg |= AE2FUNCTION_MAP_VALID;
		else
			reg &= ~AE2FUNCTION_MAP_VALID;
		WRITE_CSR_AE2FUNCTION_MAP_A(pmisc_addr, i, reg);
	}

	for (i = 0; i < hw_data->num_engines; i++) {
		reg = READ_CSR_AE2FUNCTION_MAP_B(pmisc_addr, i);
		if (enable)
			reg |= AE2FUNCTION_MAP_VALID;
		else
			reg &= ~AE2FUNCTION_MAP_VALID;
		WRITE_CSR_AE2FUNCTION_MAP_B(pmisc_addr, i, reg);
	}
}
EXPORT_SYMBOL_GPL(adf_configure_iov_threads);

static void adf_iov_send_resp(struct work_struct *work)
{
	struct adf_pf2vf_resp *pf2vf_resp =
		container_of(work, struct adf_pf2vf_resp, pf2vf_resp_work);

	adf_vf2pf_req_hndl(pf2vf_resp->vf_info);
	kfree(pf2vf_resp);
}

void adf_vf2pf_handler(struct adf_accel_vf_info *vf_info)
{
	struct adf_pf2vf_resp *pf2vf_resp;

	pf2vf_resp = kzalloc(sizeof(*pf2vf_resp), GFP_ATOMIC);
	if (!pf2vf_resp)
		return;

	pf2vf_resp->vf_info = vf_info;
	INIT_WORK(&pf2vf_resp->pf2vf_resp_work, adf_iov_send_resp);
	queue_work(pf2vf_resp_wq, &pf2vf_resp->pf2vf_resp_work);
}

static int adf_enable_sriov(struct adf_accel_dev *accel_dev)
{
	struct pci_dev *pdev = accel_to_pci_dev(accel_dev);
	int totalvfs = pci_sriov_get_totalvfs(pdev);
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	int bar_id = hw_data->get_misc_bar_id(hw_data);
	struct adf_bar *pmisc = &GET_BARS(accel_dev)[bar_id];
	void __iomem *pmisc_addr = pmisc->virt_addr;

	struct adf_accel_vf_info *vf_info;
	int i = 0;
	int ret = 0;

	/* init workqueue in PF */
	ret = adf_init_pf_wq();
	if (ret) {
		return ret;
	}

	for (i = 0, vf_info = accel_dev->pf.vf_info; i < totalvfs;
	     i++, vf_info++) {
		/* This ptr will be populated when VFs will be created */
		vf_info->accel_dev = accel_dev;
		vf_info->vf_nr = i;
		vf_info->compat_ver = 0;

		mutex_init(&vf_info->pf2vf_lock);
		ratelimit_state_init(&vf_info->vf2pf_ratelimit,
				     ADF_IOV_RATELIMIT_INTERVAL,
				     ADF_IOV_RATELIMIT_BURST);
	}

	/* Set Valid bits in AE Thread to PCIe Function Mapping */
	hw_data->configure_iov_threads(accel_dev, true);

	/* Enable VF to PF interrupts for all VFs */
	for (i = 0; i < ADF_MAX_VF2PF_SET; i++)
		hw_data->enable_vf2pf_interrupts(pmisc_addr, 0xFFFFFFFF, i);

	adf_pfvf_debugfs_add(accel_dev);

	/*
	 * Due to the hardware design, when SR-IOV and the ring arbiter
	 * are enabled all the VFs supported in hardware must be enabled in
	 * order for all the hardware resources (i.e. bundles) to be usable.
	 * When SR-IOV is enabled, each of the VFs will own one bundle.
	 */
	return pci_enable_sriov(pdev, totalvfs);
}

/**
 * adf_disable_sriov() - Disable SRIOV for the device
 * @accel_dev:  Pointer to accel device.
 *
 * Function disables SRIOV for the accel device.
 *
 * Return: 0 on success, error code otherwise.
 */
void adf_disable_sriov(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	int bar_id = hw_data->get_misc_bar_id(hw_data);
	struct adf_bar *pmisc = &GET_BARS(accel_dev)[bar_id];
	void __iomem *pmisc_addr = pmisc->virt_addr;
	int totalvfs = pci_sriov_get_totalvfs(accel_to_pci_dev(accel_dev));
	struct adf_accel_vf_info *vf;
	int i;

	if (!accel_dev->pf.vf_info)
		return;

	adf_pf2vf_notify_restarting(accel_dev);

	pci_disable_sriov(accel_to_pci_dev(accel_dev));
	debugfs_remove_recursive(accel_dev->pfvf_dbgdir);
	accel_dev->pfvf_dbgdir = NULL;

	/* Disable VF to PF interrupts */
	for (i = 0; i < ADF_MAX_VF2PF_SET; i++)
		hw_data->disable_vf2pf_interrupts(pmisc_addr, 0xFFFFFFFF, i);

	/* Clear Valid bits in AE Thread to PCIe Function Mapping */
	hw_data->configure_iov_threads(accel_dev, false);

	for (i = 0, vf = accel_dev->pf.vf_info; i < totalvfs; i++, vf++)
		mutex_destroy(&vf->pf2vf_lock);

	/* destroy workqueue in PF */
	adf_exit_pf_wq();

	kfree(accel_dev->pf.vf_info);
	accel_dev->pf.vf_info = NULL;
}
EXPORT_SYMBOL_GPL(adf_disable_sriov);

static int adf_dev_prepare_restart(struct adf_accel_dev *accel_dev)
{
	int ret;
	struct adf_cfg_section sec = { {0} };

	ret = adf_cfg_save_section(accel_dev, ADF_GENERAL_SEC, &sec);
	if (ret)
		return ret;
	adf_dev_stop(accel_dev);
	adf_dev_shutdown(accel_dev);
	ret = adf_cfg_restore_section(accel_dev, &sec);
	adf_cfg_keyval_del_all(&sec.param_head);
	if (ret)
		return ret;
	set_bit(ADF_STATUS_SRIOV_RESTARTING, &accel_dev->status);

	return 0;
}

static int adf_dev_reinit(struct adf_accel_dev *accel_dev)
{
	int ret;

	ret = adf_dev_init(accel_dev);
	if (ret) {
		dev_err(&GET_DEV(accel_dev), "Failed to init qat_dev%d\n",
			accel_dev->accel_id);
		adf_dev_shutdown(accel_dev);
		return ret;
	}

	ret = adf_dev_start(accel_dev);
	if (ret) {
		dev_err(&GET_DEV(accel_dev), "Failed to start qat_dev%d\n",
			accel_dev->accel_id);
		adf_dev_stop(accel_dev);
		adf_dev_shutdown(accel_dev);
		return ret;
	}
	clear_bit(ADF_STATUS_SRIOV_RESTARTING, &accel_dev->status);

	return 0;
}

static int adf_config_cy(struct adf_accel_dev *accel_dev)
{
	const unsigned long val = 0;

	if (adf_cfg_section_add(accel_dev, ADF_KERNEL_SEC))
		return -EFAULT;

	if (adf_cfg_add_key_value_param(accel_dev, ADF_KERNEL_SEC,
					ADF_NUM_CY, (void *)&val, ADF_DEC))
		return -EFAULT;

	set_bit(ADF_STATUS_CONFIGURED, &accel_dev->status);

	return 0;
}

static int adf_sriov_enable(struct adf_accel_dev *accel_dev, const int totalvfs)
{
	int ret;

	if (!iommu_present(&pci_bus_type))
		dev_warn(&GET_DEV(accel_dev),
			 "IOMMU should be enabled for SR-IOV to work correctly\n");

	if (accel_dev->pf.vf_info) {
		dev_info(&GET_DEV(accel_dev),
			 "Already enabled for this device\n");
		return -EINVAL;
	}

	if (adf_dev_started(accel_dev)) {
		if (adf_devmgr_in_reset(accel_dev) ||
		    adf_dev_in_use(accel_dev)) {
			dev_err(&GET_DEV(accel_dev), "Device busy\n");
			return -EBUSY;
		}
		ret = adf_dev_prepare_restart(accel_dev);
		if (ret)
			return ret;
	}

	ret = adf_config_cy(accel_dev);
	if (ret)
		goto err_del_cfg;

	/* Allocate memory for VF info structs */
	ret = -ENOMEM;
	accel_dev->pf.vf_info = kcalloc(totalvfs,
					sizeof(struct adf_accel_vf_info),
					GFP_KERNEL);
	if (!accel_dev->pf.vf_info)
		goto err_del_cfg;

	ret = adf_dev_reinit(accel_dev);
	if (ret)
		goto err_free_vf;

	ret = adf_enable_sriov(accel_dev);
	if (ret)
		goto err_free_vf;

	return totalvfs;
err_free_vf:
	kfree(accel_dev->pf.vf_info);
	accel_dev->pf.vf_info = NULL;
err_del_cfg:
	adf_cfg_del_all(accel_dev);
	return ret;
}

static int adf_sriov_disable(struct adf_accel_dev *accel_dev)
{
	int ret;

	if (!accel_dev->pf.vf_info) {
		dev_info(&GET_DEV(accel_dev),
			 "Already disabled for this device\n");
		return -EINVAL;
	}

	if (adf_dev_started(accel_dev)) {
		if (adf_devmgr_in_reset(accel_dev)) {
			dev_err(&GET_DEV(accel_dev), "Device in reset\n");
			return -EBUSY;
		}

		ret = adf_dev_prepare_restart(accel_dev);
		if (ret)
			return ret;
	} else {
		adf_disable_sriov(accel_dev);
	}

	ret = qat_crypto_dev_config(accel_dev);
	if (ret)
		goto err_del_cfg;

	ret = adf_dev_reinit(accel_dev);
	if (ret)
		goto err_del_cfg;

	return 0;
err_del_cfg:
	adf_cfg_del_all(accel_dev);
	return ret;
}

/**
 * adf_sriov_configure() - Enable/disable SRIOV for the device
 * @pdev:  Pointer to pci device.
 *
 * Function enables or disables SRIOV for the pci device.
 *
 * Return: number of VFs enabled on success, error code otherwise.
 */
int adf_sriov_configure(struct pci_dev *pdev, int numvfs)
{
	struct adf_accel_dev *accel_dev = adf_devmgr_pci_to_accel_dev(pdev);
	const int totalvfs = pci_sriov_get_totalvfs(pdev);

	if (!accel_dev) {
		dev_err(&pdev->dev, "Failed to find accel_dev\n");
		return -EFAULT;
	}

	if (numvfs)
		return adf_sriov_enable(accel_dev, totalvfs);
	else
		return adf_sriov_disable(accel_dev);
}
EXPORT_SYMBOL_GPL(adf_sriov_configure);

/**
 * adf_init_pf_wq() - Init workqueue for PF
 * @accel_dev:  Pointer to accel device.
 *
 * Function init workqueue 'qat_pf2vf_resp_wq' for PF.
 *
 * Return: 0 on success, error code otherwise.
 */
int adf_init_pf_wq(void)
{
	/* Workqueue for PF2VF responses */
	mutex_lock(&pf2vf_resp_wq_ref_lock);
	if (!pf2vf_resp_wq)
		pf2vf_resp_wq = alloc_workqueue("qat_pf2vf_resp_wq",
						WQ_MEM_RECLAIM, 0);

	if (!pf2vf_resp_wq) {
		mutex_unlock(&pf2vf_resp_wq_ref_lock);
		return -ENOMEM;
	}

	pf2vf_resp_wq_reference++;
	mutex_unlock(&pf2vf_resp_wq_ref_lock);
	return 0;
}

/**
 * adf_exit_pf_wq() - Destroy workqueue for PF
 * @accel_dev:  Pointer to accel device.
 *
 * Function destroy workqueue 'qat_pf2vf_resp_wq' for PF.
 *
 * Return: void.
 */
void adf_exit_pf_wq(void)
{
	mutex_lock(&pf2vf_resp_wq_ref_lock);
	if (--pf2vf_resp_wq_reference == 0 && pf2vf_resp_wq) {
		destroy_workqueue(pf2vf_resp_wq);
		pf2vf_resp_wq = NULL;
	}
	mutex_unlock(&pf2vf_resp_wq_ref_lock);
}
