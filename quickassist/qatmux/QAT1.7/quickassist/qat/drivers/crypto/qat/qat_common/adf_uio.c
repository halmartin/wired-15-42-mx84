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
#include <linux/uio_driver.h>
#include <linux/sched.h>
#include <linux/kobject.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>

#include "adf_common_drv.h"
#include "adf_uio_control.h"
#include "adf_transport_access_macros.h"
#include "adf_uio_cleanup.h"
#include "adf_uio.h"
#include "adf_cfg.h"
#include "adf_cfg_user.h"
#include "qdm.h"
#include "adf_transport_internal.h"

#define ADF_UIO_NAME "UIO_%s_%02d_BUNDLE_%02d"
#define ADF_UIO_DEV_NAME "/dev/uio%i"
#define ADF_UIO_MAP_NAME "ADF_%s_ETR_BUNDLE_%02d"

#define ADF_UIO_GET_NAME(accel_dev) (GET_HW_DATA(accel_dev)->dev_class->name)
#define ADF_UIO_GET_TYPE(accel_dev) (GET_HW_DATA(accel_dev)->dev_class->type)
#define ADF_UIO_GET_BAR(accel_dev)  (GET_HW_DATA(accel_dev)->get_etr_bar_id(\
				     GET_HW_DATA(accel_dev)))

static struct service_hndl adf_uio_hndl;

static inline int adf_uio_get_minor(struct uio_info *info)
{
	struct uio_device *uio_dev = info->uio_dev;

	return uio_dev->minor;
}

/*
 * Structure defining the QAT UIO device private information
 */
struct qat_uio_pci_dev {
	uint8_t nb_bundles;
};

static inline
void adf_uio_init_bundle_ctrl(struct adf_uio_control_bundle *bundle)
{
	struct uio_info *info = &bundle->uio_info;
	int minor = adf_uio_get_minor(info);
	struct qat_uio_bundle_dev *priv = info->priv;

	snprintf(bundle->name, sizeof(bundle->name), ADF_UIO_DEV_NAME,
		 minor);
	bundle->hardware_bundle_number = priv->hardware_bundle_number;
	bundle->device_minor = minor;
	INIT_LIST_HEAD(&bundle->list);
	priv->bundle = bundle;
	mutex_init(&bundle->lock);
	mutex_init(&bundle->list_lock);
	bundle->csr_addr = info->mem[0].internal_addr;
}

static inline void adf_uio_init_accel_ctrl(struct adf_uio_control_accel *accel,
					   struct adf_accel_dev *accel_dev,
					   unsigned int nb_bundles)
{
	int i;

	accel->nb_bundles = nb_bundles;

	for (i = 0; i < nb_bundles; i++)
		adf_uio_init_bundle_ctrl(accel->bundle[i]);

	accel->first_minor = accel->bundle[0]->device_minor;
	accel->last_minor = accel->bundle[nb_bundles - 1]->device_minor;
}

static struct adf_uio_control_bundle *adf_ctl_ioctl_bundle(
		struct adf_user_reserve_ring reserve)
{
	struct adf_accel_dev *accel_dev;
	struct adf_uio_control_accel *accel;
	struct adf_uio_control_bundle *bundle;
	u8 num_rings_per_bank = 0;

	accel_dev = adf_devmgr_get_dev_by_id(reserve.accel_id);
	if (!accel_dev) {
		pr_err("QAT: Failed to get accel_dev\n");
		return NULL;
	}
	num_rings_per_bank = accel_dev->hw_device->num_rings_per_bank;

	accel = accel_dev->accel;
	if (!accel) {
		pr_err("QAT: Failed to get accel\n");
		return NULL;
	}

	if (reserve.bank_nr >= GET_MAX_BANKS(accel_dev)) {
		pr_err("QAT: Invalid bank bunber %d\n", reserve.bank_nr);
		return NULL;
	}
	if (reserve.ring_mask & ~((1 << num_rings_per_bank) - 1)) {
		pr_err("QAT: Invalid ring mask %0X\n", reserve.ring_mask);
		return NULL;
	}
	if (accel->num_ker_bundles > reserve.bank_nr) {
		pr_err("QAT: Invalid user reserved bank\n");
		return NULL;
	}
	bundle = accel->bundle[reserve.bank_nr - accel->num_ker_bundles];

	return bundle;
}

int adf_ctl_ioctl_reserve_ring(unsigned long arg)
{
	struct adf_user_reserve_ring reserve;
	struct adf_uio_control_bundle *bundle;
	struct adf_uio_instance_rings *instance_rings;
	int pid_entry_found = 0;

	if (copy_from_user(&reserve, (void __user *)arg,
			   sizeof(struct adf_user_reserve_ring))) {
		pr_err("QAT: failed to copy from user.\n");
		return -EFAULT;
	}

	bundle = adf_ctl_ioctl_bundle(reserve);
	if (!bundle) {
		pr_err("QAT: Failed to get bundle\n");
		return -EINVAL;
	}

	if (bundle->rings_used & (reserve.ring_mask)) {
		pr_err("QAT: Bundle %d, rings 0x%04X already reserved\n",
				reserve.bank_nr, reserve.ring_mask);
		return -EINVAL;
	}

	/* Find the list entry for this process */
	mutex_lock(&bundle->list_lock);
	list_for_each_entry(instance_rings, &bundle->list, list) {
		if (instance_rings->user_pid == current->tgid) {
			pid_entry_found = 1;
			break;
		}
	}
	mutex_unlock(&bundle->list_lock);

	if (!pid_entry_found) {
		pr_err("QAT: process %d not found\n", current->tgid);
		return -EINVAL;
	}

	instance_rings->ring_mask |= reserve.ring_mask;
	mutex_lock(&bundle->lock);
	bundle->rings_used |= reserve.ring_mask;
	mutex_unlock(&bundle->lock);

	return 0;
}

int adf_ctl_ioctl_release_ring(unsigned long arg)
{
	struct adf_user_reserve_ring reserve;
	struct adf_uio_control_bundle *bundle;
	struct adf_uio_instance_rings *instance_rings;
	int pid_entry_found;

	if (copy_from_user(&reserve, (void __user *)arg,
			   sizeof(struct adf_user_reserve_ring))) {
		pr_err("QAT: failed to copy from user.\n");
		return -EFAULT;
	}

	bundle = adf_ctl_ioctl_bundle(reserve);
	if (!bundle) {
		pr_err("QAT: Failed to get bundle\n");
		return -EINVAL;
	}

	/* Find the list entry for this process */
	pid_entry_found = 0;
	mutex_lock(&bundle->list_lock);
	list_for_each_entry(instance_rings, &bundle->list, list) {
		if (instance_rings->user_pid == current->tgid) {
			pid_entry_found = 1;
			break;
		}
	}
	mutex_unlock(&bundle->list_lock);

	if (!pid_entry_found) {
		pr_err("QAT: No ring reservation found for PID %d\n",
			current->tgid);
		return -EINVAL;
	}

	if ((instance_rings->ring_mask & reserve.ring_mask) !=
			reserve.ring_mask) {
		pr_err("QAT: Attempt to release rings not reserved by this process\n");
		return -EINVAL;
	}

	instance_rings->ring_mask &= ~reserve.ring_mask;
	mutex_lock(&bundle->lock);
	bundle->rings_used &= ~reserve.ring_mask;
	mutex_unlock(&bundle->lock);

	return 0;
}


int adf_ctl_ioctl_enable_ring(unsigned long arg)
{
	struct adf_user_reserve_ring reserve;
	struct adf_uio_control_bundle *bundle;

	if (copy_from_user(&reserve, (void __user *)arg,
			   sizeof(struct adf_user_reserve_ring))) {
		pr_err("QAT: failed to copy from user.\n");
		return -EFAULT;
	}

	bundle = adf_ctl_ioctl_bundle(reserve);
	if (!bundle) {
		pr_err("QAT: Failed to get bundle\n");
		return -EINVAL;
	}

	mutex_lock(&bundle->lock);
	adf_enable_ring_arb(bundle->csr_addr,
			    reserve.ring_mask);
	mutex_unlock(&bundle->lock);

	return 0;
}
int adf_ctl_ioctl_disable_ring(unsigned long arg)
{
	struct adf_user_reserve_ring reserve;
	struct adf_uio_control_bundle *bundle;

	if (copy_from_user(&reserve, (void __user *)arg,
			   sizeof(struct adf_user_reserve_ring))) {
		pr_err("QAT: failed to copy from user.\n");
		return -EFAULT;
	}

	bundle = adf_ctl_ioctl_bundle(reserve);
	if (!bundle) {
		pr_err("QAT: Failed to get bundle\n");
		return -EINVAL;
	}

	mutex_lock(&bundle->lock);
	adf_disable_ring_arb(bundle->csr_addr,
			    reserve.ring_mask);
	mutex_unlock(&bundle->lock);

	return 0;
}

static int adf_uio_open(struct uio_info *info, struct inode *inode)
{
	struct qat_uio_bundle_dev *priv = info->priv;

	adf_dev_get(priv->accel->accel_dev);
	return 0;
}

static int adf_uio_release(struct uio_info *info, struct inode *inode)
{
	return 0;
}

static int adf_uio_remap_bar(struct adf_accel_dev *accel_dev,
			     struct uio_info *info,
			     uint8_t bundle, uint8_t bank_offset)
{
	struct adf_bar bar =
		accel_dev->accel_pci_dev.pci_bars[ADF_UIO_GET_BAR(accel_dev)];
	char bar_name[ADF_DEVICE_NAME_LENGTH];
	unsigned int offset = bank_offset * ADF_RING_BUNDLE_SIZE;

	snprintf(bar_name, sizeof(bar_name), ADF_UIO_MAP_NAME,
		 ADF_UIO_GET_NAME(accel_dev), bundle);
	info->mem[0].name = kstrndup(bar_name, sizeof(bar_name), GFP_KERNEL);
	info->mem[0].addr = bar.base_addr + offset;
	info->mem[0].internal_addr = bar.virt_addr + offset;
	info->mem[0].size = ADF_RING_BUNDLE_SIZE;
	info->mem[0].memtype = UIO_MEM_PHYS;

	return 0;
}

/*   adf memory map operatoin   */
/*   in the close operation, we do the ring clean up if needed  */
static void adf_uio_mmap_close(struct vm_area_struct *vma)
{
	struct uio_info *info = vma->vm_private_data;
	struct qat_uio_bundle_dev *priv;
	struct adf_uio_control_bundle *bundle;
	struct adf_uio_instance_rings *instance_rings, *tmp;

	if (!info)
		return;

	priv = info->priv;

	/*
	 * Walk the instance list, if process not found but
	 * vma is found, apply pid fixup
	 */
	bundle = priv->bundle;
	mutex_lock(&bundle->list_lock);
	list_for_each_entry(instance_rings, &bundle->list, list) {
		if (instance_rings->user_pid == current->tgid) {
			break;
		} else if (instance_rings->vma == (uintptr_t)vma) {
			/* user_pid fixup */
			instance_rings->user_pid = current->tgid;
			break;
		}
	}
	mutex_unlock(&bundle->list_lock);

	/* Ensure that an uncontrolled device removal did not occur */
	if (priv->accel->accel_dev) {
		adf_uio_do_cleanup_orphan(info, priv->accel);
		adf_dev_put(priv->accel->accel_dev);
	}

	mutex_lock(&bundle->list_lock);
	list_for_each_entry_safe(instance_rings, tmp, &bundle->list, list) {
		if (instance_rings->user_pid == current->tgid) {
			list_del(&instance_rings->list);
			kfree(instance_rings);
			break;
		}
	}
	mutex_unlock(&bundle->list_lock);

	/* Decrease a reference counter for the accel kobj. */
	adf_uio_accel_unref(priv->accel);
	/* Decrease a reference counter for the bundle kobj. */
	adf_uio_bundle_unref(priv->bundle);
}

static struct vm_operations_struct adf_uio_mmap_operation = {
	.close = adf_uio_mmap_close,
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys,
#endif
};

static int find_mem_index(struct vm_area_struct *vma)
{
	struct uio_info *info = vma->vm_private_data;

	if (!info)
		return -1;

	if (vma->vm_pgoff < MAX_UIO_MAPS) {
		if (!info->mem[vma->vm_pgoff].size)
			return -1;
		return (int)vma->vm_pgoff;
	}

	return -EINVAL;
}

static int adf_uio_mmap(struct uio_info *info, struct vm_area_struct *vma)
{
	int mi;
	struct uio_mem *mem;
	struct qat_uio_bundle_dev *priv;
	struct adf_uio_instance_rings *instance_rings;
	struct adf_uio_control_bundle *bundle;
	int ret;

	if (!info)
		return -EINVAL;

	if (!vma)
		return -EINVAL;

	priv = info->priv;
	if (!priv)
		return -EINVAL;

	bundle = priv->bundle;
	if (!bundle)
		return -EINVAL;

	vma->vm_private_data = info;
	mi = find_mem_index(vma);
	if (mi < 0)
		return -EINVAL;

	/*  only support PHYS type here  */
	if (info->mem[mi].memtype != UIO_MEM_PHYS)
		return -EINVAL;

	instance_rings = kzalloc(sizeof(*instance_rings), GFP_KERNEL);
	if (!instance_rings)
		return -ENOMEM;

	instance_rings->user_pid = current->tgid;
	instance_rings->ring_mask = 0;
	instance_rings->vma = (uintptr_t)vma;
	mutex_lock(&bundle->list_lock);
	list_add_tail(&instance_rings->list, &bundle->list);
	mutex_unlock(&bundle->list_lock);

	/* Increment a reference counter for the accel object. */
	adf_uio_accel_ref(priv->accel);
	/* Increment a reference counter for the bundle object. */
	adf_uio_bundle_ref(priv->bundle);

	mem = info->mem + mi;
	vma->vm_ops = &adf_uio_mmap_operation;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = remap_pfn_range(vma,
			      vma->vm_start,
			      mem->addr >> PAGE_SHIFT,
			      vma->vm_end - vma->vm_start,
			      vma->vm_page_prot);
	if (ret) {
		mutex_lock(&bundle->list_lock);
		list_del(&instance_rings->list);
		mutex_unlock(&bundle->list_lock);
		kfree(instance_rings);
	}

	return ret;
}

static irqreturn_t adf_uio_isr_bundle(int irq, struct uio_info *info)
{
	struct qat_uio_bundle_dev *priv = info->priv;
	struct adf_accel_dev *accel_dev = priv->accel->accel_dev;
	struct adf_etr_data *etr_data = accel_dev->transport;
	struct adf_etr_bank_data *bank =
		&etr_data->banks[priv->hardware_bundle_number];

	WRITE_CSR_INT_FLAG_AND_COL(bank->csr_addr, bank->bank_number, 0);

	return IRQ_HANDLED;
}

static int adf_uio_init_bundle_dev(struct adf_accel_dev *accel_dev,
				   u8 bundle, u8 nb_bundles)
{
	char name[ADF_DEVICE_NAME_LENGTH];
	struct uio_info *info =
		&accel_dev->accel->bundle[bundle]->uio_info;
	struct qat_uio_bundle_dev *priv =
		&accel_dev->accel->bundle[bundle]->uio_priv;
	struct adf_accel_pci *pci_dev_info = &accel_dev->accel_pci_dev;
	unsigned hw_bundle_number = bundle +
			(GET_MAX_BANKS(accel_dev) - nb_bundles);
	struct adf_etr_data *etr_data = accel_dev->transport;
	struct adf_etr_bank_data *bank = &etr_data->banks[hw_bundle_number];
	unsigned int irq_flags = 0;

	priv->hardware_bundle_number = GET_MAX_BANKS(accel_dev)
				       - nb_bundles + bundle;
	priv->accel = accel_dev->accel;

	if (adf_uio_remap_bar(accel_dev, info, bundle,
			      priv->hardware_bundle_number))
		return -ENOMEM;

	snprintf(name, sizeof(name), ADF_UIO_NAME,
		 ADF_UIO_GET_NAME(accel_dev), accel_dev->accel_id, bundle);
	info->name = kstrndup(name, sizeof(name), GFP_KERNEL);
	info->version = ADF_DRV_VERSION;
	info->priv = priv;
	info->open = adf_uio_open;
	info->release = adf_uio_release;
	info->handler = adf_uio_isr_bundle;
	info->mmap = adf_uio_mmap;

	/* Use MSIX vector for PF and the proper IRQ for VF */
	if (!accel_dev->is_vf) {
		struct msix_entry *msixe = pci_dev_info->msix_entries.entries;
		info->irq = msixe[hw_bundle_number].vector;
	} else {
		struct pci_dev *pdev = accel_to_pci_dev(accel_dev);
		info->irq = pdev->irq;

		/* In VF we are sharing the interrupt */
		irq_flags = IRQF_SHARED;
	}
	irq_flags |= IRQF_ONESHOT;

	info->irq_flags = irq_flags;

	/* There is no need to set a hint for IRQs affinity cause the CPU
	 * affinity will be set from user space in adf_ctl
	 */

	/* Disable interrupts for this bundle but set the coalescence timer so
	 * that interrupts can be enabled on demand when creating a trans handle
	 */
	WRITE_CSR_INT_COL_EN(bank->csr_addr, hw_bundle_number, 0);
	WRITE_CSR_INT_COL_CTL(bank->csr_addr, hw_bundle_number,
			      bank->irq_coalesc_timer);

	if (uio_register_device(&accel_to_pci_dev(accel_dev)->dev, info))
		return -ENODEV;
	return 0;
}

static void adf_uio_del_bundle_dev(struct adf_accel_dev *accel_dev,
				   uint8_t nb_bundles)
{
	u8 i;

	for (i = 0; i < nb_bundles; i++)
		/* Decrease a reference counter for the bundle kobj. */
		adf_uio_sysfs_bundle_delete(accel_dev, i);
}

static void adf_uio_unregiser_dev(struct adf_accel_dev *accel_dev,
				  u8 nb_bundles)
{
	u8 i;

	for (i = 0; i < nb_bundles; i++) {
		struct uio_info *info = &accel_dev->accel->bundle[i]->uio_info;

		irq_set_affinity_hint(info->irq, NULL);
		uio_unregister_device(info);
	}
}

static void adf_uio_clean(struct adf_accel_dev *accel_dev,
			  struct qat_uio_pci_dev *uiodev)
{
	if (uiodev->nb_bundles)
		qdm_detach_device(&GET_DEV(accel_dev));
	adf_uio_unregiser_dev(accel_dev, uiodev->nb_bundles);
	adf_uio_del_bundle_dev(accel_dev, uiodev->nb_bundles);
	kfree(uiodev);
	pci_set_drvdata(accel_to_pci_dev(accel_dev), NULL);
}

int adf_uio_register(struct adf_accel_dev *accel_dev)
{
	struct qat_uio_pci_dev *uiodev;
	char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
	unsigned long num_ker_bundles = 0;
	struct pci_dev *pdev = accel_to_pci_dev(accel_dev);
	u8 i, j, nb_bundles;

	if (adf_cfg_get_param_value(accel_dev, ADF_GENERAL_SEC,
				    ADF_FIRST_USER_BUNDLE, val)) {
		nb_bundles = 0;
	}
	else {
		if (kstrtoul(val, 10, &num_ker_bundles))
			return -EINVAL;
		nb_bundles = GET_MAX_BANKS(accel_dev) - num_ker_bundles;
	}

	accel_dev->accel->num_ker_bundles = num_ker_bundles;

	if (nb_bundles) {
		uiodev = kzalloc(sizeof(*uiodev), GFP_KERNEL);
		if (!uiodev)
			return -ENOMEM;

		uiodev->nb_bundles = nb_bundles;

		for (i = 0; i < nb_bundles; i++)
			if (adf_uio_sysfs_bundle_create(pdev, i,
							accel_dev->accel))
				goto fail_bundle_create;
		for (j = 0; j < nb_bundles; j++)
			if (adf_uio_init_bundle_dev(accel_dev, j, nb_bundles))
				goto fail_init_bundle;

		adf_uio_init_accel_ctrl(accel_dev->accel, accel_dev,
					nb_bundles);
		pci_set_drvdata(pdev, uiodev);

		if (qdm_attach_device(&GET_DEV(accel_dev)))
			goto fail_unregister;
	} else {
		pci_set_drvdata(pdev, NULL);
	}

	return 0;

fail_unregister:
	pci_set_drvdata(accel_to_pci_dev(accel_dev), NULL);
fail_init_bundle:
	adf_uio_unregiser_dev(accel_dev, j);
fail_bundle_create:
	adf_uio_del_bundle_dev(accel_dev, i);
	kfree(uiodev);
	dev_err(&accel_to_pci_dev(accel_dev)->dev,
		"Failed to register UIO devices\n");
	return -ENODEV;
}

void adf_uio_remove(struct adf_accel_dev *accel_dev)
{
	struct qat_uio_pci_dev *uiodev =
		pci_get_drvdata(accel_to_pci_dev(accel_dev));

	if (uiodev)
		adf_uio_clean(accel_dev, uiodev);
}

static int adf_uio_event_handler(struct adf_accel_dev *accel_dev,
				 enum adf_event event)
{
	int ret = 0;
	struct device *dev = &GET_DEV(accel_dev);
	char *event_str = NULL;
	char *dev_id = NULL;
	char *envp[3];

	switch (event) {
	case ADF_EVENT_INIT:
		return ret;
	case ADF_EVENT_SHUTDOWN:
		return ret;
	case ADF_EVENT_RESTARTING:
		event_str = "qat_event=restarting";
		break;
	case ADF_EVENT_RESTARTED:
		event_str = "qat_event=restarted";
		break;
	case ADF_EVENT_START:
		return ret;
	case ADF_EVENT_STOP:
		return ret;
	case ADF_EVENT_ERROR:
		event_str = "qat_event=error";
		break;
	default:
		return -EINVAL;
	}

	dev_id = kasprintf(GFP_ATOMIC, "accelid=%d", accel_dev->accel_id);
	if (!dev_id) {
		dev_err(&GET_DEV(accel_dev), "Failed to allocate memory\n");
		return -ENOMEM;
	}

	envp[0] = event_str;
	envp[1] = dev_id;
	envp[2] = NULL;
	ret = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
	if (ret) {
		dev_err(&GET_DEV(accel_dev), "Failed to send event %s\n",
			event_str);
		goto end;
	}

	if (event == ADF_EVENT_RESTARTING) {
		event_str = "qat_event=stop";
		envp[0] = event_str;
		envp[1] = dev_id;
		envp[2] = NULL;
		ret = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
		if (ret) {
			dev_err(&GET_DEV(accel_dev),
				"Failed to send event %s\n", event_str);
			goto end;
		}

		event_str = "qat_event=shutdown";
		envp[0] = event_str;
		envp[1] = dev_id;
		envp[2] = NULL;
		ret = kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
		if (ret) {
			dev_err(&GET_DEV(accel_dev),
				"Failed to send event %s\n", event_str);
			goto end;
		}
	}

end:
	kfree(dev_id);

	return ret;
}

int adf_uio_service_register(void)
{
	memset(&adf_uio_hndl, 0, sizeof(adf_uio_hndl));
	adf_uio_hndl.event_hld = adf_uio_event_handler;
	adf_uio_hndl.name = "adf_event_handler";
	return adf_service_register(&adf_uio_hndl);
}

int adf_uio_service_unregister(void)
{
	return adf_service_unregister(&adf_uio_hndl);
}
