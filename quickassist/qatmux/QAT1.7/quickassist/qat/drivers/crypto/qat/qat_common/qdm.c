/*
 *  This file is provided under a dual BSD/GPLv2 license.  When using or
 *  redistributing this file, you may do so under either license.
 *
 *  GPL LICENSE SUMMARY
 *  Copyright(c) 2016 Intel Corporation.
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
 *  Copyright(c) 2016 Intel Corporation.
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
#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#else
#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
#endif

#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/pci.h>
#include "qdm.h"

static struct iommu_domain *domain;

/**
 * qdm_attach_device() - Attach a device to the QAT IOMMU domain
 * @dev: Device to be attached
 *
 * Function attaches the device to the QDM IOMMU domain.
 *
 * Return: 0 on success, error code otherwise.
 */
int qdm_attach_device(struct device *dev)
{
	if (!domain)
		return 0;

	if (!dev) {
		pr_err("QDM: Invalid device\n");
		return -ENODEV;
	}

	return iommu_attach_device(domain, dev);
}

/**
 * qdm_detach_device() - Detach a device from the QAT IOMMU domain
 * @dev: Device to be detached
 *
 * Function detaches the device from the QDM IOMMU domain.
 *
 * Return: 0 on success, error code otherwise.
 */
int qdm_detach_device(struct device *dev)
{
	if (!domain)
		return 0;

	if (!dev) {
		pr_err("QDM: Invalid device\n");
		return -ENODEV;
	}

	iommu_detach_device(domain, dev);
	return 0;
}

/**
 * qdm_iommu_map() - Map a block of memory to the QAT IOMMU domain
 * @iova:   Device virtual address
 * @vaddr:  Kernel virtual address
 * @size:   Size (in bytes) of the memory block.
 *          Must be a multiple of PAGE_SIZE
 *
 * Function maps a block of memory to the QDM IOMMU domain.
 *
 * Return: 0 on success, error code otherwise.
 */
int qdm_iommu_map(dma_addr_t *iova, void *vaddr, size_t size)
{
	phys_addr_t paddr = (phys_addr_t) virt_to_phys(vaddr);
	*iova = (dma_addr_t) paddr;

	if (!domain)
		return 0;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
	return iommu_map_range(domain, *iova, paddr, size,
			IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE);
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(3,2,45) && \
    LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
	return iommu_map(domain, *iova, paddr, get_order(size),
			IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE);
#else
	return iommu_map(domain, *iova, paddr, size,
			IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE);
#endif
}
EXPORT_SYMBOL_GPL(qdm_iommu_map);

/**
 * qdm_iommu_unmap() - Unmap a block of memory from the QAT IOMMU domain
 * @iova:   Device virtual address
 * @size:   Size (in bytes) of the memory block
 *          Must be the same size as mapped.
 *
 * Function unmaps a block of memory from the QDM IOMMU domain.
 *
 * Return: 0 on success, error code otherwise.
 */
int qdm_iommu_unmap(dma_addr_t iova, size_t size)
{
	if (!domain)
		return 0;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
	iommu_unmap_range(domain, (unsigned long)iova, size);
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(3,2,45) && \
    LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
	iommu_unmap(domain, (unsigned long)iova, get_order(size));
#else
	iommu_unmap(domain, (unsigned long)iova, size);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(qdm_iommu_unmap);

int __init qdm_init(void)
{
	if (!iommu_present(&pci_bus_type))
		return 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
	domain = iommu_domain_alloc();
#else
	domain = iommu_domain_alloc(&pci_bus_type);
#endif
	if (!domain) {
		pr_err("QDM: Failed to allocate a domain\n");
		return -1;
	}
	return 0;
}

void __exit qdm_exit(void)
{
	if (domain)
		iommu_domain_free(domain);
	domain = NULL;
}
