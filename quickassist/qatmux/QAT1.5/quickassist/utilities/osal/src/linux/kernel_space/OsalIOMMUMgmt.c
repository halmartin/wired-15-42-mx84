/**
 * @file kernel_space/OsalMMUMgmt.c (linux)
 *
 * @brief IOMMU module.
 *
 *
 * @par
 * GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify 
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but 
 *   WITHOUT ANY WARRANTY; without even the implied warranty of 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License 
 *   along with this program; if not, write to the Free Software 
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution 
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *  version: QAT1.5.L.1.11.0-36
*/

#include "Osal.h"
#include "OsalOsTypes.h"
#include <linux/pci.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
#include <linux/iommu.h>
#endif

#ifndef ICP_WITHOUT_IOMMU

static struct iommu_domain *domain = NULL;

int osalIOMMUMap(UINT64 iova, UINT64 phaddr, size_t size)
{
    OSAL_MEM_ASSERT(domain);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,2,45) && \
    LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
    return iommu_map(domain, (unsigned long)iova,
              (phys_addr_t)phaddr, get_order(size),
              IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE);
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
    return iommu_map_range(domain,(unsigned long)iova,
              (phys_addr_t)phaddr,size,
              IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE);
#else
    return iommu_map(domain, (unsigned long)iova,
              (phys_addr_t)phaddr, size,
              IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE);
#endif
}

int osalIOMMUUnmap(UINT64 iova, size_t size)
{
    OSAL_MEM_ASSERT(domain);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,2,45) && \
    LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
    return iommu_unmap(domain, (unsigned long)iova, get_order(size));
#else

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
    iommu_unmap_range(domain, (unsigned long)iova, size);
#else 
    if ( size != iommu_unmap(domain, (unsigned long)iova, size))
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDERR,
            "osalIOMMUUnmap(): Failed to unmap \n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
#endif
    return OSAL_SUCCESS;
#endif
}

UINT64 osalIOMMUVirtToPhys(UINT64 iova)
{
    OSAL_MEM_ASSERT(domain);
    return (UINT64)iommu_iova_to_phys(domain, (unsigned long)iova);
}

int osalIOMMUAttachDev(void *dev)
{
    OSAL_MEM_ASSERT(domain);
    if( NULL == dev ) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDERR,
            "osalIOMMUAttachDev(): Invalid device \n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return -ENODEV;
    }
    return iommu_attach_device(domain, dev);
}

void osalIOMMUDetachDev(void *dev)
{
    OSAL_MEM_ASSERT(domain);
    if( NULL == dev ) {
        osalLog (OSAL_LOG_LVL_WARNING, OSAL_LOG_DEV_STDERR,
            "osalIOMMUDetachDev(): Invalid device \n",
            0, 0, 0, 0, 0, 0, 0, 0);
	return;
    }
    iommu_detach_device(domain, dev);
}

size_t osalIOMMUgetRemappingSize(size_t size)
{
    /* To improve memory usage efficiency 
     * a bit-map based allocation algorithm will be 
     * implemented with the page as the smallest allocation unit
     * therefore remapping size is at least PAGE_SIZE
     */
    int pages = size % PAGE_SIZE ? size/PAGE_SIZE + 1 : size/PAGE_SIZE;        	
    size_t new_size = (pages * PAGE_SIZE);
    return new_size;
}

int osalIOMMUInit(void)
{
    struct iommu_domain* dummy_domain = NULL;
/* depending on the linux version, we first check
 * if iommu is available and allocate the iommu domain
 * to a local variable.
 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,1,10)
    if (!iommu_found()) {
#else
    if (!iommu_present(&pci_bus_type)) {
#endif
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDERR,
            "osalIOMMUInit(): iommu not found \n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,1,10)
    dummy_domain = iommu_domain_alloc();
#else
    dummy_domain = iommu_domain_alloc(&pci_bus_type);
#endif
    if ( __sync_bool_compare_and_swap((volatile struct iommu_domain **)&domain,NULL,dummy_domain))
    {
/* If domain is NULL it is initialized with dummy_domain.
*/
        if ( NULL == dummy_domain ) 
        {
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDERR,
                    "osalIOMMUInit(): Failed to init \n",
                    0, 0, 0, 0, 0, 0, 0, 0);
            return OSAL_FAIL;
        }
        return OSAL_SUCCESS;
    }

/* If the domain was already allocated then it is
 * still valid and dummy_domain is released.
 */
    if ( dummy_domain ) 
    {
        iommu_domain_free(dummy_domain);    
    }     
    return OSAL_SUCCESS;
}

void osalIOMMUExit(void)
{
    struct iommu_domain* existing_domain =  
            (struct iommu_domain*)__sync_lock_test_and_set(
                                    (volatile struct iommu_domain **)&domain,NULL);
    if( existing_domain )
    {
        iommu_domain_free(existing_domain);
    }
}

#else
int osalIOMMUMap(UINT64 iova, UINT64 phaddr, size_t size)
{
    return 0;
}

int osalIOMMUUnmap(UINT64 iova, size_t size)
{
    return 0;
}

UINT64 osalIOMMUVirtToPhys(UINT64 iova)
{
    return iova;
}

int osalIOMMUAttachDev(void *dev)
{
    return OSAL_SUCCESS;
}

void osalIOMMUDetachDev(void *dev)
{
}

size_t osalIOMMUgetRemappingSize(size_t size)
{
    return size;
}

int osalIOMMUInit(void)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29))
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,1,10)
    if (iommu_found()) {
#else
    if (iommu_present(&pci_bus_type)) {
#endif /* End of Kernel Version check for 3.1.10 */
#ifndef ICP_SRIOV
      osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDERR, \
          "osalIOMMUInit(): iommu is enabled, \
           but driver is not built with SRIOV \n", \
            0, 0, 0, 0, 0, 0, 0, 0);
      return OSAL_FAIL;
#endif /* End of Is ICP_SRIOV defined check */
    }
#endif /* End of Kernel Version check for 2.6.29 */
    return OSAL_SUCCESS;
}

void osalIOMMUExit(void)
{
}
#endif
