/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *  version: QAT1.7.L.4.6.0-00025
 *
 ***************************************************************************/
/**
*****************************************************************************
 * @file qae_mem_utils.c
 *
 * This file provides linux kernel memory allocation for quick assist API
 *
 *****************************************************************************/

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include "qae_mem.h"
#include "qae_mem_utils.h"

#define IS_VMALLOC_ADDR(addr) (((uintptr_t)(addr) >= VMALLOC_START) && \
        ((uintptr_t)(addr) < VMALLOC_END))

/**
******************************************************************************
* @ingroup max_mem_numa
*       maximum amount of memory allocated in kernel space
* @description
*       This is a command line parameter that defines the maximum
*       amount of memory allocated by the driver in kernel space.
*       Measured in kilobytes.
*****************************************************************************/
static uint32_t max_mem_numa = 0;
/**
******************************************************************************
* @ingroup mem_allocated
*       amount of memory currently allocated in kernel space
* @description
*       This variable holds the overall
*       amount of memory allocated by the driver in kernel space.
*       Measured in bytes.
*****************************************************************************/
static size_t mem_allocated = 0;
module_param(max_mem_numa, uint, S_IRUGO);
MODULE_PARM_DESC(max_mem_numa,"Maximum number of allocatable memory in 1k units");

static uint32_t numaAllocations_g = 0;
static uint32_t normalAllocations_g = 0;

/*Defining Max Size limit to be used, to allocate using kmalloc as 4MB */
static const int QAE_MEM_SIZE_LIMIT = 1024 * 4096;

/**************************************
 * Memory functions
 *************************************/
void* qaeMemAlloc (size_t memsize)
{
   if(memsize > QAE_MEM_SIZE_LIMIT)
   {
       return ( vmalloc(memsize) );
   }
   normalAllocations_g++;
   return (kmalloc (memsize, GFP_KERNEL));
}

void* qaeMemAllocNUMA(size_t size, int node, size_t alignment)
{
    void* ptr = NULL;
    void* phys_ptr = NULL;
    void* pRet = NULL;
    size_t alignment_offset = 0;
    qae_mem_alloc_info_t memInfo = {0};
    size_t totalInKBytes = (mem_allocated + size)/QAE_KBYTE;

    if( (mem_allocated + size) % QAE_KBYTE )
    {
        totalInKBytes += 1;
    }

    if( max_mem_numa && max_mem_numa < totalInKBytes)
    {
        mm_err("%s:%d Maximum NUMA allocation of %u kB reached "
                    "currently allocated %zu bytes requested %zu bytes\n",
                    __func__,__LINE__,max_mem_numa,mem_allocated, size);
        return NULL;
    }

    if(!size || alignment < 1)
    {
        mm_err("%s:%d Either size or alignment is zero - size = %zu, "
               "alignment = %zu \n",__func__,__LINE__,size,alignment);
        return NULL;
    }
    /*alignment should be 1,2,4,8....*/
    if(alignment & (alignment-1))
    {
        mm_err("%s:%d Expecting alignment of a power of "\
                "two but did not get one\n",__func__,__LINE__);
        return NULL;
    }
    /*add the alignment and the struct size to the buffer size*/
    memInfo.mSize = icp_iommu_get_remapping_size(size + alignment +
                                            sizeof(qae_mem_alloc_info_t));
    if(memInfo.mSize > QAE_MEM_SIZE_LIMIT)
    {
        mm_err("%s:%d Total size needed for this " \
            "set of size and alignment (%zu) exceeds the OS " \
            "limit %d\n", __func__,__LINE__,memInfo.mSize,QAE_MEM_SIZE_LIMIT);
        return NULL;
    }
    /*allocate contigous memory*/
    ptr = kmalloc_node (memInfo.mSize, GFP_KERNEL, node);
    if(!ptr)
    {
        mm_err("%s:%d failed to allocate memory\n",__func__,__LINE__);
        return NULL;
    }
    /*store the base address into the struct*/
    memInfo.mAllocMemPtr = ptr;
#ifdef ICP_IOMMU_DISABLED
    icp_iommu_map(&phys_ptr, ptr, memInfo.mSize);
#else
    if (icp_iommu_map(&phys_ptr, ptr, memInfo.mSize))
    {
        mm_err("%s:%d failed to iommu map\n",__func__,__LINE__);
        kfree(ptr);
        return NULL;
    }
#endif
    /*add the size of the struct to the return pointer*/
    pRet = (char *)memInfo.mAllocMemPtr +  sizeof(qae_mem_alloc_info_t);
    /*compute the offset from the alignement*/
    alignment_offset = (uintptr_t)pRet % alignment;
    /*in order to obtain the pointer to the buffer add the alignment and
    subtract the offset, now we have the return pointer aligned*/
    pRet = (char*)pRet + (alignment - alignment_offset);
    /*copy the struct immediately before the buffer pointer*/
    memcpy((void*)((char*)pRet - sizeof(qae_mem_alloc_info_t)),
           (void*)(&memInfo),
           sizeof(qae_mem_alloc_info_t));
    /*increment the NUMA allocations counter*/
    numaAllocations_g++;
    mem_allocated += memInfo.mSize;
    return pRet;
}

void qaeMemFreeNUMA (void** ptr)
{
    qae_mem_alloc_info_t *memInfo = NULL;
    uint64_t phy_addr = 0;

    if(!ptr || !(*ptr) )
    {
        mm_err("%s:%d Pointer to be freed cannot be NULL\n",
              __func__,__LINE__);
        return;
    }
    memInfo = (qae_mem_alloc_info_t *)((int8_t *)*ptr -
                                  sizeof(qae_mem_alloc_info_t));

    if (memInfo->mSize == 0 || memInfo->mAllocMemPtr == NULL)
    {
        mm_err("%s:%d Detected the corrupted data: memory leak!! \n",
               __func__,__LINE__);
        mm_err("%s:%d Size: %zu, memPtr: %p\n",
                __func__,__LINE__,memInfo->mSize, memInfo->mAllocMemPtr);
        return;
    }
    phy_addr = virt_to_phys(memInfo->mAllocMemPtr);
#ifdef ICP_IOMMU_DISABLED
    icp_iommu_unmap((void*)(uintptr_t) phy_addr, memInfo->mSize);
#else
    if (icp_iommu_unmap((void*)(uintptr_t) phy_addr, memInfo->mSize))
    {
        mm_warning("%s:%d failed to iommu unmap\n",__func__,__LINE__);
    }
#endif
    kfree (memInfo->mAllocMemPtr);
    numaAllocations_g--;
    if ( mem_allocated > memInfo->mSize )
    {
        mem_allocated -= memInfo->mSize;
    }
    else
    {
        mem_allocated = 0;
    }
    *ptr = NULL;
}

void qaeMemFree (void **ptr)
{
    if(!ptr || !(*ptr) )
    {
        mm_err("%s:%d Pointer to be freed cannot be NULL\n",__func__,__LINE__);
        return;
    }
    if(IS_VMALLOC_ADDR(*ptr))
    {
        vfree(*ptr);
        return;
    }
    kfree (*ptr);
    normalAllocations_g--;
    *ptr = NULL;
}

uint64_t qaeVirtToPhysNUMA(void* ptr)
{
    if (!ptr)
    {
        mm_err("%s:%d Input parameter cannot be NULL \n",
               __func__,__LINE__);
        return 0;
    }
    return (uint64_t)(uintptr_t)virt_to_phys(ptr);
}
