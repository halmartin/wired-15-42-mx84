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
 *
 * @file qae_mem_drv.c
 *
 * @brief Kernel-space support for user-space contiguous memory allocation
 *
 */

#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/nodemask.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/hugetlb.h>

#include "qae_mem_utils.h"

#define DEV_MEM_NAME            "usdm_drv"
#define MODULE_NAME             "USDM"
#define DEV_MEM_MAJOR           0
#define DEV_MEM_MAX_MINOR       1
#define DEV_MEM_BASE_MINOR      0
#define QAE_LOCAL_ENSURE(c, str, ret)        \
    if (!(c)) {                              \
        mm_err("%s in file %s, ret = %d\n",  \
            str, __FILE__, ret);             \
    return ret; }

#define FREE(ptr)   kfree(ptr)
#define IS_PAGE_ALIGNED(x)  (PAGE_ALIGN((uintptr_t) (x)) == (uintptr_t) (x))

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
/**
******************************************************************************
* @ingroup max_huge_pages
*       total number of huge pages currently reserved
* @description
*       This variable holds the total number of
*       huge pages reserved by the memory driver.
*       Measured in number of huge pages.
*****************************************************************************/
static uint max_huge_pages = 0;
/**
******************************************************************************
* @ingroup max_huge_pages_per_process
*       number of huge pages could be allocated
*       for each user space process
* @description
*       This variable holds the number of
*       huge pages allocated by each process.
*       Measured in number of huge pages.
*****************************************************************************/
static uint max_huge_pages_per_process = 0;


module_param(max_mem_numa, uint, S_IRUGO);
MODULE_PARM_DESC(max_mem_numa,
        "Maximum number of allocatable memory in 1k units");

module_param(max_huge_pages, uint, S_IRUGO);
MODULE_PARM_DESC(max_huge_pages,
        "Maximum number of huge pages enabled for the module");

module_param(max_huge_pages_per_process, uint, S_IRUGO);
MODULE_PARM_DESC(max_huge_pages_per_process,
        "Maximum number of huge pages enabled for each process");

/* Version 0.7.1:
 * - Slab caching in user space introduced;
 * - Slab hash introduced for fast searching;
 * - Performance optimizations;
 * - Adding huge pages support.
 */
static const char VERSION_STRING[]="Version 0.7.1";

static DEFINE_MUTEX(dev_mem_lock);
static user_mem_dev_t *mem_dev_numa = NULL;

/*directory entry structure for debug root directory and debug file*/
static struct dentry *qae_dbg_root_dir = NULL;
static struct dentry *qae_dbg_slabs_file = NULL;

typedef struct chr_drv_info_s
{
    unsigned               major;
    unsigned               min_minor;
    unsigned               max_minor;
    char                   *name;
    struct cdev            drv_cdev;
    struct class           *drv_class;
    struct device          *drv_class_dev;
    unsigned               num_devices;
    unsigned               unregistered;
} chr_drv_info_t;

typedef struct {
    kdev_mem_info_t *head;
    kdev_mem_info_t *tail;
} slab_list_t;
/* Kernel space hash for fast slab searching */
static slab_list_t g_slab_list[PAGE_SIZE] = {{0}};

extern int handle_other_ioctls(uint32_t cmd);
/******************************************************************************
 * debug: /sys/kernel/debug/qae_mem_dbg directory
 * qae_mem_slabs file
 * cat qae_mem_slabs shows the allocated slabs for each process with the
 * physical and virtual start address
 * echo "d processid virt_addr" > qae_mem_slabs
 * echo "d processid phys_addr" > qae_mem_slabs
 * write dump command to debug file, the next cat qae_mem_slabs command
 * shows the 256 byte from address in hex and ascii format
 * echo "c processid slabid" > qae_mem_slabs
 * write dump command to debug file, the next cat qae_mem_slabs command
 * shows the 32 x 64 allocation bit map for small buffers allocations
 ******************************************************************************/

/*****************************************************************************
                             memory mgt code begin
*****************************************************************************/

static inline uint64_t get_key(const uint64_t phys)
{
    /* Use bits 20-31 of a physical address as a hash key.
     * It provides a good distribution for 1Mb/2Mb slabs
     * and a moderate distribution for 128Kb/256Kb/512Kb slabs.
     */
    return (phys >> 20) & ~PAGE_MASK;
}

static inline void add_slab_to_hash(kdev_mem_info_t *slab)
{
    const size_t key = get_key(slab->phy_addr);

    ADD_ELEMENT_TO_HEAD_LIST(slab, g_slab_list[key].head,
            g_slab_list[key].tail, _kernel_hash);
}

static inline void del_slab_from_hash(kdev_mem_info_t *slab)
{
    const size_t key = get_key(slab->phy_addr);

    REMOVE_ELEMENT_FROM_LIST(slab, g_slab_list[key].head,
            g_slab_list[key].tail, _kernel_hash);
}

static inline kdev_mem_info_t *find_slab(const uint64_t phy_addr)
{
    const size_t key = get_key(phy_addr);
    kdev_mem_info_t *slab = g_slab_list[key].head;

    while (slab)
    {
        if (phy_addr == slab->phy_addr)
            return slab;
        slab = slab->pNext_kernel_hash;
    }

    return NULL;
}

/*
 * Find memory information
 */
static kdev_mem_info_t*
userMemGetInfo(struct file* fp, uint64_t id)
{
    user_proc_mem_list_t* list = NULL;
    if(!fp)
    {
        mm_err("%s:%d Invalid file pointer\n",__func__,__LINE__ );
        return NULL;
    }
    list = (user_proc_mem_list_t*)fp->private_data;
    if(!list)
    {
        mm_info("%s:%d empty list\n",__func__,__LINE__);
        return NULL;
    }
    return find_slab(id);
}
/*
 * Allocate numa memory
 */
static dev_mem_info_t *
userMemAlloc(struct file* fp, size_t size,
             int node, int large_memory)
{
    block_ctrl_t *block_ctrl = NULL;
    dev_mem_info_t *mem_info = NULL;
    kdev_mem_info_t *kmem = NULL;
    user_proc_mem_list_t *list = NULL;
    void *phy_addr = NULL;
    size_t totalInKBytes = 0;

    if(!size || !fp)
    {
        mm_err("%s:%d Invalid parameter value [%zu] %p\n",
                __func__, __LINE__, size, fp);
        return NULL;
    }

    if(node != NUMA_NO_NODE)
    {
        /* node 0.. (MAX_NUMNODES-1) */
        if(node >= 0 && node < MAX_NUMNODES)
        {
           if(!node_online(node))
           {
               mm_err("%s:%d Requested node %d is not online. "
               "Using node 0 as default\n",
               __func__, __LINE__,node);
               node = 0;
           }
       }
       else
       {
           /*greater than MAX_NUMNODES  */
           mm_err("%s:%d Requested node %d not present. "
               "Using node 0 as default\n",
               __func__, __LINE__,node);
           node = 0;
       }
    }

    /*
     * Find the process allocation list
     */
    list = (user_proc_mem_list_t*)fp->private_data;
    if (!list)
    {
        mm_err("%s:%d User process memory list is NULL \n",__func__,__LINE__);
        return NULL;
    }

    size = icp_iommu_get_remapping_size(size);
    totalInKBytes = DIV_ROUND_UP(mem_allocated + size, QAE_KBYTE);

    /* for request > 2M mem_info control block allocated separately */
    if(large_memory)
    {
        /*one page is used for block control information*/
        const uint32_t pageSizeInKb = PAGE_SIZE / QAE_KBYTE;
        if ( max_mem_numa && max_mem_numa < (totalInKBytes + pageSizeInKb))
        {
            mm_err(KERN_ERR "%s:%d Maximum NUMA allocation of %u kB reached "
                    "currently allocated %zu bytes requested %zu bytes\n",
                    __func__,__LINE__,max_mem_numa,
                    mem_allocated , (size_t)(size + PAGE_SIZE) );
            return NULL;
        }
        mem_info = (dev_mem_info_t*) get_zeroed_page(GFP_KERNEL);
        if ( !mem_info )
        {
            mm_err("%s:%d Unable to allocate control block\n",
                    __func__,__LINE__);
            return NULL;
        }
        kmem = kmalloc (sizeof(kdev_mem_info_t), GFP_KERNEL);
        if ( !kmem )
        {
            mm_err("%s:%d Unable to allocate Kernel control block\n",
                   __func__,__LINE__);
            free_page((unsigned long) mem_info);
            return NULL;
        }
        /* kmalloc is faster than kzalloc */
        kmem->kmalloc_ptr = kmalloc_node(size, GFP_KERNEL, node);
        if (!kmem->kmalloc_ptr || !IS_PAGE_ALIGNED(kmem->kmalloc_ptr))
        {
            mm_err("%s:%d Unable to allocate memory slab size %zu"
                    " or wrong alignment: %p\n",
                    __func__, __LINE__, size, kmem->kmalloc_ptr);
            FREE(kmem->kmalloc_ptr);
            FREE(kmem);
            free_page((unsigned long) mem_info);
            return NULL;
        }
        /* Initialize the huge page control */
        kmem->huge_mem_ctrl = mem_info;
        /* Update slab size */
        kmem->size = size;
        /* Update allocated size */
        mem_allocated += (size + PAGE_SIZE);
    }
    else
    {
        if ( max_mem_numa && max_mem_numa < totalInKBytes )
        {
            mm_err(KERN_ERR "%s:%d Maximum NUMA allocation of %u kB reached"
                    " currently allocated %zu bytes requested %zu bytes\n",
                    __func__, __LINE__, max_mem_numa,
                    mem_allocated, size);
            return NULL;
        }
        block_ctrl = kmalloc_node(size, GFP_KERNEL, node);
        if (!block_ctrl || !IS_PAGE_ALIGNED(block_ctrl))
        {
            mm_err("%s:%d Unable to allocate memory slab"
                    " or wrong alignment: %p\n",
                    __func__, __LINE__, block_ctrl);
            FREE(block_ctrl);
            return NULL;
        }

        kmem = kmalloc (sizeof(kdev_mem_info_t), GFP_KERNEL);
        if ( !kmem )
        {
             mm_err("%s:%d Unable to allocate Kernel control block\n",
                    __func__,__LINE__);
             FREE(block_ctrl);
             return NULL;
        }

        /* It is faster to alloc a slab and memset it later vs. kZalloc_node. */
        memset(block_ctrl, 0, sizeof(block_ctrl_t));
        mem_info = &block_ctrl->mem_info;
        kmem->kmalloc_ptr = block_ctrl;
        /* Huge page control block not applicable here for small slabs. */
        kmem->huge_mem_ctrl = NULL;
        /* Update slab size */
        kmem->size = size;
        /* Update allocated size */
        mem_allocated += size;
    }
    mem_info->nodeId = node;
    mem_info->size = size;
    mem_info->type = large_memory;
#ifdef ICP_IOMMU_DISABLED
    icp_iommu_map(&phy_addr, kmem->kmalloc_ptr, mem_info->size);
#else
    if (icp_iommu_map(&phy_addr, kmem->kmalloc_ptr, mem_info->size))
    {
        mm_err("%s:%d iommu map failed\n",__func__,__LINE__);
        if( LARGE == mem_info->type )
        {
            free_page((unsigned long) mem_info);
            mem_allocated -= PAGE_SIZE;
        }
        /* For small block size, kmalloc_ptr points to block_ctrl */
        FREE(kmem->kmalloc_ptr);
        FREE(kmem);
        mem_allocated -= size;
        return NULL;
    }
#endif
    mem_info->phy_addr = (uintptr_t) phy_addr;
    kmem->phy_addr = (uintptr_t) phy_addr;
    list->allocs_nr++;
    ADD_ELEMENT_TO_END_LIST(kmem, list->head, list->tail, _kernel);
    add_slab_to_hash(kmem);

    return mem_info;
}
/*
 * Free slab
 */
static void
free_slab(user_proc_mem_list_t *list, kdev_mem_info_t *slab,
          const int cleanup)
{
    void *ptr = slab->kmalloc_ptr;
    const size_t size = slab->size;

    icp_iommu_unmap((void *) (uintptr_t) slab->phy_addr, size);
    REMOVE_ELEMENT_FROM_LIST(slab, list->head, list->tail, _kernel);

    del_slab_from_hash(slab);

    /* If we are dealing with huge pages, then huge_mem_ctrl is not NULL
     * and the slab can be freed.
     */
    if(slab->huge_mem_ctrl)
    {
        free_page((unsigned long) slab->huge_mem_ctrl);
        mem_allocated -= PAGE_SIZE;
    }

    if (cleanup)
    {
        /* Cleanup released memory. */
        memset(ptr, 0, size);
    }
    FREE(ptr);

    /* Destroy the slab as it is no longer needed */
    FREE(slab);

    mem_allocated -= size;
    list->allocs_nr -= 1;
}
/*
 * Free memory
 */
static int
userMemFree(struct file* fp, uint64_t id)
{
    user_proc_mem_list_t* list = NULL;
    kdev_mem_info_t *kmem = NULL;

    if (!fp)
    {
        mm_err("%s:%d Invalid file pointer\n",__func__,__LINE__);
        return -EIO;
    }
    list = (user_proc_mem_list_t *)fp->private_data;
    if(!list)
    {
        mm_warning("%s:%d No slab to free\n",__func__,__LINE__);
        return -EIO;
    }
    kmem = find_slab(id);
    if (kmem)
    {
        /* Free memory slab, no cleanup. */
        free_slab(list, kmem, 0);
        return 0;
    }
    mm_warning("%s:%d Could not find slab with id: %llu \n",
            __func__,__LINE__,id);
    return -EIO;
}
/*
 * Clean all memory for a process
 */
static int
userMemFreeSlabs(struct file* fp)
{
    kdev_mem_info_t* kmem = NULL, *next = NULL;
    user_proc_mem_list_t* list = NULL;

    if (!fp)
    {
        mm_err("%s:%d Invalid file pointer\n",__func__,__LINE__);
        return -EIO;
    }
    list = (user_proc_mem_list_t*)fp->private_data;
    if(!list)
    {
        mm_warning("%s:%d No slab to free\n",__func__,__LINE__);
        return -EIO;
    }
    max_huge_pages += list->hugepages_nr;
#ifdef ICP_DEBUG
    mm_info("[FREE] pid: %u return number of pages: %llu total number: %u\n",
           current->pid,
           list->hugepages_nr,
           max_huge_pages);
#endif
    list->hugepages_nr = 0;
    kmem = list->head;
    while(kmem)
    {
#ifdef ICP_DEBUG
        mm_warning("%s:%d Potential memory leak, Process Id %d "
               "Virtual address %p "
               "Physical address %px has allocated block\n",
               __func__,__LINE__,
               list->pid,
               kmem->kmalloc_ptr,
               (void*)kmem->phy_addr);
#endif
        next = kmem->pNext_kernel;
        /* Free and cleanup memory slab. */
        free_slab(list, kmem, 1);
        kmem = next;
    }
    return 0;
}

/*****************************************************************************
                              memory mgt code end
*****************************************************************************/

static int
dev_mem_alloc(struct file* fp, uint32_t cmd, unsigned long arg)
{
    unsigned long ret = 0;
    dev_mem_info_t* mem_info = NULL;
    dev_mem_info_t user_mem_info = {0};

    if( fp == NULL )
    {
        mm_err("%s:%d Invalid file descriptor\n",__func__,__LINE__);
        return -EIO;
    }
    if( fp->private_data == NULL)
    {
        mm_err("%s:%d Invalid file private data \n",__func__,__LINE__);
        return -EIO;
    }
    ret = copy_from_user(&user_mem_info,
                         (dev_mem_info_t *)arg,
                         sizeof(dev_mem_info_t));
    if (unlikely(ret))
    {
        mm_err("%s:%d copy_from_user failed, ret=%lu\n",
                  __func__,__LINE__,ret);
        return -EIO;
    }
    mem_info = userMemAlloc(fp, user_mem_info.size,
                            (int) user_mem_info.nodeId,
                            user_mem_info.type);
    if (!mem_info)
    {
        mm_err("%s:%d userMemAlloc failed\n",__func__,__LINE__);
        return -ENOMEM;
    }
    ret = copy_to_user((dev_mem_info_t *)arg,
                        mem_info,
                        sizeof(dev_mem_info_t));
    if (unlikely(ret))
    {
        (void) userMemFree(fp, user_mem_info.phy_addr);
        mm_err("%s:%d copy_to_user failed, ret=%lu\n",
                  __func__,__LINE__,ret);
        return -EIO;
    }
    return 0;
}

static int
dev_mem_free(struct file *fp, uint32_t cmd, unsigned long arg)
{
    unsigned long ret = 0;
    dev_mem_info_t user_mem_info = {0};

    if( fp == NULL )
    {
        mm_err("%s:%d Invalid file descriptor\n",__func__,__LINE__);
        return -EIO;
    }
    if( fp->private_data == NULL)
    {
        mm_err("%s:%d Invalid file private data\n",__func__,__LINE__);
        return -EIO;
    }
    ret = copy_from_user(&user_mem_info,
                             (dev_mem_info_t *)arg,
                             sizeof(dev_mem_info_t));
    if (ret)
    {
        mm_err("%s:%d dev_mem_free: copy_from_user failed, ret=%lu\n",
                __func__,__LINE__,ret);
                return -EIO;
    }
    return userMemFree(fp, user_mem_info.phy_addr);
}

static int
dev_release_pid(struct file *fp, uint32_t cmd, unsigned long arg)
{
    return userMemFreeSlabs(fp);
}
static int
dev_get_user_page(struct file *fp, uint32_t cmd, unsigned long arg)
{
    unsigned long ret;
    struct page *page;
    int errno = 0;
    user_page_info_t user_mem_info = {0};

    if( fp == NULL )
    {
        mm_err("%s:%d Invalid file descriptor\n",__func__,__LINE__);
        return -EIO;
    }
    if( fp->private_data == NULL)
    {
        mm_err("%s:%d Invalid file private data\n",__func__,__LINE__);
        return -EIO;
    }
    ret = copy_from_user(&user_mem_info, (user_page_info_t *)arg,
                sizeof(user_page_info_t));
    if (ret)
    {
        mm_err("%s:%d dev_get_user_page: copy_from_user failed, ret=%lu\n",
                __func__,__LINE__,ret);
                return -EIO;
    }

    errno = get_user_pages_fast(
           (unsigned long)user_mem_info.virt_addr, 1, 1, &page);
    if ( errno != 1 )
    {
       user_mem_info.phy_addr = 0x00;
       mm_err("%s:%d dev_get_user_page: get_user_pages_fast failed, ret=%d\n",
                __func__,__LINE__,errno);
       return -EIO;
    }
    else
    {
       if (PageHuge(page))
       {
           user_mem_info.phy_addr = page_to_phys(page);
       }
       else
       {
           user_mem_info.phy_addr = 0x00;
       }

    }
    put_page(page);

    ret = copy_to_user( (user_page_info_t *)arg, &user_mem_info,
                sizeof(user_page_info_t));
    if (ret)
    {
        mm_err("%s:%d dev_get_user_page: copy_to_user failed, ret=%lu\n",
                __func__,__LINE__,ret);
                return -EIO;
    }
    return 0;
}

static int
dev_num_hp_get(struct file *fp, uint32_t cmd, unsigned long arg)
{
    unsigned long ret = 0;
    uint actual_num_hugepages = 0;

    if( fp == NULL )
    {
        mm_err("%s:%d Invalid file descriptor\n",__func__,__LINE__);
        return -EIO;
    }
    if( fp->private_data == NULL)
    {
        mm_err("%s:%d Invalid file private data\n",__func__,__LINE__);
        return -EIO;
    }
    actual_num_hugepages = min(max_huge_pages, max_huge_pages_per_process);
    ret = copy_to_user((uint32_t *)arg,
                       &actual_num_hugepages,
                       sizeof(uint32_t));
    if (ret)
    {
        mm_err("%s:%d dev_num_hp_get: copy_to_user failed, ret=%lu\n",
                __func__,__LINE__,ret);
                return -EIO;
    }
    max_huge_pages -= actual_num_hugepages;
    ((user_proc_mem_list_t*)fp->private_data)->hugepages_nr =
        actual_num_hugepages;
#ifdef ICP_DEBUG
    mm_info("[ALLOC] pid: %u max_huge_pages: %u actual_num_hugepages: %u\n",
           current->pid,
           max_huge_pages,
           actual_num_hugepages);
#endif
    return 0;
}

static long
mem_ioctl(struct file *fp, uint32_t cmd, unsigned long arg)
{
    int ret = 0;
    switch(cmd) {
        case DEV_MEM_IOC_MEMALLOC:
            mutex_lock(&dev_mem_lock);
            ret = dev_mem_alloc(fp, cmd, arg);
            mutex_unlock(&dev_mem_lock);
            if (ret)
            {
                return -ENOMEM;
            }
            break;

        case DEV_MEM_IOC_MEMFREE:
            mutex_lock(&dev_mem_lock);
            ret = dev_mem_free(fp, cmd, arg);
            mutex_unlock(&dev_mem_lock);
            if (unlikely(ret))
            {
               return -EIO;
            }
            break;

        case DEV_MEM_IOC_RELEASE:
             mutex_lock(&dev_mem_lock);
             ret = dev_release_pid(fp, cmd, arg);
             mutex_unlock(&dev_mem_lock);
             if (unlikely(ret))
             {
                return -EIO;
             }
             break;

        case DEV_MEM_IOC_GET_NUM_HPT:
             mutex_lock(&dev_mem_lock);
             ret = dev_num_hp_get(fp, cmd, arg);
             mutex_unlock(&dev_mem_lock);
             if (unlikely(ret))
             {
                return -EIO;
             }
             break;

        case DEV_MEM_IOC_GET_USER_PAGE:
             mutex_lock(&dev_mem_lock);
             ret = dev_get_user_page(fp, cmd, arg);
             mutex_unlock(&dev_mem_lock);
             if (unlikely(ret))
             {
                return -EIO;
             }
             break;

        default:
             ret = handle_other_ioctls(cmd);
             return ret;
    }
    return 0;
}

static int cmd_mmap_access(struct vm_area_struct *vma,
        unsigned long addr, void *buf, int len, int write)
{
    int size = vma->vm_end - addr;
    unsigned long offs = addr - vma->vm_start;
    unsigned long phy_addr = vma->vm_pgoff << PAGE_SHIFT;
    void *virt_addr = phys_to_virt(phy_addr);

    len = min(len, size);

    if (write)
        memcpy(virt_addr + offs, buf, len);
    else
        memcpy(buf, virt_addr + offs, len);

    return len;
}

static struct vm_operations_struct cmd_mmap_operations = {
    .access = cmd_mmap_access,
};

static int
mem_mmap(struct file *fp, struct vm_area_struct *vma)
{
    int ret = 0;
    uint64_t id = 0;
    unsigned long phys_kmalloc_area = 0;
    kdev_mem_info_t *kmem = NULL;
    unsigned long size = vma->vm_end - vma->vm_start;
    id = vma->vm_pgoff << PAGE_SHIFT;

    mutex_lock(&dev_mem_lock);
    kmem = userMemGetInfo(fp, id);
    if (!kmem)
    {
        mutex_unlock(&dev_mem_lock);
        mm_err("%s:%d cannot find meminfo\n",__func__,__LINE__);
        return -ENOMEM;
    }

    /* Ensure memory mapping does not exceed the allocated memory region */
    if (size > kmem->size)
    {
        mutex_unlock(&dev_mem_lock);
        mm_err("%s:%d cannot map allocated memory region\n",
               __func__, __LINE__);
        return -ENOMEM;
    }

    /* There is an agreement that mmap(PAGE_SIZE) means control block. */
    if (PAGE_SIZE == size)
    {
        phys_kmalloc_area = virt_to_phys(kmem->huge_mem_ctrl);
    }
    /* Any other size means memory block. */
    else
    {
        phys_kmalloc_area = virt_to_phys(kmem->kmalloc_ptr);
    }
    mutex_unlock(&dev_mem_lock);

    vma->vm_ops = &cmd_mmap_operations;
    ret = remap_pfn_range(vma,
                          vma->vm_start,
                          phys_kmalloc_area >> PAGE_SHIFT,
                          size,
                          vma->vm_page_prot);
    if (unlikely(ret))
    {
        mm_err("%s:%d remap_pfn_range failed, ret = %d\n",
                __func__,__LINE__,ret);
    }
    return ret;
}
static int
mem_open(struct inode *inp, struct file *fp)
{
    user_proc_mem_list_t *list = NULL;
    mutex_lock(&dev_mem_lock);
    if (!fp->private_data)
    {
        list = kzalloc(sizeof(user_proc_mem_list_t), GFP_KERNEL);
        if(!list)
        {
            mm_err("%s:%d memory allocation failed\n",
                    __func__,__LINE__);
            mutex_unlock(&dev_mem_lock);
            return -ENODEV;
        }
        fp->private_data = list;
        ADD_ELEMENT_TO_END_LIST(list, mem_dev_numa->head,
                            mem_dev_numa->tail, );
        list->pid = current->tgid;
    }
    mutex_unlock(&dev_mem_lock);
    return 0;
}

static inline void remove_element(user_proc_mem_list_t * p)
{
    if (NULL == p)
        return;
    if (NULL != p->pPrev) {
        p->pPrev->pNext = p->pNext;
    }
    if (NULL != p->pNext) {
        p->pNext->pPrev = p->pPrev;
    }
}

static int
mem_release(struct inode *inp, struct file *fp)
{
    user_proc_mem_list_t *list = NULL;
    mutex_lock(&dev_mem_lock);
    list=(user_proc_mem_list_t *)fp->private_data;
    if( list )
    {
        (void)userMemFreeSlabs(fp);
        if (NULL != mem_dev_numa)
        {
            REMOVE_ELEMENT_FROM_LIST(list,
                    mem_dev_numa->head, mem_dev_numa->tail, );
        }
        else
        {
            remove_element(list);
        }
        FREE(list);
        fp->private_data=NULL;
    }
    mutex_unlock(&dev_mem_lock);
    return 0;
}

static struct file_operations mem_ops = {
        owner:THIS_MODULE,
        mmap:mem_mmap,
        unlocked_ioctl:mem_ioctl,
        compat_ioctl:mem_ioctl,
        open:mem_open,
        release:mem_release,
};

static chr_drv_info_t mem_drv_info = {
        major:0,
        min_minor:DEV_MEM_BASE_MINOR,
        max_minor:DEV_MEM_MAX_MINOR,
        name:DEV_MEM_NAME,
};

static int32_t
chr_drv_create_class(chr_drv_info_t* drv_info)
{
    QAE_LOCAL_ENSURE(drv_info,
                     "chr_drv_create_class(): Invalid parameter value ",
                     -EINVAL);

    drv_info->drv_class = class_create(THIS_MODULE, drv_info->name);
    if (IS_ERR(drv_info->drv_class))
    {
        mm_err("%s:%d class_create failed\n",__func__,__LINE__);
        return -ENODEV;
    }
    return 0;
}

static void
chr_drv_destroy_class(chr_drv_info_t* drv_info)
{
     if (NULL == drv_info)
     {
         mm_err("%s:%d Invalid parameter value\n",__func__,__LINE__);
         return;
     };
     class_destroy( drv_info->drv_class );
     drv_info->drv_class = NULL;
     return;
}

static inline void
chr_drv_destroy_device(chr_drv_info_t *drv_info)
{
    if (NULL == drv_info)
    {
        mm_err("%s:%d Invalid parameter value\n",__func__,__LINE__);
        return;
    }
    if (NULL != drv_info->drv_class_dev)
    {
        device_destroy(drv_info->drv_class, MKDEV(drv_info->major,
                               DEV_MEM_BASE_MINOR));
    }
    cdev_del(&(drv_info->drv_cdev));
    unregister_chrdev_region( MKDEV(drv_info->major, DEV_MEM_BASE_MINOR),
                                  drv_info->max_minor);
    return;
}

static int
chr_drv_create_device(chr_drv_info_t *drv_info)
{
    int ret = 0;
    dev_t devid = 0;

    QAE_LOCAL_ENSURE(drv_info,
                     "chr_drv_create_device(): Invalid parameter value ",
                     -ENODEV);
    ret = alloc_chrdev_region(&devid,
                                  drv_info->min_minor,
                                  drv_info->max_minor,
                                  drv_info->name);
    if (unlikely(ret))
    {
        mm_err("%s:%d Unable to allocate chrdev region\n",
                __func__,__LINE__);
                return -ENOMEM;
    }
    drv_info->major = MAJOR(devid);
    drv_info->drv_cdev.owner=THIS_MODULE;
    cdev_init(&(drv_info->drv_cdev), &mem_ops);
    ret = cdev_add(&(drv_info->drv_cdev), devid, drv_info->max_minor);
    if (unlikely(ret))
    {
        mm_err("%s:%d cdev add failed\n",__func__,__LINE__);
        chr_drv_destroy_device(drv_info);
        return -ENOENT;
    }
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
    drv_info->drv_class_dev = device_create(drv_info->drv_class,
                    NULL, MKDEV(drv_info->major, DEV_MEM_BASE_MINOR),
                    NULL, "%s", drv_info->name);
#else
    drv_info->drv_class_dev = device_create(drv_info->drv_class,
                   NULL, MKDEV(drv_info->major, DEV_MEM_BASE_MINOR),
                   drv_info->name);
#endif
    if( NULL == drv_info->drv_class_dev )
    {
        mm_err("%s:%d device_create failed\n",__func__,__LINE__);
        chr_drv_destroy_device(drv_info);
        return -ENOMEM;
    }
    return 0;
}

static int32_t register_mem_device_driver(void)
{
    int ret = 0;
    mutex_init(&dev_mem_lock);
    mem_dev_numa = kzalloc(sizeof(user_mem_dev_t), GFP_KERNEL);
    if(!mem_dev_numa)
    {
        mm_err("failed to allocate memory for numa mem device\n");
        return -ENOMEM;
    }
    ret = chr_drv_create_class(&mem_drv_info);
    if(unlikely(ret))
    {
        mm_err("failed to create device driver class\n");
        FREE(mem_dev_numa);
        return -ENODEV;
    }
    ret = chr_drv_create_device(&mem_drv_info);
    if(unlikely(ret))
    {
        mm_err("failed to create mem numa device driver\n");
        chr_drv_destroy_class(&mem_drv_info);
        FREE(mem_dev_numa);
        return -ENODEV;
    }
    mem_drv_info.unregistered = 0;
    return 0;
}
/*
 * unregister the device driver
 */
static void unregister_mem_device_driver(void)
{
    if(!mem_drv_info.unregistered)
    {
        chr_drv_destroy_device(&mem_drv_info);
        chr_drv_destroy_class(&mem_drv_info);
        FREE(mem_dev_numa);
        mem_dev_numa = NULL;
        mem_drv_info.unregistered = 1;
    }
}

static inline char printable(char sym)
{
    if (sym >= 0x20 && sym <= 0x7E)
        /*check if printable ascii*/
        return sym;
    else
        /*else put out a dot*/
        return '.';
}

static char qae_dbg_ascii[128];
static char qae_dbg_command[128];
static char qae_dbg_slab_data[4096];
/*dumps memory data in 16 8 hex bytes and 8 ascii chars columns and 32 rows*/
static int
dumpData(void *start, void *end)
{
    int row = 0;
    int col = 0;
    char *src = start;
    char *endaddr = end;
    size_t offs = 0;
    const int ROWS = 32;
    const int COLUMNS = 8;

    for (row = 0; row < ROWS; ++row)
    {
        size_t ascii = 0;

        for (col = 0; col < COLUMNS; ++col)
        {
            if (src > endaddr)
            {
                offs += scnprintf(qae_dbg_slab_data + offs,
                        sizeof(qae_dbg_slab_data) - offs, "   ");
                ascii += scnprintf(qae_dbg_ascii + ascii,
                        sizeof(qae_dbg_ascii) - ascii, "  ");
            }
            else
            {
                /*in the first 8 columns print bytes in hex with 2 nibbles*/
                offs += scnprintf(qae_dbg_slab_data + offs,
                        sizeof(qae_dbg_slab_data) - offs, "%02hhx ", *src);
                /*in the last 8 columns print ascii char or dot*/
                ascii += scnprintf(qae_dbg_ascii + ascii,
                        sizeof(qae_dbg_ascii) - ascii, "%c ", printable(*src));
                src++;
            }
        }
        offs += scnprintf(qae_dbg_slab_data + offs,
                sizeof(qae_dbg_slab_data) - offs, "%.128s\n", qae_dbg_ascii);
        if (src > endaddr)
            return offs;
    }
    return offs;
}
/*
 * findSlabsForPid - find the link list of slabs for a given pid
 */
static kdev_mem_info_t*
findSlabsForPid(const uint64_t pid)
{
    if (mem_dev_numa)
    {
        user_proc_mem_list_t *list = mem_dev_numa->head;
        while (list)
        {
            if (list->pid == pid )
                return list->head;
            list=list->pNext;
        }
    }
    return NULL;
}
/*
 * execute dump command
 * returns length of data in output buffer
 */
static int
execDump(kdev_mem_info_t* slab, const uintptr_t param, const uint64_t pid)
{
    uintptr_t endaddr = 0;
    uintptr_t startaddr = param;
    uintptr_t offset = 0;
    size_t len = 0;

    mm_info("Process dump command \n");
    /* traverse thru slabs */
    while (slab)
    {
        uintptr_t phy_addr = (uintptr_t) slab->phy_addr;
        uintptr_t virt_addr = (uintptr_t) slab->kmalloc_ptr;
        /*calculate virtual address end of slab*/
        endaddr = virt_addr + slab->size;
        /*check if this slab was sought after by virtual address*/
        if (startaddr >= virt_addr && startaddr < endaddr)
        {
            offset = startaddr - virt_addr;
            mm_info("Block found: "
                    "start %p block end %p dump addr %p offset %p\n",
                    (void *) virt_addr, (void *) endaddr,
                    (void *) startaddr, (void *) offset);
            break;
        }
        /*calculate physical address end of slab*/
        endaddr = phy_addr + slab->size;
        /*check if this slab was sought after by phy address*/
        if (startaddr >= phy_addr && startaddr < endaddr)
        {
            offset = startaddr - phy_addr;
            mm_info("Block found (using phy_addr): "
                    "start %p block end %p dump addr %p offset %p\n",
                    (void *) phy_addr, (void *) endaddr,
                    (void *) startaddr, (void *) offset);
            break;
        }
       /* take next slab if no hit */
        slab = slab->pNext_kernel;
    }
    /* log slab not found */
    if( !slab )
    {
        len = scnprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                "Slab not found PID %llu Address %p\n",
                pid, (void *) startaddr);
    }
    else /*dump 256 byte of slab data */
    {
        startaddr = (uintptr_t) slab + offset;
        endaddr = (uintptr_t) slab +
                       slab->size - 1;
        len = dumpData((void *) startaddr, (void *) endaddr);
    }
    return len;
}
/*
 * execute dump control area command
 * returns length of data in output buffer
 */
static int32_t
execDumpControl(kdev_mem_info_t* slab, const uintptr_t param, const uint64_t pid)
{
    uint64_t id = param;
    uintptr_t endaddr = 0;
    size_t len = 0;

    /*traverse thru slabs search by slab id*/
    while(slab)
    {
        endaddr = slab->phy_addr + slab->size;
        if (id >= slab->phy_addr && id < endaddr)
        {
            break;
        }
        slab = slab->pNext_kernel;
    }
    if( !slab ) /* log slab not found*/
    {
        len = scnprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                "Slab not found PID %llu slab ID %llu\n", pid, id);
    }
    else /*dump bitmap*/
    {
        int row;
        uint64_t bitmap_row,mask;
        /* banner message */
        len = scnprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                "Small buffer allocation bitmap \n Slab id %llu \n", id);
        /* display 0/1 in bitmap positions throughout the bitmap */
        for ( row = 0; row < BITMAP_LEN; ++row )
        {
            /* The slab does not contain any bitmap information anymore.
             * We must now access with kmalloc_ptr */
            bitmap_row = ((block_ctrl_t*)slab->kmalloc_ptr)->bitmap[row];
            for ( mask = 1ULL<<(QWORD_WIDTH-1); mask; mask>>=1)
            {
                char bit = '0';
                if ( mask & bitmap_row )
                {
                    bit = '1';
                }
                len += scnprintf(qae_dbg_slab_data + len,
                    sizeof(qae_dbg_slab_data) - len, "%c", bit);
            }
            len += scnprintf(qae_dbg_slab_data + len,
                    sizeof(qae_dbg_slab_data) - len, "\n");
        }
    }
    return len;
}
/* processCommand
 * performs the command found in the command buffer
 * returns the number of characters the debug
 * buffer has after command was executed
 */
static int
processCommand(void)
{
    char *arg = NULL;
    char *cmd = NULL;
    char command = '\0'; /*command char c/d*/
    uint64_t param = 0; /*command parameter*/
    uint64_t pid = 0; /*process id*/
    kdev_mem_info_t* slab = NULL; /*slab the info is required for*/
    size_t len = 0; /*length of string in output buffer*/

    command = qae_dbg_command[0];
    if ('\0' == command) /*check if there is a command*/
    {
        return 0;
    }
    /* Search for a first numeric argument after the command itself. */
    cmd = strpbrk(qae_dbg_command, "0123456789");
    arg = strsep(&cmd, " ");
    if (NULL != arg) {
        int status = kstrtoll(arg, 0, &pid);
        pid *= (status == 0);

        /* Find a next argument. */
        arg = strsep(&cmd, " ");
        if (NULL != arg)
        {
            status = kstrtoll(arg, 0, &param);
            param *= (status == 0);
        }
    }
    mm_info("%s:%d "
            "Command %c Param %llu %llu Buffer %s Arg %s\n",
            __func__, __LINE__, command, pid, param, qae_dbg_command, arg);
    /* Destroy the original command. */
    qae_dbg_command[0] = '\0';

    switch (command)
    {
        case 'd':
            slab = findSlabsForPid(pid); /* find slab for process id*/
            if(!slab)
            {
                mm_info("%s:%d "
                        "Could not find slab for process id: %llu\n",
                        __func__,__LINE__,pid);
                return 0;
            }
            /*dump memory content*/
            len = execDump(slab,param,pid);
            break;
        case 'c':
            slab = findSlabsForPid(pid); /* find slab for process id*/
            if(!slab)
            {
                mm_info("%s:%d "
                        "Could not find slab for process id: %llu\n",
                        __func__,__LINE__,pid);
                return 0;
            }
            /* control block data (bitmap) */
            len = execDumpControl(slab,param,pid);
            break;
        case 't':
            /* print total allocated NUMA memory */
            len = scnprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                    "Total allocated NUMA memory: %zu bytes\n",
                    mem_allocated);
            break;
        default:
            len = scnprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                    "Invalid command %c\n", command);
            break;
    }
    return len;
}
/* print info about a slab in debug buffer
 * return number of byte in buffer
 * 0 return value will end the file read operation
 * each time this function is called one slab data
 * is entered in the debug buffer
 * process and slab ptrs are saved in static variables
 * to traverse the linked list by file read until a 0
 * return value is received.
 */
static int
getMemInfo(user_proc_mem_list_t** pmem_list)
{
    /*memory info for slab in memory list*/
    static kdev_mem_info_t* mem_info;
    /*memory list element of current process*/
    user_proc_mem_list_t* mem_list = *pmem_list;
    int length = 0;
    /*initialise list of processes that allocated slabs*/
    if (!mem_info && !mem_list )
    {
        mem_list = mem_dev_numa->head;
        /*return if list is empty*/
        if ( !mem_list)
           return 0;
        mem_info = mem_list->head;
    }
    /* iterate through all processes in the list*/
    while(mem_list)
    {
        /*check if there is a valid slab entry*/
        if(mem_info)
        {
            length = scnprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                    "Pid %d, Slab Id %llu \n"
                    "Virtual address %p, Physical Address %llx, Size %lld\n",
                    mem_list->pid, mem_info->phy_addr, mem_info->kmalloc_ptr,
                    mem_info->phy_addr, mem_info->size);
            /*advance slab pointer for next call*/
            mem_info = mem_info->pNext_kernel;
            /*send slab info into read buffer*/
            break;
        }
        else
        {
            /* null slab ptr in list of previous process
             * get next process from list*/
            mem_list = mem_list->pNext;
            /*get first slab from next list element*/
            if(mem_list)
                mem_info = mem_list->head;
        }
    }
    /* if at the end of process list chain*/
    if(!mem_list)
    {
        mem_list = mem_dev_numa->head;
        mem_info = NULL;
    }
    /* save current process in list in a static for next call*/
    *pmem_list = mem_list;
    return length;
}
/*
*qae_mem_update_slab_data
* updates data in debug buffer depending on last command
* open - non-null if called from debug file open routine
*       otherwise 0
*/
static int
qae_mem_update_slab_data(int open)
{
    /* memory list of current process*/
    static user_proc_mem_list_t* mem_list;
    static int count; /*number of chars in debug buffer*/
    if( !mem_dev_numa )
        return 0;
    /* if file just opened initialise; make sure
     * list of slabs are generated from the top
     * if qae_dbg_command buffer is empty */
    if(open)
    {
        mem_list = NULL;
        count = 0;
        return 0;
    }
    /* last time a buffer with chars were sent in response to read operation
       return 0 now to complete read operation.*/
    if(count)
    {
        count = 0;
        return 0;
    }
    /* process command and report to read op if there is any result*/
    count = processCommand();
    if(count)
        return count;
    /*get next slab info into debug data buffer*/
    /* when 0 is returned it marks the end of buffer list*/
    /* and will end the file read operation as well*/
    return getMemInfo(&mem_list);
}
/*read function for debug file
 returns number of bytes read
 read operation completes when 0 is returned here*/
static ssize_t
qae_mem_slabs_data_read(struct file* filp, char __user *buffer,
        size_t count, loff_t * pos)
{
    /*update data in debug buffer*/
    int data_len = qae_mem_update_slab_data(false);
    /*check length and position */
    if( 0 == data_len || *pos >= data_len )
       return 0;
    /* Ensure the addition of (*pos + count) does not overflow */
    if ((*pos + count) > ULLONG_MAX)
        return 0;
    if( *pos + count > data_len )
        count = data_len - *pos;
    /*copy from kernel buffer to user*/
    if( copy_to_user(buffer ,qae_dbg_slab_data  + *pos,
            (unsigned)count))
        return -EFAULT;
    return count;
}
/*write function for write operation of the debug file*/
static ssize_t
qae_mem_slabs_data_write (struct file *filp,
                                          const char __user *buffer,
                                          size_t count, loff_t *pos)
{
    /*write command to qae_dbg_command buffer
     *next read on debug file will parse the command string
     *and execute the requested command
     *if command buffer empty the next read
     *lists the allocated slabs */
    /* check count vs size of command buffer*/
    if (count >= sizeof(qae_dbg_command) )
    {
        return -EFAULT;
    }
    /* copy command string from user buffer*/
    if ( copy_from_user(qae_dbg_command, buffer, count) )
    {
        return -EFAULT;
    }
    /*terminating 0*/
    qae_dbg_command[count] = '\0';
    return count;
}
/*called when debug file is opened
 used for initialisation */
static int
qae_mem_slabs_data_open(struct inode *inode, struct file* filep)
{
    qae_mem_update_slab_data(1);
    return 0;
}
static struct file_operations qae_mem_slabs_file_fops = {
    .owner = THIS_MODULE,
    .open = qae_mem_slabs_data_open,
    .read  = qae_mem_slabs_data_read,
    .write = qae_mem_slabs_data_write
};
/*
 * Initialisation function to insmod device driver
 */
static inline void
qae_debug_init(void)
{
    if ( ( qae_dbg_root_dir = debugfs_create_dir("qae_mem_dbg", NULL) )
            == ERR_PTR(-ENODEV) ||
         ( qae_dbg_slabs_file = debugfs_create_file("qae_mem_slabs", 0666,
           qae_dbg_root_dir, NULL,
           &qae_mem_slabs_file_fops) ) == ERR_PTR(-ENODEV) )
    {
        mm_warning(
                "Debug FS not initialised, debug info not available\n");
    }
}

static int
qae_mem_init( void )
{
    mm_info("Loading %s Module %s ...\n", MODULE_NAME, VERSION_STRING);
    mm_info("IOCTLs: %lx, %lx, %lx, %lx\n",
            (unsigned long)DEV_MEM_IOC_MEMALLOC,
            (unsigned long)DEV_MEM_IOC_MEMFREE,
            (unsigned long)DEV_MEM_IOC_RELEASE,
            (unsigned long)DEV_MEM_IOC_GET_NUM_HPT);
    if(register_mem_device_driver())
    {
        mm_err("Error loading %s Module\n", MODULE_NAME);
        return -1;
    }
    qae_debug_init();
    return 0;
}
/*
 * tear down function to rmmod device driver
 */
STATIC void
qae_mem_exit( void )
{
    mm_info("Unloading %s Module %s...\n", MODULE_NAME, VERSION_STRING);
    unregister_mem_device_driver();
    if( NULL != qae_dbg_root_dir )
    {
        debugfs_remove_recursive(qae_dbg_root_dir);
        qae_dbg_root_dir = NULL;
    }
}
module_init(qae_mem_init);
module_exit(qae_mem_exit);


MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("User Space DMA-able Memory Driver");


