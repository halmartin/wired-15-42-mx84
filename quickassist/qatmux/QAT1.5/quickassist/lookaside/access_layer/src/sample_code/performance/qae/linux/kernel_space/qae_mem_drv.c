/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
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
 *   BSD LICENSE 
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
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
 *  version: QAT1.5.L.1.11.0-36
 *
 ***************************************************************************/

/**
 *
 * @file qae_mem_drv.c
 *
 * @brief Kernel-space support for user-space contiguous memory allocation
 *
 */


#include "qae_mem.h"
#ifdef SAL_IOMMU_CODE
#include <icp_sal_iommu.h>
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/slab.h>


#include <asm/uaccess.h>
#include <linux/string.h>
#include <asm/io.h>


#include <linux/sched.h>

/**
 *****************************************************************************
 * @description
 *      This structure contains data relating to the device driver that this
 *      file implements
 *
 ****************************************************************************/
typedef struct chr_drv_info_s {
    struct module          *owner;
    unsigned               major;
    unsigned               min_minor;
    unsigned               max_minor;
    char                   *name;
    struct file_operations *file_ops;
    struct cdev            drv_cdev;
    struct class           *drv_class;
    struct device          *drv_class_dev;
} chr_drv_info_t;



#define DEV_MEM_NAME            "qae_mem"
#define DEV_MEM_MAJOR           0
#define DEV_MEM_MAX_MINOR       1
#define DEV_MEM_BASE_MINOR      0
#define FAIL                    1
#define SUCCESS                 0
#define FREE(ptr) kfree(ptr)

static DEFINE_MUTEX(dev_mem_lock_g);
static user_mem_dev_t *mem_dev_numa = NULL;


/*
 * Find memory information
 */
qae_dev_mem_info_t *userMemGetInfo(user_mem_dev_t* dev, unsigned int id, unsigned int pid)
{
    user_proc_mem_list_t *list = NULL;
    qae_dev_mem_info_t *mem = NULL;

    list = dev->head;
    while(list)
    {
        if(pid == list->pid)
        {
            mem = list->head;
            while(mem)
            {
                if(mem->id == id)
                {
                    return mem;
                }
                mem = mem->pNext;
            }
        }
        list = list->pNext;
    }
    return NULL;
}

/*
 * Allocate numa memory
 */
qae_dev_mem_info_t *userMemAlloc(user_mem_dev_t* dev, unsigned int sizeInBytes,
        unsigned int node, unsigned int pid)
{
    qae_dev_mem_info_t *mem_info = NULL;
    user_proc_mem_list_t *list = NULL;

    if (0 == sizeInBytes)
    {
        printk("%s:%d Invalid parameter value [%u]\n",
                __func__, __LINE__,sizeInBytes);
        return NULL;
    }

    /*
     * Find the process allocation list
     */
    list = dev->head;
    while(list)
    {
        if(pid == list->pid)
        {
            break;
        }
        list = list->pNext;
    }
    ENSURE_NOT_NULL(list, "userMemAlloc(): find pid in the list failed",
            NULL);
    /*
     * Check if we have some free slots
     */
    if(list->allocs_nr == list->max_id)
    {
        /* Allocate one if we dont have a free slot */
        mem_info = kmalloc_node(sizeof(qae_dev_mem_info_t), GFP_KERNEL, node);
        ENSURE_NOT_NULL(mem_info,
                "userMemAlloc(): allocation failed ",
                NULL);
        memset(mem_info, '\0', sizeof(qae_dev_mem_info_t));
        mem_info->id = list->max_id++;
    }
    else
    {
        /* Reuse a free slot */
        mem_info = list->freed_tail;
        REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail, list->freed_head);
    }
    mem_info->size = sizeInBytes;
    mem_info->kmalloc_ptr = kmalloc_node(sizeInBytes, GFP_KERNEL, node);
    if (NULL == mem_info->kmalloc_ptr)
    {
        printk("%s:%d Unable to allocate memory mem_info->kmalloc_ptr\n",
                __func__,__LINE__);
        ADD_ELEMENT_TO_END_OF_LIST(mem_info, list->freed_tail,
                list->freed_head);
        return NULL;
    }

    mem_info->kmalloc_area = ((int *)
        ((((unsigned long)mem_info->kmalloc_ptr) + PAGE_SIZE -1) & PAGE_MASK));
    mem_info->phy_addr = virt_to_phys(mem_info->kmalloc_ptr);
#ifdef SAL_IOMMU_CODE
    icp_sal_iommu_map(mem_info->phy_addr, mem_info->phy_addr, mem_info->size);
#endif
    list->allocs_nr++;
    ADD_ELEMENT_TO_END_OF_LIST(mem_info, list->tail, list->head);
    return mem_info;
}


/*
 * Free memory
 */
void userMemFree(user_mem_dev_t* dev, unsigned int id, unsigned int pid)
{
    qae_dev_mem_info_t *mem_info = NULL;
    qae_dev_mem_info_t *freed_info = NULL;
    user_proc_mem_list_t *list = NULL;

    list = dev->head;
    while(list)
    {
        if(pid == list->pid)
        {
            mem_info = list->head;
            while(mem_info)
            {
                if(mem_info->id == id)
                {
#ifdef SAL_IOMMU_CODE
                    icp_sal_iommu_unmap(mem_info->phy_addr, mem_info->size);
#endif
                    FREE(mem_info->kmalloc_ptr);
                    mem_info->kmalloc_ptr = NULL;
                    mem_info->size = 0;
                    REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
                    list->allocs_nr--;
                    if(mem_info->id != list->max_id)
                    {
                        ADD_ELEMENT_TO_END_OF_LIST(mem_info,
                                list->freed_tail, list->freed_head);
                    }
                    else
                    {
                        FREE(mem_info);
                        list->max_id--;
                        freed_info = list->freed_tail;
                        while( freed_info->id == list->max_id )
                        {
                            qae_dev_mem_info_t *ptr = freed_info->pPrev;
                            list->max_id--;
                            FREE(freed_info);
                            freed_info = ptr;
                        }
                    }
                    return;
                }
                mem_info = mem_info->pNext;
            }
        }
        list = list->pNext;
    }
    printk("Could not find id: %d pid: %d\n", id, pid);
}

/*
 * Free memory associated with PID
 */
void userMemFreeAllPid(user_mem_dev_t* dev, unsigned int pid)
{
    qae_dev_mem_info_t *mem_info = NULL, *next = NULL;
    user_proc_mem_list_t *list = NULL, *l_next = NULL;

    list = dev->head;
    while(list)
    {
        l_next = list->pNext;
        if(list->pid == pid)
        {
            mem_info = list->head;
            while(mem_info)
            {
                next = mem_info->pNext;
#ifdef SAL_IOMMU_CODE
                icp_sal_iommu_unmap(mem_info->phy_addr, mem_info->size);
#endif
                FREE(mem_info->kmalloc_ptr);
                REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
                FREE(mem_info);
                mem_info = next;
            }
            mem_info = list->freed_head;
            while(mem_info)
            {
                next = mem_info->pNext;
                REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail,
                        list->freed_head);
                FREE(mem_info);
                mem_info = next;
            }
            REMOVE_ELEMENT_FROM_LIST(list, dev->tail, dev->head);
            FREE(list);
        }
        list = l_next;
    }
}

/*
 * Free all memory
 */
void userMemFreeAll(user_mem_dev_t* dev)
{
    qae_dev_mem_info_t *mem_info = NULL, *next = NULL;
    user_proc_mem_list_t *list = NULL, *l_next = NULL;

    list = dev->head;
    while(list)
    {
        l_next = list->pNext;
        mem_info = list->head;
        while(mem_info)
        {
            next = mem_info->pNext;
#ifdef SAL_IOMMU_CODE
            icp_sal_iommu_unmap(mem_info->phy_addr, mem_info->size);
#endif
            FREE(mem_info->kmalloc_ptr);
            REMOVE_ELEMENT_FROM_LIST(mem_info, list->tail, list->head);
            FREE(mem_info);
            mem_info = next;
        }
        mem_info = list->freed_head;
        while(mem_info)
        {
            next = mem_info->pNext;
            REMOVE_ELEMENT_FROM_LIST(mem_info, list->freed_tail,
                    list->freed_head);
            FREE(mem_info);
            mem_info = next;
        }

        REMOVE_ELEMENT_FROM_LIST(list, dev->tail, dev->head);
        FREE(list);
        list = l_next;
    }
}

/*
 * driver allocate memory function
 */
static int
dev_mem_alloc(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    qae_dev_mem_info_t *mem_info = NULL;
    qae_dev_mem_info_t user_mem_info = {{0}};

    ret = copy_from_user(&user_mem_info,
            (qae_dev_mem_info_t *)arg,
            sizeof(qae_dev_mem_info_t));

    if (ret != 0)
    {
        printk("%s:%d dev_mem_alloc: copy_from_user failed, ret=%d\n",
                __func__,__LINE__,ret);
        return -EIO;
    }
    mem_info = userMemAlloc(mem_dev_numa, user_mem_info.size,
            user_mem_info.nodeId, current->tgid);
    ENSURE_NOT_NULL(mem_info,
            "dev_mem_alloc(): userMemAlloc failed\n",
            -ENOMEM);

    ret = copy_to_user((qae_dev_mem_info_t *)arg,
            mem_info,
            sizeof(qae_dev_mem_info_t));

    if (ret != 0)
    {
        userMemFree(mem_dev_numa, mem_info->id, current->tgid);
        printk("%s:%d dev_mem_alloc: copy_to_user failed, ret=%d\n",
                __func__,__LINE__,ret);
        return -EIO;
    }

    return 0;
}


/*
 * driver free memory function
 */
static int
dev_mem_free(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    qae_dev_mem_info_t user_mem_info = {{0}};

    ret = copy_from_user(&user_mem_info,
            (qae_dev_mem_info_t *)arg,
            sizeof(qae_dev_mem_info_t));
    if (ret != 0)
    {
        printk("%s:%d dev_mem_free: copy_from_user failed, ret=%d\n",
                __func__,__LINE__,ret);
        return -EIO;
    }
    userMemFree(mem_dev_numa, user_mem_info.id, current->tgid);
    return 0;
}

/*
 * driver ioctl handling function
 */

static long
mem_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    switch(cmd) {
    case DEV_MEM_IOC_MEMALLOC:
        if (mutex_lock_interruptible(&dev_mem_lock_g) != 0)
        {
            return -ENOMEM;
        }

        ret = dev_mem_alloc(fp, cmd, arg);
        if (0 != ret)
        {
            mutex_unlock(&dev_mem_lock_g);
            return ret;
        }
        mutex_unlock(&dev_mem_lock_g);
        break;

    case DEV_MEM_IOC_MEMFREE:
        if (mutex_lock_interruptible(&dev_mem_lock_g) != 0)
        {
            return -EIO;
        }

        ret = dev_mem_free(fp, cmd, arg);
        if (0 != ret)
        {
            mutex_unlock(&dev_mem_lock_g);
            return ret;
        }
        mutex_unlock(&dev_mem_lock_g);
        break;
    default:
        printk("%s:%d Invalid IOCTL command specified(0x%x)\n",
                __func__,__LINE__,cmd);

        return -ENOTTY;
    }
    return 0;
}

/*
 * mmap for mapping kernel allocated memory to user space application
 */
static int
mem_mmap(struct file *fp, struct vm_area_struct *vma)
{
    int ret = 0, id = 0;
    unsigned long phys_kmalloc_area = 0;
    qae_dev_mem_info_t *mem_info = NULL;

    id = vma->vm_pgoff;
    if (mutex_lock_interruptible(&dev_mem_lock_g) != 0)
    {
        return -EIO;
    }
    mem_info = userMemGetInfo(mem_dev_numa, id, current->tgid);
    if(NULL == mem_info)
    {
        printk("userMemGerInfo failed\n");
        ret = -1;
    }
    else
    {
        mutex_unlock(&dev_mem_lock_g);
        phys_kmalloc_area =
            virt_to_phys((void*)((unsigned long)mem_info->kmalloc_area));

        ret = remap_pfn_range(vma,
                vma->vm_start,
                phys_kmalloc_area >> PAGE_SHIFT,
                vma->vm_end-vma->vm_start,
                vma->vm_page_prot);
        if (ret != 0)
        {
            printk("remap_pfn_range failed\n");
        }
    }
    return ret;
}


/*
 * driver open function
 */
static int
mem_open(struct inode *inp, struct file *fp)
{
    user_proc_mem_list_t *list = NULL;

    /*
     * Alloc process allocation list
     */
    list = kzalloc(sizeof(user_proc_mem_list_t), GFP_KERNEL);
    if(!list)
    {
        printk("%s:%d memory allocation failed\n",__func__,__LINE__);
        return -ENOMEM;
    }
    list->pid = current->tgid;
    if (mutex_lock_interruptible(&dev_mem_lock_g) != 0)
    {
        kfree(list);
        return -EIO;
    }
    ADD_ELEMENT_TO_END_OF_LIST(list, mem_dev_numa->tail, mem_dev_numa->head);
    mutex_unlock(&dev_mem_lock_g);
    return 0;
}

/*
 * driver close/release function
 */
static int
mem_release(struct inode *inp, struct file *fp)
{
    if(mutex_lock_interruptible(&dev_mem_lock_g) != 0)
    {
        return -EIO;
    }
    userMemFreeAllPid(mem_dev_numa, current->tgid);
    mutex_unlock(&dev_mem_lock_g);
    return 0;
}

/*
 * structure describing device function mappings
 */
static struct file_operations mem_ops = {
        owner:THIS_MODULE,
        mmap:mem_mmap,
        unlocked_ioctl:mem_ioctl,
#ifdef CONFIG_COMPAT
        compat_ioctl:mem_ioctl,
#endif
        open:mem_open,
        release:mem_release,
};


/*
 * instantiation of the driver
 */
static chr_drv_info_t mem_drv_info = {
        owner:THIS_MODULE,
        major:DEV_MEM_MAJOR,
        min_minor:DEV_MEM_BASE_MINOR,
        max_minor:DEV_MEM_MAX_MINOR,
        name:DEV_MEM_NAME,
        file_ops:&mem_ops,
};

/*
 * create the device driver class
 */
static int
chr_drv_create_class(chr_drv_info_t* drv_info)
{
    ENSURE_NOT_NULL(drv_info,
            "chr_drv_create_class(): parameter is NULL ",
            FAIL);

    drv_info->drv_class = class_create(THIS_MODULE, drv_info->name);
    if (IS_ERR(drv_info->drv_class))
    {
        printk("class_create failed\n");
        return FAIL;
    }
    return SUCCESS;
}

/*
 * destroy the device driver class
 */
static void
chr_drv_destroy_class(chr_drv_info_t* drv_info)
{
    ENSURE_NOT_NULL(drv_info,
            "chr_drv_destroy_class(): Invalid parameter value ",
    );
    class_destroy( drv_info->drv_class );
}

/*
 * destroy the device driver
 */
static void
chr_drv_destroy_device(chr_drv_info_t *drv_info)
{
    ENSURE_NOT_NULL(drv_info,
            "chr_drv_destroy(): Invalid parameter value ",);

    if (NULL != drv_info->drv_class_dev)
    {
        device_destroy(drv_info->drv_class, MKDEV(drv_info->major,
                drv_info->min_minor));
    }
    cdev_del(&(drv_info->drv_cdev));
    unregister_chrdev_region( MKDEV(drv_info->major, drv_info->min_minor),
            drv_info->max_minor);
}


/*
 * create the device driver
 */
static int
chr_drv_create_device(chr_drv_info_t *drv_info)
{
    int ret = 0;
    dev_t devid = 0;

    ENSURE_NOT_NULL(drv_info,
            "chr_drv_create_device(): NULL parameter value ",
            FAIL);

    ret = alloc_chrdev_region(&devid,
            drv_info->min_minor,
            drv_info->max_minor,
            drv_info->name);

    if (ret < 0)
    {
        printk("%s:%d unable to allocate chrdev region\n", __func__,__LINE__);
        return FAIL;
    }

    drv_info->major = MAJOR(devid);
    cdev_init(&(drv_info->drv_cdev), drv_info->file_ops);
    drv_info->drv_cdev.owner = drv_info->owner;

    ret = cdev_add(&(drv_info->drv_cdev), devid, drv_info->max_minor);
    if (ret < 0)
    {
        printk("%s:%d cdev add failed\n", __func__,__LINE__);
        chr_drv_destroy_device(drv_info);
        return FAIL;
    }

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26)
    drv_info->drv_class_dev = device_create(drv_info->drv_class,
            NULL, MKDEV(drv_info->major, drv_info->min_minor),
            drv_info->name);
#else
    drv_info->drv_class_dev = device_create(drv_info->drv_class,
            NULL, MKDEV(drv_info->major, drv_info->min_minor),
            NULL, drv_info->name);
#endif

    if( NULL == drv_info->drv_class_dev )
    {
        printk("%s:%d chr_drv_create_device: device_create failed\n",
                __func__,__LINE__);
        chr_drv_destroy_device(drv_info);
        return FAIL;
    }
    return SUCCESS;
}

/*
 * register the device driver
 */
int register_mem_device_driver(void)
{
    int ret = 0;

    mutex_init(&dev_mem_lock_g);
    mem_dev_numa = kzalloc(sizeof(user_mem_dev_t), GFP_KERNEL);
    if(!mem_dev_numa)
    {
        printk("%s:%d failed to allocate memory for numa mem device\n",
                __func__,__LINE__);
        return FAIL;
    }
    ret = chr_drv_create_class(&mem_drv_info);
    if (SUCCESS != ret)
    {
        printk("%s:%d failed to create device driver class\n",
                __func__,__LINE__);
        kfree(mem_dev_numa);
        return FAIL;
    }
    ret = chr_drv_create_device(&mem_drv_info);
    if (SUCCESS != ret)
    {
        printk("%s:%d failed to create mem numa device driver\n",
                __func__,__LINE__);
        chr_drv_destroy_class(&mem_drv_info);
        kfree(mem_dev_numa);
        return FAIL;
    }
    return SUCCESS;
}

/*
 * unregister the device driver
 */

void unregister_mem_device_driver(void)
{
    chr_drv_destroy_device(&mem_drv_info);
    chr_drv_destroy_class(&mem_drv_info);
    userMemFreeAll(mem_dev_numa);
    kfree(mem_dev_numa);
}

/*
 * Initialization function to insmod device driver
 */
int qae_mem_init( void )
{

    printk( "Loading QAE MEM Module ...\n" ) ;
    if(SUCCESS != register_mem_device_driver())
    {
        printk("Error loading QAE MEM Module\n");
        return FAIL;
    }
    return SUCCESS;
}

/*
 * tear down function to rmmod device driver
 */
void qae_mem_exit( void )
{
    printk("Unloading QAE MEM Module ...\n" ) ;
    unregister_mem_device_driver();
}

module_init(qae_mem_init);
module_exit(qae_mem_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("QAE Mem Utils");


