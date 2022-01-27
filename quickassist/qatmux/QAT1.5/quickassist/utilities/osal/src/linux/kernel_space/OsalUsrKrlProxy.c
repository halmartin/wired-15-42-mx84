/**
 *
 * @file kernel_space/OsalUsrKrlProxy.c
 *
 * @brief Kernel-space support for user-space contiguous memory allocation
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
#include "OsalDevDrv.h"
#include "OsalDevDrvCommon.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>
#include <linux/string.h>
#include <asm/io.h>

#define DEV_MEM_MAX_MINOR       1
#define DEV_MEM_BASE_MINOR      0

static DEFINE_MUTEX(dev_mem_lock);
static DEFINE_MUTEX(dev_mem_page_lock);
static user_mem_dev_t *mem_dev_numa = NULL;
static user_mem_dev_t *mem_dev_page = NULL;

static int
dev_mem_alloc(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    dev_mem_info_t *mem_info = NULL;
    dev_mem_info_t user_mem_info = {0};

    ret = copy_from_user(&user_mem_info,
                 (dev_mem_info_t *)arg,
                 sizeof(dev_mem_info_t));

    if (ret != 0) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_alloc: copy_from_user failed, ret=%d\n",
        ret, 0, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }
    mem_info = userMemAlloc(mem_dev_numa, user_mem_info.size,
                user_mem_info.nodeId, current->tgid);
    if (NULL == mem_info)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
             "dev_mem_alloc(): userMemAlloc failed\n",
             0, 0, 0, 0, 0, 0, 0, 0);
        return -ENOMEM;
    }

    ret = copy_to_user((dev_mem_info_t *)arg,
               mem_info,
               sizeof(dev_mem_info_t));

    if (ret != 0) {
        userMemFree(mem_dev_numa, mem_info->id, current->tgid);
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_alloc: copy_to_user failed, ret=%d\n",
        ret, 0, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }
    return OSAL_SUCCESS;
}

static int
dev_mem_alloc_page(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    dev_mem_info_t *mem_info = NULL;
    dev_mem_info_t user_mem_info = {0};

    ret = copy_from_user(&user_mem_info,
                 (dev_mem_info_t *)arg,
                 sizeof(dev_mem_info_t));

    if (ret != 0) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_alloc_page: copy_from_user failed, ret=%d\n",
        0, ret, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }

    mem_info = userMemAllocPage(mem_dev_page, user_mem_info.nodeId,
                    current->tgid);
    if (NULL == mem_info)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
             "dev_mem_alloc_page(): alloc_pages_node failed\n",
             0, 0, 0, 0, 0, 0, 0, 0);
        return -ENOMEM;
    }

    ret = copy_to_user((dev_mem_info_t *)arg,
               mem_info,
               sizeof(dev_mem_info_t));

    if (ret != 0) {
        userMemFreePage(mem_dev_page, mem_info->id, current->tgid);
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_alloc_page: copy_to_user failed, ret=%d\n",
        ret, 0, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }
    return OSAL_SUCCESS;
}

static int
dev_mem_free(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    dev_mem_info_t user_mem_info = {0};

    ret = copy_from_user(&user_mem_info,
                 (dev_mem_info_t *)arg,
                 sizeof(dev_mem_info_t));

    if (ret != 0) {
         osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_free: copy_from_user failed, ret=%d\n",
        ret, 0, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }
    userMemFree(mem_dev_numa, user_mem_info.id, current->tgid);
    return OSAL_SUCCESS;
}

static int
dev_mem_freepage(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    dev_mem_info_t user_mem_info = {0};

    ret = copy_from_user(&user_mem_info,
                 (dev_mem_info_t *)arg,
                 sizeof(dev_mem_info_t));

    if (ret != 0) {
         osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_free: copy_from_user failed, ret=%d\n",
        ret, 0, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }
    userMemFreePage(mem_dev_page, user_mem_info.id, current->tgid);
    return OSAL_SUCCESS;
}

static int
dev_mem_iommu_map(struct file *fp, unsigned long arg)
{
    int ret = 0;
    dev_iommu_info_t user_info = {0};

    ret = copy_from_user(&user_info,
                 (dev_iommu_info_t *)arg,
                 sizeof(dev_iommu_info_t));

    if (ret != 0) {
         osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_iommu_map: copy_from_user failed, ret=%d\n",
        ret, 0, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }
    return osalIOMMUMap(user_info.phaddr, user_info.iova, user_info.size);
}

static int
dev_mem_iommu_unmap(struct file *fp, unsigned long arg)
{
    int ret = 0;
    dev_iommu_info_t user_info = {0};

    ret = copy_from_user(&user_info,
                 (dev_iommu_info_t *)arg,
                 sizeof(dev_iommu_info_t));

    if (ret != 0) {
         osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_iommu_map: copy_from_user failed, ret=%d\n",
        ret, 0, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }
    return osalIOMMUUnmap(user_info.iova, user_info.size);
}

static int
dev_mem_iommu_virt_to_phys(struct file *fp, unsigned long arg)
{
    int ret = 0;
    dev_iommu_info_t user_info = {0};

    ret = copy_from_user(&user_info,
                 (dev_iommu_info_t *)arg,
                 sizeof(dev_iommu_info_t));

    if (ret != 0) {
         osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_iommu_virt_to_phys: copy_from_user failed, ret=%d\n",
        ret, 0, 0, 0, 0, 0, 0, 0);
        return -EIO;
    }
    user_info.phaddr = osalIOMMUVirtToPhys(user_info.iova);
    if (user_info.phaddr) {
        ret = copy_to_user((dev_iommu_info_t *)arg,
                   &user_info, sizeof(dev_iommu_info_t));
        if (ret != 0) {
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "dev_mem_iommu_virt_to_phys: \
            copy_to_user failed, ret=%d\n",
            ret, 0, 0, 0, 0, 0, 0, 0);
            return -EIO;
        }
        return  OSAL_SUCCESS;
    }
    return  OSAL_FAIL;
}

static long
mem_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    switch(cmd) {
        case DEV_MEM_IOC_MEMALLOC:
            ret = mutex_lock_interruptible(&dev_mem_lock);
            if (ret)
                return ret;

            ret = dev_mem_alloc(fp, cmd, arg);
            if (OSAL_SUCCESS != ret) {
                mutex_unlock(&dev_mem_lock);
                return -ENOMEM;
            }
            mutex_unlock(&dev_mem_lock);
            break;

        case DEV_MEM_IOC_MEMFREE:
            ret = mutex_lock_interruptible(&dev_mem_lock);
            if (ret)
                return ret;

            ret = dev_mem_free(fp, cmd, arg);
            if (OSAL_SUCCESS != ret) {
                mutex_unlock(&dev_mem_lock);
                return -EIO;
            }
            mutex_unlock(&dev_mem_lock);
            break;
        case DEV_MEM_IOC_IOMMUMAP:
            return dev_mem_iommu_map(fp, arg);
            break;
        case DEV_MEM_IOC_IOMMUUNMAP:
            return dev_mem_iommu_unmap(fp, arg);
            break;
        case DEV_MEM_IOC_IOMMUVTOP:
            return dev_mem_iommu_virt_to_phys(fp, arg);
            break;
        default:
               osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "Invalid IOCTL command specified(0x%x)\n",
                 cmd, 0, 0, 0, 0, 0, 0, 0);
            return -ENOTTY;
    }
    return OSAL_SUCCESS;
}

static long
mem_ioctl_page(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    switch(cmd) {
        case DEV_MEM_IOC_MEMALLOCPAGE:
            ret = mutex_lock_interruptible(&dev_mem_page_lock);
            if (ret)
                return ret;

            ret = dev_mem_alloc_page(fp, cmd, arg);
            if (OSAL_SUCCESS != ret) {
                mutex_unlock(&dev_mem_page_lock);
                return -ENOMEM;
            }
            mutex_unlock(&dev_mem_page_lock);
            break;

         case DEV_MEM_IOC_MEMFREEPAGE:
            ret = mutex_lock_interruptible(&dev_mem_page_lock);
            if (ret)
                return ret;

            ret = dev_mem_freepage(fp, cmd, arg);
            if (OSAL_SUCCESS != ret) {
                mutex_unlock(&dev_mem_page_lock);
                return -EIO;
            }
            mutex_unlock(&dev_mem_page_lock);
            break;
        default:
               osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "Invalid IOCTL command specified(0x%x)\n",
                 cmd, 0, 0, 0, 0, 0, 0, 0);

            return -ENOTTY;
    }
    return OSAL_SUCCESS;
}

static int
mem_mmap(struct file *fp, struct vm_area_struct *vma)
{
    int ret = 0, id = 0;
    unsigned long phys_kmalloc_area = 0;
    dev_mem_info_t *mem_info = NULL;

    id = vma->vm_pgoff;
    ret = mutex_lock_interruptible(&dev_mem_lock);
    if (ret)
        return ret;
    mem_info = userMemGetInfo(mem_dev_numa, id, current->tgid);
    mutex_unlock(&dev_mem_lock);

    if (!mem_info) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
             "mem_mmap(): cannot find meminfo\n",
             0, 0, 0, 0, 0, 0, 0, 0);
        return -ENOMEM;
    }
    phys_kmalloc_area =
        virt_to_phys((void*)mem_info->kmalloc_area);

    ret = remap_pfn_range(vma,
                  vma->vm_start,
                  phys_kmalloc_area >> PAGE_SHIFT,
                  vma->vm_end - vma->vm_start,
                  vma->vm_page_prot);

    if (ret != 0) {
           osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "remap_pfn_range failed\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
    }
    return ret;
}

static int
mem_mmap_page(struct file *fp, struct vm_area_struct *vma)
{
    int ret = 0, id = 0;
    unsigned long phys_kmalloc_area = 0;
    dev_mem_info_t *mem_info = NULL;

    id = vma->vm_pgoff;
    ret = mutex_lock_interruptible(&dev_mem_page_lock);
    if (ret)
        return ret;
    mem_info = userMemGetInfoPage(mem_dev_page, id, current->tgid);
    mutex_unlock(&dev_mem_page_lock);

    if (!mem_info) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
           "mem_mmap_page(): cannot find meminfo\n",
           0, 0, 0, 0, 0, 0, 0, 0);
        return -ENOMEM;
    }
    phys_kmalloc_area =
        virt_to_phys((void*)mem_info->kmalloc_area);

    ret = remap_pfn_range(vma,
                  vma->vm_start,
                  phys_kmalloc_area >> PAGE_SHIFT,
                  vma->vm_end-vma->vm_start,
                  vma->vm_page_prot);
    if (ret != 0) {
           osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "remap_pfn_range failed\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
    }
    return ret;
}

static int
mem_open(struct inode *inp, struct file *fp)
{
    user_proc_mem_list_t *list = NULL;
    int ret = 0;
    /*
     * Alloc process allocation list
     */
    list = kzalloc(sizeof(user_proc_mem_list_t), GFP_KERNEL);
    if(!list)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
             "memory allocation failed\n",
             0, 0, 0, 0, 0, 0, 0, 0);
        return -ENOMEM;
    }
    list->pid = current->tgid;
    ret = mutex_lock_interruptible(&dev_mem_lock);
    if (ret) {
        kfree(list);
        return ret;
    }
    ADD_ELEMENT_TO_END_OF_LIST(list, mem_dev_numa->tail, mem_dev_numa->head);
    mutex_unlock(&dev_mem_lock);
    return OSAL_SUCCESS;
}

static int
mem_open_page(struct inode *inp, struct file *fp)
{
    user_proc_mem_list_t *list = NULL;
    int ret = 0;
    /*
     * Alloc process allocation list
     */
    list = kzalloc(sizeof(user_proc_mem_list_t), GFP_KERNEL);
    if(!list)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
         "memory allocation failed\n",
             0, 0, 0, 0, 0, 0, 0, 0);
        return -ENOMEM;
    }
    list->pid = current->tgid;
    ret = mutex_lock_interruptible(&dev_mem_page_lock);
    if (ret) {
        kfree(list);
        return ret;
    }
    ADD_ELEMENT_TO_END_OF_LIST(list, mem_dev_page->tail, mem_dev_page->head);
    mutex_unlock(&dev_mem_page_lock);
    return OSAL_SUCCESS;
}

static int
mem_release(struct inode *inp, struct file *fp)
{
    mutex_lock(&dev_mem_lock);
    userMemFreeAllPid(mem_dev_numa, current->tgid);
    mutex_unlock(&dev_mem_lock);
    return OSAL_SUCCESS;
}

static int
mem_release_page(struct inode *inp, struct file *fp)
{
    mutex_lock(&dev_mem_page_lock);
    userMemFreeAllPagePid(mem_dev_page, current->tgid);
    mutex_unlock(&dev_mem_page_lock);
    return OSAL_SUCCESS;
}

static struct file_operations mem_ops = {
    owner:THIS_MODULE,
    mmap:mem_mmap,
    unlocked_ioctl:mem_ioctl,
    compat_ioctl:mem_ioctl,
    open:mem_open,
    release:mem_release,
};

static struct file_operations mem_ops_page = {
    owner:THIS_MODULE,
    mmap:mem_mmap_page,
    unlocked_ioctl:mem_ioctl_page,
    compat_ioctl:mem_ioctl_page,
    open:mem_open_page,
    release:mem_release_page,
};

static chr_drv_info_t mem_drv_info = {
    owner:THIS_MODULE,
    major:0,
    min_minor:DEV_MEM_BASE_MINOR,
    max_minor:DEV_MEM_MAX_MINOR,
    name:DEV_MEM_NAME,
    file_ops:&mem_ops,
};

static chr_drv_info_t mem_drv_info_page = {
    owner:THIS_MODULE,
    major:0,
    min_minor:DEV_MEM_BASE_MINOR,
    max_minor:DEV_MEM_MAX_MINOR,
    name:DEV_MEM_NAME_PAGE,
    file_ops:&mem_ops_page,
};

static int
chr_drv_create_class(chr_drv_info_t* drv_info, char* path)
{
    char name[DEV_PATH_SIZE] = "";
    size_t path_len = 0;
    size_t drv_info_name_len = 0;

    OSAL_LOCAL_ENSURE(drv_info,
              "chr_drv_create_class(): Invalid parameter value ",
              OSAL_FAIL);

    if(path != NULL)
    {
        path_len = OSAL_OS_GET_STRING_LENGTH(path);
        drv_info_name_len = OSAL_OS_GET_STRING_LENGTH(drv_info->name);

        if (path_len >
        (DEV_PATH_SIZE - drv_info_name_len - strlen("/")))
        {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "path to device is greater that max length\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
        }

        strncpy(name, path, path_len);
        strncat(name, "/", 1);
        strncat(name, drv_info->name, drv_info_name_len);
    }

    drv_info->drv_class = class_create(THIS_MODULE,
                       (path) ? name : drv_info->name);
    if (IS_ERR(drv_info->drv_class)) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "class_create failed\n", 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
    return OSAL_SUCCESS;
}

static void
chr_drv_destroy_class(chr_drv_info_t* drv_info)
{
    OSAL_LOCAL_ENSURE(drv_info,
              "chr_drv_destroy_class(): Invalid parameter value ",
              );
    class_destroy( drv_info->drv_class );
    return ;
}

static inline void
chr_drv_destroy_device(chr_drv_info_t *drv_info)
{
    OSAL_LOCAL_ENSURE(drv_info,
              "chr_drv_destroy(): Invalid parameter value ",);

    if (NULL != drv_info->drv_class_dev)
    {
        device_destroy(drv_info->drv_class, MKDEV(drv_info->major,
                   DEV_MEM_BASE_MINOR));
    }
    cdev_del(&(drv_info->drv_cdev));
    unregister_chrdev_region( MKDEV(drv_info->major, DEV_MEM_BASE_MINOR),
                  drv_info->max_minor);
}

static int
chr_drv_create_device(chr_drv_info_t *drv_info, char *path)
{
    int ret = 0;
    dev_t devid = 0;
    char name[DEV_PATH_SIZE] = "";
    size_t path_len = 0;
    size_t drv_info_name_len = 0;

    OSAL_LOCAL_ENSURE(drv_info,
              "chr_drv_destroy(): Invalid parameter value ",
              OSAL_FAIL);

    if (path != NULL)
    {
        path_len = OSAL_OS_GET_STRING_LENGTH(path);
        drv_info_name_len = OSAL_OS_GET_STRING_LENGTH(drv_info->name);
        if (path_len > (DEV_PATH_SIZE - drv_info_name_len - 1))
        {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "path to device is greater that max length\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
        }

        strncpy(name, path, path_len);
        strncat(name, "/", 1);
        strncat(name, drv_info->name, drv_info_name_len);
    }

    ret = alloc_chrdev_region(&devid,
                  drv_info->min_minor,
                  drv_info->max_minor,
                  (path) ? name: drv_info->name);

    if (ret < 0) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "unable to allocate chrdev region\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    drv_info->major = MAJOR(devid);
    cdev_init(&(drv_info->drv_cdev), drv_info->file_ops);
    drv_info->drv_cdev.owner = drv_info->owner;

    ret = cdev_add(&(drv_info->drv_cdev), devid, drv_info->max_minor);
    if (ret < 0) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "cdev add failed\n", 0, 0, 0, 0, 0, 0, 0, 0);
        chr_drv_destroy_device(drv_info);
        return OSAL_FAIL;
    }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
    drv_info->drv_class_dev = device_create(drv_info->drv_class,
              NULL, MKDEV(drv_info->major, DEV_MEM_BASE_MINOR),
              NULL, (path)?name:drv_info->name);
#else
       drv_info->drv_class_dev = device_create(drv_info->drv_class,
              NULL, MKDEV(drv_info->major, DEV_MEM_BASE_MINOR),
              (path)?name:drv_info->name);
#endif

    if(NULL == drv_info->drv_class_dev)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "dev_mem_alloc: device_create failed\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        chr_drv_destroy_device(drv_info);
        return OSAL_FAIL;
    }
    return OSAL_SUCCESS;
}

int register_mem_device_driver(char* path)
{
    int ret = 0;
    mem_dev_numa = kzalloc(sizeof(user_mem_dev_t), GFP_KERNEL);
    if(!mem_dev_numa)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "failed to allocate memory for numa mem device\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
    mem_dev_page = kzalloc(sizeof(user_mem_dev_t), GFP_KERNEL);
    if(!mem_dev_page)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "failed to allocate memory for page mem device\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        kfree(mem_dev_numa);
        return OSAL_FAIL;
    }

    ret = chr_drv_create_class(&mem_drv_info, path);
    if (OSAL_SUCCESS != ret) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "failed to create device driver class\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        kfree(mem_dev_numa);
        kfree(mem_dev_page);
        return OSAL_FAIL;
    }
    ret = chr_drv_create_device(&mem_drv_info, path);
    if (OSAL_SUCCESS != ret) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "failed to create mem numa device driver\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        chr_drv_destroy_class(&mem_drv_info);
        kfree(mem_dev_numa);
        kfree(mem_dev_page);
        return OSAL_FAIL;
    }
    mem_drv_info_page.drv_class = mem_drv_info.drv_class;
    ret = chr_drv_create_device(&mem_drv_info_page, path);
    if (OSAL_SUCCESS != ret) {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "failed to create mem numa device driver\n",
        0, 0, 0, 0, 0, 0, 0, 0);
        chr_drv_destroy_device(&mem_drv_info);
        chr_drv_destroy_device(&mem_drv_info_page);
        kfree(mem_dev_numa);
        kfree(mem_dev_page);
        return OSAL_FAIL;
    }
    mutex_init(&dev_mem_lock);
    mutex_init(&dev_mem_page_lock);
    return OSAL_SUCCESS;
}

void unregister_mem_device_driver(void)
{
    chr_drv_destroy_device(&mem_drv_info);
    chr_drv_destroy_device(&mem_drv_info_page);
    chr_drv_destroy_class(&mem_drv_info);
    userMemFreeAll(mem_dev_numa);
    kfree(mem_dev_numa);
    userMemFreeAllPage(mem_dev_page);
    kfree(mem_dev_page);
    mutex_destroy(&dev_mem_page_lock);
    mutex_destroy(&dev_mem_lock);
}

