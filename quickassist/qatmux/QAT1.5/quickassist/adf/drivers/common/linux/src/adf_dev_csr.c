/*****************************************************************************
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
 *****************************************************************************/

/*****************************************************************************
 * @file adf_dev_csr.c
 *
 * @description
 *      This file contains the ADF code to register the CSR device driver
 *      and the device drivers associated methods.
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_platform.h"
#include "adf_chr_drv.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_user_proxy.h"
#include "adf_dev_csr.h"
#include "adf_platform.h"
#include <linux/mutex.h>

#define ADF_DEV_CSR_NAME            ("icp_dev_csr")
#define ADF_DEV_CSR_MAX_MINOR       (255)
#define ADF_DEV_CSR_BASE_MINOR      (0)

/* device lock */
STATIC DEFINE_MUTEX(csr_lock);

/* Map the memory to user space */
STATIC int adf_csr_mmap(struct file *fp,
                        struct vm_area_struct *vma);

/* Open the device. */
STATIC int adf_csr_open(struct inode *inp,
                        struct file *fp);

/* Release the device. */
STATIC int adf_csr_release(struct inode *inp,
                           struct file *fp);

/* Read from the device. */
STATIC ssize_t adf_csr_read(struct file *fp,
                            char __user *buf,
                            size_t count,
                            loff_t *pos);
/* IOCTL to the device. */
STATIC long adf_csr_ioctl(struct file *, unsigned int, unsigned long);

/* Increase module usage counter function */
extern int adf_get_module(void);

/* Decrease module usage counter function */
extern void adf_put_module(void);

STATIC struct file_operations adf_csr_ops = {
        owner:THIS_MODULE,
        mmap:adf_csr_mmap,
        open:adf_csr_open,
        read:adf_csr_read,
        unlocked_ioctl:adf_csr_ioctl,
        compat_ioctl:adf_csr_ioctl,
        release:adf_csr_release,
};

STATIC adf_chr_drv_info_t adf_csr_drv_info = {
        owner:THIS_MODULE,
        major:0,
        min_minor:ADF_DEV_CSR_BASE_MINOR,
        max_minor:ADF_DEV_CSR_MAX_MINOR,
        name:ADF_DEV_CSR_NAME,
        file_ops:&adf_csr_ops,
};

/*
 * adf_csr_chrdev_register
 * Function creates character device interface
 */
int adf_csr_chrdev_register(void)
{
        int ret = 0;

        ret = adf_chr_drv_create(&adf_csr_drv_info);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create CSR device driver\n");
                return FAIL;
        }
        return SUCCESS;
}

/*
 * adf_csr_chrdev_create
 * Function creates CSR character device instance in /dev filesystem
 */
int adf_csr_chrdev_create(int accel_id)
{
        int ret = 0;

        ret = adf_chr_drv_create_device(&adf_csr_drv_info,
                                        accel_id,
                                        "icp_dev%d_csr");
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create CSR device\n");
                return FAIL;
        }
        return SUCCESS;
}

/*
 * adf_csr_chrdev_unregister
 * Function unregisters character device interface
 */
void adf_csr_chrdev_unregister(void)
{
        adf_chr_drv_destroy(&adf_csr_drv_info);
}

#ifdef ICP_WITHOUT_THREAD

/*
 * adf_csr_read
 * Read from the device
 * Blocking, multi threaded model version
 */
STATIC ssize_t adf_csr_read(struct file *fp,
                            char __user *buff,
                            size_t count,
                            loff_t *pos)
{
        adf_user_process_t* user_proc = NULL;
        Cpa32U events = 0;
        Cpa32U ctr = 0;
        Cpa32U queue[ADF_MAX_PENDING_EVENT];
        size_t size = 0, cpy_size = 0;
        Cpa8U *dest = buff;
        int ret = 0;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        user_proc = (adf_user_process_t*)fp->private_data;
        ICP_SPINLOCK_LOCK(&user_proc->userProcLock);
        events = user_proc->pendingEvents;
        user_proc->pendingEvents = 0;
        memcpy(queue, user_proc->pendingEventsQueue, sizeof(Cpa32U) * events);
        ICP_SPINLOCK_UNLOCK(&user_proc->userProcLock);
        if (events) {
               /* Send all pending messages */
                while(events > ctr)
                {
                        cpy_size = copy_to_user(dest, &queue[ctr++],
                                                sizeof(Cpa32U));
                        if (cpy_size) {
                            ADF_ERROR("Copy to user failed."
                                      "Probably message is lost\n");
                        }
                        dest += sizeof(Cpa32U) - cpy_size;
                        size += sizeof(Cpa32U) - cpy_size;
                 }
                 ret = size;
        }
        return ret;
}

#else

/*
 * adf_csr_read
 * Read from the device
 * Blocking, multi threaded model version
 */
STATIC ssize_t adf_csr_read(struct file *fp,
                            char __user *buff,
                            size_t count,
                            loff_t *pos)
{
        adf_user_process_t* user_proc = NULL;
        Cpa32U events = 0;
        Cpa32U queue[ADF_MAX_PENDING_EVENT];
        size_t size = 0, cpy_size = 0;
        Cpa8U *dest = buff;
        int ret = 0;
        DECLARE_WAITQUEUE(wait_queue, current) ;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        user_proc = (adf_user_process_t*)fp->private_data;
        add_wait_queue(user_proc->processWaitQueue, &wait_queue);
        while(ADF_PROCESS_PROXY_STOPPED != user_proc->processProxyState) {
                Cpa32U ctr = 0;
                set_current_state(TASK_INTERRUPTIBLE);
                ICP_SPINLOCK_LOCK(&user_proc->userProcLock);
                events = user_proc->pendingEvents;
                user_proc->pendingEvents = 0;
                if (events < ADF_MAX_PENDING_EVENT)
                {
                    memcpy(queue, user_proc->pendingEventsQueue, sizeof(Cpa32U)
                                                                  * events);
                }
                else
                {
                    ADF_ERROR("number of pendingEvents %d, max is 10\n", events);
                    ICP_SPINLOCK_UNLOCK(&user_proc->userProcLock);
                    break;
                }
                ICP_SPINLOCK_UNLOCK(&user_proc->userProcLock);
                if (events) {
                        /* Send all pending messages */
                        while(events > ctr)
                        {
                                cpy_size = copy_to_user(dest, &queue[ctr++],
                                            sizeof(Cpa32U));
                                if (cpy_size) {
                                    ADF_ERROR("Copy to user failed."
                                              "Probably message is lost\n");
                                }
                                dest += sizeof(Cpa32U) - cpy_size;
                                size += sizeof(Cpa32U) - cpy_size;
                        }
                        ret = size;
                        break;
                }
                if (signal_pending(current)) {
                       ret = -EAGAIN;
                       break;
                }
                schedule();
        } 
        if (ADF_PROCESS_PROXY_STOPPED == user_proc->processProxyState) {
                ret = -EINTR;
        }
        set_current_state(TASK_RUNNING);
        remove_wait_queue(user_proc->processWaitQueue, &wait_queue);
        return ret;
}
#endif

/*IOCTL defintion*/
STATIC long adf_csr_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
        int ret = 0;
        adf_user_process_t* user_proc = NULL;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        user_proc = (adf_user_process_t*)fp->private_data;

        switch(cmd) {
        case ADF_USER_PROCESS_IOCTL_RELEASE:
            user_proc->processProxyState = ADF_PROCESS_PROXY_STOPPED;
            adf_csr_user_process_event_notify(user_proc);
            break;
        default:
            ret = -ENOTTY;
            break;
        }
        return ret;
}

/*
 * adf_csr_mmap
 * Map the memory to user space
 */
STATIC int adf_csr_mmap(struct file *fp,
                        struct vm_area_struct *vma)
{
        int ret = 0;
        adf_user_process_t* user_proc = NULL;
        icp_accel_dev_t* accel_dev = NULL;
        icp_accel_pci_info_t *pci_info = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        unsigned long rc_csr_size = 0;
        Cpa32U ringCsrBarId = 0;
        uint8_t *rc_bar_area = NULL;
        Cpa32U etrOffset = 0;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        user_proc = (adf_user_process_t*) fp->private_data;
        accel_dev = user_proc->accelHandle;
        pci_info = &(accel_dev->pciAccelDev);
        rc_csr_size = vma->vm_end - vma->vm_start;
        hw_data = accel_dev->pHwDeviceData;
        if (!rc_csr_size) {
                ADF_ERROR("invalid size\n");
                return -EINVAL;
        }
        ringCsrBarId = hw_data->getEtrBarId(hw_data, &etrOffset);
        rc_bar_area = (uint8_t *)(
                      (unsigned long)
                      pci_info->pciBars[ringCsrBarId].baseAddr
                      + etrOffset);

#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))) \
     || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)))
        vma->vm_flags |= VM_IO | VM_RESERVED;
#else
/*
 * VM_RESERVED was removed from the 3.10 kernel and replaced with
 * (VM_DONTEXPAND | VM_DONTDUMP) 
 */
        vma->vm_flags |= VM_IO | (VM_DONTEXPAND | VM_DONTDUMP);
#endif
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        ret = remap_pfn_range(vma,
                vma->vm_start,
                ((unsigned long)rc_bar_area) >> PAGE_SHIFT,
                rc_csr_size,
                vma->vm_page_prot);
        if (ret != 0) {
                ADF_ERROR("remap_pfn_range failed\n");
                return -EAGAIN;
        }
        return 0;
}

/*
 * adf_ring_open
 * Open the device.
 */
STATIC int adf_csr_open(struct inode *inp,
                        struct file *fp)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        adf_user_process_t* user_proc = NULL;
        wait_queue_head_t* wait_queue = NULL;

        /* Increment module usage counter to prevent
         * unloading module when userspace adf is connected */
        if (!adf_get_module()) {
                return -ENODEV;
        }
        mutex_lock(&csr_lock);
        status = adf_userProcessConnect(&fp->private_data,
                                        MINOR(inp->i_rdev),
                                        current->tgid);
        if (CPA_STATUS_SUCCESS != status) {
                adf_put_module();
                mutex_unlock(&csr_lock);
                return -EACCES;
        }

        user_proc = (adf_user_process_t*) fp->private_data;
        wait_queue = ICP_ZALLOC_GEN(sizeof(wait_queue_head_t));
        user_proc->processProxyState = ADF_PROCESS_PROXY_RUNNING;
        if (wait_queue) {
                init_waitqueue_head(wait_queue);
                user_proc->processWaitQueue = (void*) wait_queue;
        }
        else {
                adf_userProcessDisconnect(fp->private_data);
                ADF_ERROR("Failed to allocate memory "
                              "for process wait queue\n");
                adf_put_module();
                mutex_unlock(&csr_lock);
                return -ENOMEM;
        }
        mutex_unlock(&csr_lock);
        return 0;
}

/*
 * adf_csr_release
 * Release the device.
 */
STATIC int adf_csr_release(struct inode *inp,
                           struct file *fp)
{
        CpaStatus status = CPA_STATUS_SUCCESS;
        adf_user_process_t* user_proc = NULL;
        wait_queue_head_t* wait_queue = NULL;
        int ret = 0;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        user_proc = (adf_user_process_t*) fp->private_data;
        mutex_lock(&csr_lock);
        if (user_proc) {
                wait_queue = user_proc->processWaitQueue;
                user_proc->processWaitQueue = NULL;
                status = adf_userProcessDisconnect(&fp->private_data);
                if (CPA_STATUS_SUCCESS != status) {
                        ADF_ERROR("adf_userProcessDisconnet failed\n");
                        ret = -EIO;
                }
                ICP_FREE(wait_queue);
        }
        mutex_unlock(&csr_lock);
        adf_put_module();
        return ret;
}

/*
 * adf_csr_user_process_event_notify
 * Notifies user process waiting for events
 */
void adf_csr_user_process_event_notify(adf_user_process_t* user_proc)
{
        wait_queue_head_t* wait_queue = NULL;
        if (user_proc) {
                wait_queue = (wait_queue_head_t*)user_proc->processWaitQueue;
                if (wait_queue) {
                        wake_up_interruptible(wait_queue);
                }
        }
        return;
}
