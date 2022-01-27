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
 *  version: QAT1.6.L.2.6.0-65
 *
 *****************************************************************************/

/*****************************************************************************
 * @file adf_dev_vf2pf_comms.c
 *
 * @description
 *      char device driver code for managing the userspace vf2pf comms.
 *
 *
 *****************************************************************************/
#include <linux/list.h>

#include "cpa.h"
#include "icp_platform.h"
#include "adf_chr_drv.h"
#include "icp_accel_devices.h"
#include "adf_devmgr.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"
#include "adf_init.h"
#include "adf_drv_sriov.h"
#include "adf_pfvfcomms.h"
#include "icp_adf_accel_mgr.h"

#define ADF_DEV_PFVFCOMMS_MAX_MINOR        (255)
#define ADF_DEV_PFVFCOMMS_BASE_MINOR       (0)
#define ADF_DEV_PFVFCOMMS_MAX_BUFFER_SIZE  32768

STATIC Cpa32U *msgBuffer = NULL;

/*
 * File operation declarations
 */

/* open function. */
STATIC int adf_pfvfcomms_open(struct inode *inp,
                struct file *fp);

/* release function. */
STATIC int adf_pfvfcomms_release(struct inode *inp,
                struct file *fp);

/* ioctl function */
STATIC long vfpfcomms_ioctl(struct file *fp,
                unsigned int cmd,
                unsigned long arg);

STATIC int adf_vfpf_buffer_mmap(struct file *fp,
                         struct vm_area_struct *vma);


/*
 * Character dev file operations
 */
STATIC struct file_operations adf_pfvfcomms_ops = {
                owner:THIS_MODULE,
                open:adf_pfvfcomms_open,
                release:adf_pfvfcomms_release,
                unlocked_ioctl:vfpfcomms_ioctl,
                compat_ioctl:vfpfcomms_ioctl,
                mmap:adf_vfpf_buffer_mmap,
};

/*
 * Character driver info
 */
STATIC adf_chr_drv_info_t adf_pfvfcomms_drv_info = {
                owner:THIS_MODULE,
                major:0,
                min_minor:ADF_DEV_PFVFCOMMS_BASE_MINOR,
                max_minor:ADF_DEV_PFVFCOMMS_MAX_MINOR,
                name:ADF_DEV_PFVFCOMMS_NAME,
                file_ops:&adf_pfvfcomms_ops,
};

/*
 * adf_vfpf_storeMsgForUser
 * Store a message received from VF/PF in kernel space buffer
 * which user processes can read.
 */
CpaStatus adf_vfpf_storeMsgForUser(icp_accel_dev_t *accel_dev, Cpa32U vfNum , Cpa16U msg)
{
        Cpa32U msgToStore = 0;
        Cpa32U count = 0;
        Cpa32U messageAndCounter;
        Cpa32U accelId;

        adf_hw_device_data_t *hw_data = NULL;

        if (!accel_dev) {
                ADF_ERROR("accel_dev NULL\n");
                return CPA_STATUS_FAIL;
        }
        accelId = accel_dev->accelId;

        hw_data = accel_dev->pHwDeviceData;
        if (!hw_data) {
                ADF_ERROR("failed to get hw data\n");
                return CPA_STATUS_FAIL;
        }

        if(!accel_dev->virtualization.virtualized) {
                /* on the PF */
                if(hw_data->maxNumVf <= vfNum) {
                        ADF_ERROR("Invalid VF number\n");
                        return CPA_STATUS_FAIL;
                }
        }

        if (!msgBuffer) {
                ADF_ERROR("failed to get the message buffer\n");
                return CPA_STATUS_FAIL;
        }

        messageAndCounter = adf_vfpf_messageInBuffer(msgBuffer, accelId, vfNum);
        count = messageAndCounter & ICP_PFVF_BUFFER_COUNT_MASK;

        /* increment the message count each time we write */
        count ++;

        /* bit 31 - 16 is the message */
        msgToStore = (msg << ICP_PFVF_BUFFER_MSG_SHIFT);

        /*bit 15 - 0 is the message count */
        msgToStore |= count;

        adf_vfpf_messageInBuffer(msgBuffer, accelId, vfNum) = msgToStore;

        return CPA_STATUS_SUCCESS;
}

/*
 * adf_sendMsg_to_vf_from_pf
 *
 * IOCTL function for sending a message from the PF to the VF
 * function should only be called on the PF.
 *
 */
long adf_sendMsg_to_vf_from_pf(struct file *fp,
                                unsigned int cmd,
                                unsigned long arg)
{
        Cpa32U msg = 0;
        icp_accel_dev_t *accel_dev = NULL;
        icp_accel_pci_vf_info_t *vf = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        adf_sendPfVfMsg_args_t args = {0};
        CpaStatus status = CPA_STATUS_SUCCESS;
        int ret = 0;

        if (!fp || !arg) {
                ADF_ERROR("invalid input descriptors\n");
                return -EBADF;
        }

        if (copy_from_user(&args, (adf_sendPfVfMsg_args_t *)arg,
                        sizeof(adf_sendPfVfMsg_args_t))) {
                ADF_ERROR("failed to get input params\n");
                return -EBADF;
        }

        accel_dev = adf_devmgrGetAccelDevByAccelId(args.accelId);
        if (!accel_dev) {
                ADF_ERROR("failed to get accel device\n");
                return -EBADF;
        }

        /* Ensure we are not a VF */
        if (accel_dev->virtualization.virtualized) {
                ADF_ERROR("Invalid ioctl call\n");
                return -EBADF;
        }

        hw_data = accel_dev->pHwDeviceData;
        if (!hw_data) {
                ADF_ERROR("failed to get hw data\n");
                return -EBADF;
        }

        if ((args.vfNum == 0) || (args.vfNum > hw_data->maxNumVf)) {
                ADF_ERROR("invalid vf number\n");
                return -EBADF;
        }

        vf = (icp_accel_pci_vf_info_t *)accel_dev->pci_vfs[(args.vfNum)-1];
        if (!vf) {
                ADF_ERROR("failed to get valid vf for pf\n");
                return -EBADF;
        }
        /* Ensure that the VF we are sending a message to is initialised */
        if (!vf->init) {
                ADF_ERROR("VF is not initialised\n");
                return -EBADF;
        }

        adf_pf2vf_set_msg_origin(&msg,ICP_PFVF_MSGORIGIN_USER);
        adf_pf2vf_set_msg_content(&msg, args.message);

        status = hw_data->adf_PF2VF_putMsg(vf, msg);
        switch (status) {
        case CPA_STATUS_RESOURCE:   ret = -EAGAIN;
                                    break;
        case CPA_STATUS_SUCCESS:    ret = 0;
                                    break;
        default:                    ret = -EBADF;
                                    break;
        }

        return ret;
}

/*
 * adf_sendMsg_to_vf_from_pf
 *
 * IOCTL function for sending a message from the VF to the PF
 * function should only be called on the VF.
 *
 */
long  adf_sendMsg_to_pf_from_vf(struct file *fp,
                                unsigned int cmd,
                                unsigned long arg)
{
        Cpa32U msg = 0;
        icp_accel_dev_t *accel_dev = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        adf_sendPfVfMsg_args_t args = {0};
        CpaStatus status = CPA_STATUS_SUCCESS;
        int ret = 0;

        if (!fp || !arg) {
                ADF_ERROR("invalid input descriptors\n");
                return -EBADF;
        }

        if (copy_from_user(&args, (adf_sendPfVfMsg_args_t *)arg,
                        sizeof(adf_sendPfVfMsg_args_t))) {
                ADF_ERROR("failed to get input params\n");
                return -EBADF;
        }

        accel_dev = adf_devmgrGetAccelDevByAccelId(args.accelId);
        if (!accel_dev) {
                ADF_ERROR("failed to get accel device \n");
                return -EBADF;
        }

        /* Ensure we are not a PF */
        if (!accel_dev->virtualization.virtualized) {
                ADF_ERROR("Invalid ioctl call\n");
                return -EBADF;
        }

        hw_data = accel_dev->pHwDeviceData;
        if (!hw_data) {
                ADF_ERROR("failed to get hw_data\n");
                return -EBADF;
        }

        if (!BIT_IS_SET(accel_dev->adfSubsystemStatus, ADF_STATUS_SYSTEM_STARTED))
        {
                ADF_ERROR("Device%d not up, can't send user msg to PF\n",
                                                          args.accelId);
                return -EBADF;
        }
        adf_vf2pf_set_msg_origin(&msg,ICP_PFVF_MSGORIGIN_USER);
        adf_vf2pf_set_msg_content(&msg, args.message);

        status = hw_data->adf_VF2PF_putMsg(accel_dev, msg);
        switch (status) {
        case CPA_STATUS_RESOURCE:   ret = -EAGAIN;
                                    break;
        case CPA_STATUS_SUCCESS:    ret = 0;
                                    break;
        default:                    ret = -EBADF;
                                    break;
        }

        return ret;
}

/*
 * adf_ring_mmap
 * mmap file operation
 */
STATIC int adf_vfpf_buffer_mmap(struct file *fp,
                         struct vm_area_struct *vma)
{
        int ret = SUCCESS;
        unsigned long vm_size = 0;
        unsigned long size_to_map = 0;
        int *kmalloc_area = NULL;

        if (NULL == fp) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        vm_size = vma->vm_end - vma->vm_start;
        size_to_map = (ADF_DEV_PFVFCOMMS_MAX_BUFFER_SIZE < PAGE_SIZE) ?
                                  PAGE_SIZE : ADF_DEV_PFVFCOMMS_MAX_BUFFER_SIZE;

        /* Already allocated on a page boundary */
        kmalloc_area = (int *)(msgBuffer);

        ret = remap_pfn_range(vma,
                vma->vm_start,
                virt_to_phys((void*)((unsigned long)kmalloc_area))
                              >> PAGE_SHIFT,
                vma->vm_end-vma->vm_start,
                vma->vm_page_prot);

        if(ret != 0) {
                ADF_ERROR("remap_pfn_range failed\n");
                return -EAGAIN;
        }
        return 0;
}

/*
 * vfpfcomms_ioctl
 *
 * IOCTL function managing all IOCTL calls into the device
 *
 */
STATIC long vfpfcomms_ioctl(struct file *fp,
                unsigned int cmd,
                unsigned long arg)
{
        long ret = 0;

        switch(cmd) {
        case ADF_PFVF_IOC_SENDMSG_TO_VF_FROM_PF:
                ret = adf_sendMsg_to_vf_from_pf(fp, cmd, arg);
                break;
        case ADF_PFVF_IOC_SENDMSG_TO_PF_FROM_VF:
                ret = adf_sendMsg_to_pf_from_vf(fp, cmd, arg);
                break;
        default:
                ret = 0;
                break;
        };

        return ret;
}

/*
 * adf_pfvfcomms_chrdev_unregister
 * Function unregisters character device interface
 */
void adf_pfvfcomms_chrdev_unregister(void)
{
        adf_chr_drv_destroy(&adf_pfvfcomms_drv_info);
}

/*
 * adf_pfvfcomms_open
 * open file operation
 */
STATIC int adf_pfvfcomms_open(struct inode *inp, struct file *fp)
{
        if (NULL == fp) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        return 0;
}

/*
 * adf_pfvfcomms_release
 * release file operation
 */
STATIC int adf_pfvfcomms_release(struct inode *inp, struct file *fp)
{
        if (NULL == fp) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        return 0;
}

/*
 * register_adf_pfvfcomms_device_driver
 *
 * Function which dynamically allocates the major number, creates the pfvfcomms
 * char device driver and adds it to the system.
 */
int register_adf_pfvfcomms_device_driver(void)
{
        int ret = 0;
        int bufferSize, order, orderSize;

        ret = adf_chr_drv_create(&adf_pfvfcomms_drv_info);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create vf2pf device driver\n");
                return FAIL;
        }
        ret = adf_chr_drv_create_device(&adf_pfvfcomms_drv_info, 0,
                        ADF_DEV_PFVFCOMMS_NAME);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create vf2pf device\n");
                adf_chr_drv_destroy(&adf_pfvfcomms_drv_info);
                return FAIL;
        }

        bufferSize = ICP_ADF_MAX_NUM_VF * ADF_MAX_DEVICES * sizeof(Cpa32U);
        orderSize = PAGE_SIZE;
        order = 0;
        while (bufferSize  < orderSize) {
                order++;
                orderSize *= 2;
        }
        msgBuffer = (Cpa32U *) ICP_GET_FREE_PAGES(GFP_KERNEL, order);

        if(NULL == msgBuffer)
        {
                ADF_ERROR("failed to allocate memory for mesg buffer\n");
                adf_chr_drv_destroy(&adf_pfvfcomms_drv_info);
                return FAIL;
        }
        memset(msgBuffer, 0 ,(ICP_ADF_MAX_NUM_VF * sizeof(Cpa32U)
                *ADF_MAX_DEVICES));
        return SUCCESS;
}

/*
 * unregister_adf_pfvfcomms_device_driver
 *
 * Function which removes the pfvfcomms char device from the system, deallocates
 * the major number.
 */
void unregister_adf_pfvfcomms_device_driver(void)
{
        int bufferSize, order, orderSize;

        adf_chr_drv_destroy(&adf_pfvfcomms_drv_info);

        bufferSize = ICP_ADF_MAX_NUM_VF * ADF_MAX_DEVICES * sizeof(Cpa32U);
        orderSize = PAGE_SIZE;
        order = 0;
        while (bufferSize  < orderSize) {
                order++;
                orderSize *= 2;
        }

        if(NULL != msgBuffer)
        {
                ICP_FREE_PAGES((unsigned long) msgBuffer, order);
        }
        msgBuffer = NULL;
}
