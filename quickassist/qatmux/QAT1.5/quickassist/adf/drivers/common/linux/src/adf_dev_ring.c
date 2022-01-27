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
 * @file adf_dev_ring.c
 *
 * @description
 *      This file contains the ADF code to register the RING device driver
 *      and the device drivers associated methods.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "adf_chr_drv.h"
#include "icp_accel_devices.h"
#include "adf_devmgr.h"
#include "adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_dev_ring_ctl.h"
#include "adf_dev_ring.h"
#include "adf_user_proxy.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "icp_adf_init.h"
#include "icp_adf_cfg.h"
#include "adf_proc_debug.h"

#define ADF_DEV_RING_NAME            ("icp_dev_ring")
#define ADF_DEV_RING_MAX_MINOR       (255)
#define ADF_DEV_RING_BASE_MINOR      (0)
#define ADF_RING_VAR_SIZE            (4)

#define ADF_RING_CLOSING_DOWN        0xFFFFFFFF
#define ADF_RING_DELETE              0xFFFFFFFE
#define ADF_RING_INITIALIZED         0x01
#define ADF_RING_NOT_INITIALIZED     0x00
#define SERVICE_NAME_LEN             64
#define MAX_BANKS_IN_DEV             32


/*
 * File operation declarations
 */
STATIC long adf_ring_ioctl(struct file *fp,
                           unsigned int cmd,
                           unsigned long arg);

/* mmap function. */
STATIC int adf_ring_mmap(struct file *fp,
                         struct vm_area_struct *vma);

/* open function. */
STATIC int adf_ring_open(struct inode *inp,
                         struct file *fp);

/* release function. */
STATIC int adf_ring_release(struct inode *inp,
                            struct file *fp);

/* read function. */
STATIC ssize_t adf_ring_read(struct file *fp,
                             char __user *buf,
                             size_t count,
                             loff_t *pos);

/*
 * Structure that is stored in file->private_data
 */
typedef struct adf_ring_priv_data_s {
        icp_accel_dev_t      *accel_dev;
        Cpa32U               rings_mask_state;
        adf_dev_rings_mask_t rings_mask;
        Cpa32U               *rings;
        void                 *read_rings_mask;
        ICP_SPINLOCK         ring_mask_lock;
        ICP_MUTEX            ring_data_lock;
        wait_queue_head_t    ring_wait_queue;
} adf_userspace_ring_priv_data_t;

/*
 * Character dev file operations
 */
STATIC struct file_operations adf_ring_ops = {
        owner:THIS_MODULE,
        mmap:adf_ring_mmap,
        unlocked_ioctl:adf_ring_ioctl,
        compat_ioctl:adf_ring_ioctl,
        open:adf_ring_open,
        release:adf_ring_release,
        read:adf_ring_read,
};

/*
 * Character driver info
 */
STATIC adf_chr_drv_info_t adf_ring_drv_info = {
        owner:THIS_MODULE,
        major:0,
        min_minor:ADF_DEV_RING_BASE_MINOR,
        max_minor:ADF_DEV_RING_MAX_MINOR,
        name:ADF_DEV_RING_NAME,
        file_ops:&adf_ring_ops,
};

/*
 * adf_ring_chrdev_register
 * Function creates character device interface
 */
int adf_ring_chrdev_register(void)
{
        int ret = 0;

        ret = adf_chr_drv_create(&adf_ring_drv_info);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create RING device driver\n");
                return FAIL;
        }

        return SUCCESS;
}

/*
 * adf_ring_chrdev_create
 * Function creates ring character device instance in /dev filesystem
 */
int adf_ring_chrdev_create(int accel_id)
{
        int ret = 0;

        ret = adf_chr_drv_create_device(&adf_ring_drv_info,
                                        accel_id,
                                        "icp_dev%d_ring");
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create RING device\n");
                return FAIL;
        }

        return SUCCESS;
}

/*
 * adf_ring_chrdev_unregister
 * Function unregisters character device interface
 */
void adf_ring_chrdev_unregister(void)
{
        adf_chr_drv_destroy(&adf_ring_drv_info);
}

/*
 * Dummy kernelspace callback for userspace rings
 */
void adf_ring_dummy_callback(icp_comms_trans_handle comms_handle, void *pMsg)
{
        /* The real callback lives in user space */
        return;
}

/*
 * adf_ring_ioc_create_handle
 * ioctl function to create rings
 */
STATIC long adf_ring_ioc_create_handle(struct file *fp,
                                       unsigned int cmd,
                                       unsigned long arg)
{
        int ret = SUCCESS;
        CpaStatus status = CPA_STATUS_SUCCESS;
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;
        adf_dev_ring_handle_t ring_handle = {{0}};
        icp_comms_trans_handle comms_handle = NULL;
        icp_trans_handle *trans_handle = NULL;
        icp_et_ring_data_t *ring_data = NULL;
        icp_et_ring_bank_data_t *bank_data = NULL;
        icp_etr_priv_data_t *priv_data = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        char *service_name = NULL, *section_name=NULL;
        Cpa32U bank_number_in_dev = 0, bank_number = 0;
        Cpa32U ring_number = 0, ring_num_in_bank = 0;
        char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
        Cpa32U numBanksPerAccel = 0;
        Cpa32U numRingsPerBank = 0;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        ring_priv_data = (adf_userspace_ring_priv_data_t *)fp->private_data;
        ICP_MUTEX_LOCK(&ring_priv_data->ring_data_lock);
        accel_dev = ring_priv_data->accel_dev;

        ret = copy_from_user(&ring_handle,
                             (adf_dev_ring_handle_t *)arg,
                             sizeof(adf_dev_ring_handle_t));
        if (SUCCESS != ret) {
                ADF_ERROR("failed to copy ring handle info\n");
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }
        /*Allocate memory for the service name and copy from user space. */
        if (ring_handle.service_name_len <= SERVICE_NAME_LEN) {
            service_name = ICP_ZALLOC_GEN(ring_handle.service_name_len + 1);
            if (NULL == service_name) {
                ADF_ERROR("failed to allocate service name buffer\n");
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -ENOMEM;
            }
        }
        else {
            ADF_ERROR("Service name cannot be"
                      "more than %d\n",SERVICE_NAME_LEN);
            ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
            return -EIO;
        }

        ret = copy_from_user(service_name,
                             ring_handle.service_name,
                             ring_handle.service_name_len);
        if (SUCCESS != ret) {
                ADF_ERROR("unable to copy service_name\n");
                ICP_FREE(service_name);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }
        /*Allocate memory for the section name and copy from user space. */
        section_name = ICP_ZALLOC_GEN(ring_handle.section_name_len + 1);
        if (NULL == section_name) {
                ADF_ERROR("failed to allocate section name buffer\n");
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                ICP_FREE(service_name);
                return -ENOMEM;
        }

        ret = copy_from_user(section_name,
                             ring_handle.section_name,
                             ring_handle.section_name_len);
        if (SUCCESS != ret) {
                ADF_ERROR("unable to copy section_name\n");
                ICP_FREE(section_name);
                ICP_FREE(service_name);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }

        numBanksPerAccel = GET_NUM_BANKS_PER_ACCEL(accel_dev);
        service_name[ring_handle.service_name_len] = '\0';
        bank_number = ring_handle.bank_num;
        bank_number_in_dev = bank_number +
                     (ring_handle.accel_num * numBanksPerAccel);

        /* setup dummy callback for response ring, NULL callback
         * means request ring */
        if (ring_handle.callback != NULL) {
                ring_handle.callback = adf_ring_dummy_callback;
        }

        /* Change resp from POLL to USER_POLL. */
        if (ICP_RESP_TYPE_POLL == ring_handle.resp) {
                ring_handle.resp = ADF_RESP_TYPE_USER_POLL;
        }
        ring_handle.is_wireless = CPA_FALSE;
        ring_handle.is_dyn = CPA_FALSE;

        if ((!strncmp(section_name, WIRELESS_SEC, strlen(WIRELESS_SEC)))
                || (!strncmp(section_name, DYN_SEC, strlen(DYN_SEC)))) {
            accel_dev = ring_priv_data->accel_dev;
            priv_data = (icp_etr_priv_data_t *)accel_dev->pCommsHandle;
            bank_data = &priv_data->banks[bank_number_in_dev];

            if (!strncmp(section_name, WIRELESS_SEC, strlen(WIRELESS_SEC))) {
                ring_handle.is_wireless = CPA_TRUE;
            }
            if (!strncmp(section_name, DYN_SEC, strlen(DYN_SEC))) {
                ring_handle.is_dyn = CPA_TRUE;
            }

            status = icp_adf_cfgGetParamValue(accel_dev,
                                            section_name, service_name, val);
            if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("No such entry in the config file: %s\n",
                        service_name);
                ICP_FREE(section_name);
                ICP_FREE(service_name);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return CPA_STATUS_FAIL;
            }
            ring_number = (Cpa32U)ICP_STRTOUL(val, NULL, ADF_CFG_BASE_DEC);

            status = adf_setRingInUse(bank_data, ring_number);

            if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("adf_ring_ioc_create_handle "
                        "Service %s on Dev %d failed to set ring InUse. \n",
                                    service_name,accel_dev->accelId);
                ICP_FREE(section_name);
                ICP_FREE(service_name);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
            }

            /* Set up comms handle */
            status = icp_adf_transGetHandle(ring_priv_data->accel_dev,
                    ring_handle.trans_type,
                    section_name,
                    ring_handle.accel_num,
                    ring_handle.bank_num,
                    service_name,
                    &comms_handle);

            if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("icp_adf_transGetHandle for service %s failed.\n",
                                                        service_name);
                ICP_FREE(section_name);
                ICP_FREE(service_name);
                /* Mark the ring as "not InUse" */
                ICP_SPINLOCK_LOCK(&bank_data->bankLock);
                bank_data->ringsInUseMask &= ~(1<<ring_number);
                ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
            }
        }
        else {

            /* Check if this ring exists as a Request Ring  */
            status = icp_adf_transCreateHandle(
                                        ring_priv_data->accel_dev,
                                        ring_handle.trans_type,
                                        section_name,
                                        ring_handle.accel_num,
                                        ring_handle.bank_num,
                                        service_name,
                                        ring_handle.info,
                                        ring_handle.callback,
                                        ring_handle.resp,
                                        ring_handle.ring_size,
                                        &comms_handle);
            if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("icp_adf_transCreateHandle for service %s failed.\n",
                                                            service_name);
                ICP_FREE(section_name);
                ICP_FREE(service_name);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
            }
        }

        /* Get Ring Number */
        status = icp_adf_transGetRingNum(comms_handle, &ring_number);
        if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("icp_adf_transGetRingNum for "
                              "service %s failed.\n",
                              service_name);
                icp_adf_transReleaseHandle(comms_handle);
                ICP_FREE(section_name);
                ICP_FREE(service_name);
                /* Mark the ring as "not InUse" */
                ICP_SPINLOCK_LOCK(&bank_data->bankLock);
                bank_data->ringsInUseMask &= ~(1<<ring_number);
                ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }
        numRingsPerBank = GET_NUM_RINGS_PER_BANK(accel_dev);
        ring_num_in_bank = ring_number % numRingsPerBank;

        ICP_FREE(service_name);
        ICP_FREE(section_name);
        ring_handle.k_accelhandle = ring_priv_data->accel_dev;
        ring_handle.k_comms_handle = comms_handle;
        /* Switch resp type back again after exiting
         * kernel space create ring. */
        if (ADF_RESP_TYPE_USER_POLL == ring_handle.resp) {
                ring_handle.resp = ICP_RESP_TYPE_POLL;
        }

        priv_data = (icp_etr_priv_data_t *)accel_dev->pCommsHandle;
        if (NULL == priv_data) {
                ADF_ERROR("Comms Handle invalid\n");
                icp_adf_transReleaseHandle(comms_handle);
                ICP_SPINLOCK_LOCK(&bank_data->bankLock);
                bank_data->ringsInUseMask &= ~(1<<ring_num_in_bank);
                ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EINVAL;
        }

        bank_data = &priv_data->banks[bank_number_in_dev];
        if (ICP_RESP_TYPE_IRQ == ring_handle.resp) {

                ring_handle.interrupt_user_mask = bank_data->interruptMask;
                ring_handle.interrupt_user_mask |= bank_data->interruptUserMask;
                bank_data->interruptUserMask = ring_handle.interrupt_user_mask;
                /* Clean the IRQ flag for user space rings */
                bank_data->interruptMask &= ~(1 << ring_num_in_bank);
                ring_handle.interrupt_kernel_mask = bank_data->interruptMask;
        }
        else if (ICP_RESP_TYPE_POLL == ring_handle.resp) {
                /* Set the polling mask for this ring handle. */
                ring_handle.pollingMask = 1 << ring_num_in_bank;
                /* This is a user space ring - remove its mask from the
                 * kernel space structure. */
                bank_data->pollingMask &= ~(1 << ring_num_in_bank);
        }

        trans_handle = (icp_trans_handle *)comms_handle;
        trans_handle->is_wireless = ring_handle.is_wireless;
        trans_handle->is_dyn = ring_handle.is_dyn;
        ring_data = (icp_et_ring_data_t *)trans_handle->trans_data;
        ring_data->userSpaceRing = CPA_TRUE;
        ring_data->info = ring_handle.info;
        ring_handle.ring_num = ring_data->ringNumber;
        ring_handle.bank_num = ring_data->bankNumber;
        ring_handle.ring_size = ring_data->sizeInBytes;
        ring_handle.message_size = ring_data->msgSizeInBytes;
        ring_handle.modulo = ring_data->modulo;
        ring_handle.head = 0;
        ring_handle.tail = 0;
        ring_handle.timed_coalesc_enabled = bank_data->timedCoalescEnabled;
        ring_handle.number_msg_coalesc_enabled =
                                        bank_data->numberMsgCoalescEnabled;

        /* we could not use all the space because in this case,
         * tail CSR tail could not move */
        ring_handle.max_requests_inflight = (ring_handle.ring_size
                                            / ring_handle.message_size) - 1;
        ring_handle.min_resps_per_head_write = ring_data->minRespsPerHeadWrite;

        /* When using interrupt mode in user space, the response ring
         * head is not always updated during response processing and so the
         * interrupt condition persists when the interrupts are re-enabled.
         * This results in a refiring of the same interrupt multiple times
         * which causes a significant drop in system performance.
         * To prevent this from happening, the min_resps_per_head_write should
         * be reset in interrupt mode */
        if(ICP_RESP_TYPE_IRQ == ring_handle.resp)
        {
            ring_handle.min_resps_per_head_write = 0;
        }
        else
        {
            ring_handle.min_resps_per_head_write = ring_data->minRespsPerHeadWrite;
        }
        ring_handle.coal_write_count = ring_handle.min_resps_per_head_write;
        ret = copy_to_user((adf_dev_ring_handle_t *)arg,
                           &ring_handle,
                           sizeof(adf_dev_ring_handle_t));
        if (SUCCESS != ret) {
                ADF_ERROR("copy_to_user failed\n");
                icp_adf_transReleaseHandle(comms_handle);
                ICP_SPINLOCK_LOCK(&bank_data->bankLock);
                bank_data->ringsInUseMask &= ~(1<<ring_num_in_bank);
                ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }

        ring_data->userSpacePrivateData = (void*)ring_priv_data;
        status = adf_userProcessCreateRing(ring_priv_data->accel_dev,
                                           trans_handle, current->tgid);
        if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("User proxy create ring failed\n");
                icp_adf_transReleaseHandle(comms_handle);
                ICP_SPINLOCK_LOCK(&bank_data->bankLock);
                bank_data->ringsInUseMask &= ~(1<<ring_num_in_bank);
                ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }
        if ((!ring_handle.is_wireless) && (!ring_handle.is_dyn)) {
                if (ring_handle.callback == NULL) {
                        ringInfoCb cb = (ringInfoCb) accel_dev->ringInfoCallBack;
                        if (NULL != cb) {
                                cb(accel_dev, ring_number, ICP_ADF_RING_ENABLE,
                                    ring_handle.info);
                        }
                        else if (!accel_dev->virtualization.virtualized) {
                                ADF_ERROR("Ring Info Callback is NULL, "
                                              "the ring will be disabled\n");
                        }
                }
                osalAtomicInc(&accel_dev->userspaceRingsCounter);
        }
        ring_priv_data->rings[ring_data->bankNumber] |=
                                         (1 << ring_data->ringNumber);
        ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
        return 0;
}

/*
 * adf_ring_ioc_release_handle
 * ioctl function to release rings
 */
STATIC long adf_ring_ioc_release_handle(struct file *fp,
                                         unsigned int cmd,
                                         unsigned long arg)
{
        int ret = SUCCESS;
        CpaStatus status = CPA_STATUS_SUCCESS;
        adf_dev_ring_handle_t ring_handle = {{0}};
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        icp_et_ring_data_t *ring_data = NULL;
        icp_et_ring_bank_data_t *bank_data = NULL;
        icp_etr_priv_data_t *priv_data = NULL;
        icp_comms_trans_handle comms_handle = NULL;
        icp_trans_handle *trans_handle = NULL;
        Cpa32U bank_number = 0, bank_number_in_dev = 0;
        Cpa32U ring_number = 0;
        Cpa32U ring_number_in_bank = 0;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        ring_priv_data = (adf_userspace_ring_priv_data_t *)fp->private_data;
        accel_dev = ring_priv_data->accel_dev;
        ICP_MUTEX_LOCK(&ring_priv_data->ring_data_lock);
        ret = copy_from_user(&ring_handle,
                             (adf_dev_ring_handle_t *)arg,
                             sizeof(adf_dev_ring_handle_t));
        if (SUCCESS != ret) {
                ADF_ERROR("failed to copy ring handle info\n");
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }
        comms_handle = ring_handle.k_comms_handle;
        trans_handle = (icp_trans_handle *)comms_handle;
        ring_data = (icp_et_ring_data_t *)trans_handle->trans_data;
        status = icp_adf_transGetRingNum(comms_handle, &ring_number);
        if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("icp_adf_transGetRingNum failed\n");
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }
        if ((ring_handle.is_wireless) || (ring_handle.is_dyn))
        {
                accel_dev = ring_priv_data->accel_dev;
                priv_data = (icp_etr_priv_data_t *)accel_dev->pCommsHandle;
                bank_number = ring_handle.bank_num %
                                            GET_NUM_BANKS_PER_ACCEL(accel_dev);
                bank_number_in_dev = bank_number + (ring_handle.accel_num *
                                           GET_NUM_BANKS_PER_ACCEL(accel_dev));
                if (bank_number_in_dev > MAX_BANKS_IN_DEV) {
                    ADF_ERROR("Bank number cannot be"
                             " more than %d\n",MAX_BANKS_IN_DEV);
                    ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                    return -EIO;
                }
                bank_data = &priv_data->banks[bank_number_in_dev];
                ring_number_in_bank = ring_number % ICP_ETR_MAX_RINGS_PER_BANK;

                if (bank_data->allocatedRings & (1<<ring_number_in_bank)) {
                            /* Allocated, now check the InUse bit */
                            ICP_SPINLOCK_LOCK(&bank_data->bankLock);
                            if (!(bank_data->ringsInUseMask &
                                                   (1<<ring_number_in_bank))) {
                                    ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                                    ADF_ERROR("adf_ring_ioc_release_handle "
                                          "failed. Ring not in use.\n");
                                    ICP_MUTEX_UNLOCK(
                                              &ring_priv_data->ring_data_lock);
                                    return -EIO;
                            }
                            else {
                                    /* Mark the ring as not "InUse" */
                                    bank_data->ringsInUseMask &=
                                                     ~(1<<ring_number_in_bank);
                            }
                            ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                }
                else {
                        ADF_ERROR("adf_ring_ioc_release_handle "
                                  " failed. Ring not allocated.\n");
                        ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                        return -EIO;
                }
                status = adf_userProcessDeleteRing(ring_priv_data->accel_dev,
                                                   (icp_trans_handle*)
                                                   ring_handle.k_comms_handle,
                                                   current->tgid);
                if (CPA_STATUS_SUCCESS != status) {
                        ADF_ERROR("User delete create ring failed\n");
                        ret = -EIO;
                }
        }
        else
        {
                ring_priv_data->rings[ring_data->bankNumber] &=
                                             ~(1 << ring_data->ringNumber);
                /* send disable ring message */
                if (ring_handle.callback == NULL) {
                        ringInfoCb cb =
                                    (ringInfoCb)accel_dev->ringInfoCallBack;
                        if(NULL != cb) {
                                cb(accel_dev, ring_number,
                                   ICP_ADF_RING_DISABLE, ring_handle.info);
                        }
                        else if(!accel_dev->virtualization.virtualized) {
                                ADF_ERROR("Ring Info Callback is NULL, "
                                          "the ring will not be disabled\n");
                        }
                }
                else {
                        /* send message to userspace to close
                         * the reading thread */
                        adf_ring_process_ring_del_notify(ring_data);
                }
                status = adf_userProcessDeleteRing(ring_priv_data->accel_dev,
                                                   (icp_trans_handle*)
                                                   ring_handle.k_comms_handle,
                                                   current->tgid);
                if (CPA_STATUS_SUCCESS != status) {
                        ADF_ERROR("User delete create ring failed\n");
                        ret = -EIO;
                }

                status = icp_adf_transReleaseHandle(ring_handle.k_comms_handle);
                if (CPA_STATUS_SUCCESS != status) {
                        ADF_ERROR("icp_adf_transReleaseHandle failed, "
                                  "status = %d\n", status);
                        ret = -EIO;
                }
        }
        if((!ring_handle.is_wireless) && (!ring_handle.is_dyn)) {
                osalAtomicDec(&accel_dev->userspaceRingsCounter);
        }
        ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
        return ret;
}

STATIC long adf_ring_get_dyninstance(struct file *fp,
                           unsigned int cmd,
                           unsigned long arg)
{
        long ret = SUCCESS;
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;
        adf_instance_assignment_t assign;
        icp_accel_dev_t *accel_dev = NULL;
        CpaStatus status = CPA_STATUS_SUCCESS;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        ret = copy_from_user(&assign,
                             (adf_instance_assignment_t *)arg,
                             sizeof(adf_instance_assignment_t));

        if (SUCCESS != ret) {
            ADF_ERROR("failed to copy current assignment\n");
            return -EIO;
        }

        ring_priv_data = (adf_userspace_ring_priv_data_t *)fp->private_data;
        accel_dev = ring_priv_data->accel_dev;

        status = adf_trans_getDynInstance(accel_dev, assign.stype, &assign.val,
					assign.accelID);
        if(CPA_STATUS_SUCCESS != status)
        {
            return -EBUSY;
        }
        status = adf_userProcessCreateDynInstance(accel_dev,
                                                  assign.val,
                                                  assign.stype,
                                                  current->tgid);
        if(CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("adf_userProcessCreateDynInstance failed\n");
            icp_adf_putDynInstance(accel_dev, assign.stype, assign.val);
            return -EBUSY;
        }
        ret = copy_to_user((adf_instance_assignment_t *)arg,
                           &assign,
                           sizeof(adf_instance_assignment_t));
        if (SUCCESS != ret) {
            ADF_ERROR("copy_to_user failed\n");
            icp_adf_putDynInstance(accel_dev, assign.stype, assign.val);
            return -EIO;
        }
        return ret;
}

STATIC long adf_ring_put_dyninstance(struct file *fp,
                           unsigned int cmd,
                           unsigned long arg)
{
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;
        adf_instance_assignment_t assign;
        long        ret = 0;
        icp_accel_dev_t *accel_dev = NULL;
        CpaStatus status = CPA_STATUS_SUCCESS;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        ring_priv_data = (adf_userspace_ring_priv_data_t *)fp->private_data;
        accel_dev = ring_priv_data->accel_dev;
        ret = copy_from_user(&assign,
                             (adf_instance_assignment_t *)arg,
                             sizeof(adf_instance_assignment_t));

        if (SUCCESS != ret) {
            ADF_ERROR("failed to copy current assignment\n");
            return -EIO;
        }
        status = icp_adf_putDynInstance(accel_dev, assign.stype, assign.val);
        if(CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("fail to put dyn instance\n");
            return -EIO;
        }

        adf_userProcessDeleteDynInstance(accel_dev,
                                         assign.val,
                                         assign.stype,
                                         current->tgid);
        return SUCCESS;
}

STATIC long adf_ring_get_numdyninstance(struct file *fp,
                           unsigned int cmd,
                           unsigned long arg)
{
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;
        adf_instance_assignment_t assign;
        long        ret = 0;
        icp_accel_dev_t *accel_dev = NULL;
        CpaStatus status = CPA_STATUS_SUCCESS;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        ret = copy_from_user(&assign,
                             (adf_instance_assignment_t *)arg,
                             sizeof(adf_instance_assignment_t));

        if (SUCCESS != ret) {
            ADF_ERROR("failed to copy current assignment\n");
            return -EIO;
        }

        ring_priv_data = (adf_userspace_ring_priv_data_t *)fp->private_data;
        accel_dev = ring_priv_data->accel_dev;
        status = adf_trans_getNumAvailDynInstance(accel_dev, assign.stype,
						assign.accelID, &assign.val);
        if(CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("adf_trans_getNumAvailDynInstance failed\n");
            return -EBUSY;
        }
        ret = copy_to_user((adf_instance_assignment_t *)arg,
                           &assign,
                           sizeof(adf_instance_assignment_t));
        if (SUCCESS != ret) {
            ADF_ERROR("copy_to_user failed\n");
            return -EIO;
        }

        return ret;
}

/*
 * adf_ring_ioctl
 * ioctl file operation
 */
STATIC long adf_ring_ioctl(struct file *fp,
                           unsigned int cmd,
                           unsigned long arg)
{
        long ret = FAIL;

        switch(cmd) {
        case ADF_DEV_RING_IOC_CREATEHANDLE:
                ret = adf_ring_ioc_create_handle(fp, cmd, arg);
                break;
        case ADF_DEV_RING_IOC_RELEASEHANDLE:
                ret = adf_ring_ioc_release_handle(fp, cmd, arg);
                break;
        case ADF_DEV_RING_GET_DYNINSTANCE:
                ret = adf_ring_get_dyninstance(fp, cmd, arg);
                break;
        case ADF_DEV_RING_PUT_DYNINSTANCE:
                ret = adf_ring_put_dyninstance(fp, cmd, arg);
                break;
        case ADF_DEV_RING_GETNUM_AVAIL_DYNINSTANCE:
                ret = adf_ring_get_numdyninstance(fp, cmd, arg);
                break;
        default:
                ADF_ERROR("Invalid command specified(0x%x)\n", cmd);
                return -ENOTTY;
                break;
        }
        return ret;
}

/*
 * adf_ring_mmap
 * mmap file operation
 */
STATIC int adf_ring_mmap(struct file *fp,
                         struct vm_area_struct *vma)
{
        int ret = SUCCESS;
        CpaStatus status = CPA_STATUS_SUCCESS;
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;
        int ring_number = 0;
        icp_trans_handle *trans_handle = NULL;
        icp_et_ring_data_t *ring_data = NULL;
        unsigned long vm_size = 0;
        unsigned long size_to_map = 0;
        int *ring_kmalloc_area = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        Cpa32U maxNumRings = 0;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        ring_priv_data = (adf_userspace_ring_priv_data_t *)fp->private_data;
        accel_dev = ring_priv_data->accel_dev;

        maxNumRings = GET_MAX_BANKS(accel_dev)
                    * GET_NUM_RINGS_PER_BANK(accel_dev);

        ICP_MUTEX_LOCK(&ring_priv_data->ring_data_lock);
        ring_number = vma->vm_pgoff;
        if (ring_number < 0 || ring_number >= maxNumRings) {
                ADF_ERROR("invalid ring number %d\n", ring_number);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }

        status = adf_findETHandle(ring_priv_data->accel_dev,
                                  ring_number,
                                  &trans_handle);
        if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("Ring handle not found, status = %d\n", status);
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EIO;
        }

        ring_data = (icp_et_ring_data_t *)trans_handle->trans_data;

        vm_size = vma->vm_end - vma->vm_start;
        size_to_map = (ring_data->sizeInBytes < PAGE_SIZE) ?
                                         PAGE_SIZE : ring_data->sizeInBytes;
        if (vm_size != size_to_map) {
                ADF_ERROR("invalid size\n");
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EINVAL;
        }
        /*rounded up to a page boundary */
        ring_kmalloc_area = (int *)(
                            (((unsigned long) ring_data->ringBaseAddress) +
                            PAGE_SIZE -1) & PAGE_MASK);

        if ((UARCH_INT)ring_kmalloc_area !=
                (UARCH_INT)ring_data->ringBaseAddress) {
                ADF_ERROR("ring base address and page alignment "
                              "are not the same\n");
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EINVAL;
        }

        ret = remap_pfn_range(vma,
                vma->vm_start,
                virt_to_phys((void*)((unsigned long)ring_kmalloc_area))
                              >> PAGE_SHIFT,
                vma->vm_end-vma->vm_start,
                vma->vm_page_prot);

        if(ret != 0) {
                ADF_ERROR("remap_pfn_range failed\n");
                ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
                return -EAGAIN;
        }
        ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
        return 0;
}

/*
 * adf_ring_open
 * open file operation
 */
STATIC int adf_ring_open(struct inode *inp, struct file *fp)
{
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        Cpa32U size = 0;

        accel_dev = adf_devmgrGetAccelDevByAccelId(MINOR(inp->i_rdev));
        if (NULL == accel_dev) {
                ADF_ERROR("adf_devmgrGetAccelDevByAccelId failed\n");
                return -EACCES;
        }

        if (!(BIT_IS_SET(accel_dev->adfSubsystemStatus,
                         ADF_STATUS_SYSTEM_STARTED))) {
                ADF_ERROR("Device not yet ready.\n");
                return -EACCES;
        }
        ring_priv_data = ICP_ZALLOC_GEN(sizeof(adf_userspace_ring_priv_data_t));
        if (NULL == ring_priv_data) {
                ADF_ERROR("Failed to allocated private data\n");
                return -ENOMEM;
        }

        size = sizeof(Cpa32U) * GET_MAX_BANKS(accel_dev);
        ring_priv_data->rings = ICP_ZALLOC_GEN(size);
        if (NULL == ring_priv_data->rings) {
                ADF_ERROR("Failed to allocated private data rings\n");
                ICP_FREE(ring_priv_data);
                return -ENOMEM;
        }

        size = sizeof(Cpa32U) * GET_MAX_BANKS(accel_dev);
        ring_priv_data->rings_mask.bank_irq_mask = ICP_ZALLOC_GEN(size);
        if (NULL == ring_priv_data->rings_mask.bank_irq_mask) {
                ADF_ERROR("Failed to allocated private data ring masks\n");
                ICP_FREE(ring_priv_data->rings);
                ICP_FREE(ring_priv_data);
                return -ENOMEM;
        }

        size = sizeof(adf_dev_rings_mask_t) +
                (sizeof(Cpa32U) * GET_MAX_BANKS(accel_dev));
        ring_priv_data->read_rings_mask = ICP_ZALLOC_GEN(size);
        if (NULL == ring_priv_data->read_rings_mask) {
                ADF_ERROR("Failed to allocated private data read rings mask\n");
                ICP_FREE(ring_priv_data->rings_mask.bank_irq_mask);
                ICP_FREE(ring_priv_data->rings);
                ICP_FREE(ring_priv_data);
                return -ENOMEM;
        }

        ICP_MUTEX_INIT(&ring_priv_data->ring_data_lock);
        ICP_SPINLOCK_INIT(&ring_priv_data->ring_mask_lock);
        ring_priv_data->accel_dev = accel_dev;
        init_waitqueue_head(&ring_priv_data->ring_wait_queue);
        ring_priv_data->rings_mask_state = ADF_RING_NOT_INITIALIZED;
        ring_priv_data->rings_mask.bank_mask = 0;
        ring_priv_data->rings_mask.ring_mask = 0;
        fp->private_data = ring_priv_data;
        return 0;
}

/*
 * adf_ring_release
 * release file operation
 */
STATIC int adf_ring_release(struct inode *inp, struct file *fp)
{
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        ring_priv_data = (adf_userspace_ring_priv_data_t*)fp->private_data;
        ICP_MUTEX_LOCK(&ring_priv_data->ring_data_lock);
        fp->private_data = NULL;
        ICP_MUTEX_UNLOCK(&ring_priv_data->ring_data_lock);
        ICP_FREE(ring_priv_data->read_rings_mask);
        ICP_FREE(ring_priv_data->rings_mask.bank_irq_mask);
        ICP_FREE(ring_priv_data->rings);
        ICP_MUTEX_UNINIT(&ring_priv_data->ring_data_lock);
        ICP_SPINLOCK_UNINIT(&ring_priv_data->ring_mask_lock);
        ICP_FREE(ring_priv_data);
        return 0;
}

/*
 * adf_ring_read
 * read file operation
 */
STATIC ssize_t adf_ring_read(struct file *fp, char __user *buf,
                             size_t count, loff_t *pos)
{
        int ret = 0;
        adf_userspace_ring_priv_data_t *ring_priv_data = NULL;
        Cpa32U rings_mask_state = 0, i = 0, send = 0;
        adf_dev_rings_mask_t *rings_mask = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        Cpa32U rings_mask_size = 0;

        DECLARE_WAITQUEUE(wait_queue, current);

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        ring_priv_data = (adf_userspace_ring_priv_data_t *)fp->private_data;
        accel_dev = ring_priv_data->accel_dev;
        rings_mask_size = sizeof(adf_dev_rings_mask_t) +
                                sizeof(Cpa32U) * GET_MAX_BANKS(accel_dev);
        ICP_MEMSET(ring_priv_data->read_rings_mask, 0, rings_mask_size);
        rings_mask = (adf_dev_rings_mask_t *)ring_priv_data->read_rings_mask;
        rings_mask->bank_irq_mask = (Cpa32U *)
            (ring_priv_data->read_rings_mask + sizeof(adf_dev_rings_mask_t));
        add_wait_queue(&ring_priv_data->ring_wait_queue, &wait_queue);
        do{
                set_current_state(TASK_INTERRUPTIBLE);
                ICP_SPINLOCK_LOCK_BH(&ring_priv_data->ring_mask_lock);
                /* Keep a local copy of state, bank and ring masks */
                rings_mask_state = ring_priv_data->rings_mask_state;
                rings_mask->bank_mask =
                                 ring_priv_data->rings_mask.bank_mask;
                rings_mask->ring_mask =
                                 ring_priv_data->rings_mask.ring_mask;
                for(i = 0; i < GET_MAX_BANKS(accel_dev); i++)
                {
                        if (ring_priv_data->rings_mask.bank_irq_mask[i] &&
                                      (ring_priv_data->rings[i] &
                                       ring_priv_data->rings_mask.ring_mask)) {
                                rings_mask->bank_irq_mask[i] =
                                   ring_priv_data->rings_mask.bank_irq_mask[i];
                                ring_priv_data->rings_mask.bank_irq_mask[i]=0;
                                send = 1;
                        }
                }
                /* Reset state, bank and ring masks */
                ring_priv_data->rings_mask_state = ADF_RING_NOT_INITIALIZED;
                ring_priv_data->rings_mask.bank_mask = 0;
                ring_priv_data->rings_mask.ring_mask = 0;
                ICP_SPINLOCK_UNLOCK_BH(&ring_priv_data->ring_mask_lock);

                if (rings_mask_state != ADF_RING_NOT_INITIALIZED &&
                                        rings_mask_state != ADF_RING_DELETE) {
                        if (send) {
                                ret = copy_to_user(buf,
                                                 ring_priv_data->read_rings_mask,
                                                 rings_mask_size);
                                if (SUCCESS != ret) {
                                        ADF_ERROR("copy_to_user failed\n");
                                        ret = -EIO;
                                }
                                else {
                                        ret = rings_mask_size;
                                }
                                send = 0;
                                break;
                        }
                }
                else if (rings_mask_state == ADF_RING_DELETE) {
                        ret = -EAGAIN;
                        break;
                }
                if (signal_pending(current)) {
                        ret = -EAGAIN;
                        break;
                }
                schedule();
        } while(1);
        set_current_state(TASK_RUNNING);
        remove_wait_queue(&ring_priv_data->ring_wait_queue, &wait_queue);
        return ret;
}

/*
 * adf_ring_process_notify
 * Notify functions that wakes up a userspace process
 * waiting on the blocking read.
 * This function executes in tasklet context.
 */
void adf_ring_process_notify(icp_et_ring_data_t *ring, Cpa16U irq_mask)
{
        adf_userspace_ring_priv_data_t *ring_priv_data =
            (adf_userspace_ring_priv_data_t*) ring->userSpacePrivateData;

        if (likely(ring_priv_data)) {
                ICP_SPINLOCK_LOCK_BH(&ring_priv_data->ring_mask_lock);
                ring_priv_data->rings_mask_state = ADF_RING_INITIALIZED;
                ring_priv_data->rings_mask.bank_mask |= 1 << ring->bankNumber;
                ring_priv_data->rings_mask.ring_mask |= 1 << ring->ringNumber;
                ring_priv_data->rings_mask.bank_irq_mask[ring->bankNumber]
                                                                   = irq_mask;
                ICP_SPINLOCK_UNLOCK_BH(&ring_priv_data->ring_mask_lock);
                wake_up_interruptible(&ring_priv_data->ring_wait_queue);
        }
}

/*
 * adf_ring_process_finish_notify
 * Notify functions that wakes up a userspace process
 * waiting on the blocking read and makes the reading thread exit.
 */
void adf_ring_process_finish_notify(icp_et_ring_data_t *ring)
{
        adf_userspace_ring_priv_data_t *ring_priv_data =
            (adf_userspace_ring_priv_data_t*) ring->userSpacePrivateData;

        if (ring_priv_data) {
                ICP_SPINLOCK_LOCK_BH(&ring_priv_data->ring_mask_lock);
                ring_priv_data->rings_mask_state = ADF_RING_CLOSING_DOWN;
                ICP_SPINLOCK_UNLOCK_BH(&ring_priv_data->ring_mask_lock);
                wake_up_interruptible(&ring_priv_data->ring_wait_queue);
        }
}

/*
 * adf_ring_process_ring_del_notify
 * Notify functions that wakes up a userspace process
 * waiting on the blocking read and makes the reading thread exit.
 */
void adf_ring_process_ring_del_notify(icp_et_ring_data_t *ring)
{
        adf_userspace_ring_priv_data_t *ring_priv_data =
            (adf_userspace_ring_priv_data_t*) ring->userSpacePrivateData;

        if (ring_priv_data) {
                ICP_SPINLOCK_LOCK_BH(&ring_priv_data->ring_mask_lock);
                ring_priv_data->rings_mask_state = ADF_RING_DELETE;
                ICP_SPINLOCK_UNLOCK_BH(&ring_priv_data->ring_mask_lock);
                wake_up_interruptible(&ring_priv_data->ring_wait_queue);
        }
}
