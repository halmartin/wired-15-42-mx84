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
 * @file adf_dev_bank.c
 *
 * @description
 *      This file contains the ADF code to register the BANK device driver
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
#include "adf_adminreg_mgr.h"
#include "adf_dev_ring_ctl.h"
#include "adf_dev_bank.h"
#include "adf_user_proxy.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "icp_adf_init.h"
#include "icp_adf_cfg.h"
#include "adf_proc_debug.h"
#include <linux/errno.h>
#include <linux/poll.h>

#define ADF_DEV_BANK_NAME            ("/icp_dev_bank")
#define ADF_DEV_BANK_MAX_MINOR       (255)
#define ADF_DEV_BANK_BASE_MINOR      (0)
#define BANK_DATA_AVIAL              0xff00
#define ACCEL_ID_SHIFT               5
#define BANK_IDX_MASK                0x1f;

typedef struct adf_bank_priv_data_s {
        icp_accel_dev_t      *accel_dev;
        int                  bank_id;
        int                  responses_available;
        Cpa32U               csr;   /* the bullet to trigger the epoll*/
        ICP_SPINLOCK         bank_data_lock;
        wait_queue_head_t    bank_wait_queue;
        uint32_t             num_notifies;
        uint32_t             num_notifies_with_resp_available_set;
        uint32_t             num_polls;
        uint32_t             num_polls_with_resp_avail;
        icp_et_ring_bank_data_t *bank_data;

        /* bank polling interface */
        CpaBoolean           poll_bank_enabled;
        struct file         *poll_bank_file;
} adf_userspace_bank_priv_data_t;

/*
 * adf_bank_open
 * open file operation
 */
STATIC int adf_bank_open(struct inode *inp, struct file *fp)
{
        adf_userspace_bank_priv_data_t *priv = NULL;
        icp_et_ring_bank_data_t* bank_data = NULL;
        icp_etr_priv_data_t* etr_priv_data = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        int dev_minor_id = 0;
        int accel_id = 0;
        int bank_id = 0;

        dev_minor_id = MINOR(inp->i_rdev);
        accel_id = dev_minor_id >> ACCEL_ID_SHIFT;
        bank_id = dev_minor_id & BANK_IDX_MASK;

        accel_dev = adf_devmgrGetAccelDevByAccelId(accel_id);
        if (NULL == accel_dev) {
                ADF_ERROR("adf_devmgrGetAccelDevByAccelId failed\n");
                return -EACCES;
        }

        if (!(BIT_IS_SET(accel_dev->adfSubsystemStatus,
                    ADF_STATUS_SYSTEM_STARTED))) {
                ADF_ERROR("Device not yet ready.\n");
                return -EACCES;
        }

        etr_priv_data = (icp_etr_priv_data_t*)accel_dev->pCommsHandle;
        bank_data = &etr_priv_data->banks[bank_id];

        ICP_SPINLOCK_LOCK(&bank_data->bankLock);

        if(NULL != bank_data->bankUserSpacePrivateData) {
                ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                ADF_ERROR("We should follow the limit: one process for one bank.\n");
                return -EINVAL;
        }

        priv = ICP_ZALLOC_GEN(sizeof(adf_userspace_bank_priv_data_t));
        if (NULL == priv) {
                ADF_ERROR("Failed to allocated private data\n");
                ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
                return -ENOMEM;
        }

        ICP_SPINLOCK_INIT(&priv->bank_data_lock);
        priv->accel_dev = accel_dev;
        priv->bank_id = bank_id;
        priv->bank_data = bank_data;
        priv->poll_bank_enabled = CPA_FALSE;
        init_waitqueue_head(&priv->bank_wait_queue);

        bank_data->bankUserSpacePrivateData = priv;

        fp->private_data = priv;

        ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);

        return 0;
}

/*
 * adf_bank_release
 * release file operation
 */
STATIC int adf_bank_release(struct inode *inp, struct file *fp)
{
        adf_userspace_bank_priv_data_t *priv = NULL;
        icp_et_ring_bank_data_t* bank_data = NULL;
        icp_etr_priv_data_t* etr_priv_data = NULL;
        icp_accel_dev_t *accel_dev = NULL;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        priv = (adf_userspace_bank_priv_data_t*)fp->private_data;

        accel_dev = priv->accel_dev;
        etr_priv_data = (icp_etr_priv_data_t*)accel_dev->pCommsHandle;
        bank_data = &etr_priv_data->banks[priv->bank_id];

        ICP_SPINLOCK_LOCK(&bank_data->bankLock);

        ADF_PRINT("[ %s ] Dev: %d, Bank %.2d, file %p, notifies=%d,"
                  " polls=%d, polls_with_resp_avail=%d\n",
                  __func__, accel_dev->accelId, priv->bank_id, fp,
                  priv->num_notifies, priv->num_polls,
                  priv->num_polls_with_resp_avail);

        bank_data->bankUserSpacePrivateData = NULL;

        fp->private_data = NULL;
        ICP_FREE(priv);

        ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
        return 0;
}

 /*
  * adf_bank_poll
  * Non-blocking IO support for polling for activity on a ring.
  */
STATIC unsigned int adf_bank_poll(struct file *fp, poll_table *wait)
{
        unsigned int mask = 0;
        adf_userspace_bank_priv_data_t *priv = NULL;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }

        priv = (adf_userspace_bank_priv_data_t *)fp->private_data;

        poll_wait(fp, &priv->bank_wait_queue, wait);

        ICP_SPINLOCK_LOCK_BH(&priv->bank_data_lock);

        if (priv->csr & BANK_DATA_AVIAL) {
                priv->csr = 0; /* reset it */
                mask |= POLLIN | POLLRDNORM; /* readable */
                priv->num_polls_with_resp_avail++;
        }
        priv->num_polls++;

        ICP_SPINLOCK_UNLOCK_BH(&priv->bank_data_lock);

        return mask;
}

/*
 * Character dev file operations
 */
STATIC struct file_operations adf_bank_ops = {
        owner:THIS_MODULE,
        open:adf_bank_open,
        release:adf_bank_release,
        poll:adf_bank_poll,
};

/*
 * Character driver info
 */
STATIC adf_chr_drv_info_t adf_bank_drv_info = {
        owner:THIS_MODULE,
        major:0,
        min_minor:ADF_DEV_BANK_BASE_MINOR,
        max_minor:ADF_DEV_BANK_MAX_MINOR,
        name:ADF_DEV_BANK_NAME,
        file_ops:&adf_bank_ops,
};

/*
 * adf_bank_chrdev_register
 * Function creates character device interface
 */
int adf_bank_chrdev_register(void)
{
        int ret = 0;

        ret = adf_chr_drv_create(&adf_bank_drv_info);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create BANK device driver\n");
                return FAIL;
        }

        return SUCCESS;
}

/*
 * adf_bank_chrdev_create
 * Function creates bank character device instances in /dev filesystem
 */
int adf_bank_chrdev_create(int accel_id)
{
        int bank_id = 0;
        struct device *drv_device = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        int dev_minor = 0;

        accel_dev = adf_devmgrGetAccelDevByAccelId(accel_id);
        if (NULL == accel_dev) {
                ADF_ERROR("adf_devmgrGetAccelDevByAccelId failed\n");
                return -EACCES;
        }

        for (; bank_id < GET_MAX_BANKS(accel_dev); ++bank_id) {
                dev_minor = (accel_id << ACCEL_ID_SHIFT) | bank_id;

                drv_device = device_create(adf_bank_drv_info.drv_class,
                                NULL, MKDEV(adf_bank_drv_info.major, dev_minor),
                                NULL, ADF_DEV_BANK_NAME_FMT, accel_id, bank_id);
                if (!drv_device) {
                        ADF_ERROR("failed to create device %d bank %d\n",
                                accel_id, bank_id);
                        return CPA_STATUS_FAIL;
                }

                adf_bank_drv_info.num_devices++;
        }
        return CPA_STATUS_SUCCESS;
}

/*
 * adf_bank_chrdev_unregister
 * Function unregisters character device interface
 */
void adf_bank_chrdev_unregister(void)
{
        adf_chr_drv_destroy(&adf_bank_drv_info);
}

/*
 * adf_bank_process_notify
 * Notify functions that wakes up the userspace processes waiting on the waitq.
 * This function executes in tasklet context.
 */
void adf_bank_process_notify(icp_et_ring_bank_data_t *bank, Cpa32U csr)
{
        adf_userspace_bank_priv_data_t *priv = NULL;

        if (!bank) {
                ADF_ERROR("empty bank\n");
                return;
        }

        priv = bank->bankUserSpacePrivateData;
        if (likely(priv)) {
                ICP_SPINLOCK_LOCK_BH(&priv->bank_data_lock);
                priv->num_notifies++;
                if(priv->csr & BANK_DATA_AVIAL) {
                    priv->num_notifies_with_resp_available_set++;
                }
                priv->csr |= csr; /* fill the bullet */

                ICP_SPINLOCK_UNLOCK_BH(&priv->bank_data_lock);

                wake_up_interruptible(&priv->bank_wait_queue);
        }
}
