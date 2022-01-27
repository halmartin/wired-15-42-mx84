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
 * @file adf_adminreg_mgr.c
 *
 * @description
 *      Implementation of a transport mechanism for admin/init messages using
 *      status registers and a coherent DMA memory zone in the host memory.
 *
 *      That memory zone is divided into "message slots", each acceleration
 *      engine having its own slot.
 *      Each slot is split into a RX and a TX buffer (named from the CPU's
 *      point of view in the code below).
 *      Read/Write on these buffers are controlled via bits in 2 registers:
 *      ADMINMSG_TRIGGER and ADMINMSG_STATUS.
 *
 *      This feature first appeared in DH895xCC.
 *
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_adminreg_mgr.h"
#include "icp_adf_cfg.h"
#include "adf_platform_common.h"
#include "adf_platform_dh895x.h"

struct icp_adminreg_msg_slot
{
    Cpa8U ae_number;
    volatile Cpa32U *mailbox0_addr;
    unsigned long tx_kv_addr;
    unsigned long rx_kv_addr;
    unsigned int  tx_msg_size;
    unsigned int  rx_msg_size;
    icp_trans_callback callback;
    ICP_MUTEX   lock;
};


/**
 * Internal transport function.
 * These are registered to the transport manager and accessed through it.
 */
static CpaStatus create_handle(icp_accel_dev_t *, const char *, Cpa32U,
                               Cpa32U, icp_trans_handle **,
                               icp_adf_ringInfoService_t, const char *,
                               const Cpa32U, const Cpa32U, Cpa32U);

static CpaStatus release_handle(icp_trans_handle *);
static CpaStatus put_msg_sync(icp_trans_handle *trans_handle,Cpa32U *requestBuffer,
                              Cpa32U *responseBuffer, Cpa32U bufsLen);


/**
 * Internal helper functions.
 */
static void notify_AE(struct icp_adminreg_msg_slot *);
static CpaStatus wait_for_AE_response(struct icp_adminreg_msg_slot *);

/**
 * The actual transport manager.
 * It is registered to the transport controller after being initialized.
 */
static icp_trans_mgr adminreg_mgr;

/*
 * These two values go hand in hand - wait for a response from an AE for
 * (RESPONSE_POLL_FREQ_MS*MAX_POLL_RETRIES) msecs.
 */
#define RESPONSE_POLL_FREQ_MS      10
#define MAX_POLL_RETRIES           200


CpaStatus adf_init_adminregManager(void)
{
    adminreg_mgr.trans_type     = ICP_TRANS_TYPE_ADMINREG;
    adminreg_mgr.create_handle  = create_handle;
    adminreg_mgr.release_handle = release_handle;
    adminreg_mgr.find_handle    = NULL;
    adminreg_mgr.get_ring_num   = NULL;

    return adf_trans_registerTransMgr(&adminreg_mgr);
}

CpaStatus adf_shutdown_adminregManager(void)
{
    return adf_trans_deregisterTransMgr(&adminreg_mgr);
}

static CpaStatus create_handle(icp_accel_dev_t * accel_dev,
                               const char *section,
                               Cpa32U ae_nr,
                               Cpa32U bank_nr,
                               icp_trans_handle **trans_handle,
                               icp_adf_ringInfoService_t info,
                               const char *service_name,
                               const Cpa32U num_msgs,
                               const Cpa32U msg_size,
                               Cpa32U flags)
{
    adf_hw_device_data_t *hw_data;

    Cpa32U num_aes;
    Cpa32U pmisc_bar_id;
    unsigned long tx_kv_addr, pmisc_bar_addr, offset;
    struct icp_adminreg_msg_slot *msg_slot;
    Cpa32U msgSizeTx, msgSizeRx;
    char paramStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    CpaStatus status = CPA_STATUS_SUCCESS;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);

    hw_data    = accel_dev->pHwDeviceData;
    ICP_CHECK_FOR_NULL_PARAM(hw_data);
    num_aes = GET_MAX_ACCELENGINES(accel_dev);

    ICP_CHECK_PARAM_RANGE(ae_nr,  0, num_aes);


    /* Check if the device is equipped with the needed facilities */
    if (hw_data->isETRlessAdminInitCommSupported != CPA_TRUE)
    {
        ADF_ERROR("This device doesn't support the 'adminreg' transport.\n");
        return CPA_STATUS_FAIL;
    }

    /* Check if the DMA memory for the message slots is allocated */
    if (!accel_dev->pMessageSlots)
    {
        ADF_ERROR("accel_dev->pMessageSlots is NULL.");
        ADF_ERROR("Something went wrong during device initialization.\n");
        return CPA_STATUS_FAIL;
    }

    /* read the msg size from the config table */
    status = icp_adf_cfgGetParamValue(accel_dev,
            GENERAL_SEC,
            ICP_CFG_FW_MSG_SIZE_ADMIN_TX_KEY,
            paramStr);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Problem getting ADMIN_TX size from config table");
        return CPA_STATUS_FAIL;
    }
    msgSizeTx = ICP_STRTOUL(paramStr, NULL, ADF_CFG_BASE_DEC);

    status = icp_adf_cfgGetParamValue(accel_dev,
            GENERAL_SEC,
            ICP_CFG_FW_MSG_SIZE_ADMIN_RX_KEY,
            paramStr);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Problem getting ADMIN_RX size from config table");
        return CPA_STATUS_FAIL;
    }
    msgSizeRx = ICP_STRTOUL(paramStr, NULL, ADF_CFG_BASE_DEC);

    /* Allocate and zero out a new trans_handle structure */
    *trans_handle = ICP_ZALLOC_GEN(sizeof(icp_trans_handle));

    if (*trans_handle == NULL)
    {
        ADF_ERROR("Unable to allocate memory for trans_handle\n");
        return CPA_STATUS_FAIL;
    }

    /* Set up the new handle */
    (*trans_handle)->accel_dev = accel_dev;
    (*trans_handle)->handle_id = ae_nr;
    (*trans_handle)->trans_type = ICP_TRANS_TYPE_ADMINREG;
    (*trans_handle)->put_msg = NULL;
    (*trans_handle)->put_msg_sync = put_msg_sync;
    (*trans_handle)->reg_callback = NULL;
    (*trans_handle)->notify = NULL;
    (*trans_handle)->polling_notify = NULL;


    /* Allocate and zero out the message slot structure associated with the handle */
    (*trans_handle)->trans_data = ICP_ZALLOC_GEN(sizeof(*msg_slot));

    if ((*trans_handle)->trans_data == NULL)
    {
        ADF_ERROR("Unable to allocate memory for trans_handle->trans_data\n");
        ICP_FREE(*trans_handle);
        return CPA_STATUS_FAIL;
    }

    /* Set up the new message slot */
    msg_slot = (*trans_handle)->trans_data;

    /* Get the memory location of the tx/rx buffer for that AE */
    offset = (ae_nr * (msgSizeTx+msgSizeRx));
    tx_kv_addr = (unsigned long) (accel_dev->pMessageSlots) + offset;

    msg_slot->ae_number   = ae_nr;
    msg_slot->tx_kv_addr  = tx_kv_addr;
    msg_slot->rx_kv_addr  = tx_kv_addr + msgSizeTx;
    msg_slot->tx_msg_size = msgSizeTx;
    msg_slot->rx_msg_size = msgSizeRx;
    msg_slot->callback    = NULL;

    /* Get the memory location of this AE's mailbox0 register */
    pmisc_bar_id   = hw_data->getMiscBarId(hw_data);
    pmisc_bar_addr = accel_dev->pciAccelDev.pciBars[pmisc_bar_id].virtAddr;

    msg_slot->mailbox0_addr = (Cpa32U *) (pmisc_bar_addr
                                          + ICP_DH895xCC_MAILBOX0_BASE_OFFSET
                                          + ae_nr * ICP_DH895xCC_MAILBOX_STRIDE);

    /* Initialize the slot's lock */
    ICP_MUTEX_INIT(&msg_slot->lock);
    return CPA_STATUS_SUCCESS;
}

static CpaStatus release_handle(icp_trans_handle *trans_handle)
{
    struct icp_adminreg_msg_slot *msg_slot;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(trans_handle->trans_data);

    msg_slot = (struct icp_adminreg_msg_slot *) trans_handle->trans_data;

    ICP_MUTEX_UNINIT(&msg_slot->lock);

    ICP_FREE(msg_slot);
    ICP_FREE(trans_handle);

    return CPA_STATUS_SUCCESS;
}

static CpaStatus put_msg_sync(icp_trans_handle *trans_handle,Cpa32U *requestBuffer,
                              Cpa32U *responseBuffer, Cpa32U bufsLen)
{
    CpaStatus status;
    struct icp_adminreg_msg_slot *msg_slot;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(requestBuffer);
    ICP_CHECK_FOR_NULL_PARAM(responseBuffer);

    msg_slot = (struct icp_adminreg_msg_slot *) (trans_handle->trans_data);

    if (bufsLen > msg_slot->tx_msg_size)
    {
        ADF_ERROR("The message is bigger than the message slot\n");
        return CPA_STATUS_FAIL;
    }

    /* Acquire the slot's lock */
    if(ICP_MUTEX_LOCK(&msg_slot->lock))
    {
        ADF_ERROR("Unable to acquire admin slot lock\n");
        return CPA_STATUS_FAIL;
    }

    /* Make sure the last transaction completed and the slot is available for use.
     * If it didn't the FW is probably hung - no point in sending it another msg. */
    if (*(msg_slot->mailbox0_addr) == 1)
    {
        ADF_ERROR("The msg slot never cleared after last request msg\n");
        ICP_MUTEX_UNLOCK(&msg_slot->lock);
        return CPA_STATUS_RETRY;
    }

    ICP_MEMCPY((void *) (msg_slot->tx_kv_addr), requestBuffer, bufsLen);

    notify_AE(msg_slot);

    status = wait_for_AE_response(msg_slot);

    if (CPA_STATUS_SUCCESS == status)
    {
        ICP_MEMCPY(responseBuffer, (void *)msg_slot->rx_kv_addr, bufsLen);
    }
    else if (CPA_STATUS_FAIL == status)
    {
        ADF_ERROR("Failed to receive an admin response from AE.\n");
    }

    ICP_MUTEX_UNLOCK(&msg_slot->lock);

    return status;
}


/**
 * Put a non zero value in the mailbox0 register of the targeted AE to
 * notify it of a new message from the CPU that's ready to be read
 */
static void notify_AE(struct icp_adminreg_msg_slot *msg_slot)
{
    *(msg_slot->mailbox0_addr) = 1;
}


/**
 * Poll the mailbox0 register until the AE resets it, meaning that
 * the CPU can read the response message
 */
static CpaStatus wait_for_AE_response(struct icp_adminreg_msg_slot *msg_slot)
{
    Cpa16U times = 0;

    for (times=0; times<MAX_POLL_RETRIES; times++)
    {
        if (OSAL_SUCCESS != osalSleep(RESPONSE_POLL_FREQ_MS))
        {
            return CPA_STATUS_RESOURCE;
        }

        if (*(msg_slot->mailbox0_addr) == 0)
        {
            return CPA_STATUS_SUCCESS;
        }
    }
    return CPA_STATUS_FAIL;
}

/**
 *
 */
CpaStatus adf_sendInitReq(icp_accel_dev_t *accel_dev, Cpa16U target_AE,
                          void *requestBuffer, void *responseBuffer)
{
    CpaStatus status;
    icp_trans_handle *handle;
    char paramStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U msgSizeTx;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(requestBuffer);
    ICP_CHECK_FOR_NULL_PARAM(responseBuffer);

    status = icp_adf_cfgGetParamValue(accel_dev,
            GENERAL_SEC,
            ICP_CFG_FW_MSG_SIZE_ADMIN_TX_KEY,
            paramStr);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Problem getting ADMIN_TX size from config table");
        return CPA_STATUS_FAIL;
    }
    msgSizeTx = ICP_STRTOUL(paramStr, NULL, ADF_CFG_BASE_DEC);

    status = create_handle(accel_dev, NULL,
                           target_AE, 0,
                           &handle,
                           (icp_adf_ringInfoService_t) NULL, NULL,
                           1, msgSizeTx,
                           0);

    if (status != CPA_STATUS_SUCCESS)
        return CPA_STATUS_FAIL;

    status = put_msg_sync(handle, requestBuffer, responseBuffer, msgSizeTx);

    if (status != CPA_STATUS_SUCCESS)
    {
        release_handle(handle);
        return CPA_STATUS_FAIL;
    }

    status = release_handle(handle);

    if (status != CPA_STATUS_SUCCESS)
        return CPA_STATUS_FAIL;

    return CPA_STATUS_SUCCESS;
}

/*
 * Initialize the various registers and mailboxes used by the admin interface.
 * This must be called before the FW is loaded.
 */
CpaStatus
adf_initAdminComms(icp_accel_dev_t *accel_dev)
{
    Cpa32U num_aes;
    Cpa32U pmisc_bar_id;
    unsigned long pmisc_bar_addr;
    int i;
    adf_hw_device_data_t *hw_data;

    hw_data = accel_dev->pHwDeviceData;
    num_aes = hw_data->getNumAccelEngines(hw_data, accel_dev->aeMask);
    pmisc_bar_id   = hw_data->getMiscBarId(hw_data);
    pmisc_bar_addr = accel_dev->pciAccelDev.pciBars[pmisc_bar_id].virtAddr;

    /* Write the message slots' bus address to the appropriate registers */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ADMINMSGUR_OFFSET,
                   (Cpa32U)(accel_dev->pMessageSlotsDma >> 32));
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ADMINMSGLR_OFFSET,
                   (Cpa32U)(accel_dev->pMessageSlotsDma));

    /* Zero the mailboxes */
    for (i = 0; i < num_aes; i++)
    {
        *(Cpa32U *)(pmisc_bar_addr
                    + ICP_DH895xCC_MAILBOX0_BASE_OFFSET
                    + i * ICP_DH895xCC_MAILBOX_STRIDE) = 0;
    }

    return CPA_STATUS_SUCCESS;
}

