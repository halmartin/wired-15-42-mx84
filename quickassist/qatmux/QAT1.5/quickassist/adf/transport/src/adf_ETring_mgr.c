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
 * @file adf_ETring_mgr.c
 *
 * @description
 *      ET Ring Manager
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "icp_adf_cfg.h"
#include "adf_cfg.h"
#include "adf_dev_ring.h"
#include "adf_poll_tasklets.h"
#include "adf_init.h"
#include "adf_ETring_ap.h"

/*Default and max time interval for coalescing. */
#define COALESCING_TIME_INTERVAL 10000
#define MIN_COALESCING_TIME_INTERVAL 500
#define MAX_COALESCING_TIME_INTERVAL 0xFFFFF
#define RING_CREATE_RETRY_TIMES 10
#define RING_CREATE_SLEEP_TIME  50

/* Minimum ring size in bytes that will keep it ring size alligned */
#define MIN_RING_SIZE_IN_BYTES          0x1000

/* Local instance of the Transport Manager for ET Rings */
STATIC icp_trans_mgr ETRingMgr;

/*
 * Get free ring on the device and reserve it.
 */
STATIC Cpa32U adf_reserveFreeRing(icp_et_ring_bank_data_t* bank, Cpa32U ring_nr)
{
    Cpa32U ring_mask = 0;
    ICP_CHECK_FOR_NULL_PARAM(bank);

    /* Start of CRITICAL SECTION */
    ICP_SPINLOCK_LOCK(&bank->bankLock);
    /* Check if there are any free rings */
    if ((~bank->allocatedRings) == 0)
    {
        ICP_SPINLOCK_UNLOCK(&bank->bankLock);
        return 0;
    }
    /* return 0 if ring already allocated */
    if (bank->allocatedRings & RING_NUMBER_TO_ID(ring_nr))
    {
        ICP_SPINLOCK_UNLOCK(&bank->bankLock);
        return 0;
    }
    ring_mask = 1 << ring_nr;
    /* Mark the ring as allocated */
    bank->allocatedRings |= ring_mask;
    /* End of CRITICAL SECTION */
    ICP_SPINLOCK_UNLOCK(&bank->bankLock);
    return ring_mask;
}

/*
 * Unreserve the ring on the device
 */
STATIC void unreserveRing(icp_et_ring_bank_data_t* bank, Cpa32U ringID)
{
    /* Start of CRITICAL SECTION */
    ICP_SPINLOCK_LOCK(&bank->bankLock);
    bank->allocatedRings &= ~ringID;
    /* End of CRITICAL SECTION */
    ICP_SPINLOCK_UNLOCK(&bank->bankLock);
}


/*
 * Sets InUse flag for this ring so dyn or wireless inst can use it.
 * Ring must be allocated already.
 *
 */
CpaStatus adf_setRingInUse(icp_et_ring_bank_data_t* bank_data,
                                                    Cpa32U ring_number)
{
    CpaBoolean gotRing = CPA_FALSE;
    Cpa32U count = 0;
    CpaStatus status = CPA_STATUS_FAIL;

    /* Start of CRITICAL SECTION */
    ICP_SPINLOCK_LOCK(&bank_data->bankLock);

    /* First make sure ring is allocated for use */
    if (bank_data->allocatedRings & (1<<ring_number))
    {
        do
        {
            if (count > 0)
            {
                /*The first time we have the lock, on subsequent
                 * loops it's been released so must be got here  */
                ICP_SPINLOCK_LOCK(&bank_data->bankLock);
            }

            if (!(bank_data->ringsInUseMask & (1<<ring_number)))
            {
               /* Ring is not InUse so mark as "InUse" */
               bank_data->ringsInUseMask |= (1<<ring_number);
               /* End of CRITICAL SECTION */
               ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
               gotRing = CPA_TRUE;
            }
            else
            {
               /* ring is InUse */
               ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);

               /* wait in case orphan thread is cleaning it */
               ICP_MSLEEP(RING_CREATE_SLEEP_TIME);
            }

        } while(!gotRing && count++ < RING_CREATE_RETRY_TIMES);


        if (gotRing)
        {
            status = CPA_STATUS_SUCCESS;
        }
        else
        {
            ADF_ERROR("Timed out waiting to set ring InUse. B%d R%d\n",
                    bank_data->bankNumber, ring_number);
            status = CPA_STATUS_FAIL;

        }
    }
    else
    {
        ICP_SPINLOCK_UNLOCK(&bank_data->bankLock);
        ADF_ERROR("Ring not allocated so failed to set InUse. B%d R%d\n",
                                     bank_data->bankNumber, ring_number);
        status = CPA_STATUS_FAIL;
    }

    return status;
}


/*
 * Put messages onto the ET Ring
 */
STATIC CpaStatus putETMsgs(icp_trans_handle *trans_handle,
                           Cpa32U *inBuf,
                           Cpa32U bufLen)
{
    icp_et_ring_data_t *ringData = NULL;

    Cpa32U* targetAddr = NULL;
    Cpa32U* csr_base_addr = NULL;
    Cpa32U  req_ring_index;
    OsalAtomic  *pRingInflight;
    icp_accel_dev_t *accel_dev = NULL;
    INT64 flight;

    ringData = (icp_et_ring_data_t *) trans_handle->trans_data;
    accel_dev = (icp_accel_dev_t*) trans_handle->accel_dev;
    csr_base_addr = ringData->ringCSRAddress;
    pRingInflight = (OsalAtomic*)accel_dev->pRingInflight;

    ICP_SPINLOCK_LOCK_BH(&ringData->ringLock);
    /* Check if there is enough space in the ring */
    req_ring_index = ringData->inFlightIndex;
    flight = osalAtomicInc(pRingInflight + req_ring_index);
    if (flight > ringData->maxRequestsInflight)
    {
        osalAtomicDec(pRingInflight + req_ring_index);
        ICP_SPINLOCK_UNLOCK_BH(&ringData->ringLock);
        return CPA_STATUS_RETRY;
    }

      /* We have enough space - write the message */
    targetAddr = (Cpa32U*)
                     (((UARCH_INT)ringData->ringBaseAddress)
                     + ringData->tail);

    ICP_MEMCPY(targetAddr, inBuf, ICP_ET_DEFAULT_MSG_SIZE);

    /* Update shadow copy values */
    ringData->tail = modulo((ringData->tail + ICP_ET_DEFAULT_MSG_SIZE),
                              ringData->modulo);

    /* Data written - update CSR */
    WRITE_CSR_RING_TAIL(ringData->bankNumber,
                        ringData->ringNumber,
                        ringData->tail);
    ICP_SPINLOCK_UNLOCK_BH(&ringData->ringLock);
    return CPA_STATUS_SUCCESS;
}

/*
 * Hooks for clients to add their callback to the handle
 */
STATIC CpaStatus regETCallback(icp_trans_handle *trans_handle,
                               icp_trans_callback callback)
{
    icp_et_ring_data_t *ringData = NULL;
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(callback);

    ringData = (icp_et_ring_data_t *) trans_handle->trans_data;
    ringData->callbackFn = callback;
    return CPA_STATUS_SUCCESS;
}

/*
 * Notifies the transport handle in question.
 */
STATIC CpaStatus notifyETMsg(icp_trans_handle *trans_handle)
{
    /*
     * Algorithm:
     * (1) Check if current message is empty message or not
     * (2) Invoke CB for each message on the ring
     * (3) Update inflight number
     * (4) Advance Ring Head CSR by amount of data removed from ring
     *
     * Note:
     * - No locking is performed since it is assumed that this function
     *   is being called in the context of a tasklet and that there is
     *   a singled consumer of the ring. If these are incorrect assumptions
     *   then a locked form of this function will be required.
     * - It is assumed that the ring contains at least one message.
     *   N.B. This means that this method should not be used in its current
     *   form for polling.
     * - It is assumed that the size of the ring is an integer multiple
     *   of the size of the messages on the ring.
     */

    volatile Cpa32U *msg = NULL;
    Cpa32U* csr_base_addr = NULL;
    icp_et_ring_data_t *ringData = NULL;
    icp_accel_dev_t *accel_dev = NULL;
    icp_etr_priv_data_t *etr_priv_data = NULL;
    Cpa32U msg_counter=0;
    OsalAtomic *pRingInflight;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ringData = (icp_et_ring_data_t *) trans_handle->trans_data;
    accel_dev = (icp_accel_dev_t*) trans_handle->accel_dev;
    ICP_CHECK_FOR_NULL_PARAM(ringData->ringCSRAddress);
    csr_base_addr = ringData->ringCSRAddress;

    /* Check if the handler is in the user space */
    if(ringData->userSpaceRing)
    {
        /* If the parent userspace process still exists,
         * then notify the process. */
        if(likely(!ringData->orphanRing))
        {
            Cpa16U irq_mask = 0;
            icp_et_ring_bank_data_t *bank;
            if(unlikely(! BIT_IS_SET(accel_dev->adfSubsystemStatus,
                             ADF_STATUS_SYSTEM_STARTED)))
            {
                /* subsystem is going down - dont send notify to userspace */
                return CPA_STATUS_SUCCESS;
            }
            etr_priv_data = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;
            bank = &etr_priv_data->banks[ringData->bankNumber];
            irq_mask =  bank->interruptMask;
            irq_mask |= bank->interruptUserMask;
            bank->interruptPendingUserMask |= (1 << ringData->ringNumber);
            irq_mask |= bank->interruptPendingUserMask;
            adf_ring_process_notify(ringData, irq_mask);
        }
        /* if the parent userspace process doesn't exist */
        else
        {
         /* This is a response to an orphan ring no one is waiting for it
             * as the userspace parent died so just need to adjust the head
             * to tail and return */
            ringData->tail = READ_CSR_RING_TAIL(ringData->bankNumber,
                                        ringData->ringNumber);

            ringData->head = ringData->tail;

            WRITE_CSR_RING_HEAD(ringData->bankNumber,
                        ringData->ringNumber, ringData->head);

            /* increment the orphan response counter
             * This is needed for the userspace proxy which handles
             * the orphan rings - if there will be no activity on the ring
             * userspace proxy will remove the rings for a process
             * If there in some activity on the ring we can not clean it,
             * because the acceleration engine is still using it */
            ringData->orphanResponseCount++;
        }
        return CPA_STATUS_SUCCESS;
    }

    pRingInflight = (OsalAtomic*)accel_dev->pRingInflight +
        ringData->inFlightIndex;
    msg = (Cpa32U*)
              (((UARCH_INT)ringData->ringBaseAddress) + ringData->head);
    while(*msg != EMPTY_RING_ENTRY_SIG)
    {
        adf_process_msg(trans_handle, &msg, &msg_counter, pRingInflight);
    }
       /* Update the head CSR if any messages were processed */
    if(msg_counter > 0)
    {
        /* May need to do this earlier to prevent perf impact in
            multi-threaded scenarios */
        WRITE_CSR_RING_HEAD(ringData->bankNumber,
                     ringData->ringNumber,
                     ringData->head);
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Notify function used for polling. Messages are read until the ring is
 * empty or the response quota has been fulfilled. The response quota
 * for the ring is updated at the end to reflect the number of messages read.
 * If the response quota is zero, messages are read until the ring is drained.
 */
CpaStatus notifyETMsgForPolling(icp_trans_handle *trans_handle,
                                       Cpa32U response_quota)
{
   /*
     * Algorithm:
     * (1) Read the ring message, and check if this is an empty msg
     * (2) Invoke CB for each message on the ring
     * (3) Update inflight number for ring
     * (4) Advance Ring Head CSR by amount of data removed from ring
     *
     * Note:
     * - No locking is performed since it is assumed that this function
     *   is being called in the context of a tasklet and that there is
     *   a single consumer of the ring. If these are incorrect assumptions
     *   then a locked form of this function will be required.
     * - It is assumed that the size of the ring is an integer multiple
     *   of the size of the messages on the ring.
     */

    volatile Cpa32U *msg = NULL;
    Cpa32U* csr_base_addr = NULL;
    icp_et_ring_data_t *ringData = NULL;
    Cpa32U msg_counter=0;
    OsalAtomic *pRingInflight = NULL;

    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*)
        trans_handle->accel_dev;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ringData = (icp_et_ring_data_t *) trans_handle->trans_data;

    ICP_CHECK_FOR_NULL_PARAM(ringData->ringCSRAddress);
    csr_base_addr = ringData->ringCSRAddress;

    pRingInflight = (OsalAtomic*)accel_dev->pRingInflight +
        ringData->inFlightIndex;

    ICP_CHECK_FOR_NULL_PARAM(pRingInflight);

    if(response_quota == 0)
    {
        response_quota = ICP_NO_RESPONSE_QUOTA;
    }

     /* Setup the pointer to the message on the ring */
    msg = (Cpa32U*)
              (((UARCH_INT)ringData->ringBaseAddress) + ringData->head);
    while ((*msg != EMPTY_RING_ENTRY_SIG) && (msg_counter < response_quota))
    {
        adf_process_msg(trans_handle, &msg, &msg_counter, pRingInflight);
    }
     /* Update the head CSR if any messages were processed */
    if(msg_counter > 0)
    {
        /* May need to do this earlier to prevent perf impact in
            multi-threaded scenarios */
        WRITE_CSR_RING_HEAD(ringData->bankNumber,
                     ringData->ringNumber,
                     ringData->head);
    }
    return CPA_STATUS_SUCCESS;
}


/*
 * Searches the passed in trans_handle list for the specific
 * handler
 */
STATIC CpaStatus findETHandleInBank(icp_et_ring_bank_data_t* bank,
                                    Cpa32U numRingsInBank,
                                    icp_trans_identifier trans_id,
                                    icp_trans_handle** trans_handle)
{
    icp_trans_handle* tmpHandle = NULL;
    Cpa32U ringNum = 0;

    for (ringNum = 0; ringNum < numRingsInBank; ringNum++)
    {
        if ((RING_NUMBER_TO_ID(ringNum) & bank->allocatedRings) != 0)
        {
            tmpHandle = bank->rings[ringNum].parent;
            if (tmpHandle != NULL)
            {
                if (trans_id == tmpHandle->handle_id)
                {
                    *trans_handle = tmpHandle;
                    return CPA_STATUS_SUCCESS;
                }
            }
        }
    }
    return CPA_STATUS_FAIL;
}

/*
 * Find an ET Ring Handle from the trans_id
 */
CpaStatus adf_findETHandle(icp_accel_dev_t *accel_dev,
                              icp_trans_identifier trans_id,
                              icp_trans_handle** trans_handle)
{
    icp_etr_priv_data_t *privData = NULL;
    Cpa32S i = 0;
    Cpa32U maxBanks = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pCommsHandle);
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(GET_HW_DATA(accel_dev));

    privData = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;

    /* Need to iterate through each list of rings
     * to see if we can find the handle */
    maxBanks = GET_MAX_BANKS(accel_dev);
    for (i = 0; i < maxBanks; i++)
    {
        if (findETHandleInBank(&privData->banks[i],
                GET_NUM_RINGS_PER_BANK(accel_dev), trans_id, trans_handle)
            == CPA_STATUS_SUCCESS)
        {
            return CPA_STATUS_SUCCESS;
        }
    }
    return CPA_STATUS_FAIL;
}

/*
 * Calculate nearly full/nearly empty values and set irq source mask.
 * We need to validate the conf_val value read from the config file
 * and find the closest valid value and base on the value set the
 * nearly empty or nearly full watermark and nearly empty going
 * false or nearly full going true value for IAIntSrcSel register.
 * Note: We want to get an IRQ when NE becomes not true or
 * when the NF becomes true.
 */
STATIC void calculate_nf_ne_val(Cpa32U conf_val, Cpa8U *nearly_empty,
                                                 Cpa8U *nearly_full,
                                                 Cpa8U *irq_src_val)
{
    if(NEAR_EMPTY_CONFIG_VAL_0 > conf_val)
    {
        *nearly_empty = NEAR_EMPTY_WATERMARK_VAL_0;
        *irq_src_val = INT_SRC_N_EMPTY_FALSE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_0 < conf_val &&
            NEAR_EMPTY_CONFIG_VAL_1 >= conf_val)
    {
        *nearly_empty = NEAR_EMPTY_WATERMARK_VAL_1;
        *irq_src_val = INT_SRC_N_EMPTY_FALSE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_1 < conf_val &&
            NEAR_EMPTY_CONFIG_VAL_2 >= conf_val)
    {
        *nearly_empty = NEAR_EMPTY_WATERMARK_VAL_2;
        *irq_src_val = INT_SRC_N_EMPTY_FALSE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_2 < conf_val &&
            NEAR_EMPTY_CONFIG_VAL_3 >= conf_val)
    {
        *nearly_empty = NEAR_EMPTY_WATERMARK_VAL_3;
        *irq_src_val = INT_SRC_N_EMPTY_FALSE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_3 < conf_val &&
            NEAR_EMPTY_CONFIG_VAL_4 >= conf_val)
    {
        *nearly_empty = NEAR_EMPTY_WATERMARK_VAL_4;
        *irq_src_val = INT_SRC_N_EMPTY_FALSE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_4 < conf_val &&
            NEAR_EMPTY_CONFIG_VAL_5 >= conf_val)
    {
        *nearly_empty = NEAR_EMPTY_WATERMARK_VAL_5;
        *irq_src_val = INT_SRC_N_EMPTY_FALSE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_5 < conf_val &&
            NEAR_EMPTY_CONFIG_VAL_6 >= conf_val)
    {
        *nearly_full = NEAR_FULL_WATERMARK_VAL_6;
        *irq_src_val = INT_SRC_N_FULL_TRUE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_6 < conf_val &&
            NEAR_EMPTY_CONFIG_VAL_7 >= conf_val)
    {
        *nearly_full = NEAR_FULL_WATERMARK_VAL_7;
        *irq_src_val = INT_SRC_N_FULL_TRUE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_7 < conf_val &&
            NEAR_EMPTY_CONFIG_VAL_8 >= conf_val)
    {
        *nearly_full = NEAR_FULL_WATERMARK_VAL_8;
        *irq_src_val = INT_SRC_N_FULL_TRUE;
    }
    else if(NEAR_EMPTY_CONFIG_VAL_8 < conf_val)
    {
        *nearly_full = NEAR_FULL_WATERMARK_VAL_9;
        *irq_src_val = INT_SRC_N_FULL_TRUE;
    }
}

/*
 * Take size in number of entries and give back valid
 * config number number to write it to the dev.
 */
STATIC Cpa32U validateRingSize(Cpa32U size_in_msg, Cpa32U *modulo)
{
    Cpa32U size_config = ICP_ET_DEFAULT_RING_SIZE;
    Cpa32U size_in_bytes = size_in_msg * ICP_ET_DEFAULT_MSG_SIZE;
    Cpa32U k2bytes=0;
    Cpa32U k2_size=0;

      /* first right shift 11 */
    k2bytes = ((size_in_bytes-1) >> MODULO_SHIFT_FOR_2K);
    while (k2bytes > 0) {
          k2bytes = k2bytes >> 1;
          k2_size++;
    }
    size_config = ICP_RINGSIZE_KILO_2 + k2_size;
    *modulo = MODULO_SHIFT_FOR_2K + k2_size;

    return size_config;
}

/*
 * Initialise the ring
 */
STATIC CpaStatus initRing(icp_accel_dev_t *accel_dev,
                          icp_et_ring_bank_data_t *bank_data,
                          icp_et_ring_data_t* ring_data,
                          Cpa32U* csr_base_addr,
                          Cpa32U ring_num,
                          Cpa32U bank_num,
                          Cpa32U accel_num,
                          Cpa32U ring_size_msg,
                          Cpa32U msg_size,
                          Cpa32S flags,
                          Cpa32U node_id)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U ringConfig = 0;
    Cpa8U nearly_full_wm = ICP_RING_NEAR_WATERMARK_512;
    Cpa8U nearly_empty_wm = ICP_RING_NEAR_WATERMARK_0;
    Cpa32U ring_size_org = 0;
    Cpa32U modulo = 0;
    Cpa32U ring_size = validateRingSize(ring_size_msg, &modulo);
    Cpa32U numBanksPerAccel = 0;
    Cpa32U numRingPerBank = 0;
    Cpa32U max_space, watermark_nf;

    ICP_CHECK_FOR_NULL_PARAM(ring_data);
    ICP_MEMSET(ring_data, 0, sizeof(icp_et_ring_data_t));
    ring_data->ringNumber = ring_num;
    ring_data->bankNumber = bank_num;
    ring_data->accelNumber = accel_num;
    ring_data->ringmaskId = RING_NUMBER_TO_ID(ring_num);

    numBanksPerAccel = GET_NUM_BANKS_PER_ACCEL(accel_dev);
    numRingPerBank = GET_NUM_RINGS_PER_BANK(accel_dev);

    /* Setup the CSR addresses and pointers */
    ring_data->ringCSRAddress = csr_base_addr;

    /*
     * Setup the shadow variables
     */
    ring_data->head = 0;
    ring_data->tail = 0;
    ring_data->msgSizeInBytes = msg_size;
    ring_data->sizeInBytes = ICP_ET_SIZE_TO_BYTES(ring_size);
    ring_data->flags = flags;
    ring_data->modulo = modulo;
    /* map the rx and tx ring to be the same index number */
    ring_data->inFlightIndex = ((numRingPerBank >> 1) *
                       ring_data->bankNumber) + (ring_data->ringNumber >> 1);
    ring_data->maxRequestsInflight =
       (ring_data->sizeInBytes / (ring_data->msgSizeInBytes)) - 1;

    /*
     * To make sure that ring is alligned to ring size allocate
     * at least 4k and then tell the user it is smaller.
     */
    if(ring_data->sizeInBytes < MIN_RING_SIZE_IN_BYTES)
    {
        ring_size_org = ring_data->sizeInBytes;
        ring_data->sizeInBytes = MIN_RING_SIZE_IN_BYTES;
    }
   /*
     * Setup the ring memory
     */
#ifndef ICP_NUMA_MEM
    if(IS_VF_BUNDLE(ring_data->bankNumber))
    {
        ring_data->ringBaseAddress =
                   ICP_MALLOC_GEN_NUMA(ring_data->sizeInBytes, node_id);
        ring_data->dma_addr = (NULL != ring_data->ringBaseAddress) ?
                   (Cpa64U)ICP_VIRT_TO_PHYS(ring_data->ringBaseAddress) : 0;

        if(ring_data->dma_addr)
        {
            if(osalIOMMUMap(ring_data->dma_addr, ring_data->dma_addr,
                                             ring_data->sizeInBytes))
            {
                ICP_FREE_NUMA(ring_data->ringBaseAddress);
                ADF_ERROR("Failed to map ring address into iommu domain\n");
                return CPA_STATUS_FAIL;
            }
        }
    }
    else
    {
        ring_data->ringBaseAddress = ICP_MALLOC_COHERENT(&GET_DEV(accel_dev),
                                     ring_data->sizeInBytes,
                                     (ICP_DMA_ADDR*)&ring_data->dma_addr);
    }
#else
    ring_data->ringBaseAddress = ICP_MALLOC_GEN_NUMA(ring_data->sizeInBytes,
                                                     node_id);
    ring_data->dma_addr = (NULL != ring_data->ringBaseAddress) ?
                   (Cpa64U)ICP_VIRT_TO_PHYS(ring_data->ringBaseAddress) : 0;
#endif
    if (NULL == ring_data->ringBaseAddress)
    {
        return CPA_STATUS_RESOURCE;
    }
    /* In case we lie it was 2k and we allocated the min (4k)
     * lied we need to set back to 2k */
    if(ring_size_org)
    {
        ring_data->sizeInBytes = ring_size_org;
    }

    ICP_MEMSET(ring_data->ringBaseAddress, ADF_RING_PATTERN,
                        ring_data->sizeInBytes);
    /*
     * Check if the ringBaseAddress is aligned to the size of the buffer
     * sizeInBytes is used as mask and must be round to a power of 2
     * this is guaranteed by validateRingSize()
     */
    status = adf_checkRingAlignment(ring_data->dma_addr,
                                                   ring_data->sizeInBytes);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Ring Base Address is not aligned\n");
#ifndef ICP_NUMA_MEM
        if(IS_VF_BUNDLE(ring_data->bankNumber))
        {
            osalIOMMUUnmap(ring_data->dma_addr, ring_data->sizeInBytes);
            ICP_FREE_NUMA(ring_data->ringBaseAddress);
        }
        else
        {
            ICP_FREE_COHERENT(&GET_DEV(accel_dev), ring_data->sizeInBytes,
                          ring_data->ringBaseAddress, ring_data->dma_addr);
        }
#else
        ICP_FREE_NUMA(ring_data->ringBaseAddress);
#endif
        return CPA_STATUS_FAIL;
    }

    /* value to write to RING BASE register to reset heads and tails for
     * Wireless or dyn userspace rings */
    ring_data->ringBase = BUILD_RING_BASE_ADDR(ring_data->dma_addr,
                                               ring_size);

    /*
     * Configure the Ring CONFIG CSR
     */
    if (flags & (1 << ICP_RESP_TYPE_NONE))
    {
        if (flags & RING_CONFIG_LATE_HEAD_POINTER_MODE)
            ringConfig = BUILD_RING_CONFIG(ring_size |
                                           RING_CONFIG_LATE_HEAD_POINTER_MODE);
        else
            ringConfig = BUILD_RING_CONFIG(ring_size);
    }
    else
    {
        /*
         * Activate coalescing based on number of messages
         * only if the timed coalescing is enabled and the ring size is 16K
         */
        if(bank_data->timedCoalescEnabled &&
                 ring_size == ICP_RINGSIZE_KILO_16)
        {
            char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
            char section[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
            char bank[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
            Cpa32U *irq_src_mask = NULL;
            Cpa32U irq_src_shift = 0;
            Cpa8U  irq_src_val = INT_SRC_EMPTY_FALSE;

            Cpa32U nearly_empty_config = 0;

            snprintf(section, ADF_CFG_MAX_VAL_LEN_IN_BYTES, ACCEL_STR,
                                                                accel_num);
            snprintf(bank, ADF_CFG_MAX_VAL_LEN_IN_BYTES, ADF_NEARLY_FULL,
                                  bank_num % numBanksPerAccel);

            status = icp_adf_cfgGetParamValue(accel_dev,
                                      section, bank, val);
            if (CPA_STATUS_SUCCESS == status)
            {
                nearly_empty_config =
                  (Cpa32U)ICP_STRTOUL(val, NULL, ADF_CFG_BASE_DEC);
                if(nearly_empty_config)
                {
                    /*
                     * Get the nearly empty/full and irq src values
                     */
                    calculate_nf_ne_val(nearly_empty_config,
                                    &nearly_empty_wm,
                                    &nearly_full_wm,
                                    &irq_src_val);
                    /*
                     * Now have to write the calculated irq_src_val to
                     * appropriateregister. There are two registers per
                     * bank (4 bits per each ring). Forst get the address
                     * of the register we want to update and calculate
                     * a shift to the 4 bits we want to update. Then
                     * clean the bits, update it with the value and update
                     * the whole IAIntSrcSel register.
                     */
                    irq_src_mask = (ring_num > IAINTSRCSEL_REG_MAX_RING_IN_REG)
                                              ? &bank_data->irq_src_val_hi :
                                                &bank_data->irq_src_val_low;

                    irq_src_shift = ring_num % IAINTSRCSEL_REG_RINGS_PER_REG;
                    *irq_src_mask &= ~(IAINTSRCSEL_REG_RING_VAL_MASK
                             << (irq_src_shift * IAINTSRCSEL_REG_BITSPERRING));
                    *irq_src_mask |= irq_src_val << (irq_src_shift *
                                                  IAINTSRCSEL_REG_BITSPERRING);
                    /*
                     * Make sure that the IRQ Mode bit is set
                     */
                      bank_data->irq_src_val_low |=
                                           (1 << IAINTSRCSEL_REG_IRQ_MODE_BIT);
                    /*
                     * Write irq source val for the ring
                     */
                    UPDATE_CSR_INT_SRCSEL_RING(bank_num,
                                           ring_num, *irq_src_mask);
                    bank_data->numberMsgCoalescEnabled = 1;
                }
            }
        }
        ringConfig = BUILD_RESP_RING_CONFIG(ring_size, nearly_full_wm,
                                                       nearly_empty_wm);
    }

    WRITE_CSR_RING_CONFIG(bank_num, ring_num, ringConfig);
    ring_data->ringConfig = ringConfig;

    /* calculate the min response per head write */
    watermark_nf = (ringConfig >> RING_CONFIG_NEAR_FULL_WM) &
               ((1<<RING_CONFIG_NEAR_FULL_WM)-1);
    if (watermark_nf == 0)
    {
        max_space = ring_data->sizeInBytes;
    }
    else
    {
       /* according to EAS for Nealy Full Watermark */
       /* calcuate the maximum space we can use before Nearly Full */
        max_space = ring_data->sizeInBytes -
                     sizeof(Cpa32U)*(1<<(watermark_nf-1));
       /* Make sure available space is bigger than half of ring */
        max_space = (max_space > ring_data->sizeInBytes>>1)?
                       max_space:
                       (ring_data->sizeInBytes>>1);
    }
     /* Calculate the number for coal write
      * In genernal, it will be the minimum between half message number
      * and MIN_RESPONSES_PER_HEAD_WRITE
      */
    ring_data->minRespsPerHeadWrite = ((max_space / msg_size) >> 1 >
                    MIN_RESPONSES_PER_HEAD_WRITE)?
                    MIN_RESPONSES_PER_HEAD_WRITE:
                    (max_space/msg_size) >> 1;

    /*
     * Configure the Ring BASE CSR.
     * Note: head and tail will be cleared to zero
     */
    WRITE_CSR_RING_BASE(bank_num, ring_num, ring_data->ringBase);
    ICP_SPINLOCK_INIT(&ring_data->ringLock);
    return CPA_STATUS_SUCCESS;
}

/*
 * Cleanup and free the ring memory
 */
STATIC void cleanupRing(icp_accel_dev_t *accel_dev,
                        icp_et_ring_data_t *ring_data)
{
    if (NULL != ring_data->ringBaseAddress)
    {
        ICP_MEMSET(ring_data->ringBaseAddress, ADF_RING_PATTERN,
                   ring_data->sizeInBytes);
#ifndef ICP_NUMA_MEM
        if(IS_VF_BUNDLE(ring_data->bankNumber))
        {
            osalIOMMUUnmap(ring_data->dma_addr, ring_data->sizeInBytes);
            ICP_FREE_NUMA(ring_data->ringBaseAddress);
        }
        else
        {
            ICP_FREE_COHERENT(&GET_DEV(accel_dev), ring_data->sizeInBytes,
                          ring_data->ringBaseAddress, ring_data->dma_addr);
        }
#else
        ICP_FREE_NUMA(ring_data->ringBaseAddress);
#endif
    }
    ICP_FREE(ring_data->serviceName);
    ICP_MEMSET(ring_data, 0, sizeof(icp_et_ring_data_t));
}

/*
 * Create the ETR ringData, and assoc Transport Handle
 * Preconfigure with shadow copies of tail/head, explicit values
 * for addresses and CSRs
 */
STATIC CpaStatus createETHandle(icp_accel_dev_t *accel_dev,
                                const char *section,
                                Cpa32U accel_num, Cpa32U bank_num,
                                icp_trans_handle** trans_handle,
                                icp_adf_ringInfoService_t info,
                                const char* service_name,
                                const Cpa32U size,
                                Cpa32U flags)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_etr_priv_data_t *priv_data = NULL;
    icp_et_ring_data_t *ring_data = NULL;
    icp_et_ring_bank_data_t *bank_data = NULL;
    icp_trans_handle *tmp_handle = NULL;
    Cpa32U ring_mask_id = 0;
    Cpa32U ring_num_in_dev = 0;
    Cpa32U ring_num_in_bank = 0;
    Cpa32U bank_num_in_dev = 0;
    Cpa32U* csr_base_addr = NULL;
    char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U numRingsPerBank   = 0;
    Cpa32U numBanksPerAccel  = 0;
    Cpa32U numAccelerators   = 0;
    adf_hw_device_data_t  *hw_data = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pCommsHandle);
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(GET_HW_DATA(accel_dev));

    priv_data = (icp_etr_priv_data_t *)accel_dev->pCommsHandle;
    hw_data = accel_dev->pHwDeviceData;
    numRingsPerBank   = GET_NUM_RINGS_PER_BANK(accel_dev);
    numBanksPerAccel  = GET_NUM_BANKS_PER_ACCEL(accel_dev);
    numAccelerators = hw_data->getNumAccelerators(hw_data,accel_dev->accelMask);
    if (accel_num >= numAccelerators)
    {
        ADF_ERROR("Invalid accelerator number (%d)\n", accel_num);
        return CPA_STATUS_FAIL;
    }
    ICP_CHECK_PARAM_RANGE(bank_num, 0, numBanksPerAccel);
    status = icp_adf_cfgGetParamValue(accel_dev,
                                      section, service_name, val);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("No such entry in the configuration file: %s\n",
                                                          service_name);
        return CPA_STATUS_FAIL;
    }
    ring_num_in_bank = (Cpa32U)ICP_STRTOUL(val, NULL, ADF_CFG_BASE_DEC);
    ICP_CHECK_PARAM_RANGE(ring_num_in_bank, 0, numRingsPerBank);
    bank_num_in_dev = (accel_num * numBanksPerAccel) + bank_num;
    ring_num_in_dev = (bank_num_in_dev * numRingsPerBank) +
                                ring_num_in_bank;
    bank_data = &priv_data->banks[bank_num_in_dev];
    csr_base_addr = (Cpa32U*)(UARCH_INT)priv_data->csrBaseAddress;

    /* need to resrve a ring on the appropriate bank */
    ring_mask_id = adf_reserveFreeRing(bank_data, ring_num_in_bank);
    if (ring_mask_id == 0)
    {
        int ctr = 0;
        do {
            /* Need to account for orphan thread */
            ICP_MSLEEP(RING_CREATE_SLEEP_TIME);
            ring_mask_id = adf_reserveFreeRing(bank_data, ring_num_in_bank);
        } while(ring_mask_id == 0 && ctr++ < RING_CREATE_RETRY_TIMES);
        if (ring_mask_id == 0)
        {
            ADF_ERROR("Ring id %d for service %s already exists.\n",
                                       ring_num_in_bank, service_name);
            *trans_handle = NULL;
            return CPA_STATUS_FAIL;
        }
    }
    ring_data = &bank_data->rings[ring_num_in_bank];
    status = initRing(accel_dev, bank_data, ring_data,
                      csr_base_addr,
                      ring_num_in_bank,
                      bank_num_in_dev,
                      accel_num,
                      size,
                      ICP_ET_DEFAULT_MSG_SIZE,
                      flags,
                      accel_dev->pkg_id);

    if (CPA_STATUS_SUCCESS != status)
    {
        unreserveRing(bank_data, ring_mask_id);
        return status;
    }
    status = adf_etrApSetRingRegister(accel_dev, ring_num_in_dev, flags);
    if (CPA_STATUS_SUCCESS != status)
    {
        cleanupRing(accel_dev, ring_data);
        unreserveRing(bank_data, ring_mask_id);
        return status;
    }
    /* Add ringData here to accelhandle*/
    if (flags & (1 << ICP_RESP_TYPE_IRQ))
    {
        Cpa32U irq_mask = 0;
        ICP_SPINLOCK_LOCK_BH(&ring_data->ringLock);
        /* Add to interrupt mask */
        bank_data->interruptMask |= ring_data->ringmaskId;
        irq_mask = bank_data->interruptMask;
        irq_mask |= bank_data->interruptUserMask;
        irq_mask &= ~(bank_data->interruptPendingUserMask);
        ICP_SPINLOCK_UNLOCK_BH(&ring_data->ringLock);

        /* Enable appropriate interrupts */
        if (bank_data->timedCoalescEnabled)
        {
            WRITE_CSR_INT_COL_EN(bank_num_in_dev, irq_mask);
            WRITE_CSR_INT_COL_CTL(bank_num_in_dev,
                                  bank_data->coalIntrDelayCount);
        }
        if(!bank_data->timedCoalescEnabled ||
                bank_data->numberMsgCoalescEnabled)
        {
            WRITE_CSR_INT_REG(bank_num_in_dev, irq_mask);
            WRITE_CSR_INT_EN(bank_num_in_dev, irq_mask);
        }
    }
    else if (flags & (1 << ICP_RESP_TYPE_POLL))
    {
        /* Add to Polling mask */
        bank_data->pollingMask |= ring_data->ringmaskId;
        /* Initialize the Ring tasklet for Polling. */
        /* This is done on a per ring basis. */
        status = adf_setup_poll_tasklets(accel_dev,
                        ring_num_in_bank, bank_num_in_dev);
        if(SUCCESS != status)
        {
            ADF_ERROR("Failed to setup tasklets for Ring Polling.\n");
            cleanupRing(accel_dev, ring_data);
            unreserveRing(bank_data, ring_mask_id);
            return status;
        }
    }

    ring_data->serviceName = ICP_MALLOC_GEN(strlen(service_name) + 1);
    if (NULL == ring_data->serviceName)
    {
        ADF_ERROR("Unable to allocate memory for service name\n");
        cleanupRing(accel_dev, ring_data);
        unreserveRing(bank_data, ring_mask_id);
        return CPA_STATUS_FAIL;
    }
    memcpy(ring_data->serviceName, service_name, strlen(service_name) + 1);
    ring_data->userSpaceRing = 0;

    /* create the ring handle */
    tmp_handle = ICP_MALLOC_GEN(sizeof(icp_trans_handle));
    if (NULL == tmp_handle)
    {
        ADF_ERROR("Unable to allocate memory for tmp_handle\n");
        cleanupRing(accel_dev, ring_data);
        unreserveRing(bank_data, ring_mask_id);
        return CPA_STATUS_FAIL;
    }
    tmp_handle->accel_dev = accel_dev;
    tmp_handle->put_msgs = putETMsgs;
    tmp_handle->reg_callback = regETCallback;

    if (flags & (1 << ICP_RESP_TYPE_IRQ))
    {
        tmp_handle->notify = notifyETMsg;
        tmp_handle->polling_notify = NULL;
    }
    else if (flags & (1 << ICP_RESP_TYPE_POLL))
    {
        tmp_handle->notify = NULL;
        tmp_handle->polling_notify = notifyETMsgForPolling;
    }
    tmp_handle->trans_data = ring_data;
    tmp_handle->handle_id = ring_num_in_dev;
    ring_data->info = info;
    ring_data->parent = tmp_handle;

    /* hand the handle back */
    *trans_handle = tmp_handle;
    if(adf_debug_create_ring(accel_dev, bank_data,
        ring_data) != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Failed creating ring debug proc entry\n");
    }
    return CPA_STATUS_SUCCESS;
}

/* Returns true if the ring is a polled ring */
CpaBoolean adf_is_ring_polled(icp_et_ring_data_t *ring_data)
{
    icp_accel_dev_t *accel_dev = NULL;
    icp_etr_priv_data_t *priv_data = NULL;
    icp_et_ring_bank_data_t *bank_data = NULL;

    if(NULL == ring_data ||
       NULL == ring_data->parent ||
       NULL == ring_data->parent->accel_dev)
    {
        ADF_ERROR("Null parameter passed");
        return CPA_FALSE;
    }

    if (!ring_data->userSpaceRing)
    {
         return CPA_FALSE;
    }

    accel_dev = (icp_accel_dev_t*)ring_data->parent->accel_dev;

    if(NULL == accel_dev->pCommsHandle)
    {
        ADF_ERROR("pCommsHandle is Null");
        return CPA_FALSE;
    }

    priv_data = (icp_etr_priv_data_t *)accel_dev->pCommsHandle;
    bank_data = &priv_data->banks[ring_data->bankNumber];
    if (bank_data->interruptUserMask & ring_data->ringmaskId)
        return CPA_FALSE;

    return CPA_TRUE;
}

/*
 * Returns ring number for given trans handle
 */
STATIC CpaStatus getETRingNumber(icp_trans_handle* trans_handle,
                                 Cpa32U *ringNum)
{
    icp_et_ring_data_t *ringData = NULL;
    icp_accel_dev_t *accel_dev = NULL;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(trans_handle->trans_data);

    accel_dev = (icp_accel_dev_t*)trans_handle->accel_dev;
    ringData = (icp_et_ring_data_t*) trans_handle->trans_data;
    *ringNum = (ringData->bankNumber * GET_NUM_RINGS_PER_BANK(accel_dev))
                                            + ringData->ringNumber;
    return CPA_STATUS_SUCCESS;
}

/*
 * Remove the transport handle and assoc ringData
 */
STATIC CpaStatus releaseETHandle(icp_trans_handle* trans_handle)
{
    CpaStatus status=CPA_STATUS_SUCCESS;
    icp_et_ring_data_t *ringData = NULL;
    icp_etr_priv_data_t *pRingPrivData = NULL;
    icp_et_ring_bank_data_t* bankData = NULL;
    icp_accel_dev_t *accel_dev = NULL;

    Cpa32U bankNumber = 0;
    Cpa32U *csr_base_addr = NULL;
    Cpa32U current_mask = 0;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    accel_dev = (icp_accel_dev_t*)trans_handle->accel_dev;

    /* Setup the local variable pointers now that params have been validated */
    ringData = (icp_et_ring_data_t *) trans_handle->trans_data;

    ICP_CHECK_FOR_NULL_PARAM(ringData);
    csr_base_addr = ringData->ringCSRAddress;

    pRingPrivData = ((icp_accel_dev_t*)
                             trans_handle->accel_dev)->pCommsHandle;
    if(NULL == pRingPrivData)
    {
        ADF_ERROR("Transport data cleaned before ring release is called\n");
        return CPA_STATUS_INVALID_PARAM;
    }
    bankNumber = ringData->bankNumber;
    bankData = &pRingPrivData->banks[bankNumber];

    /*Free resources consumed by tasklets for polling. */
    if(bankData->pollingMask & RING_NUMBER_TO_ID(ringData->ringNumber))
    {
      /* Remove the tasklet used for Ring Polling.*/
        status = adf_cleanup_poll_tasklets(trans_handle->accel_dev,
                                           ringData->ringNumber, bankNumber);
        if(SUCCESS != status)
        {
            ADF_ERROR("Failed to clean-up tasklet for Ring Polling.\n");
        }
        bankData->pollingMask &= ~ringData->ringmaskId;
    }
    /* clear the ring from the interrupt mask */
    ICP_SPINLOCK_LOCK(&bankData->bankLock);
    bankData->interruptUserMask &= ~ringData->ringmaskId;
    bankData->interruptMask &= ~ringData->ringmaskId;
    bankData->interruptPendingUserMask &= ~ringData->ringmaskId;

    current_mask = bankData->interruptMask;
    current_mask |= bankData->interruptUserMask;
    current_mask |= bankData->interruptPendingUserMask;
    ICP_SPINLOCK_UNLOCK(&bankData->bankLock);

    /* Disable appropriate interrupts */
    if (bankData->timedCoalescEnabled)
    {
        WRITE_CSR_INT_COL_EN(bankNumber, current_mask);
        WRITE_CSR_INT_COL_CTL(bankNumber, bankData->coalIntrDelayCount);
    }

    if(!bankData->timedCoalescEnabled ||
         bankData->numberMsgCoalescEnabled)
    {
        WRITE_CSR_INT_REG(bankNumber, current_mask);
        WRITE_CSR_INT_EN(bankNumber, current_mask);
    }

    /* Clear PCI Config space */
    WRITE_CSR_RING_CONFIG(bankNumber, ringData->ringNumber, 0);
    WRITE_CSR_RING_BASE(bankNumber, ringData->ringNumber, 0);

    status = adf_etrApClrRingRegister(
                        (icp_accel_dev_t*)trans_handle->accel_dev,
                        trans_handle->handle_id,
                        ringData->flags);

    adf_debug_remove_ring(ringData);
    /* remove from internal table */
    unreserveRing(bankData, ringData->ringmaskId);

    /* cleanup the ring resources */
    cleanupRing(accel_dev, ringData);

    /* free ring handle */
    ICP_FREE(trans_handle);
    trans_handle = NULL;
    return CPA_STATUS_SUCCESS;
}

/*
 * Map rings needing servicing to execution of notify() funcs on ring handles
 */
STATIC void adf_ring_notify_handler(icp_et_ring_bank_data_t *bank,
                                    Cpa32U numRingsInBank,
                                    Cpa32U* csr_base_addr)
{
    Cpa32U csrVal = 0;
    Cpa32U ringNum = 0;
    icp_et_ring_data_t* ring = NULL;

    /* Read the ring status CSR to determine which rings have data */
    /* If one of our response rings has data to be processed */
    /* Read CSR to see which rings need servicing */
    csrVal = READ_CSR_E_STAT(bank->bankNumber);
    csrVal = ~csrVal;
    /* Need to handle both user & kernel space */
    csrVal &= (bank->interruptMask | bank->interruptUserMask);

    /* Service the rings */
    for(ringNum = 0; ringNum < numRingsInBank; ++ringNum)
    {
        if (csrVal & RING_NUMBER_TO_ID(ringNum))
        {
            ring = &bank->rings[ringNum];
            ring->parent->notify(ring->parent);
        }
    }
}

/*
 * Response handler from ISR
 */
void adf_ringResponseHandler(void* handle, Cpa32U bankNumber)
{
    icp_accel_dev_t *accel_dev = NULL;
    icp_etr_priv_data_t *privData = NULL;
    Cpa32U *csr_base_addr = NULL;
    Cpa32U currentIrqMask = 0, currentIrqMaskWithoutPending = 0;

    accel_dev = handle;
    privData = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    currentIrqMask |= privData->banks[bankNumber].interruptMask;
    currentIrqMask |= privData->banks[bankNumber].interruptUserMask;

    /*
     * Write in advance to IAIntReg register to clean IRQs
     */
    if (!privData->banks[bankNumber].timedCoalescEnabled ||
        privData->banks[bankNumber].numberMsgCoalescEnabled)
    {
            WRITE_CSR_INT_REG(bankNumber, currentIrqMask);
    }

    /*
     * Now handle all the responses
     */
    adf_ring_notify_handler(&privData->banks[bankNumber],
            GET_NUM_RINGS_PER_BANK(accel_dev), csr_base_addr);

    /* Done processing - need to reenable interrupts again.
     * Filter out userspace pending interrputs.
     * They will be handled and reenabled by userspace process.
     * Here we just have to enable interrupts for kernelspace rings.
     */
    currentIrqMaskWithoutPending = currentIrqMask &
                 ~(privData->banks[bankNumber].interruptPendingUserMask);

    if (!privData->banks[bankNumber].timedCoalescEnabled ||
        privData->banks[bankNumber].numberMsgCoalescEnabled)
    {
            WRITE_CSR_INT_EN(bankNumber, currentIrqMaskWithoutPending);
    }
    if (privData->banks[bankNumber].timedCoalescEnabled)
    {
        /* If there are are some IRQ to userspace pending
         * then reenable just the kernelsace IRQs.
         * If not then updating the coalesc counter is
         * enough to reenable IRQ for the bank
         */
        if(currentIrqMask != currentIrqMaskWithoutPending)
        {
            WRITE_CSR_INT_COL_EN(bankNumber, currentIrqMaskWithoutPending);
        }
        WRITE_CSR_INT_COL_CTL(bankNumber,
             privData->banks[bankNumber].coalIntrDelayCount);
    }
    privData->banks[bankNumber].interruptPendingUserMask = 0;
}

/*
 * Response handler for the Ring Polling on a per Bank basis.
 * The bank CSR is read and the rings that have data on them
 * are identified. Only those rings are then polled. The number
 * of messages polled is determined by the bank response quota
 * which is given by the user application.
 */
void adf_bankResponsePolling(icp_accel_dev_t *accelHandle,
                             Cpa32U bankNumber)
{
    icp_accel_dev_t *accel_dev = NULL;
    icp_etr_priv_data_t *privData = NULL;
    icp_et_ring_bank_data_t *bank = NULL;
    icp_et_ring_data_t* ring = NULL;
    Cpa32U csrVal = 0;
    Cpa32U ring_number_in_bank=0;
    Cpa32U numRingsPerBank = 0;

    accel_dev = accelHandle;
    privData = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;

    bank = &privData->banks[bankNumber];

    numRingsPerBank = GET_NUM_RINGS_PER_BANK(accel_dev);

    /* The csrVal is stored in the ringsFull variable
     * It contains which rings have data and are polling
     * rings on this bank. */
    csrVal = bank->ringsFullMask;

    /* Service the bank. Loop until either all the rings have been
     * serviced or the response quota of messages have been processed.
     */
    do
    {
        if (csrVal & RING_NUMBER_TO_ID(ring_number_in_bank))
        {
            ring = &bank->rings[ring_number_in_bank];
            ring->parent->polling_notify(ring->parent,
                                      bank->bankResponseQuota);
        }
        ring_number_in_bank++;

    } while ((ring_number_in_bank < numRingsPerBank)) ;
    /*Reset the flag to one indicating tasklet's work is done. */
    osalAtomicSet(1, &(bank->pollingTaskletScheduled));
}

/*
 * Response handler for the Ring Polling on a per ring basis.
 * The calling function has already checked if there is data
 * on this ring. The number of messages polled is determined
 * by the ring response quota which is supplied by the user
 * application.
 */
void adf_ringResponsePolling(void* ringHandle)
{
    icp_et_ring_data_t* ring = NULL;

    ring = (icp_et_ring_data_t*)ringHandle;
    ring->parent->polling_notify(ring->parent,
                              ring->ringResponseQuota);
    /*Reset the flag to one indicating tasklet's work is done. */
    osalAtomicSet(1, &ring->pollingTaskletScheduled);
}

/*
 * Gets value from configuration file
 */
STATIC CpaStatus adf_etrGetParamValue(icp_accel_dev_t *accel_dev,
                                      char *section,
                                      Cpa8S * key_format,
                                      Cpa32U key_id,
                                      Cpa32U *value)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8S key[ADF_CFG_MAX_KEY_LEN_IN_BYTES] = {0};
    Cpa8S val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    snprintf((char*)key, ADF_CFG_MAX_KEY_LEN_IN_BYTES,
             (const char*)key_format, key_id);

    status = icp_adf_cfgGetParamValue(accel_dev,
                                      section,
                                      (char*)key,
                                      (char*)val);
    if (CPA_STATUS_SUCCESS != status)
    {
        return CPA_STATUS_FAIL;
    }
    *value = ICP_STRTOUL((char*)val, NULL, ADF_CFG_BASE_DEC);

    return CPA_STATUS_SUCCESS;
}

/*
 * Initializes bank structure
 */
STATIC CpaStatus initBank(icp_accel_dev_t *accel_dev,
                          icp_et_ring_bank_data_t* bankData,
                          Cpa32U bankNum,
                          Cpa32U* csr_base_addr)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0, accelerator = 0, bank_num_in_accel = 0;
    char section[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
    Cpa32U timedCoalescEnabled = 0;
    Cpa32U maxBanks    = 0;
    Cpa32U numRingsPerBank  = 0;
    Cpa32U numBanksPerAccel = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(bankData);
    ICP_CHECK_FOR_NULL_PARAM(csr_base_addr);

    maxBanks    = GET_MAX_BANKS(accel_dev);
    numRingsPerBank  = GET_NUM_RINGS_PER_BANK(accel_dev);
    numBanksPerAccel = GET_NUM_BANKS_PER_ACCEL(accel_dev);

    if (maxBanks < bankNum + 1)
    {
        ADF_ERROR("Invalid bank number\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    ICP_MEMSET(bankData, 0, sizeof(icp_et_ring_bank_data_t));

    bankData->bankNumber = bankNum;

    ICP_SPINLOCK_INIT(&bankData->bankLock);

    accelerator = bankNum / numBanksPerAccel;
    bank_num_in_accel = bankNum % numBanksPerAccel;

    snprintf(section, ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
                    "Accelerator%d", accelerator);

    status = adf_etrGetParamValue(accel_dev, section,
                        (Cpa8S*)ADF_ETRMGR_COALESCING_ENABLED_FORMAT,
                        bank_num_in_accel, &timedCoalescEnabled);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_DEBUG(ADF_ETRMGR_COALESCING_ENABLED_FORMAT
                  " in accelerator %d is not set.\n",
                  bank_num_in_accel, accelerator);
    }
    if (timedCoalescEnabled > 1)
    {
        ADF_PRINT(ADF_ETRMGR_COALESCING_ENABLED_FORMAT
                  " in accelerator %d is set to an invalid value(%d)."
                  " Should be 0 or 1. Defaulting to 1.\n",
                  bank_num_in_accel, accelerator, timedCoalescEnabled);
        timedCoalescEnabled = 1;
    }
    if (timedCoalescEnabled)
    {
        bankData->timedCoalescEnabled = 1;
        status = adf_etrGetParamValue(accel_dev, section,
                        (Cpa8S*)ADF_ETRMGR_COALESCE_TIMER_FORMAT,
                        bank_num_in_accel, &bankData->coalIntrDelayCount);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR(ADF_ETRMGR_COALESCE_TIMER_FORMAT " not found.\n",
                            bank_num_in_accel);
            bankData->coalIntrDelayCount = COALESCING_TIME_INTERVAL;
            ADF_PRINT("Using default value %d\n", COALESCING_TIME_INTERVAL);
        }
        if(MAX_COALESCING_TIME_INTERVAL < bankData->coalIntrDelayCount ||
           MIN_COALESCING_TIME_INTERVAL > bankData->coalIntrDelayCount)
        {
            bankData->coalIntrDelayCount = COALESCING_TIME_INTERVAL;
            ADF_PRINT("Bank coalesing value invalid. Using default value %d\n",
                       COALESCING_TIME_INTERVAL);
        }
    }

    /*
     * Clean CRS for all rings within the bank
     */
    for(i = 0; i < numRingsPerBank; i++)
    {
        WRITE_CSR_RING_CONFIG(bankNum, i, 0);
        WRITE_CSR_RING_BASE(bankNum, i, 0);
    }
    /*
     * For now, all rings within the bank are setup such that the generation
     * of flag interrupts will be triggered when ring leaves the empty state.
     * Note that in order for the ring interrupt to generate an IRQ the
     * interrupt must also be enabled for the ring.
     */

    WRITE_CSR_INT_SRCSEL(bankNum);
    bankData->irq_src_val_low = BANK_INT_SRC_SEL_MASK_0;
    bankData->irq_src_val_hi = BANK_INT_SRC_SEL_MASK_X;
    /*Initialise the bank polling mask. */
    bankData->pollingMask = 0;

    status = adf_debug_transport_create_dir(accel_dev, bankData);
    return status;
}

/*
 * Setup private data structures for the ETR device
 */
CpaStatus adf_init_ETR_Data(icp_accel_dev_t *accelHandle)
{
    icp_accel_dev_t *accel_dev = NULL;
    icp_etr_priv_data_t* privData = NULL;
    icp_et_ring_bank_data_t *bankData = NULL;
    icp_et_ring_ap_bank_data_t *apBankData = NULL;
    Cpa32U bank_num = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U bankDataSize = 0;
    Cpa32U bankApDataSize = 0;
    Cpa32U maxBanks = 0;
    Cpa32U maxRingInBank = 0;
    Cpa32U maxApBanks = 0;
    Cpa32U etrBarId = 0;
    Cpa32U arrId=0;
    adf_hw_device_data_t  *hw_data = NULL;
    Cpa32U ringInflightSize = 0;
    OsalAtomic *ringInflightsData = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accelHandle);
    accel_dev = accelHandle;

    /* Allocate priv_data                                    */
    privData = ICP_MALLOC_GEN_NUMA(sizeof(icp_etr_priv_data_t),
                                   accel_dev->pkg_id);
    if (NULL == privData)
    {
        ADF_ERROR("Unable to allocate memory for privData\n");
        return CPA_STATUS_FAIL;
    }
    ICP_MEMSET(privData, '\0', sizeof(icp_etr_priv_data_t));

    /* Allocate bank_data                                    */
    maxBanks = GET_MAX_BANKS(accel_dev);
    bankDataSize = maxBanks * sizeof (icp_et_ring_bank_data_t);
    bankData = ICP_MALLOC_GEN_NUMA(bankDataSize,
                                   accel_dev->pkg_id);
    if (NULL == bankData)
    {
        ICP_FREE_NUMA(privData);
        ADF_ERROR("Unable to allocate memory for bankData\n");
        return CPA_STATUS_FAIL;
    }
    ICP_MEMSET(bankData, '\0', bankDataSize);

    /* Allocate ap bank data                                 */
    maxApBanks = GET_MAX_AP_BANKS(accel_dev);
    bankApDataSize = maxApBanks * sizeof (icp_et_ring_ap_bank_data_t);
    apBankData = ICP_MALLOC_GEN_NUMA(bankApDataSize,
            accel_dev->pkg_id);
    if (NULL == apBankData)
    {
        ICP_FREE_NUMA(bankData);
        ICP_FREE_NUMA(privData);
        ADF_ERROR("Unable to allocate memory for bankData\n");
        return CPA_STATUS_FAIL;
    }
    ICP_MEMSET(apBankData, '\0', bankApDataSize);

    hw_data = accel_dev->pHwDeviceData;

    etrBarId = hw_data->getEtrBarId(hw_data, NULL);
    privData->banks          = bankData;
    privData->apBanks        = apBankData;
    privData->csrBaseAddress = (ARCH_INT)
        (accel_dev->pciAccelDev.pciBars[etrBarId].virtAddr);

    accel_dev->pCommsHandle  = privData;

    status = adf_init_ETRings_proc_debug(accel_dev);
    if(CPA_STATUS_SUCCESS != status)
    {
        ICP_FREE_NUMA(privData->apBanks);
        ICP_FREE_NUMA(privData->banks);
        ICP_FREE_NUMA(privData);
        ADF_ERROR("Unable to allocate proc debug entry\n");
        return status;
    }

    for(bank_num = 0; bank_num < maxBanks; bank_num++)
    {
        status = initBank(accelHandle, &privData->banks[bank_num], bank_num,
                           (Cpa32U*)(UARCH_INT)privData->csrBaseAddress);
        if(CPA_STATUS_SUCCESS != status)
        {
            adf_clean_ETRings_proc_debug(accel_dev);
            ICP_FREE_NUMA(privData->apBanks);
            ICP_FREE_NUMA(privData->banks);
            ICP_FREE_NUMA(privData);
            return status;
        }
    }

    /* allocate an ring inflight array */
    maxRingInBank = GET_NUM_RINGS_PER_BANK(accel_dev);
    ringInflightSize = (sizeof(OsalAtomic) * (maxRingInBank >> 1)) * maxBanks;
    ringInflightsData = ICP_MALLOC_GEN_NUMA(ringInflightSize,
                                            accel_dev->pkg_id);
    if (NULL == ringInflightsData)
    {
        ADF_ERROR("Unable to allocate memory for ring Inflight array\n");
        adf_clean_ETRings_proc_debug(accel_dev);
        ICP_FREE_NUMA(privData->apBanks);
        ICP_FREE_NUMA(privData->banks);
        ICP_FREE_NUMA(privData);
        return CPA_STATUS_FAIL;
    }
    for (arrId=0; arrId<ringInflightSize/sizeof(OsalAtomic); arrId++)
    {
        osalAtomicSet(0, &ringInflightsData[arrId]);
    }
     /* set the inflight index to accel dev structure */
    accel_dev->pRingInflight = (void*)ringInflightsData;
    adf_etrApClrBanks(accel_dev, CPA_TRUE);
    return status;
}

/*
 * Cleanup ring bundles
 */
STATIC CpaStatus cleanupBank(icp_accel_dev_t *accel_dev,
                             icp_et_ring_bank_data_t* bankData,
                             Cpa32U numRingsInBank)
{
    icp_et_ring_data_t* ringData = NULL;
    Cpa32U ringNum = 0;

    ICP_CHECK_FOR_NULL_PARAM(bankData);

    for (ringNum = 0; ringNum < numRingsInBank; ++ringNum)
    {
        ringData = &bankData->rings[ringNum];
        if (bankData->allocatedRings & ringData->ringmaskId)
        {
            if(NULL != ringData->parent)
            {
                ICP_FREE(ringData->parent);
                ringData->parent = NULL;
            }
            cleanupRing(accel_dev, ringData);
        }
    }
    ICP_MEMSET(bankData, 0, sizeof(icp_et_ring_bank_data_t));
    return CPA_STATUS_SUCCESS;
}

/*
 * Cleanup ET Handles
 */
STATIC CpaStatus cleanupETHandles(icp_accel_dev_t *accel_dev)
{
    Cpa32U i = 0;
    Cpa32U maxBanks = 0;
    icp_etr_priv_data_t* ringPrivData = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ringPrivData = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;

    maxBanks = GET_MAX_BANKS(accel_dev);
    for (i = 0; i < maxBanks; i++)
    {
        adf_debug_transport_remove_dir(accel_dev, &ringPrivData->banks[i]);
        cleanupBank(accel_dev, &ringPrivData->banks[i],
                    GET_NUM_RINGS_PER_BANK(accel_dev));
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Cleanup private data structures for the ETR device
 */
CpaStatus adf_cleanup_ETR_Data(icp_accel_dev_t *accelHandle)
{
    icp_accel_dev_t *accel_dev = NULL;
    icp_etr_priv_data_t *privData = NULL;
    OsalAtomic *pRingInflight;
    void *ptr;

    ICP_CHECK_FOR_NULL_PARAM(accelHandle);
    accel_dev = accelHandle;

    privData = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;
    if (privData)
    {
        adf_etrApClrBanks(accel_dev, CPA_FALSE);
        /* release the handles */
        cleanupETHandles(accel_dev);
        adf_clean_ETRings_proc_debug(accel_dev);
        /* cleanup any coupled data structures */
        ICP_FREE_NUMA(privData->apBanks);
        ICP_FREE_NUMA(privData->banks);
        ICP_FREE_NUMA(privData);
        accel_dev->pCommsHandle = NULL;
    }
    pRingInflight = accel_dev->pRingInflight;
    if (pRingInflight)
    {
        ptr = (void*)pRingInflight;
        ICP_FREE_NUMA(ptr);
        accel_dev->pRingInflight = NULL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Initialise the ET Ring Manager
 */
CpaStatus adf_init_ETManager()
{
    /* Create the Ring Manager */
    ETRingMgr.trans_type = ICP_TRANS_TYPE_ETR;
    ETRingMgr.create_handle = createETHandle;
    ETRingMgr.find_handle = adf_findETHandle;
    ETRingMgr.release_handle = releaseETHandle;
    ETRingMgr.get_ring_num = getETRingNumber;
    /* Register with the Ring Factory */
    return adf_trans_registerTransMgr(&ETRingMgr);
}

/*
 * Shutdown the ET Ring Manager
 */
CpaStatus adf_shutdown_ETManager()
{
    /* Unregister from the Ring Factory */
    return adf_trans_deregisterTransMgr(&ETRingMgr);
}
