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
 * @file adf_ETring_mgr_dp.c
 *
 * @description
 *      ET Ring Manager for data plain
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_user_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_platform.h"
#include "adf_dev_ring_ctl.h"

/*
 * icp_adf_getQueueMemory
 * Data plain support function - returns the pointer to next message on the ring
 * or NULL if there is not enough space.
 */
INLINE void icp_adf_getQueueMemory(icp_comms_trans_handle trans_hnd,
                                   Cpa32U numberRequests,
                                   void** pCurrentQatMsg)
{
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_hnd;
    Cpa32U** targetAddr = (Cpa32U**) pCurrentQatMsg;
    Cpa64U  flight;
    OsalAtomic *ring_in_flight;

    /* Check if there is enough space in the ring */
    ring_in_flight = pRingHandle->trad_in_flight;
    flight = osalAtomicAdd(numberRequests, ring_in_flight);
    if (flight > pRingHandle->max_requests_inflight - 1)
    {
        osalAtomicSub(numberRequests, ring_in_flight);
        *targetAddr = NULL;
        return;
    }

    /* We have enough space - get the address of next message */
    *targetAddr = (Cpa32U*)
        (((UARCH_INT)pRingHandle->ring_virt_addr)
         + pRingHandle->tail);
}

/*
 * icp_adf_getSingleQueueAddr
 * Data plane support function - returns the pointer to next message on the ring
 * or NULL if there is not enough space - it also updates the shadow tail copy.
 */
INLINE void icp_adf_getSingleQueueAddr(icp_comms_trans_handle trans_hnd,
        void** pCurrentQatMsg)
{
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_hnd;
    Cpa32U** targetAddr = (Cpa32U**) pCurrentQatMsg;
    Cpa64U flight;
    OsalAtomic *ring_in_flight;

    /* Check if there is enough space in the ring */
    ring_in_flight = pRingHandle->trad_in_flight;
    flight = osalAtomicInc(ring_in_flight);
    if (flight > pRingHandle->max_requests_inflight - 1)
    {
        osalAtomicDec(ring_in_flight);
        *targetAddr = NULL;
        return;
    }

    /* We have enough space - get the address of next message */
    *targetAddr = (Cpa32U*)
        (((UARCH_INT)pRingHandle->ring_virt_addr)
         + pRingHandle->tail);

    /* Update the shadow tail */
    pRingHandle->tail = modulo((pRingHandle->tail + ICP_ET_DEFAULT_MSG_SIZE),
            pRingHandle->modulo);
}

/*
 * icp_adf_getQueueNext
 * Data plain support function - increments the tail pointer and returns
 * the pointer to next message on the ring.
 */
INLINE void icp_adf_getQueueNext(icp_comms_trans_handle trans_hnd,
                                 void** pCurrentQatMsg)
{
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_hnd;
    Cpa32U** targetAddr = (Cpa32U**) pCurrentQatMsg;

    /* Increment tail to next message */
    pRingHandle->tail = modulo((pRingHandle->tail + ICP_ET_DEFAULT_MSG_SIZE),
                                pRingHandle->modulo);

    /* Get the address of next message */
    *targetAddr = (Cpa32U*)
                     (((UARCH_INT)pRingHandle->ring_virt_addr)
                     + pRingHandle->tail);
}


/*
 * icp_adf_updateQueueTail
 * Data plain support function - Writes the tail shadow copy to the device.
 */
INLINE void icp_adf_updateQueueTail(icp_comms_trans_handle trans_hnd)
{
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_hnd;
    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*) pRingHandle->accel_dev;
    Cpa32U *csr_base_addr = ((Cpa32U*)accel_dev->virtConfigSpace);

    WRITE_CSR_RING_TAIL(pRingHandle->bank_num,
                            pRingHandle->ring_num,
                            pRingHandle->tail);
}

/*
 * icp_adf_isRingEmpty
 * Data plain support function -  check if the ring is empty
 */
INLINE CpaBoolean icp_adf_isRingEmpty(icp_comms_trans_handle trans_hnd)
{
    Cpa32U mask = 0;
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_hnd;
    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*) pRingHandle->accel_dev;
    Cpa32U *csr_base_addr = ((Cpa32U*)accel_dev->virtConfigSpace);

    mask = READ_CSR_E_STAT(pRingHandle->bank_num);
    mask = ~mask;

    if(mask & (1 << (pRingHandle->ring_num)))
    {
        return CPA_FALSE;
    }
    return CPA_TRUE;
}


/*
 *  * icp_adf_pollQueue
 *   * Data plain support function - Poll messages from the queue.
 *    */
INLINE CpaStatus icp_adf_pollQueue(icp_comms_trans_handle trans_hnd,
        Cpa32U response_quota)
{
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_hnd;
    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*) pRingHandle->accel_dev;
    Cpa32U *csr_base_addr = ((Cpa32U*)accel_dev->virtConfigSpace);
    Cpa32U msg_counter = 0;
    volatile Cpa32U *msg = NULL;

    if(response_quota == 0)
    {
        response_quota = ICP_NO_RESPONSE_QUOTA;
    }

    /* point to where the next message should be */
    msg = (Cpa32U*)
        (((UARCH_INT)pRingHandle->ring_virt_addr)
         + pRingHandle->head);

    /* If there are valid messages then process them */
    while((*msg != EMPTY_RING_ENTRY_SIG) && (msg_counter < response_quota))
    {
        adf_user_process_msg(pRingHandle, &msg, &msg_counter, csr_base_addr);
    }

    /* Coalesce head writes to reduce impact of MMIO write */
    if((msg_counter > 0) && (msg_counter >= pRingHandle->coal_write_count))
    {
        pRingHandle->coal_write_count =
                pRingHandle->min_resps_per_head_write;
        WRITE_CSR_RING_HEAD(pRingHandle->bank_num,
                pRingHandle->ring_num,
                pRingHandle->head);
    }
    else
    {
        /* Not enough responses have been processed to warrant the cost
         * of a head write. Updating the count for the next time. */
        pRingHandle->coal_write_count -= msg_counter;
    }

    if(msg_counter == 0)
    {
        return CPA_STATUS_RETRY;
    }
    return CPA_STATUS_SUCCESS;
}


/*
 * icp_adf_queueDataToSend
 * LAC lite support function - Indicates if there is data on the ring to be
 * send. This should only be called on request rings. If the function returns
 * true then it is ok to call icp_adf_updateQueueTail() function on this ring.
 */
INLINE CpaBoolean icp_adf_queueDataToSend(icp_comms_trans_handle trans_hnd)
{
    adf_dev_ring_handle_t *ringData = (adf_dev_ring_handle_t *)trans_hnd;
    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*) ringData->accel_dev;
    Cpa32U *csr_base_addr = ((Cpa32U*)accel_dev->virtConfigSpace);
    Cpa32U tail = READ_CSR_RING_TAIL(ringData->bank_num,
                                     ringData->ring_num);
    return (tail == ringData->tail) ? CPA_FALSE : CPA_TRUE;
}

