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
 * @file adf_ETring_mgr_dp.c
 *
 * @description
 *      ET Ring Manager for data plane
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"

/*
 * icp_adf_getQueueMemory
 * LAC lite support function - returns the pointer to next message on the ring
 * or NULL if there is not enough space.
 */
INLINE void icp_adf_getQueueMemory(icp_comms_trans_handle trans_hnd,
                                   Cpa32U numberRequests,
                                   void** pCurrentQatMsg)
{
    icp_trans_handle *trans_handle = (icp_trans_handle*) trans_hnd;
    icp_et_ring_data_t *ringData = (icp_et_ring_data_t *)
                                    trans_handle->trans_data;
    icp_accel_dev_t * accel_dev = (icp_accel_dev_t*)
                                  trans_handle->accel_dev;
    Cpa32U** targetAddr = (Cpa32U**) pCurrentQatMsg;
    Cpa32U req_ring_index;
    OsalAtomic *pRingInflight;
    INT64 flight;

     /* Check if there is enough space in the ring */
    req_ring_index = ringData->inFlightIndex;
    pRingInflight = (OsalAtomic*)accel_dev->pRingInflight + req_ring_index;

    flight = osalAtomicAdd(numberRequests, pRingInflight);
    if (flight > ringData->maxRequestsInflight - 1)
    {
        osalAtomicSub(numberRequests, pRingInflight);
        *targetAddr = NULL;
        return;
    }
    /* We have enough space - get the address of next message */
    *targetAddr = (Cpa32U*)
                     (((UARCH_INT)ringData->ringBaseAddress)
                     + ringData->tail);
}


/*
 * icp_adf_getSingleQueueAddr
 * LAC lite support function - returns the pointer to next message on the ring
 * or NULL if there is not enough space - it also updates the shadow tail copy.
 */
INLINE void icp_adf_getSingleQueueAddr(icp_comms_trans_handle trans_hnd,
                                       void** pCurrentQatMsg)
{
    icp_trans_handle *trans_handle = (icp_trans_handle*)trans_hnd;
    icp_et_ring_data_t *ringData = (icp_et_ring_data_t *)
        trans_handle->trans_data;
    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*)
                      trans_handle->accel_dev;
    Cpa32U** targetAddr = (Cpa32U**) pCurrentQatMsg;
    Cpa32U req_ring_index;
    OsalAtomic *pRingInflight;
    INT64 flight;

     /* Check if there is enough space in the ring */
    req_ring_index = ringData->inFlightIndex;
    pRingInflight = accel_dev->pRingInflight;

     /* Check if there is enough space in the ring */
    flight = osalAtomicInc(&pRingInflight[req_ring_index]);
    if (flight > ringData->maxRequestsInflight - 1)
    {
         osalAtomicDec(&pRingInflight[req_ring_index]);
         *targetAddr = NULL;
         return;
    }
     /* We have enough space - get the address of next message */
    *targetAddr = (Cpa32U*)
                     (((UARCH_INT)ringData->ringBaseAddress)
                     + ringData->tail);

    /* Update the shadow tail */
    ringData->tail = adf_modulo((ringData->tail + ringData->msgSizeInBytes),
                                              ringData->modulo);
}


/*
 * icp_adf_getQueueNext
 * LAC lite support function - increments the tail pointer and returns
 * the pointer to next message on the ring.
 */
INLINE void icp_adf_getQueueNext(icp_comms_trans_handle trans_hnd,
                                 void** pCurrentQatMsg)
{
    icp_trans_handle *trans_handle = (icp_trans_handle*) trans_hnd;
    icp_et_ring_data_t *ringData = (icp_et_ring_data_t *)
                                         trans_handle->trans_data;
    Cpa32U** targetAddr = (Cpa32U**) pCurrentQatMsg;

    /* Increment tail to next message */
    ringData->tail = adf_modulo((ringData->tail + ringData->msgSizeInBytes),
                             ringData->modulo);

    /* Get the address of next message */
    *targetAddr = (Cpa32U*)
                     (((UARCH_INT)ringData->ringBaseAddress)
                     + ringData->tail);
}

/*
 * icp_adf_updateQueueTail
 * LAC lite support function - Writes the tail shadow copy to the device.
 */
INLINE void icp_adf_updateQueueTail(icp_comms_trans_handle trans_hnd)
{
    icp_trans_handle *trans_handle = (icp_trans_handle*) trans_hnd;
    icp_et_ring_data_t *ringData = (icp_et_ring_data_t *)
                                       trans_handle->trans_data;
    Cpa32U* csr_base_addr = ringData->ringCSRAddress;

    WRITE_CSR_RING_TAIL(ringData->bankOffset,
                            ringData->ringNumber,
                            ringData->tail);

    /* Save the CSR tail value to the shadow tail variable (csrTailOffset)
     * to avoid an unnecessary MMIO read of this CSR later in
     * icp_adf_queueDataToSend().  We can assume that nothing else will
     * modify this CSR in the meantime, because this is data-plane code
     * which is required to be used in a single-threaded context
     */ 
    ringData->csrTailOffset = ringData->tail;
}

/*
 * icp_adf_queueDataToSend
 * Data-plane support function - Indicates if there is data on the ring to be
 * sent. This should only be called on request rings. If the function returns
 * true then it is ok to call icp_adf_updateQueueTail() function on this ring.
 */
INLINE CpaBoolean icp_adf_queueDataToSend(icp_comms_trans_handle trans_hnd)
{


    icp_trans_handle *trans_handle = (icp_trans_handle*) trans_hnd;
    icp_et_ring_data_t *ringData = (icp_et_ring_data_t *)
                                       trans_handle->trans_data;
    
    /* if the original CSR tail offset is different from the shadow tail
     * value we know there must be new messages on the ring, so return true
     */
    if(ringData->tail != ringData->csrTailOffset)
        return CPA_TRUE;
    else
        return CPA_FALSE;

}

/*
 * icp_adf_isRingEmpty
 * LAC lite support function -  check if the ring is empty
 */
INLINE CpaBoolean icp_adf_isRingEmpty(icp_comms_trans_handle trans_hnd)
{
    Cpa32U mask = 0;
    icp_trans_handle *trans_handle = (icp_trans_handle*) trans_hnd;
    icp_et_ring_data_t *pRingHandle = (icp_et_ring_data_t *)
                                         trans_handle->trans_data;

    Cpa32U* csr_base_addr = (Cpa32U*) pRingHandle->ringCSRAddress;

    mask = READ_CSR_E_STAT(pRingHandle->bankOffset);
    mask = ~mask;

    if(mask & (1 << (pRingHandle->ringNumber)))
    {
        return CPA_FALSE;
    }
    return CPA_TRUE;
}

/*
 *  * icp_adf_pollQueue
 *   * Data plane support function - Poll messages from the queue.
 *    */
INLINE CpaStatus icp_adf_pollQueue(icp_comms_trans_handle trans_hdl,
        Cpa32U response_quota)
{
    icp_trans_handle *trans_handle = (icp_trans_handle*) trans_hdl;
    icp_et_ring_data_t *ringData = (icp_et_ring_data_t *)
        trans_handle->trans_data;
    Cpa32U msg_counter=0;
    volatile Cpa32U *msg = NULL;
    Cpa32U* csr_base_addr = ringData->ringCSRAddress;

    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*)
        trans_handle->accel_dev;
    OsalAtomic *pRingInflight = (OsalAtomic*)accel_dev->pRingInflight +
        ringData->inFlightIndex;

    if(response_quota == 0)
    {
        response_quota = ICP_NO_RESPONSE_QUOTA;
    }

    /* Point to where the next message should be */
    msg = (Cpa32U*)
        (((UARCH_INT)ringData->ringBaseAddress) + ringData->head);

    /* If there are valid messages then process them */
    while((*msg != EMPTY_RING_ENTRY_SIG) && (msg_counter < response_quota))
    {
        adf_process_msg(trans_handle, &msg, &msg_counter, pRingInflight);
    }

    /* Update the head CSR if any messages were processed */
    if(msg_counter > 0)
    {
        WRITE_CSR_RING_HEAD(ringData->bankOffset,
                     ringData->ringNumber,
                     ringData->head);
    }
    else
    {
        return CPA_STATUS_RETRY;
    }
    return CPA_STATUS_SUCCESS;
}
