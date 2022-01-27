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
 * User space implementation of ETring transport functions
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_platform.h"
#include "adf_platform.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_user_transport.h"
#include "icp_adf_cfg.h"
#include "adf_cfg.h"
#include "adf_dev_ring_ctl.h"

/*
 * Put messages onto the ET Ring
 */
CpaStatus adf_user_put_msgs(adf_dev_ring_handle_t *pRingHandle,
                             Cpa32U *inBuf,
                             Cpa32U bufLen)
{
    icp_accel_dev_t *accel_dev;
    Cpa32U* targetAddr;
    Cpa8U* csr_base_addr;
    OsalAtomic *ring_in_flight;
    INT64 flight;

    ICP_CHECK_FOR_NULL_PARAM(pRingHandle);
    ICP_CHECK_FOR_NULL_PARAM(pRingHandle->accel_dev);

    accel_dev = (icp_accel_dev_t*) pRingHandle->accel_dev;
    csr_base_addr =  ((Cpa8U*)accel_dev->virtConfigSpace);

    ICP_MUTEX_LOCK(pRingHandle->user_lock);

    /* Check if there is enough space in the ring */
    ring_in_flight = pRingHandle->trad_in_flight;
    flight = osalAtomicInc(ring_in_flight);
    if (flight > pRingHandle->max_requests_inflight)
    {
        osalAtomicDec(ring_in_flight);
        ICP_MUTEX_UNLOCK(pRingHandle->user_lock);
        return CPA_STATUS_RETRY;
    }

    /*
     * We have enough space - copy the message to the ring
     */
    targetAddr = (Cpa32U*)
                     (((UARCH_INT) pRingHandle->ring_virt_addr)
                     + pRingHandle->tail);

    icp_adf_memcpy(targetAddr, inBuf);

    /* Update shadow copy values */
    pRingHandle->tail = modulo((pRingHandle->tail + ICP_ET_DEFAULT_MSG_SIZE),
                                 pRingHandle->modulo);

    /* and the config space of the device */
    WRITE_CSR_RING_TAIL(pRingHandle->bank_num,
                        pRingHandle->ring_num,
                        pRingHandle->tail);
    ICP_MUTEX_UNLOCK(pRingHandle->user_lock);
    return CPA_STATUS_SUCCESS;
}


/*
 * Notifies the transport handle in question.
 */
CpaStatus adf_user_notify_msgs(adf_dev_ring_handle_t *pRingHandle)
{
    volatile Cpa32U *msg;
    Cpa32U msg_counter = 0;
    Cpa32U *csr_base_addr;
    icp_accel_dev_t *accel_dev;

    ICP_CHECK_FOR_NULL_PARAM(pRingHandle);
    ICP_CHECK_FOR_NULL_PARAM(pRingHandle->accel_dev);

    accel_dev = (icp_accel_dev_t*) pRingHandle->accel_dev;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->virtConfigSpace);
    csr_base_addr = ((Cpa32U*)accel_dev->virtConfigSpace);

    msg = (Cpa32U*)
        (((UARCH_INT)pRingHandle->ring_virt_addr)
         + pRingHandle->head);

    /* If there are valid messages then process them */
    while(*msg != EMPTY_RING_ENTRY_SIG)
    {
        adf_user_process_msg(pRingHandle, &msg, &msg_counter, csr_base_addr);

    }

    /* Need to write the head for irq driven always
     * otherwise the IRQ will be constantly triggered */
    WRITE_CSR_RING_HEAD(pRingHandle->bank_num,
            pRingHandle->ring_num,
            pRingHandle->head);

    /* enable interrupts */
    if(pRingHandle->bank_data->timed_coalesc_enabled)
    {
        WRITE_CSR_INT_COL_EN(pRingHandle->bank_data->bank_number,
                          pRingHandle->bank_data->interrupt_mask);
    }
    if(!pRingHandle->bank_data->timed_coalesc_enabled ||
           pRingHandle->bank_data->number_msg_coalesc_enabled)
    {
        WRITE_CSR_INT_EN(pRingHandle->bank_data->bank_number,
                           pRingHandle->bank_data->interrupt_mask);
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Notify function used for polling. Messages are read until the ring is
 * empty or the response quota has been fulfilled.
 * If the response quota is zero, messages are read until the ring is drained.
 */
CpaStatus adf_user_notify_msgs_poll(adf_dev_ring_handle_t *pRingHandle)
{
    icp_accel_dev_t *accel_dev=NULL;
    volatile Cpa32U *msg = NULL;
    Cpa32U msg_counter=0, response_quota=ICP_NO_RESPONSE_QUOTA;
    Cpa32U* csr_base_addr = NULL;

    accel_dev = (icp_accel_dev_t*) pRingHandle->accel_dev;
    csr_base_addr = ((Cpa32U*)accel_dev->virtConfigSpace);

    response_quota = pRingHandle->ringResponseQuota;
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

    return CPA_STATUS_SUCCESS;
}

