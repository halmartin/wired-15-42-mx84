/******************************************************************************
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

/******************************************************************************
 * @file adf_user_transport.h
 *
 * @description
 * User space transport functions
 *****************************************************************************/
#ifndef ADF_USER_TRANSPORT_H
#define ADF_USER_TRANSPORT_H

#include "adf_dev_ring_ctl.h"
#include "adf_transport_ctrl.h"
#include "adf_platform.h"

/*
 * adf_user_put_msgs
 *
 * Description
 * Function puts the message onto the ring
 */
CpaStatus adf_user_put_msgs(adf_dev_ring_handle_t *pRingHandle,
                            Cpa32U *inBuf,
                            Cpa32U bufLen);
/*
 * adf_user_notify_msgs
 *
 * Description
 * Function notifies client that there is response message
 */
CpaStatus adf_user_notify_msgs(adf_dev_ring_handle_t *pRingHandle);

/*
 * adf_user_notify_msgs_poll
 *
 * Description
 * Function notifies client that there is response message on polling rings
 */
CpaStatus adf_user_notify_msgs_poll(adf_dev_ring_handle_t *pRingHandle);

/*
 * adf_user_unmap_rings
 *
 * Description
 * Function unmaps all rings allocated for a given device
 */
CpaStatus adf_user_unmap_rings(icp_accel_dev_t *accel_dev);

/*
 * adf_pollRing
 *
 * Description
 * Internal functions which polls
 * a polling ring. This function does not check
 * to see if the ring is a polling ring or
 * if the ring exists.
 *
 */
CpaStatus adf_pollRing(icp_accel_dev_t *accel_dev,
                       adf_dev_ring_handle_t *pRingHandle,
                       Cpa32U response_quota);

/*
 * adf_user_transport_init
 *
 * Description
 * Function initializes internal transport data
 */
CpaStatus adf_user_transport_init(icp_accel_dev_t *accel_dev);

/*
 * adf_user_transport_exit
 *
 * Description
 * Function deinitializes internal transport data
 */
CpaStatus adf_user_transport_exit(icp_accel_dev_t *accel_dev);

/*
 * adf_user_process_msg
 *
 * Description
 * Process the Message, trigger the associated callback
 * update the CSR (if needed) and the inflight counter.
 */
static inline void adf_user_process_msg(adf_dev_ring_handle_t *pRingHandle,
        volatile Cpa32U **msg, Cpa32U *msg_counter, Cpa32U* csr_base_addr)
{

        osalAtomicDec(pRingHandle->trad_in_flight);

        /* Invoke the callback for the message */
        pRingHandle->callback((icp_comms_trans_handle *)pRingHandle,
                (Cpa32U*)*msg);

        /* Mark the message as processed */
        **msg = EMPTY_RING_ENTRY_SIG;

        /* Advance the head offset and handle wraparound */
        pRingHandle->head = modulo((pRingHandle->head +
                    ICP_ET_DEFAULT_MSG_SIZE), pRingHandle->modulo);
       (*msg_counter)++;

        /* Point to where the next message should be */
        *msg = (Cpa32U*)
            (((UARCH_INT)pRingHandle->ring_virt_addr)
             + pRingHandle->head);

}

#endif /* ADF_USER_TRANSPORT_H */
