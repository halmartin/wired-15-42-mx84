/******************************************************************************
 *
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
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

/*
 * adf_user_put_msg
 *
 * Description
 * Function puts the message onto the ring
 */
CpaStatus adf_user_put_msg(adf_dev_ring_handle_t *pRingHandle,
                           Cpa32U *inBuf,
                           uint64_t *seq_num);
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

#endif /* ADF_USER_TRANSPORT_H */
