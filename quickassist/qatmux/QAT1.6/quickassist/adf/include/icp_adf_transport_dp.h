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
 * @file icp_adf_transport_dp.h
 *
 * @description
 *      File contains Public API definitions for ADF transport for data plane.
 *
 *****************************************************************************/
#ifndef ICP_ADF_TRANSPORT_DP_H
#define ICP_ADF_TRANSPORT_DP_H

#include "cpa.h"
#include "icp_adf_transport.h"
#include "icp_platform.h"

/*
 * icp_adf_getQueueMemory
 * Data plain support function - returns the pointer to next message on the ring
 * or NULL if there is not enough space.
 */
INLINE void icp_adf_getQueueMemory(icp_comms_trans_handle trans_handle,
                                   Cpa32U numberRequests,
                                   void** pCurrentQatMsg);
/*
 * icp_adf_getSingleQueueAddr
 * Data plain support function - returns the pointer to next message on the ring
 * or NULL if there is not enough space - it also updates the shadow tail copy.
 */
INLINE void icp_adf_getSingleQueueAddr(icp_comms_trans_handle trans_handle,
                                               void** pCurrentQatMsg);

/*
 * icp_adf_getQueueNext
 * Data plain support function - increments the tail pointer and returns
 * the pointer to next message on the ring.
 */
INLINE void icp_adf_getQueueNext(icp_comms_trans_handle trans_handle,
                                         void** pCurrentQatMsg);

/*
 * icp_adf_updateQueueTail
 * Data plain support function - Writes the tail shadow copy to the device.
 */
INLINE void icp_adf_updateQueueTail(icp_comms_trans_handle trans_handle);

/*
 * icp_adf_isRingEmpty
 * Data plain support function - check if the ring is empty
 */
INLINE CpaBoolean icp_adf_isRingEmpty(icp_comms_trans_handle trans_handle);

/*
 * icp_adf_pollQueue
 * Data plain support function - Poll messages from the queue.
 */
INLINE CpaStatus icp_adf_pollQueue(icp_comms_trans_handle trans_handle,
                                                 Cpa32U response_quota);

/*
 * icp_adf_queueDataToSend
 * LAC lite support function - Indicates if there is data on the ring to be
 * send. This should only be called on request rings. If the function returns
 * true then it is ok to call icp_adf_updateQueueTail() function on this ring.
 */
INLINE CpaBoolean icp_adf_queueDataToSend(icp_comms_trans_handle trans_hnd);

#endif /* ICP_ADF_TRANSPORT_DP_H */
