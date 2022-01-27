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
 * @file icp_adf_transport.h
 *
 * @description
 *      File contains Public API Definitions for ADF transport.
 *
 *****************************************************************************/
#ifndef ICP_ADF_TRANSPORT_H
#define ICP_ADF_TRANSPORT_H

#include "cpa.h"

/*
 * Enumeration on Transport Types exposed
 */
typedef enum icp_transport_type_e
{
    ICP_TRANS_TYPE_NONE = 0,
    ICP_TRANS_TYPE_ETR,
    ICP_TRANS_TYPE_DP_ETR,
    ICP_TRANS_TYPE_DELIMIT
} icp_transport_type;

/*
 * Enumeration on response delivery method
 */
typedef enum icp_resp_deliv_method_e
{
    ICP_RESP_TYPE_NONE = 0,
    ICP_RESP_TYPE_IRQ,
    ICP_RESP_TYPE_POLL,
    ICP_RESP_TYPE_DELIMIT
} icp_resp_deliv_method;

/*
 * Unique identifier of a transport handle
 */
typedef Cpa32U icp_trans_identifier;

/*
 * Opaque Transport Handle
 */
typedef void *icp_comms_trans_handle;

/*
 * Function Pointer invoked when a set of messages is received for the given
 * transport handle
 */
typedef void (*icp_trans_callback)(icp_comms_trans_handle trans_handle,
                                   void *pMsg);

/*
 * icp_adf_getDynInstance
 *
 * Description:
 * Get an available instance from dynamic instance pool
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 *
 */
CpaStatus icp_adf_getDynInstance(icp_accel_dev_t *accel_dev, 
                                adf_service_type_t stype,
                                Cpa32U *pinstance_id);

/*
 * icp_adf_getDynInstance_byAccel
 *
 * Description:
 * Get an available instance from dynamic instance pool by the accelerator id.
 * For crypto, the @accelID could be 0, 1, 2, 3;
 * For compression, the @accelID should be ACCEL_ANY.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 *
 */
CpaStatus icp_adf_getDynInstance_byAccel(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U *pinstance_id, Cpa32U accelID);

/*
 * icp_adf_putDynInstance
 *
 * Description:
 * Put back an instance to dynamic instance pool
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 *
 */
CpaStatus icp_adf_putDynInstance(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U instance_id);

/*
 * icp_adf_getNumAvailDynInstance_byAccel
 *
 * Description:
 * Get the number of the available dynamic instances by the accelerator ID
 * For crypto, the @accelID could be 0, 1, 2, 3;
 * For compression, the @accelID should be ACCEL_ANY.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 *
 */
CpaStatus icp_adf_getNumAvailDynInstance_byAccel(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype, Cpa32U accelID,
                                Cpa32U *avail_num);

/*
 * icp_adf_getNumAvailDynInstance
 *
 * Description:
 * Get the number of the available dynamic instances
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 *
 */
CpaStatus icp_adf_getNumAvailDynInstance(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U *avail_num);

/*
 * icp_adf_transCreateHandle
 *
 * Description:
 * Create a transport handle
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 *
 *   Note: size is in number of requests.
 *   A requests is 64bytes in size.
 *   Supported sizes:
 *     32, 64, 128, 256, 512, 1024, 2048 number of requests.
 *
 */
CpaStatus icp_adf_transCreateHandle(icp_accel_dev_t *accel_dev,
                                    icp_transport_type trans_type,
                                    const char *section,
                                    const Cpa32U accel_nr,
                                    const Cpa32U bank_nr,
                                    const char *service_name,
                                    const icp_adf_ringInfoService_t info,
                                    icp_trans_callback callback,
                                    icp_resp_deliv_method resp,
                                    const Cpa32U size,
                                    icp_comms_trans_handle* trans_handle);

/*
 * icp_adf_transGetHandle
 *
 * Description:
 * Gets a pointer to a previously created transport handle 
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 *
 */
CpaStatus icp_adf_transGetHandle(icp_accel_dev_t *accel_dev,
                                    icp_transport_type trans_type,
                                    const char *section,
                                    const Cpa32U accel_nr,
                                    const Cpa32U bank_nr,
                                    const char *service_name,
                                    icp_comms_trans_handle* trans_handle);

/*
 * icp_adf_transReleaseHandle
 *
 * Description:
 * Release a transport handle
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_transReleaseHandle(icp_comms_trans_handle trans_handle);

/*
 * icp_adf_transPutMsg
 *
 * Description:
 * Put Message onto the transport handle
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_transPutMsg(icp_comms_trans_handle trans_handle,
                              Cpa32U *inBufs,
                              Cpa32U bufLen);


/*
 * icp_adf_transGetRingNum
 *
 * Description:
 *  Function Returns ring number of the given trans_handle
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_transGetRingNum(icp_comms_trans_handle trans_handle,
                                  Cpa32U *ringNum);

#endif /* ICP_ADF_TRANSPORT_H */
