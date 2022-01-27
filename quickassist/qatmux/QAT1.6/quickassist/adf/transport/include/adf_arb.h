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
 * @file adf_arb.h
 *
 * @description
 *      This header file contains the prototypes and definitions required
 *      for arbitration configuration.
 *
 *****************************************************************************/

#ifndef ADF_ARB_H
#define ADF_ARB_H

/* Number of service arbiters */
#define ICP_ARB_NUM              4
/* Number of request rings */
#define ICP_ARB_REQ_RING_NUM     8
/* Size of a regular register */
#define ICP_ARB_REG_SIZE         0x4
/* Size of a weight register */
#define ICP_ARB_WTR_SIZE         0x20

/* Putlen value for 16 byte responses */
#define ICP_ARB_PUTLEN_16B_RESP      0x0
/* Putlen value for 32 byte responses */
#define ICP_ARB_PUTLEN_32B_RESP     0x1
/* Putlen value for 64 byte responses */
#define ICP_ARB_PUTLEN_64B_RESP     0x2

/* Threshold value for 16B resp */
#define ICP_ARB_THOLD_16B_RESP  0x3
/* Threshold value for 32B resp */
#define ICP_ARB_THOLD_32B_RESP  0x4
/* Threshold value for 64B resp */
#define ICP_ARB_THOLD_64B_RESP  0x5

/* Number of worker threads per AE */
#define ICP_ARB_WRK_THREAD_TO_SARB        12

/*
 * Enumeration on arbiter numbers
 */
typedef enum icp_service_arbiter_number_e
{
    ICP_SARB_ZERO = 0,
    ICP_SARB_ONE,
    ICP_SARB_TWO,
    ICP_SARB_THREE

} icp_service_arbiter_number;

/*
 * Enumeration on arbiter weights
 */
typedef enum icp_service_arbiter_w_e
{
    ICP_SARB_WEIGHT_DEFAULT = 1,
    ICP_SARB_WEIGHT_HI

} icp_service_arbiter_w;

#define ICP_ARB_PKE_WEIGHT        ICP_SARB_WEIGHT_DEFAULT
#define ICP_ARB_CY_WEIGHT         ICP_SARB_WEIGHT_DEFAULT
#define ICP_ARB_TRNG_WEIGHT       ICP_SARB_WEIGHT_DEFAULT
#define ICP_ARB_DC_WEIGHT         ICP_SARB_WEIGHT_DEFAULT

/*
* @description
* Static initialisation of the arbitration unit hardware.  This will
* assign all rings in the accelerator to a specific service arbitrator.
* The assignment is statically provisioned.
* Note: This does not enable rings for arbitration, but configures all
* service arbiters.
*/
CpaStatus adf_arbInit(icp_accel_dev_t *pAccelDev);

/*
* @description
* Enable a specific ring for arbitration. Only valid for request rings.
* Note: Assumption is arbitration initialisation has completed.
*/
CpaStatus adf_arbEnableRing(icp_accel_dev_t *accel_dev,
        icp_et_ring_data_t *pRing);

/*
* @description
* Disable a specific ring for arbitration.
* Note: Assumption is arbitration initialisation has completed. Only valid for
* request rings
*/
CpaStatus adf_arbDisableRing(icp_accel_dev_t *accel_dev,
        icp_et_ring_data_t *pRing);

/*
* @description
* Verify that the Arbiter config for the response ring msg size (putLen)
* matches the msg size passed to createHandle.
* Note: Assumption is arbitration initialisation has completed.
*/
CpaStatus adf_arbVerifyResponseMsgSize(icp_accel_dev_t *accel_dev,
        icp_et_ring_data_t *pRing);

/*
* @description
* Disable the arbitration of all rings associated with the acceleration
* device and disable the arbitration hardware.
* This is called at device shutdown.
*/
CpaStatus adf_arbShutdown(icp_accel_dev_t *pAccelDev);


#endif /* ADF_ARB_H */
