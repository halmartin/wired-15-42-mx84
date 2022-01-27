/*
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
 */

/**
 *****************************************************************************
 * @file qat_rings.h
 * @defgroup qat_ctrl
 * @ingroup qat_ctrl
 *
 * @description
 *      This file contains declarations of functions used to create and update
 *      rings tables and polling masks.
 *
 *****************************************************************************/

#ifndef __QAT_RINGS_H__
#define __QAT_RINGS_H__

/* Number of rings available to a QAT instance. */
#define SAL_RINGS_NUM_PER_QAT   128

/* Word size in bits. */
#define SAL_WORD_SIZE_IN_BITS   32

/* macros to set/unset the bit mask. */
#define SAL_QAT_MASK_128_SET_BIT(mask, bit_num) \
    ((Cpa32U*)(mask))[((bit_num)/SAL_WORD_SIZE_IN_BITS)] \
        |= 1<<((bit_num) % SAL_WORD_SIZE_IN_BITS)

#define SAL_QAT_MASK_128_UNSET_BIT(mask, bit_num) \
    if( (((Cpa32U*)(mask))[((bit_num)/SAL_WORD_SIZE_IN_BITS)]) \
            & (1<<((bit_num) % SAL_WORD_SIZE_IN_BITS))){ \
    ((Cpa32U*)(mask))[((bit_num)/SAL_WORD_SIZE_IN_BITS)] \
            ^= 1<<((bit_num) % SAL_WORD_SIZE_IN_BITS);}


/*
 * convenience macros for updating the weights and the mask
 * on rings that are created by user space processes.
 */

#define ENABLE_CY_A_RING_TABLE(ringNumber,weighting)                    \
        qatInstance->pCyARingTable->bulk_rings[(ringNumber)].curr_weight\
                        = (weighting);                                  \
        qatInstance->pCyARingTable->bulk_rings[(ringNumber)].init_weight\
                        = (weighting);                                  \
        ICP_QAT_FW_INIT_RING_MASK_SET(                                  \
                        (*qatInstance->pCyARingTable), (ringNumber))

#define DISABLE_CY_A_RING_TABLE(ringNumber, weighting)                  \
        ICP_QAT_FW_INIT_RING_MASK_CLEAR(                                \
                        (*qatInstance->pCyARingTable), ringNumber);     \
        qatInstance->pCyARingTable->bulk_rings[ringNumber].curr_weight  \
                        = (weighting);                                  \
        qatInstance->pCyARingTable->bulk_rings[ringNumber].init_weight  \
                        = (weighting);

#define ENABLE_CY_B_RING_TABLE(ringNumber,weighting)                    \
        qatInstance->pCyBRingTable->bulk_rings[(ringNumber)].curr_weight\
                        = (weighting);                                  \
        qatInstance->pCyBRingTable->bulk_rings[(ringNumber)].init_weight\
                        = (weighting);                                  \
        ICP_QAT_FW_INIT_RING_MASK_SET(                                  \
                        (*qatInstance->pCyBRingTable), (ringNumber))

#define DISABLE_CY_B_RING_TABLE(ringNumber, weighting)                  \
        ICP_QAT_FW_INIT_RING_MASK_CLEAR(                                \
                        (*qatInstance->pCyBRingTable), ringNumber);     \
        qatInstance->pCyBRingTable->bulk_rings[ringNumber].curr_weight  \
                        = (weighting);                                  \
        qatInstance->pCyBRingTable->bulk_rings[ringNumber].init_weight  \
                        = (weighting);

#define ENABLE_DC_RING_TABLE(ringNumber,weighting)                      \
        qatInstance->pDcRingTable->bulk_rings[(ringNumber)].curr_weight \
                        = (weighting);                                  \
        qatInstance->pDcRingTable->bulk_rings[(ringNumber)].init_weight \
                        = (weighting);                                  \
           ICP_QAT_FW_INIT_RING_MASK_SET(                               \
                        (*qatInstance->pDcRingTable), (ringNumber))

#define DISABLE_DC_RING_TABLE(ringNumber, weighting)                    \
        ICP_QAT_FW_INIT_RING_MASK_CLEAR(                                \
                        (*qatInstance->pDcRingTable), ringNumber);      \
        qatInstance->pDcRingTable->bulk_rings[ringNumber].curr_weight   \
                        = (weighting);                                  \
        qatInstance->pDcRingTable->bulk_rings[ringNumber].init_weight   \
                        = (weighting);


/**
*******************************************************************************
 * @ingroup qat_ctrl
 *      Build the kernel space ring table
 *
 * @description
 *       Build the kernel ring table. There is one master table which
 *       contains all of the sub tables. The sub tables are the
 *       crypto table and the compression table.
 *       Entry points (indices) are provided so that the
 *       Cy and Dc tables can be easily located. Once the ring
 *       table is build it can be sent to the FW.
 *
 * @param[in] device           Accelaration Device Handle
 * @param[in] qatInstance      QAT instance handle
 * @param[in] ae_index_per_qat AE index to ring table, range 0-num_aes_per_qat
 *
 * @retval CPA_STATUS_SUCCESS   On success
 * @retval CPA_STATUS_FAIL      On general failure
 *
 *
 *****************************************************************************/
CpaStatus
QatCtrl_buildKernelRingTable(icp_accel_dev_t* device,
                             sal_qat_service_t* qatInstance,
                             Cpa32U ae_index_per_qat);

/**
*******************************************************************************
 * @ingroup qat_ctrl
 *      Update the ring table due to user space processes
 *
 * @description
 *       Update the ring table as Rings are enabled or disabled.
 *       This function called when user space processes create
 *       and release ring handles.
 *
 * @param[in] device             Accelaration Device Handle
 * @param[in] qatInstance        QAT instance handle
 * @param[in] ringNumber         The ring number to update
 * @param[in] operation          Enable or disable
 * @param[in] ring_service_type  The type of ring
 *
 * @retval CPA_STATUS_SUCCESS    On success
 * @retval CPA_STATUS_FAIL       If the ring service type is not valid
 *
 *****************************************************************************/
CpaStatus
QatCtrl_updateRingTable(icp_accel_dev_t* device,
                        sal_qat_service_t* qatInstance,
                        Cpa32U ringNumber,
                        icp_adf_ringInfoOperation_t operation,
                        Cpa32U ring_service_type);

#endif
