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

/**
 *****************************************************************************
 * @file lac_sym_qat.h
 *
 * @defgroup LacSymQat  Symmetric QAT
 *
 * @ingroup LacSym
 *
 * Interfaces for populating the qat structures for a symmetric operation
 *
 * @lld_start
 *
 * @lld_overview
 * This file documents the interfaces for populating the qat structures
 * that are common for all symmetric operations.
 *
 * @lld_dependencies
 * - \ref LacSymQatHash "Hash QAT Comms" Sym Qat commons for Hash
 * - \ref LacSymQat_Cipher "Cipher QAT Comms" Sym Qat commons for Cipher
 * - OSAL: logging
 * - \ref LacMem "Memory" - Inline memory functions
 *
 * @lld_initialisation
 * This component is initialied during the LAC initialisation sequence. It
 * is called by the Symmetric Initialisation function.
 *
 * @lld_module_algorithms
 *
 * @lld_process_context
 * Refer to \ref LacHash "Hash" and \ref LacCipher "Cipher" for sequence
 * diagrams to see their interactions with this code.
 *
 *
 * @lld_end
 *
 *****************************************************************************/


/*****************************************************************************/

#ifndef LAC_SYM_QAT_H
#define LAC_SYM_QAT_H


/*
******************************************************************************
* Include public/global header files
******************************************************************************
*/

#include "cpa.h"
#include "cpa_cy_sym.h"
#include "icp_accel_devices.h"
#include "icp_qat_fw_la.h"
#include "icp_qat_hw.h"
#include "sal_qat_cmn_msg.h"
#include "lac_common.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/

/* The ARC4 key will not be stored in the content descriptor so we only need to
 * reserve enough space for the next biggest cipher setup block.
 * Kasumi needs to store 2 keys and to have the size of 2 blocks for fw*/
#define LAC_SYM_QAT_MAX_CIPHER_SETUP_BLK_SZ \
        (sizeof(icp_qat_hw_cipher_config_t) + 2*ICP_QAT_HW_KASUMI_KEY_SZ + \
        2*ICP_QAT_HW_KASUMI_BLK_SZ)
/**< @ingroup LacSymQat
 * Maximum size for the cipher setup block of the content descriptor */

#define LAC_SYM_QAT_MAX_HASH_SETUP_BLK_SZ   sizeof(icp_qat_hw_auth_algo_blk_t)
/**< @ingroup LacSymQat
 * Maximum size for the hash setup block of the content descriptor */

#define LAC_SYM_QAT_HASH_CONTENT_DESC_SIZE \
        (sizeof(icp_qat_fw_auth_hdr_t) + LAC_SYM_QAT_MAX_HASH_SETUP_BLK_SZ)
/**< @ingroup LacSymQat
 * Maximum size for a hash-only content descriptor */

#define LAC_SYM_QAT_CIPHER_CONTENT_DESC_SIZE \
        (sizeof(icp_qat_fw_cipher_hdr_t) + LAC_SYM_QAT_MAX_CIPHER_SETUP_BLK_SZ)
/**< @ingroup LacSymQat
 * Maximum size for a cipher-only content descriptor */

#define LAC_SYM_QAT_CONTENT_DESC_MAX_SIZE   LAC_ALIGN_POW2_ROUNDUP( \
        LAC_SYM_QAT_HASH_CONTENT_DESC_SIZE +                        \
        LAC_SYM_QAT_CIPHER_CONTENT_DESC_SIZE,                       \
        (1 << LAC_64BYTE_ALIGNMENT_SHIFT))
/**< @ingroup LacSymQat
 *  Maximum size of content descriptor. This is incremented to the next multiple
 * of 64 so that it can be 64 byte aligned */


/**
 *******************************************************************************
 * @ingroup LacSymQat
 *      Initialise the Symmetric QAT code
 *
 * @description
 *      This function initialises the symmetric QAT code
 *
 * @param[in] device                Pointer to the acceleration device
 *                                  structure
 * @param[in] instanceHandle        Instance handle
 * @param[in] numSymRequests        Number of concurrent requests a pair
 *                                  (tx and rx) need to support
 *
 * @return CPA_STATUS_SUCCESS       Operation successful
 * @return CPA_STATUS_FAIL           Initialisation Failed
 *
 *****************************************************************************/
CpaStatus
LacSymQat_Init(icp_accel_dev_t *device, CpaInstanceHandle instanceHandle,
                                                  Cpa32U numSymRequests);


/**
 *******************************************************************************
 * @ingroup LacSymQat
 *      Register a response handler function for a symmetric command ID
 *
 * @description
 *      This function registers a response handler function for a symmetric
 *      operation.
 *
 *      Note: This operation should only be performed once by the init function
 *      of a component. There is no corresponding deregister function, but
 *      registering a NULL function pointer will have the same effect. There
 *      MUST not be any requests in flight when calling this function.
 *
 * @param[in] lacCmdId          Command Id of operation
 * @param[in] pCbHandler        callback handler function
 *
 * @return None
 *
 *****************************************************************************/
void
LacSymQat_RespHandlerRegister(
    icp_qat_fw_la_cmd_id_t lacCmdId,
    sal_qat_resp_handler_func_t pCbHandler);


/**
 ******************************************************************************
 * @ingroup LacSymQat
 *      get the QAT packet type
 *
 * @description
 *      This function returns the QAT packet type for a LAC packet type. The
 *      LAC packet type does not indicate a first partial. therefore for a
 *      partial request, the previous packet type needs to be looked at to
 *      figure out if the current partial request is a first partial.
 *
 *
 * @param[in] packetType          LAC Packet type
 * @param[in] packetState         LAC Previous Packet state
 * @param[out] pQatPacketType     Packet type using the QAT macros
 *
 * @return none
 *
 *****************************************************************************/
void
LacSymQat_packetTypeGet(
    CpaCySymPacketType packetType,
    CpaCySymPacketType packetState,
    Cpa32U *pQatPacketType);

/**
 ******************************************************************************
 * @ingroup LacSymQat
 *      Populate the command flags based on the packet type
 *
 * @description
 *      This function populates the packet type and update state LA command Flag
 *      based on looking at the packet type.
 *
 * @param[in] qatPacketType          Packet type
 * @param[in] cmdId                  Command Id
 * @param[out] pLaCommandFlags       Command Flags
 *
 * @return none
 *
 *****************************************************************************/
void
LacSymQat_LaPacketCommandFlagSet(
    Cpa32U qatPacketType,
    icp_qat_fw_la_cmd_id_t laCmdId,
    Cpa16U *pLaCommandFlags);

#endif /* LAC_SYM_QAT_H */
