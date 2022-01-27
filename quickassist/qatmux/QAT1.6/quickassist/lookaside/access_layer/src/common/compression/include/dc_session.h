/****************************************************************************
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
 ***************************************************************************/

/**
 *****************************************************************************
 * @file dc_session.h
 *
 * @ingroup Dc_DataCompression
 *
 * @description
 *      Definition of the Data Compression session parameters.
 *
 *****************************************************************************/
#ifndef DC_SESSION_H
#define DC_SESSION_H

#include "cpa_dc_dp.h"
#include "icp_adf_transport.h"
#include "icp_qat_fw_comp.h"
#include "sal_qat_cmn_msg.h"

/* Maximum number of intermediate buffers SGLs */
#define DC_MAX_NUM_INTERMEDIATE_BUFFERS (12)

/* Maximum size of the state registers 64 bytes */
#define DC_QAT_STATE_REGISTERS_MAX_SIZE (64)

/* Size of the history window.
 * Base 2 logarithm of maximum window size minus 8 */
#define DC_8K_WINDOW_SIZE  (5)
#define DC_32K_WINDOW_SIZE (7)

/* Context size */
#define DC_DEFLATE_CONTEXT_SIZE (49152)
#define DC_INFLATE_CONTEXT_SIZE (36864)

/* Retrieve the session descriptor pointer from the session context structure
 * that the user allocates. The pointer to the internally realigned address
 * is stored at the start of the session context that the user allocates */
#define DC_SESSION_DESC_FROM_CTX_GET(pSession) \
    (dc_session_desc_t *) (*(LAC_ARCH_UINT *)pSession)

/* Maximum size for the compression part of the content descriptor */
#define DC_QAT_COMP_CONTENT_DESC_SIZE \
        sizeof(icp_qat_fw_comp_cd_hdr_t)

/* Maximum size for the translator part of the content descriptor */
#define DC_QAT_TRANS_CONTENT_DESC_SIZE \
        (sizeof(icp_qat_fw_xlt_cd_hdr_t) + DC_QAT_MAX_TRANS_SETUP_BLK_SZ)

/* Maximum size of the decompression content descriptor */
#define DC_QAT_CONTENT_DESC_DECOMP_MAX_SIZE   LAC_ALIGN_POW2_ROUNDUP( \
        DC_QAT_COMP_CONTENT_DESC_SIZE,                                \
        (1 << LAC_64BYTE_ALIGNMENT_SHIFT))

/* Maximum size of the compression content descriptor */
#define DC_QAT_CONTENT_DESC_COMP_MAX_SIZE   LAC_ALIGN_POW2_ROUNDUP( \
        DC_QAT_COMP_CONTENT_DESC_SIZE +                        \
        DC_QAT_TRANS_CONTENT_DESC_SIZE,                        \
        (1 << LAC_64BYTE_ALIGNMENT_SHIFT))

/* Direction of the request */
typedef enum dc_request_dir_e
{
    DC_COMPRESSION_REQUEST = 1,
    DC_DECOMPRESSION_REQUEST
} dc_request_dir_t;

/* Type of the compression request */
typedef enum dc_request_type_e
{
    DC_REQUEST_FIRST = 1,
    DC_REQUEST_SUBSEQUENT
} dc_request_type_t;

/* Session descriptor structure for compression */
typedef struct dc_session_desc_s
{
    Cpa8U stateRegistersComp[DC_QAT_STATE_REGISTERS_MAX_SIZE];
    /**< State registers for compression */
    Cpa8U stateRegistersDecomp[DC_QAT_STATE_REGISTERS_MAX_SIZE];
    /**< State registers for decompression */
    icp_qat_fw_comp_req_t reqCacheComp;
    /**< Cache as much as possible of the compression request in a pre-built
     * request */
    icp_qat_fw_comp_req_t reqCacheDecomp;
    /**< Cache as much as possible of the decompression request in a pre-built
     * request */
    dc_request_type_t requestType;
    /**< Type of the compression request. As stateful mode do not support more
     * than one in-flight request there is no need to use spinlocks */
    dc_request_type_t previousRequestType;
    /**< Type of the previous compression request. Used in cases where there the
     * stateful operation needs to be resubmitted */
    CpaDcHuffType huffType;
    /**< Huffman tree type */
    CpaDcCompType compType;
    /**< Compression type */
    CpaDcChecksum checksumType;
    /**< Type of checksum */
    CpaDcAutoSelectBest autoSelectBestHuffmanTree;
    /**< Indicates if the implementation selects the best Huffman encoding */
    CpaDcSessionDir sessDirection;
    /**< Session direction */
    CpaDcSessionState sessState;
    /**< Session state */
    Cpa32U deflateWindowSize;
    /**< Window size */
    CpaDcCompLvl compLevel;
    /**< Compression level */
    CpaDcCallbackFn pCompressionCb;
    /**< Callback function defined for the traditional compression session */
    OsalAtomic pendingStatelessCbCount;
    /**< Keeps track of number of pending requests on stateless session */
    OsalAtomic pendingStatefulCbCount;
    /**< Keeps track of number of pending requests on stateful session */
    Cpa64U pendingDpStatelessCbCount;
    /**< Keeps track of number of data plane pending requests on stateless
     * session */
    lac_lock_t sessionLock;
    /**< Lock used to provide exclusive access for number of stateful in-flight
     * requests update */
    CpaBoolean isDcDp;
    /**< Indicates if the data plane API is used */
    Cpa32U minContextSize;
    /**< Indicates the minimum size required to allocate the context buffer */
    CpaBufferList *pContextBuffer;
    /**< Context buffer */
    Cpa32U historyBuffSize;
    /**< Size of the history buffer */
    Cpa64U cumulativeConsumedBytes;
    /**< Cumulative amount of consumed bytes. Used to build the footer in the
     * stateful case */
    Cpa32U previousChecksum;
    /**< Save the previous value of the checksum. Used to process zero byte
     * stateful compression or decompression requests */
    CpaBoolean isSopForCompressionProcessed;
    /**< Indicates whether a Compression Request is received in this session */
    CpaBoolean isSopForDecompressionProcessed;
    /**< Indicates whether a Decompression Request is received in this session */
    CpaBoolean parityErrorEnabled;
    /**< Indicates whether parity error (fatal error) is reported */
} dc_session_desc_t;

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Initialise a compression session
 *
 * @description
 *      This function will initialise a compression session
 *
 * @param[in]       dcInstance       Instance handle derived from discovery
 *                                   functions
 * @param[in,out]   pSessionHandle   Pointer to a session handle
 * @param[in,out]   pSessionData     Pointer to a user instantiated structure
 *                                   containing session data
 * @param[in]       pContextBuffer   Pointer to context buffer
 *
 * @param[in]       callbackFn       For synchronous operation this callback
 *                                   shall be a null pointer
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully
 * @retval CPA_STATUS_FAIL           Function failed
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in
 * @retval CPA_STATUS_RESOURCE       Error related to system resources
 *****************************************************************************/
CpaStatus
dcInitSession(CpaInstanceHandle dcInstance,
        CpaDcSessionHandle pSessionHandle,
        CpaDcSessionSetupData* pSessionData,
        CpaBufferList *pContextBuffer,
        CpaDcCallbackFn callbackFn);

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Get the size of the memory required to hold the session information
 *
 * @description
 *      This function will get the size of the memory required to hold the
 *      session information
 *
 * @param[in]       dcInstance       Instance handle derived from discovery
 *                                   functions
 * @param[in]       pSessionData     Pointer to a user instantiated structure
 *                                   containing session data
 * @param[out]      pSessionSize     On return, this parameter will be the size
 *                                   of the memory that will be
 *                                   required by cpaDcInitSession() for session
 *                                   data.
 * @param[out]      pContextSize     On return, this parameter will be the size
 *                                   of the memory that will be required
 *                                   for context data.  Context data is
 *                                   save/restore data including history and
 *                                   any implementation specific data that is
 *                                   required for a save/restore operation.
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully
 * @retval CPA_STATUS_FAIL           Function failed
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in
 *****************************************************************************/
CpaStatus
dcGetSessionSize(CpaInstanceHandle dcInstance,
        CpaDcSessionSetupData* pSessionData,
        Cpa32U* pSessionSize,
        Cpa32U* pContextSize);

#endif /* DC_SESSION_H */
