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
 *  version: QAT1.5.L.1.11.0-36
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file dc_session.c
 *
 * @ingroup Dc_DataCompression
 *
 * @description
 *      Implementation of the Data Compression session operations.
 *
 *****************************************************************************/

/*
 *******************************************************************************
 * Include public/global header files
 *******************************************************************************
 */
#include "cpa.h"
#include "cpa_dc.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_debug.h"
#include "icp_adf_transport_dp.h"
#include "icp_qat_fw.h"
#include "icp_qat_fw_comp.h"
#include "icp_qat_hw.h"


/*
 *******************************************************************************
 * Include private header files
 *******************************************************************************
 */
#include "dc_session.h"
#include "dc_datapath.h"
#include "lac_mem_pools.h"
#include "sal_types_compression.h"
#include "lac_buffer_desc.h"
#include "sal_service_state.h"
#include "sal_qat_cmn_msg.h"

#ifdef ICP_PARAM_CHECK
/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Check that pSessionData is valid
 *
 * @description
 *      Check that all the parameters defined in the pSessionData are valid
 *
 * @param[in]       pSessionData     Pointer to a user instantiated structure
 *                                   containing session data
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in
 *
 *****************************************************************************/
STATIC CpaStatus
dcCheckSessionData(const CpaDcSessionSetupData *pSessionData)
{
    if((pSessionData->compLevel < CPA_DC_L1) ||
       (pSessionData->compLevel > CPA_DC_L9))
    {
        LAC_INVALID_PARAM_LOG("Invalid compLevel value");
        return CPA_STATUS_INVALID_PARAM;
    }

    if((pSessionData->compType < CPA_DC_LZS) ||
       (pSessionData->compType > CPA_DC_DEFLATE) ||
       (CPA_DC_ELZS == pSessionData->compType) ||
       (CPA_DC_LZSS == pSessionData->compType))
    {
        LAC_INVALID_PARAM_LOG("Invalid compType value");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* As V1.6 of Compression API, LZS has been deprecated */
    if(CPA_DC_LZS == pSessionData->compType) 
    {
        LAC_LOG("LZS has been deprecated\n");
    }
#ifndef ICP_DC_DYN_NOT_SUPPORTED
    if((pSessionData->huffType < CPA_DC_HT_STATIC) ||
       (pSessionData->huffType > CPA_DC_HT_FULL_DYNAMIC) ||
       (CPA_DC_HT_PRECOMP == pSessionData->huffType))
    {
        LAC_INVALID_PARAM_LOG("Invalid huffType value");
        return CPA_STATUS_INVALID_PARAM;
    }
#else
    if (pSessionData->huffType != CPA_DC_HT_STATIC)
    {
        LAC_INVALID_PARAM_LOG("Invalid huffType value, dynamic sessions "
                              "not supported");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    /*******************************************************************
        As of V1.6 of the Compression API, the CpaDcFileType fileType 
        in the structure CpaDcSessionSetupData has been deprecated 
        and should not be used. 
    *******************************************************************/ 
    if((pSessionData->fileType < CPA_DC_FT_ASCII) ||
       (pSessionData->fileType > CPA_DC_FT_OTHER))
    {
        LAC_INVALID_PARAM_LOG("Invalid fileType value");
        return CPA_STATUS_INVALID_PARAM;
    }

    if((pSessionData->sessDirection < CPA_DC_DIR_COMPRESS) ||
       (pSessionData->sessDirection > CPA_DC_DIR_COMBINED))
    {
        LAC_INVALID_PARAM_LOG("Invalid sessDirection value");
        return CPA_STATUS_INVALID_PARAM;
    }

    if((pSessionData->sessState < CPA_DC_STATEFUL) ||
       (pSessionData->sessState > CPA_DC_STATELESS))
    {
        LAC_INVALID_PARAM_LOG("Invalid sessState value");
        return CPA_STATUS_INVALID_PARAM;
    }

    /*******************************************************************
        As of V1.6 of the Compression API, the Cpa32U deflateWindowSize
        in the structure CpaDcSessionSetupData has been deprecated
        and the input from user will be ignored and will be modified to
        the right size according to the compression level by the 
        application. Here we need to check whether the right size has
        been set after this modification.
    *******************************************************************/

    if(CPA_DC_L1 == pSessionData->compLevel) 
    {     
        if(DC_32K_WINDOW_SIZE != pSessionData->deflateWindowSize)  
        {     
            LAC_INVALID_PARAM_LOG("The deflateWindowSize is not right\n");  
            return CPA_STATUS_INVALID_PARAM; 
        }     
    } else if(CPA_DC_L1 < pSessionData->compLevel) 
    {     
        if(DC_8K_WINDOW_SIZE != pSessionData->deflateWindowSize) 
        {     
            LAC_INVALID_PARAM_LOG("The deflateWindowSize is not right\n");  
            return CPA_STATUS_INVALID_PARAM; 
        }     
    }     

    if((pSessionData->checksum < CPA_DC_NONE) ||
       (pSessionData->checksum > CPA_DC_ADLER32))
    {
        LAC_INVALID_PARAM_LOG("Invalid checksum value");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* LZS only supports stateless */
    if((CPA_DC_STATEFUL == pSessionData->sessState)&&
       (CPA_DC_LZS == pSessionData->compType))
    {
        LAC_INVALID_PARAM_LOG("LZS only supports stateless");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* LZS only supports static trees */
    if((CPA_DC_HT_STATIC != pSessionData->huffType)&&
       (CPA_DC_LZS == pSessionData->compType))
    {
        LAC_INVALID_PARAM_LOG("LZS only supports static trees");
        return CPA_STATUS_INVALID_PARAM;
    }

    return CPA_STATUS_SUCCESS;
}
#endif

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Populate the compression hardware block
 *
 * @description
 *      This function will populate the compression hardware block and update
 *      the size in bytes of the block
 *
 * @param[in]   pSessionDesc            Pointer to the session descriptor
 * @param[in]   pCompHwBlock            Pointer to the compression hardware
 *                                      block
 * @param[out]  pCompHwBlockSizeBytes   Pointer to the size of the block
 * @param[in]   compDecomp              Direction of the operation
 *
 *****************************************************************************/
STATIC void
dcCompHwBlockPopulate(
    dc_session_desc_t *pSessionDesc,
    void *pCompHwBlock,
    Cpa32U *pCompHwBlockSizeBytes,
    dc_request_dir_t compDecomp)
{
    icp_qat_hw_compression_direction_t dir =
        ICP_QAT_HW_COMPRESSION_DIR_COMPRESS;
    icp_qat_hw_compression_delayed_match_t delay =
        ICP_QAT_HW_COMPRESSION_DELAYED_MATCH_DISABLED;
    icp_qat_hw_compression_algo_t algo =
        ICP_QAT_HW_COMPRESSION_ALGO_DEFLATE;
    icp_qat_hw_compression_depth_t depth =
        ICP_QAT_HW_COMPRESSION_DEPTH_1;
    icp_qat_hw_compression_file_type_t filetype =
        ICP_QAT_HW_COMPRESSION_FILE_TYPE_0;

    icp_qat_hw_compression_config_t *pCompConfig =
        (icp_qat_hw_compression_config_t *) LAC_CONST_PTR_CAST(pCompHwBlock);

    /* Set the direction */
    if(DC_COMPRESSION_REQUEST == compDecomp)
    {
        dir = ICP_QAT_HW_COMPRESSION_DIR_COMPRESS;
    }
    else
    {
        dir = ICP_QAT_HW_COMPRESSION_DIR_DECOMPRESS;
    }

    /* Set the delay */
    delay = ICP_QAT_HW_COMPRESSION_DELAYED_MATCH_DISABLED;

    /* Set the algorithm */
    if(CPA_DC_LZS == pSessionDesc->compType)
    {
        algo = ICP_QAT_HW_COMPRESSION_ALGO_LZS;
    }
    else if(CPA_DC_DEFLATE == pSessionDesc->compType)
    {
        algo = ICP_QAT_HW_COMPRESSION_ALGO_DEFLATE;
    }
    else
    {
        LAC_ENSURE(CPA_FALSE, "Algorithm not supported for Compression\n");
    }

    /* Set the depth */
    if(DC_DECOMPRESSION_REQUEST == compDecomp)
    {
        depth = ICP_QAT_HW_COMPRESSION_DEPTH_1;
    }
    else
    {
        if(pSessionDesc->compLevel >= CPA_DC_L4)
        {
            depth = ICP_QAT_HW_COMPRESSION_DEPTH_16;
        }
        else if(pSessionDesc->compLevel == CPA_DC_L3)
        {
            depth = ICP_QAT_HW_COMPRESSION_DEPTH_8;
        }
        else if(pSessionDesc->compLevel == CPA_DC_L2)
        {
            depth = ICP_QAT_HW_COMPRESSION_DEPTH_4;
        }
        else
        {
            depth = ICP_QAT_HW_COMPRESSION_DEPTH_1;
        }
    }

    /* The file type is set to ICP_QAT_HW_COMPRESSION_FILE_TYPE_0. The other
     * modes will be used in the future for precompiled huffman trees */
    filetype = ICP_QAT_HW_COMPRESSION_FILE_TYPE_0;

    pCompConfig->val = ICP_QAT_HW_COMPRESSION_CONFIG_BUILD(
                           dir, delay, algo, depth, filetype);

    pCompConfig->reserved = 0;

    *pCompHwBlockSizeBytes += sizeof(icp_qat_hw_compression_config_t);
}

#ifndef ICP_DC_DYN_NOT_SUPPORTED
/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Populate the translator hardware block
 *
 * @description
 *      This function will populate the translator hardware block and update
 *      the size in bytes of the block
 *
 * @param[in]   pSessionData            Pointer to a user instantiated structure
 *                                      containing session data
 * @param[in]   pTransHwBlock           Pointer to the translator hardware
 *                                      block
 * @param[out]  pTransHwBlockSizeBytes  Pointer to the size of the block
 *
 *****************************************************************************/
STATIC void
dcTransHwBlockPopulate(
    CpaDcSessionSetupData *pSessionData,
    void *pTransHwBlock,
    Cpa32U *pTransHwBlockSizeBytes)
{
    icp_qat_hw_translator_config_t *pTransConfig =
        (icp_qat_hw_translator_config_t *) LAC_CONST_PTR_CAST(pTransHwBlock);

    pTransConfig->val = ICP_QAT_HW_TRANSLATOR_CONFIG_BUILD(
                            ICP_QAT_HW_TRANSLATOR_LFCT1_DISABLED,
                            ICP_QAT_HW_TRANSLATOR_LFCT0_ENABLED,
                            ICP_QAT_HW_TRANSLATOR_LFCT0_RESET_ON,
                            ICP_QAT_HW_TRANSLATOR_SELECT_LFCT0);

    pTransConfig->reserved = 0;

    *pTransHwBlockSizeBytes += sizeof(icp_qat_hw_compression_config_t);
}
#endif

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Populate the compression content descriptor
 *
 * @description
 *      This function will populate the compression content descriptor
 *
 * @param[in]   pService                Pointer to the service
 * @param[in]   pSessionDesc            Pointer to the session descriptor
 * @param[in]   contextBufferAddrPhys   Physical address of the context buffer
 * @param[out]  pCompControlBlock       Pointer to the compression control block
 * @param[in]   pHwBlockBase            Pointer to the hardware base block
 * @param[in]   compBlkOffsetInHwBlock  Compression offset in the hardware block
 * @param[in]   nextSlice               Next slice
 * @param[in]   pCompHwBlockSizeBytes   Pointer to the size of the block
 * @param[in]   compDecomp              Direction of the operation
 *
 *****************************************************************************/
STATIC void
dcCompContentDescPopulate(
    sal_compression_service_t *pService,
    dc_session_desc_t *pSessionDesc,
    CpaPhysicalAddr contextBufferAddrPhys,
    icp_qat_fw_comp_hdr_t *pCompControlBlock,
    void *pHwBlockBase,
    Cpa32U compBlkOffsetInHwBlock,
    icp_qat_fw_slice_t nextSlice,
    Cpa32U *pCompHwBlockSizeBytes,
    dc_request_dir_t compDecomp)
{
    CpaBoolean bankEnabled = CPA_FALSE;

    LAC_ENSURE_NOT_NULL(pCompControlBlock);
    LAC_ENSURE_NOT_NULL(pHwBlockBase);
    LAC_ENSURE_NOT_NULL(pCompHwBlockSizeBytes);

    pCompControlBlock->next_id = nextSlice;
    pCompControlBlock->curr_id = ICP_QAT_FW_SLICE_COMP;
    pCompControlBlock->offset = LAC_BYTES_TO_QUADWORDS(compBlkOffsetInHwBlock);

    /* In case ICP_PARAM_CHECK is not enabled. Here we recheck whether the right 
     * deflateWindowSize has been set, if the size user set is not right, we 
     * reset it to the right size and print a message to the user */

    if(CPA_DC_L1 == pSessionDesc->compLevel) 
    {
        pSessionDesc->deflateWindowSize = DC_32K_WINDOW_SIZE;
    } 
    else if(CPA_DC_L1 < pSessionDesc->compLevel) 
    {
        pSessionDesc->deflateWindowSize = DC_8K_WINDOW_SIZE;
    }

    /* Build the ram bank flags in the compression content descriptor which
     * specify which banks are used to save history */
    if((CPA_DC_STATEFUL == pSessionDesc->sessState)
     &&(CPA_DC_DEFLATE == pSessionDesc->compType)
     &&(DC_COMPRESSION_REQUEST == compDecomp))
    {
        if(CPA_DC_L1 == pSessionDesc->compLevel) 
        {     
            /* Enables bank A, B, C, D, F, G */ 
            pCompControlBlock->ram_flags = 
                ICP_QAT_FW_COMP_RAM_FLAGS_BUILD(ICP_QAT_FW_COMP_BANK_DISABLED, 
                   ICP_QAT_FW_COMP_BANK_ENABLED, ICP_QAT_FW_COMP_BANK_ENABLED, 
                   ICP_QAT_FW_COMP_BANK_DISABLED, ICP_QAT_FW_COMP_BANK_ENABLED, 
                   ICP_QAT_FW_COMP_BANK_ENABLED, ICP_QAT_FW_COMP_BANK_ENABLED, 
                   ICP_QAT_FW_COMP_BANK_ENABLED); 
 
            bankEnabled = CPA_TRUE; 
        }     
        else if(CPA_DC_L2 <= pSessionDesc->compLevel)  
        {     
            /* Enables bank A, C, D, F, G */ 
            pCompControlBlock->ram_flags = 
                ICP_QAT_FW_COMP_RAM_FLAGS_BUILD(ICP_QAT_FW_COMP_BANK_DISABLED, 
                   ICP_QAT_FW_COMP_BANK_ENABLED, ICP_QAT_FW_COMP_BANK_ENABLED, 
                   ICP_QAT_FW_COMP_BANK_DISABLED, ICP_QAT_FW_COMP_BANK_ENABLED, 
                   ICP_QAT_FW_COMP_BANK_ENABLED, ICP_QAT_FW_COMP_BANK_DISABLED, 
                   ICP_QAT_FW_COMP_BANK_ENABLED); 
 
            bankEnabled = CPA_TRUE; 
        }     
    }
    else if((CPA_DC_STATEFUL == pSessionDesc->sessState)
          &&(CPA_DC_DEFLATE == pSessionDesc->compType)
          &&(DC_DECOMPRESSION_REQUEST == compDecomp))
    {
        /* Enables bank A, B, C, D, E */
        pCompControlBlock->ram_flags =
            ICP_QAT_FW_COMP_RAM_FLAGS_BUILD(ICP_QAT_FW_COMP_BANK_DISABLED,
                ICP_QAT_FW_COMP_BANK_DISABLED, ICP_QAT_FW_COMP_BANK_DISABLED,
                ICP_QAT_FW_COMP_BANK_ENABLED, ICP_QAT_FW_COMP_BANK_ENABLED,
                ICP_QAT_FW_COMP_BANK_ENABLED, ICP_QAT_FW_COMP_BANK_ENABLED,
                ICP_QAT_FW_COMP_BANK_ENABLED);

        bankEnabled = CPA_TRUE;
    }
    else
    {
        pCompControlBlock->ram_flags =
            ICP_QAT_FW_COMP_RAM_FLAGS_BUILD(ICP_QAT_FW_COMP_BANK_DISABLED,
                ICP_QAT_FW_COMP_BANK_DISABLED, ICP_QAT_FW_COMP_BANK_DISABLED,
                ICP_QAT_FW_COMP_BANK_DISABLED, ICP_QAT_FW_COMP_BANK_DISABLED,
                ICP_QAT_FW_COMP_BANK_DISABLED, ICP_QAT_FW_COMP_BANK_DISABLED,
                ICP_QAT_FW_COMP_BANK_DISABLED);
    }

    if(DC_COMPRESSION_REQUEST == compDecomp)
    {
        LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_EXTERNAL(
            pService->generic_service_info,
            pCompControlBlock->comp_state_addr,
            pSessionDesc->stateRegistersComp);
    }
    else
    {
        LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_EXTERNAL(
            pService->generic_service_info,
            pCompControlBlock->comp_state_addr,
            pSessionDesc->stateRegistersDecomp);
    }

    if(CPA_TRUE == bankEnabled)
    {
        pCompControlBlock->ram_banks_addr = contextBufferAddrPhys;
    }
    else
    {
        pCompControlBlock->ram_banks_addr = 0;
    }

    pCompControlBlock->resrvd = 0;

    /* Populate Compression Hardware Setup Block */
    dcCompHwBlockPopulate(pSessionDesc,
                          ((Cpa8U *)pHwBlockBase +
                              compBlkOffsetInHwBlock),
                          pCompHwBlockSizeBytes,
                          compDecomp);
}

#ifndef ICP_DC_DYN_NOT_SUPPORTED
/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Populate the translator content descriptor
 *
 * @description
 *      This function will populate the translator content descriptor
 *
 * @param[in]   pSessionData             Pointer to a user instantiated
 *                                       structure containing session data
 * @param[out]  pTransControlBlock       Pointer to the translator control block
 * @param[in]   pHwBlockBase             Pointer to the hardware base block
 * @param[in]   transBlkOffsetInHwBlock  Translator offset in the hardware block
 * @param[in]   nextSlice                Next slice
 * @param[in]   pTransHwBlockSizeBytes   Pointer to the size of the block
 *
 *****************************************************************************/
STATIC void
dcTransContentDescPopulate(
    CpaDcSessionSetupData* pSessionData,
    icp_qat_fw_trans_hdr_t *pTransControlBlock,
    void *pHwBlockBase,
    Cpa32U transBlkOffsetInHwBlock,
    icp_qat_fw_slice_t nextSlice,
    Cpa32U *pTransHwBlockSizeBytes)
{
    LAC_ENSURE_NOT_NULL(pTransControlBlock);
    LAC_ENSURE_NOT_NULL(pHwBlockBase);
    LAC_ENSURE_NOT_NULL(pCompHwBlockSizeBytes);

    pTransControlBlock->next_id = nextSlice;
    pTransControlBlock->curr_id = ICP_QAT_FW_SLICE_XLAT;
    pTransControlBlock->offset =
        LAC_BYTES_TO_QUADWORDS(transBlkOffsetInHwBlock);

    /* Populate Translator Hardware Setup Block */
    dcTransHwBlockPopulate(pSessionData,
                           ((Cpa8U *)pHwBlockBase +
                               transBlkOffsetInHwBlock),
                           pTransHwBlockSizeBytes);
}
#endif

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Get the context size and the history size
 *
 * @description
 *      This function will get the size of the context buffer and the history
 *      buffer. The history buffer is a subset of the context buffer and its
 *      size is needed for stateful compression. This function can be called
 *      with a NULL pointer for the history size if not needed
 *
 * @param[in]   pSessionData       Pointer to a user instantiated
 *                                 structure containing session data
 * @param[out]  pContextSize       Pointer to the context size
 * @param[out]  pHistorySize       Pointer to the history size
 *
 *****************************************************************************/
STATIC void
dcGetContextSize(CpaDcSessionSetupData* pSessionData,
        Cpa32U* pContextSize, Cpa32U* pHistorySize)
{
    /* Get the context and history size for session data */
    if((CPA_DC_STATEFUL == pSessionData->sessState)
     &&(CPA_DC_DEFLATE == pSessionData->compType))
    {
        /* Get the history size for session data */
        if(NULL != pHistorySize)
        {
            if(CPA_DC_L1 == pSessionData->compLevel)
            {
                *pHistorySize = DC_MAX_HISTORY_SIZE;
            }
            else if(CPA_DC_L1 < (CpaDcCompLvl)pSessionData->sessDirection)
            {
                *pHistorySize = DC_MIN_HISTORY_SIZE;
            }
        }
        /* Get the context size for session data */
        *pContextSize = DC_DEFLATE_CONTEXT_SIZE;
    }
    else
    {
        *pContextSize = 0;
        if(NULL != pHistorySize)
        {
            *pHistorySize = 0;
        }
    }
}

CpaStatus
findIndexInSglBuffer (CpaBufferList *sourceBuffer, sgl_buffer_index_t *index, Cpa32U OffsetToFind)
{
    Cpa32U i = 0;
    Cpa32U totalDataLen = 0;
    Cpa32U previousTotalDataLen = 0;

    for (i=0; i<sourceBuffer->numBuffers; i++)
    {
        previousTotalDataLen = totalDataLen;
        totalDataLen += sourceBuffer->pBuffers[i].dataLenInBytes;
        if (totalDataLen >= OffsetToFind)
        {
            index->bufferIndex = i;
            index->OffsetInBuffer = OffsetToFind - previousTotalDataLen;
            return CPA_STATUS_SUCCESS;
        }
    }
    return CPA_STATUS_FAIL;
}

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Get the location of bank E in the context buffer
 *
 * @description
 *      This function will get the buffer index and the offset in the buffer
 *      that correspond to Bank E location. This function updates two
 *      bank_index_t type structures with start/stop index and start/stop
 *      offset in context buffer.
 *
 * @param[in]   pSessionDescriptor       Pointer to structure containing
 *                                       session descriptor
 * @param[out]  bankEStartIndex          Structure holding buffer index and
 *                                       buffer offset for beginning of bank E.
 * @param[out]  bankEEndIndex            Structure holding buffer index and
 *                                       buffer offset for end of bank E.
 *
 *****************************************************************************/
STATIC CpaStatus
findBankE(dc_session_desc_t *pSessionDescriptor)
{
    CpaBufferList * const historyBuffer = pSessionDescriptor->pContextBuffer;
    sgl_buffer_index_t * const startIndex = &(pSessionDescriptor->bankEStartIndex);
    sgl_buffer_index_t * const endIndex = &(pSessionDescriptor->bankEEndIndex);
    CpaStatus status = CPA_STATUS_SUCCESS;

    status = findIndexInSglBuffer (historyBuffer,
            startIndex,
            OFFSET_TO_BANK_E);
    /* Return an error if the either the start index has not been found */
    if (CPA_STATUS_FAIL == status)
    {
        LAC_LOG_ERROR("Unable to find beginning of Bank E "
                "in context buffer.\n");
        return CPA_STATUS_FAIL;
    }
    else
    {
        status = findIndexInSglBuffer (historyBuffer,
                endIndex,
                OFFSET_TO_BANK_E + BANK_E_SIZE);
    }
    /* Return an error if the either the end index has not been found */
    if (CPA_STATUS_FAIL == status)
    {
        LAC_LOG_ERROR("Unable to find end of Bank E "
                        "in context buffer.\n");
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

CpaStatus
dcInitSession(CpaInstanceHandle dcInstance,
        CpaDcSessionHandle pSessionHandle,
        CpaDcSessionSetupData* pSessionData,
        CpaBufferList *pContextBuffer,
        CpaDcCallbackFn callbackFn)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_qat_content_desc_info_t *pCdInfoComp = NULL, *pCdInfoDecomp = NULL;
    sal_compression_service_t* pService = NULL;
    icp_qat_fw_la_bulk_req_t *pReqCache = NULL;
    dc_session_desc_t* pSessionDesc = NULL;
    CpaPhysicalAddr contextAddrPhys = 0;
    CpaPhysicalAddr physAddress = 0;
    CpaPhysicalAddr physAddressAligned = 0;
    Cpa32U ctrlBlockHdrSzComp = 0, ctrlBlockHdrSzDecomp = 0;

    CpaBoolean parity_report = CPA_FALSE;
    Cpa8U *pCtrlBlockComp = NULL, *pCtrlBlockDecomp = NULL;
    Cpa8U *pHwBlockBaseComp = NULL, *pHwBlockBaseDecomp = NULL;
    Cpa32U ctrlBlockOffsetComp = 0, ctrlBlockOffsetDecomp = 0;
    Cpa32U hwBlockOffsetComp = 0, hwBlockOffsetDecomp = 0;
    Cpa32U minContextSize = 0, historySize = 0;
    Cpa32U i = 0;
    Cpa32U contextBufferSize = 0;
    CpaFlatBuffer *pFlatBuffer = NULL;

    icp_qat_fw_la_cmd_id_t dcCmdId = (icp_qat_fw_la_cmd_id_t)(ICP_QAT_FW_COMP_CMD_STATIC);
    Cpa8U reqParamsOffset = (Cpa8U)sizeof(icp_qat_fw_comp_req_params_t);

    Cpa16U cmdFlags = 0;
    Cpa8U autoSelectBest = ICP_QAT_FW_COMP_NOT_AUTO_SELECT_BEST;
    Cpa8U enhancedAutoSelectBest = ICP_QAT_FW_COMP_NOT_ENH_AUTO_SELECT_BEST;
    Cpa8U disableType0EnhancedAutoSelectBest =
            ICP_QAT_FW_COMP_NOT_DISABLE_TYPE0_ENH_AUTO_SELECT_BEST;

    pService = (sal_compression_service_t*) dcInstance;

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(pSessionHandle);
    LAC_CHECK_NULL_PARAM(pSessionData);

    if(CPA_DC_L1 == pSessionData->compLevel) 
    {  
        pSessionData->deflateWindowSize = DC_32K_WINDOW_SIZE; 
    } 
    else if(CPA_DC_L1 < pSessionData->compLevel) 
    {
        pSessionData->deflateWindowSize = DC_8K_WINDOW_SIZE; 
    }     

    /* Check that the parameters defined in the pSessionData are valid */
    if(dcCheckSessionData(pSessionData) != CPA_STATUS_SUCCESS)
    {
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

#ifndef ICP_DC_DYN_NOT_SUPPORTED
    if(CPA_DC_HT_FULL_DYNAMIC == pSessionData->huffType)
    {
        /* Test if neither eSRAM nor DRAM is available for the intermediate
         * buffers */
        if((NULL == pService->pInterBuff1)
           && (0 == pService->interBuff1eSRAMPhyAddr))
        {
            if(CPA_DC_ASB_DISABLED != pSessionData->autoSelectBestHuffmanTree)
            {
                /* Define the Huffman tree as static */
                pSessionData->huffType = CPA_DC_HT_STATIC;
            }
            else
            {
                LAC_LOG_ERROR("No buffer defined for this instance - see "
                        "cpaDcStartInstance");
                return CPA_STATUS_RESOURCE;
            }
        }
    }
#else
    if(CPA_DC_HT_FULL_DYNAMIC == pSessionData->huffType)
    {
        LAC_INVALID_PARAM_LOG("Invalid huffType value, dynamic sessions "
                              "not supported");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    if((CPA_DC_STATEFUL == pSessionData->sessState)&&
       (CPA_DC_DEFLATE == pSessionData->compType))
    {
        /* Get the size of the context buffer */
        dcGetContextSize(pSessionData, &minContextSize, &historySize);

#ifdef ICP_PARAM_CHECK
        /* If the minContextSize is zero it means we will not save or restore
         * any history */
        if(0 != minContextSize)
        {
            Cpa64U contextBuffSize = 0;

            LAC_CHECK_NULL_PARAM(pContextBuffer);

            if(LacBuffDesc_BufferListVerify(pContextBuffer, &contextBuffSize,
                    LAC_NO_ALIGNMENT_SHIFT)
                        != CPA_STATUS_SUCCESS)
            {
                return CPA_STATUS_INVALID_PARAM;
            }

            /* Ensure that the context buffer size is greater or equal
             * to minContextSize */
            if(contextBuffSize < minContextSize)
            {
                LAC_INVALID_PARAM_LOG1("Context buffer size should be "
                    "greater or equal to %d", minContextSize);
                return CPA_STATUS_INVALID_PARAM;
            }
        }
#endif
    }

    /* Re-align the session structure to 64 byte alignment */
    physAddress = LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                      pService->generic_service_info,
                      (Cpa8U *)pSessionHandle + sizeof(void *));

    if (physAddress == 0)
    {
        LAC_LOG_ERROR("Unable to get the physical address of the session\n");
        return CPA_STATUS_FAIL;
    }

    physAddressAligned = (CpaPhysicalAddr)LAC_ALIGN_POW2_ROUNDUP(
                             physAddress, LAC_64BYTE_ALIGNMENT);

    pSessionDesc = (dc_session_desc_t *)
                    /* Move the session pointer by the physical offset
                    between aligned and unaligned memory */
                    ((Cpa8U *) pSessionHandle + sizeof(void *)
                    + (physAddressAligned-physAddress));

    /* Save the aligned pointer in the first bytes (size of LAC_ARCH_UINT)
     * of the session memory */
    *((LAC_ARCH_UINT *)pSessionHandle) = (LAC_ARCH_UINT)pSessionDesc;

    /* Zero the compression session */
    LAC_OS_BZERO(pSessionDesc, sizeof(dc_session_desc_t));

    /* Write the buffer descriptor for context/history */
    if(0 != minContextSize)
    {
        status = LacBuffDesc_BufferListDescWrite(pContextBuffer,
                 &contextAddrPhys,
                 CPA_FALSE,
                 &(pService->generic_service_info));

        if (status != CPA_STATUS_SUCCESS)
        {
            return status;
        }

        pSessionDesc->pContextBuffer = pContextBuffer;
        pSessionDesc->historyBuffSize = historySize;
        pSessionDesc->byteToUpdateOffset = 0;
    }

    pSessionDesc->cumulativeConsumedBytes = 0;

    /* Setup content descriptor info structures
     * assumption that the compression content descriptor is the first field in
     * the session descriptor */
    if(CPA_DC_DIR_DECOMPRESS != pSessionData->sessDirection)
    {
        pSessionDesc->contentDescInfoComp.pData = (Cpa8U *)pSessionDesc;
        pSessionDesc->contentDescInfoComp.dataPhys = physAddressAligned;
    }
    if(CPA_DC_DIR_COMPRESS != pSessionData->sessDirection)
    {
        pSessionDesc->contentDescInfoDecomp.pData =
            (Cpa8U *)(&(pSessionDesc->contentDescDecomp));
        pSessionDesc->contentDescInfoDecomp.dataPhys =
            LAC_OS_VIRT_TO_PHYS_EXTERNAL(
                pService->generic_service_info,
                &(pSessionDesc->contentDescDecomp));

        if (pSessionDesc->contentDescInfoDecomp.dataPhys == 0)
        {
            LAC_LOG_ERROR("Unable to get the physical address of the "
                "content descriptor\n");
            return CPA_STATUS_FAIL;
        }
    }

    /* If the huffman type is static or the direction is decompress there is no
     * need to use the translator slice*/
    if((CPA_DC_HT_STATIC == pSessionData->huffType)||
       (CPA_DC_DIR_DECOMPRESS == pSessionData->sessDirection))
    {
        /* The compression content descriptor is required */
        if((CPA_DC_DIR_COMPRESS == pSessionData->sessDirection) ||
           (CPA_DC_DIR_COMBINED == pSessionData->sessDirection))
        {
            pSessionDesc->qatSlicesComp[DC_QAT_SLICE_INDEX0] =
                ICP_QAT_FW_SLICE_COMP;
            pSessionDesc->qatSlicesComp[DC_QAT_SLICE_INDEX1] =
                ICP_QAT_FW_SLICE_DRAM_WR;
            ctrlBlockHdrSzComp = sizeof(icp_qat_fw_comp_hdr_t);
        }

        /* The decompression content descriptor is required */
        if((CPA_DC_DIR_DECOMPRESS == pSessionData->sessDirection) ||
           (CPA_DC_DIR_COMBINED == pSessionData->sessDirection))
        {
            pSessionDesc->qatSlicesDecomp[DC_QAT_SLICE_INDEX0] =
                ICP_QAT_FW_SLICE_COMP;
            pSessionDesc->qatSlicesDecomp[DC_QAT_SLICE_INDEX1] =
                ICP_QAT_FW_SLICE_DRAM_WR;
            ctrlBlockHdrSzDecomp = sizeof(icp_qat_fw_comp_hdr_t);
        }

        pSessionDesc->cmnReqFlags =
             ICP_QAT_FW_COMN_FLAGS_BUILD(ICP_QAT_FW_COMN_ORD_FLAG_STRICT,
                                   QAT_COMN_PTR_TYPE_SGL,
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE,
                                   QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                                   QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                                   QAT_COMN_XLAT_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CPR_SLICE_REQUIRED,
                                   QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                                   QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                                   QAT_COMN_RND_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED);
    }
#ifndef ICP_DC_DYN_NOT_SUPPORTED
    /* We need to use the translator slice */
    else if(CPA_DC_HT_FULL_DYNAMIC == pSessionData->huffType)
    {
        pSessionDesc->qatSlicesComp[DC_QAT_SLICE_INDEX0] =
            ICP_QAT_FW_SLICE_COMP;
        pSessionDesc->qatSlicesComp[DC_QAT_SLICE_INDEX1] =
            ICP_QAT_FW_SLICE_XLAT;
        pSessionDesc->qatSlicesComp[DC_QAT_SLICE_INDEX2] =
            ICP_QAT_FW_SLICE_DRAM_WR;
        ctrlBlockHdrSzComp = sizeof(icp_qat_fw_comp_hdr_t)
                           + sizeof(icp_qat_fw_trans_hdr_t);

        /* The decompression content descriptor is required */
        if(CPA_DC_DIR_COMBINED == pSessionData->sessDirection)
        {
            pSessionDesc->qatSlicesDecomp[DC_QAT_SLICE_INDEX0] =
                ICP_QAT_FW_SLICE_COMP;
            pSessionDesc->qatSlicesDecomp[DC_QAT_SLICE_INDEX1] =
                ICP_QAT_FW_SLICE_DRAM_WR;
            ctrlBlockHdrSzDecomp = sizeof(icp_qat_fw_comp_hdr_t);
        }

        pSessionDesc->cmnReqFlags =
             ICP_QAT_FW_COMN_FLAGS_BUILD(ICP_QAT_FW_COMN_ORD_FLAG_STRICT,
                                   QAT_COMN_PTR_TYPE_SGL,
                                   ICP_QAT_FW_COMN_EXT_ORD_FLAG_NONE,
                                   QAT_COMN_SHRAM_INIT_NOT_REQUIRED,
                                   QAT_COMN_REGEX_SLICE_NOT_REQUIRED,
                                   QAT_COMN_XLAT_SLICE_REQUIRED,
                                   QAT_COMN_CPR_SLICE_REQUIRED,
                                   QAT_COMN_BULK_SLICE_NOT_REQUIRED,
                                   QAT_COMN_STORAGE_SLICE_NOT_REQUIRED,
                                   QAT_COMN_RND_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_PKE0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_AUTH0_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER1_SLICE_NOT_REQUIRED,
                                   QAT_COMN_CIPHER0_SLICE_NOT_REQUIRED);
    }
#endif
    else
    {
        LAC_INVALID_PARAM_LOG("Invalid huffType value, dynamic sessions "
                              "not supported");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Initialise pSessionDesc */
    if(CPA_DC_STATEFUL == pSessionData->sessState)
    {
        /* Init the spinlock used to lock the access to the number of stateful
         * in-flight requests */
        status = LAC_SPINLOCK_INIT(&(pSessionDesc->sessionLock));
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Spinlock init failed for sessionLock");
            return CPA_STATUS_RESOURCE;
        }
    }

    pSessionDesc->requestType = DC_REQUEST_FIRST;
    pSessionDesc->huffType = pSessionData->huffType;
    pSessionDesc->compType = pSessionData->compType;
    pSessionDesc->checksumType = pSessionData->checksum;
    pSessionDesc->autoSelectBestHuffmanTree =
    pSessionData->autoSelectBestHuffmanTree;
    pSessionDesc->sessDirection = pSessionData->sessDirection;
    pSessionDesc->sessState = pSessionData->sessState;
    pSessionDesc->deflateWindowSize = pSessionData->deflateWindowSize;
    pSessionDesc->compLevel = pSessionData->compLevel;
    pSessionDesc->isDcDp = CPA_FALSE;
    pSessionDesc->minContextSize = minContextSize;
    pSessionDesc->isSopForCompressionProcessed = CPA_FALSE;
    pSessionDesc->isSopForDecompressionProcessed = CPA_FALSE;

    /* Read the parity_report state and update the session descriptor. */
    status = SalCtrl_GetParityReportValue(dcInstance, &parity_report);

    if (status != CPA_STATUS_SUCCESS)
    {
        return status;
    }
    pSessionDesc->parityErrorEnabled = parity_report;
 
    if(CPA_DC_ADLER32 == pSessionDesc->checksumType)
    {
        pSessionDesc->previousChecksum = 1;
    }
    else
    {
        pSessionDesc->previousChecksum = 0;
    }

    /* Set the flow ID */
    pSessionDesc->flowId = 0;

    /* For asynchronous - use the user supplied callback
     * for synchronous - use the internal synchronous callback */
    pSessionDesc->pCompressionCb =
        ((void*) NULL != (void *) callbackFn) ?
          callbackFn :
          LacSync_GenWakeupSyncCaller;

    /* Reset the pending callback counters */
    osalAtomicSet(0, &pSessionDesc->pendingStatelessCbCount);
    osalAtomicSet(0, &pSessionDesc->pendingStatefulCbCount);
    pSessionDesc->pendingDpStatelessCbCount = 0;

    /* Pointers to the content descriptor info structures */
    pCdInfoComp = &(pSessionDesc->contentDescInfoComp);
    pCdInfoDecomp = &(pSessionDesc->contentDescInfoDecomp);

    /* Pointers to the data of the content descriptor info structures */
    pCtrlBlockComp = (Cpa8U *)pCdInfoComp->pData;
    pCtrlBlockDecomp = (Cpa8U *)pCdInfoDecomp->pData;

    /* Pointers to the hardware blocks */
    pHwBlockBaseComp = pCtrlBlockComp + ctrlBlockHdrSzComp;
    pHwBlockBaseDecomp = pCtrlBlockDecomp + ctrlBlockHdrSzDecomp;

    /* Walk the QAT slice chain for compression */
    if(CPA_DC_DIR_DECOMPRESS != pSessionData->sessDirection)
    {
        for (i = 0; (i < DC_MAX_NUM_QAT_SLICES_COMP - 1); i++)
        {
            if (ICP_QAT_FW_SLICE_COMP == pSessionDesc->qatSlicesComp[i])
            {
                icp_qat_fw_comp_hdr_t *pCompControlBlock =
                    (icp_qat_fw_comp_hdr_t *)(pCtrlBlockComp +
                                              ctrlBlockOffsetComp);
                Cpa32U compHwBlkSizeBytes = 0;

                /* Populate the compression section of the content descriptor */
                dcCompContentDescPopulate(
                    pService,
                    pSessionDesc,
                    contextAddrPhys,
                    pCompControlBlock,
                    pHwBlockBaseComp,
                    hwBlockOffsetComp,
                    pSessionDesc->qatSlicesComp[i + 1],
                    &compHwBlkSizeBytes,
                    DC_COMPRESSION_REQUEST);

                ctrlBlockOffsetComp += sizeof(*pCompControlBlock);
                hwBlockOffsetComp += compHwBlkSizeBytes;

                /* Set the first 2 LWs of icp_qat_fw_comp_req_params_t */
                pSessionDesc->dcRequestParamsCompCache =
                    ((pCompControlBlock->next_id
                            << DC_QAT_NEXT_ID_BIT_OFFSET) +
                     (pCompControlBlock->curr_id
                            << DC_QAT_CURR_ID_BIT_OFFSET));

                /* Initialise the translator request parameters cache to 0 in
                 * case dynamic is not used */
                pSessionDesc->dcRequestParamsTransCache = 0;
            }
#ifndef ICP_DC_DYN_NOT_SUPPORTED
            else if (ICP_QAT_FW_SLICE_XLAT ==
                         pSessionDesc->qatSlicesComp[i])
            {
                icp_qat_fw_trans_hdr_t *pTransControlBlock =
                    (icp_qat_fw_trans_hdr_t *)
                        (pCtrlBlockComp + ctrlBlockOffsetComp);
                Cpa32U transHwBlkSizeBytes = 0;

                /* Populate the translator section of the content descriptor */
                dcTransContentDescPopulate(
                    pSessionData,
                    pTransControlBlock,
                    pHwBlockBaseComp,
                    hwBlockOffsetComp,
                    pSessionDesc->qatSlicesComp[i + 1],
                    &transHwBlkSizeBytes);

                ctrlBlockOffsetComp += sizeof(*pTransControlBlock);
                hwBlockOffsetComp += transHwBlkSizeBytes;

                /* Set the first 2 LWs of icp_qat_fw_trans_req_params_t */
                pSessionDesc->dcRequestParamsTransCache =
                    ((pTransControlBlock->next_id
                            << DC_QAT_NEXT_ID_BIT_OFFSET) +
                     (pTransControlBlock->curr_id
                             << DC_QAT_CURR_ID_BIT_OFFSET));
            }
#endif
            /* Chain terminators */
            else if (ICP_QAT_FW_SLICE_DRAM_WR ==
                         pSessionDesc->qatSlicesComp[i])
            {
                /* End of chain */
                break;
            }
        } /* for (i = 0; ... */
    }

    /* Populate the compression section of the content descriptor for
     * the decompression case or combined */
    if(CPA_DC_DIR_COMPRESS != pSessionData->sessDirection)
    {
        icp_qat_fw_comp_hdr_t *pDecompControlBlock =
            (icp_qat_fw_comp_hdr_t *)
                (pCtrlBlockDecomp + ctrlBlockOffsetDecomp);
        Cpa32U decompHwBlkSizeBytes = 0;

        dcCompContentDescPopulate(
            pService,
            pSessionDesc,
            contextAddrPhys,
            pDecompControlBlock,
            pHwBlockBaseDecomp,
            hwBlockOffsetDecomp,
            ICP_QAT_FW_SLICE_DRAM_WR,
            &decompHwBlkSizeBytes,
            DC_DECOMPRESSION_REQUEST);

        ctrlBlockOffsetDecomp += sizeof(*pDecompControlBlock);
        hwBlockOffsetDecomp += decompHwBlkSizeBytes;

        /* Set the first 2 LWs of icp_qat_fw_comp_req_params_t */
        pSessionDesc->dcRequestParamsDecompCache =
            ((pDecompControlBlock->next_id << DC_QAT_NEXT_ID_BIT_OFFSET) +
             (pDecompControlBlock->curr_id << DC_QAT_CURR_ID_BIT_OFFSET));
    }

    pCdInfoComp->dataPhys = LAC_OS_VIRT_TO_PHYS_EXTERNAL(
        (pService->generic_service_info),
        &(pSessionDesc->contentDescComp));

    if (pCdInfoComp->dataPhys == 0)
    {
        LAC_LOG_ERROR("Unable to get the physical address of the"
            "Compression content descriptor\n");
        return CPA_STATUS_FAIL;
    }

    pCdInfoDecomp->dataPhys = LAC_OS_VIRT_TO_PHYS_EXTERNAL(
        (pService->generic_service_info),
        &(pSessionDesc->contentDescDecomp));

    if (pCdInfoDecomp->dataPhys == 0)
    {
        LAC_LOG_ERROR("Unable to get the physical address of the"
            "Decompression content descriptor\n");
        return CPA_STATUS_FAIL;
    }

    /* Fill out the remaining sal_qat_content_desc_info_t fields */
    pCdInfoComp->hdrSzQuadWords = LAC_BYTES_TO_QUADWORDS(ctrlBlockOffsetComp);
    pCdInfoComp->hwBlkSzQuadWords = LAC_BYTES_TO_QUADWORDS(hwBlockOffsetComp);
    pCdInfoDecomp->hdrSzQuadWords =
        LAC_BYTES_TO_QUADWORDS(ctrlBlockOffsetDecomp);
    pCdInfoDecomp->hwBlkSzQuadWords =
        LAC_BYTES_TO_QUADWORDS(hwBlockOffsetDecomp);

    if(CPA_DC_STATEFUL == pSessionData->sessState)
    {
        LAC_OS_BZERO(&pSessionDesc->stateRegistersComp,
                     sizeof(pSessionDesc->stateRegistersComp));

        LAC_OS_BZERO(&pSessionDesc->stateRegistersDecomp,
                     sizeof(pSessionDesc->stateRegistersDecomp));
    }

    if(CPA_DC_DIR_DECOMPRESS != pSessionData->sessDirection)
    {
        SalQatMsg_ReqTypePopulate(
            &pSessionDesc->reqCacheComp.comn_hdr.arch_if,
            ICP_ARCH_IF_REQ_QAT_FW_COMPRESS,
            pService->response_ring_id);
    }
    if(CPA_DC_DIR_COMPRESS != pSessionData->sessDirection)
    {
        SalQatMsg_ReqTypePopulate(
            &pSessionDesc->reqCacheDecomp.comn_hdr.arch_if,
            ICP_ARCH_IF_REQ_QAT_FW_COMPRESS,
            pService->response_ring_id);
    }

    /* Populate the cmdFlags */
    switch(pSessionDesc->autoSelectBestHuffmanTree)
    {
        case CPA_DC_ASB_DISABLED:
            break;
        case CPA_DC_ASB_STATIC_DYNAMIC:
            autoSelectBest = ICP_QAT_FW_COMP_AUTO_SELECT_BEST;
            break;
        case CPA_DC_ASB_UNCOMP_STATIC_DYNAMIC_WITH_STORED_HDRS:
            autoSelectBest = ICP_QAT_FW_COMP_AUTO_SELECT_BEST;
            enhancedAutoSelectBest = ICP_QAT_FW_COMP_ENH_AUTO_SELECT_BEST;
            break;
        case CPA_DC_ASB_UNCOMP_STATIC_DYNAMIC_WITH_NO_HDRS:
            autoSelectBest = ICP_QAT_FW_COMP_AUTO_SELECT_BEST;
            enhancedAutoSelectBest = ICP_QAT_FW_COMP_ENH_AUTO_SELECT_BEST;
            disableType0EnhancedAutoSelectBest =
                ICP_QAT_FW_COMP_DISABLE_TYPE0_ENH_AUTO_SELECT_BEST;
            break;
        default:
            break;
    }

    cmdFlags = ICP_QAT_FW_COMP_FLAGS_BUILD(
            ICP_QAT_FW_COMP_SOP,
            ICP_QAT_FW_COMP_EOP,
            ICP_QAT_FW_COMP_STATELESS_SESSION,
            autoSelectBest,
            enhancedAutoSelectBest,
            disableType0EnhancedAutoSelectBest,
            ICP_QAT_FW_COMP_BFINAL);

    if(CPA_DC_DIR_DECOMPRESS != pSessionData->sessDirection)
    {
#ifndef ICP_DC_DYN_NOT_SUPPORTED
        if(CPA_DC_HT_FULL_DYNAMIC == pSessionDesc->huffType)
        {
            dcCmdId = (icp_qat_fw_la_cmd_id_t)(ICP_QAT_FW_COMP_CMD_DYNAMIC);
            reqParamsOffset += (Cpa8U)sizeof(icp_qat_fw_trans_req_params_t);
        }
#endif
        pReqCache = (icp_qat_fw_la_bulk_req_t*)&(pSessionDesc->reqCacheComp);

        /* Populate LW1-4 of the common request message */
        SalQatMsg_CmnMsgHdrPopulate(pReqCache, pCdInfoComp,
            pSessionDesc->cmnReqFlags, pSessionDesc->flowId);

        /* Populate the service command request field of the request message */
        SalQatMsg_ServiceCmdPopulate(pReqCache, dcCmdId, cmdFlags);

        /* Set the request parameter block size */
        SalQatMsg_ReqParamsPopulate(pReqCache, 0, reqParamsOffset);
    }
    if(CPA_DC_DIR_COMPRESS != pSessionData->sessDirection)
    {
        dcCmdId = (icp_qat_fw_la_cmd_id_t)(ICP_QAT_FW_COMP_CMD_DECOMPRESS);
        pReqCache = (icp_qat_fw_la_bulk_req_t*)&(pSessionDesc->reqCacheDecomp);

        /* Populate LW1-4 of the common request message */
        SalQatMsg_CmnMsgHdrPopulate(pReqCache, pCdInfoDecomp,
            pSessionDesc->cmnReqFlags, pSessionDesc->flowId);

        /* Populate the service command request field of the request message */
        SalQatMsg_ServiceCmdPopulate(pReqCache, dcCmdId, cmdFlags);

        /* Set the request parameter block size */
        SalQatMsg_ReqParamsPopulate(pReqCache, 0, reqParamsOffset);
    }

    /* Calculate size of Context buffer before looking for Bank E */
    pSessionDesc->bankEAvailable = CPA_FALSE;
    if (NULL != pSessionDesc->pContextBuffer)
    {
        pFlatBuffer = &pSessionDesc->pContextBuffer->pBuffers[0];
        for (i=0; i<pSessionDesc->pContextBuffer->numBuffers; i++)
        {
            contextBufferSize += pFlatBuffer->dataLenInBytes;
            pFlatBuffer++;
        }
    }


    /* Find bank E boundaries */
    if((CPA_DC_STATEFUL == pSessionData->sessState) &&
       (contextBufferSize >= DC_INFLATE_CONTEXT_SIZE))
    {
        /* Search for the location of bank E */
        status = findBankE(pSessionDesc);
        pSessionDesc->bankEAvailable = CPA_TRUE;
    }
    return status;
}

CpaStatus
cpaDcInitSession(CpaInstanceHandle dcInstance,
        CpaDcSessionHandle pSessionHandle,
        CpaDcSessionSetupData* pSessionData,
        CpaBufferList *pContextBuffer,
        CpaDcCallbackFn callbackFn)
{
    CpaInstanceHandle insHandle = NULL;
    sal_compression_service_t * pService = NULL;

#ifdef ICP_TRACE
    LAC_LOG5("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)dcInstance,
            (LAC_ARCH_UINT)pSessionHandle,
            (LAC_ARCH_UINT)pSessionData,
            (LAC_ARCH_UINT)pContextBuffer,
            (LAC_ARCH_UINT)callbackFn);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == dcInstance)
    {
        insHandle = dcGetFirstHandle();
    }
    else
    {
        insHandle = dcInstance;
    }

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_INSTANCE_HANDLE(insHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(insHandle);
    SAL_CHECK_INSTANCE_TYPE(insHandle, SAL_SERVICE_TYPE_COMPRESSION);
#endif

    pService = (sal_compression_service_t*) insHandle;

    /* Check if SAL is initialised otherwise return an error */
    SAL_RUNNING_CHECK(pService);

    return dcInitSession(insHandle, pSessionHandle, pSessionData,
               pContextBuffer, callbackFn);
}

CpaStatus
cpaDcResetSession(const CpaInstanceHandle dcInstance,
        CpaDcSessionHandle pSessionHandle)
{
#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
              (LAC_ARCH_UINT)dcInstance,
              (LAC_ARCH_UINT)pSessionHandle);
#endif
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus
cpaDcRemoveSession(const CpaInstanceHandle dcInstance,
        CpaDcSessionHandle pSessionHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle insHandle = NULL;
    dc_session_desc_t* pSessionDesc = NULL;
    Cpa64U numPendingStateless = 0;
    Cpa64U numPendingStateful = 0;
    icp_comms_trans_handle trans_handle = NULL;

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(pSessionHandle);
#endif
    pSessionDesc = DC_SESSION_DESC_FROM_CTX_GET(pSessionHandle);
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(pSessionDesc);
#endif

#ifdef ICP_TRACE
    LAC_LOG2("Called with params (0x%lx, 0x%lx)\n",
            (LAC_ARCH_UINT)dcInstance,
            (LAC_ARCH_UINT)pSessionHandle);
#endif

    if(CPA_TRUE == pSessionDesc->isDcDp)
    {
        insHandle = dcInstance;
    }
    else
    {
        if(CPA_INSTANCE_HANDLE_SINGLE == dcInstance)
        {
            insHandle = dcGetFirstHandle();
        }
        else
        {
            insHandle = dcInstance;
        }
    }

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(insHandle);
    SAL_CHECK_INSTANCE_TYPE(insHandle, SAL_SERVICE_TYPE_COMPRESSION);
#endif

    /* Check if SAL is running otherwise return an error */
    SAL_RUNNING_CHECK(insHandle);

    if(CPA_TRUE == pSessionDesc->isDcDp)
    {
        trans_handle = ((sal_compression_service_t*)
            dcInstance)->trans_handle_compression_tx;

        if (CPA_TRUE == icp_adf_queueDataToSend(trans_handle))
        {
           /* Process the remaining messages on the ring */
            icp_adf_updateQueueTail(trans_handle);
            LAC_LOG_ERROR("There are remaining messages on the ring");
            return CPA_STATUS_RETRY;
         }

        /* Check if there are stateless pending requests */
        if(0 != pSessionDesc->pendingDpStatelessCbCount)
        {
            LAC_LOG_ERROR1("There are %d stateless DP requests pending",
                pSessionDesc->pendingDpStatelessCbCount);
            return CPA_STATUS_RETRY;
        }
    }
    else
    {
        numPendingStateless =
            osalAtomicGet(&(pSessionDesc->pendingStatelessCbCount));
        numPendingStateful =
            osalAtomicGet(&(pSessionDesc->pendingStatefulCbCount));

        /* Check if there are stateless pending requests */
        if(0 != numPendingStateless)
        {
            LAC_LOG_ERROR1("There are %d stateless requests pending",
                numPendingStateless);
            status = CPA_STATUS_RETRY;
        }

        /* Check if there are stateful pending requests */
        if(0 != numPendingStateful)
        {
            LAC_LOG_ERROR1("There are %d stateful requests pending",
                numPendingStateful);
            status = CPA_STATUS_RETRY;
        }

        /* Only destroy the spinlock when we don't get RETRY error. */
        if((CPA_DC_STATEFUL == pSessionDesc->sessState) &&
           (CPA_STATUS_SUCCESS == status))
        {
            LAC_SPINLOCK_DESTROY(&(pSessionDesc->sessionLock));
        }
    }

    return status;
}

CpaStatus
dcGetSessionSize(CpaInstanceHandle dcInstance,
        CpaDcSessionSetupData* pSessionData,
        Cpa32U* pSessionSize,
        Cpa32U* pContextSize)
{
#ifdef ICP_PARAM_CHECK
    CpaInstanceHandle insHandle = NULL;

    if(CPA_INSTANCE_HANDLE_SINGLE == dcInstance)
    {
        insHandle = dcGetFirstHandle();
    }
    else
    {
        insHandle = dcInstance;
    }

    /* Check parameters */
    LAC_CHECK_NULL_PARAM(insHandle);
    LAC_CHECK_NULL_PARAM(pSessionData);
    LAC_CHECK_NULL_PARAM(pSessionSize);

    if(CPA_DC_L1 == pSessionData->compLevel)
    {
        pSessionData->deflateWindowSize = DC_32K_WINDOW_SIZE;
    }
    else if(CPA_DC_L1 < pSessionData->compLevel)
    {
         pSessionData->deflateWindowSize = DC_8K_WINDOW_SIZE;
    }

    if(dcCheckSessionData(pSessionData) != CPA_STATUS_SUCCESS)
    {
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    /* Get session size for session data */
    *pSessionSize = sizeof(dc_session_desc_t) + LAC_64BYTE_ALIGNMENT
                    + sizeof(LAC_ARCH_UINT);

    if(NULL != pContextSize)
    {
        dcGetContextSize(pSessionData, pContextSize, NULL);
    }

    return CPA_STATUS_SUCCESS;
}

CpaStatus
cpaDcGetSessionSize(CpaInstanceHandle dcInstance,
        CpaDcSessionSetupData* pSessionData,
        Cpa32U* pSessionSize,
        Cpa32U* pContextSize)
{
    /* Check parameter */
#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(pContextSize);
#endif

    return dcGetSessionSize(dcInstance, pSessionData, pSessionSize,
            pContextSize);

#ifdef ICP_TRACE
    LAC_LOG6("Called with params (0x%lx, 0x%lx, 0x%lx[%d], 0x%lx[%d])\n",
            (LAC_ARCH_UINT)dcInstance,
            (LAC_ARCH_UINT)pSessionData,
            (LAC_ARCH_UINT)pSessionSize,
            *pSessionSize,
            (LAC_ARCH_UINT)pContextSize,
            *pContextSize);
#endif
}
