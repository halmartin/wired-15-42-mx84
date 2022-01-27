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
 * @file dc_datapath.c
 *
 * @defgroup Dc_DataCompression DC Data Compression
 *
 * @ingroup Dc_DataCompression
 *
 * @description
 *      Implementation of the Data Compression datapath operations.
 *
 *****************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include "cpa.h"
#include "cpa_dc.h"
#include "cpa_dc_dp.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_qat_fw_comp.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "dc_session.h"
#include "dc_datapath.h"
#include "sal_statistics.h"
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_log.h"
#include "sal_types_compression.h"
#include "dc_stats.h"
#include "lac_buffer_desc.h"
#include "lac_sal.h"
#include "lac_sync.h"
#include "sal_service_state.h"
#include "sal_qat_cmn_msg.h"
#include "dc_header_footer.h"
#include "dc_flattree.h"


/* Bit definition */
#define BIT_0                       (0x01)
#define BIT_1                       (0x02)
#define BIT_2                       (0x04)
#define BIT_3                       (0x08)
#define BIT_4                       (0x10)
#define BIT_5                       (0x20)
#define BIT_6                       (0x40)
#define BIT_7                       (0x80)

#define BYTE_SIZE                   (8)
#define RESIDUE_COUNT               (4)

/* Indexes in state registers */
#define DECOMP_ERROR_STATUS_INDEX   (4)
#define DECOMP_SLICE_STATUS_INDEX   (5)
#define IN_BYTE_COUNT_INDEX         (8)
#define IN_RESIDUE_INDEX            (24)
#define IN_RESIDUE_PTR_INDEX        (28)
#define NB_RESIDUE_VALID_BITS       (4)

#define CD_CRC32_CHECKSUM_OFFSET    (24)
#define CD_ADLER32_CHECKSUM_OFFSET  (28)

/* List of parameters required to update the state registers */
typedef struct register_update_params_s
{
    Cpa8U *decompStateRegisters;
    Cpa64U headerLengthInBits;
    Cpa8U residueBytes;
    Cpa8U residueBits;
    CpaBufferList *sourceBuffer;
    Cpa32U *consumedData;
} register_update_params_t;


/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Update content descriptor checksum field both for Adler 32 and CRC 32
 *
 * @description
 *      Update content descriptor checksum field both for Adler 32 and CRC 32
 *
 * @param[in]     newValue            New checksum value to be uploaded
 * @param[in/out] ctDesc              Pointer to the checksum in
 *                                    the content descriptor
 *
 *****************************************************************************/
STATIC void
dcUpdate_ContentDescriptor(Cpa32U newValue, Cpa8U *ctDesc)
{
    *(ctDesc + 3) = (newValue >> 24) & 0xFF;
    *(ctDesc + 2) = (newValue >> 16) & 0xFF;
    *(ctDesc + 1) = (newValue >>  8) & 0xFF;
    *ctDesc = newValue & 0xFF;
}

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Get the post pended adler32 checksum from uncompressed data
 *
 * @description
 *      Get the post pended adler32 checksum from uncompressed data
 *      for statelesss session
 *
 * @param[in]   consumed              Octets consumed by the operation
 * @param[out]  pPostPendedAdler32    Pointer to the post-pended adler32
 * @param[in]   pSrcBuff              Pointer to buffer of compressed data
 *
 * @retval CPA_STATUS_SUCCESS       Function executed successfully
 * @retval CPA_STATUS_FAIL          Function failed
 *
 *****************************************************************************/
STATIC CpaStatus
dcGet_PostPendedAdler32(Cpa32U consumed,
                        Cpa32U * pPostPendedAdler32,
                        CpaBufferList *pSrcBuff)
{
    CpaFlatBuffer *pEndBuf = NULL;
    Cpa32U numBuffers = 0;
    Cpa32U dataLen = 0;
    Cpa32U index = 0;
    Cpa32U leftShiftBits = 0;
    Cpa8U * pData = NULL;

    if(consumed <= DC_ZLIB_FOOTER_SIZE)
    { /* no post-pended adler32 */
        return CPA_STATUS_FAIL;
    }
#ifdef ICP_PARAM_CHECK
    LAC_ASSERT_NOT_NULL(pPostPendedAdler32);
    LAC_ASSERT_NOT_NULL(pSrcBuff);
#endif

    pEndBuf = pSrcBuff->pBuffers;
    numBuffers = pSrcBuff->numBuffers;

    LAC_LOG_DEBUG1("numBuffers = %d\n", numBuffers);
    LAC_LOG_DEBUG1("consumed = %d\n", consumed);
    LAC_LOG_DEBUG1("pEndBuf->dataLenInBytes = %d\n", pEndBuf->dataLenInBytes);
    /* Find the last flat buffer pEndBuf in buffer list pSrcBuff,
     * and get the data length in bytes of the data
     * in the last flat buffer */
    while ((0 != numBuffers) && (0 != consumed)
           && (0 != pEndBuf->dataLenInBytes))
    {
        if(consumed <= pEndBuf->dataLenInBytes)
        {
            dataLen = consumed;
        }
        else
        {
            dataLen = pEndBuf->dataLenInBytes;
            pEndBuf ++;
            numBuffers --;
        }
        consumed -= dataLen;
    }
    if(((0 == numBuffers) && (0 != consumed))
       || ((0 != consumed) && (0 == pEndBuf->dataLenInBytes)))
    {
        return CPA_STATUS_FAIL;
    }
    LAC_LOG_DEBUG1("numBuffers = %d\n", numBuffers);
    LAC_LOG_DEBUG1("consumed = %d\n", consumed);
    LAC_LOG_DEBUG1("dataLen = %d\n", dataLen);
    if(dataLen < DC_ZLIB_FOOTER_SIZE)
    {
        *pPostPendedAdler32 = 0;
        pData = pEndBuf->pData;
        index = dataLen;
        while(index)
        {
            leftShiftBits = ((dataLen - index) *
                              LAC_NUM_BITS_IN_BYTE);
            LAC_LOG_DEBUG1("left shift %d bytes\n",
                             (dataLen - index));
            index --;
            /* save to pPostPendedAdler32 in little endian order */
            *pPostPendedAdler32 += ((Cpa32U)(*(pData + index)) & 0xff ) <<
                                   leftShiftBits;
            LAC_LOG_DEBUG1("index = %d\n", index);
        }
        pData = (pEndBuf-1)->pData +
                (pEndBuf-1)->dataLenInBytes -
                (DC_ZLIB_FOOTER_SIZE - dataLen);
        index = DC_ZLIB_FOOTER_SIZE - dataLen;
        while(index)
        {
            leftShiftBits = ((DC_ZLIB_FOOTER_SIZE - index) * LAC_NUM_BITS_IN_BYTE);
            LAC_LOG_DEBUG1("left shift %d bytes\n", DC_ZLIB_FOOTER_SIZE - index);
            index --;
            /* save to pPostPendedAdler32 in little endian order */
            *pPostPendedAdler32 += ((Cpa32U)(*(pData + index)) & 0xff ) <<
                                   leftShiftBits;
            LAC_LOG_DEBUG1("index = %d\n", index);
        }
    }
    else
    {
        pData = (Cpa8U *)pPostPendedAdler32;
        pData[0] = *(pEndBuf->pData +
                     dataLen - DC_ZLIB_FOOTER_SIZE + 3);
        pData[1] = *(pEndBuf->pData +
                     dataLen - DC_ZLIB_FOOTER_SIZE + 2);
        pData[2] = *(pEndBuf->pData +
                     dataLen - DC_ZLIB_FOOTER_SIZE + 1);
        pData[3] = *(pEndBuf->pData +
                     dataLen - DC_ZLIB_FOOTER_SIZE + 0);
    }
    LAC_LOG_DEBUG1("*pPostPendedAdler32 = 0x%x\n", *pPostPendedAdler32);
    return CPA_STATUS_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Update Adler32 Checksum for decompression
 *
 * @description
 *      Update Adler32 Checksum for decompression
 *
 * @param[in]     pSessionDesc        Pointer to the session descriptor
 * @param[in/out] pAdler32            Pointer to the adler32 checksum
 * @param[in]     pSrcBuff            Pointer to data buffer of decompressed
 * @param[in]     pDestBuff           Pointer to buffer space for data after
 * @param[in]     pSrcBuff            Pointer to data buffer for decompression
 * @param[in]     pDestBuff           Pointer to buffer space for data after
 *                                    decompression
 * @param[in]     flushFlag           Indicates the type of flush to be
 *                                    performed
 * @param[in]     produced            Octets produced by the operation
 * @param[in]     consumed            Octets consumed by the operation
 *
 *****************************************************************************/
STATIC void
dcUpdate_Checksum(dc_session_desc_t* pSessionDesc,
                  Cpa32U *pAdler32,
                  CpaBufferList *pSrcBuff,
                  CpaBufferList *pDestBuff,
                  CpaDcFlush flushFlag,
                  Cpa32U produced,
                  Cpa32U consumed)
{
    CpaFlatBuffer *pFlatBuf = NULL;
    Cpa32U numBuffers = 0;
    Cpa32U bufSizeInBytes = 0;
    Cpa32U dataLen = 0;
    Cpa32U postPendedAdler32 = 0;
    Cpa8U *pData = NULL;
    Cpa32U isUpdate = 1;
    Cpa32U tempAdler32 = 0;
    Cpa32U tempPreviousAdler32 = 0;

#ifdef ICP_PARAM_CHECK
    LAC_ASSERT_NOT_NULL(pSessionDesc);
    LAC_ASSERT_NOT_NULL(pAdler32);
    LAC_ASSERT_NOT_NULL(pSrcBuff);
    LAC_ASSERT_NOT_NULL(pDestBuff);
#endif
    if((0 == consumed) || (0 == produced)
       || (0 == pSrcBuff->numBuffers)
       || (0 == pDestBuff->numBuffers))
    {
        return;
    }
    pFlatBuf = pDestBuff->pBuffers;
    numBuffers = pDestBuff->numBuffers;
    bufSizeInBytes = pFlatBuf->dataLenInBytes;
    tempPreviousAdler32 = pSessionDesc->previousChecksum;

    if(CPA_DC_STATELESS == pSessionDesc->sessState)
    {
        CpaStatus status = CPA_STATUS_SUCCESS;
        status = dcGet_PostPendedAdler32(consumed,
                                         &postPendedAdler32,
                                         pSrcBuff);

        if((CPA_STATUS_SUCCESS == status)
            && (((*pAdler32 & 0xffff0000)
                            == (postPendedAdler32 & 0xffff0000))
                || ((*pAdler32 & 0xffff)
                            != (postPendedAdler32 & 0xffff))))
        {
            /* For stateless session, not update Adler32
             * if the upper 16-bit post-pended Adler32
             * is the same as the upper 16-bit Adler32
             * calculated by the slice;
             * Or if the lower 16-bit post-pended Adler32
             * is the diff from the lower 16-bit Adler32
             * calculated by the slice. */
            isUpdate = 0;
        }
    }
    LAC_LOG_DEBUG1("numBuffers = %d\n", numBuffers);
    LAC_LOG_DEBUG1("bufSizeInBytes = %d\n", bufSizeInBytes);
    LAC_LOG_DEBUG1("consumed = %d\n", consumed);
    LAC_LOG_DEBUG1("produced = %d\n", produced);
    while (isUpdate && (0 != numBuffers) && (0 != produced)
           && (0 != bufSizeInBytes))
    {
        pData = pFlatBuf->pData;
        if(produced < bufSizeInBytes)
        {
            dataLen = produced;
        }
        else
        {
            dataLen = bufSizeInBytes;
            pFlatBuf++;
            numBuffers--;
            bufSizeInBytes = pFlatBuf->dataLenInBytes;
        }
        produced -= dataLen;
        LAC_LOG_DEBUG1("tempPreviousAdler32 = 0x%x\n",
                                tempPreviousAdler32);
        /* calculate adler32 checksum for
         * each byte in pDestBuff->pBuffers->pData */
        tempAdler32 = tempPreviousAdler32;
        osalAdler32(&tempAdler32, pData, dataLen);

        /* Save the checksum for the next request */
        tempPreviousAdler32 = tempAdler32;
        LAC_LOG_DEBUG1("numBuffers = %d\n", numBuffers);
        LAC_LOG_DEBUG1("bufSizeInBytes = %d\n", bufSizeInBytes);
        LAC_LOG_DEBUG1("consumed = %d\n", consumed);
        LAC_LOG_DEBUG1("produced = %d\n", produced);
        LAC_LOG_DEBUG1("dataLen = %d\n", dataLen);
    }
    if((!isUpdate) || ((0 == numBuffers) && (0 != produced))
       || ((0 != produced) && (0 == bufSizeInBytes)))
    { /* not update adler32 */
        return;
    }
    *pAdler32 = tempAdler32;
    pSessionDesc->previousChecksum = tempPreviousAdler32;
    LAC_LOG_DEBUG1("after update: checksum = 0x%x\n", *pAdler32);
}

STATIC Cpa8U
getResidueByteOffset(Cpa8U *stateRegisters, Cpa8U *bitOffset, Cpa8U *residueBuffer)
{
    Cpa8U byteOffset = 0;
    Cpa8U residueValue = 0;
    Cpa8U i = 0;

    /* Determine byte offset to beginning of header */
    residueValue = *(stateRegisters + IN_RESIDUE_PTR_INDEX);

    /* Determine bit offset to beginning of header if we're not byte aligned */
    *bitOffset = residueValue >> 4;

    for (i=0; i<NB_RESIDUE_VALID_BITS; i++)
    {
        if(residueValue & 0x1)
        {
            byteOffset++;
            residueValue >>= 1;
        }
    }
    /* Copy residues */
    osalMemCopy(residueBuffer, stateRegisters + IN_RESIDUE_INDEX, byteOffset);
    return (byteOffset);
}

STATIC CpaStatus
updateStateRegisters (register_update_params_t *regsParams)
{
    int i = 0, j = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U residueValidBits = 0;
    Cpa32U inputByteCount = 0;
    Cpa32U originalHeaderSizeInBytes = 0;
    Cpa32U headerSizeInBytes = 0;
    Cpa64U registerValue = 0;
    Cpa8U numberOfResidues = 0;
    Cpa8U residue[RESIDUE_COUNT] = {0, 0, 0, 0};
    Cpa32U dataIdx = 0;
    Cpa32U buffIdx = 0;
    Cpa32U bufferOffset=0;
    sgl_buffer_index_t sglIndex = {0};
    Cpa32U consumedDataCopy = *(regsParams->consumedData);
    Cpa32U consumedData = consumedDataCopy;
    Cpa8U * const stateRegisters = regsParams->decompStateRegisters;
    CpaBufferList * const srcBuffer = regsParams->sourceBuffer;

    /* Retrieve input byte count */
    for (i=3; i>=0; i--)
    {
        registerValue <<= 8;
        registerValue  |= *(stateRegisters + IN_BYTE_COUNT_INDEX + i);
    }
    inputByteCount = registerValue;
    inputByteCount -= regsParams->residueBytes;

    if (consumedDataCopy >= regsParams->residueBytes)
    {
        consumedDataCopy -= regsParams->residueBytes;
    }
    headerSizeInBytes = (regsParams->headerLengthInBits / BYTE_SIZE);

    /* Check if we are byte aligned */
    regsParams->residueBits = regsParams->headerLengthInBits % BYTE_SIZE;
    if (headerSizeInBytes & 0x1)
    {
        headerSizeInBytes--;
        regsParams->residueBits += 8;
    }
    originalHeaderSizeInBytes = headerSizeInBytes;

    /* If there are residue bits then increase the
     * byte counter by 2
     */
    if (regsParams->residueBits)
    {
        headerSizeInBytes += 2;
    }
    numberOfResidues = headerSizeInBytes - originalHeaderSizeInBytes;
    if (originalHeaderSizeInBytes & 0x1)
    {
        numberOfResidues = 4;
    }

    /*
     * Update input residue register
     */

    /* Find the data in the source at the index defined by the header length
     * But first find the right offset in the SGL. This offset is found with
     * "inputByteCount".
     */
    if (consumedData >= regsParams->residueBytes)
    {
        bufferOffset = consumedDataCopy + originalHeaderSizeInBytes;
    }
    else
    {
        bufferOffset = originalHeaderSizeInBytes - regsParams->residueBytes;
    }
    status = findIndexInSglBuffer (srcBuffer, &sglIndex, bufferOffset);
    if (CPA_STATUS_FAIL == status)
    {
        LAC_LOG_ERROR("Could not find header in source buffer.\n");
        return CPA_STATUS_FAIL;
    }
    buffIdx = sglIndex.bufferIndex;
    dataIdx = sglIndex.OffsetInBuffer;

    residueValidBits = 0;
    for (i=0, j=0; i<numberOfResidues; i++, j++)
    {
        /* Update residue valid bits */
        residueValidBits <<= 1;
        residueValidBits  |= 1;

        /* Check if we are at the last byte of the current buffer */
        if ((dataIdx + j) >= srcBuffer->pBuffers[buffIdx].dataLenInBytes)
        {
            buffIdx++;
            dataIdx = 0;
            j = 0;
        }
        residue[i] = srcBuffer->pBuffers[buffIdx].pData[dataIdx+j];
    }

    /*
     * Update input byte counter with header
     * size added to new byte count
     */
    if (consumedData >= regsParams->residueBytes)
    {
        *(regsParams->consumedData) =
                consumedDataCopy + headerSizeInBytes;
    }
    else
    {
        *(regsParams->consumedData) =
                headerSizeInBytes - regsParams->residueBytes;
    }
    registerValue = inputByteCount + headerSizeInBytes;

    /* Restore new byte count */
    for (i=0; i<3; i++)
    {
        *(stateRegisters + IN_BYTE_COUNT_INDEX + i) = registerValue & 0xFF;
        registerValue >>= 8;
    }

    /*
     * Set bit<49> in state register 0 to indicate
     * to the slice that bank E has been loaded.
     * Bit<49> is in byte 5.
     */
    *(stateRegisters + DECOMP_SLICE_STATUS_INDEX) |= BIT_1;
    /* Clear error (-7) */
    *(stateRegisters + DECOMP_ERROR_STATUS_INDEX) = 0x00;

    /*
     * Write back residue bit field in State register 3.
     */
    for (i=0; i<RESIDUE_COUNT; i++)
    {
        *(stateRegisters + IN_RESIDUE_INDEX + i) = residue[i];
    }

    /* Update residue pointer field */
    *(stateRegisters + IN_RESIDUE_PTR_INDEX ) &= 0x0F;
    *(stateRegisters + IN_RESIDUE_PTR_INDEX ) |= (regsParams->residueBits<<4);
    /* Update residue valid field */
    *(stateRegisters + IN_RESIDUE_PTR_INDEX ) &= 0xF0;
    *(stateRegisters + IN_RESIDUE_PTR_INDEX ) |= residueValidBits;
    return CPA_STATUS_SUCCESS;
}

void
dcCompression_ProcessCallback(icp_comms_trans_handle transHandle,
                              void *pRespMsg)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_qat_fw_comp_resp_t *pCompRespMsg = NULL;
    void *callbackTag = NULL;
    Cpa64U *pReqData = NULL;
    CpaDcDpOpData *pResponse = NULL;
    CpaDcRqResults *pResults = NULL;
    CpaDcCallbackFn pCbFunc = NULL;
    dc_session_desc_t *pSessionDesc = NULL;
    osalz_stream stream = {0};
    sal_compression_service_t* pService = NULL;
    dc_compression_cookie_t* pCookie = NULL;
    CpaBoolean cmpPass = CPA_TRUE, xlatPass = CPA_TRUE;
#ifndef ICP_DC_DYN_NOT_SUPPORTED
    Cpa8U cmpErr = ERR_CODE_NO_ERROR, xlatErr = ERR_CODE_NO_ERROR;
#else
    Cpa8U cmpErr = ERR_CODE_NO_ERROR;
#endif
    dc_request_dir_t compDecomp = DC_COMPRESSION_REQUEST;
    Cpa8U opStatus = ICP_QAT_FW_COMN_STATUS_FLAG_OK;
    Cpa32S bufferOffset = 0;
    Cpa32U srcSize = 0;
    state_t s = {0};
    Cpa8U *srcPtr = NULL;
    Cpa8U residueByteOffset = 0;
    Cpa8U residueBitOffset = 0;
    sgl_buffer_index_t sglIdx = {0};
    Cpa8U residues[RESIDUE_COUNT];
    register_update_params_t register_modify_params;
    Cpa8U* decompStatetRegisters = NULL;

    /* Cast response message to compression response message type */
    pCompRespMsg = (icp_qat_fw_comp_resp_t *)pRespMsg;
#ifdef ICP_PARAM_CHECK
    LAC_ASSERT_NOT_NULL(pCompRespMsg);
#endif

    /* Extract request data pointer from the opaque data */
    LAC_MEM_SHARED_READ_TO_PTR(pCompRespMsg->comn_resp.opaque_data, pReqData);
#ifdef ICP_PARAM_CHECK
    LAC_ASSERT_NOT_NULL(pReqData);
#endif

    /* Extract fields from the request data structure */
    pCookie = (dc_compression_cookie_t*) pReqData;
#ifdef ICP_PARAM_CHECK
    LAC_ASSERT_NOT_NULL(pCookie);
#endif

    pSessionDesc = DC_SESSION_DESC_FROM_CTX_GET(pCookie->pSessionHandle);
    decompStatetRegisters = &pSessionDesc->stateRegistersDecomp[0];

    if(CPA_TRUE == pSessionDesc->isDcDp)
    {
        pResponse = (CpaDcDpOpData *) pReqData;
        pResults = &(pResponse->results);
    }
    else
    {
        pSessionDesc = pCookie->pSessionDesc;
        pResults = pCookie->pResults;
        callbackTag = pCookie->callbackTag;
        pCbFunc = pCookie->pSessionDesc->pCompressionCb;
        compDecomp = pCookie->compDecomp;
    }

    pService = (sal_compression_service_t*) (pCookie->dcInstance);
    opStatus = pCompRespMsg->comn_resp.comn_status;

    /* Check compression response status */
    cmpPass = (CpaBoolean)(ICP_QAT_FW_COMN_STATUS_FLAG_OK ==
            ICP_QAT_FW_COMN_RESP_CMP_STAT_GET(opStatus));

    /* Get the cmp error code */
    cmpErr = pCompRespMsg->comn_resp.comn_error.s1.cmp_err_code;

    if ((Cpa8U)ERR_CODE_FATAL_ERROR == cmpErr)
    {
        if (CPA_FALSE == pSessionDesc->parityErrorEnabled)
        {
            cmpErr = ERR_CODE_NO_ERROR;
        }
    }

    /* We return the compression error code for now. We would need to update
     * the API if we decide to return both error codes */
    pResults->status = (Cpa8S)cmpErr;

#ifndef ICP_DC_DYN_NOT_SUPPORTED
    /* Check the translator status */
    if((DC_COMPRESSION_REQUEST == compDecomp)
     &&(CPA_DC_HT_FULL_DYNAMIC == pSessionDesc->huffType))
    {
        /* Check translator response status */
        xlatPass = (CpaBoolean)(ICP_QAT_FW_COMN_STATUS_FLAG_OK ==
                ICP_QAT_FW_COMN_RESP_XLAT_STAT_GET(opStatus));

        /* Get the translator error code */
        xlatErr = pCompRespMsg->comn_resp.comn_error.s1.xlat_err_code;

        /* Return a fatal error or a potential error in the translator slice
         * if the compression slice did not return any error */
        if((CPA_DC_OK == pResults->status) ||
           (CPA_DC_FATALERR == (Cpa8S)xlatErr))
        {
            pResults->status = (Cpa8S)xlatErr;
        }
    }
#endif

    if (CPA_FALSE == pSessionDesc->isDcDp)
    {
        /* In case of any error for an end of packet request, we need to update
         * the request type for the following request */
         if((CPA_DC_STATEFUL == pSessionDesc->sessState) &&
           ((CPA_DC_FLUSH_FINAL == pCookie->flushFlag) ||
           (CPA_DC_FLUSH_FULL == pCookie->flushFlag)) &&
           ((CPA_TRUE != cmpPass)||(CPA_TRUE != xlatPass)))
        {
            pSessionDesc->requestType = DC_REQUEST_SUBSEQUENT;
        }
        else if((CPA_DC_STATELESS == pSessionDesc->sessState) &&
               ((CPA_DC_FLUSH_FINAL == pCookie->flushFlag)))
        {
            pSessionDesc->requestType = DC_REQUEST_FIRST;
        }
        else if((CPA_DC_STATELESS == pSessionDesc->sessState) &&
               ((CPA_DC_FLUSH_FULL == pCookie->flushFlag)))
        {
            pSessionDesc->requestType = DC_REQUEST_SUBSEQUENT;
        }
    }

    /* Stateful overflow is a valid use case. The previous
     * update on the request type is still required */
    if((CPA_DC_STATEFUL == pSessionDesc->sessState)
     &&(CPA_DC_OVERFLOW == pResults->status))
    {
        cmpPass = CPA_TRUE;

        if(DC_COMPRESSION_REQUEST == compDecomp)
        {
            LAC_LOG("Recoverable error: stateful compression overflow. You may "
                "need to increase the size of your destination buffer and "
                "resubmit this request");
        }
    }

    if(((CPA_TRUE == cmpPass)&&(CPA_TRUE == xlatPass))||
       ((CPA_DC_STATEFUL == pSessionDesc->sessState)&&
        (CPA_DC_BAD_LITLEN_CODES == pResults->status)&&
        (DC_DECOMPRESSION_REQUEST == compDecomp)))
    {
        /* Extract the response from the firmware */
        pResults->consumed = pCompRespMsg->comn_resp_params.input_byte_counter;
        pResults->produced = pCompRespMsg->comn_resp_params.output_byte_counter;

        if(CPA_DC_STATEFUL == pSessionDesc->sessState)
        {
            pSessionDesc->cumulativeConsumedBytes += pResults->consumed;
        }
        else
        {
            /* In the stateless case all requests have both SOP and EOP set */
            pSessionDesc->cumulativeConsumedBytes = pResults->consumed;
        }

        if(CPA_DC_CRC32 == pSessionDesc->checksumType)
        {
            pResults->checksum = pCompRespMsg->comn_resp_params.current_crc32;
        }
        else if(CPA_DC_ADLER32 == pSessionDesc->checksumType)
        {
            pResults->checksum = pCompRespMsg->comn_resp_params.current_adler;
        }

        LAC_LOG_DEBUG1("checksum = 0x%x\n", pResults->checksum);
        if((CPA_TRUE != pSessionDesc->isDcDp)
            && (CPA_DC_ADLER32 == pSessionDesc->checksumType)
            && (DC_DECOMPRESSION_REQUEST == compDecomp)
            && (NULL != pCookie->pSourceBuffer)
            && (NULL != pCookie->pDestinationBuffer))
        {
            dcUpdate_Checksum(pSessionDesc,
                              &pResults->checksum,
                              pCookie->pSourceBuffer,
                              pCookie->pDestinationBuffer,
                              pCookie->flushFlag,
                              pResults->produced,
                              pResults->consumed);
        }
        /* Save the checksum for the next request */
        pSessionDesc->previousChecksum = pResults->checksum;


        if(CPA_TRUE == pSessionDesc->isDcDp)
        {
            pResponse->responseStatus = CPA_STATUS_SUCCESS;
        }
        else
        {
            if(DC_COMPRESSION_REQUEST == compDecomp)
            {
                COMPRESSION_STAT_INC(numCompCompleted, pService);
            }
            else
            {
                COMPRESSION_STAT_INC(numDecompCompleted, pService);
            }
        }
    }
    else
    {
        if ((CPA_DC_BAD_LITLEN_CODES == pResults->status) &&
            (CPA_DC_STATELESS == pSessionDesc->sessState) &&
            (CPA_DC_DIR_DECOMPRESS == pSessionDesc->sessDirection))
        {
            /* Decompress data using ZLIB inflate */
            status = osalZlibInflate((void*)pCookie->pSourceBuffer,
                                     (void*)pCookie->pDestinationBuffer,
                                     &pResults->produced,
                                     pSessionDesc->checksumType,
                                     &pResults->checksum,
                                     &stream);
            if(CPA_STATUS_SUCCESS == status)
            {
                pResults->status = CPA_DC_OK;
                pResults->consumed = stream.total_in;
            }
            else if (CPA_DC_OVERFLOW == status)
            {
                pResults->status = CPA_DC_OVERFLOW;
                pResults->checksum = 0;
                pResults->consumed = 0;
                pResults->produced = 0;
            }
            else
            {
                pResults->status = CPA_DC_BAD_LITLEN_CODES;
                pResults->checksum = 0;
                pResults->consumed = 0;
                pResults->produced = 0;
            }
        }
        else
        {
            LAC_LOG_ERROR1("Unexpected response status = %d",
                pResults->status);
        }

        if(CPA_DC_OVERFLOW == pResults->status &&
           CPA_DC_STATELESS == pSessionDesc->sessState)
        {
            LAC_LOG_ERROR("Unrecoverable error: stateless overflow. You may "
                "need to increase the size of your destination buffer");
        }

        if(CPA_TRUE == pSessionDesc->isDcDp)
        {
            pResponse->responseStatus = CPA_STATUS_FAIL;
        }
        else
        {
            if(CPA_DC_OK != pResults->status)
            {
                status = CPA_STATUS_FAIL;
            }

            if(DC_COMPRESSION_REQUEST == compDecomp)
            {
                COMPRESSION_STAT_INC(numCompCompletedErrors, pService);
            }
            else
            {
                COMPRESSION_STAT_INC(numDecompCompletedErrors, pService);
            }
        }
    }

#ifdef ICP_TRACE
    LAC_LOG4("status: %d, consumed: %d, produced: %d, checksum: 0x%x\n",
            pResults->status,
            pResults->consumed,
            pResults->produced,
            pResults->checksum);
#endif

    /* Stateful decompression using flat tree construction */
    if((CPA_DC_STATEFUL == pSessionDesc->sessState)&&
       (CPA_DC_BAD_LITLEN_CODES == pResults->status)&&
       (DC_DECOMPRESSION_REQUEST == compDecomp)&&
       (CPA_TRUE == pSessionDesc->bankEAvailable))
    {
        srcSize = pCookie->srcTotalDataLenInBytes - pResults->consumed;
        residueByteOffset = getResidueByteOffset(decompStatetRegisters,
                &residueBitOffset,
                residues);

        register_modify_params.residueBytes = residueByteOffset;
        register_modify_params.residueBits = residueBitOffset;

        if (pResults->consumed >= residueByteOffset)
        {
            /* Residues are inside the packet */
            bufferOffset = pResults->consumed - residueByteOffset;
            status = findIndexInSglBuffer (pCookie->pSourceBuffer,
                    &sglIdx, bufferOffset);

            srcPtr = pCookie->pSourceBuffer->pBuffers[sglIdx.bufferIndex].pData;
            srcPtr += sglIdx.OffsetInBuffer;
        }
        else
        {
            /* Residues are in the state registers */
            srcPtr = pCookie->pSourceBuffer->pBuffers[0].pData;
        }

        if (CPA_STATUS_SUCCESS == status)
        {
            osalMemSet (&s, 0, sizeof(state_t));
            if ((residueByteOffset > pResults->consumed) && residueByteOffset)
            {
                s.in = residues;
                s.inlen = residueByteOffset;
                /* Decode dynamic block and construct Bank E */
                status = decodeDynamicBlock (&s, residueBitOffset,
                        srcPtr, srcSize,
                        pCookie->pSessionDesc->pContextBuffer,
                        pCookie->pSessionDesc->bankEStartIndex.bufferIndex,
                        pCookie->pSessionDesc->bankEStartIndex.OffsetInBuffer);
            }
            else
            {
                s.in = srcPtr;
                s.inlen = srcSize;
                /* Decode dynamic block and construct Bank E */
                status = decodeDynamicBlock (&s, residueBitOffset,
                        NULL, 0,
                        pCookie->pSessionDesc->pContextBuffer,
                        pCookie->pSessionDesc->bankEStartIndex.bufferIndex,
                        pCookie->pSessionDesc->bankEStartIndex.OffsetInBuffer);
            }
        }

        if (CPA_STATUS_FAIL == status)
        {
            osalAtomicDec(&(pCookie->pSessionDesc->pendingStatefulCbCount));
            LAC_LOG_ERROR("Error in flat tree construction.");

            /* Free the memory pool */
            if (NULL != pCookie)
            {
                Lac_MemPoolEntryFree(pCookie);
                pCookie = NULL;
            }

            if(NULL != pCbFunc)
            {
                pCbFunc(callbackTag, status);
            }
            return;
        }
        else if (CPA_STATUS_SUCCESS == status)
        {
            osalAtomicDec(&(pCookie->pSessionDesc->pendingStatefulCbCount));

            /* Restart the slice */
            register_modify_params.consumedData = &pResults->consumed;
            register_modify_params.decompStateRegisters = decompStatetRegisters;
            register_modify_params.headerLengthInBits = s.bits_total;
            register_modify_params.sourceBuffer = pCookie->pSourceBuffer;
            status = updateStateRegisters(&register_modify_params);
        }

        if (CPA_STATUS_SUCCESS == status || CPA_STATUS_RETRY == status)
        {
            pSessionDesc->cumulativeConsumedBytes += pResults->consumed;
            pResults->status = CPA_DC_OK;
        }
    }


    if(CPA_TRUE == pSessionDesc->isDcDp)
    {
        /* Decrement number of stateless pending callbacks for session */
        pSessionDesc->pendingDpStatelessCbCount--;
        (pService->pDcDpCb)(pResponse);
    }
    else
    {
        /* Decrement number of pending callbacks for session */
        if(CPA_DC_STATELESS == pSessionDesc->sessState)
        {
            osalAtomicDec(&(pCookie->pSessionDesc->pendingStatelessCbCount));
        }
        else if (0 != osalAtomicGet(&pCookie->pSessionDesc->pendingStatefulCbCount))
        {
            osalAtomicDec(&(pCookie->pSessionDesc->pendingStatefulCbCount));
        }

        /* Free the memory pool */
        if (NULL != pCookie)
        {
            Lac_MemPoolEntryFree(pCookie);
            pCookie = NULL;
        }

        if(NULL != pCbFunc)
        {
            pCbFunc(callbackTag, status);
        }
    }
}

#ifdef ICP_PARAM_CHECK
/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Check the compression or decompression function parameters
 *
 * @description
 *      Check that all the parameters used for a compression or decompression
 *      request are valid
 *
 * @param[in]   pService              Pointer to the compression service
 * @param[in]   pSessionHandle        Session handle
 * @param[in]   pSrcBuff              Pointer to data buffer for compression
 * @param[in]   pDestBuff             Pointer to buffer space for data after
 *                                    compression
 * @param[in]   pResults              Pointer to results structure
 * @param[in]   flushFlag             Indicates the type of flush to be
 *                                    performed
 * @param[in]   srcBuffSize           Size of the source buffer
 * @param[in]   compDecomp            Direction of the operation
 *
 * @retval CPA_STATUS_SUCCESS         Function executed successfully
 * @retval CPA_STATUS_INVALID_PARAM   Invalid parameter passed in
 *
 *****************************************************************************/
STATIC CpaStatus
dcCheckCompressData(sal_compression_service_t *pService,
        CpaDcSessionHandle pSessionHandle,
        CpaBufferList *pSrcBuff,
        CpaBufferList *pDestBuff,
        CpaDcRqResults *pResults,
        CpaDcFlush flushFlag,
        Cpa64U srcBuffSize,
        dc_request_dir_t compDecomp)
{
    dc_session_desc_t *pSessionDesc = NULL;
    Cpa64U destBuffSize = 0;

    LAC_CHECK_NULL_PARAM(pSessionHandle);
    LAC_CHECK_NULL_PARAM(pSrcBuff);
    LAC_CHECK_NULL_PARAM(pDestBuff);
    LAC_CHECK_NULL_PARAM(pResults);

    pSessionDesc = DC_SESSION_DESC_FROM_CTX_GET(pSessionHandle);
    if(NULL == pSessionDesc)
    {
        LAC_INVALID_PARAM_LOG("Session handle not as expected");
        return CPA_STATUS_INVALID_PARAM;
    }

    if((flushFlag < CPA_DC_FLUSH_NONE) ||
       (flushFlag > CPA_DC_FLUSH_FULL))
    {
        LAC_INVALID_PARAM_LOG("Invalid flushFlag value");
        return CPA_STATUS_INVALID_PARAM;
    }

    if(pSrcBuff == pDestBuff)
    {
        LAC_INVALID_PARAM_LOG("In place operation not supported");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Compressing or decompressing zero bytes is not supported for stateless
     * sessions */
    if((CPA_DC_STATELESS == pSessionDesc->sessState) && (0 == srcBuffSize))
    {
        LAC_INVALID_PARAM_LOG("The source buffer size needs to be greater than "
            "zero byte for stateless sessions");
        return CPA_STATUS_INVALID_PARAM;
    }

    if(srcBuffSize > DC_BUFFER_MAX_SIZE)
    {
        LAC_INVALID_PARAM_LOG("The source buffer size needs to be less than or "
            "equal to 2^32-1 bytes");
        return CPA_STATUS_INVALID_PARAM;
    }

    if(LacBuffDesc_BufferListVerify(pDestBuff, &destBuffSize,
            LAC_NO_ALIGNMENT_SHIFT)
                != CPA_STATUS_SUCCESS)
    {
        LAC_INVALID_PARAM_LOG("Invalid destination buffer list parameter");
        return CPA_STATUS_INVALID_PARAM;
    }

    if(destBuffSize > DC_BUFFER_MAX_SIZE)
    {
        LAC_INVALID_PARAM_LOG("The destination buffer size needs to be less "
            "than or equal to 2^32-1 bytes");
        return CPA_STATUS_INVALID_PARAM;
    }

    if(CPA_TRUE == pSessionDesc->isDcDp)
    {
        LAC_INVALID_PARAM_LOG("The session type should not be data plane");
        return CPA_STATUS_INVALID_PARAM;
    }

    if(DC_COMPRESSION_REQUEST == compDecomp)
    {
#ifndef ICP_DC_DYN_NOT_SUPPORTED
        if(CPA_DC_HT_FULL_DYNAMIC == pSessionDesc->huffType)
        {
            /* Check if eSRAM or DRAM is supported */
            if((0 == pService->interBuff1eSRAMPhyAddr) &&
               (NULL == pService->pInterBuff1))
            {
                LAC_LOG_ERROR("No buffer defined for this instance - see "
                    "cpaDcStartInstance");
                return CPA_STATUS_INVALID_PARAM;
            }

            /* Ensure that the destination buffer size is greater or equal
             * to 128B */
            if(destBuffSize < DC_DEST_BUFFER_DYN_MIN_SIZE)
            {
                LAC_INVALID_PARAM_LOG("Destination buffer size should be "
                    "greater or equal to 128 bytes");
                return CPA_STATUS_INVALID_PARAM;
            }
        }
        else
#else
        if(CPA_DC_HT_FULL_DYNAMIC == pSessionDesc->huffType)
        {
            LAC_INVALID_PARAM_LOG("Invalid huffType value, dynamic sessions "
                "not supported");
            return CPA_STATUS_INVALID_PARAM;
        }
        else
#endif
        {
            /* Ensure that the destination buffer size is greater or equal
             * to 64B */
            if(destBuffSize < DC_DEST_BUFFER_STA_MIN_SIZE)
            {
                LAC_INVALID_PARAM_LOG("Destination buffer size should be "
                    "greater or equal to 64 bytes");
                return CPA_STATUS_INVALID_PARAM;
            }
        }
    }
    else
    {
        /* Ensure that the destination buffer size is greater or equal
         * to 16B */
        if(destBuffSize < DC_DEST_BUFFER_DEC_MIN_SIZE)
        {
            LAC_INVALID_PARAM_LOG("Destination buffer size should be "
                "greater or equal to 16 bytes");
            return CPA_STATUS_INVALID_PARAM;
        }
    }

    return CPA_STATUS_SUCCESS;
}
#endif

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Populate the compression request parameters
 *
 * @description
 *      This function will populate the compression request parameters
 *
 * @param[out]  pCompReqParams   Pointer to the compression request parameters
 * @param[in]   nextSlice        Next slice
 * @param[in]   pCookie          Pointer to the compression cookie
 * @param[in]   pService         Pointer to the compression service
 *
 *****************************************************************************/
STATIC void
dcCompRequestParamsPopulate(
    icp_qat_fw_comp_req_params_t *pCompReqParams,
    icp_qat_fw_slice_t nextSlice,
    dc_compression_cookie_t *pCookie,
    sal_compression_service_t *pService)
{
    LAC_ENSURE_NOT_NULL(pCompReqParams);

    pCompReqParams->next_id = nextSlice;
    pCompReqParams->curr_id = ICP_QAT_FW_SLICE_COMP;
    pCompReqParams->comp_len = pCookie->srcTotalDataLenInBytes;
    pCompReqParams->out_buffer_sz = pCookie->dstTotalDataLenInBytes;

    pCompReqParams->resrvd = 0;
    pCompReqParams->resrvd1 = 0;
}

#ifndef ICP_DC_DYN_NOT_SUPPORTED
/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Populate the translator request parameters
 *
 * @description
 *      This function will populate the translator request parameters
 *
 * @param[out]  pTransReqParams     Pointer to the translator request parameters
 * @param[in]   nextSlice           Next slice
 * @param[in]   interBuff1PhyAddr   Physical address of the first intermediate
 *                                  buffer
 * @param[in]   interBuff2PhyAddr   Physical address of the second intermediate
 *                                  buffer
 * @param[in]   interBufferType     Type of intermediate buffer
 *
 *****************************************************************************/
STATIC void
dcTransRequestParamsPopulate(
    icp_qat_fw_trans_req_params_t *pTransReqParams,
    icp_qat_fw_slice_t nextSlice,
    CpaPhysicalAddr interBuff1PhyAddr,
    CpaPhysicalAddr interBuff2PhyAddr,
    icp_qat_fw_comp_inter_buffer_type_t interBufferType)
{
    LAC_ENSURE_NOT_NULL(pTransReqParams);

    pTransReqParams->next_id = nextSlice;
    pTransReqParams->curr_id = ICP_QAT_FW_SLICE_XLAT;
    pTransReqParams->inter_buffer_1 = interBuff1PhyAddr;
    pTransReqParams->inter_buffer_2 = interBuff2PhyAddr;
    pTransReqParams->inter_buffer_type = interBufferType;

    pTransReqParams->resrvd = 0;
}
#endif

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Create the requests for compression or decompression
 *
 * @description
 *      Create the requests for compression or decompression. This function
 *      will update the cookie will all required information.
 *
 * @param{out]  pCookie             Pointer to the compression cookie
 * @param[in]   pService            Pointer to the compression service
 * @param[in]   pSessionDesc        Pointer to the session descriptor
 * @param[in    pSessionHandle      Session handle
 * @param[in]   pSrcBuff            Pointer to data buffer for compression
 * @param[in]   pDestBuff           Pointer to buffer space for data after
 *                                  compression
 * @param[in]   pResults            Pointer to results structure
 * @param[in]   flushFlag           Indicates the type of flush to be
 *                                  performed
 * @param[in]   callbackTag         Pointer to the callback tag
 * @param[in]   compDecomp          Direction of the operation
 *
 * @retval CPA_STATUS_SUCCESS       Function executed successfully
 * @retval CPA_STATUS_INVALID_PARAM Invalid parameter passed in
 *
 *****************************************************************************/
STATIC CpaStatus
dcCreateRequest(
    dc_compression_cookie_t *pCookie,
    sal_compression_service_t *pService,
    dc_session_desc_t *pSessionDesc,
    CpaDcSessionHandle pSessionHandle,
    CpaBufferList *pSrcBuff,
    CpaBufferList *pDestBuff,
    CpaDcRqResults *pResults,
    CpaDcFlush flushFlag,
    void *callbackTag,
    dc_request_dir_t compDecomp)
{
    Cpa8U *pReqParams = NULL;
    Cpa8U reqParamsOffset = 0;
    icp_qat_fw_la_bulk_req_t* pMsg = NULL;
    Cpa64U srcAddrPhys = 0, dstAddrPhys = 0;
    Cpa64U srcTotalDataLenInBytes = 0, dstTotalDataLenInBytes = 0;
    Cpa16U cmdFlags = 0;
    Cpa8U sop = ICP_QAT_FW_COMP_SOP;
    Cpa8U eop = ICP_QAT_FW_COMP_EOP;
    Cpa8U sessType = ICP_QAT_FW_COMP_STATELESS_SESSION;
    Cpa8U autoSelectBest = ICP_QAT_FW_COMP_NOT_AUTO_SELECT_BEST;
    Cpa8U enhancedAutoSelectBest = ICP_QAT_FW_COMP_NOT_ENH_AUTO_SELECT_BEST;
    Cpa8U disableType0EnhancedAutoSelectBest =
            ICP_QAT_FW_COMP_NOT_DISABLE_TYPE0_ENH_AUTO_SELECT_BEST;
    Cpa8U bFinal = ICP_QAT_FW_COMP_NOT_BFINAL;
    Cpa32U i = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
#ifndef ICP_DC_DYN_NOT_SUPPORTED
    CpaBoolean useDRAM = CPA_FALSE;
#endif
    icp_qat_fw_comp_req_t *pReqCache = NULL;
    Cpa8U *contentDescPtr = NULL;
    Cpa32U newChecksumValue = 0;

    /* Write the buffer descriptors */
    status = LacBuffDesc_BufferListDescWriteAndGetSize(pSrcBuff, &srcAddrPhys,
            CPA_FALSE, &srcTotalDataLenInBytes,
            &(pService->generic_service_info));
    if (status != CPA_STATUS_SUCCESS)
    {
        return status;
    }

    status = LacBuffDesc_BufferListDescWriteAndGetSize(pDestBuff, &dstAddrPhys,
            CPA_FALSE, &dstTotalDataLenInBytes,
            &(pService->generic_service_info));
    if (status != CPA_STATUS_SUCCESS)
    {
        return status;
    }

    if((DC_REQUEST_FIRST == pSessionDesc->requestType) || (CPA_DC_STATELESS == pSessionDesc->sessState))
    {
        if(DC_COMPRESSION_REQUEST == compDecomp)
        {
            contentDescPtr = &pSessionDesc->contentDescComp[0];
        }
        else if(DC_DECOMPRESSION_REQUEST == compDecomp)
        {
            contentDescPtr = &pSessionDesc->contentDescDecomp[0];
        }

        if(CPA_DC_ADLER32 == pSessionDesc->checksumType)
        {
            contentDescPtr += CD_ADLER32_CHECKSUM_OFFSET;
        }
        else if(CPA_DC_CRC32 == pSessionDesc->checksumType)
        {
            contentDescPtr += CD_CRC32_CHECKSUM_OFFSET;
        }
    }
    if(DC_REQUEST_FIRST == pSessionDesc->requestType)
    {
        if(CPA_DC_ADLER32 == pSessionDesc->checksumType)
        {
            newChecksumValue = 1;
            pSessionDesc->previousChecksum = 1;
        }
        else
        {
            newChecksumValue = 0;
            pSessionDesc->previousChecksum = 0;
        }
        dcUpdate_ContentDescriptor(newChecksumValue, contentDescPtr);
    }
    else if (CPA_DC_STATELESS == pSessionDesc->sessState)
    {
        newChecksumValue = pResults->checksum;
        pSessionDesc->previousChecksum = pResults->checksum;
        dcUpdate_ContentDescriptor(newChecksumValue, contentDescPtr);
    }

    /* Populate the compression cookie */
    pCookie->dcInstance = pService;
    pCookie->pSessionHandle = pSessionHandle;
    pCookie->callbackTag = callbackTag;
    pCookie->pSessionDesc = pSessionDesc;
    pCookie->flushFlag = flushFlag;
    pCookie->pResults = pResults;
    pCookie->compDecomp = compDecomp;

    /* The firmware expects the length in bytes for source and destination to be
     * Cpa32U parameters. However the total data length could be bigger as
     * allocated by the user. We ensure that this is not the case in
     * dcCheckCompressData and cast the values to Cpa32U here */
    pCookie->srcTotalDataLenInBytes = (Cpa32U)srcTotalDataLenInBytes;

#ifndef ICP_DC_DYN_NOT_SUPPORTED
    if((DC_COMPRESSION_REQUEST == compDecomp) &&
       (CPA_DC_HT_FULL_DYNAMIC == pSessionDesc->huffType))
    {
        if(NULL != pService->pInterBuff1)
        {
            useDRAM = CPA_TRUE;
        }

        if(0 != pService->interBuff1eSRAMPhyAddr)
        {
            /* The destination length passed to the firmware shall be the
             * minimum of the length of the intermediate buffer and the
             * destination buffer */
            if(pService->minSRAMBuffSizeInBytes <
                (Cpa32U)dstTotalDataLenInBytes)
            {
                if(CPA_TRUE == useDRAM)
                {
                    /* In this case we need to check if the DRAM has a bigger
                     * intermediate buffer size than eSRAM to try to limit the
                     * probability of getting an overflow */
                    if(pService->minSRAMBuffSizeInBytes >=
                            pService->minInterBuffSizeInBytes)
                    {
                        pCookie->dstTotalDataLenInBytes =
                            pService->minSRAMBuffSizeInBytes;
                        useDRAM = CPA_FALSE;
                    }
                }
                else
                {
                    pCookie->dstTotalDataLenInBytes =
                        pService->minSRAMBuffSizeInBytes;
                }
            }
            else
            {
                pCookie->dstTotalDataLenInBytes =
                    (Cpa32U)dstTotalDataLenInBytes;
                useDRAM = CPA_FALSE;
            }
        }

        if(CPA_TRUE == useDRAM)
        {
            if(pService->minInterBuffSizeInBytes <
                (Cpa32U)dstTotalDataLenInBytes)
            {
                pCookie->dstTotalDataLenInBytes =
                    (Cpa32U)(pService->minInterBuffSizeInBytes);
            }
            else
            {
                pCookie->dstTotalDataLenInBytes =
                    (Cpa32U)dstTotalDataLenInBytes;
            }
        }
    }
    else
#endif
    {
        pCookie->dstTotalDataLenInBytes = (Cpa32U)dstTotalDataLenInBytes;
    }

    if((CPA_DC_STATEFUL == pSessionDesc->sessState)
     &&(CPA_DC_FLUSH_FINAL != flushFlag)
     &&(CPA_DC_FLUSH_FULL != flushFlag)
     &&(DC_DECOMPRESSION_REQUEST == compDecomp)
     &&(1 == (pCookie->srcTotalDataLenInBytes & 0x00000001)))
    {
        /* If the total length of the source buffer is odd then update it to
         * an even number */
        pCookie->srcTotalDataLenInBytes--;
    }

    pReqParams = pCookie->dcReqParamsBuffer;

    pMsg = (icp_qat_fw_la_bulk_req_t*) &pCookie->request;

    /* Populate the cmdFlags */
    if(CPA_DC_STATEFUL == pSessionDesc->sessState)
    {
        pSessionDesc->previousRequestType = pSessionDesc->requestType;

        sessType = ICP_QAT_FW_COMP_STATEFUL_SESSION;

        if(DC_REQUEST_FIRST == pSessionDesc->requestType)
        {
            /* Update the request type for following requests */
            pSessionDesc->requestType = DC_REQUEST_SUBSEQUENT;

            /* Reinitialise the cumulative amount of consumed bytes */
            pSessionDesc->cumulativeConsumedBytes = 0;

            /* Reinitialise the previous checksum */
            if(CPA_DC_ADLER32 == pSessionDesc->checksumType)
            {
                pSessionDesc->previousChecksum = 1;
            }
            else
            {
                pSessionDesc->previousChecksum = 0;
            }
            LAC_LOG_DEBUG1("pSessionDesc->previousChecksum = 0x%x\n",
                      pSessionDesc->previousChecksum);

            if(DC_COMPRESSION_REQUEST == compDecomp)
            {
               pSessionDesc->isSopForCompressionProcessed = CPA_TRUE;
            }
            else if(DC_DECOMPRESSION_REQUEST == compDecomp)
            {
               pSessionDesc->isSopForDecompressionProcessed = CPA_TRUE;
            }
        }
        else
        {
            if (DC_COMPRESSION_REQUEST == compDecomp)
            {
               if(CPA_TRUE  == pSessionDesc->isSopForCompressionProcessed)
               {
                  sop = ICP_QAT_FW_COMP_NOT_SOP;
               }
               else
               {
                  pSessionDesc->isSopForCompressionProcessed = CPA_TRUE;
               }
            }
            else if (DC_DECOMPRESSION_REQUEST == compDecomp)
            {
               if(CPA_TRUE  == pSessionDesc->isSopForDecompressionProcessed)
               {
                  sop = ICP_QAT_FW_COMP_NOT_SOP;
               }
               else
               {
                  pSessionDesc->isSopForDecompressionProcessed = CPA_TRUE;
               }
            }
        }

        if((CPA_DC_FLUSH_FINAL == flushFlag)||(CPA_DC_FLUSH_FULL == flushFlag))
        {
            /* Update the request type for following requests */
            pSessionDesc->requestType = DC_REQUEST_FIRST;
        }
        else
        {
            eop = ICP_QAT_FW_COMP_NOT_EOP;
        }
    }

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

    if(CPA_DC_FLUSH_FINAL == flushFlag)
    {
        bFinal = ICP_QAT_FW_COMP_BFINAL;
    }

    cmdFlags = ICP_QAT_FW_COMP_FLAGS_BUILD(sop,
                       eop,
                       sessType,
                       autoSelectBest,
                       enhancedAutoSelectBest,
                       disableType0EnhancedAutoSelectBest,
                       bFinal);

    /* Walk the QAT slice chain for compression, excluding last (terminating)
     * entry */
    if(DC_COMPRESSION_REQUEST == compDecomp)
    {
        for (i = 0; (i < DC_MAX_NUM_QAT_SLICES_COMP - 1); i++)
        {
            if (ICP_QAT_FW_SLICE_COMP == pSessionDesc->qatSlicesComp[i])
            {
                icp_qat_fw_comp_req_params_t *pCompReqParams =
                    (icp_qat_fw_comp_req_params_t *)
                    (pReqParams + reqParamsOffset);

                LAC_ENSURE_NOT_NULL(pCompReqParams);

                dcCompRequestParamsPopulate(
                    pCompReqParams,
                    pSessionDesc->qatSlicesComp[i + 1],
                    pCookie,
                    pService);

                /* Update offset */
                reqParamsOffset += sizeof(*pCompReqParams);
            }
#ifndef ICP_DC_DYN_NOT_SUPPORTED
            else if (ICP_QAT_FW_SLICE_XLAT ==
                         pSessionDesc->qatSlicesComp[i])
            {
                CpaPhysicalAddr interBuff1PhyAddr = 0;
                CpaPhysicalAddr interBuff2PhyAddr = 0;
                icp_qat_fw_comp_inter_buffer_type_t interBufferType =
                    ICP_QAT_FW_INTER_USE_SGL;

                icp_qat_fw_trans_req_params_t *pTransReqParams =
                    (icp_qat_fw_trans_req_params_t *)
                    (pReqParams + reqParamsOffset);

                LAC_ENSURE_NOT_NULL(pTransReqParams);

                if(CPA_FALSE == useDRAM)
                {
                    interBuff1PhyAddr = pService->interBuff1eSRAMPhyAddr;
                    interBuff2PhyAddr = pService->interBuff2eSRAMPhyAddr;
                    interBufferType = ICP_QAT_FW_INTER_USE_FLAT;
                }
                else
                {
                    interBuff1PhyAddr = pService->interBuff1PhyAddr;
                    interBuff2PhyAddr = pService->interBuff2PhyAddr;
                    interBufferType = ICP_QAT_FW_INTER_USE_SGL;
                }

                dcTransRequestParamsPopulate(
                    pTransReqParams,
                    pSessionDesc->qatSlicesComp[i + 1],
                    interBuff1PhyAddr,
                    interBuff2PhyAddr,
                    interBufferType);

                /* Update offset */
                reqParamsOffset += sizeof(*pTransReqParams);
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
    else
    {
        /* Walk the QAT slice chain for decompression, excluding last
         * (terminating) entry */
        for (i = 0; (i < DC_MAX_NUM_QAT_SLICES_DECOMP - 1); i++)
        {
            if (ICP_QAT_FW_SLICE_COMP == pSessionDesc->qatSlicesDecomp[i])
            {
                icp_qat_fw_comp_req_params_t *pCompReqParams =
                    (icp_qat_fw_comp_req_params_t *)
                    (pReqParams + reqParamsOffset);

                LAC_ENSURE_NOT_NULL(pCompReqParams);

                dcCompRequestParamsPopulate(
                    pCompReqParams,
                    pSessionDesc->qatSlicesDecomp[i + 1],
                    pCookie,
                    pService);

                /* Update offset */
                reqParamsOffset += sizeof(*pCompReqParams);
            }
            /* Chain terminators */
            else if (ICP_QAT_FW_SLICE_DRAM_WR ==
                         pSessionDesc->qatSlicesDecomp[i])
            {
                /* End of chain */
                break;
            }
        } /* for (i = 0; ... */
    }

    if(DC_COMPRESSION_REQUEST == compDecomp)
    {
        pReqCache = &(pSessionDesc->reqCacheComp);
    }
    else
    {
        pReqCache = &(pSessionDesc->reqCacheDecomp);
    }

    pReqCache->flow_id = pSessionDesc->flowId;

    /* Fills in the initial 20 bytes of the ET ring message - cached from the
     * session descriptor */
    osalMemCopy((void*)pMsg, (void*)(pReqCache),
        SAL_SESSION_REQUEST_CACHE_SIZE_IN_BYTES);

    pMsg->comn_la_req.u.la_flags = cmdFlags;

    /* Populates the QAT common request middle part of the message
     * (LW 6 to 11) */
    LAC_MEM_SHARED_WRITE_FROM_PTR(
        pMsg->comn_mid.opaque_data, pCookie);

    pMsg->comn_mid.src_data_addr = srcAddrPhys;
    pMsg->comn_mid.dest_data_addr = dstAddrPhys;
    pMsg->req_params_addr = pCookie->dcReqParamsBufferPhyAddr;
    pMsg->comn_ftr.next_request_addr = 0;

    return CPA_STATUS_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Send a compression request to QAT
 *
 * @description
 *      Send the requests for compression or decompression to QAT
 *
 * @param{in]   pCookie               Pointer to the compression cookie
 * @param[in]   pService              Pointer to the compression service
 * @param[in]   pSessionDesc          Pointer to the session descriptor
 * @param[in]   compDecomp            Direction of the operation
 *
 * @retval CPA_STATUS_SUCCESS         Function executed successfully
 * @retval CPA_STATUS_INVALID_PARAM   Invalid parameter passed in
 *
 *****************************************************************************/
STATIC CpaStatus
dcSendRequest(dc_compression_cookie_t* pCookie,
              sal_compression_service_t* pService,
              dc_session_desc_t* pSessionDesc,
              dc_request_dir_t compDecomp)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    /* Send the request */
    status = icp_adf_transPutMsg(pService->trans_handle_compression_tx,
                            (void*)&(pCookie->request),
                            LAC_QAT_MSG_SZ_LW);

    if(CPA_STATUS_SUCCESS != status)
    {
        if((CPA_DC_STATEFUL == pSessionDesc->sessState) &&
          (CPA_STATUS_RETRY == status))
        {
            /* reset requestType after recieving an retry on
             * the stateful request */
            pSessionDesc->requestType = pSessionDesc->previousRequestType;
        }

        /* The pending requests counter will be decremented in the main calling
         * function */
    }
    return status;
}

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Process the synchronous and asynchronous case for compression or
 *      decompression
 *
 * @description
 *      Process the synchronous and asynchronous case for compression or
 *      decompression. This function will then create and send the request to
 *      the firmware.
 *
 * @param[in]   pService            Pointer to the compression service
 * @param[in]   pSessionDesc        Pointer to the session descriptor
 * @param[in]   pSessionHandle      Session handle
 * @param[in]   pSrcBuff            Pointer to data buffer for compression
 * @param[in]   pDestBuff           Pointer to buffer space for data after
 *                                  compression
 * @param[in]   pResults            Pointer to results structure
 * @param[in]   flushFlag           Indicates the type of flush to be
 *                                  performed
 * @param[in]   callbackTag         Pointer to the callback tag
 * @param[in]   compDecomp          Direction of the operation
 * @param[in]   isAsyncMode         Used to know if synchronous or asynchronous
 *                                  mode
 *
 * @retval CPA_STATUS_SUCCESS       Function executed successfully
 * @retval CPA_STATUS_FAIL          Function failed
 * @retval CPA_STATUS_RESOURCE      Resource error
 *
 *****************************************************************************/
STATIC CpaStatus
dcCompDecompData(sal_compression_service_t* pService,
        dc_session_desc_t* pSessionDesc,
        CpaDcSessionHandle pSessionHandle,
        CpaBufferList *pSrcBuff,
        CpaBufferList *pDestBuff,
        CpaDcRqResults *pResults,
        CpaDcFlush flushFlag,
        void *callbackTag,
        dc_request_dir_t compDecomp,
        CpaBoolean isAsyncMode)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    dc_compression_cookie_t *pCookie = NULL;

    if((LacSync_GenWakeupSyncCaller == pSessionDesc->pCompressionCb)
        && isAsyncMode == CPA_TRUE)
    {
        lac_sync_op_data_t *pSyncCallbackData = NULL;

        status = LacSync_CreateSyncCookie(&pSyncCallbackData);

        if(CPA_STATUS_SUCCESS == status)
        {
            status = dcCompDecompData(pService, pSessionDesc,
                    pSessionHandle, pSrcBuff, pDestBuff, pResults,
                    flushFlag, pSyncCallbackData, compDecomp, CPA_FALSE);
        }
        else
        {
            return status;
        }

        if(CPA_STATUS_SUCCESS == status)
        {
            CpaStatus syncStatus = CPA_STATUS_SUCCESS;

            syncStatus = LacSync_WaitForCallback(pSyncCallbackData,
                            DC_SYNC_CALLBACK_TIMEOUT,
                            &status, NULL);

            /* If callback doesn't come back */
            if(CPA_STATUS_SUCCESS != syncStatus)
            {
                if(DC_COMPRESSION_REQUEST == compDecomp)
                {
                    COMPRESSION_STAT_INC(numCompCompletedErrors, pService);
                }
                else
                {
                    COMPRESSION_STAT_INC(numDecompCompletedErrors, pService);
                }
                LAC_LOG_ERROR("Callback timed out");
                status = syncStatus;
            }
        }
        else
        {
            /* As the Request was not sent the Callback will never
             * be called, so need to indicate that we're finished
             * with cookie so it can be destroyed. */
            LacSync_SetSyncCookieComplete(pSyncCallbackData);
        }

        LacSync_DestroySyncCookie(&pSyncCallbackData);
        return status;
    }

    /* Allocate the compression cookie
     * The memory is freed in callback or in sendRequest if an error occurs
     */
    do {
        pCookie = (dc_compression_cookie_t*)
                                  Lac_MemPoolEntryAlloc(pService->compression_mem_pool);
        if (NULL == pCookie)
        {
            LAC_LOG_ERROR("Cannot get mem pool entry for compression");
            status = CPA_STATUS_RESOURCE;
        }
        else if ((void*)CPA_STATUS_RETRY == pCookie)
        {
            /* Give back the control to the OS */
            osalYield();
        }
    }while ((void*)CPA_STATUS_RETRY == pCookie);

    /* Initialise the addresses of the source and destination buffers
     * in the pCookie.
     */
    if(CPA_STATUS_SUCCESS == status)
    {
        pCookie->pSourceBuffer = pSrcBuff;
        pCookie->pDestinationBuffer = pDestBuff;
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        status = dcCreateRequest(pCookie, pService,
                pSessionDesc, pSessionHandle, pSrcBuff, pDestBuff,
                pResults, flushFlag, callbackTag, compDecomp);
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        status = dcSendRequest(pCookie, pService, pSessionDesc, compDecomp);
    }

    if(CPA_STATUS_SUCCESS == status)
    {
        if(DC_COMPRESSION_REQUEST == compDecomp)
        {
            COMPRESSION_STAT_INC(numCompRequests, pService);
        }
        else
        {
            COMPRESSION_STAT_INC(numDecompRequests, pService);
        }
    }
    else
    {
        if(DC_COMPRESSION_REQUEST == compDecomp)
        {
            COMPRESSION_STAT_INC(numCompRequestsErrors, pService);
        }
        else
        {
            COMPRESSION_STAT_INC(numDecompRequestsErrors, pService);
        }

        /* Decrement number of pending callbacks for session */
        if(CPA_DC_STATELESS == pSessionDesc->sessState)
        {
            osalAtomicDec(&(pSessionDesc->pendingStatelessCbCount));
        }
        else
        {
            osalAtomicDec(&(pSessionDesc->pendingStatefulCbCount));
        }
        /* Free the memory pool */
        if (NULL != pCookie)
        {
            Lac_MemPoolEntryFree(pCookie);
            pCookie = NULL;
        }
    }
    return status;
}

/**
 *****************************************************************************
 * @ingroup Dc_DataCompression
 *      Handle zero length compression or decompression requests
 *
 * @description
 *      Handle zero length compression or decompression requests
 *
 * @param[in]   pService              Pointer to the compression service
 * @param[in]   pSessionDesc          Pointer to the session descriptor
 * @param[in]   pResults              Pointer to results structure
 * @param[in]   flushFlag             Indicates the type of flush to be
 *                                    performed
 * @param[in]   callbackTag           User supplied value to help correlate
 *                                    the callback with its associated request
 * @param[in]   compDecomp            Direction of the operation
 *
 * @retval CPA_TRUE                   Zero length SOP or MOP processed
 * @retval CPA_FALSE                  Zero length EOP
 *
 *****************************************************************************/
STATIC CpaBoolean
dcZeroLengthRequests(sal_compression_service_t *pService,
        dc_session_desc_t *pSessionDesc,
        CpaDcRqResults *pResults,
        CpaDcFlush flushFlag,
        void *callbackTag,
        dc_request_dir_t compDecomp)
{
    CpaBoolean status = CPA_FALSE;
    CpaDcCallbackFn pCbFunc = pSessionDesc->pCompressionCb;

    if(DC_REQUEST_FIRST == pSessionDesc->requestType)
    {
        /* Reinitialise the cumulative amount of consumed bytes */
        pSessionDesc->cumulativeConsumedBytes = 0;

        /* Zero length SOP */
        if(CPA_DC_ADLER32 == pSessionDesc->checksumType)
        {
            pResults->checksum = 1;
        }
        else
        {
            pResults->checksum = 0;
        }

        status = CPA_TRUE;
    }
    else if((CPA_DC_FLUSH_NONE == flushFlag) ||
            (CPA_DC_FLUSH_SYNC == flushFlag))
    {
        /* Zero length MOP */
        pResults->checksum = pSessionDesc->previousChecksum;
        status = CPA_TRUE;
    }

    if(CPA_TRUE == status)
    {
        pResults->status = CPA_DC_OK;
        pResults->produced = 0;
        pResults->consumed = 0;

        /* Increment statistics */
        if(DC_COMPRESSION_REQUEST == compDecomp)
        {
            COMPRESSION_STAT_INC(numCompRequests, pService);
            COMPRESSION_STAT_INC(numCompCompleted, pService);
        }
        else
        {
            COMPRESSION_STAT_INC(numDecompRequests, pService);
            COMPRESSION_STAT_INC(numDecompCompleted, pService);
        }

        LAC_SPINUNLOCK(&(pSessionDesc->sessionLock));

        if((NULL != pCbFunc) && (LacSync_GenWakeupSyncCaller != pCbFunc))
        {
            pCbFunc(callbackTag, CPA_STATUS_SUCCESS);
        }

        return CPA_TRUE;
    }

    return CPA_FALSE;
}

CpaStatus
cpaDcCompressData(CpaInstanceHandle dcInstance,
        CpaDcSessionHandle pSessionHandle,
        CpaBufferList *pSrcBuff,
        CpaBufferList *pDestBuff,
        CpaDcRqResults *pResults,
        CpaDcFlush flushFlag,
        void *callbackTag)
{
    sal_compression_service_t* pService = NULL;
    dc_session_desc_t* pSessionDesc = NULL;
    CpaInstanceHandle insHandle = NULL;
    Cpa64U srcBuffSize = 0;

#ifdef ICP_TRACE
    LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx, "
            "0x%x, 0x%lx)\n",
            (LAC_ARCH_UINT)dcInstance,
            (LAC_ARCH_UINT)pSessionHandle,
            (LAC_ARCH_UINT)pSrcBuff,
            (LAC_ARCH_UINT)pDestBuff,
            (LAC_ARCH_UINT)pResults,
            flushFlag,
            (Cpa64U)callbackTag);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == dcInstance)
    {
         insHandle = dcGetFirstHandle();
    }
    else
    {
         insHandle = dcInstance;
    }

    pService = (sal_compression_service_t*) insHandle;

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(insHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(insHandle);
#endif

    /* Check if SAL is initialised otherwise return an error */
    SAL_RUNNING_CHECK(insHandle);

    /* This check is outside the parameter checking as it is needed to manage
     * zero length requests */
    if(LacBuffDesc_BufferListVerifyNull(pSrcBuff, &srcBuffSize,
            LAC_NO_ALIGNMENT_SHIFT)
               != CPA_STATUS_SUCCESS)
    {
        LAC_INVALID_PARAM_LOG("Invalid source buffer list parameter");
        return CPA_STATUS_INVALID_PARAM;
    }

#ifdef ICP_PARAM_CHECK
    /* Ensure this is a compression instance */
    SAL_CHECK_INSTANCE_TYPE(insHandle, SAL_SERVICE_TYPE_COMPRESSION);

    if(dcCheckCompressData(pService, pSessionHandle, pSrcBuff, pDestBuff,
        pResults, flushFlag, srcBuffSize, DC_COMPRESSION_REQUEST)
        != CPA_STATUS_SUCCESS)
    {
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    pSessionDesc = DC_SESSION_DESC_FROM_CTX_GET(pSessionHandle);

#ifdef ICP_PARAM_CHECK
    if(CPA_DC_DIR_DECOMPRESS == pSessionDesc->sessDirection)
    {
        LAC_INVALID_PARAM_LOG("Invalid sessDirection value");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

#ifdef ICP_DC_DYN_NOT_SUPPORTED
    if (pSessionDesc->huffType == CPA_DC_HT_FULL_DYNAMIC)
    {
        LAC_INVALID_PARAM_LOG("Invalid huffType value, dynamic sessions "
                              "not supported");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    if(CPA_DC_STATEFUL == pSessionDesc->sessState)
    {
        /* Lock the session to check if there are in-flight stateful requests */
        LAC_SPINLOCK(&(pSessionDesc->sessionLock));

        /* Check if there is already one in-flight stateful request */
        if(0 != osalAtomicGet(&(pSessionDesc->pendingStatefulCbCount)))
        {
            LAC_LOG_ERROR("Only one in-flight stateful request supported");
            LAC_SPINUNLOCK(&(pSessionDesc->sessionLock));
            return CPA_STATUS_RETRY;
        }

        if(0 == srcBuffSize)
        {
            if(CPA_TRUE == dcZeroLengthRequests(pService, pSessionDesc,
                    pResults, flushFlag, callbackTag, DC_COMPRESSION_REQUEST))
            {
                return CPA_STATUS_SUCCESS;
            }
        }

        osalAtomicInc(&(pSessionDesc->pendingStatefulCbCount));
        LAC_SPINUNLOCK(&(pSessionDesc->sessionLock));
    }
    else
    {
        osalAtomicInc(&(pSessionDesc->pendingStatelessCbCount));
    }

    return dcCompDecompData(pService, pSessionDesc,
               pSessionHandle, pSrcBuff, pDestBuff, pResults,
               flushFlag, callbackTag, DC_COMPRESSION_REQUEST, CPA_TRUE);
}

CpaStatus
cpaDcDecompressData(CpaInstanceHandle dcInstance,
        CpaDcSessionHandle pSessionHandle,
        CpaBufferList *pSrcBuff,
        CpaBufferList *pDestBuff,
        CpaDcRqResults *pResults,
        CpaDcFlush flushFlag,
        void *callbackTag)
{
    sal_compression_service_t* pService = NULL;
    dc_session_desc_t* pSessionDesc = NULL;
    CpaInstanceHandle insHandle = NULL;
    Cpa64U srcBuffSize = 0;

#ifdef ICP_TRACE
    LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx, "
                "0x%x, 0x%lx)\n",
                (LAC_ARCH_UINT)dcInstance,
                (LAC_ARCH_UINT)pSessionHandle,
                (LAC_ARCH_UINT)pSrcBuff,
                (LAC_ARCH_UINT)pDestBuff,
                (LAC_ARCH_UINT)pResults,
                flushFlag,
                (Cpa64U)callbackTag);
#endif

    if(CPA_INSTANCE_HANDLE_SINGLE == dcInstance)
    {
         insHandle = dcGetFirstHandle();
    }
    else
    {
         insHandle = dcInstance;
    }

    pService = (sal_compression_service_t*) insHandle;

#ifdef ICP_PARAM_CHECK
    LAC_CHECK_NULL_PARAM(insHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(insHandle);
#endif

    /* Check if SAL is initialised otherwise return an error */
    SAL_RUNNING_CHECK(insHandle);

    /* This check is outside the parameter checking as it is needed to manage
     * zero length requests */
    if(LacBuffDesc_BufferListVerifyNull(pSrcBuff, &srcBuffSize,
            LAC_NO_ALIGNMENT_SHIFT)
               != CPA_STATUS_SUCCESS)
    {
        LAC_INVALID_PARAM_LOG("Invalid source buffer list parameter");
        return CPA_STATUS_INVALID_PARAM;
    }

#ifdef ICP_PARAM_CHECK
    /* Ensure this is a compression instance */
    SAL_CHECK_INSTANCE_TYPE(insHandle, SAL_SERVICE_TYPE_COMPRESSION);

    if(dcCheckCompressData(pService, pSessionHandle, pSrcBuff, pDestBuff,
           pResults, flushFlag, srcBuffSize, DC_DECOMPRESSION_REQUEST)
           != CPA_STATUS_SUCCESS)
    {
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    pSessionDesc = DC_SESSION_DESC_FROM_CTX_GET(pSessionHandle);

#ifdef ICP_PARAM_CHECK
    if(CPA_DC_DIR_COMPRESS == pSessionDesc->sessDirection)
    {
        LAC_INVALID_PARAM_LOG("Invalid sessDirection value");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

#ifdef ICP_DC_DYN_NOT_SUPPORTED
    if (pSessionDesc->huffType == CPA_DC_HT_FULL_DYNAMIC)
    {
        LAC_INVALID_PARAM_LOG("Invalid huffType value, dynamic sessions "
                              "not supported");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    if(CPA_DC_STATEFUL == pSessionDesc->sessState)
    {
        /* Lock the session to check if there are in-flight stateful requests */
         LAC_SPINLOCK(&(pSessionDesc->sessionLock));

        /* Check if there is already one in-flight stateful request */
        if(0 != osalAtomicGet(&(pSessionDesc->pendingStatefulCbCount)))
        {
            LAC_LOG_ERROR("Only one in-flight stateful request supported");
            LAC_SPINUNLOCK(&(pSessionDesc->sessionLock));
            return CPA_STATUS_RETRY;
        }

        if(0 == srcBuffSize)
        {
            if(CPA_TRUE == dcZeroLengthRequests(pService, pSessionDesc,
                    pResults, flushFlag, callbackTag, DC_DECOMPRESSION_REQUEST))
            {
                return CPA_STATUS_SUCCESS;
            }
        }

        osalAtomicInc(&(pSessionDesc->pendingStatefulCbCount));
        LAC_SPINUNLOCK(&(pSessionDesc->sessionLock));
    }
    else
    {
        osalAtomicInc(&(pSessionDesc->pendingStatelessCbCount));
    }

    return dcCompDecompData(pService, pSessionDesc,
               pSessionHandle, pSrcBuff, pDestBuff, pResults,
               flushFlag, callbackTag, DC_DECOMPRESSION_REQUEST, CPA_TRUE);
}

CpaStatus
cpaDcBufferListGetMetaSize(const CpaInstanceHandle instanceHandle,
        Cpa32U numBuffers,
        Cpa32U *pSizeInBytes)
{
#ifdef ICP_PARAM_CHECK
    CpaInstanceHandle insHandle = NULL;

    if(CPA_INSTANCE_HANDLE_SINGLE == instanceHandle)
    {
         insHandle = dcGetFirstHandle();
    }
    else
    {
         insHandle = instanceHandle;
    }

    LAC_CHECK_INSTANCE_HANDLE(insHandle);
    LAC_CHECK_NULL_PARAM(pSizeInBytes);

    /* Ensure this is a compression instance */
    SAL_CHECK_INSTANCE_TYPE(insHandle, SAL_SERVICE_TYPE_COMPRESSION);

    if(0 == numBuffers)
    {
        LAC_INVALID_PARAM_LOG("Number of Buffers");
        return CPA_STATUS_INVALID_PARAM;
    }
#endif

    /* icp_buffer_list_desc_t is 8 bytes in size and icp_flat_buffer_desc_t
     * is 16 bytes in size. Therefore if icp_buffer_list_desc_t is aligned
     * so will each icp_flat_buffer_desc_t structure */
    /* The size of an icp_buffer_list_desc_t is added as well as the size of the
     * appropriate number of icp_flat_buffer_desc_t. This is to allow the
     * processing of the intermediate buffers */

    *pSizeInBytes =  (DC_NUM_INTER_BUFFERS * sizeof(icp_buffer_list_desc_t)) +
                     (sizeof(icp_flat_buffer_desc_t) *
                         (numBuffers + DC_NUM_INTER_BUFFERS - 1)) +
                     ICP_DESCRIPTOR_ALIGNMENT_BYTES;

#ifdef ICP_TRACE
    LAC_LOG4("Called with params (0x%lx, %d, 0x%lx[%d])\n",
            (LAC_ARCH_UINT)instanceHandle,
            numBuffers,
            (LAC_ARCH_UINT)pSizeInBytes, *pSizeInBytes);
#endif

    return CPA_STATUS_SUCCESS;
}
