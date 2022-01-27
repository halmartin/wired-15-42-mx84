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
 ***************************************************************************
 * @file lac_pke_qat_comms.c
 *
 * @ingroup LacAsymCommonQatComms
 *
 * This file implements the API for creating PKE QAT messages and sending
 * these to the QAT. It implements an API for creating a PKE QAT request, and
 * for sending a PKE message to the QAT.
 *
 ***************************************************************************/

/*
****************************************************************************
* Include public/global header files
****************************************************************************
*/

#include "cpa.h"

/*
****************************************************************************
* Include private header files
****************************************************************************
*/
/* ADF incldues */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/* QAT includes */
#include "icp_qat_fw_la.h"
#include "icp_qat_fw_pke.h"
#include "icp_qat_fw_mmp.h"

/* SAL includes */
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_list.h"
#include "lac_sym_qat.h"
#include "lac_sal_types_crypto.h"
#include "lac_pke_qat_comms.h"
#include "lac_pke_mmp.h"

/*
****************************************************************************
* Static Variables
****************************************************************************
*/

/*
****************************************************************************
* Define static function definitions
****************************************************************************
*/

/**
 ***************************************************************************
 * @ingroup LacAsymCommonQatComms
 *      Resizes parameters for a PKE request if required
 *
 * @description
 *      This function resizes the flat buffer parameters for a PKE request, if
 *  required, by calling icp_LacBufferResize for each input/output flat buffer
 *  parameter in the request data structure.  LacPke_RestoreParams is the
 *  corresponding function for undoing the buffer copies.
 *
 * @param[in] pParamInfo  The data pointers of the flat buffers from the
 *                        clientInputParams and clientOutputParams
 *                        arrays are resized as necessary and stored in
 *                        the pkeInputParams and pkeOutputParams arrays
 *                        respectively.  The client...Params arrays are
 *                        processed one-by-one from the start, and
 *                        processing ends once a NULL parameter is
 *                        encountered.  Consequently, the pke...Params
 *                        arrays should be initialized to zero as NULL
 *                        inputs won't be written as NULL outputs.
 * @param[in/out] pInternalInMemList
 *                        pointer to a list of Booleans that indicate if
 *                        input data buffers passed to QAT are internally or
 *                        externally allocated. This information needs to
 *                        be tracked to ensure we use the corect virt2phys
 *                        function, values may be updated by this function.
 * @param[in/out] pInternalInMemList
 *                        pointer to a list of Booleans that indicate if
 *                        output data buffers passed to QAT are internally
 *                        or externally allocated, values may be updated by
 *                        this function.
 * @param[in] instanceHandle  instanceHandle
 *
 *
 * @retval CPA_STATUS_SUCCESS     No error
 * @retval CPA_STATUS_RESOURCE    Resource error (e.g. failed memory allocation)
 *
 * @see LacPke_RestoreParams()
 * @see icp_LacBufferResize()
 ***************************************************************************/
STATIC
CpaStatus
LacPke_ResizeParams(lac_pke_qat_req_data_param_info_t *pParamInfo,
                     CpaBoolean *pInternalInMemList,
                     CpaBoolean *pInternalOutMemList,
                     CpaInstanceHandle instanceHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0;

    /* resize input parameter flat buffers (end if NULL encountered) */
    for (i = 0; (i < LAC_MAX_MMP_INPUT_PARAMS) &&
        (NULL != pParamInfo->clientInputParams[i]); i++)
    {
        /* resize buffer (round length up to whole quadwords) if
           required */
        Cpa32U dataLen = pParamInfo->clientInputParams[i]->dataLenInBytes;
        Cpa32U dataRoundLen = dataLen;
        Cpa32U offset = 0;
        if(pParamInfo->inArgSizeList[i])
        {
            dataRoundLen = pParamInfo->inArgSizeList[i];
            offset = (dataLen > pParamInfo->inArgSizeList[i]) ?
            (dataLen - pParamInfo->inArgSizeList[i]) : 0;
        }
        pParamInfo->pkeInputParams[i] = icp_LacBufferResize(
            instanceHandle,
            pParamInfo->clientInputParams[i]->pData + offset,
            dataLen - offset,
            LAC_ALIGN_POW2_ROUNDUP(
                dataRoundLen,
                LAC_QUAD_WORD_IN_BYTES),
                &(pInternalInMemList[i]));
        status =
            (!pParamInfo->pkeInputParams[i] ? CPA_STATUS_RESOURCE : status);
        LAC_CHECK_STATUS(status);
    }

    /* resize output parameter flat buffers (end if NULL encountered) */
    for (i = 0; (i < LAC_MAX_MMP_OUTPUT_PARAMS) &&
        (NULL != pParamInfo->clientOutputParams[i]); i++)
    {
        /* resize buffer (and round length up to whole quadwords) if
           required */
        /* Need to copy when resizing output buffer for case
           where status returned by PKE is _FALSE */
        Cpa32U dataLen = pParamInfo->clientOutputParams[i]->dataLenInBytes;
        Cpa32U dataRoundLen = dataLen;
        Cpa32U offset = 0;
        if(pParamInfo->outArgSizeList[i])
        {
            dataRoundLen = pParamInfo->outArgSizeList[i];
            offset = (dataLen > pParamInfo->outArgSizeList[i]) ?
            (dataLen - pParamInfo->outArgSizeList[i]) : 0;
        }
        pParamInfo->pkeOutputParams[i] = icp_LacBufferResize(
            instanceHandle,
            pParamInfo->clientOutputParams[i]->pData + offset,
            dataLen - offset,
            LAC_ALIGN_POW2_ROUNDUP(
                dataRoundLen,
                LAC_QUAD_WORD_IN_BYTES),
                 &(pInternalOutMemList[i]));

        status =
            (!pParamInfo->pkeOutputParams[i] ? CPA_STATUS_RESOURCE : status);
        LAC_CHECK_STATUS(status);
    }
    return status;
}

/**
 ***************************************************************************
 * @ingroup LacAsymCommonQatComms
 *      Restores parameters for a PKE request
 *
 * @description
 *      This function restores the flat buffer parameters for a PKE request, by
 * calling icp_LacBufferRestore for each input/output flat buffer parameter in
 * the request data structure.  LacPke_ResizeParams is the corresponding
 * function for doing the buffer resize.
 *
 * @param pParamInfo        IN  The data pointers from the pkeInputParams and
 *                              pkeOutputParams arrays are restored and stored
 *                              in the data pointers of the flat buffers in
 *                              the clientInputParams and clientOutputParams
 *                              arrays respectively.  The pke...Params arrays
 *                              are processed one-by-one from the start, and
 *                              processing ends once a NULL parameter is
 *                              encountered.  Consequently, the
 *                              client...Params arrays should be initialized
 *                              to zero as NULL inputs won't be written as
 *                              NULL outputs.
 *
 * @retval CPA_STATUS_SUCCESS       No error
 * @retval CPA_STATUS_RESOURCE       Resource error (e.g. failed memory free)
 *
 * @see LacPke_ResizeParams()
 * @see icp_LacBufferRestore()
 ***************************************************************************/
STATIC
CpaStatus
LacPke_RestoreParams(lac_pke_qat_req_data_param_info_t *pParamInfo)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0;

    /* restore input parameter flat buffers (end if NULL encountered) */
    for (i = 0; (i < LAC_MAX_MMP_INPUT_PARAMS) &&
        (NULL != pParamInfo->pkeInputParams[i]); i++)
    {
        /* restore buffer */
        /* don't copy when restoring an input buffer */
        Cpa32U dataLen = pParamInfo->clientInputParams[i]->dataLenInBytes;
        Cpa32U dataRoundLen = dataLen;
        Cpa32U offset = 0;
        if(pParamInfo->inArgSizeList[i])
        {
            dataRoundLen = pParamInfo->inArgSizeList[i];
            offset = (dataLen > pParamInfo->inArgSizeList[i]) ?
            (dataLen - pParamInfo->inArgSizeList[i]) : 0;
        }
        status = icp_LacBufferRestore(
            pParamInfo->clientInputParams[i]->pData + offset,
            dataLen - offset,
            pParamInfo->pkeInputParams[i],
            LAC_ALIGN_POW2_ROUNDUP(
                dataRoundLen,
                LAC_QUAD_WORD_IN_BYTES),
            CPA_FALSE);
        LAC_CHECK_STATUS(status);
    }

    /* restore output parameter flat buffers (end if NULL encountered) */
    for (i = 0; (i < LAC_MAX_MMP_OUTPUT_PARAMS) &&
        (NULL != pParamInfo->pkeOutputParams[i]); i++)
    {
        /* restore buffer */
        Cpa32U dataLen = pParamInfo->clientOutputParams[i]->dataLenInBytes;
        Cpa32U dataRoundLen = dataLen;
        Cpa32U offset = 0;
        if(pParamInfo->outArgSizeList[i])
        {
            dataRoundLen = pParamInfo->outArgSizeList[i];
            offset = (dataLen > pParamInfo->outArgSizeList[i]) ?
            (dataLen - pParamInfo->outArgSizeList[i]) : 0;
        }
        status = icp_LacBufferRestore(
            pParamInfo->clientOutputParams[i]->pData + offset,
            dataLen - offset,
            pParamInfo->pkeOutputParams[i],
            LAC_ALIGN_POW2_ROUNDUP(
                dataRoundLen,
                LAC_QUAD_WORD_IN_BYTES),
            CPA_TRUE);
        LAC_CHECK_STATUS(status);
    }
    return status;
}

/**
 ***************************************************************************
 * @ingroup LacAsymCommonQatComms
 *      Destroys a PKE request
 *
 * @description
 *      This function destroys a PKE request that was created using
 * LacPke_CreateRequest().  It should be called if an error occurs during
 * request create or request send, or else as part of the response callback.
 *
 * @param pRequestHandle    IN  Pointer to the request handle that identifies
 *                              the request to be destroyed.  The request
 *                              handle will get set to LAC_PKE_INVALID_HANDLE.
 *
 * @retval CPA_STATUS_SUCCESS       No error
 * @retval CPA_STATUS_RESOURCE       Resource error (e.g. failed memory free)
 *
 * @see LacPke_CreateRequest()
 ***************************************************************************/
STATIC
CpaStatus
LacPke_DestroyRequest(lac_pke_request_handle_t *pRequestHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_pke_qat_req_data_t *pReqData = NULL;

    /* extract head request data pointer from the request handle */
    pReqData = *pRequestHandle;

    /* invalidate the request handle */
    *pRequestHandle = LAC_PKE_INVALID_HANDLE;

    /* free all request data structures in the chain - continue even in
       the case of errors */
    while (NULL != pReqData)
    {
        lac_pke_qat_req_data_t *pNextReqData =
            pReqData->pNextReqData;

        /* restore parameters (i.e. undo resizing) */
        if (CPA_STATUS_SUCCESS != LacPke_RestoreParams(&pReqData->paramInfo))
        {
            status = CPA_STATUS_RESOURCE;
        }

        Lac_MemPoolEntryFree(pReqData);
        pReqData = pNextReqData;
    }
    return status;
}

/**
 ***************************************************************************
 * @ingroup LacAsymCommonQatComms
 *      PKE callback
 ***************************************************************************/
void
LacPke_MsgCallback(icp_comms_trans_handle transHandle, void *pRespMsg)
{
    icp_qat_fw_pke_response_t *pPkeRespMsg = NULL;

    lac_pke_request_handle_t requestHandle = LAC_PKE_INVALID_HANDLE;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaBoolean pass = CPA_TRUE;
    lac_pke_qat_req_data_t *pReqData = NULL;
    lac_pke_op_cb_func_t pCbFunc = NULL;
    lac_pke_op_cb_data_t cbData = {0};
    CpaInstanceHandle instanceHandle = CPA_INSTANCE_HANDLE_SINGLE;
    Cpa8U opStatus = ICP_QAT_FW_COMN_STATUS_FLAG_OK;

    /* cast response message to PKE response message type */
    pPkeRespMsg = (icp_qat_fw_pke_response_t *)pRespMsg;

    opStatus = pPkeRespMsg->comn_resp.comn_status;

    /* check QAT response status */
    pass = (CpaBoolean)(ICP_QAT_FW_COMN_STATUS_FLAG_OK ==
        ICP_QAT_FW_COMN_RESP_PKE_STAT_GET(opStatus));

    /* extract request data pointer from the opaque data */
    LAC_MEM_SHARED_READ_TO_PTR(pPkeRespMsg->comn_resp.opaque_data, pReqData);


    /* extract fields from request data struct */
    pCbFunc = pReqData->cbInfo.cbFunc;
    cbData = pReqData->cbInfo.cbData;
    instanceHandle = pReqData->cbInfo.instanceHandle;

    /* destroy the request */
    requestHandle = (lac_pke_request_handle_t)pReqData->pHeadReqData;
    status = LacPke_DestroyRequest(&requestHandle);

    /* call the client callback */
    (*pCbFunc)(status, pass, instanceHandle, &cbData);
}

/**
 ***************************************************************************
 * @ingroup LacAsymCommonQatComms
 *      Init PKE requests
 ***************************************************************************/

void
LacPke_InitAsymRequest(Cpa8U *pData, CpaInstanceHandle instanceHandle)
{
    sal_crypto_service_t *pCryptoService =
                                (sal_crypto_service_t *) instanceHandle;
    lac_pke_qat_req_data_t *pReqData = (lac_pke_qat_req_data_t *) pData;
    sal_qat_content_desc_info_t mmplibCD = {0};

    /* Populate content descriptor info with MMP Lib details */
    mmplibCD.dataPhys = pCryptoService->mmpPhysAddr;
    mmplibCD.hdrSzQuadWords = 0;
    mmplibCD.hwBlkSzQuadWords = 0;

    /* LW 0 */
    SalQatMsg_ReqTypePopulate(&(pReqData->u1.request.comn_hdr.arch_if),
                             ICP_ARCH_IF_REQ_QAT_FW_PKE,
                             pCryptoService->asymResponseRingId);

    /* LW 1-4 */
    SalQatMsg_CmnMsgHdrPopulate(
        (icp_qat_fw_la_bulk_req_t *)(&(pReqData->u1.request)),
        &mmplibCD,
        pCryptoService->pkeCmnReqFlags,
        pCryptoService->pkeFlowId);

    /* Common request middle part (LW 6-11) */
    SalQatMsg_CmnMsgMidPopulate(&(pReqData->u1.request.comn_mid),
                  pReqData,
                  LAC_OS_VIRT_TO_PHYS_INTERNAL(&pReqData->u2.inArgList),
                  LAC_OS_VIRT_TO_PHYS_INTERNAL(&pReqData->u3.outArgList));

     /* part of LW12 */
     pReqData->u1.request.resrvd = 0;

     /* LW13 */
     pReqData->u1.request.resrvd1 = 0;

}

/**
 ***************************************************************************
 * @ingroup LacAsymCommonQatComms
 *      PKE request creation
 ***************************************************************************/

CpaStatus
LacPke_CreateRequest(
    lac_pke_request_handle_t *pRequestHandle,
    Cpa32U functionalityId,
    Cpa32U *pInArgSizeList,
    Cpa32U *pOutArgSizeList,
    icp_qat_fw_mmp_input_param_t *pInArgList,
    icp_qat_fw_mmp_output_param_t *pOutArgList,
    CpaBoolean *pInternalInMemList,
    CpaBoolean *pInternalOutMemList,
    lac_pke_op_cb_func_t pPkeOpCbFunc,
    lac_pke_op_cb_data_t *pCbData,
    CpaInstanceHandle instanceHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_pke_qat_req_data_t *pReqData = NULL;
    Cpa8U i = 0;
    sal_crypto_service_t* pCryptoService =
                                 (sal_crypto_service_t*) instanceHandle;

    /* allocate request data */
    pReqData = Lac_MemPoolEntryAlloc(pCryptoService->lac_pke_req_pool);
    if ((NULL == pReqData) || ((void*)CPA_STATUS_RETRY == pReqData))
    {
        LAC_LOG_ERROR("Cannot get a mem pool entry");
        return CPA_STATUS_RESOURCE;
    }
    LAC_ASSERT_NOT_NULL(pReqData);

    /* ensure correct request structure alignment */
    LAC_ASSERT(
            LAC_ADDRESS_ALIGNED(
                &pReqData->u1.request, LAC_OPTIMAL_ALIGNMENT_SHIFT),
            "request structure not correctly aligned");

    /* ensure correct input argument list structure alignment */
    LAC_ASSERT(
            LAC_ADDRESS_ALIGNED(
                &pReqData->u2.inArgList, LAC_OPTIMAL_ALIGNMENT_SHIFT),
            "inArgList structure not correctly aligned");

    /* ensure correct output argument list structure alignment */
    LAC_ASSERT(
            LAC_ADDRESS_ALIGNED(
                &pReqData->u3.outArgList, LAC_OPTIMAL_ALIGNMENT_SHIFT),
            "outArgList structure not correctly aligned");

    /* initialize handle for single request, or first in a chain */
    if (*pRequestHandle == LAC_PKE_INVALID_HANDLE)
    {
        /* store request data pointer in the request handle */
        *pRequestHandle =  (lac_pke_request_handle_t)pReqData;
        /* initialize next, head, and tail request data pointers */
        pReqData->pNextReqData = NULL;
        pReqData->pHeadReqData = pReqData;
        /* note: tail pointer is only valid in head request data struct */
        pReqData->pTailReqData = pReqData;
    }
    else /* handle second or subsequent request in a chain */
    {
        lac_pke_qat_req_data_t *pHeadReqData = NULL;
        lac_pke_qat_req_data_t *pTailReqData = NULL;

        /* extract head request data pointer from the request handle */
        pHeadReqData = *pRequestHandle;
        LAC_ASSERT_NOT_NULL(pHeadReqData);

        /* get tail request data pointer from head request data pointer */
        pTailReqData = pHeadReqData->pTailReqData;
        LAC_ASSERT_NOT_NULL(pTailReqData);

        /* chain the two requests */
        LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_INTERNAL(
                pTailReqData->u1.request.comn_ftr.next_request_addr,
                pReqData);

        /* chain the request data structures */
        pTailReqData->pNextReqData = pReqData;
        pHeadReqData->pTailReqData = pReqData;
        pReqData->pNextReqData = NULL;
        pReqData->pHeadReqData = pHeadReqData;
        /* note: tail pointer not stored here as it changes (unlike head) */
    }

    /* populate request data structure */
    pReqData->cbInfo.cbFunc = pPkeOpCbFunc;
    pReqData->cbInfo.cbData = *pCbData;
    pReqData->cbInfo.instanceHandle = instanceHandle;
    pReqData->pNextReqData = NULL;

    /* clear the previous param info */
    LAC_OS_BZERO(&pReqData->paramInfo, sizeof(pReqData->paramInfo));

    /* if the list is passed by the user, store it in prealocated memory */
    if(NULL != pInArgSizeList)
    {
        memcpy(&pReqData->paramInfo.inArgSizeList, pInArgSizeList,
                sizeof(pReqData->paramInfo.inArgSizeList));
    }
    if(NULL != pOutArgSizeList)
    {
        memcpy(&pReqData->paramInfo.outArgSizeList, pOutArgSizeList,
                sizeof(pReqData->paramInfo.outArgSizeList));
    }

    /** @performance : the caller's input/output parameter lists are copied
       here into internal structures.  it would be more efficient, if
       possible, to have the caller populate the internal structure
       directly. */

    /* store input parameters in req struct (end if NULL encountered) */
    for (i = 0; (i < LAC_MAX_MMP_INPUT_PARAMS) &&
            (0 != pInArgList->flat_array[i]); i++)
    {
        LAC_MEM_SHARED_READ_TO_PTR(
                pInArgList->flat_array[i],
                pReqData->paramInfo.clientInputParams[i]);
    }

    /* store output parameters in req struct (end if NULL encountered) */
    for (i = 0; (i < LAC_MAX_MMP_OUTPUT_PARAMS) &&
            (0 != pOutArgList->flat_array[i]); i++)
    {
        LAC_MEM_SHARED_READ_TO_PTR(
                pOutArgList->flat_array[i],
                pReqData->paramInfo.clientOutputParams[i]);
    }

    /* resize parameters */
    status = LacPke_ResizeParams(
            &pReqData->paramInfo, pInternalInMemList,
            pInternalOutMemList, instanceHandle);

    if (CPA_STATUS_SUCCESS == status)
    {
        Cpa8U numInputParams = 0;
        Cpa8U numOutputParams = 0;

        /* LW5 */
        pReqData->u1.request.functionalityId = functionalityId;

        /* LW 14 and 15 set to zero for this request for now */
        pReqData->u1.request.comn_ftr.next_request_addr = 0;

        /* store correctly sized in params in QAT struct
           (end if NULL encountered) */
        for (i = 0; (i < LAC_MAX_MMP_INPUT_PARAMS) &&
            (NULL != pReqData->paramInfo.pkeInputParams[i]); i++)
        {
            if(CPA_TRUE == pInternalInMemList[i])
            {
                /* pkeInputParams[i] is referencing internally allocated
                   memory */
                LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_INTERNAL(
                   pReqData->u2.inArgList.flat_array[i],
                   pReqData->paramInfo.pkeInputParams[i]);
            }
            else
            {
                /* pkeInputParams[i] is referencing externally allocated
                   memory */
                LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_EXTERNAL(
                   pCryptoService->generic_service_info,
                   pReqData->u2.inArgList.flat_array[i],
                   pReqData->paramInfo.pkeInputParams[i]);
            }
        }
        numInputParams = i;

        /* store correctly sized out params in QAT struct
            (end if NULL encountered) */
        for (i = 0; (i < LAC_MAX_MMP_OUTPUT_PARAMS) &&
            (NULL != pReqData->paramInfo.pkeOutputParams[i]); i++)
        {
            if(CPA_TRUE == pInternalOutMemList[i])
            {
                LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_INTERNAL(
                    pReqData->u3.outArgList.flat_array[i],
                    pReqData->paramInfo.pkeOutputParams[i]);
            }
            else
            {
                LAC_MEM_SHARED_WRITE_VIRT_TO_PHYS_PTR_EXTERNAL(
                    pCryptoService->generic_service_info,
                    pReqData->u3.outArgList.flat_array[i],
                    pReqData->paramInfo.pkeOutputParams[i]);
            }
        }
        numOutputParams = i;

        LAC_ASSERT(((numInputParams + numOutputParams) <= LAC_MAX_MMP_PARAMS),
            "number of input/output parameters exceeds maximum allowed");

        /* Complete LW12 */
        pReqData->u1.request.input_param_count = numInputParams;
        pReqData->u1.request.output_param_count = numOutputParams;
    }

    /* clean up in the event of an error */
    if (CPA_STATUS_SUCCESS != status)
    {
        /* destroy the request (chain) */
        (void) LacPke_DestroyRequest(pRequestHandle);
    }

    return status;
}

/**
 ***************************************************************************
 * @ingroup LacAsymCommonQatComms
 *      PKE request send to QAT
 ***************************************************************************/
CpaStatus
LacPke_SendRequest(
    lac_pke_request_handle_t *pRequestHandle,
    CpaInstanceHandle instanceHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_pke_qat_req_data_t *pHeadReqData = NULL;
    sal_crypto_service_t* pCryptoService =
        (sal_crypto_service_t*) instanceHandle;

    LAC_ASSERT_NOT_NULL(pRequestHandle);

    /* extract head request data pointer from the request handle */
    pHeadReqData = *pRequestHandle;
    LAC_ASSERT_NOT_NULL(pHeadReqData);

    /* send the request (chain) */
    status = icp_adf_transPutMsg(pCryptoService->trans_handle_asym_tx,
                                 (void*)&(pHeadReqData->u1.request),
                                 LAC_QAT_MSG_SZ_LW);
    if (CPA_STATUS_SUCCESS != status)
    {
        /* destroy the request (chain) */
        (void) LacPke_DestroyRequest(pRequestHandle);
        return status;
    }

    return status;
}

/**
 ***************************************************************************
 * @ingroup LacAsymCommonQatComms
 *      PKE request create and send to QAT
 ***************************************************************************/
CpaStatus
LacPke_SendSingleRequest(
    Cpa32U functionalityId,
    Cpa32U *pInArgSizeList,
    Cpa32U *pOutArgSizeList,
    icp_qat_fw_mmp_input_param_t *pInArgList,
    icp_qat_fw_mmp_output_param_t *pOutArgList,
    CpaBoolean *pInMemBool,
    CpaBoolean *pOutMemBool,
    lac_pke_op_cb_func_t pPkeOpCbFunc,
    lac_pke_op_cb_data_t *pCbData,
    CpaInstanceHandle instanceHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_pke_request_handle_t requestHandle = LAC_PKE_INVALID_HANDLE;

    /* prepare the request */
    status = LacPke_CreateRequest(
        &requestHandle, functionalityId,
        pInArgSizeList, pOutArgSizeList,
        pInArgList, pOutArgList,
        pInMemBool, pOutMemBool,
        pPkeOpCbFunc, pCbData,
        instanceHandle);

    if (CPA_STATUS_SUCCESS == status)
    {
        /* send the request */
        status = LacPke_SendRequest( &requestHandle, instanceHandle);
    }

    return status;
}

