/***************************************************************************
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
 ***************************************************************************
 * @file lac_pke_utils.c
 *
 * @ingroup LacAsymCommonUtils
 *
 * Implementation of utils that are PKE specific
 *
 ******************************************************************************/

/*
********************************************************************************
* Include public/global header files
********************************************************************************
*/
#include "cpa.h"

/*
********************************************************************************
* Include private header files
********************************************************************************
*/

/* OSAL includes */
#include "Osal.h"

/* ADF includes */
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_debug.h"
#include "icp_adf_transport.h"
#include "icp_adf_poll.h"
#include "icp_accel_devices.h"

/* FW includes */
#include "icp_qat_fw_mmp_ids.h"
#include "icp_qat_fw_la.h"

/* SAL includes */
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_log.h"
#include "lac_mem_pools.h"
#include "lac_ec.h"
#include "lac_sym.h"
#include "lac_list.h"
#include "lac_sal_types_crypto.h"
#include "lac_pke_qat_comms.h"
#include "lac_pke_utils.h"


/*
********************************************************************************
* Static Variables
********************************************************************************
*/

/*
 * Values below used to wait on completion of mmp liveness request
 * by polling completion variable.  10-second timeout chosen, to avoid
 * infinite wait
 */
#define LAC_PKE_WAIT_COUNT      (1000)
/**< Number of times to sleep and wakeup */
#define LAC_PKE_DELAY_IN_MS   (10)
/**< Time to sleep in ms */

#define MMP_LIVENESS_DATA_SIZE 64
/**< Size of buffer sent in PKE liveness requests */

#define PKE_FW_ID_IDX 0
/**< index of Cpa32U array where fw id can be found */

/*
********************************************************************************
* Define static function definitions
********************************************************************************
*/

/*
********************************************************************************
* Global Variables
********************************************************************************
*/

/*
********************************************************************************
* Define public/global function definitions
********************************************************************************
*/

Cpa32S
LacPke_Compare(
    const CpaFlatBuffer* pFlatBufferA,
    const Cpa32S deltaA,
    const CpaFlatBuffer* pFlatBufferB,
    const Cpa32S deltaB)
{
    Cpa32S delta = 0;
    Cpa32S notZero = 0;
    Cpa32U lenA = 0;
    Cpa32U lenB = 0;

    LAC_ASSERT_NOT_NULL(pFlatBufferA);
    LAC_ASSERT_NOT_NULL(pFlatBufferB);
    LAC_ASSERT_NOT_NULL(pFlatBufferA->pData);
    LAC_ASSERT_NOT_NULL(pFlatBufferB->pData);
    LAC_ENSURE(((deltaA > -(1<<30)) && (deltaA < (1<<30))),
                                          "deltaA out of range");
    LAC_ENSURE(((deltaB > -(1<<30)) && (deltaB < (1<<30))),
                                          "deltaB out of range");

    delta = deltaA - deltaB;
    lenA = pFlatBufferA->dataLenInBytes;
    lenB = pFlatBufferB->dataLenInBytes;

    LAC_ENSURE((lenA + lenB > 0), "Invalid Buffer lengths");

    /* Shortcut for buffers of the same size */
   if((lenA == lenB) && (delta == 0))
   {
       return memcmp(pFlatBufferA->pData, pFlatBufferB->pData, lenA);
   }

   /* Run a full subtraction byte per byte starting
      from the least significant byte and taking
      care of buffers of different sizes */
    while(lenA | lenB)
    {
       if(lenA)
       {
           delta += pFlatBufferA->pData[--lenA];
       }
       if(lenB)
       {
           delta -= pFlatBufferB->pData[--lenB];
       }
       notZero |= (delta & 0xFF);
       /* signed arithmetic shift */
       delta >>= LAC_NUM_BITS_IN_BYTE;
   }
   /* Convert all neg numbers to -1 and all positive numbers to +1 */
   return notZero ? LAC_SIGNED_CONVERT32(delta): 0;

}

Cpa32S
LacPke_CompareZero (
    const CpaFlatBuffer* pLargeInteger,
    const Cpa32S delta)
{
    Cpa32S result = 0;
    Cpa32U i = 0;

    LAC_ASSERT_NOT_NULL(pLargeInteger);
    LAC_ENSURE(((delta >= -128) && (delta <= 127)), "delta out of range");

    i=pLargeInteger->dataLenInBytes-1;
    while(i--)
    {
        if(pLargeInteger->pData[i])
        {
           /* Data in Buffer+delta>0 */
           return 1;
        }
    }
    result = pLargeInteger->pData[(pLargeInteger->dataLenInBytes-1)] + delta;
    return result;
}

Cpa32U
LacPke_GetMinBytes(const CpaFlatBuffer *pBuffer)
{
   Cpa32U buffSizeInBytes = pBuffer->dataLenInBytes;
   Cpa32U i=0;

   /* starting at the MSB loop through all bytes until
      find the first non-zero byte if it exists */
   while((i < buffSizeInBytes) && (pBuffer->pData[i] ==0))
   {
       i++;
   }
   return (buffSizeInBytes - i);
}

CpaStatus
LacPke_GetBitPos(const CpaFlatBuffer *pBuffer , Cpa32U *pBitPos,
                            Cpa32U *pIndexMsb , CpaBoolean *pIsZero)
{
    Cpa32U numBytes = 0;
    Cpa32U buffSizeInBytes = pBuffer->dataLenInBytes;
    Cpa32U bitPosition = 0;
    Cpa8U msByte = 0;

    /* Set zero flag false initally */
    *pIsZero = CPA_FALSE;
    /* Find the first non-zero byte from end of buffer */
    numBytes = LacPke_GetMinBytes(pBuffer);
    /* set pIndexMsb */
    *pIndexMsb = buffSizeInBytes - numBytes;

    if(0 == numBytes)
    {
        /* Number is 0 */
        *pIsZero = CPA_TRUE;
        (*pIndexMsb)--;
        bitPosition = 0;
    }
    else
    {
        if(numBytes > LAC_BITS_TO_BYTES(LAC_4096_BITS))
        {
            LAC_INVALID_PARAM_LOG("Number size not supported");
            return CPA_STATUS_INVALID_PARAM;
        }
        /* calc bit position rounded up to nearest byte */
        bitPosition = (numBytes*LAC_NUM_BITS_IN_BYTE) - 1;

        msByte = pBuffer->pData[(*pIndexMsb)];

        /* Locate the msb in the MSB */
        while((msByte & 0x80) == 0)
        {
            msByte <<= 1;
            bitPosition--;
        }
    }
    *pBitPos = bitPosition;
    return CPA_STATUS_SUCCESS;
}

CpaStatus
LacPke_GetBitLen(const CpaFlatBuffer *pBuffer , Cpa32U *pBitLen)
{
    Cpa32U numBytes = 0;
    Cpa32U buffSizeInBytes = pBuffer->dataLenInBytes;
    Cpa32U bitLength = 0;
    Cpa8U msByte = 0;

    /* Find the first non-zero byte */
    numBytes = LacPke_GetMinBytes(pBuffer);
    if(0 == numBytes)
    {
        /* Number is 0 */
        bitLength = 0;
    }
    else
    {
        if(numBytes > LAC_BITS_TO_BYTES(LAC_4096_BITS))
        {
            LAC_INVALID_PARAM_LOG("Number size not supported");
            return CPA_STATUS_INVALID_PARAM;
        }
        /* calc bit position rounded up to nearest byte */
        bitLength = (numBytes*LAC_NUM_BITS_IN_BYTE);
        msByte = pBuffer->pData[(buffSizeInBytes - numBytes)];

        /* Locate the msb in the MSB */
        while((msByte & 0x80) == 0)
        {
            msByte <<= 1;
            bitLength--;
        }
    }
    *pBitLen = bitLength;
    return CPA_STATUS_SUCCESS;
}

STATIC void
LacPke_VerifyMmpCbFunc(
    CpaStatus status,
    CpaBoolean pass,
    CpaInstanceHandle instanceHandle,
    lac_pke_op_cb_data_t *pCbData)
{
    CpaBoolean *pCompletion = NULL;

    LAC_ASSERT_NOT_NULL(pCbData);
    pCompletion = (CpaBoolean *)pCbData->pOpaqueData;

    if ((CPA_STATUS_SUCCESS == status ) && (NULL != pCompletion))
    {
        /* set the completion to true to unblock function pending on this */
        *pCompletion = CPA_TRUE;
    }
    else
    {
        LAC_LOG_ERROR("Error in internal callback for PKE MMP Liveness");
    }
}

STATIC CpaStatus
LacPke_VerifyMmpLib(CpaInstanceHandle instanceHandle)
{
    CpaStatus status = CPA_STATUS_FAIL;
    sal_crypto_service_t *pCryptoService = NULL;
    icp_qat_fw_mmp_input_param_t inArgList;
    icp_qat_fw_mmp_output_param_t outArgList;
    CpaBoolean *internalMemIn = NULL;
    CpaBoolean *internalMemOut = NULL;
    Cpa32U inArgSizeList[LAC_MAX_MMP_INPUT_PARAMS] = {0};
    Cpa32U outArgSizeList[LAC_MAX_MMP_OUTPUT_PARAMS] = {0};
    lac_pke_op_cb_data_t cbData;
    CpaFlatBuffer outBuffer = {0};
    Cpa32U *pOutBufferData = NULL;
    volatile CpaBoolean *pCompletion = NULL;
    Cpa32U count = 0;
    Cpa32U loadedFwId = 0;
    Cpa32U quota = 1;  /* 1 message to be polled from asym rx ring */
    Cpa32U numTransHandles = 1; /* 1 asym rx ring */
    outArgSizeList[0] = MMP_LIVENESS_DATA_SIZE;

    status = LAC_OS_MALLOC(&pCompletion, sizeof(CpaBoolean));

    if (CPA_STATUS_SUCCESS == status)
    {
        status = LAC_OS_CAMALLOC(&pOutBufferData, MMP_LIVENESS_DATA_SIZE,
                     LAC_64BYTE_ALIGNMENT, 0);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        status = LAC_OS_CAMALLOC(&internalMemIn, LAC_MAX_MMP_INPUT_PARAMS,
                                 LAC_64BYTE_ALIGNMENT, 0);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        status = LAC_OS_CAMALLOC(&internalMemOut, LAC_MAX_MMP_OUTPUT_PARAMS,
                                 LAC_64BYTE_ALIGNMENT, 0);
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_OS_BZERO(&inArgList, sizeof(inArgList));
        LAC_OS_BZERO(&outArgList, sizeof(outArgList));
        LAC_OS_BZERO(pOutBufferData, MMP_LIVENESS_DATA_SIZE);
        memset(internalMemIn, CPA_FALSE, LAC_MAX_MMP_INPUT_PARAMS);
        memset(internalMemOut, CPA_FALSE, LAC_MAX_MMP_OUTPUT_PARAMS);

        outBuffer.dataLenInBytes = MMP_LIVENESS_DATA_SIZE;
        outBuffer.pData = (Cpa8U *)pOutBufferData;

        LAC_MEM_SHARED_WRITE_FROM_PTR(outArgList.flat_array[0], &outBuffer);

        /* 1 output buffer allocated internally */
        internalMemOut[0]=CPA_TRUE;

        /* set up while check variable */
        *pCompletion = CPA_FALSE;
        cbData.pOpaqueData = LAC_CONST_PTR_CAST(pCompletion);

        status = LacPke_SendSingleRequest(
            PKE_LIVENESS,
            inArgSizeList,
            outArgSizeList,
            &inArgList,
            &outArgList,
            internalMemIn,
            internalMemOut,
            LacPke_VerifyMmpCbFunc,
            &cbData,
            instanceHandle);
    }

    /* pend until the lock is unlocked by callback function */
    if (CPA_STATUS_SUCCESS == status)
    {
        pCryptoService = (sal_crypto_service_t *) instanceHandle;
        while ((CPA_FALSE == *pCompletion) && (count < LAC_PKE_WAIT_COUNT))
        {
            if(SAL_RESP_POLL_CFG_FILE == pCryptoService->isPolled)
            {
               status = icp_adf_pollInstance(
                   &(pCryptoService->trans_handle_asym_rx),
                   numTransHandles, quota);
               if((status != CPA_STATUS_RETRY) &&
                              (status != CPA_STATUS_SUCCESS))
               {
                   LAC_LOG_ERROR1("Polling MMP liveness message "
                                                  "returned %d\n", status);
                   break;
               }
            }
            osalSleep(LAC_PKE_DELAY_IN_MS);
            count ++;
        }

        if ((CPA_STATUS_SUCCESS == status) && (CPA_TRUE == *pCompletion))
        {
            LAC_OS_FREE(pCompletion);
            loadedFwId = pOutBufferData[PKE_FW_ID_IDX];
            LAC_OS_CAFREE(pOutBufferData);
            LAC_OS_CAFREE(internalMemOut);
            LAC_OS_CAFREE(internalMemIn);
        }
        else
        {
            /* The completion is not freed when then the timeout expires.
             * This results in a memory leak but it will prevent a system
             * crash. The callback may arrive at some stage in the future and
             * the callback function will try and access the memory for the
             * completion to set it to true. Therefore the memory must for the
             * completion must remain */
            LAC_LOG_ERROR("Timeout for MMP Liveness callback has expired");
            status = CPA_STATUS_FAIL;
            /* Free memory */
            LAC_OS_FREE(pCompletion);
            LAC_OS_CAFREE(pOutBufferData);
            LAC_OS_CAFREE(internalMemOut);
            LAC_OS_CAFREE(internalMemIn);
        }
    }
    else
    {
        LAC_OS_FREE(pCompletion);
        LAC_OS_CAFREE(pOutBufferData);
        LAC_OS_CAFREE(internalMemOut);
        LAC_OS_CAFREE(internalMemIn);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /*Assuming only one version of MMP lib present*/
        if (loadedFwId != PKE_INTERFACE_SIGNATURE)
        {
            LAC_LOG_ERROR2("Error in LAC initialisation. Compiled firmware "
                "version (0x%08X) does not match loaded firmware version "
                "(0x%08X)",PKE_INTERFACE_SIGNATURE, loadedFwId);
            status = CPA_STATUS_FAIL;
        }
    }
    return status;
}

CpaStatus
LacPke_Init(CpaInstanceHandle instanceHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    status = LacPke_VerifyMmpLib(instanceHandle);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Error in LAC initialisation. Cannot initialise PKE");
    }
    return status;
}
