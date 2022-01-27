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
 *****************************************************************************
 * @file cpa_nrbg_sample.c
 *
 * @ingroup sampleNrbgFunctional
 *
 * @description
 *     This is sample code that implements the DRBG-registered functions. This
 *     also implements the GetEntropy and HealthTest functions using NRGB API.
 *
 *****************************************************************************/
#include "cpa.h"
#include "cpa_cy_im.h"
#include "cpa_cy_nrbg.h"
#include "cpa_cy_drbg.h"

#include "icp_sal_nrbg_ht.h"
#include "icp_sal_drbg_impl.h"

#include "cpa_nrbg_sample.h"
#include "cpa_sample_utils.h"


/* External variable used in print debug macros */
extern int gDebugParam ;

/**
 *****************************************************************************
 * @ingroup sampleNrbgFunctional
 *      NRBG Buffer structure
 *
 * @description
 *      Buffer structure used to store information for a particular NRBG
 *      GetEntropy operation. This includes NRBG OpData structure, and
 *      pointers to client supplied callback function, callback tag and
 *      opData.
 *
 *****************************************************************************/
typedef struct nrbg_sample_data_s {
        CpaCyNrbgOpData opData;
        /* NRBG client information */
        IcpSalDrbgGetEntropyInputCbFunc pClientCbFunc;
        void *pClientCallbackTag;
        void *pClientOpData;
} nrbg_sample_data_t;

/* Instance handle */
CpaInstanceHandle cyInstHandle = CPA_INSTANCE_HANDLE_SINGLE;

/* Previously set GetEntropy function pointer */
static IcpSalDrbgGetEntropyInputFunc pPrevGetEntropyInputFunc = NULL;

/* Previously set GetNonce function pointer */
static IcpSalDrbgGetNonceFunc pPrevGetNonceFunc = NULL;

/* Previously set IsDFReq function pointer */
static IcpSalDrbgIsDFReqFunc pPrevDrbgIsDFReqFunc = NULL;

/**
 *****************************************************************************
 * @ingroup sampleNrbgFunctional
 *      NRBG Internal Callback Function
 *
 * @description
 *      This is an internal callback function that will be used to signal
 *      the caller that the asynchronous operation is completed. The signal
 *      is performed by calling the client-supplied callback function.
 *
 * @see
 *      IcpSalDrbgGetEntropyInputCbFunc
 *
 *****************************************************************************/
static void
nrbgCallback(void *pCallbackTag,
        CpaStatus status,
        void *pOpdata,
        CpaFlatBuffer *pOut)
{
    nrbg_sample_data_t *pNrbgData = NULL;
    IcpSalDrbgGetEntropyInputCbFunc pClientCb = NULL;
    void *pClientCallbackTag = NULL;
    void *pClientOpData = NULL;
    Cpa32U lengthReturned = 0;

    PRINT_DBG("%s\n", __FUNCTION__);

    if (NULL == pCallbackTag)
    {
        PRINT_ERR("pCallbackTag is null");
        return;
    }

    pNrbgData = (nrbg_sample_data_t *)pCallbackTag;

    if (CPA_STATUS_SUCCESS == status)
    {
        lengthReturned = pNrbgData->opData.lengthInBytes;
        PRINT_DBG("lengthReturned = %u\n", lengthReturned);
    }

    pClientCb = pNrbgData->pClientCbFunc;
    pClientCallbackTag = pNrbgData->pClientCallbackTag;
    pClientOpData = pNrbgData->pClientOpData;

    OS_FREE(pNrbgData);

    pClientCb(pClientCallbackTag, status, pClientOpData, lengthReturned, pOut);
}

/**
 *****************************************************************************
 * @ingroup sampleNrbgFunctional
 *      NRBG Get Entropy Function
 *
 * @description
 *      This function implements the DRBG implementation-specific
 *      'Get Entropy Input' function by calling cpaCyNrbgGetEntropy API.
 *
 * @see
 *     IcpSalDrbgGetEntropyInputFunc
 *
 *****************************************************************************/
static CpaStatus
nrbgGetEntropy(IcpSalDrbgGetEntropyInputCbFunc pCb,
        void * pCallbackTag,
        icp_sal_drbg_get_entropy_op_data_t *pOpData,
        CpaFlatBuffer *pBuffer,
        Cpa32U *pLengthReturned)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaCyGenFlatBufCbFunc pNrbgCbFunc = NULL;
    nrbg_sample_data_t *pNrbgData = NULL;
    CpaInstanceHandle instanceHandle = CPA_INSTANCE_HANDLE_SINGLE; 
    CpaCyCapabilitiesInfo cap = {0};

    /* For now use the first crypto instance - assumed to be 
       started already */
    sampleCyGetInstance(&instanceHandle);
    if (instanceHandle == NULL)
    {
        return CPA_STATUS_FAIL;
    }
    /* Verify that the instance has NRBG capabilities */
    status = cpaCyQueryCapabilities(instanceHandle, &cap);
    if(CPA_STATUS_SUCCESS != status)
    {
        return status;
    }
    if(cap.nrbgSupported != CPA_TRUE)
    {
       PRINT_ERR("Instance does not support NRBG\n");
       return CPA_STATUS_RESOURCE;
    }   

    PRINT_DBG("%s\n", __FUNCTION__);

    if (NULL == pOpData)
    {
        PRINT_ERR("Invalid parameter -- pOpData\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    if ((NULL == pLengthReturned) && (NULL == pCb))
    {
        PRINT_ERR("Invalid parameter -- pLengthReturned\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    status = OS_MALLOC(&pNrbgData, sizeof(nrbg_sample_data_t));
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("Failed to allocate pNrbgData. (status = %d)\n", status);
        return CPA_STATUS_FAIL;
    }

    /* number of bytes to be generated */
    pNrbgData->opData.lengthInBytes = pOpData->maxLength;

    /* store client information */
    pNrbgData->pClientCbFunc = pCb;
    pNrbgData->pClientCallbackTag = pCallbackTag;
    pNrbgData->pClientOpData = (void *)pOpData;

    /* use local callback function on asynchronous operation */
    if (NULL != pCb)
    {
        pNrbgCbFunc = nrbgCallback;
    }
    do {
        status = cpaCyNrbgGetEntropy(instanceHandle,
                        pNrbgCbFunc, pNrbgData, &(pNrbgData->opData), pBuffer);
    }
    while (CPA_STATUS_RETRY == status);

    if (CPA_STATUS_SUCCESS != status)
    {
        status = (CPA_STATUS_INVALID_PARAM == status)? status: CPA_STATUS_FAIL;
        PRINT_ERR("cpaCyNrbgGetEntropy failed. (status = %d)\n", status);
        OS_FREE(pNrbgData);
        return status;
    }

    if (NULL == pCb)
    {
        *pLengthReturned = pNrbgData->opData.lengthInBytes;
        OS_FREE(pNrbgData);
    }

    return CPA_STATUS_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup sampleNrbgFunctional
 *      NRBG Get Nonce Function
 *
 * @description
 *      This function implements the DRBG implementation-specific
 *      'Get Nonce' function by calling the nrbgGetEntropy() function.
 *
 * @see
 *      IcpSalDrbgGetNonceFunc
 *
 *****************************************************************************/
static CpaStatus
nrbgGetNonce(icp_sal_drbg_get_entropy_op_data_t *pOpData,
        CpaFlatBuffer *pBuffer,
        Cpa32U *pLengthReturned)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    PRINT_DBG("%s\n", __FUNCTION__);

    status = nrbgGetEntropy(NULL, NULL, pOpData, pBuffer, pLengthReturned);
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("nrbgGetEntropy failed. (status = %d)\n", status);
    }

    return status;
}

/**
 *****************************************************************************
 * @ingroup sampleNrbgFunctional
 *      NRBG Is Derivation Function(DF) Required Function
 *
 * @description
 *      This function implements the DRBG implementation-specific
 *      'Is Derivation Function Required' function.
 *
 * @see
 *      IcpSalDrbgIsDFReqFunc
 *
 *****************************************************************************/
static CpaBoolean
nrbgIsDFRequired(void)
{
    PRINT_DBG("%s\n", __FUNCTION__);
    return CPA_FALSE;
}

void
nrbgRegisterDrbgImplFunctions(void)
{
    PRINT_DBG("%s\n", __FUNCTION__);

    pPrevGetEntropyInputFunc =
        icp_sal_drbgGetEntropyInputFuncRegister(nrbgGetEntropy);

    pPrevGetNonceFunc =
        icp_sal_drbgGetNonceFuncRegister(nrbgGetNonce);

    pPrevDrbgIsDFReqFunc =
        icp_sal_drbgIsDFReqFuncRegister(nrbgIsDFRequired);
}

void
nrbgUnregisterDrbgImplFunctions(void)
{
    PRINT_DBG("%s\n", __FUNCTION__);

    icp_sal_drbgGetEntropyInputFuncRegister(pPrevGetEntropyInputFunc);

    icp_sal_drbgGetNonceFuncRegister(pPrevGetNonceFunc);

    icp_sal_drbgIsDFReqFuncRegister(pPrevDrbgIsDFReqFunc);
}

/**
 * nrbgSampleStart
 */
CpaStatus
nrbgSampleStart(void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U continousRngTestFailures = 0;

    PRINT_DBG("%s\n", __FUNCTION__);
    /*
     * In this simplified version of instance discovery, we discover
     * exactly one instance of a crypto service.
     */
    sampleCyGetInstance(&cyInstHandle);
    if (cyInstHandle == NULL)
    {
        return CPA_STATUS_FAIL;
    }

    /* Start Cryptographic component */
    PRINT_DBG("cpaCyStartInstance\n");
    status = cpaCyStartInstance(cyInstHandle);

    if(CPA_STATUS_SUCCESS == status)
    {
       /*
        * Set the address translation function for the instance
        */
        status = cpaCySetAddressTranslation(cyInstHandle, sampleVirtToPhys);
    }

    if(CPA_STATUS_SUCCESS == status)
    {
       /*
        * If the instance is polled start the polling thread. Note that
        * how the polling is done is implementation-dependant.
        */
       sampleCyStartPolling(cyInstHandle);


       status = icp_sal_nrbgHealthTest(cyInstHandle, &continousRngTestFailures);
       if (CPA_STATUS_SUCCESS != status)
       {
          PRINT_ERR("icp_sal_nrbgHealthTest failed. (status = %d)\n", status);
          cpaCyStopInstance(cyInstHandle);
          return CPA_STATUS_FAIL;
       }
    }

    /*
     * In this sample code, continousRngTestFailures is printed for
     * debug purposes only. Refer to icp_sal_nrbgHealthTest API for
     * more details about this counter.
     */
    PRINT_DBG("continousRngTestFailures = %d\n", continousRngTestFailures);

    if((CPA_STATUS_SUCCESS == status) && (continousRngTestFailures == 0))
    {
       nrbgRegisterDrbgImplFunctions();
       PRINT_DBG("Successfully registered DRBG entropy functions\n");
    }

    return status;
}

/**
 * nrbgSampleStop
 */
void
nrbgSampleStop(void)
{
    PRINT_DBG("%s\n", __FUNCTION__);

    nrbgUnregisterDrbgImplFunctions();

    /* Stop the polling thread */
    sampleCyStopPolling();

    PRINT_DBG("cpaCyStopInstance\n");
    cpaCyStopInstance(cyInstHandle);
}
