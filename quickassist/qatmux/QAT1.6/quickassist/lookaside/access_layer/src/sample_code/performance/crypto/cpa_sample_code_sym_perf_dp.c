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
 *  version: QAT1.6.L.2.6.0-65
 *
 ***************************************************************************/


/*****************************************************************************
 * @file cpa_sample_code_sym_perf_dp.c
 *
 * @defgroup sampleSymmetricDpPerf  Symmetric Data Plane Performance code
 *
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 *      This file contains the main symmetric Data Plane performance sample
 *      code.
 *      It is capable of performing all ciphers, all hashes, authenticated
 *      hashes and algorithm chaining. Nested Hashes are not supported
 *
 *      setup->numBuffLists, each bufferlist includes several flat buffers which
 *      its size is equal to buffer size. The pre-allocated buffers are then
 *      continuously looped until the numLoops is met.
 *      Time stamping is started prior to the
 *      Operation and is stopped when all callbacks have returned.
 *      The packet size and algorithm to be tested is setup using the
 *      setupSymmetricDpTest function. The framework is used to create the
 *      threads which calls functions here to execute symmetric data plane
 *      performance.
 *****************************************************************************/

#include "cpa_sample_code_sym_perf_dp.h"
#include "icp_sal_poll.h"

#ifdef WITH_CPA_MUX
#include "cpa_impl_mux.h"
#endif


#define REL_LOOP_MULTIPLIER                     (2)
#define SYM_OPERATIONS_DEFAULT_POLLING_INTERVAL         (16)
Cpa32U symPollingInterval_g = SYM_OPERATIONS_DEFAULT_POLLING_INTERVAL;
extern CpaBoolean digestAppended_g;
extern int signOfLife;

extern Cpa32U packageIdCount_g;
/*****************************************************************************
 *
 *  Internal Function Interfaces
 *
 *****************************************************************************/

 /*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * Set the polling interval. The Polling interval is the number of succesful
 * submissions before the driver is polled for responses
 * ***************************************************************************/
 CpaStatus setSymPollingInterval(Cpa64U pollingInterval)
 {
    symPollingInterval_g = pollingInterval;
    return CPA_STATUS_SUCCESS;
 }
 EXPORT_SYMBOL(setSymPollingInterval);



/*****************************************************************************
* @ingroup sampleSymmetricDpPerf
*
* @description
* print the digestAppend flag state
* ***************************************************************************/
CpaStatus printDigestAppend(CpaBoolean flag)
{
   PRINT("digestAppended_g %d\n",digestAppended_g);
   return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(printDigestAppend);
 /*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * Print the polling interval. The Polling interval is the number
 * of succesful submissions before the driver is polled for responses
 * ***************************************************************************/
 CpaStatus printSymPollingInterval(void)
 {
    PRINT("Symmetric Polling Interval: %u\n", symPollingInterval_g);
    return CPA_STATUS_SUCCESS;
 }
 EXPORT_SYMBOL(printSymPollingInterval);


 /*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * Poll the number of DP operations
 * ***************************************************************************/
CpaStatus cyDpPollNumOperations(perf_data_t *pPerfData,
                CpaInstanceHandle instanceHandle, Cpa64U numOperations)
{
    CpaStatus status = CPA_STATUS_FAIL;

    perf_cycles_t startCycles = 0, totalCycles = 0;
    Cpa32U freq = sampleCodeGetCpuFreq();
    startCycles = sampleCodeTimestamp();
    while(pPerfData->responses != numOperations)
    {
        status = icp_sal_CyPollDpInstance(instanceHandle,0);
        if(CPA_STATUS_FAIL == status)
        {
            PRINT_ERR("Error polling instance\n");
            return CPA_STATUS_FAIL;
        }
        if(CPA_STATUS_RETRY == status)
        {
            AVOID_SOFTLOCKUP;
        }
        totalCycles = (sampleCodeTimestamp() - startCycles);
        if(totalCycles > 0)
        {
            do_div(totalCycles,freq);
        }

        if(totalCycles > SAMPLE_CODE_WAIT_THIRTY_SEC)
        {
            PRINT_ERR("Timeout on polling remaining Operations\n");
            return status;
        }
    }
    return CPA_STATUS_SUCCESS;
}


/**
 *****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 *      calculate the pointer of digest result in the buffer list
 *      digest result should be located in the end of Plaintext and
 *      digest result should be align with block size of cipher.
 *      Please see example as the following:
@verbatim
+--------+----------------------------------------------------+--------+----+
|        |                   Ciphertext                       | Digest |Pad +
+--------+----------------------------------------------------+--------+----+
         <-FlatBuffer[0]-><-FlatBuffer[1]-><-FlatBuffer[2]-><-FlatBuffer[3]->
         <-                               Buffer List                      ->
@endverbatim
 * @param[in] packetSize       Data packet size
 * @param[in] bufferSizeInByte buffer size in the flatbuffer of bufferlist
 * @param[in] blockSizeInBytes min block length of the cipher, Digest need to
 *                             align with it.
 * @param[in] pBufferList      the pointer of Buffer list which store data and
 *                             comprised of flatbuffers.
 *
 *****************************************************************************/
static CpaPhysicalAddr symDpCalDigestAddress(
        Cpa32U packetSize,
        Cpa32U bufferSizeInByte,
        Cpa32U blockSizeInBytes,
        CpaPhysBufferList *pBufferList)
{
      Cpa32U packsetSizePad = 0;
      Cpa32U digestOffset = 0;
      CpaPhysicalAddr pDigestResult=0;
      Cpa32U indexBuffer = 0;

      /* check if  packetSize is 0  */
      if (bufferSizeInByte == 0)
      {
         pDigestResult = (CpaPhysicalAddr)(SAMPLE_CODE_UINT)
                         (pBufferList->flatBuffers[0].
                         bufferPhysAddr + packetSize);
      }
      else
      {
        /* since Digest address (pDigestResult) need to align with
        * blockSizeInBytes, we will check if packetSize is align with
        * blockSizeInBytes,
        * if not, padding will added after message */
        if(packetSize % blockSizeInBytes != 0)
        {
           packsetSizePad = blockSizeInBytes - (packetSize % blockSizeInBytes);
        }
        /* calculate actual offset of digest result in flatbuffer*/
        digestOffset = (packetSize + packsetSizePad)% bufferSizeInByte;

        /* calculate the which flat buffer store pDigestResult
        * pDigestResult will appended in the end of pData */
        indexBuffer = (packetSize
                      + packsetSizePad)/bufferSizeInByte;
        pDigestResult = (CpaPhysicalAddr)(SAMPLE_CODE_UINT)(
                      pBufferList->flatBuffers[indexBuffer].bufferPhysAddr + digestOffset);
      }
      return pDigestResult;
}
/**
 *****************************************************************************
 * @ingroup sampleSymmetricPerf
 *
 * @description
 *      initialize the digest buffer
 *
 *****************************************************************************/
static void symDpSetDigestBuffer(
        Cpa32U messageLenToCipherInBytes,
        Cpa32U digestLengthInBytes,
        Cpa8U value,
        CpaBufferList *pBufferList)
{
      Cpa8U * pDigestResult = 0;
      Cpa32U indexBuffer = 0;
      Cpa32U i = 0;
      Cpa32U bufferSizeInByte = pBufferList->pBuffers[0].dataLenInBytes;
      /*  all the rest of data including padding will initialized ,
      * so ivLenInBytes is 1.*/
      if (bufferSizeInByte == 0)
      {
         pDigestResult = (Cpa8U *)(pBufferList->pBuffers[0].
                         pData + messageLenToCipherInBytes);
      }
      else
      {
          /* calculate the which flat buffer store pDigestResult
           * pDigestResult will appended in the end of pData */
          indexBuffer = messageLenToCipherInBytes/bufferSizeInByte;
          pDigestResult = (Cpa8U *)(pBufferList->pBuffers[indexBuffer].pData
                  + (messageLenToCipherInBytes % bufferSizeInByte));
      }

      /* reset the digest memory to 0 */

      memset((void *)pDigestResult,
           value,
           (pBufferList->pBuffers[0].dataLenInBytes -
           messageLenToCipherInBytes%pBufferList->pBuffers[0].dataLenInBytes)
           );

      indexBuffer ++;
      for (i = indexBuffer ; i< pBufferList->numBuffers; i++)
      {
        memset((void *)(SAMPLE_CODE_UINT)pBufferList->pBuffers[i].pData,
           value,
           pBufferList->pBuffers[i].dataLenInBytes
           );
      }

}
/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * Free memory allocated in the symmetricPerformOpDataSetup function
 * ***************************************************************************/

static void dpOpDataMemFree(CpaCySymDpOpData *pOpdata[], Cpa32U numBuffers)
{
    Cpa32U k = 0;

    for(k = 0; k < numBuffers; k++)
    {
        if(NULL != pOpdata[k])
        {
            if(NULL != pOpdata[k]->pIv)
            {
                qaeMemFreeNUMA((void**)&pOpdata[k]->pIv);
            }
            if(NULL != pOpdata[k]->pAdditionalAuthData)
            {
                qaeMemFreeNUMA((void**)&pOpdata[k]->pAdditionalAuthData);
            }
            qaeMemFreeNUMA((void**)&pOpdata[k]);
        }/*end of if(NULL != pOpdata[k]) */
    }/* End of main loop */
}

/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * Free memory allocated in the sampleSymmetricDpPerform function
 * ***************************************************************************/
static void symDpPerformMemFree(symmetric_test_params_t* setup,
        CpaBufferList **ppSrcBuffListArray,
        CpaPhysBufferList **ppSrcPyhsBuffListArray,
        CpaCySymDpOpData **ppOpData,
        CpaCySymDpSessionCtx* pSessionCtx,
        Cpa8U *authKey,
        Cpa8U *cipherKey,
        Cpa32U *packetSize)
{
    Cpa32U m = 0;

    /*free bufferLists, flatBuffers and data*/

    if(NULL != ppOpData)
    {
        dpOpDataMemFree(ppOpData, setup->numBuffLists);
        qaeMemFreeNUMA((void**)&ppOpData);
    }
    /* free session memory - calling code is responsible for
     * removing the sessions */
    if (NULL != pSessionCtx)
    {
        if(NULL != *pSessionCtx)
        {
            for (m = 0; m < setup->numSessions; m++)
            {
                qaeMemFreeNUMA((void**)&pSessionCtx[m]);
            }

        }
        qaeMemFree((void**)&pSessionCtx);
    }
    /* Free session array , Operation Data, buffer List array*/
    if(NULL != cipherKey)
    {
        qaeMemFree((void**)&cipherKey);
    }
    if (NULL != authKey)
    {
        qaeMemFree((void**)&authKey);
    }
    if (NULL != packetSize)
    {
        qaeMemFree((void**)&packetSize);
    }

    if (NULL != ppSrcBuffListArray)
    {
        dpSampleFreeBuffers(ppSrcBuffListArray, ppSrcPyhsBuffListArray,
             setup->numBuffLists, ppSrcBuffListArray[0]->numBuffers);
        qaeMemFreeNUMA((void**)&ppSrcBuffListArray);
    }
    if (NULL != ppSrcPyhsBuffListArray)
    {
        qaeMemFreeNUMA((void**)&ppSrcPyhsBuffListArray);
    }

}


/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 *      Callback function for symmetric perform operation.
 *      Response will increased 1 after each symmetric operation finished.
 *      When the number of response is equal to the number of operations,
 *      it will release one semaphore for async operation.
 *****************************************************************************/
void symDpPerformCallback(
        CpaCySymDpOpData *pOpData,
        CpaStatus status,
        CpaBoolean verifyResult)
{
    /* pCallbacktag in the pOpData structure is used to store
     * index of to the perf_data_t associated the thread
     * */
    perf_data_t * pPerfData = pOpData->pCallbackTag;
    pPerfData->responses++;
#ifdef LATENCY_CODE
    if((0 != pPerfData->responses) && (NULL != pPerfData->response_times))
    {
        if(pPerfData->responses ==  pPerfData->nextCount)
        {
            pPerfData->response_times[pPerfData->latencyCount++] =
                sampleCodeTimestamp();
            pPerfData->nextCount += pPerfData->countIncrement;
        }
    }
#endif    /*if we have received the pre-set numOperations, then get the clock cycle
     * as a timestamp and post the Semaphore to release parent thread*/
    if (pPerfData->numOperations == pPerfData->responses)
    {
        pPerfData->endCyclesTimestamp=sampleCodeTimestamp();
    }
}

/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * Create a symmetric session
 *
 *****************************************************************************/
static CpaStatus symmetricDpSetupSession(
        CpaCySymDpCbFunc pSymCb,
        Cpa8U *pCipherKey,
        Cpa8U *pAuthKey,
        CpaCySymDpSessionCtx *pSession,
        CpaBoolean digestVerify,
        CpaBoolean digestIsEncrypted,
        symmetric_test_params_t *setup
)
{
    Cpa32U sessionCtxSizeInBytes = 0;
#if CPA_CY_API_VERSION_NUM_MINOR >= 8
    Cpa32U sessionCtxDynamicSizeInBytes = 0;
#endif
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaCySymDpSessionCtx pLocalSession = NULL;
    Cpa32U cipherKeyLen = 0;
    Cpa32U authKeyLen = 0;
    Cpa32U node = 0;

    status = sampleCodeCyGetNode(setup->cyInstanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("sampleCodeCyGetNode error, status: %d\n", status);
        return status;
    }
    /*set the cipher and authentication key len*/
    cipherKeyLen = setup->setupData.cipherSetupData.cipherKeyLenInBytes;
    authKeyLen =
        setup->setupData.hashSetupData.authModeSetupData.authKeyLenInBytes;
    /*generate a random cipher and authentication key*/
    generateRandomData(pCipherKey,cipherKeyLen);
    generateRandomData(pAuthKey,authKeyLen);
    /*cipher setup only needs to be set for alg chaining, cipher, AES-GCM
     * and AES-CCM*/
    setup->setupData.cipherSetupData.pCipherKey = pCipherKey;
    /*hash setup only needs to be set for hash, AES-GCM
     * and AES-CCM*/
    setup->setupData.hashSetupData.authModeSetupData.authKey = pAuthKey;

    /*hash algorithm initialization*/
    if(CPA_CY_SYM_HASH_AES_GMAC == setup->setupData.hashSetupData.hashAlgorithm)
    {
        setup->setupData.hashSetupData.authModeSetupData.authKey = NULL;
        setup->setupData.hashSetupData.authModeSetupData.authKeyLenInBytes = 0;
    }
    else if(CPA_CY_SYM_HASH_SNOW3G_UIA2 ==
            setup->setupData.hashSetupData.hashAlgorithm
#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#endif
    )
    {
        setup->setupData.hashSetupData.authModeSetupData.aadLenInBytes =
            KEY_SIZE_128_IN_BYTES;
    }
    else
    {
        /* For AES-CCM, AAD(additional auth data) is optional.*/
        setup->setupData.hashSetupData.authModeSetupData.aadLenInBytes = 0;
    }
    setup->setupData.verifyDigest = CPA_FALSE;

    /*this is the original API to get the required context size, we show the
     * function used here, but we only use the size for the older version of the
     * API get size for memory allocation*/
    status = cpaCySymDpSessionCtxGetSize( setup->cyInstanceHandle,
            &setup->setupData, &sessionCtxSizeInBytes);
    if(status != CPA_STATUS_SUCCESS)
    {
        PRINT_ERR("cpaCySymSessionCtxGetSize error, status: %d", status);
        return status;
    }

#if CPA_CY_API_VERSION_NUM_MINOR >= 8
    /*get dynamic context size*/

    status = cpaCySymDpSessionCtxGetDynamicSize( setup->cyInstanceHandle,
            &setup->setupData, &sessionCtxDynamicSizeInBytes);
    if(status != CPA_STATUS_SUCCESS)
    {
        PRINT_ERR("cpaCySymSessionCtxGetDynamicSize error, status: %d", status);
        return status;
    }

    /*
     * allocate session memory using dynamic context size
     */
    sessionCtxSizeInBytes = sessionCtxDynamicSizeInBytes;
#endif

    pLocalSession = qaeMemAllocNUMA(sessionCtxSizeInBytes,
            node, BYTE_ALIGNMENT_64);
    if(NULL == pLocalSession)
    {
        PRINT_ERR("Could not allocate pLocalSession memory\n");
        return CPA_STATUS_FAIL;
    }
    /*zero session memory*/
    memset(pLocalSession, 0, sessionCtxSizeInBytes);
    /*
     * init session - pLocalSession will contain the session context
     */
    status = cpaCySymDpInitSession(setup->cyInstanceHandle,
                  &setup->setupData,
                  pLocalSession);

    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCySymInitSession error, status: %d\n", status);
        qaeMemFreeNUMA((void**)&pLocalSession);
        return status;
    }
    *pSession = pLocalSession;


#if CPA_CY_API_VERSION_NUM_MINOR >= 8
#endif


    /* Register asynchronous callback with instance handle*/
    status = cpaCySymDpRegCbFunc(setup->cyInstanceHandle, pSymCb);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCySymDpRegCbFunc error, status: %d\n", status);
        qaeMemFreeNUMA((void**)&pLocalSession);
        return status;
    }

    return status;
}

/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * Setup symmetric operation data
 * ***************************************************************************/
static CpaStatus symmetricDpPerformOpDataSetup(
        CpaCySymDpSessionCtx* pSessionCtx,
        Cpa32U *pPacketSize,
        CpaCySymDpOpData *pOpdata[],
        symmetric_test_params_t *setup,
        CpaPhysBufferList *ppSrcBuffListArray[],
        CpaPhysBufferList *ppDestBuffListArray[])
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U createCount = 0;
    Cpa32U node = 0;

    /*get the node we are running on for local memory allocation*/
    status = sampleCodeCyGetNode(setup->cyInstanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("sampleCodeCyGetNode error, status: %d\n", status);
        return status;
    }
    /*for each bufferList set the symmetric operation data*/
    for (createCount = 0; createCount < setup->numBuffLists; createCount++)
    {
        /* allocate op data structure for each entry*/
        pOpdata[createCount]=qaeMemAllocNUMA(sizeof(CpaCySymDpOpData),
                node,BYTE_ALIGNMENT_64);
        if(pOpdata[createCount] == NULL)
        {
            PRINT_ERR("Could not allocte Opdata memory at index %u\n",
                    createCount);
            dpOpDataMemFree(pOpdata, createCount);
            return CPA_STATUS_FAIL;
        }
        /* Zero initialize Op data structure*/
        memset(pOpdata[createCount], 0, sizeof(CpaCySymDpOpData));
        /*
        * Initialize Op data structure with different sessions :
        * multiple session
        */
        pOpdata[createCount]->sessionCtx =
        pSessionCtx[createCount % setup->numSessions];

        /* Starting point for cipher processing */
        pOpdata[createCount]->cryptoStartSrcOffsetInBytes =
                    setup->cryptoSrcOffset;

        /* messageLenToCipherInBytes and messageLenToHashInBytes do not have
         * to be the same. In this code we want to either hash the entire buffer
         * or encrypt the entire buffer, depending on the SymOperation.
         * For Alg Chaining, depending on the chain order, for HashThenCipher,
         * the digest will be the hash of the unencrypted buffer and then we
         * cipher  the buffer. OR for CipherThenHash, we cipher the buffer, then
         * the perform the hash on the encrypted buffer, so that the digest is
         * the digest of the encrypted data*/

        pOpdata[createCount]->messageLenToCipherInBytes =
                    pPacketSize[createCount]-setup->cryptoSrcOffset;

        /* Starting point for hash processing */
        pOpdata[createCount]->hashStartSrcOffsetInBytes = HASH_OFFSET_BYTES;
        pOpdata[createCount]->messageLenToHashInBytes =
            pPacketSize[createCount];
        pOpdata[createCount]->pAdditionalAuthData = NULL;

        /* In GMAC mode, there is no message to Cipher */
        if(CPA_CY_SYM_HASH_AES_GMAC ==
                setup->setupData.hashSetupData.hashAlgorithm)
        {
            pOpdata[createCount]->cryptoStartSrcOffsetInBytes = 0;
            pOpdata[createCount]->messageLenToCipherInBytes = 0;
        }
        /*these only need to be set for HASH_SNOW3G_UIA2*/
        if(CPA_CY_SYM_HASH_SNOW3G_UIA2 ==
            setup->setupData.hashSetupData.hashAlgorithm
#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#endif
         )
        {

            pOpdata[createCount]->pAdditionalAuthData =
                qaeMemAllocNUMA(KEY_SIZE_128_IN_BYTES ,node,BYTE_ALIGNMENT_64);
            if(NULL == pOpdata[createCount]->pAdditionalAuthData)
            {
                PRINT_ERR("Could not allocate additional auth data index %d\n",
                        createCount);
                dpOpDataMemFree(pOpdata, createCount);
                return CPA_STATUS_FAIL;
            }
            memset(pOpdata[createCount]->pAdditionalAuthData,
                    SYM_AUTH_INIT_VALUE, KEY_SIZE_128_IN_BYTES);
        }/*End of if(CPA_CY_SYM_HASH_SNOW3G_UIA2 == ... */
        else if(setup->setupData.cipherSetupData.cipherAlgorithm ==
                    CPA_CY_SYM_CIPHER_AES_CCM)
        {
            /*must allocate to the nearest block size required
              (above 18 bytes)*/
            pOpdata[createCount]->pAdditionalAuthData =
                qaeMemAllocNUMA(AES_CCM_MIN_AAD_ALLOC_LENGTH,
                                node,BYTE_ALIGNMENT_64);
            if(NULL == pOpdata[createCount]->pAdditionalAuthData)
            {
                PRINT_ERR("Could not allocate additional auth data index %u\n",
                        createCount);
                dpOpDataMemFree(pOpdata, createCount);
                return CPA_STATUS_FAIL;
            }
            memset(pOpdata[createCount]->pAdditionalAuthData,
                    0, AES_CCM_MIN_AAD_ALLOC_LENGTH);
        }/* End of else if(setup->setupData.cipherSetupData.cipherAlgorithm .*/

        /* in this code we always allocate the IV, but it is not always used
         * this reduces the if/else logic when trying to cater for as many
         * symmetric crypto options as possible*/
        pOpdata[createCount]->ivLenInBytes = IV_LEN_FOR_8_BYTE_BLOCK_CIPHER;
        /*set IV len depending on what we are testing*/
        if(setup->setupData.cipherSetupData.cipherAlgorithm ==
            CPA_CY_SYM_CIPHER_AES_CBC ||
            setup->setupData.cipherSetupData.cipherAlgorithm ==
                CPA_CY_SYM_CIPHER_AES_CTR ||
            setup->setupData.cipherSetupData.cipherAlgorithm ==
                CPA_CY_SYM_CIPHER_AES_CCM ||
            setup->setupData.cipherSetupData.cipherAlgorithm ==
                CPA_CY_SYM_CIPHER_SNOW3G_UEA2 ||
            setup->setupData.cipherSetupData.cipherAlgorithm ==
                CPA_CY_SYM_CIPHER_AES_F8||
            setup->setupData.cipherSetupData.cipherAlgorithm ==
                CPA_CY_SYM_CIPHER_AES_XTS
#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#endif
                                                         )
        {
            pOpdata[createCount]->ivLenInBytes =
                IV_LEN_FOR_16_BYTE_BLOCK_CIPHER;
        }
        else if(setup->setupData.cipherSetupData.cipherAlgorithm ==
                        CPA_CY_SYM_CIPHER_AES_GCM)
        {
            pOpdata[createCount]->ivLenInBytes = IV_LEN_FOR_12_BYTE_GCM;
        }

        /*allocate NUMA aware aligned memory for IV*/
        pOpdata[createCount]->pIv = qaeMemAllocNUMA(
                pOpdata[createCount]->ivLenInBytes, node, BYTE_ALIGNMENT_64);
        if ( NULL == pOpdata[createCount]->pIv )
        {
            PRINT_ERR("IV is null\n");
            dpOpDataMemFree(pOpdata, createCount);
            return CPA_STATUS_FAIL;
        }
        memset(pOpdata[createCount]->pIv, 0,
                pOpdata[createCount]->ivLenInBytes);
        if(setup->setupData.cipherSetupData.cipherAlgorithm ==
                    CPA_CY_SYM_CIPHER_AES_CCM)
        {
            /*Although the IV data length for CCM must be 16 bytes,
              The nonce length must be between 7 and 13 inclusive*/
            pOpdata[createCount]->ivLenInBytes = AES_CCM_DEFAULT_NONCE_LENGTH;
        }

        /*if we are testing HASH or Alg Chaining, set the location to place
         * the digest result, this space was allocated in
         * sampleSymmetricDpPerform function*/
        if(setup->setupData.symOperation==CPA_CY_SYM_OP_HASH ||
                setup->setupData.symOperation==CPA_CY_SYM_OP_ALGORITHM_CHAINING)
        {
            pOpdata[createCount]->digestResult = symDpCalDigestAddress(
                   pPacketSize[createCount],
                   setup->flatBufferSizeInBytes,
                   IV_LEN_FOR_16_BYTE_BLOCK_CIPHER,
                   ppSrcBuffListArray[createCount]
            );
        }

        if(setup->setupData.cipherSetupData.cipherAlgorithm ==
                    CPA_CY_SYM_CIPHER_AES_CCM)
        {
            /*digestAppend is always true for CCM*/
            pOpdata[createCount]->digestResult = (CpaPhysicalAddr)CPA_TRUE;
            /*generate a random IV*/
            generateRandomData(&(pOpdata[createCount]->pIv[SYM_DP_LV_OFFSET_CCM]),
                   pOpdata[createCount]->ivLenInBytes);

          /*API requires copying of the nonce to the first byte of the AAD*/
          memcpy(&(pOpdata[createCount]->
                      pAdditionalAuthData[SYM_DP_LV_OFFSET_CCM]),
                 &(pOpdata[createCount]->pIv[SYM_DP_LV_OFFSET_CCM]),
                 pOpdata[createCount]->ivLenInBytes);
        }
        else
        {
            /*generate a random IV*/
            generateRandomData(pOpdata[createCount]->pIv,
                    pOpdata[createCount]->ivLenInBytes);
        }
        /* initialize iv structure */
        if (0 != pOpdata[createCount]->ivLenInBytes)
        {
            pOpdata[createCount]->iv = (CpaPhysicalAddr)(SAMPLE_CODE_UINT)
                    qaeVirtToPhysNUMA(pOpdata[createCount]->pIv);
        }
        else
        {
            pOpdata[createCount]->iv = (CpaPhysicalAddr)(SAMPLE_CODE_UINT)NULL;
        }
        /* the physical address of additionalAuthData */
        if(NULL != pOpdata[createCount]->pAdditionalAuthData)
        {
            pOpdata[createCount]->additionalAuthData=
               (CpaPhysicalAddr)(SAMPLE_CODE_UINT)
               qaeVirtToPhysNUMA(pOpdata[createCount]->pAdditionalAuthData);
        }
        else
        {
            pOpdata[createCount]->additionalAuthData=
                           (CpaPhysicalAddr)(SAMPLE_CODE_UINT)NULL;
        }

        /* if No buffer size specified, default value is flat buffers
        * (no buffer chaining) */
        if (0 == setup->flatBufferSizeInBytes)
        {
            /* Set physical address for srcbuffer */
            pOpdata[createCount]->srcBuffer = ppSrcBuffListArray[createCount]->
                 flatBuffers->bufferPhysAddr;
            /* Set length of srcbuffer */
            pOpdata[createCount]->srcBufferLen=ppSrcBuffListArray[createCount]->
                 flatBuffers->dataLenInBytes;
            /* Set physcial address for dstBuffer */
            pOpdata[createCount]->dstBuffer=ppDestBuffListArray[createCount]->
                 flatBuffers->bufferPhysAddr;
            /* Set length of dstBuffer */
            pOpdata[createCount]->dstBufferLen=
                ppDestBuffListArray[createCount]->flatBuffers->dataLenInBytes;
            if(setup->setupData.digestIsAppended == CPA_FALSE)
            {
                if(setup->setupData.symOperation==CPA_CY_SYM_OP_ALGORITHM_CHAINING)
                {
                    pOpdata[createCount]->srcBufferLen -=
                        setup->setupData.hashSetupData.digestResultLenInBytes;
                    pOpdata[createCount]->dstBufferLen -=
                        setup->setupData.hashSetupData.digestResultLenInBytes;
                }
            }
        }
        else
        {
            /* Set physical address for srcbuffer */
            pOpdata[createCount]->srcBuffer = (CpaPhysicalAddr)(SAMPLE_CODE_UINT)
                    qaeVirtToPhysNUMA((SAMPLE_CODE_UINT  *)( SAMPLE_CODE_UINT )
                     ppSrcBuffListArray[createCount]);
            /* Set length of srcbuffer */
            pOpdata[createCount]->srcBufferLen = CPA_DP_BUFLIST;
            /* Set physcial address for dstBuffer */
            pOpdata[createCount]->dstBuffer =(CpaPhysicalAddr)(SAMPLE_CODE_UINT)
                    qaeVirtToPhysNUMA((SAMPLE_CODE_UINT *)( SAMPLE_CODE_UINT )
                     ppDestBuffListArray[createCount]);
            /* Set length of dstBuffer */
            pOpdata[createCount]->dstBufferLen = CPA_DP_BUFLIST;
        }
        /* Instance handle */
        pOpdata[createCount]->instanceHandle = setup->cyInstanceHandle;

        /* the physical address of this structure  */
        pOpdata[createCount]->thisPhys=(CpaPhysicalAddr)(SAMPLE_CODE_UINT)
                qaeVirtToPhysNUMA(pOpdata[createCount]);
        /* Set response : initialise pCallbackTag with perf_data_t to
        * callback operation */
        pOpdata[createCount]->pCallbackTag = setup->performanceStats;
    }/* End of for main loop*/
    return CPA_STATUS_SUCCESS;
}

/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * symmetric encryption operations enqueued to be performed later
 * The request is submitted to be performed by invoking the function
 * @ref cpaCySymDpPerformOpNow.
 * ***************************************************************************/
CpaStatus symDpPerformEnqueueOp(symmetric_test_params_t* setup,
        Cpa32U numOfLoops,
        CpaCySymDpOpData **ppOpData,
        CpaBufferList **ppSrcBuffListArray,
        CpaCySymCipherDirection cipherDirection
)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U outsideLoopCount = 0;
    Cpa32U insideLoopCount = 0;
    Cpa32U maxRequestNum = 0;
    perf_data_t *pSymData = NULL;
    Cpa64U numOps = 0;
    Cpa64U nextPoll = symPollingInterval_g;
    CpaBoolean performNowFlag = CPA_TRUE;
    CpaInstanceInfo2 instanceInfo = {0};
#ifdef LATENCY_CODE
    Cpa32U submissions = 0;
    Cpa32U i =0;
    perf_cycles_t request_submit_start[100] = {0};
    perf_cycles_t request_respnse_time[100] = {0};
#endif


    /* Check if setup is NULL */
    if(NULL == setup)
    {
        PRINT_ERR("parameter setup is NULL\n");
        status = CPA_STATUS_FAIL;
        return status;
    }
    /* Initialize perform data structure with setup->performanceStats*/
    pSymData = setup->performanceStats;
    /* Zero initialize pSymData*/
    memset(pSymData, 0, sizeof(perf_data_t));
    status = cpaCyInstanceGetInfo2(setup->cyInstanceHandle, &instanceInfo);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("%s::%d cpaCyInstanceGetInfo2 failed", __func__,__LINE__);
         return CPA_STATUS_FAIL;
    }
    pSymData->packageId = instanceInfo.physInstId.packageId;
    /*preset the number of ops we plan to submit*/
    pSymData->numOperations = (Cpa64U)setup->numBuffLists * numOfLoops;
    /* reset number of response to 0, sync sym opertations within outside loop*/
    pSymData->responses = 0;

    pSymData->retries = 0;
#ifdef LATENCY_CODE
    if(pSymData->numOperations > LATENCY_SUBMISSION_LIMIT)
    {
        PRINT_ERR("Error max submissions for latency  must be <= %d\n",
                LATENCY_SUBMISSION_LIMIT);
        return CPA_STATUS_FAIL;
    }
    pSymData->nextCount = (Cpa32U)(pSymData->numOperations/100);
    pSymData->countIncrement = pSymData->nextCount;

    pSymData->response_times=request_respnse_time;
#endif


    /* Check if numRequests:
    * the number of requests should no more than numbuffer*/
    if (setup->numRequests > setup->numBuffLists)
    {
        maxRequestNum = setup->numBuffLists;
        PRINT_ERR("parameter numRequests is error %d\n", setup->numRequests);
    }
    else
    {
        maxRequestNum = setup->numRequests;
    }

    /* CCM algorithm, initialize digest buffer to 0 for encryption */
    if(setup->setupData.cipherSetupData.cipherAlgorithm ==
                   CPA_CY_SYM_CIPHER_AES_CCM)
    {
        for(outsideLoopCount = 0;outsideLoopCount < setup->numBuffLists;
                 outsideLoopCount++)
        {
            if(CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT != cipherDirection)
            {
               /* Initialize digest buffer to 0 */
               symDpSetDigestBuffer(
                  ppOpData[outsideLoopCount]->messageLenToCipherInBytes,
                  AES_CCM_DIGEST_LENGTH_IN_BYTES,
                  0,
                  ppSrcBuffListArray[outsideLoopCount]);
            }
        }/* End of outsideLoopCount */
    }/* End of if which cipherAlgorithm = CPA_CY_SYM_CIPHER_AES_CCM */

    /* this barrier will wait until all threads get to this point*/
    sampleCodeBarrier();

    /* Get the time, collect this only for the first
     * request, the callback collects it for the last */
    pSymData->startCyclesTimestamp=sampleCodeTimestamp();

    /* The outside for-loop will loop around the preallocated
     * buffer list array*/
    for( outsideLoopCount = 0; outsideLoopCount < numOfLoops;
            outsideLoopCount++)
    {

        /* This inner for-loop loops around the number of Buffer Lists
        * that have been preallocated.  Once the array has completed
        * exit to the outer loop to move on the next iteration of the
        * preallocated loop. */
        for (insideLoopCount = 0;insideLoopCount < setup->numBuffLists;
                  insideLoopCount++)
        {
            /* Check the submitted request is equal to max request number or
             * if we are submitting the last packet
            * Then perform queued requests with PerformOpNowFlag is CPA_TRUE
            */
            if ((insideLoopCount % maxRequestNum == 0) ||
                    ( (insideLoopCount == setup->numBuffLists-1) &&
                            (outsideLoopCount == numOfLoops -1)))
            {
                performNowFlag = CPA_TRUE;
            }
            else
            {
                performNowFlag = CPA_FALSE;
            }
            do
            {
#ifdef LATENCY_CODE
                if(submissions+1 == pSymData->nextCount)
                {
                        request_submit_start[pSymData->latencyCount] =
                            sampleCodeTimestamp();
                }
#endif
                /* Enqueue a single symmetric crypto request, perform later*/
                status = cpaCySymDpEnqueueOp(ppOpData[insideLoopCount],
                       performNowFlag);

                /* When cpaCySymDpEnqueueOp return CPA_STATUS_RETRY, perform
                * cpaCySymDpPerformOpNow to clear those requests in the queue,
                * and pSymData->retries will increase 1
                * also re-schedule thread and wait for finishing operation.
                */
                if(CPA_STATUS_RETRY == status)
                {
                    pSymData->retries++;
                    icp_sal_CyPollDpInstance(setup->cyInstanceHandle,0);
                    nextPoll = numOps + symPollingInterval_g;
                    AVOID_SOFTLOCKUP;
                }

            }
            while (CPA_STATUS_RETRY == status);
            /* if cpaCySymDpEnqueueOp operation return non-success,
            * break the inside loop directly
            */
            if (CPA_STATUS_SUCCESS != status)
            {
                break;
            }
#ifdef LATENCY_CODE
            submissions++;
#endif
            /*If reach the limitation,
            * system will poll all requests in the queue*/
           ++numOps;
           if(numOps == nextPoll)
           {
              icp_sal_CyPollDpInstance(setup->cyInstanceHandle,0);
              nextPoll = numOps + symPollingInterval_g;

           }

        }/* End of  inside Loop */


        /* if status != CPA_STATUS_SUCCESS, break the out loop directly */
        if (CPA_STATUS_SUCCESS != status)
        {
            break;
        }

    }/* End of outside Loop */
    /* When the callback returns it will increment the responses
    * counter and test if its equal to numBuffLists, in that
    * case all responses have been successfully received. */
    if (CPA_STATUS_SUCCESS == status)
    {
        /*
        ** Now need to wait for all the inflight Requests.
        */
        status = cyDpPollNumOperations(pSymData,setup->cyInstanceHandle,
                                        pSymData->numOperations);

    }

#ifdef LATENCY_CODE
    for(i=0; i<pSymData->latencyCount;i++)
    {
        pSymData->aveLatency +=
            pSymData->response_times[i] - request_submit_start[i];
    }
    if(pSymData->latencyCount>0)
    {
        do_div(pSymData->aveLatency,pSymData->latencyCount);
    }
#endif
    return status;
}



/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * Enqueue multiple requests with one operation, perform later
 * ***************************************************************************/
CpaStatus symDpPerformEnqueueOpBatch(symmetric_test_params_t* setup,
        Cpa32U numOfLoops,
        CpaCySymDpOpData **ppOpData,
        CpaBufferList **ppSrcBuffListArray,
        CpaCySymCipherDirection cipherDirection
        )
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U outsideLoopCount = 0;
    Cpa32U insideLoopCount = 0;
    Cpa32U batchCount = 0;
    Cpa32U maxBatchNum = 0;
    Cpa32U maxRequestNum = 0;
    Cpa32U queueRequestNum = 0;
    CpaBoolean performNow = CPA_FALSE;
    perf_data_t *pSymData = NULL;
    Cpa32U numOps = 0;
    CpaInstanceInfo2 instanceInfo = {0};
    /* Check if setup is NULL */
    if(NULL == setup)
    {
        PRINT_ERR("parameter setup is NULL\n");
      status = CPA_STATUS_FAIL;
    return status;
    }

    /* Initialize perform data structure with setup->performanceStats*/
    pSymData = setup->performanceStats;
    memset(pSymData, 0, sizeof(perf_data_t));
    status = cpaCyInstanceGetInfo2(setup->cyInstanceHandle, &instanceInfo);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("%s::%d cpaCyInstanceGetInfo2 failed", __func__,__LINE__);
         return CPA_STATUS_FAIL;
    }
    pSymData->packageId = instanceInfo.physInstId.packageId;


    /*preset the number of ops we plan to submit*/
    pSymData->numOperations = (Cpa64U)setup->numBuffLists * numOfLoops;
    /* reset number of response to 0, sync sym opertations within outside
     * loop*/
    pSymData->responses = 0;
    pSymData->retries = 0;

    /* Check if numRequests: number of requests should no more than numbuffer*/
    if (setup->numRequests > setup->numBuffLists)
    {
        maxRequestNum = setup->numBuffLists;
        PRINT_ERR("parameter numRequests is error %d\n", setup->numRequests);
    }
    else
    {
        maxRequestNum = setup->numRequests;
    }

    /* Check if numOpDpBatch:number of Batch Op should no more than numbuffer*/
    if (setup->numOpDpBatch > setup->numBuffLists)
    {
        maxBatchNum = setup->numBuffLists;
        PRINT_ERR("parameter numOpDpBatch is error %d\n", setup->numOpDpBatch);
    }
    else
    {
        maxBatchNum = setup->numOpDpBatch;
    }


    /* Check if numOpDpBatch: numOpDpBatch should no more than numRequests*/
    if (maxBatchNum > maxRequestNum)
    {
        maxBatchNum = maxRequestNum;
        PRINT_ERR("parameter numOpDpBatch %d is more than numRequests %d\n",
             setup->numOpDpBatch,setup->numRequests);
    }

    /* CCM algorithm, initialize digest buffer to 0 when encrypto */
    if(setup->setupData.cipherSetupData.cipherAlgorithm ==
              CPA_CY_SYM_CIPHER_AES_CCM)
    {
         for(outsideLoopCount = 0;outsideLoopCount < setup->numBuffLists;
                 outsideLoopCount++)
        {
            if(CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT != cipherDirection)
            {
               symDpSetDigestBuffer(
                  ppOpData[outsideLoopCount]->messageLenToCipherInBytes,
                  AES_CCM_DIGEST_LENGTH_IN_BYTES,
                  0,
                  ppSrcBuffListArray[outsideLoopCount]);
            }
        }/* End of outsideLoopCount */
    }/* End of if which cipherAlgorithm = CPA_CY_SYM_CIPHER_AES_CCM */

    /*this barrier will wait until all threads get to this point*/
    sampleCodeBarrier();

    /* Get the time, collect this only for the first
     * request, the callback collects it for the last */
    pSymData->startCyclesTimestamp=sampleCodeTimestamp();

    /*The outside for-loop will loop around the preallocated buffer list array*/
    for(outsideLoopCount = 0;outsideLoopCount < numOfLoops;outsideLoopCount++)
    {

        /* This inner for-loop loops around the number of Buffer Lists
         * that have been preallocated.  Once the array has completed-
         * exit to the outer loop to move on the next iteration of the
         * preallocated loop. */

        insideLoopCount = 0;
        queueRequestNum = 0;

        while(insideLoopCount < setup->numBuffLists)
        {
            /* Calculate the number of request for batch operation */
            if((insideLoopCount + maxBatchNum) >= setup->numBuffLists)
            {
                batchCount = setup->numBuffLists - insideLoopCount;
                /* since it's last btach op in the inside loop,
                 * all batch operation need to perform directly
                 */
                performNow = CPA_TRUE;
            }
            else
            {
                batchCount = maxBatchNum;
                performNow = CPA_FALSE;
            }

            /* make sure that queued requests is no more than maxRequestNum*/
           if((queueRequestNum + batchCount) >= maxRequestNum)
           {
               batchCount = maxRequestNum - queueRequestNum;
               /* current queued requests is more than request number
               *  which allowed to be queued,
               *  all batch operations need to perform directly
               */
               performNow = CPA_TRUE;
               queueRequestNum = 0;
           }
           else
           {
              /* update request number in the queue */
              queueRequestNum += batchCount;
           }

           /* Perform cpaCySymDpEnqueueOpBatch */
           do
           {

               status = cpaCySymDpEnqueueOpBatch(batchCount,
                                &ppOpData[insideLoopCount],performNow);

               /* When cpaCySymDpEnqueueOp return CPA_STATUS_RETRY,
               * perform cpaCySymDpPerformOpNow to clear those requests
               *  in the queue,and pSymData->retries will increase 1
               * also re-schedule thread and wait for finishing operation.
               */
               if(CPA_STATUS_RETRY == status)
               {
                   setup->performanceStats->retries++;
                  AVOID_SOFTLOCKUP;
               }
           }
           while (CPA_STATUS_RETRY == status);

           /* if status isn't CPA_STATUS_SUCCESS,
           * break the inside loop directly */
           if (CPA_STATUS_SUCCESS != status)
           {
                 break;
           }

           /* update insideLoopCount */
           insideLoopCount += batchCount;
           numOps += batchCount;
           if((numOps >= symPollingInterval_g))
           {
              icp_sal_CyPollDpInstance(setup->cyInstanceHandle,0);
              numOps = 0;

           }
        }/* End of  inside Loop while*/


        /* if status != CPA_STATUS_SUCCESS, break the out loop directly */
        if (CPA_STATUS_SUCCESS != status)
        {
            break;
        }
    }/* End of outside Loop */

   /* When the callback returns it will increment the responses
     * counter and test if its equal to numBuffLists, in that
     * case all responses have been successfully received. */
    if (CPA_STATUS_SUCCESS == status)
    {
        /*
        ** Now need to wait for all the inflight Requests.
        */
        status = cyDpPollNumOperations(pSymData,setup->cyInstanceHandle,
                                        pSymData->numOperations);

    }


    return status;
}



/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 *  Main executing function
 *****************************************************************************/
static CpaStatus sampleSymmetricDpPerform( symmetric_test_params_t* setup)
{
    /* start of local variable declarations */
    CpaCySymDpSessionCtx *pEncryptSessionCtx = NULL;
    CpaCySymDpOpData **ppOpData = NULL;
    CpaPhysBufferList **ppSrcPhysBuffListArray = NULL;
    CpaBufferList    **ppSrcBuffListArray = NULL;
    Cpa32U i = 0;
    Cpa32U *totalSizeInBytes = NULL;
    perf_data_t *pSymPerfData = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U numOfLoops = 0;
    Cpa32U insideLoopCount = 0;
    Cpa32U numOfOperations= 0;
    Cpa8U *cipherKey = NULL;
    Cpa8U *authKey = NULL;
    CpaCySymDpCbFunc pSymCb = NULL;
    Cpa32U node = 0;
    CpaCySymCipherDirection cipherDirection =
        CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT;

    /* Check if setup is null */
    if(NULL == setup)
    {
        PRINT_ERR("parameter setup is error \n");
        return CPA_STATUS_FAIL;
    }

    /*get the node we are running on for local memory allocation*/
    status = sampleCodeCyGetNode(setup->cyInstanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("sampleCodeCyGetNode error, status: %d\n", status);
        return CPA_STATUS_FAIL;

    }
    /* Initialize local variables */
    numOfLoops = setup->numLoops;
    cipherKey = qaeMemAlloc(setup->setupData.cipherSetupData.cipherKeyLenInBytes
                          * sizeof(Cpa8U));
    if(NULL == cipherKey)
    {
        PRINT_ERR("cipherKey memory allocation error\n");
        return CPA_STATUS_FAIL;
    }
    authKey = qaeMemAlloc(setup->setupData.hashSetupData.authModeSetupData.
                          authKeyLenInBytes * sizeof(Cpa8U));
    if(NULL == authKey)
    {
        PRINT_ERR("authKey memory allocation error\n");
        qaeMemFree((void**)&cipherKey);
        return CPA_STATUS_FAIL;
    }

    totalSizeInBytes =
        qaeMemAlloc(setup->numBuffLists * sizeof(Cpa32U));
    if(NULL == totalSizeInBytes)
    {
        PRINT_ERR("totalSizeInBytes memory allocation error\n");
        qaeMemFree((void**)&authKey);
        qaeMemFree((void**)&cipherKey);
        return CPA_STATUS_FAIL;
    }

    pEncryptSessionCtx = qaeMemAlloc(sizeof(CpaCySymDpSessionCtx)*
                setup->numSessions);

    if(NULL == pEncryptSessionCtx)
    {
        PRINT_ERR("Encrypto Session Array memory allocation error\n");
        qaeMemFree((void**)&authKey);
        qaeMemFree((void**)&cipherKey);
        qaeMemFree((void**)&totalSizeInBytes);
        return CPA_STATUS_FAIL;
    }

    /*allocate memory for an array of bufferList pointers, flatBuffer pointers
     * and operation data, the bufferLists and Flat buffers are created in
     * dpSampleCreateBuffers of cpa_sample_code_crypto_utils.c*/
    status = allocArrayOfPointers(
            setup->cyInstanceHandle,
            (void**)&ppOpData,
            setup->numBuffLists);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("could not allocate opData Array\n");
        symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
             ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);
        return CPA_STATUS_FAIL;
    }
    status = allocArrayOfPointers(
            setup->cyInstanceHandle,
            (void**)&ppSrcBuffListArray,
            setup->numBuffLists);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("could not allocate ppSrcBuffListArray\n");
        symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
             ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);
        return CPA_STATUS_FAIL;
    }
    status = allocArrayOfPointers(
            setup->cyInstanceHandle,
            (void**)&ppSrcPhysBuffListArray,
            setup->numBuffLists);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("could not allocate ppSrcPhysBuffListArray\n");
        symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
             ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);
        return CPA_STATUS_FAIL;
    }

    /*use the preallocated performance stats to store performance data, this
    * points to an element in perfStats array in the framework, each thread
    * points to a unique element of perfStats array
    */
    pSymPerfData = setup->performanceStats;
    if(NULL == pSymPerfData)
    {
        PRINT_ERR("perf data pointer is NULL\n");
        symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
             ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);

        return CPA_STATUS_FAIL;
    } /* End of if(NULL == pSymPerfData) */
    memset(pSymPerfData, 0, sizeof(perf_data_t));

    if(setup->setupData.symOperation == CPA_CY_SYM_OP_HASH &&
            (setup->setupData.hashSetupData.hashAlgorithm ==
                CPA_CY_SYM_HASH_SNOW3G_UIA2 ||
                setup->setupData.hashSetupData.hashAlgorithm ==
                    CPA_CY_SYM_HASH_KASUMI_F9
#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#endif
            ))
    {
        setup->setupData.hashSetupData.digestResultLenInBytes =
            DIGEST_RESULT_4BYTES;

    }
    /*if we are testing hash or alg chain, get the hash size that needs to be
     *  allocated for the digest result. sampleCreateBuffers uses the hash size
     *  to allocate the appropriate memory*/
    for (insideLoopCount = 0;
    insideLoopCount < setup->numBuffLists;
    insideLoopCount++)
    {
        /* Calculate totalSizeInBytes:
        * It should no more than packetSizeInBytes + digestResultLenInBytes.
        * In cipher algorithm, digestResultLenInBytes should be 0.
        */
        totalSizeInBytes[insideLoopCount]=
            setup->packetSizeInBytesArray[insideLoopCount] +
            setup->setupData.hashSetupData.digestResultLenInBytes;
    }

    /*init the symmetric session*/
    /*if the mode is asynchronous then set the callback function*/
    if(ASYNC == setup->syncMode)
    {
        pSymCb = (CpaCySymDpCbFunc)symDpPerformCallback;
    }
    /* Setup session array with cipher key and authKey*/
    for(i = 0; i < setup->numSessions; i++)
    {
        status = symmetricDpSetupSession(pSymCb,
            cipherKey,
            authKey,
            &pEncryptSessionCtx[i],
            CPA_FALSE,
            CPA_FALSE,
            setup
            );

       if (CPA_STATUS_SUCCESS != status)
       {
            PRINT_ERR("symmetricSetupSession error, status %d\n", status);
            symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
             ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);

            return CPA_STATUS_FAIL;
        }/* End of if (CPA_STATUS_SUCCESS != status) */
    }/* End of session loop */

    /*if we are testing hash or alg chain, will create sample buffers with
     * space for digest result, otherwise we just create sample physical buffer
     * list and physical flat buffers based on the bufferSize we are testing.
     */
    status = dpSampleCreateBuffers(setup->cyInstanceHandle,
        totalSizeInBytes,
        ppSrcBuffListArray,
        ppSrcPhysBuffListArray,
        setup);


    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("dpSampleCreateBuffers error, status %d\n", status);
        symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
         ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);
        return CPA_STATUS_FAIL;
    }/*End of if which symOperation == CPA_CY_SYM_OP_HASH */


    /* setup the encrypt operation data with session array, packet size array
     * the pointer of operation data structure, the pointer of setup parameter
     * the pointer of source buffer list array and the pointer of destination
     * buffer list
     */
    status = symmetricDpPerformOpDataSetup(
            pEncryptSessionCtx,
            setup->packetSizeInBytesArray,
            ppOpData,
            setup,
            ppSrcPhysBuffListArray,
            ppSrcPhysBuffListArray);
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("symmetricPerformOpDataSetup error, status %d\n", status);
        symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
         ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);
        return status;
    }


 /*Perform different symmetric Data Plane Operations with four functions
 *numOpDpBatch        : numRequests              : Functions
 *SYM_DP_ENQUEUEING   : >SYM_DP_PERFORM_NOW_FLAG : symDpPerformEnqueueOp
 *Description: enqueue a single symmetric request, perform later
 *SYM_DP_ENQUEUEING   : SYM_DP_PERFORM_NOW_FLAG  : symDpPerformEnqueueOpNow
 *Description: perform a single symmetric request immediately
 *!=SYM_DP_ENQUEUEING : >SYM_DP_PERFORM_NOW_FLAG : symDpPerformEnqueueOpBatch
 *Description: enqueue multiple requests with one operation, perform later
 *!=SYM_DP_ENQUEUEING : SYM_DP_PERFORM_NOW_FLAG  :symDpPerformEnqueueOpBatchNow
 *Description: perform multiple requests with one operation immediately
 */

        if(SYM_DP_ENQUEUEING == setup->numOpDpBatch)
        {
            status = symDpPerformEnqueueOp(setup,numOfLoops,ppOpData,
                                 ppSrcBuffListArray,
                                 cipherDirection);
        }
        else
        {
            status = symDpPerformEnqueueOpBatch(setup,numOfLoops,
                    ppOpData,ppSrcBuffListArray,
                                cipherDirection);
        }/* End of if(SYM_DP_ENQUEUEING == setup->numOpDpBatch) */
        numOfOperations += pSymPerfData->numOperations;
        if (CPA_STATUS_SUCCESS != status)
        {

            PRINT_ERR("symPerform error, status %d, \n", status);
            symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
             ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);
            return status;
        }

    /* we must set again the number of operations and responses
    * after perform sym operation */
    pSymPerfData->numOperations = numOfOperations;
    pSymPerfData->responses = pSymPerfData->numOperations;

    /* Free up resources allocated */
    for(i=0; i<setup->numSessions;i++)
    {
        if (CPA_STATUS_SUCCESS != cpaCySymDpRemoveSession(
            setup->cyInstanceHandle, pEncryptSessionCtx[i]))
        {
            PRINT_ERR("Deregister session failed\n");
            status = CPA_STATUS_FAIL;
        }
    }/* for(i=0; i<setup->numSessions;i++) */

    symDpPerformMemFree(setup, ppSrcBuffListArray,ppSrcPhysBuffListArray,
     ppOpData, pEncryptSessionCtx,authKey,cipherKey,totalSizeInBytes);
    if(CPA_STATUS_SUCCESS != setup->performanceStats->threadReturnStatus)
    {
        status = CPA_STATUS_FAIL;
    }
    return status;
}


/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 *  Setup a symmetric crypto thread for a given packet size or mix
 *****************************************************************************/
static void sampleSymmetricDpPerformance(single_thread_test_data_t* testSetup)
{
    symmetric_test_params_t symTestSetup;
    symmetric_test_params_t* pSetup =
        ((symmetric_test_params_t*)testSetup->setupPtr);
    Cpa32U loopIteration = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    /*define the distribution of the packet mix
     * here we defined 2 lots of 10 sizes
     * later it is replicated into 100 buffers*/
    Cpa32U packetMix[NUM_PACKETS_IMIX] =
    {BUFFER_SIZE_64,  BUFFER_SIZE_752,
            BUFFER_SIZE_1504, BUFFER_SIZE_64,
            BUFFER_SIZE_752, BUFFER_SIZE_1504,
            BUFFER_SIZE_64, BUFFER_SIZE_64,
            BUFFER_SIZE_1504, BUFFER_SIZE_1504,
            BUFFER_SIZE_752, BUFFER_SIZE_64,
            BUFFER_SIZE_752, BUFFER_SIZE_64,
            BUFFER_SIZE_1504, BUFFER_SIZE_1504,
            BUFFER_SIZE_64, BUFFER_SIZE_8992,
            BUFFER_SIZE_64, BUFFER_SIZE_1504};
    Cpa32U *pPacketSize;
    Cpa16U numInstances = 0;
    CpaInstanceHandle *cyInstances = NULL;
    CpaInstanceInfo2 instanceInfo = {0};
#if defined(USER_SPACE) && !defined(SC_EPOLL_DISABLED)
    int fd = -1;
#endif

    memset(&symTestSetup, 0 ,sizeof(symmetric_test_params_t));

    /*cast the setup to a known structure so that we can populate our local
         * test setup*/
    symTestSetup.setupData     = pSetup->setupData;
    /*this barrier is to halt this thread when run in user space context, the
     * startThreads function releases this barrier, in kernel space it does
     * nothing, but kernel space threads do not start until we call startThreads
     * anyway*/
    startBarrier();
    /*give our thread a unique memory location to store performance stats*/
    symTestSetup.performanceStats = testSetup->performanceStats;
    /*get the instance handles so that we can start our thread on the selected
     * instance*/
    status = cpaCyGetNumInstances(&numInstances);
    if( CPA_STATUS_SUCCESS != status  || numInstances == 0)
    {
        PRINT_ERR("cpaCyGetNumInstances error, status:%d, numInstanaces:%d\n",
                status, numInstances);
        symTestSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        sampleCodeThreadExit();
    }
    cyInstances = qaeMemAlloc(sizeof(CpaInstanceHandle)*numInstances);
    if(cyInstances == NULL)
    {
        PRINT_ERR("Error allocating memory for instance handles\n");
        symTestSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        sampleCodeThreadExit();
    }
    if(cpaCyGetInstances(numInstances, cyInstances) != CPA_STATUS_SUCCESS)
    {
        PRINT_ERR("Failed to get instances\n");
        symTestSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        qaeMemFree((void**)&cyInstances);
        sampleCodeThreadExit();
    }
    if(testSetup->logicalQaInstance > numInstances)
    {
        PRINT_ERR("%u is Invalid Logical QA Instance, max is: %u\n",
                testSetup->logicalQaInstance, numInstances);
        symTestSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        qaeMemFree((void**)&cyInstances);
        sampleCodeThreadExit();
    }

    /* give our thread a logical crypto instance to use*/
    symTestSetup.cyInstanceHandle=cyInstances[testSetup->logicalQaInstance];

    status = cpaCyInstanceGetInfo2(symTestSetup.cyInstanceHandle,
            &instanceInfo);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("%s::%d cpaCyInstanceGetInfo2 failed", __func__,__LINE__);
        qaeMemFree((void**)&cyInstances);
        symTestSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        sampleCodeThreadExit();
    }

    if(instanceInfo.physInstId.packageId > packageIdCount_g)
    {
        packageIdCount_g = instanceInfo.physInstId.packageId;   
    }

    if(instanceInfo.isPolled == CPA_FALSE)
    {
        PRINT("Data-Plane operations not supported on non-polled instances\n");
        qaeMemFree((void**)&cyInstances);
        sampleCodeThreadExit();
    }
#if defined(USER_SPACE) && !defined(SC_EPOLL_DISABLED)

    status = icp_sal_CyGetFileDescriptor(symTestSetup.cyInstanceHandle, &fd);
    if (CPA_STATUS_SUCCESS == status)
    {
        PRINT("Data-Plane operations not supported on Epoll instances\n");
        qaeMemFree((void**)&cyInstances);
        icp_sal_CyPutFileDescriptor(symTestSetup.cyInstanceHandle, fd);
        symTestSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        sampleCodeThreadExit();
    }
#endif

    /* Assign the unique thread id to the symTestSetup structure */
    symTestSetup.threadID = testSetup->threadID;

    pPacketSize = qaeMemAlloc(
            sizeof(Cpa32U)*pSetup->numBuffLists);
    if(NULL == pPacketSize)
    {
        PRINT_ERR("Could not allocate memory for pPacketSize\n");
        symTestSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        qaeMemFree((void**)&cyInstances);
        sampleCodeThreadExit();
    }

    if(testSetup->packetSize == PACKET_IMIX)
    {
        /*we are testing IMIX so we copy buffer sizes from preallocated
         * array into symTestSetup.numBuffLists*/
        Cpa32U indexer = sizeof(packetMix)/sizeof(Cpa32U);
        for( loopIteration = 0; loopIteration < pSetup->numBuffLists;
        loopIteration++ )
        {
            pPacketSize[loopIteration] =
                packetMix[loopIteration % indexer];
        }
    }
    else
    {
        /*we are testing a uniform bufferSize, so we set the bufferSize array
         * accordingly*/
        for( loopIteration = 0; loopIteration < pSetup->numBuffLists;
        loopIteration++)
        {
            pPacketSize[loopIteration]=testSetup->packetSize;
        }
    }


    symTestSetup.numOpDpBatch  = pSetup->numOpDpBatch;
    symTestSetup.numRequests   = pSetup->numRequests;
    symTestSetup.numSessions   = pSetup->numSessions;
    symTestSetup.flatBufferSizeInBytes    = pSetup->flatBufferSizeInBytes;
    symTestSetup.numBuffLists  = pSetup->numBuffLists;
    symTestSetup.numLoops      = pSetup->numLoops;
    /*reset the stats print function to NULL, we set it to the proper function
     * if the test passes at the end of this function*/
    testSetup->statsPrintFunc=NULL;
    /*assign the array of buffer sizes we are testing to the symmetric test
     * setup*/
    symTestSetup.packetSizeInBytesArray = pPacketSize;
    /*assign our thread a unique memory location to store performance stats*/
    symTestSetup.performanceStats = testSetup->performanceStats;

    symTestSetup.performanceStats->averagePacketSizeInBytes =
        testSetup->packetSize;
    /* give our thread a logical crypto instance to use*/
    symTestSetup.cyInstanceHandle = cyInstances[testSetup->logicalQaInstance];
    symTestSetup.syncMode = pSetup->syncMode;
    /*store core affinity, this assumes logical cpu core number is the same
     * logicalQaInstace */
    symTestSetup.performanceStats->logicalCoreAffinity =
        testSetup->logicalQaInstance;
    symTestSetup.isDpApi = pSetup->isDpApi;
    symTestSetup.cryptoSrcOffset = pSetup->cryptoSrcOffset;
    symTestSetup.digestAppend = pSetup->digestAppend;
    /*launch function that does all the work*/

    if (CPA_TRUE !=
        checkCapability(cyInstances[testSetup->logicalQaInstance],
                         &symTestSetup))
    {
        PRINT("\nThread %u Invalid test\n", testSetup->threadID);
        testSetup->statsPrintFunc=
               (stats_print_func_t)printSymmetricPerfDataAndStopCyService;
        symTestSetup.performanceStats->threadReturnStatus =
                                                      CPA_STATUS_SUCCESS;
        sampleCodeBarrier();
        symTestSetup.packetSizeInBytesArray = NULL;
        qaeMemFree((void**)&pPacketSize);
        qaeMemFree((void**)&cyInstances);
        sampleCodeThreadExit();
    }

    status = sampleSymmetricDpPerform(&symTestSetup);
    if(CPA_STATUS_SUCCESS != status)
    {
        printSymTestType(&symTestSetup);
        PRINT("Symmetric thread %u FAILED\n", testSetup->threadID);
        symTestSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
    }
    else
    {
  	/*set the print function that can be used to print stats at the end of
         * the test*/
        testSetup->statsPrintFunc=
            (stats_print_func_t)printSymmetricPerfDataAndStopCyService;
    }
    /*free memory and exit*/
    qaeMemFree((void**)&pPacketSize);
    qaeMemFree((void**)&cyInstances);
    sampleCodeThreadExit();
}


/*****************************************************************************
 *
 *  External Function Interfaces
 *
 ****************************************************************************/

/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * setup a symmetric test
 * This function needs to be called from main to setup a symmetric test.
 * then the framework createThreads function is used to propagate this setup
 * across cores using different crypto logical instances
 ****************************************************************************/
CpaStatus setupSymmetricDpTest
(
        CpaCySymOp opType,
        CpaCySymCipherAlgorithm cipherAlg,
        Cpa32U cipherKeyLengthInBytes,
        Cpa32U cipherOffset,
        CpaCyPriority priority,
        CpaCySymHashAlgorithm hashAlg,
        CpaCySymHashMode hashMode,
        Cpa32U authKeyLengthInBytes,
        CpaCySymAlgChainOrder chainOrder,
        sync_mode_t syncMode,
        CpaCySymHashNestedModeSetupData *nestedModeSetupDataPtr,
        Cpa32U packetSize,
        Cpa32U numDpBatchOp,
        Cpa32U numRequests,
        Cpa32U numSessions,
        Cpa32U bufferSizeInBytes,
        Cpa32U numBuffLists,
        Cpa32U numLoops,
        Cpa32U digestAppend
)
{
    symmetric_test_params_t* symmetricSetup = NULL;
    Cpa8S name[] = {'D','P','_','S','Y','M','\0'};

    if(testTypeCount_g >= MAX_THREAD_VARIATION)
    {
        PRINT_ERR("Maximum Supported Thread Variation has been exceeded\n");
        PRINT_ERR("Number of Thread Variations created: %d",
                testTypeCount_g);
        PRINT_ERR(" Max is %d\n", MAX_THREAD_VARIATION);
        return CPA_STATUS_FAIL;
    }

    /* Return an error if the number of packets is not modulus zero of the
     * number of packets to cover IMIX packet mix.
     */
    if(packetSize == PACKET_IMIX && (numBuffLists % NUM_PACKETS_IMIX) != 0)
    {
        PRINT_ERR("To ensure that the weighting of IMIX packets is correct "
                ", the number of buffers (%d) should be a multiple of %d\n",
                numBuffLists, NUM_PACKETS_IMIX);
        return CPA_STATUS_FAIL;
    }

    /*start crypto service if not already started*/
    if(CPA_STATUS_SUCCESS != startCyServices())
    {
        PRINT_ERR("Failed to start Crypto services\n");
        return CPA_STATUS_FAIL;
    }

    /*as setup is a multidimensional char array we need to cast it to the
     * symmetric structure*/
    memcpy(&thread_name_g[testTypeCount_g][0],name,THREAD_NAME_LEN);
    symmetricSetup =
        (symmetric_test_params_t*)&thread_setup_g[testTypeCount_g][0];
    memset(symmetricSetup, 0 , sizeof(symmetric_test_params_t));
    testSetupData_g[testTypeCount_g].performance_function =
        (performance_func_t)sampleSymmetricDpPerformance;
    testSetupData_g[testTypeCount_g].packetSize=packetSize;


    /*then we store the test setup in the above location*/
    symmetricSetup->setupData.sessionPriority=priority;
    symmetricSetup->setupData.symOperation=opType;
    symmetricSetup->setupData.cipherSetupData.cipherAlgorithm = cipherAlg;
    symmetricSetup->setupData.cipherSetupData.cipherDirection =
        CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT;
    symmetricSetup->setupData.cipherSetupData.cipherKeyLenInBytes
    = cipherKeyLengthInBytes;
    symmetricSetup->setupData.hashSetupData.hashAlgorithm = hashAlg;
    symmetricSetup->setupData.hashSetupData.hashMode=hashMode;
    symmetricSetup->setupData.digestIsAppended  = digestAppend;
    if((symmetricSetup->setupData.symOperation ==
        CPA_CY_SYM_OP_ALGORITHM_CHAINING)&&
          (symmetricSetup->setupData.cipherSetupData.cipherAlgorithm
              ==CPA_CY_SYM_CIPHER_AES_CCM))
    {
        symmetricSetup->setupData.digestIsAppended = CPA_TRUE;
    }
    symmetricSetup->isDpApi = CPA_TRUE;
    symmetricSetup->cryptoSrcOffset=cipherOffset;
    /* in this code we limit the digest result len to be the same as the the
     * authentication key len*/
    symmetricSetup->setupData.hashSetupData.digestResultLenInBytes =
        authKeyLengthInBytes;

#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#endif

    /* check which kind of hash mode is selected */
    /*nested mode */
    if(CPA_CY_SYM_HASH_MODE_NESTED == hashMode)
    {
        /*set the struct for nested hash mode */
        if(NULL == nestedModeSetupDataPtr)
        {
            /* set a default nested mode setup data */
            symmetricSetup->setupData.hashSetupData.nestedModeSetupData.
            outerHashAlgorithm = hashAlg;
            symmetricSetup->setupData.hashSetupData.nestedModeSetupData.
            pInnerPrefixData = NULL;
            symmetricSetup->setupData.hashSetupData.nestedModeSetupData.
            innerPrefixLenInBytes = 0;
            symmetricSetup->setupData.hashSetupData.nestedModeSetupData.
            pOuterPrefixData = NULL;
            symmetricSetup->setupData.hashSetupData.nestedModeSetupData.
            outerPrefixLenInBytes = 0;
        }
        else
        {
            symmetricSetup->setupData.hashSetupData.nestedModeSetupData
            = *nestedModeSetupDataPtr;
        }
    }

    if((CPA_CY_SYM_HASH_AES_XCBC == hashAlg )&&
            (AES_XCBC_DIGEST_LENGTH_IN_BYTES !=authKeyLengthInBytes))
    {
        symmetricSetup->setupData.hashSetupData.authModeSetupData.
        authKeyLenInBytes = AES_XCBC_DIGEST_LENGTH_IN_BYTES;
    }
#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#endif
    else
    {
        symmetricSetup->setupData.hashSetupData.authModeSetupData.
        authKeyLenInBytes = authKeyLengthInBytes;
    }

    symmetricSetup->setupData.algChainOrder = chainOrder;
    symmetricSetup->syncMode = syncMode;
    symmetricSetup->numOpDpBatch = numDpBatchOp;
    symmetricSetup->numRequests = numRequests;
    symmetricSetup->numSessions = numSessions;
    symmetricSetup->flatBufferSizeInBytes=bufferSizeInBytes;
    symmetricSetup->numBuffLists=numBuffLists;
    symmetricSetup->numLoops=numLoops;
    if(((bufferSizeInBytes != 0 ) && (packetSize == PACKET_IMIX))
      || (bufferSizeInBytes % IV_LEN_FOR_16_BYTE_BLOCK_CIPHER != 0 ))
    {
        PRINT_ERR("Doesn't support PACKET_IMIX  "
                  "when the flat buffer size is not 0 or "
                  " it's not align with block size (%d): ",
                   bufferSizeInBytes);
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}


/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * setup a cipher test
 * This function needs to be called from main to setup a cipher test.
 * then the framework createThreads function is used to propagate this setup
 * across cores using different crypto logical instances
 *****************************************************************************/
CpaStatus setupCipherDpTest
(
        CpaCySymCipherAlgorithm cipherAlg,
        Cpa32U cipherKeyLengthInBytes,
        CpaCyPriority priority,
        sync_mode_t syncMode,
        Cpa32U packetSize,
        Cpa32U numDpBatchOp,
        Cpa32U flatBufferSize,
        Cpa32U numRequests,
        Cpa32U numSessions,
        Cpa32U numBuffLists,
        Cpa32U numLoops
)
{
    return setupSymmetricDpTest
    (
            CPA_CY_SYM_OP_CIPHER,
            cipherAlg,
            cipherKeyLengthInBytes,
            NOT_USED,
            priority,
            NOT_USED /* hash alg not needed in cipher test*/,
            NOT_USED /* hash mode not needed in cipher test*/,
            NOT_USED /* auth key len not needed in cipher test*/,
            NOT_USED /* chain mode not needed in cipher test*/,
            syncMode,
            NULL, /* nested hash data not needed in cipher test*/
            packetSize,
            numDpBatchOp,
            numRequests,
            numSessions,
            flatBufferSize,
            numBuffLists,
            numLoops,
            digestAppended_g
    );
}

/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * setup a hash test
 * This function needs to be called from main to setup a hash test.
 * then the framework createThreads function is used to propagate this setup
 * across cores using different crypto logical instances
 *****************************************************************************/
CpaStatus setupHashDpTest
(
        CpaCySymHashAlgorithm hashAlg,
        CpaCySymHashMode hashMode,
        Cpa32U authKeyLengthInBytes,
        CpaCyPriority priority,
        sync_mode_t syncMode,
        Cpa32U packetSize,
        Cpa32U numDpBatchOp,
        Cpa32U numRequests,
        Cpa32U numSessions,
        Cpa32U numBuffLists,
        Cpa32U numLoops
)
{
    return setupSymmetricDpTest
    (
            CPA_CY_SYM_OP_HASH,
            NOT_USED /* cipher alg not needed in cipher test*/,
            NOT_USED /* cipher key len not needed in cipher test*/,
            NOT_USED,
            priority,
            hashAlg,
            hashMode,
            authKeyLengthInBytes,
            NOT_USED /* chain mode not needed in cipher test*/,
            syncMode,
            NULL, /* nested hash data not needed in cipher test*/
            packetSize,
            numDpBatchOp,
            numRequests,
            numSessions,
            BUFFER_SIZE_0,
            numBuffLists,
            numLoops,
            digestAppended_g
    );
}


/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * setup a alg chain test.
 * This function needs to be called from main to setup an alg chain test.
 * then the framework createThreads function is used to propagate this setup
 * across IA cores using different crypto logical instances
 *****************************************************************************/
CpaStatus setupAlgChainDpTest
(
        CpaCySymCipherAlgorithm cipherAlg,
        Cpa32U cipherKeyLengthInBytes,
        CpaCySymHashAlgorithm hashAlg,
        CpaCySymHashMode hashMode,
        Cpa32U authKeyLengthInBytes,
        CpaCySymAlgChainOrder chainOrder,
        CpaCyPriority priority,
        sync_mode_t syncMode,
        Cpa32U packetSize,
        Cpa32U numDpBatchOp,
        Cpa32U flatBufferSize,
        Cpa32U numRequests,
        Cpa32U numSessions,
        Cpa32U numBuffLists,
        Cpa32U numLoops
)
{
    return setupSymmetricDpTest
    (
            CPA_CY_SYM_OP_ALGORITHM_CHAINING,
            cipherAlg,
            cipherKeyLengthInBytes,
            NOT_USED,
            priority,
            hashAlg,
            hashMode,
            authKeyLengthInBytes,
            chainOrder,
            syncMode,
            NULL,
            packetSize,
            numDpBatchOp,
            numRequests,
            numSessions,
            flatBufferSize,
            numBuffLists,
            numLoops,
            digestAppended_g
    );
}

/******************************************************************************
 * @ingroup sampleSymmetricPerf
 *
 * @description
 * setup an IPsec scenario where payload = IP packet, the IP header is not
 * encrypted thus requires an offset into the buffer to test.
 *
 * This function needs to be called from main to setup an alg chain test.
 * then the framework createThreads function is used to propagate this setup
 * across IA cores using different crypto logical instances
 ******************************************************************************/
CpaStatus setupIpSecDpTest
(
        CpaCySymCipherAlgorithm cipherAlg,
        Cpa32U cipherKeyLengthInBytes,
        Cpa32U cipherOffset,
        CpaCySymHashAlgorithm hashAlg,
        CpaCySymHashMode hashMode,
        Cpa32U authKeyLengthInBytes,
        CpaCySymAlgChainOrder chainOrder,
        Cpa32U packetSize,
        Cpa32U numDpBatchOp,
        Cpa32U numRequests,
        Cpa32U numSessions,
        Cpa32U numBuffLists,
        Cpa32U numLoops
)
{
    return setupSymmetricDpTest
    (
            CPA_CY_SYM_OP_ALGORITHM_CHAINING,
            cipherAlg,
            cipherKeyLengthInBytes,
            cipherOffset,
            CPA_CY_PRIORITY_HIGH,
            hashAlg,
            hashMode,
            authKeyLengthInBytes,
            chainOrder,
            ASYNC,
            NULL,
            packetSize,
            numDpBatchOp,
            numRequests,
            numSessions,
            BUFFER_SIZE_0,
            numBuffLists,
            numLoops,
            digestAppended_g
    );
}
EXPORT_SYMBOL(setupIpSecDpTest);

/*****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 * setup a alg chain test
 * This function needs to be called from main to setup an alg chain test.
 * then the framework createThreads function is used to propagate this setup
 * across IA cores using different crypto logical instances
 *****************************************************************************/
CpaStatus setupAlgChainTestDpNestedMode
(
        CpaCySymCipherAlgorithm cipherAlg,
        Cpa32U cipherKeyLengthInBytes,
        CpaCySymHashAlgorithm hashAlg,
        Cpa32U authKeyLengthInBytes,
        CpaCySymAlgChainOrder chainOrder,
        CpaCyPriority priority,
        sync_mode_t syncMode,
        CpaCySymHashNestedModeSetupData *nestedModeSetupData,
        Cpa32U packetSize,
        Cpa32U numDpOpBatch,
        Cpa32U numRequests,
        Cpa32U numSessions,
        Cpa32U numBuffLists,
        Cpa32U numLoops
)
{
    return setupSymmetricDpTest
    (
            CPA_CY_SYM_OP_ALGORITHM_CHAINING,
            cipherAlg,
            cipherKeyLengthInBytes,
            NOT_USED,
            priority,
            hashAlg,
            CPA_CY_SYM_HASH_MODE_NESTED,
            authKeyLengthInBytes,
            chainOrder,
            syncMode,
            nestedModeSetupData,
            packetSize,
            numDpOpBatch,
            numRequests,
            numSessions,
            BUFFER_SIZE_0,
            numBuffLists,
            numLoops,
            digestAppended_g
    );
}
