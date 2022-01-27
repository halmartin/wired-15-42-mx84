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
 * @file cpa_sample_code_ecdsa_perf.c
 *
 * @ingroup cryptoThreads
 *
 * @description
 *      This file contains the main ecdsa performance code. This code generates
 *      a random private key, then uses Elliptic curves to get a public key.
 *      A User defined number of  random messages are generated and signed
 *      using the private key. A User defined number of messages are then
 *      verified repeatedly using the public key to  measure performance
 *
 *****************************************************************************/

#include "cpa_cy_ec.h"
#include "cpa_cy_ecdsa.h"
#include "cpa_sample_code_crypto_utils.h"
#include "cpa_sample_code_ec_curves.h"
#ifdef NEWDISPLAY
#include "cpa_sample_code_NEWDISPLAY_crypto_utils.h"
#endif

extern Cpa32U packageIdCount_g;

///*
// * ***********************************************************************
// * elliptic curve definitions as defined in:
// *      http://csrc.nist.gov/groups/ST/toolkit/documents/dss/NISTReCur.pdf
// * ************************************************************************/



/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      Callback function for ECDSA verify operations
***************************************************************************/
void ecdsaPerformCallback(void *pCallbackTag,
        CpaStatus status,
        void *pOpData,
        CpaBoolean verifyStatus)
{
    processCallback(pCallbackTag);
}

/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      Callback function for ECDSA verify operations
***************************************************************************/
void ecdsaSignOnlyPerformCallback(void *pCallbackTag,
        CpaStatus status,
        void *pOpData,
        CpaBoolean multiplyStatus,
        CpaFlatBuffer *pR,
        CpaFlatBuffer *pS)
{
    processCallback(pCallbackTag);
}

/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      get the relevant curve data for the test parameters passed in
***************************************************************************/
CpaStatus getCurveData(ecdsa_test_params_t* setup)
{
    Cpa32U i = 0;
    Cpa32U numCurves = sizeof(curves_g)/sizeof(ec_curves_t);
    /*loop through the pre-defined curves and match on the nLenInBytes and the
     * fieldType than pass a pointer to the matched curve into the setup
     * structure*/
    for(i = 0; i<numCurves; i++)
    {
        if(curves_g[i].nLenInBytes == setup->nLenInBytes
                && curves_g[i].fieldType == setup->fieldType)
        {
            break;
        }
    }
    if(i == numCurves)
    {
        PRINT_ERR("Could not find curve data for the user input supplied\n");
        return CPA_STATUS_FAIL;
    }
    /*set the setup to the matched curve*/
    setup->pCurve = &curves_g[i];
    return CPA_STATUS_SUCCESS;
}

#define CALC_EC_POINT_MEM_FREE \
do { \
    qaeMemFreeNUMA((void**)&opData.a.pData); \
    qaeMemFreeNUMA((void**)&opData.b.pData); \
    qaeMemFreeNUMA((void**)&opData.q.pData); \
    qaeMemFreeNUMA((void**)&opData.xg.pData); \
    qaeMemFreeNUMA((void**)&opData.yg.pData); \
}while(0)
/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      Calculate a point on an Elliptic curve, using the curve in the setup
 *      parameter and a given k, and place the calculated point in pXk and
 *      pYk
***************************************************************************/
CpaStatus calcEcPoint(ecdsa_test_params_t* setup, CpaFlatBuffer* k,
        CpaFlatBuffer *pXk, CpaFlatBuffer *pYk)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaBoolean multiplyStatus = CPA_TRUE;
    CpaCyEcPointMultiplyOpData opData = {{0}};
    Cpa32U retries = 0;
    perf_data_t *pPerfData = NULL;
    CpaCyEcPointMultiplyCbFunc cbFunc = NULL;



    /*allocate the operation data structure and copy in the elliptic curve
     *  data*/
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, pXk,
            setup->nLenInBytes, NULL, 0, CALC_EC_POINT_MEM_FREE);
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, pYk,
            setup->nLenInBytes, NULL, 0, CALC_EC_POINT_MEM_FREE);

    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.a),
            setup->nLenInBytes, setup->pCurve->a,
            setup->pCurve->sizeOfa, CALC_EC_POINT_MEM_FREE);

    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.b),
            setup->pCurve->sizeOfb, setup->pCurve->b,
            setup->pCurve->sizeOfb, CALC_EC_POINT_MEM_FREE);

//    displayHexArray("opData.b", opData.b.pData,
//            opData.b.dataLenInBytes);
//
//    PRINT("opData.b.dataLenInBytes length = %u\n",opData.b.dataLenInBytes);

    /* We copy p into q,  the QA API uses q and NIST calls the same thing p*/
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.q),
            setup->nLenInBytes, setup->pCurve->p,
            setup->pCurve->sizeOfp, CALC_EC_POINT_MEM_FREE);

    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.xg),
            setup->nLenInBytes, setup->pCurve->xg,
            setup->pCurve->sizeOfxg, CALC_EC_POINT_MEM_FREE);

    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.yg),
            setup->nLenInBytes, setup->pCurve->yg,
            setup->pCurve->sizeOfyg, CALC_EC_POINT_MEM_FREE);

    /*make sure the private key is less than the modulus*/
    makeParam1SmallerThanParam2(k->pData, opData.q.pData,
            k->dataLenInBytes, CPA_FALSE);

    opData.k.pData = k->pData;
    opData.k.dataLenInBytes = k->dataLenInBytes;
    /*set h param*/
    opData.h.pData = NULL;
    opData.h.dataLenInBytes = 0;
    /*set fieldType*/
    opData.fieldType = setup->fieldType;

    /*Calculate the point on the curve*/
    do{
        status = cpaCyEcPointMultiply(setup->cyInstanceHandle,
                        cbFunc,
                        pPerfData,
                        &opData,
                        &multiplyStatus,
                        pXk,
                        pYk);

        if(CPA_STATUS_RETRY == status)
        {
            retries++;
            /*once we get to many retries, perform a context switch
             * to give the acceleration engine a small break */
            if(RETRY_LIMIT == (retries % (RETRY_LIMIT+1)))
            {
                AVOID_SOFTLOCKUP;
            }
        }
    }
    while(CPA_STATUS_RETRY==status);

    if(status != CPA_STATUS_SUCCESS)
    {
        PRINT_ERR("Failed to perform cpaCyEcPointMultiply: status: %d\n",
                status);
        CALC_EC_POINT_MEM_FREE;
        return status;
    }
    else
    {
        if(multiplyStatus == CPA_FALSE)
        {
            PRINT_ERR("cpaCyEcPointMultiply status is FALSE\n");
            CALC_EC_POINT_MEM_FREE;
            return CPA_STATUS_FAIL;

        }
    }
    /*free the memory in the operation data structure*/
    CALC_EC_POINT_MEM_FREE;
    return status;
}

/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      Free any memory allocated in the ecdsaSignRS function
***************************************************************************/
#define ECDSA_SIGN_RS_MEM_FREE \
do{ \
    qaeMemFreeNUMA((void**)&opData.a.pData); \
    qaeMemFreeNUMA((void**)&opData.b.pData); \
    qaeMemFreeNUMA((void**)&opData.k.pData); \
    qaeMemFreeNUMA((void**)&opData.n.pData); \
    qaeMemFreeNUMA((void**)&opData.q.pData); \
    qaeMemFreeNUMA((void**)&opData.xg.pData); \
    qaeMemFreeNUMA((void**)&opData.yg.pData); \
    qaeMemFreeNUMA((void**)&opData.d.pData); \
    qaeMemFreeNUMA((void**)&opData.m.pData); \
    qaeMemFreeNUMA((void**)&digest.pData); \
}while(0)


/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      Sign the digest of a random message using elliptic curve data in setup
 *      parameter
***************************************************************************/
CpaStatus ecdsaSignRS(ecdsa_test_params_t* setup, CpaFlatBuffer *d,
        CpaFlatBuffer* r, CpaFlatBuffer* s, CpaFlatBuffer* message,
        CpaFlatBuffer* z, CpaCyEcdsaSignRSCbFunc cbFunc,
        perf_data_t* pEcdsaData)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaBoolean signStatus = CPA_FALSE;
    Cpa32U node = 0;
    CpaCyEcdsaSignRSOpData opData = {{0}};
    CpaFlatBuffer digest = {0};
    Cpa32U retries = 0;
    perf_data_t *pPerfData = NULL;
    CpaCyEcdsaSignRSCbFunc signRSCbFunc = NULL;
    CpaInstanceInfo2 instanceInfo2 = {0};


    status = sampleCodeCyGetNode(setup->cyInstanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("sampleCodeCyGetNode failed with status %u\n", status);
        return status;
    }

    status = cpaCyInstanceGetInfo2(setup->cyInstanceHandle, &instanceInfo2);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCyInstanceGetInfo2 error, status: %d\n", status);
        return CPA_STATUS_FAIL;
    }
    if(instanceInfo2.physInstId.packageId > packageIdCount_g)
    {
        packageIdCount_g = instanceInfo2.physInstId.packageId;
    }
    /*allocate memory for a SHA512 digest*/
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &digest,
            SHA512_DIGEST_LENGTH_IN_BYTES, NULL, 0, ECDSA_SIGN_RS_MEM_FREE);
    /*allocate sign parameters*/
    generateRandomData(message->pData, message->dataLenInBytes);
    /*Calculate the digest of the message*/
    status = calcDigest(setup->cyInstanceHandle,
            message,
            &digest,
            CPA_CY_SYM_HASH_SHA512);
    if(status != CPA_STATUS_SUCCESS)
    {
        PRINT_ERR("ECDSA Calc Digest Error hashAlg %u\n", CPA_CY_SYM_HASH_SHA512);
	return status;
    }
    /*copy left most Bytes of digest into z parameter, the length should
     * be equal or less than SHA512_DIGEST_LENGTH_IN_BYTES*/
    memcpy(z->pData,digest.pData,z->dataLenInBytes);

    /*set the opData parameters for signing the message*/
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.a),
            setup->nLenInBytes, setup->pCurve->a,
            setup->pCurve->sizeOfa, ECDSA_SIGN_RS_MEM_FREE);

    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.b),
            setup->nLenInBytes, setup->pCurve->b,
            setup->pCurve->sizeOfb, ECDSA_SIGN_RS_MEM_FREE);

    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.k),
            setup->nLenInBytes, NULL, 0, ECDSA_SIGN_RS_MEM_FREE);
    generateRandomData(opData.k.pData, opData.k.dataLenInBytes);
    /*k is a value >0 and < the order of the base point*/
    makeParam1SmallerThanParam2(opData.k.pData, setup->pCurve->r,
            opData.k.dataLenInBytes, CPA_FALSE);

    /* we copy r into n, the QA API uses the parameter n as the order of the
     * base point, the NIST definition of the same is r*/
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.n),
            setup->nLenInBytes, setup->pCurve->r,
            setup->pCurve->sizeOfr, ECDSA_SIGN_RS_MEM_FREE);

    /* We copy p into q,  the QA API uses q and NIST calls the same thing p*/
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.q),
            setup->nLenInBytes, setup->pCurve->p,
            setup->pCurve->sizeOfp, ECDSA_SIGN_RS_MEM_FREE);
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.xg),
            setup->nLenInBytes, setup->pCurve->xg,
            setup->pCurve->sizeOfxg, ECDSA_SIGN_RS_MEM_FREE);
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.yg),
            setup->nLenInBytes, setup->pCurve->yg,
            setup->pCurve->sizeOfyg, ECDSA_SIGN_RS_MEM_FREE);
    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.d),
            d->dataLenInBytes, d->pData,
            d->dataLenInBytes, ECDSA_SIGN_RS_MEM_FREE);

    ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &(opData.m),
            z->dataLenInBytes, z->pData,
            z->dataLenInBytes, ECDSA_SIGN_RS_MEM_FREE);

    opData.fieldType = setup->fieldType;

    /*perform the sign operation*/
    do{
        status = cpaCyEcdsaSignRS(setup->cyInstanceHandle,
                        signRSCbFunc,
                        pPerfData,
                        &opData,
                        &signStatus,
                        r,
                        s);
        if(CPA_STATUS_RETRY == status)
        {
            retries++;
            /*once we get to many retries, perform a context switch
             * to give the acceleration engine a small break */
            if(RETRY_LIMIT == (retries % (RETRY_LIMIT+1)))
            {
                AVOID_SOFTLOCKUP;
            }
        }
    }
    while(CPA_STATUS_RETRY==status);

    if(status != CPA_STATUS_SUCCESS)
    {
        PRINT_ERR("Failed to sign message, status: %d\n", status);
        ECDSA_SIGN_RS_MEM_FREE;
        return status;
    }
    else
    {
        /*if(signStatus != CPA_TRUE)
        {
            PRINT_ERR("Unable to sign message\n");
            ECDSA_SIGN_RS_MEM_FREE;
            return CPA_STATUS_FAIL;
        }*/
    }
    /*free the memory for the operation data and the digest...we save the
     * message for the verification part*/
    ECDSA_SIGN_RS_MEM_FREE;

    return status;
}

#define ECDSA_PERFORM_MEM_FREE() \
do{ \
    ecdsaMemFree(setup, pX, pY, pR, pS, msg, pZ, vOpData, privateKey); \
}while(0)

void ecdsaMemFree(ecdsa_test_params_t* setup,
    CpaFlatBuffer* pX, CpaFlatBuffer* pY, CpaFlatBuffer* pR,
    CpaFlatBuffer* pS, CpaFlatBuffer* msg, CpaFlatBuffer* pZ,
    CpaCyEcdsaVerifyOpData** vOpData,
    CpaFlatBuffer privateKey)
{
    Cpa32U k = 0;

    /*free verify opData*/
    if(NULL != vOpData)
    {
        for(k = 0;k<setup->numBuffers;k++)
        {
            if(NULL != vOpData[k])
            {
                qaeMemFreeNUMA((void**)&vOpData[k]->yp.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->xp.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->yg.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->xg.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->s.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->r.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->q.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->n.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->m.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->a.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->b.pData);
                qaeMemFree((void**)&vOpData[k]);
            }
            qaeMemFreeNUMA((void**)&pR[k].pData);
            qaeMemFreeNUMA((void**)&pS[k].pData);
            qaeMemFreeNUMA((void**)&msg[k].pData);
            qaeMemFreeNUMA((void**)&pZ[k].pData);
        }
    }
    /* free all memory */
    qaeMemFreeNUMA((void**)&pX->pData);
    qaeMemFreeNUMA((void**)&pY->pData);
    qaeMemFree((void**)&pX);
    qaeMemFree((void**)&pY);
    qaeMemFree((void**)&pR);
    qaeMemFree((void**)&pS);
    qaeMemFree((void**)&msg);
    qaeMemFree((void**)&pZ);
    if(NULL != vOpData)
    {
        qaeMemFree((void**)&vOpData);
    }
    qaeMemFreeNUMA((void**)&privateKey.pData);
}

#define ECDSA_PERFORM_RS_MEM_FREE() \
do{ \
    ecdsaRsMemFree(setup, pX, pY, pR, pS, msg, pZ, vOpData, privateKey); \
}while(0)

void ecdsaRsMemFree(ecdsa_test_params_t* setup,
    CpaFlatBuffer* pX, CpaFlatBuffer* pY, CpaFlatBuffer* pR,
    CpaFlatBuffer* pS, CpaFlatBuffer* msg, CpaFlatBuffer* pZ,
    CpaCyEcdsaSignRSOpData** vOpData,
    CpaFlatBuffer privateKey)
{
    Cpa32U k = 0;

    /*free verify opData*/
    if(NULL != vOpData)
    {
        for(k = 0;k<setup->numBuffers;k++)
        {
            if(NULL != vOpData[k])
            {
                //qaeMemFreeNUMA((void**)&vOpData[k]->yp.pData);
                //qaeMemFreeNUMA((void**)&vOpData[k]->xp.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->yg.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->xg.pData);
                //qaeMemFreeNUMA((void**)&vOpData[k]->s.pData);
                //qaeMemFreeNUMA((void**)&vOpData[k]->r.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->q.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->n.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->a.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->b.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->k.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->m.pData);
                qaeMemFreeNUMA((void**)&vOpData[k]->d.pData);
                qaeMemFree((void**)&vOpData[k]);
            }
            qaeMemFreeNUMA((void**)&pR[k].pData);
            qaeMemFreeNUMA((void**)&pS[k].pData);
            qaeMemFreeNUMA((void**)&msg[k].pData);
            qaeMemFreeNUMA((void**)&pZ[k].pData);
        }
    }
    /* free all memory */
    qaeMemFreeNUMA((void**)&pX->pData);
    qaeMemFreeNUMA((void**)&pY->pData);
    qaeMemFree((void**)&pX);
    qaeMemFree((void**)&pY);
    qaeMemFree((void**)&pR);
    qaeMemFree((void**)&pS);
    qaeMemFree((void**)&msg);
    qaeMemFree((void**)&pZ);
    if(NULL != vOpData)
    {
        qaeMemFree((void**)&vOpData);
    }
    qaeMemFreeNUMA((void**)&privateKey.pData);
}
/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      setup a number of buffers to be signed by ECDSA then verified using
 *      EC curve data
***************************************************************************/
CpaStatus ecdsaPerform(ecdsa_test_params_t* setup)
{
    Cpa32U i = 0;
    Cpa32U numLoops = 0;
    CpaBoolean verifyStatus;
    CpaCyEcdsaStats64 ecdsaStats;
    CpaStatus status = CPA_STATUS_FAIL;
    /*pointer to Eliptic curve public key points*/
    CpaFlatBuffer* pX = NULL;
    CpaFlatBuffer* pY = NULL;
    /*array of pointers to Signature R & S of the messages below*/
    CpaFlatBuffer* pR = NULL;
    CpaFlatBuffer* pS = NULL;
    /*array of pointers to messages to be signed*/
    CpaFlatBuffer* msg = NULL;
    /*array of pointers to store digest of the above messages*/
    CpaFlatBuffer* pZ = NULL;
    /*private key used for all messages and to generate public key*/
    CpaFlatBuffer privateKey;
    /*array of pointers to the operation data structure for each verify
     * operation*/
    CpaCyEcdsaVerifyOpData** vOpData = NULL;
    /*variable to store what cpu thread is running on*/
    Cpa32U node = 0;
    /*pointer to location to store performance data*/
    perf_data_t* pEcdsaData  = NULL;
    CpaCyEcdsaVerifyCbFunc cbFunc = NULL;

    status = cpaCyEcdsaQueryStats64(setup->cyInstanceHandle,&ecdsaStats);
    if(status != CPA_STATUS_SUCCESS)
    {
        PRINT_ERR("Could not retrieve stats, error status %d\n", status);
    }
    /*get the node we are running on for local memory allocation*/
    status = sampleCodeCyGetNode(setup->cyInstanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("sampleCodeCyGetNode error, status: %d\n", status);
        return status;
    }
    /*get the curve data based on the test setup*/
    status = getCurveData(setup);
    if(CPA_STATUS_SUCCESS != status)
    {
        /*any error is printed in the getCurveData function*/
        return status;
    }

    /*get memory location to write performance stats to*/
    pEcdsaData = setup->performanceStats;
    memset(pEcdsaData, 0, sizeof(perf_data_t));

    /*get the number of operations to be done in this test*/
    pEcdsaData->numOperations = (Cpa64U)setup->numBuffers*setup->numLoops;
    pEcdsaData->responses = 0;

    /* Initilise semaphore used in callback */
    sampleCodeSemaphoreInit(&pEcdsaData->comp, 0);

    /*allocate memory to store public key points*/
    pX = qaeMemAlloc(sizeof(CpaFlatBuffer));
    if(NULL == pX)
    {
        PRINT_ERR("pY mem allocation error\n");
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }

    memset(pX,0,sizeof(CpaFlatBuffer));

    pY = qaeMemAlloc(sizeof(CpaFlatBuffer));
    if(NULL == pY)
    {
        PRINT_ERR("pY mem allocation error\n");
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }

    memset(pY,0,sizeof(CpaFlatBuffer));

    privateKey.pData = qaeMemAllocNUMA(
            setup->nLenInBytes, node, BYTE_ALIGNMENT_64);
    if(NULL == privateKey.pData)
    {
        PRINT_ERR("privateKey pData  mem allocation error\n");
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }

    privateKey.dataLenInBytes = setup->nLenInBytes;
    /*generate a random private key*/
    generateRandomData(privateKey.pData, privateKey.dataLenInBytes);
    makeParam1SmallerThanParam2(privateKey.pData, setup->pCurve->r,
            setup->nLenInBytes, CPA_FALSE);

    /*allocate memory for array of signatures, messages, digests and opData*/

    pR = qaeMemAlloc(sizeof(CpaFlatBuffer) * setup->numBuffers);
    if(NULL == pR)
    {
        PRINT_ERR("pR mem allocation error\n");
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }

    pS = qaeMemAlloc(sizeof(CpaFlatBuffer) * setup->numBuffers);
    if(NULL == pS)
    {
        PRINT_ERR("pS mem allocation error\n");
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }

    msg = qaeMemAlloc(sizeof(CpaFlatBuffer) * setup->numBuffers);
    if(NULL == msg)
    {
        PRINT_ERR("msg mem allocation error\n");
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }

    pZ = qaeMemAlloc(sizeof(CpaFlatBuffer) * setup->numBuffers);
    if(NULL == pZ)
    {
        PRINT_ERR("pZ mem allocation error\n");
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }

    /*calculate the public key p(X,Y) points*/
    status = calcEcPoint(setup, &privateKey, pX, pY);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("calcEcPoint Failes with status %d\n", status);
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }
    /*for each buffer:
     *      generate a random message
     *      calculate the digest
     *      sign the digest of the message with R&S*/
    for(i = 0; i < setup->numBuffers; i++)
    {
        /*allocate the pointers within the array of pointers*/
        /*allocate the pData for each CpaFlatBuffer*/
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &pR[i],
                setup->nLenInBytes, NULL, 0, ECDSA_PERFORM_MEM_FREE());
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &pS[i],
                setup->nLenInBytes, NULL, 0, ECDSA_PERFORM_MEM_FREE());
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &msg[i],
                setup->nLenInBytes, NULL, 0, ECDSA_PERFORM_MEM_FREE());
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &pZ[i],
                setup->nLenInBytes, NULL, 0, ECDSA_PERFORM_MEM_FREE());
        /*sign the message (the message is populated within the ecdsaSignRS
         * function with random data*/

        status = ecdsaSignRS(setup,
                &privateKey,
                &pR[i],
                &pS[i],
                &msg[i],
                &pZ[i],
                NULL,
                NULL);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("ecdsaSignRS error %d\n", status);
            ECDSA_PERFORM_MEM_FREE();
            return status;
        }
    }

    /*verify the signatures to the messages*/

    status = allocArrayOfVirtPointers(setup->cyInstanceHandle,
            (void**)&vOpData, setup->numBuffers);
    if(CPA_STATUS_SUCCESS !=status)
    {
        PRINT_ERR("vOpData mem allocation error\n");
        ECDSA_PERFORM_MEM_FREE();
        return CPA_STATUS_FAIL;
    }

    /*allocate and populate all the verifyOpData buffers*/
    for(i = 0; i < setup->numBuffers; i++)
    {
        vOpData[i] = qaeMemAlloc(sizeof(CpaCyEcdsaVerifyOpData));
        if(NULL == vOpData[i])
        {
            PRINT_ERR("vOpData[%u] memory allocation error\n", i);
            ECDSA_PERFORM_MEM_FREE();
            return status;
        }
        memset(vOpData[i],0,sizeof(CpaCyEcdsaVerifyOpData));

        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->a,
                setup->nLenInBytes, setup->pCurve->a,
                setup->pCurve->sizeOfa, ECDSA_PERFORM_MEM_FREE());

        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->b,
                setup->nLenInBytes, setup->pCurve->b,
                setup->pCurve->sizeOfb, ECDSA_PERFORM_MEM_FREE());

        vOpData[i]->fieldType = setup->fieldType;

        /*m pZ contains the digest of msg*/
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->m,
                setup->nLenInBytes, pZ[i].pData,
                pZ[i].dataLenInBytes, ECDSA_PERFORM_MEM_FREE());

        /*http://csrc.nist.gov/groups/ST/toolkit/documents/dss/NISTReCur.pdf:
         * Any point of order r can serve as the base point
         * QA-API uses n value to describe base point*/
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->n,
                setup->nLenInBytes,setup->pCurve->r,
                setup->pCurve->sizeOfr, ECDSA_PERFORM_MEM_FREE());

        /*http://csrc.nist.gov/groups/ST/toolkit/documents/dss/NISTReCur.pdf:
         * use p for either prime modulus for GFP or polynomial for GF2
         * QA API uses q for the same*/
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->q,
                setup->nLenInBytes, setup->pCurve->p,
                setup->pCurve->sizeOfp, ECDSA_PERFORM_MEM_FREE());

        /*r is part of the RS signature*/
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->r,
                setup->nLenInBytes, pR[i].pData,
                pR[i].dataLenInBytes, ECDSA_PERFORM_MEM_FREE());

        /*s is part of the RS signature*/
        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->s,
                setup->nLenInBytes, pS[i].pData,
                pS[i].dataLenInBytes, ECDSA_PERFORM_MEM_FREE());

        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->xg,
                setup->nLenInBytes, setup->pCurve->xg,
                setup->pCurve->sizeOfxg, ECDSA_PERFORM_MEM_FREE());

        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->yg,
                setup->nLenInBytes, setup->pCurve->yg,
                setup->pCurve->sizeOfyg, ECDSA_PERFORM_MEM_FREE());

        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->xp,
                setup->nLenInBytes, pX->pData,
                pX->dataLenInBytes, ECDSA_PERFORM_MEM_FREE());

        ALLOC_FLAT_BUFF_DATA(setup->cyInstanceHandle, &vOpData[i]->yp,
                setup->nLenInBytes, pY->pData,
                pY->dataLenInBytes, ECDSA_PERFORM_MEM_FREE());

    }
    /*this barrier will wait until all threads get to this point*/
    /*set the callback function if asynchronous mode is set*/
    if(ASYNC == setup->syncMode)
    {
        cbFunc = ecdsaPerformCallback;
    }
    pEcdsaData->numOperations = (Cpa64U)setup->numBuffers*setup->numLoops;
    pEcdsaData->responses = 0;
    sampleCodeBarrier();

    /*record the start time, the callback measures the end time when the last
     * response is received*/
    pEcdsaData->startCyclesTimestamp = sampleCodeTimestamp();
    for(numLoops = 0; numLoops < setup->numLoops; numLoops++)
    {
        for(i = 0; i < setup->numBuffers; i++)
        {

            do
            {
                status = cpaCyEcdsaVerify(setup->cyInstanceHandle,
                        cbFunc,
                        pEcdsaData,
                        vOpData[i],
                        &verifyStatus);
                if(CPA_STATUS_RETRY == status)
                {
                    pEcdsaData->retries++;
                    /*if the acceleration engine is busy pause for a
                     * moment by making a context switch*/
                    if(RETRY_LIMIT == (pEcdsaData->retries % (RETRY_LIMIT+1)))
                    {
                        AVOID_SOFTLOCKUP;
                    }
                }
            } while (CPA_STATUS_RETRY == status);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("ECDSA Verify function failed with status:%d\n",
                        status);
                ECDSA_PERFORM_MEM_FREE();
                return status;
            }
            if(ASYNC != setup->syncMode)
            {
                if(CPA_TRUE != verifyStatus)
                {
                    PRINT_ERR("ECDSA Verify function verification failed "
                                    "but status = %d\n", status);
                    status = CPA_STATUS_FAIL;
                }
                /*else {
                    PRINT_ERR("ECDSA Verify function verification "
                                    "succeeded\n");
                }*/
            }
        } /*end buffers loop */
    } /* end of numLoops loop*/
    if (CPA_STATUS_SUCCESS == status)
    {
        status = waitForResponses(pEcdsaData, setup->syncMode,
                setup->numBuffers,
                setup->numLoops);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("Thread %u timeout. Going into infinite while loop!!\n",
                    setup->threadID);
            while(1)
            {
                AVOID_SOFTLOCKUP;
            }
        }
    }

    sampleCodeSemaphoreDestroy(&pEcdsaData->comp);
    /*Free all memory*/
    ECDSA_PERFORM_MEM_FREE();
    if(CPA_STATUS_SUCCESS != setup->performanceStats->threadReturnStatus)
    {
        status = CPA_STATUS_FAIL;
    }
    return status;
}


#ifndef NEWDISPLAY
/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      Print the performance stats of the elliptic curve dsa operations
***************************************************************************/
void ecdsaPrintStats(thread_creation_data_t* data)
{
    PRINT("ECDSA VERIFY\n");
    PRINT("EC Size %23u\n",data->packetSize);
    printAsymStatsAndStopServices(data);
}
#endif
/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      setup an elliptic curve performance thread
***************************************************************************/
void ecdsaPerformance(single_thread_test_data_t* testSetup)
{
    ecdsa_test_params_t ecdsaSetup;
    Cpa16U numInstances = 0;
    CpaInstanceHandle *cyInstances = NULL;
    CpaStatus status = CPA_STATUS_FAIL;
    ecdsa_test_params_t* params = (ecdsa_test_params_t*)testSetup->setupPtr;
    CpaInstanceInfo2 instanceInfo = {0};

    /*this barrier is to halt this thread when run in user space context, the
     * startThreads function releases this barrier, in kernel space it does
     * nothing, but kernel space threads do not start until we call
     * startThreads anyway*/
    startBarrier();
    /*give our thread a unique memory location to store performance stats*/
    ecdsaSetup.performanceStats = testSetup->performanceStats;
    /*get the instance handles so that we can start our thread on the selected
    * instance*/
    status = cpaCyGetNumInstances(&numInstances);
    if( CPA_STATUS_SUCCESS != status  || numInstances == 0)
    {
        PRINT_ERR("cpaCyGetNumInstances error, status:%d, numInstanaces:%d\n",
                status, numInstances);
        ecdsaSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        sampleCodeThreadExit();
    }
    cyInstances = qaeMemAlloc(sizeof(CpaInstanceHandle)*numInstances);
    if(cyInstances == NULL)
    {
        PRINT_ERR("Error allocating memory for instance handles\n");
        ecdsaSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        sampleCodeThreadExit();
    }
    if(cpaCyGetInstances(numInstances, cyInstances) != CPA_STATUS_SUCCESS)
    {
        PRINT_ERR("Failed to get instances\n");
        ecdsaSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        qaeMemFree((void**)&cyInstances);
        sampleCodeThreadExit();
    }

    /* give our thread a logical crypto instance to use
     * use % to wrap around the max number of instances*/
    ecdsaSetup.cyInstanceHandle =
            cyInstances[(testSetup->logicalQaInstance)%numInstances];

    status = cpaCyInstanceGetInfo2(ecdsaSetup.cyInstanceHandle,
                &instanceInfo);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("%s::%d cpaCyInstanceGetInfo2 failed", __func__,__LINE__);
        qaeMemFree((void**)&cyInstances);
        ecdsaSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
        sampleCodeThreadExit();
    }
    if(instanceInfo.physInstId.packageId > packageIdCount_g)
    {
        packageIdCount_g = instanceInfo.physInstId.packageId;
    }

    ecdsaSetup.threadID = testSetup->threadID;


    ecdsaSetup.nLenInBytes = params->nLenInBytes;
    ecdsaSetup.fieldType = params->fieldType;
    ecdsaSetup.numBuffers = params->numBuffers;
    ecdsaSetup.numLoops = params->numLoops;
    ecdsaSetup.syncMode = params->syncMode;
    /*launch function that does all the work*/

    switch(params->step)
    {
    case(ECDSA_STEP_VERIFY):
        status = ecdsaPerform(&ecdsaSetup);
        break;
    default:
        PRINT_ERR("Function not supported for step %d\n",params->step);
        status = CPA_STATUS_FAIL;
        break;
    }
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT("ECDSA Thread %u FAILED\n", testSetup->threadID);
        ecdsaSetup.performanceStats->threadReturnStatus = CPA_STATUS_FAIL;
    }
    else
    {
         /*set the print function that can be used to print stats at the end of
         * the test*/
        testSetup->statsPrintFunc = (stats_print_func_t)ecdsaPrintStats;
    }
    qaeMemFree((void**)&cyInstances);
    sampleCodeThreadExit();
}

/***************************************************************************
 * @ingroup cryptoThreads
 *
 * @description
 *      This function is used to set the parameters to be used in the elliptic
 *      curve performance thread. It is called before the createThreads
 *      function of the framework. The framework replicates it across many
 *      cores
***************************************************************************/
CpaStatus setupEcdsaTest(Cpa32U nLenInBits, CpaCyEcFieldType fieldType,
        sync_mode_t syncMode, ecdsa_step_t step, Cpa32U numBuffers,
        Cpa32U numLoops)
{
    /* testSetupData_g is a multi-dimensional array that stores the setup for
     * all thread variations in an array of characters. we store our test setup
     * at the start of the second array ie index 0. There maybe multi thread
     * types (setups) running as counted by testTypeCount_g*/

    /*as setup is a multi-dimensional char array we need to cast it to the
     * symmetric structure*/
    ecdsa_test_params_t* ecdsaSetup = NULL;

    if(testTypeCount_g >= MAX_THREAD_VARIATION)
    {
        PRINT_ERR("Maximum Support Thread Variation has been exceeded\n");
        PRINT_ERR("Number of Thread Variations created: %d",
                testTypeCount_g);
        PRINT_ERR(" Max is %d\n", MAX_THREAD_VARIATION);
        return CPA_STATUS_FAIL;
    }
    /*start crypto service if not already started*/
    if(CPA_STATUS_SUCCESS != startCyServices())
    {
        PRINT_ERR("Error starting Crypto Services\n");
        return CPA_STATUS_FAIL;
    }
    if(!poll_inline_g)
    {
        /* start polling threads if polling is enabled in the configuration file */
        if(CPA_STATUS_SUCCESS != cyCreatePollingThreadsIfPollingIsEnabled())
        {
            PRINT_ERR("Error creating polling threads\n");
            return CPA_STATUS_FAIL;
        }
    }
    ecdsaSetup =
        (ecdsa_test_params_t*)&thread_setup_g[testTypeCount_g][0];
    testSetupData_g[testTypeCount_g].performance_function =
        (performance_func_t)ecdsaPerformance;
    testSetupData_g[testTypeCount_g].packetSize = nLenInBits;
    /*if nLenInBits is not an even number of bytes then round up
     * ecdsaSetup->nLenInBytes*/
    ecdsaSetup->nLenInBytes = (nLenInBits + NUM_BITS_IN_BYTE - 1) /
        NUM_BITS_IN_BYTE;
    ecdsaSetup->fieldType = fieldType;
    ecdsaSetup->syncMode = syncMode;
    ecdsaSetup->numBuffers = numBuffers;
    ecdsaSetup->numLoops = numLoops;
    ecdsaSetup->step = step;
    return CPA_STATUS_SUCCESS;

}
