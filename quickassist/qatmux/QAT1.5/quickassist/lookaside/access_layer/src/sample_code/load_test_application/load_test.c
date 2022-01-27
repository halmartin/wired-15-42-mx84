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


/******************************************************************************
 *
 * load_test.c
 *
 * Re-uses cpa_sample_code functionality to execute five distinct tests that are
 * replicated across available logical QuickAssist(QA) instances using the
 * sample code multi-threaded framework.
 * The tests are:
 *          AES128-CBC using 4k buffers.
 *          Authenticated SHA512 using 4k buffers.
 *          RSA 1024.
 *          Dynamic Deflate Compression using 8k buffers.
 *          True Random Number Generator 128bit
 * These tests are designed to target individual hardware engines via the QA
 * instance handles available and allow for TDP measurements to be taken.
 *
 * The targeting policy is enforced by the configuration files supplied
 * (dh89xxcc_qa_dev<N>.conf).
 *
 * Enlarge the max threads number in setup_load_test_env.sh(OS dependent):
 *          echo 11165530 > /proc/sys/vm/max_map_count
 *
 *
 *****************************************************************************/
#include "cpa_sample_code_crypto_utils.h"
#include "cpa_sample_code_dc_perf.h"
#include "cpa_sample_code_dc_utils.h"
#include "cpa_sample_code_framework.h"
#include "cpa_sample_code_nrbg_perf.h"

#include "qae_mem.h"

#ifdef USER_SPACE
#include <string.h>
#include "icp_sal_user.h"
#endif


#define NUM_SYMMETRIC_BUFFERS           (10)
#define NUM_ASYMMETRIC_BUFFERS          (10)

#define NUM_CY_FUNCTIONS                (9)
#define NUM_DC_FUNCTIONS                (1)
#define SINGLE_CORE                     (1)
#define SINGLE_INSTANCE                 (1)

#define NUM_DC_THREADS                  (4)
#define NUM_PKE_THREADS                 (4)
#define NUM_CIPHER_THREADS              (2)
#define NUM_AUTH_THREADS                (2)
#define NUM_TRNG_THREADS                (2)
#define NUM_UNSET                       (0)
#define NUM_LOOPS                       (0x7FFFFFFF)


/******************************************************************************
 *
 * Declared types for load test application
 *
 *****************************************************************************/

#ifndef USER_SPACE

#define CLI_OPT_LEN             	(25)
typedef struct option_s {
    const char optName[CLI_OPT_LEN];
    int optValue;
} option_t;

#endif

Cpa32U numLoops, numDc, numPke, numCipher, numAuth, numAlgchain, numTrng,
        symPacketSize, dcPacketSize;

option_t optArray[] = {
            {"numDc", NUM_DC_THREADS},
            {"numPke", NUM_PKE_THREADS},
            {"numCipher", NUM_CIPHER_THREADS},
            {"numAuth", NUM_AUTH_THREADS},
            {"numAlgchain", NUM_UNSET},
            {"numTrng", NUM_TRNG_THREADS},
            {"numLoops", NUM_LOOPS},
            {"symPacketSize", BUFFER_SIZE_2048},
            {"dcPacketSize", BUFFER_SIZE_4096},
        };

typedef enum _LoadTestService
{
    LOAD_TEST_ALL,
    LOAD_TEST_DC,
    LOAD_TEST_CIPHER,
    LOAD_TEST_AUTH,
    LOAD_TEST_PKE,
    LOAD_TEST_TRNG,
}LoadTestService;

LoadTestService serviceMode;

typedef enum inst_type_s
{
    DC = 0,
    CY
} inst_type_t;

typedef struct load_test_inst_info_s
{
    Cpa16U instanceNumber;
    CpaInstanceInfo2 instanceInfo;
    Cpa32U coreAffinity;

}load_test_inst_info_t;

typedef CpaStatus (*ptr2SetupFunction)( load_test_inst_info_t );

typedef CpaStatus (*ptr2GetInstancesFn)(Cpa16U , CpaInstanceHandle* );

typedef CpaStatus (*ptr2GetInfo2Fn)(const CpaInstanceHandle ,
                                    CpaInstanceInfo2 *);

/******************************************************************************
 *
 * External functions from sample code framework
 *
 *****************************************************************************/
#ifdef USER_SPACE
extern CpaStatus qaeMemInit(void);
extern void qaeMemDestroy(void);



/******************************************************************************
 *
 * Initialise SAL and User/Kernel memory mapping driver
 *
 *****************************************************************************/

static CpaStatus loadTestInit(void)
{

    if(CPA_STATUS_SUCCESS != qaeMemInit())
    {
        PRINT_ERR("Could not start qae mem for user space\n");
        PRINT("Has the qae memory module been loaded?\n");
        return CPA_STATUS_FAIL;
    }

    PRINT("Initializing user space \"LoadTest\" instances...\n");
    if(CPA_STATUS_SUCCESS != icp_sal_userStart("LoadTest"))
    {
        PRINT_ERR("Could not start sal for user space\n");
        qaeMemDestroy();
        return CPA_STATUS_FAIL;
    }
    else
    {
        PRINT("\"LoadTest\" instances initialization completed\n");
    }
    return CPA_STATUS_SUCCESS;
}

/******************************************************************************
 *
 * Shutdown SAL and exit User/Kernel memory mapping driver
 *
 *****************************************************************************/
static void loadTestExit(void)
{
    icp_sal_userStop();
    qaeMemDestroy();
}

#endif
/******************************************************************************
 *
 * Get logical instance to core mapping
 *
 *****************************************************************************/
static CpaStatus getCoreAffinity(load_test_inst_info_t * const pInstanceInfo)
{
    Cpa32U i = 0;

    for(i = 0; i < CPA_MAX_CORES; i++)
    {
        if(CPA_BITMAP_BIT_TEST(pInstanceInfo->instanceInfo.coreAffinity,i))
        {
            pInstanceInfo->coreAffinity = i;
            return CPA_STATUS_SUCCESS;
        }
    }
    PRINT_ERR("Could not find core affinity\n");
    return CPA_STATUS_FAIL;
}

/******************************************************************************
 *
 * Get logical instance to core and physical device mapping
 *
 *****************************************************************************/
static CpaStatus getInstanceInfo(load_test_inst_info_t * const pInstanceInfo,
                                    const Cpa16U numInstances,
                                    const inst_type_t instType)
{
    CpaInstanceHandle *pInstanceHandles = NULL;
    Cpa16U i = 0;
    CpaStatus status = CPA_STATUS_FAIL;
    ptr2GetInfo2Fn getInstanceInfo2;
    ptr2GetInstancesFn getInstances;

    switch(instType)
    {
        case(DC):
            getInstanceInfo2 = cpaDcInstanceGetInfo2;
            getInstances = cpaDcGetInstances;
            break;
        case(CY):
            getInstanceInfo2 = cpaCyInstanceGetInfo2;
            getInstances = cpaCyGetInstances;
            break;
        default:
            PRINT_ERR("Unknown Instance Type\n");
            return CPA_STATUS_FAIL;
    }

    pInstanceHandles = qaeMemAlloc(sizeof(CpaInstanceHandle)*numInstances);
    if(NULL == pInstanceHandles)
    {
        PRINT_ERR("Cannot allocate memory for Instance Handles\n");
        return CPA_STATUS_FAIL;
    }

    if(CPA_STATUS_SUCCESS !=
            getInstances(numInstances, pInstanceHandles))
    {
        PRINT_ERR("Get Instances failed\n");
        qaeMemFree((void**)&pInstanceHandles);
        return CPA_STATUS_FAIL;
    }
    for(i = 0; i < numInstances; i++)
    {
        status = getInstanceInfo2(pInstanceHandles[i],
                                    &pInstanceInfo[i].instanceInfo);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("could not get instance info\n");
            qaeMemFree((void**)&pInstanceHandles);
            return CPA_STATUS_FAIL;
        }
        status = getCoreAffinity(&pInstanceInfo[i]);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("getCoreAffinity failed\n");
            qaeMemFree((void**)&pInstanceHandles);
            return CPA_STATUS_FAIL;
        }

        pInstanceInfo[i].instanceNumber = i;
    }
    qaeMemFree((void**)&pInstanceHandles);
    return CPA_STATUS_SUCCESS;
}


/******************************************************************************
 *
 * AES128-CBC test targeting Cipher Engines
 *
 *****************************************************************************/
static CpaStatus setupAESCipherTest(load_test_inst_info_t instanceInfo)
{
    CpaStatus status = CPA_STATUS_FAIL;
    status = setupCipherTest(
            CPA_CY_SYM_CIPHER_AES_CBC,
            KEY_SIZE_128_IN_BYTES,
            CPA_CY_PRIORITY_NORMAL,
            ASYNC,
            symPacketSize,
            DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
            NUM_SYMMETRIC_BUFFERS,
            numLoops);
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error setting up Cipher(AES_CBC) Test with status %d\n",
                    status);
        return status;
    }
    PRINT("Creating Cipher(AES_CBC) thread:      instance %3d, core %2d, "
            "device %2d, accelerator %2d, engine %2d\n",
            instanceInfo.instanceNumber,
            instanceInfo.coreAffinity,
            instanceInfo.instanceInfo.physInstId.packageId,
            instanceInfo.instanceInfo.physInstId.acceleratorId,
            instanceInfo.instanceInfo.physInstId.executionEngineId);

    status = createPerfomanceThreads( SINGLE_CORE, &instanceInfo.coreAffinity,
                                      SINGLE_INSTANCE,
                                      instanceInfo.instanceNumber );
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error creating Cipher Test threads with status %d\n",
                    status);
    }
    return status;
}


/******************************************************************************
 *
 * Authenticated Hash(SHA512) test targeting Auth Engines
 *
 *****************************************************************************/
static CpaStatus setupAuthTest(load_test_inst_info_t instanceInfo)
{
    CpaStatus status = CPA_STATUS_FAIL;
    status = setupHashTest(
            CPA_CY_SYM_HASH_SHA512,
            CPA_CY_SYM_HASH_MODE_AUTH,
            SHA512_DIGEST_LENGTH_IN_BYTES,
            CPA_CY_PRIORITY_HIGH,
            ASYNC,
            symPacketSize,
            NUM_SYMMETRIC_BUFFERS,
            numLoops);
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error setting up Hash(SHA512) Test with status %d\n",status);
        return status;
    }

    PRINT("Creating Hash(Auth SHA512) thread:    instance %3d, core %2d, "
            "device %2d, accelerator %2d, engine %2d\n",
            instanceInfo.instanceNumber,
            instanceInfo.coreAffinity,
            instanceInfo.instanceInfo.physInstId.packageId,
            instanceInfo.instanceInfo.physInstId.acceleratorId,
            instanceInfo.instanceInfo.physInstId.executionEngineId);

    status = createPerfomanceThreads( SINGLE_CORE, &instanceInfo.coreAffinity,
                                      SINGLE_INSTANCE,
                                      instanceInfo.instanceNumber );
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error creating Hash(SHA512) thread with status %d\n",status);
    }
    return status;
}


/******************************************************************************
 *
 * Algorithm Chain Test with Cipher Engines and Auth Engines combined
 *
 *****************************************************************************/
static CpaStatus setupAlgorithmChainTest(load_test_inst_info_t instanceInfo)
{
    CpaStatus status = CPA_STATUS_FAIL;
    status = setupSymmetricTest(CPA_CY_SYM_OP_ALGORITHM_CHAINING,
                                CPA_CY_SYM_CIPHER_AES_CBC,
                                KEY_SIZE_256_IN_BYTES,
                                NOT_USED,
                                CPA_CY_PRIORITY_HIGH,
                                CPA_CY_SYM_HASH_SHA512,
                                CPA_CY_SYM_HASH_MODE_AUTH,
                                SHA512_DIGEST_LENGTH_IN_BYTES,
                                CPA_CY_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH,
                                CPA_TRUE,
                                NULL,
                                symPacketSize,
                                DEFAULT_CPA_FLAT_BUFFERS_PER_LIST,
                                100,
                                numLoops,
                                CPA_TRUE);
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error setting up Algorithm Chain Test(AES-CBC-256 + SHA-512)"
            "with status %d\n",status);
        return status;
    }
    PRINT("Creating Algorithm Chain(AES-CBC-256 + SHA-512) thread:  "
        "  instance %3d, core %2d, device %2d, accelerator %2d, engine %2d\n",
            instanceInfo.instanceNumber,
            instanceInfo.coreAffinity,
            instanceInfo.instanceInfo.physInstId.packageId,
            instanceInfo.instanceInfo.physInstId.acceleratorId,
            instanceInfo.instanceInfo.physInstId.executionEngineId);

    status = createPerfomanceThreads( SINGLE_CORE, &instanceInfo.coreAffinity,
                                      SINGLE_INSTANCE,
                                      instanceInfo.instanceNumber );
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error creating Hash(SHA512) thread with status %d\n",status);
    }
    return status;
}


/******************************************************************************
 *
 * RSA mod 1024 Type 2 test targeting PKE Engines
 *
 *****************************************************************************/
static CpaStatus setupPkeTest(load_test_inst_info_t instanceInfo)
{
    CpaStatus status = CPA_STATUS_FAIL;
    status = setupRsaTest(MODULUS_1024_BIT,
            CPA_CY_RSA_PRIVATE_KEY_REP_TYPE_2,
            ASYNC,
            NUM_ASYMMETRIC_BUFFERS,
            numLoops );
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error setting up RSA Test with status %d\n",status);
        return status;
    }

    PRINT("Creating RSA(mod 1024 Type 2) thread: instance %3d, core %2d, "
            "device %2d, accelerator %2d, engine %2d\n",
            instanceInfo.instanceNumber,
            instanceInfo.coreAffinity,
            instanceInfo.instanceInfo.physInstId.packageId,
            instanceInfo.instanceInfo.physInstId.acceleratorId,
            instanceInfo.instanceInfo.physInstId.executionEngineId);

    status = createPerfomanceThreads( SINGLE_CORE, &instanceInfo.coreAffinity,
                                      SINGLE_INSTANCE,
                                      instanceInfo.instanceNumber );
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error creating RSA(mod 1024) Test threads with status %d\n",
                status);
    }
    return status;
}

static CpaStatus setupTrngTest(load_test_inst_info_t instanceInfo)
{
    CpaStatus status = CPA_STATUS_FAIL;
    status = setupNrbgTest(128, ASYNC, 1, numLoops);
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error setting up Nrbg Test with status %d\n",status);
        return status;
    }

    PRINT("Creating Nrbg thread:\t\t      instance %3d, core %2d, "
            "device %2d, accelerator %2d, engine %2d\n",
            instanceInfo.instanceNumber,
            instanceInfo.coreAffinity,
            instanceInfo.instanceInfo.physInstId.packageId,
            instanceInfo.instanceInfo.physInstId.acceleratorId,
            instanceInfo.instanceInfo.physInstId.executionEngineId);

    status = createPerfomanceThreads( SINGLE_CORE, &instanceInfo.coreAffinity,
                                      SINGLE_INSTANCE,
                                      instanceInfo.instanceNumber );
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error creating Nrbg Test threads with status %d\n",
                status);
    }
    return status;
}

/******************************************************************************
 *
 * Dynamic Deflate 8k (Calgary Corpus) test targeting Compression and
 * Translator Engines
 *
 *****************************************************************************/
static CpaStatus setupCompressionTest(load_test_inst_info_t instanceInfo)
{
    CpaBoolean dynamicEnabled = CPA_FALSE;
    CpaDcHuffType huffType = CPA_DC_HT_STATIC;
    CpaStatus status = CPA_STATUS_FAIL;

    dynamicHuffmanEnabled(NULL, &dynamicEnabled);
    if(dynamicEnabled)
    {
        huffType = CPA_DC_HT_FULL_DYNAMIC;
    }
    status = setupDcTest( CPA_DC_DEFLATE,
            CPA_DC_DIR_COMPRESS,
            SAMPLE_CODE_CPA_DC_L1,
            huffType,
            CPA_DC_FT_ASCII,
            CPA_DC_STATELESS,
            DEFAULT_COMPRESSION_WINDOW_SIZE,
            BUFFER_SIZE_8192,
            SIGN_OF_LIFE_CORPUS,
            CPA_SAMPLE_ASYNCHRONOUS,
            numLoops);

    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error setting up Compression Test with status %d\n",
                    status);
        return status;
    }

    PRINT("Creating Compression thread:          instance %3d, core %2d, "
            "device %2d, accelerator %2d\n",
            instanceInfo.instanceNumber,
            instanceInfo.coreAffinity,
            instanceInfo.instanceInfo.physInstId.packageId,
            instanceInfo.instanceInfo.physInstId.acceleratorId);

    status = createPerfomanceThreads( SINGLE_CORE, &instanceInfo.coreAffinity,
                                    SINGLE_INSTANCE,
                                    instanceInfo.instanceNumber );
    if( CPA_STATUS_SUCCESS != status )
    {
        PRINT_ERR("Error creating Compression Test threads with status %d\n",
                    status);
    }
    return status;
}

/******************************************************************************
 *
 * Setup the compression test threads.
 *
 *****************************************************************************/
static CpaStatus setupCompressionThreads(Cpa16U * const numInstancesFound)
{
    CpaStatus status = CPA_STATUS_FAIL;
    load_test_inst_info_t *pDcInstanceInfo;
    Cpa16U numDcInstances = 0, i = 0;

    if(CPA_STATUS_SUCCESS != cpaDcGetNumInstances(&numDcInstances))
    {
        PRINT_ERR("cpaDcGetNumInstances failed\n");
        *numInstancesFound = 0;
        return CPA_STATUS_FAIL;
    }

    *numInstancesFound = numDcInstances;
    /* If no instances are available, print a warning as device under test may
     * not have Compression available and has been disabled.
     */

    if(0 == numDcInstances)
    {
        PRINT("Warning: No Data Compression Instances present\n");
        return CPA_STATUS_SUCCESS;
    }

    pDcInstanceInfo = qaeMemAlloc(sizeof(load_test_inst_info_t)*numDcInstances);
    if(NULL == pDcInstanceInfo)
    {
        PRINT_ERR("Error allocating Data Compression Instance info\n");
        return CPA_STATUS_FAIL;
    }

    if(CPA_STATUS_SUCCESS !=
            getInstanceInfo(pDcInstanceInfo,numDcInstances,DC))
    {
        PRINT_ERR("Cannot determine the mapping of Data Compression instances "
                "to cores and HW device information\n");
        qaeMemFree((void**)&pDcInstanceInfo);
        return CPA_STATUS_FAIL;
    }

    PRINT("Creating Compression Tests across %d logical instances\n",
            numDcInstances);
    
    for(i = 0; i < numDc; i++)
    {
        status = setupCompressionTest(pDcInstanceInfo[i%numDcInstances]);
        if( CPA_STATUS_SUCCESS != status )
        {
            PRINT_ERR("Error calling Data Compression setup function\n");
            qaeMemFree((void**)&pDcInstanceInfo);
            return CPA_STATUS_FAIL;
        }
    }
    qaeMemFree((void**)&pDcInstanceInfo);
    return CPA_STATUS_SUCCESS;
}


/******************************************************************************
 *
 * Setup the crypto test threads.
 *
 *****************************************************************************/
static CpaStatus setupCryptoThreads(Cpa16U * const numInstancesFound)
{
    /* Declare and initialise local and status variables */
    CpaStatus status = CPA_STATUS_FAIL;
    load_test_inst_info_t *pCyInstanceInfo;
    Cpa16U numCyInstances = 0, i = 0, instNum = 0;

    if(CPA_STATUS_SUCCESS != cpaCyGetNumInstances(&numCyInstances))
    {
        PRINT_ERR("cpaDcGetNumInstances failed\n");
        *numInstancesFound = 0;
        return CPA_STATUS_FAIL;
    }
    *numInstancesFound = numCyInstances;

    /* If no instances are available, print a warning as device under test may
     * have Crypto disabled.
     */
    if(0 == numCyInstances)
    {
        PRINT("Warning No Crypto Instances present\n");
        return CPA_STATUS_SUCCESS;
    }
    pCyInstanceInfo = qaeMemAlloc(sizeof(load_test_inst_info_t)*numCyInstances);
    if(NULL == pCyInstanceInfo)
    {
        PRINT_ERR("Error allocating Crypto Instance info\n");
        return CPA_STATUS_FAIL;
    }

    if(CPA_STATUS_SUCCESS !=
            getInstanceInfo(pCyInstanceInfo,numCyInstances,CY))
    {
     
   PRINT_ERR("Cannot determine the mapping of Crypto instances "
                "to cores\n");
        qaeMemFree((void**)&pCyInstanceInfo);
        return CPA_STATUS_FAIL;
    }
    
    PRINT("Creating Crypto Tests across %d logical instances\n",
            numCyInstances);
    for(i = 0; i < numPke; i++)
    {
        instNum = instNum % numCyInstances;
        status = setupPkeTest(pCyInstanceInfo[instNum++]);
        if( CPA_STATUS_SUCCESS != status )
        {
            PRINT_ERR("Error calling Crypto setup function\n");
            qaeMemFree((void**)&pCyInstanceInfo);
            return CPA_STATUS_FAIL;
        }
    }
    for(i = 0; i < numCipher; i++)
    {
        instNum = instNum % numCyInstances;
        status = setupAESCipherTest(pCyInstanceInfo[instNum++]);
        if( CPA_STATUS_SUCCESS != status )
        {
            PRINT_ERR("Error calling Crypto setup function\n");
            qaeMemFree((void**)&pCyInstanceInfo);
            return CPA_STATUS_FAIL;
        }
    }  
    for(i = 0; i < numAuth; i++)
    {
        instNum = instNum % numCyInstances;
        status = setupAuthTest(pCyInstanceInfo[instNum++]);
        if( CPA_STATUS_SUCCESS != status )
        {
            PRINT_ERR("Error calling Crypto setup function\n");
            qaeMemFree((void**)&pCyInstanceInfo);
            return CPA_STATUS_FAIL;
        }
    }
    for(i = 0; i < numTrng; i++)
    {
        if(!(strncmp((const char *)
            pCyInstanceInfo[i%numCyInstances].instanceInfo.partName,
            "dh89xxcc",strlen("dh89xxcc"))) ||
            !(strncmp((const char *)
            pCyInstanceInfo[i%numCyInstances].instanceInfo.partName,
            "dh895xcc",strlen("dh895xcc"))))
        {
            instNum = instNum % numCyInstances;
            status = setupTrngTest(pCyInstanceInfo[instNum++]);
            if( CPA_STATUS_SUCCESS != status )
            {
                PRINT_ERR("Error calling Crypto setup function\n");
                qaeMemFree((void**)&pCyInstanceInfo);
                return CPA_STATUS_FAIL;
            }
        }
    }
    for(i = 0; i < numAlgchain; i++)
    {
        status = setupAlgorithmChainTest(pCyInstanceInfo[i%numCyInstances]);
        if( CPA_STATUS_SUCCESS != status )
        {
            PRINT_ERR("Error calling Crypto setup function\n");
            qaeMemFree((void**)&pCyInstanceInfo);
            return CPA_STATUS_FAIL;
        }
    }
    qaeMemFree((void**)&pCyInstanceInfo);
    return CPA_STATUS_SUCCESS;
}


/******************************************************************************
 *
 * Execute test threads
 *
 *****************************************************************************/
CpaStatus runTests(void)
{
    Cpa16U numDcInstances = 0, numCyInstances = 0;
    if(!(numTrng == 0 && numPke == 0 && numCipher ==0 && numAuth == 0 
        && numAlgchain == 0))
    {
        if(CPA_STATUS_SUCCESS != setupCryptoThreads(&numCyInstances))
        {
            PRINT_ERR("Error setting up Crypto tests\n");
            return CPA_STATUS_FAIL;
        }
    }
    if(numDc != 0)
    {
        if(CPA_STATUS_SUCCESS != setupCompressionThreads(&numDcInstances))
        {
            PRINT_ERR("Error setting up Compression Tests\n");
            return CPA_STATUS_FAIL;
        }
    }
    if(numDcInstances == 0 && numCyInstances == 0)
    {
        PRINT_ERR("No instances found for both Crypto and Compression "
                    "services\n");
        return CPA_STATUS_FAIL;
    }
    /* Call start threads. Return if fail */
    if( CPA_STATUS_SUCCESS != startThreads() )
    {
        PRINT_ERR("Error starting test threads\n");
        return CPA_STATUS_FAIL;
    }
    PRINT("Starting test execution, press Ctl-c to exit\n");

    return waitForThreadCompletion();
}


#ifdef USER_SPACE
int main(int argc, char **argv)
{
    if(CPA_STATUS_SUCCESS != parseArg(argc, argv,
        optArray, sizeof(optArray)/sizeof(option_t)))
    {
        return 0;
    }
    numDc = optArray[0].optValue;
    numPke = optArray[1].optValue;
    numCipher = optArray[2].optValue;
    numAuth = optArray[3].optValue;
    numAlgchain = optArray[4].optValue;
    numTrng = optArray[5].optValue;
    numLoops = optArray[6].optValue;
    symPacketSize = optArray[7].optValue;
    dcPacketSize = optArray[8].optValue;
    
    if(CPA_STATUS_SUCCESS != loadTestInit())
    {
        PRINT_ERR("Error initialising load test application\n");
        return CPA_STATUS_FAIL;
    }

    if(CPA_STATUS_SUCCESS != runTests())
    {
        PRINT_ERR("Error running load test application\n");
        loadTestExit();
        return CPA_STATUS_FAIL;
    }
    loadTestExit();
    return CPA_STATUS_SUCCESS;
}
#endif
