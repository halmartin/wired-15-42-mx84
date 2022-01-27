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

/*
 * This is sample code that demonstrates usage of the drbg API.
 */

#include "cpa.h"
#include "cpa_cy_drbg.h"

#include "cpa_sample_utils.h"
#include "cpa_nrbg_sample.h"

#include "icp_sal_poll.h"

extern int gDebugParam ;


#define TIMEOUT_MS 5000 /* 5 seconds */

#ifdef USER_SPACE
#define PRINT_OUT printf
#else
#define PRINT_OUT printk
#endif

/*
 *****************************************************************************
 * Forward declaration
 *****************************************************************************
 */
CpaStatus
drbgSample(void);

/*
 * Callback function
 *
 * This function is "called back" (invoked by the implementation of
 * the API) when the asynchronous operation has completed.  The
 * context in which it is invoked depends on the implementation, but
 * as described in the API it should not sleep (since it may be called
 * in a context which does not permit sleeping, e.g. a Linux bottom
 * half).
 *
 * This function can perform whatever processing is appropriate to the
 * application.  For example, it may free memory, continue processing
 * etc.  In this example, the function only sets the complete variable
 * to indicate it has been called.
 */
static void
drbgGenCallback(void *pCallbackTag,
        CpaStatus status,
        void *pOpData,
        CpaFlatBuffer *pOut)
{
    PRINT_DBG("Callback called with status = %d.\n", status);

    if (NULL != pCallbackTag)
    {
        /* indicate that the function has been called */
#ifdef DRBG_POLL_AND_WAIT
        /* Single threaded operation, tread as int */
        *(int *)pCallbackTag = 1;
#else
        COMPLETE((struct COMPLETION_STRUCT *)pCallbackTag);
#endif
    }
}


static void
drbgReseedCallback(void *pCallbackTag,
        CpaStatus status,
        void *pOpData,
        CpaFlatBuffer *pOut)
{
    PRINT_DBG("Reseed Callback called with status = %d.\n", status);
    if (NULL != pCallbackTag)
    {
        /* indicate that the function has been called */
#ifdef DRBG_POLL_AND_WAIT
        /* Single threaded operation, tread as int */
        *(int *)pCallbackTag = 1;
#else
        COMPLETE((struct COMPLETION_STRUCT *)pCallbackTag);
#endif
    }
}


/*
 * This function performs a drbg generate operation.
 */
static CpaStatus
drbgPerformOp(CpaInstanceHandle cyInstHandle, CpaCyDrbgSessionHandle sessionHdl)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa8U *pDrbgData = NULL;
    CpaCyDrbgGenOpData *pOpData = NULL;
    Cpa32U drbgDataSize = 128;
    int i=0;

#ifdef DRBG_POLL_AND_WAIT
    /* Single threaded operation, use int for completion rather than a semaphore */
    Cpa32U complete = 0;
#else
    /* The following variables are allocated on the stack because we block
     * until the callback comes back. If a non-blocking approach was to be
     * used then these variables should be dynamically allocated */
    struct COMPLETION_STRUCT complete;
#endif
    CpaFlatBuffer drbgOut;

    status = PHYS_CONTIG_ALLOC(&pDrbgData, drbgDataSize);

    if (CPA_STATUS_SUCCESS == status)
    {
        drbgOut.pData = pDrbgData;
        drbgOut.dataLenInBytes = drbgDataSize;

        status = OS_MALLOC(&pOpData, sizeof(CpaCyDrbgGenOpData));
    }

    if (CPA_STATUS_SUCCESS == status)
    {
//<snippet name="opData">
        pOpData->sessionHandle = sessionHdl;
        pOpData->lengthInBytes = drbgDataSize;
        pOpData->secStrength = CPA_CY_RBG_SEC_STRENGTH_128;
        pOpData->predictionResistanceRequired = CPA_FALSE;
        pOpData->additionalInput.dataLenInBytes = 0;
        pOpData->additionalInput.pData = NULL;
//</snippet>
    }

    /*
     * Now, we initialize the completion variable which is used by the callback function
     * to indicate that the operation is complete.  We then perform the operation.
     */
    if (CPA_STATUS_SUCCESS == status)
    {
        PRINT_DBG("cpaCyDrbgGen\n");

//<snippet name="perfOp">
#ifdef DRBG_POLL_AND_WAIT
        complete = 0;
#else
        COMPLETION_INIT(&complete);
#endif

        status = cpaCyDrbgGen(cyInstHandle,
                (void *)&complete, /* data sent as is to the callback function*/
                pOpData,           /* operational data struct */
                &drbgOut);         /* dst buffer list */
//</snippet>

        if (CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("cpaCyDrbgGen failed. (status = %d)\n", status);
        }

#ifdef DRBG_POLL_AND_WAIT
        /*
         * Single threaded operation, do in-line polling
         */
        do
        {
            PRINT_DBG("perform Polling...\n");
            status = icp_sal_CyPollInstance(cyInstHandle, 0);
            OS_SLEEP(10);
        } while(complete==0);
#else
        /*
         * We now wait until the pollign thread to complete
         * the operation.
         */
        if (CPA_STATUS_SUCCESS == status)
        {
            if (!COMPLETION_WAIT(&complete, TIMEOUT_MS))
            {
                PRINT_ERR("timeout or interruption in cpaCyDrbgGen\n");
                status = CPA_STATUS_FAIL;
            }
        }
#endif
        /*
         * Print out random data
         */
         if(CPA_STATUS_SUCCESS == status)
         {
             PRINT_OUT("DRBG output: \n");
             for(i = 0; i < drbgDataSize; i++)
             {
                if (i && (i % 16 == 0))
                {
                    PRINT_OUT("\n");
                }

                PRINT_OUT("%02X", pDrbgData[i]);
              }
              PRINT_OUT("\n");

         }

    }

    /*
     * At this stage, the callback function has returned, so it is
     * sure that the structures won't be needed any more.  Free the
     * memory!
     */
    PHYS_CONTIG_FREE(pDrbgData);
    OS_FREE(pOpData);

#ifndef DRBG_POLL_AND_WAIT
    COMPLETION_DESTROY(&complete);
#endif

    return status;
}

static CpaStatus
drbgReseed(CpaInstanceHandle cyInstHandle, CpaCyDrbgSessionHandle sessionHdl)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaCyDrbgReseedOpData *pOpData = NULL;
#ifdef DRBG_POLL_AND_WAIT
    /* Single threaded operation, use int for completion rather than a semaphore */
    Cpa32U complete = 0;
#else
    /* The following variables are allocated on the stack because we block
     * until the callback comes back. If a non-blocking approach was to be
     * used then these variables should be dynamically allocated */
    struct COMPLETION_STRUCT complete;
#endif

    /* The following variables are allocated on the stack because we block
     * until the callback comes back. If a non-blocking approach was to be
     * used then these variables should be dynamically allocated */

    status = OS_MALLOC(&pOpData, sizeof(CpaCyDrbgGenOpData));

    pOpData->sessionHandle = sessionHdl;
    pOpData->additionalInput.dataLenInBytes = 0;
    pOpData->additionalInput.pData = NULL;

#ifdef DRBG_POLL_AND_WAIT
        complete = 0;
#else
        COMPLETION_INIT(&complete);
#endif

    if (CPA_STATUS_SUCCESS == status)
    {
        PRINT_DBG("cpaCyDrbgReseed\n");
        status = cpaCyDrbgReseed(cyInstHandle,
                (void *)&complete, /* data sent as is to the callback function*/
                pOpData);           /* reseed operational data struct */

        if (CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("cpaCyDrbgReseed failed. (status = %d)\n", status);
            OS_FREE(pOpData);
            return status;
        }
#ifdef DRBG_POLL_AND_WAIT
        /*
         * Single threaded operation, do in-line polling
         */
        do
        {
            status = icp_sal_CyPollInstance(cyInstHandle, 0);
            OS_SLEEP(10);
        } while(complete==0);
#else
        /*
         * We now wait until the pollign thread to complete
         * the operation.
         */
        if (CPA_STATUS_SUCCESS == status)
        {
            if (!COMPLETION_WAIT(&complete, TIMEOUT_MS))
            {
                PRINT_ERR("timeout or interruption in cpaCyDrbgGen\n");
                status = CPA_STATUS_FAIL;
            }
        }
#endif
    }

    PRINT_DBG("Reseed Complete...\n");
    OS_FREE(pOpData);
#ifndef DRBG_POLL_AND_WAIT
    COMPLETION_DESTROY(&complete);
#endif

    return status;
}



#ifdef DRBG_POLL_AND_WAIT_EXTRA
static CpaStatus
drbgHT(CpaInstanceHandle cyInstHandle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U pTestSessionSize = 0;
    IcpSalDrbgTestSessionHandle testSessionHandle = NULL;

    PRINT_OUT("Running Health Test functions...\n");

    status = icp_sal_drbgHTGetTestSessionSize(cyInstHandle,
                                 &pTestSessionSize);

    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("icp_sal_drbgHTGetTestSessionSize failed. (status = %d)\n", status);
        return status;
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* Allocate session */
        status = PHYS_CONTIG_ALLOC(&testSessionHandle, pTestSessionSize);
    }


    status = icp_sal_drbgHTInstantiate(cyInstHandle, testSessionHandle);
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("icp_sal_drbgHTInstantiate failed. (status = %d)\n", status);
        PHYS_CONTIG_FREE(testSessionHandle);
        return status;
    }

    PRINT_OUT("icp_sal_drbgHTInstantiate done\n");

    status = icp_sal_drbgHTGenerate(cyInstHandle, testSessionHandle);
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("icp_sal_drbgHTGenerate failed. (status = %d)\n", status);
        PHYS_CONTIG_FREE(testSessionHandle);
        return status;
    }

    PRINT_OUT("icp_sal_drbgHTGenerate done\n");


    status = icp_sal_drbgHTReseed(cyInstHandle, testSessionHandle);
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("icp_sal_drbgHTReseed failed. (status = %d)\n", status);
        PHYS_CONTIG_FREE(testSessionHandle);
        return status;
    }

    PRINT_OUT("icp_sal_drbgHTReseed done\n");

    PHYS_CONTIG_FREE(testSessionHandle);

    return status;
}
#endif


/*
 * This is the main entry point for the sample drbg code.  It
 * demonstrates the sequence of calls to be made to the API in order
 * to create a session, generate a random number, and
 * then tear down the session.
 */
CpaStatus
drbgSample(void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U sessionSize = 0;
    CpaCyDrbgSessionHandle sessionHdl = NULL;
    CpaInstanceHandle cyInstHandle = NULL;
    CpaCyDrbgSessionSetupData sessionSetupData = {0};
    Cpa32U seedLen = 0;
    CpaCyDrbgStats64 drbgStats = {0};

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

    if (CPA_STATUS_SUCCESS == status)
    {
       /*
        * We don't start the polling thread if the driver has been built with
        * DRBG_POLL_AND_WAIT enabled. This is for single threaded operatio of
        * DRBG.
        */
#ifndef DRBG_POLL_AND_WAIT
       /*
        * If the instance is polled start the polling thread. Note that
        * how the polling is done is implementation-dependant.
        */
        PRINT_DBG("Enabling user polling thread\n");
       sampleCyStartPolling(cyInstHandle);
#endif

       /* Register get entropy functions */
       nrbgRegisterDrbgImplFunctions();

       /*
        * We now populate the fields of the session operational data and create
        * the session.  Note that the size required to store a session is
        * implementation-dependent, so we query the API first to determine how
        * much memory to allocate, and then allocate that memory.
        */
//<snippet name="initSession">
        sessionSetupData.predictionResistanceRequired = CPA_FALSE;
        sessionSetupData.secStrength = CPA_CY_RBG_SEC_STRENGTH_128;
        sessionSetupData.personalizationString.dataLenInBytes = 0;
        sessionSetupData.personalizationString.pData = NULL;

        /* Determine size of session to allocate */
        PRINT_DBG("cpaCyDrbgSessionGetSize\n");
        status = cpaCyDrbgSessionGetSize(cyInstHandle,
                    &sessionSetupData, &sessionSize);
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /* Allocate session */
        status = PHYS_CONTIG_ALLOC(&sessionHdl, sessionSize);
    }

#ifdef DRBG_POLL_AND_WAIT_EXTRA
    status = drbgHT(cyInstHandle);
#endif

    /* Initialize DRBG session */
    if (CPA_STATUS_SUCCESS == status) {
        PRINT_DBG("cpaCyDrbgInitSession\n");
        status = cpaCyDrbgInitSession(cyInstHandle,
                            drbgGenCallback,    /* callback function for generate */
                            (CpaCyGenericCbFunc)drbgReseedCallback, /* callback function for reseed */
                            &sessionSetupData,  /* session setup data */
                            sessionHdl,
                            &seedLen);
    }
//</snippet>

    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus sessionStatus = CPA_STATUS_SUCCESS;

        /* Perform DRBG operation */
        status = drbgPerformOp(cyInstHandle, sessionHdl);
        if (CPA_STATUS_SUCCESS == status)
        {
            PRINT_DBG("PerformOp Done\n");
        } else {
            PRINT_DBG("PerformOp Failed\n");
        }
        /* For NRNG the Prediction Resistance Required is TRUE 
         * and reseed supported not for RDRAND
         */  
        if (sessionSetupData.predictionResistanceRequired == CPA_TRUE)
	{
	    /* Reseed the session */
            status = drbgReseed(cyInstHandle, sessionHdl);
            if (CPA_STATUS_SUCCESS == status)
            {
               PRINT_DBG("Reseed Done\n");
            } else {
               PRINT_DBG("Reseed Failed\n");
            }
	}
        /* Perform DRBG operation after reseed */
        status = drbgPerformOp(cyInstHandle, sessionHdl);
        if (CPA_STATUS_SUCCESS == status)
        {
            PRINT_DBG("PerformOp Done\n");
        } else {
            PRINT_DBG("PerformOp Failed\n");
        }

        /*
         * In a typical usage, the session might be used to generate
         * more random numbers.  In this example however, we
         * can now tear down the session.
         */
//<snippet name="removeSession">
        sessionStatus = cpaCyDrbgRemoveSession(
                cyInstHandle, sessionHdl);
//</snippet>

        /* Maintain status of remove session only when status of all operations
         * before it are successful. */
        if (CPA_STATUS_SUCCESS == status)
        {
            status = sessionStatus;
        }
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        /*
         * We can now query the statistics on the instance.
         */
        status = cpaCyDrbgQueryStats64(cyInstHandle, &drbgStats);

        if (CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("cpaCyDrbgQueryStats64 failed, status = %d\n", status);
        }
        else
        {
            PRINT_DBG("Number of drbg generate operations completed: %llu\n",
                    (unsigned long long)drbgStats.numGenCompleted);
            PRINT_DBG("Number of drbg reseeed operations completed: %llu\n",
                    (unsigned long long)drbgStats.numReseedCompleted);
        }
    }

    /*
     * Free up memory, stop the instance, etc.
     */

    /* Free session Context */
    PHYS_CONTIG_FREE(sessionHdl);

    /* Stop the polling thread */
    sampleCyStopPolling();

    /* Unregister entropy functions */
    nrbgUnregisterDrbgImplFunctions();

    PRINT_DBG("cpaCyStopInstance\n");
    cpaCyStopInstance(cyInstHandle);

    if (CPA_STATUS_SUCCESS == status)
    {
        PRINT_DBG("Sample code ran successfully\n");
    }
    else
    {
        PRINT_DBG("Sample code failed with status of %d\n", status);
    }

    return status;
}


