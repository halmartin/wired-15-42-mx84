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
 * @file cpa_sample_code_crypto_utils.c
 *
 * @ingroup cryptoThreads
 *
 * @description
 *      This file contains utility functions for peformance sample code
 *
 *****************************************************************************/

#include "cpa_sample_code_crypto_utils.h"
#include "cpa_cy_common.h"
#include "cpa_cy_im.h"
#include "cpa_cy_prime.h"
#include "icp_sal_poll.h"
#ifdef NEWDISPLAY
#include "cpa_sample_code_NEWDISPLAY_crypto_utils.h"
#endif
#ifdef LATENCY_CODE
#include <assert.h>
#endif

#define POLL_AND_SLEEP 1

#ifdef USER_SPACE
Cpa32U poll_type_g = 0;
Cpa32U sleep_time_g = 0;
#else
Cpa32U poll_type_g = POLL_AND_SLEEP;
Cpa32U sleep_time_g = DEFAULT_POLL_INTERVAL_MSEC;
#endif


#define NUM_BITS_IN_MEGABIT         (1000000)
#define LOWEST_EVEN_NUMBER          (2)
#define KILOBITS_IN_MEGABITS        (1000)
#define MILLI_SECONDS_IN_SECOND     (1000)
#define SECOND_LAST_BYTE            (2)
#define INC_BY_TWO                  (2)
#define RESPONSE_NOT_CHECKED        (-1)
#define SINGLE_SOCKET               (1)

CpaBoolean running_dsa_g = CPA_FALSE;

/*flag to indicate whether a thread has started the crypto acceleration service
 */
volatile CpaBoolean cy_service_started_g = CPA_FALSE;
volatile CpaBoolean cy_polling_started_g = CPA_FALSE;
CpaCySymCipherDirection cipherDirection_g = CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT;

CpaBoolean digestAppended_g = CPA_FALSE;
CpaInstanceHandle *cyInstances_g = NULL;
Cpa16U numInstances_g = 0;
Cpa16U numPolledInstances_g = 0;
CpaBoolean allocateMemOnOppositeNode = CPA_FALSE;
extern Cpa32U packageIdCount_g;
#ifndef NEWDISPLAY
Cpa32U packetSizes[] = {
        BUFFER_SIZE_64, BUFFER_SIZE_128,BUFFER_SIZE_256,
        BUFFER_SIZE_512,
        BUFFER_SIZE_1024,
        BUFFER_SIZE_2048,  BUFFER_SIZE_4096,
        PACKET_IMIX

        };


Cpa32U numPacketSizes = sizeof(packetSizes)/sizeof(Cpa32U);
EXPORT_SYMBOL(packetSizes);
EXPORT_SYMBOL(numPacketSizes);
#endif
Cpa32U wirelessPacketSizes[] = {BUFFER_SIZE_40, BUFFER_SIZE_64,
        BUFFER_SIZE_256, BUFFER_SIZE_320, BUFFER_SIZE_512,
        BUFFER_SIZE_1024};

Cpa32U numWirelessPacketSizes = sizeof(wirelessPacketSizes)/sizeof(Cpa32U);
EXPORT_SYMBOL(wirelessPacketSizes);
EXPORT_SYMBOL(numWirelessPacketSizes);

Cpa32U modSizes[] = {
        MODULUS_1024_BIT,
        MODULUS_2048_BIT,
        MODULUS_4096_BIT};

Cpa32U numModSizes = sizeof(modSizes)/sizeof(Cpa32U);
EXPORT_SYMBOL(modSizes);
EXPORT_SYMBOL(numModSizes);

extern int signOfLife;
extern int verboseOutput;

#ifdef LATENCY_CODE
extern int latency_debug; /* set to 1 for debug PRINT() */
extern int latency_enable; /* set to 1 for enable latency testing */
extern int isLatencyEnabled();
#endif


sample_code_thread_t* pollingThread_g;

Cpa32U getThroughput(Cpa64U numPackets, Cpa32U packetSize,perf_cycles_t cycles)
{
    unsigned long long bytesSent = 0;
    unsigned long long time = cycles;
    unsigned long long rate = 0;
    /* declare frequency in kiloHertz*/
    Cpa32U freq = sampleCodeGetCpuFreq();
    bytesSent = packetSize;
    bytesSent = bytesSent*numPackets;

    /*get time in milli seconds by dividing numberOfClockCycles by frequency
     * in kilohertz ie: cycles/(cycles/millsec) = time (mSec) */
    do_div(time,freq);
    /*check that the sample time was not to small*/
    if(time ==0)
    {
        PRINT_ERR("Sample time is too small to calculate throughput\n");
        return 0;
    }
    /*set rate to be bytesSent, once we perform the do_div rate changes from
     * bytes to bytes/milli second or kiloBytes/second*/
    rate = bytesSent;
    /*rate in kBps*/
    do_div(rate,time);
    /*check that the rate is high enough to convert to Megabits per second*/
    if(rate ==0)
    {
       PRINT_ERR("no data was sent to calculate throughput\n");
       return 0;
    }
    /* convert Kilobytes/second to Kilobits/second*/
    rate = rate*NUM_BITS_IN_BYTE;
    /*then convert rate from Kilobits/second to Megabits/second*/
    do_div(rate,KILOBITS_IN_MEGABITS);
    return (Cpa32U)rate;
}

Cpa32U getOpsPerSecond(Cpa64U responses, perf_cycles_t cycles)
{
    unsigned long long time = cycles;
    unsigned long long freq = sampleCodeGetCpuFreq();
    /*multiply responses by 1000 so that we dont loose precision
     * precision is lost if we convert millisec to seconds to calculate
     * ops per second ie 2238 ms becomes 2 seconds the decimal place is lost*/
    unsigned long long opsPerSec = responses;
    opsPerSec = opsPerSec * MILLI_SECONDS_IN_SECOND;
    /*convert cycles into time(ms)*/
    do_div(time,freq);
    if(time ==0)
    {
        PRINT_ERR("Sample time is too small to calculate OpsPerSecond\n");
        return 0;
    }
    do_div(opsPerSec,time);
    return opsPerSec;

}
#ifndef NEWDISPLAY

void accumulateAsymPerfData(Cpa32U numberOfThreads,
        perf_data_t* performanceStats[],
        perf_data_t* stats,
        asym_test_params_t *setup,
        Cpa64U* buffersProcessed,
        Cpa32U* responsesPerThread)
{
    Cpa32U i = 0;


    /*accumulate the responses into one perf_data_t structure*/
    for (i=0; i<numberOfThreads; i++)
    {
        stats->responses += performanceStats[i]->responses;
        /*is the data was submitted in multiple buffers per list, then the
         * number of buffers processed is  number of responses multiplied
         * by the numberOfBuffers*/
       *buffersProcessed += performanceStats[i]->responses;
        stats->retries += performanceStats[i]->retries;
        stats->numOperations += performanceStats[i]->numOperations;
        *responsesPerThread = performanceStats[i]->responses;
        clearPerfStats(performanceStats[i]);
    }

}
CpaStatus printAsymStatsAndStopServices(thread_creation_data_t* data)
{
    Cpa32U i=0;


    Cpa32U responsesPerThread = 0;
    perf_cycles_t numOfCycles = 0;
    perf_data_t stats = {0};
/*
    perf_data_t stats2[packageIdCount_g];
    Cpa32U perfDataDeviceOffsets[packageIdCount_g];
    Cpa32U threadCountPerDevice[packageIdCount_g];
*/
    perf_data_t *stats2;
    Cpa32U *perfDataDeviceOffsets;
    Cpa32U *threadCountPerDevice;

    Cpa64U buffersProcessed = 0;
    Cpa32U throughput = 0;
    Cpa32U devThoughput = 0;
    asym_test_params_t *setup= (asym_test_params_t *)data->setupPtr;

    /*stop all crypto instances, There is no other place we can stop CyServices
     * as all other function run in thread context and its not safe to call
     * stopCyServices in thread context, otherwise we could stop threads that
     * have requests in flight. This function is called by the framework
     * after all threads have completed*/
    stopCyServices();
    stats2 = qaeMemAlloc(sizeof( perf_data_t) * (packageIdCount_g + 1));
    if(stats2 == NULL)
    {
        PRINT_ERR("Unable to allocate memory for stats2\n");
        return CPA_STATUS_FAIL;
    }
    perfDataDeviceOffsets = qaeMemAlloc(sizeof(Cpa32U) * (packageIdCount_g + 1));
    if(perfDataDeviceOffsets == NULL)
    {
        qaeMemFree((void **)&stats2);
        PRINT_ERR("Unable to allocate memory for perfDataDeviceOffsets\n");
        return CPA_STATUS_FAIL;
    }

    threadCountPerDevice = qaeMemAlloc(sizeof(Cpa32U) * (packageIdCount_g + 1));
    if(threadCountPerDevice == NULL)
    {
        qaeMemFree((void **)&stats2);
        qaeMemFree((void **)&perfDataDeviceOffsets);
        PRINT_ERR("Unable to allocate memory for threadCountPerDevice\n");
        return CPA_STATUS_FAIL;
    }    

    for(i = 0; i< data->numberOfThreads; i++)
    {
        if(CPA_STATUS_FAIL == data->performanceStats[i]->threadReturnStatus)
        {
            qaeMemFree((void **)&stats2);
            qaeMemFree((void **)&perfDataDeviceOffsets);
            qaeMemFree((void **)&threadCountPerDevice);
            return CPA_STATUS_FAIL;
        }
    }
    memset(stats2, 0, sizeof(perf_data_t)*(packageIdCount_g + 1));
    stats.averagePacketSizeInBytes = data->packetSize;
    getLongestCycleCount2(stats2, data->performanceStats,
                data->numberOfThreads, perfDataDeviceOffsets, threadCountPerDevice);
    for(i=0; i < (packageIdCount_g + 1); i++)
    {
        accumulateAsymPerfData(threadCountPerDevice[i],
                    &(data->performanceStats[perfDataDeviceOffsets[i]]),
                        &stats2[i],
                        setup,
                        &buffersProcessed, &responsesPerThread);

        numOfCycles = (stats2[i].endCyclesTimestamp - stats2[i].startCyclesTimestamp);
        if(!signOfLife) 
        {
                devThoughput = getOpsPerSecond(buffersProcessed, numOfCycles);
        }
        buffersProcessed = 0;
        throughput += devThoughput;
        stats.numOperations += stats2[i].numOperations;
        stats.responses += stats2[i].responses;
        stats.retries += stats2[i].retries;

    }
#ifdef BLOCKOUT
    getLongestCycleCount(&stats, data->performanceStats,data->numberOfThreads);
    /*loop over each thread perf stats and total up the responses
     * also check of any of the threads has a fail status and return fail if
     * this is the case*/
    for (i=0; i<data->numberOfThreads; i++)
    {

        stats.responses += data->performanceStats[i]->responses;
        stats.numOperations += data->performanceStats[i]->numOperations;
        stats.retries += data->performanceStats[i]->retries;
        responsesPerThread = data->performanceStats[i]->responses;
        clearPerfStats(data->performanceStats[i]);
    }
#endif
    numOfCycles = (stats.endCyclesTimestamp - stats.startCyclesTimestamp);
    PRINT("Number of Threads     %u\n", data->numberOfThreads);
    PRINT("Total Submissions     %llu\n",(unsigned long long )stats.numOperations);
    PRINT("Total Responses       %llu\n",(unsigned long long )stats.responses);
    PRINT("Total Retries         %u\n",stats.retries);
    if(!signOfLife)
    {
        PRINT("Clock Cycles Start    %llu\n",stats.startCyclesTimestamp);
        PRINT("Clock Cycles End      %llu\n",stats.endCyclesTimestamp);
        PRINT("Total Cycles          %llu\n", numOfCycles);
        PRINT("CPU Frequency(kHz)    %u\n", sampleCodeGetCpuFreq());
        PRINT("Operations per second %8u\n", throughput);
    }
    qaeMemFree((void **)&stats2);
    qaeMemFree((void **)&perfDataDeviceOffsets);
    qaeMemFree((void **)&threadCountPerDevice);
    return CPA_STATUS_SUCCESS;
}
#endif


/*****************************************************************************
* * @description
* Poll the number of crypto operations
* ***************************************************************************/
CpaStatus cyPollNumOperations(perf_data_t *pPerfData,
               CpaInstanceHandle instanceHandle, Cpa64U numOperations)
{
   CpaStatus status = CPA_STATUS_FAIL;

   perf_cycles_t startCycles = 0, totalCycles = 0;
   Cpa32U freq = sampleCodeGetCpuFreq();
   startCycles = sampleCodeTimestamp();

   while(pPerfData->responses != numOperations)
   {
       status = icp_sal_CyPollInstance(instanceHandle,0);
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
           PRINT("Responses expected = %llu, recieved = %llu\n",
                   (unsigned long long)numOperations,
                   (unsigned long long)pPerfData->responses);
           return CPA_STATUS_FAIL;
       }
   }
   return CPA_STATUS_SUCCESS;
}

CpaStatus setCyPollWaitFn(Cpa32U poll_type,
                          Cpa32U sleep_time)
{
   poll_type_g = poll_type;
   sleep_time_g = sleep_time;
   return CPA_STATUS_SUCCESS;
}

void sampleCodePoll(CpaInstanceHandle instanceHandle_in)
{
    CpaStatus status = CPA_STATUS_FAIL;
    while(cy_service_started_g == CPA_TRUE)
    {
        /*poll for 0 means process all packets on the ET ring */
        status = icp_sal_CyPollInstance(instanceHandle_in, 0);
        if(CPA_STATUS_SUCCESS == status || CPA_STATUS_RETRY == status)
        {
           /* do nothing */
        }
        else
        {
           PRINT_ERR("WARNING icp_sal_CyPollInstance returned status %d\n",
                   status);
        }
        switch(poll_type_g)
        {
            case POLL_AND_SLEEP:
                sampleCodeSleepMilliSec(sleep_time_g);
                break;
            default:
                AVOID_SOFTLOCKUP;
        }
    }
    sampleCodeThreadExit();
}


/*start crypto acceleration service if its not already started*/
CpaStatus startCyServices(void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0;
    Cpa32U nProcessorsOnline = 0;
    /*if the service started flag is false*/
    if(cy_service_started_g == CPA_FALSE)
    {
        /*set the started flag to true*/
        cy_service_started_g = CPA_TRUE;
        /*start all crypto instances*/
        status = cpaCyGetNumInstances(&numInstances_g);
        if(CPA_STATUS_SUCCESS != status)
        {
            PRINT_ERR("cpaCyGetNumInstances failed with status: %d\n", status);
            return status;
        }
        if(numInstances_g > 0)
        {
            cyInstances_g = qaeMemAlloc(sizeof(CpaInstanceHandle)*
                                                            numInstances_g);
            if(cyInstances_g == NULL)
            {
                PRINT_ERR("Failed to allocate memory for instances\n");
                return CPA_STATUS_FAIL;
            }
            status = cpaCyGetInstances(numInstances_g, cyInstances_g);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("cpaCyGetInstances failed with status: %d\n", status);
                qaeMemFree((void**)&cyInstances_g);
                return status;
            }
            /*start all instances*/
            for(i=0;i<numInstances_g;i++)
            {
                status  = cpaCySetAddressTranslation(cyInstances_g[i],
                        (CpaVirtualToPhysical)qaeVirtToPhysNUMA);
                if(status != CPA_STATUS_SUCCESS)
                {
                    PRINT_ERR("Error setting memory config for instance %d\n",
                                                                            i);
                    qaeMemFree((void**)&cyInstances_g);
                    return CPA_STATUS_FAIL;
                }
                status = cpaCyStartInstance(cyInstances_g[i]);
                if(status != CPA_STATUS_SUCCESS)
                {
                    PRINT_ERR("Error starting crypto instance %d\n", i);
                    /*attempt to stop any started service, we dont check status
                     * as some instances may not have been started and this
                     * might return fail*/
                    stopCyServices();
                    qaeMemFree((void**)&cyInstances_g);
                    return CPA_STATUS_FAIL;
                }
            }
        }
        else
        {
            PRINT("There are no crypto instances available\n");
            return CPA_STATUS_FAIL;
        }
    }
    /*determine number of cores on system and limit the number of cores to be
     *used to be the smaller of the numberOf Instances or the number of cores*/
    nProcessorsOnline  = sampleCodeGetNumberOfCpus();
    if(nProcessorsOnline > numInstances_g)
    {
        setCoreLimit(numInstances_g);
    }
    /*status should be success if we get to here*/
    return status;

}

/*stop all crypto services*/
CpaStatus stopCyServices(void)
{
    Cpa32U i = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus returnStatus = CPA_STATUS_SUCCESS;
    /* set polling flag to default */
    cy_polling_started_g = CPA_FALSE;
    /*stop only if the services are in a started state*/
    if(cy_service_started_g ==CPA_TRUE)
    {
        /*stop all instances*/
        for(i=0;i<numInstances_g;i++)
        {

            status = cpaCyStopInstance(cyInstances_g[i]);
            if(CPA_STATUS_SUCCESS != status)
            {
                PRINT_ERR("Could not stop instance: %d\n", i);
                /*if we fail to stop a service then something odd has happened
                 * internally and its probably best to reboot*/
                PRINT_ERR("Internal error has occur which probably can only ");
                PRINT(" be fixed by a reboot\n");
                return CPA_STATUS_FAIL;
            }
        }
        /*set the service started flag to false*/
        cy_service_started_g = CPA_FALSE;
    }

    /*free the polling threads*/
    /* Wait for all threads_g to complete */
    for (i=0; i < numPolledInstances_g; i++)
    {
        sampleCodeThreadJoin(&pollingThread_g[i]);

    }
    if (0 < numPolledInstances_g)
    {
        qaeMemFree((void**)&pollingThread_g);
        numPolledInstances_g = 0;
    }
    if ( cyInstances_g != NULL )
    {
         qaeMemFree((void**)&cyInstances_g);
         cyInstances_g = NULL;
    }
    return returnStatus;
}

CpaStatus sampleCreateBuffers(CpaInstanceHandle instanceHandle,
        Cpa32U packetSizeInBytes[],
        CpaFlatBuffer *pFlatBuffArray[],
        CpaBufferList *pBuffListArray[],
        symmetric_test_params_t *setup)
{
    CpaStatus      status          = CPA_STATUS_SUCCESS;
    Cpa32U         bufferMetaSize  = 0;
    Cpa32U         createBufferCount     = 0;
    Cpa32U         createListCount = 0;
    Cpa8U          *pBufferMeta    = NULL;
    Cpa32U          node = 0;
    Cpa32U         numBufferLists = setup->numBuffLists;
    Cpa32U         numBuffers = 0;
    Cpa32U         bufferSizeInBytes = 0;
    CpaFlatBuffer *pTempFlatBuffArray = NULL;


    if(NULL == pFlatBuffArray)
    {
        PRINT_ERR("pFlatBuffArray is NULL\n");
        return CPA_STATUS_FAIL;
    }
    if(NULL == pBuffListArray)
    {
        PRINT_ERR("pBuffListArray is NULL\n");
        return CPA_STATUS_FAIL;
    }
    if(NULL == packetSizeInBytes)
    {
        PRINT_ERR("packetSizeInBytes is NULL\n");
        return CPA_STATUS_FAIL;
    }
    status = sampleCodeCyGetNode(instanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("Failed to get node for instance\n");
        return CPA_STATUS_FAIL;
    }
    /* Caculate number of flatbuffers in one list */
    if ( 0 == setup->flatBufferSizeInBytes )
    {
        numBuffers = NUM_UNCHAINED_BUFFERS;
    }
    else
    {
        numBuffers = (packetSizeInBytes[0] -
                    setup->setupData.hashSetupData.digestResultLenInBytes)/
                    setup->flatBufferSizeInBytes;
    }
    /*
     * calculate memory size which is required for pPrivateMetaData
     * member of CpaBufferList
     */
    status = cpaCyBufferListGetMetaSize( instanceHandle,
           numBuffers, &bufferMetaSize);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCyBufferListGetMetaSize Failed with status: %d\n",
            status);
        return status;
    }

    /*allocate memory for bufferLists, FlatBuffers and Data*/
    for (createListCount = 0;
         createListCount < numBufferLists;
         createListCount++)
    {
        /* Allocate memory for temp flat buffer Array*/
        pTempFlatBuffArray = qaeMemAllocNUMA(sizeof(CpaFlatBuffer)*numBuffers,
               node, BYTE_ALIGNMENT_64);
        if(pTempFlatBuffArray == NULL)
        {
            PRINT_ERR("Could not allocate pFlatBuffArray[%u]\n",createListCount);
            sampleFreeBuffers(pFlatBuffArray, pBuffListArray, setup);
            return CPA_STATUS_FAIL;
        }

        pFlatBuffArray[createListCount] = pTempFlatBuffArray;
        /* Allocate pData memory for each flat buffer */
        for (createBufferCount = 0;
             createBufferCount < numBuffers;
             createBufferCount++)
        {
            /* Decide flat buffers Size: if setup->flatBufferSizeInBytes is 0
             * for the IMIX case,
            * there is only single buffer in List, and bufferSizeInBytes is
            * equal to packetSizeInBytes */
            if ( 0 == setup->flatBufferSizeInBytes )
            {
                /*bufferSize includes space for the digest in the case of hash
                 * or alg chain*/
                bufferSizeInBytes = packetSizeInBytes[createListCount];
            }
            /*while not the last buffer, allocate the normal flat buffer size*/
            else if (createBufferCount != numBuffers-1)
            {
                bufferSizeInBytes = setup->flatBufferSizeInBytes;
            }
            /*else allocate flat buffer + space for digest*/
            else
            {
                bufferSizeInBytes = setup->flatBufferSizeInBytes +
                setup->setupData.hashSetupData.digestResultLenInBytes;
            }
            /* Allocate aligned memory for specified packet size on the node
             * that the thread is running on*/
            pTempFlatBuffArray[createBufferCount].pData = qaeMemAllocNUMA(
                    bufferSizeInBytes,
                    node,
                    BYTE_ALIGNMENT_64);
            if (pTempFlatBuffArray[createBufferCount].pData == NULL)
            {
                PRINT_ERR("Failed to allocate flatBuffer[%u].pData:(%u) memory\n",
                        createBufferCount, bufferSizeInBytes);
                sampleFreeBuffers(pFlatBuffArray, pBuffListArray, setup);
                return CPA_STATUS_FAIL;
            }
            /*initialize dataLenInBytes for each flat buffer*/
            pTempFlatBuffArray[createBufferCount].dataLenInBytes =
                bufferSizeInBytes;

            /*populate the data source with random data*/
            generateRandomData(pTempFlatBuffArray[createBufferCount].pData,
                    bufferSizeInBytes);
        }

        /*allocate memory for bufferLists, FlatBuffers and Data*/
        /* Allocate memory for pPrivateMetaData */
        pBufferMeta=qaeMemAllocNUMA(bufferMetaSize, node, BYTE_ALIGNMENT_64);
        if(pBufferMeta == NULL)
        {
            PRINT_ERR("Failed to allocate pBufferMeta memory\n");
            sampleFreeBuffers(pFlatBuffArray, pBuffListArray, setup);
            return  CPA_STATUS_FAIL;
        }

        /* Allocate memory for buffer list structure */
        pBuffListArray[createListCount] = qaeMemAllocNUMA(sizeof(CpaBufferList),
                node, BYTE_ALIGNMENT_64);
        if(pBuffListArray[createListCount] == NULL)
        {
            PRINT_ERR("Failed to allocate bufferlist memory\n");
            sampleFreeBuffers(pFlatBuffArray, pBuffListArray, setup);
            qaeMemFreeNUMA((void**)&pBufferMeta);
            return CPA_STATUS_FAIL;
        }

        /*
         * Fill in elements of buffer list struct.
         * For this scenario- each buffer list only
         * contains one buffer
         */
        pBuffListArray[createListCount]->numBuffers = numBuffers;
        pBuffListArray[createListCount]->pPrivateMetaData = pBufferMeta;

        /* set up the pBuffers pointer */
        pBuffListArray[createListCount]->pBuffers = pTempFlatBuffArray;
    } /* end of pre allocated buffer for loop */

    /*
     * Return CPA_STATUS_SUCCESS if all buffers have
     * been correctly allocated
     */
    return CPA_STATUS_SUCCESS;
}

void sampleFreeBuffers( CpaFlatBuffer *srcBuffPtrArray[],
                         CpaBufferList *srcBuffListArray[],
                         symmetric_test_params_t *setup)
{
    Cpa32U      freeMemListCount = 0;
    Cpa32U      freeMemCount     = 0;
    CpaFlatBuffer *pTempFlatBuffArray = NULL;

   for (freeMemListCount = 0; freeMemListCount< setup->numBuffLists;
        freeMemListCount++)
   {
          /* Check if bufferListArray is NULL */
       if (NULL == srcBuffListArray)
       {
           break;
       }
       /* Check if bufferList is NULL */
       if( NULL !=srcBuffListArray[freeMemListCount])
       {
            /* Free pPrivateMetaData if it's not NULL */
            if (NULL != srcBuffListArray[freeMemListCount]->pPrivateMetaData)
            {
               qaeMemFreeNUMA(
                        &srcBuffListArray[freeMemListCount]->pPrivateMetaData);
            }
            if (NULL == srcBuffPtrArray)
            {
                break;
            }
            pTempFlatBuffArray = srcBuffPtrArray[freeMemListCount];
           /*
             * Loop through and free all buffers that have been
             * pre-allocated.
             */
            for ( freeMemCount = 0;
                  freeMemCount < srcBuffListArray[freeMemListCount]->numBuffers;
                  freeMemCount++)
            {

                if(NULL != pTempFlatBuffArray)
                {
                    if (NULL != pTempFlatBuffArray[freeMemCount].pData)
                    {
                        qaeMemFreeNUMA((void**)&pTempFlatBuffArray
                                                    [freeMemCount].pData);
                    }
                }
            }
           qaeMemFreeNUMA((void**)&srcBuffPtrArray[freeMemListCount]);
           qaeMemFreeNUMA((void**)&srcBuffListArray[freeMemListCount]);
       }
   }
}
/**
 *****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 *  function to create buffer list for symmetric crypto sample code
 ******************************************************************************/
void dpSampleFreeBuffers( CpaBufferList *srcBuffListArray[],
                         CpaPhysBufferList *srcPhyBuffListArray[],
                         Cpa32U numBuffLists,
                         Cpa32U numBuffers)
{
    Cpa32U      freeMemListCount = 0;
    Cpa32U      freeMemCount     = 0;
    CpaFlatBuffer *pTempFlatBuffArray = NULL;

   for (freeMemListCount = 0; freeMemListCount< numBuffLists;
        freeMemListCount++)
   {
          /* Check if bufferListArray is NULL */
       if (NULL == srcBuffListArray)
       {
           break;
       }
       /* Check if bufferList is NULL */
       if( NULL !=srcBuffListArray[freeMemListCount])
       {
            /* Free pPrivateMetaData if it's not NULL */
            if (NULL != srcBuffListArray[freeMemListCount]->pPrivateMetaData)
            {
               qaeMemFreeNUMA(
                        &srcBuffListArray[freeMemListCount]->pPrivateMetaData);
            }

            pTempFlatBuffArray = srcBuffListArray[freeMemListCount]->pBuffers;
           /*
             * Loop through and free all buffers that have been
             * pre-allocated.
             */
            for ( freeMemCount = 0; freeMemCount < numBuffers;
                  freeMemCount++)
            {

                if(NULL != pTempFlatBuffArray)
                {
                    if (NULL != pTempFlatBuffArray[freeMemCount].pData)
                    {
                        qaeMemFreeNUMA((void**)&pTempFlatBuffArray
                                                    [freeMemCount].pData);
                    }
                }
            }
            if (NULL != pTempFlatBuffArray)
            {
                 qaeMemFreeNUMA((void**)&pTempFlatBuffArray);
            }
            if (NULL != srcPhyBuffListArray[freeMemListCount])
            {
                 qaeMemFreeNUMA((void**)&srcPhyBuffListArray[freeMemListCount]);
            }
            qaeMemFreeNUMA((void**)&srcBuffListArray[freeMemListCount]);
       }
   }
}

/**
 *****************************************************************************
 * @ingroup sampleSymmetricDpPerf
 *
 * @description
 *  function to create buffer list for symmetric crypto sample code
 ******************************************************************************/
CpaStatus dpSampleCreateBuffers(CpaInstanceHandle instanceHandle,
        Cpa32U packetSizeInBytesArray[],
        CpaBufferList *pBuffListArray[],
        CpaPhysBufferList *pPhyBuffListArray[],
        symmetric_test_params_t *setup)
{
    CpaStatus      status          = CPA_STATUS_SUCCESS;
    Cpa32U         bufferMetaSize  = 0;
    Cpa32U         createBufferCount     = 0;
    Cpa32U         createListCount = 0;
    Cpa8U          *pBufferMeta    = NULL;
    Cpa32U          node = 0;
    CpaFlatBuffer *pFlatBuffArray = NULL;
    Cpa32U         bufferSizeInBytes   = 0;
    Cpa32U         numBufferLists      = setup->numBuffLists;
    Cpa32U         numBuffers          = 0;

    if(NULL == pBuffListArray)
    {
        PRINT_ERR("pBuffListArray is NULL\n");
        return CPA_STATUS_FAIL;
    }
    if(NULL == packetSizeInBytesArray)
    {
        PRINT_ERR("packetSizeInBytesArray is NULL\n");
        return CPA_STATUS_FAIL;
    }
    status = sampleCodeCyGetNode(instanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("Failed to get node for instance\n");
        return CPA_STATUS_FAIL;
    }
    /* Caculate number of flatbuffers in one list */
    if ( 0 == setup->flatBufferSizeInBytes )
    {
        numBuffers = NUM_UNCHAINED_BUFFERS;
    }
    else
    {
        /*if packet size is not align with block size of cipher,
        * we need to some padding data into the buffers. */
        if ( 0 == packetSizeInBytesArray[0]%IV_LEN_FOR_16_BYTE_BLOCK_CIPHER)
        {
            numBuffers = (packetSizeInBytesArray[0]
              + setup->flatBufferSizeInBytes -1)/ setup->flatBufferSizeInBytes;
        }
        else
        {
            numBuffers = (packetSizeInBytesArray[0] -
                        setup->setupData.hashSetupData.digestResultLenInBytes)/
                        setup->flatBufferSizeInBytes;
        }
    }

    /*
     * calculate memory size which is required for pPrivateMetaData
     * member of CpaBufferList
     */
    status = cpaCyBufferListGetMetaSize( instanceHandle,
           numBuffers, &bufferMetaSize);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCyBufferListGetMetaSize Failed with status: %d\n",
            status);
        return status;
    }

    for (createListCount = 0;
         createListCount < numBufferLists;
         createListCount++)
    {
        /*allocate memory for bufferLists and Data*/
        /* Allocate memory for temp flat buffer Array*/
        pFlatBuffArray = qaeMemAllocNUMA(sizeof(CpaFlatBuffer)*numBuffers,
               node, BYTE_ALIGNMENT_64);
        if(pFlatBuffArray == NULL)
        {
            PRINT_ERR("Could not allocate pFlatBuffArray[%u]\n",createListCount);
            dpSampleFreeBuffers(pBuffListArray,pPhyBuffListArray,
                           createListCount,numBuffers);
            return CPA_STATUS_FAIL;
        }

        /* Allocate memory for pPrivateMetaData */
        pBufferMeta=qaeMemAllocNUMA(bufferMetaSize, node, BYTE_ALIGNMENT_64);
        if(pBufferMeta == NULL)
        {
            PRINT_ERR("Failed to allocate pBufferMeta memory\n");
            dpSampleFreeBuffers(pBuffListArray,pPhyBuffListArray,
                           createListCount,numBuffers);
            return  CPA_STATUS_FAIL;
        }

        /* Allocate memory for buffer list structure */
        pBuffListArray[createListCount] = qaeMemAllocNUMA(sizeof(CpaBufferList),
                node, BYTE_ALIGNMENT_64);
        if(pBuffListArray[createListCount] == NULL)
        {
            PRINT_ERR("Failed to allocate bufferlist memory\n");
            dpSampleFreeBuffers(pBuffListArray,pPhyBuffListArray,
                           createListCount,numBuffers);
            qaeMemFreeNUMA((void**)&pBufferMeta);
            return CPA_STATUS_FAIL;
        }
        /*
         * Fill in elements of buffer list struct.
         * For this scenario- each buffer list only
         * contains one buffer
         */
        pBuffListArray[createListCount]->numBuffers = numBuffers;
        pBuffListArray[createListCount]->pPrivateMetaData = pBufferMeta;

        /* set up the pBuffers pointer */
        pBuffListArray[createListCount]->pBuffers = pFlatBuffArray;

        /* Allocate memory for physical flat buffer list structure
        * total size of one Buffer list should be equal to size
        * sizeof(CpaPhysBufferList) + sizeof(CpaPhysFlatBuffer)* numBuffers */
        pPhyBuffListArray[createListCount] = qaeMemAllocNUMA(
              sizeof(CpaPhysBufferList) + sizeof(CpaPhysFlatBuffer)* numBuffers,
              node, BYTE_ALIGNMENT_64);
        if(NULL == pPhyBuffListArray[createListCount])
        {
            PRINT_ERR("Failed to allocate bufferlist memory\n");
            dpSampleFreeBuffers(pBuffListArray,pPhyBuffListArray,
                           createListCount,numBuffers);
            return CPA_STATUS_FAIL;
        }
        pPhyBuffListArray[createListCount]->numBuffers =
                             pBuffListArray[createListCount]->numBuffers;
        /* Allocate pData memory for each flat buffer */
        for (createBufferCount = 0;
             createBufferCount < numBuffers;
             createBufferCount++)
        {
            /* Decide flat buffers Size: if setup->flatBufferSizeInBytes is 0,
            * there is only single buffer in List, and bufferSizeInBytes is
            * equal to packetSizeInBytes */
            if ( 0 == setup->flatBufferSizeInBytes )
            {
                bufferSizeInBytes = packetSizeInBytesArray[createListCount];
            }
            else
            {
                bufferSizeInBytes = setup->flatBufferSizeInBytes;
            }

            /* Allocate aligned memory for specified packet size on the node
             * that the thread is running on*/
            pFlatBuffArray[createBufferCount].pData = qaeMemAllocNUMA(
                    bufferSizeInBytes,
                    node,
                    BYTE_ALIGNMENT_64);
            if (pFlatBuffArray[createBufferCount].pData == NULL)
            {
                PRINT_ERR("Failed to allocate flatBuffer[%u].pData:(%u) memory\n",
                        createBufferCount, bufferSizeInBytes);
                dpSampleFreeBuffers(pBuffListArray,pPhyBuffListArray,
                           createListCount,numBuffers);
                return CPA_STATUS_FAIL;
            }
            /*initialize dataLenInBytes for each flat buffer*/
            pFlatBuffArray[createBufferCount].dataLenInBytes =
                bufferSizeInBytes;

            /*populate the data source with random data*/
            generateRandomData(pFlatBuffArray[createBufferCount].pData,
                    bufferSizeInBytes);

            pPhyBuffListArray[createListCount]->
               flatBuffers[createBufferCount].bufferPhysAddr =
               (CpaPhysicalAddr)(SAMPLE_CODE_UINT)
               qaeVirtToPhysNUMA((SAMPLE_CODE_UINT *)
               (SAMPLE_CODE_UINT )pFlatBuffArray[createBufferCount].pData);
            pPhyBuffListArray[createListCount]->
               flatBuffers[createBufferCount].dataLenInBytes
              = pFlatBuffArray[createBufferCount].dataLenInBytes;
        }
    } /* end of pre allocated buffer for loop */

    /*
     * Return CPA_STATUS_SUCCESS if all buffers have
     * been correctly allocated
     */
    return CPA_STATUS_SUCCESS;
}


void setCpaFlatBufferMSB(CpaFlatBuffer* buf)
{
    buf->pData[0] |= MSB_SETTING;
}
EXPORT_SYMBOL(setCpaFlatBufferMSB);

/*Function assumes each number is the same length in bytes*/
CpaBoolean isFbALessThanFbB(CpaFlatBuffer *pFbA,CpaFlatBuffer *pFbB)
{
    Cpa32U i = 0;

    for(i = 0; i< pFbA->dataLenInBytes; i++)
    {
        if(pFbA->pData[i] < pFbB->pData[i])
        {
            return CPA_TRUE;
        }
        else if(pFbB->pData[i] < pFbA->pData[i])
        {
            return CPA_TRUE;
        }
        /*continue if equal*/
    }
    /*buffers contain the same number*/
    return CPA_FALSE;
}

/*Function assumes each number is the same length in bytes*/
CpaFlatBuffer * findSmallestNumber(CpaFlatBuffer *numbers, Cpa32U numNumbers)
{
    CpaFlatBuffer *result = numbers;
    Cpa32U i = 0;

    for(i = 0; i < numNumbers; i ++)
    {
        if(CPA_TRUE == isFbALessThanFbB(result, &numbers[i]))
        {
            result = &numbers[i];
        }

    }
    return result;
}

void makeParam1SmallerThanParam2(
        Cpa8U* param1,
        Cpa8U* param2,
        Cpa32U len,
        CpaBoolean msbSettingRequired)
{
    int i =0;
    int startLen =0;
    if(msbSettingRequired == CPA_TRUE)
    {

        /*set startLen = 1 so that next for loop starts
         * at 1 rather than 0, we handle element 0 here*/
        startLen=1;
        /*Ignoring MSB, if param2 is less then param1, and param2 is not 0,
         * then make param1 to be smaller than param2, and we are done*/
        if( ((param2[0]&(~MSB_SETTING)) <= (param1[0]&(~MSB_SETTING))) &&
                (param2[0]&(~MSB_SETTING))!=0 )
        {
            param1[0] = param2[0]-1;
            return;
        }
        /*Ignoring MSB, if param2 is 0 then param1 needs to be zero with MSB
         * set and we check the next index*/
        else if((param2[0]&(~MSB_SETTING))==0)
        {
            param1[i] = MSB_SETTING;
        }
        /* else Param1 is smaller than param2 so set i = len to skip next for
         * loop*/
        else
        {
            return;
        }
    }
    for(i=startLen;i<len;i++)
    {
        /*if param2 is less then param1, and param2 is not 0, then make param1
         *  to be smaller than param2, and we are done*/
        if( (param2[i] <= param1[i]) && param2[i]!=0 )
        {
            param1[i] = param2[i]-1;
            break;
        }

        /*if param2 is 0 then param1 needs to be zero and we check the next
         * index*/
        else if(param2[i]==0)
        {
            param1[i] = 0;
        }
        /*Param1 is smaller than param2 so we break*/
        else
        {
            break;
        }
    }
}

void conformMillerRabinData(CpaFlatBuffer *pMR,CpaFlatBuffer *pSmallestPC,
                            Cpa32U rounds)
{
    Cpa32S difference = 0;
    Cpa8U i = 0, mrLength = 0;

    /* Get the length of the Miller Rabin Data */
    mrLength = pMR->dataLenInBytes/rounds;

    /* Get the difference in buffer length of the Miller Rabin round and the
     * smallest Prime candidate
     */
    difference = mrLength - pSmallestPC->dataLenInBytes;

    /* As there's a limit on the smallest buffer size used to contain the Miller
     * Rabin Data(MAX(64,required_buffer_size)), we still must satisfy the
     * conditions that Miller Rabin data is >1 and less than Prime -1.
     * If the Miller Rabin buffer length is greater than the smallest Prime
     * Candidate buffer length, we need to zero the most significant bytes of
     * the difference and then ensure that the actual data length is the same.
     */
    if(difference > 0)
    {
        for(i = 0; i < rounds; i++)
        {
            memset(pMR->pData+(i*mrLength),0,difference);
            /* Ensure that Miller Rabin data is less than Prime -1 */
            makeParam1SmallerThanParam2(pMR->pData+(i*mrLength)+difference,
                            pSmallestPC->pData,
                            pSmallestPC->dataLenInBytes,
                            CPA_FALSE);
        }
    }
    else
    {
        for(i = 0; i < rounds; i++)
        {
            makeParam1SmallerThanParam2(pMR->pData +
                    (i * pSmallestPC->dataLenInBytes),
                    pSmallestPC->pData,
                    pSmallestPC->dataLenInBytes,
                    CPA_FALSE);
        }
    }
}

/*assumption is that primeCandidate is an odd number*/
static void incrementPrimeCandidate(CpaFlatBuffer* primeCandidate)
{
    Cpa32S i = 0;


    /*increment by 2 to keep the primeCandidate odd*/
    if(primeCandidate->pData[primeCandidate->dataLenInBytes - 1] != 0xFF)
    {
        primeCandidate->pData[primeCandidate->dataLenInBytes - 1] +=INC_BY_TWO;
        /*roll over did not occur we can exit*/
        return;
    }
    primeCandidate->pData[primeCandidate->dataLenInBytes - 1] = 1;
    /*other wise roll over occurred and we need to increment the high order
     * bytes*/
    for(i = primeCandidate->dataLenInBytes-SECOND_LAST_BYTE; i >= 0; i--)
    {
        /*if the byte is not 0xff then roll over wont occur, and we
         * can increment and exit*/
        if(primeCandidate->pData[i] != 0xFF)
        {
            /*we can increment high order bytes by 1 because it does not
             * effect odd/even*/
            primeCandidate->pData[i] += 1;
            break;
        }
        else
        {
            primeCandidate->pData[i] = 0;
        }
    }
}

void
primeCallback(void *pCallbackTag,
        CpaStatus status,
        void *pOpData,
        CpaBoolean testPassed)
{
    perf_data_t *pPerfData = (perf_data_t *)pCallbackTag;

    /*check perf_data pointer is valid*/
    if (pPerfData == NULL)
    {
        PRINT_ERR("Invalid data in CallbackTag\n");
        return;
    }
    /* response has been received */
    pPerfData->responses++;
    /*if we have received the pre-set numOperations, then get the clock cycle
     * as a timestamp and post the Semaphore to release parent thread*/
    if (testPassed == CPA_TRUE)
    {
        /*record the index of the prime candidate where the primeCandidate
         * passed the primeTest, averagePacketSize in bytes
         * is not the most logical variable to use,  we are re-using the
         * averagePacketSizeInBytes member for a completely different purpose
         * In this case we want to know how many requests it took to find a
         * prime number so that we can be sure we have set the
         * NUM_PRIME_GENERATION_ATTEMPTS so that we find a prime 99.9% of the
         * time*/
        pPerfData->averagePacketSizeInBytes = pPerfData->responses;

    }
    if ( pPerfData->numOperations == pPerfData->responses)
    {
        /*let calling thread know that we are done*/
        sampleCodeSemaphorePost(&pPerfData->comp);
    }
}

void generatePrimeCandidates(CpaFlatBuffer* primeCandidate,
        Cpa32U numCandiates)
{
    Cpa32U i = 0;
    /*generate a random number to test for prime*/
    generateRandomData(primeCandidate[0].pData,
            primeCandidate[0].dataLenInBytes);
    /*make sure MSB is set*/
    setCpaFlatBufferMSB(&primeCandidate[0]);
    /*at the very least prime number candidate should be odd, so we perform
     * OR bitwise operation to make sure value is odd*/
    primeCandidate[0].pData[primeCandidate[0].dataLenInBytes - 1] |= 1;

    /*create set of primeCandidates starting with an odd number and
     * each subsequent candidate is incremented by 2*/
    for(i=1;i<numCandiates;i++)
    {
        memcpy(primeCandidate[i].pData,
                primeCandidate[i-1].pData,
                primeCandidate[i-1].dataLenInBytes);
        incrementPrimeCandidate(&primeCandidate[i]);
    }
}

/*****************************************************************************
 * frees any memory allocated in the generatePrime function
 *****************************************************************************/
#define FREE_GENPRIME_MEM() \
do { \
    Cpa32U j = 0; \
    qaeMemFreeNUMA((void**)&pPrimeTestOpData); \
    qaeMemFreeNUMA((void**)&pMillerRabinData); \
    for(j = 0; j<NUM_PRIME_GENERATION_ATTEMPTS ; j++) \
    { \
        qaeMemFreeNUMA((void**)&primeCandidates[j].pData); \
     } \
     qaeMemFreeNUMA((void**)&primeCandidates); \
} while(0)

CpaStatus generatePrime(CpaFlatBuffer* primeCandidate,
        CpaInstanceHandle cyInstanceHandle,
        asym_test_params_t *setup)
{
    Cpa32U i =0;
    Cpa32U attempt =0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    /** Default is false (meaning the number is not a prime), except if the
     *  test explicitely says it is probably a prime */
    CpaBoolean testPassed = CPA_FALSE;
    perf_data_t primePerfData;

    /** Structure containing the operational data */
    CpaCyPrimeTestOpData *pPrimeTestOpData = NULL;
    CpaFlatBuffer* primeCandidates=NULL;
    /** Random numbers for Miller-Rabin */
    CpaFlatBuffer pMR = {0};
    CpaFlatBuffer *smallestPC = NULL;
    Cpa8U* pMillerRabinData = NULL;
    Cpa8U* pMillerRabinRound[NB_MR_ROUNDS] = {0};
    Cpa32U millerRabinDataLen = 0;
    Cpa32U node = 0;
    millerRabinDataLen = primeCandidate->dataLenInBytes;
    /* The QA API has a a limit on the minimum size( 64 bytes) of the buffer
     * used to contain the Miller Rabin Round data.
     */
    MR_PRIME_LEN(millerRabinDataLen);
    status = sampleCodeCyGetNode(cyInstanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("sampleCodeCyGetNode failed with status %u\n", status);
        return CPA_STATUS_FAIL;
    }
    primeCandidates =
        qaeMemAllocNUMA(sizeof(CpaFlatBuffer) * NUM_PRIME_GENERATION_ATTEMPTS,
                node, BYTE_ALIGNMENT_64);
    if(NULL == primeCandidates)
    {
        PRINT_ERR("primeCandidates is NULL\n");
        return CPA_STATUS_FAIL;
    }
    pPrimeTestOpData =
        qaeMemAllocNUMA(sizeof(CpaCyPrimeTestOpData)
                *NUM_PRIME_GENERATION_ATTEMPTS, node,BYTE_ALIGNMENT_64);
    if(NULL == pPrimeTestOpData)
    {
        PRINT_ERR("pPrimeTestOpData is NULL\n");
        FREE_GENPRIME_MEM();
        return CPA_STATUS_FAIL;
    }
    for(i=0;i<NUM_PRIME_GENERATION_ATTEMPTS;i++)
    {
        bufferDataMemAlloc(cyInstanceHandle, &primeCandidates[i],
                primeCandidate->dataLenInBytes, NULL, 0);
    }

    /*generate random numbers for miller rabin rounds*/
    pMillerRabinData =
        qaeMemAllocNUMA(millerRabinDataLen * NB_MR_ROUNDS, node,
                BYTE_ALIGNMENT_64);
    if(NULL == pMillerRabinData)
    {
        PRINT_ERR("Could not allocate memory for pMillerRabinData\n");
        FREE_GENPRIME_MEM();
        return CPA_STATUS_FAIL;
    }
    generateRandomData(pMillerRabinData,
            millerRabinDataLen * NB_MR_ROUNDS);
    /*set pointer to each miller rabin rounds number*/
    for(i=0;i<NB_MR_ROUNDS;i++)
    {
        Cpa32U byteCheck = millerRabinDataLen -1;
        pMillerRabinRound[i] =
            &pMillerRabinData[i * millerRabinDataLen];
        /*make sure the number is greater than 1 (quick check)*/
        if(1 >= pMillerRabinRound[i][byteCheck])
        {
            generateRandomData(&(pMillerRabinRound[i][byteCheck]), 1);
            /*In case of failure of the random number generator!*/
            if(1 >= pMillerRabinRound[i][byteCheck])
            {
                pMillerRabinRound[i][byteCheck] += INC_BY_TWO;
            }
        }
    }
    pMR.pData = pMillerRabinData;
    pMR.dataLenInBytes = millerRabinDataLen * NB_MR_ROUNDS;
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("Could not allocate pMR->pData\n");
        FREE_GENPRIME_MEM();
        return CPA_STATUS_FAIL;
    }

    for(i=0;i<NUM_PRIME_GENERATION_ATTEMPTS;i++)
    {
        /* Populate the structure containing the data about the number to test:
         * - the number of which we want to test the primality
         * - its length
         * - perform a GCD Primality Test
         * - perform a Fermat Primality Test
         * - number of Miller-Rabin rounds to perform (from 0 to 50)
         * - Miller-Rabin random numbers (one for each test)
         * - perform a Lucas Primality Test */
        pPrimeTestOpData[i].primeCandidate.pData =  primeCandidates[i].pData;
        pPrimeTestOpData[i].primeCandidate.dataLenInBytes =
            primeCandidates[i].dataLenInBytes;
        pPrimeTestOpData[i].performGcdTest =          CPA_TRUE;
        pPrimeTestOpData[i].performFermatTest =       CPA_TRUE;
        pPrimeTestOpData[i].numMillerRabinRounds =    NB_MR_ROUNDS;
        pPrimeTestOpData[i].millerRabinRandomInput.pData =
                                                    pMR.pData;

        pPrimeTestOpData[i].millerRabinRandomInput.dataLenInBytes =
                                                    pMR.dataLenInBytes;
        pPrimeTestOpData[i].performLucasTest =        CPA_TRUE;
    }
    /*Each of miller rabin round number has to be greater than 1 and
     * smaller than the number to test -1 */
    for(attempt=0;attempt<NUM_PRIME_GENERATION_RETRY_ATTEMPTS;attempt++)
    {
        /*we will use the averagePacketSize to store the index of what passes
         * as a prime number in our array of prime candidates*/
        primePerfData.averagePacketSizeInBytes =
            NUM_PRIME_GENERATION_ATTEMPTS+1;
        primePerfData.numOperations=NUM_PRIME_GENERATION_ATTEMPTS;
        primePerfData.responses=0;
        /* Completion used in callback */
        sampleCodeSemaphoreInit(&primePerfData.comp, 0);
        generatePrimeCandidates(primeCandidates,
                    NUM_PRIME_GENERATION_ATTEMPTS);
        /*no need to regenerate randomness - MR rounds just have to be
          greater than 1 and less than (Prime -1)*/
        /*Find smallest prime candidate*/
        smallestPC = findSmallestNumber(primeCandidates,
                            NUM_PRIME_GENERATION_ATTEMPTS);
        /*make all numbers less than the smallest candidate -1 */
        smallestPC->pData[smallestPC->dataLenInBytes -1] &=~1;

        conformMillerRabinData(&pMR,smallestPC,NB_MR_ROUNDS);

        smallestPC->pData[smallestPC->dataLenInBytes -1] |=1;
        for(i=0;i<NUM_PRIME_GENERATION_ATTEMPTS;i++)
        {
            do{
                status = cpaCyPrimeTest (cyInstanceHandle,
                    primeCallback,  /* CB function */
                    &primePerfData, /* callback tag */
                    &pPrimeTestOpData[i],  /* operation data */
                    &testPassed);   /* return value:
                                      true if the number is probably
                                      a prime, false if it is not a prime */
                AVOID_SOFTLOCKUP;
                if(CPA_STATUS_RETRY == status)
                {
                    primePerfData.retries++;
                    if(RETRY_LIMIT == primePerfData.retries)
                    {
                        primePerfData.retries=0;
                        AVOID_SOFTLOCKUP;
                    }
                }
            } while (CPA_STATUS_RETRY == status ||
                    CPA_STATUS_RESOURCE == status);
            /*we check for resource error above because if the driver is
             * testing a lot of large primes, it can run out of memory pools.
             * In this circumstance it can be reported as a
             * CPA_STATUS_RESOURCE*/
            if (CPA_STATUS_SUCCESS != status)
            {
                PRINT("Error Generating Prime\n");
                status = CPA_STATUS_FAIL;
                break;
            }
        }
        AVOID_SOFTLOCKUP;
        /*the callback posts the semaphore when all requests have
         * been processed. If there was a fail, then the callback will never
         * receive the expected number of response and will never post the
         * semaphore*/
        if(CPA_STATUS_SUCCESS == status)
        {
            if(sampleCodeSemaphoreWait(&primePerfData.comp,
                        SAMPLE_CODE_WAIT_FOREVER) != CPA_STATUS_SUCCESS)
            {
                PRINT_ERR("timeout or interruption in cpaCyPrimeTest\n");
                status = CPA_STATUS_FAIL;
            }
        }
        sampleCodeSemaphoreDestroy(&primePerfData.comp);
        /*here we re-use averagePacketSizeInBytes for another purpose. In this
         * case we use it to record the index in our primeNumber candidates
         * that a prime was found
         * if the index has changed then we have found a prime number*/
        if(primePerfData.averagePacketSizeInBytes !=
            NUM_PRIME_GENERATION_ATTEMPTS+1)
        {
            memcpy(primeCandidate->pData,
                    primeCandidates[
                        primePerfData.averagePacketSizeInBytes-1].pData,
                        primeCandidate->dataLenInBytes);
            break;
        }
        else if(i == NUM_PRIME_GENERATION_ATTEMPTS-1 &&
                attempt ==NUM_PRIME_GENERATION_RETRY_ATTEMPTS-1 &&
                primePerfData.averagePacketSizeInBytes ==
                            NUM_PRIME_GENERATION_ATTEMPTS+1)
        {
            PRINT_ERR("\nPRIME NUMBER NOT FOUND\n");
            status = CPA_STATUS_FAIL;
        }
    }

    /** Free all allocated structures before exit*/
    FREE_GENPRIME_MEM();
    return status;
}
#undef FREE_GENPRIME_MEM
EXPORT_SYMBOL(generatePrime);

void freeArrayFlatBufferNUMA(CpaFlatBuffer* buf, Cpa32U numBuffs)
{
    Cpa32U i = 0;

    /* this function maybe called before some memory is allocated,
     * in which case we just return */
    if(NULL == buf)
    {
        return;
    }
    for(i=0; i<numBuffs; i++)
    {
        FREE_NUMA_MEM(buf[i].pData);
    }
    //PRINT("%d, S %p\n", __LINE__, buf);
    qaeMemFree((void**)&buf);
}

/*set hash len based on hash algorithm*/
Cpa32U setHashDigestLen(CpaCySymHashAlgorithm hashAlgorithm)
{
    switch(hashAlgorithm)
    {
    case CPA_CY_SYM_HASH_MD5:
        return MD5_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_SHA1:
        return SHA1_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_SHA224:
        return SHA224_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_SHA256:
        return SHA256_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_SHA384:
        return SHA384_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_SHA512:
        return SHA512_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_AES_XCBC:
        return AES_XCBC_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_AES_CCM:
        return AES_CCM_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_AES_GCM:
        return AES_GCM_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_KASUMI_F9:
        return KASUMI_F9_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_SNOW3G_UIA2:
        return SNOW3G_UIA2_DIGEST_LENGTH_IN_BYTES;
    case CPA_CY_SYM_HASH_AES_CMAC:
        return AES_CMAC_DIGEST_LENGTH_IN_BYTES;
#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#elif CPA_CY_API_VERSION_NUM_MINOR >= 8
    case CPA_CY_SYM_HASH_AES_CBC_MAC:
        return AES_CBC_MAC_DIGEST_LENGTH_IN_BYTES;
#endif
    default:
        PRINT_ERR("Unknown hash algorithm\n");
        /*we return 0, when the the API is called it should fail with
         * invalid param*/
        return 0;
    }
}

/*****************************************************************************
 * frees any memory allocated in the calcDigest function
 *****************************************************************************/
#define FREE_CALC_DIGEST_MEM() \
do { \
    qaeMemFreeNUMA((void**)&pSrcBuffer); \
    qaeMemFreeNUMA((void**)&pBufferList); \
    qaeMemFreeNUMA((void**)&pBufferMeta); \
    qaeMemFreeNUMA((void**)&pOpData); \
    qaeMemFreeNUMA((void**)&pSessionCtx); \
} while(0)

/*calculate digest of msg using hashAlg and place it in digest*/
CpaStatus calcDigest(CpaInstanceHandle instanceHandle,
        CpaFlatBuffer* msg,
        CpaFlatBuffer* digest,
        CpaCySymHashAlgorithm hashAlg)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaStatus ret = CPA_STATUS_SUCCESS;
    Cpa32U sessionCtxSize = 0;
    CpaCySymSessionCtx pSessionCtx = NULL;
    CpaCySymSessionSetupData sessionSetupData = {0};


    Cpa8U  *pBufferMeta = NULL;
    Cpa32U bufferMetaSize = 0;
    CpaBufferList *pBufferList = NULL;
    CpaFlatBuffer *pFlatBuffer = NULL;
    CpaCySymOpData *pOpData = NULL;
    Cpa32U bufferSize = 0;
    Cpa32U digestLenInBytes = 0;
    Cpa32U numBuffers = 1;  /* only using 1 buffer in this case */
    /* allocate memory for bufferlist and array of flat buffers in a contiguous
     * area and carve it up to reduce number of memory allocations required. */
    Cpa32U bufferListMemSize = sizeof(CpaBufferList) +
        (numBuffers * sizeof(CpaFlatBuffer));
    Cpa8U  *pSrcBuffer = NULL;
    Cpa32U node = 0;
    CpaCySymCbFunc symCb = NULL;
    perf_data_t *pPerfData = NULL;


    status = sampleCodeCyGetNode(instanceHandle, &node);
    if(CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("sampleCodeCyGetNode failed with status %u\n", status);
        return CPA_STATUS_FAIL;
    }
    /* populate symmetric session data structure
     * for a plain hash operation */
    sessionSetupData.sessionPriority = CPA_CY_PRIORITY_NORMAL;
    sessionSetupData.symOperation = CPA_CY_SYM_OP_HASH;
    sessionSetupData.hashSetupData.hashMode = CPA_CY_SYM_HASH_MODE_PLAIN;
    sessionSetupData.hashSetupData.hashAlgorithm = hashAlg;
    sessionSetupData.hashSetupData.digestResultLenInBytes =
        setHashDigestLen(hashAlg);
    sessionSetupData.verifyDigest = CPA_FALSE;

    digestLenInBytes = sessionSetupData.hashSetupData.digestResultLenInBytes;
    bufferSize = msg->dataLenInBytes + digestLenInBytes;
    /* Determine size of session context to allocate */
    status = cpaCySymSessionCtxGetSize(instanceHandle, &sessionSetupData,
            &sessionCtxSize);
    /* Allocate session context */
    pSessionCtx = qaeMemAllocNUMA(sessionCtxSize, node, BYTE_ALIGNMENT_64);
    if(NULL == pSessionCtx)
    {
        PRINT_ERR("Could not allocate memory for pSessionCtx\n");
        return CPA_STATUS_FAIL;
    }

    status = cpaCySymInitSession(instanceHandle,
            symCb, &sessionSetupData, pSessionCtx);
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCySymInitSession failed with status %u\n", status);
        FREE_CALC_DIGEST_MEM();
        return status;
    }

    status = cpaCyBufferListGetMetaSize( instanceHandle,
                numBuffers, &bufferMetaSize);
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCyBufferListGetMetaSize failed with status %u\n",
                                                                 status);
        FREE_CALC_DIGEST_MEM();
        return status;
    }
    pBufferMeta = qaeMemAllocNUMA(bufferMetaSize,
            node, BYTE_ALIGNMENT_64);
    if(NULL == pBufferMeta)
    {
        PRINT_ERR("could not allocate pBufferMeta\n");
        FREE_CALC_DIGEST_MEM();
        return CPA_STATUS_FAIL;
    }
    pBufferList = qaeMemAllocNUMA(bufferListMemSize,
            node, BYTE_ALIGNMENT_64);
    if(NULL == pBufferList)
    {
        PRINT_ERR("could not allocate pBufferMeta\n");
        FREE_CALC_DIGEST_MEM();
        return CPA_STATUS_FAIL;
    }
    pSrcBuffer = qaeMemAllocNUMA(bufferSize, node, BYTE_ALIGNMENT_64);
    if(NULL == pSrcBuffer)
    {
        PRINT_ERR("could not allocate pSrcBuffer\n");
        FREE_CALC_DIGEST_MEM();
        return CPA_STATUS_FAIL;

    }
    memcpy(pSrcBuffer, msg->pData, msg->dataLenInBytes);
    /*memory was allocated for bufferList and flatbuffer together so
     * the flatBuffer offset is just after the CpaBufferList*/
    pFlatBuffer = (CpaFlatBuffer*) (pBufferList + 1);
    pBufferList->pBuffers = pFlatBuffer;
    pBufferList->numBuffers = 1;
    pBufferList->pPrivateMetaData = pBufferMeta;
    pFlatBuffer->dataLenInBytes = bufferSize;
    pFlatBuffer->pData = pSrcBuffer;
    pOpData=qaeMemAllocNUMA(sizeof(CpaCySymOpData),
            node, BYTE_ALIGNMENT_64);
    if(NULL == pOpData)
    {
        PRINT_ERR("could not allocate pSrcBuffer\n");
        FREE_CALC_DIGEST_MEM();
        return CPA_STATUS_FAIL;

    }
    pOpData->sessionCtx=pSessionCtx;
    pOpData->packetType=CPA_CY_SYM_PACKET_TYPE_FULL;
    pOpData->hashStartSrcOffsetInBytes=0;
    pOpData->messageLenToHashInBytes=msg->dataLenInBytes;

    /* Place digest after data in the source buffer */
    pOpData->pDigestResult=pSrcBuffer + msg->dataLenInBytes;

    /** Perform symmetric operation */
    status = cpaCySymPerformOp(instanceHandle,
                pPerfData,               /* perform synchronous operation*/
                pOpData,            /* operational data struct */
                pBufferList,        /* source buffer list */
                pBufferList,        /* in-place operation*/
                NULL);

    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCySymPerformOp failed. (status = %u)\n", status);
        ret = CPA_STATUS_FAIL;
    }

    memcpy(digest->pData, pOpData->pDigestResult, digestLenInBytes);
    digest->dataLenInBytes = digestLenInBytes;
     /* Remove the session - session init has already succeeded */
    status = cpaCySymRemoveSession(instanceHandle, pSessionCtx);
    if (CPA_STATUS_SUCCESS != status)
    {
        PRINT_ERR("cpaCySymRemoveSession failed. (status = %u)\n", status);
        ret = CPA_STATUS_FAIL;
    }
    FREE_CALC_DIGEST_MEM();

    return ret;
}
#undef FREE_CALC_DIGEST_MEM

/*allocate pData of buf*/
CpaStatus bufferDataMemAlloc(CpaInstanceHandle instanceHandle,
       CpaFlatBuffer* buf, Cpa32U size, Cpa8U* copyData, Cpa32U sizeOfCopyData)
{
    Cpa32U node = 0;
    CpaStatus status = CPA_STATUS_FAIL;
    if(NULL == buf)
    {
        PRINT_ERR("buf is null\n");
        return CPA_STATUS_FAIL;
    }
    if((NULL != copyData) && (sizeOfCopyData > size))
    {
        PRINT_ERR("copy size is > allocated size\n");
        return CPA_STATUS_FAIL;
    }
    /* get the node the thread is running on and allocate memory to the same
     * node
     */
    status = sampleCodeCyGetNode(instanceHandle, &node);
    if(CPA_STATUS_SUCCESS == status)
    {
        buf->pData = (Cpa8U *)qaeMemAllocNUMA(size, node, BYTE_ALIGNMENT_64);
        buf->dataLenInBytes = size;
        if(NULL == buf->pData)
        {
            PRINT_ERR("pData allocation error\n");
            return CPA_STATUS_FAIL;
        }
        if(NULL != copyData)
        {
            memcpy(buf->pData,copyData,size);
        }
    }
    else
    {
        PRINT_ERR("Failed to get node\n");
    }
    return status;
}


CpaStatus sampleCodeCyGetNode(CpaInstanceHandle instanceHandle,
        Cpa32U* node)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaInstanceInfo2 instanceInfo2;

    status = cpaCyInstanceGetInfo2(instanceHandle, &instanceInfo2);
    if(status == CPA_STATUS_SUCCESS)
    {
        *node = instanceInfo2.nodeAffinity;
    }
    if(allocateMemOnOppositeNode)
    {
        *node =~ *node;
        *node = *node & 0x01;
    }
    return status;
}


void processCallback(void* pCallbackTag)
{
    perf_data_t *pPerfData = (perf_data_t *)pCallbackTag;
    /*check perf_data pointer is valid*/
    if (pPerfData == NULL)
    {
        PRINT_ERR("Invalid data in CallbackTag\n");
        return;
    }
    /* response has been received */
    pPerfData->responses++;
#ifdef LATENCY_CODE
    if (latency_enable) {
        /* Did we setup the array pointer? */
        assert(NULL != pPerfData->response_times);

        /* Have we sampled too many buffer operations? */
        assert(pPerfData->latencyCount < MAX_LATENCY_COUNT);

        /* Is this the buffer we calculate latency on?
         * And have we calculated too many for array? */
        if(pPerfData->responses == pPerfData->nextCount)
        {
            int i = pPerfData->latencyCount;

            /* Now get the end timestamp - before any print outs */
            pPerfData->response_times[i] = sampleCodeTimestamp();

            pPerfData->nextCount += pPerfData->countIncrement;

            if (latency_debug) PRINT("%s: responses=%u, latencyCount=%d, end[i]:%llu, start[i]:%llu, nextCount=%u\n",
                    __FUNCTION__,
                    (unsigned int)pPerfData->responses,
                    i,
                    pPerfData->response_times[i],
                    pPerfData->start_times[i],
                    pPerfData->nextCount );

            pPerfData->latencyCount++;
        }
    }
#endif // LATENCY_CODE
    /*if we have received the pre-set numOperations, then get the clock cycle
     * as a timestamp and post the Semaphore to release parent thread*/
    if (pPerfData->numOperations == pPerfData->responses)
    {
        pPerfData->endCyclesTimestamp=sampleCodeTimestamp();
        if(CPA_STATUS_SUCCESS != sampleCodeSemaphorePost(&pPerfData->comp))
        {
            PRINT_ERR("sampleCodeSemaphorePost Error\n");
            pPerfData->threadReturnStatus = CPA_STATUS_FAIL;

        }
    }
}


CpaStatus allocArrayOfPointers(CpaInstanceHandle instanceHandle,
        void** buf,
        Cpa32U numBuffs)
{
    /*Cpa32U m =0;*/
    Cpa32U node = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    status = sampleCodeCyGetNode(instanceHandle, &node);
    if(CPA_STATUS_SUCCESS!= status)
    {
        PRINT_ERR("Could not get Node\n");
        return CPA_STATUS_FAIL;
    }
    *buf = qaeMemAllocNUMA((sizeof(void*) * numBuffs),
                                        node, BYTE_ALIGNMENT_64);
    if(NULL != *buf)
    {
        memset(*buf,0, (sizeof(void*) * numBuffs));
    }
    else
    {
        PRINT_ERR("Error getting allocating array of pointers \n");
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

CpaStatus allocArrayOfVirtPointers(CpaInstanceHandle instanceHandle,
        void** buf,
        Cpa32U numBuffs)
{
    *buf = qaeMemAlloc((sizeof(void*) * numBuffs));
    if(NULL != *buf)
    {
        memset(*buf,0, (sizeof(void*) * numBuffs));
    }
    else
    {
        PRINT_ERR("Error getting allocating array of pointers \n");
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}


CpaStatus waitForResponses(
        perf_data_t *perfData,
        sync_mode_t syncMode,
        Cpa32U numBuffers,
        Cpa32U numLoops)
{
    Cpa64S responsesReceived = RESPONSE_NOT_CHECKED;
    CpaStatus status = CPA_STATUS_SUCCESS;

    if(SYNC == syncMode)
    {
        perfData->endCyclesTimestamp=sampleCodeTimestamp();
        sampleCodeSemaphorePost(&perfData->comp);
        perfData->responses = numBuffers * numLoops;
    }
    /*wait for the callback to receive all responses and free the
     * semaphore, or if in sync mode, the semaphore should already be free*/

    while(sampleCodeSemaphoreWait(&perfData->comp,
            SAMPLE_CODE_WAIT_THIRTY_SEC
            ) != CPA_STATUS_SUCCESS)
    {
        if(RESPONSE_NOT_CHECKED != responsesReceived &&
                responsesReceived != perfData->numOperations &&
                responsesReceived == perfData->responses)
        {
            PRINT_ERR("System is not responding\n");
            PRINT("Responses expected/received: %llu/%llu\n",
                    (unsigned long long )perfData->numOperations,
                    (unsigned long long )perfData->responses);
            status = CPA_STATUS_FAIL;
            break;
        }
        else
        {
            responsesReceived = perfData->responses;
        }
    }

    return status;
}
CpaStatus cyCreatePollingThreadsIfPollingIsEnabled(void)
{
    CpaInstanceInfo2 *instanceInfo2 = NULL;
    Cpa16U i = 0, j = 0, numCreatedPollingThreads = 0;
    Cpa32U coreAffinity = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    performance_func_t *pollFnArr = NULL;
#if defined(USER_SPACE) && !defined(SC_EPOLL_DISABLED)
    int fd = -1;
#endif
    if( CPA_FALSE == cy_polling_started_g )
    {
        instanceInfo2 = qaeMemAlloc(numInstances_g * sizeof(CpaInstanceInfo2));
        if(NULL == instanceInfo2)
        {
            PRINT_ERR("Failed to allocate memory for pInstanceInfo2\n");
            return CPA_STATUS_FAIL;
        }
        pollFnArr = qaeMemAlloc(numInstances_g * sizeof(performance_func_t));
        if(NULL == pollFnArr)
        {
            PRINT_ERR("Failed to allocate memory for polling functions\n");

            qaeMemFree((void**)&instanceInfo2);
            return CPA_STATUS_FAIL;
        }
        for(i = 0; i < numInstances_g; i++)
        {
            status = cpaCyInstanceGetInfo2(cyInstances_g[i], &instanceInfo2[i]);
            if(CPA_STATUS_SUCCESS != status)
            {
                qaeMemFree((void**)&instanceInfo2);
                qaeMemFree((void**)&pollFnArr);
                return CPA_STATUS_FAIL;
            }
            pollFnArr[i] = NULL;
            if(CPA_TRUE == instanceInfo2[i].isPolled)
            {
                numPolledInstances_g++;
#if defined(USER_SPACE) && !defined(SC_EPOLL_DISABLED)
                status = icp_sal_CyGetFileDescriptor(cyInstances_g[i], &fd);
                if (CPA_STATUS_SUCCESS == status)
                {
                    pollFnArr[i] = sampleCodeCyEventPoll;
                    icp_sal_CyPutFileDescriptor(cyInstances_g[i], fd);
                    continue;

                }
                else if(CPA_STATUS_FAIL == status)
                {
                    PRINT_ERR("Error getting file descriptor for Event based "
                            "instance #%d\n", i);
                    qaeMemFree((void**)&instanceInfo2);
                    qaeMemFree((void**)&pollFnArr);
                    return CPA_STATUS_FAIL;

                }
                /* else feature is unsupported and sampleCodePoll() is to be
                 * used.
                 */
#endif
#if !defined(USER_SPACE)
                if(strcmp(instanceInfo2[i].partName,"dh89xxcc")==0)
                {
                    setCyPollWaitFn(1,0);
                }
#endif
                pollFnArr[i] = sampleCodePoll;
            }

        }
        if (0 == numPolledInstances_g)
        {
            qaeMemFree((void**)&instanceInfo2);
            qaeMemFree((void**)&pollFnArr);
            return CPA_STATUS_SUCCESS;
        }
        pollingThread_g =
            qaeMemAlloc(numPolledInstances_g * sizeof(sample_code_thread_t));
        if(NULL == pollingThread_g)
        {
            PRINT_ERR("Failed to allocate memory for polling threads\n");
            qaeMemFree((void**)&instanceInfo2);
            qaeMemFree((void**)&pollFnArr);
            return CPA_STATUS_FAIL;
        }
        for(i = 0; i < numInstances_g; i++)
        {
            if(NULL != pollFnArr[i])
            {
                status = sampleCodeThreadCreate(
                        &pollingThread_g[numCreatedPollingThreads],
                        NULL,
                        pollFnArr[i],
                        cyInstances_g[i]);
                if(status != CPA_STATUS_SUCCESS)
                {
                    PRINT_ERR("Error starting polling thread %d\n", status);
                    /*attempt to stop any started service, we dont check status
                     * as some instances may not have been started and this
                     * might return fail
                     * */
                    qaeMemFree((void**)&instanceInfo2);
                    qaeMemFree((void**)&pollFnArr);
                    return CPA_STATUS_FAIL;
                }
                /*loop of the instanceInfo coreAffinity bitmask to find the core
                 *  affinity*/
                for(j = 0; j< CPA_MAX_CORES; j++)
                {
                    if(CPA_BITMAP_BIT_TEST(instanceInfo2[i].coreAffinity,j))
                    {
                        coreAffinity = j;
                        break;
                    }
                }
                sampleCodeThreadBind(&pollingThread_g[numCreatedPollingThreads],
                        coreAffinity);


                sampleCodeThreadStart(
                        &pollingThread_g[numCreatedPollingThreads]);

                numCreatedPollingThreads++;
            }
        }
        qaeMemFree((void**)&instanceInfo2);
        qaeMemFree((void**)&pollFnArr);

        cy_polling_started_g = CPA_TRUE;
    }
    return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(cyCreatePollingThreadsIfPollingIsEnabled);


CpaBoolean cyCheckAllInstancesArePolled(void)
{
    CpaInstanceInfo2 *instanceInfo2 = NULL;
    Cpa16U i = 0;

    instanceInfo2 = qaeMemAlloc(numInstances_g * sizeof(CpaInstanceInfo2));
    if(NULL == instanceInfo2)
    {
        PRINT_ERR("Failed to allocate memory for pInstanceInfo2\n");
        return CPA_FALSE;
    }
    for(i = 0; i < numInstances_g; i++)
    {
        if( CPA_STATUS_SUCCESS != cpaCyInstanceGetInfo2(cyInstances_g[i],
                &instanceInfo2[i]))
        {
            PRINT_ERR("Call to cpaCyInstanceGetInfo2 failed\n");
            qaeMemFree((void**)&instanceInfo2);
            return CPA_FALSE;

        }

        if(CPA_FALSE == instanceInfo2[i].isPolled)
        {
            qaeMemFree((void**)&instanceInfo2);
            return CPA_FALSE;
        }
    }
    qaeMemFree((void**)&instanceInfo2);
    return CPA_TRUE;
}

CpaStatus cyDpPollRemainingOperations(perf_data_t *pPerfData,
                CpaInstanceHandle instanceHandle)
{
    CpaStatus status = CPA_STATUS_FAIL;

    perf_cycles_t startCycles = 0, totalCycles = 0;
    Cpa32U freq = sampleCodeGetCpuFreq();
    startCycles = sampleCodeTimestamp();

    while(pPerfData->responses != pPerfData->numOperations)
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
            return CPA_STATUS_FAIL;
        }
    }
    return CPA_STATUS_SUCCESS;
}

#ifndef NEWDISPLAY
/**
 *****************************************************************************
 * @ingroup sampleSymmetricPerf
 *
 * @description
 *  function to print out cipher performance header
 ******************************************************************************/
void printCipherAlg(CpaCySymCipherSetupData cipherSetupData)
{
    switch(cipherSetupData.cipherAlgorithm)
    {
    case CPA_CY_SYM_CIPHER_NULL:
        PRINT("NULL");
        break;
    case CPA_CY_SYM_CIPHER_ARC4:
        PRINT("ARC4");
        break;
    case CPA_CY_SYM_CIPHER_AES_XTS:
        if(cipherSetupData.cipherKeyLenInBytes ==
            KEY_SIZE_256_IN_BYTES)
        {
            PRINT("AES128-");
        }
        else if(cipherSetupData.cipherKeyLenInBytes ==
            KEY_SIZE_512_IN_BYTES)
        {
            PRINT("AES256-");
        }
        PRINT("XTS");
        break;
    case CPA_CY_SYM_CIPHER_AES_ECB:
    case CPA_CY_SYM_CIPHER_AES_CBC:
    case CPA_CY_SYM_CIPHER_AES_CTR:
    case CPA_CY_SYM_CIPHER_AES_CCM:
    case CPA_CY_SYM_CIPHER_AES_GCM:
        if(cipherSetupData.cipherKeyLenInBytes ==
            KEY_SIZE_128_IN_BYTES)
        {
            PRINT("AES128-");
        }
        else if(cipherSetupData.cipherKeyLenInBytes ==
            KEY_SIZE_192_IN_BYTES)
        {
            PRINT("AES192-");
        }
        else if(cipherSetupData.cipherKeyLenInBytes ==
            KEY_SIZE_256_IN_BYTES)
        {
            PRINT("AES256-");
        }
        else
        {
            PRINT("AES with unknown key size\n");
        }
        if(cipherSetupData.cipherAlgorithm ==
            CPA_CY_SYM_CIPHER_AES_ECB)
        {
            PRINT("ECB");
        }
        if(cipherSetupData.cipherAlgorithm ==
            CPA_CY_SYM_CIPHER_AES_CBC)
        {
            PRINT("CBC");
        }
        if(cipherSetupData.cipherAlgorithm ==
            CPA_CY_SYM_CIPHER_AES_CTR)
        {
            PRINT("CTR");
        }
        if(cipherSetupData.cipherAlgorithm ==
            CPA_CY_SYM_CIPHER_AES_CCM)
        {
            PRINT("CCM");
        }
        if(cipherSetupData.cipherAlgorithm ==
            CPA_CY_SYM_CIPHER_AES_GCM)
        {
            PRINT("GCM");
        }
        break;
    case CPA_CY_SYM_CIPHER_DES_ECB:
        PRINT("DES-ECB");
        break;
    case CPA_CY_SYM_CIPHER_DES_CBC:
        PRINT("DES-CBC");
        break;
    case CPA_CY_SYM_CIPHER_3DES_ECB:
        PRINT("3DES-ECB");
        break;
    case CPA_CY_SYM_CIPHER_3DES_CBC:
        PRINT("3DES-CBC");
        break;
    case CPA_CY_SYM_CIPHER_3DES_CTR:
        PRINT("3DES-CTR");
        break;
    case CPA_CY_SYM_CIPHER_KASUMI_F8:
        PRINT("KASUMI_F8");
        break;
    case CPA_CY_SYM_CIPHER_SNOW3G_UEA2:
        PRINT("SNOW3G_UEA2");
        break;
    case CPA_CY_SYM_CIPHER_AES_F8:
        if(cipherSetupData.cipherKeyLenInBytes ==
            KEY_SIZE_256_IN_BYTES)
        {
            PRINT("AES128-");
        }
        else if(cipherSetupData.cipherKeyLenInBytes ==
            KEY_SIZE_384_IN_BYTES)
        {
            PRINT("AES192-");
        }
        else if(cipherSetupData.cipherKeyLenInBytes ==
            KEY_SIZE_512_IN_BYTES)
        {
            PRINT("AES256-");
        }
        else
        {
            PRINT("AES with unknown key size\n");
        }
        PRINT("F8");
        break;
#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#endif

    default:
        PRINT("UNKNOWN_CIPHER %d\n",cipherSetupData.cipherAlgorithm);
        break;
    }
}
#endif

#ifndef NEWDISPLAY
/**
 *****************************************************************************
 * @ingroup sampleSymmetricPerf
 *
 * @description
 * function to print out hash performance header
 ******************************************************************************/
void printHashAlg(CpaCySymHashSetupData hashSetupData)
{
    if(hashSetupData.hashMode == CPA_CY_SYM_HASH_MODE_AUTH &&
            (hashSetupData.hashAlgorithm != CPA_CY_SYM_HASH_AES_XCBC ||
                    hashSetupData.hashAlgorithm != CPA_CY_SYM_HASH_AES_CCM ||
                    hashSetupData.hashAlgorithm != CPA_CY_SYM_HASH_AES_GCM ||
                    hashSetupData.hashAlgorithm != CPA_CY_SYM_HASH_AES_GMAC ||
                    hashSetupData.hashAlgorithm != CPA_CY_SYM_HASH_AES_CMAC))
    {
        PRINT("HMAC-");
    }
    switch(hashSetupData.hashAlgorithm)
    {
    case CPA_CY_SYM_HASH_MD5:
        PRINT("MD5");
        break;
    case CPA_CY_SYM_HASH_SHA1:
        PRINT("SHA1");
        break;
    case CPA_CY_SYM_HASH_SHA224:
        PRINT("SHA2-224");
        break;
    case CPA_CY_SYM_HASH_SHA256:
        PRINT("SHA2-256");
        break;
    case CPA_CY_SYM_HASH_SHA384:
        PRINT("SHA2-384");
        break;
    case CPA_CY_SYM_HASH_SHA512:
        PRINT("SHA2-512");
        break;
    case CPA_CY_SYM_HASH_AES_XCBC:
        PRINT("AES-XCBC");
        break;
    case CPA_CY_SYM_HASH_AES_CCM:
        PRINT("AES-CCM");
        break;
    case CPA_CY_SYM_HASH_AES_GCM:
        PRINT("AES-GCM");
        break;
    case CPA_CY_SYM_HASH_KASUMI_F9:
        PRINT("KASUMI-F9");
        break;
    case CPA_CY_SYM_HASH_SNOW3G_UIA2:
        PRINT("SNOW3G-UIA2");
        break;
    case CPA_CY_SYM_HASH_AES_CMAC:
        PRINT("AES-CMAC");
        break;
    case CPA_CY_SYM_HASH_AES_GMAC:
        PRINT("AES-GMAC");
        break;
#if CPA_CY_API_VERSION_NUM_MAJOR >= 2
#elif CPA_CY_API_VERSION_NUM_MINOR >= 8
    case CPA_CY_SYM_HASH_AES_CBC_MAC:
        PRINT("AES-CBC-MAC");
        break;
#endif
    default:
        PRINT("UNKNOWN_HASH\n");
        break;
    }
}
#endif
#ifndef NEWDISPLAY
/**
 *****************************************************************************
 * @ingroup sampleSymmetricPerf
 *
 * @description
 * print out performance test type
 ******************************************************************************/
void printSymTestType(symmetric_test_params_t *setup)
{
    CpaStatus status;
    if(setup->setupData.symOperation == CPA_CY_SYM_OP_CIPHER)
    {
        PRINT("Cipher ");
        if(cipherDirection_g == CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT)
        {
            PRINT("Encrypt ");
        }
        else
        {
            PRINT("Decrypt ");
        }
        printCipherAlg(setup->setupData.cipherSetupData);
    }
    else if(setup->setupData.symOperation == CPA_CY_SYM_OP_HASH)
    {
        PRINT("HASH ");
        printHashAlg(setup->setupData.hashSetupData);
    }
    else if(setup->setupData.symOperation == CPA_CY_SYM_OP_ALGORITHM_CHAINING)
    {
        PRINT("Algorithm Chaining - ");
        printCipherAlg(setup->setupData.cipherSetupData);
        PRINT(" ");
        printHashAlg(setup->setupData.hashSetupData);
    }
    PRINT("\n");
    if(setup->isDpApi)
    {
        PRINT("API                   Data_Plane\n");
    }
    else
    {
        PRINT("API                   Traditional\n");
    }
    /*stop crypto services if not already stopped, this is the only reasonable
     * location we can do this as this function is called after all threads are
     * complete*/
    status = stopCyServices();
    if(CPA_STATUS_SUCCESS != status)
    {
        /*no need to print error, stopCyServices already does it*/
	PRINT("stopCyServices Failed with status : %d ",status);
        return ;
    }

}
#endif

#ifndef NEWDISPLAY


void accumulateSymPerfData(Cpa32U numberOfThreads,
        perf_data_t* performanceStats[],
        perf_data_t* stats,
        symmetric_test_params_t *setup,
        Cpa64U* buffersProcessed,
        Cpa32U* responsesPerThread)
{
    Cpa32U i = 0;


    /*accumulate the responses into one perf_data_t structure*/
    for (i=0; i<numberOfThreads; i++)
    {
        stats->responses += performanceStats[i]->responses;
        /*is the data was submitted in multiple buffers per list, then the
         * number of buffers processed is  number of responses multiplied
         * by the numberOfBuffers*/
        if(setup->isMultiSGL)
        {
            *buffersProcessed +=
                performanceStats[i]->responses * setup->numBuffers;
        }
        else
        {
            *buffersProcessed += performanceStats[i]->responses;
        }
        stats->retries += performanceStats[i]->retries;
        stats->numOperations += performanceStats[i]->numOperations;
        *responsesPerThread = performanceStats[i]->responses;
        clearPerfStats(performanceStats[i]);
    }

}

/**
 *****************************************************************************
 * @ingroup sampleSymmetricPerf
 *
 * @description
 * print out performance data from a collection of threads that
 * were all running the same setup
 ******************************************************************************/
CpaStatus printSymmetricPerfDataAndStopCyService(thread_creation_data_t* data)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    perf_data_t stats = {0};
    perf_cycles_t numOfCycles = 0;
    Cpa32U responsesPerThread = 0;
    Cpa32U thoughputSize = 0;
    Cpa32U devThoughput = 0;
    Cpa32U throughput = 0;
    Cpa64U buffersProcessed = 0;
    int i = 0;
    symmetric_test_params_t *setup= (symmetric_test_params_t *)data->setupPtr;
    Cpa32U *perfDataDeviceOffsets;
    Cpa32U *threadCountPerDevice;
    perf_data_t *stats2;

/*
    Cpa32U perfDataDeviceOffsets[packageIdCount_g];
    Cpa32U threadCountPerDevice[packageIdCount_g];
    perf_data_t stats2[packageIdCount_g];
*/



    
    /*stop crypto services if not already stopped, this is the only reasonable
     * location we can do this as this function is called after all threads are
     * complete*/
    status = stopCyServices();
    if(CPA_STATUS_SUCCESS != status)
    {
        /*no need to print error, stopCyServices already does it*/
        return status;
    }
    stats2 = qaeMemAlloc(sizeof( perf_data_t) * (packageIdCount_g + 1));
    if(NULL == stats2)
    {
        PRINT_ERR("Error allocating memory for performance stats\n");
        return CPA_STATUS_FAIL;
    }
    perfDataDeviceOffsets = qaeMemAlloc(sizeof(Cpa32U) * (packageIdCount_g + 1));
    if(NULL == perfDataDeviceOffsets)
    {
        PRINT_ERR("Error allocating memory for performance stats\n");
        qaeMemFree((void**)&stats2);
        return CPA_STATUS_FAIL;
    }
    threadCountPerDevice = qaeMemAlloc(sizeof(Cpa32U) * (packageIdCount_g + 1));
    if(NULL == threadCountPerDevice)
    {
        PRINT_ERR("Error allocating memory for performance stats\n");
        qaeMemFree((void**)&stats2);
        qaeMemFree((void**)&perfDataDeviceOffsets);
        return CPA_STATUS_FAIL;
    }
    for(i = 0; i< data->numberOfThreads; i++)
    {
        if(CPA_STATUS_FAIL == data->performanceStats[i]->threadReturnStatus)
        {
            qaeMemFree((void**)&stats2);
            qaeMemFree((void**)&perfDataDeviceOffsets);
            qaeMemFree((void**)&threadCountPerDevice);
            return CPA_STATUS_FAIL;
        }
    }
    memset(stats2, 0, sizeof(perf_data_t)*(packageIdCount_g + 1));
    /*point perf stats to clear structure*/
    setup->performanceStats = &stats;
    for(i=0; i < (packageIdCount_g + 1); i++)
    {
        setup->performanceStats = &stats2[i];
        stats2[i].averagePacketSizeInBytes = data->packetSize;
        if(setup->performanceStats->averagePacketSizeInBytes == PACKET_IMIX)
        {
        	stats2[i].averagePacketSizeInBytes = BUFFER_SIZE_1152;
        }
    }
    /*get our test bufferSize*/
    stats.averagePacketSizeInBytes = data->packetSize;
    thoughputSize = data->packetSize;
    if ( data->packetSize == PACKET_IMIX)
    {
        thoughputSize = setup->performanceStats->averagePacketSizeInBytes;
    }
    /*get the lowest and highest cycle count from the list of threads (all the
     * same setup executed*/
    getLongestCycleCount2(stats2, data->performanceStats,
            data->numberOfThreads, perfDataDeviceOffsets, threadCountPerDevice);


    /*calc the total cycles of all threads (of one setup type) took to complete
     * and then print out the data*/
    for(i=0; i < (packageIdCount_g + 1); i++)
    {
        accumulateSymPerfData(threadCountPerDevice[i],
                &(data->performanceStats[perfDataDeviceOffsets[i]]),
                    &stats2[i],
                    setup,
                    &buffersProcessed, &responsesPerThread);

        numOfCycles = (stats2[i].endCyclesTimestamp - stats2[i].startCyclesTimestamp);
        if(!signOfLife)
        {
            devThoughput = getThroughput(buffersProcessed,
            thoughputSize,
            numOfCycles);
        }
        buffersProcessed = 0;
        throughput += devThoughput;
        stats.numOperations += stats2[i].numOperations;
        stats.responses += stats2[i].responses;
        stats.retries += stats2[i].retries;
#ifdef LATENCY_CODE
        if (isLatencyEnabled())
        {
            stats.minLatency += stats2[i].minLatency;
            stats.aveLatency += stats2[i].aveLatency;
            stats.maxLatency += stats2[i].maxLatency;
        }
#endif
    }


    printSymTestType(setup);
    if(setup->performanceStats->averagePacketSizeInBytes == PACKET_IMIX)
    {
        PRINT("Packet Mix\
        40%%-64B 20%%-752B 35%% 1504B 5%%-8892B\n");
    }
    else
    {
        PRINT("Packet Size           %u\n",
                setup->performanceStats->averagePacketSizeInBytes);
    }
    PRINT("Number of Threads     %u\n", data->numberOfThreads);
    PRINT("Total Submissions     %llu\n",
                                    (unsigned long long )stats.numOperations);
    PRINT("Total Responses       %llu\n",(unsigned long long )stats.responses);
    PRINT("Total Retries         %u\n",stats.retries);
    if(!signOfLife)
    {
        PRINT("CPU Frequency(kHz)    %u\n", sampleCodeGetCpuFreq());
        if(responsesPerThread < THROUGHPUT_MIN_SUBMISSIONS)
        {
            PRINT("Need to submit >= %u per thread for accurate throughput\n",
                    THROUGHPUT_MIN_SUBMISSIONS);
        }
        else
        {
            PRINT("Throughput(Mbps)      %u\n", throughput);
        }


#ifdef LATENCY_CODE
        if (isLatencyEnabled())
        {
            perf_cycles_t   uSecs = 0;
            perf_cycles_t   cpuFreqKHz = sampleCodeGetCpuFreq();

            /* Display how long it took on average to process a buffer in uSecs
             * Also include min/max to show variance */
            assert(cpuFreqKHz != 0);
            assert(data->numberOfThreads != 0);

            do_div(stats.minLatency, data->numberOfThreads);
            uSecs = (perf_cycles_t)(1000 * stats.minLatency / cpuFreqKHz);
            PRINT("Min. Latency (uSecs)   %llu\n", uSecs);
            do_div(stats.aveLatency, data->numberOfThreads);
            uSecs = (perf_cycles_t)(1000 * stats.aveLatency / cpuFreqKHz);
            PRINT("Ave. Latency (uSecs)   %llu\n", uSecs);
            do_div(stats.maxLatency, data->numberOfThreads);
            uSecs = (perf_cycles_t)(1000 * stats.maxLatency / cpuFreqKHz);
            PRINT("Max. Latency (uSecs)   %llu\n", uSecs);
        }
#endif
    }
    qaeMemFree((void **)&stats2);
    qaeMemFree((void **)&perfDataDeviceOffsets);
    qaeMemFree((void **)&threadCountPerDevice);
    return CPA_STATUS_SUCCESS;
}
#endif

/**/
CpaStatus switchCipherDirection()
{
    PRINT("New cipher direction: ");
    if(cipherDirection_g == CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT)
    {
        PRINT("CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT\n");
        cipherDirection_g = CPA_CY_SYM_CIPHER_DIRECTION_DECRYPT;
    }
    else
    {
        PRINT("CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT\n");
        cipherDirection_g = CPA_CY_SYM_CIPHER_DIRECTION_ENCRYPT;
    }
    return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(switchCipherDirection);

/*****************************************************************************
* @ingroup sampleSymmetricDpPerf
*
* @description
* Set the digestAppend flag to true or false
* ***************************************************************************/
CpaStatus setDigestAppend(CpaBoolean flag)
{
   digestAppended_g = flag;
   return CPA_STATUS_SUCCESS;
}
EXPORT_SYMBOL(setDigestAppend);



/************************************************************************
 *  * Name: checkCapability
 *   * Description: Checks whether the given logical instance supports
 *    *  cipherAlg/hashAlg
 *     * Return : True if supported, False, otherwise
 *      ************************************************************************/

CpaBoolean checkCapability(CpaInstanceHandle *cyInstanceHandle,
                           symmetric_test_params_t *symTestSetup)
{
    CpaCySymCapabilitiesInfo cySymCapInfo={};
    CpaCySymCapabilitiesInfo *pCapInfo = NULL;

    if(!(symTestSetup->setupData.cipherSetupData.cipherAlgorithm | symTestSetup->setupData.hashSetupData.hashAlgorithm))
    {
        return CPA_FALSE;
    }
    if (CPA_STATUS_SUCCESS !=
             cpaCySymQueryCapabilities(cyInstanceHandle,&cySymCapInfo))
    {
        PRINT_ERR("cpaCySymQueryCapabilities fail\n");
        return CPA_FALSE;
    }
    pCapInfo = &cySymCapInfo;
    switch(symTestSetup->setupData.symOperation)
    {
        case  CPA_CY_SYM_OP_CIPHER:
            if(symTestSetup->setupData.cipherSetupData.cipherAlgorithm &&
               ((CPA_BITMAP_BIT_TEST(pCapInfo->ciphers,
                  symTestSetup->setupData.cipherSetupData.cipherAlgorithm))
                    == CPA_FALSE))
            {
                PRINT("\nUn supported Cipher ");
                printCipherAlg(symTestSetup->setupData.cipherSetupData);
                return CPA_FALSE;
            }
            break;
        case  CPA_CY_SYM_OP_HASH:
            if(symTestSetup->setupData.hashSetupData.hashAlgorithm &&
               ((CPA_BITMAP_BIT_TEST(pCapInfo->hashes,
                  symTestSetup->setupData.hashSetupData.hashAlgorithm))
                    == CPA_FALSE))
            {
                PRINT("\nUn supported Hash ");
                printHashAlg(symTestSetup->setupData.hashSetupData);
                return CPA_FALSE;
            }
            break;
        case  CPA_CY_SYM_OP_ALGORITHM_CHAINING:
            if(symTestSetup->setupData.cipherSetupData.cipherAlgorithm &&
               ((CPA_BITMAP_BIT_TEST(pCapInfo->ciphers,
                  symTestSetup->setupData.cipherSetupData.cipherAlgorithm))
                    == CPA_FALSE))
            {
                PRINT("\nUn supported AlgChain ");
                printCipherAlg(symTestSetup->setupData.cipherSetupData);
                return CPA_FALSE;
            }
            if(symTestSetup->setupData.hashSetupData.hashAlgorithm &&
               ((CPA_BITMAP_BIT_TEST(pCapInfo->hashes,
                  symTestSetup->setupData.hashSetupData.hashAlgorithm))
                    == CPA_FALSE))
            {
                PRINT("\nUn supported AlgChain ");
                printHashAlg(symTestSetup->setupData.hashSetupData);
                return CPA_FALSE;
            }
            break;
        default:
            PRINT_ERR("\nUn supported Sym operation: %d\n",symTestSetup->setupData.symOperation);
            return CPA_FALSE;
    }
    return CPA_TRUE;
}
EXPORT_SYMBOL(checkCapability);
EXPORT_SYMBOL(setCyPollWaitFn);
