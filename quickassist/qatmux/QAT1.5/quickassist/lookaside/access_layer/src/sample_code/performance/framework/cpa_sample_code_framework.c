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
 *****************************************************************************
 * Doxygen group definitions
 ****************************************************************************/

/**
 *****************************************************************************
 * @file cpa_sample_code_framework.c
 *
 * @defgroup perfCodeFramework peformance sample code framework
 *
 * @ingroup perfCodeFramework
 *
 * @description
 *      This file contains the test creation functions of the performance sample
 *      code framework.
 *      Functions contained in this file:
 *        - startThreads
 *        - waitForThreadCompletion
 *        - createPerformanceThreads
 *        - killCreatedThreads
 *        - clearPerfStats
 *
 *      functions within allow to create and run multiple threads.
 *
 *      The user defines the test setup(s) and function(s) to run and uses
 *      functions implemented here to create, start and wait for the threads to
 *      complete
 *
 *      the setup and functions called are defined elsewhere, but they must fit
 *      within the thread_creation_data_t and single_thread_test_data_t
 *      structures. Examples of how these are set can be seen in the crypto
 *      folder cpa_sample_code_crypto_utils.c ->setupSymetricTest
 *****************************************************************************/

#include "cpa_sample_code_framework.h"

/******************************************************************************
 * GLOBAL VARIABLES
 * These variables are used in the framework to provide setup information, a
 * location to write performance stats, count the number of threads and types of
 * threads, and to control the start of user space threads. THEY ARE NOT THREAD
 * SAFE. The only variable that the user should need to use is the
 * thread_setup_g array
 * The user will be required to write some setup data to this array.
 *
 * testTypeCount_g is used to control where the user writes there setup
 * data in thread_setup_g array
 *
 * The framework code ensures that writing to these variables is only done
 * before threads start and after they all complete. The only exception to this
 * is performance stats, where each thread as a unique location to store stats
 *
 ******************************************************************************/

/*store the threadId, this only used for thread setup and after threads are
 * complete, so it not required to be atomic*/
sample_code_thread_t threads_g[MAX_THREADS];

/*declare multi dimension array of stats so that we can create upto
 * MAX_NUM_OF_ONE_TYPE_OF_THREAD, and we can create up to MAX_THREAD_VARIATION
 * each thread has its own performance stats within this array, as this is array
 * is not atomic there is no sharing of these stats between threads*/
perf_data_t perfStats_g[MAX_THREAD_VARIATION][MAX_NUM_OF_ONE_TYPE_OF_THREAD];

/*this array stores the setup and performance data of all threads created.
 * there is duplication between this and testSetupData_g,
 * however this makes it easier for collation of stats when there are
 * multiple creations of one type of thread.
 * There is no sharing data between threads*/
single_thread_test_data_t singleThreadData_g[MAX_THREADS];

/*this array stores the setup and performance data of ONE_TYPE_OF_THREAD. This
 * is updated on thread setup and read and clear once all threads are complete*/
thread_creation_data_t testSetupData_g[MAX_THREAD_VARIATION];

/*declare space to store setup structures in. This stores all the setup of each
 * thread, there is no sharing between threads, so each section of the array
 * is autonomous to 1 thread*/
Cpa8U thread_setup_g[MAX_THREAD_VARIATION][MAX_SETUP_STRUCT_SIZE_IN_BYTES];
Cpa8U thread_name_g[MAX_THREAD_VARIATION][THREAD_NAME_LEN];

/*global variables -note syntax is to use _g for all global variables*/

/*this is used to count the total number of threads created, this is
 * incremented during the thread creation phase once threads are started it
 * is read only and set to zero when all threads are finished*/
int numCreatedThreads_g = 0;

int numThreadsToCreate_g = 0;

/*this is used to count the number of different thread variations created,
 * it is incremented during thread setup and is read only once threads are
 * started. It is set to zero one all threads are complete*/
int testTypeCount_g = 0;
Cpa32U packageIdCount_g = 0;
EXPORT_SYMBOL(packageIdCount_g);
CpaBoolean devicesCounted_g = CPA_FALSE;
/*this is used to limit the maximum number of cores uses in the thread
 * creation*/
Cpa32U coreLimit_g = 0;

/*this is used to stop user space threads from executing to the main barrier
 * before all threads are created*/
/*this is a shared mutex between user space threads, if one thread locks this
 * all other threads wait until its released, which is done by calling the
 * cpa_sample_code_mutex_unlock ot cpa_sample_code_barrier_wait function*/
sample_code_thread_mutex_t threadControlMutex_g;

/*this is the conditional varible that user space threads wait on to start, the
 * conditional varible is broadcast on call of startThreads, sends a broadcast
 * to all user space threads waiting at cpa_sample_code_barrier, each thread
 * attempts to lock the mutex above, restart its code, then release the mutex*/
sample_code_thread_cond_t threadConditionControl_g;

/* this is used to stop user space threads from executing once registered with
 * the framework.
 * This mutex is initialized by startBarrierInit() and the locking/unlocking
 * mechanism is supported in startBarrier().
 */
sample_code_thread_mutex_t startThreadControlMutex_g;

/*this is the conditional varible that user space threads wait on to start, the
 * conditional varible is broadcast on call of startThreads, sends a broadcast
 * to all user space threads waiting at cpa_sample_code_barrier, each thread
 * attempts to lock the mutex above, restart its code, then release the mutex*/
sample_code_thread_cond_t startThreadConditionControl_g;

/*this is local to this file and it only used once thread setup to indicate
 * wheater threadControlMutex_g and threadConditionControl_g have been
 * initilised. It is reset to false once all threads are complete*/
CpaBoolean threadControlInitilised_g = CPA_FALSE;

/*this is local to this file and it only used check if all the threads have
 * been started, before that waitForCompletion function will hang forever
 * waiting for threads to end their work.*/
CpaBoolean threadControlStarted_g = CPA_FALSE;



volatile CpaBoolean poll_inline_g = CPA_FALSE;


/***************************************************************************
 * FUNCTION IMPLEMENTATIONS
 * *************************************************************************/

/* This function calls the OS to start all the created threads*/
CpaStatus startThreads(void) {
    int threadId = 0;
    FUNC_ENTRY();
    /*broadcast to user space threads that they can start, this does nothing
     * in kernel space*/
    sampleCodeSleep(1);
    sample_code_thread_cond_broadcast(&startThreadConditionControl_g);
    /*start all threads*/
    for (threadId = 0; threadId < numCreatedThreads_g; threadId++) {
        if (sampleCodeThreadStart(&threads_g[threadId]) != CPA_STATUS_SUCCESS)
            {
            /* if we cant start one thread we kill all created threads
             * and return fail*/
            for (threadId = 0; threadId < numCreatedThreads_g; threadId++)
            {
                sampleCodeThreadKill(&threads_g[numCreatedThreads_g--]);
            }
            PRINT("startThreads: we cant start one thread CPA_STATUS_FAIL \n");
            return CPA_STATUS_FAIL;
        }
    }
    /* after that each thread has started, set the flag that indicate
     * startThreads has been called*/
    threadControlStarted_g = CPA_TRUE;
    FUNC_EXIT();
    return CPA_STATUS_SUCCESS;
}

/*This functions waits for all threads to complete including all perform
 * operations. It is required to wait for all threads to complete so that
 * performance stats can be collated*/
CpaStatus waitForThreadCompletion(void) {
    CpaStatus status = CPA_STATUS_SUCCESS;
    int i = 0;
    stats_print_func_t statsPrintFunc;
    FUNC_ENTRY();

    /* Check if starThread has been called*/
    if (threadControlStarted_g) {
        /* Wait for all threads_g to complete */
        for (i = 0; i < numCreatedThreads_g; i++)
        {
            sampleCodeThreadJoin(&threads_g[i]);
            if(CPA_STATUS_FAIL ==
                singleThreadData_g[i].performanceStats->threadReturnStatus)
            {
                status = CPA_STATUS_FAIL;
            }
        }
        /*print out collated stats for all types of threads*/
#ifndef NEWDISLAY
        PRINT("---------------------------------------\n");
#endif
        for (i = 0; i < testTypeCount_g; i++) {
            statsPrintFunc = *(testSetupData_g[i].statsPrintFunc);
            if (statsPrintFunc != NULL) {
                statsPrintFunc(&testSetupData_g[i]);
            } else {
                PRINT_ERR("Unable to print stats for thread variation %d\n", i);
            }
            if (i < testTypeCount_g - 1) {
                PRINT("\n");
            }
        }
#ifndef NEWDISLAY
        PRINT("---------------------------------------\n\n");
#endif
        /* Clean up and exit */
        threadControlInitilised_g = CPA_FALSE;
        threadControlStarted_g = CPA_FALSE;
        numCreatedThreads_g = 0;
        testTypeCount_g = 0;
        packageIdCount_g = 0;
        devicesCounted_g = CPA_FALSE;
        sampleCodeBarrierDestroy();

    }
    else
    {
        PRINT_ERR("startThreads() has not been called\n");
    }
    FUNC_EXIT();

    return status;
}

/*this function limits the number of cores to be used in the create threads
 * function*/
CpaStatus setCoreLimit(Cpa32U limit) {
    Cpa32U nProcessorsOnline = sampleCodeGetNumberOfCpus();
    if (limit > nProcessorsOnline) {
        PRINT_ERR("exceeds number of cores (%u) on system\n",
                nProcessorsOnline);
        return CPA_STATUS_FAIL;
    }
    coreLimit_g = limit;
    return CPA_STATUS_SUCCESS;
}

/*This function creates threads based on the pre-condition that the user has
 * called a setup function that has populated the oneTypeOfThreadData. This
 * function creates threads based on what is in the oneTypeOfThreadData at the
 * current threadVariationCount (which is an index into the oneTypeOfThreadData
 * array.
 * This function replicates threads across cores*/
CpaStatus createPerfomanceThreads(Cpa32U numLogicalIaCoresToUse,
        Cpa32U* logicalIaCore, Cpa32U numberLogicalInstancesToUse,
        Cpa32U startingQaLogicalInstanceOffset) {
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32U i = 0;

    Cpa32U qaLogicalInstance = startingQaLogicalInstanceOffset;
    Cpa32U totalNumberOfThreads = 0;
    /*this is the total number of threads to create of the current setup and
     * depends on either how many cores are on the SUT or how many the user
     * wants to use*/

    performance_func_t functionPtr = NULL;
    Cpa32U nProcessorsOnline = 0;
    sample_code_thread_attr_t *threadAttr = NULL;

    if (testSetupData_g[testTypeCount_g].performance_function == NULL) {
        PRINT_ERR("No thread function has been set\n");
        return CPA_STATUS_FAIL;
    }
    /*initilise thread control for user space, in kernel space this code
     * does nothing, we only do this once*/
    if (threadControlInitilised_g == CPA_FALSE) {
        sampleCodeBarrierInit();
        startBarrierInit();
        threadControlInitilised_g = CPA_TRUE;
    }
    /*1st we check that we still have room to store details of the threads to
     * be created*/
    if (testTypeCount_g >= MAX_THREAD_VARIATION) {
        PRINT_ERR("Maximum Support Thread Variation has been exceeded\n");
        PRINT_ERR("Number of Thread Variations created: %d",
                testTypeCount_g);
        PRINT_ERR(" Max is %d\n", MAX_THREAD_VARIATION);
        return CPA_STATUS_FAIL;
    }

    /*get the number of cores on the system*/
    nProcessorsOnline = sampleCodeGetNumberOfCpus();
    /*calculate the number of threads to be setup*/
    if (numLogicalIaCoresToUse != USE_ALL_CORES)
    {
        totalNumberOfThreads = numLogicalIaCoresToUse;
    }
    else
    {
        if (coreLimit_g > 0)
        {
            totalNumberOfThreads = coreLimit_g;
        }
        else
        {
            totalNumberOfThreads = nProcessorsOnline;
        }
    }
    /*populate the threadData we area creating*/
    /*&thread_setup_g[testTypeCount_g][0] is a pointer to an area of memory
     * that contains the setup, so get the base address of this memory location
     * */
    testSetupData_g[testTypeCount_g].setupPtr
            = &thread_setup_g[testTypeCount_g][0];
    testSetupData_g[testTypeCount_g].numberOfThreads = totalNumberOfThreads;

    functionPtr = testSetupData_g[testTypeCount_g].performance_function;

    /* Set numThreadsToCreate so that if any thread rushes to doing its actual processing,
     * it's stopped by sampleCodeBarrier until number of threads reaching barrier becomes
     * equal to this variable
     */
    numThreadsToCreate_g = totalNumberOfThreads;

    /* Initialise and start the barrier to stop all threads from running once
     * sampleCodeThreadCreate is called.
     */


    /*populate threadData with performance stats,
     * setupData and sessions etc...*/
    for (i = 0; i < totalNumberOfThreads; i++) {
        /*wrap around core to use if we have exceed max number of cores*/
        if (logicalIaCore[i] >= nProcessorsOnline) {
            PRINT("Warning we have reached core limit ");
            PRINT("resetting core to 0\n");
            logicalIaCore[i] = 0;
        }
        /*check that we have not reached allocated memory limit to store
         * thread data*/
        if (numCreatedThreads_g >= MAX_THREADS) {
            PRINT_ERR("Maximum Support Thread has been reached\n");
            PRINT_ERR("Number of Threads created: %d ", numCreatedThreads_g);
            PRINT_ERR("Max is %d\n", MAX_THREADS);
            /*kill any threads we have created in this current function
             * context*/
            killCreatedThreads(i);
            return CPA_STATUS_FAIL;
        }
        testSetupData_g[testTypeCount_g].performanceStats[i]
                = &perfStats_g[testTypeCount_g][i];
        testSetupData_g[testTypeCount_g].statsPrintFunc
                = &singleThreadData_g[numCreatedThreads_g].statsPrintFunc;
        /*&thread_setup_g[testTypeCount_g][0] is a pointer to an area of memory
         * that contains the setup, so get the base address of this memory
         * location, the thread that we call is expected to be able to cast this
         * memory location into the structure that it expects*/
        singleThreadData_g[numCreatedThreads_g].setupPtr
                = &thread_setup_g[testTypeCount_g][0];
        singleThreadData_g[numCreatedThreads_g].performanceStats
                = &perfStats_g[testTypeCount_g][i];
        singleThreadData_g[numCreatedThreads_g].packetSize
                = testSetupData_g[testTypeCount_g].packetSize;
        singleThreadData_g[numCreatedThreads_g].logicalQaInstance
                = qaLogicalInstance;
        singleThreadData_g[numCreatedThreads_g].threadID = numCreatedThreads_g;
        //threadAttr.name = (char*)&thread_name_g[testTypeCount_g][0];

        status = sampleCodeThreadCreate(&threads_g[numCreatedThreads_g],
                threadAttr, functionPtr,
                &singleThreadData_g[numCreatedThreads_g]);

        if (status != CPA_STATUS_SUCCESS) {
            PRINT_ERR("Failed to create thread: %d", i);
            /*we can safely kill all threads in kernel space because
             *  they have not started...in user space,  dynamically
             *  allocated memory is assumed to be freed by the OS*/
            killCreatedThreads(i);
            return CPA_STATUS_FAIL;
        } else {

            /*tie a create thread to a specific core and increment
             * the threads created*/
#if defined (_WIN64) || defined (WIN32)
            singleThreadData_g[numCreatedThreads_g].performanceStats->logicalCoreAffinity =
                logicalIaCore[i];
#endif
            sampleCodeThreadBind(&threads_g[numCreatedThreads_g++],
                    logicalIaCore[i]);
        }
        qaLogicalInstance++;
        /*wrap around qaLogicalInstance if we have used all that was intended
         * to use
         * WARNING this framework is not aware of the number of QA instances
         * in the system it assumes the user will pass in the right
         * information*/
        if (qaLogicalInstance - startingQaLogicalInstanceOffset
                == numberLogicalInstancesToUse) {
            qaLogicalInstance = startingQaLogicalInstanceOffset;
        }

    }
    testTypeCount_g++;

    return CPA_STATUS_SUCCESS;

}

/* Commenting to prevent the compilation error in package
 * #include <pthread.h>
 * */
void killCreatedThreads(int numThreadsToKill) {
    int j = 0;
    /*kill any create threads*/
    for (j = numThreadsToKill; j >= 0; j--) {
        //pthread_cancel(threads_g[numCreatedThreads_g--]);
        //sampleCodeThreadKill(&threads_g[numCreatedThreads_g--]);
    }

}

void clearPerfStats(perf_data_t *stats) {
    memset(stats, 0, sizeof(perf_data_t));
}


void getLongestCycleCount(perf_data_t *dest,
        perf_data_t *src[],
        Cpa32U count)
{
    Cpa32U i = 0;

    if(NULL != dest && NULL != src && NULL != src[0])
    {
        dest->startCyclesTimestamp = src[0]->startCyclesTimestamp;
        dest->endCyclesTimestamp = src[0]->endCyclesTimestamp;
    }
    else
    {
        PRINT_ERR("Cannot get cycle counts,"
                        " performance data points to NULL\n");
        return;
    }
    for(i=1;i<count;i++)
    {
        /*get the lowest start time*/
        if((src[i]->startCyclesTimestamp <
                dest->startCyclesTimestamp) )
        {
            dest->startCyclesTimestamp = src[i]->startCyclesTimestamp;
        }
        /*get the high finish time*/
        if((src[i]->endCyclesTimestamp > dest->endCyclesTimestamp) )
        {
            dest->endCyclesTimestamp = src[i]->endCyclesTimestamp;
        }
    }
}

void getLongestCycleCount2(perf_data_t* dest,
        perf_data_t *src[],
        Cpa32U count,
        Cpa32U *perfDataDeviceOffsets,
        Cpa32U *threadCountPerDevice)
{
    Cpa32U i = 0;
    Cpa32U j = 0;

    /*zero out perfDataDeviceOffsets*/
    for(i=0; i < (packageIdCount_g + 1); i++)
    {
        threadCountPerDevice[i] = 0;
        perfDataDeviceOffsets[i] = 0;
    }
    threadCountPerDevice[0] = 1;
    for(i = 1; i<count; i++)
    {
        if (src[i]->packageId > src[i-1]->packageId)
        {
            j++;
            perfDataDeviceOffsets[j] = i;
        }
        threadCountPerDevice[j]++;
    }
    for(i=0; i < (packageIdCount_g + 1); i++)
    {
        getLongestCycleCount(&dest[i],&src[perfDataDeviceOffsets[i]],threadCountPerDevice[i]);
    }
}
