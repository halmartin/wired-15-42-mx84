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

/**
*****************************************************************************
 * @file cpa_sample_code_framework.h
 *
 * This file provides protypes, macros and structures required in the sample
 * code framework
 *
 *****************************************************************************/
#ifndef _SAMPLECODEFRAMEWORK_H__
#define _SAMPLECODEFRAMEWORK_H__

#include "cpa.h"
#include "cpa_sample_code_utils_common.h"

#ifdef WITH_CPA_MUX
#include "cpa_impl_mux.h"
#endif

/*the limit of the number of different types of threads to be created*/
#define MAX_THREAD_VARIATION                        (300)
#define THREAD_NAME_LEN                                (4)
#define CORE_MASK_MSB                               (0x10000000)

/*the limit of the number of one type of thread, should be at least the number
 * of cores on the system we want to test*/
#define MAX_NUM_OF_ONE_TYPE_OF_THREAD               (80)

/*total number of threads that the framework will support*/
#define MAX_THREADS     MAX_THREAD_VARIATION*MAX_NUM_OF_ONE_TYPE_OF_THREAD

/* this is used to allocate space for all types test setups, this is unknown to
 * the framework.
 * the size of this is based on the assumption that the CpaCySymSessionSetupData
 * is the largest steup structure in the QA API's so we set our size to be 2
 * times the size of CpaCySymSessionSetupData ~90bytes to allow enough room for
 * all setup types*/
#define MAX_SETUP_STRUCT_SIZE_IN_BYTES                       (256)

/*the following macros are defined for default cores to be used in tests,
 * */
/*some functions, such as symmetric crypto have multi purpose (hash, cipher)
 * this means that some parameters passed to functions are not used, we define
 * and use this macro in the code where this an input parameter is not used*/
#define NOT_USED                                    (0)

/*flag to indicate to createThreads function to create and bind a thread on all
 *  Available cores or just one core*/
#define USE_ALL_CORES                               (0)
#define USE_ONE_CORE                                (1)

/*flag to indicate to createThreads function start creating threads for this
 * core id and upwards*/
#define DEFAULT_CORE_OFFSET                      (0)

/*flag to the createThread function to attempt to use as many qa logical
 * instances as possible, however as the framework applies 1 thread per core
 * and each threads uses its own qaInstance, and there is generally more
 * qaInstances available then cores means that use of all qaLogicalInstances
 * is normally not possible*/
#define USE_ALL_QA_LOGICAL_INSTANCES              (0)
#define USE_ONE_QA_LOGICAL_INSTANCE               (1)

/*this defines the 1st qaLogicalInstance to be used with the 1st create thread
 * the each threads uses and incremental instance from this offset*/
#define DEFAULT_LOGICAL_INST_INSTANCE_OFFSET        (0)

#define DEFAULT_MAP         (0x1)
#define CRYPTO              (1)
#define COMPRESSION         (2)

/**
 *****************************************************************************
 * @ingroup perfCodeFramework
 *      Thread Creation Setup Data.
 * @description
 *      This structure contains data relating to setting and replicating a
 *      thread across cores
 *
 ****************************************************************************/
typedef struct thread_creation_data_s
{
    performance_func_t performance_function;
    /*function to launch as a thread to measure performance */
    void* setupPtr;
    /*pointer to setup data specific to the performance Test type*/
    Cpa32U numberOfThreads;
    /*stores the number of threads to be created with the data store in
     * setupPtr*/
    perf_data_t* performanceStats[MAX_THREADS];
    /*stores performance stats for all theads of same test_type and same setup*/
    Cpa32U packetSize;
    /*flat buffer size to be tested*/
    stats_print_func_t *statsPrintFunc;
    /*pointer to function capable of printing our stat related to specific
     * test varation*/
}thread_creation_data_t;

/**
 *****************************************************************************
 * @ingroup perfCodeFramework
 *      Thread Creation Setup Data.
 * @description
 *      This structure contains data relating to a single thread
 *
 ****************************************************************************/
typedef struct single_thread_test_data_s
{
    void* setupPtr;
    /*pointer to setup data specific to the performance Test type*/
    Cpa32U numberOfThreads;
    perf_data_t* performanceStats;
    /*performance stats points to one of the performanceStats of
     * thread_creation_data*/
    Cpa32U packetSize;
    /*flat buffer size to be tested*/
    Cpa32U logicalQaInstance;
    /*the logicalQaInstance for the thread to use*/
    stats_print_func_t statsPrintFunc;
    /*pointer to function capable of printing our status related to specific
     * test variation*/
    Cpa32U threadID;
    /* Unique Thread ID based on the order in which the thread is created */
}single_thread_test_data_t;



extern volatile CpaBoolean poll_inline_g;

/* *****************************************************************************
 * FUNCTION PROTOTYPES
 * ****************************************************************************/
/**
 *****************************************************************************
 * @ingroup perfCodeFramework
 *      setCoreLimit
 *
 * @description
 *      this function limits the number of cores to be used in the create
 *      threads function
 *
 * @threadSafe
 *      No
 *
 *
 *
 * @param[in]      limit    the number of cores to create  threads on
 *
 *
 * @retval This function returns
 *  CPA_STATUS_SUCCESS when the limit is less than the number of cores on the
 *  system
 *  CPA_STATUS_FAIL when limit is > number of cores on the system
 *
 * @pre
 *      none
 * @post
 *      threads will be created on the limited number or cores
 *
 *****************************************************************************/
CpaStatus setCoreLimit(Cpa32U limit);

/* *****************************************************************************
 * FUNCTION PROTOTYPES
 * ****************************************************************************/
/**
 *****************************************************************************
 * @ingroup perfCodeFramework
 *      createPerfomanceThreads
 *
 * @description
 *      This function creates threads. The threads are created across cores
 *      and to use separate qaLogicalInstances
 *
 * @threadSafe
 *      No
 *
 *
 *
 * @param[in]      numLogicalIaCoresToUse    the number of cores to create
 *                  threads on
 *
 * @param[in]   logicalIaCore - array containing cores to bind threads to
 *
 * @param[in]   numberLogicalInstancesToUse the number of qaLogicalInstancs to
 *              use, note the implementation will create the number of threads
 *              bases on the lower of numberOfIaCores to use or
 *              numberLogicalInstancesToUse
 *
 * @param[in]   startingLogicalInstanceOffset qaLogicalInstance to be used in
 *              1st threads, each subsequent thread increments this value
 *
 * @retval This function returns
 *  CPA_STATUS_SUCCESS when all threads were created
 *  CPA_STATUS_FAIL when some thing went wrong, an error should be printed to
 *      STDOUT in this case
 *
 * @pre
 *      user defined setup has been called which populates thread_setup_g with
 *      all the
 *      parameters and the function required to start the thread
 * @post
 *      Threads are created (but not started in the case of kernel threads, user
 *      threads have a barrier at the start to stop them until all threads
 *      have been created
 *
 *****************************************************************************/
CpaStatus createPerfomanceThreads
(
    Cpa32U numLogicalIaCoresToUse,
    Cpa32U* logicalIaCore,
    Cpa32U numberLogicalInstancesToUse,
    Cpa32U startingLogicalInstanceOffset
);
/**
 *****************************************************************************
 * @ingroup perfCodeFramework
 *      waitForThreadCompletion
 *
 * @description
 *      This function waits for all created threads to complete, then print all
 *      stats to STDOUT
 *
 * @threadSafe
 *      No
 *
 * @retval This function returns the success of the performance threads
 *
 * @pre
 *      threads have been started
 * @post
 *      threads are complete and stats printed to STDOUT
 *
 *****************************************************************************/
CpaStatus waitForThreadCompletion(void);

/**
 *****************************************************************************
 * @ingroup perfCodeFramework
 *      startThreads
 *
 * @description
 *      This function starts all the created threads, or in the case of user
 *      threads frees the barrier to let them continue
 *
 * @threadSafe
 *      No
 *
 * @retval This function returns
 *  CPA_STATUS_SUCCESS when all threads were started
 *  CPA_STATUS_FAIL when some thing went wrong, an error shold be printed to
 *      STDOUT in this case
 *
 * @pre
 *      threads have been started
 * @post
 *      threads are complete and stats printed to STDOUT
 *
 *****************************************************************************/
CpaStatus startThreads(void);

/**
 *****************************************************************************
 * @ingroup perfCodeFramework
 *      killCreatedThreads
 *
 * @description
 *      This function kills the the last "numThreadsToKill" created threads
 *
 *
 * @threadSafe
 *      No
 *
 * @param[in]   numThreadsToKill thsi defines how many threads from the last one
 *              created to be killed
 *
 * @retval This function returns void
 *
 * @pre
 *      threads have been started
 * @post
 *      threads are complete and stats printed to STDOUT
 *
 *****************************************************************************/
void killCreatedThreads(int numThreadsToKill);

/**
 *****************************************************************************
 * @ingroup perfCodeFramework
 *      clearPerfStats
 *
 * @description
 *      clears (zeros) an instance of perf_data_t
 *
 *
 * @threadSafe
 *      No
 *
 * @param[in]   *stats pointer to perf_data_t structure to be cleared
 *
 * @retval This function returns void
 *
 * @pre
 *      none
 * @post
 *      perf_data_t structure pointed to contains all 0's
 *
 *****************************************************************************/
void clearPerfStats(perf_data_t *stats);

CpaStatus muxRegister(void);
CpaStatus muxDeRegister(void);


/**
 *****************************************************************************
 * @ingroup cryptoThreads
 *      getLongestCycleCount
 *
 * @description
 *      get the smallest starting cycle and the largest end cycle from a list
 *      of perf_data_t structures. This function should be used on a collection
 *      of threads all testing the same thread Variation. the src data is set to
 *      zero once read.
 *****************************************************************************/
void getLongestCycleCount(perf_data_t *dest,
        perf_data_t *src[], Cpa32U count);

void getLongestCycleCount2(perf_data_t* dest,
        perf_data_t *src[],
        Cpa32U count,
        Cpa32U *perfDataDeviceOffsets,
        Cpa32U *threadCountPerDevice);


#endif /*_SAMPLECODEFRAMEWORK_H__*/
