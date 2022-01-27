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
 * @file cpa_sample_code_utils.c
 *
 * This file provides linux kernel os abstraction functions
 *
 *****************************************************************************/

#include "cpa_sample_code_utils.h"
#include "cpa_sample_code_utils_common.h"

#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#include <linux/semaphore.h>
#endif
#include <linux/random.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/cpumask.h>

atomic_t bGo;
extern int numCreatedThreads_g;
atomic_t arrived;
#ifdef INCLUDE_COMPRESSION
extern struct device perf_device;
#endif


/* Define a number for timeout */
#define SAMPLE_CODE_MAX_LONG (0x7FFFFFFF)

/* Max timeout in MS, used to guard against possible overflow */
#define SAMPLE_CODE_MAX_TIMEOUT_MS (SAMPLE_CODE_MAX_LONG/HZ)
#define CPU_INFO_REGISTERS                  (4)
#define UPPER_HALF_OF_REGISTER              (32)
/* cpuid for intel e gnu compiler */
#if defined (__INTEL_COMPILER)
#define __CPUID(a, b)\
    __cpuid((a), (b))
#endif

#if defined (__INTEL_COMPILER)
    int CPUInfo[CPU_INFO_REGISTERS] = {-1};
    __CPUID(CPUInfo, 1);
#else /* #if defined (__INTEL_COMPILER) */
#define __CPUID(in,a,b,c,d)\
    asm volatile ("cpuid": "=a" (a), "=b" (b), "=c" (c), "=d" (d) : "a" (in));
#endif

static void __inline__ sampleCodeCpuid(void)
{
    unsigned int a = 0x00, b = 0x00, c= 0x00, d = 0x00;
    __CPUID(0x00, a, b, c, d);
}

static __inline__ Cpa64U sampleCoderdtsc(void)
{
    unsigned long a, d;

    asm volatile ("rdtsc" : "=a" (a), "=d" (d));
    return (((Cpa64U)a) | (((Cpa64U)d) << UPPER_HALF_OF_REGISTER));
}

/*
    Serialized version of the rdtsc instruction
    to properly count cycles need to remove the cpuid overhead

    ticks_t cpuid_cycles = get_cpuid_cycles();
    ticks_t s1 = rdtscp();
    {
        code_to_benchmark();
    }
    ticks_t s2 = rdtscp();
    ticks_t measured_time = s2 - s1 - cpuid_cycles;
*/
static __inline__ Cpa64U sampleCodeRdtscp(void)
{
    volatile unsigned long a = 0, d = 0;
    Cpa64U returnval = 0;

    sampleCodeCpuid();
    asm volatile ("rdtsc" : "=a" (a), "=d" (d));
    returnval = (((Cpa64U)d) << UPPER_HALF_OF_REGISTER);
    returnval |= ((Cpa64U)a);

    return returnval;
}


/*
    This function estimates the number of clock
    cycles for the cpuid instruction
    Needs to run the be run at least 3 times to "warm-up"
    and report a stable number
*/
static __inline__ Cpa64U getCpuidCycles(void)
{
    volatile Cpa64U cpuid_cycles  = 0;
    volatile Cpa64U s1;

    sampleCodeCpuid();
    s1 = sampleCoderdtsc();
    sampleCodeCpuid();
    cpuid_cycles = sampleCoderdtsc();
    cpuid_cycles -= s1;
    //printf("cpuid_cycles: %llu \n", cpuid_cycles);

    sampleCodeCpuid();
    s1 = sampleCoderdtsc();
    sampleCodeCpuid();
    cpuid_cycles = sampleCoderdtsc();
    cpuid_cycles -= s1;
    //printf("cpuid_cycles: %llu \n", cpuid_cycles);

    sampleCodeCpuid();
    s1 = sampleCoderdtsc();
    sampleCodeCpuid();
    cpuid_cycles = sampleCoderdtsc();
    cpuid_cycles -= s1;
    //printf("cpuid_cycles: %llu \n", cpuid_cycles);

    return cpuid_cycles;
}

perf_cycles_t sampleCodeTimestamp(void)
{
    /*get time stamp twice, because we need to prime the timestamp counter*/
    sampleCodeRdtscp();
    return (perf_cycles_t)sampleCodeRdtscp();
}
void sampleCodeSleep(Cpa32U seconds)
{
    set_current_state((long)TASK_INTERRUPTIBLE);
    schedule_timeout (seconds* HZ);
}

void sampleCodeSleepMilliSec(Cpa32U milliseconds)
{
    if (milliseconds != 0)
    {
        set_current_state((long)TASK_INTERRUPTIBLE);
        schedule_timeout ((milliseconds * HZ) / SAMPLE_CODE_THOUSAND);
    }
    else
    {
        schedule ();
    }
}

Cpa32U sampleCodeGetNumberOfCpus(void)
{
    /*struct cpuinfo_x86 *c = &cpu_data(0);
    return (long)(c->x86_max_cores*cpumask_weight(cpu_sibling_mask(0)));*/
    return (Cpa32U)num_online_cpus();
}

Cpa32S sampleCodeGetCpu(void)
{
    return (Cpa32S)get_cpu();
}

Cpa32U sampleCodeGetCpuFreq()
{
    unsigned int ret = cpufreq_quick_get(0);
    if (!ret)
    {
        ret = cpu_khz;
    }
    return (Cpa32U)ret;

}


/*generate random data using the linux build int get_random_bytes functions*/
void generateRandomData(Cpa8U* pWriteRandData, Cpa32U lengthOfRand)
{
    get_random_bytes(pWriteRandData, lengthOfRand);
}

CpaStatus sampleCodeThreadCreate
(
        sample_code_thread_t *threadPtr,
        sample_code_thread_attr_t *threadAttrPtr,
        performance_func_t function,
        void *paramsPtr)
{
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(threadPtr);
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(function);

    *threadPtr = kthread_create((void*)function, paramsPtr, "%s",
            (NULL != threadAttrPtr && NULL != threadAttrPtr->name) ?
                    threadAttrPtr->name : "SAMPLE CODE");
    if(IS_ERR(threadPtr))
    {
        PRINT_ERR("kthread_create failed\n");
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

CpaStatus sampleCodeThreadBind (
        sample_code_thread_t *threadPtr,
        Cpa32U logicalCore)
{

    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(threadPtr);
    kthread_bind(*threadPtr, logicalCore);
    /*kthread_bind returns a void, so we assume success*/
    return CPA_STATUS_SUCCESS;
}


CpaStatus sampleCodeThreadStart (sample_code_thread_t *threadPtr)
{
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(threadPtr);
    wake_up_process(*threadPtr);
    /*wake_up_process returns a void, so we assume success*/
    return CPA_STATUS_SUCCESS;
}


CpaStatus sampleCodeThreadKill (sample_code_thread_t *threadPtr)
{
    struct task_struct *task = NULL;
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(threadPtr);
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(*threadPtr);

    task = *threadPtr;

    /* Can't kill already defunct thread */
    if (EXIT_DEAD == task->exit_state || EXIT_ZOMBIE == task->exit_state)
    {
         return CPA_STATUS_FAIL;
    }

    if (-EINTR == kthread_stop(task))
    {
        PRINT_ERR("sampleCodeThreadKill(): Failed to kill thread\n");

        return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}


CpaStatus sampleCodeThreadPrioritySet(
    sample_code_thread_t *threadPtr,
    Cpa32U priority)
{
    struct task_struct *task = NULL;
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(threadPtr);
    task = *threadPtr;
    if (priority > MAX_PRIORITY_VALUE)
    {
         priority = MAX_PRIORITY_VALUE;
    }
    /* sending the nice equivalent of priority as the parameter */
    set_user_nice ( task, priority - NICE_VAL_DIFFERENCE );
    return CPA_STATUS_SUCCESS;
}

CpaStatus sampleCodeThreadSetPolicyAndPriority(
    sample_code_thread_t *threadPtr,
    Cpa32U policy,
    Cpa32U priority)
{
    /*no such thing in kernel space, we provide this function so that our code
     * builds in kernel space and user space*/
    return CPA_STATUS_SUCCESS;
}

CpaStatus sampleCodeThreadJoin(sample_code_thread_t *threadPtr)
{
    struct task_struct *task = NULL;
    task = (struct task_struct*)*threadPtr;
    while(EXIT_DEAD != task->exit_state)
    {
        yield();
    }
    return CPA_STATUS_SUCCESS;
}



/*****************************
 *
 *  Time
 *
 *****************************/

/* Retrieve current system time */
CpaStatus sampleCodeTimeTGet (sample_code_time_t* pTime)
{
    /*
     * linux struct timeval has subfields:
     * -- time_t   (type long, second)
     * -- suseconds_t ( type long, usecond)
     */
    struct timeval _pTime;
    do_gettimeofday (&_pTime);
    /*
     * Translate microsecond to nanosecond,
     * second field is identical so no translation
     * there.
     */
    pTime->secs = _pTime.tv_sec;
    pTime->nsecs = (_pTime.tv_usec * NUM_MILLISEC_IN_SEC);
    return CPA_STATUS_SUCCESS;
}


CpaStatus sampleCodeSemaphoreInit (
        sample_code_semaphore_t* semPtr,
        Cpa32U start_value)
{
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(semPtr);
    *semPtr = kmalloc (sizeof (struct semaphore), GFP_KERNEL);
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(*semPtr);
    sema_init (*semPtr, start_value);
    return CPA_STATUS_SUCCESS;
}

/**
 * DESCRIPTION: If the semaphore is unset, the calling thread is blocked.
 *         If the semaphore is set, it is taken and control is returned
 *         to the caller. If the time indicated in 'timeout' is reached,
 *         the thread will unblock and return an error indication. If the
 *         timeout is set to 'WAIT_NONE', the thread will never block;
 *         if it is set to 'WAIT_FOREVER', the thread will block until
 *         the semaphore is available.
 *
 *
 */



/**
 *
 * DESCRIPTION: This function causes the next available thread in the pend queue
 *              to be unblocked. If no thread is pending on this semaphore, the
 *              semaphore becomes 'full'.
 */
CpaStatus sampleCodeSemaphorePost (sample_code_semaphore_t * semPtr)
{
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(semPtr);

    up (*semPtr);
    return CPA_STATUS_SUCCESS;
}

CpaStatus sampleCodeSemaphoreDestroy (sample_code_semaphore_t * semPtr)
{
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(semPtr);

    kfree (*semPtr);
    *semPtr = NULL;

    return CPA_STATUS_SUCCESS;
}

CpaStatus sampleCodeSemaphoreWait(
              sample_code_semaphore_t *semPtr,
              Cpa32S           timeout)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    unsigned long timeoutTime;
    CHECK_POINTER_AND_RETURN_FAIL_IF_NULL(semPtr);

    /*
     * Guard against illegal timeout values
     */
    if ((timeout < 0) && (timeout != SAMPLE_CODE_WAIT_FOREVER))
    {
        PRINT_ERR("illegal timeout value\n");
        return CPA_STATUS_FAIL;
    }
    else if (timeout > SAMPLE_CODE_MAX_TIMEOUT_MS)
    {
        PRINT_ERR("use smaller timeout value to avoid overflow \n");
        return CPA_STATUS_FAIL;
    }

    if (timeout == SAMPLE_CODE_WAIT_FOREVER)
    {
        if (down_interruptible(*semPtr) < 0)
        {
            return CPA_STATUS_FAIL;
        }
    }
    else if (timeout == SAMPLE_CODE_WAIT_NONE)
    {
        if (down_trylock (*semPtr))
        {
            status = CPA_STATUS_FAIL;
        }
    }
    else
    {
        /* Convert timeout in milliseconds to HZ */
        timeoutTime = jiffies + ((Cpa32U)timeout * HZ) /NUM_MILLISEC_IN_SEC;
        while (1)
        {
            if (!down_trylock (*semPtr))
            {
                break;
            }
            else
            {
                if (time_after(jiffies, timeoutTime))
                {
                    status = CPA_STATUS_FAIL;
                    break;
                }
            }
            /* Switch to next running process instantly */
            set_current_state((long)TASK_INTERRUPTIBLE);
            schedule_timeout(1);

        }  /* End of while loop */
    }      /* End of if */
    return status;

} /* sample_code_semaphoreWaitInterruptible */

/*This is a barrier function that all performance threads_g need to call after
 * setting up sessions and population of bufferList, but prior to the "Do Work"
 * functions, this allows all threads_g to be ready and start "Work" all at the
 * same time*/
void sampleCodeBarrier(void)
{

    atomic_inc(&arrived);
    if(atomic_read(&arrived)<numCreatedThreads_g)
    {
        while(atomic_read(&bGo)==0)
        {
            yield();
        }
    }
    else
    {
        atomic_inc(&bGo);
    }
}

/*reset so that the barrier can be re-used*/
void sampleCodeBarrierInit(void)
{
    atomic_set(&bGo, 0);
    atomic_set(&arrived, 0);
}

void startBarrier(void)
{
    /*this is not needed in kernel space, we can control start of threads via
     * sampleCodeStartThreads function*/
    return;
}
void startBarrierInit(void)
{
    /*this is not needed in kernel space, we can control start of threads via
     * sampleCodeStartThreads function*/
    return;
}

void sampleCodeBarrierDestroy(void)
{
    /*this is not needed in kernel space, we can control start of threads via
     * sampleCodeStartThreads function*/
    return;

}


#ifdef INCLUDE_COMPRESSION
CpaStatus getCorpusFile(Cpa8U **ppSrcBuff, char *filename, Cpa32U *size)
{

    const struct firmware *firmware_p = NULL;

#ifdef DC_DEBUG
    PRINT("***********************\n");
    PRINT("Entered getCorpus File API\n");
#endif
   if(request_firmware(&firmware_p, filename, &perf_device) != 0)
    {
        PRINT_ERR("%s file not available\n", filename);
        return CPA_STATUS_FAIL;
    }
    if(firmware_p == NULL)
    {
        PRINT_ERR("firmware_p is NULL\n");
        return CPA_STATUS_FAIL;
    }
    *size = firmware_p->size;
    *ppSrcBuff = (Cpa8U *)qaeMemAlloc(*size );
#ifdef DC_DEBUG
    PRINT("%s: allocation :: %p\n", __FUNCTION__, *ppSrcBuff);
#endif
    if(*ppSrcBuff == NULL)
    {
        PRINT("VMALLOC Failed\n");
        return CPA_STATUS_FAIL;
    }
    memcpy(*ppSrcBuff, firmware_p->data, firmware_p->size);
    release_firmware(firmware_p);

#ifdef DC_DEBUG
    PRINT("Exit from  getCorpus File API\n");
    PRINT("***********************\n");
#endif
    return CPA_STATUS_SUCCESS;
}

CpaStatus getCompressedFile(Cpa8U **ppSrcBuff, char *filename, Cpa32U *size)
{

    const struct firmware *firmware_p = NULL;
    char* compressedDataPtr = NULL;

#ifdef DC_DEBUG
    PRINT("***********************\n");
    PRINT("Entered getCorpus File API\n");
#endif
   if(request_firmware(&firmware_p, filename, &perf_device) != 0)
    {
        PRINT_ERR("%s file not available\n", filename);
        return CPA_STATUS_FAIL;
    }
    if(firmware_p == NULL)
    {
        PRINT_ERR("firmware_p is NULL\n");
        return CPA_STATUS_FAIL;
    }
    *size = firmware_p->size;
    *ppSrcBuff = (Cpa8U *)qaeMemAlloc(*size );
#ifdef DC_DEBUG
    PRINT("%s: allocation :: %p\n", __FUNCTION__, *ppSrcBuff);
#endif
    if(*ppSrcBuff == NULL)
    {
        PRINT("VMALLOC Failed\n");
        return CPA_STATUS_FAIL;
    }
    /*skip over the zlib header*/
    compressedDataPtr = (char *)firmware_p->data + ZLIB_HEADER_SIZE;
    memcpy(*ppSrcBuff, compressedDataPtr, firmware_p->size - ZLIB_HEADER_SIZE);
    release_firmware(firmware_p);

#ifdef DC_DEBUG
    PRINT("Exit from  getCorpus File API\n");
    PRINT("***********************\n");
#endif
    return CPA_STATUS_SUCCESS;
}
#endif

