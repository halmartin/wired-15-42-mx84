/**
 * @file kernel_space/OsalServices.c (linux)
 *
 * @brief Implementation for Mem and Sleep.
 *
 *
 * @par
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
 */
#include "Osal.h"
#include "OsalOsTypes.h"
#include "OsalDevDrv.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/mempool.h>
#include <linux/vmalloc.h>
#include <linux/highmem.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <linux/random.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,0))
#include <linux/time64.h>
#endif

/* Trace Message Logging Levels */

static char *traceHeaders[] = {
    "",
    "[fatal] ",
    "[error] ",
    "[warn] ",
    "[message] ",
    "[debug1] ",
    "[debug2] ",
    "[debug3] ",
    "[all] "
};

static CHAR osalModuleName[OSAL_MAX_MODULE_NAME_LENGTH] = "";

/* by default trace all but debug message */
OSAL_PRIVATE UINT32 osalCurrLogLevel = OSAL_LOG_LVL_MESSAGE;

#define IS_VMALLOC_ADDR(addr) (((addr) >= (void*)VMALLOC_START) && \
        ((addr) < (void*)VMALLOC_END))

/* Maximum memory (in bytes) that can be allocated using kmalloc.
 *    Beyond this, vmalloc is to be used to allcoate memory */
#define OSAL_MAX_KMALLOC_MEM      (1024 * 128)

/*********************
 * Log function
 *********************/

INT32
osalLog(OsalLogLevel level, OsalLogDevice device, char *format, ...)
{
    va_list args;
    /*
     * Return -1 for custom display devices
     */
    if ((device != OSAL_LOG_DEV_STDOUT) && (device != OSAL_LOG_DEV_STDERR))
    {
        printk("osalLog: only OSAL_LOG_DEV_STDOUT and \
                       OSAL_LOG_DEV_STDERR are supported \n");
        return (OSAL_LOG_ERROR);
    }
    if (level <= osalCurrLogLevel && level != OSAL_LOG_LVL_NONE)
    {
        INT32 headerByteCount =
            (level == OSAL_LOG_LVL_USER)
            ? 0
            : printk("%s", traceHeaders[level - 1]);

        if (OSAL_OS_GET_STRING_LENGTH(((VOID *)osalModuleName)) != 0)
        {
            headerByteCount += printk("%s :", osalModuleName);
        }
        va_start(args, format);
        headerByteCount += vprintk(format, args);
        va_end(args);
        return headerByteCount;
    }
    else
    {
        /*
         * Return zero byte printed
         */
        return (OSAL_NO_LOG);
    }
}

INT32
osalLog64 (OsalLogLevel level,
    OsalLogDevice device,
    char *format, INT64 arg1, INT64 arg2, INT64 arg3, INT64 arg4, INT64 arg5,
    INT64 arg6, INT64 arg7, INT64 arg8)
{
    /*
     * Return -1 for custom display devices
     */
    if ((device != OSAL_LOG_DEV_STDOUT)
        && (device != OSAL_LOG_DEV_STDERR))
    {
        printk("osalLog: only OSAL_LOG_DEV_STDOUT and \
                       OSAL_LOG_DEV_STDERR are supported \n");
        return (OSAL_LOG_ERROR);
    }

    if (level <= osalCurrLogLevel && level != OSAL_LOG_LVL_NONE)
    {
        int headerByteCount =
            (level == OSAL_LOG_LVL_USER)
            ? 0
            : printk ("%s", traceHeaders[level - 1]);

        if (OSAL_OS_GET_STRING_LENGTH(((VOID *) osalModuleName)) != 0)
        {
            headerByteCount +=
                printk("%s :",osalModuleName);
        }
        headerByteCount +=
           printk (format,
           arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8);
        return  (INT32)headerByteCount;
    }
    else
    {
        /*
         * Return zero byte printed
         */
        return (OSAL_NO_LOG);
    }
}

INT32
osalLogString (OsalLogLevel level,
    OsalLogDevice device,
    char *format, char* arg1, char* arg2,
    char* arg3, char* arg4, char* arg5, char* arg6)
{
    /*
     * Return -1 for custom display devices
     */
    if ((device != OSAL_LOG_DEV_STDOUT)
        && (device != OSAL_LOG_DEV_STDERR))
    {
        printk("osalLog: only OSAL_LOG_DEV_STDOUT and \
                       OSAL_LOG_DEV_STDERR are supported \n");
        return (OSAL_LOG_ERROR);
    }

    if (level <= osalCurrLogLevel && level != OSAL_LOG_LVL_NONE)
    {
        int headerByteCount =
            (level == OSAL_LOG_LVL_USER)
            ? 0
            : printk ("%s", traceHeaders[level - 1]);

        if (OSAL_OS_GET_STRING_LENGTH(((VOID *) osalModuleName)) != 0)
        {
            headerByteCount +=
                printk("%s :",osalModuleName);
        }
        headerByteCount +=
           printk (format,
           arg1, arg2, arg3, arg4, arg5,arg6);
        return (INT32)headerByteCount;
    }
    else
    {
        /*
         * Return zero byte printed
         */
        return (OSAL_NO_LOG);
    }
}

OSAL_PUBLIC OSAL_STATUS
osalStdLog(const char* arg_pFmtString, ...)
{
    OSAL_STATUS err = OSAL_SUCCESS;
    va_list argList;

    va_start(argList, arg_pFmtString);
    if (OSAL_OS_GET_STRING_LENGTH(((VOID *) osalModuleName)) != 0)
    {
         printk("%s :",osalModuleName);
    }

    vprintk(arg_pFmtString, argList);
    va_end(argList);

   return err;

}

OSAL_PUBLIC UINT32
osalLogLevelSet (UINT32 level)
{
    UINT32 oldLevel;

    /*
     * Check value first
     */
    if (level > OSAL_LOG_LVL_ALL)
    {
        osalLog (OSAL_LOG_LVL_MESSAGE,
            OSAL_LOG_DEV_STDOUT,
            "osalLogLevelSet: Log Level is between %d and%d \n",
            OSAL_LOG_LVL_NONE, OSAL_LOG_LVL_ALL, 0, 0, 0, 0, 0, 0);
        return OSAL_LOG_LVL_NONE;
    }
    oldLevel = osalCurrLogLevel;
    osalCurrLogLevel = level;
    return oldLevel;
}

/**************************************
 * Memory functions
 *************************************/
void *
osalMemAlloc (UINT32 memsize)
{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23))
    return (kmalloc(memsize, GFP_KERNEL));
#endif
    if(memsize > OSAL_MAX_KMALLOC_MEM)
    {
        return (vmalloc(memsize));
    }
    return (kmalloc(memsize, GFP_KERNEL));
}

void *
osalMemAllocAtomic (UINT32 memsize)
{
    return (kmalloc (memsize, GFP_ATOMIC));
}

void *
osalMemAllocContiguousNUMA (UINT32 size, UINT32 node, UINT32 alignment)
{
    VOID* ptr = NULL;
    VOID* pRet = NULL;
    UINT32 alignment_offset = 0;
    UINT64 phy_ptr = 0;

    OsalMemAllocInfoStruct memInfo = {0};
    if (size == 0 || alignment < 1)
    {
        osalLog (OSAL_LOG_LVL_MESSAGE, OSAL_LOG_DEV_STDOUT,
        "[OsalMemAllocNUMA] size or alignment are zero \n",
            size, alignment, 0, 0, 0, 0, 0, 0);
        return NULL;
    }
    if (alignment & (alignment-1))
    {
         osalLog (OSAL_LOG_LVL_MESSAGE, OSAL_LOG_DEV_STDOUT,
        "[OsalMemAllocNUMA] Expecting alignment of a power "
         "of two but did not get one\n", 0, 0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }

    memInfo.mSize = osalIOMMUgetRemappingSize(size + alignment +
                                              sizeof(OsalMemAllocInfoStruct));
    ptr = kmalloc_node(memInfo.mSize, GFP_KERNEL, node);
    if (NULL == ptr)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "OsalMemAllocNUMA(): memory allocation failes\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }

    memInfo.mAllocMemPtr = ptr;
    pRet = (char *)memInfo.mAllocMemPtr + sizeof(OsalMemAllocInfoStruct);
#ifdef __x86_64__
    alignment_offset = (UINT64)pRet % alignment;
#else
    alignment_offset = (UINT32)pRet % alignment;
#endif
    pRet = (char *)pRet + (alignment - alignment_offset);
    memcpy((void*)(char *)pRet - sizeof(OsalMemAllocInfoStruct),
            (void *)&memInfo, sizeof(OsalMemAllocInfoStruct));

    phy_ptr = virt_to_phys(ptr);
    if(osalIOMMUMap(phy_ptr, phy_ptr, memInfo.mSize))
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "osalIOMMUMap(): failed\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        kfree(ptr);
        return NULL;
    }
    return pRet;
}

OSAL_PUBLIC void*
osalMemAllocPage(UINT32 node, UINT64 *physAddr)
{
    void* ptr = NULL;
    ptr = (void *)__get_free_page(GFP_KERNEL);
    *physAddr = virt_to_phys(ptr);
    return ptr;
}

OSAL_PUBLIC void
osalMemFreePage(void* pVirtAddress)
{
    if (NULL == pVirtAddress)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
          "Invalid virtual address\n", 0, 0, 0, 0, 0, 0, 0, 0);
          return;
    }
    free_page((unsigned long)pVirtAddress);
}

void
osalMemFreeNUMA(void* ptr)
{
    OsalMemAllocInfoStruct *memInfo = NULL;

    memInfo = (OsalMemAllocInfoStruct *)((CHAR *)ptr -
                                  sizeof(OsalMemAllocInfoStruct));
    if (memInfo->mSize == 0 || memInfo->mAllocMemPtr == NULL)
    {
        osalLog (OSAL_LOG_LVL_MESSAGE, OSAL_LOG_DEV_STDOUT,
        "[OsalMemAlignedFree] ERROR: Detected the corrupted "
        "data: memory leak!! \n", 0, 0, 0, 0, 0, 0, 0, 0);
        return;
    }
    osalIOMMUUnmap(virt_to_phys(memInfo->mAllocMemPtr),
                                       memInfo->mSize);
    kfree (memInfo->mAllocMemPtr);
}

UINT64
osalVirtToPhysNUMA(void* ptr)
{
    return (UINT64)virt_to_phys(ptr);
}

void
osalMemFree(void *ptr)
{
    OSAL_MEM_ASSERT (ptr != NULL);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23))
    kfree(ptr);
    return;
#endif
    if(IS_VMALLOC_ADDR(ptr))
    {
        vfree(ptr);
        return;
    }
    kfree(ptr);
    return;
}

/*
 * Copy count bytes from pSrc to pDest ,
 * returns pointer to the pDest mem zone.
 */
void *
osalMemCopy (void *pDest, const void *pSrc, UINT32 count)
{
    OSAL_MEM_ASSERT (pDest != NULL);
    OSAL_MEM_ASSERT (pSrc != NULL);
    return (memcpy (pDest, pSrc, count));
}

/*
 * Fills a memory zone with a given constant byte,
 * returns pointer to the memory zone.
 */
void *
osalMemSet (void *ptr, UINT8 filler, UINT32 count)
{
    OSAL_MEM_ASSERT (ptr != NULL);
    return (memset (ptr, filler, count));
}

/*****************************
 *
 *  Time
 *
 *****************************/

/* Retrieve current system time */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,0))
OSAL_STATUS
osalTimeGet (OsalTimeval * pTime)
{
    struct timespec64 _pTime;

    ktime_get_real_ts64 (&_pTime);

    pTime->secs = _pTime.tv_sec;
    pTime->nsecs = _pTime.tv_nsec;

    return OSAL_SUCCESS;
}
#else
OSAL_STATUS
osalTimeGet (OsalTimeval * pTime)
{
    /*
     * linux struct timeval has subfields:
     * -- time_t   (type long, second)
     * -- suseconds_t (type long, usecond)
     */
    struct timeval _pTime;

    do_gettimeofday (&_pTime);
    /*
     * Translate microsecond to nanosecond,
     * second field is identical so no translation
     * there.
     */
    pTime->secs = _pTime.tv_sec;
    pTime->nsecs = _pTime.tv_usec * OSAL_THOUSAND;
    return OSAL_SUCCESS;
}
#endif

OSAL_PUBLIC OSAL_STATUS
osalSleep (UINT32 milliseconds)
{
    signed long timeout = 0;

    if (milliseconds != 0)
    {
        set_current_state((long)TASK_INTERRUPTIBLE);
        timeout = schedule_timeout ((milliseconds * HZ) / OSAL_THOUSAND);
        if (timeout != 0)
        {
            return OSAL_FAIL;
        }
    }
    else
    {
        schedule();
    }
    return OSAL_SUCCESS;
}

OSAL_PUBLIC void
osalYield (void)
{
    schedule();
}

OSAL_PUBLIC UINT32
osalSysClockRateGet(void)
{
    return HZ;
}

OSAL_PUBLIC UINT64
osalTimestampGet (void)
{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4,2,8))
    /* Read the 64-bit time stamp counter */
    return (UINT64)rdtsc_ordered();
#else
    UINT64 timestamp;

    /* Read the 64-bit LSB of the time stamp counter */
    rdtscll(timestamp);
    return (timestamp);
#endif
}

OSAL_PUBLIC UINT32
osalTimevalToTicks (OsalTimeval tv)
{
    UINT32 tickPerSecs = 0;
    UINT32 nanoSecsPerTick = 0;
    UINT32 maxSecs = 0;
    UINT64 nsecs = tv.nsecs;

    tickPerSecs = osalSysClockRateGet ();
    nanoSecsPerTick = OSAL_BILLION / tickPerSecs;
    /*
     * Make sure we do not overflow
     */
    do_div(nsecs, OSAL_BILLION);
    maxSecs = (0xFFFFFFFF / tickPerSecs) - nsecs;
    if (maxSecs < tv.secs)
    {
        return 0;
    }
    nsecs = tv.nsecs;
    do_div(nsecs, nanoSecsPerTick);
    return ((tv.secs * tickPerSecs) + nsecs);
}

OSAL_PUBLIC void
osalTicksToTimeval (UINT32 ticks, OsalTimeval * pTv)
{
    UINT32 tickPerSecs = 0;
    UINT32 nanoSecsPerTick = 0;
    /*
     * Reset the time value
     */
    pTv->secs = 0;
    pTv->nsecs = 0;
    tickPerSecs = osalSysClockRateGet ();
    nanoSecsPerTick = OSAL_BILLION / tickPerSecs;
    /*
     * value less than 1 sec
     */
     if (tickPerSecs > ticks)    /* value less then 1 sec */
     {
        pTv->nsecs = ticks * nanoSecsPerTick;
     }
     else
     {
         pTv->secs = ticks / tickPerSecs;
          pTv->nsecs = (ticks % tickPerSecs) * nanoSecsPerTick;
     }
}

/*
 *  The function allocate a chunck of memory bigger than the requested size
 *  in order to perform the memory alignment.
 *  +---+-------------------------+------------------------------- +---+
 *  |xxx|OsalMemAllocInfoStruct | memory returned to user (size) |xxx|
 *  +---+-------------------------+--------------------------------+---+
 *  ^                             ^
 *  mAllocMemPtr                  Ptr returned to the caller of MemAlloc
 *
 */
OSAL_PUBLIC VOID *
osalMemAllocAligned(UINT32 space, UINT32 size, UINT32 alignment)
{
    VOID* ptr = NULL;
    VOID* pRet = NULL;
    UINT32 alignment_offset = 0;

    OsalMemAllocInfoStruct memInfo = {0};
    if (size == 0 || alignment < 1)
    {
        osalLog (OSAL_LOG_LVL_MESSAGE, OSAL_LOG_DEV_STDOUT,
        "[OsalMemAllocAligned] size or alignment are zero \n",
            size, alignment, 0, 0, 0, 0, 0, 0);
        return NULL;
    }
    if (alignment & (alignment-1))
    {
         osalLog (OSAL_LOG_LVL_MESSAGE, OSAL_LOG_DEV_STDOUT,
        "[OsalMemAllocAligned] Expecting alignment of a power "
         "of two but did not get one\n", 0, 0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }
    memInfo.mSize = size + alignment + sizeof(OsalMemAllocInfoStruct);
    ptr = kmalloc (memInfo.mSize, GFP_KERNEL);
    if (NULL == ptr)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "OsalMemAllocAligned(): memory allocation fail\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return NULL;
    }

    memInfo.mAllocMemPtr = ptr;
    pRet = (char *)memInfo.mAllocMemPtr +  sizeof(OsalMemAllocInfoStruct);

#ifdef __x86_64__
    alignment_offset = (UINT64)pRet % alignment;
#else
    alignment_offset = (UINT32)pRet % alignment;
#endif
    pRet= (char *)pRet +(alignment - alignment_offset);
    memcpy((void*)(char *)pRet - sizeof(OsalMemAllocInfoStruct), (void *)&memInfo,
            sizeof(OsalMemAllocInfoStruct));
    return pRet;
}

VOID
osalMemAlignedFree(VOID *ptr)
{
    OsalMemAllocInfoStruct *memInfo = NULL;

    memInfo = (OsalMemAllocInfoStruct *)((CHAR *)ptr -
                                  sizeof(OsalMemAllocInfoStruct));
    if (memInfo->mSize == 0 || memInfo->mAllocMemPtr == NULL)
    {
        osalLog (OSAL_LOG_LVL_MESSAGE, OSAL_LOG_DEV_STDOUT,
        "[OsalMemAlignedFree] ERROR: Detected the corrupted "
        "data: memory leak!! \n", 0, 0, 0, 0, 0, 0, 0, 0);
        return;
    }
    kfree (memInfo->mAllocMemPtr);
}
