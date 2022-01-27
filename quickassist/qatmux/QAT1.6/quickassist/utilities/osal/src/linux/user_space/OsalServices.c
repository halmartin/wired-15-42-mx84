/**
 * @file OsalServices.c (user space)
 *
 * @brief Implementation for Mem, Sleep and Log.
 *
 *
 * @par
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
 *  version: QAT1.6.L.2.6.0-65
 */

#include <stdarg.h>
#include <stdio.h>
#ifdef __linux__
#include <bits/time.h>
#endif
#include <sched.h>
#include <time.h>
#include <sys/utsname.h>
#include <sys/time.h>
#include <errno.h>
#include "Osal.h"

#ifdef __FreeBSD__
/* For sysctlbyname() usage in osalSysClockRateGet(). */
#include <sys/types.h>
#include <sys/sysctl.h>
#endif

#ifdef __FreeBSD__
#define OSAL_IOFILE FILE
#else
#define OSAL_IOFILE struct _IO_FILE
#endif

#define OSAL_TIMEVAL_TOO_LARGE       0

/*********************
 * Log function
 *********************/
static char *traceHeaders[] = {
    "",
    "[fatal] ",
    "[error] ",
    "[warning] ",
    "[message] ",
    "[debug1] ",
    "[debug2] ",
    "[debug3] ",
    "[all] "
};

static CHAR osalModuleName[OSAL_MAX_MODULE_NAME_LENGTH] = "";

OSAL_PRIVATE UINT32 max_UINT32 = 0xFFFFFFFF;

/* by default trace all but debug message */
OSAL_PRIVATE int osalCurrLogLevel = OSAL_LOG_LVL_MESSAGE;

#ifdef ICP_LOG_SYSLOG
OSAL_PRIVATE int osalCurrOutput = OSAL_LOG_OUTPUT_SYSLOG;
#else
OSAL_PRIVATE int osalCurrOutput = OSAL_LOG_OUTPUT_STD;
#endif

/*Writing Log*/
INT32
osalLog (OsalLogLevel level,
    OsalLogDevice device,
    char *format, INT32 arg1, INT32 arg2, INT32 arg3, INT32 arg4,
    INT32 arg5, INT32 arg6, INT32 arg7, INT32 arg8)
{

    OSAL_IOFILE *output_stream_s;
    int mask = 0;
    /*
     * Return -1 for custom display devices
     */
    switch(device)
    {
        case OSAL_LOG_DEV_STDOUT:
            output_stream_s = (OSAL_IOFILE *)stdout;
            break;
        case OSAL_LOG_DEV_STDERR:
            output_stream_s = (OSAL_IOFILE *)stderr;
            break;
        default:
            printf
                ("osalLog: only OSAL_LOG_DEV_STDOUT"
                  " and OSAL_LOG_DEV_STDERR are supported \n");
            return (OSAL_LOG_ERROR);
    }

    if ((level <= osalCurrLogLevel) && (level != OSAL_LOG_LVL_NONE))
    {
        if(OSAL_LOG_OUTPUT_SYSLOG == osalCurrOutput)
        {
            mask = setlogmask(0);
            mask = setlogmask(mask | LOG_MASK(LOG_INFO));
            if(level != OSAL_LOG_LVL_USER)
            {
                syslog(LOG_INFO, "%s", traceHeaders[level - 1]);
            }
            if ( OSAL_OS_GET_STRING_LENGTH(osalModuleName) != 0 )
            {
                syslog(LOG_INFO, "%s :", osalModuleName);
            }

            syslog(LOG_INFO, format,
                            arg1, arg2, arg3, arg4, arg5, arg6,
                            arg7, arg8);
            return 0;
        }
        int headerByteCount =
            (level == OSAL_LOG_LVL_USER) ?
             0 : fprintf ((FILE *)output_stream_s, "%s", traceHeaders[level - 1]);

        if ( OSAL_OS_GET_STRING_LENGTH(osalModuleName) != 0 )
        {
            headerByteCount +=
               fprintf((FILE *)output_stream_s, "%s :", osalModuleName);
        }

         headerByteCount +=
               fprintf ((FILE *)output_stream_s, format,
                        arg1, arg2, arg3, arg4, arg5, arg6,
                        arg7, arg8);
         return (INT32)headerByteCount;
    }
    else
    {
        return (OSAL_LOG_ERROR);
    }
}

INT32
osalLog64 (OsalLogLevel level,
    OsalLogDevice device,
    char *format, INT64 arg1, INT64 arg2, INT64 arg3, INT64 arg4,
    INT64 arg5, INT64 arg6, INT64 arg7, INT64 arg8)
{
    OSAL_IOFILE *output_stream_s;
    int mask = 0;
    /*
     * Return -1 for custom display devices
     */
    switch(device)
    {
        case OSAL_LOG_DEV_STDOUT:
            output_stream_s = (OSAL_IOFILE *)stdout;
            break;
        case OSAL_LOG_DEV_STDERR:
            output_stream_s = (OSAL_IOFILE *)stderr;
            break;
        default:
            printf
                ("osalLog: only OSAL_LOG_DEV_STDOUT"
                  " and OSAL_LOG_DEV_STDERR are supported \n");
            return (OSAL_LOG_ERROR);
    }

    if ((level <= osalCurrLogLevel) && (level != OSAL_LOG_LVL_NONE))
    {
        if(OSAL_LOG_OUTPUT_SYSLOG == osalCurrOutput)
        {
            mask = setlogmask(0);
            mask = setlogmask(mask | LOG_MASK(LOG_INFO));
            if(level != OSAL_LOG_LVL_USER)
            {
                syslog(LOG_INFO, "%s", traceHeaders[level - 1]);
            }
            if ( OSAL_OS_GET_STRING_LENGTH(osalModuleName) != 0 )
            {
                syslog(LOG_INFO, "%s :", osalModuleName);
            }

            syslog(LOG_INFO, format,
                            arg1, arg2, arg3, arg4, arg5, arg6,
                            arg7, arg8);
            return 0;
        }

        int headerByteCount =
            (level == OSAL_LOG_LVL_USER) ?
             0 : fprintf ((FILE *)output_stream_s, "%s", traceHeaders[level - 1]);

        if ( OSAL_OS_GET_STRING_LENGTH(osalModuleName) != 0 )
        {
            headerByteCount +=
               fprintf((FILE *)output_stream_s, "%s :", osalModuleName);
        }

        headerByteCount +=
              fprintf ((FILE *)output_stream_s, format,
                        arg1, arg2, arg3, arg4, arg5, arg6,
                        arg7, arg8);
         return (INT32)headerByteCount;
    }
    else
    {
        return (OSAL_LOG_ERROR);
    }
}

INT32
osalLogString (OsalLogLevel level,
    OsalLogDevice device, char *format, char* arg1, char* arg2,
    char* arg3, char* arg4, char* arg5, char* arg6)
{
    OSAL_IOFILE *output_stream_s;
    int mask = 0;
    /*
     * Return -1 for custom display devices
     */
    switch(device)
    {
        case OSAL_LOG_DEV_STDOUT:
            output_stream_s = (OSAL_IOFILE *)stdout;
            break;
        case OSAL_LOG_DEV_STDERR:
            output_stream_s = (OSAL_IOFILE *)stderr;
            break;
        default:
            printf
                ("osalLog: only OSAL_LOG_DEV_STDOUT"
                  " and OSAL_LOG_DEV_STDERR are supported \n");
            return (OSAL_LOG_ERROR);
    }

    if ((level <= osalCurrLogLevel) && (level != OSAL_LOG_LVL_NONE))
    {
        if(OSAL_LOG_OUTPUT_SYSLOG == osalCurrOutput)
        {
            mask = setlogmask(0);
            mask = setlogmask(mask | LOG_MASK(LOG_INFO));
            if(level != OSAL_LOG_LVL_USER)
            {
                syslog(LOG_INFO, "%s", traceHeaders[level - 1]);
            }
            if ( OSAL_OS_GET_STRING_LENGTH(osalModuleName) != 0 )
            {
                syslog(LOG_INFO, "%s :", osalModuleName);
            }

            syslog(LOG_INFO, format,
                            arg1, arg2, arg3, arg4, arg5,arg6);
            return 0;
        }
        int headerByteCount =
            (level == OSAL_LOG_LVL_USER) ?
             0 : fprintf ((FILE *)output_stream_s, "%s", traceHeaders[level - 1]);

        if ( OSAL_OS_GET_STRING_LENGTH(osalModuleName) != 0 )
        {
            headerByteCount +=
               fprintf((FILE *)output_stream_s, "%s :", osalModuleName);
        }

         headerByteCount +=
               fprintf ((FILE *)output_stream_s, format,
                        arg1, arg2, arg3, arg4, arg5,arg6);
         return (INT32)headerByteCount;
    }
    else
    {
        return (OSAL_LOG_ERROR);
    }
}

OSAL_PUBLIC OSAL_STATUS
osalStdLog(const char* arg_pFmtString, ...)
{
    OSAL_STATUS err = OSAL_SUCCESS;
    va_list argList;

    va_start(argList, arg_pFmtString);
    if ( OSAL_OS_GET_STRING_LENGTH(((void *) osalModuleName)) != 0 )
    {
        printf("%s :",osalModuleName);
    }

    vprintf(arg_pFmtString, argList);
    va_end(argList);

    return err;
}

/* Returns the old level that got overwritten */
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
            "osalLogLevelSet: Log Level is between %d and %d \n",
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
osalMemAlloc (UINT32 size)
{
    return (malloc (size));
}

void *
osalMemAllocAtomic (UINT32 size)
{
    return (malloc (size));
}

/*
 * MemAlloc with Alignment API:
 * Make sure memory is aligned to alignment ( passed as last parameter)
 * This function is maintained for backward compatibiltiy
 * IDS team has requested not to touch this code, as this is in working condition.
 * another implementaion of this function can be found in API
 * "void * _OsalMemAllocAlligned(unsigned int size, unsigned int alignment)"
 *
 * return - Pointer to requested alligned memory.
 * in - space  (unknown....why?)
      - size need to malloc
 *    - allignement required ( 4, 8....256)
 *
 */
void *
osalMemAllocAligned(UINT32 space, UINT32 size, UINT32 alignment)
{
    void *pMem = NULL;

    if (alignment < 1 || alignment > OSAL_MAX_ALIGNMENT)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT, \
            "osalMemAllocAligned(): invalid alignment value %d \n",
            alignment, 0, 0, 0, 0, 0, 0, 0);
        return (void *)NULL;
    }

    if (alignment == 1)
    {
        pMem = osalMemAlloc(size);
        return pMem;
    }

    if (posix_memalign(&pMem, alignment, size) != 0)
    {
        osalLog(OSAL_LOG_LVL_ERROR,OSAL_LOG_DEV_STDOUT,
            "osalMemAllocAlligned(): not able to align memory\n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return (void *)NULL;
    }
    return pMem;
}

void
osalMemFree (void *ptr)
{
    OSAL_MEM_ASSERT (ptr != NULL);
    free (ptr);
}

/*
 * Copy count bytes from src to dest ,
 * returns pointer to the dest mem zone.
 */
void *
osalMemCopy (void *dest, const void *src, UINT32 count)
{
    OSAL_MEM_ASSERT (dest != NULL);
    OSAL_MEM_ASSERT (src != NULL);
    return (memcpy (dest, src, count));
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

OSAL_PUBLIC void
osalMemAlignedFree(void *ptr)
{
    osalMemFree(ptr);
}

/**********************************************
 * Time module
 **********************************************/
/*
 *  Retrieve current system time.
 */
OSAL_PUBLIC OSAL_STATUS
osalTimeGet (OsalTimeval * ptime)
{
    struct timeval tval;

    if (gettimeofday(&tval,NULL) == -1)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "osalTimeGet(): gettimeofday system call failed \n",
            0, 0, 0, 0, 0, 0, 0, 0);

        return OSAL_FAIL;
    }

     ptime->secs = tval.tv_sec;

    /*
     * gettimeofday returns in terms of sec and uSec.
     * Convert it into sec and nanoseconds into OSAL type
     */
    ptime->nsecs = tval.tv_usec * OSAL_THOUSAND;

    return OSAL_SUCCESS;
}

OSAL_PUBLIC UINT64
osalTimestampGet(void)
{
    OsalTimeval ptime;
    osalTimeGet(&ptime);
    return ((ptime.secs * OSAL_MILLION) + ptime.nsecs/OSAL_THOUSAND);

}

#ifdef __FreeBSD__
OSAL_PUBLIC UINT32
osalSysClockRateGet( void )
{
    int rate = 0;
    size_t rate_size = sizeof(rate);

    if (sysctlbyname("kern.hz", &rate, &rate_size, NULL, 0))
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT, \
            "osalSysClockRateGet(): could not read kern.hz sysctl\n", \
            0, 0, 0, 0, 0, 0, 0, 0);
        return 0;
    }

    return (UINT32)rate;
}
#else
OSAL_PUBLIC UINT32
osalSysClockRateGet( void )
{
            return (HZ);
}
#endif

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

OSAL_PUBLIC UINT32
osalTimevalToTicks (OsalTimeval tv)
{
    UINT32 tickPerSecs = 0;
    UINT32 nanoSecsPerTick = 0;
    UINT32 maxSecs = 0;

    tickPerSecs = osalSysClockRateGet ();
    nanoSecsPerTick = OSAL_BILLION / tickPerSecs;

    /*
     * Make sure we do not overflow
     */
    maxSecs = (max_UINT32 / tickPerSecs) - (tv.nsecs / OSAL_BILLION);

    if ( maxSecs < tv.secs )
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "osalTimevalToTicks(): Timeval too high. Max value allowed"
            "in seconds is %u < %u\n", maxSecs, tv.secs,
            0, 0, 0, 0, 0, 0);
        return OSAL_TIMEVAL_TOO_LARGE;

       }

    return ((tv.secs * tickPerSecs) + (tv.nsecs / nanoSecsPerTick));
}



/**************************************
 * Task services module
 *************************************/
/*
 * Sleep for the specified number of milliseconds
 */
OSAL_PUBLIC OSAL_STATUS
osalSleep (UINT32 milliseconds)
{
    struct timespec resTime,remTime;
    INT32 status;
    UINT16 mil_rem;

    /* Divide by number of millisec per second */
    resTime.tv_sec = milliseconds / OSAL_THOUSAND;
    /* Multiply remainder by number of nanosecs per millisecond */
    mil_rem = milliseconds % OSAL_THOUSAND;
    resTime.tv_nsec =  mil_rem * OSAL_MILLION;

    do
    {
        status = nanosleep(&resTime,&remTime);
        resTime.tv_sec = remTime.tv_sec;
        resTime.tv_nsec = remTime.tv_nsec;
     }while((status != OSAL_SUCCESS)&&(errno == EINTR));

    if (status != OSAL_SUCCESS)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT, \
            "osalSleep():nanosleep() failed; errno=%d\n", \
            errno, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }else{
        return OSAL_SUCCESS;
    }
}

OSAL_PUBLIC void
osalYield (void)
{
    sched_yield();
}

