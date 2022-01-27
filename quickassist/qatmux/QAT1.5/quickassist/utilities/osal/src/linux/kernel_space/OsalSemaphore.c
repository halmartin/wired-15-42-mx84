/**
 * @file OsalSemaphore.c (linux)
 *
 * @brief Implementation for semaphore and pMutex.
 *
 *
 * @par
 * GPL LICENSE SUMMARY
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
 *  version: QAT1.5.L.1.11.0-36
 */
#include "Osal.h"
#include "OsalTypes.h"

#include <linux/slab.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
#include <asm/semaphore.h>
#else
#include <linux/semaphore.h>
#endif

#include <asm/atomic.h>
#include <linux/hardirq.h>


/* Define a 64 bit number */
#define OSAL_MAX_LONG (0x7FFFFFFF)

/* Max timeout in MS, used to guard against possible overflow */
#define OSAL_MAX_TIMEOUT_MS (OSAL_MAX_LONG/HZ)

OSAL_PUBLIC OSAL_STATUS
osalSemaphoreInit (OsalSemaphore * pSid, UINT32 start_value)
{
    OSAL_LOCAL_ENSURE(pSid,
                      "OsalSemaphoreInit(): NULL semaphore pointer",
                       OSAL_FAIL);

    *pSid = kmalloc (sizeof (struct semaphore), GFP_KERNEL);
    if (NULL == *pSid)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "OsalSemaphoreInit(): fail to allocate memory for semaphore\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    sema_init (*pSid, start_value);

    return OSAL_SUCCESS;
}

/**
 * DESCRIPTION: If the semaphore is unset, the calling thread is blocked.
 *         If the semaphore is set, it is taken and control is returned
 *         to the caller. If the time indicated in 'timeout' is reached,
 *         the thread will unblock and return an error indication. If the
 *         timeout is set to 'OSAL_WAIT_NONE', the thread will never block;
 *         if it is set to 'OSAL_WAIT_FOREVER', the thread will block until
 *         the semaphore is available.
 *
 *
 */
OSAL_PUBLIC OSAL_STATUS
osalSemaphoreWait (OsalSemaphore * pSid, INT32 timeout)
{

    OSAL_STATUS status = OSAL_SUCCESS;
    unsigned long timeoutTime;

    OSAL_LOCAL_ENSURE(pSid,
                      "OsalSemaphoreWait(): NULL semaphore pointer",
                       OSAL_FAIL);

    /*
     * Guard against illegal timeout values
     */
    if ((timeout < 0) && (timeout != OSAL_WAIT_FOREVER))
    {
        osalLog (OSAL_LOG_LVL_ERROR,
            OSAL_LOG_DEV_STDOUT,
            "OsalSemaphoreWait(): illegal timeout value \n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
    else if (timeout > OSAL_MAX_TIMEOUT_MS)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "OsalSemaphoreWait(): use a smaller timeout value to avoid \
            overflow \n", 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    if (timeout == OSAL_WAIT_FOREVER)
    {
        down (*pSid);
    }
    else if (timeout == OSAL_WAIT_NONE)
    {
        if (down_trylock (*pSid))
        {
            status = OSAL_FAIL;
        }
    }
    else
    {
        /* Convert timeout in milliseconds to HZ */
        timeoutTime = jiffies + (timeout * HZ) /OSAL_THOUSAND;
        while (1)
        {
            if (!down_trylock (*pSid))
            {
                break;
            }
            else
            {
                if (time_after(jiffies, timeoutTime))
                {
                    status = OSAL_FAIL;
                    break;
                }
            }
            /* Switch to next running process instantly */
            set_current_state((long)TASK_INTERRUPTIBLE);
            schedule_timeout(1);

        }  /* End of while loop */
    }      /* End of if */

    return status;

}

OSAL_PUBLIC OSAL_STATUS
osalSemaphoreTryWait (OsalSemaphore * pSid)
 {
     OSAL_LOCAL_ENSURE(pSid,
                      "OsalSemaphoreTryWait(): NULL semaphore pointer",
                       OSAL_FAIL);

     if (down_trylock (*pSid))
        {
            return OSAL_FAIL;
       }
       return OSAL_SUCCESS;
}

/**
 *
 * DESCRIPTION: This function causes the next available thread in the pend queue
 *              to be unblocked. If no thread is pending on this semaphore, the
 *              semaphore becomes 'full'.
 */
OSAL_PUBLIC OSAL_STATUS
osalSemaphorePost (OsalSemaphore * pSid)
{
    OSAL_LOCAL_ENSURE(pSid,
                      "OsalSemaphorePost(): NULL semaphore pointer",
                       OSAL_FAIL);

    up (*pSid);
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalSemaphoreDestroy (OsalSemaphore * pSid)
{
    OSAL_LOCAL_ENSURE(pSid,
                      "OsalSemaphoreDestroy(): NULL semaphore pointer",
                       OSAL_FAIL);

    kfree (*pSid);
    *pSid = NULL;

    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalSemaphoreWaitInterruptible(
              OsalSemaphore *pSid,
              INT32           timeout)
{
    OSAL_STATUS status = OSAL_SUCCESS;
    unsigned long timeoutTime;

    OSAL_LOCAL_ENSURE(pSid,
                      "OsalSemaphoreWaitInterruptible(): NULL semaphore pointer",
                       OSAL_FAIL);

    /*
     * Guard against illegal timeout values
     */
    if ((timeout < 0) && (timeout != OSAL_WAIT_FOREVER))
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "OsalSemaphoreWaitInterruptible: illegal timeout value\n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
    else if (timeout > OSAL_MAX_TIMEOUT_MS)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "OsalSemaphoreWaitInterruptible():use smaller timeout \
             value to avoid overflow \n", 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    if (timeout == OSAL_WAIT_FOREVER)
    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
        down_interruptible(*pSid);
#else
        if (down_interruptible(*pSid) < 0)
        {
            return OSAL_FAIL;
        }
#endif
    }
    else if (timeout == OSAL_WAIT_NONE)
    {
        if (down_trylock (*pSid))
        {
            status = OSAL_FAIL;
        }
    }
    else
    {
        /* Convert timeout in milliseconds to HZ */
        timeoutTime = jiffies + (timeout * HZ) /OSAL_THOUSAND;
        while (1)
        {
            if (!down_trylock (*pSid))
            {
                break;
            }
            else
            {
                if (time_after(jiffies, timeoutTime))
                {
                    status = OSAL_FAIL;
                    break;
                }
            }
            /* Switch to next running process instantly */
            set_current_state((long)TASK_INTERRUPTIBLE);
            schedule_timeout(1);

        }  /* End of while loop */
    }      /* End of if */

    return status;

} /* OsalSemaphoreWaitInterruptible */

OSAL_PUBLIC OSAL_STATUS
osalSemaphorePostWakeup(OsalSemaphore *pSid)
{
    OSAL_LOCAL_ENSURE(pSid,
                      "OsalSemaphoreInit(): NULL semaphore handle",
                       OSAL_FAIL);

    OSAL_LOCAL_ENSURE(*pSid,
                      "OsalSemaphoreInit(): NULL semaphore pointer",
                       OSAL_FAIL);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
    if( atomic_read(&((*pSid)->count)) < 1)
    {
        up(*pSid);
    }
    wake_up(&((*pSid)->wait));
#else
    if( (*pSid)->count < 1)
     {
        up(*pSid);
    }
#endif
    return OSAL_SUCCESS;

}/* OsalSemaphorePostWakeup */

OSAL_PUBLIC OSAL_STATUS
osalSemaphoreGetValue (OsalSemaphore * sid, UINT32 * value)
{
    if (sid == NULL)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                    "ixOsalSemaphoreGetValue(): NULL semaphore handle \n",
                    0, 0, 0, 0, 0, 0, 0, 0);
         return OSAL_FAIL;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
      *value = atomic_read(&((*sid)->count));
#else
    *value = (*sid)->count;
#endif
    return OSAL_SUCCESS;
}


/****************************
 *    Mutex
 ****************************/

OSAL_PUBLIC OSAL_STATUS
osalMutexInit (OsalMutex * pMutex)
{
    OSAL_LOCAL_ENSURE(pMutex,
                      "OsalMutexInit(): NULL mutex handle",
                       OSAL_FAIL);

    *pMutex =(struct semaphore *) kmalloc(sizeof(struct semaphore), GFP_KERNEL);
    if (NULL == *pMutex)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                 "OsalMutexInit(): Fail to allocate memory for pMutex\n",
                 0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    sema_init (*pMutex, 1);
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalMutexLock (OsalMutex * pMutex, INT32 timeout)
{
    unsigned long timeoutTime;

    OSAL_LOCAL_ENSURE(pMutex,
                      "OsalMutexLock(): NULL semaphore handle",
                       OSAL_FAIL);
    OSAL_LOCAL_ENSURE(*pMutex,
                       "OsalMutexLock(): NULL mutex pointer",
                       OSAL_FAIL);

    if ((timeout < 0) && (timeout != OSAL_WAIT_FOREVER))
    {
        osalLog (OSAL_LOG_LVL_ERROR,
            OSAL_LOG_DEV_STDOUT,
            "OsalMutexLock(): Illegal timeout value \n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }
    else if (timeout > OSAL_MAX_TIMEOUT_MS)
    {
        osalLog (OSAL_LOG_LVL_ERROR,
           OSAL_LOG_DEV_STDOUT,
           "OsalMutexLock(): use smaller timeout value to avoid overflow \n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    if (timeout == OSAL_WAIT_FOREVER)
    {
        down (*pMutex);
    }
    else if (timeout == OSAL_WAIT_NONE)
    {
        if (down_trylock (*pMutex))
        {
            return OSAL_FAIL;
        }
    }
   else
    {
        timeoutTime = jiffies + (timeout * HZ) / OSAL_THOUSAND;
        while (1)
        {
            if (!down_trylock (*pMutex))
            {
                break;
            }
            else
            {
                if (time_after(jiffies, timeoutTime))
                {
                    return OSAL_FAIL;
                }
            }

           /* Switch to next running process if not in atomic state  */
           if (!in_atomic())
           {
                set_current_state((long)TASK_INTERRUPTIBLE);
                schedule_timeout(1);
           }
        }       /* End of while loop */
    }           /* End of if */
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalMutexUnlock (OsalMutex * pMutex)
{
    OSAL_LOCAL_ENSURE(pMutex,
                      "OsalMutexUnlock(): NULL mutex handle",
                       OSAL_FAIL);

    OSAL_LOCAL_ENSURE(*pMutex,
                      "OsalMutexUnlock(): NULL mutex pointer",
                       OSAL_FAIL);

    up (*pMutex);
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalMutexDestroy (OsalMutex * pMutex)
{
    OSAL_LOCAL_ENSURE(pMutex,
                      "OsalMutexDestroy(): NULL mutex handle",
                       OSAL_FAIL);

    OSAL_LOCAL_ENSURE(*pMutex,
                      "OsalMutexDestroy(): NULL mutex handle",
                       OSAL_FAIL);

    kfree (*pMutex);
    *pMutex = NULL;

    return OSAL_SUCCESS;
}

