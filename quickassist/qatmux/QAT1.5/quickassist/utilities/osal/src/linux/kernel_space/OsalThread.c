/**
 *
 * @file kernel_space/OsalThread.c
 *
 * @brief OS-specific Threads API's implementation.
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

#include <linux/version.h>
#include <linux/sched.h>

#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/hardirq.h>

/* declaring mutexes */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
DECLARE_MUTEX (osalThreadMutex);
DECLARE_MUTEX (osalThreadStopMutex);
#else
DEFINE_SEMAPHORE (osalThreadMutex);
DEFINE_SEMAPHORE (osalThreadStopMutex);
#endif

OSAL_PUBLIC OSAL_STATUS
osalThreadCreate (OsalThread * pTid,
    OsalThreadAttr * pThreadAttr, OsalVoidFnVoidPtr entryPoint, void *pArg)
{
    OSAL_LOCAL_ENSURE(pTid,
                      "OsalThreadCreate(): NULL thread pointer",
                      OSAL_FAIL);

    *pTid = kthread_create((void *)entryPoint, pArg, "%s",
                           (NULL != pThreadAttr && NULL != pThreadAttr->name) ? 
                           pThreadAttr->name : "OSAL");

    return OSAL_SUCCESS;
}

OSAL_PUBLIC void
osalThreadBind (OsalThread * pTid, UINT32 cpu)
{
    kthread_bind(*pTid, cpu);
    return;
}

OSAL_PUBLIC OSAL_STATUS
osalThreadStart (OsalThread *pTid)
{
    OSAL_LOCAL_ENSURE(pTid,
                        "OsalThreadStart(): NULL thread pointer",
                      OSAL_FAIL);

     OSAL_LOCAL_ENSURE(*pTid,
                        "OsalThreadStart(): NULL thread pointer",
                      OSAL_FAIL);

    wake_up_process(*pTid);
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalThreadKill (OsalThread * pTid)
{
    struct task_struct *task = NULL;

    OSAL_LOCAL_ENSURE(pTid,
                        "OsalThreadKill(): NULL thread pointer",
                      OSAL_FAIL);

    OSAL_LOCAL_ENSURE(*pTid,
                        "OsalThreadKill(): NULL thread pointer",
                      OSAL_FAIL);

    task = (struct task_struct*)*pTid;

    /* Can't kill already defunct thread */
    if (EXIT_DEAD == task->exit_state || EXIT_ZOMBIE == task->exit_state)
    {
         return OSAL_FAIL;
    }

    if (-EINTR == kthread_stop(task))
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "OsalThreadKill(): Failed to kill thread\n",
            0, 0, 0, 0, 0, 0, 0, 0);

        return OSAL_FAIL;
    }

    return OSAL_SUCCESS;
}

OSAL_PUBLIC void
osalThreadExit (void)
{
    osalLog (OSAL_LOG_LVL_MESSAGE, OSAL_LOG_DEV_STDOUT,
                  "OsalThreadExit(): not implemented in linux\n",
                  0, 0, 0, 0, 0, 0, 0, 0);
}

/********************************************************************
 * UINT32 priority - the value of priority can range from 0 to 39   *
 *                   with 0 being the highest priority.                *
 *                                                                     *
 * Any values with priority higher than 255 will be discared and a  *
 * OSAL_FAIL will be returned to the caller.                        *
 * Values bigger than 39 will be rounder in this implementation.    *
 * Internally, the range is converted to the corresponding nice     *
 * value that can range from -20 to 19.                                *
 ********************************************************************/

OSAL_PUBLIC OSAL_STATUS
osalThreadPrioritySet (OsalThread * pTid, UINT32 priority)
{
    struct task_struct *pTask = NULL;

    OSAL_LOCAL_ENSURE(pTid,
                        "OsalThreadPrioritySet(): NULL thread pointer",
                      OSAL_FAIL);

    OSAL_LOCAL_ENSURE(*pTid,
                        "OsalThreadPrioritySet(): NULL thread pointer",
                      OSAL_FAIL);

    pTask = (struct task_struct*)*pTid;

    if (priority > OSAL_PRIO_SET_MAX_VALID_VAL)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDERR,
                   "OsalThreadPrioritySet(): OSAL_FAIL \n",
                   0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    if (priority > OSAL_PRIO_SET_MAX_VAL) 
    {
         priority = OSAL_PRIO_SET_MAX_VAL;
    }

    /* sending the nice equivalent of priority as the parameter */
    set_user_nice ( pTask, priority - OSAL_NICE_VAL_DIFFERENCE ); 

    osalLog (OSAL_LOG_LVL_MESSAGE, OSAL_LOG_DEV_STDOUT,
        "OsalThreadPrioritySet(): Priority changed successfully \n",
        0, 0, 0, 0, 0, 0, 0, 0);

    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalThreadSetPolicyAndPriority(OsalThread *thread, UINT32 policy,
                                  UINT32 priority)
{
    return OSAL_SUCCESS;
}

