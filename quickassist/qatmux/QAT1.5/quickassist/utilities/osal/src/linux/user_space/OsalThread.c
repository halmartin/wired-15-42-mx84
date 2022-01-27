/**
 * @file OsalThread.c (user space)
 *
 * @brief POSIX thread implementation.
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
 *  version: QAT1.5.L.1.11.0-36
 */

#include "Osal.h"

OSAL_PUBLIC OSAL_STATUS osalThreadCreate(OsalThread *thread,
                                 OsalThreadAttr    *threadAttr,
                                 OsalVoidFnVoidPtr startRoutine,
                                 void *arg)
{
    INT32 status;
    pthread_attr_t attr;
    struct sched_param param;

    OSAL_LOCAL_ENSURE(startRoutine,
             "OsalThreadCreate(): Null start routine pointer",
              OSAL_FAIL);

    if(pthread_attr_init(&attr) != 0)
    {

        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "\nosalThreadCreate:"
            "Failed to initialize Thread Attributes !!!\n",
            0, 0, 0, 0, 0, 0, 0, 0);

        return OSAL_FAIL;
    }

    /* Setting scheduling parameter will fail for non root user,
     * as the default value of inheritsched is PTHREAD_EXPLICIT_SCHED in
     * POSIX. It is not required to set it explicitly before setting the
     * scheduling policy */

    if(threadAttr == NULL)
    {
        if(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0){
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                "\nosalThreadCreate:"
                "Failed to set inherit sched for thread!!!\n",
                0, 0, 0, 0, 0, 0, 0, 0);
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }

        if(pthread_attr_setschedpolicy(&attr,
            OSAL_OS_THREAD_DEFAULT_SCHED_POLICY ) != 0)
        {
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                "\nosalThreadCreate:"
                "Failed to set scheduling policy for thread !!!\n",
                0, 0, 0, 0, 0, 0, 0, 0);
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }

        /* Set priority based on value in threadAttr */
        osalMemSet(&param, 0, sizeof(param));
        param.sched_priority = OSAL_OS_DEFAULT_THREAD_PRIORITY;

        if(pthread_attr_setschedparam(&attr, &param) != 0)
        {
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                "\nosalThreadCreate:"
                "Failed to set the sched parameters atribute !!!\n",
                0, 0, 0, 0, 0, 0, 0, 0);
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }
    }
    else
    {
        /* Set scheduling policy based on value in threadAttr */

        if((threadAttr->policy != SCHED_RR) &&
                (threadAttr->policy != SCHED_FIFO) &&
                (threadAttr->policy != SCHED_OTHER))
        {
            osalLog (OSAL_LOG_LVL_DEBUG3, OSAL_LOG_DEV_STDOUT,
                "\nosalThreadCreate: Invalid policy type! " \
                "Changing to OSAL_THREAD_DEFAULT_SCHED_POLICY\n",
                0, 0, 0, 0, 0, 0, 0, 0);
            threadAttr->policy = OSAL_THREAD_DEFAULT_SCHED_POLICY;
        }

        if(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0){
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                "\nosalThreadCreate:"
                "Failed to set inherit sched for thread!!!\n",
                0, 0, 0, 0, 0, 0, 0, 0);
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }


        if(pthread_attr_setschedpolicy(&attr, threadAttr->policy) != 0)
        {
            osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                "\nosalThreadCreate:"
                "Failed to set scheduling policy for thread !!!\n",
                0, 0, 0, 0, 0, 0, 0, 0);
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }

        /* Set priority based on value in threadAttr */
        osalMemSet(&param, 0, sizeof(param));
        param.sched_priority = threadAttr->priority;

        if(threadAttr->policy != SCHED_OTHER)
        {
            if(pthread_attr_setschedparam(&attr, &param) != 0)
            {
                osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
                    "\nosalThreadCreate:"
                    "Failed to set the sched parameters atribute !!!\n",
                    0, 0, 0, 0, 0, 0, 0, 0);
                pthread_attr_destroy(&attr);
                return OSAL_FAIL;
            }
        }
    }/* end of if(threadAttr == NULL) */

    /* Always create detached thread as JOINABLE thread is not needed by
     * any of the OSAL users currently */

    if(pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "\nosalThreadCreate:"
            "Failed to set the dettachState attribute !!!\n",
            0, 0, 0, 0, 0, 0, 0, 0);
        pthread_attr_destroy(&attr);
        return OSAL_FAIL;
    }

    if(pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM) != 0)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "\nosalThreadCreate: Failed to set the attribute !!!\n",
            0, 0, 0, 0, 0, 0, 0, 0);
        pthread_attr_destroy(&attr);
        return OSAL_FAIL;
    }

    status = pthread_create(thread, &attr,
            (void* (*)(void *))startRoutine, arg);
    if(status)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "\nosalThreadCreate: Failed to create the thread!\n",
            0, 0, 0, 0, 0, 0, 0, 0);
        pthread_attr_destroy(&attr);
        return OSAL_FAIL;
    }

    return OSAL_SUCCESS;
} /* osalThreadCreate */



OSAL_PUBLIC OSAL_STATUS osalThreadStart (OsalThread * thread)
{

   osalLog (OSAL_LOG_LVL_DEBUG3, OSAL_LOG_DEV_STDOUT,
       "\nosalThreadStart: Nothing to be done !!\n",
       0, 0, 0, 0, 0, 0, 0, 0);

   return OSAL_SUCCESS;
}


OSAL_PUBLIC OSAL_STATUS osalThreadKill (OsalThread * thread)
{
    INT32           status;

    OSAL_LOCAL_ENSURE(thread,
                "osalThreadKill(): Null thread pointer",
                OSAL_FAIL);

    status = pthread_cancel(*thread);

    if(status)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "\nosalThreadKill: Failed to cancel the thread!\n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    return OSAL_SUCCESS;
} /* osalThreadKill */


OSAL_PUBLIC OSAL_STATUS osalThreadPrioritySet (OsalThread *thread,
                                               UINT32 priority)
{
#ifdef __linux__
    INT32              status;
    struct sched_param param1;
    int                policy1;
    int                minPrio, maxPrio;

    OSAL_LOCAL_ENSURE(thread,
                  "osalThreadPrioritySet(): Null thread pointer",
                  OSAL_FAIL);

    if(pthread_getschedparam(*thread, &policy1, &param1) != 0){
           osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
               "osalThreadPrioritySet(): could not get sched param !!!\n",
               0, 0, 0, 0, 0, 0, 0, 0);
           return OSAL_FAIL;
    }

    minPrio = sched_get_priority_min(policy1);
    maxPrio =  sched_get_priority_max(policy1);

    if ((priority < minPrio) || (priority > maxPrio))
    {
          osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
              "osalThreadPrioritySet(): Erroneous priority !!!\n",
              0, 0, 0, 0, 0, 0, 0, 0);
          return OSAL_FAIL;
    }

    status = pthread_setschedprio(*thread, priority);
    if(status)
    {
       osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
           "\nosalThreadPrioritySet(): set prio failed \n !",
           0, 0, 0, 0, 0, 0, 0, 0);
       return OSAL_FAIL;
    }
#else
    osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
        "\nosalThreadPrioritySet(): not implemented on this OS \n !",
        0, 0, 0, 0, 0, 0, 0, 0);
#endif
    return OSAL_SUCCESS;
}


OSAL_PUBLIC void osalThreadExit (void)
{
    pthread_exit(NULL);
}



/* API to set scheduling policy and priority of an executing thread */

OSAL_PUBLIC OSAL_STATUS
osalThreadSetPolicyAndPriority(OsalThread *thread, UINT32 policy,
                                              UINT32 priority)
{
    int                 ret = 0;
    struct sched_param  param;
    int                 minPrio, maxPrio, policy1;

    OSAL_LOCAL_ENSURE(thread,
                      "osalThreadSetPolicyAndPriority(): Null thread pointer",
                       OSAL_FAIL);


   /* check for a valid value for 'policy' */
   if ((policy != SCHED_RR) &&
       (policy != SCHED_FIFO) &&
       (policy != SCHED_OTHER))
   {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "osalThreadSetPolicyAndPriority(): invalid value for policy!!! \n",
            0, 0, 0, 0, 0, 0, 0, 0);

        return OSAL_FAIL;
    }

   osalMemSet(&param, 0, sizeof(param));


    if(pthread_getschedparam(*thread, &policy1, &param) != 0){
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "osalThreadSetPolicyAndPriority(): could not get sched param !!!\n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
     }

    minPrio = sched_get_priority_min(policy);
    maxPrio =  sched_get_priority_max(policy);

    if ((priority < minPrio) || (priority > maxPrio))
    {
        osalLog (OSAL_LOG_LVL_ERROR,
            OSAL_LOG_DEV_STDOUT,
            "osalThreadPrioritySet(): Errorneous priority !!!\n",
            0, 0, 0, 0, 0, 0, 0, 0);

         return OSAL_FAIL;
    }

    param.sched_priority = priority;
    ret = pthread_setschedparam (*thread, policy, &param);

    if (ret !=0)
    {
        osalLog (OSAL_LOG_LVL_ERROR, OSAL_LOG_DEV_STDOUT,
            "osalThreadSetPolicyAndPriority:"
            "Policy and Priority Set operation failed!!! \n",
            0, 0, 0, 0, 0, 0, 0, 0);
        return OSAL_FAIL;
    }

    return OSAL_SUCCESS;

}


