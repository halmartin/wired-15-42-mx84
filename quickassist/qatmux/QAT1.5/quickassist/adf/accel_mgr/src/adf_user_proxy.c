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

/******************************************************************************
 * @file  adf_user_proxy.c
 *
 * @description
 *        This file is the kernel part of proxy between user and
 *        kernel space for the components of the Acceleration
 *        Driver Framework (ADF).
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_adf_init.h"
#include "icp_adf_cfg.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_transport.h"
#include "icp_adf_accel_mgr.h"
#include "adf_init.h"
#include "adf_transport_ctrl.h"
#include "icp_adf_transport_dp.h"
#include "adf_user_proxy.h"
#include "adf_devmgr.h"
#include "adf_dev_csr.h"
#include "adf_cfg.h"
#include "adf_ETring_mgr.h"
#include "adf_dev_ring.h"
#include "adf_wireless.h"
#include "adf_dyn.h"

#define USER_SUBSYSTEM_NAME     ("ADF_USER_PROXY")
#define TIME_DELAY              (240)
#define TIME_TO_SLEEP_IN_MS     (80)

STATIC const char* thread_name = "adf_orphan_kthread";

/*
 * Increase module usage counter function
 */
extern int adf_get_module(void);
/*
 * Decrease module usage counter function
 */
extern void adf_put_module(void);

/* Create a subservice registration handle for user_space. */
STATIC subservice_registation_handle_t user_space_reg_handle;
STATIC char* subsystem_name = USER_SUBSYSTEM_NAME;

/* structure for handling orphaned rings when a userspace process
 * that created them dies */
typedef struct orphan_rings_list_s {
    Cpa32U pid;
    icp_accel_dev_t *pAccelDev;
    /* response rings list */
    icp_trans_handle *response_rings_list_head;
    icp_trans_handle *response_rings_list_tail;
    /* request rings list */
    icp_trans_handle *request_rings_list_head;
    icp_trans_handle *request_rings_list_tail;
    /* response counters */
    Cpa32U responses_before;
    Cpa32U responses_now;
    OsalTimeval time;
    CpaBoolean ok_to_clean;
    struct orphan_rings_list_s* pNext;
    struct orphan_rings_list_s* pPrev;
} orphan_rings_list_t;

STATIC orphan_rings_list_t *orphans_list_head = NULL;
STATIC orphan_rings_list_t *orphans_list_tail = NULL;
STATIC ICP_SPINLOCK orphan_list_lock;

/*
 * flag that prevents multiple orphan threads
 * running at the same time
 */
STATIC OsalAtomic thread_running_flag;

/*
 * adf_user_proxyEventGetPut
 * Function increments the device usage counter
 */
CpaStatus adf_user_proxyEventGetPut(icp_accel_dev_t *accel_dev,
                                             Cpa32U pid, Cpa32U cmd)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_t *process = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    /*
     * Get a list of userspace processes connected to the device
     */
    procList = (adf_user_process_list_t*) accel_dev->pUserSpaceHandle;

    if(procList)
    {
        process = procList->listHead;
        /*
         * Loop through all userspace processes connected to the device
         */
        while(process)
        {
            /* if this is the process we want */
            if(process->processId == pid)
            {
                /* Increment or decrement the counter */
                if(DEV_GET_CMD == cmd)
                {
                    process->processUsageCounter++;
                }
                else if(DEV_PUT_CMD == cmd)
                {
                    process->processUsageCounter--;
                }
                break;
            }
            process = process->pNext;
        }
    }
    return stat;
}

/*
 * kthread function that will clean all the orphans rings
 * in the background if the rings are not used
 * This function is kicked off in a separate thread from
 * adf_userProcessDisconnect function. The idea is that when a process gets
 * killed while having allocated rings and some pending requests we can
 * not release the rings and free the ring's memory as MEs will write to
 * it and if we release it before the last response comes then we will be
 * in trouble. So what we need to do is to wait for all the outstanding
 * responses and only free the ring when we are sure that we
 * received all the outstanding responses. This function loops few times
 * counting all the responses on the process ring and if we don't get any
 * responses for three loops then we fire up a counter and loop again
 * checking the number of responses every time. If there is no responses
 * then we mark the process as ok_to_clean and it gets cleaned in the next
 * loop iteration i.e. all the rings allocated by the process are released.
 */
STATIC void* clean_orphan_list(void* arg)
{
    Cpa8U done = 0;

    /* Run till all is cleaned */
    while(!done)
    {
        orphan_rings_list_t *process_rings = orphans_list_head;
        orphan_rings_list_t *process_rings_next = NULL;

        /* for every process in in the orphans_list */
        while(process_rings)
        {
            Cpa32U responses = 0;

            /* if we are ok to clean */
            if (process_rings->ok_to_clean)
            {
                icp_trans_handle *response_ring = NULL;
                icp_trans_handle *request_ring = NULL;

                /* first clean the request rings */
                request_ring = process_rings->request_rings_list_head;
                while (request_ring)
                {
                    if (request_ring->is_dyn)
                    {
                        adf_dynResetRings(request_ring->accel_dev,
                                            request_ring->trans_data);
                    }
                    else if (request_ring->is_wireless)
                    {
                        adf_wirelessResetRings(request_ring->accel_dev,
                                            request_ring->trans_data);
                    }
                    else
                    {
                        icp_adf_transReleaseHandle((icp_comms_trans_handle)
                                                request_ring);
                    }
                    ICP_REMOVE_ELEMENT_FROM_LIST(
                               request_ring,
                               process_rings->request_rings_list_tail,
                               process_rings->request_rings_list_head);

                    request_ring = request_ring->pNext;
                }

                /* then the response rings */
                response_ring = process_rings->response_rings_list_head;
                while (response_ring)
                {
                    /* remove the ring  */
                    if (response_ring->is_dyn)
                    {
                        adf_dynResetRings(response_ring->accel_dev,
                                                response_ring->trans_data);
                    }
                    else if (response_ring->is_wireless)
                    {
                        adf_wirelessResetRings(response_ring->accel_dev,
                                                response_ring->trans_data);
                    }
                    else
                    {
                        icp_adf_transReleaseHandle((icp_comms_trans_handle)
                                                response_ring);
                    }
                    /* remove the ring from response ring list */
                    ICP_REMOVE_ELEMENT_FROM_LIST(
                               response_ring,
                               process_rings->response_rings_list_tail,
                               process_rings->response_rings_list_head);
                    response_ring = response_ring->pNext;
                }

                /*
                 * Cleaned all orphans for the process
                 * so we can remove the process from the list
                 */
                ICP_SPINLOCK_LOCK(&orphan_list_lock);
                ICP_REMOVE_ELEMENT_FROM_LIST(process_rings,
                                          orphans_list_tail,
                                          orphans_list_head);
                ICP_SPINLOCK_UNLOCK(&orphan_list_lock);

                CLEAR_STATUS_BIT(process_rings->pAccelDev->adfSubsystemStatus,
                                    ADF_STATUS_ORPHAN_TH_RUNNING);
                process_rings_next = process_rings->pNext;
                ICP_FREE(process_rings);
                process_rings = process_rings_next;
                break;
            }
            else
            {
                /* if not have a look what is going on
                 * and if there is no responses coming for some time
                 * go ahead and clean */

                icp_trans_handle *response_ring = NULL;
                icp_et_ring_data_t *ringData = NULL;
                response_ring = process_rings->response_rings_list_head;

                /* count the responses the (dead) process received recently */
                while(response_ring)
                {
                    ringData = (icp_et_ring_data_t *) response_ring->trans_data;
                    /* Make sure that the ring is empty */
                    if(adf_is_ring_polled(ringData))
                    {
                        if(CPA_STATUS_RETRY !=
                                icp_adf_pollQueue((icp_comms_trans_handle)
                                           response_ring, 0))
                        {
                            responses++;
                        }
                   }
                    responses += ringData->orphanResponseCount;
                    response_ring = response_ring->pNext;
                }

                if(0 == process_rings->time.secs)
                {
                    /* this is the first run of the loop
                     * set initial values and go to sleep */

                    process_rings->responses_now = responses;
                    ICP_GET_TIME(&process_rings->time);
                }
                /* check if we are still getting responses */
                else if(process_rings->responses_now != responses)
                {
                    /* if we do just note the fact */
                    process_rings->responses_before =
                                   process_rings->responses_now;
                    process_rings->responses_now = responses;
                    ICP_GET_TIME(&process_rings->time);
                }
                else if(process_rings->responses_now == responses)
                {
                    /* if we didn't get responses since the last loop run */
                    if(process_rings->responses_before == responses)
                    {
                        /* and we didn't get responses for two loop run
                         */
                        Cpa32U time_elapsed =
                            OSAL_TIMEVAL_TO_MS(process_rings->time)
                                                             + TIME_DELAY;
                        OsalTimeval time_now;
                        ICP_GET_TIME(&time_now);

                        /*
                         * It means that we didn't get any responses recently
                         * and we are probably ok to clean but better wait for
                         * some time. Make sure that enough time passed (at
                         * least three seconds) since we got the last
                         * response
                         */
                        if(time_elapsed < OSAL_TIMEVAL_TO_MS(time_now))
                        {
                            /*
                             * we didn't get any responses for a while
                             * and we are  ok to clean the list
                             */
                            process_rings->ok_to_clean = CPA_TRUE;
                        }
                    }
                    else
                    {
                        /* still getting responses */
                        process_rings->responses_before =
                                   process_rings->responses_now;
                        process_rings->responses_now = responses;
                        /* update timestamp as we want to wait
                           from the last response */
                        ICP_GET_TIME(&process_rings->time);
                    }
                }
            }
            process_rings = process_rings->pNext;
        }

        /* if the list is empty then we are done */
        if(NULL == orphans_list_head)
        {
            /* Cleaned all orphanes. We are done */
            done = 1;
        }
        else
        {
            /* if not relax and wait for the next loop iteration */
            ICP_MSLEEP(TIME_TO_SLEEP_IN_MS);
        }
    }
    /* unset the running flag and exit */
    adf_put_module();
    osalAtomicSet(1, &thread_running_flag);
    return NULL;
}

/*
 * adf_user_proxyInit
 * Initialise internal structures and bind it to the device
 */
CpaStatus adf_user_proxyInit(icp_accel_dev_t* accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_user_process_list_t *procList = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    procList = ICP_ZALLOC_GEN(sizeof(adf_user_process_list_t));

    if(procList)
    {
        ICP_MUTEX_INIT(&procList->procListLock);
        accel_dev->pUserSpaceHandle = procList;
    }
    else
    {
        ADF_ERROR("failed allocate memory for user process list\n");
        stat = CPA_STATUS_RESOURCE;
    }

    return stat;
}

/*
 * adf_user_proxyShutdown event handler
 * Shutting down but free all allocated
 */
CpaStatus adf_user_proxyShutdown(icp_accel_dev_t* accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_t *process = NULL;
    adf_user_process_rings_list_t *ringList = NULL;
    icp_trans_handle *userRing = NULL;
    icp_et_ring_data_t *ringData = NULL;
    adf_user_process_dyn_instances_list_t *pDynInstanceList = NULL;
    icp_dyn_instance_handle_t *pDynHandle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    procList = (adf_user_process_list_t*) accel_dev->pUserSpaceHandle;
    if(procList)
    {
        while(procList->listHead)
        {
            int processRingNotified = 1;
            process = procList->listTail;
            ringList = process->pUserRings;
            pDynInstanceList = process->pDynInstances;
            /*
             * proxy performs shutdown and there are still
             * user processes out there in user space connected.
             * Have to wait for them to close the handle.
             */
            if (OSAL_SUCCESS != osalSemaphoreWaitInterruptible(
                       &process->processDisconnectedFlag, OSAL_WAIT_FOREVER))
            {
                ADF_ERROR("Semaphore wait failed\n");
            }
            while (ringList && ringList->listHead)
            {
                userRing = ringList->listTail;
                ringData = (icp_et_ring_data_t *) userRing->trans_data;

                /* If the process has response rings need to notify
                 * the reading thread to finish - just once per process */
                if(processRingNotified && (NULL != ringData->callbackFn))
                {
                    adf_ring_process_finish_notify(ringData);
                    processRingNotified = 0;
                }

                ringData->userSpacePrivateData = NULL;
                ICP_REMOVE_ELEMENT_FROM_LIST(userRing,
                                              ringList->listTail,
                                              ringList->listHead);
                if ((!userRing->is_wireless) && (!userRing->is_dyn))
                {
                    icp_adf_transReleaseHandle((icp_comms_trans_handle) userRing);
                    osalAtomicDec(&accel_dev->userspaceRingsCounter);
                }
            }
            ICP_FREE(ringList);
            while (pDynInstanceList && pDynInstanceList->listHead)
            {
                pDynHandle = pDynInstanceList->listTail;
                if (!icp_adf_is_dev_in_reset(accel_dev))
                {
                    icp_adf_putDynInstance(accel_dev, pDynHandle->stype,
                                             pDynHandle->instance_id);
                }
                ICP_REMOVE_ELEMENT_FROM_LIST(pDynHandle,
                                             pDynInstanceList->listTail,
                                             pDynInstanceList->listHead);
                ICP_FREE(pDynHandle);
            }
            ICP_FREE(pDynInstanceList);
            ICP_MUTEX_LOCK(&procList->procListLock);
            ICP_REMOVE_ELEMENT_FROM_LIST(process,
                                          procList->listTail,
                                          procList->listHead);
            ICP_MUTEX_UNLOCK(&procList->procListLock);
            if (OSAL_SUCCESS !=
                    osalSemaphoreDestroy (&process->processDisconnectedFlag))
            {
                ADF_ERROR("Semaphore destroy failed\n");
            }

            ICP_FREE(process);
        }

        ICP_MUTEX_UNINIT(&procList->procListLock);
        ICP_FREE(procList);
    }

    return stat;
}

/*
 * adf_user_proxyEventInit event handler
 * Forward the init message to all connected userspace processes
 */
STATIC CpaStatus adf_user_proxyEventInit(icp_accel_dev_t *accel_dev,
                                          void* param)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_t *process = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    procList = (adf_user_process_list_t*) accel_dev->pUserSpaceHandle;

    /*
     * If procList is NULL this means that there is no process connected yet
     * and it will be the case almost always so function just returns SUCCESS.
     * Init and Start events are sent to each process once it connects.
     */
    if(procList)
    {
        process = procList->listHead;
        while(process)
        {
            /* Add the event to the pending queue */
            ICP_SPINLOCK_LOCK(&process->userProcLock);
            process->pendingEventsQueue[process->pendingEvents] =
                                                      ICP_ADF_EVENT_INIT;
            process->pendingEvents++;
            ICP_SPINLOCK_UNLOCK(&process->userProcLock);
            /* Notify the process */
            adf_csr_user_process_event_notify(process);
            process = process->pNext;
        }
    }

    return stat;
}

/*
 * adf_user_proxyEventStart event handler
 * Forward the start message to all connected userspace processes
 */
STATIC CpaStatus adf_user_proxyEventStart(icp_accel_dev_t *accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_t *process = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    procList = (adf_user_process_list_t*) accel_dev->pUserSpaceHandle;

    if(procList)
    {
        process = procList->listHead;
        while(process)
        {
            /* Add the event to the pending queue */
            ICP_SPINLOCK_LOCK(&process->userProcLock);
            process->pendingEventsQueue[process->pendingEvents] =
                                                      ICP_ADF_EVENT_START;
            process->pendingEvents++;
            ICP_SPINLOCK_UNLOCK(&process->userProcLock);
            /* Notify the process */
            adf_csr_user_process_event_notify(process);
            process = process->pNext;
        }
    }

    return stat;
}

/*
 * adf_user_proxyEventStop event handler
 * Forward the stop message to all connected userspace processes
 */
STATIC CpaStatus adf_user_proxyEventStop(icp_accel_dev_t *accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_t *process = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    procList = (adf_user_process_list_t*) accel_dev->pUserSpaceHandle;
    if(procList)
    {
        process = procList->listHead;
        while(process)
        {
            /* Add the event to the pending queue */
            ICP_SPINLOCK_LOCK(&process->userProcLock);
            process->pendingEventsQueue[process->pendingEvents] =
                                                      ICP_ADF_EVENT_STOP;
            process->pendingEvents++;
            ICP_SPINLOCK_UNLOCK(&process->userProcLock);
            /* Notify the process */
            adf_csr_user_process_event_notify(process);
            process = process->pNext;
        }
    }
    return stat;
}

/*
 * adf_user_proxyEventShutdown event handler
 * Forward the shutdown message to all connected userspace processes
 */
STATIC CpaStatus adf_user_proxyEventShutdown(icp_accel_dev_t *accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_t *process = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    procList = (adf_user_process_list_t*) accel_dev->pUserSpaceHandle;
    if(procList)
    {
        process = procList->listHead;
        while(process)
        {
            /* Add the event to the pending queue */
            ICP_SPINLOCK_LOCK(&process->userProcLock);
            process->pendingEventsQueue[process->pendingEvents] =
                                                      ICP_ADF_EVENT_SHUTDOWN;
            process->pendingEvents++;
            ICP_SPINLOCK_UNLOCK(&process->userProcLock);
            /* Notify the process */
            adf_csr_user_process_event_notify(process);
            process = process->pNext;
        }
    }
    return stat;
}

/*
 * adf_user_proxyEventRestarting event handler
 * Forward the restarting message to all connected userspace processes
 */
STATIC CpaStatus adf_user_proxyEventRestarting(icp_accel_dev_t *accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_t *process = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    procList = (adf_user_process_list_t*) accel_dev->pUserSpaceHandle;
    if(procList)
    {
        process = procList->listHead;
        while(process)
        {
            /* Add the event to the pending queue */
            ICP_SPINLOCK_LOCK(&process->userProcLock);
            process->pendingEventsQueue[process->pendingEvents] =
                                                      ICP_ADF_EVENT_RESTARING;
            process->pendingEvents++;
            ICP_SPINLOCK_UNLOCK(&process->userProcLock);
            /* Notify the process */
            adf_csr_user_process_event_notify(process);
            process = process->pNext;
        }
    }
    return stat;
}

/*
 * Get the connected user process up to speed
 */
STATIC inline void adf_userProcessStateCatchUp(adf_user_process_t* pUserProc)
{
    Cpa32U state = ICP_ADF_EVENT_INIT;

    for(state = ICP_ADF_EVENT_INIT; state < ICP_ADF_EVENT_STOP ; state++)
    {
        pUserProc->pendingEventsQueue[pUserProc->pendingEvents++] = state;
    }
}

/*
 * adf_userProcessCreateRing
 * ET ring has been created on behalf of the user space process pid
 * Need to add it to a process ring list just in case the process
 * fails to release it so we can do it for the process.
 */
CpaStatus adf_userProcessCreateRing(icp_accel_dev_t *accel_hdl,
                                     icp_trans_handle *ring_hdl,
                                     Cpa32U pid)
{
    CpaStatus stat = CPA_STATUS_FAIL;
    adf_user_process_t *pUserProc = NULL;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_rings_list_t *rings = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_hdl);
    ICP_CHECK_FOR_NULL_PARAM(ring_hdl);

    procList = accel_hdl->pUserSpaceHandle;
    ICP_CHECK_FOR_NULL_PARAM(procList);

    pUserProc = procList->listHead;

    /* Loop for all processes connected to the device */
    while(pUserProc)
    {
        /* if this is our process */
        if(pUserProc->processId == pid)
        {
            /* add the ring to the process ring list.
             * For other processes will send new ring event */
            rings = pUserProc->pUserRings;
            if(rings)
            {
                ICP_ADD_ELEMENT_TO_END_OF_LIST(ring_hdl,
                                     rings->listTail, rings->listHead);
                stat = CPA_STATUS_SUCCESS;
            }
        }
        pUserProc = pUserProc->pNext;
    }
    if(CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Failed to add user ring to process list\n");
    }

    return stat;
}

/*
 * adf_userProcessDeleteRing
 * User space process doesn't need the ring anymore so need to remove it
 * from the process ring list
 */
CpaStatus adf_userProcessDeleteRing(icp_accel_dev_t *accel_hdl,
                                     icp_trans_handle *ring_hdl,
                                     Cpa32U pid)
{
    CpaStatus stat = CPA_STATUS_FAIL;
    adf_user_process_t *pUserProc = NULL;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_rings_list_t *rings = NULL;
    icp_et_ring_data_t *ringData = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_hdl);
    ICP_CHECK_FOR_NULL_PARAM(ring_hdl);

    procList = accel_hdl->pUserSpaceHandle;
    ICP_CHECK_FOR_NULL_PARAM(procList);

    pUserProc = procList->listHead;

    ringData = (icp_et_ring_data_t *) ring_hdl->trans_data;

    if (BIT_IS_SET(accel_hdl->adfSubsystemStatus,
                                       ADF_STATUS_WIRELESS_ENABLED))
    {
        stat = adf_wirelessResetRings(accel_hdl, ringData);
        ringData->userSpacePrivateData = NULL;
    }
    if (CPA_TRUE == ring_hdl->is_dyn)
    {
        stat = adf_dynResetRings(accel_hdl, ringData);
        ringData->userSpacePrivateData = NULL;
    }

    /* Loop for all process connected to the device */
    while(pUserProc)
    {
        /* if this is our process */
        if(pUserProc->processId == pid)
        {
            rings = pUserProc->pUserRings;
            if(rings)
            {
                /* remove the ring from its ring list */
                ICP_REMOVE_ELEMENT_FROM_LIST(ring_hdl,
                                     rings->listTail, rings->listHead);
                stat = CPA_STATUS_SUCCESS;
                break;
            }
        }
        pUserProc = pUserProc->pNext;
    }
    return stat;
}

/*
 * adf_userProcessCreateDynInstance
 * Dyn Instance has been created on behalf of the user space process pid
 * Need to add it to a process dyn instance list just in case the process
 * fails to release it so we can do it for the process.
 */
inline CpaStatus adf_userProcessCreateDynInstance(icp_accel_dev_t *accel_hdl,
                                           Cpa32U inst_id,
                                           adf_service_type_t stype,
                                           Cpa32U pid)
{
    CpaStatus stat = CPA_STATUS_FAIL;
    adf_user_process_t *pUserProc = NULL;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_dyn_instances_list_t *pDynInstanceList = NULL;
    icp_dyn_instance_handle_t *pDynHandle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_hdl);

    procList = accel_hdl->pUserSpaceHandle;
    ICP_CHECK_FOR_NULL_PARAM(procList);

    pUserProc = procList->listHead;

    pDynHandle =
           ICP_ZALLOC_GEN(sizeof(icp_dyn_instance_handle_t));
    if (NULL == pDynHandle)
    {
        ADF_ERROR("Unable to allocate memory for pDynHandle\n");
        return CPA_STATUS_FAIL;
    }
    pDynHandle->instance_id = inst_id;
    pDynHandle->stype = stype;
    ICP_MUTEX_LOCK(&procList->procListLock);
    /* Loop for all processes connected to the device */
    while(pUserProc)
    {
        /* if this is our process */
        if(pUserProc->processId == pid)
        {
            /* add the dyn instances to the process dyn instance list. */
            pDynInstanceList = pUserProc->pDynInstances;
            if(pDynInstanceList)
            {
                ICP_ADD_ELEMENT_TO_END_OF_LIST(pDynHandle,
                                     pDynInstanceList->listTail,
                                     pDynInstanceList->listHead);
                stat = CPA_STATUS_SUCCESS;
                break;
            }
        }
        pUserProc = pUserProc->pNext;
    }
    ICP_MUTEX_UNLOCK(&procList->procListLock);
    if(CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Failed to add user dyn instance to process list\n");
        ICP_FREE(pDynHandle);
    }

    return stat;
}

/*
 * adf_userProcessDeleteDynInstance
 * User space process doesn't need the dyn instances anymore
 * so need to remove it
 * from the process dyn instances list
 */
inline CpaStatus adf_userProcessDeleteDynInstance(icp_accel_dev_t *accel_hdl,
                                           Cpa32U inst_id,
                                           adf_service_type_t stype,
                                           Cpa32U pid)
{
    CpaStatus stat = CPA_STATUS_FAIL;
    adf_user_process_t *pUserProc = NULL;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_dyn_instances_list_t *pDynInstanceList = NULL;
    icp_dyn_instance_handle_t *pDynHandle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_hdl);

    procList = accel_hdl->pUserSpaceHandle;
    ICP_CHECK_FOR_NULL_PARAM(procList);

    pUserProc = procList->listHead;

    ICP_MUTEX_LOCK(&procList->procListLock);
    /* Loop for all process connected to the device */
    while(pUserProc)
    {
        /* if this is our process */
        if(pUserProc->processId == pid)
        {
            pDynInstanceList = pUserProc->pDynInstances;
            if(NULL == pDynInstanceList)
            {
                pUserProc = pUserProc->pNext;
                continue;
            }
            pDynHandle = pDynInstanceList->listHead;
            while(pDynHandle)
            {
                if((pDynHandle->instance_id == inst_id)
                   && (pDynHandle->stype == stype))
                {
                    /* remove the dyn instance from its dyn instance list */
                    ICP_REMOVE_ELEMENT_FROM_LIST(pDynHandle,
                                         pDynInstanceList->listTail,
                                         pDynInstanceList->listHead);
                    ICP_FREE(pDynHandle);
                    stat = CPA_STATUS_SUCCESS;
                    break;
                }
                pDynHandle = pDynHandle->pNext;
            }
            if (CPA_STATUS_SUCCESS == stat)
            {
                break;
            }
        }
        pUserProc = pUserProc->pNext;
    }
    ICP_MUTEX_UNLOCK(&procList->procListLock);
    return stat;
}

/*
 * adf_userProcessConnect
 * User space process procId connected to the device accelDevId
 * Check if we are ready to handle userspace processes and if so
 * initialize appropriate data structures and get the process up to
 * speed by sending it init and start events.
 */
CpaStatus adf_userProcessConnect(void** proxyPrivateData,
                                          Cpa32U accelDevId, Cpa32U procId)
{
    CpaStatus stat = CPA_STATUS_FAIL;
    icp_accel_dev_t *pAccelDev = NULL;
    adf_user_process_t *pUserProc = NULL;
    adf_user_process_list_t *procList = NULL;

    adf_devmgrGetAccelHead(&pAccelDev);

    while(pAccelDev)
    {
        if (pAccelDev->accelId == accelDevId)
        {
            /* Only accept connections on initialized device */
            if(!(BIT_IS_SET(pAccelDev->adfSubsystemStatus,
                                                  ADF_STATUS_SYSTEM_STARTED)))
            {
                ADF_DEBUG("Quick Assist device icp_dev%d not initialised.\n",
                                                          pAccelDev->accelId);
                stat = CPA_STATUS_RESOURCE;
                break;
            }

            pUserProc = ICP_ZALLOC_GEN(sizeof(adf_user_process_t));

            if(NULL == pUserProc)
            {
                ADF_ERROR("Failed allocate memory for user process\n");
                stat = CPA_STATUS_RESOURCE;
                break;
            }

            ICP_SPINLOCK_INIT(&pUserProc->userProcLock);
            procList = pAccelDev->pUserSpaceHandle;

            if(NULL == procList)
            {
                ADF_ERROR("Device uninitialised\n");
                stat = CPA_STATUS_FAIL;
                ICP_FREE(pUserProc);
                break;
            }

            ICP_MUTEX_LOCK(&procList->procListLock);
            ICP_ADD_ELEMENT_TO_END_OF_LIST(pUserProc,
                                    procList->listTail, procList->listHead);
            ICP_MUTEX_UNLOCK(&procList->procListLock);

            pUserProc->accelHandle = pAccelDev;
            pUserProc->processId = procId;

            /* store user process struct in process private data*/
            *proxyPrivateData = (void*) pUserProc;
            /* Init the disconnected flag */
            if(OSAL_SUCCESS != osalSemaphoreInit(
                                   &pUserProc->processDisconnectedFlag, 0))
            {
                ADF_ERROR("Error initing process semaphore\n");
                ICP_MUTEX_LOCK(&procList->procListLock);
                ICP_REMOVE_ELEMENT_FROM_LIST(pUserProc,
                                procList->listTail, procList->listHead);
                ICP_MUTEX_UNLOCK(&procList->procListLock);
                ICP_FREE(pUserProc);
                stat = CPA_STATUS_RESOURCE;
                break;
            }
            /*Create dyn instance list for the process */
            pUserProc->pDynInstances = ICP_ZALLOC_GEN(
                               sizeof(adf_user_process_dyn_instances_list_t));
            if(NULL == pUserProc->pDynInstances)
            {
                ADF_ERROR("Error initialising user process\n");
                ICP_MUTEX_LOCK(&procList->procListLock);
                ICP_REMOVE_ELEMENT_FROM_LIST(pUserProc,
                                      procList->listTail, procList->listHead);
                ICP_MUTEX_UNLOCK(&procList->procListLock);
                osalSemaphoreDestroy(&pUserProc->processDisconnectedFlag);
                ICP_FREE(pUserProc);
                stat = CPA_STATUS_FAIL;
                break;
            }
            /*Create ring list for the process */
            pUserProc->pUserRings = ICP_ZALLOC_GEN(
                                       sizeof(adf_user_process_rings_list_t));
            if(pUserProc->pUserRings)
            {
                /* We are started so let the process catchup */
                adf_userProcessStateCatchUp(pUserProc);
                stat = CPA_STATUS_SUCCESS;
                break;
            }
            else
            {
                ADF_ERROR("Error initialising user process\n");
                ICP_FREE(pUserProc->pDynInstances);
                ICP_MUTEX_LOCK(&procList->procListLock);
                ICP_REMOVE_ELEMENT_FROM_LIST(pUserProc,
                                      procList->listTail, procList->listHead);
                ICP_MUTEX_UNLOCK(&procList->procListLock);
                osalSemaphoreDestroy(&pUserProc->processDisconnectedFlag);
                ICP_FREE(pUserProc);
                stat = CPA_STATUS_FAIL;
                break;
            }
        }
        pAccelDev = pAccelDev->pNext;
    }
    return stat;
}

STATIC void orphan_callback(icp_comms_trans_handle trans_handle,
                                               void *pMsg)
{
    return;
}

/*
 * adf_userProcessDisconnect
 * User space process disconnected (or got killed) from the device
 * Need to clean it up in case it failed to release rings etc
 */
CpaStatus adf_userProcessDisconnect(void** proxyPrivateData)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    icp_accel_dev_t *pAccelDev = NULL;
    adf_user_process_t *pUserProc = NULL;
    adf_user_process_list_t *procList = NULL;
    adf_user_process_rings_list_t *ringList = NULL;
    icp_trans_handle *userRing = NULL;
    icp_et_ring_data_t *ringData = NULL;
    adf_user_process_dyn_instances_list_t *pDynInstanceList = NULL;
    icp_dyn_instance_handle_t *pDynHandle = NULL;
    orphan_rings_list_t *list = NULL;
    OsalThread thread;
    OsalThreadAttr thread_attr;
    int have_rings = 0;

    pUserProc = (adf_user_process_t*) *proxyPrivateData;
    if (pUserProc)
    {
        pAccelDev = pUserProc->accelHandle;
        procList = pAccelDev->pUserSpaceHandle;
        ringList = pUserProc->pUserRings;
        pDynInstanceList = pUserProc->pDynInstances;
        if (!procList)
        {
            ADF_ERROR("procList is NULL\n");
            return CPA_STATUS_INVALID_PARAM;
        }
        /*
         * If the process called device get function and it didn't
         * call device put function we need to call it now.
         */
        if (!icp_adf_is_dev_in_reset(pAccelDev) &&
                                    pUserProc->processUsageCounter)
        {
            osalAtomicSub(pUserProc->processUsageCounter,
                    &pAccelDev->usageCounter);
        }

        /* If system is going down don't need to
         * defer rings cleaning - proxy shutdown function
         * is waiting to cleanup stuff just wake it up */
        if (!BIT_IS_SET(pAccelDev->adfSubsystemStatus,
                            ADF_STATUS_SYSTEM_STARTED))
        {
            if (OSAL_SUCCESS !=
                  osalSemaphorePostWakeup(&pUserProc->processDisconnectedFlag))
            {
                ADF_ERROR("Semaphore wakeup failed\n");
            }
            return CPA_STATUS_SUCCESS;
        }

        if (ringList && ringList->listHead)
        {
            list = ICP_MALLOC_ATOMIC(sizeof(orphan_rings_list_t));
            if (!list)
            {
                ADF_ERROR("Could not allocate memory\n");
                return CPA_STATUS_RESOURCE;
            }
            ICP_MEMSET(list, '\0', sizeof(orphan_rings_list_t));
            list->pAccelDev = pAccelDev;
            list->pid = pUserProc->processId;
        }

        ICP_MUTEX_LOCK(&procList->procListLock);

        /* Need to remove all the rings if the process didn't
         * release it because got killed or something like that */
        while (ringList && ringList->listHead)
        {
            userRing = ringList->listTail;
            /*
             * Can not just remove rings as there can be some
             * requests in flight so first remove the ring
             * from process ring list
             */
            ICP_REMOVE_ELEMENT_FROM_LIST(userRing,
                                         ringList->listTail,
                                         ringList->listHead);
            /*
             * and add the ring to the orphan list
             * and clean the list later
             */
            ringData = (icp_et_ring_data_t *) userRing->trans_data;
            if (!ringData)
            {
                ADF_ERROR("Coudn't get ring data\n");
                ICP_MUTEX_UNLOCK(&procList->procListLock);
                return CPA_STATUS_FAIL;
            }

            if (NULL != ringData->callbackFn)
            {
                ringData->callbackFn = orphan_callback;
                ICP_ADD_ELEMENT_TO_END_OF_LIST(userRing,
                                 list->response_rings_list_tail,
                                 list->response_rings_list_head);
            }
            else
            {
                ringInfoCb cb = (ringInfoCb) pAccelDev->ringInfoCallBack;
                Cpa32U ringNumber = ringData->ringNumber +
                        (ringData->bankNumber * ICP_ETR_MAX_RINGS_PER_BANK);

                ICP_ADD_ELEMENT_TO_END_OF_LIST(userRing,
                                 list->request_rings_list_tail,
                                 list->request_rings_list_head);
                 /*
                  * send ring disable message
                  * Don't disable ring for dyn
                  */
                if ((NULL != cb) && (CPA_FALSE == userRing->is_dyn)
                           && (CPA_FALSE == userRing->is_wireless))
                {
                        cb(pAccelDev, ringNumber,
                           ICP_ADF_RING_DISABLE,
                           ringData->info);
                }
            }

            ringData->userSpacePrivateData = NULL;
            /*
             * mark the ring as orphan so the ETring mgr knows that
             * there is no one waiting for the response
             */
            ringData->orphanRing = CPA_TRUE;
            osalAtomicDec(&pAccelDev->userspaceRingsCounter);
            have_rings = 1;
        }

        if (have_rings)
        {
            /* add a new entry to the orphan list */
            ICP_SPINLOCK_LOCK(&orphan_list_lock);
            ICP_ADD_ELEMENT_TO_END_OF_LIST(list, orphans_list_tail,
                                           orphans_list_head);
            ICP_SPINLOCK_UNLOCK(&orphan_list_lock);
            SET_STATUS_BIT(pAccelDev->adfSubsystemStatus,
                               ADF_STATUS_ORPHAN_TH_RUNNING);

            /* kick off a thread that will clean the list
             * if it is not already running */
            if(osalAtomicDecAndTest(&thread_running_flag))
            {
                /* Make sure ADF could not be rmmod till the thread
                 * will finish - otherwise do not even start it */
                if(adf_get_module())
                {
                    thread_attr.name = (char*)thread_name;
                    osalThreadCreate(&thread, &thread_attr,
                               (OsalVoidFnVoidPtr)clean_orphan_list, NULL);
                    osalThreadStart(&thread);
                }
                else
                {
                    /* We are in real trouble now - cannot rm the rings as
                     * AE may write back to it and if we don't clean them
                     * we will have mem leak */
                    ADF_ERROR("Can not increase module usage counter.\n");
                    osalAtomicSet(1, &thread_running_flag);
                    stat = CPA_STATUS_FAIL;
                    CLEAR_STATUS_BIT(pAccelDev->adfSubsystemStatus,
                               ADF_STATUS_ORPHAN_TH_RUNNING);
                }
            }
        }
        while (pDynInstanceList && pDynInstanceList->listHead)
        {
            pDynHandle = pDynInstanceList->listTail;
            /*
             * Can not just remove dyn instance as there can be some
             * requests in flight so first remove the dyn instance
             * from process dyn instance list
             */
            icp_adf_putDynInstance(pAccelDev, pDynHandle->stype,
                                     pDynHandle->instance_id);
            ICP_REMOVE_ELEMENT_FROM_LIST(pDynHandle,
                                         pDynInstanceList->listTail,
                                         pDynInstanceList->listHead);
            ICP_FREE(pDynHandle);
        }

        /* Remove the process from the processes list */
        ICP_FREE(ringList);
        ICP_FREE(pDynInstanceList);
        ICP_REMOVE_ELEMENT_FROM_LIST(pUserProc,
                       procList->listTail, procList->listHead);

        ICP_MUTEX_UNLOCK(&procList->procListLock);
        if (OSAL_SUCCESS !=
                osalSemaphoreDestroy (&pUserProc->processDisconnectedFlag))
        {
            ADF_ERROR("Semaphore destroy failed\n");
        }
        ICP_FREE(pUserProc);
        *proxyPrivateData = NULL;
    }
    else
    {
        ADF_ERROR("User process disconnect - invalid param\n");
        stat = CPA_STATUS_FAIL;
    }
    return stat;
}

/*
 * user_proxy_EventHandler - User proxy event dispatcher
 * Call appropriate event handler function of user proxy module
 */
STATIC CpaStatus user_proxy_EventHandler(icp_accel_dev_t *accel_dev,
                              icp_adf_subsystemEvent_t event, void* param)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;

    switch(event)
    {
        case ICP_ADF_EVENT_INIT :
            stat = adf_user_proxyEventInit(accel_dev, param);
            break;
        case ICP_ADF_EVENT_START :
            stat = adf_user_proxyEventStart(accel_dev);
            break;
        case ICP_ADF_EVENT_STOP :
            stat = adf_user_proxyEventStop(accel_dev);
            break;
        case ICP_ADF_EVENT_SHUTDOWN :
            stat = adf_user_proxyEventShutdown(accel_dev);
            break;
        case ICP_ADF_EVENT_RESTARING:
            stat = adf_user_proxyEventRestarting(accel_dev);
            break;
        default:
            stat = CPA_STATUS_SUCCESS;
    }
    return stat;
}

/*
 * adf_userSpaceRegister
 * Register user proxy event dispatcher into ADF
 */
CpaStatus adf_userSpaceRegister(void)
{
    ICP_SPINLOCK_INIT(&orphan_list_lock);
    osalAtomicSet(1, &thread_running_flag);

    user_space_reg_handle.subsystem_name  = subsystem_name;
    user_space_reg_handle.subserviceEventHandler = user_proxy_EventHandler;
    return icp_adf_subsystemRegister(&user_space_reg_handle);
}

/*
 * adf_userSpaceUnregister
 * Unregister user proxy event dispatcher from ADF
 */
CpaStatus adf_userSpaceUnregister(void)
{
    ICP_SPINLOCK_UNINIT(&orphan_list_lock);
    return icp_adf_subsystemUnregister(&user_space_reg_handle);
}
