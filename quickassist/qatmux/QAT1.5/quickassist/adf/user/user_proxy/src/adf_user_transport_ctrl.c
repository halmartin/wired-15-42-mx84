/*****************************************************************************
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

/*****************************************************************************
 * @file adf_user_transport_ctrl.c
 *
 * @description
 *      Transport Controller for user space
 *
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_platform.h"
#include "adf_dev_ring_ctl.h"
#include "adf_user_transport.h"
#include "icp_adf_init.h"
#include "adf_devmgr.h"
#include "adf_cfg.h"

/*
 * Time to wait for a thread to exit in ms
 */
#define TIME_TO_WAIT 50
/*
 * Value indicating not to bind a thread to a core.
 */
#define DO_NOT_BIND 0xFFFF
/*
 * Passing thread parameter
 */

typedef union {
    Cpa64U  dp_inflight;
    OsalAtomic trad_inflight;
} adf_ring_flight_t;

STATIC adf_dev_bank_handle_t *bankHandles[ADF_MAX_DEVICES] = {NULL};
STATIC adf_dev_ring_handle_t **ringHandles[ADF_MAX_DEVICES] = {NULL};
STATIC adf_ring_flight_t *ringInflights[ADF_MAX_DEVICES] = {NULL};

/*
 * This macro was created to make the adf_proxy_populate_device_info
 * shorter and to make freeing allocating memory in one place
 */
#define BREAK_ON_NULL(val, msg, stat)    \
    if (NULL == val) {                   \
        ADF_ERROR(msg);                  \
        stat = CPA_STATUS_FAIL;          \
        break;                           \
    }

#ifndef ICP_WITHOUT_THREAD
typedef struct pass_thread_param_s
{
    icp_accel_dev_t       *accel_dev;
    adf_dev_ring_handle_t *ring_handl;
    Cpa32U                ring_number;
    Cpa32U                cpu_affinity;
} pass_thread_param_t;

/*****************************************************************************
 * STATIC VARIABLES
 ****************************************************************************/

/* One msg reading thread per device */
STATIC pthread_t reading_threads[ADF_MAX_DEVICES] = {'\0'};
/* One msg passing thread per Rx ring. This thread will call user's callback*/
STATIC pthread_t *passing_threads[ADF_MAX_DEVICES] = {NULL};
STATIC uint8_t threads_ctr[ADF_MAX_DEVICES] = {'\0'};
STATIC uint8_t *pass_threads_flag[ADF_MAX_DEVICES] = {NULL};
STATIC pthread_mutex_t *mutex[ADF_MAX_DEVICES] = {NULL};
STATIC pthread_cond_t  *condition[ADF_MAX_DEVICES] = {NULL};
STATIC pass_thread_param_t *params[ADF_MAX_DEVICES] = {NULL};
STATIC CpaBoolean *messageReceived[ADF_MAX_DEVICES] = {NULL};

/*****************************************************************************
 * TRANSPORT HANDLE FUNCTIONS
 ****************************************************************************/

/*
 * adf_proxy_pass_message
 * The function runs in a separate thread (one thread per Rx ring)
 * and waits for the "mesage get" thread to read ring number.
 * Once a ring number passed from kernel space the reading thread wakeups
 * the passing thread for that ring, which calls the function
 * that reads the messages on the ring and notifies the client
 */
STATIC void* adf_proxy_pass_message(void* param)
{
    int ret = 0, ret_lock=0, ret_unlock=0;
    struct sched_param sp;
    cpu_set_t cpu_affinity;
    pass_thread_param_t *th_param = (pass_thread_param_t*) param;
    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*) th_param->accel_dev;
    Cpa32U dev_id = accel_dev->accelId;
    Cpa32U ring_nr = th_param->ring_number;

    /* Set thread scheduling priority */
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    if(sched_setscheduler(0, SCHED_FIFO, &sp))
    {
        ADF_ERROR("sched_setscheduler failed. Exiting\n");
        pthread_exit((void*)0);
    }

    /* Set thread core affinity */
    if(DO_NOT_BIND != th_param->cpu_affinity)
    {
        ADF_DEBUG("Set thread core affinity to CPU%d\n",
                                               th_param->cpu_affinity);
        CPU_ZERO(&cpu_affinity);
        CPU_SET(th_param->cpu_affinity, &cpu_affinity);
        if(sched_setaffinity(0, sizeof(cpu_affinity), &cpu_affinity))
        {
            ADF_ERROR("sched_setaffinity failed. Exiting\n");
            pthread_exit((void*)0);
        }
    }

    /* Run the thread main loop */
    while(pass_threads_flag[dev_id][ring_nr])
    {
        /* Wait if no message has been received */
        ret_lock = pthread_mutex_lock(&(mutex[dev_id][ring_nr]));
        if (CPA_FALSE == messageReceived[dev_id][ring_nr])
        {
            ret = pthread_cond_wait(&condition[dev_id][ring_nr],
                        &mutex[dev_id][ring_nr]);
        }
        messageReceived[dev_id][ring_nr] = CPA_FALSE;
        ret_unlock = pthread_mutex_unlock(&(mutex[dev_id][ring_nr]));
        if(ret || ret_lock  || ret_unlock)
        {
            pthread_exit((void*)0);
        }
        if(!pass_threads_flag[dev_id][ring_nr])
        {
            pthread_exit((void*)0);
        }
        /* Notify client */
        adf_user_notify_msgs(th_param->ring_handl);
    }
    pthread_exit((void*)0);
}

/*
 * adf_proxy_get_message
 * The function runs in a separate thread (one thread per device)
 * and calls the blocking read. Once the read returns we should get
 * a mask of rings where there is data to be handled. For each ring
 * we call the passing thread to process data and notify the user.
 */
STATIC void* adf_proxy_get_message(void* dev)
{
    icp_accel_dev_t *accel_dev = (icp_accel_dev_t*) dev;
    Cpa32U accelId = accel_dev->accelId;
    struct sched_param sp;
    int _read = 0;
    void *ptr = NULL;
    adf_dev_rings_mask_t *rings_mask = NULL;
    int rings_mask_size = 0;
    Cpa32U bank = 0, ring = 0, ring_num = 0;

    rings_mask_size = sizeof(adf_dev_rings_mask_t) +
                        (sizeof(Cpa32U) * accel_dev->maxNumBanks);
    ptr = ICP_MALLOC_GEN(rings_mask_size);
    if(NULL == ptr)
    {
        ADF_ERROR("unable to allocate ptr (rings_mask_size) buffer\n");
        pthread_exit((void*)0);
    }

    ICP_MEMSET(ptr, 0, rings_mask_size);

    /* Set thread scheduling priority */
    sp.sched_priority = sched_get_priority_max(SCHED_RR);
    if(sched_setscheduler(0, SCHED_RR, &sp))
    {
        ADF_ERROR("sched_setscheduler failed. Exiting\n");
        ICP_FREE(ptr);
        pthread_exit((void*)0);
    }

    while(threads_ctr[accelId])
    {
        _read = ICP_READ(accel_dev->ringFileHdl, ptr, rings_mask_size);

        rings_mask = (adf_dev_rings_mask_t *)ptr;
        rings_mask->bank_irq_mask =
            (Cpa32U *)((unsigned char*)ptr + sizeof(adf_dev_rings_mask_t));

        if ((_read == rings_mask_size) &&
                    (rings_mask->bank_mask) && (rings_mask->ring_mask))
        {
            for (bank = 0; bank < accel_dev->maxNumBanks; bank++)
            {
                if (rings_mask->bank_mask & 1 << bank)
                {
                    for (ring = 0; ring < accel_dev->maxNumRingsPerBank; ring++)
                    {
                        if (rings_mask->ring_mask & 1 << ring)
                        {
                            ring_num = ring + (bank *
                                    accel_dev->maxNumRingsPerBank);
                            bankHandles[accelId][bank].interrupt_mask =
                                                rings_mask->bank_irq_mask[bank];
                            /* Notify the passing thread */
                            pthread_mutex_lock(&mutex[accelId][ring_num]);
                            messageReceived[accelId][ring_num] = CPA_TRUE;
                            pthread_cond_signal(&condition[accelId][ring_num]);
                            pthread_mutex_unlock(&mutex[accelId][ring_num]);
                        }
                    }
                }
            }
        }
        else if (EAGAIN == _read)
        {
            continue;
        }
        else
        {
            /* Exit the thread */
            threads_ctr[accelId] = 0;
            ICP_FREE(ptr);
            pthread_exit((void*)0);
        }
    }
    ICP_FREE(ptr);
    pthread_exit((void*)0);
}

STATIC CpaStatus
adf_proxy_populate_threads_info(icp_accel_dev_t *accel_dev)
{
    union {
        pthread_t              *passing_thread;
        uint8_t                *pass_thread_flag;
        pthread_mutex_t        *mutex;
        pthread_cond_t         *condition;
        pass_thread_param_t    *param;
        CpaBoolean             *messageReceived;
    } ptr;
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32U numOfRingsPerDevice = 0;
    Cpa32U size = 0;
    Cpa32U device_id = 0;

    device_id = accel_dev->accelId;
    numOfRingsPerDevice = accel_dev->maxNumBanks *
        accel_dev->maxNumRingsPerBank;

    do {
        /* allocate passing threads array */
        size = sizeof (pthread_t) * numOfRingsPerDevice;
        ptr.passing_thread = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.passing_thread,
            "Failed to allocate memory - passing_thread\n", status)
        ICP_MEMSET(ptr.passing_thread, 0, size);
        passing_threads[device_id] = ptr.passing_thread;

        /* allocate pass threads flag array */
        size = sizeof (uint8_t) * numOfRingsPerDevice;
        ptr.pass_thread_flag = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.pass_thread_flag,
            "Failed to allocate memory - pass_thread_flag\n", status)
        ICP_MEMSET(ptr.pass_thread_flag, 0, size);
        pass_threads_flag[device_id] = ptr.pass_thread_flag;

        /* allocate mutex array */
        size = sizeof (pthread_mutex_t) * numOfRingsPerDevice;
        ptr.mutex = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.mutex,
            "Failed to allocate memory - mutex\n", status)
        ICP_MEMSET(ptr.mutex, 0, size);
        mutex[device_id] = ptr.mutex;

        /* allocate conditions */
        size = sizeof (pthread_cond_t) * numOfRingsPerDevice;
        ptr.condition = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.condition,
            "Failed to allocate memory - condition\n", status)
        ICP_MEMSET(ptr.condition, 0, size);
        condition[device_id] = ptr.condition;

        /* allocate params array */
        size = sizeof (pass_thread_param_t) * numOfRingsPerDevice;
        ptr.param = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.param,
            "Failed to allocate memory - param\n", status)
        ICP_MEMSET(ptr.param, 0, size);
        params[device_id] = ptr.param;

        /* allocate message received array */
        size = sizeof (CpaBoolean) * numOfRingsPerDevice;
        ptr.messageReceived = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.messageReceived,
            "Failed to allocate memory - messageReceived\n", status)
        ICP_MEMSET(ptr.messageReceived, 0, size);
        messageReceived[device_id] = ptr.messageReceived;

        status = CPA_STATUS_SUCCESS;

    } while(0);
    if (CPA_STATUS_FAIL == status) {
        if (passing_threads[device_id])
            ICP_FREE(passing_threads[device_id]);
        if (pass_threads_flag[device_id])
            ICP_FREE(pass_threads_flag[device_id]);
        if (mutex[device_id])
            ICP_FREE(mutex[device_id]);
        if (condition[device_id])
            ICP_FREE(condition[device_id]);
        if (params[device_id])
            ICP_FREE(params[device_id]);
        if (messageReceived[device_id])
            ICP_FREE(messageReceived[device_id]);
    }
    return status;
}

STATIC void
adf_proxy_depopulate_threads_info(icp_accel_dev_t *accel_dev)
{
    Cpa32U device_id = 0;

    device_id = accel_dev->accelId;
    ICP_FREE(passing_threads[device_id]);
    ICP_FREE(pass_threads_flag[device_id]);
    ICP_FREE(mutex[device_id]);
    ICP_FREE(condition[device_id]);
    ICP_FREE(params[device_id]);
    ICP_FREE(messageReceived[device_id]);
    return;
}

#else

STATIC CpaStatus
adf_proxy_populate_threads_info(icp_accel_dev_t *accel_dev)
{
    return CPA_STATUS_SUCCESS;
}

STATIC void
adf_proxy_depopulate_threads_info(icp_accel_dev_t *accel_dev)
{
    return;
}
#endif

STATIC CpaStatus
adf_proxy_populate_bank_ring_info(icp_accel_dev_t *accel_dev)
{
    union {
        adf_dev_bank_handle_t  *bankHandler;
        adf_dev_ring_handle_t  **ringHandler;
        adf_ring_flight_t      *inflight;
    } ptr;
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32U numOfRingsPerDevice = 0;
    Cpa32U numOfBanksPerDevice = 0;
    Cpa32U size = 0;
    Cpa32U device_id = 0;
    void *currentPtr = NULL;

    device_id = accel_dev->accelId;
    numOfRingsPerDevice = accel_dev->maxNumBanks *
        accel_dev->maxNumRingsPerBank;
    numOfBanksPerDevice = accel_dev->maxNumBanks;

    do {
        /* allocate bank handler array */
        size = sizeof (adf_dev_bank_handle_t) * numOfBanksPerDevice;
        ptr.bankHandler = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.bankHandler,
            "Failed to allocate memory - bankHandler\n",  status)
        ICP_MEMSET(ptr.bankHandler, 0, size);
        bankHandles[device_id] = ptr.bankHandler;

        /* allocate ring handler array */
        size = sizeof (adf_dev_ring_handle_t*) * numOfRingsPerDevice;
        ptr.ringHandler = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.ringHandler,
            "Failed to allocate memory - ringHandler\n", status)
        ICP_MEMSET(ptr.ringHandler, 0, size);
        ringHandles[device_id] = ptr.ringHandler;

        /* allocate ring inflight array ring put/get optimization*/
        size = sizeof(adf_ring_flight_t) *
            (accel_dev->maxNumRingsPerBank >> 1) * numOfBanksPerDevice;
        ptr.inflight = ICP_MALLOC_GEN(size);
        BREAK_ON_NULL(ptr.inflight,
            "Failed to allocate memory - ringInflights\n", status)
        ringInflights[device_id] = ptr.inflight;
        status = CPA_STATUS_SUCCESS;
    } while(0);

    if (CPA_STATUS_FAIL == status) {
        if (bankHandles[device_id])
            ICP_FREE(bankHandles[device_id]);
        if (ringHandles[device_id])
            ICP_FREE(ringHandles[device_id]);
        if (ringInflights[device_id])
        {
            currentPtr = (void*)ringInflights[device_id];
            ICP_FREE(currentPtr);
        }
    }
    return status;
}

STATIC CpaStatus
adf_proxy_populate_device_info(icp_accel_dev_t *accel_dev)
{
    if(adf_proxy_populate_bank_ring_info(accel_dev)
            != CPA_STATUS_SUCCESS)
        return CPA_STATUS_FAIL;
    if(adf_proxy_populate_threads_info(accel_dev)
            != CPA_STATUS_SUCCESS)
        return CPA_STATUS_FAIL;

    return CPA_STATUS_SUCCESS;
}

void
adf_proxy_depopulate_bank_ring_info(icp_accel_dev_t *accel_dev)
{
    Cpa32U device_id = 0;
    void *currentPtr = NULL;

    device_id = accel_dev->accelId;
    ICP_FREE(bankHandles[device_id]);
    ICP_FREE(ringHandles[device_id]);
    currentPtr = (void*)ringInflights[device_id];
    if (currentPtr)
    {
        ICP_FREE(currentPtr);
    }
    return;
}

void
adf_proxy_depopulate_device_info(icp_accel_dev_t *accel_dev)
{
    adf_proxy_depopulate_bank_ring_info(accel_dev);
    adf_proxy_depopulate_threads_info(accel_dev);
    return;
}

/*
 * Create a transport handle
 * The function sends ioctl request to adf user proxy to create
 * a ring and then mmaps it to userspace memory. If it is a response
 * ring and there is no reading thread running for the device
 * the function creates one.
 */
CpaStatus icp_adf_transCreateHandle(icp_accel_dev_t *accel_dev,
                                    icp_transport_type trans_type,
                                    const char *section,
                                    const Cpa32U accel_nr,
                                    const Cpa32U bank_nr,
                                    const char *service_name,
                                    const icp_adf_ringInfoService_t info,
                                    icp_trans_callback callback,
                                    icp_resp_deliv_method resp,
                                    const Cpa32U size,
                                    icp_comms_trans_handle* trans_handle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    int ret = 0;
    adf_dev_ring_handle_t *pRingHandle = NULL;
    Cpa32U ring_number = 0;
    Cpa32U in_flight_index=0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);

    /* allocate and setup ring handle structure */
    pRingHandle = ICP_MALLOC_GEN(sizeof(adf_dev_ring_handle_t));
    if (NULL == pRingHandle)
    {
        ADF_ERROR("unable to allocate pRingHandle buffer\n");
        return CPA_STATUS_FAIL;
    }
    memset(pRingHandle, 0, sizeof(adf_dev_ring_handle_t));
    pRingHandle->accel_dev = accel_dev;
    pRingHandle->trans_type = trans_type;
    pRingHandle->service_name = ICP_MALLOC_GEN(ICP_STRLEN(service_name)+1);
    if(NULL == pRingHandle->service_name)
    {
        ADF_ERROR("unable to allocate service buffer\n");
        ICP_FREE(pRingHandle);
        return CPA_STATUS_FAIL;
    }
    memcpy(pRingHandle->service_name, service_name,
                                      ICP_STRLEN(service_name) + 1);
    pRingHandle->service_name_len = ICP_STRLEN(service_name) + 1;
    pRingHandle->callback = callback;
    pRingHandle->accel_num = accel_nr;
    pRingHandle->bank_num = bank_nr;
    pRingHandle->section_name = ICP_MALLOC_GEN(ICP_STRLEN(section) + 1);
    if(NULL == pRingHandle->section_name)
    {
        ADF_ERROR("unable to allocate section name buffer\n");
        ICP_FREE(pRingHandle->service_name);
        ICP_FREE(pRingHandle);
        return CPA_STATUS_FAIL;
    }
    memcpy(pRingHandle->section_name, section,
                                              ICP_STRLEN(section) + 1);
    pRingHandle->section_name_len = ICP_STRLEN(section) + 1;
    pRingHandle->resp = resp;
    pRingHandle->info = info;
    /* Zero the masks to get rid of garbage. */
    pRingHandle->interrupt_user_mask = 0;
    pRingHandle->interrupt_kernel_mask = 0;
    pRingHandle->pollingMask = 0;
    /*
     * ring_size going down will be in number of messages (64bytes)
     * and ring_size when it will come back will by ring size in bytes
     * that will be used to allocate appropriate size for the ring (mmap)
     */
    pRingHandle->ring_size = size;
    /* send the create handle request to kernel space  */
    ret = ioctl(accel_dev->ringFileHdl,
                ADF_DEV_RING_IOC_CREATEHANDLE, pRingHandle);
    if (ret != 0) {
        ADF_ERROR("ioctl failed, ret=%d\n", ret);
        ICP_FREE(pRingHandle->service_name);
        pRingHandle->service_name = NULL;
        ICP_FREE(pRingHandle->section_name);
        pRingHandle->section_name = NULL;
        ICP_FREE(pRingHandle);
        return CPA_STATUS_FAIL;
    }
    pRingHandle->accel_dev = accel_dev;
    *trans_handle = (icp_comms_trans_handle *)pRingHandle;
    status = icp_adf_transGetRingNum(*trans_handle, &ring_number);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("icp_adf_transGetRingNum failed\n");
        icp_adf_transReleaseHandle(*trans_handle);
        *trans_handle = NULL;
        return CPA_STATUS_FAIL;
    }
    /* mmap the ring memory into userspace so it can be accessed directly
     * in userspace as we will write and read to/from it  */
    pRingHandle->ring_virt_addr = ICP_MMAP(0,
                                       pRingHandle->ring_size,
                                       PROT_READ|PROT_WRITE,
                                       MAP_FILE|MAP_SHARED|MAP_LOCKED,
                                       accel_dev->ringFileHdl,
                                       ring_number * getpagesize());
    if (MAP_FAILED == pRingHandle->ring_virt_addr)
    {
        ADF_ERROR("mmap failed\n");
        icp_adf_transReleaseHandle(*trans_handle);
        *trans_handle = NULL;
        return CPA_STATUS_FAIL;
    }

#ifndef ICP_WITHOUT_THREAD
    /* Kick of the reading threads for userspace
     * if the ring in a response ring */
    if((NULL != callback ) && (ICP_RESP_TYPE_IRQ == resp))
    {
        if(threads_ctr[accel_dev->accelId] == 0)
        {
            threads_ctr[accel_dev->accelId]++;
            ret = pthread_create( &reading_threads[accel_dev->accelId],
                              NULL, adf_proxy_get_message,
                              (void*) accel_dev);
            if(ret)
            {
                ADF_ERROR("pthread_create failed, errno = %d\n", ret);
                icp_adf_transReleaseHandle(*trans_handle);
                *trans_handle = NULL;
                return CPA_STATUS_FAIL;
            }
        }
        else
        {
            threads_ctr[accel_dev->accelId]++;
        }
        pthread_mutex_init(&(mutex[accel_dev->accelId][ring_number]), NULL);
        ret = pthread_cond_init(&(condition[accel_dev->accelId][ring_number]),
                                NULL);
        if(ret)
        {
            ADF_ERROR("pthread_cond_init failed, errno = %d\n", ret);
            icp_adf_transReleaseHandle(*trans_handle);
            *trans_handle = NULL;
            return CPA_STATUS_FAIL;
        }
        else
        {
            /* Set thread core affinity */
            char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
            char section[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
            char key[ADF_CFG_MAX_KEY_LEN_IN_BYTES] = {0};
            snprintf(section, ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
                     ACCEL_STR, accel_nr);
            snprintf(key, ADF_CFG_MAX_KEY_LEN_IN_BYTES,
                     ADF_ETRMGR_COREID_AFFINITY_FORMAT, bank_nr);

            status = icp_adf_cfgGetParamValue(accel_dev,
                                      section, key, val);
            if(CPA_STATUS_SUCCESS == status)
            {
                params[accel_dev->accelId][ring_number].cpu_affinity =
                  (Cpa32U)ICP_STRTOUL(val, NULL, ADF_CFG_BASE_DEC);
            }
            else
            {
                ADF_ERROR("Can't find core affinity settings for accel "
                          " %d bank %d\n", accel_nr, bank_nr);
                /* if can't find a valid settings for core affinity
                 * let it run wherever OS wan't to run it */
                params[accel_dev->accelId][ring_number].cpu_affinity =
                                                                  DO_NOT_BIND;
            }
        }
        params[accel_dev->accelId][ring_number].accel_dev = accel_dev;
        params[accel_dev->accelId][ring_number].ring_number = ring_number;
        params[accel_dev->accelId][ring_number].ring_handl = pRingHandle;
        pass_threads_flag[accel_dev->accelId][ring_number] = 1;

        /* Create message passing thread. One thread per ring to separate
         * irq reading thread from callback from. Who knows what user is going
         * to do in the callback. */
        ret = pthread_create(&passing_threads[accel_dev->accelId][ring_number],
                             NULL, adf_proxy_pass_message,
                             (void*)&params[accel_dev->accelId][ring_number]);
        if(ret)
        {
            ADF_ERROR("pthread_create failed, errno = %d\n", ret);
            icp_adf_transReleaseHandle(*trans_handle);
            *trans_handle = NULL;
            return CPA_STATUS_FAIL;
        }
    }

    messageReceived[accel_dev->accelId][ring_number] = CPA_FALSE;
#else
    if((NULL != callback ) && (ICP_RESP_TYPE_IRQ == resp))
    {
            ADF_ERROR("Can't create IRQ driven rings in IRQless mode\n", ret);
            icp_adf_transReleaseHandle(*trans_handle);
            *trans_handle = NULL;
            return CPA_STATUS_FAIL;
    }
#endif

    /* callback has been overwritten in kernelspace
     * so have to set it to the userspace callback again */
    pRingHandle->callback = callback;
    pRingHandle->bank_data =
      &bankHandles[accel_dev->accelId][pRingHandle->bank_num];
    ringHandles[accel_dev->accelId][ring_number] = pRingHandle;
    bankHandles[accel_dev->accelId]
        [pRingHandle->bank_num].interrupt_mask |=
                                             pRingHandle->interrupt_user_mask;
    bankHandles[accel_dev->accelId]
        [pRingHandle->bank_num].bank_number =pRingHandle->bank_num;
    bankHandles[accel_dev->accelId]
        [pRingHandle->bank_num].timed_coalesc_enabled =
                                           pRingHandle->timed_coalesc_enabled;
    bankHandles[accel_dev->accelId]
        [pRingHandle->bank_num].number_msg_coalesc_enabled =
                                      pRingHandle->number_msg_coalesc_enabled;
    bankHandles[accel_dev->accelId]
        [pRingHandle->bank_num].pollingMask |=
                                             pRingHandle->pollingMask;

    /* request and response ring will share the same index */
    in_flight_index = ((accel_dev->maxNumRingsPerBank *
            pRingHandle->bank_num) >> 1) + (pRingHandle->ring_num >> 1);
    /* Initialise the pRingHandle inflight */
    /* Take it as dp_inflight as only thread is used for initalization*/
    pRingHandle->dp_in_flight = (Cpa64U*)ringInflights[accel_dev->accelId] +
                                         in_flight_index;
    *pRingHandle->dp_in_flight = 0;
    /* Initialise the pRingHandle atomic flag. */
    osalAtomicSet(1, (OsalAtomic*)&pRingHandle->pollingInProgress);
    pRingHandle->user_lock = ICP_MALLOC_GEN(sizeof(ICP_MUTEX));
    if(!pRingHandle->user_lock)
    {
            ADF_ERROR("Could not alloc memory for ring lock\n");
            icp_adf_transReleaseHandle(*trans_handle);
            *trans_handle = NULL;
            return CPA_STATUS_FAIL;
    }
    ICP_MUTEX_INIT(pRingHandle->user_lock);
    return CPA_STATUS_SUCCESS;
}

/*
 * Release a transport handle
 * The function sends ioctl request to adf user proxy to release a ring
 */
CpaStatus icp_adf_transReleaseHandle(icp_comms_trans_handle trans_handle)
{
    int ret = 0;
    icp_accel_dev_t *accel_dev = NULL;
    adf_dev_ring_handle_t *pRingHandle = NULL;
    Cpa32U ring_number;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    pRingHandle = (adf_dev_ring_handle_t *)trans_handle;
    ICP_CHECK_FOR_NULL_PARAM(pRingHandle->accel_dev);
    accel_dev = pRingHandle->accel_dev;
    ret = icp_adf_transGetRingNum(pRingHandle, &ring_number);
    if (CPA_STATUS_SUCCESS != ret) {
        ADF_ERROR("icp_adf_transGetRingNum failed\n");
        return CPA_STATUS_FAIL;
    }

#ifndef ICP_WITHOUT_THREAD
    if ((NULL != pRingHandle->callback) &&
              (ICP_RESP_TYPE_IRQ == pRingHandle->resp)) {
        threads_ctr[accel_dev->accelId]--;
        pass_threads_flag[accel_dev->accelId][ring_number] = 0;
        pthread_mutex_lock(&mutex[accel_dev->accelId][ring_number]);
        pthread_cond_signal(&condition[accel_dev->accelId]
                            [ring_number]);
        pthread_mutex_unlock(&mutex[accel_dev->accelId][ring_number]);
    }
#endif
    /* update user process IRQ mask
     * Everytime userspace ring gets a message it reads it from the ring
     * and as the last step needs to enable the IRQ for the ring so
     * the driver could get notifications that there is data on the ring.
     * So this is important to keep the IRQ mask up to date */
    bankHandles[accel_dev->accelId][pRingHandle->bank_num].interrupt_mask &=
       (~(1 << pRingHandle->ring_num));
    bankHandles[accel_dev->accelId][pRingHandle->bank_num].pollingMask &=
      (~(1 << pRingHandle->ring_num));
    /* send the request down to the kernel.
     * NOTE: Don't send release_handle if the kernel proxy is not running
     * The proxy down there will cleanup the rings anyway.
     * */
    if (accel_dev->adfSubsystemStatus) {
        ret = ioctl(accel_dev->ringFileHdl,
                ADF_DEV_RING_IOC_RELEASEHANDLE, pRingHandle);
        if (ret != 0) {
            ADF_ERROR("ioctl failed, ret=%d\n", ret);
        }
    }
    ICP_MUNMAP(pRingHandle->ring_virt_addr, pRingHandle->ring_size);
    pRingHandle->ring_virt_addr = NULL;

    if (NULL != pRingHandle->service_name) {
        ICP_FREE( pRingHandle->service_name );
        pRingHandle->service_name = NULL;
        ICP_FREE( pRingHandle->section_name );
        pRingHandle->section_name = NULL;
    }
#ifndef ICP_WITHOUT_THREAD
    if ((NULL != pRingHandle->callback) &&
       (ICP_RESP_TYPE_IRQ == pRingHandle->resp)) {
        /* Wait for the thread to exit */
        ICP_MSLEEP(TIME_TO_WAIT);
        pthread_mutex_destroy(&mutex[accel_dev->accelId]
                              [ring_number]);
        pthread_cond_destroy (&condition[accel_dev->accelId]
                              [ring_number]);
    }
#endif
    if (NULL != pRingHandle->user_lock) {
        ICP_MUTEX_UNINIT(pRingHandle->user_lock);
        ICP_FREE(pRingHandle->user_lock);
        pRingHandle->user_lock = NULL;
    }
    ICP_FREE(pRingHandle);
    pRingHandle = NULL;
    ringHandles[accel_dev->accelId][ring_number] = NULL;
    return CPA_STATUS_SUCCESS;
}

/*
 * Returns ring number for the trans handle
 */
CpaStatus icp_adf_transGetRingNum(icp_comms_trans_handle trans_handle,
                                  Cpa32U *ringNum)
{
    adf_dev_ring_handle_t *pRingHandle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    pRingHandle = (adf_dev_ring_handle_t *)trans_handle;
    *ringNum = (pRingHandle->bank_num * ICP_ETR_MAX_RINGS_PER_BANK)
                                            + pRingHandle->ring_num;
    return CPA_STATUS_SUCCESS;
}

/*
 * Put a message on the transport handle
 */
CpaStatus icp_adf_transPutMsg(icp_comms_trans_handle trans_handle,
                              Cpa32U *inBuf,
                              Cpa32U bufLen)
{
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t*)trans_handle;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    return adf_user_put_msgs(pRingHandle, inBuf, bufLen);
}


/*
 * adf_user_unmap_rings
 * Device is going down - unmap all rings allocated for this device
 */
CpaStatus adf_user_unmap_rings(icp_accel_dev_t *accel_dev )
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_dev_ring_handle_t *pRingHandle = NULL;
    int i = 0, ret = 0;
    Cpa32U maxNumRingsPerDevice = 0;

    maxNumRingsPerDevice =
        accel_dev->maxNumRingsPerBank * accel_dev->maxNumBanks;

#ifndef ICP_WITHOUT_THREAD
    threads_ctr[accel_dev->accelId] = 0;
#endif
    for(i = 0; i < maxNumRingsPerDevice; i++)
    {
        pRingHandle = ringHandles[accel_dev->accelId][i];
        if (pRingHandle && pRingHandle->ring_virt_addr) {
            ret = munmap( pRingHandle->ring_virt_addr,
                    pRingHandle->ring_size );
            if (ret) {
                ADF_ERROR("Failed unmapping ring memory\n");
                stat = CPA_STATUS_FAIL;
            }
            pRingHandle->ring_virt_addr = NULL;
        }
    }
    return stat;
}

/*
 * Internal functions which performs all the
 * tasks necessary to poll a response ring.
 */
CpaStatus adf_pollRing(icp_accel_dev_t *accel_dev,
                       adf_dev_ring_handle_t *pRingHandle,
                       Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_RETRY;

    /* Check to see if this ring is already being polled by
     * another core or thread. DecAndTest returns TRUE
     * only if pRingHandle->pollingInProgress was previously
     * equal to one and then sets the var to zero. While
     * pRingHandle->pollingInProgress is still zero no other
     * thread will be able to poll. pollingInProgress is
     * reset to one once the notify function is done.
     */
    if (osalAtomicDecAndTest((OsalAtomic*)&pRingHandle->pollingInProgress)) {
        /*Set the ring response quota.*/
        pRingHandle->ringResponseQuota = response_quota;
        status = adf_user_notify_msgs_poll(pRingHandle);
        osalAtomicSet(1, (OsalAtomic*)&pRingHandle->pollingInProgress);
    }
    return status;
}
/*
 * This function allows the user to poll the response rings of a given
 * bank to determine if any of the rings have messages that need to be
 * read. This method is used as an alternative to reading messages
 * via the ISR method.
 * N.B. response quota is per ring.
 */
CpaStatus icp_sal_pollBank(Cpa32U accelId,
                           Cpa32U bank_number,
                           Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_RETRY;
    icp_accel_dev_t *accel_dev = NULL;
    adf_dev_bank_handle_t *bank=NULL;
    adf_dev_ring_handle_t *pRingHandle = NULL;
    Cpa8U* csr_base_addr = NULL;
    Cpa32U csrVal=0;
    Cpa32U ring_number=0, ringnum_in_bank=0;
    Cpa32U stat_total=0;

    /* Find the accel device associated with the accelId
     * passed in.
     */
    accel_dev = adf_devmgrGetAccelDevByAccelId(accelId);
    if(!accel_dev) {
        ADF_ERROR("There is no accel device associated"
                  " with this accel id.\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    ICP_CHECK_PARAM_LT_MAX(bank_number, accel_dev->maxNumBanks);
    bank = &bankHandles[accel_dev->accelId][bank_number];
    ring_number = bank_number * accel_dev->maxNumRingsPerBank;
    ICP_MUTEX_LOCK(bank->user_bank_lock);
    csr_base_addr =  ((Cpa8U*)accel_dev->virtConfigSpace);
    /* Read the ring status CSR to determine which rings are empty. */
    csrVal = READ_CSR_E_STAT(bank_number);
    /* Complement to find which rings have data to be processed. */
    csrVal = ~csrVal;
    /* Return RETRY if the bank polling rings
     * are all empty. */
    if (!(csrVal & bank->pollingMask)) {
        ICP_MUTEX_UNLOCK(bank->user_bank_lock);
        return CPA_STATUS_RETRY;
    }

    /*
     * Loop over all rings within this bank.
     * The ringHandles structure is global to all
     * rings hence while we loop over all rings in the
     * bank we use ring_number to get the global
     * RingHandle.
     */
    for(ringnum_in_bank=0;
        ringnum_in_bank < ICP_ETR_MAX_RINGS_PER_BANK;
        ringnum_in_bank++)
    {
        pRingHandle = ringHandles[accel_dev->accelId][ring_number++];
        /* If this ring has not being created move to next ring. */
        if (NULL == pRingHandle) {
            continue;
        }
        /* And with polling ring mask.
         * If the there is no data on this ring
         * move to the next one.*/
        if (!(csrVal & pRingHandle->pollingMask)) {
            continue;
        }
        /*Poll the ring. */
        status = adf_pollRing(accel_dev, pRingHandle, response_quota);
        if (CPA_STATUS_SUCCESS == status) {
            stat_total++;
        }
    }
    /* Return SUCCESS if adf_pollRing returned SUCCESS
     * at any stage. */
    ICP_MUTEX_UNLOCK(bank->user_bank_lock);
    if (stat_total) {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_RETRY;
}

/*
 * This function allows the user to poll all the response rings
 * belonging to a process per device.
 * This method is used as an alternative to the reading messages
 * via the ISR method.
 * N.B. response_quota is per ring.
 */
CpaStatus icp_sal_pollAllBanks(Cpa32U accelId,
                               Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_RETRY;
    icp_accel_dev_t *accel_dev = NULL;
    adf_dev_bank_handle_t *bank=NULL;
    Cpa32U bank_num=0;
    Cpa32U stat_total=0;

    /* Find the accel device associated with the accelId
     * passed in.
     */
    accel_dev = adf_devmgrGetAccelDevByAccelId(accelId);
    if (!accel_dev) {
        ADF_ERROR("There is no accel device associated"
                  " with this accel id.\n");
        return CPA_STATUS_INVALID_PARAM;
    }
    /* Loop over banks and call icp_sal_pollBank. */
    for(bank_num=0; bank_num < accel_dev->maxNumBanks; bank_num++)
    {
        bank = &bankHandles[accelId][bank_num];
        /* if there are no polling rings on this bank
         * continue to the next bank number. */
        if (bank->pollingMask == 0) {
            continue;
        }
        status = icp_sal_pollBank(accelId,
                                  bank_num,
                                  response_quota);
        if (CPA_STATUS_SUCCESS == status) {
            stat_total++;
        }
    }
    /* Return SUCCESS if icp_sal_pollBank returned SUCCESS
     * at any stage. icp_sal_pollBank cannot
     * return fail in the above case. */
    if (stat_total) {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_RETRY;
}

/*
 * This function allows the user to poll the response ring. The
 * ring number to be polled is supplied by the user via the
 * trans handle for that ring. The trans_hnd is a pointer
 * to an array of trans handles. This ring is
 * only polled if it contains data.
 * This method is used as an alternative to the reading messages
 * via the ISR method.
 * This function will return RETRY if the ring is empty.
 */
CpaStatus icp_adf_pollInstance(icp_comms_trans_handle *trans_hnd,
                               Cpa32U num_transHandles,
                               Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_RETRY;
    adf_dev_ring_handle_t *ring_hnd=NULL;
    adf_dev_ring_handle_t *ring_hnd_first=NULL;
    Cpa32U i=0;
    Cpa32U stat_total=0;

    ICP_CHECK_FOR_NULL_PARAM(trans_hnd);
    ring_hnd_first = (adf_dev_ring_handle_t *)trans_hnd[0];
    if (!ring_hnd_first) {
        return CPA_STATUS_FAIL;
    }
    ICP_MUTEX_LOCK(ring_hnd_first->user_lock);

    for(i = 0; i < num_transHandles; i++)
    {
        ring_hnd =  (adf_dev_ring_handle_t *)trans_hnd[i];
        if (!ring_hnd) {
            ICP_MUTEX_UNLOCK(ring_hnd_first->user_lock);
            return CPA_STATUS_FAIL;
        }

        /* Check polling ring mask.
         * If not polling ring move to the next ring handle. */
        if (!ring_hnd->pollingMask) {
            continue;
        }

        /* Poll the ring. */
        status = adf_pollRing(ring_hnd->accel_dev,
                              ring_hnd,
                              response_quota);
        if (CPA_STATUS_SUCCESS == status) {
            stat_total++;
        }
    }
    ICP_MUTEX_UNLOCK(ring_hnd_first->user_lock);
    /* If any of the rings in the instance had data and was polled
     * return SUCCESS. */
    if (stat_total) {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_RETRY;
}

/*
 * Function initializes internal transport data
 */
CpaStatus adf_user_transport_init(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0;
    Cpa32S x = 0;
    adf_dev_bank_handle_t *bank = NULL;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    status = adf_proxy_populate_device_info(accel_dev);
    if (CPA_STATUS_SUCCESS != status){
        return status;
    }

    for(i=0; i< accel_dev->maxNumBanks; i++)
    {
        bank = &bankHandles[accel_dev->accelId][i];
        bank->user_bank_lock = ICP_MALLOC_GEN(sizeof(ICP_MUTEX));
        if (!bank->user_bank_lock) {
            ADF_ERROR("Could not alloc memory for bank mutex\n");
            for(x = i-1; x >= 0; x--)
            {
                bank = &bankHandles[accel_dev->accelId][x];
                ICP_MUTEX_UNINIT(bank->user_bank_lock);
                ICP_FREE(bank->user_bank_lock);
            }
            adf_proxy_depopulate_device_info(accel_dev);
            return CPA_STATUS_FAIL;
        }
        ICP_MUTEX_INIT(bank->user_bank_lock);
    }
    return status;
}

/*
 * Function deinitializes internal transport data
 */
CpaStatus adf_user_transport_exit(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0;
    adf_dev_bank_handle_t *bank = NULL;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    for(i=0; i< accel_dev->maxNumBanks; i++)
    {
        bank = &bankHandles[accel_dev->accelId][i];
        if (bank->user_bank_lock) {
            ICP_MUTEX_UNINIT(bank->user_bank_lock);
            ICP_FREE(bank->user_bank_lock);
        }
    }
    adf_proxy_depopulate_device_info(accel_dev);
    return status;
}
