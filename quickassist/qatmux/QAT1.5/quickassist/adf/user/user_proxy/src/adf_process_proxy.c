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
 * @file adf_process_proxy.c
 *
 * @description
 * User space interface to ADF in kernel space
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/param.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_platform.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_init.h"
#include "adf_transport_ctrl.h"
#include "adf_user_proxy.h"
#include "adf_user_init.h"
#include "icp_adf_user_proxy.h"
#include "adf_user_transport.h"
#include "adf_init.h"
#include "adf_cfg.h"
#include "icp_adf_debug.h"

/*
 * Value returned by open function in case of failure
 */
#define OPEN_ERROR       -1

/*
 * Number of microsecond
 * that the main thread needs to wait for the reading
 * thread to receive init and start events
 */
#define TIME_DELAY       10

/*
 * Time to sleep between each loop iterration in monitor devices func
 */
#define SLEEP_TIME 2000

/*
 * Process info file and descriptor.
 */
#define PROCESS_INFO_FILE "/dev/icp_dev_processes"

/*
 * proc prefix entry for the device
 */
#define PROCESS_DEVICE_PREFIX "/proc/icp_dh89xxcc_dev"

/*
 * proc prefix entry len
 */
#define PROCESS_DEVICE_PREFIX_LEN 22

/*
 * proc prefix entry for the device
 */
static char * PROCESS_DEVICE_POSTFIX[] = {"/qat0", "/qat1"};

/*
 * proc prefix entry len
 */
#define PROCESS_DEVICE_POSTFIX_LEN 5

/*
 * accelerator number toString Len
 */
#define ACCEL_NAME_LEN 2

#define MAX_ACCEL 2

/*
 * Mutex guarding serialized access to icp_dev_processes
 */
STATIC pthread_mutex_t processes_lock = PTHREAD_MUTEX_INITIALIZER;
STATIC pthread_mutex_t device_lock = PTHREAD_MUTEX_INITIALIZER;
STATIC int process_info_file = -1;

/*
 * Forward function declarations
 */
STATIC void adf_init_devices();
STATIC CpaStatus adf_process_proxy_shutdown(Cpa32U dev_id);

/*
 * User space copy of acceleration devices
 */
STATIC icp_accel_dev_t* accel_tbl[ADF_MAX_DEVICES] = {0};

/*
 * Need to keep track of what device is curently in reset state
 */
STATIC char accel_dev_reset_stat[ADF_MAX_DEVICES] = {0};

/*
 * Need to preserv sal handle during restart
 */
STATIC void* accel_dev_sal_hdl_ptr[ADF_MAX_DEVICES] = {0};

#ifndef ICP_WITHOUT_THREAD
/*
 * Device events listening threads
 */
STATIC pthread_t listeners[ADF_MAX_DEVICES] = {0};

/*
 * Devices monitor thread
 */
STATIC pthread_t monitor_thread;
#endif
/*
 * Mutex guarding access to accel_tbl on exit
 */
STATIC pthread_mutex_t accel_tbl_mutex;

/*
 * Number of acceleration devices
 */
STATIC Cpa16U num_of_instances = 0;

/*
 * Proxy init counter
 */
STATIC Cpa16U init_ctr = 0;

/*
 * Process proxy running state
 */
STATIC OsalAtomic process_proxy_status = 0;

/*
 * adf_process_proxy_stop
 * Sets the process proxy running state to stopped
 */
STATIC inline void adf_process_proxy_stop(void)
{
    osalAtomicSet(0, &process_proxy_status);
}

/*
 * adf_process_proxy_start
 * Sets the process proxy running state to started
 */
STATIC inline void adf_process_proxy_start(void)
{
    osalAtomicSet(1, &process_proxy_status);
}

STATIC inline Cpa32U adf_process_proxy_running(void)
{
    return (Cpa32U)osalAtomicGet(&process_proxy_status);
}

/*
 * adf_is_system_running
 * Returns user proxy running state
 */
STATIC inline Cpa32U adf_is_system_running(icp_accel_dev_t* accel_dev)
{
    return accel_dev->adfSubsystemStatus;
}

/*
 * adf_stop_system
 * Sets the user proxy running state to stopped
 */
STATIC inline void adf_stop_system(icp_accel_dev_t* accel_dev)
{
    accel_dev->adfSubsystemStatus = 0;
}

/*
 * adf_start_system
 * Sets the user proxy running state to started
 */
STATIC inline void adf_start_system(icp_accel_dev_t* accel_dev)
{
    accel_dev->adfSubsystemStatus = 1;
}

/*
 * subsystem_notify
 * Forwards the event to each registered subsystem
 */
STATIC CpaStatus subsystem_notify(icp_accel_dev_t* accel_dev, Cpa32U event)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    CpaStatus stat_proxy = CPA_STATUS_SUCCESS;
    CpaStatus stat_restart = CPA_STATUS_SUCCESS;

    switch (event) {
    case ICP_ADF_EVENT_INIT:
        if (accel_dev_sal_hdl_ptr[accel_dev->accelId]) {
            accel_dev->pSalHandle =
                accel_dev_sal_hdl_ptr[accel_dev->accelId];
            accel_dev_sal_hdl_ptr[accel_dev->accelId] = NULL;
        }
        stat = adf_user_subsystemInit(accel_dev);
        break;
    case ICP_ADF_EVENT_START:
        stat = adf_user_subsystemStart(accel_dev);
        adf_start_system(accel_dev);
        if (accel_dev_reset_stat[accel_dev->accelId]) {
            accel_dev_reset_stat[accel_dev->accelId] = 0;
            stat_restart = adf_subsystemRestarted(accel_dev);
        }
        break;
    case ICP_ADF_EVENT_STOP:
        adf_stop_system(accel_dev);
        stat = adf_user_subsystemStop(accel_dev);
        break;
    case ICP_ADF_EVENT_SHUTDOWN:
        stat = adf_user_subsystemShutdown(accel_dev);
        /* Close user proxy for given device */
        stat_proxy = adf_process_proxy_shutdown(accel_dev->accelId);
        break;
    case ICP_ADF_EVENT_RESTARING:
        accel_dev_reset_stat[accel_dev->accelId] = 1;
        stat = adf_subsystemRestarting(accel_dev);
        accel_dev_sal_hdl_ptr[accel_dev->accelId] = accel_dev->pSalHandle;
        break;
    default:
        stat = CPA_STATUS_INVALID_PARAM;
    }
    if (CPA_STATUS_SUCCESS != stat_proxy) {
        ADF_ERROR("Failed to close process proxy\n");
    }
    if (CPA_STATUS_SUCCESS != stat_restart) {
        ADF_ERROR("Failed to restart device\n");
        stat = stat_restart;
    }
    return stat;
}

STATIC int adf_proxy_get_dev_events(int dev_id);

STATIC void adf_proxy_get_device(int dev_id)
{
    char buff[ADF_DEVICE_NAME_LENGTH] = {'\0'};
    int csr_file_hnd = 0, ring_file_hnd = 0, ret = 0;
    char config_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES]={0};

    /* This one is already initialised */
    if (NULL != accel_tbl[dev_id]) {
        return;
    }
    snprintf(buff, ADF_DEVICE_NAME_LENGTH,
             ADF_CSR_DEVICE_NAME_FORMAT, dev_id);
    /* open event interface */
    csr_file_hnd = open(buff, O_RDWR);
    if (OPEN_ERROR != csr_file_hnd) {
        memset(buff,'\0', ADF_DEVICE_NAME_LENGTH);
        snprintf(buff, ADF_DEVICE_NAME_LENGTH,
                 ADF_RING_DEVICE_NAME_FORMAT, dev_id);
        /* open ring interface */
        ring_file_hnd = open(buff, O_RDWR);
        if (OPEN_ERROR != ring_file_hnd) {
            /* populate userspace accel dev structure */
            accel_tbl[dev_id] =
                (icp_accel_dev_t*)ICP_MALLOC_GEN(sizeof(icp_accel_dev_t));
            if (accel_tbl[dev_id]) {
                memset(accel_tbl[dev_id], '\0', sizeof(icp_accel_dev_t));
                accel_tbl[dev_id]->accelId = dev_id;
                /* Read Configuration data for a device
                 * read Maximal Number on Ring Per Bank */
                icp_adf_cfgGetParamValue(accel_tbl[dev_id],
                                         GENERAL_SEC, ADF_DEV_MAX_RINGS_PER_BANK,
                                         config_value);
                accel_tbl[dev_id]->maxNumRingsPerBank =
                    (Cpa32U)ICP_STRTOUL(config_value, NULL,
                                        ADF_CFG_BASE_DEC);

                /* read Maximal Number of Banks          */
                icp_adf_cfgGetParamValue(accel_tbl[dev_id],
                                         GENERAL_SEC, ADF_DEV_MAX_BANKS,
                                         config_value);
                accel_tbl[dev_id]->maxNumBanks =
                    (Cpa32U)ICP_STRTOUL(config_value, NULL,
                                        ADF_CFG_BASE_DEC);

                /* read User Space Ring Csr Size         */
                icp_adf_cfgGetParamValue(accel_tbl[dev_id],
                                         GENERAL_SEC, ADF_DEV_USER_RING_CSR_SIZE,
                                         config_value);
                accel_tbl[dev_id]->userRingCsrSize =
                    (Cpa32U)ICP_STRTOUL(config_value, NULL,
                                        ADF_CFG_BASE_DEC);

                /* Get accel mask */
                icp_adf_cfgGetParamValue(accel_tbl[dev_id],
                                         GENERAL_SEC, ADF_DEV_ACCEL_MASK,
                                         config_value);
                accel_tbl[dev_id]->accelMask =
                    (Cpa32U)ICP_STRTOUL(config_value, NULL,
                                        ADF_CFG_BASE_DEC);

                /* Get ae mask */
                icp_adf_cfgGetParamValue(accel_tbl[dev_id],
                                         GENERAL_SEC, ADF_DEV_AE_MASK,
                                         config_value);
                accel_tbl[dev_id]->aeMask =
                    (Cpa32U)ICP_STRTOUL(config_value, NULL,
                                        ADF_CFG_BASE_DEC);

                /* Get max number accelerators */
                icp_adf_cfgGetParamValue(accel_tbl[dev_id],
                                         GENERAL_SEC, ADF_DEV_MAX_ACCEL,
                                         config_value);
                accel_tbl[dev_id]->maxNumAccel =
                    (Cpa32U)ICP_STRTOUL(config_value, NULL,
                                        ADF_CFG_BASE_DEC);

                /* Get capabilities mask */
                icp_adf_cfgGetParamValue(accel_tbl[dev_id],
                                         GENERAL_SEC, ADF_DEV_CAPABILITIES_MASK,
                                         config_value);
                accel_tbl[dev_id]->accelCapabilitiesMask =
                    (Cpa32U)ICP_STRTOUL(config_value, NULL,
                                        ADF_CFG_BASE_DEC);

                /* Get sku */
                icp_adf_cfgGetParamValue(accel_tbl[dev_id],
                                         GENERAL_SEC, ADF_DEV_SKU,
                                         config_value);
                accel_tbl[dev_id]->sku =
                    (dev_sku_info_t)ICP_STRTOUL(config_value, NULL,
                                        ADF_CFG_BASE_DEC);

                if (adf_user_transport_init(accel_tbl[dev_id])
                        != CPA_STATUS_SUCCESS) {
                    ADF_ERROR("Failed initialize transport dev%d\n",
                              dev_id);
                    close(csr_file_hnd);
                    close(ring_file_hnd);
                    ICP_FREE(accel_tbl[dev_id]);
                    accel_tbl[dev_id] = NULL;
                    return;
                }
                memcpy(accel_tbl[dev_id]->devFileName,
                       buff, ADF_DEVICE_NAME_LENGTH);
                accel_tbl[dev_id]->csrFileHdl = csr_file_hnd;
                accel_tbl[dev_id]->ringFileHdl = ring_file_hnd;
                /* mmap the ring controller config space */
                accel_tbl[dev_id]->virtConfigSpace =
                    (Cpa8U *) ICP_MMAP(NULL,
                                       accel_tbl[dev_id]->userRingCsrSize,
                                       PROT_READ|PROT_WRITE,
                                       MAP_FILE|MAP_SHARED,
                                       accel_tbl[dev_id]->csrFileHdl,
                                       0);
                if (MAP_FAILED == accel_tbl[dev_id]->virtConfigSpace) {
                    ADF_ERROR("ET Ring controller csr mmap failed\n");
                    ADF_ERROR("for acceleration device %d\n", dev_id);
                    adf_user_transport_exit(accel_tbl[dev_id]);
                    close(csr_file_hnd);
                    close(ring_file_hnd);
                    ICP_FREE(accel_tbl[dev_id]);
                    accel_tbl[dev_id] = NULL;
                    return;
                }
                ret = adf_proxy_get_dev_events(dev_id);

                if (ret) {
                    ADF_ERROR("Creating listeners error");
                    ADF_ERROR("for acceleration device %d\n", dev_id);
                    ADF_ERROR("freeing memory\n");
                    close(csr_file_hnd);
                    close(ring_file_hnd);
                    ICP_FREE(accel_tbl[dev_id]);
                    accel_tbl[dev_id] = NULL;
                    return;
                }

                num_of_instances++;
            } else {
                ADF_ERROR("Alloc memory for accel_dev error\n");
                close(csr_file_hnd);
                close(ring_file_hnd);
                return;
            }
        } else {
            close(csr_file_hnd);
            return;
        }
    }
    return;
}

#ifdef ICP_WITHOUT_THREAD

STATIC int adf_proxy_get_dev_events(int dev_id)
{
    int file_hndl = 0;
    Cpa8U buff[ADF_MAX_PENDING_EVENT * sizeof(icp_adf_subsystemEvent_t)];
    Cpa32S size = 0;
    Cpa8U events = 0, i = 0;
    Cpa32U *event = NULL, eventsize=0;

    if (accel_tbl[dev_id] != NULL) {
        file_hndl = accel_tbl[dev_id]->csrFileHdl;

        /*
         * Event size will be 4 bytes and hence the maximum
         * that can be read in will be
         * ADF_MAX_PENDING_EVENT * eventsize.
         */
        eventsize = sizeof(icp_adf_subsystemEvent_t);
        memset(buff, '\0', ADF_MAX_PENDING_EVENT * eventsize);
        size = ICP_READ(file_hndl, buff, ADF_MAX_PENDING_EVENT * eventsize);
        if (0 >= size) {
            /* no events or handle closed */
            return 0;
        }
        events = (Cpa8U)(size / eventsize);
        event = (Cpa32U*)buff;
        for (i = 0; i< events; i++) {
            subsystem_notify(accel_tbl[dev_id], *event++);
        }
    }
    return 0;
}

STATIC int adf_proxy_get_devices(void)
{
    Cpa32U ctr = 0;
    for (ctr = 0; ctr < ADF_MAX_DEVICES; ctr++) {
        adf_proxy_get_device(ctr);
    }
    return 0;
}

CpaStatus icp_adf_find_new_devices(void)
{
    adf_proxy_get_devices();
    return CPA_STATUS_SUCCESS;
}

CpaStatus icp_adf_poll_device_events(void)
{
    Cpa32U ctr = 0;
    for (ctr = 0; ctr < ADF_MAX_DEVICES; ctr++) {
        adf_proxy_get_dev_events(ctr);
    }
    return CPA_STATUS_SUCCESS;
}
#else

/*
* adf_proxy_get_event
* functions runs in a separate thread (one thread per device)
* and calls the blocking read. Once the read returns the events buffer
* should be filled with events. For each event subsystem_notify is called,
* which sends the event to registered subsystem.
*/
STATIC void* adf_proxy_get_event(void* hndl)
{
    int file_hndl = 0, dev_id = ((int)(UARCH_INT) hndl);
    Cpa8U buff[ADF_MAX_PENDING_EVENT * sizeof(icp_adf_subsystemEvent_t)];
    Cpa32S size = 0;
    Cpa8U events = 0, i = 0;
    Cpa32U *event = NULL, eventsize=0;

    if (accel_tbl[dev_id] != NULL) {
        file_hndl = accel_tbl[dev_id]->csrFileHdl;

        /*
        * Event size will be 4 bytes and hence the maximum
        * that can be read in will be
        * ADF_MAX_PENDING_EVENT * eventsize.
        */
        eventsize = sizeof(icp_adf_subsystemEvent_t);
        memset(buff, '\0', ADF_MAX_PENDING_EVENT*eventsize);
        while (adf_process_proxy_running() && (NULL != accel_tbl[dev_id])) {
            size = ICP_READ(file_hndl, buff, ADF_MAX_PENDING_EVENT * eventsize);
            if (EAGAIN == size) {
                continue;
            }
            if (0 > size) {
                /* handle closed */
                return NULL;
            }
            events = (Cpa8U)(size / eventsize);
            event = (Cpa32U*)buff;
            for (i = 0; i< events; i++) {
                subsystem_notify(accel_tbl[dev_id], *event++);
            }
        }
    }
    return NULL;
}

STATIC int adf_proxy_get_dev_events(int dev_id)
{
    /* create event listening thread */
    int ret = pthread_create(&listeners[dev_id], NULL,
                              adf_proxy_get_event,
                             (void *)(ARCH_INT)dev_id);

                /* wait a bit for the reading thread */
    usleep(TIME_DELAY);
    return ret;
}

/*
* adf_proxy_monitor_devices
* Funtion waits for available acceleration devices
* and opens and configures them as they become available
*/
STATIC void* adf_proxy_monitor_devices(void* arg)
{
    Cpa32U ctr = 0;

    while (adf_process_proxy_running()) {
        /* Try to open all icp_dev device in the system */
        for (ctr=0; ctr < ADF_MAX_DEVICES; ctr++) {
            adf_proxy_get_device(ctr);
        }
        if (!adf_process_proxy_running()) {
            break;
        }
        /*
        * take a breath
        */
        ICP_MSLEEP(SLEEP_TIME);
    }
    return NULL;
}

STATIC int adf_proxy_get_devices(void)
{
    return pthread_create(&monitor_thread, NULL,
                          adf_proxy_monitor_devices, NULL);
}

CpaStatus icp_adf_find_new_devices(void)
{
    return CPA_STATUS_SUCCESS;
}

CpaStatus icp_adf_poll_device_events(void)
{
    return CPA_STATUS_SUCCESS;
}
#endif

/*
 * adf_process_proxy_init
 * Init process proxy and connect to kernel space.
 * For every acceleration device in the system open
 * events and rings interface and start event listening threads
 */
STATIC CpaStatus adf_process_proxy_init(void)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    int ret = 0;
    if (adf_process_proxy_running()) {
        ADF_ERROR("Proxy already running\n");
        return CPA_STATUS_FAIL;
    }
    adf_process_proxy_start();
    adf_init_devices();
    ret = adf_proxy_get_devices();
    if (ret) {
        stat = CPA_STATUS_FAIL;
    }
    return stat;
}

/*
 * adf_process_proxy_shutdown
 * User space proxy is shutting down. Close and clean all opened devices
 */
STATIC CpaStatus adf_process_proxy_shutdown(Cpa32U dev_id)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;

    if (NULL != accel_tbl[dev_id]) {
        int ret = 0;
        pthread_mutex_lock(&accel_tbl_mutex);
        if (0 != ioctl(accel_tbl[dev_id]->csrFileHdl,
                          ADF_USER_PROCESS_IOCTL_RELEASE, NULL)) {
            ADF_ERROR("Error in ioctl return\n");
        }
#ifndef ICP_WITHOUT_THREAD
            if (0 != listeners[dev_id]) {
                (void)pthread_join(listeners[dev_id] ,NULL);
            }
#endif
        ret = munmap(accel_tbl[dev_id]->virtConfigSpace,
                     accel_tbl[dev_id]->userRingCsrSize);
        if (ret) {
            ADF_ERROR("Failed to munmap virt config space %d\n", dev_id);
        }
        close(accel_tbl[dev_id]->csrFileHdl);
        stat = adf_user_unmap_rings(accel_tbl[dev_id]);
        if (CPA_STATUS_SUCCESS != stat) {
            ADF_ERROR("Failed to munmap memory for dev %d\n", dev_id);
        }
        close(accel_tbl[dev_id]->ringFileHdl);
        adf_user_transport_exit(accel_tbl[dev_id]);
        ICP_FREE(accel_tbl[dev_id]);
        accel_tbl[dev_id] = NULL;
        pthread_mutex_unlock(&accel_tbl_mutex);
    }
    return stat;
}

/*
 * icp_adf_userProxyInit
 * This function is called by the application to bring the proxy up & running
 * Every userspace process has to call it to be able to create rings
 * and receive events.
 */
CpaStatus icp_adf_userProxyInit(const char *const name)
{
    CpaStatus status = CPA_STATUS_FAIL;

    ICP_CHECK_FOR_NULL_PARAM(name);
    /* Allow the user to call init just once */
    if (init_ctr) {
        ADF_ERROR("User proxy already initialized\n");
        return status;
    }
    pthread_mutex_init(&accel_tbl_mutex, NULL);
    /* Connect to kernel space. */
    status = adf_process_proxy_init();
    if (CPA_STATUS_SUCCESS != status) {
        ADF_ERROR("adf_process_proxy_init failed\n");
        return CPA_STATUS_FAIL;
    }
    init_ctr = 1;
    return status;
}

/*
 * icp_adf_userProxyShutdown
 * This function is called by the application to shutdown the proxy
 */
CpaStatus icp_adf_userProxyShutdown(void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U ctr = 0;

    adf_process_proxy_stop();
#ifndef ICP_WITHOUT_THREAD
    if(0 != monitor_thread) {
        (void)pthread_join(monitor_thread,NULL);
    }
#endif
    num_of_instances = 0;
    for (ctr=0; ctr < ADF_MAX_DEVICES; ctr++) {
        status = adf_process_proxy_shutdown(ctr);
        if (CPA_STATUS_SUCCESS != status) {
            ADF_ERROR("adf_process_proxy_shutdown failed for device %d\n",ctr);
            status = CPA_STATUS_FAIL;
        }
    }
    init_ctr = 0;
    pthread_mutex_destroy(&accel_tbl_mutex);
    return status;
}

/*
 * icp_amgr_getNumInstances
 * Return the number of acceleration devices it the system.
 */
CpaStatus icp_amgr_getNumInstances(Cpa16U *pNumInstances)
{
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);
    *pNumInstances = num_of_instances;
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_amgr_getInstances
 * Return table of acceleration instances it the system.
 */
CpaStatus icp_amgr_getInstances(Cpa16U numInstances,
                                icp_accel_dev_t **pCyInstances)
{
    Cpa16U i = 0, x = 0;
    ICP_CHECK_FOR_NULL_PARAM(pCyInstances);

    if (numInstances > num_of_instances) {
        /* Too many acceleration devices requested */
        ADF_ERROR("Too many instances of accel device requested\n");
        return CPA_STATUS_FAIL;
    }

    for (i = 0; i < ADF_MAX_DEVICES; i++) {
        if (NULL != accel_tbl[i]) {
            pCyInstances[x++] = (icp_accel_dev_t *) accel_tbl[i];
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_GetInstances
 * Return the acceleration instance by name.
 */
CpaStatus icp_amgr_getAccelDevByName(unsigned char* instanceName,
                                     icp_accel_dev_t **pAccel_dev)
{
    Cpa16U i = 0;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_dev);
    ICP_CHECK_FOR_NULL_PARAM(instanceName);

    for (i = 0; i < ADF_MAX_DEVICES; i++) {
        if (NULL != accel_tbl[i]) {
            if (strncmp((const char*)instanceName,
                        (const char*)accel_tbl[i]->pAccelName,
                        MAX_ACCEL_NAME_LEN) == 0) {
                *pAccel_dev = (icp_accel_dev_t *) accel_tbl[i];
                return CPA_STATUS_SUCCESS;
            }
        }
    }
    return CPA_STATUS_FAIL;
}

/*
 * icp_amgr_getAccelDevByCapabilities
 * Returns a started accel device that implements
 * the capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAccelDevByCapabilities(Cpa32U capabilitiesMask,
        icp_accel_dev_t **pAccel_devs,
        Cpa16U *pNumInstances)
{
    icp_accel_dev_t **ptr = accel_tbl;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_devs);
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);
    Cpa16U i = 0;

    *pNumInstances = 0;
    for (i = 0; i < ADF_MAX_DEVICES; i++, ptr++) {
        if (NULL != *ptr) {
            if ((*ptr)->accelCapabilitiesMask & capabilitiesMask) {
                if ((*ptr)->adfSubsystemStatus) {
                    *pAccel_devs = (icp_accel_dev_t *) *ptr;
                    *pNumInstances = 1;
                    return CPA_STATUS_SUCCESS;
                }
            }
        }
    }
    return CPA_STATUS_FAIL;
}

/*
 * icp_amgr_getAllAccelDevByCapabilities
 * Returns table of accel devicess that are started and implement
 * the capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAllAccelDevByCapabilities(Cpa32U capabilitiesMask,
        icp_accel_dev_t **pAccel_devs,
        Cpa16U *pNumInstances)
{
    icp_accel_dev_t **ptr = accel_tbl;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_devs);
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);
    Cpa16U i = 0;

    *pNumInstances = 0;
    for (i = 0; i < ADF_MAX_DEVICES; i++, ptr++) {
        if (NULL != *ptr) {
            if ((*ptr)->accelCapabilitiesMask & capabilitiesMask) {
                if ((*ptr)->adfSubsystemStatus) {
                    pAccel_devs[(*pNumInstances)++] = (icp_accel_dev_t *) *ptr;
                }
            }
        }
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * icp_amgr_getAccelDevCapabilities
 * Returns accel devices capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAccelDevCapabilities(icp_accel_dev_t *accel_dev,
        Cpa32U *pCapabilitiesMask)
{
    icp_accel_dev_t *pAccelDev = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pCapabilitiesMask);

    pAccelDev = accel_dev;
    *pCapabilitiesMask = pAccelDev->accelCapabilitiesMask;
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_devmgrGetAccelHead
 * Sets the AccelDev to the head of the accelerator table.
 * Note: This function returns pointer to acceleration table
 * unlike the same function in kernelspace where it returns
 * pointer to list head.
 */
CpaStatus adf_devmgrGetAccelHead(icp_accel_dev_t **pAccelDev)
{
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    *pAccelDev = (icp_accel_dev_t*)accel_tbl;
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_init_devices
 * Initialise the table of accel_dev pointers to NULL.
 */
void adf_init_devices(void)
{
    Cpa32U i=0;
    for (i=0; i< ADF_MAX_DEVICES; i++) {
        accel_tbl[i] = NULL;
    }
    return;
}

/*
 * icp_adf_debugAddDir
 * Stub function for SAL userspace
 */

CpaStatus
icp_adf_debugAddDir(icp_accel_dev_t *accel_dev,
                    debug_dir_info_t* dir_info)
{
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_debugRemoveDir
 * Stub function for SAL userspace
 */
void
icp_adf_debugRemoveDir(debug_dir_info_t* dir_info)
{
}
/*
 * icp_adf_debugAddFile
 * Stub function for SAL userspace
 */
CpaStatus
icp_adf_debugAddFile(icp_accel_dev_t *accel_dev,
                     debug_file_info_t* file_info)
{
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_debugRemoveFile
 * Stub function for SAL userspace
 */
void
icp_adf_debugRemoveFile(debug_file_info_t* file_info)
{
}

/*
 * icp_adf_esramGetAddress
 * This function retrieves the eSRAM information stored in the device's
 * internal configuration table.
 */
CpaStatus icp_adf_esramGetAddress(icp_accel_dev_t *accel_dev,
                                  Cpa32U accelNumber,
                                  Cpa64U *pPhysAddr,
                                  Cpa64U *pVirtAddr,
                                  Cpa32U *pSize)
{
    CpaStatus status = CPA_STATUS_FAIL;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(accel_dev,
                                      INTERNAL_SEC, ADF_SRAM_PHYSICAL_ADDRESS, valStr);
    ICP_CHECK_STATUS_AND_LOG(status,
                             "Failed to get %s value\n", ADF_SRAM_PHYSICAL_ADDRESS);
    *pPhysAddr = (Cpa64U)ICP_STRTOULL(valStr, NULL, ADF_CFG_BASE_HEX);
    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(accel_dev,
                                      INTERNAL_SEC, ADF_SRAM_VIRTUAL_ADDRESS, valStr);
    ICP_CHECK_STATUS_AND_LOG(status,
                             "Failed to get %s value\n", ADF_SRAM_VIRTUAL_ADDRESS);
    *pVirtAddr = (Cpa64U)ICP_STRTOULL(valStr, NULL, ADF_CFG_BASE_HEX);
    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(accel_dev,
                                      INTERNAL_SEC, ADF_SRAM_SIZE_IN_BYTES, valStr);
    ICP_CHECK_STATUS_AND_LOG(status,
                             "Failed to get %s value\n", ADF_SRAM_SIZE_IN_BYTES);
    *pSize = (Cpa32U)ICP_STRTOUL(valStr, NULL, ADF_CFG_BASE_DEC);
    *pSize = (*pSize) >> 1;
    if (1 == accelNumber) {
        *pPhysAddr += *pSize;
        *pVirtAddr += *pSize;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_devmgrGetAccelDevByAccelId
 * Check the accel table for a structure that contains the correct
 * accel ID. If the accelId is found return the pointer to the accelerator
 * structure.
 * Returns a pointer to the accelerator structure or NULL if not found.
 */
icp_accel_dev_t *adf_devmgrGetAccelDevByAccelId(Cpa32U accelId)
{
    icp_accel_dev_t **ptr = accel_tbl;
    Cpa16U i = 0;

    for (i = 0; i < ADF_MAX_DEVICES; i++, ptr++) {
        if (NULL != *ptr) {
            if ((*ptr)->accelId == accelId) {
                return *ptr;
            }
        }
    }
    return NULL;
}

/*
 * icp_adf_getAccelDevByAccelId
 * Same as adf_devmgrGetAccelDevByAccelId() but for external use
 * structure.
 * Returns a pointer to the accelerator structure or NULL if not found.
 */
icp_accel_dev_t* icp_adf_getAccelDevByAccelId(Cpa32U accelId)
{
    return adf_devmgrGetAccelDevByAccelId(accelId);
}


/*
 * icp_adf_userProcessToStart
 *
 *  This function checks if an user space process with a given name has
 *  already been started.
 *  Returns:
 *  False - process with a givern name is not started. I.e. it is safe
 *          to start one.
 *  True - process is started or couldn't figure out if it is started.
 */
CpaStatus icp_adf_userProcessToStart(const char *const name_tml,
                                     char *name)
{
    int res = 0, name_len = 0;

    /* Validate process name */
    if (!name_tml || !name) {
        ADF_ERROR("Invalid pointer\n");
        return CPA_STATUS_FAIL;
    }

    name_len = strlen(name_tml);
    if (name_len + 1 > ADF_CFG_MAX_SECTION_LEN_IN_BYTES || 0 == name_len) {
        ADF_ERROR("Invalid Process name\n");
        return CPA_STATUS_FAIL;
    }

    if (pthread_mutex_lock(&processes_lock)) {
        ADF_ERROR("Mutex lock error %d\n", errno);
        return CPA_STATUS_FAIL;
    }
    if (process_info_file != -1) {
        ADF_ERROR("File "PROCESS_INFO_FILE" already opened\n");
        pthread_mutex_unlock(&processes_lock);
        return CPA_STATUS_FAIL;
    }
    process_info_file = open(PROCESS_INFO_FILE, O_RDWR);
    if (process_info_file < 0) {
        ADF_ERROR("Can not open "PROCESS_INFO_FILE" file\n");
        /* we cannot check if the process is in use or not - we'll consider
        * that it is to be not save*/
        pthread_mutex_unlock(&processes_lock);
        return CPA_STATUS_FAIL;
    }
    res = write(process_info_file, name_tml, name_len);
    if (res < 0) {
        ADF_ERROR("Error reading "PROCESS_INFO_FILE" file\n");
        close(process_info_file);
        process_info_file = -1;
        pthread_mutex_unlock(&processes_lock);
        return CPA_STATUS_FAIL;
    }
    if (res == 0) {
        res = read(process_info_file, name, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
        if (pthread_mutex_unlock(&processes_lock)) {
            ADF_ERROR("Mutex unlock error %d\n", errno);
            close(process_info_file);
            process_info_file = -1;
            return CPA_STATUS_FAIL;
        }
        if (res == 0)
            return CPA_STATUS_SUCCESS;
    }
    close(process_info_file);
    process_info_file = -1;
    pthread_mutex_unlock(&processes_lock);
    return CPA_STATUS_FAIL;
}

void icp_adf_userProcessStop(void)
{
    if (pthread_mutex_lock(&processes_lock)) {
        ADF_ERROR("Mutex lock error %d\n", errno);
        return;
    }
    if (process_info_file != -1) {
        close(process_info_file);
        process_info_file = -1;
    }
    pthread_mutex_unlock(&processes_lock);
}

/*
 * icp_adf_get_busAddress
 */
Cpa16U icp_adf_get_busAddress(Cpa16U packageId)
{
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa16U busAddress = 0;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    icp_accel_dev_t *accel_dev = NULL;

    accel_dev = icp_adf_getAccelDevByAccelId(packageId);
    if (!accel_dev) {
        ADF_ERROR("Can not find dev%d\n", packageId);
        return 0;
    }
    ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(accel_dev,
                                      GENERAL_SEC, ADF_DEV_BUS_ADDRESS, valStr);
    ICP_CHECK_STATUS_AND_LOG(status,
                             "Failed to get %s value\n", ADF_DEV_BUS_ADDRESS);
    busAddress = (Cpa32U)ICP_STRTOUL(valStr, NULL, ADF_CFG_BASE_DEC);
    return busAddress;
}

/*
 * icp_adf_is_dev_in_reset
 * Check if device is in reset state.
 */
CpaBoolean icp_adf_is_dev_in_reset(icp_accel_dev_t *accel_dev)
{
    return (CpaBoolean)accel_dev_reset_stat[accel_dev->accelId];
}

/*
 * icp_adf_check_device
 * Function checks the status of the firmware/hardware for a given device.
 */
CpaStatus icp_adf_check_device(Cpa32U accelId)
{
    int process_device_file = -1;
    int spacer = (accelId < 10)?-1:0;
    char *proc_name = NULL;
    char tmp[3];
    CpaStatus status = CPA_STATUS_SUCCESS;
    int i = 0;
    int res = 0;
    icp_accel_dev_t *device = NULL;

    device = icp_adf_getAccelDevByAccelId(accelId);
    if (NULL == device)
    {
        ADF_ERROR("Can not find device%d\n", accelId);
        return CPA_STATUS_FAIL;
    }

    proc_name = ICP_ZALLOC_GEN(PROCESS_DEVICE_PREFIX_LEN + ACCEL_NAME_LEN +
            PROCESS_DEVICE_POSTFIX_LEN + 1);
    if (NULL == proc_name) {
        ADF_ERROR("Can not allocate proc_name\n");
        return CPA_STATUS_FAIL;
    }
    memcpy(proc_name, PROCESS_DEVICE_PREFIX, PROCESS_DEVICE_PREFIX_LEN);
    sprintf(proc_name+PROCESS_DEVICE_PREFIX_LEN, "%d", accelId);
    for (i = 0; i < device->maxNumAccel; i++) {
        if(!(device->accelMask & (1<<i))) continue;

        memcpy(proc_name+PROCESS_DEVICE_PREFIX_LEN + ACCEL_NAME_LEN + spacer,
                 PROCESS_DEVICE_POSTFIX[i], PROCESS_DEVICE_POSTFIX_LEN);
        if (pthread_mutex_lock(&device_lock)) {
            ADF_ERROR("Mutex lock error %d\n", errno);
            status = CPA_STATUS_FAIL;
            break;
        }
        process_device_file = -1;
        process_device_file = open(proc_name, O_RDONLY);
        if (process_device_file < 0) {
            ADF_ERROR("Can not open %s file\n", proc_name);
            pthread_mutex_unlock(&device_lock);
            status = CPA_STATUS_FAIL;
            break;
        }
        memset(tmp, '\0', sizeof(tmp));
        res = read(process_device_file, tmp, sizeof(tmp));
        if ((res < 0) || ('+' != tmp[0])) {
			ADF_ERROR("Error reading "PROCESS_INFO_FILE" file\n");
			pthread_mutex_unlock(&device_lock);
			close(process_device_file);
			process_device_file = -1;
			status = CPA_STATUS_FAIL;
			break;
		}
        if (pthread_mutex_unlock(&device_lock)) {
            ADF_ERROR("Mutex unlock error %d\n", errno);
            close(process_device_file);
            process_device_file = -1;
            status = CPA_STATUS_FAIL;
            break;
        }
        close(process_device_file);
        process_device_file = -1;
    }
    ICP_FREE(proc_name);
    return status;
}

/*
 * icp_adf_check_all_devices
 * Function checks the status of the firmware/hardware for all devices.
 */
CpaStatus icp_adf_check_all_devices(void)
{
    Cpa32U i;
    CpaStatus res  = CPA_STATUS_FAIL;
    for (i = 0; i < ADF_MAX_DEVICES; i++)
    {
        if(NULL != accel_tbl[i])
        {
            res = icp_adf_check_device(i);
            if(CPA_STATUS_SUCCESS != res)
            {
                ADF_ERROR("Device Check failed for device %d\n", i);
                return res;
            }
        }
    }
    return CPA_STATUS_SUCCESS;
}
