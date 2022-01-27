/******************************************************************************
 *
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
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
#include <libudev.h>

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_platform.h"
#include "adf_kernel_types.h"
#include "adf_cfg_user.h"
#include "icp_adf_init.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_user_init.h"
#include "adf_user_transport.h"
#include "adf_init.h"
#include "uio_user.h"
#include "uio_user_cfg.h"

#define ADF_DEV_PROCESSES_PATH "/dev/qat_dev_processes"
/*
 * Value returned by open function in case of failure
 */
#define OPEN_ERROR -1

/*
 * Number of microsecond
 * that the main thread needs to wait for the reading
 * thread to receive init and start events
 */
#define TIME_DELAY 10
#define EVENT_MAX_LEN 20
#define ACCELID_MAX_LEN 5

/*
 * Time to sleep between each loop iteration in monitor devices func
 */
#define SLEEP_TIME 2000

STATIC struct udev *udev;
STATIC struct udev_monitor *mon;

/*
 * Proxy init counter
 */
STATIC Cpa16U init_ctr = 0;

/*
 * Mutex guarding serialized access to icp_dev_processes
 */
STATIC OsalMutex processes_lock;
STATIC int process_info_file = -1;

/*
 * Process proxy running state
 */
STATIC OsalAtomic process_proxy_status = 0;

/*
 * adf_reset_userProxy
 *
 * Description:
 *  Function to reset the ADF proxy status in user space.
 *  It resets proxy status and related parameters.
 *
 * Returns: void
 */
void adf_reset_userProxy(void);

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

STATIC CpaStatus adf_event_monitor_create(void)
{
    int ret = CPA_STATUS_SUCCESS;

    udev = udev_new();
    if (!udev)
    {
        ADF_ERROR("Can't create udev\n");
        ret = CPA_STATUS_FAIL;
    }

    if (ret == CPA_STATUS_SUCCESS)
    {
        mon = udev_monitor_new_from_netlink(udev, "udev");
        if (!mon)
        {
            ADF_ERROR("Can't create udev monitor\n");
            ret = CPA_STATUS_FAIL;
        }
    }

    if (ret == CPA_STATUS_SUCCESS)
    {
        if (udev_monitor_filter_add_match_subsystem_devtype(mon, "pci", NULL))
        {
            ADF_ERROR("Can't add udev match filter\n");
            ret = CPA_STATUS_FAIL;
        }
    }

    if (ret == CPA_STATUS_SUCCESS)
    {
        if (udev_monitor_enable_receiving(mon))
        {
            ADF_ERROR("Can't bind monitor to event source\n");
            ret = CPA_STATUS_FAIL;
        }
    }

    if (ret != CPA_STATUS_SUCCESS)
    {
        if (mon)
            udev_monitor_unref(mon);
        mon = NULL;
        if (udev)
            udev_unref(udev);
        udev = NULL;
    }
    return ret;
}

void adf_event_monitor_delete(void)
{
    if (mon)
        udev_monitor_unref(mon);
    if (udev)
        udev_unref(udev);
}

int adf_proxy_poll_event(Cpa32U *dev_id, enum adf_event *event)
{
    int fd;
    struct udev_device *dev;
    fd_set fds;
    struct timeval tv;
    const char *eventStr = NULL;
    const char *accelIdStr = NULL;
    char eventString[EVENT_MAX_LEN];
    char accelIdString[ACCELID_MAX_LEN];

    fd = udev_monitor_get_fd(mon);
    if (fd)
    {
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        if (select(fd + 1, &fds, NULL, NULL, &tv) > 0 && FD_ISSET(fd, &fds))
        {
            dev = udev_monitor_receive_device(mon);
            if (dev)
            {
                eventStr = udev_device_get_property_value(dev, "qat_event");
                if (eventStr)
                    strncpy(eventString, eventStr, strlen(eventStr) + 1);
                accelIdStr = udev_device_get_property_value(dev, "accelid");
                if (accelIdStr)
                    strncpy(accelIdString, accelIdStr, strlen(accelIdStr) + 1);
                udev_device_unref(dev);
            }
        }
        if (!eventStr || !accelIdStr)
            return 0;

        if (!strcmp(eventString, "init"))
            *event = ADF_EVENT_INIT;
        else if (!strcmp(eventString, "shutdown"))
            *event = ADF_EVENT_SHUTDOWN;
        else if (!strcmp(eventString, "restarting"))
            *event = ADF_EVENT_RESTARTING;
        else if (!strcmp(eventString, "restarted"))
            *event = ADF_EVENT_RESTARTED;
        else if (!strcmp(eventString, "start"))
            *event = ADF_EVENT_START;
        else if (!strcmp(eventString, "stop"))
            *event = ADF_EVENT_STOP;
        else if (!strcmp(eventString, "error"))
            *event = ADF_EVENT_ERROR;
        else
        {
            ADF_ERROR("Unknown event \"%s\" received\n", eventString);
            return 0;
        }

        *dev_id = strtoul(accelIdString, NULL, 0);

        return 1;
    }
    return 0;
}

/*
 * adf_process_proxy_init
 * Init process proxy and connect to kernel space.
 * For every acceleration device in the system open
 * events and rings interface and start event listening threads
 */
STATIC CpaStatus adf_process_proxy_init(void)
{
    if (adf_process_proxy_running())
    {
        ADF_ERROR("Proxy already running\n");
        return CPA_STATUS_FAIL;
    }

    if (adf_event_monitor_create() != CPA_STATUS_SUCCESS)
        return CPA_STATUS_FAIL;

    adf_process_proxy_start();
    if (adf_init_devices())
    {
        ADF_ERROR("Error initializing devices\n");
        return CPA_STATUS_FAIL;
    }

    return adf_proxy_get_devices();
}

/*
 * adf_process_proxy_shutdown
 * User space proxy is shutting down. Close and clean all opened devices
 */
static __inline__ CpaStatus adf_process_proxy_shutdown()
{
    adf_event_monitor_delete();

    return adf_cleanup_devices();
}

/*
 * icp_adf_userProxyInit
 * This function is called by the application to bring the proxy up & running
 * Every userspace process has to call it to be able to create rings
 * and receive events.
 */
CpaStatus icp_adf_userProxyInit(char const *const name)
{
    CpaStatus status = CPA_STATUS_FAIL;

    ICP_CHECK_FOR_NULL_PARAM(name);
    /* Allow the user to call init just once */
    if (init_ctr)
    {
        ADF_ERROR("User proxy alreay initialized\n");
        return status;
    }
    init_ctr = 1;
    /* Connect to kernel space. */
    status = adf_process_proxy_init();
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("adf_process_proxy_init failed\n");
        return CPA_STATUS_FAIL;
    }
    if (!processes_lock)
        osalMutexInit(&processes_lock);

    return status;
}

/*
 * icp_adf_userProxyShutdown
 * This function is called by the application to shutdown the proxy
 */
CpaStatus icp_adf_userProxyShutdown(void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    adf_process_proxy_stop();
    status = adf_process_proxy_shutdown();
    init_ctr = 0;
    osalMutexDestroy(&processes_lock);

    return status;
}

/*
 * icp_adf_userProcessToStart
 *
 *  This function checks if an user space process with a given name has
 *  already been started.
 *  Returns:
 *  False - process with a given name is not started. I.e. it is safe
 *          to start one.
 *  True - process is started or couldn't figure out if it is started.
 */
CpaStatus icp_adf_userProcessToStart(char const *const name_tml, char *name)
{
    int res = 0, name_len = 0;

    /* Validate process name */
    if (!name_tml || !name)
    {
        ADF_ERROR("Invalid pointer\n");
        return CPA_STATUS_FAIL;
    }

    name_len = strnlen(name_tml, ADF_CFG_MAX_SECTION_LEN_IN_BYTES + 1);
    if (name_len + 1 > ADF_CFG_MAX_SECTION_LEN_IN_BYTES || 0 == name_len)
    {
        ADF_ERROR("Invalid Process name\n");
        return CPA_STATUS_FAIL;
    }

    if (osalMutexLock(&processes_lock, OSAL_WAIT_FOREVER))
    {
        ADF_ERROR("Mutex lock error %d\n", errno);
        return CPA_STATUS_FAIL;
    }
    if (process_info_file != -1)
    {
        ADF_ERROR("File " ADF_DEV_PROCESSES_PATH " already opened\n");
        osalMutexUnlock(&processes_lock);
        return CPA_STATUS_FAIL;
    }
    process_info_file = open(ADF_DEV_PROCESSES_PATH, O_RDWR);
    if (process_info_file < 0)
    {
        ADF_ERROR("Cannot open " ADF_DEV_PROCESSES_PATH " file\n");
        /* we cannot check if the process is in use or not - we'll consider
         * that it is to be not save */
        osalMutexUnlock(&processes_lock);
        return CPA_STATUS_FAIL;
    }
    res = write(process_info_file, name_tml, name_len);
    if (res < 0)
    {
        close(process_info_file);
        process_info_file = -1;
        ADF_ERROR("Error reading " ADF_DEV_PROCESSES_PATH " file\n");
        osalMutexUnlock(&processes_lock);
        return CPA_STATUS_FAIL;
    }
    if (res == 0)
    {
        res = read(process_info_file, name, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
        if (osalMutexUnlock(&processes_lock))
        {
            ADF_ERROR("Mutex unlock error %d\n", errno);
            return CPA_STATUS_FAIL;
        }
        if (res == 0)
        {
            return CPA_STATUS_SUCCESS;
        }
    }

    close(process_info_file);
    process_info_file = -1;
    osalMutexUnlock(&processes_lock);
    return CPA_STATUS_FAIL;
}

/*
 * icp_adf_userProcessStop
 *
 *  This function stops the user process.
 */
void icp_adf_userProcessStop(void)
{
    if (process_info_file > 0)
        close(process_info_file);
    process_info_file = -1;

    return;
}

/*
 * icp_adf_get_busAddress
 *
 *  This function gets the bus address of the device.
 */
Cpa16U icp_adf_get_busAddress(Cpa16U packageId)
{
    return icp_adf_cfgGetBusAddress(packageId);
}

/*
 * icp_adf_get_kptAcHandle
 *
 *  This function gets the kpt achandle of the device.
 */
Cpa32U icp_adf_get_kptAcHandle(Cpa16U packageId)
{
    return icp_adf_cfgGetKptAcHandle(packageId);
}

/*
 * adf_reset_userProxy
 *
 *  Function to reset the ADF proxy status in user space.
 */
void adf_reset_userProxy(void)
{
    init_ctr = 0;
    osalAtomicSet(0, &process_proxy_status);
    process_info_file = -1;
    /* there is no option to reset the mutex, hence destroying
     * it and re-initializing. */
    if (processes_lock)
        osalMutexDestroy(&processes_lock);
    osalMutexInit(&processes_lock);
}
