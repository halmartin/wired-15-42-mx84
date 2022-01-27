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
 *  version: QAT1.6.L.2.6.0-65
 *
 *****************************************************************************/

/**
 ***************************************************************************
 * @file sal_qat_ctrl.c     Qat instance handling functions
 *
 * @ingroup SalCtrl
 *
 ***************************************************************************/
/* Include cpa header file. */
#include "cpa.h"

/* Log and Osal includes. */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "Osal.h"
#include "lac_list.h"

/* ADF includes */
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_ae.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_init_admin.h"

/* SAL includes */
#include "lac_sal_types.h"
#include "lac_sal_types_qat_ctrl.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"

/* QAT includes */
#include "qat_ctrl.h"
#include "qat_init.h"
#include "qat_rings.h"
#include "qat_admin_common.h"
#include "qat_admin.h"
#include "sal_statistics.h"
#include "sal_string_parse.h"
#include "qat_initadmin_dh895x.h"

/* Proc debug defines */
#define DEBUG_QAT_NAME                      "qat"
#define DEBUG_QAT_NAME_LEN                  4
#define HEARTBEAT_QAT_NAME                  "heartbeat"
#define HEARTBEAT_QAT_NAME_LEN              10
#define HEARTBEAT_QUERY_DELAY_MS            100
#define HEARTBEAT_QUERY_DELAY_MULTIPLIER    2
#define HEARTBEAT_QUERY_RETRY               5

#define WIRELESS_ENABLED "WirelessEnabled"

STATIC CpaStatus QatCtrl_QueryHeartbeat(sal_qat_service_t *qatInstance,
                                        CpaStatus *heartbeatStatus)
{
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32U retry = 0;
    Cpa32U delay = 0;

    delay = HEARTBEAT_QUERY_DELAY_MS;

    for (retry=0; retry < HEARTBEAT_QUERY_RETRY; retry++)
    {
        status = QatCtrl_SendHeartbeat_dh895x(qatInstance,
                                              delay,
                                              heartbeatStatus);
        if (CPA_STATUS_RESOURCE == status)
        {
            /* Process received signal */
            status = CPA_STATUS_SUCCESS;
            *heartbeatStatus = CPA_STATUS_SUCCESS;
            break;
        }
        else if (CPA_STATUS_SUCCESS != status)
        {
            status = CPA_STATUS_FAIL;
            *heartbeatStatus = CPA_STATUS_FAIL;
            break;
        }

        if (CPA_STATUS_SUCCESS == *heartbeatStatus)
        {
            /* Heartbeat OK */
            break;
        }

        if (CPA_FALSE == SalCtrl_getHeartbeatStatus(qatInstance))
        {
            /* Heartbeat shutdown */
            break;
        }

        delay = delay * HEARTBEAT_QUERY_DELAY_MULTIPLIER;
    }

    return status;
}

/*
 * Prints statistics for a QAT instance
 */
int QatCtrl_Debug(void* private_data, char* data, int size, int offset)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus deviceStatus = CPA_STATUS_SUCCESS;
    qat_admin_stats_t qatStats = {NULL, NULL};
    sal_qat_service_t *qatInstance = (sal_qat_service_t*)private_data;
    Cpa32S len = 0;
    int i = 0;
    Cpa16U numAEs = qatInstance->num_aes;

    if ((SAL_SERVICE_STATE_RUNNING != qatInstance->generic_service_info.state)||
	     (CPA_FALSE == icp_adf_isDevStarted(qatInstance->debug->accel_dev)))
    {
        len += snprintf(data+len, size-len, "Device not started\n");
        return 0;
    }

    /* Alloc memory for the stats */
    status = LAC_OS_CAMALLOC(&(qatStats.numSent),
                            (numAEs)
                            * sizeof(*(qatStats.numSent)),
                            LAC_64BYTE_ALIGNMENT, qatInstance->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate qatInstance->numSent memory\n");
        return 0;
    }

    status = LAC_OS_CAMALLOC(&(qatStats.numRec),
                            (numAEs)
                            * sizeof(*(qatStats.numRec)),
                            LAC_64BYTE_ALIGNMENT, qatInstance->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate qatInstance->numRec memory\n");
        LAC_OS_CAFREE(qatStats.numSent);
        return 0;
    }

    /* Zero the stats */
    for (i=0; i < qatInstance->num_aes; i++)
    {
        qatStats.numRec[i] = 0;
        qatStats.numSent[i] = 0;
    }
    status = QatCtrl_FWCountGet_dh895x(qatInstance, &qatStats);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_OS_CAFREE(qatStats.numSent);
        LAC_OS_CAFREE(qatStats.numRec);

        if (CPA_STATUS_RESOURCE == status)
        {
            return 0;
        }

        LAC_LOG_ERROR("Failed to get counters from Firmware\n");

#ifdef ICP_AUTO_DEVICE_RESET_ON_HB
        LAC_LOG_ERROR("Device will be restarted\n");

        len += snprintf(data+len, size-len,
                      " ERROR: Qat is not responding and will be restarted\n");
        status = icp_adf_reset_dev(qatInstance->debug->accel_dev,
                                        ICP_ADF_DEV_RESET_ASYNC);
#else
        len += snprintf(data+len, size-len,
                 " ERROR: Qat is not responding. Please restart the device\n");
#endif
        return 0;
    }

    len = snprintf(data+len, size-len,
                   SEPARATOR
                   BORDER " Statistics for Qat Instance %u                    "
                   BORDER"\n"
                   SEPARATOR,
                   qatInstance->generic_service_info.instance );

    for (i=0; i< qatInstance->num_aes; i++)
    {
        len += snprintf(data+len, size-len,
                       BORDER " Firmware Requests[AE %d]:        %16llu "
                       BORDER "\n"
                       BORDER " Firmware Responses[AE %d]:       %16llu "
                       BORDER "\n"
                       SEPARATOR,
                       i, (long long unsigned int)qatStats.numRec[i],
                       i, (long long unsigned int)qatStats.numSent[i]);
    }

    /*
     * Run an Heartbeat query (feature compatibility between devices)
     */
    if (CPA_TRUE == SalCtrl_getHeartbeatStatus(qatInstance))
    {
        status = QatCtrl_QueryHeartbeat(qatInstance, &deviceStatus);
        if (CPA_STATUS_SUCCESS != deviceStatus)
        {
#ifdef ICP_AUTO_DEVICE_RESET_ON_HB
            len += snprintf(data+len, size-len,
                    " ERROR: Qat is not responding and will be restarted\n");
            status = icp_adf_reset_dev(qatInstance->debug->accel_dev,
                                            ICP_ADF_DEV_RESET_ASYNC);
#else
            len += snprintf(data+len, size-len,
                    " ERROR: Qat is not responding. Please restart the device\n");
#endif
        }
        else if (CPA_STATUS_SUCCESS != status)
        {
            len += snprintf(data+len, size-len,
                    " ERROR: Heartbeat query failed\n");
        }
    }

    LAC_OS_CAFREE(qatStats.numSent);
    LAC_OS_CAFREE(qatStats.numRec);
    return 0;
}

/*
 * Reports the result of an Heartbeat query
 */
int QatCtrl_Heartbeat(void* private_data, char* data, int size, int offset)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus deviceStatus = CPA_STATUS_SUCCESS;
    sal_qat_service_t *qatInstance = (sal_qat_service_t*)private_data;
    Cpa32S len = 0;

    if (SAL_SERVICE_STATE_RUNNING != qatInstance->generic_service_info.state)
    {
        len += snprintf(data+len, size-len, "Device not started\n");
        return 0;
    }

    if (CPA_FALSE == SalCtrl_getHeartbeatStatus(qatInstance))
    {
        len += snprintf(data+len, size-len, "Heartbeat feature is not enabled\n");
        return 0;
    }
    status = QatCtrl_QueryHeartbeat(qatInstance, &deviceStatus);
    if (CPA_STATUS_SUCCESS != deviceStatus)
    {
        LAC_LOG_ERROR("Heartbeat failed.\n");
        LAC_LOG_ERROR("Device needs to be restarted.\n");

#ifdef ICP_AUTO_DEVICE_RESET_ON_HB
        len += snprintf(data+len, size-len,
                      " ERROR: Qat is not responding and will be restarted\n");
        status = icp_adf_reset_dev(qatInstance->debug->accel_dev,
                                        ICP_ADF_DEV_RESET_ASYNC);
#else
        len += snprintf(data+len, size-len,
                 " ERROR: Qat is not responding. Please restart the device\n");
#endif
        return 0;
    }
    else if (CPA_STATUS_SUCCESS != status)
    {
        len += snprintf(data+len, size-len,
                 " ERROR: Heartbeat query failed\n");
        return 0;
    }

    len += snprintf(data+len, size-len, "Device up and running!\n");

    return 0;
}

CpaStatus SalCtrl_enableDebug(sal_qat_service_t *qat_instance, icp_accel_dev_t *device)
{
    CpaStatus status;

    status = LAC_OS_MALLOC(&qat_instance->debug,
                          sizeof(sal_service_debug_t));
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create service debug handler.\n");
        return status;
    }

    osalMemSet(qat_instance->debug, 0, sizeof(sal_service_debug_t));
    status = LAC_OS_MALLOC(&qat_instance->debug->debug_file.name,
                                                DEBUG_QAT_NAME_LEN);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to create service debug handler.\n");
        return status;
    }

    snprintf(qat_instance->debug->debug_file.name, DEBUG_QAT_NAME_LEN,
                DEBUG_QAT_NAME);

    qat_instance->debug->debug_file.seq_read = QatCtrl_Debug;
    qat_instance->debug->debug_file.private_data = qat_instance;
    qat_instance->debug->accel_dev = device;

    status = icp_adf_debugAddFile(device,
                  &qat_instance->debug->debug_file);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to add service debug handler.\n");
        LAC_OS_FREE(qat_instance->debug->debug_file.name);
        LAC_OS_FREE(qat_instance->debug);
        return status;
    }
    return status;
}

/*
 * Exposes the Heartbeat feature through the proc filesystem
 */
CpaStatus SalCtrl_enableHeartbeat(sal_qat_service_t *qat_instance, icp_accel_dev_t *device)
{
    CpaStatus status;

    status = LAC_OS_MALLOC(&qat_instance->heartbeat,
                          sizeof(sal_service_debug_t));
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate memory for heartbeat structure.\n");
        return status;
    }

    osalMemSet(qat_instance->heartbeat, 0, sizeof(sal_service_debug_t));
    status = LAC_OS_MALLOC(&qat_instance->heartbeat->debug_file.name,
                                                HEARTBEAT_QAT_NAME_LEN);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate memory for heartbeat proc fs name.\n");
        LAC_OS_FREE(qat_instance->heartbeat);
        return status;
    }

    /* Initialize Heartbeat Lock */
    osalMutexInit(&qat_instance->heartbeatLock);

    snprintf(qat_instance->heartbeat->debug_file.name, HEARTBEAT_QAT_NAME_LEN,
                HEARTBEAT_QAT_NAME);

    qat_instance->heartbeat->debug_file.seq_read = QatCtrl_Heartbeat;
    qat_instance->heartbeat->debug_file.private_data = qat_instance;
    qat_instance->heartbeat->accel_dev = device;

    status = icp_adf_debugAddFile(device,
                  &qat_instance->heartbeat->debug_file);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to add service debug handler.\n");
        LAC_OS_FREE(qat_instance->heartbeat->debug_file.name);
        LAC_OS_FREE(qat_instance->heartbeat);
        osalMutexDestroy(&qat_instance->heartbeatLock);
        return status;
    }

    return status;
}

/*
 * Disables the Heartbeat feature
 */
void SalCtrl_stopHeartbeat(sal_qat_service_t *qat_instance)
{

    if (CPA_TRUE == SalCtrl_getHeartbeatStatus(qat_instance))
    {
        /* Wait till the lock is acquired to ensure nobody is
         * polling the Heartbeat */
        if (osalMutexLock(&qat_instance->heartbeatLock, OSAL_WAIT_FOREVER))
        {
            LAC_LOG_ERROR("Acquire Heartbeat Mutex failed.");
            return;
        }

        /* Set Heartbeat status to FALSE */
        SalCtrl_setHeartbeatStatus(qat_instance, CPA_FALSE);

        /* Release the mutex */
        osalMutexUnlock(&qat_instance->heartbeatLock);
    }
}

/*
 * Shutdowns the Heartbeat feature
 */
void SalCtrl_shutdownHeartbeat(sal_qat_service_t *qat_instance)
{

    if (qat_instance->heartbeat &&
           qat_instance->heartbeat->debug_file.name)
    {
        icp_adf_debugRemoveFile(&qat_instance->heartbeat->debug_file);
        LAC_OS_FREE(qat_instance->heartbeat->debug_file.name);
        LAC_OS_FREE(qat_instance->heartbeat);
    }

    if (qat_instance->heartbeatLock)
    {
        osalMutexDestroy(&qat_instance->heartbeatLock);
    }
}

/*
 * Returns a boolean which reports the Heartbeat status
 */
CpaBoolean SalCtrl_getHeartbeatStatus(sal_qat_service_t *qat_instance)
{
    CpaBoolean heartbeatEnabled = CPA_FALSE;

    if (0 == osalAtomicGet(&qat_instance->heartbeatEnabled))
    {
        heartbeatEnabled = CPA_FALSE;
    }
    else
    {
        heartbeatEnabled = CPA_TRUE;
    }

    return heartbeatEnabled;
}

/*
 * Sets the Heartbeat status (enabled/disabled)
 */
void SalCtrl_setHeartbeatStatus(sal_qat_service_t *qat_instance,
                                CpaBoolean heartbeatEnabled)
{
    if (CPA_TRUE == heartbeatEnabled)
    {
        osalAtomicSet(1, &qat_instance->heartbeatEnabled);
    }
    else
    {
        osalAtomicSet(0, &qat_instance->heartbeatEnabled);
    }

}

