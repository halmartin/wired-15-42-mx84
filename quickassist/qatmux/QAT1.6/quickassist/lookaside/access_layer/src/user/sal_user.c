/***************************************************************************
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
 ***************************************************************************/

/**
 *****************************************************************************
 * @file sal_user.c
 *
 * @defgroup SalUser
 *
 * @description
 *    This file contains implementation of functions to start/stop user process
 *
 *****************************************************************************/

/* QAT-API includes */
#include "cpa.h"

/* Osal includes */
#include "Osal.h"

/* ADF includes */
#include "icp_adf_init.h"
#include "icp_accel_devices.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_user_proxy.h"
#include "icp_adf_transport.h"
#include "icp_adf_cfg.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_la.h"

/* SAL includes */
#include "icp_sal_user.h"
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_list.h"
#ifndef ICP_DC_ONLY
#include "lac_sal_types_crypto.h"
#endif
#include "sal_types_compression.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"

#define ICP_MUX_DIRECTORY "icp_mux/"

static pthread_mutex_t sync_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sync_multi_lock = PTHREAD_MUTEX_INITIALIZER;
static char multi_section_name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
static int start_ref_count = 0;

#ifndef ICP_DC_ONLY
extern int lacSymDrbgLock_init(void);
extern void lacSymDrbgLock_exit(void);
#endif

CpaStatus icp_sal_userStartMultiProcess(const char *pProcessName,
                                        CpaBoolean limitDevAccess)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char sectionName[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
    char tmpName[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};

    if (ADF_CFG_MAX_SECTION_LEN_IN_BYTES <= strlen(pProcessName))
    {
        LAC_LOG_ERROR("Process name too long\n");
        return CPA_STATUS_FAIL;
    }
    if (CPA_TRUE == limitDevAccess)
    {
        if (ADF_CFG_MAX_SECTION_LEN_IN_BYTES <= strlen(pProcessName) +
                                        strlen(DEV_LIMIT_CFG_ACCESS_TMPL))
        {
            LAC_LOG_ERROR("Process name too long\n");
            return CPA_STATUS_FAIL;
        }
        strncpy(tmpName, DEV_LIMIT_CFG_ACCESS_TMPL,
                                       ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
        strncat(tmpName, pProcessName, ADF_CFG_MAX_SECTION_LEN_IN_BYTES
                - strlen(tmpName) - 1);
    }
    else
    {
        if (ADF_CFG_MAX_SECTION_LEN_IN_BYTES <= strlen(pProcessName))
        {
            LAC_LOG_ERROR("Process name too long\n");
            return CPA_STATUS_FAIL;
        }
        strncpy(tmpName, pProcessName, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
    }
    if(pthread_mutex_lock(&sync_multi_lock))
    {
        LAC_LOG_ERROR("Mutex lock failed\n");
        return CPA_STATUS_FAIL;
    }
    if(0 == start_ref_count)
    {
            status = icp_adf_userProcessToStart(tmpName, sectionName);
            if(CPA_STATUS_SUCCESS != status)
            {
                LAC_LOG_ERROR("icp_adf_userProcessToStart failed\n");
                pthread_mutex_unlock(&sync_multi_lock);
                return CPA_STATUS_FAIL;
            }
            strncpy(multi_section_name, sectionName,
                    ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
    }
    status = icp_sal_userStart(multi_section_name);
    if(pthread_mutex_unlock(&sync_multi_lock))
    {
        LAC_LOG_ERROR("Mutex unlock failed\n");
        return CPA_STATUS_FAIL;
    }
    return status;
}

static CpaStatus do_userStart(const char *pProcessName)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    OSAL_STATUS osal_status = OSAL_SUCCESS;
    status = icpSetProcessName(pProcessName);
    LAC_CHECK_STATUS(status);
#ifndef ICP_DC_ONLY
    osal_status = lacSymDrbgLock_init();
    LAC_CHECK_STATUS(osal_status);
#endif
    status = SalCtrl_AdfServicesRegister();
    LAC_CHECK_STATUS(status);
#ifdef WITH_CPA_MUX
    osal_status = osalMemInitialize(ICP_MUX_DIRECTORY);
#else
    osal_status = osalMemInitialize(NULL);
#endif
    if (OSAL_SUCCESS != osal_status)
    {
       LAC_LOG_ERROR("Failed to initialize memory\n");
       SalCtrl_AdfServicesUnregister();
       return CPA_STATUS_FAIL;
    }
    status = icp_adf_userProxyInit(pProcessName);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to initialize proxy\n");
        SalCtrl_AdfServicesUnregister();
        osalMemDestroy();
        return status;
    }
    status = SalCtrl_AdfServicesStartedCheck();
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to start services\n");
        SalCtrl_AdfServicesUnregister();
        osalMemDestroy();
    }
    return status;
}

CpaStatus icp_sal_userStart(const char *pProcessName)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    if(pthread_mutex_lock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex lock failed\n");
        return CPA_STATUS_FAIL;
    }
    if(0 == start_ref_count)
    {
        status = do_userStart(pProcessName);
    }
    if(CPA_STATUS_SUCCESS == status)
    {
        start_ref_count += 1;
    }
    if(pthread_mutex_unlock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex unlock failed\n");
        return CPA_STATUS_FAIL;
    }
    return status;

}

static CpaStatus do_userStop()
{
    CpaStatus status = SalCtrl_AdfServicesUnregister();

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to unregister\n");
        return status;
    }

    status = icp_adf_userProxyShutdown();
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to shutdown proxy\n");
        return status;
    }
#ifndef ICP_DC_ONLY
    lacSymDrbgLock_exit();
#endif
    icp_adf_userProcessStop();
    osalMemDestroy();
    return status;
}

CpaStatus icp_sal_userStop()
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    if(pthread_mutex_lock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex lock failed\n");
        return CPA_STATUS_FAIL;
    }
    start_ref_count -= 1;
    if(0 == start_ref_count)
    {
        status = do_userStop();
    }
    if(0 > start_ref_count)
    {
        start_ref_count = 0;
    }
    memset(multi_section_name, '\0',
                    ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
    if(pthread_mutex_unlock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex unlock failed\n");
        return CPA_STATUS_FAIL;
    }
    return status;
}

CpaStatus icp_sal_find_new_devices(void)
{
    return icp_adf_find_new_devices();
}

CpaStatus icp_sal_poll_device_events(void)
{
    return icp_adf_poll_device_events();
}

CpaStatus  icp_sal_check_device(Cpa32U accelId)
{
    return icp_adf_check_device(accelId);
}

CpaStatus  icp_sal_check_all_devices(void)
{
    return icp_adf_check_all_devices();
}

/*
 * @ingroup SalUser
 * @description
 *      This is a function to reset the device
 *
 * @context
 *      None
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * param[in]  accelId Accelerator Device Id
*/
CpaStatus icp_sal_reset_device(Cpa32U accelId)
{
    return icp_adf_reset_device(accelId);
}

/*
 * @ingroup SalUser
 * @description
 *      This is a function to send messages to VF.
 *      It should only be called on the PF/Host.
 *
 * @context
 *      None
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * param[in]  accelId Accelerator Device Id
 * param[in]  vfNum   VFNumber (1-32) to which message should be sent.
 * param[in]  message 14-bit message to be sent
*/
CpaStatus icp_sal_userSendMsgToVf(Cpa32U accelId, Cpa32U vfNum, Cpa32U message)
{
     return icp_adf_send_msg_to_vf(accelId, vfNum,  message );
}


/*
 * @ingroup SalUser 
 * @description
 *      This is a function to send messages to PF.
 *      It should only be called on a VM/Guest.
 * @context
 *      None
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * param[in]  accelId Accelerator Device Id
 * param[in]  message 14-bit message to be sent
 *
*/

CpaStatus icp_sal_userSendMsgToPf(Cpa32U accelId, Cpa32U message )
{
    return icp_adf_send_msg_to_pf(accelId, message );
}


/*
 * @ingroup SalUser
 * @description
 *      This is a function to get a message from a VF.
 *      It should only be called on the PF/Host.
 *
 * @context
 *      None
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * param[in]   accelId Accelerator Device Id
 * param[in]   vfNum   vf number (1-32)
 * param[out]  message 14-bit Message will be stored here if available
 * param[out]  messageCounter Number of messages received on this channel
 *                            since API last called.
 *                               0 => No new message
 *                               1 => One message available
 *                               n (>1) => Last message available,
 *                               but missed n-1 messages. As only the last
 *                               message (per device and per VF)
 *                               is stored a message could be missed if the
 *                               API is not called often enough.
 *
*/
CpaStatus icp_sal_userGetMsgFromVf(Cpa32U accelId, Cpa32U vfNum,
                                   Cpa32U *message, Cpa32U *messageCounter)
{
    return icp_adf_get_msg_from_vf(accelId,vfNum, message,messageCounter);
}


/*
 * @ingroup SalUser
 * @description
 *      This is a function to get a message from the PF.
 *      It should only be called on a VM/Guest.
 *
 * @context
 *      None
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * param[in]   accelId Accelerator Device Id
 * param[out]  message 14-bit Message will be stored here if available
 * param[out]  messageCounter Number of messages received on this channel
 *                            since API last called.
 *                               0 => No new message
 *                               1 => One message available
 *                               n (>1) => Last message available,
 *                               but missed n-1 messages. As only the last
 *                               message (per device) is stored a message
 *                               could be missed if the
 *                               API is not called often enough.
 *
*/
CpaStatus icp_sal_userGetMsgFromPf(Cpa32U accelId, Cpa32U * message,
                                   Cpa32U *messageCounter)
{
    return icp_adf_get_msg_from_pf(accelId,message,messageCounter);
}

/*
 * @ingroup SalUser
 * @description
 *      This is a function to get pfvf comms status
 *
 * @context
 *      None
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * param[out]  unreadMessage: CpaTrue if at least one message is available
 *             which hasn't been returned in a call to icp_get_msg_from_p/vf
*/
CpaStatus icp_sal_userGetPfVfcommsStatus(CpaBoolean *unreadMessage)
{
    return icp_adf_get_pfvfcomms_status(unreadMessage);
}
