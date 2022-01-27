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

/* LAC and Osal includes. */
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
#ifndef ICP_DC_ONLY
#include "lac_sym_qat_constants_table.h"
#endif

#define WIRELESS_ENABLED "WirelessEnabled"

/*
 * Free all memory allocated for the QAT instance
 */
void SalCtrl_QatClean_dh895x(sal_qat_service_t *qat_instance)
{
    LAC_OS_CAFREE(qat_instance->qatAeMask);
    LAC_OS_CAFREE(qat_instance->aeTargetIds);
}

/*
 * The Init function takes care of initializing all
 * of the memory and arrays that will be needed to enable
 * the AEs on this QAT accelerator.
 */
CpaStatus
SalCtrl_QatInit_dh895x(icp_accel_dev_t* device,
                sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char temp_value[SAL_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    sal_qat_service_t *qat_instance = (sal_qat_service_t*) service;
    sal_statistics_collection_t *pStatsCollection =
        (sal_statistics_collection_t*)device->pQatStats;
    Cpa32U me_num = 0;
    Cpa32S index = 0;
    icp_comms_trans_handle *tmp_handle;
    Cpa32U msgSize=0;
    CpaBoolean wireless_enabled = CPA_FALSE;

    if (SAL_SERVICE_STATE_UNINITIALIZED !=
          qat_instance->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call init\n");
        return CPA_STATUS_FAIL;
    }

    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_INITIALIZING;

    status = SalCtrl_getNumberOfAE(device,
                                   &(qat_instance->num_aes));
    LAC_CHECK_STATUS_LOG(status, "Failed to get number of accel engines");

    /*
     * We will also need to know the Node Id so that we can
     * allocate memory from the correct node.
     */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_NODE_ID,
                                      temp_value);
    LAC_CHECK_STATUS(status);
    qat_instance->nodeId = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);

    /* Get max AEs */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_MAX_AE,
                                      temp_value);
    LAC_CHECK_STATUS(status);
    qat_instance->max_aes = Sal_Strtoul(temp_value, NULL, ADF_CFG_BASE_DEC);

    /* Get the skuing information */
    status = LAC_OS_CAMALLOC(&(qat_instance->qatAeMask),
                            1 * sizeof(*(qat_instance->qatAeMask)),
                            LAC_64BYTE_ALIGNMENT, qat_instance->nodeId);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to allocate master qatAeMask memory\n");
        return CPA_STATUS_RESOURCE;
    }

    /* Get Response msgSize and store it */
    status = icp_adf_cfgGetParamValue(device,
            LAC_CFG_SECTION_GENERAL,
            ICP_CFG_FW_MSG_SIZE_ADMIN_RX_KEY,
            temp_value);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Problem getting ADMIN_RX string from config table");
        SalCtrl_QatClean_dh895x(qat_instance);
        return CPA_STATUS_FAIL;
    };
    msgSize = Sal_Strtoul(temp_value, NULL, SAL_CFG_BASE_DEC);

    if ((msgSize != sizeof(icp_qat_fw_init_admin_resp_t))
        ||
        (msgSize != QAT_DH895X_ADMIN_MSG_SZ))
    {
        /* Data in config table (or in .conf file) is out of sync with
         * this version of SAL. */
        LAC_LOG_ERROR1("Admin response size is out of sync."
                " This SAL expects admin responses of %d bytes",
                QAT_DH895X_ADMIN_MSG_SZ);
        LAC_LOG_ERROR2("The FW expects admin responses of %d bytes, "
                "The transport layer expects responses of %d bytes",
                sizeof(icp_qat_fw_init_admin_resp_t),
                msgSize);

        SalCtrl_QatClean_dh895x(qat_instance);
        return CPA_STATUS_FAIL;
    }
    qat_instance->qat_admin_rx_msg_size = msgSize;

    /* Get Request msgSize, store it and pass in to createHandle */
    status = icp_adf_cfgGetParamValue(device,
            LAC_CFG_SECTION_GENERAL,
            ICP_CFG_FW_MSG_SIZE_ADMIN_TX_KEY,
            temp_value);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Problem getting ADMIN_TX string from config table");
        SalCtrl_QatClean_dh895x(qat_instance);
        return CPA_STATUS_FAIL;
    };
    msgSize = Sal_Strtoul(temp_value, NULL, SAL_CFG_BASE_DEC);

    if ((msgSize != sizeof(icp_qat_fw_init_admin_req_t))
            ||
            (msgSize != QAT_DH895X_ADMIN_MSG_SZ))
    {
        /* Data in config table (or in .conf file) is out of sync with
         * this version of SAL. */
        LAC_LOG_ERROR1("Admin request size is out of sync."
                " This SAL expects admin requests of %d bytes",
                QAT_DH895X_ADMIN_MSG_SZ);
        LAC_LOG_ERROR2("The FW expects admin requests of %d bytes, "
                "The transport layer expects requests of %d bytes",
                sizeof(icp_qat_fw_init_admin_req_t),
                msgSize);

        SalCtrl_QatClean_dh895x(qat_instance);
        return CPA_STATUS_FAIL;
    }
    qat_instance->qat_admin_tx_msg_size = msgSize;

    /* Setup the init/admin interface by using the ICP_TRANS_TYPE_ADMINREG
     * type. This uses mailbox registers for init/admin rather than "admin"
     * rings */

    for (me_num = 0; me_num < qat_instance->num_aes; me_num++)
    {
        status = icp_adf_transCreateHandle(device,
                                           ICP_TRANS_TYPE_ADMINREG,
                                           NULL,
                                           me_num,
                                           0,
                                           "Admin Interface",
                                           ICP_ADF_RING_SERVICE_0,
                                           NULL,
                                           ICP_RESP_TYPE_NONE,
                                           1,
                                           msgSize,
                                           (icp_comms_trans_handle *)
                        &(qat_instance->trans_handle_qat_admin_tx[me_num]));
        if (status != CPA_STATUS_SUCCESS)
        {
            LAC_LOG_ERROR("Failed to create Admin Tx handle\n");
            SalCtrl_QatClean_dh895x(qat_instance);

            /* Release the previously allocated handles */
            for(index=me_num-1; index>=0; index--)
            {
                tmp_handle = (icp_comms_trans_handle *)
                             &(qat_instance->trans_handle_qat_admin_tx[index]);

                icp_adf_transReleaseHandle(tmp_handle);
                *tmp_handle = NULL;
            }
            return CPA_STATUS_FAIL;
        }
    }
    /* Get the Wireless Enable/Disable config, valid values: 1,0 */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      WIRELESS_ENABLED,
                                      temp_value);
    if (CPA_STATUS_SUCCESS == status)
    {
        wireless_enabled = ((Sal_Strtoul
            (temp_value, NULL, SAL_CFG_BASE_DEC)) > 0 ? CPA_TRUE:CPA_FALSE);
    }
    else
    {
        status = CPA_STATUS_SUCCESS;
    }
    if (!wireless_enabled)
    {
        status = SalCtrl_enableHeartbeat(qat_instance, device);
        if (status != CPA_STATUS_SUCCESS)
        {
            /*
            * If we got a failure we try to continue with init,
            * heartbeat feature will be not available
            */
            SalCtrl_setHeartbeatStatus(qat_instance, CPA_FALSE);
           
            LAC_LOG_ERROR("Heartbeat feature not available\n");
           
            status = CPA_STATUS_SUCCESS;
        }
        else
        {
            /* Heartbeat is enabled */
            SalCtrl_setHeartbeatStatus(qat_instance, CPA_TRUE);
        }
       
        if(CPA_TRUE == pStatsCollection->bStatsEnabled)
        {
            status = SalCtrl_enableDebug(qat_instance, device);
            if (CPA_STATUS_SUCCESS != status)
            {
                LAC_LOG_ERROR("Failed to create service debug handler.\n");
                SalCtrl_QatClean_dh895x(qat_instance);
                /* Release the allocated handles */
                for(me_num = 0; me_num < qat_instance->num_aes; me_num++)
                {
                    tmp_handle = (icp_comms_trans_handle *)
                             &(qat_instance->trans_handle_qat_admin_tx[index]);
                   
                    icp_adf_transReleaseHandle(tmp_handle);
                }
                return status;
            }
        }
    }
    qat_instance->generic_service_info.stats = pStatsCollection;
    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_INITIALIZED;

    /* Initialize the admin callback counter to zero. */
    osalAtomicSet(0,
                  &(qat_instance->adminCallbackPending));

    qat_instance->num_aes_per_qat = 0;
    qat_instance->max_num_rings_per_qat = 0;
    qat_instance->serviceMask = NULL;
    qat_instance->max_qat = 0;
    qat_instance->active_aes_per_qat = 0;

    return status;
}

/*
 * The start function sends the init messages to the AEs.
 */
CpaStatus SalCtrl_QatStart_dh895x(icp_accel_dev_t* device,
                        sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_qat_service_t* qat_instance = (sal_qat_service_t*) service;
#ifndef ICP_DC_ONLY
#ifndef TRNG_DISABLED
    Cpa32U capabilitiesMask = 0;
    Cpa32U wireless_enabled = 0;
    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
#endif
#endif
    if (SAL_SERVICE_STATE_INITIALIZED !=
         qat_instance->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call start\n");
        return CPA_STATUS_FAIL;
    }

#ifndef ICP_DC_ONLY
#ifdef TRNG_DISABLED
    LAC_LOG("TRNG is disabled\n");
#else
    /******************************************************/
    /* Send TRNG_ENABLE msg to AEs */
    /******************************************************/

    /* Get the number of Wireless Procs */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      WIRELESS_ENABLED,
                                      adfGetParam);
    if (CPA_STATUS_SUCCESS == status)
    {
        wireless_enabled = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    }
    else
    {
        wireless_enabled = 0;
        status = CPA_STATUS_SUCCESS;
    }

    LAC_LOG_DEBUG1("WirelessEnabled: %d \n", wireless_enabled);

    if (SalCtrl_IsServiceSupported(device, SAL_SERVICE_TYPE_CRYPTO)
            && (wireless_enabled == 0))
    {
        /* Not running wireless firmware, so we send TRNG_ENABLE message */
        status = icp_amgr_getAccelDevCapabilities(device,
                                                  &capabilitiesMask);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to get device capabilities\n");
            return status;
        }


        if (capabilitiesMask & ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER)
        {
            /*
             * Send the TRNG_ENABLE message
             * A TRNG_ENABLE msg will be sent to each AE
             * per QAT instance that has crypto enabled.
             */
            status = QatCtrl_SendAdminCmd_dh895x(qat_instance,
                                            ICP_QAT_FW_TRNG_ENABLE);
            if (CPA_STATUS_SUCCESS != status)
            {
                LAC_LOG_ERROR("Sending TRNG_ENABLE msg Failed\n");
                return status;
            }
        }
    }
#endif

    /******************************************************/
    /* Send Constants_CFG msg to AEs */
    /******************************************************/
    if (SalCtrl_IsServiceSupported(device, SAL_SERVICE_TYPE_CRYPTO))
    {
        CpaBoolean use_table = CPA_TRUE;

        status = LacSymQat_ConstantsInitTableForFW(device,
                                                   &use_table);
        if (CPA_STATUS_SUCCESS != status)
        {
            return CPA_STATUS_FAIL;
        }

        if (use_table)
        {
            status = QatCtrl_SendAdminCmd_dh895x(qat_instance, ICP_QAT_FW_CONSTANTS_CFG);
            if (status != CPA_STATUS_SUCCESS)
            {
                LAC_LOG_ERROR("Failed to send CONSTANTS_CFG messages\n");
                return CPA_STATUS_FAIL;
            }
        }

        LacSymQat_ConstantsReleaseTableForFW();

    }
#endif

    /******************************************************/
    /* Send INIT_ME msg to AEs */
    /******************************************************/
    status = QatCtrl_SendAdminCmd_dh895x(qat_instance, ICP_QAT_FW_INIT_ME);
    if (status != CPA_STATUS_SUCCESS)
    {
        LAC_LOG_ERROR("Failed to send INIT_ME messages\n");
        return CPA_STATUS_FAIL;
    }

    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_RUNNING;
    return status;
}

/*
 * The Stop function sends the TRNG disable msg to each AE
 * in this QAT instance.
 */
CpaStatus
SalCtrl_QatStop_dh895x(icp_accel_dev_t* device, sal_service_t* service)
{
#ifndef ICP_DC_ONLY
#ifndef TRNG_DISABLED
    Cpa32U wireless_enabled = 0;
    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U capabilitiesMask  = 0;
#endif
#endif
    sal_qat_service_t* qat_instance = (sal_qat_service_t*) service;


    if (SAL_SERVICE_STATE_RUNNING !=
            qat_instance->generic_service_info.state)
    {
        LAC_LOG_ERROR("Not in the correct state to call stop");
    }

#ifndef ICP_DC_ONLY
#ifdef TRNG_DISABLED
    LAC_LOG("TRNG is disabled\n");
#else
    /* Get the number of Wireless Procs */
    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      WIRELESS_ENABLED,
                                      adfGetParam);
    if (CPA_STATUS_SUCCESS == status)
    {
        wireless_enabled = Sal_Strtoul(adfGetParam, NULL, SAL_CFG_BASE_DEC);
    }
    else
    {
        wireless_enabled = 0;
        status = CPA_STATUS_SUCCESS;
    }

    LAC_LOG_DEBUG1("WirelessEnabled: wireless_enabled %d \n", wireless_enabled);

    if (wireless_enabled == 0)
    {
        /* Not running wireless firmware, so we send TRNG_DISABLE message */
        status = icp_amgr_getAccelDevCapabilities(device,
                                                    &capabilitiesMask);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to get device capabilities\n");
        }

        if (capabilitiesMask & ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER)
        {
            /*
             * Send the TRNG_DISABLE message to each AE
             * per QAT instance that has crypto enabled
             */
           status = QatCtrl_SendAdminCmd_dh895x(qat_instance,
                   ICP_QAT_FW_TRNG_DISABLE);
           if (CPA_STATUS_SUCCESS != status)
           {
               LAC_LOG_ERROR("Sending TRNG_DISABLE msg Failed\n");
           }
        }
    }
#endif
#endif
    /* Disable Heartbeat */
    SalCtrl_stopHeartbeat(qat_instance);

    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_SHUTTING_DOWN;
    /* If there are admin messages in-flight return RETRY */
    if (osalAtomicGet(&(qat_instance->adminCallbackPending)) &&
                                            (!icp_adf_is_dev_in_reset(device)))
    {
        return CPA_STATUS_RETRY;
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * The shutdown releases the rings and frees all the memory allocated
 * during the init.
 */
CpaStatus
SalCtrl_QatShutdown_dh895x(icp_accel_dev_t* device, sal_service_t* service)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus ret_status = CPA_STATUS_SUCCESS;
    Cpa32U me_num;
    sal_qat_service_t* qat_instance = (sal_qat_service_t*) service;
    sal_statistics_collection_t *pStatsCollection =
    (sal_statistics_collection_t*)device->pQatStats;

    if ((SAL_SERVICE_STATE_INITIALIZED !=
          qat_instance->generic_service_info.state) &&
        (SAL_SERVICE_STATE_SHUTTING_DOWN !=
         qat_instance->generic_service_info.state))
    {
        LAC_LOG_ERROR("Not in the correct state to call shutdown \n");
        return CPA_STATUS_FAIL;
    }

    for (me_num = 0; me_num < qat_instance->num_aes; me_num++)
    {
        status = icp_adf_transReleaseHandle(
                     qat_instance->trans_handle_qat_admin_tx[me_num]);

        if (status != CPA_STATUS_SUCCESS)
        {
            LAC_LOG_ERROR("Failed to release Admin Rx/Tx handle\n");
            ret_status = status;
        }
    }

    /* Cleanup Heartbeat */
    SalCtrl_shutdownHeartbeat(qat_instance);

    if(CPA_TRUE == pStatsCollection->bStatsEnabled &&
       qat_instance->debug &&
       qat_instance->debug->debug_file.name)
    {
        icp_adf_debugRemoveFile(&qat_instance->debug->debug_file);
        LAC_OS_FREE(qat_instance->debug->debug_file.name);
        LAC_OS_FREE(qat_instance->debug);
    }
    qat_instance->generic_service_info.stats = NULL;

    SalCtrl_QatClean_dh895x(qat_instance);

    /* change state to be shutdown */
    qat_instance->generic_service_info.state = SAL_SERVICE_STATE_SHUTDOWN;

    return ret_status;
}
