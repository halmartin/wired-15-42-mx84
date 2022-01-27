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
 *  version: QAT1.5.L.1.11.0-36
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file sal_ctrl_qat.c
 *
 * @ingroup SalCtrl
 *
 * @description
 *    This file contains the core of the qat controller implementation.
 *
 *****************************************************************************/

/* QAT-API includes */
#include "cpa.h"

/* Osal includes */
#include "Osal.h"

/* ADF includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_cfg.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_init.h"
#include "icp_qat_fw_admin.h"

/* SAL includes */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_list.h"
#include "sal_string_parse.h"
#include "lac_sal_types.h"
#include "lac_sal_types_qat_ctrl.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"
#include "sal_statistics.h"

/* QAT includes */
#include "qat_priority.h"
#include "qat_ctrl.h"
#include "qat_init.h"
#include "qat_admin.h"
#include "qat_init_defs.h"
#include "qat_rings.h"

static char* subsystem_name = "QAT";
/**< Name used by ADF to identify this component. */

static subservice_registation_handle_t sal_qat_reg_handle;
/**< Data structure used by ADF to keep a reference to this component. */

/*
 * This function is used to query ADF for the QAT/AE Mask
 */
CpaStatus
SalCtrl_getQatAeMask(icp_accel_dev_t* device, Cpa32U *qatMask)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0;
    char keyStr[ADF_CFG_MAX_KEY_LEN_IN_BYTES] = {0};
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U maxQat = 0;

    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_MAX_ACCEL,
                                      valStr);
    LAC_CHECK_STATUS(status);
    maxQat = Sal_Strtoul(valStr, NULL, ADF_CFG_BASE_DEC);

    for (i = 0; i < maxQat; i++)
    {

        memset(keyStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
        memset(valStr, 0, ADF_CFG_MAX_KEY_LEN_IN_BYTES);

        snprintf(keyStr, ADF_CFG_MAX_KEY_LEN_IN_BYTES,
                ADF_DEV_ACCELAE_MASK_FMT, i);

        status = icp_adf_cfgGetParamValue(device,
                                          LAC_CFG_SECTION_GENERAL,
                                          keyStr,
                                          valStr);
        LAC_CHECK_STATUS(status);
        *(qatMask++) = Sal_Strtoul(valStr, NULL, SAL_CFG_BASE_HEX);
    }

    return status;
}


/*
 * This function is used to query ADF for the number of QAT accelerators
 * available on the device.
 */

CpaStatus
SalCtrl_getNumberOfQat(icp_accel_dev_t* device, Cpa32U *num_qats)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char    param_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
    memset(param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_NUM_ACCEL,
                                      param_value);

    LAC_CHECK_STATUS(status);
    *num_qats = Sal_Strtoul(param_value, NULL, SAL_CFG_BASE_DEC);

    return status;
}

/*
 * This function is used to query ADF for the number of accelerator
 * engines available on the device.
 */
CpaStatus
SalCtrl_getNumberOfAE(icp_accel_dev_t* device, Cpa32U *num_ae)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char    param_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
    memset(param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_NUM_AE,
                                      param_value);

    LAC_CHECK_STATUS(status);

    *num_ae = Sal_Strtoul(param_value, NULL, SAL_CFG_BASE_DEC);

    return status;
}


STATIC CpaBoolean SalCtrl_isCryptoA(Cpa32U ringServiceType)
{
       return (ringServiceType == SAL_RING_TYPE_A_SYM_HI ||
               ringServiceType == SAL_RING_TYPE_A_SYM_LO ||
               ringServiceType == SAL_RING_TYPE_A_ASYM);
}

STATIC CpaBoolean SalCtrl_isCryptoB(Cpa32U ringServiceType)
{
       return (ringServiceType == SAL_RING_TYPE_B_SYM_HI ||
               ringServiceType == SAL_RING_TYPE_B_SYM_LO ||
               ringServiceType == SAL_RING_TYPE_B_ASYM);
}

/*
*******************************************************************************
 * @ingroup SalCtrl
 * @description
 *    This function is called whenever a trans handle for a TX ring
 *    is being created in user space. It updates the ring table
 *    and ring polling mask.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * param[in] device           Represents a device
 * param[in] ringNumber       Ring number for which trans handle is created
 * param[in] oper             Operation type: either ICP_ADF_RING_ENABLE or
 *                            ICP_ADF_RING_DISABLE
 * param[in] info_type        Ring info type passed through create trans
 *                            handle function and back again
 *
 * @retVal                    SUCCESS on successfully updating the ring mask
 *                            FAIL on general fail
 *******************************************************************************
*/
STATIC CpaStatus
SalCtrl_QatRingInfoCb(icp_accel_dev_t *device,
        Cpa32U ringNumber,
        icp_adf_ringInfoOperation_t oper,
        icp_adf_ringInfoService_t info_type)
{
    CpaStatus status=CPA_STATUS_FAIL;
    sal_qat_service_t*  qatInstance = NULL;
    sal_list_t* list_addr = NULL;
    Cpa32U ringNum=0, ringServiceType = 0, i = 0;
    Cpa32U aeIndex =0;

    LAC_LOG_DEBUG2("Ring info callback called for ring %d, operation: %d\n",
                   ringNumber, oper);
    if(NULL == device)
    {
        LAC_LOG_ERROR("accel dev is NULL\n");
        return CPA_STATUS_FAIL;
    }
    list_addr = (sal_list_t *) device->pQatHandle;
    if(NULL == list_addr)
    {
        LAC_LOG_ERROR("No QAT instances started in system\n");
        return CPA_STATUS_FAIL;
    }

    ringServiceType = lac_getServiceType(info_type);

    ringNum = ringNumber % SAL_RINGS_NUM_PER_QAT;

    /* From ring number find out if qat instance 0 or 1 */
    if(ringNum == ringNumber)
    {
        qatInstance = (sal_qat_service_t *) list_addr->pObj;
    }
    else
    {
        if(ringNumber == ringNum + SAL_RINGS_NUM_PER_QAT)
        {
            list_addr = list_addr->next;
            if(NULL == list_addr)
            {
                LAC_LOG_ERROR("Invalid ring number - only one "
                              "QAT instance configured\n");
                return CPA_STATUS_FAIL;
            }
            qatInstance = (sal_qat_service_t *) list_addr->pObj;
        }
        else
        {
            LAC_LOG_ERROR("Ring number out of range\n");
            return CPA_STATUS_FAIL;
        }
    }

    if(qatInstance == NULL)
    {
        LAC_LOG_ERROR("QAT instance is NULL\n");
        return CPA_STATUS_FAIL;
    }

    status = QatCtrl_updateRingTable(device,
                            qatInstance,
                            ringNum,
                            oper,
                            ringServiceType);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Updating Ring Table Failed\n");
        return CPA_STATUS_FAIL;
    }
    /*
     * The ring table for the given service has been updated.
     * We now need to send the updated ring table to the AE
     * that is reponsible for the service.
     *
     * Check if the ring is a crypto A or crypto B ring. If it is,
     * find which AE is dealing with crypto and send it
     * the SET_RING_INFO msg.
     */
    if(SalCtrl_isCryptoA(ringServiceType))
    {
        for(i=0; i< qatInstance->num_aes_per_qat; i++)
        {
            aeIndex = ((qatInstance->generic_service_info.instance *
                         qatInstance->num_aes_per_qat) + i);

            if(qatInstance->serviceMask[i] & SAL_SERVICE_TYPE_CRYPTO_A)
            {
                status = QatCtrl_InitMsgSendSync(device,
                            qatInstance,
                            ICP_QAT_FW_INIT_CMD_SET_RING_INFO,
                            aeIndex);
                if(CPA_STATUS_SUCCESS != status)
                {
                    LAC_LOG_ERROR("QatRingInfoCb - Sending SET_RING_INFO "
                                  " msg Failed\n");
                    return CPA_STATUS_FAIL;
                }
                return status;
            }
        }
    }
    /* Now crypto B */
    if(SalCtrl_isCryptoB(ringServiceType))
    {
        for(i=0; i< qatInstance->num_aes_per_qat; i++)
        {
            aeIndex = ((qatInstance->generic_service_info.instance *
                         qatInstance->num_aes_per_qat) + i);

            if(qatInstance->serviceMask[i] & SAL_SERVICE_TYPE_CRYPTO_B)
            {
                status = QatCtrl_InitMsgSendSync(device,
                            qatInstance,
                            ICP_QAT_FW_INIT_CMD_SET_RING_INFO,
                            aeIndex);
                if(CPA_STATUS_SUCCESS != status)
                {
                    LAC_LOG_ERROR("QatRingInfoCb - Sending SET_RING_INFO "
                                  " msg Failed\n");
                    return CPA_STATUS_FAIL;
                }
                return status;
            }
        }
    }

    /*
     * Check if the ring is a compression ring. If it is,
     * find which AE is dealing with compression and send it
     * the SET_RING_INFO msg.
     */
    if(ringServiceType == SAL_RING_TYPE_DC )
    {
        for(i=0; i< qatInstance->num_aes_per_qat; i++)
        {
            aeIndex = ((qatInstance->generic_service_info.instance *
                         qatInstance->num_aes_per_qat) + i);

            if(qatInstance->serviceMask[i] & SAL_SERVICE_TYPE_COMPRESSION)
            {
                status = QatCtrl_InitMsgSendSync(device,
                            qatInstance,
                            ICP_QAT_FW_INIT_CMD_SET_RING_INFO,
                            aeIndex);
                if(CPA_STATUS_SUCCESS != status)
                {
                    LAC_LOG_ERROR("QatRingInfoCb - Sending SET_RING_INFO "
                                  "msg Failed\n");
                    return CPA_STATUS_FAIL;
                }
                return status;
            }
        }
    }
    return status;
}

/**
*******************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function creates a qat service for a specific instance.
 *
 * @context
 *      This function is called from SalCtrl_QatEventInit.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No (ADF ensures that this function doesn't need to be thread safe)
 *
 * @param[in]  instance  The specific logical instance which will
 *                       run the service
 * @retVal               Pointer to specific qat service instance memory
 *                       or NULL in event of error.
 *
 *****************************************************************************/
STATIC sal_service_t* SalCtrl_ServiceQatCreate(Cpa32U instance)
{
     sal_qat_service_t* pQat_ctrl_service = NULL;

     pQat_ctrl_service = osalMemAlloc(sizeof(sal_qat_service_t));

     if (pQat_ctrl_service == NULL)
     {
         LAC_LOG_ERROR("Failed to allocate qat service memory");
         return NULL;
     }
     /* Zero memory */
     osalMemSet(pQat_ctrl_service, 0, sizeof(sal_qat_service_t));

     pQat_ctrl_service->generic_service_info.type =
                    (sal_service_type_t) SAL_SERVICE_TYPE_QAT;
     pQat_ctrl_service->generic_service_info.state =
                                  SAL_SERVICE_STATE_UNINITIALIZED;
     pQat_ctrl_service->generic_service_info.instance = instance;
     pQat_ctrl_service->generic_service_info.init = SalCtrl_QatInit;
     pQat_ctrl_service->generic_service_info.start = SalCtrl_QatStart;
     pQat_ctrl_service->generic_service_info.stop = SalCtrl_QatStop;
     pQat_ctrl_service->generic_service_info.shutdown = SalCtrl_QatShutdown;
     return &(pQat_ctrl_service->generic_service_info);
}

/****************************************************************************
 * @ingroup SalCtrl
 * @description
 *    Queries ADF for the number of QAT HW instances, allocates memory for
 *    each instance and then invokes the init function on each instance.
 *
 * @context
 *      This function is called from the SalCtrl_QatEventHandler function.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No (ADF ensures that this function doesn't need to be thread safe)
 *
 * @param[in] device    An icp_accel_dev_t* type
 *
 * @retVal              status from init event
 **************************************************************************/

STATIC CpaStatus
SalCtrl_QatEventInit(icp_accel_dev_t* device)
{
    sal_list_t *qat_instances = NULL;
    sal_list_t *tail_list = NULL;
    Cpa32U num_qat, i = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;

    status = SalCtrl_getNumberOfQat(device, &num_qat);
    LAC_CHECK_STATUS(status);

    /* Create memory for each qat instance */
    for(i=0; i<num_qat; i++)
    {
        status = SalList_add(&qat_instances, &tail_list,
                      SalCtrl_ServiceQatCreate(i));
        if(CPA_STATUS_SUCCESS != status)
        {
            break;
        }
    }
    if(CPA_STATUS_SUCCESS != status)
    {
        SalList_free(&qat_instances);
        return status;
    }
    device->pQatHandle = qat_instances;

    /* Call init function for each qat instance */
    SAL_FOR_EACH(qat_instances, sal_service_t, device, init, status);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to initialise all QAT instances");
        /* shutdown all qat instances initialised before error */
        SAL_FOR_EACH_STATE(qat_instances,
                           sal_service_t, device, shutdown,
                           SAL_SERVICE_STATE_INITIALIZED);
    }

    return status;
}
/*****************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function calls the start function for each QAT instance.
 *
 * @context
 *      This function is called from the SalCtrl_QatEventHandler function.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No (ADF ensures that this function doesn't need to be thread safe)
 *
 * @param[in] device    An icp_accel_dev_t* type
 *
 * @retVal              status from start event
 ****************************************************************************/

STATIC CpaStatus
SalCtrl_QatEventStart(icp_accel_dev_t* device)
{
    sal_list_t *qat_instances = device->pQatHandle;
    CpaStatus status = CPA_STATUS_SUCCESS;

    /* call start function for each instance */
    SAL_FOR_EACH(qat_instances, sal_service_t, device, start, status);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to start all qat instances");
        /* stop all qat instances started before error */
        SAL_FOR_EACH_STATE(qat_instances,
                           sal_service_t, device, stop,
                           SAL_SERVICE_STATE_RUNNING);
    }
    return status;
}

/****************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function calls the stop function for each QAT instance.
 *
 * @context
 *      This function is called from the SalCtrl_QatEventHandler function.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No (ADF ensures that this function doesn't need to be thread safe)
 *
 * @param[in] device    An icp_accel_dev_t* type
 *
 * @retVal              status from stop event
 ****************************************************************************/
STATIC CpaStatus
SalCtrl_QatEventStop(icp_accel_dev_t* device)
{
    sal_list_t *qat_instances = device->pQatHandle;
    CpaStatus status = CPA_STATUS_SUCCESS;

    /* call stop function for each instance */
    SAL_FOR_EACH(qat_instances, sal_service_t, device, stop, status);

    return status;
}

/***************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function calls the shutdown function for each QAT instance.
 *      It also frees QAT instance memory allocated at Init.
 *
 * @context
 *      This function is called from the SalCtrl_QatEventHandler function.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No (ADF ensures that this function doesn't need to be thread safe)
 *
 * @param[in] device    An icp_accel_dev_t* type
 *
 * @retVal              status from shutdown event
 ***************************************************************************/

STATIC CpaStatus
SalCtrl_QatEventShutdown(icp_accel_dev_t* device)
{
    sal_list_t *qat_instances = device->pQatHandle;
    CpaStatus status = CPA_STATUS_SUCCESS;

    /* call shutdown function for each instance */
    SAL_FOR_EACH(qat_instances,
                    sal_service_t, device, shutdown, status);

    /* Free memory allocated at init. */
    SalList_free(&qat_instances);
    device->pQatHandle = NULL;
    return status;
}

/***************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function is the events handler registered with ADF
 *      for the QAT service (init and admin) - kernel only
 *
 * @context
 *      This function is called from an ADF context.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No (ADF ensures that this function doesn't need to be thread safe)
 *
 * @param[in]  accel_dev    An icp_accel_dev_t* type
 * @param[in]  event       Event from ADF
 * @param[in]  param       Parameter used for back compatibility
 *
 * @retVal                 status from call to event
 *************************************************************************/
STATIC CpaStatus
SalCtrl_QatEventHandler(icp_accel_dev_t *accel_dev,
        icp_adf_subsystemEvent_t event, void* param)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    switch(event)
    {
        case ICP_ADF_EVENT_INIT:
        {
            status = SalStatistics_InitStatisticsCollection(accel_dev);
            if(CPA_STATUS_SUCCESS == status)
            {
                status = SalCtrl_QatEventInit(accel_dev);
            }
            break;
        }
        case ICP_ADF_EVENT_START:
        {
            status = SalCtrl_QatEventStart(accel_dev);
            LAC_CHECK_STATUS(status);
            status = icp_adf_accesLayerRingInfoCbRegister(accel_dev,
                                                      SalCtrl_QatRingInfoCb);
            break;
        }
        case ICP_ADF_EVENT_STOP:
        {
            status = SalCtrl_QatEventStop(accel_dev);
            icp_adf_accesLayerRingInfoCbUnregister(accel_dev);
            break;
        }
        case ICP_ADF_EVENT_SHUTDOWN:
        {
            status = SalCtrl_QatEventShutdown(accel_dev);
            break;
        }
        default:
        status = CPA_STATUS_SUCCESS;
    }
    return status;
}

/*
 * Register the QAT service with ADF.
 */
CpaStatus
SalCtrl_AdfQatRegister(void)
{
    sal_qat_reg_handle.subserviceEventHandler = SalCtrl_QatEventHandler;
    sal_qat_reg_handle.subsystem_name = subsystem_name;

    return icp_adf_subsystemRegister( &sal_qat_reg_handle );
}

/*
 * Unregister the QAT service with ADF.
 */
CpaStatus
SalCtrl_AdfQatUnregister(void)
{
    return icp_adf_subsystemUnregister( &sal_qat_reg_handle );
}
