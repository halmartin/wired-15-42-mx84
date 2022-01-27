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
#include "qat_ctrl.h"
#include "qat_init.h"
#include "qat_admin_common.h"
#include "qat_admin.h"
#include "qat_rings.h"

static char* subsystem_name = "QAT";
/**< Name used by ADF to identify this component. */

static subservice_registation_handle_t sal_qat_reg_handle;
/**< Data structure used by ADF to keep a reference to this component. */


/*
 * This function is used to query ADF for the AE Mask
 */

CpaStatus
SalCtrl_getAeMask(icp_accel_dev_t* device, Cpa32U *aeMask)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char    param_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
    memset(param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    status = icp_adf_cfgGetParamValue(device,
                                      LAC_CFG_SECTION_GENERAL,
                                      ADF_DEV_AE_MASK,
                                      param_value);

    LAC_CHECK_STATUS(status);
    *aeMask = Sal_Strtoul(param_value, NULL, SAL_CFG_BASE_HEX);

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
STATIC sal_service_t* SalCtrl_ServiceQatCreate(icp_accel_dev_t *device, Cpa32U instance)
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

     pQat_ctrl_service->generic_service_info.init = SalCtrl_QatInit_dh895x;
     pQat_ctrl_service->generic_service_info.start = SalCtrl_QatStart_dh895x;
     pQat_ctrl_service->generic_service_info.stop = SalCtrl_QatStop_dh895x;
     pQat_ctrl_service->generic_service_info.shutdown = SalCtrl_QatShutdown_dh895x;

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
    CpaStatus status = CPA_STATUS_SUCCESS;

    LacLogMsg_SetConfig(device);

    /* Create memory for each qat instance */
    status = SalList_add(&qat_instances, &tail_list,
                  SalCtrl_ServiceQatCreate(device, 0));
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
