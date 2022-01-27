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
 * @file sal_ctrl_services.c
 *
 * @ingroup SalCtrl
 *
 * @description
 *    This file contains the core of the service controller implementation.
 *
 *****************************************************************************/

/* QAT-API includes */
#include "cpa.h"
#include "cpa_cy_key.h"
#include "cpa_cy_ln.h"
#include "cpa_cy_dh.h"
#include "cpa_cy_dsa.h"
#include "cpa_cy_rsa.h"
#include "cpa_cy_ec.h"
#include "cpa_cy_prime.h"
#include "cpa_cy_sym.h"
#include "cpa_cy_common.h"
#include "cpa_dc.h"
/* Osal includes */
#include "Osal.h"

/* ADF includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_init.h"
#include "icp_qat_fw_la.h"

/* SAL includes */
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_list.h"
#include "lac_hooks.h"
#include "sal_string_parse.h"
#include "lac_sym.h"
#include "lac_sym_key.h"
#include "lac_common.h"
#include "lac_sym_qat_hash_defs_lookup.h"
#include "lac_sym_qat.h"
#include "lac_sal_types.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"

#define SAL_USER_SPACE_START_TIMEOUT_MS 120000
#define MAX_SUBSYSTEM_RETRY             64

static char* subsystem_name = "SAL";
/**< Name used by ADF to identify this component. */
static char* cy_dir_name = "cy";
static char* dc_dir_name = "dc";
/**< Stats dir names. */
static char* ver_file_name = "version";

static subservice_registation_handle_t sal_service_reg_handle;
/**< Data structure used by ADF to keep a reference to this component. */

/*
 * @ingroup SalCtrl
 * @description
 *      This function is used to parse the results from ADF
 *    in response to ServiceEnabled query.The results are semi-colon separeted.
 *    Internally, the bitmask represented by the enabled_service
 *    is used to track which features are enabled.
 *
 * @context
 *      This functions is called from the SalCtrl_ServiceEventInit function.
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
 * @param[in] device              pointer to icp_accel_dev_t structure
 * @param[in] pEnabledServices    pointer to memory where enabled services will
 *                               be written.
 * @retval Status
 */
CpaStatus SalCtrl_GetEnabledServices(icp_accel_dev_t* device,
    Cpa32U* pEnabledServices)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char param_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES] =
    { 0 };
    char* token = NULL;
    char* running = NULL;
    Cpa32U capabilitiesMask = 0;

    *pEnabledServices = 0;

    memset(param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_amgr_getAccelDevCapabilities((CpaInstanceHandle)device,
                                                &capabilitiesMask);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to get device capabilities\n");
        return status;
    }

    status = icp_adf_cfgGetParamValue(device, LAC_CFG_SECTION_GENERAL,
        "ServicesEnabled", param_value);

    if (CPA_STATUS_SUCCESS == status)
    {
        running = param_value;

        token = strsep(&running, ";");

        while (NULL != token)
        {
            do {
                if(strncmp(token, "cy0", strlen("cy0")) == 0)
                {
                    *pEnabledServices |= SAL_SERVICE_TYPE_CRYPTO_A;
                    break;
                }
                if(strncmp(token, "cy1", strlen("cy1")) == 0)
                {
                    if( (capabilitiesMask &
                            ICP_ACCEL_CAPABILITIES_CRYPTO_1) )
                    {
                        *pEnabledServices |= SAL_SERVICE_TYPE_CRYPTO_B;
                        break;
                    }
                    else
                    {
                        LAC_LOG_ERROR("Unsupported service in config:cy1\n");
                        return CPA_STATUS_UNSUPPORTED;
                    }
                }
                if(strncmp(token, "cy", strlen("cy")) == 0)
                {
                    *pEnabledServices |= SAL_SERVICE_TYPE_CRYPTO_A;
                    break;
                }
                if(strncmp(token, "dc", strlen("dc")) == 0)
                {
                    if( (capabilitiesMask &
                            ICP_ACCEL_CAPABILITIES_COMPRESSION) ||
                        (capabilitiesMask &
                                ICP_ACCEL_CAPABILITIES_LZS_COMPRESSION)    )
                    {
                        *pEnabledServices |= SAL_SERVICE_TYPE_COMPRESSION;
                        break;
                    }
                    else
                    {
                        LAC_LOG_ERROR("Unsupported service in config:dc\n");
                        return CPA_STATUS_UNSUPPORTED;
                    }
                }

                LAC_LOG_ERROR("Error parsing enabled services from ADF\n");
                return CPA_STATUS_FAIL;

            }while(0);
            token = strsep(&running, ";");
        }
    }
    else
    {
        LAC_LOG_ERROR("Failed to get enabled services from ADF");
    }
    return status;
}

/*
 * @ingroup SalCtrl
 * @description
 *    This function is used to check whether a service is enabled
 *
 * @context
 *      This functions is called from the SalCtrl_ServiceEventInit function.
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
 * param[in] enabled_services    It is the bitmask for the enabled services
 * param[in] service    It is the service we want to check for
 */
CpaBoolean SalCtrl_IsServiceEnabled(Cpa32U enabled_services,
    sal_service_type_t service)
{
    return (CpaBoolean)((enabled_services & (Cpa32U)(service)) != 0);
}

/*
 * @ingroup SalCtrl
 * @description
 *    This function is used to check whether enabled services has associated
 *    hardware capability support
 *
 * @context
 *      This functions is called from the SalCtrl_ServiceEventInit function.
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
 * param[in] device              A pointer to an icp_accel_dev_t
 * param[in] enabled_services    It is the bitmask for the enabled services
 */

CpaStatus SalCtrl_GetSupportedServices(icp_accel_dev_t* device,
    Cpa32U enabled_services)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U capabilitiesMask = 0;

    status = icp_amgr_getAccelDevCapabilities(device,
                                                &capabilitiesMask);
    if (CPA_STATUS_SUCCESS == status)
    {
        if (SalCtrl_IsServiceEnabled(enabled_services,
                                        SAL_SERVICE_TYPE_CRYPTO))
        {
            if (!(capabilitiesMask & ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC) ||
                !(capabilitiesMask & ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC))
            {
                LAC_LOG_ERROR("Device does not support Crypto service");
                status = CPA_STATUS_FAIL;
            }
        }

        if (SalCtrl_IsServiceEnabled(enabled_services,
                                        SAL_SERVICE_TYPE_COMPRESSION))
        {
            if (!(capabilitiesMask & ICP_ACCEL_CAPABILITIES_COMPRESSION))
            {
                LAC_LOG_ERROR("Device does not support Compression service");
                status = CPA_STATUS_FAIL;
            }
        }
    }
    return status;
}

/*
 * @ingroup SalCtrl
 * @description
 *    This function is used to retrieve how many instances are
 *    to be configured for process specific service.
 *
 * @context
 *      This functions is called from the SalCtrl_ServiceEventInit function.
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
 * @param[in] device    A pointer to an icp_accel_dev_t
 * @param[in] key       Represents the parameter's name we want to query
 * @param[out] pCount   Pointer to memory where num instances will be stored
 * @retval status       returned status from ADF or _FAIL if number of instances
 *                      is out of range for the device.
 */
STATIC CpaStatus SalCtrl_GetInstanceCount(icp_accel_dev_t* device, char* key,
    Cpa32U* pCount)
{
    CpaStatus status = CPA_STATUS_FAIL;
    char param_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES];

    memset(param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(device, icpGetProcessName(), key,
        param_value);
    if (CPA_STATUS_SUCCESS == status)
    {
        *pCount = (Cpa32U) (Sal_Strtoul(param_value, NULL, SAL_CFG_BASE_DEC));
        if (*pCount > SAL_MAX_NUM_INSTANCES_PER_DEV)
        {
            LAC_LOG_ERROR("num instances out of range");
            status = CPA_STATUS_FAIL;
        }
    }
    return status;
}

/*
 * @ingroup SalCtrl
 * @description
 *    This function is used to print hardware and software versions in proc
 *    filesystem entry via ADF Debug interface
 *
 * @context
 *      This functions is called from proc filesystem interface
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
 * @param[in]  private_data    A pointer to a private data passed to the
 *                             function while adding a debug file.
 * @param[out] data            Pointer to a buffer where version information
 *                             needs to be printed to.
 * @param[in]  size            Size of a buffer pointed by data.
 * @param[in]  offset          Offset in a debug file
 *
 * @retval 0    This function always returns 0
 */
STATIC int SalCtrl_VersionDebug(void* private_data,
                           char* data, int size, int offset)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U len = 0;
    icp_accel_dev_t *device = (icp_accel_dev_t *)private_data;
    char param_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = { 0 };

    len += snprintf(data+len, size-len,
                    SEPARATOR
                    BORDER " Hardware Software and API versions for device %d   "
                    BORDER "\n"
                    SEPARATOR, device->accelId);

    memset(param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(device, LAC_CFG_SECTION_GENERAL,
                    ICP_CFG_HW_REV_ID_KEY, param_value);
    LAC_CHECK_STATUS(status);

    len += snprintf(data+len, size-len,
                    " Hardware Version:             %s %s \n",
                    param_value, get_sku_info(device->sku));

    memset(param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(device, LAC_CFG_SECTION_GENERAL,
                    ICP_CFG_UOF_VER_KEY, param_value);
    LAC_CHECK_STATUS(status);

    len += snprintf(data+len, size-len,
                    " Firmware Version:             %s \n",
                    param_value);

    memset(param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    status = icp_adf_cfgGetParamValue(device, LAC_CFG_SECTION_GENERAL,
                    ICP_CFG_MMP_VER_KEY, param_value);
    LAC_CHECK_STATUS(status);

    len += snprintf(data+len, size-len,
                    " MMP Version:                  %s \n",
                    param_value);

    len += snprintf(data+len, size-len,
                    " Driver Version:               %d.%d.%d \n",
                    SAL_INFO2_DRIVER_SW_VERSION_MAJ_NUMBER,
                    SAL_INFO2_DRIVER_SW_VERSION_MIN_NUMBER,
                    SAL_INFO2_DRIVER_SW_VERSION_PATCH_NUMBER);

    len += snprintf(data+len, size-len,
                    " QuickAssist API CY Version:   %d.%d \n",
                    CPA_CY_API_VERSION_NUM_MAJOR,
                    CPA_CY_API_VERSION_NUM_MINOR);

    len += snprintf(data+len, size-len,
                    " QuickAssist API DC Version:   %d.%d \n",
                    CPA_DC_API_VERSION_NUM_MAJOR,
                    CPA_DC_API_VERSION_NUM_MINOR);

    len += snprintf(data+len, size-len, SEPARATOR);
    return 0;
}


/*************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function is used to initialize the service instances.
 *    It first checks (via ADF query) which services are enabled in the
 *    system and the number of each services.
 *    It then allocates memory for service instances and invokes the
 *    init function on them.
 *
 * @context
 *    This function is called from the SalCtrl_ServiceEventHandler function.
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
 * @param[in] device   An icp_accel_dev_t* type
 *
 *************************************************************************/
STATIC CpaStatus SalCtrl_ServiceEventInit(icp_accel_dev_t* device)
{
    sal_t* service_container = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U enabled_services, i = 0;
    Cpa32U instance_count = 0;
    sal_list_t *tail_list = NULL;
    sal_service_t *pInst = NULL;

    if (!icp_adf_is_dev_in_reset(device))
    {
        service_container = osalMemAlloc(sizeof(sal_t));
        if (NULL == service_container)
        {
            LAC_LOG_ERROR("Failed to allocate service memory");
            return CPA_STATUS_RESOURCE;
        }
        device->pSalHandle = service_container;
        service_container->crypto_services = NULL;
        service_container->compression_services = NULL;
    }
    else
    {
        service_container = device->pSalHandle;
    }
    service_container->cy_dir = NULL;
    service_container->dc_dir = NULL;
    service_container->ver_file = NULL;

    status = LAC_OS_MALLOC(&service_container->ver_file,
                              sizeof(debug_file_info_t));
    if(CPA_STATUS_SUCCESS != status)
    {
        osalMemFree(service_container);
        return status;
    }

    osalMemSet(service_container->ver_file, 0, sizeof(debug_file_info_t));
    service_container->ver_file->name = ver_file_name;
    service_container->ver_file->seq_read = SalCtrl_VersionDebug;
    service_container->ver_file->private_data = device;
    service_container->ver_file->parent = NULL;

    status = icp_adf_debugAddFile(device,
                     service_container->ver_file);
    if(CPA_STATUS_SUCCESS != status)
    {
        LAC_OS_FREE(service_container->ver_file);
        osalMemFree(service_container);
        return status;
    }

    status = SalCtrl_GetEnabledServices(device, &enabled_services);
    if (CPA_STATUS_SUCCESS != status)
    {
        icp_adf_debugRemoveFile(service_container->ver_file);
        LAC_OS_FREE(service_container->ver_file);
        osalMemFree(service_container);
        device->pSalHandle = NULL;
        return status;
    }

    status = SalCtrl_GetSupportedServices(device, enabled_services);
    if (CPA_STATUS_SUCCESS != status)
    {
        icp_adf_debugRemoveFile(service_container->ver_file);
        LAC_OS_FREE(service_container->ver_file);
        osalMemFree(service_container);
        device->pSalHandle = NULL;
        return status;
    }

    /* Allocate memory for each service instance described in config file */
    if (SalCtrl_IsServiceEnabled(enabled_services, SAL_SERVICE_TYPE_CRYPTO))
    {
        status = SalCtrl_GetInstanceCount(device, "NumberCyInstances",
            &instance_count);
        if (CPA_STATUS_SUCCESS != status)
        {
            instance_count = 0;
        }

        status = LAC_OS_MALLOC(&service_container->cy_dir,
                                   sizeof(debug_dir_info_t));
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to allocate memory for cy proc dir");
            icp_adf_debugRemoveFile(service_container->ver_file);
            LAC_OS_FREE(service_container->ver_file);
            osalMemFree(service_container);
            device->pSalHandle = NULL;
            return status;
        }
        service_container->cy_dir->name = cy_dir_name;
        service_container->cy_dir->parent = NULL;
        status = icp_adf_debugAddDir(device, service_container->cy_dir);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to add cy proc dir");
            icp_adf_debugRemoveFile(service_container->ver_file);
            LAC_OS_FREE(service_container->ver_file);
            LAC_OS_FREE(service_container->cy_dir);
            service_container->cy_dir = NULL;
            osalMemFree(service_container);
            device->pSalHandle = NULL;
            return status;
        }

        if (!icp_adf_is_dev_in_reset(device))
        {
            for (i = 0; i < instance_count ; i++)
            {
                status = SalCtrl_ServiceCreate(SAL_SERVICE_TYPE_CRYPTO, i,
                                                                  &pInst);
                if (CPA_STATUS_SUCCESS == status)
                {
                    pInst->debug_parent_dir = service_container->cy_dir;
                    pInst->capabilitiesMask = device->accelCapabilitiesMask;
                    status = SalList_add(&service_container->crypto_services,
                        &tail_list, pInst);
                    if (CPA_STATUS_SUCCESS != status)
                    {
                        osalMemFree(pInst);
                    }
                }
                if (CPA_STATUS_SUCCESS != status)
                {
                    break;
                }
            }
        }
        else
        {
            sal_list_t *curr_element = service_container->crypto_services;
            sal_service_t *service = NULL;
            while(NULL != curr_element)
            {
                service = (sal_service_t*)SalList_getObject(curr_element);
                service->debug_parent_dir = service_container->cy_dir;
                curr_element = SalList_next(curr_element);
            }
        }
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to allocate all crypto instances");
            icp_adf_debugRemoveFile(service_container->ver_file);
            LAC_OS_FREE(service_container->ver_file);
            SalList_free(&service_container->crypto_services);
            icp_adf_debugRemoveDir(service_container->cy_dir);
            LAC_OS_FREE(service_container->cy_dir);
            osalMemFree(service_container);
            device->pSalHandle = NULL;
            return status;
        }
    }

    if (SalCtrl_IsServiceEnabled(enabled_services,
        SAL_SERVICE_TYPE_COMPRESSION))
    {
        status = SalCtrl_GetInstanceCount(device, "NumberDcInstances",
            &instance_count);
        if (CPA_STATUS_SUCCESS != status)
        {
            instance_count = 0;
        }
        status = LAC_OS_MALLOC(&service_container->dc_dir,
                                   sizeof(debug_dir_info_t));
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to allocate memory for dc proc dir");
            if(service_container->cy_dir)
            {
                icp_adf_debugRemoveDir(service_container->cy_dir);
                LAC_OS_FREE(service_container->cy_dir);
                service_container->cy_dir = NULL;
            }
            icp_adf_debugRemoveFile(service_container->ver_file);
            LAC_OS_FREE(service_container->ver_file);
            osalMemFree(service_container);
            device->pSalHandle = NULL;
            return status;
        }
        service_container->dc_dir->name = dc_dir_name;
        service_container->dc_dir->parent = NULL;
        status = icp_adf_debugAddDir(device, service_container->dc_dir);
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to add dc proc dir");
            if(service_container->cy_dir)
            {
                icp_adf_debugRemoveDir(service_container->cy_dir);
                LAC_OS_FREE(service_container->cy_dir);
                service_container->cy_dir = NULL;
            }
            LAC_OS_FREE(service_container->dc_dir);
            icp_adf_debugRemoveFile(service_container->ver_file);
            LAC_OS_FREE(service_container->ver_file);
            osalMemFree(service_container);
            device->pSalHandle = NULL;
            return status;
        }

        if (!icp_adf_is_dev_in_reset(device))
        {
            for (i = 0; i < instance_count ; i++)
            {
                status = SalCtrl_ServiceCreate(SAL_SERVICE_TYPE_COMPRESSION, i,
                    &pInst);
                if (CPA_STATUS_SUCCESS == status)
                {
                    pInst->debug_parent_dir = service_container->dc_dir;
                    pInst->capabilitiesMask = device->accelCapabilitiesMask;
                    status = SalList_add(&service_container->compression_services,
                        &tail_list, pInst);
                    if (CPA_STATUS_SUCCESS != status)
                    {
                        osalMemFree(pInst);
                    }
                }
                if (CPA_STATUS_SUCCESS != status)
                {
                    break;
                }
            }
        }
        else
        {
            sal_list_t *curr_element = service_container->compression_services;
            sal_service_t *service = NULL;
            while(NULL != curr_element)
            {
                service = (sal_service_t*)SalList_getObject(curr_element);
                service->debug_parent_dir = service_container->dc_dir;
                curr_element = SalList_next(curr_element);
            }
        }
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("Failed to allocate all compression instances");
            if(service_container->cy_dir)
            {
                icp_adf_debugRemoveDir(service_container->cy_dir);
                LAC_OS_FREE(service_container->cy_dir);
                service_container->cy_dir = NULL;
            }
            icp_adf_debugRemoveDir(service_container->dc_dir);
            LAC_OS_FREE(service_container->dc_dir);
            service_container->dc_dir = NULL;
            SalList_free(&service_container->compression_services);
            SalList_free(&service_container->crypto_services);
            icp_adf_debugRemoveFile(service_container->ver_file);
            LAC_OS_FREE(service_container->ver_file);
            osalMemFree(service_container);
            device->pSalHandle = NULL;
            return status;
        }
    }

    /* Call init function for each service instance */
    SAL_FOR_EACH(service_container->crypto_services,
        sal_service_t, device, init, status);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to initialise all crypto instances");
        /* shutdown all crypto instances initialised before error */
        SAL_FOR_EACH_STATE(service_container->crypto_services,
            sal_service_t, device, shutdown,
            SAL_SERVICE_STATE_INITIALIZED);
        if(service_container->cy_dir)
        {
            icp_adf_debugRemoveDir(service_container->cy_dir);
            LAC_OS_FREE(service_container->cy_dir);
            service_container->cy_dir = NULL;
        }
        if(service_container->dc_dir)
        {
            icp_adf_debugRemoveDir(service_container->dc_dir);
            LAC_OS_FREE(service_container->dc_dir);
            service_container->dc_dir = NULL;
        }
        SalList_free(&service_container->compression_services);
        SalList_free(&service_container->crypto_services);
        icp_adf_debugRemoveFile(service_container->ver_file);
        LAC_OS_FREE(service_container->ver_file);
        osalMemFree(service_container);
        device->pSalHandle = NULL;
        return status;
    }

    SAL_FOR_EACH(service_container->compression_services,
        sal_service_t, device, init, status);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to initialise all compression instances");
        /* shutdown all compression instances initialised before error */
        SAL_FOR_EACH_STATE(service_container->compression_services,
            sal_service_t, device, shutdown,
            SAL_SERVICE_STATE_INITIALIZED);
        /* shutdown all crypto instances */
        SAL_FOR_EACH_STATE(service_container->crypto_services,
            sal_service_t, device, shutdown,
            SAL_SERVICE_STATE_INITIALIZED);
        if(service_container->cy_dir)
        {
            icp_adf_debugRemoveDir(service_container->cy_dir);
            LAC_OS_FREE(service_container->cy_dir);
            service_container->cy_dir = NULL;
        }
        if(service_container->dc_dir)
        {
            icp_adf_debugRemoveDir(service_container->dc_dir);
            LAC_OS_FREE(service_container->dc_dir);
            service_container->dc_dir = NULL;
        }
        SalList_free(&service_container->compression_services);
        SalList_free(&service_container->crypto_services);
        icp_adf_debugRemoveFile(service_container->ver_file);
        LAC_OS_FREE(service_container->ver_file);
        osalMemFree(service_container);
        device->pSalHandle = NULL;
        return status;
    }

    return status;
}

/**************************************************************************
 * @ingroup SalCtrl
 * @description
 *    This function calls the start function on all the service instances.
 *
 * @context
 *      This function is called from the SalCtrl_ServiceEventHandler function.
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
 * @param[in] device   An icp_accel_dev_t* type
 *
 **************************************************************************/
STATIC CpaStatus SalCtrl_ServiceEventStart(icp_accel_dev_t* device)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_t* service_container = (sal_t*) device->pSalHandle;

    if (service_container == NULL)
    {
        LAC_LOG_ERROR("Private data is NULL");
        return CPA_STATUS_FATAL;
    }

    /* Call Start function for each service instance */

    SAL_FOR_EACH(service_container->crypto_services,
        sal_service_t, device, start, status);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to start all crypto instances");
        /* stop all crypto instances started before error */
        SAL_FOR_EACH_STATE(service_container->crypto_services,
            sal_service_t, device, stop,
            SAL_SERVICE_STATE_RUNNING);
        return status;
    }
    SAL_FOR_EACH(service_container->compression_services,
        sal_service_t, device, start, status);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to start all compression instances");
        /* stop all compression instances started before error */
        SAL_FOR_EACH_STATE(service_container->compression_services,
            sal_service_t, device, stop,
            SAL_SERVICE_STATE_RUNNING);
        /* stop all crypto instances */
        SAL_FOR_EACH_STATE(service_container->crypto_services,
            sal_service_t, device, stop,
            SAL_SERVICE_STATE_RUNNING);
        return status;
    }

    /* Calling restart functions */
    if(icp_adf_is_dev_in_reset(device))
    {
        sal_list_t *curr_element = service_container->crypto_services;
        sal_service_t *service = NULL;
        while(NULL != curr_element)
        {
            service = (sal_service_t*)SalList_getObject(curr_element);
            if(service->notification_cb)
            {
                service->notification_cb(service, service->cb_tag,
                                         CPA_INSTANCE_EVENT_RESTARTED);
            }
            curr_element = SalList_next(curr_element);
        }
        curr_element = service_container->compression_services;
        while(NULL != curr_element)
        {
            service = (sal_service_t*)SalList_getObject(curr_element);
            if(service->notification_cb)
            {
                service->notification_cb(service, service->cb_tag,
                                         CPA_INSTANCE_EVENT_RESTARTED);
            }
            curr_element = SalList_next(curr_element);
        }
    }
    return status;
}

/****************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function calls the stop function on all the service instances.
 *
 * @context
 *      This function is called from the SalCtrl_ServiceEventHandler function.
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
 * @param[in] device    An icp_accel_dev_t* type
 *
 *************************************************************************/
STATIC CpaStatus SalCtrl_ServiceEventStop(icp_accel_dev_t* device)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_t* service_container = (sal_t*) device->pSalHandle;

    if (service_container == NULL)
    {
        LAC_LOG_ERROR("Private data is NULL");
        return CPA_STATUS_FATAL;
    }

    /* Calling restarint functions */
    if(icp_adf_is_dev_in_reset(device))
    {
        sal_list_t *curr_element = service_container->crypto_services;
        sal_service_t *service = NULL;
        while(NULL != curr_element)
        {
            service = (sal_service_t*)SalList_getObject(curr_element);
            if(service->notification_cb)
            {
                service->notification_cb(service, service->cb_tag,
                                       CPA_INSTANCE_EVENT_RESTARTING);
            }
            curr_element = SalList_next(curr_element);
        }
        curr_element = service_container->compression_services;
        while(NULL != curr_element)
        {
            service = (sal_service_t*)SalList_getObject(curr_element);
            if(service->notification_cb)
            {
                service->notification_cb(service, service->cb_tag,
                                       CPA_INSTANCE_EVENT_RESTARTING);
            }
            curr_element = SalList_next(curr_element);
        }
    }

    /* Call Stop function for each service instance */
    SAL_FOR_EACH(service_container->crypto_services,
        sal_service_t, device, stop, status);
    SAL_FOR_EACH(service_container->compression_services,
        sal_service_t, device, stop, status);
    return status;
}

/**************************************************************************
 * @ingroup SalCtrl
 * @description
 *     This function calls the shutdown function on all the service instances.
 *     It also frees all service instance memory allocated at Init.
 *
 * @context
 *      This function is called from the SalCtrl_ServiceEventHandler function.
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
 * @param[in] device    An icp_accel_dev_t* type
 *
 ****************************************************************************/
STATIC CpaStatus SalCtrl_ServiceEventShutdown(icp_accel_dev_t* device)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    sal_t* service_container = (sal_t*) device->pSalHandle;
    sal_list_t * dyn_service = NULL;
    sal_service_t *inst = NULL;

    if (service_container == NULL)
    {
        LAC_LOG_ERROR("Private data is NULL");
        return CPA_STATUS_FATAL;
    }

    /* Call Shutdown function for each service instance */
    SAL_FOR_EACH(service_container->crypto_services,
        sal_service_t, device, shutdown, status);
    SAL_FOR_EACH(service_container->compression_services,
        sal_service_t, device, shutdown, status);

    if(service_container->cy_dir)
    {
        icp_adf_debugRemoveDir(service_container->cy_dir);
        LAC_OS_FREE(service_container->cy_dir);
        service_container->cy_dir = NULL;
    }
    if(service_container->dc_dir)
    {
        icp_adf_debugRemoveDir(service_container->dc_dir);
        LAC_OS_FREE(service_container->dc_dir);
        service_container->dc_dir = NULL;
    }
    if(service_container->ver_file)
    {
        icp_adf_debugRemoveFile(service_container->ver_file);
        LAC_OS_FREE(service_container->ver_file);
        service_container->ver_file = NULL;
    }
    if(!icp_adf_is_dev_in_reset(device))
    {
        dyn_service = service_container->crypto_services;
        while(dyn_service)
        {
            inst = (sal_service_t *)SalList_getObject(dyn_service);
            if(CPA_TRUE == inst->is_dyn)
            {
                icp_adf_putDynInstance(device,
                                  ADF_SERVICE_CRYPTO, inst->instance);
            }
            dyn_service = SalList_next(dyn_service);
        }
        dyn_service = service_container->compression_services;
        while(dyn_service)
        {
            inst = (sal_service_t *)SalList_getObject(dyn_service);
            if(CPA_TRUE == inst->is_dyn)
            {
                icp_adf_putDynInstance(device,
                                  ADF_SERVICE_COMPRESS, inst->instance);
            }
            dyn_service = SalList_next(dyn_service);
        }
        /* Free Sal services controller memory */
        SalList_free(&service_container->crypto_services);
        SalList_free(&service_container->compression_services);

        /* Free container also */
        osalMemFree(service_container);
        device->pSalHandle = NULL;
    }
    else
    {
        sal_list_t *curr_element = service_container->crypto_services;
        sal_service_t *service = NULL;
        while(NULL != curr_element)
        {
            service = (sal_service_t*)SalList_getObject(curr_element);
            service->state = SAL_SERVICE_STATE_RESTARTING;
            curr_element = SalList_next(curr_element);
        }
        curr_element = service_container->compression_services;
        while(NULL != curr_element)
        {
            service = (sal_service_t*)SalList_getObject(curr_element);
            service->state = SAL_SERVICE_STATE_RESTARTING;
            curr_element = SalList_next(curr_element);
        }
    }
    return status;
}

/*************************************************************************
 * @ingroup SalCtrl
 * @description
 *      This function is the events handler registered with ADF
 *      for the QA API services (cy, dc) - kernel and user
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
 *      Yes
 *
 * @param[in]  device      An icp_accel_dev_t* type
 * @param[in]  event       Event from ADF
 * @param[in]  param       Parameter used for back compatibility
 *
 ***********************************************************************/
STATIC CpaStatus SalCtrl_ServiceEventHandler(icp_accel_dev_t *device,
    icp_adf_subsystemEvent_t event, void* param)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaStatus stats_status = CPA_STATUS_SUCCESS;

    switch(event)
    {
        case ICP_ADF_EVENT_INIT:
        {
            /* In case there is no QAT SAL needs to call InitStats */
            if(NULL == device->pQatStats)
            {
                status = SalStatistics_InitStatisticsCollection(device);
            }
            if(CPA_STATUS_SUCCESS != status)
            {
                return status;
            }
            status = SalCtrl_ServiceEventInit(device);
            break;
        }
        case ICP_ADF_EVENT_START:
        {
            status = SalCtrl_ServiceEventStart(device);
            break;
        }
        case ICP_ADF_EVENT_STOP:
        {
            status = SalCtrl_ServiceEventStop(device);
            break;
        }
        case ICP_ADF_EVENT_SHUTDOWN:
        {
            status = SalCtrl_ServiceEventShutdown(device);
            stats_status = SalStatistics_CleanStatisticsCollection(device);
            if(CPA_STATUS_SUCCESS != status ||
                CPA_STATUS_SUCCESS != stats_status)
            {
                return CPA_STATUS_FAIL;
            }
            break;
        }
        default:
        status = CPA_STATUS_SUCCESS;
    }
    return status;
}

CpaStatus SalCtrl_AdfServicesRegister(void)
{
    /* Fill out the global sal_service_reg_handle structure */
    sal_service_reg_handle.subserviceEventHandler = SalCtrl_ServiceEventHandler;
    /* Set subsystem name to globally defined name */
    sal_service_reg_handle.subsystem_name = subsystem_name;

    return icp_adf_subsystemRegister(&sal_service_reg_handle);
}

CpaStatus SalCtrl_AdfServicesUnregister(void)
{
    return icp_adf_subsystemUnregister(&sal_service_reg_handle);
}

CpaStatus SalCtrl_AdfServicesStartedCheck(void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U retry_num = 0;
    CpaBoolean state = CPA_FALSE;

    do
    {
        state = icp_adf_isSubsystemStarted(&sal_service_reg_handle);
        retry_num ++;
    } while ((CPA_FALSE == state)&&(retry_num < MAX_SUBSYSTEM_RETRY));

    if (CPA_FALSE == state)
    {
        LAC_LOG_ERROR("Sal Ctrl failed to start in given time\n");
        status = CPA_STATUS_FAIL;
    }

    return status;
}
