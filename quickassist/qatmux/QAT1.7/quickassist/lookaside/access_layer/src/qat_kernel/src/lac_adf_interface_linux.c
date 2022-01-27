/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *   Copyright(c) 2007-2019 Intel Corporation. All rights reserved.
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
 *  version: QAT1.7.L.4.6.0-00025
 *
 ***************************************************************************/
#include "adf_cfg.h"
#include "cpa.h"
#include "icp_accel_devices.h"
#include "adf_common_drv.h"
#include "icp_platform_linux.h"
#include "icp_adf_init.h"
#include "lac_sal_ctrl.h"
#include "lac_log.h"

STATIC subservice_registation_handle_t *salService = NULL;
STATIC struct service_hndl adfService = {0};
STATIC icp_accel_dev_t *adfDevices = NULL;
STATIC icp_accel_dev_t *adfDevicesHead = NULL;
STATIC ICP_MUTEX adfDevicesLock;

/*
 * Need to keep track of what device is curently in reset state
 */
STATIC char accel_dev_reset_stat[ADF_MAX_DEVICES] = {0};

/*
 * Need to keep track of what device is curently in error state
 */
STATIC char accel_dev_error_stat[ADF_MAX_DEVICES] = {0};

/*
 * Need to preserve sal handle during restart
 */
STATIC void *accel_dev_sal_hdl_ptr[ADF_MAX_DEVICES] = {0};

static icp_accel_dev_t *create_adf_dev_structure(
    struct adf_accel_dev *accel_dev)
{
    icp_accel_dev_t *adf = NULL;

    struct adf_hw_device_data *hw_data = accel_dev->hw_device;

    adf = kzalloc(sizeof(*adf), GFP_KERNEL);
    if (NULL == adf)
    {
        return NULL;
    }

    adf->accelId = accel_dev->accel_id;
    adf->pAccelName = (char *)hw_data->dev_class->name;
    adf->deviceType = (device_type_t)hw_data->dev_class->type;
    strlcpy(adf->deviceName, hw_data->dev_class->name, sizeof(adf->deviceName));
    adf->accelCapabilitiesMask = hw_data->accel_capabilities_mask;
    adf->sku = hw_data->get_sku(hw_data);
    adf->accel_dev = accel_dev;
    adf->revisionId = accel_dev->accel_pci_dev.revid;
    accel_dev->lac_dev = adf;

    return adf;
}

/*
 * adf_event_handler
 * Handle device init/uninit/start/stop event
 */
STATIC CpaStatus adf_event_handler(struct adf_accel_dev *accel_dev,
                                   enum adf_event event)
{
    CpaStatus status = CPA_STATUS_FAIL;
    icp_accel_dev_t *adf = NULL;

    if (!adf_cfg_sec_find(accel_dev, ADF_KERNEL_SAL_SEC))
    {
        return CPA_STATUS_SUCCESS;
    }

    if (event == ADF_EVENT_INIT)
    {
        if (!try_module_get(qat_api_module))
            return CPA_STATUS_FAIL;

        adf = create_adf_dev_structure(accel_dev);
        if (NULL == adf)
        {
            return CPA_STATUS_FAIL;
        }
        if (accel_dev_sal_hdl_ptr[accel_dev->accel_id])
        {
            adf->pSalHandle = accel_dev_sal_hdl_ptr[accel_dev->accel_id];
            accel_dev_sal_hdl_ptr[accel_dev->accel_id] = NULL;
        }

        ICP_MUTEX_LOCK(&adfDevicesLock);
        ICP_ADD_ELEMENT_TO_END_OF_LIST(adf, adfDevices, adfDevicesHead);
        ICP_MUTEX_UNLOCK(&adfDevicesLock);
    }
    else
    {
        adf = accel_dev->lac_dev;
    }

    if (event == ADF_EVENT_START)
    {
        adf->dcExtendedFeatures =
            accel_dev->hw_device->extended_dc_capabilities;
    }

    if (event == ADF_EVENT_RESTARTING)
    {
        accel_dev_reset_stat[accel_dev->accel_id] = 1;
        accel_dev_sal_hdl_ptr[accel_dev->accel_id] = adf->pSalHandle;
    }

    if (event == ADF_EVENT_RESTARTED)
    {
        accel_dev_reset_stat[accel_dev->accel_id] = 0;
    }

    status = salService->subserviceEventHandler(adf, event, NULL);

    if ((status == CPA_STATUS_SUCCESS && event == ADF_EVENT_SHUTDOWN) ||
        (status != CPA_STATUS_SUCCESS && event == ADF_EVENT_INIT))
    {
        ICP_MUTEX_LOCK(&adfDevicesLock);
        ICP_REMOVE_ELEMENT_FROM_LIST(adf, adfDevices, adfDevicesHead);
        ICP_MUTEX_UNLOCK(&adfDevicesLock);
        accel_dev->lac_dev = NULL;
        kfree(adf);
    }

    if (status == CPA_STATUS_SUCCESS && event == ADF_EVENT_START)
    {
        ICP_MUTEX_LOCK(&adfDevicesLock);
        adf->adfSubsystemStatus = 1;
        ICP_MUTEX_UNLOCK(&adfDevicesLock);
    }

    if ((status == CPA_STATUS_SUCCESS && event == ADF_EVENT_STOP) ||
        (status == CPA_STATUS_RETRY && event == ADF_EVENT_STOP))
    {
        ICP_MUTEX_LOCK(&adfDevicesLock);
        adf->adfSubsystemStatus = 0;
        ICP_MUTEX_UNLOCK(&adfDevicesLock);
        status = CPA_STATUS_SUCCESS;
    }

    if (event == ADF_EVENT_SHUTDOWN)
    {
        module_put(qat_api_module);
    }
    return status;
}

/*
 * icp_adf_subsystemRegister
 * adapter function from SAL to adf driver
 * call adf_service_register from adf driver directly with same
 * parameters
 */
CpaStatus icp_adf_subsystemRegister(
    subservice_registation_handle_t *sal_service_reg_handle)
{
    if (salService != NULL)
        return CPA_STATUS_FAIL;

    salService = sal_service_reg_handle;
    adfService.name = sal_service_reg_handle->subsystem_name;
    adfService.event_hld = adf_event_handler;

    if (adf_service_register(&adfService) == 0)
    {
        return CPA_STATUS_SUCCESS;
    }
    else
    {
        salService = NULL;
        return CPA_STATUS_FAIL;
    }
}

/*
 * icp_adf_subsystemUnegister
 * adapter function from SAL to adf driver
 */
CpaStatus icp_adf_subsystemUnregister(
    subservice_registation_handle_t *sal_service_reg_handle)
{
    if (adf_service_unregister(&adfService) == 0)
    {
        salService = NULL;
        return CPA_STATUS_SUCCESS;
    }
    else
    {
        return CPA_STATUS_FAIL;
    }
}

/*
 * icp_adf_cfgGetParamValue
 * get parameter value from section @section with key @param
 */
CpaStatus icp_adf_cfgGetParamValue(icp_accel_dev_t *adf,
                                   const char *section,
                                   const char *param,
                                   char *value)
{
    if (adf_cfg_get_param_value(adf->accel_dev, section, param, value) == 0)
    {
        return CPA_STATUS_SUCCESS;
    }
    else
    {
        return CPA_STATUS_FAIL;
    }
}

/*
 * icp_adf_getAccelDevByAccelId
 * return acceleration device with id @accelId
 */
icp_accel_dev_t *icp_adf_getAccelDevByAccelId(Cpa32U accelId)
{
    icp_accel_dev_t *adf = NULL;

    ICP_MUTEX_LOCK(&adfDevicesLock);
    adf = adfDevicesHead;
    while (adf != NULL && adf->accelId != accelId)
        adf = adf->pNext;
    ICP_MUTEX_UNLOCK(&adfDevicesLock);
    return adf;
}

/*
 * icp_amgr_getNumInstances
 * Return the number of acceleration devices it the system.
 */
CpaStatus icp_amgr_getNumInstances(Cpa16U *pNumInstances)
{
    icp_accel_dev_t *adf = NULL;
    Cpa16U count = 0;

    ICP_MUTEX_LOCK(&adfDevicesLock);
    for (adf = adfDevicesHead; adf != NULL; adf = adf->pNext)
        count++;
    ICP_MUTEX_UNLOCK(&adfDevicesLock);
    *pNumInstances = count;
    return CPA_STATUS_SUCCESS;
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
    icp_accel_dev_t *adf = NULL;
    *pNumInstances = 0;

    ICP_MUTEX_LOCK(&adfDevicesLock);
    for (adf = adfDevicesHead; adf != NULL; adf = adf->pNext)
    {
        if (adf->accelCapabilitiesMask & capabilitiesMask)
        {
            if (adf->adfSubsystemStatus)
            {
                pAccel_devs[0] = adf;
                *pNumInstances = 1;
                ICP_MUTEX_UNLOCK(&adfDevicesLock);
                return CPA_STATUS_SUCCESS;
            }
        }
    }
    ICP_MUTEX_UNLOCK(&adfDevicesLock);
    return CPA_STATUS_FAIL;
}

/*
 * icp_amgr_getAllAccelDevByEachCapabilities
 * Returns table of accel devices that are started and implement
 * each of the capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAllAccelDevByEachCapability(Cpa32U capabilitiesMask,
                                                  icp_accel_dev_t **pAccel_devs,
                                                  Cpa16U *pNumInstances)
{
    icp_accel_dev_t *adf = NULL;
    *pNumInstances = 0;
    ICP_MUTEX_LOCK(&adfDevicesLock);
    for (adf = adfDevicesHead; adf != NULL; adf = adf->pNext)
    {
        Cpa32U enabled_caps = adf->accelCapabilitiesMask & capabilitiesMask;
        if (enabled_caps == capabilitiesMask)
        {
            if (adf->adfSubsystemStatus)
            {
                pAccel_devs[(*pNumInstances)++] = (icp_accel_dev_t *)adf;
            }
        }
    }
    ICP_MUTEX_UNLOCK(&adfDevicesLock);
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_amgr_getAllAccelDevByCapabilities
 * Fetches accel devices based on the capability
 * and returns the count of the device
 */
CpaStatus icp_amgr_getAllAccelDevByCapabilities(Cpa32U capabilitiesMask,
                                                icp_accel_dev_t **pAccel_devs,
                                                Cpa16U *pNumInstances)
{
    icp_accel_dev_t *adf = NULL;
    Cpa16U i = 0;

    ICP_MUTEX_LOCK(&adfDevicesLock);
    for (adf = adfDevicesHead; adf != NULL; adf = adf->pNext)
    {
        if (adf->accelCapabilitiesMask & capabilitiesMask)
        {
            if (adf->adfSubsystemStatus)
            {
                pAccel_devs[i++] = adf;
            }
        }
    }
    ICP_MUTEX_UNLOCK(&adfDevicesLock);
    *pNumInstances = i;
    if (i == 0)
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_amgr_getAccelDevCapabilities
 * Returns accel devices capabilities specified in capabilitiesMask.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_amgr_getAccelDevCapabilities(icp_accel_dev_t *accel_dev,
                                           Cpa32U *pCapabilitiesMask)
{
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pCapabilitiesMask);

    *pCapabilitiesMask = accel_dev->accelCapabilitiesMask;
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_qa_dev_get
 *
 * Description:
 * Function increments the device usage counter.
 *
 * Returns: void
 */
void icp_qa_dev_get(icp_accel_dev_t *pDev)
{
    ICP_CHECK_FOR_NULL_PARAM_VOID(pDev);
    adf_dev_get(pDev->accel_dev);
}

/*
 * icp_qa_dev_put
 *
 * Description:
 * Function decrements the device usage counter.
 *
 * Returns: void
 */
void icp_qa_dev_put(icp_accel_dev_t *pDev)
{
    ICP_CHECK_FOR_NULL_PARAM_VOID(pDev);
    adf_dev_put(pDev->accel_dev);
}

Cpa16U icp_adf_get_busAddress(Cpa16U packageId)
{
    Cpa16U busAddr = 0xFFFF;
    icp_accel_dev_t *adf = NULL;

    ICP_MUTEX_LOCK(&adfDevicesLock);
    for (adf = adfDevicesHead; adf != NULL; adf = adf->pNext)
    {
        if (adf->accelId == packageId)
        {
            busAddr = accel_to_pci_dev(adf->accel_dev)->bus->number |
                      PCI_SLOT(accel_to_pci_dev(adf->accel_dev)->devfn) << 3 |
                      PCI_FUNC(accel_to_pci_dev(adf->accel_dev)->devfn);
            break;
        }
    }
    ICP_MUTEX_UNLOCK(&adfDevicesLock);
    return busAddr;
}

CpaBoolean icp_adf_isSubsystemStarted(
    subservice_registation_handle_t *subsystem_hdl)
{
    if (subsystem_hdl == salService)
        return CPA_TRUE;
    else
        return CPA_FALSE;
}

CpaBoolean icp_adf_is_dev_in_reset(icp_accel_dev_t *accel_dev)
{
    return (CpaBoolean)accel_dev_reset_stat[accel_dev->accelId];
}

CpaBoolean icp_adf_is_dev_in_error(icp_accel_dev_t *accel_dev)
{
    return (CpaBoolean)accel_dev_error_stat[accel_dev->accelId];
}

int adf_module_load(void)
{
    CpaStatus ret;

    LAC_LOG_DEBUG("Loading SAL Module ...\n");

    ICP_MUTEX_INIT(&adfDevicesLock);
    ret = SalCtrl_AdfServicesRegister();
    if (CPA_STATUS_SUCCESS != ret)
    {
        ICP_MUTEX_UNINIT(&adfDevicesLock);
        return -EFAULT;
    }

    return 0;
}

void adf_module_unload(void)
{
    LAC_LOG_DEBUG("Unloading SAL Module ...\n");

    SalCtrl_AdfServicesUnregister();
    ICP_MUTEX_UNINIT(&adfDevicesLock);
}
