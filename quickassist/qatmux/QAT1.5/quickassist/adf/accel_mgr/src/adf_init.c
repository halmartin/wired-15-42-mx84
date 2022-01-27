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
 * @file  adf_init.c
 *
 * @description
 *        This file contains the subcomponent module initialisation code
 *        for the Acceleration Driver Framework (ADF).
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_firml_interface.h"
#include "icp_adf_init.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_init.h"
#include "adf_ae.h"
#include "adf_isr.h"
#include "adf_cfg.h"
#include "adf_devmgr.h"
#include "adf_sysmeminfo.h"
#include "adf_ae_fw.h"
#include "adf_fw.h"
#include "adf_user_proxy.h"
#include "adf_drv.h"
#include "adf_drv_sriov.h"
#include "icp_adf_accel_mgr.h"

#define DH89xxCC_MINOR_STEPPING_MASK  0x3
#define DH89xxCC_MAJOR_STEPPING_MASK  0x30
#define DH89xxCC_MAJOR_STEPPING_SHIFT 0x4

#define C2XXX_MINOR_STEPPING_MASK  0x1
#define C2XXX_MAJOR_STEPPING_MASK  0x6
#define C2XXX_MAJOR_STEPPING_SHIFT 0x1
/*
 * The subsystem_table variable described the current position of the
 * tail of the list, this will also be the newest entry to the list.
 * The subsystem_table_head contains the pointer to the head of the list.
 */
STATIC subservice_registation_handle_t *pSubsystemTable = NULL;
STATIC subservice_registation_handle_t *pSubsystemTableHead = NULL;
STATIC ICP_MUTEX subsystemTableLock;
/*
 * adf_subsystemAdd
 * Add a new subsystem structure to the subsystem Table
 */
STATIC inline CpaStatus
adf_subsystemAdd(subservice_registation_handle_t* subsystem)
{
    CpaStatus status = CPA_STATUS_FAIL;
    subservice_registation_handle_t *subsystem_hdl = NULL;

    ICP_CHECK_FOR_NULL_PARAM(subsystem);

    subsystem_hdl = pSubsystemTableHead;
    if( NULL == pSubsystemTableHead )
    {
        ICP_MUTEX_INIT(&subsystemTableLock);
    }

    ICP_MUTEX_LOCK(&subsystemTableLock);
    /*Search the linked list for the subsystem. */
    ICP_FIND_ELEMENT_IN_LIST(subsystem, subsystem_hdl, status);
    if (CPA_STATUS_SUCCESS == status)
    {
        ADF_ERROR("subservice %s already in table.\n",
                      subsystem->subsystem_name);
        ICP_MUTEX_UNLOCK(&subsystemTableLock);
        return CPA_STATUS_FAIL;
    }

    ICP_ADD_ELEMENT_TO_END_OF_LIST(subsystem,
                                   pSubsystemTable, pSubsystemTableHead);

    ICP_MUTEX_UNLOCK(&subsystemTableLock);
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_subsystemRemove
 * Remove the subsystem structure from the subsystem Table
 * Returns CPA_STATUS_SUCCESS on success,
 * CPA_STATUS_FAIL if a NULL pointer supplied
 */
STATIC inline CpaStatus
adf_subsystemRemove(subservice_registation_handle_t * subsystem)
{
    subservice_registation_handle_t *subsystem_hdl = NULL;
    CpaStatus status = CPA_STATUS_FAIL;

    ICP_CHECK_FOR_NULL_PARAM(subsystem);

    subsystem_hdl = pSubsystemTableHead;

    ICP_MUTEX_LOCK(&subsystemTableLock);
    ICP_FIND_ELEMENT_IN_LIST(subsystem, subsystem_hdl, status);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("subservice %s not found.\n",
                      subsystem->subsystem_name);
        ICP_MUTEX_UNLOCK(&subsystemTableLock);
        return CPA_STATUS_FAIL;
    }
    else
    {
        ADF_DEBUG("subservice in table - removing.\n");
    }

    ICP_REMOVE_ELEMENT_FROM_LIST(subsystem,
                                 pSubsystemTable, pSubsystemTableHead);

    ICP_MUTEX_UNLOCK(&subsystemTableLock);
    if( NULL == pSubsystemTableHead )
    {
        ICP_MUTEX_UNINIT(&subsystemTableLock);
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * do_catchup
 * Function sends init & start event to a subsystem.
 */
STATIC CpaStatus do_catchup(icp_accel_dev_t *accel_dev,
                   subservice_registation_handle_t* subsystem_hdl)
{
    CpaStatus status = CPA_STATUS_FAIL;
    CpaStatus ret_status = CPA_STATUS_FAIL;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(subsystem_hdl);

    ADF_DEBUG("Sending event %d to %s\n", ICP_ADF_EVENT_INIT,
                                    subsystem_hdl->subsystem_name);

    status = subsystem_hdl->subserviceEventHandler(accel_dev,
                                             ICP_ADF_EVENT_INIT,
                                             NULL);

    if( CPA_STATUS_SUCCESS == status )
    {
        SET_STATUS_BIT(subsystem_hdl->subsystemStatus
                       [accel_dev->accelId].subsystemInitBit, 0);

        ADF_DEBUG("Sending event %d to %s\n", ICP_ADF_EVENT_START,
                                         subsystem_hdl->subsystem_name);

        status = subsystem_hdl->subserviceEventHandler(accel_dev,
                                                 ICP_ADF_EVENT_START,
                                                 NULL);
        /* Check if start was successful */
        if( CPA_STATUS_SUCCESS == status )
        {
            SET_STATUS_BIT(subsystem_hdl->subsystemStatus
                       [accel_dev->accelId].subsystemStartBit, 0);
        }
        else
        {
            ADF_ERROR("Failed to start Subservice %s\n",
                               subsystem_hdl->subsystem_name);

            ADF_DEBUG("Sending event %d to %s\n", ICP_ADF_EVENT_SHUTDOWN,
                                               subsystem_hdl->subsystem_name);

            /* Send shutdown the subsystem if start failed */
            ret_status = subsystem_hdl->subserviceEventHandler(accel_dev,
                                             ICP_ADF_EVENT_SHUTDOWN,
                                             NULL);
            if( CPA_STATUS_SUCCESS == ret_status )
            {
                CLEAR_STATUS_BIT(subsystem_hdl->subsystemStatus
                       [accel_dev->accelId].subsystemInitBit, 0);
            }
        }
    }
    else
    {
        ADF_ERROR("Failed to initialise Subservice %s\n",
                   subsystem_hdl->subsystem_name);
    }
    return status;
}


/*
 * icp_adf_subsystemRegister
 * Register a new subsystem.
 */
CpaStatus
icp_adf_subsystemRegister(subservice_registation_handle_t* subsystem_hdl)
{
    CpaStatus status = CPA_STATUS_FAIL;
    icp_accel_dev_t *accel_dev=NULL;
    ICP_CHECK_FOR_NULL_PARAM(subsystem_hdl);

    ADF_DEBUG("Registering subsystem %s\n", subsystem_hdl->subsystem_name);

    status = adf_subsystemAdd(subsystem_hdl);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add subsystem to the list.\n");
        return CPA_STATUS_FAIL;
    }
    status = adf_devmgrGetAccelHead(&accel_dev);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to get accel head.\n");
        adf_subsystemRemove(subsystem_hdl);
        return CPA_STATUS_SUCCESS;
    }
    /* Loop over each accel_dev in the system and clear the status
     * bits for each one. */
    while(accel_dev)
    {
        /* Initialising the masks during the subsystem registration
         * and prior to init. */
        CLEAR_STATUS_BIT(subsystem_hdl->subsystemStatus
                         [accel_dev->accelId].subsystemInitBit, 0);
        CLEAR_STATUS_BIT(subsystem_hdl->subsystemStatus
                         [accel_dev->accelId].subsystemStartBit, 0);

        /* Send init and start events to subsystem when ADF is started */
        if( BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                        ADF_STATUS_SYSTEM_STARTED))
        {
            status = do_catchup(accel_dev, subsystem_hdl);
            if( CPA_STATUS_SUCCESS != status )
            {
                ADF_ERROR("Failed to start subservice %s for device %d.\n",
                          subsystem_hdl->subsystem_name,
                          accel_dev->accelId );
            }
        }
        accel_dev = accel_dev->pNext;
    }
    return status;
}

/*
 * do_shutdown
 * Function sends a shutdown event to a subsystem.
 */
STATIC CpaStatus do_shutdown(icp_accel_dev_t *accel_dev,
                   subservice_registation_handle_t* subsystem_hdl)
{
    CpaStatus status = CPA_STATUS_FAIL;
    ICP_CHECK_FOR_NULL_PARAM(subsystem_hdl);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_PARAM_RANGE(accel_dev->accelId, 0, ADF_MAX_DEVICES - 1);

    /* Shutdown the subsystem if required */
    if (BIT_IS_SET(subsystem_hdl->subsystemStatus
                   [accel_dev->accelId].subsystemInitBit, 0))
    {
        /* Send shutdown event */
        ADF_DEBUG("Sending event %d to %s\n", ICP_ADF_EVENT_SHUTDOWN,
                                        subsystem_hdl->subsystem_name);

        status = subsystem_hdl->subserviceEventHandler(accel_dev,
                 ICP_ADF_EVENT_SHUTDOWN, NULL);

        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to shutdown subservice %s\n",
                          subsystem_hdl->subsystem_name);
        }
        else
        {
            CLEAR_STATUS_BIT(subsystem_hdl->subsystemStatus
                         [accel_dev->accelId].subsystemInitBit, 0);
        }
    }
    return status;
}

/*
 * icp_adf_subsystemUnregister
 * Unregister a subsystem that is about to be removed.
 * If the system is initialised and started it will be stopped first.
 * This function also allows for an asynchronous shutdown of the
 * subservice depending on the signal returned by the subservice
 * from the STOP event.
 */
CpaStatus
icp_adf_subsystemUnregister(subservice_registation_handle_t* subsystem_hdl)
{
    CpaStatus status = CPA_STATUS_FAIL;
    icp_accel_dev_t *accel_dev=NULL;
    Cpa32U sleepflag=0;
    ICP_CHECK_FOR_NULL_PARAM(subsystem_hdl);

    ADF_DEBUG("Unregistering subsystem %s\n", subsystem_hdl->subsystem_name);

    status = adf_devmgrGetAccelHead(&accel_dev);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to get accel head.\n");
        return status;
    }
    /*
     * Loop for each device and send stop
     */
    while(accel_dev)
    {
        /* Stop the subsystem if required */
        if (BIT_IS_SET(subsystem_hdl->subsystemStatus
                       [accel_dev->accelId].subsystemStartBit, 0))
        {
            ADF_DEBUG("Sending event %d to %s\n", ICP_ADF_EVENT_STOP,
                                 subsystem_hdl->subsystem_name);
            status = subsystem_hdl->subserviceEventHandler(accel_dev,
                                                     ICP_ADF_EVENT_STOP,
                                                     NULL);
            if (CPA_STATUS_SUCCESS != status)
            {
                if (CPA_STATUS_RETRY == status)
                {
                    sleepflag++;
                    CLEAR_STATUS_BIT(subsystem_hdl->subsystemStatus
                             [accel_dev->accelId].subsystemStartBit, 0);

                    ADF_DEBUG("Received pending from subservice %s.\n",
                               subsystem_hdl->subsystem_name );
                }
                else
                {

                    ADF_ERROR("Failed to stop subservice %s for dev %d\n",
                            subsystem_hdl->subsystem_name,
                            accel_dev->accelId);
                }
            }
            else
            {
                CLEAR_STATUS_BIT(subsystem_hdl->subsystemStatus
                             [accel_dev->accelId].subsystemStartBit, 0);
            }
        }
        accel_dev = accel_dev->pNext;
    }

    /* sleep for PENDING_DELAY msecs before calling shutdown. */
    if(sleepflag)
    {
        ICP_MSLEEP(PENDING_DELAY);
    }

    status = adf_devmgrGetAccelHead(&accel_dev);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to get accel head.\n");
        return status;
    }
    /*
     * Loop for each device and send shutdown
     */
    while(accel_dev)
    {
        /*Shutdown the subsystem now no matter what. */
        status = do_shutdown(accel_dev, subsystem_hdl);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_DEBUG("Removing subservice from the subservice table.\n");
        }
        accel_dev = accel_dev->pNext;
    }
    return adf_subsystemRemove(subsystem_hdl);
}

/*
 * adf_cfgAddInternalValues
 * Add internal config values to general section of the given accel dev
 */
CpaStatus
adf_cfgAddInternalValues(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_FAIL;
    adf_cfg_device_data_t *cfg = NULL;
    adf_cfg_section_t *section_general = NULL;
    Cpa32U *qatMask = NULL;
    Cpa32U i = 0;
    Cpa64U val = 0;
    char keyStr[ADF_CFG_MAX_KEY_LEN_IN_BYTES] = {0};
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32S enabled = 0;
    Cpa32U rev_id = accel_dev->pciAccelDev.revisionId;
    char maj_step = 'A';
    int  min_step  = 0;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U maxNumAccelerators = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);

    status = adf_cfgDeviceFind(accel_dev->accelId, &cfg);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to find cfg device\n");
        return status;
    }

    status = icp_adf_cfgGetParamValue(accel_dev,
                                      GENERAL_SEC,
                                      ADF_PROC_DEBUG,
                                      (char*)valStr);
    if (CPA_STATUS_SUCCESS == status)
    {
        enabled = ICP_STRTOUL((char*)valStr, NULL, ADF_CFG_BASE_DEC);
    }
    if (enabled)
    {
        status = adf_debug_create_cfg(accel_dev, cfg);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to create cfg debug file\n");
            return status;
        }
    }
    if (BIT_IS_SET(accel_dev->adfSubsystemStatus,
                              ADF_STATUS_SYSTEM_RESTARTING))
    {
        /*
         * If we are in reset triggered from error handling
         * all the values are already there.
         */
        return CPA_STATUS_SUCCESS;
    }

    section_general = adf_cfgSectionFind(cfg->config_section,
                                         GENERAL_SEC);
    if (NULL == section_general)
    {
        ADF_ERROR("Could not find section %s\n", GENERAL_SEC);
        return CPA_STATUS_FAIL;
    }

    /* Add device node id */
    val = accel_dev->pkg_id;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_NODE_ID, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add device device node id\n");
        return status;
    }

    /* Add device pkg id */
    val = accel_dev->accelId;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_PKG_ID, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add device pkg id\n");
        return status;
    }

    /* Add device bus address */
    val = icp_adf_get_busAddress((Cpa16U)(accel_dev->accelId));
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_BUS_ADDRESS, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add device bus address\n");
        return status;
    }

    hw_data = accel_dev->pHwDeviceData;

    /* Add number of Acceleration Engines */
    val = hw_data->getNumAccelEngines(hw_data, accel_dev->aeMask);
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_NUM_AE, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add number of Acceleration Engines\n");
        return status;
    }

    /* Add number of Accelerators */
    val = hw_data->getNumAccelerators(hw_data, accel_dev->accelMask);
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_NUM_ACCEL, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add number of Accelerators\n");
        return status;
    }

    val = hw_data->maxNumAccelEngines;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_MAX_AE, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add maximum of Acceleration Engines\n");
        return status;
    }

    maxNumAccelerators = GET_MAX_ACCEL(accel_dev);
    val = maxNumAccelerators;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_MAX_ACCEL, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add maximum of Acceleration Engines\n");
        return status;
    }

    val = GET_MAX_BANKS(accel_dev);
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_MAX_BANKS, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add maximum Number of Banks\n");
        return status;
    }

    val = GET_NUM_RINGS_PER_BANK(accel_dev);
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_MAX_RINGS_PER_BANK, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add maximum Rings Per Bank\n");
        return status;
    }

    val = hw_data->userEtringCsrSize;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ADF_DEV_USER_RING_CSR_SIZE, (void*)&val,
            ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add User Space Ring Csr Size\n");
        return status;
    }

    val = accel_dev->accelCapabilitiesMask;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_CAPABILITIES_MASK, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add capabilities mask\n");
        return status;
    }

    val = accel_dev->accelMask;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_ACCEL_MASK, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add accel mask\n");
        return status;
    }

    val = accel_dev->aeMask;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                  ADF_DEV_AE_MASK, (void*)&val,
                                  ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add AE mask\n");
        return status;
    }

    val = accel_dev->sku;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                     ADF_DEV_SKU, (void*)&val,
                                     ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add AE mask\n");
        return status;
    }

    /*
     * Calculate the accelengines associated with each
     * accelerator and add each mask to the config table
     */
    qatMask = ICP_MALLOC_GEN_NUMA(sizeof (*qatMask) * maxNumAccelerators,
            accel_dev->pkg_id);
    if (NULL == qatMask)
    {
        ADF_ERROR("Failed to allocate memory\n");
        return CPA_STATUS_FAIL;
    }

    hw_data->getAccelMaskList(hw_data,
                accel_dev->accelMask, accel_dev->aeMask, qatMask);

    for(i=0; i < maxNumAccelerators; i++)
    {
        ICP_MEMSET(keyStr, 0, ADF_CFG_MAX_KEY_LEN_IN_BYTES);
        ICP_MEMSET(valStr, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
        snprintf(keyStr, ADF_CFG_MAX_KEY_LEN_IN_BYTES,
                ADF_DEV_ACCELAE_MASK_FMT, i);

        snprintf(valStr, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                "0x%X", qatMask[i]);

        status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                         keyStr, (void *)valStr, ADF_STR);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to add Accel/AE Mask %d\n", i);
            ICP_FREE_NUMA(qatMask);
            return status;
        }
    }
    ICP_FREE_NUMA(qatMask);
    ICP_MEMSET(keyStr, 0, ADF_CFG_MAX_KEY_LEN_IN_BYTES);
    if(DEV_C2XXX == hw_data->dev_class->type)
    {
        maj_step += ((rev_id & C2XXX_MAJOR_STEPPING_MASK)
            >> C2XXX_MAJOR_STEPPING_SHIFT);
        min_step  += (rev_id & C2XXX_MINOR_STEPPING_MASK);
    }
    else
    {
        maj_step += ((rev_id & DH89xxCC_MAJOR_STEPPING_MASK)
            >> DH89xxCC_MAJOR_STEPPING_SHIFT);
        min_step  += (rev_id & DH89xxCC_MINOR_STEPPING_MASK);
    }
    snprintf(valStr, ADF_CFG_MAX_VAL_LEN_IN_BYTES, "%c%d", maj_step, min_step);
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                     ICP_CFG_HW_REV_ID_KEY,
                                     (void *)valStr, ADF_STR);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add HW version\n");
        return status;
    }

    val = adf_VF2PF_get_ring_offset(accel_dev);
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
                                     ADF_VF_RING_OFFSET_KEY,
                                     &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add vf ring offset\n");
        return status;
    }
    return status;
}

/*
 * This function initiates the initialisation of all sub-component modules.
 * Sub-component initialisation involves initing the AEs, allocating
 * interrupt resources, loading the firmware and sending an INIT event to
 * the subservice.
 */
CpaStatus
adf_subsystemInit(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_FAIL;
    char valStr[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U num_wireless_procs = 0;
#ifdef ICP_SRIOV
    Cpa32U sriov_enabled = 0;
#endif
    adf_hw_device_data_t *hw_data = NULL;
    subservice_registation_handle_t *subsystem_hdl = pSubsystemTableHead;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);
    ICP_CHECK_PARAM_RANGE(accel_dev->accelId, 0, ADF_MAX_DEVICES - 1);

    /* Retrieve the device configuration data */
    hw_data = accel_dev->pHwDeviceData;

    /*
     * Set the status to "STARTING" so that in case of a fail
     * during init or start the stop process will be able to
     * clean everything up.
     */
    SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                   ADF_STATUS_SYSTEM_STARTING);

    /*
     * Sub-component initialisation is divided into two stages: init and start.
     * This is to facilitate any ordering dependencies between sub-components
     * prior to starting any of the accelerators.
     */
    ADF_DEBUG("Calling adf_aeInit\n");
    status = adf_aeInit(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to initialise Acceleration Engine\n");
        return CPA_STATUS_FAIL;
    }
    else
    {
        SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_AE_INITIALISED);
    }

    /*
     * Allocate interrupt resources
     */
    ADF_DEBUG("Calling adf_isr_resource_alloc\n");
    status = adf_isr_resource_alloc(accel_dev);
    if ( SUCCESS != status )
    {
        ADF_ERROR("Failed to allocate IRQ resources\n");
        return CPA_STATUS_FAIL;
    }
    else
    {
        SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_ISR_RES_ALLOCATED);
    }

    /*
     * Activate Uncorrectable Errors Interrupts for supported devices
     */
    if(NULL != hw_data->enableUncoErrInterrupts)
    {
        ADF_DEBUG("Activate Uncorrectable Errors Interrupts\n");
        status = hw_data->enableUncoErrInterrupts(accel_dev);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to activate Uncorrectable Errors Interrupts\n");
            return CPA_STATUS_FAIL;
        }
    }

#ifdef ICP_SRIOV
    if(DEV_DH89XXCC == hw_data->dev_class->type)
    {
        /* Check if we want to virtualize the device
         * NOTE: this needs to be done before adf_aeFwLoad as the MMP address
         * needs to be remapped after VF is added to iommu domain.
         */
        status = icp_adf_cfgGetParamValue(accel_dev,
                                          GENERAL_SEC,
                                          ADF_SRIOV_ENABLE_KEY,
                                          (char*)valStr);
        if (CPA_STATUS_SUCCESS == status)
        {
            sriov_enabled = ICP_STRTOUL((char*)valStr, NULL, ADF_CFG_BASE_DEC);
        }
        status = CPA_STATUS_SUCCESS;
        if(sriov_enabled)
        {
            if (SUCCESS != adf_enable_sriov(accel_dev))
            {
                return CPA_STATUS_FAIL;
            }
            SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                                      ADF_STATUS_SRIOV_ENABLED);
        }
        else
        {
            ADF_ERROR("Invalid %s value, driver is built with SRIOV.\n",
                ADF_SRIOV_ENABLE_KEY);
            return CPA_STATUS_FAIL;
        }
     }
#endif

    /*
     * Load firmware
     */
    ADF_DEBUG("Calling adf_aeFwLoad\n");
    status = adf_aeFwLoad(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to load firmware\n");
        adf_aeReleaseFirmware(accel_dev);
        return CPA_STATUS_FAIL;
    }
    else
    {
        SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_AE_UCODE_MAPPED);
    }

    /* Release the FW - local copies are kept in memory. */
    status = adf_aeReleaseFirmware(accel_dev);
    if(status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Failed to release fw\n");
        return CPA_STATUS_FAIL;
    }
    /*
     * Add internal values to cfg table including
     * node_id, pkg_id, num of AEs and num of
     * accelerators.
     */
    ADF_DEBUG("Calling adf_cfgAddInternalValues\n");
    status = adf_cfgAddInternalValues(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed add internal config values\n");
        return CPA_STATUS_FAIL;
    }

    /* Check if we are running wireless firmware */
    status = icp_adf_cfgGetParamValue(accel_dev,
                                      GENERAL_SEC,
                                      NUMBER_OF_WIRELESS_PROCS,
                                      (char*)valStr);
    if (CPA_STATUS_SUCCESS == status)
    {
       num_wireless_procs = ICP_STRTOUL((char*)valStr, NULL, ADF_CFG_BASE_DEC);
    }
    status = CPA_STATUS_SUCCESS;
    if(num_wireless_procs > 0)
    {
        SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                                  ADF_STATUS_WIRELESS_ENABLED);
    }
    /*
     * Initialise all subsystems if not already inited
     */
    while (NULL != subsystem_hdl)
    {
        ADF_DEBUG("Sending event %d to %s\n", ICP_ADF_EVENT_INIT,
                                 subsystem_hdl->subsystem_name);
        /*
         * A function pointer is used as the final argument to the
         * init event so that Hal is correctly initialised each time as well.
         */
        status = subsystem_hdl->subserviceEventHandler(accel_dev,
                 ICP_ADF_EVENT_INIT,
                 NULL);

        if (CPA_STATUS_SUCCESS == status)
        {
            SET_STATUS_BIT(subsystem_hdl->subsystemStatus
                       [accel_dev->accelId].subsystemInitBit, 0);
        }
        else
        {
            ADF_ERROR("Failed to initialise Subservice %s\n",
                          subsystem_hdl->subsystem_name);
            return status;
        }
        subsystem_hdl = subsystem_hdl->pNext;
    }

    /*
     * Download Ucode - write firmware to the AEs
     */
    ADF_DEBUG("Calling adf_aeUcodeDownload\n");
    status = adf_aeUcodeDownload(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to download firmware\n");
        return status;
    }
    else
    {
        SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_AE_UCODE_LOADED);
    }

    status = adf_VF2PF_init(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to send init event to PF\n");
        return status;
    }

    /*
     * Finally start all subservice and the AEs.
     */
    ADF_DEBUG("Calling adf_subsystemStart\n");
    return adf_subsystemStart(accel_dev);
}

/*
 * This function starts the AEs and the subservices as well as downloading
 * the ucode.
 */
CpaStatus
adf_subsystemStart(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_FAIL;
    subservice_registation_handle_t *subsystem_hdl = pSubsystemTableHead;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_PARAM_RANGE(accel_dev->accelId, 0, ADF_MAX_DEVICES - 1);

    /*
     * Start AE
     */
    ADF_DEBUG("Calling adf_aeStart\n");
    status = adf_aeStart(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("AE Start Failed\n");
        return status;
    }
    else
    {
        SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_AE_STARTED);
    }

    ADF_DEBUG("Calling adf_aefwGetVersion\n");
    status = adf_aefwGetVersion(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Could not get firmware version\n");
        return status;
    }

    /*
     * Start the sub-component services in the sequence how they registered.
     * Depending on whether the subservices are being started from scratch.
     */
    while (NULL != subsystem_hdl)
    {
        icp_adf_subsystemEvent_t event = ICP_ADF_EVENT_START;
        ADF_DEBUG("Sending event %d to %s\n", event,
                                 subsystem_hdl->subsystem_name);

        status = subsystem_hdl->subserviceEventHandler(accel_dev,
                                                       event, NULL);
        if (CPA_STATUS_SUCCESS == status)
        {
            SET_STATUS_BIT(subsystem_hdl->subsystemStatus
                       [accel_dev->accelId].subsystemStartBit,0);
        }
        else
        {
            ADF_ERROR("Failed to start subservice %s\n",
                          subsystem_hdl->subsystem_name);
            return status;
        }
        subsystem_hdl = subsystem_hdl->pNext;
    }

    /*
     * Once the start process is completed - set the
     * status to "STARTED".
     */
    SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                      ADF_STATUS_SYSTEM_STARTED);

    CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_SYSTEM_STARTING);
    return status;
}

/*
 * This function initiates the cleanup of all sub-component modules.
 * Sub-component cleanup is also divided into two stages: stop and shutdown.
 * The adf_subsystemStop function stops the subservice, stops the AEs,
 * releases the ucode and releases the firmware.
 */
CpaStatus
adf_subsystemStop(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    subservice_registation_handle_t *subsystem_hdl = pSubsystemTableHead;
    Cpa32U sleepflag=0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_PARAM_RANGE(accel_dev->accelId, 0, ADF_MAX_DEVICES - 1);

    CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                        ADF_STATUS_SYSTEM_STARTED);
    CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_SYSTEM_STARTING);
   /*
     * Stop registered subsystems. Send a stop event.
     * An asynchronous stop is also employed in case the subsystem
     * returns retry.
     */
    while (NULL != subsystem_hdl)
    {
        if (BIT_IS_SET(subsystem_hdl->subsystemStatus
                       [accel_dev->accelId].subsystemStartBit, 0))
        {
            icp_adf_subsystemEvent_t event = ICP_ADF_EVENT_STOP;
            ADF_DEBUG("Sending event %d to %s\n", event,
                                 subsystem_hdl->subsystem_name);

            status = subsystem_hdl->subserviceEventHandler(accel_dev,
                                                           event, NULL);
            if (CPA_STATUS_FAIL == status)
            {
                ADF_ERROR("Failed to stop subservice %s.\n",
                          subsystem_hdl->subsystem_name);
            }
            else
            {
                if (CPA_STATUS_RETRY == status)
                {
                    sleepflag++;
                    ADF_DEBUG("Pending received from %s\n",
                        subsystem_hdl->subsystem_name);
                }
                CLEAR_STATUS_BIT(subsystem_hdl->subsystemStatus
                             [accel_dev->accelId].subsystemStartBit, 0);
            }
        }
        subsystem_hdl = subsystem_hdl->pNext;
    }

   /*
    * AE stop
    */
    if (BIT_IS_SET(accel_dev->adfSubsystemStatus, ADF_STATUS_AE_STARTED))
    {
        status = adf_aeStop(accel_dev);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("failed to stop AE\n");
        }
        else
        {
            CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                             ADF_STATUS_AE_STARTED);
            ADF_DEBUG("Successfully stopped AE\n");
        }
    }

    /*
     * If a pending was received need to return RETRY.
     */
    if (sleepflag && (!icp_adf_is_dev_in_reset(accel_dev)))
    {
        ICP_MSLEEP(PENDING_DELAY);
        status = CPA_STATUS_RETRY;
    }
    return status;
}

/*
 * This function completes the cleanup of all sub-component modules.
 * Memory is freed, subservices and AEs shutdown.
 */
CpaStatus
adf_subsystemShutdown(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    subservice_registation_handle_t *subsystem_hdl = pSubsystemTableHead;
    adf_cfg_device_data_t *pCfgData = NULL;
#ifdef ICP_SRIOV
    adf_hw_device_data_t *hw_data = NULL;
#endif
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

#ifdef ICP_SRIOV
    /* Retrieve the device configuration data */
    hw_data = accel_dev->pHwDeviceData;
#endif

    status = adf_VF2PF_shutdown(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to send shutdown event to PF\n");
    }

    /*
     * Release the ucode
     */
    if (BIT_IS_SET(accel_dev->adfSubsystemStatus, ADF_STATUS_AE_UCODE_MAPPED))
    {
        status = adf_aeUcodeRelease(accel_dev);

        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to release the ucode\n");
        }
        else
        {
            CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                             ADF_STATUS_AE_UCODE_MAPPED);
            CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                             ADF_STATUS_AE_UCODE_LOADED);
        }
    }
    /*
     * Free MMP buffer
     */
    adf_freeMmpBuffer(accel_dev);

    /*
     * Free UOF buffer
     */
    adf_freeUofBuffer(accel_dev);

    /*
    * Free IRQ resources
    */
    if (BIT_IS_SET(accel_dev->adfSubsystemStatus, ADF_STATUS_ISR_RES_ALLOCATED))
    {
        /* No need to check return value as it must return success. */
        status = adf_isr_resource_free(accel_dev);
        CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                             ADF_STATUS_ISR_RES_ALLOCATED);
    }
    /*
     * AE shutdown
     */
    if (BIT_IS_SET(accel_dev->adfSubsystemStatus, ADF_STATUS_AE_INITIALISED))
    {

        status = adf_aeShutdown(accel_dev);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to shutdown Acceleration Engine\n");
        }
        else
        {
            CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                             ADF_STATUS_AE_INITIALISED);
        }
    }
    /*
     * Subservice shutdown
     */
    while (NULL != subsystem_hdl)
    {
        status = do_shutdown(accel_dev, subsystem_hdl);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to shutdown subservice %s. "
                          "Continuing with the shutdown of the AE.\n",
                          subsystem_hdl->subsystem_name);
        }
        subsystem_hdl = subsystem_hdl->pNext;
    }

#ifdef ICP_SRIOV
    if(DEV_DH89XXCC == hw_data->dev_class->type)
    {
       /*
        * Disable sriov
        */
        if (BIT_IS_SET(accel_dev->adfSubsystemStatus,
                     ADF_STATUS_SRIOV_ENABLED))
        {
            adf_disable_sriov(accel_dev);
            CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                             ADF_STATUS_SRIOV_ENABLED);
            ADF_DEBUG("Disabled SRIOV\n");
        }
    }
#endif

   /*
    * Disable wireless
    */
    if (BIT_IS_SET(accel_dev->adfSubsystemStatus, ADF_STATUS_WIRELESS_ENABLED))
    {
        CLEAR_STATUS_BIT(accel_dev->adfSubsystemStatus,
                                ADF_STATUS_WIRELESS_ENABLED);
        ADF_DEBUG("Disabled WIRELESS\n");
    }

    /*
     * Remove the config section on shutdown.
     */
    if (CPA_STATUS_SUCCESS == adf_cfgDeviceFind(accel_dev->accelId,
                                                &pCfgData))
    {
        /* Delete each section, the key values will be
         * deleted also.
         * NOTE: if the reset is triggered by error handling logic
         * the config data can not be freed */
        if (!BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                  ADF_STATUS_SYSTEM_RESTARTING))
        {
            adf_cfgSectionDel(&(pCfgData->config_section));
        }
        adf_debug_remove_cfg(pCfgData);
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Register the call back function with the accelerator
 * structure.
 */
CpaStatus
icp_adf_accesLayerRingInfoCbRegister(icp_accel_dev_t *accel_dev,
                                     ringInfoCb cb)
{
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(cb);
    accel_dev->ringInfoCallBack = (void*) cb;
    return CPA_STATUS_SUCCESS;
}

/*
 * UnRegister the call back function with the accelerator
 * structure.
 */
void
icp_adf_accesLayerRingInfoCbUnregister(icp_accel_dev_t *accel_dev)
{
    if(accel_dev)
    {
        accel_dev->ringInfoCallBack = NULL;
    }
}

/*
 * icp_adf_isSubsystemStarted
 * Function returns true if the service is started
 */
CpaBoolean icp_adf_isSubsystemStarted(subservice_registation_handle_t*
                                                           subsystem_hdl)
{
    Cpa32U i = 0;
    for (i=0; i<ADF_MAX_DEVICES; i++)
    {
        if(BIT_IS_SET(subsystem_hdl->subsystemStatus[i].subsystemStartBit,0))
        {
            return CPA_TRUE;
        }
    }
    return CPA_FALSE;
}

/*
 * icp_adf_isDevStarted
 * Function returns true if the device is started
 */
CpaBoolean icp_adf_isDevStarted(icp_accel_dev_t *accel_dev)
{
    if(BIT_IS_SET(accel_dev->adfSubsystemStatus, ADF_STATUS_SYSTEM_STARTED))
    {
        return CPA_TRUE;
    }
    return CPA_FALSE;
}

/*
 * Function sends restarting event to all subsystems.
 * This function should be used by error handling funct. only
 */
CpaStatus
adf_subsystemRestarting(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    subservice_registation_handle_t *subsystem_hdl = pSubsystemTableHead;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_PARAM_RANGE(accel_dev->accelId, 0, ADF_MAX_DEVICES - 1);

    while (NULL != subsystem_hdl)
    {
        icp_adf_subsystemEvent_t event = ICP_ADF_EVENT_RESTARING;
        ADF_DEBUG("Sending event %d to %s\n", event,
                                 subsystem_hdl->subsystem_name);

        status = subsystem_hdl->subserviceEventHandler(accel_dev,
                                                       event, NULL);
        if (CPA_STATUS_FAIL == status)
        {
            ADF_ERROR("Failed to restart subservice %s.\n",
                          subsystem_hdl->subsystem_name);
        }
        else
        {
            ADF_DEBUG("Pending received from %s\n",
                    subsystem_hdl->subsystem_name);
        }
        subsystem_hdl = subsystem_hdl->pNext;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Function sends restarted event to all subsystems.
 * This function should be used by error handling funct. only
 */
CpaStatus
adf_subsystemRestarted(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    subservice_registation_handle_t *subsystem_hdl = pSubsystemTableHead;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_PARAM_RANGE(accel_dev->accelId, 0, ADF_MAX_DEVICES - 1);

    while (NULL != subsystem_hdl)
    {
        icp_adf_subsystemEvent_t event = ICP_ADF_EVENT_RESTARTED;
        ADF_DEBUG("Sending event %d to %s\n", event,
                                 subsystem_hdl->subsystem_name);

        status = subsystem_hdl->subserviceEventHandler(accel_dev,
                                                       event, NULL);
        if (CPA_STATUS_FAIL == status)
        {
            ADF_ERROR("Failed to restart subservice %s.\n",
                          subsystem_hdl->subsystem_name);
        }
        else
        {
            ADF_DEBUG("Pending received from %s\n",
                    subsystem_hdl->subsystem_name);
        }
        subsystem_hdl = subsystem_hdl->pNext;
    }
    return CPA_STATUS_SUCCESS;
}
