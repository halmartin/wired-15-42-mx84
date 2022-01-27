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
 * @file  adf_wireless_rings.c
 *
 * @description
 *        This file contains the function for ring managnent in wireless
 *        driver mode.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_cfg.h"
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"
#include "adf_platform.h"
#include "adf_wireless.h"

#define INFLIGHT_COMPLETION_MS 250

STATIC char wireless_instance_name[ADF_MAX_DEVICES]
                             [WIRELESS_MAX_PROCS]
                             [WIRELESS_SECTION_NAME_STR_LEN];

STATIC wireless_cy_instance_t *cy_instances[ADF_MAX_DEVICES]
                                        [WIRELESS_MAX_INSTANCES];
/*
 * Dummy kernelspace callback for wireless rings
 */
void adf_dummy_callback(icp_comms_trans_handle comms_handle, void *pMsg)
{
        /* The real callback lives in wireless */
        return;
}

STATIC CpaStatus adf_create_cy_instance(icp_accel_dev_t* dev, int inst_nr,
                                            wireless_cy_instance_t** instance,
                                            char* section_name)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char param_name[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char param_val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U bank_nr = 0, accel_nr = 0;
    wireless_cy_instance_t *inst = NULL;
    Cpa32U num_sym_messages_per_ring = 0;
    Cpa32U num_asym_messages_per_ring = 0;

    ICP_CHECK_FOR_NULL_PARAM(instance);

    inst = ICP_ZALLOC_GEN(sizeof(*inst));
    if(!inst)
    {
        ADF_ERROR("%s: Mem alloc failed for instance %d\n", __FUNCTION__, inst_nr);
        return status;
    }
    *instance = inst;
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_ACCEL_NR_PARAM, inst_nr);
    status = icp_adf_cfgGetParamValue(dev,
                                   section_name,
                                   param_name,
                                   param_val);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("%s: AcceleratorNumber not configured\n", __FUNCTION__);
        ICP_FREE(inst);
        return status;
    }
    accel_nr = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);

	/* Get bank_nr */
    memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_BANK_NR_PARAM, inst_nr);
    status = icp_adf_cfgGetParamValue(dev,
                                   section_name,
                                   param_name,
                                   param_val);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("%s: BankNumber not configured\n", __FUNCTION__);
        ICP_FREE(inst);
        return status;
    }
    bank_nr = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);

    /* Get the number of concurrent symmetric messages per ring */
    memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_SYM_REQUESTS_PARAM, inst_nr);
    status = icp_adf_cfgGetParamValue(dev,
                                   section_name,
                                   param_name,
                                   param_val);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("%s: NumConcurrentSymRequests not configured\n", __FUNCTION__);
        ICP_FREE(inst);
        return status;
    }
    num_sym_messages_per_ring = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);

    /* Get the number of concurrent asymmetric messages per ring */
    memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_ASYM_REQUESTS_PARAM, inst_nr);
    status = icp_adf_cfgGetParamValue(dev,
                                   section_name,
                                   param_name,
                                   param_val);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("%s: NumConcurrentAsymRequests not configured\n", __FUNCTION__);
        ICP_FREE(inst);
        return status;
    }
    num_asym_messages_per_ring = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_ASYM_TX_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    (icp_adf_ringInfoService_t)NULL,
                                    NULL,
                                    ICP_RESP_TYPE_NONE
                                         | RING_CONFIG_LATE_HEAD_POINTER_MODE,
                                    num_asym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->asym_tx);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        ICP_FREE(inst);
        return status;
    }

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_ASYM_RX_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    (icp_adf_ringInfoService_t)NULL,
                                    adf_dummy_callback,
                                    ADF_RESP_TYPE_USER_POLL,
                                    num_asym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->asym_rx );

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_tx);
        ICP_FREE(inst);
        return status;
    }

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_SYM_TX_H_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    (icp_adf_ringInfoService_t)NULL,
                                    NULL,
                                    ICP_RESP_TYPE_NONE
                                           | RING_CONFIG_LATE_HEAD_POINTER_MODE,
                                    num_sym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->sym_tx_hi);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_tx);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_rx);
        ICP_FREE(inst);
        return status;
    }

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_SYM_RX_H_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    (icp_adf_ringInfoService_t)NULL,
                                    adf_dummy_callback,
                                    ADF_RESP_TYPE_USER_POLL,
                                    num_sym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->sym_rx_hi );

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_tx);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_rx);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_tx_hi);
        ICP_FREE(inst);
        return status;
    }

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_SYM_TX_L_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    (icp_adf_ringInfoService_t)NULL,
                                    NULL,
                                    ICP_RESP_TYPE_NONE
                                           | RING_CONFIG_LATE_HEAD_POINTER_MODE,
                                    num_sym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->sym_tx_lo);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_tx);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_rx);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_tx_hi);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_rx_hi);
        ICP_FREE(inst);
        return status;
    }

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   WIRELESS_INST_SYM_RX_L_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    (icp_adf_ringInfoService_t)NULL,
                                    adf_dummy_callback,
                                    ADF_RESP_TYPE_USER_POLL,
                                    num_sym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->sym_rx_lo );

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_tx);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_rx);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_tx_hi);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_rx_hi);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_tx_lo);
        ICP_FREE(inst);
        return status;
    }

    inst->instance_nr = inst_nr;
    inst->dev = dev;
    return status;
}

STATIC CpaStatus adf_delete_cy_instance( wireless_cy_instance_t** instance )
{
    wireless_cy_instance_t *inst = NULL;
    ICP_CHECK_FOR_NULL_PARAM(instance);
    inst = *instance;
    icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_tx);
    icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_rx);
    icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_tx_hi);
    icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_rx_hi);
    icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_tx_lo);
    icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->sym_rx_lo);
    inst->dev = NULL;
    ICP_FREE(inst);
    *instance = NULL;
    return CPA_STATUS_SUCCESS;
}


CpaStatus adf_wirelessCreateInstance(icp_accel_dev_t* accel_dev )
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    char param_val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U number_wireless_procs = 0, wireless_instances = 0, i = 0, x = 0;
    char* section_name = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    /* Create app instances */
    memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    stat = icp_adf_cfgGetParamValue(accel_dev,
                                     GENERAL_SEC,
                                     NUMBER_OF_WIRELESS_PROCS,
                                     param_val);
    if (CPA_STATUS_SUCCESS != stat)
    {
        number_wireless_procs = 0;
        stat = CPA_STATUS_SUCCESS;
    }
    else
    {
        number_wireless_procs = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);
    }

    if( WIRELESS_MAX_PROCS < number_wireless_procs )
    {
        ADF_ERROR("Only allowed %d NAE instances\n", WIRELESS_MAX_PROCS);
        return CPA_STATUS_FAIL;
    }

    for(x = 0; x < number_wireless_procs; x++)
    {
        section_name = wireless_instance_name[accel_dev->accelId][x];
        snprintf(section_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   "%s%d", WIRELESS_SEC, x);

        /* Create app instances */
        memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
        stat = icp_adf_cfgGetParamValue(accel_dev,
                                     section_name,
                                     WIRELESS_INST_NR_PARAM,
                                     param_val);
        if (CPA_STATUS_SUCCESS != stat)
        {
            ADF_ERROR("%s: Parameter not configured\n", WIRELESS_INST_NR_PARAM);
            return stat;
        }
        else
        {
            wireless_instances = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);
        }

        if( WIRELESS_MAX_INSTANCES < wireless_instances )
        {
            ADF_ERROR("Only allowed %d wireless instances\n", WIRELESS_MAX_INSTANCES);
            return CPA_STATUS_FAIL;
        }

        for(i = 0; (i < wireless_instances) && (CPA_STATUS_SUCCESS == stat) ; i++)
        {
            stat = adf_create_cy_instance(accel_dev, i,
                                          &cy_instances[accel_dev->accelId][i],
                                          section_name );
            if (CPA_STATUS_SUCCESS != stat)
            {
                ADF_ERROR("Create wireless crypto instance failed\n");
                return stat;
            }
        }
    }
    return stat;
}

CpaStatus adf_wirelessDeleteInstance(icp_accel_dev_t* accel_dev )
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    char param_val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U number_wireless_procs = 0, wireless_instances = 0, i = 0, x = 0;
    char* section_name = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    stat = icp_adf_cfgGetParamValue(accel_dev,
                                     GENERAL_SEC,
                                     NUMBER_OF_WIRELESS_PROCS,
                                     param_val);
    if (CPA_STATUS_SUCCESS != stat)
    {
        number_wireless_procs = 0;
        stat = CPA_STATUS_SUCCESS;
    }
    else
    {
        number_wireless_procs = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);
    }

    if( WIRELESS_MAX_PROCS < number_wireless_procs )
    {
        ADF_ERROR("Only allowed %d NAE instances\n", WIRELESS_MAX_PROCS);
        return CPA_STATUS_FAIL;
    }
    for(x = 0; x < number_wireless_procs; x++)
    {
        section_name = wireless_instance_name[accel_dev->accelId][x];

        /* Delete app instances */
        memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
        stat = icp_adf_cfgGetParamValue(accel_dev,
                                     section_name,
                                     WIRELESS_INST_NR_PARAM,
                                     param_val);

        if (CPA_STATUS_SUCCESS != stat)
        {
            ADF_ERROR("%s: Parameter not configured\n", WIRELESS_INST_NR_PARAM);
        }
        else
        {
            wireless_instances = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);
        }

        if( WIRELESS_MAX_INSTANCES < wireless_instances )
        {
            ADF_ERROR("Only allowed %d wireless instances\n", WIRELESS_MAX_INSTANCES);
            return CPA_STATUS_FAIL;
        }

        for(i = 0; (i < wireless_instances) && (CPA_STATUS_SUCCESS == stat) ; i++)
        {
            if(NULL != cy_instances[accel_dev->accelId][i])
            {
                stat = adf_delete_cy_instance(&cy_instances[accel_dev->accelId][i]);
            }
        }
        if (CPA_STATUS_SUCCESS != stat)
        {
            ADF_ERROR("Delete wireless crypto instance failed\n");
            return stat;
        }
    }
    return stat;
}

CpaStatus
adf_wirelessResetRings(icp_accel_dev_t *pAccelDev,
                         icp_et_ring_data_t *ringData)
{
    Cpa32U *csr_base_addr = NULL;
    icp_etr_priv_data_t *pRingPrivData = NULL;
    icp_et_ring_bank_data_t *pBankData = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(ringData);

    csr_base_addr = ringData->ringCSRAddress;
    pRingPrivData = pAccelDev->pCommsHandle;

    /* Reset the rings by writing ringBase */

    ICP_SPINLOCK_LOCK(&ringData->ringLock);
    WRITE_CSR_RING_BASE(ringData->bankNumber,
                         ringData->ringNumber, ringData->ringBase);
    ICP_MEMSET(ringData->ringBaseAddress,
                ADF_RING_PATTERN,
                ringData->sizeInBytes);
    ICP_SPINLOCK_UNLOCK(&ringData->ringLock);

    /* Mark the ring as not "InUse" */
    pBankData = &pRingPrivData->banks[ringData->bankNumber];
    ICP_SPINLOCK_LOCK(&pBankData->bankLock);
    pBankData->ringsInUseMask &= ~(1<<ringData->ringNumber);
    ICP_SPINLOCK_UNLOCK(&pBankData->bankLock);

    return CPA_STATUS_SUCCESS;
}

