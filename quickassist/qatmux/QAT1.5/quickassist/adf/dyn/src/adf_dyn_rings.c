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
 * @file  adf_dyn_rings.c
 *
 * @description
 *        This file contains the function for ring managnent in dyn
 *        driver mode.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_accel_mgr.h"
#include "adf_transport_ctrl.h"
#include "adf_cfg.h"
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"
#include "adf_platform.h"
#include "adf_init.h"
#include "adf_dyn.h"

#define INFLIGHT_COMPLETION_MS 10

STATIC dyn_cy_instance_t *cy_instance_table[ADF_MAX_DEVICES];
STATIC dyn_cy_instance_t *cy_instance_table_head[ADF_MAX_DEVICES];
STATIC dyn_dc_instance_t *dc_instance_table[ADF_MAX_DEVICES];
STATIC dyn_dc_instance_t *dc_instance_table_head[ADF_MAX_DEVICES];

/*
 * Dummy kernelspace callback for dyn rings
 */
void adf_dyn_dummy_callback(icp_comms_trans_handle comms_handle, void *pMsg)
{
        /* The real callback lives in dyn */
        return;
}

STATIC CpaStatus adf_create_cy_instance(icp_accel_dev_t* dev, int inst_nr,
                                            dyn_cy_instance_t** instance,
                                            char* section_name)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char param_name[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char param_val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U bank_nr = 0, accel_nr = 0;
    dyn_cy_instance_t *inst = NULL;
    Cpa32U num_sym_messages_per_ring = 0;
    Cpa32U num_asym_messages_per_ring = 0;
    icp_resp_deliv_method rx_resp_type = ICP_RESP_TYPE_IRQ;
    Cpa8U is_polled = 0;
    Cpa8U execution_engine = 0;
    Cpa32U sym_hi_ring_type = 0, sym_lo_ring_type = 0,
           asym_ring_type = 0;

    ICP_CHECK_FOR_NULL_PARAM(instance);

    inst = ICP_ZALLOC_GEN(sizeof(*inst));
    if(!inst)
    {
        ADF_ERROR("%s: Mem alloc failed for instance %d\n", __FUNCTION__,
                       inst_nr);
        return status;
    }
    *instance = inst;

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_CY_INST_EXEC_ENG_PARAM, inst_nr);
    status = icp_adf_cfgGetParamValue(dev,
                                   section_name,
                                   param_name,
                                   param_val);
    execution_engine = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);

    /*
     * ADF to SAL service mapping can be found in lac_common.h
     */
    if(execution_engine)
    {
        sym_hi_ring_type = ICP_ADF_RING_SERVICE_4;
        sym_lo_ring_type = ICP_ADF_RING_SERVICE_5;
        asym_ring_type = ICP_ADF_RING_SERVICE_6;
    }
    else
    {
        sym_hi_ring_type = ICP_ADF_RING_SERVICE_1;
        sym_lo_ring_type = ICP_ADF_RING_SERVICE_2;
        asym_ring_type = ICP_ADF_RING_SERVICE_3;
    }

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_CY_INST_IS_POLLED_PARAM, inst_nr);
    status = icp_adf_cfgGetParamValue(dev,
                                   section_name,
                                   param_name,
                                   param_val);
    is_polled = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);
    if(1 == is_polled)
    {
         rx_resp_type = ADF_RESP_TYPE_USER_POLL;
    }

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("%s: AcceleratorNumber not configured\n", __FUNCTION__);
        ICP_FREE(inst);
        return status;
    }

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_CY_INST_ACCEL_NR_PARAM, inst_nr);
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
                                   DYN_CY_INST_BANK_NR_PARAM, inst_nr);
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
                                   DYN_CY_INST_SYM_REQUESTS_PARAM, inst_nr);
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
                                   DYN_CY_INST_ASYM_REQUESTS_PARAM, inst_nr);
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
                                   DYN_CY_INST_ASYM_TX_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    asym_ring_type,
                                    NULL,
                                    ICP_RESP_TYPE_NONE,
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
                                   DYN_CY_INST_ASYM_RX_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    ICP_ADF_RING_SERVICE_0,
                                    adf_dyn_dummy_callback,
                                    rx_resp_type,
                                    num_asym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->asym_rx);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->asym_tx);
        ICP_FREE(inst);
        return status;
    }

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_CY_INST_SYM_TX_H_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    sym_hi_ring_type,
                                    NULL,
                                    ICP_RESP_TYPE_NONE,
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
                                   DYN_CY_INST_SYM_RX_H_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    ICP_ADF_RING_SERVICE_0,
                                    adf_dyn_dummy_callback,
                                    rx_resp_type,
                                    num_sym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->sym_rx_hi);

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
                                   DYN_CY_INST_SYM_TX_L_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    sym_lo_ring_type,
                                    NULL,
                                    ICP_RESP_TYPE_NONE,
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
                                   DYN_CY_INST_SYM_RX_L_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    ICP_ADF_RING_SERVICE_0,
                                    adf_dyn_dummy_callback,
                                    rx_resp_type,
                                    num_sym_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->sym_rx_lo);

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

STATIC CpaStatus adf_delete_cy_instance(dyn_cy_instance_t** instance)
{
    dyn_cy_instance_t *inst = NULL;
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

STATIC CpaStatus adf_create_dc_instance(icp_accel_dev_t* dev, int inst_nr,
                                            dyn_dc_instance_t** instance,
                                            char* section_name)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char param_name[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char param_val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U bank_nr = 0, accel_nr = 0;
    dyn_dc_instance_t *inst = NULL;
    Cpa32U num_messages_per_ring = 0;
    icp_resp_deliv_method rx_resp_type = ICP_RESP_TYPE_IRQ;
    Cpa8U is_polled;

    ICP_CHECK_FOR_NULL_PARAM(instance);

    inst = ICP_ZALLOC_GEN(sizeof(*inst));
    if(!inst)
    {
        ADF_ERROR("%s: Mem alloc failed for instance %d\n", __FUNCTION__, inst_nr);
        return status;
    }
    *instance = inst;

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_DC_INST_IS_POLLED_PARAM, inst_nr);
    status = icp_adf_cfgGetParamValue(dev,
                                   section_name,
                                   param_name,
                                   param_val);
    is_polled = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);
    if(1 == is_polled)
    {
         rx_resp_type = ADF_RESP_TYPE_USER_POLL;
    }

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("%s: AcceleratorNumber not configured\n", __FUNCTION__);
        ICP_FREE(inst);
        return status;
    }

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_DC_INST_ACCEL_NR_PARAM, inst_nr);
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
                                   DYN_DC_INST_BANK_NR_PARAM, inst_nr);
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

    /* Get the number of concurrent messages per ring */
    memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_DC_INST_REQUESTS_PARAM, inst_nr);
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
    num_messages_per_ring = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_DC_INST_RING_TX_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    ICP_ADF_RING_SERVICE_7,
                                    NULL,
                                    ICP_RESP_TYPE_NONE,
                                    num_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->dc_tx);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        ICP_FREE(inst);
        return status;
    }

    memset(param_name, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                                   DYN_DC_INST_RING_RX_PARAM, inst_nr);

    status = icp_adf_transCreateHandle(dev,
                                    ICP_TRANS_TYPE_ETR,
                                    section_name,
                                    accel_nr,
                                    bank_nr,
                                    param_name,
                                    ICP_ADF_RING_SERVICE_0,
                                    adf_dyn_dummy_callback,
                                    rx_resp_type,
                                    num_messages_per_ring,
                                    (icp_comms_trans_handle *)&inst->dc_rx);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Create Trans Handle %s failed\n", param_name);
        icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->dc_tx);
        ICP_FREE(inst);
        return status;
    }

    inst->instance_nr = inst_nr;
    inst->dev = dev;
    return status;
}

STATIC CpaStatus adf_delete_dc_instance(dyn_dc_instance_t** instance)
{
    dyn_dc_instance_t *inst = NULL;
    ICP_CHECK_FOR_NULL_PARAM(instance);
    inst = *instance;
    icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->dc_tx);
    icp_adf_transReleaseHandle((icp_comms_trans_handle)inst->dc_rx);
    inst->dev = NULL;
    ICP_FREE(inst);
    *instance = NULL;
    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_dynCreateInstance(icp_accel_dev_t* accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    char param_val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    Cpa32U cy_instances = 0, dc_instances = 0, i = 0;
    dyn_cy_instance_t *cy_instance = NULL;
    dyn_dc_instance_t *dc_instance = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    memset(cy_instance_table, 0, sizeof(cy_instance_table));
    memset(cy_instance_table_head, 0, sizeof(cy_instance_table_head));
    memset(dc_instance_table, 0, sizeof(dc_instance_table));
    memset(dc_instance_table_head, 0, sizeof(dc_instance_table_head));

    /* Create cy app instances */
    memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    stat = icp_adf_cfgGetParamValue(accel_dev,
                                 DYN_SEC,
                                 DYN_CY_INST_NR_PARAM,
                                 param_val);
    if (CPA_STATUS_SUCCESS != stat)
    {
        cy_instances = 0;
        stat = CPA_STATUS_SUCCESS;
    }
    else
    {
        cy_instances = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);
    }
    for(i = 0; (i < cy_instances) && (CPA_STATUS_SUCCESS == stat) ; i++)
    {
        stat = adf_create_cy_instance(accel_dev, i,
                                      &cy_instance,
                                      DYN_SEC);
        if (CPA_STATUS_SUCCESS != stat)
        {
            ADF_ERROR("Create dyn crypto instance failed\n");
            return stat;
        }
        ICP_ADD_ELEMENT_TO_END_OF_LIST(cy_instance,
            cy_instance_table[accel_dev->accelId],
            cy_instance_table_head[accel_dev->accelId]);
    }

    /* Create dc app instances */
    memset(param_val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    stat = icp_adf_cfgGetParamValue(accel_dev,
                                 DYN_SEC,
                                 DYN_DC_INST_NR_PARAM,
                                 param_val);
    if (CPA_STATUS_SUCCESS != stat)
    {
        dc_instances = 0;
        stat = CPA_STATUS_SUCCESS;
    }
    else
    {
        dc_instances = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);
    }
    for(i = 0; (i < dc_instances) && (CPA_STATUS_SUCCESS == stat) ; i++)
    {
        stat = adf_create_dc_instance(accel_dev, i,
                                      &dc_instance,
                                      DYN_SEC);
        if (CPA_STATUS_SUCCESS != stat)
        {
            ADF_ERROR("Create dyn compression instance failed\n");
            return stat;
        }
        ICP_ADD_ELEMENT_TO_END_OF_LIST(dc_instance,
            dc_instance_table[accel_dev->accelId],
            dc_instance_table_head[accel_dev->accelId]);
    }
    if (!icp_adf_is_dev_in_reset(accel_dev))
    {
        if((0 != cy_instances) || (0 != dc_instances))
        {
            stat = adf_trans_initDynInstancePool(accel_dev, cy_instances, dc_instances);
            if (CPA_STATUS_SUCCESS != stat)
            {
                ADF_ERROR("Initialize dyn instance pool failed\n");
            }
        }
    }
    return stat;
}

CpaStatus adf_dynDeleteInstance(icp_accel_dev_t* accel_dev)
{
    CpaStatus ret = CPA_STATUS_SUCCESS;
    CpaStatus stat = CPA_STATUS_SUCCESS;
    dyn_cy_instance_t *cy_instance = NULL;
    dyn_dc_instance_t *dc_instance = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    if (!icp_adf_is_dev_in_reset(accel_dev))
    {
        stat = adf_trans_destroyDynInstancePool(accel_dev);
        if (CPA_STATUS_SUCCESS != stat)
        {
            ADF_ERROR("Destroy dyn instance pool failed\n");
        }
    }

    /* Delete app instances */
    while((NULL != cy_instance_table_head[accel_dev->accelId])
          && (CPA_STATUS_SUCCESS == stat))
    {
        cy_instance = cy_instance_table_head[accel_dev->accelId];
        ICP_REMOVE_ELEMENT_FROM_LIST(cy_instance,
               cy_instance_table[accel_dev->accelId],
               cy_instance_table_head[accel_dev->accelId]);
        ret = adf_delete_cy_instance(&cy_instance);
        if(CPA_STATUS_SUCCESS != ret)
        {
            stat = CPA_STATUS_FAIL;
        }
    }
    /* Delete app instances */
    while((NULL != dc_instance_table_head[accel_dev->accelId])
          && (CPA_STATUS_SUCCESS == stat))
    {
        dc_instance = dc_instance_table_head[accel_dev->accelId];
        ICP_REMOVE_ELEMENT_FROM_LIST(dc_instance,
               dc_instance_table[accel_dev->accelId],
               dc_instance_table_head[accel_dev->accelId]);
        ret = adf_delete_dc_instance(&dc_instance);
        if(CPA_STATUS_SUCCESS != ret)
        {
            stat = CPA_STATUS_FAIL;
        }
    }
    if (CPA_STATUS_SUCCESS != stat)
    {
        ADF_ERROR("Delete dyn crypto instance failed\n");
    }
    return stat;
}

CpaStatus
adf_dynResetRings(icp_accel_dev_t *pAccelDev,
                  icp_et_ring_data_t *ringData)
{
    Cpa32U *csr_base_addr = NULL;
    icp_etr_priv_data_t *pRingPrivData = NULL;
    icp_et_ring_bank_data_t *pBankData = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(ringData);

    csr_base_addr = ringData->ringCSRAddress;
    pRingPrivData = pAccelDev->pCommsHandle;

    /* Reset the ring by writing ringBase */
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

CpaStatus adf_dynCyInitInstance(icp_accel_dev_t *dev, adf_instancemgr_t *inst_info)
{
    char param_name[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char param_val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U accel_nr = 0, exec_eng = 0, i = 0;

    /* Init the instances with the Accelerator info, only for crypto now. */
    for (; i < inst_info->max_instance; i++) {
        /* Get the Accelerator number */
        snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                   DYN_CY_INST_ACCEL_NR_PARAM, i);
        status = icp_adf_cfgGetParamValue(dev,
                                   DYN_SEC, param_name, param_val);
        if (CPA_STATUS_SUCCESS != status) {
            ADF_ERROR("%s: AcceleratorNumber not configured\n", param_name);
            return status;
        }
        accel_nr = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);

        /* Get the Engine number */
        snprintf(param_name, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                   DYN_CY_INST_EXEC_ENG_PARAM, i);
        status = icp_adf_cfgGetParamValue(dev,
                                   DYN_SEC , param_name, param_val);
        if (CPA_STATUS_SUCCESS != status) {
            ADF_ERROR("%s: ExecutionEngine not configured\n", param_name);
            return status;
        }
        exec_eng = ICP_STRTOUL(param_val, NULL, ADF_CFG_BASE_DEC);

        if (accel_nr >= ACCEL_NR_MAX) {
            ADF_ERROR("%s: Accelerator Number(%d) is invalid\n", accel_nr);
            return CPA_STATUS_FAIL;
        }

        if (exec_eng >= EXEC_ENGINE_MAX) {
            ADF_ERROR("%s: ExecutionEngine(%d) is invalid\n", exec_eng);
            return CPA_STATUS_FAIL;
        }

        /*
         * combine the accelerator number and execution engine number
         * to an index, and store it at the upper 16 bits of
         * the inst_info->instances.
         */
        inst_info->instances[i] =
                ((accel_nr << ACCEL_NR_SHIFT) + exec_eng) << INST_ACCEL_SHIFT;
    }
    return CPA_STATUS_SUCCESS;
}
