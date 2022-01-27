/*****************************************************************************
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

/*****************************************************************************
 * @file adf_transport_dyn_pool.c
 *
 * @description
 *      Transport Dynamic Instance Pool
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_cfg.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_dyn.h"

CpaStatus adf_trans_destroyDynInstancePool(icp_accel_dev_t *accel_dev)
{
    Cpa32U     i = 0;
    adf_instancemgr_t  *inst_info = NULL;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    if (NULL == accel_dev->pInstMgr) {
        /* dynamic instance pool is already free */
        return(CPA_STATUS_SUCCESS);
    }

    for(i=0; i<ADF_SERVICE_MAX; i++) {
        inst_info = ((adf_instancemgr_t *)accel_dev->pInstMgr) + i;
        if (inst_info->instances) {
            ICP_FREE(inst_info->instances);
        }
        if (inst_info->instance_lock) {
            ICP_MUTEX_UNINIT(&(inst_info->instance_lock));
        }
        ICP_MEMSET(inst_info, 0, sizeof(adf_instancemgr_t));
        inst_info->serv_type = ADF_SERVICE_MAX;
        inst_info->last_found = ADF_CFG_NO_INSTANCE;
    }
    ICP_FREE(accel_dev->pInstMgr);
    return(CPA_STATUS_SUCCESS);
}

CpaStatus adf_trans_initDynInstancePool(icp_accel_dev_t *accel_dev,
                              Cpa32U crypto_num, Cpa32U compress_num)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U     inst_num[ADF_SERVICE_MAX];
    Cpa32U     i = 0;
    Cpa32U     max_instance_num = 0;
    adf_instancemgr_t  *inst_mgr = NULL;
    adf_instancemgr_t  *inst_info = NULL;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    /*
     * Will re-visit it if more services will be used
     */
    inst_num[ADF_SERVICE_CRYPTO] = crypto_num;
    inst_num[ADF_SERVICE_COMPRESS] = compress_num;

    inst_mgr = ICP_ZALLOC_GEN(ADF_SERVICE_MAX * sizeof(adf_instancemgr_t));
    if (NULL == inst_mgr) {
        return CPA_STATUS_FAIL;
    }
    accel_dev->pInstMgr = (void *)inst_mgr;

    for (i=0; i<ADF_SERVICE_MAX; i++) {
        inst_info = inst_mgr+i;

        ICP_MEMSET(inst_info, 0, sizeof(adf_instancemgr_t));
        inst_info->serv_type = (adf_service_type_t)i;
        inst_info->last_found = ADF_CFG_NO_INSTANCE;
        max_instance_num = inst_num[i];
        inst_info->max_instance = max_instance_num;
        inst_info->avail = max_instance_num;
        ICP_MUTEX_INIT(&(inst_info->instance_lock));
        if(0 == max_instance_num) {
            continue;
        }
        inst_info->instances = ICP_ZALLOC_GEN(max_instance_num * sizeof(Cpa32U));
        if (NULL == inst_info->instances) {
            status = CPA_STATUS_FAIL;
            break;
        }
        ICP_MEMSET(inst_info->instances, 0, max_instance_num * sizeof(Cpa32U));

        if (inst_info->serv_type == ADF_SERVICE_CRYPTO) {
            status = adf_dynCyInitInstance(accel_dev, inst_info);
            if (status != CPA_STATUS_SUCCESS)
                break;
        }
    }
    if (status != CPA_STATUS_SUCCESS) {
        adf_trans_destroyDynInstancePool(accel_dev);
    }
    return status;
}

static CpaBoolean inst_is_candidate(Cpa32U inst, Cpa32U accelID)
{
    /* Is this instance used ? */
    if ((inst & INST_MASK) == INST_USED)
        return CPA_FALSE;

    /* This is the legacy way */
    if (accelID == ACCEL_ANY)
        return CPA_TRUE;

    return ((inst >> INST_ACCEL_SHIFT) == accelID) ? CPA_TRUE : CPA_FALSE;
}

CpaStatus adf_trans_getDynInstance(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U *pinstance_id, Cpa32U accelID)
{
    adf_instancemgr_t *inst_info = NULL;
    adf_service_type_t indx = ADF_SERVICE_MAX;
    Cpa32U found = ADF_CFG_NO_INSTANCE;
    Cpa32U i = 0, search = 0, max_instance = 0;

    *pinstance_id = ADF_CFG_NO_INSTANCE;
    if(NULL == accel_dev->pInstMgr)
    {
        return CPA_STATUS_FAIL;
    }
    else
    {

        indx = stype;
        if (indx >= ADF_SERVICE_MAX) {
            return CPA_STATUS_FAIL;
        }
        inst_info = ((adf_instancemgr_t *)accel_dev->pInstMgr) + indx;

        ICP_MUTEX_LOCK(&inst_info->instance_lock);
        max_instance = inst_info->max_instance;
        if (inst_info->avail <= 0 ) {
             /* No avail data */
            ICP_MUTEX_UNLOCK(&inst_info->instance_lock);
            return CPA_STATUS_FAIL;
        }

        search = (inst_info->last_found + 1 ) % max_instance;
        for (i=0; i<max_instance; i++) {
             if (inst_is_candidate(inst_info->instances[search], accelID)) {
                  found = search;
                  inst_info->last_found = search;
                  inst_info->instances[search] |= INST_USED;
                  inst_info->avail --;
                  break;
             }
             search = (search + 1) % max_instance;
        }
        ICP_MUTEX_UNLOCK(&inst_info->instance_lock);

        *pinstance_id = found;
    }
    return CPA_STATUS_SUCCESS;
}

CpaStatus icp_adf_putDynInstance(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U instance_id)
{
    adf_service_type_t indx = ADF_SERVICE_MAX;
    adf_instancemgr_t *inst_info = NULL;
    if(NULL == accel_dev->pInstMgr)
    {
        return CPA_STATUS_FAIL;
    }
    else
    {
        indx = stype;
        if (indx >= ADF_SERVICE_MAX) {
            return CPA_STATUS_FAIL;
        }
        inst_info = ((adf_instancemgr_t *)accel_dev->pInstMgr) + indx;
        if (instance_id >= inst_info->max_instance) {
            return CPA_STATUS_FAIL;
        }

        ICP_MUTEX_LOCK(&inst_info->instance_lock);
        inst_info->instances[instance_id] &= ~INST_USED;
        inst_info->avail++;

        ICP_MUTEX_UNLOCK(&inst_info->instance_lock);
    }
    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_trans_getNumAvailDynInstance(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype, Cpa32U accelID,
                                Cpa32U *num)
{
    adf_instancemgr_t *inst_info = NULL;
    adf_service_type_t indx = ADF_SERVICE_MAX;
    int candidate;
    int i;

    if(NULL == accel_dev->pInstMgr)
    {
        *num = 0;
    }
    else
    {
        indx = stype;
        if (indx >= ADF_SERVICE_MAX) {
            *num = 0;
            return CPA_STATUS_FAIL;
        }
        inst_info = ((adf_instancemgr_t *)accel_dev->pInstMgr) + indx;

        ICP_MUTEX_LOCK(&inst_info->instance_lock);
        *num = inst_info->avail;

        if (stype == ADF_SERVICE_CRYPTO && accelID != ACCEL_ANY) {
            for (i = 0, candidate = 0; i < inst_info->max_instance; i++) {
                 if (inst_is_candidate(inst_info->instances[i], accelID))
                     candidate++;
            }
            *num = candidate;
        }
        ICP_MUTEX_UNLOCK(&inst_info->instance_lock);
    }
    return CPA_STATUS_SUCCESS;
}
