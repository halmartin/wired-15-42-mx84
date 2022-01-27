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
 * @file adf_user_transport_dyn_pool.c
 *
 * @description
 *      Transport Dynamic Instance Pool for user space
 *
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <sched.h>

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_platform.h"
#include "adf_dev_ring_ctl.h"
#include "adf_user_transport.h"
#include "icp_adf_init.h"
#include "adf_devmgr.h"
#include "adf_cfg.h"

CpaStatus icp_adf_getDynInstance_byAccel(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U *pinstance_id, Cpa32U accelID)
{
    int ret = 0;
    adf_instance_assignment_t assign;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pinstance_id);
    assign.val = ADF_CFG_NO_INSTANCE;
    assign.accelID = accelID;
    *pinstance_id = assign.val;
    if (stype >= ADF_SERVICE_MAX) {
         return CPA_STATUS_FAIL;
    }
    assign.stype = stype;
    ret = ioctl(accel_dev->ringFileHdl,
                ADF_DEV_RING_GET_DYNINSTANCE, &assign);
    if (ret != 0) {
        return CPA_STATUS_FAIL;
    }
    *pinstance_id = assign.val;

    return CPA_STATUS_SUCCESS;
}

CpaStatus icp_adf_getDynInstance(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U *pinstance_id)
{
    return icp_adf_getDynInstance_byAccel(accel_dev, stype, pinstance_id,
                                        ACCEL_ANY);
}

CpaStatus icp_adf_putDynInstance(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U instance_id)
{
    int ret = SUCCESS;
    adf_instance_assignment_t   assign;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    if (stype >= ADF_SERVICE_MAX) {
         ADF_ERROR("Service not supported\n");
         return CPA_STATUS_FAIL;
    }
    if (instance_id == ADF_CFG_NO_INSTANCE) {
         ADF_ERROR("Invalid Instance Id\n");
         return CPA_STATUS_FAIL;
    }
    assign.stype = stype;
    assign.val = instance_id;

    ret = ioctl(accel_dev->ringFileHdl,
                ADF_DEV_RING_PUT_DYNINSTANCE, &assign);
    if (ret != 0) {
        ADF_ERROR("ioctl failed to put dyn instance, ret=%d\n", ret);
        return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}

CpaStatus icp_adf_getNumAvailDynInstance_byAccel(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype, Cpa32U accelID,
                                Cpa32U *num)
{
    int ret = SUCCESS;
    adf_instance_assignment_t   assign;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(num);

    assign.val = 0;
    *num = assign.val;
    if (stype >= ADF_SERVICE_MAX) {
         ADF_ERROR("Service not supported\n");
         return CPA_STATUS_FAIL;
    }
    assign.stype = stype;
    assign.accelID = accelID;
    ret = ioctl(accel_dev->ringFileHdl,
                ADF_DEV_RING_GETNUM_AVAIL_DYNINSTANCE, &assign);
    if (ret != 0) {
        return CPA_STATUS_FAIL;
    }
    *num = assign.val;

    return CPA_STATUS_SUCCESS;
}

CpaStatus icp_adf_getNumAvailDynInstance(icp_accel_dev_t *accel_dev,
                                adf_service_type_t stype,
                                Cpa32U *num)
{
    return icp_adf_getNumAvailDynInstance_byAccel(accel_dev, stype,
                                                  ACCEL_ANY, num);
}
