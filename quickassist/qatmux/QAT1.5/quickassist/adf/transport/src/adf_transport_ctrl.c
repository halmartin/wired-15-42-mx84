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
 * @file adf_transport_ctrl.c
 *
 * @description
 *      Transport Controller
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_cfg.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_platform.h"
#include "adf_drv_sriov.h"

STATIC icp_trans_mgr *mgr_list_first = NULL;
STATIC icp_trans_mgr *mgr_list_last = NULL;

STATIC CpaStatus adf_findTransportMgr(icp_transport_type trans_type,
                                      icp_trans_mgr** trans_mgr)
{
    /* Walk the list of Managers to find the right type (if exists) */
    icp_trans_mgr *tmpMgr = mgr_list_first;

    ICP_CHECK_FOR_NULL_PARAM(trans_mgr);

    *trans_mgr = NULL;
    while (tmpMgr != NULL)
    {
        if (tmpMgr->trans_type == trans_type)
        {
            ADF_DEBUG("INFO: Manager found for type %x\n", trans_type);
            *trans_mgr = tmpMgr;
            return CPA_STATUS_SUCCESS;
        }
        tmpMgr = tmpMgr->pNext;
    }

    return CPA_STATUS_FAIL;
}

/*****************************************************************************
 * TRANSPORT MANAGER REG/DEREG FUNCTIONS
 ****************************************************************************/

/* Register the transport manager with the transport controller */
CpaStatus adf_trans_registerTransMgr(icp_trans_mgr* trans_mgr)
{
    icp_trans_mgr *tmpMgr = mgr_list_first;
    ICP_CHECK_FOR_NULL_PARAM(trans_mgr);

    if (tmpMgr == NULL)
    {
        /* No Manager exists yet, so let's just set first here */
        mgr_list_first = trans_mgr;
        mgr_list_last = trans_mgr;
    }
    else
    {
        if (adf_findTransportMgr(trans_mgr->trans_type, &tmpMgr)
                == CPA_STATUS_FAIL)
        {
            /* No Existing Mgr found, lets add it to the end */
            ADF_DEBUG("INFO: Adding Manager for type %x\n",
                                               trans_mgr->trans_type);
            ICP_ADD_ELEMENT_TO_END_OF_LIST(trans_mgr,
                                               mgr_list_last, mgr_list_first);
        }
        else
        {
            /*
             * An existing Manager for this type
             * If it's the same manager, all is ok, otherwise a problem has
             * occurred
             */
            if (tmpMgr != trans_mgr)
            {
                return CPA_STATUS_FAIL;
            }
        }
    }
    return CPA_STATUS_SUCCESS;
}

/* Remove the transport manager from the factory */
CpaStatus adf_trans_deregisterTransMgr(icp_trans_mgr* trans_mgr)
{
    ICP_CHECK_FOR_NULL_PARAM(trans_mgr);
    ICP_REMOVE_ELEMENT_FROM_LIST(trans_mgr, mgr_list_last, mgr_list_first);
    return CPA_STATUS_SUCCESS;
}

/*****************************************************************************
 * TRANSPORT HANDLE FUNCTIONS
 ****************************************************************************/

/*
 * Create a transport handle
 */
CpaStatus icp_adf_transCreateHandle(icp_accel_dev_t *accel_dev,
                                    icp_transport_type trans_type,
                                    const char *section,
                                    const Cpa32U accel_nr,
                                    const Cpa32U bank_nr,
                                    const char *service_name,
                                    icp_adf_ringInfoService_t info,
                                    icp_trans_callback callback,
                                    icp_resp_deliv_method resp,
                                    const Cpa32U size,
                                    icp_comms_trans_handle* trans_handle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_trans_mgr *mgr = NULL;
    icp_trans_handle* trans_hnd = NULL;
    Cpa32U flags = 0;
    Cpa32U late_bit = 0;

    late_bit = resp & RING_CONFIG_LATE_HEAD_POINTER_MODE;
    /*
     * Workaround: Need to get the late mode flag into this function.
     * Pass it in high bit of resp, and mask off before we use it.
     */
    resp &= ~RING_CONFIG_LATE_HEAD_POINTER_MODE;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_PARAM_RANGE(resp, ICP_RESP_TYPE_NONE, ADF_RESP_TYPE_USER_POLL+1);

    if( ICP_RESP_TYPE_NONE != resp && NULL == callback )
    {
        ADF_ERROR("Trying to create response ring without callback\n");
        return CPA_STATUS_FAIL;
    }

    /*
     *  We will only set the RING_CONFIG_LATE_HEAD_POINTER_MODE bit in the
     *  ring configuration register for the wireless rings. If this bit is set,
     *  we will 'or' it into the flags with the response type bit so that when
     *  we build the config value, it will include this bit.
     */
    if (late_bit)
    {
        /* OR the flags with the hig bit for late pointer mode */
        flags |= RING_CONFIG_LATE_HEAD_POINTER_MODE;
    }
    flags |= ( 1 << resp );

    if (adf_findTransportMgr(trans_type, &mgr) == CPA_STATUS_SUCCESS)
    {
        /* Manager found - try to create the handle */
        ADF_DEBUG("INFO: Manager found for type %x, attempting to " \
                  "create handle\n", trans_type);
        status = mgr->create_handle(accel_dev,
                                    section,
                                    accel_nr,
                                    bank_nr,
                                    &trans_hnd,
                                    info,
                                    service_name,
                                    size,
                                    flags);
        *trans_handle = (icp_comms_trans_handle*) trans_hnd;
        if (status == CPA_STATUS_SUCCESS)
        {
            trans_hnd->trans_type = trans_type;
            if (NULL != callback)
            {
                status = trans_hnd->reg_callback(trans_hnd, callback);
            }
            if(CPA_STATUS_SUCCESS == status)
            {
                status = adf_VF2PF_notify(accel_dev,
                                          trans_hnd->trans_data,
                                          info, ICP_ADF_RING_ENABLE,
                                          flags);
                if(CPA_STATUS_SUCCESS != status)
                {
                    ADF_ERROR("Notify PF failed to enable ring\n");
                    mgr->release_handle(*trans_handle);
                }
            }
        }
        return status;
    }
    else
    {
        ADF_ERROR("No Manager found for transport type %x\n", trans_type);
        return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * Get a pointer to a previously created transport handle
 */
CpaStatus icp_adf_transGetHandle(icp_accel_dev_t *accel_dev,
                                    icp_transport_type trans_type,
                                    const char *section,
                                    const Cpa32U accel_nr,
                                    const Cpa32U bank_nr,
                                    const char *service_name,
                                    icp_comms_trans_handle* trans_handle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_trans_mgr *mgr = NULL;
    icp_trans_handle* trans_hnd = NULL;
    Cpa32U bank_num_in_dev = 0;
    Cpa32U ring_num_in_dev = 0;
    Cpa32U ring_nr = 0;
    Cpa32U *csr_base_addr = NULL;
    char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    icp_et_ring_data_t *ring_data = NULL;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    status = icp_adf_cfgGetParamValue(accel_dev,
                                      section, service_name, val);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("No such entry in the configuration file: %s\n",
                                                          service_name);
        return CPA_STATUS_FAIL;
    }

    if (NULL == accel_dev->pHwDeviceData)
    {
        ADF_ERROR("pHwDeviceData is NULL\n");
        return CPA_STATUS_FAIL;
    }
    ring_nr = (Cpa32U)ICP_STRTOUL(val, NULL, ADF_CFG_BASE_DEC);
    bank_num_in_dev = (accel_nr * GET_NUM_BANKS_PER_ACCEL(accel_dev)) + bank_nr;
    ring_num_in_dev = (bank_num_in_dev * ICP_ETR_MAX_RINGS_PER_BANK) +
                                ring_nr;
    if (adf_findTransportMgr(trans_type, &mgr) == CPA_STATUS_SUCCESS)
    {
        /* Manager found - try to get the handle */
        ADF_DEBUG("INFO: Manager found for type %x, attempting to " \
                  "get handle\n", trans_type);
        status = mgr->find_handle(accel_dev,
                                    ring_num_in_dev,
                                    &trans_hnd);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Could not find trans handle for %x\n", trans_type);
            return CPA_STATUS_FAIL;
        }
        *trans_handle = (icp_comms_trans_handle*) trans_hnd;
        ring_data = (icp_et_ring_data_t *) trans_hnd->trans_data;
        ICP_CHECK_FOR_NULL_PARAM(ring_data);
        csr_base_addr = ring_data->ringCSRAddress;
        /* reset ring base - and heads and tails */
        WRITE_CSR_RING_BASE(bank_num_in_dev, ring_nr, ring_data->ringBase);
    }
    else
    {
        ADF_ERROR("No Manager found for transport type %x\n", trans_type);
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Release a transport handle
 */
CpaStatus icp_adf_transReleaseHandle(icp_comms_trans_handle trans_handle)
{
    icp_trans_handle *trans_hnd = (icp_trans_handle *) trans_handle;
    icp_trans_mgr *mgr = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);

    if(adf_findTransportMgr(trans_hnd->trans_type, &mgr) == CPA_STATUS_SUCCESS)
    {
        icp_et_ring_data_t *ring_data =
                          (icp_et_ring_data_t *) trans_hnd->trans_data;

        /* Manager found - try to release the handle */
        ADF_DEBUG("INFO: Manager found for type %x, attempting to "
                  "release handle %x\n", trans_hnd->trans_type,
                  trans_hnd->handle_id);
        status = adf_VF2PF_notify(trans_hnd->accel_dev,
                                  trans_hnd->trans_data,
                                  ring_data->info, ICP_ADF_RING_DISABLE,
                                  ring_data->flags);
        if(CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Notify PF failed to disable ring\n");
        }
        return mgr->release_handle(trans_hnd);
    }
    else
    {
        ADF_ERROR("No Manager found for transport type %x\n",
                   trans_hnd->trans_type);
        return CPA_STATUS_FAIL;
    }
}

/*
 * Put a message on the transport handle
 */
CpaStatus icp_adf_transPutMsg(icp_comms_trans_handle trans_handle,
                              Cpa32U *inBuf,
                              Cpa32U bufLen)
{
    icp_trans_handle *trans_hnd = (icp_trans_handle *) trans_handle;
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    return trans_hnd->put_msgs(trans_hnd, inBuf, bufLen);
}

/*
 * Returns ring number for the trans handle
 */
CpaStatus icp_adf_transGetRingNum(icp_comms_trans_handle trans_handle,
                                  Cpa32U *ringNum)
{
    icp_trans_handle *trans_hnd = (icp_trans_handle *) trans_handle;
    icp_trans_mgr *mgr = NULL;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);

    if (adf_findTransportMgr(trans_hnd->trans_type, &mgr) == CPA_STATUS_SUCCESS)
    {
        /* Manager found - try to release the handle */
        ADF_DEBUG("INFO: Manager found for type %x, attempting to " \
                "get ring number for ring Id %d\n", trans_hnd->trans_type,
                trans_hnd->handle_id);
        return mgr->get_ring_num(trans_hnd,ringNum);
    }
    else
    {
        ADF_ERROR("No Manager found for transport type %x\n",
                trans_hnd->trans_type);
        return CPA_STATUS_FAIL;
    }
}
