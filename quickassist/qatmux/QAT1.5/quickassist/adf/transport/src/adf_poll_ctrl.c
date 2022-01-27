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
 * @file adf_poll_ctrl.c
 *
 * @description
 *      The icp_sal_pollBank, icp_sal_pollAllBanks and icp_adf_pollInstance
 *      functions allow the user to poll the response rings to determine
 *      if a message is ready to be read and if so reads the msg(s).
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_poll.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "icp_platform.h"
#include "adf_platform.h"
#include "adf_poll_tasklets.h"
#include "adf_devmgr.h"

/*
 * Internal function which takes care of scheduling
 * the tasklet for polling and checking that the
 * bank is not already being polled.
 */
CpaStatus adf_pollBank(icp_et_ring_bank_data_t *bank,
                       Cpa32U response_quota,
                       Cpa32U csrVal)
{
    /* Check to see if this bank is already being polled by
     * another core or thread. DecAndTest returns SUCCESS
     * only if bank->pollingTaskletScheduled was previously
     * equal to one and then sets the var to zero. While
     * bank->pollingTaskletScheduled is still zero no other
     * thread will be able to poll. pollingTaskletScheduled is
     * reset to one once the notify function is done.
     */
    if(!osalAtomicDecAndTest(&(bank->pollingTaskletScheduled)))
    {
        return CPA_STATUS_RETRY;
    }
    else
    {
        bank->ringsFullMask = csrVal;
        bank->bankResponseQuota = response_quota;
        /* Schedule a tasklet to poll the bank. */
        /* The tasklet will also check to see if there
         * is data on any of the polling rings before
         * calling the notify function.  */
        adf_schedule_bank_tasklet(bank);
        return CPA_STATUS_SUCCESS;
    }
}

/*
 * This function allows the user to poll the reponse rings of a given
 * bank to determine if any of the rings have messages that need to be
 * read. This method is used as an alternative to the reading messages
 * via the ISR method.
 * N.B. The response_quota is per ring.
 */
CpaStatus icp_sal_pollBank(Cpa32U accelId,
                           Cpa32U bank_number,
                           Cpa32U response_quota)
{
    icp_accel_dev_t *accel_dev    = NULL;
    icp_etr_priv_data_t *privData = NULL;
    icp_et_ring_bank_data_t *bank = NULL;
    Cpa32U *csr_base_addr = NULL;
    Cpa32U csrVal = 0;

    accel_dev = adf_devmgrGetAccelDevByAccelId(accelId);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    privData = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);

    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    ICP_CHECK_PARAM_LT_MAX(bank_number, GET_MAX_BANKS(accel_dev));
    bank = &privData->banks[bank_number];

    /* Check the polling mask for the bank. If the polling mask
     * is zero then this bank contains no polling rings. */
    ICP_CHECK_PARAM_GT_MIN(bank->pollingMask,0);
     /* Read the ring status CSR to see which Rings are empty. */
    csrVal = READ_CSR_E_STAT(bank_number);
    /* Complement to see which rings have data. */
    csrVal = ~csrVal;
    /* Mask with the bank kernel pollingMask. */
    csrVal &= bank->pollingMask ;

    /* If all the polling rings are empty return. */
    if(!csrVal)
    {
        return CPA_STATUS_RETRY;
    }

    /* Poll the bank. */
    return (adf_pollBank(bank, response_quota, csrVal));
}

/*
 * This function allows the user to poll the reponse rings of all banks
 * to determine if any of the rings have messages that need to be
 * read. This method is used as an alternative to the reading messages
 * via the ISR method.
 * N.B. The response_quota is per ring.
 */
CpaStatus icp_sal_pollAllBanks(Cpa32U accelId,
                               Cpa32U response_quota)
{
    icp_accel_dev_t *accel_dev = NULL;
    icp_etr_priv_data_t *privData = NULL;
    icp_et_ring_bank_data_t *bank = NULL;
    CpaStatus status=CPA_STATUS_SUCCESS;
    Cpa32U stat_total=CPA_STATUS_SUCCESS;
    Cpa32U bank_num=0;
    Cpa32U *csr_base_addr = NULL;
    Cpa32U csrVal = 0;

    accel_dev = adf_devmgrGetAccelDevByAccelId(accelId);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    privData = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);

    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    /*
     * Loop over all banks and poll those banks that
     * contain polling rings.
     */
    for(bank_num=0; bank_num < GET_MAX_BANKS(accel_dev); bank_num++)
    {
        /* Select the correct bank handle. */
        bank = &privData->banks[bank_num];
        /* Check the polling mask for the bank. If the polling mask
         * is zero then this bank contains no polling rings. */
        if(bank->pollingMask == 0)
        {
            continue;
        }
         /* Read the ring status CSR to see which Rings are empty. */
        csrVal = READ_CSR_E_STAT(bank_num);
        /* Complement to see which rings have data. */
        csrVal = ~csrVal;
        /* Mask with the bank kernel pollingMask. */
        csrVal &= bank->pollingMask ;
        /* If all the polling rings are empty return. */
        if(!csrVal)
        {
            continue;
        }

        status = adf_pollBank(bank,response_quota, csrVal);
        if(CPA_STATUS_SUCCESS == status)
        {
            stat_total++;
        }
    }
    /* Return SUCCESS if adf_pollBank returned SUCCESS
     * for any bank that was polled. */
    if(stat_total)
    {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_RETRY;
}


/*
 * This function allows the user to poll the response ring. The
 * ring number to be polled is supplied by the user. This ring is
 * only polled if it contains data.
 * This method is used as an alternative to the reading messages
 * via the ISR method.
 * This function will return RETRY if the ring is empty.
 */
CpaStatus icp_adf_pollInstance(icp_comms_trans_handle *trans_hnd,
                               Cpa32U num_transHandles,
                               Cpa32U response_quota)
{
    icp_accel_dev_t *accel_dev = NULL;
    icp_etr_priv_data_t *privData = NULL;
    icp_et_ring_bank_data_t *bank=NULL;
    icp_et_ring_data_t* ring = NULL;
    icp_trans_handle *trans_handle = NULL;
    Cpa32U bankNumber = 0, csrVal=0, i=0;
    Cpa32U ring_number=0, instancepolled=0;
    Cpa32U ring_num_in_bank = 0, ring_id=0;
    Cpa32U *csr_base_addr = NULL;
    Cpa32U numRingsPerBank = 0;

    ICP_CHECK_FOR_NULL_PARAM(trans_hnd);
    trans_handle = (icp_trans_handle *)trans_hnd[i];
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    /* Retrieve the accel_dev from the trans_handle. */
    accel_dev = trans_handle->accel_dev;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    privData = (icp_etr_priv_data_t *) accel_dev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);

    /* Now need to get the ring number to poll. */
    ring_number = trans_handle->handle_id;

    numRingsPerBank = GET_NUM_RINGS_PER_BANK(accel_dev);

    /* The bankNumber is the same for each ring number
     * in this instance so can calculate this now. */
    bankNumber = ring_number/numRingsPerBank;
    bank = &privData->banks[bankNumber];
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;
    /* Read the ring status CSR to determine which rings are empty */
    csrVal = READ_CSR_E_STAT(bankNumber);
    /*Find which Rings have data. */
    csrVal = ~csrVal;
    /* Mask with kernel polling rings. */
    csrVal &= bank->pollingMask;
    if(!csrVal)
    {
        return CPA_STATUS_RETRY;
    }

    for(i = 0; i < num_transHandles; i++)
    {
        trans_handle = (icp_trans_handle *)trans_hnd[i];
        ICP_CHECK_FOR_NULL_PARAM(trans_handle);
        ring_number = trans_handle->handle_id;
        /*Calculate ring number in bank and id in bank. */
        ring_num_in_bank = ring_number % numRingsPerBank;
        ring_id = RING_NUMBER_TO_ID(ring_num_in_bank);

        ring = &bank->rings[ring_num_in_bank];

        /* If the ring in question is empty try the next ring.*/
        if(!(csrVal & ring_id))
        {
            continue;
        }
        /* Check to see if this ring is already being polled by
         * another core or thread. DecAndTest returns SUCCESS
         * only if ring->pollingTaskletScheduled was previously
         * equal to one and then sets the var to zero. While
         * ring->pollingTaskletScheduled is still zero no other
         * thread will be able to poll. pollingTaskletScheduled is
         * reset to one once the notify function is done.
         */
        if(!osalAtomicDecAndTest(&ring->pollingTaskletScheduled))
        {
            continue;
        }
        else
        {
            ring->ringResponseQuota = response_quota;
            /* Schedule tasklet on a per Ring basis.
             * This tasklet will poll this response ring. */
            adf_schedule_ring_tasklet(ring);
            instancepolled = 1;
        }
    }
    /*
     * If any ring within the instance was successfully polled
     * return success.
     */
    if(instancepolled)
    {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_RETRY;
}
