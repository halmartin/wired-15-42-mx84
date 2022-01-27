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
 * @file adf_ETring_ap.c
 *
 * @description
 *      File containing implementation of ETRing Autopush feature.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_adf_init.h"
#include "icp_accel_devices.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_ETring_ap.h"
#include "adf_platform.h"
#include "icp_adf_accel_mgr.h"

#define NF_MASK 0xAAAAAAAA
#define NE_MASK 0x55555555

/*
 * Sets/Unsets the specified ring in the autopush Mask
 */
STATIC
void adf_etrApSetRingMask(Cpa32U *pApMask,
                        Cpa32U ringNumInDev,
                        CpaBoolean setBit)
{
    if (NULL == pApMask)
    {
        return;
    }
    if (CPA_TRUE == setBit)
    {
        *pApMask |= (1 << RING_NUMBER_IN_AP_BANK(ringNumInDev));
    }
    else
    {
        *pApMask &= ~(1 << RING_NUMBER_IN_AP_BANK(ringNumInDev));
    }
}

/*
 * Sets/Unsets the specified ring's autopush destination
 */
STATIC
CpaStatus adf_etrApSetRingDest(icp_accel_dev_t *accel_dev,
                        Cpa32U *pApDest,
                        Cpa32U ringNumInDev,
                        CpaBoolean setDest)
{
    Cpa8U *pDestArray = NULL;
    Cpa8U mailboxNum = 0;
    Cpa8U aeId = 0;
    Cpa8U aeCtr = 0;
    Cpa8U accelId = 0;
    Cpa8U accelCtr = 0;
    Cpa8U ringAccelId = 0;
    Cpa32U *accelAeMaskArr = NULL;
    Cpa32U accelAeMask = 0;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U devAeMask = 0;
    Cpa32U devAccelMask = 0;
    Cpa32U maxAccelEngines = 0;
    Cpa32U maxAccelerators = 0;
    Cpa32U index = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);
    ICP_CHECK_FOR_NULL_PARAM(pApDest);

    ADF_DEBUG("*pApDest=0x%X\n", *pApDest);

    devAeMask = accel_dev->aeMask;
    devAccelMask = accel_dev->accelMask;

    if (0 == devAeMask || 0 == devAccelMask)
    {
        ADF_ERROR("Invalid AE or Accelerator Mask\n", devAeMask, devAccelMask);
        return CPA_STATUS_FAIL;
    }

    hw_data = accel_dev->pHwDeviceData;
    maxAccelerators = GET_MAX_ACCEL(accel_dev);
    maxAccelEngines = GET_MAX_ACCELENGINES(accel_dev);
    accelAeMaskArr = ICP_MALLOC_ATOMIC(
            sizeof(Cpa32U *) * maxAccelerators);
    if (NULL == accelAeMaskArr){
        ADF_ERROR("Failed to allocate memory.\n");
        return CPA_STATUS_FAIL;
    }
    ICP_MEMSET(accelAeMaskArr, '\0', sizeof(Cpa32U *) * maxAccelerators);

    for (index = 0; index < maxAccelerators; ++index){
        accelAeMaskArr[index] = 0;
    }

    pDestArray = (Cpa8U *)pApDest;

    /* Get mailbox number, range 0-3 */
    mailboxNum = RING_AP_MAILBOX_NUMBER(ringNumInDev);
    ADF_DEBUG("mailboxNum=%d\n", mailboxNum);

    /* Get the accelerator AE mask */
    hw_data->getAccelMaskList(hw_data,
                devAccelMask, devAeMask, accelAeMaskArr);

    /* Get the accelerator */
    ringAccelId = RING_AP_ACCEL_NUMBER(ringNumInDev);
    for (accelCtr = 0; accelCtr < maxAccelerators; accelCtr++)
    {
        if (accelAeMaskArr[accelCtr])
        {
            if (ringAccelId == accelId)
            {
                /* Found an accelerator */
                accelAeMask = accelAeMaskArr[accelCtr];
                break;
            }
            accelId++;
        }
    }
    ADF_DEBUG("accelId=%d, accelAeMask=0x%X\n", accelId, accelAeMask);

    aeCtr = 0;
    for (aeId = 0; aeId < maxAccelEngines; aeId++)
    {
        /* Check if AE is enabled */
        if ( !(accelAeMask & (1 << (aeId))) )
        {
            ADF_DEBUG("AE %d is disabled\n", aeId);
            continue;
        }
        if (CPA_TRUE == setDest)
        {
            pDestArray[aeCtr] = (aeId) << AP_DEST_AE_OFFSET;
            pDestArray[aeCtr] |= mailboxNum;
            pDestArray[aeCtr] |= (1 << AP_DEST_ENABLE_OFFSET);
        }
        else
        {
            pDestArray[aeCtr] = 0;
        }
        ADF_DEBUG("pDestArray[%d]=0x%X\n", aeCtr, pDestArray[aeCtr]);
        aeCtr++;
        if (MAX_AE_PER_MAILBOX == aeCtr)
        {
            break;
        }
    }
    ICP_FREE(accelAeMaskArr);
    return CPA_STATUS_SUCCESS;
}

/*
 * Setup the ring's the autopush mask and destination
 */
CpaStatus adf_etrApSetRingRegister(icp_accel_dev_t *pAccelDev,
                                    Cpa32U ringNumInDev,
                                    Cpa32U flags)
{
    icp_etr_priv_data_t *pEtrPrivData = NULL;
    icp_et_ring_ap_bank_data_t *pApBanks = NULL;
    Cpa32U* csr_base_addr = NULL;
    Cpa32U  apBankNum = 0;
    Cpa32U  nfMaskOff = 0, nfDestOff = 0, neMaskOff = 0, neDestOff = 0;
    CpaStatus status = CPA_STATUS_FAIL;
    adf_hw_device_data_t  *hw_data = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pCommsHandle);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pHwDeviceData);

    hw_data = pAccelDev->pHwDeviceData;;
    if (CPA_FALSE == hw_data->isAutoPushSupported)
    {
        ADF_DEBUG("AutoPush Feature is not supported for this device.\n");
        return CPA_STATUS_SUCCESS;
    }

    pEtrPrivData = (icp_etr_priv_data_t *)pAccelDev->pCommsHandle;
    csr_base_addr = (Cpa32U*)(UARCH_INT)pEtrPrivData->csrBaseAddress;

    /* Get AutoPush bank number, range: 0-7 */
    apBankNum = RING_AP_BANK_NUMBER(ringNumInDev);
    pApBanks = &pEtrPrivData->apBanks[apBankNum];
    ADF_DEBUG("apBankNum=%d\n", apBankNum);
    hw_data->getApCsrOffsets(apBankNum, &nfMaskOff, &nfDestOff,
                                &neMaskOff, &neDestOff, NULL);

    /*
     * Set AutoPush mask and destination based on ring type
     * i.e. set NEMask for request ring, NFMask for response ring
     */
    if (flags & (1 << ICP_RESP_TYPE_NONE))
    {
        adf_etrApSetRingMask(&pApBanks->apNEMask, ringNumInDev, CPA_TRUE);
        if (!pApBanks->apNEDest)
        {
            status = adf_etrApSetRingDest(pAccelDev, &pApBanks->apNEDest,
                    ringNumInDev, CPA_TRUE);
            if (CPA_STATUS_FAIL == status){
                return CPA_STATUS_FAIL;
            }
            ICP_ADF_CSR_WR(csr_base_addr, neDestOff, pApBanks->apNEDest);
        }
        ADF_DEBUG("Dest=0x%X\n", pApBanks->apNEDest);
    }
    else
    {
        if (DEV_DH89XXCC == hw_data->dev_class->type)
        {
            if(pAccelDev->pciAccelDev.revisionId == 0)
            {
                return CPA_STATUS_SUCCESS;
            }
        }

        adf_etrApSetRingMask(&pApBanks->apNFMask, ringNumInDev, CPA_TRUE);
        if (!pApBanks->apNFDest)
        {
            status = adf_etrApSetRingDest(pAccelDev, &pApBanks->apNFDest,
                    ringNumInDev, CPA_TRUE);
            if (CPA_STATUS_FAIL == status){
                return CPA_STATUS_FAIL;
            }
            ICP_ADF_CSR_WR(csr_base_addr, nfDestOff, pApBanks->apNFDest);
        }
        ADF_DEBUG("Dest=0x%X\n", pApBanks->apNFDest);
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Remove the ring from the autopush mask and destination
 */
CpaStatus adf_etrApClrRingRegister(icp_accel_dev_t *pAccelDev,
                                    Cpa32U ringNumInDev,
                                    Cpa32U flags)
{
    icp_etr_priv_data_t *pEtrPrivData = NULL;
    icp_et_ring_ap_bank_data_t *pApBanks = NULL;
    Cpa32U* csr_base_addr = NULL;
    Cpa32U  apBankNum = 0;
    Cpa32U  nfMaskOff = 0, nfDestOff = 0, neMaskOff = 0, neDestOff = 0;
    CpaStatus status = CPA_STATUS_FAIL;
    adf_hw_device_data_t  *hw_data = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pCommsHandle);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pHwDeviceData);

    ADF_DEBUG("ringNumInDev=%d, flags=0x%x\n", ringNumInDev, flags);

    hw_data = pAccelDev->pHwDeviceData;;
    if (CPA_FALSE == hw_data->isAutoPushSupported)
    {
        ADF_DEBUG("AutoPush Feature is not supported for this device.\n");
        return CPA_STATUS_SUCCESS;
    }

    pEtrPrivData = (icp_etr_priv_data_t *)pAccelDev->pCommsHandle;
    csr_base_addr = (Cpa32U*)(UARCH_INT)pEtrPrivData->csrBaseAddress;
    /* Get AutoPush bank number, range: 0-7 */
    apBankNum = RING_AP_BANK_NUMBER(ringNumInDev);
    pApBanks = &pEtrPrivData->apBanks[apBankNum];
    ADF_DEBUG("apBankNum=%d\n", apBankNum);

    hw_data->getApCsrOffsets(apBankNum, &nfMaskOff, &nfDestOff,
                                &neMaskOff, &neDestOff, NULL);
    /*
     * Clear AutoPush mask and destination based on ring type
     * i.e. clear NEMask for request ring, NFMask for response ring
     */
    if (flags & (1 << ICP_RESP_TYPE_NONE))
    {
        adf_etrApSetRingMask(&pApBanks->apNEMask, ringNumInDev, CPA_FALSE);
        if (!pApBanks->apNEMask) {
            status = adf_etrApSetRingDest(pAccelDev, &pApBanks->apNEDest,
                    ringNumInDev, CPA_FALSE);
            if (CPA_STATUS_FAIL == status){
                return CPA_STATUS_FAIL;
            }
            ICP_ADF_CSR_WR(csr_base_addr, neDestOff, pApBanks->apNEDest);
        }
        ADF_DEBUG("Dest=0x%X\n", pApBanks->apNEDest);
    }
    else
    {
        if (DEV_DH89XXCC == hw_data->dev_class->type)
        {
            if(pAccelDev->pciAccelDev.revisionId == 0)
            {
                return CPA_STATUS_SUCCESS;
            }
        }

        adf_etrApSetRingMask(&pApBanks->apNFMask, ringNumInDev, CPA_FALSE);
        if (!pApBanks->apNFMask) {
            status = adf_etrApSetRingDest(pAccelDev, &pApBanks->apNFDest,
                    ringNumInDev, CPA_FALSE);
            if (CPA_STATUS_FAIL == status){
                return CPA_STATUS_FAIL;
            }
            ICP_ADF_CSR_WR(csr_base_addr, nfDestOff, pApBanks->apNFDest);
        }
        ADF_DEBUG("Dest=0x%X\n", pApBanks->apNFDest);
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Clears the autopush bank settings
 */
CpaStatus adf_etrApClrBanks(icp_accel_dev_t *pAccelDev, CpaBoolean set_autopush)
{
    icp_etr_priv_data_t *pEtrPrivData = NULL;
    icp_et_ring_ap_bank_data_t *pApBank = NULL;
    Cpa32U* csr_base_addr = NULL;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U  apBankNum = 0;
    Cpa32U  maxApBanks = 0;
    Cpa32U  nfMaskOff = 0, nfDestOff = 0, neMaskOff = 0, neDestOff = 0;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pCommsHandle);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev->pHwDeviceData);

    hw_data = pAccelDev->pHwDeviceData;
    if (CPA_FALSE == hw_data->isAutoPushSupported)
    {
        ADF_DEBUG("AutoPush Feature is not supported for this device.\n");
        return CPA_STATUS_SUCCESS;
    }

    if(CPA_TRUE != set_autopush && icp_adf_is_dev_in_reset(pAccelDev))
    {
        /* Device is reset. Skip the MMIO writes */
        return CPA_STATUS_SUCCESS;
    }

    maxApBanks = hw_data->maxNumApBanks;
    pEtrPrivData = (icp_etr_priv_data_t *)pAccelDev->pCommsHandle;
    csr_base_addr = (Cpa32U*)(UARCH_INT)pEtrPrivData->csrBaseAddress;
    pEtrPrivData->apDelay = 0;

    for (apBankNum = 0; apBankNum < maxApBanks; apBankNum++)
    {
        pApBank = &pEtrPrivData->apBanks[apBankNum];
        ICP_MEMSET(pApBank, 0, sizeof(icp_et_ring_ap_bank_data_t));

        /*
         * Clear AutoPush mask and destination based on ring type
         * i.e. clear NEMask for request ring, NFMask for response ring
         */
        hw_data->getApCsrOffsets(apBankNum, &nfMaskOff, &nfDestOff,
                            &neMaskOff, &neDestOff, NULL);

        if (CPA_TRUE == set_autopush)
        {
            ICP_ADF_CSR_WR(csr_base_addr, nfMaskOff, NF_MASK);
            ICP_ADF_CSR_WR(csr_base_addr, nfDestOff, 0);
            ICP_ADF_CSR_WR(csr_base_addr, neMaskOff, NE_MASK);
            ICP_ADF_CSR_WR(csr_base_addr, neDestOff, 0);
        }
        else
        {
            ICP_ADF_CSR_WR(csr_base_addr, nfMaskOff, 0);
            ICP_ADF_CSR_WR(csr_base_addr, nfDestOff, 0);
            ICP_ADF_CSR_WR(csr_base_addr, neMaskOff, 0);
            ICP_ADF_CSR_WR(csr_base_addr, neDestOff, 0);
        }
    }
    return CPA_STATUS_SUCCESS;
}
