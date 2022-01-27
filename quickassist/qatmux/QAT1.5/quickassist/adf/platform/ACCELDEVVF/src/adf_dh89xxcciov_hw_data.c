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
 * @file adf_dh89xxcciov_hw_data.c
 *
 * @ingroup
 *
 * @description
 *  This file contains the hw device data for DH89xxcc virtual function.
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_platform_common.h"
#include "adf_platform_dh89xxcciov.h"

/******************************************************************************
* External symbols
*******************************************************************************/
extern void adf_bank0_handler(void* handle);
extern void adf_bank0_polling_handler(void* handle);

/*******************************************************************************
* Static Variables
*******************************************************************************/
STATIC adf_hw_device_class_t dh89xxcciov_class = {
    .name               = "dh89xxccvf"    ,
    .type               = DEV_DH89XXCC_VF ,
    .numInstances       = 0               ,
    .currentInstanceId  = 0
};

STATIC bank_handler bh_bank_handlers[] = {
    adf_bank0_handler,
};

STATIC bank_handler bh_polling_bank_handlers[] = {
    adf_bank0_polling_handler,
};

#define NUM_ELEM(array)                   \
    sizeof (array) / sizeof (*array)

/*******************************************************************************
* Functions
*******************************************************************************/

/*
 * getAcceleratorsMask
 * Gets the acceleration mask based on the fuse information
 */
STATIC Cpa32U
getAcceleratorsMask(Cpa32U fuse)
{
    return ICP_DH89xxCCIOV_ACCELERATORS_MASK;
}

/*
 * getAccelEnginesMask
 * Gets the accelerator engine mask based on the fuse information
 */
STATIC Cpa32U
getAccelEnginesMask(Cpa32U fuse)
{
    return ICP_DH89xxCCIOV_ACCELENGINES_MASK;
}

/*
 * getNumAccelerators
 * Function returns the number of accelerators
 *
 */
STATIC Cpa32U
getNumAccelerators(adf_hw_device_data_t *self, Cpa32U mask)
{
    return ICP_DH89xxCCIOV_MAX_ACCELERATORS;
}

/*
 * getNumAccelEngines
 * Function returns the number of accelerator engines
 */
STATIC Cpa32U
getNumAccelEngines(adf_hw_device_data_t *self, Cpa32U mask)
{
    return ICP_DH89xxCCIOV_MAX_ACCELENGINES;
}

/*
 * getAccelMaskList
 * Function returns the accelerator mask for both
 * accelerators in the system
 */
STATIC void
getAccelMaskList(adf_hw_device_data_t *self,
                Cpa32U accelMask, Cpa32U aeMask, Cpa32U *pMaskList)
{
    pMaskList[0] = ICP_DH89xxCCIOV_ACCELENGINES_MASK;
    return;
}

/*
 * getAccFuncMask
 * Gets the accelerator functionality mask based on the accelerator capability
 */
STATIC Cpa32U
getAccFuncMask(Cpa32U func)
{
    Cpa32U mask = 0;

    mask |= ICP_ACCEL_CAPABILITIES_CIPHER;
    mask |= ICP_ACCEL_CAPABILITIES_AUTHENTICATION;
    mask |= ICP_ACCEL_CAPABILITIES_REGEX;
    mask |= ICP_ACCEL_CAPABILITIES_COMPRESSION;
    mask |= ICP_ACCEL_CAPABILITIES_LZS_COMPRESSION;
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC;
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
    mask |= ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER;
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_0;
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_1;
    return mask;
}

/*
 * getMiscBarId
 * Returns the VF-to-PF BAR id of the virtual function
 */
STATIC Cpa32U
getMiscBarId(adf_hw_device_data_t *self)
{
    return ICP_DH89xxCCIOV_VFTOPF_BAR;
}

/*
 * getEtrBarId
 * Returns the ETR BAR id of the device
 */
STATIC Cpa32U
getEtrBarId(adf_hw_device_data_t *self, Cpa32U *offset)
{
    if (offset)
    {
        *offset = ICP_DH89xxCCIOV_ETRING_CSR_OFFSET;
    }
    return ICP_DH89xxCCIOV_PETRINGCSR_BAR;
}

/*
 * isBarAvailable
 * Returns whether the barId is available on the device
 */
STATIC CpaBoolean
isBarAvailable(adf_hw_device_data_t *self, Cpa32U barId)
{
    STATIC CpaBoolean bars[] = {CPA_TRUE, CPA_TRUE};

    return (barId < (sizeof (bars) / sizeof (*bars))) ?
        bars[barId]                                    :
        CPA_FALSE ;
}

/*
 * init
 * Function initialise internal hw data
 */
STATIC CpaStatus
init(adf_hw_device_data_t *self, Cpa8U node_id)
{
    self->instanceId = self->dev_class->currentInstanceId++;
    ++self->dev_class->numInstances;

    return CPA_STATUS_SUCCESS;
}

/*
 * cleanup
 * Function cleanup internal hw data
 */
STATIC void
cleanup(adf_hw_device_data_t *self)
{
    --self->dev_class->numInstances;
    self->instanceId = self->dev_class->currentInstanceId--;

    return;
}

/*
 * getBankBhHandler
 * Function returns a bank handler based on bankId
 */
STATIC bank_handler
getBankBhHandler(Cpa32U bankId)
{
    return (bankId < NUM_ELEM(bh_bank_handlers)) ?
        bh_bank_handlers[bankId]                 :
        NULL;
}

/*
 * getBankBhPollingHandler
 * Function returns a polling bank handler based on bankId
 */
STATIC bank_handler
getBankBhPollingHandler(Cpa32U bankId)
{
    return (bankId < NUM_ELEM(bh_polling_bank_handlers)) ?
        bh_polling_bank_handlers[bankId]                 :
        NULL;
}

/*
 * getDevSKU
 * Function returns device SKU info
 */
STATIC dev_sku_info_t
getDevSKU(Cpa32U NumAccelerators, Cpa32U NumAccelEngines, Cpa32U ClkMask,
          Cpa32U fuse)
{
    return DEV_SKU_VF;
}

/*
 * getPf2VfDbIntOffset
 * This function returns byte offset of the doorbell and interrupt CSR
 * from VF. Id is not used.
*/
STATIC Cpa32U
getPf2VfDbIntOffset(Cpa32U id)
{
    return ICP_DH89xxCCIOV_PF2VFDBINT_OFFSET;
}

/*
 * getVIntSrcOffset
 * This function returns byte offset of the VF interrupt srource CSR.
 */
STATIC Cpa32U
getVIntSrcOffset(Cpa32U id)
{
    return ICP_DH89xxCCIOV_VINTSRC_OFFSET;
}

/*
 * getVIntMskOffset
 * This function returns byte offset of the VF interrupt mask CSR.
 */
STATIC Cpa32U
getVIntMskOffset(Cpa32U id)
{
    return ICP_DH89xxCCIOV_VINTMSK_OFFSET;
}

/*
 * getVIntMskDefault
 * This function returns the default VF interrupt mask.
 */
STATIC Cpa32U
getVIntMskDefault(void)
{
    return ICP_DH89xxCCIOV_VINTMSK_DEFAULT;
}

/*
 * irqGetBunSource
 * Function returns the bundle source based on interrupt source(vintsou)
 */
STATIC Cpa32U
irqGetBunSource(adf_hw_device_data_t *self,
            Cpa32U vintsou)
{
    return irq_bun_source(vintsou);
}

/*
 * irqGetPF2VFSource
 * Function returns the Pf2VF source based on interrupt source(vintsou)
 */
STATIC Cpa32U
irqGetPF2VFSource(adf_hw_device_data_t *self, Cpa32U vintsou)
{
    return irq_pf2vf_source(vintsou);
}

/*
 * adf_set_hw_data_dh89xxcc
 * Initialise the hw data structure
 */
void
adf_set_hw_data_dh89xxcciov(adf_hw_device_data_t *hw_data)
{
    adf_hw_device_data_t *hw = NULL;

    ICP_CHECK_FOR_NULL_PARAM_VOID(hw_data);
    hw = hw_data;
    hw->init    = init;
    hw->cleanup = cleanup;
    hw->dev_class = &dh89xxcciov_class;

    /* populate bank/ac information */
    hw->maxBars               = ICP_DH89xxCCIOV_MAX_PCI_BARS;
    hw->maxNumBanks           = ICP_DH89xxCCIOV_ETR_MAX_BANKS;
    hw->maxNumApBanks         = 0;
    hw->numRingsPerBank       = ICP_DH89xxCCIOV_ETR_MAX_RINGS_PER_BANK;
    hw->numBanksPerAccel      = ICP_DH89xxCCIOV_BANKS_PER_ACCELERATOR;
    hw->maxNumAccel           = ICP_DH89xxCCIOV_MAX_ACCELERATORS;
    hw->maxNumAccelEngines    = ICP_DH89xxCCIOV_MAX_ACCELENGINES;
    hw->aeClockInMhz          = ICP_DH89xxCCIOV_AE_CLOCK_IN_MHZ;
    hw->sintpfOffset          = 0;
    hw->userEtringCsrSize     = ICP_DH89xxCCIOV_USER_ETRING_CSR_SIZE;

    /* populate msix information */
    hw->msix.banksVectorStart = 0;
    hw->msix.banksVectorNum   = 0;
    hw->msix.aeVectorStart    = 0;
    hw->msix.aeVectorNum      = 0;

    hw->isMemScrubSupported     = CPA_FALSE;

    /* device fuse */
    hw->getDevFuseOffset      = NULL;
    hw->getDevClkOffset       = NULL;
    hw->getDevSKU             = getDevSKU;
    hw->getAcceleratorsMask   = getAcceleratorsMask;
    hw->getAccelEnginesMask   = getAccelEnginesMask;
    hw->getNumAccelerators    = getNumAccelerators;
    hw->getNumAccelEngines    = getNumAccelEngines;
    hw->getAccelMaskList      = getAccelMaskList;

    /* accelerator functions */
    hw->getAccFuncOffset      = NULL;
    hw->getAccFuncMask        = getAccFuncMask;

    /* populate bars callbacks  */
    hw->getEtrBarId           = getEtrBarId;
    hw->getMiscBarId          = getMiscBarId;
    hw->getSramBarId          = NULL;
    hw->isBarAvailable        = isBarAvailable;

    /* populate esram callbacks    */
    hw->getEsramInfo          = NULL;

    /* populate SHAC callback */
    hw->getScratchRamInfo     = NULL;

    /* populate PMiscBar callback  */
    hw->getSmiapfOffsetMask   = NULL;
    hw->irqGetBankNumber      = NULL;
    hw->irqGetAeSource        = NULL;
    hw->irqGetBunSource       = irqGetBunSource;
    hw->irqGetPF2VFSource     = irqGetPF2VFSource;
    hw->getBankBhHandler        = getBankBhHandler;
    hw->getBankBhPollingHandler = getBankBhPollingHandler;

    /* Auto-push feature callback */
    hw->isAutoPushSupported     = CPA_FALSE;
    hw->getApCsrOffsets         = NULL;

    /* VF to PF */
    hw->getVf2PfIntSourceOffset = NULL;
    hw->getVf2PfBitOffset       = NULL;
    hw->getVf2PfIntMaskOffset   = NULL;
    hw->getVf2PfIntMaskDefault  = NULL;
    hw->getTiMiscIntCtlOffset   = NULL;
    hw->getVIntSrcOffset        = getVIntSrcOffset;
    hw->getVIntMskOffset        = getVIntMskOffset;
    hw->getVIntMskDefault       = getVIntMskDefault;
    hw->getPf2VfDbIntOffset     = getPf2VfDbIntOffset;

    hw->getResetOffsets         = NULL;

    /* uncorrectable error support functions */
    hw->enableUncoErrInterrupts = NULL;
    hw->handleUncoErrInterrupts = NULL;
}
