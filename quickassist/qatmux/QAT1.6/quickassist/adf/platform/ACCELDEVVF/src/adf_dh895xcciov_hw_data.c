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
 *  version: QAT1.6.L.2.6.0-65
 *
 *****************************************************************************/

/*****************************************************************************
 * @file adf_dh895xcciov_hw_data.c
 *
 * @ingroup
 *
 * @description
 *  This file contains the hw device data for DH895xcc virtual function.
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_platform_common.h"
#include "adf_platform_dh895xcciov.h"
#include "adf_platform_dh895x.h"
#include "adf_cfg.h"
#include "adf_drv_sriov.h"

/******************************************************************************
* External symbols
*******************************************************************************/
extern void adf_bank0_handler(void* handle);
extern void adf_bank0_polling_handler(void* handle);

/*******************************************************************************
* Static Variables
*******************************************************************************/
STATIC adf_hw_device_class_t dh895xcciov_class = {
    .name               = "dh895xcc"    ,
    .type               = DEV_DH895XCC  ,
    .numInstances       = 0             ,
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
    return ICP_DH895xCCIOV_ACCELERATORS_MASK;
}

/*
 * getAccelEnginesMask
 * Gets the accelerator engine mask based on the fuse information
 */
STATIC Cpa32U
getAccelEnginesMask(Cpa32U fuse)
{
    return ICP_DH895xCCIOV_ACCELENGINES_MASK;
}

/*
 * getNumAccelerators
 * Function returns the number of accelerators
 *
 */
STATIC Cpa32U
getNumAccelerators(adf_hw_device_data_t *self, Cpa32U mask)
{
    return ICP_DH895xCCIOV_MAX_ACCELERATORS;
}

/*
 * getNumAccelEngines
 * Function returns the number of accelerator engines
 */
STATIC Cpa32U
getNumAccelEngines(adf_hw_device_data_t *self, Cpa32U mask)
{
    return ICP_DH895xCCIOV_MAX_ACCELENGINES;
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
    pMaskList[0] = ICP_DH895xCCIOV_ACCELENGINES_MASK;

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
    mask |= ICP_ACCEL_CAPABILITIES_COMPRESSION;
    mask |= ICP_ACCEL_CAPABILITIES_LZS_COMPRESSION;
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC;
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
    mask |= ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER;

    return mask;
}

/*
 * getMiscBarId
 * Returns the VF-to-PF BAR id of the virtual function
 */
STATIC Cpa32U
getMiscBarId(adf_hw_device_data_t *self)
{
    return ICP_DH895xCCIOV_VFTOPF_BAR;
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
        *offset = ICP_DH895xCCIOV_ETRING_CSR_OFFSET;
    }
    return ICP_DH895xCCIOV_PETRINGCSR_BAR;
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
 * Function initialise internal hw_data data
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
 * Function cleanup internal hw_data data
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
getDevSKU(Cpa32U numAccEngines, Cpa32U numAccels, Cpa32U clkMask,
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
    return ICP_DH895xCCIOV_PF2VFDBINT_OFFSET;
}

/*
 * getVIntSrcOffset
 * This function returns byte offset of the VF interrupt srource CSR.
 */
STATIC Cpa32U
getVIntSrcOffset(Cpa32U id)
{
    return ICP_DH895xCCIOV_VINTSRC_OFFSET;
}

/*
 * getVIntMskOffset
 * This function returns byte offset of the VF interrupt mask CSR.
 */
STATIC Cpa32U
getVIntMskOffset(Cpa32U id)
{
    return ICP_DH895xCCIOV_VINTMSK_OFFSET;
}

/*
 * getVIntMskDefault
 * This function returns the default VF interrupt mask.
 */
STATIC Cpa32U
getVIntMskDefault(void)
{
    return ICP_DH895xCCIOV_VINTMSK_DEFAULT;
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
 * adf_addFwMsgSizesToCfgTable
 * Add internal config values to general section of the given accel dev config table
 */
STATIC CpaStatus
adf_addFwMsgSizesToCfgTable(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_FAIL;
    adf_cfg_device_data_t *cfg = NULL;
    adf_cfg_section_t *section_general = NULL;
    Cpa64U val = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    status = adf_cfgDeviceFind(accel_dev->accelId, &cfg);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to find cfg device\n");
        return status;
    }

    section_general = adf_cfgSectionFind(cfg->config_section,
                                         GENERAL_SEC);
    if(NULL == section_general)
    {
        ADF_ERROR("Could not find section %s\n", GENERAL_SEC);
        return CPA_STATUS_FAIL;
    }

    /************* add msg size values **********************/
    val = ICP_DH895xCC_FW_MSG_SIZE_SYMCY_TX;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ICP_CFG_FW_MSG_SIZE_SYMCY_TX_KEY,
            &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add %s\n",ICP_CFG_FW_MSG_SIZE_SYMCY_TX_KEY);
        return status;
    }

    val = ICP_DH895xCC_FW_MSG_SIZE_SYMCY_RX;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ICP_CFG_FW_MSG_SIZE_SYMCY_RX_KEY,
            &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add %s\n",ICP_CFG_FW_MSG_SIZE_SYMCY_RX_KEY);
        return status;
    }

    val = ICP_DH895xCC_FW_MSG_SIZE_ASYMCY_TX;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ICP_CFG_FW_MSG_SIZE_ASYMCY_TX_KEY,
            &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add %s\n",ICP_CFG_FW_MSG_SIZE_ASYMCY_TX_KEY);
        return status;
    }

    val = ICP_DH895xCC_FW_MSG_SIZE_ASYMCY_RX;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ICP_CFG_FW_MSG_SIZE_ASYMCY_RX_KEY,
            &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add %s\n",ICP_CFG_FW_MSG_SIZE_ASYMCY_RX_KEY);
        return status;
    }

    val = ICP_DH895xCC_FW_MSG_SIZE_DC_TX;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ICP_CFG_FW_MSG_SIZE_DC_TX_KEY,
            &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add %s\n",ICP_CFG_FW_MSG_SIZE_DC_TX_KEY);
        return status;
    }

    val = ICP_DH895xCC_FW_MSG_SIZE_DC_RX;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ICP_CFG_FW_MSG_SIZE_DC_RX_KEY,
            &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add %s\n",ICP_CFG_FW_MSG_SIZE_DC_RX_KEY);
        return status;
    }

    val = ICP_DH895xCC_FW_MSG_SIZE_ADMIN_TX;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ICP_CFG_FW_MSG_SIZE_ADMIN_TX_KEY,
            &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add %s\n",ICP_CFG_FW_MSG_SIZE_ADMIN_TX_KEY);
        return status;
    }

    val = ICP_DH895xCC_FW_MSG_SIZE_ADMIN_RX;
    status = adf_cfgAddKeyValueParam(accel_dev, section_general,
            ICP_CFG_FW_MSG_SIZE_ADMIN_RX_KEY,
            &val, ADF_DEC);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to add %s\n",ICP_CFG_FW_MSG_SIZE_ADMIN_RX_KEY);
        return status;
    }

    return status;
}

/*
 * adf_isr_enable_unco_err_interrupts
 * Does nothing on VF
 */
STATIC CpaStatus
adf_enable_unco_err_interrupts(icp_accel_dev_t *accel_dev)
{
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_isr_handleAEInterrupt
 * Returns false VF
 */
STATIC CpaBoolean
adf_isr_handleUncoInterrupt(icp_accel_dev_t *accel_dev)
{
    return CPA_FALSE;
}

/*
 * getHardwareVersion
 * Function returns the hardware version in a string.
 */
STATIC CpaStatus
getHardwareVersion(char *str, Cpa32U revision_id)
{
    char maj_step = 'A';
    int  min_step  = 0;

    if (NULL == str)
    {
        return CPA_STATUS_FAIL;
    }

    maj_step += ((revision_id & DH895xCC_MAJOR_STEPPING_MASK)
             >> DH895xCC_MINOR_STEPPING_MASK);
    min_step  += (revision_id & DH895xCC_MINOR_STEPPING_MASK);
    snprintf(str, ADF_CFG_MAX_VAL_LEN_IN_BYTES, "%c%d",
             maj_step, min_step);

        return CPA_STATUS_SUCCESS;
}

/*
 * adf_systemMsg_VF2PF_putMsg
 * called by the system (kernel space) to send
 * a message from the VF to the PF
 */
CpaStatus adf_VF2PF_putMsg(icp_accel_dev_t *accel_dev, Cpa32U msgIn)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U ctr = 0, val = 0;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U vf2pf_bar_id = 0;
    Cpa32U doorbell_csr_offset = 0;
    Cpa32U msg = msgIn;
    void *pmisc_bar_addr = NULL;
    ADF_DEBUG("Msg 0x%x",msgIn);

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    hw_data = accel_dev->pHwDeviceData;
    ICP_CHECK_FOR_NULL_PARAM(hw_data);
    vf2pf_bar_id = hw_data->getMiscBarId(hw_data);
    pmisc_bar_addr = (void*)
        accel_dev->pciAccelDev.pciBars[vf2pf_bar_id].virtAddr;
    ICP_CHECK_FOR_NULL_PARAM(pmisc_bar_addr);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->lock);
    
    doorbell_csr_offset = hw_data->getPf2VfDbIntOffset(0);

    status = ICP_MUTEX_LOCK_TIMEOUT(accel_dev->lock, 
            ICP_PFVFCOMMS_INTERUPT_PENDING_DELAY);
    
    if (CPA_STATUS_SUCCESS == status)
    {
        /* Check if CSR is in use by the other side */
        val = ICP_ADF_CSR_RD(pmisc_bar_addr, doorbell_csr_offset);
        if (adf_pfvfcomms_isCSRInUseByPF(val))
        {
            ADF_DEBUG("CSR for Dev%d PFVF comms is in use by PF\n",
                                accel_dev->accelId);
            status = CPA_STATUS_RESOURCE;
            ICP_MUTEX_UNLOCK(accel_dev->lock);
        }

        /* No-one is using the CSR, so go ahead and use it */
        if (CPA_STATUS_SUCCESS == status)
        {
            adf_pfvfcomms_setVFInUse(&msg);
            ADF_DEBUG("writing to Dev%d doorbell CSR: 0x%08x\n",
                            accel_dev->accelId, msg);
            ICP_ADF_CSR_WR(pmisc_bar_addr, doorbell_csr_offset, msg);

            /* Now wait in case other side tried to use at same time */
            ICP_MSLEEP(ICP_PFVFCOMMS_COLLISION_DETECTION_DELAY);
            
            /* collision detection */
            val = ICP_ADF_CSR_RD(pmisc_bar_addr, doorbell_csr_offset);
            if (CPA_FALSE == adf_pfvfcomms_isCSRInUseByVF(val))
            {
                /* Didn't read back what we just wrote! */
                ADF_DEBUG("Collision detected. CSR for PFVF comms"
                        " is in use by PF\n");
                status = CPA_STATUS_RESOURCE;
                ICP_MUTEX_UNLOCK(accel_dev->lock);
            }
            else
            {

                /* Now we own the CSR :) */
                /* The VF_IN_USE pattern must remain in CSR for all writes
                 * including ack from other side, until we relinquish the CSR*/

                /* interrupt the other side */
                adf_vf2pf_set_irq(&msg);
                ADF_DEBUG("writing to dev%d doorbell CSR: 0x%08x\n",
                                accel_dev->accelId, msg);
                ICP_ADF_CSR_WR(pmisc_bar_addr, doorbell_csr_offset, msg);
            
                /* And wait for confirmation that it picked off the msg */
                do{
                    ICP_MSLEEP(ICP_PFVFCOMMS_CONFIRMATION_SLEEP_TIME);
                    val = ICP_ADF_CSR_RD(pmisc_bar_addr, doorbell_csr_offset);
                } while(
                    (ICP_VF2PF_IRQ_CLEARED != (val & ICP_VF2PF_MSGMASK_IRQ))
                        && (ctr++ < ICP_PFVFCOMMS_CONFIRMATION_MAX_LOOPS));

                /* Return success only if we got ACK from PF */
                status = (ICP_VF2PF_IRQ_CLEARED == (val&ICP_VF2PF_MSGMASK_IRQ))
                        ? CPA_STATUS_SUCCESS : CPA_STATUS_RESOURCE;
                
                if(CPA_STATUS_SUCCESS != status)
                {
                    ADF_DEBUG("Ack not received on Dev%d for VF->PF message\n",
                            accel_dev->accelId);
                    /* Clear the irq bit as the PF hasn't done this */
                    adf_pf2vf_ack(&val);
                }

                /* Finished with CSR. Relinquish it.
                 * Leave msg in CSR for debug */
                msg = val;
                adf_pfvfcomms_clearVFInUse(&msg);
                ADF_DEBUG("writing to DEV%d doorbell CSR: 0x%08x\n",
                            accel_dev->accelId, msg);
                ICP_ADF_CSR_WR(pmisc_bar_addr,doorbell_csr_offset, msg);
                ICP_MUTEX_UNLOCK(accel_dev->lock);
            }
        }
    }
    else 
    {
        ADF_DEBUG("PFVF CSR for dev%d already in use by another thread on VF,"
                        " could not get access\n", accel_dev->accelId);
        status = CPA_STATUS_RESOURCE;
    }
    
    ADF_DEBUG("%s: Dev%d status = %d\n", __FUNCTION__,
                            accel_dev->accelId, status);
    return status;
}

/*
 * adf_systemMsg_VF2PF_putMsg
 * 
 * called by the system (kernel space) to send a message
 * from the VF to the PF
 */
CpaStatus adf_systemMsg_VF2PF_putMsg(icp_accel_dev_t *vf, Cpa32U msg )
{
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32U cntr = 0;

    do{
        status = adf_VF2PF_putMsg(vf, msg );
        cntr++;
        if (CPA_STATUS_FAIL == status)
        {
            break;        
        }
        else if (CPA_STATUS_RESOURCE == status)
        {
            ICP_MSLEEP(ICP_PFVFCOMMS_PUTMSG_RETRY);
        }
    }while (status != CPA_STATUS_SUCCESS && cntr < 
            ICP_PFVFCOMMS_PUTMSG_MAXRETRIES );

    return status;
}

/*
 * adf_set_hw_data_dh895xcc
 * Initialise the hw_data structure
 */
void
adf_set_hw_data_dh895xcciov(adf_hw_device_data_t *hw_data)
{
    adf_hw_device_data_t *hw = NULL;

    ICP_CHECK_FOR_NULL_PARAM_VOID(hw_data);

    hw = hw_data;

    hw->init    = init;
    hw->cleanup = cleanup;

    hw->dev_class = &dh895xcciov_class;
    hw->adf_addFwMsgSizesToCfgTable = adf_addFwMsgSizesToCfgTable;

    /* populate bank/ac information */
    hw->maxBars               = ICP_DH895xCCIOV_MAX_PCI_BARS;
    hw->maxNumBanks           = ICP_DH895xCCIOV_ETR_MAX_BANKS;
    hw->maxNumApBanks         = 0;
    hw->numRingsPerBank       = ICP_DH895xCCIOV_ETR_MAX_RINGS_PER_BANK;
    hw->numBanksPerAccel      = ICP_DH895xCCIOV_BANKS_PER_ACCELERATOR;
    hw->maxNumAccel           = ICP_DH895xCCIOV_MAX_ACCELERATORS;
    hw->maxNumAccelEngines    = ICP_DH895xCCIOV_MAX_ACCELENGINES;
    hw->aeClockInMhz          = ICP_DH895xCCIOV_AE_CLOCK_IN_MHZ;
    hw->sintpfOffset          = 0;
    hw->sintpf1Offset         = 0;
    hw->userEtringCsrSize     = ICP_DH895xCCIOV_USER_ETRING_CSR_SIZE;

    /* populate msix information */
    hw->msix.banksVectorStart = 0;
    hw->msix.banksVectorNum   = 0;
    hw->msix.aeVectorStart    = 0;
    hw->msix.aeVectorNum      = 0;

    /* device fuse */
    hw->getDevFuseOffset      = NULL;
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

    /* Get the reset constants and offsets */
    hw->getResetOffsets       = NULL;

    /* populate PMiscBar callback  */
    hw->setSmiapfOffsetMask   = NULL;

    hw->irqGetBankNumber      = NULL;
    hw->irqGetAeSource        = NULL;
    hw->irqGetBunSource       = irqGetBunSource;
    hw->irqGetPF2VFSource     = irqGetPF2VFSource;
    hw->getBankBhHandler        = getBankBhHandler;
    hw->getBankBhPollingHandler = getBankBhPollingHandler;

    hw->accessCsrIntFlagAndCol = access_csr_int_flag_and_col;

    /* Auto-push feature callback */
    hw->isAeAutoPushSupported   = CPA_FALSE;
    hw->getAeApCsrOffsets       = NULL;

    hw->initAdminComms          = NULL;
    hw->restoreDevice           = NULL;

    /* Arbitration feature */
    hw->isArbitrationSupported  = CPA_TRUE;
    hw->arbInit                  = NULL;
    hw->arbShutdown              = NULL;

    hw->isGigeWatchdogSupported = CPA_FALSE;

    hw->getVf2PfIntSourceOffset = NULL;
    hw->getVf2PfBitOffset       = NULL;
    hw->getVf2PfIntMaskOffset   = NULL;
    hw->getVf2PfIntMaskDefault  = NULL;
    hw->getTiMiscIntCtlOffset   = NULL;
    
    hw->disableVf2PfInterrupts  = NULL;
    hw->enableVf2PfInterrupts   = NULL;
    hw->getVf2PfIntSrcMask      = NULL;

    hw->getVIntSrcOffset        = getVIntSrcOffset;
    hw->irqGetPF2VFSource       = irqGetPF2VFSource;
    hw->getVIntMskOffset        = getVIntMskOffset;
    hw->getVIntMskDefault       = getVIntMskDefault;
    hw->getPf2VfDbIntOffset     = getPf2VfDbIntOffset;
    hw->getHardwareVersion      = getHardwareVersion;    

    hw->enableUncoErrInterrupts = adf_enable_unco_err_interrupts;
    hw->handleUncoErrInterrupts = adf_isr_handleUncoInterrupt;
    
    hw->adf_PF2VF_putMsg              = NULL;
    hw->adf_VF2PF_putMsg              = adf_VF2PF_putMsg;
    hw->adf_systemMsg_VF2PF_putMsg    = adf_systemMsg_VF2PF_putMsg;
    hw->adf_systemMsg_PF2VF_putMsg    = NULL;

    hw->icp_version.versionMsgAvailable = CPA_FALSE;
    hw->icp_version.device_lowestSupportMajorVersion =
            ICP_DH895xCC_LOWESTSUPPORTED_MAJOR_VERSION;
    hw->icp_version.device_lowestSupportMinorVersion =
            ICP_DH895xCC_LOWESTSUPPORTED_MINOR_VERSION;
}
