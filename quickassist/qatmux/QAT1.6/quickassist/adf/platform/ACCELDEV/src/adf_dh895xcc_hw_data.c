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
 * @file adf_dh895xcc_hw_data.c
 *
 * @ingroup
 *
 * @description
 * This file contains the device configuration for DH895xcc device
 *****************************************************************************/

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_platform_common.h"
#include "adf_platform_dh895x.h"
#include "adf_cfg.h"
#include "adf_adminreg_mgr.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_arb.h"
#include "icp_firml_interface.h"
#include "icp_adf_accel_mgr.h"
#include "adf_drv.h"
#include "adf_ae.h"
#include "adf_drv_sriov.h"

#define ICP_DH895xCC_CTX_ECC_ERR(val)      ((val & (1<<29)) == 0 ? "No": "Yes")
#define ICP_DH895xCC_CTX_PAR_ERR(val)      ((val & (1<<25)) == 0 ? "No": "Yes")
#define ICP_DH895xCC_EPERRLOG_ERR(val)     ((val == 0) ? "No": "Yes")
#define ICP_DH895xCC_USTORE_IS_UE(val)     ((val & (1<<31)) == 0 ? "No": "Yes")
#define ICP_DH895xCC_REG_GET_REG_AD(val)    (val & 0xFFFF) /* Faulty Reg/LM */
#define ICP_DH895xCC_REG_TYPE(val)         ((val & (0x3<<16)) >> 16)
#define ICP_DH895xCC_REG_TYP_STR(val)     \
    ((val == 0)? "GPR":                   \
    ((val == 1)? "Transfer":              \
    ((val == 2)? "Next Neighbour":        \
                 "Local Memory")))
#define ICP_DH895xCC_UERRSSMMMP_ERRTYPE(val) \
    ((val & (0xF<<4)) >> 4)
#define ICP_DH895xCC_UERRSSMMMP_ERRTYPE_STR(val) \
    ((val == 0) ? "ECC data error in MMP program memory": \
    ((val == 1) ? "ECC data error in MMP OPA Data Ram":     \
    ((val == 2) ? "ECC data error in MMP OPB Data Ram":     \
                  "Reserved")))
#define ICP_DH895xCC_UERRSSMSH_GET_OP(val) \
    ((val & ICP_DH895xCC_UERRSSMSH_R) != 0 ? "Read"  : \
    ((val & ICP_DH895xCC_UERRSSMSH_W) != 0 ? "Write" : \
                                             "Error"))
#define ICP_DH895xCC_UERRSSMSH_ERRTYPE(val) \
    ((val & (0xF<<4)) >> 4)
#define ICP_DH895xCC_UERRSSMSH_GET_ERRTYPE(val)    \
    ((val == 0) ? "ECC data error on Slice Push Command" : \
    ((val == 2) ? "ECC data error on AE Push Command"    : \
    ((val == 3) ? "ECC data error on AE Pull Command"    : \
    ((val == 5) ? "ECC data error on DRAM Pull Command"  : \
    ((val == 6) ? "ECC data error on CPM Memory read (DRAM push)" : \
    ((val == 7) ? "ECC data error on CPM Memory Bus Master Read"  : \
                  "Reserved"))))))
#define ICP_DH895xCC_PPERR_GET_STATUS(val)     \
    (((val & ICP_DH895xCC_PPERR_PPERR)) != 0 ? \
    (((val & ICP_DH895xCC_PPERR_MERR) != 0)  ? "Multiple errors occurred" : \
                                               "One error occured")       : \
                                               "No errors occurred")
#define ICP_DH895xCC_PPERR_TYPE(val)     ((val & (0xF<<4)) >> 4)
#define ICP_DH895xCC_PPERR_TYPE_STR(val) \
    ((val == 0) ? "Data asserted on AE Pull Data Bus"   : \
    ((val == 1) ? "Data asserted on Dram Push Data Bus" : \
    ((val == 2) ? "Master Bus Read Return Error"        : \
    ((val == 3) ? "ECC error on Shared mem access to Dram Pull Data Bus" : \
                  "Reserved"))))
#define ICP_DH895xCC_CPP_SHAC_ERTYP_STR(val) \
    ((val == 0)? "Data error during Scratch Operation" : \
    ((val == 1)? "Data error during Hash Operation"    : \
    ((val == 2)? "Data error during CAP - impossible for push" : \
                 "reserved")))
#define ICP_DH895xCC_ESRAMUERR_GET_UERR(val) \
    ((val & ICP_DH895xCC_ESRAMUERR_UERR) != 0 ? "Yes" : "No")
#define ICP_DH895xCC_ESRAMUERR_GET_ERRSTR(val) \
    (val == 0 ? "ECC data Error" : \
                "Reserved")
#define ICP_DH895xCC_ESRAMUERR_GET_OP(val) \
    ((val & ICP_DH895xCC_ESRAMUERR_R) != 0? "Read" : \
    ((val & ICP_DH895xCC_ESRAMUERR_W) != 0? "Write": "Error"))
#define ICP_DH895xCC_CPPMEMTGTERR_ERRTYP_STR(val) \
    ((val == 4) ? "Memory Read Transaction" : \
    ((val == 5) ? "Memory write" : \
    ((val == 3) ? "P2S Data Path" : \
                  "reserved")))
#define ICP_DH895xCC_CPPMEMTGTERR_GET_STATUS(val) \
    (((val & ICP_DH895xCC_CPPMEMTGTERR_ERR)) != 0  ? \
    (((val & ICP_DH895xCC_CPPMEMTGTERR_MERR) != 0) ? "Multiple errors occurred": \
                                                     "One error occured"):       \
                                                     "No errors occurred")
#define ICP_DH895xCC_TICPP_GETOP(val) \
    ((val&ICP_DH895xCC_TICPP_PUSH) != 0 ? "Push" : "Pull")

#define ICP_DH895XCC_MSLEEP_TIME                      1000
#define ICP_DH895XCC_MSLEEP_TRIP_TIME                  200
#define ICP_DH895XCC_MSLEEP_SBR_TIME                    50

/*****************************************************************************
 * External symbols
 *****************************************************************************/
extern void adf_bank0_handler(void* handle);
extern void adf_bank1_handler(void* handle);
extern void adf_bank2_handler(void* handle);
extern void adf_bank3_handler(void* handle);
extern void adf_bank4_handler(void* handle);
extern void adf_bank5_handler(void* handle);
extern void adf_bank6_handler(void* handle);
extern void adf_bank7_handler(void* handle);
extern void adf_bank8_handler(void* handle);
extern void adf_bank9_handler(void* handle);
extern void adf_bank10_handler(void* handle);
extern void adf_bank11_handler(void* handle);
extern void adf_bank12_handler(void* handle);
extern void adf_bank13_handler(void* handle);
extern void adf_bank14_handler(void* handle);
extern void adf_bank15_handler(void* handle);
extern void adf_bank16_handler(void* handle);
extern void adf_bank17_handler(void* handle);
extern void adf_bank18_handler(void* handle);
extern void adf_bank19_handler(void* handle);
extern void adf_bank20_handler(void* handle);
extern void adf_bank21_handler(void* handle);
extern void adf_bank22_handler(void* handle);
extern void adf_bank23_handler(void* handle);
extern void adf_bank24_handler(void* handle);
extern void adf_bank25_handler(void* handle);
extern void adf_bank26_handler(void* handle);
extern void adf_bank27_handler(void* handle);
extern void adf_bank28_handler(void* handle);
extern void adf_bank29_handler(void* handle);
extern void adf_bank30_handler(void* handle);
extern void adf_bank31_handler(void* handle);

extern void adf_bank0_polling_handler(void* handle);
extern void adf_bank1_polling_handler(void* handle);
extern void adf_bank2_polling_handler(void* handle);
extern void adf_bank3_polling_handler(void* handle);
extern void adf_bank4_polling_handler(void* handle);
extern void adf_bank5_polling_handler(void* handle);
extern void adf_bank6_polling_handler(void* handle);
extern void adf_bank7_polling_handler(void* handle);
extern void adf_bank8_polling_handler(void* handle);
extern void adf_bank9_polling_handler(void* handle);
extern void adf_bank10_polling_handler(void* handle);
extern void adf_bank11_polling_handler(void* handle);
extern void adf_bank12_polling_handler(void* handle);
extern void adf_bank13_polling_handler(void* handle);
extern void adf_bank14_polling_handler(void* handle);
extern void adf_bank15_polling_handler(void* handle);
extern void adf_bank16_polling_handler(void* handle);
extern void adf_bank17_polling_handler(void* handle);
extern void adf_bank18_polling_handler(void* handle);
extern void adf_bank19_polling_handler(void* handle);
extern void adf_bank20_polling_handler(void* handle);
extern void adf_bank21_polling_handler(void* handle);
extern void adf_bank22_polling_handler(void* handle);
extern void adf_bank23_polling_handler(void* handle);
extern void adf_bank24_polling_handler(void* handle);
extern void adf_bank25_polling_handler(void* handle);
extern void adf_bank26_polling_handler(void* handle);
extern void adf_bank27_polling_handler(void* handle);
extern void adf_bank28_polling_handler(void* handle);
extern void adf_bank29_polling_handler(void* handle);
extern void adf_bank30_polling_handler(void* handle);
extern void adf_bank31_polling_handler(void* handle);

/*******************************************************************************
* Static Variables
*******************************************************************************/
STATIC adf_hw_device_class_t dh895xcc_class = {
    .name         = ICP_DH895XCC_DEVICE_NAME,
    .type         = DEV_DH895XCC  ,
    .numInstances = 0             ,
    .currentInstanceId = 0
};

STATIC bank_handler adf_dh895xcc_bh_bank_handlers[] = {
    adf_bank0_handler,
    adf_bank1_handler,
    adf_bank2_handler,
    adf_bank3_handler,
    adf_bank4_handler,
    adf_bank5_handler,
    adf_bank6_handler,
    adf_bank7_handler,
    adf_bank8_handler,
    adf_bank9_handler,
    adf_bank10_handler,
    adf_bank11_handler,
    adf_bank12_handler,
    adf_bank13_handler,
    adf_bank14_handler,
    adf_bank15_handler,
    adf_bank16_handler,
    adf_bank17_handler,
    adf_bank18_handler,
    adf_bank19_handler,
    adf_bank20_handler,
    adf_bank21_handler,
    adf_bank22_handler,
    adf_bank23_handler,
    adf_bank24_handler,
    adf_bank25_handler,
    adf_bank26_handler,
    adf_bank27_handler,
    adf_bank28_handler,
    adf_bank29_handler,
    adf_bank30_handler,
    adf_bank31_handler,
};

STATIC bank_handler adf_dh895xcc_bh_polling_bank_handlers[] = {
    adf_bank0_polling_handler,
    adf_bank1_polling_handler,
    adf_bank2_polling_handler,
    adf_bank3_polling_handler,
    adf_bank4_polling_handler,
    adf_bank5_polling_handler,
    adf_bank6_polling_handler,
    adf_bank7_polling_handler,
    adf_bank8_polling_handler,
    adf_bank9_polling_handler,
    adf_bank10_polling_handler,
    adf_bank11_polling_handler,
    adf_bank12_polling_handler,
    adf_bank13_polling_handler,
    adf_bank14_polling_handler,
    adf_bank15_polling_handler,
    adf_bank16_polling_handler,
    adf_bank17_polling_handler,
    adf_bank18_polling_handler,
    adf_bank19_polling_handler,
    adf_bank20_polling_handler,
    adf_bank21_polling_handler,
    adf_bank22_polling_handler,
    adf_bank23_polling_handler,
    adf_bank24_polling_handler,
    adf_bank25_polling_handler,
    adf_bank26_polling_handler,
    adf_bank27_polling_handler,
    adf_bank28_polling_handler,
    adf_bank29_polling_handler,
    adf_bank30_polling_handler,
    adf_bank31_polling_handler,
};

#define NUM_ELEM(array)                   \
    sizeof (array) / sizeof (*array)

/*
 * getAcceleratorsMask
 * Gets the acceleration mask based on the fuse information
 */
STATIC Cpa32U
adf_dh895xcc_getAcceleratorsMask(Cpa32U fuse)
{
    return ((~fuse) >> (ICP_DH895xCC_ACCELERATORS_REG_OFFSET) &
                                ICP_DH895xCC_ACCELERATORS_MASK);
}

/*
 * getAccelEnginesMask
 * Gets the accelerator engine mask based on the fuse information
 */
STATIC Cpa32U
adf_dh895xcc_getAccelEnginesMask(Cpa32U fuse)
{
    return ((~fuse) & ICP_DH895xCC_ACCELENGINES_MASK);
}

/*
 * getDevFuseOffset
 * Function returns the device fuse control offset
 */
STATIC Cpa32U
adf_dh895xcc_getDevFuseOffset(void)
{
    return ICP_DH895xCC_FUSECTL_OFFSET;
}

/*
 * getDevSKU
 * Function returns device SKU info
 */
STATIC dev_sku_info_t
adf_dh895xcc_getDevSKU(Cpa32U numAccEngines, Cpa32U numAccels, 
                       Cpa32U clkMask, Cpa32U fuse)
{
    Cpa32U productSKU = 0;

    productSKU = (fuse & ICP_DH895xCC_FUSECTL_SKU_MASK)
                            >> ICP_DH895xCC_FUSECTL_SKU_SHIFT;

    switch (productSKU)
    {
        case ICP_DH895xCC_FUSECTL_SKU_1:
            return DEV_SKU_1;
        case ICP_DH895xCC_FUSECTL_SKU_2:
            return DEV_SKU_2;
        case ICP_DH895xCC_FUSECTL_SKU_3:
            return DEV_SKU_3;
        case ICP_DH895xCC_FUSECTL_SKU_4:
            return DEV_SKU_4;
        default:
            return DEV_SKU_UNKNOWN;
    }

    return DEV_SKU_UNKNOWN;
}

/*
 * getNumAccelerators
 * Function returns the number of accelerators
 *
 */
STATIC Cpa32U
adf_dh895xcc_getNumAccelerators(adf_hw_device_data_t *self, Cpa32U mask)
{
    Cpa32U i = 0, ctr = 0;

    if (!mask)
    {
        return 0;
    }
    for (i = 0; i < ICP_DH895xCC_MAX_ACCELERATORS; i++)
    {
        if (mask & (1 << i))
        {
            ctr++;
        }
    }

    return ctr;
}

/*
 * getNumAccelEngines
 * Function returns the number of accelerator engines
 */
STATIC Cpa32U
adf_dh895xcc_getNumAccelEngines(adf_hw_device_data_t *self, Cpa32U mask)
{
    Cpa32U i, ctr = 0;

    if (!mask)
    {
        return 0;
    }

    for (i = 0; i < ICP_DH895xCC_MAX_ACCELENGINES; i++)
    {
        if (mask & (1 << i))
        {
            ctr++;
        }
    }

    return ctr;
}

/*
 * getAccelMaskList
 * Function returns the accelerator mask for the accelerators in the system
 */
STATIC void
adf_dh895xcc_getAccelMaskList(struct adf_hw_device_data_s *self,
                Cpa32U accelMask, Cpa32U aeMask, Cpa32U *pMaskList)
{
    Cpa32U i = 0;

    /*initialise acclerator mask to zero*/
    for(i = 0; i < self->maxNumAccel; i++)
    {
        pMaskList[i] = 0;
    }

    /*
     * Since we only have 1 accelerator to associate with, just return the mask
     */
    if(0 != self->getNumAccelerators(self, accelMask))
    {
        pMaskList[0] = aeMask;
    }

    return;
}

/*
 * getHardwareVersion
 * Function returns the hardware version in a string.
 */
STATIC CpaStatus
adf_dh895xcc_getHardwareVersion(char *str, Cpa32U revision_id)
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
 * getAccFuncOffset
 * Function returns the accelerator capabilities offset
 */
STATIC Cpa32U
adf_dh895xcc_getAccFuncOffset(void)
{
    return ICP_DH895xCC_LEGFUSE_OFFSET;
}

/*
 * getAccFuncMask
 * Gets the accelerator functionality mask based on the accelerator capability
 */
STATIC Cpa32U
adf_dh895xcc_getAccFuncMask(Cpa32U func)
{
    Cpa32U mask = 0;
    Cpa32U accFunc = 0;

    /* func is the value of the register */
    accFunc = func;

    /* invert CSR value */
    accFunc = ~accFunc;

    /* mask as only care about bottom 5 bits */
    accFunc &= ICP_DH895xCC_LEGFUSE_MASK;

    if( accFunc & ( 1 << ICP_DH895xCC_LEGFUSES_FN0BIT0 ))
        mask |= ICP_ACCEL_CAPABILITIES_CIPHER;

    if( accFunc & ( 1 << ICP_DH895xCC_LEGFUSES_FN0BIT1 ))
        mask |= ICP_ACCEL_CAPABILITIES_AUTHENTICATION;

    if( accFunc & ( 1 << ICP_DH895xCC_LEGFUSES_FN0BIT2 ))
        mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;

    if( accFunc & ( 1 << ICP_DH895xCC_LEGFUSES_FN0BIT3 ))
        mask |= ICP_ACCEL_CAPABILITIES_COMPRESSION;

    if( accFunc & ( 1 << ICP_DH895xCC_LEGFUSES_FN0BIT4 ))
        mask |= ICP_ACCEL_CAPABILITIES_LZS_COMPRESSION;

    if( (accFunc & ( 1 << ICP_DH895xCC_LEGFUSES_FN0BIT0 )) &&
                (accFunc & ( 1 << ICP_DH895xCC_LEGFUSES_FN0BIT1 )) )
    {
            mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC;

            /* add random number capability if crypto is supported*/
            mask |= ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER;
    }

    return mask;
}

/*
 * getMiscBarId
 * Returns the MISC BAR id of the device
 */
STATIC Cpa32U
adf_dh895xcc_getMiscBarId(adf_hw_device_data_t *self)
{
    return ICP_DH895xCC_PMISC_BAR;
}

/*
 * setSmiapfOffsetMask
 * Function sets the SMIAPF offset and default mask
 */
STATIC void
adf_dh895xcc_setSmiapfOffsetMask(adf_hw_device_data_t *self,
                                 icp_accel_pci_info_t *pci_info)
{
    Cpa32U pmisc_bar_id = 0;
    UARCH_INT *pmisc_bar_addr = NULL;

    if((NULL == self) || (NULL == pci_info))
        return;

    pmisc_bar_id = self->getMiscBarId(self);

    /*Get base address (PMISCBAROFFSET must still be added,
    value is included in CSR defines)*/
    pmisc_bar_addr = (void*)pci_info->pciBars[pmisc_bar_id].virtAddr;

    /*Enable all bundle interrupts*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_SMIAPF0_MASK_OFFSET,
                                                ICP_DH895xCC_SMIA0_MASK);
    /*Enable Misc interrupt source*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_SMIAPF1_MASK_OFFSET,
                                                ICP_DH895xCC_SMIA1_MASK);
}

/*
 * getSramBarId
 * Returns the SRAM BAR id of the device
 */
STATIC Cpa32U
adf_dh895xcc_getSramBarId(adf_hw_device_data_t *self)
{
    return ICP_DH895xCC_PESRAM_BAR;
}

/*
 * getEtrBarId
 * Returns the ETR BAR id of the device
 */
STATIC Cpa32U
adf_dh895xcc_getEtrBarId(adf_hw_device_data_t *self, Cpa32U *offset)
{
    if (NULL != offset)
    {
        *offset = ICP_DH895xCC_ETRING_CSR_OFFSET;
    }
    return ICP_DH895xCC_PETRINGCSR_BAR;
}

/*
 * isBarAvailable
 * Returns whether the barId is available on the device
 */
STATIC CpaBoolean
adf_dh895xcc_isBarAvailable(adf_hw_device_data_t *self, Cpa32U barId)
{
    CpaBoolean bars[] = {CPA_TRUE, CPA_TRUE, CPA_TRUE};

    return (barId >= (sizeof (bars) / sizeof (*bars))) ?
        CPA_FALSE                                      :
        bars[barId];
}

/*
 * getEsramInfo
 * Function sets the eSRAM information of the device
 */
STATIC void
adf_dh895xcc_getEsramInfo(adf_hw_device_data_t *self,
                          adf_esram_data_t *esramInfo)
{
    if (NULL != esramInfo)
    {
        esramInfo->sramAeAddr = ICP_DH895xCC_SRAM_AE_ADDR;
        esramInfo->sramAeSize = ICP_DH895xCC_SRAM_AE_SIZE;
    }
}

/*
 * getScratchRamInfo
 * Function sets the scratch information of the device
 */
STATIC void
adf_dh895xcc_getScratchRamInfo(struct adf_hw_device_data_s *self,
            Cpa32U *offset, Cpa32U *size)
{
    if (NULL != offset)
    {
        *offset = ICP_DH895xCC_REGION_SCRATCH_RAM_OFFSET;
    }

    if (NULL != size)
    {
        *size = ICP_DH895xCC_REGION_SCRATCH_RAM_SIZE;
    }
}

/*
 * irqGetBankNumber
 * Function returns the bundle interrupt mask based on interrupt source
 * (sintpf0, which has already been read in the calling function)
 */
STATIC Cpa32U
adf_dh895xcc_irqGetBankNumber(adf_hw_device_data_t *self,
            Cpa32U sintpf, Cpa32U *miscAddr)
{
    /*Single bank per IRQ so no need to pull out an interrupt mask*/
    return sintpf;
}

/*
 * irqGetAeSource
 * Function returns the AE source based on interrupt source(sintpf1)
 */
STATIC Cpa32U
adf_dh895xcc_irqGetAeSource(Cpa32U sintpf, Cpa32U *miscAddr)
{
    if(NULL == miscAddr)
        return 0;

    return irq_ae_source(miscAddr);
}

/*
 * init
 * Function initialise internal hw_data data
 */
STATIC CpaStatus
adf_dh895xcc_init(adf_hw_device_data_t *self, Cpa8U node_id)
{
    ICP_CHECK_FOR_NULL_PARAM(self);

    self->instanceId = self->dev_class->currentInstanceId++;
    ++self->dev_class->numInstances;

    return CPA_STATUS_SUCCESS;
}

/*
 * cleanup
 * Function cleanup internal hw_data data
 */
STATIC void
adf_dh895xcc_cleanup(adf_hw_device_data_t *self)
{
    ICP_CHECK_FOR_NULL_PARAM_VOID(self);

    --self->dev_class->numInstances;
    self->instanceId = self->dev_class->currentInstanceId--;

    return;
}

/*
 * adf_addFwMsgSizesToCfgTable
 * Add internal config values to general section of the accel dev config table
 */
STATIC CpaStatus
adf_dh895xcc_adf_addFwMsgSizesToCfgTable(icp_accel_dev_t *accel_dev)
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
 * getBankBhHandler
 * Function returns a bank handler based on bankId
 */
STATIC bank_handler
adf_dh895xcc_getBankBhHandler(Cpa32U bankId)
{
    return (bankId < NUM_ELEM(adf_dh895xcc_bh_bank_handlers)) ?
        adf_dh895xcc_bh_bank_handlers[bankId]                 :
        NULL;
}

/*
 * getBankBhPollingHandler
 * Function returns a polling bank handler based on bankId
 */
STATIC bank_handler
adf_dh895xcc_getBankBhPollingHandler(Cpa32U bankId)
{
    return (bankId < NUM_ELEM(adf_dh895xcc_bh_polling_bank_handlers)) ?
        adf_dh895xcc_bh_polling_bank_handlers[bankId]                 :
        NULL;
}

/*
 * Virtualization section
 */

/*
 * getVf2PfIntSourceOffset
 * This function returns byte offset of the VF-to-PF interrupt source CSR.
 */
STATIC Cpa32U
adf_dh895xcc_getVf2PfIntSourceOffset(void)
{
    return ICP_PMISCBAR_VFTOPFINTSRC;
}

/*
 * getVf2PfBitOffset
 * This function returns the VF-to-PF bit offset in the interrupt source CSR.
 */
STATIC Cpa32U
adf_dh895xcc_getVf2PfBitOffset(void)
{
    return ICP_VFTOPF_INTSOURCEOFFSET;
}

/*
 * getVf2PfIntMaskOffset
 * This function returns byte offset of the VF-to-PF interrupt mask CSR.
 */
STATIC Cpa32U
adf_dh895xcc_getVf2PfIntMaskOffset(void)
{
    return ICP_PMISCBAR_VFTOPFINTMSK;
}

/*
 * getVf2PfIntMaskDefault
 * This function returns the default VF-to-PF interrupt mask.
 */
STATIC Cpa32U
adf_dh895xcc_getVf2PfIntMaskDefault(void)
{
    return ICP_VFTOPF_INTSOURCEMASK;
}

/*
 * adf_dh895xcc_disableVf2PfInterrupts
 * This function disables VF to PF interrupts based on the vfMsk
 * e.g If we want to disable VF to PF interrupts for all 32 VFs a
 * vfMsk of 0xFFFFFFFF should be passed in.
 */
STATIC void
adf_dh895xcc_disableVf2PfInterrupts(icp_accel_dev_t *accel_dev, Cpa32U vfMsk)
{
    void *pmisc_bar_addr = NULL;
    Cpa32U temp_reg = 0;

    ICP_CHECK_FOR_NULL_PARAM_VOID(accel_dev);

    pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);

    /* Disable VF to PF interrupts for VFs that are in
     * range of 1 to 16 corresponding to bits 0-15 in vfMsk */
    if (vfMsk & ICP_DH895xCC_VFTOPF_MSK_16BITS)
    {
        temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK3) |
                ICP_DH895xCC_VFTOPF_MSK_1TO16(vfMsk);
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK3, temp_reg);
    }

    /* Disable VF to PF interrupts for VFs that are in
     * range of 17 to 32 corresponding to bits 16-31 in vfMsk */
    if (vfMsk >> ICP_DH895xCC_VFTOPF_SHIFT_17TO32)
    {
        temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK5) |
            ICP_DH895xCC_VFTOPF_MSK_17TO32(vfMsk);
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK5, temp_reg);
    }

    return;
}

/*
 * adf_dh895xcc_enableVf2PfInterrupts
 * This function enables VF to PF interrupts based on the vfMsk
 * e.g If we want to enable VF to PF interrupts for all 32 VFs a
 * vfMsk of 0xFFFFFFFF should be passed in.
 */
 STATIC void
adf_dh895xcc_enableVf2PfInterrupts(icp_accel_dev_t *accel_dev, Cpa32U vfMsk)
{
    void *pmisc_bar_addr = NULL;
    Cpa32U temp_reg = 0;

    ICP_CHECK_FOR_NULL_PARAM_VOID(accel_dev);

    pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);

    /* Enable VF to PF interrupts for VFs 1 to 16 */
    if (vfMsk & ICP_DH895xCC_VFTOPF_MSK_16BITS)
    {
        temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK3) &
            ~(ICP_DH895xCC_VFTOPF_MSK_1TO16(vfMsk));
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK3, temp_reg);
    }

    /* Enable VF to PF interrupts for VFs 17 to 32 */
    if (vfMsk >> ICP_DH895xCC_VFTOPF_SHIFT_17TO32)
    {
        temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK5) &
            ~(ICP_DH895xCC_VFTOPF_MSK_17TO32(vfMsk));
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK5, temp_reg);
    }

    return;
}

/*
 * adf_dh895xcc_getVf2PfIntSrcMask
 * This function returns a mask based on the VFs
 * that triggered the interrupt
 */
STATIC Cpa32U
adf_dh895xcc_getVf2PfIntSrcMask(icp_accel_dev_t *accel_dev)
{
    void *pmisc_bar_addr = NULL;
    Cpa32U temp_reg = 0;
    Cpa32U vfIntMsk = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);

    /* Determine VFs that triggered the interrupt, VFs 1 to 16 */
    temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRSOU3) &
            ICP_DH895xCC_VFTOPF_SRC_1TO16;
    /* Re-map VF 1 to 16 bits from bit-offset 9 in ERRSOU3 to bit-offset
     * 0 in vfIntMsk */
    vfIntMsk = temp_reg >> ICP_DH895xCC_VFTOPF_BIT_1TO16;

    /* Determine VFs that triggered the interrupt, VFs 17 to 32 */
    temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRSOU5) &
            ICP_DH895xCC_VFTOPF_SRC_17TO32;
    /* Re-map VF 17 to 32 bits from bit-offset 0 in ERRSOU5 to bit-offset
     * 16 in vfIntMsk */
    vfIntMsk |= temp_reg << ICP_DH895xCC_VFTOPF_SHIFT_17TO32;

    ADF_DEBUG("VF Interrupt sources mask for dev%d: 0x%.8X\n",
                                    accel_dev->accelId, vfIntMsk);

    return vfIntMsk;
}

/*
 * getTiMiscIntCtlOffset
 * This function returns byte offset of the miscellaneous interrupt
 * control register.
 */
STATIC Cpa32U
adf_dh895xcc_getTiMiscIntCtlOffset(void)
{
    return ICP_PMISCBAR_TIMISCINTCTL;
}

/*
 * getVIntMskOffset
 * This function returns byte offset of the VF interrupt mask CSR.
 */
STATIC Cpa32U
adf_dh895xcc_getVIntMskOffset(Cpa32U id)
{
    return (ICP_PMISCBAR_VINTMSK_OFFSET + (id * ICP_PMISCBAR_VFDEVOFFSET));
}

/*
 * getVIntMskDefault
 * This function returns the default VF interrupt mask.
 */
STATIC Cpa32U
adf_dh895xcc_getVIntMskDefault(void)
{
    return ICP_PMISCBAR_VINTMSK_DEFAULT;
}

/*
 * getPf2VfDbIntOffset
 * This function returns byte offset of the doorbell and interrupt CSR
 */
STATIC Cpa32U
adf_dh895xcc_getPf2VfDbIntOffset(Cpa32U id)
{
    return (ICP_PMISCBAR_PF2VFDBINT_OFFSET + (id * ICP_PMISCBAR_VFDEVOFFSET));
}

/*
 * getResetOffsets
 * This function returns the offset to the reset vector doe the DH895 device.
 */
STATIC void 
adf_dh895xcc_getResetOffsets(Cpa32U *resetOffset, Cpa32U *resetVector)
{
    if (NULL != resetOffset) {
        *resetOffset = ICP_DH895xCC_RESET_OFFSET;
    }
    if (NULL != resetVector) {
        *resetVector = ICP_DH895xCC_RESET_VECTOR;
    }
}

/*
 * writeCsrSrcSel
 * This function configures the source for flag interrupts.
 */
STATIC void
adf_dh895xcc_writeCsrSrcSel(Cpa32U *csr_base_addr, Cpa32U bank_offset,
               Cpa32U *irq_src_val_low, Cpa32U *irq_src_val_hi)
{
    write_csr_srcsel(csr_base_addr, bank_offset);
    *irq_src_val_low = ICP_DH895xCC_BANK_INT_SRC_SEL_MASK;
    *irq_src_val_hi = ICP_DH895xCC_BANK_INT_SRC_SEL_MASK;
}

/*
 * accessCsrIntFlagAndCol
 * Use the INT, Flag and Coalesced CSR (optomisation for DH895xCC)
 */
STATIC void
adf_dh895xcc_accessCsrIntFlagAndCol(Cpa32U *csr_base_addr, Cpa32U bank_offset,
                       Cpa32U value, CpaBoolean write)
{
    access_csr_int_flag_and_col(csr_base_addr, bank_offset, value, write);
}

/*
 * adf_isr_logAE_Int
 * Log AEs based UErrors
 */
STATIC inline void
adf_dh895xcc_adf_isr_LogAE_Int(uint8_t index, void *pmisc_bar_addr)
{
    Cpa32U ustore = ICP_ADF_CSR_RD(pmisc_bar_addr,
            ICP_DH895xCC_USTORE_ERROR_STATUS(index));
    Cpa32U reg_error = ICP_ADF_CSR_RD(pmisc_bar_addr,
            ICP_DH895xCC_REG_ERROR_STATUS(index));
    Cpa32U eperrlog = ICP_ADF_CSR_RD(pmisc_bar_addr,
            ICP_DH895xCC_EPERRLOG);
    Cpa32U ctx_enables = ICP_ADF_CSR_RD(pmisc_bar_addr,
            ICP_DH895xCC_CTX_ENABLES(index));

    ADF_DEBUG("USTORE_ERROR_STATUS = 0x%.8X\n", ustore);
    ADF_DEBUG("REG_ERROR_STATUS    = 0x%.8X\n", reg_error);
    ADF_DEBUG("EPERRLOG            = 0x%.8X\n", eperrlog);
    ADF_DEBUG("CTX_ENABLES         = 0x%.8X\n", ctx_enables);

    ADF_ERROR("AE #%x Error\n"
            "Uncorrectable Error: %s\n"
            "Control Store Error: %s\n"
            "Control Store information: Address 0x%X - syndrome 0x%X\n"
            "Register ECC Error %s - Parity Error: %s\n"
            "Register information: address 0x%X - type %s\n",
            index,
            ICP_DH895xCC_EPERRLOG_ERR(
                ICP_DH895xCC_GET_EPERRLOG(eperrlog, index)),
                ICP_DH895xCC_USTORE_IS_UE(ustore),
                ICP_DH895xCC_USTORE_UADDR(ustore),
                ICP_DH895xCC_USTORE_GET_SYN(ustore),
                ICP_DH895xCC_CTX_ECC_ERR(ctx_enables),
                ICP_DH895xCC_CTX_PAR_ERR(ctx_enables),
                ICP_DH895xCC_REG_GET_REG_AD(reg_error),
                ICP_DH895xCC_REG_TYP_STR(ICP_DH895xCC_REG_TYPE(reg_error)));
}

/*
 * adf_isr_logSSM_Int
 * Log SSM based Uerrors (shared Mem, MMP and Push/Pull)
 */
STATIC inline CpaBoolean
adf_dh895xcc_adf_isr_LogSSM_Int(void *pmisc_bar_addr,
        Cpa8U cpm, Cpa8U extra, Cpa8U type)
{
    Cpa32U uerrssm = 0, uerrssmad = 0;
    Cpa32U uerrssm_off = 0, uerrssmad_off = 0;

    if (ICP_DH895xCC_PP_ERR == type) {
        Cpa32U pperr = 0, pperrid = 0;
        pperr = ICP_ADF_CSR_RD(pmisc_bar_addr,
                ICP_DH895xCC_PPERR(cpm));
        pperrid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                ICP_DH895xCC_PERRID(cpm));
        ADF_DEBUG("pperr 0x%X pperrid 0x%X\n", pperr, pperrid);
        ADF_ERROR("Push/Pull Error on CPM %i\n"
                "%s - Error type: %s\n"
                "id 0x%X\n",
                cpm, ICP_DH895xCC_PPERR_GET_STATUS(pperr),
                ICP_DH895xCC_PPERR_TYPE_STR(
                    ICP_DH895xCC_PPERR_TYPE(pperr)),
                pperrid);
        return CPA_TRUE;
    }
    else if (ICP_DH895xCC_SHAREDMEM_ERR == type) {
            uerrssm_off = ICP_DH895xCC_UERRSSMSH(cpm);
            uerrssmad_off = ICP_DH895xCC_UERRSSMSHAD(cpm);
    }
    else {
        /* MMP */
        switch(extra) {
        case 4:
            {
                uerrssm_off = ICP_DH895xCC_UERRSSMMMP4(cpm);
                uerrssmad_off = ICP_DH895xCC_UERRSSMMMP4AD(cpm);
                break;
            }
        case 3:
            {
                uerrssm_off = ICP_DH895xCC_UERRSSMMMP3(cpm);
                uerrssmad_off = ICP_DH895xCC_UERRSSMMMP3AD(cpm);
                break;
            }
        case 2:
            {
                uerrssm_off = ICP_DH895xCC_UERRSSMMMP2(cpm);
                uerrssmad_off = ICP_DH895xCC_UERRSSMMMP2AD(cpm);
                break;
            }
        case 1:
            {
                uerrssm_off = ICP_DH895xCC_UERRSSMMMP1(cpm);
                uerrssmad_off = ICP_DH895xCC_UERRSSMMMP1AD(cpm);
                break;
            }
        case 0:
        default:
            {
                uerrssm_off = ICP_DH895xCC_UERRSSMMMP0(cpm);
                uerrssmad_off = ICP_DH895xCC_UERRSSMMMP0AD(cpm);
                break;
            }
        }
    }

    uerrssm = ICP_ADF_CSR_RD(pmisc_bar_addr,
            uerrssm_off);
    uerrssmad = ICP_ADF_CSR_RD(pmisc_bar_addr,
            uerrssmad_off);

    if (ICP_DH895xCC_MMP_ECC_ERR == type) {
        /* MMP */
        ADF_DEBUG("uerrssmmmp 0x%X, uerrssmmmpad 0x%X\n",
                uerrssm, uerrssmad);
        ADF_ERROR("MMP %i CPM %i - Uncorrectable Error: %s\n"
                "Operation: %s -Address: 0x%X\n",
                extra, cpm,
                (((uerrssm & ICP_DH895xCC_UERRSSMMMP_UERR))!= 0 ? "Yes": "No"),
                ICP_DH895xCC_UERRSSMMMP_ERRTYPE_STR(
                    ICP_DH895xCC_UERRSSMMMP_ERRTYPE(uerrssm)),
                (uerrssmad & ICP_DH895xCC_UERRSSMMMPAD_ADDR));
    }
    else {
        ADF_DEBUG("uerrssmsh 0x%X, uerrssmshad 0x%X\n",
                uerrssm, uerrssmad);
        ADF_ERROR("SharedMem Error CPM %x - Uncorrectable Error: %s\n"
                  "Operation: %s - Type: %s - Address: 0x%X\n",
                  cpm,
                  (uerrssm & ICP_DH895xCC_UERRSSMSH_UERR) != 0? "Yes": "No",
                  ICP_DH895xCC_UERRSSMSH_GET_OP(uerrssm),
                  ICP_DH895xCC_UERRSSMSH_GET_ERRTYPE(
                  ICP_DH895xCC_UERRSSMSH_ERRTYPE(uerrssm)),
                  (uerrssmad & ICP_DH895xCC_UERRSSMSHAD_ADDR));
    }
    return (uerrssm & ICP_DH895xCC_UERRSSMMMP_UERR)!= 0 ? CPA_TRUE : CPA_FALSE;
}

/*
 * adf_isr_handleAEInterrrupt
 * Handle Ae interrupt sources: Uncorrectable errors and firmware custom
 * This function is executed in the Top Half of the ISR and doesn't need
 * to be performant since it is called only when a MISC interrupt
 * is triggered.
 * It detects if a reset is needed looking at the ERRSOU registers
 * and all the error specific registers.
 * If an uncorrectable error is detected, it writes to logs and returns
 * TRUE (device reset needed). Otherwise it just returns FALSE
 * and exits quickly (case of spurious interrupt).
 */
STATIC CpaBoolean
adf_dh895xcc_adf_isr_handleUncoInterrupt(icp_accel_dev_t *accel_dev)
{
    CpaBoolean reset_needed = CPA_FALSE;
    void *pmisc_bar_addr = NULL;
    Cpa32U errsou0, errsou1, errsou3, errsou4, errsou5 = 0;

    pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);

    errsou0 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRSOU0);
    errsou1 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRSOU1);
    errsou3 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRSOU3);
    errsou4 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRSOU4);
    errsou5 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRSOU5);

    /* UERR interrupts */
    if ((0 != errsou0) || (0 != errsou1) || (0 != errsou3) ||
                          (0 != errsou4) || (0 != errsou5)) {
        /* Identify the interrupt Sources and log them */
        /* AE0-3 Errors */
        if (0 != errsou0) {
            if (errsou0 & ICP_DH895xCC_M0UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE0,
                                               pmisc_bar_addr);
            }
            if (errsou0 & ICP_DH895xCC_M1UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE1,
                                               pmisc_bar_addr);
            }
            if (errsou0 & ICP_DH895xCC_M2UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE2,
                                               pmisc_bar_addr);
            }
            if (errsou0 & ICP_DH895xCC_M3UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE3,
                                               pmisc_bar_addr);
            }
        }
        /* AE4-7 Errors */
        if (0 != errsou1) {
            if (errsou1 & ICP_DH895xCC_M4UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE4,
                                               pmisc_bar_addr);
            }
            if (errsou1 & ICP_DH895xCC_M5UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE5,
                                               pmisc_bar_addr);
            }
            if (errsou1 & ICP_DH895xCC_M6UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE6,
                                               pmisc_bar_addr);
            }
            if (errsou1 & ICP_DH895xCC_M7UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE7,
                                               pmisc_bar_addr);
            }
        }
        /* AE8-11 Errors */
        if (0 != errsou4) {
            if (errsou4 & ICP_DH895xCC_M8UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE8,
                                               pmisc_bar_addr);
            }
            if (errsou4 & ICP_DH895xCC_M9UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE9,
                                               pmisc_bar_addr);
            }
            if (errsou4 & ICP_DH895xCC_M10UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE10,
                                               pmisc_bar_addr);
            }
            if (errsou4 & ICP_DH895xCC_M11UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_dh895xcc_adf_isr_LogAE_Int(ICP_DH895xCC_AE11,
                                               pmisc_bar_addr);
            }
        }

        if ((0 != errsou3) || (0 != errsou5)) {
            /* eSRAM errors */
            if (errsou3 & ICP_DH895xCC_UERR_MASK) {
                Cpa32U  esramuerr = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH895xCC_ESRAMUERR);
                Cpa32U  esramuerrad = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH895xCC_ESRAMUERRAD);
                reset_needed = CPA_TRUE;
                ADF_DEBUG("esramuerr 0x%X - esramuerrad 0x%X\n",
                        esramuerr, esramuerrad);
                ADF_ERROR("Uncorrectable Error Occurred on eSram\n"
                        "Interrupt triggered: %s\n"
                        "Operation type %s - Error Type %s\n"
                        "Address (quad words) 0x%X\n",
                        ICP_DH895xCC_ESRAMUERR_GET_UERR(esramuerr),
                        ICP_DH895xCC_ESRAMUERR_GET_OP(esramuerr),
                        ICP_DH895xCC_ESRAMUERR_GET_ERRSTR(
                                ICP_DH895xCC_ESRAMUERR_GET_ERRTYPE(esramuerr)),
                                esramuerrad);
            }
            /* CPM: MMP, SH, PUSH/PULL errors */
            if ((errsou3 & ICP_DH895xCC_EMSK3_CPM0_MASK) ||
                    (errsou3 & ICP_DH895xCC_EMSK3_CPM1_MASK ) ||
                    (errsou5 & ICP_DH895xCC_EMSK5_CPM2_MASK ) ||
                    (errsou5 & ICP_DH895xCC_EMSK5_CPM3_MASK ) ||
                    (errsou5 & ICP_DH895xCC_EMSK5_CPM4_MASK ) ||
                    (errsou5 & ICP_DH895xCC_EMSK5_CPM5_MASK ))
            {
                Cpa8U cpm = 0;
                Cpa32U instatssm = 0;
                if (errsou3 & ICP_DH895xCC_EMSK3_CPM0_MASK) {
                    cpm = 0;
                }
                else if (errsou3 & ICP_DH895xCC_EMSK3_CPM1_MASK){
                    cpm = 1;
                }
                else if (errsou5 & ICP_DH895xCC_EMSK5_CPM2_MASK){
                    cpm = 2;
                }
                else if (errsou5 & ICP_DH895xCC_EMSK5_CPM3_MASK){
                    cpm = 3;
                }
                else if (errsou5 & ICP_DH895xCC_EMSK5_CPM4_MASK){
                    cpm = 4;
                }
                else if (errsou5 & ICP_DH895xCC_EMSK5_CPM5_MASK){
                    cpm = 5;
                }
                else {
                    /* It is possible to hit this else only in the
                     * case of spurious interrupt.
                     * Here it is not possible to clear the
                     * interrupt since we don't know what caused it
                     */
                    return CPA_FALSE;
                }

                instatssm = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH895xCC_INTSTATSSM(cpm));

                ADF_ERROR("Uncorrectable Error Occurred on CPM %i\n", cpm);
                ADF_DEBUG("instatssm 0x%X\n", instatssm);
                if (instatssm & ICP_DH895xCC_INTSTATSSM_MMP0) {
                    reset_needed =
                        adf_dh895xcc_adf_isr_LogSSM_Int(pmisc_bar_addr, cpm,
                            ICP_DH895xCC_MMP_SLICE0, ICP_DH895xCC_MMP_ECC_ERR);
                }
                if (instatssm & ICP_DH895xCC_INTSTATSSM_MMP1) {
                    reset_needed =
                        adf_dh895xcc_adf_isr_LogSSM_Int(pmisc_bar_addr, cpm,
                            ICP_DH895xCC_MMP_SLICE1,ICP_DH895xCC_MMP_ECC_ERR);
                }
                if (instatssm & ICP_DH895xCC_INTSTATSSM_MMP2) {
                    reset_needed =
                        adf_dh895xcc_adf_isr_LogSSM_Int(pmisc_bar_addr, cpm,
                            ICP_DH895xCC_MMP_SLICE2, ICP_DH895xCC_MMP_ECC_ERR);
                }
                if (instatssm & ICP_DH895xCC_INTSTATSSM_MMP3) {
                    reset_needed =
                        adf_dh895xcc_adf_isr_LogSSM_Int(pmisc_bar_addr, cpm,
                            ICP_DH895xCC_MMP_SLICE3, ICP_DH895xCC_MMP_ECC_ERR);
                }
                if (instatssm & ICP_DH895xCC_INTSTATSSM_MMP4) {
                    reset_needed =
                        adf_dh895xcc_adf_isr_LogSSM_Int(pmisc_bar_addr, cpm,
                            ICP_DH895xCC_MMP_SLICE4, ICP_DH895xCC_MMP_ECC_ERR);
                }
                if (instatssm & ICP_DH895xCC_INTSTATSSM_SH) {
                    reset_needed =
                        adf_dh895xcc_adf_isr_LogSSM_Int(pmisc_bar_addr, cpm,
                            ICP_DH895xCC_MMP_SLICE0/*not used*/,
                            ICP_DH895xCC_SHAREDMEM_ERR);
                }
                if (instatssm & ICP_DH895xCC_INTSTATSSM_PPERR) {
                    reset_needed =
                        adf_dh895xcc_adf_isr_LogSSM_Int(pmisc_bar_addr, cpm,
                            ICP_DH895xCC_MMP_SLICE0/*not used*/,
                            ICP_DH895xCC_PP_ERR);
                }
            }
            /* Or'ed Shac Data Error */
            if (errsou3 & ICP_DH895xCC_EMSK3_SHaC0_MASK) {
                Cpa32U err_status = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH895xCC_CPP_SHAC_ERR_STATUS);
                Cpa32U err_ppid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH895xCC_CPP_SHAC_ERR_PPID);
                reset_needed = CPA_TRUE;
                /*Log error*/
                ADF_DEBUG("ccp_shac_err_status 0x%X -"
                        "cpp_shac_err_ppid 0x%X\n", err_status, err_ppid);
                ADF_ERROR("Uncorrectable Data Error Occurred in SHaC\n"
                        "Operation type: %s, Error target %s\n"
                        "Error ppid 0x%X\nInterrupt Triggered %s\n",
                        ICP_DH895xCC_CPP_SHAC_GET_TYP(err_status) == 0 ?
                                "Push": "Pull",
                                ICP_DH895xCC_CPP_SHAC_ERTYP_STR(
                                    ICP_DH895xCC_CPP_SHAC_GET_ERTYP(err_status)),
                                    err_ppid,
                                    ICP_DH895xCC_CPP_SHAC_GET_INT(err_status) == 1
                                        ? "Yes": "No");
            }
            /* Push/Pull Misc */
            if (errsou3 & ICP_DH895xCC_PPMISCERR_MASK) {
                Cpa32U cpptgterr = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH895xCC_CPPMEMTGTERR);
                Cpa32U errppid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH895xCC_ERRPPID);
                Cpa32U ticppintsts = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH895xCC_TICPPINTSTS);
                Cpa32U tierrid = 0;
                reset_needed = CPA_TRUE;
                if(ticppintsts & ICP_DH895xCC_TICPP_PUSH) {
                    tierrid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                            ICP_DH895xCC_TIERRPUSHID);
                }
                else {
                    tierrid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                            ICP_DH895xCC_TIERRPULLID);
                }
                ADF_DEBUG("cpptgterr 0x%X, errppid 0x%X, ticppintsts 0%X,"
                        "tierrxxxid 0x%X\n", cpptgterr, errppid,
                        ticppintsts, tierrid);
                ADF_ERROR("Uncorrectable Push/Pull Misc Error\n"
                        "memory status: %s - Transaction Id 0x%X - "
                        "Error type %s\n"
                        "Bus Operation Type %s - Id 0x%X\n",
                        ICP_DH895xCC_CPPMEMTGTERR_GET_STATUS(cpptgterr),
                        errppid,
                        ICP_DH895xCC_CPPMEMTGTERR_ERRTYP_STR(
                                ICP_DH895xCC_CPPMEMTGTERR_ERRTYP(cpptgterr)),
                                ICP_DH895xCC_TICPP_GETOP(ticppintsts), tierrid);
            }

        } /* ERRSOU3/ERRSOU5 */


        if (reset_needed) {
            /* Mask off all fatal error sources. These will be re-enabled
             * on device reset. These need masking here so pfvfcomms
             * interrupts can still be handled even
             * if reset is not done immediately*/
            Cpa32U temp_reg = 0;

            temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK0) |
                    ICP_DH895xCC_ERRMSK0_MASK_FATAL_ERRS;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK0, temp_reg);
            temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK1) |
                    ICP_DH895xCC_ERRMSK1_MASK_FATAL_ERRS;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK1, temp_reg);
            temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK3) |
                    ICP_DH895xCC_ERRMSK3_MASK_FATAL_ERRS;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK3, temp_reg);
            temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK4) |
                    ICP_DH895xCC_ERRMSK4_MASK_FATAL_ERRS;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK4, temp_reg);
            temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_ERRMSK5) |
                    ICP_DH895xCC_ERRMSK5_MASK_FATAL_ERRS;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK5, temp_reg);

        }

        return reset_needed;
    }

    return CPA_FALSE;
}

/*
 * adf_isr_enable_unco_err_interrupts
 * Enable All the supported Uncorrectable Error sources and
 * their logging/debug abilities
 */
STATIC CpaStatus
adf_dh895xcc_adf_enable_unco_err_interrupts(icp_accel_dev_t *accel_dev)
{
    Cpa8U i = 0;
    void *pmisc_bar_addr = NULL;
    volatile Cpa32U val = 0;
    volatile Cpa32U temp_reg_mmp = 0;
    adf_hw_device_data_t *hw_data = NULL;
    Cpa32U numAccelEngines = 0;
    Cpa32U numAccelerators = 0;

    /*
     * Parameter check
     */
    if (NULL == accel_dev)
    {
        ADF_ERROR("accel_dev is NULL\n");
        return CPA_STATUS_FAIL;
    }

    /*
     * Get hw_data structure
     */
    hw_data = accel_dev->pHwDeviceData;
    if (NULL == hw_data)
    {
        ADF_ERROR("hw_data is NULL\n");
        return CPA_STATUS_FAIL;
    }

    /*
     * Retrieve the pmiscbar address
     */
    pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);

    /*
     * Get number of accelerators engines
     */
    numAccelEngines = hw_data->getNumAccelEngines(hw_data, accel_dev->aeMask);

    /*
     * Get number of accelerators
     */
    numAccelerators = hw_data->getNumAccelerators(hw_data, accel_dev->accelMask);

    /*
     * Setup interrupt mask
     */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK0,
            ICP_DH895xCC_ERRMSK0_UERR); /* AE0-AE3  */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK1,
            ICP_DH895xCC_ERRMSK1_UERR); /* AE4-AE7  */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK4,
            ICP_DH895xCC_ERRMSK4_UERR); /* AE8-AE11 */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK3,
            ICP_DH895xCC_ERRMSK3_UERR); /* MISC, CPM0-1 */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ERRMSK5,
            ICP_DH895xCC_ERRMSK5_UERR); /* CPM1-2 */

    /*
     * Setup Error Sources
     */
    for(i = 0; i < numAccelEngines; i++)
    {
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CTX_ENABLES(i),
                ICP_DH895xCC_ENABLE_ECC_ERR); /* AEi */

        /*@note: this code enable ECC and parity AE hw correction.
         * The driver is enabling them for the FW/HW but the handling of those
         * correction is not covered by the driver*/
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_PMISCBAR_MISC_CONTROL(i),
            ICP_DH895xCC_ENABLE_ECC_PARITY_CORR); /*AEi*/
    }

    for(i = 0; i < numAccelerators; i++)
    {
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_INTMASKSSM(i),
                ICP_DH895xCC_INTMASKSSM_UERR); /* CPMi */
    }

    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CPP_SHAC_ERR_CTRL,
            ICP_DH895xCC_CPP_SHAC_UE); /* SHaC */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ESRAMUERR,
            ICP_DH895xCC_ESRAM_UERR); /* eSRAM */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_ESRAMCERR,
            ICP_DH895xCC_ESRAM_CERR); /* eSRAM ecc Correction*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CPPMEMTGTERR,
            ICP_DH895xCC_TGT_UERR); /* Push/Pull Misc Error */

    /* Check if signal misc errors are set and interrupts routed to IA */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_SMIAPF1_MASK_OFFSET,
            ICP_DH895xCC_SMIA1_MASK);

    /*
     * Enable MMP Logging
     */
    for(i = 0; i < numAccelerators; i++)
    {
        /* Set power-up */
        temp_reg_mmp =
            ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_SLICEPWRDOWN(i));
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_SLICEPWRDOWN(i),
            temp_reg_mmp & ICP_DH895xCC_MMP_PWRUPMSK);

        if (accel_dev->accelCapabilitiesMask &
                ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC)
        {
            /* The device supports PKE so enable error reporting
             * from MMP memory */
            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP0(i)) |
                    ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP0(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP1(i)) |
                    ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP1(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP2(i)) |
                    ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP2(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP3(i)) |
                    ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP3(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP4(i)) |
                    ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP4(i), val);

            /* The device supports PKE so enable error correction
             * from MMP memory */
            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP0(i)) |
                    ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP0(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP1(i)) |
                    ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP1(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP2(i)) |
                    ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP2(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP3(i)) |
                    ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP3(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP4(i)) |
                    ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP4(i), val);
        }
        else
        {
            /* The device doesn't support PKE so disable error reporting
             * from MMP memory */
            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP0(i));
            val &= ~ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP0(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP1(i));
            val &= ~ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP1(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP2(i));
            val &= ~ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP2(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP3(i));
            val &= ~ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP3(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP4(i));
            val &= ~ICP_DH895xCC_UERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMMMP4(i), val);

            /* The device doesn't support PKE so disable error correction
             * from MMP memory */
            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP0(i));
            val &= ~ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP0(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP1(i));
            val &= ~ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP1(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP2(i));
            val &= ~ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP2(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP3(i));
            val &= ~ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP3(i), val);

            val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP4(i));
            val &= ~ICP_DH895xCC_CERRSSMMMP_EN;
            ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMMMP4(i), val);
        }

        /* Restore power-down value */
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_SLICEPWRDOWN(i), temp_reg_mmp);
    }

    /*
     * Enable SRAM and PPERR Logging
     */
    for(i = 0; i < numAccelerators; i++)
    {

        /* Shared Memory */
        val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_UERRSSMSH(i)) |
            ICP_DH895xCC_UERRSSMSH_EN;
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_UERRSSMSH(i), val);
        /* Shared Memory ecc correction*/
        val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_CERRSSMSH(i)) |
            ICP_DH895xCC_CERRSSMSH_EN;
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_CERRSSMSH(i), val);

        /* Push/Pull CPM */
        val = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH895xCC_PPERR(i)) |
            ICP_DH895xCC_PPERR_EN;
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_PPERR(i), val);
    }

    /*Push/Pull Misc*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH895xCC_TICPPINTCTL,
            ICP_DH895xCC_TICPP_EN);

    return CPA_STATUS_SUCCESS;
}

/*
 * adf_dh895xcc_PF2VF_putMsg
 * Called in kernel space to send a message from the PF to the VF.
 * This can be either a system msg from kernel driver or a user msg via ioctl.
 * The mutex is used to ensure that only one thread on this side can
 * put a msg on the CSR.
 * The collision detection is used to ensure that a message on this side
 * doesn't clash with a msg coming in from the opposite side.
 * We wait for confirmation that msg has been received on other side before
 * returning success.
 */
STATIC CpaStatus
    adf_dh895xcc_PF2VF_putMsg(icp_accel_pci_vf_info_t *vf, Cpa32U msgIn)
{
    void *pmisc_bar_addr = NULL;
    Cpa32U  val = 0, ctr = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_accel_dev_t *accel_dev = NULL;
    adf_hw_device_data_t  *hw_data = NULL;
    Cpa32U doorbell_csr_offset = 0;
    Cpa32U msg = msgIn;
    ADF_DEBUG("Msg 0x%x",msgIn);

    ICP_CHECK_FOR_NULL_PARAM(vf);
    accel_dev = vf->accel_dev;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    hw_data = accel_dev->pHwDeviceData;
    ICP_CHECK_FOR_NULL_PARAM(hw_data);
    pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pmisc_bar_addr);
    ICP_CHECK_FOR_NULL_PARAM(vf->lock);

    doorbell_csr_offset = hw_data->getPf2VfDbIntOffset(vf->deviceId);

    status = ICP_MUTEX_LOCK_TIMEOUT(vf->lock,
                    ICP_PFVFCOMMS_DOORBELL_CSR_MUTEX_WAIT);

    if (CPA_STATUS_SUCCESS == status )
    {
        /* Check if CSR is in use by the other side */
        val = ICP_ADF_CSR_RD(pmisc_bar_addr, doorbell_csr_offset);
        if (adf_pfvfcomms_isCSRInUseByVF(val))
        {
            ADF_DEBUG("CSR for PFVF comms on Dev%d VF%d is in use by guest\n",
                    accel_dev->accelId, vf->deviceId+1);
            status = CPA_STATUS_RESOURCE;
            ICP_MUTEX_UNLOCK(vf->lock);
        }

        /* No-one is using the CSR, so go ahead and use it */
        if (CPA_STATUS_SUCCESS == status)
        {
            adf_pfvfcomms_setPFInUse(&msg);
            ADF_DEBUG("writing to doorbell CSR for Device%d VF%d: 0x%08x\n",
                                     accel_dev->accelId, vf->deviceId+1, msg);
            ICP_ADF_CSR_WR(pmisc_bar_addr, doorbell_csr_offset, msg);

            /* Now wait in case other side tried to use at same time */
            ICP_MSLEEP(ICP_PFVFCOMMS_COLLISION_DETECTION_DELAY);

            /* collision detection */
            val = ICP_ADF_CSR_RD(pmisc_bar_addr, doorbell_csr_offset);
            if (CPA_FALSE == adf_pfvfcomms_isCSRInUseByPF(val))
            {
                /* Didn't read back what we just wrote! */
                ADF_DEBUG("Collision detected. CSR for PFVF comms "
                        "on Dev%d VF%d is in use by guest\n",
                        accel_dev->accelId, vf->deviceId+1);
                   status = CPA_STATUS_RESOURCE;
                   ICP_MUTEX_UNLOCK(vf->lock);
            }
            else
               {
                /* Now we own the CSR :) */
                /* The PF_IN_USE pattern must remain in CSR for all writes
                 * including ack from other side, until we relinquish the CSR*/
                adf_pf2vf_set_irq(&msg);
                ADF_DEBUG("writing to doorbell CSR for Dev%d vf%d: 0x%08x\n",
                                  accel_dev->accelId, vf->deviceId+1, msg);
                ICP_ADF_CSR_WR(pmisc_bar_addr, doorbell_csr_offset, msg);

                /* And wait for confirmation that it picked off the msg */
                do{
                    ICP_MSLEEP(ICP_PFVFCOMMS_CONFIRMATION_SLEEP_TIME);
                    val = ICP_ADF_CSR_RD(pmisc_bar_addr,doorbell_csr_offset);
                } while((ICP_VF2PF_IRQ_CLEARED != (val & ICP_PF2VF_MSGMASK_IRQ))
                        && (ctr++ < ICP_PFVFCOMMS_CONFIRMATION_MAX_LOOPS));

                /* Return success only if we got ACK from VF */
                status = (ICP_VF2PF_IRQ_CLEARED == (val & ICP_PF2VF_MSGMASK_IRQ))
                    ? CPA_STATUS_SUCCESS : CPA_STATUS_RESOURCE;

                if(CPA_STATUS_SUCCESS != status)
                {
                    ADF_DEBUG("Ack not received from guest on Dev%d VF%d\n",
                            accel_dev->accelId, vf->deviceId+1);
                    /* Clear the irq bit as the VF hasn't done this */
                    adf_vf2pf_ack(&val);
                }

                /* Finished with CSR. Relinquish it.
                 * Leave msg in CSR for debug */
                msg = val;
                adf_pfvfcomms_clearPFInUse(&msg);
                ADF_DEBUG("writing to doorbell CSR for Dev%d Vf%d: 0x%08x\n",
                        accel_dev->accelId, vf->deviceId+1, msg );
                ICP_ADF_CSR_WR(pmisc_bar_addr,doorbell_csr_offset, msg);
                ICP_MUTEX_UNLOCK(vf->lock);
            }
        }
    }
    else
    {
        ADF_DEBUG("CSR for Dev%d VF%d already in use by another thread on PF,"
                        " could not get access\n",
                        accel_dev->accelId, vf->deviceId+1);
        status = CPA_STATUS_RESOURCE;
    }

    ADF_DEBUG("%s: to Dev%d VF%d. Status=%d\n",__FUNCTION__,
                        accel_dev->accelId,vf->deviceId+1,status);
    return status;
}

/*
 * adf_dh895xcc_systemMsg_PF2VF_putMsg
 * Called by the system (kernel space) to send a message
 * from the PF to the VF
 */
STATIC CpaStatus adf_dh895xcc_systemMsg_PF2VF_putMsg
                    (icp_accel_pci_vf_info_t *vf, Cpa32U msg )
{
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32U cntr = 0;

    do{
        status = adf_dh895xcc_PF2VF_putMsg(vf, msg );
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
adf_set_hw_data_dh895xcc(void *hw_data)
{

    adf_hw_device_data_t *hw = NULL;

    ICP_CHECK_FOR_NULL_PARAM_VOID(hw_data);

    hw = (adf_hw_device_data_t *)hw_data;

    hw->init                  = adf_dh895xcc_init;
    hw->cleanup               = adf_dh895xcc_cleanup;
    hw->adf_addFwMsgSizesToCfgTable = adf_dh895xcc_adf_addFwMsgSizesToCfgTable;

    hw->dev_class = &dh895xcc_class;

    /* populate bank/ac information */
    hw->maxBars               = ICP_DH895xCC_MAX_PCI_BARS;
    hw->maxNumBanks           = ICP_DH895xCC_ETR_MAX_BANKS;
    hw->maxNumApBanks         = ICP_DH895xCC_ETR_MAX_AP_BANKS;
    hw->numRingsPerBank       = ICP_DH895xCC_ETR_MAX_RINGS_PER_BANK;
    hw->numBanksPerAccel      = ICP_DH895xCC_BANKS_PER_ACCELERATOR;
    hw->maxNumAccel           = ICP_DH895xCC_MAX_ACCELERATORS;
    hw->maxNumAccelEngines    = ICP_DH895xCC_MAX_ACCELENGINES;
    hw->aeClockInMhz          = ICP_DH895xCC_AE_CLOCK_IN_MHZ;
    hw->sintpfOffset          = ICP_DH895xCC_SINTPF0_OFFSET;
    hw->sintpf1Offset         = ICP_DH895xCC_SINTPF1_OFFSET;
    hw->userEtringCsrSize     = ICP_DH895xCC_USER_ETRING_CSR_SIZE;
    hw->maxNumVf              = ICP_DH895xCC_MAX_NUM_VF;
    hw->bankSize              = ICP_DH895xCC_BUNDLE_SIZE;
    hw->maxNumRingsPerAccelerator = ICP_DH895xCC_MAX_NUM_RINGS_PER_ACCELERATOR;

    /* populate msix information */
    hw->msix.banksVectorStart = ICP_DH895xCC_MSIX_BANK_VECTOR_START;
    hw->msix.banksVectorNum   = ICP_DH895xCC_MSIX_BANK_VECTOR_NUM;
    hw->msix.aeVectorStart    = ICP_DH895xCC_MSIX_AE_VECTOR_START;
    hw->msix.aeVectorNum      = ICP_DH895xCC_MSIX_AE_VECTOR_NUM;
    hw->isMemScrubSupported   = CPA_TRUE;

    /* device fuse */
    hw->getDevFuseOffset      = adf_dh895xcc_getDevFuseOffset;
    hw->getDevClkOffset       = NULL;
    hw->getDevSKU             = adf_dh895xcc_getDevSKU;
    hw->getAcceleratorsMask   = adf_dh895xcc_getAcceleratorsMask;
    hw->getAccelEnginesMask   = adf_dh895xcc_getAccelEnginesMask;
    hw->getNumAccelerators    = adf_dh895xcc_getNumAccelerators;
    hw->getNumAccelEngines    = adf_dh895xcc_getNumAccelEngines;
    hw->getAccelMaskList      = adf_dh895xcc_getAccelMaskList;

    /* Hardware revision */
    hw->getHardwareVersion    = adf_dh895xcc_getHardwareVersion;

    /* sleep times */
    hw->msleep_time           = ICP_DH895XCC_MSLEEP_TIME;
    hw->msleep_trip_time      = ICP_DH895XCC_MSLEEP_TRIP_TIME;
    hw->msleep_sbr_time       = ICP_DH895XCC_MSLEEP_SBR_TIME;

    /* accelerator functions */
    hw->getAccFuncOffset      = adf_dh895xcc_getAccFuncOffset;
    hw->getAccFuncMask        = adf_dh895xcc_getAccFuncMask;

    /* populate bars callbacks  */
    hw->getEtrBarId           = adf_dh895xcc_getEtrBarId;
    hw->getSramBarId          = adf_dh895xcc_getSramBarId;
    hw->getMiscBarId          = adf_dh895xcc_getMiscBarId;
    hw->isBarAvailable        = adf_dh895xcc_isBarAvailable;

    /* populate esram callbacks    */
    hw->getEsramInfo          = adf_dh895xcc_getEsramInfo;

    /* populate SHAC callback */
    hw->getScratchRamInfo     = adf_dh895xcc_getScratchRamInfo;

    /* populate PMiscBar callback  */
    hw->setSmiapfOffsetMask   = adf_dh895xcc_setSmiapfOffsetMask;

    hw->irqGetBankNumber      = adf_dh895xcc_irqGetBankNumber;
    hw->irqGetAeSource        = adf_dh895xcc_irqGetAeSource;

    hw->irqGetBunSource       = NULL;
    hw->irqGetPF2VFSource     = NULL;

    hw->getBankBhHandler          = adf_dh895xcc_getBankBhHandler;
    hw->getBankBhPollingHandler   = adf_dh895xcc_getBankBhPollingHandler;

    /*Override src sel CSR write for dh895xcc*/
    hw->writeCsrSrcSel            = adf_dh895xcc_writeCsrSrcSel;

    /* Override the reset vector for the dh895xcc */
    hw->getResetOffsets           = adf_dh895xcc_getResetOffsets;

    /*Additional CRS available for dh895xcc*/
    hw->accessCsrIntFlagAndCol    = adf_dh895xcc_accessCsrIntFlagAndCol;

    /* Auto-push feature callback */
    hw->isAeAutoPushSupported     = CPA_FALSE;
    hw->getAeApCsrOffsets         = NULL;

    hw->isETRlessAdminInitCommSupported = CPA_TRUE;

    hw->initAdminComms            = adf_initAdminComms;
    hw->restoreDevice             = adf_restore_dev;

    hw->arbInit                   = adf_arbInit;
    hw->arbShutdown               = adf_arbShutdown;
    hw->isArbitrationSupported    = CPA_TRUE;

    hw->arbVerifyResponseMsgSize  = adf_arbVerifyResponseMsgSize;

    hw->isGigeWatchdogSupported   = CPA_FALSE;
    hw->isWirelessSupported       = CPA_TRUE;

    hw->isSriovSupported          = CPA_TRUE;
    hw->getVf2PfIntSourceOffset   = adf_dh895xcc_getVf2PfIntSourceOffset;
    hw->getVf2PfBitOffset         = adf_dh895xcc_getVf2PfBitOffset;
    hw->getVf2PfIntMaskOffset     = adf_dh895xcc_getVf2PfIntMaskOffset;
    hw->getVf2PfIntMaskDefault    = adf_dh895xcc_getVf2PfIntMaskDefault;

    hw->disableVf2PfInterrupts    = adf_dh895xcc_disableVf2PfInterrupts;
    hw->enableVf2PfInterrupts     = adf_dh895xcc_enableVf2PfInterrupts;
    hw->getVf2PfIntSrcMask        = adf_dh895xcc_getVf2PfIntSrcMask;

    hw->getTiMiscIntCtlOffset     = adf_dh895xcc_getTiMiscIntCtlOffset;
    hw->getVIntSrcOffset          = NULL;
    hw->getVIntMskOffset          = adf_dh895xcc_getVIntMskOffset;
    hw->getVIntMskDefault         = adf_dh895xcc_getVIntMskDefault;
    hw->getPf2VfDbIntOffset       = adf_dh895xcc_getPf2VfDbIntOffset;

    hw->setShimReqNumBit          = CPA_FALSE;
    hw->enableUncoErrInterrupts   = adf_dh895xcc_adf_enable_unco_err_interrupts;
    hw->handleUncoErrInterrupts   = adf_dh895xcc_adf_isr_handleUncoInterrupt;

    hw->checkForRevisionId        = CPA_FALSE;

    hw->adf_PF2VF_putMsg            = adf_dh895xcc_PF2VF_putMsg;
    hw->adf_systemMsg_PF2VF_putMsg  = adf_dh895xcc_systemMsg_PF2VF_putMsg;
    hw->adf_VF2PF_putMsg            = NULL;
    hw->adf_systemMsg_VF2PF_putMsg  = NULL;

    hw->icp_version.versionMsgAvailable = CPA_FALSE;
    hw->icp_version.device_lowestSupportMajorVersion =
                ICP_DH895xCC_LOWESTSUPPORTED_MAJOR_VERSION;
    hw->icp_version.device_lowestSupportMinorVersion =
                ICP_DH895xCC_LOWESTSUPPORTED_MINOR_VERSION;

}
