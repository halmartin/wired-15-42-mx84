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
 * @file adf_dh89xxcc_hw_data.c
 *
 * @ingroup
 *
 * @description
 *    This file contains the hw device data for DH89xxcc
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_platform_common.h"
#include "adf_platform_dh89xxcc.h"

#define ICP_DH89xxCC_CTX_ECC_ERR(val)     ((val & (1<<29)) == 0 ? "No": "Yes")
#define ICP_DH89xxCC_CTX_PAR_ERR(val)     ((val & (1<<25)) == 0 ? "No": "Yes")
#define ICP_DH89xxCC_EPERRLOG_ERR(val)    ((val  == 0)      ?     "No": "Yes")
#define ICP_DH89xxCC_USTORE_IS_UE(val)    ((val & (1<<31)) == 0 ? "No": "Yes")

/*Mem type mask (00-GPR 01-transfer 10-NextNeighbor 11-Local Mem)*/
#define ICP_DH89xxCC_REG_TYP_STR(val)                  \
    ((val == 0)? "GPR":                                \
    ((val == 1)? "Transfer":                           \
    ((val == 2)? "Next Neighbour" : "Local Memory")))

/*MMP ECC Err TypeMask  0 - MMP program mem 1 - MMP OPA Ram, 2 MMP OPB Ram
 *(3-F rsd)*/
#define ICP_DH89xxCC_UERRSSMMMP_ERRTYPE_STR(val)       \
    ((val == 0)? "MMP program Mem":                    \
    ((val == 1)? "MMP OPA Ram":                        \
    ((val == 2)? "MMP OPB Ram": "Reserved")))

#define ICP_DH89xxCC_UERRSSMSH_GET_OP(val)             \
        ((val& ICP_DH89xxCC_UERRSSMSH_R) != 0? "Read": \
        ((val& ICP_DH89xxCC_UERRSSMSH_W) != 0? "Write": "Error"))

/*Error type 1,4,8-F reserved ECC 0-slice push 2-AE Push command
 * 3-Me Pull Command 5-DRAM Pull 6-CPM Memory read (RMW DRAM  push)
 * 7-CPM bus master read*/
#define ICP_DH89xxCC_UERRSSMSH_GET_ERRTYPE(val)        \
    ((val == 0)? "Slice Push":                         \
    ((val == 2)? "AE Push command":                    \
    ((val == 3)? "AE Pull command":                    \
    ((val == 5)? "DRAM pull":                          \
    ((val == 6)? "CPM Memory read (DRAM  push)":       \
    ((val == 7)? "CPM bus master read" : "Reserved"))))))

#define ICP_DH89xxCC_PPERR_GET_STATUS(val)             \
   (((val & ICP_DH89xxCC_PPERR_PPERR)) != 0?           \
   (((val & ICP_DH89xxCC_PPERR_MERR) != 0)?            \
       "Multiple errors occurred":                     \
       "One error occured"):                           \
       "No errors occurred")

/*Error type 0-data asserted on AE Pull Data 1- data asserted on  Dram Push
 * 2-Master bus error 3-ECC error on Shared mem access to Dram pull data bus*/
#define ICP_DH89xxCC_PPERR_TYPE_STR(val)               \
    ((val == 0)? "Data asserted on AE Pull":           \
    ((val == 1)? "Data asserted on Dram Push":         \
    ((val == 2)? "Master Bus Error":                   \
         "ECC error on Shared mem access to Dram pull data bus")))

#define ICP_DH89xxCC_CPP_SHAC_ERTYP_STR(val)           \
    ((val == 0)? "Scratch":                            \
    ((val == 1)? "Hash":                               \
    ((val == 2)? "CAP - impossible for push": "reserved")))
/*Error type operation 00-scratch 01-hash 10-CAP(not possible for push)
 * 11-reserved*/

#define ICP_DH89xxCC_ESRAMUERR_GET_UERR(val)           \
    ((val & ICP_DH89xxCC_ESRAMUERR_UERR) != 0?         \
        "Yes": "No")

/*eSRAM UERR Type mask 0 -ECC data error 1-F reserved*/
#define ICP_DH89xxCC_ESRAMUERR_GET_ERRSTR(val)         \
    (val == 0? "Ecc Data Error": "Reserved")

#define ICP_DH89xxCC_ESRAMUERR_GET_OP(val)             \
        ((val& ICP_DH89xxCC_ESRAMUERR_R) != 0? "Read": \
        ((val& ICP_DH89xxCC_ESRAMUERR_W) != 0? "Write": "Error"))

/*Error Type 3-P2S data path error 4-SRAM memory read transaction 5-
 * Pull bus during memory write - see EAS 13-221 - 0 to 2- 6 to F - reserved*/
#define ICP_DH89xxCC_CPPMEMTGTERR_ERRTYP_STR(val)      \
    ((val == 4)? "Memory Read Transaction":            \
    ((val == 5)? "Memory write":                       \
    ((val == 3)? "P2S Data Path": "reserved")))

#define ICP_DH89xxCC_CPPMEMTGTERR_GET_STATUS(val)      \
   (((val & ICP_DH89xxCC_CPPMEMTGTERR_ERR)) != 0?      \
   (((val & ICP_DH89xxCC_CPPMEMTGTERR_MERR) != 0)?     \
       "Multiple errors occurred":                     \
       "One error occured"):                           \
       "No errors occurred")

#define ICP_DH89xxCC_TICPP_GETOP(val)                  \
    ((val&ICP_DH89xxCC_TICPP_PUSH) != 0?"Push": "Pull")

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

/*******************************************************************************
* Static Variables
*******************************************************************************/
STATIC adf_hw_device_class_t dh89xxcc_class = {
    .name         = "dh89xxcc"    ,
    .type         = DEV_DH89XXCC  ,
    .numInstances = 0             ,
    .currentInstanceId = 0
};

STATIC bank_handler bh_bank_handlers[] = {
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
};

STATIC bank_handler bh_polling_bank_handlers[] = {
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
};

#define NUM_ELEM(array)                   \
    sizeof (array) / sizeof (*array)

/*
 * getAcceleratorsMask
 * Gets the acceleration mask based on the fuse information
 */
STATIC Cpa32U
getAcceleratorsMask(Cpa32U fuse)
{
    return (((~fuse) >> ICP_DH89xxCC_MAX_ACCELENGINES) &
                                ICP_DH89xxCC_ACCELERATORS_MASK);
}

/*
 * getAccelEnginesMask
 * Gets the accelerator engine mask based on the fuse information
 */
STATIC Cpa32U
getAccelEnginesMask(Cpa32U fuse)
{
    return ((~fuse) & ICP_DH89xxCC_ACCELENGINES_MASK);
}

/*
 * getDevFuseOffset
 * Function returns the device fuse control offset
 */
STATIC Cpa32U
getDevFuseOffset(void)
{
    return ICP_DH89xxCC_FUSECTL_OFFSET;
}

/*
 * getDevClkOffset
 * Function returns the device clock control offset
 */
STATIC Cpa32U
getDevClkOffset(void)
{
    return ICP_DH89xxCC_CLKCTL_OFFSET;
}

/*
 * getDevSKU
 * Function returns device SKU info
 */
STATIC dev_sku_info_t
getDevSKU(Cpa32U NumAccelerators, Cpa32U NumAccelEngines, Cpa32U ClkMask,
          Cpa32U fuse)
{
    Cpa32U clock = (ClkMask & ICP_DH89xxCC_CLKCTL_MASK)
                                           >> ICP_DH89xxCC_CLKCTL_SHIFT;

    switch (NumAccelerators)
    {
        case 1:
            return DEV_SKU_2;
        case ICP_DH89xxCC_MAX_ACCELERATORS:
        switch(clock)
        {
            case ICP_DH89xxCC_CLKCTL_DIV_SKU3:
                return DEV_SKU_3;
            case ICP_DH89xxCC_CLKCTL_DIV_SKU4:
                return DEV_SKU_4;
            default:
                return DEV_SKU_UNKNOWN;
        }
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
getNumAccelerators(adf_hw_device_data_t *self, Cpa32U mask)
{
    Cpa32U i = 0, ctr = 0;

    if (!mask)
    {
        return 0;
    }

    for (i = 0; i < ICP_DH89xxCC_MAX_ACCELERATORS; i++)
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
getNumAccelEngines(adf_hw_device_data_t *self, Cpa32U mask)
{
    Cpa32U i, ctr = 0;

    if (!mask)
    {
        return 0;
    }

    for (i = 0; i < ICP_DH89xxCC_MAX_ACCELENGINES; i++)
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
 * Function returns the accelerator mask for both
 * accelerators in the system
 */
STATIC void
getAccelMaskList(adf_hw_device_data_t *self,
                Cpa32U accelMask, Cpa32U aeMask, Cpa32U *pMaskList)
{
    /* This function sets the accelerator mask for both accelerators */
    Cpa32U i = 0, accelId = 0;
    Cpa32U ae_ctr = 0, tmpMask = 0;
    Cpa32U numAe = 0, numAccel = 0;
    Cpa32U num_aes_per_accel = 0;

    for(i = 0; i < self->maxNumAccel; i++)
    {
        pMaskList[i] = 0;
    }
    /*
     * For the case where we only have 1 accelerator
     * the all the AEs must be associated with that
     * accelerator.
     */
    numAccel = self->getNumAccelerators(self, accelMask);
    if(self->maxNumAccel != numAccel)
    {
        for(accelId = 0; accelId < self->maxNumAccel; accelId++)
        {
            /*
             * The accelMask is equal to 1 or 2 in decimal units.
             * Once we discover if we are 1 or 2 then we are done.
             */
            if(accelMask == (accelId+1))
            {
                pMaskList[accelId] = aeMask;
                /*In this case we are done.*/
                return;
            }
            else
            {
                pMaskList[accelId] = 0;
            }
        }
    }
    /*
     * For the other cases we must have 2 accelerator
     * active. Calculate the mask for the first accelerator
     * and the second one is clear then
     */
    accelId = 0;
    numAe = self->getNumAccelEngines(self, aeMask);
    num_aes_per_accel = numAe/numAccel;
    for(i = 0; i < self->maxNumAccelEngines; i++)
    {
        if (aeMask & (1 << i))
        {
            ae_ctr++;
            pMaskList[accelId] |= (1 << i);
        }
        if(ae_ctr == num_aes_per_accel)
        {
            /* We are done with this accelerator. */
            break;
        }
    }
    /* Now to find the mask of the other accelerator is simple. */
    tmpMask = ~pMaskList[accelId] & aeMask;
    pMaskList[++accelId] = tmpMask;

    return;
}

/*
 * getAccFuncOffset
 * Function returns the accelerator capabilities offset
 */
STATIC Cpa32U
getAccFuncOffset(void)
{
    return ICP_DH89xxCC_LEGFUSE_OFFSET;
}

/*
 * getAccFuncMask
 * Gets the accelerator functionality mask based on the accelerator capability
 */
STATIC Cpa32U
getAccFuncMask(Cpa32U func)
{
    Cpa32U mask = 0;
    Cpa32U accFunc = 0;

    /* "func" is the value of the register.
     * Since a bit set to "1" disables the feature,
     * we must invert the value of the Fuse CSR and
     * keep only the 5 lower bits.
     */
    accFunc = ~func;
    accFunc &= ICP_DH89xxCC_LEGFUSE_MASK;

    if (accFunc & (1 << ICP_DH89xxCC_LEGFUSES_FN0BIT0))
        mask |= ICP_ACCEL_CAPABILITIES_CIPHER;

    if (accFunc & (1 << ICP_DH89xxCC_LEGFUSES_FN0BIT1))
        mask |= ICP_ACCEL_CAPABILITIES_AUTHENTICATION;

    if (accFunc & (1 << ICP_DH89xxCC_LEGFUSES_FN0BIT2))
        mask |= ICP_ACCEL_CAPABILITIES_REGEX;

    if (accFunc & (1 << ICP_DH89xxCC_LEGFUSES_FN0BIT3))
        mask |= ICP_ACCEL_CAPABILITIES_COMPRESSION;

    if (accFunc & (1 << ICP_DH89xxCC_LEGFUSES_FN0BIT4))
        mask |= ICP_ACCEL_CAPABILITIES_LZS_COMPRESSION;

    if ((accFunc & (1 << ICP_DH89xxCC_LEGFUSES_FN0BIT0)) &&
        (accFunc & (1 << ICP_DH89xxCC_LEGFUSES_FN0BIT1)) )
    {
        mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC;
        mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
        mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_0;
        mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_1;
        /* Add random number capability if crypto is supported */
        mask |= ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER;
    }

    return mask;
}

/*
 * getMiscBarId
 * Returns the MISC BAR id of the device
 */
STATIC Cpa32U
getMiscBarId(adf_hw_device_data_t *self)
{
    return ICP_DH89xxCC_PMISC_BAR;
}

/*
 * getSmiapfOffsetMask
 * Function sets the SMIAPF offset and default mask
 */
STATIC void
getSmiapfOffsetMask(adf_hw_device_data_t *self, Cpa32U *offset, Cpa32U *mask)
{
    if (NULL != offset)
    {
        *offset = ICP_DH89xxCC_SMIAOF;
    }
    if (NULL != mask)
    {
        *mask   = ICP_DH89xxCC_SMIA_MASK;
    }
}

/*
 * getSramBarId
 * Returns the SRAM BAR id of the device
 */
STATIC Cpa32U
getSramBarId(adf_hw_device_data_t *self)
{
    return ICP_DH89xxCC_PESRAM_BAR;
}

/*
 * getEtrBarId
 * Returns the ETR BAR id of the device
 */
STATIC Cpa32U
getEtrBarId(adf_hw_device_data_t *self, Cpa32U *offset)
{
    if (NULL != offset)
    {
        *offset = ICP_DH89xxCC_ETRING_CSR_OFFSET;
    }
    return ICP_DH89xxCC_PETRINGCSR_BAR;
}

/*
 * isBarAvailable
 * Returns whether the barId is available on the device
 */
STATIC CpaBoolean
isBarAvailable(adf_hw_device_data_t *self, Cpa32U barId)
{
    CpaBoolean bars[] = {CPA_TRUE, CPA_TRUE, CPA_TRUE};

    return (barId < (sizeof (bars) / sizeof (*bars))) ?
        bars[barId]                                   :
        CPA_FALSE;
}

/*
 * getEsramInfo
 * Function sets the eSRAM information of the device
 */
STATIC void
getEsramInfo(adf_hw_device_data_t *self, adf_esram_data_t *esramInfo)
{
    if (NULL != esramInfo)
    {
        esramInfo->sramAeAddr = ICP_DH89xxCC_SRAM_AE_ADDR;
        esramInfo->sramAeSize = ICP_DH89xxCC_SRAM_AE_SIZE;
    }
}

/*
 * getScratchRamInfo
 * Function returns the Scratch RAM information of the device
 *
 */
STATIC void
getScratchRamInfo(adf_hw_device_data_t *self,
            Cpa32U *offset, Cpa32U *size)
{
    if (NULL != offset)
    {
        *offset = ICP_DH89xxCC_REGION_SCRATCH_RAM_OFFSET;
    }

    if (NULL != size)
    {
        *size = ICP_DH89xxCC_REGION_SCRATCH_RAM_SIZE;
    }
}

/*
 * irqGetBankNumber
 * Function returns the bank number based on interrupt source(sintpf)
 */
STATIC Cpa32U
irqGetBankNumber(adf_hw_device_data_t *self,
            Cpa32U sintpf)
{
    return irq_get_bank_number(sintpf);
}

/*
 * irqGetAeSource
 * Function returns the AE source based on interrupt source(sintpf)
 */
STATIC Cpa32U
irqGetAeSource(adf_hw_device_data_t *self, Cpa32U sintpf)
{
    return irq_ae_source(sintpf);
}

/*
 * init
 * Function initialise internal hw data
 */
STATIC CpaStatus
init(adf_hw_device_data_t *self, Cpa8U node_id)
{
    ICP_CHECK_FOR_NULL_PARAM(self);

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
    ICP_CHECK_FOR_NULL_PARAM_VOID(self);

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
 * getApCsrOffsets
 * This function set the auto-push register offset based on
 * the bank id.
 */
STATIC void
getApCsrOffsets(Cpa32U bankId, Cpa32U *nfMask, Cpa32U *nfDest,
                Cpa32U *neMask, Cpa32U *neDest, Cpa32U *apDelay)
{
    if (NULL != nfMask)
    {
        *nfMask = ICP_DH89xxCC_AP_NF_MASK +
                    (bankId * ICP_DH89xxCC_AP_BANK_BYTE_OFFSET);
    }

    if (NULL != nfDest)
    {
        *nfDest = ICP_DH89xxCC_AP_NF_DEST +
                    (bankId * ICP_DH89xxCC_AP_BANK_BYTE_OFFSET);
    }

    if (NULL != neMask)
    {
        *neMask = ICP_DH89xxCC_AP_NE_MASK +
                    (bankId * ICP_DH89xxCC_AP_BANK_BYTE_OFFSET);
    }

    if (NULL != neDest)
    {
        *neDest = ICP_DH89xxCC_AP_NE_DEST +
                    (bankId * ICP_DH89xxCC_AP_BANK_BYTE_OFFSET);
    }

    if (NULL != apDelay)
    {
        *apDelay = ICP_DH89xxCC_AP_DELAY;
    }
}

/*
 * getVf2PfIntSourceOffset
 * This function returns byte offset of the VF-to-PF interrupt source CSR.
 */
STATIC Cpa32U
getVf2PfIntSourceOffset(void)
{
    return ICP_PMISCBAR_VFTOPFINTSRC;
}

/*
 * getVf2PfBitOffset
 * This function returns the VF-to-PF bit offset in the interrupt source CSR.
 */
STATIC Cpa32U
getVf2PfBitOffset(void)
{
    return ICP_VFTOPF_INTSOURCEOFFSET;
}

/*
 * getVf2PfIntMaskOffset
 * This function returns byte offset of the VF-to-PF interrupt mask CSR.
 */
STATIC Cpa32U
getVf2PfIntMaskOffset(void)
{
    return ICP_PMISCBAR_VFTOPFINTMSK;
}

/*
 * getVf2PfIntMaskDefault
 * This function returns the default VF-to-PF interrupt mask.
 */
STATIC Cpa32U
getVf2PfIntMaskDefault(void)
{
    return ICP_VFTOPF_INTSOURCEMASK;
}

/*
 * getTiMiscIntCtlOffset
 * This function returns byte offset of the miscellaneous interrupt
 * control register.
 */
STATIC Cpa32U
getTiMiscIntCtlOffset(void)
{
    return ICP_PMISCBAR_TIMISCINTCTL;
}

/*
 * getVIntMskOffset
 * This function returns byte offset of the VF interrupt mask CSR.
 */
STATIC Cpa32U
getVIntMskOffset(Cpa32U id)
{
    return (ICP_PMISCBAR_VINTMSK_OFFSET + (id * ICP_PMISCBAR_VFDEVOFFSET));
}

/*
 * getVIntMskDefault
 * This function returns the default VF interrupt mask.
 */
STATIC Cpa32U
getVIntMskDefault(void)
{
    return ICP_PMISCBAR_VINTMSK_DEFAULT;
}

/*
 * getPf2VfDbIntOffset
 * This function returns byte offset of the doorbell and interrupt CSR
 */
STATIC Cpa32U
getPf2VfDbIntOffset(Cpa32U id)
{
    return (ICP_PMISCBAR_PF2VFDBINT_OFFSET + (id * ICP_PMISCBAR_VFDEVOFFSET));
}

/*
 * getResetOffsets
 * Retrieve reset offset and vector
 */
STATIC void
getResetOffsets(Cpa32U *resetOffset, Cpa32U *resetVector)
{
	if (NULL != resetOffset) {
        *resetOffset = ICP_DH89xxCC_RESET_OFFSET;
	}
	if (NULL != resetVector) {
        *resetVector = ICP_DH89xxCC_RESET_OFFSET;
	}
}

/*
 * enable_unco_err_interrupts
 * Enable all the supported uncorrectable error sources and their
 * logging/debug abilities
 */

STATIC CpaStatus
enableUncoErr_interrupts(icp_accel_dev_t *accel_dev)
{
    Cpa8U i = 0;
    void *pmisc_bar_addr = NULL;
    volatile Cpa32U temp_reg = 0;
    Cpa32U numAEs = 0;
    Cpa32U numAccel = 0;
    adf_hw_device_data_t *hw_data = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);
    hw_data = accel_dev->pHwDeviceData;

#ifdef ICP_SRIOV
        return 0;
#endif

    /* Get the number of accelerator engines */
    numAEs = hw_data->getNumAccelEngines(hw_data, accel_dev->aeMask);

    /* Get the number of accelerators */
    numAccel = hw_data->getNumAccelerators(hw_data, accel_dev->accelMask);

    /*Retrieve the pmiscbar address*/
    pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);

    /*Setup interrupt mask*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_ERRMSK0,
            ICP_DH89xxCC_ERRMSK0_UERR); /*AE3-AE0*/
    if(numAccel > 1)
    {
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_ERRMSK1,
            ICP_DH89xxCC_ERRMSK1_UERR); /*AE7-AE4*/
    }
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_ERRMSK3,
        ICP_DH89xxCC_ERRMSK3_UERR); /*MISC*/

    /*
     * Setup Error Sources
     */
    for(i = 0; i < numAEs; i++)
    {
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_PMISCBAR_CTX_ENABLES(i),
            ICP_DH89xxCC_ENABLE_ECC_ERR); /*AEi*/

        /*@note: this code enable ECC and parity AE hw correction.
         * The driver is enabling them for the FW/HW but the handling of those
         * correction is not covered by the driver*/
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_PMISCBAR_MISC_CONTROL(i),
            ICP_DH89xxCC_ENABLE_ECC_PARITY_CORR); /*AEi*/
    }

    for(i = 0; i < numAccel; i++)
    {
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_INTMASKSSM(i),
            ICP_DH89xxCC_INTMASKSSM_UERR) /* Accelerator(i) */;
    }

    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_CPP_SHAC_ERR_CTRL,
        ICP_DH89xxCC_CPP_SHAC_UE); /*SHaC*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_ESRAMCERR,
        ICP_DH89xxCC_ESRAM_CERR); /*eSRAM ecc correction*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_ESRAMUERR,
        ICP_DH89xxCC_ESRAM_UERR); /*eSRAM*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_CPPMEMTGTERR,
        ICP_DH89xxCC_TGT_UERR); /*Push/Pull Misc Error*/

    /*Check if signal misc errors are set and interrupts routed to IA*/
    temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH89xxCC_SMIAOF);
    if (!(temp_reg & ICP_DH89xxCC_AE_IRQ_MASK)) {
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_SMIAOF,
            ICP_DH89xxCC_AE_IRQ_MASK);
    }

    /*
     * Enable Logging
     */
    for(i = 0; i < numAccel; i++)
    {
        /*Shared Memory*/
        temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH89xxCC_UERRSSMSH(i)) |
            ICP_DH89xxCC_UERRSSMSH_EN;
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_UERRSSMSH(i), temp_reg);
        /*Shared Memory Ecc Error Correction*/
        temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH89xxCC_CERRSSMSH(i)) |
            ICP_DH89xxCC_CERRSSMSH_EN;
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_CERRSSMSH(i), temp_reg);

        /*MMP*/
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_UERRSSMMMP0(i),
            ICP_DH89xxCC_UERRSSMMMP_EN);
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_UERRSSMMMP1(i),
            ICP_DH89xxCC_UERRSSMMMP_EN);
        /*MMP Correctable ECC correction*/
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_CERRSSMMMP0(i),
            ICP_DH89xxCC_CERRSSMMMP_EN);
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_CERRSSMMMP1(i),
            ICP_DH89xxCC_CERRSSMMMP_EN);

        /*Push/Pull*/
        temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH89xxCC_PPERR(i)) |
            ICP_DH89xxCC_PPERR_EN;
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_PPERR(i), temp_reg);
    }
    /*Push/Pull Misc*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_DH89xxCC_TICPPINTCTL,
            ICP_DH89xxCC_TICPP_EN);

    return CPA_STATUS_SUCCESS;
}

/*
 * adf_isr_LogAE_Int
 * Log accel engine based uncorrectable errors
 */
STATIC inline void
adf_isr_LogAE_Int(Cpa8U index, void *pmisc_bar_addr)
{
    Cpa32U ustore = ICP_ADF_CSR_RD(pmisc_bar_addr,
        ICP_DH89xxCC_USTORE_ERROR_STATUS(index));
    Cpa32U reg_error = ICP_ADF_CSR_RD(pmisc_bar_addr,
        ICP_DH89xxCC_REG_ERROR_STATUS(index));
    Cpa32U eperrlog = ICP_ADF_CSR_RD(pmisc_bar_addr,
        ICP_DH89xxCC_EPERRLOG);
    Cpa32U ctx_enables = ICP_ADF_CSR_RD(pmisc_bar_addr,
        ICP_DH89xxCC_PMISCBAR_CTX_ENABLES(index));

    ADF_DEBUG("ustore 0x%X, reg_error 0x%X, \n"
        "eperrlog 0x%X ctx_enables 0x%X\n",
        ustore, reg_error, eperrlog, ctx_enables);
    ADF_ERROR("AE #%x Error\n"
        "Uncorrectable Error: %s\n"
        "Control Store Error: %s\n"
        "Control Store information: Address 0x%X - syndrom 0x%X\n"
        "Register ECC Error %s - Parity Error: %s\n"
        "Register information: address 0x%X - type %s\n",
        index,
        ICP_DH89xxCC_EPERRLOG_ERR(
            ICP_DH89xxCC_GET_EPERRLOG(eperrlog, index)),
        ICP_DH89xxCC_USTORE_IS_UE(ustore),
        ICP_DH89xxCC_USTORE_UADDR(ustore),
        ICP_DH89xxCC_USTORE_GET_SYN(ustore),
        ICP_DH89xxCC_CTX_ECC_ERR(ctx_enables),
        ICP_DH89xxCC_CTX_PAR_ERR(ctx_enables),
        ICP_DH89xxCC_REG_GET_REG_AD(reg_error),
        ICP_DH89xxCC_REG_TYP_STR(ICP_DH89xxCC_REG_TYPE(reg_error)));
}

/*
 * adf_isr_LogSSM_Int
 * Log SSM based uncorrectable errors (shared Mem, MMP and Push/Pull)
 */
STATIC inline CpaBoolean
adf_isr_LogSSM_Int(void *pmisc_bar_addr,
        Cpa8U accel, Cpa8U mmp, Cpa8U type)
{
    Cpa32U uerrssm = 0, uerrssmad = 0;
    Cpa32U uerrssm_off = 0, uerrssmad_off = 0;

    if (ICP_DH89xxCC_PPERR_T == type) {
        Cpa32U pperr = 0, pperrid = 0;
        pperr = ICP_ADF_CSR_RD(pmisc_bar_addr,
            ICP_DH89xxCC_PPERR(accel));
        pperrid = ICP_ADF_CSR_RD(pmisc_bar_addr,
            ICP_DH89xxCC_PPERRID(accel));
        ADF_DEBUG("pperr 0x%X pperrid 0x%X\n", pperr, pperrid);
        ADF_ERROR("Push/Pull Error on Accelerator %i\n"
            "%s - Error type: %s\n"
            "id 0x%X\n",
            accel, ICP_DH89xxCC_PPERR_GET_STATUS(pperr),
            ICP_DH89xxCC_PPERR_TYPE_STR(
                ICP_DH89xxCC_PPERR_TYPE(pperr)),
            pperrid);
        return CPA_TRUE;
    }
    else if (ICP_DH89xxCC_SHAREDMEM_T == type) {
        uerrssm_off = ICP_DH89xxCC_UERRSSMSH(accel);
        uerrssmad_off = ICP_DH89xxCC_UERRSSMSHAD(accel);
    }
    else {
        /*MMP*/
        switch(mmp) {
        case 1:
            {
                uerrssm_off = ICP_DH89xxCC_UERRSSMMMP1(accel);
                uerrssmad_off = ICP_DH89xxCC_UERRSSMMMPAD1(accel);
                break;
            }
        case 0:
        default:
            {
                uerrssm_off = ICP_DH89xxCC_UERRSSMMMP0(accel);
                uerrssmad_off = ICP_DH89xxCC_UERRSSMMMPAD0(accel);
                break;
            }
        }
    }
    uerrssm = ICP_ADF_CSR_RD(pmisc_bar_addr, uerrssm_off);
    uerrssmad = ICP_ADF_CSR_RD(pmisc_bar_addr, uerrssmad_off);

    if (ICP_DH89xxCC_MMP_T == type) {
        /*MMP*/
        ADF_DEBUG("uerrssmmmp 0x%X, uerrssmmmpad 0x%X\n",
            uerrssm, uerrssmad);
        ADF_ERROR("MMP %i Accelerator %i - Uncorrectable Error: %s\n"
            "Operation: %s - Address: 0x%X\n",
            mmp, accel,
            (((uerrssm & ICP_DH89xxCC_UERRSSMMMP_UERR))!= 0 ? "Yes": "No"),
            ICP_DH89xxCC_UERRSSMMMP_ERRTYPE_STR(
                ICP_DH89xxCC_UERRSSMMMP_ERRTYPE(uerrssm)),
            (uerrssmad & ICP_DH89xxCC_UERRSSMMMPAD_ADDR));
        return (uerrssm & ICP_DH89xxCC_UERRSSMMMP_UERR)!= 0
            ? CPA_TRUE: CPA_FALSE;
    }
    else {
        ADF_DEBUG("uerrssmsh 0x%X, uerrssmshad 0x%X\n",
            uerrssm, uerrssmad);
        ADF_ERROR("SharedMem Error CPM %x - Uncorrectable Error: %s\n"
            "Operation: %s - Type: %s - Address: 0x%X\n",
            accel,
            (uerrssm & ICP_DH89xxCC_UERRSSMSH_UERR) != 0? "Yes": "No",
            ICP_DH89xxCC_UERRSSMSH_GET_OP(uerrssm),
            ICP_DH89xxCC_UERRSSMSH_GET_ERRTYPE(
                ICP_DH89xxCC_UERRSSMSH_ERRTYPE(uerrssm)),
            (uerrssmad & ICP_DH89xxCC_UERRSSMSHAD_ADDR));
        return (uerrssm & ICP_DH89xxCC_UERRSSMSH_UERR) != 0
            ? CPA_TRUE: CPA_FALSE;
    }
}

/*
 * handleUncoErr_Interrupts
 * Handle AE interrupt sources: Uncorrectable errors and firmware custom
 */
STATIC CpaBoolean
handleUncoErr_Interrupts(icp_accel_dev_t *accel_dev)
{
    void *pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);
    CpaBoolean reset_needed = CPA_FALSE;

    Cpa32U errsou0 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH89xxCC_ERRSOU0);
    Cpa32U errsou1 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH89xxCC_ERRSOU1);
    Cpa32U errsou3 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_DH89xxCC_ERRSOU3);

    /*UERR interrupts ?*/
    if ((0 != errsou0) || (0 != errsou1) || (0 != errsou3)) {
        ADF_DEBUG("UERR interrupt occurred");
        /*Identify the interrupt Sources and log them*/
        /*AE0-3 Errors*/
        if (0 != errsou0) {
            if (errsou0 & ICP_DH89xxCC_M0UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_isr_LogAE_Int(ICP_DH89xxCC_AE0, pmisc_bar_addr);
            }
            if (errsou0 & ICP_DH89xxCC_M1UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_isr_LogAE_Int(ICP_DH89xxCC_AE1, pmisc_bar_addr);
            }
            if (errsou0 & ICP_DH89xxCC_M2UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_isr_LogAE_Int(ICP_DH89xxCC_AE2, pmisc_bar_addr);
            }
            if (errsou0 & ICP_DH89xxCC_M3UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_isr_LogAE_Int(ICP_DH89xxCC_AE3, pmisc_bar_addr);
            }
        }
        /*AE4-7 Errors*/
        if (0 != errsou1) {
            if (errsou1 & ICP_DH89xxCC_M4UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_isr_LogAE_Int(ICP_DH89xxCC_AE4, pmisc_bar_addr);
            }
            if (errsou1 & ICP_DH89xxCC_M5UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_isr_LogAE_Int(ICP_DH89xxCC_AE5, pmisc_bar_addr);
            }
            if (errsou1 & ICP_DH89xxCC_M6UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_isr_LogAE_Int(ICP_DH89xxCC_AE6, pmisc_bar_addr);
            }
            if (errsou1 & ICP_DH89xxCC_M7UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_isr_LogAE_Int(ICP_DH89xxCC_AE7, pmisc_bar_addr);
            }
        }
        if (0 != errsou3) {
            /*eSRAM errors*/
            if (errsou3 & ICP_DH89xxCC_UERR_MASK) {
                Cpa32U  esramuerr = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_DH89xxCC_ESRAMUERR);
                Cpa32U  esramuerrad = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_DH89xxCC_ESRAMUERRAD);
                reset_needed = CPA_TRUE;
                ADF_DEBUG("esramuerr 0x%X - esramuerrad 0x%X\n",
                    esramuerr, esramuerrad);
                ADF_ERROR("Uncorrectable Error Occurred on eSram\n"
                    "Interrupt triggered: %s\n"
                    "Operation type %s - Error Type %s\n"
                    "Address (quad words) 0x%X\n",
                    ICP_DH89xxCC_ESRAMUERR_GET_UERR(esramuerr),
                    ICP_DH89xxCC_ESRAMUERR_GET_OP(esramuerr),
                    ICP_DH89xxCC_ESRAMUERR_GET_ERRSTR(
                         ICP_DH89xxCC_ESRAMUERR_GET_ERRTYPE(esramuerr)),
                    esramuerrad);
            }
            /*CPM: MMP, SH, PUSH/PULL errors*/
            if ((errsou3 & ICP_DH89xxCC_EMSK3_CPM0_MASK) ||
                    (errsou3 & ICP_DH89xxCC_EMSK3_CPM1_MASK)) {
                Cpa8U accel = 0;
                Cpa32U instatssm = 0;
                if ((errsou3 & ICP_DH89xxCC_EMSK3_CPM0_MASK)) {
                    accel = 0;
                    instatssm = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH89xxCC_INTSTATSSM(accel));
                }
                else {
                    accel = 1;
                    instatssm = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH89xxCC_INTSTATSSM(accel));
                }
                ADF_DEBUG("Possible Uncorrectable Error Occurred"
                    " on Accelerator %i\n", accel);
                ADF_DEBUG("intstatssm 0x%X\n", instatssm);
                if (instatssm & ICP_DH89xxCC_INTSTATSSM_MMP1) {
                    reset_needed = adf_isr_LogSSM_Int(pmisc_bar_addr, accel,
                        ICP_DH89xxCC_SLICE1, ICP_DH89xxCC_MMP_T);
                }
                if (instatssm & ICP_DH89xxCC_INTSTATSSM_MMP0) {
                    reset_needed = adf_isr_LogSSM_Int(pmisc_bar_addr, accel,
                        ICP_DH89xxCC_SLICE0, ICP_DH89xxCC_MMP_T);
                }
                if (instatssm & ICP_DH89xxCC_INTSTATSSM_SH) {
                    reset_needed = adf_isr_LogSSM_Int(pmisc_bar_addr, accel,
                        ICP_DH89xxCC_SLICE0/*not used*/, ICP_DH89xxCC_SHAREDMEM_T);
                }
                if (instatssm & ICP_DH89xxCC_INTSTATSSM_PPERR) {
                    reset_needed = adf_isr_LogSSM_Int(pmisc_bar_addr, accel,
                        ICP_DH89xxCC_SLICE0/*not used*/, ICP_DH89xxCC_PPERR_T);
                }
            }
            /*Or'ed Shac Data Error*/
            if (errsou3 & ICP_DH89xxCC_EMSK3_SHaC0_MASK) {
                Cpa32U err_status = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_DH89xxCC_CPP_SHAC_ERR_STATUS);
                Cpa32U err_ppid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_DH89xxCC_CPP_SHAC_ERR_PPID);
                reset_needed = CPA_TRUE;
                /*Log error*/
                ADF_DEBUG("ccp_shac_err_status 0x%X -"
                    "cpp_shac_err_ppid 0x%X\n", err_status, err_ppid);
                ADF_ERROR("Uncorrectable Data Error Occurred in SHaC\n"
                    "Operation type: %s, Error target %s\n"
                    "Error ppid 0x%X\nInterrupt Triggered %s\n",
                    ICP_DH89xxCC_CPP_SHAC_GET_TYP(err_status) == 0 ?
                    "Push": "Pull",
                    ICP_DH89xxCC_CPP_SHAC_ERTYP_STR(
                    ICP_DH89xxCC_CPP_SHAC_GET_ERTYP(err_status)),
                    err_ppid,
                    ICP_DH89xxCC_CPP_SHAC_GET_INT(err_status) == 1 ?
                    "Yes": "No");
            }
            /*Push/Pull Misc*/
            if (errsou3 & ICP_DH89xxCC_PPMISCERR_MASK) {
                Cpa32U cpptgterr = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_DH89xxCC_CPPMEMTGTERR);
                Cpa32U errppid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_DH89xxCC_ERRPPID);
                Cpa32U ticppintsts = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_DH89xxCC_TICPPINTSTS);
                Cpa32U tierrid = 0;
                if(ticppintsts & ICP_DH89xxCC_TICPP_PUSH) {
                    tierrid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH89xxCC_TIERRPUSHID);
                }
                else {
                    tierrid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_DH89xxCC_TIERRPULLID);
                }
                reset_needed = CPA_TRUE;
                ADF_DEBUG("cpptgterr 0x%X, errppid 0x%X, ticppintsts 0%X,"
                    "tierrxxxid 0x%X\n", cpptgterr, errppid,
                    ticppintsts, tierrid);
                ADF_ERROR("Uncorrectable Push/Pull Misc Error\n"
                    "memory status: %s - Transaction Id 0x%X - "
                    "Error type %s\n"
                    "Bus Operation Type %s - Id 0x%X\n",
                    ICP_DH89xxCC_CPPMEMTGTERR_GET_STATUS(cpptgterr),
                    errppid,
                    ICP_DH89xxCC_CPPMEMTGTERR_ERRTYP_STR(
                        ICP_DH89xxCC_CPPMEMTGTERR_ERRTYP(cpptgterr)),
                    ICP_DH89xxCC_TICPP_GETOP(ticppintsts), tierrid);
            }
        } /* ERRSOU3*/
    }

    return reset_needed;
}

/*
 * adf_set_hw_data_dh89xxcc
 * Initialise the hw data structure
 */
void
adf_set_hw_data_dh89xxcc(void *hw_data)
{
    adf_hw_device_data_t *hw = NULL;

    ICP_CHECK_FOR_NULL_PARAM_VOID(hw_data);

    hw = (adf_hw_device_data_t *)hw_data;

    hw->init    = init;
    hw->cleanup = cleanup;

    hw->dev_class = &dh89xxcc_class;

    /* populate bank/ac information */
    hw->maxBars                 = ICP_DH89xxCC_MAX_PCI_BARS;
    hw->maxNumBanks             = ICP_DH89xxCC_ETR_MAX_BANKS;
    hw->maxNumApBanks           = ICP_DH89xxCC_ETR_MAX_AP_BANKS;
    hw->numRingsPerBank         = ICP_DH89xxCC_ETR_MAX_RINGS_PER_BANK;
    hw->numBanksPerAccel        = ICP_DH89xxCC_BANKS_PER_ACCELERATOR;
    hw->maxNumAccel             = ICP_DH89xxCC_MAX_ACCELERATORS;
    hw->maxNumAccelEngines      = ICP_DH89xxCC_MAX_ACCELENGINES;
    hw->aeClockInMhz            = ICP_DH89xxCC_AE_CLOCK_IN_MHZ;
    hw->sintpfOffset            = ICP_DH89xxCC_SINTPF;
    hw->userEtringCsrSize       = ICP_DH89xxCC_USER_ETRING_CSR_SIZE;
    hw->maxNumVf                = ICP_DH89xxCC_MAX_NUM_VF;

    /* populate msix information */
    hw->msix.banksVectorStart   = ICP_DH89xxCC_MSIX_BANK_VECTOR_START;
    hw->msix.banksVectorNum     = ICP_DH89xxCC_MSIX_BANK_VECTOR_NUM;
    hw->msix.aeVectorStart      = ICP_DH89xxCC_MSIX_AE_VECTOR_START;
    hw->msix.aeVectorNum        = ICP_DH89xxCC_MSIX_AE_VECTOR_NUM;

    hw->isMemScrubSupported     = CPA_TRUE;

    /* device fuse */
    hw->getDevFuseOffset        = getDevFuseOffset;
    hw->getDevClkOffset         = getDevClkOffset;
    hw->getDevSKU               = getDevSKU;
    hw->getAcceleratorsMask     = getAcceleratorsMask;
    hw->getAccelEnginesMask     = getAccelEnginesMask;
    hw->getNumAccelerators      = getNumAccelerators;
    hw->getNumAccelEngines      = getNumAccelEngines;
    hw->getAccelMaskList        = getAccelMaskList;

    /* accelerator functions */
    hw->getAccFuncOffset        = getAccFuncOffset;
    hw->getAccFuncMask          = getAccFuncMask;

    /* populate bars callbacks  */
    hw->getEtrBarId             = getEtrBarId;
    hw->getSramBarId            = getSramBarId;
    hw->getMiscBarId            = getMiscBarId;
    hw->isBarAvailable          = isBarAvailable;

    /* populate esram callbacks    */
    hw->getEsramInfo            = getEsramInfo;

    /* populate SHAC callback */
    hw->getScratchRamInfo       = getScratchRamInfo;

    /* populate PMiscBar callback  */
    hw->getSmiapfOffsetMask     = getSmiapfOffsetMask;

    hw->irqGetBankNumber        = irqGetBankNumber;
    hw->irqGetAeSource          = irqGetAeSource;

    hw->irqGetBunSource       = NULL;
    hw->irqGetPF2VFSource     = NULL;

    hw->getBankBhHandler        = getBankBhHandler;
    hw->getBankBhPollingHandler = getBankBhPollingHandler;

    /* Auto-push feature callback */
    hw->isAutoPushSupported     = CPA_TRUE;
    hw->getApCsrOffsets         = getApCsrOffsets;

    hw->getVf2PfIntSourceOffset = getVf2PfIntSourceOffset;
    hw->getVf2PfBitOffset       = getVf2PfBitOffset;
    hw->getVf2PfIntMaskOffset   = getVf2PfIntMaskOffset;
    hw->getVf2PfIntMaskDefault  = getVf2PfIntMaskDefault;

    hw->getTiMiscIntCtlOffset   = getTiMiscIntCtlOffset;
    hw->getVIntSrcOffset        = NULL;
    hw->getVIntMskOffset        = getVIntMskOffset;
    hw->getVIntMskDefault       = getVIntMskDefault;
    hw->getPf2VfDbIntOffset     = getPf2VfDbIntOffset;

    hw->getResetOffsets         = getResetOffsets;

    hw->enableUncoErrInterrupts = enableUncoErr_interrupts;
    hw->handleUncoErrInterrupts = handleUncoErr_Interrupts;
}
