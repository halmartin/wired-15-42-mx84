/*******************************************************************************
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
*******************************************************************************/

/*******************************************************************************
* @file adf_c2xxx_hw_data.c
*
* @ingroup
*
* @description
*   This file contains the hw device data for a C2XXX device
*
*******************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_platform_common.h"
#include "adf_platform_c2xxx.h"

#define ICP_C2XXX_CTX_ECC_ERR(val)         ((val & (1<<29)) == 0 ? "No": "Yes")
#define ICP_C2XXX_CTX_PAR_ERR(val)         ((val & (1<<25)) == 0 ? "No": "Yes")
#define ICP_C2XXX_EPERRLOG_ERR(val)        ((val  == 0)      ?     "No": "Yes")
#define ICP_C2XXX_USTORE_IS_UE(val)        ((val & (1<<31)) == 0 ? "No": "Yes")

/*Mem type mask (00-GPR 01-transfer 10-NextNeighbor 11-Local Mem)*/
#define ICP_C2XXX_REG_TYP_STR(val)                  \
        ((val == 0)? "GPR":                         \
        ((val == 1)? "Transfer":                    \
        ((val == 2)? "Next Neighbour" :             \
        "Local Memory")))

/*MMP ECC Err TypeMask  0 - MMP program mem 1 - MMP OPA Ram, 2 MMP OPB Ram
 *(3-F Reserved)*/
#define ICP_C2XXX_UERRSSMMMP_ERRTYPE_STR(val)       \
    ((val == 0)? "MMP program Mem":                 \
    ((val == 1)? "MMP OPA Ram"    :                 \
    ((val == 2)? "MMP OPB Ram": "Reserved")))

#define ICP_C2XXX_UERRSSMSH_GET_OP(val)             \
        ((val& ICP_C2XXX_UERRSSMSH_R) != 0? "Read": \
        ((val& ICP_C2XXX_UERRSSMSH_W) != 0? "Write": "Error"))

/*Error type 1,4,7-F reserved ECC 0-slice push 2-AE Push command
 * 3-Me Pull Command 5-DRAM Pull 6-Accelerator Memory read (RMW DRAM  push)*/
#define ICP_C2XXX_UERRSSMSH_GET_ERRTYPE(val)        \
    ((val == 0)? "Slice Push":                      \
    ((val == 2)? "AE Push command":                 \
    ((val == 3)? "AE Pull command":                 \
    ((val == 5)? "DRAM pull":                       \
    ((val == 6)? "Accelerator Memory read (DRAM  push)" : "Reserved")))))

#define ICP_C2XXX_PPERR_GET_STATUS(val)             \
   (((val & ICP_C2XXX_PPERR_PPERR)) != 0?           \
   (((val & ICP_C2XXX_PPERR_MERR) != 0)?            \
       "Multiple errors occurred":                  \
       "One error occured"):                        \
       "No errors occurred")

/*Error type 0-data asserted on AE Pull Data 1- data asserted on  Dram Push
 * 2-Master bus error 3-ECC error on Shared mem access to Dram pull data bus
 * 4-F Reserved*/
#define ICP_C2XXX_PPERR_TYPE_STR(val)               \
    ((val == 0)? "Data asserted on AE Pull" :       \
    ((val == 1)? "Data asserted on Dram Push":      \
    ((val == 2)? "Master Bus Error":                \
    ((val == 3)? "ECC error on Shared mem access"   \
                 "to Dram pull data bus": "Reserved"))))

/*Error type operation 10-CAP(not possible for push)*/
#define ICP_C2XXX_CPP_ERTYP_STR(val)                \
    ((val == 2)? "CAP - impossible for push": "reserved")

#define ICP_C2XXX_TICPP_GETOP(val)                  \
    ((val&ICP_C2XXX_TICPP_PUSH) != 0?"Push": "Pull")


extern void adf_bank0_handler(void* handle);
extern void adf_bank1_handler(void* handle);
extern void adf_bank2_handler(void* handle);
extern void adf_bank3_handler(void* handle);
extern void adf_bank4_handler(void* handle);
extern void adf_bank5_handler(void* handle);
extern void adf_bank6_handler(void* handle);
extern void adf_bank7_handler(void* handle);

extern void adf_bank0_polling_handler(void* handle);
extern void adf_bank1_polling_handler(void* handle);
extern void adf_bank2_polling_handler(void* handle);
extern void adf_bank3_polling_handler(void* handle);
extern void adf_bank4_polling_handler(void* handle);
extern void adf_bank5_polling_handler(void* handle);
extern void adf_bank6_polling_handler(void* handle);
extern void adf_bank7_polling_handler(void* handle);

/*******************************************************************************
* STATIC Variables
*******************************************************************************/

STATIC adf_hw_device_class_t c2xxx_class = {
    .name         = "c2xxx"    ,
    .type         = DEV_C2XXX  ,
    .numInstances = 0             ,
    .currentInstanceId = 0
};

STATIC bank_handler adf_c2xxx_bh_bank_handlers[] = {
    adf_bank0_handler,
    adf_bank1_handler,
    adf_bank2_handler,
    adf_bank3_handler,
    adf_bank4_handler,
    adf_bank5_handler,
    adf_bank6_handler,
    adf_bank7_handler,
};

STATIC bank_handler adf_c2xxx_bh_polling_bank_handlers[] = {
    adf_bank0_polling_handler,
    adf_bank1_polling_handler,
    adf_bank2_polling_handler,
    adf_bank3_polling_handler,
    adf_bank4_polling_handler,
    adf_bank5_polling_handler,
    adf_bank6_polling_handler,
    adf_bank7_polling_handler,
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
adf_c2xxx_getAcceleratorsMask(Cpa32U fuse)
{
    return ((~fuse) & ICP_C2XXX_ACCELERATORS_MASK);
}

/*
 * getAccelEnginesMask
 * Gets the accelerator engine mask based on the fuse information
 */
STATIC Cpa32U
adf_c2xxx_getAccelEnginesMask(Cpa32U fuse)
{
    /* return zero if both slices of any service are disabled */
    if( ((fuse & (1 << ICP_C2XXX_PKE_DISABLE_BIT))) ||
        ((fuse & (1 << ICP_C2XXX_ATH_DISABLE_BIT))) ||
        ((fuse & (1 << ICP_C2XXX_CPH_DISABLE_BIT))) )
        return 0;
    else
        return ( (~((fuse >> ICP_C2XXX_AE1_DISABLE_BIT) <<
                                 ICP_C2XXX_AE1_DISABLE_BIT)) &
                                     ICP_C2XXX_ACCELENGINES_MASK);
}

/*
 * getDevFuseOffset
 * Function returns the device fuse control offset
 */
STATIC Cpa32U
adf_c2xxx_getDevFuseOffset(void)
{
    return ICP_C2XXX_FUSECTL_OFFSET;
}

/*
 * getDevSKU
 * Function returns device SKU info
 */
STATIC dev_sku_info_t
adf_c2xxx_getDevSKU(Cpa32U NumAccelerators, Cpa32U NumAccelEngines, Cpa32U ClkMask,
                    Cpa32U fuse)
{

    switch (NumAccelEngines)
    {
        case 1:
            if(fuse & (1 << ICP_C2XXX_LOW_SKU_BIT))
            {
                return DEV_SKU_3;
            }
               if(fuse & (1 << ICP_C2XXX_MID_SKU_BIT))
            {
                return DEV_SKU_2;
            }
        case ICP_C2XXX_MAX_ACCELENGINES:
            return DEV_SKU_1;
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
adf_c2xxx_getNumAccelerators(adf_hw_device_data_t *self, Cpa32U mask)
{
    Cpa32U i = 0, ctr = 0;

    if (!mask)
    {
        return 0;
    }

    for (i = 0; i < ICP_C2XXX_MAX_ACCELERATORS; i++)
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
adf_c2xxx_getNumAccelEngines(adf_hw_device_data_t *self, Cpa32U mask)
{
    Cpa32U i, ctr = 0;

    if (!mask)
    {
        return 0;
    }

    for (i = 0; i < ICP_C2XXX_MAX_ACCELENGINES; i++)
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
 * Function returns the accelerator mask for the
 * accelerator in the system
 */
STATIC void
adf_c2xxx_getAccelMaskList(struct adf_hw_device_data_s *self,
                Cpa32U devAccelMask, Cpa32U devAeMask, Cpa32U *pMaskList)
{
    pMaskList[0] = devAeMask;
}

/*
 * getAccFuncOffset
 * Function returns the accelerator capabilities offset
 */
STATIC Cpa32U
adf_c2xxx_getAccFuncOffset(void)
{
    return ICP_C2XXX_FUSECTL_OFFSET;
}

/*
 * getAccFuncMask
 * Gets the accelerator functionality mask based on the accelerator capability
 */
STATIC Cpa32U
adf_c2xxx_getAccFuncMask(Cpa32U func)
{
    Cpa32U mask = 0;
    Cpa32U accFunc = 0;

    /* func is the value of the FUSECTL register */
    accFunc = func;

    /* All the SKUs have the same acceleration capabilities
     * and therefore the mask can be hard coded.
     */
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC;
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
    mask |= ICP_ACCEL_CAPABILITIES_AUTHENTICATION;
    mask |= ICP_ACCEL_CAPABILITIES_CIPHER;
    mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_0;

    if( (!(accFunc & (1 << ICP_C2XXX_LOW_SKU_BIT))) &&
        (!(accFunc & (1 << ICP_C2XXX_MID_SKU_BIT)))  )
        mask |= ICP_ACCEL_CAPABILITIES_CRYPTO_1;

    return mask;
}

/*
 * getMiscBarId
 * Returns the MISC BAR id of the device
 */
STATIC Cpa32U
adf_c2xxx_getMiscBarId(adf_hw_device_data_t *self)
{
    return ICP_C2XXX_PMISC_BAR;
}

/*
 * getSmiapfOffsetMask
 * Function sets the SMIAPF offset and default mask
 */
STATIC void
adf_c2xxx_getSmiapfOffsetMask(adf_hw_device_data_t *self, Cpa32U *offset, Cpa32U *mask)
{
    if(NULL != offset)
    {
        *offset = ICP_C2XXX_SMIAOF;
    }
    if(NULL != mask)
    {
        *mask   = ICP_C2XXX_SMIA_MASK;
    }
}

/*
 * getEtrBarId
 * Returns the ETR BAR id of the device
 */
STATIC Cpa32U
adf_c2xxx_getEtrBarId(adf_hw_device_data_t *self, Cpa32U *offset)
{
    if (offset)
    {
        *offset = ICP_C2XXX_ETRING_CSR_OFFSET;
    }
    return ICP_C2XXX_PETRINGCSR_BAR;
}

/*
 * isBarAvailable
 * Returns whether the barId is available on the device
 */
STATIC CpaBoolean
adf_c2xxx_isBarAvailable(adf_hw_device_data_t *self, Cpa32U barId)
{
    /* isBarAvailable shall return FALSE
    for barId 0 */
    STATIC CpaBoolean bars[] = {CPA_FALSE, CPA_TRUE, CPA_TRUE};

    return (barId < (sizeof (bars) / sizeof (*bars))) ?
        bars[barId]                                   :
        CPA_FALSE;
}

/*
 * irqGetBankNumber
 * Function returns the bank number based on interrupt source(sintpf)
 */
STATIC Cpa32U
adf_c2xxx_irqGetBankNumber(adf_hw_device_data_t *self,
            Cpa32U sintpf)
{
    return irq_get_bank_number(sintpf);
}

/*
 * irqGetAeSource
 * Function returns the AE source based on interrupt source(sintpf)
 */
STATIC Cpa32U
adf_c2xxx_irqGetAeSource(adf_hw_device_data_t *self, Cpa32U sintpf)
{
    return irq_ae_source(sintpf);
}

/*
 * init
 * Function initialise internal hw data
 */
STATIC CpaStatus
adf_c2xxx_init(adf_hw_device_data_t *self, Cpa8U node_id)
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
adf_c2xxx_cleanup(adf_hw_device_data_t *self)
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
adf_c2xxx_getBankBhHandler(Cpa32U bankId)
{
    return (bankId < NUM_ELEM(adf_c2xxx_bh_bank_handlers)) ?
        adf_c2xxx_bh_bank_handlers[bankId]                 :
        NULL;
}

/*
 * getBankBhPollingHandler
 * Function returns a polling bank handler based on bankId
 */
STATIC bank_handler
adf_c2xxx_getBankBhPollingHandler(Cpa32U bankId)
{
    return (bankId < NUM_ELEM(adf_c2xxx_bh_polling_bank_handlers)) ?
        adf_c2xxx_bh_polling_bank_handlers[bankId]                 :
        NULL;
}

/*
 * getApCsrOffsets
 * This function set the auto-push register offset based on
 * the bank id.
 */
STATIC void
adf_c2xxx_getApCsrOffsets(Cpa32U bankId, Cpa32U *nfMask, Cpa32U *nfDest,
                Cpa32U *neMask, Cpa32U *neDest, Cpa32U *apDelay)
{
    if (NULL != nfMask)
    {
        *nfMask = ICP_RING_CSR_AP_NF_MASK +
                    (bankId * AP_BANK_CSR_BYTE_OFFSET);
    }

    if (NULL != nfDest)
    {
        *nfDest = ICP_RING_CSR_AP_NF_DEST +
                    (bankId * AP_BANK_CSR_BYTE_OFFSET);
    }

    if (NULL != neMask)
    {
        *neMask = ICP_RING_CSR_AP_NE_MASK +
                    (bankId * AP_BANK_CSR_BYTE_OFFSET);
    }

    if (NULL != neDest)
    {
        *neDest = ICP_RING_CSR_AP_NE_DEST +
                    (bankId * AP_BANK_CSR_BYTE_OFFSET);
    }

    if (NULL != apDelay)
    {
        *apDelay = ICP_RING_CSR_AP_DELAY;
    }
}

/*
 * getVf2PfIntSourceOffset
 * This function returns byte offset of the VF-to-PF interrupt source CSR.
 */
STATIC Cpa32U
adf_c2xxx_getVf2PfIntSourceOffset(void)
{
    return ICP_PMISCBAR_VFTOPFINTSRC;
}

/*
 * getVf2PfBitOffset
 * This function returns the VF-to-PF bit offset in the interrupt source CSR.
 */
STATIC Cpa32U
adf_c2xxx_getVf2PfBitOffset(void)
{
    return ICP_VFTOPF_INTSOURCEOFFSET;
}

/*
 * getVf2PfIntMaskOffset
 * This function returns byte offset of the VF-to-PF interrupt mask CSR.
 */
STATIC Cpa32U
adf_c2xxx_getVf2PfIntMaskOffset(void)
{
    return ICP_PMISCBAR_VFTOPFINTMSK;
}

/*
 * getVf2PfIntMaskDefault
 * This function returns the default VF-to-PF interrupt mask.
 */
STATIC Cpa32U
adf_c2xxx_getVf2PfIntMaskDefault(void)
{
    return ICP_VFTOPF_INTSOURCEMASK;
}

/*
 * getTiMiscIntCtlOffset
 * This function returns byte offset of the miscellaneous interrupt
 * control register.
 */
STATIC Cpa32U
adf_c2xxx_getTiMiscIntCtlOffset(void)
{
    return ICP_PMISCBAR_TIMISCINTCTL;
}

/*
 * getVIntMskOffset
 * This function returns byte offset of the VF interrupt mask CSR.
 */
STATIC Cpa32U
adf_c2xxx_getVIntMskOffset(Cpa32U id)
{
    return (ICP_PMISCBAR_VINTMSK_OFFSET + (id * ICP_PMISCBAR_VFDEVOFFSET));
}

/*
 * getVIntMskDefault
 * This function returns the default VF interrupt mask.
 */
STATIC Cpa32U
adf_c2xxx_getVIntMskDefault(void)
{
    return ICP_PMISCBAR_VINTMSK_DEFAULT;
}

/*
 * getPf2VfDbIntOffset
 * This function returns byte offset of the doorbell and interrupt CSR
 */
STATIC Cpa32U
adf_c2xxx_getPf2VfDbIntOffset(Cpa32U id)
{
    return (ICP_PMISCBAR_PF2VFDBINT_OFFSET + (id * ICP_PMISCBAR_VFDEVOFFSET));
}

/*
 * adf_c2xxx_getResetOffsets
 * Retrieve reset offset and vector
 */
STATIC void
adf_c2xxx_getResetOffsets(Cpa32U *resetOffset, Cpa32U *resetVector)
{
	if (NULL != resetOffset) {
        *resetOffset = ICP_C2XXX_RESET_OFFSET;
	}
	if (NULL != resetVector) {
        *resetVector = ICP_C2XXX_RESET_VECTOR;
	}
}

/*
 * adf_c2xxx_enableUncoErr_interrupts
 * Enable all the supported uncorrectable error sources and their
 * logging/debug abilities
 */

STATIC CpaStatus
adf_c2xxx_enableUncoErr_interrupts(icp_accel_dev_t *accel_dev)
{
    Cpa8U i = 0;
    void *pmisc_bar_addr = NULL;
    volatile Cpa32U temp_reg = 0;
    Cpa32U numAEs = 0;
    adf_hw_device_data_t *hw_data = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(accel_dev->pHwDeviceData);
    hw_data = accel_dev->pHwDeviceData;

    /* Get the number of accelerator engines */
    numAEs = hw_data->getNumAccelEngines(hw_data, accel_dev->aeMask);

    /* Retrieve the pmiscbar address */
    pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);

    /* Setup interrupt mask */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_ERRMSK0,
        ICP_C2XXX_ERRMSK0_UERR); /* AE1-AE0 */

    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_ERRMSK3,
        ICP_C2XXX_ERRMSK3_UERR); /* MISC */

    /*
     * Setup Error Sources
     */
    for(i = 0; i < numAEs; i++)
    {
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_PMISCBAR_CTX_ENABLES(i),
            ICP_C2XXX_ENABLE_ECC_ERR); /* AE(i) */

        /*@note: this code enable ECC and parity AE hw correction.
         * The driver is enabling them for the FW/HW but the handling of those
         * correction is not covered by the driver*/
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_PMISCBAR_MISC_CONTROL(i),
                ICP_C2XXX_ENABLE_ECC_PARITY_CORR); /*AEi*/
    }
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_INTMASKSSM,
        ICP_C2XXX_INTMASKSSM_UERR); /* Accelerator 0 */

    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_CPP_CAP_ERR_CTRL,
        ICP_C2XXX_CPP_UE); /* CPP */

    /* Check if signal misc errors are set and interrupts routed to IA*/
    temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_C2XXX_SMIAOF);
    if (!(temp_reg & ICP_C2XXX_AE_IRQ_MASK)) {
        ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_SMIAOF,
            ICP_C2XXX_AE_IRQ_MASK);
    }

    /*
     * Enable Logging
     */
    /* Shared Memory */
    temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_C2XXX_UERRSSMSH) |
        ICP_C2XXX_UERRSSMSH_EN;
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_UERRSSMSH, temp_reg);
    /*Shared Memory Ecc Error Correction*/
    temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_C2XXX_CERRSSMSH) |
        ICP_C2XXX_CERRSSMSH_EN;
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_CERRSSMSH, temp_reg);
    /* MMP */
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_UERRSSMMMP0,
        ICP_C2XXX_UERRSSMMMP_EN);
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_UERRSSMMMP1,
        ICP_C2XXX_UERRSSMMMP_EN);
    /* MMP ECC correction*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_CERRSSMMMP0,
        ICP_C2XXX_CERRSSMMMP_EN);
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_CERRSSMMMP1,
        ICP_C2XXX_CERRSSMMMP_EN);
    /*Push/Pull*/
    temp_reg = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_C2XXX_PPERR) |
        ICP_C2XXX_PPERR_EN;
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_PPERR, temp_reg);
    /*Push/Pull Misc*/
    ICP_ADF_CSR_WR(pmisc_bar_addr, ICP_C2XXX_TICPPINTCTL,
        ICP_C2XXX_TICPP_EN);

    return CPA_STATUS_SUCCESS;
}

/*
 * adf_c2xxx_isr_LogAE_Int
 * Log accel engine based uncorrectable errors
 */
STATIC inline void
adf_c2xxx_isr_LogAE_Int(Cpa8U index, void *pmisc_bar_addr)
{
    Cpa32U ustore = ICP_ADF_CSR_RD(pmisc_bar_addr,
        ICP_C2XXX_USTORE_ERROR_STATUS(index));
    Cpa32U reg_error = ICP_ADF_CSR_RD(pmisc_bar_addr,
        ICP_C2XXX_REG_ERROR_STATUS(index));
    Cpa32U eperrlog = ICP_ADF_CSR_RD(pmisc_bar_addr,
        ICP_C2XXX_EPERRLOG);
    Cpa32U ctx_enables = ICP_ADF_CSR_RD(pmisc_bar_addr,
        ICP_C2XXX_PMISCBAR_CTX_ENABLES(index));

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
        ICP_C2XXX_EPERRLOG_ERR(
            ICP_C2XXX_GET_EPERRLOG(eperrlog, index)),
        ICP_C2XXX_USTORE_IS_UE(ustore),
        ICP_C2XXX_USTORE_UADDR(ustore),
        ICP_C2XXX_USTORE_GET_SYN(ustore),
        ICP_C2XXX_CTX_ECC_ERR(ctx_enables),
        ICP_C2XXX_CTX_PAR_ERR(ctx_enables),
        ICP_C2XXX_REG_GET_REG_AD(reg_error),
        ICP_C2XXX_REG_TYP_STR(ICP_C2XXX_REG_TYPE(reg_error)));
}

/*
 * adf_c2xxx_isr_LogSSM_Int
 * Log SSM based uncorrectable errors (shared Mem, MMP and Push/Pull)
 */
STATIC inline CpaBoolean
adf_c2xxx_isr_LogSSM_Int(void *pmisc_bar_addr,
    Cpa8U mmp, Cpa8U type)
{
    Cpa32U uerrssm = 0, uerrssmad = 0;
    Cpa32U uerrssm_off = 0, uerrssmad_off = 0;

    if (ICP_C2XXX_PPERR_T == type) {
        Cpa32U pperr = 0, pperrid = 0;
        pperr = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_C2XXX_PPERR);
        pperrid = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_C2XXX_PPERRID);
        ADF_DEBUG("pperr 0x%X pperrid 0x%X\n", pperr, pperrid);
        ADF_ERROR("Push/Pull Error - Accelerator 0\n"
            "%s - Error type: %s\n"
            "id 0x%X\n",
            ICP_C2XXX_PPERR_GET_STATUS(pperr),
            ICP_C2XXX_PPERR_TYPE_STR(
                ICP_C2XXX_PPERR_TYPE(pperr)),
            pperrid);
        return CPA_TRUE;
    }
    else if (ICP_C2XXX_SHAREDMEM_T == type) {
        uerrssm_off = ICP_C2XXX_UERRSSMSH;
        uerrssmad_off = ICP_C2XXX_UERRSSMSHAD;
    }
    else {
        /*MMP*/
        switch(mmp) {
        case 1:
            {
                uerrssm_off = ICP_C2XXX_UERRSSMMMP1;
                uerrssmad_off = ICP_C2XXX_UERRSSMMMPAD1;
                break;
            }
        case 0:
        default:
            {
                uerrssm_off = ICP_C2XXX_UERRSSMMMP0;
                uerrssmad_off = ICP_C2XXX_UERRSSMMMPAD0;
                break;
            }
        }
    }
    uerrssm = ICP_ADF_CSR_RD(pmisc_bar_addr, uerrssm_off);
    uerrssmad = ICP_ADF_CSR_RD(pmisc_bar_addr, uerrssmad_off);

    if (ICP_C2XXX_MMP_T == type) {
        /*MMP*/
        ADF_DEBUG("uerrssmmmp 0x%X, uerrssmmmpad 0x%X\n",
            uerrssm, uerrssmad);
        ADF_ERROR("MMP %i Accelerator 0 - Uncorrectable Error: %s\n"
            "Operation: %s - Address: 0x%X\n",
            mmp, (((uerrssm & ICP_C2XXX_UERRSSMMMP_UERR))!= 0 ? "Yes": "No"),
            ICP_C2XXX_UERRSSMMMP_ERRTYPE_STR(
                ICP_C2XXX_UERRSSMMMP_ERRTYPE(uerrssm)),
            (uerrssmad & ICP_C2XXX_UERRSSMMMPAD_ADDR));
        return (uerrssm & ICP_C2XXX_UERRSSMMMP_UERR)!= 0
            ? CPA_TRUE: CPA_FALSE;
    }
    else {
        ADF_DEBUG("uerrssmsh 0x%X, uerrssmshad 0x%X\n",
            uerrssm, uerrssmad);
        ADF_ERROR("SharedMem Error Accelerator 0 - Uncorrectable Error: %s\n"
            "Operation: %s - Type: %s - Address: 0x%X\n",
            (uerrssm & ICP_C2XXX_UERRSSMSH_UERR) != 0? "Yes": "No",
            ICP_C2XXX_UERRSSMSH_GET_OP(uerrssm),
            ICP_C2XXX_UERRSSMSH_GET_ERRTYPE(
                ICP_C2XXX_UERRSSMSH_ERRTYPE(uerrssm)),
            (uerrssmad & ICP_C2XXX_UERRSSMSHAD_ADDR));
        return (uerrssm & ICP_C2XXX_UERRSSMSH_UERR) != 0? CPA_TRUE: CPA_FALSE;
    }
}

/*
 * adf_c2xxx_handleUncoErr_Interrupts
 * Handle AE interrupt sources: Uncorrectable errors and firmware custom
 */
STATIC CpaBoolean
adf_c2xxx_handleUncoErr_Interrupts(icp_accel_dev_t *accel_dev)
{
    void *pmisc_bar_addr = adf_isr_getMiscBarAd(accel_dev);
    CpaBoolean reset_needed = CPA_FALSE;

    Cpa32U errsou0 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_C2XXX_ERRSOU0);
    Cpa32U errsou3 = ICP_ADF_CSR_RD(pmisc_bar_addr, ICP_C2XXX_ERRSOU3);

    /*UERR interrupts ?*/
    if ((0 != errsou0) || (0 != errsou3)) {
        ADF_DEBUG("UERR interrupt occurred");
        /*Identify the interrupt Sources and log them*/
        /*AE0-AE1 Errors*/
        if (0 != errsou0) {
            if (errsou0 & ICP_C2XXX_M0UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_c2xxx_isr_LogAE_Int(ICP_C2XXX_AE0, pmisc_bar_addr);
            }
            if (errsou0 & ICP_C2XXX_M1UNCOR_MASK) {
                reset_needed = CPA_TRUE;
                adf_c2xxx_isr_LogAE_Int(ICP_C2XXX_AE1, pmisc_bar_addr);
            }
        }
        if (0 != errsou3) {
            /*Accelerator: MMP, CAP, PUSH/PULL errors*/
            if (errsou3 & ICP_C2XXX_EMSK3_ACCEL_MASK) {
                Cpa32U instatssm = 0;
                instatssm = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_C2XXX_INTSTATSSM);

                ADF_DEBUG("Possible Uncorrectable Error Occurred"
                    " on Accelerator 0\n");
                ADF_DEBUG("intstatssm 0x%X\n", instatssm);
                if (instatssm & ICP_C2XXX_INTSTATSSM_MMP1) {
                    reset_needed = adf_c2xxx_isr_LogSSM_Int(pmisc_bar_addr,
                        ICP_C2XXX_SLICE1, ICP_C2XXX_MMP_T);
                }
                if (instatssm & ICP_C2XXX_INTSTATSSM_MMP0) {
                    reset_needed = adf_c2xxx_isr_LogSSM_Int(pmisc_bar_addr,
                        ICP_C2XXX_SLICE0, ICP_C2XXX_MMP_T);
                }
                if (instatssm & ICP_C2XXX_INTSTATSSM_SH) {
                    reset_needed = adf_c2xxx_isr_LogSSM_Int(pmisc_bar_addr,
                        ICP_C2XXX_SLICE0/*not used*/,
                        ICP_C2XXX_SHAREDMEM_T);
                }
                if (instatssm & ICP_C2XXX_INTSTATSSM_PPERR) {
                    reset_needed = adf_c2xxx_isr_LogSSM_Int(pmisc_bar_addr,
                        ICP_C2XXX_SLICE0/*not used*/,
                        ICP_C2XXX_PPERR_T);
                }
            }
            /*Or'ed CAP Data Error*/
            if (errsou3 & ICP_C2XXX_EMSK3_CAP_MASK) {
                Cpa32U err_status = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_C2XXX_CPP_ERR_STATUS);
                Cpa32U err_ppid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_C2XXX_CPP_SHAC_ERR_PPID);
                reset_needed = CPA_TRUE;
                /*Log error*/
                ADF_DEBUG("ccp_shac_err_status 0x%X -"
                    "cpp_shac_err_ppid 0x%X\n", err_status, err_ppid);
                ADF_ERROR("Uncorrectable Data Error Occurred in SHaC\n"
                    "Operation type: %s, Error target %s\n"
                    "Error ppid 0x%X\nInterrupt Triggered %s\n",
                    ICP_C2XXX_CPP_GET_TYP(err_status) == 0 ?
                    "Push": "Pull",
                    ICP_C2XXX_CPP_ERTYP_STR(
                    ICP_C2XXX_CPP_GET_ERTYP(err_status)),
                    err_ppid,
                    ICP_C2XXX_CPP_SHAC_GET_INT(err_status) == 1 ?
                    "Yes": "No");
            }
            /*Push/Pull*/
            if (errsou3 & ICP_C2XXX_PPMISCERR_MASK) {
                Cpa32U ticppintsts = ICP_ADF_CSR_RD(pmisc_bar_addr,
                    ICP_C2XXX_TICPPINTSTS);
                Cpa32U tierrid = 0;
                if(ticppintsts & ICP_C2XXX_TICPP_PUSH) {
                    tierrid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_C2XXX_TIERRPUSHID);
                }
                else {
                    tierrid = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        ICP_C2XXX_TIERRPULLID);
                }
                reset_needed = CPA_TRUE;
                ADF_DEBUG("ticppintsts 0%X, tierrxxxid 0x%X\n",
                    ticppintsts, tierrid);
                ADF_ERROR("Uncorrectable Push/Pull Error\n"
                    "Bus Operation Type %s - Id 0x%X\n",
                    ICP_C2XXX_TICPP_GETOP(ticppintsts), tierrid);
            }
        } /* ERRSOU3*/
    }

    return reset_needed;
}

void
adf_c2xxx_set_hw_data(void *hw_data)
{
    register adf_hw_device_data_t *hw = NULL;

    ICP_CHECK_FOR_NULL_PARAM_VOID(hw_data);

    hw = (adf_hw_device_data_t *)hw_data;

    hw->init      = adf_c2xxx_init;
    hw->cleanup   = adf_c2xxx_cleanup;

    hw->dev_class = &c2xxx_class;

    /* populate bank/ac information */
    hw->maxBars                 = ICP_C2XXX_MAX_PCI_BARS;
    hw->maxNumBanks             = ICP_C2XXX_ETR_MAX_BANKS;
    hw->maxNumApBanks           = ICP_C2XXX_ETR_MAX_AP_BANKS;
    hw->numRingsPerBank         = ICP_C2XXX_ETR_MAX_RINGS_PER_BANK;
    hw->numBanksPerAccel        = ICP_C2XXX_BANKS_PER_ACCELERATOR;
    hw->maxNumAccel             = ICP_C2XXX_MAX_ACCELERATORS;
    hw->maxNumAccelEngines      = ICP_C2XXX_MAX_ACCELENGINES;
    hw->aeClockInMhz            = ICP_C2XXX_AE_CLOCK_IN_MHZ;
    hw->sintpfOffset            = ICP_C2XXX_SINTPF;
    hw->userEtringCsrSize       = ICP_C2XXX_USER_ETRING_CSR_SIZE;
    hw->maxNumVf                = ICP_C2XXX_MAX_NUM_VF;

    /* populate msix information */
    hw->msix.banksVectorStart   = ICP_C2XXX_MSIX_BANK_VECTOR_START;
    hw->msix.banksVectorNum     = ICP_C2XXX_MSIX_BANK_VECTOR_NUM;
    hw->msix.aeVectorStart      = ICP_C2XXX_MSIX_AE_VECTOR_START;
    hw->msix.aeVectorNum        = ICP_C2XXX_MSIX_AE_VECTOR_NUM;

    hw->isMemScrubSupported     = CPA_TRUE;

    /* device fuse */
    hw->getDevFuseOffset        = adf_c2xxx_getDevFuseOffset;
    hw->getDevClkOffset         = NULL;
    hw->getDevSKU               = adf_c2xxx_getDevSKU;
    hw->getAcceleratorsMask     = adf_c2xxx_getAcceleratorsMask;
    hw->getAccelEnginesMask     = adf_c2xxx_getAccelEnginesMask;
    hw->getNumAccelerators      = adf_c2xxx_getNumAccelerators;
    hw->getNumAccelEngines      = adf_c2xxx_getNumAccelEngines;
    hw->getAccelMaskList        = adf_c2xxx_getAccelMaskList;

    /* accelerator functions */
    hw->getAccFuncOffset        = adf_c2xxx_getAccFuncOffset;
    hw->getAccFuncMask          = adf_c2xxx_getAccFuncMask;

    /* populate bars callbacks  */
    hw->getEtrBarId             = adf_c2xxx_getEtrBarId;
    hw->getSramBarId            = NULL;
    hw->getMiscBarId            = adf_c2xxx_getMiscBarId;
    hw->isBarAvailable          = adf_c2xxx_isBarAvailable;

    /* populate esram callbacks    */
    hw->getEsramInfo            = NULL;

    /* populate SHAC callback */
    hw->getScratchRamInfo       = NULL;

    /* populate PMiscBar callback  */
    hw->getSmiapfOffsetMask     = adf_c2xxx_getSmiapfOffsetMask;

    hw->irqGetBankNumber        = adf_c2xxx_irqGetBankNumber;
    hw->irqGetAeSource          = adf_c2xxx_irqGetAeSource;

    hw->irqGetBunSource       = NULL;
    hw->irqGetPF2VFSource     = NULL;

    hw->getBankBhHandler        = adf_c2xxx_getBankBhHandler;
    hw->getBankBhPollingHandler = adf_c2xxx_getBankBhPollingHandler;

    /* Auto-push feature callback */
    hw->isAutoPushSupported     = CPA_TRUE;
    hw->getApCsrOffsets         = adf_c2xxx_getApCsrOffsets;

    hw->getVf2PfIntSourceOffset = adf_c2xxx_getVf2PfIntSourceOffset;
    hw->getVf2PfBitOffset       = adf_c2xxx_getVf2PfBitOffset;
    hw->getVf2PfIntMaskOffset   = adf_c2xxx_getVf2PfIntMaskOffset;
    hw->getVf2PfIntMaskDefault  = adf_c2xxx_getVf2PfIntMaskDefault;

    hw->getTiMiscIntCtlOffset   = adf_c2xxx_getTiMiscIntCtlOffset;
    hw->getVIntSrcOffset        = NULL;
    hw->getVIntMskOffset        = adf_c2xxx_getVIntMskOffset;
    hw->getVIntMskDefault       = adf_c2xxx_getVIntMskDefault;
    hw->getPf2VfDbIntOffset     = adf_c2xxx_getPf2VfDbIntOffset;

    hw->getResetOffsets         = adf_c2xxx_getResetOffsets;

    hw->enableUncoErrInterrupts = adf_c2xxx_enableUncoErr_interrupts;
    hw->handleUncoErrInterrupts = adf_c2xxx_handleUncoErr_Interrupts;

    return;
}
