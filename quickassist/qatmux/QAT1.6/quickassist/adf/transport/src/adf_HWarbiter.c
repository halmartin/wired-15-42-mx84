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
 * @file adf_HWarbiter.c
 *
 * @description
 *      Hardware arbiter functionality
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_dev_ring.h"
#include "adf_init.h"
#include "adf_ETring_ap.h"
#include "adf_arb.h"
#include "icp_adf_debug.h"
#include "adf_platform_common.h"

#define WIRELESS_ENABLED "WirelessEnabled"

#define MISC_CTRL_ARB_MODE_ENABLE_MASK          0x08


/* Worker thread to service arbiter mappings based on SKUs */
#define ICP_ARB_SKU4CPM_MAP {0x12222AAA, 0x11666666, 0x12222AAA, 0x11666666,    \
    0x12222AAA, 0x11222222, 0x12222AAA, 0x11222222, 0x00000000, 0x00000000,    \
    0x00000000,  0x00000000}

#define ICP_ARB_SKU6CPM_MAP {0x12222AAA, 0x11666666, 0x12222AAA, 0x11666666,    \
    0x12222AAA, 0x11222222, 0x12222AAA, 0x11222222, 0x12222AAA, 0x11222222,    \
    0x12222AAA,  0x11222222}

#define ICP_ARB_SKU_DC_ONLY_MAP {0x00000888, 0x00000000, 0x00000888, 0x00000000, \
    0x00000888, 0x00000000, 0x00000888, 0x00000000, 0x00000888, 0x00000000,    \
    0x00000888,  0x00000000}

#define ICP_ARB_SKU4CPM_WIRELESS_MAP {0x22222222, 0x22222222, 0x22222222,0x22222222,   \
    0x22222222, 0x22222222, 0x22222222, 0x22222222, 0x00000000, 0x00000000,    \
    0x00000000,  0x00000000}

#define ICP_ARB_SKU6CPM_WIRELESS_MAP {0x22222222, 0x22222222, 0x22222222,0x22222222,   \
    0x22222222, 0x22222222, 0x22222222, 0x22222222, 0x22222222, 0x22222222,    \
    0x22222222,  0x22222222}


/* Arbiter offset within the ETR bar */
#define ICP_ARB_OFFSET_START 0x30000
/* Arbiter BAR */
#define ICP_ARB_BAR csr_base_addr + ICP_ARB_OFFSET_START
/* Arbiter slot size */
#define ICP_ARB_REG_SLOT    0x1000

/* Offset starts on per register basis */
#define ICP_ARB_SARB_CFG_OFFSET_START         0x000
#define ICP_ARB_WTR_OFFSET_START              0x010
#define ICP_ARB_RO_EN_OFFSET_START            0x090
#define ICP_ARB_WQCFG_OFFSET_START            0x100
#define ICP_ARB_WQSTAT_OFFSET_START           0x140
#define ICP_ARB_WRKTHD2SARBMAP_OFFSET_START   0x180
#define ICP_ARB_RINGSRVARBEN_OFFSET_START     0x19C


/* Bitmasks for SARB_CONFIG CSRs */
#define ICP_ARB_FCEN_BITMASK    0x80000000
#define ICP_ARB_THOLD_BITMASK   0x000000F0
#define ICP_ARB_PUTLEN_BITMASK  0x00000003


/* Macros for reading/writing to the arbitration registers */
#define READ_CSR_ARB_RINGSRVARBEN(csr_base_addr, index)                        \
    ICP_ADF_CSR_RD(csr_base_addr, ICP_ARB_RINGSRVARBEN_OFFSET_START +          \
            ICP_ARB_REG_SLOT * index );

#define WRITE_CSR_ARB_RINGSRVARBEN(csr_base_addr, index, value)                \
    ICP_ADF_CSR_WR(csr_base_addr, ICP_ARB_RINGSRVARBEN_OFFSET_START +          \
            ICP_ARB_REG_SLOT * index, value);

#define READ_CSR_ARB_RESPORDERING(csr_base_addr, index)                        \
    ICP_ADF_CSR_RD(ICP_ARB_BAR, ICP_ARB_RO_EN_OFFSET_START +                   \
                    ICP_ARB_REG_SIZE * index);

#define WRITE_CSR_ARB_RESPORDERING(csr_base_addr, index, value)                \
    ICP_ADF_CSR_WR(ICP_ARB_BAR, ICP_ARB_RO_EN_OFFSET_START +                   \
            ICP_ARB_REG_SIZE * index, value);

#define READ_CSR_ARB_WEIGHT(csr_base_addr, arbNumber, index)                   \
    ICP_ADF_CSR_RD(ICP_ARB_BAR, ICP_ARB_WTR_OFFSET_START +                     \
            (ICP_ARB_WTR_SIZE * arbNumber) + (ICP_ARB_REG_SIZE * index));

#define WRITE_CSR_ARB_WEIGHT(csr_base_addr, arbNumber, index, value)           \
    ICP_ADF_CSR_WR(ICP_ARB_BAR, ICP_ARB_WTR_OFFSET_START +                     \
            (ICP_ARB_WTR_SIZE * arbNumber) + (ICP_ARB_REG_SIZE * index), value);

#define READ_CSR_ARB_SARBCONFIG(csr_base_addr, arbNumber)                      \
    ICP_ADF_CSR_RD(ICP_ARB_BAR,                                                \
            ICP_ARB_SARB_CFG_OFFSET_START + ICP_ARB_REG_SIZE * arbNumber);

#define WRITE_CSR_ARB_SARCONFIG(csr_base_addr, arbNumber, value)               \
    ICP_ADF_CSR_WR(ICP_ARB_BAR,                                                \
            ICP_ARB_SARB_CFG_OFFSET_START + ICP_ARB_REG_SIZE * arbNumber,      \
            value );

#define READ_CSR_ARB_WRKTHD2SRVARBMAP(csr_base_addr, index)                    \
    ICP_ADF_CSR_RD(ICP_ARB_BAR,                                                \
                ICP_ARB_WRKTHD2SARBMAP_OFFSET_START + ICP_ARB_REG_SIZE * index);

#define WRITE_CSR_ARB_WRKTHD2SRVARBMAP(csr_base_addr, index, value)            \
    ICP_ADF_CSR_WR(ICP_ARB_BAR,                                                \
            ICP_ARB_WRKTHD2SARBMAP_OFFSET_START + ICP_ARB_REG_SIZE * index,    \
            value);

#define READ_CSR_ARB_WQCFG(csr_base_addr, index)                               \
    ICP_ADF_CSR_RD(ICP_ARB_BAR, ICP_ARB_WQCFG_OFFSET_START +                   \
            ICP_ARB_REG_SIZE * index);

#define WRITE_CSR_ARB_WQCFG(csr_base_addr, index, value)                       \
    ICP_ADF_CSR_WR(ICP_ARB_BAR, ICP_ARB_WQCFG_OFFSET_START +                   \
            ICP_ARB_REG_SIZE * index, value);

#define READ_CSR_ARB_WQSTAT(csr_base_addr, WorkerQueueNumber)                  \
    ICP_ADF_CSR_RD(ICP_ARB_BAR,                                                \
            ICP_ARB_WQSTAT_OFFSET_START + ICP_ARB_REG_SIZE * WorkerQueueNumber);

#define READ_CSR_MISC_CTRL(csr_base_addr, offset)                              \
    ICP_ADF_CSR_RD(csr_base_addr, offset);

#define WRITE_CSR_MISC_CTRL(csr_base_addr, offset, value)                      \
    ICP_ADF_CSR_WR(csr_base_addr, offset, value);
/*
 * Arb debug filenames
 */

#define DEBUG_ARBITER_NAME              "arbiter"
#define DEBUG_ARBITER_NAME_LEN          8
#define NUMBER_WQ_ENTRIES               12
#define NUMBER_BITS_PER_WQ_ENTRIES      8
#define NUMBER_BITS_PER_REG             32
/*
* @description
* Configuration of the service arbitration hardware
* This is called by adf_arbInit
*/
STATIC CpaStatus adf_arbConfig(icp_accel_dev_t *accel_dev);

/*
* @description
* Enable or disable response ordering for all bundles &
* rings associated with the acceleration device.
* This is called by adf_arbInit
*/
STATIC CpaStatus adf_arbConfigResponseOrdering(icp_accel_dev_t *accel_dev);

/*
* @description
* Statically configure the default weights for the service
* arbitration hardware of the acceleration device
* This is called by adf_arbInit
*/
STATIC CpaStatus adf_arbConfigWeight(icp_accel_dev_t *accel_dev);

/*
* @description
* Statically configure the default assignment of worker queues to AEs
* This is called by adf_arbInit
*/
STATIC CpaStatus adf_arbConfigWorkQueues(icp_accel_dev_t *accel_dev);

/*
* @description
* Statically configure the threading model of how the 96 worker threads
* are mapped to the four service arbiters
* This is called by adf_arbInit
*/
STATIC CpaStatus adf_arbWrkThrdToSArbMap(icp_accel_dev_t *accel_dev);


/*
* @description
* Init for the debug for the arbitration unit.  Creates a file on the
* proc file system to allow us to debug the status of the unit.
*/
STATIC CpaStatus adf_arbDebugInit(icp_accel_dev_t *device);

/*
* @description
* Debug for the arbitration unit - read the configuration and the
* current status of the unit.
*/
STATIC int adf_arbDebugRead(void* private_data, char* data, int size,
                            int offset);

/*
* @description
* Clean the debug information associated with the arbitration unit.
*/
STATIC CpaStatus adf_arbDebugRemove(icp_accel_dev_t *icp_accel_dev);


STATIC void adf_arbConfigMiscCtrl(icp_accel_dev_t *icp_accel_dev);

/* @description
 * Insert the first size bits of x into y, starting at offset of y
 */
STATIC inline void adf_insert_bits(Cpa8U size, Cpa32U x, Cpa8U offset,
    Cpa32U *y)
{
    Cpa32U width_mask = (2 << size) - 1;
    Cpa32U mask = ~0L ^ (width_mask << offset);

    *y = ((x & width_mask) << offset) | (*y & mask);
}

CpaStatus adf_arbInit(icp_accel_dev_t *pAccelDev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_etr_priv_data_t* privData = NULL;
    Cpa32U *csr_base_addr = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    ICP_CHECK_FOR_NULL_PARAM(csr_base_addr);

    /* 1. Setup the service arbiters */
    status = adf_arbConfig(pAccelDev);
    if(status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Service arbiter configuration failed\n");
        adf_arbShutdown(pAccelDev);
        return CPA_STATUS_FAIL;
    }

    /* 2. Setup the service weighting */
    status = adf_arbConfigWeight(pAccelDev);
    if(status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Weight configuration failed for service arbiter \n");
        adf_arbShutdown(pAccelDev);
        return CPA_STATUS_FAIL;
    }

    /* 3. Setup response ordering */
    status = adf_arbConfigResponseOrdering(pAccelDev);

    if(status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Response ordering configuration failed\n");
        adf_arbShutdown(pAccelDev);
        return CPA_STATUS_FAIL;
    }

    /* 4. WQCFG setup */
    status = adf_arbConfigWorkQueues(pAccelDev);

    if(status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Work queue configuration failed\n");
        adf_arbShutdown(pAccelDev);
        return CPA_STATUS_FAIL;
    }

    /* 5. Map worker threads to service arbiters */
    status = adf_arbWrkThrdToSArbMap(pAccelDev);

    if(status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Worker thread to service arbiter mapping failed\n");
        adf_arbShutdown(pAccelDev);
        return CPA_STATUS_FAIL;
    }

    /* 6. Setup the /proc arbiter debug file */
    status = adf_arbDebugInit(pAccelDev);
    if(status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Arbiter debug init failed\n");
        adf_arbShutdown(pAccelDev);
        return CPA_STATUS_FAIL;
    }

    /* 7. Set the appropriate bits in the Misc Control Register */
    adf_arbConfigMiscCtrl(pAccelDev);

    return status;
}


CpaStatus adf_arbEnableRing(icp_accel_dev_t *pAccelDev,
        icp_et_ring_data_t *pRing)
{
    Cpa32U ringSrvArbEn = 0;
    Cpa32U ringEnable = 1;
    icp_etr_priv_data_t* privData = NULL;
    icp_et_ring_bank_data_t* bank_to_lock = NULL;
    Cpa32U *csr_base_addr = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pRing);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    ICP_CHECK_FOR_NULL_PARAM(csr_base_addr);

    if(pRing->ringNumber >= ICP_ARB_REQ_RING_NUM )
    {
        ADF_ERROR("invalid ring num, request ring num should be between 0-7\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* get the bank by using the bankNumber, lock the bank which contains the */
    /* ring we'll operate on                                                  */
    bank_to_lock = &privData->banks[pRing->bankNumber];
    ICP_CHECK_FOR_NULL_PARAM(bank_to_lock);
    ICP_MUTEX_LOCK(&bank_to_lock->arbCsrLock);

    /* ringSrvArbEn is 0:31, select the correct register based on bank number */
    ringSrvArbEn = READ_CSR_ARB_RINGSRVARBEN(csr_base_addr, pRing->bankNumber);

    /* Write the correct bit based on the ringNumber */
    adf_insert_bits(0, ringEnable, pRing->ringNumber, &ringSrvArbEn);
    WRITE_CSR_ARB_RINGSRVARBEN(csr_base_addr, pRing->bankNumber, ringSrvArbEn);

    ICP_MUTEX_UNLOCK(&bank_to_lock->arbCsrLock);

    return CPA_STATUS_SUCCESS;

}

CpaStatus adf_arbDisableRing(icp_accel_dev_t *pAccelDev,
        icp_et_ring_data_t *pRing)
{
    Cpa32U ringSrvArbEn = 0;
    Cpa32U ringDisable = 0;
    icp_etr_priv_data_t* privData = NULL;
    icp_et_ring_bank_data_t* bank_to_lock = NULL;
    Cpa32U *csr_base_addr = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pRing);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    ICP_CHECK_FOR_NULL_PARAM(csr_base_addr);

    if(pRing->ringNumber >= ICP_ARB_REQ_RING_NUM )
    {
        ADF_ERROR("invalid ring num, request ring num should be between 0-7\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* get the bank by using the bankNumber, lock the bank which contains the */
    /* ring we'll operate on                                                  */
    bank_to_lock = &privData->banks[pRing->bankNumber];
    ICP_CHECK_FOR_NULL_PARAM(bank_to_lock);
    ICP_MUTEX_LOCK(&bank_to_lock->arbCsrLock);

    /* ringSrvArbEn is 0:31, select the correct register based on bank number */
    ringSrvArbEn = READ_CSR_ARB_RINGSRVARBEN(csr_base_addr, pRing->bankNumber);

    /* Write the correct bit based on the ringNumber */
    adf_insert_bits(0, ringDisable, pRing->ringNumber, &ringSrvArbEn);

    WRITE_CSR_ARB_RINGSRVARBEN(csr_base_addr, pRing->bankNumber, ringSrvArbEn);

    ICP_MUTEX_UNLOCK(&bank_to_lock->arbCsrLock);

    return CPA_STATUS_SUCCESS;
}

/*
 * @description
 * converts from value in arb config CSR putlen field to size of msg in bytes
 */
STATIC Cpa32U arbPutlenToBytes(Cpa32U putlen)
{
    Cpa32U bytes = 0;

    switch (putlen){
    case ICP_ARB_PUTLEN_16B_RESP: bytes = ADF_MSG_SIZE_16_BYTES; break;
    case ICP_ARB_PUTLEN_32B_RESP: bytes = ADF_MSG_SIZE_32_BYTES; break;
    case ICP_ARB_PUTLEN_64B_RESP: bytes = ADF_MSG_SIZE_64_BYTES; break;
    default : ADF_ERROR("Arbiter Flow Control error. Invalid putlen value %d\n",
                                                                    putlen);
              break;
    }
    return bytes;
}

/*
 * Checks that Response ring msg size requested in createETHandle matches
 * value already configured in Arbiter Config CSR putlen field.
 */
CpaStatus adf_arbVerifyResponseMsgSize(icp_accel_dev_t *pAccelDev,
                                       icp_et_ring_data_t *pRing)
{
    icp_etr_priv_data_t* privData = NULL;
    Cpa32U *csr_base_addr = NULL;
    Cpa32U req_ring_nr = 0;
    Cpa32U sarb_cfg = 0;
    Cpa32U putlen = 0;
    Cpa32U arbNumber = 0;
    Cpa32U fcen = 0;
    Cpa32U threshold = 0;

    ICP_CHECK_FOR_NULL_PARAM(pRing);
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;
    ICP_CHECK_FOR_NULL_PARAM(csr_base_addr);


    if(pRing->ringNumber < ICP_ARB_REQ_RING_NUM )
    {
        ADF_ERROR("response ring num(%d) in bank should be >= %d\n",
                        pRing->ringNumber, ICP_ARB_REQ_RING_NUM);
        return CPA_STATUS_INVALID_PARAM;
    }

    if(!(pRing->msgSizeInBytes == ADF_MSG_SIZE_32_BYTES ||
                 pRing->msgSizeInBytes == ADF_MSG_SIZE_64_BYTES ))
    {
        ADF_ERROR("response ring msg size invalid(%d)\n",
                pRing->msgSizeInBytes);
        return CPA_STATUS_INVALID_PARAM;
    }

    req_ring_nr = pRing->ringNumber - ICP_ARB_REQ_RING_NUM;

    /* There's a fixed mapping between service arbiters and rings
     * so find the corresponding arbiter */
    switch (req_ring_nr)
    {
    case 0:
    case 1: arbNumber = ICP_SARB_ZERO; break;
    case 2:
    case 3: arbNumber = ICP_SARB_ONE; break;
    case 4:
    case 5: arbNumber = ICP_SARB_TWO; break;
    case 6:
    case 7: arbNumber = ICP_SARB_THREE; break;
    default: ADF_ERROR("invalid request ring num %d\n", req_ring_nr);
            return CPA_STATUS_INVALID_PARAM;
    }

    sarb_cfg = READ_CSR_ARB_SARBCONFIG(csr_base_addr, arbNumber);

    /* parse the CSR content */
    fcen = sarb_cfg & ICP_ARB_FCEN_BITMASK;
    putlen = sarb_cfg & ICP_ARB_PUTLEN_BITMASK;

    if (fcen)
    {
         /* flow control is enabled so check if config is correct */
        if (arbPutlenToBytes(putlen) != pRing->msgSizeInBytes)
        {
            ADF_ERROR("Arbiter Flow Control cfg(%d) doesn't match response ring size(%d)\n",
                        putlen, pRing->msgSizeInBytes);
            /* return CPA_STATUS_FAIL; */

            ADF_PRINT("Overriding arbiter putlen config to match response msg size\n");
            ADF_PRINT("This will affect all response rings associated with arbiter %d\n", arbNumber);

            if (pRing->msgSizeInBytes == ADF_MSG_SIZE_32_BYTES)
            {
                threshold = ICP_ARB_THOLD_32B_RESP;
                putlen = ICP_ARB_PUTLEN_32B_RESP;
            }
            else if (pRing->msgSizeInBytes == ADF_MSG_SIZE_64_BYTES)
            {
                threshold = ICP_ARB_THOLD_64B_RESP;
                putlen = ICP_ARB_PUTLEN_64B_RESP;
            }
            else
            {
                ADF_ERROR("Unsupported response ring size (%d) - no override done\n",
                                        pRing->msgSizeInBytes);
                return CPA_STATUS_SUCCESS;
            }
            sarb_cfg = 0;

            fcen = 1;
            adf_insert_bits(2, putlen, 0, &sarb_cfg);
            adf_insert_bits(4, threshold, 4, &sarb_cfg);
            adf_insert_bits(1, fcen, 31, &sarb_cfg);

            WRITE_CSR_ARB_SARCONFIG(csr_base_addr, arbNumber, sarb_cfg);
        }
    }

    return CPA_STATUS_SUCCESS;
}


STATIC CpaStatus adf_arbConfigResponseOrdering(icp_accel_dev_t *pAccelDev)
{
    Cpa32U respOrdering = ~0;
    Cpa32U i = 0;
    icp_etr_priv_data_t* privData = NULL;
    Cpa32U *csr_base_addr = NULL;

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;

    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    /* Write to each of the 8 registers */
    for (i = 0; i < ICP_ARB_REQ_RING_NUM; i++ )
    {
        WRITE_CSR_ARB_RESPORDERING(csr_base_addr, i, respOrdering);
    }

    return CPA_STATUS_SUCCESS;
}


STATIC CpaStatus adf_arbConfig(icp_accel_dev_t *pAccelDev)
{
    Cpa32U sarb_cfg = 0;
    Cpa32U threshold = 0;
    Cpa32U putlen = 0;
    Cpa32U arbNumber = 0;
    Cpa32U msgSize = 0;
    Cpa8U fcen = 1;
    Cpa32U *csr_base_addr = NULL;
    icp_etr_priv_data_t* privData = NULL;
    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    CpaStatus status = CPA_STATUS_SUCCESS;

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    /* Build the value to be written to the register at the correct
     * bit offsets. This is made up of putlen (size of the put for the
     * service arbiter), threshold and the flow control enabling bit
     *
     * Service arbiter 1: PKE
     * Service arbiter 2: CY
     * Service arbiter 3: TRNG
     * Service arbiter 4: DC
     *
     * We only support 16, 32 or 64 byte response message sizes
     */
    for(arbNumber = ICP_SARB_ZERO; arbNumber <= ICP_SARB_THREE; arbNumber++)
    {
        switch(arbNumber)
        {
            case ICP_SARB_ZERO:
                status = icp_adf_cfgGetParamValue(pAccelDev,
                        GENERAL_SEC,
                        ICP_CFG_FW_MSG_SIZE_ASYMCY_RX_KEY,
                        adfGetParam);
                if(CPA_STATUS_SUCCESS != status)
                {
                  ADF_ERROR("Problem getting ASYMCY_RX from config table\n");
                  return CPA_STATUS_FAIL;
                }
                msgSize = ICP_STRTOUL(adfGetParam, NULL, ADF_CFG_BASE_DEC);

                if(msgSize == ADF_MSG_SIZE_16_BYTES)
                {
                    threshold = ICP_ARB_THOLD_16B_RESP;
                    putlen = ICP_ARB_PUTLEN_16B_RESP;
                }
                else if(msgSize == ADF_MSG_SIZE_32_BYTES)
                {
                    threshold = ICP_ARB_THOLD_32B_RESP;
                    putlen = ICP_ARB_PUTLEN_32B_RESP;
                }
                else if (msgSize == ADF_MSG_SIZE_64_BYTES)
                {
                    threshold = ICP_ARB_THOLD_64B_RESP;
                    putlen = ICP_ARB_PUTLEN_64B_RESP;
                }
                else
                {
                    ADF_ERROR("Unable to set service  arbiter %d threshold"
                            "& putlen\n", ICP_SARB_ZERO);
                    return CPA_STATUS_FAIL;
                }
                break;
            case ICP_SARB_ONE:

                status = icp_adf_cfgGetParamValue(pAccelDev,
                        GENERAL_SEC,
                        ICP_CFG_FW_MSG_SIZE_SYMCY_RX_KEY,
                        adfGetParam);
                if(CPA_STATUS_SUCCESS != status)
                {
                  ADF_ERROR("Problem getting SYMCY_RX from config table\n");
                  return CPA_STATUS_FAIL;
                }
                msgSize = ICP_STRTOUL(adfGetParam, NULL, ADF_CFG_BASE_DEC);

                if(msgSize == ADF_MSG_SIZE_16_BYTES)
                {
                    threshold = ICP_ARB_THOLD_16B_RESP;
                    putlen = ICP_ARB_PUTLEN_16B_RESP;
                }
                else if(msgSize == ADF_MSG_SIZE_32_BYTES)
                {
                    threshold = ICP_ARB_THOLD_32B_RESP;
                    putlen = ICP_ARB_PUTLEN_32B_RESP;
                }
                else if (msgSize == ADF_MSG_SIZE_64_BYTES)
                {
                    threshold = ICP_ARB_THOLD_64B_RESP;
                    putlen = ICP_ARB_PUTLEN_64B_RESP;
                }
                else
                {
                    ADF_ERROR("Unable to set service  arbiter %d threshold"
                            "& putlen\n", ICP_SARB_ONE);
                    return CPA_STATUS_FAIL;
                }
                break;
            case ICP_SARB_TWO:
                status = icp_adf_cfgGetParamValue(pAccelDev,
                        GENERAL_SEC,
                        ICP_CFG_FW_MSG_SIZE_SYMCY_RX_KEY,
                        adfGetParam);
                if(CPA_STATUS_SUCCESS != status)
                {
                  ADF_ERROR("Problem getting SYMCY_RX (TRNG) from config"
                          "table\n");
                  return CPA_STATUS_FAIL;
                }
                msgSize = ICP_STRTOUL(adfGetParam, NULL, ADF_CFG_BASE_DEC);

                if(msgSize == ADF_MSG_SIZE_16_BYTES)
                {
                    threshold = ICP_ARB_THOLD_16B_RESP;
                    putlen = ICP_ARB_PUTLEN_16B_RESP;
                }
                else if(msgSize == ADF_MSG_SIZE_32_BYTES)
                {
                    threshold = ICP_ARB_THOLD_32B_RESP;
                    putlen = ICP_ARB_PUTLEN_32B_RESP;
                }
                else if (msgSize == ADF_MSG_SIZE_64_BYTES)
                {
                    threshold = ICP_ARB_THOLD_64B_RESP;
                    putlen = ICP_ARB_PUTLEN_64B_RESP;
                }
                else
                {
                    ADF_ERROR("Unable to set service  arbiter %d threshold"
                            "& putlen\n", ICP_SARB_TWO);
                    return CPA_STATUS_FAIL;
                }
                break;
            case ICP_SARB_THREE:
                status = icp_adf_cfgGetParamValue(pAccelDev,
                        GENERAL_SEC,
                        ICP_CFG_FW_MSG_SIZE_DC_RX_KEY,
                        adfGetParam);
                if(CPA_STATUS_SUCCESS != status)
                {
                  ADF_ERROR("Problem getting DC_RX from config table\n");
                  return CPA_STATUS_FAIL;
                }
                msgSize = ICP_STRTOUL(adfGetParam, NULL, ADF_CFG_BASE_DEC);

                if(msgSize == ADF_MSG_SIZE_16_BYTES)
                {
                    threshold = ICP_ARB_THOLD_16B_RESP;
                    putlen = ICP_ARB_PUTLEN_16B_RESP;
                }
                else if(msgSize == ADF_MSG_SIZE_32_BYTES)
                {
                    threshold = ICP_ARB_THOLD_32B_RESP;
                    putlen = ICP_ARB_PUTLEN_32B_RESP;
                }
                else if (msgSize == ADF_MSG_SIZE_64_BYTES)
                {
                    threshold = ICP_ARB_THOLD_64B_RESP;
                    putlen = ICP_ARB_PUTLEN_64B_RESP;
                }
                else
                {
                    ADF_ERROR("Unable to set service  arbiter %d threshold"
                            "& putlen\n", ICP_SARB_THREE);
                    return CPA_STATUS_FAIL;
                }
                break;
            default:
                ADF_ERROR("invalid service arbiter number\n");
                return CPA_STATUS_FAIL;
        }

        adf_insert_bits(2, putlen, 0, &sarb_cfg);
        adf_insert_bits(4, threshold, 4, &sarb_cfg);
        adf_insert_bits(1, fcen, 31, &sarb_cfg);

        WRITE_CSR_ARB_SARCONFIG(csr_base_addr, arbNumber, sarb_cfg);

    }

    return status;
}


STATIC CpaStatus adf_arbConfigWeight(icp_accel_dev_t *pAccelDev)
{
    Cpa32U i = 0;
    Cpa32U j = 0;
    Cpa32U sarb_wtr = 0xFFFFFFFF;
    icp_etr_priv_data_t* privData = NULL;
    Cpa32U *csr_base_addr = NULL;

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    /* Register is laid as out:
     * SARB_WTR[arbiter: 0-3][n: 0-7]
     */
    for( i = 0; i < ICP_ARB_NUM; i++ )
    {
        for(j = 0; j < ICP_ARB_REQ_RING_NUM; j++)
        {
            WRITE_CSR_ARB_WEIGHT(csr_base_addr, i, j, sarb_wtr);
        }
    }

    return CPA_STATUS_SUCCESS;
}

STATIC void adf_arbConfigMiscCtrl(icp_accel_dev_t *icp_accel_dev)
{
    adf_hw_device_data_t  *hw_data = NULL;
    Cpa32U *pmisc_bar_addr = NULL;
    Cpa32U pmisc_bar_id = 0;
    Cpa32U miscCtrl;

    hw_data = icp_accel_dev->pHwDeviceData;
    pmisc_bar_id = hw_data->getMiscBarId(hw_data);

    pmisc_bar_addr = (void*)
        icp_accel_dev->pciAccelDev.pciBars[pmisc_bar_id].virtAddr;

    miscCtrl= READ_CSR_MISC_CTRL(pmisc_bar_addr, hw_data->getTiMiscIntCtlOffset());
    miscCtrl |= MISC_CTRL_ARB_MODE_ENABLE_MASK;

    WRITE_CSR_MISC_CTRL(pmisc_bar_addr,hw_data->getTiMiscIntCtlOffset(),miscCtrl);
}


STATIC CpaStatus adf_arbWrkThrdToSArbMap(icp_accel_dev_t *pAccelDev)
{
    icp_etr_priv_data_t* privData = NULL;
    Cpa32U *csr_base_addr = NULL;
    Cpa32U i = 0;
    Cpa32U wrkThd2SrvArbMapConfig[12] = {0};
    Cpa32U wireless_enabled = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U wrkThd2SrvArbSKU4CPMMap[12] = ICP_ARB_SKU4CPM_MAP;
    Cpa32U wrkThd2SrvArbSKU6CPMMap[12] = ICP_ARB_SKU6CPM_MAP;
    Cpa32U wrkThd2SrvArbSKU4CPMWirelessMap[12] = ICP_ARB_SKU4CPM_WIRELESS_MAP;
    Cpa32U wrkThd2SrvArbSKU6CPMWirelessMap[12] = ICP_ARB_SKU6CPM_WIRELESS_MAP;
    Cpa32U wrkThd2SrvArbSKUDCOnlyMap[12] = ICP_ARB_SKU_DC_ONLY_MAP;


    char adfGetParam[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    status = icp_adf_cfgGetParamValue(pAccelDev,
            GENERAL_SEC,
            WIRELESS_ENABLED,
            adfGetParam);
    if(CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("Cannot find WirelessEnabled in config file, assuming "
                "no wireless processes are wanted.\n");
    }

    wireless_enabled = ICP_STRTOUL(adfGetParam, NULL, ADF_CFG_BASE_DEC);

    switch(pAccelDev->sku)
    {
        case DEV_SKU_1:
            /* 4 CPMs, 600Mhz */
            if(wireless_enabled)
            {
                for(i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++ )
                {
                    wrkThd2SrvArbMapConfig[i] =
                                        wrkThd2SrvArbSKU4CPMWirelessMap[i];
                }
            }
            else
            {
                for(i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++ )
                {
                    wrkThd2SrvArbMapConfig[i] = wrkThd2SrvArbSKU4CPMMap[i];
                }
            }

        break;

        case DEV_SKU_2:  /* 6 CPMs, 700Mhz */
        case DEV_SKU_4:  /* 6 CPMs, 600Mhz */
            if(wireless_enabled)
            {
                for(i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++ )
                {
                    wrkThd2SrvArbMapConfig[i] =
                                    wrkThd2SrvArbSKU6CPMWirelessMap[i];
                }
            }
            else
            {
                for(i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++ )
                {
                    wrkThd2SrvArbMapConfig[i] = wrkThd2SrvArbSKU6CPMMap[i];
                }
            }
            break;

        case DEV_SKU_3:  /* 6 CPMs, 600Mhz, DC only */
                /* No CY capability on this device, setup the
                 * DC only arbiter thread mapping */
                for(i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++ )
                {
                    wrkThd2SrvArbMapConfig[i] =
                                        wrkThd2SrvArbSKUDCOnlyMap[i];
                }

            break;

        default:
            ADF_ERROR("Invalid SKU. NO arbiter configuration available.\n");
            return CPA_STATUS_FAIL;
    }

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;

    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    /*
     * It is possible for one thread to be mapped to more than one service
     * arbiter (ie: thread can be shared between cy & dc). The register is laid
     * out as Cpa32U wrkThd2SrvArbMapConfig[0:11].
     *
     * In the 50G SKU:
     * 12 AEs, 8 Threads per AE = 96 threads. One register maps to 1 AEs
     * 8 threads => 4 bits per thread config.
     *
     * In the 25G SKU:
     * There are 8 AEs, so the last four registers have no mapping. The first 8
     * registers are the same as in the 50G SKU
     *
     * The mappings come in 3 flavours (not including wireless & dc only)
     * across the 12 wrkThd2SrvArbMapConfig registers, as defined above.
     * These mappings are as follows:
     *
     * Thread     | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
     * Service    |  CY & DC  |       CY      |PKE|
     *
     * Thread     | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
     * Service    |       CY & TRNG       |  PKE  |
     *
     * Thread     | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
     * Service    |             CY        |  PKE  |
     */

    for (i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++)
    {
        WRITE_CSR_ARB_WRKTHD2SRVARBMAP(csr_base_addr, i,
                wrkThd2SrvArbMapConfig[i]);
    }

    return CPA_STATUS_SUCCESS;
}

STATIC CpaStatus adf_arbConfigWorkQueues(icp_accel_dev_t *pAccelDev)
{
    Cpa32U wq_cfg = 0;
    Cpa32U i = 0;
    Cpa32U *csr_base_addr = NULL;
    icp_etr_priv_data_t* privData = NULL;

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;

    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    for (i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++ )
    {
        adf_insert_bits(ICP_ARB_REG_SIZE, i, 0, &wq_cfg );
        WRITE_CSR_ARB_WQCFG(csr_base_addr, i, wq_cfg);
    }
    return CPA_STATUS_SUCCESS;
}

CpaStatus adf_arbShutdown(icp_accel_dev_t *pAccelDev)
{
    Cpa32U wq_cfg = 0;
    Cpa32U ringSrvArbEn = 0;
    Cpa32U respOrdering = 0xFFFFFFFF;
    Cpa32U wrkThd2SrvArbMapConfig = 0;
    Cpa32U arbNumber = 0;
    Cpa32U i = 0;
    Cpa32U j = 0;
    Cpa32U *csr_base_addr = NULL;
    icp_etr_priv_data_t* privData = NULL;
    Cpa8U thold = 0x8;
    Cpa8U fcen = 0x1;
    Cpa32U sarb_cfg = 0;
    Cpa32U sarb_wtr = 0xFFFFFFFF;
    Cpa32U maxBanks = 0;
    CpaStatus status;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    ICP_CHECK_FOR_NULL_PARAM(csr_base_addr);

    /* 1. Service arbiters shutdown */
    adf_insert_bits(ICP_ARB_REG_SIZE, thold, 4, &sarb_cfg );
    adf_insert_bits(ICP_ARB_REG_SIZE, fcen, 31, &sarb_cfg );

    for(arbNumber = ICP_SARB_ZERO; arbNumber <= ICP_SARB_THREE; arbNumber++)
    {
        WRITE_CSR_ARB_SARCONFIG(csr_base_addr, arbNumber, sarb_cfg);
    }

    /* 2. Service weighting shutdown */
    for( i = 0; i < ICP_ARB_NUM; i++ )
    {
        for(j = 0; j < ICP_ARB_REQ_RING_NUM; j++)
        {
            WRITE_CSR_ARB_WEIGHT(csr_base_addr, i, j, sarb_wtr);
        }
    }

    /* 3. Response ordering shutdown */
    for (i = 0; i < ICP_ARB_REQ_RING_NUM; i++ )
    {
        WRITE_CSR_ARB_RESPORDERING(csr_base_addr, i, respOrdering);
    }

    /* 4. Work queue shutdown */
    for (i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++ )
    {
        adf_insert_bits(ICP_ARB_REG_SIZE, i, 0, &wq_cfg );
        WRITE_CSR_ARB_WQCFG(csr_base_addr, i, wq_cfg);
    }

    /* 5. Unmap worker threads to service arbiters */
    for (i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++)
    {
        WRITE_CSR_ARB_WRKTHD2SRVARBMAP(csr_base_addr, i,
                wrkThd2SrvArbMapConfig);
    }

    maxBanks = GET_MAX_BANKS(pAccelDev);

    /* 6. Disable arbitration on all rings */
    for (i = 0; i < maxBanks; i++)
    {
        WRITE_CSR_ARB_RINGSRVARBEN(csr_base_addr, i, ringSrvArbEn);
    }

    /* 7. Remove the arbiter debug */
    status = adf_arbDebugRemove(pAccelDev);

    if(status != CPA_STATUS_SUCCESS)
    {
        ADF_ERROR("Could not remove /proc arbiter debug file \n");
        return CPA_STATUS_FAIL;
    }

    return status;
}

CpaStatus adf_arbDebugInit(icp_accel_dev_t *device)
{
    CpaStatus status = CPA_STATUS_FAIL;
    debug_file_info_t *file;

    file = ICP_MALLOC_GEN(sizeof(debug_file_info_t));

    if (NULL == file)
    {
        ADF_ERROR("Failed to create arbiter debug file.\n");
        return CPA_STATUS_FAIL;
    }

    osalMemSet(file, 0, sizeof(debug_file_info_t));
    file->name = ICP_MALLOC_GEN(DEBUG_ARBITER_NAME_LEN);

    if (NULL == file->name)
    {
        ADF_ERROR("Failed to create arbiter debug file.\n");
        return CPA_STATUS_FAIL;
    }
    strncpy(file->name, DEBUG_ARBITER_NAME, DEBUG_ARBITER_NAME_LEN);

    file->seq_read = adf_arbDebugRead;
    file->private_data = device;
    file->parent = device->base_dir;

    if (NULL != file->parent)
    {
        status = icp_adf_debugAddFile(device,
                  file);
    }
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to create arbiter debug file.\n");
        ICP_FREE(file->name);
        ICP_FREE(file);
        device->arbDebugHandle = NULL;
        return status;
    }
    device->arbDebugHandle = file;
    return status;
}

STATIC int adf_arbDebugRead(void* private_data, char* data, int size, int offset)
{
    Cpa32U len = 0, value, *csr_base_addr,entry;
    Cpa32S i = 0;
    Cpa32U j = 0;
    icp_accel_dev_t *pAccelDev = (icp_accel_dev_t *)private_data;
    icp_etr_priv_data_t *privData;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    ICP_CHECK_FOR_NULL_PARAM(privData);
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;

    len = snprintf(data+len, size-len,
                       "------- Arbiter Config -------\n");

    /* Service arbiters config */
    j = 1;
    for (entry=ICP_SARB_ZERO; entry <= ICP_SARB_THREE; entry++)
    {
        value = READ_CSR_ARB_SARBCONFIG(csr_base_addr, entry);

        len += snprintf(data+len, size-len, " SARB_CFG [%d]:\t\t",
                entry);

        for(i = NUMBER_BITS_PER_REG-1; i >= 0; i-- )
        {
            if( value & (1 << i))
            {
                len += snprintf(data+len, size-len, "1");
            }
            else
            {
                len += snprintf(data+len, size-len, "0");
            }
            if (j == 4)
            {
                len += snprintf(data+len, size-len, " ");
                j = 0;
            }
            j++;
        }
        len += snprintf(data+len, size-len, "\n");
    }

    /* Work queue config */
    j = 1;
    for (entry=0; entry < NUMBER_WQ_ENTRIES; entry++)
    {
        value = READ_CSR_ARB_WQCFG(csr_base_addr, entry);

        len += snprintf(data+len, size-len, " WQCFG [%d]:\t\t",
                entry);

        for(i = NUMBER_BITS_PER_REG-1; i >= 0; i-- )
        {
            if( value & (1 << i))
            {
                len += snprintf(data+len, size-len, "1");
            }
            else
            {
                len += snprintf(data+len, size-len, "0");
            }
            if (j == 4)
            {
                len += snprintf(data+len, size-len, " ");
                j = 0;
            }
            j++;
        }
        len += snprintf(data+len, size-len, "\n");

    }

    /* Worker threads to service arbiters map */
    for (entry=0; entry < ICP_ARB_WRK_THREAD_TO_SARB; entry++)
    {
        value = READ_CSR_ARB_WRKTHD2SRVARBMAP(csr_base_addr, entry);

        len += snprintf(data+len, size-len, " WrkThd2SrvArbMap [%d]:\t\t",
                entry);

        j = 1;
        for(i = NUMBER_BITS_PER_REG-1; i >= 0; i-- )
        {
            if( value & (1 << i))
            {
                len += snprintf(data+len, size-len, "1");
            }
            else
            {
                len += snprintf(data+len, size-len, "0");
            }
            if (j == 4)
            {
                len += snprintf(data+len, size-len, " ");
                j = 0;
            }
            j++;
        }
        len += snprintf(data+len, size-len, "\n");

    }

    len += snprintf(data+len, size-len,
                       "------- Arbiter Status -------\n");
    /* Worker queue status register(s) */
    for (entry=0; entry < NUMBER_WQ_ENTRIES; entry++)
    {
        value = READ_CSR_ARB_WQSTAT(csr_base_addr,entry);
        len += snprintf(data+len, size-len, " Worker Queue Entry [%d]:\t\t",
                entry);
        for(i = NUMBER_BITS_PER_WQ_ENTRIES; i >= 0; i-- )
        {
            if( value & (1 << i))
            {
                len += snprintf(data+len, size-len, " 1");
            }
            else
            {
                len += snprintf(data+len, size-len, " 0");
            }
        }
        len += snprintf(data+len, size-len, "\n");
    }

    len += snprintf(data+len, size-len,
            "-------------------------------------\n");

    return 0;
}


STATIC CpaStatus adf_arbDebugRemove(icp_accel_dev_t *icp_accel_dev)
{
    debug_file_info_t *file =
            (debug_file_info_t *)icp_accel_dev->arbDebugHandle;
    if (NULL != file)
    {
        icp_adf_debugRemoveFile(file);
        ICP_FREE(file->name);
        ICP_FREE(file);
    }
    return CPA_STATUS_SUCCESS;
}

/*
* @description
* Overwrite the default thread to service arbiter mappings.
* The size of the array must equal ICP_ARB_WRK_THREAD_TO_SARB
*/
CpaStatus adf_arbOverwriteWorkerCSRs(icp_accel_dev_t *pAccelDev,
        Cpa32U wrkThrdMapArr[])
{
    icp_etr_priv_data_t* privData = NULL;
    Cpa32U *csr_base_addr = NULL;
    Cpa32U i = 0;

    privData = (icp_etr_priv_data_t *) pAccelDev->pCommsHandle;
    csr_base_addr = (Cpa32U*)(UARCH_INT)privData->csrBaseAddress;


    for (i = 0; i < ICP_ARB_WRK_THREAD_TO_SARB; i++)
    {
        WRITE_CSR_ARB_WRKTHD2SRVARBMAP(csr_base_addr, i,
                wrkThrdMapArr[i]);
    }
    return CPA_STATUS_SUCCESS;
}

