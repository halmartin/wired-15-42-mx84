/***************************************************************************
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
 ***************************************************************************/

/**
 ***************************************************************************
 * @file lac_pke_utils.c
 *
 * @ingroup LacAsymCommonUtils
 *
 * Implementation of utils that are PKE specific
 *
 ******************************************************************************/

/*
********************************************************************************
* Include public/global header files
********************************************************************************
*/
#include "cpa.h"

/*
********************************************************************************
* Include private header files
********************************************************************************
*/

/* OSAL includes */
#include "Osal.h"

/* ADF includes */
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_debug.h"
#include "icp_adf_transport.h"
#include "icp_adf_poll.h"
#include "icp_accel_devices.h"

/* FW includes */
#include "icp_qat_fw_mmp_ids.h"
#include "icp_qat_fw_la.h"

/* SAL includes */
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_log.h"
#include "lac_mem_pools.h"
#include "lac_ec.h"
#include "lac_sym.h"
#include "lac_list.h"
#include "lac_sal_types_crypto.h"
#include "lac_pke_qat_comms.h"
#include "lac_pke_utils.h"


/*
********************************************************************************
* Static Variables
********************************************************************************
*/

/*
 * Values below used to wait on completion of mmp liveness request
 * by polling completion variable.  10-second timeout chosen, to avoid
 * infinite wait
 */
#define LAC_PKE_WAIT_COUNT      (1000)
/**< Number of times to sleep and wakeup */
#define LAC_PKE_DELAY_IN_MS   (10)
/**< Time to sleep in ms */

#define MMP_LIVENESS_DATA_SIZE 64
/**< Size of buffer sent in PKE liveness requests */

#define PKE_FW_ID_IDX 0
/**< index of Cpa32U array where fw id can be found */

/*
********************************************************************************
* Define static function definitions
********************************************************************************
*/

/*
********************************************************************************
* Global Variables
********************************************************************************
*/

/*
********************************************************************************
* Define public/global function definitions
********************************************************************************
*/

Cpa32S
LacPke_Compare(
    const CpaFlatBuffer* pFlatBufferA,
    const Cpa32S deltaA,
    const CpaFlatBuffer* pFlatBufferB,
    const Cpa32S deltaB)
{
    Cpa32S delta = 0;
    Cpa32S notZero = 0;
    Cpa32U lenA = 0;
    Cpa32U lenB = 0;

    LAC_ASSERT_NOT_NULL(pFlatBufferA);
    LAC_ASSERT_NOT_NULL(pFlatBufferB);
    LAC_ASSERT_NOT_NULL(pFlatBufferA->pData);
    LAC_ASSERT_NOT_NULL(pFlatBufferB->pData);
    LAC_ENSURE(((deltaA > -(1<<30)) && (deltaA < (1<<30))),
                                          "deltaA out of range");
    LAC_ENSURE(((deltaB > -(1<<30)) && (deltaB < (1<<30))),
                                          "deltaB out of range");

    delta = deltaA - deltaB;
    lenA = pFlatBufferA->dataLenInBytes;
    lenB = pFlatBufferB->dataLenInBytes;

    LAC_ENSURE((lenA + lenB > 0), "Invalid Buffer lengths");

    /* Shortcut for buffers of the same size */
   if((lenA == lenB) && (delta == 0))
   {
       return memcmp(pFlatBufferA->pData, pFlatBufferB->pData, lenA);
   }

   /* Run a full subtraction byte per byte starting
      from the least significant byte and taking
      care of buffers of different sizes */
    while(lenA | lenB)
    {
       if(lenA)
       {
           delta += pFlatBufferA->pData[--lenA];
       }
       if(lenB)
       {
           delta -= pFlatBufferB->pData[--lenB];
       }
       notZero |= (delta & 0xFF);
       /* signed arithmetic shift */
       delta >>= LAC_NUM_BITS_IN_BYTE;
   }
   /* Convert all neg numbers to -1 and all positive numbers to +1 */
   return notZero ? LAC_SIGNED_CONVERT32(delta): 0;

}

Cpa32S
LacPke_CompareZero (
    const CpaFlatBuffer* pLargeInteger,
    const Cpa32S delta)
{
    Cpa32S result = 0;
    Cpa32U i = 0;

    LAC_ASSERT_NOT_NULL(pLargeInteger);
    LAC_ENSURE(((delta >= -128) && (delta <= 127)), "delta out of range");

    i=pLargeInteger->dataLenInBytes-1;
    while(i--)
    {
        if(pLargeInteger->pData[i])
        {
           /* Data in Buffer+delta>0 */
           return 1;
        }
    }
    result = pLargeInteger->pData[(pLargeInteger->dataLenInBytes-1)] + delta;
    return result;
}

Cpa32U
LacPke_GetMinBytes(const CpaFlatBuffer *pBuffer)
{
   Cpa32U buffSizeInBytes = pBuffer->dataLenInBytes;
   Cpa32U i=0;

   /* starting at the MSB loop through all bytes until
      find the first non-zero byte if it exists */
   while((i < buffSizeInBytes) && (pBuffer->pData[i] ==0))
   {
       i++;
   }
   return (buffSizeInBytes - i);
}

CpaStatus
LacPke_GetBitPos(const CpaFlatBuffer *pBuffer , Cpa32U *pBitPos,
                            Cpa32U *pIndexMsb , CpaBoolean *pIsZero)
{
    Cpa32U numBytes = 0;
    Cpa32U buffSizeInBytes = pBuffer->dataLenInBytes;
    Cpa32U bitPosition = 0;
    Cpa8U msByte = 0;

    /* Set zero flag false initally */
    *pIsZero = CPA_FALSE;
    /* Find the first non-zero byte from end of buffer */
    numBytes = LacPke_GetMinBytes(pBuffer);
    /* set pIndexMsb */
    *pIndexMsb = buffSizeInBytes - numBytes;

    if(0 == numBytes)
    {
        /* Number is 0 */
        *pIsZero = CPA_TRUE;
        (*pIndexMsb)--;
        bitPosition = 0;
    }
    else
    {
        if(numBytes > LAC_BITS_TO_BYTES(LAC_4096_BITS))
        {
            LAC_INVALID_PARAM_LOG("Number size not supported");
            return CPA_STATUS_INVALID_PARAM;
        }
        /* calc bit position rounded up to nearest byte */
        bitPosition = (numBytes*LAC_NUM_BITS_IN_BYTE) - 1;

        msByte = pBuffer->pData[(*pIndexMsb)];

        /* Locate the msb in the MSB */
        while((msByte & 0x80) == 0)
        {
            msByte <<= 1;
            bitPosition--;
        }
    }
    *pBitPos = bitPosition;
    return CPA_STATUS_SUCCESS;
}

CpaStatus
LacPke_GetBitLen(const CpaFlatBuffer *pBuffer , Cpa32U *pBitLen)
{
    Cpa32U numBytes = 0;
    Cpa32U buffSizeInBytes = pBuffer->dataLenInBytes;
    Cpa32U bitLength = 0;
    Cpa8U msByte = 0;

    /* Find the first non-zero byte */
    numBytes = LacPke_GetMinBytes(pBuffer);
    if(0 == numBytes)
    {
        /* Number is 0 */
        bitLength = 0;
    }
    else
    {
        if(numBytes > LAC_BITS_TO_BYTES(LAC_4096_BITS))
        {
            LAC_INVALID_PARAM_LOG("Number size not supported");
            return CPA_STATUS_INVALID_PARAM;
        }
        /* calc bit position rounded up to nearest byte */
        bitLength = (numBytes*LAC_NUM_BITS_IN_BYTE);
        msByte = pBuffer->pData[(buffSizeInBytes - numBytes)];

        /* Locate the msb in the MSB */
        while((msByte & 0x80) == 0)
        {
            msByte <<= 1;
            bitLength--;
        }
    }
    *pBitLen = bitLength;
    return CPA_STATUS_SUCCESS;
}

