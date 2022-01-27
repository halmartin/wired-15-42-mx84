/**
 **************************************************************************
 * @file halMemScrub.c
 *
 * @description
 *      This file provides Implementation of shared RAM and MMP memory initilization 
 *
 * @par 
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
 **************************************************************************/ 

#include "halMemScrub.h"
#include "halAeApi.h"
#include "halAeDrv.h"

extern int execMicroInst(icp_firml_handle_t *handle,
                         unsigned char ae, 
                         unsigned char ctx, 
                         uword_T *microInst,
                         unsigned int numInst, 
                         int condCodeOff,
                         unsigned int maxCycles, 
                         unsigned int *endPC);
extern int
halAe_ContinuousDramAlloc(icp_firml_handle_t *Handle,
                          icp_firml_dram_desc_t *pDram_desc,
                          unsigned int size);
extern void 
halAe_ContinuousDramFree(icp_firml_handle_t *Handle,
                         icp_firml_dram_desc_t *pDram_desc);


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     returns the name of the passes CSR.
 *
 * @param CSR_offset - IN
 * @param 
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
#if (defined(_DBG_PRINT))

static const char *
halMem_getCsrName(unsigned int CSR_offset)
{
    switch(CSR_offset)
    {
    case SSU_DBPCSRSSUMMP:  return ("DBPCSRSSUMMP");
    case SSU_DBPPCSSUMMP:   return ("DBPPCSSUMMP");
    case SSU_DIOVRDSSUMMP:  return ("DIOVRDSSUMMP");
    case SSU_DMWISSUMMP:    return ("DMWISSUMMP");
    case SSU_DOAWDSSUMMP:   return ("DOAWDSSUMMP");
    case SSU_DOBWDSSUMMP:   return ("DOBWDSSUMMP");
    case SSU_DPMWDSSUMMP:   return ("DPMWDSSUMMP");
    case SSU_DMRISSUMMP:    return ("DMRISSUMMP");
    case SSU_DOARDSSUMMP:   return ("DOARDSSUMMP");
    case SSU_DOBRDSSUMMP:   return ("DOBRDSSUMMP");
    case SSU_DPMRDSSUMMP:   return ("DPMRDSSUMMP");
    case SSU_DSMRDSSUMMP:   return ("DSMRDSSUMMP");
    case SSU_DISRDSSUMMP:   return ("DISRDSSUMMP");
    default:                break;
    }
    return ("UNKNOWN");
}

#endif

/**
*****************************************************************************
* @ingroup icp_hal
* 
* @description
*      Convert the halAe return codes to the halMem codes
*
*
*
* @param halRet - IN
*
* @retval
* 
* 
*****************************************************************************/
int halMem_CnvHalAeRetCode(int halRet)
{
    switch(halRet)
    {
        case HALAE_SUCCESS: 
            return (HALMEM_SUCCESS);
        case HALAE_FAIL: 
            return (HALMEM_DEVICE_ERROR);
        case HALAE_TIMEOUT:
            return (HALMEM_TIMEOUT);
        case HALAE_AEACTIVE: 
            return (HALMEM_AEACTIVE);
        default: 
            return (HALMEM_DEVICE_ERROR);
    }
}


/**
*****************************************************************************
* @ingroup icp_hal
* 
* @description
*    Enable the PKE or SHRam if it is power-off.
*    If fail to enable PKE or SHRam, it returns POWER_OFF as end state for unavailable PKE or SHRam
*    
* @assumptions
*    None
* @sideEffects
*    None
* @blocking
*    No
* @reentrant
*    No
* @threadSafe
*    Yes
* 
* @param handle    - IN A pointer to internal data structure which contains useful info, 
*                               e.g. blocks' base address
* @param Qat        - IN  Specifies the Qat of interest
* @param QatType - IN  QAT type, i.e. QAT_PKE_TYPE and QAT_SHRAM_TYPE
*
*    
* @retval ON_TO_ON     PKE or SHRam started out in power-on state, and return with power-on state
* @retval OFF_TO_ON    PKE or SHRam started out in power-off state, and return with power-on state
* @retval ON_TO_OFF    PKE or SHRam started out in power-on state, and return with power-off state
* @retval OFF_TO_OFF       PKE or SHRam started out in power-off state, and return with power-off state
* 
*****************************************************************************/
unsigned int
halMem_EnableQat (
     icp_firml_handle_t *Handle,
     unsigned int       Qat,
     qat_type_t         QatType)
 {
     unsigned int SlicePwrDown, BitPos = SHRAM_PWRDOWN_BIT, i;
     unsigned int QatState = OFF_TO_OFF;
     if (Handle == NULL)
     {
         ERRINFO(("Invalid parameter for funciton halMem_EnableQat\n"));
         return OFF_TO_OFF;
     }  
 
     if(QatType == QAT_SHRAM_TYPE) 
     {
         BitPos = SHRAM_PWRDOWN_BIT;
     }

     switch(Handle->sysMemInfo.deviceId)
     {
     case ICP_ACCELCOMP_PCIE_DEVICE_ID_AC:
     case ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC:
         if(QatType == QAT_PKE_TYPE) 
         {
             BitPos = PKE_PWRDOWN_BIT;
         }
         if (Handle->sysMemInfo.qatMask & (1<<Qat))
         {
             SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN);
             if (!TEST((~SlicePwrDown), BitPos))
             {
                 QatState = (POWER_OFF << START_BITPOS) & START_MASK;
                 //
                 // Enable PKE or SHRAM
                 //
                 SlicePwrDown &= ~(1<<BitPos);
                 PUT_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN, SlicePwrDown);
             }
             else
             {
                 QatState = (POWER_ON << START_BITPOS) & START_MASK;
             }
             SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN);
             if (TEST((~SlicePwrDown), BitPos))
             {
                 QatState |= ((POWER_ON << END_BITPOS) & END_MASK);
             }
             else
             {
                 QatState |= ((POWER_OFF << END_BITPOS) & END_MASK);
             }
             return QatState;
         }
         break;
     case ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC:
     case ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC:
         if(QatType == QAT_PKE_TYPE) 
         {
             for (i=0; i<ACCELCOMP_C_NUM_MMPS; i++)
             {
                 BitPos = ACCELCOMP_C_PKE_PWRDOWN_BIT(i);
                 if (Handle->sysMemInfo.qatMask & (1<<Qat))
                 {
                     SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, 
                                                    SSU_SLICEPWRDOWN);
                     if (!TEST((~SlicePwrDown), BitPos))
                     {
                         QatState = (POWER_OFF << START_BITPOS) & START_MASK;
                         //
                         // Enable PKE or SHRAM
                         //
                         SlicePwrDown &= ~(1<<BitPos);
                         PUT_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN, 
                                         SlicePwrDown);
                     }
                     else
                     {
                         QatState = (POWER_ON << START_BITPOS) & START_MASK;
                     }
                     SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, 
                                                    SSU_SLICEPWRDOWN);
                     if (TEST((~SlicePwrDown), BitPos))
                     {
                          QatState |= ((POWER_ON << END_BITPOS) & END_MASK);
                     }
                     else
                     {
                         QatState |= ((POWER_OFF << END_BITPOS) & END_MASK);
                     }                
                 }
             }
             return QatState;            
         }
         else
         {
             if (Handle->sysMemInfo.qatMask & (1<<Qat))
             {
                 SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN);
                 if (!TEST((~SlicePwrDown), BitPos))
                 {
                     QatState = (POWER_OFF << START_BITPOS) & START_MASK;
                     //
                     // Enable PKE or SHRAM
                     //
                     SlicePwrDown &= ~(1<<BitPos);
                     PUT_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN, 
                                     SlicePwrDown);
                 }
                 else
                 {
                     QatState = (POWER_ON << START_BITPOS) & START_MASK;
                 }
                 SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN);
                 if (TEST((~SlicePwrDown), BitPos))
                 {
                     QatState |= ((POWER_ON << END_BITPOS) & END_MASK);
                 }
                 else
                 {
                     QatState |= ((POWER_OFF << END_BITPOS) & END_MASK);
                 }
                 return QatState;
             }
         }
                 
         break;
     default:
         return QatState;
     }
     return QatState;
 }

 /**
*****************************************************************************
* @ingroup icp_hal
* 
* @description
*    Disable the specified PKE or SHRam if it started out in power-off state.
*    
* @assumptions
*    None
* @sideEffects
*    None
* @blocking
*    No
* @reentrant
*    No
* @threadSafe
*    Yes
* 
* @param handle    - IN A pointer to internal data structure which contains useful info, 
*                               e.g. blocks' base address
* @param Qat        - IN  Specifies the Qat of interest
* @param QatType - IN  QAT type, i.e. QAT_PKE_TYPE and QAT_SHRAM_TYPE
*
*
* @retval HALMEM_SUCCESS            The function completed successfully
* @retval HALMEM_DEVICE_ERROR       Operation failed
* 
*****************************************************************************/
 int
 halMem_DisableQat (
    icp_firml_handle_t  *Handle,
    unsigned int        Qat,
    qat_type_t          QatType)
 {
     unsigned int SlicePwrDown, BitPos = SHRAM_PWRDOWN_BIT, i;
     int Status = HALMEM_SUCCESS;
     if (Handle == NULL)
     {
         ERRINFO(("Invalid parameter for funciton halMem_DisableQat\n"));
         return HALMEM_INVALID_PARAMETER;
     }   
 
     if(QatType == QAT_SHRAM_TYPE) 
     {
         BitPos = SHRAM_PWRDOWN_BIT;
     }

     switch(Handle->sysMemInfo.deviceId)
     {
     case ICP_ACCELCOMP_PCIE_DEVICE_ID_AC:
     case ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC:
         if(QatType == QAT_PKE_TYPE) 
         {
             BitPos = PKE_PWRDOWN_BIT;
         }
         if(Handle->sysMemInfo.qatMask & (1<<Qat))
         {
             SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN);
             if (TEST((~SlicePwrDown), BitPos))
             {
                 //
                 // Disable PKE or SHRAM
                 //
                 SlicePwrDown |= 1<<BitPos;
                 PUT_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN, SlicePwrDown);
             }
             SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN);
             if (TEST((~SlicePwrDown), BitPos))
             {
                 Status = HALMEM_DEVICE_ERROR;
             }
         }
         break;
     case ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC:
     case ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC:
         if(QatType == QAT_PKE_TYPE) 
         {
             for (i=0; i<ACCELCOMP_C_NUM_MMPS; i++)
             {
                 BitPos = ACCELCOMP_C_PKE_PWRDOWN_BIT(i);
                 if (Handle->sysMemInfo.qatMask & (1<<Qat))
                 {
                     SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, 
                                                    SSU_SLICEPWRDOWN);
                     if (TEST((~SlicePwrDown), BitPos))
                     {
                         //
                         // Disable PKE or SHRAM
                         //
                         SlicePwrDown |= 1<<BitPos;
                         PUT_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN,
                                         SlicePwrDown);
                     }
                     SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, 
                                                    SSU_SLICEPWRDOWN);
                     if (TEST((~SlicePwrDown), BitPos))
                     {
                         Status = HALMEM_DEVICE_ERROR;
                     }
                 }
             }
         }
         else
         {
             if (Handle->sysMemInfo.qatMask & (1<<Qat))
             {
                 SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN);
                 if (TEST((~SlicePwrDown), BitPos))
                 {
                     //
                     // Disable PKE or SHRAM
                     //
                     SlicePwrDown |= 1<<BitPos;
                     PUT_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN, 
                                     SlicePwrDown);
                 }
                 SlicePwrDown = GET_SSU_QAT_CSR(Handle, Qat, SSU_SLICEPWRDOWN);
                 if (TEST((~SlicePwrDown), BitPos))
                 {
                     Status = HALMEM_DEVICE_ERROR;
                 }
             }
         }
         break;
     default:
         return HALMEM_DEVICE_ERROR;
     }
     return Status;
 }


 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *     Read the specified CSR and return the value.
  *
  * @param QatNum - IN
  * @param MmpNum - IN
  * @param CSR_offset - IN
  *
  * @retval  
  * 
  * 
  *****************************************************************************/
 unsigned int 
 halMem_MmpReadCsr(
     icp_firml_handle_t *handle,
     unsigned int QatNum,
     unsigned int MmpNum,
     unsigned int CSR_offset)
 {
     unsigned int value;
     value = GET_SSU_QAT_MMP_CSR(handle, QatNum, MmpNum, CSR_offset);
     return value;
 }
 
 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *     Write the specified CSR with the value.
 
  *
  * @param QatNum - IN
  * @param MmpNum - IN
  * @param CSR_offset - IN
  * @param value - IN
  *
  * @retval  
  * 
  * 
  *****************************************************************************/
 void 
 halMem_MmpWriteCsr(
     icp_firml_handle_t *handle,
     unsigned int QatNum,
     unsigned int MmpNum,
     unsigned int CSR_offset,
     unsigned int value)
 {
     PUT_SSU_QAT_MMP_CSR(handle, QatNum, MmpNum, CSR_offset, value);
     return;
 }
 
 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *     This function reads MMP memory and internal state. The passed 
  *  CSR offset defines what is read (program memory, operand memory, scope
  *  memory, or internal state). The Buffer receives the read memory values.
  *
  * 
  * @param QatNum - IN
  * @param MmpNum - IN
  * @param CSR_offset - IN
  * @param Start - IN
  * @param Longwords - IN
  * @param Buffer - OUT
  *
  * @retval  
  * 
  * 
  *****************************************************************************/
 unsigned int 
 halMem_MmpReadMemory(
     const char* FunctionName,
     icp_firml_handle_t *handle,
     unsigned int QatNum, 
     unsigned int MmpNum, 
     unsigned int CSR_offset, 
     unsigned int Start,
     unsigned int Longwords,
     unsigned int* Buffer)
 {
     unsigned int index;
     unsigned int value;
 
     if (CSR_offset != SSU_DPMRDSSUMMP &&
         CSR_offset != SSU_DOARDSSUMMP &&
         CSR_offset != SSU_DOBRDSSUMMP &&
         CSR_offset != SSU_DSMRDSSUMMP &&
         CSR_offset != SSU_DISRDSSUMMP)
     {
         ERRINFO(("%s: Unexpected memory read CSR 0x%08X\n", 
                  FunctionName, CSR_offset));
         return HALMEM_DEVICE_ERROR;
     }
 
     if (Longwords == 0)
     {
         /* nothing to do */
         return HALMEM_SUCCESS;
     }
 
     /* Read the block of memory or internal status.  First setup the index */
     /* CSR. Then read each longword until the specified number have */
     /* been read. */
     halMem_MmpWriteCsr(handle, 
                        QatNum, 
                        MmpNum, 
                        SSU_DMRISSUMMP, 
                        (Start & 0x1FF)); /* index CSR has nine useful bits */
 
     for (index = 0; index < Longwords; index++)
     {
         /* Read the memory CSR. The index register auto-increments. */
         Buffer[index] = halMem_MmpReadCsr(handle, QatNum, MmpNum, CSR_offset);
     }
 
     /* Read the index CSR in order to help confirm success. */
     /* Did it auto-increment the correct number of times? */
     value = halMem_MmpReadCsr(handle, QatNum, MmpNum, SSU_DMRISSUMMP);
     /* index CSR has nine useful bits */
     if ((value & 0x1FF) != ((Start + Longwords) & 0x1FF))  
     {
         ERRINFO(("%s: Unexpected value (%u) in CSR %s-QAT%u-MMP%u after "\
                  "reading CSR %s-QAT%u-MMP%u\n", FunctionName, value, 
                  halMem_getCsrName(SSU_DMRISSUMMP), QatNum, MmpNum, 
                  halMem_getCsrName(CSR_offset), QatNum, MmpNum));
         return HALMEM_DEVICE_ERROR;
     }
 
     /* success logged by caller */
     return HALMEM_SUCCESS;
 }
 
 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *       This function writes MMP memory and internal state. The passed 
  *  CSR offset defines what is written (program memory, operand memory). 
  *  The Buffer contains the memory longwords to be written.  If the bInitFlag
  *  is non-zero, the first Buffer element is written to all memory elements, 
  *  effectively initializing them all to the same value. Otherwise but Buffer
  *  is assumed to contain an array of values to be written.
  *
  *
  * @param QatNum - IN
  * @param MmpNum - IN
  * @param CSR_offset - IN
  * @param Start - IN
  * @param Longwords - IN
  * @param Buffer - OUT
  * @param bInitFlag - IN
  *
  * @retval  
  * 
  * 
  *****************************************************************************/
 unsigned int 
 halMem_MmpWriteMemory(
     const char* FunctionName,
     icp_firml_handle_t *handle,
     unsigned int QatNum, 
     unsigned int MmpNum, 
     unsigned int CSR_offset, 
     unsigned int Start,
     unsigned int Longwords,
     unsigned int* Buffer,
     unsigned int bInitFlag)
 {
     unsigned int RD_CSR_offset;
     unsigned int End;
     unsigned int ActualStart;
     unsigned int ActualEnd;
     unsigned int TmpStartBuffer = 0;
     unsigned int TmpEndBuffer = 0;
     unsigned int index;
     unsigned int value;
 
     if (CSR_offset == SSU_DPMWDSSUMMP)
     {
         RD_CSR_offset = SSU_DPMRDSSUMMP;
     }
     else if (CSR_offset == SSU_DOAWDSSUMMP)
     {
         RD_CSR_offset = SSU_DOARDSSUMMP;
     }
     else if (CSR_offset == SSU_DOBWDSSUMMP)
     {
         RD_CSR_offset = SSU_DOBRDSSUMMP;
     }
     else
     {
         ERRINFO(("%s: Unexpected memory write CSR 0x%08X\n", 
                 FunctionName, CSR_offset));
         return HALMEM_DEVICE_ERROR;
     }
 
     if (Longwords == 0)
     {
         /* nothing to do */
         return HALMEM_SUCCESS;
     }
 
     /* Because the memories are quadwords and because there is buffering, */
     /* we always need to start the write on an even index and end on an */
     /* odd index.  This means that we may need to read the current value */
     /* of these locations and rewrite them. */
     End = Start + Longwords - 1;
     if ((End & 1) == 0)
     {
         /* End index is even, so we need to read and then later write back */
         /* the longword after the original end */
         ActualEnd = End + 1;
         if (halMem_MmpReadMemory(FunctionName, handle, QatNum, MmpNum, 
                                  RD_CSR_offset, ActualEnd, 1, 
                                  &TmpEndBuffer) != HALMEM_SUCCESS)
         {
             /* error logged in function */
             return HALMEM_DEVICE_ERROR;
         }
     }
     else
     {
         ActualEnd = End;
     }
     if ((Start & 0x1) != 0)
     {
         /* Start index is odd, so we need to read and then later write */
         /* back the longword prior to the original start */
         ActualStart = Start - 1;
         if (halMem_MmpReadMemory(FunctionName, handle, QatNum, MmpNum, 
                                  RD_CSR_offset, ActualStart, 1, 
                                  &TmpStartBuffer) != HALMEM_SUCCESS)
         {
             /* error logged in function */
             return HALMEM_DEVICE_ERROR;
         }
     }
     else
     {
         ActualStart = Start;
     }
 
     /* Write the block of memory.  First setup the index CSR to the */
     /* ActualStart index. Then write each longword until the specified  */
     /* number have been written.  */
     /* Index CSR has nine useful bits */
     halMem_MmpWriteCsr(handle, 
                        QatNum, 
                        MmpNum, 
                        SSU_DMWISSUMMP, 
                        (ActualStart & 0x1FF));
 
     /* Write the pre-start longword back again if necessary */
     if (ActualStart != Start)
     {
         halMem_MmpWriteCsr(handle, QatNum, MmpNum, CSR_offset, TmpStartBuffer);
     }
 
     /* Write each of the designated memory locations */
     for (index = 0; index < Longwords; index++)
     {
         halMem_MmpWriteCsr(handle, QatNum, MmpNum, CSR_offset, 
                            (bInitFlag ? Buffer[0] : Buffer[index]));
     }
 
     /* Write the post-end longword back again if necessary */
     if (ActualEnd != End)
     {
         halMem_MmpWriteCsr(handle, 
                            QatNum, 
                            MmpNum,
                            CSR_offset,
                            TmpEndBuffer);
     }
 
     /* Read the index CSR in order to help confirm success. */
     /* Did it auto-increment the correct number of times? */
     value = halMem_MmpReadCsr(handle, QatNum, MmpNum, SSU_DMWISSUMMP);
     /* index CSR has nine useful bits */
     if ((value & 0x1FF) != ((ActualEnd + 1) & 0x1FF)) 
     {
         ERRINFO(("%s: Unexpected value (%u) in CSR %s-QAT%u-MMP%u after "\
                  "writing CSR %s-QAT%u-MMP%u\n", FunctionName, value, 
                  halMem_getCsrName(SSU_DMWISSUMMP), QatNum, MmpNum, 
                  halMem_getCsrName(CSR_offset), QatNum, MmpNum));
         return HALMEM_DEVICE_ERROR;
     }
 
     /* success logged by caller */
     return HALMEM_SUCCESS;
 }

/**
 *****************************************************************************
 * @ingroup  icp_hal
 * 
 * @description
 *       This function initializes the specified MMP's writable memories by 
*        manipulating various SSU debug csrs.  
 *
 *
 * @param QatNum - IN
 * @param MmpNum - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
int 
halMem_MmpInitializeMemory(const char* FunctionName,
                                 icp_firml_handle_t *Handle,
                                 unsigned int QatNum,
                                 unsigned int MmpNum)
{
    unsigned int status;
    unsigned int Value;

    status = HALMEM_SUCCESS;  /* assume success */

    /* Initialize the Debug Breakpoint Program Counter (DBPPCSSUMMP). */
    halMem_MmpWriteCsr(Handle, QatNum, MmpNum, SSU_DBPPCSSUMMP, 0x00000000);

    /* Initialize the Program Memory by interatively writing DPMWDSSUMMP. 
     The value should represent two "setup(mode, nop)" instructions. */
    Value = TWO_SETUP_MODE_NOP;
    if (halMem_MmpWriteMemory(FunctionName, Handle, QatNum, MmpNum, 
                              SSU_DPMWDSSUMMP, 0, MMP_PROGRAM_MEMORY_LONGWORDS,
                              &Value, 1) != HALMEM_SUCCESS)
    {
        /* error logged in function */
        ERRINFO(("%s: Fail to write QAT%d MMP%d program memory\n", 
                FunctionName, QatNum, MmpNum));
        status = HALMEM_DEVICE_ERROR;
    }

    /* Initialize the Operand A Memory by interatively writing DOAWDSSUMMP. */
    Value = 0;
    if (halMem_MmpWriteMemory(FunctionName, Handle, QatNum, MmpNum, 
                              SSU_DOAWDSSUMMP, 0, MMP_OPERAND_MEMORY_LONGWORDS,
                              &Value, 1) != HALMEM_SUCCESS)
    {
        /* error logged in function */
        ERRINFO(("%s: Fail to write QAT%d MMP%d OpA memory\n", 
                FunctionName, QatNum, MmpNum));
        status = HALMEM_DEVICE_ERROR;
    }

    /* Initialize the Operand B Memory by interatively writing DOBWDSSUMMP. */
    Value = 0;
    if (halMem_MmpWriteMemory(FunctionName, Handle, QatNum, MmpNum, 
                              SSU_DOBWDSSUMMP, 0, MMP_OPERAND_MEMORY_LONGWORDS,
                              &Value, 1) != HALMEM_SUCCESS)
    {
        /* error logged in function */
        ERRINFO(("%s: Fail to write QAT%d MMP%d OpB memory\n", 
                FunctionName, QatNum, MmpNum));
        status = HALMEM_DEVICE_ERROR;
    }

    DBGINFO(((status == HALMEM_SUCCESS ? "%s: Success\n" : "%s: Failed\n"), 
            FunctionName));
    return status;
}


 /**
   ****************************************************************************
   * @ingroup icp_hal
   * 
   * @description
   *      Get mmp number according to current HW platform
   *      
   * @assumptions
   *      None
   * @sideEffects
   *      None
   * @blocking
   *      No
   * @reentrant
   *      No
   * @threadSafe
   *      Yes
   * 
   * @param handle - IN A pointer to internal data structure which contains
   *                           useful info, e.g. blocks' base address
   *
   * @retval
   *    
   * 
   ***************************************************************************/
 unsigned int
 halMem_getMmpNum(icp_firml_handle_t *handle)
 {
     unsigned int mmpNum = 0, fuseCtl = 0;

     GET_PCI_CSR(handle, AC_FUSECTL, sizeof(unsigned int), &fuseCtl);
     switch (handle->sysMemInfo.deviceId)
     {
     case ICP_ACCELCOMP_PCIE_DEVICE_ID_AC:
        mmpNum = ACCELCOMP_NUM_MMPS;
        break;
     case ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC:
        if ((fuseCtl & 0x1) || (fuseCtl & (0x1 << ACCELCOMP_R_PKE_DIS_BIT)))
	{
	    mmpNum = 0;
	}
	else if ((fuseCtl & (0x3 << ACCELCOMP_R_SKU_BIT)))
	{
	    mmpNum = 1;
	}
	else
	{
	    mmpNum = ACCELCOMP_R_NUM_MMPS;
	}
        break;
     case ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC:
        /* If ACCELCOMP_C SKU3, all MMPs are fused out */
        if (((fuseCtl & (0x3 << ACCELCOMP_C_SKU_BIT)) 
            >> ACCELCOMP_C_SKU_BIT) == 0x2)
        {
            mmpNum = 0;       
        }
        else
        {
            mmpNum = ACCELCOMP_C_NUM_MMPS;
        }
        break; 
     case ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC:
        mmpNum = ACCELCOMP_B_NUM_MMPS;
        break;
     default:
        mmpNum = 0;
        break;    
     }

     return mmpNum;
 }


 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *      Scrub All QATs' MMP OpA/OpB/Program Memory
  *      
  * @assumptions
  *      None
  * @sideEffects
  *      None
  * @blocking
  *      No
  * @reentrant
  *      No
  * @threadSafe
  *      Yes
  * 
  * @param handle - IN A pointer to internal data structure which contains 
  *                           useful info, e.g. blocks' base address
  *
  * @retval HALMEM_SUCCESS The function completed successfully
  * @retval HALMEM_INVALID_PARAMETER  Invalid parameters
  * @retval HALMEM_DEVICE_ERROR Operation failed
  *    
  * 
  *****************************************************************************/
int 
halMem_ScrubMMPMemory(icp_firml_handle_t *handle)
{
    int Status = HALMEM_SUCCESS;
    unsigned int Qat, Mmp, maxNumQats = 0, mmpNum = 0;
    unsigned int MmpState = OFF_TO_OFF;
    static const char* FunctionName = "halMem_ScrubMMPMemory";

    if (handle == NULL)
    {
        ERRINFO(("Invalid parameter\n"));   
        return HALMEM_INVALID_PARAMETER;
    }

    MAX_VAL(maxNumQats, 
            ACCELCOMP_NUM_QATS, 
            ACCELCOMP_C_NUM_QATS, 
            ACCELCOMP_R_NUM_QATS, 
            ACCELCOMP_B_NUM_QATS, 
            ACCELCOMP_RS_NUM_QATS); 
    
    mmpNum = halMem_getMmpNum(handle);
    DBGINFO((" mmpNum = %u\n", mmpNum));

    for (Qat = 0; Qat < maxNumQats; Qat++)
    {
        if (!(handle->sysMemInfo.qatMask & (1 << Qat)))
        {
            continue;
        }

        MmpState = halMem_EnableQat(handle, Qat, QAT_PKE_TYPE);
        // power on PKE, and check PKE start and end state
        if (POWER_OFF == ((MmpState & END_MASK) >> END_BITPOS)) 
        {
            continue;       
        }
                
        for (Mmp = 0; Mmp < mmpNum; Mmp++)
        {
            DBGINFO(("Initialize memory for Qat%u Mmp%u\n", Qat, Mmp));
            Status = halMem_MmpInitializeMemory(FunctionName, handle,
                                                Qat, Mmp);
            if (HALMEM_ERROR (Status))
            {
               break;
            }               
        }
        if (POWER_OFF == ((MmpState & START_MASK) >> START_BITPOS))
        {
            // power off PKE if PKE started out in power-off state
            halMem_DisableQat(handle, Qat, QAT_PKE_TYPE);
        }
        if (HALMEM_ERROR (Status))
        {
            return Status;
        }
    }

    return Status;

} 
 
 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *      This function retrieve data from SHRAM region
  *
  *
  *
  * @param dbgHandle - IN AC firmware loader handle
  * @param QatNum - IN Specifies the QAT instance
  * @param ae - IN Specifies the ae for executing microcode snippet
  * @param ShramAddr - IN Specifies the SHRAM address
  * @param Size - IN Specifies the size of data retrieved in Longwords
  * @param Buffer - OUT A pointer to the location of the data to read. It should be 
  *                     large enough to accommodate the number of data requested.
  * @param EndianSwap - IN Specifies if the data need be swapped when retrieving.
  *
  * @retval HALMEM_SUCCESS The function completed successfully
  * @retval HALMEM_INVALID_PARAMETER  Invalid parameters
  * @retval HALMEM_DEVICE_ERROR Operation failed
  * @retval HALMEM_TIMEOUT The operation exceeded the time limit
  * 
  * 
  *****************************************************************************/
 int halMem_GetSharedRam(
     icp_firml_handle_t *Handle, 
     unsigned int QatNum, 
     unsigned int ae,
     unsigned int ShramAddr,
     unsigned int Size, /* in LongWords, at most MAX_SHRAM_USED is allowed */
     unsigned int *Buffer,
     unsigned int EndianSwap)
 {
     unsigned int gprA0, gprA1, gprB0, gprB1, sw0, sw1;
     unsigned int * dram=NULL;
     int status = HALAE_SUCCESS;
     uint64 dramStart;
     unsigned int addrHi, addrLow, desc0, desc1;
     unsigned int i;
 
     uword_T microInst[] = 
     {
         /*
          * Resources Used:
          * GPR:     A0, A1, B0, B1
          * XFER:    SRAM W0, W1
          * DRAM1:   offset 0 .. 1023
          *
          */
         /*
                 .reg $xfer0, $xfer1
                 .xfer_order $xfer0 $xfer1
                 .addr $xfer0 0
                 .addr $xfer1 1
 
                 .reg addr_hi
                 .reg addr_low
                 .reg desc0
                 .reg desc1
                 .areg addr_hi 0
                 .breg addr_low 0
                 .areg desc0 1
                 .breg desc1 1
                 .sig sig_dt
                 .addr sig_dt 2
 
                 br=ctx[0, start#]
                 br[kill_context#]
 
             start#:
                 immed_w0[addr_hi, 0]
                 immed_w1[addr_hi, 0]
                 immed_w0[addr_low, 0]
                 immed_w1[addr_low, 0]
 
                 immed_w0[desc1, 0]
                 immed_w1[desc1, 0]
                 alu[$xfer1, --, b, desc1]
 
                 immed_w0[desc0, 0]
                 immed_w1[desc0, 0]
                 alu[$xfer0, --, b, desc0]
 
                 ram[data_transfer_write, $xfer0, addr_hi, addr_low, 1], sig_done[sig_dt], no_endian_swap, 64_bit_addr, dual_sig
             wait_for_sig_dt0#:
                 br_!signal[sig_dt[0], wait_for_sig_dt0#]
             wait_for_sig_dt1#:
                 br_!signal[sig_dt[1], wait_for_sig_dt1#]
                 nop
                 nop
                 nop
                 nop
 
             kill_context#:
                 ctx_arb[kill], any
         */
         0x0D800800010ull, /* .0 br=ctx[0, start#] */
         0x0D804C00018ull, /* .1 br[kill_context#] */
         0x0F0400C0000ull, /* .2 immed_w0[l0000!addr_hi, 0] */
         0x0F4400C0000ull, /* .3 immed_w1[l0000!addr_hi, 0] */
         0x0F040000300ull, /* .4 immed_w0[l0000!addr_low, 0] */
         0x0F440000300ull, /* .5 immed_w1[l0000!addr_low, 0] */
         0x0F040000700ull, /* .6 immed_w0[l0000!desc1, 0] */
         0x0F440000700ull, /* .7 immed_w1[l0000!desc1, 0] */
         0x0A018100400ull, /* .8 alu[$l0000!xfer1, --, b, l0000!desc1] */
         0x0F0400C0001ull, /* .9 immed_w0[l0000!desc0, 0] */
         0x0F4400C0001ull, /* .10 immed_w1[l0000!desc0, 0] */
         0x0A0580C0001ull, /* .11 alu[$l0000!xfer0, --, b, l0000!desc0] */
         0x80320000300ull, /* .12 ram[data_transfer_write, $l0000!xfer0,
                                                  l0000!addr_hi, 
                                                  l0000!addr_low, 1], 
                                                  sig_done[l0000!sig_dt], 
                                                  no_endian_swap, 64_bit_addr,
                                                  dual_sig */
         0x0D803408213ull, /* .13 wait_for_sig_dt0#: 
                                                  br_!signal[l0000!sig_dt[0],
                                                  wait_for_sig_dt0#] */
         0x0D80380C213ull, /* .14 wait_for_sig_dt1#: 
                                                   br_!signal[l0000!sig_dt[1],
                                                   wait_for_sig_dt1#] */
         0x0F0000C0300ull, /* .15 nop */
         0x0F0000C0300ull, /* .16 nop */
         0x0F0000C0300ull, /* .17 nop */
         0x0F0000C0300ull, /* .18 nop */
         0x0E000010000ull, /* .19 kill_context#: ctx_arb[kill], any */
     };
 
     const int num_inst = sizeof(microInst)/sizeof(microInst[0]);
     const int condCodeOff = 1;
     const unsigned char ctx = 0;
     icp_firml_dram_desc_t dram_desc;

     status = halAe_ContinuousDramAlloc(Handle, &dram_desc,
                                        RAM_CHUNK_INIT_SIZE);
     if (status != HALAE_SUCCESS)
     {
         ERRINFO(("halMem_GetSharedRam: function halAe_ContinuousDramAlloc "\
                  "failed (code = 0x%.8x)\n", status));
         return HALMEM_DEVICE_ERROR;
     }

     dram = (unsigned int*)osalMemAlloc(MAX_SSU_SHARED_RAM);
     if (dram == NULL)
     {    
         halAe_ContinuousDramFree (Handle, &dram_desc); 
         ERRINFO(("Fail to allocate memory\n"));    
         return HALMEM_DEVICE_ERROR;
     }
     osalMemSet(dram, 0, MAX_SSU_SHARED_RAM);
     
     if (Size == 0) 
     {
         halAe_ContinuousDramFree (Handle, &dram_desc); 
         osalMemFree (dram);
         return HALMEM_SUCCESS;
     }
 
     /* fixup instruction for data and address */
     dramStart = dram_desc.dramBusAddr + dram_desc.startOffset;
     /* always use DRAM1 starting from offset 0 */
     addrHi = (unsigned int)((dramStart >> BITS_IN_LONGWORD) & 0xffffffff);
     addrLow = (unsigned int)(dramStart & 0xffffffff); 
     desc0 = ShramAddr;
     desc1 = (Size * BYTES_IN_LONGWORD - 1) << BYTE_LENGTH_OFFSET | 
             (QatNum + 0x2) << QATNUM_LEFTSHIFT | 
             (EndianSwap << ENDIAN_SWAP_OFFSET);
     INSERT_IMMED_GPRA_CONST(microInst[0x2], (addrHi >>  0));
     INSERT_IMMED_GPRA_CONST(microInst[0x3], (addrHi >> BITS_IN_WORD));
     INSERT_IMMED_GPRB_CONST(microInst[0x4], (addrLow >>  0));
     INSERT_IMMED_GPRB_CONST(microInst[0x5], (addrLow >> BITS_IN_WORD));
     INSERT_IMMED_GPRB_CONST(microInst[0x6], (desc1 >>  0));
     INSERT_IMMED_GPRB_CONST(microInst[0x7], (desc1 >> BITS_IN_WORD));
     INSERT_IMMED_GPRA_CONST(microInst[0x9], (desc0 >>  0));
     INSERT_IMMED_GPRA_CONST(microInst[0xa], (desc0 >> BITS_IN_WORD));
 
     /* get and save the value of gpr and xfer_out reg */
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPA_REL, 0, &gprA0);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPB_REL, 0, &gprB0);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPA_REL, 1, &gprA1);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPB_REL, 1, &gprB1);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_SR_WR_REL, 0, &sw0);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_SR_WR_REL, 1, &sw1);
 
     /* save the value in Debug Memory */
     for (i = 0; i < Size; i++)
     {
         dram[i] = READ_LWORD_CACHED(dram_desc.dramBaseAddr_v + 
                                     dram_desc.startOffset + (i << 0x2));
     }
 
     /* execute microcode snippet */
     status = execMicroInst(Handle, (unsigned char)ae, ctx, microInst, 
                            num_inst, condCodeOff, 
                            HALMEM_EXECMICRO_LONGTIME, NULL);
     if (status != HALAE_SUCCESS)
     {
         ERRINFO(("halMem_GetSharedRam: microcode execution failed "\
                  "(code = 0x%.8x)\n", status));
     }
     else    /* retrieve data from shram memory */
     {  
         for (i = 0; i < Size; i++)
         {
             Buffer[i] = READ_LWORD_CACHED(dram_desc.dramBaseAddr_v + 
                                           dram_desc.startOffset + (i << 0x2));
         }
     }
 
     /* restore the value in Debug Memory */
     for (i = 0; i < Size; i++)
     {
         WRITE_LWORD_CACHED(dram_desc.dramBaseAddr_v + 
                            dram_desc.startOffset + (i << 0x2), 
                            dram[i]);
     }
 
     /* restore the registers */
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPA_REL, 0, gprA0);
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPB_REL, 0, gprB0);    
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPA_REL, 1, gprA1);
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPB_REL, 1, gprB1);    
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_SR_WR_REL, 0, sw0);
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_SR_WR_REL, 1, sw1);

     halAe_ContinuousDramFree (Handle, &dram_desc); 
     osalMemFree (dram);
     return halMem_CnvHalAeRetCode(status);
 }
 
 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *      This function write data to SHRAM region
  *
  *
  *
  * @param dbgHandle - IN AC firmware loader handle
  * @param QatNum - IN Specifies the QAT instance
  * @param ae - IN Specifies the ae for executing microcode snippet
  * @param ShramAddr - IN Specifies the SHRAM address
  * @param Size - IN Specifies the size of data retrieved in Longwords
  * @param Buffer - IN A pointer to the location of the data to write.
  * @param EndianSwap - IN Specifies if the data need be swapped when writing.
  *
  * @retval HALMEM_SUCCESS The function completed successfully
  * @retval HALMEM_INVALID_PARAMETER  Invalid parameters
  * @retval HALMEM_DEVICE_ERROR Operation failed
  * @retval HALMEM_TIMEOUT The operation exceeded the time limit
  * 
  * 
  *****************************************************************************/
 int halMem_PutSharedRam(
     icp_firml_handle_t *Handle, 
     unsigned int QatNum, 
     unsigned int ae,
     unsigned int ShramAddr,
     unsigned int Size, /* in LongWords, at most MAX_SHRAM_USED is allowed */
     unsigned int *Buffer,
     unsigned int EndianSwap)
 {
     unsigned int gprA0, gprA1, gprB0, gprB1, sw0, sw1;
     unsigned int * dram=NULL;
     int status = HALAE_SUCCESS;
     uint64 dramStart;
     unsigned int addrHi, addrLow, desc0, desc1;
     unsigned int i;
 
     uword_T microInst[] = 
     {
         /*
          * Resources Used:
          * GPR:     A0, A1, B0, B1
          * XFER:    SRAM W0, W1
          * DRAM1:   offset 0 .. 1023
          *
          */
         /*
                 .reg $xfer0, $xfer1
                 .xfer_order $xfer0 $xfer1
                 .addr $xfer0 0
                 .addr $xfer1 1
 
                 .reg addr_hi
                 .reg addr_low
                 .reg desc0
                 .reg desc1
                 .areg addr_hi 0
                 .breg addr_low 0
                 .areg desc0 1
                 .breg desc1 1
                 .sig sig_dt
                 .addr sig_dt 2
 
                 br=ctx[0, start#]
                 br[kill_context#]
 
             start#:
                 immed_w0[addr_hi, 0]
                 immed_w1[addr_hi, 0]
                 immed_w0[addr_low, 0]
                 immed_w1[addr_low, 0]
 
                 immed_w0[desc1, 0]
                 immed_w1[desc1, 0]
                 alu[$xfer1, --, b, desc1]
 
                 immed_w0[desc0, 0]
                 immed_w1[desc0, 0]
                 alu[$xfer0, --, b, desc0]
 
                 ram[data_transfer_read, $xfer0, addr_hi, addr_low, 1], 
                       sig_done[sig_dt], no_endian_swap, 64_bit_addr, dual_sig
                       
             wait_for_sig_dt0#:
                 br_!signal[sig_dt[0], wait_for_sig_dt0#]
             wait_for_sig_dt1#:
                 br_!signal[sig_dt[1], wait_for_sig_dt1#]
                 nop
                 nop
                 nop
                 nop
                 nop
 
             kill_context#:
                 ctx_arb[kill], any
         */
         0x0D800800010ull, /* .0 br=ctx[0, start#] */
         0x0D804C00018ull, /* .1 br[kill_context#] */
         0x0F0400C0000ull, /* .2 immed_w0[l0000!addr_hi, 0] */
         0x0F4400C0000ull, /* .3 immed_w1[l0000!addr_hi, 0] */
         0x0F040000300ull, /* .4 immed_w0[l0000!addr_low, 0] */
         0x0F440000300ull, /* .5 immed_w1[l0000!addr_low, 0] */
         0x0F040000700ull, /* .6 immed_w0[l0000!desc1, 0] */
         0x0F440000700ull, /* .7 immed_w1[l0000!desc1, 0] */
         0x0A018100400ull, /* .8 alu[$l0000!xfer1, --, b, l0000!desc1] */
         0x0F0400C0001ull, /* .9 immed_w0[l0000!desc0, 0] */
         0x0F4400C0001ull, /* .10 immed_w1[l0000!desc0, 0] */
         0x0A0580C0001ull, /* .11 alu[$l0000!xfer0, --, b, l0000!desc0] */
         0x80220000300ull, /* .12 ram[data_transfer_read, 
                                                    $l0000!xfer0, l0000!addr_hi, 
                                                    l0000!addr_low, 1], 
                                                    sig_done[l0000!sig_dt], 
                                                    no_endian_swap,
                                                    64_bit_addr, dual_sig */
         0x0D803408213ull, /* .13 wait_for_sig_dt0#: 
                                                   br_!signal[l0000!sig_dt[0], 
                                                  wait_for_sig_dt0#] */
         0x0D80380C213ull, /* .14 wait_for_sig_dt1#: 
                                                   br_!signal[l0000!sig_dt[1], 
                                                         wait_for_sig_dt1#] */
         0x0F0000C0300ull, /* .15 nop */
         0x0F0000C0300ull, /* .16 nop */
         0x0F0000C0300ull, /* .17 nop */
         0x0F0000C0300ull, /* .18 nop */
         0x0E000010000ull, /* .19 kill_context#: ctx_arb[kill], any */
     };
 
     const int num_inst = sizeof(microInst)/sizeof(microInst[0]);
     const int condCodeOff = 1;
     const unsigned char ctx = 0;
     icp_firml_dram_desc_t dram_desc;

     status = halAe_ContinuousDramAlloc(Handle, &dram_desc, 
                                        RAM_CHUNK_INIT_SIZE);
     if (status != HALAE_SUCCESS)
     {
         ERRINFO(("halMem_PutSharedRam: function halAe_ContinuousDramAlloc "\
                  "failed (code = 0x%.8x)\n", status));
         return HALMEM_DEVICE_ERROR;
     }

     dram = (unsigned int*)osalMemAlloc(MAX_SSU_SHARED_RAM);
     if (dram == NULL)
     {   
         halAe_ContinuousDramFree (Handle, &dram_desc);
         ERRINFO(("Fail to allocate memory\n"));    
         return HALMEM_DEVICE_ERROR;
     }
     osalMemSet(dram, 0, MAX_SSU_SHARED_RAM);

     if (Size == 0) 
     {
         halAe_ContinuousDramFree (Handle, &dram_desc);
         osalMemFree (dram);
         return HALMEM_SUCCESS;
     }
 
     /* fixup instruction for data and address */
     dramStart = dram_desc.dramBusAddr + dram_desc.startOffset;
     /* always use DRAM1 starting from offset 0 */
     addrHi = (unsigned int)((dramStart >> BITS_IN_LONGWORD) & 0xffffffff);
     addrLow = (unsigned int)(dramStart & 0xffffffff); 
     desc0 = ShramAddr;
     desc1 = (Size * BYTES_IN_LONGWORD - 1) << BYTE_LENGTH_OFFSET |
             (QatNum + 0x2) << QATNUM_LEFTSHIFT | 
             (EndianSwap << ENDIAN_SWAP_OFFSET);
     INSERT_IMMED_GPRA_CONST(microInst[0x2], (addrHi >>  0));
     INSERT_IMMED_GPRA_CONST(microInst[0x3], (addrHi >> BITS_IN_WORD));
     INSERT_IMMED_GPRB_CONST(microInst[0x4], (addrLow >>  0));
     INSERT_IMMED_GPRB_CONST(microInst[0x5], (addrLow >> BITS_IN_WORD));
     INSERT_IMMED_GPRB_CONST(microInst[0x6], (desc1 >>  0));
     INSERT_IMMED_GPRB_CONST(microInst[0x7], (desc1 >> BITS_IN_WORD));
     INSERT_IMMED_GPRA_CONST(microInst[0x9], (desc0 >>  0));
     INSERT_IMMED_GPRA_CONST(microInst[0xa], (desc0 >> BITS_IN_WORD));
 
     /* get and save the value of gpr and xfer_out reg */
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPA_REL, 0, &gprA0);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPB_REL, 0, &gprB0);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPA_REL, 1, &gprA1);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPB_REL, 1, &gprB1);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_SR_WR_REL, 0, &sw0);
     halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_SR_WR_REL, 1, &sw1);
 
     /* save the value in Debug Memory */
     for (i = 0; i < Size; i++)
     {
         dram[i] = READ_LWORD_CACHED(dram_desc.dramBaseAddr_v + 
                                     dram_desc.startOffset + (i << 0x2));
     }
 
     /* retrieve data from shram memory */
     for (i = 0; i < Size; i++)
     {
         WRITE_LWORD_CACHED(dram_desc.dramBaseAddr_v + 
                            dram_desc.startOffset + (i << 0x2), 
                            Buffer[i]);
     }
 
     /* execute microcode snippet */
     status = execMicroInst(Handle, (unsigned char)ae, ctx, microInst,
                            num_inst, condCodeOff, 
                            HALMEM_EXECMICRO_LONGTIME, NULL);
     if (status != HALAE_SUCCESS)
     {
         ERRINFO(("halMem_PutSharedRam: microcode execution failed "\
                  "(code = 0x%.8x)\n", status));
     }     

     /* restore the value in Debug Memory */
     for (i = 0; i < Size; i++)
     {
         WRITE_LWORD_CACHED(dram_desc.dramBaseAddr_v +
                            dram_desc.startOffset +  (i << 0x2),
                            dram[i]);
     }
 
     /* restore the registers */
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPA_REL, 0, gprA0);
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPB_REL, 0, gprB0);
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPA_REL, 1, gprA1);
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_GPB_REL, 1, gprB1);
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_SR_WR_REL, 0, sw0);
     halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                         ICP_SR_WR_REL, 1, sw1);

     halAe_ContinuousDramFree (Handle, &dram_desc);
     osalMemFree (dram);
     return halMem_CnvHalAeRetCode(status);
 }

/**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *      This function move data from SHRAM region to DRAM
  *
  *
  *
  * @param dbgHandle - IN AC firmware loader handle
  * @param QatNum - IN Specifies the QAT instance
  * @param ae - IN Specifies the ae for executing microcode snippet
  * @param ShramAddr - IN Specifies the SHRAM address
  * @param Size - IN Specifies the size of data retrieved in Longwords
  * @param DRamAddr - IN Physical DRAM address.
  * @param EndianSwap - IN Specifies if the data need be swapped when retrieving.
  *
  * @retval HALMEM_SUCCESS The function completed successfully
  * @retval HALMEM_INVALID_PARAMETER  Invalid parameters
  * @retval HALMEM_DEVICE_ERROR Operation failed
  * @retval HALMEM_TIMEOUT The operation exceeded the time limit
  * 
  * 
  *****************************************************************************/
int halMem_SharedRamToDRam(
      icp_firml_handle_t *Handle, 
      unsigned int QatNum, 
      unsigned int ae,
      unsigned int ShramAddr,
      unsigned int Size,
      unsigned int *DRamAddr,
      unsigned int EndianSwap)
{
      unsigned int gprA0, gprA1, gprB0, gprB1, sw0, sw1;
      int status = HALAE_SUCCESS;
      uint64 dramStart;
      unsigned int addrHi, addrLow, desc0, desc1;
  
      uword_T microInst[] = 
      {
          /*
           * Resources Used:
           * GPR:      A0, A1, B0, B1
           * XFER:      SRAM W0, W1
           * DRAM1:   offset 0 .. 1023
           *
           */
          /*
                  .reg $xfer0, $xfer1
                  .xfer_order $xfer0 $xfer1
                  .addr $xfer0 0
                  .addr $xfer1 1
  
                  .reg addr_hi
                  .reg addr_low
                  .reg desc0
                  .reg desc1
                  .areg addr_hi 0
                  .breg addr_low 0
                  .areg desc0 1
                  .breg desc1 1
                  .sig sig_dt
                  .addr sig_dt 2
  
                  br=ctx[0, start#]
                  br[kill_context#]
  
              start#:
                  immed_w0[addr_hi, 0]
                  immed_w1[addr_hi, 0]
                  immed_w0[addr_low, 0]
                  immed_w1[addr_low, 0]
  
                  immed_w0[desc1, 0]
                  immed_w1[desc1, 0]
                  alu[$xfer1, --, b, desc1]
  
                  immed_w0[desc0, 0]
                  immed_w1[desc0, 0]
                  alu[$xfer0, --, b, desc0]
  
                  ram[data_transfer_write, $xfer0, addr_hi, addr_low, 1], sig_done[sig_dt], no_endian_swap, 64_bit_addr, dual_sig
              wait_for_sig_dt0#:
                  br_!signal[sig_dt[0], wait_for_sig_dt0#]
              wait_for_sig_dt1#:
                  br_!signal[sig_dt[1], wait_for_sig_dt1#]
                  nop
                  nop
                  nop
                  nop
  
              kill_context#:
                  ctx_arb[kill], any
          */
          0x0D800800010ull, /* .0 br=ctx[0, start#] */
          0x0D804C00018ull, /* .1 br[kill_context#] */
          0x0F0400C0000ull, /* .2 immed_w0[l0000!addr_hi, 0] */
          0x0F4400C0000ull, /* .3 immed_w1[l0000!addr_hi, 0] */
          0x0F040000300ull, /* .4 immed_w0[l0000!addr_low, 0] */
          0x0F440000300ull, /* .5 immed_w1[l0000!addr_low, 0] */
          0x0F040000700ull, /* .6 immed_w0[l0000!desc1, 0] */
          0x0F440000700ull, /* .7 immed_w1[l0000!desc1, 0] */
          0x0A018100400ull, /* .8 alu[$l0000!xfer1, --, b, l0000!desc1] */
          0x0F0400C0001ull, /* .9 immed_w0[l0000!desc0, 0] */
          0x0F4400C0001ull, /* .10 immed_w1[l0000!desc0, 0] */
          0x0A0580C0001ull, /* .11 alu[$l0000!xfer0, --, b, l0000!desc0] */
          0x80320000300ull, /* .12 ram[data_transfer_write, $l0000!xfer0,
                                                   l0000!addr_hi, 
                                                   l0000!addr_low, 1], 
                                                   sig_done[l0000!sig_dt], 
                                                   no_endian_swap, 64_bit_addr,
                                                   dual_sig */
          0x0D803408213ull, /* .13 wait_for_sig_dt0#: 
                                                   br_!signal[l0000!sig_dt[0],
                                                   wait_for_sig_dt0#] */
          0x0D80380C213ull, /* .14 wait_for_sig_dt1#: 
                                                    br_!signal[l0000!sig_dt[1],
                                                    wait_for_sig_dt1#] */
          0x0F0000C0300ull, /* .15 nop */
          0x0F0000C0300ull, /* .16 nop */
          0x0F0000C0300ull, /* .17 nop */
          0x0F0000C0300ull, /* .18 nop */
          0x0E000010000ull, /* .19 kill_context#: ctx_arb[kill], any */
      };
  
      const int num_inst = sizeof(microInst)/sizeof(microInst[0]);
      const int condCodeOff = 1;
      const unsigned char ctx = 0;

      if (Size > MAX_SHRAM_USED)
      {
          return HALMEM_DEVICE_ERROR;
      }

      /* fixup instruction for data and address */
      dramStart = (long)DRamAddr;
      addrHi = (unsigned int)
               ((dramStart >> BITS_IN_LONGWORD) & 0xffffffff);
      addrLow = (unsigned int)(dramStart & 0xffffffff);
      
      desc0 = ShramAddr;
      desc1 = (Size * BYTES_IN_LONGWORD - 1) << BYTE_LENGTH_OFFSET | 
              (QatNum + 0x2) << QATNUM_LEFTSHIFT | 
              (EndianSwap << ENDIAN_SWAP_OFFSET);
      INSERT_IMMED_GPRA_CONST(microInst[0x2], (addrHi >>  0));
      INSERT_IMMED_GPRA_CONST(microInst[0x3], (addrHi >> BITS_IN_WORD));
      INSERT_IMMED_GPRB_CONST(microInst[0x4], (addrLow >>  0));
      INSERT_IMMED_GPRB_CONST(microInst[0x5], (addrLow >> BITS_IN_WORD));
      INSERT_IMMED_GPRB_CONST(microInst[0x6], (desc1 >>  0));
      INSERT_IMMED_GPRB_CONST(microInst[0x7], (desc1 >> BITS_IN_WORD));
      INSERT_IMMED_GPRA_CONST(microInst[0x9], (desc0 >>  0));
      INSERT_IMMED_GPRA_CONST(microInst[0xa], (desc0 >> BITS_IN_WORD));
  
      /* get and save the value of gpr and xfer_out reg */
      halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_GPA_REL, 0, &gprA0);
      halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_GPB_REL, 0, &gprB0);
      halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_GPA_REL, 1, &gprA1);
      halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_GPB_REL, 1, &gprB1);
      halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_SR_WR_REL, 0, &sw0);
      halAe_GetRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_SR_WR_REL, 1, &sw1);
  
      /* execute microcode snippet */
      status = execMicroInst(Handle, (unsigned char)ae, ctx, microInst, 
                             num_inst, condCodeOff, 
                             HALMEM_EXECMICRO_LONGTIME, NULL);
      if (status != HALAE_SUCCESS)
      {
          ERRINFO(("halMem_SharedRamToDRam: microcode execution failed "\
                   "(code = 0x%.8x)\n", status));
      }
  
      /* restore the registers */
      halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_GPA_REL, 0, gprA0);
      halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_GPB_REL, 0, gprB0);     
      halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_GPA_REL, 1, gprA1);
      halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_GPB_REL, 1, gprB1);     
      halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_SR_WR_REL, 0, sw0);
      halAe_PutRelDataReg(Handle, (unsigned char)ae, ctx, 
                          ICP_SR_WR_REL, 1, sw1);
 
      return halMem_CnvHalAeRetCode(status);
}


 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *      This function reads SSU shared RAM using an idle AE to perform the read
    instructions.  
  *
  *
  *
  * @param Ae - IN this idle AE will be used to perform the read 
  * @param QatNum - IN
  * @param Start - IN
 
  * @param Longwords - IN
  * @param Buffer - OUT
  *
  * @retval  
  * 
  * 
  *****************************************************************************/
 int 
 halMem_ReadSharedRam(const char* FunctionName,
                              icp_firml_handle_t *Handle,
                              unsigned int Ae,  
                              unsigned int QatNum, 
                              unsigned int Start, 
                              unsigned int Longwords, 
                              unsigned int* Buffer)
 {   
     unsigned int status;
     unsigned char ii;
     /* in LongWords. At most 1024 bytes are allowed in 
          one function call per hardware limitation */
     const unsigned int MaxSizePerCall = MAX_SHRAM_USED; 
     unsigned int RemainedSize = Longwords;
     unsigned int ReadOffset = 0;
     unsigned int ReadSize;
     
     if ((Buffer == 0) || (Longwords == 0) || 
         ((Start + Longwords) > SHARED_SRAM_SIZE))
     {
         ERRINFO(("%s: Badarg\n", FunctionName));
         return HALMEM_INVALID_PARAMETER;
     }
 
     if(halAe_GetExecutableAe(Handle, &ii)!= HALAE_SUCCESS)
     {
         return HALMEM_AEACTIVE;
     }
     Ae = (unsigned int)ii;
 
     status = HALMEM_SUCCESS;  /* assume success */
 
     while (RemainedSize > 0)
     {
         if (RemainedSize >= MaxSizePerCall)
         {
             ReadSize = MaxSizePerCall;
         }
         else
         {
             ReadSize = RemainedSize;
         }
 
         /* Read the SSU Shared RAM by iteratively calling 
                    halMem_GetSharedRam - make sure we pass a byte 
                    address instead of a longword index */
         status = halMem_GetSharedRam(Handle, QatNum, Ae, 
                                      (Start + ReadOffset) << 0x2, 
                                      ReadSize, Buffer + ReadOffset, 0);
         if(HALMEM_ERROR (status))
         {
             ERRINFO(("halMem_GetSharedRam returned 0x%.8x, "\
                      "%d longwords have been read.\n", status, ReadOffset));
             return status;
         }
 
         RemainedSize -= ReadSize;
         ReadOffset += ReadSize;
     }
 
     DBGINFO(("%s: Success\n", FunctionName));
     return status;
 }
 
 /**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *      This function writes SSU shared RAM using an idle AE to perform the write
    instructions.  
  *
  *
  *
  * @param Ae - IN  this idle AE will be used to perform the read
  * @param QatNum - IN
  * @param Start - IN 
  * @param Longwords - IN
  * @param Buffer - IN
  *
  * @retval  
  * 
  * 
  *****************************************************************************/
 int 
 halMem_WriteSharedRam(const char* FunctionName,
                              icp_firml_handle_t *Handle,
                              unsigned int Ae, 
                              unsigned int QatNum, 
                              unsigned int Start, 
                              unsigned int Longwords, 
                              unsigned int* Buffer)
 {
     unsigned int status;
     unsigned char ii;
     /* in LongWords. At most 1024 bytes are allowed in one function 
          call per hardware limitation */
     const unsigned int MaxSizePerCall = MAX_SHRAM_USED; 
     unsigned int RemainedSize = Longwords;
     unsigned int WriteOffset = 0;
     unsigned int WriteSize;
 
     if ((Buffer == 0) || (Longwords == 0) || 
         ((Start + Longwords) > SHARED_SRAM_SIZE))
     {
         ERRINFO(("%s: Badarg\n", FunctionName));
         return HALMEM_INVALID_PARAMETER;
     }
 
     if(halAe_GetExecutableAe(Handle, &ii)!= HALAE_SUCCESS)
     {
         return HALMEM_AEACTIVE;
     }
     Ae = (unsigned int)ii;

     while (RemainedSize > 0)
     {
         if (RemainedSize >= MaxSizePerCall)
         {
             WriteSize = MaxSizePerCall;
         }
         else
         {
             WriteSize = RemainedSize;
         }
 
         /* Write the SSU Shared RAM by iteratively calling 
                    halMem_PutSharedRam - make sure we pass a byte 
                    address instead of a longword index */
         status = halMem_PutSharedRam(Handle, QatNum, Ae, 
                                     (Start + WriteOffset) << 0x2, 
                                     WriteSize, Buffer + WriteOffset, 0);
         if(HALMEM_ERROR (status))
         {
             ERRINFO(("halMem_PutSharedRam returned 0x%.8x, "\
                      "%d longwords have been read.\n", status, WriteOffset));
             return status;
         }
 
         RemainedSize -= WriteSize;
         WriteOffset += WriteSize;
     }

     DBGINFO(("%s: Success\n", FunctionName));
     return(HALMEM_SUCCESS);
 }

/**
  *****************************************************************************
  * @ingroup icp_hal
  * 
  * @description
  *      Write 64K Shared RAM
  *      
  * @assumptions
  *      None
  * @sideEffects
  *      None
  * @blocking
  *      No
  * @reentrant
  *      No
  * @threadSafe
  *      Yes
  * 
  * @param handle - IN A pointer to internal data structure which contains 
  *                           useful info, e.g. blocks' base address
  *
  * @retval HALMEM_SUCCESS Operation was successful
  * @retval HALMEM_INVALID_PARAMETER  Invalid parameters
  * @retval HALMEM_DEVICE_ERROR          Operation failed
  * 
  * 
  *****************************************************************************/
int 
halMem_ScrubSharedRam(icp_firml_handle_t *handle)
{
    int Status = HALMEM_SUCCESS, halAeStatus = HALAE_SUCCESS;
    unsigned int AddrHi, AddrLow;
    unsigned int EndPC;
    unsigned int i, MaxEnabledQatNum;
    unsigned int maxNumQats = 0;
    const int condCodeOff = 0;
    unsigned int *dram=NULL;
    uint64 dramStart;
    unsigned int gprA0, gprA1, gprB0, gprB1, sw0, sw1;

    uword_T MicroInst[] = 
    {
         //
         // Resources Used:
         // GPR:     A0, A1, A2, A3, B0, B1, B2, B3
         // XFER:    SRAM W0, W1
         //
     /*
                .reg $xfer0, $xfer1
                .xfer_order $xfer0 $xfer1
                .addr $xfer0 0
                .addr $xfer1 1

                .reg addr_hi
                .reg addr_low
                .reg desc0
                .reg desc1
                .reg size
                .reg numQats
                .reg qatMask
                .reg checkQat
                .reg cnt
                .reg blk
                .areg addr_hi 0
                .breg addr_low 0
                .areg desc0 1
                .breg desc1 1
                .areg numQats 2
                .areg qatMask 4
                .breg checkQat 4
                .breg size 2
                .areg blk 3
                .breg cnt 3
                .sig sig_dt
                .addr sig_dt 2

                br=ctx[0, start#]
                br[kill_context#]

            start#:
                immed_w0[numQats, 0] ; the number of qats, need fixup
                immed_w1[numQats, 0] ; the number of qats, need fixup
                immed_w0[qatMask, 0] ; need fixup
                immed_w1[qatMask, 0] ; need fixup
                immed_w0[addr_hi, 0] ; dram address, need fixup
                immed_w1[addr_hi, 0] ; dram address, need fixup
                immed_w0[addr_low, 0] ; dram address, need fixup
                immed_w1[addr_low, 0] ; dram address, need fixup
                immed_w0[size, 1024]
                immed_w1[size, 0]
                nop
            init_qat#:
                ; 2 QATs: qat0 and qat1, blk2 and blk3
                alu[blk, numQats, +, 1]
                nop

                ; add logic for "if((!qatMask&(1<<qatNum))) continue;"
                alu[numQats, numQats, -, 1]
                nop
                blo[kill_context#]
                nop
                alu[--, numQats, B, checkQat]
                alu_shf[--, qatMask, AND, 1, <<indirect]
                nop
                beq[init_qat#]
                nop
                alu[numQats, numQats, +, 1]
        
                ;desc1 = (ByteSize - 1) << 6 | (qat + 0x2) << 2 | (EndianSwap << 30)
                ;shared ram byte size = 0x3FF = 1KB, QAT = 0|1, no endian swap
                immed_w0[desc1, 0xFFC0]
                immed_w1[desc1, 0x00]
                alu_shf[$xfer1, desc1, or, blk, <<2] ; (qat+0x2)<<2
                nop
                immed_w0[desc0, 0] ; shram byte address
                immed_w1[desc0, 0] ; shram byte address
                alu[$xfer0, --, b, desc0]
                immed_w0[cnt, 64] ; shram init size = cnt * 1KB = 64KB
                immed_w1[cnt, 0]

            init_shram#:
                nop
                ram[data_transfer_read, $xfer0, addr_hi, addr_low, 1], sig_done[sig_dt], no_endian_swap, 64_bit_addr, dual_sig
            wait_for_sig_dt0#:
                br_!signal[sig_dt[0], wait_for_sig_dt0#]
            wait_for_sig_dt1#:
                br_!signal[sig_dt[1], wait_for_sig_dt1#]
                nop
                nop
                nop
                nop
                alu[desc0, desc0, +, size] ; 1KB
                nop
                alu[$xfer0, --, b, desc0]
                nop
                alu[cnt, cnt, -, 1]
                nop
                bne[init_shram#]
                nop
                alu[numQats, numQats, -, 1]
                nop
                bne[init_qat#]
                nop

            kill_context#:
                ctx_arb[kill], any
        */

        0x0D800800010ull, /* .0 br=ctx[0, start#] */
        0x0D80D800018ull, /* .1 br[kill_context#] */
        0x0F0400C0002ull, /* .2 start#: immed_w0[l0000!numQats, 0] ; 
                                the number of qats */
        0x0F4400C0002ull, /* .3 immed_w1[l0000!numQats, 0] ; 
                                the number of qats */
        0x0F0400C0004ull, /* .4 immed_w0[l0000!qatMask, 0] ; need fixup */
        0x0F4400C0004ull, /* .5 immed_w1[l0000!qatMask, 0] ; need fixup */
        0x0F0400C0000ull, /* .6 immed_w0[l0000!addr_hi, 0] ; dram address */
        0x0F4400C0000ull, /* .7 immed_w1[l0000!addr_hi, 0] ; dram address */
        0x0F040000300ull, /* .8 immed_w0[l0000!addr_low, 0] ; dram address */
        0x0F440000300ull, /* .9 immed_w1[l0000!addr_low, 0] ; dram address */
        0x0F040400B00ull, /* .10 immed_w0[l0000!size, 1024] */
        0x0F440000B00ull, /* .11 immed_w1[l0000!size, 0] */
        0x0F0000C0300ull, /* .12 nop */
        0x0A0803C0402ull, /* .13 init_qat#: alu[l0000!blk, l0000!numQats, +,
                                 1] ; 2 QATs: qat0 and qat1, blk2 and blk3 */
        0x0F0000C0300ull, /* .14 nop */

        0x0AA802C0402ull, /* .15 alu[l0000!numQats, l0000!numQats, -, 1] ;
                                 add logic for "if((!qatMask&(1<<qatNum))) 
                                 continue;" */
        0x0F0000C0300ull,  /* .16 nop */
        0x0D80D800005ull, /* .17 blo[kill_context#] */
        0x0F0000C0300ull, /* .18 nop */
        0x0A030001002ull, /* .19 alu[--, l0000!numQats, b, l0000!checkQat] */
        0x08402008604ull, /* .20 alu_shf[--, l0000!qatMask, and, 1, 
                                 <<indirect] */
        0x0F0000C0300ull, /* .21 nop */
        0x0D803400000ull, /* .22 beq[init_qat#] */
        0x0F0000C0300ull, /* .23 nop */
        0x0A0802C0402ull, /* .24 alu[l0000!numQats, l0000!numQats, +, 1] */

        0x0F04FF007C0ull, /* .25 immed_w0[l0000!desc1, 0xFFC0] */
        0x0F440000700ull, /* .26 immed_w1[l0000!desc1, 0x00] */
        0x08BE8180603ull, /* .27 alu_shf[$l0000!xfer1, l0000!desc1, 
                                 or, l0000!blk, <<2] ; (qat+0x2)<<2 */
        0x0F0000C0300ull, /* .28 nop */
        0x0F0400C0001ull, /* .29 immed_w0[l0000!desc0, 0] ;shram byte address */
        0x0F4400C0001ull, /* .30 immed_w1[l0000!desc0, 0] ;shram byte address */
        0x0A0580C0001ull, /* .31 alu[$l0000!xfer0, --, b, l0000!desc0] */
        0x0F040000F40ull, /* .32 immed_w0[l0000!cnt, 64] ; 
                                 shram init size = cnt * 1KB = 64KB */
        0x0F440000F00ull, /* .33 immed_w1[l0000!cnt, 0] */
        0x0F0000C0300ull, /* .34 init_shram#: nop */
        0x80220000300ull, /* .35 ram[data_transfer_read, $l0000!xfer0, 
                                 l0000!addr_hi, l0000!addr_low, 1], 
                                 sig_done[l0000!sig_dt], no_endian_swap,
                                 64_bit_addr, dual_sig */
        0x0D809008213ull, /* .36 wait_for_sig_dt0#: br_!signal[l0000!sig_dt[0],
                                 wait_for_sig_dt0#] */
        0x0D80940C213ull, /* .37 wait_for_sig_dt1#: br_!signal[l0000!sig_dt[1],
                                 wait_for_sig_dt1#] */
        0x0F0000C0300ull, /* .38 nop */
        0x0F0000C0300ull, /* .39 nop */
        0x0F0000C0300ull, /* .40 nop */
        0x0F0000C0300ull, /* .41 nop */
        0x0A080100801ull, /* .42 alu[l0000!desc0, l0000!desc0, +, l0000!size] ;
                                 1KB */
        0x0F0000C0300ull, /* .43 nop */
        0x0A0580C0001ull, /* .44 alu[$l0000!xfer0, --, b, l0000!desc0] */
        0x0F0000C0300ull, /* .45 nop */
        0x0BAC0300F01ull, /* .46 alu[l0000!cnt, l0000!cnt, -, 1] */
        0x0F0000C0300ull, /* .47 nop */
        0x0D808800001ull, /* .48 bne[init_shram#] */
        0x0F0000C0300ull, /* .49 nop */
        0x0AA802C0402ull, /* .50 alu[l0000!numQats, l0000!numQats, -, 1] */
        0x0F0000C0300ull, /* .51 nop */
        0x0D803400001ull, /* .52 bne[init_qat#] */
        0x0F0000C0300ull, /* .53 nop */
        0x0E000010000ull, /* .54 kill_context#: ctx_arb[kill], any */
    };
    const int NumInsts = sizeof(MicroInst)/sizeof(MicroInst[0]);
    unsigned char Ctx = 0, Ae = 0;
    uint64 DramAddr;
    unsigned int QatMask = 0, Qat;
    unsigned int* SHRamState=NULL;
    icp_firml_dram_desc_t dram_desc;

    if (handle == NULL)
    {
        ERRINFO(("Invalid parameter\n"));   
        return HALMEM_INVALID_PARAMETER;
    }

    halAeStatus = halAe_ContinuousDramAlloc(handle, &dram_desc, 
                                            RAM_CHUNK_INIT_SIZE);
    if (halAeStatus != HALAE_SUCCESS)
    {
        ERRINFO(("halMem_ScrubSharedRam: function halAe_ContinuousDramAlloc "\
                 "failed (code = 0x%.8x)\n", halAeStatus));
        return HALMEM_DEVICE_ERROR;
    }

    dram = (unsigned int*)osalMemAlloc(KILO_1);
    if (dram == NULL)
    {   
        halAe_ContinuousDramFree (handle, &dram_desc);
        ERRINFO(("Fail to allocate memory\n"));    
        return HALMEM_DEVICE_ERROR;
    }
    osalMemSet(dram, 0, KILO_1);

    MAX_VAL(maxNumQats, 
            ACCELCOMP_NUM_QATS, 
            ACCELCOMP_C_NUM_QATS, 
            ACCELCOMP_R_NUM_QATS, 
            ACCELCOMP_B_NUM_QATS, 
            ACCELCOMP_RS_NUM_QATS); 

    SHRamState = (unsigned int*)osalMemAlloc(maxNumQats*sizeof(unsigned int));
    if (SHRamState == NULL)
    {
        ERRINFO(("halMem_ScrubSharedRam: Fail to allocate memory\n"));   
        halAe_ContinuousDramFree (handle, &dram_desc);
        osalMemFree(dram);
        return HALMEM_DEVICE_ERROR;
    }   
    osalMemSet(SHRamState, 0, maxNumQats*sizeof(unsigned int));
    
    for (Qat = 0; Qat < maxNumQats; Qat ++)
    {
        if (!(handle->sysMemInfo.qatMask & (1<<Qat))) 
        {
            continue;
        }
        // power on SHRam, and check SHRam start and end state
        SHRamState[Qat] = halMem_EnableQat(handle, Qat, QAT_SHRAM_TYPE);
        if (POWER_ON == ((SHRamState[Qat] & END_MASK) >> END_BITPOS)) 
        {
            QatMask |= (1<<Qat);
        }
    }      
   
    if (QatMask == 0)
    {
        // No SHRam is power-on
        halAe_ContinuousDramFree (handle, &dram_desc);
        osalMemFree(dram);
        osalMemFree(SHRamState);
        return HALMEM_SUCCESS;
    }
    
    do
    {
        Status = halAe_GetExecutableAe(handle, &Ae);        
        if (HALMEM_ERROR (Status))
        {
            ERRINFO(("Fail to GetExecutableAe \n"));
            Status = HALMEM_DEVICE_ERROR;
            break;
        }
        MaxEnabledQatNum = 0;
        for(i = 1; i <= maxNumQats; i++)
        {
            if((1<<(i-1)) & QatMask) 
            {
                MaxEnabledQatNum = i;
            }
        }
        DBGINFO(("MaxEnabledQatNum=%d, QatMask=0x%x\n", 
                  MaxEnabledQatNum, QatMask));

        //
        // fixup instruction for MaxEnabledQatNum and Dram Address
        // 
        dramStart = dram_desc.dramBusAddr + dram_desc.startOffset;      
        AddrHi = (unsigned int)((dramStart >> BITS_IN_LONGWORD) & 0xffffffff);
        AddrLow = (unsigned int)(dramStart & 0xffffffff); 
        INSERT_IMMED_GPRA_CONST(MicroInst[0x2], 
                                (MaxEnabledQatNum >>  0));
        INSERT_IMMED_GPRA_CONST(MicroInst[0x3], 
                                (MaxEnabledQatNum >> BITS_IN_WORD));
        INSERT_IMMED_GPRA_CONST(MicroInst[0x4], 
                                (QatMask >>  0));
        INSERT_IMMED_GPRA_CONST(MicroInst[0x5],
                                (QatMask >> BITS_IN_WORD));
        INSERT_IMMED_GPRA_CONST(MicroInst[0x6], 
                                (AddrHi >>  0));
        INSERT_IMMED_GPRA_CONST(MicroInst[0x7], 
                                (AddrHi >> BITS_IN_WORD));
        INSERT_IMMED_GPRB_CONST(MicroInst[0x8],
                                (AddrLow >>  0));
        INSERT_IMMED_GPRB_CONST(MicroInst[0x9],
                                (AddrLow >> BITS_IN_WORD));

        /* get and save the value of gpr and xfer_out reg */
        halAe_GetRelDataReg(handle, (unsigned char)Ae, Ctx, 
                            ICP_GPA_REL, 0, &gprA0);
        halAe_GetRelDataReg(handle, (unsigned char)Ae, Ctx, 
                            ICP_GPB_REL, 0, &gprB0);
        halAe_GetRelDataReg(handle, (unsigned char)Ae, Ctx, 
                            ICP_GPA_REL, 1, &gprA1);
        halAe_GetRelDataReg(handle, (unsigned char)Ae, Ctx, 
                            ICP_GPB_REL, 1, &gprB1);
        halAe_GetRelDataReg(handle, (unsigned char)Ae, Ctx, 
                            ICP_SR_WR_REL, 0, &sw0);
        halAe_GetRelDataReg(handle, (unsigned char)Ae, Ctx, 
                            ICP_SR_WR_REL, 1, &sw1);
    
        /* save the value in Debug Memory */
        for (i = 0; i < (KILO_1)/sizeof(unsigned int); i++)
        {
            dram[i] = READ_LWORD_CACHED(dram_desc.dramBaseAddr_v + 
                                        dram_desc.startOffset + (i << 0x2));
        }
     
        //
        // Clear 1KB Dram Region 
        // 
        DramAddr = dram_desc.dramBaseAddr_v + dram_desc.startOffset;
        // 256 DWORDs = 1K Bytes
        for (i = 0; i < (KILO_1)/sizeof(unsigned int); i++)
        {
            WRITE_LWORD(DramAddr, 0x0);
            DramAddr += sizeof(unsigned int);
        }
  
        //
        // execute microcode snippet 
        // 
        Status = execMicroInst(handle, (unsigned char)Ae, Ctx, MicroInst,
                               NumInsts, condCodeOff, 
                               HALMEM_EXECMICRO_LONGTIME, &EndPC);

        if (HALMEM_ERROR (Status))
        {
            ERRINFO(("Fail to execMicroInst\n"));
            Status = HALMEM_DEVICE_ERROR;
            break;
        }
        if (EndPC != NumInsts)
        {
            ERRINFO(("Wrong EndPC=0x%x, expected=0x%x\n", EndPC, NumInsts));
            Status = HALMEM_DEVICE_ERROR;
            break;
        }

         /* restore the value in Debug Memory */
         for (i = 0; i < (KILO_1)/sizeof(unsigned int); i++)
         {
             WRITE_LWORD_CACHED(dram_desc.dramBaseAddr_v +
                                dram_desc.startOffset +  (i << 0x2),
                                dram[i]);
         }
     
         /* restore the registers */
         halAe_PutRelDataReg(handle, (unsigned char)Ae, Ctx,
                             ICP_GPA_REL, 0, gprA0);
         halAe_PutRelDataReg(handle, (unsigned char)Ae, Ctx,
                             ICP_GPB_REL, 0, gprB0);
         halAe_PutRelDataReg(handle, (unsigned char)Ae, Ctx,
                             ICP_GPA_REL, 1, gprA1);
         halAe_PutRelDataReg(handle, (unsigned char)Ae, Ctx,
                             ICP_GPB_REL, 1, gprB1);
         halAe_PutRelDataReg(handle, (unsigned char)Ae, Ctx,
                             ICP_SR_WR_REL, 0, sw0);
         halAe_PutRelDataReg(handle, (unsigned char)Ae, Ctx,
                             ICP_SR_WR_REL, 1, sw1);
    } while(0);
    
    for (Qat = 0; Qat < maxNumQats; Qat ++)
    {
        // power off SHRam if SHRam started out in power-off state
        if (!(QatMask & (1<<Qat)))
        {
            continue;
        }
        if (POWER_OFF == ((SHRamState[Qat] & START_MASK) >> START_BITPOS))
        {
            halMem_DisableQat(handle, Qat, QAT_SHRAM_TYPE);
        }   
    }

    if (dram)
    {
      osalMemFree(dram);
    }
    if (SHRamState)
    {
      osalMemFree(SHRamState);
    }
    halAe_ContinuousDramFree (handle, &dram_desc);

    return Status;

}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Initialize the Specified QAT Memory
 *      
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      No
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 * 
 * @param handle - IN A pointer to internal data structure which contains 
 *                           useful info, e.g. blocks' base address
 * @param QatMemType - IN The type of QAT memory to be cleared
 *
 * @retval HALMEM_SUCCESS                   Operation was successful
 * @retval HALMEM_INVALID_PARAMETER  Invalid parameters
 * @retval HALMEM_DEVICE_ERROR          Operation failed
 * 
 * 
 *****************************************************************************/
int
halMem_InitMemory(icp_firml_handle_t *handle,
                       QAT_MEM_TYPE QatMemType)
{
    int Status = HALMEM_SUCCESS;

    if (handle == NULL)
    {
        ERRINFO(("Invalid parameter\n"));   
        return HALMEM_INVALID_PARAMETER;
    }

    if ((handle->sysMemInfo.deviceId != ICP_ACCELCOMP_PCIE_DEVICE_ID_AC) &&
          (handle->sysMemInfo.deviceId != ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC) &&
          (handle->sysMemInfo.deviceId != ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC))
    /*if ((handle->sysMemInfo.deviceId != ICP_ACCELCOMP_PCIE_DEVICE_ID_AC) &&
      (handle->sysMemInfo.deviceId != ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC) &&
      (handle->sysMemInfo.deviceId != ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC) &&
      (handle->sysMemInfo.deviceId != ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC))*/
    {
        ERRINFO(("Platform is not supported\n"));   
        return HALMEM_DEVICE_ERROR;
    }

    if (handle->sysMemInfo.qatMask== 0)
    {
        ERRINFO(("No fused-in QAT\n"));   
        return HALMEM_SUCCESS;
    }

    //
    // Initialize QAT system
    //
    //halAe_Reset(handle, 0, 0);
    halAe_Reset(handle, 0x1, 0x1);
    //Status = halAe_ClrReset(handle, 0);
    Status = halAe_ClrReset(handle, 0x1);
    if (HALMEM_ERROR (Status))
    {
        ERRINFO(("Fail to ClrReset AE\n"));
        return HALMEM_DEVICE_ERROR;
    }
        
    switch(QatMemType)
    {
    case SHRAM_TYPE:
        // Only need to scrub shared ram on ACCELCOMP, no need to do this 
        // on ACCELCOMP_C since HW reset could scrub shared ram on ACCELCOMP_C
        if (handle->sysMemInfo.deviceId != 
            ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC)
        {
            Status = halMem_ScrubSharedRam(handle);
            if (HALMEM_ERROR (Status))
            {
                ERRINFO(("Fail to ScrubSHRam\n"));
                return HALMEM_DEVICE_ERROR;
            }
        }
        break;
    case MMP_TYPE:
        Status = halMem_ScrubMMPMemory(handle);
        if (HALMEM_ERROR (Status))
        {
            ERRINFO(("Fail to ScrubMmpMemory\n"));
            return HALMEM_DEVICE_ERROR;
        }
        break;
    case ALL_TYPE:
        // Only need to scrub shared ram on ACCELCOMP, no need to do this 
        // on ACCELCOMP_C since HW reset could scrub shared ram on ACCELCOMP_C
        if (handle->sysMemInfo.deviceId != 
            ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC)
        {
            Status = halMem_ScrubSharedRam(handle);
            if (HALMEM_ERROR (Status))
            {
                ERRINFO(("Fail to ScrubSHRam\n"));
                return HALMEM_DEVICE_ERROR;
            }
        }
        
        Status = halMem_ScrubMMPMemory(handle);
        if (HALMEM_ERROR (Status))
        {
            ERRINFO(("Fail to ScrubMmpMemory\n"));
            return HALMEM_DEVICE_ERROR;
        }
            
        break;
    default:
        ERRINFO(("Input memory type is not supported \n"));
        return HALMEM_INVALID_PARAMETER;           
    }

    return HALMEM_SUCCESS;

}


