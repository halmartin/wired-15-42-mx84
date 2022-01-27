/**
 **************************************************************************
 * @file halAeHw.c
 *
 * @description
 *      This file provides Implementation of Ucode AE Library
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
 *  version: QAT1.5.L.1.11.0-36
 *
 **************************************************************************/ 

#include "core_io.h"
#include "halAeApi.h"

void halAe_ExecuteCycles(unsigned int cycles);
void halAe_WaitAeLocalCsrReady(icp_firml_handle_t *handle,
                               unsigned char ae);
int waitNumCycles(icp_firml_handle_t *handle,
                  unsigned char ae, 
                  unsigned int cycles, 
                  int chkInactive);
int getRelDataReg(icp_firml_handle_t *handle,
                  unsigned char ae, 
                  unsigned char ctx, 
                  icp_RegType_T regType,
                  unsigned short regNum, 
                  unsigned int *data);
int getRelNNReg(icp_firml_handle_t *handle,
                unsigned char ae, 
                unsigned char ctx, 
                icp_RegType_T regType,
                unsigned short regNum,
                unsigned int *data);
int putRelDataReg(icp_firml_handle_t *handle,
                  unsigned char ae, 
                  unsigned char ctx, 
                  icp_RegType_T regType,
                  unsigned short regNum, 
                  unsigned int data);
int putRelWrXfer(icp_firml_handle_t *handle,
                 unsigned char ae, 
                 unsigned char ctx, 
                 icp_RegType_T regType,
                 unsigned short regNum, 
                 unsigned int data);
int putRelNN(icp_firml_handle_t *handle,
             unsigned char ae, 
             unsigned char ctx, 
             unsigned short nnNum,
             unsigned int value);
int getCtxIndrCsr(icp_firml_handle_t *handle,
                  unsigned char ae, 
                  unsigned char ctx, 
                  unsigned int aeCsr, 
                  unsigned int *csrVal);
int halAe_intrSupported(void);

int halAe_PutLM_Common(icp_firml_handle_t *handle,
                       unsigned char ae, 
                       unsigned short lmAddr, 
                       unsigned int value);
int halAe_GetLM_Common(icp_firml_handle_t *handle,
                       unsigned char ae, 
                       unsigned short lmAddr, 
                       unsigned int *value);
int halAe_PutSharedRam_Common(icp_firml_handle_t *handle,
                              unsigned char ae,
                              unsigned char qat, 
                              unsigned int addr, 
                              unsigned int value);
int halAe_GetSharedRam_Common(icp_firml_handle_t *handle,
                              unsigned char ae,
                              unsigned char qat, 
                              unsigned int addr, 
                              unsigned int *value);
int getRelDataReg_Common(icp_firml_handle_t *handle, 
                         unsigned char ae, 
                         unsigned char ctx, 
                         icp_RegType_T regType,
                         unsigned short regNum, 
                         unsigned int *data);
int getRelNNReg_Common(icp_firml_handle_t *handle,
                       unsigned char ae, 
                       unsigned char ctx, 
                       icp_RegType_T regType,
                       unsigned short regNum,
                       unsigned int *data);
int putRelDataReg_Common(icp_firml_handle_t *handle,
                         unsigned char ae, 
                         unsigned char ctx, 
                         icp_RegType_T regType,
                         unsigned short regNum, 
                         unsigned int data);
int putRelWrXfer_Common(icp_firml_handle_t *handle,
                        unsigned char ae, 
                        unsigned char ctx, 
                        icp_RegType_T regType,
                        unsigned short regNum, 
                        unsigned int data);
int putRelNN_Common(icp_firml_handle_t *handle,
                    unsigned char ae, 
                    unsigned char ctx, 
                    unsigned short nnNum,
                    unsigned int value);
int halAe_GetRelDataReg_Common(icp_firml_handle_t *handle,
                               unsigned char ae, 
                               unsigned char ctx, 
                               icp_RegType_T regType,
                               unsigned short regNum, 
                               unsigned int *regData);
int halAe_PutRelDataReg_Common(icp_firml_handle_t *handle,
                               unsigned char ae, 
                               unsigned char ctx, 
                               icp_RegType_T regType,
                               unsigned short regNum, 
                               unsigned int regData);
int halAe_GetAbsDataReg_Common(icp_firml_handle_t *handle,
                               unsigned char ae, 
                               icp_RegType_T regType,
                               unsigned short absRegNum, 
                               unsigned int *regData);
int halAe_PutAbsDataReg_Common(icp_firml_handle_t *handle,
                               unsigned char ae, 
                               icp_RegType_T regType,
                               unsigned short absRegNum, 
                               unsigned int regData);
int getCtxIndrCsr_Common(icp_firml_handle_t *handle,
                         unsigned char ae, 
                         unsigned char ctx, 
                         unsigned int aeCsr, 
                         unsigned int *csrVal);
int halAe_getMMScheme(icp_firml_handle_t *handle);
int halAe_clearGPRs_Common(icp_firml_handle_t *handle, 
                unsigned int aeMask);
int halAe_BatchPutLM_Common(icp_firml_handle_t *handle, unsigned char ae,
            batch_init_t *lm_init_header);

#define SHRAM_INIT_CYCLES 2060
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     
 *
 * @param 
 * @param 
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
void halAe_ExecuteCycles(unsigned int cycles)
{
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     
 *
 * @param ae - IN
 * @param 
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
/* once fixing HSD#2783356 - need delay between AE local CSR transaction */
void halAe_WaitAeLocalCsrReady(icp_firml_handle_t *handle,
                               unsigned char ae)
{
    /* using dummy reading and writing to improve the performance */
    /* read PROFILE_COUNT register as dummy read */
    GET_AE_CSR(handle, ae, PROFILE_COUNT);
   /* write read only register ALU_OUT as dummy write */
    SET_AE_CSR(handle, ae, ALU_OUT, 0x0);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     
 *
 * @param ae - IN
 * @param 
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
void halAe_WaitShramReady(icp_firml_handle_t *handle,
                               unsigned int aeMask)
{
    unsigned int ae;
    for (ae = 0; ae < MAX_AE; ae++)
    {
        if(aeMask & (1<<ae))
        {
            break;
        }
    }
    // if aeMask=0, this function will not be called.
    waitNumCycles(handle, (unsigned char)ae, SHRAM_INIT_CYCLES, 0);
    return;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Get Memory Map Scheme
 *
 * @param handle - IN 
 *
 * @retval  0
 * 
 * 
 *****************************************************************************/
int halAe_getMMScheme(icp_firml_handle_t *handle)
{
    if(handle->sysMemInfo.deviceId == ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC)
    { 
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word value to the AE's LM location specified by 
 *               the lmAddr word-address. It's unsafe to call this function while 
 *               the AE is enabled.
 *
 * @param ae - IN
 * @param lmAddr - IN
 * @param value - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_PutLM(icp_firml_handle_t *handle,
            unsigned char ae, 
            unsigned short lmAddr, 
            unsigned int value)
{
    return halAe_PutLM_Common(handle, ae, lmAddr, value);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword value from the specified accelaration engine local memory 
 *      location specified by the lmAddr word address.
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param lmAddr - IN Local memory word address
 * @param value - IN A pointer to the location of location of the longword 
 *                read from the specified register
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_GetLM(icp_firml_handle_t *handle,
            unsigned char ae, 
            unsigned short lmAddr, 
            unsigned int *value)
{
    return halAe_GetLM_Common(handle, ae, lmAddr, value);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a long-word value to the SSU Share Ram location. It's unsafe to 
 *      call this function while the AE is enabled.
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param addr - IN Specified shared ram address index
 * @param value - IN A data to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/
int 
halAe_PutSharedRam(icp_firml_handle_t *handle,
                   unsigned char ae,
                   unsigned char qat, 
                   unsigned int addr, 
                   unsigned int value)
{
    return halAe_PutSharedRam_Common(handle, ae, qat, addr, value);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a long-word value from the SSU Share Ram location. It's unsafe to 
 *      call this function while the AE is enabled.
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param addr - IN Specified shared ram address index
 * @param value - OUT A point to the location of data to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/
int 
halAe_GetSharedRam(icp_firml_handle_t *handle,
                   unsigned char ae, 
                   unsigned char qat,
                   unsigned int addr, 
                   unsigned int *value)
{
    return halAe_GetSharedRam_Common(handle, ae, qat, addr, value);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a long-word from a GPR, read-xfer, and LM index registers.
 *               It's unsafe to called this function while the AE is enabled.
 *               Type is one of: ICP_GPA_REL, ICP_GPB_REL, ICP_SR_REL, 
 *                               ICP_SR_RD_REL, ICP_DR_REL, ICP_DR_RD_REL, 
 *                               ICP_LMEM0, ICP_LMEM1.
 *               Ctx = 0-7/0-6 (even).
 *
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum- IN
 * @param data - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG, HALAE_BADLIB, HALAE_RESET
 * 
 * 
 *****************************************************************************/
int 
getRelDataReg(icp_firml_handle_t *handle,
              unsigned char ae, 
              unsigned char ctx, 
              icp_RegType_T regType,
              unsigned short regNum, 
              unsigned int *data)
{
    return getRelDataReg_Common(handle, ae, ctx, regType, regNum, data);
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a long-word from a next-neigh registers.
 *               It's unsafe to called this function while the AE is enabled.
 *               Type is ICP_NEIGH_REL.
 *               Ctx = 0-7/0-6 (even).
 *
 * @param 
 * @param 
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG, HALAE_BADLIB, HALAE_RESET
 * 
 * 
 *****************************************************************************/
int 
getRelNNReg(icp_firml_handle_t *handle,
            unsigned char ae, 
            unsigned char ctx, 
            icp_RegType_T regType,
            unsigned short regNum, 
            unsigned int *data)
{
    return getRelNNReg_Common(handle, ae, ctx, regType, regNum, data);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word to a GPR or LM index register. This logic 
 *               cannot work for neighbor or transfer registers.
 *               The AE must be disabled prior to calling this function.
 *
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum- IN
 * @param data - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_ENABLED, 
 *               HALAE_FAIL, HALAE_RESET
 *               Type is one of: ICP_GPA_REL, ICP_GPB_REL, ICP_LMEM0, ICP_LMEM1.
 *               Ctx = 0-7/0, 2, 4, 6.
 * 
 * 
 *****************************************************************************/
int 
putRelDataReg(icp_firml_handle_t *handle,
              unsigned char ae, 
              unsigned char ctx, 
              icp_RegType_T regType, 
              unsigned short regNum, 
              unsigned int data)
{
    return putRelDataReg_Common(handle, ae, ctx, regType, regNum, data);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word to a relative write-xfer reg.  This logic
 *               also supports neighbor registers.
 *               The AE must be disabled prior to calling this function.
 *
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum- IN
 * @param data - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_ENABLED, HALAE_FAIL,
 *               HALAE_RESET
 * 
 * 
 *****************************************************************************/
int 
putRelWrXfer(icp_firml_handle_t *handle,
             unsigned char ae, 
             unsigned char ctx, 
             icp_RegType_T regType,
             unsigned short regNum, 
             unsigned int data)
{
    return putRelWrXfer_Common(handle, ae, ctx, regType, regNum, data);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word value to the AE's NN register. The AE must be
 *               disabled prior to calling this function.
 *
 * @param ae - IN
 * @param ctx - IN
 * @param nnNum - IN
 * @param value- IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
int 
putRelNN(icp_firml_handle_t *handle,
         unsigned char ae, 
         unsigned char ctx, 
         unsigned short nnNum,
         unsigned int value)
{
    return putRelNN_Common(handle, ae, ctx, nnNum, value);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword from the relative accelaration engine general purpose register
 *      specified by the ae, ctx, regType, and regNum parameters. The 
 *      accelaration engine must be disabled before calling this function.
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the context
 * @param regType - IN Specifies the type of register to read from. This must 
 *                  be one of {ICP_GPA_REL,ICP_GPB_REL, ICP_DR_RD_REL, 
 *                  ICP_SR_RD_REL, ICP_DR_WR_REL,
 *                  ICP_SR_WR_REL, ICP_NEIGH_REL}.
 * @param regNum - IN Specifies the register number to read from. The register 
 *                 number is relative to the context. Therefore, this number 
 *                 is in the range of 0 through max - 1, where max is the 
 *                 maximum number of context-relative registers for the type 
 *                 specified by regType argument and the context mode
 *
 * @param regData - OUT A pointer to the location of the data to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/
int 
halAe_GetRelDataReg(icp_firml_handle_t *handle,
                    unsigned char ae, 
                    unsigned char ctx, 
                    icp_RegType_T regType,
                    unsigned short regNum, 
                    unsigned int *regData)
{
    return halAe_GetRelDataReg_Common(handle, ae, 
                                      ctx, regType, regNum, regData);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword to the relative accelaration engine general purpose register
 *      specified by the ae, ctx, regType, and regNum parameters. The 
 *      accelaration engine must be disabled before calling this function.
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param ctx - IN Specifies the context
 * @param regType - IN Specifies the type of register to read from. This must 
 *                  be one of {ICP_GPA_REL,ICP_GPB_REL, ICP_DR_RD_REL, 
 *                  ICP_SR_RD_REL, ICP_DR_WR_REL,
 *                  ICP_SR_WR_REL, ICP_NEIGH_REL}.
 * @param regNum - IN Specifies the register number to read from. The register 
 *                 number is relative to the context. Therefore, this number 
 *                 is in the range of 0 through max - 1, where max is the 
 *                 maximum number of context-relative registers for the type 
 *                 specified by regType argument and the context mode
 * @param regData - IN The longword to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/
int 
halAe_PutRelDataReg(icp_firml_handle_t *handle,
                    unsigned char ae, 
                    unsigned char ctx, 
                    icp_RegType_T regType,
                    unsigned short regNum, 
                    unsigned int regData)
{
    return halAe_PutRelDataReg_Common(handle, ae, 
                                      ctx, regType, regNum, regData);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword from the accelaration engine absolute general purpose register
 *      specified by the ae, ctx, regType, and regNum parameters. The 
 *      accelaration engine must be disabled before calling this function.
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param regType - IN Specifies the type of register to read from. This must 
 *                  be one of {ICP_GPA_ABS, ICP_GPB_ABS, ICP_DR_RD_ABS, 
 *                  ICP_SR_RD_ABS, ICP_DR_WR_ABS,
 *                  ICP_SR_WR_ABS, ICP_NEIGH_ABS}.
 * @param absRegNum - IN Specifies the register number to read from
 * @param regData - OUT A pointer to the location of the data to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/
int 
halAe_GetAbsDataReg(icp_firml_handle_t *handle,
                    unsigned char ae, 
                    icp_RegType_T regType,
                    unsigned short absRegNum, 
                    unsigned int *regData)
{
    return halAe_GetAbsDataReg_Common(handle, ae, regType, absRegNum, regData);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword to the accelaration engine absolute general purpose register
 *      specified by the ae, regType, and regNum parameters. The 
 *      accelaration engine must be disabled before calling this function.
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
 * @param ae - IN Specifies the accelaration engine of interest
 * @param regType - IN Specifies the type of register to read from. This must 
 *                  be one of {ICP_GPA_ABS, ICP_GPB_ABS, ICP_DR_RD_ABS, 
 *                  ICP_SR_RD_ABS, ICP_DR_WR_ABS,
 *                  ICP_SR_WR_ABS, ICP_NEIGH_ABS}.
 * @param absRegNum - IN Specifies the register number to write
 * @param regData - IN The longword to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_ENABLED Acceleration engine enabled
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/
int 
halAe_PutAbsDataReg(icp_firml_handle_t *handle,
                    unsigned char ae, 
                    icp_RegType_T regType,
                    unsigned short absRegNum, 
                    unsigned int regData)
{
    return halAe_PutAbsDataReg_Common(handle, ae, regType, absRegNum, regData);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a context indirect-csr.
 *
 * @param ae - IN
 * @param ctx - IN
 * @param aeCsr - IN
 * @param csrVal - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
int 
getCtxIndrCsr(icp_firml_handle_t *handle,
              unsigned char ae, 
              unsigned char ctx, 
              unsigned int aeCsr, 
              unsigned int *csrVal)
{
    return getCtxIndrCsr_Common(handle, ae, ctx, aeCsr, csrVal);
}
 
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Return if interrupt is supported.
 *
 *
 * @retval   
 * 
 * 
 *****************************************************************************/
int 
halAe_intrSupported(void)
{
    return 1;
}

/*
 *****************************************************************************
 * @ingroup icp_hal
 *
 * @description
 *    Set all GPRs, transfer regs., next neigh reg., and local-memory
 *          to zero.
 *    This function is to optimize the clear operation into one Microcode
 * @param aeMask - IN
 *
 * @retval   HALAE_SUCCESS, HALAE_FAIL
 *
 *
 *****************************************************************************/
int 
halAe_clearGPRs(icp_firml_handle_t *handle, 
                unsigned int aeMask)
{
    unsigned int stat = HALAE_SUCCESS; 
    stat = halAe_clearGPRs_Common(handle, aeMask);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 *
 * @description
 *   Batch writes to the specified accelaration engine~Rs local memory
 *      location specified by a link of the lmaddr word address.
 *
 * @param ae - IN
 * @param lmAddr - IN
 * @param value - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_AEACTIVE
 *
 *
 ****************************************************************************/
int
halAe_BatchPutLM(icp_firml_handle_t *handle, unsigned char ae,
            batch_init_t *lm_init_header)
{
     unsigned int stat = HALAE_SUCCESS;
     stat = halAe_BatchPutLM_Common(handle, ae, lm_init_header);
     return (stat);
}


