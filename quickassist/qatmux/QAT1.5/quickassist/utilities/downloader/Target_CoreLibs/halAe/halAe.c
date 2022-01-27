/**
 **************************************************************************
 * @file halAe.c
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

#include "halAe_platform.h"
#include "core_io.h"

#include "hal_ae.h"
#include "halAeApi.h"
#include "halOS.h"
#include "halAeDrv.h"
#include "halAe.h"
#include "uof.h"
uword_T inst_4b[] =
{
    0x0F0400C0000ull,  /* .0 immed_w0[l0000!indx, 0] */
    0x0F4400C0000ull,  /* .1 immed_w1[l0000!indx, 0] */
    0x0F040000300ull,  /* .2 immed_w0[l0000!myvalue, 0x0] */
    0x0F440000300ull,  /* .3 immed_w1[l0000!myvalue, 0x0] */
    0x0FC066C0000ull,  /* .4 local_csr_wr[active_lm_addr_0, 
                                    l0000!indx]; put indx to lm_addr */
    0x0F0000C0300ull,  /* .5 nop */
    0x0F0000C0300ull,  /* .6 nop */
    0x0F0000C0300ull,  /* .7 nop */
    0x0A021000000ull,  /* .8 alu[*l$index0++, --, b, l0000!myvalue] */
};

uword_T inst_1b[] =
{
    0x0F0400C0000ull,  /* .0 immed_w0[l0000!indx, 0] */
    0x0F4400C0000ull,  /* .1 immed_w1[l0000!indx, 0] */
    0x0F040000300ull,  /* .2 immed_w0[l0000!myvalue, 0x0] */
    0x0F440000300ull,  /* .3 immed_w1[l0000!myvalue, 0x0] */
    0x0FC066C0000ull,  /* .4 local_csr_wr[active_lm_addr_0, 
                                   l0000!indx]; put indx to lm_addr */
    0x0F0000C0300ull,  /* .5 nop */
    0x0F0000C0300ull,  /* .6 nop */
    0x0F0000C0300ull,  /* .7 nop */
    0x0A000180000ull,  /* .8 alu[l0000!val, --, b, *l$index0] */
    0x09080000200ull,  /* .9 alu_shf[l0000!myvalue, --, b, 
                                      l0000!myvalue,  <<24 ] */
    0x08180280201ull,  /* .10 alu_shf[l0000!val1, --, b, l0000!val,  <<8 ] */
    0x08080280102ull,  /* .11 alu_shf[l0000!val1, --, b, l0000!val1 , >>8 ] */
    0x0BA00100002ull,  /* .12 alu[l0000!val2, l0000!val1, or, l0000!myvalue] */

};

uword_T inst_2b[] =
{
    0x0F0400C0000ull,  /* .0 immed_w0[l0000!indx, 0] */
    0x0F4400C0000ull,  /* .1 immed_w1[l0000!indx, 0] */
    0x0F040000300ull,  /* .2 immed_w0[l0000!myvalue, 0x0] */
    0x0F440000300ull,  /* .3 immed_w1[l0000!myvalue, 0x0] */
    0x0FC066C0000ull,  /* .4 local_csr_wr[active_lm_addr_0, 
                                   l0000!indx]; put indx to lm_addr */
    0x0F0000C0300ull,  /* .5 nop */
    0x0F0000C0300ull,  /* .6 nop */
    0x0F0000C0300ull,  /* .7 nop */
    0x0A000180000ull,  /* .8 alu[l0000!val, --, b, *l$index0] */
    0x09100000200ull,  /* .9 alu_shf[l0000!myvalue, --, b, 
                                          l0000!myvalue,  <<16 ] */
    0x08100280201ull,  /* .10 alu_shf[l0000!val1, --, b, l0000!val,  <<16 ] */
    0x08100280102ull,  /* .11 alu_shf[l0000!val1, --, b, l0000!val1 , >>16 ] */
    0x0BA00100002ull,  /* .12 alu[l0000!val2, l0000!val1, or, l0000!myvalue] */
};

uword_T inst_3b[] =
{
    0x0F0400C0000ull,  /* .0 immed_w0[l0000!indx, 0] */
    0x0F4400C0000ull,  /* .1 immed_w1[l0000!indx, 0] */
    0x0F040000300ull,  /* .2 immed_w0[l0000!myvalue, 0x0] */
    0x0F440000300ull,  /* .3 immed_w1[l0000!myvalue, 0x0] */
    0x0FC066C0000ull,  /* .4 local_csr_wr[active_lm_addr_0, 
                                   l0000!indx]; put indx to lm_addr */
    0x0F0000C0300ull,  /* .5 nop */
    0x0F0000C0300ull,  /* .6 nop */
    0x0F0000C0300ull,  /* .7 nop */
    0x0A000180000ull,  /* .8 alu[l0000!val, --, b, *l$index0] */
    0x09180000200ull,  /* .9 alu_shf[l0000!myvalue, --, 
                                    b, l0000!myvalue,  <<8 ] */
    0x08080280201ull,  /* .10 alu_shf[l0000!val1, --, b, l0000!val,  <<24 ] */
    0x08180280102ull,  /* .11 alu_shf[l0000!val1, --, b, l0000!val1 , >>24 ] */
    0x0BA00100002ull,  /* .12 alu[l0000!val2, l0000!val1, or, l0000!myvalue] */
};



/**
 * @description uclo callback
*/
typedef struct hal_uclocallframe_s {
    HalAe_UcloCall_T uclo_callback;      /* callback function pointer */
    void*            uclo_callback_data; /* callback data */
} hal_uclocallframe_t;

static HalAePageChangeCallback_T new_page_callback = NULL;
static void*                     new_page_callback_data;
static hal_uclocallframe_t ucloCallbacks[MAX_AE];

const unsigned int TmpUaddr = 0;

int halAe_GetExecutableAe(icp_firml_handle_t *handle, unsigned char *ae);
int halAe_getMMScheme(icp_firml_handle_t *handle);
static int disableCtx(icp_firml_handle_t *handle,
                      unsigned char ae,
                      unsigned int ctxMask);
static int enableCtx(icp_firml_handle_t *handle,
                     unsigned char ae, 
                     unsigned int ctxMask);
int isAeEnabled(icp_firml_handle_t *handle,
                unsigned char ae);
unsigned int isAeActive(icp_firml_handle_t *handle,
                        unsigned char ae);
int putAeCsr(icp_firml_handle_t *handle,
                    unsigned char ae, 
                    unsigned int csr, 
                    unsigned int value);
int getAeCsr(icp_firml_handle_t *handle,
                    unsigned char ae, 
                    unsigned int csr, 
                    unsigned int *value);
static int getCtxWakeupEvents(icp_firml_handle_t *handle,
                              unsigned char ae, 
                              unsigned char ctx, 
                              unsigned int *events);
static int putCtxWakeupEvents(icp_firml_handle_t *handle,
                              unsigned char ae, 
                              unsigned int ctxMask, 
                              unsigned int events);
static int getCtxSigEvents(icp_firml_handle_t *handle,
                           unsigned char ae, 
                           unsigned char ctx, 
                           unsigned int *events);
static int putCtxSigEvents(icp_firml_handle_t *handle,
                           unsigned char ae, 
                           unsigned int ctxMask, 
                           unsigned int events);
static int putCtxIndrCsr(icp_firml_handle_t *handle,
                         unsigned char ae, 
                         unsigned int ctxMask,
                         unsigned int aeCsr, 
                         unsigned int csrVal);
int getRelRdXfer(icp_firml_handle_t *handle,
                 unsigned char ae,
                 unsigned char ctx, 
                 icp_RegType_T regType,
                 unsigned short regNum, 
                 unsigned int *data);
int putRelRdXfer(icp_firml_handle_t *handle,
                 unsigned char ae, 
                 unsigned char ctx, 
                 icp_RegType_T regType,
                 unsigned short regNum, 
                 unsigned int val);
int getRelWrXfers(icp_firml_handle_t *handle,
                  unsigned char ae, 
              unsigned char ctx, 
              icp_RegType_T regType,
              unsigned short regNum, 
              unsigned int *data, 
              unsigned int count);
void halAe_clrResetStatus(icp_firml_handle_t *handle);
int waitNumCycles(icp_firml_handle_t *handle,
                  unsigned char ae, 
                  unsigned int cycles, 
                  int chkInactive);
int execMicroInst(icp_firml_handle_t *handle, 
                  unsigned char ae, 
                  unsigned char ctx, 
                  uword_T *microInst,
                  unsigned int numInst, 
                  int condCodeOff,
                  unsigned int maxCycles, 
                  unsigned int *endPC);
void halAe_setUofChecksum(icp_firml_handle_t *handle,
                          unsigned int uofChecksum);
unsigned int halAe_getUofChecksum(icp_firml_handle_t *handle);
int relToAbs(icp_firml_handle_t *handle,
             unsigned char ae, 
             unsigned char ctx, 
             unsigned short relRegNum,
             unsigned short *absRegNum);

void halAe_ExecuteCycles(icp_firml_handle_t *handle, unsigned int cycles);
void halAe_WaitAeLocalCsrReady(icp_firml_handle_t *handle,
                               unsigned char ae);
void halAe_WaitShramReady(icp_firml_handle_t *handle,
                               unsigned int aeMask);
int halAe_GetProdSetting(icp_firml_handle_t *handle);
int halAe_GetChipSetting(icp_firml_handle_t *handle);

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
unsigned short getReg10bitAddr(unsigned int type, 
                               unsigned short regNum);
uword_T setUwordECC(uword_T uword);
int putUwords(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned int uAddr, 
              unsigned int numWords, 
              uword_T *uWord);
int getUwords(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned int uAddr, 
              unsigned int numWords, 
              uword_T *uWord);
int halAe_MemSet(uint64 vaddr_dst, 
                 int c, 
                 int size);
int 
halAe_setUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int freeAddr, 
                       unsigned int freeSize);
int 
halAe_getUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int *freeAddr, 
                       unsigned int *freeSize);
void disableQatIntr(icp_firml_handle_t *handle);
int halAe_checkAes(icp_firml_handle_t *handle, unsigned int aeMask);
int halAe_enableAutoInitSram(icp_firml_handle_t *handle, unsigned int aeMask);
int halAe_clearGPRs(icp_firml_handle_t *handle, unsigned int aeMask);
static int concatMicroCode(uword_T *microInstArry, \
         unsigned int inst_num, unsigned int size, \
         unsigned int addr, unsigned int *value );

#define CLRRESET_TIMES 100
#define AE_EXEC_CYCLE  20

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determine parity of the number of bits.
 * 
 * @param word - IN 
 *
 * @retval 0 if even; 1 if odd.
 * 
 * 
 *****************************************************************************/
static unsigned int 
bitParity64(uint64 word)
{
   word ^= word >> VALUE_BIT_0;
   word ^= word >> VALUE_BIT_1;
   word ^= word >> VALUE_BIT_2;
   word ^= word >> VALUE_BIT_3;
   word ^= word >> VALUE_BIT_4;
   word ^= word >> VALUE_BIT_5;
   return ((unsigned int )(word & 1));
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determine the number of bits that are set in a long-word.
 * 
 * @param word - IN 
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
static int 
numBitsSet(unsigned int word)
{
   int nn = 0;

   if(word)
   {
      do
      {
         nn++;
      } while ((word &= word - 1));
   }
   return (nn);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Get an AE which can be used to run Micro Code for debugger purpose
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
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_AEACTIVE All AEs are active
 * 
 * 
 *****************************************************************************/
int 
halAe_GetExecutableAe(icp_firml_handle_t *handle, unsigned char *ae)
{
    unsigned int csrVal, shCtlStoreFlag;
    unsigned char aeNeigh;
    unsigned int ii;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    
    for (ii = 0; ii < MAX_AE; ii++)
    {
        if ((1 << ii) & handle->sysMemInfo.aeMask)
        {
            if (halAe_VerifyAe(handle, (unsigned char)ii) != HALAE_SUCCESS)
            {
                continue;
            }
            halAe_GetAeCsr(handle, (unsigned char)ii, AE_MISC_CONTROL, &csrVal);
            shCtlStoreFlag = csrVal & (0x1 << MMC_SHARE_CS_BITPOS);
            if(shCtlStoreFlag) 
            {
                halAe_GetSharedUstoreNeigh(handle, (unsigned char)ii, &aeNeigh);
                if(((1<<aeNeigh) & handle->sysMemInfo.aeMask)
                  && (halAe_IsAeEnabled(handle, aeNeigh) != HALAE_DISABLED))
                {
                    continue;
                }
            }
            if (halAe_IsAeEnabled(handle, (unsigned char)ii) == HALAE_DISABLED)
            {
                break;
            }
        }
    }
    if (ii == MAX_AE)
    {
        DBGINFO(("halAe_GetExecutableAe: All AEs are active\n"));
        return HALAE_AEACTIVE;
    }
    *ae = (unsigned char)ii;
    return (HALAE_SUCCESS);
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set a mask of the contexts that are loaded.
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
 * @param ctxMask - IN The context or contexts to set to alive
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/
int 
halAe_SetLiveCtx(icp_firml_handle_t *handle, 
                 unsigned char ae, 
                 unsigned int ctxMask)
{
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    SET_LIVECTXMASK(handle, ae, ctxMask);
    SPIN_UNLOCK_AE(handle, ae);
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get a mask of the contexts that are loaded.
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
 * @param ctxMask - OUT A pointer referencec to the alive context or contexts
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/
int 
halAe_GetLiveCtx(icp_firml_handle_t *handle, 
                 unsigned char ae, 
                 unsigned int *ctxMask)
{
    VERIFY_ARGPTR(handle);
    VERIFY_ARGPTR(ctxMask);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    *ctxMask = GET_LIVECTXMASK(handle, ae);
    SPIN_UNLOCK_AE(handle, ae);
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set uof file checksum
 * 
 * @param uofChecksum - IN 
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
void 
halAe_setUofChecksum(icp_firml_handle_t *handle, 
                     unsigned int uofChecksum)
{
    handle->halHandle->UofChecksum = uofChecksum;
    return;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get uof file checksum
 * 
 * @param 
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
unsigned int 
halAe_getUofChecksum(icp_firml_handle_t *handle)
{
    return (handle->halHandle->UofChecksum);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determine whether specified accelEngine is enabled, or running
 * 
 * @param ae - IN
 *
 * @retval 1 if active or 0 for inactive
 * 
 * 
 *****************************************************************************/
unsigned int 
isAeActive(icp_firml_handle_t *handle,
           unsigned char ae)
{
    unsigned int csr;
    
    if(isAeEnabled(handle, ae) != HALAE_DISABLED) 
    {
       return (1); 
    }   
    getAeCsr(handle, ae, ACTIVE_CTX_STATUS, &csr);
    return ((csr & (1 << ACS_ABO_BITPOS)));
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Wait for specified number of cycles
 *
 * @param ae - IN
 * @param cycles - IN
 * @param chkInactive - IN
 *
 * @retval returns 1 if actually waited for specified number of cycles
 * 
 * 
 *****************************************************************************/
int 
waitNumCycles(icp_firml_handle_t *handle,
              unsigned char ae, 
              unsigned int cycles, 
              int chkInactive)
{
    int PreCnt = 0;
    int CurCnt = 0;
    int ElapsedCycles = 0;
    unsigned int csr;
    unsigned int rc = HALAE_SUCCESS;
    int times = MAX_RETRY_TIMES;

    getAeCsr(handle, ae, PROFILE_COUNT, (unsigned int *)&CurCnt);
    PreCnt = CurCnt;
    PreCnt &= 0xffff;
    do {
        halAe_ExecuteCycles(handle, cycles);        

        getAeCsr(handle, ae, PROFILE_COUNT, (unsigned int *)&CurCnt); 
        CurCnt &= 0xffff;
    
        /* With this logic, there is still a very slight risk     
         that due to very slow execution or OS-related delays   
         on the IA side (greater than about 0x10000 AE cycles   
         elapsed), successive reads of the PROFILE_COUNT will  
        fail the test to terminate the loop, but should.      
         The logic to handle the slight risk could be:         
         If the number of AE cycles between successive reads   
         of PROFILE_COUNT is just over 65535, then the loop    
         should immediately terminate.                          
         Above logic could not work under whole chip simulation
         since the clock frequency domains are not the same.   
         And considering the low probability, above logic will  
         not be added.                                          */
    
        /* Calculate the elapsed AE cycles */
        ElapsedCycles = CurCnt - PreCnt;
        /* timeout mechanism to avoid SW hang caused by reset */
        if(ElapsedCycles == 0)
        {
            times --;
            DBGINFO(("times=%d, ElapsedCycles=0\n", times));
        }
        if(times <= 0)
        {
            PRINTF("waitNumCycles timeout!!\n");
            return -1;
        }

        if(ElapsedCycles < 0) 
        {
            ElapsedCycles += 0x10000;
        }
        /* make sure CTX entering executing state after CTX_ENABLE high 
         * before chkInactive */
        if ((ElapsedCycles >= NUM_CYCLES_FROM_READY2EXE) && chkInactive) 
        {
            rc = getAeCsr(handle, ae, ACTIVE_CTX_STATUS, &csr);
            if(rc == HALAE_SUCCESS)
            {
                if ((csr & (1 << ACS_ABO_BITPOS)) == 0) 
                {
                    return 0;
                }    
            }    
        }
    } while (((int)cycles)>ElapsedCycles);

    /* Before returning a timeout, check active context status once again   
     to make sure the context did not terminate between the last read of  
     ACTIVE_CTX_STATUS and the read of PROFILE_COUNT. A large OS-related  
     delay between the last read of ACTIVE_CTX_STATUS and PROFILE_COUNT   
     will result in a non-zero (timeout) return when the context actually 
     terminated on its own.                                               */
    if (chkInactive) 
    {
        rc = getAeCsr(handle, ae, ACTIVE_CTX_STATUS, &csr);
        if(rc == HALAE_SUCCESS)
        {
            if ((csr & (1 << ACS_ABO_BITPOS)) == 0) 
            {
                return 0;
            }    
        }    
    }

    return 1;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the neighbor of a shareable-ustore pair.
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
 * @param aeNeigh - OUT A pointer to neighboring accelaration engine
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_GetSharedUstoreNeigh(icp_firml_handle_t *handle, 
                           unsigned char ae, 
                           unsigned char *aeNeigh)
{
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);    
    VERIFY_AE(handle, ae);

    if(!aeNeigh) 
    {
       ERRINFO(("aeNeigh argument is NULL\n ")); 
       return (HALAE_BADARG);
    }    
    if(ae & 0x1) 
    {
       *aeNeigh = ae-1;
    }   
    else 
    {
       *aeNeigh = ae+1;
    }   
    
    return (HALAE_SUCCESS);    
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determines if the specified accelaration engine is valid, whether it is 
 *      initialized, or whether it is in or out of reset.
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
 *
 * @retval HALAE_UNINIT A bad debug library is associated with the specified
 *                      accelaration engine the library is not initialized
 * @retval HALAE_CLR_RST The accelaration engine is not in a reset state
 * @retval HALAE_RST The accelaration engine is in reset state
 * 
 * 
 *****************************************************************************/
int 
halAe_GetAeState(icp_firml_handle_t *handle, 
                 unsigned char ae)
{
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    /* verify AE */
    if(((ae) > handle->halHandle->AeMaxNum) || ((ae) >= BITS_IN_LONGWORD) ||
      (handle->halHandle->AeBadMask & setMasks[(ae)]))
    {
        return (HALAE_BADARG);
    }
    return (AE(handle, ae).state);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Checks that the value specified by the ae parameter is a valid 
 *      accelaration engine number. The library must be initialized before calling 
 *      this function.
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
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_VerifyAe(icp_firml_handle_t *handle, 
               unsigned char ae)
{
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Checks that the value specified by the ae parameter is a valid 
 *      accelaration engine number. The library must be initialized before calling 
 *      this function.
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
 * @param aeMask - IN Specifies the acceleration engines of interest
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int halAe_VerifyAeMask(icp_firml_handle_t *handle, 
                       unsigned int aeMask)
{
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    if(handle->halHandle->AeBadMask & aeMask) 
    {
       ERRINFO(("aeMask = 0x%x, is bad\n ",aeMask)); 
       return (HALAE_BADARG);
    }   
    return (HALAE_SUCCESS);
}
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the AE context mode (four or eight ctx).
 *
 * @param ae - IN
 *
 * @retval FOUR_CTX or EIGHT_CTX
 * 
 * 
 *****************************************************************************/
static int 
getAeCtxMode(icp_firml_handle_t *handle, 
             unsigned char ae)
{
    unsigned int ctx_enables;

      getAeCsr(handle, ae, CTX_ENABLES, &ctx_enables);
 
    if(CE_INUSE_CONTEXTS & ctx_enables) 
    {
        return FOUR_CTX;
    }    
    else 
    {
        return EIGHT_CTX;
    }    
    
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Gets the accelaration engine context mode to either four or eight
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
 *
 * @retval ctx mode FOUR_CTX or EIGHT_CTX
 * 
 * 
 *****************************************************************************/
int 
halAe_GetAeCtxMode(icp_firml_handle_t *handle, 
                   unsigned char ae)
{
    int ctx_mode =0;
    
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    ctx_mode = getAeCtxMode(handle, ae);
    SPIN_UNLOCK_AE(handle, ae);
    return (ctx_mode);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set the AE context mode (four or eight ctx).
 *
 * @param ae - IN
 * @param mode - IN
 *
 * @retval HALAE_UNINIT, HALAE_BADARG, HALAE_BADLIB
 * 
 * 
 *****************************************************************************/
static int 
putAeCtxMode(icp_firml_handle_t *handle, 
             unsigned char ae, 
             unsigned char mode)
{
    unsigned int csr, newCsr;

    getAeCsr(handle, ae, CTX_ENABLES, &csr);
    csr = IGNORE_W1C_MASK & csr;
    if(mode == FOUR_CTX) 
    {
        newCsr = SET_BIT(csr, CE_INUSE_CONTEXTS_BITPOS);
    }    
    else 
    {
        newCsr = CLR_BIT(csr, CE_INUSE_CONTEXTS_BITPOS);
    }    
    if(newCsr != csr) 
    {
       putAeCsr(handle, ae, CTX_ENABLES, newCsr);
    }    
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the accelaration engine context mode to either four or eight
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
 * @param mode - IN Specifies the context mode. This value must be one of {4, 8}
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_PutAeCtxMode(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned char mode)
{
    int stat = HALAE_SUCCESS;
    
    VERIFY_ARGPTR(handle);
    if((mode != FOUR_CTX) && (mode != EIGHT_CTX))
    {
        ERRINFO(("bad ctx mode=%d\n", mode));
        return (HALAE_BADARG);
    }

    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    stat = putAeCtxMode(handle, ae, mode);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the accelaration engine next-neighbor mode to either write to itself or to
 *      its neighbor.
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
 * @param mode - IN The next-neighbor mode to set. This value is one of:
 *                  1. zero - the next-neighbor mode is write to self
 *                  2. one - the next-neighbor mode is write to neighbor
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_PutAeNnMode(icp_firml_handle_t *handle, 
                  unsigned char ae, 
                  unsigned char mode)
{
    unsigned int csr, newCsr;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);

    SPIN_LOCK_AE(handle, ae);
    getAeCsr(handle, ae, CTX_ENABLES, &csr);
    csr = IGNORE_W1C_MASK & csr;
    if(mode) 
    {
       newCsr = SET_BIT(csr, CE_NN_MODE_BITPOS);
    }   
    else 
    {
       newCsr = CLR_BIT(csr, CE_NN_MODE_BITPOS);
    }   

    if(newCsr != csr) 
    {
        putAeCsr(handle, ae, CTX_ENABLES, newCsr);
    }    
    SPIN_UNLOCK_AE(handle, ae);
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set the whether localMem0/localMem1 access is relative or global.
 *               It's unsafe to call this function while the AE is enabled.
 *               lmType: ICP_LMEM0, ICP_LMEM1
 *               mode: 0=relative, 1=global
 * @param ae - IN
 * @param lmType - IN
 * @param mode - IN
 *
 * @retval HALAE_UNINIT, HALAE_BADARG, HALAE_BADLIB
 * 
 * 
 *****************************************************************************/
static int 
putAeLmMode(icp_firml_handle_t *handle, 
            unsigned char ae, 
            icp_RegType_T lmType, 
            unsigned char mode)
{
    unsigned int csr, newCsr;

    getAeCsr(handle, ae, CTX_ENABLES, &csr);
    csr = IGNORE_W1C_MASK & csr;
    switch(lmType)
    {
    case ICP_LMEM0:
        if(mode) 
        {
            newCsr = SET_BIT(csr, CE_LMADDR_0_GLOBAL_BITPOS);
        }    
        else 
        {
            newCsr = CLR_BIT(csr, CE_LMADDR_0_GLOBAL_BITPOS);
        }    
        break;   
    case ICP_LMEM1:
        if(mode) 
        {
            newCsr = SET_BIT(csr, CE_LMADDR_1_GLOBAL_BITPOS);
        }    
        else 
        {
            newCsr = CLR_BIT(csr, CE_LMADDR_1_GLOBAL_BITPOS);
        }    
        break;
    default: 
        ERRINFO((" lmType = 0x%x\n",lmType)); 
        return (HALAE_BADARG);
    }

    if(newCsr != csr) 
    {
        putAeCsr(handle, ae, CTX_ENABLES, newCsr);
    }    
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the local-memory mode to relative or global.
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
 * @param lmType - IN Specifies the local memory bank. 
                   This value is ICP_LMEM0 or ICP_LMEM1
 * @param mode - IN Specifies the local memory mode. This value is one of:
 *               1. zero - the memory mode is relative
 *               2. one - the memory mode is global
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_PutAeLmMode(icp_firml_handle_t *handle, 
                  unsigned char ae, 
                  icp_RegType_T lmType, 
                  unsigned char mode)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);

    SPIN_LOCK_AE(handle, ae);
    stat = putAeLmMode(handle, ae, lmType, mode);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set the AE shared ustore mode on/off.
 * @param ae - IN
 * @param mode - IN
 *
 * @retval  HALAE_SUCCESS
 * 
 * 
 *****************************************************************************/
static int 
putAeSharedCsMode(icp_firml_handle_t *handle, 
                  unsigned char ae, 
                  unsigned char mode)
{
    unsigned int csrVal, newCsrVal;

    getAeCsr(handle, ae, AE_MISC_CONTROL, &csrVal);
    if(mode == 1) 
    {
        newCsrVal = SET_BIT(csrVal, MMC_SHARE_CS_BITPOS);
    }    
    else 
    {
        newCsrVal = CLR_BIT(csrVal, MMC_SHARE_CS_BITPOS);
    }    

    if(newCsrVal != csrVal) 
    {
        putAeCsr(handle, ae, AE_MISC_CONTROL, newCsrVal);
    }
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set the shared ustore mode on or off for the specified AE and its 
 *      neighbor's.
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
 * @param mode - IN Specifies the mode to set
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * 
 * 
 *****************************************************************************/
int 
halAe_PutAeSharedCsMode(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned char mode)
{
    int stat = HALAE_SUCCESS;
    unsigned char aeNeigh;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);

    SPIN_LOCK(handle->halHandle->hal_GlobalLock);

    SPIN_LOCK_AE(handle, ae);
    stat = putAeSharedCsMode(handle, ae, mode);
    SPIN_UNLOCK_AE(handle, ae);

    if((handle->sysMemInfo.aeMask & (1<<aeNeigh))
        && (AE(handle, ae).state == HALAE_CLR_RST))
    {
        SPIN_LOCK_AE(handle, aeNeigh);
        stat = putAeSharedCsMode(handle, aeNeigh, mode);
        SPIN_UNLOCK_AE(handle, aeNeigh);
    }

    SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);

    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Get the AE' shared ustore mode on/off.
 *
 * @param ae - IN 
 * @param mode - OUT
 *
 * @retval HALAE_SUCCESS
 * 
 * 
 *****************************************************************************/
static int 
getAeSharedCsMode(icp_firml_handle_t *handle, 
                  unsigned char ae, 
                  unsigned char *mode)
{
    unsigned int csrVal;

    getAeCsr(handle, ae, AE_MISC_CONTROL, &csrVal);
    if(csrVal & (0x1 << MMC_SHARE_CS_BITPOS)) 
    {
       *mode = 1;
    }    
    else 
    {
       *mode = 0;
    }   

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determines if the specified accelaration engine is valid, whether it is 
 *      initialized, or whether it is in or out of reset.
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
 * @param mode - OUT A pointer to shared control store mode
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_GetAeSharedCsMode(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned char *mode)
{
    int stat = HALAE_SUCCESS;
    
    VERIFY_ARGPTR(handle);
    VERIFY_ARGPTR(mode);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);

    SPIN_LOCK_AE(handle, ae);
    stat = getAeSharedCsMode(handle, ae, mode);
    SPIN_UNLOCK_AE(handle, ae);

    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Get the 10-bit address of the specified register number.
 *
 * @param type - IN
 * @param regNum - IN 
 *
 * @retval  If the type ICP_LMEM0, or ICP_LMEM1 then regNum is ignored.
 * 
 * 
 *****************************************************************************/
unsigned short 
getReg10bitAddr(unsigned int type, 
                unsigned short regNum)
{
   unsigned short regAddr;
   switch(type)
   {
    case ICP_GPA_ABS:
    case ICP_GPB_ABS:
        regAddr = (regNum & 0x7f) | 0x80; 
        break;
    case ICP_GPA_REL:
    case ICP_GPB_REL:
        regAddr = regNum & 0x1f;     
        break;
    case ICP_SR_RD_REL:
    case ICP_SR_WR_REL:
    case ICP_SR_REL:
        regAddr = 0x180 | (regNum & 0x1f); 
        break;
    case ICP_SR_INDX: 
        regAddr = 0x140 | ((regNum & 0x3) << 1); 
        break;
    case ICP_DR_RD_REL:
    case ICP_DR_WR_REL:
    case ICP_DR_REL:
        regAddr = 0x1c0 | (regNum & 0x1f); 
        break;
    case ICP_DR_INDX: 
        regAddr = 0x100 | ((regNum & 0x3) << 1); 
        break;
    case ICP_NEIGH_INDX: 
        regAddr = 0x241 | ((regNum & 0x3) << 1); 
        break;
    case ICP_NEIGH_REL: 
        regAddr = 0x280 | (regNum & 0x1f); 
        break;
    case ICP_LMEM0: 
        regAddr = 0x200; 
        break;
    case ICP_LMEM1: 
        regAddr = 0x220; 
        break;
    case ICP_NO_DEST: 
        regAddr = 0x300 | (regNum & 0xff); 
        break;
    default: 
        regAddr = BAD_REGADDR; 
        break;
   }
   return (regAddr);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a long-word value from the specified AE CSR.
 * 
 * @param ae - IN 
 * @param csr - IN
 * @param value - OUT
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
int 
getAeCsr(icp_firml_handle_t *handle, 
         unsigned char ae, 
         unsigned int csr, 
         unsigned int *value)
{
    unsigned int iterations=CSR_DONE_COUNT;

    do{
        *value = GET_AE_CSR(handle, ae, csr);
        if(!(GET_AE_CSR(handle, ae, LOCAL_CSR_STATUS) & LCS_STATUS)) 
        {
             return (HALAE_SUCCESS);
        }     
    }while(iterations--);
    ERRINFO((" read CSR timeout\n")); 
    return (HALAE_FAIL);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the accelaration engine control status register indicated by csr and 
 *      returns the value in the value parameter. The csr value must be a valid
 *      accelaration engine CSR offset.
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
 * @param csr - IN A valid accelaration engine CSR offset
 * @param value - OUT A pointer to the location of the requested accelaration engine 
 *                    control status register
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_GetAeCsr(icp_firml_handle_t *handle, 
               unsigned char ae, 
               unsigned int csr, 
               unsigned int *value)
{
    int stat = HALAE_SUCCESS;
    
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    VERIFY_ARGPTR(value);
    SPIN_LOCK_AE(handle, ae);
    stat = getAeCsr(handle, ae, csr, value);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a long-word value to the specified AE CSR.
 * @param ae - IN
 * @param csr - IN
 * @param value - IN
 * @retval  HALAE_SUCCESS
 * 
 * 
 *****************************************************************************/
int 
putAeCsr(icp_firml_handle_t *handle, 
         unsigned char ae, 
         unsigned int csr, 
         unsigned int value)
{
    unsigned int iterations=CSR_DONE_COUNT;

    do{
        SET_AE_CSR(handle, ae, csr, value);
        if(!(GET_AE_CSR(handle, ae, LOCAL_CSR_STATUS) & LCS_STATUS)) 
        {
             return (HALAE_SUCCESS);
        }     
    }while(iterations--);

    ERRINFO(("Read CSR Timeout\n")); 
    return (HALAE_FAIL);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the longword value to the accelaration engine CSR indicated by csr 
 *      parameter. The csr value must be a valid AE CSR offset.
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
 * @param csr - IN A valid accelaration engine CSR offset
 * @param value - IN The new value for the specified accelaration engine control status
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_PutAeCsr(icp_firml_handle_t *handle, 
               unsigned char ae, 
               unsigned int csr, 
               unsigned int value)
{
    int stat = HALAE_SUCCESS;
    
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    if((csr == CSR_CTX_POINTER) && isAeActive(handle, ae)) 
    {
        stat = HALAE_AEACTIVE;
    }    
    else 
    {
        stat = putAeCsr(handle, ae, csr, value);
    }    
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Gets the silicon type and revision.
 *
 * @param majType - OUT
 * @param minType - OUT 
 * @param majRev - OUT
 * @param minRev - OUT
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
static int 
getProdId(icp_firml_handle_t *handle)
{
    unsigned int prodId;

    if(!handle->halHandle->prodId)
    {
        if(halAe_GetChipSetting(handle))
        {
            return (HALAE_FAIL);
        }
    }    

    prodId = handle->halHandle->prodId;

    handle->halHandle->PrdMajType = 
            (prodId & PID_MAJOR_PROD_TYPE) >> PID_MAJOR_PROD_TYPE_BITPOS;
    handle->halHandle->PrdMinType = 
            (prodId & PID_MINOR_PROD_TYPE) >> PID_MINOR_PROD_TYPE_BITPOS;
    handle->halHandle->PrdMajRev = 
            (prodId & PID_MAJOR_REV) >> PID_MAJOR_REV_BITPOS;
    handle->halHandle->PrdMinRev = 
            (prodId & PID_MINOR_REV) >> PID_MINOR_REV_BITPOS;

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Resets the specified acceleration engines with a corresponding bit set in 
 *      aeMask. If clrReg is set then the acceleration engines are initialized to 
 *      the states described in halAe_Init().
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
 * @param aeMask - IN Specifies the acceleration engines of interest
 * @param clrReg - IN If this parameter is set, then following register
 *                    initialization is performed:
 *                    1. CTX_ENABLES are set to zero
 *                    2. CTX_ARB_CNTL is set to zero
 *                    3. CC_ENABLE is set to x2000
 *                    4. All context program counters are set to zero
 *                    5. WAKEUP_EVENTS are set to one
 *                    6. SIG_EVENTS are set to zero
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
void 
halAe_Reset(icp_firml_handle_t *handle, 
            unsigned int aeMask, 
            int clrReg)
{
    unsigned int aeResetCsr, fuseCtl;
    unsigned char ae;
    unsigned int qatMask;

    DBGINFO(("halAe_Reset: aeMask=0x%x\n", aeMask));
    if((!handle) || (!handle->halHandle->HalAe_LibInit)) 
    {
        return;
    }    

    SPIN_LOCK(handle->halHandle->hal_GlobalLock);
    /* Enable/Disable Firmware Authentications */
    if ((handle->halHandle->PrdMinType == HWID_ACCEL_COMP_B) ||
        (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_RS))
    {
        GET_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), &fuseCtl);

        /* set or clear FWAUTHEN */
        if (handle->sysMemInfo.fwAuth == 0)
        {
            PUT_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), 
                         (fuseCtl & (~(1 << ACCELCOMP_B_AUTH_EN_BIT))));
        }
        else
        {
            PUT_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), 
                     (fuseCtl & (~(1 << ACCELCOMP_B_AUTH_EN_BIT))));
        }

        if (((fuseCtl >> ACCELCOMP_B_PROD_BIT) == 1) &&
            (handle->sysMemInfo.fwAuth == 0))
        {
            ERRINFO(("invalid configuration 0x%x(FUSE/AUTHEN)\n", 
                fuseCtl));
            SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
            return;
        }
    }

    aeMask &= ~(handle->halHandle->AeBadMask);
    if (aeMask != 0)
    {
        qatMask = handle->sysMemInfo.qatMask;
    }
    else
    {
        qatMask = 0;
    }
   
    /* reset the selected AEs */
    aeResetCsr = GET_GLB_CSR(handle, ICP_RESET);
    aeResetCsr &= ~GBE_MASK;
    
    /* set the appropriate AE bits */    
    SET_RESET_CSR_AE(aeResetCsr, aeMask);

    /* Set the SSU reset bit at the same time as the AE's
     * set the appropriate QAT bits */
    SET_RESET_CSR_QAT(aeResetCsr, qatMask);  

    /* write to the reset csr */
    SET_GLB_CSR(handle, ICP_RESET, aeResetCsr);

    for(ae = 0; 
        (ae <= handle->halHandle->AeMaxNum)
                && (ae < sizeof(setMasks)/sizeof(unsigned int)); 
        ae++)
    {
        if(handle->halHandle->AeBadMask & setMasks[ae]) 
        {
           continue;
        }   
        if((aeMask & setMasks[ae])) 
        {
            SET_RST(handle, ae);    /* set the reset indicator */
        }
    }

    SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
    
    return;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Stops the timestamp clock and zeroes the timestamps of all specified 
 *      acceleration engines then restarts the timestamp clock.
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
 * @param aeMask - IN Specifies one or more acceleration engines
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * 
 * 
 *****************************************************************************/
int 
halAe_ResetTimestamp(icp_firml_handle_t *handle, 
                     unsigned int aeMask)
{
    unsigned int miscCtl, zero=0;
    unsigned char ae;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);

    SPIN_LOCK(handle->halHandle->hal_GlobalLock);

    /* stop the timestamp timers */
    miscCtl = GET_GLB_CSR(handle, MISC_CONTROL);
    if(miscCtl & MC_TIMESTAMP_ENABLE) 
    {
        SET_GLB_CSR(handle, MISC_CONTROL, miscCtl & (~MC_TIMESTAMP_ENABLE));
    }    

    for(ae = 0; 
        (ae <= handle->halHandle->AeMaxNum)
                      && (ae < sizeof(setMasks)/sizeof(unsigned int));
        ae++)
    {
        if(handle->halHandle->AeBadMask & setMasks[ae]) 
        {
           continue;
        }   
        if((aeMask & setMasks[ae]))
        {
            halAe_PutAeCsr(handle, ae, TIMESTAMP_LOW, zero);
            halAe_PutAeCsr(handle, ae, TIMESTAMP_HIGH, zero);
        }
    }

    /* start timestamp timers */
    SET_GLB_CSR(handle, MISC_CONTROL, miscCtl | MC_TIMESTAMP_ENABLE);
    SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Force reset status to HALAE_CLR_RST
 * @param
 *
 * @retval  HALAE_SUCCESS
 * 
 * 
 *****************************************************************************/
void 
halAe_clrResetStatus(icp_firml_handle_t *handle)
{
    unsigned char ae;    
    
    for(ae = 0; 
        (ae <= handle->halHandle->AeMaxNum)
                   && (ae < sizeof(setMasks)/sizeof(unsigned int)); 
        ae++)
    {
        if(handle->halHandle->AeBadMask & setMasks[ae]) 
        {
           continue;
        }   
        CLR_RST(handle, ae);  /* clear the reset indicator */         
    }    
    
    return;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Takes the specified acceleration engines out of the reset state.
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
 * @param aeMask - IN Specifies the acceleration engines of interest
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/
int 
halAe_ClrReset(icp_firml_handle_t *handle, 
               unsigned int aeMask)
{
    unsigned int aeResetCsr, csr = 0;
    unsigned int clkEnableCsr;
    unsigned char ae;
    unsigned int qatMask;
    unsigned int slicePwrDwnMask;
    int times = CLRRESET_TIMES;
    int status = HALAE_SUCCESS;
    int i;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);

    SPIN_LOCK(handle->halHandle->hal_GlobalLock);

    aeMask &= ~(handle->halHandle->AeBadMask);
    if (aeMask != 0)
    {
        qatMask = handle->sysMemInfo.qatMask;
    }
    else
    {
        qatMask = 0;
    }

    aeResetCsr = GET_GLB_CSR(handle, ICP_RESET);
    aeResetCsr &= ~GBE_MASK;

    /* clear the appropriate AE bits */
    CLR_RESET_CSR_AE(aeResetCsr, aeMask); 

    /* Clear the SSU reset bit at the same time as the AE's
     * clear the appropriate QAT bits */
    CLR_RESET_CSR_QAT(aeResetCsr, qatMask); 

    /* write to the reset csr */
    SET_GLB_CSR(handle, ICP_RESET, aeResetCsr);

    /* make sure QAT and AEs are out of reset */
    csr = GET_GLB_CSR(handle, ICP_RESET);
    csr &= ~GBE_MASK;
    DBGINFO(("ICP_RESET=0x%x after ClrReset\n", csr));
    while(csr & (aeMask | (qatMask << RST_CSR_QAT_LSB)))
    {
        SET_GLB_CSR(handle, ICP_RESET, aeResetCsr);
        times --;
        if(times <= 0)
        {
            PRINTF("Fail to ClrReset!!\n");
            SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
            return HALAE_FAIL;
        }
        csr = GET_GLB_CSR(handle, ICP_RESET);
        csr &= ~GBE_MASK;
        DBGINFO(("times=%d, ICP_RESET=0x%x\n", times, csr));
    }
    /* enable clock for AE and QAT */
    clkEnableCsr = GET_GLB_CSR(handle, ICP_GLOBAL_CLK_ENABLE);
    SET_CLK_ENABLE_CSR_AE(clkEnableCsr, aeMask);
    SET_CLK_ENABLE_CSR_QAT(clkEnableCsr, qatMask);
    SET_GLB_CSR(handle, ICP_GLOBAL_CLK_ENABLE, clkEnableCsr);

    /* power on all slices of ACCEL_COMP_C for debug mode */
    if (((handle->halHandle->PrdMinType == HWID_ACCEL_COMP_C) ||
         (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_B) ||
         (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_RS)) &&
        (handle->sysMemInfo.debug == 1))
    {
       if (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_C)
       {
          slicePwrDwnMask = 0xfff;
       }
       else /*ACCEL_COMP_B/RS*/
       {
          slicePwrDwnMask = 0x801f3133;
       }

       for (i = 0; i < MAX_QAT; i++)
       {
           if ((qatMask >> i) & 0x1)
           {
               /* enable all slices */
               csr = GET_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN);
               csr &= ~slicePwrDwnMask;
               PUT_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN, csr);
               DBGINFO(("enable all slices of QAT %d\n", i));

               /* verify enable done */
               csr = GET_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN);
               if ((csr & slicePwrDwnMask) != 0)
               {
                   ERRINFO(("enable slices failed SLICEPWRDOWN=0x%x\n", csr));
                   SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
                   return HALAE_FAIL;
               }
           }
       }
    }

    status = halAe_checkAes(handle, aeMask);
    if(status != HALAE_SUCCESS)
    {
        ERRINFO(("halAe_checkAes error\n"));
        SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
        return (status);
    }

    /* Set undefined power-up/reset states to reasonable default values...
       just to make sure we're starting from a known point */
    for(ae = 0; 
        (ae <= handle->halHandle->AeMaxNum)
                  && (ae < sizeof(setMasks)/sizeof(unsigned int)); 
        ae++)
    {
        if(handle->halHandle->AeBadMask & setMasks[ae]) 
        {
           continue;
        }   
        if((aeMask & setMasks[ae]))
        {
            /* clear the reset indicator */            
            CLR_RST(handle, ae);                 
            /* init the ctx_enable */
            halAe_PutAeCsr(handle, ae, CTX_ENABLES, INIT_CTX_ENABLE_VALUE);
            /* initialize the PCs */
            halAe_PutCtxIndrCsr(handle, ae, AE_ALL_CTX, CTX_STS_INDIRECT, 
                          handle->halHandle->UpcMask & INIT_PC_VALUE);
            /* init the ctx_arb */
            halAe_PutAeCsr(handle, ae, CTX_ARB_CNTL, INIT_CTX_ARB_VALUE);
            /* enable cc */  
            halAe_PutAeCsr(handle, ae, CC_ENABLE, INIT_CCENABLE_VALUE);
            halAe_PutCtxWakeupEvents(handle, ae, AE_ALL_CTX, 
                            INIT_WAKEUP_EVENTS_VALUE);
            halAe_PutCtxSigEvents(handle, ae, AE_ALL_CTX,
                            INIT_SIG_EVENTS_VALUE);
        }
    }
    /* disable QAT interrupt */
    disableQatIntr(handle);

    /* for B0/C0 ACCEL_COMP, initialize sram explicitly
     * for A0 ACCEL_COMP, assume AE firmware would initialize sram
     * for ACCEL_COMP_C, hardware would initalize sram after reset
     * enable AutoInitSram only when aeMask != 0 */
    if((aeMask != 0) && !((handle->halHandle->PrdMinType == HWID_ACCEL_COMP) &&
         (handle->halHandle->PrdMajRev == ACCELCOMP_A_MAJOR_REV) &&
         (handle->halHandle->PrdMinRev == 0))
         && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_R))
    {
        /* init eSram only when this is boot time */
        if (handle->sysMemInfo.reload == 0) {
            status = halAe_enableAutoInitSram(handle, aeMask);
        }
    }

    if((aeMask != 0) && (qatMask != 0) &&
       ((handle->halHandle->PrdMinType == HWID_ACCEL_COMP_C) ||
        (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_B) ||
        (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_RS))) 
    { /* wait shram to complete initialization */
        halAe_WaitShramReady(handle, aeMask);
    }
    
    SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);

    halAe_ResetTimestamp(handle, aeMask);

    return status;
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
halAe_clearGPRs_Common(icp_firml_handle_t *handle, 
                unsigned int aeMask)
{
    unsigned int regData=0, absRegNum=0;
    unsigned char ae=0;
    unsigned int ctx_mask=AE_ALL_CTX;
    unsigned int ae_numbers[MAX_AE];
    int xCycles = AE_EXEC_CYCLE;
    int finished;
    unsigned int csrVal, newCsrVal, shCtlStoreFlag = 0;
    unsigned int savCtx = 0;
    int times = MAX_RETRY_TIMES;

    uword_T inst[] = {
        0x0F0000C0000ull,  /* .0 l0000!val = 0 ; immed[l0000!val, 0x0] */
        0x0F000000380ull,  /* .1 l0000!count = 128 ; 
                                       immed[l0000!count, 0x80] */
        0x0D805000011ull,  /* .2 br!=ctx[0, ctx_init#] */
        0x0FC082C0300ull,  /* .3 local_csr_wr[nn_put, 0] */
        0x0F0000C0300ull,  /* .4 nop */
        0x0F0000C0300ull,  /* .5 nop */
        0x0F0000C0300ull,  /* .6 nop */
        0x0F0000C0300ull,  /* .7 nop */
        0x0A0643C0000ull,  /* .8 init_nn#:alu[*n$index++, --, 
                                            b, l0000!val] */
        0x0BAC0000301ull,  /* .9 alu[l0000!count, l0000!count, -, 1] */
        0x0D802000101ull,  /* .10 bne[init_nn#] */
        0x0F0000C0001ull,  /* .11 l0000!indx = 0 ; immed[l0000!indx,
                                               0x0] */
        0x0FC066C0001ull,  /* .12 local_csr_wr[active_lm_addr_0, 
                                l0000!indx]; put indx to lm_addr */
        0x0F0000C0300ull,  /* .13 nop */
        0x0F0000C0300ull,  /* .14 nop */
        0x0F0000C0300ull,  /* .15 nop */
        0x0F000400300ull,  /* .16 l0000!count = 1024 ; 
                               immed[l0000!count, 0x400] */
        0x0A0610C0000ull,  /* .17 init_lm#:alu[*l$index0++, --, 
                                              b, l0000!val] */
        0x0BAC0000301ull,  /* .18 alu[l0000!count, l0000!count, -, 1] */
        0x0D804400101ull,  /* .19 bne[init_lm#] */
        0x0A0580C0000ull,  /* .20 ctx_init#:alu[$l0000!xfers[0],
                                        --, b, l0000!val] */
        0x0A0581C0000ull,  /* .21 alu[$l0000!xfers[1], --, b, l0000!val] */
        0x0A0582C0000ull,  /* .22 alu[$l0000!xfers[2], --, b, l0000!val] */
        0x0A0583C0000ull,  /* .23 alu[$l0000!xfers[3], --, b, l0000!val] */
        0x0A0584C0000ull,  /* .24 alu[$l0000!xfers[4], --, b, l0000!val] */
        0x0A0585C0000ull,  /* .25 alu[$l0000!xfers[5], --, b, l0000!val] */
        0x0A0586C0000ull,  /* .26 alu[$l0000!xfers[6], --, b, l0000!val] */
        0x0A0587C0000ull,  /* .27 alu[$l0000!xfers[7], --, b, l0000!val] */
        0x0A0588C0000ull,  /* .28 alu[$l0000!xfers[8], --, b, l0000!val] */
        0x0A0589C0000ull,  /* .29 alu[$l0000!xfers[9], --, b, l0000!val] */
        0x0A058AC0000ull,  /* .30 alu[$l0000!xfers[10], --, b, l0000!val] */
        0x0A058BC0000ull,  /* .31 alu[$l0000!xfers[11], --, b, l0000!val] */
        0x0A058CC0000ull,  /* .32 alu[$l0000!xfers[12], --, b, l0000!val] */
        0x0A058DC0000ull,  /* .33 alu[$l0000!xfers[13], --, b, l0000!val] */
        0x0A058EC0000ull,  /* .34 alu[$l0000!xfers[14], --, b, l0000!val] */
        0x0A058FC0000ull,  /* .35 alu[$l0000!xfers[15], --, b, l0000!val] */
        0x0A05C0C0000ull,  /* .36 alu[$l0000!xfers[16], --, b, l0000!val] */
        0x0A05C1C0000ull,  /* .37 alu[$l0000!xfers[17], --, b, l0000!val] */
        0x0A05C2C0000ull,  /* .38 alu[$l0000!xfers[18], --, b, l0000!val] */
        0x0A05C3C0000ull,  /* .39 alu[$l0000!xfers[19], --, b, l0000!val] */
        0x0A05C4C0000ull,  /* .40 alu[$l0000!xfers[20], --, b, l0000!val] */
        0x0A05C5C0000ull,  /* .41 alu[$l0000!xfers[21], --, b, l0000!val] */
        0x0A05C6C0000ull,  /* .42 alu[$l0000!xfers[22], --, b, l0000!val] */
        0x0A05C7C0000ull,  /* .43 alu[$l0000!xfers[23], --, b, l0000!val] */
        0x0A05C8C0000ull,  /* .44 alu[$l0000!xfers[24], --, b, l0000!val] */
        0x0A05C9C0000ull,  /* .45 alu[$l0000!xfers[25], --, b, l0000!val] */
        0x0A05CAC0000ull,  /* .46 alu[$l0000!xfers[26], --, b, l0000!val] */
        0x0A05CBC0000ull,  /* .47 alu[$l0000!xfers[27], --, b, l0000!val] */
        0x0A05CCC0000ull,  /* .48 alu[$l0000!xfers[28], --, b, l0000!val] */
        0x0A05CDC0000ull,  /* .49 alu[$l0000!xfers[29], --, b, l0000!val] */
        0x0A05CEC0000ull,  /* .50 alu[$l0000!xfers[30], --, b, l0000!val] */
        0x0A05CFC0000ull,  /* .51 alu[$l0000!xfers[31], --, b, l0000!val] */
        0x0A0400C0000ull,  /* .52 alu[l0000!gprega[0], --, b, l0000!val] */
        0x0B0400C0000ull,  /* .53 alu[l0000!gpregb[0], --, b, l0000!val] */
        0x0A0401C0000ull,  /* .54 alu[l0000!gprega[1], --, b, l0000!val] */
        0x0B0401C0000ull,  /* .55 alu[l0000!gpregb[1], --, b, l0000!val] */
        0x0A0402C0000ull,  /* .56 alu[l0000!gprega[2], --, b, l0000!val] */
        0x0B0402C0000ull,  /* .57 alu[l0000!gpregb[2], --, b, l0000!val] */
        0x0A0403C0000ull,  /* .58 alu[l0000!gprega[3], --, b, l0000!val] */
        0x0B0403C0000ull,  /* .59 alu[l0000!gpregb[3], --, b, l0000!val] */
        0x0A0404C0000ull,  /* .60 alu[l0000!gprega[4], --, b, l0000!val] */
        0x0B0404C0000ull,  /* .61 alu[l0000!gpregb[4], --, b, l0000!val] */
        0x0A0405C0000ull,  /* .62 alu[l0000!gprega[5], --, b, l0000!val] */
        0x0B0405C0000ull,  /* .63 alu[l0000!gpregb[5], --, b, l0000!val] */
        0x0A0406C0000ull,  /* .64 alu[l0000!gprega[6], --, b, l0000!val] */
        0x0B0406C0000ull,  /* .65 alu[l0000!gpregb[6], --, b, l0000!val] */
        0x0A0407C0000ull,  /* .66 alu[l0000!gprega[7], --, b, l0000!val] */
        0x0B0407C0000ull,  /* .67 alu[l0000!gpregb[7], --, b, l0000!val] */
        0x0A0408C0000ull,  /* .68 alu[l0000!gprega[8], --, b, l0000!val] */
        0x0B0408C0000ull,  /* .69 alu[l0000!gpregb[8], --, b, l0000!val] */
        0x0A0409C0000ull,  /* .70 alu[l0000!gprega[9], --, b, l0000!val] */
        0x0B0409C0000ull,  /* .71 alu[l0000!gpregb[9], --, b, l0000!val] */
        0x0A040AC0000ull,  /* .72 alu[l0000!gprega[10], --, b, l0000!val] */
        0x0B040AC0000ull,  /* .73 alu[l0000!gpregb[10], --, b, l0000!val] */
        0x0A040BC0000ull,  /* .74 alu[l0000!gprega[11], --, b, l0000!val] */
        0x0B040BC0000ull,  /* .75 alu[l0000!gpregb[11], --, b, l0000!val] */
        0x0A040CC0000ull,  /* .76 alu[l0000!gprega[12], --, b, l0000!val] */
        0x0B040CC0000ull,  /* .77 alu[l0000!gpregb[12], --, b, l0000!val] */
        0x0A040DC0000ull,  /* .78 alu[l0000!gprega[13], --, b, l0000!val] */
        0x0B040DC0000ull,  /* .79 alu[l0000!gpregb[13], --, b, l0000!val] */
        0x0A040EC0000ull,  /* .80 alu[l0000!gprega[14], --, b, l0000!val] */
        0x0B040EC0000ull,  /* .81 alu[l0000!gpregb[14], --, b, l0000!val] */
        0x0A040FC0000ull,  /* .82 alu[l0000!gprega[15], --, b, l0000!val] */
        0x0B040FC0000ull,  /* .83 alu[l0000!gpregb[15], --, b, l0000!val] */
        0x0D81581C010ull,  /* .84 br=ctx[7, exit#] */
        0x0E000010000ull,  /* .85 ctx_arb[kill], any */
        0x0E000010000ull,  /* .86 exit#:ctx_arb[kill], any */
    };

    if(aeMask == 0)
    {
        /* do nothing for attach mode debugging */
        return (HALAE_SUCCESS);
    }
 
    for(ae=0; ae <= handle->halHandle->AeMaxNum; ae++)
    {
        if(!(aeMask & (1 << ae))) 
        {
             continue;
        }     
        for(absRegNum=0; absRegNum < MAX_GPR_REG; absRegNum++)
        {
            halAe_PutAbsDataReg(handle, ae, ICP_SR_RD_ABS, 
                            (unsigned short)absRegNum, regData);
            halAe_PutAbsDataReg(handle, ae, ICP_DR_RD_ABS, 
                            (unsigned short)absRegNum, regData);
        }
    }
    for(ae=0; ae <= handle->halHandle->AeMaxNum; ae++)
    {
        ae_numbers[ae] = -1;

        if(!(aeMask & (1 << ae))) {
             continue;
        }
        if(halAe_VerifyAe(handle, ae) != HALAE_SUCCESS) {
            continue;
        }

        if(IS_RST(handle, ae)) {
             continue;
        }
        if(isAeActive(handle, ae)) {
             continue;
        }
          /* turn off share control store bit */
        csrVal = GET_AE_CSR(handle, ae, AE_MISC_CONTROL);
        shCtlStoreFlag = csrVal & (0x1 << MMC_SHARE_CS_BITPOS);

        newCsrVal = CLR_BIT(csrVal, MMC_SHARE_CS_BITPOS);
        SET_AE_CSR(handle, ae, AE_MISC_CONTROL, newCsrVal);
        halAe_WaitAeLocalCsrReady(handle, ae);

        /* turn off ucode parity */
        /* make sure nn_mode is set to self */
        getAeCsr(handle, ae, CTX_ENABLES, &csrVal);
        newCsrVal = csrVal & IGNORE_W1C_MASK;
        newCsrVal |= CE_NN_MODE;
        SET_AE_CSR(handle, ae, CTX_ENABLES, newCsrVal & \
                (~(1 << CE_CNTL_STORE_PARITY_ENABLE_BITPOS)));
        halAe_WaitAeLocalCsrReady(handle, ae);
    
        /* copy instructions to ustore */
        putUwords(handle, ae, 0, sizeof(inst)/sizeof(uword_T), inst);
        /* set PC */
        putCtxIndrCsr(handle, ae, ctx_mask, CTX_STS_INDIRECT, 
                      handle->halHandle->UpcMask & INIT_PC_VALUE);

        /* save current context */
        savCtx = GET_AE_CSR(handle, ae, ACTIVE_CTX_STATUS);    
        /* change the active context */
        /* start the context from ctx 0 */
        SET_AE_CSR(handle, ae, ACTIVE_CTX_STATUS, 0);
        halAe_WaitAeLocalCsrReady(handle, ae);

        /* wakeup-event voluntary */
        putCtxWakeupEvents(handle, ae, ctx_mask, XCWE_VOLUNTARY);
        /* clean signals */
        putCtxIndrCsr(handle, ae, ctx_mask, 
                        CTX_SIG_EVENTS_INDIRECT, 0);
        SET_AE_CSR(handle, ae, CTX_SIG_EVENTS_ACTIVE, 0);
        halAe_WaitAeLocalCsrReady(handle, ae);
    
        enableCtx(handle, ae, ctx_mask);

        /* set ae to be ready */
        ae_numbers[ae] = 0;
    }

     /* wait for all AEs to be finished */
    finished = 0;
    while ((!finished) && times) {
       finished = 1;
       for(ae=0; ae <= handle->halHandle->AeMaxNum; ae++) {
           if (ae_numbers[ae] == 0) {
                 /* Check if the AE has been killed */
              if(waitNumCycles(handle, ae, xCycles, 1) != 0) {
                  finished = 0;
                  times --;
                  if(times <= 0)
                  {
                      PRINTF("halAe_clearGPRs timeout!!\n");
                      return HALAE_FAIL;
                  }      
              } else {
                  ae_numbers[ae] = 1;
                  disableCtx(handle, ae, ctx_mask);
                  /* only restore shared control store bit, 
                     other bit might be changed by AE code snippet */
                  csrVal = GET_AE_CSR(handle, ae, AE_MISC_CONTROL);
                  if(shCtlStoreFlag) 
                  {
                     newCsrVal = SET_BIT(csrVal, MMC_SHARE_CS_BITPOS);
                  }   
                  else 
                  {
                     newCsrVal = CLR_BIT(csrVal, MMC_SHARE_CS_BITPOS);
                  }   
                  SET_AE_CSR(handle, ae, AE_MISC_CONTROL, newCsrVal);
                  halAe_WaitAeLocalCsrReady(handle, ae);
                  /* change the active context */    
                  SET_AE_CSR(handle, ae, ACTIVE_CTX_STATUS, savCtx & ACS_ACNO);
                  halAe_WaitAeLocalCsrReady(handle, ae);
         
                  /* Set undefined power-up/reset states to reasonable default values...
                     just to make sure we're starting from a known point */
                  /* init the ctx_enable */
                  SET_AE_CSR(handle, ae, CTX_ENABLES, INIT_CTX_ENABLE_VALUE);
                  halAe_WaitAeLocalCsrReady(handle, ae);
                  /* initialize the PCs */
                  putCtxIndrCsr(handle, ae, ctx_mask, CTX_STS_INDIRECT, 
                                handle->halHandle->UpcMask & INIT_PC_VALUE);
                  /* init the ctx_arb */
                  SET_AE_CSR(handle, ae, CTX_ARB_CNTL, INIT_CTX_ARB_VALUE);
                  halAe_WaitAeLocalCsrReady(handle, ae);
                  /* enable cc */  
                  SET_AE_CSR(handle, ae, CC_ENABLE, INIT_CCENABLE_VALUE);
                  halAe_WaitAeLocalCsrReady(handle, ae);
                  putCtxWakeupEvents(handle, ae, ctx_mask, 
                                     INIT_WAKEUP_EVENTS_VALUE);
                  putCtxSigEvents(handle, ae, ctx_mask,
                                  INIT_SIG_EVENTS_VALUE);
          
              }
           }
      }
   }
   return (HALAE_SUCCESS);

}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Set SIGNATURE_ENABLE[0] to 0x1 in order to enable ALU_OUT csr
 * @param
 *
 * @retval  HALAE_SUCCESS, HALAE_FAIL
 * 
 * 
 *****************************************************************************/
static int 
halAe_initPrivateCSR(icp_firml_handle_t *handle, 
                     unsigned int aeMask)
{
    unsigned char ae=0;
    unsigned int csrVal=0;

    for(ae=0; ae <= handle->halHandle->AeMaxNum; ae++)
    {
        if(!(aeMask & (1 << ae))) 
        {
             continue;
        }     
        halAe_GetAeCsr(handle, ae, SIGNATURE_ENABLE, &csrVal);
        csrVal |= 0x1;
        halAe_PutAeCsr(handle, ae, SIGNATURE_ENABLE, csrVal);
    }
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      
 * @param
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
static void 
halAe_enableIntr(icp_firml_handle_t *handle)
{
    if (!halAe_intrSupported())
    {
        return;
    }

    SET_CAP_CSR(handle, CAP_ATTN_MASK_SET, ~(handle->halHandle->AeBadMask));

    return;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      
 * @param
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
static void 
halAe_disableIntr(icp_firml_handle_t *handle)
{
    if (!halAe_intrSupported())
    {
        return;
    }

    SET_CAP_CSR(handle, CAP_ATTN_MASK_CLR, ~(handle->halHandle->AeBadMask));

    return;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      
 * @param
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
int 
halAe_DisableClkGating(icp_firml_handle_t *handle)
{
    unsigned int fuseCtl = 0, csr = 0;
    unsigned int slicePwrDwnMask;
    int i = 0, status = HALAE_SUCCESS;

    if ((handle->sysMemInfo.deviceId != ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC) &&
        (handle->sysMemInfo.deviceId != ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) &&
        (handle->sysMemInfo.deviceId != ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
    {
        return status;
    }

    /* return sucess if non-debug mode */
    if (handle->sysMemInfo.debug == 0)
    {
        return status;
    }
    
    GET_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), &fuseCtl);
    DBGINFO(("AC_FUSECTL value is : 0x%x\n", fuseCtl));
        
    if (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC)
    {
        /* alert the FWCG is set already */
        if (fuseCtl & (1 << ACCELCOMP_C_FWCG_BIT))
        {
            DBGINFO(("Slice clock-gating(FWCG) is already disabled"));
        }
        else
        {
            PUT_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), 
                         (fuseCtl | (1 << ACCELCOMP_C_FWCG_BIT)));
            GET_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), &fuseCtl);

            if ((fuseCtl & (1 << ACCELCOMP_C_FWCG_BIT)) == 0)
            {        
                for (i = 0; i < MAX_QAT; i++)
                {
                    if ((handle->sysMemInfo.qatMask >> i) & 0x1)
                    {
                        csr = GET_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN);
                        csr |= 1 << ACCELCOMP_C_SLICEPWRDOWN_CPMCLKGATE_BIT;
                        PUT_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN, csr);
                        
                        DBGINFO(("disable clock gating of QAT %d by set"\
                                 "SLICEPWRDOWN bit23 \n", i));
            
                        /* verify clock gating is disabled successfully or not*/
                        csr = GET_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN);
                        if ((csr & 
                            (1 << ACCELCOMP_C_SLICEPWRDOWN_CPMCLKGATE_BIT))
                            == 0)
                        {
                            DBGINFO(("Disable clock gating of QAT %d fail\n",
                                     i));
                            status = HALAE_FAIL;
                        }
                    }
                }
            }
        }
    }
    else if ((handle->sysMemInfo.deviceId == ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC)
        || (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
    {
        /* alert the FWCG is set already */
        if ((fuseCtl & (1 << ACCELCOMP_B_FWCG_BIT)) &&
            ((fuseCtl & (1 << ACCELCOMP_B_PGPKE_BIT)) == 0) &&
            ((fuseCtl & (1 << ACCELCOMP_B_PGCOM_BIT)) == 0))
        {
            DBGINFO(("Slice clock-gating(FWCG) is already disabled"));
        }
        else
        {
            if ((fuseCtl & (1 << ACCELCOMP_B_FWCG_BIT)) == 0)
            {
                PUT_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), 
                             (fuseCtl | (1 << ACCELCOMP_B_FWCG_BIT)));
                GET_PCI_CSR (handle, 
                             AC_FUSECTL, 
                             sizeof(unsigned int), 
                             &fuseCtl);

                if ((fuseCtl & (1 << ACCELCOMP_B_FWCG_BIT)) == 0)
                {        
                    for (i = 0; i < MAX_QAT; i++)
                    {
                        if ((handle->sysMemInfo.qatMask >> i) & 0x1)
                        {
                            csr = GET_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN);
                            csr |= 1<<ACCELCOMP_B_SLICEPWRDOWN_CPMCLKGATE_BIT;
                            PUT_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN, csr);

                            DBGINFO(("disable clock gating of QAT %d by set"\
                                      "SLICEPWRDOWN bit23 \n", i));
                
                            /* verify clock gating is disabled successfully
                             * or not*/
                            csr = GET_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN);
                            if ((csr & 
                                 (1 << ACCELCOMP_B_SLICEPWRDOWN_CPMCLKGATE_BIT))
                                == 0)
                            {
                                DBGINFO(("Disable clock gating of QAT %d"\
                                          "fail\n", i));
                                status = HALAE_FAIL;
                            }
                        }
                    }
                }
            }
            
            if (fuseCtl & (1 << ACCELCOMP_B_PGPKE_BIT))
            {
                PUT_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), 
                             (fuseCtl & ~(1 << ACCELCOMP_B_PGPKE_BIT)));
                GET_PCI_CSR (handle, 
                             AC_FUSECTL, 
                             sizeof(unsigned int), 
                             &fuseCtl);
                if (fuseCtl & (1 << ACCELCOMP_B_PGPKE_BIT))
                { 
                    DBGINFO(("All PKE slices are power gated \n"));
                    status = HALAE_FAIL;
                }
            }
            
            if (fuseCtl & (1 << ACCELCOMP_B_PGCOM_BIT))
            {
                PUT_PCI_CSR (handle, AC_FUSECTL, sizeof(unsigned int), 
                             (fuseCtl & ~(1 << ACCELCOMP_B_PGCOM_BIT)));
                GET_PCI_CSR (handle,
                             AC_FUSECTL, 
                             sizeof(unsigned int), 
                             &fuseCtl);
                if (fuseCtl & (1 << ACCELCOMP_B_PGCOM_BIT))
                { 
                    DBGINFO(("All Compression and Translator slices are"\
                              "power gated \n"));
                    status = HALAE_FAIL;
                }
            }
        }  
    }

    if (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_C)
    {
        slicePwrDwnMask = 0xfff;
    }
    else /*ACCEL_COMP_B/RS*/
    {
        slicePwrDwnMask = 0x801f3133;
    }

    /* power on all QAT slices */
    for (i = 0; i < MAX_QAT; i++)
    {
        if ((handle->sysMemInfo.qatMask >> i) & 0x1)
        {
            csr = GET_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN);
            csr &= ~slicePwrDwnMask;
            PUT_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN, csr);
            DBGINFO(("enable all slices of QAT %d\n", i));

            /* verify power on is successful*/
            csr = GET_SSU_QAT_CSR(handle, i, SSU_SLICEPWRDOWN);
            if ((csr & slicePwrDwnMask) != 0)
            {
                DBGINFO(("enable all slices of QAT %d fail\n", i));
                status = HALAE_FAIL;
            }
        }
    }

    return status;
}

/*------------------------------------
  Begin support for New Page callback
  ------------------------------------*/
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Initializes the acceleration engines and takes them out of reset. 
 *      acceleration engines with their corresponding bit set in aeMask are initialized
 *      to the following state:
 *      1. Context mode is set to eight
 *      2. Program counters are set to zero
 *      3. Next context to run is set to zero
 *      4. All contexts are disabled
 *      5. cc_enable is set to 0x2000
 *      6. Wakeup events are set to one
 *      7. The signal event is set to zero
 *      All other acceleration engines remain untouched thought they are taken out of 
 *      reset. This function should be called prior to calling most of the HAL
 *      functions.
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
 * @param aeMask - IN The bits correspond to the accelaration engine number and address
 *                    of acceleration engines to be initialized. For example, the first
 *                    accelaration engine in the second cluster corresponds to bit 16.
 *                    write the value uWord at the address uAddr.
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/
int 
halAe_Init(icp_firml_handle_t *handle, 
           unsigned int aeMask)
{
    unsigned char ae=0, clust=0;
    int status=HALAE_SUCCESS, ii=0;
    PageData_T *pageData=NULL;

    VERIFY_ARGPTR(handle);

    /* aeMask should be matched with real aeMask */
    aeMask = aeMask & handle->sysMemInfo.aeMask;

    DBGINFO(("halAe_Init: aeMask=0x%x, HalAe_LibInit=%d\n",
                            aeMask, handle->halHandle->HalAe_LibInit));

    SPIN_LOCK(handle->lock);
    if(!handle->halHandle->HalAe_LibInit)
    {
        if((status = halAe_OSInit(handle)) != HALAE_SUCCESS) 
        {
            SPIN_UNLOCK(handle->lock);
            ERRINFO(("halAe_OSInit error\n"));      
            return (status);
        }

        /* create spinlock to protect global variables */
        SPIN_LOCK_INIT(handle->halHandle->hal_GlobalLock);
        SPIN_LOCK_INIT(handle->halHandle->hal_CallbackThdLock);
        SPIN_LOCK_INIT(handle->halHandle->hal_InterruptLock);
        SPIN_LOCK_INIT(handle->halHandle->bk_lock);
        SPIN_LOCK_INIT(handle->halHandle->tasklet_lock);
        SPIN_LOCK_INIT(handle->halHandle->bpt_lock);

        handle->halHandle->ISR_callback_root = NULL;

        /* get the product ID */
        getProdId(handle);
        if (halAe_GetProdSetting(handle))
        {
            ERRINFO(("halAe_GetProdSetting error\n")); 
            SPIN_UNLOCK(handle->lock);
            SPIN_LOCK_FINI(handle->halHandle->hal_GlobalLock);
            SPIN_LOCK_FINI(handle->halHandle->hal_CallbackThdLock);
            SPIN_LOCK_FINI(handle->halHandle->hal_InterruptLock);
            SPIN_LOCK_FINI(handle->halHandle->bk_lock);
            SPIN_LOCK_FINI(handle->halHandle->tasklet_lock);            
            SPIN_LOCK_FINI(handle->halHandle->bpt_lock);
            return (HALAE_FAIL);
        }

        /* create AE objects */
        for(clust = 0; clust < AE_NUMCLUSTR; clust++)
        {
            for(ae = 0;
                (ae < handle->halHandle->AePerCluster)
                                && (ae < AE_PERCLUSTR); 
                ae++)
            {
                if (!(handle->sysMemInfo.aeMask & (1 << ae))) 
                {
                    continue;
                }
                /* set available ustores */
                handle->halHandle->AEs[clust][ae].freeAddr = 0;    
                handle->halHandle->AEs[clust][ae].freeSize =
                                              handle->halHandle->MaxUstore;
                handle->halHandle->AEs[clust][ae].uStoreSize =
                                              handle->halHandle->MaxUstore;
                handle->halHandle->AEs[clust][ae].liveCtxMask = AE_ALL_CTX;
                handle->halHandle->AEs[clust][ae].state = HALAE_UNINIT;
                handle->halHandle->AEs[clust][ae].ustoreDramAddr = 0;
                handle->halHandle->AEs[clust][ae].reloadSize = 0;
                SPIN_LOCK_INIT(handle->halHandle->AEs[clust][ae].aeLock);
            }
        }    

        /* Set intr CSRs on CAPBAR to enable atten
         * once hal initialization.
         * These CSRs will be reset by SHaC reset.
         * They will not be cleared after resetting AE and SSU via
         * calling halAe_Reset.
         * Hence, no need to re-config them in halAe_ClrReset */
        halAe_enableIntr(handle);
        handle->halHandle->HalAe_LibInit = 1;

        SPIN_UNLOCK(handle->lock);

        /* osalMemAlloc can put the current process to sleep waiting 
         * for a page when called in low-memory situations. 
         * So, it should not be put in spin lock.
         * osalMemAllocAtomic will not put the current process to sleep. 
         * But if the last page does not exist, the allocation fails.
         * Considering above factors, here put osalMemAlloc out of spin lock
         */
        pageData = (PageData_T*)osalMemAlloc(sizeof(PageData_T));
        if (pageData == NULL) 
        {  
            ERRINFO(("malloc pageData error\n ")); 
            SPIN_LOCK_FINI(handle->halHandle->hal_GlobalLock);
            SPIN_LOCK_FINI(handle->halHandle->hal_CallbackThdLock);
            SPIN_LOCK_FINI(handle->halHandle->hal_InterruptLock);
            SPIN_LOCK_FINI(handle->halHandle->bk_lock);
            SPIN_LOCK_FINI(handle->halHandle->tasklet_lock);
            SPIN_LOCK_FINI(handle->halHandle->bpt_lock);
            handle->halHandle->HalAe_LibInit = 0;
            return (HALAE_FAIL);
        }
        for (ae = 0; ae < MAX_AE; ae++) 
        {
            pageData->AePageData[ae].numPages = 0;
            pageData->AePageData[ae].addrs = NULL;
        }
        handle->halHandle->pageData = pageData;
    }
    else
    {
        SPIN_UNLOCK(handle->lock);
    }

    /* init selected AEs, and take all AEs out of reset */
    status = halAe_ClrReset(handle, aeMask);
    if(status != HALAE_SUCCESS)
    {
        ERRINFO(("halAe_ClrReset error\n"));
        return (status);
    }
    SPIN_LOCK(handle->lock);
    if ((handle->halHandle->PrdMinType == HWID_ICP) 
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP)
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_C)
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_B)
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_R)
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_RS))
    {
        halAe_clearGPRs(handle, aeMask);
        if (halAe_intrSupported())
        {
            halAe_initPrivateCSR(handle, aeMask);
        }
    }
    SPIN_UNLOCK(handle->lock);

    for (ii=0; ii<MAX_AE; ii++) 
    {
        ucloCallbacks[ii].uclo_callback = NULL;
    }

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Restores the system resources allocated by the halAe_Init() function. 
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
 * @param - none
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
void 
halAe_DelLib(icp_firml_handle_t *handle)
{
    if((handle == NULL) || (handle->halHandle == NULL))
    {
        return;
    }
    DBGINFO(("halAe_DelLib: HalAe_LibInit=%d\n",
             handle->halHandle->HalAe_LibInit));
    SPIN_LOCK(handle->lock);
    if(handle->halHandle->HalAe_LibInit)
    {
        unsigned char ae=0, clust=0;
        int ii=0;
        PageData_T *pageData;

        halAe_IntrDisable(handle, 0xFFFFFFFF);

        /* Remove UCLO callbacks */
        for (ii=0; ii<MAX_AE; ii++) 
        {
            ucloCallbacks[ii].uclo_callback = NULL;
        }

        /* clear AE_ATTN for all the AEs */
        halAe_disableIntr(handle);

        for(clust = 0; clust < AE_NUMCLUSTR; clust++)
        {
            for(ae = 0;
                (ae < handle->halHandle->AePerCluster)
                                && (ae < AE_PERCLUSTR);
                ae++)
            {
                if (!(handle->sysMemInfo.aeMask & (1 << ae))) 
                {
                    continue;
                }
                handle->halHandle->AEs[clust][ae].state = HALAE_UNINIT;
                SPIN_LOCK_FINI(handle->halHandle->AEs[clust][ae].aeLock);
            }
        }
         
        SPIN_LOCK_FINI(handle->halHandle->hal_GlobalLock);
        SPIN_LOCK_FINI(handle->halHandle->hal_CallbackThdLock);
        SPIN_LOCK_FINI(handle->halHandle->hal_InterruptLock);
        SPIN_LOCK_FINI(handle->halHandle->bk_lock);
        SPIN_LOCK_FINI(handle->halHandle->tasklet_lock);
        SPIN_LOCK_FINI(handle->halHandle->bpt_lock);

        halAe_OSClose(handle);

        pageData = handle->halHandle->pageData;
        for (ae = 0; ae < MAX_AE; ae++) 
        {
            if (pageData->AePageData[ae].addrs) 
            {
                osalMemFree(pageData->AePageData[ae].addrs);
            }   
        }
        osalMemFree(pageData);
        handle->halHandle->pageData = NULL;
        
        handle->halHandle->HalAe_LibInit = 0;

    }
    /* free below handle in halAe_DelLib 
     * so as to free user & kernel space handles */
    osalMemFree(handle->halHandle);
    SPIN_UNLOCK(handle->lock);

    SPIN_LOCK_FINI(handle->lock);
    osalMemFree(handle);
    
    return;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Define the reloadable ustore, reloadSize must be 2k, 6k, or 8k.
 *
 * @param
 *
 * @retval HALAE_SUCCESS, HALAE_BADARG, HALAE_MEMALLOC
 * 
 * 
 *****************************************************************************/
int 
halAe_SetReloadUstore(icp_firml_handle_t *handle, 
                      unsigned char ae, 
                      unsigned int reloadSize, 
                      int sharedMode,
                      unsigned int ustoreDramAddr)
{
    unsigned int csrVal, controlStoreReload;
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);

    DBGINFO(("halAe_SetReloadUstore: Ae=%d, reloadSize=%d\n", ae, reloadSize));
    switch(reloadSize)
    {
    case 0: controlStoreReload = 0x0; break;
    case KILO_2: controlStoreReload = 0x1; break;
    case KILO_4: controlStoreReload = 0x2; break;
    case KILO_8: controlStoreReload = 0x3; break;
    default: 
        ERRINFO(("reloadSize=0x%x\n",reloadSize)); 
        return (HALAE_BADARG);
    }

    SPIN_LOCK_AE(handle, ae);
    if(controlStoreReload)
    {
        AE(handle, ae).ustoreDramAddr = ustoreDramAddr;
    }
    AE(handle, ae).reloadSize = reloadSize;
    getAeCsr(handle, ae, AE_MISC_CONTROL, &csrVal);
    /* clear bits <22:20> and two */
    csrVal &= ~((0x7 << MMC_CS_RELOAD_BITPOS) | MMC_SHARE_CS_BITPOS); 
    csrVal |= controlStoreReload << MMC_CS_RELOAD_BITPOS | \
              ((sharedMode & 0x1) << MMC_ONE_CTX_RELOAD_BITPOS);
    putAeCsr(handle, ae, AE_MISC_CONTROL, csrVal);
    SPIN_UNLOCK_AE(handle, ae);

    return (HALAE_SUCCESS);
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Defines a region of microstore that is unused. 
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
 * @param begFreeAddr - IN Specifies the microstore address where the free 
                        region begins
 * @param size - IN Indicates the number of free microstore words
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/
int 
halAe_SetUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int freeAddr, 
                       unsigned int freeSize)
{
    unsigned char scsMode=0;
    int rc = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);

    DBGINFO(("halAe_SetUstoreFreeMem: Ae=%d, freeAddr=%d, freeSize=%d\n", ae, 
           freeAddr, freeSize));
    SPIN_LOCK_AE(handle, ae);
    getAeSharedCsMode(handle, ae, &scsMode);
    if(scsMode) 
    {
        if((freeAddr + freeSize) > (handle->halHandle->MaxUstore*0x2))
        {
            ERRINFO(("freeAddr=0x%x,freeSize=0x%x\n", freeAddr,freeSize)); 
            SPIN_UNLOCK_AE(handle, ae);
            return (HALAE_BADARG);
        }    
    }
    else
    {
        if((freeAddr + freeSize) > handle->halHandle->MaxUstore) 
        {
            ERRINFO(("freeAddr=0x%x,freeSize=0x%x\n", freeAddr,freeSize)); 
            SPIN_UNLOCK_AE(handle, ae);
            return (HALAE_BADARG);
        }    
    }
    
    rc = halAe_setUstoreFreeMem(handle, ae, freeAddr, freeSize);
    SPIN_UNLOCK_AE(handle, ae);

    return rc;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Returns the starting address and size of the specified accelaration engine 
 *      microstore free region
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
 * @param begFreeAddr - OUT A pointer to the location of the beginning of 
 *                          the free region for the specified accelaration engine 
 *                          microstore.                        
 * @param size - OUT A pointer to the location of the size of the free region
 *                   for the specified accelaration engine microstore
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADARG Bad function argument
 * 
 * 
 *****************************************************************************/
int 
halAe_GetUstoreFreeMem(icp_firml_handle_t *handle, 
                       unsigned char ae, 
                       unsigned int *freeAddr, 
                       unsigned int *freeSize)
{
    
    int rc = HALAE_SUCCESS;
    DBGINFO(("halAe_GetUstoreFreeMem: Ae=%d\n", ae));
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);

    if(!freeAddr || !freeSize) 
    {
        ERRINFO(("freeAddr=0x%p,freeSize=0x%p\n", freeAddr,freeSize)); 
        return (HALAE_BADARG);
    }    

    SPIN_LOCK_AE(handle, ae);
    rc = halAe_getUstoreFreeMem(handle, ae, freeAddr, freeSize);
    SPIN_UNLOCK_AE(handle, ae);

    return rc;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Enable the specified micro-thread(s) -- those that were
 *               already enabled, will remain enabled.
 * 
 * @param ae - IN 
 * @param ctxMask - IN
 *
 * @retval HALAE_SUCCESS, HALAE_RESET, HALAE_BADLIB
 * 
 * 
 *****************************************************************************/
static int 
enableCtx(icp_firml_handle_t *handle, 
          unsigned char ae, 
          unsigned int ctxMask)
{
    unsigned int ctxEn, wrCtxEn;

//    DBGINFO(("enableCtx: ae=%d, ctxMask=0x%x\n", ae, ctxMask));
    if(IS_RST(handle, ae)) 
    {
       return (HALAE_RESET);
    }   

    /* get the contexts that are enabled */
    getAeCsr(handle, ae, CTX_ENABLES, &ctxEn);
    /* prevent clearing the W1C bits: the breakpoint bit, 
       ECC error bit, and Parity error bit */
    ctxEn &= IGNORE_W1C_MASK;            

    if(ctxEn & CE_INUSE_CONTEXTS) {
        ctxMask &= 0x55;
    }    
    else
    {
        ctxMask &= 0xFF;
    }    

    /* set selected bits high */
    wrCtxEn = (ctxMask << CE_ENABLE_BITPOS) | ctxEn; 
    putAeCsr(handle, ae, CTX_ENABLES, wrCtxEn);

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Disable the specified micro-thread(s).
 * 
 * @param ae - IN 
 * @param ctxMask - IN
 *
 * @retval HALAE_SUCCESS, HALAE_BADLIB, HALAE_RESET, HALAE_FAIL
 * 
 * 
 *****************************************************************************/
static int 
disableCtx(icp_firml_handle_t *handle, 
           unsigned char ae, 
           unsigned int ctxMask)
{
    unsigned int ctxEn, wrCtxEn;

//    DBGINFO(("stopAe: ae=%d, ctxMask=0x%x\n", ae, ctxMask));
    if(IS_RST(handle, ae)) 
    {
       return (HALAE_RESET);
    }   

    /* get the contexts that are enabled */
    getAeCsr(handle, ae, CTX_ENABLES, &ctxEn);
    /* prevent clearing the W1C bits: the breakpoint bit, 
       ECC error bit, and Parity error bit */
    ctxEn &= IGNORE_W1C_MASK;            

    /* set the ctx_enable<enable> bits */
    wrCtxEn = (~((ctxMask & AE_ALL_CTX) << CE_ENABLE_BITPOS)) & ctxEn;
    putAeCsr(handle, ae, CTX_ENABLES, wrCtxEn);

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the wake-event to voluntary and starts the accelaration engine context 
 *      specified by the ae and startCtx parameters from that accelaration engine's 
 *      current program counter. The ctxEnMask specifies the contexts to be 
 *      enabled. If one of these acceleration engines is in reset it is taken out 
 *      of reset.
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
 * @param ctxEnMask - IN Specifies the contexts to be enabled
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB A bad debug library is associated with the specified
                        accelaration engine the library is not initialized
 * @retval HALAE_BADARG Bad function argument
 * 
 *****************************************************************************/
int 
halAe_Start(icp_firml_handle_t *handle, 
            unsigned char ae, 
            unsigned int ctxEnMask)
{
    int stat = HALAE_SUCCESS;
    int retry = 0;
    unsigned int fcuSts = 0;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);

    if (handle->sysMemInfo.fwAuth == 1)
    {
        SPIN_LOCK(handle->halHandle->hal_GlobalLock);

        /* return true if authentication done */
        if (((fcuSts >> FCU_STS_DONE_POS) & 0x1) != 0)
        {
            SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
            return (stat);
        }

        /* start AEs */
        SET_CAP_CSR (handle, FCU_CONTROL, FCU_CTRL_CMD_START);
    
        while (retry < FW_AUTH_MAX_RETRY)
        {
            fcuSts = GET_CAP_CSR (handle, FCU_STATUS);
            if (((fcuSts & FCU_AUTH_STS_MASK) == FCU_STS_EXECUTING) ||
                (((fcuSts >> FCU_STS_DONE_POS) & 0x1) != 0))
            {
                break;
            }
            else
            {
                retry++;
                osalSleep (FW_AUTH_WAIT_PERIOD);
            }
        }
                
        if (retry >= FW_AUTH_MAX_RETRY)
        {
            ERRINFO(("halAe_Start error (AE 0x%x FCU_STATUS = 0x%x)\n",
                    ae,
                    fcuSts));
            stat = HALAE_FAIL;
        }

        SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
    }
    else
    {
        SPIN_LOCK_AE(handle, ae);    
        /* disable the other ctx */
        putCtxWakeupEvents(handle, ae, 
            (~ctxEnMask) & AE_ALL_CTX, WAKEUP_EVENTS_SLEEP);
        stat = enableCtx(handle, ae, ctxEnMask);
        SPIN_UNLOCK_AE(handle, ae);
    }

    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
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
 * @param ctxEnMask - IN Specifies the contexts to be enabled
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB A bad debug library is associated with the specified
                        accelaration engine the library is not initialized
 * @retval HALAE_BADARG Bad function argument
 * 
 *****************************************************************************/
int 
halAe_StartAuthenticatedME (icp_firml_handle_t *handle, 
            unsigned char ae)
{
    int stat = HALAE_SUCCESS, retry = 0;
    unsigned int fcuSts = 0;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);

    /* issue fw_start command */
    SET_CAP_CSR (handle, FCU_CONTROL, 0x2);

    /* check authentication is successful or not */
    while (retry < FW_AUTH_MAX_RETRY)
    {
        fcuSts = GET_CAP_CSR (handle, FCU_STATUS);
        

        if ((fcuSts & 0x7) == 0x6)
        {
            break;
        }
        else
        {
            retry++;
            osalSleep (FW_AUTH_WAIT_PERIOD);
        }
    }
    
    if (retry >= FW_AUTH_MAX_RETRY)
    {
        ERRINFO(("halAe_StartAuthenticatedME error (FCU_STATUS = 0x%x)\n", 
                fcuSts & 0x7));
        stat =  HALAE_FAIL;
    }

    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Stops the accelaration engine contexts that have a corresponding bit 
 *      set in the ctxMask parameter at the next context arbitration 
 *      instruction. The context may not stop because it never executes a 
 *      context arbitration instruction. A value of HALAE_RESET is returned 
 *      if the accelaration engine is in reset.
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
 * @param ctxEnMask - IN Specifies the contexts to stop
 *
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/
int 
halAe_Stop(icp_firml_handle_t *handle, 
           unsigned char ae, 
           unsigned int ctxMask)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    if (handle->sysMemInfo.fwAuth != 1)
    {
        stat = disableCtx(handle, ae, ctxMask);
    }
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a 32-bit value to a context(s) indirect CSR.
 *
 * @param
 *
 * @retval HALAE_SUCCESS,HALAE_BADARG,HALAE_BADLIB
 * 
 * 
 *****************************************************************************/
static int 
putCtxIndrCsr(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned int ctxMask, 
              unsigned int aeCsr, 
              unsigned int csrVal)
{
    unsigned int ctx,  ctxPtr;

    switch(aeCsr)
    {
    case CTX_FUTURE_COUNT_INDIRECT:
    case FUTURE_COUNT_SIGNAL_INDIRECT:
    case CTX_WAKEUP_EVENTS_INDIRECT:
    case CTX_STS_INDIRECT:
    case CTX_SIG_EVENTS_INDIRECT:
    case LM_ADDR_0_INDIRECT:
    case LM_ADDR_1_INDIRECT:
    case INDIRECT_LM_ADDR_0_BYTE_INDEX:
    case INDIRECT_LM_ADDR_1_BYTE_INDEX:
            break;
    default: 
        ERRINFO(("aeCsr=0x%x\n", aeCsr)); 
        return (HALAE_BADARG);
    }

    getAeCsr(handle, ae, CSR_CTX_POINTER, &ctxPtr);    /* save the ctx ptr */
    for(ctx = 0; ctx < MAX_CTX; ctx++)
    {
        if(ctxMask & setMasks[ctx])
        {
            putAeCsr(handle, ae, CSR_CTX_POINTER, ctx);
            putAeCsr(handle, ae, aeCsr, csrVal);
        }
    }
    putAeCsr(handle, ae, CSR_CTX_POINTER, ctxPtr);    /* restore ctx ptr */
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a 32-bit value to the accelaration-engine context(s) indirect CSR. 
 *      It's unsafe to call this function while the AE is enabled.
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
 * @param ctxMask - IN Specifies the contexts CSR to be written
 * @param aeCsr - IN The indirect CSR to be written. Must be one of the
 *                following:
 *                1. CTX_FUTURE_COUNT_INDIRECT
 *                2. CTX_WAKEUP_EVENTS_INDIRECT
 *                3. CTX_STS_INDIRECT
 *                4. CTX_SIG_EVENTS_INDIRECT
 *                5. LM_ADDR_0_INDIRECT
 *                6. LM_ADDR_1_INDIRECT
 * @param csrVal - IN Specifies the longword value to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/
int 
halAe_PutCtxIndrCsr(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned int ctxMask, 
                    unsigned int aeCsr, 
                    unsigned int csrVal)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);

    if(isAeActive(handle, ae)) 
    {
       stat = HALAE_AEACTIVE;
    }    
    else
    {
       stat = putCtxIndrCsr(handle, ae, ctxMask, aeCsr, csrVal);
    }   
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
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
 * @retval 
 * 
 * 
 *****************************************************************************/
int 
getCtxIndrCsr_Common(icp_firml_handle_t *handle, 
                     unsigned char ae, 
                     unsigned char ctx, 
                     unsigned int aeCsr, 
                     unsigned int *csrVal)
{
    unsigned int ctxPtr;
    int status = HALAE_SUCCESS;

    switch(aeCsr)
    {
    case CTX_FUTURE_COUNT_INDIRECT:
    case FUTURE_COUNT_SIGNAL_INDIRECT:
    case CTX_STS_INDIRECT:
            break;
    case CTX_WAKEUP_EVENTS_INDIRECT:
    case CTX_SIG_EVENTS_INDIRECT:
            break;
    case LM_ADDR_0_INDIRECT:
    case LM_ADDR_1_INDIRECT:
            break;
    case INDIRECT_LM_ADDR_0_BYTE_INDEX:
    case INDIRECT_LM_ADDR_1_BYTE_INDEX:
            break;
    default: 
        ERRINFO(("aeCsr=0x%x\n", aeCsr)); 
        return (HALAE_BADARG);
    }

    getAeCsr(handle, ae, CSR_CTX_POINTER, &ctxPtr);    /* save the ctx ptr */
    if((ctxPtr & CCP_CONTEXT) != (ctx & CCP_CONTEXT))
    {
        putAeCsr(handle, ae, CSR_CTX_POINTER, ctx);
    }
    status = getAeCsr(handle, ae, aeCsr, csrVal);

    if((ctxPtr & CCP_CONTEXT) != (ctx & CCP_CONTEXT))
    {
        putAeCsr(handle, ae, CSR_CTX_POINTER, ctxPtr);    /* restore ctx ptr */
    }    

    return (status);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a 32-bit value to the accelaration-engine context(s) indirect CSR. 
 *      It's unsafe to call this function while the AE is enabled.
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
 * @param ctxMask - IN Specifies the contexts CSR to be written
 * @param aeCsr - IN The indirect CSR to be written. Must be one of the
 *                following:
 *                1. CTX_FUTURE_COUNT_INDIRECT
 *                2. CTX_WAKEUP_EVENTS_INDIRECT
 *                3. CTX_STS_INDIRECT
 *                4. CTX_SIG_EVENTS_INDIRECT
 *                5. LM_ADDR_0_INDIRECT
 *                6. LM_ADDR_1_INDIRECT
 * @param csrVal - IN Specifies the longword value to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/
int 
halAe_GetCtxIndrCsr(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned char ctx, 
                    unsigned int aeCsr, 
                    unsigned int *csrVal)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    VERIFY_ARGPTR(csrVal);
    VERIFY_CTX(ctx);
    SPIN_LOCK_AE(handle, ae);
    if(isAeActive(handle, ae)) 
    {
       stat = HALAE_AEACTIVE;
    }   
    else
    {
       stat = getCtxIndrCsr(handle, ae, ctx, aeCsr, csrVal);
    }   
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the context-status CSR of the contexts specified by the ctxMask 
 *      parameter.The accelaration engine must be stopped before calling this function.
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
 * @param ae - IN Specifies the accelaration engine to write to
 * @param ctxMask - IN Specifies the context or contexts to write to.
 * @param ctxStatus - IN Specifies the longword value to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/
int 
halAe_PutCtxStatus(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned int ctxMask, 
                   unsigned int ctxStatus)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);

    if(isAeActive(handle, ae))
    {
       stat = HALAE_AEACTIVE;
    }   
    else
    {
       stat = putCtxIndrCsr(handle, ae, ctxMask, CTX_STS_INDIRECT, ctxStatus);
    }   
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Sets the indicated accelaration engine context program counters to the value 
 *      specified by the upc parameter.
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
 * @param ctxMask - IN Specifies the accelaration engine context or contexts
 * @param upc - IN The new program counter value
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/
int 
halAe_PutPC(icp_firml_handle_t *handle, 
            unsigned char ae, 
            unsigned int ctxMask, 
            unsigned int upc)
{
    VERIFY_ARGPTR(handle);
    return (halAe_PutCtxStatus(handle, ae, ctxMask,
                               handle->halHandle->UpcMask & upc));
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the context-status CSR for the context specified by the ctx 
 *      parameter.The accelaration engine must be stopped before calling this function.
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
 * @param ae - IN Specifies the accelaration engine to write to
 * @param ctxMask - IN Specifies the context or contexts to write to.
 * @param ctxStatus - OUT A pointer to the location at which to return the 
 *                        longword value read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/
int 
halAe_GetCtxStatus(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                       unsigned char ctx, 
                       unsigned int *ctxStatus)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    VERIFY_ARGPTR(ctxStatus);
    VERIFY_CTX(ctx);
    SPIN_LOCK_AE(handle, ae);
    if(isAeActive(handle, ae))
    {
       stat = HALAE_AEACTIVE;
    }   
    else
    {
       stat = getCtxIndrCsr(handle, ae, ctx, CTX_STS_INDIRECT, ctxStatus);
    }   
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Returns the program counter for the specified accelaration engine context
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
 * @param ctx - IN Specifies the accelaration engine context of interest
 * @param upc - OUT A pointer to the location of the requested program 
 *                  counter value
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * 
 * 
 *****************************************************************************/
int 
halAe_GetPC(icp_firml_handle_t *handle, 
            unsigned char ae, 
            unsigned char ctx, 
            unsigned int *upc)
{
    unsigned int ctxStatus;
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    VERIFY_ARGPTR(upc);
    VERIFY_CTX(ctx);

    if((stat = halAe_GetCtxStatus(handle, ae, ctx, &ctxStatus)))
    {
        *upc = INVALID_PC;
    }    
    else
    {
        *upc = (ctxStatus & handle->halHandle->UpcMask) >> ICS_CTX_PC_BITPOS;
    }
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Get the state of the sig-event signals.
 *
 * @param ae - IN 
 * @param ctx - IN 
 * @param events - OUT
 *
 * @retval HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
static int 
getCtxSigEvents(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned char ctx, 
                unsigned int *events)
{
    unsigned int ctxPtr;

    getAeCsr(handle, ae, CSR_CTX_POINTER, &ctxPtr); /* save the ctx ptr */
    if((ctxPtr & CCP_CONTEXT) != (ctx & CCP_CONTEXT))
    {
        putAeCsr(handle, ae, CSR_CTX_POINTER, ctx);
    }
    getAeCsr(handle, ae, CTX_SIG_EVENTS_INDIRECT, events);

    if((ctxPtr & CCP_CONTEXT) != (ctx & CCP_CONTEXT))
    {
        putAeCsr(handle, ae, CSR_CTX_POINTER, ctxPtr); /* restore ctx ptr */
    }    

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the signal events for the specified accelaration engine context
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
 * @param ctx - IN Specifies the accelaration engine context of interest.
 * @param events - IN A pointer to the location of the current signal events 
 *                    for the specified accelaration engine context
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_GetCtxSigEvents(icp_firml_handle_t *handle, 
                      unsigned char ae, 
                      unsigned char ctx, 
                      unsigned int *events)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    VERIFY_ARGPTR(events);
    VERIFY_CTX(ctx);
    SPIN_LOCK_AE(handle, ae);
    if(isAeActive(handle, ae))
    {
       stat = HALAE_AEACTIVE;
    }   
    else
    { 
       stat = getCtxSigEvents(handle, ae, ctx, events);
    }   
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write the state of the sig-event signals.
 *
 * @param 
 * @param 
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB
 * 
 * 
 *****************************************************************************/
static int 
putCtxSigEvents(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int ctxMask, 
                unsigned int events)
{
    unsigned int ctx, ctxPtr;

    getAeCsr(handle, ae, CSR_CTX_POINTER, &ctxPtr);    /* save the ctx ptr */
    for(ctx = 0; ctx < MAX_CTX; ctx++)
    {
        /* save signaled events */
        if(ctxMask & setMasks[ctx])
        {
            putAeCsr(handle, ae, CSR_CTX_POINTER, ctx);
            putAeCsr(handle, ae, CTX_SIG_EVENTS_INDIRECT, events);
        }
    }
    putAeCsr(handle, ae, CSR_CTX_POINTER, ctxPtr);    /* restore ctx ptr */

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the longword value of the event parameter to the signal events 
 *      of the specified accelaration engine contexts.
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
 * @param ctxMask - IN Specifies the accelaration engine context or contexts
 * @param events - IN The longword specifying the new signal events
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_PutCtxSigEvents(icp_firml_handle_t *handle, 
                      unsigned char ae, 
                      unsigned int ctxMask, 
                      unsigned int events)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    if(isAeActive(handle, ae))
    {
       stat = HALAE_AEACTIVE;
    }   
    else 
    {
       stat = putCtxSigEvents(handle, ae, ctxMask, events);
    }   
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read the state of the wakeup-event signals.
 *
 * @param ae - IN 
 * @param ctx - IN 
 * @param events - OUT
 *
 * @retval HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
static int 
getCtxWakeupEvents(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned char ctx, 
                   unsigned int *events)
{
    unsigned int ctxPtr;

    getAeCsr(handle, ae, CSR_CTX_POINTER, &ctxPtr);    /* save the ctx ptr */
    if((ctxPtr & CCP_CONTEXT) != (ctx & CCP_CONTEXT)) 
    {
        putAeCsr(handle, ae, CSR_CTX_POINTER, ctx);
    }
    getAeCsr(handle, ae, CTX_WAKEUP_EVENTS_INDIRECT, events);

    if((ctxPtr & CCP_CONTEXT) != (ctx & CCP_CONTEXT))
    {
        putAeCsr(handle, ae, CSR_CTX_POINTER, ctxPtr);    /* restore ctx ptr */
    }    

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the state of the accelaration engine context wakeup-event signals.
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
 * @param ctx - IN Specifies the accelaration engine context of interest
 * @param events - OUT A pointer to the location of the wakeup-event signals 
 *                     for the specified acceleration engines and context
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_GetCtxWakeupEvents(icp_firml_handle_t *handle, 
                         unsigned char ae, 
                         unsigned char ctx, 
                         unsigned int *events)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(events);
    VERIFY_AE(handle, ae);
    VERIFY_CTX(ctx);
    SPIN_LOCK_AE(handle, ae);
    if(isAeActive(handle, ae))
    {
       stat = HALAE_AEACTIVE;
    }   
    else
    {
       stat = getCtxWakeupEvents(handle, ae, ctx, events);
    }   
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write the state of the wakeup-event signals.
 *
 * @param 
 * @param 
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB
 * 
 * 
 *****************************************************************************/
static int 
putCtxWakeupEvents(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned int ctxMask, 
                   unsigned int events)
{
    unsigned int ctx=0, ctxPtr=0;

    getAeCsr(handle, ae, CSR_CTX_POINTER, &ctxPtr);    /* save the ctx ptr */
    for(ctx = 0; ctx < MAX_CTX; ctx++)
    {
        if(ctxMask & setMasks[ctx])
        {
            putAeCsr(handle, ae, CSR_CTX_POINTER, ctx);
            putAeCsr(handle, ae, CTX_WAKEUP_EVENTS_INDIRECT, events);
        }
    }
    putAeCsr(handle, ae, CSR_CTX_POINTER, ctxPtr);    /* restore ctx ptr */

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes the longword events to the accelaration engine context wakeup-events 
 *      CSR. Only the context with a corresponding bit set in ctxMask are 
 *      written; unspecified contexts remains unchanged.
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
 * @param ctx - IN Specifies the accelaration engine context of interest
 * @param events - OUT The longword specifying the new wakeup-event signals
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_PutCtxWakeupEvents(icp_firml_handle_t *handle, 
                         unsigned char ae, 
                         unsigned int ctxMask, 
                         unsigned int events)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    if(isAeActive(handle, ae))
    {
       stat = HALAE_AEACTIVE;
    }   
    else
    {
       stat = putCtxWakeupEvents(handle, ae, ctxMask, events);
    }   
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Determine if the AE ctx is enabled -- enable to run, or  is running.
 *               enabled = ready, sleep, executing
 *                                           inactive
 *
 * @param ae - IN
 *
 * @retval  HALAE_ENABLED - at least, 
 *                          one of ctx in the AE is ready, sleep, or executing
 *          HALAE_DISABLED - all of ctx in the AE is inactive 
 *                           (inactive = ctx_arb[bpt], ustore or reg error)
 *          HALAE_AEACTIVE - at least, one of ctx in the AE is executing
 *          HALAE_BADLIB,HALAE_RESET
 * 
 * 
 *****************************************************************************/
int 
isAeEnabled(icp_firml_handle_t *handle, 
            unsigned char ae)
{
   unsigned int csr=0;
   int ret = HALAE_SUCCESS;

   ret = getAeCsr(handle, ae, CTX_ENABLES, &csr);
   if((ret != HALAE_SUCCESS) || (csr & CE_ENABLE))
   {
      return (HALAE_ENABLED);
   }   

   getAeCsr(handle, ae, ACTIVE_CTX_STATUS, &csr);
   if((csr & (1 << ACS_ABO_BITPOS))) 
   {
      return (HALAE_AEACTIVE);
   }    

   return (HALAE_DISABLED);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Determine if any of the AE ctx is enabled or is running.
 *
 * @param ae - IN
 *
 * @retval   HALAE_ENABLED,HALAE_DISABLED,HALAE_BADLIB,HALAE_RESET,
 *               HALAE_AEACTIVE
 * 
 *****************************************************************************/
int 
halAe_IsAeEnabled(icp_firml_handle_t *handle, 
                  unsigned char ae)
{
   int stat = HALAE_SUCCESS;

   VERIFY_ARGPTR(handle);
   HALAE_VERIFY_LIB(handle);
   VERIFY_AE(handle, ae);
   SPIN_LOCK_AE(handle, ae);
   if (handle->sysMemInfo.fwAuth == 0)
   {
       stat = isAeEnabled(handle, ae);
   }
   else
   {
       stat = HALAE_FAIL;
   }
   SPIN_UNLOCK_AE(handle, ae);
   return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Determine the ecc and return the uword with the appropriates
 *               bits set.
 * @param uword - IN
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
uword_T 
setUwordECC(uword_T uword)
{
    uword_T chkBit0Mask=0xff800007fffULL, chkBit1Mask=0x1f801ff801fULL,
            chkBit2Mask=0xe387e0781e1ULL, chkBit3Mask=0x7cb8e388e22ULL,
            chkBit4Mask=0xaf5b2c93244ULL, chkBit5Mask=0xf56d5525488ULL,
            chkBit6Mask=0xdaf69a46910ULL;

    uword &= ~(0x7fULL << UWORD_ECC_BIT_0);    /* clear the ecc bits */
    uword |= (uword_T)bitParity64(chkBit0Mask & uword) << UWORD_ECC_BIT_0;
    uword |= (uword_T)bitParity64(chkBit1Mask & uword) << UWORD_ECC_BIT_1;
    uword |= (uword_T)bitParity64(chkBit2Mask & uword) << UWORD_ECC_BIT_2;
    uword |= (uword_T)bitParity64(chkBit3Mask & uword) << UWORD_ECC_BIT_3;
    uword |= (uword_T)bitParity64(chkBit4Mask & uword) << UWORD_ECC_BIT_4;
    uword |= (uword_T)bitParity64(chkBit5Mask & uword) << UWORD_ECC_BIT_5;
    uword |= (uword_T)bitParity64(chkBit6Mask & uword) << UWORD_ECC_BIT_6;

    return (uword);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a number of long-words from the reloadable ustore dram 
 *               buffer. The micro engine must be inactive, possible under reset, 
 *               before writing to micro-store.
 *
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN
 * @param uWord - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
 *               HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
static int 
getReloadUwords(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int uAddr, 
                unsigned int numWords, 
                uword_T *uWord)
{
    int status = HALAE_SUCCESS;
    unsigned int ii=0;

    if(!uWord || (uAddr + numWords) > KILO_128) 
    {
        ERRINFO(("uWord=0x%p, numWords=0x%x\n",
                                (unsigned int *)uWord,numWords)); 
        return (HALAE_BADARG);
    }    
    if((status = isAeEnabled(handle, ae)) != HALAE_DISABLED) 
    {
        return (status);
    }    

    for(ii = 0; ii < numWords; ii++)
    {
        unsigned int wrdHi, wrdLo;
        wrdLo = DRAM_READ_CH0(handle, \
                     AE(handle, ae).ustoreDramAddr + uAddr);
        wrdHi = DRAM_READ_CH0(handle, \
                AE(handle, ae).ustoreDramAddr + uAddr + sizeof(unsigned int));
        uWord[ii] = (uword_T)wrdHi << DWORD_SIZE_IN_BIT | wrdLo; 
        uAddr +=sizeof(uword_T);
    }
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a number of long-words to the reloadable ustore dram 
 *               buffer.The micro engine must be inactive, possible under reset, 
 *               before writing to micro-store.
 *
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN
 * @param uWord - IN
 *
 * @retval   HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
 *               HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
static int
putReloadUwords(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int uAddr, 
                unsigned int numWords, 
                uword_T *uWord)
{
    int status = HALAE_SUCCESS;
    unsigned int ii=0;
    uword_T uwrdEcc;

    if(!uWord || (uAddr + numWords) > KILO_128) 
    {
        ERRINFO(("uWord=0x%p, numWords=0x%x\n",
                                (unsigned int *)uWord,numWords)); 
        return (HALAE_BADARG);
    }    
    if((status = isAeEnabled(handle, ae)) != HALAE_DISABLED)
    {
         return (status);
    }     

    for(ii = 0; ii < numWords; ii++)
    {
        uwrdEcc = setUwordECC(uWord[ii]);
        DRAM_WRITE_CH0(handle, AE(handle, ae).ustoreDramAddr + uAddr, \
                       (unsigned int)(uwrdEcc & 0xffffffff));
        DRAM_WRITE_CH0(handle, \
             (AE(handle, ae).ustoreDramAddr + uAddr + sizeof(unsigned int)), \
             (unsigned int)(uwrdEcc >> DWORD_SIZE_IN_BIT));
        uAddr += sizeof(uword_T);
    }

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read a number of long-words from micro-store. The accelEngine 
 *               must be disabled before reading from micro-store.
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN 
 * @param uWord - OUT 
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
 *               HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
getUwords(icp_firml_handle_t *handle, 
          unsigned char ae, 
          unsigned int uAddr, 
          unsigned int numWords, 
          uword_T *uWord)
{
    int status = HALAE_SUCCESS;
    unsigned int ii=0, uwrdLo=0, uwrdHi=0, ustoreAddr, miscControl;
    unsigned int shCtlStoreFlag;
    unsigned char aeNeigh;

    if((status = isAeEnabled(handle, ae)) != HALAE_DISABLED) 
    {
        return (status);
    }    
    /* determine whether it neighbour AE runs in shared control store status */
    miscControl = GET_AE_CSR(handle, ae, AE_MISC_CONTROL);
    shCtlStoreFlag = miscControl & (0x1 << MMC_SHARE_CS_BITPOS);
    if(shCtlStoreFlag) 
    {
       halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
       if(((1<<aeNeigh) & handle->sysMemInfo.aeMask) 
          && isAeActive(handle, aeNeigh)) 
       {
             return (HALAE_NEIGHAEACTIVE); 
       }      
    }    

    /* if reloadable, then get it all from dram-ustore */
    if((miscControl >> MMC_CS_RELOAD_BITPOS) & 0x3)
    {
        return (getReloadUwords(handle, ae, uAddr, numWords, uWord));
    }
    
    /* disable SHARE_CS bit to workaround silicon bug */
    SET_AE_CSR(handle, ae, AE_MISC_CONTROL, miscControl & 0xfffffffb);
    halAe_WaitAeLocalCsrReady(handle, ae);

    /* get it from phy-ustore */
    if(!uWord || (uAddr + numWords) > GET_USTORE_SIZE(handle, ae)) 
    {
        ERRINFO(("uWord=0x%p, numWords=0x%x\n",
                                (unsigned int *)uWord,numWords)); 
        return (HALAE_BADARG);
    }    
    /* save ustore-addr csr */
    ustoreAddr = GET_AE_CSR(handle, ae, USTORE_ADDRESS);            

    uAddr |= UA_ECS;        /* enable ecs bit */
    for(ii = 0; ii < numWords; ii++)
    {
        SET_AE_CSR(handle, ae, USTORE_ADDRESS, uAddr);
        /* delay several cycless till USTORE_ADDRESS actually updated */
        halAe_WaitAeLocalCsrReady(handle, ae);           

        uAddr++;
        uwrdLo = GET_AE_CSR(handle, ae, USTORE_DATA_LOWER);
        uwrdHi = GET_AE_CSR(handle, ae, USTORE_DATA_UPPER);
        uWord[ii] = uwrdHi;
        uWord[ii] = (uWord[ii] << DWORD_SIZE_IN_BIT) | uwrdLo;
    }
    
    /* restore SHARE_CS bit to workaround silicon bug */
    SET_AE_CSR(handle, ae, AE_MISC_CONTROL, miscControl);
    halAe_WaitAeLocalCsrReady(handle, ae);

    SET_AE_CSR(handle, ae, USTORE_ADDRESS, ustoreAddr);
    halAe_WaitAeLocalCsrReady(handle, ae);           

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads and returns the number of microword from the specified 
 *      accelaration engine microstore. acceleration engines must be disabled before reading 
 *      from the microstore.
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
 * @param uWordAddr - IN The microstore address specifying the start of the 
 *                       write operation
 * @param numWords - IN The number of microwords to write
 * @param uWord - OUT A pointer to the location of the requested microwords
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/
int 
halAe_GetUwords(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int uAddr, 
                unsigned int numWords, 
                uword_T *uWord)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(uWord);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    stat = getUwords(handle, ae, uAddr, numWords, uWord);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a number of long-words to micro-store.  The micro engine 
 *               must be inactive, possible under reset, before writing to 
 *               micro-store.
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN 
 * @param uWord - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
 *               HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
putUwords(icp_firml_handle_t *handle, 
          unsigned char ae, 
          unsigned int uAddr, 
          unsigned int numWords, 
          uword_T *uWord)
{
    int status = HALAE_SUCCESS;
    unsigned int ii=0, uwrdLo=0, uwrdHi=0, ustoreAddr, miscControl;
    unsigned int csReload, fixedSize=0;
    uword_T tmp = 0;
    unsigned int shCtlStoreFlag;
    unsigned char aeNeigh;

    if((status = isAeEnabled(handle, ae)) != HALAE_DISABLED) 
    {
        return (status);
    }    
    /* determine whether it neighbour AE runs in shared control store status */
    miscControl = GET_AE_CSR(handle, ae, AE_MISC_CONTROL);
    shCtlStoreFlag = miscControl & (0x1 << MMC_SHARE_CS_BITPOS);
    if(shCtlStoreFlag) 
    {
       halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
       if(((1<<aeNeigh) & handle->sysMemInfo.aeMask) 
          && isAeActive(handle, aeNeigh)) 
       {
             return (HALAE_NEIGHAEACTIVE); 
       }      
    }

    /* if reloadable, then update the DRAM */
    if((csReload = (miscControl >> MMC_CS_RELOAD_BITPOS) & 0x3))
    {
        if(putReloadUwords(handle, ae, uAddr, numWords, uWord))
        {
           ERRINFO(("putReloadUwords error\n")); 
           return (HALAE_BADARG);
        }   
    }
    else
    {
        if(!uWord || (uAddr + numWords) > GET_USTORE_SIZE(handle, ae))
        {
           ERRINFO(("uWord=0x%p, numWords=0x%x\n",
                                   (unsigned int *)uWord,numWords )); 
           return (HALAE_BADARG);
        }   
    }

    /* update the phy ustore -- the fixed portion */
    switch(csReload)
    {
    case 0x0: fixedSize = KILO_8; break; /* 0 reloadable, 8k fixed */
    case 0x1: fixedSize = KILO_6; break; /* 2k reloadable, 6k fixed */
    case 0x2: fixedSize = KILO_4; break; /* 4k reloadable, 4k fixed */
    case 0x3: fixedSize = 0; break;    /* 8k reloadable, 0 fixed */
    default: return (HALAE_BADARG); 
    }

    if(csReload)
    {
        if(uAddr >= fixedSize) 
        {
            /* uaddr beyond the fixed-region */    
            return (HALAE_SUCCESS);                  
        }    
        if((uAddr + numWords) > fixedSize) 
        {
            /* write only within the fixed-region */
            numWords = fixedSize - uAddr;           
        }    
    }

    if(!numWords)
    {
        return (HALAE_SUCCESS);
    }    
    ustoreAddr = GET_AE_CSR(handle, ae, 
                      USTORE_ADDRESS); /* save ustore-addr csr */
    uAddr |= UA_ECS;                                /* enable ecs bit */
    SET_AE_CSR(handle, ae, USTORE_ADDRESS, uAddr);      /* set the uaddress */
    halAe_WaitAeLocalCsrReady(handle, ae);
    for(ii = 0; ii < numWords; ii++)
    {
      if ((handle->halHandle->PrdMinType == HWID_ICP) 
          || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP)
          || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_C) 
          || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_B) 
          || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_R)
          || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_RS))
      {
        tmp = setUwordECC(uWord[ii]);
      }  
      uwrdLo = (unsigned int)(tmp & 0xffffffff);
      uwrdHi = (unsigned int)(tmp >> DWORD_SIZE_IN_BIT);

      SET_AE_CSR(handle, ae, USTORE_DATA_LOWER, uwrdLo);
      halAe_WaitAeLocalCsrReady(handle, ae);
      /* this will auto increment the address */
      SET_AE_CSR(handle, ae, USTORE_DATA_UPPER, uwrdHi);    
      halAe_WaitAeLocalCsrReady(handle, ae);
    }
    SET_AE_CSR(handle, ae, USTORE_ADDRESS, ustoreAddr);
    halAe_WaitAeLocalCsrReady(handle, ae);
                        
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *    Write a number of long-words to micro-store.  The micro engine 
 *               must be inactive, possible under reset, before writing to 
 *               micro-store. 
 *
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN
 * @param uWord - IN
 *
 * @retval   HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
                HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_PutUwords(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int uAddr, 
                unsigned int numWords,
                uword_T *uWord)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(uWord);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    stat = putUwords(handle, ae, uAddr, numWords, uWord);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}
 
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *    Write a number of long-words to micro-store memory specified 
 *                by the uAddr word-address such that the even address goes to 
 *             the even numbered AE, and the odd address goes to the odd 
 *             numbered AE.
 *
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN
 * @param uWord - IN
 *
 * @retval   HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
 *              HALAE_RESET, HALAE_FAIL, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
static int 
halAe_putCoalesceUwords(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned int uAddr,
                        unsigned int numWords, 
                        uword_T *uWord)
{
    int stat = HALAE_SUCCESS;
    unsigned int ii, evenCpyCnt=0, oddCpyCnt=0;
    unsigned char aeNeigh, evenAe, oddAe;
    
    uword_T          *evenUwords, *oddUwords;
    if((uAddr + numWords) > GET_USTORE_SIZE(handle, ae)*0x2)
    {
       ERRINFO(("uWord=0x%p, numWords=0x%x\n",
                (unsigned int *)uWord,numWords )); 
       return (HALAE_BADARG);
    }    
    
    /* osalMemAlloc can put the current process to sleep waiting 
     * for a page when called in low-memory situations. 
     * So, it should not be put in spin lock.
     * osalMemAllocAtomic will not put the current process to sleep. 
     * But if the last page does not exist, the allocation fails.
     * Considering above factors, here put osalMemAlloc out of spin lock
     */
    evenUwords = (uword_T *)osalMemAlloc(UBUF_SIZE*sizeof(uword_T));
    if(evenUwords == NULL) 
    {
        ERRINFO(("malloc evenUwords error\n")); 
        return (HALAE_FAIL);
    }    
    oddUwords = (uword_T *)osalMemAlloc(UBUF_SIZE*sizeof(uword_T));
    if(oddUwords == NULL) 
    {
        osalMemFree(evenUwords);
        ERRINFO(("malloc oddUwords error\n")); 
        return (HALAE_FAIL);
    }    

    SPIN_LOCK(handle->halHandle->hal_GlobalLock);
    
    if((stat = isAeEnabled(handle, ae)) != HALAE_DISABLED) 
    {
        osalMemFree(evenUwords);
        osalMemFree(oddUwords);        

        SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
        return (stat);
    }
    halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
    if((stat = isAeEnabled(handle, aeNeigh)) != HALAE_DISABLED) 
    {
        osalMemFree(evenUwords);
        osalMemFree(oddUwords);        

        SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
        return (HALAE_NEIGHAEACTIVE);
    }

    /* establish the even and odd ae */
    if (ae & 0x1)
    {
        oddAe = ae;
        evenAe = aeNeigh;
    }
    else
    {
        oddAe = aeNeigh;
        evenAe = ae;
    }

    /* split into even and odd ae buffers */
    for(ii=0; ii < numWords; ii++)
    {
        if((uAddr + ii) & 1) 
        {
            oddUwords[oddCpyCnt++] = uWord[ii];
        }    
        else 
        {
            evenUwords[evenCpyCnt++] = uWord[ii];
        }    
    }

    /* copy the even ae buffer if necessary */
    if(evenCpyCnt)
    {
        SPIN_LOCK_AE(handle, evenAe);
        if((stat = putUwords(handle, evenAe, 
                             (uAddr+1)/0x2, evenCpyCnt, evenUwords))) 
        {
            osalMemFree(evenUwords);
            osalMemFree(oddUwords);        

            SPIN_UNLOCK_AE(handle, evenAe);
            SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
            return (stat);
        }
        SPIN_UNLOCK_AE(handle, evenAe);
    }

    /* copy the odd ae buffer if necessary */
    if(oddCpyCnt)
    {
        SPIN_LOCK_AE(handle, oddAe);
        if((stat = putUwords(handle, oddAe, uAddr/0x2, oddCpyCnt, oddUwords)))
        {
            osalMemFree(evenUwords);
            osalMemFree(oddUwords);        
   
            SPIN_UNLOCK_AE(handle, oddAe);
            SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
            return (stat);
        }
        SPIN_UNLOCK_AE(handle, oddAe);
    }

    SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
    
    osalMemFree(evenUwords);
    osalMemFree(oddUwords);        

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a number of long-words to micro-store memory specified such that
 *      the even address goes to the even numbered AE, and the odd address 
 *      goes to the odd numbered AE.
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
 * @param uAddr - IN Specifies the microstore address to start of the 
 *                   write operation
 * @param numWords - IN Specifies The number of microwords to write
 * @param uWord - IN A pointer to the location of the microwords to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset 
 * 
 * 
 *****************************************************************************/
int 
halAe_PutCoalesceUwords(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned int uAddr,
                        unsigned int numWords, 
                        uword_T *uWord)
{
    int stat = HALAE_SUCCESS;
    unsigned char aeNeigh;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(uWord);
    VERIFY_AE(handle, ae);
    halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
    VERIFY_AE(handle, aeNeigh);
    stat = halAe_putCoalesceUwords(handle, ae, uAddr, numWords, uWord);
    return (stat);
}
 
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a number of long-words from micro-store memory specified 
 *              by the uAddr word-address such that the even address is taken 
 *              from even numbered AE, and the odd address is taken from the 
 *              odd numbered AE.
 *
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN
 * @param uWord - OUT
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
 *              HALAE_RESET, HALAE_FAIL, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
static int 
halAe_getCoalesceUwords(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned int uAddr,
                        unsigned int numWords, 
                        uword_T *uWord)
{
    unsigned int ii=0, evenCpyCnt, oddCpyCnt, stat, evenCnt, oddCnt;
    unsigned char aeNeigh, evenAe, oddAe;
    
    uword_T          *evenUwords, *oddUwords;
    /* osalMemAlloc can put the current process to sleep waiting 
     * for a page when called in low-memory situations. 
     * So, it should not be put in spin lock.
     * osalMemAllocAtomic will not put the current process to sleep. 
     * But if the last page does not exist, the allocation fails.
     * Considering above factors, here put osalMemAlloc out of spin lock
     */
    evenUwords = (uword_T *)osalMemAlloc(UBUF_SIZE*sizeof(uword_T));
    if(evenUwords == NULL) 
    {
        ERRINFO(("malloc evenUwords error\n")); 
        return (HALAE_FAIL);
    }    
    oddUwords = (uword_T *)osalMemAlloc(UBUF_SIZE*sizeof(uword_T));
    if(oddUwords == NULL) 
    {
        osalMemFree(evenUwords);        
        ERRINFO(("malloc oddUwords error\n ")); 
        return (HALAE_FAIL);
    }    

    SPIN_LOCK(handle->halHandle->hal_GlobalLock);
    if((stat = isAeEnabled(handle, ae)) != HALAE_DISABLED)  
    {
        osalMemFree(evenUwords);
        osalMemFree(oddUwords);        

        SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
        return (stat);
    }
    halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
    if((stat = isAeEnabled(handle, aeNeigh)) != HALAE_DISABLED)  
    {
        osalMemFree(evenUwords);
        osalMemFree(oddUwords);        

        SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
        return (stat);
    }

    /* establish the even and odd ae */
    if (ae & 0x1)
    {
        oddAe = ae;
        evenAe = aeNeigh;
    }
    else
    {
        oddAe = aeNeigh;
        evenAe = ae;
    }

    /* determine how many even and odd words to read */
    evenCpyCnt = numWords/0x2;
    oddCpyCnt = evenCpyCnt;
    if (uAddr & 0x1)
    {
        if(numWords & 0x1)
        {
            oddCpyCnt += 1;  /* read one more odd uword */
        }
    }
    else
    {
        if(numWords & 0x1) 
        {
            evenCpyCnt += 1;  /* read one more even uword */
        }
    }

    /* read the even words if necessary */
    if (evenCpyCnt)
    {
        SPIN_LOCK_AE(handle, evenAe);
        if((stat = getUwords(handle, evenAe, 
                             (uAddr+1)/0x2, evenCpyCnt, evenUwords))) 
        {
            osalMemFree(evenUwords);
            osalMemFree(oddUwords);        

            SPIN_UNLOCK_AE(handle, evenAe);
            SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
            return (stat);
        }
        SPIN_UNLOCK_AE(handle, evenAe);
    }

    /* read the odd words if necessary */
    if (oddCpyCnt)
    {
        SPIN_LOCK_AE(handle, oddAe);
        if((stat = getUwords(handle, oddAe, uAddr/0x2, oddCpyCnt, oddUwords))) 
        {
            osalMemFree(evenUwords);
            osalMemFree(oddUwords);        

            SPIN_UNLOCK_AE(handle, oddAe);
            SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
            return (stat);
        }
        SPIN_UNLOCK_AE(handle, oddAe);
    }

    SPIN_UNLOCK(handle->halHandle->hal_GlobalLock);
    /* merge into one buffer */
    oddCnt = 0;
    evenCnt = 0;
    for(ii=0; ii < numWords; ii++)
    {
        if((uAddr + ii) & 1) 
        {
            uWord[ii] = oddUwords[oddCnt++];
        }    
        else 
        {
            uWord[ii] = evenUwords[evenCnt++];
        }    
    }

    if (oddCnt != oddCpyCnt || evenCnt != evenCpyCnt) 
    {
        osalMemFree(evenUwords);
        osalMemFree(oddUwords);        

        ERRINFO(("oddCnt =0x%x,oddCpyCnt=0x%x, evenCnt=0x%x,evenCpyCnt=0x%x\n", 
                 oddCnt,oddCpyCnt,evenCnt,evenCpyCnt)); 
        return (HALAE_FAIL); /* should not happen */
    }

    osalMemFree(evenUwords);
    osalMemFree(oddUwords);        

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read a number of long-words from micro-store memory such that the 
 *      ven address is taken from even numbered AE, and the odd address is 
 *      taken from the odd numbered AE.
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
 * @param uAddr - IN Specifies the microstore address to start of the 
 *                   write operation
 * @param numWords - IN Specifies The number of microwords to write
 * @param uWord - OUT A pointer to the location of the microwords to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset 
 * 
 * 
 *****************************************************************************/
int 
halAe_GetCoalesceUwords(icp_firml_handle_t *handle, 
                        unsigned char ae, 
                        unsigned int uAddr,
                        unsigned int numWords, 
                        uword_T *uWord)
{
    int stat = HALAE_SUCCESS;
    unsigned char aeNeigh;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(uWord);
    VERIFY_AE(handle, ae);
    halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
    VERIFY_AE(handle, aeNeigh);
    stat = halAe_getCoalesceUwords(handle, ae, uAddr, numWords, uWord);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read a number of long-words from micro-store memory specified 
 *               by the uAddr word-address. The accelEngine must be disabled 
 *               before reading from micro-store.
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN 
 * @param data - OUT 
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
 *               HALAE_RESET, HALAE_FAIL, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
static int 
getUmem(icp_firml_handle_t *handle, 
        unsigned char ae, 
        unsigned int uAddr, 
        unsigned int numWords,
        unsigned int *data)
{
    int status = HALAE_SUCCESS;
    unsigned int ii=0, ustoreAddr, uwrdLo, uwrdHi, miscControl;
    unsigned int shCtlStoreFlag;
    unsigned char aeNeigh;

    if((handle->halHandle->PrdMinType != HWID_ICP) 
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP)
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_C)
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_B)
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_R)
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_RS))
    {
       ERRINFO(("PrdMinType=0x%x\n ", handle->halHandle->PrdMinType)); 
       return (HALAE_FAIL);
    }   
    if(!data || (uAddr + numWords) > GET_USTORE_SIZE(handle, ae)) 
    {
       ERRINFO(("data=0x%p, uAddr =0x%x, numWords=0x%x\n",
                data,uAddr,numWords)); 
       return (HALAE_BADARG);
    }   
    if((status = isAeEnabled(handle, ae)) != HALAE_DISABLED) 
    {
       return (status);
    }   

    /* determine whether it neighbour AE runs in shared control store status */
    miscControl = GET_AE_CSR(handle, ae, AE_MISC_CONTROL);
    shCtlStoreFlag = miscControl & (0x1 << MMC_SHARE_CS_BITPOS);
    if(shCtlStoreFlag) 
    {
       halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
       if(((1<<aeNeigh) & handle->sysMemInfo.aeMask) 
          && isAeActive(handle, aeNeigh)) 
       {
           return (HALAE_NEIGHAEACTIVE); 
       }
    }   

    /* disable SHARE_CS bit to workaround silicon bug */
    SET_AE_CSR(handle, ae, AE_MISC_CONTROL, miscControl & 0xfffffffb);
    halAe_WaitAeLocalCsrReady(handle, ae);

    ustoreAddr = GET_AE_CSR(handle, ae, USTORE_ADDRESS);
    uAddr |= UA_ECS;        /* enable ecs bit */
    for(ii = 0; ii < numWords; ii++)
    {
        SET_AE_CSR(handle, ae, USTORE_ADDRESS, uAddr);
        /* delay several cycless till USTORE_ADDRESS actually updated */
        halAe_WaitAeLocalCsrReady(handle, ae);           

        uAddr++;

        uwrdLo = GET_AE_CSR(handle, ae, USTORE_DATA_LOWER);
        uwrdHi = GET_AE_CSR(handle, ae, USTORE_DATA_UPPER);

        /* Data is stored in ustore data register as: 
            Uword1[9] parity for data bits [31:16], 
            Uword1[8] parity for data bits [15:0]
            Uword1[7:4] 4'b1111, Uword1[3:0] Data[31:28]
            Uword0[31:20] Data[27:16], Uword0[19:18] 2'b11, 
            Uword0[17:10] Data[15:8], Uword0[9:8] 2'b11, Uword0[7:0] Data[7:0]
        */
        data[ii] = ((uwrdHi & 0xf) << BIT_28) | \
                   ((uwrdLo & 0xfff00000) >> BIT_4) | \
                   ((uwrdLo & 0x3fc00) >> BIT_2) | (uwrdLo & 0xff);
    }

    /* restore SHARE_CS bit to workaround silicon bug */
    SET_AE_CSR(handle, ae, AE_MISC_CONTROL, miscControl);
    halAe_WaitAeLocalCsrReady(handle, ae);

    SET_AE_CSR(handle, ae, USTORE_ADDRESS, ustoreAddr);
    halAe_WaitAeLocalCsrReady(handle, ae);           

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      reads a number of longwords to the microstore memory specified by the 
 *      uAddr word-address. The accelaration engine must be inactive before reading 
 *      from microstore.
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
 * @param uWordAddr - IN The microstore address specifying the start of the 
 *                       write operation
 * @param numWords - IN The number of microwords to write
 * @param data - OUT A pointer to the location of the requested longwords
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/
int 
halAe_GetUmem(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned int uAddr, 
              unsigned int numWords,
              unsigned int *data)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(data);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    /* For data, phys==virt for uAddr */
    stat = getUmem(handle, ae, uAddr, numWords, data);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a number of long-words to micro-store memory specified by
 *               the uAddr word-address. The micro engine must be inactive, 
 *               possible under reset, before writing to micro-store.
 * @param ae - IN
 * @param uAddr - IN
 * @param numWords - IN 
 * @param data - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_ENABLED, HALAE_BADARG, 
 *               HALAE_RESET, HALAE_FAIL, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
static int 
putUmem(icp_firml_handle_t *handle, 
        unsigned char ae, 
        unsigned int uAddr, 
        unsigned int numWords,
        unsigned int *data)
{
    int status = HALAE_SUCCESS;
    unsigned int ii=0, ustoreAddr, uwrdLo, uwrdHi, tmp;
    unsigned int csrVal, shCtlStoreFlag;
    unsigned char aeNeigh;

    if((handle->halHandle->PrdMinType != HWID_ICP) 
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP)
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_C)
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_B)
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_R)
       && (handle->halHandle->PrdMinType != HWID_ACCEL_COMP_RS))
    {
       ERRINFO(("PrdMinType=0x%x\n", handle->halHandle->PrdMinType)); 
       return (HALAE_FAIL);
    }   
    if(!data || (uAddr + numWords) > GET_USTORE_SIZE(handle, ae)) 
    {
       ERRINFO(("data=0x%p, uAddr =0x%x, numWords=0x%x\n",
                data,uAddr,numWords)); 
       return (HALAE_BADARG);
    }   
    if((status = isAeEnabled(handle, ae)) != HALAE_DISABLED) 
    {
       return (status);
    }   

    /* determine whether it neighbour AE runs in shared control store status */
    csrVal = GET_AE_CSR(handle, ae, AE_MISC_CONTROL);    
    shCtlStoreFlag = csrVal & (0x1 << MMC_SHARE_CS_BITPOS);
    if(shCtlStoreFlag) 
    {
       halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
       if(((1<<aeNeigh) & handle->sysMemInfo.aeMask) 
          && isAeActive(handle, aeNeigh)) 
       {
           return (HALAE_NEIGHAEACTIVE); 
       }
    }

    ustoreAddr = GET_AE_CSR(handle, ae, 
                     USTORE_ADDRESS); /* save the uaddress */
    uAddr |= UA_ECS;                                /* enable ecs bit */
    SET_AE_CSR(handle, ae, USTORE_ADDRESS, uAddr);    /* set the uaddress */
    halAe_WaitAeLocalCsrReady(handle, ae);
    for(ii = 0; ii < numWords; ii++)
    {
        /*
        Data is stored in ustore data register as: 
            Uword1[9] parity for data bits [31:16], 
            Uword1[8] parity for data bits [15:0]
            Uword1[7:4] 4'b1111, Uword1[3:0] Data[31:28]
            Uword0[31:20] Data[27:16], Uword0[19:18] 2'b11, 
            Uword0[17:10] Data[15:8], Uword0[9:8] 2'b11, Uword0[7:0] Data[7:0]
        */
        uwrdLo =  ((data[ii] & 0xfff0000) << BIT_4) | (0x3 << BIT_18) | \
                   ((data[ii] & 0xff00) << BIT_2) | \
                   (0x3 << BIT_8) | (data[ii] & 0xff);
        uwrdHi = (0xf << BIT_4) | ((data[ii] & 0xf0000000) >> BIT_28);

        /* set parity bits -- even-parity including the parity bit */
        uwrdHi |= (numBitsSet(data[ii] & 0xffff) & 0x1) << BIT_8;
        tmp = ((data[ii] >> WORD_SIZE_IN_BIT) & 0xffff);
        uwrdHi |= (numBitsSet(tmp) & 0x1) << BIT_9;

        SET_AE_CSR(handle, ae, USTORE_DATA_LOWER, uwrdLo);
        halAe_WaitAeLocalCsrReady(handle, ae);
        /* this will auto increment the address */
        SET_AE_CSR(handle, ae, USTORE_DATA_UPPER, uwrdHi);    
        halAe_WaitAeLocalCsrReady(handle, ae);
    }

    SET_AE_CSR(handle, ae, USTORE_ADDRESS, ustoreAddr); /* clear ecs */
    halAe_WaitAeLocalCsrReady(handle, ae);
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      writes a number of longwords to the microstore. The accelaration engine must 
 *      be inactive, possibly under reset, before writing to the microstore.
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
 * @param uWordAddr - IN The microstore address specifying the start of the 
 *                       write operation
 * @param numWords - IN The number of microwords to write
 * @param data - IN A pointer to the location of the longwords to write
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * @retval HALAE_AEACTIVE The accelaration engine was active and the call failed
 * @retval HALAE_RESET Acceleration engine is in reset
 * 
 * 
 *****************************************************************************/
int 
halAe_PutUmem(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned int uAddr, 
              unsigned int numWords,
              unsigned int *data)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(data);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    /* For data, phys==virt for uAddr */
    stat = putUmem(handle, ae, uAddr, numWords, data);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads the accelaration engine context arbitration control register and returns
 *      the value in the ctxArbCtl argument.
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
 * @param ctxArbCtl - OUT A pointer to the location of the specified context 
 *                        arbitration control register
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 *
 * 
 *****************************************************************************/
int 
halAe_GetCtxArb(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int *ctxArbCtl)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(ctxArbCtl);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    stat = getAeCsr(handle, ae, CTX_ARB_CNTL, ctxArbCtl);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword ctxArbCtl value to the accelaration engine context 
 *      arbitration control register for the specified accelaration engine.
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
 * @param ctxArbCtl - IN The new value for the context arbitration 
 *                       control register
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADLIB Bad HAL library which wasn't initialized
 * @retval HALAE_BADARG Bad function argument  
 * 
 * 
 *****************************************************************************/
int 
halAe_PutCtxArb(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned int ctxArbCtl)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    SPIN_LOCK_AE(handle, ae);
    stat = putAeCsr(handle, ae, CTX_ARB_CNTL, ctxArbCtl);
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Execute a list of micro-instructions, in the specified context,
 *              and then restore the state to the previous context. The code
 *              to be executed must perform a ctx_arb in order for this 
 *              function to terminate sucessfully.
 *      Please be noted: when the micsrocode size less than MAX_EXEC_INST, 
 *              this function will save current ustore and restore it. But for 
 *              microcode more than that size, the function will not save it.
 * 
 * @param ae - IN 
 * @param ctx - IN
 * @param microInst - IN
 * @param numInst - IN
 * @param maxCycles - IN
 * @param endPC - IN
 *
 * @retval HALAE_SUCCESS, HALAE_RESET, HALAE_BADLIB
 * 
 * 
 *****************************************************************************/
int 
execMicroInst(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned char ctx, 
              uword_T *microInst,
              unsigned int numInst, 
              int condCodeOff,
              unsigned int maxCycles, 
              unsigned int *endPC)
{
    unsigned int    savCC, wakeupEvents, savCtx, savPc, ctxArbCtl, ctxEnables;
    uword_T            savUwords[MAX_EXEC_INST];
    int                status = HALAE_SUCCESS;
    unsigned int csrVal, newCsrVal, shCtlStoreFlag;
    unsigned char aeNeigh;
    unsigned int indirectLmAddr0_sav, indirectLmAddr1_sav;
    unsigned int indirectLmAddrByte0_sav, indirectLmAddrByte1_sav;
    unsigned int indirectFutureCntSig_sav;
    unsigned int indirectSig_sav, activeSig_sav;

    VERIFY_AE(handle, ae);
    if(IS_RST(handle, ae)) 
    {
       return (HALAE_RESET);
    }   
    if(isAeActive(handle, ae)) 
    {
       return (HALAE_AEACTIVE);
    }   
    if((numInst > (handle->halHandle->MaxUstore - TmpUaddr)) 
       || !microInst 
       || (ctx >= sizeof(setMasks)/sizeof(unsigned int))) 
    {
       ERRINFO(("numInst=%d, microInst=0x%p\n ",
                 numInst,(unsigned int *)microInst)); 
       return (HALAE_BADARG);
    }   

    /* save current LM addr */
    getCtxIndrCsr(handle, ae, ctx, LM_ADDR_0_INDIRECT, &indirectLmAddr0_sav);
    getCtxIndrCsr(handle, ae, ctx, LM_ADDR_1_INDIRECT, &indirectLmAddr1_sav);
    getCtxIndrCsr(handle, ae, ctx, INDIRECT_LM_ADDR_0_BYTE_INDEX, 
                    &indirectLmAddrByte0_sav);
    getCtxIndrCsr(handle, ae, ctx, INDIRECT_LM_ADDR_1_BYTE_INDEX, 
                    &indirectLmAddrByte1_sav);
    
    /* backup shared control store bit, and force AE to 
       none-shared mode before executing ucode snippet */
    csrVal = GET_AE_CSR(handle, ae, AE_MISC_CONTROL);
    shCtlStoreFlag = csrVal & (0x1 << MMC_SHARE_CS_BITPOS);
    if(shCtlStoreFlag) 
    {
       halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
       if(((1<<aeNeigh) & handle->sysMemInfo.aeMask) 
          && isAeActive(handle, aeNeigh)) 
       {
          return (HALAE_NEIGHAEACTIVE); 
       }
    }
    newCsrVal = CLR_BIT(csrVal, MMC_SHARE_CS_BITPOS);
    SET_AE_CSR(handle, ae, AE_MISC_CONTROL, newCsrVal);
    halAe_WaitAeLocalCsrReady(handle, ae);

    /* save current states: */
    if (numInst <= MAX_EXEC_INST) {
        if((status = getUwords(handle, ae, TmpUaddr, numInst, \
                    savUwords)) != HALAE_SUCCESS) {
            SET_AE_CSR(handle, ae, AE_MISC_CONTROL, csrVal);        
            halAe_WaitAeLocalCsrReady(handle, ae);
            return (status);   /* instructions at loc 0 thru numInst */
        }    
    }

    /* save wakeup-events */
    getCtxWakeupEvents(handle, ae, ctx, &wakeupEvents); 
    /* save PC */
    getCtxIndrCsr(handle, ae, ctx, CTX_STS_INDIRECT, &savPc);    
    savPc = (savPc & handle->halHandle->UpcMask) >> ICS_CTX_PC_BITPOS;

    /* save ctx enables */
    ctxEnables = IGNORE_W1C_MASK & GET_AE_CSR(handle, ae, CTX_ENABLES);
    /* save conditional-code */
    savCC = GET_AE_CSR(handle, ae, CC_ENABLE);
    /* save current context */
    savCtx = GET_AE_CSR(handle, ae, ACTIVE_CTX_STATUS);
    ctxArbCtl = GET_AE_CSR(handle, ae, CTX_ARB_CNTL);

    /* save indirect csrs */
    getCtxIndrCsr(handle, ae, ctx, FUTURE_COUNT_SIGNAL_INDIRECT, 
                    &indirectFutureCntSig_sav);
    getCtxIndrCsr(handle, ae, ctx, CTX_SIG_EVENTS_INDIRECT,
                    &indirectSig_sav);
    activeSig_sav = GET_AE_CSR(handle, ae, CTX_SIG_EVENTS_ACTIVE);
    
    /* turn off ucode parity */
    SET_AE_CSR(handle, ae, CTX_ENABLES, ctxEnables & \
              (~(1 << CE_CNTL_STORE_PARITY_ENABLE_BITPOS)));
    halAe_WaitAeLocalCsrReady(handle, ae);

    /* copy instructions to ustore */
    putUwords(handle, ae, TmpUaddr, numInst, microInst);        
    /* set PC */    
    putCtxIndrCsr(handle, ae, setMasks[ctx], 
                    CTX_STS_INDIRECT, handle->halHandle->UpcMask & TmpUaddr);
    /* change the active context */
    SET_AE_CSR(handle, ae, ACTIVE_CTX_STATUS, ctx & ACS_ACNO);
    halAe_WaitAeLocalCsrReady(handle, ae);

    if(condCodeOff) 
    {
        /* disable conditional-code*/        
        SET_AE_CSR(handle, ae, CC_ENABLE, savCC & 0xffffdfff);    
        halAe_WaitAeLocalCsrReady(handle, ae);
    }

    /* wakeup-event voluntary */
    putCtxWakeupEvents(handle, ae, setMasks[ctx], XCWE_VOLUNTARY);
    /* clean signals */
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    CTX_SIG_EVENTS_INDIRECT, 0);
    SET_AE_CSR(handle, ae, CTX_SIG_EVENTS_ACTIVE, 0);
    halAe_WaitAeLocalCsrReady(handle, ae);

    /* enable context */
    enableCtx(handle, ae, setMasks[ctx]);

    /* wait for it to finish */
    if(waitNumCycles(handle, ae, maxCycles, 1) != 0) 
    {
        status = HALAE_FAIL;
    }

    /* see if we need to get the current PC */
    if(endPC)
    {
        unsigned int ctxStatus;
        getCtxIndrCsr(handle, ae, ctx, CTX_STS_INDIRECT, &ctxStatus);
        *endPC = ctxStatus & handle->halHandle->UpcMask;
    }

    /* retore to previous states: */
     /* disable context */
    disableCtx(handle, ae, setMasks[ctx]);                      
    if (numInst <= MAX_EXEC_INST) {
        /* instructions */ 
        putUwords(handle, ae, TmpUaddr, numInst, savUwords);        
    }
    /* wakeup-events */
    putCtxWakeupEvents(handle, ae, setMasks[ctx], wakeupEvents);
    putCtxIndrCsr(handle, ae, setMasks[ctx], CTX_STS_INDIRECT,
                  handle->halHandle->UpcMask & savPc);

    /* only restore shared control store bit, 
       other bit might be changed by AE code snippet */
    csrVal = GET_AE_CSR(handle, ae, AE_MISC_CONTROL);
    if(shCtlStoreFlag) 
    {
       newCsrVal = SET_BIT(csrVal, MMC_SHARE_CS_BITPOS);
    }   
    else 
    {
       newCsrVal = CLR_BIT(csrVal, MMC_SHARE_CS_BITPOS);
    }   
    SET_AE_CSR(handle, ae, AE_MISC_CONTROL, newCsrVal);
    halAe_WaitAeLocalCsrReady(handle, ae);
    /* conditional-code */    
    SET_AE_CSR(handle, ae, CC_ENABLE, savCC);
    halAe_WaitAeLocalCsrReady(handle, ae);
    /* change the active context */    
    SET_AE_CSR(handle, ae, ACTIVE_CTX_STATUS, savCtx & ACS_ACNO);
    halAe_WaitAeLocalCsrReady(handle, ae);
    /* restore the nxt ctx to run */
    SET_AE_CSR(handle, ae, CTX_ARB_CNTL, ctxArbCtl);
    halAe_WaitAeLocalCsrReady(handle, ae);
    /* restore current LM addr */
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    LM_ADDR_0_INDIRECT, indirectLmAddr0_sav);
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    LM_ADDR_1_INDIRECT, indirectLmAddr1_sav);
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    INDIRECT_LM_ADDR_0_BYTE_INDEX, indirectLmAddrByte0_sav);
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    INDIRECT_LM_ADDR_1_BYTE_INDEX, indirectLmAddrByte1_sav);

    /* restore indirect csrs */
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    FUTURE_COUNT_SIGNAL_INDIRECT, indirectFutureCntSig_sav);
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    CTX_SIG_EVENTS_INDIRECT, indirectSig_sav);
    SET_AE_CSR(handle, ae, CTX_SIG_EVENTS_ACTIVE, activeSig_sav);
    halAe_WaitAeLocalCsrReady(handle, ae);
   
    /* ctx-enables */    
    SET_AE_CSR(handle, ae, CTX_ENABLES, ctxEnables); 
    halAe_WaitAeLocalCsrReady(handle, ae);
    
    return (status);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description 
 *        Read a long-word from a GPR, read-xfer, and LM index registers.
 *          It's unsafe to called this function while the AE is enabled.
 *          Type is one of: ICP_GPA_REL, ICP_GPB_REL, ICP_SR_REL, 
 *                               ICP_SR_RD_REL, ICP_DR_REL, ICP_DR_RD_REL, 
 *                               ICP_LMEM0, ICP_LMEM1.
 *          Ctx = 0-7/0-6 (even).
 *     
 *
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum - IN
 * @param data - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG, HALAE_BADLIB, HALAE_RESET
 * 
 * 
 *****************************************************************************/
int 
getRelDataReg_Common(icp_firml_handle_t *handle, 
                     unsigned char ae, 
                     unsigned char ctx, 
                     icp_RegType_T regType,
                     unsigned short regNum, 
                     unsigned int *data)
{
    unsigned int savCtx, uAddr, uwrdLo, uwrdHi;
    unsigned int ctxArbCntl, ctxEnables, ustoreAddr;
    unsigned short    regAddr;
    int status = HALAE_SUCCESS;
    uword_T            inst, savUword;
    unsigned int csrVal, newCsrVal, shCtlStoreFlag;
    unsigned char aeNeigh;
    unsigned short mask;
    
    if ((regType != ICP_GPA_REL) && (regType != ICP_GPB_REL) &&
        (regType != ICP_SR_REL) && (regType != ICP_SR_RD_REL) &&
        (regType != ICP_DR_REL) && (regType != ICP_DR_RD_REL) &&
        (regType != ICP_LMEM0) && (regType != ICP_LMEM1)) 
    {
        ERRINFO(("regType=0x%x\n", regType)); 
        return (HALAE_BADARG);
    }
    if ((regType == ICP_GPA_REL) || (regType == ICP_GPB_REL) ||
        (regType == ICP_SR_REL) || (regType == ICP_SR_RD_REL) ||
        (regType == ICP_DR_REL) || (regType == ICP_DR_RD_REL)) 
    {
        /* determine the context mode */
        getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
        if(CE_INUSE_CONTEXTS & ctxEnables) /* 4-ctx mode */
        {
            if(ctx & 0x1)
            {
                ERRINFO(("bad ctx argument in 4-ctx mode,ctx=0x%x\n",ctx)); 
                return (HALAE_BADARG);
            }
            mask = 0x1f;
        }
        else   /* 8-ctx mode */
        {
            mask = 0x0f;
        }    
        if (regNum & ~mask) 
        {
            return (HALAE_BADARG);
        }  
    }
    if((regAddr = getReg10bitAddr(regType, regNum)) == BAD_REGADDR) 
    {
        ERRINFO(("bad regaddr=0x%x\n",regAddr)); 
        return (HALAE_BADARG);
    }    

    /* instruction -- alu[--, --, B, reg] */
    switch(regType)
    {
    case ICP_GPA_REL:            /* A rel source */
        inst = 0xA070000000ull | (regAddr & 0x3ff);
        break;
    default:
        inst = ((uword_T)0xA030000000ull | ((regAddr & 0x3ff) << BIT_10));
        break;
    }

    /* backup shared control store bit, and force AE to 
    none-shared mode before executing ucode snippet */
    getAeCsr(handle, ae, AE_MISC_CONTROL, &csrVal);
    shCtlStoreFlag = csrVal & (0x1 << MMC_SHARE_CS_BITPOS);
    if(shCtlStoreFlag) 
    {
        halAe_GetSharedUstoreNeigh(handle, ae, &aeNeigh);
        if(((1<<aeNeigh) & handle->sysMemInfo.aeMask) 
           && isAeActive(handle, aeNeigh)) 
        {
            return (HALAE_NEIGHAEACTIVE); 
        }
    }

    newCsrVal = CLR_BIT(csrVal, MMC_SHARE_CS_BITPOS);
    putAeCsr(handle, ae, AE_MISC_CONTROL, newCsrVal);

    /* read current context */
    getAeCsr(handle, ae, ACTIVE_CTX_STATUS, &savCtx);
    getAeCsr(handle, ae, CTX_ARB_CNTL, &ctxArbCntl);

    getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
    /* prevent clearing the W1C bits: the breakpoint bit, 
    ECC error bit, and Parity error bit */
    ctxEnables &= IGNORE_W1C_MASK;            

    /* change the context */
    if(ctx != (savCtx & ACS_ACNO)) 
    {
        putAeCsr(handle, ae, ACTIVE_CTX_STATUS, ctx & ACS_ACNO);
    }
    /* save a ustore location */
    if((status = getUwords(handle, ae, TmpUaddr, 1, &savUword)) 
       != HALAE_SUCCESS)
    {
        /* restore AE_MISC_CONTROL csr */
        putAeCsr(handle, ae, AE_MISC_CONTROL, csrVal);    

        /* restore the context */
        if(ctx != (savCtx & ACS_ACNO)) 
        {
            putAeCsr(handle, ae, ACTIVE_CTX_STATUS, savCtx & ACS_ACNO);
        }    
        putAeCsr(handle, ae, CTX_ARB_CNTL, ctxArbCntl);
        return (status);
    }

    /* turn off ustore parity */
    putAeCsr(handle, ae, CTX_ENABLES, ctxEnables & \
        (~(1 << CE_CNTL_STORE_PARITY_ENABLE_BITPOS)));

    /* save ustore-addr csr */
    getAeCsr(handle, ae, USTORE_ADDRESS, &ustoreAddr);

    /* write the ALU instruction to ustore, enable ecs bit */
    uAddr = TmpUaddr | UA_ECS;                            

    /* set the uaddress */
    putAeCsr(handle, ae, USTORE_ADDRESS, uAddr);                    
    if ((handle->halHandle->PrdMinType == HWID_ICP) 
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP)
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_C)
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_B)
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_R)
        || (handle->halHandle->PrdMinType == HWID_ACCEL_COMP_RS))
    {
        inst = setUwordECC(inst);
    } 

    uwrdLo = (unsigned int)(inst & 0xffffffff);
    uwrdHi = (unsigned int)(inst >> DWORD_SIZE_IN_BIT);


    putAeCsr(handle, ae, USTORE_DATA_LOWER, uwrdLo);

    /* this will auto increment the address */
    putAeCsr(handle, ae, USTORE_DATA_UPPER, uwrdHi);

    /* set the uaddress */
    putAeCsr(handle, ae, USTORE_ADDRESS, uAddr);

    /* delay for at least 8 cycles */
    waitNumCycles(handle, ae, 0x8, 0);

    /* read ALU output -- the instruction should have been executed
    prior to clearing the ECS in putUwords */
    getAeCsr(handle, ae, ALU_OUT, data);                            

    /* restore ustore-addr csr */
    putAeCsr(handle, ae, USTORE_ADDRESS, ustoreAddr);     

    /* restore the ustore */
    status = putUwords(handle, ae, TmpUaddr, 1, &savUword);

    /* restore the context */
    if(ctx != (savCtx & ACS_ACNO)) 
    {
        putAeCsr(handle, ae, ACTIVE_CTX_STATUS, savCtx & ACS_ACNO);
    }    
    putAeCsr(handle, ae, CTX_ARB_CNTL, ctxArbCntl);

    /* restore AE_MISC_CONTROL csr */
    putAeCsr(handle, ae, AE_MISC_CONTROL, csrVal);    

    putAeCsr(handle, ae, CTX_ENABLES, ctxEnables);
    
    return (status);
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
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum - IN
 * @param data - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG, HALAE_BADLIB, HALAE_RESET
 * 
 * 
 *****************************************************************************/
int 
getRelNNReg_Common(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned char ctx, 
                   icp_RegType_T regType,
                   unsigned short regNum,
                   unsigned int *data)
{
    uword_T inst[] = {
        0x0B000000000ull,        /* alu[gpr, --, b, n$reg] */        
        0x0F0000C0300ull,        /* nop */
        0x0E000010000ull         /* ctx_arb[kill] */
    };
    unsigned short    regAddr;
    int stat = HALAE_SUCCESS;
    const int num_inst = sizeof(inst)/sizeof(inst[0]), condCodeOff = 1;
    unsigned int gprB0;
    unsigned int ctxEnables;
    unsigned short mask;

    if(regType != ICP_NEIGH_REL) 
    {
       ERRINFO(("regType=0x%x\n", regType)); 
       return (HALAE_BADARG);
    }   
    /* determine the context mode */
    getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
    if(CE_INUSE_CONTEXTS & ctxEnables) /* 4-ctx mode */
    {
        if(ctx & 0x1)
        {
            ERRINFO(("bad ctx argument in 4-ctx mode,ctx=0x%x\n",ctx)); 
            return (HALAE_BADARG);
        }
        mask = 0x1f;
    }
    else   /* 8-ctx mode */
    {
        mask = 0x0f;
    }    
    if (regNum & ~mask) 
    {
        return (HALAE_BADARG);
    }  

    /* get the 10-bit address of the destination register */
    if((regAddr = getReg10bitAddr(regType, regNum)) == BAD_REGADDR) 
    {
        ERRINFO(("bad regAddr=0x%x\n",regAddr)); 
        return (HALAE_BADARG);
    }   

    /* backup the value of gpr reg */
    getRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, &gprB0);

    /* fixup neighbor csr address */
    inst[0] = ((uword_T)inst[0] | ((regAddr & 0x3ff) << BIT_10));

    stat = execMicroInst(handle, ae, ctx, inst, num_inst, condCodeOff, 
                         num_inst*0x5, NULL);

    /* read the value from gpr */
    getRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, data);

    /* restore the gpr reg */
    putRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, gprB0);

    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word to a GPR or LM index register. This logic 
 *              cannot work for neighbor or transfer registers.
 *              The AE must be disabled prior to calling this function.
 *
 * @param ae - IN
 * @param ctx - IN 
 * @param regType - IN 
 * @param regNum - IN 
 * @param data - IN 
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_ENABLED, 
 *              HALAE_FAIL, HALAE_RESET
 *              Type is one of: ICP_GPA_REL, ICP_GPB_REL, ICP_LMEM0, ICP_LMEM1.
 *              Ctx = 0-7/0, 2, 4, 6.
 * 
 * 
 *****************************************************************************/
int 
putRelDataReg_Common(icp_firml_handle_t *handle, 
                     unsigned char ae, 
                     unsigned char ctx, 
                     icp_RegType_T regType, 
                     unsigned short regNum, 
                     unsigned int data)
{
    unsigned short srcHiAddr, srcLoAddr, destAddr, data16Hi, data16Lo;
    uword_T inst[] = 
    {
        0x0F440000000ull,        /* immed_w1[reg, val_hi16] */
        0x0F040000000ull,        /* immed_w0[reg, val_lo16] */
        0x0F0000C0300ull,        /* nop */
        0x0E000010000ull         /* ctx_arb[kill] */
    };
    const int num_inst = sizeof(inst)/sizeof(inst[0]), condCodeOff = 1;
    const int imm_w1=0, imm_w0=1;
    unsigned int ctxEnables;
    unsigned short mask;

    /* This logic only works for GPRs and LM index registers, 
       not NN or XFER registers! */
    if((regType != ICP_GPA_REL) && (regType != ICP_GPB_REL) &&
        (regType != ICP_LMEM0) && (regType != ICP_LMEM1)) 
    {
        ERRINFO(("regType=0x%x\n", regType)); 
        return (HALAE_BADARG);
    }     

    if ((regType == ICP_GPA_REL) || (regType == ICP_GPB_REL)) 
    {
        /* determine the context mode */
        getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
        if(CE_INUSE_CONTEXTS & ctxEnables) /* 4-ctx mode */
        {
            if(ctx & 0x1)
            {
                ERRINFO(("bad ctx argument in 4-ctx mode,ctx=0x%x\n",ctx)); 
                return (HALAE_BADARG);
            }
            mask = 0x1f;
        }
        else   /* 8-ctx mode */
        {
            mask = 0x0f;
        }    
        if (regNum & ~mask) 
        {
            return (HALAE_BADARG);
        }  
    }   

    /* get the 10-bit address of the destination register */
    if((destAddr = getReg10bitAddr(regType, regNum)) == BAD_REGADDR) 
    {
        ERRINFO(("bad destAddr=0x%x\n",destAddr)); 
        return (HALAE_BADARG);
    }    

    data16Lo = 0xffff & data;
    data16Hi = 0xffff & (data >> WORD_SIZE_IN_BIT);
    srcHiAddr = getReg10bitAddr(ICP_NO_DEST, 
        (unsigned short)(0xff & data16Hi));
    srcLoAddr = getReg10bitAddr(ICP_NO_DEST, 
        (unsigned short)(0xff & data16Lo));

    switch(regType)
    {
    case ICP_GPA_REL:            /* A rel source */
        inst[imm_w1] = inst[imm_w1] | ((data16Hi >> BIT_8) << BIT_20) |
            ((srcHiAddr & 0x3ff) << BIT_10) | (destAddr & 0x3ff);

        inst[imm_w0] = inst[imm_w0] | ((data16Lo >> BIT_8) << BIT_20) | 
            ((srcLoAddr & 0x3ff) << BIT_10) | (destAddr & 0x3ff);
        break;

    default:
        inst[imm_w1] = inst[imm_w1] | ((data16Hi >> BIT_8) << BIT_20) |
            ((destAddr & 0x3ff) << BIT_10) | (srcHiAddr & 0x3ff);

        inst[imm_w0] = inst[imm_w0] | ((data16Lo >> BIT_8) << BIT_20) | 
            ((destAddr & 0x3ff) << BIT_10) | (srcLoAddr & 0x3ff);
        break;
    }

    return (execMicroInst(handle, ae, ctx, inst, num_inst, condCodeOff, 
                          num_inst*0x5, NULL));  
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 *
 * @description
 *     Calculate the number of instruction to init Local memory
 *
 * @param LMByteSize - IN
 *
 * @retval the number of the instruction number
 *
 *****************************************************************************/
int
halAe_GetInstrucNum(int LMByteSize)
{
    int ret, left;

    if (LMByteSize == 0) {
        return(0);
    }

    left = LMByteSize % sizeof(unsigned int);

    if (left) {
        ret = sizeof(inst_1b)/sizeof(uword_T) + \
                     halAe_GetInstrucNum(LMByteSize - left);
    } else {
        /* 3 instruction is needed for further code*/
        ret = (LMByteSize - sizeof(unsigned int))*0x3/0x4 + \
                    sizeof(inst_4b)/sizeof(uword_T);
   }

    return (ret);

}

static int
concatMicroCode(uword_T *microInst,
                unsigned int inst_num,
                unsigned int size,
                unsigned int addr,
                unsigned int *value )
{
    int ii, val_indx;
    unsigned int cur_value;
    uword_T *inst_arr;
    int fixup_offset;
    int usize=0;
    int orig_num;

    if (size == 0) {
        return 0;
    }

    orig_num = inst_num;
    val_indx = 0;
    cur_value = value[val_indx++];
    if (size < 0x4 ) {
       switch(size) {
          case 0x1:
              inst_arr = inst_1b;
              usize = sizeof(inst_1b)/sizeof(inst_1b[0]);
              break;
          case 0x2:
              inst_arr = inst_2b;
              usize = sizeof(inst_2b)/sizeof(inst_2b[0]);
              break;
          case 0x3:
              inst_arr = inst_3b;
              usize = sizeof(inst_3b)/sizeof(inst_3b[0]);
              break;
          default:
              inst_arr = inst_4b;  /* could not reach */
              usize = sizeof(inst_4b)/sizeof(inst_4b[0]);
       }
    } else {
       inst_arr = inst_4b;
       usize = sizeof(inst_4b)/sizeof(inst_4b[0]);
    }

    fixup_offset = inst_num;
    for (ii=0; ii<usize; ii++) {
        microInst[inst_num++] = inst_arr[ii];
    }

    INSERT_IMMED_GPRA_CONST(microInst[fixup_offset], (addr));
    fixup_offset++;
    INSERT_IMMED_GPRA_CONST(microInst[fixup_offset], 0);
    fixup_offset++;
    INSERT_IMMED_GPRB_CONST(microInst[fixup_offset], (cur_value >>  0));
    fixup_offset++;
    INSERT_IMMED_GPRB_CONST(microInst[fixup_offset], \
                    (cur_value >> WORD_SIZE_IN_BIT));

    if (size <= 0x4 ) {
        return (inst_num - orig_num);
    }

    size -= sizeof(unsigned int);
    while (size >= sizeof(unsigned int)) {
       cur_value = value[val_indx++];
       fixup_offset = inst_num;
       microInst[inst_num++] = inst_4b[0x2];
       microInst[inst_num++] = inst_4b[0x3];
       microInst[inst_num++] = inst_4b[0x8];
       INSERT_IMMED_GPRB_CONST(microInst[fixup_offset],\
                     (cur_value >> WORD_SIZE_IN_BIT));
       fixup_offset++;
       INSERT_IMMED_GPRB_CONST(microInst[fixup_offset], (cur_value >>  0));

       addr += sizeof(unsigned int);
       size -= sizeof(unsigned int);
    }
    /* call this function recusive when the left size less than 4 */
    inst_num += concatMicroCode(microInst, inst_num, size, \
                   addr, value+val_indx) ;
    return(inst_num-orig_num);

}

static int
execMicroInitLm(icp_firml_handle_t *handle,
                  unsigned char ae,
                  unsigned char ctx,
                  int *pfirst_exec,
                  uword_T *microInst,
                  unsigned int numInst,
                  unsigned int *pgprA0,
                  unsigned int *pgprA1,
                  unsigned int *pgprA2,
                  unsigned int *pgprB0,
                  unsigned int *pgprB1)

{
    int stat = HALAE_SUCCESS;

    DBGINFO(("Run Micro to initLM: AE%d the total inst is %d\n", ae, numInst));

    if (*pfirst_exec) {
        getRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0, pgprA0);
        getRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0x1, pgprA1);
        getRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0x2, pgprA2);
        getRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, pgprB0);
        getRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0x1, pgprB1);
        *pfirst_exec = 0;
    }
    stat = execMicroInst(handle, ae, ctx, microInst, numInst, 1,
                     numInst*0x5, NULL);
    if (stat != HALAE_SUCCESS) {
        return(HALAE_FAIL);
    }
    return(HALAE_SUCCESS);
}

static int
resotreInitLmGPRs(icp_firml_handle_t *handle,
                  unsigned char ae,
                  unsigned char ctx,
                  unsigned int gprA0,
                  unsigned int gprA1,
                  unsigned int gprA2,
                  unsigned int gprB0,
                  unsigned int gprB1)
{
    putRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0, gprA0);
    putRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0x1, gprA1);
    putRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0x2, gprA2);
    putRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, gprB0);
    putRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0x1, gprB1);
    return (HALAE_SUCCESS);
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
 *****************************************************************************/
int
halAe_BatchPutLM_Common(icp_firml_handle_t *handle, unsigned char ae,
            batch_init_t *lm_init_header)
{
    batch_init_t *plmInit;
    unsigned int addr, *value, size;
    uword_T *microInstArry;
    int microInstNum;
    int alloc_inst_size, this_inst_size;
    int first_exec=1, exec_ed = 0;
    int ctx=0;
    unsigned int gprA0, gprA1, gprA2, gprB0, gprB1;
    int stat = HALAE_SUCCESS;

    if (lm_init_header == NULL) {
        return(HALAE_SUCCESS);
    }

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    if(IS_RST(handle, ae))
    {
        return (HALAE_RESET);
    }

    plmInit = lm_init_header->next;
    /* malloc a space which is enough */
    alloc_inst_size = lm_init_header->size;
    if((unsigned int)alloc_inst_size > 
       (handle->halHandle->MaxUstore - TmpUaddr))
    {
        alloc_inst_size = handle->halHandle->MaxUstore - TmpUaddr;
    }
    microInstArry = (uword_T*)osalMemAlloc(alloc_inst_size*sizeof(uword_T));
    if (microInstArry == NULL) {
         ERRINFO(("Failed to allocate memory\n"));
         return (HALAE_FAIL);
    }
    microInstNum = 0;
    while (plmInit) {
        ae = plmInit->hwAe;
        addr = plmInit->addr;
        value = plmInit->value;
         /* here size > 0 */
        size = plmInit->size;
        DBGINFO(("ae=%d, addr=0x%x, value=0x%x, size=%d\n", \
                           ae, addr, *value, size));
        this_inst_size = halAe_GetInstrucNum(size);
        if (this_inst_size + microInstNum > alloc_inst_size) {
              DBGINFO(("code page is full, call exection unit\n"));
              /* add ctx_arb[kill] */
              microInstArry[microInstNum++] = 0x0E000010000ull;
              exec_ed = 1;
              stat = execMicroInitLm(handle, ae, (unsigned char)ctx, \
                       &first_exec, microInstArry, microInstNum, &gprA0, \
                       &gprA1, &gprA2, &gprB0, &gprB1);
              if (stat != HALAE_SUCCESS ){
                  resotreInitLmGPRs(handle, ae, (unsigned char)ctx, \
                      gprA0, gprA1, gprA2, gprB0, gprB1);
                  osalMemFree(microInstArry);
                  DBGINFO(("AE %d Failed to execute micro code\n", ae ));
                  return(HALAE_FAIL);
              }
               /* run microExec to execute the microcode */
              microInstNum = 0;
        }
        microInstNum += concatMicroCode(microInstArry, \
                      microInstNum, size, addr, value);

        plmInit = plmInit->next;
    }
    /* execMmicro */
    if (microInstArry && (microInstNum > 0)) {
         microInstArry[microInstNum++] = 0x0E000010000ull;
         exec_ed = 1;
         stat = execMicroInitLm(handle, ae, (unsigned char)ctx, \
             &first_exec, microInstArry, microInstNum, &gprA0, &gprA1, \
             &gprA2, &gprB0, &gprB1);
    }
    if (exec_ed) {
        resotreInitLmGPRs(handle, ae, (unsigned char)ctx, gprA0, gprA1,  \
                            gprA2, gprB0, gprB1);
    }

    osalMemFree(microInstArry);
    /* release the micro code allocated */
    return(stat);
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a long-word from read-xfer register. The AE must be 
 *               disabled prior to calling this function.
 *
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum - IN 
 * @param data - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG 
 * 
 * 
 *****************************************************************************/
int 
getRelRdXfer(icp_firml_handle_t *handle, 
             unsigned char ae, 
             unsigned char ctx, 
             icp_RegType_T regType,
             unsigned short regNum, 
             unsigned int *data)
{
    int status = HALAE_SUCCESS;

    status = getRelDataReg(handle, ae, ctx, regType, regNum, data);
    return (status);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a long-word to read-xfer register.The AE must be disabled
 *               prior to calling this function.
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN 
 * @param regNum - IN
 * @param val - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG 
 * 
 * 
 *****************************************************************************/
int 
putRelRdXfer(icp_firml_handle_t *handle, 
             unsigned char ae, 
             unsigned char ctx, 
             icp_RegType_T regType,
             unsigned short regNum, 
             unsigned int val)
{
    int status = HALAE_SUCCESS;
    unsigned int regAddr;
    unsigned int ctx_enables;
    unsigned short mask;
    unsigned short dr_offset = 0x10;

    if ((regType != ICP_SR_REL) && (regType != ICP_DR_REL) &&
        (regType != ICP_SR_RD_REL) && (regType != ICP_DR_RD_REL)) 
    {
        ERRINFO(("regType=0x%x\n", regType)); 
        return (HALAE_BADARG);
    }
    /* determine the context mode */
    status = getAeCsr(handle, ae, CTX_ENABLES, &ctx_enables);
    if(CE_INUSE_CONTEXTS & ctx_enables)
    {
        if(ctx & 0x1)
        {
           ERRINFO(("bad ctx argument in 4-ctx mode,ctx=0x%x\n",ctx)); 
           return (HALAE_BADARG);
        }
        mask = 0x1f;
        dr_offset = 0x20;
    }
    else 
    {
        mask = 0x0f;
    }    
    if (regNum & ~mask) 
    {
        return (HALAE_BADARG);
    }
    regAddr = regNum + (ctx << 0x5);

    switch(regType)
    {
    case ICP_SR_RD_REL:
    case ICP_SR_REL:
        SET_AE_XFER(handle, ae, regAddr, val); break;

    case ICP_DR_RD_REL:
    case ICP_DR_REL:
        SET_AE_XFER(handle, ae, regAddr + dr_offset, val); break;

    default: status = HALAE_BADARG; break;
    }

    return (status);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read a long-word from write-xfer register.  Reading and
 *               writing to/from transfer registers are relative from the
 *                accelEngine's perspective.  Therefore, when the core reads
 *               from the xfer register it's actually reading the write-transfer
 *                registers.  The AE must be disabled prior to
 *                calling this function.
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum - IN 
 * @param data - OUT
 * @param count - IN 
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
int 
getRelWrXfers(icp_firml_handle_t *handle, 
              unsigned char ae, 
              unsigned char ctx, 
              icp_RegType_T regType,
              unsigned short regNum, 
              unsigned int *data, 
              unsigned int count)
{
    int status = HALAE_SUCCESS;
    unsigned int regAddr;
    unsigned int ctx_enables;
    unsigned short regMask;
    unsigned int ii;
    unsigned short dr_offset = 0x10;

    if((regType != ICP_SR_REL) && (regType != ICP_DR_REL) &&
        (regType != ICP_SR_WR_REL) && (regType != ICP_DR_WR_REL)) 
    {
        ERRINFO(("regType=0x%x\n", regType)); 
        return (HALAE_BADARG);
    }    

    /* determine the context mode */
    status = getAeCsr(handle, ae, CTX_ENABLES, &ctx_enables);
    if(CE_INUSE_CONTEXTS & ctx_enables) 
    {
        /* four-ctx mode: 32 reg per ctx */
        if(ctx & 0x1) 
        {
           ERRINFO(("bad ctx argument in 4-ctx mode,ctx=0x%x\n",ctx)); 
           return (HALAE_BADARG);
        }   
        regMask = (unsigned short)~0x1f;
        dr_offset = 0x20;
    } 
    else 
    {
        /* eight-ctx mode: 16 reg per ctx */
        regMask = (unsigned short)~0xf;
    }
    if((regNum & regMask) || ((regNum + count-1) & regMask)) 
    {
        return (HALAE_BADARG);
    }

    switch(regType)
    {
    case ICP_DR_WR_REL:
    case ICP_DR_REL : 
        regAddr = regNum + (ctx << 0x5);
        for (ii=0; ii<count; ii++)
        {
            data[ii] = GET_AE_XFER(handle, ae, regAddr+dr_offset+ii);
        }

        break;

    case ICP_SR_WR_REL:
    case ICP_SR_REL :
        regAddr = regNum + (ctx << 0x5);
        for (ii=0; ii<count; ii++)
        {
            data[ii] = GET_AE_XFER(handle, ae, regAddr+ii);
        }

        break;

    default: status = HALAE_BADARG; break;
    }

    return (status);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write a long-word to a relative write-xfer reg.  This logic
 *               also supports neighbor registers.
 *                The AE must be disabled prior to calling this function.
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN 
 * @param regNum - IN
 * @param data - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_ENABLED, HALAE_FAIL,
 *               HALAE_RESET
 * 
 * 
 *****************************************************************************/
int 
putRelWrXfer_Common(icp_firml_handle_t *handle, 
                    unsigned char ae, 
                    unsigned char ctx, 
                    icp_RegType_T regType,
                    unsigned short regNum, 
                    unsigned int data)
{
    unsigned int gprVal, ctx_enables;
    unsigned short srcHiAddr, srcLoAddr, gprAddr, xfrAddr, data16Hi, data16Lo;
    unsigned short regMask;
    int status = HALAE_SUCCESS;
    uword_T microInst[] = 
    {
        0x0F440000000ull,        /* immed_w1[reg, val_hi16] */
        0x0F040000000ull,        /* immed_w0[reg, val_lo16] */
        0x0A000000000ull,        /* alu[xferReg, --,b, reg] */
        0x0F0000C0300ull,        /* nop */
        0x0E000010000ull         /* ctx_arb[kill] */
    };
    const int num_inst = sizeof(microInst)/sizeof(microInst[0]),condCodeOff=1;
    const unsigned short gprNum = 0, dly=num_inst*0x5;

    if ((regType != ICP_SR_REL) && (regType != ICP_DR_REL) &&
        (regType != ICP_SR_WR_REL) && (regType != ICP_DR_WR_REL) &&
        (regType != ICP_NEIGH_REL)) 
    {
        ERRINFO(("regType=0x%x\n", regType)); 
        return (HALAE_BADARG);
    }    

    getAeCsr(handle, ae, CTX_ENABLES, &ctx_enables);
    if(CE_INUSE_CONTEXTS & ctx_enables) 
    {
        /* four-ctx mode: 32 reg per ctx */
        if(ctx & 0x1)
        {
            ERRINFO(("bad ctx argument in 4-ctx mode,ctx=0x%x\n",ctx)); 
            return (HALAE_BADARG);
        }
        regMask = (unsigned short)~0x1f;
    } 
    else 
    {
        /* eight-ctx mode: 16 reg per ctx */
        regMask = (unsigned short)~0xf;
    }
    if(regNum & regMask) 
    {
        return (HALAE_BADARG);
    }

    /* get the 10-bit address of the destination register */
    if((xfrAddr = getReg10bitAddr(regType, regNum)) == BAD_REGADDR) 
    {
        ERRINFO(("bad xfrAddr=0x%x\n",xfrAddr)); 
        return (HALAE_BADARG);
    }    
    /* backup the value of a b-bank gpr */
    getRelDataReg(handle, ae, ctx, ICP_GPB_REL, gprNum, &gprVal);

    gprAddr = getReg10bitAddr(ICP_GPB_REL, gprNum);

    data16Lo = 0xffff & data;
    data16Hi = 0xffff & (data >> WORD_SIZE_IN_BIT);
    srcHiAddr = getReg10bitAddr(ICP_NO_DEST, 
                                (unsigned short)(0xff & data16Hi));
    srcLoAddr = getReg10bitAddr(ICP_NO_DEST, 
                                (unsigned short)(0xff & data16Lo));

    /* fixup immed_wx[gpr, const] instruction for onstant and register */
    microInst[0] = microInst[0x0] | ((data16Hi >> BIT_8) << BIT_20) |
                   ((gprAddr & 0x3ff) << BIT_10) | (srcHiAddr & 0x3ff);

    microInst[1] = microInst[0x1] | ((data16Lo >> BIT_8) << BIT_20) |
                   ((gprAddr & 0x3ff) << BIT_10) | (srcLoAddr & 0x3ff);

    /* fixup alu[$xfer, --, b, gpr] instruction for source/dest register */
    microInst[0x2] = microInst[0x2] | \
                   ((xfrAddr & 0x3ff) << BIT_20) | \
                   ((gprAddr & 0x3ff) << BIT_10);

    status = execMicroInst(handle, ae, ctx, microInst, num_inst, 
                           condCodeOff, dly, NULL);
    /* restore the value of a b-bank gpr */
    putRelDataReg(handle, ae, ctx, ICP_GPB_REL, gprNum, gprVal);

    return (status);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word value to the AE's NN register. The AE must be
 *              disabled prior to calling this function.
 *
 * @param ae - IN
 * @param ctx - IN
 * @param nnNum - IN
 * @param value - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
int 
putRelNN_Common(icp_firml_handle_t *handle, 
                unsigned char ae, 
                unsigned char ctx, 
                unsigned short nnNum,
                unsigned int value)
{
    unsigned int ctxEnables;
    int stat = HALAE_SUCCESS;

    /* make sure nn_mode is set to self */
    getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
    /* prevent clearing the W1C bits: the breakpoint bit, 
       ECC error bit, and Parity error bit */        
    ctxEnables &= IGNORE_W1C_MASK;
    putAeCsr(handle, ae, CTX_ENABLES, ctxEnables | CE_NN_MODE);

    stat = putRelWrXfer(handle, ae, ctx, ICP_NEIGH_REL, nnNum, value);
    putAeCsr(handle, ae, CTX_ENABLES, ctxEnables);

    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word value to the AE's LM location specified by 
 *              the lmAddr word-address. It's unsafe to call this function while 
 *              the AE is enabled.
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
halAe_PutLM_Common(icp_firml_handle_t *handle, 
                   unsigned char ae, 
                   unsigned short lmAddr,
                   unsigned int value)
{
    unsigned int ctxEnables, lmAddr_sav, indirectLmAddr_sav;
    unsigned int lmAddrByte0_sav, indirectLmAddrByte0_sav;
    int stat = HALAE_SUCCESS;
    const unsigned char global=1, ctx=0;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    if(IS_RST(handle, ae)) 
    {
        return (HALAE_RESET);
    }    
    if(lmAddr >= handle->halHandle->MaxLmemReg) 
    {
       ERRINFO((" lmAddr=0x%x\n", lmAddr)); 
       return (HALAE_BADARG);
    }   

    SPIN_LOCK_AE(handle, ae);

    if(isAeActive(handle, ae)) 
    {
        SPIN_UNLOCK_AE(handle, ae);
        return (HALAE_AEACTIVE);
    }    

    /* set LM mode global */
    getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
    /* prevent clearing the W1C bits: the breakpoint bit, 
       ECC error bit, and Parity error bit */        
    ctxEnables &= IGNORE_W1C_MASK;

    putAeLmMode(handle, ae, ICP_LMEM0, global);

    /* save current LM addr */
    getCtxIndrCsr(handle, ae, ctx, INDIRECT_LM_ADDR_0_BYTE_INDEX, 
                    &indirectLmAddrByte0_sav);
    getAeCsr(handle, ae, ACTIVE_LM_ADDR_0_BYTE_INDEX, &lmAddrByte0_sav);
    getCtxIndrCsr(handle, ae, ctx, LM_ADDR_0_INDIRECT, &indirectLmAddr_sav);    
    getAeCsr(handle, ae, LM_ADDR_0_ACTIVE, &lmAddr_sav);

    /* set LM addr */
    putAeCsr(handle, ae, LM_ADDR_0_ACTIVE, lmAddr << 0x2);
    stat = putRelDataReg(handle, ae, ctx, ICP_LMEM0, lmAddr, value);

    /* restore ctx_enables, and LM_ADDR */
    putAeCsr(handle, ae, CTX_ENABLES, ctxEnables);
    putAeCsr(handle, ae, LM_ADDR_0_ACTIVE, lmAddr_sav);
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    LM_ADDR_0_INDIRECT, indirectLmAddr_sav);    
    putAeCsr(handle, ae, ACTIVE_LM_ADDR_0_BYTE_INDEX, lmAddrByte0_sav);
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    INDIRECT_LM_ADDR_0_BYTE_INDEX, indirectLmAddrByte0_sav);

    SPIN_UNLOCK_AE(handle, ae);

    return (stat);
}
 
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word value to the AE's LM location specified by
 *              the lmAddr word-address. 
 *
 * @param ae - IN
 * @param lmAddr - IN
 * @param value - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_GetLM_Common(icp_firml_handle_t *handle, 
                   unsigned char ae,
                   unsigned short lmAddr,
                   unsigned int *value)
{
    unsigned int ctxEnables, lmAddr_sav, indirectLmAddr_sav;
    unsigned int lmAddrByte0_sav, indirectLmAddrByte0_sav;
    int stat = HALAE_SUCCESS;
    const unsigned char global=1, ctx=0;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(value);
    VERIFY_AE(handle, ae);
    if(IS_RST(handle, ae)) 
    {
        return (HALAE_RESET);
    }    
    if(lmAddr >= handle->halHandle->MaxLmemReg) 
    {
       ERRINFO((" lmAddr=0x%x\n", lmAddr)); 
       return (HALAE_BADARG);
    }   

    SPIN_LOCK_AE(handle, ae);

    if(isAeActive(handle, ae)) 
    {
        SPIN_UNLOCK_AE(handle, ae);
        return (HALAE_AEACTIVE);
    }   

    /* set LM mode global */
    getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
    /* prevent clearing the W1C bits: the breakpoint bit, 
       ECC error bit, and Parity error bit */        
    ctxEnables &= IGNORE_W1C_MASK;
    putAeLmMode(handle, ae, ICP_LMEM0, global);

    /* save current LM addr */
    getCtxIndrCsr(handle, ae, ctx, INDIRECT_LM_ADDR_0_BYTE_INDEX, 
                    &indirectLmAddrByte0_sav);
    getAeCsr(handle, ae, ACTIVE_LM_ADDR_0_BYTE_INDEX, &lmAddrByte0_sav);
    getCtxIndrCsr(handle, ae, ctx, LM_ADDR_0_INDIRECT, &indirectLmAddr_sav);    
    getAeCsr(handle, ae, LM_ADDR_0_ACTIVE, &lmAddr_sav);

    /* set LM addr */
    putAeCsr(handle, ae, LM_ADDR_0_ACTIVE, lmAddr << 0x2);

    stat = getRelDataReg(handle, ae, ctx, ICP_LMEM0, lmAddr, value);

    /* restore ctx_enables, and LM_ADDR */
    putAeCsr(handle, ae, CTX_ENABLES, ctxEnables);
    putAeCsr(handle, ae, LM_ADDR_0_ACTIVE, lmAddr_sav);
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    LM_ADDR_0_INDIRECT, indirectLmAddr_sav);    
    putAeCsr(handle, ae, ACTIVE_LM_ADDR_0_BYTE_INDEX, lmAddrByte0_sav);
    putCtxIndrCsr(handle, ae, (1 << ctx), 
                    INDIRECT_LM_ADDR_0_BYTE_INDEX, indirectLmAddrByte0_sav);

    SPIN_UNLOCK_AE(handle, ae);

    return (stat);
}
 
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write a long-word value to the SSU Share Ram location specified
 *              by the addr. It's unsafe to call this function while the AE 
 *              is enabled.
 *
 * @param ae - IN
 * @param addr - IN
 * @param value - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_PutSharedRam_Common(icp_firml_handle_t *handle, 
                          unsigned char ae,
                          unsigned char qat,
                          unsigned int addr, 
                          unsigned int value)
{
    unsigned int gprA0, gprA1, gprB0, sr0;
    uword_T tgtCmd, token;
    int stat = HALAE_SUCCESS;
    uword_T microInst[] = 
    {
        /*
        .reg addr value
        .reg qat
        .areg addr 0
        .areg value 1
        .breg qat 0
        .reg $value
        .addr $value 0
        .sig sigWrite
        */
        0x0D800800010ull, /*0. br=ctx[0,l000_01#] */
        0x0D804C00018ull, /*1. br[kill_context#] */
        0x0F0400C0000ull, /*2. l000_01#:immed_w0[l0000!addr, 0x0] */
        0x0F4400C0000ull, /*3. immed_w1[l0000!addr, 0x0] */
        0x0F0400C0001ull, /*4. immed_w0[l0000!value, 0x0] */
        0x0F4400C0001ull, /*5. immed_w1[l0000!value, 0x0] */
        0x0F040000300ull, /*6. immed_w0[l0000!qat, 0x0] */
        0x0F440000300ull, /*7. immed_w1[l0000!qat, 0x0] */
        0x0A0580C0001ull, /*8. alu[$l0000!value, --, b, l0000!value] */
        0x08B00009200ull, /*9. alu_shf[l0000!addr, l0000!addr, or, 4, <<16] */
        0x09050000200ull, /*10. alu_shf[l0000!qat, --, b, l0000!qat,  <<27] */
        0x02010000300ull, /*11. ssu_transfer[write, shared_ram, 
                                $l0000!value, l0000!addr, l0000!qat, 1], 
                                ssua_0_1, sig_done[l0000!sigWrite] 
                                instruction for accelcomp_c */
        0x0D803104213ull, /*12. nop_loop#: 
                            br_!signal[l0000!sigWrite, nop_loop#], defer[1] */
        0x0F0000C0300ull, /*13. nop */
        0x0F0000C0300ull, /*14. nop */
        0x0F0000C0300ull, /*15. nop */
        0x0F0000C0300ull, /*16. nop */
        0x0F0000C0300ull, /*17. end_of_program#: nop */
        0x0E000010000ull, /*18. ctx_arb[kill], any */
        0x0E000010000ull, /*19. kill_context#: ctx_arb[kill], any */
        0x0F0000C0300ull, /*20. nop */
    };
    
    const int num_inst = sizeof(microInst)/sizeof(microInst[0]),condCodeOff=1;
    const unsigned char ctx=0;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    if(IS_RST(handle, ae)) 
    {
        return (HALAE_RESET);
    }    

    if(!(handle->sysMemInfo.qatMask & (1 << qat)))
    {
       ERRINFO(("qat=0x%x\n", qat)); 
       return (HALAE_BADARG);
    }    

    if(addr > MAX_SSU_SHARED_RAM - sizeof(unsigned int)) 
    {
       ERRINFO(("addr=0x%x\n", addr)); 
       return (HALAE_BADARG);
    }   

    SPIN_LOCK_AE(handle, ae);

    if(isAeActive(handle, ae)) 
    {
        SPIN_UNLOCK_AE(handle, ae);
        return (HALAE_AEACTIVE);
    }

    /* Compares to accel_comp_C, accel_comp_B/RS skip qat1 TargetId */
    if ((handle->sysMemInfo.deviceId == ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
    {
        if (qat >= 0x1)
        {
            qat++;
        }
    }
    
    /* fixup instruction for data and address */
    INSERT_IMMED_GPRA_CONST(microInst[0x2], (addr >>  0));
    INSERT_IMMED_GPRA_CONST(microInst[0x3], (addr >> WORD_SIZE_IN_BIT));
    INSERT_IMMED_GPRA_CONST(microInst[0x4], (value >>  0));
    INSERT_IMMED_GPRA_CONST(microInst[0x5], (value >> WORD_SIZE_IN_BIT));
    INSERT_IMMED_GPRB_CONST(microInst[0x6], ((qat%0x2) >>  0));
    INSERT_IMMED_GPRB_CONST(microInst[0x7], ((qat%0x2) >> WORD_SIZE_IN_BIT));

    if ((handle->sysMemInfo.deviceId == ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
    {
        tgtCmd = (ACCELCOMP_C_IS_SSUA(qat)) ? 0 : 1;
        token = qat >> 0x2;
        
        INSERT_SSU_TRANSFER_OPTTOKEN_CONST(microInst[0xb], tgtCmd, token);
    }

    /* get and save the value of gpr and xfer_out reg */
    getRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0, &gprA0);
    getRelDataReg(handle, ae, ctx, ICP_GPA_REL, 1, &gprA1);
    getRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, &gprB0);
    getRelWrXfers(handle, ae, ctx, ICP_SR_WR_REL, 0, &sr0, 1);

    /* execute ssu_transfer instruction */
    stat = execMicroInst(handle, ae, ctx, microInst, num_inst, 
                         condCodeOff, HAL_EXECMICROINST_TIME, NULL);

    /* restore the registers */
    putRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0, gprA0);
    putRelDataReg(handle, ae, ctx, ICP_GPA_REL, 1, gprA1);
    putRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, gprB0);    
    putRelWrXfer(handle, ae, ctx, ICP_SR_WR_REL, 0, sr0);

    SPIN_UNLOCK_AE(handle, ae);

    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read a long-word value from the SSU Share Ram location 
 *               specified by the addr. It's unsafe to call this function while 
 *               the AE is enabled.
 *
 * @param ae - IN
 * @param addr - IN
 * @param value - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADLIB, HALAE_BADARG, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_GetSharedRam_Common(icp_firml_handle_t *handle, 
                          unsigned char ae, 
                          unsigned char qat,
                          unsigned int addr, 
                          unsigned int *value)
{
    unsigned int gprA0, gprB0, sr0;
    uword_T tgtCmd, token;
    int stat = HALAE_SUCCESS;
    uword_T microInst[] = 
    {
        /*
        .reg addr
        .areg addr 0
        .reg qat
        .breg qat 0
        .reg $value
        .addr $value 0
        .sig sigRead
        */
        0x0D800800010ull, /*0. br=ctx[0,l000_01#] */
        0x0D804800018ull, /*1. br[kill_context#] */
        0x0D804000011ull, /*2. br!=ctx[0,l001_01#] */
        0x0F0400C0000ull, /*3. immed_w0[l0000!addr, 0x0] */
        0x0F4400C0000ull, /*4. immed_w1[l0000!addr, 0x0] */
        0x0F040000300ull, /*5. immed_w0[l0000!qat, 0x0] */
        0x0F440000300ull, /*6. immed_w1[l0000!qat, 0x0] */
        0x08B00009200ull, /*7. alu_shf[l0000!addr, l0000!addr, or, 4, <<16] */
        0x09050000200ull, /*8. alu_shf[l0000!qat, --, b, l0000!qat,  <<27] */
        0x0F0000C0300ull, /*9. nop */
        0x02210000300ull, /*10. ssu_transfer[read, shared_ram, 
                            $l0000!value, l0000!addr, l0000!qat, 1], 
                            ssua_0_1, sig_done[l0000!sigRead]
                            instruction for accelcomp_c */
        0x0D802D04213ull, /*11. nop_loop#: 
                            br_!signal[l0000!sigRead, nop_loop#], defer[1] */
        0x0F0000C0300ull, /*12. nop */
        0x0F0000C0300ull, /*13. nop */
        0x0F0000C0300ull, /*14. nop */
        0x0F0000C0300ull, /*15. nop */
        0x0F0000C0300ull, /*16. l001_01#: l001_end#: end_of_program#: nop */
        0x0E000010000ull, /*17. ctx_arb[kill], any */
        0x0E000010000ull, /*18. kill_context#: ctx_arb[kill], any */
        0x0F0000C0300ull, /*19. nop */
    };
    const int num_inst = sizeof(microInst)/sizeof(microInst[0]),condCodeOff=1;
    const unsigned char ctx=0;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    if(IS_RST(handle, ae)) 
    {
        return (HALAE_RESET);
    }    

    if(!(handle->sysMemInfo.qatMask & (1 << qat)))
    {
       ERRINFO(("qat=0x%x\n", qat)); 
       return (HALAE_BADARG);
    }   

    if(value == NULL)
    {
       ERRINFO(("addr=0x%x\n", addr)); 
       return (HALAE_BADARG);
    }    

    if(addr > MAX_SSU_SHARED_RAM - sizeof(unsigned int)) 
    {
       ERRINFO(("addr=0x%x\n", addr)); 
       return (HALAE_BADARG);
    }   

    SPIN_LOCK_AE(handle, ae);

    if(isAeActive(handle, ae)) 
    {
        SPIN_UNLOCK_AE(handle, ae);
        return (HALAE_AEACTIVE);
    }   

    /* Compares to accel_comp_C, accel_comp_B/RS skip qat1 TargetId */
    if ((handle->sysMemInfo.deviceId == ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
    {
        if (qat >= 0x1)
        {
            qat++;
        } 
    }

    /* fixup instruction for data and address */
    INSERT_IMMED_GPRA_CONST(microInst[0x3], (addr >>  0));
    INSERT_IMMED_GPRA_CONST(microInst[0x4], (addr >> WORD_SIZE_IN_BIT));
    INSERT_IMMED_GPRB_CONST(microInst[0x5], (qat%0x2 >>  0));
    INSERT_IMMED_GPRB_CONST(microInst[0x6], (qat%0x2 >> WORD_SIZE_IN_BIT));

    if ((handle->sysMemInfo.deviceId == ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) ||
        (handle->sysMemInfo.deviceId == ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC))
    {
        tgtCmd = (ACCELCOMP_C_IS_SSUA(qat)) ? 0 : 1;
        token = qat >> 0x2;

        INSERT_SSU_TRANSFER_OPTTOKEN_CONST(microInst[0xa], tgtCmd, token);
    }

    /* get and save the value of gpr and xfer_out reg */
    getRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0, &gprA0);
    getRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, &gprB0);
    getRelRdXfer(handle, ae, ctx, ICP_SR_RD_REL, 0, &sr0);

    /* execute ssu_transfer instruction */
    stat = execMicroInst(handle, ae, ctx, microInst, num_inst, 
                         condCodeOff, HAL_EXECMICROINST_TIME, NULL);

    /* get the read value */
    getRelRdXfer(handle, ae, ctx, ICP_SR_RD_REL, 0, value);

    /* restore the registers */
    putRelDataReg(handle, ae, ctx, ICP_GPA_REL, 0, gprA0);
    putRelDataReg(handle, ae, ctx, ICP_GPB_REL, 0, gprB0);    
    putRelRdXfer(handle, ae, ctx, ICP_SR_RD_REL, 0, sr0);

    SPIN_UNLOCK_AE(handle, ae);

    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get dram1 physical base address.
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
 * @param baseAddr - OUT A point to the location of base address to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_UNINIT A bad debug library is associated with the specified
                        accelaration engine the library is not initialized
 * @retval HALAE_CLR_RST The accelaration engine is not in a reset state
 * @retval HALAE_RST The accelaration engine is in reset state
 * 
 * 
 *****************************************************************************/
int 
halAe_GetDram1BaseAddr(icp_firml_handle_t *handle, 
                       uint64 *baseAddr)
{
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    if(!baseAddr)
    {
        ERRINFO(("bad argument baseAddr, is NULL\n ")); 
        return (HALAE_BADARG);
    } 

    *baseAddr = handle->sysMemInfo.dramDesc[1].dramBaseAddr;
   
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get dram0 physical base address.
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
 * @param baseAddr - OUT A point to the location of base address to read
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * @retval HALAE_UNINIT A bad debug library is associated with the specified
                        accelaration engine the library is not initialized
 * @retval HALAE_CLR_RST The accelaration engine is not in a reset state
 * @retval HALAE_RST The accelaration engine is in reset state
 * 
 * 
 *****************************************************************************/
int 
halAe_GetDram0BaseAddr(icp_firml_handle_t *handle, 
                        uint64 *baseAddr)
{
    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    if(!baseAddr)
    {
        ERRINFO(("bad argument baseAddr, is NULL\n ")); 
        return (HALAE_BADARG);
    }

    *baseAddr = handle->sysMemInfo.dramDesc[0].dramBaseAddr;
    
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Convert ctx relative reg to absolute.
 * 
 * @param ae - IN 
 * @param ctx - IN 
 * @param relRegNum - OUT
 * @param absRegNum - OUT
 *
 * @retval HALAE_SUCCESS 
 * 
 * 
 *****************************************************************************/
 
int 
relToAbs(icp_firml_handle_t *handle, 
         unsigned char ae, 
         unsigned char ctx, 
         unsigned short relRegNum,
         unsigned short *absRegNum)
{
    unsigned int ctxEnables;

    VERIFY_ARGPTR(absRegNum);

    getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
    if(ctxEnables & CE_INUSE_CONTEXTS) 
    {
        /* 4-ctx mode */
        if(ctx & 0x1) 
        {
           ERRINFO(("bad ctx in 4-ctx mode,ctx=0x%x\n", ctx)); 
           return (HALAE_BADARG);
        }   
        *absRegNum = ((ctx & 0x6) << 0x4) | (relRegNum & 0x1f);
    } 
    else 
    {
        /* 8-ctx mode */
        *absRegNum = ((ctx & 0x7) << 0x4) | (relRegNum & 0x0f);
    }
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Convert absolute reg to ctx relative reg.
 * 
 * @param ae - IN 
 * @param absRegNum - IN
 * @param relRegNum - OUT
 * @param ctx - OUT
 *
 * @retval HALAE_SUCCESS 
 * 
 * 
 *****************************************************************************/
 static int 
absToRel(icp_firml_handle_t *handle, 
         unsigned char ae, 
         unsigned short absRegNum, 
         unsigned short *relRegNum, 
         unsigned char *ctx)
{
    unsigned int ctxEnables;

    getAeCsr(handle, ae, CTX_ENABLES, &ctxEnables);
    if(ctxEnables & CE_INUSE_CONTEXTS) 
    {
        /* 4-ctx mode */
        *relRegNum = absRegNum & 0x1F;
        *ctx = (absRegNum >> 0x4) & 0x6;
    } else 
    {
        /* 8-ctx mode */
        *relRegNum = absRegNum & 0x0F;
        *ctx = (absRegNum >> 0x4) & 0x7;
    }
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Read long-words from the specified register type:
 *               ICP_GPA_REL, ICP_GPB_REL, ICP_DR_RD_REL, ICP_SR_RD_REL,
 *               ICP_DR_WR_REL, ICP_SR_WR_REL, ICP_NEIGH_REL.
 *               The AE must be disabled prior to calling this function.
 *
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum- IN
 * @param regData - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG, HALAE_ENABLED, HALAE_FAIL,
                HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_GetRelDataReg_Common(icp_firml_handle_t *handle, 
                           unsigned char ae, 
                           unsigned char ctx, 
                           icp_RegType_T regType,
                           unsigned short regNum, 
                           unsigned int *regData)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_ARGPTR(regData);
    VERIFY_AE(handle, ae);
    VERIFY_CTX(ctx);
    if(IS_RST(handle, ae)) 
    {
        return (HALAE_RESET);
    }    

    SPIN_LOCK_AE(handle, ae);
    if(isAeActive(handle, ae)) 
    {
       SPIN_UNLOCK_AE(handle, ae);
       return (HALAE_AEACTIVE);
    }   

    switch(regType)
    {
    case ICP_GPA_REL:
    case ICP_GPB_REL:
        stat = getRelDataReg(handle, ae, ctx, regType, regNum, regData);
        break;

    case ICP_DR_RD_REL:
    case ICP_SR_RD_REL:
        stat = getRelRdXfer(handle, ae, ctx, regType, regNum, regData);
        break;

    case ICP_DR_WR_REL:
    case ICP_SR_WR_REL:
        stat = getRelWrXfers(handle, ae, ctx, regType, regNum, regData, 1);
        break;

    case ICP_NEIGH_REL:
        stat = getRelNNReg(handle, ae, ctx, regType, regNum, regData);
        break;
    default:
        stat = HALAE_BADARG;
        break;
    }
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Write long-words to the specified register type:
 *               ICP_GPA_REL, ICP_GPB_REL, ICP_DR_RD_REL, ICP_SR_RD_REL,
 *               ICP_DR_WR_REL, ICP_SR_WR_REL, ICP_NEIGH_REL
 *               The AE must be disabled prior to calling this function.
 *
 * @param ae - IN
 * @param ctx - IN
 * @param regType - IN
 * @param regNum- IN
 * @param regData - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG, HALAE_ENABLED, HALAE_FAIL,
                HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_PutRelDataReg_Common(icp_firml_handle_t *handle, 
                           unsigned char ae, 
                           unsigned char ctx, 
                           icp_RegType_T regType,
                           unsigned short regNum, 
                           unsigned int regData)
{
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    VERIFY_CTX(ctx);
    if(IS_RST(handle, ae)) 
    {
        return (HALAE_RESET);
    }    

    SPIN_LOCK_AE(handle, ae);
    if(isAeActive(handle, ae)) 
    {
       SPIN_UNLOCK_AE(handle, ae);
       return (HALAE_AEACTIVE);
    }   

    switch(regType)
    {
    case ICP_GPA_REL:
    case ICP_GPB_REL:
        stat = putRelDataReg(handle, ae, ctx, regType, regNum, regData);
        break;

    case ICP_DR_RD_REL:
    case ICP_SR_RD_REL:
        stat = putRelRdXfer(handle, ae, ctx, regType, regNum, regData);
        break;

    case ICP_DR_WR_REL:
    case ICP_SR_WR_REL:
        stat = putRelWrXfer(handle, ae, ctx, regType, regNum, regData);
        break;

    case ICP_NEIGH_REL:
        stat = putRelNN(handle, ae, ctx, regNum, regData);
        break;

    default:
        stat = HALAE_BADARG;
        break;
    }
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *   Read long-words from the specified register type:
 *               ICP_GPA_ABS, ICP_GPB_ABS, ICP_DR_RD_ABS, ICP_SR_RD_ABS,
 *               ICP_DR_WR_ABS, ICP_SR_WR_ABS, ICP_NEIGH_ABS
 *               The AE must be disabled prior to calling this function.  
 *
 * @param ae - IN
 * @param regType - IN
 * @param absRegNum - IN
 * @param regData - OUT
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG, HALAE_ENABLED, HALAE_FAIL,
 *               HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_GetAbsDataReg_Common(icp_firml_handle_t *handle, 
                           unsigned char ae, 
                           icp_RegType_T regType,
                           unsigned short absRegNum, 
                           unsigned int *regData)
{
    unsigned char ctx;
    unsigned short regNum;
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    if(IS_RST(handle, ae)) 
    {
        return (HALAE_RESET);
    }    

    if(regData == NULL)
    {
        ERRINFO(("bad argumetn, regData is NULL\n")); 
        return (HALAE_BADARG);
    }    

    SPIN_LOCK_AE(handle, ae);
    absToRel(handle, ae, absRegNum, &regNum, &ctx);
    if(isAeActive(handle, ae)) 
    {
       SPIN_UNLOCK_AE(handle, ae);
       return (HALAE_AEACTIVE);
    }   

    switch(regType)
    {
    case ICP_GPA_ABS:
        if(absRegNum >= MAX_GPR_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = getRelDataReg(handle, ae, ctx, ICP_GPA_REL, regNum, regData);
        break;

    case ICP_GPB_ABS:
        if(absRegNum >= MAX_GPR_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = getRelDataReg(handle, ae, ctx, ICP_GPB_REL, regNum, regData);
        break;

    case ICP_DR_RD_ABS:
        if(absRegNum >= MAX_XFER_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = getRelRdXfer(handle, ae, ctx, ICP_DR_RD_REL, regNum, regData);
        break;

    case ICP_SR_RD_ABS:
        if(absRegNum >= MAX_XFER_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = getRelRdXfer(handle, ae, ctx, ICP_SR_RD_REL, regNum, regData);
        break;

    case ICP_DR_WR_ABS:
        if(absRegNum >= MAX_XFER_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = getRelWrXfers(handle, ae, ctx, 
                        ICP_DR_WR_REL, regNum, regData, 1);
        break;

    case ICP_SR_WR_ABS:
        if(absRegNum >= MAX_XFER_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = getRelWrXfers(handle, ae, ctx, 
                        ICP_SR_WR_REL, regNum, regData, 1);
        break;

    case ICP_NEIGH_ABS:
        if(absRegNum >= MAX_NN_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = getRelNNReg(handle, ae, ctx, ICP_NEIGH_REL, regNum, regData);
        break;

    default:
        stat = HALAE_BADARG;
        break;
    }
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write long-words to the specified register type:
 *               ICP_GPA_ABS, ICP_GPB_ABS, ICP_DR_RD_ABS, ICP_SR_RD_ABS,
 *               ICP_DR_WR_ABS, ICP_SR_WR_ABS, ICP_NEIGH_ABS
 *               The AE must be disabled prior to calling this function.
 *
 * @param ae - IN
 * @param regType - IN
 * @param absRegNum - IN
 * @param regData - IN 
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG, HALAE_ENABLED, HALAE_FAIL,
 *               HALAE_RESET, HALAE_AEACTIVE
 * 
 * 
 *****************************************************************************/
int 
halAe_PutAbsDataReg_Common(icp_firml_handle_t *handle, 
                           unsigned char ae, 
                           icp_RegType_T regType,
                           unsigned short absRegNum, 
                           unsigned int regData)
{
    unsigned char ctx;
    unsigned short regNum;
    int stat = HALAE_SUCCESS;

    VERIFY_ARGPTR(handle);
    HALAE_VERIFY_LIB(handle);
    VERIFY_AE(handle, ae);
    if(IS_RST(handle, ae)) 
    {
        return (HALAE_RESET);
    }    
    
    SPIN_LOCK_AE(handle, ae);
    absToRel(handle, ae, absRegNum, &regNum, &ctx);
    if(isAeActive(handle, ae)) 
    {
       SPIN_UNLOCK_AE(handle, ae);
       return (HALAE_AEACTIVE);
    }   

    switch(regType)
    {
    case ICP_GPA_ABS:
        if(absRegNum >= MAX_GPR_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = putRelDataReg(handle, ae, ctx, ICP_GPA_REL, regNum, regData);
        break;
    case ICP_GPB_ABS:
        if(absRegNum >= MAX_GPR_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = putRelDataReg(handle, ae, ctx, ICP_GPB_REL, regNum, regData);
        break;

    case ICP_DR_RD_ABS:
        if(absRegNum >= MAX_XFER_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = putRelRdXfer(handle, ae, ctx, ICP_DR_RD_REL, regNum, regData);
        break;
    case ICP_SR_RD_ABS:
        if(absRegNum >= MAX_XFER_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = putRelRdXfer(handle, ae, ctx, ICP_SR_RD_REL, regNum, regData);
        break;

    case ICP_DR_WR_ABS:
        if(absRegNum >= MAX_XFER_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = putRelWrXfer(handle, ae, ctx, ICP_DR_WR_REL, regNum, regData);
        break;
    case ICP_SR_WR_ABS:
        if(absRegNum >= MAX_XFER_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = putRelWrXfer(handle, ae, ctx, ICP_SR_WR_REL, regNum, regData);
        break;

    case ICP_NEIGH_ABS:
        if(absRegNum >= MAX_NN_REG)
        {
            stat = HALAE_BADARG;
            break;
        }
        stat = putRelNN(handle, ae, ctx, regNum, regData);
        break;

    default:
        stat = HALAE_BADARG;
        break;
    }
    SPIN_UNLOCK_AE(handle, ae);
    return (stat);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read a generic data register of any type.
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
 * @param regType - IN Specifies the register type
 * @param regAddr - IN Specifies the register address offset
 * @param regData - IN A pointer to the location of the data to read
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
halAe_GetDataReg(icp_firml_handle_t *handle, 
                 unsigned char hwAeNum, 
                 unsigned char ctx,
                 icp_RegType_T regType,
                 unsigned short regAddr, 
                 unsigned int *regData)
{
    int status = HALAE_SUCCESS;
    unsigned int csrVal, aeCsr;
    unsigned short dr_offset = 0x10;

    switch(regType)
    {
    case ICP_GPA_REL:
    case ICP_GPB_REL:
    case ICP_DR_RD_REL:
    case ICP_SR_RD_REL:
    case ICP_DR_WR_REL:
    case ICP_SR_WR_REL:
    case ICP_NEIGH_REL:
        status = halAe_GetRelDataReg(handle, hwAeNum, ctx, 
                        regType, regAddr, regData);
        break;
    case ICP_GPA_ABS:
    case ICP_GPB_ABS:
    case ICP_DR_RD_ABS:
    case ICP_SR_RD_ABS:
    case ICP_DR_WR_ABS:
    case ICP_SR_WR_ABS:
    case ICP_NEIGH_ABS:
        status = halAe_GetAbsDataReg(handle, hwAeNum, 
                        regType, regAddr, regData);
        break;
    case ICP_SR_INDX:
    case ICP_DR_INDX:
        /* only zero is allowed as regAddr for ICP_SR_INDX/ICP_DR_INDX */
        if(regAddr != 0) 
        {
            status = HALAE_BADARG;
            break;
        }
        /* read the T_INDEX CSR and get the xferAddr */
        status = halAe_GetAeCsr(handle, hwAeNum, T_INDEX, &csrVal);
        if(status != HALAE_SUCCESS) 
        {
            break;
        }     

        regAddr = ((csrVal & TI_XFER_REG_INDEX) >> TI_XFER_REG_INDEX_BITPOS);

        status = halAe_GetAeCsr(handle, hwAeNum, CTX_ENABLES, &csrVal);
        if(status != HALAE_SUCCESS) 
        {
            break;
        }

        /* XFER_INDEX in T_INDEX is to specify one of 256 registers
         * for TRANSFER_IN or TRANSFER_OUT.
         * So, 8 ctx Mode:
         *     for ctx0's SR xfer, XFER_INDEX=0x0 - 0xf
         *     for ctx0's DR xfer, XFER_INDEX=0x10 - 0x1f
         *     ......
         *     for ctx7's SR xfer, XFER_INDEX=0xe0 - 0xef
         *     for ctx7's DR xfer, XFER_INDEX=0xf0 - 0xff
         *     4 ctx Mode:
     *     for ctx0's SR xfer, XFER_INDEX=0x0 - 0x1f
         *     for ctx0's DR xfer, XFER_INDEX=0x20 - 0x3f
         *     ......
         *     for ctx6's SR xfer, XFER_INDEX=0xc0 - 0xdf
         *     for ctx6's DR xfer, XFER_INDEX=0xe0 - 0xff
         * the parameter absRegNum of halAe_GetAbsDataReg is 
         * to sepcify one of 128 registers of SR xfer or DR xfer 
         * in TRANSFER_IN or TRANSFER_OUT
         * So, 8 ctx Mode:
         *     for ctx0's SR xfer, absRegNum=0x0 - 0xf
         *     for ctx0's DR xfer, absRegNum=0x0 - 0xf
         *     ......
         *     for ctx7's SR xfer, absRegNum=0xe70 - 0x7f
         *     for ctx7's DR xfer, absRegNum=0x70 - 0x7f
         *     4 ctx Mode:
     *     for ctx0's SR xfer, absRegNum=0x0 - 0x1f
         *     for ctx0's DR xfer, absRegNum=0x0 - 0x1f
         *     ......
         *     for ctx6's SR xfer, absRegNum=0x60 - 0x7f
         *     for ctx6's DR xfer, absRegNum=0x60 - 0x7f
         * Here, XFER_INDEX should be coverted to absRegNum
         * before input it to halAe_GetAbsDataReg
         */
        if(csrVal & CE_INUSE_CONTEXTS) 
        {
            /* 4-ctx mode */
            dr_offset = 0x20;
            regAddr = (((regAddr & (~dr_offset)) & 0xe0) >> 0x1)
                      | (regAddr & 0x1f);
        } else 
        {
            /* 8-ctx mode */
            dr_offset = 0x10;
            regAddr = (((regAddr & (~dr_offset)) & 0xf0) >> 0x1) 
                      | (regAddr & 0xf);
        }
        if(regType == ICP_SR_INDX) /* ICP_SR_INDX=0x00-0x0f */ 
        {
            regType = ICP_SR_RD_ABS;
        }    
        else                       /* ICP_DR_INDX=0x10-0x1f */ 
        {
            regType = ICP_DR_RD_ABS;
        }    
        status = halAe_GetAbsDataReg(handle, hwAeNum, 
                        regType, regAddr, regData);
        break;
    case ICP_LMEM0:
    case ICP_LMEM1:
        /* read the LM CSR and "or" with the regAddr to get the lmAddr */
        if(regType == ICP_LMEM0) 
        {
           aeCsr = LM_ADDR_0_INDIRECT;
        }   
        else
        {
           aeCsr = LM_ADDR_1_INDIRECT;
        }   
        if((status = halAe_GetCtxIndrCsr(handle, hwAeNum, ctx, aeCsr, &csrVal)))
        {
            break;
        }

        regAddr |= ((csrVal & XLA_LM_ADDR) >> XLA_LM_ADDR_BITPOS);
        status = halAe_GetLM(handle, hwAeNum, regAddr, regData);
        break;
    default: status = HALAE_BADARG;
    }
    return status;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Initialize a block of memory to a constant value
 *
 * @param vaddr_dst - IN
 * @param c - IN 
 * @param size - IN 
 *
 * @retval  Error status HALAE_*
 * 
 * 
 *****************************************************************************/
int 
halAe_MemSet(uint64 vaddr_dst, 
             int c, 
             int size)
{
    int status = HALAE_SUCCESS;
    unsigned long dst_addr;
    unsigned char ch;

    dst_addr = (unsigned long)vaddr_dst;
    ch = (unsigned char) c;
    memset((void*)dst_addr, ch, size);
    status = HALAE_SUCCESS;
 
    return status;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Call back thread: waits on halAe_IntrPoll() and then calls
 *               a callback function. This version follows chaining
 *
 * @param data - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
halAe_IntrCallbackThdEx(void* data)
{
    HalAeCallback_T *thread_data = (HalAeCallback_T*)data;
    int status = HALAE_SUCCESS;
    Hal_IntrMasks_T masks;
    HalAeCallbackChain_T *chain;

    icp_firml_handle_t *handle = (icp_firml_handle_t *)(thread_data->handle);

    osalMemSet(&masks, 0, sizeof(masks));
    while (1) 
    {
        status = halAe_IntrPoll(handle, thread_data->type_mask, &masks);
        if (status == HALAE_DISABLED) 
        {
            break;
        }    
        if (status != HALAE_SUCCESS) 
        {
            PRINTF("halAe_IntrPoll returns 0x%X\n",status);
            break;
        }
        SPIN_LOCK(handle->halHandle->hal_CallbackThdLock);
        for (chain = thread_data->callback_chain.next;
             chain;
             chain = chain->next) 
        {
            (*chain->callback_func)(&masks, chain->callback_data);
        }
        SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
    } /* end while 1 */

    SEM_POST(&thread_data->terminate_sem);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      This function is similar to halAe_IntrPoll() except that it does not 
 *      block waiting for the specified interrupts. Instead, it returns 
 *      immediately and invokes the specified callback function for subsequent
 *      occurrences of the interrupt or interrupts. The callback data are 
 *      passed to the callback function on invocation. A handle associated 
 *      with  this call is returned and this handle can be used to terminate
 *      the request and disregard the specified interrupts.
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
 * @param type_mask - IN Specifies the mask type must be one or more of the 
 *                       following values {HALAE_INTR_ATTN_BKPT_MASK, 
 *                       HALAE_INTR_ATTN_PARITY_MASK,
 * @param callback_func - IN A pointer to the location of a callback function
 * @param callback_data - IN A pointer to the location of application-specific 
 *                           data. This pointer is passed to the callback 
 *                           function when it is invoked
 * @param thd_priority - IN The thread priority
 * @param callback_priority - IN The callback priority
 * @param handle - IN A handle associated with and returned by this call. 
 *                    This handle can be used to terminate the request and 
 *                    disregard the specified interrupts
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/
int 
halAe_SpawnIntrCallbackThdEx(unsigned int        type_mask,
                             HalAeIntrCallback_T callback_func,
                             void*               callback_data,
                             int                 thd_priority,
                             unsigned int        callback_priority,
                             void*               *int_handle)
{
    HalAeCallback_T *thread_data;
    HalAeCallbackChain_T *new_chain;
    int status = HALAE_SUCCESS;
    icp_firml_handle_t *handle = (icp_firml_handle_t *)callback_data;
    
    /* See if there is already a thread for this mask */
    SPIN_LOCK(handle->halHandle->hal_CallbackThdLock);
    for (thread_data = handle->halHandle->ISR_callback_root;
         thread_data;
         thread_data = thread_data->next)
    {
        HalAeCallbackChain_T *prev_chain, *curr_chain;
        if (thread_data->type_mask != type_mask) 
        {
            continue;
        }    
        /* osalMemAllocAtomic will not put the current process to sleep. */
        /* Found the right type, link us in */
        new_chain = 
             (HalAeCallbackChain_T*)osalMemAllocAtomic
                                      (sizeof(HalAeCallbackChain_T));
        if (new_chain == NULL) 
        {
            SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
            return HALAE_FAIL;
        }
        new_chain->callback_func = callback_func;
        new_chain->callback_data = callback_data;
        new_chain->priority      = callback_priority;
        new_chain->thread_data   = thread_data;
        /* insert new_chain with the highest priority first */
        for (prev_chain = &thread_data->callback_chain,
             curr_chain = prev_chain->next;
             curr_chain;
             prev_chain = curr_chain, curr_chain = curr_chain->next) 
        {
            if (curr_chain->priority < new_chain->priority) 
            {
                break;
            }    
        } /* end for prev/curr_chain */
        /* At this point, either curr_chain is NULL or a lower priority.
           Insert new_chain after prev_chain */
        new_chain->next = prev_chain->next;
        prev_chain->next = new_chain;
        *int_handle = new_chain;
        SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
        return (HALAE_SUCCESS);
    } /* end for thread_data */

    if(!(thread_data = 
         (HalAeCallback_T*)osalMemAllocAtomic(sizeof(HalAeCallback_T))))
    {
        SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
        return HALAE_FAIL;
    }
    new_chain = 
         (HalAeCallbackChain_T*)osalMemAllocAtomic
                                  (sizeof(HalAeCallbackChain_T));
    if (new_chain == NULL) 
    {
        osalMemFree(thread_data);
        SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
        return HALAE_FAIL;
    }
    if((status = halAe_IntrEnable(handle, type_mask)) != HALAE_SUCCESS)
    {
        osalMemFree(new_chain);
        osalMemFree(thread_data);
        SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
        return status;
    }

    new_chain->callback_func = callback_func;
    new_chain->callback_data = callback_data;
    new_chain->priority      = callback_priority;
    new_chain->next          = NULL;
    new_chain->thread_data   = thread_data;

    thread_data->type_mask = type_mask;
    thread_data->callback_chain.next = new_chain;
    thread_data->callback_chain.thread_data = thread_data;
    thread_data->handle = (void *)handle;
    
    /* Link thread_data into global list */
    thread_data->next = handle->halHandle->ISR_callback_root;
    handle->halHandle->ISR_callback_root = thread_data;

    /* avoid the deadlock problem: halAe_IntrCallbackThdEx uses 
     * handle->halHandle->hal_CallbackThdLock 
     * before it is released in main thread */
    SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
    
    osalSemaphoreInit(&thread_data->terminate_sem, 0);
    /* Pass threadAttr=NULL to osalThreadCreate so that the priority of 
     * the created thread will always be inherited from parent. */    
    status = osalThreadCreate(&thread_data->threadID, 
                                NULL, 
                                (OsalVoidFnVoidPtr)halAe_IntrCallbackThdEx,
                                (void *)thread_data);  
    
    if(status == OSAL_SUCCESS) 
    {
        status = osalThreadStart(&thread_data->threadID);
    }
    SPIN_LOCK(handle->halHandle->hal_CallbackThdLock);
    
    if(status != OSAL_SUCCESS) 
    {        
        halAe_IntrDisable(handle, type_mask);
        osalMemFree(new_chain);
        osalMemFree(thread_data);
        SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
        return HALAE_FAIL;
    }

    *int_handle = new_chain;

    SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Terminates callback thread if all callbacks are removed
 * @param callback_chain - IN
 *
 * @retval 
 * 
 * 
 *****************************************************************************/
static int 
terminateCallbackThdEx(icp_firml_handle_t *handle,
                       HalAeCallbackChain_T *callback_chain)
{
    int status = HALAE_FAIL;
    HalAeCallback_T *thread_data, *prev, *curr;
    HalAeCallbackChain_T *prev_chain, *curr_chain;

    thread_data = callback_chain->thread_data;

    SPIN_LOCK(handle->halHandle->hal_CallbackThdLock);
    /* remove ourselves from list, if list is empty call function below */
    for (prev_chain = &thread_data->callback_chain,
             curr_chain = prev_chain->next;
         curr_chain;
         prev_chain = curr_chain, curr_chain = curr_chain->next) 
    {
        if (curr_chain != callback_chain) 
        {
            continue;
        }    
        /* curr_chain == callback_chain, remove it */
        prev_chain->next = curr_chain->next;
        osalMemFree(curr_chain);
        status = HALAE_SUCCESS;
        break;
    } /* end for prev_chain, curr_chain */
    /* See if they are all gone */
    if (thread_data->callback_chain.next == NULL) 
    {
        /* remove thread_data from global list */
        for (prev = NULL, curr = handle->halHandle->ISR_callback_root;
             curr;
             prev = curr, curr = curr->next) 
        {
            if (curr != thread_data) 
            {
                continue;
            }    
            /* curr == thread_data, remove it */
            if (prev) 
            {
                /* In middle of list */
                prev->next = curr->next;
            } 
            else 
            {
                /* At start of list */
                handle->halHandle->ISR_callback_root = curr->next;
            }
            break;
        } /* end for prev/curr */

        /* unlock since ehalAe_TerminateCallbackThd may be blocked */
        SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);

        /* terminate thread and free thread_data */
        status = halAe_TerminateCallbackThd(handle, 
                        &thread_data->callback_chain);
    }
    else
    {
        SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);
    }

    return status;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Cancels a request initiated by the halAe_SpawnIntrCallbackThd() 
 *      function.
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
 * @param handle - IN The handle returned by a call to 
 *                    halAe_SpawnIntrCallbackThd(). This handle
 *                    specifies which request to terminate.
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_FAIL Operation failed
 * 
 * 
 *****************************************************************************/
int 
halAe_TerminateCallbackThd(icp_firml_handle_t *handle, 
                           void* int_handle)
{
    int status = HALAE_SUCCESS;
    HalAeCallbackChain_T *callback_chain;
    HalAeCallback_T *thread_data;

    callback_chain = (HalAeCallbackChain_T*)int_handle;
    thread_data = callback_chain->thread_data;
    if (thread_data->callback_chain.next) 
    {
        return terminateCallbackThdEx(handle, callback_chain);
    }
    status = halAe_IntrDisable(handle, thread_data->type_mask);

    SEM_WAIT(&thread_data->terminate_sem, OSAL_WAIT_FOREVER);
    osalSemaphoreDestroy(&thread_data->terminate_sem);

    SPIN_LOCK(handle->halHandle->hal_CallbackThdLock);
    osalMemFree(thread_data);
    SPIN_UNLOCK(handle->halHandle->hal_CallbackThdLock);

    return status;
}

/*------------------------------------
  Begin support for New Page callback
  ------------------------------------*/

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Register new page callback function.
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
 * @param callback_func - IN A pointer to the location of a callback function
 * @param user_data - IN A pointer to user input callback function data
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
void
halAe_DefineNewPageCallback(HalAePageChangeCallback_T callback_func,
                            void*                     user_data)
{
    new_page_callback      = callback_func;
    new_page_callback_data = user_data;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Calls new page callback function.
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
 * @param reason - IN Specifies the reason why callback function is invoked. 
 *                    Must be one of the following values
 *                    {START_OF_PAGE_CHANGE, NEW_PAGE_LOADED,
 *                     END_OF_PAGE_CHANGE, WAITING_FOR_PAGE_CHANGE} 
 *                      
 * @param hwAeNum - IN Specifies the accelaration engine of interest
 * @param new_page_num - IN Specifies the new page number
 * @param old_page_num - IN Specifies the old page number
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
void 
halAe_CallNewPageCallback(icp_firml_handle_t *handle, 
                          Hal_PageChangeReason_T reason,
                          unsigned int           hwAeNum,
                          unsigned int           arg0,
                          unsigned int           arg1)
{
    PageData_T   *pageData   = handle->halHandle->pageData;
    AePageData_T *aePageData = &pageData->AePageData[hwAeNum];

    if (reason == NEW_PAGE_LOADED) 
    {
        unsigned int new_page_num = arg0;
        unsigned int old_page_num = arg1;
        /* if *_page_num is -1, because it is unsigned, it will be larger
           than numPages and so the array will not be referenced */
        if (old_page_num < aePageData->numPages) 
        {
            aePageData->addrs[old_page_num].loaded = 0;
        }
        if (new_page_num < aePageData->numPages) 
        {
            aePageData->addrs[new_page_num].loaded = 1;
        }
    } /* end if reason == NEW_PAGE_LOADED */
    
    if (new_page_callback) {
        (*new_page_callback)(handle, reason, hwAeNum, arg0, arg1,
                             new_page_callback_data);
    }                             
}
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Defines UCLO callback function.
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
 * @param callback_func - IN A pointer to the location of a callback function
 * @param user_data - IN A pointer to user input callback function data
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
void
halAe_DefineUcloCallback(HalAe_UcloCall_T callback_func,
                         void*            user_data)
{
    int ii;

    for (ii=0; ii<MAX_AE; ii++) 
    {
        if (ucloCallbacks[ii].uclo_callback == NULL) 
        {
            ucloCallbacks[ii].uclo_callback = callback_func;
            ucloCallbacks[ii].uclo_callback_data = user_data;
            return;
        } 
    } /* end for ii */
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Deletes UCLO callback function.
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
 * @param callback_func - IN A pointer to the location of a callback function
 * @param user_data - IN A pointer to user input callback function data
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
void
halAe_DeleteUcloCallback(HalAe_UcloCall_T callback_func,
                         void*            user_data)
{
    int ii;

    for (ii=0; ii<MAX_AE; ii++) 
    {
        if ((ucloCallbacks[ii].uclo_callback == callback_func) &&
            (ucloCallbacks[ii].uclo_callback_data == user_data))
        {
            ucloCallbacks[ii].uclo_callback = NULL;
            return;
        } 
    } /* end for ii */
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Calls UCLO callback function.
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
 * @param reason - IN Specifies the reason why callback function is invoked. 
 *                    Must be one of the following values
 *                    {UPDATE_AE_ENABLES, PAUSING_AES,
 *                     END_OF_PAGE_CHANGE, WAITING_FOR_PAGE_CHANGE} 
 * @param arg0 - IN Callback function argument 
 * @param arg1 - IN Callback function argument 
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
void 
halAe_CallUcloCallback(Hal_UcloCallReason_T reason,
                       unsigned int         arg0,
                       unsigned int*        arg1)
{
    int ii;

    for (ii=0; ii<MAX_AE; ii++) 
    {
        if (ucloCallbacks[ii].uclo_callback != NULL)
        {
            (*ucloCallbacks[ii].uclo_callback)(reason,
                arg0,
                arg1,
                ucloCallbacks[ii].uclo_callback_data);
        } 
    } /* end for ii */
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Convert from a virtual to a physical uword address.
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
 * @param hwAeNum - IN Specifies the accelaration engine of interest
 * @param v_addr - IN Specifies the uword virtual address
 * @param p_addr - OUT A pointer to uwrod physical address
 * @param loaded - OUT A pointer to indicate wether or not the address is 
 *                     currently in ustore
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADADDR Bad address
 * 
 * 
 *****************************************************************************/
int
halAe_MapVirtToPhysUaddr(icp_firml_handle_t *handle, 
                         unsigned char hwAeNum,
                         unsigned int  vaddr,
                         unsigned int *paddr,
                         unsigned int *loaded)
{
    PageData_T   *pageData;
    AePageData_T *aePageData;
    AddrPair_T   *addrPair;
    int  ii;

    VERIFY_ARGPTR(handle);
    pageData   = handle->halHandle->pageData;

    if((loaded == NULL) || (paddr == NULL))
    {
        ERRINFO(("loaded=0x%p,paddr=0x%p\n",loaded,paddr)); 
        return (HALAE_BADARG);
    }    
    if(hwAeNum >= MAX_AE)
    {
        return (HALAE_BADARG);
    }
    
    if ((pageData == NULL) || (pageData->AePageData[hwAeNum].numPages == 0)) 
    {
        *paddr = vaddr;
        *loaded = 1;
        return (HALAE_SUCCESS);
    }
    aePageData = &pageData->AePageData[hwAeNum];

    /* Assume addrs array is in ascending vaddr order */
    /* Assume also that addrs[0].vaddr == 0 */
    for (ii = aePageData->numPages-1, addrPair = &aePageData->addrs[ii];
         ii>=0;
         ii--, addrPair--) 
    {
         if ((vaddr - addrPair->vaddr) < addrPair->size) 
         {
             break;        
         }    
    } /* end for ii, addrPair */
    
    if (ii < 0) 
    {

        /*put invalid value*/
        *paddr = (unsigned int) -1;
        *loaded = 0;
        ERRINFO(("halAe_MapVirtToPhysUaddr:  \
               vaddr (0x%x) exceeds page size: %d 0x%x 0x%x\n", \
               vaddr, ii, addrPair->vaddr, addrPair->size));
        return HALAE_BADADDR;
    }

    *paddr = addrPair->paddr + (vaddr - addrPair->vaddr);
    *loaded = addrPair->loaded;

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Convert from a virtual to a physical uword address.
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
 * @param hwAeNum - IN Specifies the accelaration engine of interest
 * @param v_addr - IN Specifies the uword virtual address
 * @param p_addr - OUT A pointer to uwrod physical address
 * @param loaded - OUT A pointer to indicate wether or not the address is 
 *                     currently in ustore
 * @param page_num - OUT A pointer to indicate page number associated with 
 *                       the physical address
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADADDR Bad address
 * 
 * 
 *****************************************************************************/
int 
halAe_MapVirtToPhysUaddrEx(icp_firml_handle_t *handle, 
                           unsigned char hwAeNum,
                           unsigned int  vaddr,
                           unsigned int *paddr,
                           unsigned int *loaded,
                           unsigned int *page_num)
{
    PageData_T   *pageData;
    AePageData_T *aePageData;
    AddrPair_T   *addrPair;
    int  ii;

    VERIFY_ARGPTR(handle);
    pageData   = handle->halHandle->pageData;

    if((loaded == NULL) || (paddr == NULL) || (page_num == NULL))
    {
        ERRINFO(("loaded=0x%p,paddr=0x%p,page_num=0x%p",
                  loaded,paddr,page_num)); 
        return (HALAE_BADARG);
    }    
    
    if ((pageData == NULL) || (pageData->AePageData[hwAeNum].numPages == 0)) 
    {
        *paddr = vaddr;
        *loaded = 1;
        *page_num = 0;
        return (HALAE_SUCCESS);
    }
    
    aePageData = &pageData->AePageData[hwAeNum];

    /* Assume addrs array is in ascending vaddr order */
    /* Assume also that addrs[0].vaddr == 0 */
    for (ii = aePageData->numPages-1, addrPair = &aePageData->addrs[ii];
         ii>=0;
         ii--, addrPair--) 
    {
         if ((vaddr - addrPair->vaddr) < addrPair->size) 
         {
             break;
         }    
    } /* end for ii, addrPair */

    if (ii < 0) 
    {

        /*put invalid value*/
        *paddr = (unsigned int) -1;
        *loaded = 0;
        *page_num = (unsigned int) -1;
        ERRINFO(("HalAe_MapVirtToPhysUaddrEx: \
               vaddr (0x%x) exceeds page size: %d 0x%x 0x%x\n", \
               vaddr, ii, addrPair->vaddr, addrPair->size));
        return HALAE_BADADDR;
    }

    *paddr = addrPair->paddr + (vaddr - addrPair->vaddr);
    *loaded = addrPair->loaded;
    *page_num = ii;

    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Convert from a physical to a virtual uword address.
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
 * @param hwAeNum - IN Specifies the accelaration engine of interest
 * @param p_addr - IN Specifies the uword physical address
 * @param v_addr - OUT A pointer to uwrod virtual address
 * @param loaded - OUT A pointer to indicate wether or not the address is 
 *                     currently in ustore
 *
 * @retval HALAE_SUCCESS Operation was successful
 * @retval HALAE_BADADDR Bad address
 * 
 * 
 *****************************************************************************/
int 
halAe_MapPhysToVirtUaddr(icp_firml_handle_t *handle, 
                         unsigned char hwAeNum,
                         unsigned int  paddr,
                         unsigned int *vaddr)
{
    PageData_T   *pageData;
    AePageData_T *aePageData;
    AddrPair_T   *addrPair;
    int           ii;

    VERIFY_ARGPTR(handle);
    pageData   = handle->halHandle->pageData;

    if(vaddr == NULL)
    {
        ERRINFO(("bad argument vaddr is NULL\n ")); 
        return (HALAE_BADARG);
    }    
    if(hwAeNum >= MAX_AE)
    {
        return (HALAE_BADARG);
    }
    
    if ((pageData == NULL) || (pageData->AePageData[hwAeNum].numPages == 0)) 
    {
        *vaddr = paddr;
        return (HALAE_SUCCESS);
    }

    aePageData = &pageData->AePageData[hwAeNum];    

    for (ii = aePageData->numPages-1, addrPair = &aePageData->addrs[ii];
         ii>=0;
         ii--, addrPair--) 
    {
        if ((addrPair->loaded) &&
            (addrPair->paddr <= paddr) &&
            (addrPair->paddr + addrPair->size > paddr))
        {
            *vaddr = addrPair->vaddr + (paddr - addrPair->paddr);
            return (HALAE_SUCCESS);
        }
    } /* end for ii, addrPair */
    ERRINFO(("bad argument vaddr=%p\n ",vaddr));
    return HALAE_BADADDR;
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Set the number of pages and allocate arrays
 *
 * @param hwAeNum - IN
 * @param numPages - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_FAIL
 * 
 * 
 *****************************************************************************/
int 
halAe_SetNumPages(icp_firml_handle_t *handle, 
                  unsigned char hwAeNum, 
                  unsigned int numPages)
{
    PageData_T   *pageData;
    AePageData_T *aePageData;
    AddrPair_T   *addrPair;
    unsigned int ii;

    VERIFY_ARGPTR(handle);
    if(hwAeNum >= MAX_AE)
    {
        return (HALAE_BADARG);
    }
    pageData   = handle->halHandle->pageData;
    aePageData = &pageData->AePageData[hwAeNum]; 

    aePageData->addrs = (AddrPair_T*) osalMemAlloc(sizeof(AddrPair_T)*numPages);
    if (aePageData->addrs == NULL) 
    {
        return HALAE_FAIL;
    }    
    aePageData->numPages = numPages;
    for (ii=0, addrPair = &aePageData->addrs[0];
         ii<numPages;
         ii++, addrPair++) 
    {
        addrPair->vaddr = 0;
        addrPair->paddr = 0;
        addrPair->size = 0;
        addrPair->loaded = 0;
    }
    return (HALAE_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Set one page entry
 *
 * @param hwAeNum - IN
 * @param page - IN
 * @param vaddr - IN
 * @param paddr - IN
 * @param size - IN
 *
 * @retval  HALAE_SUCCESS, HALAE_BADARG
 * 
 * 
 *****************************************************************************/
int 
halAe_SetPageData(icp_firml_handle_t *handle, 
                  unsigned char hwAeNum, 
                  unsigned int page,
                  unsigned int vaddr, 
                  unsigned int paddr, 
                  unsigned int size)
{
    PageData_T   *pageData;
    AePageData_T *aePageData;
    AddrPair_T   *addrPair;

    VERIFY_ARGPTR(handle);
    if(hwAeNum >= MAX_AE)
    {
        return (HALAE_BADARG);
    }
    pageData   = handle->halHandle->pageData;
    aePageData = &pageData->AePageData[hwAeNum];
    addrPair   = &aePageData->addrs[page];

    if (page >= aePageData->numPages) 
    {
        return (HALAE_BADARG);
    }
    addrPair->vaddr = vaddr;
    addrPair->paddr = paddr;
    addrPair->size  = size;
    addrPair->loaded = 0;
    return (HALAE_SUCCESS);
}


/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the error description string for provided error code.
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
 * @param errCode - IN Specifies the error code
 *
 * @retval error description string
 * 
 * 
 *****************************************************************************/
char *
halAe_GetErrorStr(int errCode)
{
    switch(errCode)
    {
    case HALAE_SUCCESS:         
            return ("The operation is successful.");
    case HALAE_FAIL:
            return ("The operation is failed.");        
    case HALAE_BADARG:
            return ("Bad function argument passed.");        
    case HALAE_DUPBKP:
        return ("Add duplicated breakpoint.");        
    case HALAE_NOUSTORE:
        return ("No free ustore available.");
    case HALAE_BADADDR:
        return ("Bad address argument passed.");
    case HALAE_BADLIB:
        return ("HAL library isn't initialized.");
    case HALAE_DISABLED:
        return ("Acceleration Engine is disabled.");
    case HALAE_ENABLED:
        return ("Acceleration Engine is enabled.");
    case HALAE_RESET:
        return ("Acceleration Engine is in reset.");
    case HALAE_TIMEOUT:
        return ("The operation times out.");
    case HALAE_ISSET:
        return ("Condition/evaluation is set/true.");
    case HALAE_NOTSET:
        return ("Condition/evaluation is not set/false.");
    case HALAE_AEACTIVE:
        return ("Acceleration Engine is running.");
    case HALAE_MEMALLOC:
        return ("Memory allocation error.");
    case HALAE_NEIGHAEACTIVE:
        return ("Neighbouring ae is running.");        
    default:
        return (NULL);
    }        
}

