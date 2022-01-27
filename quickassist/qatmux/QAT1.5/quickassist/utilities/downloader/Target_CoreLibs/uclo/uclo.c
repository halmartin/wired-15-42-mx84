/**
 **************************************************************************
 * @file uclo.c
 *
 * @description
 *      This file provides Ucode Object File Loader facilities
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

#include "uclo_platform.h"
#include "core_io.h"
#include "uclo_dev.h"
#include "uclo.h"
#include "uclo_helper.h"
#include "uof_dbg.h"
#include "uof_prf.h"
#include "uclo_status.h"

#include "hal_ae.h"
#include "halMem.h"
#include "halMemScrub.h"
#include "halAeApi.h"
#include "icp_firml_handle.h"
#include "uclo_overlay.h"
#include "suof.h"
#include "uclo_suof.h"

#define USTORE_8K        0x2000            /* 8 kbytes ustore size */    
#define USTORE_16K       0x4000            /* 16 kbytes ustore size */ 
#define UOF_CFID_CC      0xCC              /* .uof file ID */
#define UOF_CFID_C3      0xC3              /* .uof file ID */
#define PRE_CUR_MAJVER   1                 /* .uof major version */ 
#define PRE_CUR_MINVER   2                 /* .uof minor version */

#define KILO_4 (0x400 << 2)

#define UCLO_ALLAE       0xff0fff          /* all AE mask */

#define BAD_EP80579_HWAE_MASK     0xfffffff0
#define BAD_ACCEL_COMP_HWAE_MASK  0xffffff00

#define UWORD_CPYBUF_SIZE 1024    /* micro-store copy buffer size(bytes) */

#define UENGINE_ID "__uengine_id"          /* micro engine ID */  

#define ADD_ADDR(high,low)  ((((uint64)high) << 32) + low)
#define BITS_IN_DWORD 0x20

/* micro-store dram base address symbol */
#define USTORE_DRAM_BASE_SYM "__USTORE_DRAM_BASE"

extern int UcloExp_evalPostfix(icp_firml_handle_t *handle,
                               unsigned char swAe,
                               char *pExp, 
                               int *pRes);

static unsigned int LmemByteSize=(MAX_ICP_LMEM_REG << 0x2);    /* 2560 bytes */
unsigned int BadHwAeMask = BAD_ACCEL_COMP_HWAE_MASK;

unsigned int UcLo_getSwAe(unsigned int prodType, 
                          unsigned char aeNum);
unsigned int UcLo_getHwAe(unsigned int prodType, 
                          unsigned char aeNum);
int UcLo_getSharedUstoreNeigh(uclo_objHandle_T *objHandle, 
                          unsigned int hwAeNum);
int UcLo_WriteNumUword(icp_firml_handle_t *handle, 
                       unsigned int aeMask, 
                       unsigned int phyUaddr,
                       unsigned int numWords, 
                       uword_T *uWord);
char *UcLo_getString(uof_strTab_T *strTable, 
                     unsigned int strOffset);
static void UcLo_wrScratchBytes(icp_firml_handle_t *handle,
                                unsigned int addr, 
                                unsigned int *val,
                                unsigned int numBytes);
uword_T Uclo_getUofUword_nocheck(uclo_objHandle_T *objHandle, 
                                 uof_encapPage_T *page, 
                                 unsigned int addr);
uword_T Uclo_getUofUword(uclo_objHandle_T *objHandle, 
                         uof_encapPage_T *page, 
                         unsigned int addr);
int UcLo_computeFreeUstore(icp_firml_handle_t *handle);
static int UcLo_fixupExpr(icp_firml_handle_t *handle, 
                          uof_encapPage_T *page,
                          unsigned char swAe);
uof_encapAe_T *UcLo_getEncapImage(uclo_objHandle_T *objHandle, 
                                  char *ucodeImageName);
int UcLo_GetAeSliceImageName(icp_firml_handle_t *handle, 
                             unsigned char hwAe, unsigned int slice,
                             uclo_imageassign_t *imageAssign);
void UcLo_wrScratch(icp_firml_handle_t *handle,
                    unsigned int byteAddr, 
                    unsigned int *val, 
                    unsigned int count);                   
int UcLo_fixupLocals(icp_firml_handle_t *handle, 
                     uof_encapPage_T *page,
                     unsigned char swAe);
static int UcLo_fixupLocalNeighReg(uclo_objHandle_T *objHandle, 
                                   uof_encapPage_T *page,
                                   unsigned char swAe);
static int UcLo_fixupGlobals(uclo_objHandle_T *objHandle, 
                             uof_encapPage_T *page);
static int UcLo_initGlobals(icp_firml_handle_t *handle);
static int UcLo_initReg(icp_firml_handle_t *handle,
                        unsigned char hwAe, 
                        unsigned char ctxMask, 
                        icp_RegType_T regType,
                        unsigned short regAddr, 
                        unsigned int value);
static int UcLo_initSym(icp_firml_handle_t *handle, 
                        unsigned char hwAe, 
                        char *symName,
                        unsigned int offset, 
                        unsigned int value);
int UcLo_bindIvd(void *objHandle);
static int UcLo_initRegSymExpr(icp_firml_handle_t *handle, 
                               unsigned char hwAe, 
                               uof_encapAe_T *encapAe);
void UcLo_ProcessHalCallbacks(Hal_UcloCallReason_T reason,
                              unsigned int         arg0,
                              unsigned int*        arg1,
                              void*                user_data);

void halAe_setUofChecksum(icp_firml_handle_t *handle,
                          unsigned int uofChecksum);
                               
uclo_objHandle_T *UcLo_getObjHandle(void);
uclo_suof_handle_t *UcLo_getSObjHandle(void);
int UcLo_overlayObj(icp_firml_handle_t *handle, 
                    int readOnly);

int UcLo_isProfileSupported(void);
static int UcLo_initUstore(icp_firml_handle_t *handle);

int halMem_PutSharedRam(icp_firml_handle_t *Handle, 
                        unsigned int QatNum, 
                        unsigned int ae,
                        unsigned int ShramAddr,
                        unsigned int Size,
                        unsigned int *Buffer,
                        unsigned int EndianSwap);
int halMem_SharedRamToDRam(icp_firml_handle_t *Handle, 
                           unsigned int QatNum, 
                           unsigned int ae,
                           unsigned int ShramAddr,
                           unsigned int Size,
                           unsigned int *DRamAddr,
                           unsigned int EndianSwap);

int halAe_ContinuousDramAlloc(icp_firml_handle_t *Handle,
                          icp_firml_dram_desc_t *pDram_desc,
                          unsigned int size);
void halAe_ContinuousDramFree(icp_firml_handle_t *Handle, 
                          icp_firml_dram_desc_t *pDram_desc);

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     return the AE number as used by this software.
                EP80579 AE numbering is 0x00-0x03
                Internal AE numbering is 0x00-0x0f
 *
 * @param prodType - IN
 * @param aeNum - IN
 *
 * @retval  UCLO_BADAE, or SW AE number
 * 
 * 
 *****************************************************************************/
unsigned int 
UcLo_getSwAe(unsigned int prodType, 
             unsigned char aeNum)
{
    if(((unsigned int)aeNum >= MAX_AE)
       || ((1 << (unsigned int)aeNum) & BadHwAeMask)) 
    {
        return (UCLO_BADAE);
    }    

    if(prodType & (EP80579_CPU_TYPE)) 
    {
        aeNum = aeNum & 0x3;
    }
    else if(prodType & (ACCEL_COMP_TYPE)) 
    {
        aeNum = aeNum & 0x7;
    }        
    else if(prodType & (ACCELCOMP_C_CPU_TYPE)) 
    {
        aeNum = aeNum & 0xf;
    }        
    else if(prodType & (ACCELCOMP_B_CPU_TYPE)) 
    {
        aeNum = aeNum & 0xf;
    }    
    else if(prodType & (ACCELCOMP_RS_CPU_TYPE)) 
    {
        aeNum = aeNum & 0xf;
    }
    else if(prodType & (ACCELCOMP_R_CPU_TYPE)) 
    {
        aeNum = aeNum & 0x1;
    }        
    else 
    { 
        return (UCLO_BADAE);
    }
    return (aeNum);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *    return the AE number as used by the hadrware.
                EP80579 AE numbering is 0x00-0x03
                Internal AE numbering is 0x00-0x0f
 *
 * @param prodType - IN
 * @param aeNum - IN
 *
 * @retval  UCLO_BADAE, or the HW AE number
 * 
 * 
 *****************************************************************************/
unsigned int 
UcLo_getHwAe(unsigned int prodType, 
             unsigned char aeNum)
{
    if(((unsigned int)aeNum >= MAX_AE)
       || ((1 << (unsigned int)aeNum) & BadHwAeMask)) 
    {
        return (UCLO_BADAE);
    }    
    if(prodType & (EP80579_CPU_TYPE)) 
    {
        aeNum = aeNum & 0x3;
    }
    else if(prodType & (ACCEL_COMP_TYPE)) 
    {
        aeNum = aeNum & 0x7;
    }        
    else if(prodType & (ACCELCOMP_C_CPU_TYPE)) 
    {
        aeNum = aeNum & 0xf;
    }        
    else if(prodType & (ACCELCOMP_B_CPU_TYPE)) 
    {
        aeNum = aeNum & 0xf;
    } 
    else if(prodType & (ACCELCOMP_RS_CPU_TYPE)) 
    {
        aeNum = aeNum & 0xf;
    }
    else if(prodType & (ACCELCOMP_R_CPU_TYPE)) 
    {
        aeNum = aeNum & 0x1;
    }        
    else 
    {
        return (UCLO_BADAE);
    }
    return (aeNum);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *    Gets string from the string table.
 *
 * @param strTable - IN
 * @param strOffset - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
char *
UcLo_getString(uof_strTab_T *strTable, 
               unsigned int strOffset)
{
    if((!strTable->tableLen) || (strOffset > strTable->tableLen)) 
    {
        return (NULL);
    }    
    return ((char *)(((unsigned long)(strTable->strings)) + strOffset));
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *    Set the file format version in the handle.
 *
 * @param minVer - IN
 * @param majVer - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_UOFVERINCOMPAT
 * 
 * 
 *****************************************************************************/
static int 
UcLo_verifyVersion(short *minVer, 
                   short *majVer)
{
    if((*majVer == UOF_CFID_C3) && (*minVer == UOF_CFID_CC))
    {
        ERRINFO(("bad uof version, majVer=0x%x, minVer=0x%x\n",
                 UOF_CFID_C3,UOF_CFID_CC));
        /* pre UOF v2.0 supported versions... Releases 1.2, & 1.3 */
        *minVer = PRE_CUR_MINVER;
        *majVer = PRE_CUR_MAJVER;
        return (UCLO_UOFVERINCOMPAT);
    }
    else
    { 
      if((*minVer != UOF_MINVER) || (*majVer != UOF_MAJVER)) 
      {
          ERRINFO(("bad uof version, majVer=0x%x, minVer=0x%x\n",
                   *majVer,*minVer));  
          return (UCLO_UOFVERINCOMPAT);
      }
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      First, the function verifies that the file is an UOF. If it is, then 
 *      the UOF format version of the file is returned in the minVer and majVer
 *      arguments.
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
 * @param fileHdr - IN A handle to the file 
 * @param minVer - IN The UOF format minor version
 * @param majVer - IN The UOF format major version
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_UOFINCOMPAT The UOF is incompatible with this version of 
 *                          the Loader library
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/
int 
UcLo_verifyFile(char *fileHdr, 
                short *minVer, 
                short *majVer)
{
    uof_fileHdr_T    fHdr;

    /* copy fileHdr so we can modified it without any reprecussion */
    memmove(&fHdr, fileHdr, sizeof(uof_fileHdr_T));

    /* check new format proper, endian */
    if(fHdr.fileId != UOF_FID) 
    {
        ERRINFO(("fHdr.fileId=0x%x\n", fHdr.fileId));  
        return (UCLO_FAILURE);
    }
    
    *minVer = fHdr.minVer & 0xff; /* The valid data of minVer is 8 LSBs */
    *majVer = fHdr.majVer & 0xff; /* The valid data of majVer is 8 LSBs */
    return (UcLo_verifyVersion(minVer, majVer));
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Write a specified number of bytes to SRAM.
 *
 * @param addr - IN
 * @param val - IN
 * @param numBytes - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
UcLo_wrSramBytes(icp_firml_handle_t *handle,
                 uint64 addr, 
                 unsigned int *val, 
                 unsigned int numBytes)
{
    unsigned int  outVal=0x0, cpSize=0x4;
    unsigned char *chPtr=(unsigned char *)val;

    while(numBytes)
    {
        if(numBytes < 0x4)
        {
            cpSize = numBytes;
            outVal = SRAM_READ(handle, addr);
        }
        memmove(&outVal, chPtr, cpSize);
        SRAM_WRITE(handle, addr, outVal);
        numBytes -= cpSize;
        chPtr += cpSize;
        addr += cpSize;
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Write a specified number of bytes to DRAM.
 *
 * @param channel - IN
 * @param addr - IN
 * @param val - IN
 * @param numBytes - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
UcLo_wrDramBytes(icp_firml_handle_t *handle,
                 unsigned int channel, 
                 unsigned int addr,
                 unsigned int *val, 
                 unsigned int numBytes)
{
    unsigned int  outVal, cpSize=0x4;
    unsigned char *chPtr=(unsigned char *)val;

    while(numBytes)
    {
        if(numBytes < 0x4)
        {
            cpSize = numBytes;
            if(channel == 0) 
            {
                 outVal = DRAM_READ_CH0(handle, addr);
            }    
            else
            {
                 outVal = DRAM_READ_CH1(handle, addr);
            }     
        }
        memmove(&outVal, chPtr, cpSize);
        DRAM_WRITE_XA(handle, channel, addr, outVal);
        numBytes -= cpSize;
        chPtr += cpSize;
        addr += cpSize;
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Write a specified number of bytes to SCRATCH.
 *
 * @param addr - IN
 * @param val - IN
 * @param numBytes - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
UcLo_wrScratchBytes(icp_firml_handle_t *handle,
                    unsigned int addr, 
                    unsigned int *val,
                    unsigned int numBytes)
{
    unsigned int  outVal, cpSize=0x4;
    unsigned char *chPtr=(unsigned char *)val;

    while(numBytes)
    {
        if(numBytes < 0x4)
        {
            cpSize = numBytes;
            outVal = SCRATCH_READ(handle, addr);
        }
        memmove(&outVal, chPtr, cpSize);
        SCRATCH_WRITE(handle, addr, outVal);
        numBytes -= cpSize;
        chPtr += cpSize;
        addr += cpSize;
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Write a specified number of bytes to USTORE-MEM starting at the
  *              byte-address specified by addr.
 *
 * @param hwAe - IN
 * @param addr - IN
 * @param val - IN
 * @param numBytes - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
UcLo_wrUstoreMemBytes(icp_firml_handle_t *handle,
                      unsigned char hwAe, 
                      unsigned int addr,
                      unsigned int *val, 
                      unsigned int numBytes)
{
    unsigned int  outVal, cpSize=0x4;
    unsigned char *chPtr=(unsigned char *)val;
    
    addr >>= 0x2; /* convert to uword address */

    while(numBytes)
    {
        if(numBytes < 0x4)
        {
            cpSize = numBytes;
            halAe_GetUmem(handle, hwAe, addr, 1, &outVal);
        }
        memmove(&outVal, chPtr, cpSize);
        halAe_PutUmem(handle, hwAe, addr++, 1, &outVal);
        numBytes -= cpSize;
        chPtr += cpSize;
    }

    return;
}

static void
UcLo_BatchWriteUstoreMem(icp_firml_handle_t *handle,
                         unsigned char ae,
                         batch_init_t *umem_init_header)
{
    batch_init_t *plmInit;
    unsigned int addr, *value, size; 

    if (umem_init_header == NULL) {
        return;
    }
    plmInit = umem_init_header->next;
    while (plmInit) {
        ae = plmInit->hwAe;
        addr = plmInit->addr;
        value = plmInit->value;
          /* here size > 0 */
        size = plmInit->size;
        UcLo_wrUstoreMemBytes(handle, ae,  addr, value, size);

        plmInit = plmInit->next;
    }

    return;

}


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Write a specified number of longwords to USTORE-MEM
 *
 * @param handle - IN
 * @param hwAe - IN
 * @param byteAddr - IN
 * @param val - IN
 * @param count - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
UcLo_wrUstoreMem(icp_firml_handle_t *handle,
                      unsigned char hwAe, 
                      unsigned int byteAddr,
                      unsigned int *val, 
                      unsigned int count)
{
    unsigned int i = 0;
    unsigned int ustoreAddr;
    byteAddr >>= 0x2;
    for(i = 0; i < count; i++)
    {
        ustoreAddr = byteAddr+i;
        halAe_PutUmem(handle, hwAe, ustoreAddr, 1, &val[i]);
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Write a specified number of longwords to SRAM
 *
 * @param addr - IN
 * @param val - IN
 * @param count - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
UcLo_wrSram(icp_firml_handle_t *handle,
            unsigned int addr, 
            unsigned int *val, 
            unsigned int count)
{
    unsigned int i=0;

    for(i = 0; i < count; i++)
    {
        SRAM_WRITE(handle, addr+(i<<0x2), val[i]);
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Write a specified number of longwords to DRAM
 *
 * @param channel - IN
 * @param addr - IN
 * @param val - IN
 * @param count - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
UcLo_wrDram(icp_firml_handle_t *handle,
            unsigned int channel, 
            unsigned int addr, 
            unsigned int *val, 
            unsigned int count)
{
    unsigned int i=0;

    for(i = 0; i < count; i++)
    {
        DRAM_WRITE_XA(handle, channel, addr+(i<<0x2), val[i]);
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Write a specified number of longwords to SCRATCH PAD MEMORY
 *
 * @param byteAddr - IN
 * @param val - IN
 * @param count - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
void 
UcLo_wrScratch(icp_firml_handle_t *handle,
               unsigned int byteAddr, 
               unsigned int *val, 
               unsigned int count)
{
    unsigned int i = 0;

    for(i = 0; i < count; i++)
    {
        SCRATCH_WRITE(handle, byteAddr+(i<<0x2), val[i]);
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Write a specified number of longwords to LOCAL-MEMORY
 *
 * @param hwAe - IN
 * @param byteAddr - IN
 * @param val - IN
 * @param count - IN
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
static void 
UcLo_wrLocalMem(icp_firml_handle_t *handle,
                unsigned char hwAe, 
                unsigned int byteAddr,
                unsigned int *val, 
                unsigned int count)
{
    unsigned int i = 0;
    unsigned short lmaddr;

    byteAddr >>= 0x2;
    for(i = 0; i < count; i++)
    {
        lmaddr = (unsigned short)(byteAddr+i);
        halAe_PutLM(handle, hwAe, lmaddr, val[i]);
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Locates a memory-resident symbol
 *
 * @param objHandle - IN
 * @param symName - IN
 *
 * @retval  Pointer to the symbol, or NULL
 * 
 * 
 *****************************************************************************/
uof_initMem_T *
UcLo_findMemSym(uclo_objHandle_T *objHandle, 
                char *symName)
{
    unsigned int i;
    uof_initMem_T *initMem;
    char          *str;

    if(!objHandle) {
        return (NULL);
    }    

    initMem = objHandle->initMemTab.initMem;
    for(i = 0; i < objHandle->initMemTab.numEntries; i++)
    {
        str = UcLo_getString(&objHandle->strTable, initMem->symName);
        if(!str)
        {
            continue;
        }    
        if(!strcmp(str, symName)) 
        {
            return (initMem);
        }    
        initMem = (uof_initMem_T *)((unsigned long)((unsigned long)initMem +
                                                    sizeof(uof_initMem_T)) +
                              (sizeof(uof_memValAttr_T) * initMem->numValAttr));
    }
    return (NULL);
}

static int
cleanupLmInitLink(icp_firml_handle_t *handle, int ae)
{
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    batch_init_t *plmInit, *pre;

    plmInit = objHandle->lmInitTab[ae];
    while (plmInit) {
        pre = plmInit;
        plmInit = plmInit->next;
        osalMemFree(pre);
    }
    objHandle->lmInitTab[ae] = NULL;

    return (UCLO_SUCCESS);
}

static int
cleanupUmemInitLink(icp_firml_handle_t *handle, int ae)
{
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    batch_init_t *plmInit, *pre;

    plmInit = objHandle->umemInitTab[ae];
    while (plmInit) {
        pre = plmInit;
        plmInit = plmInit->next;
        osalMemFree(pre);
    }
    objHandle->umemInitTab[ae] = NULL;

    return (UCLO_SUCCESS);
}


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Init the memory of a symbol
 *
 * @param objHandle - IN
 * @param initMem - IN
 *
 * @retval UCLO_EXCDSRAMSIZE, UCLO_EXCDDRAMSIZE, UCLO_EXCDSCRTHSIZE,
                UCLO_EXCDUMEMSIZE, UCLO_FAILURE, UCLO_BADOBJ, UCLO_SUCCESS
 * 
 * 
 *****************************************************************************/
static int 
initMemory(icp_firml_handle_t *handle, 
           uof_initMem_T *initMem)
{
    unsigned int i, size=0x4, hwAe, swAe, uAddr, s;
    unsigned int ustoreSize;
    uof_memValAttr_T *memValAttr;
    unsigned int ScratchOffset, SramOffset;
    unsigned int dram0Offset, dram1Offset;

    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    batch_init_t *lm_init_header, *last_link, *plmInit;
    batch_init_t *umem_init_header;

    memValAttr = (uof_memValAttr_T *)((unsigned long)initMem +
                                      sizeof(uof_initMem_T));
    halAe_GetMemoryStartOffset(handle, &ScratchOffset,
                               &SramOffset, &dram0Offset, &dram1Offset); 

    switch(initMem->region)
    {
    /* write the bytes values to the appropriate memory region.
       The values are stored as bytes and should be written as such to avoid
       system-endian conversion */
    case SRAM_REGION:
        if((SramOffset + initMem->addr + initMem->numBytes)
           > (handle->sysMemInfo.sramDesc.sramSize)) 
        {
            ERRINFO(("addr = 0x%x, SramOffset=0x%x, numBytes = 0x%x\n",
                    initMem->addr,SramOffset,initMem->numBytes )); 
            return (UCLO_EXCDSRAMSIZE);
        }    
        for(i=0; i < initMem->numValAttr; i++)
        {
            UcLo_wrSramBytes(handle,
                            (uint64)(SramOffset + 
                            initMem->addr + memValAttr->byteOffset),
                            &memValAttr->value, size);
            memValAttr++;
        }
        break;
    case DRAM_REGION:
        /* offset into the dram0 region */
        if((initMem->addr < handle->sysMemInfo.dramDesc[0].aeDramOffset)
           || ((dram0Offset + initMem->addr + initMem->numBytes) 
               > (handle->sysMemInfo.dramDesc[0].aeDramOffset
                  + handle->sysMemInfo.dramDesc[0].dramSize))) 
        {
            ERRINFO(("addr=0x%x,dram0Offset=0x%x, numBytes =0x%x\n ",
                     initMem->addr,dram0Offset,initMem->numBytes));  
            return (UCLO_EXCDDRAMSIZE);
        }    
        for(i=0; i < initMem->numValAttr; i++)
        {
            UcLo_wrDramBytes(handle, 0,
                           dram0Offset + initMem->addr + memValAttr->byteOffset,
                           &memValAttr->value, size);
            memValAttr++;
        }
        break;
    case DRAM1_REGION:
        /* offset into the dram1 region */
        if((initMem->addr < handle->sysMemInfo.dramDesc[1].aeDramOffset)
           || ((dram1Offset + initMem->addr + initMem->numBytes)
               > (handle->sysMemInfo.dramDesc[1].aeDramOffset
                  + handle->sysMemInfo.dramDesc[1].dramSize))) 
        {
            ERRINFO(("addr=0x%x,dram1Offset=0x%x, numBytes =0x%x\n ",
                     initMem->addr,dram1Offset,initMem->numBytes));  
            return (UCLO_EXCDDRAM1SIZE);
        }    
        for(i=0; i < initMem->numValAttr; i++)
        {
            UcLo_wrDramBytes(handle, 1,
                           dram1Offset + initMem->addr + memValAttr->byteOffset,
                           &memValAttr->value, size);
            memValAttr++;
        }
        break;
    case SCRATCH_REGION:
        if((ScratchOffset + initMem->addr + initMem->numBytes)
           > (handle->sysMemInfo.scratchDesc.scratchSize)) 
        {
            ERRINFO(("addr=0x%x,ScratchOffset=0x%x, numBytes=0x%x\n",
                     initMem->addr,ScratchOffset,initMem->numBytes)); 
            return (UCLO_EXCDSCRTHSIZE);
        }    
        for(i=0; i < initMem->numValAttr; i++)
        {
            UcLo_wrScratchBytes(handle, 
                         ScratchOffset + initMem->addr + memValAttr->byteOffset,
                         &memValAttr->value, size);
            memValAttr++;
        }
        break;
    case LMEM_REGION:
        if((initMem->addr + initMem->numBytes) > LmemByteSize) 
        {
            ERRINFO(("addr=0x%x,numBytes=0x%x\n",
                     initMem->addr, initMem->numBytes)); 
            return (UCLO_EXCDLMEMSIZE);
        }    
        if(initMem->scope != LOCAL_SCOPE) 
        {
            ERRINFO(("Memory scope =0x%x\n",initMem->scope )); 
            return (UCLO_BADOBJ);
        }    
        if(UcLo_parseNum(UcLo_getString(&(objHandle->strTable),
                                        initMem->symName),
                         (int *)(&hwAe))) 
        {
            ERRINFO((" UcLo_parseNum error\n"));  
            return (UCLO_BADOBJ);
        }    
        if(!isBitSet(handle->sysMemInfo.aeMask,
                    hwAe))
        {
            DBGINFO(("Init LM of the fused-out hwAe=0x%x\n", hwAe));
            return (UCLO_SUCCESS);
        }
        if(hwAe >= MAX_AE)
        {
            ERRINFO(("hwAe =0x%x\n", hwAe));  
            return (UCLO_BADOBJ);                
        }    
        lm_init_header = objHandle->lmInitTab[hwAe];
        if (lm_init_header == NULL) {
            lm_init_header = (batch_init_t *)osalMemAlloc(sizeof(batch_init_t));
            if (lm_init_header == NULL) {
                ERRINFO(("Memory allocation failed for lm_init_header\n"));
                return(UCLO_FAILURE);
            }
            osalMemSet(lm_init_header, 0, sizeof(batch_init_t));
              /* at least the ctx_arb[kill] is needed */
            lm_init_header->size = 1;  
            objHandle->lmInitTab[hwAe] = lm_init_header;
        }

        for(i=0; i < initMem->numValAttr; i++)
        {
            plmInit = (batch_init_t *)osalMemAlloc(sizeof(batch_init_t));
            if (plmInit == NULL) {
                ERRINFO(("Memory allocation failed for plmInit in cycle %d\n", 
                                  i));
                return(UCLO_FAILURE);
            }
            plmInit->hwAe = hwAe;
            plmInit->addr = initMem->addr + memValAttr->byteOffset;
            plmInit->value = &memValAttr->value;
            plmInit->size = size;
            plmInit->next = NULL;
          

            if (plmInit->size > 0) {
               /* insert this data to the LM initalization table */
                last_link = lm_init_header;
                while (last_link->next) {
                    last_link = last_link->next;
                }
                last_link->next = plmInit;

                /* the header will use this field to record the number */
                lm_init_header->hwAe ++;
                lm_init_header->size += halAe_GetInstrucNum(size);
            }

            memValAttr++;
        }
        break;
    case UMEM_REGION:
        if(initMem->scope != LOCAL_SCOPE)
        {
            ERRINFO(("Memory scope =0x%x\n",initMem->scope ));  
            return (UCLO_BADOBJ);
        }    
        if(UcLo_parseNum(UcLo_getString(&(objHandle->strTable),
                                        initMem->symName),
                         (int *)(&hwAe))) 
        {
            ERRINFO(("UcLo_parseNum error\n "));  
            return (UCLO_BADOBJ);
        }    
        if(!isBitSet(handle->sysMemInfo.aeMask,
                    hwAe))
        {
            DBGINFO(("Init UMEM of the fused-out hwAe=0x%x\n", hwAe));
            return (UCLO_SUCCESS);
        }
        if(hwAe >= MAX_AE)
        {
            ERRINFO(("hwAe =0x%x\n", hwAe));  
            return (UCLO_BADOBJ);
        }    

        ustoreSize = 
                (objHandle)->aeData[objHandle->swAe[hwAe]].effectUstoreSize;
        if((initMem->addr + initMem->numBytes) > (ustoreSize << 0x2)) 
        {
            ERRINFO(("addr=0x%x,numBytes=0x%x\n",
                     initMem->addr, initMem->numBytes));
            return (UCLO_EXCDUMEMSIZE);
        }
        
        umem_init_header = objHandle->umemInitTab[hwAe];
        if (umem_init_header == NULL) {
            umem_init_header = (batch_init_t *)osalMemAlloc \
                            (sizeof(batch_init_t));
            if (umem_init_header == NULL) {
                ERRINFO(("Memory allocation failed for umem_init_header\n"));
                return(UCLO_FAILURE);
            }
            osalMemSet(umem_init_header, 0, sizeof(batch_init_t));
            objHandle->umemInitTab[hwAe] = umem_init_header;
        }

        for(i=0; i < initMem->numValAttr; i++)
        {
            plmInit = (batch_init_t *)osalMemAlloc(sizeof(batch_init_t));
            if (plmInit == NULL) {
                ERRINFO(("Memory allocation failed for plmInit in cycle %d\n",
                              i));
                return(UCLO_FAILURE);
            }
            plmInit->hwAe = hwAe;
            plmInit->addr = initMem->addr + memValAttr->byteOffset;
            plmInit->value = &memValAttr->value;
            plmInit->size = size;
            plmInit->next = NULL;


            if (plmInit->size > 0) {
               /* insert this data to the LM initalization table */
                last_link = umem_init_header;
                while (last_link->next) {
                    last_link = last_link->next;
                }
                last_link->next = plmInit;

                /* the header will use this field to record the number */
                umem_init_header->hwAe ++;
                umem_init_header->size += halAe_GetInstrucNum(size);
            }

            memValAttr++;
        }
        /* set the highest ustore address referenced */
        if((swAe = UcLo_getSwAe(objHandle->prodType, (unsigned char)hwAe))
           == UCLO_BADAE) 
        {
            ERRINFO(("UcLo_getSwAe error\n "));  
            return (UCLO_BADARG);
        }    
        uAddr = (initMem->addr + initMem->numBytes) >> 0x2;
        for(s=0; s < objHandle->aeData[swAe].numSlices; s++)
        {
            if(objHandle->aeData[swAe].aeSlice[s].encapImage->numUwordsUsed
               < uAddr) 
            {
                objHandle->aeData[swAe].aeSlice[s].encapImage->numUwordsUsed =
                                            uAddr;
            }    
        }
        break;
    case SHRAM_REGION:
    case SHRAM1_REGION:
    case SHRAM2_REGION:
    case SHRAM3_REGION:
    case SHRAM4_REGION:
    case SHRAM5_REGION:
        if(!(handle->sysMemInfo.qatMask & 
           (1 << (initMem->region - SHRAM_REGION))))
        {
            return (UCLO_BADOBJ);
        }
        /* the loader is not supposed to initialize the shram region and
         * this code won't be executed in case if we want to support it
         * in future, just add code to initialize the shram region */
        if((initMem->addr + initMem->numBytes) > MAX_SSU_SHARED_RAM) 
        {
            ERRINFO(("addr = 0x%x, numBytes=0x%x\n",
                     initMem->addr,initMem->numBytes));
            return (UCLO_EXCDSHRAMSIZE);
        }    
        break;
    default:
        ERRINFO(("region type=0x%x\n", initMem->region));
        return (UCLO_BADOBJ);
    }

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Initialize allocated memory
 *
 * @param objHandle - IN
 *
 * @retval UCLO_SUCCESS, UCLO_BADARG, UCLO_FAILURE, UCLO_EXCDLMEMSIZE,
                 UCLO_EXCDUMEMSIZE, UCLO_EXCDSRAMSIZE, UCLO_EXCDDRAMSIZE, 
                 UCLO_EXCDSCRTHSIZE
 * 
 * 
 *****************************************************************************/
static int 
UcLo_initMemory(icp_firml_handle_t *handle)
{
    unsigned int    i;
    int             status = UCLO_SUCCESS;
    uof_initMem_T   *initMem;
    uint64 dram0BaseAddr, dram1BaseAddr;
    uint64 SramBaseAddr;
    unsigned int ScratchOffset, SramOffset;
    unsigned int dram0Offset, dram1Offset;
    uclo_objHandle_T *objHandle;
    int ae, ret;

#ifdef __x86_64__
    uint64 tmp, tmp1, tmp2;
#else
    unsigned int tmp, tmp1, tmp2;
#endif /* __x86_64__ */ 

    if(!handle) 
    {
        ERRINFO(("bad argument, handle is NULL\n "));  
        return (UCLO_BADARG);
    } 
    objHandle = (uclo_objHandle_T *)handle->objHandle; 

    SramBaseAddr = handle->sysMemInfo.sramDesc.sramAeAddr;

    /* Verify that the offsets meet the alignment requirements
     * of the memory regions */
    if((halAe_GetDram0BaseAddr(handle, &dram0BaseAddr) != HALAE_SUCCESS)
       || (halAe_GetDram1BaseAddr(handle, &dram1BaseAddr) != HALAE_SUCCESS))
    {
        ERRINFO(("get Rram base addr error\n "));  
        return (UCLO_FAILURE);
    }
    halAe_GetMemoryStartOffset(handle, &ScratchOffset,
                               &SramOffset, &dram0Offset, &dram1Offset); 
    
    div_u64_rem(dram0BaseAddr + dram0Offset,
                objHandle->encapUofObj.varMemSeg->sdramAlignment, &tmp);
    div_u64_rem(dram1BaseAddr + dram1Offset,
                objHandle->encapUofObj.varMemSeg->sdram1Alignment, &tmp1);
    div_u64_rem(SramBaseAddr + SramOffset, 
                objHandle->encapUofObj.varMemSeg->sramAlignment, &tmp2);
    
    if((ScratchOffset % objHandle->encapUofObj.varMemSeg->scratchAlignment) != 0
       || tmp != 0
       || tmp1 != 0
       || tmp2 != 0)
    {
        ERRINFO(("base addr alignment error\n "));  
        return (UCLO_FAILURE);
    }

    /* remove the operation to zero memory blocks 
     * perform initializations. */

    initMem = objHandle->initMemTab.initMem;
    for(i = 0; i < objHandle->initMemTab.numEntries; i++)
    {
        if(initMem->numBytes) 
        {
            if((status = initMemory(handle, initMem))) 
            {
                return (status);
            }    
        }
        initMem = (uof_initMem_T *)((unsigned long)((unsigned long)initMem +
                                                    sizeof(uof_initMem_T)) +
                                    (sizeof(uof_memValAttr_T) *
                                     initMem->numValAttr));
    }
    
     /* run Batch put LM API */
    for (ae = 0; ae < MAX_AE; ae++) {
        ret = halAe_BatchPutLM(handle, (unsigned char)ae,
            objHandle->lmInitTab[ae]);
        if (ret != HALAE_SUCCESS) {
            status = UCLO_FAILURE;
            DBGINFO(("Error, AE %d failed to batch put Local memory\n", ae));
        } else {
            DBGINFO(("AE %d Successfully batch put Local memory\n", ae));
        }
        cleanupLmInitLink(handle, ae);
    }

    /* initUstore must be put after init local memory
     * because local memory initalization will overwrite ustore
     */
    if (UcLo_initUstore(handle))
    {
        DBGINFO(("initalize Ustore failed\n"));
        status = UCLO_FAILURE;
    }

       /* run Batch put LM API */
    for (ae = 0; ae < MAX_AE; ae++) {
        UcLo_BatchWriteUstoreMem(handle, (unsigned char)ae,
            objHandle->umemInitTab[ae]);
        cleanupUmemInitLink(handle, ae);
    }

    return (status);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Locates UOF chunk header matching the type specified by the
                input parameter 'chunkId' and excluding the current chunk.
 *
 * @param objHdr - IN
 * @param chunkId - IN
  * @param cur - IN
 *
 * @retval pointer to the chunk header or NULL if not found
 * 
 * 
 *****************************************************************************/
void *
UcLo_findChunk(uof_objHdr_T *objHdr, 
               char *chunkId, 
               void *cur)
{
    int i;
    uof_chunkHdr_T *chunkHdr =
            (uof_chunkHdr_T *)((unsigned long)objHdr + sizeof(uof_objHdr_T));

    for(i = 0; i < objHdr->numChunks; i++) 
    {
      if((cur < (void *)&chunkHdr[i])
         && !(strncmp(chunkHdr[i].chunkId, chunkId, UOF_OBJID_LEN))) 
      {
          return (&chunkHdr[i]);
      }    
    }  
    return (NULL);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Given a valid uword address, return the associated uword from
                the UOF. The function does not check if uAddr is an invalid. 
                NOTE: the "addr" parameter is the address relative to the start of the page.
 *
 * @param objHandle - IN
 * @param page - IN
 * @param addr - IN
 *
 * @retval uword value, or INVLD_UWORD
 * 
 * 
 *****************************************************************************/
uword_T 
Uclo_getUofUword_nocheck(uclo_objHandle_T *objHandle, 
                 uof_encapPage_T *page, 
                 unsigned int addr)
{
    uword_T uwrd=0;

    if(!page) 
    {
        return (INVLD_UWORD);
    }    

    /* find the block */
    if(page->numUwBlocks == 1) 
    {
        addr *= objHandle->uWordBytes;
        memmove(&uwrd,
                (void *)(((unsigned long)(page->uwBlocks[0].microWords)) +
                         addr),
                objHandle->uWordBytes);
        uwrd = uwrd & ICP_UWORD_MASK;
    }
    else
    {
        uwrd = Uclo_getUofUword(objHandle, page, addr);
    }

    return (uwrd);
}


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Given a valid uword address, return the associated uword from
                the UOF. The function will fail if uAddr is an invalid or unused
                address. NOTE: the "addr" parameter is the address relative to the
                start of the page.
 *
 * @param objHandle - IN
 * @param page - IN
  * @param addr - IN
 *
 * @retval uword value, or INVLD_UWORD
 * 
 * 
 *****************************************************************************/
uword_T 
Uclo_getUofUword(uclo_objHandle_T *objHandle, 
                 uof_encapPage_T *page, 
                 unsigned int addr)
{
    uword_T uwrd=0;
    unsigned int i;

    if(!page) 
    {
        return (INVLD_UWORD);
    }    

    /* find the block */
    for(i = 0; i < page->numUwBlocks; i++)
    {
        if((addr >= page->uwBlocks[i].startAddr)
           && (addr <= (page->uwBlocks[i].startAddr +
                        (page->uwBlocks[i].numWords-1))))
        {
            /* unpack n bytes and assigned to the 64-bit uword value.
            note: the microwords are stored as packed bytes.
            */
            addr -= page->uwBlocks[i].startAddr;
            addr *= objHandle->uWordBytes;
            memmove(&uwrd,
                    (void *)(((unsigned long)(page->uwBlocks[i].microWords)) +
                             addr),
                    objHandle->uWordBytes);
            uwrd = uwrd & ICP_UWORD_MASK;

            return (uwrd);
        }
    }

    return (INVLD_UWORD);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Given a valid uword address, set the associated uword in
                the UOF. The function will fail if uAddr is an invalid or unused
                address.
 *
 * @param objHandle - IN
 * @param page - IN
 * @param addr - IN
 * @param uwrd - IN
 *
 * @retval 0 = success, -1 = failure
 * 
 * 
 *****************************************************************************/
static int 
Uclo_setUofUword(uclo_objHandle_T *objHandle, 
                 uof_encapPage_T *page,
                 unsigned int addr, 
                 uword_T uwrd)
{
    unsigned int i;

    if(!page) 
    {
        return (-1);
    }    

    /* find the block */
    for(i = 0; i < page->numUwBlocks; i++)
    {
        if((addr >= page->uwBlocks[i].startAddr)
           && (addr <= (page->uwBlocks[i].startAddr +
                        (page->uwBlocks[i].numWords-1))))
        {
            /* assigned n bytes to microwords -- stored as packed bytes. */
            addr -= page->uwBlocks[i].startAddr;
            addr *= objHandle->uWordBytes;

            memmove((void *)(((unsigned long)(page->uwBlocks[i].microWords)) +
                             addr),
                    &uwrd, 
                    objHandle->uWordBytes);
            return (0);
        }
    }
    return (-1);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Maps the file header -- must be the first entry in the buffer.
 *
 * @param buf - IN
 * @param fileHdr - IN
 * @param chunkId - IN
 *
 * @retval Pointer to the chunk or NULL
 * 
 * 
 *****************************************************************************/
static uclo_objHdr_T *
UcLo_mapFileChunk(char *buf, 
                  uof_fileHdr_T *fileHdr, 
                  char *chunkId)
{
    int                    i;
    uof_fileChunkHdr_T  *fileChunk;
    void                *chunk;
    uclo_objHdr_T        *objHdr;

    fileChunk = (uof_fileChunkHdr_T *)(buf + sizeof(uof_fileHdr_T));
    /* find the chunk, verify it, return a pointer to it */
    for(i = 0; i < fileHdr->numChunks; i++)
    {
        if(!(strncmp(fileChunk->chunkId, chunkId, UOF_OBJID_LEN)))
        {

            /* verify chunk checksum */
            chunk = buf + fileChunk->offset;
            if(fileChunk->checksum
               != UcLo_strChecksum((char*)chunk, fileChunk->size)) 
            {
                break;
            }
            if(!(objHdr = osalMemAlloc(sizeof(uclo_objHdr_T)))) 
            {
                break;
            }
            objHdr->fBuf = chunk;
            objHdr->checksum = fileChunk->checksum;
            objHdr->size = fileChunk->size;
            return (objHdr);
        }
        fileChunk++;
    }
    return (NULL);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Map the uof image
 *
 * @param objHandle - IN
 * @param uniqueAeImage - IN
 * @param maxImages - IN
 *
 * @retval number of images, otherwise 0 if none or failure
 * 
 * 
 *****************************************************************************/
static int UcLo_mapImage(uclo_objHandle_T *objHandle,
                         uof_encapAe_T *uniqueAeImage,
                         int maxImages)
{
    int                        a = 0, p, i;
    uof_chunkHdr_T            *chunkHdr = NULL;
    uof_Image_T                *image;
    uof_codePage_T            *codePage;
    uof_aeRegTab_T            *aeRegTab;
    uof_initRegSymTab_T     *initRegSymTab;
    uof_sbreakTab_T            *sbreakTab;
    uof_encapUofObj_T        *encapUofObj = &objHandle->encapUofObj;

    for(i = 0; i < maxImages; i++)
    {
        uniqueAeImage[i].imagePtr = NULL;
        uniqueAeImage[i].pages = NULL;
        uniqueAeImage[i].aeReg = NULL;
        uniqueAeImage[i].numAeReg = 0;
        uniqueAeImage[i].initRegSym = NULL;
        uniqueAeImage[i].numInitRegSym = 0;
        uniqueAeImage[i].numSbreak = 0;
    }

    for(a = 0; a < maxImages; a++)
    {
        /* get the next IMAGE chunk header */
        if(!(chunkHdr = UcLo_findChunk(encapUofObj->objHdr, UOF_IMAG,
                                       (void *)chunkHdr))) 
        {
            break;
        }    

        image = (uof_Image_T *)(encapUofObj->begUof + chunkHdr->offset);

        aeRegTab = (uof_aeRegTab_T *)(image->regTabOffset +
                                      objHandle->objHdr->fBuf);
        uniqueAeImage[a].numAeReg = aeRegTab->numEntries;

        uniqueAeImage[a].aeReg = (uof_aeReg_T *)(((char *)aeRegTab) +
                                                 sizeof(uof_aeRegTab_T));

        initRegSymTab = (uof_initRegSymTab_T *)(image->initRegSymTab +
                                                objHandle->objHdr->fBuf);
        uniqueAeImage[a].numInitRegSym = initRegSymTab->numEntries;

        uniqueAeImage[a].initRegSym =
                (uof_initRegSym_T *)(((char *)initRegSymTab) +
                                     sizeof(uof_initRegSymTab_T));
        
        sbreakTab = (uof_sbreakTab_T *)(image->sbreakTab +
                                        objHandle->objHdr->fBuf);
        uniqueAeImage[a].numSbreak = sbreakTab->numEntries;

        uniqueAeImage[a].sbreak = (uof_sbreak_T *)(((char *)sbreakTab) +
                                                   sizeof(uof_sbreakTab_T));

        uniqueAeImage[a].imagePtr = image;
        codePage = (uof_codePage_T *)((char *)image + sizeof(uof_Image_T));

        if(!(uniqueAeImage[a].pages =
              (uof_encapPage_T *)osalMemAlloc(image->numOfPages * 
                                              sizeof(uof_encapPage_T))))
        {
            return (0);
        }
        osalMemSet(uniqueAeImage[a].pages, 0,
                   image->numOfPages * sizeof(uof_encapPage_T));

        /* assign the pages */
        for(p = 0; p < image->numOfPages; p++)
        {
            uof_ucVarTab_T      *ucVarTab;
            uof_impVarTab_T     *impVarTab;
            uof_impExprTab_T    *impExprTab;
            uof_neighRegTab_T   *neighRegTab;
            uof_codeArea_T      *codeArea;
            uof_uwordBlockTab_T *uwBlockTab;
            uof_uWordBlock_T    *uwBlocks;

            uniqueAeImage[a].pages[p].pageNum = codePage[p].pageNum;
            uniqueAeImage[a].pages[p].defPage = codePage[p].defPage;
            uniqueAeImage[a].pages[p].pageRegion = codePage[p].pageRegion;
            uniqueAeImage[a].pages[p].begVirtAddr = codePage[p].begVirtAddr;
            uniqueAeImage[a].pages[p].begPhyAddr = codePage[p].begPhyAddr;

            /* assign uC variables */
            ucVarTab = (uof_ucVarTab_T *)(encapUofObj->begUof +
                            codePage[p].ucVarTabOffset);
            uniqueAeImage[a].pages[p].numUcVar = ucVarTab->numEntries;

            uniqueAeImage[a].pages[p].ucVar =
                    (uof_ucVar_T *)((char *)ucVarTab + sizeof(uof_ucVarTab_T));

            /* assign import variables */
            impVarTab = (uof_impVarTab_T *)(encapUofObj->begUof +
                                            codePage[p].impVarTabOffset);
            uniqueAeImage[a].pages[p].numImpVar = impVarTab->numEntries;

            uniqueAeImage[a].pages[p].impVar =
                    (uof_importVar_T *)((char *)impVarTab +
                                        sizeof(uof_impVarTab_T));
            
            /* assign import expression variables */
            impExprTab = (uof_impExprTab_T *)(encapUofObj->begUof +
                                              codePage[p].impExprTabOffset);
            uniqueAeImage[a].pages[p].numImpExpr = impExprTab->numEntries;

            uniqueAeImage[a].pages[p].impExpr =
                    (uof_impExpr_T *)((char *)impExprTab +
                                      sizeof(uof_impExprTab_T));

            /* assign neighbour */
            neighRegTab = (uof_neighRegTab_T *)(encapUofObj->begUof +
                                                codePage[p].neighRegTabOffset);
            uniqueAeImage[a].pages[p].numNeighReg = neighRegTab->numEntries;

            uniqueAeImage[a].pages[p].neighReg =
                    (uof_neighReg_T *)((char *)neighRegTab +
                                       sizeof(uof_neighRegTab_T));
            
            /* assign code area */
            codeArea = (uof_codeArea_T *)(encapUofObj->begUof +
                                          codePage[p].codeAreaOffset);
            uniqueAeImage[a].pages[p].numMicroWords = codeArea->numMicroWords;
            uwBlockTab = (uof_uwordBlockTab_T *)(encapUofObj->begUof +
                                                 codeArea->uwordBlockTab);
            uniqueAeImage[a].pages[p].numUwBlocks = uwBlockTab->numEntries;
            uwBlocks = (uof_uWordBlock_T *)((char *)uwBlockTab +
                                            sizeof(uof_uwordBlockTab_T));

            /* reuses a memory block (uof_uWordBlock_T) in uof buffer 
             * for a uclo internal data struct (uof_encapuwblock_t) */
            uniqueAeImage[a].pages[p].uwBlocks =
                    (uof_encapuwblock_t *)uwBlocks;

            for(i=0; (unsigned int)i < uwBlockTab->numEntries; i++)
            {
                uniqueAeImage[a].pages[p].uwBlocks[i].microWords =
                        ARCH_ALIGN(encapUofObj->begUof +
                                        uwBlocks[i].uwordOffset);
            }
        }
    }

   return (a);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Maps Uengines in UOF to the handle
 *
 * @param objHandle - IN
 * @param maxAe - IN
 *
 * @retval UCLO_SUCCESS
 * 
 *****************************************************************************/
static int 
UcLo_mapAe(icp_firml_handle_t *handle,
           int maxAe)
{
    int imageNum, swAe;
    int status = UCLO_SUCCESS;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;    

    for(swAe = 0; swAe < maxAe; swAe++)
    {
        unsigned int hwAe = UcLo_getHwAe(objHandle->prodType,
                                         (unsigned char)swAe);
        UcLo_ClearAeData(&objHandle->aeData[swAe]);
        if(!isBitSet(handle->sysMemInfo.aeMask,
                    hwAe))
        {
            continue;
        }

        /* traverse the images and check if AE is assigned */
        for(imageNum = 0; imageNum < objHandle->numUniqueImage; imageNum++)
        {
            if(isBitSet(objHandle->uniqueAeImage[imageNum].imagePtr->aeAssigned,
                        hwAe))
            {
                status = UcLo_InitAeData(objHandle, swAe, imageNum);
                if(status)
                {
                    return (status);
                }
            }
        }
        if((status = UcLo_AssignHalPages(handle, &objHandle->aeData[swAe],
                                         objHandle->hwAeNum[swAe])))
        {
            return (status);
        }    
    }

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Maps the UOF string table
 *
 * @param objHdr - IN
 * @param tabName - IN
 * @param strTable - IN
 *
 * @retval pointer to the string table or NULL
 * 
 *****************************************************************************/
static uof_strTab_T *
UcLo_mapStrTable(uclo_objHdr_T *objHdr, 
                 char *tabName, 
                 uof_strTab_T *strTable)
{
    uof_chunkHdr_T *chunkHdr = NULL;
    int hdrSize;

    /* get the first string table -- should be only one */
    if((chunkHdr = UcLo_findChunk((uof_objHdr_T *)objHdr->fBuf, tabName, NULL)))
    {
        memmove(&strTable->tableLen, (objHdr->fBuf + chunkHdr->offset),
                sizeof(strTable->tableLen));
        hdrSize = (char *)&strTable->strings - (char *)strTable;
        strTable->strings = ARCH_ALIGN((char *)(objHdr->fBuf +
                                            chunkHdr->offset + hdrSize));
    }
    return (strTable);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Maps the UOF init memory table
 *
 * @param encapUofObj - IN
 * @param initMemTab - IN
 *
 * @retval pointer to the initMem table or NULL
 * 
 *****************************************************************************/
static uof_initMemTab_T *
UcLo_mapInitMemTable(uof_encapUofObj_T *encapUofObj, 
                     uof_initMemTab_T *initMemTab)
{
    uof_chunkHdr_T *chunkHdr = NULL;

    /* get ustore mem inits table -- should be only one */
    if((chunkHdr = UcLo_findChunk(encapUofObj->objHdr, UOF_IMEM, NULL)))
    {
        memmove(&initMemTab->numEntries,
                (encapUofObj->begUof + chunkHdr->offset),
                sizeof(unsigned int));
        if(initMemTab->numEntries) 
        {
            initMemTab->initMem =
                    (uof_initMem_T *)(encapUofObj->begUof +
                                      chunkHdr->offset + sizeof(unsigned int));
        }    
    }
    else
    {
        initMemTab->numEntries = 0;
        initMemTab->initMem = NULL;
    }
    return (initMemTab);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Maps the uC memory segments
 *
 * @param encapUofObj - IN
 *
 * @retval pointer to the uof_uCmemSeg_T or NULL
 * 
 *****************************************************************************/
static uof_varMemSeg_T *
UcLo_mapVarMemSeg(uof_encapUofObj_T *encapUofObj)
{
    uof_chunkHdr_T *chunkHdr = NULL;
    uof_varMemSeg_T *varMemSeg = NULL;
    unsigned int numEntries;

    /* get first memory segment header -- should only be one in the UOF */
    if((chunkHdr = UcLo_findChunk(encapUofObj->objHdr, UOF_MSEG, NULL)))
    {
        /* numEntries should be one
         * -- allows us to entend the number of uof_varMemSeg_T without
         * affecting the UOF format */
        memmove(&numEntries, (encapUofObj->begUof + chunkHdr->offset),
                sizeof(unsigned int));
        varMemSeg =
                (uof_varMemSeg_T *)(encapUofObj->begUof +
                                    chunkHdr->offset + sizeof(numEntries));
    }
    return (varMemSeg);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Assigns 'value' to a group of bits, (sub-field), as described
                by the 'field' input parameter.  Unspecified bits will remain
                unaffected, so, this function should work for 32bit-uwords if
                the field doesn't extend beyond bit31. 
 *
 * @param uword - IN
 * @param field - IN
 * @param value - IN
 *
 * @retval the updated word
 *****************************************************************************/
static uword_T 
UcLo_setValue(uword_T uword, 
              char *field, 
              unsigned int value)
{
    int i;

    /* validate the sub-fields */
    for(i=0; i < 0xC;)
    {
        unsigned int val;
        unsigned char msb, lsb, begBitPos, len, rShtValue;

        if((msb = field[i++]) >= 0x40)
        {
           break;
        }   
        lsb = field[i++];
        begBitPos = lsb;
        len = (msb - lsb + 1);

        rShtValue = field[i++];
        val = value >> rShtValue; 
        uword = UcLo_setField64(uword, begBitPos, len, val);
    }
    return (uword);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Creates an UOF object handle
 *
 * @retval pointer to handle, otherwise NULL if failed
 *
 *****************************************************************************/
uclo_objHandle_T *
UcLo_getObjHandle(void)
{
    uclo_objHandle_T *objHandle;

    if(!(objHandle =
             (uclo_objHandle_T*)osalMemAlloc(sizeof(uclo_objHandle_T)))) 
    {
        return (NULL);
    }    
    osalMemSet(objHandle, 0, sizeof(uclo_objHandle_T));

    objHandle->readOnly = 1;
    return (objHandle);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *   Creates an SUOF object handle
 *
 * @retval pointer to handle, otherwise NULL if failed
 *
 *****************************************************************************/
uclo_suof_handle_t *
UcLo_getSObjHandle(void)
{
    uclo_suof_handle_t *objHandle;

    if(!(objHandle =
             (uclo_suof_handle_t*)osalMemAlloc(sizeof(uclo_suof_handle_t)))) 
    {
        return (NULL);
    }    
    osalMemSet(objHandle, 0, sizeof(uclo_suof_handle_t));

    return (objHandle);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Locates an UOF image by name as specified by the 'ucodeImageName'
                parameter.
 *
 * @param objHandle - IN
 * @param ucodeImageName - IN
 *
 * @retval pointer to the encapsulated image or NULL
 *****************************************************************************/
uof_encapAe_T *
UcLo_getEncapImage(uclo_objHandle_T *objHandle, 
                   char *ucodeImageName)
{
    int i;
    char          *str;    

    if(!ucodeImageName || !objHandle || !objHandle->objHdr->fBuf)
    {
        return (NULL);
    }    

    /* find the image */
    for(i = 0; i < objHandle->numUniqueImage; i++) 
    {
        str = UcLo_getString(&objHandle->strTable,
                             objHandle->uniqueAeImage[i].imagePtr->imageName);
        if(!str) 
        { 
           continue;
        }   
        if(!UcLo_strcmp(ucodeImageName, str,
                        objHandle->uniqueAeImage[i].imagePtr->sensitivity)) 
        {
            return (&objHandle->uniqueAeImage[i]);
        }    
    }        

    return (NULL);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Return the image-name(s) and ctxAssignments of the specified AE.
                The return value to the image-name must not be modified.
 *
 * @param handle - IN
 * @param hwAe - IN 
 * @param slice - IN
 * @param imageAssign - IN
 *
 * @retval UCLO_SUCCESS, UCLO_BADARG, UCLO_BADAE, UCLO_NOOBJ,UCLO_IMGNOTFND
 *
 *****************************************************************************/
int 
UcLo_GetAeSliceImageName(icp_firml_handle_t *handle, 
                         unsigned char hwAe, 
                         unsigned int slice,
                         uclo_imageassign_t *imageAssign)
{
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    unsigned int swAe;

    if(!imageAssign) 
    {
        ERRINFO(("bad argument, imageAssign is NULL\n"));  
        return (UCLO_BADARG);
    }    
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad argument, objHandle=NULL \
                  or objHandle->objHdr->fBuf=NULL\n"));  
        return (UCLO_NOOBJ);
    }    
    if((swAe = UcLo_getSwAe(objHandle->prodType, hwAe)) == UCLO_BADAE) 
    {
        ERRINFO(("bad swAe=0x%x\n ",swAe));  
        return (UCLO_BADARG);
    }    

    if(slice > objHandle->aeData[swAe].numSlices) 
    {
        ERRINFO(("bad slices number=0x%x\n",
                slice > objHandle->aeData[swAe].numSlices));  
        return (UCLO_IMGNOTFND);
    }    
    if((!objHandle->aeData[swAe].aeSlice[slice].encapImage)
       || (!objHandle->aeData[swAe].aeSlice[slice].encapImage->imagePtr)) 
    {
        ERRINFO(("bad image in Ae=0x%x, Slice=0x%x\n",swAe,slice));  
        return (UCLO_IMGNOTFND);
    }    
    if(objHandle->aeData[swAe].aeSlice[slice].encapImage->imagePtr->imageName
       == 0xffffffff) 
    {
        ERRINFO(("bad image name = 0xffffffff\n "));  
        return (UCLO_IMGNOTFND);
    }    
    imageAssign->imageName = UcLo_getString(&objHandle->strTable,
        objHandle->aeData[swAe].aeSlice[slice].encapImage->imagePtr->imageName);
    imageAssign->assignedCtxMask =
            objHandle->aeData[swAe].aeSlice[slice].assignedCtxMask;
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Return the image-name by name of the specified AE. The return
 *      value to the image-name must not be modified micro address
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 *
 * @retval String pointer to the name of the image
 * 
 * 
 *****************************************************************************/
char *
UcLo_GetAeImageName(icp_firml_handle_t *handle, 
                    unsigned char hwAe)
{
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    unsigned int swAe;

    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        return (NULL);
    }    
    if((swAe = UcLo_getSwAe(objHandle->prodType, hwAe)) == UCLO_BADAE) 
    {
        return (NULL);
    }    

    if((!objHandle->aeData[swAe].aeSlice[0].encapImage)
       || (!objHandle->aeData[swAe].aeSlice[0].encapImage->imagePtr)) 
    {
        return (NULL);
    }    
    if(objHandle->aeData[swAe].aeSlice[0].encapImage->imagePtr->imageName
       == 0xffffffff) 
    {
        return (NULL);
    }    
    return (UcLo_getString(&objHandle->strTable, 
        objHandle->aeData[swAe].aeSlice[0].encapImage->imagePtr->imageName));
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Check if the UOF objects are compatible with the chip.
 *
 * @param objHandle - IN
 *
 * @retval UCLO_SUCCESS, or UCLO_UOFINCOMPAT
 *
 *****************************************************************************/
static int 
UcLo_checkCompat(uclo_objHandle_T *objHandle)
{
    unsigned int majVer, prodType=objHandle->prodType;
    
    /* check is chip is compatable with the UOF */
    if(!(prodType & objHandle->encapUofObj.objHdr->cpuType)) 
    {
        ERRINFO(("prodType=0x%x, cpuType=0x%x\n",
                    prodType,objHandle->encapUofObj.objHdr->cpuType ));  
        return (UCLO_UOFINCOMPAT);
    }    

    majVer = objHandle->prodRev & 0xff;
    /* check the min and max cpu version required against the chip's */
    if((objHandle->encapUofObj.objHdr->maxCpuVer < majVer) ||
       (objHandle->encapUofObj.objHdr->minCpuVer > majVer)) 
    {
        ERRINFO(("majVer=0x%x\n",majVer)); 
        return (UCLO_UOFINCOMPAT);
    }    

    return (UCLO_SUCCESS);
}

static int 
UcLo_checkAssignedAEs(icp_firml_handle_t *handle)
{

    unsigned int assignedAeMask = 0, ae = 0;
    assignedAeMask = UcLo_GetAssignedAEs(handle);
 
    if((assignedAeMask & ~(handle->sysMemInfo.aeMask)) != 0)
    {
        DBGINFO(("object file uses non-existant AEs\n")); 
    }
    for(ae=0; ae<MAX_AE; ae++)
    {
        if(!isBitSet(handle->sysMemInfo.aeMask, ae))
        {
            continue;
        }
        if((assignedAeMask & (1<<ae)))
        {
            if(halAe_VerifyAe(handle, (unsigned char)ae))
            {
                ERRINFO(("object file uses uninitialized AEs\n")); 
                return (UCLO_BADOBJ);
            }
        }
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Set the __CHIP_ID, __CHIP_REVISION constants to the appropriate
                values in the micro-instructions.
 *
 * @param objHandle - IN
 * @param page - IN 
 *
 * @retval UCLO_SUCCESS, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_fixupGlobalConst(uclo_objHandle_T *objHandle, 
                      uof_encapPage_T *page)
{
    unsigned int    iv;
    char            *varName;
    char            *chipRev = "__chip_revision";
    char            *chipId = "__chip_id";
    uof_importVar_T    *impVar;


    for(iv = 0; iv < page->numImpVar; iv++)
    {
        varName = UcLo_getString(&objHandle->strTable, page->impVar[iv].name);
        if(varName == NULL) 
        {
           continue;       
        }   
        if(!strncmp(varName, chipRev, strlen(chipRev)))
        {
            impVar = &page->impVar[iv];
            impVar->value = objHandle->prodRev;
            SET_FIXUP_ASSIGN(impVar->valueAttrs, 1);
        }
        else if(!strncmp(varName, chipId, strlen(chipId)))
        {
            impVar = &page->impVar[iv];
            impVar->value = objHandle->prodType;
            SET_FIXUP_ASSIGN(impVar->valueAttrs, 1);
        }
        else
        {
            continue;
        }    
    }

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *  Evaluate import_expression and fixup the micro-instruction.
 *
 * @param objHandle - IN
 * @param page - IN 
 * @param swAe - IN
 *
 * @retval UCLO_SUCCESS, UCLO_UNINITVAR, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_fixupExpr(icp_firml_handle_t *handle, 
               uof_encapPage_T *page,
               unsigned char swAe)
{
    uword_T            uword;
    unsigned int    iv;
    char            *fields;
    uof_impExpr_T    *impExpr;
    int                expRes = 0, retVal=UCLO_SUCCESS, status = UCLO_SUCCESS;
    char          *str;    
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    for(iv = 0; iv < page->numImpExpr; iv++)
    {
        impExpr = &page->impExpr[iv];

        if(impExpr->valType == EXPR_VAL)
        {
            str = UcLo_getString(&objHandle->strTable,impExpr->exprValue);
            if(!str) 
            {
                continue;
            }    
            if((status = UcloExp_evalPostfix(handle, swAe, str, &expRes)))
            {
                if(status != UCLO_UNINITVAR) 
                {  
                   return (status);
                }   
                retVal = UCLO_UNINITVAR;
            }
        }
        else
        {
            if(!GET_FIXUP_ASSIGN(impExpr->valueAttrs)) 
            {  
                retVal = UCLO_UNINITVAR;
            }    
        }

        fields = impExpr->fieldAttrs;

        /* bind the global constant */
        if((uword = Uclo_getUofUword(objHandle, page, impExpr->uwordAddress))
           == INVLD_UWORD) 
        {
            ERRINFO(("invalid uword 0x%"XFMT64"\n ", uword));  
            return (UCLO_FAILURE);
        }    

        uword = UcLo_setValue(uword, fields, expRes);
        Uclo_setUofUword(objHandle, page, impExpr->uwordAddress, uword);
        SET_FIXUP_ASSIGN(impExpr->valueAttrs, 1);
        
    }

    return (retVal);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 * Set the __UENGINE_ID, and __USTORE_DRAM_BASE constants to the appropriate
                values in the micro-instructions.
 *
 * @param objHandle - IN
 * @param page - IN 
 * @param swAe - IN
 *
 * @retval UCLO_SUCCESS, UCLO_UNINITVAR, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_fixupLocalConst(uclo_objHandle_T *objHandle, 
                     uof_encapPage_T *page,
                     unsigned char swAe)
{
    unsigned int    i;
    char            *varName;
    uof_importVar_T *impVar;
    int             status = UCLO_SUCCESS;

    if(swAe >= UOF_MAX_NUM_OF_AE)
    {
        return (UCLO_BADARG);
    }

    for(i = 0; i < page->numImpVar; i++)
    {
        impVar = &page->impVar[i];

        varName = UcLo_getString(&objHandle->strTable, impVar->name);
        if(varName == NULL) 
        {
           continue;
        }   
        
        if(!strncmp(varName, UENGINE_ID, strlen(UENGINE_ID)))
        {
            impVar->value = UcLo_getHwAe(objHandle->prodType, swAe);
            SET_FIXUP_ASSIGN(impVar->valueAttrs, 1);
            
        }
        else
        {
            if(!strncmp(varName, USTORE_DRAM_BASE_SYM,
                        strlen(USTORE_DRAM_BASE_SYM)))
            {
                if(objHandle->aeData[swAe].relocUstoreDram != -1)
                {   
                    impVar->value =
                       (unsigned int)objHandle->aeData[swAe].relocUstoreDram;
                    SET_FIXUP_ASSIGN(impVar->valueAttrs, 1);
               }
            }
        }
    }

    return (status);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 * Fixup the local variables for the specified uengine in the UOF
 *
 * @param objHandle - IN
 * @param page - IN 
 * @param swAe - IN
 *
 * @retval UCLO_SUCCESS, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_fixupLocalUcVar(uclo_objHandle_T *objHandle, 
                     uof_encapPage_T *page,
                     char swAe)
{
    uword_T uword;
    unsigned int ucv, uaddr;
    char *fields;
    char prefix[0xC];
    char          *str;    

    /* format the local variable prefix */
    LOCAL_NAME(prefix, "", UcLo_getHwAe(objHandle->prodType, swAe));
    for(ucv = 0; ucv < page->numUcVar; ucv++)
    {
        str = UcLo_getString(&objHandle->strTable, page->ucVar[ucv].name);
        if(!str) 
        {
           continue;
        }   
        if((GET_FIXUP_SCOPE(page->ucVar[ucv].valueAttrs) == LOCAL_SCOPE) &&
                !strncmp(str, prefix, strlen(prefix)))
        {
            fields = page->ucVar[ucv].fieldAttrs;
            uaddr = page->ucVar[ucv].uwordAddress;

            /* bind the local variable */
            if((uword = Uclo_getUofUword(objHandle, page, uaddr))
               == INVLD_UWORD) 
            {
                ERRINFO(("invalid uword 0x%"XFMT64"\n ", uword));  
                return (UCLO_FAILURE);
            }    

            uword = UcLo_setValue(uword, fields, page->ucVar[ucv].exprValue);
            Uclo_setUofUword(objHandle, page, uaddr, uword);
            
        }
    }

   return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 * Fixup the global variables in the UOF
 *
 * @param objHandle - IN
 * @param page - IN 
 *
 * @retval UCLO_SUCCESS, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_fixupGlobalUcVar(uclo_objHandle_T *objHandle, 
                      uof_encapPage_T *page)
{
    uword_T uword;
    unsigned int uaddr, ucv;
    char *fields;

    for(ucv = 0; ucv < page->numUcVar; ucv++)
    {

        if(GET_FIXUP_SCOPE(page->ucVar[ucv].valueAttrs) == GLOBAL_SCOPE)
        {
            fields = page->ucVar[ucv].fieldAttrs;
            uaddr = page->ucVar[ucv].uwordAddress;

            /* bind the global variable */
            if((uword = Uclo_getUofUword(objHandle, page, uaddr))
               == INVLD_UWORD) 
            {
                ERRINFO(("get invalid uword\n"));  
                return (UCLO_FAILURE);
            }    

            uword = UcLo_setValue(uword, fields, page->ucVar[ucv].exprValue);
            Uclo_setUofUword(objHandle, page, uaddr, uword);
            
        }
    }

   return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 * Locate an import variable by name
 *
 * @param objHandle - IN
 * @param swAe - IN 
 * @param varName - IN 
 *
 * @retval pointer to the import-variable, or NULL
 *
 *****************************************************************************/
uof_importVar_T *
UcLo_getImportVar(uclo_objHandle_T *objHandle,
                  unsigned char swAe,
                  char *varName)
{
    unsigned int i, p, s;
    char          *str;    

    for(s=0; s < objHandle->aeData[swAe].numSlices; s++)
    {
        if(!objHandle->aeData[swAe].aeSlice[s].encapImage) 
        {
            continue;
        }    
        for(p = 0;
            p < objHandle->aeData[swAe].aeSlice[s]. \
                             encapImage->imagePtr->numOfPages;
            p++)
        {
            for(i = 0;
                i < objHandle->aeData[swAe].aeSlice[s]. \
                                 encapImage->pages[p].numImpVar;
                i++)
            {
                str = UcLo_getString(&objHandle->strTable, 
                           objHandle->aeData[swAe].aeSlice[s]. \
                                     encapImage->pages[p].impVar[i].name);
                if(!str) 
                {
                    continue;
                }    
                if(!UcLo_strcmp(str, varName, 
        objHandle->aeData[swAe].aeSlice[s].encapImage->imagePtr->sensitivity))
                {
                    return (&objHandle->aeData[swAe].aeSlice[s]. \
                              encapImage->pages[p].impVar[i]);
                }    
            }
        }
    }
    return (NULL);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Locate a AE register by name and type
 *
 * @param objHandle - IN
 * @param encapAeImage - IN 
 * @param regName - OUT 
 * @param type - IN 
 *
 * @retval pointer to the register, or NULL
 *
 *****************************************************************************/
uof_aeReg_T *
UcLo_getAeRegName(uclo_objHandle_T *objHandle, 
                  uof_encapAe_T *encapAeImage,
                  char *regName, 
                  unsigned int type)
{
    unsigned int    i;
    char            *strPtr;
    uof_aeReg_T        *reg=NULL;
    char          *str;
    
    if((strPtr = strstr(regName, "[")))
    {
        *strPtr = '\0';
    }    

    for(i = 0; i < encapAeImage->numAeReg; i++)
    {
        str = UcLo_getString(&objHandle->strTable, 
                             encapAeImage->aeReg[i].visName);
        if(!str) 
        {
            continue;
        }    
        if(!UcLo_strcmp(str, regName, encapAeImage->imagePtr->sensitivity)
           && ((type == ICP_ANY_REG)
           || (encapAeImage->aeReg[i].type == type)))
        {
            reg = &encapAeImage->aeReg[i];
            break;
        }
    }

    if(strPtr && reg)
    {
        for(i = 0; i < encapAeImage->numAeReg; i++)
        {
            if((encapAeImage->aeReg[i].xoId == reg->xoId) &&
                    (encapAeImage->aeReg[i].refCount == reg->refCount) &&
                    (encapAeImage->aeReg[i].type == reg->type))
            {
                reg = &encapAeImage->aeReg[i];
                break;
            }
        }
        *strPtr = '[';
    }

    return (reg);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Locate a AE register by string-table-name-offset and type
 *
 * @param encapAeImage - IN 
 * @param regNameOffset - OUT 
 * @param type - IN 
 *
 * @retval pointer to the register, or NULL
 *
 *****************************************************************************/
static uof_aeReg_T *
UcLo_findAeReg(uof_encapAe_T *encapAeImage, 
               unsigned int regNameOffset,
               unsigned int type)
{
    unsigned int i;
    if(!encapAeImage)
    {
        return (NULL);
    }    
    for(i = 0; i < encapAeImage->numAeReg; i++)
    {
        if((regNameOffset == encapAeImage->aeReg[i].visName) &&
            ((type == ICP_ANY_REG) || (encapAeImage->aeReg[i].type == type))) 
        {
            return (&encapAeImage->aeReg[i]);
        }    
    }
    return (NULL);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         
 *
 * @param objHandle - IN 
 * @param hwAeNum - OUT 
 *
 * @retval 
 *
 *****************************************************************************/
int 
UcLo_getSharedUstoreNeigh(uclo_objHandle_T *objHandle, 
                          unsigned int hwAeNum)
{
    if(hwAeNum >= MAX_AE)
    {
        return (UCLO_BADAE);
    }
    if(hwAeNum & 0x1)
    {
        if(!((1<<(hwAeNum-1)) & BadHwAeMask))
        {
            return (hwAeNum-1);
        }
        else
        {
            return (UCLO_BADAE);
        }
    }
    return (UCLO_AE_NEIGHBOR(objHandle->swAe[hwAeNum], objHandle->numAEs));
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Fixup the neighbour-reg definitions for the specified uengine
                in the UOF
 *
 * @param objHandle - IN 
 * @param page - IN 
 * @param swAe - IN 
 *
 * @retval UCLO_SUCCESS, UCLO_NEIGHNOTFND, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_fixupLocalNeighReg(uclo_objHandle_T *objHandle, 
                        uof_encapPage_T *page,
                        unsigned char swAe)
{
    uword_T         uword;
    unsigned int    i, uaddr, aeNeighbor, s;
    char            *fields;
    uof_aeReg_T     *aeReg = 0;

    if(page->numNeighReg == 0) 
    {
       return (UCLO_SUCCESS);
    }   
    /* if UCLO_AE_NEIGHBOR does not exist, report error. */    
    aeNeighbor = UCLO_AE_NEIGHBOR(swAe, objHandle->numAEs);
    if((aeNeighbor == UCLO_BADAE)
      || (aeNeighbor >= UOF_MAX_NUM_OF_AE))
    {
       ERRINFO(("invalid aeNeighbor\n"));  
       return (UCLO_NEIGHNOTFND);
    }   

    for(i = 0; i < page->numNeighReg; i++)
    {
        fields = page->neighReg[i].fieldAttrs;
        uaddr = page->neighReg[i].uwordAddress;

        /* get the address of corresponding neigh-reg of the neighboring AE */
        for(s=0; s < objHandle->aeData[aeNeighbor].numSlices; s++)
        {
            if((aeReg =
            UcLo_findAeReg(objHandle->aeData[aeNeighbor].aeSlice[s].encapImage,
                           page->neighReg[i].name, ICP_NEIGH_REL))
               != NULL)
            {
                 break;
            }     
        }
        if(!aeReg)
        {
            ERRINFO(("error aeReg\n"));  
            return (UCLO_FAILURE);
        }

        /* bind the local variable */
        if((uword = Uclo_getUofUword(objHandle, page, uaddr)) == INVLD_UWORD) 
        {
            ERRINFO(("invalid uword 0x%"XFMT64"\n ", uword));  
            return (UCLO_FAILURE);
        }    

        uword = UcLo_setValue(uword, fields, aeReg->addr);
        Uclo_setUofUword(objHandle, page, uaddr, uword);
        SET_FIXUP_ASSIGN(page->neighReg[i].valueAttrs, 1);
        
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Fixup local variables and constants in the UOF for the specified
                accelEngine.
 *
 * @param objHandle - IN 
 * @param page - IN 
 * @param swAe - IN 
 *
 * @retval UCLO_SUCCESS, UCLO_UNINITVAR, or UCLO_FAILURE
 *
 *****************************************************************************/
int 
UcLo_fixupLocals(icp_firml_handle_t *handle, 
                 uof_encapPage_T *page,
                 unsigned char swAe)
{
    int status = UCLO_SUCCESS, retVal = UCLO_SUCCESS;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;  

    if(UcLo_fixupLocalUcVar(objHandle, page, swAe)) 
    {
        ERRINFO(("UcLo_fixupLocalUcVar error\n"));  
        return (UCLO_FAILURE);
    } 
    
    if((status =
           UcLo_fixupLocalNeighReg(objHandle, page, swAe))
       != UCLO_SUCCESS)
    {
        if(status != UCLO_UNINITVAR) 
        {
            ERRINFO((" status=0x%x\n", status));
            return (UCLO_FAILURE);
        }    
        else 
        {
            retVal = status;
        }    
    }
    if((status = UcLo_fixupLocalConst(objHandle, page, swAe)) != UCLO_SUCCESS)
    {
        if(status != UCLO_UNINITVAR) 
        {
            return (status);
        }    
    }

    if((status = UcLo_fixupExpr(handle, page, swAe)) != UCLO_SUCCESS)
    {
        if(status != UCLO_UNINITVAR) 
        {
            return (status);
        }    
        else
        {
            retVal = status;
        }    
    }
    return (retVal);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Fixup global variables and constants in the UOF for the specified
                accelEngine.
 *
 * @param objHandle - IN 
 * @param page - IN 
 *
 * @retval  UCLO_SUCCESS, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_fixupGlobals(uclo_objHandle_T *objHandle, 
                  uof_encapPage_T *page)
{
    if(page->numUcVar) 
    {
        if(UcLo_fixupGlobalUcVar(objHandle, page))
        {
           ERRINFO(("UcLo_fixupGlobalUcVar error\n "));  
           return (UCLO_FAILURE);
        }   
    }    
    if(UcLo_fixupGlobalConst(objHandle, page)) 
    {
        ERRINFO(("UcLo_fixupGlobalConst error\n "));  
        return (UCLO_FAILURE);
    }    
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *          Initialize global variables and constants in memory and the UOF.
 *
 * @param objHandle - IN 
 *
 * @retval   UCLO_IVDWARN, UCLO_SUCCESS, UCLO_BADARG, UCLO_FAILURE,
                UCLO_EXCDLMEMSIZE, UCLO_EXCDUMEMSIZE, UCLO_EXCDSRAMSIZE, 
                UCLO_EXCDDRAMSIZE, UCLO_EXCDSCRTHSIZE
 *
 *****************************************************************************/
static int 
UcLo_initGlobals(icp_firml_handle_t *handle)
{
    int retVal=UCLO_SUCCESS;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(!objHandle->globalInited)
    {
        int i, stat;
        unsigned int j, s;
        unsigned int hwAe;

        /* initialize the memory segments */
        if(objHandle->initMemTab.numEntries) 
        {
            if((stat = UcLo_initMemory(handle))) 
            {
                return (stat);
            }    
        } else {
           /* Need to init ustore here */
             if ((stat = UcLo_initUstore(handle))) {
                DBGINFO(("initalize Ustore failed\n"));
                return (stat);
            }
        }

        /* bind import variables with ivd values */
        if((stat = UcLo_bindIvd(handle)))
        {
            /* ignore these failures and allow load to continue */
            if((stat == UCLO_IMGNOTFND) || (stat == UCLO_SYMNOTFND))
            {
                retVal = UCLO_IVDWARN;
            }    
            else
            {
                return (stat);
            }    
        }

        /* bind the uC global variables
         * -- local variables will done on-the-fly */
        for(i = 0; i < objHandle->numUniqueImage; i++)
        {
            int p;

            for(p = 0;
                p < objHandle->uniqueAeImage[i].imagePtr->numOfPages;
                p++)
            {
                if(objHandle->uniqueAeImage[i].pages[p].numUwBlocks)
                {
                    if((stat = UcLo_fixupGlobals(objHandle,
                                  &objHandle->uniqueAeImage[i].pages[p]))
                       != UCLO_SUCCESS) 
                    {
                        return (stat);
                    }    
                }
            }
        }

        /* init reg and sym */
        for(j=0; (j < objHandle->numAEs) && (j < UOF_MAX_NUM_OF_AE); j++)
        {
            hwAe = UcLo_getHwAe(objHandle->prodType, (unsigned char)j);
            if(hwAe == UCLO_BADAE)
            {
                continue;
            }
            for(s=0; s < objHandle->aeData[j].numSlices; s++)
            {
                if(objHandle->aeData[j].aeSlice[s].encapImage)
                {
                    if((stat = 
                          UcLo_initRegSymExpr(handle, (unsigned char)hwAe,
                            objHandle->aeData[j].aeSlice[s].encapImage))) 
                    {
                        return (stat);
                    }    
                }
            }
        }

        objHandle->globalInited=1;
    }
    return (retVal);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *          Initialize AE register.
 *
 * @param hwAe - IN 
 * @param ctxMask - IN 
 * @param regType - IN 
 * @param regAddr - IN 
 * @param value - IN 
 *
 * @retval   UCLO_SUCCESS, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_initReg(icp_firml_handle_t *handle,
             unsigned char hwAe, 
             unsigned char ctxMask, 
             icp_RegType_T regType,
             unsigned short regAddr, 
             unsigned int value)
{
    unsigned char ctx;

    switch(regType){
    case ICP_GPA_REL:
    case ICP_GPB_REL:
    case ICP_SR_REL:
    case ICP_SR_RD_REL:
    case ICP_SR_WR_REL:
    case ICP_DR_REL:
    case ICP_DR_RD_REL:
    case ICP_DR_WR_REL:
    case ICP_NEIGH_REL:
        /* init for all valid ctx */
        for(ctx=0; ctx < MAX_CTX; ctx++)
        {
            if(!IS_BIT_SET(ctxMask, ctx)) 
            {
                continue;
            }    
            if(halAe_PutRelDataReg(handle, hwAe, ctx, regType, regAddr, value)) 
            {
               ERRINFO(("halAe_PutRelDataReg error\n"));  
               return (UCLO_FAILURE);
            }   
        }
        break;
    case ICP_GPA_ABS:
    case ICP_GPB_ABS:
    case ICP_SR_ABS:
    case ICP_SR_RD_ABS:
    case ICP_SR_WR_ABS:
    case ICP_DR_ABS:
    case ICP_DR_RD_ABS:
    case ICP_DR_WR_ABS:
        if(halAe_PutAbsDataReg(handle, hwAe, regType, regAddr, value)) 
        {
           ERRINFO(("halAe_PutAbsDataReg error\n"));  
           return (UCLO_FAILURE);
        }   
        break;
    default: 
        ERRINFO(("regType=0x%x\n",regType));  
        return (UCLO_FAILURE);
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Init symbol in memory.
 *
 * @param objHandle - IN 
 * @param hwAe - IN 
 * @param symName - IN 
 * @param offset - IN 
 * @param value - IN 
 *
 * @retval   UCLO_SUCCESS, or UCLO_BADOBJ
 *
 *****************************************************************************/

static int 
UcLo_initSym(icp_firml_handle_t *handle, 
             unsigned char hwAe, 
             char *symName,
             unsigned int offset, 
             unsigned int value)
{
    uof_initMem_T *initMem[0x2];
    const unsigned int numWords=1;
    char localName[MAX_VARNAME];
    int i;
    unsigned int ScratchOffset, SramOffset;
    unsigned int dram0Offset, dram1Offset;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    halAe_GetMemoryStartOffset(handle, &ScratchOffset,
                               &SramOffset, &dram0Offset, 
                               &dram1Offset);

    initMem[0] = UcLo_findMemSym(objHandle, symName);

    /* format the local variable name */    
    LOCAL_NAME(localName, symName, hwAe); 
    initMem[1] = UcLo_findMemSym(objHandle, localName);

    for(i=0; i < 0x2; i++)
    {
        if(initMem[i])
        {
            switch(initMem[i]->region)
            {
            case SRAM_REGION:
                UcLo_wrSram(handle, 
                            SramOffset + initMem[i]->addr+offset, 
                            &value, numWords);
                break;
            case DRAM_REGION:
                UcLo_wrDram(handle, 0,
                            dram0Offset + initMem[i]->addr+offset,
                            &value, numWords);
                break;
            case DRAM1_REGION:
                UcLo_wrDram(handle, 1, 
                            dram1Offset + initMem[i]->addr+offset, 
                            &value, numWords);
                break;
            case SCRATCH_REGION:
                UcLo_wrScratch(handle, 
                               ScratchOffset + initMem[i]->addr+offset,
                               &value, numWords);
                break;
            case LMEM_REGION:
                UcLo_wrLocalMem(handle, (unsigned char)hwAe,
                                initMem[i]->addr+offset, &value, numWords);
                break;
            case UMEM_REGION:
                UcLo_wrUstoreMem(handle, (unsigned char)hwAe, 
                                initMem[i]->addr+offset, &value, numWords);
                break;
            case SHRAM_REGION:
            case SHRAM1_REGION:
            case SHRAM2_REGION:
            case SHRAM3_REGION:
            case SHRAM4_REGION:
            case SHRAM5_REGION:
                if(!(handle->sysMemInfo.qatMask & 
                   (1 << (initMem[i]->region - SHRAM_REGION))))
                {
                    return (UCLO_BADOBJ);
                }
                /* not support init for shram now, add initialization code
                 * here if want to support it in future */
                break;
            default: 
                ERRINFO(("region type=0x%x\n", initMem[i]->region));  
                return (UCLO_BADOBJ);
            }
        }
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Fixup register, or sym with value from an expression.
 *
 * @param objHandle - IN 
 * @param hwAe - IN 
 * @param encapAe - IN 
 *
 * @retval   UCLO_SUCCESS, UCLO_INVLDCTX, or UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_initRegSymExpr(icp_firml_handle_t *handle, 
                    unsigned char hwAe,
                    uof_encapAe_T *encapAe)
{
    unsigned int        i, expRes, swAe;
    unsigned char        ctxMask;
    uof_initRegSym_T    *initRegSym;
    char          *str;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    
    if((swAe = UcLo_getSwAe(objHandle->prodType, hwAe)) == UCLO_BADAE) 
    {
        ERRINFO(("UcLo_getSwAe error, hwAe=0x%x\n", hwAe));  
        return (UCLO_FAILURE);
    }
    for(i=0; i < encapAe->numInitRegSym; i++)
    {
        initRegSym = &encapAe->initRegSym[i];
        if(initRegSym->valueType == EXPR_VAL)
        {
            str = UcLo_getString(&(objHandle->strTable), initRegSym->value);
            if(!str) 
            {
                continue;
            }    
            if(UcloExp_evalPostfix(handle, (unsigned char)swAe,
                                   str, (int *)(&expRes)))
            {                    
                ERRINFO((" UcloExp_evalPostfix error\n"));  
                return (UCLO_FAILURE);
            }
        }
        else
        {
            expRes = initRegSym->value;
        }

        switch(initRegSym->initType)
        {
        case INIT_EXPR: /* init sym */                
            str = UcLo_getString(&(objHandle->strTable), initRegSym->symName); 
            if(str)
            {
               UcLo_initSym(handle, hwAe, str, 
                            initRegSym->regAddrOrOffset, expRes);
            }   
            break;
        case INIT_REG: /* init register */
            if(CTX_MODE(encapAe->imagePtr->aeMode) == MAX_CTX) 
            {
                ctxMask = 0xff; /* eight ctx-mode */
            }    
            else
            {
                ctxMask = 0x55; /* four ctx-mode */
            }    

            UcLo_initReg(handle, hwAe, ctxMask,
                (icp_RegType_T)initRegSym->regType,
                (unsigned short)initRegSym->regAddrOrOffset, expRes);
            break;
        case INIT_REG_CTX: /* init ctx relative register */
            if(CTX_MODE(encapAe->imagePtr->aeMode) == MAX_CTX) 
            {
                ctxMask = 0xff; /* eight ctx-mode */
            }    
            else
            {
                ctxMask = 0x55; /* four ctx-mode */
            }    

            /* check if ctx is appropriate for the ctxMode */
            if(!((1 << initRegSym->ctx) & ctxMask)) 
            {
                ERRINFO(("ctx num =0x%x\n", initRegSym->ctx));
                return (UCLO_INVLDCTX);
            }    

            UcLo_initReg(handle, hwAe, (unsigned char)(1 << initRegSym->ctx),
                (icp_RegType_T)initRegSym->regType,
                (unsigned short)initRegSym->regAddrOrOffset, expRes);
            break;
        case INIT_EXPR_ENDIAN_SWAP: /* init sym with endian_swap*/
            expRes = ENDIAN_SWAP32(expRes);
            str = UcLo_getString(&(objHandle->strTable), initRegSym->symName);
            if(str) 
            {
                UcLo_initSym(handle, hwAe, str,
                             initRegSym->regAddrOrOffset, expRes);
            }    
            break;
        default: break;
        }
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Get AE number of contexts mode.
 *
 * @param objHandle - IN 
 * @param ae - IN 
 *
 * @retval  aeMode
 *****************************************************************************/
unsigned int 
UcLo_getAeMode(uclo_objHandle_T *objHandle, 
               int ae)
{
    uof_Image_T *image = objHandle->aeData[ae].aeSlice[0].encapImage->imagePtr;
    return image->aeMode;
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Set AE number of contexts mode.
 *
 * @param objHandle - IN 
 *
 * @retval UCLO_SUCCESS, UCLO_FAILURE, or UCLO_BADOBJ
 *
 *****************************************************************************/
static int  
UcLo_setAeMode(icp_firml_handle_t *handle)
{
    unsigned char i, hwAeNum, nnMode, s;
    uof_Image_T      *uofImage;
    uclo_aedata_t    *aeData;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    for(i = 0; (i < objHandle->numAEs) && (i < UOF_MAX_NUM_OF_AE); i++)
    {
        aeData = &(objHandle->aeData[i]);
        hwAeNum = objHandle->hwAeNum[i];
        if(!isBitSet(handle->sysMemInfo.aeMask, hwAeNum))
        {
            continue;
        }
        for(s=0; (s < aeData->numSlices) && (s < MAX_CONTEXTS); s++)
        {
            if(objHandle->aeData[i].aeSlice[s].encapImage)
            {
                uofImage = aeData->aeSlice[s].encapImage->imagePtr;
                if(halAe_PutAeCtxMode(handle, hwAeNum,
                                      (char)CTX_MODE(uofImage->aeMode))) 
                {
                    ERRINFO(("halAe_PutAeCtxMode error\n"));  
                    return (UCLO_FAILURE);
                }
                /* ignore if UOF_NN_MODE_DONTCARE -- 
                 * this module don't care about the NNmode; the mode will be set
                 * by another module or the default value will be used. */
                if((nnMode = NN_MODE(uofImage->aeMode))
                   != UOF_NN_MODE_DONTCARE) 
                {
                    if(halAe_PutAeNnMode(handle, hwAeNum, nnMode)) 
                    {
                        ERRINFO(("halAe_PutAeNnMode error\n"));  
                        return (UCLO_FAILURE);
                    }
                }        

                if(halAe_PutAeLmMode(handle, hwAeNum, ICP_LMEM0,
                                     (char)LOC_MEM0_MODE(uofImage->aeMode))) 
                {
                    ERRINFO(("halAe_PutAeLmMode error\n "));  
                    return (UCLO_FAILURE);
                }
                if(halAe_PutAeLmMode(handle, hwAeNum, ICP_LMEM1, 
                                     (char)LOC_MEM1_MODE(uofImage->aeMode))) 
                {
                    ERRINFO(("halAe_PutAeLmMode error\n "));  
                    return (UCLO_FAILURE);
                }
                /* set the share control store mode on/off */
                if(halAe_PutAeSharedCsMode(handle, hwAeNum, 
                        (unsigned char)SHARED_USTORE_MODE(uofImage->aeMode))) 
                {
                    ERRINFO(("halAe_PutAeSharedCsMode error\n "));  
                    return (UCLO_FAILURE);
                }
                /* set relodable mode and assign ustore-dram if neccessary */
                if(halAe_SetReloadUstore(handle, hwAeNum, 
                                  uofImage->reloadableSize, 
                                  RELOADABLE_CTX_SHARED_MODE(uofImage->aeMode),
                                  aeData->relocUstoreDram)) 
                {
                   ERRINFO(("halAe_SetReloadUstore error\n "));  
                   return (UCLO_FAILURE);
                }   
 
            }
        }
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 *
 * @description
 *           set the numUwordsUsed in the image data structure
 *
 * @param objHandle - IN
 *
 * @retval UCLO_SUCCESS, UCLO_FAILURE
 *
 *****************************************************************************/
static int
UcLo_initNumUwordUsed(icp_firml_handle_t *handle)
{
    int a, p;
    uof_encapAe_T *image;
    uof_encapPage_T *page;
    uof_Image_T *uofImage;
    unsigned int numUwordsUsed, tmp;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    for(a = 0; a < objHandle->numUniqueImage; a++)
    {
        image = &(objHandle->uniqueAeImage[a]);
        uofImage = image->imagePtr;

        numUwordsUsed = 0;

        for (p=0; p<uofImage->numOfPages; p++)
        {
            page = &image->pages[p];

            /* update numUwordsUsed */
            tmp = page->begPhyAddr+page->numMicroWords;
            if (tmp > numUwordsUsed)
            {
                numUwordsUsed = tmp;
            }
        } /* end for p */

        image->numUwordsUsed = numUwordsUsed;
    } /* end for a */
 
    return (UCLO_SUCCESS);

}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Fill ustore with specified pattern
 *
 * @param objHandle - IN 
 *
 * @retval UCLO_SUCCESS, UCLO_FAILURE
 *
 *****************************************************************************/
static int 
UcLo_initUstore(icp_firml_handle_t *handle)
{

    /* For simplicity, we initialize all of ustore. Later, if we want to
       get fancy, we could limit our initialization to not include words
       in pages being loaded by default */
    int a, p;
    unsigned int i;
    uof_encapAe_T *image;
    uof_encapPage_T *page;
    uof_Image_T *uofImage;
    unsigned int doNotInit;
    unsigned int start, end;
    unsigned char swAe;
    unsigned int hwAeNum;
    unsigned int ustoreSize;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    uword_T *fill_data = NULL;
    fill_data =
      (uword_T *)osalMemAlloc(ICP_MAX_USTORE*(sizeof(uword_T)/sizeof(char)));
    if(fill_data == NULL) 
    {
        ERRINFO(("fill_data == NULL\n"));  
        return (UCLO_FAILURE);
    }        
    
    for(a = 0; a < objHandle->numUniqueImage; a++)
    {
        image = &(objHandle->uniqueAeImage[a]);
        uofImage = image->imagePtr;

        for (i=0; i<ICP_MAX_USTORE; i++) 
        {
            memmove(&fill_data[i], &uofImage->fillPattern, sizeof(uword_T));
        }

        /* Compute doNotInit value as a value that will not be equal to
           fill_data when cast to an int */
        doNotInit = 0;
        if (doNotInit == (unsigned int) fill_data[0]) 
        { 
            doNotInit = 0xFFFFFFFF;
        } 

        for (p=0; p<uofImage->numOfPages; p++) 
        {
            page = &image->pages[p];

            /* If this is not a default page, do not mark it as doNotInit */
            if (!page->defPage) 
            {
                continue;
            }    

            for (i=page->begPhyAddr;
                 i<page->begPhyAddr+page->numMicroWords;
                 i++)
            {
                fill_data[i] = (uword_T)doNotInit;
            } /* end for i */
        } /* end for p */


        for(swAe = 0;
            (swAe < objHandle->numAEs) && (swAe < UOF_MAX_NUM_OF_AE);
            swAe++)
        {
            hwAeNum = UcLo_getHwAe(objHandle->prodType, swAe);
            if(!IS_BIT_SET(uofImage->aeAssigned, hwAeNum)) 
            {
                continue;
            }  
            /* if odd numbered AE, then assume that 
             * the even numbered AE already fill the ustore */
            if(objHandle->aeData[swAe].shareableUstore && (hwAeNum & 1))
            {
                unsigned int hwNeighNum;
                hwNeighNum = UcLo_getSharedUstoreNeigh(objHandle, hwAeNum);
                if (hwNeighNum == UCLO_BADAE)
                {
                    osalMemFree(fill_data);
                    ERRINFO(("hwNeighNum is bad\n"));  
                    return (UCLO_NEIGHNOTFND);
                }
                if(IS_BIT_SET(uofImage->aeAssigned, hwNeighNum)) 
                {
                    continue;
                }    
            }

            ustoreSize = objHandle->aeData[swAe].effectUstoreSize;

            /* initialize the areas not going to be overwritten */
            end = (unsigned int)-1;
            do 
            {
                /* find next uword that needs to be initialized */
                for (start = end+1; start < ustoreSize; start++) 
                {
                    if (((unsigned int)fill_data[start]) != doNotInit)
                    {
                        break;
                    }    
                }
                /* see if there are no more such uwords */
                if (start >= ustoreSize) 
                {
                    break;
                }    
                for (end = start+1; end < ustoreSize; end++) 
                {
                    if (((unsigned int)fill_data[end]) == doNotInit) 
                    {
                        break;
                    }    
                }
                /* we need to fill start - (end-1) */
                if(objHandle->aeData[swAe].shareableUstore)
                {
                    if(halAe_PutCoalesceUwords(handle,
                                               (unsigned char)hwAeNum, 
                                               start, end-start, 
                                               &fill_data[start])
                       != HALAE_SUCCESS) 
                    {
                        osalMemFree(fill_data);
                        ERRINFO(("halAe_PutCoalesceUwords error\n"));  
                        return (UCLO_FAILURE);
                    }
                }
                else
                { 
                    if(halAe_PutUwords(handle, (unsigned char)hwAeNum,
                                       start, end-start, &fill_data[start])
                       != HALAE_SUCCESS)  
                    {
                        osalMemFree(fill_data);
                        ERRINFO(("halAe_PutUwords error\n"));  
                        return (UCLO_FAILURE);
                    }
                }
            } while (end < ustoreSize);
        } /* end for swAe */

    } /* end for a */

    osalMemFree(fill_data);

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Fill ustore with specified pattern
 *
 * @param objHandle - IN 
 *
 * @retval UCLO_SUCCESS
 *
 *****************************************************************************/
static int 
UcLo_initUstoreDramBase(uclo_objHandle_T *objHandle)
{
    uof_initMem_T *initMem;
    char          symName[MAX_VARNAME];
    unsigned int  swAe;

    for (swAe=0; swAe<objHandle->numAEs; swAe++) 
    {
        if((1<<swAe) & BadHwAeMask)
        {
            continue;
        }
        LOCAL_NAME(symName, USTORE_DRAM_BASE_SYM, objHandle->hwAeNum[swAe]);
        if((initMem = UcLo_findMemSym(objHandle, symName))) 
        { 
            objHandle->aeData[swAe].relocUstoreDram = initMem->addr;
        }    
        else 
        {
            objHandle->aeData[swAe].relocUstoreDram = -1;
        }    
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Overlays the UOF objects
 *
 * @param objHandle - IN 
 * @param readOnly - IN 
 *
 * @retval UCLO_SUCCESS, UCLO_UOFINCOMPAT, UCLO_INVLDCTX, UCLO_BADOBJ, UCLO_FAILURE
 *
 *****************************************************************************/
int 
UcLo_overlayObj(icp_firml_handle_t *handle, 
                int readOnly)
{
    unsigned int swAe, hwAeNum;
    unsigned int maxEnabledAeNum = 0, i;

    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    if(!objHandle)
    {
        ERRINFO(("Bad uclo object handle\n "));  
        return (UCLO_BADOBJ);
    }    

    objHandle->uwordsCpyBuf =
     (uword_T *)osalMemAlloc(UWORD_CPYBUF_SIZE*(sizeof(uword_T)/sizeof(char)));
    if(objHandle->uwordsCpyBuf == NULL) {
        ERRINFO(("malloc objHandle->uwordsCpyBuf error\n"));  
        return (UCLO_FAILURE);
    }
 
    /* set the UOF object offset */
    objHandle->encapUofObj.begUof = objHandle->objHdr->fBuf;
    objHandle->encapUofObj.objHdr = (uof_objHdr_T *)objHandle->objHdr->fBuf;
    objHandle->encapUofObj.chunkHdr =
            (uof_chunkHdr_T *)(objHandle->objHdr->fBuf + sizeof(uof_objHdr_T));
    objHandle->readOnly = readOnly;


    objHandle->uWordBytes = AEV2_PACKED_UWORD_BYTES;
    objHandle->numCtxPerAe = MAX_AE2_CTX;

    /* get the cpu type and rev */
    halAe_GetProdInfo(handle, &(objHandle->prodType), &(objHandle->prodRev));
    switch(objHandle->prodType)
    {
    case (EP80579_CPU_TYPE):
        objHandle->numAEs = 0x4;
        objHandle->ustorePhySize = USTORE_8K;
        LmemByteSize = (MAX_ICP_LMEM_REG << 0x2);
        BadHwAeMask = BAD_EP80579_HWAE_MASK;  
        break;
    case ACCEL_COMP_TYPE:
    case ACCELCOMP_C_CPU_TYPE:
    case ACCELCOMP_B_CPU_TYPE:
    case ACCELCOMP_RS_CPU_TYPE:    
    case ACCELCOMP_R_CPU_TYPE:
        objHandle->numAEs = 0;
        /*Since the mask are consecutive now from bit 0 to 15, 
          just check these 16 bits. If mask design changes to 
          discrete, here need to be changed correspondingly*/
        for(i = 0; i < UOF_MAX_NUM_OF_AE; i++)
        {
            if((1<<i) & handle->sysMemInfo.aeMask)
            {
                maxEnabledAeNum = i;
                objHandle->numAEs = maxEnabledAeNum + 1;
            }
        }
        objHandle->ustorePhySize = USTORE_16K;
        LmemByteSize = (MAX_ICP_LMEM_REG << 0x2);  
        BadHwAeMask = ~(handle->sysMemInfo.aeMask);
        break;        
    default:
        ERRINFO(("ProdType = 0x%x\n", objHandle->prodType));  
        return (UCLO_BADOBJ);
    }

    /* Set up swAe <-> hwAeNum mapping table */
    for (swAe=0;
            ((swAe < objHandle->numAEs)&&(swAe < UOF_MAX_NUM_OF_AE));
            swAe++)
    {
        /* if swAe=nonexistentAe, hwAeNum=UCLO_BADAE */
        hwAeNum = UcLo_getHwAe(objHandle->prodType, (unsigned char)swAe);
        objHandle->hwAeNum[swAe] = (unsigned char)hwAeNum;
        if(hwAeNum != UCLO_BADAE)
        {
            objHandle->swAe[hwAeNum] = (unsigned char)swAe;
        }
    }

    /* map the dbg string-table */
    UcLo_mapStrTable(objHandle->objHdr, UOF_STRT, &(objHandle->strTable));
    if(objHandle->dbgObjHdr) 
    {
        UcLo_mapStrTable(objHandle->dbgObjHdr,
                         DBG_STRT, &(objHandle->dbgObjStrTable));
    }    

    if (UcLo_isProfileSupported())
    {
        /* map the prf string-table */
        if(objHandle->prfObjHdr) 
        {
            UcLo_mapStrTable(objHandle->prfObjHdr, 
                             PRF_STRT, &(objHandle->prfObjStrTable));
        }    
    }

    /* map the images */
    if((objHandle->numUniqueImage = UcLo_mapImage(objHandle,
            objHandle->uniqueAeImage, UOF_MAX_NUM_OF_AE * MAX_CONTEXTS)))
    {
        if(UcLo_checkCompat(objHandle)) 
        {
           ERRINFO(("uof incompatable\n "));  
           return (UCLO_UOFINCOMPAT);
        }   
        if(UcLo_checkAssignedAEs(handle)) 
        {
           ERRINFO(("Bad object\n "));  
           return (UCLO_BADOBJ);
        }   
        if(UcLo_mapAe(handle, objHandle->numAEs)) 
        {
           ERRINFO(("Bad object\n "));  
           return (UCLO_BADOBJ);
        }   
    }

    /* initalize numUword */
    UcLo_initNumUwordUsed(handle);
    
    /* map memory initialization table and memory segments table */
    UcLo_mapInitMemTable(&(objHandle->encapUofObj), &(objHandle->initMemTab));
    objHandle->encapUofObj.varMemSeg = 
            UcLo_mapVarMemSeg(&objHandle->encapUofObj);

    /* init the reloadable-ustore dram base */
    UcLo_initUstoreDramBase(objHandle);

    /* set the AE number of ctx mode -- ctxMode */
    if(UcLo_setAeMode(handle)) 
    {
       ERRINFO(("UcLo_setAeMode error\n "));  
       return (UCLO_FAILURE);
    }   

    objHandle->pausingAeMask = 0;
    osalMutexInit(&objHandle->overlayMutex);
   
    halAe_DefineUcloCallback(UcLo_ProcessHalCallbacks, (void*) handle);

    return (UCLO_SUCCESS);
}


/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Writes a word to the specified uEngine
 *
 * @param handle - IN 
 * @param aeMask - IN 
 * @param phyUaddr - IN 
 * @param numWords - IN 
 * @param uWord - IN 
 *
 * @retval  UCLO_SUCCESS, UCLO_BADARG, UCLO_FAILURE
 *
 *****************************************************************************/
int 
UcLo_WriteNumUword(icp_firml_handle_t *handle, 
                   unsigned int aeMask, 
                   unsigned int phyUaddr,
                   unsigned int numWords, 
                   uword_T *uWord)
{
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    unsigned char swAe;
    unsigned int hwAeNum;
    unsigned int copyCnt=numWords, i=0;
    if((objHandle == NULL) || (uWord == NULL)
      || (phyUaddr >= objHandle->ustorePhySize)
      || ((phyUaddr + numWords) > objHandle->ustorePhySize)) 
    {
        ERRINFO(("phyUaddr=0x%x,numWords=0x%x\n",phyUaddr, numWords));  
        return (UCLO_BADARG);
    }    

    while(numWords)
    {
        if(numWords < UWORD_CPYBUF_SIZE) 
        {
           copyCnt = numWords;
        }   
        else
        {
           copyCnt = UWORD_CPYBUF_SIZE;
        }   

        for(swAe = 0; swAe < objHandle->numAEs; swAe++)
        {
            if(halAe_VerifyAe(handle, swAe) != HALAE_SUCCESS)
            {
                continue;
            }
            hwAeNum = UcLo_getHwAe(objHandle->prodType, swAe);
            if(IS_BIT_SET(aeMask, hwAeNum))
            {
                if(halAe_PutUwords(handle,
                                   (unsigned char)hwAeNum,
                                   phyUaddr+i, copyCnt, &uWord[i]) 
                   != HALAE_SUCCESS) 
                {
                    ERRINFO(("halAe_PutUwords error\n "));  
                    return (UCLO_FAILURE);
                }    
            }
        }
        i += copyCnt;
        numWords -= copyCnt;
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Writes a 32-bit unsigned value to the specified AccelEngine(s) and 
 *      micro address
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param aeMask - IN An integer mask specifying the AccelEngines to which to
 *                    write the value uWord at the address uAddr
 * @param phyUaddr - IN An integer value indicating the microstore physical  
 *                      address to write the value to
 * @param uWord - IN The value to be written to one or more AccelEngines 
 *                   initialize
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/
int 
UcLo_WriteUword(icp_firml_handle_t *handle, 
                unsigned int aeMask, 
                unsigned int phyUaddr, 
                uword_T uWord)
{
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    unsigned char swAe;
    unsigned int hwAeNum;
    if((objHandle == NULL) || (phyUaddr >= objHandle->ustorePhySize)) 
    {
        ERRINFO(("phyUaddr=0x%x\n", phyUaddr));  
        return (UCLO_BADARG);
    }    

    for(swAe = 0; swAe < objHandle->numAEs; swAe++)
    {
        if(halAe_VerifyAe(handle, swAe) != HALAE_SUCCESS)
        {
            continue;
        }
        hwAeNum = UcLo_getHwAe(objHandle->prodType, swAe);
        if(IS_BIT_SET(aeMask, hwAeNum))
        {
            if(halAe_PutUwords(handle,
                               (unsigned char)hwAeNum,
                               phyUaddr, 1, &uWord)
               != HALAE_SUCCESS) 
            {
                ERRINFO(("halAe_PutUwords error\n"));  
                return (UCLO_FAILURE);
            }    
        }
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Reads a word from the specified uEngine
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * @param phyUaddr - IN An integer value indicating the microstore physical  
 *                      address to write the value to
 * @param uWord - OUT The value read from the specified AccelEngines at address 
 *                    uAddr 
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/
int 
UcLo_ReadUword(icp_firml_handle_t *handle, 
               unsigned char hwAe, 
               unsigned int phyUaddr, 
               uword_T *uWord)
{
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if((uWord == NULL) || (objHandle == NULL)) 
    {
        ERRINFO(("uWord = %p,objHandle=0x%p\n",
                (unsigned int *)uWord, (unsigned int *)objHandle));  
        return (UCLO_BADARG);
    }    
    if(((unsigned int)hwAe >= MAX_AE)
       || ((1 << (unsigned int)hwAe) & BadHwAeMask)) 
    {
        return (UCLO_BADAE);
    }    
    if(phyUaddr >= objHandle->ustorePhySize) 
    {
       ERRINFO((" phyUaddr =0x%x\n", phyUaddr));  
       return (UCLO_BADARG);
    }

    *uWord = 0;
    if(halAe_GetUwords(handle, hwAe, phyUaddr, 1, uWord) != HALAE_SUCCESS) 
    {
       ERRINFO((" halAe_GetUwords error\n"));  
       return (UCLO_FAILURE);
    }   
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Checks to see if the specified page is currently loaded
 *
 * @param objHandle - IN 
 * @param sliceNum - IN 
 * @param pageNum - IN 
 * @param swAe - IN 
 *
 * @retval  0 if not loaded,
 *
 *****************************************************************************/
static int 
isPageLoaded(uclo_objHandle_T *objHandle, 
             unsigned int sliceNum,
             int pageNum, 
             unsigned char swAe)
{
    uclo_aedata_t *aeData;
    uclo_page_t   *page;
    uclo_region_t *region;

    aeData = &objHandle->aeData[swAe];
    page = &aeData->aeSlice[sliceNum].pages[pageNum];
    region = page->region;
    return (region->loaded == page);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *         Compares the encapsulated images to images that are loaded in
                the Uengines.
 *
 * @param objHandle - IN 
 * @param encapImage - IN 
 * @param swAe - IN 
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, or UCLO_BADARG, 
 *
 *****************************************************************************/
static int 
UcLo_verifyEncapUimage(icp_firml_handle_t *handle, 
                       uof_encapAe_T *encapImage,
                       unsigned char swAe)
{
    uof_encapPage_T      *pages;
    uof_Image_T          *image;
    int                  p;
    unsigned int        hwAeNum;
    unsigned int         p_offset, s;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if((!encapImage) || (swAe >= UOF_MAX_NUM_OF_AE))
    {
        ERRINFO(("encapImage is NULL, or incorrect swAe\n"));  
        return (UCLO_BADARG);
    }    

    image = encapImage->imagePtr;
    pages = encapImage->pages;
    
    /* find the slice to which the image is assigned */
    for(s=0; s < objHandle->aeData[swAe].numSlices; s++)
    {
        if(objHandle->aeData[swAe].aeSlice[s].assignedCtxMask
           & image->ctxAssigned) 
        {
           break;
        }   
    }
    if(s >= objHandle->aeData[swAe].numSlices) 
    {
        ERRINFO(("slice number is too large: slice=0x%x\n",s));  
        return (UCLO_FAILURE);
    }    

    hwAeNum = UcLo_getHwAe(objHandle->prodType, swAe);
    if(hwAeNum == UCLO_BADAE) 
    {
        ERRINFO(("bad hwAeNum\n"));  
        return (UCLO_BADARG);
    }    

    /* traverse the pages */
    for(p = 0; p < image->numOfPages; p++)
    {
        unsigned int addr;
        uword_T fEngUword = 0, uword = 0;

        if(s >= MAX_CONTEXTS) 
        { 
           ERRINFO(("s=0x%x\n", s));  
           return (UCLO_FAILURE);
        }    
        if (!isPageLoaded(objHandle, s, p, swAe)) 
        {
            continue;
        }
        if(UcLo_fixupLocals(handle, &pages[p], swAe)) 
        {
           ERRINFO((" UcLo_fixupLocals error\n"));  
           return (UCLO_FAILURE);
        }   

        p_offset = pages[p].begPhyAddr;

        for(addr = 0; addr < (unsigned int)pages[p].numMicroWords; addr++)
        {
             if(objHandle->aeData[swAe].shareableUstore) 
             {
                /* For shareable ustore, should use halAe_GetCoalesceUwords
                 * to get uwords from ustore, instead of halAe_GetUwords. */
                if(halAe_GetCoalesceUwords(handle, (unsigned char)hwAeNum,
                                           addr + p_offset, 1, &fEngUword)
                   != HALAE_SUCCESS) 
                {
                    ERRINFO(("halAe_GetCoalesceUwords error\n "));  
                    return (UCLO_FAILURE);
                }    
                /* For shared ctrl store, in uof file, the page 
                 * relative address in the next AE will be added by 
                 * the physical offset of the page in the previous AE.
                 * So here, we need to pass the address to Uclo_getUofUword
                 * after adding in the physical offset of the page 
                 * in the previous AE. */
                if((uword =
                      Uclo_getUofUword(objHandle, &pages[p], 
                                       addr + p_offset))
                   == INVLD_UWORD) 
                {
                    ERRINFO(("get invalide uword 0x%"XFMT64"\n", uword));  
                    return (UCLO_FAILURE);
                }    
            } 
            else 
            {
                if(halAe_GetUwords(handle, (unsigned char)hwAeNum,
                                   addr + p_offset, 1, &fEngUword)
                   != HALAE_SUCCESS) 
                {
                    ERRINFO(("get invalide uword 0x%"XFMT64"\n", uword));  
                    return (UCLO_FAILURE);
                }    
                /* Uclo_getUofUword() takes an address that is relative 
                 * to the start of the page. So we don't need to add in 
                 * the physical offset of the page. */
                if((uword = 
                      Uclo_getUofUword(objHandle, &pages[p], addr))
                   == INVLD_UWORD) 
                {
                    ERRINFO(("get invalide uword 0x%"XFMT64"\n", uword));  
                    return (UCLO_FAILURE);
                }    
            }

            /* ignore parity */
            fEngUword &= ICP_UWORD_MASK;
            uword &= ICP_UWORD_MASK;   

            if(fEngUword != uword) 
            {
                ERRINFO((" fEngUword =0x%"XFMT64", uword=0x%"XFMT64"\n",
                         fEngUword,uword));  
                return (UCLO_FAILURE);
            }    
        }
    }

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Maps the memory location pointed to by the addrPtr parameter to the 
 *      microcode Object File (UOF). The library verifies that the mapped 
 *      object fits within the size specified by the memSize parameter and 
 *      that the object checksum is valid.
 *      If the readOnly parameter is not equal to zero, then the library copies
 *      any region of memory that it needs to modify. Also, the memory region, 
 *      (addrPtr through addrPtr + memSize + 1), should not be modified by the
 *      caller while the image is mapped to the region -- unless through the 
 *      use of uclo library functions.
 *      The user should call UcLo_DeleObj() to remove reference to the object 
 *      and to free the resources that was allocated by the library.
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param addrPtr - IN Pointer to the memory location where the Ucode Object 
 *                     File image resides
 * @param memSize - IN An integer indicating the size of the memory region 
 *                     pointed to by the addrPtr parameter  
 * @param readOnly - IN Indicates whether the memory space being pointed to by
 *                       addrPtr is read-only initialize
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/
int 
UcLo_MapObjAddr(icp_firml_handle_t *handle,
                void *addrPtr, 
                int memSize, 
                int readOnly)
{
    uof_fileHdr_T       *fileHdr;
    uclo_objHandle_T    *objHdl;
    void                *objAddr = addrPtr;
    int                 status = UCLO_SUCCESS;

    if(!handle || !objAddr || memSize < UCLO_MIN_UOF_SIZE) 
    {
        ERRINFO(("memsize=0x%x\n",memSize));  
        return (UCLO_BADARG);
    }   
    
    if(!(objHdl = UcLo_getObjHandle())) 
    {
        ERRINFO(("get bad object handle\n"));  
        return (UCLO_MEMFAIL);
    }    

    if (handle->sysMemInfo.fwAuth == 1)
    {
        if (handle->sobjHandle != NULL)
        {
            UcLo_DelSuof(handle);
        }
        
        if ((handle->sobjHandle = 
            (void *) UcLo_getSObjHandle()) == NULL)
        {
            ERRINFO(("get bad object handle\n"));  
            return (UCLO_MEMFAIL);
        }

        return UcLo_MapSuofAddr (handle, addrPtr, memSize);        
    }

    if(readOnly)
    {
        if(!(objHdl->objBuf = osalMemAlloc(memSize))) 
        {
            ERRINFO(("malloc object buffer error\n"));  
            return (UCLO_MEMFAIL);
        }
        memmove(objHdl->objBuf,
               addrPtr,
               memSize);
        objAddr = objHdl->objBuf;
    }

    /* map the file header */
    fileHdr = (uof_fileHdr_T *)objAddr;

   /* verify file id */
    if((status =
          UcLo_verifyFile((char *)fileHdr, &objHdl->uofMinVer, 
                          &objHdl->uofMajVer)) 
       != UCLO_SUCCESS)
    {
        if(readOnly && objHdl->objBuf)
        {
            osalMemFree(objHdl->objBuf);
        }
        osalMemFree(objHdl);
        return (status);
    }
    
    objHdl->objCopied = 0;
    
    /* get the UOF object chunk  */
    if(!(objHdl->objHdr =
         UcLo_mapFileChunk((char *)objAddr, fileHdr, UOF_OBJS)))
    {
        if(readOnly && objHdl->objBuf)
        {
            osalMemFree(objHdl->objBuf);
        }
        osalMemFree(objHdl);
        ERRINFO(("object file chunk is null\n")); 
        return (UCLO_BADOBJ);
    }

    /* get the UOF-dbgObj file header  */
    objHdl->dbgObjHdr = UcLo_mapFileChunk((char *)objAddr, fileHdr, DBG_OBJS);

    if (UcLo_isProfileSupported())
    {
        /* get the UOF-prfObj file header  */
        objHdl->prfObjHdr =
                UcLo_mapFileChunk((char *)objAddr, fileHdr, PRF_OBJS);
    }
    handle->objHandle = objHdl;

    if((status = UcLo_overlayObj(handle, readOnly)))
    {
        osalMemFree(objHdl->objHdr);
        if(objHdl->dbgObjHdr) 
        {
           osalMemFree(objHdl->dbgObjHdr); 
        }   

        if (UcLo_isProfileSupported())
        {
            if(objHdl->prfObjHdr) 
            {
               osalMemFree(objHdl->prfObjHdr);
            }   
        }

        if(readOnly && objHdl->objBuf)
        {
            osalMemFree(objHdl->objBuf);
        }
        osalMemFree(objHdl);

        handle->objHandle = NULL;
        return (status);
    }

    return (status);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Removes all references to the UOF image -- if one was either loaded 
 *      or mapped. The memory region of mapped UOF, by calling 
 *      UcLo_MapImageAddr, is not deallocated by this function and it is 
 *      responsible of the caller to explicitly delete it
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/
int 
UcLo_DeleObj(icp_firml_handle_t *handle)
{
    int          a;
    uclo_objHandle_T     *objHandle;

    if(!handle) 
    {
        ERRINFO(("NULL handle\n "));  
        return (UCLO_NOOBJ);
    }    
    
    objHandle = (uclo_objHandle_T *)handle->objHandle;

    /* check if an object is loaded or mapped */
    if(!objHandle || !objHandle->objHdr || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("uof object handle error\n "));  
        return (UCLO_NOOBJ);
    }    
    
    halAe_DeleteUcloCallback(UcLo_ProcessHalCallbacks, (void*) handle);

    if(objHandle->uwordsCpyBuf)
    {
        osalMemFree(objHandle->uwordsCpyBuf);
    }
    
    MUTEX_DESTROY(objHandle->overlayMutex);

    if (objHandle->bkptCallbackHandle) 
    {
        halAe_TerminateCallbackThd(handle, objHandle->bkptCallbackHandle);

        objHandle->bkptCallbackHandle = NULL;
    }

    /* free resource that were allocated by the library */
    for(a = 0; a < objHandle->numUniqueImage; a++)
    {
        /* free encapsulated pages */
        if(objHandle->uniqueAeImage[a].pages) 
        {
            osalMemFree(objHandle->uniqueAeImage[a].pages);
        }    
    }

    for(a = 0; a < (int)objHandle->numAEs; a++)
    {
        UcLo_FreeAeData(&objHandle->aeData[a]);
    }

    if(objHandle->objCopied)
    {
        if(objHandle->objHdr)
        {
            if(objHandle->objHdr->fBuf)
            {
                osalMemFree(objHandle->objHdr->fBuf);
            }
            osalMemFree(objHandle->objHdr);
        }
        if(objHandle->dbgObjHdr)
        {
            if(objHandle->dbgObjHdr->fBuf)
            {
                osalMemFree(objHandle->dbgObjHdr->fBuf);
            }
            osalMemFree(objHandle->dbgObjHdr);
        }
        
        if (UcLo_isProfileSupported())
        {
            if(objHandle->prfObjHdr)
            {
                if(objHandle->prfObjHdr->fBuf)
                {
                    osalMemFree(objHandle->prfObjHdr->fBuf);
                }
                osalMemFree(objHandle->prfObjHdr);
            }
        }
    }
    else
    {
        if(objHandle->objHdr)
        {
            osalMemFree(objHandle->objHdr);
        }        
        if(objHandle->dbgObjHdr)
        {
            osalMemFree(objHandle->dbgObjHdr);
        }
        
        if (UcLo_isProfileSupported())
        {
            if(objHandle->prfObjHdr)
            {
                osalMemFree(objHandle->prfObjHdr);
            }
        }
    }

    if((objHandle->readOnly) && (objHandle->objBuf))
    {
        osalMemFree(objHandle->objBuf);
    }

    osalMemFree(objHandle);
    handle->objHandle = NULL;

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     List the uninitialized ucode symbols in the UOF image
 *
 * @param handle - IN
 * @param ucodeImageName - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_NOOBJ, UCLO_UNINITVAR, UCLO_IMGNOTFND
 * 
 * 
 *****************************************************************************/
static int 
UcLo_listUninitSym(icp_firml_handle_t *handle, 
                   char *ucodeImageName)
{
    int                 p, status=UCLO_SUCCESS;
    uof_Image_T         *image;
    uof_encapPage_T     *pages;
    uclo_objHandle_T    *objHandle=(uclo_objHandle_T *)handle->objHandle;
    unsigned int        lastVarName=0;
    uof_encapAe_T       *encapImage;

    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("uof object handle error\n "));  
        return (UCLO_NOOBJ);
    }    
    if(!(encapImage = UcLo_getEncapImage(objHandle, ucodeImageName))) 
    {
        ERRINFO(("UcLo_getEncapImage error\n"));  
        return (UCLO_IMGNOTFND);
    }    

    /* the image exists */
    if(!(image = encapImage->imagePtr)) 
    {
        ERRINFO(("get image error\n "));  
        return (UCLO_NOOBJ);
    }    
    if(!(pages = encapImage->pages)) 
    {
        ERRINFO((" bad image pages"));  
        return (UCLO_NOOBJ);
    }    

   /* traverse the pages */
    for(p = 0; p < image->numOfPages; p++)
    {
        uof_importVar_T   *impVar;
        unsigned int v;

        /* check all instance of the variable */
        for(v = 0; v < pages[p].numImpVar; v++)
        {
            impVar = &pages[p].impVar[v];

            if((!GET_FIXUP_ASSIGN(impVar->valueAttrs)))
            {
                /* attempt to print only one instance of the
                 * uninitialized symbol; this works because
                 * the instances are normally grouped */
                if(impVar->name != lastVarName)
                {
                    if(objHandle->encapUofObj.objHdr->cpuType
                                 == EP80579_CPU_TYPE)
                    { /* EP80579 UOF */
                        DBGINFO(("WARNING: Uninitialized symbol \
                          '%s' in image '%s', default value=0x%x\n", \
                          UcLo_getString(&objHandle->strTable,
                                         impVar->name),
                          UcLo_getString(&objHandle->strTable,
                                         image->imageName),
                          (unsigned int)impVar->value));
                    }    
                    else
                    { /* ACCELCOMP and later UOF */
                        if((objHandle->encapUofObj.objHdr->cpuType
                                 == ACCEL_COMP_TYPE) ||
                           (objHandle->encapUofObj.objHdr->cpuType
                                 == ACCELCOMP_C_CPU_TYPE) ||
                           (objHandle->encapUofObj.objHdr->cpuType
                                 == ACCELCOMP_B_CPU_TYPE) ||
                            (objHandle->encapUofObj.objHdr->cpuType
                                  == ACCELCOMP_RS_CPU_TYPE) ||
                           (objHandle->encapUofObj.objHdr->cpuType
                                 == ACCELCOMP_R_CPU_TYPE))
                        {
                            DBGINFO(("WARNING: Uninitialized symbol \
                              '%s' in image '%s', \
                              default value=0x%"XFMT64"\n", \
                              UcLo_getString(&objHandle->strTable,
                                             impVar->name),
                              UcLo_getString(&objHandle->strTable,
                                             image->imageName),
                              impVar->value));
                        }
                    }
                    lastVarName = impVar->name;
                }
                status = UCLO_UNINITVAR;
            }
        }
    }
    return (status);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Binds a value to the Ucode symbol in all of the UOF images
 *
 * @param handle - IN
 * @param ucodeSymName - IN
 * @param value - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_NOOBJ, UCLO_BADARG, UCLO_SYMNOTFND
 * 
 * 
 *****************************************************************************/
static int 
UcLo_bindAllImagesSym(icp_firml_handle_t *handle, 
                      char *ucodeSymName, 
                      uint64 value)
{
    int                 i, p, validSym=0;
    uof_Image_T         *image;
    uof_encapPage_T     *pages;
    uclo_objHandle_T    *objHandle=(uclo_objHandle_T *)handle->objHandle;
    char                *str;

    if(!ucodeSymName) 
    {
        ERRINFO(("bad argument, ucodeSymName is NULL\n"));  
        return (UCLO_BADARG);
    }    
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information\n "));  
        return (UCLO_NOOBJ);
    }    

    /* bind the symbol in all the images */
    for(i = 0; i < objHandle->numUniqueImage; i++)
    {
        if(!(image = objHandle->uniqueAeImage[i].imagePtr)) 
        {
             continue;
        }     
        if(!(pages = objHandle->uniqueAeImage[i].pages)) 
        {
             continue;
        }     

        /* traverse the pages */
        for(p = 0; p < image->numOfPages; p++)
        {
            uof_importVar_T   *impVar;
            unsigned int v;

            /* update all instance of the variable */
            for(v = 0; v < pages[p].numImpVar; v++)
            {
                impVar = &pages[p].impVar[v];
                str = UcLo_getString(&objHandle->strTable, impVar->name);
                if(!str) 
                {
                    continue;
                }    
                if(!UcLo_strcmp(ucodeSymName, str,
                                image->sensitivity))
                {
                    impVar->value = (import_var_type)value;
                    SET_FIXUP_ASSIGN(impVar->valueAttrs, 1);
                    validSym = 1;
                }
            }
        }
    }
    if(!validSym)
    {
        ERRINFO(("WARNING: Symbol '%s' is not found in \
                  any of the UOF image(s)\n", \
                  ucodeSymName));
        return (UCLO_SYMNOTFND);
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Binds a value to the Ucode symbol in the UOF image
 *
 * @param handle - IN
 * @param encapImage - IN
 * @param ucodeSymName - IN 
 * @param value - IN 
 *
 * @retval  UCLO_SUCCESS, UCLO_NOOBJ, UCLO_BADARG, UCLO_SYMNOTFND
 * 
 * 
 *****************************************************************************/
static int 
UcLo_bindSym(icp_firml_handle_t *handle, 
             uof_encapAe_T *encapImage,
             char *ucodeSymName, 
             uint64 value)
{
    int                    p, validSym=0;
    uof_Image_T            *image;
    uof_encapPage_T        *pages;
    uclo_objHandle_T    *objHandle=(uclo_objHandle_T *)handle->objHandle;
    char                   *str;
    uof_importVar_T   *impVar;
    unsigned int v;

    if(!ucodeSymName) 
    {
        ERRINFO(("bad argument, ucodeSymName is NULL\n ")); 
        return (UCLO_BADARG);
    }    
    if(!objHandle || !objHandle->objHdr->fBuf || !encapImage) 
    {
        ERRINFO(("bad object handle information\n "));  
        return (UCLO_NOOBJ);
    }    

    /* the image exists */
    if(!(image = encapImage->imagePtr)) 
    {
        ERRINFO(("No image exists\n")); 
        return (UCLO_NOOBJ);
    }    
    if(!(pages = encapImage->pages)) 
    {
        ERRINFO(("bad image pages\n "));  
        return (UCLO_NOOBJ);
    }    

   /* traverse the pages */
    for(p = 0; p < image->numOfPages; p++)
    {
        /* update all instance of the variable */
        for(v = 0; v < pages[p].numImpVar; v++)
        {
            impVar = &pages[p].impVar[v];
            str = UcLo_getString(&objHandle->strTable, impVar->name);
            if(!str) 
            {
                continue;
            }    
            if(!UcLo_strcmp(ucodeSymName, str,
                            image->sensitivity))
            {
                impVar->value = (import_var_type)value;
                SET_FIXUP_ASSIGN(impVar->valueAttrs, 1);
                validSym = 1;
            }
        }
    }
    if(!validSym) 
    {
        ERRINFO(("No valid Sym \n"));  
        return (UCLO_SYMNOTFND);
    }    
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Creates an association between an application value and a microcode 
 *      symbol. It initializes all occurrences of the specified symbol in the 
 *      UOF image to the 32-bit value, or portion of the 32-bit value, as 
 *      defined by the ucode assembler (uca). 
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ucodeImageName - IN Pointer to a character string containing the name
 *                            of the AccelEngine image
 * @param ucodeSymName - IN String pointer to the name of the microcode symbol 
 *                          to bind 
 * @param value - IN An unsigned 32-bit value of which to initialize the 
 *                   microcode symbols 
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_SYMNOTFND Symbol not found
 * 
 * 
 *****************************************************************************/
int 
UcLo_BindSymbol(icp_firml_handle_t *handle, 
                char *ucodeImageName, 
                char *ucodeSymName, 
                uint64 value)
{
    uclo_objHandle_T    *objHandle=(uclo_objHandle_T *)handle->objHandle;
    uof_encapAe_T        *encapImage;

    if(!ucodeSymName || !ucodeImageName) 
    {
        ERRINFO(("bad ucodeSymName or ucodeImageName\n"));  
        return (UCLO_BADARG);
    }    
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information "));  
        return (UCLO_NOOBJ);
    }    
    if(!(strcmp(ucodeImageName, "*")))
    {
        return (UcLo_bindAllImagesSym(handle, ucodeSymName, value));
    }

    if(!(encapImage = UcLo_getEncapImage(objHandle, ucodeImageName))) 
    {
      ERRINFO(("No encamImage information\n "));  
      return (UCLO_IMGNOTFND);
    }  

    return (UcLo_bindSym(handle, encapImage, ucodeSymName, value));
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Binds a value to the microcode symbol in the UOF image. Because 
 *      multiple AccelEngines may be assigned to the same UOF image, the 
 *      result of this function applies to the specified AccelEngine
 *      and to all assigned AccelEngines
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * @param ucodeSymName - IN String pointer to the name of the microcode symbol 
 *                          to bind 
 * @param value - IN An unsigned 32-bit value of which to initialize the 
 *                   microcode symbols 
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_SYMNOTFND Symbol not found
 * 
 * 
 *****************************************************************************/
int 
UcLo_AeBindSymbol(icp_firml_handle_t *handle, 
                  unsigned char hwAe, 
                  char *ucodeSymName, 
                  uint64 value)
{
    unsigned int swAe, s;
    int status=UCLO_SUCCESS, stat = UCLO_SUCCESS;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }    
    if((swAe = UcLo_getSwAe(objHandle->prodType, hwAe)) == UCLO_BADAE) 
    {
        ERRINFO(("bad swAe number\n"));  
        return (UCLO_BADARG);
    }    
    if(!objHandle->aeData[swAe].numSlices) 
    {
        ERRINFO(("No slices information\n "));  
        return (UCLO_SYMNOTFND);
    }    

    for(s=0; s < objHandle->aeData[swAe].numSlices; s++) 
    {
        if((stat = 
              UcLo_bindSym(handle, 
                           objHandle->aeData[swAe].aeSlice[s].encapImage, 
                           ucodeSymName, value)))
        {
            if(stat == UCLO_SYMNOTFND) 
            {
                status = stat;
            }    
            else 
            {
                if(stat != UCLO_SUCCESS) 
                {
                     return (stat);  /* return UCLO_NOOBJ or UCLO_BADARG */
                }
            }    
        }
    }    
    return (status);  /* return UCLO_SUCCESS or UCLO_SYMNOTFND */
}

/**
 *****************************************************************************
 * @ingroup uclo
 * 
 * @description
 *     Writes the specified page in the UOF image to the assigned
                uEngines. This only does the writing of ustore.
 *
 * @param objHandle - IN
 * @param encapPage - IN
 * @param swAe - IN 
 *
 * @retval  UCLO_SUCCESS, UCLO_*
 * 
 *****************************************************************************/
int 
UcLo_writeUimagePageRaw(icp_firml_handle_t *handle, 
                        uof_encapPage_T  *encapPage, 
                        unsigned int swAe)
{
    unsigned int     hwAeNum;
    unsigned int     uPhysicalAddr, uRelativeAddr, i, numWords, cpyLen;
    int              status = UCLO_SUCCESS;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    
    uword_T          fillPat;

    if (swAe >= UOF_MAX_NUM_OF_AE)
    {
        ERRINFO((" Invalid swAe index\n"));
        return (UCLO_FAILURE);
    }
    
    hwAeNum = objHandle->hwAeNum[swAe];
    /* load the page starting at appropriate ustore address */
    if((status = 
           UcLo_fixupLocals(handle, encapPage, 
                            (unsigned char)swAe))
        == UCLO_FAILURE) 
    {
        ERRINFO((" UcLo_fixupLocals error\n"));  
        return (UCLO_FAILURE);
    }

    /* get fill-pattern from an image -- they are all the same */
    memmove(&fillPat,
            objHandle->uniqueAeImage[0].imagePtr->fillPattern, 
            sizeof(uword_T));
    
    uPhysicalAddr = encapPage->begPhyAddr;
    uRelativeAddr = 0;
    numWords = encapPage->numMicroWords;
    while(numWords)
    {
        if(numWords < UWORD_CPYBUF_SIZE) 
        {
           cpyLen = numWords;
        }               
        else 
        {
           cpyLen = UWORD_CPYBUF_SIZE;
        }   

        /* load the buffer */
        for(i=0; i < cpyLen; i++)
        {
            /* keep below code structure in case there are 
             * different handling for shared secnarios */
            if(!objHandle->aeData[swAe].shareableUstore) 
            {
                /* Uclo_getUofUword() takes an address that is relative 
                 * to the start of the page. So we don't need to add 
                 * in the physical offset of the page. */
                if(encapPage->pageRegion !=0 ) 
                {
                    /* initial uRelativeAddr needs to be the total ustore 
                     * size of lower regions instead of zero, 
                       Uclo_getUofUword_nocheck does not check invalid address,
                       and then just retrieve correct uword from uword block.
                       There is no easy way to calculate total ustore size of 
                       lower regions in current .uof and loader design, 
                       hencn Uclo_getUofUword_nocheck is provided. */
                    if((objHandle->uwordsCpyBuf[i] =
                           Uclo_getUofUword_nocheck(objHandle, encapPage,
                                                    uRelativeAddr+i))
                        == INVLD_UWORD)
                    {
                        /* fill hole in the uof */
                        objHandle->uwordsCpyBuf[i] = fillPat;
                    }    
                } 
                else
                {
                    /* for mixing case, it should take physical address */
                    if((objHandle->uwordsCpyBuf[i] =
                           Uclo_getUofUword(objHandle, encapPage, 
                                            uPhysicalAddr+i))
                       == INVLD_UWORD)
                    {
                        /* fill hole in the uof */
                        objHandle->uwordsCpyBuf[i] = fillPat;
                    }    
                } 
            } 
            else 
            {
                /* For shared ctrl store, in uof file, the page relative  
                 * address in the next AE will be added by the physical 
                 * offset of the page in the previous AE. So here, we need 
                 * to pass the address to Uclo_getUofUword after adding 
                 * in the physical offset of the page in the previous AE. */ 
                if(encapPage->pageRegion !=0 ) 
                {
                    /* initial uRelativeAddr needs to be the total ustore size
                     * of lower regions instead of zero,
                     * Uclo_getUofUword_nocheck does not check invalid address, 
                     * and then just retrieve correct uword from uword block. 
                     * There is no easy way to calculate total ustore size of 
                     * lower regions in current .uof and loader design, 
                     * hencn Uclo_getUofUword_nocheck is provided. */
                    if((objHandle->uwordsCpyBuf[i] =
                           Uclo_getUofUword_nocheck(objHandle, encapPage,
                                                    uRelativeAddr+i))
                        == INVLD_UWORD) 
                    {
                        /* fill hole in the uof */
                        objHandle->uwordsCpyBuf[i] = fillPat;
                    }    
                } 
                else
                {
                    if((objHandle->uwordsCpyBuf[i] = 
                        Uclo_getUofUword(objHandle, encapPage,
                                         uPhysicalAddr+i))
                       == INVLD_UWORD) 
                    {
                        /* fill hole in the uof */
                        objHandle->uwordsCpyBuf[i] = fillPat;
                    }    
                }
            }
        }

        /* copy the buffer to ustore */
        if(objHandle->aeData[swAe].shareableUstore)
        {
            if(halAe_PutCoalesceUwords(handle, (unsigned char)hwAeNum,
                                       uPhysicalAddr, cpyLen, 
                                       objHandle->uwordsCpyBuf)
               != HALAE_SUCCESS) 
            {
                ERRINFO(("halAe_PutCoalesceUwords error\n "));  
                return (UCLO_FAILURE);
            }
        }
        else 
        {
            if(halAe_PutUwords(handle, (unsigned char)hwAeNum, 
                               uPhysicalAddr, cpyLen, objHandle->uwordsCpyBuf)
               != HALAE_SUCCESS)
            {
                ERRINFO(("halAe_PutUwords error\n "));  
                return (UCLO_FAILURE);
            }
        }
        uPhysicalAddr += cpyLen;
        uRelativeAddr += cpyLen;
        numWords -= cpyLen;
    }

    return (status);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *     Writes the specified page in the UOF image to the assigned uEngines.
 *
 * @param objHandle - IN
 * @param ucodeImageName - IN
 * @param startingPageChange - IN
 *
 * @retval  UCLO_SUCCESS, UCLO_NOOBJ, UCLO_IMGNOTFND, UCLO_BADARG, UCLO_UNINITVAR,
 *               UCLO_IVDWARN, UCLO_FAILURE
 * 
 * 
 *****************************************************************************/
static int 
UcLo_writeUimagePages(icp_firml_handle_t *handle,
                      char *ucodeImageName, 
                      int *startingPageChange)
{
    unsigned int        ctxMask, s;
    uof_encapAe_T       *encapImage;
    uclo_page_t         *page, *oldPage=NULL;
    uof_Image_T         *image;
    unsigned char        swAe;
    unsigned int         hwAeNum;
    int                  retval = UCLO_SUCCESS, status = UCLO_SUCCESS;
    int                  pageNum, ctx;
    uclo_objHandle_T *objHandle =
                          (uclo_objHandle_T *)handle->objHandle;         
    
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }    
    if((retval = UcLo_initGlobals(handle))) 
    {
        if(retval != UCLO_IVDWARN) {
           return (retval);
        }   
    }    
    if(!(encapImage = UcLo_getEncapImage(objHandle, ucodeImageName))) 
    {
        ERRINFO(("No encapImage information\n"));  
        return (UCLO_IMGNOTFND);
    }    

    image = encapImage->imagePtr;

    if(CTX_MODE(image->aeMode) == MAX_CTX) 
    {
        ctxMask = 0xff; /* eight ctx-mode */
    }    
    else
    {
        ctxMask = 0x55; /* four ctx-mode */
    }    

    /* load the default page and set assigned CTX PC 
     * to the entrypoint address */
    for(swAe = 0;
        (swAe < objHandle->numAEs) && (swAe < UOF_MAX_NUM_OF_AE);
        swAe++)
    {
        /* check if AE is fused-out */
        hwAeNum = UcLo_getHwAe(objHandle->prodType, swAe);
        /* check if AE is assigned AE */
        if(isBitSet(image->aeAssigned, hwAeNum))
        {
            /* find the slice to which this image is assigned */
            for (s=0; s < objHandle->aeData[swAe].numSlices; s++)
            {
                if(image->ctxAssigned
                     & objHandle->aeData[swAe].aeSlice[s].assignedCtxMask) 
                {
                    break;
                }    
            }
            if (s >= objHandle->aeData[swAe].numSlices) 
            { 
                continue;
            }    

            for (pageNum = 0; pageNum < image->numOfPages; pageNum++) 
            {
                page = &objHandle->aeData[swAe].aeSlice[s].pages[pageNum];
                /* Only load pages loaded by default */
                if (! page->encapPage->defPage) 
                { 
                    continue;
                }    

                if((status = UcLo_doPageIn(handle, swAe, page,
                                           oldPage, startingPageChange))) 
                {
                    if (status == UCLO_UNINITVAR) 
                    {
                        retval = status;
                    }    
                    else
                    { 
                        return (status);
                    }    
                }
            } /* end for pageNum */

            /* Assume starting page is page 0 */
            page = &objHandle->aeData[swAe].aeSlice[s].pages[0];
            for (ctx=0; ctx<MAX_CONTEXTS; ctx++) 
            {
                if (ctxMask & (1<<ctx)) 
                {
                    objHandle->aeData[swAe].aeSlice[s].currentPage[ctx] = page;
                }    
                else
                {
                    objHandle->aeData[swAe].aeSlice[s].currentPage[ctx] = NULL;
                }    
            }

            /* set the live context */
            if(halAe_SetLiveCtx(handle, (unsigned char)hwAeNum,
                                image->ctxAssigned)) 
            {
               ERRINFO(("halAe_SetLiveCtx error\n"));  
               return (UCLO_FAILURE);
            }   

            /* set context PC to the image entrypoint address */
            if(halAe_PutPC(handle, (unsigned char)hwAeNum,
                           image->ctxAssigned, image->entryAddress)) 
            {
               ERRINFO((" halAe_PutPC error\n"));  
               return (UCLO_FAILURE);
            }   
        }
    }

    /* store the checksum in the HAL for convenience */
    halAe_setUofChecksum(handle, objHandle->objHdr->checksum);

    if ((! objHandle->bkptCallbackHandle) && (image->numOfPages > 1)) 
    {
        halAe_SpawnIntrCallbackThdEx(HALAE_INTR_ATTN_BKPT_MASK,
                                     UcLo_IntrCallback,
                                     (void*)handle,
                                     OSAL_DEFAULT_THREAD_PRIORITY,
                                     0x14, /* callback priority */
                                     &objHandle->bkptCallbackHandle);
    }

    return (retval);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Initializes the Loader library. AccelEngines whose corresponding bit 
 *      is set in aeMask are set to the powerup default state. The library 
 *      assumes that those AccelEngines that are not specified are in a
 *      reset state
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
 * @param aeMask           IN An integer mask specifying the AccelEngines to 
 *                            initialize
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/
void 
UcLo_InitLibUeng(icp_firml_handle_t *handle,
                 unsigned int aeMask)
{
    static int onlyOnce = 0;

    if(onlyOnce) 
    {
       return;
    }   
    onlyOnce = 1;
 
    halAe_Init(handle, aeMask);
    return;
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      This function initializes the uclo library and performs any AccelEngine
 *      driver initialization. It should be called prior to calling any of the 
 *      other functions in the uclo library
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
 * @param  - none
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/
void 
UcLo_InitLib(icp_firml_handle_t *handle)
{
    halAe_Init(handle, UCLO_ALLAE);
    return;
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Writes the specified image in the UOF object to the assigned 
 *      AccelEngines 
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_UNINITVAR A variable was not initialized
 * 
 * 
*****************************************************************************/
int 
UcLo_WriteUimage(icp_firml_handle_t *handle, 
                 char *ucodeImageName)
{
    int stat;
    int startingPageChange = 0;
    if(!handle)
    {
        ERRINFO(("bad argument, handle is NULL\n"));  
        return (UCLO_NOOBJ);
    }    
    if((stat = UcLo_writeUimagePages(handle,
                                     ucodeImageName,
                                     &startingPageChange)) == UCLO_UNINITVAR) 
    {
        UcLo_listUninitSym(handle, ucodeImageName);
    }    
    if (startingPageChange) 
    {
        halAe_CallNewPageCallback(handle, END_OF_PAGE_CHANGE, 0, 0, 0);
    }

    return (stat);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *     Computes the amount of free ustore in each of the assigned uEngines
 *               and informs HAL of the results.
 *
 * @param objHandle - IN
 * @param 
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE
 * 
 * 
 *****************************************************************************/
int 
UcLo_computeFreeUstore(icp_firml_handle_t *handle)
{
    /* We have to wait until all images are written before we can calculate
     * the amount of free ustore in each AE. This is because there may be
     * multiple images loaded into different slices in the same AE and/or 
     * shareable ustore may be enabled.
    */
    uclo_aedata_t  *aeData;
    unsigned int    sliceNum, totalUwords;
    int             imageNum;
    unsigned char   swAe = 0, swNeighAe = 0, shareableUstore = 0;
    unsigned int    hwAeNum = 0, hwNeighNum = 0;
    uof_Image_T    *imagePtr;
    uclo_objHandle_T *objHandle = 
                          (uclo_objHandle_T *)handle->objHandle;         

    for (swAe = 0;
         (swAe < objHandle->numAEs) && (swAe < UOF_MAX_NUM_OF_AE);
         swAe++)
    {
        if(halAe_VerifyAe(handle, swAe) != HALAE_SUCCESS)
        {
            continue;
        }
        if ((hwAeNum = UcLo_getHwAe(objHandle->prodType, swAe)) == UCLO_BADAE) 
        {
            DBGINFO(("fused-out swAe=0x%x\n", swAe));  
            continue;
        }    

        totalUwords = 0;

        /* get the AE data for the AE */
        aeData = &objHandle->aeData[swAe];

        /* determine whether this AE is sharing ustore with its neighbor AE. If
           sharing is enabled and both AEs have images loaded into them, then 
           both will have their shareableUstore flag set. However, if one of
           the AEs doesn't have an image loaded into it, then its flag won't
           be set. So in that case we need to check the paired AE's flag. */
        hwNeighNum = UcLo_getSharedUstoreNeigh(objHandle, hwAeNum);
        if(halAe_VerifyAe(handle, (unsigned char)hwNeighNum) != HALAE_SUCCESS)
        {
            hwNeighNum = UCLO_BADAE;
        }
        if ((hwNeighNum != UCLO_BADAE) && (hwNeighNum < MAX_AE))
        {
           swNeighAe = objHandle->swAe[hwNeighNum];
           if (swNeighAe >= UOF_MAX_NUM_OF_AE)
           {
               return (UCLO_NEIGHNOTFND);
           }
        }
        if (aeData->numSlices == 0)
        {
            if (hwNeighNum != UCLO_BADAE)
            {
                /* no images loaded into the AE, so use the paired AE's flag. */
                shareableUstore =
                  (unsigned char)(objHandle->aeData[swNeighAe].shareableUstore);
            }
            else
            {
                shareableUstore = 0;
            }
        }
        else
        {
            /* AE has an image loaded, so use its flag. */
            shareableUstore =
                (unsigned char)(objHandle->aeData[swAe].shareableUstore);
        }

        /* if ustore is being shared and the neighbor AE number is 
         * lower than the current AE number, then skip processing 
         * this AE because it was already processed when its neighbor
         * was processed. */
        if (shareableUstore && (hwNeighNum != UCLO_BADAE)
            && (swNeighAe < swAe)) 
        {
            continue;
        }    

        /* the # of used uwords for each slice has already been computed
         * taking into account the offset required if other images are mixed
         * in the same ustore or if ustore is shared. 
         * so traverse all slices and find the maximum uwords used. */
        for (sliceNum = 0; sliceNum < aeData->numSlices; sliceNum++)
        {
            if (aeData->aeSlice[sliceNum].assignedCtxMask != 0)
            {
                imagePtr = aeData->aeSlice[sliceNum].encapImage->imagePtr;
                for (imageNum = 0;
                     imageNum < objHandle->numUniqueImage;
                     imageNum++)
                {
                    if (objHandle->uniqueAeImage[imageNum].imagePtr == imagePtr)
                    {
                        /* Found image that is loaded into this slice. 
                           Update the uwords used if necessary. */
                        if (objHandle->uniqueAeImage[imageNum].numUwordsUsed
                            > totalUwords) 
                        {
                            totalUwords =
                              objHandle->uniqueAeImage[imageNum].numUwordsUsed;
                        }
                    }
                }
            }
        }

        /* if sharing ustore then we need to consider the neighbor. Else, just
           treat this AE independently */
        if (shareableUstore && (hwNeighNum != UCLO_BADAE))
        {
            /* sharing ustore */
            /* get the AE data for the neighbor AE */
            aeData = &objHandle->aeData[swNeighAe];
            for (sliceNum = 0; sliceNum < aeData->numSlices; sliceNum++)
            {
                if (aeData->aeSlice[sliceNum].assignedCtxMask != 0)
                {
                    imagePtr = aeData->aeSlice[sliceNum].encapImage->imagePtr;
                    for (imageNum = 0;
                         imageNum < objHandle->numUniqueImage;
                         imageNum++)
                    {
                        if (objHandle->uniqueAeImage[imageNum].imagePtr
                            == imagePtr)
                        {
                            /* Found image that is loaded into this slice.
                               Update the uwords used if necessary. */
                            if (objHandle->uniqueAeImage[imageNum].numUwordsUsed
                                > totalUwords) 
                            {
                                totalUwords = 
                               objHandle->uniqueAeImage[imageNum].numUwordsUsed;
                            }    
                        }
                    }
                }
            }

            /* two shared AE ustore have the same free ustore view */
            if (halAe_SetUstoreFreeMem(handle, (unsigned char)hwAeNum,
                     totalUwords, objHandle->ustorePhySize*0x2 - totalUwords)
                != HALAE_SUCCESS) 
            {
                ERRINFO(("halAe_SetUstoreFreeMem error\n"));  
                return (UCLO_FAILURE);
            }    
            if (halAe_SetUstoreFreeMem(handle, (unsigned char)hwNeighNum,
                     totalUwords, objHandle->ustorePhySize*0x2 - totalUwords)
                != HALAE_SUCCESS) 
            {
                ERRINFO(("halAe_SetUstoreFreeMem error\n"));  
                return (UCLO_FAILURE);
            }    
        }
        else
        {
            /* not sharing ustore. let the ae lib know 
             * the number of useful micro words */
            if (halAe_SetUstoreFreeMem(handle, (unsigned char)hwAeNum, 
                     totalUwords, objHandle->ustorePhySize - totalUwords)
                != HALAE_SUCCESS) 
            {
                ERRINFO(("halAe_SetUstoreFreeMem error\n"));  
                return (UCLO_FAILURE);
            }    
        }
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      This function interact with Authentication Firmware/FCU to authenticate 
 *      simg object
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
 * @param  - none
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/
int 
UcLo_AuthenticateFW(icp_firml_handle_t *handle, fw_auth_desc_t* desc)
{
    unsigned int fcuSts = 0, retry = 0;
    //fw_auth_desc_t* phyAddr = desc;
    uint64 phyAddr = 0;

    phyAddr = ADD_ADDR(desc->cssHeaderH,
                       desc->cssHeaderL) - 
                       SIMG_OFFSET_2_DESC;

    fcuSts = GET_CAP_CSR (handle, FCU_STATUS);
    if (((fcuSts >> FCU_STS_DONE_POS) & FCU_STS_DONE_MASK) != 0)
    {
        ERRINFO(("UcLo_AuthenticateFW error (FCU_Done = 1)\n"));
        return UCLO_FAILURE;    
    }

    SET_CAP_CSR (handle, FCU_DRAM_ADDR_HI, (phyAddr >> BITS_IN_DWORD));

    SET_CAP_CSR (handle, FCU_DRAM_ADDR_LO, (phyAddr & 0xffffffff));

    /* issue fw_authenticate command */
    SET_CAP_CSR (handle, FCU_CONTROL, FCU_CTRL_CMD_AUTH);

    /* TBD: check authentication is successful or not */
    /* Disable timeout here for debug purpose */
    //while (retry < FW_AUTH_MAX_RETRY)
    while (1)
    {
        fcuSts = GET_CAP_CSR (handle, FCU_STATUS);
        
        if ((fcuSts & FCU_AUTH_STS_MASK) == FCU_STS_VERI_DONE)
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
        ERRINFO(("UcLo_AuthenticateFW error (FCU_STATUS = 0x%x)\n", 
                fcuSts & FCU_AUTH_STS_MASK));
        return UCLO_FAILURE;
    }

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      This function interact with Authentication Firmware/FCU to authenticate 
 *      simg object
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
 * @param  - none
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/
int 
UcLo_UnmapAuthFw (icp_firml_handle_t *handle, fw_auth_desc_t** desc)
{
    icp_firml_dram_desc_t dramDesc;

    if ((desc == NULL) || (*desc == NULL))
    {
        return UCLO_FAILURE;
    }

    dramDesc.dramBaseAddr_v = (long)(*desc);
    dramDesc.dramBaseAddr = 0;
    dramDesc.dramSize = ((fw_auth_chunk_t*)(*desc))->chunkSize;
    
    halAe_ContinuousDramFree (handle, &dramDesc);

    * desc = NULL;
    return UCLO_SUCCESS;
}


/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      This function map simg inside authentication firmware to a descriptor
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
 * @param  - none
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/
int 
UcLo_MapAuthFw (icp_firml_handle_t *handle, char* image, 
                unsigned int size, fw_auth_desc_t** desc)
{
    css_header_T* cssHeader = NULL;
    uint64 virtAddr = 0,  phyAddr = 0, virtBase = 0;
    unsigned int length = 0;
    icp_firml_dram_desc_t imgDesc;

#ifdef MMP_ENDIAN_SWAP
    unsigned int i = 0;
#endif

    if (SIMG_OFFSET_2_DESC != sizeof(fw_auth_chunk_t) - sizeof(char*))
    {
        ERRINFO(("SIMG_OFFSET_2_DESC is not enough to store fw_auth_chunk\n"));
        return UCLO_FAILURE;
    }

    /* check input image size */
    if (size > (sizeof (css_header_T) + 
        CSS_FWSK_MODULUS_LEN + CSS_FWSK_EXPONENT_LEN + 
        CSS_SIGNATURE_LEN + CSS_MAX_IMAGE_LEN))
    {
        ERRINFO(("UcLo_MapAuthFw error, input image size overflow %d\n", 
                size));
        return UCLO_FAILURE;
    }

    /* cleanup desc */
    if (*desc != 0)
    {
        UcLo_UnmapAuthFw (handle, desc);
    }
    *desc = NULL;

    /* memory chunk includes descriptor & simg  (fw_auth_chunk_t)
     *   +---------------+
     *   | descriptor    |
     *   +---------------+
     *   | chunk size    |
     *   +---------------+
     *   | reserved      |
     *   +---------------+ SIMG_OFFSET_2_DESC
     *   | simg          | 
     *   |               |
     *   +---------------+
     */
    cssHeader = (css_header_T *)image;
    if (cssHeader->fwType == CSS_AE_FIRMWARE)
    {
        length = CSS_AE_SIMG_LEN;
    }
    else
    {
        length = size + CSS_FWSK_PADDING_LEN;
    }
    
    length += SIMG_OFFSET_2_DESC;

    /* allocate memory chunck to store descriptor & simg */
    if (HALAE_SUCCESS != halAe_ContinuousDramAlloc(handle, &imgDesc, length))
    {
        ERRINFO(("UcLo_MapAuthFw error, allocate continuous dram fail\n"));
        return UCLO_FAILURE;
    }

    if ((imgDesc.dramBusAddr % 0x8 != 0) ||
        (imgDesc.dramBusAddr == 0))
    {
        halAe_ContinuousDramFree (handle, &imgDesc);
        return UCLO_MEMFAIL;
    }

    /* memory chunck size will be used during free time */
    ((fw_auth_chunk_t*)((long)(imgDesc.dramBaseAddr_v)))->chunkSize = 
        imgDesc.dramSize;

    virtAddr = imgDesc.dramBaseAddr_v + SIMG_OFFSET_2_DESC;
    phyAddr  = imgDesc.dramBusAddr + SIMG_OFFSET_2_DESC;
    virtBase = virtAddr;

    *desc = (fw_auth_desc_t*)((long)imgDesc.dramBaseAddr_v);
    (*desc)->cssHeaderH = (unsigned int)(phyAddr >> BITS_IN_DWORD);
    (*desc)->cssHeaderL = (unsigned int)(phyAddr & 0xffffffff);    

    /* CSS image layout
     * ----------------------------------+
     * | css header ()                   |
     * +---------------------------------+
     * | pub key (256B) -> pad to 512B   |
     * +---------------------------------+
     * | signature (256B)                |
     * +---------------------------------+
     * | meModeData ()    |              |
     * +------------------+              |
     * | meInitData ()    |    mmpData   |
     * +------------------+              |
     * | meInstructions ()|              |
     * +------------------+--------------+
     */
     
    /* css header */
    virtAddr = virtBase;
    osalMemCopy ((void *)((long)virtAddr), (void *)((long)image),
                sizeof (css_header_T));

    /* pub key */
    phyAddr = ADD_ADDR((*desc)->cssHeaderH,
                       (*desc)->cssHeaderL) + 
                       sizeof (css_header_T);
    virtAddr = virtAddr + sizeof (css_header_T);

    (*desc)->fwskPubH = (unsigned int)(phyAddr >> BITS_IN_DWORD);
    (*desc)->fwskPubL = (unsigned int)(phyAddr & 0xffffffff);

    /* modulus */
#ifdef MMP_ENDIAN_SWAP
    for (i = 0; i < CSS_FWSK_MODULUS_LEN;)
    {
        osalMemCopy ((void *)((long)virtAddr + i),
                (void *)(image + sizeof (css_header_T) + 
                CSS_FWSK_MODULUS_LEN - i - sizeof (unsigned int)),
                sizeof (unsigned int));
        i = i + sizeof (unsigned int);
    }
#else
    osalMemCopy ((void *)(long)virtAddr,
                 (void *)(image + sizeof (css_header_T)),
                 CSS_FWSK_MODULUS_LEN);
#endif

    /* padding */
    osalMemSet ((void *)((long)virtAddr + CSS_FWSK_MODULUS_LEN), 0,
                CSS_FWSK_PADDING_LEN);

    /* exponent */
    osalMemCopy (
      (void *)((long)virtAddr + CSS_FWSK_MODULUS_LEN + CSS_FWSK_PADDING_LEN),
      (void *)(image + sizeof (css_header_T) + CSS_FWSK_MODULUS_LEN),
      sizeof (unsigned int));

    /* signature */
    phyAddr = ADD_ADDR((*desc)->fwskPubH,
                       (*desc)->fwskPubL) + 
                       CSS_FWSK_PUB_LEN;
    virtAddr = virtAddr + CSS_FWSK_PUB_LEN;

    (*desc)->signatureH = (unsigned int)(phyAddr >> BITS_IN_DWORD);
    (*desc)->signatureL = (unsigned int)(phyAddr & 0xffffffff);

#ifdef MMP_ENDIAN_SWAP
    for (i = 0; i < CSS_SIGNATURE_LEN; )
    {
        osalMemCopy (
            (void *)((long)virtAddr + i),
            (void *)(image + sizeof (css_header_T) + CSS_FWSK_MODULUS_LEN + 
            CSS_FWSK_EXPONENT_LEN + CSS_SIGNATURE_LEN - i - 
            sizeof (unsigned int)),
            sizeof (unsigned int));
        i = i + sizeof (unsigned int);
    }
#else
    osalMemCopy ((void *)(long)virtAddr,
        (void *)(image + sizeof (css_header_T) + CSS_FWSK_MODULUS_LEN + 
        CSS_FWSK_EXPONENT_LEN),
        CSS_SIGNATURE_LEN);
#endif

    /* simg */
    phyAddr = ADD_ADDR((*desc)->signatureH,
                       (*desc)->signatureL) + 
                       CSS_SIGNATURE_LEN;
    virtAddr = virtAddr + CSS_SIGNATURE_LEN;

    (*desc)->imgH = (unsigned int)(phyAddr >> BITS_IN_DWORD);
    (*desc)->imgL = (unsigned int)(phyAddr & 0xffffffff);

    
    (*desc)->imgLen = size - sizeof (css_header_T) -
        CSS_FWSK_MODULUS_LEN - CSS_FWSK_EXPONENT_LEN - CSS_SIGNATURE_LEN;

    osalMemCopy ((void *)((long)virtAddr),
            (void *)((long)image + sizeof (css_header_T) +
            CSS_FWSK_MODULUS_LEN + CSS_FWSK_EXPONENT_LEN +
            CSS_SIGNATURE_LEN),
            (*desc)->imgLen);

    virtAddr = virtBase;
    /* AE firmware */
    if (((css_header_T *)((long)virtAddr))->fwType == CSS_AE_FIRMWARE)
    {
        (*desc)->imgAeModeDataH = (*desc)->imgH;
        (*desc)->imgAeModeDataL = (*desc)->imgL;

        phyAddr = ADD_ADDR((*desc)->imgAeModeDataH, 
                           (*desc)->imgAeModeDataL) +
                           sizeof (img_aeMode_T);

        (*desc)->imgAeInitDataH = (unsigned int)(phyAddr >> BITS_IN_DWORD);
        (*desc)->imgAeInitDataL = (unsigned int)(phyAddr & 0xffffffff);
        
        phyAddr = ADD_ADDR((*desc)->imgAeInitDataH,
                           (*desc)->imgAeInitDataL) + 
                           SIMG_AE_INIT_SEQ_LEN;

        (*desc)->imgAeInstructionsH = (unsigned int)(phyAddr >> BITS_IN_DWORD);
        (*desc)->imgAeInstructionsL = (unsigned int)(phyAddr & 0xffffffff);
    }
    /* MMP image */
    else
    {
        (*desc)->imgAeModeDataH = 0;
        (*desc)->imgAeModeDataL = 0;
        
        (*desc)->imgAeInitDataH = 0;
        (*desc)->imgAeInitDataL = 0;

        (*desc)->imgAeInstructionsH = (*desc)->imgH;
        (*desc)->imgAeInstructionsL = (*desc)->imgL;
    }

    DBGINFO(("UcLo_MapAuthFw\n cssheader=0x%x-0x%x \n"
            " img=0x%x-0x%x \n sig=0x%x-0x%x \n"
            " key=0x%x-0x%x \n modeData==0x%x-0x%x \n "
            " initData=0x%x-0x%x \n instucts =0x%x-0x%x\n",
            (*desc)->cssHeaderH,
            (*desc)->cssHeaderL,
            (*desc)->imgH,
            (*desc)->imgL,
            (*desc)->signatureH,
            (*desc)->signatureL,
            (*desc)->fwskPubH,
            (*desc)->fwskPubL,
            (*desc)->imgAeModeDataH,
            (*desc)->imgAeModeDataL,
            (*desc)->imgAeInitDataH,
            (*desc)->imgAeInitDataL,
            (*desc)->imgAeInstructionsH,
            (*desc)->imgAeInstructionsL));
    
    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      This function interact with Authentication Firmware/FCU to authenticate 
 *      simg object
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
 * @param  - none
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/
int 
Uclo_LoadFW (icp_firml_handle_t *handle, fw_auth_desc_t* desc)
{
    int i = 0;
    unsigned int fcuSts = 0, retry = 0;
    void* virtAddr = 0;

    /* issue fw_load commands */
    for (i = 0; i < MAX_AE; i++)
    {
        virtAddr = (void *)((long)desc + SIMG_OFFSET_2_DESC + 
                            sizeof (css_header_T) + CSS_FWSK_PUB_LEN + 
                            CSS_SIGNATURE_LEN);        

        if ((((img_aeMode_T *) virtAddr)->aeMask >> i) & 0x1)
        {
            retry = 0;
            
            /* load AE */
            SET_CAP_CSR (handle, 
                FCU_CONTROL,
                (FCU_CTRL_CMD_LOAD | (i << FCU_CTRL_AE_POS)));

            /* TBD: disable timeout here for debug purpose */
            //while (retry < FW_AUTH_MAX_RETRY)
            while (1)
            {
                fcuSts = GET_CAP_CSR (handle, FCU_STATUS);
                
                /* verify related AE load done */
                 if (((fcuSts & FCU_AUTH_STS_MASK) == FCU_STS_LOAD_DONE) && 
                    ((((fcuSts >> FCU_LOADED_AE_POS) & FCU_LOADED_AE_MASK) 
                    & (1 << i)) != 0))
                {
                    break;
                }
                else
                {
                    retry++;
                    osalSleep (FW_AUTH_WAIT_PERIOD);
                }
            }

#if 0                
            if (retry >= FW_AUTH_MAX_RETRY)
            {
                ERRINFO(("Uclo_LoadFW error (AE %d FCU_STATUS = 0x%x)\n",
                        i,
                        fcuSts));
                return UCLO_FAILURE;
            }
#endif
        }
    }

    return UCLO_SUCCESS;
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      This function interact with Authentication Firmware/FCU to authenticate 
 *      MMP object
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
 * @param  - none
 *
 * @retval - none
 * @retval - none
 * 
 * 
 *****************************************************************************/
int 
UcLo_AuthenticateMMP (icp_firml_handle_t *handle, char *filePtr,
                      uint64 fileSize)
{
    fw_auth_desc_t* desc = NULL;
    int status = UCLO_SUCCESS;
    unsigned int ScratchOffset, SramOffset, Dram0Offset, Dram1Offset, size;
    uint64 secDram = 0, i = 0;
    unsigned char ae = 0;

    /* authenticate and load mmp objects to sram*/
    if (handle->sysMemInfo.fwAuth == 1)
    {
        if ((status = UcLo_MapAuthFw (handle, filePtr,
            (unsigned int)fileSize, &desc))
            == UCLO_SUCCESS)
        {
            status = UcLo_AuthenticateFW (handle, desc);
        }

        UcLo_UnmapAuthFw(handle, &desc);
    }
    else
    {
        if (fileSize > MAX_MMP_IMG_SIZE)
        {
            ERRINFO(("MMP image size exceeds maximum limitation 0x%llx\n", 
                    fileSize));
            return UCLO_BADOBJ;
        }

        halAe_GetMemoryStartOffset(handle, 
            &ScratchOffset, 
            &SramOffset, 
            &Dram0Offset, 
            &Dram1Offset);

        if (handle->Hal_sram_virtAddr == 0)
        {
            /* retrive secure RAM address from IMR */    
            secDram = GET_CAP_CSR (handle, FCU_RAMBASE_ADDR_HI);
            SramOffset = GET_CAP_CSR (handle, FCU_RAMBASE_ADDR_LO);
            secDram = (secDram << 0x20) + SramOffset;

            if(halAe_GetExecutableAe(handle, &ae)!= HALAE_SUCCESS)
            {
                ERRINFO(("All AEs are busy\n"));
                return HALMEM_AEACTIVE;
            }

            while (i < fileSize)
            {
                /* maximum payload size is MAX_SHRAM_USED*4 bytes */
                size = (((fileSize - i)/BYTES_IN_LONGWORD) >= MAX_SHRAM_USED) ? 
                    MAX_SHRAM_USED : 
                    (((fileSize - i)%BYTES_IN_LONGWORD == 0) ?
                     ((fileSize - i)/BYTES_IN_LONGWORD) : 
                     (((fileSize - i)/BYTES_IN_LONGWORD) + 1));

                /* move data from Dram to SharedRam */
                if (HALMEM_SUCCESS == halMem_PutSharedRam (
                    handle, 0, ae, 0, size, (unsigned int *)&filePtr[i], 0))
                {
                    /* move data from SharedRam to IMR which can 
                     * only be accessed by AE */
                    if (HALMEM_SUCCESS == halMem_SharedRamToDRam (
                        handle, 0, ae, 0, size, 
                        (unsigned int *)(long)(secDram + i), 0))
                    {
                        i = i + size*BYTES_IN_LONGWORD;
                        continue;
                    }
                }

                status = UCLO_FAILURE;
                break;
            }
        }
        else
        {
            /* write data to sRAM BAR */
            secDram = SramOffset;
            UcLo_wrSramBytes (handle, secDram, 
                (unsigned int *)filePtr, (unsigned int)fileSize);
        }
    }

    return (status);
}


/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Writes all the microcode images to one or more appropriate 
 *      AccelEngine(s)
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/
int 
UcLo_WriteUimageAll(icp_firml_handle_t *handle)
{
    int              i, stat, retVal = UCLO_SUCCESS;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    char             *imageName;
    int              startingPageChange = 0, bailOut=0;
    fw_auth_desc_t*  desc = NULL;

    if ((!objHandle || !objHandle->objHdr->fBuf) &&
        (!handle->sobjHandle))
    {
        ERRINFO(("Bad object/sobject handle information\n"));  
        return (UCLO_NOOBJ);
    }    

    /* authenticate and load simg objects */
    if (handle->sysMemInfo.fwAuth == 1)
    {
        for (i = 0; 
               i < (int)((uclo_suof_handle_t *
                   )handle->sobjHandle)->imgTable.numImgs;
               i++)
        {
            if (i < (int)((uclo_suof_handle_t *)
                handle->sobjHandle)->imgTable.numImgs - 1)
            {
                if (((uclo_suof_handle_t *)
                    handle->sobjHandle)->imgTable.ImgHdr[i].aeMask & 0x1)
                {
                    ERRINFO(("AE0 image must be loaded after all others\n"));
                    UcLo_UnmapAuthFw (handle, &desc);
                    return UCLO_FAILURE;
                }
            }
            
            /* create descriptor */
            if (UCLO_SUCCESS != UcLo_MapAuthFw (
                handle,
                (char *)((uclo_suof_handle_t *)
                (handle->sobjHandle))->imgTable.ImgHdr[i].simgBuf,
                (unsigned int)(((uclo_suof_handle_t *)
                (handle->sobjHandle))->imgTable.ImgHdr[i].simgLen),
                &desc))
            {
                ERRINFO(("UcLo_MapAuthFw returns fail \n")); 
                UcLo_UnmapAuthFw (handle, &desc);
                return UCLO_FAILURE;
            }

            /* authenticate img */
            DBGINFO(("Start UcLo_AuthenticateFW \n"));  
            if (UCLO_SUCCESS != UcLo_AuthenticateFW (handle, desc))
            {
                UcLo_UnmapAuthFw (handle, &desc);
                return UCLO_FAILURE;
            }

            /* load img */
            DBGINFO(("Start Uclo_LoadFW \n"));  
            if (UCLO_SUCCESS != Uclo_LoadFW (handle, desc))
            {
                UcLo_UnmapAuthFw (handle, &desc);
                return UCLO_FAILURE;
            }

            DBGINFO(("UcLo_UnmapAuthFw release all memory\n"));
            UcLo_UnmapAuthFw (handle, &desc);
        }
               
        return UCLO_SUCCESS;
    }
    
    if (!objHandle)
    {
        ERRINFO(("Bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }

    for (i = 0; (i < objHandle->numUniqueImage) && !bailOut; i++)
    {
        imageName = 
                UcLo_getString(&objHandle->strTable, 
                               objHandle->uniqueAeImage[i].imagePtr->imageName);
        if ((stat = 
             UcLo_writeUimagePages(handle, imageName, &startingPageChange))
            != UCLO_SUCCESS)
        {
            switch(stat)
            {
            case UCLO_UNINITVAR:
                retVal= stat; /* warning -- uninitialized value in image */
                UcLo_listUninitSym(handle, imageName);
                break;
            case UCLO_IVDWARN:
                retVal= stat; /* warning -- unknown image or symbol in IVD */
                break;
            default:
               retVal = stat;
               bailOut=1;
               break;
            }
        }
    }

    /* We have to wait until all images are written before we can calculate
     * the amount of free ustore in each AE. This is because there may be  
     * multiple images loaded into different slices in the same AE and/or 
     * shareable ustore may be enabled. */
    stat = UcLo_computeFreeUstore(handle);

    /* if we previously detected an error, 
     * then let it stand. Else, use the new return status */
    if (retVal == UCLO_SUCCESS) 
    {
        retVal = stat;
    }    

    if (startingPageChange) 
    {
        halAe_CallNewPageCallback(handle, END_OF_PAGE_CHANGE, 0, 0, 0);
    }    

    return (retVal);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Compares the UOF image that was either loaded or mapped to the content 
 *      of the assigned AccelEngine(s)
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ucodeImageName - IN Pointer to character string containing name of 
 *                             the AccelEngine image
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/
int 
UcLo_VerifyUimage(icp_firml_handle_t *handle, 
                  char *ucodeImageName)
{
    uof_encapAe_T     *encapImage;
    uof_Image_T         *image;
    unsigned char        swAe;
    uclo_objHandle_T     *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("Bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }    

    if(!(encapImage = UcLo_getEncapImage(objHandle, ucodeImageName))) 
    {
        ERRINFO(("bad encapImage information\n")); 
        return (UCLO_IMGNOTFND);
    }    

    /* compare all assigned uEngine images */
    image = encapImage->imagePtr;
    for(swAe = 0;
        (swAe < objHandle->numAEs) && (swAe < UOF_MAX_NUM_OF_AE); 
        swAe++)
    {
        if(isBitSet(image->aeAssigned, UcLo_getHwAe(objHandle->prodType, swAe)))
        {
            if(UcLo_verifyEncapUimage(handle, encapImage, swAe)) 
            {
               ERRINFO((" UcLo_verifyEncapUimage error\n"));  
               return (UCLO_FAILURE);
            }   
        }
    }

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Compares the content of the specified AccelEngine to its assigned UOF 
 *      image that was either mapped or loaded
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * @retval UCLO_IMGNOTFND Image not found
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/
int 
UcLo_VerifyUengine(icp_firml_handle_t *handle, 
                   unsigned char hwAe)
{
    uclo_objHandle_T     *objHandle = (uclo_objHandle_T *)handle->objHandle;
    unsigned int swAe, s;
    int status = UCLO_SUCCESS;

    if((!objHandle || !objHandle->objHdr || 
        !objHandle->objHdr->fBuf) && (!handle->sobjHandle))
    {
        ERRINFO(("bad object/sobject handle information\n"));  
        return (UCLO_NOOBJ);
    }

    /* return success if FW Authentication feature is eanble */
    if (handle->sysMemInfo.fwAuth == 1)
    {
        return status;
    }

    if (!objHandle)
    {
        ERRINFO(("bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }
    
    swAe = UcLo_getSwAe(objHandle->prodType, hwAe);
    if((swAe == UCLO_BADAE) || (swAe >= UOF_MAX_NUM_OF_AE)) 
    {
        ERRINFO(("bad hwAe 0x%x\n",hwAe));  
        return (UCLO_BADARG);
    }    

    if(objHandle->aeData[swAe].numSlices == 0) 
    {
        ERRINFO(("bad slices number\n"));  
        return (UCLO_IMGNOTFND);
    }    

    for(s=0; s < objHandle->aeData[swAe].numSlices; s++)
    {
        if(!objHandle->aeData[swAe].aeSlice[s].encapImage ||
           !objHandle->aeData[swAe].aeSlice[s].encapImage->pages ||
           !objHandle->aeData[swAe].aeSlice[s].encapImage->imagePtr) 
        {
            ERRINFO(("Bad encapImage information\n")); 
            return (UCLO_IMGNOTFND);
        }      
        if((status = 
            UcLo_verifyEncapUimage(handle, 
                    objHandle->aeData[swAe].aeSlice[s].encapImage,
                    (unsigned char)swAe))) 
        {
            break;
        }    
    }
    return (status);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieve the locations, sizes, and alignment of the C compiler variables
 *      memory segments 
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param varMemSeg - OUT A pointer to variable segment structure
 * 
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/
int 
UcLo_GetVarMemSegs(icp_firml_handle_t *handle, 
                   uclo_varmemseg_t *varMemSegs)
{
    unsigned int hwAe, swAe, s;
    uclo_objHandle_T     *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information "));  
        return (UCLO_NOOBJ);
    }    
    if(!varMemSegs) 
    {
        ERRINFO(("bad argument,  varMemSegs is NULL\n ")); 
        return (UCLO_BADARG);
    }    

    varMemSegs->sramBaseAddr = objHandle->encapUofObj.varMemSeg->sramBase;
    varMemSegs->sramSize = objHandle->encapUofObj.varMemSeg->sramSize;
    varMemSegs->sramAlignment = objHandle->encapUofObj.varMemSeg->sramAlignment;

    varMemSegs->sdramBaseAddr = objHandle->encapUofObj.varMemSeg->sdramBase;
    varMemSegs->sdramSize = objHandle->encapUofObj.varMemSeg->sdramSize;
    varMemSegs->sdramAlignment = 
            objHandle->encapUofObj.varMemSeg->sdramAlignment;

    varMemSegs->sdram1BaseAddr = objHandle->encapUofObj.varMemSeg->sdram1Base;
    varMemSegs->sdram1Size = objHandle->encapUofObj.varMemSeg->sdram1Size;
    varMemSegs->sdram1Alignment = 
            objHandle->encapUofObj.varMemSeg->sdram1Alignment;

    varMemSegs->scratchBaseAddr = objHandle->encapUofObj.varMemSeg->scratchBase;
    varMemSegs->scratchSize = objHandle->encapUofObj.varMemSeg->scratchSize;
    varMemSegs->scratchAlignment = 
            objHandle->encapUofObj.varMemSeg->scratchAlignment;

    osalMemSet(varMemSegs->aeUmemSeg, 0, 
            objHandle->numAEs * sizeof(uclo_varumemseg_t));

    for(swAe = 0;
        (swAe < objHandle->numAEs) && (swAe < UOF_MAX_NUM_OF_AE);
        swAe++)
    {
        if((hwAe = UcLo_getHwAe(objHandle->prodType, (unsigned char)swAe))
           == UCLO_BADAE) 
        {
            continue;
        }        

        for(s=0; s < objHandle->aeData[swAe].numSlices; s++)
        {
            /* check if a page is assigned */
            if((!objHandle->aeData[swAe].aeSlice[s].encapImage)
               || (!objHandle->aeData[swAe].aeSlice[s].encapImage->pages)) 
            {
                continue;
            }    

            /* get the number of ustore used
             * -- assuming that umem starts at the end of the ucode through the
             * last highest uaddr referenced. */
            if(varMemSegs->aeUmemSeg[hwAe].umemSize 
               < objHandle->aeData[swAe].aeSlice[s].encapImage->numUwordsUsed) 
            {
                varMemSegs->aeUmemSeg[hwAe].umemSize
                 = objHandle->aeData[swAe].aeSlice[s].encapImage->numUwordsUsed;
            }    
        }
    }

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Returns the specified object's CRC checksum
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param checksum - OUT The specified object's checksum
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/
int 
UcLo_GetChecksum(icp_firml_handle_t *handle, 
                 unsigned int *checksum)
{
    uclo_objHandle_T     *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(!checksum) 
    {
        ERRINFO(("bad argument, check sum is NULL\n"));  
        return (UCLO_BADARG);
    }    
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information\n ")); 
        return (UCLO_NOOBJ);
    }    
    *checksum = objHandle->objHdr->checksum;

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Returns the mask specifying the AccelEngines that are assigned to an 
 *      image in the UOF
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 *
 * @retval Mask of the assigned AEs
 * 
 * 
 *****************************************************************************/
unsigned int 
UcLo_GetAssignedAEs(icp_firml_handle_t *handle)
{
    int i;
    unsigned int aeMask=0;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    uclo_suof_handle_t *sobjHandle = (uclo_suof_handle_t *)handle->sobjHandle;

    if (handle->sysMemInfo.fwAuth == 1)
    {
        if(sobjHandle && sobjHandle->imgTable.ImgHdr)
        {
            for (i = 0; i < (int)(sobjHandle->imgTable.numImgs); i++)
            {
                aeMask |= sobjHandle->imgTable.ImgHdr[i].aeMask;
            }
        }
    }
    else
    {
        if(objHandle && objHandle->objHdr->fBuf)
        {
            /* traverse the images and set aeMask */
            for(i = 0; i < objHandle->numUniqueImage; i++)
            {
                aeMask |= objHandle->uniqueAeImage[i].imagePtr->aeAssigned;
            }
        }
    }

    return (aeMask);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Returns the mask specifying the contexts that are assigned to the 
 *      AccelEngine in the UOF 
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 *
 * @retval Mask of the assigned contexts
 * 
 * 
 *****************************************************************************/
unsigned int 
UcLo_GetAssignedCtxs(icp_firml_handle_t *handle, 
                     unsigned char hwAe)
{
    unsigned int ctxMask=0, swAe, s;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(objHandle && objHandle->objHdr->fBuf)
    {
        if((swAe = UcLo_getSwAe(objHandle->prodType, hwAe)) == UCLO_BADAE) 
        {
            return (ctxMask);
        }    

        for(s=0; s < objHandle->aeData[swAe].numSlices; s++) 
        {
            ctxMask |= objHandle->aeData[swAe].aeSlice[s].assignedCtxMask;
        }    
    }
    return (ctxMask);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *     Delete resource the contents of a dbgAeInfo_Image_T structure.
 *
 * @param image - IN
 * @param 
 *
 * @retval  pointer to the chunk header or NULL if not found
 * 
 * 
 *****************************************************************************/
void 
UcLo_deleDbgInfo(dbgAeInfo_Image_T *image)
{
    if(!image) 
    {
        return;
    }    
    while(image->numImage--)
    {
        if(image->srcImage[image->numImage].numSrcLine) 
        {
            osalMemFree(image->srcImage[image->numImage].srcLine);
        }    
        if(image->srcImage[image->numImage].label) 
        {
            osalMemFree(image->srcImage[image->numImage].label);
        }    
    }
    return;
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieve the udbug information from debug section of an uof.
 *
 * @param dbgObjHdr
 * @param image
 * @param strTable
 *
 * @retval  UCLO_SUCCESS, UCLO_FAILURE, UCLO_BADARG
 * 
 * 
 *****************************************************************************/
int 
UcLo_getDbgInfo(uof_objHdr_T *dbgObjHdr,
                dbgAeInfo_Image_T *image,
                uof_strTab_T *strTable) /* strTable may be NULL */
{
    unsigned int         numDbgInfo = 0;
    unsigned int         i, ii, lastAddr;
    void                *cur = NULL;
    dbg_Image_T         *dbgImage;
    dbg_Source_T        *dbgSrc;
    dbgAeInfo_SrcLine_T *srcLine;
    uof_chunkHdr_T      *chunk;
    dgb_ObjTable_T      *objTab;
    dbg_Label_T         *dbgLbl;
    dbgAeInfo_Label_T   *dbgInfoLabel;

    if(!dbgObjHdr || !image) 
    {
        ERRINFO(("bad argument, dbgObjHdr or Image\n"));  
        return (UCLO_BADARG);
    }    

    image->numImage = 0;

    while((chunk = (uof_chunkHdr_T *)UcLo_findChunk(dbgObjHdr, DBG_IMAG, cur)))
    {
        cur = chunk;
        dbgImage = (dbg_Image_T *)(chunk->offset + (char *)dbgObjHdr);

        objTab = (dgb_ObjTable_T *)((char *)dbgObjHdr + dbgImage->srcTabOffset);
        dbgSrc = (dbg_Source_T *)((char *)objTab + sizeof(dgb_ObjTable_T));
        image->srcImage[image->numImage].aeAssigned = dbgImage->aeAssigned;
        image->srcImage[image->numImage].ctxAssigned = dbgImage->ctxAssigned;

        /* count the number of useful dbg info */
        lastAddr = 0xffffffff;
        numDbgInfo = 0;
        for(i = 0; i < objTab->numEntries; i++)
        {
            /* skip src lines that are associated with the same uword */
            if(lastAddr == dbgSrc[i].addr) 
            {
                continue;
            }    
            else
            {
                lastAddr = dbgSrc[i].addr;
            }    
            numDbgInfo++;
        }

        if(!(srcLine = 
             (dbgAeInfo_SrcLine_T *)osalMemAlloc(sizeof(dbgAeInfo_SrcLine_T)
                                                 * numDbgInfo)))
        {             
            UcLo_deleDbgInfo(image);
            ERRINFO(("malloc dbgAeInfo_SrcLine_T error\n "));
            return (UCLO_FAILURE);
        }

        ii = 0;
        lastAddr = 0xffffffff;
        for(i = 0; i < objTab->numEntries; i++)
        {
            /* store only the last src line info 
             * that is associated with uword */
            if((lastAddr != 0xffffffff) && (lastAddr != dbgSrc[i].addr)) 
            {
                ii++;
            }    

            lastAddr = dbgSrc[i].addr;
            srcLine[ii].uAddr = dbgSrc[i].addr;
            srcLine[ii].brkPtAllowed = dbgSrc[i].validBkPt;
            srcLine[ii].deferCount = dbgSrc[i].deferCount;
            srcLine[ii].brAddr = dbgSrc[i].brAddr;
            srcLine[ii].regAddr = dbgSrc[i].regAddr;
            srcLine[ii].regType = dbgSrc[i].regType;
            srcLine[ii].ctxArbKill = dbgSrc[i].ctxArbKill;
            srcLine[ii].reserved1 = 0;
            if (strTable) 
            {
                srcLine[ii].text = UcLo_getString(strTable, dbgSrc[i].lines);
            }    
            else
            {
                srcLine[ii].text = "";
            }    
        }

        image->srcImage[image->numImage].numSrcLine
                = (unsigned short)numDbgInfo;
        image->srcImage[image->numImage].srcLine = srcLine;

        /* Look up label info */
        if (strTable) 
        {
            objTab = 
                    (dgb_ObjTable_T *)((char *)dbgObjHdr
                                             + dbgImage->lblTabOffset);
            dbgLbl = (dbg_Label_T*)((char *)objTab + sizeof(dgb_ObjTable_T));
            dbgInfoLabel = 
                    (dbgAeInfo_Label_T*)osalMemAlloc(sizeof(dbgAeInfo_Label_T)
                                             * objTab->numEntries);
            if (dbgInfoLabel == NULL) 
            {
                osalMemFree(srcLine);
                UcLo_deleDbgInfo(image);
                ERRINFO(("malloc dbgAeInfo_Label_T error\n"));  
                return (UCLO_FAILURE);
            }
            for (i=0; i<objTab->numEntries; i++) 
            {
                dbgInfoLabel[i].name = UcLo_getString(strTable, dbgLbl[i].name);
                dbgInfoLabel[i].addr = dbgLbl[i].addr;
            }

            image->srcImage[image->numImage].numLabels
                    = (unsigned short)objTab->numEntries;
            image->srcImage[image->numImage].label     = dbgInfoLabel;
        } 
        else 
        {
            image->srcImage[image->numImage].numLabels = 0;
            image->srcImage[image->numImage].label     = NULL;
        }

        image->numImage++;
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieves the udbug information from debug section of an UOF. If the 
 *      UOF is linked with debug information, then this function allocates 
 *      memory, extracts the debug information, and returns a pointer in the 
 *      image parameter. It is the caller's responsibility to delete the debug 
 *      information. 
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param image - OUT An pointer reference to the debug information
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/
int 
UcLo_GetDebugInfo(icp_firml_handle_t *handle, 
                  dbgAeInfo_Image_T *image)
{
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(!objHandle || !objHandle->dbgObjHdr || !objHandle->dbgObjHdr->fBuf) 
    {
        ERRINFO((" bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }    
    return (UcLo_getDbgInfo((uof_objHdr_T *)objHandle->dbgObjHdr->fBuf,
                           image, &objHandle->dbgObjStrTable));
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieves the storage information of a local symbol
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 * @param symName - IN String pointer to the name of the microcode local symbol 
 *                     to bind 
 * @param symInfo - OUT A pointer reference to the symbol information
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/
int 
UcLo_GetLocalSymbolInfo(icp_firml_handle_t *handle, 
                        unsigned int hwAe, 
                        char *symName, 
                        uclo_symbolinfo_t *symInfo)
{
    uof_initMem_T *initMem;
    char localName[MAX_VARNAME];
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(!symInfo) 
    {
        ERRINFO(("bad argument, symInfo is NULL\n"));  
        return (UCLO_BADARG);
    }    
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information\n"));
        return (UCLO_NOOBJ);
    }    

    /* format the local variable name */
    LOCAL_NAME(localName, symName, hwAe); 
    if(!(initMem = UcLo_findMemSym(objHandle, localName))) 
    {
        ERRINFO(("UcLo_findMemSym errror\n"));  
        return (UCLO_SYMNOTFND);
    }    
    if(initMem->scope != LOCAL_SCOPE) 
    {
        ERRINFO(("memory scope is not local\n"));  
        return (UCLO_SYMNOTFND);
    }    

    symInfo->memType = initMem->region;
    symInfo->baseAddr = initMem->addr;
    symInfo->allocSize = initMem->numBytes;

    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieves the storage information of a global symbol
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param symName - IN String pointer to the name of the microcode global symbol 
 *                     to bind 
 * @param symInfo - OUT A pointer reference to the symbol information
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/
int 
UcLo_GetGlobalSymbolInfo(icp_firml_handle_t *handle, 
                         char *symName, 
                         uclo_symbolinfo_t *symInfo)
{
    uof_initMem_T *initMem;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if(!symInfo) 
    {
        ERRINFO(("bad argument, symInfo is NULL\n"));  
        return (UCLO_BADARG);
    }    
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }    

    if(!(initMem = UcLo_findMemSym(objHandle, symName))) 
    {
        ERRINFO(("UcLo_findMemSym errror\n")); 
        return (UCLO_SYMNOTFND);
    }    
    if(initMem->scope != GLOBAL_SCOPE) 
    {
        ERRINFO(("memory scope is not local\n"));  
        return (UCLO_SYMNOTFND);
    }    

    symInfo->memType = initMem->region;
    symInfo->baseAddr = initMem->addr;
    symInfo->allocSize = initMem->numBytes;
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieve the application meta-data asociated with the AE or any
 *      AE if hwAe is UCLO_BADAE.
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param ae - IN Specifies a AccelEngine
 *
 * @retval Pointer to character string containing meta-data
 * 
 * 
 *****************************************************************************/
char *
UcLo_GetAppMetaData(icp_firml_handle_t *handle, 
                    unsigned int hwAe)
{
    unsigned int swAe, appMetadata = 0xffffffff, s;
    int i;
    uclo_objHandle_T *objHandle;
    uclo_suof_handle_t *sobjHandle;

    if(!handle)
    {
        return (NULL);
    }

    if (handle->sysMemInfo.fwAuth == 1)
    {
        sobjHandle = (uclo_suof_handle_t *)handle->sobjHandle;
        if(!sobjHandle) 
        {
            return (NULL);
        }
    
        for (i = 0; i < (int)(sobjHandle->imgTable.numImgs); i++)
        {
            if (UCLO_BADAE == hwAe)
            {
                if (sobjHandle->imgTable.ImgHdr->pAppMetaData != 0)
                {
                    return ((char *)sobjHandle->imgTable.ImgHdr->pAppMetaData);
                }
            }
            else
            {
                if (((0x1 << hwAe) & sobjHandle->imgTable.ImgHdr->aeMask) != 0)
                {
                    return ((char *)sobjHandle->imgTable.ImgHdr->pAppMetaData);
                }
            }
        }
        
        return (NULL);
    }
        
    objHandle = (uclo_objHandle_T *)handle->objHandle;
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        return (NULL);
    }    

    if(hwAe == UCLO_BADAE)
    {
        /* find any image that has a meta-data */
        for(i = 0; i < objHandle->numUniqueImage; i++)
        {
            if(objHandle->uniqueAeImage[i].imagePtr->appMetadata == 0xffffffff)
            {
                continue;
            }    
            appMetadata = objHandle->uniqueAeImage[i].imagePtr->appMetadata;
            break;
        }
    }
    else
    {
        if((swAe = UcLo_getSwAe(objHandle->prodType, (unsigned char)hwAe))
           == UCLO_BADAE) 
        {
            return (NULL);
        }    
        for(s=0; s < objHandle->aeData[swAe].numSlices; s++)
        { 
            if((!objHandle->aeData[swAe].aeSlice[s].encapImage) 
              || (!objHandle->aeData[swAe].aeSlice[s].encapImage->imagePtr)) 
            {
                continue;
            }    
            if((appMetadata
         = objHandle->aeData[swAe].aeSlice[s].encapImage->imagePtr->appMetadata)
            == 0xffffffff) 
            {
                continue;
            }    
        }
    }
    if(appMetadata == 0xffffffff) 
    {
        return (NULL);
    }    
    return (UcLo_getString(&objHandle->strTable,    appMetadata));
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Retrieves the physical ustore address for the specified virtual 
 *      ustore address
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param hwAe - IN Specifies a AccelEngine
 * @param virtUaddr - IN Virtual ustore address 
 * @param pageNum - IN Page number associated with the physical address
 * @param phyUaddr - OUT Physical ustore address
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_ADDRNOTFOUND Address not found
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/
int 
UcLo_GetPhyUaddr(icp_firml_handle_t *handle, 
                 unsigned int hwAe, 
                 unsigned int virtUaddr,
                 unsigned int*pageNum, 
                 unsigned int *phyUaddr)
{
    unsigned int swAe, p, s;
    uof_encapPage_T *encapPage;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;

    if((!pageNum) || (!phyUaddr)) 
    {
        ERRINFO(("bad argument, pageNum or phyUaddr is NULL\n"));  
        return (UCLO_BADARG);
    }    
    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }    

    if((swAe = UcLo_getSwAe(objHandle->prodType, (unsigned char)hwAe))
        == UCLO_BADAE) 
    {
        ERRINFO(("bad hw AE number = 0x%x\n", hwAe));  
        return (UCLO_BADARG);
    }    
    for(s=0; s < objHandle->aeData[swAe].numSlices; s++)
    {
        if(!objHandle->aeData[swAe].aeSlice[s].encapImage) 
        {
            continue;
        }    
        for(p=0; 
            p < objHandle->aeData[swAe].aeSlice[s]. \
                    encapImage->imagePtr->numOfPages; 
            p++)
        {
            encapPage = 
                 &objHandle->aeData[swAe].aeSlice[s].encapImage->pages[p];
            if((virtUaddr >= encapPage->begVirtAddr) 
               && (virtUaddr < 
                    (encapPage->begVirtAddr + encapPage->numMicroWords)))
            {
                    *pageNum = encapPage->pageNum;
                    *phyUaddr = (virtUaddr - encapPage->begVirtAddr)
                                +  encapPage->begPhyAddr;
                    return (UCLO_SUCCESS);
            }
        }
    }
    ERRINFO(("can find the dedicated Addr=0x%x\n",virtUaddr));
    return (UCLO_ADDRNOTFOUND);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Locate an import variable and returns its value and an attribute 
 *      indicating whether or not the variable was initialized, and whether 
 *      its scope is global or local
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
 * @param handle - IN A void pointer reference to the loaded and mapped object
 * @param hwAe - IN An integer mask specifying the AccelEngines to which to
 *                    write the value uWord at the address uAddr.
 * @param varName - IN Name of the import-variable 
 * @param importVal - OUT Pointer to structure to receive the import-variable's
 *                    value and attributes
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * @retval UCLO_NOOBJ No object was either loaded or mapped
 * 
 * 
 *****************************************************************************/
int 
UcLo_GetImportVal(icp_firml_handle_t *handle, 
                  unsigned int hwAe, 
                  char *varName, 
                  uclo_importvalue_t *importVal)
{
    uof_importVar_T *impVar;
    uclo_objHandle_T *objHandle = (uclo_objHandle_T *)handle->objHandle;
    unsigned int swAe;

    if(!objHandle || !objHandle->objHdr->fBuf) 
    {
        ERRINFO(("bad object handle information\n"));  
        return (UCLO_NOOBJ);
    }    
    if((!importVal) || (!varName)) 
    {
        ERRINFO(("bad argument, importVal or varName is NULL\n"));  
        return (UCLO_BADARG);
    }    
    if(((unsigned int)hwAe >= MAX_AE) 
       || ((1 << (unsigned int)hwAe) & BadHwAeMask) ) 
    {
        return (UCLO_BADAE);
    }    
    if(objHandle->swAe[hwAe] >= UOF_MAX_NUM_OF_AE) 
    {
        return (UCLO_BADAE);
    }    

    if(!(impVar = UcLo_getImportVar(objHandle, objHandle->swAe[hwAe], varName)))
    {
        for(swAe = 0; 
             (swAe < objHandle->numAEs) && (swAe < UOF_MAX_NUM_OF_AE); 
             swAe++) 
        {
            if((impVar = UcLo_getImportVar(objHandle, 
                                        (unsigned char)swAe, varName)) != NULL) 
            {
                break;
            }    
        }
    }
        
    if(!impVar) 
    {
        ERRINFO(("symbol information not found\n"));  
        return (UCLO_SYMNOTFND);
    }    

    importVal->value = impVar->value;
    importVal->valueAttrs = impVar->valueAttrs;
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Set the SCRATCH/SRAM/NCDRAM/CDRAM allocated base address for AccelEngine
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
 * @param scratchOffset - IN Scratch memory address offset     
 * @param sramOffset - IN SRAM memory address offset
 * @param dram0Offset - IN DRAM0 memory address offset
 * @param dram1Offset - IN DRAM1 memory address offset
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * 
 * 
 *****************************************************************************/
int 
UcLo_SetMemoryStartOffset(icp_firml_handle_t *handle,
                          unsigned int scratchOffset, 
                          unsigned int sramOffset, 
                          unsigned int dram0Offset, 
                          unsigned int dram1Offset) 
{ 
    if(halAe_SetMemoryStartOffset(handle, scratchOffset, 
                            sramOffset, dram0Offset, dram1Offset)) 
    {
        ERRINFO(("halAe_SetMemoryStartOffset error\n "));  
        return (UCLO_FAILURE);    
    }    
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      retrieve UOF version info
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
 * @param objBuf - IN a Pointer to the memory location 
 *                    where the Ucode Object image resides
 * @param uofVer - OUT UOF version info
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/
int UcLo_GetUofVer(void * objBuf,
                   unsigned int objBufSize,
                   uof_ver_t * uofVer)
{ 
    int                 i;
    uof_fileHdr_T       *fileHdr = NULL;
    uof_fileChunkHdr_T  *fileChunk = NULL;
    suof_fileHdr_T      *sfileHdr = NULL;
    suof_fileChunkHdr_T  *sfileChunk = NULL;
    void                *chunk = NULL;
    uof_objHdr_T        *objHdr = NULL;
    suof_objHdr_T       *sobjHdr = NULL;
    img_aeMode_T        *sImgHdr = NULL;

    if((!objBuf) || (!uofVer)) 
    {
        ERRINFO(("bad argument, objBuf or uofVer is NULL\n"));  
        return (UCLO_BADARG);
    }
    if((objBufSize < sizeof(uof_fileHdr_T)) &&
       (objBufSize < sizeof(suof_fileHdr_T)))
    {
        ERRINFO(("bad argument, objBufSize=%d\n", objBufSize));  
        return (UCLO_BADARG);
    }

    if (((suof_fileHdr_T *)objBuf)->fileId == SUOF_FID)
    {
        sfileHdr = (suof_fileHdr_T *)objBuf;
        uofVer->fileId = (unsigned int)sfileHdr->fileId;
        uofVer->minVer = (unsigned int)sfileHdr->minVer;
        uofVer->majVer = (unsigned int)sfileHdr->majVer;

        if(objBufSize < sizeof(suof_fileHdr_T) + 
            (sfileHdr->numChunks * sizeof(suof_fileChunkHdr_T))) 
        {
            ERRINFO(("bad argument, sobjBufSize=%d\n", objBufSize));  
            return (UCLO_BADARG);
        }
        
        /* get version info to reflect chip change in SUOF obj header */
        sfileChunk = (suof_fileChunkHdr_T *)((char *)objBuf + 
                     sizeof(suof_fileHdr_T));
        for(i = 0; i < sfileHdr->numChunks; i++)
        {
            /* find the SIMG_OBJS chunk, get version info in its header */
            if(!(strncmp(sfileChunk->chunkId, SIMG_OBJS, UOF_OBJID_LEN)))
            {
                chunk = (char *)objBuf + sfileChunk->offset;
                sobjHdr = (suof_objHdr_T *)chunk;
                sImgHdr = (img_aeMode_T *)((char *)sobjHdr + 
                    sizeof (suof_objHdr_T) +
                    sizeof (css_header_T) + CSS_FWSK_MODULUS_LEN + 
                    CSS_FWSK_EXPONENT_LEN + CSS_SIGNATURE_LEN);
                
                uofVer->acType = sImgHdr->devType;
                uofVer->minAcVer = (unsigned int)sImgHdr->devMinVer;
                uofVer->maxAcVer = (unsigned int)sImgHdr->devMaxVer;
                break;
            }
            sfileChunk++;
        }
        if (i >= sfileHdr->numChunks)
        {
            return (UCLO_FAILURE);      
        }
        
        return (UCLO_SUCCESS);
    }
    
    /* get version info to reflect UOF format change in file header */
    fileHdr = (uof_fileHdr_T *)objBuf;
    uofVer->fileId = (unsigned int)fileHdr->fileId;
    uofVer->minVer = (unsigned int)fileHdr->minVer;
    uofVer->majVer = (unsigned int)fileHdr->majVer;

    if(objBufSize < sizeof(uof_fileHdr_T) + 
       (fileHdr->numChunks * sizeof(uof_fileChunkHdr_T))) 
    {
        ERRINFO(("bad argument, objBufSize=%d\n", objBufSize));  
        return (UCLO_BADARG);
    }
    /* get version info to reflect chip change in UOF obj header */
    fileChunk = (uof_fileChunkHdr_T *)((char *)objBuf + sizeof(uof_fileHdr_T));
    for(i = 0; i < fileHdr->numChunks; i++)
    {
        /* find the UOF_OBJS chunk, get version info in its header */
        if(!(strncmp(fileChunk->chunkId, UOF_OBJS, UOF_OBJID_LEN)))
        {
            chunk = (char *)objBuf + fileChunk->offset;
            objHdr = (uof_objHdr_T *)chunk;
            uofVer->acType = objHdr->cpuType;
            uofVer->minAcVer = (unsigned int)objHdr->minCpuVer;
            uofVer->maxAcVer = (unsigned int)objHdr->maxCpuVer;
            break;
        }
        fileChunk++;
    }
    /* return fail if not found UOF_OBJS chunk */
    if(objHdr == NULL)
    {
        return (UCLO_FAILURE);    
    }
    return (UCLO_SUCCESS);
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      retrieve MMP version info
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
 * @param objBuf - IN a Pointer to the memory location 
 *                    where the Ucode Object image resides
 * @param uofVer - OUT UOF version info
 *
 * @retval UCLO_SUCCESS Operation was successful
 * @retval UCLO_FAILURE Operation failed
 * @retval UCLO_BADARG Invalid argument specified
 * 
 * 
 *****************************************************************************/
int UcLo_GetMmpVer (icp_firml_handle_t *handle,
                    void * objBuf,
                    unsigned int* mmpVer)
{
    mmpVer = 0;
    
    return (UCLO_SUCCESS);;
}

/**
 *****************************************************************************
 * @ingroup Uclo
 * 
 * @description
 *      Get the error string for provided error code
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
 * @param errCode - IN Loader library error code
 *
 * @retval Pointer to character string containing error description
 * 
 * 
 *****************************************************************************/
char *
UcLo_GetErrorStr(int errCode)
{
    switch(errCode)
    {
    case UCLO_SUCCESS:
        return ("The operation is successful.");
    case UCLO_FAILURE:
        return ("The operation is failed.");
    case UCLO_FILEFAIL:
        return ("The file operation is failed.");        
    case UCLO_BADOBJ:
        return ("Bad object.");        
    case UCLO_MEMFAIL:
        return ("The memory operation is failed.");        
    case UCLO_BADARG:
        return ("Bad function argument passed.");        
    case UCLO_NOOBJ:
        return ("Bad object handler used.");        
    case UCLO_IMGNOTFND:
        return ("Can not find specified image.");        
    case UCLO_SYMNOTFND:
        return ("Can not find specified symbol.");        
    case UCLO_NEIGHNOTFND:
        return ("Can not find neighbouring Acceleration Engine.");        
    case UCLO_UOFINCOMPAT:
        return (".uof file is incompatible with loader.");        
    case UCLO_UOFVERINCOMPAT:
        return (".uof file version is incompatible with loader.");        
    case UCLO_UNINITVAR:
        return ("There is uninitialized import variables.");        
    case UCLO_EXPRFAIL:
        return ("Incorrect expression specified.");        
    case UCLO_EXCDDRAMSIZE:
        return ("Excede dram0 size.");        
    case UCLO_EXCDDRAM1SIZE:
        return ("Excede dram1 size.");        
    case UCLO_EXCDSRAMSIZE:
        return ("Excede sram size.");        
    case UCLO_EXCDSCRTHSIZE:
        return ("Excede scratch size.");        
    case UCLO_EXCDLMEMSIZE:
        return ("Excede local memory size.");        
    case UCLO_INVLDCTX:
        return ("Invalid context mode.");        
    case UCLO_EXCDUMEMSIZE:
        return ("Excede micro-ustore size.");        
    case UCLO_ADDRNOTFOUND:
        return ("Can not find specified address.");        
    case UCLO_PAGENOTFOUND:
        return ("Can not find specified microcode page.");        
    case UCLO_IVDWARN:
        return ("Unknown image or symbol defined in IVD.");        
    case UCLO_EXCDSHRAMSIZE:
        return ("Excede shared ram size.");     
    default:
        return (NULL);
    }        
}
