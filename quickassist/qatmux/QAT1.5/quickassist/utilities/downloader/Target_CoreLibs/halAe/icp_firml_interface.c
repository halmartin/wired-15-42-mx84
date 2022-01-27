/**
 **************************************************************************
 * @file icp_firml_interface.c
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
#include "halAe_platform.h"
#include "uclo.h"
#include "icp_firml_interface.h"
#include "halAeDrv.h"
#include "halMemScrub.h"

#define MAX_HANDLES   16

/* add offset to va, unless va is zero, in that case, return 0 */
#define SADD(va, offset) (((va) != 0) ? ((va)+(offset)) : 0)

static int cnvReturnCode(int retCode);
extern int halAe_GetChipSetting(icp_firml_handle_t *handle);
extern int halAe_DisableClkGating(icp_firml_handle_t *handle);
void initVirAddr(icp_firml_handle_t * handle);

int 
icp_FirmLoader_Init(icp_firml_sys_mem_info_t *sysMemInfo, 
                    void **handle)
{
    icp_firml_handle_t  *myHandle = NULL;
    int status = ICP_FIRMLOADER_SUCCESS;

    if((handle == NULL) || (sysMemInfo == NULL))
    {
        return (ICP_FIRMLOADER_BADARG);
    }
    if(!(myHandle = 
        (icp_firml_handle_t *)osalMemAlloc(sizeof(icp_firml_handle_t)))) 
    {
        return (ICP_FIRMLOADER_FAIL);
    }    
    osalMemSet(myHandle, 0, sizeof(icp_firml_handle_t));

    if(!(myHandle->halHandle = 
        (Hal_Handle_T *)osalMemAlloc(sizeof(Hal_Handle_T)))) 
    {
        osalMemFree(myHandle); 
        return (ICP_FIRMLOADER_FAIL);
    }    
    osalMemSet(myHandle->halHandle, 0, sizeof(Hal_Handle_T));
    
    /* copy sysMemInfo to handle */
    osalMemCopy(&(myHandle->sysMemInfo), 
                sysMemInfo, sizeof(icp_firml_sys_mem_info_t)); 
   
    /* initialize all locks and virtual address symbols prior to halAe_Init */
    SPIN_LOCK_INIT(myHandle->lock);
    initVirAddr(myHandle);

    /* initialize the chip settings */
    halAe_GetChipSetting(myHandle);

    /* disable clk gating if debug mode */
    halAe_DisableClkGating(myHandle);
    
    /* initialize HAL/UCLO */
    status = halAe_Init(myHandle, sysMemInfo->aeMask);
    if(status != HALAE_SUCCESS)
    {
        osalMemFree(myHandle->halHandle); 
        osalMemFree(myHandle); 
        return (cnvReturnCode(status));
    }
    
    myHandle->valid = 1;

    *handle = (void *)myHandle;
    
    return (status);    
}
                   
int 
icp_FirmLoader_Deinit(void *handle)
{
    icp_firml_handle_t *myHandle = (icp_firml_handle_t *)handle;

    if(myHandle == NULL)
    {
        return (ICP_FIRMLOADER_BADARG);
    }
    UcLo_DeleObj(myHandle);
    UcLo_DelSuof(myHandle);
    UcLo_DelMof (myHandle);
    halAe_DelLib(myHandle);

    return (0);
}    

int
icp_FirmLoader_MapObjAddr(void *handle, 
                          void *addrPtr, 
                          int memSize, 
                          int readOnly)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;
    
    status = UcLo_MapObjAddr(myHandle, addrPtr, memSize, readOnly);
    
    return (cnvReturnCode(status));
}                             

int
icp_FirmLoader_MapMofAddr(void *handle, 
                          void *filePtr, 
                          uint64 fileSize, 
                          char* uofName, 
                          void** uofPtr, 
                          unsigned int* uofSize)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;
    
    status = UcLo_MapMofAddr(myHandle, filePtr, fileSize, 
                             uofName, (char **)uofPtr, uofSize);

    return (cnvReturnCode(status));
}

int 
icp_FirmLoader_IsAeEnabled(void *handle, 
                           unsigned char ae)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;
    status = halAe_IsAeEnabled(myHandle, ae);
    
    return (cnvReturnCode(status));
}                             
                         
int 
icp_FirmLoader_VerifyUengine(void *handle, 
                             unsigned char ae)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;
    
    status = UcLo_VerifyUengine(myHandle, ae);
    
    return (cnvReturnCode(status));
}        

int
icp_FirmLoader_AuthenticateMMP (void *handle, 
                                void *filePtr, 
                                uint64 fileSize)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;

    status = UcLo_AuthenticateMMP (myHandle, filePtr, fileSize);
    
    return (cnvReturnCode(status));    
}

int 
icp_FirmLoader_BindSymbol(void *handle, 
                          char *ucodeImageName, 
                          char *ucodeSymName, 
                          uint64 value)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    
    myHandle = (icp_firml_handle_t *)handle;
    
    status = UcLo_BindSymbol(myHandle, ucodeImageName, ucodeSymName, value);
    
    return (cnvReturnCode(status));
}        
                                                  
int 
icp_FirmLoader_AeBindSymbol(void *handle, 
                            unsigned char ae, 
                            char *ucodeSymName, 
                            uint64 value)
{                           
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;

    status = UcLo_AeBindSymbol(myHandle, ae, ucodeSymName, value);
    
    return (cnvReturnCode(status));
}
                           
int
icp_FirmLoader_GetVarMemSegs(void *handle, 
                             icp_FirmLoader_VarMemSeg_T *varMemSeg)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;

    status = UcLo_GetVarMemSegs(myHandle, varMemSeg);
    
    return (cnvReturnCode(status));    
}
                            
int
icp_FirmLoader_SetMemoryStartOffset(void *handle, 
                                    unsigned int scratchOffset, 
                                    unsigned int sramOffset, 
                                    unsigned int dram0Offset, 
                                    unsigned int dram1Offset)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;

    status = UcLo_SetMemoryStartOffset(myHandle, 
                                       scratchOffset, 
                                       sramOffset,
                                       dram0Offset,
                                       dram1Offset);
    
    return (cnvReturnCode(status));        
}   
                                    
int
icp_FirmLoader_WriteUimageAll(void *handle)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;

    status = UcLo_WriteUimageAll(myHandle);
    
    return (cnvReturnCode(status));
}

int 
icp_FirmLoader_Start(void *handle,
                     unsigned char ae, 
                     unsigned int ctxEnMask)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;
    
    status = halAe_Start(myHandle, ae, ctxEnMask);
    
    return (cnvReturnCode(status));
}
                        
int
icp_FirmLoader_Stop(void *handle,
                    unsigned char ae, 
                    unsigned int ctxEnMask)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;
    
    status = halAe_Stop(myHandle, ae, ctxEnMask);
    
    return (cnvReturnCode(status));
}
                   
int 
icp_FirmLoader_Reset(void *handle,
                     unsigned int aeMask, 
                     int clrReg)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;
    
    halAe_Reset(myHandle, aeMask, clrReg);
    
    return (status);
}

int 
icp_FirmLoader_ClrReset(void *handle,
                        unsigned int aeMask)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;
        
    myHandle = (icp_firml_handle_t *)handle;
    
    status = halAe_ClrReset(myHandle, aeMask);
    
    return (cnvReturnCode(status));
}

char * 
icp_FirmLoader_GetAppMetaData(void *handle, 
                              unsigned int ae)
{
    icp_firml_handle_t *myHandle;
    char *str;
    myHandle = (icp_firml_handle_t *)handle;
    
    str = UcLo_GetAppMetaData(myHandle, ae);
    
    return (str);
}

int
icp_FirmLoader_GetUofVer(void * objBuf, 
                         unsigned int objBufSize,
                         icp_FirmLoader_UofVer_T * uofVer)
{
    int status = ICP_FIRMLOADER_SUCCESS;
    status = UcLo_GetUofVer(objBuf, objBufSize, uofVer);
    return (cnvReturnCode(status));
}

int
icp_FirmLoader_InitMemory(void *handle,
                               QAT_MEM_TYPE QatMemType)
{
    icp_firml_handle_t *myHandle;
    int status = ICP_FIRMLOADER_SUCCESS;    

    myHandle = (icp_firml_handle_t *)handle;
    
    status = halMem_InitMemory(myHandle, QatMemType);
    return (cnvReturnCode(status));
}

int
icp_FirmLoader_GetMmpVer(void *handle, 
                         void *objBuf,
                         unsigned int* mmpVer)
{
    int status = ICP_FIRMLOADER_SUCCESS;
    status = UcLo_GetMmpVer (handle, objBuf, mmpVer);
    return (cnvReturnCode(status));
}

static int 
cnvReturnCode(int retCode)
{
    switch(retCode)
    {
    case HALAE_SUCCESS:            return (ICP_FIRMLOADER_SUCCESS);
    case HALAE_FAIL:               return (ICP_FIRMLOADER_FAIL);
    case HALAE_BADARG:             return (ICP_FIRMLOADER_BADARG);
    case HALAE_DUPBKP:             return (ICP_FIRMLOADER_DUPBKP);
    case HALAE_BADADDR:            return (ICP_FIRMLOADER_BADADDR);
    case HALAE_BADLIB:             return (ICP_FIRMLOADER_BADLIB);
    case HALAE_NOUSTORE:           return (ICP_FIRMLOADER_NOUSTORE);
    case HALAE_DISABLED:           return (ICP_FIRMLOADER_DISABLED);
    case HALAE_ENABLED:            return (ICP_FIRMLOADER_ENABLED);
    case HALAE_AEACTIVE:           return (ICP_FIRMLOADER_AEACTIVE);
    case HALAE_NEIGHAEACTIVE:      return (ICP_FIRMLOADER_NEIGHAEACTIVE);
    case HALAE_RESET:              return (ICP_FIRMLOADER_RESET);
    case HALAE_TIMEOUT:            return (ICP_FIRMLOADER_TIMEOUT);
    case HALAE_ISSET:              return (ICP_FIRMLOADER_ISSET);
    case HALAE_NOTSET:             return (ICP_FIRMLOADER_NOTSET);
    case UCLO_FAILURE:             return (ICP_FIRMLOADER_FAIL);
    case UCLO_BADOBJ:              return (ICP_FIRMLOADER_BADOBJ);
    case UCLO_MEMFAIL:             return (ICP_FIRMLOADER_MEMFAIL);
    case UCLO_NOOBJ:               return (ICP_FIRMLOADER_NOOBJ);
    case UCLO_BADARG:              return (ICP_FIRMLOADER_BADARG);
    case UCLO_IMGNOTFND:           return (ICP_FIRMLOADER_IMGNOTFND);
    case UCLO_SYMNOTFND:           return (ICP_FIRMLOADER_SYMNOTFND);
    case UCLO_NEIGHNOTFND:         return (ICP_FIRMLOADER_NEIGHNOTFND);
    case UCLO_UOFINCOMPAT:         return (ICP_FIRMLOADER_UOFINCOMPAT);
    case UCLO_UOFVERINCOMPAT:      return (ICP_FIRMLOADER_UOFVERINCOMPAT);
    case UCLO_UNINITVAR:           return (ICP_FIRMLOADER_UNINITVAR);
    case UCLO_EXPRFAIL:            return (ICP_FIRMLOADER_EXPRFAIL);
    case UCLO_EXCDDRAMSIZE:        return (ICP_FIRMLOADER_EXCDDRAMSIZE);
    case UCLO_EXCDDRAM1SIZE:       return (ICP_FIRMLOADER_EXCDDRAM1SIZE);
    case UCLO_EXCDSRAMSIZE:        return (ICP_FIRMLOADER_EXCDSRAMSIZE);
    case UCLO_EXCDSCRTHSIZE:       return (ICP_FIRMLOADER_EXCDSCRTHSIZE);
    case UCLO_EXCDLMEMSIZE:        return (ICP_FIRMLOADER_EXCDLMEMSIZE);
    case UCLO_INVLDCTX:            return (ICP_FIRMLOADER_INVLDCTX);
    case UCLO_EXCDUMEMSIZE:        return (ICP_FIRMLOADER_EXCDUMEMSIZE);
    case UCLO_ADDRNOTFOUND:        return (ICP_FIRMLOADER_ADDRNOTFOUND);
    case UCLO_PAGENOTFOUND:        return (ICP_FIRMLOADER_PAGENOTFOUND);
    case UCLO_IVDWARN:             return (ICP_FIRMLOADER_IVDWARN);
    case UCLO_EXCDSHRAMSIZE:       return (ICP_FIRMLOADER_EXCDSHRAMSIZE);
    case HALMEM_DEVICE_ERROR:      return (ICP_FIRMLOADER_FAIL);
    case HALMEM_INVALID_PARAMETER: return (ICP_FIRMLOADER_BADARG);
    case HALMEM_TIMEOUT:           return (ICP_FIRMLOADER_TIMEOUT);
    case HALMEM_AEACTIVE:          return (ICP_FIRMLOADER_AEACTIVE);
    default:                       return (ICP_FIRMLOADER_FAIL);
    }
    ERRINFO(("retCode =0x%x\n", retCode)); return (ICP_FIRMLOADER_FAIL);
}

