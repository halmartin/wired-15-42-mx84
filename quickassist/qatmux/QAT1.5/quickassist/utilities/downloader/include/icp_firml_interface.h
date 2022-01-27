/**
 **************************************************************************
 * @file icp_firml_interface.h
 *
 * @description
 *      This is the header file for AE Loader
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

#ifndef __ICP_FIRML_INTERFACE_H
#define __ICP_FIRML_INTERFACE_H

#include "icp_firml_handle.h"
#include "uclo.h"
#include "halMem.h"

#define ICP_FIRMLOADER_BADAE MAX_AE     /* bad AccelEngine */

typedef uclo_varmemseg_t icp_FirmLoader_VarMemSeg_T;
typedef uof_ver_t icp_FirmLoader_UofVer_T;

enum{
   ICP_FIRMLOADER_SUCCESS=0,     /**< the operation was successful */
   ICP_FIRMLOADER_FAIL=0xA100,   /**< the operation failed */
   ICP_FIRMLOADER_INVALIDHANDLE, /**< invalid handle */
   ICP_FIRMLOADER_BADARG,        /**< bad function argument */
   ICP_FIRMLOADER_DUPBKP,        /**< duplicate break point */
   ICP_FIRMLOADER_NOUSTORE,      /**< not ustore available */
   ICP_FIRMLOADER_BADADDR,       /**< bad address */
   ICP_FIRMLOADER_BADLIB,        /**< bad debug library -- wasn't initialized */
   ICP_FIRMLOADER_DISABLED,      /**< acceleration engine or interrupt disabled */
   ICP_FIRMLOADER_ENABLED,       /**< acceleration engine enabled */
   ICP_FIRMLOADER_RESET,         /**< acceleration engine is in reset */
   ICP_FIRMLOADER_TIMEOUT,       /**< the operation exceed th etime limit */
   ICP_FIRMLOADER_ISSET,         /**< condition/evaluation is set/true */
   ICP_FIRMLOADER_NOTSET,        /**< condition/evaluation is not set/false */
   ICP_FIRMLOADER_AEACTIVE,      /**< ae is running */
   ICP_FIRMLOADER_MEMALLOC,      /**< memory allocation error */
   ICP_FIRMLOADER_NEIGHAEACTIVE, /**< neighbour ae is running */
   ICP_FIRMLOADER_BADOBJ,		/**< bad object */
   ICP_FIRMLOADER_MEMFAIL,		/**< memory access fail */
   ICP_FIRMLOADER_NOOBJ,			/**< no object */
   ICP_FIRMLOADER_IMGNOTFND,		/**< image not found */
   ICP_FIRMLOADER_SYMNOTFND,		/**< symbol not found */
   ICP_FIRMLOADER_NEIGHNOTFND,   /**< neighbour AE not found */
   ICP_FIRMLOADER_UOFINCOMPAT,   /**< UOF is incompatible with the chip type/revision */
   ICP_FIRMLOADER_UOFVERINCOMPAT,/**< UOF version incompatible with the loader -- mismatched uof format */
   ICP_FIRMLOADER_UNINITVAR,     /**< uninitialized import variable */
   ICP_FIRMLOADER_EXPRFAIL,      /**< expression fail */
   ICP_FIRMLOADER_EXCDDRAMSIZE,  /**< address exceed dram0 size */
   ICP_FIRMLOADER_EXCDDRAM1SIZE, /**< address exceed dram1 size */
   ICP_FIRMLOADER_EXCDSRAMSIZE, /**< address exceed sram size */
   ICP_FIRMLOADER_EXCDSCRTHSIZE, /**< address exceed scratch size */
   ICP_FIRMLOADER_EXCDLMEMSIZE,  /**< address exceed local memory size */
   ICP_FIRMLOADER_INVLDCTX,      /**< invalid context */
   ICP_FIRMLOADER_EXCDUMEMSIZE,  /**< address exceed ustore memory size */
   ICP_FIRMLOADER_ADDRNOTFOUND,  /**< address not found */
   ICP_FIRMLOADER_PAGENOTFOUND,  /**< page not found */
   ICP_FIRMLOADER_IVDWARN,       /**< unknown image or symbol defined */
   ICP_FIRMLOADER_EXCDSHRAMSIZE  /**< address exceed shared ram size */
};

#ifdef __cplusplus
extern "C" {
#endif

int 
icp_FirmLoader_Init(icp_firml_sys_mem_info_t *sysMemInfo, 
                    void **handle);
int 
icp_FirmLoader_Deinit(void *handle);

int 
icp_FirmLoader_ISR(void *handle);

int 
icp_FirmLoader_MapObjAddr(void *handle, 
                          void *addrPtr, 
                          int memSize, 
                          int readOnly);
int
icp_FirmLoader_MapMofAddr(void *handle, 
                          void *filePtr, 
                          uint64 fileSize, 
                          char* uofName, 
                          void** uofPtr, 
                          unsigned int* uofSize);
int 
icp_FirmLoader_IsAeEnabled(void *handle, 
                           unsigned char ae);
int 
icp_FirmLoader_VerifyUengine(void *handle, 
                             unsigned char ae);
int 
icp_FirmLoader_BindSymbol(void *handle, 
                          char *ucodeImageName, 
                          char *ucodeSymName, 
                          uint64 value);
int 
icp_FirmLoader_AeBindSymbol(void *handle, 
                            unsigned char ae, 
                            char *ucodeSymName, 
                            uint64 value);
int 
icp_FirmLoader_GetVarMemSegs(void *handle, 
                             icp_FirmLoader_VarMemSeg_T *varMemSeg);
int
icp_FirmLoader_AuthenticateMMP (void *handle, 
                                void *filePtr, 
                                uint64 fileSize);
int 
icp_FirmLoader_SetMemoryStartOffset(void *handle, 
                                    unsigned int scratchOffset, 
                                    unsigned int sramOffset, 
                                    unsigned int dram0Offset, 
                                    unsigned int dram1Offset);
int 
icp_FirmLoader_WriteUimageAll(void *handle);

int 
icp_FirmLoader_Start(void *handle,
                     unsigned char ae, 
                     unsigned int ctxEnMask);
int 
icp_FirmLoader_Stop(void *handle,
                    unsigned char ae, 
                    unsigned int ctxEnMask);
int 
icp_FirmLoader_Reset(void *handle,
                     unsigned int aeMask, 
	                 int clrReg);
int 
icp_FirmLoader_ClrReset(void *handle,
                        unsigned int aeMask);

char * 
icp_FirmLoader_GetAppMetaData(void *handle, 
                              unsigned int ae);
int
icp_FirmLoader_GetUofVer(void * objBuf, 
                         unsigned int objBufSize,
                         icp_FirmLoader_UofVer_T * uofVer);
int
icp_FirmLoader_InitMemory(void *handle,
                          QAT_MEM_TYPE QatMemType);
int
icp_FirmLoader_GetMmpVer(void *handle, 
                         void *objBuf,
                         unsigned int* mmpVer);

#ifdef __cplusplus
}
#endif

#endif /* __ICP_FIRML_INTERFACE_H */
