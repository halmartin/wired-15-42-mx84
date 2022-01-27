/**
 **************************************************************************
 * @file halAeLinux.hxx
 *
 * @description
 *      This file provides implementation of Ucode AE Library 
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

#ifndef __HALAELINUX_HXX
#define __HALAELINUX_HXX

#include "halAeApi.h"
#include "halAeDrv.h"
#include "icp_firml_handle.h"

#define AEDRV_IOC_MAGIC 'x'
typedef struct aeDrv_HalAeIntr_S {
    unsigned long       instanceId;    /**< instance id */
    unsigned long       IntrType;      /**< Interrupt type */
    Hal_IntrMasks_T     IntrMasks;     /**< Interrupt masks */
} aeDrv_HalAeIntr_T;

typedef struct aeDrv_HalMemoryOffset_S {
    unsigned long       instanceId;    /**< instance id */
    unsigned long       scratchOffset; /**< Scratch offset */
    unsigned long       sramOffset;    /**< Sram offset */
    unsigned long       dram0Offset;   /**< dram region 0 offset */ 
    unsigned long       dram1Offset;   /**< dram region 1 offset  */
} aeDrv_HalMemoryOffset_T;

typedef struct aeDrv_HalPciCsr_S {
    unsigned long       instanceId;    /**< instance id */
    unsigned long       csrOffset;
    unsigned long       numBytes;    
    unsigned long       data;
} aeDrv_HalPciCsr_T;

typedef struct aeDrv_HalDbgDram_S {
    unsigned long       instanceId;    /**< instance id */
    icp_firml_dram_desc_t dbgDramDesc;
} aeDrv_HalDbgDram_T;

typedef struct aeDrv_HalFreeUstore_S {
    unsigned long       instanceId;    /**< instance id */
    unsigned long       ae;            /**< ae number */
    unsigned long       freeAddr;      /**< free micro-store address */
    unsigned long       freeSize;      /**< free micro-store size */
} aeDrv_HalFreeUstore_T;

/** @{ */
/*
 * IOCTL codes for interface to the accelEngine driver.
 */
#define AEDRV_SYSMEMINFO        _IOWR(AEDRV_IOC_MAGIC, 1, aedrv_sysmeminfo_t)
#define AEDRV_INTRPOLL          _IOWR(AEDRV_IOC_MAGIC, 2, aeDrv_HalAeIntr_T)
#define AEDRV_INTRENAB          _IOWR(AEDRV_IOC_MAGIC, 3, aeDrv_HalAeIntr_T)
#define AEDRV_INTRDISAB         _IOWR(AEDRV_IOC_MAGIC, 4, aeDrv_HalAeIntr_T)
#define AEDRV_SET_MEMORYOFFSET  _IOWR(AEDRV_IOC_MAGIC, 5, aeDrv_HalMemoryOffset_T)
#define AEDRV_GET_MEMORYOFFSET  _IOWR(AEDRV_IOC_MAGIC, 6, aeDrv_HalMemoryOffset_T)
#define AEDRV_INTR_INIT         _IOWR(AEDRV_IOC_MAGIC, 7, unsigned int)
#define AEDRV_INTR_CLEANUP      _IOWR(AEDRV_IOC_MAGIC, 8, unsigned int)
#define AEDRV_GETPCICSR         _IOWR(AEDRV_IOC_MAGIC, 9, aeDrv_HalPciCsr_T)
#define AEDRV_PUTPCICSR         _IOWR(AEDRV_IOC_MAGIC, 10, aeDrv_HalPciCsr_T)
#define AEDRV_ALLOCDBGDRAM      _IOWR(AEDRV_IOC_MAGIC, 11, aeDrv_HalDbgDram_T)
#define AEDRV_FREEDBGDRAM       _IOWR(AEDRV_IOC_MAGIC, 12, aeDrv_HalDbgDram_T)
#define AEDRV_SET_FREEUSTORE    _IOWR(AEDRV_IOC_MAGIC, 13, aeDrv_HalFreeUstore_T)
#define AEDRV_GET_FREEUSTORE    _IOWR(AEDRV_IOC_MAGIC, 14, aeDrv_HalFreeUstore_T)
#define AEDRV_DEVICES                   _IOWR(AEDRV_IOC_MAGIC, 15, vadf_devinfo_t)

/** @} */

#endif          /* __HALAELINUX_HXX leave next blank line */
