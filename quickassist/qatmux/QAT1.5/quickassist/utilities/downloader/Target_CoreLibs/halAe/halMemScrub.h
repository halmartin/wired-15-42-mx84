/**
 **************************************************************************
 * @file halMemScrub.h
 *
 * @description
 *      This is the header file for memory scrub
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

#ifndef __HALMEMSCRUB_H
#define __HALMEMSCRUB_H

#include "halMem.h"
#include "halAe.h"

#define END_BITPOS 0x0
#define END_MASK (0xf << END_BITPOS)
#define START_BITPOS 0x4
#define START_MASK (0xf << START_BITPOS)
#define POWER_ON 0x1
#define POWER_OFF 0x0
#define PKE_PWRDOWN_BIT 3
#define SHRAM_PWRDOWN_BIT 0
#define ACCELCOMP_C_PKE_PWRDOWN_BIT(mmp) (3+mmp)
typedef enum
{
    ON_TO_ON   = (((POWER_ON << START_BITPOS) & START_MASK) | ((POWER_ON << END_BITPOS) & END_MASK)),  // CPM slice started out in power-on state, and return with power-on state
    OFF_TO_ON  = (((POWER_OFF << START_BITPOS) & START_MASK) | ((POWER_ON << END_BITPOS) & END_MASK)), // CPM slice started out in power-off state, and return with power-on state
    ON_TO_OFF  = (((POWER_ON << START_BITPOS) & START_MASK) | ((POWER_OFF << END_BITPOS) & END_MASK)), // CPM slice started out in power-on state, and return with power-off state
    OFF_TO_OFF = (((POWER_OFF << START_BITPOS) & START_MASK) | ((POWER_OFF << END_BITPOS) & END_MASK)) // CPM slice started out in power-off state, and return with power-off state
} qat_state_t;

typedef enum
{
    QAT_SHRAM_TYPE = 0,     // Shared Ram Type
    QAT_PKE_TYPE            // PKE Type
} qat_type_t;

#define HALMEM_ERROR(stat) stat

#define TWO_SETUP_MODE_NOP           (0x61806180)        /* setup(mode, nop) instruction encoding */
#define TWO_SETUP_MODE_IO            (0x60006000)        /* setup(mode, io) instruction encoding */

#define BITS_IN_LONGWORD   32
#define BYTES_IN_LONGWORD 4
#define BITS_IN_WORD  16
#define BYTE_LENGTH_OFFSET 6
#define QATNUM_LEFTSHIFT 2
#define ENDIAN_SWAP_OFFSET 30

#define HALMEM_EXECMICRO_LONGTIME 100000
#define RAM_CHUNK_INIT_SIZE (MAX_SHRAM_USED*sizeof(unsigned int))

unsigned int
halMem_EnableQat (
     icp_firml_handle_t *Handle,
     unsigned int       Qat,
     qat_type_t         QatType);

int
halMem_DisableQat (
   icp_firml_handle_t  *Handle,
   unsigned int        Qat,
   qat_type_t          QatType);

unsigned int 
 halMem_MmpReadCsr(
     icp_firml_handle_t *handle,
     unsigned int QatNum,
     unsigned int MmpNum,
     unsigned int CSR_offset);

void 
halMem_MmpWriteCsr(
    icp_firml_handle_t *handle,
    unsigned int QatNum,
    unsigned int MmpNum,
    unsigned int CSR_offset,
    unsigned int value);

unsigned int 
halMem_MmpReadMemory(
    const char* FunctionName,
    icp_firml_handle_t *handle,
    unsigned int QatNum, 
    unsigned int MmpNum, 
    unsigned int CSR_offset, 
    unsigned int Start,
    unsigned int Longwords,
    unsigned int* Buffer);

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
    unsigned int bInitFlag);

int 
halMem_MmpInitializeMemory(const char* FunctionName,
                                 icp_firml_handle_t *Handle,
                                 unsigned int QatNum,
                                 unsigned int MmpNum);

int halMem_GetSharedRam(
    icp_firml_handle_t *Handle, 
    unsigned int QatNum, 
    unsigned int ae,
    unsigned int ShramAddr,
    unsigned int Size, /* in LongWords, at most MAX_SHRAM_USED is allowed */
    unsigned int *Buffer,
    unsigned int EndianSwap);

int halMem_PutSharedRam(
    icp_firml_handle_t *Handle, 
    unsigned int QatNum, 
    unsigned int ae,
    unsigned int ShramAddr,
    unsigned int Size, /* in LongWords, at most MAX_SHRAM_USED is allowed */
    unsigned int *Buffer,
    unsigned int EndianSwap);

int 
halMem_ReadSharedRam(const char* FunctionName,
                             icp_firml_handle_t *Handle,
                             unsigned int Ae,  /* this idle AE will be used to perform the read */
                             unsigned int QatNum, 
                             unsigned int Start, 
                             unsigned int Longwords, 
                             unsigned int* Buffer);

int 
halMem_WriteSharedRam(const char* FunctionName,
                             icp_firml_handle_t *Handle,
                             unsigned int Ae,  /* this idle AE will be used to perform the read */
                             unsigned int QatNum, 
                             unsigned int Start, 
                             unsigned int Longwords, 
                             unsigned int* Buffer);

unsigned int
halMem_getMmpNum(icp_firml_handle_t *handle);

int
halMem_InitMemory(icp_firml_handle_t *handle,
                       QAT_MEM_TYPE QatMemType);

#endif  /* __HALMEMSCRUB_H */ 


