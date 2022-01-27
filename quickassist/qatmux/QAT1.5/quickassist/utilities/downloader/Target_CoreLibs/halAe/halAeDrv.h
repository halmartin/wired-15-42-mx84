/**
 **************************************************************************
 * @file halAeDrv.h
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

#ifndef __HALAEDRV_HXX
#define __HALAEDRV_HXX

#include "halAe_platform.h"
#include "halAeChip.h"
#include "halAeApi.h"

#define QAT_NB_IA_EVENTMSK_CMP 0x0038
#define QAT_NB_IA_EVENTMSK_RE  0x0040
#define QAT_NB_IA_EVENTMSK_XLT 0x0048
#define QAT_DMRISSUMMP 0x03DC
#define QAT_DOARDSSUMMP 0x03E0

#define CMP_MASK_NOTIFY_JOBDONE   0x30000
#define REGEX_MASK_NOTIFY_JOBDONE   0x30000
#define XLT_MASK_NOTIFY_JOBDONE   0x30000

#define AC_FUSECTL 0x40
#define AC_LEGFUSES 0x4c
#define TEST(mask, bitpos) (((mask) & (1<<bitpos)) >> (bitpos))

#define SADD(va, offset) (((va) != 0) ? ((va)+(offset)) : 0)		/* add offset to va, unless va is zero, in that case, return 0 */

/** @{ */
/* memory size definition */
#define KILO_1    (0x400)              /* 1K bytes size */
#define KILO_2    (KILO_1 << 1)        /* 2K bytes size */
#define KILO_4    (KILO_1 << 2)        /* 4K bytes size */
#define KILO_6    (KILO_1 * 6)         /* 6K bytes size */
#define KILO_8    (KILO_1 << 3)        /* 8K bytes size */
#define KILO_16   (KILO_1 << 4)        /* 16K bytes size */
#define KILO_32   (KILO_1 << 5)        /* 32K bytes size */
#define KILO_64   (KILO_1 << 6)        /* 64K bytes size */
#define KILO_128  (KILO_1 << 7)        /* 128K bytes size */
#define KILO_256  (KILO_1 << 8)        /* 256K bytes size */
#define KILO_512  (KILO_1 << 9)        /* 512K bytes size */

#define MEG_1     (0x100000)           /* 1M bytes size */
#define MEG_2     (MEG_1 << 1)         /* 2M bytes size */
#define MEG_4     (MEG_1 << 2)         /* 4M bytes size */
#define MEG_8     (MEG_1 << 3)         /* 8M bytes size */
#define MEG_16    (MEG_1 << 4)         /* 16M bytes size */
#define MEG_32    (MEG_1 << 5)         /* 32M bytes size */
#define MEG_64    (MEG_1 << 6)         /* 64M bytes size */
#define MEG_128   (MEG_1 << 7)         /* 128M bytes size */
#define MEG_256   (MEG_1 << 8)         /* 256M bytes size */
#define MEG_192   (0x0c000000)         /* 192M bytes size */

/** @} */

#define VALUE_BIT_0 1  /* The number of values represented by bit-0 */
#define VALUE_BIT_1 2  /* The number of values represented by bit-1 */
#define VALUE_BIT_2 4  /* The number of values represented by bit-2 */
#define VALUE_BIT_3 8  /* The number of values represented by bit-3 */
#define VALUE_BIT_4 16 /* The number of values represented by bit-4 */
#define VALUE_BIT_5 32 /* The number of values represented by bit-5 */

#define UWORD_ECC_BIT_0 44
#define UWORD_ECC_BIT_1 45
#define UWORD_ECC_BIT_2 46
#define UWORD_ECC_BIT_3 47
#define UWORD_ECC_BIT_4 48
#define UWORD_ECC_BIT_5 49
#define UWORD_ECC_BIT_6 50

#define DWORD_SIZE_IN_BIT    (sizeof(unsigned int) * 8)   /* the size of dword is 32 bit */
#define WORD_SIZE_IN_BIT     (sizeof(unsigned short) * 8) /* the size of word is 16 bit */

#define BIT_28 28
#define BIT_27 27
#define BIT_20 20
#define BIT_18 18
#define BIT_16 16
#define BIT_12 12
#define BIT_10 10
#define BIT_9  9
#define BIT_8  8
#define BIT_5  5
#define BIT_4  4
#define BIT_3  3
#define BIT_2  2

/** @{ */
/* CAP CSR offset definition */
#define CAP_IA_INTERRUPT_A_OFFSET      0xB20                     /* IA interrupt A csr offset */
#define CAP_IA_INTERRUPT_B_OFFSET      0xB24                     /* IA interrupt A csr offset */
#define CAP_THD_INTERRUPT_A_OFFSET     0xB30                     /* thread interrupt A csr offset */ 
#define CAP_THD_INTERRUPT_B_OFFSET     0xB50                     /* thread interrupt B csr offset */   
#define CAP_THD_ENABLE_A_OFFSET        0xB70                     /* thread interrupt A enable csr offset */ 
#define CAP_THD_ENABLE_B_OFFSET        0xB90                     /* thread interrupt B enable csr offset */ 
#define CAP_THD_ENABLE_SET_A_OFFSET    CAP_THD_ENABLE_A_OFFSET   /* thread interrupt A enable set csr offset */ 
#define CAP_THD_ENABLE_SET_B_OFFSET    CAP_THD_ENABLE_B_OFFSET   /* thread interrupt B enable set csr offset */ 
#define CAP_THD_ENABLE_CLR_A_OFFSET    0xBB0                     /* thread interrupt A enable clear csr offset */ 
#define CAP_THD_ENABLE_CLR_B_OFFSET    0xBD0                     /* thread interrupt B enable set csr offset */ 
#define CAP_RAW_ATTN_STATUS            0xA88                     /* raw attention interrupt csr offset */ 
#define CAP_ATTN_MASK                  0xA8C                     /* attention mask csr offset */ 
#define CAP_ATTN_MASK_SET              CAP_ATTN_MASK             /* attention mask set csr offset */ 
#define CAP_ATTN_MASK_CLR              0xA90                     /* attention mask clear csr offset */ 
/** @} */

/** @{ */
/* Define interrupt vector ID */
#define IRQ_AE_ATTN   2     /* AE attention interrupt ID */   
#define MAX_AE_INT    0x10  /* maximum interrupt ID */   
/** @} */

#define AE_ATTN_ALLBITS            0xFFFFFFFF		/* All AE' attention interrupt bits */

/** @{ */
/* Define parity error bit position */
#define CE_BREAKPOINT_BITPOS              27    /* breakpoint bit position */   
#define CE_CNTL_STORE_PARITY_ERROR_BITPOS 29    /* control store parrity error bit position */   
#define CE_BREAKPOINT_BIT              (1 << CE_BREAKPOINT_BITPOS)   /* breakpoint bit */      
#define CE_CNTL_STORE_PARITY_ERROR_BIT (1 << CE_CNTL_STORE_PARITY_ERROR_BITPOS)  /* control store parrity error bit */   
/** @} */

#define HWID_ICP           0x80      /* ICP hardware ID  */
#define HWID_ACCEL_COMP    0x81      /* acceleration complex hardware ID  */
#define HWID_ACCEL_COMP_C  0x82      /* acceleration complex C hardware ID */
#define HWID_ACCEL_COMP_R  0x83      /* NanoQAT hardware ID */
#define HWID_ACCEL_COMP_B  0x84      /* acceleration complex B hardware ID */
#define HWID_ACCEL_COMP_RS 0x85      /* acceleration complex RS hardware ID */


#define HAL_EXECMICROINST_TIME  600

#define HALAE_VERIFY_LIB(handle) if(!handle->halHandle->HalAe_LibInit) { return (HALAE_BADLIB); } /* verify HAL library */

#define HALAE_CLUSTR(ae) (((ae) & 0x10) >> 4)           /* cluster number of AE */  
#define HALAE_NUM(handle, ae) ((ae) & handle->halHandle->AeMask)                   /* AE number within cluster */    
#define AE(handle, ae) handle->halHandle->AEs[HALAE_CLUSTR(ae)][HALAE_NUM(handle, ae)]     /* AE control */  

#define BITS_IN_LONGWORD             32                /* bits number in a word */        

#define VERIFY_ARGPTR(arg) if((arg) == NULL) { ERRINFO(("Null pointer\n")); return (HALAE_BADARG); }    /* verify argument pointer */
#define VERIFY_AE(handle, ae) if(((ae) > handle->halHandle->AeMaxNum) || ((ae) >= BITS_IN_LONGWORD) || \
                          (handle->halHandle->AeBadMask & setMasks[(ae)]) || \
                          (AE(handle, ae).state == HALAE_UNINIT)) \
                          return (HALAE_BADARG)                            /* verify AE */
#define VERIFY_CTX(ctx) if(ctx >= MAX_CTX) { ERRINFO(("Invalid ctx number=0x%x",ctx)); return (HALAE_BADARG); }      /* verify context */
 
#define SPIN_LOCK_AE(handle, ae) SPIN_LOCK(AE(handle, ae).aeLock)                          /* lock AE resource */
#define SPIN_UNLOCK_AE(handle, ae) SPIN_UNLOCK(AE(handle, ae).aeLock)                      /* unlock AE resource */
    
#define GET_LIVECTXMASK(handle, ae) AE(handle, ae).liveCtxMask                           /* get live context mask */
#define SET_LIVECTXMASK(handle, ae, ctxMask) AE(handle, ae).liveCtxMask = ctxMask        /* set live context mask */

#define IS_RST(handle, ae) (AE(handle, ae).state == HALAE_RST)                           /* check if AE in reset status */
#define SET_RST(handle, ae) AE(handle, ae).state = HALAE_RST                             /* set AE in reset status */
#define CLR_RST(handle, ae) AE(handle, ae).state = HALAE_CLR_RST                         /* clear AE in reset status */
#define GET_USTORE_SIZE(handle, ae) AE(handle, ae).uStoreSize                            /* get micro-store size */

#define AE_NUM(index) index

/**
 * @brief system memory info 
 * @note system memory info 
*/
typedef struct aedrv_sysmeminfo_s {
    unsigned long       instanceId;
    icp_firml_sys_mem_info_t sysMemInfo;
} aedrv_sysmeminfo_t;

static const unsigned int clrMasks[BITS_IN_LONGWORD] ={
    0xfffffffe, 0xfffffffd, 0xfffffffb, 0xfffffff7,
    0xffffffef, 0xffffffdf, 0xffffffbf, 0xffffff7f,
    0xfffffeff, 0xfffffdff, 0xfffffbff, 0xfffff7ff,
    0xffffefff, 0xffffdfff, 0xffffbfff, 0xffff7fff,
    0xfffeffff, 0xfffdffff, 0xfffbffff, 0xfff7ffff,
    0xffefffff, 0xffdfffff, 0xffbfffff, 0xff7fffff,
    0xfeffffff, 0xfdffffff, 0xfbffffff, 0xf7ffffff,
    0xefffffff, 0xdfffffff, 0xbfffffff, 0x7fffffff};

static const unsigned int setMasks[BITS_IN_LONGWORD] ={
    0x00000001, 0x00000002, 0x00000004, 0x00000008,
    0x00000010, 0x00000020, 0x00000040, 0x00000080,
    0x00000100, 0x00000200, 0x00000400, 0x00000800,
    0x00001000, 0x00002000, 0x00004000, 0x00008000,
    0x00010000, 0x00020000, 0x00040000, 0x00080000,
    0x00100000, 0x00200000, 0x00400000, 0x00800000,
    0x01000000, 0x02000000, 0x04000000, 0x08000000,
    0x10000000, 0x20000000, 0x40000000, 0x80000000};



struct PageData_S;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif            /* __HALAEDRV_HXX leave next blank line */
