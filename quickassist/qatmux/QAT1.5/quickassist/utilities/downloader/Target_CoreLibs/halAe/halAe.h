/**
 **************************************************************************
 * @file halAe.h
 *
 * @description
 *      This is the header file for Ucode AE Library
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

#ifndef __HALAE_H
#define __HALAE_H

#include "halAeChip.h"

#define MAX_RETRY_TIMES 10000

/* cycles from CTX_ENABLE high to CTX entering executing state */
#define NUM_CYCLES_FROM_READY2EXE 8

#ifndef CHAR_BIT
#define CHAR_BIT 8
#endif

#ifndef WORD_BIT
#define WORD_BIT ((unsigned int)(CHAR_BIT * sizeof(unsigned int)))
#endif

#define MASK (WORD_BIT-1)
#define CLR_BIT(wrd, bit) ((wrd) & ~(1 << ((bit) & MASK)))
#define SET_BIT(wrd, bit) ((wrd) | (1 << ((bit) & MASK)))

#define FOUR_CTX                     4                 /* Four context ID */
#define EIGHT_CTX                    8                 /* Eight context ID */
#define INVALID_PC                   0xffffffff        /* Invalid PC */
#define INVALID_AE                   0xffffffff        /* Invalid AE */
#define CSR_DONE_COUNT               500               /* loop count for reading CSR */        

#define INIT_CTX_ARB_VALUE           0x0               /* context arbitor initialization value */   
#define INIT_CTX_ENABLE_VALUE        0x0               /* context enable initialization value */   
#define INIT_PC_VALUE                0x0               /* program counter initialization value */   
#define INIT_WAKEUP_EVENTS_VALUE     XCWE_VOLUNTARY    /* wakeup events initialization value */   
#define INIT_SIG_EVENTS_VALUE        0x1               /* signal events initialization value */   
#define INIT_CCENABLE_VALUE          0x2000            /* context enable initialization value */   
#define WAKEUP_EVENTS_SLEEP          0x10000           /* wakeup enents sleep initialization value */    

#define SET_RESET_CSR_QAT(regVal, qatMask) ((regVal) |= ((qatMask) << RST_CSR_QAT_LSB))  /* set reset csr QAT bits */   
#define CLR_RESET_CSR_QAT(regVal, qatMask) ((regVal) &= ~((qatMask) << RST_CSR_QAT_LSB))  /* clear reset csr QAT bits */   

#define RST_CSR_AE_LSB               0                 /* reset CSR AE bit */     
#define SET_RESET_CSR_AE(regVal, aeMask) ((regVal) |= ((aeMask) << RST_CSR_AE_LSB))  /* set reset csr AE bits */   
#define CLR_RESET_CSR_AE(regVal, aeMask) ((regVal) &= ~((aeMask) << RST_CSR_AE_LSB))  /* clear reset csr AE bits */   

#define CLK_ENABLE_CSR_AE_LSB               0                 /* clock enable CSR AE bit */     
#define SET_CLK_ENABLE_CSR_AE(regVal, aeMask) ((regVal) |= ((aeMask) << CLK_ENABLE_CSR_AE_LSB))  /* set clock enable csr AE bits */   
#define CLR_CLK_ENABLE_CSR_AE(regVal, aeMask) ((regVal) &= ~((aeMask) << CLK_ENABLE_CSR_AE_LSB))  /* clear clock enable csr AE bits */   

#define CLK_ENABLE_CSR_QAT_LSB               20                 /* clock enable CSR QAT bit */     
#define SET_CLK_ENABLE_CSR_QAT(regVal, qatMask) ((regVal) |= ((qatMask) << CLK_ENABLE_CSR_QAT_LSB))  /* set clock enable csr QAT bits */   
#define CLR_CLK_ENABLE_CSR_QAT(regVal, qatMask) ((regVal) &= ~((qatMask) << CLK_ENABLE_CSR_QAT_LSB))  /* set clock enable csr QAT bits */   

#define BAD_REGADDR                  0xffff             /* bad register address */

#define IGNORE_BKPT_MASK  (~(1<<CE_BREAKPOINT_BITPOS))              /* ignore breakpoint mask */  
#define IGNORE_ECCER_MASK (~(1<<CE_CNTL_STORE_PARITY_ERROR_BITPOS)) /* ignore ecc error mask */  
#define IGNORE_PARER_MASK (~(1<<CE_REG_PAR_ERR_BITPOS))             /* ignore parity error mask */  
#define IGNORE_W1C_MASK (IGNORE_BKPT_MASK & IGNORE_ECCER_MASK & \
                         IGNORE_PARER_MASK)                         /* ignore write-1-clear mask */         
#define IGNORE_W1C_ERR_MASK (IGNORE_ECCER_MASK & IGNORE_PARER_MASK) /* ignore write-1-clear mask for error bits */         

#define SETBIT(value, bit) (setMasks[bit] | (value))                /* set  bit */  
#define CLRBIT(value, bit) (clrMasks[bit] & (value))                /* clear bit */   
  
#define INSERT_IMMED_GPRA_CONST(inst, const_val) \
    inst = (inst               & 0xFFFF00C03FFull) | \
           ((((const_val) << 12) & 0x0FF00000ull) | \
            (((const_val) << 10) & 0x0003FC00ull))                         /* patch value to immed[]' GPRA field */      
#define INSERT_IMMED_GPRB_CONST(inst, const_val) \
    inst = (inst               & 0xFFFF00FFF00ull) | \
           ((((const_val) << 12) & 0x0FF00000ull) | \
            (((const_val) <<  0) & 0x000000FFull))                         /* patch value to immed[]' GPRB field */

/* patch value to ssu_transfer[] opt_token field indicating target ssu
 * bit37-32 tgt_cmd: 100_xxx for ssua, 101_xxx for ssub, so bit35 indicating ssua/b
 * bit19-18  Tokens: 0 for ssux_0_1, 1 for ssux_2_3, so bit18 indicating 
 */
#define INSERT_SSU_TRANSFER_OPTTOKEN_CONST(inst, tgtcmd, token) \
    inst = (inst & 0xFF7FFFBFFFFull) | \
           (((tgtcmd << 35) & 0x800000000ull) | \
            ((token << 18) & 0x40000ull))                                  /* patch value to ssu_transfer[]' opt_token field */

#define TWO_ZEROS {0,0}

#endif /* __HAL_AE_H */
