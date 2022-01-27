/**
 **************************************************************************
 * @file halAeChip.h
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

#ifndef __HALAECHIP_H
#define __HALAECHIP_H

#include "icptype.h"
#define ICP_AC_PESRAM_BAR         0  /* 512K eSram */
#define ICP_AC_PMISC_BAR          1  /* 16K CAP, 16K Scratch, 32K SSU(QATs), 
                                        32K AE CSRs and transfer registers, 8K CHAP/PMU, 
                                        4K EP CSRs, 4K MSI-X Tables */
#define ICP_AC_PETRINGCSR_BAR     2  /* 8K 16 bundles of ET Ring CSRs */

/* EP RI memory mapped registers */
#define EP_SMIAPF 0x28    
#define EP_SINTPF 0x24 
#define EP_SMIAPF1 0x30
#define EP_SINTPF1 0x2c
#define EP_ERRSOU3 0x0c
#define EP_ERRMSK3 0x1c
#define SINTPF_MISC_BITPOS 0x10000
#define SINTPF1_MISC_BITPOS 0x1
#define ERRSOU3_SHAC1_BITPOS 0x2
#define ERRMSK3_SHAC1_BITPOS 0x2

#define ACCELCOMP_C_FWCG_BIT     24

#define ACCELCOMP_B_PROD_BIT     31
#define ACCELCOMP_B_AUTH_EN_BIT  26
#define ACCELCOMP_B_FWCG_BIT     25
#define ACCELCOMP_B_PGPKE_BIT    24
#define ACCELCOMP_B_PGCOM_BIT    23

#define ACCELCOMP_R_SKU_BIT      2
#define ACCELCOMP_R_PKE_DIS_BIT  6

#define ACCELCOMP_C_SLICEPWRDOWN_CPMCLKGATE_BIT     23
#define ACCELCOMP_C_SKU_BIT      20

#define ACCELCOMP_B_SLICEPWRDOWN_CPMCLKGATE_BIT     30

/* eSram CSRs memory mapped registers
 * EP_OFFSET=0x1A00, eSram to RI OFFSET is 0x800 
 * TO use the same EP_OFFSET with RI, add 0x800 to each eSram CSRs offset */
#define ESRAM_DEBUGB 0x81C
#define ESRAM_DEBUGB_ACCELCOMP_C 0xC1C
#define DEBUGB_SRAMAUTO_TINIT_BITPOS 2
#define DEBUGB_SRAMAUTO_TINITDONE_BITPOS 3
#define DEBUGB_SRAMAUTO_TINIT (1<<DEBUGB_SRAMAUTO_TINIT_BITPOS)
#define DEBUGB_SRAMAUTO_TINITDONE (1<<DEBUGB_SRAMAUTO_TINITDONE_BITPOS)

#define ACCELCOMP_A_MAJOR_REV  0x0   /* Accel Complex A Stepping Major Revision ID */
#define ACCELCOMP_B_MAJOR_REV  0x1   /* Accel Complex B Stepping Major Revision ID */

#define ACCELCOMP_A0_RID  0x00    /* Accel Complex A0 Revision ID */
#define ACCELCOMP_B0_RID  0x10    /* Accel Complex B0 Revision ID */
#define ACCELCOMP_B1_RID  0x11    /* Accel Complex B1 Revision ID */
#define ACCELCOMP_C0_RID  0x20    /* Accel Complex C0 Revision ID */
#define ACCELCOMP_D3_RID  0x33    /* Accel Complex D3 Revision ID */

#define RST_CSR_QAT_LSB              20                 /* reset CSR QAT bit */

#define UBUF_SIZE                    (1024 * 16)         /* micro-store buffer size */

#define LOCAL_TO_XFER_REG_OFFSET   0x800
#define HASH_TO_GLOBAL_REG_OFFSET  0x900

#define ICP_CAP_SIZE KILO_16
#define ICP_SCRATCH_SIZE KILO_16
#define ICP_SSU_SIZE KILO_32
#define ICP_AE_SIZE  KILO_32
#define ICP_PMU_SIZE KILO_8
#define ICP_EP_SIZE  KILO_4
#define ICP_MSI_X_TAB_SIZE KILO_4 

#define CAP_OFFSET 0x0
#define SCRATCH_OFFSET (CAP_OFFSET + ICP_CAP_SIZE) /* 0x4000 */
#define SSU_OFFSET (SCRATCH_OFFSET + ICP_SCRATCH_SIZE) /* 0x8000 */
#define AE_OFFSET  (SSU_OFFSET + ICP_SSU_SIZE) /* 0x10000 */
#define PMU_OFFSET (AE_OFFSET + ICP_AE_SIZE) /* 0x18000 */
#define EP_OFFSET  (PMU_OFFSET + ICP_PMU_SIZE) /* 0x1a000 */
#define MSI_X_TAB_OFFSET (EP_OFFSET + ICP_EP_SIZE) /* 0x1b000 */

#define ICP_SSU_SIZE_ACCELCOMP_C KILO_128
#define ICP_AE_SIZE_ACCELCOMP_C  KILO_64

#define SSU_OFFSET_ACCELCOMP_C 0x0
#define AE_OFFSET_ACCELCOMP_C  (SSU_OFFSET_ACCELCOMP_C + ICP_SSU_SIZE_ACCELCOMP_C) /* 0x20000 */
#define CAP_OFFSET_ACCELCOMP_C (AE_OFFSET_ACCELCOMP_C + ICP_AE_SIZE_ACCELCOMP_C) /* 0x30000 */
#define SCRATCH_OFFSET_ACCELCOMP_C (CAP_OFFSET_ACCELCOMP_C + ICP_CAP_SIZE) /* 0x34000 */
#define PMU_OFFSET_ACCELCOMP_C (SCRATCH_OFFSET_ACCELCOMP_C + ICP_SCRATCH_SIZE) /* 0x38000 */
#define EP_OFFSET_ACCELCOMP_C  (PMU_OFFSET_ACCELCOMP_C + ICP_PMU_SIZE) /* 0x3a000 */
#define MSI_X_TAB_OFFSET_ACCELCOMP_C (EP_OFFSET_ACCELCOMP_C + ICP_EP_SIZE) /* 0x3b000 */

	
#define GET_OFFSET(deviceId, cap_offset, ae_offset, pmu_offset, scratch_offset, ssu_offset, ep_offset) { \
    if((deviceId == ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC) || \
       (deviceId == ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC) || \
       (deviceId == ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC)) \
    { \
        cap_offset = CAP_OFFSET_ACCELCOMP_C; \
        scratch_offset = SCRATCH_OFFSET_ACCELCOMP_C; \
        ssu_offset = SSU_OFFSET_ACCELCOMP_C; \
        ae_offset = AE_OFFSET_ACCELCOMP_C; \
        pmu_offset = PMU_OFFSET_ACCELCOMP_C; \
        ep_offset = EP_OFFSET_ACCELCOMP_C; \
    } \
    else \
    { \
        cap_offset = CAP_OFFSET; \
        scratch_offset = SCRATCH_OFFSET; \
        ssu_offset = SSU_OFFSET; \
        ae_offset = AE_OFFSET; \
        pmu_offset = PMU_OFFSET; \
        ep_offset = EP_OFFSET; \
    } \
  }

#define ICP_UWORD_MASK   0xbffffffffffull  /* micro-word mask without parity */

#define SRAMAUTO_INIT_TIMES 30
/* p_clk = 466 MHz (CRB)
 * p_clk = 10 MHz (FPGA)
 * All 4 SRAM are initialized in parallel and take 8200 p_clk 
 * (~18 usecs on CRB, 820 usecs on FPGA) 
 * ME speed is 2 times quicker than pclock
 */
#define SRAMAUTO_INIT_USECS (820*2)

/**
 * @description uword block
*/
typedef struct uof_encapuwblock_s{            
    unsigned int        startAddr;          /* start address */ 
    unsigned int        numWords;           /* number of micro-words */
    uint64              microWords;         /* pointer to packed microWords */
} uof_encapuwblock_t;

typedef uint64 import_var_type; 

#endif /* __HALAECHIP_H */
