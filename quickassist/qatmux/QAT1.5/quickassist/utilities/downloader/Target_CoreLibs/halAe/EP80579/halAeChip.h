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
#define ICP_AE_CLUSTER_CAP_BAR         0  /* 4K CAP & Conv. Bridge register */
#define ICP_AE_CLUSTER_SP_BAR          1  /* 16K Scratch memory */
#define ICP_AE_CLUSTER_AE_BAR          2  /* 16K AE CSRs and transfer registers */
#define ICP_AE_CLUSTER_SSU_BAR         3  /* 4K SSU CSRs */

#define EP80579_A_MAJOR_REV  0x0   /* EP80579 A Stepping Major Revision ID */
#define EP80579_B_MAJOR_REV  0x1   /* EP80579 B Stepping Major Revision ID */

#define EP80579_A0_RID  0x00       /* EP80579 A0 Revision ID */
#define EP80579_B0_RID  0x01       /* EP80579 B0 Revision ID */
#define EP80579_B1_RID  0x02       /* EP80579 B1 Revision ID */
#define EP80579_B2_RID  0x03       /* EP80579 B2 Revision ID */

#define RST_CSR_QAT_LSB              16                /* reset CSR QAT bit */
#define UBUF_SIZE                    (1024 * 8)         /* micro-store buffer size */

#define ICP_UWORD_MASK   0x3ffffffffffull  /* micro-word mask without parity */

#define LOCAL_TO_XFER_REG_OFFSET   0x800
#define HASH_TO_GLOBAL_REG_OFFSET  0x900

/**
 * @description uword block
*/
typedef struct uof_encapuwblock_s{            
    unsigned int        startAddr;          /* start address */ 
    unsigned int        numWords;           /* number of micro-words */
    unsigned int        microWords;         /* pointer to packed microWords */
} uof_encapuwblock_t;

typedef unsigned int import_var_type;

#endif /* __HALAECHIP_H */
