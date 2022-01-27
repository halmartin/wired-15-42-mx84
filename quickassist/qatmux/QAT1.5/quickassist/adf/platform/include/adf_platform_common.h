/*****************************************************************************
 *
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
 *****************************************************************************/

/*****************************************************************************
 * @file adf_platform_common.h
 *
 * @description
 *      This file contains the macros common to supported platform(s).
 *
 *****************************************************************************/
#ifndef ADF_PLATFORM_COMMON_H
#define ADF_PLATFORM_COMMON_H

/* Number of Rings Per Bank */
#define ICP_ETR_MAX_RINGS_PER_BANK          16

/* Ring size values */
#define ICP_RINGSIZE_64                     0x00
#define ICP_RINGSIZE_128                    0x01
#define ICP_RINGSIZE_256                    0x02
#define ICP_RINGSIZE_512                    0x03
#define ICP_RINGSIZE_KILO_1                 0x04
#define ICP_RINGSIZE_KILO_2                 0x05
#define ICP_RINGSIZE_KILO_4                 0x06
#define ICP_RINGSIZE_KILO_8                 0x07
#define ICP_RINGSIZE_KILO_16                0x08
#define ICP_RINGSIZE_KILO_32                0x09
#define ICP_RINGSIZE_KILO_64                0x0A
#define ICP_RINGSIZE_KILO_128               0x0B
#define ICP_RINGSIZE_KILO_256               0x0C
#define ICP_RINGSIZE_KILO_512               0x0D
#define ICP_RINGSIZE_MEG_1                  0x0E
#define ICP_RINGSIZE_MEG_2                  0x0F
#define ICP_RINGSIZE_MEG_4                  0x10

/* Ring Near Full/Empty watermarks values */
#define ICP_RING_NEAR_WATERMARK_0           0x00
#define ICP_RING_NEAR_WATERMARK_4           0x01
#define ICP_RING_NEAR_WATERMARK_8           0x02
#define ICP_RING_NEAR_WATERMARK_16          0x03
#define ICP_RING_NEAR_WATERMARK_32          0x04
#define ICP_RING_NEAR_WATERMARK_64          0x05
#define ICP_RING_NEAR_WATERMARK_128         0x06
#define ICP_RING_NEAR_WATERMARK_256         0x07
#define ICP_RING_NEAR_WATERMARK_512         0x08
#define ICP_RING_NEAR_WATERMARK_KILO_1      0x09
#define ICP_RING_NEAR_WATERMARK_KILO_2      0x0A
#define ICP_RING_NEAR_WATERMARK_KILO_4      0x0B
#define ICP_RING_NEAR_WATERMARK_KILO_8      0x0C
#define ICP_RING_NEAR_WATERMARK_KILO_16     0x0D
#define ICP_RING_NEAR_WATERMARK_KILO_32     0x0E
#define ICP_RING_NEAR_WATERMARK_KILO_64     0x0F
#define ICP_RING_NEAR_WATERMARK_KILO_128    0x10
#define ICP_RING_NEAR_WATERMARK_KILO_256    0x11
#define ICP_RING_NEAR_WATERMARK_KILO_512    0x12
#define ICP_RING_NEAR_WATERMARK_MEG_1       0x13
#define ICP_RING_NEAR_WATERMARK_MEG_2       0x14
#define ICP_RING_NEAR_WATERMARK_MEG_4       0x15


/* Possible nearly empty/full threshold values.
 * Note: this values are for 16KBytes ring size.
 * If the ring size will be changed these values
 * need to be changed as well.
 */
#define NEAR_EMPTY_WATERMARK_VAL_0    ICP_RING_NEAR_WATERMARK_128
#define NEAR_EMPTY_WATERMARK_VAL_1    ICP_RING_NEAR_WATERMARK_256
#define NEAR_EMPTY_WATERMARK_VAL_2    ICP_RING_NEAR_WATERMARK_512
#define NEAR_EMPTY_WATERMARK_VAL_3    ICP_RING_NEAR_WATERMARK_KILO_1
#define NEAR_EMPTY_WATERMARK_VAL_4    ICP_RING_NEAR_WATERMARK_KILO_2
#define NEAR_EMPTY_WATERMARK_VAL_5    ICP_RING_NEAR_WATERMARK_KILO_4
#define NEAR_FULL_WATERMARK_VAL_6     ICP_RING_NEAR_WATERMARK_KILO_4
#define NEAR_FULL_WATERMARK_VAL_7     ICP_RING_NEAR_WATERMARK_KILO_2
#define NEAR_FULL_WATERMARK_VAL_8     ICP_RING_NEAR_WATERMARK_KILO_1
#define NEAR_FULL_WATERMARK_VAL_9     ICP_RING_NEAR_WATERMARK_512

/* Possible nearly empty/full threshold config values */
#define NEAR_EMPTY_CONFIG_VAL_0       2
#define NEAR_EMPTY_CONFIG_VAL_1       4
#define NEAR_EMPTY_CONFIG_VAL_2       8
#define NEAR_EMPTY_CONFIG_VAL_3       16
#define NEAR_EMPTY_CONFIG_VAL_4       32
#define NEAR_EMPTY_CONFIG_VAL_5       64
#define NEAR_EMPTY_CONFIG_VAL_6       192
#define NEAR_EMPTY_CONFIG_VAL_7       224
#define NEAR_EMPTY_CONFIG_VAL_8       240
#define NEAR_EMPTY_CONFIG_VAL_9       248

/* Interrupt source values */
#define INT_SRC_N_FULL_TRUE           2
#define INT_SRC_EMPTY_FALSE           4
#define INT_SRC_N_EMPTY_FALSE         5

/* Defines for the IAIntSrcSel register setup */
#define IAINTSRCSEL_REG_MAX_RING_IN_REG  7
#define IAINTSRCSEL_REG_RINGS_PER_REG    8
#define IAINTSRCSEL_REG_BITSPERRING      4
#define IAINTSRCSEL_REG_IRQ_MODE_BIT     3
#define IAINTSRCSEL_REG_RING_VAL_MASK    0xF

/* Modulo shifts for different ring sizes */
#define MODULO_SHIFT_FOR_64           6
#define MODULO_SHIFT_FOR_128          7
#define MODULO_SHIFT_FOR_256          8
#define MODULO_SHIFT_FOR_512          9
#define MODULO_SHIFT_FOR_1K           10
#define MODULO_SHIFT_FOR_2K           11
#define MODULO_SHIFT_FOR_4K           12
#define MODULO_SHIFT_FOR_8K           13
#define MODULO_SHIFT_FOR_16K          14
#define MODULO_SHIFT_FOR_32K          15
#define MODULO_SHIFT_FOR_64K          16
#define MODULO_SHIFT_FOR_128K         17
#define MODULO_SHIFT_FOR_256K         18
#define MODULO_SHIFT_FOR_512K         19
#define MODULO_SHIFT_FOR_1M           20
#define MODULO_SHIFT_FOR_2M           21
#define MODULO_SHIFT_FOR_4M           22

/* Ring size conversion */
#define ICP_ET_SIZE_TO_BYTES(size)      (64 << (size))

/* Ring watermark conversion */
#define ICP_ET_WATERMARK_TO_BYTES(wm)   ((wm == 0) ? 0 : (4 << (wm-1)))

/* Default ring size */
#define ICP_ET_DEFAULT_RING_SIZE        ICP_RINGSIZE_KILO_16

/* Default modulo shift - must correspond to the default ring size */
#define ICP_ET_DEFAULT_MODULO_SHIFT     MODULO_SHIFT_FOR_16K

/* Default message size in bytes */
#define ICP_ET_DEFAULT_MSG_SIZE         64

/* Minimum ring free space */
#define ICP_ET_RING_MIN_FREE_SPACE (ICP_ET_DEFAULT_MSG_SIZE + 1)

/* Set the response quota to a high number. */
#define ICP_NO_RESPONSE_QUOTA           10000

/* Translates from a ringNum (integer) to a ringmaskId (bit mask) */
#define RING_NUMBER_TO_ID(ring_num)     (1 << ring_num)

/*
 * Internal parameter to describe user polling method in kernel space.
 * This must be set to a value higher than the max icp_resp_deliv_method.
 * and less than the width of the Cpa32U type.
 */
#define ADF_RESP_TYPE_USER_POLL         31

/* Number of responses we need to get before we update the head
 * in a Rx ring. NOTE: this needs to be smaller than
 * min ring size - 8 msg for NF threashold. */
#define MIN_RESPONSES_PER_HEAD_WRITE    32 

/*
 * Fast message copy functions for userspace
 */
#ifdef __x86_64__
/*
 * icp_adf_memcpy64 - 64bit version of fast memcpy
 * Function copies 64 bytes (whole message) from src to dst
 */
#define icp_adf_memcpy64(dst, src)\
do {                              \
    __asm__ __volatile__ (        \
    "mov %0, %%rsi \n"            \
    "mov %1, %%rdi \n"            \
    "movdqu (%%rsi), %%xmm0 \n"   \
    "movdqu 16(%%rsi), %%xmm1 \n" \
    "movdqu 32(%%rsi), %%xmm2 \n" \
    "movdqu 48(%%rsi), %%xmm3 \n" \
    "movdqu %%xmm0, (%%rdi) \n"   \
    "movdqu %%xmm1, 16(%%rdi) \n" \
    "movdqu %%xmm2, 32(%%rdi) \n" \
    "movdqu %%xmm3, 48(%%rdi) \n" \
    :/* no output */              \
    :"r"(src), "r"(dst)           \
    :"%esi", "%edi"               \
    );                            \
} while(0);
#define icp_adf_memcpy icp_adf_memcpy64
#else
/*
 * icp_adf_memcpy32 - 32bit version of fast memcpy
 * Function copies 64 bytes (whole message) from src to dst
 */
#define icp_adf_memcpy32(dst, src)\
do {                              \
    __asm__ __volatile__ (        \
    "mov %0, %%esi \n"            \
    "mov %1, %%edi \n"            \
    "movdqu (%%esi), %%xmm0 \n"   \
    "movdqu 16(%%esi), %%xmm1 \n" \
    "movdqu 32(%%esi), %%xmm2 \n" \
    "movdqu 48(%%esi), %%xmm3 \n" \
    "movdqu %%xmm0, (%%edi) \n"   \
    "movdqu %%xmm1, 16(%%edi) \n" \
    "movdqu %%xmm2, 32(%%edi) \n" \
    "movdqu %%xmm3, 48(%%edi) \n" \
    :/* no output */              \
    :"r"(src), "r"(dst)           \
    :"%esi", "%edi"               \
    );                            \
} while(0);
#define icp_adf_memcpy icp_adf_memcpy32
#endif

/* modulo function that doesnt use slow divide operation */
static inline unsigned int modulo(unsigned int data, unsigned int shift)
{
    unsigned int div = data >> shift;
    unsigned int mult = div << shift;
    return data - mult;
}

/* Ring controler CSR Accessor Macros */
/* CSR write macro */
#define ICP_ADF_CSR_WR(csrAddr, csrOffset, val) \
    (void)((*((volatile Cpa32U*)(((Cpa8U*)csrAddr) + csrOffset)) = (val)))

/* CSR read macro */
#define ICP_ADF_CSR_RD(csrAddr, csrOffset)      \
    (*((volatile Cpa32U*)(((Cpa8U*)csrAddr) + csrOffset)))

/*
 * adf_set_hw_data_dh89xxcc
 * Function to setup DH89xxCC specific configuration
 */
void adf_set_hw_data_dh89xxcc(void *hw_data);

/*
 * adf_set_hw_data_dh89xxcciov
 * Function to setup DH89xxCC IOV specific configuration
 */
void adf_set_hw_data_dh89xxcciov(adf_hw_device_data_t *hw_data);

/*
 * adf_c2xxx_set_hw_data
 * Function to setup C2XXX specific configuration
 */
void adf_c2xxx_set_hw_data(void *hw_data);

/*
 * adf_c2xxx_set_hw_data_iov
 * Function to setup C2XXX IOV specific configuration
 */
void adf_c2xxx_set_hw_data_iov(adf_hw_device_data_t *hw_data);




#endif /* ADF_PLATFORM_COMMON_H */


