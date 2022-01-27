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
 *  version: QAT1.6.L.2.6.0-65
 *
 *****************************************************************************/

/*****************************************************************************
 * @file adf_platform_acceldev_common.h
 *
 * @description
 *      This file contains the platform specific macros for DH89xxCC that are
 *      common for PF and VF
 *
 *****************************************************************************/
#ifndef ADF_PLATFORM_ACCELDEV_COMMON_H
#define ADF_PLATFORM_ACCELDEV_COMMON_H

/*****************************************************************************
 * Define Constants and Macros
 *****************************************************************************/

/* Coalesced Interrupt Enable */
#define ETR_CSR_INTR_COL_CTL_ENABLE        0x80000000

/* Initial bank Interrupt Source mask */
#define BANK_INT_SRC_SEL_MASK_0            0x4444444CUL
#define BANK_INT_SRC_SEL_MASK_X            0x44444444UL

/* VF2PF communication masks and offsets */

/* offset for where the VF section starts in the CSR */
#define ICP_VF2PF_START_SHIFT                  16
#define ICP_PF2VF_CSR_MASK                 0x0000FFFF
#define ICP_VF2PF_CSR_MASK                 0xFFFF0000

/* PFVF user / system irq bit */
#define ICP_PFVF_IRQ_MASK                   0x1
#define ICP_VF2PF_MSGMASK_IRQ   (ICP_PFVF_IRQ_MASK << ICP_VF2PF_START_SHIFT)
#define ICP_PF2VF_MSGMASK_IRQ              ICP_PFVF_IRQ_MASK
#define ICP_VF2PF_IRQ_CLEARED              0x00

/* PFVF user / system msg origin field */
/* The mask matches the field size */
#define ICP_PFVF_MSGORIGIN_MASK           0x01
#define ICP_PFVF_MSGORIGIN_SHIFT          1
/* Field values */
#define ICP_PFVF_MSGORIGIN_SYSTEM         1
#define ICP_PFVF_MSGORIGIN_USER           0

/* PFVF system msg type field */
/* The mask matches the field size */
#define ICP_PFVF_MSGTYPE_MASK             0x0F
#define ICP_PFVF_MSGTYPE_SHIFT            2
/* Field values */
/* VF2PF system message types */
#define ICP_VF2PF_MSGTYPE_RESERVED         0x0
/*#define ICP_VF2PF_MSGTYPE_RINGOP           0x1 */
/*#define ICP_VF2PF_MSGTYPE_RINGOFFSET       0x2 */
#define ICP_VF2PF_MSGTYPE_INIT             0x3
#define ICP_VF2PF_MSGTYPE_SHUTDOWN         0x4
#define ICP_VF2PF_MSGTYPE_VERSION_REQ      0x5

/* PF2VF system message types */
#define ICP_PF2VF_MSGTYPE_RESERVED         0x0
#define ICP_PF2VF_MSGTYPE_RESTARTING       0x1
#define ICP_PF2VF_MSGTYPE_VERSION_RESP     0x2
/*#define ICP_VF2PF_MSGRINGINFO_RESP         0x3 */

/* PFVF user msg content field */
/* The mask matches the field size */
#define ICP_PFVF_MSGCONTENT_MASK             0x3FFF
#define ICP_PFVF_MSGCONTENT_SHIFT            2

/* PFVF version fields for request and response msgs */
/* The mask matches the field size */
#define ICP_PFVF_MINORVERSION_MASK      0x0F
#define ICP_PFVF_MINORVERSION_SHIFT     6
#define ICP_PFVF_MAJORVERSION_MASK      0x0F
#define ICP_PFVF_MAJORVERSION_SHIFT     10

/* PFVF version_supported field */
/* The mask matches the field size */
#define ICP_PFVF_VF_SUPPORTED_MASK      0x03
#define ICP_PFVF_VF_SUPPORTED_SHIFT     14
/* Field values */
#define ICP_PFVF_VF_SUPPORTED      0x1
#define ICP_PFVF_VF_NOTSUPPORTED   0x2
#define ICP_PFVF_VF_SUPPORTUNKNOWN 0x3


/* IN_USE pattern must be unique, i.e. different from any pattern a
 * message can generate. For that reason System message type 0x0
 * is reserved above, so xxxx xxxx xx00 001x can only occur in
 * IN_USE pattern. For bits 6-15 an arbitrary pattern was chosen
 * 0110 1010 11xx xxxx */
#define ICP_IN_USE_BY_PF                   0x6AC20000
#define ICP_IN_USE_BY_VF                   0x6AC2

#define ICP_IN_USE_BY_PF_MASK              0xFFFE0000
#define ICP_IN_USE_BY_VF_MASK              0xFFFE

/* support for VF2PF comms messaging */ 
#define ICP_PFVFCOMMS_INTERUPT_PENDING_DELAY     2
#define ICP_PFVFCOMMS_COLLISION_DETECTION_DELAY  10
#define ICP_PFVFCOMMS_CONFIRMATION_MAX_LOOPS     100
#define ICP_PFVFCOMMS_CONFIRMATION_SLEEP_TIME    2
#define ICP_PFVFCOMMS_PUTMSG_MAXRETRIES          3
#define ICP_PFVFCOMMS_DOORBELL_CSR_MUTEX_WAIT    2
#define ICP_PFVFCOMMS_PUTMSG_RETRY               5

#define ADF_PFVF_MAX_NUM_PF                     1
#define ICP_ADF_MAX_NUM_VF                     32
#define ICP_PFVF_BUFFER_COUNT_MASK             0xFFFF
#define ICP_PFVF_BUFFER_MSG_SHIFT              16


/* Ring Csrs offsets */
#define ICP_RING_CSR_RING_CONFIG           0x000
#define ICP_RING_CSR_RING_LBASE            0x040
#define ICP_RING_CSR_RING_UBASE            0x080
#define ICP_RING_CSR_RING_HEAD_OFFSET      0x0C0
#define ICP_RING_CSR_RING_TAIL_OFFSET      0x100
#define ICP_RING_CSR_RING_STAT             0x140
#define ICP_RING_CSR_UO_STAT               0x148
#define ICP_RING_CSR_E_STAT                0x14C
#define ICP_RING_CSR_NE_STAT               0x150
#define ICP_RING_CSR_NF_STAT               0x154
#define ICP_RING_CSR_F_STAT                0x158
#define ICP_RING_CSR_C_STAT                0x15C
#define ICP_RING_CSR_INT_EN                0x16C
#define ICP_RING_CSR_INT_REG               0x170
#define ICP_RING_CSR_INT_SRCSEL            0x174
#define ICP_RING_CSR_INT_SRCSEL_2          0x178
#define ICP_RING_CSR_INT_COL_EN            0x17C
#define ICP_RING_CSR_INT_COL_CTL           0x180
/*This value is included here as we dont yet need to create device specific
  header files for userspace*/
#define ICP_DH895xCC_RING_CSR_FLAG_AND_COL_EN               0x184
#define DEFAULT_BUNDLE_SIZE                0x200

/* RingConfig CSR Parameter Watermark Offsets */
#define RING_CONFIG_NEAR_FULL_WM           0x0A
#define RING_CONFIG_NEAR_EMPTY_WM          0x05
#define RING_CONFIG_LATE_HEAD_POINTER_MODE 0x80000000

/* Default RingConfig is Nearly Full = Full and Nearly Empty = Empty */
#define BUILD_RING_CONFIG( size )    \
    ( ( ICP_RING_NEAR_WATERMARK_0 << RING_CONFIG_NEAR_FULL_WM )  \
    | ( ICP_RING_NEAR_WATERMARK_0 << RING_CONFIG_NEAR_EMPTY_WM ) \
    | size )

/* Response Ring Configuration */
#define BUILD_RESP_RING_CONFIG( size, watermark_nf, watermark_ne)  \
    ( ( watermark_nf << RING_CONFIG_NEAR_FULL_WM )  \
    | ( watermark_ne << RING_CONFIG_NEAR_EMPTY_WM ) \
    | size )

/* All Ring Base Addresses are 64 byte aligned, thus
 * bits[43:0] of the RingBase register correspond to
 * bits[49:6] of the Rings Memory Address. */

#define BUILD_RING_BASE_ADDR( addr, size ) \
   ( (addr >> 6) & (0xFFFFFFFFFFFFFFFFULL << size ) )

#define IRQ_READ_SINT_OFFSET( dev, val ) \
        pci_read_config_dword( dev, SINT_OFFSET, val )

/* CSR read/write macros */
#define READ_CSR_RING_CONFIG( bank_offset, ring ) \
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                        ICP_RING_CSR_RING_CONFIG + ( ring << 2 ) )

#define READ_CSR_RING_HEAD( bank_offset, ring ) \
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_RING_HEAD_OFFSET + ( ring << 2 ) )

#define READ_CSR_RING_TAIL( bank_offset, ring ) \
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_RING_TAIL_OFFSET + ( ring << 2 ) )


#define READ_CSR_E_STAT( bank_offset ) \
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_E_STAT )

#define READ_CSR_NE_STAT( bank_offset )\
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_NE_STAT )

#define READ_CSR_NF_STAT( bank_offset )\
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_NF_STAT )

#define READ_CSR_F_STAT( bank_offset )\
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_F_STAT )

#define READ_CSR_INT_EN( bank_offset ) \
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_INT_EN )

#define READ_CSR_INT_REG( bank_offset ) \
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_INT_REG )

#define READ_CSR_INT_COL_EN( bank_offset ) \
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_INT_COL_EN )

#define READ_CSR_INT_COL_CTL( bank_offset ) \
ICP_ADF_CSR_RD( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_INT_COL_CTL )

#define WRITE_CSR_RING_CONFIG( bank_offset, ring, value ) \
ICP_ADF_CSR_WR( csr_base_addr , bank_offset + \
           ICP_RING_CSR_RING_CONFIG + ( ring << 2 ), value )

#define WRITE_CSR_RING_BASE( bank_offset, ring, value )                \
do {                                                            \
     Cpa32U l_base = 0, u_base = 0;                             \
     l_base = (Cpa32U)(value & 0xFFFFFFFF);                     \
     u_base = (Cpa32U)((value & 0xFFFFFFFF00000000ULL) >> 32);  \
     ICP_ADF_CSR_WR( csr_base_addr , bank_offset +   \
           ICP_RING_CSR_RING_LBASE + ( ring << 2 ), l_base );   \
     ICP_ADF_CSR_WR( csr_base_addr , bank_offset +   \
           ICP_RING_CSR_RING_UBASE + ( ring << 2 ), u_base );   \
} while(0)

static inline Cpa64U read_base(Cpa32U *csr_base_addr,
                               Cpa32U bank_offset, Cpa32U ring)
{
    Cpa32U l_base = ICP_ADF_CSR_RD(csr_base_addr , bank_offset +
                                  ICP_RING_CSR_RING_LBASE + (ring << 2 ));
    Cpa32U u_base = ICP_ADF_CSR_RD(csr_base_addr , bank_offset +
                                   ICP_RING_CSR_RING_UBASE + (ring << 2));
    Cpa64U addr = (l_base & 0xFFFFFFFF);
    addr |= ((((Cpa64U)u_base) << 32) & 0xFFFFFFFF00000000ULL);
    return addr;
}

#define READ_CSR_RING_BASE(bank_offset, ring) \
                read_base(csr_base_addr, bank_offset, ring)

#define WRITE_CSR_RING_HEAD( bank_offset, ring, value  ) \
ICP_ADF_CSR_WR( csr_base_addr , bank_offset + \
           ICP_RING_CSR_RING_HEAD_OFFSET + ( ring << 2 ), value )

#define WRITE_CSR_RING_TAIL( bank_offset, ring, value ) \
ICP_ADF_CSR_WR( csr_base_addr , bank_offset + \
           ICP_RING_CSR_RING_TAIL_OFFSET + ( ring << 2 ), value )

#define WRITE_CSR_INT_EN( bank_offset, value ) \
ICP_ADF_CSR_WR( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_INT_EN, value )

#define WRITE_CSR_INT_SRCSEL( bank_offset )                        \
do {                                                        \
    ICP_ADF_CSR_WR( csr_base_addr, bank_offset + \
    ICP_RING_CSR_INT_SRCSEL, BANK_INT_SRC_SEL_MASK_0 );     \
    ICP_ADF_CSR_WR( csr_base_addr, bank_offset + \
    ICP_RING_CSR_INT_SRCSEL_2, BANK_INT_SRC_SEL_MASK_X );   \
 } while (0)

#define UPDATE_CSR_INT_SRCSEL_RING( bank_offset, ring, val ) \
do {                                                  \
    if(ring > IAINTSRCSEL_REG_MAX_RING_IN_REG)        \
    {                                                 \
        ICP_ADF_CSR_WR( csr_base_addr, bank_offset + \
        ICP_RING_CSR_INT_SRCSEL_2, val );             \
    }                                                 \
    else                                              \
    {                                                 \
        ICP_ADF_CSR_WR( csr_base_addr, bank_offset + \
        ICP_RING_CSR_INT_SRCSEL, val );               \
    }                                                 \
 } while (0)


#define WRITE_CSR_INT_REG( bank_offset, value ) \
ICP_ADF_CSR_WR( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_INT_REG, value )

#define WRITE_CSR_INT_COL_EN( bank_offset, value ) \
ICP_ADF_CSR_WR( csr_base_addr , bank_offset + \
                   ICP_RING_CSR_INT_COL_EN, value )

#define WRITE_CSR_INT_COL_CTL( bank_offset, value ) \
ICP_ADF_CSR_WR( csr_base_addr , bank_offset + \
                        ICP_RING_CSR_INT_COL_CTL, \
                        ETR_CSR_INTR_COL_CTL_ENABLE | value )

#define WRITE_CSR_INT_COL_CTL_CLR( bank_offset ) \
ICP_ADF_CSR_WR( csr_base_addr , bank_offset + \
                        ICP_RING_CSR_INT_COL_CTL, 0)

#endif /* ADF_PLATFORM_ACCELDEV_COMMON_H */
