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
 * @file adf_platform_dh89xxcc.h
 *
 * @description
 *      This file contains the platform specific macros for DH89xxCC processor
 *
 *****************************************************************************/
#ifndef ADF_PLATFORM_DH89xxCC_H
#define ADF_PLATFORM_DH89xxCC_H

/*****************************************************************************
 * Define Constants and Macros
 *****************************************************************************/

/* PCIe configuration space */
#define ICP_DH89xxCC_PESRAM_BAR              0
#define ICP_DH89xxCC_PESRAM_BAR_SIZE         0x80000 /* (512KB) */
#define ICP_DH89xxCC_PMISC_BAR               1
#define ICP_DH89xxCC_PMISC_BAR_SIZE          0x20000 /* (128KB) */
#define ICP_DH89xxCC_PETRINGCSR_BAR          2
#define ICP_DH89xxCC_PETRINGCSR_BAR_SIZE     0x4000  /* (16KB) */
#define ICP_DH89xxCC_MAX_PCI_BARS            3

/* eSRAM */
#define ICP_DH89xxCC_SRAM_AE_ADDR            0x2000000000000ull
#define ICP_DH89xxCC_SRAM_AE_SIZE            0x80000

/* Clock */
#define ICP_DH89xxCC_AE_CLOCK_IN_MHZ         1066

/* ETR */
#define ICP_DH89xxCC_ETR_MAX_BANKS           16
#define ICP_DH89xxCC_ETR_MAX_RINGS_PER_BANK  16
#define ICP_DH89xxCC_BANKS_PER_ACCELERATOR   8
#define ICP_DH89xxCC_MAX_ET_RINGS            \
    (ICP_DH89xxCC_ETR_MAX_BANKS * ICP_DH89xxCC_ETR_MAX_RINGS_PER_BANK)
#define ICP_DH89xxCC_ETR_MAX_AP_BANKS        8

/* msix vector */
#define ICP_DH89xxCC_MSIX_BANK_VECTOR_START  0
#define ICP_DH89xxCC_MSIX_BANK_VECTOR_NUM    16
#define ICP_DH89xxCC_MSIX_AE_VECTOR_START    16
#define ICP_DH89xxCC_MSIX_AE_VECTOR_NUM      1

/* Fuse Control */
#define ICP_DH89xxCC_FUSECTL_OFFSET         0x40
#define ICP_DH89xxCC_CLKCTL_OFFSET          0x4C
#define ICP_DH89xxCC_CLKCTL_MASK            0x380000
#define ICP_DH89xxCC_CLKCTL_SHIFT           0x13
#define ICP_DH89xxCC_CLKCTL_DIV_SKU3        0x6
#define ICP_DH89xxCC_CLKCTL_DIV_SKU4        0x7

#define ICP_DH89xxCC_MAX_ACCELERATORS       2
#define ICP_DH89xxCC_MAX_ACCELENGINES       8
#define ICP_DH89xxCC_ACCELERATORS_MASK      0x03
#define ICP_DH89xxCC_ACCELENGINES_MASK      0xFF

/* Accelerator Functions */
#define ICP_DH89xxCC_LEGFUSE_OFFSET         0x4C
#define ICP_DH89xxCC_LEGFUSE_MASK           0x1F
#define ICP_DH89xxCC_LEGFUSES_FN0BIT4       0x04
#define ICP_DH89xxCC_LEGFUSES_FN0BIT3       0x03
#define ICP_DH89xxCC_LEGFUSES_FN0BIT2       0x02
#define ICP_DH89xxCC_LEGFUSES_FN0BIT1       0x01
#define ICP_DH89xxCC_LEGFUSES_FN0BIT0       0x00


/* PMISC BAR offsets */
#define PMISCBAROFFSET    0x1A000

/* Interrupt */
#define ICP_DH89xxCC_SINTPF                PMISCBAROFFSET + 0x24
#define ICP_DH89xxCC_SMIAOF                PMISCBAROFFSET + 0x28
#define ICP_DH89xxCC_BUNDLES_IRQ_MASK      0xFFFF
#define ICP_DH89xxCC_AE_IRQ_MASK           0x10000
#define ICP_DH89xxCC_SMIA_MASK             \
    (ICP_DH89xxCC_BUNDLES_IRQ_MASK | ICP_DH89xxCC_AE_IRQ_MASK)

/* VF-to-PF Messaging Interrupt */
#define ICP_DH89xxCC_MAX_NUM_VF            16
#define ICP_PMISCBAR_TIMISCINTCTL          PMISCBAROFFSET + 0x548
#define ICP_PMISCBAR_VFTOPFINTSRC          PMISCBAROFFSET + 0x0C
#define ICP_PMISCBAR_VFTOPFINTMSK          PMISCBAROFFSET + 0x1C
#define ICP_VFTOPF_INTSOURCEMASK           0x1FFFE00
#define ICP_VFTOPF_INTSOURCEOFFSET         0x09

#define ICP_PMISCBAR_PF2VFDBINT_OFFSET     PMISCBAROFFSET + 0xD0
#define ICP_PMISCBAR_VINTSOU_OFFSET        PMISCBAROFFSET + 0x180
#define ICP_PMISCBAR_VINTMSK_OFFSET        PMISCBAROFFSET + 0x1C0
#define ICP_PMISCBAR_VINTMSK_DEFAULT       0x02
#define ICP_PMISCBAR_VFDEVOFFSET           0x04

/* PETRING BAR offsets */
#define ICP_DH89xxCC_ETRING_CSR_OFFSET      0x00

/* Ring configuration space size
 * This chunk of memory will be mapped up to userspace
 * It is 0x200 times 16 bundles
 */
#define ICP_DH89xxCC_ETRING_CSR_SIZE        0x2000

/* Autopush CSR Offsets */
#define ICP_DH89xxCC_AP_REGS_OFFSET         0x2000
#define ICP_DH89xxCC_AP_NF_MASK             ICP_DH89xxCC_AP_REGS_OFFSET + 0x00
#define ICP_DH89xxCC_AP_NF_DEST             ICP_DH89xxCC_AP_REGS_OFFSET + 0x20
#define ICP_DH89xxCC_AP_NE_MASK             ICP_DH89xxCC_AP_REGS_OFFSET + 0x40
#define ICP_DH89xxCC_AP_NE_DEST             ICP_DH89xxCC_AP_REGS_OFFSET + 0x60
#define ICP_DH89xxCC_AP_DELAY               ICP_DH89xxCC_AP_REGS_OFFSET + 0x80
#define ICP_DH89xxCC_AP_BANK_BYTE_OFFSET    4

/* User space */
#define ICP_DH89xxCC_USER_ETRING_CSR_SIZE       0x2000

#define SYS_PAGE_SIZE_4k                   0x0
#define SYS_PAGE_SIZE_8k                   0x1
#define SYS_PAGE_SIZE_64k                  0x2
#define SYS_PAGE_SIZE_256k                 0x4
#define SYS_PAGE_SIZE_1M                   0x8
#define SYS_PAGE_SIZE_4M                   0x10

/* PMISC BAR region offsets and sizes */
#define ICP_DH89xxCC_REGION_CAP_OFFSET           0x0
#define ICP_DH89xxCC_REGION_CAP_SIZE             0x4000  /* (16KB) */
#define ICP_DH89xxCC_REGION_SCRATCH_RAM_OFFSET   0x4000
#define ICP_DH89xxCC_REGION_SCRATCH_RAM_SIZE     0x4000  /* (16KB) */
#define ICP_DH89xxCC_REGION_CPMS_OFFSET
#define ICP_DH89xxCC_REGION_CPMS_SIZE            0x8000  /* (32KB) */
#define ICP_DH89xxCC_REGION_MES_OFFSET
#define ICP_DH89xxCC_REGION_MES_SIZE             0x8000  /* (32KB) */
#define ICP_DH89xxCC_REGION_CHAP_PMU_OFFSET
#define ICP_DH89xxCC_REGION_CHAP_PMU_SIZE        0x2000  /* (8KB) */
#define ICP_DH89xxCC_REGION_EP_MMIO_CSRS_OFFSET
#define ICP_DH89xxCC_REGION_EP_MMIO_CSRS_SIZE    0x1000  /* (4KB) */
#define ICP_DH89xxCC_REGION_MSIX_TABS_OFFSET
#define ICP_DH89xxCC_REGION_MSIX_TABS_SIZE       0x1000  /* (4KB) */

/*
 * Offsets
 */
#define ICP_DH89xxCC_RESET_OFFSET                  (0xA0C)
#define ICP_DH89xxCC_RESET_VECTOR                  (0x3BC0FF)
#define ICP_DH89xxCC_PMISCBAR_AE_OFFSET            (0x10000)
#define ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET         (0x8000)
#define ICP_DH89xxCC_PMISCBAR_RING_CLUSTER_OFFSET  (0x1A800)
#define ICP_DH89xxCC_PMISCBAR_CPP_SHAC_OFFSET      (0xC00)

/*
 * SSM error type
 * List the various SSM based Uncorrectable Errors
 */
#define ICP_DH89xxCC_SHAREDMEM_T         0 /*Shared Memory UErrors*/
#define ICP_DH89xxCC_MMP_T               1 /*MMP ECC UErrors*/
#define ICP_DH89xxCC_PPERR_T             2 /*Push/Pull UErrors*/

/*
 * List all the AEs
 */
#define ICP_DH89xxCC_AE0                 0
#define ICP_DH89xxCC_AE1                 1
#define ICP_DH89xxCC_AE2                 2
#define ICP_DH89xxCC_AE3                 3
#define ICP_DH89xxCC_AE4                 4
#define ICP_DH89xxCC_AE5                 5
#define ICP_DH89xxCC_AE6                 6
#define ICP_DH89xxCC_AE7                 7

/* MMP SLICES */
#define ICP_DH89xxCC_SLICE0              0 /*MMP Slice 0*/
#define ICP_DH89xxCC_SLICE1              1 /*MMP Slice 1*/

/*
 * Uncorrectable errors Mask registers
 */

/*PMISCBAR Uncorrectable Error Mask*/
#define ICP_DH89xxCC_ERRMSK0             (PMISCBAROFFSET + 0x10)
#define ICP_DH89xxCC_ERRMSK1             (PMISCBAROFFSET + 0x14)
#define ICP_DH89xxCC_ERRMSK2             (PMISCBAROFFSET + 0x18)
#define ICP_DH89xxCC_ERRMSK3             (PMISCBAROFFSET + 0x1C)
/*PMISCBAR Uncorrectable Error Mask value*/
#define ICP_DH89xxCC_M3UNCOR             (0<<25) /*AE3 uncorrectable errors*/
#define ICP_DH89xxCC_M3COR               (1<<24) /*!AE3 correctable errors*/
#define ICP_DH89xxCC_M2UNCOR             (0<<17) /*AE2 uncorrectable errors*/
#define ICP_DH89xxCC_M2COR               (1<<16) /*!AE2 correctable errors*/
#define ICP_DH89xxCC_M1UNCOR             (0<<9)  /*AE1 uncorrectable errors*/
#define ICP_DH89xxCC_M1COR               (1<<8)  /*!AE1 correctable errors*/
#define ICP_DH89xxCC_M0UNCOR             (0<<1)  /*AE0 uncorrectable errors*/
#define ICP_DH89xxCC_M0COR               (1<<0)  /*!AE0 correctable errors*/

#define ICP_DH89xxCC_ERRMSK0_UERR                  \
    (ICP_DH89xxCC_M3UNCOR | ICP_DH89xxCC_M2UNCOR | \
    ICP_DH89xxCC_M1UNCOR  | ICP_DH89xxCC_M0UNCOR | \
    ICP_DH89xxCC_M3COR    | ICP_DH89xxCC_M2COR   | \
    ICP_DH89xxCC_M1COR    | ICP_DH89xxCC_M0COR)

#define ICP_DH89xxCC_M7UNCOR             (0<<25) /*AE7 uncorrectable errors*/
#define ICP_DH89xxCC_M7COR               (1<<24) /*!AE7 correctable errors*/
#define ICP_DH89xxCC_M6UNCOR             (0<<17) /*AE6 uncorrectable errors*/
#define ICP_DH89xxCC_M6COR               (1<<16) /*!AE6 correctable errors*/
#define ICP_DH89xxCC_M5UNCOR             (0<<9)  /*AE5 uncorrectable errors*/
#define ICP_DH89xxCC_M5COR               (1<<8)  /*!AE5 correctable errors*/
#define ICP_DH89xxCC_M4UNCOR             (0<<1)  /*AE4 uncorrectable errors*/
#define ICP_DH89xxCC_M4COR               (1<<0)  /*!AE4 correctable errors*/

#define ICP_DH89xxCC_ERRMSK1_UERR                  \
    (ICP_DH89xxCC_M7UNCOR | ICP_DH89xxCC_M6UNCOR | \
    ICP_DH89xxCC_M5UNCOR  | ICP_DH89xxCC_M4UNCOR | \
    ICP_DH89xxCC_M7COR    | ICP_DH89xxCC_M6COR   | \
    ICP_DH89xxCC_M5COR    | ICP_DH89xxCC_M4COR)

#define ICP_DH89xxCC_UERR                (0<<8) /*eSRam uncorrectable errors*/
#define ICP_DH89xxCC_PPMISCERR           (0<<6) /*Push Pull Misc uncorrectable
                                                   errors*/
#define ICP_DH89xxCC_EMSK3_CPM1          (0<<3) /*CPM1 interrupts*/
#define ICP_DH89xxCC_EMSK3_CPM0          (0<<2) /*CPM0 interrupts*/
#define ICP_DH89xxCC_EMSK3_SHaC1         (0<<1) /*OR'ed ShaC Attention Signal
                                                 needed by fw debug interrupt*/
#define ICP_DH89xxCC_EMSK3_SHaC0         (0<<0) /*OR'ed ShaC data errors*/
#define ICP_DH89xxCC_TIMISC              (1<<4) /*!FLR/BME Errors*/
#define ICP_DH89xxCC_RIMISC              (1<<5) /*!Push Pull error
                                                  detected by RI*/
#define ICP_EMSK3_DISABLE_OTHER          \
    ((1<<27) | (1<<26) | (1<<25) |(1<<7) |(0xFFFF<<9)) /* !PMU, !CPPCMD,
                             !VFTOPF (will be activated later if needed) */
#define ICP_DH89xxCC_ERRMSK3_UERR        \
    (ICP_DH89xxCC_UERR        | ICP_DH89xxCC_PPMISCERR                    | \
     ICP_DH89xxCC_EMSK3_CPM1  | ICP_DH89xxCC_EMSK3_CPM0                   | \
     ICP_DH89xxCC_EMSK3_SHaC0 | ICP_DH89xxCC_RIMISC | ICP_DH89xxCC_TIMISC | \
     ICP_DH89xxCC_EMSK3_SHaC1 | ICP_EMSK3_DISABLE_OTHER)

/*AEs(idx) ctx_enable registers*/
#define ICP_DH89xxCC_PMISCBAR_CTX_ENABLES(idx) \
    (ICP_DH89xxCC_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x818)

#define ICP_DH89xxCC_ENABLE_ECC_ERR      (1<<28) /*AE(idx) Ecc error on*/

/*AEs(idx) misc control register*/
#define ICP_DH89xxCC_PMISCBAR_MISC_CONTROL(idx) \
    (ICP_DH89xxCC_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x960)

#define ICP_DH89xxCC_ENABLE_ECC_PARITY_CORR ((1<<12) | (1<<24))
/*offset 12: CS_ECC_correct_Enable, offset 24: Parity_Enable: enable parity and
 * ECC AE fix*/

/*CPM 0 AND CPM 1 Uncorrectable Errors*/
#define ICP_DH89xxCC_INTMASKSSM(idx)     \
    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET + (idx * 0x4000))

#define ICP_DH89xxCC_PPERR_MASK          (0<<6) /*Push/Pull Uncorrectable
                                                  interrupt*/
#define ICP_DH89xxCC_MMP1_UIMASK         (0<<4) /*MMP1 Uncor interrupt*/
#define ICP_DH89xxCC_MMP0_UIMASK         (0<<2) /*MMP0 Uncor interrupt*/
#define ICP_DH89xxCC_SH_UIMASK           (0<<0) /*Shared Memory Uncorrectable
                                                  interrupt*/
#define ICP_DH89xxCC_MMP1_CIMASK         (1<<5) /*!MMP1 correctable interrupt*/
#define ICP_DH89xxCC_MMP0_CIMASK         (1<<3) /*!MMP0 correctable interrupt*/
#define ICP_DH89xxCC_SH_CIMASK           (1<<1) /*!Shared Memory correctable
                                                  interrupt*/
#define ICP_DH89xxCC_TLB_PARITY_MASK     (0<<7) /*Push/Pull Uncorrectable
                                                  interrupt*/
#define ICP_DH89xxCC_INTMASKSSM_UERR     \
    (ICP_DH89xxCC_MMP0_UIMASK | ICP_DH89xxCC_MMP1_UIMASK | \
    ICP_DH89xxCC_MMP0_CIMASK  | ICP_DH89xxCC_MMP1_CIMASK | \
    ICP_DH89xxCC_SH_CIMASK    | ICP_DH89xxCC_SH_UIMASK   | \
    ICP_DH89xxCC_PPERR_MASK   | ICP_DH89xxCC_TLB_PARITY_MASK)

/*SHAC Uncorrectable Errors*/
#define ICP_DH89xxCC_CPP_SHAC_ERR_CTRL   (ICP_DH89xxCC_PMISCBAR_CPP_SHAC_OFFSET)

/*Push/Pull control reg*/
#define ICP_DH89xxCC_SHAC_INTEN          (1<<1) /*Enable interrupt*/
#define ICP_DH89xxCC_SHAC_PPERREN        (1<<0) /*Enable push/pull errors
                                                  detection*/
#define ICP_DH89xxCC_CPP_SHAC_UE         \
    (ICP_DH89xxCC_SHAC_INTEN | ICP_DH89xxCC_SHAC_PPERREN)

/*eSRAM Uncorrectable Errors register*/
#define ICP_DH89xxCC_ESRAMUERR           \
    (ICP_DH89xxCC_PMISCBAR_RING_CLUSTER_OFFSET + 0x4)
#define ICP_DH89xxCC_ESRAMCERR           \
    (ICP_DH89xxCC_PMISCBAR_RING_CLUSTER_OFFSET + 0x0)

/*eSRAM UERROR Reg*/
#define ICP_DH89xxCC_ESRAM_INTEN         (1<<17) /*Enable interrupts*/
#define ICP_DH89xxCC_ESRAM_EN            (1<<3)  /*Enable Error Detection*/
#define ICP_DH89xxCC_ESRAM_CEN           (1<<3)  /*Enable Error Correction*/
#define ICP_DH89xxCC_ESRAM_UERR          \
    (ICP_DH89xxCC_ESRAM_INTEN | ICP_DH89xxCC_ESRAM_EN)
#define ICP_DH89xxCC_ESRAM_CERR          \
    (ICP_DH89xxCC_ESRAM_CEN)

/*Push/Pull/Misc Errors register*/
#define ICP_DH89xxCC_CPPMEMTGTERR        \
    (ICP_DH89xxCC_PMISCBAR_RING_CLUSTER_OFFSET + 0x10)

#define ICP_DH89xxCC_TGT_PPEREN          (1<<3) /*Enable Error Detection*/
#define ICP_DH89xxCC_TGT_INTEN           (1<<2) /*Enable Interrupts*/
#define ICP_DH89xxCC_TGT_UERR            \
    (ICP_DH89xxCC_TGT_PPEREN | ICP_DH89xxCC_TGT_INTEN)

/*
 * Status/Log Register
 */

/*PMISCBAR Uncorrectable Error*/
#define ICP_DH89xxCC_ERRSOU0             (PMISCBAROFFSET)
#define ICP_DH89xxCC_ERRSOU1             (PMISCBAROFFSET + 0x04)
#define ICP_DH89xxCC_ERRSOU2             (PMISCBAROFFSET + 0x08)
#define ICP_DH89xxCC_ERRSOU3             (PMISCBAROFFSET + 0x0C)
/*errsou 1*/
#define ICP_DH89xxCC_M7UNCOR_MASK        (1<<25) /*AE7 uncorrectable errors*/
#define ICP_DH89xxCC_M6UNCOR_MASK        (1<<17) /*AE6 uncorrectable errors*/
#define ICP_DH89xxCC_M5UNCOR_MASK        (1<<9)  /*AE5 uncorrectable errors*/
#define ICP_DH89xxCC_M4UNCOR_MASK        (1<<1)  /*AE4 uncorrectable errors*/
/*errsou 0*/
#define ICP_DH89xxCC_M3UNCOR_MASK        (1<<25) /*AE3 uncorrectable errors*/
#define ICP_DH89xxCC_M2UNCOR_MASK        (1<<17) /*AE2 uncorrectable errors*/
#define ICP_DH89xxCC_M1UNCOR_MASK        (1<<9)  /*AE1 uncorrectable errors*/
#define ICP_DH89xxCC_M0UNCOR_MASK        (1<<1)  /*AE0 uncorrectable errors*/
/*errsou 3*/
#define ICP_DH89xxCC_UERR_MASK           (1<<8)  /*eSRam uncorrectable errors*/
#define ICP_DH89xxCC_PPMISCERR_MASK      (1<<6)  /*Push Pull Misc
                                                   uncorrectable errors*/
#define ICP_DH89xxCC_EMSK3_CPM1_MASK     (1<<3)  /*CPM1 interrupts*/
#define ICP_DH89xxCC_EMSK3_CPM0_MASK     (1<<2)  /*CPM0 interrupts*/
#define ICP_DH89xxCC_EMSK3_SHaC0_MASK    (1)     /*OR'ed ShaC data errors*/

/*AE Status*/
#define ICP_DH89xxCC_EPERRLOG            (PMISCBAROFFSET + 0x20)
#define ICP_DH89xxCC_GET_EPERRLOG(val, me)    \
    (val & (1 <<(1 + (2 * me))))

/*AE(idx) Control*/
#define ICP_DH89xxCC_USTORE_ERROR_STATUS(idx) \
    (ICP_DH89xxCC_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x80C)
#define ICP_DH89xxCC_USTORE_UADDR(val)   (val & 0x3FFF) /* Faulty address*/
#define ICP_DH89xxCC_USTORE_GET_SYN(val) ((val & (0x7F << 20)) >> 20)
/*Syndrom*/

/*AE(idx) Parity Error*/
#define ICP_DH89xxCC_REG_ERROR_STATUS(idx) \
    (ICP_DH89xxCC_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x830)
#define ICP_DH89xxCC_REG_GET_REG_AD(val) (val & 0xFFFF) /*Faulty Register
                                                           or local mem*/
#define ICP_DH89xxCC_REG_TYPE(val)       ((val & (0x3<<15)) >> 15)

/*32bits reg for accelerator 0/1*/
#define ICP_DH89xxCC_INTSTATSSM(idx)     (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET +\
                                         0x4 + (idx * 0x4000))
#define ICP_DH89xxCC_INTSTATSSM_PPERR    (1<<6) /*Push Pull Error
                                                   -> log in PPERRSSM*/
#define ICP_DH89xxCC_INTSTATSSM_MMP1     (1<<4) /*MMP UERR -> log in
                                                  UERRSSMMMP*/
#define ICP_DH89xxCC_INTSTATSSM_MMP0     (1<<2) /*MMP UERR -> log in
                                                   UERRSSMMMP*/
#define ICP_DH89xxCC_INTSTATSSM_SH       (1)    /*Shared Memory UERR ->
                                                   log in UERRSSMSHx*/
/* MMP0-Accelerator(idx) */
#define ICP_DH89xxCC_UERRSSMMMP0(idx)    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET +\
                                         (idx*0x4000) + 0x388)

#define ICP_DH89xxCC_CERRSSMMMP0(idx)    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET +\
                                         (idx*0x4000) + 0x380)
/* MMP1-Accelerator(idx) */
#define ICP_DH89xxCC_UERRSSMMMP1(idx)    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET +\
                                         (idx*0x4000) + 0x1388)
#define ICP_DH89xxCC_CERRSSMMMP1(idx)    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET +\
                                         (idx*0x4000) + 0x1380)

#define ICP_DH89xxCC_CERRSSMMMP_EN       (1<<3)   /*1 = Enable correction*/
#define ICP_DH89xxCC_UERRSSMMMP_EN       (1<<3)   /*1 = Enable logging*/
#define ICP_DH89xxCC_UERRSSMMMP_UERR     (1)      /*1 = Uerr*/
#define ICP_DH89xxCC_UERRSSMMMP_ERRTYPE(val) \
    ((val & (0xF<<4)) >> 4)

/*MMP0 address ACCEL(idx)*/
#define ICP_DH89xxCC_UERRSSMMMPAD0(idx)  \
    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET + 0x38C + (idx * 0x4000))
/*MMP1 address ACCEL(idx)*/
#define ICP_DH89xxCC_UERRSSMMMPAD1(idx)  \
    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET + 0x138C + (idx * 0x4000))
#define ICP_DH89xxCC_UERRSSMMMPAD_ADDR   (0xFFFF) /*Address Mask*/

/*Shared memory*/
#define ICP_DH89xxCC_UERRSSMSH(idx)      \
    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET + 0x18 + (idx * 0x4000))
#define ICP_DH89xxCC_CERRSSMSH(idx)      \
    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET + 0x10 + (idx * 0x4000))
#define ICP_DH89xxCC_CERRSSMSH_EN        (1<<3) /*Enable Correction*/
#define ICP_DH89xxCC_UERRSSMSH_EN        (1<<3) /*Enable logging*/
#define ICP_DH89xxCC_UERRSSMSH_UERR      (1<<0) /*Uncorrectable error*/
#define ICP_DH89xxCC_UERRSSMSH_R         (1<<1) /*On read operation*/
#define ICP_DH89xxCC_UERRSSMSH_W         (1<<2) /*On Write operation*/
#define ICP_DH89xxCC_UERRSSMSH_ERRTYPE(val) \
    ((val & (0xF<<4)) >> 4)

/* Shared Ram status Accelrator 0,1 */
#define ICP_DH89xxCC_UERRSSMSHAD(idx)    (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET +\
                                         0x1C + (idx * 0x4000))
/*Shared Ram status Address Mask*/
#define ICP_DH89xxCC_UERRSSMSHAD_ADDR    (0xFFFF) /*Address Mask*/

/*Push/Pull*/
#define ICP_DH89xxCC_PPERR(idx)          (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET +\
                                         0x8 + (idx * 0x4000))
/*push/pull status accelerator 0,1*/
#define ICP_DH89xxCC_PPERR_EN            (1<<2) /*Enable Err logging*/
#define ICP_DH89xxCC_PPERR_MERR          (1<<1) /*Multiple Errors*/
#define ICP_DH89xxCC_PPERR_PPERR         (1<<0) /*Uncorrectable Error*/
#define ICP_DH89xxCC_PPERR_TYPE(val)     ((val & (0xF<<4)) >> 4)

/*push/pull faulty ID accelerator 0,1*/
#define ICP_DH89xxCC_PPERRID(idx)        (ICP_DH89xxCC_PMISCBAR_ACCEL_OFFSET +\
                                         0xC + (idx * 0x4000))
#define ICP_DH89xxCC_CPP_SHAC_ERR_STATUS     \
    (ICP_DH89xxCC_PMISCBAR_CPP_SHAC_OFFSET+0x4) /*Push/Pull status reg*/

#define ICP_DH89xxCC_CPP_SHAC_GET_TYP(val)   \
    ((val & (1<<3)) >> 3) /*Error type 0-push(read) 1-pull(write)*/
#define ICP_DH89xxCC_CPP_SHAC_GET_ERTYP(val) \
    ((val & (0x3<<4)) >> 4)

#define ICP_DH89xxCC_CPP_SHAC_GET_INT(val)  ((val & (1<<2)) >> 2)
/*Push/Pull status interrupt occured*/
#define ICP_DH89xxCC_CPP_SHAC_ERR_PPID   \
    (ICP_DH89xxCC_PMISCBAR_CPP_SHAC_OFFSET + 0xC)
/*Push/Pull faulty ID(32b) reg*/

/*eSRAM*/
#define ICP_DH89xxCC_ESRAMUERR_UERR      (1<<0) /*eSRAM UERR*/

#define ICP_DH89xxCC_ESRAMUERR_R         (1<<1) /*eSRAM UERR on read*/
#define ICP_DH89xxCC_ESRAMUERR_W         (1<<2) /*eSRAM UERR on write*/
#define ICP_DH89xxCC_ESRAMUERR_GET_ERRTYPE(val) \
    ((val & (0xF<<4)) >> 4)

#define ICP_DH89xxCC_ESRAMUERRAD         \
    (ICP_DH89xxCC_PMISCBAR_RING_CLUSTER_OFFSET + 0xC)
/*eSRAM UERR Addr in Qword->address|000*/

/*Miscellaneous Memory Target Error*/
#define ICP_DH89xxCC_CPPMEMTGTERR_ERRTYP(val) \
    ((val & (0xF<<16)) >> 16)

#define ICP_DH89xxCC_CPPMEMTGTERR_ERR    (1<<0)  /*Misc Memory Target Error
                                                    occured*/
#define ICP_DH89xxCC_CPPMEMTGTERR_MERR   (1<<1)  /*Multiple Errors occurred*/

#define ICP_DH89xxCC_ERRPPID             \
    (ICP_DH89xxCC_PMISCBAR_RING_CLUSTER_OFFSET + 0x14)

/*32 bits push/pull UERR id*/
#define ICP_DH89xxCC_TICPPINTCTL         (0x1A400 + 0x138) /*TI CPP control*/
#define ICP_DH89xxCC_TICPP_PUSH          (1<<0) /*push error log*/
#define ICP_DH89xxCC_TICPP_PULL          (1<<1) /*pull error log*/

#define ICP_DH89xxCC_TICPP_EN            \
    (ICP_DH89xxCC_TICPP_PUSH | ICP_DH89xxCC_TICPP_PULL)    /*Enable Error log*/

#define ICP_DH89xxCC_TICPPINTSTS         (0x1A400 + 0x13C) /*captures the
                                                            CPP error detected
                                                            on the TI CPP bus*/

#define ICP_DH89xxCC_TIERRPUSHID         (0x1A400 + 0x140) /*log the id of the
                                                            push error*/
#define ICP_DH89xxCC_TIERRPULLID         (0x1A400 + 0x144) /*log the id of the
                                                            pull error*/

/*
 * adf_isr_getMiscBarAd
 * Returns the MISC BAR address of the device
 */
static inline void *
adf_isr_getMiscBarAd(icp_accel_dev_t *accel_dev)
{
    Cpa32U pmisc_bar_id = 0;
    adf_hw_device_data_t  *hw_data = NULL;

    hw_data = accel_dev->pHwDeviceData;
    pmisc_bar_id = hw_data->getMiscBarId(hw_data);
    return (void*)(UARCH_INT)accel_dev->pciAccelDev.pciBars[pmisc_bar_id].virtAddr;
}

/*
 * irq_get_bank_number
 * Function returns the interrupt source's bank number
 */
static inline Cpa32U irq_get_bank_number(Cpa32U sint)
{
    Cpa32U bank_number = 0;
    bank_number = sint & ICP_DH89xxCC_BUNDLES_IRQ_MASK;
    return bank_number;
}

/*
 * irq_ae_source
 * Function returns the value of AE interrupt source
 */
static inline Cpa32U irq_ae_source(Cpa32U sint)
{
    Cpa32U ae_irq = 0;
    ae_irq = sint & ICP_DH89xxCC_AE_IRQ_MASK;
    return ae_irq;
}

#endif /* ADF_PLATFORM_DH89xxCC_H */
