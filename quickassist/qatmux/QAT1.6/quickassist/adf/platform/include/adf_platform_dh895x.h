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
 * @file adf_platform_dh895x.h
 *
 * @description
 *      This file contains the platform specific macros for DH895X processor
 *
 *****************************************************************************/
#ifndef ADF_PLATFORM_DH895X_H
#define ADF_PLATFORM_DH895X_H


/*****************************************************************************
 * Define Constants and Macros
 *****************************************************************************/

/*
 * List all the AEs
 */
#define ICP_DH895xCC_AE0        0
#define ICP_DH895xCC_AE1        1
#define ICP_DH895xCC_AE2        2
#define ICP_DH895xCC_AE3        3
#define ICP_DH895xCC_AE4        4
#define ICP_DH895xCC_AE5        5
#define ICP_DH895xCC_AE6        6
#define ICP_DH895xCC_AE7        7
#define ICP_DH895xCC_AE8        8
#define ICP_DH895xCC_AE9        9
#define ICP_DH895xCC_AE10       10
#define ICP_DH895xCC_AE11       11

/* Device minor and major */
#define DH895xCC_MINOR_STEPPING_MASK  0x3
#define DH895xCC_MAJOR_STEPPING_MASK  0x30
#define DH895xCC_MAJOR_STEPPING_SHIFT 0x4

/*
 * List of MMP slices
 */
#define ICP_DH895xCC_MMP_SLICE0    0 /* MMP Slice 0 */
#define ICP_DH895xCC_MMP_SLICE1    1 /* MMP Slice 1 */
#define ICP_DH895xCC_MMP_SLICE2    2 /* MMP Slice 2 */
#define ICP_DH895xCC_MMP_SLICE3    3 /* MMP Slice 3 */
#define ICP_DH895xCC_MMP_SLICE4    4 /* MMP Slice 4 */

/* Initial bank Interrupt Source mask */
#define ICP_DH895xCC_BANK_INT_SRC_SEL_MASK                  0x44444444UL

/* Ring Csrs offsets (most defined in common header)*/
#define ICP_DH895xCC_RING_CSR_INT_SRCSEL                    0x174
#define ICP_DH895xCC_RING_CSR_INT_SRCSEL_2                  0x178

#define ICP_DH895xCC_RING_CSR_FLAG_AND_COL_EN               0x184


/* PCIe configuration space */
#define ICP_DH895xCC_PESRAM_BAR              0
#define ICP_DH895xCC_PESRAM_BAR_SIZE         0x80000 /* (512KB) */
#define ICP_DH895xCC_PMISC_BAR               1
#define ICP_DH895xCC_PMISC_BAR_SIZE          0x40000 /* (256KB) */
#define ICP_DH895xCC_PETRINGCSR_BAR          2
#define ICP_DH895xCC_PETRINGCSR_BAR_SIZE     0x40000  /* (256KB) */
#define ICP_DH895xCC_MAX_PCI_BARS            3
#define ICP_DH895xCC_BUNDLE_SIZE             0x1000

/* eSRAM */
#define ICP_DH895xCC_SRAM_AE_ADDR            0x2000000000000ull
#define ICP_DH895xCC_SRAM_AE_SIZE            0x80000

/* Clock */
#define ICP_DH895xCC_AE_CLOCK_IN_MHZ         1066

/*Ring size in bytes which firmware supports for each sub-service*/
#define ICP_DH895xCC_FW_MSG_SIZE_SYMCY_TX    128
#define ICP_DH895xCC_FW_MSG_SIZE_SYMCY_RX    32
#define ICP_DH895xCC_FW_MSG_SIZE_ASYMCY_TX   64
#define ICP_DH895xCC_FW_MSG_SIZE_ASYMCY_RX   32
#define ICP_DH895xCC_FW_MSG_SIZE_DC_TX       128
#define ICP_DH895xCC_FW_MSG_SIZE_DC_RX       32

#define ICP_DH895xCC_FW_MSG_SIZE_ADMIN_TX    32
#define ICP_DH895xCC_FW_MSG_SIZE_ADMIN_RX    32

/* msix vectors for bundle interrupts*/
#define ICP_DH895xCC_MSIX_BANK_VECTOR_START 0
#define ICP_DH895xCC_MSIX_BANK_VECTOR_NUM   32

/*msix vector for MISC interrupts*/
#define ICP_DH895xCC_MSIX_AE_VECTOR_START   32
#define ICP_DH895xCC_MSIX_AE_VECTOR_NUM     1

/*
 * Fuse Control - Offset, constants and masks.
 */
#define ICP_DH895xCC_FUSECTL_OFFSET         0x40

#define ICP_DH895xCC_FUSECTL_SKU_MASK       0x300000
#define ICP_DH895xCC_FUSECTL_SKU_SHIFT      20
#define ICP_DH895xCC_FUSECTL_SKU_1          0x0
#define ICP_DH895xCC_FUSECTL_SKU_2          0x1
#define ICP_DH895xCC_FUSECTL_SKU_3          0x2
#define ICP_DH895xCC_FUSECTL_SKU_4          0x3


/*Documentation refers to Accelerators and CPMs interchangeably*/
#define ICP_DH895xCC_MAX_ACCELERATORS       6

/*Documentation refers to Accel Engines and MEs interchangeably*/
#define ICP_DH895xCC_MAX_ACCELENGINES       12

#define ICP_DH895xCC_ACCELERATORS_REG_OFFSET (ICP_DH895xCC_MAX_ACCELENGINES+1)
#define ICP_DH895xCC_ACCELERATORS_MASK      0x3F
#define ICP_DH895xCC_ACCELENGINES_MASK      0xFFF

/* Accelerator Functions */
#define ICP_DH895xCC_LEGFUSE_OFFSET                     0x4C
#define ICP_DH895xCC_LEGFUSE_MASK                       0x1F
#define ICP_DH895xCC_LEGFUSES_FN0BIT4                   0x04
#define ICP_DH895xCC_LEGFUSES_FN0BIT3                   0x03
#define ICP_DH895xCC_LEGFUSES_FN0BIT2                   0x02
#define ICP_DH895xCC_LEGFUSES_FN0BIT1                   0x01
#define ICP_DH895xCC_LEGFUSES_FN0BIT0                   0x00

/* PMISC BAR offsets */
#define PMISCBAROFFSET    0x3A000

/* ETR */
#define ICP_DH895xCC_ETR_MAX_BANKS                  32
#define ICP_DH895xCC_ETR_MAX_RINGS_PER_BANK         16
/*32 banks can be used for any Accelerator with arbitration enabled*/
#define ICP_DH895xCC_BANKS_PER_ACCELERATOR          ICP_DH895xCC_ETR_MAX_BANKS

#define ICP_DH895xCC_MAX_ET_RINGS   \
            (ICP_DH895xCC_ETR_MAX_BANKS * ICP_DH895xCC_ETR_MAX_RINGS_PER_BANK)
#define ICP_DH895xCC_ETR_MAX_AP_BANKS               8

/*All rings available due to arbiter*/
#define ICP_DH895xCC_MAX_NUM_RINGS_PER_ACCELERATOR  (ICP_DH895xCC_MAX_ET_RINGS)
/*
 * Interrupt Register definition
 */

/* SINTPF interrupt register. */
#define ICP_DH895xCC_SINTPF0_OFFSET             (PMISCBAROFFSET + 0x24)
#define ICP_DH895xCC_SINTPF1_OFFSET             (PMISCBAROFFSET + 0x2C)
#define ICP_DH895xCC_SINTPF1_INT_MASK           0x1

/* SMIAPF - Mask Register. */
#define ICP_DH895xCC_SMIAPF0_MASK_OFFSET        (PMISCBAROFFSET + 0x28)
#define ICP_DH895xCC_SMIAPF1_MASK_OFFSET        (PMISCBAROFFSET + 0x30)

/*
 * Default mask of the SMIAPF register.
 */
#define ICP_DH895xCC_SMIA0_MASK             (0xFFFFFFFF)
#define ICP_DH895xCC_SMIA1_MASK             (0x1)

/*
 * Admin Messages Registers
 */
#define ICP_DH895xCC_ADMINMSGUR_OFFSET       (PMISCBAROFFSET + 0x400 + 0x174)
#define ICP_DH895xCC_ADMINMSGLR_OFFSET       (PMISCBAROFFSET + 0x400 + 0x178)

/*
 * ME mailboxes base offsets
 */
#define ICP_DH895xCC_MAILBOX0_BASE_OFFSET (0x20000 + 0x970)
#define ICP_DH895xCC_MAILBOX1_BASE_OFFSET (0x20000 + 0x974)
#define ICP_DH895xCC_MAILBOX2_BASE_OFFSET (0x20000 + 0x978)
#define ICP_DH895xCC_MAILBOX3_BASE_OFFSET (0x20000 + 0x97C)
#define ICP_DH895xCC_MAILBOX_STRIDE       (0x1000)

/*
 * Reset constants and offsets
 */
#define ICP_DH895xCC_RESET_VECTOR 0x3FBCFFF
#define ICP_DH895xCC_RESET_OFFSET 0xA0C

/* This is for the version request message sent over VF2PF comms
 * it is the lowest supported driver version a VF or PF can support
 * with the current driver
 * */
#define ICP_DH895xCC_LOWESTSUPPORTED_MAJOR_VERSION 2
#define ICP_DH895xCC_LOWESTSUPPORTED_MINOR_VERSION 3
/******************************************************************************
 Change Log (Fill in whenever above changes)
 Lowest supported version changed to X In Driver version Y
 ------------------------------------------------------------------------------
 X   / Y     / Reason
 ------------------------------------------------------------------------------
 2.0 / 2.0.0 / Initial version
 2.3 / 2.3.0 / pfvfcomms _IN_USE pattern changed due to corner case.

 *****************************************************************************/

/*
 * VF-to-PF Messaging Interrupt
 */
#define ICP_DH895xCC_MAX_NUM_VF            32
#define ICP_PMISCBAR_TIMISCINTCTL          (PMISCBAROFFSET + 0x548)
#define ICP_PMISCBAR_VFTOPFINTSRC          (PMISCBAROFFSET + 0x0C)
#define ICP_PMISCBAR_VFTOPFINTMSK          (PMISCBAROFFSET + 0x1C)
#define ICP_VFTOPF_INTSOURCEMASK           0x1FFFE00
#define ICP_VFTOPF_INTSOURCEOFFSET         0x09

/* Mask VF to PF interrupts for VFs in range of 1 to 16 */
#define ICP_DH895xCC_VFTOPF_MSK_16BITS          0xFFFF
#define ICP_DH895xCC_VFTOPF_BIT_1TO16           0x09
#define ICP_DH895xCC_VFTOPF_MSK_1TO16(vfMsk)    \
    ((vfMsk & ICP_DH895xCC_VFTOPF_MSK_16BITS)   \
        << ICP_DH895xCC_VFTOPF_BIT_1TO16)

/* Mask VF to PF interrupts for VFs in range of 17 to 32 */
#define ICP_DH895xCC_VFTOPF_SHIFT_17TO32        16
#define ICP_DH895xCC_VFTOPF_MSK_17TO32(vfMsk)   \
    (vfMsk >> ICP_DH895xCC_VFTOPF_SHIFT_17TO32)

/* VF to PF interrupt bits 1 to 16 are at bit-offset 9 in errsou3 */
#define ICP_DH895xCC_VFTOPF_SRC_1TO16           \
    (0xFFFF << ICP_DH895xCC_VFTOPF_BIT_1TO16)
/* VF to PF interrupt bits 17 to 32 are at bit-offset 0 in errsou5 */
#define ICP_DH895xCC_VFTOPF_SRC_17TO32          0xFFFF

#define ICP_PMISCBAR_PF2VFDBINT_OFFSET     (PMISCBAROFFSET + 0x280)
#define ICP_PMISCBAR_VINTSOU_OFFSET        (PMISCBAROFFSET + 0x180)
#define ICP_PMISCBAR_VINTMSK_OFFSET        (PMISCBAROFFSET + 0x200)
#define ICP_PMISCBAR_VINTMSK_DEFAULT       0x02
#define ICP_PMISCBAR_VFDEVOFFSET           0x04

/* PETRING BAR offsets */
#define ICP_DH895xCC_ETRING_CSR_OFFSET      0x00

/* Ring configuration space size
 * This chunk of memory will be mapped up to userspace
 * It is 0x1000 times 32 bundles
 */
#define ICP_DH895xCC_ETRING_CSR_SIZE        0x20000

/*
 * Not including ME auto push.
 */

/* User space */
#define ICP_DH895xCC_USER_ETRING_CSR_SIZE       0x20000

/*
 * PMISC BAR region offsets and sizes
 */
#define ICP_DH895xCC_REGION_CAP_OFFSET           0x0
#define ICP_DH895xCC_REGION_CAP_SIZE             0x4000   /* (16KB) */
#define ICP_DH895xCC_REGION_SCRATCH_RAM_OFFSET   0x4000
#define ICP_DH895xCC_REGION_SCRATCH_RAM_SIZE     0x4000   /* (16KB) */
#define ICP_DH895xCC_REGION_CPMS_OFFSET
#define ICP_DH895xCC_REGION_CPMS_SIZE            0x20000  /* (128KB) */
#define ICP_DH895xCC_REGION_MES_OFFSET
#define ICP_DH895xCC_REGION_MES_SIZE             0x10000  /* (64KB) */
#define ICP_DH895xCC_REGION_CHAP_PMU_OFFSET
#define ICP_DH895xCC_REGION_CHAP_PMU_SIZE        0x2000   /* (8KB) */
#define ICP_DH895xCC_REGION_EP_MMIO_CSRS_OFFSET
#define ICP_DH895xCC_REGION_EP_MMIO_CSRS_SIZE    0x1000   /* (4KB) */
#define ICP_DH895xCC_REGION_MSIX_TABS_OFFSET
#define ICP_DH895xCC_REGION_MSIX_TABS_SIZE       0x1000   /* (4KB) */


/*
 * Offsets
 */

#define ICP_DH895xCC_PMISCBAR_AE_OFFSET               (0x20000)
#define ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET      (0x0)
#define ICP_DH895xCC_PMISCBAR_CPP_SHAC_OFFSET         (0x30000)
#define ICP_DH895xCC_PMISCBAR_TI_ERR_OFFSET           (0x3A400)
#define ICP_DH895xCC_PMISCBAR_ESRAM_ERR_OFFSET        (0x3AC00)

#define ICP_DH895xCC_SLICEPWRDOWN(idx)   (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                                          (idx * 0x4000) + 0x2C)

#define ICP_DH895xCC_MMP0_PWRUPMSK      (1<<3)
#define ICP_DH895xCC_MMP1_PWRUPMSK      (1<<4)
#define ICP_DH895xCC_MMP2_PWRUPMSK      (1<<5)
#define ICP_DH895xCC_MMP3_PWRUPMSK      (1<<6)
#define ICP_DH895xCC_MMP4_PWRUPMSK      (1<<7)
#define ICP_DH895xCC_MMP_PWRUPMSK                  \
    ~(ICP_DH895xCC_MMP0_PWRUPMSK | ICP_DH895xCC_MMP1_PWRUPMSK | \
      ICP_DH895xCC_MMP2_PWRUPMSK | ICP_DH895xCC_MMP3_PWRUPMSK | \
      ICP_DH895xCC_MMP4_PWRUPMSK)

#define ICP_DH895xCC_CTX_ENABLES(idx)    (ICP_DH895xCC_PMISCBAR_AE_OFFSET + \
                                                            (idx * 0x1000) + 0x818)
#define ICP_DH895xCC_PMISCBAR_MISC_CONTROL(idx) \
    (ICP_DH895xCC_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x960)

#define ICP_DH895xCC_ENABLE_ECC_PARITY_CORR ((1<<12) | (1<<24))
/*offset 12: CS_ECC_correct_Enable, offset 24: Parity_Enable: enable parity and
 * ECC AE fix*/

/*
 * Uncorrectable Errors 
 */

/*
 * SSM error type
 * List the various SSM based Uncorrectable Errors
 */
#define ICP_DH895xCC_SHAREDMEM_ERR 0 /* Shared Memory UErrors */
#define ICP_DH895xCC_MMP_ECC_ERR   1 /* MMP ECC UErrors       */
#define ICP_DH895xCC_PP_ERR        2 /* Push/Pull UErrors     */


/*
 * Uncorrectable errors Mask registers
 */

/* PMISCBAR Uncorrectable Error Mask Offsets */
#define ICP_DH895xCC_ERRMSK0            (PMISCBAROFFSET+0x10)
#define ICP_DH895xCC_ERRMSK1            (PMISCBAROFFSET+0x14)
#define ICP_DH895xCC_ERRMSK2            (PMISCBAROFFSET+0x18)
#define ICP_DH895xCC_ERRMSK3            (PMISCBAROFFSET+0x1C)
#define ICP_DH895xCC_ERRMSK4            (PMISCBAROFFSET+0xD4)
#define ICP_DH895xCC_ERRMSK5            (PMISCBAROFFSET+0xDC)

/* PMISCBAR Uncorrectable Error Mask 0 value */
/* Note 0<<(value) is 0, #define like that has been added for readability */
#define ICP_DH895xCC_M3UNCOR_ENABLE     (0<<25) /*  AE3 uncorrectable errors */
#define ICP_DH895xCC_M3UNCOR_DISABLE    (1<<25) /* !AE3 uncorrectable errors */
#define ICP_DH895xCC_M3COR_DISABLE      (1<<24) /* !AE3 correctable errors   */
#define ICP_DH895xCC_M2UNCOR_ENABLE     (0<<17) /*  AE2 uncorrectable errors */
#define ICP_DH895xCC_M2UNCOR_DISABLE    (1<<17) /* !AE2 uncorrectable errors */
#define ICP_DH895xCC_M2COR_DISABLE      (1<<16) /* !AE2 correctable errors   */
#define ICP_DH895xCC_M1UNCOR_ENABLE     (0<<9)  /*  AE1 uncorrectable errors */
#define ICP_DH895xCC_M1UNCOR_DISABLE    (1<<9)  /* !AE1 uncorrectable errors */
#define ICP_DH895xCC_M1COR_DISABLE      (1<<8)  /* !AE1 correctable errors   */
#define ICP_DH895xCC_M0UNCOR_ENABLE     (0<<1)  /*  AE0 uncorrectable errors */
#define ICP_DH895xCC_M0UNCOR_DISABLE    (1<<1)  /* !AE0 uncorrectable errors */
#define ICP_DH895xCC_M0COR_DISABLE      (1)     /* !AE0 correctable errors   */
#define ICP_DH895xCC_ERRMSK0_UERR                  \
    (ICP_DH895xCC_M3UNCOR_ENABLE  | ICP_DH895xCC_M2UNCOR_ENABLE  | \
     ICP_DH895xCC_M1UNCOR_ENABLE  | ICP_DH895xCC_M0UNCOR_ENABLE  | \
     ICP_DH895xCC_M3COR_DISABLE   | ICP_DH895xCC_M2COR_DISABLE   | \
     ICP_DH895xCC_M1COR_DISABLE   | ICP_DH895xCC_M0COR_DISABLE)
#define ICP_DH895xCC_ERRMSK0_MASK_FATAL_ERRS       \
    (ICP_DH895xCC_M3UNCOR_DISABLE | ICP_DH895xCC_M2UNCOR_DISABLE | \
     ICP_DH895xCC_M1UNCOR_DISABLE | ICP_DH895xCC_M0UNCOR_DISABLE)

/* PMISCBAR Uncorrectable Error Mask 1 value */
#define ICP_DH895xCC_M7UNCOR_ENABLE     (0<<25) /*  AE7 uncorrectable errors */
#define ICP_DH895xCC_M7UNCOR_DISABLE    (1<<25) /* !AE7 uncorrectable errors */
#define ICP_DH895xCC_M7COR_DISABLE      (1<<24) /* !AE7 correctable errors   */
#define ICP_DH895xCC_M6UNCOR_ENABLE     (0<<17) /*  AE6 uncorrectable errors */
#define ICP_DH895xCC_M6UNCOR_DISABLE    (1<<17) /* !AE6 uncorrectable errors */
#define ICP_DH895xCC_M6COR_DISABLE      (1<<16) /* !AE6 correctable errors   */
#define ICP_DH895xCC_M5UNCOR_ENABLE     (0<<9)  /*  AE5 uncorrectable errors */
#define ICP_DH895xCC_M5UNCOR_DISABLE    (1<<9)  /* !AE5 uncorrectable errors */
#define ICP_DH895xCC_M5COR_DISABLE      (1<<8)  /* !AE5 correctable errors   */
#define ICP_DH895xCC_M4UNCOR_ENABLE     (0<<1)  /*  AE4 uncorrectable errors */
#define ICP_DH895xCC_M4UNCOR_DISABLE    (1<<1)  /* !AE4 uncorrectable errors */
#define ICP_DH895xCC_M4COR_DISABLE      (1)     /* !AE4 correctable errors   */
#define ICP_DH895xCC_ERRMSK1_UERR                  \
    (ICP_DH895xCC_M7UNCOR_ENABLE | ICP_DH895xCC_M6UNCOR_ENABLE | \
     ICP_DH895xCC_M5UNCOR_ENABLE | ICP_DH895xCC_M4UNCOR_ENABLE | \
     ICP_DH895xCC_M7COR_DISABLE  | ICP_DH895xCC_M6COR_DISABLE  | \
     ICP_DH895xCC_M5COR_DISABLE  | ICP_DH895xCC_M4COR_DISABLE  )
#define ICP_DH895xCC_ERRMSK1_MASK_FATAL_ERRS       \
    (ICP_DH895xCC_M7UNCOR_DISABLE | ICP_DH895xCC_M6UNCOR_DISABLE | \
     ICP_DH895xCC_M5UNCOR_DISABLE | ICP_DH895xCC_M4UNCOR_DISABLE )

/* PMISCBAR Uncorrectable Error Mask 4 value */
#define ICP_DH895xCC_M11UNCOR_ENABLE   (0<<25) /*  AE11 uncorrectable errors */
#define ICP_DH895xCC_M11UNCOR_DISABLE  (1<<25) /* !AE11 uncorrectable errors */
#define ICP_DH895xCC_M11COR_DISABLE    (1<<24) /* !AE11 correctable errors   */
#define ICP_DH895xCC_M10UNCOR_ENABLE   (0<<17) /*  AE10 uncorrectable errors */
#define ICP_DH895xCC_M10UNCOR_DISABLE  (1<<17) /* !AE10 uncorrectable errors */
#define ICP_DH895xCC_M10COR_DISABLE    (1<<16) /* !AE10 correctable errors   */
#define ICP_DH895xCC_M9UNCOR_ENABLE    (0<<9)  /*  AE9  uncorrectable errors */
#define ICP_DH895xCC_M9UNCOR_DISABLE   (1<<9)  /* !AE9  uncorrectable errors */
#define ICP_DH895xCC_M9COR_DISABLE     (1<<8)  /* !AE9  correctable errors   */
#define ICP_DH895xCC_M8UNCOR_ENABLE    (0<<1)  /*  AE8  uncorrectable errors */
#define ICP_DH895xCC_M8UNCOR_DISABLE   (1<<1)  /* !AE8  uncorrectable errors */
#define ICP_DH895xCC_M8COR_DISABLE     (1)     /* !AE8  correctable errors   */
#define ICP_DH895xCC_ERRMSK4_UERR                    \
    (ICP_DH895xCC_M11UNCOR_ENABLE | ICP_DH895xCC_M10UNCOR_ENABLE  | \
     ICP_DH895xCC_M9UNCOR_ENABLE  | ICP_DH895xCC_M8UNCOR_ENABLE   | \
     ICP_DH895xCC_M11COR_DISABLE  | ICP_DH895xCC_M10COR_DISABLE   | \
     ICP_DH895xCC_M9COR_DISABLE   | ICP_DH895xCC_M8COR_DISABLE    )
#define ICP_DH895xCC_ERRMSK4_MASK_FATAL_ERRS       \
    (ICP_DH895xCC_M11UNCOR_DISABLE| ICP_DH895xCC_M10UNCOR_DISABLE | \
     ICP_DH895xCC_M9UNCOR_DISABLE | ICP_DH895xCC_M8UNCOR_DISABLE  )

/* PMISCBAR Uncorrectable Error Mask 3 value */
#define ICP_DH895xCC_UERR_ENABLE         (0<<8) /* eSRam uncorrectable errors */
#define ICP_DH895xCC_UERR_DISABLE        (1<<8) /* !eSRam uncorrectable errors*/
#define ICP_DH895xCC_CERR_DISABLE        (1<<7) /* eSRam correctable   errors */
#define ICP_DH895xCC_PPMISCERR_ENABLE    (0<<6) /* Push Pull Misc uncorrectable
                                                   errors                     */
#define ICP_DH895xCC_PPMISCERR_DISABLE   (1<<6) /* !Push Pull Misc uncorrectable
                                                  errors                      */
#define ICP_DH895xCC_RIMISC_DISABLE      (1<<5) /* !Push Pull error
                                                   detected by RI             */
#define ICP_DH895xCC_TIMISC_DISABLE      (1<<4) /* !FLR/BME Errors            */
#define ICP_DH895xCC_EMSK3_CPM1_ENABLE   (0<<3) /* CPM1 interrupts            */
#define ICP_DH895xCC_EMSK3_CPM1_DISABLE  (1<<3) /* !CPM1 interrupts           */
#define ICP_DH895xCC_EMSK3_CPM0_ENABLE   (0<<2) /* CPM0 interrupts            */
#define ICP_DH895xCC_EMSK3_CPM0_DISABLE  (1<<2) /* !CPM0 interrupts           */
#define ICP_DH895xCC_EMSK3_SHaC1_ENABLE  (0<<1) /* OR'ed ShaC Attention Signal
                                                   needed by fw debug interrup*/
#define ICP_DH895xCC_EMSK3_SHaC0_ENABLE  (0<<0) /* OR'ed ShaC data errors     */
#define ICP_DH895xCC_EMSK3_SHaC0_DISABLE (1<<0) /* !OR'ed ShaC data errors    */

#define ICP_EMSK3_DISABLE_OTHER          \
    ((1<<27) | (1<<26) | (1<<25) |(0xFFFF<<9)) /* !PMU, !CPPCMD,
                                  !VFTOPF (will be activated later if needed) */

#define ICP_DH895xCC_ERRMSK3_UERR \
    (ICP_DH895xCC_UERR_ENABLE       | ICP_DH895xCC_CERR_DISABLE      | \
     ICP_DH895xCC_PPMISCERR_ENABLE  |                                  \
     ICP_DH895xCC_EMSK3_CPM1_ENABLE | ICP_DH895xCC_EMSK3_CPM0_ENABLE | \
     ICP_DH895xCC_EMSK3_SHaC1_ENABLE| ICP_DH895xCC_EMSK3_SHaC0_ENABLE| \
     ICP_DH895xCC_RIMISC_DISABLE    | ICP_DH895xCC_TIMISC_DISABLE    | \
     ICP_EMSK3_DISABLE_OTHER)
#define ICP_DH895xCC_ERRMSK3_MASK_FATAL_ERRS \
    (ICP_DH895xCC_UERR_DISABLE        | ICP_DH895xCC_PPMISCERR_DISABLE  | \
     ICP_DH895xCC_EMSK3_CPM1_DISABLE  | ICP_DH895xCC_EMSK3_CPM0_DISABLE | \
     ICP_DH895xCC_EMSK3_SHaC0_DISABLE )

/* PMISCBAR Uncorrectable Error Mask 5 value */
#define ICP_DH895xCC_EMSK5_CPM5_ENABLE   (0<<19)     /* CPM5 interrupts */
#define ICP_DH895xCC_EMSK5_CPM5_DISABLE  (1<<19)     /* !CPM5 interrupts */
#define ICP_DH895xCC_EMSK5_CPM4_ENABLE   (0<<18)     /* CPM4 interrupts */
#define ICP_DH895xCC_EMSK5_CPM4_DISABLE  (1<<18)     /* !CPM4 interrupts */
#define ICP_DH895xCC_EMSK5_CPM3_ENABLE   (0<<17)     /* CPM3 interrupts */
#define ICP_DH895xCC_EMSK5_CPM3_DISABLE  (1<<17)     /* !CPM3 interrupts */
#define ICP_DH895xCC_EMSK5_CPM2_ENABLE   (0<<16)     /* CPM2 interrupts */
#define ICP_DH895xCC_EMSK5_CPM2_DISABLE  (1<<16)     /* !CPM2 interrupts */
#define ICP_DH895xCC_VF2PF3217           (0xFFFF)    /* VFtoPF32 17 MSI */

#define ICP_DH895xCC_ERRMSK5_UERR \
    (ICP_DH895xCC_EMSK5_CPM5_ENABLE | ICP_DH895xCC_EMSK5_CPM4_ENABLE | \
     ICP_DH895xCC_EMSK5_CPM3_ENABLE | ICP_DH895xCC_EMSK5_CPM2_ENABLE | \
     ICP_DH895xCC_VF2PF3217)
#define ICP_DH895xCC_ERRMSK5_MASK_FATAL_ERRS \
    (ICP_DH895xCC_EMSK5_CPM5_DISABLE | ICP_DH895xCC_EMSK5_CPM4_DISABLE | \
     ICP_DH895xCC_EMSK5_CPM3_DISABLE | ICP_DH895xCC_EMSK5_CPM2_DISABLE )

/* AE ECC */
#define ICP_DH895xCC_CTX_ENABLES(idx)    (ICP_DH895xCC_PMISCBAR_AE_OFFSET + \
                                                            (idx * 0x1000) + 0x818)
#define ICP_DH895xCC_ENABLE_ECC_ERR      (1<<28) /* AEx Ecc error on */

/* CPM Uncorrectable Errors */
#define ICP_DH895xCC_INTMASKSSM(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                                                    (idx * 0x4000)) 

#define ICP_DH895xCC_PPERR_MASK          (0<<12) /* Push/Pull Uncorrectable
                                                    interrupt                     */
#define ICP_DH895xCC_MMP4_CIMASK         (1<<11) /* !MMP4 Correctable  interrupt  */
#define ICP_DH895xCC_MMP4_UIMASK         (0<<10) /*  MMP4 Uncorrectable interrupt */
#define ICP_DH895xCC_MMP3_CIMASK         (1<<9)  /* !MMP3 Correctable  interrupt  */
#define ICP_DH895xCC_MMP3_UIMASK         (0<<8)  /*  MMP3 Uncorrectable interrupt */
#define ICP_DH895xCC_MMP2_CIMASK         (1<<7)  /* !MMP2 Correctable  interrupt  */
#define ICP_DH895xCC_MMP2_UIMASK         (0<<6)  /*  MMP2 Uncorrectable interrupt */
#define ICP_DH895xCC_MMP1_CIMASK         (1<<5)  /* !MMP1 Correctable  interrupt  */
#define ICP_DH895xCC_MMP1_UIMASK         (0<<4)  /*  MMP1 Uncorrectable interrupt */
#define ICP_DH895xCC_MMP0_CIMASK         (1<<3)  /* !MMP0 Correctable  interrupt  */
#define ICP_DH895xCC_MMP0_UIMASK         (0<<2)  /*  MMP0 Uncorrectable interrupt */
#define ICP_DH895xCC_SH_CIMASK           (1<<1)  /* !Shared Memory correctable
                                                    interrupt                     */
#define ICP_DH895xCC_SH_UIMASK           (0<<0)  /* Shared Memory Uncorrectable
                                                    interrupt                     */

#define ICP_DH895xCC_INTMASKSSM_UERR \
        (ICP_DH895xCC_MMP4_CIMASK | ICP_DH895xCC_MMP4_UIMASK | \
         ICP_DH895xCC_MMP3_CIMASK | ICP_DH895xCC_MMP3_UIMASK | \
         ICP_DH895xCC_MMP2_CIMASK | ICP_DH895xCC_MMP2_UIMASK | \
         ICP_DH895xCC_MMP1_CIMASK | ICP_DH895xCC_MMP1_UIMASK | \
         ICP_DH895xCC_MMP0_CIMASK | ICP_DH895xCC_MMP0_UIMASK | \
         ICP_DH895xCC_SH_CIMASK   | ICP_DH895xCC_SH_UIMASK)  

/* SHAC Uncorrectable Errors */
#define ICP_DH895xCC_CPP_SHAC_ERR_CTRL   (ICP_DH895xCC_PMISCBAR_CPP_SHAC_OFFSET \
                                                                           + 0xC00)

/* Push/Pull control reg */
#define ICP_DH895xCC_SHAC_INTEN          (1<<1) /* Enable interrupt               */
#define ICP_DH895xCC_SHAC_PPERREN        (1)    /* Enable push/pull errors
                                                   detection                      */
#define ICP_DH895xCC_CPP_SHAC_UE   \
        (ICP_DH895xCC_SHAC_INTEN | ICP_DH895xCC_SHAC_PPERREN)

/* eSRAM Uncorrectable Errors */
#define ICP_DH895xCC_ESRAMUERR           (ICP_DH895xCC_PMISCBAR_ESRAM_ERR_OFFSET \
                                                                             + 0x4)
#define ICP_DH895xCC_ESRAMCERR           (ICP_DH895xCC_PMISCBAR_ESRAM_ERR_OFFSET \
                                                                             + 0x0)

/* eSRAM UERROR Reg */
#define ICP_DH895xCC_ESRAM_INTEN         (1<<17) /* Enable interrupts */
#define ICP_DH895xCC_ESRAM_EN            (1<<3)  /* Enable Error Detection */
#define ICP_DH895xCC_ESRAM_CEN           (1<<3)  /*Enable Error correction*/
#define ICP_DH895xCC_ESRAM_UERR          \
        (ICP_DH895xCC_ESRAM_INTEN | ICP_DH895xCC_ESRAM_EN)
#define ICP_DH895xCC_ESRAM_CERR          (ICP_DH895xCC_ESRAM_CEN)

/* Push/Pull/Misc Errors */
#define ICP_DH895xCC_CPPMEMTGTERR        (ICP_DH895xCC_PMISCBAR_ESRAM_ERR_OFFSET \
                                                                           + 0x10)
#define ICP_DH895xCC_TGT_PPEREN          (1<<3) /* Enable Error Detection */
#define ICP_DH895xCC_TGT_INTEN           (1<<2) /* Enable Interrupts */
#define ICP_DH895xCC_TGT_UERR            \
        (ICP_DH895xCC_TGT_PPEREN | ICP_DH895xCC_TGT_INTEN) 

/* PMISCBAR Uncorrectable Error */
#define ICP_DH895xCC_ERRSOU0              (PMISCBAROFFSET)
#define ICP_DH895xCC_ERRSOU1              (PMISCBAROFFSET + 0x04)
#define ICP_DH895xCC_ERRSOU2              (PMISCBAROFFSET + 0x08)
#define ICP_DH895xCC_ERRSOU3              (PMISCBAROFFSET + 0x0C)
#define ICP_DH895xCC_ERRSOU4              (PMISCBAROFFSET + 0x0D0)
#define ICP_DH895xCC_ERRSOU5              (PMISCBAROFFSET + 0x0D8)

/* errsou 0 */
#define ICP_DH895xCC_M3UNCOR_MASK         (1<<25) /* AE3 uncorrectable errors  */
#define ICP_DH895xCC_M2UNCOR_MASK         (1<<17) /* AE2 uncorrectable errors  */
#define ICP_DH895xCC_M1UNCOR_MASK         (1<<9)  /* AE1 uncorrectable errors  */
#define ICP_DH895xCC_M0UNCOR_MASK         (1<<1)  /* AE0 uncorrectable errors  */

/* errsou 1 */
#define ICP_DH895xCC_M7UNCOR_MASK         (1<<25) /* AE7 uncorrectable errors  */
#define ICP_DH895xCC_M6UNCOR_MASK         (1<<17) /* AE6 uncorrectable errors  */
#define ICP_DH895xCC_M5UNCOR_MASK         (1<<9)  /* AE5 uncorrectable errors  */
#define ICP_DH895xCC_M4UNCOR_MASK         (1<<1)  /* AE4 uncorrectable errors  */

/* errsou 3 */
#define ICP_DH895xCC_UERR_MASK            (1<<8)  /* eSRam uncorrectable errors */
#define ICP_DH895xCC_PPMISCERR_MASK       (1<<6)  /* Push Pull Misc
                                                     uncorrectable errors       */
#define ICP_DH895xCC_EMSK3_CPM1_MASK      (1<<3)  /* CPM1 interrupts            */
#define ICP_DH895xCC_EMSK3_CPM0_MASK      (1<<2)  /* CPM0 interrupts            */
#define ICP_DH895xCC_EMSK3_SHaC0_MASK     (1)     /* OR'ed ShaC data errors     */

/* errsou 4 */
#define ICP_DH895xCC_M11UNCOR_MASK        (1<<25) /* AE11 uncorrectable errors  */
#define ICP_DH895xCC_M10UNCOR_MASK        (1<<17) /* AE10 uncorrectable errors  */
#define ICP_DH895xCC_M9UNCOR_MASK         (1<<9)  /* AE9  uncorrectable errors  */
#define ICP_DH895xCC_M8UNCOR_MASK         (1<<1)  /* AE8  uncorrectable errors  */

/* errsou 5 */
#define ICP_DH895xCC_EMSK5_CPM5_MASK      (1<<19) /* CPM5 interrupts            */
#define ICP_DH895xCC_EMSK5_CPM4_MASK      (1<<18) /* CPM4 interrupts            */
#define ICP_DH895xCC_EMSK5_CPM3_MASK      (1<<17) /* CPM3 interrupts            */
#define ICP_DH895xCC_EMSK5_CPM2_MASK      (1<<16) /* CPM2 interrupts            */

/* AE Status */
#define ICP_DH895xCC_EPERRLOG              (PMISCBAROFFSET + 0x20)
#define ICP_DH895xCC_GET_EPERRLOG(val, me) (val & (1 << (1 + (2 * me))))

/* Me Control */
#define ICP_DH895xCC_USTORE_ERROR_STATUS(idx) \
    (ICP_DH895xCC_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x80C)
#define ICP_DH895xCC_USTORE_GET_UE(val)    ((val & (1<<31)) >> 31)
#define ICP_DH895xCC_USTORE_UADDR(val)      (val & 0x3FFF) /* Faulty address */
#define ICP_DH895xCC_USTORE_GET_SYN(val)   ((val & (0x7F << 20)) >> 20)

/* AE Parity Error */
#define ICP_DH895xCC_REG_ERROR_STATUS(idx) \
    (ICP_DH895xCC_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x830)

/* CPM */
#define ICP_DH895xCC_INTSTATSSM(idx)      (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                                                  (idx * 0x4000) + 0x4) 
#define ICP_DH895xCC_INTSTATSSM_PPERR     (1<<12) /* Push Pull Error
                                                     -> log in PPERRSSM    */
#define ICP_DH895xCC_INTSTATSSM_MMP4      (1<<10) /* MMP UERR -> log in
                                                     UERRSSMMMPx4          */
#define ICP_DH895xCC_INTSTATSSM_MMP3      (1<<8)  /* MMP UERR -> log in
                                                     UERRSSMMMPx3          */
#define ICP_DH895xCC_INTSTATSSM_MMP2      (1<<6)  /* MMP UERR -> log in
                                                     UERRSSMMMPx2          */
#define ICP_DH895xCC_INTSTATSSM_MMP1      (1<<4)  /* MMP UERR -> log in
                                                     UERRSSMMMPx1          */
#define ICP_DH895xCC_INTSTATSSM_MMP0      (1<<2)  /* MMP UERR -> log in
                                                     UERRSSMMMPx0          */
#define ICP_DH895xCC_INTSTATSSM_SH        (1)     /* Shared Memory UERR ->
                                                     log in UERRSSMSHx     */

/* MMP */
#define ICP_DH895xCC_UERRSSMMMP0(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x388)     /* CPMx - MMP0 */
#define ICP_DH895xCC_CERRSSMMMP0(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x380)     /* CPMx - MMP0 */
#define ICP_DH895xCC_UERRSSMMMP1(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x1388)    /* CPMx - MMP1 */
#define ICP_DH895xCC_CERRSSMMMP1(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x1380)    /* CPMx - MMP1 */
#define ICP_DH895xCC_UERRSSMMMP2(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x2388)    /* CPMx - MMP2 */
#define ICP_DH895xCC_CERRSSMMMP2(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x2380)    /* CPMx - MMP2 */
#define ICP_DH895xCC_UERRSSMMMP3(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x3388)    /* CPMx - MMP3 */
#define ICP_DH895xCC_CERRSSMMMP3(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x3380)    /* CPMx - MMP3 */
#define ICP_DH895xCC_UERRSSMMMP4(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0xB88)     /* CPMx - MMP4 */
#define ICP_DH895xCC_CERRSSMMMP4(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0xB80)     /* CPMx - MMP4 */

#define ICP_DH895xCC_UERRSSMMMP_EN        (1<<3)   /* 1 = Enable logging    */
#define ICP_DH895xCC_CERRSSMMMP_EN        (1<<3)   /* 1 = Enable correction */
#define ICP_DH895xCC_UERRSSMMMP_UERR      (1)      /* 1 = Uerr              */

#define ICP_DH895xCC_UERRSSMMMP0AD(idx)   (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x38C)     /* CPMx - MMP0 */
#define ICP_DH895xCC_UERRSSMMMP1AD(idx)   (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x138C)    /* CPMx - MMP1 */
#define ICP_DH895xCC_UERRSSMMMP2AD(idx)   (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x238C)    /* CPMx - MMP2 */
#define ICP_DH895xCC_UERRSSMMMP3AD(idx)   (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x338C)    /* CPMx - MMP3 */
#define ICP_DH895xCC_UERRSSMMMP4AD(idx)   (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0xB8C)     /* CPMx - MMP4 */

#define ICP_DH895xCC_UERRSSMMMPAD_ADDR    (0xFFFF) /*Address Mask*/

/* Shared memory */
#define ICP_DH895xCC_UERRSSMSH(idx)       (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x18) /* UERRSSMSH[0:5] */
#define ICP_DH895xCC_CERRSSMSH(idx)       (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x10) /* CERRSSMSH[0:5] */
#define ICP_DH895xCC_UERRSSMSH_EN         (1<<3) /* Enable logging      */
#define ICP_DH895xCC_CERRSSMSH_EN         (1<<3) /* Enable correction   */
#define ICP_DH895xCC_UERRSSMSH_UERR       (1)    /* Uncorrectable error */
#define ICP_DH895xCC_UERRSSMSH_R          (1<<1) /* On read operation   */
#define ICP_DH895xCC_UERRSSMSH_W          (1<<2) /* On Write operation  */
#define ICP_DH895xCC_UERRSSMSHAD(idx)     (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x1C)
#define ICP_DH895xCC_UERRSSMSHAD_ADDR     (0xFFFF) /* Address Mask */

/* Push/Pull */
#define ICP_DH895xCC_PPERR(idx)           (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0x8)
#define ICP_DH895xCC_PPERR_EN             (1<<2) /* Enable Err logging  */
#define ICP_DH895xCC_PPERR_MERR           (1<<1) /* Multiple Errors     */
#define ICP_DH895xCC_PPERR_PPERR          (1)    /* Uncorrectable Error */
#define ICP_DH895xCC_PERRID(idx)          (ICP_DH895xCC_PMISCBAR_CONFIG_REGS_OFFSET + \
                                          (idx * 0x4000) + 0xC)
#define ICP_DH895xCC_CPP_SHAC_ERR_STATUS  \
    (ICP_DH895xCC_PMISCBAR_CPP_SHAC_OFFSET + 0xC04) /* Push/Pull status reg */
#define ICP_DH895xCC_CPP_SHAC_ERR_PPID       (ICP_DH895xCC_PMISCBAR_CPP_SHAC_OFFSET + 0xC0C)
#define ICP_DH895xCC_CPP_SHAC_GET_ERR(val)   (val & 1) /* Push/Pull status Error(s) occured */
#define ICP_DH895xCC_CPP_SHAC_GET_TYP(val)   ((val & (1<<3)) >> 3) /* 0-push(rd) 1-pull(wr) */
#define ICP_DH895xCC_CPP_SHAC_GET_ERTYP(val) ((val & (0x3<<4)) >> 4)
#define ICP_DH895xCC_CPP_SHAC_GET_INT(val)   ((val & (1<<2)) >> 2)

/* eSRAM */
#define ICP_DH895xCC_ESRAMUERR_UERR       (1)    /* eSRAM UERR          */
#define ICP_DH895xCC_ESRAMUERR_R          (1<<1) /* eSRAM UERR on read  */
#define ICP_DH895xCC_ESRAMUERR_W          (1<<2) /* eSRAM UERR on write */
#define ICP_DH895xCC_ESRAMUERRAD          (ICP_DH895xCC_PMISCBAR_ESRAM_ERR_OFFSET+0xC)
#define ICP_DH895xCC_ESRAMUERR_GET_ERRTYPE(val) \
                                          ((val & (0xF<<4)) >> 4)
#define ICP_DH895xCC_CPPMEMTGTERR_ERR     (1)    /* Misc Memory Target Error occured */
#define ICP_DH895xCC_CPPMEMTGTERR_MERR    (1<<1) /* Multiple Errors occurred         */
#define ICP_DH895xCC_ERRPPID              (ICP_DH895xCC_PMISCBAR_ESRAM_ERR_OFFSET+0x14)
#define ICP_DH895xCC_CPPMEMTGTERR_ERRTYP(val) \
    ((val & (0xF<<16)) >> 16)

/* 32 bits push/pull UERR id */
#define ICP_DH895xCC_TICPPINTCTL          (ICP_DH895xCC_PMISCBAR_TI_ERR_OFFSET+0x138) /* TI CPP control */
#define ICP_DH895xCC_TICPP_PUSH           (1)    /* push error log */
#define ICP_DH895xCC_TICPP_PULL           (1<<1) /* pull error log */
#define ICP_DH895xCC_TICPP_EN             \
    (ICP_DH895xCC_TICPP_PUSH | ICP_DH895xCC_TICPP_PULL) /* Enable Error log */
/* Captures the CPP error detected on the TI CPP bus */
#define ICP_DH895xCC_TICPPINTSTS          (ICP_DH895xCC_PMISCBAR_TI_ERR_OFFSET+0x13C)
/* Log the id of the push error */
#define ICP_DH895xCC_TIERRPUSHID          (ICP_DH895xCC_PMISCBAR_TI_ERR_OFFSET+0x140)
/* Log the id of the pull error */
#define ICP_DH895xCC_TIERRPULLID          (ICP_DH895xCC_PMISCBAR_TI_ERR_OFFSET+0x144)


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
 * access_csr_int_flag_and_col
 *
 * Enable or disable Intx, Flag, and Coelesced interrupts.
 * Disable is done by setting 'value' input to zero.
 */
static inline void
access_csr_int_flag_and_col(Cpa32U *csr_base_addr, Cpa32U bank_offset,
                            Cpa32U value, CpaBoolean write)
{
    if(CPA_TRUE == write)
    {
        ICP_ADF_CSR_WR(csr_base_addr, bank_offset +
                       ICP_DH895xCC_RING_CSR_FLAG_AND_COL_EN, value);
    }
    else
    {
        /*This read is done to make sure that a write to the INTx CSR
          has been propagated to that register (otherwise we might get
          spurious interrupts).*/
        ICP_ADF_CSR_RD(csr_base_addr, bank_offset +
                       ICP_DH895xCC_RING_CSR_FLAG_AND_COL_EN);
    }
    
}

/*
 * write_csr_srcsel
 *
 * Configure source for flag interrupts. All are configured 'Empty going
 * False'. See IaIntSrcSel CSR definition for more information.
 * These are default values and may be changed at ring configuration time
 * based on the ring size and whether it is Tx or Rx.
 *
 * Calling this function loads the initial values into the SrcSel CSRs.
 */
static inline void
write_csr_srcsel(Cpa32U *csr_base_addr, Cpa32U bank_offset)
{
    ICP_ADF_CSR_WR(csr_base_addr, bank_offset +
                    ICP_DH895xCC_RING_CSR_INT_SRCSEL,
                    ICP_DH895xCC_BANK_INT_SRC_SEL_MASK);

    ICP_ADF_CSR_WR(csr_base_addr, bank_offset +
                    ICP_DH895xCC_RING_CSR_INT_SRCSEL_2,
                    ICP_DH895xCC_BANK_INT_SRC_SEL_MASK);
}

/*
 * irq_ae_source
 * Function returns the value of AE interrupt source
 */
static inline Cpa32U irq_ae_source(Cpa32U *misc_addr)
{
    return (ICP_DH895xCC_SINTPF1_INT_MASK &
                (ICP_ADF_CSR_RD(misc_addr, ICP_DH895xCC_SINTPF1_OFFSET)));
}

#endif /* ADF_PLATFORM_DH895XCC_H */
