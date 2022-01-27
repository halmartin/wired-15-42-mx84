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
 * @file adf_platform_c2xxx.h
 *
 * @description
 *      This file contains the platform specific macros for C2XXX processor
 *
 *****************************************************************************/
#ifndef ADF_PLATFORM_C2XXX_H
#define ADF_PLATFORM_C2XXX_H

/*****************************************************************************
 * Define Constants and Macros
 *****************************************************************************/

/* PCIe configuration space */
#define ICP_C2XXX_PMISC_BAR               1
#define ICP_C2XXX_PMISC_BAR_SIZE          0x20000 /* (128KB) */
#define ICP_C2XXX_PETRINGCSR_BAR          2
#define ICP_C2XXX_PETRINGCSR_BAR_SIZE     0x4000  /* (16KB) */
#define ICP_C2XXX_MAX_PCI_BARS            3

/* Clock */
#define ICP_C2XXX_AE_CLOCK_IN_MHZ         800

/* ETR */
#define ICP_C2XXX_ETR_MAX_BANKS           8
#define ICP_C2XXX_BANKS_PER_ACCELERATOR   8
#define ICP_C2XXX_ETR_MAX_RINGS_PER_BANK  16
#define ICP_MAX_ET_RINGS                  \
    (ICP_C2XXX_ETR_MAX_BANKS * ICP_C2XXX_ETR_MAX_RINGS_PER_BANK)
#define ICP_C2XXX_ETR_MAX_AP_BANKS        4

/* msix vector */
#define ICP_C2XXX_MSIX_BANK_VECTOR_START 0
#define ICP_C2XXX_MSIX_BANK_VECTOR_NUM   8
#define ICP_C2XXX_MSIX_AE_VECTOR_START   16
#define ICP_C2XXX_MSIX_AE_VECTOR_NUM     1

/* Fuse Control */
#define ICP_C2XXX_FUSECTL_OFFSET         0x40
#define ICP_C2XXX_MAX_ACCELERATORS       1
#define ICP_C2XXX_MAX_ACCELENGINES       2
#define ICP_C2XXX_ACCELERATORS_MASK      0x1
#define ICP_C2XXX_ACCELENGINES_MASK      0x3

/* Accelerator Functions */
#define ICP_C2XXX_PKE_DISABLE_BIT        0x6
#define ICP_C2XXX_ATH_DISABLE_BIT        0x5
#define ICP_C2XXX_CPH_DISABLE_BIT        0x4
#define ICP_C2XXX_LOW_SKU_BIT            0x3
#define ICP_C2XXX_MID_SKU_BIT            0x2
#define ICP_C2XXX_AE1_DISABLE_BIT        0x1

/* PMISC BAR offsets */
#define PMISCBAROFFSET    0x1A000

/* Interrupt */
#define ICP_C2XXX_SINTPF                PMISCBAROFFSET + 0x24
#define ICP_C2XXX_SMIAOF                PMISCBAROFFSET + 0x28
#define ICP_C2XXX_BUNDLES_IRQ_MASK      0xFF
#define ICP_C2XXX_AE_IRQ_MASK           0x10000
#define ICP_C2XXX_SMIA_MASK             \
    (ICP_C2XXX_BUNDLES_IRQ_MASK | ICP_C2XXX_AE_IRQ_MASK)

/* VF-to-PF Messaging Interrupt */
#define ICP_C2XXX_MAX_NUM_VF               8
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
#define ICP_C2XXX_ETRING_CSR_OFFSET     0x00

/* Ring configuration space size
 * This chunk of memory will be mapped up to userspace
 * It is 0x200 times 16 bundles
 */
#define ICP_C2XXX_ETRING_CSR_SIZE       0x2000

/* Autopush CSR Offsets */
#define ICP_RING_CSR_AP_NF_MASK            0x2000
#define ICP_RING_CSR_AP_NF_DEST            0x2020
#define ICP_RING_CSR_AP_NE_MASK            0x2040
#define ICP_RING_CSR_AP_NE_DEST            0x2060
#define ICP_RING_CSR_AP_DELAY              0x2080

/* Autopush CSR Byte Offset */
#define AP_BANK_CSR_BYTE_OFFSET            4

/* user space */
#define ICP_C2XXX_USER_ETRING_CSR_SIZE   0x2000

#define SYS_PAGE_SIZE_4k                   0x0
#define SYS_PAGE_SIZE_8k                   0x1
#define SYS_PAGE_SIZE_64k                  0x2
#define SYS_PAGE_SIZE_256k                 0x4
#define SYS_PAGE_SIZE_1M                   0x8
#define SYS_PAGE_SIZE_4M                   0x10

/*
 * Offsets
 */
#define ICP_C2XXX_RESET_OFFSET                  (0xA0C)
#define ICP_C2XXX_RESET_VECTOR                  (0x14D800)
#define ICP_C2XXX_PMISCBAR_AE_OFFSET            (0x10000)
#define ICP_C2XXX_PMISCBAR_ACCEL_OFFSET         (0x8000)
#define ICP_C2XXX_PMISCBAR_CPP_CAP_OFFSET       (0xC00)

/*
 * SSM error type
 * List the various SSM based Uncorrectable Errors
 */
#define ICP_C2XXX_SHAREDMEM_T             0    /* Shared Memory UErrors */
#define ICP_C2XXX_MMP_T                   1    /* MMP ECC UErrors */
#define ICP_C2XXX_PPERR_T                 2    /* Push/Pull UErrors */


/* List all the AEs */
#define ICP_C2XXX_AE0                     0
#define ICP_C2XXX_AE1                     1

/* MMP Slices */
#define ICP_C2XXX_SLICE0                  0    /* MMP Slice 0 */
#define ICP_C2XXX_SLICE1                  1    /* MMP Slice 1 */
/*
 * Uncorrectable errors Mask registers
 */
/* PMISCBAR Uncorrectable Error Mask */
#define ICP_C2XXX_ERRMSK0                 (PMISCBAROFFSET + 0x10)
#define ICP_C2XXX_ERRMSK2                 (PMISCBAROFFSET + 0x18)
#define ICP_C2XXX_ERRMSK3                 (PMISCBAROFFSET + 0x1C)
/*PMISCBAR Uncorrectable Error Mask value*/
#define ICP_C2XXX_AE1UNCOR                (0<<9)  /*AE1 uncorrectable errors*/
#define ICP_C2XXX_AE1COR                  (1<<8)  /*!AE1 correctable errors*/
#define ICP_C2XXX_AE0UNCOR                (0<<1)  /*AE0 uncorrectable errors*/
#define ICP_C2XXX_AE0COR                  (1<<0)  /*!AE0 correctable errors*/
#define ICP_C2XXX_ERRMSK0_UERR                 \
    (ICP_C2XXX_AE1UNCOR | ICP_C2XXX_AE0UNCOR | \
    ICP_C2XXX_AE1COR    | ICP_C2XXX_AE0COR)

#define ICP_C2XXX_PPMISCERR               (0<<6) /*Push Pull Misc uncor err*/
#define ICP_C2XXX_EMSK3_ACCEL0            (0<<2) /*Accelerator-0 interrupts*/
#define ICP_C2XXX_EMSK3_CAP1              (0<<1) /*OR'ed CAP Attention Signal
                                             needed by fw debug interrupt*/
#define ICP_C2XXX_EMSK3_CAP0              (0<<0) /*OR'ed CAP data errors*/
#define ICP_C2XXX_TIMISC                  (1<<4) /*!FLR/BME Errors*/
#define ICP_C2XXX_RIMISC                  (1<<5) /*!Push Pull error -RI*/
#define ICP_C2XXX_EMSK3_DISABLE_OTHER     ((1<<27) | (0x7F<<9))
/*!CPPCMD,!VFTOPF (!VFTOPF will be activated later if needed)*/

#define ICP_C2XXX_ERRMSK3_UERR            \
    (ICP_C2XXX_PPMISCERR   | ICP_C2XXX_EMSK3_ACCEL0                    | \
     ICP_C2XXX_EMSK3_CAP0  | ICP_C2XXX_EMSK3_CAP1   | ICP_C2XXX_TIMISC | \
     ICP_C2XXX_EMSK3_CAP1  | ICP_C2XXX_EMSK3_DISABLE_OTHER)

/*AE ECC*/
#define ICP_C2XXX_PMISCBAR_CTX_ENABLES(idx) \
    (ICP_C2XXX_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x818)
#define ICP_C2XXX_ENABLE_ECC_ERR          (1<<28) /*AEx Ecc error on*/

/*AEs(idx) misc control register*/
#define ICP_C2XXX_PMISCBAR_MISC_CONTROL(idx) \
    (ICP_C2XXX_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x960)

#define ICP_C2XXX_ENABLE_ECC_PARITY_CORR ((1<<12) | (1<<24))
/*offset 12: CS_ECC_correct_Enable, offset 24: Parity_Enable: enable parity and
 * ECC AE fix*/

/*Accelerator Uncorrectable Errors*/
#define ICP_C2XXX_INTMASKSSM              (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET)

#define ICP_C2XXX_PPERR_MASK              (0<<6) /*Push/Pull Uncorrectable
                                                 interrupt*/
#define ICP_C2XXX_MMP1_CIMASK             (1<<5) /*!MMP1 cor interrupt*/
#define ICP_C2XXX_MMP1_UIMASK             (0<<4) /*MMP1 Uncor interrupt*/
#define ICP_C2XXX_MMP0_CIMASK             (1<<3) /*!MMP0 cor interrupt*/
#define ICP_C2XXX_MMP0_UIMASK             (0<<2) /*MMP0 Uncor interrupt*/
#define ICP_C2XXX_SH_CIMASK               (1<<1) /*!Shared Memory correctable
                                                 interrupt*/
#define ICP_C2XXX_SH_UIMASK               (0<<0) /*Shared Memory Uncorrectable
                                                 interrupt*/
#define ICP_C2XXX_INTMASKSSM_UERR         \
    (ICP_C2XXX_MMP0_UIMASK | ICP_C2XXX_MMP1_UIMASK | ICP_C2XXX_MMP0_CIMASK | \
    ICP_C2XXX_MMP1_CIMASK  | ICP_C2XXX_SH_CIMASK   | ICP_C2XXX_SH_UIMASK   | \
    ICP_C2XXX_PPERR_MASK)

/*CPP Uncorrectable Errors*/
#define ICP_C2XXX_CPP_CAP_ERR_CTRL        (ICP_C2XXX_PMISCBAR_CPP_CAP_OFFSET)

/*Push/Pull control reg*/
#define ICP_C2XXX_CPP_INTEN               (1<<1) /*Enable interrupt*/
#define ICP_C2XXX_CPP_PPERREN             (1<<0) /*Enable push/pull errors
                                                  detection*/
#define ICP_C2XXX_CPP_UE                  \
    (ICP_C2XXX_CPP_INTEN | ICP_C2XXX_CPP_PPERREN)

/*Push/Pull Errors*/
#define ICP_C2XXX_TGT_PPEREN              (1<<3) /*Enable Error Detection*/
#define ICP_C2XXX_TGT_INTEN               (1<<2) /*Enable Interrupts*/
#define ICP_C2XXX_TGT_UERR                \
    (ICP_C2XXX_TGT_PPEREN | ICP_C2XXX_TGT_INTEN)

/*
 * Status/Log Register
 */

/*PMISCBAR Uncorrectable Error*/
#define ICP_C2XXX_ERRSOU0                 (PMISCBAROFFSET)
#define ICP_C2XXX_ERRSOU2                 (PMISCBAROFFSET + 0x08)
#define ICP_C2XXX_ERRSOU3                 (PMISCBAROFFSET + 0x0C)
/*errsou 0*/
#define ICP_C2XXX_M3UNCOR_MASK            (1<<25) /*AE3 uncorrectable errors*/
#define ICP_C2XXX_M2UNCOR_MASK            (1<<17) /*AE2 uncorrectable errors*/
#define ICP_C2XXX_M1UNCOR_MASK            (1<<9)  /*AE1 uncorrectable errors*/
#define ICP_C2XXX_M0UNCOR_MASK            (1<<1)  /*AE0 uncorrectable errors*/
/*errsou 3*/
#define ICP_C2XXX_UERR_MASK               (1<<8)  /*eSRam uncorr errors*/
#define ICP_C2XXX_PPMISCERR_MASK          (1<<6)  /*Push Pull Misc
                                                   uncorrectable errors*/
#define ICP_C2XXX_EMSK3_ACCEL_MASK        (1<<2)  /*Accelerator0 interrupts*/
#define ICP_C2XXX_EMSK3_CAP_MASK          (1<<0)  /*OR'ed CAP data errors*/

/*AE Status*/
#define ICP_C2XXX_EPERRLOG                (PMISCBAROFFSET + 0x20)
#define ICP_C2XXX_GET_EPERRLOG(val, ae)   \
    (val & (1 <<(1 + (2 * ae))))

/*AE Control*/
#define ICP_C2XXX_USTORE_ERROR_STATUS(idx) \
    (ICP_C2XXX_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x80C)
#define ICP_C2XXX_USTORE_UADDR(val)       (val & 0x3FFF) /* Faulty address*/
#define ICP_C2XXX_USTORE_GET_SYN(val)     ((val & (0x7F << 20)) >> 20)
/*Syndrom*/

/*AE Parity Error*/
#define ICP_C2XXX_REG_ERROR_STATUS(idx)   \
        (ICP_C2XXX_PMISCBAR_AE_OFFSET + (idx * 0x1000) + 0x830)

#define ICP_C2XXX_REG_GET_REG_AD(val)     (val & 0xFFFF)
/*Faulty Register or local mem*/
#define ICP_C2XXX_REG_TYPE(val)           ((val & (0x3<<15)) >> 15)

/*32bits reg for Accelerator 0*/
#define ICP_C2XXX_INTSTATSSM              \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x4)

#define ICP_C2XXX_INTSTATSSM_PPERR        (1<<6) /*Push/Pull Err
                                                   ->log in PPERRSSM*/
#define ICP_C2XXX_INTSTATSSM_MMP1         (1<<4) /*MMP UERR -> log in
                                                   UERRSSMMMPx0*/
#define ICP_C2XXX_INTSTATSSM_MMP0         (1<<2) /*MMP UERR -> log in
                                                   UERRSSMMMPx1*/
#define ICP_C2XXX_INTSTATSSM_SH           (1)    /*Shared Memory UERR ->
                                                 log in UERRSSMSHx*/
/* MMP0-Accelerator(0) */
#define ICP_C2XXX_UERRSSMMMP0             \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x388)
#define ICP_C2XXX_CERRSSMMMP0             \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x380)
/* MMP1-Accelerator(0) */
#define ICP_C2XXX_UERRSSMMMP1             \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x1388)
#define ICP_C2XXX_CERRSSMMMP1             \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x1380)

#define ICP_C2XXX_CERRSSMMMP_EN           (1<<3) /*1 = Enable correction*/
#define ICP_C2XXX_UERRSSMMMP_EN           (1<<3) /*1 = Enable logging*/
#define ICP_C2XXX_UERRSSMMMP_UERR         (1)    /*1 = Uerr*/
#define ICP_C2XXX_UERRSSMMMP_ERRTYPE(val) \
    ((val & (0xF<<4)) >> 4)
#define ICP_C2XXX_UERRSSMMMPAD0           \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x38C)  /*MMP0 address ACCEL(idx)*/
#define ICP_C2XXX_UERRSSMMMPAD1           \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x138C) /*MMP1 address ACCEL(idx)*/
#define ICP_C2XXX_UERRSSMMMPAD_ADDR       (0xFFFF) /*Address Mask*/

/*Shared memory*/
#define ICP_C2XXX_UERRSSMSH               \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x18)
#define ICP_C2XXX_CERRSSMSH      \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x10)
#define ICP_C2XXX_CERRSSMSH_EN            (1<<3) /*Enable Correction*/
#define ICP_C2XXX_UERRSSMSH_EN            (1<<3) /*Enable logging*/
#define ICP_C2XXX_UERRSSMSH_UERR          (1)    /*Uncorrectable error*/
#define ICP_C2XXX_UERRSSMSH_R             (1<<1) /*On read operation*/
#define ICP_C2XXX_UERRSSMSH_W             (1<<2) /*On Write operation*/
#define ICP_C2XXX_UERRSSMSH_ERRTYPE(val)  \
    ((val & (0xF<<4)) >> 4)

/* Shared Ram status Accelrator 0,1 */
#define ICP_C2XXX_UERRSSMSHAD             \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x1C)
#define ICP_C2XXX_UERRSSMSHAD_ADDR        (0xFFFF) /*Address Mask*/

/* Push/Pull */
#define ICP_C2XXX_PPERR                   \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0x8)
/* push/pull status accelerator 0 */
#define ICP_C2XXX_PPERR_EN                (1<<2) /* Enable Err logging */
#define ICP_C2XXX_PPERR_MERR              (1<<1) /* Multiple Errors */
#define ICP_C2XXX_PPERR_PPERR             (1)    /* Uncorrectable Error */

#define ICP_C2XXX_PPERR_TYPE(val)         ((val & (0xF<<4)) >> 4)
/*Push/Pull faulty ID accel 0*/
#define ICP_C2XXX_PPERRID                 \
    (ICP_C2XXX_PMISCBAR_ACCEL_OFFSET + 0xC)

#define ICP_C2XXX_CPP_ERR_STATUS          \
    (ICP_C2XXX_PMISCBAR_CPP_CAP_OFFSET + 0x4) /*Push/Pull status reg*/
#define ICP_C2XXX_CPP_GET_TYP(val)        \
    ((val & (1<<3)) >> 3) /*Error type 0-push(read) 1-pull(write)*/
#define ICP_C2XXX_CPP_GET_ERTYP(val)      ((val & (0x3<<4)) >> 4)

/*Push/Pull status interrupt occurred*/
#define ICP_C2XXX_CPP_SHAC_GET_INT(val)   ((val & (1<<2)) >> 2)

/*Push/Pull faulty ID(32b) reg*/
#define ICP_C2XXX_CPP_SHAC_ERR_PPID       \
    (ICP_C2XXX_PMISCBAR_CPP_CAP_OFFSET + 0xC)

/*32 bits push/pull UERR id*/
#define ICP_C2XXX_TICPPINTCTL             (0x1A400 + 0x138) /*TI CPP control*/
#define ICP_C2XXX_TICPP_PUSH              (1)    /*push error log*/
#define ICP_C2XXX_TICPP_PULL              (1<<1) /*pull error log*/
#define ICP_C2XXX_TICPP_EN                (ICP_C2XXX_TICPP_PUSH | \
        ICP_C2XXX_TICPP_PULL)             /*Enable Error log*/
#define ICP_C2XXX_TICPPINTSTS             (0x1A400 + 0x13C) /*captures the
                                                           CPP error detected
                                                           on the TI CPP bus*/

#define ICP_C2XXX_TIERRPUSHID             (0x1A400 + 0x140) /*log the id of the
                                                           push error*/
#define ICP_C2XXX_TIERRPULLID             (0x1A400 + 0x144) /*log the id of the
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
    return (void*)(UARCH_INT)
        accel_dev->pciAccelDev.pciBars[pmisc_bar_id].virtAddr;
}

/*
 * irq_get_bank_number
 * Function returns the interrupt source's bank number
 */
static inline Cpa32U irq_get_bank_number(Cpa32U sint)
{
    Cpa32U bank_number = 0;
    bank_number = sint & ICP_C2XXX_BUNDLES_IRQ_MASK;
    return bank_number;
}

/*
 * irq_ae_source
 * Function returns the value of AE interrupt source
 */
static inline Cpa32U irq_ae_source(Cpa32U sint)
{
    Cpa32U ae_irq = 0;
    ae_irq = sint & ICP_C2XXX_AE_IRQ_MASK;
    return ae_irq;
}

#endif /* ADF_PLATFORM_C2XXX_H */

