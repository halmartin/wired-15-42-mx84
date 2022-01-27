/**
 **************************************************************************
 * @file hal_ae.h
 *
 * @description
 *      This is the header file for Hardware Abstraction Layer -- AE Registers
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

/**
 *****************************************************************************
 * @file hal_ae.h
 * 
 * @ingroup icp_hal
 *
 * @description
 *      This header file contains the global macros used by HAL 
 *
 *****************************************************************************/

#ifndef __HAL_AE_H
#define __HAL_AE_H

#include "core_io.h"
#include "icp_firml_handle.h"

#define GBE_MASK 0xFC000000

typedef enum{
    MISC_CONTROL          = 0x04,      /**< misc_control global cap CSR offset */
    ICP_RESET             = 0x0c,      /**< icp_reset global cap CSR offset */
    ICP_GLOBAL_CLK_ENABLE = 0x50       /**< ICP_GLOBAL_CLK_ENABLE global cap CSR offset */
}Hal_Global_CSR_T;

typedef enum{
	FCU_CONTROL           = 0x8c0,      /**< FCU Control Register */
	FCU_STATUS            = 0x8c4,      /**< FCU Status Register */
	FCU_STATUS1           = 0x8c8,      /**< FCU Status Register */
	FCU_DRAM_ADDR_LO      = 0x8cc,      /**< FCU Descriptor Register */
	FCU_DRAM_ADDR_HI      = 0x8d0,      /**< FCU Descriptor Register */
	FCU_RAMBASE_ADDR_HI   = 0x8d4,      /**< FCU RAM Base Address Register */
    FCU_RAMBASE_ADDR_LO   = 0x8d8       /**< FCU RAM Base Address Register */
}Hal_FCU_CSR_T;

/** @{ */
/* Enumeration for AE local control and status register offset */
typedef enum{
    USTORE_ADDRESS                  = 0x000,  /**< Control Store Address Register */
    USTORE_DATA_LOWER               = 0x004,  /**< Control Store Data Lower Register */
    USTORE_DATA_UPPER               = 0x008,  /**< Control Store Data Upper Register */    
    USTORE_ERROR_STATUS             = 0x00c,  /**< Control Store Error Status Register */    
    ALU_OUT                         = 0x010,  /**< Arithmetic Logic Unit Output Register */
    CTX_ARB_CNTL                    = 0x014,  /**< Context Arbiter Control Register */
    CTX_ENABLES                     = 0x018,  /**< Context Enables Register */
    CC_ENABLE                       = 0x01c,  /**< Condition Code Enable Register */
    CSR_CTX_POINTER                 = 0x020,  /**< CSR Context Pointer Register */

    REG_ERROR_STATUS                = 0x030,  /**< Register Error Status Register */

    CTX_STS_INDIRECT                = 0x040,  /**< Indirect Context Status Register */
    ACTIVE_CTX_STATUS               = 0x044,  /**< Active Context Status Register */
    CTX_SIG_EVENTS_INDIRECT         = 0x048,  /**< Indirect Context Signal Events Register */
    CTX_SIG_EVENTS_ACTIVE           = 0x04c,  /**< Active Context Signal Events Register */
    CTX_WAKEUP_EVENTS_INDIRECT      = 0x050,  /**< Indirect Context Wakeup Events Register */
    CTX_WAKEUP_EVENTS_ACTIVE        = 0x054,  /**< Active Context Wakeup Events Register */
    CTX_FUTURE_COUNT_INDIRECT       = 0x058,  /**< Indirect Context Future Count Register */
    CTX_FUTURE_COUNT_ACTIVE         = 0x05c,  /**< Active Context Future Count Register */
    LM_ADDR_0_INDIRECT              = 0x060,  /**< Indirect Local Memory Address 0 Register */
    LM_ADDR_0_ACTIVE                = 0x064,  /**< Active Local Memory Address 0 Register */
    LM_ADDR_1_INDIRECT              = 0x068,  /**< Indirect Local Memory Address 1 Register */
    LM_ADDR_1_ACTIVE                = 0x06c,  /**< Active Local Memory Address 1 Register */
    BYTE_INDEX                      = 0x070,  /**< Byte Index Register */

    INDIRECT_LM_ADDR_0_BYTE_INDEX   = 0x0e0,  /**< Indirect Local Memory Address 0 Byte Index Register */
    ACTIVE_LM_ADDR_0_BYTE_INDEX     = 0x0e4,  /**< Active Local Memory Address 0 Byte Index Register */
    INDIRECT_LM_ADDR_1_BYTE_INDEX   = 0x0e8,  /**< Indirect Local Memory Address 1 Byte Index Register */
    ACTIVE_LM_ADDR_1_BYTE_INDEX     = 0x0ec,  /**< Active Local Memory Address 1 Byte Index Register */

    T_INDEX_BYTE_INDEX              = 0x0f4,  /**< Transfer Index Concatenated with Byte Index Register */
    T_INDEX                         = 0x074,  /**< Transfer Index Register */

    FUTURE_COUNT_SIGNAL_INDIRECT    = 0x078,  /**< Indirect Future Count Signal Signal Register */
    FUTURE_COUNT_SIGNAL_ACTIVE      = 0x07c,  /**< Active Context Future Count Register */

    NN_PUT                          = 0x080,  /**< Next Neighbor Put Register */
    NN_GET                          = 0x084,  /**< Next Neighbor Get Register */

    TIMESTAMP_LOW                   = 0x0c0,  /**< Timestamp Low Register  */
    TIMESTAMP_HIGH                  = 0x0c4,  /**< Timestamp High Register */

    NEXT_NEIGHBOR_SIGNAL            = 0x100,  /**< Next Neighbor Signal Register */
    PREV_NEIGHBOR_SIGNAL            = 0x104,  /**< Previous Neighbor Signal Register */
    SAME_AE_SIGNAL                  = 0x108,  /**< Same AccelEngine Signal Register */

    CRC_REMAINDER                   = 0x140,  /**< Cyclic Redundancy Check Remainder Register */
    PROFILE_COUNT                   = 0x144,  /**< Profile Count Register */
    PSEUDO_RANDOM_NUMBER            = 0x148,  /**< Pseudorandom Number Register */

    SIGNATURE_ENABLE                = 0x150,  /**< Signature Enable Register */

    AE_MISC_CONTROL                 = 0x160,  /**< Miscellaneous Control Register */

    USTORE_ADDRESS1                 = 0x158,  /**< Control Store Address 1 Register */

    LOCAL_CSR_STATUS                = 0x180,  /**< Local CSR Status Register */
    NULL_CSR                        = 0x3fc   /**< NULL Register */
}Hal_Ae_CSR_T;
/** @} */

/* NOTE: The numeric values below must remain constant, 
    because they are used in HdwCsrCfg.dat. */
typedef enum{
    HAL_CSR_CAP = 0,                          /**< Global Control CSR */
    HAL_CSR_ET_RING = 1,                      /**< ET ring CSR */
    HAL_CSR_HASH = 2,                         /**< HASH CSR */    
    HAL_CSR_QAT0 = 3,                         /**< QAT0 CSR */
    HAL_CSR_QAT1 = 4,                         /**< QAT1 CSR */    
    HAL_CSR_QAT2 = 5,                         /**< QAT2 CSR */    
    HAL_CSR_QAT3 = 6,                         /**< QAT3 CSR */    
    HAL_CSR_QAT4 = 7,                         /**< QAT4 CSR */    
    HAL_CSR_QAT5 = 8,                         /**< QAT5 CSR */    
    HAL_CSR_GBE0 = 9,                         /**< GBE0 CSR */
    HAL_CSR_GBE1 = 10,                        /**< GBE1 CSR */
    HAL_CSR_GBE2 = 11,                        /**< GBE2 CSR */
    HAL_CSR_GBE3 = 12,                        /**< GBE3 CSR */
    HAL_CSR_PCI_HEAD = 13,                    /**< PCI header CSR */    
    HAL_CSR_EP = 14,                          /**< Endpoint CSR */
    HAL_CSR_END
}Hal_CSR_T;

#define GLOBAL_CSR      0xa00			/* global CSR offset */
#define SET_CAP_CSR(handle, csr, val) WRITE_LWORD((handle->Hal_cap_global_ctl_csr_virtAddr + (csr)), (val))	\
									/**< write cap csr */
#define GET_CAP_CSR(handle, csr) READ_LWORD((handle->Hal_cap_global_ctl_csr_virtAddr + (csr)))	\
									/**< read cap csr */
#define SET_GLB_CSR(handle, csr, val) SET_CAP_CSR(handle, ((csr) + GLOBAL_CSR), (val))	/**< write cap global csr */
#define GET_GLB_CSR(handle, csr) GET_CAP_CSR(handle, (GLOBAL_CSR + (csr)))		/**< read cap global csr */

enum {
    FCU_CTRL_CMD_NOOP  = 0,
    FCU_CTRL_CMD_AUTH  = 0x1,
    FCU_CTRL_CMD_LOAD  = 0x2,
    FCU_CTRL_CMD_START = 0x3
};

enum{
    FCU_STS_NO_STS    = 0,
    FCU_STS_VERI_DONE = 1,
    FCU_STS_LOAD_DONE = 2,
    FCU_STS_VERI_FAIL = 3,
    FCU_STS_LOAD_FAIL = 4,
    FCU_STS_BUSY      = 5,
    FCU_STS_EXECUTING = 6
};

#define FCU_CTRL_AE_POS     0x8
#define FCU_AUTH_STS_MASK   0x7
#define FCU_CTRL_CMD_MASK   0xf
#define FCU_STS_DONE_POS    0x9
#define FCU_STS_DONE_MASK   0x1
#define FCU_LOADED_AE_POS   0x16
#define FCU_LOADED_AE_MASK  0x3ff

#define FW_AUTH_WAIT_PERIOD 100
#define FW_AUTH_MAX_RETRY   0x20

#define MAX_SHRAM_USED 64 /* In Longwords. At most 256 Longwords are allowed in one function call per hardware limitation */

/** @{ */
/* CSR: PRODUCT_ID masks - this is fake */
#define PID_MINOR_REV_BITPOS            0    /**< product ID minor revision bit postion */    
#define PID_MAJOR_REV_BITPOS            4    /**< product ID major revision bit postion */    
#define PID_MINOR_PROD_TYPE_BITPOS      8    /**< product ID minor product type bit postion */    
#define PID_MAJOR_PROD_TYPE_BITPOS      16   /**< product ID major product type  bit postion */    

#define PID_MINOR_REV                   (0xf)  /**< product ID minor revision */    
#define PID_MAJOR_REV                   (0xf << PID_MAJOR_REV_BITPOS)  /**< product ID major revision */    
#define PID_MINOR_PROD_TYPE             (0xff << PID_MINOR_PROD_TYPE_BITPOS)   /**< product ID minor product type */    
#define PID_MAJOR_PROD_TYPE             (0xff << PID_MAJOR_PROD_TYPE_BITPOS)   /**< product ID major product type */    
/** @} */

/** @{ */
/* CSR: MISC_CONTROL masks */
#define MC_TIMESTAMP_ENABLE_BITPOS      7    /**< TIMESTAMP enable bit postion */    
#define MC_TIMESTAMP_ENABLE     (0x1 << MC_TIMESTAMP_ENABLE_BITPOS)  /**< TIMESTAMP enable */    
/** @} */

/** @{ */
#define AENC_ALIGN_SHIFT           12   /**< DRAM address shift */    

#define OS_TYPE_LINUX           		0x0
#define OS_TYPE_FREEBSD        		0x1
#define OS_TYPE_WINXPE         		0x2
#define OS_TYPE_RESERVED      		0x03
#define OS_TYPE_BIT_POS         		0xc
#define VERSION_INFO                 		0x00000021
#define VERSION_INFO_BIT_POS   		0x2
#define VERSION_INFO_MASK 			0x00000ffc
/** @} */

/** @{ */
/* CSR: Ustore_Addr bit positions and masks */
#define UA_ECS_BITPOS               31                               /**< Bit Position of ECS in Ustore_Addr register */
#define UA_ECS                      (0x1 << UA_ECS_BITPOS)           /**< ECS mask in Ustore_Addr register */
/** @} */

/** @{ */
/* CSR: Active_CTX_Status masks */
#define ACS_ACNO_BITPOS             0                                /**< Bit Position of ACNO in Active_CTX_Status register*/
#define ACS_AE_NUMBER_BITPOS        3                                /**< Bit Position of AE Number in Active_CTX_Status register */
#define ACS_ACTXPC_BITPOS           8                                /**< Bit Position of ACTXPC in Active_CTX_Status register */
#define ACS_ABO_BITPOS              31                               /**< Bit Position of ABO in Active_CTX_Status register */

#define ACS_ACNO                    (0x7)                            /**< ACNO mask in Active_CTX_Status register */
#define ACS_AE_NUMBER               (0x1f << ACS_AE_NUMBER_BITPOS)   /**< AE Number mask in Active_CTX_Status register */
#define ACS_ACTXPC                  (0x1ffff << ACS_ACTXPC_BITPOS)    /**< ACTXPC mask in Active_CTX_Status register */
#define ACS_ABO                     (0x1 << ACS_ABO_BITPOS)          /**< ABO mask in Active_CTX_Status register */
/** @} */

/** @{ */
/* CSR: Indirect_CTX_Status masks */
#define ICS_CTX_PC_BITPOS           0                                /**< Bit Position of CTX PC in Indirect_CTX_Status register */
#define ICS_RR_BITPOS               31                               /**< Bit Position of RR in Indirect_CTX_Status register */
#define ICS_CTX_PC                  (0x1ffff)                         /**< CTX PC mask in Indirect_CTX_Status register */
#define ICS_RR                      (0x1 << ICS_RR_BITPOS)           /**< RR mask in Indirect_CTX_Status register */
/** @} */

/** @{ */
/* CSR: CTX_ARB_CNTL masks */
#define CAC_NTCX_BITPOS             0                                /**< Bit Position of NCTX in CTX_ARB_CNTL register */
#define CAC_PTCX_BITPOS             4                                /**< Bit Position of PCTX in CTX_ARB_CNTL register */
#define CAC_NTCX                    (0x7)                            /**< NCTX mask in CTX_ARB_CNTL register */
#define CAC_PTCX                    (0x7 << CAC_PTCX_BITPOS)         /**< PCTX mask in CTX_ARB_CNTL register */
/** @} */

/** @{ */
/* CSR: CTX_Enable masks */
#define CE_ENABLE_BITPOS            (0x8)                            /**< Bit Position of CTX ENABLES in CTX_Enable register */
#define CE_LMADDR_0_GLOBAL_BITPOS   16                               /**< Bit Position of LM Addr 0 Global in CTX_Enable register */
#define CE_LMADDR_1_GLOBAL_BITPOS   17                               /**< Bit Position of LM Addr 1 Global in CTX_Enable register */
#define CE_NN_RING_EMPTY_BITPOS     18                               /**< Bit Position of NN RING EMPTY in CTX_Enable register */
#define CE_NN_MODE_BITPOS           20                               /**< Bit Position of NN MODE in CTX_Enable register */
#define CE_REG_PAR_ERR_BITPOS       25                               /**< Bit Position of Reg Parity Err in CTX_Enable register */
#define CE_BREAKPOINT_BITPOS        27                               /**< Bit Position of Breakpoint in CTX_Enable register */
#define CE_CNTL_STORE_PARITY_ENABLE_BITPOS 28                        /**< Bit Position of CTL STR ECC Enable in CTX_Enable register */
#define CE_CNTL_STORE_PARITY_ERROR_BITPOS 29                         /**< Bit Position of CTL STR ECC Err in CTX_Enable register */
#define CE_INUSE_CONTEXTS_BITPOS    31                               /**< Bit Position of Inuse CTX in CTX_Enable register */

#define CE_ENABLE                   (0xff << CE_ENABLE_BITPOS)                   /**< CTX ENABLES mask in CTX_Enable register */
#define CE_LMADDR_0_GLOBAL          (0x1 << CE_LMADDR_0_GLOBAL_BITPOS)           /**< LM Addr 0 Global mask in CTX_Enable register */
#define CE_LMADDR_1_GLOBAL          (0x1 << CE_LMADDR_1_GLOBAL_BITPOS)           /**< LM Addr 1 Global mask in CTX_Enable register */
#define CE_NN_RING_EMPTY            (0x3 << CE_NN_RING_EMPTY_BITPOS)             /**< NN RING EMPTY mask in CTX_Enable register */
#define CE_NN_MODE                  (0x1 << CE_NN_MODE_BITPOS)                   /**< NN MODE mask in CTX_Enable register */
#define CE_BREAKPOINT               (0x1 << CE_BREAKPOINT_BITPOS)                /**< Breakpoint mask in CTX_Enable register */
#define CE_CNTL_STORE_PARITY_ENABLE (0x1 << CE_CNTL_STORE_PARITY_ENABLE_BITPOS)  /**< CTL STR ECC Enable mask in CTX_Enable register */
#define CE_CNTL_STORE_PARITY_ERROR  (0x1 << CE_CNTL_STORE_PARITY_ERROR_BITPOS)   /**< CTL STR ECC Err mask in CTX_Enable register */
#define CE_INUSE_CONTEXTS           (0x1 << CE_INUSE_CONTEXTS_BITPOS)            /**< Inuse CTX mask in CTX_Enable register */
/** @} */

/** @{ */
/* CSR: CC_Enable masks */
#define CCE_CU_BITPOS               13                                           /**< Bit Position of CCCU in CC_Enable register */
#define CCE_CU                      (0x1 << CCE_CU_BITPOS)                       /**< CCCU mask in CC_Enable register */
/** @} */

/** @{ */
/* CSR: CSR_CTX_Pointer masks */
#define CCP_CONTEXT                 (0x7UL)                                      /**< CTX mask in CSR_CTX_Pointer register */
/** @} */

/** @{ */
/* CSR: Active/Indirect_CTX_Sig_Events masks */
#define XCSE_VOLUNTARY_BITPOS       0                                            /**< Bit Position of VOL in Active/Indirect_CTX_Sig_Events register */
#define XCSE_SIGNALS_BITPOS         1                                            /**< Bit Position of SIGNALS in Active/Indirect_CTX_Sig_Events register */

#define XCSE_VOLUNTARY              (0x1)                                        /**< VOL mask in Active/Indirect_CTX_Sig_Events register */
#define XCSE_SIGNALS                (0x7fff << XCSE_SIGNALS_BITPOS)              /**< SIGNALS mask in Active/Indirect_CTX_Sig_Events register */
/** @} */

/** @{ */
/* CSR: Active/Indirect_CTX_Wakeup_Events masks */
#define XCWE_VOLUNTARY_BITPOS       0                                            /**< Bit Position of VOL in Active/Indirect_CTX_Wakeup_Events register */
#define XCWE_WAKEUP_EVENTS_BITPOS   1                                            /**< Bit Position of WAKEUP EVENTS in Active/Indirect_CTX_Wakeup_Events register */
#define XCWE_OR_WAKEUP_EVENTS_BITPOS 16                                          /**< Bit Position of OR WAKEUP EVENTS in Active/Indirect_CTX_Wakeup_Events register */

#define XCWE_VOLUNTARY              (0x1)                                        /**< VOL mask in Active/Indirect_CTX_Wakeup_Events register */
#define XCWE_WAKEUP_EVENTS          (0x7fff << XCWE_WAKEUP_EVENTS_BITPOS)        /**< WAKEUP EVENTS mask in Active/Indirect_CTX_Wakeup_Events register */
#define XCWE_OR_WAKEUP_EVENTS       (0x1 << XCWE_OR_WAKEUP_EVENTS_BITPOS)     /**< OR WAKEUP EVENTS mask in Active/Indirect_CTX_Wakeup_Events register */
/** @} */

/** @{ */
/* CSR: Active/Indirect_LM_Addr_x masks */
#define XLA_LM_ADDR_BITPOS          2                                            /**< Bit Position of LM Addr in Active/Indirect_LM_Addr_x register */
#define XLA_LM_ADDR                 (0x3ff << XLA_LM_ADDR_BITPOS)                /**< LM Addr mask in Active/Indirect_LM_Addr_x register */
/** @} */

/** @{ */
/* CSR: T_Index masks */
#define TI_XFER_REG_INDEX_BITPOS    2                                            /**< Bit Position of XFER Index in T_Index register */
#define TI_XFER_REG_INDEX           (0xff << TI_XFER_REG_INDEX_BITPOS)           /**< XFER Index mask in T_Index register */
/** @} */

/** @{ */
/* CSR: Local_CSR_Status */
#define LCS_STATUS_BITPOS           0                                            /**< Bit Position of STATUS in Local_CSR_Status register */
#define LCS_STATUS                  (0x1)                                        /**< STATUS mask in Local_CSR_Status register */

#define CTX                 0x7                                                  /**< CTX mask in ACTIVE_CTX_STS register */
#define CTX_ACTIVE          0x80000000                                           /**< CTX Active mask in ACTIVE_CTX_STS register */
#define CTX_ACTIVE_SHIFT    31                                                   /**< Bit Position of CTX Active in ACTIVE_CTX_STS register */
#define CTX_ACTIVE_PC_MASK  0x001FFFF00                                          /**< CTX Active PC mask in ACTIVE_CTX_STS register */
#define CTX_ACTIVE_PC_SHIFT 8                                                    /**< Bit Position of CTX Active PC in ACTIVE_CTX_STS register */
#define CTX_CC_MASK         0x00002000                                           /**< CTX CCCE mask in CC_ENABLE register */
#define CTX_CC_SHIFT        8                                                    /**< Right-Shift Bits of CTX CCCE in CC_ENABLE register to format dbgAe_AeStatus_T.enables */
#define CTX_PARITY_MASK     0x10000000                                           /**< CTX PARITY Enable mask in CTX_ENABLES register */
#define CTX_PARITY_SHIFT    24                                                   /**< Right-Shift Bits of CTX PARITY Enable in CTX_ENABLES register to format dbgAe_AeStatus_T.enables */
#define CTX_ctxMode_MASK    0x80000000                                           /**< CTX Mode mask in CTX_ENABLES register */
#define CTX_ctxMode_SHIFT   28                                                   /**< Right-Shift Bits of CTX Mode in CTX_ENABLES register to format dbgAe_AeStatus_T.enables */
#define CTX_nnMode_MASK     0x00100000                                           /**< NN Mode mask in CTX_ENABLES register */
#define CTX_nnMode_SHIFT    18                                                   /**< Right-Shift Bits of NN Mode in CTX_ENABLES register to format dbgAe_AeStatus_T.enables */
#define CTX_LMADDR1_MASK    0x00020000                                           /**< LM ADDR1 mask in CTX_ENABLES register */
#define CTX_LMADDR1_SHIFT   16                                                   /**< Right-Shift Bits of LM ADDR1 in CTX_ENABLES register to format dbgAe_AeStatus_T.enables */
#define CTX_LMADDR0_MASK    0x00010000                                           /**< LM ADDR0 mask in CTX_ENABLES register */
#define CTX_LMADDR0_SHIFT   16                                                   /**< Right-Shift Bits of LM ADDR0 in CTX_ENABLES register to format dbgAe_AeStatus_T.enables */
#define CTXs_ENABLED        0xFF00                                               /**< CTX ENABLEDS mask in CTX_ENABLES register */
/** @} */

/** @{ */
/* CSR: Ae_Misc_Control */
#define MMC_SHARE_CS_BITPOS         2                                            /**< Bit Position of Share CS in Ae_Misc_Control register */
#define MMC_CS_ECC_CORRECT_BITPOS   12                                           /**< Bit Position of CS ECC CORRECT Enable in Ae_Misc_Control register */
#define MMC_CS_RELOAD_BITPOS        20                                           /**< Bit Position of CS RELOAD in Ae_Misc_Control register */
#define MMC_ONE_CTX_RELOAD_BITPOS   22                                           /**< Bit Position of ONE CTX RELOAD in Ae_Misc_Control register */
#define MMC_FORCE_BAD_PARITY_BITPOS 23                                           /**< Bit Position of FORCE BAD PARITY in Ae_Misc_Control register */
#define MMC_PARITY_ENABLE_BITPOS    24                                           /**< Bit Position of PARITY ENABLE in Ae_Misc_Control register */
/** @} */

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the base address of AE CSR for the specified AE
 *      
 * @param ae - IN Specifies AE number
 *
 * @retval - the base address of AE CSR
 * 
 * 
 *****************************************************************************/
#define AE_CSR(handle, ae) (handle->Hal_cap_ae_local_csr_virtAddr + (((ae) & handle->halHandle->AeMask) << LOCAL_CSR_SHIFT))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get The specified CSR address for the specified AE
 *      
 * @param ae - IN Specifies AE number
 * @param csr - IN Specifies AE CSR address offset
 *
 * @retval - AE CSR address
 * 
 * 
 *****************************************************************************/
#define AE_CSR_ADDR(handle, ae, csr) (AE_CSR(handle, ae)+(0x3ff & (csr)))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write data to the specified CSR for the specified AE
 *      
 * @param ae - IN Specifies AE number
 * @param csr - IN Specifies AE CSR address offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
#define SET_AE_CSR(handle, ae, csr, val) (WRITE_LWORD(AE_CSR_ADDR((handle),(ae),(csr)),(val)))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read data from the specified CSR for the specified AE
 *      
 * @param ae - IN Specifies AE number
 * @param csr - IN Specifies AE CSR address offset
 *
 * @retval - read value
 * 
 * 
 *****************************************************************************/
#define GET_AE_CSR(handle, ae, csr) (READ_LWORD(AE_CSR_ADDR((handle),(ae),(csr))))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the base address of AE Transfer Register for the specified AE
 *      
 * @param ae - IN Specifies AE number
 *
 * @retval - The base address of AE Transfer Register
 * 
 * 
 *****************************************************************************/
#define AE_XFER(handle, ae) (handle->Hal_cap_ae_xfer_csr_virtAddr + (((ae) & handle->halHandle->AeMask) << XFER_CSR_SHIFT))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Get the specified Transfer Register address for the specified AE
 *      
 * @param ae - IN Specifies AE number
 * @param reg - IN Specifies AE Transfer Register address offset
 *
 * @retval - AE Transfer Register address
 * 
 * 
 *****************************************************************************/
#define AE_XFER_ADDR(handle, ae, reg) (AE_XFER(handle, ae) + (((reg) & 0xff) << 2))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Write data to the specified Transfer Register for the specified AE
 *      
 * @param ae - IN Specifies AE number
 * @param reg - IN Specifies AE Transfer Register address offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
#define SET_AE_XFER(handle, ae, reg, val)  (WRITE_LWORD(AE_XFER_ADDR((handle), (ae),(reg)), (val)))
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Read data from the specified Transfer Register for the specified AE
 *      
 * @param ae - IN Specifies AE number
 * @param reg - IN Specifies AE Transfer Register address offset
 *
 * @retval - read value
 * 
 * 
 *****************************************************************************/
#define GET_AE_XFER(handle, ae, reg)       (READ_LWORD(AE_XFER_ADDR((handle),(ae),(reg))))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword data to DRAM0 location.
 *      
 * @param addr - IN Specifies dram0 address offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

#define DRAM_WRITE(handle, addr, val) WRITE_LWORD_CACHED(((addr)) + handle->Hal_dram_ch0_virtAddr, (val))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword data from DRAM0 location.
 *      
 * @param addr - IN Specifies dram0 address offset
 *
 * @retval - read data
 * 
 * 
 *****************************************************************************/

#define DRAM_READ(handle, addr) READ_LWORD_CACHED(((addr)) + handle->Hal_dram_ch0_virtAddr)

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword data to DRAM location.
 *      
 * @param chan - IN Specifies dram channel. 
 * @param addr - IN Specifies dram address offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

#define DRAM_WRITE_XA(handle, chan, addr, val) {\
    if(chan==1) {WRITE_LWORD_CACHED((addr) + handle->Hal_dram_ch1_virtAddr, (val));} \
    else {WRITE_LWORD_CACHED((addr) + handle->Hal_dram_ch0_virtAddr, (val));}}

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword data to DRAM0 location.
 *      
 * @param addr - IN Specifies dram0 address offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

#define DRAM_WRITE_CH0(handle, addr, val) WRITE_LWORD_CACHED(((addr)) + handle->Hal_dram_ch0_virtAddr, (val))
    
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword data to DRAM1 location.
 *      
 * @param addr - IN Specifies dram1 address offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

#define DRAM_WRITE_CH1(handle, addr, val) WRITE_LWORD_CACHED(((addr)) + handle->Hal_dram_ch1_virtAddr, (val))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword data from DRAM0 location.
 *      
 * @param addr - IN Specifies dram0 address offset
 *
 * @retval - read data
 * 
 * 
 *****************************************************************************/

#define DRAM_READ_CH0(handle, addr) READ_LWORD_CACHED((addr) + handle->Hal_dram_ch0_virtAddr)

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword data from DRAM1 location.
 *      
 * @param addr - IN Specifies dram1 address offset
 *
 * @retval - read data
 * 
 * 
 *****************************************************************************/
    
#define DRAM_READ_CH1(handle, addr) READ_LWORD_CACHED((addr) + handle->Hal_dram_ch1_virtAddr)
    
/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword data to SRAM location.
 *      
 * @param addr - IN Specifies SRAM address offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/

#define SRAM_WRITE(handle, addr, val) (WRITE_LWORD((addr) + handle->Hal_sram_virtAddr, (val)))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword data from SRAM location.
 *      
 * @param addr - IN Specified SRAM address offset
 *
 * @retval - read data
 * 
 * 
 *****************************************************************************/

#define SRAM_READ(handle, addr)        (READ_LWORD((addr) + handle->Hal_sram_virtAddr))

#define SCRATCH_SIZE       0x4000         /**< SCRATCH memory size */
#define SCRATCH_CONFIG_CSR 0x800          /**< SCRATCH CSR offset */ 

/** @{ */
/* internal macros to access SCRATCH CSR */
#define SCRATCH_CSR(handle, csr, ring) ((handle->Hal_cap_global_ctl_csr_virtAddr + SCRATCH_CONFIG_CSR + ((csr) + ((0xf & (ring)) << 4))))
#define SET_SCRATCH_CSR(handle, csr, ring, val) WRITE_LWORD(SCRATCH_CSR((handle),(csr),(ring)), (val))
#define GET_SCRATCH_CSR(handle, csr, ring) READ_LWORD(SCRATCH_CSR((handle), (csr), (ring)))
/** @} */

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Writes a longword data to SCRATCH location.
 *      
 * @param addr - IN Specifies SCRATCH address offset
 * @param val - IN Specifies value to write
 *
 * @retval - none
 * 
 * 
 *****************************************************************************/
#define SCRATCH_WRITE(handle, addr, val) WRITE_LWORD((addr) + handle->Hal_scratch_rd_wr_swap_virtAddr, (val))

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *      Reads a longword data from SCRATCH location.
 *      
 * @param addr - IN Specified SRAM SCRATCH offset
 *
 * @retval - read data
 * 
 * 
 *****************************************************************************/

#define SCRATCH_READ(handle, addr) READ_LWORD((addr) + handle->Hal_scratch_rd_wr_swap_virtAddr)

#endif          /* __HAL_AE_H */
