/**
 **************************************************************************
 * @file icp_firml_interface.h
 *
 * @description
 *      This is the header file for AE Loader
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
 
#ifndef __ICP_FIRML_HANDLE_H__
#define __ICP_FIRML_HANDLE_H__

#include "icptype.h"
#include "ae_constants.h"
#include "core_platform.h"
#include "hal_ssu.h"

#define ACCELCOMP_NUM_AES    0x8
#define ACCELCOMP_ALL_AE_MASK  0xFF
#define ACCELCOMP_NUM_QATS   0x2
#define ACCELCOMP_ALL_QAT_MASK 0x3
#define ACCELCOMP_AE_CLK       933
#define ACCELCOMP_NUM_MMPS   0x2

#define ACCELCOMP_C_NUM_AES    12
#define ACCELCOMP_C_ALL_AE_MASK  0xFFF
#define ACCELCOMP_C_NUM_QATS   6
#define ACCELCOMP_C_ALL_QAT_MASK 0x3F
#define ACCELCOMP_C_AE_CLK       933
#define ACCELCOMP_C_NUM_MMPS   0x5

#define ACCELCOMP_R_NUM_AES    2
#define ACCELCOMP_R_ALL_AE_MASK  0x3
#define ACCELCOMP_R_NUM_QATS   1
#define ACCELCOMP_R_ALL_QAT_MASK 0x1
#define ACCELCOMP_R_AE_CLK       800
#define ACCELCOMP_R_NUM_MMPS   0x2

#define ACCELCOMP_B_NUM_AES    10
#define ACCELCOMP_B_ALL_AE_MASK  0x3FF
#define ACCELCOMP_B_NUM_QATS   5
#define ACCELCOMP_B_ALL_QAT_MASK 0x1F
#define ACCELCOMP_B_AE_CLK       800
#define ACCELCOMP_B_NUM_MMPS   0x5

#define ACCELCOMP_RS_NUM_AES    6
#define ACCELCOMP_RS_ALL_AE_MASK  0x3F
#define ACCELCOMP_RS_NUM_QATS   3
#define ACCELCOMP_RS_ALL_QAT_MASK 0x7
#define ACCELCOMP_RS_AE_CLK       800
#define ACCELCOMP_RS_NUM_MMPS   0x5


#define ACCELCOMP_AE_FUSEOUT(fuse) ((fuse & ACCELCOMP_ALL_AE_MASK) >> 0) /* FUSECTL[0:7] */
#define ACCELCOMP_QAT_FUSEOUT(fuse) ((fuse & (ACCELCOMP_ALL_QAT_MASK << 8)) >> 8) /* FUSECTL[8:9] */
#define ACCELCOMP_MASK(fuse, aeMask, qatMask)                  \
        do {                                                   \
               (aeMask) = ~ACCELCOMP_AE_FUSEOUT(fuse) & ACCELCOMP_ALL_AE_MASK;  \
               (qatMask) = ~ACCELCOMP_QAT_FUSEOUT(fuse) & ACCELCOMP_ALL_QAT_MASK; \
        } while(0)

#define ACCELCOMP_C_SKU_1(aeMask, qatMask) do { (aeMask) &= 0x0ff; (qatMask) &= 0xf; } while(0) /* 8 AEs and 4 QATs */
#define ACCELCOMP_C_SKU_2(aeMask, qatMask) do { (aeMask) &= 0x0fff; (qatMask) &= 0x3f; } while(0) /* 12 AEs and 6 QATs */
#define ACCELCOMP_C_SKU_3(aeMask, qatMask) do { (aeMask) &= 0x0fff; (qatMask) &= 0x3f; } while(0) // 12 AEs and 6 QATs
#define ACCELCOMP_C_SKU_4(aeMask, qatMask) do { (aeMask) &= 0x0fff; (qatMask) &= 0x3f; } while(0) // 12 AEs and 6 QATs
#define ACCELCOMP_C_AE_FUSEOUT(fuse) ((fuse & ACCELCOMP_C_ALL_AE_MASK) >> 0) /* FUSECTL[0:11] */
#define ACCELCOMP_C_QAT_FUSEOUT(fuse) ((fuse & (ACCELCOMP_C_ALL_QAT_MASK << 13)) >> 13) /* FUSECTL[13:18] */
/* FUSECTL[31:30] */
#define ACCELCOMP_C_SKU(sku, aeMask, qatMask)                                   \
        do {                                                                    \
               if((sku) == 0) { ACCELCOMP_C_SKU_1(aeMask, qatMask); }           \
               if((sku) == 1) { ACCELCOMP_C_SKU_2(aeMask, qatMask); }           \
               if((sku) == 2) { ACCELCOMP_C_SKU_3(aeMask, qatMask); }           \
               if((sku) == 3) { ACCELCOMP_C_SKU_4(aeMask, qatMask); }           \
        } while(0)

#define ACCELCOMP_C_MASK(fuse, aeMask, qatMask)                                  \
        do {                                                                     \
               (aeMask) = ~ACCELCOMP_C_AE_FUSEOUT(fuse) & ACCELCOMP_C_ALL_AE_MASK;                 \
               (qatMask) = ~ACCELCOMP_C_QAT_FUSEOUT(fuse) & ACCELCOMP_C_ALL_QAT_MASK;                \
               ACCELCOMP_C_SKU(((fuse & (0x3 << 20)) >> 20), aeMask, qatMask);   \
        } while(0)

#define ACCELCOMP_R_FUSEOUT(fuse) ((fuse & 0x1) >> 0) /* FUSECTL[0] */
#define ACCELCOMP_R_AE1_FUSEOUT(fuse) ((fuse & (0x1 << 1)) >> 1) /* FUSECTL[1] */

#define ACCELCOMP_R_MASK(fuse, aeMask, qatMask)                    \
        do {                                                       \
               (aeMask) = 0;                                       \
               (qatMask) = 0;                                      \
               if(!ACCELCOMP_R_FUSEOUT(fuse))                      \
               {                                                   \
                   (aeMask) = 0x1;                                 \
                   (qatMask) = 0x1;                                \
                   if(!ACCELCOMP_R_AE1_FUSEOUT(fuse))              \
                               (aeMask) |= 0x2;                    \
               }                                                   \
        } while(0)

#define ACCELCOMP_B_AE_FUSEOUT(fuse) ((fuse & ACCELCOMP_B_ALL_AE_MASK) >> 0) /* FUSECTL[0:9] */
#define ACCELCOMP_B_QAT_FUSEOUT(fuse) ((fuse & (ACCELCOMP_B_ALL_QAT_MASK << 16)) >> 16) /* FUSECTL[16:20] */
#define ACCELCOMP_B_MASK(fuse, aeMask, qatMask)                                    \
    do {                                                                           \
            (aeMask) = ~ACCELCOMP_B_AE_FUSEOUT(fuse) & ACCELCOMP_B_ALL_AE_MASK;    \
            (qatMask) = ~ACCELCOMP_B_QAT_FUSEOUT(fuse) & ACCELCOMP_B_ALL_QAT_MASK; \
       } while(0)

#define ACCELCOMP_RS_AE_FUSEOUT(fuse) ((fuse & ACCELCOMP_RS_ALL_AE_MASK) >> 0) /* FUSECTL[0:9] */
#define ACCELCOMP_RS_QAT_FUSEOUT(fuse) ((fuse & (ACCELCOMP_RS_ALL_QAT_MASK << 16)) >> 16) /* FUSECTL[16:20] */
#define ACCELCOMP_RS_MASK(fuse, aeMask, qatMask)                                    \
    do {                                                                           \
            (aeMask) = ~ACCELCOMP_RS_AE_FUSEOUT(fuse) & ACCELCOMP_RS_ALL_AE_MASK;    \
            (qatMask) = ~ACCELCOMP_RS_QAT_FUSEOUT(fuse) & ACCELCOMP_RS_ALL_QAT_MASK; \
       } while(0)

#define MAX_VAL(max, v, w, x, y, z)        \
        do {                               \
               max = v;                    \
               if(w > max) max = w;        \
               if(x > max) max = x;        \
               if(y > max) max = y;        \
               if(z > max) max = z;        \
        } while(0)

#define ICP_MAX_PCI_BARS    6 
#define MAX_DRAM_CONFIG     2

#define ICP_ACCELCOMP_PCIE_DEVICE_ID_AC      0x0434
#define ICP_ACCELCOMP_C_PCIE_DEVICE_ID_AC    0x0435
#define ICP_ACCELCOMP_R_PCIE_DEVICE_ID_AC    0x1F18
#define ICP_ACCELCOMP_B_PCIE_DEVICE_ID_AC    0x3748       /* Accel_Complex_B CPM 0, 1, 2 */
#define ICP_ACCELCOMP_RS_PCIE_DEVICE_ID_AC   0x19E2       /* Accel_Complex_RS DID */

/* ADF does not need to fill icp_firml_sram_desc_t data structure as it is assumed that whole 
   sram is viewed as a pci BAR, but loader will fill it and store it in LoaderHandle */
typedef struct icp_firml_sram_desc_s {
    uint64 sramBaseAddr;      /**< sram base physical address */
    uint64 sramBaseAddr_v;    /**< sram base virtual address */
    uint64 sramAeAddr;        /**< sram base address from AE prospective, hard-coding 
                                   0x2_0000_0000_0000 for Accel Complex */
    unsigned int sramSize;    /**< size of sram */
    unsigned int startOffset; /**< start offset for AE */
} icp_firml_sram_desc_t;

/* ADF does not need to fill icp_firml_scratch_desc_t data structure as it is assumed that whole 
   scratch is viewed as a pci BAR, but loader will fill it and store it in LoaderHandle */
typedef struct icp_firml_scratch_desc_s {
    uint64 scratchBaseAddr;    /**< scratch base physical address */
    uint64 scratchBaseAddr_v;  /**< scratch base virtual address */
    unsigned int scratchSize;  /**< size of scratch */
    unsigned int startOffset;  /**< start offset for AE */
} icp_firml_scratch_desc_t;

/* it assumes that ADF allocates loader-visible DRAM0 and DRAM1 for each Accel Complex instance */
typedef struct icp_firml_dram_desc_s {
    uint64 dramBaseAddr;    /**< dram base physical address for this Accel Complex, AE uses same physical address */
    uint64 dramBusAddr;     /**< dram bus physical address for this Accel Complex, 
                                 AE uses same physical address,
                                 It is used by AE device.
                                 It equals dramBaseAddr when VT-d is disabled, and not when VT-d is enabled since it is Guest Phy Addr */
    uint64 dramBaseAddr_v;  /**< dram base virtual address for this Accel Complex */
    uint64 dramSize;        /**< dram size for this Accel Complex */
    uint64 aeDramOffset;    /**< offset to the ae-dram relative to beg of dram */     
    uint64 startOffset;     /**< start offset for AE */
} icp_firml_dram_desc_t;

typedef struct icp_firml_bar_s {
    uint64 baseAddr; /**< read from PCI config */
    uint64 virtAddr; /**< mapped to kernel VA */
    uint64 size;     /**< size of BAR */
} icp_firml_bar_t;

typedef struct Hal_IntrMasks_S {
    unsigned int attn_bkpt_mask;    /**< Mask of AEs */
    unsigned int attn_parity_mask;  /**< Mask of AEs */
} Hal_IntrMasks_T;

typedef void (*HalAeIntrCallback_T)(Hal_IntrMasks_T *masks, 
                                    void* data);

struct HalAeCallback_S;
typedef struct HalAeCallbackChain_S {
    struct HalAeCallbackChain_S *next;
    unsigned int                   priority;      /**< higher value called first */
    HalAeIntrCallback_T            callback_func; /**< callback function pointer */
    void*                          callback_data; /**< callback data */
    struct HalAeCallback_S         *thread_data;  /**< back pointer */
} HalAeCallbackChain_T;

typedef struct HalAeCallback_S {
    struct HalAeCallback_S  *next;                /**< next callback */  
    unsigned int            type_mask;            /**< type mask */
    OsalSemaphore         terminate_sem;        /**< termination semaphore */
    OsalThread            threadID;             /**< thread ID */
    HalAeCallbackChain_T    callback_chain;       /**< When chained, this first
                                                  CallbackChain structure is 
                                                  just used as a pointer to 
                                                  the chain */
    void *                  handle;               /**< icp_firml_handle_t handle */  
} HalAeCallback_T;

typedef struct REQUEST {                    /* data structures to support an interrupt "driver" */
    struct REQUEST   *next;
    OsalSemaphore  sem;
    unsigned int     type_mask;
    Hal_IntrMasks_T  *masks;
    int              status;
}REQUEST_T;

typedef struct {
    unsigned int vaddr;                    /* page virtual */
    unsigned int paddr;                    /* page physical address */
    unsigned int size;                     /* page size */
    unsigned int loaded;                   /* loaded flag */
} AddrPair_T;

typedef struct {
    unsigned int  numPages;                /* number of pages */
    AddrPair_T   *addrs;                   /* page pairs pointer */
} AePageData_T;

typedef struct PageData_S {
    AePageData_T AePageData[MAX_AE];       /* using hardware AE addressing */
} PageData_T;

typedef struct{
   unsigned int state;                     /* AE state */
   unsigned int uStoreSize;                /* micro-store size */
   unsigned int freeAddr;                  /* free micro-store address */
   unsigned int freeSize;                  /* free micro-store size */
   unsigned int liveCtxMask;               /* live context mask */
   unsigned int ustoreDramAddr;            /* micro-store dram address */
   unsigned int reloadSize;                /* reloadable code size */
   OsalLock aeLock;                  /* AE lock */
}Ae_T;

typedef struct{
    unsigned int prodId;                             /**< AE product ID and revisions as per PRM */
    unsigned int PrdMajType, PrdMinType, PrdMajRev, PrdMinRev;
    Ae_T AEs[AE_NUMCLUSTR][AE_PERCLUSTR]; 
    unsigned int  HalAe_LibInit;                     /**< HAL initializaion flag */
    unsigned int  UofChecksum;                       /**< .uof check sum */
    OsalLock  hal_GlobalLock;                  /**< HAL global lock */ 
    OsalLock  hal_CallbackThdLock;             /**< HAL callback thread lock*/
    OsalLock  hal_InterruptLock;               /**< HAL interrupt lock */
    HalAeCallback_T *ISR_callback_root;              /**< ISR callback root */
    struct PageData_S *pageData;                     /**< page data pointer */
 
    OsalLock bk_lock;                          /**< HAL attention breakpoint lock */ 
    OsalLock tasklet_lock;                     /**< HAL tasklet lock */ 
    OsalLock bpt_lock;                         /** SMP lock for ISR handling*/
    unsigned int    g_summary_mask;                  /**< irq summary mask */ 
    unsigned int    g_enabled_mask;                  /**< enabled irq mask */ 
    Hal_IntrMasks_T g_intr_masks;                    /**< irq mask */  
    struct REQUEST *g_requests;                      /**< irq request */ 
    struct REQUEST *g_available;                     /**< irq availiable */ 
    unsigned int AeMask;                    /**< map AE number to number within the cluster */
    unsigned int AePerCluster;    /**< the number of AE per the cluster */
    unsigned int AeBadMask;       /**< bad aeMask */
    unsigned int AeMaxNum;        /**< the max enabled AE number */
    unsigned int UpcMask;         /**< ustore PC mask */
    unsigned int MaxUstore;       /**< the max size of ustore */
    unsigned int MaxLmemReg;      /**< the max size of local memory register */
    icp_firml_dram_desc_t dbgDramDesc;  /**< reserved dram descriptor for debugging purpose */    
    unsigned int qatDevEnable[QAT_SLICE_END]; /**< enabled QAT slices */
    uint64 dmaHandle;                                 /**< DMA Bus Address */
}Hal_Handle_T;

typedef struct icp_firml_sys_mem_info_s {
    unsigned int           instanceId;     /**< Accel Complex instance ID assigned by ADF */
    unsigned int           deviceId;       /**< Accel Complex PCIe endpoint device ID */
    unsigned int           numAe;          /**< number of AEs */    
    unsigned int           numQat;         /**< number of QATs */    
    unsigned int           aeMask;         /**< enabled AE mask */
    unsigned int           qatMask;        /**< enabled QAT mask */       
    unsigned int           aeClkMhz;       /**< AE clock speed in mega-hertz */    
    unsigned int           numBars;        /**< number of BARs */    
    icp_firml_bar_t    pciBars[ICP_MAX_PCI_BARS];  /**< BAR descriptor */       
    unsigned int           numDramDesc;    /**< number of dram channels */
    icp_firml_dram_desc_t  dramDesc[MAX_DRAM_CONFIG]; /**< dram descriptor */
    icp_firml_sram_desc_t  sramDesc;       /**< sram descriptor */    
    icp_firml_scratch_desc_t scratchDesc;  /**< scratch descriptor */    
    unsigned char            revisionId;   /**< revision ID */
    unsigned char            reload;       /**<1 -reload time; 0 -boot time */
    void * dev;                            /**< pointer to device structure of function0 */
    unsigned int           fwAuth;         /**<  0- traditional loader, 1-authentication loader*/
    unsigned int           debug;          /**< debug is enabled, so disable clock gating feature */
} icp_firml_sys_mem_info_t;

typedef struct icp_firml_handle_s {
    OsalLock lock;                                   /**< global lock */
    icp_firml_sys_mem_info_t sysMemInfo;             /**< system and memory information */
    Hal_Handle_T * halHandle;                        /**< HAL global data handle */ 
    void *objHandle;                                 /**< loader handle */ 
    void *dbgAeHandle;                               /**< debug handle */ 
    void *pmuHandle;                                 /**< PMU handle */ 
    void *sobjHandle;                                /**< signed object loader handle */
    void *mofHandle;                                 /**< MOF handle */

    volatile uint64 Hal_dram_ch0_virtAddr;           /**< DRAM chanel0 (DRAM0) */
    volatile uint64 Hal_dram_ch1_virtAddr;           /**< DRAM chanel1 (DRAM1) */
    volatile uint64 Hal_sram_virtAddr;               /**< SRAM begin through end of read-write region */
    volatile uint64 Hal_cap_global_ctl_csr_virtAddr; /**< Global Control CSR */
    volatile uint64 Hal_cap_ae_xfer_csr_virtAddr;    /**< AE Xfer CSR */
    volatile uint64 Hal_cap_ae_local_csr_virtAddr;   /**< AE local CSR */
    volatile uint64 Hal_cap_pmu_csr_virtAddr;        /**< PMU CSR */
    volatile uint64 Hal_cap_hash_csr_virtAddr;       /**< hash CSR */
    volatile uint64 Hal_scratch_rd_wr_swap_virtAddr; /**< Scratch read write swap */
    volatile uint64 Hal_ae_fastaccess_csr_virtAddr;  /**< fast access CSR */
    volatile uint64 Hal_ssu_csr_virtAddr;            /**< SSU CSR */      
    volatile uint64 Hal_eagletail_ring_csr_virtAddr; /**< ET ring CSR start address */
    volatile uint64 Hal_ep_csr_virtAddr;             /**< EP CSR start address */

    unsigned int valid;                              /**< valid flag */
} icp_firml_handle_t;


#endif /* __ICP_FIRML_HANDLE_H__ */
