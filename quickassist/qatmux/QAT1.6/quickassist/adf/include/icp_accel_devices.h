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
 * @file icp_accel_devices.h
 *
 * @defgroup Acceleration Driver Framework
 *
 * @ingroup icp_Adf
 *
 * @description
 *      This is the top level header file that contains the layout of the ADF
 *      icp_accel_dev_t structure and related macros/definitions.
 *      It can be used to dereference the icp_accel_dev_t *passed into upper
 *      layers.
 *
 *****************************************************************************/

#ifndef ICP_ACCEL_DEVICES_H_
#define ICP_ACCEL_DEVICES_H_

#if (defined (__linux__) && defined (__KERNEL__))
#include <linux/types.h>
#include <asm/atomic.h>
#include <linux/pci.h>
#include <linux/firmware.h>
#include <linux/completion.h>
#endif

#if (defined (__freebsd) && defined (__KERNEL__))
#include <sys/bus.h>
#endif

#include "cpa.h"
#include "Osal.h"

/*
 * PCI Macros
 */

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      BAR Registers for AE Cluster / Ring Controller
 *
 *****************************************************************************/
#define ICP_EP80579_MAX_PCI_BARS       6

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *     Number of Accelerators and accel engines
 *
 *****************************************************************************/
#define EP80579_MAX_ACCELERATORS       1
#define EP80579_MAX_ACCELENGINES       1
#define ICP_ADF_MAX_AE_PER_DEV         12

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *       4K CAP & Conv. Bridge register
 *
 *****************************************************************************/
#define ICP_AE_CLUSTER_CAP_BAR         0

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *       16K Scratch memory
 *
 *****************************************************************************/
#define ICP_AE_CLUSTER_SP_BAR          1

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      16K AE CSRs and transfer registers
 *
 *****************************************************************************/
#define ICP_AE_CLUSTER_AE_BAR          2

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      4K SSU CSRs
 *
 *****************************************************************************/
#define ICP_AE_CLUSTER_SSU_BAR         3

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      AE Cluster Base Address Registers
 *
 *****************************************************************************/
#define ICP_AE_CLUSTER_BARS            4

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      4K control register
 *
 *****************************************************************************/
#define ICP_RING_CONTROLLER_CSR_BAR    0

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      4K Get/Put access
 *
 *****************************************************************************/
#define ICP_RING_CONTROLLER_RING_BAR   1

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *       256K on-chip SRAM
 *
 *****************************************************************************/
#define ICP_RING_CONTROLLER_SRAM_BAR   2

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      The number of Ring Controller Base Address Registers
 *
 *****************************************************************************/
#define ICP_RING_CONTROLLER_BARS       3

/*****************************************************************************
 * Device ID section
 *****************************************************************************/
/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device IDs for the DH895xCC,DH89XXCC and C2XXX 
 *
 *****************************************************************************/


#define ICP_DH89XXCC_DEVICE_NAME      "dh89xxcc"
#define ICP_C2XXX_DEVICE_NAME         "c2xxx"
#define ICP_DH895XCC_DEVICE_NAME      "dh895xcc"
#define ICP_QAT_1_6_DRIVER_NAME       "qat_1_6"


/*
 * Device specific Arbiter/Ring settings
 */
#define ICP_DH895xCC_ASYM_TX_RING_0     0
#define ICP_DH895xCC_ASYM_TX_RING_1     1
#define ICP_DH895xCC_SYM_TX_RING_0      2
#define ICP_DH895xCC_SYM_TX_RING_1      3
#define ICP_DH895xCC_NRBG_TX_RING_0     4
#define ICP_DH895xCC_NRBG_TX_RING_1     5
#define ICP_DH895xCC_DC_TX_RING_0       6
#define ICP_DH895xCC_DC_TX_RING_1       7
#define ICP_DH895xCC_RX_RINGS_OFFSET    8
#define ICP_DH895xCC_RINGS_PER_BANK     16

/*
 * Offset to PCI device capabilities
 */
#define PCIE_CAP_OFFSET                 0x50

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Maximum PCI BARs
 *
 *****************************************************************************/
#define ICP_PCI_MAX_BARS       6

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device revision id offset
 *
 *****************************************************************************/
#define ICP_PRID_OFFSET     0x8

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Defined for device name length
 *
 *****************************************************************************/
#define MAX_ACCEL_NAME_LEN              16
#define ADF_DEVICE_NAME_LENGTH          32

#ifndef WITH_CPA_MUX
#define ADF_DEV_CSR_NAME                    "icp_dev_csr"
#define ADF_DEV_CSR_NAME_FMT                "icp_dev%d_csr"
#define ADF_DEV_CSR_PATH_FMT                "/dev/icp_dev%d_csr"
#define ADF_DEV_RING_NAME                   "icp_dev_ring"
#define ADF_DEV_RING_NAME_FMT               "icp_dev%d_ring"
#define ADF_DEV_RING_PATH_FMT               "/dev/icp_dev%d_ring"
#define ADF_DEV_BANK_NAME_FMT               "icp_dev%d_bank%d"
#define ADF_DEV_BANK_PATH_FMT               "/dev/icp_dev%d_bank%d"
#define ADF_DEV_PROCESSES_NAME              "icp_dev_processes"
#define ADF_DEV_PROCESSES_PATH              "/dev/icp_dev_processes"
#define ADF_DEV_PFVFCOMMS_NAME              "icp_dev_pfvfcomms"
#define ADF_DEV_PFVFCOMMS_PATH              "/dev/icp_dev_pfvfcomms"
#define ADF_CFG_IOCTL_DEVICE_NAME           "icp_adf_ctl"
#define ADF_CFG_IOCTL_DEVICE_PATH           "/dev/icp_adf_ctl"
#endif
#ifdef WITH_CPA_MUX
#define ADF_DEV_CSR_NAME                    "icp_mux/icp_dev_csr"
#define ADF_DEV_CSR_NAME_FMT                "icp_mux/icp_dev%d_csr"
#define ADF_DEV_CSR_PATH_FMT                "/dev/icp_mux/icp_dev%d_csr"
#define ADF_DEV_RING_NAME                   "icp_mux/icp_dev_ring"
#define ADF_DEV_RING_NAME_FMT               "icp_mux/icp_dev%d_ring"
#define ADF_DEV_RING_PATH_FMT               "/dev/icp_mux/icp_dev%d_ring"
#define ADF_DEV_BANK_NAME_FMT               "icp_mux/icp_dev%d_bank%d"
#define ADF_DEV_BANK_PATH_FMT               "/dev/icp_mux/icp_dev%d_bank%d"
#define ADF_DEV_PROCESSES_NAME              "icp_mux/icp_dev_processes"
#define ADF_DEV_PROCESSES_PATH              "/dev/icp_mux/icp_dev_processes"
#define ADF_DEV_PFVFCOMMS_NAME              "icp_mux/icp_dev_pfvfcomms"
#define ADF_DEV_PFVFCOMMS_PATH              "/dev/icp_mux/icp_dev_pfvfcomms"
#define ADF_CFG_IOCTL_DEVICE_NAME           "icp_mux/icp_adf_ctl"
#define ADF_CFG_IOCTL_DEVICE_PATH           "/dev/icp_mux/icp_adf_ctl"
#endif


#define ADF_DEVICE_TYPE_LENGTH          8


/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Accelerator capabilities
 *
 *****************************************************************************/
typedef enum
{
    ICP_ACCEL_CAPABILITIES_NULL                 = 0,
    ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC     = 1,
    ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC    = 2,
    ICP_ACCEL_CAPABILITIES_CIPHER               = 4,
    ICP_ACCEL_CAPABILITIES_AUTHENTICATION       = 8,
    ICP_ACCEL_CAPABILITIES_REGEX                = 16,
    ICP_ACCEL_CAPABILITIES_COMPRESSION          = 32,
    ICP_ACCEL_CAPABILITIES_LZS_COMPRESSION      = 64,
    ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER        = 128,
    ICP_ACCEL_CAPABILITIES_CRYPTO_0             = 256,
    ICP_ACCEL_CAPABILITIES_CRYPTO_1             = 512
} icp_accel_capabilities_t;

typedef struct icp_accel_bar_s
{
    UARCH_INT            baseAddr;  /* read from PCI config */
    UARCH_INT            virtAddr;  /* mapped to kernel VA */
    Cpa32U               size;      /* size of BAR */
} icp_accel_bar_t;

typedef struct icp_version_s
{
    /* lowest supported Driver version */
    Cpa32U device_lowestSupportMajorVersion;
    Cpa32U device_lowestSupportMinorVersion;
    /* These fields are only used on VF */
    CpaBoolean versionMsgAvailable;
    Cpa32U pf_major;
    Cpa32U pf_minor;
    Cpa32U supported; 
} icp_version_t;

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device States
 *
 *****************************************************************************/
typedef enum
{
    ICP_ACCEL_STATE_UNINITIALIZED=0,
    ICP_ACCEL_STATE_INITIALIZED,
    ICP_ACCEL_STATE_STARTED,
    ICP_ACCEL_STATE_STOPPED,
    ICP_ACCEL_STATE_SHUTDOWN
} icp_accel_state_t;

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      PCI information
 *
 *****************************************************************************/
/* Maximum length of MSIX name */
#define MAX_MSIX_DEVICE_NAME    16

typedef enum
{
    ADF_ISR_MODE_INTX = 0,
    ADF_ISR_MODE_MSI,
    ADF_ISR_MODE_MSIX
} adf_isr_mode_t;

typedef struct icp_accel_msix_info_s
{
    struct msix_entry   *value;
    Cpa8U               (*name)[MAX_MSIX_DEVICE_NAME];
} icp_accel_msix_info_t;

typedef struct icp_accel_domain_info_s
{
    Cpa32U  domain;
    Cpa8U   bus;
    Cpa8U   slot;
    Cpa8U   function;
} icp_accel_domain_info_t;

typedef struct icp_accel_pci_info_s
{
    /* pDev is used by kernel and UT builds, but not user space */
    struct pci_dev      *pDev;
    icp_accel_msix_info_t msixEntries;
    adf_isr_mode_t      isr_mode;
    icp_accel_domain_info_t pciDomain;
    icp_accel_state_t   state;
    Cpa32U              deviceId;
    Cpa32U              irq;
    Cpa32U              numBars;
    icp_accel_bar_t     pciBars[ICP_PCI_MAX_BARS];
    Cpa32U              devFuses;
    Cpa8U               revisionId;
} icp_accel_pci_info_t;

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device Configuration Data Structure
 *
 *****************************************************************************/
typedef enum dev_sku_info_e
{
    DEV_SKU_1 = 0,
    DEV_SKU_2,
    DEV_SKU_3,
    DEV_SKU_4,
    DEV_SKU_VF,
    DEV_SKU_UNKNOWN,
} dev_sku_info_t;

static inline const char* get_sku_info(dev_sku_info_t info)
{
    switch (info)
    {
        case DEV_SKU_1:
              return "SKU1";
        case DEV_SKU_2:
              return "SKU2";
        case DEV_SKU_3:
              return "SKU3";
        case DEV_SKU_4:
              return "SKU4";
        case DEV_SKU_VF:
              return "SKUVF";
        case DEV_SKU_UNKNOWN:
        default:
              return "UNKNOWN SKU";
    }
    return "UNKNOWN SKU";
}

typedef enum dev_state_e
{
    DEV_DOWN = 0,
    DEV_UP
} dev_state_t;

typedef enum device_type_e
{
    DEV_UNKNOWN = 0,
    DEV_DH89XXCC,
    DEV_C2XXX,
    DEV_DH89XXCC_VF,
    DEV_DH895XCC
} device_type_t;

typedef void (*bank_handler)(void*);

typedef struct adf_esram_data_s
{
    Cpa64U sramAeAddr;
    Cpa32U sramAeSize;
} adf_esram_data_t;

typedef struct adf_hw_device_class_s
{
    const device_type_t   type;
    const char            *name;
    Cpa32U                numInstances;
    Cpa32U                currentInstanceId;
}adf_hw_device_class_t;

/* Forward declaration of icp_accel_dev_t as needed by adf_hw_device_data_s */
typedef struct accel_dev_s icp_accel_dev_t;
typedef struct icp_et_ring_data_s icp_et_ring_data_s_t;

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      PCI VF Structure
 *
 *****************************************************************************/

typedef struct icp_accel_pci_vf_info_s
{
    Cpa32U                  deviceId;
#if defined (__KERNEL__)
    struct pci_dev          *pDev;
#endif
    icp_accel_dev_t         *accel_dev;
    void                    *tasklet_vf;
    void                    *vf_ring_info_tbl;
    Cpa8U                   attached : 1;
    Cpa8U                   init : 1;
    void                    *lock;
} icp_accel_pci_vf_info_t;

typedef struct adf_hw_device_data_s
{
    adf_hw_device_class_t *dev_class;
    Cpa32U                  instanceId;
    /* all data fields are MANDATORY */
    Cpa32U          maxBars;             /* max number of BARs                */
    Cpa32U          maxNumBanks;         /* max number of banks               */
    Cpa32U          maxNumApBanks;       /* max number of AutoPush banks      */
    Cpa32U          numRingsPerBank;     /* rings per bank                    */
    Cpa32U          numBanksPerAccel;    /* banks per accelerator             */
    Cpa32U          maxNumAccel;         /* max number of accelerators        */
    Cpa32U          maxNumAccelEngines;  /* max number of accelerator engines */
    Cpa32U          aeClockInMhz;        /* clock speed in Mhz                */
    Cpa32U          sintpfOffset;        /* SINTPF register offset            */
    Cpa32U          sintpf1Offset;       /* SINTPF1 register offset (set if available)*/
    Cpa32U          userEtringCsrSize;   /* user space etring size            */
    Cpa32U          maxNumVf;            /* max number of virtual functions   */
    Cpa32U          bankSize;            /* Offset between banks              */

    /* number of rings split accross the max available accelerators.*/
    Cpa32U          maxNumRingsPerAccelerator;

    /* fields in msix structure are MANDATORY */
    struct {
        Cpa32U      banksVectorStart;    /* bank msix first vector index     */
        Cpa32U      banksVectorNum;      /* number of bank vectors from first*/
        Cpa32U      aeVectorStart;       /* ae msix first vector index       */
        Cpa32U      aeVectorNum;         /* number of ae vectors from first  */
    } msix;

    CpaBoolean isMemScrubSupported;

    /* If not used _MUST_ be set to NULL */
    CpaStatus  (*init)(struct adf_hw_device_data_s *hw_data, Cpa8U node_id);
    void    (*cleanup)(struct adf_hw_device_data_s *hw_data);
    CpaStatus  (*adf_addFwMsgSizesToCfgTable)(icp_accel_dev_t *accel_dev);

    /* Accelerator/Engine & SKU functions */
    Cpa32U (*getDevFuseOffset)(void);
    Cpa32U (*getDevClkOffset)(void);

    Cpa32U (*getAcceleratorsMask)(Cpa32U fuse);
    Cpa32U (*getAccelEnginesMask)(Cpa32U fuse);
    dev_sku_info_t (*getDevSKU)(Cpa32U numAccEngines, Cpa32U numAccels,
                                Cpa32U clkMask, Cpa32U fuse);

    Cpa32U (*getNumAccelerators)(struct adf_hw_device_data_s *self,
            Cpa32U mask);
    Cpa32U (*getNumAccelEngines)(struct adf_hw_device_data_s *self,
            Cpa32U mask);

    void (*getAccelMaskList)(struct adf_hw_device_data_s *self,
            Cpa32U accelMask, Cpa32U aeMask, Cpa32U *pMaskList);

    /* Capabilities Function */
    Cpa32U (*getAccFuncOffset)(void);
    Cpa32U (*getAccFuncMask)(Cpa32U func);

    /* BAR functions */
    Cpa32U (*getMiscBarId)(struct adf_hw_device_data_s *self);

    Cpa32U (*getSramBarId)(struct adf_hw_device_data_s *self);

    Cpa32U (*getEtrBarId)(struct adf_hw_device_data_s *self, Cpa32U *offset);

    CpaBoolean (*isBarAvailable)(struct adf_hw_device_data_s *self,
            Cpa32U barId);

    /* eSRAM functions  */
    void (*getEsramInfo)(struct adf_hw_device_data_s *self,
            adf_esram_data_t *esramInfo);

    /* SHAC functions */
    void (*getScratchRamInfo)(struct adf_hw_device_data_s *self,
            Cpa32U *offset, Cpa32U *size);

    /* PF interrupt functions */
    void (*setSmiapfOffsetMask)(struct adf_hw_device_data_s *self,
            icp_accel_pci_info_t *pci_info);

    Cpa32U (*irqGetBankNumber)(struct adf_hw_device_data_s *self,
            Cpa32U sintpf, Cpa32U *miscAddr);

    Cpa32U (*irqGetAeSource)(Cpa32U sintpf, Cpa32U *miscAddr);

    /* VF interrupt functions */
    Cpa32U (*irqGetBunSource)(struct adf_hw_device_data_s *self,
            Cpa32U vintsou);

    Cpa32U (*irqGetPF2VFSource)(struct adf_hw_device_data_s *self, Cpa32U vintsou);

    /* bank handlers */
    bank_handler (*getBankBhHandler)(Cpa32U bankId);
    bank_handler (*getBankBhPollingHandler)(Cpa32U bankId);

    /* Acceleration engine auto-push feature */
    CpaBoolean isAeAutoPushSupported;
    void (*getAeApCsrOffsets)(Cpa32U bankId, Cpa32U *nfMask, Cpa32U *nfDest,
            Cpa32U *neMask, Cpa32U *neDest, Cpa32U *apDelay);

    /*Interrupt Optimisation feature (IAIntFlagANDColEn)*/
    void (*accessCsrIntFlagAndCol)(Cpa32U *csr_base_addr, Cpa32U bank_offset,
                                   Cpa32U value, CpaBoolean write);

	/* Used for enabling SHIM_REQ_NUM bit for improved performance on c2xxx device*/
	CpaBoolean      setShimReqNumBit;

    /* Allow overriding of SRCSEL CSR.
       If NULL, code uses DH89xxCC functionality */
    void  (*writeCsrSrcSel)(Cpa32U *csr_base_addr, Cpa32U bank_offset,
                            Cpa32U *irq_src_val_low, Cpa32U *irq_src_val_hi);

    /* ETR less admin/init communication */
    CpaBoolean isETRlessAdminInitCommSupported;
    CpaStatus (*initAdminComms)(icp_accel_dev_t *accel_dev);

    int (*restoreDevice)(icp_accel_dev_t *accel_dev);

    /* Arbitration feature. */
    CpaBoolean isArbitrationSupported;
    CpaStatus (*arbInit)(icp_accel_dev_t *accel_dev);
    CpaStatus (*arbShutdown)(icp_accel_dev_t *accel_dev);
    CpaStatus (*arbVerifyResponseMsgSize)(icp_accel_dev_t *pAccelDev,
                                          icp_et_ring_data_s_t *pRing);

	/* Other features. */
    CpaBoolean isGigeWatchdogSupported;
	CpaBoolean checkForRevisionId;
    CpaBoolean isWirelessSupported;
    CpaBoolean isSriovSupported;

	/* Device sleep times */
    Cpa32U msleep_time;
    Cpa32U msleep_trip_time;
    Cpa32U msleep_sbr_time;


    /* PF-to-VF functions */
    void (*disableVf2PfInterrupts)(icp_accel_dev_t *accel_dev, Cpa32U vfMsk);
    void (*enableVf2PfInterrupts)(icp_accel_dev_t *accel_dev, Cpa32U vfMsk);

    Cpa32U (*getVf2PfIntSrcMask)(icp_accel_dev_t *accel_dev);
    Cpa32U (*getVf2PfIntSourceOffset)(void);
    Cpa32U (*getVf2PfBitOffset)(void);
    Cpa32U (*getVf2PfIntMaskOffset)(void);
    Cpa32U (*getVf2PfIntMaskDefault)(void);
    Cpa32U (*getTiMiscIntCtlOffset)(void);
    Cpa32U (*getVIntSrcOffset)(Cpa32U id);
    Cpa32U (*getVIntMskOffset)(Cpa32U id);
    Cpa32U (*getVIntMskDefault)(void);
    Cpa32U (*getPf2VfDbIntOffset)(Cpa32U id);

    /* Hardware revision */
    CpaStatus (*getHardwareVersion)(char *str, Cpa32U revision_id);

    /* Reset offset and reset vector */
    void (*getResetOffsets)(Cpa32U *resetOffset, Cpa32U *resetVector);

    /* Uncorrectable errors functions */
    CpaStatus  (*enableUncoErrInterrupts)(icp_accel_dev_t *accel_dev);
    CpaBoolean (*handleUncoErrInterrupts)(icp_accel_dev_t *accel_dev);

    /* Private Data */
    void *privateData;
    
    CpaStatus (*adf_PF2VF_putMsg)(icp_accel_pci_vf_info_t *accel_dev, Cpa32U msg);
    CpaStatus (*adf_VF2PF_putMsg)(icp_accel_dev_t *accel_dev, Cpa32U msg);
    CpaStatus (*adf_systemMsg_PF2VF_putMsg)(icp_accel_pci_vf_info_t *accel_dev, Cpa32U msg);
    CpaStatus (*adf_systemMsg_VF2PF_putMsg)(icp_accel_dev_t *accel_dev, Cpa32U msg);

    icp_version_t       icp_version;

}adf_hw_device_data_t;

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Helper macros for the device configuration structure
 *
 *****************************************************************************/
#define NULL_FUNCTION(function) ((function) == NULL)

#define NOT_NULL_FUNCTION(function) ((function) != NULL)

#define GET_HW_DATA(accel_dev)                           \
             (accel_dev->pHwDeviceData)

#define GET_MAX_BARS(accel_dev)                             \
            (GET_HW_DATA(accel_dev)->maxBars)

#define GET_MAX_BANKS(accel_dev)                            \
             (GET_HW_DATA(accel_dev)->maxNumBanks)

#define GET_MAX_AP_BANKS(accel_dev)                         \
             (GET_HW_DATA(accel_dev)->maxNumApBanks)

#define GET_NUM_RINGS_PER_BANK(accel_dev)                   \
             (GET_HW_DATA(accel_dev)->numRingsPerBank)

#define GET_NUM_BANKS_PER_ACCEL(accel_dev)                  \
             (GET_HW_DATA(accel_dev)->numBanksPerAccel)

#define GET_MAX_ACCEL(accel_dev)                            \
             (GET_HW_DATA(accel_dev)->maxNumAccel)

#define GET_MAX_ACCELENGINES(accel_dev)                     \
             (GET_HW_DATA(accel_dev)->maxNumAccelEngines)

#define GET_PCI_DEV(accel_dev)                     \
             ((accel_dev)->pciAccelDev.pDev)

#define GET_DEV(accel_dev)                         \
             ((accel_dev)->pciAccelDev.pDev->dev)


#define FIRST_ADMIN_BUNDLE 0
#define SECOND_ADMIN_BUNDLE 8
#define IS_VF_BUNDLE(bundle_nr) \
        (bundle_nr != FIRST_ADMIN_BUNDLE && \
         bundle_nr != SECOND_ADMIN_BUNDLE)

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Accelerator Device Structure.
 *
 *****************************************************************************/
#define ADF_CFG_NO_INSTANCE 0xFFFFFFFF
/*
 * Enumeration on Service Type
 */
typedef enum adf_service_type_s
{
    ADF_SERVICE_CRYPTO,
    ADF_SERVICE_COMPRESS,
    ADF_SERVICE_MAX   /* this is always the last one */
} adf_service_type_t;

/* store the file descriptors for each bank */
struct bank_files {
#define MAX_INST_PER_BANK 4
    int fd[MAX_INST_PER_BANK];
};

struct accel_dev_s
{
    /* Some generic information */
	char            type[ADF_DEVICE_TYPE_LENGTH]; /* Dev type for SAL */
    Cpa32U          accelId;
    Cpa8U           *pAccelName;            /* Name given to accelerator */
    Cpa32U          accelMask;              /* Accelerator mask */
    Cpa32U          aeMask;                 /* Acceleration Engine mask */
    Cpa32U          accelCapabilitiesMask;  /* Accelerator's capabilities
                                               mask */
    OsalAtomic      usageCounter;           /* Usage counter. Prevents
                                               shutting down the dev if not 0*/
    OsalAtomic      userspaceRingsCounter;  /* Counts the number of
                                               user space rings */
    void            *pUofFirmwareLocation;
    void            *pMmpFirmwareLocation;
    icp_accel_pci_info_t     pciAccelDev; /* Generic PCI Device */
    void            **pci_vfs;
    void            *pf2vf_tasklet;
    adf_hw_device_data_t *pHwDeviceData;    /* Device Configuration Data */
    void            *pUcodePrivData;    /* UCLO private data handle */
    /* Component specific fields - cast to relevent layer */
    void            *pRingInflight;     /* For offload optimization */
    void            *pCommsHandle;      /* For ADF */
    void            *pSalHandle;        /* For SAL*/
    void            *pQatHandle;        /* For QATAL*/
    void            **pQatCpuMem;       /* DMA virtual memory for QAT */
    Cpa64U          *pQatDmaMem;        /* DMA phys. memory for QAT */
    void            *pMessageSlots;     /* Virtual address of coherent DMA mapping */
    Cpa64U          pMessageSlotsDma;   /* Bus address of coherent DMA mapping */
    void            *pShramConstants;   /* Virtual address of Shram constants page */
    Cpa64U          pShramConstantsDma; /* Bus address of Shram constants page */
    void            *pQatStats;         /* For QATAL/SAL stats */
    void            *pUserSpaceHandle;  /* For user space proxy */
    void            *ringInfoCallBack;  /* Callback for user space
                                                       ring enabling */
    void            *lock;              /* lock around CSR*/
    void            *esram_addr;        /*virtual address for eSRAM. */
    /* Debug handle */
    void            *base_dir;
    /* Status of ADF and registered subsystems */
    Cpa32U          adfSubsystemStatus;
    /* Physical processor to which the dev is connected */
    Cpa8U           pkg_id;
    /* Flags indicating if virtualization is enabled
     * and if the device instance is virtualized */
    struct {
        Cpa8U       enabled : 1;
        Cpa8U       virtualized : 1;
    } virtualization;
    void            *vf_rings_info;
    void            *pci_state;
    dev_sku_info_t  sku;
    /* Dma coherent pointers to the buffer used by the mem scrubber */

    /* these elements are used only in user space */
    Cpa64U          firmwareVersion;        /* Firmware version */
    Cpa8U           devFileName[ADF_DEVICE_NAME_LENGTH];
    Cpa32S          csrFileHdl;
    Cpa32S          ringFileHdl;
    Cpa8U           *virtConfigSpace;
    void            *shmem;             /*virtual address for share memory.*/
    Cpa32U          maxNumBanks;
    Cpa32U          maxNumRingsPerBank;
    Cpa32U          userRingCsrSize;
    void            *arbDebugHandle;
    void            *pfvfDebugHandle;
    Cpa32U          maxNumAccel;        /* max number of accelerators */
    Cpa32U          maxNumVf;      /* max number of virtual functions*/

    /* pointer to dynamic instance resource manager */
    void            *pInstMgr;

    /* Linked list of accel_handles - for easy traversing */
    struct accel_dev_s  *pNext;
    struct accel_dev_s  *pPrev;

    /* for epoll */
    struct bank_files *bankFileHdls;
};

/*
 * Hardware capabilities of the device
 */
static inline CpaBoolean icp_adf_isArbitrationSupported(icp_accel_dev_t *accel_dev)
{
    if (GET_HW_DATA(accel_dev) == NULL)
        return CPA_FALSE;
    return (GET_HW_DATA(accel_dev))->isArbitrationSupported;
}

#endif /* ICP_ACCEL_HANDLE_H */
