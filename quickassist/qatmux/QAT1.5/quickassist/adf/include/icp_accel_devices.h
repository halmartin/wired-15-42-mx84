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
#endif

#if (defined (__freebsd) && defined (__KERNEL__))
#include <sys/bus.h>
#endif

#include "cpa.h"
#include "Osal.h"

/*
 * PCI Macros
 */
#define PCI_VENDOR_ID_INTEL 0x8086

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device ID for the AE Cluster without ACP i.e. non-accelerated
 *
 *****************************************************************************/
#define ICP_EP80579_PCI_DEVICE_ID_AE_CLUSTER_NO_ACP 0x502C

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device ID for the AE Cluster with ACP on A0 silicon
 *
 *****************************************************************************/
#define ICP_EP80579_PCI_DEVICE_ID_AE_CLUSTER_A0_WITH_ACP 0x502C

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device ID for the AE Cluster with ACP
 *
 *****************************************************************************/
#define ICP_EP80579_PCI_DEVICE_ID_AE_CLUSTER_WITH_ACP 0x502D

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device ID for the Ring Controller
 *
 *****************************************************************************/
#define ICP_EP80579_PCI_DEVICE_ID_RING_CTRLR 0x503F

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
 * DH89xxCC section
 *****************************************************************************/
/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device IDs for the DH89xxCC
 *
 *****************************************************************************/
#define ICP_DH89xxCC_PCI_DEVICE_ID           0x434
#define ICP_DH89xxCC_PCI_IOV_DEVICE_ID       0x442

/*****************************************************************************
 * C2XXX section
 *****************************************************************************/
/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device IDs for the C2XXX
 *
 *****************************************************************************/
#define ICP_C2XXX_PCI_DEVICE_ID           0x1F18
#define ICP_C2XXX_PCI_IOV_DEVICE_ID       0x1F19
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
#define ADF_DEVICE_NAME_LENGTH          20
#define ADF_CSR_DEVICE_NAME_FORMAT      ("/dev/icp_dev%d_csr")
#define ADF_RING_DEVICE_NAME_FORMAT     ("/dev/icp_dev%d_ring")

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
    adf_isr_mode_intx = 0,
    adf_isr_mode_msi,
    adf_isr_mode_msix
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

#ifdef __KERNEL__
typedef struct icp_accel_pci_info_s
{
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
#endif /* __KERNEL__ */

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
    DEV_DH89XXCC_VF
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

typedef struct adf_hw_device_data_s
{
    adf_hw_device_class_t *dev_class;
    Cpa32U                  instanceId;
    /* all data fields are MANDATORY */
    Cpa32U          maxBars;             /* max number of BARs               */
    Cpa32U          maxNumBanks;         /* max number of banks              */
    Cpa32U          maxNumApBanks;       /* max number of AutoPush banks     */
    Cpa32U          numRingsPerBank;     /* rings per bank                   */
    Cpa32U          numBanksPerAccel;    /* banks per accelerator            */
    Cpa32U          maxNumAccel;         /* max number of accelerators       */
    Cpa32U          maxNumAccelEngines;  /* max number of accelerator engines*/
    Cpa32U          aeClockInMhz;        /* clock speed in Mhz               */
    Cpa32U          sintpfOffset;        /* SINTPF register offset           */
    Cpa32U          userEtringCsrSize;   /* user space etring size           */
    Cpa32U          maxNumVf;            /* max number of virtual functions  */

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

    /* Accelerator/Engine & SKU functions */
    Cpa32U (*getDevFuseOffset)(void);
    Cpa32U (*getDevClkOffset)(void);

    Cpa32U (*getAcceleratorsMask)(Cpa32U fuse);
    Cpa32U (*getAccelEnginesMask)(Cpa32U fuse);
    dev_sku_info_t (*getDevSKU)(Cpa32U NumAccelerators, Cpa32U NumAccelEngines,
                                Cpa32U ClkMask, Cpa32U fuse);

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
    void (*getSmiapfOffsetMask)(struct adf_hw_device_data_s *self,
            Cpa32U *offset, Cpa32U *mask);

    Cpa32U (*irqGetBankNumber)(struct adf_hw_device_data_s *self,
            Cpa32U sintpf);

    Cpa32U (*irqGetAeSource)(struct adf_hw_device_data_s *self, Cpa32U sintpf);

    /* VF interrupt functions */
    Cpa32U (*irqGetBunSource)(struct adf_hw_device_data_s *self,
            Cpa32U vintsou);

    Cpa32U (*irqGetPF2VFSource)(struct adf_hw_device_data_s *self, Cpa32U vintsou);

    /* bank handlers */
    bank_handler (*getBankBhHandler)(Cpa32U bankId);
    bank_handler (*getBankBhPollingHandler)(Cpa32U bankId);

    /* auto-push feature */
    CpaBoolean isAutoPushSupported;
    void (*getApCsrOffsets)(Cpa32U bankId, Cpa32U *nfMask, Cpa32U *nfDest,
            Cpa32U *neMask, Cpa32U *neDest, Cpa32U *apDelay);

    /* PF-to-VF functions */
    Cpa32U (*getVf2PfIntSourceOffset)(void);
    Cpa32U (*getVf2PfBitOffset)(void);
    Cpa32U (*getVf2PfIntMaskOffset)(void);
    Cpa32U (*getVf2PfIntMaskDefault)(void);
    Cpa32U (*getTiMiscIntCtlOffset)(void);
    Cpa32U (*getVIntSrcOffset)(Cpa32U id);
    Cpa32U (*getVIntMskOffset)(Cpa32U id);
    Cpa32U (*getVIntMskDefault)(void);
    Cpa32U (*getPf2VfDbIntOffset)(Cpa32U id);

    /* Reset offset and reset vector */
    void (*getResetOffsets)(Cpa32U *resetOffset, Cpa32U *resetVector);

    /* Uncorrectable error functions */
    CpaStatus (*enableUncoErrInterrupts)(icp_accel_dev_t *accel_dev);
    CpaBoolean (*handleUncoErrInterrupts)(icp_accel_dev_t *accel_dev);

    /* Private Data */
    void *privateData;
} adf_hw_device_data_t;

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
#define GET_VF_PCI_DEV(accel_dev, bundle)          \
        (((icp_accel_pci_vf_info_t*)((accel_dev)->pci_vfs[bundle]))->pDev)
#define GET_VF_DEV(accel_dev, bundle)              \
        (((icp_accel_pci_vf_info_t*)((accel_dev)->pci_vfs[bundle]))->pDev->dev)
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

#if defined (__KERNEL__)
struct accel_dev_s
{
    /* Some generic information */
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
    void            *pQatStats;         /* For QATAL/SAL stats */
    void            *pUserSpaceHandle;  /* For user space proxy */
    void            *ringInfoCallBack;  /* Callback for user space
                                                       ring enabling */
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

    /* pointer to dynamic instance resource manager */
    void            *pInstMgr;
    /* Linked list of accel_devs - for easy traversing */
    struct accel_dev_s  *pNext;
    struct accel_dev_s  *pPrev;
};

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
    struct pci_dev          *pDev;
    icp_accel_dev_t         *accel_dev;
    void                    *tasklet_vf;
    void                    *vf_ring_info_tbl;
    Cpa8U                   attached : 1;
    Cpa8U                   init : 1;
} icp_accel_pci_vf_info_t;

#else /* User space version of accel dev */

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Accelerator Device Structure - userspace representation.
 *
 *****************************************************************************/
struct accel_dev_s
{
    /* Some generic information may be used by all components */
    Cpa32U          accelId;
    Cpa8U           *pAccelName;            /* Name given to accelerator */
    Cpa32U          accelMask;              /* Accelerator mask */
    Cpa32U          aeMask;                 /* Acceleration Engine mask */
    Cpa32U          maxNumAccel;            /* max number of accelerators */
    Cpa32U          accelCapabilitiesMask;  /* Accelerator's capabilities */
    Cpa64U          firmwareVersion;        /* Firmware version */
    Cpa8U           devFileName[ADF_DEVICE_NAME_LENGTH];
    Cpa32S          csrFileHdl;
    Cpa32S          ringFileHdl;
    Cpa8U           *virtConfigSpace;

    /* Component specific fields - cast to relevent layer */
    void            *pCommsHandle;      /* For ADF */
    void            *pSalHandle;        /* For SAL*/
    void            *pQatHandle;        /* For QATAL*/
    void            *pQatStats;         /* For QATAL/SAL stats */
    void            *esram_addr;        /*virtual address for eSRAM. */
    void            *shmem;             /*virtual address for share memory.*/
    /* Status of the device */
    Cpa32U          adfSubsystemStatus;
    /* Device Configuration Info */
    Cpa32U          maxNumBanks;
    Cpa32U          maxNumRingsPerBank;
    Cpa32U          userRingCsrSize;
    dev_sku_info_t  sku;
    /* Linked list of accel_devs - for easy traversing */
    struct accel_dev_s  *pNext;
    struct accel_dev_s  *pPrev;
};
#endif /* defined __KERNEL__ */

#endif /* ICP_ACCEL_HANDLE_H */
