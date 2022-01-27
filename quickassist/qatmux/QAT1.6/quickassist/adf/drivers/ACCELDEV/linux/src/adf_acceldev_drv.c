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
 * @file adf_acceldev_drv.c
 *
 * @description
 *      This file contains the module init/release functions, performs PCI
 *      device driver registration, setup of system resource table and calls
 *      the main acceleration subsystem init and shutdown functions.
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include <Osal.h>
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_drv.h"
#include "adf_isr.h"
#include "adf_cfg.h"
#include "adf_init.h"
#include "adf_ae_fw.h"
#include "adf_ctl_drv.h"
#include "adf_devmgr.h"
#include "adf_user_proxy.h"
#include "adf_dev_csr.h"
#include "adf_dev_ring.h"
#include "adf_dev_bank.h"
#include "adf_config_ctl.h"
#include "adf_platform.h"
#include "adf_bh.h"
#include "adf_pci_ids.h"

static int num_device = 0;
char adf_driver_name[] = ICP_QAT_1_6_DRIVER_NAME"_adf";

extern CpaStatus adf_esramSetConfigInfo(icp_accel_dev_t *pAccelDev,
                                    Cpa64U physAddr,
                                    UARCH_INT virtAddr,
                                    Cpa32U size);


/* Device BAR Offset */
#define ADF_BAR_OFFSET      2
#define PCI_DEV_ID_OFFSET   2
#define PCI_FUNCTION_OFFSET 8

/* Max number of PCI buses in system */
#define MAX_PCI_BUS 256

/* Reset vectors */
#define RESET_GIGE_RESET_VECTOR 0x4000000
#define RESET_GIGE_VECTOR 0x7C000000

#define PPDSTAT_OFFSET 0x7E
#define WAIT_FOR_TRIP_LOOPS 3
#define GIGES_NUM           4
#define MAX_SBR_SLEEP_TIME  1000

/* The PCIe MRRS (Max Read Request Size) field is 3 bits wide but the Silicon
 * implementation in DH895xcc only uses to bottom 2 bits. By default, the
 * Linux PCIe bus driver will set this field to the maximum value
 * (4096 bytes = a bit encoding of 101). Since only the bottom 2 bits are
 * considered by DH895xcc, this gets interpreted as a setting of 256 bytes.
 * This causes extra PCIe traffic for each DMA read/write transfer between
 * memory and the device. The preferred setting for DH895xcc is
 * 1024 bytes (= a bit encoding of 011). */
#define ADF_PCI_MAX_READ_REQUEST_SIZE 1024

/* RIMISCCTL and PFCGCIOSFPRIR offsets for c2xxx device */
#define RIMISCCTL_OFFSET     (0x1A000 + 0xC4)
#define RIMISCCTL_MASK       0x40000000
#define PFCGCIOSFPRIR_OFFSET 0x2C0
#define PFCGCIOSFPRIR_MASK   0xFFFF7FFF

/*Sriov capabilities*/
struct adf_pci_sriov {
        int pos;
        int nres;
        uint32_t cap;
        uint16_t ctrl;
        uint16_t total;
        uint16_t initial;
        uint16_t nr_virtfn;
        uint16_t offset;
        uint16_t stride;
        uint32_t pgsz;
        uint8_t link;
        struct pci_dev *dev;
        struct pci_dev *self;
        struct mutex lock;
        struct work_struct mtask;
        uint8_t __iomem *mstate;
};

extern struct pci_device_id adf_pci_tbl[];

/* This is set to true if ICP_DH89xxCC_PCI_DEVICE_ID or ICP_DH89xxCC_PCI_IOV_DEVICE_ID
   is present in the adf_pci_tbl[]*/
static CpaBoolean gigeWDEnable = CPA_FALSE;

MODULE_DEVICE_TABLE(pci, adf_pci_tbl);

/*
 * Increase module usage counter function
 */
extern int adf_get_module(void);
/*
 * Decrease module usage counter function
 */
extern void adf_put_module(void);

static int adf_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void adf_remove(struct pci_dev *pdev);

static struct pci_driver adf_driver = {
        .id_table = adf_pci_tbl,
        .name = adf_driver_name,
        .probe = adf_probe,
        .remove =  adf_remove
};

/*
 * Module init/exit prototypes
 */
#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(3,10,0))) \
     || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)))
static int __devinit adfdrv_init(void);
#else
/*
 * __devinit macro was removed from the 3.10 kernel (RedHat : 3.8 kernel)
 * as the functionality was no longer relevent
 */
static int adfdrv_init(void);
#endif
static void adfdrv_release(void);
static void adf_free_coherent_buf(icp_accel_dev_t *accel_dev);

/*
 * adf_cleanup_vf
 * Cleans up any system resources used by an vf
 */
static void adf_cleanup_vf(icp_accel_pci_vf_info_t *vf)
{
        if (vf) {
                adf_cleanup_vf_bh(vf);
                ICP_FREE(vf->vf_ring_info_tbl);
                ICP_MUTEX_UNINIT(vf->lock);
                ICP_FREE(vf->lock);
                ICP_FREE(vf);
        }
}

/*
 * adf_cleanup_accel
 * Cleans up any system resources used by an accelerator structure
 */
static int adf_cleanup_accel(icp_accel_dev_t * accel_dev)
{
        int status = 0, i = 0;
        icp_accel_pci_info_t *pci_info = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        int ret = 0;

        if (accel_dev == NULL) {
                ADF_ERROR("NULL accelerator structure\n");
                return -EFAULT;
        }

        if (!accel_dev->pHwDeviceData) {
                ADF_ERROR("Invalid dev config data\n");
                return ret;
        }
        hw_data = accel_dev->pHwDeviceData;

        if (accel_dev->pciAccelDev.pDev) {
                pci_release_regions(accel_dev->pciAccelDev.pDev);
                pci_disable_device(accel_dev->pciAccelDev.pDev);
        }

        if (NULL != accel_dev->pci_vfs) {
                ICP_FREE_NUMA(accel_dev->pci_vfs);
        }

        status = adf_cfgDeviceRemove(accel_dev->accelId);
        if (status) {
                ADF_ERROR("adf_cfgDeviceRemove failed, id =%d\n",
                              accel_dev->accelId);
        }

        /* Remove DMA buffers */
        adf_free_coherent_buf(accel_dev);

        /* unmap the pcidev bars */
        pci_info = &accel_dev->pciAccelDev;
        for (i = 0; i < ICP_PCI_MAX_BARS; i++) {
                if (pci_info->pciBars[i].virtAddr) {
                        pci_iounmap(pci_info->pDev, (void __iomem *)
                                   (UARCH_INT) pci_info->pciBars[i].virtAddr);
                        pci_info->pciBars[i].virtAddr = (UARCH_INT) NULL;
                }
        }

        /* Remove accelerator device from accelerator table */
        status = adf_devmgrRemoveAccelDev(accel_dev);

        if (NULL != accel_dev->pHwDeviceData) {
                /* clean dev config structure if needed                 */
                if (NOT_NULL_FUNCTION(accel_dev->pHwDeviceData->cleanup)) {
                        accel_dev->pHwDeviceData->cleanup(
                            accel_dev->pHwDeviceData);
                }
                ICP_FREE_NUMA(accel_dev->pHwDeviceData);
                accel_dev->pHwDeviceData = NULL;
        }
        if (!hw_data->isArbitrationSupported) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
                ICP_FREE(accel_dev->pci_state);
#else
                if(accel_dev->pci_state)
                        kfree(accel_dev->pci_state);
#endif
        }
        ICP_FREE(accel_dev);
        return status;
}

/*
 * adf_get_dev_node_id
 *
 * Function determines to what physical die the pci dev is connected to.
 */
static int adf_get_dev_node_id(struct pci_dev *pdev)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)) && defined(CONFIG_NUMA)
        if (pdev->dev.numa_node < 0) {
                return 0;
        }
        return (pdev->dev.numa_node);
#else
        unsigned int bus_per_cpu = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24))
        struct cpuinfo_x86 *c = &cpu_data(num_online_cpus()-1);
#else
        struct cpuinfo_x86 *c = &cpu_data[num_online_cpus()-1];
#endif

        ADF_DEBUG("CPU INFO: \n"
                  "processors\t: %u\n"
                  "vendor_id\t: %s\n"
                  "cpu family\t: %d\n"
                  "model\t\t: %u\n"
                  "model name\t: %s\n",
                   num_online_cpus(),
                   c->x86_vendor_id,
                   c->x86,
                   c->x86_model,
                   c->x86_model_id);

        /* if there is only one physical processor don't need
         * to do any further calculations */
        if (c->phys_proc_id == 0)
                return 0;
        bus_per_cpu = MAX_PCI_BUS / (c->phys_proc_id + 1);
        if (0 != bus_per_cpu) {
                return pdev->bus->number / bus_per_cpu;
        }
#endif
        return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
/*
 * adf_vf_probe
 * ICP ADF Device Driver Probe function for vf
 */
static int adf_vf_probe(struct pci_dev *pdev)
{
        struct pci_dev *pdev_pf = pdev->physfn;
        icp_accel_pci_vf_info_t *vf = NULL;
        int dev_nr = 0;
        Cpa32U numRingsPerBank = 0;
        icp_accel_dev_t *accel_dev = adf_devmgrGetAccelDev(pdev_pf);
        adf_hw_device_data_t *hw_data = NULL;
        int ret = 0;

        if (!accel_dev) {
                ADF_ERROR("Can not find PF device %p\n", pdev_pf);
                return -ENODEV;
        }
        if (!accel_dev->pHwDeviceData) {
                ADF_ERROR("Invalid dev config data\n");
                return ret;
        }
        hw_data = accel_dev->pHwDeviceData;

        /* Calculate vf device number */
        dev_nr = ((PCI_SLOT(pdev->devfn) - 1) * PCI_FUNCTION_OFFSET)
                                              + PCI_FUNC(pdev->devfn);

        /* If SRIOV is enabled enable VF in VMM as we can use it as well */
        osalSetPCICapabilitiesOffset(pdev);
        if (pci_enable_device(pdev)) {
                ADF_ERROR("Failed to enable VF\n");
                return -ENOMEM;
        }

        if (pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
                if ((pci_set_dma_mask(pdev, DMA_BIT_MASK(32)))) {

                        ADF_ERROR("No usable DMA configuration in VF,"
                                      "aborting\n");
                        pci_disable_device(pdev);
                        return -EIO;
                } else {
                        pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
                }
        } else {
                pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
        }

        pci_set_master(pdev);

        /* Attach VF to iommu domain */
        if (!hw_data->isArbitrationSupported) {
               if (IS_VF_BUNDLE(dev_nr)) {
                        ret = osalIOMMUAttachDev(&pdev->dev);
                }
        } else {
                ret = osalIOMMUAttachDev(&pdev->dev);
        }
        if (ret) {
                pci_disable_device(pdev);
                ADF_ERROR("Failed to attach VF to iommu domain\n");
                return ret;
        }

        if (accel_dev->pci_vfs[dev_nr]) {
                vf = accel_dev->pci_vfs[dev_nr];
                vf->attached = 1;
                vf->init = 0;
                vf->lock = ICP_MALLOC_GEN(sizeof(ICP_MUTEX));
                ICP_MUTEX_INIT(vf->lock);
                if(!BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                                ADF_STATUS_SYSTEM_STARTED)) {
                        /* Dont enable msix here as we are starting */
                        return SUCCESS;
                }
                /* This means that a guest went down
                 * need to sent ring info updates for all rings
                 * allocated by guest on this vf */
                adf_vf_clean_ring_info(vf);
                /* Decrease module ref counter */
                adf_put_module();
                osalAtomicDec(&accel_dev->usageCounter);
                return adf_enable_bundle_msix(accel_dev, dev_nr);
        }

        vf = ICP_ZALLOC_GEN(sizeof(icp_accel_pci_vf_info_t));
        if (!vf) {
                pci_disable_device(pdev);
                ADF_ERROR("failed to allocate vf structure\n");
                return -ENOMEM;
        }

        numRingsPerBank = GET_NUM_RINGS_PER_BANK(accel_dev);
        vf->vf_ring_info_tbl = ICP_ZALLOC_GEN(numRingsPerBank * sizeof(void*));
        if (!vf->vf_ring_info_tbl) {
                pci_disable_device(pdev);
                ICP_FREE(vf);
                ADF_ERROR("failed to allocate memory for vf ring info tbl\n");
                return -ENOMEM;
        }

        vf->deviceId = dev_nr;
        vf->attached = 1;
        vf->init = 0;
        vf->lock = ICP_MALLOC_GEN(sizeof(ICP_MUTEX));
        ICP_MUTEX_INIT(vf->lock);
        accel_dev->pci_vfs[dev_nr] = (void*)vf;
        vf->accel_dev = accel_dev;
        vf->pDev = pdev;

        if (adf_setup_vf_bh(vf)) {
                pci_disable_device(pdev);
                ADF_ERROR("Failed init bottom half for VF\n");
                adf_cleanup_vf(vf);
        }
        return SUCCESS;
}
#endif

/*
 * adf_free_coherent_buf
 * Frees DMA buffers
 */
static void adf_free_coherent_buf(icp_accel_dev_t *accel_dev)
{
        int i = 0;
        dma_addr_t dma_handle;
        adf_hw_device_data_t *hw_data;

        ICP_CHECK_FOR_NULL_PARAM_VOID(accel_dev);
        ICP_CHECK_FOR_NULL_PARAM_VOID(accel_dev->pHwDeviceData);
        hw_data = accel_dev->pHwDeviceData;

        if(!accel_dev->pHwDeviceData->isETRlessAdminInitCommSupported) {
                if (accel_dev->pQatCpuMem && accel_dev->pQatDmaMem) {
                        for (i = 0; i < GET_MAX_ACCEL(accel_dev); i++) {
                                if (accel_dev->pQatCpuMem[i]) {
                                        dma_handle =
                                        (dma_addr_t)accel_dev->pQatDmaMem[i];

                                        dma_free_coherent(&(GET_DEV(accel_dev)),
                                                        PAGE_SIZE,
                                                        accel_dev->pQatCpuMem[i],
                                                        dma_handle);
                                        accel_dev->pQatCpuMem[i] = NULL;
                                }
                        }
                    ICP_FREE_NUMA(accel_dev->pQatCpuMem);
                    ICP_FREE_NUMA(accel_dev->pQatDmaMem);
                }
        }
        /*
         * Free the coherent DMA mapping for admin/init messages
         * (message slots) if the device supports it
         */
        else {
                if (accel_dev->pMessageSlots) {
                        dma_handle = (dma_addr_t)accel_dev->pMessageSlotsDma;
                        dma_free_coherent(&(GET_DEV(accel_dev)), PAGE_SIZE,
                                        accel_dev->pMessageSlots, dma_handle);

                        accel_dev->pMessageSlots = NULL;
                        accel_dev->pMessageSlotsDma = 0;
                }

                if (accel_dev->pShramConstants) {
                        dma_handle = (dma_addr_t)accel_dev->pShramConstantsDma;
                        dma_free_coherent(&(GET_DEV(accel_dev)), PAGE_SIZE,
                                        accel_dev->pShramConstants,
                                        dma_handle);

                        accel_dev->pShramConstants = NULL;
                        accel_dev->pShramConstantsDma = 0;
                }
        }
}

/*
 * adf_alloc_coherent_buf
 * Allocates DMA buffers for QAT
 */
#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(3,10,0))) \
     || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)))
static int __devinit adf_alloc_coherent_buf(icp_accel_dev_t *accel_dev)
#else
/*
 * __devinit macro was removed from the 3.10 kernel (RedHat : 3.8 kernel)
 * as the functionality was no longer relevent
 */
static int adf_alloc_coherent_buf(icp_accel_dev_t *accel_dev)
#endif
{
        dma_addr_t dma_handle;
        int i = 0;
        Cpa32U maxNumAccel = GET_MAX_ACCEL(accel_dev);

        if (!accel_dev->pHwDeviceData->isETRlessAdminInitCommSupported) {
                accel_dev->pQatCpuMem = (void *)
                        ICP_MALLOC_GEN_NUMA(sizeof(void *) * maxNumAccel,
                        accel_dev->pkg_id);
                if (!accel_dev->pQatCpuMem) {
                        return -ENOMEM;
                }

                accel_dev->pQatDmaMem = (Cpa64U *)
                        ICP_MALLOC_GEN_NUMA(sizeof(Cpa64U) * maxNumAccel,
                        accel_dev->pkg_id);
                if (!accel_dev->pQatDmaMem) {
                        ICP_FREE_NUMA(accel_dev->pQatCpuMem);
                        return -ENOMEM;
                }

                for (i = 0; i < maxNumAccel; i++) {
                        accel_dev->pQatCpuMem[i] = dma_alloc_coherent(
                                                       &(GET_DEV(accel_dev)),
                                                       PAGE_SIZE,
                                                       &dma_handle,
                                                       GFP_KERNEL);


                        if (!accel_dev->pQatCpuMem[i]) {
                            adf_free_coherent_buf(accel_dev);
                                return -ENOMEM;
                        }
                        accel_dev->pQatDmaMem[i] = (Cpa64U)dma_handle;
                }
        }

        /*
         * Create the coherent DMA mapping for admin/init messages
         * (message slots) if the device supports it
         */
        if (accel_dev->pHwDeviceData->isETRlessAdminInitCommSupported) {
                accel_dev->pMessageSlots = 
                            dma_alloc_coherent(&(GET_DEV(accel_dev)),
                                                PAGE_SIZE,
                                                &dma_handle,
                                                GFP_KERNEL);

                accel_dev->pMessageSlotsDma = (Cpa64U) dma_handle;

                if (!accel_dev->pMessageSlots) {
                        adf_free_coherent_buf(accel_dev);
                        return -ENOMEM;
                }

                /*
                * Create the coherent DMA mapping for Shram Constants Page
                */
                accel_dev->pShramConstants = 
                            dma_alloc_coherent(&(GET_DEV(accel_dev)),
                                                PAGE_SIZE,
                                                &dma_handle,
                                                GFP_KERNEL);

                accel_dev->pShramConstantsDma = (Cpa64U) dma_handle;

                if (!accel_dev->pShramConstants) {
                        adf_free_coherent_buf(accel_dev);
                        return -ENOMEM;
                }
        }
        return SUCCESS;
}
/*
 * adf_probe
 * ICP ADF Device Driver Probe function
 */
#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(3,10,0))) \
     || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)))
static int __devinit adf_probe(struct pci_dev *pdev,
                               const struct pci_device_id *ent)
#else
/*
 * __devinit macro was removed from the 3.10 kernel (RedHat : 3.8 kernel)
 * as the functionality was no longer relevent
 */
static int adf_probe(struct pci_dev *pdev,
                               const struct pci_device_id *ent)
#endif
{
        int i = 0, status = 0, bar_index = 0;
        uint8_t rev_id = 0;
        icp_accel_dev_t *accel_dev = NULL;
        icp_accel_pci_info_t *pci_info = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        uint32_t sramBarId = 0;
        Cpa32U clk = 0;
        Cpa32U fusectl_offset = 0;
        Cpa32U accfunc_offset = 0, accfunc = 0;
        int aer_cap = 0;

        /* Ensure have valid device */
        switch (ent->device) {
        case ICP_DH89xxCC_PCI_DEVICE_ID:
        case ICP_DH895xCC_PCI_DEVICE_ID:
        case ICP_C2XXX_PCI_DEVICE_ID:
                break;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
        case ICP_DH89xxCC_PCI_IOV_DEVICE_ID:
        case ICP_DH895xCC_PCI_IOV_DEVICE_ID:
        case ICP_C2XXX_PCI_IOV_DEVICE_ID:
                return adf_vf_probe(pdev);
                break;
#endif
        default:
                ADF_ERROR("invalid device 0x%x found.\n", ent->device);
        }

        if (ADF_MAX_DEVICES == num_device) {
                ADF_ERROR("Only support upto %d devices\n", ADF_MAX_DEVICES);
                return -EIO;
        }

        /* Allocate memory for the accel device. */
        accel_dev = ICP_ZALLOC_GEN(sizeof(icp_accel_dev_t));
        if (NULL == accel_dev) {
                ADF_ERROR("failed to allocate accel structure\n");
                return -ENOMEM;
        }
        /* Set accel id */
        accel_dev->accelId = num_device;

        /* Create device configuration table */
        if (SUCCESS != adf_cfgDeviceAdd(num_device)) {
                adf_cleanup_accel(accel_dev);
                return -ENOMEM;
        }
        pci_info = &accel_dev->pciAccelDev;

        /* Get device node */
        accel_dev->pkg_id = adf_get_dev_node_id(pdev);

        /* Allocate memory for device configuration structure */
        hw_data = ICP_MALLOC_GEN_NUMA(sizeof(adf_hw_device_data_t),
                                                accel_dev->pkg_id);
        if (NULL == hw_data) {
                ADF_ERROR("Failed to allocate memory for device config\n");
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        ICP_MEMSET(hw_data, '\0', sizeof(adf_hw_device_data_t));
        accel_dev->pHwDeviceData = hw_data;
        switch (ent->device) {
        case ICP_DH89xxCC_PCI_DEVICE_ID:
                adf_set_hw_data_dh89xxcc((void *)accel_dev->pHwDeviceData);
                break;
        case ICP_C2XXX_PCI_DEVICE_ID:
                adf_c2xxx_set_hw_data((void *)accel_dev->pHwDeviceData);
                break;
        case ICP_DH895xCC_PCI_DEVICE_ID:
                adf_set_hw_data_dh895xcc((void *)accel_dev->pHwDeviceData);
                break;
        default:
                ADF_ERROR("Unsupported device id\n");
                adf_cleanup_accel(accel_dev);
                 return -EIO;
        }
        /* setup vf specific data*/
        accel_dev->virtualization.virtualized = CPA_FALSE;
        accel_dev->maxNumVf = hw_data->maxNumVf;

        /* Initialize device config */
        if (NOT_NULL_FUNCTION(hw_data->init)) {
                status = hw_data->init(hw_data, accel_dev->pkg_id);
                if (CPA_STATUS_FAIL == status) {
                        ADF_ERROR("Failed to initialize hw_data init\n");
                        adf_cleanup_accel(accel_dev);
                        return -EIO;
                }
        }

        /* Retrieve the revision id and add it to the PCI info structure */
        pci_read_config_byte(pdev, ICP_PRID_OFFSET, &rev_id);
        pci_info->revisionId = rev_id;

        /* Read the fusectl register */
        if (NOT_NULL_FUNCTION(hw_data->getDevFuseOffset)) {
                fusectl_offset = hw_data->getDevFuseOffset();
                pci_read_config_dword(pdev, fusectl_offset,
                                      &pci_info->devFuses);
        }

        /* Get Accelerators and Accelerators Engines masks */
        accel_dev->accelMask =
            hw_data->getAcceleratorsMask(pci_info->devFuses);
        accel_dev->aeMask =
            hw_data->getAccelEnginesMask(pci_info->devFuses);

        /* Read the clkctl register */
        if (NOT_NULL_FUNCTION(hw_data->getDevClkOffset)) {
                Cpa32U ckl_offset = hw_data->getDevClkOffset();
                pci_read_config_dword(pdev, ckl_offset, &clk);
        }
        /* Get the device SKU */
        if (NOT_NULL_FUNCTION(hw_data->getDevSKU)) {
            accel_dev->sku =
                hw_data->getDevSKU(hw_data->getNumAccelerators(hw_data,
                                   accel_dev->accelMask),
                                   hw_data->getNumAccelEngines(hw_data,
                                   accel_dev->aeMask), clk,
                                   pci_info->devFuses);
        }

        /* Setup PCI domain information */
        pci_info->pciDomain.domain = pci_domain_nr(pdev->bus);
        pci_info->pciDomain.bus = pdev->bus->number;
        pci_info->pciDomain.slot = PCI_SLOT(pdev->devfn);
        pci_info->pciDomain.function = PCI_FUNC(pdev->devfn);

        /*
         * If the device has no acceleration engines the
         * device needs to be freed and ignored.
         */
        if (!accel_dev->aeMask || !accel_dev->accelMask ||
                            ((~accel_dev->aeMask) & 0x01)) {
                ADF_ERROR("%04x:%02x:%02x.%d: "
                          "There is no acceleration associated "
                          "with this device.\n",
                          pci_info->pciDomain.domain,
                          pci_info->pciDomain.bus,
                          pci_info->pciDomain.slot,
                          pci_info->pciDomain.function);
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        num_device++;
        /* enable PCI device */
        if (pci_enable_device(pdev)) {
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        /* Setup PCI Info structure for upper layers */
        pci_info->pDev = pdev;
        pci_info->deviceId = ent->device;
        pci_info->irq = pdev->irq;

        /* set dma identifier */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
        #define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
#endif
        if (pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
                if ((pci_set_dma_mask(pdev, DMA_BIT_MASK(32)))) {

                        ADF_ERROR("No usable DMA configuration,"
                                      "aborting\n");
                        adf_cleanup_accel(accel_dev);
                        return -EIO;
                }
                else {
                        pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
                }
        }
        else {
                pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
        }

        status = adf_alloc_coherent_buf(accel_dev);
        if (SUCCESS != status) {
                ADF_ERROR("Can't allocate DMA buffers");
                adf_cleanup_accel(accel_dev);
                return status;
        }

        if ((status = pci_request_regions(pdev, adf_driver_name))) {
                adf_cleanup_accel(accel_dev);
                return status;
        }

        /* Get accelerator functions */
        accfunc = 0;
        if (NOT_NULL_FUNCTION(hw_data->getAccFuncOffset)) {
                accfunc_offset = hw_data->getAccFuncOffset();
                pci_read_config_dword(pdev, accfunc_offset, &accfunc);
        }
        accel_dev->accelCapabilitiesMask = hw_data->getAccFuncMask(accfunc);
        /* Set the type of the device */
        ICP_STRNCPY(accel_dev->type,
                accel_dev->pHwDeviceData->dev_class->name,
                ADF_DEVICE_TYPE_LENGTH);

        for (i = 0; i < GET_MAX_BARS(accel_dev); i++) {
                bar_index = i * ADF_BAR_OFFSET;

                /* check if bar available and reset pciBars entry if not    */
                if (CPA_FALSE == hw_data->isBarAvailable(hw_data, i)) {
                        pci_info->pciBars[i].baseAddr = 0;
                        pci_info->pciBars[i].virtAddr = 0;
                        pci_info->pciBars[i].size = 0;
                        pci_info->numBars++;
                        continue;
                }

                /* Find all the device's BARS */
                pci_info->pciBars[i].baseAddr = pci_resource_start(pdev,
                                                                   bar_index);
                if (!pci_info->pciBars[i].baseAddr) {
                        break;
                }
                pci_info->pciBars[i].size = pci_resource_len(pdev, bar_index);
                pci_info->numBars++;

                ADF_DEBUG("Found BAR %d size 0x%X\n", i, pci_info->pciBars[i].size);

                /* Map the IO Memory */
                pci_info->pciBars[i].virtAddr =
                             (UARCH_INT) pci_iomap(pci_info->pDev, bar_index, 0);
                if (!pci_info->pciBars[i].virtAddr) {
                        ADF_ERROR("Failed to map BAR %d region\n",i);
                        status = FAIL;
                }
                ADF_DEBUG("PF v_addr 0x%llX ph_addr 0x%llX \n",
                            (Cpa64U)pci_info->pciBars[i].virtAddr,
                            (Cpa64U)pci_info->pciBars[i].baseAddr);
        }

        if (SUCCESS != status) {
                adf_cleanup_accel(accel_dev);
                return status;
        }

        pcie_set_readrq(pdev, ADF_PCI_MAX_READ_REQUEST_SIZE);
        pci_set_master(pdev);
        /*
         * Enable advanced error reporting
         */
        aer_cap = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_ERR);
        if (0 != aer_cap) {
                status = adf_enable_aer(accel_dev, (void *)&adf_driver);
                if (status) {
                        ADF_ERROR("Failed to enable advanced error"
                                  " reporting\n");
                        adf_cleanup_accel(accel_dev);
                        return -EIO;
                }
        }
        else {
                ADF_PRINT("No support for advanced error reporting\n");
        }

        /*
         * Enable interrupts to the IA by setting the appropriate bits
         * in the SMIA register
         */
        hw_data->setSmiapfOffsetMask(hw_data, pci_info);

        /* Check if SRAM ar exists */
        if (NOT_NULL_FUNCTION(hw_data->getSramBarId)) {
                adf_esram_data_t esram_info = {0};
                /* Set eSRAM configuration
                 * Note: used predefined actual base physical address in CPP
                 * space from the EAS instead of the one returned by OS */
                sramBarId = hw_data->getSramBarId(hw_data);

                /* get ESRAM information from device config */
                hw_data->getEsramInfo(hw_data, &esram_info);
                status = adf_esramSetConfigInfo(accel_dev,
                                    esram_info.sramAeAddr,
                                    pci_info->pciBars[sramBarId].virtAddr,
                                    esram_info.sramAeSize);
        }
        else {
                status = adf_esramSetConfigInfo(accel_dev, 0, 0, 0);
        }

        if (status) {
                ADF_ERROR("Failed to set eSRAM info\n");
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        /* Allocate buffers for virtual functions */
        if (hw_data->maxNumVf != 0) {
                Cpa32U pci_vfs_size = 0;
                void *pci_vfs = NULL;

                pci_vfs_size = (sizeof(void*) * hw_data->maxNumVf);
                pci_vfs = ICP_MALLOC_GEN_NUMA(pci_vfs_size, accel_dev->pkg_id);
                if (NULL == pci_vfs) {
                        ADF_ERROR("Failed to allocate VF buffer\n");
                        adf_cleanup_accel(accel_dev);
                        return -EIO;
                }
                ICP_MEMSET(pci_vfs, 0, pci_vfs_size);
                accel_dev->pci_vfs = pci_vfs;
        }

        /* Add accel device to table */
        status = adf_devmgrAddAccelDev(accel_dev);
        if (status) {
                ADF_ERROR("Failed to add new accelerator device.\n");
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        status = adf_csr_chrdev_create(accel_dev->accelId);
        if (SUCCESS != status) {
                ADF_ERROR("Failed to add csr device %d\n",
                                      accel_dev->accelId);
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        status = adf_ring_chrdev_create(accel_dev->accelId);
        if (SUCCESS != status) {
                ADF_ERROR("Failed to add ring device %d\n",
                                       accel_dev->accelId);
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        status = adf_bank_chrdev_create(accel_dev->accelId);
        if (SUCCESS != status) {
                ADF_ERROR("Failed to add bank devices %d\n",
                                        accel_dev->accelId);
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        /* Set usage counter to zero */
        osalAtomicSet(0, &accel_dev->usageCounter);

        accel_dev->pci_state = osalPCIStateStore(pdev, accel_dev->pkg_id);

        if (!accel_dev->pci_state) {
                ADF_ERROR("Failed to store pci state for device %d\n",
                                                  accel_dev->accelId);
                adf_cleanup_accel(accel_dev);
                return -ENOMEM;
        }
        pci_info->state = ICP_ACCEL_STATE_INITIALIZED;
        return status;
}

/*
 * adf_restore_dev
 * Brings device to state just after probe - ready to be started.
 */
int adf_restore_dev(icp_accel_dev_t *accel_dev)
{
        int gige_ret = 0;
        struct pci_dev *gige = NULL;
        uint16_t i = 0;

        int ret = 0, pending = 0, ctr = 0;
        struct pci_dev *pdev = accel_dev->pciAccelDev.pDev;
        struct pci_dev *parent = pdev->bus->self;
        uint32_t pmisc_bar_id = 0;
        uint32_t *pmisc_bar_addr = NULL;
        uint16_t ppdstat = 0, bridge_ctl = 0;
        uint32_t resetOffset=0, resetVector=0;
        icp_accel_pci_info_t *pci_info = &accel_dev->pciAccelDev;
        adf_hw_device_data_t *hw_data = accel_dev->pHwDeviceData;

        pmisc_bar_id = hw_data->getMiscBarId(hw_data);
        pmisc_bar_addr = (void*)pci_info->pciBars[pmisc_bar_id].virtAddr;

        ADF_PRINT("Resetting device icp_dev%d\n", accel_dev->accelId);
        /* Reset the device */
        if (NULL == parent) {
        /* In VM pdev->bus->self bus will be NULL
         * update dev info from accel_dev
         */
            parent = pdev;
        }
        if (!parent) {
                ADF_ERROR("Can not issue secondary bus reset\n");
                ADF_ERROR("Trying FLR\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
                ret = __pci_reset_function(pdev);
                if (ret) {
                        ADF_ERROR("Could not reset device\n");
                        return ret;
                }
#endif
                hw_data->getResetOffsets(&resetOffset, &resetVector);

                /* Reset everything but endpoint and GigEs */
                ICP_ADF_CSR_WR(pmisc_bar_addr, resetOffset, resetVector);
                msleep(hw_data->msleep_time);
                ICP_ADF_CSR_WR(pmisc_bar_addr, resetOffset, 0);
                msleep(hw_data->msleep_time);
        }
        else {
                if(CPA_TRUE == hw_data->isGigeWatchdogSupported) {
                        /* Notify gige watchdog if it is running */
                        gige_ret = adf_gige_notify_restarting_dev(accel_dev);
                }
                /* Reset endpoint using secondary bus reset.
                 * Wait for all transactions first. */
                do {
                        pci_read_config_word(pdev, PPDSTAT_OFFSET, &ppdstat);
                        pending = ppdstat & PCI_EXP_DEVSTA_TRPND;
                        if (pending)
                                msleep(hw_data->msleep_trip_time);
                } while(pending && ctr++ < WAIT_FOR_TRIP_LOOPS);
                if (pending) {
                        ADF_ERROR("Transaction still in progress. " \
                                  "Proceeding with reset\n");
                }
                if(CPA_TRUE == hw_data->isGigeWatchdogSupported) {
                        /* Reset GigEs */
                        for (i = 1; i < GIGES_NUM + 1 &&
                                    gige_ret != GIGE_WATCHDOG_NOT_RUNNING;
                                    i++) {
                                gige = pci_get_slot(pdev->bus,
                                             PCI_DEVFN(PCI_SLOT(pdev->devfn),
                                                   PCI_FUNC(pdev->devfn) + i));
                                if (gige) {
                                        pci_save_state(gige);
                                }
                        }
                }

                /* Reset endpoint using secondary bus reset. */
                pci_read_config_word(parent, PCI_BRIDGE_CONTROL, &bridge_ctl);
                bridge_ctl |= PCI_BRIDGE_CTL_BUS_RESET;
                pci_write_config_word(parent, PCI_BRIDGE_CONTROL, bridge_ctl);
                msleep(hw_data->msleep_sbr_time);
                bridge_ctl &= ~PCI_BRIDGE_CTL_BUS_RESET;
                pci_write_config_word(parent, PCI_BRIDGE_CONTROL, bridge_ctl);

                /* Wait for the device to reset */
                ctr = 0;
                do {
                        ctr += hw_data->msleep_sbr_time;
                        msleep(hw_data->msleep_sbr_time);
                        pci_read_config_word(pdev, PPDSTAT_OFFSET, &ppdstat);
                        pending = ppdstat & PCI_EXP_DEVSTA_TRPND;
                } while(pending && ctr < MAX_SBR_SLEEP_TIME);

                if(CPA_TRUE == hw_data->isGigeWatchdogSupported) {
                        if (gige_ret != GIGE_WATCHDOG_NOT_RUNNING) {
                                ICP_ADF_CSR_WR(pmisc_bar_addr, resetOffset,
                                                     RESET_GIGE_RESET_VECTOR);
                                msleep(hw_data->msleep_time);
                                ICP_ADF_CSR_WR(pmisc_bar_addr, resetOffset,
                                                           RESET_GIGE_VECTOR);
                                msleep(hw_data->msleep_time);
                        }
                        for (i = 1; i < GIGES_NUM + 1 &&
                                gige_ret != GIGE_WATCHDOG_NOT_RUNNING; i++) {
                                gige = pci_get_slot(pdev->bus,
                                            PCI_DEVFN(PCI_SLOT(pdev->devfn),
                                            PCI_FUNC(pdev->devfn) + i));
                                if (gige) {
                                        pci_restore_state(gige);
                                }
                        }
                }
        }

        /* Restore dev state */
        ret = osalPCIStateRestore(pdev, accel_dev->pci_state);
        if (OSAL_SUCCESS == ret) {
                /* Enable bus master and sets SmiapfOffset Mask */
                adf_enable_busmaster(pdev);
                /* Enable interrupts to the IA */
                hw_data->setSmiapfOffsetMask(hw_data, pci_info);
        }
       return ret;
}

/* adf_enable_busmaster
 * This function enables the BusMaster of the VFs if the VF device is enabled
 * The Busmaster is disabled after a reset.This function enables the
 * busmaster bit in the Control Register
 */
void adf_enable_busmaster(struct pci_dev *pdev) {
        int i = 0;
        icp_accel_pci_vf_info_t *vf = NULL;
        adf_hw_device_data_t *hw_data = NULL;
        icp_accel_dev_t *accel_dev = NULL;

        ICP_CHECK_FOR_NULL_PARAM_VOID(pdev);

        accel_dev = adf_devmgrGetAccelDev(pdev);
        ICP_CHECK_FOR_NULL_PARAM_VOID(accel_dev);

        hw_data = accel_dev->pHwDeviceData;
        ICP_CHECK_FOR_NULL_PARAM_VOID(hw_data);

        for (i = 0; i < hw_data->maxNumVf; i++) {
                vf = accel_dev->pci_vfs[i];
                if (!vf) {
                        continue;
                }
                else {
                        /* Checks if the VF device is enabled */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30))
                        if (pci_is_enabled(vf->pDev)) {
#elif ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)) && \
       (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,29))) 
                        if (pdev->enable_cnt.counter > 0) {
/* (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,19) */
#else
                        if (pdev->is_enabled != 0) {
#endif
                                /*Sets the Busmaster bit in ctrl reg*/
                                pci_set_master(vf->pDev);
                        }
                }
        }
}

/*
 * adf_remove
 */
static void adf_remove(struct pci_dev *pdev)
{
        int status = 0, i = 0;
        icp_accel_dev_t *accel_dev = NULL;
        icp_accel_pci_info_t *pci_info = NULL;
        icp_accel_pci_vf_info_t *vf = NULL;
        adf_hw_device_data_t  *hw_data = NULL;
        int aer_cap = 0;

        if (!pdev) {
                ADF_ERROR("%s(): pdev is NULL\n", __FUNCTION__);
                return;
        }

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
        if(ICP_DH89xxCC_PCI_IOV_DEVICE_ID == pdev->device ||
           ICP_C2XXX_PCI_IOV_DEVICE_ID == pdev->device ||
           ICP_DH895xCC_PCI_IOV_DEVICE_ID == pdev->device) {
                /* The VFs will be cleaned when PF will be removed.
                 * just need to disable MSIX IRQ for the bundle.
                 * It will be handled as MSI by VF */
                struct pci_dev *pdev_pf = pdev->physfn;
                int dev_nr = 0;

                /* Disable VF in VMM */
                pci_disable_device(pdev);

                /* Detache VF from iommu domain */
                osalIOMMUDetachDev(&pdev->dev);
                accel_dev = adf_devmgrGetAccelDev(pdev_pf);
                if(!accel_dev) {
                        ADF_ERROR("Can not find PF device "
                                "for given VF\n");
                        return;
                }

                dev_nr = ((PCI_SLOT(pdev->devfn) - 1) * PCI_FUNCTION_OFFSET)
                                                       + PCI_FUNC(pdev->devfn);

                if(!BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                                ADF_STATUS_SYSTEM_STARTED)) {
                        /*Clear VFs when going down*/
                        hw_data = accel_dev->pHwDeviceData;
                        for (i = 0; i < hw_data->maxNumVf; i++) {
                                vf = accel_dev->pci_vfs[i];
                                if (!vf) {
                                        continue;
                                }
                                adf_cleanup_vf(vf);
                                accel_dev->pci_vfs[i] = NULL;
                        }
                        /* Dont disable msix as we are going down */
                        return;
                }

                /* Disable MSIX for the bundle.
                 * It will be handled by the VF now */
                adf_disable_bundle_msix(accel_dev, dev_nr);
                vf = accel_dev->pci_vfs[dev_nr];
                vf->attached = 0;
                vf->init = 0;

                /* Increase module ref counter */
                adf_get_module();
                osalAtomicInc(&accel_dev->usageCounter);
                return;
        }
#endif

        /* Lookup the accel_dev assoc with this pdev */
        accel_dev = adf_devmgrGetAccelDev(pdev);
        if (NULL == accel_dev) {
                ADF_ERROR("Can not find accel device\n");
                return;
        }

        if (pdev == accel_dev->pciAccelDev.pDev) {
                accel_dev->pciAccelDev.state = ICP_ACCEL_STATE_UNINITIALIZED;
        }

        hw_data = accel_dev->pHwDeviceData;
        if(!hw_data) {
            ADF_ERROR("Can not find hw_data for accel device\n");
            return;
        }

        /* Clean all VF */
        for (i = 0; i < hw_data->maxNumVf; i++) {
                vf = accel_dev->pci_vfs[i];
                if (!vf) {
                        continue;
                }
                adf_cleanup_vf(vf);
                accel_dev->pci_vfs[i] = NULL;
        }
        pci_info = &accel_dev->pciAccelDev;
        pci_info->irq = 0;

        /* unmap the pcidev bars */
        for (i = 0; i < GET_MAX_BARS(accel_dev); i++) {
                if (pci_info->pciBars[i].virtAddr) {
                        pci_iounmap(pci_info->pDev, (void __iomem *)
                                   (UARCH_INT) pci_info->pciBars[i].virtAddr);
                        pci_info->pciBars[i].virtAddr = (UARCH_INT) NULL;
                }
        }

        aer_cap = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_ERR);
        if (0 != aer_cap) {
                /* Disable advanced error reporting */
                adf_disable_aer(accel_dev);
        }

        /* Cleanup the transport Data */
        adf_cleanup_ETR_Data(accel_dev);

        /* Free acces device */
        status = adf_cleanup_accel(accel_dev);
        if (status) {
                ADF_ERROR("adf_cleanup_accel failure\n");
        }
        num_device--;
}

/*
 * adfdrv_init
 * ICP ADF Module Init function
 */
#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(3,10,0))) \
     || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)))
static int __devinit adfdrv_init()
#else
/*
 * __devinit macro was removed from the 3.10 kernel (RedHat : 3.8 kernel)
 * as the functionality was no longer relevent
 */
static int adfdrv_init()
#endif
{
        int status = 0;
        int i = 0;

        /* Allocate Resource Config table */
        status = adf_cfgInit();

        /*Check whether the GIGE Watchdog should used for devices
          which support it*/
        for (i = 0; adf_pci_tbl[i].device !=0; i++) {
                if((ICP_DH89xxCC_PCI_DEVICE_ID == adf_pci_tbl[i].device) ||
                   (ICP_DH89xxCC_PCI_IOV_DEVICE_ID == adf_pci_tbl[i].device)) {
                        gigeWDEnable = CPA_TRUE;
                        break;
                }
        }

        status = adf_csr_chrdev_register();
        if (status) {
                ADF_ERROR("failed to register adf csr device driver\n");
                return status;
        }

        status = adf_ring_chrdev_register();
        if (status) {
                ADF_ERROR("failed to register adf ring device driver\n");
                adf_csr_chrdev_unregister();
                return status;
        }

        status = adf_bank_chrdev_register();
        if (status) {
                ADF_ERROR("failed to register adf bank device driver\n");
                adf_csr_chrdev_unregister();
                adf_ring_chrdev_unregister();
                return status;
        }

        /*
         * Register the adf ctl character device driver
         * for dynamic configuration of the system resource
         * variables via the adf_ctl user space program
         */
        status = register_adf_ctl_device_driver();
        if (status) {
                ADF_ERROR("failed to register adf ctrl device driver\n");
                adf_csr_chrdev_unregister();
                adf_ring_chrdev_unregister();
                adf_bank_chrdev_unregister();
                return status;
        }

        /*
         * Register the adf processes character device driver
         */
        status = register_adf_processes_device_driver();
        if (status) {
                ADF_ERROR("failed to register adf processes device driver\n");
                unregister_adf_ctl_device_driver();
                adf_csr_chrdev_unregister();
                adf_ring_chrdev_unregister();
                adf_bank_chrdev_unregister();
                return status;
        }

#ifdef ICP_SRIOV
         /*
         * Register the adf_pfvfcomms character device driver
         */
        status = register_adf_pfvfcomms_device_driver();
        if (status) {
                ADF_ERROR("failed to register adf_pfvfcomms device driver\n");
                unregister_adf_processes_device_driver();
                unregister_adf_ctl_device_driver();
                adf_csr_chrdev_unregister();
                adf_ring_chrdev_unregister();
                adf_bank_chrdev_unregister();
                return status;
        }

#endif
        if(CPA_TRUE == gigeWDEnable){
            status = register_adf_gige_wd_device_driver();
            if (status) {
                ADF_ERROR("failed to register adf gige wd device driver\n");
#ifdef ICP_SRIOV 
                unregister_adf_pfvfcomms_device_driver();
#endif
                unregister_adf_ctl_device_driver();
                unregister_adf_processes_device_driver();
                adf_csr_chrdev_unregister();
                adf_ring_chrdev_unregister();
                adf_bank_chrdev_unregister();
                return status;
            }
        }

        status = pci_register_driver(&adf_driver);
        if (status) {
                ADF_ERROR("failed call to pci_register_driver\n");
#ifdef ICP_SRIOV
                unregister_adf_pfvfcomms_device_driver();
#endif
                unregister_adf_ctl_device_driver();
                unregister_adf_processes_device_driver();
                if(CPA_TRUE == gigeWDEnable) {
                        unregister_adf_gige_wd_device_driver();
                }
                adf_csr_chrdev_unregister();
                adf_ring_chrdev_unregister();
                adf_bank_chrdev_unregister();
                return status;
        }
        return SUCCESS;
}

/*
 * adfdrv_release
 * ADF Module release function
 */
static void adfdrv_release()
{
        adf_bank_chrdev_unregister();
        adf_ring_chrdev_unregister();
        adf_csr_chrdev_unregister();
        unregister_adf_ctl_device_driver();
        unregister_adf_processes_device_driver();
#ifdef ICP_SRIOV
        unregister_adf_pfvfcomms_device_driver();
#endif
        if(CPA_TRUE == gigeWDEnable){
            unregister_adf_gige_wd_device_driver();
        }
        pci_unregister_driver(&adf_driver);
        adf_cfgUninit();

}

int __init driver_module_init()
{
        return adfdrv_init();
}

void driver_module_exit()
{
        adfdrv_release();
}
