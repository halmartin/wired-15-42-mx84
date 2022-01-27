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

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_drv.h"
#include "adf_cfg.h"
#include "adf_init.h"
#include "adf_ae_fw.h"
#include "icp_platform.h"
#include "adf_ctl_drv.h"
#include "adf_devmgr.h"
#include "adf_user_proxy.h"
#include "adf_dev_csr.h"
#include "adf_dev_ring.h"
#include "adf_config_ctl.h"
#include "adf_platform.h"
#include "icp_adf_init.h"
#include "adf_drv_sriov.h"

static int num_device = 0;
char adf_driver_name[] = "adfvf";

extern CpaStatus adf_esramSetConfigInfo(icp_accel_dev_t *pAccelDev,
                                    Cpa64U physAddr,
                                    UARCH_INT virtAddr,
                                    Cpa32U size);

#define ADF_SYSTEM_DEVICE(device_id) \
        { PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id) }

/* Device BAR Offset */
#define ADF_BAR_OFFSET      2

/*
 * PCI Infrastructure prototypes/definitions
 */
static struct pci_device_id adf_pci_tbl[] = {
        ADF_SYSTEM_DEVICE( ICP_DH89xxCC_PCI_IOV_DEVICE_ID ),
        ADF_SYSTEM_DEVICE( ICP_C2XXX_PCI_IOV_DEVICE_ID ),
        {0,}
};

MODULE_DEVICE_TABLE(pci, adf_pci_tbl);

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
#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))) \
     || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)))
static int __devinit adfdrv_init(void);
#else
/*
 * __devinit macro was removed from the 3.10 kernel (RedHat: 3.8 kernel)
 * as the functionality was no longer relevent
 */
static int adfdrv_init(void);
#endif
static void adfdrv_release(void);

/*
 * adf_pf2vf_bh_handler
 * Bottom half handles for PF2VF irq
 */
void adf_pf2vf_bh_handler(void* handle)
{
        uint32_t *pmisc_bar_addr = NULL, mask = 0;
        icp_accel_dev_t *accel_dev = (icp_accel_dev_t*) handle;
        icp_accel_pci_info_t *pci_info = &accel_dev->pciAccelDev;
        adf_hw_device_data_t  *hw_data = NULL;
        Cpa32U pmisc_bar_id = 0;
        Cpa32U defVfIntMask = 0;

        hw_data = accel_dev->pHwDeviceData;
        pmisc_bar_id = hw_data->getMiscBarId(hw_data);

        pmisc_bar_addr = (void*)
                         pci_info->pciBars[pmisc_bar_id].virtAddr;

        hw_data = accel_dev->pHwDeviceData;

        /* Read the message from VF */
        mask = ICP_ADF_CSR_RD(pmisc_bar_addr,
                hw_data->getPf2VfDbIntOffset(0));
        /* Clean interrupt */
        ICP_ADF_CSR_WR(pmisc_bar_addr,
                hw_data->getPf2VfDbIntOffset(0), 0);
        /* Disable IRQ for this VF */
        defVfIntMask = hw_data->getVIntMskDefault();
        ICP_ADF_CSR_WR(pmisc_bar_addr,
            hw_data->getVIntMskOffset(0), defVfIntMask);

        /* Get msg type */
        if (ICP_PMISCBAR_PF2VFMSGTYPERESTARTING ==
                                     adf_pf2vf_get_msg_type(mask)) {
            /* Reset device and wait for result */
            adf_subsystemRestarting(accel_dev);
            adf_subsystemStop(accel_dev);

            /* this is restarting event from PF
             * so write ack and enable back the vf irq */
            mask = ICP_PMISCBAR_PF2VFMSGACK;

            ICP_ADF_CSR_WR(pmisc_bar_addr,
                    hw_data->getPf2VfDbIntOffset(0), mask);
            /* Enable IRQ for this VF */
            ICP_ADF_CSR_WR(pmisc_bar_addr,
                hw_data->getVIntMskOffset(0), 0);
            return;

        }
        ADF_ERROR("not request msg\n");
        /* enable IRQ from this VF */
        ICP_ADF_CSR_WR(pmisc_bar_addr,
            hw_data->getVIntMskOffset(0), 0);
        /* Send ring info update error to VF */
        ICP_ADF_CSR_WR(pmisc_bar_addr,
            hw_data->getPf2VfDbIntOffset(0),
                ICP_PMISCBAR_PF2VFMSGERR);
        return;
}

/*
 * adf_cleanup_accel
 * Cleans up any system resources used by an accelerator structure
 */
static int adf_cleanup_accel(icp_accel_dev_t * accel_dev)
{
        int status = 0;

        if (accel_dev == NULL) {
                ADF_ERROR("NULL accelerator structure\n");
                return -EFAULT;
        }

        if (accel_dev->pciAccelDev.pDev) {
                pci_release_regions(accel_dev->pciAccelDev.pDev);
                pci_disable_device(accel_dev->pciAccelDev.pDev);
        }

        status = adf_cfgDeviceRemove(accel_dev->accelId);
        if (status) {
                ADF_ERROR("adf_cfgDeviceRemove failed, id =%d\n",
                              accel_dev->accelId);
        }

        /* Remove accelerator device from accelerator table */
        status = adf_devmgrRemoveAccelDev(accel_dev);
        if (status) {
                ADF_ERROR("failed to remove device from accel table\n");
        }

        if (NULL != accel_dev->pHwDeviceData){
            /* clean dev config structure if needed                         */
            if (NOT_NULL_FUNCTION(accel_dev->pHwDeviceData->cleanup)){
                accel_dev->pHwDeviceData->cleanup(accel_dev->pHwDeviceData);
            }
            ICP_FREE_NUMA(accel_dev->pHwDeviceData);
            accel_dev->pHwDeviceData = NULL;
        }

        ICP_FREE(accel_dev);
        return status;
}

/*
 * adf_sriov_enable
 * Enable accel dev virtualization
 */
int adf_enable_sriov(icp_accel_dev_t *accel_dev)
{
        return 0;
}

/*
 * adf_sriov_disable
 * Disable accel dev virtualization
 */
void adf_disable_sriov(icp_accel_dev_t *accel_dev)
{
        return;
}

/*
 * adf_probe
 * ICP ADF Device Driver Probe function
 */
#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))) \
     || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)))
static int __devinit adf_probe(struct pci_dev *pdev,
                               const struct pci_device_id *ent)
#else
/*
 * __devinit macro was removed from the 3.10 kernel (RedHat: 3.8 kernel)
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

        /* Ensure have valid device */
        switch (ent->device) {
                case ICP_DH89xxCC_PCI_IOV_DEVICE_ID:
                case ICP_C2XXX_PCI_IOV_DEVICE_ID:
                        break;
                default:
                        ADF_ERROR("invalid device 0x%x found.\n",
                                      ent->device);
                        return -ENODEV;
        }

        if(ADF_MAX_DEVICES == num_device)
        {
                ADF_ERROR("Only support upto %d devices\n", ADF_MAX_DEVICES);
                return -EIO;
        }

        /* Allocate memory for the accel device. */
        accel_dev = ICP_ZALLOC_GEN(sizeof(icp_accel_dev_t));
        if (NULL == accel_dev) {
                ADF_ERROR("failed to allocate accel structure\n");
                return -ENOMEM;
        }
        /* set accel id */
        accel_dev->accelId = num_device;

        /* create device configuration table */
        if (SUCCESS != adf_cfgDeviceAdd(num_device)) {
                adf_cleanup_accel(accel_dev);
                return -ENOMEM;
        }

        /* Get device node */
        accel_dev->pkg_id = 0;

        /* Allocate memory for device configuration structure */
        hw_data = ICP_MALLOC_GEN_NUMA(sizeof (adf_hw_device_data_t),
                accel_dev->pkg_id);
        if (NULL == hw_data){
                ADF_ERROR("Failed to allocate memory for device config\n");
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        ICP_MEMSET(hw_data, '\0', sizeof (adf_hw_device_data_t));
        accel_dev->pHwDeviceData = hw_data;
        switch (ent->device) {
          case ICP_DH89xxCC_PCI_IOV_DEVICE_ID:
                adf_set_hw_data_dh89xxcciov(accel_dev->pHwDeviceData);
                break;
          case ICP_C2XXX_PCI_IOV_DEVICE_ID:
            adf_c2xxx_set_hw_data_iov(accel_dev->pHwDeviceData);
            break;
          default:
                ADF_ERROR("Unsupported device id\n");
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        /* Initialize device config */
        if (NOT_NULL_FUNCTION(hw_data->init)) {
                status = hw_data->init(hw_data, accel_dev->pkg_id);
                if (CPA_STATUS_FAIL == status){
                        ADF_ERROR("Failed to initialize hw_data init\n");
                        adf_cleanup_accel(accel_dev);
                        return -EIO;
                }
        }

        pci_info = &accel_dev->pciAccelDev;
        /* Retrieve the revision id and add it to the PCI info structure */
        pci_read_config_byte(pdev, ICP_PRID_OFFSET, &rev_id);
        pci_info->revisionId = rev_id;

        /* set the device fuses */
        pci_info->devFuses = 0;
        /* Get Accelerators and Accelerators Engines masks */
        accel_dev->accelMask =
                            hw_data->getAcceleratorsMask(pci_info->devFuses);
        accel_dev->aeMask = hw_data->getAccelEnginesMask(pci_info->devFuses);

        /* Read the fusectl register */
        if (NOT_NULL_FUNCTION(hw_data->getDevSKU)) {
                accel_dev->sku = hw_data->getDevSKU(0, 0, 0, 0);
        }

        /* Setup PCI domain information */
        pci_info->pciDomain.domain = pci_domain_nr(pdev->bus);
        pci_info->pciDomain.bus = pdev->bus->number;
        pci_info->pciDomain.slot = PCI_SLOT(pdev->devfn);
        pci_info->pciDomain.function = PCI_FUNC(pdev->devfn);
        num_device++;

        /* enable PCI device */
        if (pci_enable_device(pdev)) {
                adf_cleanup_accel(accel_dev);
                return -EIO;
        }

        /* set dma identifier */
        if (pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
                if ((pci_set_dma_mask(pdev, DMA_BIT_MASK(32)))) {

                        ADF_ERROR("No usable DMA configuration,"
                                      "aborting\n");
                        adf_cleanup_accel(accel_dev);
                        return -EIO;
                }
        }

        if ((status = pci_request_regions(pdev, adf_driver_name))) {
                adf_cleanup_accel(accel_dev);
                return status;
        }

        pci_set_master(pdev);
        /* Get accelerator functions */
        accel_dev->accelCapabilitiesMask = hw_data->getAccFuncMask(0);

        /* Setup PCI Info structure for upper layers */
        pci_info->pDev = pdev;
        pci_info->deviceId = ent->device;
        pci_info->irq = pdev->irq;

        for (i = 0; i < GET_MAX_BARS(accel_dev); i++) {
                bar_index = i * ADF_BAR_OFFSET;

                /* check if bar available and reset pciBars entry if not    */
                if (CPA_FALSE == hw_data->isBarAvailable(hw_data, i)){
                        pci_info->pciBars[i].baseAddr = 0;
                        pci_info->pciBars[i].virtAddr = 0;
                        pci_info->pciBars[i].size = 0;
                        pci_info->numBars++;
                        continue;
                }

                /* Find all the device's BARS */
                pci_info->pciBars[i].baseAddr =
                                        pci_resource_start(pdev, bar_index);
                if (!pci_info->pciBars[i].baseAddr) {
                        break;
                }

                pci_info->pciBars[i].size = pci_resource_len(pdev, bar_index);
                pci_info->numBars++;
                ADF_DEBUG("Found BAR %d size 0x%X\n", i,
                          pci_info->pciBars[i].size);

                /* Map the IO Memory */
                pci_info->pciBars[i].virtAddr = (UARCH_INT) ioremap_nocache(
                                                pci_info->pciBars[i].baseAddr,
                                                pci_info->pciBars[i].size);
                if (!pci_info->pciBars[i].virtAddr ) {
                        ADF_ERROR("Failed to map BAR %d region\n",i);
                        status = FAIL;
                }
        }

        if ( SUCCESS != status ) {
                for (i = 0; i < GET_MAX_BARS(accel_dev); i++) {
                        if (pci_info->pciBars[i].virtAddr) {
                                ADF_DEBUG("Freeing BAR %d region 0x%llX\n",i,
                           (Cpa64U)(UARCH_INT)pci_info->pciBars[i].virtAddr);
                                iounmap((void __iomem *) (UARCH_INT)
                                 pci_info->pciBars[i].virtAddr);
                                pci_info->pciBars[i].virtAddr =
                                 (UARCH_INT) NULL;
                        }
                }
                adf_cleanup_accel(accel_dev);
                return status;
        }

        /* Add fake esram address */
        status = adf_esramSetConfigInfo(accel_dev, 0, 0, 0);
        if (status) {
                ADF_ERROR("Failed to set eSRAM info\n");
                adf_cleanup_accel(accel_dev);
                return -EIO;
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
        /* Set usage counter to zero */
        osalAtomicSet(0, &accel_dev->usageCounter);

        /* This is virtualized device */
        accel_dev->virtualization.virtualized = 1;
        pci_info->state = ICP_ACCEL_STATE_INITIALIZED;
        return status;
}

/*
 * adf_remove
 */
static void adf_remove(struct pci_dev *pdev)
{
        int status = 0, i = 0;
        icp_accel_dev_t *accel_dev = NULL;
        icp_accel_pci_info_t *pci_info = NULL;

        if(NULL == pdev) {
                ADF_ERROR("%s(): pdev is NULL\n", __FUNCTION__);
                return;
        }
        /* Lookup the accel_dev assoc with this pdev */
        accel_dev = adf_devmgrGetAccelDev(pdev);
        if (NULL == accel_dev) {
                ADF_ERROR("failed to find accel_dev\n");
                return;
        }
        adf_config_ctl_stop_devices(accel_dev->accelId);

        if (pdev == accel_dev->pciAccelDev.pDev) {
                accel_dev->pciAccelDev.state = ICP_ACCEL_STATE_UNINITIALIZED;
        }

        pci_info = &accel_dev->pciAccelDev;
        pci_info->irq = 0;

        /* unmap the pcidev bars */
        for (i = 0; i < GET_MAX_BARS(accel_dev); i++) {
                if (pci_info->pciBars[i].virtAddr) {
                        iounmap( (void __iomem *)
                                 (UARCH_INT) pci_info->pciBars[i].virtAddr );
                        pci_info->pciBars[i].virtAddr =
                                (UARCH_INT) NULL;
                }
        }

        status = adf_cleanup_accel(accel_dev);
        if (status) {
                ADF_ERROR("adf_cleanup_accel failure\n");
        }

        /* Cleanup the ETR Data */
        adf_cleanup_ETR_Data(accel_dev);
}

/*
 * adfdrv_init
 * ICP ADF Module Init function
 */
#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))) \
     || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)))
static int __devinit adfdrv_init()
#else
/*
 * __devinit macro was removed from the 3.10 kernel (RedHat: 3.8 kernel)
 * as the functionality was no longer relevent
 */
static int adfdrv_init()
#endif
{
        int status = 0;

        /* Allocate Resource Config table */
        status = adf_cfgInit();

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
                return status;
        }

        status = pci_register_driver(&adf_driver);
        if (status) {
                ADF_ERROR("failed call to pci_register_driver\n");
                unregister_adf_ctl_device_driver();
                unregister_adf_processes_device_driver();
                adf_csr_chrdev_unregister();
                adf_ring_chrdev_unregister();
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
        adf_ring_chrdev_unregister();
        adf_csr_chrdev_unregister();
        unregister_adf_ctl_device_driver();
        unregister_adf_processes_device_driver();
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

/* Error handling function stub for VF */
int adf_aer_schedule_reset_dev(icp_accel_dev_t *accel_dev, int sync)
{
    return 0;
}

