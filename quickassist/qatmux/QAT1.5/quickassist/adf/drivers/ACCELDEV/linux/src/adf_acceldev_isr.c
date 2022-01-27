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
 * @file adf_acceldev_isr.c
 *
 * @description
 *    This file contains ADF code for ISR services to upper layers
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include "cpa.h"
#include "icp_adf_init.h"
#include "icp_accel_devices.h"
#include "adf_drv.h"
#include "icp_platform.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_isr.h"
#include "adf_bh.h"
#include "icp_firml_interface.h"
#include "adf_ae.h"
#include "adf_cfg.h"
#include "adf_drv.h"

/*
 * Define reset Mode
 */
#define RESET_ASYNC 0

/*
 * adf_isr_get_value
 * Function to retrieve the value of a configuration parameter key
 */
STATIC int adf_isr_get_value(icp_accel_dev_t *accel_dev,
                                char *section, char *key, int *value)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};

    status = icp_adf_cfgGetParamValue(accel_dev, section, key, val);
    if (CPA_STATUS_SUCCESS != status) {
            return FAIL;
    }
    *value = ICP_STRTOUL(val, NULL, ADF_CFG_BASE_DEC);
    return SUCCESS;
}

/*
 * adf_isr_get_intr_mode
 * Function to retrieve interrupt mode from the configuration file
 *
 * If no interrupt mode is set in the configuration, MSI-X will be
 * used as a default interrupt mode.
 */
STATIC void adf_isr_get_intr_mode(icp_accel_dev_t *accel_dev,
                                    adf_isr_mode_t *mode)
{
    int status = SUCCESS;
    int isr_value = 0;

    /* MSIX has the highest priority */
    status = adf_isr_get_value(accel_dev,
                    GENERAL_SEC, ADF_ISR_MSIX_KEY, &isr_value);
    if ((SUCCESS == status) && (isr_value == 1)) {
            ADF_DEBUG("%s is set\n", ADF_ISR_MSIX_KEY);
            *mode = adf_isr_mode_msix;
            return;
    }
    status = adf_isr_get_value(accel_dev,
                    GENERAL_SEC, ADF_ISR_MSI_KEY, &isr_value);
    if ((SUCCESS == status) && (isr_value == 1)) {
            ADF_DEBUG("%s is set\n", ADF_ISR_MSI_KEY);
            *mode = adf_isr_mode_msi;
            return;
    }
    status = adf_isr_get_value(accel_dev,
                    GENERAL_SEC, ADF_ISR_INTX_KEY, &isr_value);
    if ((SUCCESS == status) && (isr_value == 1)) {
            ADF_DEBUG("%s is set\n", ADF_ISR_INTX_KEY);
            *mode = adf_isr_mode_intx;
            return;
    }
    /* In case no mode is set */
    ADF_DEBUG("Interrupt Mode undefined, defaulting to MSIX.\n");
    *mode = adf_isr_mode_msix;
}


/*
 * adf_enable_msix
 * Function enables MSIX capability
 */
STATIC int adf_enable_msix(icp_accel_dev_t *accel_dev)
{
        adf_hw_device_data_t *hw_data = NULL;
        icp_accel_pci_info_t *pci_dev_info = NULL;
        int stat = SUCCESS;
        int index = 0;
        int vector = 0;
        Cpa32U msix_num_entries = 0;
        Cpa32U lastVector = 0;

        if (NULL == accel_dev) {
                ADF_ERROR("invalid param accel_dev\n");
                return FAIL;
        }

        if (NULL == accel_dev->pHwDeviceData) {
                ADF_ERROR("pHwDeviceData is null\n");
                return FAIL;
        }
        hw_data = accel_dev->pHwDeviceData;
        pci_dev_info = &accel_dev->pciAccelDev;
        if (adf_isr_mode_msix != pci_dev_info->isr_mode) {
            return FAIL;
        }

        msix_num_entries = hw_data->msix.banksVectorNum +
            hw_data->msix.aeVectorNum;
        /* create bundle entries */
        lastVector =
            hw_data->msix.banksVectorStart + hw_data->msix.banksVectorNum;

        for (vector = hw_data->msix.banksVectorStart, index = 0;
             vector < lastVector; ++vector, ++index) {
                pci_dev_info->msixEntries.value[index].entry = index;
        }

        /* create AE Cluster entry */
        lastVector =
            hw_data->msix.aeVectorStart + hw_data->msix.aeVectorNum;

        for (vector = hw_data->msix.aeVectorStart;
             vector < lastVector; ++vector, ++index) {
            /*
             * If some vectors are skipped between the bundle entries and
             * the AE entries then we need to skip these vectors between
             * banksVectorNum and aeVectorStart.
             */
            pci_dev_info->msixEntries.value[index].entry = index +
               (hw_data->msix.aeVectorStart - hw_data->msix.banksVectorNum);
        }

        stat = pci_enable_msix(pci_dev_info->pDev,
                               pci_dev_info->msixEntries.value,
                               msix_num_entries);
        if (SUCCESS != stat){
                ADF_ERROR("Unable to enable MSIX\n");
                pci_dev_info->isr_mode = adf_isr_mode_msi;
        }
        else {
                ADF_DEBUG("MSIX capability enabled\n");
                pci_dev_info->isr_mode = adf_isr_mode_msix;
        }
        return stat;
}

/*
 * adf_disable_msix
 * Function disables MSIX capability
 */
STATIC void adf_disable_msix(icp_accel_pci_info_t *pci_dev_info)
{
        pci_dev_info->isr_mode = adf_isr_mode_intx;
        ADF_DEBUG("Disabling MSIX capability\n");
        pci_disable_msix(pci_dev_info->pDev);
}

/*
 * adf_enable_msi
 * Function enables MSI capability
 */
STATIC int adf_enable_msi(icp_accel_pci_info_t *pci_dev_info)
{
        int stat = SUCCESS;

        if (adf_isr_mode_msi != pci_dev_info->isr_mode) {
            return FAIL;
        }

        stat = pci_enable_msi(pci_dev_info->pDev);
        if (SUCCESS != stat){
                ADF_ERROR("Unable to enable MSI\n");
                pci_dev_info->isr_mode = adf_isr_mode_intx;
        }
        else {
                ADF_DEBUG("MSI capability enabled\n");
                pci_dev_info->isr_mode = adf_isr_mode_msi;
                pci_dev_info->irq = pci_dev_info->pDev->irq;
        }
        return stat;
}

/*
 * adf_disable_msi
 * Function disables MSI capability
 */
STATIC void adf_disable_msi(icp_accel_pci_info_t *pci_dev_info)
{
        pci_dev_info->isr_mode = adf_isr_mode_intx;
        ADF_DEBUG("Disabling MSI capability\n");
        pci_disable_msi(pci_dev_info->pDev);
        pci_dev_info->irq = pci_dev_info->pDev->irq;
}

/*
 * adf_msix_isr_bundle
 * Top Half ISR for MSIX
 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,18))
STATIC irqreturn_t adf_msix_isr_bundle(int irq, void *privdata,
                          struct pt_regs *regs)
#else
STATIC irqreturn_t adf_msix_isr_bundle(int irq, void *privdata)
#endif
{
        icp_accel_dev_t *accel_dev = NULL;
        icp_etr_priv_data_t* priv_ring_data = NULL;
        int status = 0, i = 0;
        unsigned long* csr_base_addr = 0;
        icp_et_ring_bank_data_t *bank_data = NULL;

        /* Read the ring status CSR to determine which rings have data */
        accel_dev = (icp_accel_dev_t *) privdata;
        priv_ring_data = (icp_etr_priv_data_t*) accel_dev->pCommsHandle;
        csr_base_addr = (unsigned long*)(UARCH_INT)
                                 priv_ring_data->csrBaseAddress;

        for (i = 0; i < accel_dev->pHwDeviceData->msix.banksVectorNum; i++) {
                if (accel_dev->pciAccelDev.msixEntries.value[i].vector == irq) {
                        bank_data = &(priv_ring_data->banks[i]);
                        if (!bank_data->timedCoalescEnabled ||
                                           bank_data->numberMsgCoalescEnabled) {
                                WRITE_CSR_INT_EN(i, 0);
                        }
                        if (bank_data->timedCoalescEnabled) {
                                WRITE_CSR_INT_COL_CTL_CLR(i);
                        }
                        tasklet_hi_schedule(bank_data->ETR_bank_tasklet);
                        status = 1;
                }
        }
        if (status == TRUE) {
                return IRQ_HANDLED;
        }
        return IRQ_NONE;
}

/*
 *  adf_msix_isr_ae
 *  Top Half ISR
 *  Handles VF doorbell and AE Cluster IRQs
 *  Calls the Firmware Loader TopHalf handler.
 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,18))
STATIC irqreturn_t adf_msix_isr_ae(int irq, void *privdata,
                    struct pt_regs *regs)
#else
STATIC irqreturn_t adf_msix_isr_ae(int irq, void *privdata)
#endif
{
        icp_accel_dev_t *accel_dev = NULL;
        int status = ICP_FIRMLOADER_FAIL, i = 0;
        adf_hw_device_data_t  *hw_data = NULL;
        CpaBoolean isResetNeeded = CPA_FALSE;
        accel_dev = (icp_accel_dev_t *) privdata;
        hw_data = accel_dev->pHwDeviceData;

        /*
         * If virtualization is enabled then need to check the
         * source of the IRQ and in case it is VF2PF handle it
         */
        if (accel_dev->virtualization.enabled) {
                Cpa32U mask = 0, reg = 0;
                Cpa32U *pmisc_bar_addr = NULL;
                Cpa32U pmisc_bar_id = 0;
                Cpa32U vf2pfDefMask = 0;
                Cpa32U vf2pfBitOffset = 0;

                pmisc_bar_id = hw_data->getMiscBarId(hw_data);

                pmisc_bar_addr = (void*)
                    accel_dev->pciAccelDev.pciBars[pmisc_bar_id].virtAddr;

                reg = ICP_ADF_CSR_RD(pmisc_bar_addr,
                        hw_data->getVf2PfIntSourceOffset());

                /* write to PF2VF register to clean IRQ */
                vf2pfBitOffset = hw_data->getVf2PfBitOffset();
                vf2pfDefMask = hw_data->getVf2PfIntMaskDefault();
                ICP_ADF_CSR_WR(pmisc_bar_addr,
                        hw_data->getVf2PfIntMaskOffset(), vf2pfDefMask);
                mask = reg & vf2pfDefMask;
                mask >>= vf2pfBitOffset;

                if (mask) {
                        icp_accel_pci_vf_info_t *vf;
                        /* Handle IRQ from VF */
                        for (i = 0; i < hw_data->maxNumVf; i++) {
                                if (mask & (1 << i)) {
                                        vf = accel_dev->pci_vfs[i];
                                        tasklet_hi_schedule(vf->tasklet_vf);
                                        status = SUCCESS;
                                }
                        }
                        if (SUCCESS == status) {
                                return IRQ_HANDLED;
                        }
                }
        }
        /* If it wasn't VF2PF then it is AE.*/
        if(NULL != hw_data->handleUncoErrInterrupts)
        {
            isResetNeeded = hw_data->handleUncoErrInterrupts(accel_dev);
        }
        if(CPA_TRUE == isResetNeeded)
        {
                /* Reset the device-async mode */
                ADF_DEBUG("Reset scheduled\n");
                adf_aer_schedule_reset_dev(accel_dev, RESET_ASYNC);
                return IRQ_HANDLED;
        }

        return IRQ_NONE;
}

/*
 * adf_isr
 * Top Half ISR for MSI and legacy INTX mode
 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,18))
STATIC irqreturn_t adf_isr(int irq, void *privdata, struct pt_regs *regs)
#else
STATIC irqreturn_t adf_isr(int irq, void *privdata)
#endif
{
        icp_accel_dev_t *accel_dev = NULL;
        icp_etr_priv_data_t* priv_ring_data = NULL;
        uint32_t irq_source = 0;
        int status = 0, i = 0;
        unsigned long* csr_base_addr = 0;
        uint32_t s_int = 0;
        uint32_t *pmisc_addr = NULL;
        icp_et_ring_bank_data_t *bank_data = NULL;
        adf_hw_device_data_t   *hw_data = NULL;

        accel_dev = (icp_accel_dev_t *) privdata;
        hw_data = accel_dev->pHwDeviceData;
        priv_ring_data = (icp_etr_priv_data_t*) accel_dev->pCommsHandle;
        csr_base_addr = (unsigned long*)(UARCH_INT)
                                 priv_ring_data->csrBaseAddress;

        /*
         * Perform top-half processing.
         * Check SINT register
         */
        /* If one of our response rings has data to be processed */
        pmisc_addr = (void*)accel_dev->pciAccelDev.
                            pciBars[hw_data->getMiscBarId(hw_data)].virtAddr;
        /* Read the ring status CSR to determine which rings have data */
        s_int = ICP_ADF_CSR_RD(pmisc_addr, hw_data->sintpfOffset);

        /* Check bundle interrupt */
        irq_source = hw_data->irqGetBankNumber(hw_data, s_int);
        if (0 != irq_source)
        {
                for (i = 0; i < hw_data->maxNumBanks; i++) {
                        if (irq_source & (1 << i)) {
                                bank_data = &(priv_ring_data->banks[i]);

                                if (!bank_data->timedCoalescEnabled ||
                                           bank_data->numberMsgCoalescEnabled) {
                                        WRITE_CSR_INT_EN(i, 0);
                                }
                                if (bank_data->timedCoalescEnabled) {
                                        WRITE_CSR_INT_COL_CTL_CLR(i);
                                }
                                tasklet_hi_schedule(
                                        bank_data->ETR_bank_tasklet);
                                status = TRUE;
                        }
                }
        }
        /* Check miscellaneous interrupt */
        irq_source = hw_data->irqGetAeSource(hw_data, s_int);
        if (irq_source) {
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,18))
            return adf_msix_isr_ae(irq, privdata, NULL);
#else
            return adf_msix_isr_ae(irq, privdata);
#endif
        }

        if (status == TRUE) {
                return IRQ_HANDLED;
        }

        return IRQ_NONE;
}

/*
 * adf_isr_free_msix_entry_table
 * Free buffer for MSIX entry value and name
 */
STATIC void adf_isr_free_msix_entry_table(icp_accel_dev_t *accel_dev)
{
        if (NULL == accel_dev) {
                ADF_ERROR("invalid param accel_dev\n");
                return;
        }
        /* release memory allocates for msixEntries */
        if (NULL != accel_dev->pciAccelDev.msixEntries.value) {
            ICP_FREE_NUMA(accel_dev->pciAccelDev.msixEntries.value);
        }
        if (NULL != accel_dev->pciAccelDev.msixEntries.name) {
            ICP_FREE_NUMA(accel_dev->pciAccelDev.msixEntries.name);
        }
}

/*
 * adf_isr_resource_free
 * Free up the required resources
 */
int adf_isr_resource_free(icp_accel_dev_t *accel_dev)
{
        int status = SUCCESS, i = 0;
        adf_hw_device_data_t *hw_data = NULL;
        Cpa32U msixEntries = 0;
        icp_accel_pci_vf_info_t *vf = NULL;

        hw_data = accel_dev->pHwDeviceData;
        msixEntries = hw_data->msix.banksVectorNum +
            hw_data->msix.aeVectorNum;

        if (accel_dev->pciAccelDev.isr_mode == adf_isr_mode_msix) {
                struct msix_entry *msixe =
                           accel_dev->pciAccelDev.msixEntries.value;

                for(i = 0; i < msixEntries - hw_data->msix.aeVectorNum; i++) {
                        vf = (icp_accel_pci_vf_info_t *)accel_dev->pci_vfs[i];
                        if((NULL == vf) || ((NULL != vf) && (1 == vf->attached)))
                        {
                            free_irq(msixe[i].vector, (void*) accel_dev);
                        }
                }
                for(; i < msixEntries; i++) {
                        free_irq(msixe[i].vector, (void*) accel_dev);
                }

                ssleep(1);
                adf_disable_msix(&accel_dev->pciAccelDev);
        }
        else {
                free_irq(accel_dev->pciAccelDev.irq, (void*) accel_dev);
                if (accel_dev->pciAccelDev.isr_mode == adf_isr_mode_msi) {
                        adf_disable_msi(&accel_dev->pciAccelDev);
                }
        }

        /* Clean bottom half handlers */
        adf_cleanup_bh(accel_dev);
        /* Free MSIX entry table */
        adf_isr_free_msix_entry_table(accel_dev);
        return status;
}

/*
 * adf_request_msix_irq
 * Request IRQ resources for all the MSIX_ENTRIES
 */
STATIC int adf_request_msix_irq(icp_accel_dev_t *accel_dev)
{
        int status = SUCCESS, i = 0;
        char *name = NULL;
        Cpa32U msixBundleEntries = 0;
        adf_hw_device_data_t *hw_data = NULL;
        icp_accel_pci_info_t *pci_dev_info = &accel_dev->pciAccelDev;
        struct msix_entry *msixe = pci_dev_info->msixEntries.value;
        icp_accel_pci_vf_info_t *vf = NULL;

        hw_data = accel_dev->pHwDeviceData;
        msixBundleEntries = hw_data->msix.banksVectorNum;


        /* Request msix irq for bundles */
        for(i = 0; i < msixBundleEntries; i++)
        {
                vf = (icp_accel_pci_vf_info_t *)accel_dev->pci_vfs[i];
                if((NULL == vf) || ((NULL != vf) && (1 == vf->attached)))
                {
                    name = pci_dev_info->msixEntries.name[i];

                    snprintf(name, MAX_MSIX_DEVICE_NAME, "dev%dBundle%d",
                            accel_dev->accelId, i);
                    status = request_irq(msixe[i].vector,
                            adf_msix_isr_bundle, 0, name,
                            (void *)accel_dev);

                    if (status) {
                        ADF_ERROR(
                                "failed request_irq IRQ %d for %s status=%d\n",
                                msixe[i].vector, name, status);
                        return status;
                    }
                }
        }

        /* Request msix irq for AE Cluster */
        name = pci_dev_info->msixEntries.name[i];
        snprintf(name, MAX_MSIX_DEVICE_NAME, "dev%dAECluster",
                accel_dev->accelId);
        status = request_irq(msixe[i].vector,
                adf_msix_isr_ae, IRQF_SHARED, name,
                (void *)accel_dev);

        if (status) {
            ADF_ERROR(
                    "failed IRQ %d, for %s status=%d\n",
                    msixe[i].vector, name, status);
            return status;
        }
        return status;
}

/*
 * adf_isr_alloc_msix_entry_table
 * Allocate buffer for MSIX entry value and name
 */
int adf_isr_alloc_msix_entry_table(icp_accel_dev_t *accel_dev)
{
        adf_hw_device_data_t *hw_data = NULL;
        Cpa32U msix_num_entries = 0;
        struct msix_entry *entries = NULL;
        void *name = NULL;

        if (NULL == accel_dev) {
                ADF_ERROR("invalid param accel_dev\n");
                return FAIL;
        }

        if (NULL == accel_dev->pHwDeviceData) {
                ADF_ERROR("accel_dev->pHwDeviceData is null\n");
                return FAIL;
        }

        hw_data = accel_dev->pHwDeviceData;
        /* Allocate memory for msix entries in pci info structure */
        msix_num_entries = hw_data->msix.banksVectorNum +
            hw_data->msix.aeVectorNum;

        entries = ICP_MALLOC_GEN_NUMA(
                msix_num_entries * sizeof (struct msix_entry),
                accel_dev->pkg_id);
        if (NULL == entries) {
            ADF_ERROR("Failed to allocate memory\n");
            return FAIL;
        }
        ICP_MEMSET(entries, '\0',
                msix_num_entries * sizeof (struct msix_entry));

        accel_dev->pciAccelDev.msixEntries.value = entries;

        name = ICP_MALLOC_GEN_NUMA(
                msix_num_entries * sizeof (Cpa8U) * MAX_MSIX_DEVICE_NAME,
                accel_dev->pkg_id);
        ICP_MEMSET(name, '\0',
                msix_num_entries * sizeof (Cpa8U) * MAX_MSIX_DEVICE_NAME);

        accel_dev->pciAccelDev.msixEntries.name= name;

        return SUCCESS;
}

/*
 * adf_isr_resource_alloc
 * Allocate the required ISR services.
 */
int adf_isr_resource_alloc(icp_accel_dev_t *accel_dev)
{
        int status = SUCCESS, flags = 0;

        status = adf_isr_alloc_msix_entry_table(accel_dev);
        if (status) {
            ADF_ERROR("adf_isr_alloc_msix_entry_table failed\n");
            return status;
        }
        /* setup bottom half handlers */
        adf_setup_bh(accel_dev);
        /* get the interrupt mode */
        adf_isr_get_intr_mode(accel_dev, &(accel_dev->pciAccelDev.isr_mode));
        /*
         * Setup the structures in accel_dev
         */
        if (accel_dev->pciAccelDev.irq) {
                /* Try to enable MSIX or MSI mode */
                status = adf_enable_msix(accel_dev);
                if (SUCCESS != status) {
                        status = adf_enable_msi(&accel_dev->pciAccelDev);
                }
                /* We only need to set the SHARED flag for INTX mode */
                flags =
                  (accel_dev->pciAccelDev.isr_mode == adf_isr_mode_intx)
                  ? IRQF_SHARED : 0;

                if (accel_dev->pciAccelDev.isr_mode == adf_isr_mode_msix) {
                        /* Managed to enable MSIX.
                         * Now request an IRQ for each entry */
                        status = adf_request_msix_irq(accel_dev);
                        if (status) {
                                goto err_failure;
                        }
                }
                else {
                    /* Need to handle IRQ in the old way */
                    status = request_irq(accel_dev->pciAccelDev.irq, adf_isr,
                                    flags, adf_driver_name, (void *)accel_dev);
                }
                if (status) {
                        ADF_ERROR("failed request_irq %d, status=%d\n",
                                  accel_dev->pciAccelDev.irq, status);
                        goto err_failure;
                }
        }
        return SUCCESS;

err_failure:
        adf_isr_resource_free(accel_dev);
        return status;
}

/*
 * adf_enable_bundle_msix
 * Enable single MSIX for a bundle while VF goes down.
 */
int adf_enable_bundle_msix(icp_accel_dev_t *accel_dev, int vf_number)
{
        int status = SUCCESS;
        icp_accel_pci_info_t *pci_dev_info = &accel_dev->pciAccelDev;
        struct msix_entry *msixe = &pci_dev_info->msixEntries.value[vf_number];
        char *name = pci_dev_info->msixEntries.name[vf_number];
        snprintf(name, MAX_MSIX_DEVICE_NAME, "dev%dBundle%d",
                                       accel_dev->accelId, vf_number);
        status = request_irq(msixe->vector,
                             adf_msix_isr_bundle, 0, name,
                             (void *)accel_dev);
        if (status) {
                ADF_ERROR("failed request_irq IRQ %d for %s status=%d\n",
                                            msixe->vector, name, status);
        }
        return status;
}

/*
 * adf_disable_bundle_msix
 * Disable single MSIX for a bundle that becomes VF.
 */
int adf_disable_bundle_msix(icp_accel_dev_t *accel_dev, int vf_number)
{
        int status = SUCCESS;
        icp_accel_pci_info_t *pci_dev_info = &accel_dev->pciAccelDev;
        struct msix_entry *msixe = &pci_dev_info->msixEntries.value[vf_number];
        free_irq(msixe->vector, (void*)accel_dev);
        return status;
}
