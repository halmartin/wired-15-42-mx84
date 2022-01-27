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
#include "icp_adf_transport.h"
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

void adf_pf2vf_bh_handler(void* handle);
/*
 * adf_enable_msi
 * Function enables MSI capability
 */
STATIC int adf_enable_msi(icp_accel_pci_info_t *pci_dev_info)
{
        int stat = SUCCESS;
        ADF_DEBUG("Enabling MSI capability\n");
        stat = pci_enable_msi(pci_dev_info->pDev);
        if( SUCCESS != stat ){
                ADF_ERROR("Unable to enable MSI\n");
        }
        else {
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
        ADF_DEBUG("Disabling MSI capability\n");
        pci_disable_msi(pci_dev_info->pDev);
        pci_dev_info->irq = pci_dev_info->pDev->irq;
}

/*
 * adf_setup_pf2vf_bh
 * Setup & Initialise the tasklet structs for pf2vf bh of this device
 */
STATIC int adf_setup_pf2vf_bh(icp_accel_dev_t *accel_dev)
{
        if(NULL == accel_dev) {
                ADF_ERROR("adf_setup_pf2vf_bh invalid param accel_dev\n");
                return FAIL;
        }
        accel_dev->pf2vf_tasklet = ICP_ZALLOC_GEN(
                                    sizeof(struct tasklet_struct));
        if (!accel_dev->pf2vf_tasklet) {
                ADF_ERROR("Failed to allocate PF2VF tasklet\n");
                return FAIL;
        }
        tasklet_init(accel_dev->pf2vf_tasklet,
                     (void*)adf_pf2vf_bh_handler,
                     (unsigned long)accel_dev);

        return SUCCESS;
}

/*
 * adf_cleanup_pf2vf_bh
 * Cleanup Tasklet structures
 */
STATIC int adf_cleanup_pf2vf_bh(icp_accel_dev_t *accel_dev)
{
        if(NULL == accel_dev) {
                ADF_ERROR("adf_cleanup_pf2vf_bh invalid param accel_dev\n");
                return FAIL;
        }
        /*Cleanup the PF2VF tasklet */
        if (accel_dev->pf2vf_tasklet != NULL) {
                ADF_DEBUG("Cleanup bh tasklet for pf2vf \n");
                tasklet_disable(accel_dev->pf2vf_tasklet);
                tasklet_kill(accel_dev->pf2vf_tasklet);
                ICP_FREE(accel_dev->pf2vf_tasklet);
                accel_dev->pf2vf_tasklet = NULL;
        }
        return SUCCESS;
}

/*
 * adf_isr
 * Top Half ISR for MSI and legacy INTX mode
 *  Handles PF doorbell and ETR Bundle IRQs
 */
STATIC irqreturn_t adf_isr(int irq, void *privdata)
{
        icp_accel_dev_t *accel_dev = NULL;
        icp_etr_priv_data_t* priv_ring_data = NULL;
        unsigned long* csr_base_addr = 0;
        icp_et_ring_bank_data_t *bank_data = NULL;
        uint32_t vf2pf_bar_id = 0;
        adf_hw_device_data_t *hw_data = NULL;
        uint32_t vint_src_offset = 0;
        uint32_t *reg = NULL;
        uint32_t v_int = 0;
        uint32_t irq_source = 0;

        accel_dev = (icp_accel_dev_t *) privdata;

        hw_data = accel_dev->pHwDeviceData;
        vf2pf_bar_id = hw_data->getMiscBarId(hw_data);
        vint_src_offset = hw_data->getVIntSrcOffset(0);
        reg = (void*)
            accel_dev->pciAccelDev.pciBars[vf2pf_bar_id].virtAddr;
        /* Read VF INT source CSR to determine the source of VF interrupt */
        v_int = ICP_ADF_CSR_RD(reg, vint_src_offset);
        /* Check PF2VF interrupt */
        irq_source = hw_data->irqGetPF2VFSource(hw_data, v_int);
        if (0 != irq_source)
        {
            tasklet_hi_schedule(accel_dev->pf2vf_tasklet);
            return IRQ_HANDLED;
        }
        /* Check bundle interrupt */
        irq_source = hw_data->irqGetBunSource(hw_data, v_int);
        if (0 != irq_source)
        {
            priv_ring_data = (icp_etr_priv_data_t*) accel_dev->pCommsHandle;
            csr_base_addr = (unsigned long*)(UARCH_INT)
                                     priv_ring_data->csrBaseAddress;
            /*
             * Perform top-half processing.
             */
            bank_data = &(priv_ring_data->banks[0]);
            if(!bank_data->timedCoalescEnabled ||
                        bank_data->numberMsgCoalescEnabled) {
                    WRITE_CSR_INT_EN(0, 0);
            }
            if(bank_data->timedCoalescEnabled) {
                    WRITE_CSR_INT_COL_CTL_CLR(0);
            }
            tasklet_hi_schedule(bank_data->ETR_bank_tasklet);
            return IRQ_HANDLED;
        }
        return IRQ_NONE;
}

/*
 * adf_isr_resource_free
 * Free up the required resources
 */
int adf_isr_resource_free(icp_accel_dev_t *accel_dev)
{
        int status = SUCCESS;
        free_irq(accel_dev->pciAccelDev.irq, (void*) accel_dev);
        adf_disable_msi(&accel_dev->pciAccelDev);
        /* Clean bottom half handlers */
        adf_cleanup_bh(accel_dev);
        adf_cleanup_pf2vf_bh(accel_dev);
        return status;
}

/*
 * adf_isr_resource_alloc
 * Allocate the required ISR services.
 */
int adf_isr_resource_alloc(icp_accel_dev_t *accel_dev)
{
        int status = SUCCESS, flags = 0;

        /* setup bottom half handlers */
        adf_setup_bh(accel_dev);
        adf_setup_pf2vf_bh(accel_dev);

        /* Try to enable MSIX or MSI mode */
        status = adf_enable_msi(&accel_dev->pciAccelDev);
        if( SUCCESS != status ) {
                ADF_ERROR("failed request_irq %d, status=%d\n",
                           accel_dev->pciAccelDev.irq, status);
        }
        if (accel_dev->pciAccelDev.irq) {
                status = request_irq(accel_dev->pciAccelDev.irq,
                                     adf_isr,
                                     flags, adf_driver_name,
                                     (void*)accel_dev);
                if( SUCCESS != status ) {
                        ADF_ERROR("failed request_irq %d, status=%d\n",
                                   accel_dev->pciAccelDev.irq, status);
                        adf_isr_resource_free(accel_dev);
                }
        }
        return status;
}
