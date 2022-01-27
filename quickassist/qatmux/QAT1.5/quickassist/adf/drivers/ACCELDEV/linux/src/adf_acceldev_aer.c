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
 * @file adf_acceldev_aer.c
 *
 * @description
 *
 *      This file contains advanced error reporting handlers
 *****************************************************************************/
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#if ((!defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)))\
   || ((defined(RHEL_MAJOR)) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)))
#include <linux/aer.h>
#endif
#include <linux/completion.h>
#include <linux/workqueue.h>

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "adf_init.h"
#include "adf_devmgr.h"
#include "icp_platform.h"
#include "adf_drv.h"
#include "adf_platform.h"
#include "adf_drv_sriov.h"
#include "adf_cfg.h"
#include "adf_config_ctl.h"

static struct workqueue_struct *device_reset_wq = NULL;

/*
 * PCI error registers offsets
 */
#define PPAERUCS   0x104 /* AER Uncorrectable Error Status Register */
#define PPAERUCM   0x108 /* AER Uncorrectable Error Mask Register */
#define PPAERUCSEV 0x10C /* AER Uncorrectable Error Severity Register */
#define PPAERCS    0x110 /* AER Correctable Error Register */
#define PPAERCM    0x114 /* AER Correctable Error Mask Register */
#define PPAERMASK  0xFFFFFFFF /* AER enable all mask */

#ifdef CONFIG_PCIEAER
/*
 * Error detected handler
 */
static pci_ers_result_t adf_error_detected(struct pci_dev *pdev,
                                                pci_channel_state_t state)
{
        Cpa32U reg = 0;
        icp_accel_dev_t *accel_dev = adf_devmgrGetAccelDev(pdev);
        ADF_PRINT("Acceleration driver hardware error detected.\n");
        if(!accel_dev) {
                ADF_ERROR("Can't find acceleration device\n");
                return PCI_ERS_RESULT_DISCONNECT;
        }

        if (state == pci_channel_io_perm_failure) {
                ADF_ERROR("Can't recover from device error\n");
                return PCI_ERS_RESULT_DISCONNECT;
        }

        ADF_PRINT("Error registers dump:\n");
        pci_read_config_dword(pdev, PPAERUCS, &reg);
        ADF_PRINT("Uncorrectable Error Register %X\n", reg);

        pci_read_config_dword(pdev, PPAERCS, &reg);
        ADF_PRINT("Correctable Error Register %X\n", reg);
        ADF_PRINT("Going to reset device.\n");
        return PCI_ERS_RESULT_NEED_RESET;
}
#endif

/* reset dev data */
typedef struct adf_reset_dev_data_s {
        int sync;
        icp_accel_dev_t *accel_dev;
        struct completion compl;
        struct work_struct reset_work;
} adf_reset_dev_data_t;

/*
 * adf_device_reset_worker
 * Reset the given device to recover from an uncorrectable error.
 * Function runs in kernel thread context.
 */
static void adf_device_reset_worker(struct work_struct *work)
{
        adf_reset_dev_data_t *reset_data =
                  container_of(work, adf_reset_dev_data_t, reset_work);

        if (reset_data->accel_dev->virtualization.enabled) {
            if(adf_PF2VF_notify(reset_data->accel_dev)) {
                    ADF_ERROR("adf_PF2VF_notify failed\n");
            }
        }
        /* Ignore status from stop devices - assume it will fail */
        adf_subsystemRestarting(reset_data->accel_dev);
        adf_config_ctl_stop_devices(reset_data->accel_dev->accelId);
        /* Now perform the reset */
        if (adf_restore_dev(reset_data->accel_dev)) {
                /* The device hanged and we can't restart it. Try to
                 * reinitialize it anyway - miracles sometimes happen */
                ADF_ERROR("Restart device failed, trying to reinit anyway\n");
        }
        if (adf_init_devices(reset_data->accel_dev->accelId,
                             IOCTL_RESET_ACCEL_DEV)) {
                /* The device is hung and we can't restart it so stop here */
                ADF_ERROR("Restart device failed\n");
                ICP_FREE(reset_data);
                BUG();
                return;
        }
        adf_subsystemRestarted(reset_data->accel_dev);
        CLEAR_STATUS_BIT(reset_data->accel_dev->adfSubsystemStatus,
                                                ADF_STATUS_SYSTEM_RESTARTING);

        /* Everyting went good. We are back - if in sync mode then
         * notify the caller */
        if (reset_data->sync) {
                complete(&reset_data->compl);
        }
        else {
                ICP_FREE(reset_data);
        }
        return;
}

/* Max reset time is 15 seconds */
#define MAX_RESET_TIME 15000
/*
 * Schedule a thread that will perform device reset
 */
int adf_aer_schedule_reset_dev(icp_accel_dev_t *accel_dev, int sync)
{
        adf_reset_dev_data_t *reset_data = NULL;

        /* If ready for dev reset and reset not already in progress */
        if (BIT_IS_SET(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_SYSTEM_STARTED) &&
            (!BIT_IS_SET(accel_dev->adfSubsystemStatus,
                           ADF_STATUS_SYSTEM_RESTARTING))) {

                SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
                           ADF_STATUS_SYSTEM_RESTARTING);
                reset_data = ICP_MALLOC_ATOMIC(sizeof(adf_reset_dev_data_t));
                if (reset_data) {
                        memset(reset_data, '\0', sizeof(adf_reset_dev_data_t));
                        reset_data->accel_dev = accel_dev;
                        init_completion(&reset_data->compl);
                        reset_data->sync = sync;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
                        INIT_WORK(&reset_data->reset_work,
                                   adf_device_reset_worker);
#else
                        INIT_WORK(&reset_data->reset_work,
                                   adf_device_reset_worker,
                                   NULL);
#endif
                        queue_work(device_reset_wq, &reset_data->reset_work);
                } else {
                        ADF_ERROR("Couldn't allocate memory for reset job\n");
                        BUG();
                        return -ENOMEM;
                }
                /* If in sync mode wait for the result */
                if (sync) {
                        int ret = SUCCESS;
                        unsigned long wait_jiffies =
                                              msecs_to_jiffies(MAX_RESET_TIME);
                        unsigned long timeout =
                            wait_for_completion_timeout(&reset_data->compl,
                                                                 wait_jiffies);
                        if (!timeout) {
                                ADF_ERROR("Reset device timeout expired\n");
                                ret = FAIL;
                        }
                        ICP_FREE(reset_data);
                        return ret;
                }
        } else {
                   ADF_PRINT("Device is not ready for reset\n");  
                   return CPA_STATUS_FAIL;  
        } 
       
        return SUCCESS;
}

#ifdef CONFIG_PCIEAER
/*
 * Reset handler
 */
static pci_ers_result_t adf_slot_reset(struct pci_dev *pdev)
{
        icp_accel_dev_t *accel_dev = adf_devmgrGetAccelDev(pdev);
        if (!accel_dev) {
                ADF_ERROR("Can't find acceleration device\n");
                return PCI_ERS_RESULT_DISCONNECT;
        }
        pci_cleanup_aer_uncorrect_error_status(pdev);
        /* Reset device and wait for result */
        if (adf_aer_schedule_reset_dev(accel_dev, 1)) {

                ADF_ERROR("adf_subsystemInit error\n");
                return PCI_ERS_RESULT_DISCONNECT;
        }
        return PCI_ERS_RESULT_RECOVERED;
}

/*
 * Resume handler
 */
static void adf_resume(struct pci_dev *pdev)
{
        ADF_PRINT("Acceleration driver reset completed\n");
        ADF_PRINT("Device is up and runnig\n");
}

/*
 * AER handlers
 */
static struct pci_error_handlers adf_err_handler = {
              .error_detected = adf_error_detected,
              .slot_reset = adf_slot_reset,
              .resume = adf_resume,
};
#endif

/*
 * adf_enable_aer
 * Enable accel dev advanced error reporting
 */
int adf_enable_aer(icp_accel_dev_t *accel_dev, void *adf_driver)
{
#ifdef CONFIG_PCIEAER
        struct pci_dev *pdev = accel_dev->pciAccelDev.pDev;
        struct pci_driver *drv = NULL;
#endif

        /*
         * If AER is enabled enable all errors
         */
#ifdef CONFIG_PCIEAER
        pci_write_config_dword(pdev, PPAERUCM, PPAERMASK);
        pci_write_config_dword(pdev, PPAERCM, PPAERMASK);

        drv = (struct pci_driver *)adf_driver;
        drv->err_handler = &adf_err_handler;
        if (pci_enable_pcie_error_reporting(pdev)) {
                ADF_PRINT("Failed to enable PCI error reporting\n");
        }
#endif
        return SUCCESS;

}

/*
 * adf_disable_aer
 * Disable accel dev advanced error reporting
 */
void adf_disable_aer(icp_accel_dev_t *accel_dev)
{
        #ifdef CONFIG_PCIEAER
        struct pci_dev *pdev = accel_dev->pciAccelDev.pDev;
        if (pci_disable_pcie_error_reporting(pdev)) {
                ADF_ERROR("Can not disable AER for AccelDev\n");
        }
        #endif
}

/*
 * adf_init_aer
 */
int adf_init_aer(void)
{
        device_reset_wq = create_workqueue("device_reset_wq");
        return (device_reset_wq == NULL) ? FAIL : SUCCESS;
}

/*
 * adf_exit_aer
 */
void adf_exit_aer(void)
{
        if(device_reset_wq)
                destroy_workqueue(device_reset_wq);
        device_reset_wq = NULL;
}
