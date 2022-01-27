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
 * @file adf_common_drv.c
 *
 * @description
 *        This file contains the functions for initialising, starting,
 *        stopping and shutting down the accelerators and registered
 *        subsystems.
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_adf_accel_mgr.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_drv.h"
#include "adf_cfg.h"
#include "adf_init.h"
#include "adf_ctl_drv.h"
#include "adf_config_ctl.h"
#include "adf_devmgr.h"
#include "adf_user_proxy.h"
#include "adf_proc_debug.h"

/*
 * adf_get_module
 *
 * Increases the module usage counter.
 */
extern int adf_get_module(void);
/*
 * adf_put_module
 *
 * Decreases the module usage counter.
 */
extern int adf_put_module(void);

/*
 * adf_do_init
 *
 * Function to initialize the underlying acceleration subsystem for the
 * required accelerator.
 */
STATIC int adf_do_init(icp_accel_dev_t *accel_dev)
{
    int status = SUCCESS;

    if (accel_dev->pciAccelDev.state != ICP_ACCEL_STATE_INITIALIZED) {
        ADF_ERROR("accelerator not ready for init ..\n");
        return -EIO;
    }

    adf_get_module();
    status = adf_subsystemInit(accel_dev);
    if (status != CPA_STATUS_SUCCESS) {
        ADF_ERROR("adf_subsystemInit"
        " error, stopping and shutting down\n");
        status = adf_config_ctl_stop_devices(accel_dev->accelId);
        if (status != CPA_STATUS_SUCCESS) {
            ADF_ERROR("adf_config_ctl_stop_devices"
            " error, stop failed\n");
        }
        return -EIO;
    }
    return status;
}

/*
 * adf_init_all
 *
 * Wrapper function which initialises all the
 * subsystems.
 */
int adf_init_all(icp_accel_dev_t *accel_dev)
{
    int status = SUCCESS;
    /*
     * setup specific data structures for
     * the manager of the device
     */
    if (adf_debugInit(accel_dev) != CPA_STATUS_SUCCESS) {
        ADF_ERROR("Failed to initialise proc debug data.");
        return FAIL;
    }
    if (adf_init_ETR_Data(accel_dev) != CPA_STATUS_SUCCESS) {
        ADF_ERROR("Failed to initialise the ETR data.");
        adf_debugShutdown(accel_dev);
        return FAIL;
    }
    status = adf_user_proxyInit(accel_dev);
    if (SUCCESS != status) {
        ADF_ERROR("Failed to initialise user proxy for"
        "device %d\n", accel_dev->accelId);
        adf_debugShutdown(accel_dev);
        return FAIL;
    }
    status = adf_do_init(accel_dev);
    if (status) {
        ADF_ERROR("failed to initialize accelerator\n");
        adf_debugShutdown(accel_dev);
        return status;
    }
    return SUCCESS;
}

/*
 * adf_init_devices
 *
 * Catchall function to initialize the Acceleration subsystem for all
 * accelerators found in the system
 */
int adf_init_devices(int accel_id,  unsigned int ioctl_cmd)
{
    int status = SUCCESS;
    icp_accel_dev_t *accel_dev = NULL;
    struct pci_dev *pdev;

    /*Start all acceleration devices. */
    if (ADF_CFG_ALL_DEVICES == accel_id) {
        if (adf_devmgrGetAccelHead(&accel_dev) != CPA_STATUS_SUCCESS) {
            ADF_ERROR("Failed to get an accelerator.\n");
            return FAIL;
        }

        while (accel_dev) {
            if(BIT_IS_SET(accel_dev->adfSubsystemStatus,
                ADF_STATUS_SYSTEM_STARTED)) {
                ADF_PRINT("System already started.\n");
            } else if (IOCTL_START_ACCEL_DEV == ioctl_cmd &&
                       BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                  ADF_STATUS_SYSTEM_RESTARTING)) {
                           ADF_ERROR("System already restarting.\n");
                           return CPA_STATUS_RETRY;
            } else {

                /*Initialise all subsystems. */
                status = adf_init_all(accel_dev);
                if (SUCCESS != status) {
                    ADF_ERROR("failed to init accel\n");
                    return FAIL;
                }
                pdev = accel_dev->pciAccelDev.pDev;
                accel_dev->pci_state = osalPCIStateStore(pdev,
                                                accel_dev->pkg_id);
            }
            accel_dev = accel_dev->pNext;
        }
    } /*else start only the accel device with this id. */
    else {
        accel_dev =  adf_devmgrGetAccelDevByAccelId(accel_id);
        if (accel_dev) {

            if(BIT_IS_SET(accel_dev->adfSubsystemStatus,
                ADF_STATUS_SYSTEM_STARTED)) {
                ADF_PRINT("System already started.\n");
            } else if (IOCTL_START_ACCEL_DEV == ioctl_cmd &&
                       BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                  ADF_STATUS_SYSTEM_RESTARTING)) {
                           ADF_ERROR("System already restarting.\n");
                           return CPA_STATUS_RETRY;
            } else {
                /*Initialise all subsystems. */
                status = adf_init_all(accel_dev);
                if (SUCCESS != status) {
                    ADF_ERROR("failed init accel %d\n",
                              accel_dev->accelId);
                    return FAIL;
                }
                pdev = accel_dev->pciAccelDev.pDev;
                accel_dev->pci_state = osalPCIStateStore(pdev,
                                               accel_dev->pkg_id);
            }
        } else {
            ADF_ERROR("failed to find accel with id %d\n",
                      accel_id);
            return FAIL;
        }
    }
    return SUCCESS;
}

/*
 * adf_config_ctl_stop_devices
 *
 * Stop all devices and subsystems registered.
 */
#define ORPHAN_WAIT_TIME 250
int adf_config_ctl_stop_devices(int dev_id)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_accel_dev_t *accel_dev = NULL;

    if (adf_devmgrGetAccelHead(&accel_dev) != CPA_STATUS_SUCCESS) {
        ADF_ERROR("Failed to get an accelerator.\n");
        return FAIL;
    }
    while (accel_dev) {
           /*
        * Check if orphan thread is running for the device
        * and if so wait for it ot finish
        */
        while(BIT_IS_SET(accel_dev->adfSubsystemStatus,
            ADF_STATUS_ORPHAN_TH_RUNNING))
        {
            msleep(ORPHAN_WAIT_TIME);
        }
        if ((dev_id == accel_dev->accelId)
                || (ADF_CFG_ALL_DEVICES == dev_id)) {
            if((BIT_IS_SET(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_SYSTEM_STARTED))
                || (BIT_IS_SET(accel_dev->adfSubsystemStatus,
                    ADF_STATUS_SYSTEM_STARTING))) {

                status = adf_subsystemStop(accel_dev);
                if (status == CPA_STATUS_FAIL) {
                    ADF_ERROR("Acceleration "
                        "Subsystem stop failure\n");
                }
                status = adf_subsystemShutdown(accel_dev);
                if (status != CPA_STATUS_SUCCESS) {
                    ADF_ERROR("Acceleration "
                        "Subsystem shutdown failure\n");
                }

                status = adf_user_proxyShutdown(accel_dev);
                if (status != CPA_STATUS_SUCCESS) {
                    ADF_ERROR("User proxy "
                          "shutdown failure\n");
                }
                adf_put_module();
            }
            /* Cleanup the ETR Data */
            adf_cleanup_ETR_Data(accel_dev);
            adf_debugShutdown(accel_dev);
        }
        accel_dev = accel_dev->pNext;
    }
    return SUCCESS;
}

/*
 * adf_config_ctl_reset_devices
 *
 * Attempt to reset all devices registered.  Limited checking here because all
 * it does is schedule an async device reset.  icp_adf_reset_dev() will only do
 * that if a device is already started and is not currently resetting.
 */
int adf_config_ctl_reset_devices(int dev_id)
{
    icp_accel_dev_t *accel_dev = NULL;

    if (adf_devmgrGetAccelHead(&accel_dev) != CPA_STATUS_SUCCESS) {
        ADF_ERROR("Failed to get an accelerator.\n");
        return FAIL;
    }
    while (accel_dev) {
        if ((dev_id == accel_dev->accelId)
                || (ADF_CFG_ALL_DEVICES == dev_id)) {
            icp_adf_reset_dev(accel_dev, ICP_ADF_DEV_RESET_ASYNC);
        }
        accel_dev = accel_dev->pNext;
    }
    return SUCCESS;
}
