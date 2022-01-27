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
 * @file adf_drv.h
 *
 * @description
 *      This is the header file for the adf_common_drv.c source file.
 *      It contains the functions for initialising, starting, stopping
 *      and shutting down the devices and registered subsystems.
 *
 *****************************************************************************/
#ifndef ADF_DRV_H
#define ADF_DRV_H

#define GIGE_WATCHDOG_NOT_RUNNING -1

/* OS Driver name */
extern char adf_driver_name[];

/*
 * adf_init_devices
 * Initialize the Acceleration Subsystem for the device
 * id.
 * Returns: SUCCESS, FAIL
 */
int adf_init_devices(int dev_id, unsigned int ioctl_cmd);

/*
 * adf_init_all
 * Initialise all the subsystems.
 *  Returns: SUCCESS, FAIL
 */
int adf_init_all(icp_accel_dev_t *accel_dev);

/*
 * adf_config_ctl_stop_devices
 * Stop the Acceleration devices
 * Returns: SUCCESS, FAIL
 */
int adf_config_ctl_stop_devices(int dev_id);

/*
 * adf_config_ctl_reset_devices
 * Reset the Acceleration devices
 * Returns: SUCCESS, FAIL
 */
int adf_config_ctl_reset_devices(int dev_id);

/*
 * driver_module_init
 * Initialise the pci device and register the adf ctl character device driver.
 * Returns: SUCCESS, FAIL
 */
extern int driver_module_init(void);

/*
 * adf_sriov_enable
 * Enable accel dev virtualization
 * Returns: SUCCESS, FAIL
 */
int adf_enable_sriov(icp_accel_dev_t *accel_dev);

/*
 * adf_sriov_disable
 * Disable accel dev virtualization
 */
void adf_disable_sriov(icp_accel_dev_t *accel_dev);

/*
 * adf_vf_clean_ring_info
 * Unset autopush and send ring info update to AE
 */
int adf_vf_clean_ring_info(icp_accel_pci_vf_info_t *vf);

/*
 * driver_module_exit
 * Unregisters the pci device driver and
 * unregister the adf ctl character device driver.
 */
extern void driver_module_exit(void);

/*
 * adf_enable_aer
 * Enable accel dev advanced error reporting
 */
int adf_enable_aer(icp_accel_dev_t *accel_dev,
                  void *adf_driver);
/*
 * adf_disable_aer
 * Disable accel dev advanced error reporting
 */
void adf_disable_aer(icp_accel_dev_t *accel_dev);

/*
 * adf_init_aer
 */
int adf_init_aer(void);

/*
 * adf_exit_aer
 */
void adf_exit_aer(void);

/*
 * adf_restore_dev
 * Brings device to state just after probe - ready to be started.
 * Function is used for error handling.
 */
int adf_restore_dev(icp_accel_dev_t *accel_dev);

/*
 * adf_aer_schedule_reset_dev
 * Function to be used for hardware uncorrectable error handing and
 * other hardware issues.
 * After receiving an error interrupt indicating uncorrectable error
 * in the acceleration complex this function should be called from
 * top half interrupt handler in async mode. It will queue a worker
 * thread that will log the event in syslog and restart the device.
 */
int adf_aer_schedule_reset_dev(icp_accel_dev_t *accel_dev, int sync);

/*
 * adf_gige_notify_restarting_dev
 * If a gige watchdog is running this function notifies it that
 * the device will be restarted so that it can bring down the
 * network interfaces before the restart.
 */
int adf_gige_notify_restarting_dev(icp_accel_dev_t *accel_dev);

/* adf_enable_busmaster
 * Since Busmasters of VFs in the Host are disabled after a reset, this function
 * enables the BusMaster of the VFs, if the VF device is enabled
 * PARAM - pdev
 */
void adf_enable_busmaster(struct pci_dev *pdev);

#endif /* ADF_DRV_H */
