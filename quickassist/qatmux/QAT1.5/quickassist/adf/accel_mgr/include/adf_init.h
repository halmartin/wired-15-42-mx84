/******************************************************************************
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

/******************************************************************************
 * @file adf_init.h
 *
 * @description
 *      This header file that contains the prototypes and definitions required
 *      for ADF initialisation.
 *
 *****************************************************************************/
#ifndef ADF_INIT_H
#define ADF_INIT_H

#include "icp_accel_devices.h"
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"


/*
 * Version numbers for ADF Version Registration
 *
 * Major Version Number is updated upon significant feature change(s)
 * Significant POR changes
 *
 * Minor Version Number is updated upon less significant feature change(s)
 *
 * Patch Version Number is updated upon bug fixes, release candidate updates
 */
#define ADF_MAJOR_VERSION 0
#define ADF_MINOR_VERSION 0
#define ADF_PATCH_VERSION 1

/*
 * Initialisation value for the adfModuleId, a value of 0 implies ADF has
 * not registered the version information with the DCC component.
 */
#define VERSION_INFO_UNREGISTERED 0

/*
 * This macro sets the specified bit in status to 1
 * i.e. set the bit
 */
#define SET_STATUS_BIT( status, bit )   status |= (1 << bit)

/*
 * This macro sets the specified bit in status to 0
 * i.e. clears the bit
 */
#define CLEAR_STATUS_BIT( status, bit )  status &= ~(1 << bit)

/*
 * This macro checks if the specified bit in status is set or not.
 */
#define BIT_IS_SET( status, bit )       ( status & (1 << bit) )

/*
 * ADF system status bits
 */
#define ADF_STATUS_ISR_RES_ALLOCATED  0  /* ISR Service res are allocated*/
#define ADF_STATUS_AE_INITIALISED     1  /* AccelEngines are initialised */
#define ADF_STATUS_AE_UCODE_MAPPED    2  /* AccelEngine microcode is mapped*/
#define ADF_STATUS_AE_UCODE_LOADED    3  /* AccelEngines are loaded */
#define ADF_STATUS_AE_STARTED         4  /* AccelEngines are started */
#define ADF_STATUS_ORPHAN_TH_RUNNING  5  /* Orphnan thread running on dev*/
#define ADF_STATUS_AE_UOF_LOADED      6  /* UOF FW is not released*/
#define ADF_STATUS_SRIOV_ENABLED      7  /* SRIOV is enabled*/
#define ADF_STATUS_WIRELESS_ENABLED   8  /* Wireless firmware is enabled*/
#define ADF_STATUS_SYSTEM_RESTARTING  28 /* ADF is restartind device */
#define ADF_STATUS_SYSTEM_STARTING    29 /* ADF is starting device */
#define ADF_STATUS_SYSTEM_CONFIGURED  30 /* ADF has configured device */
#define ADF_STATUS_SYSTEM_STARTED     31 /* ADF has started device */

/*
 * Pending time in ms that ADF will sleep before sending shutdown
 * when a subsystem return pending on stop
 */
#define PENDING_DELAY 100

/**
 *****************************************************************************
 * @description
 *      This function will initialise the AEs, map the firmware,
 *      send an init event to the subservice and call adf_subsystemStart.
 *
 *****************************************************************************/
CpaStatus adf_subsystemInit( icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This function will download the Ucode, start the AEs and send
 *      a start event to the subservices
 *
 *****************************************************************************/
CpaStatus adf_subsystemStart( icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This function will load the firmware, patch the previously loaded
 *      symbols and send a start event to the subservices
 *
 *****************************************************************************/
CpaStatus adf_subsystemResume( icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This function will stop the subcomponents in the system,
 *      and free resources for ISR and firmware loading that have been allocated
 *
 *****************************************************************************/
CpaStatus adf_subsystemStop( icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This is a wrapper function to adf_subsystemStop called when the OS
 *      issues a suspend.
 *
 *****************************************************************************/
CpaStatus adf_subsystemSuspend( icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This function will shutdown the subcomponents in the system.
 *
 *****************************************************************************/
CpaStatus adf_subsystemShutdown( icp_accel_dev_t *accel_dev);

#endif /* ADF_INIT_H */
