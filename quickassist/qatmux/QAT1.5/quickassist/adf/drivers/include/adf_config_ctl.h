/****************************************************************************
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
 ****************************************************************************/

/****************************************************************************
 * @file adf_config_ctl.h
 *
 * @description
 *      Header file containing the variables needed by
 *      the configuration functions.
 *
 ****************************************************************************/
#ifndef ADF_CONFIG_CTL_H
#define ADF_CONFIG_CTL_H



#if (defined (__linux__))
#include <linux/ioctl.h>
#endif
#if (defined __KERNEL__)
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "icp_accel_devices.h"

#define ADF_CFG_ALL_DEVICES                   0xFFFE
#define ADF_CFG_NO_DEVICE                     0xFFFF


#define MAX_DEVICE_NAME_SIZE        32
#define MAX_NUM_BANKS               64
/*
 * Structure used to uniquely describe a device
 * based on device id and key-value pair.
 */
typedef struct adf_cfg_ctl_data_s {
        uint64_t device_id;
        union {
            adf_cfg_section_t *config_section;
            uint64_t           padding_config_section;
        };
} adf_cfg_ctl_data_t;

typedef struct adf_dev_status_info_s{
    Cpa32U        accelId;
    Cpa32U        numAe;
    Cpa32U        numAccel;
    Cpa32U        maxNumAccel;
    Cpa32U        maxNumBanks;
    Cpa32U        numBanksPerAccel;
    Cpa32U        numRingsPerBank;
    char          deviceName[MAX_DEVICE_NAME_SIZE + 1];
    Cpa32U        instanceId;
    Cpa32U        nodeId;
    dev_state_t   state;
    device_type_t type;
    Cpa8U         busId;
    Cpa8U         slotId;
    Cpa8U         functionId;
    #ifdef WITH_CPA_MUX
    Cpa16U        core_affinity_setup[MAX_NUM_BANKS];
    #endif
}adf_dev_status_info_t;
/*
 * IOCTL number for use between the adf_ctl and the kernel
 */
#define ADF_CTL_IOC_MAGIC 'a'
#define IOCTL_CONFIG_SYS_RESOURCE_PARAMETERS \
                _IOW(ADF_CTL_IOC_MAGIC, 0, adf_cfg_ctl_data_t)

#define IOCTL_STOP_ACCEL_DEV \
                _IOW(ADF_CTL_IOC_MAGIC, 1, adf_cfg_ctl_data_t )

#define IOCTL_START_ACCEL_DEV \
                _IOW(ADF_CTL_IOC_MAGIC, 2, adf_cfg_ctl_data_t )

#define IOCTL_STATUS_ACCEL_DEV \
                _IOW(ADF_CTL_IOC_MAGIC, 3, uint64_t)

#define IOCTL_GET_VALUE_ACCEL_DEV \
                _IOW(ADF_CTL_IOC_MAGIC, 4, adf_cfg_ctl_data_t )

#define IOCTL_GET_NUM_DEVICES \
                _IOW(ADF_CTL_IOC_MAGIC, 5, int64_t )

#define IOCTL_DEV_GET \
                _IOW(ADF_CTL_IOC_MAGIC, 6, int64_t )

#define IOCTL_DEV_PUT \
                _IOW(ADF_CTL_IOC_MAGIC, 7, int64_t )

#define IOCTL_RESET_ACCEL_DEV \
                _IOW(ADF_CTL_IOC_MAGIC, 8, adf_cfg_ctl_data_t )

#endif /* ADF_CONFIG_CTL_H */
