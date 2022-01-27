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
 * @file icp_adf_cfg.h
 *
 * @defgroup icp_AdfCfg Acceleration Driver Framework Configuration Interface.
 *
 * @ingroup icp_Adf
 *
 * @description
 *      This is the top level header file for the run-time system configuration
 *      parameters. This interface may be used by components of this API to
 *      access the supported run-time configuration parameters.
 *
 *****************************************************************************/

#ifndef ICP_ADF_CFG_H
#define ICP_ADF_CFG_H

#include "cpa.h"
#include "icp_accel_devices.h"

/******************************************************************************
* Section for #define's & typedef's
******************************************************************************/
/* Address of the UOF firmware */
#define ICP_CFG_UOF_ADDRESS_KEY         ("Firmware_UofAddress")
/* Size of the UOF firmware */
#define ICP_CFG_UOF_SIZE_BYTES_KEY      ("Firmware_UofSizeInBytes")
/* Address of the MMP firmware */
#define ICP_CFG_MMP_ADDRESS_KEY         ("Firmware_MmpAddress")
/* Size of the MMP firmware */
#define ICP_CFG_MMP_SIZE_BYTES_KEY      ("Firmware_MMpSizeInBytes")
/* Physical address of the MMP firmware */
#define ICP_CFG_MMP_PHYS_ADDRESS_KEY    ("Firmware_MmpPhysAddress")
/* MMP firmware version */
#define ICP_CFG_MMP_VER_KEY             ("Firmware_MmpVer")
/* UOF firmware version */
#define ICP_CFG_UOF_VER_KEY             ("Firmware_UofVer")
/* Tools version */
#define ICP_CFG_TOOLS_VER_KEY           ("Firmware_ToolsVer")
/* Hardware rev id */
#define ICP_CFG_HW_REV_ID_KEY           ("HW_RevId")
/* SRAM Physical Address Key */
#define ADF_SRAM_PHYSICAL_ADDRESS       ("Sram_PhysicalAddress")
/* SRAM Virtual Address Key */
#define ADF_SRAM_VIRTUAL_ADDRESS        ("Sram_VirtualAddress")
/* SRAM Size In Bytes Key */
#define ADF_SRAM_SIZE_IN_BYTES          ("Sram_SizeInBytes")
/* Device node id, tells to which die the device is
 * connected to */
#define ADF_DEV_NODE_ID                 ("Device_NodeId")
/* Device package id, this is accel_dev id */
#define ADF_DEV_PKG_ID                  ("Device_PkgId")
/* Device bus address, B.D.F (Bus(8bits),Device(5bits),Function(3bits)) */
#define ADF_DEV_BUS_ADDRESS             ("Device_BusAddress")
/* Number of Acceleration Engines */
#define ADF_DEV_NUM_AE                  ("Device_Num_AE")
/* Number of Accelerators */
#define ADF_DEV_NUM_ACCEL               ("Device_Num_Accel")
/* Max Number of Acceleration Engines */
#define ADF_DEV_MAX_AE                  ("Device_Max_AE")
/* Max Number of Accelerators */
#define ADF_DEV_MAX_ACCEL               ("Device_Max_Accel")
/* QAT/AE Mask*/
#define ADF_DEV_ACCELAE_MASK_FMT        ("Device_Accel_AE_Mask_%d")
/* VF ring offset */
#define ADF_VF_RING_OFFSET_KEY          ("VF_RingOffset")
/* Configure reporting of parity error */
#define ICP_CFG_REPORT_DC_PARITY_ERROR_KEY  ("ReportParityCompressionError")

/* Maximum lengths for the configuration values. */
#define ADF_CFG_MAX_KEY_LEN_IN_BYTES     (64)
#define ADF_CFG_MAX_VAL_LEN_IN_BYTES     (64)
#define ADF_CFG_MAX_SECTION_LEN_IN_BYTES (64)
#define ADF_CFG_BASE_DEC                 (10)
#define ADF_CFG_BASE_HEX                 (16)
#define ADF_CFG_HEX_PREFIX_LENGTH        (2)

/* String names for the exposed sections of config file. */
#define GENERAL_SEC                     "GENERAL"
#define WIRELESS_SEC                    "WIRELESS_INT_"
#define DYN_SEC                         "DYN"
#define DEV_LIMIT_CFG_ACCESS_TMPL       "_D_L_ACC"

/*
 * icp_adf_cfgGetParamValue
 *
 * Description:
 * This function is used to determine the value for a given parameter name.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_cfgGetParamValue(icp_accel_dev_t *accel_dev,
                                   const char *section,
                                   const char * param_name,
                                   char * param_value);
/*
 * icp_adf_cfgGetParamValueList
 *
 * Description:
 * This function is used to determine the values configured for a given
 * set of parameter names.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_cfgGetParamValueList(icp_accel_dev_t *accel_dev,
                                       int numElements,
                                       const char * section[],
                                       const char * param_names[],
                                       char * param_values[]);
/*
 * icp_adf_cfgGetRingNumber
 *
 * Description:
 * Function returns ring number configured for the service.
 * NOTE: this function will only be used by QATAL in kernelspace.
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_cfgGetRingNumber(icp_accel_dev_t *accel_dev,
                                   const char *section_name,
                                   const Cpa32U accel_num,
                                   const Cpa32U bank_num,
                                   const char *pServiceName,
                                   Cpa32U* pRingNum);

/*
 * icp_adf_get_busAddress
 * Gets the B.D.F. of the physical device
 */
Cpa16U icp_adf_get_busAddress(Cpa16U packageId);


#endif /* ICP_ADF_CFG_H */
