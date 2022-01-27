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
 * @file adf_cfg.h
 *
 * @description
 *      This is the header file for the ADF Resource Configuration Table.
 *      This file also contains function prototypes for configuration and
 *      struct definitions.
 *
 *****************************************************************************/
#ifndef ADF_CFG_H
#define ADF_CFG_H

#include "icp_adf_cfg.h"
#include "adf_platform.h"
#include "adf_cfg_types.h"

/******************************************************************************
* Section for #define's & typedef's
******************************************************************************/
/* Address of the UOF firmware*/
#define ICP_CFG_UOF_ADDRESS_KEY         ("Firmware_UofAddress")
/* Size of the UOF firmware */
#define ICP_CFG_UOF_SIZE_BYTES_KEY      ("Firmware_UofSizeInBytes")
/* CDRAM Address */
#define ICP_CFG_CDRAM_ADDRESS_KEY       ("Cdram_Address")
/* CDRAM Size */
#define ICP_CFG_CDRAM_SIZE_BYTES_KEY    ("Cdram_SizeInBytes")
/* NCDRAM address */
#define ICP_CFG_NCDRAM_ADDRESS_KEY      ("Ncdram_Address")
/* NCDRAM Size*/
#define ICP_CFG_NCDRAM_SIZE_BYTES_KEY   ("Ncdram_SizeInBytes")

/* MOF Firmware Name Configuration Key */
#define ADF_AE_FW_MOF_NAME_KEY          ("Firmware_MofPath")
/* UOF Firmware Name Configuration Key */
#define ADF_AE_FW_UOF_NAME_KEY          ("Firmware_Uof")
/* MMP Firmware Name Configuration Key */
#define ADF_AE_FW_MMP_NAME_KEY          ("Firmware_MmpPath")
/* SRIOV Enable Key */
#define ADF_SRIOV_ENABLE_KEY            ("SRIOV_Enabled")
/* Enable or disable reporting of compression parity error */
#define ICP_CFG_REPORT_DC_PARITY_ERROR_KEY  ("ReportParityCompressionError")
/* PF_bundle_offset key*/
#define ADF_PF_BUNDLE_OFFSET_KEY        ("PF_bundle_offset")

/* Coalescing Enabled Configuration Key Format */
#define ADF_ETRMGR_COALESCING_ENABLED_FORMAT    \
                                        "Bank%dInterruptCoalescingEnabled"
/* Coalescing Timer Configuration Key Format */
#define ADF_ETRMGR_COALESCE_TIMER_FORMAT        \
                                        "Bank%dInterruptCoalescingTimerNs"
/* Core ID Affinity Key Format */
#define ADF_ETRMGR_COREID_AFFINITY_FORMAT        \
                                        "Bank%dCoreIDAffinity"
/* Ring nearly watermark as a number of messages */
#define ADF_NEARLY_FULL                 "Bank%dInterruptCoalescingNumResponses"
/* Internal debug freature */
#define ADF_PROC_DEBUG                  ("ProcDebug")
#define ACCEL_STR                       "Accelerator%d"
/* Number of Wireless Procs that will have rings pre-created on start */
#define NUMBER_OF_WIRELESS_PROCS        ("NumberOfWirelessProcs")

/* Max Number of Banks in a Device */
#define ADF_DEV_MAX_BANKS               ("DevCfg_Max_Banks")
/* Max Number of Rings Per Bank */
#define  ADF_DEV_MAX_RINGS_PER_BANK     ("DevCfg_Max_Rings_Per_Bank")
/* Accelerator Mask */
#define ADF_DEV_ACCEL_MASK              ("DevCfg_Accel_Mask")
/* Accleration Engine Mask */
#define ADF_DEV_AE_MASK                 ("DevCfg_Ae_Mask")
/* DEV SKU */
#define ADF_DEV_SKU                     ("DevCfg_SKU")
/* Capabilities Mask */
#define ADF_DEV_CAPABILITIES_MASK       ("DevCfg_Capabilities_Mask")
/* User Space CSR ring Size */
#define ADF_DEV_USER_RING_CSR_SIZE      ("DevCfg_User_Ring_Csr_Size")
/* Userspace configuration for Multidevice system is limited to this device */
#define ADF_CFG_LIMIT_DEV_ACCESS        ("LimitDevAccess")

/* String names for the internal sections in config file. */
#define INTERNAL_SEC                    "INTERNAL"
#define SYMBOLS_SEC                     "SYMBOLS"
#define INTERNAL_USERSPACE_SEC_SUFF     "_INT_"

#define CY "Cy"
#define DC "Dc"
/******************************************************************************
* Section for struct definitions
******************************************************************************/
/*
 * Structure to hold device information.
 * Each device requires three tables, one for the
 * configuration info (taken from the configuration file), one for
 * internal info (taken from bios) and one for the symbols used
 * to patch the firmware.
 */
typedef struct adf_cfg_device_data_s
{
    unsigned int device_id;
    adf_cfg_section_t *config_section;
    adf_cfg_section_t *internal_section;
    adf_cfg_section_t *symbol_section;
    void *debug_hndl;
    struct adf_cfg_device_data_s *pPrev;
    struct adf_cfg_device_data_s *pNext;
} adf_cfg_device_data_t;

/******************************************************************************
* Section for interface function prototypes
******************************************************************************/

/******************************************************************************
 * @description
 * Initialize configuration internal settings
 *
 ******************************************************************************/
CpaStatus adf_cfgInit(void);

/******************************************************************************
 * @description
 * Uninitialize configuration internal settings
 *
 ******************************************************************************/
void adf_cfgUninit(void);

/******************************************************************************
 * @description
 * Search the accelerator table for a given device with device_id.
 *
 ******************************************************************************/
CpaStatus adf_cfgDeviceFind(Cpa32S device_id, adf_cfg_device_data_t** cfg_data);

/******************************************************************************
 * @description
 * Add a device with device id equal to device_id to the accelerator table.
 *
 ******************************************************************************/
CpaStatus adf_cfgDeviceAdd(Cpa32S device_id);

/******************************************************************************
 * @description
 * Remove a device with device id equal to device_id to the accelerator table.
 *
 ******************************************************************************/
CpaStatus adf_cfgDeviceRemove(Cpa32S device_id);

/******************************************************************************
 * @description
 * Add a node of key-value parameter to the key-value table.
 * It is assumed that the key-value node buffer has been allocated.
 *
 ******************************************************************************/
CpaStatus adf_cfgKeyValueAddNode(adf_cfg_key_val_t **keyValueList,
                                 adf_cfg_key_val_t *keyValueNode);


/******************************************************************************
 * @description
 * Add a section node to the configuration table.
 *
 ******************************************************************************/
CpaStatus adf_cfgSectionAddNode(adf_cfg_section_t **pSectionList,
                                adf_cfg_section_t *pSectionNode);

/******************************************************************************
 * @description
 * This function adds a section to the section list.
 *
 *****************************************************************************/
CpaStatus adf_cfgSectionAdd(icp_accel_dev_t *accel_dev,
                            adf_cfg_section_t **pSectionList,
                            adf_cfg_section_t *pSectionNode);

/******************************************************************************
 * @description
 * Delete all Section parameters from the section table.
 *
 ******************************************************************************/
void adf_cfgSectionDel(adf_cfg_section_t **pSectionList);

/******************************************************************************
 * @description
 * This function enables the functionality to add a key-value
 * parameter into a section list. Calls are made to adf_cfgDeviceFind
 * and adf_cfgKeyValueAddNode from within adf_cfgAddKeyValueParam.
 *
 *****************************************************************************/
CpaStatus adf_cfgAddKeyValueParam(icp_accel_dev_t *accel_dev,
                                  adf_cfg_section_t *pSectionList,
                                  char *key,
                                  void *val,
                                  adf_cfg_val_type_t type);

/******************************************************************************
 * @description
 * This function enables other internal components to delete a key-value
 * parameter from the configuration table. Calls are made to adf_cfgDeviceFind
 * and adf_cfgKeyValueDelete from within adf_cfgDelKeyValueParam.
 *
 *****************************************************************************/
CpaStatus adf_cfgDelKeyValueParam(icp_accel_dev_t *accel_dev,
                                  adf_cfg_section_t *pSectionNode,
                                  char *pKey);

/******************************************************************************
 * @description
 * Gets the ring number associated to a given service name.
 *
 *****************************************************************************/
CpaStatus adf_cfgGetRingNumber(icp_accel_dev_t *accel_dev,
                               const char *service_name,
                               const Cpa32U accel_num,
                               const Cpa32U bank_num,
                               const char *pServiceName,
                               Cpa32U *ringnum);

/******************************************************************************
 * @description
 *
 * Find a section pointer given a section name.
 *****************************************************************************/
adf_cfg_section_t *adf_cfgSectionFind(adf_cfg_section_t *pSectionList,
                                      const char *pSectionName);


/******************************************************************************
 * @description
 *
 * Create a debug file for device config table
 *****************************************************************************/
CpaStatus adf_debug_create_cfg(icp_accel_dev_t *accel_dev,
                               adf_cfg_device_data_t *dev_cfg);
/******************************************************************************
 * @description
 *
 * adf_debug_remove_cfg
 * Remove debug file for device config table
 *****************************************************************************/
void adf_debug_remove_cfg(adf_cfg_device_data_t *dev_cfg);

#endif /* ADF_CFG_H */
