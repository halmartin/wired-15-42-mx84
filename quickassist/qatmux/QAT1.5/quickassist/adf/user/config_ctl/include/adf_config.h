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
 * @file adf_config.h
 *
 * @description
 *      This is the header file for the ADF user/config executable.
 *      This file also contains function prototypes for the user space
 *      executable as well as struct and variable definitions.
 *****************************************************************************/
#ifndef ADF_CONFIG_H
#define ADF_CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>
#include "adf_config_ctl.h"
#include "icp_platform.h"
#include <sys/ioctl.h>
#include "cpa.h"

#define STATIC                          static
#define ADF_CFG_MAIN                    main

/* Directory where the configuration is located. */
#define ADF_CFG_FILE_DIR                ("/etc")

/* The device node to open. */
#define ADF_CFG_IOCTL_DEVICE            ("/dev/icp_adf_ctl")

/* Number of retries allowed when trying to open device node for ioctl. */
#define ADF_CFG_NUM_OPEN_ATTEMPTS       10

/* The format of the configuration file. */
#define ADF_CFG_FILE_FORMAT             ("%s_qa_dev%d.conf")

/*Maximum string length allowed in config file. */
#define ADF_CFG_MAX_CONFIG_STRING_LENGTH      160

/* Number of arguments for value in the configuration file. */
#define ADF_CFG_EXPECTED_NUM_ARGS       2

/* Maximum number of lines allowed in the configuration file. */
#define ADF_CFG_MAX_NUMBER_LINES        10000

#define ADF_MAX_DEVICES                 32
/* Size of the range array*/
#define MAX_RANGE_SIZE                  128

#define ADF_CONFIG_VERSION_2            2
#define PARAM_NAME_MIN_PREFIX_LEN       3

#define NUM_OF_CY_RINGS                 6
#define NUM_OF_DC_RINGS                 2

#define ADF_CFG_CHK_NULL_PARAM(param, ret) \
do {                                       \
    if (NULL == param)                     \
    {                                      \
        return ret;                        \
    }                                      \
} while(0)

#define INSTANCE_NAME_LEN 8

/* PREFIX_LEN = 3 to offset from parameters
 * starting by prefix cy0, cy1, etc
 */
#define PREFIX_LEN                      3

#define MAX_ACCEL_NUMBER                3

/* config file strings*/
#define CONFIG_VERSION_STR             "ConfigVersion"
#define SERVICES_ENABLED_STR           "ServicesEnabled"
#define CY0_SERVICES_ENABLED_STR       "cy0"
#define CY1_SERVICES_ENABLED_STR       "cy1"
#define CY_CONC_SYM_REQ_STR            "CyNumConcurrentSymRequests"
#define CY_CONC_ASYM_REQ_STR           "CyNumConcurrentAsymRequests"
#define DC_CONC_REQ_STR                "DcNumConcurrentRequests"
#define ACCEL_ADMINBANK_STR            "Accel%dAdminBankNumber"
#define ACCEL_ACCELNUM_STR             "Accel%dAcceleratorNumber"
#define ACCEL_ADMIN_TX_STR             "Accel%dAdminTx"
#define ACCEL_ADMIN_RX_STR             "Accel%dAdminRx"

#define NUM_CY_INST_STR                "NumberCyInstances"
#define NUM_DC_INST_STR                "NumberDcInstances"
#define NUM_PROCESSES_STR              "NumProcesses"
#define ACCEL_NUM_STR                  "AcceleratorNumber"
#define EXEC_ENGINE_STR                "ExecutionEngine"
#define CORE_AFFINITY_STR              "CoreAffinity"

/* section names*/
#define KERNEL                        "KERNEL"
#define WIRELESS                      "WIRELESS"
#define DYN                           "DYN"

#define COALESCING_ENABLED_STR        "InterruptCoalescingEnabled"
#define COALESCING_TIMER_STR          "InterruptCoalescingTimerNs"
#define COALESCING_NUM_STR            "InterruptCoalescingNumResponses"
#define COALESCING_NUM_MAX            248

/* Generated config file keys*/
#define BANK_NUM_STR                  "%sBankNumber"
#define RING_ASYM_TX_STR              "%sRingAsymTx"
#define RING_ASYM_RX_STR              "%sRingAsymRx"
#define RING_SYM_TX_HI_STR            "%sRingSymTxHi"
#define RING_SYM_RX_HI_STR            "%sRingSymRxHi"
#define RING_SYM_TX_LO_STR            "%sRingSymTxLo"
#define RING_SYM_RX_LO_STR            "%sRingSymRxLo"
#define DC_RING_TX_STR                "%sRingTx"
#define DC_RING_RX_STR                "%sRingRx"

/* Processes */
#define NUM_PROCS_PER_CY_ACCEL        8
#define NUM_PROCS_PER_DC_ACCEL        16

/* Default values */
#define DEFAULT_CY_NUM_CONC_SYM_REQUESTS    512
#define DEFAULT_CY_NUM_CONC_ASYM_REQUESTS    64
#define DEFAULT_DC_NUM_CONC_REQUESTS        512
#define DEFAULT_INT_COALESCING_TIMER_NS   10000
#define DEFAULT_INT_COALESCING_NUM_RESP     0

#define ADF_AE_FW_NAME_NORMAL         "icp_qat_ae_C0.uof"
#define ADF_AE_FW_NAME_WIRELESS       "icp_qat_ae_C0_wireless.uof"
#define ADF_AE_FW_NAME_SRIOV          "icp_qat_ae_C0_iov.uof"

#ifndef ICP_A0_C2XXX_SILICON
#define ADF_NAE_FW_NAME_NORMAL        "icp_qat_nae_b0.uof"
#define ADF_NAE_FW_NAME_WIRELESS      "icp_qat_nae_wireless_b0.uof"
#else
#define ADF_NAE_FW_NAME_NORMAL        "icp_qat_nae.uof"
#define ADF_NAE_FW_NAME_WIRELESS      "icp_qat_nae_wireless.uof"
#endif

typedef struct adf_config_subsection_s
{
    char name[INSTANCE_NAME_LEN];
    Cpa8U acceleratorNumberArray[MAX_RANGE_SIZE];
    Cpa8U acceleratorNumber;
    Cpa8U numAcceleratorNumberSet;
    Cpa8U coreAffinityArray[MAX_RANGE_SIZE];
    Cpa8U coreAffinity;
    Cpa8U numCoreAffinitySet;
} adf_config_subsection_t;

typedef struct adf_dev_setup_s {
    uint32_t accelId;
    uint32_t numAccel;
    uint32_t maxNumAccel;
    uint32_t maxNumBanks;
    uint32_t numBanksPerAccel;
    uint16_t *setup;
} adf_dev_setup_t;

typedef struct adf_config_default_s
{
    Cpa32U cyNumConcurrentSymRequests;
    Cpa32U cyNumConcurrentAsymRequests;
    Cpa32U dcNumConcurrentRequests;
    CpaBoolean numConcurrentSymRequestsOverwritten;
    CpaBoolean numConcurrentAsymRequestsOverwritten;
    CpaBoolean numConcurrentDcRequestsOverwritten;
    Cpa8U interruptCoalescingEnabled;
    Cpa32U interruptCoalescingTimerNs;
    Cpa32U interruptCoalescingNumResponses;
} adf_config_default_t;



extern adf_cfg_ctl_data_t ctl_data;

/*
 * adf_remove_white_space
 *
 * Description
 * Function to remove the white space characeter from the start and end
 * of the input string.
 *
 * Returns
 * Pointer to the start of the string.
 */
char* adf_remove_white_spaces(char * string_ptr);

/*
 * adf_parse_section_name
 *
 * Description
 * Extract the section name from the input string.
 */
CpaStatus adf_parse_section_name(char * section_ptr, char *section_name);

/*
 * adf_parse_parameter_pair
 *
 * Description
 * Extract the parameter name and parameter value from the input string.
 */
CpaStatus adf_parse_parameter_pair(int dev_id,
		                   char * parameter_pair_ptr,
                                   char * param_name,
                                   char * param_value);

/*
 * adf_add_section_param
 *
 * Description
 *  This function is used to add a section entry to the ctl_data
 *  structure.
 */
CpaStatus adf_add_section_param(char *param_name, char *param_value);

/*
 * adf_add_section
 *
 * Add a new section to the head of section list
 */
CpaStatus adf_add_section(adf_cfg_section_t **section,
                          char *name);

/*
 * adf_del_section
 *
 * Description
 *  Delete an entry from the list of adf_cfg_section_t
 *  structure including all its key value entries.
 */
void adf_del_section(adf_cfg_section_t **section);

/*
 * adf_delall_section
 *
 * Description
 *  Delete every entry from the list of adf_cfg_section_t
 *  structure including all its key value entries.
 */
void adf_delall_section(adf_cfg_section_t **section);

/*
 * adf_add_keyvalue
 *
 * Description
 *  Add the key, value and type variable to the adf_cfg_key_val_t
 *  structure list.
 */
CpaStatus adf_add_keyvalue(adf_cfg_key_val_t **kv,
                           char *key,
                           char *val,
                           adf_cfg_val_type_t type);

/*
 * adf_del_keyvalue
 *
 * Description
 *  Delete an entry from the list of  adf_cfg_key_val_t
 *  structures.
 */
void adf_del_keyvalue(adf_cfg_key_val_t **kv);

/*
 * adf_delall_keyvalue
 *
 * Description
 *  Delete every  entry from the list of  adf_cfg_key_val_t
 *  structures.
 */
void adf_delall_keyvalue(adf_cfg_key_val_t **kv);

/*
 * adf_list_keyvalue
 *
 * Description
 *  Display every entry from the list of adf_cfg_key_val_t
 *  structures.
 *
 */
void adf_list_keyvalues(adf_cfg_key_val_t *kv);

/*
 * adf_param_core_affinity
 *
 * Description
 * Function returns 1 if the param is bank core affinity
 */
int adf_param_core_affinity(int dev_id, const char *param_name, int *bank_nr);

/*
 * adf_set_core_affinity
 *
 * Description
 *  Function sets the core affinity for the given bank.
 *  If the settings is not correct it will be set to a default
 *  value.
 */
int adf_set_core_affinity(int dev_id, int bank, int core);

/*
 * adf_set_core_affinity_gen
 *
 * Function sets the core affinity for auto generated values.
 */
int adf_set_core_affinity_gen(int dev_id, int accel, int bank, int core);

/*
 * adf_config_core_affinity
 *
 * Description
 *  Function configures the core affinity for the given bank
 */
void adf_config_core_affinity(int dev, adf_dev_status_info_t *dev_status);

/*
 * adf_core_affinity_section_accelerator
 *
 * Description
 * Function set accel number offset
 */
void adf_core_affinity_section_accelerator(const char *param_name);

/*
 * adf_core_affinity_init
 *
 * Description
 * Initialize variables
 */
int adf_core_affinity_init(adf_dev_status_info_t *dev_status);

/*
 * adf_core_affinity_clean
 *
 * Description
 * Clean variables
 */
void adf_core_affinity_clean(void);

/*
 * adf_is_instance
 *
 * checks if the string reprensents a CY or a DC instance
 */
CpaStatus adf_is_instance(char* str_value_ptr,
                    Cpa32U* pLen,
                    CpaBoolean *cy_instance,
                    CpaBoolean *dc_instance);

/*
 * adf_​generate_con​fig_params
 * Find the banks and rings for the given instance
 */
CpaStatus adf_generate_config_params(
                                adf_config_subsection_t *adf_config_subsection,
                                Cpa32U numProcesses,
                                Cpa8U number_of_banks_to_skip,
                                adf_config_default_t* adf_config_default,
                                adf_dev_status_info_t *dev_status);


/* initialise the available rings for all the banks */
 CpaStatus adf_init_bank_info(adf_dev_status_info_t *dev_status);

 /* free the bank array */
 void adf_free_bank_info(adf_dev_status_info_t *dev_status);
/*
 * adf_set_admin_param
 * Sets the admin ring and bank
 */
CpaStatus adf_set_admin_param(int nb_accel);

/*
 * adf_find_section
 * Finds the section with the given name
 */
CpaStatus adf_find_section(adf_cfg_section_t *section_list,
                           char * name,
                           adf_cfg_section_t **out_section);

 /*
 * adf_find_section
 * Adds a section at the end of the section list
 */
CpaStatus adf_add_section_end(adf_cfg_section_t **section_list,
                             char *name,
                             adf_cfg_section_t **section_added);


 /*
 * adf_add_param
 * Adds a parameter (key/value) to the given section
 */
CpaStatus adf_add_param(char *param_name,
                        char *param_value,
                        adf_cfg_section_t *pSection);

  /*
 * adf_add_multiple_section_param
 * Adds the same parameter in the numProcesses next sections
 */
CpaStatus adf_add_multiple_section_param(char *param_name,
                                         char *param_value,
                                         Cpa32U numProcesses);


/* fills the array with the keys from the string */
CpaStatus adf_get_range(Cpa8U range_array[],
                        Cpa8U* num_ranges,
                        char*   string);

 /*
  * Duplicate the section numProcesses times, and set the
  * wireless_firmware boolean.
  */
CpaStatus adf_duplicate_section(Cpa32U numProcesses,
                                CpaBoolean *wireless_firmware);

/*
 * Set the firmware name based on the device type, SRIOV settings and
 * number of wireless processes.
 */
CpaStatus adf_set_firmware_name(device_type_t type,
                                CpaBoolean wireless_firmware,
                                CpaBoolean sriov_firmware);

/*
 * Set the report parity field.
 */
CpaStatus adf_config_set_report_parity_error(CpaBoolean report_parity);


/*
 * Set the report parity field.
 */
CpaStatus adf_config_get_report_parity_error(CpaBoolean* report_parity);


/*
 * This function renames section name to provide per device configuration
 * for userspace processes where the "LimitDevAccess" param is set.
 */
CpaStatus adf_rename_section(char *section_name,
                             Cpa32U dev_id,
                             Cpa32U numProcesses);
#ifdef _DEBUG_
/* list all the sections */
void adf_list_section(adf_cfg_section_t *section);
#endif

#endif /* ADF_CONFIG_H */
