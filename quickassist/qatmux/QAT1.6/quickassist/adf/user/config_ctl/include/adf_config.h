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
 *  version: QAT1.6.L.2.6.0-65
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

/* The ioctl device node(s) to open.
 * If only one driver in the system its ioctl device uses the standard path.
 * If 2 drivers in the system (mux case) the second driver (qat1.6+) will
 * create its ioctl device at the MUX path */
#define ADF_CFG_IOCTL_DEVICE_STD_PATH            ("/dev/icp_adf_ctl")
#define ADF_CFG_IOCTL_DEVICE_MUX_PATH            ("/dev/icp_mux/icp_adf_ctl")


/* Number of retries allowed when trying to open device node for ioctl. */
#define ADF_CFG_NUM_OPEN_ATTEMPTS       10

/* The format of the configuration file. */
#define ADF_CFG_FILE_FORMAT             ("%s_qa_dev%d.conf")

/*Maximum string length allowed in config file. */
#define ADF_CFG_MAX_CONF_STR_LEN        160

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
#define CY_SERVICES_ENABLED_STR        "cy"
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
#define IS_POLLED_STR                  "IsPolled"
#define BANK_NUMBER_STR                "BankNumber"
/* section names*/
#define KERNEL                        "KERNEL"
#define GENERAL                       "GENERAL"
#define ACCELERATOR0                  "Accelerator0"
#define WIRELESS                      "WIRELESS"
#define BANKS                         "BANKS"
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

#define RING_SYM_TX_STR               "%sRingSymTx"
#define RING_SYM_RX_STR               "%sRingSymRx"
#define RING_NRBG_TX_STR              "%sRingNrbgTx"
#define RING_NRBG_RX_STR              "%sRingNrbgRx"

#define RING_DC_TX_STR                "%sRingTx"
#define RING_DC_RX_STR                "%sRingRx"

/* Processes */
#define DH89XXCC_NUM_PROCS_PER_CY_ACCEL       8
#define DH89XXCC_NUM_PROCS_PER_DC_ACCEL      16

/* Default values */
#define DEFAULT_CY_NUM_CONC_SYM_REQUESTS    512
#define DEFAULT_CY_NUM_CONC_ASYM_REQUESTS    64
#define DEFAULT_DC_NUM_CONC_REQUESTS        512
#define DEFAULT_INT_COALESCING_TIMER_NS   10000
/* Default is to have coalesing off, 0 does this */
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

#define ADF_DH895XCC_AE_FW_NAME_NORMAL         "icp_qat_ae.uof"
#define ADF_DH895XCC_AE_FW_NAME_WIRELESS       "icp_qat_ae_wireless.uof"
/*
 * A bank may not be used in multiple address spaces.
 * Therefore we must track what type of instance is created on it.
 */
#define ADF_BANK_PROCESS_NUM_NOT_SET (0xFF)
/* User process number starts at zero, so we need a special value for the
 * Kernel process section*/
#define ADF_BANK_KERN_PROCESS_NUM    (0xFE)
#define ADF_BANK_DYN_PROCESS_NUM     (0xFD)

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

typedef struct adf_config_subsection_s
{
    char name[INSTANCE_NAME_LEN];
    Cpa8U acceleratorNumberArray[MAX_RANGE_SIZE];
    Cpa8U acceleratorNumber;
    Cpa8U numAcceleratorNumberSet;
    Cpa8U coreAffinityArray[MAX_RANGE_SIZE];
    Cpa8U coreAffinity;
    Cpa8U isPolledValueSet;
    Cpa8U numCoreAffinitySet;
    CpaStatus (* generate_config_params)(
                struct adf_config_subsection_s *current_subsection,
                Cpa32U numProcesses,
                Cpa8U number_of_banks_to_skip,
                adf_config_default_t* adf_config_default,
                adf_dev_status_info_t *dev_status);
} adf_config_subsection_t;

typedef struct adf_driver_data_s {
    char *ctl_dev;
    int  ctl_fd;
    int  num_dev;
    adf_dev_status_info_t dev_status[ADF_MAX_DEVICES];
} adf_driver_data_t;


/*
 * For DH895xCC, We can store four processes for each bank which has 16 rings.
 *
 * Crypto:
 *   ALLOCATED_CY_INSTANCE_0_BIT: TX ring(0, 2, 4) ==> RX ring(8, 10, 12)
 *   ALLOCATED_CY_INSTANCE_1_BIT: TX ring(1, 3, 5) ==> RX ring(9, 11, 13)
 *
 * Compression:
 *   ALLOCATED_DC_INSTANCE_0_BIT: TX ring(6)       ==> RX ring(14)
 *   ALLOCATED_DC_INSTANCE_1_BIT: TX ring(7)       ==> RX ring(15)
 */
#define DH895xCC_MAX_PROCESS_NUM_PER_BANK    4

#define ADF_EACH_TYPE_MAX           2

enum adf_process_type {
        ADF_CY_PROCESS = 0,
        ADF_DC_PROCESS,
};

/*
 * Structure storing affinity and available rings informations in the banks
 */
typedef struct adf_bank_info_s
{
    CpaBoolean coreAffinitySet;
    /*
     * We save the process Number for a bank in the following order:
     *   index 0 ==> process for ALLOCATED_CY_INSTANCE_0_BIT
     *   index 1 ==> process for ALLOCATED_CY_INSTANCE_1_BIT
     *   index 2 ==> process for ALLOCATED_DC_INSTANCE_0_BIT
     *   index 3 ==> process for ALLOCATED_DC_INSTANCE_1_BIT
     */
#define ADF_CY_PROCESS_OFFSET  0
#define ADF_DC_PROCESS_OFFSET  2
    Cpa8U processNum[DH895xCC_MAX_PROCESS_NUM_PER_BANK];

    Cpa8U coreAffinity;
    Cpa8U firstAvailableRing;
} adf_bank_info_t;

extern adf_cfg_ctl_data_t ctl_data;

/*
 * adf_remove_white_space
 *
 * Description
 * Function to remove the white space character from the start and end
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
CpaStatus adf_parse_section_name(char * section_ptr, char *section_name,
                                 CpaBoolean simplifiedConfig);

/*
 * adf_parse_parameter_pair
 *
 * Description
 * Extract the parameter name and parameter value from the input string.
 */
CpaStatus adf_parse_parameter_pair(int dev_id,
                                   char * parameter_pair_ptr,
                                   char * param_name,
                                   char * param_value,
                                   adf_dev_status_info_t *dev_status);

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
int adf_param_core_affinity(int dev_id, const char *param_name, int *bank_nr,
                            adf_dev_status_info_t *dev_status);
/*
 * adf_set_bank_core_affinity
 *
 * Function sets the core affinity for auto generated values.
 */
int adf_set_bank_core_affinity(int dev_id, int accel, int bank, int core,
                               adf_dev_status_info_t *dev_status);

/*
 * adf_config_core_affinity
 *
 * Description
 *  Function configures the core affinity for the given bank
 */
void adf_config_core_affinity(int dev, adf_dev_status_info_t *dev_status);

/*
 * adf_set_section_accelerator_num
 *
 * Description
 * Function set accel number
 */
void adf_set_section_accelerator_num(const char *param_name);
/*
 * adf_get_section_accelerator_num
 *
 * get accel number
 */
int adf_get_section_accelerator_num();

/*
 * adf_core_affinity_init
 *
 * Description
 * Initialize variables
 */
int adf_core_affinity_init(adf_dev_status_info_t *dev_status, int num_dev);

/*
 * adf_is_instance
 *
 * checks if the string represents a CY or a DC instance
 */
CpaStatus adf_is_instance(char* str_value_ptr,
                    Cpa32U* pLen,
                    CpaBoolean *cy_instance,
                    CpaBoolean *dc_instance);

/*
 * adf_add_new_bank_default
 *
 * This function adds the default parameters associated with a new bank.
 */
CpaStatus adf_add_new_bank_default(Cpa8U bank,
                                   Cpa8U accelNumber,
                                   adf_config_subsection_t *adf_config_subsection,
                                   adf_config_default_t *adf_config_default,
                                   adf_dev_status_info_t *dev_status);
/*
 * adf_generate_config_params
 * Find the banks and rings for the given instance
 */
CpaStatus adf_generate_config_params(
                                adf_config_subsection_t *adf_config_subsection,
                                Cpa32U numProcesses,
                                Cpa8U number_of_banks_to_skip,
                                adf_config_default_t* adf_config_default,
                                adf_dev_status_info_t *dev_status);
/*
 * adf_generate_config_params_dh895x
 * Find the banks and rings for the given DH895x instance
 */
int adf_generate_config_params_dh895x(
                                adf_config_subsection_t *adf_config_subsection,
                                Cpa32U numProcesses,
                                Cpa8U number_of_banks_to_skip,
                                adf_config_default_t* adf_config_default,
                                adf_dev_status_info_t *dev_status);


/* initialise the available rings for all the banks */
CpaStatus adf_init_bank_info(adf_dev_status_info_t *dev_status);
CpaStatus adf_init_bank_info_dh895x(adf_dev_status_info_t *dev_status);

/* free the bank array */
void adf_free_bank_info(adf_dev_status_info_t *dev_status);
void adf_free_bank_info_dh895x(adf_dev_status_info_t *dev_status);

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
                                CpaBoolean *wireless_firmware,
                                device_type_t type);

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
 * This function renames section name to provide per device configuration
 * for userspace processes where the "LimitDevAccess" param is set.
 */
CpaStatus adf_rename_section(char *section_name,
                             Cpa32U dev_id,
                             Cpa32U numProcesses);
/*
 * adf_add_param_from_string
 *
 * This function adds the parameter with param_value and with the name composed
 * of the string format and the name in the section passed in.
 */
CpaStatus adf_add_param_from_string(Cpa32U param_value,
                                    char* string_format,
                                    char* name,
                                    adf_cfg_section_t *pSection);
/* list all the sections */
void adf_list_section(adf_cfg_section_t *section);

#endif /* ADF_CONFIG_H */
