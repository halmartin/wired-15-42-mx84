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
 * @file adf_cfg_main.c
 *
 * @description
 *      Implementation of the ADF configuration control to parse
 *      the configuration file.
 *****************************************************************************/
#include "icp_accel_devices.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "adf_config.h"
#include "adf_config_ctl.h"

#define ADF_OP_START    ("up")
#define ADF_OP_STOP     ("down")
#define ADF_OP_STATUS   ("status")
#define ADF_OP_RESET    ("reset")
#define ADF_OP_PARSE   ("parse")
#define START_ALL_CASE      1
#define UP_DOWN_RESET_CASE  2
#define DEVICE_NUM_CASE     3
#define PARSE_ONLY_CASE     4
#define SERVICE_BUFFER_LEN  30
#define ACCEL_NUM_EVEN_ODD  2
#define DH895XCC_MAX_NUM_PROCESSES (64)
#define MAX_ACCELERATOR_NUM 2
/* Bundle offset range values */
#define DH895XCC_MIN_IOV_PF_BUNDLE_OFFSET 0 
#define DH89XXCC_MIN_IOV_PF_BUNDLE_OFFSET 1
#define DH89XXCC_MAX_IOV_PF_BUNDLE_OFFSET 7
#define DH89XXCC_DEFAULT_IOV_PF_BUNDLE_OFFSET 1
#define PARSE_SEMICOLON     ';'
#define PARSE_OPEN_BRACKET  '['
#define PARSE_HASH          '#'
#define PARSE_UNDERSCORE    '_'
#define ARG_TWO              2
#define ARG_THREE            3

#define PARSE_ONLY_NUM_DEV   1

#define ICP_DH89XXCC_MAX_ACCEL 2
#define ICP_DH89XXCC_MAX_BANK  16
#define ICP_DH89XXCC_NUM_ACCEL 2
#define ICP_DH89XXCC_NUM_AE    2
#define ICP_DH89XXCC_NUM_BANKS_PER_ACCEL 8
#define ICP_DH89XXCC_NUM_RINGS_PER_BANK  16
#define ICP_DH89XXCC_NODE_ID   1

#define ICP_DH895XCC_MAX_ACCEL 6
#define ICP_DH895XCC_MAX_BANK  32
#define ICP_DH895XCC_NUM_ACCEL 6
#define ICP_DH895XCC_NUM_AE    12
#define ICP_DH895XCC_NUM_BANKS_PER_ACCEL 32
#define ICP_DH895XCC_NUM_RINGS_PER_BANK  16
#define ICP_DH895XCC_NODE_ID   1

#define ICP_C2XXX_MAX_ACCEL 1
#define ICP_C2XXX_MAX_BANK  8
#define ICP_C2XXX_NUM_ACCEL 1
#define ICP_C2XXX_NUM_AE    2
#define ICP_C2XXX_NUM_BANKS_PER_ACCEL 8
#define ICP_C2XXX_NUM_RINGS_PER_BANK  16
#define ICP_C2XXX_NODE_ID   1

/*
 * Systems may exist with a mixture of devices fitted.
 * The first entries in the following enum must match the entries in
 * the device_formats[] and device_names array below.
 */

#define ICP_GENERIC_DEVICE_NAME   "icp_dev"
#define NUM_DRIVERS 2

enum {
    DEV_TYPE_ICP,
    DEV_TYPE_DH89XXCC,
    DEV_TYPE_C2XXX,
    DEV_TYPE_DH895XCC,
    DEV_TYPE_NONE,
    DEV_TYPE_ALL
};

STATIC char *device_formats[] = {
    ICP_GENERIC_DEVICE_NAME"%d",
    ICP_DH89XXCC_DEVICE_NAME".%d",
    ICP_C2XXX_DEVICE_NAME".%d",
    ICP_DH895XCC_DEVICE_NAME".%d"
};

STATIC char *device_names[] = {
        ICP_GENERIC_DEVICE_NAME,
        ICP_DH89XXCC_DEVICE_NAME,
        ICP_C2XXX_DEVICE_NAME,
        ICP_DH895XCC_DEVICE_NAME,
};


/* Per-driver data.  This is partially initialised at compile time, but
 * fd and num_dev are updated at runtime.
 */
STATIC adf_driver_data_t drivers[] =
{
    {
        .ctl_dev = ADF_CFG_IOCTL_DEVICE_MUX_PATH,
        .ctl_fd  = -1,
        .num_dev = 0
    },
    {
        .ctl_dev = ADF_CFG_IOCTL_DEVICE_STD_PATH,
        .ctl_fd  = -1,
        .num_dev = 0
    }
};

adf_cfg_ctl_data_t ctl_data = {0};

/*
 * adf_overwrite_default_config
 *
 * This function checks if the given parameter overwrites a default config value
 * In that case, it adds it in the structure and returns CPA_TRUE
 */
STATIC CpaBoolean adf_overwrite_default_config(char *param_name,
                                    char *param_value,
                                    adf_config_default_t *adf_config_default)
{
    if (0 == strncmp(param_name, CY_CONC_SYM_REQ_STR,
                     strlen(CY_CONC_SYM_REQ_STR)))
    {
        adf_config_default->cyNumConcurrentSymRequests = atoi(param_value);
        return CPA_TRUE;
    }

    if (0 == strncmp(param_name, CY_CONC_ASYM_REQ_STR,
             strlen(CY_CONC_ASYM_REQ_STR)))
    {
        adf_config_default->cyNumConcurrentAsymRequests = atoi(param_value);
        return CPA_TRUE;
    }

    if (0 == strncmp(param_name, DC_CONC_REQ_STR, strlen(DC_CONC_REQ_STR)))
    {
        adf_config_default->dcNumConcurrentRequests = atoi(param_value);
        return CPA_TRUE;
    }

    if (0 == strncmp(param_name, COALESCING_ENABLED_STR,
             strlen(COALESCING_ENABLED_STR)))
    {
        adf_config_default->interruptCoalescingEnabled = atoi(param_value);
        return CPA_TRUE;
    }

    if (0 == strncmp(param_name, COALESCING_TIMER_STR,
             strlen(COALESCING_TIMER_STR)))
    {
        adf_config_default->interruptCoalescingTimerNs = atoi(param_value);
        return CPA_TRUE;
    }

    if (0 == strncmp(param_name, COALESCING_NUM_STR,
             strlen(COALESCING_NUM_STR)))
    {
        adf_config_default->interruptCoalescingNumResponses =
            atoi(param_value);
        return CPA_TRUE;
    }
    return CPA_FALSE;
}
/*
 * adf_process_instance_param
 *
 * Process an instance parameter and generate the missing parameters if
 * enough information is available.
 * instance param processed only in case of v2,i.e.simplifiedConfig file
 */
STATIC CpaStatus adf_process_instance_param(char* param_name,
                                  char* param_value,
                                  Cpa32U num_processes,
                                  Cpa8U number_of_banks_to_skip,
                                  adf_config_subsection_t * current_subsection,
                                  adf_config_default_t* adf_config_default,
                                  CpaBoolean* do_not_copy,
                                  CpaBoolean* first_instance,
                                  Cpa8U* cy_instances_to_parse,
                                  Cpa8U* dc_instances_to_parse,
                                  adf_dev_status_info_t *dev_status)

{
    Cpa32U len = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaBoolean is_cy_inst = CPA_FALSE;
    CpaBoolean is_dc_inst = CPA_FALSE;

    status = adf_is_instance(param_name, &len, &is_cy_inst, &is_dc_inst);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }
    if ((CPA_TRUE == is_cy_inst) || (CPA_TRUE == is_dc_inst))
    {
        if (NULL != strstr(param_name, "NumConcurrentSymRequests"))
        {
            adf_config_default->numConcurrentSymRequestsOverwritten = CPA_TRUE;
        }
        if (NULL != strstr(param_name, "NumConcurrentAsymRequests"))
        {
            adf_config_default->numConcurrentAsymRequestsOverwritten =
                CPA_TRUE;
        }
        if (NULL != strstr(param_name, "DcNumConcurrentRequests"))
        {
            adf_config_default->numConcurrentDcRequestsOverwritten = CPA_TRUE;
        }

        if ((NULL != strstr(param_name, "Ring")) ||
            (NULL != strstr(param_name, "Bank")))
        {
            ADF_ERROR("ERROR: Ring and Bank numbers cannot be manually set\n");
            return CPA_STATUS_FAIL;
        }

        if (0 == strncmp(param_name, current_subsection->name, len))
        {
            /* we are already using this instance */
            if (0 == strncmp(param_name + len, ACCEL_NUM_STR,
                strlen(ACCEL_NUM_STR)))
            {
                if (DEV_DH895XCC == dev_status->type)
                {
                   ADF_ERROR("ERROR: 'AcceleratorNumber' is not a valid"
                              " option for dh895xcc device:\n"
                              " please check dh895xcc conf file \n");
                    status = CPA_STATUS_FAIL;
                    return status;
                }
                else
                {
                   *do_not_copy = CPA_TRUE;
                   status = adf_get_range(
                            current_subsection->acceleratorNumberArray,
                            &(current_subsection->numAcceleratorNumberSet),
                            param_value);

                   if (CPA_STATUS_SUCCESS != status)
                   {
                       ADF_ERROR("Failed to get parameter range\n");
                       return status;
                   }
                }
            }
            if ((DEV_DH895XCC == dev_status->type) &&
                (0 == strncmp(param_name + len, IS_POLLED_STR,
                 strlen(IS_POLLED_STR))))
            {
                current_subsection->isPolledValueSet = 1;
            }
            if (0 == strncmp(param_name + len, CORE_AFFINITY_STR,
                    strlen(CORE_AFFINITY_STR)))
            {
                *do_not_copy = CPA_TRUE;
                status = adf_get_range(current_subsection->coreAffinityArray,
                             &(current_subsection->numCoreAffinitySet),
                             param_value);

                if (CPA_STATUS_SUCCESS != status)
                {
                    ADF_ERROR("Failed to get parameter range");
                    return status;
                }
            }
        }
        else
        {
            /* this parameter does not belong to the same instance
             * as the previous one
             */
            if ((CPA_FALSE == *first_instance) &&
                (*cy_instances_to_parse>0 || *dc_instances_to_parse>0))
            {
                if (((DEV_DH895XCC != dev_status->type) &&
                     ((0 != current_subsection->numAcceleratorNumberSet) ||
                      (0 != current_subsection->numCoreAffinitySet))) ||
                    ((DEV_DH895XCC == dev_status->type) &&
                     ((0 != current_subsection->isPolledValueSet)||
                      (0 != current_subsection->numCoreAffinitySet))))
                {
                   /* we found an entry for a new instance: close off the
                    * previous one
                    */
                   status = current_subsection->generate_config_params(
                                               current_subsection,
                                               num_processes,
                                               number_of_banks_to_skip,
                                               adf_config_default,
                                               dev_status);
                   if (CPA_STATUS_SUCCESS != status)
                   {
                       ADF_ERROR("Failed to generate config param\n");
                       return status;
                   }
                   /*This setting only pertains to dh895 processing*/
                   current_subsection->isPolledValueSet = 0;
                }
            }
            if (CPA_TRUE == is_cy_inst)
            {
                if (0 == *cy_instances_to_parse)
                {
                    /* ignore extra instances*/
                    ADF_DEBUG("Ignoring parameter %s\n", param_name);
                    *do_not_copy = CPA_TRUE;
                    return CPA_STATUS_SUCCESS;
                }
                (*cy_instances_to_parse)--;
            }
            if (CPA_TRUE == is_dc_inst)
            {
                if (0 == *dc_instances_to_parse)
                {
                    /* ignore extra instances*/
                    ADF_DEBUG("Ignoring parameter %s\n", param_name);
                    *do_not_copy = CPA_TRUE;
                    return CPA_STATUS_SUCCESS;
                }
                (*dc_instances_to_parse)--;
            }
            *first_instance = CPA_FALSE;
            snprintf(current_subsection->name, len+1, "%s", param_name);
            current_subsection->numAcceleratorNumberSet = 0;
            current_subsection->numCoreAffinitySet = 0;
            current_subsection->isPolledValueSet = 0;

            if ((DEV_DH895XCC != dev_status->type) &&
                (0 == strncmp(param_name + len, ACCEL_NUM_STR,
                    strlen(ACCEL_NUM_STR))))
            {
                *do_not_copy = CPA_TRUE;
                 status = adf_get_range(
                    current_subsection->acceleratorNumberArray,
                    &(current_subsection->numAcceleratorNumberSet),
                    param_value);
                if (CPA_STATUS_SUCCESS != status)
                {
                    ADF_ERROR("Failed to get parameter range\n");
                    return status;
                }
            }
            if ((DEV_DH895XCC == dev_status->type) &&
                (0 == strncmp(param_name + len, IS_POLLED_STR,
                 strlen(IS_POLLED_STR))))
            {
                current_subsection->isPolledValueSet = 1;
            }
            if (0 == strncmp(param_name + len, CORE_AFFINITY_STR,
                    strlen(CORE_AFFINITY_STR)))
            {
                *do_not_copy = CPA_TRUE;
                status = adf_get_range(current_subsection->coreAffinityArray,
                                &(current_subsection->numCoreAffinitySet),
                                 param_value);
                if (CPA_STATUS_SUCCESS != status)
                {
                    ADF_ERROR("Failed to get parameter range\n");
                    return status;
                }
            }
        }
    }
    return status;
}


/*
 * adf_check_cy_service_enabled
 * This function checkes if cy0 and cy1
 * are enabled in the ServicesEnabled field.
 */
STATIC void adf_check_cy_service_enabled (char *parameter_value,
                                          CpaBoolean *cy_enabled,
                                          CpaBoolean *cy0_enabled,
                                          CpaBoolean *cy1_enabled)
{
    Cpa16U param_len = 0;
    Cpa16U i = 0, j = 0;
    char buff[SERVICE_BUFFER_LEN];

    param_len = strlen(parameter_value);
    *cy_enabled  = CPA_FALSE;
    *cy0_enabled = CPA_FALSE;
    *cy1_enabled = CPA_FALSE;
    memset (buff, '\0' ,SERVICE_BUFFER_LEN);

    /* Go through all the parameter value and look for ";"
     * Once we find it, we check if the param value is "cy0"
     * or "cy1".
     */
    for (i=0; i<param_len; i++)
    {
        if (j < SERVICE_BUFFER_LEN)
        {
            if (PARSE_SEMICOLON == *(parameter_value + i))
            {
                if (0 == strcmp(buff, CY0_SERVICES_ENABLED_STR))
                {
                    *cy0_enabled = CPA_TRUE;
                }
                else if (0 == strcmp(buff, CY1_SERVICES_ENABLED_STR))
                {
                    /* Crypto service 1 is enabled */
                    *cy1_enabled = CPA_TRUE;
                }
                else if (0 == strcmp(buff, CY_SERVICES_ENABLED_STR))
                {
                    *cy_enabled = CPA_TRUE;
                }
                memset (buff, '\0' ,SERVICE_BUFFER_LEN);
                j = 0;
            }
            else
            {
                buff[j++] = *(parameter_value + i);
            }
        }
    }

    /* Check the buffer in case there would be no ";" */
    if (j > 0)
    {
        if (0 == strcmp(buff, CY0_SERVICES_ENABLED_STR))
        {
            /* Crypto service 0 is enabled */
            *cy0_enabled = CPA_TRUE;
        }
        else if (0 == strcmp(buff, CY1_SERVICES_ENABLED_STR))
        {
            /* Crypto service 1 is enabled */
            *cy1_enabled = CPA_TRUE;
        }
        else if (0 == strcmp(buff, CY_SERVICES_ENABLED_STR))
        {
            /* Crypto service (cy) is enabled */
            *cy_enabled = CPA_TRUE;
        }
    }
}

/*
 * adf_read_config_file
 *
 * This function parses the configuration file to extract the section,
 * parameter keys and its associated values. It then adds this information
 * to the ctl_data structure.
 */
int adf_read_config_file(char *filename, adf_dev_status_info_t *dev_status)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    FILE *fp = NULL;
    int i = 0;
    int line_number = 0;
    int parsed_line = 0;
    int parsed_params = 0;
    char *str_ptr = NULL;
    int acceleratorNum = 0;
    char namevalue_str[ADF_CFG_MAX_CONF_STR_LEN] = {0};
    char config_str[ADF_CFG_MAX_NUMBER_LINES][ADF_CFG_MAX_CONF_STR_LEN]= {{0}};
    char param_name[ADF_CFG_MAX_KEY_LEN_IN_BYTES] = {0};
    char param_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};
    char section_name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
    CpaBoolean section_tag = CPA_FALSE;
    adf_config_subsection_t current_subsection = {{0}};
    CpaBoolean first_instance = CPA_FALSE;
    Cpa32U num_processes = 1;
    CpaBoolean do_not_copy = CPA_FALSE;
    CpaBoolean sriov_firmware = CPA_FALSE;
    CpaBoolean wireless_firmware = CPA_FALSE;
    CpaBoolean report_compression_parity_error = CPA_FALSE;
    CpaBoolean firmware_override = CPA_FALSE;
    Cpa8U cy_instances_to_parse = 0;
    Cpa8U dc_instances_to_parse = 0;
    CpaBoolean crypto_service_enabled = CPA_FALSE;
    CpaBoolean crypto_0_service_enabled = CPA_FALSE;
    CpaBoolean crypto_1_service_enabled = CPA_FALSE;
    CpaBoolean general_section_scanned = CPA_FALSE;
    CpaBoolean verify_instances = CPA_FALSE;
    Cpa8U instance_accel_number = 0;
    Cpa8U execution_engine_number = 0;
    Cpa8U pf_bundle_offset_param_value = 0;
    Cpa8U pf_bundle_offset = 0;
    CpaBoolean dynEnabled = CPA_FALSE;

    /* default values to be used if nothing is set in the GENERAL section*/
    adf_config_default_t adf_config_default = {
        .cyNumConcurrentSymRequests = DEFAULT_CY_NUM_CONC_SYM_REQUESTS,
        .cyNumConcurrentAsymRequests = DEFAULT_CY_NUM_CONC_ASYM_REQUESTS,
        .dcNumConcurrentRequests = DEFAULT_DC_NUM_CONC_REQUESTS,
        .numConcurrentSymRequestsOverwritten = CPA_FALSE,
        .numConcurrentAsymRequestsOverwritten = CPA_FALSE,
        .numConcurrentDcRequestsOverwritten = CPA_FALSE,
        .interruptCoalescingEnabled = 1,
        .interruptCoalescingTimerNs = DEFAULT_INT_COALESCING_TIMER_NS,
        .interruptCoalescingNumResponses = DEFAULT_INT_COALESCING_NUM_RESP };

    CpaBoolean simplifiedConfig = CPA_FALSE;

    fp = fopen(filename, "r");
    if (NULL == fp)
    {
        ADF_PRINT("Error: Could not open file: %s\n", filename);
        return CPA_STATUS_FAIL;
    }
    /* Read conf file */
    while (NULL != fgets(namevalue_str, ADF_CFG_MAX_CONF_STR_LEN, fp))
    {

        if (line_number > ADF_CFG_MAX_NUMBER_LINES)
        {
            ADF_PRINT("Max number of configuration lines read.\n");
            fclose(fp);
            return CPA_STATUS_FAIL;
        }
        line_number++;
        str_ptr = adf_remove_white_spaces(namevalue_str);
        if (NULL == str_ptr)
        {
             continue;
        }
        /* Ignore comment lines and blank lines */
        if ((PARSE_HASH == *str_ptr) || (0 == *str_ptr))
        {
            continue;
        }
        strcpy(config_str[parsed_line],str_ptr);
        parsed_line++;
    }

    fclose(fp);

    memset(&ctl_data, 0, sizeof(adf_cfg_ctl_data_t));
    ctl_data.device_id = dev_status->accelId;
    /*
     * If this is DH895xCC device and V2 config file, we need to set instance
     * defaults differently
     */
    if(DEV_DH895XCC == dev_status->type)
    {
        current_subsection.generate_config_params =
                                        adf_generate_config_params_dh895x;
    }
    else
    {
        current_subsection.generate_config_params =
                                        adf_generate_config_params;
    }
    /* Parse conf file content */
    for (i=0;i<parsed_line;i++)
    {
        strcpy(str_ptr,config_str[i]);
        do_not_copy = CPA_FALSE;
        /* Start of a section in the configuration file */
        if (PARSE_OPEN_BRACKET == *str_ptr)
        {
            section_tag = CPA_FALSE;
            cy_instances_to_parse = 0;
            dc_instances_to_parse = 0;
            memset(section_name, 0, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);

            status = adf_parse_section_name(str_ptr,
                                      section_name, simplifiedConfig);
            if (CPA_STATUS_SUCCESS != status)
            {
                ADF_PRINT("%s:%d: Parse Error - skipping line\n",
                                filename, line_number);
            }
            else
            {
                /* check for dh895xcc and Accelerator number > 0 */
                acceleratorNum = adf_get_section_accelerator_num();

                if ((DEV_DH895XCC == dev_status->type)
                     && (acceleratorNum >= MAX_ACCELERATOR_NUM))
                {
                     ADF_ERROR("ERROR: 'Accelerator%d' is incompatible"
                                " with dh895xcc device:\n"
                                " only one accelerator is supported:"
                                " only 'Accelerator0' should be specified"
                                "in %s \n",
                                acceleratorNum, filename);
                     status = CPA_STATUS_FAIL;
                     return status;
                }

                if (simplifiedConfig == CPA_TRUE)
                {
                    if (((DEV_DH895XCC == dev_status->type)
                         &&(0 != current_subsection.isPolledValueSet))
                         ||((DEV_DH895XCC != dev_status->type)
                         &&((0 != current_subsection.numAcceleratorNumberSet)
                         || (0 != current_subsection.numCoreAffinitySet))))
                    {
                       /* last instance of the previous section */
                       status = current_subsection.generate_config_params(
                                                          &current_subsection,
                                                          num_processes,
                                                          pf_bundle_offset,
                                                          &adf_config_default,
                                                          dev_status);
                       if (CPA_STATUS_SUCCESS != status)
                       {
                           return status;
                       }
                    }
                }
                /*This makes sure the instance isn't added to the next section
                  for dh895xcc*/
                current_subsection.isPolledValueSet = 0;
                status = adf_add_section(&(ctl_data.config_section),
                                         section_name);
                if (CPA_STATUS_SUCCESS != status)
                {
                    ADF_PRINT("unable to add new section\n");
                    return status;
                }
                section_tag = CPA_TRUE;
                num_processes = 1;
            }
            continue;
        }
        if (CPA_FALSE == section_tag)
        {
            ADF_PRINT("%s:%d: Section Name unknown - skipping line\n",
                                filename, line_number);
            continue;
        }
        memset(&param_name, 0, ADF_CFG_MAX_KEY_LEN_IN_BYTES);
        memset(&param_value, 0, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
        /*
         * str_ptr is a string with a parameter name followed
         * by a value, for example: AsymCrypto_Tx_Lo = 0,13.
         * str_ptr also has all the whitespaces removed from
         * the previous function.
         */
        status = adf_parse_parameter_pair(dev_status->accelId,
                                          str_ptr,
                                          param_name,
                                          param_value,
                                          dev_status);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_PRINT("%s:%d: Parse Error - skipping line\n",
                                filename, line_number);
            continue;
        }
        if (0 == strncmp(param_name, CONFIG_VERSION_STR,
                 strlen(CONFIG_VERSION_STR)))
        {
            if (atoi(param_value) == ADF_CONFIG_VERSION_2)
            {
                simplifiedConfig = CPA_TRUE;
                continue;
            }
        }
        /*
         * Verify enabled services
         */
        if (0 == strncmp(param_name, SERVICES_ENABLED_STR,
                 strlen(SERVICES_ENABLED_STR)))
        {
            /* We found the "ServicesEnabled", therefore we
             * now look for CY, CY0, CY1 or DC
             */
            adf_check_cy_service_enabled (param_value,
                    &crypto_service_enabled,
                    &crypto_0_service_enabled,
                    &crypto_1_service_enabled);

            /* Verify instances only when one crypto service is enabled */
            if (crypto_0_service_enabled ^ crypto_1_service_enabled)
            {
                verify_instances = CPA_TRUE;
            }
            /* check for cy0 or cy1 with dh895 */
            if ((DEV_DH895XCC == dev_status->type)
                && (crypto_0_service_enabled || crypto_1_service_enabled))
            {
                ADF_ERROR("ERROR: cy(n) is incompatible with dh895xcc device:"
                            " use 'cy' in %s \n", filename);
                status = CPA_STATUS_FAIL;
                return status;
            }

        }
        /* Make sure we've passed the general section in the file */
        if (0 == strncmp(section_name, KERNEL, strlen(KERNEL)))
        {
            general_section_scanned = CPA_TRUE;
        }
        /*dh89xxcc & C2XXX supports ACCEL_NUM & EXEC_ENGINE param*/
        if ((DEV_DH895XCC == dev_status->type)
            && (CPA_TRUE == general_section_scanned))
        {
           if (strstr(param_name, EXEC_ENGINE_STR))
           {
               ADF_ERROR("ERROR: 'ExecutionEngine' is not a valid option"
                         " for dh895xcc device:\n"
                         "         please check %s \n", filename);
               status = CPA_STATUS_FAIL;
               return status;
           }
           if (strstr(param_name, ACCEL_NUM_STR))
           {
               ADF_ERROR("ERROR: 'AcceleratorNumber' is not a valid option"
                         " for dh895xcc device:\n"
                         "        please check %s \n", filename);
               status = CPA_STATUS_FAIL;
               return status;
           }
        }
        else if ((DEV_DH895XCC != dev_status->type)
                  && (CPA_TRUE == verify_instances
                   && CPA_TRUE == general_section_scanned))
        {
            if (strstr(param_name, ACCEL_NUM_STR))
            {
                instance_accel_number = atoi(param_value);
            }
            if (CPA_TRUE == simplifiedConfig)
            {
                if (instance_accel_number > MAX_ACCEL_NUMBER)
                {
                    ADF_ERROR("Cy<n>AcceleratorNumber should be from 0"
                            " to 3 for crypto.\n");
                    status = CPA_STATUS_FAIL;
                    return status;
                }
                /* Config file version 2 */
                if( (strstr(param_name, ACCEL_NUM_STR)) &&
                    (0 == (instance_accel_number % ACCEL_NUM_EVEN_ODD)) &&
                    (CPA_FALSE == crypto_0_service_enabled))
                {
                    ADF_ERROR("AcceleratorNumber should be either 1 or 3 "
                        "when only cy1 crypto service enabled, "
                        "found AcceleratorNumber = %d at line number %d\n",
                        instance_accel_number, line_number);

                    status = CPA_STATUS_FAIL;
                    return status;
                }
                else if((strstr(param_name, ACCEL_NUM_STR)) &&
                        ((instance_accel_number % ACCEL_NUM_EVEN_ODD) > 0) &&
                        (CPA_FALSE == crypto_1_service_enabled))
                {
                    ADF_ERROR("AcceleratorNumber should be either 0 or 2 "
                        "when only cy0 crypto service enabled, "
                        "found AcceleratorNumber = %d at line number %d\n",
                        instance_accel_number, line_number);
                    status = CPA_STATUS_FAIL;
                    return status;
                }
            }
            else
            {
                /* Config file version 1 */
                if (strstr(param_name, EXEC_ENGINE_STR))
                {
                    execution_engine_number = atoi(param_value);
                }
                if ((0 == execution_engine_number) &&
                    (CPA_FALSE == crypto_0_service_enabled))
                {
                    ADF_ERROR("Cy<n>ExecutionEngine should be 1 when only cy1 "
                        "is the only crypto service enabled\n");
                    status = CPA_STATUS_FAIL;
                    return status;
                }
                else if ((1 == execution_engine_number) &&
                        (CPA_FALSE == crypto_1_service_enabled))
                {
                    ADF_ERROR("Cy<n>ExecutionEngine should be 0 when only cy0 "
                        "is the only crypto service enabled\n");
                    status = CPA_STATUS_FAIL;
                    return status;
                }
            }
        }
        if (0 == strncmp(param_name, NUM_CY_INST_STR, strlen(NUM_CY_INST_STR)))
        {
            cy_instances_to_parse = atoi(param_value);
        }
        if (0 == strncmp(param_name, NUM_DC_INST_STR, strlen(NUM_DC_INST_STR)))
        {
            dc_instances_to_parse = atoi(param_value);
        }
        if ((0 == strncmp(section_name, DYN, strlen(DYN))) &&
            ((0 != cy_instances_to_parse) || (0 != dc_instances_to_parse)))
        {
            dynEnabled = CPA_TRUE;
        }
        if (strstr(param_name, COALESCING_NUM_STR))
        {
            if (COALESCING_NUM_MAX < atoi(param_value))
            {
                ADF_ERROR("ERROR: %s cannot be greater than %d\n",
                           COALESCING_NUM_STR, COALESCING_NUM_MAX);
                status = CPA_STATUS_FAIL;
                return status;
            }
        }
#ifndef ICP_SRIOV
#ifndef PARSE_ONLY
        if (strstr(param_name, ADF_SRIOV_ENABLE_KEY))
        {
            if(0 != atoi(param_value))
            {
                ADF_ERROR("%s is enabled in the config file, but driver"
                          " has been built without SR_IOV support.\n"
                          "Please rebuild with ICP_SRIOV variable set to 1\n",
                          ADF_SRIOV_ENABLE_KEY);
                status = CPA_STATUS_FAIL;
                return status;
            }
        }
#endif
#endif

        if (CPA_TRUE == adf_overwrite_default_config(param_name,
                        param_value, &adf_config_default))
        {
            continue;
        }
        if (0 == strncmp(param_name, NUM_PROCESSES_STR,
                 strlen(NUM_PROCESSES_STR)))
        {
            if (0 == strncmp(section_name, KERNEL, strlen(KERNEL)))
            {
                ADF_ERROR("numProcesses ignored for Kernel sections\n");
                continue;
            }
            if (0 == strncmp(section_name, DYN, strlen(DYN)))
            {
                ADF_ERROR("numProcesses ignored for DYN sections\n");
                continue;
            }
            if (0 == strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)))
            {
               ADF_ERROR("numProcesses is not a valid entry for"
                " GENERAL section\n");
               continue;
            }

            num_processes = atoi(param_value);

            if((DEV_DH895XCC == dev_status->type)
                 && (num_processes > DH895XCC_MAX_NUM_PROCESSES))
            {
                ADF_ERROR("ERROR: %s must be less than or equal to the number "
                        "of available processes [%d] for this device\n",
                          NUM_PROCESSES_STR, DH895XCC_MAX_NUM_PROCESSES);
                status = CPA_STATUS_FAIL;
                return status;
            }

            do_not_copy = CPA_TRUE;
            if (0 == num_processes)
            {
                cy_instances_to_parse = 0;
                dc_instances_to_parse = 0;
            }
            else
            {
                if ((0 == cy_instances_to_parse) &&
                    (0 == dc_instances_to_parse))
                {
                    num_processes = 0;
                    ADF_PRINT("WARNING: No instances were set in "
                            "the configuration file in [%s] section "
                            "therefore the parameter 'NumProcesses' "
                            "has been set to 0.\n", section_name);
                }
            }
            if ((num_processes >= 1) ||
                (0 == strncmp(section_name, WIRELESS, strlen(WIRELESS))))
            {
                status = adf_duplicate_section(num_processes,
                                               &wireless_firmware,
                                               dev_status->type);
                if (CPA_STATUS_SUCCESS != status)
                {
                    ADF_ERROR("Failed to set up parameters for "
                              "user-space processes\n");
                    return status;
                }
            }
        }
        /*dh89xxcc & DEV_C2XXX device supports NUMBER_OF_WIRELESS_PROCS param*/
        if (0 == strncmp(param_name, NUMBER_OF_WIRELESS_PROCS,
                 strlen(NUMBER_OF_WIRELESS_PROCS)))
        {
            if (0 == strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)))
            {
                if (simplifiedConfig)
                {
                    ADF_ERROR("%s can not be used with config ver 2."
                     " Please set NumProcesses in the WIRELESS section"
                     " instead.\n",
                     NUMBER_OF_WIRELESS_PROCS);
                    status = CPA_STATUS_FAIL;
                    return status;
                }
                else
                {
                    if(atoi(param_value) > 0)
                    {
                       if (DEV_DH895XCC != dev_status->type)
                       {
                           wireless_firmware = CPA_TRUE;
                       }
                       else
                       {
                          ADF_ERROR
                          ("ERROR:'%s' is incompatible with dh895xcc device.\n"
                                     "'%s' should be specified in %s \n",
                           NUMBER_OF_WIRELESS_PROCS, WIRELESS_ENABLED,
                           filename);
                           status = CPA_STATUS_FAIL;
                           return status;
                       }
                    }
                }
            }
            else
            {
                ADF_ERROR("%s must be in the"
                 " GENERAL section\n", NUMBER_OF_WIRELESS_PROCS);
                status = CPA_STATUS_FAIL;
                return status;
            }
        }
        /*only dh895xcc device supports WIRELESS_ENABLED param*/
        if (0 == strncmp(param_name, WIRELESS_ENABLED,
                         strlen(WIRELESS_ENABLED)))
        {
            if (0 == strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)))
            {
                if(atoi(param_value) > 0)
                {
                   if (DEV_DH895XCC == dev_status->type)
                   {
                      wireless_firmware = CPA_TRUE;
                   }
                   else
                   {
                          ADF_ERROR
                          ("ERROR:'%s' is incompatible with dh89xxcc device.\n"
                                     "'%s' should be specified in %s \n",
                           WIRELESS_ENABLED,NUMBER_OF_WIRELESS_PROCS,
                           filename);
                           status = CPA_STATUS_FAIL;
                           return status;
                   }
                }
            }
            else
            {
                ADF_ERROR("ERROR: %s must be in the"
                 " GENERAL section\n", WIRELESS_ENABLED);
                status = CPA_STATUS_FAIL;
                return status;
            }
        }
        /*only dh895xcc and dh89xxcc devices support report parity enable */
        if (0 == strncmp(param_name, ICP_CFG_REPORT_DC_PARITY_ERROR_KEY,
                         strlen(ICP_CFG_REPORT_DC_PARITY_ERROR_KEY)))
        {
            if (0 == strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)))
            {
                if (1 == atoi(param_value))
                {
                   if (DEV_DH895XCC == dev_status->type ||
                       DEV_DH89XXCC == dev_status->type)
                   {
                       report_compression_parity_error = CPA_TRUE;
                   }
                   else /* device incompatible with parameter */
                   {
                          ADF_ERROR
                          ("ERROR:'%s' is incompatible with c2xxx device.\n",
                           ICP_CFG_REPORT_DC_PARITY_ERROR_KEY);
                   }
                } 
                else /* default is CPA_FALSE */
                {
                   report_compression_parity_error = CPA_FALSE;
                }
            }
            else /* parameter is in the wrong section */
            {
                ADF_ERROR("ERROR: %s must be in the"
                 " GENERAL section\n", ICP_CFG_REPORT_DC_PARITY_ERROR_KEY);
                status = CPA_STATUS_FAIL;
                return status;
            }
        }
        /*only dh89xxcc & dh895xcc device supports PF_BUNDLE_OFFSET param*/
        if (0 == strncmp(param_name, ADF_SRIOV_ENABLE_KEY,
                 strlen(ADF_SRIOV_ENABLE_KEY)))
        {
            if (0 == strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)))
            {
               sriov_firmware = (CpaBoolean)atoi(param_value);
               if (DEV_DH89XXCC == dev_status->type)
               {
                   if (CPA_TRUE==sriov_firmware)
                   {
                       /* Change 'pf_bundle_offset' to 1 by default as soon as
                        * SRIOV is enabled.
                        */
                       pf_bundle_offset = DH89XXCC_DEFAULT_IOV_PF_BUNDLE_OFFSET;
                   }
               }
               if (DEV_DH895XCC == dev_status->type)
               {
                   if (CPA_TRUE==sriov_firmware)
                   { 
                      pf_bundle_offset = dev_status->maxNumBanks;
                   }
               }
            }
            else
            {
                ADF_ERROR("%s must be in the"
                 " GENERAL section\n", ADF_SRIOV_ENABLE_KEY);
                status = CPA_STATUS_FAIL;
                return status;
            }
        }
        if (strstr(param_name, ADF_PF_BUNDLE_OFFSET_KEY))
        {
            pf_bundle_offset_param_value = atoi(param_value);
            
            if(DEV_DH895XCC == dev_status->type)
            {
                if(((pf_bundle_offset_param_value < 
                        DH895XCC_MIN_IOV_PF_BUNDLE_OFFSET) ||
                    (pf_bundle_offset_param_value >= dev_status->maxNumBanks)))
                {
                    ADF_ERROR("%s must be within the range %d to %d\n",
                            ADF_PF_BUNDLE_OFFSET_KEY,
                            DH895XCC_MIN_IOV_PF_BUNDLE_OFFSET, 
                            dev_status->maxNumBanks-1);
                    status = CPA_STATUS_FAIL;
                    return status;
                }
                else if(CPA_TRUE == sriov_firmware)
                {
                    /* SRIOV is ON and the bundle offset is in the correct range
                     * therefore set the offset specified by PF_bundle_offset.
                     */
                    pf_bundle_offset = pf_bundle_offset_param_value;
                }
            }
            else if (DEV_DH89XXCC == dev_status->type)
            {
                if((pf_bundle_offset_param_value < 
                        DH89XXCC_MIN_IOV_PF_BUNDLE_OFFSET) ||
                    (pf_bundle_offset_param_value > 
                        DH89XXCC_MAX_IOV_PF_BUNDLE_OFFSET))
                {
                    ADF_ERROR("%s must be within the range %d to %d\n",
                            ADF_PF_BUNDLE_OFFSET_KEY,
                            DH89XXCC_MIN_IOV_PF_BUNDLE_OFFSET, 
                            DH89XXCC_MAX_IOV_PF_BUNDLE_OFFSET);
                    status = CPA_STATUS_FAIL;
                    return status;
                }
                else if(CPA_TRUE == sriov_firmware)
                {
                    pf_bundle_offset = pf_bundle_offset_param_value;
                }
            }
        }
        /*
         * We check for Firmware_Uof in the config file, if it's there we use
         * it. If it's not, we generate the name based on the number
         * of wireless process being >0 and sriov being enabled.
         */
        if (0 == strncmp(param_name, ADF_AE_FW_UOF_NAME_KEY,
                 strlen(ADF_AE_FW_UOF_NAME_KEY)))
        {
            if (0 == strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)))
            {
                firmware_override = CPA_TRUE;
            }
            else
            {
                ADF_ERROR("%s must be in the"
                 " GENERAL section\n", ADF_AE_FW_UOF_NAME_KEY);
                status = CPA_STATUS_FAIL;
                return status;
            }
        }
        if ((0 == strncmp(param_name, ADF_CFG_LIMIT_DEV_ACCESS,
             strlen(ADF_CFG_LIMIT_DEV_ACCESS))) && simplifiedConfig)
        {
            if (1 == atoi(param_value))
            {
                if (adf_rename_section(section_name,
                                       dev_status->accelId,
                                       num_processes))
                {
                    status = CPA_STATUS_FAIL;
                    return status;
                }
            }
            continue;
        }
        if (CPA_TRUE == simplifiedConfig)
        {
            status = adf_process_instance_param(param_name,
                                                param_value,
                                                num_processes,
                                                pf_bundle_offset,
                                                &current_subsection,
                                                &adf_config_default,
                                                &do_not_copy,
                                                &first_instance,
                                                &cy_instances_to_parse,
                                                &dc_instances_to_parse,
                                                dev_status);
            if (CPA_STATUS_SUCCESS != status)
            {
                ADF_ERROR("Failed to processs instance parameter\n");
                return status;
            }
        }
        if (CPA_TRUE != do_not_copy)
        {
            status = adf_add_multiple_section_param(param_name,
                                                    param_value,
                                                    num_processes);
            if (CPA_STATUS_SUCCESS != status)
            {
                ADF_PRINT("%s:%d: Failed to add parameter '%s'\n",
                           filename, line_number, param_name);
                return status;
            }
        }
        parsed_params++;
    }
    if (CPA_TRUE == simplifiedConfig)
    {
        if (((DEV_DH895XCC == dev_status->type)
             &&(0 != current_subsection.isPolledValueSet))
            ||((DEV_DH895XCC != dev_status->type)
             &&((0 != current_subsection.numAcceleratorNumberSet)
            ||(0 != current_subsection.numCoreAffinitySet))))
       {
           status = current_subsection.generate_config_params(
                                               &current_subsection,
                                               num_processes,
                                               pf_bundle_offset,
                                               &adf_config_default,
                                               dev_status);
           if (CPA_STATUS_SUCCESS != status)
           {
              return status;
           }
        }
    }
    if ((CPA_TRUE == dynEnabled) && (CPA_TRUE == wireless_firmware))
    {
        ADF_ERROR("%s: Cannot support both WIRELESS and DYN sections\n",
                                          filename);
        return CPA_STATUS_FAIL;
    }
    if (!parsed_params)
    {
        ADF_PRINT("%s: No configuration parameters found\n", filename);
        return CPA_STATUS_FAIL;
    }
    if (DEV_DH895XCC != dev_status->type)
    {
        status = adf_set_admin_param(dev_status->numAccel);
        if (CPA_STATUS_SUCCESS != status)
        {
           ADF_PRINT("Error: failed to set the admin params\n");
           adf_delall_section(&ctl_data.config_section);
           return status;
        }
    }
    /* Set UOF_Firmware correctly based on Wireless/SRIOV settings */
    if (CPA_FALSE == firmware_override)
    {
        status = adf_set_firmware_name(dev_status->type,
                                       wireless_firmware,
                                       sriov_firmware);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_PRINT("Error: Failed to set Firmware Name\n");
            return status;
        }
    }

    /* Set report_compression_parity_error based on value determined above */
    /* If parameter is not found in config file then default is CPA_FALSE  */
    status = adf_config_set_report_parity_error(
                 report_compression_parity_error);

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("Error: Failed to set report parity\n");
        return status;
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * adf_ctl_help
 *
 * Prints a simple help on how to use the tool
 */
STATIC void adf_ctl_help( char *prog )
{
    ADF_PRINT("Use:\n  %s [dev] [up|down|reset] - "
              "to bring up or down or reset device(s)\n", prog);
    ADF_PRINT("  or:\n  %s status - to print device(s) status\n", prog);
    ADF_PRINT("\nDon't use any parameters to configure the device(s)\n");
}

/* ***************************************************************************
 * adf_log_dev_status()
 * prints out all data got from driver via ioctl
 *
 * ***************************************************************************/
void adf_log_dev_status(adf_dev_status_info_t *dev_status)
{
    int i = 0;
    adf_dev_status_info_t *device = NULL;

    for(i = 0; i < ADF_MAX_DEVICES; i++)
    {
        device = &dev_status[i];
        if (device->numAccel > 0)
        {
            ADF_DEBUG("icp_dev%d:\n"
                    "  accelId=%d, numAe=%d, numAccel=%d,"
                    "  maxNumAccel=%d, MaxNumBanks=%d\n"
                    "  numBanksPerAccel=%d, numRingsPerBank=%d\n"
                    "  name=%s, instId=%d, node_id=%d, state=%s,"
                    "  type=%d, bdf=%02x:%02x:%1d\n",
                    i,
                    device->accelId                        ,
                    device->numAe                          ,
                    device->numAccel                       ,
                    device->maxNumAccel                    ,
                    device->maxNumBanks                    ,
                    device->numBanksPerAccel               ,
                    device->numRingsPerBank                ,
                    device->deviceName                     ,
                    device->instanceId                     ,
                    device->nodeId                         ,
                    (device->state == DEV_UP) ? "up" : "down",
                    device->type                           ,
                    device->busId                          ,
                    device->slotId                         ,
                    device->functionId
                    );
        }
    }
}

/*
 * adf_set_driver_and_id
 *
 * Given an instance id type, work out which driver and driver specific
 * index to use. 
 * Devices have different driver instances and start from instance 0
 * Example numbering in a system with multiple devices:
 * icp_dev0 dh89xxcc.0 instance 0 in qat_1.5 driver
 * icp_dev1 c2xxx.0 instance 0 in qat_1.5 driver
 * icp_dev2 dh895xcc.0 instance 0 in qat_1.6 driver
 * icp_dev3 dh895xcc.1 instance 1 in qat_1.6 driver
 *
 */
STATIC CpaStatus adf_set_driver_and_id(int id_type, int dev_id,
                                                    int *dr, int *id)
{
    int d = 0;
    int i = 0;

    if (id_type == DEV_TYPE_ICP)
    {
        i = dev_id;
        for (d = 0; d < NUM_DRIVERS; d++)
        {
            if (i < drivers[d].num_dev)
            {
                *dr = d;
                *id = i;
                return CPA_STATUS_SUCCESS;
            }
            i -= drivers[0].num_dev;
        }
        ADF_PRINT("Error: There is no icp_dev%d in the system\n", dev_id);
        return CPA_STATUS_FAIL;
    }

    for (d = 0; d < NUM_DRIVERS; d++)
    {
        for (i = 0; i < drivers[d].num_dev; i++)
        {
            if (drivers[d].dev_status[i].instanceId == dev_id &&
                      !strcmp(device_names[id_type],
                              drivers[d].dev_status[i].deviceName))
            {
                *dr = d;
                *id = i;
                return CPA_STATUS_SUCCESS;
            }
        }
    }
    ADF_PRINT("Error: There is no %s instance %d in the system\n",
              device_names[id_type], dev_id);
    return CPA_STATUS_FAIL;
}


/*
 * adf_open_ioctl_device
 *
 * Open the icp_adf_ctl device.
 */
int adf_open_ioctl_device(char *nodename)
{
    int fd = -1;
    int num_retries = 0;

    /*
     * Open the icp_adf_ctl device
     *
     * As the device node is created by uDev when the ADF module is loaded,
     * it may take a few seconds for the device node to appear in the /dev
     * directory tree.
     * The following retry mechanism is in place to cater for this scenario.
     */

    while ((num_retries < ADF_CFG_NUM_OPEN_ATTEMPTS) && (fd < 0))
    {
        sleep(1);
        fd = open(nodename, O_RDWR);
        num_retries++;
    }
    return fd;
}

/*
 * adf_close_ioctl_device
 *
 * Close the file.
 */
void adf_close_ioctl_device(int fd)
{
    close(fd);
}

/*
 * adf_ioctl_load
 *
 * ioctl call which sends data to kernel space.
 */
CpaStatus adf_ioctl_load(int fd)
{
    if (0 != ioctl(fd, IOCTL_CONFIG_SYS_RESOURCE_PARAMETERS, &ctl_data))
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_ioctl_stop
 *
 * Sends a request to a kernel driver to stop one or all of its devices.
 * dev_id can be a specific device number or ADF_CFG_ALL_DEVICES
 */
STATIC CpaStatus adf_ioctl_stop(int fd, int dev_id)
{
    adf_cfg_ctl_data_t ctl_data = {0};
    ctl_data.device_id = dev_id;
    int ret = 0;

    ret = ioctl(fd, IOCTL_STOP_ACCEL_DEV, &ctl_data);
    if ( 0 != ret)
    {
        if( EBUSY == ret)
        {
            ADF_PRINT("Device is in use\n");
        }
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_ioctl_stop_on_all_drivers
 *
 * Iterates over all kernel drivers to stop one or all of the devices.
 * dev_id can be a specific device number or ADF_CFG_ALL_DEVICES
 *
 */
STATIC CpaStatus adf_ioctl_stop_on_all_drivers(int id_type, int dev_id)
{
    int dr = 0;
    int id = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;

    if (dev_id == ADF_CFG_ALL_DEVICES)
    {
        for (dr = 0; dr < NUM_DRIVERS; dr++)
        {
            if (drivers[dr].ctl_fd >= 0)
            {
                status = adf_ioctl_stop(drivers[dr].ctl_fd, dev_id);
                if (CPA_STATUS_SUCCESS != status)
                {
                   continue;
                }
            }
        }
    }
    else
    {
        adf_set_driver_and_id(id_type, dev_id, &dr, &id);
        status = adf_ioctl_stop(drivers[dr].ctl_fd, id);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_ERROR("Failed to stop device \n");
        }
    }
    return status;
}

/*
 * adf_ioctl_reset
 *
 *  Sends a request to a kernel driver to reset one or all of its devices.
 */
STATIC CpaStatus adf_ioctl_reset( int fd, int dev_id )
{
    adf_cfg_ctl_data_t ctl_data = {0};

    ctl_data.device_id = dev_id;

    if (ioctl(fd, IOCTL_RESET_ACCEL_DEV, &ctl_data) != 0)
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}


/*
 * adf_ioctl_reset
 *
 * Iterates over all kernel drivers to stop one or all of the devices.
 * dev_id can be a specific device number or ADF_CFG_ALL_DEVICES
 * id_type is ?
 *
 */
STATIC CpaStatus adf_ioctl_reset_on_all_drivers(int id_type, int dev_id)
{
    CpaStatus res = CPA_STATUS_SUCCESS;
    int dr = 0;
    int id = 0;

    if (ADF_CFG_ALL_DEVICES == dev_id)
    {
        for (dr = 0; dr < NUM_DRIVERS; dr++)
        {
            if (drivers[dr].ctl_fd >= 0)
            {
                res = adf_ioctl_reset(drivers[dr].ctl_fd, dev_id);
                if (CPA_STATUS_SUCCESS != res)
                {
                   ADF_ERROR("Failed to reset device(%d)\n", dr);
                   return res;
                }
            }
        }
    }
    else
    {
        adf_set_driver_and_id(id_type, dev_id, &dr, &id);;
        res = adf_ioctl_reset(drivers[dr].ctl_fd, id);
        if (CPA_STATUS_SUCCESS != res)
        {
            ADF_ERROR("Failed to reset device\n");
        }
    }
    return res;
}

/*
 * adf_ioctl_start
 *
 * Sends a request to the kernel to start a device.
 */
STATIC CpaStatus adf_ioctl_start( int fd, int dev_id )
{
    adf_cfg_ctl_data_t ctl_data = {0};

    ctl_data.device_id = dev_id;

    if (0 != ioctl(fd, IOCTL_START_ACCEL_DEV, &ctl_data))
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_ioctl_get_num_phys_devices
 *
 * Sends a request to the kernel to get the number of accelerator devices
 * physically present on the system.
 */
STATIC CpaStatus adf_ioctl_get_num_phys_devices(int fd, int *num_phys_dev)
{
    if (0 != ioctl(fd, IOCTL_GET_NUM_DEVICES, num_phys_dev))
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * Sends a request to a kernel driver to get data for all of the
 * devices it handles.
 *
 */

STATIC CpaStatus
adf_ioctl_get_device_status(int fd, adf_dev_status_info_t *dev_status)
{
    if (0 != ioctl(fd, IOCTL_STATUS_ACCEL_DEV, &dev_status[0]))
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_ioctl_status
 *
 * Checks and prints status of all devices in the system.
 * Assumed that there is no more than 32 devices per driver.
 */
STATIC CpaStatus
adf_ioctl_status(void)
{
    adf_dev_status_info_t *device = NULL;
    int dr = 0;
    int i = 0;
    Cpa32U index = 0;

    ADF_PRINT("There is %d acceleration device(s) in the system:\n",
            drivers[0].num_dev + drivers[1].num_dev);

    for (dr = 0; dr < NUM_DRIVERS; dr++) {
        for(i = 0; i < drivers[dr].num_dev; i++, index++) {
            device = &drivers[dr].dev_status[i];

            ADF_PRINT
                (" icp_dev%d - type=%s, inst_id=%d, node_id=%d, "
                  " bdf=%02x:%02x:%1d, #accel=%d, #engines=%d, state=%s\n",
                    index                                     ,
                    device->deviceName                        ,
                    device->instanceId                        ,
                    device->nodeId                            ,
                    device->busId                             ,
                    device->slotId                            ,
                    device->functionId                        ,
                    device->numAccel                          ,
                    device->numAe                             ,
                    (device->state == DEV_UP) ? "up" : "down"
                    );
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_get_cmd
 *
 * Determines the command requested by the user.
 */
CpaStatus adf_get_cmd(int argc, char *argv[], int *cmd)
{
    if (0 == strncmp(argv[argc-1], ADF_OP_START, strlen(argv[argc-1])))
    {
       *cmd = IOCTL_START_ACCEL_DEV;
    }
    else if(0 == strncmp( argv[argc-1], ADF_OP_STOP, strlen(argv[argc-1])))
    {
       *cmd = IOCTL_STOP_ACCEL_DEV;
    }
    else if(0 == strncmp( argv[argc-1], ADF_OP_STATUS, strlen(argv[argc-1])))
    {
       *cmd = IOCTL_STATUS_ACCEL_DEV;
    }
    else if(0 == strncmp( argv[argc-1], ADF_OP_RESET, strlen(argv[argc-1])))
    {
           *cmd = IOCTL_RESET_ACCEL_DEV;
    }
    else
    {
       return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}


/*
 * adf_get_devid
 *
 * Extract the device id and type from the given device name.
 */
CpaStatus adf_get_devid(char *dev_name, int *dev_type, int *dev_id)
{
    char name[NAME_MAX] = {0};
    int  id = 0;
    int  type = DEV_TYPE_NONE;

    for (type = DEV_TYPE_ICP; type < DEV_TYPE_NONE; type++)
    {
        if (sscanf(dev_name, device_formats[type], &id))
        {
            break;
        }
    }
    if (DEV_TYPE_NONE == type)
    {
        ADF_PRINT("Error: Invalid format.%s\n", dev_name);
        return CPA_STATUS_FAIL;
    }

    snprintf(name, NAME_MAX, device_formats[type], id);

    if (strncmp(dev_name, name, strlen(dev_name)))
    {
        ADF_PRINT("Error: Invalid device name.\n");
        return CPA_STATUS_FAIL;
    }

    *dev_type = type;
    *dev_id = id;

    return CPA_STATUS_SUCCESS;
}
/* adf_read_and_parse_config_file
 * reads, parses, generates and displays internal table for one config file
 * Doesn't load table to kernel
 */
CpaStatus adf_read_and_parse_config_file(char *pathname, char *filename)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char path[NAME_MAX] = {0};
    int separator_index = 0;
    int i = 0;
    int num_dev = PARSE_ONLY_NUM_DEV;
    adf_dev_status_info_t dev_status[ADF_MAX_DEVICES] = {{0}};

    ICP_CHECK_FOR_NULL_PARAM(pathname);
    ICP_CHECK_FOR_NULL_PARAM(filename);

    memset (dev_status, 0 ,sizeof(dev_status));


    /* Extract the device name from the filename.
     * depends on filename being in format <device>_qa_devn.conf */
    for (i = 0; (i < NAME_MAX) && (filename[i] != '\0'); i++ )
    {
        if (PARSE_UNDERSCORE == filename[i])
        {
            separator_index = i;
            break;
        }
    }
    if (0 == separator_index)
    {
        ADF_PRINT("filename format invalid");
        return CPA_STATUS_FAIL;
    }
    snprintf(dev_status[0].deviceName, separator_index+1, "%s", filename);

    /* set up device data based on device type */
    if (0 == (strcmp(dev_status[0].deviceName, ICP_DH89XXCC_DEVICE_NAME)))
    {
        dev_status[0].maxNumAccel = ICP_DH89XXCC_MAX_ACCEL;
        dev_status[0].maxNumBanks = ICP_DH89XXCC_MAX_BANK;
        dev_status[0].numAccel = ICP_DH89XXCC_NUM_ACCEL;
        dev_status[0].numAe = ICP_DH89XXCC_NUM_AE;
        dev_status[0].numBanksPerAccel = ICP_DH89XXCC_NUM_BANKS_PER_ACCEL;
        dev_status[0].numRingsPerBank = ICP_DH89XXCC_NUM_RINGS_PER_BANK;
        dev_status[0].nodeId = ICP_DH89XXCC_NODE_ID;
        dev_status[0].type = DEV_DH89XXCC;

    }
    else if (0 == (strcmp(dev_status[0].deviceName, ICP_C2XXX_DEVICE_NAME)))
    {
        dev_status[0].maxNumAccel = ICP_C2XXX_MAX_ACCEL;
        dev_status[0].maxNumBanks = ICP_C2XXX_MAX_BANK;
        dev_status[0].numAccel = ICP_C2XXX_NUM_ACCEL;
        dev_status[0].numAe = ICP_C2XXX_NUM_AE;
        dev_status[0].numBanksPerAccel = ICP_C2XXX_NUM_BANKS_PER_ACCEL;
        dev_status[0].numRingsPerBank = ICP_C2XXX_NUM_RINGS_PER_BANK;
        dev_status[0].nodeId = ICP_C2XXX_NODE_ID;
        dev_status[0].type = DEV_C2XXX;
    }
    else if (0 == (strcmp(dev_status[0].deviceName, ICP_DH895XCC_DEVICE_NAME)))
    {
        dev_status[0].maxNumAccel = ICP_DH895XCC_MAX_ACCEL;
        dev_status[0].maxNumBanks = ICP_DH895XCC_MAX_BANK;
        dev_status[0].numAccel = ICP_DH895XCC_NUM_ACCEL;
        dev_status[0].numAe = ICP_DH895XCC_NUM_AE;
        dev_status[0].numBanksPerAccel = ICP_DH895XCC_NUM_BANKS_PER_ACCEL;
        dev_status[0].numRingsPerBank = ICP_DH895XCC_NUM_RINGS_PER_BANK;
        dev_status[0].nodeId = ICP_DH895XCC_NODE_ID;
        dev_status[0].type = DEV_DH895XCC;
    }
    else
    {
        ADF_PRINT("device name at start of filename not recognised");
        return CPA_STATUS_FAIL;
    }

    snprintf(path, NAME_MAX, "%s/%s", pathname, filename);
    ADF_PRINT("Processing file: %s for device %s \n",
                            path, dev_status[0].deviceName);

    adf_log_dev_status(dev_status);

    /* init core affinity data */
    if (!adf_core_affinity_init(dev_status, num_dev))
    {
         return CPA_STATUS_FAIL;
    }
    /* init available rings in all the banks*/
    if (DEV_DH895XCC == dev_status->type)
    {
        status = adf_init_bank_info_dh895x(dev_status);
    }
    else
    {
        status = adf_init_bank_info(dev_status);
    }
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to init bank_info\n");
        return status;
    }
    status = adf_read_config_file(path, dev_status);
    if (CPA_STATUS_SUCCESS == status)
    {
        ADF_PRINT("-----------------------------------------------\n");
        ADF_PRINT("Config Table derived from %s:\n\n", filename);
        adf_list_section(ctl_data.config_section);
        ADF_PRINT("-----------------------------------------------\n");
    }
    else
    {
        ADF_PRINT("---- PARSING FAILED. %d ------\n", status);
    }

    if (DEV_DH895XCC == dev_status->type)
    {
       adf_free_bank_info_dh895x(dev_status);
    }
    else
    {
       adf_free_bank_info(dev_status);
    }
    adf_delall_section(&ctl_data.config_section);

    return status;
}

/*
 * Sanity check:
 *
 * Currently, we check the "IsPoll" value here.
 *
 * For kernel space:
 *      IsPoll = 0 (interrupt mode);
 *      IsPoll = 1 (poll mode);
 *
 * For user space:
 *      IsPoll = 0 (interrupt mode, deprecated);
 *      IsPoll = 1 (poll mode);
 *      IsPoll = 2 (epoll mode -- event based polling mode);
 */
STATIC CpaStatus adf_sanity_check(adf_cfg_section_t *section, device_type_t device_type)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_cfg_key_val_t *kv = NULL;
    Cpa32U v = 0;

    while (section) {
        /* Skip the GENERAL/Accelerator0 section */
        if (!strncmp(section->name, GENERAL, strlen(GENERAL)) ||
            !strncmp(section->name, ACCELERATOR0, strlen(ACCELERATOR0))) {
            section = section->pNext;
            continue;
        }

        /* KERNEL section */
        if (!strncmp(section->name, KERNEL, strlen(KERNEL))) {
            kv = section->params;

            while (kv) {
                /* Check the "IsPoll" now */
                if (!strstr(kv->key, IS_POLLED_STR)) {
                    kv = kv->pNext;
                    continue;
                }

                v = atoi(kv->val);

                switch(v) {
                case 0:
                case 1:
                    break;
                default:
                    ADF_PRINT("IsPoll = %d is not supported.\n", v);
                    return CPA_STATUS_FAIL;
                }

                kv = kv->pNext;
            }

            section = section->pNext;
            continue;
        }

        /* The followings are the user space sections, such as "DYN", "SSL". */
        kv = section->params;
        while (kv) {
            /* Check the "IsPoll" now */
            if (!strstr(kv->key, IS_POLLED_STR)) {
                kv = kv->pNext;
                continue;
            }

            v = atoi(kv->val);

            switch(v) {
            case 0:
                /*  dh895xcc device doesn't support interrupt mode */
                if (DEV_DH895XCC == device_type)
                {
                    ADF_PRINT("User space interrupt mode (%s = 0) deprecated.\n",
                               kv->key);
                    return CPA_STATUS_FAIL;
                }
            case 1:
                /* validate the IsPoll value for dh895xcc/dh89xxcc device */
                break;
            case 2:
                /* dh89xxcc device doesn't support epolled mode */
                if (DEV_DH895XCC != device_type)
                {
                    ADF_PRINT("User space epolled mode (%s = 2) is not supported.\n",
                               kv->key);
                    return CPA_STATUS_FAIL;
                }
                break;
            default:
                ADF_PRINT("IsPoll = %d is not supported.\n", v);
                return CPA_STATUS_FAIL;
            }
            kv = kv->pNext;
        }

        section = section->pNext;
    }

    return status;
}

/*
 * adf_load_config
 *
 * This function reads the configuration file of a device and
 * sends a request to the kernel to load the configuration parameters
 * into a memory region.
 */
CpaStatus adf_load_config(int ioc_fd, adf_dev_status_info_t *dev_status)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char basename[NAME_MAX] = {0};
    char filename[NAME_MAX] = {0};

    ICP_CHECK_FOR_NULL_PARAM(dev_status);

    snprintf(basename, NAME_MAX, ADF_CFG_FILE_FORMAT, dev_status->deviceName,
            dev_status->instanceId);

    snprintf(filename, NAME_MAX,
             "%s/%s", ADF_CFG_FILE_DIR, basename);
    ADF_PRINT("Processing file: %s\n", filename);

    /* init available rings in all the banks*/

    if (DEV_DH895XCC == dev_status->type)
    {
        status = adf_init_bank_info_dh895x(dev_status);
    }
    else
    {
        status = adf_init_bank_info(dev_status);
    }

    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to init banks\n");
        return status;
    }

    status = adf_read_config_file(filename, dev_status);

    if (DEV_DH895XCC == dev_status->type)
    {
       adf_free_bank_info_dh895x(dev_status);
    }
    else
    {
       adf_free_bank_info(dev_status);
    }

    if (CPA_STATUS_SUCCESS != status)
    {
        adf_delall_section(&ctl_data.config_section);
        return status;
    }

#ifdef _DEBUG_
    adf_list_section(ctl_data.config_section);
#endif
    
     /* Added an argument to check the device type */
    status = adf_sanity_check(ctl_data.config_section, dev_status->type);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("Error: failed to load %s.\n", filename);
        adf_delall_section(&ctl_data.config_section);
        return status;
    }

    status = adf_ioctl_load(ioc_fd);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("Error: failed to load %s.\n", filename);
        adf_delall_section(&ctl_data.config_section);
        return status;
    }

    adf_delall_section(&ctl_data.config_section);
    status = adf_ioctl_start(ioc_fd, dev_status->accelId);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("Error: failed to start device %d\n", dev_status->accelId);
        return status;
    }
    return status;
}


/*
 * adf_load_config_on_all_drivers
 *
 * This function iterates across drivers and loads single or
 * multiple configuration files as required by dev_id.
 * dev_id can be a specific device number or ADF_CFG_ALL_DEVICES
 */
CpaStatus adf_load_config_on_all_drivers(int id_type, int dev_id)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    int dr = 0, id = 0, num_devices = 0;

    if (ADF_CFG_ALL_DEVICES == dev_id)
    {
        for (dr = 0; dr < NUM_DRIVERS; dr++)
        {
            num_devices += drivers[dr].num_dev;
        }
        if (num_devices == 0)
        {
            ADF_PRINT("WARNING: No acceleration devices were found. "
                      "Exiting.\n");
            return status;
        }
        for (dr = 0; dr < NUM_DRIVERS; dr++)
        {
            for (id = 0; id < drivers[dr].num_dev; id++)
            {
                status = adf_load_config(drivers[dr].ctl_fd,
                                                  drivers[dr].dev_status + id);
                if (CPA_STATUS_SUCCESS != status)
                {
                    return status;
                }
            }
        }
    }
    else
    {
        adf_set_driver_and_id(id_type, dev_id, &dr, &id);
        status = adf_load_config(drivers[dr].ctl_fd, drivers[dr].dev_status
                + id);
    }

    return status;
}

/*
 * adf_get_device_status_for_all_drivers
 *
 * Iterates across drivers opening ioctl and getting
 * device data for all devices handled by each driver
 *
 */
STATIC CpaStatus adf_get_device_status_for_all_drivers(void)
{
    CpaStatus status = CPA_STATUS_FAIL;
    int dr = 0;

    for (dr = 0; dr < NUM_DRIVERS; dr++)
    {
        if (access(drivers[dr].ctl_dev, F_OK) < 0)
        {
            continue;
        }
        drivers[dr].ctl_fd = adf_open_ioctl_device(drivers[dr].ctl_dev);
        if (drivers[dr].ctl_fd < 0)
        {
            ADF_PRINT("Error: Failed to open device %s\n", drivers[dr].ctl_dev);
            return CPA_STATUS_FAIL;
        }
        status = adf_ioctl_get_device_status(drivers[dr].ctl_fd,
                                             drivers[dr].dev_status);
        if (CPA_STATUS_FAIL == status)
        {
            continue;
        }
        status = adf_ioctl_get_num_phys_devices(drivers[dr].ctl_fd,
                                                         &drivers[dr].num_dev);
        if (CPA_STATUS_FAIL == status)
        {
            ADF_PRINT("Get number of devices failed on %s\n",
                                                          drivers[dr].ctl_dev);
            return CPA_STATUS_FAIL;
        }
        if (!adf_core_affinity_init(drivers[dr].dev_status,
                                    drivers[dr].num_dev))
        {
            return CPA_STATUS_FAIL;
        }

    }

    return CPA_STATUS_SUCCESS;
}


CpaStatus adf_process_command(int cmd, int id_type, int dev_id)
{
    CpaStatus status = CPA_STATUS_FAIL;
    int dr = 0, id = 0;

    status = adf_get_device_status_for_all_drivers();

    if (CPA_STATUS_FAIL == status)
    {
        return status;
    }

    if (dev_id != ADF_CFG_ALL_DEVICES)
    {
        if(adf_set_driver_and_id(id_type, dev_id, &dr, &id) !=
                                                 CPA_STATUS_SUCCESS)
        {
            return CPA_STATUS_FAIL;    /* Scanning for the all devices
                                          hence any device can fail */
        }
    }

    if (IOCTL_START_ACCEL_DEV == cmd)
    {
        status = adf_load_config_on_all_drivers(id_type, dev_id);
    }
    else if (IOCTL_STOP_ACCEL_DEV == cmd)
    {
        status = adf_ioctl_stop_on_all_drivers(id_type, dev_id);
    }
    else if (IOCTL_STATUS_ACCEL_DEV == cmd)
    {
        status = adf_ioctl_status();
    }
    else if (IOCTL_RESET_ACCEL_DEV == cmd)
    {
        status = adf_ioctl_reset_on_all_drivers(id_type, dev_id);
    }
    else
    {
        ADF_PRINT("Error: Invalid command id\n");
        status = CPA_STATUS_FAIL;
    }

    for (dr = 0; dr < NUM_DRIVERS; dr++)
    {
        if (drivers[dr].ctl_fd >= 0)
        {
            adf_close_ioctl_device(drivers[dr].ctl_fd);
            drivers[dr].ctl_fd = -1;
        }
    }

    if (IOCTL_START_ACCEL_DEV == cmd)
    {
        if(ADF_CFG_ALL_DEVICES != dev_id)
        {
            adf_set_driver_and_id(id_type, dev_id, &dr, &id);
            adf_config_core_affinity(id, drivers[dr].dev_status);
        }
        else
        {
            for (dr = 0; dr < NUM_DRIVERS; dr++)
            {
                for(id = 0; id < drivers[dr].num_dev; id++)
                {
                    adf_config_core_affinity(id, drivers[dr].dev_status);
                }
            }
        }
    }

    return status;
}

/*
 * main function
 */
int ADF_CFG_MAIN(int argc, char *argv[])
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    int ioc_cmd = IOCTL_START_ACCEL_DEV;
    int ioc_dev = ADF_CFG_ALL_DEVICES;
    int id_type = DEV_TYPE_ALL;

    switch(argc) {
    case START_ALL_CASE:
       /* Initialise and start all devices */
       ioc_dev = ADF_CFG_ALL_DEVICES;
       ioc_cmd = IOCTL_START_ACCEL_DEV;
       break;

    case UP_DOWN_RESET_CASE:
       /* Format: command <up|down|reset> */
       ioc_dev = ADF_CFG_ALL_DEVICES;
       status = adf_get_cmd(argc, argv, &ioc_cmd);
       if (CPA_STATUS_SUCCESS != status)
       {
            adf_ctl_help(argv[0]);
            return CPA_STATUS_FAIL;
       }
       break;

    case DEVICE_NUM_CASE:
       /* Format: command <device> <up|down|reset> */
       status = adf_get_devid(argv[1], &id_type, &ioc_dev);
       if (CPA_STATUS_SUCCESS != status)
       {
          return CPA_STATUS_FAIL;
       }

       status = adf_get_cmd(argc, argv, &ioc_cmd);
       if (CPA_STATUS_SUCCESS != status)
       {
            adf_ctl_help(argv[0]);
            return CPA_STATUS_FAIL;
       }
       break;

    case PARSE_ONLY_CASE:
        /* Format: command parse <path> <file> */
        if(strncmp( argv[1], ADF_OP_PARSE, strlen(argv[1])) == 0)
        {
            ADF_PRINT("Running in parse_only mode.\n");
            return adf_read_and_parse_config_file(argv[ARG_TWO],
                                                  argv[ARG_THREE]);
        }

        adf_ctl_help(argv[0]);
        return CPA_STATUS_FAIL;

        break;

    default:
       adf_ctl_help(argv[0]);
       return CPA_STATUS_FAIL;
       break;
    }
    status = adf_process_command(ioc_cmd, id_type, ioc_dev);
    return status;
}
