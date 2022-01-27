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

#define ADF_OP_START    ("up")
#define ADF_OP_STOP     ("down")
#define ADF_OP_STATUS   ("status")
#define ADF_OP_RESET    ("reset")
#define START_ALL_CASE  1
#define UP_DOWN_RESET_CASE  2
#define DEVICE_NUM_CASE 3
#define SERVICE_BUFFER_LEN  30
#define ACCEL_NUM_EVEN_ODD  2

/* Bundle offset range values */
#define MIN_BUNDLE_OFFSET 1
#define MAX_BUNDLE_OFFSET 7

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
    if (strncmp(param_name, CY_CONC_SYM_REQ_STR,
        strlen(CY_CONC_SYM_REQ_STR)) == 0)
    {
        adf_config_default->cyNumConcurrentSymRequests = atoi(param_value);
        return CPA_TRUE;
    }

    if (strncmp(param_name, CY_CONC_ASYM_REQ_STR,
        strlen(CY_CONC_ASYM_REQ_STR)) == 0)
    {
        adf_config_default->cyNumConcurrentAsymRequests = atoi(param_value);
        return CPA_TRUE;
    }

    if (strncmp(param_name, DC_CONC_REQ_STR, strlen(DC_CONC_REQ_STR)) == 0)
    {
        adf_config_default->dcNumConcurrentRequests = atoi(param_value);
        return CPA_TRUE;
    }

    if (strncmp(param_name, COALESCING_ENABLED_STR,
        strlen(COALESCING_ENABLED_STR)) == 0)
    {
        adf_config_default->interruptCoalescingEnabled = atoi(param_value);
        return CPA_TRUE;
    }

    if (strncmp(param_name, COALESCING_TIMER_STR,
        strlen(COALESCING_TIMER_STR)) == 0)
    {
        adf_config_default->interruptCoalescingTimerNs = atoi(param_value);
        return CPA_TRUE;
    }

    if (strncmp(param_name, COALESCING_NUM_STR,
        strlen(COALESCING_NUM_STR)) == 0)
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
 * enough information is available
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
    if (status != CPA_STATUS_SUCCESS)
    {
        return status;
    }
    if ((is_cy_inst == CPA_TRUE) || (is_dc_inst == CPA_TRUE))
    {
        if (strstr(param_name, "NumConcurrentSymRequests") != NULL)
        {
            adf_config_default->numConcurrentSymRequestsOverwritten = CPA_TRUE;
        }
        if (strstr(param_name, "NumConcurrentAsymRequests") != NULL)
        {
            adf_config_default->numConcurrentAsymRequestsOverwritten =
                CPA_TRUE;
        }
        if (strstr(param_name, "DcNumConcurrentRequests") != NULL)
        {
            adf_config_default->numConcurrentDcRequestsOverwritten = CPA_TRUE;
        }

        if ((strstr(param_name, "Ring") != NULL) ||
            (strstr(param_name, "Bank") != NULL))
        {
            ADF_ERROR("ERROR: Ring and Bank numbers cannot be manually set\n");
            return CPA_STATUS_FAIL;
        }

        if (strncmp(param_name, current_subsection->name, len) == 0)
        {
            /* we are already using this instance */
            if (strncmp(param_name + len, ACCEL_NUM_STR,
                strlen(ACCEL_NUM_STR)) == 0)
            {
                *do_not_copy = CPA_TRUE;
                status = adf_get_range(
                        current_subsection->acceleratorNumberArray,
                        &(current_subsection->numAcceleratorNumberSet),
                        param_value);

                if (status != CPA_STATUS_SUCCESS)
                {
                    ADF_ERROR("Failed to get parameter range\n");
                    return status;
                }
            }
            if (strncmp(param_name + len, CORE_AFFINITY_STR,
                    strlen(CORE_AFFINITY_STR)) == 0)
            {
                *do_not_copy = CPA_TRUE;
                status = adf_get_range(current_subsection->coreAffinityArray,
                             &(current_subsection->numCoreAffinitySet),
                             param_value);

                if (status != CPA_STATUS_SUCCESS)
                {
                    ADF_ERROR("Failed to get parameter range");
                    return status;
                }
            }

            if ((current_subsection->numAcceleratorNumberSet != 0) &&
                (current_subsection->numCoreAffinitySet != 0)  &&
                (*cy_instances_to_parse>0 || *dc_instances_to_parse>0))
            {
                status = adf_generate_config_params(current_subsection,
                                           num_processes,
                                           number_of_banks_to_skip,
                                           adf_config_default,
                                           dev_status);
                if (status != CPA_STATUS_SUCCESS)
                {
                    ADF_ERROR("Failed to generate config param\n");
                    return status;
                }
            }
        }
        else
        {
            /* this parameter does not belong to the same instance
             * as the previous one
             */
            if ((*first_instance == CPA_FALSE) &&
                (*cy_instances_to_parse>0 || *dc_instances_to_parse>0) &&
               ((current_subsection->numAcceleratorNumberSet !=0) ||
                (current_subsection->numCoreAffinitySet != 0)))
            {
                /* we found an entry for a new instance: close off the
                previous one */
                status = adf_generate_config_params(current_subsection,
                                           num_processes,
                                           number_of_banks_to_skip,
                                           adf_config_default,
                                           dev_status);
                if (status != CPA_STATUS_SUCCESS)
                {
                    ADF_ERROR("Failed to generate config param\n");
                    return status;
                }
            }

            if (is_cy_inst == CPA_TRUE)
            {
                if (*cy_instances_to_parse == 0)
                {
                    /* ignore extra instances*/
                    ADF_DEBUG("Ignoring parameter %s\n", param_name);
                    *do_not_copy = CPA_TRUE;
                    return CPA_STATUS_SUCCESS;
                }
                (*cy_instances_to_parse)--;
            }
            if (is_dc_inst == CPA_TRUE)
            {
                if (*dc_instances_to_parse == 0)
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

            if (strncmp(param_name + len, ACCEL_NUM_STR,
                    strlen(ACCEL_NUM_STR)) == 0)
            {
                *do_not_copy = CPA_TRUE;
                 status = adf_get_range(
                    current_subsection->acceleratorNumberArray,
                    &(current_subsection->numAcceleratorNumberSet),
                    param_value);
                if (status != CPA_STATUS_SUCCESS)
                {
                    ADF_ERROR("Failed to get parameter range\n");
                    return status;
                }
            }
            if (strncmp(param_name + len, CORE_AFFINITY_STR,
                    strlen(CORE_AFFINITY_STR)) == 0)
            {
                *do_not_copy = CPA_TRUE;
                status = adf_get_range(current_subsection->coreAffinityArray,
                                &(current_subsection->numCoreAffinitySet),
                                 param_value);
                if (status != CPA_STATUS_SUCCESS)
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
                                          CpaBoolean *cy0_enabled,
                                          CpaBoolean *cy1_enabled)
{
    Cpa16U param_len = 0;
    Cpa16U i = 0, j = 0;
    char buff[SERVICE_BUFFER_LEN];

    param_len = strlen(parameter_value);
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
            if (*(parameter_value + i) == ';')
            {
                if (strcmp(buff, CY0_SERVICES_ENABLED_STR) == 0)
                {
                    /* Crypto service 0 is enabled */
                    *cy0_enabled = CPA_TRUE;
                }
                else if (strcmp(buff, CY1_SERVICES_ENABLED_STR) == 0)
                {
                    /* Crypto service 1 is enabled */
                    *cy1_enabled = CPA_TRUE;
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
    if (*cy0_enabled == CPA_FALSE || *cy1_enabled == CPA_FALSE)
    {
        if (strcmp(buff, CY0_SERVICES_ENABLED_STR) == 0)
        {
            /* Crypto service 0 is enabled */
            *cy0_enabled = CPA_TRUE;
        }
        else if (strcmp(buff, CY1_SERVICES_ENABLED_STR) == 0)
        {
            /* Crypto service 1 is enabled */
            *cy1_enabled = CPA_TRUE;
        }
    }
}


/*
 * adf_read_config_file
 *
 * This function parses the configuration file to extracts the section,
 * parameter keys and its associated values. It then adds these information
 * to the ctl_data structure.
 */
int adf_read_config_file(char *filename, adf_dev_status_info_t *dev_status)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    FILE *fp = NULL;
    int line_number = 0;
    int parsed_params = 0;
    char *str_ptr = NULL;
    char namevalue_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
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
    CpaBoolean firmware_override = CPA_FALSE;
    Cpa8U cy_instances_to_parse = 0;
    Cpa8U dc_instances_to_parse = 0;
    CpaBoolean crypto_0_service_enabled = CPA_FALSE;
    CpaBoolean crypto_1_service_enabled = CPA_FALSE;
    CpaBoolean general_section_scanned = CPA_FALSE;
    CpaBoolean verify_instances = CPA_FALSE;
    Cpa8U instance_accel_number = 0;
    Cpa8U execution_engine_number = 0;
    Cpa8U pf_bundle_offset_param_value = 0;
    Cpa8U pf_bundle_offset = 0;
    CpaBoolean dynEnabled = CPA_FALSE;
    CpaBoolean report_compression_parity_error = CPA_FALSE;

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

    memset(&ctl_data, 0, sizeof(adf_cfg_ctl_data_t));
    ctl_data.device_id = dev_status->accelId;

    while (NULL != fgets(namevalue_str, ADF_CFG_MAX_CONFIG_STRING_LENGTH, fp))
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
        if (('#' == *str_ptr) || (0 == *str_ptr))
        {
            continue;
        }

        do_not_copy = CPA_FALSE;

        /* Start of a section in the configuration file */
        if ('[' == *str_ptr)
        {
            section_tag = CPA_FALSE;
            cy_instances_to_parse = 0;
            dc_instances_to_parse = 0;
            memset(section_name, 0, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);

            status = adf_parse_section_name(str_ptr, section_name);
            if (CPA_STATUS_SUCCESS != status)
            {
                ADF_PRINT("%s:%d: Parse Error - skipping line\n",
                                filename, line_number);
            }
            else
            {
                if (((current_subsection.numAcceleratorNumberSet !=0) ||
                    (current_subsection.numCoreAffinitySet !=0)) &&
                    (simplifiedConfig == CPA_TRUE))
                {
                    /* last instance of the previous section */
                    status = adf_generate_config_params(&current_subsection,
                                           num_processes,
                                           pf_bundle_offset,
                                           &adf_config_default,
                                           dev_status);

                    if (status != CPA_STATUS_SUCCESS)
                    {
                        fclose(fp);
                        return status;
                    }
                }

                status = adf_add_section(&(ctl_data.config_section),
                                         section_name);
                if (CPA_STATUS_SUCCESS != status)
                {
                    ADF_PRINT("unable to add new section\n");
                    fclose(fp);
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
                                          str_ptr, param_name, param_value);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_PRINT("%s:%d: Parse Error - skipping line\n",
                                filename, line_number);
            continue;
        }

        if (strncmp(param_name, CONFIG_VERSION_STR,
            strlen(CONFIG_VERSION_STR)) == 0)
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
        if (strncmp(param_name, SERVICES_ENABLED_STR,
            strlen(SERVICES_ENABLED_STR)) == 0)
        {
            /* We found the "ServicesEnabled", therefore we
             * now look for CY0, CY1 or DC
             */
            adf_check_cy_service_enabled (param_value,
                    &crypto_0_service_enabled,
                    &crypto_1_service_enabled);

            /* Verify instances only when one crypto service is enabled */
            if (crypto_0_service_enabled ^ crypto_1_service_enabled)
            {
                verify_instances = CPA_TRUE;
            }
        }

        /* Make sure we've passed the general section in the file */
        if (strncmp(section_name, KERNEL, strlen(KERNEL)) == 0)
        {
            general_section_scanned = CPA_TRUE;
        }

        if (CPA_TRUE == verify_instances &&
                CPA_TRUE == general_section_scanned)
        {
            if (strstr(param_name, ACCEL_NUM_STR))
            {
                instance_accel_number = atoi(param_value);
            }

            if (simplifiedConfig == CPA_TRUE)
            {
                if (instance_accel_number > MAX_ACCEL_NUMBER)
                {
                    ADF_ERROR("Cy<n>AcceleratorNumber should be from 0"
                            " to 3 for crypto.\n");
                    status = CPA_STATUS_FAIL;
                    fclose(fp);
                    return status;
                }

                /* Config file version 2 */
                if( (strstr(param_name, ACCEL_NUM_STR)) &&
                    ((instance_accel_number % ACCEL_NUM_EVEN_ODD) == 0) &&
                    (CPA_FALSE == crypto_0_service_enabled))
                {
                    ADF_ERROR("AcceleratorNumber should be either 1 or 3 "
                        "when only cy1 crypto service enabled, "
                        "found AcceleratorNumber = %d at line number %d\n",
                        instance_accel_number, line_number);

                    status = CPA_STATUS_FAIL;
                    fclose(fp);
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
                    fclose(fp);
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
                if ((execution_engine_number == 0) &&
                    (CPA_FALSE == crypto_0_service_enabled))
                {
                    ADF_ERROR("Cy<n>ExecutionEngine should be 1 when only cy1 "
                        "is the only crypto service enabled\n");
                    status = CPA_STATUS_FAIL;
                    fclose(fp);
                    return status;
                }
                else if ((execution_engine_number == 1) &&
                        (CPA_FALSE == crypto_1_service_enabled))
                {
                    ADF_ERROR("Cy<n>ExecutionEngine should be 0 when only cy0 "
                        "is the only crypto service enabled\n");
                    status = CPA_STATUS_FAIL;
                    fclose(fp);
                    return status;
                }
            }
        }

        if (strncmp(param_name, NUM_CY_INST_STR, strlen(NUM_CY_INST_STR)) == 0)
        {
            cy_instances_to_parse = atoi(param_value);
        }
        if (strncmp(param_name, NUM_DC_INST_STR, strlen(NUM_DC_INST_STR)) == 0)
        {
            dc_instances_to_parse = atoi(param_value);
        }

        if ((strncmp(section_name, DYN, strlen(DYN)) == 0) &&
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
                fclose(fp);
                return status;
            }
        }

#ifndef ICP_SRIOV
        if (strstr(param_name, ADF_SRIOV_ENABLE_KEY))
        {
            if(0 != atoi(param_value))
            {
                ADF_ERROR("%s is enabled in the config file, but driver"
                          " has been built without SR_IOV support.\n"
                          "Please rebuild with ICP_SRIOV variable set to 1\n",
                          ADF_SRIOV_ENABLE_KEY);
                status = CPA_STATUS_FAIL;
                fclose(fp);
                return status;
            }
        }
#endif

        if (adf_overwrite_default_config(param_name,
                param_value, &adf_config_default) == CPA_TRUE)
        {
            continue;
        }

        if (strncmp(param_name, NUM_PROCESSES_STR,
             strlen(NUM_PROCESSES_STR)) == 0)
        {
            if (strncmp(section_name, KERNEL, strlen(KERNEL)) == 0)
            {
                ADF_ERROR("numProcesses ignored for Kernel sections\n");
               continue;
            }
            if (strncmp(section_name, DYN, strlen(DYN)) == 0)
            {
                ADF_ERROR("numProcesses ignored for DYN sections\n");
               continue;
            }

            if (strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)) == 0)
            {
               ADF_ERROR("numProcesses is not a valid entry for"
                " GENERAL section\n");
               continue;
            }

            num_processes = atoi(param_value);
            do_not_copy = CPA_TRUE;
            if (num_processes == 0)
            {
                cy_instances_to_parse = 0;
                dc_instances_to_parse = 0;
            }
            else
            {
                if (0 == cy_instances_to_parse && 0 == dc_instances_to_parse)
                {
                    num_processes = 0;
                    ADF_PRINT("WARNING: No instances were set in "
                            "the configuration file in [%s] section "
                            "therefore the parameter 'NumProcesses' "
                            "has been set to 0.\n", section_name);
                }
            }
            if ((num_processes >= 1) ||
                (strncmp(section_name, WIRELESS, strlen(WIRELESS)) == 0))
            {
                status = adf_duplicate_section(num_processes,
                                                        &wireless_firmware);
                if (status != CPA_STATUS_SUCCESS)
                {
                    ADF_ERROR("Failed to set up parameters for "
                              "user-space processes\n");
                    fclose(fp);
                    return status;
                }
            }
        }

        if (strncmp(param_name, NUMBER_OF_WIRELESS_PROCS,
             strlen(NUMBER_OF_WIRELESS_PROCS)) == 0)
        {

            if (strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)) == 0)
            {
                if (simplifiedConfig)
                {
                    ADF_ERROR("%s can not be used with config ver 2."
                     " Please set NumProcesses in the WIRELESS section"
                     " instead.\n",
                     NUMBER_OF_WIRELESS_PROCS);
                    status = CPA_STATUS_FAIL;
                    fclose(fp);
                    return status;
                }
                else
                {
                    if(atoi(param_value) > 0)
                    {
                        wireless_firmware = CPA_TRUE;
                    }
                }
            }
            else
            {
                ADF_ERROR("%s must be in the"
                 " GENERAL section\n", NUMBER_OF_WIRELESS_PROCS);
                status = CPA_STATUS_FAIL;
                fclose(fp);
                return status;
            }
        }

        /* dh89xxcc device supports report compression parity enable */
        if (0 == strncmp(param_name, ICP_CFG_REPORT_DC_PARITY_ERROR_KEY,
                         strlen(ICP_CFG_REPORT_DC_PARITY_ERROR_KEY)))
        {
            if (0 == strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)))
            {
                if(1 == atoi(param_value))
                {
                   if (DEV_DH89XXCC == dev_status->type)
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
                fclose(fp);
                return status;
            }
        }

        if (strncmp(param_name, ADF_SRIOV_ENABLE_KEY,
             strlen(ADF_SRIOV_ENABLE_KEY)) == 0)
        {
            if (strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)) == 0)
            {
               if (DEV_DH89XXCC == dev_status->type)
               {
                   sriov_firmware = (CpaBoolean)atoi(param_value);
                   if (CPA_TRUE==sriov_firmware)
                   {
                       /* Change 'pf_bundle_offset' to 1 by default as soon as
                        * SRIOV is enabled.
                        */
                       pf_bundle_offset = 1;
                   }
               }
            }
            else
            {
                ADF_ERROR("%s must be in the"
                 " GENERAL section\n", ADF_SRIOV_ENABLE_KEY);
                status = CPA_STATUS_FAIL;
                fclose(fp);
                return status;
            }
        }

        if (strstr(param_name, ADF_PF_BUNDLE_OFFSET_KEY))
        {
            pf_bundle_offset_param_value = atoi(param_value);

            if (pf_bundle_offset_param_value < MIN_BUNDLE_OFFSET ||
                    pf_bundle_offset_param_value > MAX_BUNDLE_OFFSET)
            {
                ADF_ERROR("%s must be within the range %d to %d\n",
                        ADF_PF_BUNDLE_OFFSET_KEY,
                        MIN_BUNDLE_OFFSET, MAX_BUNDLE_OFFSET);
                status = CPA_STATUS_FAIL;
                fclose(fp);
                return status;
            }
            if (CPA_TRUE==sriov_firmware && DEV_DH89XXCC==dev_status->type)
            {
                /* SRIOV is ON and the bundle offset is in the correct range
                 * therefore set the offset specified by PF_bundle_offset.
                 */
                pf_bundle_offset = pf_bundle_offset_param_value;
            }
        }

        /*
         * We check for Firmware_Uof in the config file, if it's there we use
         * it. If it's not, we generate the name based on the number
         * of wireless process being >0 and sriov being enabled.
         */
        if (strncmp(param_name, ADF_AE_FW_UOF_NAME_KEY,
             strlen(ADF_AE_FW_UOF_NAME_KEY)) == 0)
        {
            if (strncmp(section_name, GENERAL_SEC, strlen(GENERAL_SEC)) == 0)
            {
                firmware_override = CPA_TRUE;
            }
            else
            {
                ADF_ERROR("%s must be in the"
                 " GENERAL section\n", ADF_AE_FW_UOF_NAME_KEY);
                status = CPA_STATUS_FAIL;
                fclose(fp);
                return status;
            }
        }

        if ((strncmp(param_name, ADF_CFG_LIMIT_DEV_ACCESS,
             strlen(ADF_CFG_LIMIT_DEV_ACCESS)) == 0) && simplifiedConfig)
        {
            if (atoi(param_value) == 1)
            {
                if (adf_rename_section(section_name,
                                       dev_status->accelId,
                                       num_processes))
                {
                    status = CPA_STATUS_FAIL;
                    fclose(fp);
                    return status;
                }
            }
            continue;
        }

        if (simplifiedConfig == CPA_TRUE)
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

            if (status != CPA_STATUS_SUCCESS)
            {
                ADF_ERROR("Failed to processs instance parameter\n");
                fclose(fp);
                return status;
            }
        }

        if (do_not_copy != CPA_TRUE)
        {
            status = adf_add_multiple_section_param(param_name,
                                                    param_value,
                                                    num_processes);
            if (CPA_STATUS_SUCCESS != status)
            {
                ADF_PRINT("%s:%d: Failed to add parameter '%s'\n",
                           filename, line_number, param_name);
                fclose(fp);
                return status;
            }
        }
        parsed_params++;
    }

    if (((current_subsection.numAcceleratorNumberSet > 0) ||
         (current_subsection.numCoreAffinitySet != 0)) &&
        (simplifiedConfig == CPA_TRUE))
    {
         status = adf_generate_config_params(&current_subsection,
                                             num_processes,
                                             pf_bundle_offset,
                                             &adf_config_default,
                                             dev_status);

        if (status != CPA_STATUS_SUCCESS)
        {
            fclose(fp);
            return status;
        }
    }

    fclose(fp);
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

    status = adf_set_admin_param(dev_status->numAccel);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("Error: failed to set the admin params\n");
        adf_delall_section(&ctl_data.config_section);
        return status;
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
    /* If parameter is not found in config file then default is CPA_FALSE */
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
    if (ioctl(fd, IOCTL_CONFIG_SYS_RESOURCE_PARAMETERS, &ctl_data) != 0)
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_ioctl_stop
 *
 * Sends a request to the kernel to stop a device.
 */
STATIC CpaStatus adf_ioctl_stop(int fd, int dev_id)
{
    adf_cfg_ctl_data_t ctl_data = {0};
    ctl_data.device_id = dev_id;
    int ret = 0;

    ret = ioctl(fd, IOCTL_STOP_ACCEL_DEV, &ctl_data);
    if ( ret != 0)
    {
        if( EBUSY == ret)
        {
            ADF_PRINT("Device is in use\n");
        }
        else
        {
            ADF_PRINT("Shutting down device failed\n");
        }
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
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
 * adf_ioctl_start
 *
 * Sends a request to the kernel to start a device.
 */
STATIC CpaStatus adf_ioctl_start( int fd, int dev_id )
{
    adf_cfg_ctl_data_t ctl_data = {0};

    ctl_data.device_id = dev_id;

    if (ioctl(fd, IOCTL_START_ACCEL_DEV, &ctl_data) != 0)
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
    if (ioctl(fd, IOCTL_GET_NUM_DEVICES, num_phys_dev) != 0)
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

STATIC CpaStatus
adf_ioctl_get_device_status(int fd, adf_dev_status_info_t *dev_status)
{
    if (ioctl(fd, IOCTL_STATUS_ACCEL_DEV, &dev_status[0]) != 0)
    {
        ADF_PRINT("Get device info failed\n");
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_ioctl_status
 *
 * Checks and prints status of all devices in the system.
 * Assumed that there is no more than 32 devices in the system.
 */
STATIC CpaStatus
adf_ioctl_status(int fd, int dev_id, adf_dev_status_info_t *dev_status)
{
    adf_dev_status_info_t *device = NULL;
    Cpa32U num_phys_dev = 0;
    Cpa32U index = 0;
    CpaStatus status = CPA_STATUS_SUCCESS;

    status = adf_ioctl_get_num_phys_devices(fd, (int*)&num_phys_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("Get number of devices failed\n");
        return CPA_STATUS_FAIL;

    }
    ADF_PRINT("There is %d acceleration device(s) in the system:\n",
            num_phys_dev);

    for(index = 0; index < num_phys_dev; index++){
        device = &dev_status[index];

        ADF_PRINT
           (" icp_dev%d - type=%s, inst_id=%d, node_id=%d,"
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
 * Extract the device id from the given device name.
 */
CpaStatus adf_get_devid(char *dev_name, char *dev_format, int *dev_id)
{
    char name[NAME_MAX] = {0};
    int  id = 0;

    if (!sscanf(dev_name, dev_format, &id))
    {
        ADF_PRINT("Error: Invalid format.\n");
        return CPA_STATUS_FAIL;
    }

    snprintf(name, NAME_MAX, dev_format, id);

    if (strncmp(dev_name, name, strlen(dev_name)))
    {
        ADF_PRINT("Error: Invalid device name.\n");
        return CPA_STATUS_FAIL;
    }

    *dev_id = id;
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_load_file
 *
 * This function reads the configuration file of a device and
 * sends a request to the kernel to load the configuration parameters
 * into a memory region.
 */
CpaStatus adf_load_file(int ioc_fd, adf_dev_status_info_t *dev_status)
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
    status = adf_init_bank_info(dev_status);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("Failed to init banks\n");
        return status;
    }
    status = adf_read_config_file(filename, dev_status);

    adf_free_bank_info(dev_status);

    if (CPA_STATUS_SUCCESS != status)
    {
        adf_delall_section(&ctl_data.config_section);
        return status;
    }

#ifdef _DEBUG_
    adf_list_section(ctl_data.config_section);
#endif
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
 * adf_load_config
 *
 * This function determines if the request needs to
 * load single or multiple configuration files.
 */
CpaStatus adf_load_config(int ioc_fd, int dev_id,
                            adf_dev_status_info_t *dev_status)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    int num_PhysDevices = 0;
    int id = 0;

    if (ADF_CFG_ALL_DEVICES == dev_id)
    {
        status = adf_ioctl_get_num_phys_devices(ioc_fd, &num_PhysDevices);
        if (CPA_STATUS_SUCCESS != status)
        {
            ADF_PRINT("Error: Failed to get number of devices.\n");
            return status;
        }
        if(!num_PhysDevices)
        {
            ADF_PRINT("WARNING: No acceleration devices were found. "
                      "Exiting.\n");
            return status;
        }
        for (id = 0; id < num_PhysDevices; id++)
        {
            status = adf_load_file(ioc_fd, &dev_status[id]);
            if (CPA_STATUS_SUCCESS != status)
            {
                return status;
            }
        }
    }
    else
    {
        status = adf_load_file(ioc_fd, &dev_status[dev_id]);
    }
    return status;
}

CpaStatus adf_process_command(int cmd, int dev_id)
{
    CpaStatus status = CPA_STATUS_FAIL;
    int ioc_fd = -1;
    adf_dev_status_info_t dev_status[ADF_MAX_DEVICES] = {{0}};

    ioc_fd = adf_open_ioctl_device(ADF_CFG_IOCTL_DEVICE);
    if (ioc_fd < 0)
    {
        ADF_PRINT("Error: Failed to open device %s\n",
                        ADF_CFG_IOCTL_DEVICE);
        return CPA_STATUS_FAIL;
    }

    status = adf_ioctl_get_device_status(ioc_fd, dev_status);
    if (CPA_STATUS_FAIL == status){
        adf_close_ioctl_device(ioc_fd);
        return status;
    }

    if (!adf_core_affinity_init(dev_status))
    {
        adf_core_affinity_clean();
        adf_close_ioctl_device(ioc_fd);
        return CPA_STATUS_FAIL;
    }

    if (IOCTL_START_ACCEL_DEV == cmd)
    {
        status = adf_load_config(ioc_fd, dev_id, dev_status);
    }
    else if (IOCTL_STOP_ACCEL_DEV == cmd)
    {
        status = adf_ioctl_stop(ioc_fd, dev_id);
    }
    else if (IOCTL_STATUS_ACCEL_DEV == cmd)
    {
        status = adf_ioctl_status(ioc_fd, dev_id, dev_status);
    }
    else if (IOCTL_RESET_ACCEL_DEV == cmd)
    {
        status = adf_ioctl_reset(ioc_fd, dev_id);
    }
    else
    {
        ADF_PRINT("Error: Invalid command id\n");
        status = CPA_STATUS_FAIL;
    }

    adf_close_ioctl_device(ioc_fd);

    if (IOCTL_START_ACCEL_DEV == cmd)
    {
        if(ADF_CFG_ALL_DEVICES != dev_id)
        {
            adf_config_core_affinity(dev_id, dev_status);
        }
        else
        {
            int i = 0;
            for(i = i; i < ADF_MAX_DEVICES; i++)
            {
                adf_config_core_affinity(i, dev_status);
            }
        }
    }
    adf_core_affinity_clean();
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
       status = adf_get_devid(argv[1], "icp_dev%d", &ioc_dev);
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

    default:
       adf_ctl_help(argv[0]);
       return CPA_STATUS_FAIL;
       break;
    }
    status = adf_process_command(ioc_cmd, ioc_dev);
    return status;
}
