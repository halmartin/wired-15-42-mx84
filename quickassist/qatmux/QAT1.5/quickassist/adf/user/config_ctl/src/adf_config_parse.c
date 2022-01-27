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
******************************************************************************/

/******************************************************************************
 * @file adf_config_parse.c
 *
 * @ingroup adf_cfg
 *
 * @description Implementation of the ADF control parse mechanism
 *
******************************************************************************/
#include "icp_accel_devices.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "adf_config.h"

#define ADDITIONAL_SECION_STR_LEN 13

char *icp_module_name = "ADF_CONFIG_CTL";

/*
 * adf_add_section
 *
 * Add a new section to the head of section list
 */
CpaStatus adf_add_section(adf_cfg_section_t **section_list, char *name)
{
    adf_cfg_section_t *section = NULL;

    ICP_CHECK_FOR_NULL_PARAM(name);
    section = (adf_cfg_section_t *)ICP_MALLOC_GEN(sizeof(adf_cfg_section_t));

    ADF_CFG_CHK_NULL_PARAM(section, CPA_STATUS_FAIL);

    memset(section, 0, sizeof(adf_cfg_section_t));
    if(strlen(name) >= ADF_CFG_MAX_SECTION_LEN_IN_BYTES)
    {
        ADF_PRINT("%s: section name too long\n", __FUNCTION__);
        ICP_FREE(section);
        return CPA_STATUS_FAIL;
    }
    strncpy(section->name, name, strlen(name));
    section->name[strlen(name)] = '\0';

    ICP_ADD_ELEMENT_TO_HEAD_OF_LIST(section, NULL, (*section_list));

    return CPA_STATUS_SUCCESS;
}

/*
 * adf_add_section_end
 *
 * Adds a new section with the given name at the end of the chained list.
 */
CpaStatus adf_add_section_end(adf_cfg_section_t **section_list,
                             char *name,
                             adf_cfg_section_t **section_added)
{
    adf_cfg_section_t *section = NULL;
    adf_cfg_section_t *current_section = NULL;

    ICP_CHECK_FOR_NULL_PARAM(section_list);
    ICP_CHECK_FOR_NULL_PARAM(*section_list);
    ICP_CHECK_FOR_NULL_PARAM(section_added);
    ICP_CHECK_FOR_NULL_PARAM(name);
    section = (adf_cfg_section_t *)ICP_MALLOC_GEN(sizeof(adf_cfg_section_t));

    current_section = *section_list;
    ADF_CFG_CHK_NULL_PARAM(section, CPA_STATUS_FAIL);

    memset(section, 0, sizeof(adf_cfg_section_t));
    if(strlen(name) >= ADF_CFG_MAX_SECTION_LEN_IN_BYTES)
    {
        ADF_PRINT("%s: section name too long\n", __FUNCTION__);
        ICP_FREE(section);
        return CPA_STATUS_FAIL;
    }
    strncpy(section->name, name, strlen(name));
    section->name[strlen(name)] = '\0';

    *section_added = section;

    while (current_section->pNext != NULL)
    {
        current_section = current_section->pNext;
    }

    current_section->pNext = section;
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_duplicate_section
 *
 * Duplicate the current section numProcesses times and updates the section
 * name. Also updates the wireless_firmware boolean if numProcesses > 0
 */
CpaStatus adf_duplicate_section(Cpa32U numProcesses,
                                CpaBoolean *wireless_firmware)
{
    Cpa32U i, iProcInst=0;
    CpaStatus status = CPA_STATUS_SUCCESS;
    char * name = ctl_data.config_section->name;
    char internal_name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
    adf_cfg_section_t *original_section = ctl_data.config_section;
    adf_cfg_key_val_t *pKvList = original_section->params;
    adf_cfg_key_val_t *pCurrentKV = NULL;
    static CpaBoolean wirelessEnabled = CPA_FALSE;
    static CpaBoolean nonWirelessEnabled = CPA_FALSE;

    ICP_CHECK_FOR_NULL_PARAM(wireless_firmware);
    /* 
       The sections are added to HEAD of list & will be in reverse order
       So the sections are named in reverse order to maintain the bank 
       and section mapping, ie. SSL_INT_0 -> Bank0

       The section name per process starts from 0
       There is already 1 section available - ie current section
       Hence the iProcInst is numProcesses-2
    */
    for (i = 1, iProcInst=numProcesses-2; i<numProcesses; i++, iProcInst--)
    {
        sprintf(internal_name, "%s"INTERNAL_USERSPACE_SEC_SUFF"%d", name,\
                iProcInst);

        status = adf_add_section(&(ctl_data.config_section),
                                internal_name);
        if (status != CPA_STATUS_SUCCESS)
        {
           return status;
        }
        pCurrentKV = pKvList;
        /* copy all the existing key/values from the original section */
        while (pCurrentKV != NULL)
        {
            adf_add_section_param(pCurrentKV->key,
                                  pCurrentKV->val);
            pCurrentKV = pCurrentKV->pNext;
        }
    }

    /* Rename the current section so we're not using up valuable resources */
    /* NOTE sprintf doesn't like when one passes the same buff as both */
    sprintf(internal_name, "%s"INTERNAL_USERSPACE_SEC_SUFF"%d", name, \
            ( (0 == numProcesses) ? numProcesses : (numProcesses-1) ));
    memcpy(name, internal_name, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);

    /*
     * If we're running wireless firmware, set NumberOfWirelessProcs
     * in the GENERAL section of the config data
     */
    if (strncmp(name, WIRELESS, strlen(WIRELESS)) == 0)
    {
        if (numProcesses > 0)
        {
            *wireless_firmware = CPA_TRUE;
            wirelessEnabled = CPA_TRUE;
        }

        /* add number of processes to the general section */
        adf_cfg_section_t *pGeneralSection = NULL;
        char key_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
        char val_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};

        status = adf_find_section(ctl_data.config_section,
                                  GENERAL_SEC,
                                  &pGeneralSection);

        if (status != CPA_STATUS_SUCCESS)
        {
            return status;
        }

        sprintf(key_str, NUMBER_OF_WIRELESS_PROCS);
        sprintf(val_str, "%d", numProcesses);

        status = adf_add_param(key_str,
                               val_str,
                               pGeneralSection);
    }
    else
    {
        if (numProcesses > 0)
        {
            nonWirelessEnabled = CPA_TRUE;
        }
        /* set number of processes to 0 in the general section */
        adf_cfg_section_t *pGeneralSection = NULL;
        char key_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
        char val_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};

        status = adf_find_section(ctl_data.config_section,
                                  GENERAL_SEC,
                                  &pGeneralSection);

        if (status != CPA_STATUS_SUCCESS)
        {
            return status;
        }

        sprintf(key_str, NUMBER_OF_WIRELESS_PROCS);
        sprintf(val_str, "%d", 0);

        status = adf_add_param(key_str,
                               val_str,
                               pGeneralSection);
    }
    if ((wirelessEnabled == CPA_TRUE) && (nonWirelessEnabled == CPA_TRUE))
    {
        ADF_PRINT("%s err: Parameter \"NumProcesses\" cannot be >0 for both "
                  "WIRELESS and non-WIRELESS sections.\n", icp_module_name);
        status = CPA_STATUS_FAIL;
    }
    return status;
}

/*
 * adf_set_firmware_name
 *
 * If Firmware_Uof is not set in config file, we need to set it based on
 * the device type, number of Wireless procs and SRIOV if enabled/supported.
 */
CpaStatus adf_set_firmware_name(device_type_t type,
                                CpaBoolean wireless_firmware,
                                CpaBoolean sriov_firmware)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_cfg_section_t *pGeneralSection = NULL;
    char key_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    char val_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};

    if ((CPA_TRUE==wireless_firmware) && (CPA_TRUE==sriov_firmware))
    {
        ADF_PRINT("%s: Cannot enable both SRIOV and Wireless firmware\n",
                                                            __FUNCTION__);
        status = CPA_STATUS_FAIL;
        return status;
    }

    sprintf(key_str, ADF_AE_FW_UOF_NAME_KEY);

    if (DEV_C2XXX == type)
    {
        if (CPA_TRUE==wireless_firmware)
        {
            sprintf(val_str, "%s", ADF_NAE_FW_NAME_WIRELESS);
        }
        else
        {
            sprintf(val_str, "%s", ADF_NAE_FW_NAME_NORMAL);
        }
    }
    else
    {
        if (CPA_TRUE==wireless_firmware)
        {
            sprintf(val_str, "%s", ADF_AE_FW_NAME_WIRELESS);
        }
        else if (CPA_TRUE==sriov_firmware)
        {
            sprintf(val_str, "%s", ADF_AE_FW_NAME_SRIOV);
        }
        else
        {
            sprintf(val_str, "%s", ADF_AE_FW_NAME_NORMAL);
        }
    }

    /* Find the general section */

    status = adf_find_section(ctl_data.config_section,
                              GENERAL_SEC,
                              &pGeneralSection);
    if (status != CPA_STATUS_SUCCESS)
    {
        ADF_PRINT("%s: Cannot find %s section in config file.\n",
                                           __FUNCTION__, GENERAL_SEC);
        return status;
    }

    /* Add Firmware_Uof to the General section */

    status = adf_add_param(key_str,
                           val_str,
                           pGeneralSection);
    if (status != CPA_STATUS_SUCCESS)
    {
        ADF_PRINT("%s: Cannot add %s param to config file.\n",
                                           __FUNCTION__, key_str);
    }
    return status;
}

/*
 * adf_find_section
 *
 * Finds a section with the given name and returns a pointer to it.
 */
CpaStatus adf_find_section(adf_cfg_section_t *section_list,
                           char * name,
                           adf_cfg_section_t **out_section)

{
    while (section_list != NULL)
    {
        if (strncmp(section_list->name, name, strlen(name)) == 0)
        {
            *out_section = section_list;
            return CPA_STATUS_SUCCESS;
        }
        section_list = section_list->pNext;
    }
    return CPA_STATUS_FAIL;
}

/*
 * adf_add_keyvalue
 *
 * Adds the key, value and type variable to the adf_cfg_key_val_t
 * structure list.
 */
CpaStatus adf_add_keyvalue(adf_cfg_key_val_t **kv,
                           char *key, char *val, adf_cfg_val_type_t type)
{
    adf_cfg_key_val_t *tmp = NULL;

    ICP_CHECK_FOR_NULL_PARAM(key);
    ICP_CHECK_FOR_NULL_PARAM(val);
    tmp = (adf_cfg_key_val_t *)ICP_MALLOC_GEN(sizeof(adf_cfg_key_val_t));

    ADF_CFG_CHK_NULL_PARAM(tmp, CPA_STATUS_FAIL);

    memset(tmp, 0, sizeof(adf_cfg_key_val_t));
    if( (strlen(key) >= ADF_CFG_MAX_KEY_LEN_IN_BYTES) ||
        (strlen(val) >= ADF_CFG_MAX_VAL_LEN_IN_BYTES) )
    {
        ADF_PRINT("%s: key or value is too large\n", __FUNCTION__);
        ICP_FREE(tmp);
        return CPA_STATUS_FAIL;
    }
    strncpy(tmp->key, key, strlen(key));
    tmp->key[strlen(key)] = '\0';
    strncpy(tmp->val, val, strlen(val));
    tmp->val[strlen(val)] = '\0';
    tmp->type = type;

    ICP_ADD_ELEMENT_TO_HEAD_OF_LIST(tmp, NULL, (*kv));

    return CPA_STATUS_SUCCESS;
}


/*
 * adf_find_keyvalue
 *
 * Finds an entry in a list of  adf_cfg_key_val_t
 * structures.
 */

adf_cfg_key_val_t *adf_find_keyvalue(adf_cfg_key_val_t *kv, char *key)
{

    while (kv != NULL)
    {
        if(!strcmp(key, kv->key))
        {
            return kv;
        }
        kv = kv->pNext;
    }
    return NULL;
}

/*
 * adf_delete_keyvalue
 *
 * Finds an entry in a list of  adf_cfg_key_val_t
 * structures and removes it. This function is a different
 * to adf_del_keyvalue in that it finds an element
 * in the list and deletes it.
 */

void adf_delete_keyvalue(adf_cfg_key_val_t **kv, char *key)
{
    adf_cfg_key_val_t *current = NULL;
    adf_cfg_key_val_t *previous = NULL;

    current = *kv;

    while (current != NULL)
    {
        if(!strcmp(key, current->key))
        {
            if (previous)
            {
                /* point the previous element past current element */
                previous->pNext = current->pNext;
            }
            else
            {
                /* Move the start of the ist to the second element */
                *kv = (*kv)->pNext;
            }
            ICP_FREE(current);
            return;
        }
        previous = current;
        current = current->pNext;
    }
}

/*
 * adf_del_keyvalue
 *
 * Deletes an entry from the list of  adf_cfg_key_val_t
 * structures.
 */
void adf_del_keyvalue(adf_cfg_key_val_t **kv)
{
    adf_cfg_key_val_t *tmp = NULL;

    if (*kv != NULL)
    {
        tmp = *kv;
        *kv = (*kv)->pNext;
        ICP_FREE(tmp);
    }
}

/*
 * adf_delall_keyvalue
 *
 * Deletes the entire list of key/value pairs.
 */
void adf_delall_keyvalue(adf_cfg_key_val_t **kv)
{
    while (*kv != NULL)
    {
        adf_del_keyvalue(kv);
    }
}

/*
 * adf_del_section
 *
 * Deletes the current section and update the linked list.
 */
void adf_del_section(adf_cfg_section_t **section)
{
    adf_cfg_section_t *curr = NULL;

    if (*section != NULL)
    {
        curr = *section;
        adf_delall_keyvalue(&(curr->params));
        *section = (*section)->pNext;
        ICP_FREE(curr);
    }
}

/*
 * adf_delall_section
 *
 * Deletes all the sections
 */
void adf_delall_section(adf_cfg_section_t **section)
{
    while (*section != NULL)
    {
        adf_del_section(section);
    }
}

#ifdef _DEBUG_
/*
 * adf_list_keyvalues
 *
 * Prints out the list of key/value pairs.
 */
void adf_list_keyvalues(adf_cfg_key_val_t *kv)
{
    while (kv != NULL)
    {
        ADF_ERROR("key: %s, value: %s type: %d\n",
                kv->key, kv->val, kv->type);
        kv = kv->pNext;
    }
}

void adf_list_section(adf_cfg_section_t *section)
{
    while (section != NULL)
    {
        ADF_ERROR("section name = %s\n", section->name);
        adf_list_keyvalues(section->params);
        section = section->pNext;
    }
}
#endif

/*
 * adf_char_cmp
 *
 * Returns true if the character c is equal to cmp.
 */
CpaBoolean adf_char_cmp(int cmp, int c)
{
    if (c == ' ') {
        if (isspace(cmp))
        {
            return CPA_TRUE;
        }
    }

    if (c == '"' && cmp == c)
    {
        return CPA_TRUE;
    }

    return CPA_FALSE;
}

/*
 * adf_remove_char
 *
 * Function to remove the specified character c from the start and end
 * of the input string.
 */
char * adf_remove_char(char *str_ptr, int c)
{
    char *end_ptr = NULL;
    char *start_ptr = str_ptr;

    ADF_CFG_CHK_NULL_PARAM(str_ptr, NULL);

    /* Remove spaces from the start of the string */
    while (*start_ptr && adf_char_cmp((int)*start_ptr, c))
    {
        start_ptr++;
    }

    /* Remove spaces from the end of the string */
    end_ptr = (char *)( start_ptr + (strlen(start_ptr)-1));

    while (*end_ptr && adf_char_cmp((int)*end_ptr, c))
    {
        *end_ptr='\0';
        end_ptr--;
    }

    return start_ptr;
}

/*
 * adf_remove_white_spaces
 *
 * Function to remove the white space character from the start and end
 * of the input string.
 */
char * adf_remove_white_spaces(char * str_ptr)
{
    return adf_remove_char(str_ptr, ' ');
}

/*
 * adf_remove_double_quote
 *
 * Function to remove the double qoute(") character from the start and end
 * of the input string.
 */
char * adf_remove_double_quote(char * str_ptr)
{
    return adf_remove_char(str_ptr, '"');
}

/*
 * adf_parse_section_name
 *
 * Extract the section name from the input string
 */
CpaStatus adf_parse_section_name(char * section_ptr, char *section_name)
{
    int ctr = 0;
    char section_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    int len = 0;
    int section_end = 0;
    char *str_ptr = NULL;

    ICP_CHECK_FOR_NULL_PARAM(section_ptr);

    while (*section_ptr) {
        /*
         * Ensure that left square bracket only exists once within this string,
         * and should only exist in the beginning of the string
         */
        if (*section_ptr == '[') {
            if (ctr == 0) {
                section_ptr++;
                continue;
            }
            else {
                ADF_PRINT("%s: invalid section name\n", __FUNCTION__);
                return CPA_STATUS_FAIL;
            }
        }

        /* Ensure that we found the closing right square bracket */
        if (*section_ptr == ']')
        {
            section_end = 1;
            section_ptr++;
            break;
        }

        section_str[ctr++] = *section_ptr;
        section_ptr++;
    }

    if (section_end != 1) {
        ADF_PRINT("%s: invalid section line\n", __FUNCTION__);
        return CPA_STATUS_FAIL;
    }

    if (ctr == 0) {
        ADF_PRINT("%s: section name empty\n", __FUNCTION__);
        return CPA_STATUS_FAIL;
    }

    if (ctr > ADF_CFG_MAX_SECTION_LEN_IN_BYTES)
    {
        ADF_PRINT("%s: section name too long\n", __FUNCTION__);
        return CPA_STATUS_FAIL;
    }

    /* Ensure that the remaining texts are comment texts */
    len = strlen(section_ptr);
    if (len > 0) {
        str_ptr = adf_remove_white_spaces(section_ptr);
        if ((str_ptr != NULL) && ('#' != *str_ptr)) {
            ADF_PRINT("%s: invalid section line - extra characters detected\n",
                      __FUNCTION__);
            return CPA_STATUS_FAIL;
        }
    }

    str_ptr = adf_remove_white_spaces(section_str);
    if (str_ptr == NULL) {
        ADF_PRINT("%s: invalid section name\n", __FUNCTION__);
        return CPA_STATUS_FAIL;
    }

    len = strlen(str_ptr);
    memcpy(section_name, str_ptr, len);
    section_name[len] = '\0';
    adf_core_affinity_section_accelerator(section_name);
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_parse_parameter_pair
 *
 * Extract the parameter name and parameter value from the input string
 */
CpaStatus adf_parse_parameter_pair(int dev_id, char * parameter_pair_ptr,
                                   char * param_name, char * param_value)
{
    char key_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    char val_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    char *tmp_ptr = 0;
    int tmp_len = 0;
    int bank = 0;
    int core = 0;

    ICP_CHECK_FOR_NULL_PARAM(parameter_pair_ptr);

    if ( sscanf(parameter_pair_ptr,"%[^=] = %[^#\n]",
            key_str, val_str) != ADF_CFG_EXPECTED_NUM_ARGS)
    {
        return CPA_STATUS_FAIL;
    }

    tmp_len = strlen(key_str);
    if (tmp_len >= ADF_CFG_MAX_CONFIG_STRING_LENGTH)
    {
        ADF_PRINT("key_str exceeds maximum length.\n");
        return CPA_STATUS_FAIL;
    }
    key_str[tmp_len] = '\0';

    tmp_len = strlen(val_str);
    if (tmp_len >= ADF_CFG_MAX_CONFIG_STRING_LENGTH)
    {
        ADF_PRINT("val_str exceeds maximum length.\n");
        return CPA_STATUS_FAIL;
    }
    val_str[tmp_len] = '\0';

    tmp_ptr = adf_remove_white_spaces(key_str);

    ADF_CFG_CHK_NULL_PARAM(tmp_ptr, CPA_STATUS_FAIL);

    if (strlen(tmp_ptr) > ADF_CFG_MAX_KEY_LEN_IN_BYTES)
    {
        return CPA_STATUS_FAIL;
    }

    strncpy(param_name, tmp_ptr, ADF_CFG_MAX_KEY_LEN_IN_BYTES);
    param_name[ADF_CFG_MAX_KEY_LEN_IN_BYTES-1] = 0;

    tmp_ptr = adf_remove_white_spaces(val_str);

    ADF_CFG_CHK_NULL_PARAM(tmp_ptr, CPA_STATUS_FAIL);

    if (strlen(tmp_ptr) > ADF_CFG_MAX_VAL_LEN_IN_BYTES)
    {
        return CPA_STATUS_FAIL;
    }

    strncpy(param_value, tmp_ptr, ADF_CFG_MAX_KEY_LEN_IN_BYTES);

    if( adf_param_core_affinity( dev_id, param_name, &bank ) )
    {
        core = atoi(param_value);
        core = adf_set_core_affinity( dev_id, bank, core );
        if( core < 0 )
        {
            return CPA_STATUS_FAIL;
        }
        else if( core != atoi(param_value))
        {
            sprintf(param_value, "%d", core);
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_is_decimal
 *
 * Function which determines if the specified input string is a decimal value
 */
CpaBoolean adf_is_decimal(char *str_ptr)
{
    int index = 0;

    /* Check if each character is the string is a digit */
    for(index = 0; index < (strlen(str_ptr)); index++)
    {
        if (CPA_FALSE == (isdigit(str_ptr[index])))
        {
            return CPA_FALSE;
        }
    }

    return CPA_TRUE;
}

/*
 * adf_is_hexadecimal
 *
 * Function which determines if the specified input string is a hexadecimal
 * value
 */
CpaBoolean adf_is_hexadecimal(char *str_value_ptr)
{
    int index = 0;
    char *str_ptr = NULL;

    /*
     * If the string starts with a "0x" or a "0X" to signify that it's a
     * hexadecimal number, increment the str_ptr beyond this.
     */
    if ( (str_value_ptr[0] == '0') &&
            ((str_value_ptr[1] == 'x') || (str_value_ptr[1] == 'X')) )
    {
        str_ptr = str_value_ptr+ ADF_CFG_HEX_PREFIX_LENGTH;
    }
    else
    {
        str_ptr = str_value_ptr;
    }

    /*
     * Check that each character in the string is a valid hex character.
     * i.e. A to F or a to f or 0 to 9
     */
    for (index = 0; index < ((strlen(str_ptr))); index++)
    {
        if (CPA_FALSE == (
               ((str_ptr[index] >= 'a') && (str_ptr[index] <= 'f')) ||
               ((str_ptr[index] >= 'A') && (str_ptr[index] <= 'F')) ||
               ((str_ptr[index] >= '0') && (str_ptr[index] <= '9')) ) )
        {
            return CPA_FALSE;
        }
    }
    return CPA_TRUE;
}
/*
 * adf_is_instance
 *
 * This function checks whether the string represents a parameter that is part
 * of a section.
 */
CpaStatus adf_is_instance(char* str_value_ptr,
                    Cpa32U* pLen,
                    CpaBoolean *cy_instance,
                    CpaBoolean *dc_instance)
{
    char * local_str = NULL;
    ADF_CFG_CHK_NULL_PARAM(str_value_ptr, CPA_STATUS_INVALID_PARAM);
    ADF_CFG_CHK_NULL_PARAM(pLen, CPA_STATUS_INVALID_PARAM);
    ADF_CFG_CHK_NULL_PARAM(cy_instance, CPA_STATUS_INVALID_PARAM);
    ADF_CFG_CHK_NULL_PARAM(dc_instance, CPA_STATUS_INVALID_PARAM);

    local_str = adf_remove_white_spaces(str_value_ptr);
    Cpa32U len = 0;
    const Cpa8U name_len = strlen(CY);

    if ((strncmp(local_str, CY, name_len) == 0) ||
        (strncmp(local_str, DC, name_len) == 0))
    {
        len += name_len;
        while (isdigit(local_str[len]))
        {
            len++;
        }
        *pLen = len;
        if (strncmp(local_str, CY, name_len) == 0)
        {
            *cy_instance = CPA_TRUE;
            *dc_instance = CPA_FALSE;
        }
        else if (strncmp(local_str, DC, name_len) == 0)
        {
            *dc_instance = CPA_TRUE;
            *cy_instance = CPA_FALSE;
        }
    }
    else
    {
        *cy_instance = CPA_FALSE;
        *dc_instance = CPA_FALSE;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_add_section_param
 *
 * This function is used to add key-value parameter to the configuration
 * section of the ctl_data.
 */
CpaStatus adf_add_section_param(char *param_name, char *param_value)
{
    adf_cfg_val_type_t value_type = ADF_DEC;
    char *tmp_value = NULL;
    adf_cfg_key_val_t *kv = NULL;

    ICP_CHECK_FOR_NULL_PARAM(param_name);
    ICP_CHECK_FOR_NULL_PARAM(param_value);

    if (CPA_TRUE == adf_is_decimal(param_value))
    {
        value_type = ADF_DEC;
        tmp_value = param_value;
    }
    else if (CPA_TRUE == adf_is_hexadecimal(param_value))
    {
        value_type = ADF_HEX;
        tmp_value = param_value;
    }
    else
    {
        tmp_value = adf_remove_double_quote(param_value);
        value_type = ADF_STR;
    }

    /*
     * Check to see if this parameter already exists in this section
     * and if so, delete it before inserting.
     */
    kv = adf_find_keyvalue(ctl_data.config_section->params, param_name);
    if (kv)
    {
        adf_delete_keyvalue(&(ctl_data.config_section->params), param_name);
    }

    return adf_add_keyvalue(&(ctl_data.config_section->params),
                            param_name, tmp_value, value_type);
}

/*
 * adf_add_multiple_section_param
 *
 * This function is used to add key-value parameter to multiple configuration
 * sections of the ctl_data.
 */
CpaStatus adf_add_multiple_section_param(char *param_name,
                                         char *param_value,
                                         Cpa32U numProcesses)
{

    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_cfg_section_t *section = ctl_data.config_section;
    Cpa32U i=0;

    if ((numProcesses == 0) || (numProcesses == 1))
    {
        return adf_add_section_param(param_name,
                                     param_value);
    }

    for (i=0; i< numProcesses; i++)
    {
        status = adf_add_param(param_name,
                               param_value,
                               section);
        if (status != CPA_STATUS_SUCCESS)
        {
            return status;
        }
        section = section->pNext;
    }
    return status;
}

/*
 * adf_rename_section
 *
 * This function renames section name to provide per device configuration
 * for userspace processes where the "LimitDevAccess" param is set.
 */
CpaStatus adf_rename_section(char *section_name,
                             Cpa32U dev_id,
                             Cpa32U numProcesses)
{

    adf_cfg_section_t *section = ctl_data.config_section;
    char name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
    Cpa32U i = 0;

    if ((strlen(section_name) + ADDITIONAL_SECION_STR_LEN)
                                     >= ADF_CFG_MAX_SECTION_LEN_IN_BYTES)
    {
        ADF_ERROR("Section name %s is too long.\n", section_name);
        return CPA_STATUS_FAIL;
    }

    for (i = 0; i < numProcesses; i++)
    {
        sprintf(name, "%s_DEV%d_INT_%d", section_name, dev_id, i);
        strncpy(section->name, name, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
        section = section->pNext;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_add_section_param
 *
 * This function is used to add key-value parameter to the configuration
 * section of the ctl_data.
 */
CpaStatus adf_add_param(char *param_name,
                        char *param_value,
                        adf_cfg_section_t *section)
{
    adf_cfg_val_type_t value_type = ADF_DEC;
    char *tmp_value = NULL;
    adf_cfg_key_val_t *kv = NULL;

    ICP_CHECK_FOR_NULL_PARAM(param_name);
    ICP_CHECK_FOR_NULL_PARAM(param_value);

    if (CPA_TRUE == adf_is_decimal(param_value))
    {
        value_type = ADF_DEC;
        tmp_value = param_value;
    }
    else if (CPA_TRUE == adf_is_hexadecimal(param_value))
    {
        value_type = ADF_HEX;
        tmp_value = param_value;
    }
    else
    {
        tmp_value = adf_remove_double_quote(param_value);
        value_type = ADF_STR;
    }

    /*
     * Check to see if this parameter already exists in this section
     * and if so, delete it before inserting.
     */
    kv = adf_find_keyvalue(section->params, param_name);
    if (kv)
    {
        adf_delete_keyvalue(&(section->params), param_name);
    }

    return adf_add_keyvalue(&(section->params),
                            param_name, tmp_value, value_type);
}

/*
 * adf_get_range
 *
 * This function is used to get the list of parameters contained in the string
 * and fill the array with them.
 */
CpaStatus adf_get_range(Cpa8U range_array[],
                        Cpa8U* num_ranges,
                        char*   string)

{
    char * substr = NULL;
    ADF_CFG_CHK_NULL_PARAM(range_array, CPA_STATUS_FAIL);
    ADF_CFG_CHK_NULL_PARAM(num_ranges, CPA_STATUS_FAIL);
    ADF_CFG_CHK_NULL_PARAM(string, CPA_STATUS_FAIL);

    substr = strtok (string,",");
    int i=0;

    while (substr != NULL && i<MAX_RANGE_SIZE)
    {
        range_array[i] = atoi(substr);
        substr = strtok (NULL, ",");
        i++;
    }
    *num_ranges = i;
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_config_set_report_parity_error
 *
 * Configure set_report_parity_error as determined from config file.
 */
CpaStatus adf_config_set_report_parity_error(CpaBoolean report_parity)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_cfg_section_t *pGeneralSection = NULL;
    char key_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    char val_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};

    sprintf(key_str, ICP_CFG_REPORT_DC_PARITY_ERROR_KEY);

    if (CPA_TRUE == report_parity)
    {
        sprintf(val_str, "%s", "1");
    }
    else
    {
        sprintf(val_str, "%s", "0");

        /* If parity report is disabled then a warning must be issued. */
        ADF_PRINT("Parity err reporting is disabled.\n");
    }

    /* Find the general section */

    status = adf_find_section(ctl_data.config_section,
                              GENERAL_SEC,
                              &pGeneralSection);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("%s: Cannot find %s section in config file.\n",
                                           __FUNCTION__, GENERAL_SEC);
        return status;
    }

    /* Add report_parity key and value to the General section */
    status = adf_add_param(key_str,
                           val_str,
                           pGeneralSection);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("%s: Cannot add %s param to config file.\n",
                                           __FUNCTION__, key_str);
    }
    return status;
}

/*
 * adf_config_get_report_parity_error
 *
 * This function is used to get the value of report_parity.
 */
CpaStatus adf_config_get_report_parity_error(CpaBoolean* report_parity)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    char key_str[ADF_CFG_MAX_CONFIG_STRING_LENGTH] = {0};
    adf_cfg_key_val_t* kv = NULL;
    adf_cfg_section_t *pGeneralSection = NULL;

    ADF_CFG_CHK_NULL_PARAM(report_parity, CPA_STATUS_FAIL);

    /* Read the report parity value. */
    sprintf(key_str, ICP_CFG_REPORT_DC_PARITY_ERROR_KEY);
   
    /* Find the general section */
    status = adf_find_section(ctl_data.config_section,
                              GENERAL_SEC,
                              &pGeneralSection);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_PRINT("%s: Cannot find %s section in config file.\n",
                                           __FUNCTION__, GENERAL_SEC);
        return status;
    }

    kv = adf_find_keyvalue(pGeneralSection->params, key_str);

    if (kv != NULL)
    {
        *report_parity = (CpaBoolean)strtoul(kv->val, NULL, 10);
    }
    else /* Default value */
    {
        *report_parity = 1;
    }
    return CPA_STATUS_SUCCESS;
}

