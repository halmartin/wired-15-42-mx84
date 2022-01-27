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
 * @file adf_user_cfg.c
 *
 * @description
 *      This file contains Acceleration Driver Framework user proxy
 *      for the configuration control of Acceleration Subsystem.
 *
 *****************************************************************************/

#include <stdlib.h>
#include <unistd.h>

#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_adf_cfg.h"
#include "adf_cfg.h"
#include "adf_config_ctl.h"
#include "adf_cfg_types.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_config.h"
#include "icp_adf_accel_mgr.h"

/*
 * Open kernel driver interface
 */
STATIC int open_dev()
{
    int file_desc = -1;

    file_desc = open(ADF_CFG_IOCTL_DEVICE, O_RDWR);
    if (file_desc < 0)
    {
        ADF_ERROR("Error: Failed to open device %s\n",
                        ADF_CFG_IOCTL_DEVICE);
    }
    return file_desc;
}

/*
 * Close kernel driver interface
 */
STATIC void close_dev(int fd)
{
    close(fd);
}

/*
 * Send a command to kernel driver
 */
STATIC int send_ioctl(int fd, adf_cfg_ctl_data_t* cfg)
{
    return ioctl(fd, IOCTL_GET_VALUE_ACCEL_DEV, cfg);
}

/*
 * adf_dp_cfgGetParamValue
 * This function is used to determine the values configured for a given
 * set of parameter names in data plain
 */
STATIC CpaStatus adf_dp_cfgGetParamValue(void *ptr,
                                       int numElements,
                                       const char * pSectionName[],
                                       const char * pParamNames[],
                                       char * pParamValues[])
{
    adf_dp_config_header_t *header = NULL;
    adf_dp_config_entry_t *entry = NULL, *entry_start = NULL;
    char *sec_name = NULL;
    Cpa32U i = 0, x = 0, y = 0, number_of_values = 0,
           values_found = 0, values_not_found = 0;

    ICP_CHECK_FOR_NULL_PARAM(ptr);
    ICP_CHECK_FOR_NULL_PARAM(pSectionName);
    ICP_CHECK_FOR_NULL_PARAM(pParamNames);
    ICP_CHECK_FOR_NULL_PARAM(pParamValues);

    header = (adf_dp_config_header_t*)ptr;

    if(ADF_CFG_HEADER_VERIFIER != header->valid)
    {
        ADF_ERROR("Configuration memory is not valid.\n");
        return CPA_STATUS_FAIL;
    }

    entry_start = (adf_dp_config_entry_t*)((unsigned char *)ptr +
        sizeof(adf_dp_config_header_t));

    for(y = 0; (y < numElements); y++)
    {
        entry = entry_start;
        for(i = 0; (i < header->number_of_sections) &&
        (values_not_found + values_found < numElements); i++)
        {
            sec_name = (char *)entry->ADF_CFG_SECTION.name;

            if(0 == (ICP_STRNCMP(sec_name, pSectionName[y], strlen(sec_name))))
            {
                number_of_values = entry->ADF_CFG_SECTION.number_of_values;
                entry++;

                for(x = 0; (x < number_of_values) &&
                (values_not_found + values_found < numElements); x++)
                {

                    if(0 == (ICP_STRNCMP((char *)entry->ADF_CFG_VALUE.name,
                                pParamNames[y], 
                                strlen((char *)entry->ADF_CFG_VALUE.name))))
                    {
                        strncpy(pParamValues[values_not_found+values_found++],
                            (char *)entry->ADF_CFG_VALUE.value,
                            ADF_CFG_MAX_VAL_LEN_IN_BYTES);
                    }
                    entry++;
                }
            }
            else
            {
                entry += entry->ADF_CFG_SECTION.number_of_values + 1;
            }
        }
        if(y != (values_not_found + values_found) - 1)
        {
            values_not_found++;
            ADF_ERROR("Parameter %s in section %s not configured\n",
                    pParamNames[y], pSectionName[y]);
        }
    }
    if(numElements == values_found)
    {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_FAIL;
}

/*
 * icp_adf_cfgGetParamValue
 * This function is used to determine the value configured for the
 * given parameter name.
 */
CpaStatus icp_adf_cfgGetParamValue(icp_accel_dev_t *accel_dev,
                                   const char * pSection,
                                   const char * pParamName,
                                   char * pParamValue)
{

    char* name[1];
    char* value[1];
    char* section_name[1];

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pParamName);
    ICP_CHECK_FOR_NULL_PARAM(pParamValue);

    name[0] = (char*) pParamName;
    value[0] = pParamValue;
    section_name[0] = (char*) pSection;
    return icp_adf_cfgGetParamValueList(accel_dev,
                                        1,
                                        (const char**)section_name,
                                        (const char**)name,
                                        value);
}

/*
 * icp_adf_cfgGetParamValueList
 * This function is used to determine the values configured for a given
 * set of parameter names
 */
CpaStatus icp_adf_cfgGetParamValueList(icp_accel_dev_t *accel_dev,
                                       int numElements,
                                       const char * pSectionName[],
                                       const char * pParamNames[],
                                       char * pParamValues[])
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_cfg_ctl_data_t config = {0};
    adf_cfg_key_val_t *kval = NULL, *khead = NULL;
    adf_cfg_section_t *section = NULL;
    adf_cfg_section_t *current_section = NULL;
    adf_cfg_section_t *head_section = NULL;

    int fd = -1;
    int i = 0;
    int res = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pParamNames);
    ICP_CHECK_FOR_NULL_PARAM(pParamValues);

    /* If we are running in data plain and the config data
     * is exported then read all from the shared memory */
    if(accel_dev->shmem)
    {
        return adf_dp_cfgGetParamValue(accel_dev->shmem,
                                       numElements,
                                       pSectionName,
                                       pParamNames,
                                       pParamValues);
    }

    /* else do ioctl to get the data */
    fd = open_dev();
    if (fd < 0)
    {
        return CPA_STATUS_FAIL;
    }

    /* build request for ioctl */
    for(i = 0; i < numElements; i++)
    {
        config.device_id = accel_dev->accelId;

        /* If config section head is NULL allocate memory for the section
         * and add it to the config list */
        if(NULL == config.config_section)
        {
            section = ICP_MALLOC_GEN(sizeof(adf_cfg_section_t));
            if(NULL == section)
            {
                ADF_ERROR(" Failed to allocate memory for section.\n");
                close(fd);
                return CPA_STATUS_FAIL;
            }
            memset(section, '\0', sizeof(adf_cfg_section_t));
            if(strlen(pSectionName[i]) >= ADF_CFG_MAX_SECTION_LEN_IN_BYTES)
            {
                ADF_ERROR("Section name is too long.\n");
                ICP_FREE(section);
                close(fd);
                return CPA_STATUS_FAIL;
            }
            strncpy(section->name, pSectionName[i],
                                   ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
            ICP_ADD_ELEMENT_TO_END_OF_LIST(section, current_section,
                                            config.config_section);
        }
        /* If there is some section already there we have
         * to check if we do already have the section */
        else
        {
            section = config.config_section;
            while(section)
            {
                if(strcmp(section->name, pSectionName[i]) == 0)
                {
                    /* We have the section already.
                     * Section ptr points to it */
                    break;
                }
                section = section->pNext;
            }
            if(NULL == section)
            {
                /* New section - allocate memory for it
                 * and add it to the config list */
                section = ICP_MALLOC_GEN(sizeof(adf_cfg_section_t));
                if(NULL == section)
                {
                    ADF_ERROR(" Failed to allocate memory for section.\n");
                    close(fd);
                    head_section = config.config_section;
                    while(head_section)
                    {
                        section = head_section;
                        head_section = head_section->pNext;
                        ICP_FREE_LIST(section->params);
                        ICP_FREE(section);
                    }
                    return CPA_STATUS_FAIL;
                }
                memset(section, '\0', sizeof(adf_cfg_section_t));
                strncpy(section->name, pSectionName[i],
                                       ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
                ICP_ADD_ELEMENT_TO_END_OF_LIST(section, current_section,
                                                config.config_section);
            }
        }

        /*Allocate memory for the param value */
        kval = ICP_MALLOC_GEN(sizeof(adf_cfg_key_val_t));
        if(NULL == kval)
        {
            ADF_ERROR("Failed to allocate memory for kval.\n");
            close(fd);
            head_section = config.config_section;
            while(head_section)
            {
                section = head_section;
                head_section = head_section->pNext;
                ICP_FREE_LIST(section->params);
                ICP_FREE(section);
            }
            return CPA_STATUS_FAIL;
        }

        /*Put the key into the config structure. */
        memset(kval, '\0', sizeof(adf_cfg_key_val_t));
        if(pParamNames[i])
        {
            memcpy(kval->key, pParamNames[i], ICP_STRLEN(pParamNames[i]));
        }
        kval->user_data_ptr =  (int8_t *)pParamValues[i];

        /* Add the parameter to the list of the section.*/
        ICP_ADD_ELEMENT_TO_HEAD_OF_LIST(kval, NULL, section->params);
    }

    /* send the request down to get the configuration
     * information from kernel space. */
    res = send_ioctl(fd, &config);
    if(res < 0)
    {
        status = CPA_STATUS_FAIL;
    }

    /* copy results and free resources */
    head_section = config.config_section;
    while(head_section)
    {
        khead = head_section->params;

        while(khead)
        {
            kval = khead;
            /*if the val is empty then there was not such a key
             * in the config file - so return a fail */
            if(strlen(kval->val))
            {
                strncpy((char *)kval->user_data_ptr, (char *)kval->val,
                                    ADF_CFG_MAX_VAL_LEN_IN_BYTES);
            }
            else
            {
                kval->user_data_ptr[0] = '\0';
                status = CPA_STATUS_FAIL;
            }
            khead = khead->pNext;
            ICP_FREE(kval);
        }
        section = head_section;
        head_section = head_section->pNext;
        ICP_FREE(section);
    }
    close_dev(fd);
    return status;
}

/*
 * icp_qa_dev_get
 * Function increments the device usage counter.
 */
void icp_qa_dev_get(icp_accel_dev_t *pAccelDev)
{
    int fd = open_dev();
    if(fd < 0)
    {
            ADF_ERROR("Can not open device\n");
            return;
    }
    else
    {
        if(ioctl(fd, IOCTL_DEV_GET,  &pAccelDev->accelId) < 0)
        {
                ADF_ERROR("Device get failed for icp_dev%d\n",
                                            pAccelDev->accelId);
        }
    }
    close_dev(fd);
}

/*
 * icp_qa_dev_get
 * Function decrements the device usage counter.
 */
void icp_qa_dev_put(icp_accel_dev_t *pAccelDev)
{
    int fd = open_dev();
    if(fd < 0)
    {
            ADF_ERROR("Can not open device\n");
            return;
    }
    else
    {
        if(ioctl(fd, IOCTL_DEV_PUT, &pAccelDev->accelId) < 0)
        {
                ADF_ERROR("Device put failed for icp_dev%d\n",
                                            pAccelDev->accelId);
        }
    }
    close_dev(fd);
}

/*
 * icp_adf_cfgSetSharedMem
 *
 * Function sets the pointer to shared memory containing configuration data.
 * The function is the call in the DP to set the shared memory pointer
 * It is assumed that the export has been called at this stage.
 */
CpaStatus icp_adf_cfgSetSharedMemPtr(icp_accel_dev_t *accel_dev,
                                                       void *ptr)
{
    adf_dp_config_header_t *hd = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(ptr);

    hd = (adf_dp_config_header_t*)ptr;
    if(ADF_CFG_HEADER_VERIFIER == hd->valid)
    {
        accel_dev->shmem = ptr;
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_FAIL;
}
