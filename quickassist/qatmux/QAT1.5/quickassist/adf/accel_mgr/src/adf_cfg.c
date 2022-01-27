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
 * @file adf_cfg.c
 *
 * @description
 *      This file contains OS specific Acceleration Driver Framework code for
 *      run-time configuration control of Acceleration Subsystem.
 *
 *****************************************************************************/

/*
 * Three tables exist, a config table which contains information read in from
 * the configuration file, an internal_info table which contains internal
 * data read from the bios e.g. SRAM info, and a symbols table which stores
 * the symbols used to patch the firmware.
 *
 *
 * The ADF Tables
 * What components modify the tables and when?
 *      1) ADF itself, when it detected a device, it creates the table.
 *      2) On linux/freebsd, the adf_ctl program at runtime, when it updates
 *             the table prior to telling ADF to init it's subsystem
 *
 * What components read the table and when?
 *      By subcomponent modules when their init functions are called
 */

#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "icp_adf_cfg.h"
#include "adf_cfg.h"

/*
 * System Parameter Table Data
 */
STATIC adf_cfg_device_data_t *pAdfCfgTablePtr = NULL;
STATIC adf_cfg_device_data_t *pAdfCfgTablePtrHead = NULL;

/*
 * Lock for the adf config table to ensure that there will be no concurrent
 * accesses to the table.
 */
STATIC ICP_MUTEX adf_config_table_lock;

/*
 * adf_cfgInit
 * Initialize configuration module internal settings
 */
CpaStatus adf_cfgInit(void)
{
    /* Create a mutex lock to be used to guard the adf configuration table */
    ICP_MUTEX_INIT(&adf_config_table_lock);
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_cfgUninit
 * Uninitialize configuration module internal settings
 */
void adf_cfgUninit(void)
{
    /* Delete configuration table lock */
    ICP_MUTEX_UNINIT(&adf_config_table_lock);
    return;
}

/*
 * adf_cfgDeviceAdd
 * Add the device to the config table
 */
CpaStatus adf_cfgDeviceAdd(Cpa32S device_id)
{
    adf_cfg_device_data_t *pCfgDevData = NULL;

    if (CPA_STATUS_SUCCESS == adf_cfgDeviceFind(device_id, &pCfgDevData))
    {
        ADF_ERROR("Device %d configuration table already exists.\n",
                        device_id);
        return CPA_STATUS_FAIL;
    }

    pCfgDevData = ICP_ZALLOC_GEN(sizeof(adf_cfg_device_data_t));
    if (NULL == pCfgDevData)
    {
        ADF_ERROR("Unable to allocate memory for config data\n");
        return CPA_STATUS_FAIL;
    }

    pCfgDevData->device_id = device_id;

    /* Allocate internal section */
    pCfgDevData->internal_section = ICP_ZALLOC_GEN(sizeof(adf_cfg_section_t));
    if (NULL == pCfgDevData->internal_section)
    {
        ADF_ERROR("Unable to allocate memory for internal section\n");
        ICP_FREE(pCfgDevData);
        return CPA_STATUS_FAIL;
    }

    ICP_STRNCPY(pCfgDevData->internal_section->name,
                INTERNAL_SEC, ICP_STRLEN(INTERNAL_SEC));
    if (ICP_MUTEX_LOCK(&adf_config_table_lock))
    {
        ADF_ERROR("Unable to aquire table lock\n");
        ICP_FREE(pCfgDevData->internal_section);
        ICP_FREE(pCfgDevData);
        return CPA_STATUS_FAIL;
    }

    /* Add a device to the head of the linked list. */
    ICP_ADD_ELEMENT_TO_HEAD_OF_LIST(pCfgDevData,
                                    pAdfCfgTablePtr, pAdfCfgTablePtrHead);
    ICP_MUTEX_UNLOCK(&adf_config_table_lock);
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_cfgDeviceFind
 * Find configuration data of a given device.
 */
CpaStatus adf_cfgDeviceFind(Cpa32S device_id, adf_cfg_device_data_t** pCfgData)
{
    adf_cfg_device_data_t *ptr = pAdfCfgTablePtrHead;

    while (ptr != NULL)
    {
        if (ptr->device_id == device_id)
        {
            *pCfgData = ptr;
            return CPA_STATUS_SUCCESS;
        }
        ptr = ptr->pNext;
    }
    return CPA_STATUS_FAIL;
}

/*
 * adf_cfgDeviceRemove
 * Remove the configuration data of the device from the device
 * structure. Remove the config, internal and symbols tables.
 */
CpaStatus adf_cfgDeviceRemove(Cpa32S device_id)
{
    adf_cfg_device_data_t *pCfgDevData = NULL;

    if (CPA_STATUS_SUCCESS != adf_cfgDeviceFind(device_id, &pCfgDevData))
    {
        return CPA_STATUS_FAIL;
    }

    /* Delete each section, the key values will be
     * deleted also. */
    adf_cfgSectionDel(&(pCfgDevData->config_section));
    adf_cfgSectionDel(&(pCfgDevData->internal_section));
    adf_cfgSectionDel(&(pCfgDevData->symbol_section));

    if(ICP_MUTEX_LOCK(&adf_config_table_lock))
    {
        ADF_ERROR("Unable to aquire table lock\n");
        ICP_FREE(pCfgDevData);
        return CPA_STATUS_FAIL;
    }

    ICP_REMOVE_ELEMENT_FROM_LIST(pCfgDevData,
                                 pAdfCfgTablePtr, pAdfCfgTablePtrHead);
    ICP_MUTEX_UNLOCK(&adf_config_table_lock);
    ICP_FREE(pCfgDevData);
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_cfgKeyValueAddNode
 * Add a node of key-value parameter to the key-value table
 * It is assumed that the key-value node buffer has been allocated.
 */
CpaStatus adf_cfgKeyValueAddNode(adf_cfg_key_val_t **pKeyValueList,
                                 adf_cfg_key_val_t *pKeyValueNode)
{
    ICP_CHECK_FOR_NULL_PARAM(pKeyValueNode);
    ICP_ADD_ELEMENT_TO_HEAD_OF_LIST(pKeyValueNode, NULL, (*pKeyValueList));
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_cfgKeyValueDelete
 * Delete a key-value item from the key-value table
 * This function is called by adf_cfgDelKeyValueParam
 * to delete a key-value pair from a section.
 */
STATIC void adf_cfgKeyValueDeleteNode(adf_cfg_key_val_t **pKeyValueList,
                           adf_cfg_key_val_t *pKeyValue)
{
    if ( (*pKeyValueList == NULL) || (pKeyValue == NULL) )
    {
        return;
    }
    ICP_REMOVE_ELEMENT_FROM_LIST(pKeyValue, pKeyValue->pNext, *pKeyValueList);
}

/*
 * adf_cfgSectionAddNode
 * Add a section node to the configuration list
 * It is assumed that the section node buffer has been allocated.
 */
CpaStatus adf_cfgSectionAddNode(adf_cfg_section_t **pSectionList,
                                 adf_cfg_section_t *pSectionNode)
{
    ICP_CHECK_FOR_NULL_PARAM(pSectionNode);
    /*Add pSectionNode to the section list (pSectionList). */

    ICP_ADD_ELEMENT_TO_HEAD_OF_LIST(pSectionNode, NULL, (*pSectionList));
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_cfgKeyValueDelAll
 * Delete all key-value parameters from the List.
 * Called by the adf_cfgSectionDel function.
 */
STATIC void adf_cfgKeyValueDelAll(adf_cfg_key_val_t **pKeyValueList)
{
    adf_cfg_key_val_t *pCurr = NULL;

    while (*pKeyValueList != NULL)
    {
        pCurr = *pKeyValueList;
        *pKeyValueList = (*pKeyValueList)->pNext;
        ICP_FREE(pCurr);
    }
}

/*
 * adf_cfgSectionDel
 * Delete all section nodes and associated key-value
 * pairs from the section list.
 */
void adf_cfgSectionDel(adf_cfg_section_t **pSectionList)
{
    adf_cfg_section_t *pCurr = NULL;

    while (*pSectionList != NULL)
    {
        pCurr = *pSectionList;
        *pSectionList = (*pSectionList)->pNext;
        /*Delete the associated key value list..*/
        adf_cfgKeyValueDelAll(&(pCurr->params));
        ICP_FREE(pCurr);
    }
}

/*
 * adf_cfgKeyValueList
 * List all key-value parameters from the key-value table
 */
void adf_cfgKeyValueList(adf_cfg_key_val_t *pKeyValueList)
{
    while (pKeyValueList != NULL)
    {
        ADF_PRINT("list::key: %s, value: %s type: %d\n",
            pKeyValueList->key, pKeyValueList->val, pKeyValueList->type);
        pKeyValueList = pKeyValueList->pNext;
    }
}

/*
 * adf_cfgSectionList
 * List all sections and associated key-value pairs
 */
void adf_cfgSectionList(adf_cfg_section_t *pSectionList)
{
    adf_cfg_key_val_t *pKeyValueList;

    while(pSectionList != NULL)
    {
        pKeyValueList = pSectionList->params;
        ADF_PRINT("\nSection:%s\n", pSectionList->name);
        adf_cfgKeyValueList(pKeyValueList);
        pSectionList = pSectionList->pNext;
    }
}

/*
 * adf_cfgKeyValueFind
 * Find a key-value parameter based on the given key. This returns
 * the pointer to the key-value pair.
 */
STATIC adf_cfg_key_val_t *adf_cfgKeyValueFind(adf_cfg_key_val_t *pKeyValueList,
                                      const char *pKey)
{
    if (NULL == pKey || ICP_STRLEN(pKey) == 0)
    {
        return NULL;
    }
    while (pKeyValueList != NULL)
    {
        if (0 == ICP_STRCMP(pKeyValueList->key, pKey))
        {
            ADF_DEBUG("found: %s, value: %s type: %d\n",
                 pKeyValueList->key, pKeyValueList->val, pKeyValueList->type);
            return pKeyValueList;
        }
        pKeyValueList = pKeyValueList->pNext;
    }
    return NULL;
}

/*
 * adf_cfgSectionFind
 * Find a section pointer for a given section name.
 */
adf_cfg_section_t *adf_cfgSectionFind(adf_cfg_section_t *pSectionList,
                                      const char *pSectionName)
{
    if (NULL == pSectionName)
    {
        return NULL;
    }
    while (pSectionList != NULL)
    {
        if (0 == ICP_STRCMP(pSectionList->name, pSectionName))
        {
            return pSectionList;
        }
        pSectionList = pSectionList->pNext;
    }
    return NULL;
}

/*
 * adf_cfgKeyValueGet
 * Gets the pointer to the key-value parameter for a given section
 * table and key.
 */
STATIC CpaStatus adf_cfgKeyValueGet(icp_accel_dev_t *accel_dev,
                             const char *section,
                             const char *pParamName,
                             adf_cfg_key_val_t **pKeyValue)
{
    adf_cfg_device_data_t *pCfgData = NULL;
    icp_accel_dev_t *pAccelDev = NULL;
    adf_cfg_section_t *pKeyValSection=NULL;
    Cpa32U retval=0;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pParamName);

    pAccelDev = accel_dev;

    if (CPA_STATUS_SUCCESS != adf_cfgDeviceFind(pAccelDev->accelId, &pCfgData))
    {
        return CPA_STATUS_FAIL;
    }

    pKeyValSection = pCfgData->internal_section;

    /*First check to see if we look in the INTERNAL section. */
    if(0 == ICP_STRCMP(section, INTERNAL_SEC))
    {
        *pKeyValue = adf_cfgKeyValueFind(pKeyValSection->params, pParamName);
        if (NULL != *pKeyValue)
        {
            /* Key-value found */
            return CPA_STATUS_SUCCESS;
        }
    } /*Next check for the SYMBOLS section. */
    else if(0 == ICP_STRCMP(section, SYMBOLS_SEC))
    {
        *pKeyValue = adf_cfgKeyValueFind(pKeyValSection->params, pParamName);
        if (NULL != *pKeyValue)
        {
            /* Key-value found */
            return CPA_STATUS_SUCCESS;
        }
    }
    /*Otherwise loop over the other sections for a match. */
    else
    {
        pKeyValSection = pCfgData->config_section;
        while(pKeyValSection)
        {
            /* Find the correct section. */
            retval = ICP_STRCMP(section, pKeyValSection->name);

            if(0 == retval)
            {
                *pKeyValue = adf_cfgKeyValueFind(pKeyValSection->params,
                                                    pParamName);
                if (*pKeyValue)
                {
                    return CPA_STATUS_SUCCESS;
                }
            }
            pKeyValSection = pKeyValSection->pNext;

        }
    }
    ADF_DEBUG("Failed to find %s in section %s.\n", pParamName, section);
    return CPA_STATUS_FAIL;
}

/*
 * adf_cfgDelKeyValueParam
 * This function enables other internal components to delete a key-value
 * parameter from a SectionNode.
 */
CpaStatus adf_cfgDelKeyValueParam(icp_accel_dev_t *accel_dev,
                                  adf_cfg_section_t *pSectionNode,
                                  char *pKey)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_cfg_key_val_t *pKeyValue = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pKey);

    /* Acquire the adf_config_table_lock */
    if (ICP_MUTEX_LOCK(&adf_config_table_lock))
    {
        return CPA_STATUS_FAIL;
    }

    /*Find the Key Value pair in this section. */
    pKeyValue = adf_cfgKeyValueFind(pSectionNode->params, pKey);

    if (NULL == pKeyValue)
    {
        ICP_MUTEX_UNLOCK(&adf_config_table_lock);
        return CPA_STATUS_FAIL;
    }

    /* Delete the key-value data from the section. */
    adf_cfgKeyValueDeleteNode(&(pSectionNode->params), pKeyValue);
    ICP_FREE(pKeyValue);

    /* Release the adf_config_table_lock */
    ICP_MUTEX_UNLOCK(&adf_config_table_lock);
    return status;
}

/*
 * adf_cfgAddKeyValueParam
 * This function enables other internal components to add a key-value
 * parameter into a section list.
 */
CpaStatus adf_cfgAddKeyValueParam(icp_accel_dev_t *accel_dev,
                                  adf_cfg_section_t *pSectionList,
                                  char *pKey, void *val,
                                  adf_cfg_val_type_t type)
{
    CpaStatus status = CPA_STATUS_FAIL;

    adf_cfg_key_val_t *pKeyValue = NULL;
    Cpa64U *value_addr=NULL;
    Cpa64U value=0;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pKey);

    /* Acquire the adf_config_table_lock */
    if (ICP_MUTEX_LOCK(&adf_config_table_lock))
    {
        return CPA_STATUS_FAIL;
    }

    pKeyValue = ICP_ZALLOC_GEN(sizeof(adf_cfg_key_val_t));
    if (NULL == pKeyValue)
    {
        ADF_ERROR("Failed to allocate param_value buffer.\n");
        ICP_MUTEX_UNLOCK(&adf_config_table_lock);
        return CPA_STATUS_FAIL;
    }


    strncpy(pKeyValue->key, pKey, ADF_CFG_MAX_KEY_LEN_IN_BYTES);
    if(ADF_DEC == type)
    {
        value_addr = (Cpa64U *)val;
        value = *value_addr;
        snprintf(pKeyValue->val, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                 "%d", (int)value);
    }
    else if(ADF_STR == type)
    {
        strncpy(pKeyValue->val, (char *)val, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    }
    else if(ADF_HEX == type)
    {
        value_addr = (Cpa64U *)val;
        snprintf(pKeyValue->val, ADF_CFG_MAX_VAL_LEN_IN_BYTES,
                 "0x%p", value_addr);
    }
    else
    {
        ADF_ERROR("Unknown type given.\n");
        ICP_MUTEX_UNLOCK(&adf_config_table_lock);
        ICP_FREE(pKeyValue);
        return CPA_STATUS_FAIL;
    }
    pKeyValue->type = type;
    /* Add a KeyValue pair to a section list. */
    status = adf_cfgKeyValueAddNode(&pSectionList->params, pKeyValue);

    /* Release the adf_config_table_lock */
    ICP_MUTEX_UNLOCK(&adf_config_table_lock);
    return status;
}

/*
 * adf_cfgSectionAdd
 * Add section Node to the Section List.
 */
CpaStatus adf_cfgSectionAdd(icp_accel_dev_t *accel_dev,
                            adf_cfg_section_t **pSectionList,
                            adf_cfg_section_t *pSectionNode)
{
    adf_cfg_section_t *curr_sec=NULL;
    Cpa32U retval=0;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pSectionNode);

    curr_sec = *pSectionList;
    /*First to check to see if the section already exists. */
    while (curr_sec != NULL)
    {

        retval = ICP_STRCMP(curr_sec->name, pSectionNode->name);
        if(retval == 0)
        {
            ADF_PRINT("Section already exists.\n");
            /*Free this node since it already exists
             *there is no need to add it. */
            ICP_FREE(pSectionNode);
            return CPA_STATUS_SUCCESS;
        }
        curr_sec = curr_sec->pNext;
    }
    /*Section doesn't already exist - add it. */

    if (CPA_STATUS_SUCCESS !=
            adf_cfgSectionAddNode(&(*pSectionList), pSectionNode))
    {
        ADF_ERROR("Failed to add %s to sectionList.\n", pSectionNode->name);
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}
/*
 * adf_cfgGetRingNumber
 * Gets the ring number associated to a given service name
 */
CpaStatus adf_cfgGetRingNumber(icp_accel_dev_t *accel_dev,
                               const char *pSection,
                               const Cpa32U accel_num,
                               const Cpa32U bank_num,
                               const char *pServiceName,
                               Cpa32U * pRingNum)
{
    CpaStatus status = CPA_STATUS_FAIL;
    adf_cfg_key_val_t *pKeyValue = NULL;
    Cpa32U numBanksPerAccel = 0;
    Cpa32U numRingsPerBank  = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pServiceName);
    ICP_CHECK_FOR_NULL_PARAM(pSection);

    if (NULL == accel_dev->pHwDeviceData)
    {
        ADF_ERROR("pHwDeviceData is NULL\n");
        return CPA_STATUS_FAIL;
    }

    numBanksPerAccel = GET_NUM_BANKS_PER_ACCEL(accel_dev);
    numRingsPerBank  = GET_NUM_RINGS_PER_BANK(accel_dev);

    if(ICP_STRLEN(pSection) == 0)
    {
        ADF_ERROR("Section Name has zero length.\n");
        return CPA_STATUS_FAIL;
    }
    /* Acquire the adf_config_table_lock */
    if (ICP_MUTEX_LOCK(&adf_config_table_lock))
    {
        return CPA_STATUS_FAIL;
    }

    status = adf_cfgKeyValueGet(accel_dev,
                                pSection, pServiceName, &pKeyValue);
    if (CPA_STATUS_SUCCESS != status)
    {
        ICP_MUTEX_UNLOCK(&adf_config_table_lock);
        return CPA_STATUS_FAIL;
    }

    if (ADF_DEC != pKeyValue->type)
    {
        ICP_MUTEX_UNLOCK(&adf_config_table_lock);
        return CPA_STATUS_FAIL;
    }

    *pRingNum = (Cpa32U)ICP_STRTOUL(pKeyValue->val, NULL, ADF_CFG_BASE_DEC);

    *pRingNum += numRingsPerBank * (bank_num +
                 numBanksPerAccel * accel_num);
    ICP_MUTEX_UNLOCK(&adf_config_table_lock);
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_cfgGetParamValue
 * This function is used to determine the value configured for the
 * given parameter name.
 */
CpaStatus icp_adf_cfgGetParamValue(icp_accel_dev_t *accel_dev,
                                   const char *pSection,
                                   const char * pParamName,
                                   char * pParamValue)
{
    CpaStatus status = CPA_STATUS_FAIL;
    adf_cfg_key_val_t *pKeyValue = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pParamName);
    ICP_CHECK_FOR_NULL_PARAM(pSection);

    if(ICP_STRLEN(pSection) == 0 || ICP_STRLEN(pParamName) == 0)
    {
        ADF_ERROR("Section Name or Key has zero length.\n");
        return CPA_STATUS_FAIL;
    }

    /* Acquire the adf_config_table_lock */
    if (ICP_MUTEX_LOCK(&adf_config_table_lock))
    {
        return CPA_STATUS_FAIL;
    }

    status = adf_cfgKeyValueGet(accel_dev,
                                pSection, pParamName,&pKeyValue);
    if (CPA_STATUS_SUCCESS != status)
    {
        ICP_MUTEX_UNLOCK(&adf_config_table_lock);
        return status;
    }

    ICP_STRNCPY(pParamValue, pKeyValue->val, ADF_CFG_MAX_VAL_LEN_IN_BYTES);

    ICP_MUTEX_UNLOCK(&adf_config_table_lock);
    return CPA_STATUS_SUCCESS;

}

/*
 * icp_adf_cfgGetParamValueList
 * This function is used to determine the values configured for a given
 * set of parameter names
 */
CpaStatus icp_adf_cfgGetParamValueList(icp_accel_dev_t *accel_dev,
                                       Cpa32S numElements,
                                       const char * pSectionName[],
                                       const char * pParamNames[],
                                       char * pParamValues[])
{
    CpaStatus status = CPA_STATUS_FAIL;
    Cpa32S ctr = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pParamNames);
    ICP_CHECK_FOR_NULL_PARAM(pParamValues);
    ICP_CHECK_FOR_NULL_PARAM(pSectionName);

    for (ctr = 0; ctr < numElements; ctr++)
    {
        status = icp_adf_cfgGetParamValue(accel_dev, pSectionName[ctr],
                                          pParamNames[ctr],
                                          pParamValues[ctr]);
        if (CPA_STATUS_SUCCESS != status)
        {
            return status;
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_cfgGetRingNumber
 * Function returns ring number configures for the service
 * It needs to be exported to allow the access layer to build
 * ring poll mask
 */
CpaStatus icp_adf_cfgGetRingNumber(icp_accel_dev_t *accel_dev,
                                   const char *pSection,
                                   const Cpa32U accel_num,
                                   const Cpa32U bank_num,
                                   const char *pServiceName,
                                   Cpa32U* pRingNum)
{
    return adf_cfgGetRingNumber(accel_dev, pSection, accel_num,
                                bank_num, pServiceName, pRingNum);
}
