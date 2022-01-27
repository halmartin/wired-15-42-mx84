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
 * @file adf_devmgr.c
 *
 * @description
 * Acceleration Device Manager
 *
 ******************************************************************************/

/******************************************************************************
* Include public/global header files
*******************************************************************************/
#include "cpa.h"
#include "icp_platform.h"

/******************************************************************************
* Include private header files
*******************************************************************************/
#include "adf_devmgr.h"
#include "icp_adf_accel_mgr.h"
#include "adf_cfg.h"
#include "adf_config_ctl.h"
#include "adf_init.h"
#include "adf_drv.h"
/******************************************************************************
* Static Variables
*******************************************************************************/
/*
 * Accelerator Table: points to head of accelerator list
 * New accelerators are added to head of list (which is a doubly linked list)
 */
STATIC icp_accel_dev_t *pAccelTable = NULL;
STATIC icp_accel_dev_t *pAccelTableHead = NULL;
STATIC ICP_MUTEX       tableLock;

/*
 * adf_devmgrInit
 * Initialise that accel table
 */
CpaStatus adf_devmgrInit(void)
{
    ICP_MUTEX_INIT(&tableLock);
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_devmgrExit
 * Uninitialise that accel table
 */
void adf_devmgrExit(void)
{
    ICP_MUTEX_UNINIT(&tableLock);
    return;
}

/*
 * adf_devmgrFindAccelDev
 * Find a given accel_dev in the accelerator table
 */
STATIC CpaStatus adf_devmgrFindAccelDev(icp_accel_dev_t *pAccelDev)
{
    icp_accel_dev_t *pTmp_dev = pAccelTableHead;
    CpaStatus status = CPA_STATUS_FAIL;

    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    ICP_FIND_ELEMENT_IN_LIST(pAccelDev, pTmp_dev, status);
    if (status)
    {
        return CPA_STATUS_FAIL;
    }
    else
    {
        return CPA_STATUS_SUCCESS;
    }
}

/*
 * adf_devmgrAddAccelDev
 * Add a new accelerator to the Accelerator Table
 */
CpaStatus adf_devmgrAddAccelDev(icp_accel_dev_t *pAccelDev)
{
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    if(ICP_MUTEX_LOCK(&tableLock))
    {
        ADF_ERROR("Unable to aquire table lock\n");
        return CPA_STATUS_FAIL;
    }
    if (adf_devmgrFindAccelDev(pAccelDev) == CPA_STATUS_SUCCESS)
    {
        /* accel_dev already exists */
        ICP_MUTEX_UNLOCK(&tableLock);
        return CPA_STATUS_FAIL;
    }

    /*Add pAccelDev to the end of the list.*/
    ICP_ADD_ELEMENT_TO_END_OF_LIST(pAccelDev, pAccelTable, pAccelTableHead);
    ICP_MUTEX_UNLOCK(&tableLock);
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_devmgrRemoveAccelDev
 * Remove an accelerator from the Accelerator Table
 */
CpaStatus adf_devmgrRemoveAccelDev(icp_accel_dev_t *pAccelDev)
{
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);

    if(ICP_MUTEX_LOCK(&tableLock))
    {
        ADF_ERROR("Unable to aquire table lock\n");
        return CPA_STATUS_FAIL;
    }
    if (adf_devmgrFindAccelDev(pAccelDev) != CPA_STATUS_SUCCESS)
    {
        /* accel_dev does not exist */
        ICP_MUTEX_UNLOCK(&tableLock);
        return CPA_STATUS_FAIL;
    }
    ICP_REMOVE_ELEMENT_FROM_LIST(pAccelDev, pAccelTable, pAccelTableHead);
    pAccelDev->pNext = NULL;
    pAccelDev->pPrev = NULL;
    ICP_MUTEX_UNLOCK(&tableLock);
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_devmgrGetAccelHead
 * Sets the AccelDev to the head of the accelerator table.
 */
CpaStatus adf_devmgrGetAccelHead(icp_accel_dev_t **pAccelDev)
{
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    if(ICP_MUTEX_LOCK(&tableLock))
    {
        ADF_ERROR("Unable to aquire table lock\n");
        return CPA_STATUS_FAIL;
    }
    *pAccelDev = pAccelTableHead;
    ICP_MUTEX_UNLOCK(&tableLock);
    return (NULL == *pAccelDev) ? CPA_STATUS_FAIL : CPA_STATUS_SUCCESS;
}

/*
 * adf_devmgrGetAccelDev
 * Check the accel table for a structure that contains the PCI device
 * Returns a pointer to the accelerator structure or NULL if not found.
 */
icp_accel_dev_t *adf_devmgrGetAccelDev(void *pDev)
{
    icp_accel_dev_t *ptr = NULL;

    if(CPA_STATUS_SUCCESS !=
            adf_devmgrGetAccelHead(&ptr))
    {
        return NULL;
    }
    while (NULL != ptr)
    {
        if (ptr->pciAccelDev.pDev == pDev)
        {
            return ptr;
        }
        ptr = ptr->pNext;
    }
    return NULL;
}

/*
 * adf_devmgrGetAccelDevByAccelId
 * Check the accel table for a structure that contains the correct
 * accel ID. If the accelId is found return the pointer to the accelerator
 * structure.
 * Returns a pointer to the accelerator structure or NULL if not found.
 */
icp_accel_dev_t *adf_devmgrGetAccelDevByAccelId(Cpa32U accelId)
{
    icp_accel_dev_t *ptr = NULL;

    if(CPA_STATUS_SUCCESS !=
            adf_devmgrGetAccelHead(&ptr))
    {
        return NULL;
    }
    while (NULL != ptr)
    {
        if (ptr->accelId == accelId)
        {
            return ptr;
        }
        ptr = ptr->pNext;
    }
    return NULL;
}

/*
 * icp_adf_getAccelDevByAccelId
 * Same as adf_devmgrGetAccelDevByAccelId() but for external use
 * structure.
 * Returns a pointer to the accelerator structure or NULL if not found.
 */
icp_accel_dev_t* icp_adf_getAccelDevByAccelId(Cpa32U accelId)
{
    return adf_devmgrGetAccelDevByAccelId(accelId);
}

/*
 * adf_devmgrVerifyAccelId
 * Checks if there is an accelleration device with the given accelId
 * in the system. It also return tru if the id is equal to symbol
 * representing all accle ids in the system.
 * Returns success or false.
 */
CpaStatus adf_devmgrVerifyAccelId(Cpa32U accelId)
{
    icp_accel_dev_t *ptr = NULL;

    if(CPA_STATUS_SUCCESS !=
            adf_devmgrGetAccelHead(&ptr))
    {
        return CPA_STATUS_FAIL;
    }

    /* return true for all devices */
    if(ADF_CFG_ALL_DEVICES == accelId)
    {
        return CPA_STATUS_SUCCESS;
    }
    /* or for the acctual one in the system */
    while (NULL != ptr)
    {
        if (ptr->accelId == accelId)
        {
            return CPA_STATUS_SUCCESS;
        }
        ptr = ptr->pNext;
    }
    /* otherwise return false */
    return CPA_STATUS_FAIL;
}

/*
 * icp_adf_GetNumInstances
 * Returns the number of accelerators in the system.
 */
CpaStatus icp_amgr_getNumInstances(Cpa16U *pNumInstances)
{
    icp_accel_dev_t *ptr = NULL;

    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);

    if(CPA_STATUS_SUCCESS !=
            adf_devmgrGetAccelHead(&ptr))
    {
        *pNumInstances = 0;
    }
    while (NULL != ptr)
    {
        (*pNumInstances)++;
        ptr = ptr->pNext;
    }
    return CPA_STATUS_SUCCESS;

}

/*
 * icp_adf_GetInstances
 * Returns table of acceleration instances it the system.
 */
CpaStatus icp_amgr_getInstances(Cpa16U numInstances,
                  icp_accel_dev_t **pAccel_devs)
{
    Cpa16U instances = 0;
    icp_accel_dev_t *ptr = NULL;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_devs);
    icp_amgr_getNumInstances(&instances);

    if(numInstances != instances)
    {
        ADF_ERROR("Invalid param - number of instances\n");
        return CPA_STATUS_INVALID_PARAM;
    }
    if(CPA_STATUS_SUCCESS !=
            adf_devmgrGetAccelHead(&ptr))
    {
        return CPA_STATUS_FAIL;
    }
    while (NULL != ptr)
    {
        *pAccel_devs++ = ptr ;
        ptr = ptr->pNext;
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_GetInstances
 * Returns the accel instance by name.
 */
CpaStatus icp_amgr_getAccelDevByName(unsigned char* instanceName,
                                      icp_accel_dev_t **pAccel_dev)
{
    icp_accel_dev_t *ptr = NULL;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_dev);
    if(CPA_STATUS_SUCCESS !=
            adf_devmgrGetAccelHead(&ptr))
    {
        return CPA_STATUS_FAIL;
    }

    while (NULL != ptr)
    {
        if(ICP_STRNCMP((char *)instanceName, (char *)ptr->pAccelName,
                       ICP_STRLEN((char *)ptr->pAccelName)) == 0)
        {
            *pAccel_dev = ptr;
            return CPA_STATUS_SUCCESS;
        }
        ptr = ptr->pNext;
    }
    return CPA_STATUS_FAIL;
}

/*
 * icp_amgr_getAccelDevByCapabilities
 * Returns a started accel device that implements
 * the capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAccelDevByCapabilities(Cpa32U capabilitiesMask,
                                              icp_accel_dev_t **pAccel_devs,
                                              Cpa16U *pNumInstances)
{
    icp_accel_dev_t *ptr = NULL;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_devs);
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);

    *pNumInstances = 0;

    if(CPA_STATUS_SUCCESS != adf_devmgrGetAccelHead(&ptr))
    {
        return CPA_STATUS_FAIL;
    }

    while (NULL != ptr)
    {
        if(ptr->accelCapabilitiesMask & capabilitiesMask)
        {
            if(BIT_IS_SET(ptr->adfSubsystemStatus, ADF_STATUS_SYSTEM_STARTED))
            {
                *pAccel_devs = ptr;
                *pNumInstances = 1;
                return CPA_STATUS_SUCCESS;
            }
        }
        ptr = ptr->pNext;
    }
    return CPA_STATUS_FAIL;
}

/*
 * icp_amgr_getAllAccelDevByCapabilities
 * Returns table of accel devices that are started and implement
 * the capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAllAccelDevByCapabilities(Cpa32U capabilitiesMask,
                                              icp_accel_dev_t **pAccel_devs,
                                              Cpa16U *pNumInstances)
{
    icp_accel_dev_t *ptr = NULL;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_devs);
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);

    *pNumInstances = 0;
    if(CPA_STATUS_SUCCESS != adf_devmgrGetAccelHead(&ptr))
    {
        return CPA_STATUS_FAIL;
    }
    while (NULL != ptr)
    {
        if(ptr->accelCapabilitiesMask & capabilitiesMask)
        {
            if(BIT_IS_SET(ptr->adfSubsystemStatus, ADF_STATUS_SYSTEM_STARTED))
            {
                pAccel_devs[(*pNumInstances)++] = ptr;
            }
        }
        ptr = ptr->pNext;
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * icp_amgr_getAccelDevCapabilities
 * Returns accel devices capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAccelDevCapabilities(icp_accel_dev_t *accel_dev,
                                            Cpa32U *pCapabilitiesMask)
{
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pCapabilitiesMask);
    *pCapabilitiesMask = accel_dev->accelCapabilitiesMask;
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_qa_dev_get
 * Function increments the device usage counter.
 */
void icp_qa_dev_get(icp_accel_dev_t *pAccelDev)
{
    osalAtomicAdd(1, &pAccelDev->usageCounter);
}

/*
 * icp_qa_dev_get
 * Function decrements the device usage counter.
 */
void icp_qa_dev_put(icp_accel_dev_t *pAccelDev)
{
    osalAtomicSub(1, &pAccelDev->usageCounter);
}

/*
 * icp_adf_reset_dev
 * Function schedule device reset for the given device.
 */
CpaStatus icp_adf_reset_dev(icp_accel_dev_t *accel_dev,
                            icp_adf_dev_reset_mode_t mode)
{
    return adf_aer_schedule_reset_dev(accel_dev, mode) == SUCCESS ?
                           CPA_STATUS_SUCCESS : CPA_STATUS_FAIL;
}

/*
 * icp_adf_is_dev_in_reset
 * Check if device is in reset state.
 */
CpaBoolean icp_adf_is_dev_in_reset(icp_accel_dev_t *accel_dev)
{
    return (CpaBoolean)BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                     ADF_STATUS_SYSTEM_RESTARTING);
}
