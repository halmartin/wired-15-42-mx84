/*****************************************************************************
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

/*****************************************************************************
 * @file adf_ctl_drv.c
 *
 * @description
 *      This file contains the ADF code to register the ADF CTL device
 *      drive and the device drivers associated methods.
 *
 *****************************************************************************/
#include "cpa.h"
#include "icp_accel_devices.h"
#include "adf_drv.h"
#include "adf_ctl_drv.h"
#include "adf_cfg.h"
#include "adf_config_ctl.h"
#include "icp_platform.h"
#include "adf_cfg.h"
#include "icp_adf_cfg.h"
#include "adf_devmgr.h"
#include "adf_chr_drv.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_ETring_mgr.h"
#include "adf_init.h"
#include "adf_user_proxy.h"

/* Character Device Driver Name */
#define DEVICE_NAME             "icp_adf_ctl"
/* Number of Devices to be created */
#define MAX_DEVICES             1
/* Device Minor number */
#define BASE_MINOR_NUM          0

/* Mutex to be used to guard the ioctl function */
STATIC DEFINE_MUTEX(adf_ctl_lock);

/* IOCTL function for the adf_ctl device driver */
STATIC long adf_ctl_ioctl(struct file *fp, unsigned int cmd, unsigned long arg);

/* Copy all the section and key-value data from user space. */
STATIC int adf_copy_key_value_data(adf_cfg_ctl_data_t *ctl_data);

STATIC struct file_operations adf_ctl_ops = {
    owner:THIS_MODULE,
    unlocked_ioctl:adf_ctl_ioctl,
    compat_ioctl:adf_ctl_ioctl,
};

STATIC adf_chr_drv_info_t adf_ctl_drv_info = {
    owner:THIS_MODULE,
    major:0,
    min_minor:BASE_MINOR_NUM,
    max_minor:MAX_DEVICES,
    name:DEVICE_NAME,
    file_ops:&adf_ctl_ops,
};

/*
 * adf_copy_keyval_from_user
 *
 * Copy the configuration sections and associated
 * key-value data from user space.
 */
STATIC int adf_copy_keyval_from_user(icp_accel_dev_t *accel_dev,
                     adf_cfg_ctl_data_t *ctl_data)
{
    int retval = 0;
    CpaStatus status = CPA_STATUS_FAIL;
    adf_cfg_key_val_t *curr = NULL;
    adf_cfg_key_val_t *head = NULL;
    adf_cfg_device_data_t *cfg_data = NULL;
    adf_cfg_section_t *section_curr = NULL, *section_head = NULL;
    char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES] = {0};


    if (NULL == ctl_data) {
        return FAIL;
    }

    if (NULL == ctl_data->config_section) {
        /* nothing to copy */
        return SUCCESS;
    }

    status = adf_cfgDeviceFind((uint32_t)ctl_data->device_id, &cfg_data);
    if (CPA_STATUS_SUCCESS != status) {
        ADF_ERROR("config table not found(%d)\n",
                  (uint32_t)ctl_data->device_id);
        return FAIL;
    }
    section_head = ctl_data->config_section;

    /*Loop over all sections in the configuration tables from user space. */

    while (section_head != NULL) {
        /* Allocate kernel space memory for the user space data
           to be copied to.*/
        section_curr = ICP_ZALLOC_GEN(sizeof(adf_cfg_section_t));
        if (NULL == section_curr) {
            ADF_ERROR("failed to allocate memory\n");
            return FAIL;
        }
        retval = copy_from_user(section_curr,
                    (adf_cfg_section_t *)section_head,
                    sizeof(adf_cfg_section_t));
        if (SUCCESS != retval) {
            ADF_ERROR("failed to copy section info\n");
            ICP_FREE(section_curr);
            return FAIL;
        }

        head = section_curr->params;

        while (head != NULL) {
            /* Allocate kernel space memory for
             * the user space data to be copied to.*/
            curr = ICP_ZALLOC_GEN(sizeof(adf_cfg_key_val_t));
            if (NULL == curr) {
                ADF_ERROR("failed to allocate memory\n");
                ICP_FREE(section_curr);
                return FAIL;
            }

            retval = copy_from_user(curr,
                           (void *)head,
                        sizeof(adf_cfg_key_val_t));
            if (SUCCESS != retval) {
                ADF_ERROR("failed to copy keyvalue.\n");
                ICP_FREE(section_curr);
                ICP_FREE(curr);
                return FAIL;
            }

            memset(val, '\0', ADF_CFG_MAX_VAL_LEN_IN_BYTES);

            status = icp_adf_cfgGetParamValue(accel_dev,
                              section_curr->name,
                              curr->key,
                              val);

            /* Add the key value pairs to the section
             * structure in kernel space.*/
            if(CPA_STATUS_SUCCESS != status) {
                ADF_DEBUG("Device %d Section %s key %s"
                          " - no such value\n",
                          accel_dev->accelId,
                          section_curr->name,
                          curr->key);
            }
            else {
                retval = copy_to_user( head->val, val,
                               strlen(val)+1);
                if (SUCCESS != retval) {
                    ADF_ERROR("failed to copy"
                              "to user %d\n",
                              retval);
                }
            }

            head = curr->pNext;
            ICP_FREE(curr);
        } /*End of while params */
        section_head = section_curr->pNext;
        ICP_FREE(section_curr);
    } /*End of while loop over the sections. */
    return SUCCESS;
}

/*
 * adf_ctl_dealloc_resources
 *
 * Function to deallocate ctl data resources
 */
STATIC void adf_ctl_dealloc_resources(adf_cfg_ctl_data_t *ctl_data)
{
    ICP_FREE(ctl_data);
}

/*
 * adf_ctl_alloc_resources
 *
 * Function that allocates the resources needed to copy data from
 * user space to kernel space. It also copies the ctl data.
 */
STATIC int adf_ctl_alloc_resources(adf_cfg_ctl_data_t **ctl_data,
                   unsigned long arg)
{
    int ret = SUCCESS;
    adf_cfg_ctl_data_t *tmp_data = NULL;

    tmp_data = ICP_ZALLOC_GEN(sizeof(adf_cfg_ctl_data_t));
    if (NULL == tmp_data) {
        ADF_ERROR("failed to allocate tmp_data.\n");
        return -ENOMEM;
    }

    /* Initialize device id to NO DEVICE as 0 is a valid device id */
    tmp_data->device_id = ADF_CFG_NO_DEVICE;

    /*
     * Copy the configuration data from user address space.
     * The device id and a pointer to a user space address
     * pointing to the section data structure are also copied.
     */
    ret = copy_from_user(tmp_data,
                 (adf_cfg_ctl_data_t *)arg,
                 sizeof(adf_cfg_ctl_data_t));
    if (SUCCESS != ret) {
        ADF_ERROR("failed to copy from user tmp_data.\n");
        adf_ctl_dealloc_resources(tmp_data);
        return -EIO;
    }

    *ctl_data = tmp_data;

    return SUCCESS;
}

/*
 * adf_ctl_ioctl_dev_config
 *
 * Function to copy configuration data from user space to kernel space.
 */
STATIC int adf_ctl_ioctl_dev_config(struct file *fp, unsigned int cmd,
              unsigned long arg)
{
    int ret = SUCCESS;
    adf_cfg_ctl_data_t *ctl_data = NULL;
    icp_accel_dev_t *accel_dev = NULL;

    ret = adf_ctl_alloc_resources(&ctl_data, arg);
    if (SUCCESS != ret) {
        return ret;
    }

    /* Get the Accel handle */
    accel_dev = adf_devmgrGetAccelDevByAccelId(
        (uint32_t)ctl_data->device_id);
    if (NULL == accel_dev) {
        ADF_ERROR("Device %d not found\n",
            (uint32_t)ctl_data->device_id);
        adf_ctl_dealloc_resources(ctl_data);
        return -ENODEV;
    }

    /* Check the state of the device. */
    if (BIT_IS_SET(accel_dev->adfSubsystemStatus,
        ADF_STATUS_SYSTEM_STARTED)) {
        ADF_ERROR("System already configured & started\n");
        adf_ctl_dealloc_resources(ctl_data);
        return -EPERM;
    }

    ADF_PRINT("Reading config file.\n");

    /* Copy device configuration data */
    ret = adf_copy_key_value_data(ctl_data);
    if (SUCCESS != ret) {
        ADF_ERROR("failed to copy configuration data\n");
        adf_ctl_dealloc_resources(ctl_data);
        return -EIO;
    }

    /* Free allocated resources */
    adf_ctl_dealloc_resources(ctl_data);

    /* Set the status for this device */
    SET_STATUS_BIT(accel_dev->adfSubsystemStatus,
        ADF_STATUS_SYSTEM_CONFIGURED);

    return SUCCESS;
}

/*
 * adf_ctl_ioctl_dev_get_value
 *
 * Function to get device configuration data.
 */
STATIC int adf_ctl_ioctl_dev_get_value(struct file *fp, unsigned int cmd,
              unsigned long arg)
{
    int ret = SUCCESS;
    adf_cfg_ctl_data_t *ctl_data = NULL;
    icp_accel_dev_t *accel_dev = NULL;

    ret = adf_ctl_alloc_resources(&ctl_data, arg);
    if (SUCCESS != ret) {
        return ret;
    }

    /* Get the Accel handle */
    accel_dev = adf_devmgrGetAccelDevByAccelId(
        (uint32_t)ctl_data->device_id);
    if (NULL == accel_dev) {
        ADF_ERROR("Device %d not found\n",
            (uint32_t)ctl_data->device_id);
        adf_ctl_dealloc_resources(ctl_data);
        return -ENODEV;
    }

    /* Check the state of the device. */
    if (!BIT_IS_SET(accel_dev->adfSubsystemStatus,
        ADF_STATUS_SYSTEM_STARTED)) {
        ADF_ERROR("System not started\n");
        adf_ctl_dealloc_resources(ctl_data);
        return -EPERM;
    }

    /* Get the key-value data from user space. */
    ret = adf_copy_keyval_from_user(accel_dev, ctl_data);
    if (SUCCESS != ret) {
        adf_ctl_dealloc_resources(ctl_data);
        return -EIO;
    }

    adf_ctl_dealloc_resources(ctl_data);

    return SUCCESS;
}

/*
 * adf_ctl_is_device_in_use
 *
 * Checks if the device is used by any userspace process or
 * internal module that doesn't like the device to be shutdown.
 * Returns 0 if the device is not used by any userspace process.
 */
STATIC int adf_ctl_is_device_in_use(int device_id)
{
    icp_accel_dev_t *accel_dev = NULL;

    if (ADF_CFG_ALL_DEVICES == device_id) {
        if (CPA_STATUS_SUCCESS != adf_devmgrGetAccelHead(&accel_dev)) {
            ADF_ERROR("Unable to get acceleration devices\n");
            return -EFAULT;
        }
    }
    else {
        accel_dev = adf_devmgrGetAccelDevByAccelId(device_id);
        if (NULL == accel_dev) {
            ADF_ERROR("Unable to get acceleration devices\n");
            return -EFAULT;
        }
    }

    while(accel_dev) {
        if (icp_adf_is_dev_in_reset(accel_dev)) {
            ADF_PRINT("device icp_dev%d is busy\n",
                    accel_dev->accelId);
            return EBUSY;
        }
        if (osalAtomicGet(&accel_dev->usageCounter) > 0) {
            ADF_PRINT("device icp_dev%d is busy\n",
                    accel_dev->accelId);
            return EBUSY;
        }
        if (ADF_CFG_ALL_DEVICES != device_id) {
            break;
        }
        accel_dev = accel_dev->pNext;
    }
    return 0;
}

/*
 * adf_ctl_ioctl_dev_stop
 *
 * Function to stop the acceleration devices.
 */
STATIC int adf_ctl_ioctl_dev_stop(struct file *fp, unsigned int cmd,
              unsigned long arg)
{
    int ret = SUCCESS;
    adf_cfg_ctl_data_t *ctl_data = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;

    ret = adf_ctl_alloc_resources(&ctl_data, arg);
    if (SUCCESS != ret) {
        return ret;
    }

    /* Verify the device id */
    status = adf_devmgrVerifyAccelId((uint32_t)ctl_data->device_id);
    if (CPA_STATUS_SUCCESS != status) {
        ADF_ERROR("Device %d not found\n",
            (uint32_t)ctl_data->device_id);
        adf_ctl_dealloc_resources(ctl_data);
        return -ENODEV;
    }

    /*
     * Check if the device is used by any userspace process
     * or internal module before shutting it down
     */
    ret = adf_ctl_is_device_in_use((uint32_t)ctl_data->device_id);
    if (0 != ret) {
        adf_ctl_dealloc_resources(ctl_data);
        return EBUSY;
    }


    if (ADF_CFG_ALL_DEVICES == ctl_data->device_id) {
        ADF_PRINT("Stopping all acceleration devices.\n");
    }
    else {
        ADF_PRINT("Stopping acceleration device icp_dev%d.\n",
                         (uint32_t)ctl_data->device_id);
    }

    ret = adf_config_ctl_stop_devices((uint32_t)ctl_data->device_id);
    if (SUCCESS != ret) {
        ADF_ERROR("failed to stop device %d.\n",
            (uint32_t)ctl_data->device_id);
        adf_ctl_dealloc_resources(ctl_data);
        return -EFAULT;
    }

    adf_ctl_dealloc_resources(ctl_data);

    return SUCCESS;
}

/*
 * adf_ctl_ioctl_dev_start
 *
 * Function to start acceleration device
 */
STATIC int adf_ctl_ioctl_dev_start(struct file *fp, unsigned int cmd,
              unsigned long arg)
{
    int ret = SUCCESS;
    adf_cfg_ctl_data_t *ctl_data = NULL;
    icp_accel_dev_t *accel_dev = NULL;

    /* allocate resources, memory allocation and getting
     * info from user space. */
    ret = adf_ctl_alloc_resources(&ctl_data, arg);
    if (SUCCESS != ret) {
        return ret;
    }

    accel_dev = adf_devmgrGetAccelDevByAccelId(
        (uint32_t)ctl_data->device_id);
    if (NULL == accel_dev) {
        ADF_ERROR("Device %d not found\n",
            (uint32_t)ctl_data->device_id);
        adf_ctl_dealloc_resources(ctl_data);
        return -ENODEV;
    }

    ADF_PRINT("Starting acceleration device icp_dev%d.\n",
                           (uint32_t)ctl_data->device_id);

    /* Initialize the Acceleration Subsystem */
    ret = adf_init_devices((uint32_t)ctl_data->device_id,
                           IOCTL_START_ACCEL_DEV);
    if (ret) {
        ADF_ERROR("failed call to adf_init_devices\n");
        adf_ctl_dealloc_resources(ctl_data);
        return -EIO;
    }

    adf_ctl_dealloc_resources(ctl_data);

    return SUCCESS;
}

/*
 * adf_ctl_ioctl_dev_reset
 *
 * Function to reset acceleration device
 */
STATIC int adf_ctl_ioctl_dev_reset(struct file *fp, unsigned int cmd,
              unsigned long arg)
{
    int ret = SUCCESS;
    adf_cfg_ctl_data_t *ctl_data = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;

    ret = adf_ctl_alloc_resources(&ctl_data, arg);
    if (SUCCESS != ret) {
        return ret;
    }

    /* Verify the device id */
    status = adf_devmgrVerifyAccelId((uint32_t)ctl_data->device_id);
    if (CPA_STATUS_SUCCESS != status) {
        ADF_ERROR("Device %d not found\n",
            (uint32_t)ctl_data->device_id);
        adf_ctl_dealloc_resources(ctl_data);
        return -ENODEV;
    }

    if (ADF_CFG_ALL_DEVICES == ctl_data->device_id) {
        ADF_PRINT("Scheduling reset of all acceleration devices.\n");
    }
    else {
        ADF_PRINT("Scheduling reset of device icp_dev%d.\n",
                         (uint32_t)ctl_data->device_id);
    }

    ret = adf_config_ctl_reset_devices((uint32_t)ctl_data->device_id);
    if (SUCCESS != ret) {
        ADF_ERROR("failed to reset device %d.\n",
            (uint32_t)ctl_data->device_id);
        adf_ctl_dealloc_resources(ctl_data);
        return -EFAULT;
    }

    adf_ctl_dealloc_resources(ctl_data);

    return SUCCESS;
}

/*
 * adf_ctl_ioctl_get_num_devices
 *
 * Function to get the number of acceleration devices in the system
 */
STATIC int adf_ctl_ioctl_get_num_devices(struct file *fp, unsigned int cmd,
              unsigned long arg)
{
    int ret = 0;
    int num_devices = 0;

    /* Calculate the number of acceleration devices.*/
    if (CPA_STATUS_SUCCESS !=
            icp_amgr_getNumInstances((Cpa16U *)&num_devices)) {
        ADF_ERROR("Failed to get num_devices\n");
        return -EFAULT;
    }

    /* Copy the num_devices back to user address space. */
    ret = copy_to_user((int *)arg, &num_devices, sizeof (num_devices));
    if (SUCCESS != ret) {
        ADF_ERROR("failed to copy num_devices.\n");
        return -EFAULT;
    }

    return SUCCESS;
}

/*
 * adf_ctl_ioctl_get_status
 *
 * Function to get status of acceleration devices in the system
 */
STATIC int adf_ctl_ioctl_get_status(struct file *fp, unsigned int cmd,
              unsigned long arg)
{
    register adf_dev_status_info_t *stat = NULL;
    int ret = 0;
    int index = 0;
    icp_accel_dev_t *accel_dev = NULL;
    icp_accel_pci_info_t *pci_info = NULL;
    adf_hw_device_data_t *hw_data = NULL;
    adf_dev_status_info_t *dev_status =
                   ICP_ZALLOC_GEN(sizeof(adf_dev_status_info_t)
                                * ADF_MAX_DEVICES);
    if (!dev_status) {
        ADF_ERROR("failed to allocate memory for dev status\n");
        return -ENOMEM;
    }

    ICP_MEMSET(dev_status, '\0', sizeof(adf_dev_status_info_t)
                                * ADF_MAX_DEVICES);
    if (CPA_STATUS_SUCCESS != adf_devmgrGetAccelHead(&accel_dev)) {
        ADF_ERROR("failed to get device handle\n");
        ICP_FREE(dev_status);
        return -EFAULT;
    }

    for (index = 0; NULL != accel_dev; ++index){
        hw_data = accel_dev->pHwDeviceData;
        stat = dev_status + index;

        stat->state = BIT_IS_SET( accel_dev->adfSubsystemStatus,
            ADF_STATUS_SYSTEM_STARTED ) ? DEV_UP : DEV_DOWN;

        stat->numAe = hw_data->getNumAccelEngines(
            hw_data, accel_dev->aeMask);

        stat->numAccel = hw_data->getNumAccelerators(
            hw_data, accel_dev->accelMask);

        stat->maxNumAccel = hw_data->maxNumAccel;

        stat->maxNumBanks = hw_data->maxNumBanks;

        stat->numBanksPerAccel = hw_data->numBanksPerAccel;

        stat->numRingsPerBank = hw_data->numRingsPerBank;

        ICP_STRNCPY(stat->deviceName, hw_data->dev_class->name,
                         MAX_DEVICE_NAME_SIZE);

        stat->instanceId = hw_data->instanceId;
        stat->accelId    = accel_dev->accelId;
        stat->type       = hw_data->dev_class->type;
        stat->nodeId     = accel_dev->pkg_id;

        pci_info = &accel_dev->pciAccelDev;
        stat->busId = pci_info->pciDomain.bus;
        stat->slotId = pci_info->pciDomain.slot;
        stat->functionId = pci_info->pciDomain.function;

        /* get next accel dev            */
        accel_dev = accel_dev->pNext;
    }

    /* Copy the num_devices back to user address space. */
    ret = copy_to_user((adf_dev_status_info_t *)arg, dev_status,
           sizeof(adf_dev_status_info_t) * ADF_MAX_DEVICES);
    ICP_FREE(dev_status);
    if (SUCCESS != ret) {
        ADF_ERROR("failed to copy status.\n");
        return -EFAULT;
    }
    return SUCCESS;
}

/*
 * icp_adf_get_busAddress
 *
 * Function to get busAddress of acceleration devices in the system
 *
 * The format of the returned 16-bit bus address is as follows:
 *     8 bits - Bus Id
 *     5 bits - Slot Id
 *     3 bits - Function Id
 */
Cpa16U icp_adf_get_busAddress(Cpa16U packageId)
{
    Cpa16S busAddress = -EFAULT;
    int index = 0;
    icp_accel_dev_t *accel_dev = NULL;
    icp_accel_pci_info_t *pci_info = NULL;

    if (CPA_STATUS_SUCCESS != adf_devmgrGetAccelHead(&accel_dev)) {
        ADF_ERROR("failed to get device handle\n");
        return -EFAULT;
    }

    for (index = 0; NULL != accel_dev; ++index) {
        pci_info = &accel_dev->pciAccelDev;

        if (index == packageId) {
         busAddress = (pci_info->pciDomain.bus<<8) |
                  (pci_info->pciDomain.slot<<3) |
                   pci_info->pciDomain.function;
         break;
        }
        /* get next accel dev            */
        accel_dev = accel_dev->pNext;
    }

    if (-EFAULT == busAddress) {
        ADF_ERROR("failed to get pci info for device\n");
    }
    return busAddress;
}

/*
 * adf_ctl_put_get_dev
 *
 * Function to increases or decreases device usage counter
 */
STATIC int adf_ctl_put_get_dev(struct file *fp, unsigned int cmd,
              unsigned long arg)
{
    int ret = 0;
    Cpa32U dev_id = 0;
    icp_accel_dev_t *accel_dev = NULL;

    /* Copy the device id from address space. */
    ret = copy_from_user((void*)&dev_id, (void*)arg, sizeof(Cpa32U));
    if (SUCCESS != ret) {
        ADF_ERROR("failed to copy status.\n");
        return -EFAULT;
    }
    accel_dev = adf_devmgrGetAccelDevByAccelId(dev_id);
    if (NULL == accel_dev) {
        ADF_ERROR("failed to to get device handle\n");
        return -EFAULT;
    }
    if (cmd == DEV_PUT_CMD) {
        icp_qa_dev_put(accel_dev);
    }
    else if (cmd == DEV_GET_CMD) {
        icp_qa_dev_get(accel_dev);
    }
    adf_user_proxyEventGetPut( accel_dev, current->tgid, cmd);
    return SUCCESS;
}



/*
 * adf_ctl_ioctl
 *
 * IOCTL function for the adf_ctl char device driver. Copies the configuration
 * parameters from user space, updates the ADF Configuration Parameter Table
 * and initialise the subsystem.
 */
STATIC long adf_ctl_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret = SUCCESS;

    if (mutex_lock_interruptible(&adf_ctl_lock)) {
            return -EBUSY;
    }

    switch(cmd) {
        case IOCTL_CONFIG_SYS_RESOURCE_PARAMETERS:
            ret = adf_ctl_ioctl_dev_config(fp, cmd, arg);
            break;

        case IOCTL_GET_VALUE_ACCEL_DEV:
            ret = adf_ctl_ioctl_dev_get_value(fp, cmd, arg);
            break;

        case IOCTL_STOP_ACCEL_DEV:
            ret = adf_ctl_ioctl_dev_stop(fp, cmd, arg);
            break;

        case IOCTL_START_ACCEL_DEV:
            ret = adf_ctl_ioctl_dev_start(fp, cmd, arg);
            break;

        case IOCTL_RESET_ACCEL_DEV:
            ret = adf_ctl_ioctl_dev_reset(fp, cmd, arg);
            break;

        case IOCTL_GET_NUM_DEVICES:
            ret = adf_ctl_ioctl_get_num_devices(fp, cmd, arg);
            break;

        case IOCTL_STATUS_ACCEL_DEV:
            ret = adf_ctl_ioctl_get_status(fp, cmd, arg);
            break;

        case IOCTL_DEV_GET:
            ret = adf_ctl_put_get_dev(fp, DEV_GET_CMD, arg);
            break;

        case IOCTL_DEV_PUT:
            ret = adf_ctl_put_get_dev(fp, DEV_PUT_CMD, arg);
            break;

        default:
            ADF_ERROR("Invalid IOCTL command specified(0x%x)", cmd);
            ret = -ENOTTY;
            break;
    }
    mutex_unlock(&adf_ctl_lock);
    return ret;
}

/*
 * register_adf_ctl_device_driver
 *
 * Function which dynamically allocates the major number, creates the adf_ctl
 * char device driver and adds it to the system.
 */
int register_adf_ctl_device_driver(void)
{
    int ret = 0;

    /*
     * Create a mutex to be used to guard the device driver
     * ioctl function.
     */
    mutex_init(&adf_ctl_lock);
    ret = adf_chr_drv_create(&adf_ctl_drv_info);
    if (SUCCESS != ret) {
        ADF_ERROR("failed to create MEM device driver\n");
        return FAIL;
    }
    ret = adf_chr_drv_create_device(&adf_ctl_drv_info,
                    0,
                    DEVICE_NAME);
    if (SUCCESS != ret) {
        ADF_ERROR("failed to create MEM device\n");
        return FAIL;
    }
    return SUCCESS;
}

/*
 * unregister_adf_ctl_device_driver
 *
 * Function which removes the adf_ctl char device from the system, deallocates
 * the major number.
 */
void unregister_adf_ctl_device_driver(void)
{
    adf_chr_drv_destroy(&adf_ctl_drv_info);
}

/*
 * User data is copied from user space to kernel space and
 * the config section list is updated.
 */
int adf_copy_key_value_data(adf_cfg_ctl_data_t *ctl_data)
{
    int retval = 0;
    CpaStatus status = CPA_STATUS_FAIL;
    int keyval_len = 0, section_len=0;
    adf_cfg_key_val_t *curr = NULL;
    adf_cfg_key_val_t *next = NULL;
    adf_cfg_device_data_t *cfg_data = NULL;
    adf_cfg_section_t *section_curr = NULL, *section_next = NULL;
    adf_cfg_section_t section_copy = {{0}};

    if (NULL == ctl_data) {
        return FAIL;
    }

    if (NULL == ctl_data->config_section) {
        /* nothing to copy */
        return SUCCESS;
    }

    status = adf_cfgDeviceFind((uint32_t)ctl_data->device_id, &cfg_data);
    if (CPA_STATUS_SUCCESS != status) {
        ADF_ERROR("config table not found(%d)\n",
                  (uint32_t)ctl_data->device_id);
        return FAIL;
    }

    section_len = sizeof(adf_cfg_section_t);

    /* section_next is now the pointer to the first of the
     * cfg_section's for this device. section_next is a pointer to
     * memory in user space.*/
    section_next = ctl_data->config_section;
    /*Loop over all sections in the configuration tables from user space. */
    while (section_next != NULL) {
        /* Allocate kernel space memory for the user space data
           to be copied to.*/
        section_curr = ICP_ZALLOC_GEN(section_len);
        if (NULL == section_curr) {
            ADF_ERROR("failed to allocate memory\n");
            adf_cfgSectionDel(&(cfg_data->config_section));
            return FAIL;
        }
        /* The user space section is copied into a
         * kernel space section. */
        /* This kernel space section will also include
         * user space pointers which need to be copied. */
        retval = copy_from_user(&section_copy,
                    (adf_cfg_section_t *)section_next,
                    section_len);
        if (SUCCESS != retval) {
            ADF_ERROR("failed to copy section info\n");
            ICP_FREE(section_curr);
            adf_cfgSectionDel(&(cfg_data->config_section));
            return FAIL;
        }
        memcpy(section_curr->name,
               section_copy.name, sizeof(section_copy.name));   

        /* Within the section structure exist
         * the key-value parameters. */

        keyval_len = sizeof(adf_cfg_key_val_t);

        /* next is now the pointer to the head of the
         * key/value list for this section. next is
         * a pointer to memory in user space.*/
        next = section_copy.params;

        while (next != NULL) {
            /* Allocate kernel space memory for
             * the user space data to be copied to.*/
            curr = ICP_ZALLOC_GEN(keyval_len);
            if (NULL == curr) {
                ADF_ERROR("failed to allocate memory\n");
                adf_cfgSectionDel(
                      &(cfg_data->config_section));
                return FAIL;
            }
            retval = copy_from_user(curr,
                        (adf_cfg_key_val_t *)next,
                        keyval_len);
            if (SUCCESS != retval) {
                ADF_ERROR("failed to copy keyvalue.\n");
                ICP_FREE(curr);
                adf_cfgSectionDel(
                      &(cfg_data->config_section));
                return FAIL;
            }
            /* next is moved to be the pointer to the
             * next structure in user space. */
            next = curr->pNext;

            /* Need to zero the user space linked list
             * since we are going to make a new kernel space one. */
            curr->pNext = NULL;

            /* Add the key value pairs to the section
             * structure in kernel space.*/

            status = adf_cfgKeyValueAddNode(
                     &(section_curr->params),
                     curr);

            if(CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("Failed to add key  %s to %s.\n",
                          section_curr->params->key,
                          section_curr->name);
                return FAIL;
            }

        } /* End of while loop which loops over the
           * key/value pairs in each section. */

        /* section_next is moved to be the pointer to the
         * next structure in user space. Do not move this
         * assignment to after the adf_cfgSectionAddNode
         * - otherwise section_next would point to the
         * previous kernel space structure.
         */
        section_next = section_copy.pNext;

        /* Add the new section to the configuration
         * table in kernel space.*/
        status = adf_cfgSectionAddNode(&(cfg_data->config_section),
                           section_curr);
        if(CPA_STATUS_SUCCESS != status) {
            ADF_ERROR("Failed to add section %s to the config.\n",
                  section_curr);
            return FAIL;
        }
    } /*End of while loop over the sections. */
    return SUCCESS;
}


