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
 * @file adf_dev_processes.c
 *
 * @description
 *      This file contains the ADF code that manages user space
 *      processes names.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "adf_chr_drv.h"
#include "icp_accel_devices.h"
#include "adf_devmgr.h"
#include "adf_cfg.h"
#include "adf_platform.h"
#include "icp_adf_cfg.h"
#include "icp_adf_init.h"
#include "adf_init.h"
#include <linux/list.h>

#define ADF_DEV_PROCESSES_NAME             ("icp_dev_processes")
#define ADF_DEV_PROCESSES_MAX_MINOR        (255)
#define ADF_DEV_PROCESSES_BASE_MINOR       (0)
#define ADF_MAX_PROCESSES_PER_SINGLE_DEV   (128)
#define ADF_MAX_PROCESSES_PER_ALL_DEV      (128 * ADF_MAX_DEVICES)

/*
 * File operation declarations
 */

/* open function. */
STATIC int adf_processes_open(struct inode *inp,
                         struct file *fp);

/* release function. */
STATIC int adf_processes_release(struct inode *inp,
                            struct file *fp);

/* read function. */
STATIC ssize_t adf_processes_read(struct file *fp,
                             char __user *buf,
                             size_t count,
                             loff_t *pos);
/* write function. */
STATIC ssize_t adf_processes_write(struct file *fp,
                                   const char __user *buf,
                                   size_t count , loff_t *pos);

/*
 * Structure that is stored in file->private_data
 */
typedef struct adf_processes_priv_data_s {
        char   name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES];
        int    read_flag;
        struct list_head list;
} adf_processes_priv_data_t;

/*
 * Character dev file operations
 */
STATIC struct file_operations adf_processes_ops = {
        owner:THIS_MODULE,
        open:adf_processes_open,
        release:adf_processes_release,
        read:adf_processes_read,
        write:adf_processes_write,
};

/*
 * Character driver info
 */
STATIC adf_chr_drv_info_t adf_processes_drv_info = {
        owner:THIS_MODULE,
        major:0,
        min_minor:ADF_DEV_PROCESSES_BASE_MINOR,
        max_minor:ADF_DEV_PROCESSES_MAX_MINOR,
        name:ADF_DEV_PROCESSES_NAME,
        file_ops:&adf_processes_ops,
};

/*
 * List of processes connected to the driver
 */
STATIC LIST_HEAD(processes_list);

/*
 * Mutex guarding the list of processes connected to the driver
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
static DEFINE_SEMAPHORE(processes_list_sema);
#else
static DECLARE_MUTEX(processes_list_sema);
#endif

/*
 * adf_processes_chrdev_unregister
 * Function unregisters character device interface
 */
void adf_processes_chrdev_unregister(void)
{
        adf_chr_drv_destroy(&adf_processes_drv_info);
}

/*
 * adf_processes_open
 * open file operation
 */
STATIC int adf_processes_open(struct inode *inp, struct file *fp)
{
        Cpa32U i = 0, devices = 0;
        icp_accel_dev_t *accel_dev = NULL;
        adf_processes_priv_data_t *prv_data = NULL;

        for(i = 0; i < ADF_MAX_DEVICES; i++)
        {
                accel_dev = adf_devmgrGetAccelDevByAccelId(i);
                if (NULL == accel_dev) {
                        continue;
                }
                if (!(BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                 ADF_STATUS_SYSTEM_STARTED))) {
                        continue;
                }
                devices++;
        }
        if (!devices) {
                ADF_ERROR("Device not yet ready.\n");
                return -EACCES;
        }
        prv_data = ICP_ZALLOC_GEN(sizeof(adf_processes_priv_data_t));
        if (NULL == prv_data) {
                ADF_ERROR("Failed to allocated private data\n");
                return -ENOMEM;
        }
        INIT_LIST_HEAD(&prv_data->list);
        fp->private_data = prv_data;
        return 0;
}

/*
 * adf_get_first_started_dev
 * Returns id of the first started device.
 */
STATIC int adf_get_first_started_dev(void)
{
        Cpa32U i = 0;
        icp_accel_dev_t *accel_dev = NULL;

        for(i = 0; i < ADF_MAX_DEVICES; i++)
        {
                accel_dev = adf_devmgrGetAccelDevByAccelId(i);
                if (NULL == accel_dev) {
                        continue;
                }
                if (BIT_IS_SET(accel_dev->adfSubsystemStatus,
                                 ADF_STATUS_SYSTEM_STARTED)) {
                        return i;
                }
        }
        return -1;
}

/*
 * adf_check_dev_started
 * Returns 1 if the device is started.
 */
STATIC int adf_check_dev_started(int id)
{
        icp_accel_dev_t *accel_dev = adf_devmgrGetAccelDevByAccelId(id);
        if (NULL == accel_dev) {
                return 0;
        }
        if (BIT_IS_SET(accel_dev->adfSubsystemStatus,
                       ADF_STATUS_SYSTEM_STARTED)) {
                return 1;
        }
        return 0;
}

/*
 * adf_processes_write
 * write file operation.
 */
STATIC ssize_t adf_processes_write(struct file *fp, const char __user *buf,
                                   size_t count , loff_t *pos)
{
        adf_processes_priv_data_t *prv_data = NULL;
        adf_processes_priv_data_t *pdata = NULL;
        Cpa32U dev_num = 0, pr_num = 0, dev_access_limit = 0;
        struct list_head *lpos = NULL ;
        char usr_name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = {0};
        char *ptr = NULL;
        icp_accel_dev_t *accel_dev = NULL;
        adf_cfg_device_data_t *cfg = NULL;
        adf_cfg_section_t *section_ptr = NULL;
        CpaBoolean pr_name_available = CPA_TRUE;
        CpaStatus status = CPA_STATUS_FAIL;

        /*
         * Validate parameters
         */
        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        prv_data = (adf_processes_priv_data_t*)fp->private_data;
        if (prv_data->read_flag == 1) {
                ADF_ERROR("Can only write once\n");
                return -EBADF;
        }
        if ((count <= 0) || (count > ADF_CFG_MAX_SECTION_LEN_IN_BYTES)) {
                ADF_ERROR("wrong size %d\n", count);
                return -EIO;
        }
        /*
         * Get the user template name
         */
        if (copy_from_user(usr_name, buf, count)) {
                ADF_ERROR("Can't copy data\n");
                return -EIO;
        }
        /*
         * Lock other processes and try to find out the process name
         */
        if (down_interruptible(&processes_list_sema)) {
                ADF_ERROR("Can't aquire process info lock\n");
                return -EBADF;
        }

        if (strstr(usr_name, DEV_LIMIT_CFG_ACCESS_TMPL)) {
                dev_access_limit = 1;
                ptr = usr_name + strlen(DEV_LIMIT_CFG_ACCESS_TMPL);
        }
        else {
                ptr = usr_name;
        }

        /*
         * If there is nothing there then take the first name and return
         */
        if (list_empty(&processes_list)) {
                if (dev_access_limit) {
                        int dev_id = adf_get_first_started_dev();
                        if (-1 == dev_id) {
                                ADF_ERROR("Could not find started device\n");
                                up(&processes_list_sema);
                                return -EIO;
                        }
                        snprintf(prv_data->name,
                               ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
                               "%s_DEV%d"INTERNAL_USERSPACE_SEC_SUFF"%d",
                                                    ptr, dev_id, 0);
                }
                else {
                        snprintf(prv_data->name,
                         ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
                         "%s"INTERNAL_USERSPACE_SEC_SUFF"%d", ptr, 0);
                }
                list_add(&prv_data->list, &processes_list);
                up(&processes_list_sema);
                prv_data->read_flag = 1;
                return 0;
        }

        /*
         * If there are processess running then search for a first free name
         */

        for(dev_num = 0; dev_num < ADF_MAX_DEVICES; dev_num++) {
            if (!adf_check_dev_started(dev_num))
               continue; /* to next device */
            /* get access to the config table for this device */
            accel_dev = adf_devmgrGetAccelDevByAccelId(dev_num);
            if(NULL == accel_dev) {
                ADF_ERROR("device does not exist in the config table %d\n",
                          dev_num);
                continue;
            }
            status = adf_cfgDeviceFind(accel_dev->accelId, &cfg);
            if (CPA_STATUS_SUCCESS != status) {
                ADF_ERROR("Can't access config table for device %d\n", dev_num);
                /* Continue - this device may have just gone down but
                 * there may be other devices available
                 */
                continue;
            }
            for(pr_num=0; pr_num < ADF_MAX_PROCESSES_PER_SINGLE_DEV; pr_num++){
                /* figure out name */
                if (dev_access_limit) {
                    snprintf(prv_data->name,
                            ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
                            "%s_DEV%d"INTERNAL_USERSPACE_SEC_SUFF"%d",
                            ptr, dev_num, pr_num);
                }
                else {
                    snprintf(prv_data->name,
                            ADF_CFG_MAX_SECTION_LEN_IN_BYTES,
                            "%s"INTERNAL_USERSPACE_SEC_SUFF"%d",
                            ptr, pr_num);
                }
                pr_name_available = CPA_TRUE;
                /* Figure out if section exists in the config table */
                section_ptr = adf_cfgSectionFind(cfg->config_section,
                                                        prv_data->name);
                if (NULL == section_ptr) {
                    /* This section name doesn't exist */
                    pr_name_available = CPA_FALSE;
                    /* As process_num enumerates from 0, once we get
                     * to one which doesn't exist no further ones will
                     * exist. On to next device
                     */
                    break;
                }
                /* Figure out if it's been taken already */
                list_for_each(lpos, &processes_list) {
                    pdata = list_entry(lpos,
                            adf_processes_priv_data_t, list);
                    if (strncmp(pdata->name, prv_data->name,
                            ADF_CFG_MAX_SECTION_LEN_IN_BYTES) == 0) {
                        pr_name_available = CPA_FALSE;
                        break;
                    }
                }
                if (pr_name_available) {
                    break; /* out of process loop*/
                }
            }
            if (pr_name_available) {
                break; /* out of device loop*/
            }
        }
        /*
         * If we have a valid name that is not on the list take it and
         * add to the list
         */
        if ((pr_name_available) 
            ||(NULL != strstr(prv_data->name,"DYN"))
            ||(dev_access_limit)) {
                list_add(&prv_data->list, &processes_list);
                up(&processes_list_sema);
                prv_data->read_flag = 1;
                return 0;
        }
        /*
         * If not then the process needs to wait
         */
        else {
                up(&processes_list_sema);
                memset(prv_data->name, '\0', ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
                prv_data->read_flag = 0;
                return 1;
        }
        return 1;
}

/*
 * adf_processes_read
 * read file operation
 */
STATIC ssize_t adf_processes_read(struct file *fp, char __user *buf,
                                  size_t count, loff_t *pos)
{
        adf_processes_priv_data_t *prv_data = NULL;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        prv_data = (adf_processes_priv_data_t*)fp->private_data;

        /*
         * If there is a name that the process can use then give it
         * to the proocess.
         */
        if (prv_data->read_flag) {
                if (copy_to_user(buf, prv_data->name,
                                    strlen(prv_data->name))) {
                        ADF_ERROR("Can't copy data\n");
                        return -EIO;
                }
                return 0;
        }
        /*
         * Else there is no valid name to use.
         */
        return -EIO;
}

/*
 * adf_processes_release
 * release file operation
 */
STATIC int adf_processes_release(struct inode *inp, struct file *fp)
{
        adf_processes_priv_data_t *prv_data = NULL;

        if (NULL == fp || NULL == fp->private_data) {
                ADF_ERROR("invalid file descriptor\n");
                return -EBADF;
        }
        prv_data = (adf_processes_priv_data_t*)fp->private_data;
        if (down_interruptible(&processes_list_sema)) {
                ADF_ERROR("Can't aquire process info lock\n");
                return -EBADF;
        }
        list_del(&prv_data->list);
        up(&processes_list_sema);
        ICP_FREE(fp->private_data);
        return 0;
}

/*
 * register_adf_ctl_device_driver
 *
 * Function which dynamically allocates the major number, creates the adf
 * char device driver and adds it to the system.
 */
int register_adf_processes_device_driver(void)
{
        int ret = 0;

        ret = adf_chr_drv_create(&adf_processes_drv_info);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create processes device driver\n");
                return FAIL;
        }
        ret = adf_chr_drv_create_device(&adf_processes_drv_info, 0,
                                        ADF_DEV_PROCESSES_NAME);
        if (SUCCESS != ret) {
                ADF_ERROR("failed to create processes device\n");
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
void unregister_adf_processes_device_driver(void)
{
        adf_chr_drv_destroy(&adf_processes_drv_info);
}
