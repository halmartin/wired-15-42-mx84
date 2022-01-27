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
 * @file adf_chr_drv.c
 *
 * @description
 *      This file contains the device memory region creation/deletion
 *      function, and character device creation function.
 *
 *****************************************************************************/

#include "cpa.h"
#include "icp_platform.h"
#include "adf_chr_drv.h"

/*
 * adf_chr_drv_destroy
 * Removes the character device memory region
 */
void adf_chr_drv_destroy(adf_chr_drv_info_t *drv_info)
{
        int i = 0;

        if (NULL == drv_info) {
                ADF_ERROR("invalid parameter [drv_info]\n");
                return;
        }

        for (i = 0; i < drv_info->num_devices; i++) {
                device_destroy(drv_info->drv_class,
                               MKDEV(drv_info->major, i));
        }

        if (drv_info->drv_class != NULL) {
                class_destroy(drv_info->drv_class);
        }

        cdev_del(&(drv_info->drv_cdev));


        if (drv_info->major) {
                unregister_chrdev_region(MKDEV(drv_info->major,
                                          drv_info->min_minor),
                                          drv_info->max_minor);
        }
}

/*
 * adf_chr_drv_create
 * Creates character device memory region
 */
int adf_chr_drv_create(adf_chr_drv_info_t *drv_info)
{
        int ret = 0;
        dev_t devid = 0;

        if (NULL == drv_info) {
                ADF_ERROR("invalid parameter [drv_info]\n");
                return FAIL;
        }

        ret = alloc_chrdev_region(&devid,
                                  drv_info->min_minor,
                                  drv_info->max_minor,
                                  drv_info->name);
        if (ret < 0) {
                ADF_ERROR("unable to allocate chrdev region\n");
                return FAIL;
        }

        drv_info->major = MAJOR(devid);

        cdev_init(&(drv_info->drv_cdev), drv_info->file_ops);
        drv_info->drv_cdev.owner = drv_info->owner;

        ret = cdev_add(&(drv_info->drv_cdev), devid, drv_info->max_minor);
        if (ret < 0) {
                ADF_ERROR("cdev add failed\n");
                adf_chr_drv_destroy(drv_info);
                return FAIL;
        }

        drv_info->drv_class = class_create(drv_info->owner, drv_info->name);
        if (IS_ERR(drv_info->drv_class)) {
                ADF_ERROR("class_create failed for device: %s\n",
                              drv_info->name);
                adf_chr_drv_destroy(drv_info);
                return FAIL;
        }

        drv_info->num_devices = 0;

        return SUCCESS;
}

/*
 * adf_chr_drv_create_device
 * Create a character device
 */
int adf_chr_drv_create_device(adf_chr_drv_info_t *drv_info,
                              unsigned device_minor,
                              char *device_fmt)
{
        struct device *drv_device = NULL;

        if (NULL == drv_info) {
                ADF_ERROR("invalid parameter [drv_info]\n");
                return FAIL;
        }
        drv_device = device_create(drv_info->drv_class,
                                   NULL,
                                   MKDEV(drv_info->major, device_minor),
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27))
                                   NULL,
#endif
                                   device_fmt, device_minor);
        if (NULL == drv_device) {
                ADF_ERROR("failed to create device %d\n",
                              device_minor);
                return FAIL;
        }

        drv_info->num_devices++;

        return SUCCESS;
}
