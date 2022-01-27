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
 * @file adf_fw.c
 *
 * @description
 *      This file contains the ADF code request and release firmware
 *
 *****************************************************************************/
#include <linux/firmware.h>

#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_fw.h"

/*
 * adf_fw_get_location
 * Retrieves firmware structure
 */
static int adf_fw_get_location(icp_accel_dev_t *accel_dev, Cpa32S fw_type,
                                void **fw_location)
{
        ICP_CHECK_FOR_NULL_PARAM(accel_dev);

        switch(fw_type) {
        case ADF_FW_UOF_TYPE:
                *fw_location = accel_dev->pUofFirmwareLocation;
                break;
        case ADF_FW_MMP_TYPE:
                *fw_location = accel_dev->pMmpFirmwareLocation;
                break;
        default:
                ADF_ERROR("Firmware type %d not supported\n",fw_type);
                return FAIL;
        }

        return SUCCESS;
}

/*
 * adf_fw_set_location
 * Set firmware structure pointer in accel_dev structure
 */
static int adf_fw_set_location(icp_accel_dev_t *accel_dev, Cpa32S fw_type,
                                void *fw_location)
{
        ICP_CHECK_FOR_NULL_PARAM(accel_dev);
        ICP_CHECK_FOR_NULL_PARAM(fw_location);

        switch(fw_type) {
        case ADF_FW_UOF_TYPE:
                accel_dev->pUofFirmwareLocation = fw_location;
                break;
        case ADF_FW_MMP_TYPE:
                accel_dev->pMmpFirmwareLocation = fw_location;
                break;
        default:
                ADF_ERROR("Firmware type %d not supported\n",fw_type);
                return FAIL;
        }

        return SUCCESS;
}

/*
 * adf_fw_request
 * Load firmware file from disk and store it in accel_dev structure
 */
int adf_fw_request(icp_accel_dev_t *accel_dev, Cpa32S fw_type,
                        char *fw_name)
{
        int ret = 0;
        struct pci_dev *pci_device = NULL;
        const struct firmware *fw_location = NULL;

        ICP_CHECK_FOR_NULL_PARAM(accel_dev);
        ICP_CHECK_FOR_NULL_PARAM(fw_name);

        ret = adf_fw_get_location(accel_dev, fw_type, (void *)&fw_location);
        if (ret) {
                return ret;
        }

        if (NULL != fw_location) {
                ADF_ERROR("Firmware of %s already loaded.\n", fw_name);
                return FAIL;
        }

        pci_device = (struct pci_dev *)accel_dev->pciAccelDev.pDev;

        ret = request_firmware(&fw_location, fw_name, &pci_device->dev);
        if (ret) {
                ADF_ERROR("Failed to load Firmware (%s)\n", fw_name);
                return FAIL;
        }

        ret = adf_fw_set_location(accel_dev, fw_type, (void *)fw_location);

        return ret;
}

/*
 * adf_fw_release
 * Release firmware from memory and clean accel_dev structure pointers
 */
int adf_fw_release(icp_accel_dev_t *accel_dev, Cpa32S fw_type)
{
        int ret = 0;
        struct firmware *fw_location = NULL;

        ICP_CHECK_FOR_NULL_PARAM(accel_dev);

        ret = adf_fw_get_location(accel_dev, fw_type, (void *)&fw_location);
        if (ret) {
                return FAIL;
        }

        if (NULL == fw_location) {
                ADF_ERROR("No firmware loaded\n");
                return FAIL;
        }

        release_firmware(fw_location);

        return SUCCESS;
}

/*
 * adf_fw_get_info
 * Return firmware address and size
 */
int adf_fw_get_info(icp_accel_dev_t *accel_dev, Cpa32S fw_type,
                        void **addr, Cpa32U *size)
{
        int ret = 0;
        struct firmware *fw_location = NULL;

        ICP_CHECK_FOR_NULL_PARAM(accel_dev);
        ICP_CHECK_FOR_NULL_PARAM(addr);
        ICP_CHECK_FOR_NULL_PARAM(size);

        ret = adf_fw_get_location(accel_dev, fw_type, (void *)&fw_location);
        if (ret) {
                return ret;
        }

        if (NULL == fw_location) {
                ADF_ERROR("No firmware loaded\n");
                return FAIL;
        }

        *addr = (void*)(fw_location->data);
        *size = (Cpa32U)fw_location->size;

        return SUCCESS;
}
